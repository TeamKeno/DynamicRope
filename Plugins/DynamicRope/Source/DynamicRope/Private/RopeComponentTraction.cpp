// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeComponent.h"

#include "Components/SkeletalMeshComponent.h"
#include "Components/SphereComponent.h"
#include "DynamicRopeLog.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "Logic/RopeTractionSolver.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"
#include "RopeMathHelpers.h"
#include "Templates/Function.h"

#pragma region Tension_Query

float URopeComponent::GetSegmentTension(int32 SegmentIndex) const
{
	return Sim.SegmentTension.IsValidIndex(SegmentIndex) ? Sim.SegmentTension[SegmentIndex] : 0.0f;
}

float URopeComponent::GetMaxTension() const
{
	float MaxTension = 0.0f;
	for (const float T : Sim.SegmentTension)
	{
		MaxTension = FMath::Max(MaxTension, T);
	}
	return MaxTension;
}
#pragma endregion Tension_Query

#pragma region Wrapped_Pull_Sampling_And_Application

void URopeComponent::UpdateWrappedPullSample(float DeltaTime)
{
	// ② 장력 모델: 솔버가 채운 세그먼트 장력(F=λ/h², GPU 로프는 1~2프레임 지연 미러)의 최대치를
	// wrap 상태에 반영한다. 게임플레이(당김/절단 판정)와 디버거가 이 값을 읽는다.
	WrapController.State.Tension = GetMaxTension();

	// Pull 샘플 산출(항상 — 디버거/BP 관찰 + 견인/release의 공용 입력). 방향은 첫 직선 다리 추종(공간).
	PullDrive.LastPullSample = FRopePullSample();
	WrapController.ComputePull(Sim, HoldConfig.PullBendThresholdDeg, PullDrive.LastPullSample);

	// 전 체인 팽팽 게이트 갱신(히스테리시스 래치) — 견인 인가(③: 테더 + 능동 Pull)의 공용 선행 조건.
	// 앵커 인접 국소 관측치(세그먼트 장력/sub-leg overshoot)는 움직이는 대상이 슬랙 로프에서도 만들어내므로
	// (핀 노드가 이웃을 순간 스트레치 — 공중 Pierce/움직이는 정적 메시에서 늘어진 줄이 끌려가던 증상),
	// "로프 전체가 펴졌는가"를 서로 보완하는 세 관측치의 AND로 판정한다:
	//  - 처짐(다리별 내부 노드의 chord 직선 이탈 cm): "시각적으로 펴졌는가"의 정본 — chord 비율은
	//    처짐의 제곱에만 반응해 눈에 띄는 처짐(600cm 로프 chord 590 = ~45cm 처짐)도 통과시킨다.
	//  - 기하(chord 합 vs rest, 다리별 rest 클램프): 완만한 대형 처짐/압축(노드 뭉침)의 백스톱.
	//  - 최소 전달 장력(자유 구간 세그먼트 장력 최솟값): 기하가 못 보는 지그재그 구김/부분 스트레치를
	//    거른다 — 팽팽함 = 장력이 앵커에서 손까지 전 구간 전달(어딘가 슬랙이면 최솟값 0).
	// 샘플이 무효면 무조건 false(아래 early return과 무관하게 이번 프레임 값이 확정돼야 한다).
	// 세 판정 모두 직전 래치(bChainTaut)를 히스테리시스 기준으로 공유한다(처짐은 유지 시 ×ReleaseScale 완화).
	const float SagLimit = HoldConfig.TautMaxSag
		* (PullDrive.bChainTaut ? FMath::Max(HoldConfig.TautSlackReleaseScale, 1.0f) : 1.0f);
	PullDrive.bChainTaut = PullDrive.LastPullSample.bValid
		&& (HoldConfig.TautMaxSag <= 0.0f || PullDrive.LastPullSample.MaxLegSag <= SagLimit)
		&& (HoldConfig.TautSlackRatio <= 0.0f || RopeTraction::EvaluateChainTautGate(
			PullDrive.LastPullSample.TautChordLen, PullDrive.LastPullSample.FreeRestLen,
			HoldConfig.TautSlackRatio, HoldConfig.TautSlackReleaseScale, PullDrive.bChainTaut))
		&& RopeTraction::EvaluateTautGate(
			PullDrive.LastPullSample.MinFreeTension, HoldConfig.TautMinTension,
			HoldConfig.TautMinTensionReleaseRatio, PullDrive.bChainTaut);

	// 팽팽(taut) 게이트 갱신(히스테리시스 래치) — 능동 Pull 인가(③)와 IsPullTaut()가 공용으로 읽는다.
	// 전 체인 기하(bChainTaut) ∧ 장력 임계(보조 게이트 — ActivePullTautTension 0이면 "장력 > ~0").
	PullDrive.bPullTaut = PullDrive.bChainTaut && RopeTraction::EvaluateTautGate(
		PullDrive.LastPullSample.Tension, HoldConfig.ActivePullTautTension,
		HoldConfig.ActivePullTautReleaseRatio, PullDrive.bPullTaut);

	// Pull 스무딩(2단): (1) 조준 노드 fractional 스무딩 — 정수 AimNode의 프레임 간 이산 홉(방향 통째 점프
	// + tether 초과분 불연속)을 float EMA + 노드 사이 보간으로 없앤다. (2) 방향 EMA — 그 위에 남는 노드 위치
	// 노이즈(GPU 미러 지연 등)를 다듬는다. wrap 시작 후 첫 유효 프레임은 측정값으로 시드(래그 없음).
	if (!PullDrive.LastPullSample.bValid)
	{
		return;
	}

	// 스무딩 전 raw look-ahead(정수 조준) — 디버거 raw vs smoothed 비교.
	PullDrive.LastPullDirRaw = PullDrive.LastPullSample.Direction;

	// (1) 조준 인덱스 시간 스무딩 → fractional 조준 위치 보간.
	const float RawAimF = static_cast<float>(PullDrive.LastPullSample.AimNode);
	PullDrive.SmoothedAimNodeF = (PullDrive.SmoothedAimNodeF < 0.0f)
		? RawAimF // 첫 유효 프레임은 측정값으로 시드(래그 없음).
		: FMath::Lerp(PullDrive.SmoothedAimNodeF, RawAimF, RopeTraction::ExpSmoothAlpha(HoldConfig.PullAimSmoothTime, DeltaTime));
	const float AimF = FMath::Clamp(PullDrive.SmoothedAimNodeF, 0.0f, static_cast<float>(PullDrive.LastPullSample.AnchorNode));
	const FVector AimPos = RopeTraction::SampleFractionalAim(Sim.Positions, AimF, PullDrive.LastPullSample.AnchorNode);
	PullDrive.LastPullSample.AimNodeF = AimF;
	PullDrive.LastPullSample.AimPos = AimPos;

	// (2) 연속 조준으로 방향 재계산 후 방향 EMA. 축퇴(조준=앵커)면 raw 방향 유지.
	const FVector DirF = (AimPos - Sim.Positions[PullDrive.LastPullSample.AnchorNode]).GetSafeNormal();
	const FVector DirIn = DirF.IsNearlyZero() ? PullDrive.LastPullSample.Direction : DirF;
	PullDrive.SmoothedPullDir = RopeTraction::SmoothDirection(
		PullDrive.SmoothedPullDir, DirIn, RopeTraction::ExpSmoothAlpha(HoldConfig.PullDirSmoothTime, DeltaTime));
	PullDrive.LastPullSample.Direction = PullDrive.SmoothedPullDir;

	// (3) 앵커 월드 속도 추정(WorldPoint 프레임 차분 → EMA) — wielder 견인 피드포워드의 관측치.
	// 리엘(overshoot P 제어)은 오차만 닫으므로 순항 중인 앵커(비행 몬스터 등)는 영영 못 따라잡는다 —
	// 이 속도의 로프 축 성분을 견인 목표에 더해 따라잡음은 피드포워드가, 오차 수렴은 리엘이 맡는다.
	// 정지 앵커는 0이라 동작 불변. 첫 유효 프레임은 prev만 채워 0에서 램프업(wrap 직후 홱 당김 방지).
	// 노드 위치 지터는 방향 EMA와 같은 상수(PullDirSmoothTime)로 다듬는다.
	const FVector AnchorPoint = PullDrive.LastPullSample.WorldPoint;
	if (PullDrive.bPrevAnchorPointValid && DeltaTime > 1e-4f)
	{
		const FVector RawAnchorVel = (AnchorPoint - PullDrive.PrevAnchorPoint) / DeltaTime;
		PullDrive.SmoothedAnchorVelocity = FMath::Lerp(PullDrive.SmoothedAnchorVelocity, RawAnchorVel,
			RopeTraction::ExpSmoothAlpha(HoldConfig.PullDirSmoothTime, DeltaTime));
	}
	PullDrive.PrevAnchorPoint = AnchorPoint;
	PullDrive.bPrevAnchorPointValid = true;
}

void URopeComponent::ApplyWrappedTraction(float DeltaTime)
{
	// 이 함수의 동기 호출 구간에서만 endpoint 캐시를 유지한다. virtual ApplyPullForce가 Super를 호출해도
	// 같은 target 해석을 재사용하고, 외부에서 별도로 호출한 ApplyPullForce에는 캐시가 새지 않는다.
	WrappedEndpointCache.Reset();
	// 끌림 가능 판정: 테더 분배 관측(LastTargetShare)과 능동 Pull의 climb-in 방향이 공유한다.
	if (PullDrive.LastPullSample.bValid)
	{
		UpdateTargetPullable();
	}

	// ③-1 자동 견인(테더): 단일 λ 임펄스 제약 + 랙돌 물리 제약(Docs/PoC/05). 슬랙 브레이크는 없다 —
	// λ의 위치 회수 항은 MaxBiasSpeed로 유계라 장부에 남을 과잉 주입 자체가 없다(레거시 서보 시절 유물).
	UpdateConstraintTether(DeltaTime);

	// ③-2 능동 Pull(상수 힘): 사용자 입력(SetActivePull/Wielder)이 준 힘을 팽팽할 때만 인가한다.
	// 팽팽 판정은 ②가 갱신한 bPullTaut 래치 = 전 체인 기하(bChainTaut) ∧ 장력 임계
	// (임계/히스테리시스는 HoldConfig — 기본 임계 0 = 장력 > ~0).
	// 팽팽함 무시 2층: config(bActivePullRequiresTaut=false, 로프 전체 정책) / per-call(bActivePullIgnoresTaut,
	// SetActivePull 인자 — 애니 pull window 구간용). 어느 쪽이든 Wrapped + 유효 샘플이면 인가.
	// 장력과 무관한 상수라 피드백 폭주가 없다.
	const bool bActivePullPassesGate = PullDrive.ActivePullForce > 0.0f && PullDrive.LastPullSample.bValid
		&& (!HoldConfig.bActivePullRequiresTaut || PullDrive.bActivePullIgnoresTaut || PullDrive.bPullTaut);
#if WITH_GAMEPLAY_DEBUGGER
	// 디버거는 요청값(ActivePullForce)만으로는 실제 인가를 알 수 없다 — 팽팽 게이트와 우회 2층이 여기서
	// 갈리기 때문이다. 게이트 통과 여부를 그대로 남긴다(수신자 단계에서 힘이 버려지는 경우는 별개다).
	DebugActivePullPassedGate = bActivePullPassesGate;
#endif
	if (bActivePullPassesGate)
	{
		// 대상이 무거워 끌 수 없으면(not pullable) 힘을 wielder에 실어 앵커 쪽으로 끌어당긴다(climb-in):
		// LastPullSample.Direction은 앵커→손 방향이라 부호 반전 = 손→앵커 — "내가 끌려가야 되면 간다"
		// (벽/무거운 랙돌/드래곤에 pull = 입체기동). 끌림 가능하면 대상에 인가해 wielder 쪽으로 끈다.
		if (!PullDrive.bTargetPullable)
		{
			ApplyPullForceToWielder(-PullDrive.LastPullSample.Direction * PullDrive.ActivePullForce, DeltaTime);
		}
		else
		{
			ApplyPullForce(PullDrive.LastPullSample.Direction * PullDrive.ActivePullForce, PullDrive.LastPullSample, DeltaTime);
		}
	}
	WrappedEndpointCache.Reset();
}

#pragma endregion Wrapped_Pull_Sampling_And_Application

#pragma region Active_Pull_API

void URopeComponent::SetActivePull(float Force, bool bIgnoreTautGate)
{
	PullDrive.ActivePullForce = FMath::Max(0.0f, Force);
	PullDrive.bActivePullIgnoresTaut = bIgnoreTautGate;
}
#pragma endregion Active_Pull_API

#pragma region Traction_Endpoint_And_Tether_Policies

namespace
{
	// 본에서 부모 체인을 올라가 가장 가까운 "시뮬 중인 피직스 바디"의 본을 찾는다(없으면 None).
	// 감긴 본이 트위스트 본 등 피직스 에셋에 바디가 없는 본일 수 있다 — 그 경우 본 이름만 보고
	// 비시뮬 판정해 캐릭터 무브먼트 분기로 빠지면, 랙돌 셋업이 무브먼트를 꺼둔 상태(MOVE_None)라
	// AddForce가 조용히 버려진다. 힘/속도 인가 본은 이 함수로 승격해 찾는다.
	FName FindNearestSimulatingBone(const USkeletalMeshComponent* Mesh, FName Bone)
	{
		while (!Bone.IsNone())
		{
			if (Mesh->IsSimulatingPhysics(Bone))
			{
				return Bone;
			}
			Bone = Mesh->GetParentBone(Bone);
		}
		return NAME_None;
	}

	// 시뮬 본 위(부모 체인)에 키네마틱 바디가 있는가 = 부분 랙돌 판정. 있으면 그 구속이 본 견인을 통째로
	// 흡수해(무한질량 벽) 본에 인가한 서보/힘이 액터에 전달되지 않는다 — 이 경우 수신자 해석은 본이 아니라
	// 이동체(캐릭터)로 내려가야 한다. 없으면(루트 바디까지 전부 시뮬) 관절로 몸 전체가 끌려오는 자유 랙돌.
	// 바디 없는 본(트위스트/IK)은 구속이 아니므로 건너뛴다.
	bool IsSimBoneBoundToKinematic(const USkeletalMeshComponent* Mesh, FName SimBone)
	{
		for (FName Bone = Mesh->GetParentBone(SimBone); !Bone.IsNone(); Bone = Mesh->GetParentBone(Bone))
		{
			// 바디 존재 + 비시뮬 = 키네마틱 구속. (시뮬 상태는 컴포넌트 API로 묻는다 —
			// FBodyInstance::IsInstanceSimulatingPhysics는 비export 인라인이라 링크 불가.)
			if (Mesh->GetBodyInstance(Bone) != nullptr && !Mesh->IsSimulatingPhysics(Bone))
			{
				return true;
			}
		}
		return false;
	}

	// 캐릭터 무브먼트가 지금 힘을 소비할 수 있는가. MOVE_None(DisableMovement — 랙돌 셋업 관례)이면
	// AddForce가 누적만 되고 소비되지 않아 "성공한 척" 힘이 사라진다 — 그 경우 다른 수신자로 넘긴다.
	// wrap 대상 액터를 직접 받는다(대상이 스켈레탈/정적/물리프랍 무엇이든 무관 — 소유 액터 기준 판정).
	UCharacterMovementComponent* GetForceConsumingMovement(const AActor* Owner)
	{
		const ACharacter* Character = Cast<ACharacter>(Owner);
		UCharacterMovementComponent* Movement = Character ? Character->GetCharacterMovement() : nullptr;
		return (Movement && Movement->MovementMode != MOVE_None) ? Movement : nullptr;
	}

	// 인가 대상 바디의 질량(kg): 스켈레탈 본이면 그 바디 질량, 아니면(또는 바디가 없거나 질량이 0이면)
	// 컴포넌트 질량. 물리 바디 질량은 UE가 콜리전 볼륨×밀도로 자동 유지하는 값이라 별도 세팅이 필요 없다.
	// (UObject 의존이라 RopeTraction 순수 수학에 넣지 않는다 — 질량은 여기서 읽어 그쪽에 넘긴다.)
	float ResolveBodyMass(const UPrimitiveComponent* Prim, FName BoneName)
	{
		float Mass = 0.0f;
		if (!BoneName.IsNone())
		{
			if (const USkeletalMeshComponent* Skel = Cast<const USkeletalMeshComponent>(Prim))
			{
				if (const FBodyInstance* Body = Skel->GetBodyInstance(BoneName))
				{
					Mass = static_cast<float>(Body->GetBodyMass());
				}
			}
		}
		return (Mass > KINDA_SMALL_NUMBER) ? Mass : static_cast<float>(Prim->GetMass());
	}

	// ===== 테더 엔드포인트(수신자) 해석 — 래더를 한 번만 건넌다 =====
	// "무엇이 받는가"는 여기서 한 번만 판정해 종류·인가점·유효질량을 함께 확정한다. 예전엔 분배용 질량과 실제
	// 인가 지점이 같은 래더를 각자 복제했고, 그 위에서 인가 람다가 자기가 어느 rung인지 다시 캐스팅으로
	// 역추론했다 — 순서가 어긋나면 "질량은 앵커로 봤는데 힘은 다른 데 꽂히는" 버그가 된다(CL 392의 부분 랙돌
	// 루트 게이트가 실제로 그랬다). 한 해석을 공유하면 그 어긋남이 구조적으로 불가능하다.
	// ERopeEndpointKind/FRopeTetherEndpoint는 Core/RopeTypes.h의 공용 판정 타입이다. 컴포넌트는
	// target/wielder 결과를 같은 Wrapped 프레임 안에서 캐시해 pullable/테더/기본 Pull이 공유한다.

	// 수신자 해석(대상/wielder 공용). 순서: 스켈레탈 자유 랙돌 본 → 시뮬 프리미티브 → 시뮬 루트 → 캐릭터 → 앵커.
	// (부분 랙돌 — 시뮬 본이 위쪽 키네마틱 바디에 묶임 — 은 rung 1이 받지 않고 캐릭터/앵커로 폴스루한다.)
	//  - MeshComp: 대상이면 State.Mesh, wielder면 nullptr(스켈레탈·프리미티브 rung 자동 skip → 루트부터).
	//  - 물리 바디 질량은 UE가 콜리전 볼륨×밀도로 자동 유지하는 값이라 별도 세팅이 필요 없다.
	//  - 캐릭터 접지는 유한 브레이스(Mass×GroundBraceFactor — 발 디딤 저항), 공중은 Mass, MOVE_None은 앵커.
	FRopeTetherEndpoint ResolveTetherEndpoint(USceneComponent* MeshComp, AActor* Owner, FName WrappedBone, float GroundBraceFactor)
	{
		FRopeTetherEndpoint Out;
		Out.Actor = Owner;

		// (1) 스켈레탈 **자유 랙돌**(시뮬 본이 루트 바디까지 관절로만 이어짐): 감긴 본에서 부모 체인으로 승격한
		// 가장 가까운 *시뮬 본*(바디 없는 트위스트 본 대응)에 인가한다. 유효 질량은 본 바디가 아니라 **전신 바디
		// 질량 합**(GetMass) — 본 하나를 당겨도 관절로 끌려오는 것은 몸 전체라, 본 바디 질량(팔뚝 3kg)을 쓰면
		// 몫/끌림 판정이 "가벼운 대상"으로 오판해 물리적으로 낼 수 없는 회수를 전량
		// 배정받고 로프만 늘어난다(탄성 끌림 증상 — Docs/PoC/05 §3.4).
		//
		// **부분 랙돌**(시뮬 본 위 부모 체인에 키네마틱 바디 존재)은 여기서 받지 않고 아래로 폴스루한다: 본을
		// 아무리 서보해도 키네마틱 구속이 흡수해 로프만 늘어난다(2026-07-15 보류했던 rung 1 한계 — 이 폴스루가
		// 그 해소다). 실제로 끌 수 있는 것은 이동체(캐릭터 rung — CMC 활성)거나, 그마저 없으면 아무것도 없다
		// (앵커 = 무한질량이 물리적 진실). 감긴 본의 시각 반응(팔이 딸려오는 연출)은 후속(Docs/PoC/05 §7).
		if (USkeletalMeshComponent* Skel = Cast<USkeletalMeshComponent>(MeshComp))
		{
			const FName SimBone = FindNearestSimulatingBone(Skel, WrappedBone);
			if (!SimBone.IsNone() && !IsSimBoneBoundToKinematic(Skel, SimBone))
			{
				Out.Kind = ERopeEndpointKind::SimBody;
				Out.Prim = Skel;
				Out.Bone = SimBone;
				// 전신 질량(전 바디 합). 축퇴(바디 미생성 등으로 0)면 종전 본 바디 → 컴포넌트 질량 폴백.
				const float WholeMass = static_cast<float>(Skel->GetMass());
				Out.Mass = (WholeMass > KINDA_SMALL_NUMBER) ? WholeMass : ResolveBodyMass(Skel, SimBone);
				return Out;
			}
		}
		// (2) 대상 컴포넌트 자체가 시뮬 중인 프리미티브(가벼운 물리 프랍 등).
		if (UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(MeshComp))
		{
			if (Prim->IsSimulatingPhysics())
			{
				Out.Kind = ERopeEndpointKind::SimBody;
				Out.Prim = Prim;
				Out.Mass = ResolveBodyMass(Prim, NAME_None);
				return Out;
			}
		}
		if (!Owner)
		{
			return Out; // None.
		}
		// (3) 소유 액터 루트 프리미티브가 시뮬 중(물리 액터 구성). wielder는 MeshComp=nullptr라 여기서 시작.
		if (UPrimitiveComponent* Root = Cast<UPrimitiveComponent>(Owner->GetRootComponent()))
		{
			if (Root->IsSimulatingPhysics())
			{
				Out.Kind = ERopeEndpointKind::SimBody;
				Out.Prim = Root;
				Out.Mass = ResolveBodyMass(Root, NAME_None);
				return Out;
			}
		}
		// (4) 캐릭터: MOVE_None(랙돌 셋업 관례)이면 무브먼트가 힘을 소비하지 않으니 앵커로 본다.
		if (const ACharacter* Character = Cast<ACharacter>(Owner))
		{
			if (UCharacterMovementComponent* Movement = Character->GetCharacterMovement())
			{
				if (Movement->MovementMode != MOVE_None)
				{
					Out.Kind = ERopeEndpointKind::Character;
					Out.Movement = Movement;
					const float BraceScale = Movement->IsMovingOnGround() ? FMath::Max(GroundBraceFactor, 1.0f) : 1.0f;
					Out.Mass = Movement->Mass * BraceScale;
					return Out;
				}
			}
		}
		// (5) 정적/키네마틱/MOVE_None/비시뮬 비캐릭터 → 앵커(질량 0). 위치 폴백만 가능하다.
		Out.Kind = ERopeEndpointKind::Anchor;
		return Out;
	}

	// 해석된 수신자를 공개 확장 훅(ApplyTractionToReceiver)이 읽는 요청으로 옮긴다.
	// Source/Direction/Amount는 인가 경로가 각자 채운다(단위가 경로마다 다르다 — 요청 타입 주석 참고).
	FRopeTractionRequest MakeTractionRequest(const FRopeTetherEndpoint& Endpoint, ERopeTractionSource Source,
		const FVector& Dir, float Amount, float DeltaTime, bool bWielderSide)
	{
		FRopeTractionRequest Req;
		Req.Source = Source;
		Req.ReceiverKind = Endpoint.Kind;
		Req.Prim = Endpoint.Prim;
		Req.Bone = Endpoint.Bone;
		Req.Movement = Endpoint.Movement;
		Req.Actor = Endpoint.Actor;
		Req.Direction = Dir;
		Req.Amount = Amount;
		Req.DeltaTime = DeltaTime;
		Req.bWielderSide = bWielderSide;
		return Req;
	}

	// 테더 자동 분배용 유효 역질량(w = 1/유효질량). 0 = 앵커(무한질량).
	float EndpointInvMass(const FRopeTetherEndpoint& Endpoint)
	{
		return RopeTraction::InvMassFromMass(Endpoint.Mass);
	}


	// 테더 인가 공용 컨텍스트 — 양끝 수신자 해석 + 확장 관문. 관측/λ 산출·상태 보관은 컴포넌트가 한다.
	struct FRopeTetherContext
	{
		const FRopeTetherEndpoint& Target;
		const FRopeTetherEndpoint& Wielder;
		float DeltaTime;
		// 수신자 인가 확장 관문(컴포넌트의 virtual로 위임). true면 내장 인가를 건너뛴다.
		TFunctionRef<bool(const FRopeTractionRequest&)> TractionGate;
	};

	// 해석된 수신자에 인가를 디스패치(대상/wielder 공용 골격 — Constraint 테더의 두 인가 지점이 쓴다).
	// 콜백은 자기가 무엇을 받았는지 이미 알고 있다(재캐스팅 불필요). Step/반환은 이번 프레임 축 ΔV(cm/s).
	//  - SimApply(Prim, Bone, Dir, DeltaV): 물리 시뮬 바디.
	//  - CharacterApply(Movement, Dir, DeltaV): CMC 구동 캐릭터.
	//  - AnchorApply(Actor, Dir, DeltaV): 앵커(정적/MOVE_None) — 무동작(w=0이라 ΔV도 0).
	float ApplyToTetherEndpoint(
		const FRopeTetherContext& Ctx, bool bWielderSide, const FVector& Dir, float Step,
		TFunctionRef<float(UPrimitiveComponent*, FName, const FVector&, float)> SimApply,
		TFunctionRef<void(UCharacterMovementComponent*, const FVector&, float)> CharacterApply,
		TFunctionRef<void(AActor*, const FVector&, float)> AnchorApply)
	{
		const FRopeTetherEndpoint& Endpoint = bWielderSide ? Ctx.Wielder : Ctx.Target;

		// 확장 관문(URopeComponent::ApplyTractionToReceiver): 서브클래스가 처리했으면 내장 인가를 건너뛴다.
		// 테더의 네 인가 지점이 전부 이 골격을 지나므로, 여기 한 줄이 테더 경로 전체를 덮는다.
		// 반환은 Step(= 부족분 0) — sim-body 미적용 분기와 같은 계약이다.
		if (Ctx.TractionGate(MakeTractionRequest(Endpoint, ERopeTractionSource::Tether, Dir, Step,
			Ctx.DeltaTime, bWielderSide)))
		{
			return Step;
		}

		switch (Endpoint.Kind)
		{
		case ERopeEndpointKind::SimBody:
			return SimApply(Endpoint.Prim, Endpoint.Bone, Dir, Step);
		case ERopeEndpointKind::Character:
			CharacterApply(Endpoint.Movement, Dir, Step);
			break;
		case ERopeEndpointKind::Anchor:
			AnchorApply(Endpoint.Actor, Dir, Step);
			break;
		default:
			break;
		}
		return Step; // sim-body 미적용 → 부족분 0.
	}

}

const FRopeResolvedWrappedEndpoints* URopeComponent::GetOrResolveWrappedEndpoints()
{
	if (WrappedEndpointCache.bValid)
	{
		return &WrappedEndpointCache;
	}

	USceneComponent* MeshComp = const_cast<USceneComponent*>(WrapController.State.Mesh.Get());
	if (!MeshComp || !PullDrive.LastPullSample.bValid)
	{
		return nullptr;
	}

	WrappedEndpointCache.Target = ResolveTetherEndpoint(
		MeshComp, MeshComp->GetOwner(), PullDrive.LastPullSample.Bone, HoldConfig.GroundBraceFactor);
	WrappedEndpointCache.Wielder = ResolveTetherEndpoint(
		nullptr, GetOwner(), NAME_None, HoldConfig.GroundBraceFactor);
	WrappedEndpointCache.TargetMesh = MeshComp;
	WrappedEndpointCache.TargetBone = PullDrive.LastPullSample.Bone;
	WrappedEndpointCache.bValid = true;
	return &WrappedEndpointCache;
}

#pragma endregion Traction_Endpoint_And_Tether_Policies

#pragma region Tether_And_Pull_Application

FVector URopeComponent::ComputeSmoothedWielderDir(const FVector& Aim, const FVector& DirToAim, float DeltaTime)
{
	// 방향 = 손(노드 0)에서 로프의 첫 직선 다리를 따라. 조준(AimPos)이 벽 모서리면 모서리를 향하고, 로프가 곧아
	// 조준=손이면(chord ~0) 앵커→조준의 역방향(=손→앵커)으로 폴백한다.
	const FVector HandPos = Sim.Positions.IsValidIndex(0) ? Sim.Positions[0] : Aim;
	FVector WielderDirRaw = Aim - HandPos;
	if (!WielderDirRaw.Normalize(KINDA_SMALL_NUMBER))
	{
		WielderDirRaw = -DirToAim;
	}
	// 방향 EMA(대상 쪽 SmoothedPullDir과 동일 상수·동일 함수): AimPos 노드 노이즈/모서리 전환/근접 축퇴로 raw
	// 방향이 프레임마다 튀면 클램프/톱업이 매번 다른 축으로 들어가 벡터가 랜덤워크로 불어난다(폭주).
	PullDrive.SmoothedWielderPullDir = RopeTraction::SmoothDirection(
		PullDrive.SmoothedWielderPullDir, WielderDirRaw, RopeTraction::ExpSmoothAlpha(HoldConfig.PullDirSmoothTime, DeltaTime));
	return PullDrive.SmoothedWielderPullDir;
}

void URopeComponent::UpdateConstraintTether(float DeltaTime)
{
	// (Constraint 모드 — Docs/PoC/05) 관측(C·s·d·w) → λ 솔브(RopeTraction::SolveTetherLambda, 유닛 테스트
	// 대상) → 크기가 같은 임펄스 쌍 인가. 상시 리엘이 없고(능동 견인 = 되감기의 rest 변화율이 s에 실림),
	// 발화는 C > 0 ∧ 전 체인 팽팽(bChainTaut — 아래 게이트 주석)일 때만이다.
	PullDrive.LastTetherOvershoot = 0.0f;
	PullDrive.LastTetherLambda = 0.0f;
	PullDrive.LastTetherLambdaDt = DeltaTime;
	if (!PullDrive.LastPullSample.bValid || DeltaTime <= 1e-4f)
	{
		PullDrive.bPrevFreeRestValid = false; // 관측 공백 — 다음 유효 프레임에 rest 차분 재시드.
		return;
	}

	// 제약 위반 C = 실제 경로 길이(비클램프 chord 합) − (자유 구간 rest + 여유). 슬랙/구김/처짐은 chord가
	// rest보다 짧아 C < 0 → 무동작. 스냅샷/거리 release가 읽는 overshoot는 C의 0 클램프다(의미 동일).
	const float FreeRest = PullDrive.LastPullSample.FreeRestLen + FMath::Max(HoldConfig.TetherSlack, 0.0f);
	float C = PullDrive.LastPullSample.PathChordLen - FreeRest;

	// rest 변화율(되감기/SetRopeLength → 감김 = rest 감소 = 벌어짐 취급): 앵커 노드가 같은 프레임 간
	// 차분만 신뢰한다 — 앵커가 옮겨간 프레임의 rest는 불연속이라 속도가 아니다.
	float RestRate = 0.0f;
	if (PullDrive.bPrevFreeRestValid && PullDrive.PrevAnchorNode == PullDrive.LastPullSample.AnchorNode)
	{
		RestRate = (FreeRest - PullDrive.PrevFreeRestLen) / DeltaTime;
	}
	PullDrive.PrevFreeRestLen = FreeRest;
	PullDrive.PrevAnchorNode = PullDrive.LastPullSample.AnchorNode;
	PullDrive.bPrevFreeRestValid = true;

	// 수신자 해석은 C 안정화(아래)가 본 위치를 필요로 해 게이트보다 먼저 한다(프레임 캐시라 추가 비용 없음).
	const FRopeResolvedWrappedEndpoints* Endpoints = GetOrResolveWrappedEndpoints();
	if (!Endpoints)
	{
		PullDrive.LastTetherOvershoot = FMath::Max(0.0f, C);
		return;
	}
	USceneComponent* MeshComp = Endpoints->TargetMesh.Get();
	if (!MeshComp)
	{
		PullDrive.LastTetherOvershoot = FMath::Max(0.0f, C);
		return;
	}

	// 스켈레탈(랙돌) 대상 식별. GT 프레임당 임펄스는 관절체에서 "전신 크기 kick → 관절 솔버 지연 반응 →
	// 폭주"와 "본 크기 λ → 견인력 붕괴" 사이 딜레마가 있어(2026-07-20 Pierce 7회 반복 실측), 랙돌 쪽 절반은
	// **엔진 물리 제약**(UpdatePhysicalTether — Chaos가 서브스텝에서 관절·접촉과 함께 솔브)에 맡긴다.
	// λ 솔브에서 대상 끝은 앵커(w=0) 취급 — wielder 몫만 λ가 담당하고, 랙돌이 잘 따라오면 C가 안 쌓여
	// wielder도 자유, 랙돌이 걸리면 C가 쌓여 wielder가 로프 끝에 잡힌다(분업이 자연 유도).
	const bool bSkeletalKind = (Endpoints->Target.Kind == ERopeEndpointKind::SimBody
		&& !Endpoints->Target.Bone.IsNone());
	USkeletalMeshComponent* TargetSkel = bSkeletalKind
		? Cast<USkeletalMeshComponent>(Endpoints->Target.Prim) : nullptr;
	const bool bSkeletalTarget = (TargetSkel != nullptr);
	PullDrive.LastTetherOvershoot = FMath::Max(0.0f, C);

	// 물리 제약 테더 갱신은 발화 게이트보다 **앞**이다: 프록시(코너 추종)와 리밋(되감기 반영)은 슬랙에서도
	// 따라가야 하고, 슬랙이면 리밋이 안 걸려 힘이 0인 것이 곧 물리적 무동작이다(별도 게이트 불필요).
	const FVector Anchor = PullDrive.LastPullSample.WorldPoint;
	const FVector Aim = PullDrive.LastPullSample.AimPos;
	if (bSkeletalTarget)
	{
		const float LegRest = FMath::Max(0.0f,
			static_cast<float>(PullDrive.LastPullSample.AnchorNode) - PullDrive.LastPullSample.AimNodeF)
			* Sim.SegmentLength + FMath::Max(HoldConfig.TetherSlack, 0.0f);
		UpdatePhysicalTether(TargetSkel, Endpoints->Target.Bone, Anchor, Aim, LegRest, DeltaTime);
	}
	else
	{
		TeardownPhysicalTether(); // 대상이 스켈레탈에서 벗어남(소실/재해석) — 제약 정리.
	}

	// 발화 게이트 = C > 0 ∧ 전 체인 팽팽(bChainTaut — ②가 갱신하는 3중 게이트 래치). C만으로는 안 된다는
	// 것이 랙돌 PIE의 교훈(2026-07-20): C의 소스(비클램프 chord 합)는 **부분 스트레치에 오염**된다 — 랙돌
	// 본이 요동치면 앵커 인접 다리만 strain limit(1.5×)까지 늘어나, 나머지가 늘어져 있어도 합이 rest를 넘어
	// C > 0으로 읽힌다. 그 가짜 C에 λ가 상한까지 발화 → 쌍 임펄스로 랙돌·wielder 동시 견인 → 요동 가속 →
	// 더 큰 스트레치의 정귀환(슬랙 로프인데 T가 상한 클램프로 빨강). 클램프 chord 비율·최소 전달 장력·처짐
	// 3중 게이트(레거시가 같은 이유로 3차 보강해 얻은 판정 — CL 466→470)가 "전체가 펴졌는가"의 정본이다.
	if (C <= 0.0f || !PullDrive.bChainTaut)
	{
		return; // 슬랙(또는 부분 스트레치의 가짜 C) — λ 없음.
	}

	// 끝 방향(안쪽 = 상대 쪽): 대상 = 앵커→첫 다리(스무딩된 look-ahead), wielder = 손→첫 다리(EMA —
	// 이 프레임은 인가 후보라 무조건 진행시킨다). 축퇴 폴백은 앵커→조준 span 직선.
	const FVector Span = (Aim - Anchor).GetSafeNormal();
	const FVector DirTarget = PullDrive.SmoothedPullDir.IsNearlyZero() ? Span : PullDrive.SmoothedPullDir;
	if (DirTarget.IsNearlyZero())
	{
		return; // 축퇴(조준=앵커) — 방향 정의 불가.
	}
	const FVector DirWielder = ComputeSmoothedWielderDir(Aim, DirTarget, DeltaTime);
	if (DirWielder.IsNearlyZero())
	{
		return;
	}

	// 자기 랩(owner == 대상): 양끝이 같은 몸이라 쌍 인가가 자가 상쇄된다 — wielder 끝을 앵커(w=0)로 취급해
	// 대상 끝만 움직인다(레거시 특례와 동일).
	const bool bSelfWrap = (GetOwner() != nullptr && MeshComp->GetOwner() == GetOwner());

	// 벌어짐 속도 s = −(vT·dT + vW·dW) − dRest/dt. 끝 속도는 수신자 해석과 같은 rung에서 실측한다 —
	// 움직이는 앵커(드래곤)의 순항은 vT에 실려 별도 피드포워드 없이 추종된다(레거시의 앵커 속도 EMA 대체).
	auto EndpointVelocity = [](const FRopeTetherEndpoint& Endpoint) -> FVector
	{
		switch (Endpoint.Kind)
		{
		case ERopeEndpointKind::SimBody:
			return Endpoint.Prim ? Endpoint.Prim->GetPhysicsLinearVelocity(Endpoint.Bone) : FVector::ZeroVector;
		case ERopeEndpointKind::Character:
			return Endpoint.Movement ? Endpoint.Movement->Velocity : FVector::ZeroVector;
		default:
			return FVector::ZeroVector; // 앵커/None — 정지.
		}
	};
	// 스켈레탈(랙돌) 대상은 물리 제약이 담당하므로 λ 관점에서 앵커(정지·w=0) 취급 — s 기여 0.
	const FVector VelTarget = bSkeletalTarget ? FVector::ZeroVector : EndpointVelocity(Endpoints->Target);
	const float SepTarget = -static_cast<float>(FVector::DotProduct(VelTarget, DirTarget));
	const float SepWielder = bSelfWrap ? 0.0f
		: -static_cast<float>(FVector::DotProduct(EndpointVelocity(Endpoints->Wielder), DirWielder));
	const float SepSpeed = SepTarget + SepWielder - RestRate;

	RopeTraction::FRopeTetherConstraint In;
	In.C = C;
	In.SepSpeed = SepSpeed;
	// 스켈레탈(랙돌) 대상은 물리 제약이 담당 — λ에서는 앵커(w=0). 그 외는 종전(유효질량의 역).
	In.InvMassTarget = bSkeletalTarget ? 0.0f : EndpointInvMass(Endpoints->Target);
	In.InvMassWielder = bSelfWrap ? 0.0f : EndpointInvMass(Endpoints->Wielder);
	In.SettleAlpha = RopeTraction::ExpSmoothAlpha(HoldConfig.TetherSettleTime, DeltaTime);
	In.MaxBiasSpeed = FMath::Max(HoldConfig.TetherMaxSpeed, 0.0f);
	In.Compliance = HoldConfig.TetherCompliance;
	In.MaxTension = HoldConfig.MaxTetherTension;
	const float Lambda = RopeTraction::SolveTetherLambda(In, DeltaTime);
	PullDrive.LastTetherLambda = Lambda;
	// 유효 분배 몫(wielder 게이트/디버거 호환) = 역질량비 — λ 발화와 무관하게 이번 프레임 값으로 확정한다.
	const float WSum = In.InvMassTarget + In.InvMassWielder;
	PullDrive.LastTargetShare = (WSum > KINDA_SMALL_NUMBER) ? (In.InvMassTarget / WSum) : 0.0f;
	if (Lambda <= 0.0f)
	{
		return; // 이미 충분히 접근 중이거나 양끝 다 앵커.
	}

	// ---- 인가: 끝별 ΔV = λ × w(cm/s), 각자 다리 방향 ----
	// 모든 경로가 로프 축(+직교 감쇠) 성분만 건드려 스윙/중력은 보존되고, 결과 속력은 TetherMaxSpeed로
	// 2차 클램프된다(자유 랙돌 add 경로 제외 — ΔV 자체가 λ 상한으로 유계). 기존 디스패치 골격
	// (ApplyToTetherEndpoint)을 그대로 지나므로 확장 관문(ApplyTractionToReceiver, Amount = ΔV cm/s)도 동일.
	const float SpeedCap = FMath::Max(HoldConfig.TetherMaxSpeed, 0.0f);
	const float PerpDamp = FMath::Clamp(HoldConfig.TetherPerpDamping, 0.0f, 1.0f);
	auto ApplySimBody = [&](UPrimitiveComponent* Prim, FName BoneName, const FVector& Dir, float DeltaV) -> float
	{
		// (스켈레탈 본 endpoint는 여기 오지 않는다 — 물리 제약이 담당, w=0이라 ΔV도 0.)
		// 컴포넌트 단위 시뮬 바디: 임펄스(bVelChange) + 직교 잔여 관성 부분 감쇠 + 결과 속력 클램프.
		const FVector CurVel = Prim->GetPhysicsLinearVelocity(BoneName);
		FVector Impulse = Dir * DeltaV;
		if (PerpDamp > 0.0f)
		{
			const FVector PerpVel = CurVel - Dir * static_cast<float>(FVector::DotProduct(CurVel, Dir));
			Impulse -= PerpVel * PerpDamp;
		}
		const FVector NewVel = RopeTraction::ClampInjectedVelocity(CurVel + Impulse, CurVel, SpeedCap);
		Prim->AddImpulse(NewVel - CurVel, BoneName, /*bVelChange*/ true);
		return DeltaV;
	};
	auto ApplyCharacter = [&](UCharacterMovementComponent* Movement, const FVector& Dir, float DeltaV)
	{
		// CMC: 속도 직접 가산(이번 프레임 반영 계약). λ의 위치 회수 항은 MaxBiasSpeed로 유계라
		// 과잉 주입이 없고, 별도 장부/슬랙 브레이크도 필요 없다.
		const FVector OldVel = Movement->Velocity;
		Movement->Velocity = RopeTraction::ClampInjectedVelocity(OldVel + Dir * DeltaV, OldVel, SpeedCap);
	};

	auto TractionGate = [this](const FRopeTractionRequest& Req) { return ApplyTractionToReceiver(Req); };
	const FRopeTetherContext Ctx{ Endpoints->Target, Endpoints->Wielder, DeltaTime, TractionGate };

	const float DvTarget = Lambda * In.InvMassTarget;
	if (DvTarget > KINDA_SMALL_NUMBER)
	{
		ApplyToTetherEndpoint(Ctx, /*bWielderSide*/ false, DirTarget, DvTarget,
			[&](UPrimitiveComponent* P, FName B, const FVector& D, float S) { return ApplySimBody(P, B, D, S); },
			[&](UCharacterMovementComponent* M, const FVector& D, float S) { ApplyCharacter(M, D, S); },
			[&](AActor*, const FVector&, float) { /* 앵커 = w 0이라 ΔV도 0 — 도달 불가 */ });
	}
	const float DvWielder = bSelfWrap ? 0.0f : Lambda * In.InvMassWielder;
	if (DvWielder > KINDA_SMALL_NUMBER)
	{
		ApplyToTetherEndpoint(Ctx, /*bWielderSide*/ true, DirWielder, DvWielder,
			[&](UPrimitiveComponent* P, FName B, const FVector& D, float S) { return ApplySimBody(P, B, D, S); },
			[&](UCharacterMovementComponent* M, const FVector& D, float S) { ApplyCharacter(M, D, S); },
			[&](AActor*, const FVector&, float) { /* 앵커 무동작 */ });
	}
}

void URopeComponent::UpdatePhysicalTether(USkeletalMeshComponent* TargetSkel, FName Bone,
	const FVector& AnchorWorld, const FVector& CornerWorld, float LegRestLen, float DeltaTime)
{
	// (Constraint 테더 — 랙돌 대상 절반) 엔진 물리 제약: [코너의 키네마틱 프록시 ↔ 감긴 본의 앵커 점]을
	// 다리 rest 길이의 구면 리밋으로 묶는다. GT 프레임당 임펄스는 관절체에서 "전신 크기 kick → 폭주" vs
	// "본 크기 λ → 견인력 붕괴" 딜레마가 있었지만(2026-07-20 Pierce 7회 반복), Chaos 제약은 서브스텝에서
	// 관절·지면 접촉과 **함께** 풀리므로 폭주 없이 전신 견인이 나온다("랙돌을 손에 매달기"의 표준 패턴).
	// 본-쪽 제약 프레임을 앵커의 본-로컬(창 끝 레버)로 잡아 정렬 토크까지 엔진이 정확히 푼다.
	AActor* Owner = GetOwner();
	if (!Owner || !TargetSkel)
	{
		return;
	}

	// 대상/본이 바뀌었으면 재생성(앵커 승격/재랩).
	if (PhysicalTetherConstraint
		&& (PhysicalTetherTarget.Get() != TargetSkel || PhysicalTetherBone != Bone))
	{
		TeardownPhysicalTether();
	}

	if (!PhysicalTetherProxy)
	{
		PhysicalTetherProxy = NewObject<USphereComponent>(Owner,
			MakeUniqueObjectName(Owner, USphereComponent::StaticClass(), TEXT("RopeTetherProxy")));
		PhysicalTetherProxy->SetupAttachment(this);
		PhysicalTetherProxy->SetAbsolute(true, true, true); // 월드 배치(로프 컴포넌트 트랜스폼 무관).
		PhysicalTetherProxy->InitSphereRadius(4.0f);
		// 바디는 필요하고(제약의 한쪽) 충돌은 없어야 한다: 물리 켬 + 전 채널 무시.
		PhysicalTetherProxy->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
		PhysicalTetherProxy->SetCollisionResponseToAllChannels(ECR_Ignore);
		PhysicalTetherProxy->SetSimulatePhysics(false); // 키네마틱 — 매 프레임 코너로 이동.
		PhysicalTetherProxy->SetHiddenInGame(true);
		PhysicalTetherProxy->RegisterComponent();
	}
	// 키네마틱 이동 — Chaos가 이동 속도를 보고 제약을 당긴다(움직이는 코너/손 추종).
	PhysicalTetherProxy->SetWorldLocation(CornerWorld);

	if (!PhysicalTetherConstraint)
	{
		PhysicalTetherConstraint = NewObject<UPhysicsConstraintComponent>(Owner,
			MakeUniqueObjectName(Owner, UPhysicsConstraintComponent::StaticClass(), TEXT("RopeTetherConstraint")));
		PhysicalTetherConstraint->SetupAttachment(PhysicalTetherProxy);
		PhysicalTetherConstraint->RegisterComponent();
		PhysicalTetherConstraint->SetWorldLocation(CornerWorld);
		PhysicalTetherConstraint->SetDisableCollision(false);
		PhysicalTetherConstraint->SetConstrainedComponents(PhysicalTetherProxy, NAME_None, TargetSkel, Bone);
		// 제약 프레임 오리진: 프록시 쪽 = 프록시 원점(코너), 본 쪽 = 앵커의 본-로컬(레버 — wrap이 얼린
		// 본-로컬 앵커와 같은 규약). 거리 리밋은 이 두 점 사이에 걸린다.
		const int32 BoneIndex = TargetSkel->GetBoneIndex(Bone);
		const FTransform BoneTM = (BoneIndex != INDEX_NONE)
			? TargetSkel->GetBoneTransform(BoneIndex) : TargetSkel->GetComponentTransform();
		PhysicalTetherConstraint->ConstraintInstance.SetRefPosition(EConstraintFrame::Frame1, FVector::ZeroVector);
		PhysicalTetherConstraint->ConstraintInstance.SetRefPosition(EConstraintFrame::Frame2,
			BoneTM.InverseTransformPosition(AnchorWorld));
		// 로프는 회전을 구속하지 않는다 — 각도 전부 자유.
		PhysicalTetherConstraint->SetAngularSwing1Limit(ACM_Free, 0.0f);
		PhysicalTetherConstraint->SetAngularSwing2Limit(ACM_Free, 0.0f);
		PhysicalTetherConstraint->SetAngularTwistLimit(ACM_Free, 0.0f);
		PhysicalTetherTarget = TargetSkel;
		PhysicalTetherBone = Bone;
		PhysicalTetherLimit = -1.0f; // 아래에서 강제 갱신.
		UE_LOG(LogDynamicRope, Verbose, TEXT("[%s] physical tether created: %s/%s"),
			*GetName(), *GetNameSafe(TargetSkel), *Bone.ToString());
	}

	// 구면 거리 리밋 = 다리 rest(되감기/앵커 이동 자동 반영). XYZ Limited + 동일값 = 반경 리밋.
	const float Limit = FMath::Max(LegRestLen, 1.0f);
	if (!FMath::IsNearlyEqual(PhysicalTetherLimit, Limit, 0.5f))
	{
		PhysicalTetherConstraint->SetLinearXLimit(LCM_Limited, Limit);
		PhysicalTetherConstraint->SetLinearYLimit(LCM_Limited, Limit);
		PhysicalTetherConstraint->SetLinearZLimit(LCM_Limited, Limit);
		PhysicalTetherLimit = Limit;
	}

	// 장력 관측: 제약이 실제로 낸 힘(kg·cm/s²)을 λ 채널(λ = T×dt)로 실어 GetTetherTension()/디버거와 호환.
	FVector LinearForce = FVector::ZeroVector;
	FVector AngularForce = FVector::ZeroVector;
	PhysicalTetherConstraint->GetConstraintForce(LinearForce, AngularForce);
	PullDrive.LastTetherLambda = FMath::Max(PullDrive.LastTetherLambda,
		static_cast<float>(LinearForce.Size()) * DeltaTime);
}

void URopeComponent::TeardownPhysicalTether()
{
	if (PhysicalTetherConstraint)
	{
		PhysicalTetherConstraint->BreakConstraint();
		PhysicalTetherConstraint->DestroyComponent();
		PhysicalTetherConstraint = nullptr;
	}
	if (PhysicalTetherProxy)
	{
		PhysicalTetherProxy->DestroyComponent();
		PhysicalTetherProxy = nullptr;
	}
	PhysicalTetherTarget = nullptr;
	PhysicalTetherBone = NAME_None;
	PhysicalTetherLimit = -1.0f;
}

USkeletalMeshComponent* URopeComponent::GetWrappedMesh() const
{
	// State.Mesh는 USceneComponent(정적 랩 대비 일반화). "스켈레탈 메시" 반환 계약 유지 —
	// 정적 대상이면 Cast 실패로 null(대상 액터 반응은 호출자가 GetOwner로 이어감).
	return const_cast<USkeletalMeshComponent*>(Cast<USkeletalMeshComponent>(WrapController.State.Mesh.Get()));
}

void URopeComponent::ApplyPullForce(const FVector& Force, const FRopePullSample& Pull, float DeltaTime)
{
	// wrap 대상 컴포넌트(cross-actor 가능). 계약상 로프는 대상을 읽기만 하므로 weak가 const지만,
	// Pull은 의도된 게임플레이 개입(힘 인가)이라 여기서만 명시적으로 non-const로 푼다.
	// State.Mesh는 이제 USceneComponent(정적 랩 대비 일반화) — 대상 타입을 가리지 않고 수신자 체인으로
	// 힘을 인가한다(가벼운 물리 프랍/정적 대상도 스켈레탈과 동일 로직). null은 대상 소실(파괴)일 때뿐.
	USceneComponent* MeshComp = const_cast<USceneComponent*>(WrapController.State.Mesh.Get());
	if (!MeshComp)
	{
		return;
	}
	AActor* Owner = MeshComp->GetOwner();

	// Force = 당김 방향 × 최대 장력. 물리 바디는 장력 상한 속도 드라이브(ApplyPullVelocityDrive)로 인가한다.
	const float MaxTension = static_cast<float>(Force.Size());
	const FVector Dir = (MaxTension > KINDA_SMALL_NUMBER) ? (Force / MaxTension) : FVector::ZeroVector;

	// 수신자 해석은 테더와 같은 래더를 쓴다(ResolveTetherEndpoint) — 능동 Pull이 자기 래더를 따로 걷던 것을
	// 없앴다. 예전엔 순서가 달라(Pull은 캐릭터 무브먼트를 시뮬 프리미티브보다 먼저 봤다) "활성 CMC 캐릭터가
	// 소유한 시뮬 프리미티브"에 감기면 Pull은 무브먼트를, 테더는 그 프리미티브를 끌었다. 무브먼트 분기는
	// 원래 "애니메이션 본이라 밀 수 없으니 이동체를 민다"는 *폴백*인데 시뮬 검사보다 앞서 있어 밀 수 있는
	// 대상까지 가로챈 것 — 구체적(물리 바디) → 일반적(이동체) 순서로 통일한다.
	// (해석이 함께 내는 유효질량은 Pull이 쓰지 않는다 — 장력 상한 드라이브가 바디 질량을 직접 읽는다.)
	const bool bCanReuseWrappedEndpoint = WrappedEndpointCache.bValid &&
		WrappedEndpointCache.TargetMesh.Get() == MeshComp && WrappedEndpointCache.TargetBone == Pull.Bone;
	const FRopeTetherEndpoint Endpoint = bCanReuseWrappedEndpoint
		? WrappedEndpointCache.Target
		: ResolveTetherEndpoint(MeshComp, Owner, Pull.Bone, HoldConfig.GroundBraceFactor);

	// 확장 관문: 수신자 단위로 가로채는 서브클래스(커스텀 무브먼트/탈것)가 처리했으면 내장 인가를 생략한다.
	if (ApplyTractionToReceiver(MakeTractionRequest(Endpoint, ERopeTractionSource::ActivePull, Dir, MaxTension,
		DeltaTime, /*bWielderSide*/ false)))
	{
		return;
	}

	switch (Endpoint.Kind)
	{
	case ERopeEndpointKind::SimBody:
		// 시뮬 바디(스켈레탈 승격 본 / 시뮬 프리미티브 / 시뮬 루트): 장력 상한 속도 드라이브로 직접 인가한다
		// (무게중심 임펄스라 토크/스핀 없음, 오버슛 없어 먼지/턱턱 없음, 무거우면 뒤처짐). 각속도 클램프로 잔여 스핀 억제.
		ApplyPullVelocityDrive(Endpoint.Prim, Endpoint.Bone, Dir, MaxTension, DeltaTime);
		ClampPulledBodyVelocity(Endpoint.Prim, Endpoint.Bone);
		// (부분 랙돌의 "본 + 이동체 이중 인가"는 제거됐다: 수신자 해석이 부분 랙돌을 더 이상 본으로 내리지
		// 않고 캐릭터 rung으로 폴스루하므로 — ResolveTetherEndpoint rung 1 — 여기 오는 본 endpoint는 항상
		// 자유 랙돌이고, 힘은 관절로 몸 전체에 전달된다. 테더와 Pull이 같은 수신자를 보는 대칭 복원.)
		return;

	case ERopeEndpointKind::Character:
		// 애니메이션 구동 본에는 힘을 줄 수 없으므로 이동체 전체를 견인한다(PoC 4.2: 본/루트에 단순 힘 전달까지.
		// 팔다리 IK/랙돌 반응은 후속). MOVE_None이면 여기로 오지 않는다(해석이 앵커로 분류 → 아래 경고).
		Endpoint.Movement->AddForce(Force);
		return;

	default:
		break;
	}

	// 수신자 없음(앵커/None = 시뮬 바디 없는 본 체인·비시뮬 컴포넌트 + 무브먼트 비활성·비캐릭터 + 비시뮬 루트):
	// 힘이 조용히 사라지는 걸 wrap당 1회 알린다.
	if (!PullDrive.bLoggedPullNoReceiver)
	{
		PullDrive.bLoggedPullNoReceiver = true;
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] Pull has no force receiver: target=%s bone=%s has no simulating body up its parent chain, owner=%s has no force-consuming CharacterMovement (not a Character, or movement disabled) and its root is not simulating — pull force is dropped."),
			*GetName(), *MeshComp->GetName(), *Pull.Bone.ToString(), *GetNameSafe(Owner));
	}
}

void URopeComponent::ApplyPullVelocityDrive(UPrimitiveComponent* Prim, FName BoneName, const FVector& Dir, float MaxTension, float DeltaTime) const
{
	// 장력 상한 속도 드라이브(상수 힘 대체): 대상을 당김 방향 목표 속도(VTarget)로 몰되, 이번 프레임 적용할
	// 임펄스를 J = min(질량×ΔV, MaxTension×dt)로 클램프한다.
	//  - 가벼운 대상: J = 질량×ΔV(장력 여유) → 목표 속도에 *정확히* 도달(오버슛 없음). 상수 힘이 a=F/m로 한 프레임에
	//    목표를 훌쩍 넘겨 튕기던(먼지/턱턱) 문제가 사라진다.
	//  - 무거운 대상: J = MaxTension×dt(장력 한계) → 프레임당 ΔV=J/질량으로 천천히 가속 → 뒤처진다(현실적).
	// 임펄스는 무게중심(위치 없는 AddImpulse)이라 토크/스핀 없음. bVelChange=false = 실제 임펄스(질량 나눔).
	const float VTarget = FMath::Max(0.0f, HoldConfig.ActivePullMaxLinearSpeed);
	if (!Prim || VTarget <= 0.0f || MaxTension <= KINDA_SMALL_NUMBER || Dir.IsNearlyZero() || DeltaTime <= 0.0f)
	{
		return;
	}
	// 가속만(역추진 없음) + 정확 도달(Alpha=1). 상한은 임펄스(장력×dt)가 건다 — 테더 리엘과 같은 골격이고
	// 양방향 여부만 다르다(테더는 경계 안착을 위해 제동까지 하지만, 능동 Pull은 사용자가 놓으면 그만이라 가속만).
	const RopeTraction::FRopeAxisServo Servo{ VTarget, /*Alpha*/ 1.0f, /*bBidirectional*/ false, /*bCancelOutward*/ false };
	const float VAlong = static_cast<float>(FVector::DotProduct(Prim->GetPhysicsLinearVelocity(BoneName), Dir));
	const float J = RopeTraction::ClampAxisImpulse(RopeTraction::ComputeAxisDeltaV(VAlong, Servo), ResolveBodyMass(Prim, BoneName), MaxTension * DeltaTime);
	if (!FMath::IsNearlyZero(J))
	{
		Prim->AddImpulse(Dir * J, BoneName, /*bVelChange*/ false);
	}
}

void URopeComponent::ClampPulledBodyVelocity(UPrimitiveComponent* Prim, FName BoneName) const
{
	if (!Prim)
	{
		return;
	}
	// 각속도 상한(잔여 스핀 안전망): 힘을 무게중심에 줘 pull 토크 원인은 제거했지만, 랙돌 관절 다이내믹의 잔여
	// 스핀을 마저 가둔다. (선형 견인은 ApplyPullVelocityDrive의 장력 상한 임펄스가 담당 — 여기선 각속도만.)
	const float MaxAngDeg = FMath::Max(0.0f, HoldConfig.ActivePullMaxAngularSpeed);
	if (MaxAngDeg > 0.0f)
	{
		const float MaxAngRad = FMath::DegreesToRadians(MaxAngDeg);
		const FVector AngVel = Prim->GetPhysicsAngularVelocityInRadians(BoneName);
		if (AngVel.SizeSquared() > MaxAngRad * MaxAngRad)
		{
			Prim->SetPhysicsAngularVelocityInRadians(AngVel.GetClampedToMaxSize(MaxAngRad), /*bAddToCurrent*/ false, BoneName);
		}
	}
}

void URopeComponent::UpdateTargetPullable()
{
	// 이번 Wrapped 프레임의 끌림 가능 판정(climb-in 방향/분배 관측 공용). overshoot와 무관하게 매 프레임 산출해
	// 테더 회수(UpdateTether)와 능동 Pull 방향(ApplyWrappedTraction)이 같은 판정을 읽게 한다.
	const FRopeResolvedWrappedEndpoints* Endpoints = GetOrResolveWrappedEndpoints();
	if (!Endpoints)
	{
		return; // 대상 소실(파괴) — Hold가 곧 release. 직전 판정 유지.
	}
	USceneComponent* MeshComp = Endpoints->TargetMesh.Get();
	if (!MeshComp)
	{
		return;
	}

	// 자기 자신에 감긴 로프(owner==대상)는 분배 무의미 → 항상 대상 회수(pullable).
	const bool bSelfWrap = (GetOwner() != nullptr && MeshComp->GetOwner() == GetOwner());
	bool bPullable;
	if (bSelfWrap)
	{
		bPullable = true;
	}
	else
	{
		// 양끝 유효질량(접지 캐릭터는 GroundBraceFactor로 접지마찰 반영, MOVE_None/정적은 앵커=무한).
		// 테더 인가와 같은 해석(ResolveTetherEndpoint)을 쓴다 — 끌림 판정과 실제 인가점이 어긋나지 않는다.
		const float WT = EndpointInvMass(Endpoints->Target);
		const float WW = EndpointInvMass(Endpoints->Wielder);
		const float InfMass = TNumericLimits<float>::Max();
		const float EffMassTarget = (WT > KINDA_SMALL_NUMBER) ? (1.0f / WT) : InfMass; // invMass 0 = 앵커(무한).
		const float EffMassWielder = (WW > KINDA_SMALL_NUMBER) ? (1.0f / WW) : InfMass;
		if (!PullDrive.bTargetPullableInit)
		{
			bPullable = (EffMassTarget <= EffMassWielder); // 첫 유효 프레임: 히스테리시스 없이 순수 비교로 시드.
		}
		else
		{
			// 히스테리시스는 비노출 내부 상수(안정화 장치) — 경계에서 판정이 프레임마다 뒤집히는 것을 막는다.
			// "교차점 위치"를 정하는 노출 노브는 GroundBraceFactor 하나뿐이고, 이 값은 그 선 주변의 데드밴드일 뿐.
			constexpr float PullMassHysteresis = 1.1f;
			bPullable = DecideTargetPullable(EffMassTarget, EffMassWielder, PullDrive.bTargetPullable, PullMassHysteresis);
		}
	}
	PullDrive.bTargetPullable = bPullable;
	PullDrive.bTargetPullableInit = true;
	PullDrive.LastTargetShare = bPullable ? 1.0f : 0.0f; // wielder 게이트/디버거가 읽는 유효 몫(이진).
}

bool URopeComponent::DecideTargetPullable(float EffMassTarget, float EffMassWielder, bool bPrev, float MarginRatio)
{
	const float Margin = FMath::Max(MarginRatio, 1.0f);
	if (bPrev)
	{
		// 현재 "끌림 가능": 대상이 wielder보다 Margin배 넘게 무거워질 때만 불가로 뒤집는다(sticky).
		return !(EffMassTarget > EffMassWielder * Margin);
	}
	// 현재 "끌림 불가": 대상이 wielder × (1/Margin) 이하로 가벼워질 때만 가능으로 뒤집는다.
	return (EffMassTarget * Margin <= EffMassWielder);
}

void URopeComponent::ApplyPullForceToWielder(const FVector& Force, float DeltaTime)
{
	// (not pullable) 능동 Pull 힘을 wielder(로프 owner)에 인가 — 대상이 무거워 대신
	// wielder가 앵커 쪽으로 끌려가는 climb-in. ApplyPullForce의 owner 쪽 미러: CharacterMovement → 시뮬 루트.
	AActor* RopeOwner = GetOwner();
	if (!RopeOwner)
	{
		return;
	}

	// 수신자를 **먼저 해석**한다 — 확장 관문에 "무엇에 꽂힐 뻔했는지"를 그대로 넘기기 위함이다.
	// 해석 순서는 기존 그대로(CharacterMovement → 시뮬 루트): 여기서 ResolveTetherEndpoint 래더로
	// 갈아타면 시뮬 루트가 무브먼트보다 앞서게 되어 climb-in 동작이 바뀐다.
	FRopeTetherEndpoint Receiver;
	Receiver.Actor = RopeOwner;
	if (UCharacterMovementComponent* Movement = GetForceConsumingMovement(RopeOwner))
	{
		Receiver.Kind = ERopeEndpointKind::Character;
		Receiver.Movement = Movement;
	}
	else if (UPrimitiveComponent* Root = Cast<UPrimitiveComponent>(RopeOwner->GetRootComponent()))
	{
		if (Root->IsSimulatingPhysics())
		{
			Receiver.Kind = ERopeEndpointKind::SimBody;
			Receiver.Prim = Root;
		}
	}

	const float Magnitude = static_cast<float>(Force.Size());
	const FVector Dir = (Magnitude > KINDA_SMALL_NUMBER) ? (Force / Magnitude) : FVector::ZeroVector;
	if (ApplyTractionToReceiver(MakeTractionRequest(Receiver, ERopeTractionSource::ActivePull, Dir, Magnitude,
		DeltaTime, /*bWielderSide*/ true)))
	{
		return;
	}

	switch (Receiver.Kind)
	{
	case ERopeEndpointKind::Character:
		Receiver.Movement->AddForce(Force);
		return;
	case ERopeEndpointKind::SimBody:
		Receiver.Prim->AddForce(Force);
		return;
	default:
		// 수신자 없음(비캐릭터 + 비시뮬 루트): 조용히 드롭 — climb-in 불가한 구성.
		return;
	}
}

#pragma endregion Tether_And_Pull_Application

