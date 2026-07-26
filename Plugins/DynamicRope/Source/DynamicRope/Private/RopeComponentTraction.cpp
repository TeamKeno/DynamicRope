// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeComponent.h"

#include "Components/SkeletalMeshComponent.h"
#include "Components/SphereComponent.h"
#include "DynamicRopeLog.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "Core/RopeMovementConstraint.h"
#include "Logic/RopeLengthConstraintSolver.h"
#include "Logic/RopeTractionSolver.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"
#include "RopeMathHelpers.h"
#include "Templates/Function.h"

namespace
{
	// C1: taut 판정 히스테리시스·해제 유예는 실측 튜닝이 끝난 내부 상수다(단일 TautSensitivity로 통합하며
	// 디자이너 노출에서 제거). 값을 조정하려면 여기서.
	constexpr float TautSlackReleaseScaleConst = 2.0f;       // 슬랙/처짐 게이트 유지 배율(≥1, 진입/해제 임계 분리)
	constexpr float TautReleaseGraceTimeConst = 0.1f;        // bChainTaut 해제 유예(초)
	constexpr float ActivePullTautReleaseRatioConst = 0.5f;  // 능동 Pull load 임계 히스테리시스 [0..1]
	constexpr float TautMinTensionReleaseRatioConst = 0.5f;  // 최소 전달 장력 히스테리시스 [0..1]
	constexpr float TetherLiftLaunchSpeedConst = 100.0f;     // 접지 캐릭터 Walking→Falling 전환 상향 임계(cm/s)
}

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

void URopeComponent::UpdateWrappedPullSample(float DeltaTime, const FRopeSimState& ObservationSim)
{
	// Gameplay tension is the material-length reaction, not delayed XPBD segment strain.
	// ApplyWrappedTraction refreshes this again after the current frame's backend has solved.
	WrapController.State.Tension = GetConstraintTension();

	// Pull 샘플 산출(항상 — 디버거/BP 관찰 + 견인/release의 공용 입력). 방향은 첫 직선 다리 추종(공간).
	PullDrive.LastPullSample = FRopePullSample();
	WrapController.ComputePull(ObservationSim, HoldConfig.PullBendThresholdDeg, PullDrive.LastPullSample);

	// Taut is geometry, not a prerequisite XPBD load. Prefer the same live hand/anchor
	// material boundary used by movement authority. This removes the circular dependency
	// "segment must stretch -> tension appears -> tether may enforce no stretch".
	FRopeWielderMovementConstraint LiveConstraint;
	const bool bHasLiveConstraint = BuildWielderMovementConstraint(LiveConstraint);
	// 슬랙 비율·처짐 상한은 단일 TautSensitivity에서 해석한다(GetEffectiveTaut*), 히스테리시스는 내부 상수.
	// live material-length 경로와 legacy geometry 경로가 **같은 방향으로** 반응해야 하므로(기본값
	// bEnforceWielderLengthConstraint=true에서도 슬라이더가 체감되도록) 두 경로가 이 값들을 공유한다.
	// 진입/유지 히스테리시스: 일단 팽팽으로 판정되면 허용을 넓혀 임계 경계의 채터링을 막는다(+ 아래 grace 래치).
	const float EffectiveTautMaxSag = GetEffectiveTautMaxSag();
	const float EffectiveTautSlackRatio = GetEffectiveTautSlackRatio();
	const float TautHysteresis = PullDrive.bChainTaut ? TautSlackReleaseScaleConst : 1.0f;
	const float SagLimit = EffectiveTautMaxSag * TautHysteresis;
	const bool bSagTaut =
		EffectiveTautMaxSag <= 0.0f || PullDrive.LastPullSample.MaxLegSag <= SagLimit;

	// live 경계: 견인 시작 거리 slack을 TautSensitivity에서 파생하되, LengthConstraintActivationSlop은
	// 수치 안정성용 최소 허용치로 유지한다(둘 중 큰 값). "시각적으로 펴졌을 때만"을 위해 sag 게이트도 공유.
	const float LiveSlackAllowance = FMath::Max(
		FMath::Max(HoldConfig.LengthConstraintActivationSlop, 0.0f),
		LiveConstraint.MaxDistance * EffectiveTautSlackRatio * TautHysteresis);
	const float LiveDistance =
		static_cast<float>(FVector::Distance(GetComponentLocation(), LiveConstraint.PivotWorld));
	const bool bLiveBoundaryTaut = bHasLiveConstraint
		&& LiveDistance >= FMath::Max(0.0f, LiveConstraint.MaxDistance - LiveSlackAllowance)
		&& bSagTaut;

	// Legacy/self-wrap fallback still uses sag + chord geometry. SegmentTension is deliberately
	// excluded from gameplay taut; it remains only as a legacy analytic-path contamination guard.
	const bool bLegacyGeometryTaut =
		bSagTaut
		&& (EffectiveTautSlackRatio <= 0.0f || RopeTraction::EvaluateChainTautGate(
			PullDrive.LastPullSample.TautChordLen,
			PullDrive.LastPullSample.FreeRestLen,
			EffectiveTautSlackRatio,
			TautSlackReleaseScaleConst,
			PullDrive.bChainTaut));
	const bool bRawChainTaut = PullDrive.LastPullSample.bValid
		&& (bHasLiveConstraint ? bLiveBoundaryTaut : bLegacyGeometryTaut);
	// 해제 유예(시간 래치): 최소 전달 장력 관측치는 임계 0(기본)에서 진입/유지 임계가 같아 히스테리시스가
	// 소멸하고 GPU 로프에선 1~2프레임 지연 미러라, 경계 상태에서 판정이 프레임 단위로 퍼덕이며 "전량 속도
	// 삭감 ↔ 자유"가 교대한다(wielder 들썩임). 진입은 즉시, 해제만 TautReleaseGraceTime 동안 유예해 채터링을
	// 끊는다. 샘플 무효는 유예 없이 즉시 false(위 확정 계약 유지 — 관측 공백을 팽팽으로 연장하지 않는다).
	if (bRawChainTaut)
	{
		PullDrive.bChainTaut = true;
		PullDrive.TautGraceRemaining = TautReleaseGraceTimeConst;
	}
	else if (!PullDrive.LastPullSample.bValid)
	{
		PullDrive.bChainTaut = false;
		PullDrive.TautGraceRemaining = 0.0f;
	}
	else if (PullDrive.bChainTaut && PullDrive.TautGraceRemaining > 0.0f)
	{
		PullDrive.TautGraceRemaining -= DeltaTime;
		PullDrive.bChainTaut = PullDrive.TautGraceRemaining > 0.0f;
	}
	else
	{
		PullDrive.bChainTaut = false;
	}

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
	const FVector AimPos = RopeTraction::SampleFractionalAim(
		ObservationSim.Positions, AimF, PullDrive.LastPullSample.AnchorNode);
	PullDrive.LastPullSample.AimNodeF = AimF;
	PullDrive.LastPullSample.AimPos = AimPos;

	// (2) 연속 조준으로 방향 재계산 후 방향 EMA. 축퇴(조준=앵커)면 raw 방향 유지.
	const FVector DirF =
		(AimPos - ObservationSim.Positions[PullDrive.LastPullSample.AnchorNode]).GetSafeNormal();
	const FVector DirIn = DirF.IsNearlyZero() ? PullDrive.LastPullSample.Direction : DirF;
	PullDrive.SmoothedPullDir = RopeTraction::SmoothDirection(
		PullDrive.SmoothedPullDir, DirIn, RopeTraction::ExpSmoothAlpha(HoldConfig.PullDirSmoothTime, DeltaTime));
	PullDrive.LastPullSample.Direction = PullDrive.SmoothedPullDir;
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
	WrapController.State.Tension = GetConstraintTension();

	// Geometry starts a pull; load is an optional gameplay threshold, never a prerequisite
	// for the passive length solve. Threshold zero explicitly means geometry-only so a
	// stationary taut cable can begin an active pull and create its own reaction.
	if (HoldConfig.ActivePullTautTension <= 0.0f)
	{
		PullDrive.bPullTaut = PullDrive.bChainTaut;
	}
	else
	{
		PullDrive.bPullTaut = PullDrive.bChainTaut && RopeTraction::EvaluateTautGate(
			GetConstraintTension(),
			HoldConfig.ActivePullTautTension,
			ActivePullTautReleaseRatioConst,
			PullDrive.bPullTaut);
	}

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
	// ERopeEndpointKind/FRopeTetherEndpoint는 Core/RopeTractionTypes.h의 공용 판정 타입이다. 컴포넌트는
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
		// 가장 가까운 *시뮬 본*(바디 없는 트위스트 본 대응)에 인가한다. 여기 저장하는 기본 질량은 **전신 바디
		// 질량 합**(GetMass)이다 — hard Chaos 반력의 coarse wielder/target 몫과 pullability가 본 바디
		// (팔뚝 3kg)를 "가벼운 대상"으로 오판하지 않게 한다. Compliant analytic solve는 실제 world
		// attachment가 정해진 뒤 이 기본값을 선택 본의 병진+회전 point Jacobian으로 정밀화한다.
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
		if (UCharacterMovementComponent* Movement = GetForceConsumingMovement(Owner))
		{
			Out.Kind = ERopeEndpointKind::Character;
			Out.Movement = Movement;
			const float BraceScale = Movement->IsMovingOnGround() ? FMath::Max(GroundBraceFactor, 1.0f) : 1.0f;
			Out.Mass = Movement->Mass * BraceScale;
			return Out;
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

	bool BuildPointMassProperties(
		const FRopeTetherEndpoint& Endpoint,
		RopeTraction::FRopePointMassProperties& Out)
	{
		if (Endpoint.Kind != ERopeEndpointKind::SimBody ||
			!Endpoint.Prim)
		{
			return false;
		}
		FBodyInstance* Body =
			Endpoint.Prim->GetBodyInstance(Endpoint.Bone);
		if (!Body)
		{
			return false;
		}
		Out.Mass = static_cast<float>(Body->GetBodyMass());
		Out.InertiaTensor = Body->GetBodyInertiaTensor();
		Out.MassSpaceToWorld = Body->GetMassSpaceToWorldSpace();
		return Out.Mass > KINDA_SMALL_NUMBER;
	}

	float EndpointPointInvMass(
		const FRopeTetherEndpoint& Endpoint,
		const FVector& PointWorld,
		const FVector& DirectionWorld)
	{
		RopeTraction::FRopePointMassProperties Body;
		if (BuildPointMassProperties(Endpoint, Body))
		{
			const float PointInvMass =
				RopeTraction::ComputePointInverseMass(
					Body, PointWorld, DirectionWorld);
			if (PointInvMass > KINDA_SMALL_NUMBER)
			{
				return PointInvMass;
			}
		}
		return EndpointInvMass(Endpoint);
	}

	FVector EndpointVelocityAtPoint(
		const FRopeTetherEndpoint& Endpoint,
		const FVector& PointWorld)
	{
		switch (Endpoint.Kind)
		{
		case ERopeEndpointKind::SimBody:
			return Endpoint.Prim
				? Endpoint.Prim->GetPhysicsLinearVelocityAtPoint(
					PointWorld, Endpoint.Bone)
				: FVector::ZeroVector;
		case ERopeEndpointKind::Character:
			return Endpoint.Movement
				? Endpoint.Movement->Velocity
				: FVector::ZeroVector;
		default:
			return FVector::ZeroVector;
		}
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

float URopeComponent::ComputeWielderLengthReactionShare(
	const FRopeWielderMovementConstraint& Constraint) const
{
	if (!Constraint.IsValid())
	{
		return 1.0f;
	}

	USceneComponent* TargetComponent =
		const_cast<USceneComponent*>(Constraint.TargetComponent.Get());
	const FRopeTetherEndpoint Target = ResolveTetherEndpoint(
		TargetComponent,
		TargetComponent ? TargetComponent->GetOwner() : nullptr,
		Constraint.TargetBone,
		HoldConfig.GroundBraceFactor);
	const FRopeTetherEndpoint Wielder = ResolveTetherEndpoint(
		nullptr, GetOwner(), NAME_None, HoldConfig.GroundBraceFactor);
	if (Phase == ERopePhase::Wrapping &&
		Target.Kind != ERopeEndpointKind::SimBody)
	{
		// Wrapped PostPhysics applies the complementary analytic target share. Wrapping
		// deliberately does not run that traction stage yet, so a nonphysical target has
		// no same-frame receiver for its share; close relative velocity on the Wielder
		// instead until commit. SimBody targets still receive their share through Chaos.
		return 1.0f;
	}
	const float WTarget = EndpointInvMass(Target);
	const float WWielder = EndpointInvMass(Wielder);
	const float WSum = WTarget + WWielder;
	return WSum > KINDA_SMALL_NUMBER
		? FMath::Clamp(WWielder / WSum, 0.0f, 1.0f)
		: 1.0f;
}

float URopeComponent::ComputeWielderLengthPositionCorrectionShare(
	const FRopeWielderMovementConstraint& Constraint) const
{
	if (!Constraint.IsValid() ||
		HoldConfig.TetherCompliance > KINDA_SMALL_NUMBER)
	{
		return 1.0f;
	}

	USceneComponent* TargetComponent =
		const_cast<USceneComponent*>(Constraint.TargetComponent.Get());
	const FRopeTetherEndpoint Target = ResolveTetherEndpoint(
		TargetComponent,
		TargetComponent ? TargetComponent->GetOwner() : nullptr,
		Constraint.TargetBone,
		HoldConfig.GroundBraceFactor);
	return Target.Kind == ERopeEndpointKind::SimBody
		? ComputeWielderLengthReactionShare(Constraint)
		: 1.0f;
}

void URopeComponent::PrepareWielderLengthConstraint(
	const FRopeWielderMovementConstraint& Constraint,
	const FVector& OutwardNormal,
	float RejectedSeparatingSpeed,
	float PositionViolation,
	bool bAtLimit,
	bool bHardProjectionApplied,
	float DeltaTime)
{
	if (!Constraint.IsValid() || DeltaTime <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	if (bHardProjectionApplied &&
		bAtLimit &&
		(RejectedSeparatingSpeed > KINDA_SMALL_NUMBER ||
			PositionViolation > KINDA_SMALL_NUMBER))
	{
		LengthConstraintState.RecordWielderAttempt(
			GFrameCounter,
			Constraint.AnchorNode,
			PositionViolation,
			RejectedSeparatingSpeed,
			OutwardNormal,
			bAtLimit);
	}

	USceneComponent* TargetComponent =
		const_cast<USceneComponent*>(Constraint.TargetComponent.Get());
	AActor* TargetOwner = TargetComponent ? TargetComponent->GetOwner() : nullptr;
	const FRopeTetherEndpoint Target = ResolveTetherEndpoint(
		TargetComponent, TargetOwner, Constraint.TargetBone, HoldConfig.GroundBraceFactor);

	if (Target.Kind != ERopeEndpointKind::SimBody ||
		!Target.Prim ||
		HoldConfig.TetherCompliance > KINDA_SMALL_NUMBER)
	{
		// Runtime SimulatePhysics/compliance transitions are immediate. A compliant cable
		// uses the common analytic lambda solve (including its force cap) for every endpoint
		// type; Chaos is reserved for the uncapped, truly rigid substep constraint.
		TeardownPhysicalTether();
		return;
	}

	// The Wielder already kept only its generalized position-correction share. The
	// complementary target share must therefore be driven from that exact resulting hand
	// point. Reconstructing another offset from the original violation double-corrects the
	// position (full hand projection + target motion), creating artificial slack/tension
	// pulses and separating a zero-length hand/target pair.
	const FVector ChaosProxyWorld = GetComponentLocation();
	UpdatePhysicalTether(
		Target.Prim,
		Target.Bone,
		Constraint.AnchorWorld,
		ChaosProxyWorld,
		Constraint.MaxDistance,
		DeltaTime);
	if (PhysicalTetherConstraint)
	{
		PhysicalTetherPrePhysicsFrame = GFrameCounter;
	}
}

#pragma endregion Traction_Endpoint_And_Tether_Policies

#pragma region Tether_And_Pull_Application

FVector URopeComponent::ComputeSmoothedWielderDir(const FVector& Aim, const FVector& DirToAim, float DeltaTime, bool bInstantaneous)
{
	// 방향 = 손(노드 0)에서 로프의 첫 직선 다리를 따라. 조준(AimPos)이 벽 모서리면 모서리를 향하고, 로프가 곧아
	// 조준=손이면(chord ~0) 앵커→조준의 역방향(=손→앵커)으로 폴백한다.
	const FVector HandPos = GetComponentLocation();
	FVector WielderDirRaw = Aim - HandPos;
	if (!WielderDirRaw.Normalize(KINDA_SMALL_NUMBER))
	{
		WielderDirRaw = -DirToAim;
	}
	// 공중 스윙(bInstantaneous): EMA 생략, 순간 기하 그대로 — 궤도가 빠르게 도는 동안 래그된 축(ω·τ ≈
	// 십수도)의 접선 오차 성분이 매 발화 프레임 스윙을 제동/가속해 AirControl 조작을 방해한다. EMA의 원
	// 목적(서보 톱업의 랜덤워크 방어)은 λ 단발 임펄스+속력 클램프 체제에선 접지 코너 노이즈 쪽만 남았다.
	// 상태는 raw로 계속 시드해 착지 시 EMA 재진입이 연속이게 한다.
	if (bInstantaneous)
	{
		PullDrive.SmoothedWielderPullDir = WielderDirRaw;
		return WielderDirRaw;
	}
	// 방향 EMA(대상 쪽 SmoothedPullDir과 동일 상수·동일 함수): AimPos 노드 노이즈/모서리 전환/근접 축퇴로 raw
	// 방향이 프레임마다 튀면 클램프/톱업이 매번 다른 축으로 들어가 벡터가 랜덤워크로 불어난다(폭주).
	PullDrive.SmoothedWielderPullDir = RopeTraction::SmoothDirection(
		PullDrive.SmoothedWielderPullDir, WielderDirRaw, RopeTraction::ExpSmoothAlpha(HoldConfig.PullDirSmoothTime, DeltaTime));
	return PullDrive.SmoothedWielderPullDir;
}

void URopeComponent::UpdateConstraintTether(float DeltaTime)
{
	// One logical C <= 0 material constraint with mutually exclusive application backends:
	//   1) simulated target -> Chaos constraint,
	//   2) hard-projected Wielder -> rejected-motion reaction,
	//   3) no movement adapter -> legacy analytic endpoint impulse.
	// Position authority and tension therefore observe the same live material boundary.
	LengthConstraintState.BeginFrame(DeltaTime);
	if (DeltaTime <= 1e-4f)
	{
		LengthConstraintState.bPrevGeometryValid = false;
		return;
	}

	FRopeWielderMovementConstraint LiveConstraint;
	const bool bHasLiveConstraint = BuildWielderMovementConstraint(LiveConstraint);
	const bool bHasPullSample = PullDrive.LastPullSample.bValid;
	if (!bHasLiveConstraint && !bHasPullSample)
	{
		LengthConstraintState.bPrevGeometryValid = false;
		return;
	}
	const int32 ConstraintAnchorNode = bHasLiveConstraint
		? LiveConstraint.AnchorNode
		: PullDrive.LastPullSample.AnchorNode;
	const FVector Anchor = bHasLiveConstraint
		? LiveConstraint.PivotWorld
		: PullDrive.LastPullSample.WorldPoint;
	const float MaterialLength = bHasLiveConstraint
		? LiveConstraint.MaxDistance
		: PullDrive.LastPullSample.FreeRestLen;
	const float RequiredLength = bHasLiveConstraint
		? static_cast<float>(FVector::Distance(GetComponentLocation(), Anchor))
		: PullDrive.LastPullSample.PathChordLen;
	float C = RequiredLength - MaterialLength;
	LengthConstraintState.LastViolation = FMath::Max(C, 0.0f);
	const bool bHardWielderAttempt =
		LengthConstraintState.HasWielderAttempt(GFrameCounter, ConstraintAnchorNode);

	// Reel/material-length and kinematic anchor rates are sampled from the same live
	// geometry. Activation tolerance must not add physical cable length.
	float RestRate = 0.0f;
	if (LengthConstraintState.bPrevGeometryValid &&
		LengthConstraintState.PrevAnchorNode == ConstraintAnchorNode)
	{
		RestRate =
			(MaterialLength - LengthConstraintState.PrevMaterialLength) / DeltaTime;
		const FVector RawAnchorVel =
			(Anchor - LengthConstraintState.PrevAnchorWorldPoint) / DeltaTime;
		LengthConstraintState.SmoothedAnchorPointVelocity = FMath::Lerp(
			LengthConstraintState.SmoothedAnchorPointVelocity,
			RawAnchorVel,
			RopeTraction::ExpSmoothAlpha(HoldConfig.PullDirSmoothTime, DeltaTime));
		// 손 점(wielder 끝) 실측 속도 — Anchor-kind wielder(키네마틱 캐리어: 헬기/이동 플랫폼)는 물리
		// 속도가 없어 유한차분으로 채운다. 대상 쪽 SmoothedAnchorPointVelocity의 wielder 거울.
		const FVector RawWielderVel =
			(GetComponentLocation() - LengthConstraintState.PrevWielderWorldPoint) / DeltaTime;
		LengthConstraintState.SmoothedWielderPointVelocity = FMath::Lerp(
			LengthConstraintState.SmoothedWielderPointVelocity,
			RawWielderVel,
			RopeTraction::ExpSmoothAlpha(HoldConfig.PullDirSmoothTime, DeltaTime));
	}
	LengthConstraintState.PrevMaterialLength = MaterialLength;
	LengthConstraintState.PrevAnchorWorldPoint = Anchor;
	LengthConstraintState.PrevWielderWorldPoint = GetComponentLocation();
	LengthConstraintState.PrevAnchorNode = ConstraintAnchorNode;
	LengthConstraintState.bPrevGeometryValid = true;

	FRopeResolvedWrappedEndpoints LiveEndpoints;
	const FRopeResolvedWrappedEndpoints* Endpoints = GetOrResolveWrappedEndpoints();
	if (!Endpoints && bHasLiveConstraint)
	{
		// Pull sampling needs at least one hand-side segment and is therefore invalid for a
		// legitimate node-0 constraint (and during short observation gaps). Resolve the same
		// live binding directly so hard projection/Chaos can still publish reaction tension.
		USceneComponent* TargetComponent =
			const_cast<USceneComponent*>(LiveConstraint.TargetComponent.Get());
		LiveEndpoints.Target = ResolveTetherEndpoint(
			TargetComponent,
			TargetComponent ? TargetComponent->GetOwner() : nullptr,
			LiveConstraint.TargetBone,
			HoldConfig.GroundBraceFactor);
		LiveEndpoints.Wielder = ResolveTetherEndpoint(
			nullptr, GetOwner(), NAME_None, HoldConfig.GroundBraceFactor);
		LiveEndpoints.TargetMesh = TargetComponent;
		LiveEndpoints.TargetBone = LiveConstraint.TargetBone;
		Endpoints = &LiveEndpoints;
	}
	if (!Endpoints)
	{
		return;
	}
	USceneComponent* MeshComp = Endpoints->TargetMesh.Get();
	if (!MeshComp)
	{
		return;
	}
	const bool bSelfWrap =
		(GetOwner() != nullptr && MeshComp->GetOwner() == GetOwner());
	const float ResolvedInvMassTarget =
		EndpointInvMass(Endpoints->Target);
	const float ResolvedInvMassWielder =
		bSelfWrap ? 0.0f : EndpointInvMass(Endpoints->Wielder);
	const float ResolvedWSum =
		ResolvedInvMassTarget + ResolvedInvMassWielder;
	// Chaos needs a share before the rope directions/point Jacobians below are built.
	// Analytic paths replace this with their exact point-mass split before solving.
	PullDrive.LastTargetShare =
		(ResolvedWSum > KINDA_SMALL_NUMBER)
			? (ResolvedInvMassTarget / ResolvedWSum)
			: 0.0f;

	const bool bPhysicalTarget = (Endpoints->Target.Kind == ERopeEndpointKind::SimBody
		&& Endpoints->Target.Prim != nullptr);
	const bool bUseChaosBackend =
		bPhysicalTarget &&
		HoldConfig.TetherCompliance <= KINDA_SMALL_NUMBER;

	const FVector Aim = bHasPullSample
		? PullDrive.LastPullSample.AimPos
		: GetComponentLocation();
	if (bUseChaosBackend)
	{
		// Simulated targets are exclusively owned by Chaos. Never add the analytic
		// rejected-motion impulse on top of this constraint.
		LengthConstraintState.Backend = ERopeLengthConstraintBackend::Chaos;
		const float LegRest = bHasLiveConstraint
			? MaterialLength
			: FMath::Max(0.0f,
				(static_cast<float>(PullDrive.LastPullSample.AnchorNode) -
					PullDrive.LastPullSample.AimNodeF) * Sim.SegmentLength);
		const bool bDrivenByWielderThisFrame =
			PhysicalTetherPrePhysicsFrame == GFrameCounter &&
			PhysicalTetherTarget.Get() == Endpoints->Target.Prim &&
			PhysicalTetherBone == Endpoints->Target.Bone;
		if (bDrivenByWielderThisFrame)
		{
			// PrePhysics already wrote the attempted, unclamped hand point and material-length
			// limit. Do not replace it with the look-ahead Aim contract after Chaos.
			SamplePhysicalTetherForce(DeltaTime);
		}
		else
		{
			// Legacy/custom-mover fallback when no Wielder drove the authoritative path.
			// A live material constraint always uses the actual hand point. Pull Aim is a
			// look-ahead node and can sit deep inside a curved/sagging leg; pairing it with
			// the full hand-to-anchor rest length silently creates unrelated slack.
			const FVector ProxyWorld = bHasLiveConstraint
				? GetComponentLocation()
				: Aim;
			UpdatePhysicalTether(
				Endpoints->Target.Prim,
				Endpoints->Target.Bone,
				Anchor,
				ProxyWorld,
				LegRest,
				DeltaTime);
		}
		return;
	}
	TeardownPhysicalTether();

	// Only the fallback that still derives C from delayed/deformable particle chords needs
	// the historical partial-stretch contamination guard. Live material geometry and hard
	// movement attempts never depend on XPBD SegmentTension.
	const bool bLegacyPathLoaded = bHasPullSample && RopeTraction::EvaluateTautGate(
		PullDrive.LastPullSample.MinFreeTension,
		HoldConfig.TautMinTension,
		TautMinTensionReleaseRatioConst,
		PullDrive.bChainTaut);
	if (!bHasLiveConstraint && !bHardWielderAttempt &&
		(!PullDrive.bChainTaut || !bLegacyPathLoaded))
	{
		return;
	}

	// The live constraint uses the exact hand/anchor normal. Legacy paths retain their
	// smoothed look-ahead direction for corner noise.
	const FVector Span = (Aim - Anchor).GetSafeNormal();
	const FVector CurrentLiveOutward =
		(GetComponentLocation() - Anchor).GetSafeNormal();
	const FVector LiveOutward =
		!CurrentLiveOutward.IsNearlyZero()
			? CurrentLiveOutward
			: (bHardWielderAttempt
				? LengthConstraintState.WielderAttemptOutwardNormal
				: FVector::ZeroVector);
	const FVector LegacyTargetDir =
		PullDrive.SmoothedPullDir.IsNearlyZero() ? Span : PullDrive.SmoothedPullDir;
	const FVector DirTarget =
		(bHasLiveConstraint && !LiveOutward.IsNearlyZero()) ? LiveOutward : LegacyTargetDir;
	if (DirTarget.IsNearlyZero())
	{
		return; // 축퇴(조준=앵커) — 방향 정의 불가.
	}
	// 공중 스윙 중엔 wielder 방향 EMA를 생략(순간 기하) — 궤도가 빠르게 도는 동안(ω·τ ≈ 십수도 래그)
	// 래그된 축의 접선 오차 성분이 매 발화 프레임 스윙을 제동/가속해 조작을 방해한다(2026-07-22 PIE,
	// AirControl 부스트가 무력해지는 "특정 순간의 힘"). 접지는 코너/노드 노이즈가 커 EMA 유지.
	const bool bWielderAirborne = (Endpoints->Wielder.Kind == ERopeEndpointKind::Character
		&& Endpoints->Wielder.Movement && Endpoints->Wielder.Movement->IsFalling());
	const FVector DirWielder =
		(bHasLiveConstraint && !LiveOutward.IsNearlyZero())
			? -LiveOutward
			: ComputeSmoothedWielderDir(Aim, DirTarget, DeltaTime, bWielderAirborne);
	if (DirWielder.IsNearlyZero())
	{
		return;
	}

	// 자기 랩(owner == 대상): 양끝이 같은 몸이라 쌍 인가가 자가 상쇄된다 — wielder 끝을 앵커(w=0)로 취급해
	// 대상 끝만 움직인다(레거시 특례와 동일).
	// 벌어짐 속도 s = −(vT·dT + vW·dW) − dRest/dt. 끝 속도는 수신자 해석과 같은 rung에서 실측한다 —
	// 움직이는 앵커(드래곤)의 순항은 vT에 실려 별도 피드포워드 없이 추종된다(레거시의 앵커 속도 EMA 대체).
	const FVector TargetPoint = Anchor;
	const FVector WielderPoint = GetComponentLocation();
	const float InvMassTarget = EndpointPointInvMass(
		Endpoints->Target, TargetPoint, DirTarget);
	const float InvMassWielder = bSelfWrap
		? 0.0f
		: EndpointPointInvMass(
			Endpoints->Wielder, WielderPoint, DirWielder);
	const float WSum = InvMassTarget + InvMassWielder;
	// Analytic material response and actual point impulse use the same Jacobian. In
	// particular an off-COM wrap includes rotational inverse mass instead of pretending
	// the selected bone's COM translation is the rope attachment response.
	PullDrive.LastTargetShare =
		(WSum > KINDA_SMALL_NUMBER)
			? (InvMassTarget / WSum)
			: 0.0f;

	// ---- 대상 하드 투영(키네마틱 캐릭터 캐리) ----
	// wielder 끝이 무한질량(Anchor — 헬기 등 키네마틱 캐리어)이고 대상이 CMC 캐릭터면, λ 속도 인가만으로는
	// 위치 오차 회수가 bias 상한(TetherMaxBiasSpeed)에 캡혀 캐리어가 그보다 빠를 때 로프가 무한히 늘어난다.
	// wielder 쪽 하드 투영(ConstrainWielderLocation)의 대상 거울: 캡슐을 현 반경 방향으로 부족분만큼 손
	// 쪽으로 스윕 이동해 위치 오차를 같은 프레임에 소거한다(벽에 막히면 잔여가 C로 남아 λ/관측이 받는다).
	// 양끝이 모두 유한질량이면 λ 쌍 인가가 분배를 소유하므로 발동하지 않는다(이중 보정 방지). 탄성 모드
	// (TetherCompliance>0)는 의도적 신장이라 제외.
	if (HoldConfig.bEnforceTargetLengthConstraint
		&& HoldConfig.TetherCompliance <= KINDA_SMALL_NUMBER
		&& bHasLiveConstraint && !bSelfWrap
		&& Endpoints->Target.Kind == ERopeEndpointKind::Character
		&& Endpoints->Wielder.Kind == ERopeEndpointKind::Anchor
		&& Endpoints->Target.Actor && Endpoints->Target.Movement
		&& !LiveOutward.IsNearlyZero()
		&& C > FMath::Max(HoldConfig.LengthConstraintActivationSlop, 0.0f))
	{
		AActor* TargetActor = Endpoints->Target.Actor;
		const FVector OldLoc = TargetActor->GetActorLocation();
		TargetActor->SetActorLocation(OldLoc + LiveOutward * C, /*bSweep*/ true);
		const FVector Applied = TargetActor->GetActorLocation() - OldLoc;
		C = FMath::Max(
			C - static_cast<float>(FVector::DotProduct(Applied, LiveOutward)), 0.0f);
		LengthConstraintState.LastViolation = C;
		// 접지 캐릭터를 유의미한 속도로 들어올렸으면 Walking의 바닥 스냅/Z 삭제가 되돌리기 전에 Falling으로
		// 넘긴다(Launch 관례). 수평 towing(Applied.Z ≈ 0)은 임계 미달로 통과 — 지상 끌기 거동 유지.
		if (Endpoints->Target.Movement->IsMovingOnGround()
			&& Applied.Z > TetherLiftLaunchSpeedConst * DeltaTime)
		{
			Endpoints->Target.Movement->SetMovementMode(MOVE_Falling);
		}
	}

	// Anchor-kind 대상(정적/키네마틱/애니메이션 구동)은 물리 속도가 없어 앵커 점 실측 EMA로 채운다 —
	// 움직이는 오브젝트 towing이 bias 상한과 무관하게 벌어짐 상쇄로 추종된다(정지 앵커는 ≈0 = 무영향;
	// 2차 안전망은 인가 쪽 ClampInjectedVelocity(TetherMaxSpeed) 그대로). SimBody는 반드시 rope
	// attachment point의 vCOM+ω×r를 읽고, Character는 CMC 속도를 읽는다.
	const FVector VelTarget = (Endpoints->Target.Kind == ERopeEndpointKind::Anchor)
			? (bHasLiveConstraint
				? LiveConstraint.PivotVelocity
				: LengthConstraintState.SmoothedAnchorPointVelocity)
			: EndpointVelocityAtPoint(
				Endpoints->Target, TargetPoint);
	const float SepTarget = -static_cast<float>(FVector::DotProduct(VelTarget, DirTarget));
	// Anchor-kind wielder(키네마틱 캐리어)도 대칭으로 손 점 실측 EMA를 쓴다 — 캐리어의 이탈 속도가 λ의
	// 벌어짐 상쇄에 실려 bias 상한(TetherMaxBiasSpeed)과 무관하게 추종된다(정지 소유자는 ≈0 = 무영향).
	const FVector VelWielder = (Endpoints->Wielder.Kind == ERopeEndpointKind::Anchor)
		? LengthConstraintState.SmoothedWielderPointVelocity
		: EndpointVelocityAtPoint(
			Endpoints->Wielder, WielderPoint);
	const float SepWielder = bSelfWrap ? 0.0f
		: -static_cast<float>(FVector::DotProduct(VelWielder, DirWielder));
	const float SepSpeed =
		RopeMovementConstraint::ComputeConstraintSeparatingSpeed(
			bHardWielderAttempt,
			LengthConstraintState.WielderAttemptSeparatingSpeed,
			SepTarget + SepWielder,
			RestRate);

	RopeLengthConstraint::FInput In;
	LengthConstraintState.Backend = bHardWielderAttempt
		? ERopeLengthConstraintBackend::HardReaction
		: ERopeLengthConstraintBackend::Analytic;
	// C is sampled after the hard projection, so its rejected position is already gone and
	// normal hard frames see C=0. Any positive value left here is new live error (for example
	// an external target that moved later in PrePhysics) and must not be hidden by the attempt
	// stamp. The original rejected distance still enters only through its recorded speed.
	In.Violation = C;
	In.SeparatingSpeed = SepSpeed;
	In.EffectiveInverseMass = WSum;
	In.ActivationSlop = FMath::Max(HoldConfig.LengthConstraintActivationSlop, 0.0f);
	// The projection's original error is absent from live C. A later-moving external
	// target can create a new positive residual in the same frame, and that residual needs
	// normal position bias even though a hard attempt stamp also exists.
	In.SettleAlpha =
		(bHardWielderAttempt && C <= KINDA_SMALL_NUMBER)
			? 0.0f
			: RopeTraction::ExpSmoothAlpha(
				HoldConfig.TetherSettleTime, DeltaTime);
	In.MaxBiasSpeed = FMath::Max(HoldConfig.TetherMaxBiasSpeed, 0.0f);
	In.Compliance = HoldConfig.TetherCompliance;
	// A strictly inextensible constraint cannot also cap its reaction under arbitrary
	// kinematic input: exceeding a cap must either stretch, slip, or break. Rigid mode
	// preserves length and reports the full reaction (MaxTetherTension is then an overload
	// threshold); compliant mode may yield and therefore uses the configured force cap.
	In.MaxTension = HoldConfig.TetherCompliance > KINDA_SMALL_NUMBER
		? HoldConfig.MaxTetherTension
		: 0.0f;
	const RopeLengthConstraint::FResult SolveResult =
		RopeLengthConstraint::Solve(In, DeltaTime);
	const float Lambda = SolveResult.Lambda;
	LengthConstraintState.LastLambda = Lambda;

	// 유효 분배 몫(wielder 게이트/디버거 호환) = 역질량비 — λ 발화와 무관하게 이번 프레임 값으로 확정한다.
	if (Lambda <= 0.0f)
	{
		return; // 이미 충분히 접근 중이거나 양끝 다 앵커.
	}

	// ---- 인가: 끝별 ΔV = λ × w(cm/s), 각자 다리 방향 ----
	// 모든 경로가 로프 축(+직교 감쇠) 성분만 건드려 스윙/중력은 보존되고, 결과 속력은 TetherMaxSpeed로
	// 2차 클램프된다. 강체 시뮬 대상은 Chaos가 독점하고, compliant 시뮬 대상과 시뮬 wielder는 실제
	// 물리 임펄스(ΔV×유효질량)를 받는다. 기존 디스패치 골격(ApplyToTetherEndpoint)을 그대로 지나므로
	// 확장 관문(ApplyTractionToReceiver, Amount = ΔV cm/s)도 동일하다.
	const float SpeedCap = FMath::Max(HoldConfig.TetherMaxSpeed, 0.0f);
	// 직교 감쇠 dt 보정: 설정값은 60fps 기준 프레임당 비율 → 유효 비율 = 1−(1−d)^(dt·60). 종전엔 비율을
	// 프레임당 그대로 써 고프레임률일수록 감쇠가 세지는 프레임률 의존 물리였다.
	const float PerpDampCfg = FMath::Clamp(HoldConfig.TetherPerpDamping, 0.0f, 1.0f);
	const float PerpDamp = (PerpDampCfg > 0.0f && PerpDampCfg < 1.0f)
		? (1.0f - FMath::Pow(1.0f - PerpDampCfg, DeltaTime * 60.0f))
		: PerpDampCfg;
	auto ApplySimBody = [&](
		UPrimitiveComponent* Prim,
		FName BoneName,
		const FVector& Dir,
		const FVector& PointWorld,
		float DeltaV,
		float PointInvMass) -> float
	{
		// Compliant 대상 또는 물리 wielder: solver와 같은 attachment-point Jacobian으로
		// ΔV=λ*w를 계산했으므로 실제 로프 임펄스는 정확히 J=λ*d=(ΔV/w)*d다.
		// AddImpulseAtLocation이 ω×r와 토크를 함께 만들며, 관측도 같은 점 속도를 읽는다.
		// (직교 감쇠는 전 축 대상이다 — 수직 성분 제외안은 검토 후 되돌림. 중력 낙하가 감쇠와 평형을
		// 이뤄 저속(≈g·dt/비율)에 갇히는 "무중력" 룩은 이 값의 크기 튜닝으로 대응한다.)
		if (PointInvMass <= KINDA_SMALL_NUMBER)
		{
			return DeltaV;
		}
		const FVector CurVel =
			Prim->GetPhysicsLinearVelocityAtPoint(
				PointWorld, BoneName);
		FVector PointImpulse =
			Dir * (DeltaV / PointInvMass);

		FRopeTetherEndpoint SimEndpoint;
		SimEndpoint.Kind = ERopeEndpointKind::SimBody;
		SimEndpoint.Prim = Prim;
		SimEndpoint.Bone = BoneName;
		RopeTraction::FRopePointMassProperties Body;
		if (BuildPointMassProperties(SimEndpoint, Body))
		{
			if (PerpDamp > 0.0f)
			{
				const FVector PerpVel =
					CurVel -
					Dir * static_cast<float>(
						FVector::DotProduct(CurVel, Dir));
				const float PerpSpeed =
					static_cast<float>(PerpVel.Size());
				if (PerpSpeed > KINDA_SMALL_NUMBER)
				{
					const FVector PerpDir =
						-PerpVel / PerpSpeed;
					const float PerpInvMass =
						RopeTraction::ComputePointInverseMass(
							Body, PointWorld, PerpDir);
					if (PerpInvMass > KINDA_SMALL_NUMBER)
					{
						PointImpulse +=
							PerpDir *
							(PerpSpeed * PerpDamp /
								PerpInvMass);
					}
				}
			}

			// Predict the actual point response (including angular motion), then scale the
			// whole injected impulse if the gameplay safety cap would be exceeded.
			const FVector PredictedDelta =
				RopeTraction::ComputePointVelocityDelta(
					Body, PointWorld, PointImpulse);
			const FVector SafeVelocity =
				RopeTraction::ClampInjectedVelocity(
					CurVel + PredictedDelta,
					CurVel,
					SpeedCap);
			const FVector SafeDelta = SafeVelocity - CurVel;
			const float PredictedSizeSq =
				static_cast<float>(PredictedDelta.SizeSquared());
			if (PredictedSizeSq > SMALL_NUMBER)
			{
				const float ImpulseScale = FMath::Clamp(
					static_cast<float>(
						FVector::DotProduct(
							SafeDelta, PredictedDelta)) /
						PredictedSizeSq,
					0.0f,
					1.0f);
				PointImpulse *= ImpulseScale;
			}
			Prim->AddImpulseAtLocation(
				PointImpulse, PointWorld, BoneName);
		}
		else
		{
			// Defensive fallback for an endpoint whose body vanished after resolution.
			const FVector NewVel =
				RopeTraction::ClampInjectedVelocity(
					CurVel + Dir * DeltaV,
					CurVel,
					SpeedCap);
			Prim->AddImpulse(
				(NewVel - CurVel) / PointInvMass,
				BoneName,
				/*bVelChange*/ false);
		}
		return DeltaV;
	};
	auto ApplyCharacter = [&](UCharacterMovementComponent* Movement, const FVector& Dir, float DeltaV)
	{
		// CMC: 속도 직접 가산(이번 프레임 반영 계약). λ의 위치 회수 항은 MaxBiasSpeed로 유계라
		// 과잉 주입이 없고, 별도 장부/슬랙 브레이크도 필요 없다. (접지 중 수평 투영안은 검토 후 되돌림 —
		// 전 축 주입 유지.)
		const FVector OldVel = Movement->Velocity;
		Movement->Velocity = RopeTraction::ClampInjectedVelocity(OldVel + Dir * DeltaV, OldVel, SpeedCap);
		// 상향 주입이 임계를 넘는 접지 캐릭터는 Falling으로 — Walking은 다음 틱에 Z 속도를 바닥 구속으로
		// 버리므로(들어올리기 무력화) Launch와 같은 관례로 모드를 넘겨야 주입이 살아남는다. 수평 towing은
		// Z 주입 ≈ 0이라 통과.
		if (static_cast<float>(Movement->Velocity.Z - OldVel.Z) > TetherLiftLaunchSpeedConst
			&& Movement->IsMovingOnGround())
		{
			Movement->SetMovementMode(MOVE_Falling);
		}
	};

	auto TractionGate = [this](const FRopeTractionRequest& Req) { return ApplyTractionToReceiver(Req); };
	const FRopeTetherContext Ctx{ Endpoints->Target, Endpoints->Wielder, DeltaTime, TractionGate };

	const float DvTarget = Lambda * InvMassTarget;
	if (DvTarget > KINDA_SMALL_NUMBER)
	{
		ApplyToTetherEndpoint(Ctx, /*bWielderSide*/ false, DirTarget, DvTarget,
			[&](UPrimitiveComponent* P, FName B, const FVector& D, float S)
			{
				return ApplySimBody(
					P, B, D, TargetPoint, S, InvMassTarget);
			},
			[&](UCharacterMovementComponent* M, const FVector& D, float S) { ApplyCharacter(M, D, S); },
			[&](AActor*, const FVector&, float) { /* 앵커 = w 0이라 ΔV도 0 — 도달 불가 */ });
	}
	// The hard movement adapter already removed this exact outward velocity from the
	// Wielder. Applying its analytic share again would create an inward rebound.
	const float DvWielder =
		(bSelfWrap || bHardWielderAttempt) ? 0.0f : Lambda * InvMassWielder;
	if (DvWielder > KINDA_SMALL_NUMBER)
	{
		ApplyToTetherEndpoint(Ctx, /*bWielderSide*/ true, DirWielder, DvWielder,
			[&](UPrimitiveComponent* P, FName B, const FVector& D, float S)
			{
				return ApplySimBody(
					P, B, D, WielderPoint, S, InvMassWielder);
			},
			[&](UCharacterMovementComponent* M, const FVector& D, float S) { ApplyCharacter(M, D, S); },
			[&](AActor*, const FVector&, float) { /* 앵커 무동작 */ });
	}
}

void URopeComponent::UpdatePhysicalTether(UPrimitiveComponent* TargetPrim, FName Bone,
	const FVector& AnchorWorld, const FVector& CornerWorld, float LegRestLen, float DeltaTime)
{
	// (Constraint 테더 — 시뮬 바디 대상 절반) 엔진 물리 제약: [코너의 키네마틱 프록시 ↔ 대상 바디의 앵커
	// 점]을 다리 rest 길이의 구면 리밋으로 묶는다. GT 프레임당 속도 임펄스는 관절체의 "전신 크기 kick →
	// 폭주" vs "본 크기 λ → 견인력 붕괴" 딜레마(2026-07-20 Pierce 7회 반복)에 더해 공중 하중(매달린 프랍)의
	// 부유/진자 펌핑(2026-07-22 PIE)도 못 풀지만, Chaos 제약은 서브스텝에서 중력·관절·지면 접촉과 **함께**
	// 풀리므로 폭주 없는 전신 견인과 진짜 진자 거동이 나온다("하중을 손에 매달기"의 표준 패턴).
	// 제약 프레임을 앵커의 바디-로컬(스켈레탈 = 본 TM, 창 끝 레버 규약)로 잡아 정렬 토크까지 엔진이 푼다.
	AActor* Owner = GetOwner();
	if (!Owner || !TargetPrim)
	{
		return;
	}

	// 대상/본이 바뀌었으면 재생성(앵커 승격/재랩).
	if (PhysicalTetherConstraint
		&& (PhysicalTetherTarget.Get() != TargetPrim || PhysicalTetherBone != Bone))
	{
		TeardownPhysicalTether();
	}

	// 대상 바디-로컬 앵커(제약 Frame2)의 현재값 — 스켈레탈 = 본 TM(창 끝 레버 규약), 컴포넌트 바디 =
	// 컴포넌트 TM. 생성 시 고정되는 값이라, 같은 (대상,본) 안에서 wrap 앵커가 재배치되면(승격/시드 합류)
	// 로프 앵커와 제약 앵커가 어긋나 상시 위반 = 진동이 된다 — 드리프트가 임계를 넘으면 해체하고 아래
	// 생성 블록에서 즉시 재생성한다(정상 상태에선 발화하지 않는 가드).
	const USkeletalMeshComponent* SkelBody = Cast<USkeletalMeshComponent>(TargetPrim);
	FTransform BodyTM = TargetPrim->GetComponentTransform();
	if (SkelBody && !Bone.IsNone())
	{
		const int32 BoneIndex = SkelBody->GetBoneIndex(Bone);
		if (BoneIndex != INDEX_NONE)
		{
			BodyTM = SkelBody->GetBoneTransform(BoneIndex);
		}
	}
	const FVector AnchorLocal = BodyTM.InverseTransformPosition(AnchorWorld);
	// 순간 anchorLocal은 로프 Sim(GPU 지연 미러)과 현재 본 TM의 지연차로 빠른 랙돌에서 프레임마다 크게
	// 흔들린다 — 순간값으로 5cm 가드를 대면 진짜 재배치가 없어도 매 프레임 재생성(thrash)돼 제약이
	// warm start를 못 쌓고 오히려 떨린다(계측: 재생성 70%·힘 0 89%). 그래서 anchorLocal을 EMA로 스무딩해
	// 그 값으로 판정한다: 지연 노이즈는 평균으로 상쇄되고(스무딩값은 고정 앵커 근처에 머묾), 지속적
	// 재배치(시드 합류/승격)만 평균을 옮겨 임계를 넘긴다. 임계도 10cm로 올려 여유를 둔다.
	if (PhysicalTetherConstraint)
	{
		const float SmoothAlpha = RopeTraction::ExpSmoothAlpha(0.12f, DeltaTime); // ≈0.12s 시상수.
		PhysicalTetherSmoothedAnchorLocal = FMath::Lerp(PhysicalTetherSmoothedAnchorLocal, AnchorLocal, SmoothAlpha);
		if (FVector::DistSquared(PhysicalTetherSmoothedAnchorLocal, PhysicalTetherAnchorLocal) > FMath::Square(10.0f))
		{
			TeardownPhysicalTether();
		}
	}

	if (!PhysicalTetherProxy)
	{
		PhysicalTetherProxy = NewObject<USphereComponent>(Owner,
			MakeUniqueObjectName(Owner, USphereComponent::StaticClass(), TEXT("RopeTetherProxy")));
		PhysicalTetherProxy->SetupAttachment(this);
		PhysicalTetherProxy->SetAbsolute(true, true, true); // 월드 배치(로프 컴포넌트 트랜스폼 무관).
		PhysicalTetherProxy->InitSphereRadius(4.0f);
		// 바디는 필요하고(제약의 한쪽) 충돌도 쿼리도 없어야 한다 — PhysicsOnly + 전 채널 무시.
		// 쿼리를 켜면 안 되는 이유: USphereComponent의 오브젝트 타입 기본값은 WorldDynamic이고,
		// 채널 응답 Ignore는 **채널 질의**에만 듣는다(오브젝트 타입 질의는 셰이프의 오브젝트 타입만
		// 본다). 그래서 쿼리가 켜져 있으면 URopeStaticBodyProvider의 OverlapMultiByObjectType 스캔에
		// 잡혀, 코너를 매 프레임 따라다니는 push-out 콜라이더가 되어 제 로프의 wrap 노드를 민다.
		PhysicalTetherProxy->SetCollisionEnabled(ECollisionEnabled::PhysicsOnly);
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
		// A single body plus positional projection can snap/rebound at the moving limit, so
		// component bodies keep projection disabled. This backend is exclusively the
		// inextensible path and must always be a hard Chaos limit. Positive compliance is
		// owned by the common analytic material solver and never reaches this function.
		// The profile must be configured before SetConstrainedComponents initializes Chaos.
		FConstraintProfileProperties& Profile =
			PhysicalTetherConstraint->ConstraintInstance.ProfileInstance;
		if (!SkelBody)
		{
			Profile.bEnableProjection = false;
		}
		Profile.LinearLimit.bSoftConstraint = false;
		Profile.LinearLimit.Stiffness = 0.0f;
		Profile.LinearLimit.Damping = 0.0f;
		Profile.LinearLimit.Restitution = 0.0f;
		PhysicalTetherConstraint->SetConstrainedComponents(PhysicalTetherProxy, NAME_None, TargetPrim, Bone);
		// 제약 프레임 오리진: 프록시 쪽 = 프록시 원점(코너), 대상 쪽 = 앵커의 바디-로컬(위 AnchorLocal —
		// wrap이 얼린 본-로컬 앵커와 같은 규약, 창 끝 레버 포함). 거리 리밋은 이 두 점 사이에 걸린다.
		PhysicalTetherConstraint->ConstraintInstance.SetRefPosition(EConstraintFrame::Frame1, FVector::ZeroVector);
		PhysicalTetherConstraint->ConstraintInstance.SetRefPosition(EConstraintFrame::Frame2, AnchorLocal);
		// 로프는 회전을 구속하지 않는다 — 각도 전부 자유.
		PhysicalTetherConstraint->SetAngularSwing1Limit(ACM_Free, 0.0f);
		PhysicalTetherConstraint->SetAngularSwing2Limit(ACM_Free, 0.0f);
		PhysicalTetherConstraint->SetAngularTwistLimit(ACM_Free, 0.0f);
		PhysicalTetherTarget = TargetPrim;
		PhysicalTetherBone = Bone;
		PhysicalTetherAnchorLocal = AnchorLocal;
		PhysicalTetherSmoothedAnchorLocal = AnchorLocal; // EMA를 생성 앵커로 시드(첫 프레임 가짜 드리프트 방지).
		PhysicalTetherLimit = -1.0f; // 아래에서 강제 갱신.
		UE_LOG(LogDynamicRope, Verbose, TEXT("[%s] physical tether created: %s/%s"),
			*GetName(), *GetNameSafe(TargetPrim), *Bone.ToString());
	}

	// Spherical distance limit = material leg length. Node zero is an exact zero-radius
	// contract, not a 1 cm convenience slack: represent it with XYZ Locked because Chaos
	// Limited(0) is not a portable zero-distance limit across solver paths.
	const float Limit = FMath::Max(LegRestLen, 0.0f);
	const bool bWasZeroLimit =
		PhysicalTetherLimit >= 0.0f &&
		PhysicalTetherLimit <= KINDA_SMALL_NUMBER;
	const bool bIsZeroLimit =
		Limit <= KINDA_SMALL_NUMBER;
	if (bWasZeroLimit != bIsZeroLimit ||
		!FMath::IsNearlyEqual(PhysicalTetherLimit, Limit, 0.5f))
	{
		const ELinearConstraintMotion Motion =
			bIsZeroLimit
				? LCM_Locked
				: LCM_Limited;
		PhysicalTetherConstraint->SetLinearXLimit(Motion, Limit);
		PhysicalTetherConstraint->SetLinearYLimit(Motion, Limit);
		PhysicalTetherConstraint->SetLinearZLimit(Motion, Limit);
		PhysicalTetherLimit = Limit;
	}

	SamplePhysicalTetherForce(DeltaTime);
}

void URopeComponent::SamplePhysicalTetherForce(float DeltaTime)
{
	if (!PhysicalTetherConstraint || DeltaTime <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	// Physical backend is authoritative for this frame. Store its measured force in the
	// common lambda channel; do not add/max an analytic hard-reaction estimate on top.
	LengthConstraintState.Backend = ERopeLengthConstraintBackend::Chaos;
	FVector LinearForce = FVector::ZeroVector;
	FVector AngularForce = FVector::ZeroVector;
	PhysicalTetherConstraint->GetConstraintForce(LinearForce, AngularForce);
	LengthConstraintState.LastLambda =
		static_cast<float>(LinearForce.Size()) * DeltaTime;
	LengthConstraintState.LastLambdaDt = DeltaTime;
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
	PhysicalTetherPrePhysicsFrame = MAX_uint64;
	PhysicalTetherAnchorLocal = FVector::ZeroVector;
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

