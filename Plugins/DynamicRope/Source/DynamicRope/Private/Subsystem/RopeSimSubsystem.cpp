// Copyright Epic Games, Inc. All Rights Reserved.

#include "Subsystem/RopeSimSubsystem.h"
#include "RopeComponent.h"
#include "DynamicRopeLog.h"
// RopeSolverSubsteps
#include "Solver/RopeXPBDSolver.h"
// IRopeCollider::GetGPUCapsule
#include "Collision/RopeCollider.h"
// IRopeColliderProvider (중앙 collider gather)
#include "Collision/RopeColliderProvider.h"
// FRopeGPUSolver / FRopeGPUResidentStep / FRopeGPUCapsule (DynamicRopeShaders 모듈)
#include "RopeGPUSolver.h"
// RopeGDF::RegisterSolver / SetGDFActiveCount (GDF 통합 경로)
#include "RopeGPUSolverRegistry.h"
// StaticBodyControllerClass / StaticBodyMaxColliders(자동 스폰)
#include "Settings/DynamicRopeSettings.h"
// ARopeController(정적 바디 프로바이더 호스트)
#include "Collision/RopeController.h"
// 기본 클래스 스폰 시 MaxColliders 주입
#include "Collision/RopeStaticBodyProvider.h"
#include "Engine/World.h"
// FSceneInterface (씬→솔버 등록 키)
#include "SceneInterface.h"
// AActor::GetOwner (provider 소스 필터링)
#include "GameFramework/Actor.h"
#include "Camera/PlayerCameraManager.h"
#include "Kismet/GameplayStatics.h"
#include "Components/ActorComponent.h"
// 틱 선행조건(애니 평가 이후 보장)
#include "Components/SkeletalMeshComponent.h"
#include "Async/ParallelFor.h"
#include "Debug/RopeStats.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"
// GDynamicRHI
#include "RHI.h"
// FApp::CanEverRender
#include "Misc/App.h"
// TObjectIterator(자동 스폰 전 기존 프로바이더 스캔)
#include "UObject/UObjectIterator.h"
// GEngine->AddOnScreenDebugMessage(중복 경고)
#include "Engine/Engine.h"
// TAutoConsoleVariable(CPU 솔브 강제 토글)
#include "HAL/IConsoleManager.h"

namespace
{
	// 디버그/프로파일링용 CPU 솔브 강제 토글. 1이면 렌더 가능한 RHI가 있어도 GPU 상주 경로를 끄고 CPU
	// 폴백 솔버+감지로 내려간다(솔브·감지·핸드오프 동기가 함께 CPU 경로로 일관 전환 — 튜브는 로프별
	// bGpuSteppedThisFrame가 false가 되어 CPU 미러 센터라인으로 자동 폴백). GPU 대비 검증/성능 비교용. 기본 0.
	static TAutoConsoleVariable<int32> CVarForceCPUSolve(
		TEXT("r.DynamicRope.ForceCPUSolve"),
		0,
		TEXT("1이면 GPU가 가용해도 로프 솔브/감지를 CPU 경로로 강제한다(디버그·비교용). 0=자동 선택(기본)."),
		ECVF_Default);

	// G4: GPU가 런타임 유일 경로. 렌더 가능한 RHI가 있으면 GPU 상주 솔브+감지, 없으면(쿡/-nullrhi/
	// 서버 빌드) 자동으로 CPU 솔브+감지로 폴백한다. 유일한 클라이언트 토글은 위 r.DynamicRope.ForceCPUSolve
	// (디버그용 CPU 강제)뿐 — 평상시엔 GPU가 THE 경로다.
	// FRopeXPBDSolver는 이 폴백과 패리티 테스트를 위해 유지된다(런타임 클라이언트에선 사실상 미사용).
	bool RopeGpuRuntimeAvailable()
	{
		// CPU 강제 토글이 켜져 있으면 GPU 가용 여부와 무관하게 CPU 폴백으로 내려간다.
		if (CVarForceCPUSolve.GetValueOnGameThread() != 0)
		{
			return false;
		}
		// 렌더 가능 RHI + SM5 이상(커널이 SM5 가드로만 컴파일된다) — 판정은 RopeGPU::IsRuntimeSupported가
		// 단일 소스다(씬 프록시의 GPU 튜브 게이트와 같은 함수를 본다).
		return RopeGPU::IsRuntimeSupported();
	}

	// 중복 월드-정적 프로바이더 경고: 로그 + (에디터/개발 빌드)화면 메시지. "월드당 최대 1개" 불변식을
	// 조용히 어기지 않게 눈에 띄게 알린다 — 두 번째 프로바이더는 무시되므로 사용자가 이유를 알아야 한다.
	void WarnDuplicateWorldStaticProvider(const AActor* Offender)
	{
		UE_LOG(LogRopeCollision, Warning,
			TEXT("정적 월드 콜라이더 프로바이더가 이미 존재합니다 — %s의 중복 프로바이더는 무시됩니다(월드당 1개만 사용)."),
			*GetNameSafe(Offender));
#if !UE_BUILD_SHIPPING
		if (GEngine)
		{
			// 키를 고정(GetTypeHash 대신 상수)해 매 프레임이 아닌 이벤트당 1회만 갱신되게 한다.
			GEngine->AddOnScreenDebugMessage(uint64(0x0D0ED1CA), 8.0f, FColor::Yellow,
				FString::Printf(TEXT("[DynamicRope] 중복 정적 바디 프로바이더 무시됨(%s) — 월드당 1개만 사용됩니다."),
					*GetNameSafe(Offender)));
		}
#endif
	}

	// SDF collider view(런타임 Collision) → GPU 업로드용 SDF collider(Shaders) 평탄 복사.
	// 필드가 늘면 여기 한 곳만 갱신하면 된다(과거엔 Tick 루프 안에 흩어져 있던 17줄).
	FRopeGPUSDFCollider MakeGpuSdf(const FRopeSDFColliderView& View)
	{
		FRopeGPUSDFCollider Sdf;
		// 코드 바이트 블롭(업로드 평탄화 시 dequant)
		Sdf.Distances       = View.Distances;
		Sdf.BytesPerCode    = View.BytesPerCode;
		Sdf.NarrowBandInner = View.NarrowBandInner;
		Sdf.NarrowBandOuter = View.NarrowBandOuter;
		Sdf.ResX            = View.ResX;
		Sdf.ResY            = View.ResY;
		Sdf.ResZ            = View.ResZ;
		Sdf.LocalMin        = View.LocalMin;
		Sdf.LocalSize       = View.LocalSize;
		Sdf.BoneToWorld     = View.BoneToWorld;
		// GPU CCD/표면속도 드래그.
		Sdf.PrevBoneToWorld = View.PrevBoneToWorld;
		Sdf.InvDeltaTime    = View.InvDeltaTime;
		Sdf.VolumeKey       = View.VolumeKey;
		return Sdf;
	}

	// 솔버 시드/파라미터를 Step에 채운다(collider·override·whip 패킹은 호출부에서 추가).
	void SeedResidentStep(FRopeGPUResidentStep& Step, uint32 RopeId, uint32 Generation,
		const FRopeSimState& S, const FRopeSolverConfig& Cfg, const FRopeSubstepSchedule& Schedule)
	{
		Step.RopeId            = RopeId;
		Step.Generation        = Generation;
		Step.NumNodes          = S.Num();
		Step.SeedPositions     = S.Positions;
		Step.SeedPrevPositions = S.PrevPositions;
		Step.InvMass           = S.InvMass;
		Step.SegmentLength     = S.SegmentLength;
		Step.bStartPinned      = S.bStartPinned;
		Step.StartPinPrev      = S.StartPinPrev;
		Step.StartPinTarget    = S.StartPinTarget;
		Step.StretchCompliance = Cfg.StretchCompliance;
		Step.MaxStretchRatio   = Cfg.MaxStretchRatio;
		Step.BendCompliance    = Cfg.BendCompliance;
		Step.BendReleaseRatio  = Cfg.BendReleaseRatio;
		Step.BendFullRatio     = Cfg.BendFullRatio;
		Step.Damping           = Cfg.Damping;
		Step.Iterations        = Cfg.Iterations;
		Step.CollisionPasses   = Cfg.CollisionPassesPerSubstep;
		Step.Gravity           = Cfg.Gravity;
		// CollisionRadius는 auto(0=렌더 Radius) 해석이 필요해 호출부가 Rope.GetEffectiveCollisionRadius로
		// 덮는다(bUseWorldGDF도 컴포넌트 직속으로 이사해 호출부 소관 — 표면 감사 CL-4).
		Step.CollisionRadius   = Cfg.CollisionRadius;
		Step.Friction          = Cfg.Friction;
		Step.TipFrictionScale  = Cfg.TipFrictionScale;
		Step.SweepStep         = Cfg.SweepStep;
		Step.MaxSweepSamples   = Cfg.MaxSweepSamples;
		Step.NumSub            = Schedule.NumSub;
		Step.FixedDt           = Schedule.FixedDt;
	}
}

// FRopeNodeOverrideFrame(Core 모듈) 비트는 ERopeGPUOverride(Shaders 모듈)와 수치 1:1이어야 한다 —
// Core가 Shaders에 의존하지 않으려고 상수를 미러로 두었고, 여기(둘 다 보이는 곳)서 검증한다.
static_assert(RopeNodeOverride::Position == static_cast<uint8>(ERopeGPUOverride::Position)
	&& RopeNodeOverride::Prev == static_cast<uint8>(ERopeGPUOverride::Prev)
	&& RopeNodeOverride::PrevFromPosition == static_cast<uint8>(ERopeGPUOverride::PrevFromPosition)
	&& RopeNodeOverride::InvMass == static_cast<uint8>(ERopeGPUOverride::InvMass),
	"RopeNodeOverride bits must mirror ERopeGPUOverride");

void URopeSimSubsystem::RegisterRope(URopeComponent* Rope)
{
	if (Rope)
	{
		if (bTickingRopes)
		{
			// 틱 순회 중 재진입(핸들러가 로프 액터 스폰) — 변형을 미룬다(헤더 bTickingRopes 주석).
			DeferredRopeUnregister.RemoveSingleSwap(Rope);
			DeferredRopeRegister.AddUnique(Rope);
			return;
		}
		Ropes.AddUnique(Rope);
		// GPU 상주 자원의 주인을 ID로도 기록한다 — 컴포넌트가 정식 해제 없이 사라졌을 때 회수할 유일한 단서.
		RegisteredRopeIds.Add(Rope->GetUniqueID());
		// 손 핀(소켓 부착)이 소유 캐릭터 포즈를 따르므로.
		SetAnimPrerequisites(Rope, /*bAdd*/ true);
		UE_LOG(LogDynamicRope, Verbose, TEXT("RegisterRope: %s (%d total)"), *Rope->GetName(), Ropes.Num());
	}
}

void URopeSimSubsystem::UnregisterRope(URopeComponent* Rope)
{
	if (bTickingRopes)
	{
		// 틱 순회 중 재진입(핸들러가 로프 액터 파괴) — 실제 제거는 ApplyDeferredRopeChanges로 미룬다. 이번
		// 프레임 남은 순회는 IsValid 가드가 이 로프(파괴 → pending-kill)를 건너뛴다(헤더 bTickingRopes 주석).
		if (Rope)
		{
			DeferredRopeRegister.RemoveSingleSwap(Rope);
			DeferredRopeUnregister.AddUnique(Rope);
		}
		return;
	}
	Ropes.RemoveSingleSwap(Rope);
	if (Rope)
	{
		SetAnimPrerequisites(Rope, /*bAdd*/ false);
		// GPU 상주 버퍼/리드백 해제(렌더 스레드에서). 두 캐시와 ID 대장에서도 제거한다
		// (GpuLatestContacts는 종전에 빠져 있어 죽은 로프의 접촉 스냅샷이 월드 내내 남았다).
		const uint32 RopeId = Rope->GetUniqueID();
		GpuSolver.ReleaseRope(RopeId);
		GpuLatest.Remove(RopeId);
		GpuLatestContacts.Remove(RopeId);
		PendingSimTimeRefund.Remove(RopeId);
		RegisteredRopeIds.Remove(RopeId);
	}
	UE_LOG(LogDynamicRope, Verbose, TEXT("UnregisterRope: %s (%d remaining)"),
		Rope ? *Rope->GetName() : TEXT("null"), Ropes.Num());
}

void URopeSimSubsystem::ReleaseGpuResourcesForDeadRopes()
{
	if (RegisteredRopeIds.Num() == 0)
	{
		return;
	}

	// 살아 있는 로프의 ID 집합을 만들고, 대장에만 남은 ID = 정식 해제를 못 거친 로프로 본다.
	TSet<uint32> LiveIds;
	LiveIds.Reserve(Ropes.Num());
	for (const TObjectPtr<URopeComponent>& Rope : Ropes)
	{
		if (URopeComponent* Live = Rope.Get())
		{
			LiveIds.Add(Live->GetUniqueID());
		}
	}

	for (auto It = RegisteredRopeIds.CreateIterator(); It; ++It)
	{
		const uint32 RopeId = *It;
		if (LiveIds.Contains(RopeId))
		{
			continue;
		}
		// 애니 선행조건은 여기서 못 푼다(컴포넌트가 이미 없어 소유 메시를 되짚을 수 없다). FTickPrerequisite는
		// weak라 죽은 메시 항목은 자동으로 스킵되므로 남아도 무해하고, 메시가 살아 있는 경우는 컴포넌트가
		// 정식 EndPlay를 거쳤다는 뜻이라 이 경로로 오지 않는다.
		UE_LOG(LogDynamicRope, Verbose,
			TEXT("ReleaseGpuResourcesForDeadRopes: RopeId %u — 정식 해제 없이 사라진 로프의 GPU 자원 회수."), RopeId);
		GpuSolver.ReleaseRope(RopeId);
		GpuLatest.Remove(RopeId);
		GpuLatestContacts.Remove(RopeId);
		PendingSimTimeRefund.Remove(RopeId);
		It.RemoveCurrent();
	}
}

void URopeSimSubsystem::ApplyDeferredRopeChanges()
{
	// 순서: 해제 먼저, 등록 나중(같은 틱에 스폰+파괴된 로프도 최종 상태로 수렴). bTickingRopes는 이미
	// false라 아래 호출은 실제 Ropes 변형/GPU 해제를 수행한다(Register/Unregister는 델리게이트를 쏘지
	// 않으므로 여기서 추가 재진입은 없다).
	if (DeferredRopeUnregister.Num() > 0)
	{
		TArray<URopeComponent*> ToUnregister = MoveTemp(DeferredRopeUnregister);
		DeferredRopeUnregister.Reset();
		for (URopeComponent* Rope : ToUnregister)
		{
			UnregisterRope(Rope);
		}
	}
	if (DeferredRopeRegister.Num() > 0)
	{
		TArray<URopeComponent*> ToRegister = MoveTemp(DeferredRopeRegister);
		DeferredRopeRegister.Reset();
		for (URopeComponent* Rope : ToRegister)
		{
			RegisterRope(Rope);
		}
	}
}

URopeSimSubsystem* URopeSimSubsystem::Get(const UWorld* World)
{
	return World ? World->GetSubsystem<URopeSimSubsystem>() : nullptr;
}

void URopeSimSubsystem::RegisterColliderProvider(UActorComponent* Provider)
{
	if (!Provider)
	{
		return;
	}

	// 중복 방지 백스톱(조각 2): 월드-정적 프로바이더는 월드당 1개만. 이미 등록된 게 있으면 두 번째는
	// 거부 + 경고. 소스(수동 배치/자동 스폰/런타임)와 무관하게 인터페이스 기반으로 불변식을 강제한다.
	// 우선순위는 "먼저 등록된 것이 이긴다"(자동 스폰은 조각 1에서 기존 것에 양보하므로 수동이 이긴다).
	if (const IRopeColliderProvider* Incoming = Cast<IRopeColliderProvider>(Provider))
	{
		if (Incoming->ProvidesWorldStaticColliders())
		{
			for (const TObjectPtr<UActorComponent>& Existing : ColliderProviders)
			{
				const IRopeColliderProvider* E = Cast<IRopeColliderProvider>(Existing);
				if (E && E->ProvidesWorldStaticColliders())
				{
					WarnDuplicateWorldStaticProvider(Provider->GetOwner());
					// 등록 거부 — 이 프로바이더의 GatherColliders는 호출되지 않는다.
					return;
				}
			}
		}
	}

	ColliderProviders.AddUnique(Provider);
	// 본 콜라이더(capsule/SDF)가 소유 캐릭터 포즈를 읽으므로.
	SetAnimPrerequisites(Provider, /*bAdd*/ true);
	UE_LOG(LogRopeCollision, Verbose, TEXT("RegisterColliderProvider: %s (%d total)"),
		*Provider->GetName(), ColliderProviders.Num());
}

void URopeSimSubsystem::UnregisterColliderProvider(UActorComponent* Provider)
{
	ColliderProviders.RemoveSingleSwap(Provider);
	SetAnimPrerequisites(Provider, /*bAdd*/ false);
}

void URopeSimSubsystem::SetAnimPrerequisites(const UActorComponent* Source, bool bAdd)
{
	// "애니 평가 이후 로프 시뮬" 보장: 소스 컴포넌트 소유 액터의 스켈레탈 메시 틱을 SimTickFunction의
	// 선행조건으로 건다. 메시 틱 완료는 병렬 애니 완료 태스크를 DontCompleteUntil로 물고 있으므로
	// (SkeletalMeshComponent::DispatchParallelEvaluationTasks) 선행조건만으로 이번 프레임 포즈(버퍼
	// 플립)까지 보장된다. 같은 메시가 로프/provider 양쪽에서 중복 등록돼도 AddPrerequisite는 유니크.
	const AActor* Owner = Source ? Source->GetOwner() : nullptr;
	if (!Owner)
	{
		return;
	}
	TInlineComponentArray<USkeletalMeshComponent*> Meshes(Owner);
	for (USkeletalMeshComponent* Mesh : Meshes)
	{
		if (!Mesh)
		{
			continue;
		}
		if (bAdd)
		{
			// 첫 소비자일 때만 실제로 건다(AddPrerequisite 자체는 유니크라 중복 호출이 무해하지만,
			// 세지 않으면 해제 때 남은 소비자 몫까지 지워진다 — 헤더 주석).
			int32& RefCount = AnimPrereqRefCount.FindOrAdd(Mesh);
			if (++RefCount == 1)
			{
				SimTickFunction.AddPrerequisite(Mesh, Mesh->PrimaryComponentTick);
			}
		}
		else if (int32* RefCount = AnimPrereqRefCount.Find(Mesh))
		{
			// 마지막 소비자가 빠질 때만 해제.
			if (--(*RefCount) <= 0)
			{
				AnimPrereqRefCount.Remove(Mesh);
				SimTickFunction.RemovePrerequisite(Mesh, Mesh->PrimaryComponentTick);
			}
		}
	}

	if (bAdd)
	{
		// 액터가 정식 해제 없이 죽으면 키가 만료된 채 카운트만 남는다. 선행조건 자체는 weak라 무해하지만
		// 맵이 무한정 자라지 않도록 add 때 한 번씩 청소한다(해제 경로는 비용을 늘리지 않는다).
		for (auto It = AnimPrereqRefCount.CreateIterator(); It; ++It)
		{
			if (!It->Key.IsValid())
			{
				It.RemoveCurrent();
			}
		}
	}
}

FBox URopeSimSubsystem::ComputeRopeQueryBounds(const URopeComponent& Rope, bool bIncludeAimRay)
{
	const bool bHasAimRayBounds = Rope.SimFrame.AimRayColliderQueryBounds.IsValid != 0;
	const bool bHasLockedTargetBounds = Rope.AimTargeting.IsLockActive(Rope.Phase) &&
		Rope.SimFrame.LockedTargetColliderQueryBounds.IsValid;
	// 조준 region 요청인데 ray/활성 target 둘 다 없으면 수집 자체가 필요 없다 — 무효 박스(= 목록 비움).
	if (bIncludeAimRay && !bHasAimRayBounds && !bHasLockedTargetBounds)
	{
		return FBox(ForceInit);
	}

	// 로프 tight AABB(Pos∪Prev — 프레임 모션 포함) + 마진. provider region과 per-rope collider 컬링이
	// 이 동일 박스를 공유한다(GatherCollidersForRope / BuildFrameColliders 양쪽에서 호출).
	FBox RopeBounds(ForceInit);
	// 예측 접촉(전방 외삽) 여유 계산용 — 이번 프레임 최대 노드 변위.
	float MaxFrameDispSq = 0.0f;
	for (int32 i = 0; i < Rope.Sim.Num(); ++i)
	{
		RopeBounds += Rope.Sim.Positions[i];
		RopeBounds += Rope.Sim.PrevPositions[i];
		MaxFrameDispSq = FMath::Max(MaxFrameDispSq,
			static_cast<float>(FVector::DistSquared(Rope.Sim.Positions[i], Rope.Sim.PrevPositions[i])));
	}
	const float BaseMargin = Rope.GetEffectiveCollisionRadius() + Rope.GetEffectiveContactQueryRadius()
		+ FMath::Max(2.0f * Rope.Sim.SegmentLength, 50.0f);
	const float PredictiveMotionMargin = FMath::Sqrt(MaxFrameDispSq)
		* FMath::Max(Rope.DetectConfig.PredictiveContactFrames, 1.0f);
	const float QueryMargin = BaseMargin + PredictiveMotionMargin;
	if (RopeBounds.IsValid)
	{
		// 여유: 접촉 질의 반경 + 스윕 여유 + 예측 접촉의 전방 외삽 거리(프레임 변위 × 예측 프레임).
		// 넉넉히 잡는다 — 과대 컬링 여유는 안전(콜라이더가 몇 개 더 실릴 뿐).
		RopeBounds = RopeBounds.ExpandBy(QueryMargin);
	}
	if (bIncludeAimRay)
	{
		// 조준이 끝난 뒤 active aim lock만 남은 프레임에는 로프↔대상 사이의 거대한 AABB를 만들지 않고
		// 직전 target collider bounds 주변만 재수집한다. 결과는 component target 필터를 거쳐 승격된다.
		if (!bHasAimRayBounds && bHasLockedTargetBounds)
		{
			return Rope.SimFrame.LockedTargetColliderQueryBounds.ExpandBy(QueryMargin);
		}
		// 조준 region 전용: preview ray는 현재 rope centerline과 떨어진 곳을 지나갈 수 있다. 이 구간을
		// 합치지 않으면 ray가 SDF를 관통해도 해당 collider가 조준 목록에 없어 cyan miss가 된다.
		// 로프 주변까지 함께 덮는 합집합이라, 조준 질의(hit 판정/preview 아크 탐색)가 보는 범위는
		// 분리 이전과 같다 — 좁아지는 것은 물리·디버그가 쓰는 FrameColliders 쪽뿐이다.
		RopeBounds += Rope.SimFrame.AimRayColliderQueryBounds.Min;
		RopeBounds += Rope.SimFrame.AimRayColliderQueryBounds.Max;
		if (bHasLockedTargetBounds)
		{
			RopeBounds += Rope.SimFrame.LockedTargetColliderQueryBounds.Min;
			RopeBounds += Rope.SimFrame.LockedTargetColliderQueryBounds.Max;
		}
	}
	return RopeBounds;
}

void URopeSimSubsystem::BuildFrameColliders()
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_BuildColliders);
	FrameProviders.Reset();

	// 활성 영역(region) 리스트 — 앞쪽 N개가 물리 region(Ropes 인덱스와 1:1), 뒤쪽 N개가 같은 로프의
	// 조준 region(AimRegionIndexOf). 무효/빈 로프와 조준 중이 아닌 로프는 !IsValid 박스로 자리를 유지한다 —
	// provider가 돌려주는 region 매핑 인덱스가 이 인덱스와 그대로 대응하게 한다(provider는 !IsValid를
	// 건너뛴다). bounds-aware provider(정적 바디)는 이 리스트로 멀리 동떨어진 로프 사이 빈 공간을
	// 스캔에서 배제한다. 아래 배정(GatherCollidersForRope)과 동일 박스(단일 소스).
	FrameRopeRegions.Reset();
	FrameRopeRegions.Reserve(Ropes.Num() * 2);
	for (URopeComponent* Rope : Ropes)
	{
		FrameRopeRegions.Add(IsValid(Rope) ? ComputeRopeQueryBounds(*Rope) : FBox(ForceInit));
	}
	for (URopeComponent* Rope : Ropes)
	{
		FrameRopeRegions.Add(IsValid(Rope) ? ComputeRopeQueryBounds(*Rope, /*bIncludeAimRay*/ true) : FBox(ForceInit));
	}

	// region 처리 우선순위: 활성 로프 먼저, 그리고 물리 region이 조준 region보다 먼저. 전역 추출 상한이
	// 있는 provider(정적 바디)가 선착순으로 예산을 소진하므로, 상한이 걸리는 프레임에는 뒤 순서 region이
	// 스캔을 못 받는다 — 그때 굶는 쪽이 "실제로 시뮬되는 로프"가 되지 않게 순서만 재배열한다
	// (인덱스 불변 → 매핑 무영향). 조준 region은 HUD/preview 표시용이라 물리보다 뒤로 미룬다.
	// 키: 0 = 사용 중 페이즈(Flight~Releasing), 1 = Free 깨어있음, 2 = Free 슬립, 3 = 무효 region.
	// 조준 region은 여기에 +4(무효는 그대로 7)로, 전체 물리 region 뒤에 놓인다.
	FrameRegionGatherOrder.Reset();
	FrameRegionGatherOrder.Reserve(FrameRopeRegions.Num());
	for (int32 r = 0; r < FrameRopeRegions.Num(); ++r)
	{
		FrameRegionGatherOrder.Add(r);
	}
	auto RegionPriority = [this](int32 RegionIndex) -> int32
	{
		const bool bAimRegion = RegionIndex >= Ropes.Num();
		const int32 AimOffset = bAimRegion ? 4 : 0;
		if (!FrameRopeRegions[RegionIndex].IsValid)
		{
			return 3 + AimOffset;
		}
		const URopeComponent* Rope = Ropes[bAimRegion ? RegionIndex - Ropes.Num() : RegionIndex];
		if (!IsValid(Rope) || Rope->GetPhase() != ERopePhase::Free)
		{
			return (IsValid(Rope) ? 0 : 3) + AimOffset;
		}
		return (Rope->IsSleeping() ? 2 : 1) + AimOffset;
	};
	FrameRegionGatherOrder.StableSort([&RegionPriority](int32 A, int32 B)
	{
		return RegionPriority(A) < RegionPriority(B);
	});

	// 등록된 provider마다 1회 gather(프레임당 1회 — 로프 수와 무관). 죽은 provider는 정리.
	for (int32 i = ColliderProviders.Num() - 1; i >= 0; --i)
	{
		UActorComponent* Comp = ColliderProviders[i];
		if (!IsValid(Comp))
		{
			ColliderProviders.RemoveAtSwap(i);
			continue;
		}
		IRopeColliderProvider* Provider = Cast<IRopeColliderProvider>(Comp);
		if (!Provider)
		{
			continue;
		}
		FRopeColliderGatherContext Gather;
		Gather.RopeRegions = FrameRopeRegions;
		Gather.NumPhysicsRegions = Ropes.Num();
		Gather.RegionGatherOrder = FrameRegionGatherOrder;
		Provider->GatherColliders(Gather);
		if (Gather.Colliders.Num() == 0)
		{
			continue;
		}

		FFrameProviderColliders FP;
		FP.Owner = Comp->GetOwner();
		// 정적 월드 provider는 소유자 제외 면제.
		FP.bWorldStatic = Provider->ProvidesWorldStaticColliders();
		FP.Colliders = MoveTemp(Gather.Colliders);
		// 콜라이더별 출처 액터도 길이가 맞을 때만 신뢰한다 — 어긋나면 인덱스가 엉켜 엉뚱한 콜라이더를
		// 제외하게 되므로, 빈 채로 두고 provider 단위 판정으로 폴백한다.
		if (Gather.ColliderSourceActors.Num() == FP.Colliders.Num())
		{
			FP.SourceActors = MoveTemp(Gather.ColliderSourceActors);
		}
		// region 매핑은 길이가 로프 수와 일치할 때만 신뢰(불일치 = provider 버그 → bounds 재-컬 폴백으로 강등).
		FP.bHasRegionMapping = Gather.bHasRegionMapping
			&& Gather.RegionColliderIndices.Num() == FrameRopeRegions.Num();
		if (FP.bHasRegionMapping)
		{
			FP.RegionIndices = MoveTemp(Gather.RegionColliderIndices);
		}
		else
		{
			// 폴백 경로 전용: collider별 월드 bounds를 프레임당 1회 캐시 — 로프별 재-컬이 로프 수만큼
			// 가상 호출로 재계산하지 않게.
			FP.Bounds.Reserve(FP.Colliders.Num());
			for (const IRopeCollider* Collider : FP.Colliders)
			{
				FP.Bounds.Add(Collider ? Collider->GetWorldBounds() : FBox(ForceInit));
			}
		}
		FrameProviders.Add(MoveTemp(FP));
	}
}

void URopeSimSubsystem::GatherCollidersForRope(const URopeComponent& Rope, int32 RegionIndex, TArray<IRopeCollider*>& OutColliders) const
{
	OutColliders.Reset();

	// 기본: 월드의 모든 provider와 충돌하되 자기 owner(던진 본인) provider는 제외(throw 시 self-tangle 방지).
	// 다른 액터 body 잡기(cross-actor)는 그 액터가 "전체"에 포함되므로 자동. owner 충돌이 필요하면 옵트인.
	const AActor* OwnerToExclude = Rope.bIncludeOwnerColliders ? nullptr : Rope.GetOwner();

	// 거리 컬링: 로프 AABB(Pos∪Prev — 프레임 모션 포함)와 안 겹치는 collider는 아예 안 싣는다.
	// CPU 솔버는 자체 broad-phase가 또 있지만, GPU 커널은 콜라이더 전량을 노드마다 루프하므로
	// 여기서 거르는 것이 스케일링의 핵심이다(멀리 있는 캐릭터들의 캡슐/SDF가 스텝에 안 실림).
	// 기본 경로는 provider가 gather 때 함께 돌려준 region 매핑을 그대로 소비한다(재-컬 없음 —
	// 2026-07 수집 방식 변경). BuildFrameColliders가 provider에 넘긴 region과 동일 박스(단일 소스).
	const FBox RopeBounds = FrameRopeRegions.IsValidIndex(RegionIndex) ? FrameRopeRegions[RegionIndex] : FBox(ForceInit);
	const bool bCull = RopeBounds.IsValid != 0;

	// 로프별 정적 월드 콜라이더 예산. 전역 추출 상한(StaticBodyMaxColliders)과 별개로, 이 로프가 솔브에
	// 실을 정적 월드 콜라이더 수를 로프마다 독립으로 제한한다(멀리 있는 로프가 이 로프 예산을 못 먹음).
	// 스켈레톤 콜라이더(캡슐/SDF)는 본 수로 자연 제한되고 wrap의 핵심이라 예산 대상에서 제외 — 바로 OutColliders로.
	const UDynamicRopeSettings* Settings = UDynamicRopeSettings::Get();
	const int32 PerRopeBudget = Settings ? FMath::Max(1, Settings->StaticBodyMaxCollidersPerRope) : 32;

	// 정적 월드 후보는 따로 모아 예산 초과 시 "가장 먼 것"부터 버린다(스켈레톤은 위에서 이미 무조건 포함).
	TArray<IRopeCollider*> WorldStaticCandidates;

	// 콜라이더(=바디) 단위 소유자 제외. 정적 월드 provider는 아래에서 provider 단위 제외를 면제받는데,
	// 그 면제가 노리는 것은 "바닥/기둥 같은 월드 지오메트리"뿐이다. 같은 provider가 월드를 훑다가 로프
	// 소유 액터에 붙은 셰이프(테더 프록시·팁 메쉬·든 무기 등)까지 잡으면, 그것은 로프를 따라다니며 제
	// 로프를 미는 push-out 콜라이더가 된다 — 출처 액터로 그런 것만 골라 뺀다. 출처를 안 주는 provider는
	// 빈 배열이라 항상 false(= 기존 provider 단위 판정 그대로).
	auto IsOwnBodyCollider = [OwnerToExclude](const FFrameProviderColliders& P, int32 Index)
	{
		return RopeColliderGather::IsExcludedOwnerBody(P.SourceActors, Index, OwnerToExclude);
	};

	for (const FFrameProviderColliders& FP : FrameProviders)
	{
		// 자기 owner provider 제외 — 단 정적 월드 provider는 면제(정적 월드는 "던진 본인의 몸"이 아니므로,
		// 로프 소유 액터에 붙였다는 이유로 월드 충돌이 사라지면 안 된다). 면제분에 섞인 자기 몸 셰이프는
		// 위 IsOwnBodyCollider가 콜라이더 단위로 걸러낸다.
		if (!FP.bWorldStatic && FP.Owner == OwnerToExclude && OwnerToExclude != nullptr)
		{
			continue;
		}
		if (!bCull)
		{
			// region 없는 로프(빈 sim 등) → 전체 폴백(예산 우회, 드묾 — 기존 동작 유지).
			for (int32 c = 0; c < FP.Colliders.Num(); ++c)
			{
				if (FP.Colliders[c] && !IsOwnBodyCollider(FP, c))
				{
					OutColliders.Add(FP.Colliders[c]);
				}
			}
			continue;
		}

		// 기본 경로: provider가 만든 region(=이 로프) 매핑 소비 — bounds 재테스트 없음.
		if (FP.bHasRegionMapping)
		{
			if (!FP.RegionIndices.IsValidIndex(RegionIndex))
			{
				// 빌드에서 길이 검증하므로 도달하지 않는 방어선.
				continue;
			}
			for (const int32 Idx : FP.RegionIndices[RegionIndex])
			{
				IRopeCollider* Collider = FP.Colliders.IsValidIndex(Idx) ? FP.Colliders[Idx] : nullptr;
				if (!Collider || IsOwnBodyCollider(FP, Idx))
				{
					continue;
				}
				if (FP.bWorldStatic)
				{
					// 예산 적용 대상
					WorldStaticCandidates.Add(Collider);
				}
				else
				{
					// 스켈레톤 등 — 항상 포함
					OutColliders.Add(Collider);
				}
			}
			continue;
		}

		// 폴백 경로(매핑 없는 provider): 이전 방식의 collider bounds 재-컬.
		if (FP.Bounds.Num() != FP.Colliders.Num())
		{
			// bounds 캐시 불일치 → 전체 폴백(드묾).
			for (int32 c = 0; c < FP.Colliders.Num(); ++c)
			{
				if (FP.Colliders[c] && !IsOwnBodyCollider(FP, c))
				{
					OutColliders.Add(FP.Colliders[c]);
				}
			}
			continue;
		}
		for (int32 c = 0; c < FP.Colliders.Num(); ++c)
		{
			if (IsOwnBodyCollider(FP, c))
			{
				continue;
			}
			if (FP.Colliders[c] && FP.Bounds[c].IsValid && FP.Bounds[c].Intersect(RopeBounds))
			{
				if (FP.bWorldStatic)
				{
					// 예산 적용 대상
					WorldStaticCandidates.Add(FP.Colliders[c]);
				}
				else
				{
					// 스켈레톤 등 — 항상 포함
					OutColliders.Add(FP.Colliders[c]);
				}
			}
		}
	}

	if (WorldStaticCandidates.Num() <= PerRopeBudget)
	{
		OutColliders.Append(WorldStaticCandidates);
		return;
	}

	// 예산 초과: 이 로프의 실제 노드에 가까운 순으로 상위 PerRopeBudget개만 싣는다(먼 것부터 드롭).
	// 근접도는 콜라이더 월드 bounds 중심과 로프 노드들의 최소 제곱거리 — 후보당 1회만 계산(정렬 중 재계산 방지).
	// 이 경로는 예산 초과 프레임에서만 도는 드문 경로.
	struct FRankedCollider { IRopeCollider* Collider; float DistSq; };
	TArray<FRankedCollider> Ranked;
	Ranked.Reserve(WorldStaticCandidates.Num());
	for (IRopeCollider* Collider : WorldStaticCandidates)
	{
		const FVector Center = Collider->GetWorldBounds().GetCenter();
		float Best = TNumericLimits<float>::Max();
		for (int32 i = 0; i < Rope.Sim.Num(); ++i)
		{
			Best = FMath::Min(Best, static_cast<float>(FVector::DistSquared(Center, Rope.Sim.Positions[i])));
		}
		Ranked.Add({ Collider, Best });
	}
	Ranked.Sort([](const FRankedCollider& A, const FRankedCollider& B) { return A.DistSq < B.DistSq; });
	for (int32 i = 0; i < PerRopeBudget; ++i)
	{
		OutColliders.Add(Ranked[i].Collider);
	}
	UE_LOG(LogRopeCollision, Verbose,
		TEXT("Rope on %s: %d world-static colliders exceed per-rope budget (%d) — kept nearest, dropped %d."),
		*GetNameSafe(Rope.GetOwner()), WorldStaticCandidates.Num(), PerRopeBudget, WorldStaticCandidates.Num() - PerRopeBudget);
}

void URopeSimSubsystem::GatherAimCollidersForRope(URopeComponent& Rope, int32 RopeIndex) const
{
	const int32 RegionIndex = AimRegionIndexOf(RopeIndex);
	const bool bAiming = FrameRopeRegions.IsValidIndex(RegionIndex) && FrameRopeRegions[RegionIndex].IsValid;
	if (!bAiming)
	{
		// 조준이 끝났거나 애초에 조준 중이 아니면 목록을 비운다 — 지난 프레임 provider 포인터가
		// 남아 있으면 다음 조준 질의가 이미 파괴된 스토리지를 읽는다.
		Rope.SimFrame.AimFrameColliders.Reset();
		return;
	}
	GatherCollidersForRope(Rope, RegionIndex, Rope.SimFrame.AimFrameColliders);
}

bool URopeSimSubsystem::RefreshAimFrameCollidersForImmediateQuery(URopeComponent& /*Rope*/)
{
	// ABI/source 호환용 no-op. 정상 Tick 외부에서 BuildFrameColliders를 호출하면 provider 1회/프레임
	// 계약과 다중 Wielder region 일관성이 다시 깨지므로 즉시 경로는 복원하지 않는다.
	return false;
}

void URopeSimSubsystem::Tick(float DeltaTime)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SubsystemTick);
	SCOPE_CYCLE_COUNTER(STAT_RopeSim_Tick);

	// 무효 항목 정리. 여기로 사라지는 로프는 UnregisterRope를 거치지 않았으므로(액터가 정식 해제 없이
	// 파괴된 경우) GPU 상주 자원이 남는다 — 배열에서 빼는 것과 자원 회수를 한 몸으로 처리한다.
	Ropes.RemoveAllSwap([](const TObjectPtr<URopeComponent>& Rope) { return !IsValid(Rope.Get()); });
	ReleaseGpuResourcesForDeadRopes();
	if (Ropes.Num() == 0)
	{
		// 마지막 로프가 사라진 프레임에 GDF 수요를 내리지 않으면(종전에는 아래 SetGDFActiveCount에
		// 닿기 전에 return했다) 엔진이 아무도 안 쓰는 Global Distance Field를 계속 빌드한다.
		if (const UWorld* World = GetWorld())
		{
			RopeGDF::SetGDFActiveCount(World->Scene, 0);
		}
		return;
	}

	// G4: GPU가 유일 런타임 경로. 렌더 가능 RHI면 GPU, 아니면 CPU 폴백(자동). 감지도 GPU와 함께 켜진다.
	const bool bUseGPU = RopeGpuRuntimeAvailable();
	// GPU 솔브 시 감지도 GPU(별도 토글 없음).
	const bool bUseGPUContacts = bUseGPU;

	// 'stat DynamicRope' 프레임 대시보드용 집계. NumGdfRopes/TotalFrameColliders는 아래 GPU/gather 루프에
	// 얹어 모으고(서브시스템만 아는 값), 나머지 페이즈/솔브 경로 카운터는 RecordFrameStats가 public 게터로 집계.
	int32 NumGdfRopes = 0;
	int32 TotalFrameColliders = 0;

	// GPU 상주(M5): RT 리드백이 채운 RopeId별 최신(약 1~2프레임 지연) 위치를 회수해 캐시. 아래 Phase 2에서
	// Free/Flight 로프의 Sim(렌더/충돌 미러)에 반영한다. 순차 의존성은 GPU 영속 버퍼 안에서 충족된다.
	if (bUseGPU)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_GPUGetLatest);
		GpuSolver.GetLatest(GpuLatest);
		// 뷰 확장이 소비 못 한 채 교체된 step의 시뮬 시간을 회수한다(아래 TryBuildResidentStep이 되돌린다).
		{
			TMap<uint32, float> Dropped;
			GpuSolver.DrainDroppedSimTime(Dropped);
			for (const TPair<uint32, float>& Pair : Dropped)
			{
				PendingSimTimeRefund.FindOrAdd(Pair.Key) += Pair.Value;
			}
		}
		if (bUseGPUContacts)
		{
			// G3: 접촉 감지 결과 회수(Finalize 전에 귀속).
			GpuSolver.GetLatestContacts(GpuLatestContacts);
		}
	}

	// 아래 로프 순회 동안 Register/UnregisterRope의 Ropes 변형을 지연시킨다(재진입 가드 — 헤더 주석).
	// ResolvePendingAimThrow/Prepare/Finalize가 쏘는 델리게이트 핸들러의 로프 스폰/파괴에 대비.
	bTickingRopes = true;

	// Phase 1a (GT): collider 중앙 수집 — 등록된 provider에서 프레임당 1회 빌드 후 로프별 필터로 FrameColliders 채움.
	// (로프마다 월드를 스캔하던 것을 대체. collider 포인터는 provider 소유라 이번 프레임 solve/finalize 동안 유효.)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_GatherColliders);
		SCOPE_CYCLE_COUNTER(STAT_RopeSim_Gather);
		BuildFrameColliders();
		// 로프 인덱스 = FrameRopeRegions/provider 매핑의 물리 region 인덱스(위 무효 정리 후 순서 고정).
		for (int32 RopeIndex = 0; RopeIndex < Ropes.Num(); ++RopeIndex)
		{
			// 이 프레임 앞선 재진입으로 파괴된(pending-kill) 로프는 건너뛴다. FrameRopeRegions는 !IsValid
			// 자리를 유지하므로 인덱스 대응은 그대로다(변형은 지연됐고 순서는 불변).
			URopeComponent* Rope = Ropes[RopeIndex];
			if (!IsValid(Rope))
			{
				continue;
			}
#if WITH_GAMEPLAY_DEBUGGER
			// 이 프레임 로프를 처음 건드리는 지점 — 아래 ResolvePendingAimThrow가 Flight 전이를 만들 수
			// 있으므로 그 전에 프레임 시작 phase를 굳힌다(디버거 헤더의 "시작→종료" 표시용).
			Rope->CaptureDebugFrameStartPhase();
#endif
			GatherCollidersForRope(*Rope, RopeIndex, Rope->SimFrame.FrameColliders);
			// 조준 목록은 별도 region(로프 AABB ∪ aim ray)에서 따로 모은다 — 원거리 조준 대상의 본
			// 콜라이더가 위 물리 목록으로 새지 않게 하는 분리 계약(FRopeSimFrameIO::AimFrameColliders).
			GatherAimCollidersForRope(*Rope, RopeIndex);
			// Wielder가 PrePhysics에 등록한 HUD/preview 요청도 여기서 확정한다. 다음 Wielder tick이 이 결과를
			// 소비하므로 최대 1프레임 지연되지만, HUD 때문에 BuildFrameColliders를 다시 호출하지 않는다.
			Rope->ResolvePendingAimQuery();
			// ③ 실제 입력은 HUD 캐시를 쓰지 않는다. 입력 순간 ray를 같은 프레임 조준 목록으로 prepared까지
			// 확정하고, 즉시 실행 요청이면 여기서 던지며 몽타주 경로면 notify까지 결과를 보관한다.
			Rope->ResolvePendingGuaranteedAimThrow();
			// 입력 순간 고정한 ray bounds로 collider를 모은 직후 Aim throw를 확정한다.
			// 이 순서 덕분에 같은 요청의 최신 조준 목록으로 hit 또는 FrameForward fallback을 결정한다.
			Rope->ResolvePendingAimThrow();
			// Aim ray가 mesh+bone을 잠근 throw는 여기서 다른 본 collider를 제거한다.
			// 실제/예측 contact와 wrapping path는 항상 이 결과를 쓴다. 일반 solve도 이 목록을 쓰지만,
			// collision-free Aim Flight solve는 거리/굽힘만 풀기 위해 목록을 의도적으로 무시한다.
			Rope->FilterFrameCollidersForAimWrapTarget();
			TotalFrameColliders += Rope->SimFrame.FrameColliders.Num();
		}
	}

	// Phase 1b (GT): 준비 — init/pin + 로직 phase 처리(collider는 위에서 이미 채워짐).
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_Prepare);
		SCOPE_CYCLE_COUNTER(STAT_RopeSim_Prepare);
		TOptional<FVector> LODCameraLocation;
		bool bLODCameraResolved = false;
		for (URopeComponent* Rope : Ropes)
		{
			if (!IsValid(Rope))
			{
				continue;
			}
			// 모든 로프가 같은 로컬 플레이어 카메라를 쓰므로, 필요한 첫 로프에서 프레임당 한 번만 조회한다.
			// 조회 실패도 resolved로 기억해 서버/카메라 없는 월드에서 로프 수만큼 반복하지 않는다.
			if (!bLODCameraResolved && Rope->SolverConfig.bEnableDistanceLOD &&
				Rope->SolverConfig.LODStartDistance > 0.0f)
			{
				bLODCameraResolved = true;
				if (const APlayerCameraManager* Camera = UGameplayStatics::GetPlayerCameraManager(GetWorld(), 0))
				{
					LODCameraLocation = Camera->GetCameraLocation();
				}
			}
			Rope->PrepareSimFrame(DeltaTime, LODCameraLocation);
		}
	}

	// Phase 2: solver step (GPU 상주 / CPU 폴백). Free/Flight/Wrapping/Wrapped는 솔브하고,
	// Releasing은 override-only, Contacting은 dispatch 없음 — 로프별 판정은 TryBuildResidentStep 안에 있다.
	if (bUseGPU)
	{
		// GPU 상주 경로(M5a). 로프별 영속 버퍼를 매 프레임 in-place로 전진(라운드트립 스톨/슬로모 없음).
		// whip(G1)과 로직 페이즈(G2 — Wrapping/Wrapped/Releasing)도 GPU 상주: 로직 산출물
		// (OverrideFrame)을 override 패스로 실어 재시드 없이 커널에서 적용한다. 적분이 없는
		// 로직 프레임은 NumSub=0 override-only dispatch. 접촉 감지는 Finalize가 지연 미러로 처리(G3).
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SolveGPU);
		SCOPE_CYCLE_COUNTER(STAT_RopeSim_Solve);

		TArray<FRopeGPUResidentStep> Steps;
		Steps.Reserve(Ropes.Num());
		// Phase 2c: GDF 소비자 게이트 — 활성 GDF 로프 수(엔진 온디맨드 빌드 신호). NumGdfRopes는 Tick 상단에서 hoist.
		for (URopeComponent* Rope : Ropes)
		{
			if (!IsValid(Rope))
			{
				continue;
			}
			FRopeGPUResidentStep Step;
			if (TryBuildResidentStep(*Rope, DeltaTime, Step))
			{
				if (Step.bUseWorldGDF)
				{
					++NumGdfRopes;
				}
				Steps.Add(MoveTemp(Step));
			}
			else if (Rope->bPendingGpuCaptureHandoff && Rope->bUseWorldGDF)
			{
				// Contacting은 새 GPU step을 만들지 않지만 ReadbackNow가 Scene GDF pending을 보류했을 수
				// 있다. handoff가 끝날 때까지 GDF 수요를 유지해야 다음 view dispatch가 그 step을 소비한다.
				++NumGdfRopes;
			}
		}
		// 이 씬에 활성 GDF 로프가 있으면 커스텀 FX 시스템이 GDF를 요구 → 엔진이 온디맨드로 빌드한다.
		if (const UWorld* World = GetWorld())
		{
			RopeGDF::SetGDFActiveCount(World->Scene, NumGdfRopes);
		}
		if (Steps.Num() > 0)
		{
			// dispatch는 뷰 확장(씬 그래프, PreRenderBasePass)으로 미룬다 — GDF 파라미터가 유효한 타이밍.
			GpuSolver.EnqueueSteps(MoveTemp(Steps));
		}
	}
	else
	{
		// CPU 경로(기본): 로프는 서로 독립 + collider 스냅샷 read-only → 스레드 안전.
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SolveParallel);
		SCOPE_CYCLE_COUNTER(STAT_RopeSim_Solve);
		ParallelFor(Ropes.Num(), [this, DeltaTime](int32 Index)
		{
			URopeComponent* Rope = Ropes[Index];
			if (!IsValid(Rope))
			{
				return;
			}
			// CPU 경로 → resident 렌더 안 함(M5b).
			Rope->SimFrame.bGpuSteppedThisFrame = false;
			Rope->SolveSimFrame(DeltaTime);
		});
	}

	// Phase 3 (GT): 마무리 — Flight 접촉 감지/캡처(UObject·이벤트) + 렌더 dirty.
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_Finalize);
		SCOPE_CYCLE_COUNTER(STAT_RopeSim_Finalize);
		for (URopeComponent* Rope : Ropes)
		{
			if (!IsValid(Rope))
			{
				continue;
			}
			// G3: GPU 감지 결과를 귀속해 Finalize의 Flight 접촉 소스를 GPU 후보로 채운다.
			// 게이트는 전역이 아니라 로프별 bGpuSteppedThisFrame — 이 프레임 실제로 GPU step된 로프만
			// GPU 감지를 쓴다. GPU step 못 한 로프(노드>MaxNodes 등)는 CPU 솔브됐으므로 여기서도 GPU 후보를
			// 강제하지 않아, FinalizeSimFrame이 CPU 스윕 감지로 폴백한다(안 그러면 감지 자체가 누락돼 캡처 불가).
			Rope->SimFrame.bGpuContactsThisFrame = false;
			if (Rope->SimFrame.bGpuSteppedThisFrame && Rope->Phase == ERopePhase::Flight)
			{
				BuildGpuFlightCandidates(*Rope);
			}
			Rope->FinalizeSimFrame(DeltaTime);
		}
	}

	// 슬립(Free 정지 로프 솔브 스킵)/거리 LOD(iteration 감쇠)/gather 거리 컬링은 구현됨 — 컴포넌트
	// (UpdateSleepState/ComputeSolverLOD) + GatherCollidersForRope. TODO: 프레임당 총 솔브 비용 상한.

	// 'stat DynamicRope' — 프레임 부하/페이즈 대시보드 갱신(그룹 미수집 시 helper가 순회 스킵).
	RopeStats::FRopeFrameCounters FrameCounters;
	FrameCounters.NumGdfDispatched = NumGdfRopes;
	FrameCounters.FrameColliders = TotalFrameColliders;
	RopeStats::RecordFrameStats(Ropes, FrameCounters);

	// 순회 종료 — 미뤄둔 로프 등록/해제를 지금 반영한다(이후부터 즉시 변형 재개).
	bTickingRopes = false;
	ApplyDeferredRopeChanges();
}

void FRopeSimTickFunction::ExecuteTick(float DeltaTime, ELevelTick TickType, ENamedThreads::Type /*CurrentThread*/,
	const FGraphEventRef& /*MyCompletionGraphEvent*/)
{
	if (Target && TickType != LEVELTICK_ViewportsOnly)
	{
		Target->Tick(DeltaTime);
	}
}

FString FRopeSimTickFunction::DiagnosticMessage()
{
	return TEXT("FRopeSimTickFunction(URopeSimSubsystem)");
}

FName FRopeSimTickFunction::DiagnosticContext(bool /*bDetailed*/)
{
	return FName(TEXT("RopeSimSubsystem"));
}

bool URopeSimSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
	// 게임/PIE에서만 시뮬레이션(에디터 프리뷰/인스펙터 월드 제외 → 컴포넌트도 그때만 BeginPlay 등록).
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

void URopeSimSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);

	// TG_PostPhysics 틱 함수 등록(기존 UTickableWorldSubsystem tickable 대체). tickable은 엔진 TickObjects
	// 호출 위치(TG_PostPhysics 뒤/TG_PostUpdateWork 앞 — 엔진 구현 세부)에 묵시적으로 얹혀 있었다. 명시
	// 그룹 + 메시 틱 선행조건(SetAnimPrerequisites)으로 "애니 평가 이후" 순서를 계약으로 만든다.
	// bAllowTickOnDedicatedServer: 기존 tickable도 서버에서 돌았으므로 유지(CPU 폴백 시뮬).
	SimTickFunction.Target = this;
	SimTickFunction.TickGroup = TG_PostPhysics;
	SimTickFunction.EndTickGroup = TG_PostPhysics;
	SimTickFunction.bCanEverTick = true;
	SimTickFunction.bStartWithTickEnabled = true;
	SimTickFunction.bAllowTickOnDedicatedServer = true;
	SimTickFunction.RegisterTickFunction(InWorld.PersistentLevel);

	// GDF 통합 경로에서 뷰 확장이 씬→솔버로 찾아 dispatch할 수 있게 이 월드의 씬에 솔버를 등록한다.
	// (씬은 이 시점에 렌더링용으로 생성돼 있다.) 경로가 off여도 등록은 무해(pending이 비어 no-op).
	RopeGDF::RegisterSolver(InWorld.Scene, &GpuSolver);

	// 정적 월드 충돌 프로바이더 호스트 액터를 월드당 1개 자동 스폰한다. 세팅이 지정한 클래스(기본
	// ARopeController)를 스폰하되, None이면 자동 스폰을 끈다(수동 배치 opt-out). 스폰된 액터는 즉시
	// BeginPlay를 받아 URopeStaticBodyProvider가 RegisterColliderProvider로 등록된다. DoesSupportWorldType이
	// Game/PIE로 제한하므로 에디터 프리뷰 월드엔 생기지 않는다.
	// 중복 방지(조각 1): 자동 스폰 전에 월드에 이미 정적 프로바이더가 있으면(수동 배치 등) 양보하고 스폰하지
	// 않는다 → "수동 배치가 자동 스폰을 이긴다"는 결정적 우선순위. 레지스트리(등록 순서 의존) 대신 컴포넌트
	// 인스턴스 존재로 판정 — 배치 액터는 BeginPlay 전에 이미 인스턴스화돼 있어 등록 타이밍과 무관하게 잡힌다.
	bool bManualProviderPresent = false;
	for (TObjectIterator<URopeStaticBodyProvider> It; It; ++It)
	{
		if (IsValid(*It) && !It->IsTemplate() && It->GetWorld() == &InWorld)
		{
			bManualProviderPresent = true;
			break;
		}
	}

	if (const UDynamicRopeSettings* Settings = UDynamicRopeSettings::Get())
	{
		if (bManualProviderPresent)
		{
			UE_LOG(LogRopeCollision, Verbose,
				TEXT("RopeSimSubsystem: 기존 정적 바디 프로바이더가 있어 자동 스폰을 건너뜁니다(수동 배치 우선)."));
		}
		else if (!Settings->StaticBodyControllerClass.IsNull())
		{
			UClass* ControllerClass = Settings->StaticBodyControllerClass.LoadSynchronous();
			if (ControllerClass)
			{
				FActorSpawnParameters SpawnParams;
				// 런타임 매니저 — 레벨에 저장하지 않는다.
				SpawnParams.ObjectFlags |= RF_Transient;
				// 위치 무관(원점).
				SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
				SpawnedStaticBodyController = InWorld.SpawnActor<AActor>(ControllerClass, FTransform::Identity, SpawnParams);
				// 콜라이더 예산/컨벡스 평면 상한은 프로바이더가 BuildColliders에서 Project Settings를 직접 읽으므로
				// 여기서 주입할 필요가 없다(단일 소스 — 컴포넌트에 중복 필드를 두지 않는다).
				UE_LOG(LogRopeCollision, Verbose, TEXT("RopeSimSubsystem: spawned static-body controller %s (%s)."),
					*GetNameSafe(SpawnedStaticBodyController), *GetNameSafe(ControllerClass));
			}
			else
			{
				UE_LOG(LogRopeCollision, Warning, TEXT("RopeSimSubsystem: StaticBodyControllerClass failed to load — no static world collision provider spawned."));
			}
		}
	}
}

void URopeSimSubsystem::Deinitialize()
{
	if (SimTickFunction.IsTickFunctionRegistered())
	{
		SimTickFunction.UnRegisterTickFunction();
	}
	SimTickFunction.Target = nullptr;

	// 자동 스폰한 매니저 액터 파괴. 월드 teardown이 어차피 액터를 정리하지만, 명시적으로 지워
	// 재-Initialize(예: PIE seamless travel) 시 잔여물이 남지 않게 한다. IsValid로 이미 파괴된 경우 방어.
	if (IsValid(SpawnedStaticBodyController))
	{
		SpawnedStaticBodyController->Destroy();
	}
	SpawnedStaticBodyController = nullptr;

	if (const UWorld* World = GetWorld())
	{
		// 수요를 먼저 내리고 솔버를 뗀다(월드가 살아 있는 재-Initialize 경로에서 잔여 수요가 남지 않게).
		RopeGDF::SetGDFActiveCount(World->Scene, 0);
		RopeGDF::UnregisterSolver(World->Scene);
	}
	Super::Deinitialize();
}

void URopeSimSubsystem::BuildGpuFlightCandidates(URopeComponent& Rope)
{
	Rope.SimFrame.GpuFlightCandidates.Reset();

	const FRopeResidentContacts* Contacts = GpuLatestContacts.Find(Rope.GetUniqueID());
	if (!Contacts || Contacts->Generation != Rope.SimFrame.SimGeneration)
	{
		// 아직 회수분이 없거나 재시드 catch-up 중 — 이번 프레임은 GPU 후보 없음(캡처는 다음 프레임).
		// 소스는 GPU(빈 후보) — CPU 스윕으로 되돌아가지 않는다.
		Rope.SimFrame.bGpuContactsThisFrame = true;
		return;
	}

	// 콜라이더 집합 대응 게이트(#7): 지연된 접촉의 ColliderIndex는 **디스패치 시점** 집합 기준인데 아래
	// 귀속 테이블은 이번 프레임 것으로 재빌드됐다. 결과가 싣고 온 dispatch 서명과 지금 서명이 같을 때만
	// 인덱스가 같은 뜻이다 — 다르면 다른 본으로의 오귀속 대신 드롭(다음 프레임 캡처).
	// 서명 0은 미설정(워밍업)이라 역시 드롭한다.
	if (Contacts->AttribSig == 0 || Contacts->AttribSig != Rope.SimFrame.GpuAttribSig)
	{
		Rope.SimFrame.bGpuContactsThisFrame = true;
		return;
	}

	// Contacts는 슬롯 순서(actual 먼저, predictive 뒤)라 actual이 우선 처리된다. CPU AddUniqueCandidate와
	// 동일하게 (node, bone, mesh) 중복은 병합한다(SourceMask OR + Source 우선순위 Guided>Actual>Free) —
	// 트래커의 노드 중복 카운트를 막고 판정을 CPU와 일치시킨다.
	for (const FRopeGPUContactResult& C : Contacts->Contacts)
	{
		// 콜라이더 인덱스 → (bone, mesh) 귀속. 범위 밖(콜라이더 집합 변화)은 건너뛴다(자기수정).
		const TArray<FRopeSimFrameIO::FGpuColliderAttribution>& Attr =
			(C.ColliderType == 0) ? Rope.SimFrame.GpuCapsuleAttribution :
			(C.ColliderType == 1) ? Rope.SimFrame.GpuSdfAttribution :
			                        Rope.SimFrame.GpuBoxAttribution;
		if (!Attr.IsValidIndex(C.ColliderIndex))
		{
			continue;
		}
		const FRopeSimFrameIO::FGpuColliderAttribution& A = Attr[C.ColliderIndex];
		if (A.Bone.IsNone())
		{
			// 귀속 불가(비-스켈레탈 collider) — 캡처 대상 아님.
			continue;
		}
		// weak — 지연 중 파괴됐으면 null(판정은 bone으로 진행).
		const USceneComponent* Mesh = A.Mesh.Get();

		// 병합: 같은 (node, bone, mesh) 후보가 있으면 SourceMask OR + Source 우선순위 갱신, 새 후보는 추가 안 함.
		FRopeContactCandidate* Existing = nullptr;
		for (FRopeContactCandidate& E : Rope.SimFrame.GpuFlightCandidates)
		{
			if (E.NodeIndex == C.NodeIndex && E.Bone == A.Bone && E.Mesh == Mesh)
			{
				Existing = &E;
				break;
			}
		}
		if (Existing)
		{
			Existing->SourceMask |= C.Source;
			if (C.Source == static_cast<uint8>(ERopeContactCandidateSource::PredictiveGuided) ||
				(Existing->Source == ERopeContactCandidateSource::Actual &&
					C.Source == static_cast<uint8>(ERopeContactCandidateSource::PredictiveFree)))
			{
				Existing->Source = static_cast<ERopeContactCandidateSource>(C.Source);
			}
			continue;
		}

		FRopeContactCandidate Cand;
		Cand.bValid = true;
		Cand.NodeIndex = C.NodeIndex;
		Cand.Bone = A.Bone;
		Cand.Mesh = Mesh;
		Cand.Source = static_cast<ERopeContactCandidateSource>(C.Source);
		Cand.SourceMask = C.Source;
		Cand.WorldPoint = C.WorldPoint;
		Cand.Normal = C.Normal.GetSafeNormal();
		Cand.Penetration = C.Penetration;
		Cand.SurfaceVelocity = C.SurfaceVelocity;
		// EvaluateRelativeMotion(GT)이 채운다.
		Cand.WrapDirectionScore = 0.0f;
		Rope.SimFrame.GpuFlightCandidates.Add(Cand);
	}
	Rope.SimFrame.bGpuContactsThisFrame = true;
}

bool URopeSimSubsystem::SyncGpuPositionsForHandoff(URopeComponent& Rope)
{
	if (!RopeGpuRuntimeAvailable())
	{
		// CPU 폴백 — Sim이 이미 최신.
		return false;
	}

	FRopeSimState& S = Rope.Sim;
	TArray<FVector> Pos;
	TArray<FVector> Prev;
	uint32 Generation = 0;
	if (!GpuSolver.ReadbackNow(Rope.GetUniqueID(), Pos, Prev, Generation))
	{
		// 상주 버퍼 없음(GPU로 step된 적 없음) — 미러가 곧 진실.
		return false;
	}
	if (Generation != Rope.SimFrame.SimGeneration || Pos.Num() != S.Num() || Prev.Num() != S.Num())
	{
		// 재시드 catch-up 중이거나 노드 수 불일치 — stale 적용 방지.
		return false;
	}

	S.Positions = MoveTemp(Pos);
	S.PrevPositions = MoveTemp(Prev);
	// 잡은 끝(node 0)은 미러 규약과 동일하게 현재 핀으로 스냅.
	if (S.bStartPinned && S.Num() > 0)
	{
		S.Positions[0] = S.StartPinTarget;
		S.PrevPositions[0] = S.StartPinPrev;
	}
	return true;
}

bool URopeSimSubsystem::TryBuildResidentStep(URopeComponent& Rope, float DeltaTime, FRopeGPUResidentStep& OutStep)
{
	FRopeSimState& S = Rope.Sim;

	// GPU 상주 대상: 솔브 프레임(Free/Flight/Wrapping/Wrapped) 또는 로직 산출물만 있는 Releasing 프레임.
	// bSolveThisFrame/OverrideFrame이 Prepare에서 권위 있게 정해지므로 여기서는 별도 phase 체크가 필요 없다.
	// Contacting(산출물 없음)은 dispatch 자체가 없어 GPU 버퍼가 동결 유지된다(CPU의 "솔브 없음"과 동일).
	const bool bGpuRope = (Rope.SimFrame.bSolveThisFrame || Rope.SimFrame.OverrideFrame.HasAny())
		&& S.Num() >= 2 && S.Num() <= FRopeGPUSolver::MaxNodes;
	// M5b: 이 프레임에 GPU step되는 로프만 렌더가 resident PosBuf를 직접 읽는다(아니면 stale → CPU 미러).
	Rope.SimFrame.bGpuSteppedThisFrame = bGpuRope;
	if (!bGpuRope)
	{
		// 폴백(노드수 초과 등): bSolveThisFrame인 자유 구간은 CPU 솔브, override-only 프레임은 스킵.
		if (Rope.SimFrame.bSolveThisFrame)
		{
			Rope.SolveSimFrame(DeltaTime);
		}
		return false;
	}

	const uint32 RopeId = Rope.GetUniqueID();

	// 직전 회수분을 Sim(미러)에 반영. generation이 현재와 일치할 때만(= GPU가 현재 시드를 따라잡음);
	// 재시드 직후 catch-up 중이면 CPU Sim을 그대로 둬 시드 소스를 보존한다.
	if (const FRopeResidentLatest* L = GpuLatest.Find(RopeId))
	{
		if (L->Generation == Rope.SimFrame.SimGeneration && L->NumNodes == S.Num()
			&& L->Positions.Num() == S.Num() && L->PrevPositions.Num() == S.Num())
		{
			S.Positions = L->Positions;
			S.PrevPositions = L->PrevPositions;
			// 장력 미러(있을 때만 — 솔브 프레임에만 회수되므로 위치보다 드물 수 있다. 없으면 직전 값 유지).
			if (L->SegmentTension.Num() == S.Num() - 1)
			{
				S.SegmentTension = L->SegmentTension;
			}
		}
	}

	// 잡은 끝(node 0)을 현재 핀 위치로 정확히 맞춘다 — GPU 미러는 ~1~2프레임 지연이라 손과 어긋난다.
	// 렌더/접촉용 보정(GPU 솔브 자체는 PinTarget으로 매 스텝 핀을 처리하므로 시뮬레이션엔 영향 없음).
	if (S.bStartPinned && S.Num() > 0)
	{
		S.Positions[0] = S.StartPinTarget;
		S.PrevPositions[0] = S.StartPinPrev;
	}

	// 미러가 Prepare의 로직 산출물(앵커 위치 등)을 덮었으면 재적용 — CPU Sim 미러를
	// "최신 본 기준 로직 쓰기 + 지연된 자유 구간"의 최선 조합으로 유지한다(G2).
	if (Rope.SimFrame.OverrideFrame.HasAny())
	{
		Rope.SimFrame.OverrideFrame.ApplyToSim(S);
	}

	// 고정-timestep 스케줄(CPU accumulator). override-only 프레임(bSolveThisFrame=false)은 적분 없이
	// override만 기록한다(NumSub=0) — CPU 경로의 "솔브 없음"과 동일한 시간 처리.
	FRopeSubstepSchedule Schedule;
	Schedule.NumSub = 0;
	Schedule.FixedDt = 0.0f;
	if (Rope.SimFrame.bSolveThisFrame)
	{
		// dispatch되지 못하고 버려진 step의 시간을 accumulator로 되돌린 뒤 스케줄을 짠다 — 그래야
		// accumulator가 "시뮬된 시간"의 단일 진실로 유지된다. 되돌린 뒤 바로 아래 RopeSolverSubsteps가
		// MaxAccum으로 클램프하므로 긴 정지 뒤 몰아치기는 기존 slow-mo 정책 그대로 제한된다.
		float Refund = 0.0f;
		if (PendingSimTimeRefund.RemoveAndCopyValue(RopeId, Refund) && Refund > 0.0f)
		{
			S.TimeAccumulator += Refund;
		}
		Schedule = RopeSolverSubsteps(S, Rope.SolverConfig, DeltaTime);
	}

	// 상주 step 구성(self-contained). 시드 데이터는 매 프레임 제공(RT는 재시드 시에만 GPU 업로드).
	SeedResidentStep(OutStep, RopeId, Rope.SimFrame.SimGeneration, S, Rope.SolverConfig, Schedule);
	// CPU SolveSimFrame과 같은 phase별 비신축 계약. GPU의 최신 resident pose에서 strain-limit가
	// 적용되므로 지연된 CPU mirror를 기준으로 guide target을 보정하는 것보다 정확하다.
	OutStep.MaxStretchRatio = Rope.GetEffectiveMaxStretchRatio();
	// 컴포넌트 경계 해석값 덮기: 반지름 auto(0=렌더 Radius) + 컴포넌트 직속으로 이사한 GDF 플래그.
	OutStep.CollisionRadius = Rope.GetEffectiveCollisionRadius();
	// solve 충돌과 contact detection은 별도 계약이다. false여도 아래 PackStepColliders는 detect용으로
	// 계속 패킹하며, solve 커널에 전달되는 collider/GDF 개수만 0이 된다.
	OutStep.bSolveCollisions = Rope.SimFrame.bSolveCollisionsThisFrame;
	OutStep.bUseWorldGDF = Rope.bUseWorldGDF && OutStep.bSolveCollisions;
	// 거리 LOD: 원거리 로프는 iteration 감쇠(Prepare에서 계산). CollisionPasses는 패킹에서 Iterations로 클램프됨.
	OutStep.Iterations = Rope.GetLODScaledIterations();

	// G3: 접촉 감지는 Flight 로프에만(캡처는 Flight에서만). 이 함수는 GPU 경로에서만 호출되므로
	// GPUContacts는 항상 켜져 있다 — 게이트는 phase == Flight 하나로 충분.
	const bool bDetectThisRope = (Rope.Phase == ERopePhase::Flight);
	if (bDetectThisRope)
	{
		RequestContactDetection(Rope, DeltaTime, OutStep);
	}

	// 충돌: 이 로프의 collider를 capsule(M2)/SDF(M3)로 분류(+ 감지 시 귀속 테이블 병행).
	PackStepColliders(Rope, bDetectThisRope, OutStep);

	// G2: 로직 페이즈 산출물(OverrideFrame)을 override로 주입 — 로직 페이즈 재시드 대체.
	// CPU Sim에 적용된 것과 완전히 같은 데이터(비트 미러는 위 static_assert로 보증).
	if (Rope.SimFrame.OverrideFrame.HasAny() && Rope.SimFrame.OverrideFrame.Flags.Num() == S.Num())
	{
		OutStep.OverrideFlags = Rope.SimFrame.OverrideFrame.Flags;
		OutStep.OverridePositions = Rope.SimFrame.OverrideFrame.Positions;
		OutStep.OverridePrevPositions = Rope.SimFrame.OverrideFrame.PrevPositions;
		OutStep.OverrideInvMass = Rope.SimFrame.OverrideFrame.InvMass;
	}

	// G1: whip 가이드 타깃을 override로 주입(Flight 전용, 적분 전 적용).
	PackWhipOverride(Rope, OutStep);

	return true;
}

void URopeSimSubsystem::RequestContactDetection(URopeComponent& Rope, float DeltaTime, FRopeGPUResidentStep& Step) const
{
	const FRopeSimState& S = Rope.Sim;
	// 귀속 테이블(콜라이더 인덱스 → bone/mesh)은 PackStepColliders가 Step.Capsules/SDFColliders와
	// 같은 순서로 채우므로 여기서 먼저 리셋한다.
	Step.bDetectContacts = true;
	Step.ContactRadius = Rope.GetEffectiveContactQueryRadius();
	Step.PredictionFrames = Rope.DetectConfig.PredictiveContactFrames;
	// 감지 스윕 해상도(터널링 방지) — CPU MakeFlightDetectParams와 같은 소스에서 온다.
	Step.ContactSweepStep = Rope.DetectConfig.ContactSweepStep;
	Step.ContactMaxSweepSamples = Rope.DetectConfig.ContactMaxSweepSamples;
	// 예측 접촉 free 노드 외삽의 substep→프레임 변위 환산(#8). Step.FixedDt(=Schedule.FixedDt=(1/60)/Substeps)는
	// SeedResidentStep이 이미 채웠다 — CPU MakeFlightDetectParams의 FrameDeltaTime/SubstepDeltaTime과 동일 값.
	Step.ContactFrameToSubstepRatio = (Step.FixedDt > KINDA_SMALL_NUMBER) ? (DeltaTime / Step.FixedDt) : 1.0f;
	Rope.SimFrame.GpuCapsuleAttribution.Reset();
	Rope.SimFrame.GpuSdfAttribution.Reset();
	Rope.SimFrame.GpuBoxAttribution.Reset();

	// 예측 접촉(G3b): whip 활성 프레임엔 가이드 마스크/현재·직전·다음 타깃을 실어 GPU가
	// 가이드 노드를 외삽하게 한다(CPU AddPredictedContactCandidates와 동일 입력).
	const TArray<uint8>& WhipMask = Rope.WhipGuide.GetGuidedNodeMask();
	if (Step.PredictionFrames > KINDA_SMALL_NUMBER && WhipMask.Num() == S.Num())
	{
		TArray<FVector> NextTargets;
		Rope.WhipGuide.PreviewNextTargets(DeltaTime, S, Rope.MakeWhipGuideConfig(), NextTargets);
		Step.WhipGuidedMask = WhipMask;
		Step.WhipCurrentTargets = Rope.WhipGuide.GetCurrentTargets();
		Step.WhipPrevTargets = Rope.WhipGuide.GetPrevTargets();
		Step.WhipNextTargets = MoveTemp(NextTargets);
	}
}

// GPU 귀속 집합의 순서 있는 (bone, mesh) 서명. 지연된 접촉(1~2프레임)의 ColliderIndex가 이번 프레임
// 귀속 테이블과 안전하게 대응하는지(= 집합/순서 불변)를 프레임 간 비교로 판정하는 데 쓴다.
// 자세한 계약은 FRopeSimFrameIO::GpuAttribSig 주석 참조.
static uint32 RopeComputeAttribSig(const TArray<FRopeSimFrameIO::FGpuColliderAttribution>& Attr, uint32 Seed)
{
	uint32 H = HashCombine(Seed, static_cast<uint32>(Attr.Num()));
	for (const FRopeSimFrameIO::FGpuColliderAttribution& E : Attr)
	{
		H = HashCombine(H, GetTypeHash(E.Bone));
		H = HashCombine(H, PointerHash(E.Mesh.Get()));
	}
	return H;
}

void URopeSimSubsystem::PackStepColliders(URopeComponent& Rope, bool bDetectThisRope, FRopeGPUResidentStep& Step) const
{
	// 감지 시 collider 인덱스 → bone/mesh 귀속을 Step.Capsules/SDFColliders와 같은 순서로 병행 채운다.
	auto MakeAttribution = [](IRopeCollider* Collider)
		{
			FRopeSimFrameIO::FGpuColliderAttribution Attr;
			const USceneComponent* Mesh = nullptr;
			Collider->GetGPUAttribution(Attr.Bone, Mesh);
			Attr.Mesh = Mesh;
			return Attr;
		};

	// GPU 표현이 없는 collider의 조용한 제외를 1회 경고로 드러낸다(세션당 1회 — 스팸 방지 래치).
	// CPU 계약(Query/QuerySwept)만 구현한 커스텀 collider는 유닛 테스트/CPU 폴백에선 동작하지만
	// 런타임 정규 경로(GPU 솔브)에서는 여기서 제외된다 — 경고 없이는 "테스트에선 되는데 게임에선
	// 로프가 뚫림"으로 나타나는 최악 유형의 함정이라 로그가 계약의 일부다(RopeCollider.h 참조).
	auto WarnGpuUnrepresented = [this, &Rope](IRopeCollider* Collider)
		{
			if (bWarnedGpuUnrepresentedCollider)
			{
				return;
			}
			bWarnedGpuUnrepresentedCollider = true;
			FName Bone = NAME_None;
			const USceneComponent* Mesh = nullptr;
			Collider->GetGPUAttribution(Bone, Mesh);
			UE_LOG(LogRopeCollision, Warning,
				TEXT("[%s] A gathered rope collider has no GPU representation (GetGPUCapsule/SDF/Box/Convex all false) ")
				TEXT("and is IGNORED by the GPU solve - it only participates in the CPU fallback (cook/-nullrhi/oversized ropes). ")
				TEXT("Implement one of the GPU accessors on custom IRopeCollider types (see RopeCollider.h). ")
				TEXT("(worldStatic=%d, bone=%s, mesh=%s; further occurrences suppressed)"),
				*Rope.GetName(), Collider->IsWorldStatic() ? 1 : 0, *Bone.ToString(), *GetNameSafe(Mesh));
		};

	// FrameColliders는 Prepare에서 GT gather된 스냅샷. 2-pass: 비-정적(스켈레탈) collider를 먼저,
	// 정적(월드) collider를 뒤에 패킹한다. 감지(detect) 커널은 capsule을 [0, NumDetectCapsules)만
	// 보므로 정적 캡슐이 감지에서 자동 제외된다 — 감지는 노드당 최심 접촉 1개만 남겨, 벽 접촉이
	// 본 접촉을 가리면 랩 캡처가 조용히 실패하기 때문(박스는 감지 커널에 아예 없다). solve는 전부 본다.
	for (IRopeCollider* Collider : Rope.SimFrame.FrameColliders)
	{
		if (!Collider || Collider->IsWorldStatic())
		{
			continue;
		}
		FRopeGPUCapsule Cap;
		if (Collider->GetGPUCapsule(Cap.A, Cap.B, Cap.Radius))
		{
			// 프레임 모션(prev 끝점 + InvDt): 표면 속도 드래그/상대 운동 CCD. 정적이면 기본값(InvDt 0) 유지.
			Collider->GetGPUCapsuleMotion(Cap.PrevA, Cap.PrevB, Cap.InvDeltaTime);
			Step.Capsules.Add(Cap);
			if (bDetectThisRope)
			{
				Rope.SimFrame.GpuCapsuleAttribution.Add(MakeAttribution(Collider));
			}
			continue;
		}
		FRopeSDFColliderView View;
		if (Collider->GetGPUSDF(View))
		{
			Step.SDFColliders.Add(MakeGpuSdf(View));
			if (bDetectThisRope)
			{
				Rope.SimFrame.GpuSdfAttribution.Add(MakeAttribution(Collider));
			}
			continue;
		}
		FRopeGPUBox Box;
		if (Collider->GetGPUBox(Box.Center, Box.Rot, Box.HalfExtents))
		{
			// 랩 가능 박스(가상 본): 프레임 모션(prev + InvDt)까지 채우고 감지 범위 앞쪽에 패킹.
			Collider->GetGPUBoxMotion(Box.PrevCenter, Box.PrevRot, Box.InvDeltaTime);
			Step.Boxes.Add(Box);
			if (bDetectThisRope)
			{
				Rope.SimFrame.GpuBoxAttribution.Add(MakeAttribution(Collider));
			}
			continue;
		}
			// 비-정적은 capsule/SDF/box만 GPU에 실린다 — 둘 다 아니면 제외.
			WarnGpuUnrepresented(Collider);
	}
	// 감지 경계: 여기까지가 비-정적 캡슐.
	Step.NumDetectCapsules = Step.Capsules.Num();
	// 박스 감지 경계: 여기까지가 랩 가능 박스.
	Step.NumDetectBoxes = Step.Boxes.Num();

	// pass 2: 정적(월드) collider — solve 전용. 캡슐(스피어/스필)은 감지 경계 뒤에 append,
	// 박스는 전용 배열. 귀속 테이블은 인덱스 정렬 유지를 위해 정적 캡슐 분도 채운다(None/null —
	// 감지 커널이 경계 밖 인덱스를 emit하지 않으므로 방어적).
	for (IRopeCollider* Collider : Rope.SimFrame.FrameColliders)
	{
		if (!Collider || !Collider->IsWorldStatic())
		{
			continue;
		}
		FRopeGPUCapsule Cap;
		if (Collider->GetGPUCapsule(Cap.A, Cap.B, Cap.Radius))
		{
			// 정적 — 프레임 모션 없음(InvDt 0 기본값).
			Step.Capsules.Add(Cap);
			if (bDetectThisRope)
			{
				Rope.SimFrame.GpuCapsuleAttribution.Add(MakeAttribution(Collider));
			}
			continue;
		}
		FRopeGPUBox Box;
		if (Collider->GetGPUBox(Box.Center, Box.Rot, Box.HalfExtents))
		{
			// 프레임 모션(prev center/rot + InvDt): 표면 속도 드래그/상대 운동 CCD. 정적이면 기본값(InvDt 0) 유지.
			Collider->GetGPUBoxMotion(Box.PrevCenter, Box.PrevRot, Box.InvDeltaTime);
			Step.Boxes.Add(Box);
			if (bDetectThisRope)
			{
				// 정적 - None(감지 미참여, 인덱스 정렬용)
				Rope.SimFrame.GpuBoxAttribution.Add(MakeAttribution(Collider));
			}
			continue;
		}
		TConstArrayView<FPlane> LocalPlanes;
		FBox LocalBounds(ForceInit);
		FQuat CvRot, CvPrevRot;
		FVector CvTrans, CvPrevTrans;
		float CvInvDt = 0.0f;
		if (!Collider->GetGPUConvex(LocalPlanes, LocalBounds, CvRot, CvTrans, CvPrevRot, CvPrevTrans, CvInvDt)
			|| LocalPlanes.Num() == 0 || !LocalBounds.IsValid)
		{
			// 정적은 capsule/box/convex만 GPU에 실린다 — 전부 아니면 제외.
			WarnGpuUnrepresented(Collider);
			continue;
		}
		{
			// 바디-로컬 평면을 평탄 풀에 이어붙이고 오프셋/개수로 참조 + 강체(curr/prev) + InvDt.
			FRopeGPUConvex Cv;
			Cv.PlaneOffset = Step.ConvexPlanes.Num();
			Cv.PlaneCount = LocalPlanes.Num();
			Cv.LocalBoundsCenter = LocalBounds.GetCenter();
			Cv.LocalBoundsExtent = LocalBounds.GetExtent();
			Cv.Rot = CvRot; Cv.Trans = CvTrans;
			Cv.PrevRot = CvPrevRot; Cv.PrevTrans = CvPrevTrans;
			Cv.InvDeltaTime = CvInvDt;
			Step.ConvexPlanes.Reserve(Step.ConvexPlanes.Num() + LocalPlanes.Num());
			for (const FPlane& Pl : LocalPlanes)
			{
				// 로컬·단위·바깥, PlaneDot=dot(N,p)-W
				Step.ConvexPlanes.Add(FVector4(Pl.X, Pl.Y, Pl.Z, Pl.W));
			}
			Step.Convexes.Add(Cv);
		}
	}

	// 지연 GPU 접촉 오귀속 방지(#7): 이번 프레임 귀속 집합의 서명을 롤링 기록한다(detect 프레임에만 의미).
	// BuildGpuFlightCandidates가 최근 창(현재==Prev1==Prev2)의 안정성으로 지연 접촉 소비를 게이트한다.
	if (bDetectThisRope)
	{
		uint32 Sig = 0x9E3779B9u;
		Sig = RopeComputeAttribSig(Rope.SimFrame.GpuCapsuleAttribution, Sig);
		Sig = RopeComputeAttribSig(Rope.SimFrame.GpuSdfAttribution, Sig);
		Sig = RopeComputeAttribSig(Rope.SimFrame.GpuBoxAttribution, Sig);
		// 서명 0은 "미설정"이라는 뜻으로 예약돼 있다 — 해시가 우연히 0이면 1로 밀어 워밍업과 구분한다.
		Rope.SimFrame.GpuAttribSig = (Sig == 0) ? 1u : Sig;
		// 이 dispatch가 쓰는 집합의 서명을 step에 싣는다(감지 결과가 그대로 되싣고 돌아온다).
		Step.AttribSig = Rope.SimFrame.GpuAttribSig;
	}
}

void URopeSimSubsystem::PackWhipOverride(const URopeComponent& Rope, FRopeGPUResidentStep& Step) const
{
	// G1: Prepare의 Advance가 계산한 whip 가이드 타깃을 override로 싣는다(CPU 경로 ApplyToSim과 동일 데이터).
	// Flight 게이트: 다른 페이즈에 남은 stale 마스크 적용을 막는다(Flight는 OverrideFrame을 안 채우므로
	// G2 패킹과 겹치지 않는다).
	if (Rope.Phase != ERopePhase::Flight)
	{
		return;
	}
	const TArray<uint8>& WhipMask = Rope.WhipGuide.GetGuidedNodeMask();
	if (WhipMask.Num() == 0)
	{
		return;
	}
	const FRopeSimState& S = Rope.Sim;
	const TArray<FVector>& WhipCur = Rope.WhipGuide.GetCurrentTargets();
	const TArray<FVector>& WhipPrev = Rope.WhipGuide.GetPrevTargets();
	Step.OverrideFlags.SetNumZeroed(S.Num());
	Step.OverridePositions.SetNumZeroed(S.Num());
	Step.OverridePrevPositions.SetNumZeroed(S.Num());
	for (int32 k = 0; k < S.Num() && k < WhipMask.Num(); ++k)
	{
		if (WhipMask[k] == 0 || !WhipCur.IsValidIndex(k))
		{
			continue;
		}
		Step.OverrideFlags[k] = static_cast<uint8>(ERopeGPUOverride::Position | ERopeGPUOverride::Prev);
		Step.OverridePositions[k] = WhipCur[k];
		// 직전 타깃이 없으면(엣지 케이스) 속도 0 — CPU 폴백("직전 위치")과 근사.
		Step.OverridePrevPositions[k] = WhipPrev.IsValidIndex(k) ? WhipPrev[k] : WhipCur[k];
	}
}
