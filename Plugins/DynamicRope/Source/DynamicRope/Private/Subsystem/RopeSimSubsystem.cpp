// Copyright Epic Games, Inc. All Rights Reserved.

#include "Subsystem/RopeSimSubsystem.h"
#include "RopeComponent.h"
#include "DynamicRopeLog.h"
#include "Solver/RopeXPBDSolver.h" // RopeSolverSubsteps
#include "Collision/RopeCollider.h" // IRopeCollider::GetGPUCapsule
#include "Collision/RopeColliderProvider.h" // IRopeColliderProvider (중앙 collider gather)
#include "RopeGPUSolver.h"          // FRopeGPUSolver / FRopeGPUResidentStep / FRopeGPUCapsule (DynamicRopeShaders 모듈)
#include "RopeGPUSolverRegistry.h"  // RopeGDF::RegisterSolver / IsDispatchInVE (GDF 통합 경로)
#include "Settings/DynamicRopeSettings.h"      // StaticBodyControllerClass / StaticBodyMaxColliders(자동 스폰)
#include "Collision/RopeController.h"          // ARopeController(정적 바디 프로바이더 호스트)
#include "Collision/RopeStaticBodyProvider.h"  // 기본 클래스 스폰 시 MaxColliders 주입
#include "Engine/World.h"
#include "SceneInterface.h"         // FSceneInterface (씬→솔버 등록 키)
#include "GameFramework/Actor.h"    // AActor::GetOwner (provider 소스 필터링)
#include "Components/ActorComponent.h"
#include "Components/SkeletalMeshComponent.h" // 틱 선행조건(애니 평가 이후 보장)
#include "Async/ParallelFor.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"
#include "RHI.h"        // GDynamicRHI
#include "Misc/App.h"   // FApp::CanEverRender
#include "UObject/UObjectIterator.h" // TObjectIterator(자동 스폰 전 기존 프로바이더 스캔)
#include "Engine/Engine.h"           // GEngine->AddOnScreenDebugMessage(중복 경고)

namespace
{
	// G4: GPU가 런타임 유일 경로. 렌더 가능한 RHI가 있으면 GPU 상주 솔브+감지, 없으면(쿡/-nullrhi/
	// 서버 빌드) 자동으로 CPU 솔브+감지로 폴백한다. 클라이언트 토글(CVar) 없음 — GPU가 THE 경로.
	// FRopeXPBDSolver는 이 폴백과 패리티 테스트를 위해 유지된다(런타임 클라이언트에선 사실상 미사용).
	bool RopeGpuRuntimeAvailable()
	{
		return GDynamicRHI != nullptr && FApp::CanEverRender();
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
		Sdf.Distances       = View.Distances;   // 코드 바이트 블롭(업로드 평탄화 시 dequant)
		Sdf.BytesPerCode    = View.BytesPerCode;
		Sdf.NarrowBandInner = View.NarrowBandInner;
		Sdf.NarrowBandOuter = View.NarrowBandOuter;
		Sdf.ResX            = View.ResX;
		Sdf.ResY            = View.ResY;
		Sdf.ResZ            = View.ResZ;
		Sdf.LocalMin        = View.LocalMin;
		Sdf.LocalSize       = View.LocalSize;
		Sdf.BoneToWorld     = View.BoneToWorld;
		Sdf.PrevBoneToWorld = View.PrevBoneToWorld; // GPU CCD/표면속도 드래그.
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
		Step.BendCompliance    = Cfg.BendCompliance;
		Step.BendReleaseRatio  = Cfg.BendReleaseRatio;
		Step.BendFullRatio     = Cfg.BendFullRatio;
		Step.Damping           = Cfg.Damping;
		Step.Iterations        = Cfg.Iterations;
		Step.CollisionPasses   = Cfg.CollisionPassesPerSubstep;
		Step.Gravity           = Cfg.Gravity;
		Step.CollisionRadius   = Cfg.CollisionRadius;
		Step.Friction          = Cfg.Friction;
		Step.TipFrictionScale  = Cfg.TipFrictionScale;
		Step.SweepStep         = Cfg.SweepStep;
		Step.MaxSweepSamples   = Cfg.MaxSweepSamples;
		Step.bUseWorldGDF      = Cfg.bUseWorldGDF; // Phase 2c: 엔진 GDF 월드 밀어내기(씬 그래프 dispatch에서만 유효).
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
		Ropes.AddUnique(Rope);
		SetAnimPrerequisites(Rope, /*bAdd*/ true); // 손 핀(소켓 부착)이 소유 캐릭터 포즈를 따르므로.
		UE_LOG(LogDynamicRope, Verbose, TEXT("RegisterRope: %s (%d total)"), *Rope->GetName(), Ropes.Num());
	}
}

void URopeSimSubsystem::UnregisterRope(URopeComponent* Rope)
{
	Ropes.RemoveSingleSwap(Rope);
	if (Rope)
	{
		SetAnimPrerequisites(Rope, /*bAdd*/ false);
		// GPU 상주 버퍼/리드백 해제(렌더 스레드에서). 캐시에서도 제거.
		const uint32 RopeId = Rope->GetUniqueID();
		GpuSolver.ReleaseRope(RopeId);
		GpuLatest.Remove(RopeId);
	}
	UE_LOG(LogDynamicRope, Verbose, TEXT("UnregisterRope: %s (%d remaining)"),
		Rope ? *Rope->GetName() : TEXT("null"), Ropes.Num());
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
					return; // 등록 거부 — 이 프로바이더의 GatherColliders는 호출되지 않는다.
				}
			}
		}
	}

	ColliderProviders.AddUnique(Provider);
	SetAnimPrerequisites(Provider, /*bAdd*/ true); // 본 콜라이더(capsule/SDF)가 소유 캐릭터 포즈를 읽으므로.
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
			SimTickFunction.AddPrerequisite(Mesh, Mesh->PrimaryComponentTick);
		}
		else
		{
			SimTickFunction.RemovePrerequisite(Mesh, Mesh->PrimaryComponentTick);
		}
	}
}

FBox URopeSimSubsystem::ComputeRopeQueryBounds(const URopeComponent& Rope)
{
	// 로프 tight AABB(Pos∪Prev — 프레임 모션 포함) + 마진. provider region과 per-rope collider 컬링이
	// 이 동일 박스를 공유한다(GatherCollidersForRope / BuildFrameColliders 양쪽에서 호출).
	FBox RopeBounds(ForceInit);
	float MaxFrameDispSq = 0.0f; // 예측 접촉(전방 외삽) 여유 계산용 — 이번 프레임 최대 노드 변위.
	for (int32 i = 0; i < Rope.Sim.Num(); ++i)
	{
		RopeBounds += Rope.Sim.Positions[i];
		RopeBounds += Rope.Sim.PrevPositions[i];
		MaxFrameDispSq = FMath::Max(MaxFrameDispSq,
			static_cast<float>(FVector::DistSquared(Rope.Sim.Positions[i], Rope.Sim.PrevPositions[i])));
	}
	if (RopeBounds.IsValid)
	{
		// 여유: 접촉 질의 반경 + 스윕 여유 + 예측 접촉의 전방 외삽 거리(프레임 변위 × 예측 프레임).
		// 넉넉히 잡는다 — 과대 컬링 여유는 안전(콜라이더가 몇 개 더 실릴 뿐).
		const float Margin = Rope.SolverConfig.CollisionRadius + Rope.WrapConfig.ContactRadius
			+ FMath::Max(2.0f * Rope.Sim.SegmentLength, 50.0f)
			+ FMath::Sqrt(MaxFrameDispSq) * FMath::Max(Rope.WrapConfig.PredictiveContactFrames, 1.0f);
		RopeBounds = RopeBounds.ExpandBy(Margin);
	}
	return RopeBounds;
}

void URopeSimSubsystem::BuildFrameColliders()
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_BuildColliders);
	FrameProviders.Reset();

	// 로프별 활성 영역(region) 리스트. provider의 broad-phase에 넘긴다 — bounds-aware provider(정적 바디)는
	// 멀리 동떨어진 로프 사이 빈 공간을 스캔에서 배제해 예산/오버랩 낭비를 피한다. 전 로프 union AABB 폐기.
	// 여기서 쓰는 region은 아래 per-rope 컬링(GatherCollidersForRope)과 동일한 박스라 gather↔cull이 정합.
	TArray<FBox> RopeRegions;
	RopeRegions.Reserve(Ropes.Num());
	for (URopeComponent* Rope : Ropes)
	{
		if (!IsValid(Rope))
		{
			continue;
		}
		const FBox Region = ComputeRopeQueryBounds(*Rope);
		if (Region.IsValid)
		{
			RopeRegions.Add(Region);
		}
	}

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
		FFrameProviderColliders FP;
		FP.Owner = Comp->GetOwner();
		FP.bWorldStatic = Provider->ProvidesWorldStaticColliders(); // 정적 월드 provider는 소유자 제외 면제.
		Provider->GatherColliders(RopeRegions, FP.Colliders);
		if (FP.Colliders.Num() > 0)
		{
			// collider별 월드 bounds를 프레임당 1회 캐시 — 아래 로프별 컬링이 로프 수만큼 재계산하지 않게.
			FP.Bounds.Reserve(FP.Colliders.Num());
			for (const IRopeCollider* Collider : FP.Colliders)
			{
				FP.Bounds.Add(Collider ? Collider->GetWorldBounds() : FBox(ForceInit));
			}
			FrameProviders.Add(MoveTemp(FP));
		}
	}
}

void URopeSimSubsystem::GatherCollidersForRope(const URopeComponent& Rope, TArray<IRopeCollider*>& OutColliders) const
{
	OutColliders.Reset();

	// 기본: 월드의 모든 provider와 충돌하되 자기 owner(던진 본인) provider는 제외(throw 시 self-tangle 방지).
	// 다른 액터 body 잡기(cross-actor)는 그 액터가 "전체"에 포함되므로 자동. owner 충돌이 필요하면 옵트인.
	const AActor* OwnerToExclude = Rope.bIncludeOwnerColliders ? nullptr : Rope.GetOwner();

	// 거리 컬링: 로프 AABB(Pos∪Prev — 프레임 모션 포함)와 안 겹치는 collider는 아예 안 싣는다.
	// CPU 솔버는 자체 broad-phase가 또 있지만, GPU 커널은 콜라이더 전량을 노드마다 루프하므로
	// 여기서 거르는 것이 스케일링의 핵심이다(멀리 있는 캐릭터들의 캡슐/SDF가 스텝에 안 실림).
	// BuildFrameColliders가 provider region으로 넘긴 것과 동일 박스(단일 소스).
	const FBox RopeBounds = ComputeRopeQueryBounds(Rope);
	const bool bCull = RopeBounds.IsValid != 0;

	// 로프별 정적 월드 콜라이더 예산. 전역 추출 상한(StaticBodyMaxColliders)과 별개로, 이 로프가 솔브에
	// 실을 정적 월드 콜라이더 수를 로프마다 독립으로 제한한다(멀리 있는 로프가 이 로프 예산을 못 먹음).
	// 스켈레톤 콜라이더(캡슐/SDF)는 본 수로 자연 제한되고 wrap의 핵심이라 예산 대상에서 제외 — 바로 OutColliders로.
	const UDynamicRopeSettings* Settings = UDynamicRopeSettings::Get();
	const int32 PerRopeBudget = Settings ? FMath::Max(1, Settings->StaticBodyMaxCollidersPerRope) : 32;

	// 정적 월드 후보는 따로 모아 예산 초과 시 "가장 먼 것"부터 버린다(스켈레톤은 위에서 이미 무조건 포함).
	TArray<IRopeCollider*> WorldStaticCandidates;

	for (const FFrameProviderColliders& FP : FrameProviders)
	{
		// 자기 owner provider 제외 — 단 정적 월드 provider는 면제(정적 월드는 "던진 본인의 몸"이 아니므로,
		// 로프 소유 액터에 붙였다는 이유로 월드 충돌이 사라지면 안 된다).
		if (!FP.bWorldStatic && FP.Owner == OwnerToExclude && OwnerToExclude != nullptr)
		{
			continue;
		}
		if (!bCull || FP.Bounds.Num() != FP.Colliders.Num())
		{
			OutColliders.Append(FP.Colliders); // 컬 불가(빈 로프/bounds 캐시 불일치) → 전체 폴백(예산 우회, 드묾).
			continue;
		}
		for (int32 c = 0; c < FP.Colliders.Num(); ++c)
		{
			if (FP.Colliders[c] && FP.Bounds[c].IsValid && FP.Bounds[c].Intersect(RopeBounds))
			{
				if (FP.bWorldStatic)
				{
					WorldStaticCandidates.Add(FP.Colliders[c]); // 예산 적용 대상
				}
				else
				{
					OutColliders.Add(FP.Colliders[c]); // 스켈레톤 등 — 항상 포함
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

void URopeSimSubsystem::Tick(float DeltaTime)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SubsystemTick);

	// 무효 항목 정리.
	Ropes.RemoveAllSwap([](const TObjectPtr<URopeComponent>& Rope) { return !IsValid(Rope.Get()); });
	if (Ropes.Num() == 0)
	{
		return;
	}

	// G4: GPU가 유일 런타임 경로. 렌더 가능 RHI면 GPU, 아니면 CPU 폴백(자동). 감지도 GPU와 함께 켜진다.
	const bool bUseGPU = RopeGpuRuntimeAvailable();
	const bool bUseGPUContacts = bUseGPU; // GPU 솔브 시 감지도 GPU(별도 토글 없음).

	// GPU 상주(M5): RT 리드백이 채운 RopeId별 최신(약 1~2프레임 지연) 위치를 회수해 캐시. 아래 Phase 2에서
	// Free/Flight 로프의 Sim(렌더/충돌 미러)에 반영한다. 순차 의존성은 GPU 영속 버퍼 안에서 충족된다.
	if (bUseGPU)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_GPUGetLatest);
		GpuSolver.GetLatest(GpuLatest);
		if (bUseGPUContacts)
		{
			GpuSolver.GetLatestContacts(GpuLatestContacts); // G3: 접촉 감지 결과 회수(Finalize 전에 귀속).
		}
	}

	// Phase 1a (GT): collider 중앙 수집 — 등록된 provider에서 프레임당 1회 빌드 후 로프별 필터로 FrameColliders 채움.
	// (로프마다 월드를 스캔하던 것을 대체. collider 포인터는 provider 소유라 이번 프레임 solve/finalize 동안 유효.)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_GatherColliders);
		BuildFrameColliders();
		for (URopeComponent* Rope : Ropes)
		{
			GatherCollidersForRope(*Rope, Rope->FrameColliders);
		}
	}

	// Phase 1b (GT): 준비 — init/pin + 로직 phase 처리(collider는 위에서 이미 채워짐).
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_Prepare);
		for (URopeComponent* Rope : Ropes)
		{
			Rope->PrepareSimFrame(DeltaTime);
		}
	}

	// Phase 2: solver step (GPU 상주 / CPU 폴백). Free/Flight/Wrapped는 솔브, Wrapping/Releasing은
	// override-only, Contacting은 dispatch 없음 — 로프별 판정은 TryBuildResidentStep 안에 있다.
	if (bUseGPU)
	{
		// GPU 상주 경로(M5a). 로프별 영속 버퍼를 매 프레임 in-place로 전진(라운드트립 스톨/슬로모 없음).
		// whip(G1)과 로직 페이즈(G2 — Wrapping/Wrapped/Releasing)도 GPU 상주: 로직 산출물
		// (OverrideFrame)을 override 패스로 실어 재시드 없이 커널에서 적용한다. 적분이 없는
		// 로직 프레임은 NumSub=0 override-only dispatch. 접촉 감지는 Finalize가 지연 미러로 처리(G3).
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SolveGPU);

		TArray<FRopeGPUResidentStep> Steps;
		Steps.Reserve(Ropes.Num());
		int32 NumGdfRopes = 0; // Phase 2c: GDF 소비자 게이트 — 활성 GDF 로프 수(엔진 온디맨드 빌드 신호).
		for (URopeComponent* Rope : Ropes)
		{
			FRopeGPUResidentStep Step;
			if (TryBuildResidentStep(*Rope, DeltaTime, Step))
			{
				if (Step.bUseWorldGDF)
				{
					++NumGdfRopes;
				}
				Steps.Add(MoveTemp(Step));
			}
		}
		// 이 씬에 활성 GDF 로프가 있으면 커스텀 FX 시스템이 GDF를 요구 → 엔진이 온디맨드로 빌드한다.
		if (const UWorld* World = GetWorld())
		{
			RopeGDF::SetGDFActiveCount(World->Scene, NumGdfRopes);
		}
		if (Steps.Num() > 0)
		{
			// GDF 통합 경로(r.DynamicRope.GDFDispatchInVE, 기본 1)면 dispatch를 뷰 확장(씬 그래프)으로 미룬다(0이면 전용 그래프).
			if (RopeGDF::IsDispatchInVE())
			{
				GpuSolver.EnqueueSteps(MoveTemp(Steps));
			}
			else
			{
				GpuSolver.Step(MoveTemp(Steps));
			}
		}
	}
	else
	{
		// CPU 경로(기본): 로프는 서로 독립 + collider 스냅샷 read-only → 스레드 안전.
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SolveParallel);
		ParallelFor(Ropes.Num(), [this, DeltaTime](int32 Index)
		{
			Ropes[Index]->bGpuSteppedThisFrame = false; // CPU 경로 → resident 렌더 안 함(M5b).
			Ropes[Index]->SolveSimFrame(DeltaTime);
		});
	}

	// Phase 3 (GT): 마무리 — Flight 접촉 감지/캡처(UObject·이벤트) + 렌더 dirty.
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_Finalize);
		for (URopeComponent* Rope : Ropes)
		{
			// G3: GPU 감지 결과를 귀속해 Finalize의 Flight 접촉 소스를 GPU 후보로 채운다.
			// 게이트는 전역이 아니라 로프별 bGpuSteppedThisFrame — 이 프레임 실제로 GPU step된 로프만
			// GPU 감지를 쓴다. GPU step 못 한 로프(노드>256 등)는 CPU 솔브됐으므로 여기서도 GPU 후보를
			// 강제하지 않아, FinalizeSimFrame이 CPU 스윕 감지로 폴백한다(안 그러면 감지 자체가 누락돼 캡처 불가).
			Rope->bGpuContactsThisFrame = false;
			if (Rope->bGpuSteppedThisFrame && Rope->Phase == ERopePhase::Flight)
			{
				BuildGpuFlightCandidates(*Rope);
			}
			Rope->FinalizeSimFrame(DeltaTime);
		}
	}

	// 슬립(Free 정지 로프 솔브 스킵)/거리 LOD(iteration 감쇠)/gather 거리 컬링은 구현됨 — 컴포넌트
	// (UpdateSleepState/ComputeSolverLOD) + GatherCollidersForRope. TODO: 프레임당 총 솔브 비용 상한.
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
				SpawnParams.ObjectFlags |= RF_Transient; // 런타임 매니저 — 레벨에 저장하지 않는다.
				SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn; // 위치 무관(원점).
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
		RopeGDF::UnregisterSolver(World->Scene);
	}
	Super::Deinitialize();
}

void URopeSimSubsystem::BuildGpuFlightCandidates(URopeComponent& Rope)
{
	Rope.GpuFlightCandidates.Reset();

	const FRopeResidentContacts* Contacts = GpuLatestContacts.Find(Rope.GetUniqueID());
	if (!Contacts || Contacts->Generation != Rope.SimGeneration)
	{
		// 아직 회수분이 없거나 재시드 catch-up 중 — 이번 프레임은 GPU 후보 없음(캡처는 다음 프레임).
		Rope.bGpuContactsThisFrame = true; // 소스는 GPU(빈 후보) — CPU 스윕으로 되돌아가지 않는다.
		return;
	}

	// Contacts는 슬롯 순서(actual 먼저, predictive 뒤)라 actual이 우선 처리된다. CPU AddUniqueCandidate와
	// 동일하게 (node, bone, mesh) 중복은 병합한다(SourceMask OR + Source 우선순위 Guided>Actual>Free) —
	// 트래커의 노드 중복 카운트를 막고 판정을 CPU와 일치시킨다.
	for (const FRopeGPUContactResult& C : Contacts->Contacts)
	{
		// 콜라이더 인덱스 → (bone, mesh) 귀속. 범위 밖(콜라이더 집합 변화)은 건너뛴다(자기수정).
		const TArray<URopeComponent::FGpuColliderAttribution>& Attr =
			(C.ColliderType == 0) ? Rope.GpuCapsuleAttribution : Rope.GpuSdfAttribution;
		if (!Attr.IsValidIndex(C.ColliderIndex))
		{
			continue;
		}
		const URopeComponent::FGpuColliderAttribution& A = Attr[C.ColliderIndex];
		if (A.Bone.IsNone())
		{
			continue; // 귀속 불가(비-스켈레탈 collider) — 캡처 대상 아님.
		}
		const USkeletalMeshComponent* Mesh = A.Mesh.Get(); // weak — 지연 중 파괴됐으면 null(판정은 bone으로 진행).

		// 병합: 같은 (node, bone, mesh) 후보가 있으면 SourceMask OR + Source 우선순위 갱신, 새 후보는 추가 안 함.
		FRopeContactCandidate* Existing = nullptr;
		for (FRopeContactCandidate& E : Rope.GpuFlightCandidates)
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
		Cand.WrapDirectionScore = 0.0f; // EvaluateRelativeMotion(GT)이 채운다.
		Rope.GpuFlightCandidates.Add(Cand);
	}
	Rope.bGpuContactsThisFrame = true;
}

bool URopeSimSubsystem::SyncGpuPositionsForHandoff(URopeComponent& Rope)
{
	if (!RopeGpuRuntimeAvailable())
	{
		return false; // CPU 폴백 — Sim이 이미 최신.
	}

	FRopeSimState& S = Rope.Sim;
	TArray<FVector> Pos;
	TArray<FVector> Prev;
	uint32 Generation = 0;
	if (!GpuSolver.ReadbackNow(Rope.GetUniqueID(), Pos, Prev, Generation))
	{
		return false; // 상주 버퍼 없음(GPU로 step된 적 없음) — 미러가 곧 진실.
	}
	if (Generation != Rope.SimGeneration || Pos.Num() != S.Num() || Prev.Num() != S.Num())
	{
		return false; // 재시드 catch-up 중이거나 노드 수 불일치 — stale 적용 방지.
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

	// GPU 상주 대상: 솔브 프레임(Free/Flight/Wrapped) 또는 로직 산출물이 있는 프레임(Wrapping/Releasing —
	// override-only). bSolveThisFrame은 Prepare에서 Free/Flight/Wrapped에서만 true라 별도 phase 체크가 필요 없다.
	// Contacting(산출물 없음)은 dispatch 자체가 없어 GPU 버퍼가 동결 유지된다(CPU의 "솔브 없음"과 동일).
	const bool bGpuRope = (Rope.bSolveThisFrame || Rope.OverrideFrame.HasAny())
		&& S.Num() >= 2 && S.Num() <= FRopeGPUSolver::MaxNodes;
	// M5b: 이 프레임에 GPU step되는 로프만 렌더가 resident PosBuf를 직접 읽는다(아니면 stale → CPU 미러).
	Rope.bGpuSteppedThisFrame = bGpuRope;
	if (!bGpuRope)
	{
		// 폴백(노드수 초과 등): CPU 솔브. logic phase는 bSolveThisFrame=false라 자동 스킵.
		if (Rope.bSolveThisFrame)
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
		if (L->Generation == Rope.SimGeneration && L->NumNodes == S.Num()
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
	if (Rope.OverrideFrame.HasAny())
	{
		Rope.OverrideFrame.ApplyToSim(S);
	}

	// 고정-timestep 스케줄(CPU accumulator). 로직 프레임(bSolveThisFrame=false)은 적분 없이
	// override만 기록한다(NumSub=0) — CPU 경로의 "솔브 없음"과 동일한 시간 처리.
	FRopeSubstepSchedule Schedule;
	Schedule.NumSub = 0;
	Schedule.FixedDt = 0.0f;
	if (Rope.bSolveThisFrame)
	{
		Schedule = RopeSolverSubsteps(S, Rope.SolverConfig, DeltaTime);
	}

	// 상주 step 구성(self-contained). 시드 데이터는 매 프레임 제공(RT는 재시드 시에만 GPU 업로드).
	SeedResidentStep(OutStep, RopeId, Rope.SimGeneration, S, Rope.SolverConfig, Schedule);
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
	if (Rope.OverrideFrame.HasAny() && Rope.OverrideFrame.Flags.Num() == S.Num())
	{
		OutStep.OverrideFlags = Rope.OverrideFrame.Flags;
		OutStep.OverridePositions = Rope.OverrideFrame.Positions;
		OutStep.OverridePrevPositions = Rope.OverrideFrame.PrevPositions;
		OutStep.OverrideInvMass = Rope.OverrideFrame.InvMass;
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
	Step.ContactRadius = Rope.WrapConfig.ContactRadius;
	Step.PredictionFrames = Rope.WrapConfig.PredictiveContactFrames;
	Rope.GpuCapsuleAttribution.Reset();
	Rope.GpuSdfAttribution.Reset();

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

void URopeSimSubsystem::PackStepColliders(URopeComponent& Rope, bool bDetectThisRope, FRopeGPUResidentStep& Step) const
{
	// 감지 시 collider 인덱스 → bone/mesh 귀속을 Step.Capsules/SDFColliders와 같은 순서로 병행 채운다.
	auto MakeAttribution = [](IRopeCollider* Collider)
		{
			URopeComponent::FGpuColliderAttribution Attr;
			const USkeletalMeshComponent* Mesh = nullptr;
			Collider->GetGPUAttribution(Attr.Bone, Mesh);
			Attr.Mesh = Mesh;
			return Attr;
		};

	// FrameColliders는 Prepare에서 GT gather된 스냅샷. 2-pass: 비-정적(스켈레탈) collider를 먼저,
	// 정적(월드) collider를 뒤에 패킹한다. 감지(detect) 커널은 capsule을 [0, NumDetectCapsules)만
	// 보므로 정적 캡슐이 감지에서 자동 제외된다 — 감지는 노드당 최심 접촉 1개만 남겨, 벽 접촉이
	// 본 접촉을 가리면 랩 캡처가 조용히 실패하기 때문(박스는 감지 커널에 아예 없다). solve는 전부 본다.
	for (IRopeCollider* Collider : Rope.FrameColliders)
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
				Rope.GpuCapsuleAttribution.Add(MakeAttribution(Collider));
			}
			continue;
		}
		FRopeSDFColliderView View;
		if (Collider->GetGPUSDF(View))
		{
			Step.SDFColliders.Add(MakeGpuSdf(View));
			if (bDetectThisRope)
			{
				Rope.GpuSdfAttribution.Add(MakeAttribution(Collider));
			}
		}
	}
	Step.NumDetectCapsules = Step.Capsules.Num(); // 감지 경계: 여기까지가 비-정적 캡슐.

	// pass 2: 정적(월드) collider — solve 전용. 캡슐(스피어/스필)은 감지 경계 뒤에 append,
	// 박스는 전용 배열. 귀속 테이블은 인덱스 정렬 유지를 위해 정적 캡슐 분도 채운다(None/null —
	// 감지 커널이 경계 밖 인덱스를 emit하지 않으므로 방어적).
	for (IRopeCollider* Collider : Rope.FrameColliders)
	{
		if (!Collider || !Collider->IsWorldStatic())
		{
			continue;
		}
		FRopeGPUCapsule Cap;
		if (Collider->GetGPUCapsule(Cap.A, Cap.B, Cap.Radius))
		{
			Step.Capsules.Add(Cap); // 정적 — 프레임 모션 없음(InvDt 0 기본값).
			if (bDetectThisRope)
			{
				Rope.GpuCapsuleAttribution.Add(MakeAttribution(Collider));
			}
			continue;
		}
		FRopeGPUBox Box;
		if (Collider->GetGPUBox(Box.Center, Box.Rot, Box.HalfExtents))
		{
			// 프레임 모션(prev center/rot + InvDt): 표면 속도 드래그/상대 운동 CCD. 정적이면 기본값(InvDt 0) 유지.
			Collider->GetGPUBoxMotion(Box.PrevCenter, Box.PrevRot, Box.InvDeltaTime);
			Step.Boxes.Add(Box);
			continue;
		}
		TConstArrayView<FPlane> LocalPlanes;
		FBox LocalBounds(ForceInit);
		FQuat CvRot, CvPrevRot;
		FVector CvTrans, CvPrevTrans;
		float CvInvDt = 0.0f;
		if (Collider->GetGPUConvex(LocalPlanes, LocalBounds, CvRot, CvTrans, CvPrevRot, CvPrevTrans, CvInvDt)
			&& LocalPlanes.Num() > 0 && LocalBounds.IsValid)
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
				Step.ConvexPlanes.Add(FVector4(Pl.X, Pl.Y, Pl.Z, Pl.W)); // 로컬·단위·바깥, PlaneDot=dot(N,p)-W
			}
			Step.Convexes.Add(Cv);
		}
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
