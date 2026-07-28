// Copyright Epic Games, Inc. All Rights Reserved.

#include "Subsystem/RopeSimSubsystem.h"
#include "RopeComponent.h"
#include "DynamicRopeLog.h"
// RopeSolverSubsteps
#include "Solver/RopeXPBDSolver.h"
// IRopeCollider::GetGPUCapsule
#include "Collision/RopeCollider.h"
// IRopeColliderProvider (central collider gather)
#include "Collision/RopeColliderProvider.h"
// FRopeGPUSolver / FRopeGPUResidentStep / FRopeGPUCapsule (DynamicRopeShaders module)
#include "RopeGPUSolver.h"
// RopeGDF::RegisterSolver / SetGDFActiveCount (GDF integration path)
#include "RopeGPUSolverRegistry.h"
// StaticBodyControllerClass / StaticBodyMaxColliders (auto-spawn)
#include "Settings/DynamicRopeSettings.h"
// ARopeController(static body provider host)
#include "Collision/RopeController.h"
// Inject MaxColliders when spawning base class
#include "Collision/RopeStaticBodyProvider.h"
#include "Engine/World.h"
// FSceneInterface (Scene→solver registration key)
#include "SceneInterface.h"
// AActor::GetOwner (provider source filtering)
#include "GameFramework/Actor.h"
#include "Camera/PlayerCameraManager.h"
#include "Kismet/GameplayStatics.h"
#include "Components/ActorComponent.h"
// Tick prerequisite (guaranteed after animation evaluation)
#include "Components/SkeletalMeshComponent.h"
#include "Async/ParallelFor.h"
#include "Debug/RopeStats.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"
// GDynamicRHI
#include "RHI.h"
// FApp::CanEverRender
#include "Misc/App.h"
// TObjectIterator (scans existing providers before auto-spawning)
#include "UObject/UObjectIterator.h"
// GEngine->AddOnScreenDebugMessage (duplicate warning)
#include "Engine/Engine.h"
// TAutoConsoleVariable(CPU solve force toggle)
#include "HAL/IConsoleManager.h"

namespace
{
	// Force toggle CPU solve for debug/profiling. If it is 1, even if there is a renderable RHI, the GPU resident path is turned off and the CPU is
	// Goes down to fallback solver+detection (solve·detection·handoff synchronization together consistently switches to CPU path — tube is rope-specific)
	// bGpuSteppedThisFrame becomes false and automatically falls back to the CPU mirror centerline). For verification/performance comparison compared to GPU. Default 0.
	static TAutoConsoleVariable<int32> CVarForceCPUSolve(
		TEXT("r.DynamicRope.ForceCPUSolve"),
		0,
		TEXT("1이면 GPU가 가용해도 로프 솔브/감지를 CPU 경로로 강제한다(디버그·비교용). 0=자동 선택(기본)."),
		ECVF_Default);

	// G4: GPU is the only runtime path. GPU-resident solve+detection if there is a renderable RHI, otherwise (cook/-nullrhi/
	// Server build) Automatically falls back to CPU solve+detection. The only client toggle above is r.DynamicRope.ForceCPUSolve
	// (force CPU for debug) — GPU is usually THE path.
	// FRopeXPBDSolver is maintained for this fallback and parity testing (effectively unused in the runtime client).
	bool RopeGpuRuntimeAvailable()
	{
		// If the Force CPU toggle is on, it falls back to CPU fallback regardless of GPU availability.
		if (CVarForceCPUSolve.GetValueOnGameThread() != 0)
		{
			return false;
		}
		// capable of rendering RHI + SM5 or higher (kernel is compiled with SM5 guard only) — check that RopeGPU::IsRuntimeSupported is
		// is a single source of truth (bones the same function as the GPU tube gate in the scene proxy).
		return RopeGPU::IsRuntimeSupported();
	}

	// Duplicate world-static provider warning: log + screen mesh (editor/development build). "maximum 1 per world" invariant
	// Notice this so you don't break it quietly — the second provider will be ignored and the user needs to know why.
	void WarnDuplicateWorldStaticProvider(const AActor* Offender)
	{
		UE_LOG(LogRopeCollision, Warning,
			TEXT("정적 월드 콜라이더 프로바이더가 이미 존재합니다 — %s의 중복 프로바이더는 무시됩니다(월드당 1개만 사용)."),
			*GetNameSafe(Offender));
#if !UE_BUILD_SHIPPING
		if (GEngine)
		{
			// key is pinned (a constant instead of GetTypeHash) so that it is updated only once per event rather than every frame.
			GEngine->AddOnScreenDebugMessage(uint64(0x0D0ED1CA), 8.0f, FColor::Yellow,
				FString::Printf(TEXT("[DynamicRope] 중복 정적 바디 프로바이더 무시됨(%s) — 월드당 1개만 사용됩니다."),
					*GetNameSafe(Offender)));
		}
#endif
	}

	// SDF collider view (Runtime Collision) → Flat copy SDF collider (Shaders) for GPU upload.
	// If the field increases, you only need to update this one place (17 lines that were scattered in the tick loop in the past).
	FRopeGPUSDFCollider MakeGpuSdf(const FRopeSDFColliderView& View)
	{
		FRopeGPUSDFCollider Sdf;
		// Code byte blob (dequant when flattening upload)
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
		// Drag GPU CCD/surfacevelocity.
		Sdf.PrevBoneToWorld = View.PrevBoneToWorld;
		Sdf.InvDeltaTime    = View.InvDeltaTime;
		Sdf.VolumeKey       = View.VolumeKey;
		return Sdf;
	}

	// Fill in the solver seed/parameters in the Step (collider·override·whip packing is added in the call section).
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
		// CollisionRadius requires auto(0=render Radius) interpretation, so the caller uses Rope.GetEffectiveCollisionRadius.
		// is covered (bUseWorldGDF is also moved directly under the component and is under the jurisdiction of the call department — surface audit CL-4).
		Step.CollisionRadius   = Cfg.CollisionRadius;
		Step.Friction          = Cfg.Friction;
		Step.TipFrictionScale  = Cfg.TipFrictionScale;
		Step.SweepStep         = Cfg.SweepStep;
		Step.MaxSweepSamples   = Cfg.MaxSweepSamples;
		Step.NumSub            = Schedule.NumSub;
		Step.FixedDt           = Schedule.FixedDt;
	}
}

// FRopeNodeOverrideFrame (Core module) bits must be numerically 1:1 with ERopeGPUOverride (Shaders module) —
// The constants are mirrored so that Core does not depend on Shaders, and are verified here (where both are visible).
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
			// Re-entry during tick traversal (handler spawns rope actor) — postpones transformation (header bTickingRopes annotation).
			DeferredRopeUnregister.RemoveSingleSwap(Rope);
			DeferredRopeRegister.AddUnique(Rope);
			return;
		}
		Ropes.AddUnique(Rope);
		// The owner of the GPU-resident resource is also recorded by ID — the only clue to retrieval if the component disappears without formal release.
		RegisteredRopeIds.Add(Rope->GetUniqueID());
		// Because the hand pin (socket attachment) follows the possessing character pose.
		SetAnimPrerequisites(Rope, /*bAdd*/ true);
		UE_LOG(LogDynamicRope, Verbose, TEXT("RegisterRope: %s (%d total)"), *Rope->GetName(), Ropes.Num());
	}
}

void URopeSimSubsystem::UnregisterRope(URopeComponent* Rope)
{
	if (bTickingRopes)
	{
		// Re-entry during tick traversal (handler destroys rope actor) — actual removal is deferred to ApplyDeferredRopeChanges. this time
		// For the remaining traversal of the frame, the IsValid guard skips this rope (destroy → pending-kill) (header bTickingRopes annotation).
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
		// Free GPU resident buffer/readback (in render thread). Remove from both caches and ID ledger.
		// (GpuLatestContacts was previously missing, so a dead rope's contact snapshot remained throughout the world).
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

	// Create a set of IDs for live ropes, and IDs remaining only in the ledger = bone ropes that have not been officially unlocked.
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
		// The animation prerequisites cannot be solved here (the component already exists, so the owned mesh cannot be traced back). FTickPrerequisite
		// Dead mesh items that are weak are automatically skipped, so it is harmless if they remain, and if the mesh is alive, the component
		// This means that it has gone through official EndPlay, so it does not come to this path.
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
	// Order: Release first, register later (ropes spawned and destroyed in the same tick also convergence to the final state). bTickingRopes already has
	// is false, so the call below performs the actual Ropes transformation/GPU release (Register/Unregister does not fire delegates)
	// , so there is no additional re-entry here).
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

	// Anti-duplicate backstop (fragment 2): Only one world-static provider per world. If something is already registered, the second
	// Reject + Warning. Enforces invariants based on the interface, regardless of the source (manual placement/automatic spawning/runtime).
	// Priority is "first registered wins" (auto-spawn gives way to the existing one on piece 1, so manual wins).
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
					// Registration refused — GatherColliders for this provider will not be called.
					return;
				}
			}
		}
	}

	ColliderProviders.AddUnique(Provider);
	// Because the bone collider (capsule/SDF) reads the owning character pose.
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
	// Ensures "simulate rope after animation evaluation": skeletal mesh ticks of source component owning actor in SimTickFunction
	// is set as a prerequisite. Mesh tick completion is done through the parallel animation completion task as DontCompleteUntil.
	// (SkeletalMeshComponent::DispatchParallelEvaluationTasks) This frame's pose (buffer
	// flip) is guaranteed. Even if the same mesh is registered repeatedly in both rope/provider, AddPrerequisite is unique.
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
			// Only actually called on the first consumer (AddPrerequisite itself is unique, so duplicate calls are harmless, but
			// If you do not count, the remaining consumer share is erased upon release — header comment).
			int32& RefCount = AnimPrereqRefCount.FindOrAdd(Mesh);
			if (++RefCount == 1)
			{
				SimTickFunction.AddPrerequisite(Mesh, Mesh->PrimaryComponentTick);
			}
		}
		else if (int32* RefCount = AnimPrereqRefCount.Find(Mesh))
		{
			// Released only when the last consumer leaves.
			if (--(*RefCount) <= 0)
			{
				AnimPrereqRefCount.Remove(Mesh);
				SimTickFunction.RemovePrerequisite(Mesh, Mesh->PrimaryComponentTick);
			}
		}
	}

	if (bAdd)
	{
		// If the actor dies without being officially released, the key expires and only the count remains. The prerequisite itself is weak and harmless, but
		// Clean the map once every add to prevent it from growing indefinitely (release path does not increase cost).
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
	// This is an aiming region request, but if there is no ray/active target, collection itself is not necessary — invalid box (= empty the list).
	if (bIncludeAimRay && !bHasAimRayBounds && !bHasLockedTargetBounds)
	{
		return FBox(ForceInit);
	}

	// rope tight AABB (Pos∪Prev — including frame motion) + margin. provider region and per-rope collider culling
	// shares the same box (called by both GatherCollidersForRope and BuildFrameColliders).
	FBox RopeBounds(ForceInit);
	// For calculating predicted contact (forward extrapolation) margin — maximum node displacement this frame.
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
		* FMath::Max(Rope.WrapConfig.PredictiveContactFrames, 1.0f);
	const float QueryMargin = BaseMargin + PredictiveMotionMargin;
	if (RopeBounds.IsValid)
	{
		// Margin: Contact query radius + sweep margin + forward extrapolation distance of predicted contact (frame displacement × predicted frame).
		// Take plenty — over-culling margin is safe (it just adds a few more colliders).
		RopeBounds = RopeBounds.ExpandBy(QueryMargin);
	}
	if (bIncludeAimRay)
	{
		// After aiming is over, in the frame where only active aim lock remains, a huge AABB between rope↔target is not created.
		// Re-collects only the area around the previous target collider bounds. The results are promoted through the component target filter.
		if (!bHasAimRayBounds && bHasLockedTargetBounds)
		{
			return Rope.SimFrame.LockedTargetColliderQueryBounds.ExpandBy(QueryMargin);
		}
		// Aiming region only: The preview ray can pass through areas away from the current rope centerline. This section
		// If you do not merge, even if the ray passes through the SDF, the collider is not in the aiming list and becomes a cyan miss.
		// It is a union that covers the surrounding area of the rope, so the range seen by the aiming query (hit check/preview arc search) is
		// Same as before separation — only the FrameColliders side used for physics and debug is narrowed.
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

	// Active region list — The front N are physical regions (1:1 with the Ropes index), and the back N are of the same rope.
	// aiming region(AimRegionIndexOf). Invalid/empty ropes and ropes that are not aiming remain in place as the !IsValid box —
	// Ensure that the region mapping index returned by the provider corresponds to this index (the provider sets !IsValid to
	// is skipped). The bounds-aware provider (static body) fills the empty space between distant ropes with this list.
	// Excluded from scanning. Same as GatherCollidersForRope below box(single source of truth).
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

	// Region processing priority: active rope first, and physical region before aiming region. global extraction cap
	// exhausts its budget on a first-come, first-served basis, the next region is used in the frame where the cap is applied.
	// Can't get scanned — then just rearrange the order so that the starving side doesn't become the "actually simulated rope"
	// (index immutable → mapping unaffected). The aiming region is for HUD/preview display, so it is postponed to physics.
	// Keys: 0 = busy phase (Flight~Releasing), 1 = Free awake, 2 = Free sleep, 3 = invalid region.
	// The aiming region is +4 here (invalid is still 7) and is placed behind the entire physical region.
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

	// Gather once per registered provider (once per-frame — regardless of the number of ropes). Dead providers are cleaned up.
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
		// static world providers are exempt from owner exclusion.
		FP.bWorldStatic = Provider->ProvidesWorldStaticColliders();
		FP.Colliders = MoveTemp(Gather.Colliders);
		// The source actor for each collider is also trusted only when the length is correct — if it is misaligned, the index gets tangled and the wrong collider is used.
		// is excluded, leave it empty and fall back to the provider-level check.
		if (Gather.ColliderSourceActors.Num() == FP.Colliders.Num())
		{
			FP.SourceActors = MoveTemp(Gather.ColliderSourceActors);
		}
		// region mapping is only trusted if length matches the number of ropes (mismatch = provider bug → demoted to bounds re-curl fallback).
		FP.bHasRegionMapping = Gather.bHasRegionMapping
			&& Gather.RegionColliderIndices.Num() == FrameRopeRegions.Num();
		if (FP.bHasRegionMapping)
		{
			FP.RegionIndices = MoveTemp(Gather.RegionColliderIndices);
		}
		else
		{
			// fallback path only: Cache world bounds per collider once per frame — re-curl per rope as many times as the number of ropes
			// Avoid recalculation with virtual calls.
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

	// Basic: Collisions with all providers in the world, excluding its own provider (prevents self-tangle when throwing).
	// Grabbing the body of another actor (cross-actor) is automatic because that actor is included in the “whole”. Opt in if owner collision is required.
	const AActor* OwnerToExclude = Rope.bIncludeOwnerColliders ? nullptr : Rope.GetOwner();

	// Distance culling: Colliders that do not overlap with rope AABB (Pos∪Prev — including frame motion) are not Loaded at all.
	// The CPU solver has its own broad-phase, but the GPU kernel loops the entire collider for each node.
	// Filtering here is the key to scaling (capsules/SDFs of distant characters are not included in the step).
	// The default path consumes the region mapping returned by the provider when it gathers (no re-curl —
	// 2026-07 Collection Method Changes). Same box (single source of truth) as the region passed by BuildFrameColliders to the provider.
	const FBox RopeBounds = FrameRopeRegions.IsValidIndex(RegionIndex) ? FrameRopeRegions[RegionIndex] : FBox(ForceInit);
	const bool bCull = RopeBounds.IsValid != 0;

	// Static world collider budget per rope. Apart from the global extraction cap (StaticBodyMaxColliders), this rope is used to solve
	// Limits the number of static world colliders to be independent for each rope (a distant rope cannot use this rope's budget).
	// Skeleton colliders (capsule/SDF) are naturally limited by the number of bones and are the core of the wrap, so they are excluded from the budget — directly to OutColliders.
	const UDynamicRopeSettings* Settings = UDynamicRopeSettings::Get();
	const int32 PerRopeBudget = Settings ? FMath::Max(1, Settings->StaticBodyMaxCollidersPerRope) : 32;

	// static world candidates are collected separately and discarded from the "furthest ones" when budget is exceeded (skeletons are already unconditionally included above).
	TArray<IRopeCollider*> WorldStaticCandidates;

	// Excluding collider (=body) unit owner. Static world providers are exempt from provider-level exclusion below.
	// The only thing the exemption targets is "world geometry such as floors/pillars". The same provider scans the world and becomes a rope
	// If you grab a shape (tether proxy, tip mesh, held weapon, etc.) attached to the owning actor, it will follow the rope and control it.
	// becomes a push-out collider that pushes the rope — select only those as the source actor. Providers that do not provide a source
	// is an empty array, so it is always false (= as is the existing provider unit check).
	auto IsOwnBodyCollider = [OwnerToExclude](const FFrameProviderColliders& P, int32 Index)
	{
		return RopeColliderGather::IsExcludedOwnerBody(P.SourceActors, Index, OwnerToExclude);
	};

	for (const FFrameProviderColliders& FP : FrameProviders)
	{
		// Excluding self-owner providers — However, static world providers are exempt (since static world is not the “body of the person who threw it”)
		// world collision should not disappear just because it is attached to a rope-owned actor). The shape of one's body mixed with the exemption
		// IsOwnBodyCollider above filters by collider.
		if (!FP.bWorldStatic && FP.Owner == OwnerToExclude && OwnerToExclude != nullptr)
		{
			continue;
		}
		if (!bCull)
		{
			// rope without region (empty sim, etc.) → full fallback (budget bypass, rare — retain existing behavior).
			for (int32 c = 0; c < FP.Colliders.Num(); ++c)
			{
				if (FP.Colliders[c] && !IsOwnBodyCollider(FP, c))
				{
					OutColliders.Add(FP.Colliders[c]);
				}
			}
			continue;
		}

		// Default path: consume region(=this rope) mapping created by provider — no bounds retest.
		if (FP.bHasRegionMapping)
		{
			if (!FP.RegionIndices.IsValidIndex(RegionIndex))
			{
				// A defense line that is not reached because length is verified in the build.
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
					// Budget Applicable to
					WorldStaticCandidates.Add(Collider);
				}
				else
				{
					// skeleton, etc. — always included
					OutColliders.Add(Collider);
				}
			}
			continue;
		}

		// fallback path (provider without mapping): Re-curl collider bounds in the old fashion.
		if (FP.Bounds.Num() != FP.Colliders.Num())
		{
			// bounds Cache mismatch → full fallback (rare).
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
					// Budget Applicable to
					WorldStaticCandidates.Add(FP.Colliders[c]);
				}
				else
				{
					// skeleton, etc. — always included
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

	// Exceeded budget: Only the top PerRopeBudgets are Loaded in order of proximity to the actual node of this rope (dropped from the furthest).
	// Proximity is the least squares distance between the collider world bounds center and rope nodes — calculated only once per candidate (avoiding recalculation during alignment).
	// This path is a rare path that only runs in budget-exceeded frames.
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
		// If aiming is finished or if aiming is not in progress in the first place, the list is cleared — the last frame provider pointer is
		// remains, the next aiming query will read storage that has already been destroyed.
		Rope.SimFrame.AimFrameColliders.Reset();
		return;
	}
	GatherCollidersForRope(Rope, RegionIndex, Rope.SimFrame.AimFrameColliders);
}

bool URopeSimSubsystem::RefreshAimFrameCollidersForImmediateQuery(URopeComponent& /*Rope*/)
{
	// no-op for ABI/source compatibility. If BuildFrameColliders is called outside of the normal tick, provider once/frame
	// Because the contract and multi-wielder region consistency are broken again, the path is not restored immediately.
	return false;
}

void URopeSimSubsystem::Tick(float DeltaTime)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SubsystemTick);
	SCOPE_CYCLE_COUNTER(STAT_RopeSim_Tick);

	// Cleaning up invalid items. The rope that disappears here has not gone through UnregisterRope (the actor has not been
	// is destroyed), GPU-resident resources remain — de-array and resource recovery are handled in one piece.
	Ropes.RemoveAllSwap([](const TObjectPtr<URopeComponent>& Rope) { return !IsValid(Rope.Get()); });
	ReleaseGpuResourcesForDeadRopes();
	if (Ropes.Num() == 0)
	{
		// If the GDF demand is not lowered in the frame where the last rope disappears (previously, setGDFActiveCount below
		// returned before it was reached) The engine continues to build a Global Distance Field that no one uses.
		if (const UWorld* World = GetWorld())
		{
			RopeGDF::SetGDFActiveCount(World->Scene, 0);
		}
		return;
	}

	// G4: GPU is the only runtime path. Render possible: GPU if RHI, otherwise CPU fallback (automatic). Detection is also turned on with the GPU.
	const bool bUseGPU = RopeGpuRuntimeAvailable();
	// When solving GPU, detection is also performed on GPU (no separate toggle).
	const bool bUseGPUContacts = bUseGPU;

	// 'stat DynamicRope' frame Aggregation for dashboard. NumGdfRopes/TotalFrameColliders in GPU/gather loop below
	// is collected (a value known only to the subsystem), and the remaining phase/solve path counters are counted by RecordFrameStats as a public getter.
	int32 NumGdfRopes = 0;
	int32 TotalFrameColliders = 0;

	// GPU resident (M5): Retrieve and cache the latest (approximately 1-2 frame delay) location for each RopeId filled by RT readback. In Phase 2 below
	// It is reflected in the Sim (render/collision mirror) of the Free/Flight rope. Sequential dependencies are satisfied within the GPU persistent buffer.
	if (bUseGPU)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_GPUGetLatest);
		GpuSolver.GetLatest(GpuLatest);
		// Recovers the simulation time of the replaced step without being consumed by the view expansion (TryBuildResidentStep below returns it).
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
			// G3: Retrieve contact detection results (attribution before finalize).
			GpuSolver.GetLatestContacts(GpuLatestContacts);
		}
	}

	// Delays Ropes transformation of Register/UnregisterRope during rope traversal below (reentrant guard — header comment).
	// Prepare for rope spawning/destruction by delegate handler fired by ResolvePendingAimThrow/Prepare/Finalize.
	bTickingRopes = true;

	// Phase 1a (GT): Collider central collection — Build once per frame from registered provider and fill FrameColliders with rope-specific filters.
	// (Replaces scanning the world for each rope. The collider pointer is owned by the provider, so it is valid during this frame solve/finalize.)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_GatherColliders);
		SCOPE_CYCLE_COUNTER(STAT_RopeSim_Gather);
		BuildFrameColliders();
		// rope index = physical region index of the FrameRopeRegions/provider mapping (order pinned after invalid cleanup above).
		for (int32 RopeIndex = 0; RopeIndex < Ropes.Num(); ++RopeIndex)
		{
			// Rope destroyed (pending-kill) by re-entry before this frame is skipped. FrameRopeRegions is !IsValid
			// position is maintained, the index correspondence remains the same (the transformation is delayed and the order is unchanged).
			URopeComponent* Rope = Ropes[RopeIndex];
			if (!IsValid(Rope))
			{
				continue;
			}
#if WITH_GAMEPLAY_DEBUGGER
			// The point at which this frame rope is first touched — ResolvePendingAimThrow below can create a Flight transition.
			// , solidify the frame start phase before that (for “Start→End” indication in the debugger header).
			Rope->CaptureDebugFrameStartPhase();
#endif
			GatherCollidersForRope(*Rope, RopeIndex, Rope->SimFrame.FrameColliders);
			// The aiming list is collected separately in a separate region (rope AABB ∪ aim ray) — Bone of the distant aiming target
			// Separation contract (FRopeSimFrameIO::AimFrameColliders) to prevent colliders from leaking into the physics list above.
			GatherAimCollidersForRope(*Rope, RopeIndex);
			// The HUD/preview request registered by Wielder in PrePhysics is also confirmed here. The next Wielder tick produces this result:
			// is consumed, so there is a delay of up to 1 frame, but BuildFrameColliders is not called again because of the HUD.
			Rope->ResolvePendingAimQuery();
			// ③ Actual input does not use the HUD cache. At the moment of input, the ray is prepared in the same frame aiming list.
			// Confirm, if it is an immediate execution request, it is thrown here. If it is a montage path, the result is stored until notify.
			Rope->ResolvePendingGuaranteedAimThrow();
			// Aim throw is confirmed immediately after collider is collected with pinned ray bounds at the moment of input.
			// Thanks to this ordering, the hit or FrameForward fallback is determined by the latest aiming list of the same request.
			Rope->ResolvePendingAimThrow();
			// Aim ray locks mesh+bone and removes other bone colliders here.
			// Actual/predicted contact and Wrapping path always use this result. General solve also uses this list, but
			// The collision-Free Aim Flight solve intentionally ignores the list to only solve for distance/bend.
			Rope->FilterFrameCollidersForAimWrapTarget();
			TotalFrameColliders += Rope->SimFrame.FrameColliders.Num();
		}
	}

	// Phase 1b (GT): Preparation — init/pin + logic phase processing (collider already populated above).
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
			// Since all ropes use the same local player camera, only one query per frame is performed on the first rope needed.
			// Lookup failures are also remembered as resolved and do not iterate as many times as the number of ropes in a world without a server/camera.
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

	// Phase 2: solver step (GPU resident / CPU fallback). Solve Free/Flight/Wrapping/Wrapped,
	// Releasing is override-only, Contacting is not dispatched — check for each rope is in TryBuildResidentStep.
	if (bUseGPU)
	{
		// GPU resident path(M5a). Advances the persistent buffer for each rope every frame in-place (no round trip stall/slomo).
		// whip (G1) and logic phase (G2 — Wrapping/Wrapped/Releasing) are also GPU resident: logic output
		// (OverrideFrame) into the override pass and apply it in the kernel without reseeding. without integral
		// logic frame NumSub=0 override-only dispatch. Contact detection is handled by Finalize with a delayed mirror (G3).
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SolveGPU);
		SCOPE_CYCLE_COUNTER(STAT_RopeSim_Solve);

		TArray<FRopeGPUResidentStep> Steps;
		Steps.Reserve(Ropes.Num());
		// Phase 2c: GDF consumer gate — Number of active GDF ropes (Engine build-on-demand signal). NumGdfRopes hoists from the top of the tick.
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
				// Contacting does not create a new GPU step, but ReadbackNow may have pending Scene GDF pending.
				// . The GDF demand must be maintained until the handoff is completed before the next view dispatch consumes that step.
				++NumGdfRopes;
			}
		}
		// If there is an active GDF rope in this scene, the custom FX system requires a GDF → the engine builds it on demand.
		if (const UWorld* World = GetWorld())
		{
			RopeGDF::SetGDFActiveCount(World->Scene, NumGdfRopes);
		}
		if (Steps.Num() > 0)
		{
			// Dispatch is deferred to view expansion (scene graph, PreRenderBasePass) — timing when GDF parameters are valid.
			GpuSolver.EnqueueSteps(MoveTemp(Steps));
		}
	}
	else
	{
		// CPU path (default): ropes are independent from each other + collider snapshot read-only → thread safe.
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SolveParallel);
		SCOPE_CYCLE_COUNTER(STAT_RopeSim_Solve);
		ParallelFor(Ropes.Num(), [this, DeltaTime](int32 Index)
		{
			URopeComponent* Rope = Ropes[Index];
			if (!IsValid(Rope))
			{
				return;
			}
			// CPU path → Do not render resident (M5b).
			Rope->SimFrame.bGpuSteppedThisFrame = false;
			Rope->SolveSimFrame(DeltaTime);
		});
	}

	// Phase 3 (GT): Finalization — Flight contact detection/capture (UObject·Event) + render dirty.
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_Finalize);
		SCOPE_CYCLE_COUNTER(STAT_RopeSim_Finalize);
		for (URopeComponent* Rope : Ropes)
		{
			if (!IsValid(Rope))
			{
				continue;
			}
			// G3: Attribute the GPU detection results and fill Finalize's Flight contact source with GPU candidates.
			// gate is not global, but per-rope bGpuSteppedThisFrame — Only the rope that actually GPU stepped in this frame
			// Uses GPU detection. The ropes (node>MaxNodes, etc.) that did not make the GPU step were solved by the CPU, so the GPU candidate is also used here.
			// is not enforced, so FinalizeSimFrame falls back to CPU sweep detection (otherwise the detection itself will be missed and cannot be captured).
			Rope->SimFrame.bGpuContactsThisFrame = false;
			if (Rope->SimFrame.bGpuSteppedThisFrame && Rope->Phase == ERopePhase::Flight)
			{
				BuildGpuFlightCandidates(*Rope);
			}
			Rope->FinalizeSimFrame(DeltaTime);
		}
	}

	// Slip (Free stationary rope solve skip)/distance LOD (iteration damping)/gather distance culling implemented — component
	// (UpdateSleepState/ComputeSolverLOD) + GatherCollidersForRope. TODO: per-frame total solve cost cap.

	// 'stat DynamicRope' — update frame load/phase dashboard (helper skips traversal if group not collected).
	RopeStats::FRopeFrameCounters FrameCounters;
	FrameCounters.NumGdfDispatched = NumGdfRopes;
	FrameCounters.FrameColliders = TotalFrameColliders;
	RopeStats::RecordFrameStats(Ropes, FrameCounters);

	// End of traversal — The postponed rope registration/deregistration is now reflected (transformation resumes immediately thereafter).
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
	// Simulation only in game/PIE (excluding editor preview/inspector world → component also registers as BeginPlay only then).
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

void URopeSimSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);

	// Register TG_PostPhysics tick function (replaces existing UTickableWorldSubsystem tickable). tickables are engine TickObjects
	// was implicitly placed at the call location (after TG_PostPhysics/before TG_PostUpdateWork — engine implementation details). express
	// Group + mesh tick Prerequisites (SetAnimPrerequisites) to contract the "after animation evaluation" order.
	// bAllowTickOnDedicatedServer: Keep the existing tickable since it also ran on the server (CPU fallback simulation).
	SimTickFunction.Target = this;
	SimTickFunction.TickGroup = TG_PostPhysics;
	SimTickFunction.EndTickGroup = TG_PostPhysics;
	SimTickFunction.bCanEverTick = true;
	SimTickFunction.bStartWithTickEnabled = true;
	SimTickFunction.bAllowTickOnDedicatedServer = true;
	SimTickFunction.RegisterTickFunction(InWorld.PersistentLevel);

	// Registers a solver in the scene of this world so that the view expansion can find and dispatch it from scene → solver in the GDF integration path.
	// (The scene has been created for rendering at this point.) Even if the path is off, registration is harmless (pending is empty, no-op).
	RopeGDF::RegisterSolver(InWorld.Scene, &GpuSolver);

	// static world collision provider Automatically spawns one host actor per world. The class specified by the settings (default
	// Spawn an ARopeController, but if None disable auto-spawning (opt-out manual placement). The spawned actor is immediately
	// Upon receiving BeginPlay, URopeStaticBodyProvider is registered as RegisterColliderProvider. DoesSupportWorldType
	// It is limited to Game/PIE, so it does not appear in the editor preview world.
	// Avoid duplication (fragment 1): If there is already a static provider in the world before auto-spawn (manual placement, etc.), give way and not spawn.
	// does not → "manual placement beats auto-spawn" is a non-static priority. component instead of registry (depending on registration order)
	// Check for instance existence — The batch actor has already been instantiated before BeginPlay, so it is caught regardless of registration timing.
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
				// Runtime Manager — Do not save to level.
				SpawnParams.ObjectFlags |= RF_Transient;
				// Position independent (origin).
				SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
				SpawnedStaticBodyController = InWorld.SpawnActor<AActor>(ControllerClass, FTransform::Identity, SpawnParams);
				// collider budget/convex plane cap because the provider reads Project Settings directly from BuildColliders
				// No need to inject here (single source of truth — no duplicate fields in the component).
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

	// Auto-spawned manager actor destroyed. world teardown cleans up actors anyway, but explicitly deletes them
	// Make sure no residue remains when re-initializing (e.g. PIE seamless travel). Defends if already destroyed with IsValid.
	if (IsValid(SpawnedStaticBodyController))
	{
		SpawnedStaticBodyController->Destroy();
	}
	SpawnedStaticBodyController = nullptr;

	if (const UWorld* World = GetWorld())
	{
		// Lower the demand first and then remove the solver (so that no remaining demand remains in the re-Initialize path while the world is alive).
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
		// There is no recovery yet or reseeding catch-up is in progress — There is no GPU candidate for this frame (capture is for the next frame).
		// Source is GPU (empty candidate) — does not return to CPU sweep.
		Rope.SimFrame.bGpuContactsThisFrame = true;
		return;
	}

	// collider set response gate (#7): ColliderIndex of delayed contact is based on **dispatch time** set, as shown below.
	// The attribution table was rebuilt for this frame. Only when the resulting dispatch signature is the same as the current signature.
	// indices mean the same thing — if different, drop (capture next frame) instead of attribution to another bone.
	// Signature 0 is not set (warm-up), so it is also dropped.
	if (Contacts->AttribSig == 0 || Contacts->AttribSig != Rope.SimFrame.GpuAttribSig)
	{
		Rope.SimFrame.bGpuContactsThisFrame = true;
		return;
	}

	// Contacts are processed in slot order (actual first, predictive second), so actual is processed first. CPU AddUniqueCandidate
	// Merge duplicates (node, bone, mesh) identically (SourceMask OR + Source priority Guided>Actual>Free) —
	// Prevents the tracker's node duplicate count and matches the check with the CPU.
	for (const FRopeGPUContactResult& C : Contacts->Contacts)
	{
		// collider index → ​​(bone, mesh) attribution. Anything out of range (collider set change) is skipped (self-correction).
		// Explicit dispatch — Unknown types are dropped into anyone's table without attribution (previously else was an attribution trap with a box).
		const TArray<FRopeSimFrameIO::FGpuColliderAttribution>* AttrPtr =
			(C.ColliderType == 0) ? &Rope.SimFrame.GpuCapsuleAttribution :
			(C.ColliderType == 1) ? &Rope.SimFrame.GpuSdfAttribution :
			(C.ColliderType == 2) ? &Rope.SimFrame.GpuBoxAttribution :
			(C.ColliderType == 3) ? &Rope.SimFrame.GpuConvexAttribution :
			                        nullptr;
		if (!AttrPtr || !AttrPtr->IsValidIndex(C.ColliderIndex))
		{
			continue;
		}
		const FRopeSimFrameIO::FGpuColliderAttribution& A = (*AttrPtr)[C.ColliderIndex];
		if (A.Bone.IsNone())
		{
			// No attribution (non-skeletal collider) — Not subject to capture.
			continue;
		}
		// weak — null if destroyed during delay (check proceeds to bone).
		const USceneComponent* Mesh = A.Mesh.Get();

		// Merge: If there is the same (node, bone, mesh) candidate, update SourceMask OR + Source priority, but do not add new candidate.
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
		// Filled by EvaluateRelativeMotion(GT).
		Cand.WrapDirectionScore = 0.0f;
		Rope.SimFrame.GpuFlightCandidates.Add(Cand);
	}
	Rope.SimFrame.bGpuContactsThisFrame = true;
}

bool URopeSimSubsystem::SyncGpuPositionsForHandoff(URopeComponent& Rope)
{
	if (!RopeGpuRuntimeAvailable())
	{
		// CPU fallback — Sim is already up to date.
		return false;
	}

	FRopeSimState& S = Rope.Sim;
	TArray<FVector> Pos;
	TArray<FVector> Prev;
	uint32 Generation = 0;
	if (!GpuSolver.ReadbackNow(Rope.GetUniqueID(), Pos, Prev, Generation))
	{
		// No resident buffer (never stepped into the GPU) — the mirror is the truth.
		return false;
	}
	if (Generation != Rope.SimFrame.SimGeneration || Pos.Num() != S.Num() || Prev.Num() != S.Num())
	{
		// Reseeding catch-up in progress or node number mismatch — preventing stale application.
		return false;
	}

	S.Positions = MoveTemp(Pos);
	S.PrevPositions = MoveTemp(Prev);
	// The grabbed end (node ​​0) snaps to the current pin, identical to the mirror convention.
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

	// GPU resident target: solve frame (Free/Flight/Wrapping/Wrapped) or Releasing frame with logic output only.
	// Since bSolveThisFrame/OverrideFrame are authoritatively determined in Prepare, a separate phase check is not required here.
	// Contacting (no output) does not dispatch itself, so the GPU buffer remains frozen (same as CPU's "no solve").
	const bool bGpuRope = (Rope.SimFrame.bSolveThisFrame || Rope.SimFrame.OverrideFrame.HasAny())
		&& S.Num() >= 2 && S.Num() <= FRopeGPUSolver::MaxNodes;
	// M5b: Only ropes that GPU step in this frame render read resident PosBuf directly (or stale → CPU mirror).
	Rope.SimFrame.bGpuSteppedThisFrame = bGpuRope;
	if (!bGpuRope)
	{
		// fallback (exceeding number of nodes, etc.): CPU solves Free span of bSolveThisFrame, skips override-only frame.
		if (Rope.SimFrame.bSolveThisFrame)
		{
			Rope.SolveSimFrame(DeltaTime);
		}
		return false;
	}

	const uint32 RopeId = Rope.GetUniqueID();

	// The previous recovery amount is reflected in the Sim (mirror). Only when generation matches current (= GPU catches up with current seed);
	// If catch-up is in progress immediately after reseeding, the seed source is preserved by leaving the CPU Sim as is.
	if (const FRopeResidentLatest* L = GpuLatest.Find(RopeId))
	{
		if (L->Generation == Rope.SimFrame.SimGeneration && L->NumNodes == S.Num()
			&& L->Positions.Num() == S.Num() && L->PrevPositions.Num() == S.Num())
		{
			S.Positions = L->Positions;
			S.PrevPositions = L->PrevPositions;
			// tension mirror (only when present — it may be rarer than the position because it is only retrieved in the solve frame. If not, the previous value is maintained).
			if (L->SegmentTension.Num() == S.Num() - 1)
			{
				S.SegmentTension = L->SegmentTension;
			}
		}
	}

	// Align the gripped end (node ​​0) exactly with the current pin position — the GPU mirror has a ~1-2 frame delay, so it is misaligned with the hand.
	// Correction for render/contact (GPU solve itself processes pins at each step as PinTarget, so simulation is not affected).
	if (S.bStartPinned && S.Num() > 0)
	{
		S.Positions[0] = S.StartPinTarget;
		S.PrevPositions[0] = S.StartPinPrev;
	}

	// If the mirror covered the logic output of Prepare (anchor position, etc.), reapply — CPU Sim mirror.
	// Maintain the best combination of “write the latest bone-based logic + delayed Free span” (G2).
	if (Rope.SimFrame.OverrideFrame.HasAny())
	{
		Rope.SimFrame.OverrideFrame.ApplyToSim(S);
	}

	// pinned-timestep schedule (CPU accumulator). override-only frame(bSolveThisFrame=false) does not require integration.
	// Only records override (NumSub=0) — Same time processing as “no solve” in CPU path.
	// schedule/seed use the same solver settings.
	const FRopeSolverConfig& EffSolverCfg = Rope.SolverConfig;
	FRopeSubstepSchedule Schedule;
	Schedule.NumSub = 0;
	Schedule.FixedDt = 0.0f;
	if (Rope.SimFrame.bSolveThisFrame)
	{
		// Return the time of the discarded step that could not be dispatched to the accumulator and make a schedule — so that
		// The accumulator remains the single truth of the “simulated time”. After reverting, RopeSolverSubsteps immediately below
		// Because it is clamped to MaxAccum, rushing after a long stationary is limited as is the existing slow-mo policy.
		float Refund = 0.0f;
		if (PendingSimTimeRefund.RemoveAndCopyValue(RopeId, Refund) && Refund > 0.0f)
		{
			S.TimeAccumulator += Refund;
		}
		Schedule = RopeSolverSubsteps(S, EffSolverCfg, DeltaTime);
	}

	// Resident step configuration (self-contained). Seed data is provided every frame (RT is uploaded to GPU only when reseeding).
	SeedResidentStep(OutStep, RopeId, Rope.SimFrame.SimGeneration, S, EffSolverCfg, Schedule);
	// Non-stretchable contract per phase, such as CPU SolveSimFrame. The strain-limit in the GPU's latest resident pose is
	// is applied, so it is more accurate than calibrating the guide target based on a delayed CPU mirror.
	OutStep.MaxStretchRatio = Rope.GetEffectiveMaxStretchRatio();
	// Cover component boundary analysis value: radius auto(0=render Radius) + GDF flag moved directly to component.
	OutStep.CollisionRadius = Rope.GetEffectiveCollisionRadius();
	// Solve collision and contact detection are separate contracts. Even if it is false, the PackStepColliders below are used for detect.
	// Packing continues, and only the number of colliders/GDFs transmitted to the solve kernel becomes 0.
	OutStep.bSolveCollisions = Rope.SimFrame.bSolveCollisionsThisFrame;
	OutStep.bUseWorldGDF = Rope.bUseWorldGDF && OutStep.bSolveCollisions;
	// Distance LOD: The far rope has iteration damping (calculated in Prepare). CollisionPasses are clamped to Iterations in Packing.
	OutStep.Iterations = Rope.GetLODScaledIterations();

	// G3: contact detection only on Flight rope (capture only on Flight rope). Since this function is called only on GPU path
	// GPUContacts are always on — one gate with phase == Flight is enough.
	const bool bDetectThisRope = (Rope.Phase == ERopePhase::Flight);
	if (bDetectThisRope)
	{
		RequestContactDetection(Rope, DeltaTime, OutStep);
	}

	// collision: Classifies this rope's collider as capsule(M2)/SDF(M3) (+ parallel attribution table when detecting).
	// Only when there is a consumer (solve substep loop / detection kernel) — override-only frame(Wrapped sleep,
	// For frames with NumSub=0 at high fps, the kernel does not read the collider, so smoothing/uploading is skipped entirely.
	// (RT packs only place 1 dummy in an empty array). The collider for wake check is separate as FrameColliders (gather).
	if (OutStep.NumSub > 0 || bDetectThisRope)
	{
		PackStepColliders(Rope, bDetectThisRope, OutStep);
	}

	// G2: Inject logic phase output (OverrideFrame) into override — Replaces logic phase reseeding.
	// Exactly the same data as applied to the CPU Sim (bit mirror guaranteed by static_assert above).
	if (Rope.SimFrame.OverrideFrame.HasAny() && Rope.SimFrame.OverrideFrame.Flags.Num() == S.Num())
	{
		OutStep.OverrideFlags = Rope.SimFrame.OverrideFrame.Flags;
		OutStep.OverridePositions = Rope.SimFrame.OverrideFrame.Positions;
		OutStep.OverridePrevPositions = Rope.SimFrame.OverrideFrame.PrevPositions;
		OutStep.OverrideInvMass = Rope.SimFrame.OverrideFrame.InvMass;
	}

	// G1: Inject whip guide target as override (Flight only, applied before integration).
	PackWhipOverride(Rope, OutStep);

	return true;
}

void URopeSimSubsystem::RequestContactDetection(URopeComponent& Rope, float DeltaTime, FRopeGPUResidentStep& Step) const
{
	const FRopeSimState& S = Rope.Sim;
	// The attribution table (collider index → bone/mesh) is similar to PackStepColliders with Step.Capsules/SDFColliders.
	// It is filled in the same order, so reset here first.
	Step.bDetectContacts = true;
	Step.ContactRadius = Rope.GetEffectiveContactQueryRadius();
	Step.PredictionFrames = Rope.WrapConfig.PredictiveContactFrames;
	// detection sweep resolution (anti-tunneling) — comes from the same stored value as CPU MakeFlightDetectParams.
	Step.ContactSweepStep = Rope.WrapConfig.ContactSweepStep;
	Step.ContactMaxSweepSamples = Rope.WrapConfig.ContactMaxSweepSamples;
	// Substep of prediction contact Free node extrapolation → frame displacement conversion (#8). Step.FixedDt(=Schedule.FixedDt=(1/60)/Substeps) is
	// Already populated by SeedResidentStep — Same value as FrameDeltaTime/SubstepDeltaTime in CPU MakeFlightDetectParams.
	Step.ContactFrameToSubstepRatio = (Step.FixedDt > KINDA_SMALL_NUMBER) ? (DeltaTime / Step.FixedDt) : 1.0f;
	Rope.SimFrame.GpuCapsuleAttribution.Reset();
	Rope.SimFrame.GpuSdfAttribution.Reset();
	Rope.SimFrame.GpuBoxAttribution.Reset();
	Rope.SimFrame.GpuConvexAttribution.Reset();

	// Predictive contact (G3b): The GPU carries the guide mask/current, previous, and next target in the whip active frame.
	// Allows extrapolation of the guide node (same input as CPU AddPredictedContactCandidates).
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

// Ordered (bone, mesh) signature of the GPU attribution set. The ColliderIndex of the delayed contact (1~2 frames) is this frame.
// Used to check whether it corresponds safely to the attribution table (= invariant set/order) by comparing between frames.
// See FRopeSimFrameIO::GpuAttribSig comment for detailed contract.
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
	// Upon detection, collider index → ​​bone/mesh attribution is filled in parallel in the same order as Step.Capsules/SDFColliders.
	auto MakeAttribution = [](IRopeCollider* Collider)
		{
			FRopeSimFrameIO::FGpuColliderAttribution Attr;
			const USceneComponent* Mesh = nullptr;
			Collider->GetGPUAttribution(Attr.Bone, Mesh);
			Attr.Mesh = Mesh;
			return Attr;
		};

	// Reveal silent exclusion of colliders without GPU representation with a one-time warning (once per session — anti-spam latch).
	// A custom collider that only implements the CPU contract (Query/QuerySwept) works in unit tests/CPU fallback, but
	// This is excluded from the runtime regular path (GPU solve) — without the warning, "It works in testing, but not in game."
	// The worst type of trap is that the rope is pierced, so the log is part of the contract (see RopeCollider.h).
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

	// convex packing shared(pass 1 = wrap enabled / pass 2 = static push-out): body-local plane to flat pool
	// Concatenate and refer to offset/number + rigid body(curr/prev) + InvDt. true on success.
	auto TryPackConvex = [&Step](IRopeCollider* Collider) -> bool
	{
		TConstArrayView<FPlane> LocalPlanes;
		FBox LocalBounds(ForceInit);
		FQuat CvRot, CvPrevRot;
		FVector CvTrans, CvPrevTrans;
		float CvInvDt = 0.0f;
		if (!Collider->GetGPUConvex(LocalPlanes, LocalBounds, CvRot, CvTrans, CvPrevRot, CvPrevTrans, CvInvDt)
			|| LocalPlanes.Num() == 0 || !LocalBounds.IsValid)
		{
			return false;
		}
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
			// local·unit·outer, PlaneDot=dot(N,p)-W
			Step.ConvexPlanes.Add(FVector4(Pl.X, Pl.Y, Pl.Z, Pl.W));
		}
		Step.Convexes.Add(Cv);
		return true;
	};

	// FrameColliders are GT gathered snapshots in Prepare. 2-pass: Non-static (skeletal) collider first,
	// Pack static(world) collider behind. detection(detect) kernel only sets capsule to [0, NumDetectCapsules)
	// , so the static capsule is automatically excluded from detection — detection leaves only one deepest contact per node, and wall contact is
	// This is because if the bone contact is obscured, the wrap capture silently fails (the box is not in the detection kernel at all). Solve is all bone.
	for (IRopeCollider* Collider : Rope.SimFrame.FrameColliders)
	{
		if (!Collider || Collider->IsWorldStatic())
		{
			continue;
		}
		FRopeGPUCapsule Cap;
		if (Collider->GetGPUCapsule(Cap.A, Cap.B, Cap.Radius))
		{
			// frame motion (prev endpoint + InvDt): surface velocity drag/relative motion CCD. If static, the default value (InvDt 0) is maintained.
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
			// Wrapable box (virtual bone): Fills up to the frame motion (prev + InvDt) and packs in front of the detection range.
			Collider->GetGPUBoxMotion(Box.PrevCenter, Box.PrevRot, Box.InvDeltaTime);
			Step.Boxes.Add(Box);
			if (bDetectThisRope)
			{
				Rope.SimFrame.GpuBoxAttribution.Add(MakeAttribution(Collider));
			}
			continue;
		}
		if (TryPackConvex(Collider))
		{
			// wrap possible convex (virtual bone): Packing in front of the detection range.
			if (bDetectThisRope)
			{
				Rope.SimFrame.GpuConvexAttribution.Add(MakeAttribution(Collider));
			}
			continue;
		}
		// Non-static means that only capsule/SDF/box/convex are Loaded on the GPU — all or nothing.
		WarnGpuUnrepresented(Collider);
	}
	// detection boundary: This is the non-static capsule.
	Step.NumDetectCapsules = Step.Capsules.Num();
	// box detection boundary: The box can wrap up to this point.
	Step.NumDetectBoxes = Step.Boxes.Num();
	// convex detection boundary: Convex can wrap up to this point.
	Step.NumDetectConvexes = Step.Convexes.Num();

	// pass 2: static(world) collider — solve only. capsule (sphere/spiel) appends after the detection border,
	// box is a private array. The attribution table also fills the static capsule to maintain index alignment (None/null —
	// Defensive because the detection kernel does not emit out-of-bounds indices.
	for (IRopeCollider* Collider : Rope.SimFrame.FrameColliders)
	{
		if (!Collider || !Collider->IsWorldStatic())
		{
			continue;
		}
		FRopeGPUCapsule Cap;
		if (Collider->GetGPUCapsule(Cap.A, Cap.B, Cap.Radius))
		{
			// static — No frame motion (InvDt 0 default).
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
			// frame motion (prev center/rot + InvDt): surface velocity drag/relative motion CCD. If static, the default value (InvDt 0) is maintained.
			Collider->GetGPUBoxMotion(Box.PrevCenter, Box.PrevRot, Box.InvDeltaTime);
			Step.Boxes.Add(Box);
			if (bDetectThisRope)
			{
				// static - None (not involved in detection, for index alignment)
				Rope.SimFrame.GpuBoxAttribution.Add(MakeAttribution(Collider));
			}
			continue;
		}
		if (!TryPackConvex(Collider))
		{
			// static means only capsule/box/convex will be Loaded on the GPU — all or nothing.
			WarnGpuUnrepresented(Collider);
			continue;
		}
		if (bDetectThisRope)
		{
			// static - None (not involved in detection, for index alignment)
			Rope.SimFrame.GpuConvexAttribution.Add(MakeAttribution(Collider));
		}
	}

	// Prevent delayed GPU contact misattribution (#7): Roll the signature of this frame attribution set (meaning only for detect frames).
	// BuildGpuFlightCandidates gates delayed contact consumption with the stability of the most recent window (current==Prev1==Prev2).
	if (bDetectThisRope)
	{
		uint32 Sig = 0x9E3779B9u;
		Sig = RopeComputeAttribSig(Rope.SimFrame.GpuCapsuleAttribution, Sig);
		Sig = RopeComputeAttribSig(Rope.SimFrame.GpuSdfAttribution, Sig);
		Sig = RopeComputeAttribSig(Rope.SimFrame.GpuBoxAttribution, Sig);
		Sig = RopeComputeAttribSig(Rope.SimFrame.GpuConvexAttribution, Sig);
		// Signature 0 is reserved for "unset" — if the hash happens to be 0, it is pushed to 1 to distinguish it from a warmup.
		Rope.SimFrame.GpuAttribSig = (Sig == 0) ? 1u : Sig;
		// Loads the signature of the set used by this dispatch into the step (the detection result is returned as is).
		Step.AttribSig = Rope.SimFrame.GpuAttribSig;
	}
}

void URopeSimSubsystem::PackWhipOverride(const URopeComponent& Rope, FRopeGPUResidentStep& Step) const
{
	// G1: Load the whip guide target calculated by Advance of Prepare as override (same data as CPU path ApplyToSim).
	// Flight gate: Prevents application of remaining stale masks in other phases (since Flight does not fill the OverrideFrame)
	// does not overlap with G2 packing).
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
		// If there is no previous target (edge ​​case), velocity 0 — CPU fallback ("previous position") and approximation.
		Step.OverridePrevPositions[k] = WhipPrev.IsValidIndex(k) ? WhipPrev[k] : WhipCur[k];
	}
}
