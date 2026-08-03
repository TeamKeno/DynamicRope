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
#include "Logic/RopeFlightContactDetector.h"
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
	// Debug and profiling toggle that forces the CPU solve. At 1 the GPU resident path is off even with a
	// renderable RHI and the rope drops to the CPU fallback solver and detector — solve, detection and the
	// handoff sync all switch together, and since the rope's bGpuSteppedThisFrame goes false the tube falls
	// back to the CPU mirror centerline automatically. For verifying and benchmarking against the GPU. Default 0.
	static TAutoConsoleVariable<int32> CVarForceCPUSolve(
		TEXT("r.DynamicRope.ForceCPUSolve"),
		0,
	TEXT("1 forces rope solve and detection onto the CPU path even when the GPU is available (for debugging and comparison). 0 = choose automatically (default)."),
		ECVF_Default);

	// The GPU is the only runtime path: with a renderable RHI the rope solves and detects GPU-resident,
	// and otherwise (a cook, -nullrhi, a server build) it falls back to the CPU for both automatically. The
	// only client-facing toggle is r.DynamicRope.ForceCPUSolve above, for forcing the CPU while debugging.
	// FRopeXPBDSolver survives for that fallback and for parity testing, and is effectively unused in a
	// shipping client.
	bool RopeGpuRuntimeAvailable()
	{
		// With the force-CPU toggle on, drop to the CPU fallback regardless of what the GPU could do.
		if (CVarForceCPUSolve.GetValueOnGameThread() != 0)
		{
			return false;
		}
		// A renderable RHI and SM5 or above, since the kernels are compiled behind an SM5 guard.
		// RopeGPU::IsRuntimeSupported is the single source of truth for that test — the scene proxy's GPU tube
		// gate calls the same function.
		return RopeGPU::IsRuntimeSupported();
	}

	// Duplicate world-static provider warning: a log line plus an on-screen message in editor and development
	// builds. The invariant is at most one per world, and saying so out loud is what keeps it from being
	// broken quietly — the second provider is ignored, and whoever added it needs to know why.
	void WarnDuplicateWorldStaticProvider(const AActor* Offender)
	{
		UE_LOG(LogRopeCollision, Warning,
			TEXT("A static world collider provider already exists — the duplicate provider on %s is ignored (one per world)."),
			*GetNameSafe(Offender));
#if !UE_BUILD_SHIPPING
		if (GEngine)
		{
			// The key is a fixed constant rather than GetTypeHash, so the message refreshes once per event instead of every frame.
			GEngine->AddOnScreenDebugMessage(uint64(0x0D0ED1CA), 8.0f, FColor::Yellow,
					FString::Printf(TEXT("[DynamicRope] Duplicate static body provider ignored (%s) — only one per world is used."),
					*GetNameSafe(Offender)));
		}
#endif
	}

	// Flatten a runtime SDF collider view (Collision) into the GPU upload struct (Shaders).
	// Adding a field means updating this one place — it used to be seventeen lines scattered through the tick loop.
	FRopeGPUSDFCollider MakeGpuSdf(const FRopeSDFColliderView& View)
	{
		FRopeGPUSDFCollider Sdf;
		// Code byte blob; dequantized when the upload is flattened.
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
		// For GPU CCD and surface-velocity drag.
		Sdf.PrevBoneToWorld = View.PrevBoneToWorld;
		Sdf.InvDeltaTime    = View.InvDeltaTime;
		Sdf.VolumeKey       = View.VolumeKey;
		return Sdf;
	}

	// Fill in the step's solver seed and parameters. Collider, override and whip packing are added by the caller.
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
		// CollisionRadius needs its auto value resolved (0 = the render Radius), which the caller covers through
		// Rope.GetEffectiveCollisionRadius. bUseWorldGDF lives directly on the component now and is the
		// caller's business too.
		Step.CollisionRadius   = Cfg.CollisionRadius;
		Step.Friction          = Cfg.Friction;
		Step.TipFrictionScale  = Cfg.TipFrictionScale;
		Step.SweepStep         = Cfg.SweepStep;
		Step.MaxSweepSamples   = Cfg.MaxSweepSamples;
		Step.NumSub            = Schedule.NumSub;
		Step.FixedDt           = Schedule.FixedDt;
	}
}

// FRopeNodeOverrideFrame's bits (Core) must be numerically 1:1 with ERopeGPUOverride (Shaders). Core does not
// depend on Shaders, so the constants are mirrored, and this is where both are visible and it can be checked.
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
			// Re-entry during the tick traversal — a handler spawning a rope actor — so the mutation is deferred (see the bTickingRopes note in the header).
			DeferredRopeUnregister.RemoveSingleSwap(Rope);
			DeferredRopeRegister.AddUnique(Rope);
			return;
		}
		Ropes.AddUnique(Rope);
		// The owner of the GPU-resident resources is recorded by ID as well: it is the only way to reclaim them if the component disappears without a proper release.
		RegisteredRopeIds.Add(Rope->GetUniqueID());
		// Because the hand pin, attached to a socket, follows the owning character's pose.
		SetAnimPrerequisites(Rope, /*bAdd*/ true);
		UE_LOG(LogDynamicRope, Verbose, TEXT("RegisterRope: %s (%d total)"), *Rope->GetName(), Ropes.Num());
	}
}

void URopeSimSubsystem::UnregisterRope(URopeComponent* Rope)
{
	if (bTickingRopes)
	{
		// Re-entry during the tick traversal — a handler destroying a rope actor — so the real removal is
		// deferred to ApplyDeferredRopeChanges. For the rest of this frame's traversal the IsValid guard skips
		// this rope, since destroying it marks it pending-kill (see the bTickingRopes note in the header).
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
		// Free the GPU resident buffer and readback on the render thread, and drop the rope from both caches
		// and from the ID ledger. (GpuLatestContacts used to be missed here, so a dead rope's contact snapshot
		// lingered for the life of the world.)
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

	// Build the set of live rope IDs; anything left in the ledger alone is a rope that vanished without a proper release.
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
		// The animation prerequisite cannot be undone here, because the component is already gone and its owned
		// mesh cannot be traced back. FTickPrerequisite entries are weak and a dead mesh is skipped
		// automatically, so leaving them is harmless; and if the mesh is alive then the component went through
		// a proper EndPlay and never reaches this path.
		UE_LOG(LogDynamicRope, Verbose,
			TEXT("ReleaseGpuResourcesForDeadRopes: RopeId %u — reclaiming the GPU resources of a rope that vanished without a proper release."), RopeId);
		GpuSolver.ReleaseRope(RopeId);
		GpuLatest.Remove(RopeId);
		GpuLatestContacts.Remove(RopeId);
		PendingSimTimeRefund.Remove(RopeId);
		It.RemoveCurrent();
	}
}

void URopeSimSubsystem::ApplyDeferredRopeChanges()
{
	// Order matters: release first, register second, so a rope spawned and destroyed within one tick still
	// converges to its final state. bTickingRopes is already false, so the calls below perform the real Ropes
	// mutation and GPU release — and Register/Unregister fire no delegates, so nothing re-enters from here.
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

	// Duplicate backstop: at most one world-static provider per world. If one is already registered, the second
	// is rejected with a warning. The invariant is enforced through the interface regardless of where the
	// provider came from — placed by hand, auto-spawned, or created at runtime — and first registration wins,
	// which combined with auto-spawn standing down means a hand-placed provider always beats an automatic one.
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
					// Registration refused, so GatherColliders is never called on this provider.
					return;
				}
			}
		}
	}

	ColliderProviders.AddUnique(Provider);
	// Because a bone collider — capsule or SDF — reads the owning character's pose.
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
	// Guarantees the rope simulates after the animation has been evaluated: the skeletal mesh tick of the
	// source component's owning actor is made a prerequisite of SimTickFunction. The mesh tick's completion
	// covers the parallel animation task through DontCompleteUntil
	// (SkeletalMeshComponent::DispatchParallelEvaluationTasks), so this frame's pose — the buffer flip — is
	// guaranteed. Registering the same mesh through both a rope and a provider is fine, since AddPrerequisite
	// is idempotent.
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
			// Only actually called for the first consumer. AddPrerequisite is idempotent so a duplicate call
			// would be harmless, but without counting, one consumer releasing would wipe the others' share
			// (see the header comment).
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
		// If the actor dies without a proper release, the key goes stale and only the count remains. The
		// prerequisite itself is weak and harmless, but the map is swept once per add so it cannot grow without
		// bound — the release path pays nothing extra.
		for (auto It = AnimPrereqRefCount.CreateIterator(); It; ++It)
		{
			if (!It->Key.IsValid())
			{
				It.RemoveCurrent();
			}
		}
	}
}

FBox URopeSimSubsystem::ComputeRopeQueryBounds(const URopeComponent& Rope, float DeltaTime, bool bIncludeAimRay)
{
	const bool bHasAimRayBounds = Rope.SimFrame.AimRayColliderQueryBounds.IsValid != 0;
	const bool bHasLockedTargetBounds = Rope.AimTargeting.IsLockActive(Rope.Phase) &&
		Rope.SimFrame.LockedTargetColliderQueryBounds.IsValid;
	// This is an aiming region request, but with no ray and no active target there is nothing to gather — an invalid box empties the list.
	if (bIncludeAimRay && !bHasAimRayBounds && !bHasLockedTargetBounds)
	{
		return FBox(ForceInit);
	}

	// The rope's tight AABB (Pos ∪ Prev, so this frame's motion is included) plus a margin. The provider's
	// region and the per-rope collider culling share this one box, called from both GatherCollidersForRope and
	// BuildFrameColliders.
	FBox RopeBounds(ForceInit);
	// Predictive contact exists only in Flight. Carrying that extrapolation into Contacting/Wrapping can
	// gather remote pieces of a composite simple-collision set and change the geometry used to build the
	// wrap. Pos union Prev already covers actual current motion.
	float MaxFrameDispSq = 0.0f;
	for (int32 i = 0; i < Rope.Sim.Num(); ++i)
	{
		RopeBounds += Rope.Sim.Positions[i];
		RopeBounds += Rope.Sim.PrevPositions[i];
		if (Rope.Phase == ERopePhase::Flight)
		{
			MaxFrameDispSq = FMath::Max(MaxFrameDispSq,
				static_cast<float>(FVector::DistSquared(Rope.Sim.Positions[i], Rope.Sim.PrevPositions[i])));
		}
	}
	const float BaseMargin = Rope.GetEffectiveCollisionRadius() + Rope.GetEffectiveContactQueryRadius()
		+ FMath::Max(2.0f * Rope.Sim.SegmentLength, 50.0f);
	float PredictiveMotionMargin = 0.0f;
	if (Rope.Phase == ERopePhase::Flight)
	{
		// Positions - PrevPositions is only the last solver substep, not a whole frame, so convert it
		// with the exact ratio used by FRopeFlightContactDetector before extrapolating forward.
		const float SubstepDeltaTime = (1.0f / 60.0f)
			/ static_cast<float>(FMath::Clamp(Rope.SolverConfig.Substeps, 1, 16));
		const float FrameToSubstepRatio =
			FRopeFlightContactDetector::ComputeFrameToSubstepRatio(DeltaTime, SubstepDeltaTime);
		PredictiveMotionMargin = FMath::Sqrt(MaxFrameDispSq) * FrameToSubstepRatio
			* FMath::Max(Rope.WrapConfig.PredictiveContactFrames, 1.0f);
	}
	const float QueryMargin = BaseMargin + PredictiveMotionMargin;
	if (RopeBounds.IsValid)
	{
		// Margin: the contact query radius, plus the sweep margin, plus the predicted contact's forward
		// extrapolation (frame displacement × predicted frames). Be generous — over-including is safe, since it
		// only costs a few more colliders.
		RopeBounds = RopeBounds.ExpandBy(QueryMargin);
	}
	if (bIncludeAimRay)
	{
		// Once aiming ends and only the active aim lock remains, do not build an enormous AABB spanning rope to
		// target: re-gather only around the previous target's collider bounds. The results are promoted through
		// the component's target filter.
		if (!bHasAimRayBounds && bHasLockedTargetBounds)
		{
			return Rope.SimFrame.LockedTargetColliderQueryBounds.ExpandBy(QueryMargin);
		}
		// The aiming region alone: a preview ray can pass through space nowhere near the current rope
		// centerline, and without merging that span a ray crossing an SDF would find no collider in the aiming
		// list and read as a miss. Because it is a union with the area around the rope, what the aiming query
		// sees — the hit test and the preview arc search — is exactly what it saw before the split; only the
		// FrameColliders side, used by physics and the debugger, is narrowed.
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

void URopeSimSubsystem::BuildFrameColliders(float DeltaTime)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_BuildColliders);
	FrameProviders.Reset();

	// The active region list: the first N entries are physics regions, 1:1 with the Ropes index, and the
	// second N are those same ropes' aiming regions (AimRegionIndexOf). Invalid or empty ropes, and ropes not
	// aiming, keep their slot as an !IsValid box, so the region mapping index a provider returns still lines up
	// with this index — a provider skips !IsValid entries. A bounds-aware provider (the static body one) uses
	// this list to avoid scanning the empty space between distant ropes.
	// It is the same box GatherCollidersForRope uses below, as the single source of truth.
	FrameRopeRegions.Reset();
	FrameRopeRegions.Reserve(Ropes.Num() * 2);
	for (URopeComponent* Rope : Ropes)
	{
		FrameRopeRegions.Add(IsValid(Rope) ? ComputeRopeQueryBounds(*Rope, DeltaTime) : FBox(ForceInit));
	}
	for (URopeComponent* Rope : Ropes)
	{
		FrameRopeRegions.Add(IsValid(Rope)
			? ComputeRopeQueryBounds(*Rope, DeltaTime, /*bIncludeAimRay*/ true) : FBox(ForceInit));
	}

	// Region priority: active ropes first, and a physics region before an aiming one. The global extraction cap
	// spends its budget first-come-first-served, so on a frame that hits the cap some later region goes
	// unscanned — ordering is simply how we make sure the one that starves is not the rope actually being
	// simulated. The indices themselves never move, so the mapping is unaffected. An aiming region only feeds
	// the HUD and the preview, so it yields to physics.
	// Keys: 0 = a busy phase (Flight through Releasing), 1 = Free and awake, 2 = Free and asleep, 3 = an
	// invalid region. An aiming region adds 4 to that (invalid staying at 7), placing it behind every physics region.
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

	// Gather once per registered provider — once per frame, whatever the rope count. Dead providers are cleaned up.
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
		// Static world providers are exempt from the owner exclusion.
		FP.bWorldStatic = Provider->ProvidesWorldStaticColliders();
		FP.Colliders = MoveTemp(Gather.Colliders);
		// The per-collider source actor is only trusted when the array lengths agree: a mismatch would tangle
		// the indices and exclude the wrong collider, so it is left empty and the check falls back to
		// provider granularity.
		if (Gather.ColliderSourceActors.Num() == FP.Colliders.Num())
		{
			FP.SourceActors = MoveTemp(Gather.ColliderSourceActors);
		}
		// The region mapping is only trusted when its length matches the rope count. A mismatch is a provider bug, and it demotes to the bounds re-test fallback.
		FP.bHasRegionMapping = Gather.bHasRegionMapping
			&& Gather.RegionColliderIndices.Num() == FrameRopeRegions.Num();
		if (FP.bHasRegionMapping)
		{
			FP.RegionIndices = MoveTemp(Gather.RegionColliderIndices);
		}
		else
		{
			// Fallback path only: cache each collider's world bounds once per frame, so the re-test does not
			// recompute them through a virtual call once per rope.
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

	// By default a rope collides with every provider in the world except its own, which is what stops a rope in
	// flight tangling on the thrower. Wrapping another actor's body works automatically, because that actor is
	// part of "every provider". Opt back in when the owner's own collision is wanted.
	const AActor* OwnerToExclude = Rope.bIncludeOwnerColliders ? nullptr : Rope.GetOwner();

	// Distance culling: a collider that does not overlap the rope's AABB (Pos ∪ Prev, so this frame's motion is
	// included) is never loaded at all. The CPU solver has its own broad phase, but the GPU kernel loops every
	// collider for every node, so filtering here is what makes the system scale — a distant character's
	// capsules and SDFs never enter the step.
	// The default path consumes the region mapping the provider returned when it gathered, with no re-test. It
	// is the same box BuildFrameColliders handed the provider, as the single source of truth.
	const FBox RopeBounds = FrameRopeRegions.IsValidIndex(RegionIndex) ? FrameRopeRegions[RegionIndex] : FBox(ForceInit);
	const bool bCull = RopeBounds.IsValid != 0;

	// Per-rope budget for static world colliders. Separate from the global extraction cap
	// (StaticBodyMaxColliders), this bounds how many static world colliders *this* rope solves against, so a
	// distant rope cannot spend this rope's budget.
	// Skeletal colliders (capsule, SDF) are naturally bounded by the bone count and are the heart of wrapping,
	// so they are exempt and go straight into OutColliders.
	const UDynamicRopeSettings* Settings = UDynamicRopeSettings::Get();
	const int32 PerRopeBudget = Settings ? FMath::Max(1, Settings->StaticBodyMaxCollidersPerRope) : 32;

	// Static world candidates are collected separately, and over budget the furthest ones are dropped. Skeletal colliders were already included unconditionally above.
	TArray<IRopeCollider*> WorldStaticCandidates;

	// Per-body owner exclusion. A static world provider is exempt from the provider-level exclusion below,
	// because what the exemption is for is world geometry — floors and pillars. But that same provider scans
	// the world and may pick up shapes attached to the owning actor — the tether proxy, the tip mesh, a held
	// weapon — which then follow the rope and become push-out colliders shoving it around. Only those, by
	// source actor, are removed. A provider that supplies no source actors returns an empty array, so this is
	// always false and the old provider-level check applies unchanged.
	auto IsOwnBodyCollider = [OwnerToExclude](const FFrameProviderColliders& P, int32 Index)
	{
		return RopeColliderGather::IsExcludedOwnerBody(P.SourceActors, Index, OwnerToExclude);
	};

	for (const FFrameProviderColliders& FP : FrameProviders)
	{
		// Exclude the owner's own providers — except a static world one, since world collision should not
		// disappear merely because the provider hangs off a rope-owning actor. Shapes of the owner's own body
		// that the exemption lets through are filtered per collider by IsOwnBodyCollider above.
		if (!FP.bWorldStatic && FP.Owner == OwnerToExclude && OwnerToExclude != nullptr)
		{
			continue;
		}
		if (!bCull)
		{
			// A rope with no region — an empty sim, say — takes the full fallback, bypassing the budget. Rare, and behaviour is unchanged.
			for (int32 c = 0; c < FP.Colliders.Num(); ++c)
			{
				if (FP.Colliders[c] && !IsOwnBodyCollider(FP, c))
				{
					OutColliders.Add(FP.Colliders[c]);
				}
			}
			continue;
		}

		// Default path: consume the mapping the provider built for this region, meaning this rope. No bounds re-test.
		if (FP.bHasRegionMapping)
		{
			if (!FP.RegionIndices.IsValidIndex(RegionIndex))
			{
		// Unreachable, since the length was validated during the build.
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
					// Counts against the budget.
					WorldStaticCandidates.Add(Collider);
				}
				else
				{
					// Skeletal and the like — always included.
					OutColliders.Add(Collider);
				}
			}
			continue;
		}

		// Fallback path, for a provider with no mapping: re-test the collider bounds the old way.
		if (FP.Bounds.Num() != FP.Colliders.Num())
		{
			// The bounds cache does not match, so take the full fallback. Rare.
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
					// Counts against the budget.
					WorldStaticCandidates.Add(FP.Colliders[c]);
				}
				else
				{
					// Skeletal and the like — always included.
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

	// Over budget: keep only the nearest PerRopeBudget colliders, dropping the furthest. Nearness is the
	// smallest squared distance from the collider's world bounds centre to any rope node, computed once per
	// candidate so the sort does not recompute it. This path only runs on a frame that exceeded the budget.
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
		// If aiming has finished, or was never running, clear the list — the provider pointers from the last
		// frame would otherwise have the next aiming query read storage that has already been destroyed.
		Rope.SimFrame.AimFrameColliders.Reset();
		return;
	}
	GatherCollidersForRope(Rope, RegionIndex, Rope.SimFrame.AimFrameColliders);
}

bool URopeSimSubsystem::RefreshAimFrameCollidersForImmediateQuery(URopeComponent& /*Rope*/)
{
	// A no-op, kept for ABI and source compatibility. Calling BuildFrameColliders outside the normal tick would
	// break the once-per-frame provider contract and the multi-wielder region consistency again, so the old
	// behaviour is not being restored.
	return false;
}

void URopeSimSubsystem::Tick(float DeltaTime)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SubsystemTick);
	SCOPE_CYCLE_COUNTER(STAT_RopeSim_Tick);

	// Clean up invalid entries. A rope disappearing here never went through UnregisterRope — its actor was
	// destroyed — so its GPU resident resources are still held; dropping it from the array and reclaiming those
	// resources happen together.
	Ropes.RemoveAllSwap([](const TObjectPtr<URopeComponent>& Rope) { return !IsValid(Rope.Get()); });
	ReleaseGpuResourcesForDeadRopes();
	if (Ropes.Num() == 0)
	{
	// Not lowering the GDF demand on the frame the last rope disappears — which is what happened when this
	// returned before reaching SetGDFActiveCount below — leaves the engine building a Global Distance Field
	// nobody uses.
		if (const UWorld* World = GetWorld())
		{
			RopeGDF::SetGDFActiveCount(World->Scene, 0);
		}
		return;
	}

	// The GPU is the only runtime path: GPU when the RHI can render, CPU fallback otherwise, chosen automatically. Detection follows the solve.
	const bool bUseGPU = RopeGpuRuntimeAvailable();
	// Solving on the GPU means detecting on the GPU; there is no separate toggle.
	const bool bUseGPUContacts = bUseGPU;

	// Aggregation for the 'stat DynamicRope' frame dashboard. NumGdfRopes and TotalFrameColliders are gathered
	// in the GPU and gather loops below, since only the subsystem knows them; the remaining phase and solve
	// path counters are counted by RecordFrameStats through public getters.
	int32 NumGdfRopes = 0;
	int32 TotalFrameColliders = 0;

	// GPU resident: pull and cache the latest positions per RopeId that the render-thread readback filled in,
	// roughly one to two frames behind. Phase 2 below mirrors them into the Sim of a Free or Flight rope for
	// render and collision. The sequential dependency itself is satisfied inside the GPU's persistent buffer.
	if (bUseGPU)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_GPUGetLatest);
		GpuSolver.GetLatest(GpuLatest);
		// Reclaim the simulation time of any step that was replaced before a view expansion consumed it (TryBuildResidentStep below hands it back).
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
			// Fetch the contact detection results, attributed before Finalize runs.
			GpuSolver.GetLatestContacts(GpuLatestContacts);
		}
	}

	// Defer Register/UnregisterRope's mutation of Ropes during the traversal below — a reentrancy guard, see
	// the header comment. It covers ropes spawned or destroyed by a delegate handler fired from
	// ResolvePendingAimThrow, Prepare or Finalize.
	bTickingRopes = true;

	// Phase 1a (game thread): central collider gather — build once per frame from the registered providers and
	// fill each rope's FrameColliders through its own filter. (This replaces every rope scanning the world for
	// itself. The collider pointers belong to the provider and stay valid through this frame's solve and finalize.)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_GatherColliders);
		SCOPE_CYCLE_COUNTER(STAT_RopeSim_Gather);
		BuildFrameColliders(DeltaTime);
		// The rope index is also the physics region index in FrameRopeRegions and the provider mapping — the order was pinned when invalid entries were cleaned up above.
		for (int32 RopeIndex = 0; RopeIndex < Ropes.Num(); ++RopeIndex)
		{
			// A rope destroyed by re-entry earlier this frame is skipped. Its slot in FrameRopeRegions stays as
			// an !IsValid box, so the index correspondence is unchanged — the mutation is deferred and nothing reorders.
			URopeComponent* Rope = Ropes[RopeIndex];
			if (!IsValid(Rope))
			{
				continue;
			}
#if WITH_GAMEPLAY_DEBUGGER
			// The first point this frame that touches the rope. ResolvePendingAimThrow below can produce a
			// Flight transition, so the frame-start phase is pinned before that, which is what lets the debugger
			// header show "start → end".
			Rope->CaptureDebugFrameStartPhase();
#endif
			GatherCollidersForRope(*Rope, RopeIndex, Rope->SimFrame.FrameColliders);
			// The aiming list is gathered into its own region (the rope's AABB ∪ the aim ray), keeping a distant
			// aim target's bone colliders out of the physics list above — the separation contract on
			// FRopeSimFrameIO::AimFrameColliders.
			GatherAimCollidersForRope(*Rope, RopeIndex);
			// The HUD and preview request the Wielder registered in PrePhysics resolves here too. The Wielder's
			// next tick consumes the result, so it can be up to a frame old, but BuildFrameColliders is never
			// re-run for the HUD's sake.
			Rope->ResolvePendingAimQuery();
			// A real input does not use the HUD cache: at the moment of input the ray is resolved against this
			// same frame's aiming list, and an execute-now request is thrown from here. A montage path stores
			// the result until its notify.
			Rope->ResolvePendingGuaranteedAimThrow();
			// The aim throw resolves right after the colliders are gathered, with the ray bounds pinned at the
			// moment of input. That ordering is what makes the hit — or the FrameForward fallback — come from
			// the latest aiming list for that same request.
			Rope->ResolvePendingAimThrow();
			// The aim ray locks onto one mesh and bone, and other bones' colliders are removed here.
			// Actual and predicted contact, and the wrapping path, always use that result. An ordinary solve
			// uses the same list too, though a collision-free aim flight ignores it deliberately and solves only
			// distance and bending.
			Rope->FilterFrameCollidersForAimWrapTarget();
			TotalFrameColliders += Rope->SimFrame.FrameColliders.Num();
		}
	}

	// Phase 1b (game thread): Prepare — init and pin, plus the logic phases. The colliders were filled above.
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
		// Every rope uses the same local player camera, so the query runs once per frame, on the first rope
		// that needs it. A failed lookup is remembered as resolved too, so a world with no server and no camera
		// does not repeat it once per rope.
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

	// Phase 2: the solver step, GPU resident or CPU fallback. It solves Free, Flight, Wrapping and Wrapped;
	// Releasing is override-only, and Contacting does not dispatch at all — the per-rope decision is in
	// TryBuildResidentStep.
	if (bUseGPU)
	{
		// The GPU resident path advances each rope's persistent buffer in place every frame, with no round-trip
		// stall or slow-motion. The whip and the logic phases (Wrapping, Wrapped, Releasing) are GPU-resident
		// too: their output (OverrideFrame) goes into the override pass and the kernel applies it with no
		// reseed. A logic frame with no integration dispatches override-only, at NumSub = 0.
		// Contact detection is handled in Finalize, off the delayed mirror.
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SolveGPU);
		SCOPE_CYCLE_COUNTER(STAT_RopeSim_Solve);

		TArray<FRopeGPUResidentStep> Steps;
		Steps.Reserve(Ropes.Num());
		// GDF consumer gate: the count of active GDF ropes, which is the engine's build-on-demand signal. NumGdfRopes was hoisted to the top of the tick.
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
				// Contacting builds no new GPU step, but ReadbackNow may have left a pending step needing the
				// scene GDF. The demand has to stay up until the next view dispatch consumes that step and the
				// handoff completes.
				++NumGdfRopes;
			}
		}
		// With an active GDF rope in this scene, the custom FX system requires a GDF and the engine builds it on demand.
		if (const UWorld* World = GetWorld())
		{
			RopeGDF::SetGDFActiveCount(World->Scene, NumGdfRopes);
		}
		if (Steps.Num() > 0)
		{
		// Dispatch is deferred to view expansion (the scene graph, PreRenderBasePass), which is when the GDF parameters are valid.
			GpuSolver.EnqueueSteps(MoveTemp(Steps));
		}
	}
	else
	{
		// CPU path (the fallback): ropes are independent of one another and the collider snapshot is read-only, so this is thread-safe.
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SolveParallel);
		SCOPE_CYCLE_COUNTER(STAT_RopeSim_Solve);
		ParallelFor(Ropes.Num(), [this, DeltaTime](int32 Index)
		{
			URopeComponent* Rope = Ropes[Index];
			if (!IsValid(Rope))
			{
				return;
			}
		// On the CPU path, nothing renders from the resident buffer.
			Rope->SimFrame.bGpuSteppedThisFrame = false;
			Rope->SolveSimFrame(DeltaTime);
		});
	}

	// Phase 3 (game thread): finalize — Flight contact detection and capture (UObjects and events), plus the render dirty flags.
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_Finalize);
		SCOPE_CYCLE_COUNTER(STAT_RopeSim_Finalize);
		for (URopeComponent* Rope : Ropes)
		{
			if (!IsValid(Rope))
			{
				continue;
			}
			// Attribute the GPU detection results and fill Finalize's Flight contact source with the GPU
			// candidates. The gate is per-rope (bGpuSteppedThisFrame), not global, so only a rope that actually
			// stepped on the GPU this frame uses GPU detection. A rope that did not — over the node cap, say —
			// solved on the CPU, and forcing GPU candidates on it would leave detection missing entirely and no
			// capture possible, so FinalizeSimFrame falls back to the CPU sweep instead.
			Rope->SimFrame.bGpuContactsThisFrame = false;
			if (Rope->SimFrame.bGpuSteppedThisFrame && Rope->Phase == ERopePhase::Flight)
			{
				BuildGpuFlightCandidates(*Rope);
			}
			Rope->FinalizeSimFrame(DeltaTime);
		}
	}

	// Sleep (a still Free rope skips its solve), distance LOD (iteration falloff) and gather distance culling
	// all live in the component (UpdateSleepState, ComputeSolverLOD) and GatherCollidersForRope. A per-frame
	// cap on total solve cost is not implemented.

	// 'stat DynamicRope' — update the frame load and phase dashboard. The helper skips the traversal when the stat group is not being collected.
	RopeStats::FRopeFrameCounters FrameCounters;
	FrameCounters.NumGdfDispatched = NumGdfRopes;
	FrameCounters.FrameColliders = TotalFrameColliders;
	RopeStats::RecordFrameStats(Ropes, FrameCounters);

	// End of the traversal: the deferred rope registrations and unregistrations apply now, and mutation resumes immediately after.
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
	// Simulate in game and PIE only, excluding the editor preview and inspector worlds — which is also why the component only registers from BeginPlay.
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

void URopeSimSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);

	// Register a TG_PostPhysics tick function, replacing the old UTickableWorldSubsystem tickable. A tickable
	// was placed implicitly by the engine's TickObject ordering — after TG_PostPhysics and before
	// TG_PostUpdateWork, an implementation detail — whereas an explicit group plus the mesh tick prerequisites
	// (SetAnimPrerequisites) makes "after the animation is evaluated" a contract.
	// bAllowTickOnDedicatedServer keeps the old tickable's behaviour, which also ran on the server through the
	// CPU fallback simulation.
	SimTickFunction.Target = this;
	SimTickFunction.TickGroup = TG_PostPhysics;
	SimTickFunction.EndTickGroup = TG_PostPhysics;
	SimTickFunction.bCanEverTick = true;
	SimTickFunction.bStartWithTickEnabled = true;
	SimTickFunction.bAllowTickOnDedicatedServer = true;
	SimTickFunction.RegisterTickFunction(InWorld.PersistentLevel);

	// Register the solver with this world's scene, so the GDF integration path can find it from the scene at
	// view expansion time and dispatch it. (The scene exists for rendering by this point.) Registering is
	// harmless even with that path off — the pending queue is empty and it is a no-op.
	RopeGDF::RegisterSolver(InWorld.Scene, &GpuSolver);

	// Auto-spawn one host actor per world for the static world collision provider. The class comes from the
	// settings (ARopeController by default); setting it to None disables the auto-spawn, for placing one by
	// hand instead. The spawned actor registers URopeStaticBodyProvider through RegisterColliderProvider as
	// soon as it receives BeginPlay. DoesSupportWorldType limits this to Game and PIE, so nothing appears in
	// the editor preview world.
	// Duplicate avoidance: if the world already holds a static provider — placed by hand, say — the auto-spawn
	// stands down, which is what makes hand placement beat it. The check looks for a component instance rather
	// than a registry entry, because that would depend on registration order, and an actor placed in the level
	// is already instantiated before BeginPlay and so is found whatever the timing.
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
				TEXT("RopeSimSubsystem: a static body provider already exists, so the auto-spawn is skipped (hand placement wins)."));
		}
		else if (!Settings->StaticBodyControllerClass.IsNull())
		{
			UClass* ControllerClass = Settings->StaticBodyControllerClass.LoadSynchronous();
			if (ControllerClass)
			{
				FActorSpawnParameters SpawnParams;
				// A runtime manager — do not save it into the level.
				SpawnParams.ObjectFlags |= RF_Transient;
				// Position is irrelevant; spawn at the origin.
				SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
				SpawnedStaticBodyController = InWorld.SpawnActor<AActor>(ControllerClass, FTransform::Identity, SpawnParams);
				// The collider budget and the convex plane cap do not need injecting here: the provider reads
				// them straight from Project Settings in BuildColliders, keeping one source of truth with no
				// duplicate fields on the component.
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

	// Destroy the auto-spawned manager actor. World teardown cleans actors up anyway, but deleting it
	// explicitly makes sure nothing is left over when the subsystem re-initializes, as it does on a PIE
	// seamless travel. IsValid guards against it already being gone.
	if (IsValid(SpawnedStaticBodyController))
	{
		SpawnedStaticBodyController->Destroy();
	}
	SpawnedStaticBodyController = nullptr;

	if (const UWorld* World = GetWorld())
	{
		// Lower the demand before removing the solver, so a re-Initialize path in a live world cannot leave demand behind.
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
		// Either nothing has been read back yet, or a reseed catch-up is still in progress, so there are no GPU
		// candidates this frame and any capture waits for the next one. The source is still GPU, with an empty
		// candidate list — it does not fall back to the CPU sweep.
		Rope.SimFrame.bGpuContactsThisFrame = true;
		return;
	}

	// Collider set correspondence gate: a delayed contact's ColliderIndex refers to the set as it was at
	// **dispatch** time, while the attribution table below was rebuilt for this frame. The indices only mean
	// the same thing when the result's dispatch signature matches the current one; when they differ the contact
	// is dropped, and capture waits a frame, rather than being attributed to the wrong bone.
	// Signature 0 means unset (warm-up) and is dropped too.
	if (Contacts->AttribSig == 0 || Contacts->AttribSig != Rope.SimFrame.GpuAttribSig)
	{
		Rope.SimFrame.bGpuContactsThisFrame = true;
		return;
	}

	// Contacts arrive in slot order — actual first, predicted second — so the actual one is handled first. The
	// merge matches the CPU's AddUniqueCandidate exactly, on (node, bone, mesh), OR-ing the SourceMask and
	// taking the Source priority Guided > Actual > Free. That stops the tracker double-counting a node and
	// keeps the decision identical to the CPU's.
	for (const FRopeGPUContactResult& C : Contacts->Contacts)
	{
		// Attribute the collider index back to a (bone, mesh) pair. Anything out of range — the collider set
		// changed — is skipped, which is self-correcting. The dispatch is explicit: an unknown type is dropped
		// rather than attributed, where an else branch once let a box fall into somebody else's table.
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
			// No attribution (a non-skeletal collider), so it cannot be captured.
			continue;
		}
		// Weak, so it is null if the mesh was destroyed during the delay. The decision then proceeds on the bone alone.
		const USceneComponent* Mesh = A.Mesh.Get();

		// Merge: an existing candidate for the same (node, bone, mesh) has its SourceMask and Source priority updated rather than a new one being added.
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
		// Filled in by EvaluateRelativeMotion on the game thread.
		Cand.WrapDirectionScore = 0.0f;
		Rope.SimFrame.GpuFlightCandidates.Add(Cand);
	}
	Rope.SimFrame.bGpuContactsThisFrame = true;
}

bool URopeSimSubsystem::SyncGpuPositionsForHandoff(URopeComponent& Rope)
{
	if (!RopeGpuRuntimeAvailable())
	{
		// CPU fallback — the Sim is already current.
		return false;
	}

	FRopeSimState& S = Rope.Sim;
	TArray<FVector> Pos;
	TArray<FVector> Prev;
	uint32 Generation = 0;
	if (!GpuSolver.ReadbackNow(Rope.GetUniqueID(), Pos, Prev, Generation))
	{
		// No resident buffer, meaning the rope never stepped on the GPU, so the mirror is the truth.
		return false;
	}
	if (Generation != Rope.SimFrame.SimGeneration || Pos.Num() != S.Num() || Prev.Num() != S.Num())
	{
		// A reseed catch-up is in progress, or the node counts disagree, so stale data must not be applied.
		return false;
	}

	S.Positions = MoveTemp(Pos);
	S.PrevPositions = MoveTemp(Prev);
	// The held end (node 0) snaps to the current pin, following the same convention as the mirror.
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

	// The GPU resident path takes a solve frame (Free, Flight, Wrapping, Wrapped) or a Releasing frame that
	// carries logic output only. bSolveThisFrame and OverrideFrame were decided authoritatively in Prepare, so
	// no separate phase test is needed here. Contacting produces no output and does not dispatch at all,
	// leaving the GPU buffer frozen — the same as "no solve" on the CPU.
	const bool bGpuRope = (Rope.SimFrame.bSolveThisFrame || Rope.SimFrame.OverrideFrame.HasAny())
		&& S.Num() >= 2 && S.Num() <= FRopeGPUSolver::MaxNodes;
	// Only a rope the GPU stepped this frame may render straight from the resident PosBuf; anything stale falls back to the CPU mirror.
	Rope.SimFrame.bGpuSteppedThisFrame = bGpuRope;
	if (!bGpuRope)
	{
		// Fallback (over the node cap, say): the CPU solves the free span when bSolveThisFrame, and skips an override-only frame.
		if (Rope.SimFrame.bSolveThisFrame)
		{
			Rope.SolveSimFrame(DeltaTime);
		}
		return false;
	}

	const uint32 RopeId = Rope.GetUniqueID();

	// Mirror the previous readback into the Sim, but only when the generation matches the current one, meaning
	// the GPU has caught up with the current seed. While a catch-up is in progress right after a reseed, the
	// CPU Sim is left alone so it stays the seed source.
	if (const FRopeResidentLatest* L = GpuLatest.Find(RopeId))
	{
		if (L->Generation == Rope.SimFrame.SimGeneration && L->NumNodes == S.Num()
			&& L->Positions.Num() == S.Num() && L->PrevPositions.Num() == S.Num())
		{
			S.Positions = L->Positions;
			S.PrevPositions = L->PrevPositions;
			// Mirror the tension too, when there is any — it is only read back on a solve frame, so it can update less often than the positions, and the previous value stands otherwise.
			if (L->SegmentTension.Num() == S.Num() - 1)
			{
				S.SegmentTension = L->SegmentTension;
			}
		}
	}

	// Snap the held end (node 0) exactly onto the current pin: the GPU mirror lags one to two frames and would
	// otherwise sit away from the hand. This is a correction for render and contact only — the GPU solve
	// handles the pin itself each step through PinTarget, so the simulation is unaffected.
	if (S.bStartPinned && S.Num() > 0)
	{
		S.Positions[0] = S.StartPinTarget;
		S.PrevPositions[0] = S.StartPinPrev;
	}

	// If the mirror overwrote Prepare's logic output — an anchor position, say — reapply it to the CPU Sim
	// mirror. That keeps the best of both: the latest bone-driven logic written over a delayed free span.
	if (Rope.SimFrame.OverrideFrame.HasAny())
	{
		Rope.SimFrame.OverrideFrame.ApplyToSim(S);
	}

	// Fixed-timestep schedule from the CPU accumulator. An override-only frame (bSolveThisFrame = false) needs
	// no integration and records the override alone (NumSub = 0), which is how the CPU path treats "no solve"
	// as well. The schedule and the seed use the same solver settings.
	const FRopeSolverConfig& EffSolverCfg = Rope.SolverConfig;
	FRopeSubstepSchedule Schedule;
	Schedule.NumSub = 0;
	Schedule.FixedDt = 0.0f;
	if (Rope.SimFrame.bSolveThisFrame)
	{
		// Return the time of a dropped, undispatched step to the accumulator before building the schedule, so
		// the accumulator stays the single truth of "time simulated". After the return, RopeSolverSubsteps just
		// below clamps to MaxAccum, so catching up after a long pause is bounded by the existing slow-motion policy.
		float Refund = 0.0f;
		if (PendingSimTimeRefund.RemoveAndCopyValue(RopeId, Refund) && Refund > 0.0f)
		{
			S.TimeAccumulator += Refund;
		}
		Schedule = RopeSolverSubsteps(S, EffSolverCfg, DeltaTime);
	}

	// Assemble the resident step, self-contained. The seed data is supplied every frame, and the render thread only uploads it on a reseed.
	SeedResidentStep(OutStep, RopeId, Rope.SimFrame.SimGeneration, S, EffSolverCfg, Schedule);
	// The same per-phase inextensibility contract as the CPU SolveSimFrame. The strain limit applies to the
	// GPU's latest resident pose, which is more accurate than correcting the guide target against a delayed
	// CPU mirror.
	OutStep.MaxStretchRatio = Rope.GetEffectiveMaxStretchRatio();
	// Cover what the component boundary resolves: the auto radius (0 = the render Radius) and the GDF flag, both of which live on the component.
	OutStep.CollisionRadius = Rope.GetEffectiveCollisionRadius();
	// Solve collision and contact detection are separate contracts. Even with solve collision off,
	// PackStepColliders below still packs for detection; only the collider and GDF counts handed to the solve
	// kernel go to 0.
	OutStep.bSolveCollisions = Rope.SimFrame.bSolveCollisionsThisFrame;
	OutStep.bUseWorldGDF = Rope.bUseWorldGDF && OutStep.bSolveCollisions;
	// Distance LOD: a distant rope gets iteration falloff, computed in Prepare. CollisionPasses is clamped to Iterations during packing.
	OutStep.Iterations = Rope.GetLODScaledIterations();

	// Contact detection runs on a Flight rope only, because only a Flight rope can capture. This function is
	// reached on the GPU path alone, where GPU contacts are always on, so the single phase == Flight gate is enough.
	const bool bDetectThisRope = (Rope.Phase == ERopePhase::Flight);
	if (bDetectThisRope)
	{
		RequestContactDetection(Rope, DeltaTime, OutStep);
	}

	// Collision: classify this rope's colliders into capsules and SDFs, building the attribution table
	// alongside when detecting. Only when something will consume them — the solve substep loop or the detection
	// kernel. On an override-only frame (a Wrapped sleep, or a NumSub = 0 frame at high fps) the kernel reads no
	// colliders at all, so the flattening and upload are skipped entirely; the render-thread packing just puts
	// one dummy in the empty array. The colliders used for the wake check are separate — FrameColliders, from the gather.
	if (OutStep.NumSub > 0 || bDetectThisRope)
	{
		PackStepColliders(Rope, bDetectThisRope, OutStep);
	}

	// Inject the logic phases' output (OverrideFrame) as an override, in place of a logic-phase reseed.
	// It is exactly the data applied to the CPU Sim, with the bit mirroring guaranteed by the static_assert above.
	if (Rope.SimFrame.OverrideFrame.HasAny() && Rope.SimFrame.OverrideFrame.Flags.Num() == S.Num())
	{
		OutStep.OverrideFlags = Rope.SimFrame.OverrideFrame.Flags;
		OutStep.OverridePositions = Rope.SimFrame.OverrideFrame.Positions;
		OutStep.OverridePrevPositions = Rope.SimFrame.OverrideFrame.PrevPositions;
		OutStep.OverrideInvMass = Rope.SimFrame.OverrideFrame.InvMass;
	}

	// Inject the whip guide targets as an override. Flight only, and applied before integration.
	PackWhipOverride(Rope, OutStep);

	return true;
}

void URopeSimSubsystem::RequestContactDetection(URopeComponent& Rope, float DeltaTime, FRopeGPUResidentStep& Step) const
{
	const FRopeSimState& S = Rope.Sim;
	// The attribution table (collider index → bone and mesh) is filled in the same order PackStepColliders
	// fills Step.Capsules and Step.SDFColliders, so it is reset here first.
	Step.bDetectContacts = true;
	Step.ContactRadius = Rope.GetEffectiveContactQueryRadius();
	Step.PredictionFrames = Rope.WrapConfig.PredictiveContactFrames;
	// Detection sweep resolution, the anti-tunnelling value — from the same stored setting the CPU's MakeFlightDetectParams reads.
	Step.ContactSweepStep = Rope.WrapConfig.ContactSweepStep;
	Step.ContactMaxSweepSamples = Rope.WrapConfig.ContactMaxSweepSamples;
	// Converts a predicted contact's free-node extrapolation from substep to frame displacement.
	// Step.FixedDt (= Schedule.FixedDt = (1/60)/Substeps) was already filled by SeedResidentStep, and matches
	// FrameDeltaTime and SubstepDeltaTime in the CPU's MakeFlightDetectParams.
	Step.ContactFrameToSubstepRatio = (Step.FixedDt > KINDA_SMALL_NUMBER) ? (DeltaTime / Step.FixedDt) : 1.0f;
	Rope.SimFrame.GpuCapsuleAttribution.Reset();
	Rope.SimFrame.GpuSdfAttribution.Reset();
	Rope.SimFrame.GpuBoxAttribution.Reset();
	Rope.SimFrame.GpuConvexAttribution.Reset();

	// Predicted contact: on a frame where the whip is active, the GPU carries the guide mask and the previous,
	// current and next targets, so a guided node can be extrapolated. The same input the CPU's
	// AddPredictedContactCandidates takes.
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

// Ordered (bone, mesh) signature of the GPU attribution set. A contact delayed one to two frames carries a
// ColliderIndex, and comparing this signature between frames is what proves that index still lines up with
// this frame's attribution table — that the set and its order are unchanged.
// The full contract is on FRopeSimFrameIO::GpuAttribSig.
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
	// When detecting, the collider index → bone and mesh attribution is filled in the same order as Step.Capsules and Step.SDFColliders.
	auto MakeAttribution = [](IRopeCollider* Collider)
		{
			FRopeSimFrameIO::FGpuColliderAttribution Attr;
			const USceneComponent* Mesh = nullptr;
			Collider->GetGPUAttribution(Attr.Bone, Mesh);
			Attr.Mesh = Mesh;
			return Attr;
		};

	// Surface the silent exclusion of a collider with no GPU representation, once per session behind an
	// anti-spam latch. A custom collider that implements only the CPU contract (Query, QuerySwept) works in
	// unit tests and in the CPU fallback but is excluded from the normal runtime path, the GPU solve. Without
	// the warning that is the worst kind of trap — it works in testing and the rope goes straight through
	// things in game — so the log is part of the contract (see RopeCollider.h).
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

	// Shared convex packing (pass 1 = wrappable, pass 2 = static push-out): concatenate the body-local planes
	// into the flat pool and record the offset, the count, the rigid transform (current and previous) and
	// InvDt. Returns true on success.
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
			// Local, unit length, outward, with PlaneDot = dot(N, p) - W.
			Step.ConvexPlanes.Add(FVector4(Pl.X, Pl.Y, Pl.Z, Pl.W));
		}
		Step.Convexes.Add(Cv);
		return true;
	};

	// FrameColliders is the game-thread snapshot gathered in Prepare. Two passes: non-static (wrappable)
	// colliders first, static (world) ones after. The detection kernel only looks at capsules in
	// [0, NumDetectCapsules), so a static capsule is excluded from detection automatically — detection keeps
	// only one contact per node, and a wall contact masking a wrappable contact would silently cost the wrap
	// its capture. The solve sees all colliders from both passes.
	for (IRopeCollider* Collider : Rope.SimFrame.FrameColliders)
	{
		if (!Collider || Collider->IsWorldStatic())
		{
			continue;
		}
		FRopeGPUCapsule Cap;
		if (Collider->GetGPUCapsule(Cap.A, Cap.B, Cap.Radius))
		{
			// Frame motion (previous endpoints plus InvDt) for surface-velocity drag and relative-motion CCD. A static collider keeps the default of InvDt 0.
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
			// A wrappable box, standing in for a bone: fill in the frame motion (previous transform plus InvDt) and pack it inside the detection range.
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
			// A wrappable convex, standing in for a bone: packed inside the detection range.
			if (bDetectThisRope)
			{
				Rope.SimFrame.GpuConvexAttribution.Add(MakeAttribution(Collider));
			}
			continue;
		}
			// Non-static means only a capsule, an SDF, a box or a convex reaches the GPU — all or nothing.
		WarnGpuUnrepresented(Collider);
	}
	// The detection boundary: everything up to here is a non-static capsule.
	Step.NumDetectCapsules = Step.Capsules.Num();
	// The box detection boundary: boxes up to here can be wrapped.
	Step.NumDetectBoxes = Step.Boxes.Num();
	// The convex detection boundary: convexes up to here can be wrapped.
	Step.NumDetectConvexes = Step.Convexes.Num();

	// Guard against misattributing a delayed GPU contact. A detection result can only name colliders in the
	// prefixes above, so its signature must describe exactly those prefixes. Including the static solve-only
	// suffix made an unrelated world collider entering or leaving the rope region discard an otherwise valid
	// wrappable contact one to two frames later, even though every detection index was unchanged.
	if (bDetectThisRope)
	{
		uint32 Sig = 0x9E3779B9u;
		Sig = RopeComputeAttribSig(Rope.SimFrame.GpuCapsuleAttribution, Sig);
		Sig = RopeComputeAttribSig(Rope.SimFrame.GpuSdfAttribution, Sig);
		Sig = RopeComputeAttribSig(Rope.SimFrame.GpuBoxAttribution, Sig);
		Sig = RopeComputeAttribSig(Rope.SimFrame.GpuConvexAttribution, Sig);
		// Signature 0 is reserved for "unset", so a hash that happens to be 0 is pushed to 1 to keep it distinct from a warm-up.
		Rope.SimFrame.GpuAttribSig = (Sig == 0) ? 1u : Sig;
		// Static colliders appended below cannot invalidate the detectable set recorded on this dispatch.
		Step.AttribSig = Rope.SimFrame.GpuAttribSig;
	}

	// Pass 2: static world colliders, for the solve only. Capsules (spheres, capsules) are appended after the
	// detection boundary, and boxes have their own array. The attribution table is filled for static capsules
	// too, as None and null, to keep the indices aligned — defensive, since the detection kernel never emits an
	// out-of-range index.
	for (IRopeCollider* Collider : Rope.SimFrame.FrameColliders)
	{
		if (!Collider || !Collider->IsWorldStatic())
		{
			continue;
		}
		FRopeGPUCapsule Cap;
		if (Collider->GetGPUCapsule(Cap.A, Cap.B, Cap.Radius))
		{
			// Static, so no frame motion (InvDt stays 0).
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
			// Frame motion (previous centre and rotation plus InvDt) for surface-velocity drag and relative-motion CCD. A static collider keeps the default of InvDt 0.
			Collider->GetGPUBoxMotion(Box.PrevCenter, Box.PrevRot, Box.InvDeltaTime);
			Step.Boxes.Add(Box);
			if (bDetectThisRope)
			{
				// Static — None, since it takes no part in detection and is here only to keep the indices aligned.
				Rope.SimFrame.GpuBoxAttribution.Add(MakeAttribution(Collider));
			}
			continue;
		}
		if (!TryPackConvex(Collider))
		{
			// Static means only a capsule, a box or a convex reaches the GPU — all or nothing.
			WarnGpuUnrepresented(Collider);
			continue;
		}
		if (bDetectThisRope)
		{
			// Static — None, since it takes no part in detection and is here only to keep the indices aligned.
			Rope.SimFrame.GpuConvexAttribution.Add(MakeAttribution(Collider));
		}
	}
}

void URopeSimSubsystem::PackWhipOverride(const URopeComponent& Rope, FRopeGPUResidentStep& Step) const
{
	// Load the whip guide targets Prepare's advance computed as an override. The ordinary hard guide adds a
	// substep kinematic flag, matching the CPU fallback's FRopeKinematicTargetFrame. Aim-hit keeps its endpoint
	// solver-state blend and legacy one-shot override.
	// The Flight gate stops a stale mask being applied in another phase, and since Flight does not fill
	// OverrideFrame there is no overlap with the logic-phase packing.
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
	const bool bUseKinematicPath = !Rope.WhipGuide.HasAimTarget();
	for (int32 k = 0; k < S.Num() && k < WhipMask.Num(); ++k)
	{
		if (WhipMask[k] == 0 || !WhipCur.IsValidIndex(k))
		{
			continue;
		}
		ERopeGPUOverride Flags = ERopeGPUOverride::Position | ERopeGPUOverride::Prev;
		if (bUseKinematicPath)
		{
			Flags |= ERopeGPUOverride::KinematicPath;
		}
		Step.OverrideFlags[k] = static_cast<uint8>(Flags);
		Step.OverridePositions[k] = WhipCur[k];
		// With no previous target (an edge case), velocity is 0 — an approximation matching the CPU fallback's "previous position".
		Step.OverridePrevPositions[k] = WhipPrev.IsValidIndex(k) ? WhipPrev[k] : WhipCur[k];
	}
}
