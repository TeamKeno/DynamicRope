// Copyright 2026 TeamKeno. All Rights Reserved.
//
// World subsystem that drives the simulation of every active URopeComponent from one place, instead
// of each component ticking itself. It is the single orchestration point: it gathers colliders
// centrally once per frame (provider registry plus region mapping), runs the three stages
// Prepare -> Solve (parallel across ropes, batched on the GPU) -> Finalize, and applies sleep and
// LOD. A per-frame total solve budget remains a possible extension.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"
#include "Subsystems/WorldSubsystem.h"
// FTickFunction, for the TG_PostPhysics tick.
#include "Engine/EngineBaseTypes.h"
// FRopeGPUSolver (DynamicRopeShaders): the asynchronous GPU solve instance.
#include "RopeGPUSolver.h"
// FRopeWrappedEventInfo and ERopeReleaseReason, the payloads of the central wrap and release signals
// below.
#include "Core/RopeLifecycleTypes.h"
#include "RopeSimSubsystem.generated.h"

class URopeComponent;
class UActorComponent;
class USkeletalMeshComponent;
class USceneComponent;
class IRopeCollider;
class AActor;

/**
 * Central native (non-Blueprint) signals fired whenever any rope in the world establishes or
 * releases a wrap. Unlike a rope's per-instance Blueprint delegates
 * (URopeComponent::OnRopeWrapped and OnRopeReleased), these let the target side react by
 * subscribing once, without knowing in advance which rope will wrap it, and without walking every
 * rope in the world each frame. The wrap payload already carries the mesh as a weak pointer, and the
 * release signal carries the mesh that was wrapped at the moment of release, since the release
 * delegate itself does not. Listeners use that mesh to decide whether the event concerns them.
 * The signals are valid for the lifetime of the subsystem; register and unregister from the
 * listener's BeginPlay and EndPlay.
 *
 * Both signals also identify the rope involved: FRopeWrappedEventInfo::Rope for wraps, and the Rope
 * argument for releases. Several ropes can wrap one target at the same time, so subscribers must
 * count active engagements per (mesh, rope) pair rather than per mesh. Reverting the reaction as
 * soon as one rope releases would ignore the ropes still attached.
 */
DECLARE_MULTICAST_DELEGATE_OneParam(FRopeWrappedNotify, const FRopeWrappedEventInfo& /*Info*/);
DECLARE_MULTICAST_DELEGATE_FourParams(FRopeReleasedNotify, const URopeComponent* /*Rope*/, const USceneComponent* /*WrappedMesh*/, FName /*Bone*/, ERopeReleaseReason /*Reason*/);

/**
 * Tick function that drives the subsystem from TG_PostPhysics. An explicit tick group plus skeletal
 * mesh tick prerequisites turn "simulate the rope after the bone transforms are evaluated" into a
 * contract, rather than relying implicitly on where the engine happens to call tickable objects.
 * Ordering within the group is handled by the prerequisites: a skeletal mesh component holds its
 * parallel animation completion task with DontCompleteUntil, so the prerequisite alone also
 * guarantees the pose buffer has been flipped to the latest pose.
 */
USTRUCT()
struct DYNAMICROPE_API FRopeSimTickFunction : public FTickFunction
{
	GENERATED_BODY()

	/** The subsystem being driven, which lives as long as the world. The tick function is registered
	 *  only between OnWorldBeginPlay and Deinitialize. */
	class URopeSimSubsystem* Target = nullptr;

	virtual void ExecuteTick(float DeltaTime, ELevelTick TickType, ENamedThreads::Type CurrentThread,
		const FGraphEventRef& MyCompletionGraphEvent) override;
	virtual FString DiagnosticMessage() override;
	virtual FName DiagnosticContext(bool bDetailed) override;
};

template <>
struct TStructOpsTypeTraits<FRopeSimTickFunction> : public TStructOpsTypeTraitsBase2<FRopeSimTickFunction>
{
	enum { WithCopy = false };
};

struct FRopeSimSubsystemTestSeam;

UCLASS()
class DYNAMICROPE_API URopeSimSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	/** Adds and removes an active rope from the simulation list. Called from the component's
	 *  BeginPlay and EndPlay. */
	void RegisterRope(URopeComponent* Rope);
	void UnregisterRope(URopeComponent* Rope);

	/**
	 * Read-only view of the ropes this subsystem drives this frame; the two calls above are the only
	 * way in and out. It exists so diagnostic screens do not have to find ropes in the world
	 * themselves: a full UObject scan is expensive regardless of how many ropes exist, and worse, a
	 * screen opened to measure performance would then weigh down what it is measuring.
	 * The array can contain invalid entries that are destroyed and awaiting garbage collection, so
	 * consumers must filter with IsValid; Tick only prunes them on frame boundaries.
	 */
	const TArray<TObjectPtr<URopeComponent>>& GetRegisteredRopes() const { return Ropes; }

	/**
	 * Central wrap and release signals, described on the delegates above. They are fired alongside a
	 * rope broadcasting its own events, and are subscribed to by target-side reaction components such
	 * as URopeRagdollResponseComponent. With no subscribers they cost essentially nothing.
	 */
	FRopeWrappedNotify OnAnyRopeWrapped;
	FRopeReleasedNotify OnAnyRopeReleased;

	/**
	 * Adds and removes a collider provider, an actor component implementing IRopeColliderProvider,
	 * from the central registry. Called from the provider's BeginPlay and EndPlay. This replaces each
	 * rope scanning the world for itself with one central build per frame.
	 */
	void RegisterColliderProvider(UActorComponent* Provider);
	void UnregisterColliderProvider(UActorComponent* Provider);

	/** The world's rope sim subsystem, valid in game and PIE worlds and nullptr elsewhere. */
	static URopeSimSubsystem* Get(const UWorld* World);

	/** The GPU-resident solver, which lives as long as the world. The scene proxy uses it to obtain
	 *  the resident position buffer SRV. */
	FRopeGPUSolver* GetGpuSolver() { return &GpuSolver; }

	/**
	 * Synchronizes positions precisely for a wrap handoff. The CPU mirror (Sim) of a GPU-resident rope
	 * lags one to two frames behind, so this performs a single synchronous readback on entering the
	 * Wrapping phase to give the seed the latest positions.
	 * A no-op returning false when the GPU solver is off, there is no resident buffer, or the seed
	 * generation does not match.
	 * It blocks waiting for the GPU to go idle, so call it once per event only.
	 */
	bool SyncGpuPositionsForHandoff(URopeComponent& Rope);

	/** Drives one simulation frame. Called by FRopeSimTickFunction from TG_PostPhysics; tests may call
	 *  it directly. */
	void Tick(float DeltaTime);

	/** Legacy compatibility API. Immediate provider re-gathering was removed, so this changes no state
	 *  and returns false. */
	UE_DEPRECATED(5.7, "Immediate collider refresh was removed. Queue aim work for the normal subsystem gather.")
	bool RefreshAimFrameCollidersForImmediateQuery(URopeComponent& Rope);

	//~ UWorldSubsystem
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;
	//~ Registers the tick function and the scene-to-solver mapping, which the view extension uses to
	//~ find the solver on the global distance field path. Deinitialize undoes both.
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;
	virtual void Deinitialize() override;

private:
#if WITH_DEV_AUTOMATION_TESTS
	friend struct FRopeSimSubsystemTestSeam;
#endif

	/** The TG_PostPhysics tick function, registered between the world's BeginPlay and Deinitialize.
	 *  Its prerequisites are managed by SetAnimPrerequisites below. */
	FRopeSimTickFunction SimTickFunction;

	/**
	 * Adds and removes the skeletal mesh ticks of the source component's owning actor, whether a rope
	 * or a provider, as prerequisites of SimTickFunction. If the actor's mesh set changes between
	 * registering and unregistering, a leftover entry is harmless because FTickPrerequisite is weak
	 * and simply skipped.
	 *
	 * Several consumers can require the same mesh, such as a rope and a collider provider on one
	 * actor, or several ropes, and AddPrerequisite is unique, so duplicate adds collapse into one
	 * entry. Removal must therefore be counted as well: calling RemovePrerequisite whenever any one
	 * consumer leaves would also drop the "simulate after animation is evaluated" guarantee for the
	 * consumers that remain, and the PostPhysics gather would then read the previous frame's bone
	 * transforms. AnimPrereqRefCount counts consumers per mesh and only adds on 0 to 1 and removes on
	 * 1 to 0.
	 */
	void SetAnimPrerequisites(const UActorComponent* Source, bool bAdd);

	/** Consumer count per mesh for the prerequisites above. Weak, because the key expires when the
	 *  actor dies first; expired entries are cleaned up on the next add. */
	TMap<TWeakObjectPtr<USkeletalMeshComponent>, int32> AnimPrereqRefCount;

	/** Registered active ropes. Components are UObjects, so this is garbage-collection tracked. */
	UPROPERTY(Transient)
	TArray<TObjectPtr<URopeComponent>> Ropes;

	/**
	 * Ledger of the GPU rope IDs of registered ropes. It parallels Ropes but deliberately outlives a
	 * dead component. When a rope disappears without UnregisterRope, because its actor was destroyed
	 * and collected, Tick's invalid-entry pruning only drops it from the array, which would leave its
	 * resident VRAM and readback allocated until the world shuts down, with the pointer already gone
	 * and no way to recover the rope ID. Keeping the IDs separately makes the leak exactly the set
	 * difference: present in the ledger, absent from the live ropes.
	 */
	TSet<uint32> RegisteredRopeIds;

	/** Reclaims the GPU resident resources of dead ropes using that set difference, immediately after
	 *  Tick prunes invalid entries. */
	void ReleaseGpuResourcesForDeadRopes();

	/**
	 * Simulation time, in seconds and keyed by rope ID, from steps that were replaced because the view
	 * extension could not consume them. It is collected from the solver every tick and returned to the
	 * TimeAccumulator on the frame that rope actually solves, at which point the entry is erased. Like
	 * a logic frame, time unused this tick is preserved until the next opportunity; discarding it
	 * on the spot would lose the very time that needs returning.
	 * No separate burst protection is needed here: RopeSolverSubsteps already clamps the accumulator
	 * to MaxAccum.
	 */
	TMap<uint32, float> PendingSimTimeRefund;

	/**
	 * Re-entrancy guard for the Tick iteration. If a delegate handler run during the Prepare or
	 * Finalize loops (OnRopeWrapped, OnRopeReleased, OnRopePhaseChanged) spawns or destroys an actor
	 * carrying a rope component, Register or UnregisterRope would mutate Ropes immediately, causing
	 * two failures: the ranged-for iterator is invalidated, which asserts in development builds and
	 * dangles in Shipping, and the FrameRopeRegions index mapping collapses, so a rope receives
	 * another rope's region colliders. Mutations during iteration are therefore deferred to the lists
	 * below while bTickingRopes is set, and applied afterwards in ApplyDeferredRopeChanges. The loops
	 * additionally guard with IsValid to skip ropes destroyed during this frame. The deferred lists
	 * live within a single tick and are emptied immediately, so raw pointers are safe.
	 */
	bool bTickingRopes = false;
	TArray<URopeComponent*> DeferredRopeRegister;
	TArray<URopeComponent*> DeferredRopeUnregister;

	/** Applies the rope registrations and unregistrations deferred during iteration, at the end of
	 *  Tick once bTickingRopes is clear. */
	void ApplyDeferredRopeChanges();

	/** Registered collider providers, that is components implementing IRopeColliderProvider.
	 *  Garbage-collection tracked. */
	UPROPERTY(Transient)
	TArray<TObjectPtr<UActorComponent>> ColliderProviders;

	/**
	 * The rope manager actor spawned automatically during OnWorldBeginPlay, which hosts the static
	 * world collision provider. Null when the StaticBodyControllerClass setting is None, which opts
	 * out of the automatic spawn. Destroyed in Deinitialize.
	 */
	UPROPERTY(Transient)
	TObjectPtr<AActor> SpawnedStaticBodyController = nullptr;

	/** Colliders built centrally once per frame: the owning actor of each provider plus its collider
	 *  pointers. The pointers are owned by the provider and are only valid for that frame. */
	struct FFrameProviderColliders
	{
		/** The provider component's owning actor, used for source filtering. */
		AActor* Owner = nullptr;

		/** A static world provider, which is exempt from the per-rope owner exclusion
		 *  (ProvidesWorldStaticColliders). */
		bool bWorldStatic = false;

		/** The pointers filled in by provider->GatherColliders, referring to the provider's backing
		 *  storage. */
		TArray<IRopeCollider*> Colliders;

		/**
		 * Per-collider source actors, parallel to Colliders. Populated only when the provider filled in
		 * Gather.ColliderSourceActors and the lengths match; otherwise this is empty and the decision is
		 * made per provider. It exists so the per-rope owner exclusion can be decided per body rather
		 * than per provider: a static body provider sweeps the world and picks up shapes attached to
		 * the rope's own actor, which a provider-level exemption alone cannot stop from pushing that
		 * rope.
		 */
		TArray<const AActor*> SourceActors;

		/**
		 * The pool index mapping per region, where a region corresponds to a rope index, returned by
		 * the provider alongside the gather. When bHasRegionMapping is set, assigning colliders per
		 * rope is just consuming this list, which removes the O(ropes x pool) bounds re-cull.
		 */
		TArray<TArray<int32>> RegionIndices;
		bool bHasRegionMapping = false;

		/** World bounds cache per collider, used only on the fallback path for providers without a
		 *  region mapping, where culling is done per rope by distance. */
		TArray<FBox> Bounds;
	};
	TArray<FFrameProviderColliders> FrameProviders;

	/**
	 * This frame's region list, filled in by BuildFrameColliders. It is the single source that makes
	 * the provider gather and the per-rope assignment use the same boxes. The layout is twice the
	 * number of ropes, N:
	 *  - [0, N)   physics regions, each a rope's tight AABB plus a margin, one-to-one with Ropes.
	 *  - [N, 2N)  aim regions, the physics region unioned with the aim ray AABB, or invalid when not
	 *             aiming.
	 * See the comment on FRopeSimFrameIO::AimFrameColliders for why aim regions are kept separate.
	 * Slots without a region are left as invalid boxes so the mapping indices returned by providers
	 * still line up; providers skip them.
	 */
	TArray<FBox> FrameRopeRegions;

	/** Maps a rope index to its aim region index in the layout above. The physics region index is the
	 *  rope index itself. */
	int32 AimRegionIndexOf(int32 RopeIndex) const { return Ropes.Num() + RopeIndex; }

	/**
	 * The order regions are processed in this frame, active ropes first: phases that are in use, then
	 * awake Free ropes, then sleeping ones, then invalid entries. Providers with a global extraction
	 * budget, such as the static body provider, scan in this order so clutter around an idle rope
	 * cannot consume the budget and starve an active rope of collisions. It only affects order; region
	 * indices are unchanged, so mappings are unaffected.
	 */
	TArray<int32> FrameRegionGatherOrder;

	/** Gathers colliders once from every registered provider, before Prepare. Providers are handed the
	 *  physics and aim region lists. */
	void BuildFrameColliders(float DeltaTime);

	/**
	 * Collects one region's colliders from the central build. By default every provider is included
	 * except the rope's own owner, which can be opted back in with bIncludeOwnerColliders.
	 * RegionIndex is an index into FrameRopeRegions, which is the rope index for a physics region and
	 * AimRegionIndexOf for an aim region, and must match the region index in the provider mapping. The
	 * owner exclusion, the static budget and the cross-actor rules apply identically to both region
	 * kinds.
	 */
	void GatherCollidersForRope(const URopeComponent& Rope, int32 RegionIndex, TArray<IRopeCollider*>& OutColliders) const;

	/**
	 * Fills a rope's aim collider snapshot (SimFrame.AimFrameColliders) from its aim region. When the
	 * rope is not aiming, and the aim bounds are therefore invalid, the snapshot is cleared
	 * immediately so no pointer outlives its frame.
	 */
	void GatherAimCollidersForRope(URopeComponent& Rope, int32 RopeIndex) const;

	/**
	 * A rope's broad-phase query bounds: the tight AABB over its current and previous positions plus
	 * the contact and prediction margins. Computed in one place so the region handed to providers and
	 * the per-rope collider culling use the same box; an invalid box is returned when there is none.
	 * With bIncludeAimRay the aim ray AABB is unioned in to form the aim region. When the rope is not
	 * aiming, and the ray bounds are invalid, no aim region is needed and an invalid box is returned,
	 * which gathers nothing and leaves the aim list empty.
	 */
	static FBox ComputeRopeQueryBounds(const URopeComponent& Rope, float DeltaTime,
		bool bIncludeAimRay = false);

	/**
	 * The GPU-resident solver, which advances persistent per-rope buffers in place every frame. It
	 * holds instance state, so there is one per world.
	 * It is the only runtime path whenever a renderable RHI exists; without one, as in a cook, under
	 * -nullrhi, or on a server, the CPU solver takes over automatically.
	 */
	FRopeGPUSolver GpuSolver;

	/** Cache of the latest, slightly delayed positions per rope ID, obtained through GetLatest. Each
	 *  frame's update is mapped back onto the corresponding Sim state. */
	TMap<uint32, FRopeResidentLatest> GpuLatest;

	/** Cache of the latest, slightly delayed GPU contact detection results per rope ID, obtained
	 *  through GetLatestContacts. Attributed before Finalize. */
	TMap<uint32, FRopeResidentContacts> GpuLatestContacts;

	/**
	 * Rebuilds the GPU detection results, which are collider indices, into FRopeContactCandidate
	 * entries through the rope's attribution table and fills them into the component, where they are
	 * the contact source for the Flight phase in Finalize. The delayed results are only valid when
	 * they match the current seed generation.
	 */
	void BuildGpuFlightCandidates(URopeComponent& Rope);

	/** Applies an already-available, generation-matched GPU readback to the CPU mirror and snaps its held
	 *  end to the current pin. This never waits for the render thread. Returns true only when a snapshot was
	 *  copied; the pin correction is still applied when no matching snapshot exists. */
	bool ApplyLatestGpuMirror(URopeComponent& Rope);

	/**
	 * Builds one rope's GPU resident step. Returns true after filling in OutStep when the rope belongs
	 * on the GPU, adding it to the dispatch list; returns false when it falls back, for example
	 * because it exceeds the node count, having already solved it on the CPU. Also sets
	 * Rope.SimFrame.bGpuSteppedThisFrame.
	 */
	bool TryBuildResidentStep(URopeComponent& Rope, float DeltaTime, FRopeGPUResidentStep& OutStep);

	/** Sets the contact detection request, and the whip prediction inputs, on a Flight rope's step and
	 *  resets its attribution table. */
	void RequestContactDetection(URopeComponent& Rope, float DeltaTime, FRopeGPUResidentStep& Step) const;

	/** Classifies this rope's frame colliders into capsules and SDFs and loads them onto the step. With
	 *  bDetectThisRope the attribution table is filled in at the same time. */
	void PackStepColliders(URopeComponent& Rope, bool bDetectThisRope, FRopeGPUResidentStep& Step) const;

	/** Packs the Flight whip guide targets onto the step as overrides, applied before integration. A
	 *  no-op outside the Flight phase. */
	void PackWhipOverride(const URopeComponent& Rope, FRopeGPUResidentStep& Step) const;

	/**
	 * One-shot warning latch for colliders silently excluded from the GPU because they have no GPU
	 * representation (GetGPUCapsule, GetGPUSDF, GetGPUBox or GetGPUConvex). It surfaces the trap where
	 * a custom IRopeCollider implements only the CPU contract, so it works in tests, which run on the
	 * CPU, and is ignored at runtime, which runs on the GPU. Mutable because it is set from
	 * PackStepColliders, which is const; it is diagnostic state with no bearing on logic.
	 */
	mutable bool bWarnedGpuUnrepresentedCollider = false;
};
