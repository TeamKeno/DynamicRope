// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The aim ray targeting logic and state, as a class with no UObject dependency. It provides the
// swept ray bone query used by the wielder's aiming flow (FindAimRayBoneHit), resolves the aim throw
// context, computes the AABB that extends collider gathering, and owns the per-throw lock on the
// wrap primary (mesh plus bone), the permitted range per resolve mode, the pending HUD and preview
// query, and the aim throw queue.
// UObject context, meaning the collider snapshot, the fallback dimensions and the CanWrapTarget
// gate, is injected as parameters per call, so it can be unit tested without a world. The entry
// points that involve the StartFreshThrow transition, namely QueueAimRayThrow and
// ResolvePendingAimThrow, stay on URopeComponent as orchestration.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "Core/RopeLifecycleTypes.h"
#include "Core/RopeThrowTypes.h"

class IRopeCollider;
class USceneComponent;

/** The nearest wrappable bone hit the wielder's aim ray found on a rope collider or SDF. */
struct FRopeAimRayHitResult
{
	// Whether the mesh, bone and surface data below are all valid.
	bool bHit = false;
	// The bone to lock as the wrap target for this throw.
	FName Bone = NAME_None;
	// The component the bone belongs to, which distinguishes another actor sharing the same bone name.
	const USceneComponent* Mesh = nullptr;
	// The world position at which the swept ray first entered the collider.
	FVector HitWorldPos = FVector::ZeroVector;
	// The actual surface point obtained by SDF or contact projection.
	FVector SurfacePoint = FVector::ZeroVector;
	// The outward normal at the surface point.
	FVector Normal = FVector::UpVector;
	// The projected distance from the ray origin to HitWorldPos.
	float Distance = 0.0f;
	// An approximation of the hit collider's world bounds radius, as the half-diagonal length, used to
	// size the aiming HUD's highlight ring.
	float TargetBoundsRadius = 0.0f;
	// Whether the hit came from a collider that is a wrap target at all, meaning a skeletal bone or a
	// wrap target component, rather than plain level geometry. On a blocked hit it separates "a target
	// exists here but cannot be wrapped", which is worth reporting, from a bare floor or wall, which is
	// not, and which the aiming HUD therefore leaves as an ordinary untargeted crosshair.
	bool bWrapCandidate = false;
};

/** Run once a guaranteed prepared throw has been resolved by the normal gather. The callback may
 *  modify the values that will be used for the actual throw. */
DECLARE_DELEGATE_OneParam(FRopeAimPreparedDelegate, FRopePreparedThrowPreview&);

/** An aim throw request the wielder freezes at the moment of input, to be resolved right after
 *  RopeSimSubsystem's latest collider gather. */
struct FRopeAimRayThrowRequest
{
	FRopeThrowContext BaseContext;
	FVector RayOrigin = FVector::ZeroVector;
	FVector RayDirection = FVector::ForwardVector;
	float RayLength = 0.0f;
	// The reference for deciding whether the rope actually reaches. It is stored separately because
	// RayOrigin, at the camera or the bounds centre, can differ from the throw origin at the hand
	// socket.
	FVector ReachOrigin = FVector::ZeroVector;
	float ReachLength = 0.0f;
	float QueryRadius = 0.0f;
	float SweepStep = 2.0f;
	// Called immediately after a guaranteed prepared throw is resolved and before it executes, for
	// work such as storing the owner-local guide frame.
	FRopeAimPreparedDelegate OnPrepared;
	// Called once StartFreshThrow has completed. A C++-only completion notification that avoids
	// referencing the wielder directly.
	FSimpleDelegate OnResolved;
	// Called when queueing, or the actual execution, was refused.
	FSimpleDelegate OnRejected;

	bool IsValid() const
	{
		return RayLength > KINDA_SMALL_NUMBER && !RayDirection.IsNearlyZero();
	}
};

/** The aiming result resolved right after the normal collider gather, consumed by the HUD and the
 *  preview on the following wielder tick. */
struct FRopeAimRayQueryResult
{
	FRopeAimRayThrowRequest Request;
	FRopeThrowContext ResolvedContext;
	FRopeAimRayHitResult Hit;
	FRopeAimRayHitResult BlockedHit;
	bool bHitTarget = false;
	// The published hit contract keeps raw pointers, so these weak references verify their lifetime
	// while the result is cached across a frame boundary.
	TWeakObjectPtr<const USceneComponent> CachedHitMesh = nullptr;
	TWeakObjectPtr<const USceneComponent> CachedBlockedMesh = nullptr;

	bool IsValid() const { return Request.IsValid(); }
	void CaptureMeshReferences()
	{
		CachedHitMesh = Hit.Mesh;
		CachedBlockedMesh = BlockedHit.Mesh;
	}
	void RestoreMeshPointers()
	{
		Hit.Mesh = CachedHitMesh.Get();
		BlockedHit.Mesh = CachedBlockedMesh.Get();
	}
};

class DYNAMICROPE_API FRopeAimTargeting
{
public:
	/** The context shared by the queries, assembled from this frame's values by the caller,
	 *  URopeComponent. */
	struct FQueryContext
	{
		// This frame's aiming collider snapshot, from URopeComponent::GetAimQueryColliders. Valid only
		// for the duration of the call.
		const TArray<IRopeCollider*>* Colliders = nullptr;
		// The fallback length used when RayLength is unspecified, meaning at or below 0: the larger of
		// the current Sim.RopeLength and the initial RopeLength.
		float FallbackRayLength = 0.0f;
		// The fallback radius used when QueryRadius is unspecified, meaning at or below 0: the larger of
		// the tube radius and WrapConfig.ContactQueryRadius.
		float FallbackQueryRadius = 0.0f;

		/** The opaque world geometry probe, injected from URopeComponent because this class has no world.
		 *  It returns true when the segment is blocked, filling in the nearest blocking point and its
		 *  distance from Start. The rope's own collider pool cannot answer this: the static body provider
		 *  extracts simple collision only, so landscapes and complex-collision-only floors never appear in
		 *  it, and an engine line trace is the only thing that sees them.
		 *  Leaving it unset means nothing blocks, which is what the unit tests rely on. */
		TFunction<bool(const FVector& Start, const FVector& End, FVector& OutBlockPoint, float& OutDistance)> TraceWorldBlocker;
	};

	//~ Queries. They do not change state, so they are static and free of side effects.
	// Visualization does not live here: the Gameplay Debugger's Rope category ([J] aim) draws it by
	// reading the wielder's FRopeAimHudSample, which keeps the debug entry point single; see
	// RopeDebugSubsystem.h. This class performs pure queries only.

	/** Resolves an unspecified QueryRadius, meaning at or below 0, to the fallback radius. It is the
	 *  single source for that rule so the query and the visualization see the same radius; copying the
	 *  ternary into call sites would let them diverge silently. */
	static float ResolveEffectiveQueryRadius(const FQueryContext& Ctx, float QueryRadius);

	/** The furthest ray distance at which the aim ray still passes through the sphere the rope can
	 *  actually reach, defined by ReachOrigin and ReachLength. It keeps the aim ray from being judged
	 *  too short or too long when RayOrigin differs from the hand, that is the start of the rope. */
	static float ResolveRayLengthForReach(const FVector& RayOrigin, const FVector& AimDir,
		const FVector& ReachOrigin, float ReachLength);

	/** The endpoint for a throw into open space, meaning one with no aim target: the ray end at RayLength,
	 *  pulled back to Clearance in front of the surface when Ctx.TraceWorldBlocker reports the path
	 *  blocked. Without the clamp the endpoint can sit below the floor, and the guided throw replays node
	 *  positions with the solver switched off, so the rope would pass straight through it. */
	static FVector ResolveOpenSpaceThrowEndpoint(const FQueryContext& Ctx, const FVector& Origin,
		const FVector& AimDir, float RayLength, float Clearance);

	/** Finds the nearest wrappable mesh and bone along the ray using a swept SDF query: a broad phase,
	 *  then QuerySwept, then the minimum distance travelled along the ray. A candidate that fails the
	 *  CanWrapTarget gate is treated as absent.
	 *  A candidate lying behind opaque world geometry, as reported by Ctx.TraceWorldBlocker, is treated
	 *  as absent in the same way: what cannot be seen cannot be aimed at, so the aimed throw the mode
	 *  guarantees never starts through a wall.
	 *  OutBlockedHit is optional: the nearest hit where the ray struck a collider that cannot be
	 *  wrapped, whether because it has no bone, no source mesh, was refused by the gate, or was cut off
	 *  by the world blocker. It is independent of the return value, which reports whether a wrappable
	 *  hit exists, and drives the blocked indication on the aiming HUD. Its bWrapCandidate says whether
	 *  the blocker was a wrap target refused this frame or merely level geometry, which is what lets the
	 *  HUD stay neutral on a floor or a wall. */
	static bool FindAimRayBoneHit(const FQueryContext& Ctx,
		const FVector& Origin, const FVector& AimDir, float RayLength, float QueryRadius, float SweepStep,
		TFunctionRef<bool(const USceneComponent*, FName)> CanWrapTarget,
		FRopeAimRayHitResult& OutHit,
		FRopeAimRayHitResult* OutBlockedHit = nullptr);

	static bool FindAimRayBoneHit(const FQueryContext& Ctx,
		const FRopeAimRayThrowRequest& Request,
		TFunctionRef<bool(const USceneComponent*, FName)> CanWrapTarget,
		FRopeAimRayHitResult& OutHit,
		FRopeAimRayHitResult* OutBlockedHit = nullptr);

	/** Resolves an aim request into a hit context, filling in the frame forward and the aim guide
	 *  fields. With no hit, OutContext falls back to BaseContext and false is returned. The optional
	 *  outputs return the hit and blocked results from that same single sweep. */
	static bool ResolveAimRayThrowContext(const FQueryContext& Ctx, const FRopeAimRayThrowRequest& Request,
		TFunctionRef<bool(const USceneComponent*, FName)> CanWrapTarget,
		FRopeThrowContext& OutContext,
		FRopeAimRayHitResult* OutHit = nullptr,
		FRopeAimRayHitResult* OutBlockedHit = nullptr);

	/** Builds the AABB that extends collider gathering to cover what the aim ray tests. Invalid input
	 *  returns FBox(ForceInit), meaning no extension, which is equivalent to clearing
	 *  SimFrame.AimRayColliderQueryBounds. */
	static FBox MakeAimRayQueryBounds(const FQueryContext& Ctx,
		const FVector& Origin, const FVector& AimDir, float RayLength, float QueryRadius);

	//~ Wrap target lock, held for the duration of one throw ---------------------
	/** Sets or clears the lock from the aim guide hit on a throw context, once on entering
	 *  StartFreshThrow or a prepared throw. */
	void SetWrapTargetLock(const FRopeThrowContext& ThrowContext);

	/** Whether the lock applies in this phase. It applies only to one throw's approach, contact and
	 *  wrap, that is Flight, Contacting and Wrapping, leaving the Free preview and ordinary collision
	 *  after Wrapped untouched. */
	bool IsLockActive(ERopePhase Phase) const;

	/** Whether this is exactly the primary (mesh, bone) the aim ray locked. Consumed when assisted
	 *  aiming pins its capture and dominant target. */
	bool IsPrimaryTarget(const USceneComponent* Mesh, FName Bone) const;

	// Assisted multi-bone contract: the aimed bone decides the primary, while other bones on the same
	// mesh remain candidates for the wrap path. Merging the two into one exact-bone condition would
	// make the other bones' colliders disappear again in assisted mode.
	/** Whether (mesh, bone) is a permitted wrap target under the current resolve mode. Everything
	 *  passes while the lock is inactive.
	 *  Assisted permits other bones on the primary's mesh as candidates and for path projection, while
	 *  Guaranteed permits the exact bone only. */
	bool IsWrapTarget(ERopePhase Phase, ERopeWrapResolveMode ResolveMode,
		const USceneComponent* Mesh, FName Bone) const;

	/** While the lock is active, removes the skeletal colliders that do not match the resolve mode's
	 *  policy. Assisted keeps every bone on the same mesh and Guaranteed keeps the exact bone only.
	 *  Static world geometry is always kept, since it is needed for trajectory and environment
	 *  collision. A no-op while the lock is inactive. */
	void FilterCollidersToTarget(ERopePhase Phase, ERopeWrapResolveMode ResolveMode,
		TArray<IRopeCollider*>& Colliders) const;

	const USceneComponent* GetLockedTargetMesh() const { return TargetMesh.Get(); }
	FName GetLockedTargetBone() const { return TargetBone; }

	//~ The pending HUD and preview query, registered during PrePhysics and resolved right after the
	//~ PostPhysics collider gather.
	void QueuePendingQuery(const FRopeAimRayThrowRequest& Request) { PendingQuery = Request; }
	bool TakePendingQuery(FRopeAimRayThrowRequest& OutRequest);
	void StoreLatestQueryResult(FRopeAimRayQueryResult Result)
	{
		Result.CaptureMeshReferences();
		LatestQueryResult = MoveTemp(Result);
	}
	bool GetLatestQueryResult(FRopeAimRayQueryResult& OutResult) const;
	void ResetQuery()
	{
		PendingQuery.Reset();
		LatestQueryResult.Reset();
	}

	//~ The pending aim throw, frozen at the moment of input and consumed right after the collider
	//~ gather.
	void QueuePendingThrow(const FRopeAimRayThrowRequest& Request) { PendingThrow = Request; }
	bool HasPendingThrow() const { return PendingThrow.IsSet(); }

	/** Takes the pending request by value and clears it, returning false when there is none. The
	 *  caller's StartFreshThrow, in ResolvePendingAimThrow, resets the transient state, so the request
	 *  must be carried forward as the value taken here. */
	bool TakePendingThrow(FRopeAimRayThrowRequest& OutRequest);

	void ResetPendingThrow() { PendingThrow.Reset(); }

private:
	static bool FindAimRayBoneHit(const FQueryContext& Ctx,
		const FVector& Origin, const FVector& AimDir, float RayLength, float QueryRadius, float SweepStep,
		TFunctionRef<bool(const USceneComponent*, FName)> CanWrapTarget,
		FRopeAimRayHitResult& OutHit,
		FRopeAimRayHitResult* OutBlockedHit,
		const FVector* ReachOrigin, float ReachLength);

	// The target established from the ray hit when the throw started. The mesh is stored alongside the
	// bone to rule out another actor with the same bone name.
	bool bLocked = false;
	FName TargetBone = NAME_None;
	TWeakObjectPtr<const USceneComponent> TargetMesh = nullptr;

	// The request the wielder registered this frame, and the most recent result resolved by the normal
	// gather. The result is kept until the next wielder tick.
	TOptional<FRopeAimRayThrowRequest> PendingQuery;
	TOptional<FRopeAimRayQueryResult> LatestQueryResult;

	// Preserves the ray and frame from the moment of input, consumed once right after the subsystem's
	// collider gather.
	TOptional<FRopeAimRayThrowRequest> PendingThrow;
};
