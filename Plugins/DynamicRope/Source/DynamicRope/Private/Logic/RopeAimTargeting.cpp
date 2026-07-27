// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeAimTargeting.h"
#include "Components/SceneComponent.h"
#include "Collision/RopeCollider.h"
#include "Core/RopeWrapTarget.h"

namespace
{
	bool IsWithinAimReach(const FVector* ReachOrigin, float ReachLength, const FVector& SurfacePoint, float QueryRadius)
	{
		if (!ReachOrigin || ReachLength <= KINDA_SMALL_NUMBER)
		{
			return true;
		}

		const float AllowedReach = ReachLength + FMath::Max(0.0f, QueryRadius);
		return FVector::DistSquared(*ReachOrigin, SurfacePoint) <= FMath::Square(AllowedReach);
	}
}

float FRopeAimTargeting::ResolveEffectiveQueryRadius(const FQueryContext& Ctx, float QueryRadius)
{
	// A setting of zero uses the rope's or the contact's default thickness rather than a line ray. An explicit value sweeps at that radius.
	return QueryRadius > KINDA_SMALL_NUMBER ? QueryRadius : Ctx.FallbackQueryRadius;
}

float FRopeAimTargeting::ResolveRayLengthForReach(const FVector& RayOrigin, const FVector& AimDir,
	const FVector& ReachOrigin, float ReachLength)
{
	const FVector RayDir = AimDir.GetSafeNormal();
	if (ReachLength <= KINDA_SMALL_NUMBER || RayDir.IsNearlyZero())
	{
		return 0.0f;
	}

	const FVector RayToReach = ReachOrigin - RayOrigin;
	const float Along = FVector::DotProduct(RayToReach, RayDir);
	const float PerpSq = FMath::Max(0.0f, RayToReach.SizeSquared() - FMath::Square(Along));
	const float ReachSq = FMath::Square(ReachLength);
	if (PerpSq > ReachSq)
	{
		return 0.0f;
	}

	const float HalfChord = FMath::Sqrt(FMath::Max(0.0f, ReachSq - PerpSq));
	return FMath::Max(0.0f, Along + HalfChord);
}

FVector FRopeAimTargeting::ResolveOpenSpaceThrowEndpoint(const FQueryContext& Ctx, const FVector& Origin,
	const FVector& AimDir, float RayLength, float Clearance)
{
	const FVector RayDir = AimDir.GetSafeNormal();
	const FVector RayEnd = Origin + RayDir * FMath::Max(RayLength, 0.0f);

	FVector BlockPoint = FVector::ZeroVector;
	float BlockDistance = 0.0f;
	if (RayDir.IsNearlyZero() || !Ctx.TraceWorldBlocker ||
		!Ctx.TraceWorldBlocker(Origin, RayEnd, BlockPoint, BlockDistance))
	{
		return RayEnd;
	}

	// Stop in front of the surface rather than on it, so the tube is not left half buried once the nodes
	// are handed back to physics on landing.
	return Origin + RayDir * FMath::Max(BlockDistance - FMath::Max(Clearance, 0.0f), 0.0f);
}

bool FRopeAimTargeting::FindAimRayBoneHit(const FQueryContext& Ctx,
	const FVector& Origin, const FVector& AimDir, float RayLength, float QueryRadius, float SweepStep,
	TFunctionRef<bool(const USceneComponent*, FName)> CanWrapTarget,
	FRopeAimRayHitResult& OutHit,
	FRopeAimRayHitResult* OutBlockedHit)
{
	return FindAimRayBoneHit(Ctx, Origin, AimDir, RayLength, QueryRadius, SweepStep,
		CanWrapTarget, OutHit, OutBlockedHit, nullptr, 0.0f);
}

bool FRopeAimTargeting::FindAimRayBoneHit(const FQueryContext& Ctx,
	const FRopeAimRayThrowRequest& Request,
	TFunctionRef<bool(const USceneComponent*, FName)> CanWrapTarget,
	FRopeAimRayHitResult& OutHit,
	FRopeAimRayHitResult* OutBlockedHit)
{
	OutHit = FRopeAimRayHitResult();
	if (OutBlockedHit)
	{
		*OutBlockedHit = FRopeAimRayHitResult();
	}
	if (!Request.IsValid())
	{
		return false;
	}

	return FindAimRayBoneHit(Ctx, Request.RayOrigin, Request.RayDirection, Request.RayLength,
		Request.QueryRadius, Request.SweepStep, CanWrapTarget, OutHit, OutBlockedHit,
		&Request.ReachOrigin, Request.ReachLength);
}

bool FRopeAimTargeting::FindAimRayBoneHit(const FQueryContext& Ctx,
	const FVector& Origin, const FVector& AimDir, float RayLength, float QueryRadius, float SweepStep,
	TFunctionRef<bool(const USceneComponent*, FName)> CanWrapTarget,
	FRopeAimRayHitResult& OutHit,
	FRopeAimRayHitResult* OutBlockedHit,
	const FVector* ReachOrigin, float ReachLength)
{
	OutHit = FRopeAimRayHitResult();
	if (OutBlockedHit)
	{
		*OutBlockedHit = FRopeAimRayHitResult();
	}
	if (!Ctx.Colliders)
	{
		return false;
	}

	const FVector RayDir = AimDir.GetSafeNormal();
	const float EffectiveRayLength = RayLength > KINDA_SMALL_NUMBER ? RayLength : Ctx.FallbackRayLength;
	if (EffectiveRayLength <= KINDA_SMALL_NUMBER || RayDir.IsNearlyZero())
	{
		return false;
	}

	const FVector RayStart = Origin;
	const FVector RayEnd = RayStart + RayDir * EffectiveRayLength;
	const float EffectiveQueryRadius = ResolveEffectiveQueryRadius(Ctx, QueryRadius);
	const float EffectiveSweepStep = FMath::Clamp(SweepStep > KINDA_SMALL_NUMBER ? SweepStep : 2.0f, 0.5f, 10.0f);

	FRopeSweptQuery Query;
	Query.WorldStart = RayStart;
	Query.WorldEnd = RayEnd;
	Query.NodeRadius = EffectiveQueryRadius;
	Query.SweepStep = EffectiveSweepStep;
	Query.MaxSamples = FMath::Clamp(1 + FMath::CeilToInt(EffectiveRayLength / EffectiveSweepStep), 2, 4096);

	const FBox RayBounds(RayStart.ComponentMin(RayEnd), RayStart.ComponentMax(RayEnd));
	const FBox ExpandedRayBounds = RayBounds.ExpandBy(EffectiveQueryRadius);
	bool bFoundHit = false;
	FRopeAimRayHitResult BestHit;
	bool bFoundBlocked = false;
	FRopeAimRayHitResult BestBlocked;

	// The opaque world geometry along the ray, probed once before the collider loop. Everything past it is
	// out of sight and therefore out of aim. The tolerance is the query radius, because the swept query
	// reports a hit as soon as the ray comes within that distance of a surface, so a target resting against
	// the blocker would otherwise be judged to be behind it.
	FVector WorldBlockPoint = FVector::ZeroVector;
	float WorldBlockDistance = 0.0f;
	const bool bWorldBlocked = Ctx.TraceWorldBlocker &&
		Ctx.TraceWorldBlocker(RayStart, RayEnd, WorldBlockPoint, WorldBlockDistance);
	if (bWorldBlocked)
	{
		// The blocker is itself a blocked candidate, so aiming at a bare wall or floor lights the HUD's
		// blocked indication rather than showing nothing at all. It carries no bone or mesh, which is
		// exactly what an unwrappable collider hit reports too.
		BestBlocked = FRopeAimRayHitResult();
		BestBlocked.bHit = true;
		BestBlocked.HitWorldPos = WorldBlockPoint;
		BestBlocked.SurfacePoint = WorldBlockPoint;
		BestBlocked.Distance = WorldBlockDistance;
		bFoundBlocked = true;
	}
	const float WorldBlockTolerance = EffectiveQueryRadius;

	// Only colliders that passed the broad-phase bounds are tested with the same swept query, and the smallest distance along the ray is taken.
	for (const IRopeCollider* Collider : *Ctx.Colliders)
	{
		if (!Collider || !Collider->GetWorldBounds().Intersect(ExpandedRayBounds))
		{
			continue;
		}

		FVector HitWorldPos = FVector::ZeroVector;
		const FRopeContact Contact = Collider->QuerySwept(Query, HitWorldPos);
		if (!Contact.bHit)
		{
			continue;
		}
		if (!IsWithinAimReach(ReachOrigin, ReachLength, Contact.SurfacePoint, EffectiveQueryRadius))
		{
			continue;
		}

		const float Distance = FVector::DotProduct(HitWorldPos - RayStart, RayDir);
		if (bWorldBlocked && Distance > WorldBlockDistance + WorldBlockTolerance)
		{
			// Behind a wall or under the floor. It is dropped outright rather than demoted to a blocked
			// candidate, because the blocker in front of it is nearer and already holds that slot.
			continue;
		}
	// For the size of the aim HUD's highlight ring: an approximation of the hit collider's world bounds radius. The extent is a half size, so its length is the half diagonal.
		const float BoundsRadius = static_cast<float>(Collider->GetWorldBounds().GetExtent().Size());

	// A ray that hit something that cannot be wrapped, having no bone, no source mesh or being refused by the gate, is recorded as a blocked candidate alone.
		const bool bWrappable = !Contact.Bone.IsNone() && Contact.SourceMesh &&
			CanWrapTarget(Contact.SourceMesh, Contact.Bone);
		if (!bWrappable)
		{
			if (OutBlockedHit && (!bFoundBlocked || Distance < BestBlocked.Distance))
			{
				BestBlocked = FRopeAimRayHitResult();
				BestBlocked.bHit = true;
				BestBlocked.Bone = Contact.Bone;
				BestBlocked.Mesh = Contact.SourceMesh;
				BestBlocked.HitWorldPos = HitWorldPos;
				BestBlocked.SurfacePoint = Contact.SurfacePoint;
				BestBlocked.Normal = Contact.Normal;
				BestBlocked.Distance = Distance;
				BestBlocked.TargetBoundsRadius = BoundsRadius;
				bFoundBlocked = true;
			}
			continue;
		}

		FRopeAimRayHitResult Candidate;
		Candidate.bHit = true;
		Candidate.Bone = Contact.Bone;
		Candidate.Mesh = Contact.SourceMesh;
		Candidate.HitWorldPos = HitWorldPos;
		Candidate.SurfacePoint = Contact.SurfacePoint;
		Candidate.Normal = Contact.Normal;
		Candidate.Distance = Distance;
		Candidate.TargetBoundsRadius = BoundsRadius;
		if (!bFoundHit || Candidate.Distance < BestHit.Distance)
		{
			BestHit = Candidate;
			bFoundHit = true;
		}
	}

	if (OutBlockedHit && bFoundBlocked)
	{
		*OutBlockedHit = BestBlocked;
	}

	if (!bFoundHit)
	{
		return false;
	}

	OutHit = BestHit;
	return true;
}

bool FRopeAimTargeting::ResolveAimRayThrowContext(const FQueryContext& Ctx, const FRopeAimRayThrowRequest& Request,
	TFunctionRef<bool(const USceneComponent*, FName)> CanWrapTarget,
	FRopeThrowContext& OutContext,
	FRopeAimRayHitResult* OutHit,
	FRopeAimRayHitResult* OutBlockedHit)
{
	OutContext = Request.BaseContext;
	if (OutHit)
	{
		*OutHit = FRopeAimRayHitResult();
	}
	if (OutBlockedHit)
	{
		*OutBlockedHit = FRopeAimRayHitResult();
	}
	if (!Request.IsValid())
	{
		return false;
	}

	FRopeAimRayHitResult Hit;
	if (!FindAimRayBoneHit(Ctx, Request, CanWrapTarget, Hit, OutBlockedHit))
	{
		return false;
	}
	if (OutHit)
	{
		*OutHit = Hit;
	}

	// The final guide forward is the vector from the real throw origin to the hit, not from the ray's start point.
	const FVector HitAimDir = (Hit.HitWorldPos - OutContext.Origin).GetSafeNormal();
	if (HitAimDir.IsNearlyZero())
	{
		return false;
	}

	OutContext.FrameForward = HitAimDir;
	OutContext.bHasAimGuideHit = true;
	OutContext.AimGuideBone = Hit.Bone;
	const USceneComponent* HitMesh = Hit.Mesh;
	OutContext.AimGuideMesh = HitMesh;
	OutContext.AimGuideHitWorldPos = Hit.HitWorldPos;
	// The aim hit is also stored in the target bone's local space, so it can be restored through the bone's current
	// transform in flight and at commit and follow a moving target. A world-fixed hit position alone would leave the tip hanging in the air.
	if (HitMesh && !Hit.Bone.IsNone())
	{
		const FTransform BoneXform = ResolveBindingWorld(HitMesh, Hit.Bone);
		OutContext.AimGuideLocalHitPos = BoneXform.InverseTransformPosition(Hit.HitWorldPos);
		OutContext.bHasAimGuideLocalHit = true;
	}
	OutContext.AimGuideNormal = Hit.Normal;
	return true;
}

FBox FRopeAimTargeting::MakeAimRayQueryBounds(const FQueryContext& Ctx,
	const FVector& Origin, const FVector& AimDir, float RayLength, float QueryRadius)
{
	const FVector RayDir = AimDir.GetSafeNormal();
	const float EffectiveRayLength = RayLength > KINDA_SMALL_NUMBER ? RayLength : Ctx.FallbackRayLength;
	if (RayDir.IsNearlyZero() || EffectiveRayLength <= KINDA_SMALL_NUMBER)
	{
	// Invalid means no extension of the gather, which is the same as clearing it.
		return FBox(ForceInit);
	}

	const float EffectiveQueryRadius = QueryRadius > KINDA_SMALL_NUMBER ? QueryRadius : Ctx.FallbackQueryRadius;
	const FVector RayEnd = Origin + RayDir * EffectiveRayLength;
	return FBox(Origin.ComponentMin(RayEnd), Origin.ComponentMax(RayEnd))
		.ExpandBy(EffectiveQueryRadius);
}

void FRopeAimTargeting::SetWrapTargetLock(const FRopeThrowContext& ThrowContext)
{
	bLocked = ThrowContext.bHasAimGuideHit &&
		!ThrowContext.AimGuideBone.IsNone() && ThrowContext.AimGuideMesh.IsValid();
	TargetBone = bLocked ? ThrowContext.AimGuideBone : NAME_None;
	TargetMesh = bLocked ? ThrowContext.AimGuideMesh : nullptr;
}

bool FRopeAimTargeting::IsLockActive(ERopePhase Phase) const
{
	// The lock applies to a single throw's approach, contact and wrap path alone. The Free preview and ordinary collision after a wrap are unaffected.
	const bool bLockingPhase = Phase == ERopePhase::Flight ||
		Phase == ERopePhase::Contacting || Phase == ERopePhase::Wrapping;
	return bLockingPhase && bLocked && !TargetBone.IsNone() && TargetMesh.IsValid();
}

bool FRopeAimTargeting::IsPrimaryTarget(const USceneComponent* Mesh, FName Bone) const
{
	return bLocked && Mesh == TargetMesh.Get() && Bone == TargetBone;
}

// The permitted range is separated per resolve mode while the primary decision stays on the exact bone: treating the
// aim lock as identical to the permitted collider range would leave an assisted throw with one bone as well.
bool FRopeAimTargeting::IsWrapTarget(ERopePhase Phase, ERopeWrapResolveMode ResolveMode,
	const USceneComponent* Mesh, FName Bone) const
{
	if (!IsLockActive(Phase) || ResolveMode == ERopeWrapResolveMode::FullSimulation)
	{
		return true;
	}

	// An assisted throw's ray hit fixes the first capture, meaning the primary seed, alone. Neighbouring bones on the
	// same mesh have to remain as secondary target material for the contact tracker and as multi-bone projection
	// material for the surface vector field.
	return ResolveMode == ERopeWrapResolveMode::AssistedJudged
		? Mesh == TargetMesh.Get()
		: IsPrimaryTarget(Mesh, Bone);
}

void FRopeAimTargeting::FilterCollidersToTarget(ERopePhase Phase, ERopeWrapResolveMode ResolveMode,
	TArray<IRopeCollider*>& Colliders) const
{
	if (!IsLockActive(Phase))
	{
		return;
	}

	Colliders.RemoveAll([this, Phase, ResolveMode](const IRopeCollider* Collider)
	{
		if (!Collider)
		{
			return true;
		}
		if (Collider->IsWorldStatic())
		{
	// Static world geometry is kept, being needed for trajectory and environment collision, and the restriction to targets applies to skeletal bone colliders alone.
			return false;
		}

		FName ColliderBone = NAME_None;
		const USceneComponent* ColliderMesh = nullptr;
		Collider->GetGPUAttribution(ColliderBone, ColliderMesh);
		return !IsWrapTarget(Phase, ResolveMode, ColliderMesh, ColliderBone);
	});
}

bool FRopeAimTargeting::TakePendingQuery(FRopeAimRayThrowRequest& OutRequest)
{
	if (!PendingQuery.IsSet())
	{
		return false;
	}
	OutRequest = PendingQuery.GetValue();
	PendingQuery.Reset();
	return true;
}

bool FRopeAimTargeting::GetLatestQueryResult(FRopeAimRayQueryResult& OutResult) const
{
	if (!LatestQueryResult.IsSet())
	{
		return false;
	}
	OutResult = LatestQueryResult.GetValue();
	OutResult.RestoreMeshPointers();
	return true;
}

bool FRopeAimTargeting::TakePendingThrow(FRopeAimRayThrowRequest& OutRequest)
{
	if (!PendingThrow.IsSet())
	{
		return false;
	}
	OutRequest = PendingThrow.GetValue();
	PendingThrow.Reset();
	return true;
}
