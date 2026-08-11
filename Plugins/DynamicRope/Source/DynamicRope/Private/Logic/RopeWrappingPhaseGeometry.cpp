// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Logic/RopeWrappingPhase.h"
#include "Components/SceneComponent.h"
// ResolveBindingWorld, the single point that resolves a wrap binding, whether a bone, a socket or a
// component, into a transform.
#include "Core/RopeWrapTarget.h"
#include "DynamicRopeLog.h"
#include "Collision/RopeCollider.h"
// TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)
#include "ProfilingDebugging/CpuProfilerTrace.h"
// RopeMath::AnyTangentFromNormal, included here to avoid a duplicate definition in the unity build.
#include "RopeMathHelpers.h"

#pragma region Wrapping Geometry and Axis Resolution

bool FRopeWrappingPhase::ComputeBuiltPathWrapAngle(const FRopeSimState& Sim, const FContext& Ctx, float& OutAngleDeg) const
{
	OutAngleDeg = 0.0f;
	if (State.Anchors.Num() == 0 && State.Path.Num() == 0)
	{
		return false;
	}

	// Both the sequential surface vector field and the composite analytic helix prefer the real phase
	// accumulated during the path build. Only a path that has not advanced a single step falls back to the
	// single-axis approximation below.
	if (State.PathAccumulatedAngleRad > KINDA_SMALL_NUMBER)
	{
		OutAngleDeg = FMath::RadiansToDegrees(State.PathAccumulatedAngleRad);
		return true;
	}

	const FRopeSurfaceAnchor& LatchAnchor = State.LatchAnchor;
	const USceneComponent* Mesh = ResolveWrappingMesh(State, LatchAnchor);
	if (!Mesh || LatchAnchor.Bone.IsNone())
	{
		return false;
	}

	FVector AxisOrigin = FVector::ZeroVector;
	FVector AxisDirection = FVector::ForwardVector;
	if (!ResolveWrappingAxis(LatchAnchor, Ctx, AxisOrigin, AxisDirection))
	{
		return false;
	}
	OrientWrappingAxisByTail(LatchAnchor, Sim, Mesh, AxisDirection);

	const FTransform BoneXform = ResolveBindingWorld(Mesh, LatchAnchor.Bone);
	const FVector LatchSurfaceWorld = BoneXform.TransformPosition(LatchAnchor.LocalSurfacePosition);
	const float LatchAxisDistance = FVector::DotProduct(LatchSurfaceWorld - AxisOrigin, AxisDirection);
	const FVector LatchAxisPoint = AxisOrigin + AxisDirection * LatchAxisDistance;
	const float HelixRadius = (LatchSurfaceWorld - LatchAxisPoint).Size();
	if (HelixRadius <= KINDA_SMALL_NUMBER)
	{
		return false;
	}

	float LastBuiltDistance = 0.0f;
	for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
	{
		LastBuiltDistance = FMath::Max(LastBuiltDistance, Anchor.RopeDistance);
	}
	if (State.Path.Num() > 0)
	{
		LastBuiltDistance = FMath::Max(LastBuiltDistance, State.Path.Last().DistanceFromLatch);
	}

	// However uneven the real surface vector field path was, the failure test looks only at the wrapped
	// angle accumulated against the helix. It returns degrees: a number of turns is not used, because a
	// turn demands a rope of 2*pi*r and therefore scales the criterion with the target's size; see the
	// comment on FRopeWrapConfig::FailedWrapMinAngleDeg.
	if (State.bPathUsesPoseSpaceIsland && State.Path.Num() > 1)
	{
		// An independent helix using an automatic pitch and entry radius already stores its real
		// accumulated phase. Recomputing it from the fixed config pitch would make the angle in the commit
		// log and the quality gate disagree with the path.
		OutAngleDeg = FMath::RadiansToDegrees(
			FMath::Abs(State.PathCompositeSweepAngleRad));
		return true;
	}

	const float PitchScale = Ctx.Config.WrappingHelixPitchScale;
	const float LengthScale = FMath::Sqrt(1.0f + PitchScale * PitchScale);
	const float CircumferenceDistance = LastBuiltDistance / FMath::Max(LengthScale, KINDA_SMALL_NUMBER);
	const float AngleRadians = CircumferenceDistance / FMath::Max(HelixRadius, KINDA_SMALL_NUMBER);
	OutAngleDeg = FMath::RadiansToDegrees(FMath::Abs(AngleRadians));
	return true;
}

bool FRopeWrappingPhase::ComputeWrapEnclosureCoverage(float& OutCoverageDeg) const
{
	OutCoverageDeg = 0.0f;
	if (State.Path.Num() < 2)
	{
		return false;
	}

	const FVector Axis = State.PathAxisDirection.GetSafeNormal();
	if (Axis.IsNearlyZero())
	{
		return false;
	}

	const auto ComputeRadial = [this, &Axis](const FVector& Point, FVector& OutRadial) -> bool
	{
		const FVector Offset = Point - State.PathAxisOrigin;
		OutRadial = Offset - Axis * FVector::DotProduct(Offset, Axis);
		return OutRadial.Normalize(KINDA_SMALL_NUMBER);
	};

	// The angle of each path point about the axis, measured from the first non-degenerate point over a
	// half-open turn. Bridge points are included, since the direction a chord crosses is also a direction
	// the rope blocks.
	FVector RefRadial = FVector::ZeroVector;
	TArray<float, TInlineAllocator<128>> AngleDegrees;
	for (const FRopeWrapPathPoint& Point : State.Path)
	{
		FVector Radial = FVector::ZeroVector;
		if (!ComputeRadial(Point.SurfaceWorld, Radial))
		{
			continue;
		}

		if (RefRadial.IsNearlyZero())
		{
			RefRadial = Radial;
		}

		const float AngleRad = FMath::Atan2(
			static_cast<float>(FVector::DotProduct(Axis, FVector::CrossProduct(RefRadial, Radial))),
			static_cast<float>(FVector::DotProduct(RefRadial, Radial)));
		AngleDegrees.Add(FMath::RadiansToDegrees(AngleRad));
	}

	if (AngleDegrees.Num() < 2)
	{
		return false;
	}

	// After sorting, find the largest angular gap, both between neighbours and across the wrap-around at
	// the ends. The coverage is 360 minus that gap: points spread evenly around the axis leave a gap no
	// larger than one step and converge on 360, while a half hook leaves the opposite side entirely empty
	// and its coverage is lower by exactly that much.
	AngleDegrees.Sort();
	float MaxGapDeg = 360.0f - (AngleDegrees.Last() - AngleDegrees[0]);
	for (int32 Index = 1; Index < AngleDegrees.Num(); ++Index)
	{
		MaxGapDeg = FMath::Max(MaxGapDeg, AngleDegrees[Index] - AngleDegrees[Index - 1]);
	}

	OutCoverageDeg = FMath::Clamp(360.0f - MaxGapDeg, 0.0f, 360.0f);
	return true;
}

bool FRopeWrappingPhase::FindGuidePlaneAxis(const FRopeSurfaceAnchor& LatchAnchor, const FContext& Ctx,
	const USceneComponent* Mesh, FVector& OutAxisOrigin, FVector& OutAxisDirection)
{
	if (!Mesh || LatchAnchor.Bone.IsNone() || !Ctx.bHasGuidePlaneNormal)
	{
		return false;
	}

	const FVector GuidePlaneNormal = Ctx.GuidePlaneNormal.GetSafeNormal();
	if (GuidePlaneNormal.IsNearlyZero())
	{
		return false;
	}

	// Under CaptureTravelPlane the axis origin is placed at the centre of the contact region captured at
	// that moment rather than at the latch bone, so that with contact spanning several bones or targets,
	// such as two legs, the wrap radius is measured about the centre of the pair rather than one of them.
	// The origin is fixed at capture time and therefore does not move when the axis is reseeded on a bone
	// transition.
	// BoneCenteredGuidePlane uses the bone location as before, which leaves single-bone assisted behaviour
	// unchanged.
	if (Ctx.Config.WrappingAxisSource == ERopeWrappingAxisSource::CaptureTravelPlane &&
		Ctx.TravelFrame && Ctx.TravelFrame->bValid)
	{
		FVector Origin = Ctx.TravelFrame->RegionCenter;

		// Correcting for the contact cluster, only in the wrap mode that bridges between separate targets:
		// the region centre is the mean of the contact points at the moment of first contact, so when
		// capture happens the instant the rope touches the first leg, which it does with the default
		// minimum latch count of one, it sits on that one leg. An axis passing through a target makes
		// orbiting that target alone the forward winding direction, which silences the departure gate and
		// leaves the path unable to cross to the other leg.
		// Neighbouring targets that were not touched are still present in the collider snapshot, so the
		// centres of nearby colliders on the same mesh, within the bridge distance, are averaged to put the
		// axis through the centre of the cluster, that is the pair of legs.
		// The component along the axis is discarded: an axis is a line, so only the perpendicular component
		// means anything.
		const float ClusterRadius = Ctx.Config.WrappingMaxGapBridgeDistance;
		if (ClusterRadius > 0.0f)
		{
			FVector CenterSum = FVector::ZeroVector;
			int32 CenterCount = 0;
			for (const IRopeCollider* Collider : Ctx.Colliders)
			{
				if (!Collider)
				{
					continue;
				}

				FName ColliderBone = NAME_None;
				const USceneComponent* ColliderMesh = nullptr;
				Collider->GetGPUAttribution(ColliderBone, ColliderMesh);
				if (Mesh != nullptr && ColliderMesh != nullptr && ColliderMesh != Mesh)
				{
					continue;
				}

				FVector Center = FVector::ZeroVector;
				if (!GetColliderCenter(*Collider, Center))
				{
					continue;
				}

				const FVector Delta = Center - Origin;
				const float AlongAxis = static_cast<float>(FVector::DotProduct(Delta, GuidePlaneNormal));
				if (FMath::Abs(AlongAxis) > ClusterRadius ||
					(Delta - GuidePlaneNormal * AlongAxis).Size() > ClusterRadius)
				{
					continue;
				}

				CenterSum += Center;
				++CenterCount;
			}

			if (CenterCount > 0)
			{
				const FVector ClusterDelta = CenterSum / static_cast<float>(CenterCount) - Origin;
				Origin += ClusterDelta - GuidePlaneNormal *
					static_cast<float>(FVector::DotProduct(ClusterDelta, GuidePlaneNormal));
			}
		}

		OutAxisOrigin = Origin;
	}
	else
	{
		OutAxisOrigin = ResolveBindingWorld(Mesh, LatchAnchor.Bone).GetLocation();
	}
	OutAxisDirection = GuidePlaneNormal;
	return true;
}

bool FRopeWrappingPhase::GetColliderCenter(const IRopeCollider& Collider, FVector& OutCenter)
{
	FVector CapA = FVector::ZeroVector;
	FVector CapB = FVector::ZeroVector;
	float CapRadius = 0.0f;
	if (Collider.GetGPUCapsule(CapA, CapB, CapRadius))
	{
		OutCenter = (CapA + CapB) * 0.5f;
		return true;
	}

	FVector BoxCenter = FVector::ZeroVector;
	FVector BoxHalf = FVector::ZeroVector;
	FQuat BoxRot = FQuat::Identity;
	if (Collider.GetGPUBox(BoxCenter, BoxRot, BoxHalf))
	{
		OutCenter = BoxCenter;
		return true;
	}

	FRopeSDFColliderView SDFView;
	if (Collider.GetGPUSDF(SDFView))
	{
		OutCenter = SDFView.BoneToWorld.TransformPosition(SDFView.LocalMin + SDFView.LocalSize * 0.5);
		return true;
	}

	// A wrappable convex, from a wrap target's full set: its local bounds centre transformed into world
	// space by the rigid transform.
	TConstArrayView<FPlane> ConvexPlanes;
	FBox ConvexLocalBounds(ForceInit);
	FQuat ConvexRot = FQuat::Identity;
	FQuat ConvexPrevRot = FQuat::Identity;
	FVector ConvexTrans = FVector::ZeroVector;
	FVector ConvexPrevTrans = FVector::ZeroVector;
	float ConvexInvDt = 0.0f;
	if (Collider.GetGPUConvex(ConvexPlanes, ConvexLocalBounds, ConvexRot, ConvexTrans,
			ConvexPrevRot, ConvexPrevTrans, ConvexInvDt)
		&& ConvexLocalBounds.IsValid)
	{
		OutCenter = ConvexRot.RotateVector(ConvexLocalBounds.GetCenter()) + ConvexTrans;
		return true;
	}

	return false;
}

namespace
{
	// The candidate wrap axes a collider's shape offers, as direction plus extent pairs: a capsule offers
	// its segment axis with the full length as the extent; a box its three rotated basis axes with the
	// full side lengths; a convex its rigid-rotated local-bounds axes likewise. The extent is what makes
	// a log pick its long axis: girth wrapping winds about the direction the shape is longest in.
	void AppendColliderAxisCandidates(const IRopeCollider& Collider,
		TArray<TPair<FVector, float>, TInlineAllocator<9>>& Out)
	{
		FVector CapA = FVector::ZeroVector;
		FVector CapB = FVector::ZeroVector;
		float CapRadius = 0.0f;
		if (Collider.GetGPUCapsule(CapA, CapB, CapRadius))
		{
			FVector Dir = CapB - CapA;
			const float SegLen = static_cast<float>(Dir.Size());
			if (Dir.Normalize(KINDA_SMALL_NUMBER))
			{
				Out.Emplace(Dir, SegLen + 2.0f * CapRadius);
			}
			return;
		}

		FVector BoxCenter = FVector::ZeroVector;
		FVector BoxHalf = FVector::ZeroVector;
		FQuat BoxRot = FQuat::Identity;
		if (Collider.GetGPUBox(BoxCenter, BoxRot, BoxHalf))
		{
			Out.Emplace(BoxRot.GetAxisX(), 2.0f * static_cast<float>(BoxHalf.X));
			Out.Emplace(BoxRot.GetAxisY(), 2.0f * static_cast<float>(BoxHalf.Y));
			Out.Emplace(BoxRot.GetAxisZ(), 2.0f * static_cast<float>(BoxHalf.Z));
			return;
		}

		TConstArrayView<FPlane> ConvexPlanes;
		FBox ConvexLocalBounds(ForceInit);
		FQuat ConvexRot = FQuat::Identity;
		FQuat ConvexPrevRot = FQuat::Identity;
		FVector ConvexTrans = FVector::ZeroVector;
		FVector ConvexPrevTrans = FVector::ZeroVector;
		float ConvexInvDt = 0.0f;
		if (Collider.GetGPUConvex(ConvexPlanes, ConvexLocalBounds, ConvexRot, ConvexTrans,
				ConvexPrevRot, ConvexPrevTrans, ConvexInvDt)
			&& ConvexLocalBounds.IsValid)
		{
			const FVector Extent = ConvexLocalBounds.GetExtent();
			Out.Emplace(ConvexRot.GetAxisX(), 2.0f * static_cast<float>(Extent.X));
			Out.Emplace(ConvexRot.GetAxisY(), 2.0f * static_cast<float>(Extent.Y));
			Out.Emplace(ConvexRot.GetAxisZ(), 2.0f * static_cast<float>(Extent.Z));
		}
	}

	// The shape-extent axis for a non-skeletal target: among the axis candidates offered by the
	// wrappable colliders attributed to the latch bone, pick the longest one that stays at least
	// ~60 degrees away from the latch surface normal. Girth wrapping winds about the direction the
	// shape is longest in, so a log picks its long axis however it is rotated; an end hit rejects
	// the near-normal long axis and falls to the larger cross axis. Returns false when no attributed
	// candidate survives (no colliders in the snapshot, or every axis is too close to the normal).
	// The axis direction may come from the attributed colliders' union, but its transverse origin comes
	// from the collider nearest the latch. A single virtual bone can own disconnected convex pieces (the
	// crossbar and two posts of RopeLog_UE); using their union centre moves the axis off the contacted log
	// and turns a spiral into an almost straight surface path.
	bool TryResolveShapeExtentAxis(const FRopeSurfaceAnchor& LatchAnchor,
		const FRopeWrappingPhase::FContext& Ctx, const USceneComponent* Mesh,
		const FTransform& BoneXform, FVector& OutAxisOrigin, FVector& OutAxisDirection)
	{
		const FVector NormalWorld = BoneXform.TransformVectorNoScale(LatchAnchor.LocalNormal)
			.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);

		constexpr float MaxParallelToNormal = 0.5f;
		FVector BestShapeAxis = FVector::ZeroVector;
		float BestShapeExtent = 0.0f;
		const auto ConsiderCandidate = [&](const FVector& Dir, float Extent)
		{
			const float ParallelToNormal =
				FMath::Abs(static_cast<float>(FVector::DotProduct(Dir, NormalWorld)));
			if (ParallelToNormal <= MaxParallelToNormal && Extent > BestShapeExtent)
			{
				BestShapeExtent = Extent;
				BestShapeAxis = Dir;
			}
		};

		// The union of the attributed colliders, in the component-oriented frame: authored simple
		// collision is often a decomposition into chunks (a log split lengthwise into near-cubes), so
		// the per-piece bounds lose the overall long axis entirely - only the union still knows the
		// prop is long. World AABBs of the pieces are gathered into a rigid component frame so the
		// union's axes follow the prop's rotation.
		const FTransform CompRigid(Mesh->GetComponentQuat(), Mesh->GetComponentLocation());
		FBox LocalUnion(ForceInit);
		const FVector LatchSurfaceWorld = BoneXform.TransformPosition(LatchAnchor.LocalSurfacePosition);
		FVector ContactedShapeCenter = BoneXform.GetLocation();
		float ClosestSurfaceDistance = TNumericLimits<float>::Max();
		bool bHasContactedShapeCenter = false;
		for (const IRopeCollider* Collider : Ctx.Colliders)
		{
			if (!Collider)
			{
				continue;
			}
			FName ColliderBone = NAME_None;
			const USceneComponent* ColliderMesh = nullptr;
			Collider->GetGPUAttribution(ColliderBone, ColliderMesh);
			if (ColliderBone != LatchAnchor.Bone || (ColliderMesh && Mesh && ColliderMesh != Mesh))
			{
				continue;
			}

			// Per-piece shape axes: a single rotated elem (a diagonal capsule) offers its own exact
			// axis, which the axis-aligned union would inflate.
			TArray<TPair<FVector, float>, TInlineAllocator<9>> Candidates;
			AppendColliderAxisCandidates(*Collider, Candidates);
			for (const TPair<FVector, float>& Candidate : Candidates)
			{
				ConsiderCandidate(Candidate.Key, Candidate.Value);
			}

			const FBox WorldBounds = Collider->GetWorldBounds();
			if (WorldBounds.IsValid)
			{
				// Project the latch onto each attributed piece to identify the piece that actually owns
				// the contact, while still retaining every piece for the overall direction estimate.
				const float ProjectionRadius = static_cast<float>(
					FVector::Distance(LatchSurfaceWorld, WorldBounds.GetCenter()) +
					WorldBounds.GetExtent().Size() + 1.0);
				const FRopeSurfaceProjection Projection =
					Collider->ProjectToSurface(LatchSurfaceWorld, ProjectionRadius);
				if (Projection.bHit && Projection.Distance < ClosestSurfaceDistance)
				{
					ClosestSurfaceDistance = Projection.Distance;
					ContactedShapeCenter = WorldBounds.GetCenter();
					bHasContactedShapeCenter = true;
				}

				FVector Corners[8];
				WorldBounds.GetVertices(Corners);
				for (const FVector& Corner : Corners)
				{
					LocalUnion += CompRigid.InverseTransformPosition(Corner);
				}
			}
		}
		if (LocalUnion.IsValid)
		{
			const FVector UnionExtent = LocalUnion.GetExtent();
			ConsiderCandidate(CompRigid.GetUnitAxis(EAxis::X), 2.0f * static_cast<float>(UnionExtent.X));
			ConsiderCandidate(CompRigid.GetUnitAxis(EAxis::Y), 2.0f * static_cast<float>(UnionExtent.Y));
			ConsiderCandidate(CompRigid.GetUnitAxis(EAxis::Z), 2.0f * static_cast<float>(UnionExtent.Z));
		}
		if (BestShapeExtent <= 0.0f)
		{
			return false;
		}
		OutAxisDirection = BestShapeAxis.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		if (bHasContactedShapeCenter)
		{
			// Translation along an axis does not change its line. Preserve the union's axial coordinate
			// while taking the two radial coordinates from the actually contacted shape.
			const FVector UnionCenter = LocalUnion.IsValid
				? CompRigid.TransformPosition(LocalUnion.GetCenter())
				: ContactedShapeCenter;
			OutAxisOrigin = ContactedShapeCenter + OutAxisDirection *
				FVector::DotProduct(UnionCenter - ContactedShapeCenter, OutAxisDirection);
		}
		else
		{
			OutAxisOrigin = LocalUnion.IsValid
				? CompRigid.TransformPosition(LocalUnion.GetCenter())
				: BoneXform.GetLocation();
		}
		return true;
	}
}

bool FRopeWrappingPhase::ResolveWrappingAxis(const FRopeSurfaceAnchor& LatchAnchor, const FContext& Ctx,
	FVector& OutAxisOrigin, FVector& OutAxisDirection) const
{
	const auto LogAxisSource = [&](const TCHAR* Source, const USceneComponent* MeshForLog)
	{
		UE_LOG(LogRopeWrap, VeryVerbose,
			TEXT("[%s] Wrapping axis: source=%s, bone=%s, mesh=%s, origin=%s, dir=%s"),
			*Ctx.OwnerName,
			Source,
			*LatchAnchor.Bone.ToString(),
			*GetNameSafe(MeshForLog),
			*OutAxisOrigin.ToString(),
			*OutAxisDirection.ToString());
	};

	// Under CaptureTravelPlane the travel plane normal axis is pinned to the centre of the captured contact
	// region or collider cluster. It is the mode that keeps composite wrapping across several bones from
	// being dragged towards the location of one particular latch bone.
	bool bTriedCaptureTravelPlane = false;
	if (Ctx.Config.WrappingAxisSource == ERopeWrappingAxisSource::CaptureTravelPlane)
	{
		bTriedCaptureTravelPlane = true;
		const USceneComponent* CaptureMesh = ResolveWrappingMesh(State, LatchAnchor);
		if (CaptureMesh && FindGuidePlaneAxis(LatchAnchor, Ctx, CaptureMesh, OutAxisOrigin, OutAxisDirection))
		{
			LogAxisSource(TEXT("CaptureTravelPlane"), CaptureMesh);
			return true;
		}
	}

	const USceneComponent* Mesh = ResolveWrappingMesh(State, LatchAnchor);
	if (!Mesh || LatchAnchor.Bone.IsNone())
	{
		return false;
	}

	const FName ParentBone = RopeWrapTargets::GetParentTargetKey(Mesh, LatchAnchor.Bone);
	const FVector BoneLocation = ResolveBindingWorld(Mesh, LatchAnchor.Bone).GetLocation();

	// A non-skeletal target under the default axis source wraps about what its shape dictates, ahead of
	// the guide plane: the travel-plane normal is the throw's swing direction, so on a static prop it
	// makes the wrap direction depend on where the rope was thrown from - a log approached along its
	// length wound about a cross axis and the rope lay down the log. The shape-extent axis is the
	// physically wrappable direction regardless of the throw. Skeletal targets and an explicitly
	// configured CaptureTravelPlane, the multi-target composite intent, keep the travel-plane priority.
	if (Ctx.Config.WrappingAxisSource == ERopeWrappingAxisSource::BoneCenteredGuidePlane &&
		!RopeWrapTargets::IsSkeletalTarget(Mesh))
	{
		const FTransform ShapeBoneXform = ResolveBindingWorld(Mesh, LatchAnchor.Bone);
		if (TryResolveShapeExtentAxis(LatchAnchor, Ctx, Mesh, ShapeBoneXform, OutAxisOrigin, OutAxisDirection))
		{
			LogAxisSource(TEXT("ShapeExtent"), Mesh);
			return true;
		}
	}

	// Under BoneCenteredGuidePlane the same travel plane normal is stood at the latch bone's location.
	// Had the mode been CaptureTravelPlane it would already have been attempted and failed above, so the
	// same input is not retried.
	if (!bTriedCaptureTravelPlane && FindGuidePlaneAxis(LatchAnchor, Ctx, Mesh, OutAxisOrigin, OutAxisDirection))
	{
		LogAxisSource(TEXT("BoneCenteredGuidePlane"), Mesh);
		return true;
	}

	// Where no rope plane axis can be produced, a skeletal target falls back to the bone-to-parent axis.
	if (!ParentBone.IsNone())
	{
		const FVector ParentLocation = ResolveBindingWorld(Mesh, ParentBone).GetLocation();
		const FVector Axis = BoneLocation - ParentLocation;
		if (!Axis.IsNearlyZero())
		{
			OutAxisOrigin = ParentLocation;
			OutAxisDirection = Axis.GetSafeNormal();
			LogAxisSource(TEXT("BoneParentAxis"), Mesh);
			return true;
		}
	}

	const FTransform BoneXform = ResolveBindingWorld(Mesh, LatchAnchor.Bone);

	// A static or non-skeletal target has no bone graph, so the axis comes from the shape itself: among
	// the axis candidates offered by the wrappable colliders attributed to this bone (capsule axis, box
	// and convex bounds axes, each with its extent), pick the longest one that stays sufficiently
	// perpendicular to the latch surface normal. Girth wrapping winds about the direction the shape is
	// longest in, so a log picks its long axis however it is rotated - the old component-basis rule
	// compared only perpendicularity to the normal, which ties on a lying cylinder (a top hit is
	// perpendicular to both horizontal axes) and then picked X by iteration order.
	if (!RopeWrapTargets::IsSkeletalTarget(Mesh))
	{
		// Reached under CaptureTravelPlane with no travel frame, or with no guide plane at all: the
		// shape still decides first, then the component basis.
		if (TryResolveShapeExtentAxis(LatchAnchor, Ctx, Mesh, BoneXform, OutAxisOrigin, OutAxisDirection))
		{
			LogAxisSource(TEXT("ShapeExtentFallback"), Mesh);
			return true;
		}

		const FVector NormalWorld = BoneXform.TransformVectorNoScale(LatchAnchor.LocalNormal)
			.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);

		// No attributed shape candidate survived (no colliders in the snapshot, or every axis is too
		// close to the normal): keep the previous component-basis rule, whichever of X, Y and Z is most
		// perpendicular to the latch normal.
		FVector BestAxis = BoneXform.GetUnitAxis(EAxis::Z);
		float BestParallel = TNumericLimits<float>::Max();
		for (const EAxis::Type CandidateAxis : { EAxis::X, EAxis::Y, EAxis::Z })
		{
			const FVector AxisWorld = BoneXform.GetUnitAxis(CandidateAxis);
			const float ParallelToNormal = FMath::Abs(FVector::DotProduct(AxisWorld, NormalWorld));
			if (ParallelToNormal < BestParallel)
			{
				BestParallel = ParallelToNormal;
				BestAxis = AxisWorld;
			}
		}
		OutAxisOrigin = BoneLocation;
		OutAxisDirection = BestAxis.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		LogAxisSource(TEXT("StaticBasisFallback"), Mesh);
		return true;
	}

	OutAxisOrigin = BoneLocation;
	OutAxisDirection = BoneXform.GetUnitAxis(EAxis::X).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
	LogAxisSource(TEXT("BoneLocalXFallback"), Mesh);
	return true;
}

void FRopeWrappingPhase::OrientWrappingAxisByTail(const FRopeSurfaceAnchor& LatchAnchor, const FRopeSimState& Sim,
	const USceneComponent* Mesh, FVector& InOutAxisDirection) const
{
	if (!Mesh || !Sim.Positions.IsValidIndex(LatchAnchor.NodeIndex) || InOutAxisDirection.IsNearlyZero())
	{
		return;
	}

	// The sign of a shape or guide-plane axis is arbitrary. Resolve that ambiguity from the rope itself,
	// measured in the character/component frame: a tail on the character's upper side receives the axis
	// hemisphere pointing away from CharacterUp, and a lower tail receives the opposite hemisphere. This
	// remains stable when skeleton parenting does not describe the visual top/bottom of the wrapped island.
	const FVector CharacterUp = Mesh->GetUpVector().GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	const FVector LatchWorld = Sim.Positions[LatchAnchor.NodeIndex];
	float WeightedTailUpDistance = 0.0f;
	float TotalWeight = 0.0f;

	const auto AddProbe = [&](int32 NodeIndex, float Weight)
	{
		if (!Sim.Positions.IsValidIndex(NodeIndex) || Weight <= 0.0f)
		{
			return;
		}

		WeightedTailUpDistance += FVector::DotProduct(
			Sim.Positions[NodeIndex] - LatchWorld, CharacterUp) * Weight;
		TotalWeight += Weight;
	};

	AddProbe(LatchAnchor.NodeIndex + 1, 2.0f);
	AddProbe(Sim.Num() - 1, 1.0f);

	if (TotalWeight <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	const float TailUpDistance = WeightedTailUpDistance / TotalWeight;
	const float AxisUpAlignment = FVector::DotProduct(InOutAxisDirection, CharacterUp);
	if (!FMath::IsNearlyZero(TailUpDistance) && !FMath::IsNearlyZero(AxisUpAlignment) &&
		TailUpDistance * AxisUpAlignment > 0.0f)
	{
		InOutAxisDirection *= -1.0f;
	}
}

void FRopeWrappingPhase::ReseedWrappingAxisOnBoneTransition(FName Bone, const USceneComponent* Mesh, const FContext& Ctx)
{
	if (Bone.IsNone())
	{
		return;
	}

	// A synthetic anchor filling in only what ResolveWrappingAxis contractually needs, namely the bone and
	// mesh plus a bone-local surface frame. Moving the current path point's surface frame into the new
	// bone's local space makes the fallbacks used when the collider shape axis fails, whether the guide
	// plane or the basis axes, work from the new bone's location and the current normal.
	FRopeSurfaceAnchor AxisAnchor;
	AxisAnchor.Bone = Bone;
	AxisAnchor.Mesh = Mesh;
	const FTransform BoneXform = ResolveBindingWorld(Mesh, Bone);
	AxisAnchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(State.PathSurfaceWorld);
	AxisAnchor.LocalNormal = BoneXform.InverseTransformVectorNoScale(State.PathNormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	AxisAnchor.LocalTangent = BoneXform.InverseTransformVectorNoScale(State.PathTangentWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);

	FVector NewAxisOrigin = State.PathAxisOrigin;
	FVector NewAxisDirection = State.PathAxisDirection;
	if (!ResolveWrappingAxis(AxisAnchor, Ctx, NewAxisOrigin, NewAxisDirection))
	{
		// Deriving an axis for the new bone failed, so it continues with the previous bone's axis, which is
		// the same fallback as behaving with a single fixed axis.
		return;
	}

	// Sign alignment: neighbouring axes along a bone chain generally continue in the same direction, so an
	// axis opposed to the previous one is flipped. That keeps the pitch drift direction, meaning which way
	// the wrap slides along the axis, from reversing at a transition. OrientWrappingAxisByTail is a
	// heuristic based on the tail position at latch time and is unsuitable for re-resolving mid-path.
	if (FVector::DotProduct(NewAxisDirection, State.PathAxisDirection) < 0.0f)
	{
		NewAxisDirection *= -1.0f;
	}

	// The radial and circumferential directions are recomputed from the current surface point and the
	// winding is re-elected, so the new field turns about the new axis in the direction the path is
	// currently travelling. Without re-electing the winding here, the wrap direction could reverse
	// depending on the geometry at the transition point.
	const float AxisDistance = FVector::DotProduct(State.PathSurfaceWorld - NewAxisOrigin, NewAxisDirection);
	const FVector AxisPoint = NewAxisOrigin + NewAxisDirection * AxisDistance;
	const FVector Radial = (State.PathSurfaceWorld - AxisPoint)
		.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathNormalWorld);
	FVector CircumferenceDir = FVector::CrossProduct(NewAxisDirection, Radial)
		.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathCircumferenceDir);
	const float NewWindingSign =
		FVector::DotProduct(CircumferenceDir, State.PathTangentWorld) < 0.0f ? -1.0f : 1.0f;

	State.PathAxisOrigin = NewAxisOrigin;
	State.PathAxisDirection = NewAxisDirection;
	State.PathLatchRadial = Radial;
	State.PathWindingSign = NewWindingSign;
	State.PathCircumferenceDir = CircumferenceDir * NewWindingSign;

	UE_LOG(LogRopeWrap, VeryVerbose,
		TEXT("[%s] Wrapping axis re-seeded on bone transition: bone=%s, origin=%s, dir=%s, winding=%+.0f"),
		*Ctx.OwnerName, *Bone.ToString(),
		*State.PathAxisOrigin.ToString(), *State.PathAxisDirection.ToString(), NewWindingSign);
}

#pragma endregion
