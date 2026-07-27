// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeWrapController.h"
#include "Components/SceneComponent.h"
// FRopeBindingFrame and ResolveBindingWorld, the binding resolution seam.
#include "Core/RopeWrapTarget.h"
#include "DynamicRopeLog.h"

void FRopeWrapController::BeginWrap(const FRopeSimState& Sim, const FRopeWrapState& Seed, FRopeNodeOverrideFrame& OutFrame)
{
	State = Seed;

	// The mesh owning the caught bone arrives on the seed, propagated from the contact's source mesh,
	// which covers the cross-actor case.
	// Without one the seed is malformed: nothing is latched and the state is cleared, so no state is left
	// pretending to be wrapped.
	const USceneComponent* Mesh = State.Mesh.Get();
	if (!Mesh)
	{
		UE_LOG(LogRopeWrap, Warning, TEXT("BeginWrap aborted: no mesh for bone %s (seed has no mesh) — nodes stay dynamic."),
			*State.BoneName.ToString());
		State.Reset();
		return;
	}

	UE_LOG(LogRopeWrap, Log, TEXT("BeginWrap: bone=%s, %d latched node(s), mesh=%s"),
		*State.BoneName.ToString(), State.Latched.Num(), *Mesh->GetName());

	// Freeze each contacting node's current world position into bone-local space, with an inverse mass of
	// zero. From this point the nodes are driven by logic, meaning the skinned bone, rather than by the
	// solver.
// A legacy seed with no anchors builds temporary anchors from the latches.
	if (State.Anchors.Num() == 0)
	{
		for (const FRopeLatchNode& Latch : State.Latched)
		{
			if (!Sim.Positions.IsValidIndex(Latch.NodeIndex))
			{
				continue;
			}

			const FName Bone = Latch.Bone.IsNone() ? State.BoneName : Latch.Bone;
			const FTransform BoneXform = ResolveBindingWorld(Mesh, Bone);
			const FVector World = Sim.Positions[Latch.NodeIndex];

			FRopeSurfaceAnchor Anchor;
			Anchor.NodeIndex = Latch.NodeIndex;
			Anchor.Bone = Bone;
			Anchor.Mesh = Mesh;
			Anchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(World);
			Anchor.LocalNormal = FVector::UpVector;
			Anchor.LocalTangent = FVector::ForwardVector;
			Anchor.StartWorldPosition = World;
			Anchor.SurfaceOffset = 0.0f;
			Anchor.RopeDistance = static_cast<float>(Latch.NodeIndex) * Sim.SegmentLength;

			State.Anchors.Add(Anchor);
		}
	}


	int32 ValidAnchorCount = 0;

	for (FRopeSurfaceAnchor& Anchor : State.Anchors)
	{
		if (!Sim.Positions.IsValidIndex(Anchor.NodeIndex) ||
			!Sim.PrevPositions.IsValidIndex(Anchor.NodeIndex) ||
			!Sim.InvMass.IsValidIndex(Anchor.NodeIndex))
		{
			continue;
		}

		if (Anchor.Bone.IsNone())
		{
			Anchor.Bone = State.BoneName;
		}

		if (!Anchor.Mesh.IsValid())
		{
			Anchor.Mesh = Mesh;
		}

		const USceneComponent* AnchorComp = Anchor.Mesh.Get();
		if (!AnchorComp)
		{
			AnchorComp = Mesh;
		}

		// The binding seam: the target transform is resolved in one place, as a skinned socket on a
		// skeletal target and as the component transform on a static one.
		FRopeBindingFrame Binding;
		Binding.Component = AnchorComp;
		Binding.SocketOrBone = Anchor.Bone;
		const FTransform BoneXform = ResolveBindingWorld(Binding);

		const FVector SurfaceWorld =
			BoneXform.TransformPosition(Anchor.LocalSurfacePosition);

		const FVector NormalWorld =
			BoneXform.TransformVectorNoScale(Anchor.LocalNormal)
			.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);

		const FVector World =
			SurfaceWorld + NormalWorld * Anchor.SurfaceOffset;

		OutFrame.EnsureSize(Sim.Num());
		OutFrame.SetPosition(Anchor.NodeIndex, World, /*bZeroVelocity*/ true);
		OutFrame.SetInvMass(Anchor.NodeIndex, 0.0f);

		++ValidAnchorCount;
	}

	if (ValidAnchorCount == 0)
	{
		UE_LOG(LogRopeWrap, Warning, TEXT("BeginWrap aborted: no valid anchors for bone %s"),
			*State.BoneName.ToString());
		State.Reset();
		return;
	}

	UE_LOG(LogRopeWrap, Log, TEXT("BeginWrap: bone=%s, anchors=%d, mesh=%s"),
		*State.BoneName.ToString(), State.Anchors.Num(), *Mesh->GetName());
}

bool FRopeWrapController::Hold(const FRopeSimState& Sim, float Dt, FRopeNodeOverrideFrame& OutFrame)
{
	// The bone follows the mesh it was caught on. State.Mesh is established in BeginWrap and persisted as
	// a weak pointer, since the target may belong to another actor. If that actor is destroyed the weak
	// pointer becomes null, which is detected safely with no raw pointer dereference, and false is
	// returned with no fallback so the caller releases rather than dragging the nodes towards the wrong
	// bone.
	const USceneComponent* Mesh = State.Mesh.Get();
	if (!Mesh)
	{
		return false;
	}

	// The current mechanism: holding against surface anchors.
	if (State.Anchors.Num() > 0)
	{
		for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
		{
			if (!Sim.Positions.IsValidIndex(Anchor.NodeIndex) ||
				!Sim.PrevPositions.IsValidIndex(Anchor.NodeIndex) ||
				!Sim.InvMass.IsValidIndex(Anchor.NodeIndex))
			{
				continue;
			}

			const USceneComponent* AnchorComp = Anchor.Mesh.Get();
			if (!AnchorComp)
			{
				AnchorComp = Mesh;
			}

			const FName Bone = Anchor.Bone.IsNone() ? State.BoneName : Anchor.Bone;
			// The binding seam: following the target each frame is resolved in one place, as a skinned
			// socket on a skeletal target and as the component transform on a static one.
			FRopeBindingFrame Binding;
			Binding.Component = AnchorComp;
			Binding.SocketOrBone = Bone;
			const FTransform BoneXform = ResolveBindingWorld(Binding);

			const FVector SurfaceWorld =
				BoneXform.TransformPosition(Anchor.LocalSurfacePosition);

			const FVector NormalWorld =
				BoneXform.TransformVectorNoScale(Anchor.LocalNormal)
				.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);

			const FVector World =
				SurfaceWorld + NormalWorld * Anchor.SurfaceOffset;

			OutFrame.EnsureSize(Sim.Num());
			OutFrame.SetPosition(Anchor.NodeIndex, World, /*bZeroVelocity*/ true);
			OutFrame.SetInvMass(Anchor.NodeIndex, 0.0f);
		}

		return true;
	}

	// The legacy fallback.
	// Each latched node is replaced onto its own animated bone every frame so the wrap follows the
	// skinning. The node's velocity is held at zero, with the previous position equal to the current one,
	// so the bone's movement is not injected into the solver.
	for (const FRopeLatchNode& Latch : State.Latched)
	{
		if (!Sim.Positions.IsValidIndex(Latch.NodeIndex))
		{
			continue;
		}
		const FTransform BoneXform = ResolveBindingWorld(Mesh, Latch.Bone);
		const FVector World = BoneXform.TransformPosition(Latch.BoneLocalPos);
		OutFrame.EnsureSize(Sim.Num());
		OutFrame.SetPosition(Latch.NodeIndex, World, /*bZeroVelocity*/ true);
		OutFrame.SetInvMass(Latch.NodeIndex, 0.0f);
	}

	return true;
}

bool FRopeWrapController::ComputePull(const FRopeSimState& Sim, float BendThresholdDeg, FRopePullSample& Out) const
{
	Out = FRopePullSample();
	if (!State.IsWrapped())
	{
		return false;
	}

	// The first anchor on the hand side, meaning the lowest node index: the tension of the free span
	// between the hand and the anchor is delivered here.
	// Anchors take priority with the latches as a fallback, matching the priority Hold uses.
	int32 AnchorNode = INDEX_NONE;
	FName AnchorBone = NAME_None;
	for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
	{
		if (Sim.Positions.IsValidIndex(Anchor.NodeIndex)
			&& (AnchorNode == INDEX_NONE || Anchor.NodeIndex < AnchorNode))
		{
			AnchorNode = Anchor.NodeIndex;
			AnchorBone = Anchor.Bone.IsNone() ? State.BoneName : Anchor.Bone;
		}
	}
	if (AnchorNode == INDEX_NONE)
	{
		for (const FRopeLatchNode& Latch : State.Latched)
		{
			if (Sim.Positions.IsValidIndex(Latch.NodeIndex)
				&& (AnchorNode == INDEX_NONE || Latch.NodeIndex < AnchorNode))
			{
				AnchorNode = Latch.NodeIndex;
				AnchorBone = Latch.Bone.IsNone() ? State.BoneName : Latch.Bone;
			}
		}
	}

	// An anchor at node 0, which is the hand pin itself, leaves no segment on the hand side and therefore
	// no pull.
	if (AnchorNode <= 0)
	{
		return false;
	}

	// The pull direction points at the end node of the first straight leg, found by walking along the rope
	// from the anchor towards the hand.
	// At each step the walk stops if the next segment bends more than the threshold away from the leg
	// direction accumulated so far, which is the chord from the anchor to the current aim node. On a
	// straight rope it walks all the way to the hand at node 0 and gives exactly that chord, and at a wall
	// or an edge it stops just before and follows the first leg.
	// Comparing against the accumulated chord detects a 90 degree corner clearly while staying insensitive
	// to the sag of a single node.
	//
	// Starting the aim one segment from the anchor would make the first step's chord a single segment and
	// leave it vulnerable to node noise: even on a taut rope two adjacent segments could exceed the
	// threshold and terminate immediately, shortening the baseline to one segment and making the direction
	// angle jitter badly. Two adjacent segments are therefore always included to establish a baseline
	// before corner testing begins, or fewer if the hand is nearer. Residual jitter over time, and the
	// discrete hops, are absorbed by the caller's fractional smoothing.
	const float CosThresh = FMath::Cos(FMath::DegreesToRadians(FMath::Clamp(BendThresholdDeg, 1.0f, 179.0f)));
	const FVector AnchorPos = Sim.Positions[AnchorNode];
	// The walk proceeds leg by leg, from the anchor to the hand at node 0, breaking a leg at each corner.
	// The end of the first leg is the aim node, which gives the direction and the tether overshoot exactly
	// as before, while the sum of every leg's chord is the observation behind the whole-chain taut test:
	// sag makes a chord shorter than its rest length, and a taut rope caught on a corner keeps each leg's
	// chord close to its rest length and is still recognized as taut, so corners are not penalized.
	int32 LegStart = AnchorNode;
	// The first leg is seeded with two segments where possible, which avoids the single-segment noise of
	// the first step.
	int32 LegEnd = FMath::Max(AnchorNode - 2, 0);
	int32 AimNode = INDEX_NONE; // The end of the first leg, settled on the first pass below.
	float ChordSum = 0.0f;
	float ChordSumRaw = 0.0f; // The unclamped chord sum, which is the violation observation for the constraint tether; see FRopePullSample::PathChordLen.
	float MaxSag = 0.0f;
	while (true)
	{
		for (int32 j = LegEnd - 1; j >= 0; --j)
		{
			// The accumulated leg chord gives the long baseline, against which the next segment is compared.
			const FVector LegSoFar = (Sim.Positions[LegEnd] - Sim.Positions[LegStart]).GetSafeNormal();
			const FVector NextSeg  = (Sim.Positions[j] - Sim.Positions[LegEnd]).GetSafeNormal();
			if (LegSoFar.IsNearlyZero() || NextSeg.IsNearlyZero()
				|| FVector::DotProduct(NextSeg, LegSoFar) < CosThresh)
			{
			// A corner, or a degenerate step: the previous node ends this leg.
				break;
			}
			LegEnd = j;
		}
		if (AimNode == INDEX_NONE)
		{
			AimNode = LegEnd;
		}
		// Each leg's chord is clamped to that leg's rest length. If a moving anchor stretches a leg, so a
		// segment exceeds its rest length, an unclamped chord would exceed the rest length and mask slack
		// elsewhere; a stretch is evidence about that one leg, not about the rope being taut. Clamping
		// makes the chord sum's upper bound exactly the free-span rest length.
		const float LegChord = static_cast<float>((Sim.Positions[LegEnd] - Sim.Positions[LegStart]).Size());
		const float LegRest = static_cast<float>(LegStart - LegEnd) * Sim.SegmentLength;
		ChordSum += FMath::Min(LegChord, LegRest);
		ChordSumRaw += LegChord;
		// The largest perpendicular departure of a leg's interior nodes from its chord, in cm. It measures,
		// with linear sensitivity, the gentle catenary sag that the angular corner test cannot see. A taut
		// rope caught on a corner is straight within each leg and gives a small value.
		for (int32 k = LegEnd + 1; k < LegStart; ++k)
		{
			MaxSag = FMath::Max(MaxSag, static_cast<float>(
				FMath::PointDistToSegment(Sim.Positions[k], Sim.Positions[LegStart], Sim.Positions[LegEnd])));
		}
		if (LegEnd <= 0)
		{
			break;
		}
		// The next leg restarts at the corner node, seeded with one segment, since baseline noise barely
		// affects the chord sum.
		LegStart = LegEnd;
		LegEnd = LegEnd - 1;
	}
	const FVector Along = (Sim.Positions[AimNode] - AnchorPos).GetSafeNormal();
	if (Along.IsNearlyZero())
	{
		// Degenerate, where the aim node coincides with the anchor, so no direction is defined.
		return false;
	}

	Out.bValid = true;
	Out.AnchorNode = AnchorNode;
	Out.AimNode = AimNode;
	Out.Bone = AnchorBone;
	Out.WorldPoint = Sim.Positions[AnchorNode];
	Out.Direction = Along;
	// The tension of the segment adjacent to the anchor on the hand side. It is 0 before the first solve,
	// when the array is empty.
	Out.Tension = Sim.SegmentTension.IsValidIndex(AnchorNode - 1) ? Sim.SegmentTension[AnchorNode - 1] : 0.0f;
	// The whole-chain taut observations: the sum of the leg chords and the rest length of the free span,
	// consumed by the component's chain taut gate.
	Out.TautChordLen = ChordSum;
	Out.PathChordLen = ChordSumRaw;
	Out.FreeRestLen = static_cast<float>(AnchorNode) * Sim.SegmentLength;
	// The minimum segment tension across the free span. Being taut means the tension reaches the hand
	// across every segment, so any slack anywhere gives zero.
	// It discriminates the zigzag slack and partial stretches the chord sum geometry cannot see. Before the
	// first solve, where the array is too short, it is treated as zero.
	float MinT = TNumericLimits<float>::Max();
	for (int32 i = 0; i < AnchorNode; ++i)
	{
		MinT = FMath::Min(MinT, Sim.SegmentTension.IsValidIndex(i) ? Sim.SegmentTension[i] : 0.0f);
	}
	Out.MinFreeTension = (AnchorNode > 0) ? MinT : 0.0f;
	Out.MaxLegSag = MaxSag;
	return true;
}

void FRopeWrapController::Release(ERopeReleaseReason Reason)
{
	UE_LOG(LogRopeWrap, Log, TEXT("Release: bone=%s, reason=%d"), *State.BoneName.ToString(), static_cast<int32>(Reason));
	State.Reset();
}
