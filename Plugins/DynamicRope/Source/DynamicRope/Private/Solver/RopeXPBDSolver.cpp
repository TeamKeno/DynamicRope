// Copyright Epic Games, Inc. All Rights Reserved.

#include "Solver/RopeXPBDSolver.h"
#include "DynamicRopeLog.h"
#include "Collision/RopeCollider.h"
// TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)
#include "ProfilingDebugging/CpuProfilerTrace.h"

void FRopeColliderCandidates::Reset(int32 NumNodes)
{
	const int32 NumSeg = FMath::Max(0, NumNodes - 1);
	bValid = false;

	// Reset clears the count while keeping the slack, and AddZeroed refills it, so the heap is not reclaimed
	// every substep. The index buffer needs no initialization, since slots past the count are never read.
	NodeBounds.SetNum(NumNodes, EAllowShrinking::No);
	NodeIndices.SetNumUninitialized(NumNodes * MaxPerItem, EAllowShrinking::No);
	NodeNum.Reset(NumNodes);
	NodeNum.AddZeroed(NumNodes);
	bNodeOverflow.Reset(NumNodes);
	bNodeOverflow.AddZeroed(NumNodes);

	SegIndices.SetNumUninitialized(NumSeg * MaxPerItem, EAllowShrinking::No);
	SegNum.Reset(NumSeg);
	SegNum.AddZeroed(NumSeg);
	bSegOverflow.Reset(NumSeg);
	bSegOverflow.AddZeroed(NumSeg);
}

void FRopeColliderCandidates::AddNode(int32 NodeIndex, int32 ColliderIndex)
{
	int32& Num = NodeNum[NodeIndex];
	if (Num >= MaxPerItem)
	{
		// More colliders overlap this node than the cap allows, so the candidate list is incomplete and this
		// node falls back to looping over everything. (Looking at only the first MaxPerItem would weaken
		// detection and let the rope penetrate — being slow here is the correct trade.)
		bNodeOverflow[NodeIndex] = true;
		return;
	}
	NodeIndices[NodeIndex * MaxPerItem + Num] = ColliderIndex;
	++Num;
}

void FRopeColliderCandidates::AddSegment(int32 SegIndex, int32 ColliderIndex)
{
	int32& Num = SegNum[SegIndex];
	if (Num >= MaxPerItem)
	{
		bSegOverflow[SegIndex] = true;
		return;
	}
	SegIndices[SegIndex * MaxPerItem + Num] = ColliderIndex;
	++Num;
}

int32 FRopeColliderCandidates::NodeCount(int32 NodeIndex, int32 NumColliders, bool& bOutAll) const
{
	if (!bValid || !bNodeOverflow.IsValidIndex(NodeIndex) || bNodeOverflow[NodeIndex])
	{
		bOutAll = true;
		return NumColliders;
	}
	bOutAll = false;
	return NodeNum[NodeIndex];
}

int32 FRopeColliderCandidates::SegCount(int32 SegIndex, int32 NumColliders, bool& bOutAll) const
{
	if (!bValid || !bSegOverflow.IsValidIndex(SegIndex) || bSegOverflow[SegIndex])
	{
		bOutAll = true;
		return NumColliders;
	}
	bOutAll = false;
	return SegNum[SegIndex];
}

FRopeSubstepSchedule RopeSolverSubsteps(FRopeSimState& State, const FRopeSolverConfig& Config, float DeltaSeconds)
{
	// Fixed timestep: the substep size is fixed regardless of frame rate, which is what "Substeps" means — the
	// number of substeps in one 60fps frame. Real elapsed time is accumulated and consumed in those fixed
	// units, so a low frame rate runs more substeps and a high one fewer. The displacement per substep is
	// therefore constant, and collision and tunnelling behaviour does not depend on the frame rate.
	const int32 SubPerRef = FMath::Clamp(Config.Substeps, 1, 16);
	const float FixedDt = (1.0f / 60.0f) / static_cast<float>(SubPerRef);
	// Spiral-of-death cap, which degrades to slow motion under overload. ×1.5 catches up in real time down to
	// 40fps and goes slow-motion below that. The previous ×2 (catching up to 30fps) could double the substeps
	// in a frame, so a frame already dropped by solve load asked for even more work next frame — positive
	// feedback, a spiral. Lowering the cap bounds the worst frame's solve cost to 1.5× the normal one. The
	// catch-up after a long pause (the MaxAccum clamp) follows the same cap.
	const int32 MaxSubsteps = FMath::Clamp((SubPerRef * 3) / 2, 1, 32);

	State.TimeAccumulator += DeltaSeconds;
	const float MaxAccum = FixedDt * static_cast<float>(MaxSubsteps);
	if (State.TimeAccumulator > MaxAccum)
	{
		// Discard the excess: mild slow motion instead of a runaway.
		State.TimeAccumulator = MaxAccum;
	}

	const int32 NumSub = FMath::FloorToInt(State.TimeAccumulator / FixedDt);
	if (NumSub <= 0)
	{
		// Not even one substep's worth has accumulated (high fps), so carry it into the next frame.
		return FRopeSubstepSchedule{ 0, FixedDt };
	}
	State.TimeAccumulator -= static_cast<float>(NumSub) * FixedDt;
	return FRopeSubstepSchedule{ NumSub, FixedDt };
}

void FRopeXPBDSolver::Step(FRopeSimState& State, const FRopeSolverConfig& Config,
	const TArray<IRopeCollider*>& Colliders, float DeltaSeconds,
	const FRopeKinematicTargetFrame* KinematicTargets) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSolver_Step);
	if (State.Num() < 2)
	{
		return;
	}

	const FRopeSubstepSchedule Schedule = RopeSolverSubsteps(State, Config, DeltaSeconds);
	const int32 NumSub = Schedule.NumSub;
	const float FixedDt = Schedule.FixedDt;
	const bool bHasKinematicTargets = KinematicTargets && KinematicTargets->IsValidFor(State.Num());
	TArray<float> PersistentInvMass;
	if (bHasKinematicTargets)
	{
		PersistentInvMass = State.InvMass;
	}
	const auto ApplyKinematicTargets = [&State, KinematicTargets, bHasKinematicTargets](
		float PreviousAlpha, float CurrentAlpha)
	{
		if (!bHasKinematicTargets)
		{
			return;
		}
		for (int32 NodeIndex = 0; NodeIndex < State.Num(); ++NodeIndex)
		{
			if (KinematicTargets->Mask[NodeIndex] == 0)
			{
				continue;
			}
			State.PrevPositions[NodeIndex] = FMath::Lerp(
				KinematicTargets->PrevTargets[NodeIndex],
				KinematicTargets->CurrentTargets[NodeIndex], PreviousAlpha);
			State.Positions[NodeIndex] = FMath::Lerp(
				KinematicTargets->PrevTargets[NodeIndex],
				KinematicTargets->CurrentTargets[NodeIndex], CurrentAlpha);
			State.InvMass[NodeIndex] = 0.0f;
		}
	};
	const auto RestoreKinematicMass = [&State, KinematicTargets, bHasKinematicTargets, &PersistentInvMass]()
	{
		if (!bHasKinematicTargets)
		{
			return;
		}
		for (int32 NodeIndex = 0; NodeIndex < State.Num(); ++NodeIndex)
		{
			if (KinematicTargets->Mask[NodeIndex] != 0)
			{
				State.InvMass[NodeIndex] = PersistentInvMass[NodeIndex];
			}
		}
	};
	if (NumSub <= 0)
	{
		ApplyKinematicTargets(0.0f, 1.0f);
		RestoreKinematicMass();
		return;
	}

	const int32 Iters = FMath::Max(1, Config.Iterations);

	// Hot path, so it is off by default at VeryVerbose. Enable it with "log LogRopeSolver VeryVerbose" in the console.
	UE_LOG(LogRopeSolver, VeryVerbose, TEXT("Step: %d node(s), %d substep(s) x %d iter(s), %d collider(s)"),
		State.Num(), NumSub, Iters, Colliders.Num());

	// Per-constraint Lagrange multipliers (XPBD). Reset each substep and accumulated across that substep's iterations.
	const int32 NumDist = State.Num() - 1;
	const int32 NumBend = FMath::Max(0, State.Num() - 2);
	TArray<float> LambdaDist;
	TArray<float> LambdaBend;
	LambdaDist.SetNumZeroed(NumDist);
	LambdaBend.SetNumZeroed(NumBend);

	// Per-node cached contact constraint: detected once per substep, enforced every iteration, with friction applied once at the end.
	TArray<FRopeContactState> Contacts;
	Contacts.SetNum(State.Num());

	// Per-node and per-segment collider candidates, refilled by each detect pass and reused by that pass's
	// iterations. Declared outside the substep loop so the buffer is allocated once.
	FRopeColliderCandidates Candidates;

	// Broad phase: compute each collider's world AABB (plus CollisionRadius) once. It turns SolveCollisions'
	// cost of inverse-transforming a query against a distant collider, per node per iteration per substep,
	// into a cheap box test.
	const float CollRadius = FMath::Max(0.0f, Config.CollisionRadius);
	TArray<FBox> ColliderBounds;
	ColliderBounds.Reserve(Colliders.Num());
	for (const IRopeCollider* Collider : Colliders)
	{
		ColliderBounds.Add(Collider ? Collider->GetWorldBounds().ExpandBy(CollRadius) : FBox(ForceInit));
	}

	for (int32 s = 0; s < NumSub; ++s)
	{
		const float SubAlpha0 = static_cast<float>(s) / static_cast<float>(NumSub);
		const float SubAlpha1 = static_cast<float>(s + 1) / static_cast<float>(NumSub);
		Integrate(State, Config, FixedDt);
		ApplyKinematicTargets(SubAlpha0, SubAlpha1);

	// Sweep the pinned start toward its target across this frame's substeps, which is what stops an anchor
	// jump exploding the chain. Velocity at the pin is zeroed so no motion is injected.
		if (State.bStartPinned && State.Num() > 0)
		{
			const float Alpha = static_cast<float>(s + 1) / static_cast<float>(NumSub);
			const FVector Pin = FMath::Lerp(State.StartPinPrev, State.StartPinTarget, Alpha);
			State.Positions[0] = Pin;
			State.PrevPositions[0] = Pin;
			State.InvMass[0] = 0.0f;
		}

		// XPBD accumulates lambda within a substep, so it is zeroed before this substep's iterations.
		for (float& L : LambdaDist) { L = 0.0f; }
		for (float& L : LambdaBend) { L = 0.0f; }
		for (FRopeContactState& C : Contacts) { C.bActive = false; C.Lambda = 0.0f; }

		// CollisionPassesPerSubstep is how many times contacts are *re-detected* within a substep, refreshing
		// the cached plane. 1, the default, detects once at the start. The swept detection is expensive so it
		// runs rarely, while SolveContacts enforces the cached plane cheaply every iteration — so even at K=1
		// collision competes with distance and bending on every iteration and is not overruled under tension,
		// which is what blocks penetration. Raise K above 1 when nodes move enough within a substep that the
		// plane goes stale and needs refreshing mid-substep.
		const int32 CollPasses = FMath::Clamp(Config.CollisionPassesPerSubstep, 1, Iters);
		// Contact resolve cadence. It counts backwards from the pass's last iteration, so the last one is always included (see below).
		const int32 ContactInterval = FMath::Max(1, Config.ContactSolveInterval);
		int32 ItDone = 0;
		for (int32 p = 0; p < CollPasses; ++p)
		{
			DetectContacts(State, Config, Colliders, ColliderBounds, SubAlpha0, SubAlpha1, Contacts, Candidates);
			// Cumulative target; the last pass is guaranteed to reach Iters.
			const int32 ItTarget = ((p + 1) * Iters) / CollPasses;
			for (; ItDone < ItTarget; ++ItDone)
			{
				// Alternate the sweep direction to cancel the Gauss-Seidel bias.
				const bool bReverse = (ItDone & 1) != 0;
				SolveDistance(State, Config, FixedDt, bReverse, LambdaDist);
				SolveBending(State, Config, FixedDt, bReverse, LambdaBend);

				// Solve contacts every ContactInterval iterations. The point is counting the cadence backwards
				// from the pass's *last* iteration: the remainder hits 0 there, so the last iteration is always
				// included whatever the interval. That structurally rules out a substep ending on a
				// distance-or-bending pull with no chance to push back out, which is how penetration happens.
				// An interval at or above the iterations per pass gives exactly one solve per pass — the same
				// cadence as the GPU kernel.
				if (((ItTarget - 1 - ItDone) % ContactInterval) == 0)
				{
					SolveContacts(State, Config, Colliders, ColliderBounds, Candidates, Contacts);
					SolveSegmentContacts(State, Config, Colliders, ColliderBounds, Candidates, bReverse);
				}
			}
		}

		// Friction runs once at the end of the substep, with the Coulomb limit set by the accumulated contact normal force (Lambda).
		ApplyContactFriction(State, Config, Contacts, FixedDt);

		// Strain limiting: at the end of the substep, confine any residual over-stretch the iterations could
		// not reach — the case of a long chain hanging off an anchor pin — under the cap, with a sequential
		// sweep that propagates the correction from the pinned node out to the free end.
		SolveStrainLimit(State, Config);
	}
	RestoreKinematicMass();

	// Tension, from the converged λ of the last substep: F = λ/h² in XPBD. Stretch means C > 0 and so λ < 0,
	// so only the positive part of -λ is tension and compression or slack reads 0. The units are relative to a
	// unit-mass node — see the FRopeSimState::SegmentTension comment.
	State.SegmentTension.SetNumUninitialized(NumDist);
	const float InvDt2 = 1.0f / (FixedDt * FixedDt);
	for (int32 k = 0; k < NumDist; ++k)
	{
		State.SegmentTension[k] = FMath::Max(0.0f, -LambdaDist[k]) * InvDt2;
	}
}

void FRopeXPBDSolver::SolveStrainLimit(FRopeSimState& State, const FRopeSolverConfig& Config) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSolver_StrainLimit);
	const float MaxRatio = Config.MaxStretchRatio;
	// 0 or below 1 disables it. 1.0 is fully inextensible.
	if (MaxRatio < 1.0f)
	{
		return;
	}
	const int32 Count = State.Num();
	const float MaxLen = MaxRatio * State.SegmentLength;
	if (Count < 2 || MaxLen <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	// Follow-the-leader clamp: when a segment exceeds MaxLen, *only the follower* is pulled toward the leader,
	// exactly to MaxLen, and the leader does not move. Moving both ends would disturb the segment just
	// aligned and one sweep would never converge; moving only the follower keeps the leader — an
	// already-placed node — fixed, which is what makes the sequential propagation hold. A pinned follower
	// (InvMass 0) cannot move and is skipped. Prev moves with the position so the correction is
	// velocity-neutral and nothing is flung.
	auto ClampToward = [&State, MaxLen](int32 Leader, int32 Follower)
	{
		if (State.InvMass[Follower] <= 0.0f)
		{
			return;
		}
		const FVector Delta = State.Positions[Follower] - State.Positions[Leader];
		const float Dist = Delta.Size();
		if (Dist <= MaxLen || Dist <= KINDA_SMALL_NUMBER)
		{
			return;
		}
		const FVector Target = State.Positions[Leader] + (Delta / Dist) * MaxLen;
		const FVector Corr = Target - State.Positions[Follower];
		State.Positions[Follower]     += Corr;
		State.PrevPositions[Follower] += Corr;
	};

	// Two sweeps: forward (node 0 → N-1, leader at the low index, propagating toward the hand with its pin or
	// anchor ahead) and backward (N-1 → 0, leader at the high index, propagating the span behind a wrap
	// anchor). Each direction moves only the follower, so one pass caps that direction's segments, and running
	// both handles a chain pinned at each end — the hand pin and the wrap anchor — converging where there is
	// slack and leaving the smallest residual where there is not.
	// The same order as the strain-limit stage in the GPU RopeXPBD.usf.
	const int32 Passes = 2;
	for (int32 p = 0; p < Passes; ++p)
	{
		for (int32 k = 0; k < Count - 1; ++k) { ClampToward(k, k + 1); }      // Forward: leader=k, follower=k+1
		for (int32 k = Count - 2; k >= 0; --k) { ClampToward(k + 1, k); }     // Rear: leader=k+1, follower=k
	}
}

void FRopeXPBDSolver::Integrate(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt) const
{
	// Damping is "the velocity loss per *frame* at 60fps". Applying it once per substep would stack it with
	// the substep count — 720 times a second at 12 substeps — multiplying the effective drag many times over,
	// dropping the rope's speed below 1 m/s and leaving it drifting like a ribbon. Taking the exponent by
	// substep size preserves the weight: the acceleration ramp and the momentum.
	const float Damp = FMath::Pow(1.0f - FMath::Clamp(Config.Damping, 0.0f, 1.0f), SubDt * 60.0f);
	const float Dt2 = SubDt * SubDt;
	// Bound the displacement per substep so the chain can never diverge.
	const float MaxStep = FMath::Max(State.SegmentLength * 2.0f, 1.0f);
	const float MaxStepSq = MaxStep * MaxStep;

	for (int32 i = 0; i < State.Num(); ++i)
	{
		if (State.InvMass[i] <= 0.0f)
		{
			continue;
		}
		FVector Velocity = (State.Positions[i] - State.PrevPositions[i]) * Damp;
		if (Velocity.SizeSquared() > MaxStepSq)
		{
			Velocity = Velocity.GetSafeNormal() * MaxStep;
		}
		const FVector NewPos = State.Positions[i] + Velocity + Config.Gravity * Dt2;
		State.PrevPositions[i] = State.Positions[i];
		State.Positions[i] = NewPos;
	}
}

void FRopeXPBDSolver::SolveDistance(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt, bool bReverse,
	TArray<float>& Lambda) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSolver_Distance);
	// XPBD distance constraint C = |x_{i+1} - x_i| - Solve L with a compliant Lagrange multiplier.
	// alpha_tilde = compliance / dt^2 (0 => rigid PBD). dLambda = (-C - alpha_tilde*Lambda) / (wA+wB+alpha_tilde).
	const float AlphaTilde = (SubDt > KINDA_SMALL_NUMBER) ? (Config.StretchCompliance / (SubDt * SubDt)) : 0.0f;
	const int32 Count = State.Num() - 1;
	for (int32 k = 0; k < Count; ++k)
	{
		const int32 i = bReverse ? (Count - 1 - k) : k;
		const float WA = State.InvMass[i];
		const float WB = State.InvMass[i + 1];
		const float WSum = WA + WB;
		if (WSum <= 0.0f)
		{
			continue;
		}

		const FVector Delta = State.Positions[i + 1] - State.Positions[i];
		const float Dist = Delta.Size();
		if (Dist <= KINDA_SMALL_NUMBER)
		{
			continue;
		}

		const FVector N = Delta / Dist;
		const float C = Dist - State.SegmentLength;
		const float DLambda = (-C - AlphaTilde * Lambda[i]) / (WSum + AlphaTilde);
		Lambda[i] += DLambda;

		// grad_i = -N, grad_{i+1} = +N.
		State.Positions[i]     -= N * (WA * DLambda);
		State.Positions[i + 1] += N * (WB * DLambda);
	}
}

void FRopeXPBDSolver::SolveBending(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt, bool bReverse,
	TArray<float>& Lambda) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSolver_Bending);
	// Support-stick bending: an XPBD distance constraint spanning i..i+2 with a rest length of 2 ×
	// SegmentLength. Straight gives C = 0; folded, the span shortens, so C < 0 and the constraint pushes the
	// two ends apart, straightening the rope, with the softness set by BendCompliance. For a 1D chain it is
	// cheap and stable.
	const int32 Count = State.Num() - 2;
	if (Count <= 0)
	{
		return;
	}
	const float AlphaTilde = (SubDt > KINDA_SMALL_NUMBER) ? (Config.BendCompliance / (SubDt * SubDt)) : 0.0f;
	const float Rest = 2.0f * State.SegmentLength;
	// Bend tolerance (matching the GPU RopeXPBD.usf): release the straightening force where the bend is sharp —
	// a corner, a wrap boundary — so the node can sit at an angle without being flung, and straighten only the
	// gentle bends. The measure is r = Dist/Rest = cos(half the turn angle). Full must exceed Release for the
	// smoothstep to be defined, so a floor is enforced and equal or inverted values stay safe.
	const float BendRelease = Config.BendReleaseRatio;
	const float BendFull = FMath::Max(Config.BendFullRatio, BendRelease + 1e-4f);
	for (int32 k = 0; k < Count; ++k)
	{
		const int32 i = bReverse ? (Count - 1 - k) : k;
		const float WA = State.InvMass[i];
		const float WB = State.InvMass[i + 2];
		const float WSum = WA + WB;
		if (WSum <= 0.0f)
		{
			continue;
		}

		const FVector Delta = State.Positions[i + 2] - State.Positions[i];
		const float Dist = Delta.Size();
		if (Dist <= KINDA_SMALL_NUMBER)
		{
			continue;
		}

		const FVector N = Delta / Dist;
		const float C = Dist - Rest;
		const float BendScale = FMath::SmoothStep(BendRelease, BendFull, Dist / Rest);
		const float DLambda = BendScale * (-C - AlphaTilde * Lambda[i]) / (WSum + AlphaTilde);
		Lambda[i] += DLambda;

		State.Positions[i]     -= N * (WA * DLambda);
		State.Positions[i + 2] += N * (WB * DLambda);
	}
}

void FRopeXPBDSolver::DetectContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
	const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds,
	float SubAlpha0, float SubAlpha1, TArray<FRopeContactState>& Contacts,
	FRopeColliderCandidates& Candidates) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSolver_Collisions);

	// Re-detection: clear only the previous pass's active flags. Lambda is reset at the start of the substep and keeps accumulating within it.
	for (FRopeContactState& C : Contacts) { C.bActive = false; }
	// Clear the candidates too, first, so no early-return path leaves the previous pass's list behind.
	Candidates.Reset(State.Num());
	if (Colliders.Num() == 0)
	{
		return;
	}

	// The rope's collision thickness. At 0 the node has to be inside the surface to register a hit, which is where most penetration on thin limbs and sparse nodes comes from.
	const float Radius = FMath::Max(0.0f, Config.CollisionRadius);
	const bool bHasBounds = ColliderBounds.Num() == Colliders.Num();

	// Swept (continuous) collision: the node is treated as the segment from PrevPos to Pos rather than a
	// point. A fast node crossing a thin surface within one substep would tunnel through a discrete test, so
	// this samples along that segment and stops at the first contact. A slow contact, where the segment is
	// short, comes to one sample — just the end point — and costs nothing extra.
	// The sample spacing (cm, designer-tunable) and the cap on samples per segment set the resolution.
	const float SweepStep = FMath::Max(Config.SweepStep, 0.1f);
	const int32 MaxSweepSamples = FMath::Max(1, Config.MaxSweepSamples);

	Candidates.bValid = bHasBounds;

	// Build each node's sweep AABB (Prev → Pos) once. The whole-rope AABB used for the per-collider cull, and
	// the per-segment span boxes, both derive from it. It is built from real node positions, so nothing can
	// slip through, and a bone that does not overlap the rope is skipped before the node loop or any pose
	// blending — nearly free when a rope is simply hanging.
	const int32 NumNodes = State.Num();
	FBox RopeBounds(ForceInit);
	for (int32 i = 0; i < NumNodes; ++i)
	{
		FBox& NodeBox = Candidates.NodeBounds[i];
		NodeBox = FBox(ForceInit);
		NodeBox += State.PrevPositions[i];
		NodeBox += State.Positions[i];
		RopeBounds += NodeBox;
	}

	// Margin (cm) for the candidate test. The iterations following this detect pass can pull a node through
	// the distance, bending and segment corrections, and the candidate set has to cover that range or a
	// collider is missed. One segment rest length is comfortably conservative — if nodes move further than
	// that within one collision pass the rope is already congested and the answer is re-detection on the next
	// substep. The floor is the radius or 1 cm, for the moment just after initialization when the rest length
	// is still 0.
	const float CandidateMargin = FMath::Max3(State.SegmentLength, Radius, 1.0f);

	// Did this node take an actual swept hit in this detection? Tracked separately so a hit overwrites the proximity watch below.
	TArray<bool> bHitThisDetect;
	bHitThisDetect.Init(false, NumNodes);

	// Outside the collider loop: compute this substep's sub-pose — blending a moving bone's prev → curr by
	// alpha — once per collider, hoisted out of the node loop so it is not recomputed per node. Where a node
	// touches several colliders the last hit overwrites the cache, since a node carries one contact plane and
	// pinching is handled separately. The sub-pose is stack-local, so a parallel solve stays safe.
	for (int32 c = 0; c < Colliders.Num(); ++c)
	{
		const IRopeCollider* Collider = Colliders[c];
		if (!Collider)
		{
			continue;
		}
		const FBox ColBounds = bHasBounds ? ColliderBounds[c] : FBox(ForceInit);

		// A collider that does not overlap this substep's rope AABB is skipped whole — no node loop, no pose blend.
		if (bHasBounds && !ColBounds.Intersect(RopeBounds))
		{
			continue;
		}

		// Compute this collider's sub-pose for this substep once. Only a moving bone is blended; a still one falls through to its single current pose for free.
		FRopeSweptQuery SQ;
		SQ.NodeRadius = Radius;
		SQ.SweepStep  = SweepStep;
		SQ.MaxSamples = MaxSweepSamples;
		// The alpha range is what a collider with no rigid transform — a capsule — uses to interpolate its own prev state.
		SQ.SubAlpha0  = SubAlpha0;
		SQ.SubAlpha1  = SubAlpha1;
		FTransform PrevX, CurrX;
		if (Collider->GetFrameMotion(PrevX, CurrX) && !PrevX.Equals(CurrX))
		{
			SQ.bUseSubPose = true;
			SQ.SubPoseStart.Blend(PrevX, CurrX, SubAlpha0);
			SQ.SubPoseEnd.Blend(PrevX, CurrX, SubAlpha1);
		}

		for (int32 i = 0; i < NumNodes; ++i)
		{
			if (State.InvMass[i] <= 0.0f)
			{
				continue;
			}

			// A is the substep's start position and B its end — read fresh each time, since an earlier collider may have pushed the node.
			const FVector A = State.PrevPositions[i];
			const FVector B = State.Positions[i];

			// Grow the node's box to include wherever the previous collider's push-out moved it. It only ever
			// grows, so the segment candidate test derived from this box stays conservative.
			Candidates.NodeBounds[i] += B;

			// Broad phase: skip when the node's span AABB does not overlap the collider's AABB expanded by
			// Radius. Testing the end point alone would miss a node that crossed straight through, so the test
			// has to be on the span. It is done twice over: the version widened by Margin registers candidates
			// for the following iterations, and the narrow one decides whether to fire a swept query now.
			if (bHasBounds)
			{
				FBox SweepBox(ForceInit);
				SweepBox += A;
				SweepBox += B;
				if (!ColBounds.Intersect(SweepBox.ExpandBy(CandidateMargin)))
				{
					continue;
				}
				Candidates.AddNode(i, c);
				if (!ColBounds.Intersect(SweepBox))
				{
					continue;
				}
			}

			SQ.WorldStart = A;
			SQ.WorldEnd   = B;

			FVector HitPos;
			const FRopeContact Contact = Collider->QuerySwept(SQ, HitPos);

			FRopeContactState& CC = Contacts[i];
			if (Contact.bHit)
			{
				// Continuous collision: push out of the surface at the first contact, which is what stops the
				// node crossing it, and record this collider as the contact — a hit overwrites the proximity
				// watch. SolveContacts then re-queries it fresh on every iteration.
				State.Positions[i] = HitPos + Contact.Normal * Contact.Penetration;
				CC.bActive        = true;
				CC.Normal         = Contact.Normal;
				CC.SurfaceVel     = Contact.SurfaceVelocity;
				bHitThisDetect[i] = true;
			}
			else if (!bHitThisDetect[i])
			{
				// No swept contact, but close enough in the broad phase, so register a watch only. If the
				// distance or bending constraints later pull the node into the surface during the iterations
				// — no hit, because it started outside — or the separation heuristic suppressed the sweep,
				// SolveContacts point-queries the real penetration each iteration and pushes it out, doing
				// nothing while the node is outside, since contact is one-sided. A collider already confirmed
				// as a hit is not overwritten.
				CC.bActive       = true;
			}
			// CC.Lambda is left alone: it is zeroed at the start of the substep and accumulates within it.
		}
	}

	// Segment candidates are built once detection has finished, against the final node boxes including every
	// push-out. They cannot be the union of the node candidates: a collider can cross the middle of a segment
	// without overlapping either end's box, and the union of two boxes is a strict subset of the box of their
	// union. SolveSegmentContacts' interior samples all lie between the two ends, so filtering by this span
	// box is conservative.
	if (bHasBounds)
	{
		for (int32 k = 0; k + 1 < NumNodes; ++k)
		{
			const FBox SegBox = (Candidates.NodeBounds[k] + Candidates.NodeBounds[k + 1]).ExpandBy(CandidateMargin);
			for (int32 c = 0; c < Colliders.Num(); ++c)
			{
				if (Colliders[c] && ColliderBounds[c].Intersect(SegBox))
				{
					Candidates.AddSegment(k, c);
				}
			}
		}
	}
}

void FRopeXPBDSolver::SolveContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
	const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds,
	const FRopeColliderCandidates& Candidates, TArray<FRopeContactState>& Contacts) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSolver_Contacts);
	const float Radius = FMath::Max(0.0f, Config.CollisionRadius);
	const bool bHasBounds = ColliderBounds.Num() == Colliders.Num();
	const int32 Count = State.Num();
	for (int32 i = 0; i < Count; ++i)
	{
		FRopeContactState& CC = Contacts[i];
		// Only nodes DetectContacts marked as near some collider.
		if (!CC.bActive)
		{
			continue;
		}
		const float W = State.InvMass[i];
		if (W <= 0.0f)
		{
			continue;
		}

		// Push the node out of the surface against *every* nearby collider, re-querying each one, so all
		// overlapping bones are defended at once. Trusting a single cached collider was the bug that let a
		// node slip past a second bone, which the colIdx ≠ cached check confirmed. Re-querying the real
		// surface rather than a cached plane is also what keeps it accurate on curved and concave surfaces,
		// and it matches the per-node collider loop in the GPU .usf.
		// DetectContacts already narrowed "nearby", so only those candidates are walked; a node over the cap
		// falls back to everything.
		const FVector& P = State.Positions[i];
		// Position just before the collision push-out, which the distance constraint sees. A pinched node is frozen back to it (see below).
		const FVector PrePos = State.Positions[i];
		// Pinch detection: the sum of unit normals and the count of colliders touching this node, mirroring NodeContact in the GPU RopeXPBD.usf.
		FVector ContactNormalSum = FVector::ZeroVector;
		int32 ContactCount = 0;
		bool bAllColliders = true;
		const int32 NumCand = Candidates.NodeCount(i, Colliders.Num(), bAllColliders);
		for (int32 n = 0; n < NumCand; ++n)
		{
			const int32 c = bAllColliders ? n : Candidates.NodeAt(i, n);
			const IRopeCollider* Collider = Colliders[c];
			if (!Collider)
			{
				continue;
			}
			// Broad phase: skip when the node point is outside the collider's world bounds expanded by Radius.
			// The candidate set was chosen with Margin, generous against movement during the iterations, so
			// this exact test is still needed within it.
			if (bHasBounds && !ColliderBounds[c].IsInsideOrOn(P))
			{
				continue;
			}
			const FRopeContact Contact = Collider->Query(P, Radius);
			if (!Contact.bHit)
			{
				// Outside this collider's surface, so no contact force — contact is one-sided.
				continue;
			}

			// XPBD rigid contact (compliance 0): C = -Penetration (< 0), so ΔLambda = Penetration/W with
			// Lambda clamped at ≥ 0. The position update of Normal × Penetration reprojects the node onto the
			// surface, and Lambda accumulates the per-node normal impulse that friction reads.
			const float DLambda = Contact.Penetration / W;
			const float NewLambda = FMath::Max(0.0f, CC.Lambda + DLambda);
			const float Applied = NewLambda - CC.Lambda;
			// Cache the latest contact's normal and surface velocity for friction.
			CC.Lambda  = NewLambda;
			CC.Normal  = Contact.Normal;
			CC.SurfaceVel = Contact.SurfaceVelocity;
			State.Positions[i] += Contact.Normal * (W * Applied);
			ContactNormalSum += Contact.Normal;
			++ContactCount;
		}

		// Pinch damping (mirroring the GPU RopeXPBD.usf): a node pressed simultaneously by colliders facing
		// each other — the unit normals cancel, so |sum| is far below the count — has nowhere to go. Leaving
		// it pushed onto whichever surface won last means the next substep's distance constraint pulls it back
		// in and it re-penetrates: the position ping-pongs between frames as jitter, with a tangential bounce.
		// So the push-out is discarded and the node is *frozen at its pre-collision position* (PrePos) —
		// there is no way out, so staying put is the best answer. It is order-independent and stable, and the
		// velocity is zeroed too (Prev = Pos = PrePos). The threshold of 0.6 × count fires only when two
		// normals are more than about 106° apart, meaning genuinely opposed, so a single surface or a gentle
		// corner is unaffected.
		if (ContactCount > 1 && ContactNormalSum.Size() < 0.6f * static_cast<float>(ContactCount))
		{
			State.Positions[i] = PrePos;
			State.PrevPositions[i] = PrePos;
		}
	}
}

void FRopeXPBDSolver::SolveSegmentContacts(FRopeSimState& State, const FRopeSolverConfig& Config,
	const TArray<IRopeCollider*>& Colliders, const TArray<FBox>& ColliderBounds,
	const FRopeColliderCandidates& Candidates, bool bReverse) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSolver_SegContacts);
	const float Radius = FMath::Max(0.0f, Config.CollisionRadius);
	const bool bHasBounds = ColliderBounds.Num() == Colliders.Num();
	// Sample spacing: the larger of the configured floor and the detection coverage limit, 2 × the node
	// radius. Each interior sample is a point query of that radius, so with adjacent probes — the two end
	// nodes' queries included — no further apart than 2R, every point on the chord lies within R of some
	// probe, and by the triangle inequality even a zero-thickness wall cannot slip through. The old fixed 2 cm
	// over-sampled a thick rope, with probe coverage overlapping. The same expression as the GPU
	// RopeSolveSegmentChord, for parity. The cap (MaxSweepSamples) is unchanged.
	const float SweepStep = FMath::Max(FMath::Max(Config.SweepStep, 0.1f), 2.0f * Radius);
	const int32 MaxSamples = FMath::Max(1, Config.MaxSweepSamples);
	const int32 Count = State.Num() - 1;
	for (int32 k = 0; k < Count; ++k)
	{
		const int32 i = bReverse ? (Count - 1 - k) : k;
		const float W0 = State.InvMass[i];
		const float W1 = State.InvMass[i + 1];
		if (W0 + W1 <= 0.0f)
		{
			// Both ends pinned, as in a wrapped span, so the segment cannot move. Skip.
			continue;
		}

		// This segment's collider candidates, chosen by DetectContacts from the segment span box. With none,
		// the sample loop is never entered — which is where most segments end.
		bool bAllColliders = true;
		const int32 NumCand = Candidates.SegCount(i, Colliders.Num(), bAllColliders);
		if (NumCand == 0)
		{
			continue;
		}

		const FVector P0 = State.Positions[i];
		const FVector P1 = State.Positions[i + 1];
		const float SegLen = static_cast<float>(FVector::Dist(P0, P1));
		// Interior sample count; the two end nodes were already handled by SolveContacts, so only the interior remains. A longer segment samples more densely.
		const int32 NumInner = FMath::Clamp(FMath::FloorToInt(SegLen / SweepStep), 1, MaxSamples);
		for (int32 s = 1; s <= NumInner; ++s)
		{
		// T is the interior parameter in (0, 1).
			const float T = static_cast<float>(s) / static_cast<float>(NumInner + 1);
		// Barycentric effective inverse mass: moving the interior point by delta moves the two ends by (1-T) and T of it.
			const float WEff = (1.0f - T) * (1.0f - T) * W0 + T * T * W1;
			if (WEff <= 0.0f)
			{
				continue;
			}
		// Recompute per sample from the current positions, since an earlier sample may already have moved an end node.
			const FVector Mid = FMath::Lerp(State.Positions[i], State.Positions[i + 1], T);
			for (int32 n = 0; n < NumCand; ++n)
			{
				const int32 c = bAllColliders ? n : Candidates.SegAt(i, n);
				const IRopeCollider* Collider = Colliders[c];
				if (!Collider)
				{
					continue;
				}
				if (bHasBounds && !ColliderBounds[c].IsInsideOrOn(Mid))
				{
					continue;
				}
				const FRopeContact Contact = Collider->Query(Mid, Radius);
				if (!Contact.bHit)
				{
					continue;
				}
				// The sample penetrated the surface, so distribute the correction barycentrically to the two
				// ends. The total move is ((1-T)² W0 + T² W1)/WEff × Pen = Pen, which puts the interior point
				// exactly on the surface.
				// The correction is velocity-neutral (mirroring the GPU segment collision): a chord against a
				// curved surface penetrates even at rest, so the correction never stops, and moving Positions
				// alone would inject outward velocity and make the node bounce off a stationary collider.
				// Moving PrevPositions with it fixes the position while preserving the velocity.
				const float DLambda = Contact.Penetration / WEff;
				const FVector D0 = Contact.Normal * ((1.0f - T) * W0 * DLambda);
				const FVector D1 = Contact.Normal * (T * W1 * DLambda);
				State.Positions[i]         += D0;
				State.PrevPositions[i]     += D0;
				State.Positions[i + 1]     += D1;
				State.PrevPositions[i + 1] += D1;
			}
		}
	}
}

void FRopeXPBDSolver::ApplyContactFriction(FRopeSimState& State, const FRopeSolverConfig& Config,
	const TArray<FRopeContactState>& Contacts, float SubDt) const
{
	const float Friction = FMath::Clamp(Config.Friction, 0.0f, 1.0f);
	if (Friction <= 0.0f)
	{
		return;
	}
	const int32 Count = State.Num();
	for (int32 i = 0; i < Count; ++i)
	{
		const FRopeContactState& CC = Contacts[i];
		if (!CC.bActive || CC.Lambda <= 0.0f)
		{
			continue;
		}
		const float W = State.InvMass[i];
		if (W <= 0.0f)
		{
			continue;
		}

		// The node's tangential displacement *relative to* the surface, so a moving surface sweeps the rope. Against a still surface (SurfaceVel 0) it is just the node's displacement.
		const FVector NodeDelta = State.Positions[i] - State.PrevPositions[i];
		const FVector SurfDelta = CC.SurfaceVel * SubDt;
		const FVector RelDelta = NodeDelta - SurfDelta;
		FVector RelTangent = RelDelta - (RelDelta | CC.Normal) * CC.Normal;

		// Free-end taper: the end node carries the least tension and grips too easily, so μ falls off toward it (frac 0 → 1 at the pinned end, 1 → TipFrictionScale at the tip).
		const float Frac = (Count > 1) ? (static_cast<float>(i) / static_cast<float>(Count - 1)) : 0.0f;
		const float MuEff = Friction * FMath::Lerp(1.0f, FMath::Clamp(Config.TipFrictionScale, 0.0f, 1.0f), Frac);

		// Coulomb limit: μ × the accumulated contact normal force (Lambda) × w. λ has units of cm × mass, and
		// multiplying by w makes MaxSlip come out in cm — no magic constant. Higher tension means a larger
		// Lambda, since the plane has to be pushed harder each iteration, and so a firmer grip. Past the limit
		// the rope slips, which is what stops it gripping forever.
		const float MaxSlip = MuEff * CC.Lambda * W;
		const float TLen = RelTangent.Size();
		if (TLen > MaxSlip && TLen > KINDA_SMALL_NUMBER)
		{
			RelTangent *= (MaxSlip / TLen);
		}
		State.PrevPositions[i] += RelTangent;
	}
}
