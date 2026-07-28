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

	// Clear only the count with Reset (maintain slack) + AddZeroed → The heap is not reclaimed at each substep.
	// There is no need to initialize the index buffer because it does not read out-of-count slots.
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
		// The collider overlapping with this node exceeds the cap → Since the candidate list is incomplete, the entire amount falls back to a loop.
		// (If you only look at the MaxPerItem items in front, detection is reduced and penetration occurs — it is true that it is slow here).
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
	// pinned timestep: Pinned the substep size regardless of the frame rate (Substeps = "number of substeps per 60fps frame")
	// interpretation). Actual elapsed time is accumulated and consumed in pinned size, so more substeps at low fps and fewer substeps at high fps.
	// Rotate the substep → The displacement per substep is always constant → Collision/tunneling does not depend on the frame rate.
	const int32 SubPerRef = FMath::Clamp(Config.Substeps, 1, 16);
	const float FixedDt = (1.0f / 60.0f) / static_cast<float>(SubPerRef);
	// spiral-of-death cap (slow-mo when overloaded). ×1.5 = Real-time catchup up to 40fps, slow-mo below that.
	// Previous ×2 (catch up to 30fps) doubles the per-frame substep at most, so the more the frame drops due to the solve load,
	// Positive feedback (spiral), which increases the load for the next frame, has been increased — the cap has been lowered to reduce the cost of solving the worst frame.
	// Bind to 1.5 times the normal state. The thrust (MaxAccum clamp) after the long stationary also follows the same cap.
	const int32 MaxSubsteps = FMath::Clamp((SubPerRef * 3) / 2, 1, 32);

	State.TimeAccumulator += DeltaSeconds;
	const float MaxAccum = FixedDt * static_cast<float>(MaxSubsteps);
	if (State.TimeAccumulator > MaxAccum)
	{
		// Discard excess: mild slow-mo instead of runaway.
		State.TimeAccumulator = MaxAccum;
	}

	const int32 NumSub = FMath::FloorToInt(State.TimeAccumulator / FixedDt);
	if (NumSub <= 0)
	{
		// One substep has not yet been completed (high fps) → Carried over to the next frame.
		return FRopeSubstepSchedule{ 0, FixedDt };
	}
	State.TimeAccumulator -= static_cast<float>(NumSub) * FixedDt;
	return FRopeSubstepSchedule{ NumSub, FixedDt };
}

void FRopeXPBDSolver::Step(FRopeSimState& State, const FRopeSolverConfig& Config,
	const TArray<IRopeCollider*>& Colliders, float DeltaSeconds) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSolver_Step);
	if (State.Num() < 2)
	{
		return;
	}

	const FRopeSubstepSchedule Schedule = RopeSolverSubsteps(State, Config, DeltaSeconds);
	const int32 NumSub = Schedule.NumSub;
	const float FixedDt = Schedule.FixedDt;
	if (NumSub <= 0)
	{
		return;
	}

	const int32 Iters = FMath::Max(1, Config.Iterations);

	// hot-path: default disabled(VeryVerbose). You have to upload it to the console "log LogRopeSolver VeryVerbose" to see it.
	UE_LOG(LogRopeSolver, VeryVerbose, TEXT("Step: %d node(s), %d substep(s) x %d iter(s), %d collider(s)"),
		State.Num(), NumSub, Iters, Colliders.Num());

	// Lagrange multiplier (XPBD) by constraint. It is reset for each substep and accumulated over the relevant iterations.
	const int32 NumDist = State.Num() - 1;
	const int32 NumBend = FMath::Max(0, State.Num() - 2);
	TArray<float> LambdaDist;
	TArray<float> LambdaBend;
	LambdaDist.SetNumZeroed(NumDist);
	LambdaBend.SetNumZeroed(NumBend);

	// per-node cached contact constraint (detection per substep → force per iteration → friction once at the end).
	TArray<FRopeContactState> Contacts;
	Contacts.SetNum(State.Num());

	// collider candidate by node/segment. It is refilled for each detect pass and the iterations of that pass are reused.
	// Declare the buffer here to capture it only once, outside the substep loop.
	FRopeColliderCandidates Candidates;

	// Broad-phase: Calculate world AABB (+CollisionRadius) for each collider only once. SolveCollisions
	// Cuts the cost of inversely converting queries to distant colliders at each node/iteration/substep into a cheap box test.
	const float CollRadius = FMath::Max(0.0f, Config.CollisionRadius);
	TArray<FBox> ColliderBounds;
	ColliderBounds.Reserve(Colliders.Num());
	for (const IRopeCollider* Collider : Colliders)
	{
		ColliderBounds.Add(Collider ? Collider->GetWorldBounds().ExpandBy(CollRadius) : FBox(ForceInit));
	}

	for (int32 s = 0; s < NumSub; ++s)
	{
		Integrate(State, Config, FixedDt);

		// Sweep the pinned starting point to the target throughout the substeps of this frame (prevent explosion when anchor jumping).
		// Set the velocity at the pin to 0 to avoid injecting motion.
		if (State.bStartPinned && State.Num() > 0)
		{
			const float Alpha = static_cast<float>(s + 1) / static_cast<float>(NumSub);
			const FVector Pin = FMath::Lerp(State.StartPinPrev, State.StartPinTarget, Alpha);
			State.Positions[0] = Pin;
			State.PrevPositions[0] = Pin;
			State.InvMass[0] = 0.0f;
		}

		// XPBD: Since lambda is accumulated within a substep, it is initialized to 0 before iteration of this substep.
		for (float& L : LambdaDist) { L = 0.0f; }
		for (float& L : LambdaBend) { L = 0.0f; }
		for (FRopeContactState& C : Contacts) { C.bActive = false; C.Lambda = 0.0f; }

		// Alpha: The collider motion section occupied by this substep (evenly distributing frame motion to substeps).
		const float SubAlpha0 = static_cast<float>(s) / static_cast<float>(NumSub);
		const float SubAlpha1 = static_cast<float>(s + 1) / static_cast<float>(NumSub);

		// CollisionPassesPerSubstep = Number of contact *redetection* per substep (cache plane updates). 1=detection once at startup (default).
		// detection(swept) is expensive, so only occasionally, while SolveContacts forces the cache plane to be cheap each iteration → even at K=1
		// Collision competes equally with each iteration distance/bending and is not overtaken by tension (blocking penetration). node substeps
		// If the plane becomes worn due to a lot of movement within it, it is intermediately updated to K>1.
		const int32 CollPasses = FMath::Clamp(Config.CollisionPassesPerSubstep, 1, Iters);
		// Contact resolution cycle. Since we count backwards from the last iteration of pass, the last is always included (see below).
		const int32 ContactInterval = FMath::Max(1, Config.ContactSolveInterval);
		int32 ItDone = 0;
		for (int32 p = 0; p < CollPasses; ++p)
		{
			DetectContacts(State, Config, Colliders, ColliderBounds, SubAlpha0, SubAlpha1, Contacts, Candidates);
			// Cumulative target (last pass guarantees Iters).
			const int32 ItTarget = ((p + 1) * Iters) / CollPasses;
			for (; ItDone < ItTarget; ++ItDone)
			{
				// Change sweep direction alternately to remove Gauss-Seidel bias.
				const bool bReverse = (ItDone & 1) != 0;
				SolveDistance(State, Config, FixedDt, bReverse, LambdaDist);
				SolveBending(State, Config, FixedDt, bReverse, LambdaBend);

				// contact per ContactInterval. The point is to count cycles backwards from the *last* iteration of pass —
				// Since the last is always included at the point where the remainder of the operation becomes 0, “distance/bending” is not guaranteed in any Interval.
				// Structurally blocks penetration where the substep ends without being able to push back after the last pull.
				// Interval >= Number of iterations per pass → Exactly once per pass = Same cadence as GPU kernel.
				if (((ItTarget - 1 - ItDone) % ContactInterval) == 0)
				{
					SolveContacts(State, Config, Colliders, ColliderBounds, Candidates, Contacts);
					SolveSegmentContacts(State, Config, Colliders, ColliderBounds, Candidates, bReverse);
				}
			}
		}

		// Friction occurs once at the end of the substep: the Coulomb limit is set by the accumulated contact normal force (Lambda).
		ApplyContactFriction(State, Config, Contacts, FixedDt);

		// Strain limiting: At the end of the substep, residual overextension (when a long chain hangs on the anchor pin) that was not controlled by iteration was removed.
		// Confined within the cap by sequential sweep (correction propagation from pinned node to Free end).
		SolveStrainLimit(State, Config);
	}

	// tension (convergence λ → force of last substep): F = λ/h² in XPBD. Stretch is C>0 → λ<0, so only the positive part of -λ
	// tension (compression/slack is 0). The unit is the relative force based on mass 1 node — see the comment FRopeSimState::SegmentTension.
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
	// 0 or <1 = disabled. 1.0 = completely unstretched.
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

	// FTL (Follow-The-Leader) oriented clamp: If the segment exceeds MaxLen, *only* the follower* is pulled toward the leader to reduce the length.
	// Set exactly to MaxLen (leader does not move). If you move both sides, the segment you aligned just before will be disturbed again.
	// Convergence is not possible with one sweep — If you only move the follower, sequential propagation is not possible because the leader (=already placed node) is immutable.
	// is preserved. If the follower is pinned (InvMass 0), it cannot be moved and is skipped. When moving the position, prev is also moved to neutralize the velocity (prevent fling).
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

	// Forward(node0→N-1, leader=low index: Chain propagation to the hand with the pin/anchor in front) + Backward(N-1→0, leader=high index:
	// wrap propagates the section behind the anchor) twice. Since each direction moves only the follower, the direction segment is capped with 1 pass.
	// , and process both ends pinned (hand pin, wrap anchor) in two directions (convergence if there is slack, minimum residual if not).
	// Same order as GPU RopeXPBD.usf strain-limit stage.
	const int32 Passes = 2;
	for (int32 p = 0; p < Passes; ++p)
	{
		for (int32 k = 0; k < Count - 1; ++k) { ClampToward(k, k + 1); }      // Forward: leader=k, follower=k+1
		for (int32 k = Count - 2; k >= 0; --k) { ClampToward(k + 1, k); }     // Rear: leader=k+1, follower=k
	}
}

void FRopeXPBDSolver::Integrate(FRopeSimState& State, const FRopeSolverConfig& Config, float SubDt) const
{
	// Damping = "Velocity reduction rate per *frame* based on 60fps". If you multiply each substep, damping is proportional to the number of substeps.
	// As it piles up (720 times per second in 12 substeps), the effective drag becomes several tens of times → The longitudinal velocity falls below 1m/s and the rope
	// It floats at a constant speed like a ribbon. The weight (acceleration ramp/momentum) is preserved by dividing by the exponent according to the size of the substep.
	const float Damp = FMath::Pow(1.0f - FMath::Clamp(Config.Damping, 0.0f, 1.0f), SubDt * 60.0f);
	const float Dt2 = SubDt * SubDt;
	// Limit the displacement per substep so that the chain never diverges.
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
	// Support-stick bending: XPBD distance constraint over section i..i+2, with rest length of 2*SegmentLength.
	// When straightened => C=0; When folded, the span becomes shorter => C<0 => constraint pushes both ends together
	// (spreading), works smoothly according to BendCompliance. It is inexpensive and non-static for 1D chains.
	const int32 Count = State.Num() - 2;
	if (Count <= 0)
	{
		return;
	}
	const float AlphaTilde = (SubDt > KINDA_SMALL_NUMBER) ? (Config.BendCompliance / (SubDt * SubDt)) : 0.0f;
	const float Rest = 2.0f * State.SegmentLength;
	// Angle-allowed bending (same as GPU RopeXPBD.usf): For sharp bends (corner/wrap boundary), release the straightening force so that the node becomes angled.
	// Prevent splashing and straighten only gentle bends. Check with r=Dist/Rest=cos (turn angle/2). Full must be greater than Release
	// Since smoothstep is established, floor is enforced (safe even if the two values are the same or reversed).
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

	// Re-detection: Clear only the active flag of the previous pass (Lambda is reset only at the start of the substep and maintains accumulation within the substep).
	for (FRopeContactState& C : Contacts) { C.bActive = false; }
	// candidates are also cleared (first so that the previous pass list is not left in the early return path).
	Candidates.Reset(State.Num());
	if (Colliders.Num() == 0)
	{
		return;
	}

	// The collision thickness of the rope. If 0, the node must be inside the surface to hit → Most penetration occurs in thin limbs/sparse nodes.
	const float Radius = FMath::Max(0.0f, Config.CollisionRadius);
	const bool bHasBounds = ColliderBounds.Num() == Colliders.Num();

	// Swept (continuous) collision: The node is boned in the PrevPos->Pos section, not as a point. A fast node is thin in one substep.
	// Even if it traverses the surface (tunneling for discrete inspectors), it samples along the section and stops at the first contact. Slow contact (L small) is
	// 1 sample = Since only the endpoint is checked, there is no additional cost.
	// Sample interval (cm, designer tuning) and sample cap per section.
	const float SweepStep = FMath::Max(Config.SweepStep, 0.1f);
	const int32 MaxSweepSamples = FMath::Max(1, Config.MaxSweepSamples);

	Candidates.bValid = bHasBounds;

	// Create a per-node sweep AABB (Prev→Pos) once. The entire rope AABB (for one collider curl) and segment section
	// boxes are all derived from here. Since it is based on the actual node location, there is no risk of penetration, and bones that do not overlap the rope are nodes.
	// Skip it entirely before entering the loop/blend (almost Free if you hang on to it).
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

	// Allowance (cm) to give to candidate check. The iterations following this detect pass are distance/bending/segment corrections.
	// The range where the node can be pulled must be covered so that the collider is not missed in the candidate. Segment rest length is enough
	// Conservative — If more nodes are relocated within one collision pass, it is already congested and the next substep
	// Re-detection is the answer. Set the floor to Radius/1cm for immediately after initialization when the rest length is still 0.
	const float CandidateMargin = FMath::Max3(State.SegmentLength, Radius, 1.0f);

	// Did the node actually receive a swept hit in this detection? Differentiate so that hit overwrites proximity-watch (below).
	TArray<bool> bHitThisDetect;
	bHitThisDetect.Init(false, NumNodes);

	// collider-outer: Calculate substep sub-pose (blend prev->curr of moving bone with alpha) once per collider.
	// Hoists out of the node loop (prevents Blend recalculation for each node). When a node touches multiple colliders, the last hit is
	// Overwrite the cache (single contact plane per node — pinching the contact simplifies it). The sub-pose is stack local, so parallel solve is safe.
	for (int32 c = 0; c < Colliders.Num(); ++c)
	{
		const IRopeCollider* Collider = Colliders[c];
		if (!Collider)
		{
			continue;
		}
		const FBox ColBounds = bHasBounds ? ColliderBounds[c] : FBox(ForceInit);

		// Colliders that do not overlap with this substep rope AABB are skipped entirely (do not even enter the node loop/Blend).
		if (bHasBounds && !ColBounds.Intersect(RopeBounds))
		{
			continue;
		}

		// Calculate the sub-pose of this substep once for this collider. Blend only moving bones (Free fallback to single current pose if stationary).
		FRopeSweptQuery SQ;
		SQ.NodeRadius = Radius;
		SQ.SweepStep  = SweepStep;
		SQ.MaxSamples = MaxSweepSamples;
		// The alpha section is used by the transform-Free collider (capsule) to interpolate its prev state.
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

			// A = substep start position, B = end point (current value each time, as it may have been pushed by a previous collider).
			const FVector A = State.PrevPositions[i];
			const FVector B = State.Positions[i];

			// Accumulates in the node box up to the position moved by the push-out of the previous collider. Because it only grows bigger, in this box
			// The resulting segment candidate check continues to be conservative.
			Candidates.NodeBounds[i] += B;

			// Broad-phase: Skip if node section AABB does not overlap collider AABB(+Radius). Just look at the end point and cross it
			// Since the node that passed through is missed, it must be judged as section AABB. The check is two-fold — the side widened by Margin is
			// For registering candidates to be used in subsequent iterations, the narrow side is for whether to fire a swept query now.
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
				// CCD: Immediately pushes off the surface at the first contact (prevents passing across it), and confirms this collider as the contact.
				// (hit overwrites proximity-watch). Afterwards, SolveContacts is forced to use fresh material every iteration.
				State.Positions[i] = HitPos + Contact.Normal * Contact.Penetration;
				CC.bActive        = true;
				CC.Normal         = Contact.Normal;
				CC.SurfaceVel     = Contact.SurfaceVelocity;
				bHitThisDetect[i] = true;
			}
			else if (!bHitThisDetect[i])
			{
				// Swept uncontact, but broad-phase proximity → only registers watch. Distance/bending during iteration
				// Even if a node is brought to the surface (it is not a hit because it is initially outside) or the separation heuristic suppresses swept,
				// SolveContacts directly checks the actual penetration with each iteration point-query and pushes it out (no action if outside —
				// one side contact). Colliders that have already been confirmed as hits are not overwritten.
				CC.bActive       = true;
			}
			// CC.Lambda is left as is (it is reset to 0 and accumulated only at the start of the substep).
		}
	}

	// The segment candidate is created at once after detection is completed (to the final node box reflected until push-out).
	// cannot be replaced by the union of node candidates — a node that crosses the middle of the segment without overlapping either end box.
	// Because there is a collider (union of boxes ⊊ box of the union). All internal samples of SolveSegmentContacts have two ends.
	// , filtering through this section box is conservative.
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
		// Only nodes where DetectContacts indicates “This node is close to some collider.”
		if (!CC.bActive)
		{
			continue;
		}
		const float W = State.InvMass[i];
		if (W <= 0.0f)
		{
			continue;
		}

		// *All* colliders close to the node are pushed out of the surface using the material (preventing all overlapping bone multiple contacts).
		// Fixed bug where only one cached collider was visible (preventing penetration of other bones, confirmed by colIdx≠cached) —
		// Accurate even on curved/concave surfaces because it looks at the actual surface every time, not the cache plane. Matches the pre-collider loop per node in GPU .usf.
		// However, since DetectContacts has already covered the “close” range, only that candidate runs (only nodes exceeding the cap fallback entirely).
		const FVector& P = State.Positions[i];
		// Position just before collision push-out (reflected to distance constraint). When pinching, return here to freeze the node (see below).
		const FVector PrePos = State.Positions[i];
		// pinch detection: Unit normal sum/number of colliders touched by this node (GPU RopeXPBD.usf NodeContact mirror).
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
			// broad-phase: Skip if there is no node point in the collider world bounds (expanded to +Radius). Candidate is as much as Margin
			// has been sufficiently selected (relative to movement during iteration), this precise check is still necessary within the candidate.
			if (bHasBounds && !ColliderBounds[c].IsInsideOrOn(P))
			{
				continue;
			}
			const FRopeContact Contact = Collider->Query(P, Radius);
			if (!Contact.bHit)
			{
				// Outside this collider surface → No contact force (one-sided contact).
				continue;
			}

			// XPBD rigid contact(compliance 0): C = -Penetration(<0). ΔLambda = Penetration/W. Lambda clamps >=0.
			// Position update = Normal*Penetration: Reproject the node to the surface. Lambda per-node accumulated normal impulse (for friction).
			const float DLambda = Contact.Penetration / W;
			const float NewLambda = FMath::Max(0.0f, CC.Lambda + DLambda);
			const float Applied = NewLambda - CC.Lambda;
			// Cache the latest (last contact) normal/surface velocity for friction.
			CC.Lambda  = NewLambda;
			CC.Normal  = Contact.Normal;
			CC.SurfaceVel = Contact.SurfaceVelocity;
			State.Positions[i] += Contact.Normal * (W * Applied);
			ContactNormalSum += Contact.Normal;
			++ContactCount;
		}

		// Pinch damping (GPU RopeXPBD.usf mirror): Nodes pressed simultaneously on colliders facing each other (unit normal sum
		// offset = |sum| << count) has no place to escape. If you leave it pushed on one surface with last-wins push-out,
		// The next substep distance constraint is pulled again and re-penetrated → Position round trip (jitter)/tangential bounce between frames. So to the surface
		// Discard the result and *freeze it at the position just before the collision (PrePos)* — There is no escape, so staying in place is the best.
		// Order is irrelevant. Not static. The velocity is also 0 (gPrev=gPos=PrePos). threshold 0.6*number = two normals separated by more than ~106°
		// only ignites (when facing each other) — has no effect on single surfaces or gentle corners.
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
	// Sample interval: Larger of config floor vs detection coverage limit (2×node radius). The internal sample is a Radius point query.
	// If adjacent probes (including both end node queries) spacing ≤ 2×R, then any point on the chord is within R of the probe — of thickness 0
	// You can't even get through the wall (triangle inequality). The pinned 2cm was an overcrowded probe with overlapping coverage on the thick rope.
	// Same expression (parity) as GPU RopeSolveSegmentChord. cap(MaxSweepSamples) is the same as before.
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
			// Pins at both ends (wrap section) → The segment cannot be moved. Skip.
			continue;
		}

		// Collider candidate for this segment (DetectContacts selects segment interval box). If there is no candidate, sample
		// Skips the loop entry itself — usually most segments end here.
		bool bAllColliders = true;
		const int32 NumCand = Candidates.SegCount(i, Colliders.Num(), bAllColliders);
		if (NumCand == 0)
		{
			continue;
		}

		const FVector P0 = State.Positions[i];
		const FVector P1 = State.Positions[i + 1];
		const float SegLen = static_cast<float>(FVector::Dist(P0, P1));
		// Number of internal samples (both end nodes have already been processed by SolveContacts → internal only). The longer the segment, the denser it is.
		const int32 NumInner = FMath::Clamp(FMath::FloorToInt(SegLen / SweepStep), 1, MaxSamples);
		for (int32 s = 1; s <= NumInner; ++s)
		{
			// T is (0,1) internal parameter.
			const float T = static_cast<float>(s) / static_cast<float>(NumInner + 1);
			// barycentric effective inverse mass: To push the inner point by delta, move the two ends at the rate (1-T),(T).
			const float WEff = (1.0f - T) * (1.0f - T) * W0 + T * T * W1;
			if (WEff <= 0.0f)
			{
				continue;
			}
			// Recalculate each sample with the current position (the previous sample may have already moved the end node).
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
				// Penetration of sample points outside the surface: Distribute correction barycentrically to both ends.
				// Move sum = ((1-T)^2 W0 + T^2 W1)/WEff * Pen = Pen → The inner point is exactly on the surface.
				// Velocity neutral correction (GPU segment collision mirror): Chords always penetrate even at rest on a curved surface, so correction is not possible.
				// Continues to occur → If you just push the Positions, the external velocity is injected and the node bounces even on the stationary collider.
				// PrevPositions are also moved to fix only the position and preserve the velocity.
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

		// *Relative* tangential displacement of the node and the surface (the moving surface sweeps the rope). If it is a stationary surface (SurfaceVel 0), only node displacement.
		const FVector NodeDelta = State.Positions[i] - State.PrevPositions[i];
		const FVector SurfDelta = CC.SurfaceVel * SubDt;
		const FVector RelDelta = NodeDelta - SurfDelta;
		FVector RelTangent = RelDelta - (RelDelta | CC.Normal) * CC.Normal;

		// Free end taper: The end node has the lowest tension and is easily held, so μ is lowered (pinned point frac=0→1, end frac=1→TipFrictionScale).
		const float Frac = (Count > 1) ? (static_cast<float>(i) / static_cast<float>(Count - 1)) : 0.0f;
		const float MuEff = Friction * FMath::Lerp(1.0f, FMath::Clamp(Config.TipFrictionScale, 0.0f, 1.0f), Frac);

		// Coulomb limit: μ·(accumulated contact normal force Lambda)·w. λ is displacement in cm·mass, ×w → MaxSlip(cm) homogeneous (no magic constant).
		// The greater the tension, the Lambda↑ (since the plane must be pushed harder each iteration) → Grip↑. Excess grip slips (prevents permanent grip).
		const float MaxSlip = MuEff * CC.Lambda * W;
		const float TLen = RelTangent.Size();
		if (TLen > MaxSlip && TLen > KINDA_SMALL_NUMBER)
		{
			RelTangent *= (MaxSlip / TLen);
		}
		State.PrevPositions[i] += RelTangent;
	}
}
