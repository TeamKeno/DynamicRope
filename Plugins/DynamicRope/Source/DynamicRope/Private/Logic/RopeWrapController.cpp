// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeWrapController.h"
#include "Collision/RopeCollider.h"
#include "Components/SkeletalMeshComponent.h"

bool FRopeWrapController::DecideWrap(const FRopeSimState& Sim, const TArray<IRopeCollider*>& Colliders,
	const FRopeWrapConfig& Config, float Dt, FRopeWrapState& OutSeed)
{
	if (Colliders.Num() == 0 || Sim.Num() == 0)
	{
		CandidateBone = NAME_None;
		CandidateTime = 0.0f;
		CandidateNodes.Reset();
		return false;
	}

	// Per-node nearest contact: which bone is this node touching (deepest penetration wins)?
	// Track the mesh that owns each contacted bone so the wrap can follow the right mesh later.
	TMap<FName, TArray<int32>>                      NodesByBone;
	TMap<FName, const USkeletalMeshComponent*>      MeshByBone;
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		FName                         BestBone = NAME_None;
		const USkeletalMeshComponent* BestMesh = nullptr;
		float                         BestPen = 0.0f;
		for (const IRopeCollider* Collider : Colliders)
		{
			if (!Collider)
			{
				continue;
			}
			const FRopeContact Contact = Collider->Query(Sim.Positions[i], Config.ContactRadius);
			if (Contact.bHit && Contact.Penetration > BestPen)
			{
				BestPen = Contact.Penetration;
				BestBone = Contact.Bone;
				BestMesh = Contact.SourceMesh;
			}
		}
		if (BestBone != NAME_None)
		{
			NodesByBone.FindOrAdd(BestBone).Add(i);
			MeshByBone.FindOrAdd(BestBone) = BestMesh;
		}
	}

	// Dominant bone = the one the most nodes are touching.
	FName DominantBone = NAME_None;
	const TArray<int32>* DominantNodes = nullptr;
	for (const TPair<FName, TArray<int32>>& Pair : NodesByBone)
	{
		if (!DominantNodes || Pair.Value.Num() > DominantNodes->Num())
		{
			DominantBone = Pair.Key;
			DominantNodes = &Pair.Value;
		}
	}

	const bool bEnoughContact = DominantNodes && DominantNodes->Num() >= Config.MinLatchNodes;
	if (!bEnoughContact)
	{
		CandidateBone = NAME_None;
		CandidateTime = 0.0f;
		CandidateNodes.Reset();
		return false;
	}

	// Accumulate sustained contact on the same bone; a bone switch restarts the timer.
	if (DominantBone == CandidateBone)
	{
		CandidateTime += Dt;
	}
	else
	{
		CandidateBone = DominantBone;
		CandidateTime = 0.0f;
	}
	CandidateNodes = *DominantNodes;

	if (CandidateTime < Config.WrapDecisionTime)
	{
		return false;
	}

	// Commit: seed the wrap with the contacting nodes (BoneLocalPos filled in BeginWrap).
	OutSeed.Reset();
	OutSeed.BoneName = CandidateBone;
	OutSeed.Mesh = MeshByBone.FindRef(CandidateBone);
	for (int32 NodeIndex : CandidateNodes)
	{
		FRopeLatchNode Latch;
		Latch.NodeIndex = NodeIndex;
		Latch.Bone = CandidateBone;
		OutSeed.Latched.Add(Latch);
	}

	CandidateBone = NAME_None;
	CandidateTime = 0.0f;
	CandidateNodes.Reset();
	return true;
}

void FRopeWrapController::BeginWrap(FRopeSimState& Sim, const FRopeWrapState& Seed, const USkeletalMeshComponent* FallbackMesh)
{
	State = Seed;
	State.TimeWrapped = 0.0f;

	// Prefer the mesh that owns the caught bone (carried in the seed); the rope-owner mesh is only
	// a fallback for the same-actor case. This is what lets the wrap follow a *different* actor.
	const USkeletalMeshComponent* Mesh = State.Mesh ? State.Mesh : FallbackMesh;
	State.Mesh = Mesh;
	if (!Mesh)
	{
		return;
	}

	// Convert each contact node's current world position into bone-local and freeze it (InvMass 0).
	// From here the node is driven by logic (the skinned bone), not the solver.
	FVector Centroid = FVector::ZeroVector;
	for (FRopeLatchNode& Latch : State.Latched)
	{
		if (!Sim.Positions.IsValidIndex(Latch.NodeIndex))
		{
			continue;
		}
		const FTransform BoneXform = Mesh->GetSocketTransform(Latch.Bone);
		Latch.BoneLocalPos = BoneXform.InverseTransformPosition(Sim.Positions[Latch.NodeIndex]);
		Sim.InvMass[Latch.NodeIndex] = 0.0f;
		Centroid += Sim.Positions[Latch.NodeIndex];
	}

	if (State.Latched.Num() > 0)
	{
		Centroid /= static_cast<double>(State.Latched.Num());
		// Rough metrics; refined in M3 (true wrap-angle integration).
		State.AnchorDistance = Sim.Num() > 0 ? FVector::Dist(Sim.Positions[0], Centroid) : 0.0f;
		State.WrapTurns = 0.0f;
	}
}

void FRopeWrapController::Hold(FRopeSimState& Sim, const USkeletalMeshComponent* FallbackMesh, float Dt)
{
	// Follow the mesh the bone was caught on; fall back to the rope-owner mesh (same-actor case).
	const USkeletalMeshComponent* Mesh = State.Mesh ? State.Mesh : FallbackMesh;
	if (!Mesh)
	{
		return;
	}

	// Re-place each latched node on its (animated) bone every frame so the wrap rides the skinning.
	// Zero velocity at the node (Prev = Pos) so the bone motion doesn't get injected into the solver.
	for (const FRopeLatchNode& Latch : State.Latched)
	{
		if (!Sim.Positions.IsValidIndex(Latch.NodeIndex))
		{
			continue;
		}
		const FTransform BoneXform = Mesh->GetSocketTransform(Latch.Bone);
		const FVector World = BoneXform.TransformPosition(Latch.BoneLocalPos);
		Sim.Positions[Latch.NodeIndex] = World;
		Sim.PrevPositions[Latch.NodeIndex] = World;
		Sim.InvMass[Latch.NodeIndex] = 0.0f;
	}

	State.TimeWrapped += Dt;
}

void FRopeWrapController::Pull(FRopeSimState& /*Sim*/, const FVector& /*PullTarget*/)
{
	// TODO(M3): procedural drag of the captured limb toward PullTarget, keep wrap taut.
}

void FRopeWrapController::Release(ERopeReleaseReason /*Reason*/)
{
	State.Reset();
	CandidateBone = NAME_None;
	CandidateTime = 0.0f;
	CandidateNodes.Reset();
}
