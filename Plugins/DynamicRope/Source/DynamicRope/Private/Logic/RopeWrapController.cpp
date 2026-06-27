// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeWrapController.h"
#include "DynamicRopeLog.h"
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

	// 노드별 최근접 접촉: 이 노드는 어떤 bone 에 닿고 있는가(가장 깊은 침투가 우선)?
	// 나중에 wrap 이 올바른 mesh 를 따라갈 수 있도록 각 접촉 bone 을 소유한 mesh 를 추적한다.
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

	// dominant bone = 가장 많은 노드가 닿고 있는 bone.
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

	// 같은 bone 에 대한 지속 접촉을 누적한다. bone 이 바뀌면 타이머를 재시작한다.
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

	// 커밋: 접촉 중인 노드들로 wrap 을 시드한다(BoneLocalPos 는 BeginWrap 에서 채워진다).
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

	UE_LOG(LogRopeWrap, Log, TEXT("DecideWrap committed: bone=%s, %d node(s), dwell=%.3fs >= %.3fs"),
		*CandidateBone.ToString(), CandidateNodes.Num(), CandidateTime, Config.WrapDecisionTime);

	CandidateBone = NAME_None;
	CandidateTime = 0.0f;
	CandidateNodes.Reset();
	return true;
}

void FRopeWrapController::BeginWrap(FRopeSimState& Sim, const FRopeWrapState& Seed, const USkeletalMeshComponent* FallbackMesh)
{
	State = Seed;
	State.TimeWrapped = 0.0f;

	// 붙잡힌 bone 을 소유한 mesh 를 우선한다(시드에 실려 있다). rope-owner mesh 는 same-actor 케이스를
	// 위한 폴백일 뿐이다. 이것이 wrap 이 *다른* actor 를 따라갈 수 있게 해주는 부분이다.
	const USkeletalMeshComponent* Mesh = State.Mesh.Get() ? State.Mesh.Get() : FallbackMesh;
	State.Mesh = Mesh;
	if (!Mesh)
	{
		UE_LOG(LogRopeWrap, Warning, TEXT("BeginWrap aborted: no mesh for bone %s (seed/fallback both null) — nodes stay dynamic."),
			*State.BoneName.ToString());
		return;
	}

	UE_LOG(LogRopeWrap, Log, TEXT("BeginWrap: bone=%s, %d latched node(s), mesh=%s"),
		*State.BoneName.ToString(), State.Latched.Num(), *Mesh->GetName());

	// 각 접촉 노드의 현재 월드 위치를 bone-local 로 변환하여 동결한다(InvMass 0).
	// 이 시점부터 노드는 솔버가 아니라 logic(skinning 된 bone)에 의해 구동된다.
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
		// 대략적인 메트릭. M3 에서 정밀화된다(true wrap-angle 적분).
		State.AnchorDistance = Sim.Num() > 0 ? FVector::Dist(Sim.Positions[0], Centroid) : 0.0f;
		State.WrapTurns = 0.0f;
	}
}

bool FRopeWrapController::Hold(FRopeSimState& Sim, const USkeletalMeshComponent* FallbackMesh, float Dt)
{
	// bone 이 붙잡힌 mesh 를 따라간다. State.Mesh 는 BeginWrap 에서 해석되어 weak 포인터로
	// 영속화된다(cross-actor 대상일 수 있다). 대상 액터가 파괴되면 weak 가 null 이 되어
	// raw 포인터 역참조(use-after-free) 없이 안전하게 감지된다.
	const USkeletalMeshComponent* Mesh = State.Mesh.Get();
	if (!Mesh)
	{
		// State.Mesh 가 애초에 설정된 적 없는 same-actor 케이스만 owner mesh 로 폴백한다.
		// 이미 묶였던 mesh 가 파괴된 경우라면(IsExplicitlyNull == false) 엉뚱한 bone 으로
		// 노드를 끌어당기지 않도록 폴백하지 않고 false 를 반환해 호출자가 release 하게 한다.
		if (!State.Mesh.IsExplicitlyNull())
		{
			return false;
		}
		Mesh = FallbackMesh;
	}
	if (!Mesh)
	{
		return false;
	}

	// wrap 이 skinning 을 타고 가도록 매 프레임 각 latched 노드를 자신의 (애니메이션된) bone 위에 재배치한다.
	// bone 의 움직임이 솔버로 주입되지 않도록 노드의 속도를 0 으로 둔다(Prev = Pos).
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
	return true;
}

void FRopeWrapController::Pull(FRopeSimState& /*Sim*/, const FVector& /*PullTarget*/)
{
	// TODO(M3): 붙잡힌 limb 를 PullTarget 쪽으로 절차적으로 끌어당기고, wrap 을 팽팽하게 유지한다.
}

void FRopeWrapController::Release(ERopeReleaseReason Reason)
{
	UE_LOG(LogRopeWrap, Log, TEXT("Release: bone=%s, reason=%d"), *State.BoneName.ToString(), static_cast<int32>(Reason));
	State.Reset();
	CandidateBone = NAME_None;
	CandidateTime = 0.0f;
	CandidateNodes.Reset();
}
