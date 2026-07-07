// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeWrapController.h"
#include "DynamicRopeLog.h"
#include "Collision/RopeCollider.h"
#include "Components/SkeletalMeshComponent.h"

namespace
{
	int32 FindHeadNodeIndex(const TArray<int32>& NodeIndices)
	{
		int32 HeadNodeIndex = INDEX_NONE;
		for (const int32 NodeIndex : NodeIndices)
		{
			if (NodeIndex == INDEX_NONE)
			{
				continue;
			}

			if (HeadNodeIndex == INDEX_NONE || NodeIndex < HeadNodeIndex)
			{
				HeadNodeIndex = NodeIndex;
			}
		}
		return HeadNodeIndex;
	}
}

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
	int32 DominantHeadNode = INDEX_NONE;
	for (const TPair<FName, TArray<int32>>& Pair : NodesByBone)
	{
		const int32 PairHeadNode = FindHeadNodeIndex(Pair.Value);
		if (!DominantNodes ||
			Pair.Value.Num() > DominantNodes->Num() ||
			(Pair.Value.Num() == DominantNodes->Num() &&
				(DominantHeadNode == INDEX_NONE || PairHeadNode < DominantHeadNode)))
		{
			DominantBone = Pair.Key;
			DominantNodes = &Pair.Value;
			DominantHeadNode = PairHeadNode;
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
	const int32 LatchNodeIndex = FindHeadNodeIndex(CandidateNodes);
	if (LatchNodeIndex != INDEX_NONE)
	{
		FRopeLatchNode Latch;
		Latch.NodeIndex = LatchNodeIndex;
		Latch.Bone = CandidateBone;
		OutSeed.Latched.Add(Latch);
	}

	UE_LOG(LogRopeWrap, Log, TEXT("DecideWrap committed: bone=%s, contact=%d node(s), latch=%d, dwell=%.3fs >= %.3fs"),
		*CandidateBone.ToString(), CandidateNodes.Num(), LatchNodeIndex, CandidateTime, Config.WrapDecisionTime);

	CandidateBone = NAME_None;
	CandidateTime = 0.0f;
	CandidateNodes.Reset();
	return true;
}

void FRopeWrapController::BeginWrap(const FRopeSimState& Sim, const FRopeWrapState& Seed, FRopeNodeOverrideFrame& OutFrame)
{
	State = Seed;
	State.TimeWrapped = 0.0f;

	// 붙잡힌 bone 을 소유한 mesh 는 시드에 실려 온다(접촉의 SourceMesh 에서 전파 — cross-actor 포함).
	// 없으면 잘못된 시드다: 아무것도 latch 하지 않고 상태를 비워 "감긴 척"하는 상태를 남기지 않는다.
	const USkeletalMeshComponent* Mesh = State.Mesh.Get();
	if (!Mesh)
	{
		UE_LOG(LogRopeWrap, Warning, TEXT("BeginWrap aborted: no mesh for bone %s (seed has no mesh) — nodes stay dynamic."),
			*State.BoneName.ToString());
		State.Reset();
		return;
	}

	UE_LOG(LogRopeWrap, Log, TEXT("BeginWrap: bone=%s, %d latched node(s), mesh=%s"),
		*State.BoneName.ToString(), State.Latched.Num(), *Mesh->GetName());

	// 각 접촉 노드의 현재 월드 위치를 bone-local 로 변환하여 동결한다(InvMass 0).
	// 이 시점부터 노드는 솔버가 아니라 logic(skinning 된 bone)에 의해 구동된다.
// Anchor가 없는 legacy seed면 Latched에서 임시 Anchor를 만든다.
	if (State.Anchors.Num() == 0)
	{
		for (const FRopeLatchNode& Latch : State.Latched)
		{
			if (!Sim.Positions.IsValidIndex(Latch.NodeIndex))
			{
				continue;
			}

			const FName Bone = Latch.Bone.IsNone() ? State.BoneName : Latch.Bone;
			const FTransform BoneXform = Mesh->GetSocketTransform(Bone);
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

		const USkeletalMeshComponent* AnchorMesh = Anchor.Mesh.Get();
		if (!AnchorMesh)
		{
			AnchorMesh = Mesh;
		}

		const FTransform BoneXform = AnchorMesh->GetSocketTransform(Anchor.Bone);

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
	// bone 이 붙잡힌 mesh 를 따라간다. State.Mesh 는 BeginWrap 에서 확정되어 weak 포인터로
	// 영속화된다(cross-actor 대상일 수 있다). 대상 액터가 파괴되면 weak 가 null 이 되어
	// raw 포인터 역참조(use-after-free) 없이 안전하게 감지된다 — 엉뚱한 bone 으로 노드를
	// 끌어당기지 않도록 폴백 없이 false 를 반환해 호출자가 release 하게 한다.
	const USkeletalMeshComponent* Mesh = State.Mesh.Get();
	if (!Mesh)
	{
		return false;
	}

	// 새 방식: surface anchor 기반 hold
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

			const USkeletalMeshComponent* AnchorMesh = Anchor.Mesh.Get();
			if (!AnchorMesh)
			{
				AnchorMesh = Mesh;
			}

			const FName Bone = Anchor.Bone.IsNone() ? State.BoneName : Anchor.Bone;
			const FTransform BoneXform = AnchorMesh->GetSocketTransform(Bone);

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

		State.TimeWrapped += Dt;
		return true;
	}

	// 기존 방식 fallback
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
		OutFrame.EnsureSize(Sim.Num());
		OutFrame.SetPosition(Latch.NodeIndex, World, /*bZeroVelocity*/ true);
		OutFrame.SetInvMass(Latch.NodeIndex, 0.0f);
	}

	State.TimeWrapped += Dt;
	return true;
}

bool FRopeWrapController::ComputePull(const FRopeSimState& Sim, float BendThresholdDeg, FRopePullSample& Out) const
{
	Out = FRopePullSample();
	if (!State.IsWrapped())
	{
		return false;
	}

	// 손 쪽 첫 앵커(최소 노드 인덱스): 손~앵커 사이 자유 구간의 장력이 여기로 전달된다.
	// 새 방식(Anchors) 우선, legacy(Latched) 폴백 — Hold와 동일한 우선순위.
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

	// 앵커가 노드 0(손 핀 자체)이면 손 쪽 세그먼트가 없다 → 당김 없음.
	if (AnchorNode <= 0)
	{
		return false;
	}

	// 당김 방향 = 앵커에서 손 쪽으로 로프를 따라 걸으며 찾은 "첫 직선 다리"의 끝 노드를 향하는 방향.
	// 각 스텝에서 다음 세그먼트가 지금까지의 누적 다리 방향(앵커→현재 조준노드)에서 임계 이상 꺾이면 멈춘다.
	// 곧으면 손(노드 0)까지 걸어가 정확히 chord가 되고, 벽/모서리에선 그 직전에 멈춰 첫 다리를 따른다.
	// 누적 방향 기준이라 한 노드의 처짐/지터로 조기 종료되지 않는다(공간 평균; 시간 지터는 호출자 EMA가 흡수).
	const float CosThresh = FMath::Cos(FMath::DegreesToRadians(FMath::Clamp(BendThresholdDeg, 1.0f, 179.0f)));
	int32 AimNode = AnchorNode - 1; // 최소 인접 노드 1개는 포함(손 쪽 첫 세그먼트).
	for (int32 j = AnchorNode - 2; j >= 0; --j)
	{
		const FVector LegSoFar = (Sim.Positions[AimNode] - Sim.Positions[AnchorNode]).GetSafeNormal(); // 누적 다리(안정)
		const FVector NextSeg  = (Sim.Positions[j] - Sim.Positions[AimNode]).GetSafeNormal();           // 다음 세그먼트
		if (LegSoFar.IsNearlyZero() || NextSeg.IsNearlyZero()
			|| FVector::DotProduct(NextSeg, LegSoFar) < CosThresh)
		{
			break; // 코너(또는 축퇴) — 직전 노드(AimNode)가 첫 다리의 끝.
		}
		AimNode = j;
	}
	const FVector Along = (Sim.Positions[AimNode] - Sim.Positions[AnchorNode]).GetSafeNormal();
	if (Along.IsNearlyZero())
	{
		return false; // 축퇴(조준 노드와 앵커 겹침) — 방향 정의 불가.
	}

	Out.bValid = true;
	Out.AnchorNode = AnchorNode;
	Out.AimNode = AimNode;
	Out.Bone = AnchorBone;
	Out.WorldPoint = Sim.Positions[AnchorNode];
	Out.Direction = Along;
	// 앵커-손 쪽 인접 세그먼트(인덱스 AnchorNode-1)의 장력. 아직 솔브 전이면(배열 비어 있음) 0.
	Out.Tension = Sim.SegmentTension.IsValidIndex(AnchorNode - 1) ? Sim.SegmentTension[AnchorNode - 1] : 0.0f;
	return true;
}

void FRopeWrapController::Release(ERopeReleaseReason Reason)
{
	UE_LOG(LogRopeWrap, Log, TEXT("Release: bone=%s, reason=%d"), *State.BoneName.ToString(), static_cast<int32>(Reason));
	State.Reset();
	CandidateBone = NAME_None;
	CandidateTime = 0.0f;
	CandidateNodes.Reset();
}
