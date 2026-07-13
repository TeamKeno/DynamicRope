// Copyright Epic Games, Inc. All Rights Reserved.
//
// RopeTypes.h 데이터 타입들의 비-인라인 멤버함수 모음. 헤더는 타입 정의(대부분 POD)에 집중하고,
// 엔진 컴포넌트 include가 필요하거나 분량이 있는 로직은 여기로 모은다.

#include "Core/RopeTypes.h"

#include "Camera/CameraComponent.h"
#include "Components/SceneComponent.h"
#include "GameFramework/Actor.h"

FRopeThrowContext FRopeThrowContext::MakeDefault(const USceneComponent& RopeComponent, const FRopeThrowParams& Params)
{
	FRopeThrowContext Context;
	Context.Origin = RopeComponent.GetComponentLocation();
	if (const AActor* Owner = RopeComponent.GetOwner())
	{
		Context.OwnerVelocity = Owner->GetVelocity();
		// 기본 진입점은 소켓 속도를 따로 측정하지 못한다 — owner 속도로 근사(Wielder 경로는 직접 채움).
		Context.SocketVelocity = Context.OwnerVelocity;
	}

	// 설정 패스스루.
	Context.FrameMode = Params.FrameMode;
	Context.ThrowSpeed = Params.ThrowSpeed;
	Context.SwingPlane = Params.SwingPlane;
	Context.CustomSwingPlaneNormal = Params.CustomSwingPlaneNormal;

	// 프레임 기저: FrameMode당 한 곳에서 세 축을 함께 확정한다. (이전 구현은 컴포넌트 기저를 먼저
	// 넣고 모드별 if가 일부 축만 덮어써 — World의 Up만 삼항으로 따로 정해지는 식 — 순서 의존이 있었다.)
	// 여기서는 원값만 조립하고, 정규화/직교 폴백은 ResolveThrowContext가 담당한다(책임 분리).
	auto SetComponentBasis = [&Context, &RopeComponent]()
	{
		Context.FrameForward = RopeComponent.GetForwardVector();
		Context.FrameUp = RopeComponent.GetUpVector();
		Context.FrameRight = RopeComponent.GetRightVector();
	};
	switch (Params.FrameMode)
	{
	case ERopeThrowFrameMode::World:
		Context.FrameForward = FVector::ForwardVector;
		Context.FrameUp = FVector::UpVector;
		Context.FrameRight = FVector::RightVector;
		break;

	case ERopeThrowFrameMode::OwnerCamera:
	{
		const AActor* Owner = RopeComponent.GetOwner();
		const UCameraComponent* Camera = Owner ? Owner->FindComponentByClass<UCameraComponent>() : nullptr;
		if (Camera)
		{
			Context.FrameForward = Camera->GetForwardVector();
			Context.FrameUp = Camera->GetUpVector();
			Context.FrameRight = Camera->GetRightVector();
		}
		else
		{
			// 카메라 없는 owner — 컴포넌트 기저 폴백(기존 동작 유지).
			SetComponentBasis();
		}
		break;
	}

	case ERopeThrowFrameMode::Custom:
		Context.FrameForward = Params.CustomFrameForward;
		Context.FrameUp = Params.CustomFrameUp;
		Context.FrameRight = Params.CustomFrameRight;
		break;

	case ERopeThrowFrameMode::Owner:
	case ERopeThrowFrameMode::Socket:
	default:
		SetComponentBasis();
		break;
	}
	return Context;
}

FRopeCaptureTravelFrame FRopeCaptureTravelFrame::Compute(const FRopeSimState& Sim,
	const TArray<FRopeContactCandidate>& Candidates, float DeltaTime)
{
	FRopeCaptureTravelFrame Frame;

	// 유효 후보 수집: 표면점 평균(RegionCenter), 접촉 노드 범위(span), 노드별 속도 평균.
	// 같은 노드가 여러 콜라이더에 잡혀 후보가 중복될 수 있으므로 속도 평균은 노드 단위로 센다.
	FVector CenterSum = FVector::ZeroVector;
	int32 CenterCount = 0;
	FVector VelocitySum = FVector::ZeroVector;
	int32 VelocityCount = 0;
	int32 MinNode = TNumericLimits<int32>::Max();
	int32 MaxNode = INDEX_NONE;
	TArray<int32, TInlineAllocator<32>> CountedNodes;

	for (const FRopeContactCandidate& Candidate : Candidates)
	{
		if (!Candidate.bValid || !Sim.Positions.IsValidIndex(Candidate.NodeIndex))
		{
			continue;
		}

		CenterSum += Candidate.WorldPoint;
		++CenterCount;
		MinNode = FMath::Min(MinNode, Candidate.NodeIndex);
		MaxNode = FMath::Max(MaxNode, Candidate.NodeIndex);

		if (DeltaTime > KINDA_SMALL_NUMBER &&
			Sim.PrevPositions.IsValidIndex(Candidate.NodeIndex) &&
			!CountedNodes.Contains(Candidate.NodeIndex))
		{
			CountedNodes.Add(Candidate.NodeIndex);
			VelocitySum += (Sim.Positions[Candidate.NodeIndex] - Sim.PrevPositions[Candidate.NodeIndex]) / DeltaTime;
			++VelocityCount;
		}
	}

	if (CenterCount == 0)
	{
		return Frame;
	}

	Frame.bValid = true;
	Frame.RegionCenter = CenterSum / static_cast<float>(CenterCount);
	if (VelocityCount > 0)
	{
		Frame.AverageVelocity = VelocitySum / static_cast<float>(VelocityCount);
	}

	// span: 접촉이 한 노드뿐이면(팁 우선 착지에서 흔함) 이웃 노드로 넓혀 로프가 누운 방향을 확보한다.
	const int32 SpanStart = FMath::Max(0, MinNode - 1);
	const int32 SpanEnd = FMath::Min(Sim.Num() - 1, MaxNode + 1);
	if (SpanEnd > SpanStart)
	{
		Frame.SpanDirection = (Sim.Positions[SpanEnd] - Sim.Positions[SpanStart]).GetSafeNormal();
	}

	// 진행 평면 normal = 속도 방향 × 누운 방향(둘 다 단위벡터 — 외적 크기가 곧 sin(사잇각)).
	// 속도가 0이거나 로프가 진행 방향으로 일자 비행(창던지기)이면 축퇴한다 — bHasPlaneNormal=false로
	// 남겨 소비자가 다음 폴백(형상 축 등)으로 넘어가게 한다. 사잇각 ~6° 미만은 수치 노이즈로 보고 버린다.
	const FVector VelocityDir = Frame.AverageVelocity.GetSafeNormal();
	const FVector Cross = FVector::CrossProduct(VelocityDir, Frame.SpanDirection);
	constexpr float MinPlaneSinAngle = 0.1f;
	if (Cross.SizeSquared() > FMath::Square(MinPlaneSinAngle))
	{
		Frame.PlaneNormal = Cross.GetSafeNormal();
		Frame.bHasPlaneNormal = true;
	}
	return Frame;
}

void FRopeContactTracker::Update(const TArray<FRopeContactCandidate>& Candidates, float DeltaTime,
	const USceneComponent* PreferredMesh, FName PreferredBone, bool bRequirePreferred)
{
	if (Candidates.Num() == 0)
	{
		Decay(DeltaTime);
		return;
	}

	// 집계 키는 (Mesh, Bone) 쌍이다. 본 이름만 키로 쓰면 같은 스켈레톤(같은 본 이름)을 쓰는 두 액터가
	// 한 프레임에 함께 닿을 때 서로 다른 대상의 후보가 한 버킷으로 합산되고 mesh가 마지막 후보로
	// 오귀속된다(cross-actor 밀집 상황의 캡처 오귀속 — 2026-07 주석 전수조사에서 발견).
	using FTargetKey = TPair<const USceneComponent*, FName>;
	TMap<FTargetKey, TArray<int32>> NodesByTarget;
	TMap<FTargetKey, float> ScoreByTarget;
	TMap<FTargetKey, int32> HeadNodeByTarget;
	for (const FRopeContactCandidate& Candidate : Candidates)
	{
		if (!Candidate.bValid || Candidate.Bone.IsNone())
		{
			continue;
		}

		const FTargetKey Key(Candidate.Mesh, Candidate.Bone);
		NodesByTarget.FindOrAdd(Key).Add(Candidate.NodeIndex);
		ScoreByTarget.FindOrAdd(Key) += Candidate.Penetration + FMath::Max(0.0f, Candidate.WrapDirectionScore);
		if (int32* ExistingHeadNode = HeadNodeByTarget.Find(Key))
		{
			*ExistingHeadNode = FMath::Min(*ExistingHeadNode, Candidate.NodeIndex);
		}
		else
		{
			HeadNodeByTarget.Add(Key, Candidate.NodeIndex);
		}
	}

	// 대상별 dwell 대장 갱신(시드 다중화 재료): 이번 프레임 존재하는 키는 dwell 누적 + 노드 교체,
	// 빠진 키는 같은 양만큼 감쇠 후 소진되면 제거(플리커 관용은 dominant dwell과 같은 규칙).
	// dominant 선정(아래)은 이 대장과 독립적으로 종전 로직을 그대로 쓴다 — 단일 시드 동작 불변.
	for (int32 Index = Targets.Num() - 1; Index >= 0; --Index)
	{
		FRopeTrackedContactTarget& Target = Targets[Index];
		const FTargetKey Key(Target.Mesh, Target.Bone);
		if (const TArray<int32>* Nodes = NodesByTarget.Find(Key))
		{
			Target.DwellTime += DeltaTime;
			Target.Nodes = *Nodes;
		}
		else
		{
			Target.DwellTime -= DeltaTime;
			Target.Nodes.Reset();
			if (Target.DwellTime <= 0.0f)
			{
				Targets.RemoveAt(Index);
			}
		}
	}
	for (const TPair<FTargetKey, TArray<int32>>& Pair : NodesByTarget)
	{
		const bool bAlreadyTracked = Targets.ContainsByPredicate(
			[&Pair](const FRopeTrackedContactTarget& Target)
			{
				return Target.Mesh == Pair.Key.Key && Target.Bone == Pair.Key.Value;
			});
		if (!bAlreadyTracked)
		{
			FRopeTrackedContactTarget& Target = Targets.AddDefaulted_GetRef();
			Target.Bone = Pair.Key.Value;
			Target.Mesh = Pair.Key.Key;
			Target.Nodes = Pair.Value;
			Target.DwellTime = 0.0f;
		}
	}

	FTargetKey BestTarget(nullptr, NAME_None);
	int32 BestCount = 0;
	int32 BestHeadNode = INDEX_NONE;
	float BestScore = 0.0f;
	// 이 파일의 변경 이유: pelvis처럼 노드 수가 많은 본이 조준한 팔을 rank로 역전하지 않도록 한다.
	// preferred는 dominant 선택에만 관여하며, 위 Targets 갱신은 모든 본에 대해 그대로 수행한다.
	const FTargetKey PreferredTarget(PreferredMesh, PreferredBone);
	const bool bHasPreferredTarget = PreferredMesh && !PreferredBone.IsNone()
		&& NodesByTarget.Contains(PreferredTarget);
	if (bHasPreferredTarget)
	{
		BestTarget = PreferredTarget;
	}
	else if (!bRequirePreferred)
	{
		for (const TPair<FTargetKey, TArray<int32>>& Pair : NodesByTarget)
		{
			const float Score = ScoreByTarget.FindRef(Pair.Key);
			const int32 HeadNode = HeadNodeByTarget.FindRef(Pair.Key);
			if (Pair.Value.Num() > BestCount ||
				(Pair.Value.Num() == BestCount &&
					(BestHeadNode == INDEX_NONE || HeadNode < BestHeadNode ||
						(HeadNode == BestHeadNode && Score > BestScore))))
			{
				BestTarget = Pair.Key;
				BestCount = Pair.Value.Num();
				BestHeadNode = HeadNode;
				BestScore = Score;
			}
		}
	}

	if (BestTarget.Value.IsNone())
	{
		if (bRequirePreferred)
		{
			CandidateBone = NAME_None;
			CandidateMesh = nullptr;
			CandidateNodes.Reset();
			DwellTime = 0.0f;
		}
		else
		{
			Decay(DeltaTime);
		}
		return;
	}

	// dwell 연속성도 (Mesh, Bone) 쌍 기준: 본 이름이 같아도 mesh가 바뀌면 다른 대상이므로 리셋한다.
	if (BestTarget.Value == CandidateBone && BestTarget.Key == CandidateMesh)
	{
		DwellTime += DeltaTime;
	}
	else
	{
		CandidateBone = BestTarget.Value;
		DwellTime = 0.0f;
	}

	CandidateMesh = BestTarget.Key;
	CandidateNodes = NodesByTarget.FindRef(BestTarget);
}
