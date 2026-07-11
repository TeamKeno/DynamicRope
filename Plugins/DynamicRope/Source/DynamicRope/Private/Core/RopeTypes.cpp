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

void FRopeContactTracker::Update(const TArray<FRopeContactCandidate>& Candidates, float DeltaTime)
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

	FTargetKey BestTarget(nullptr, NAME_None);
	int32 BestCount = 0;
	int32 BestHeadNode = INDEX_NONE;
	float BestScore = 0.0f;
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

	if (BestTarget.Value.IsNone())
	{
		Decay(DeltaTime);
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
