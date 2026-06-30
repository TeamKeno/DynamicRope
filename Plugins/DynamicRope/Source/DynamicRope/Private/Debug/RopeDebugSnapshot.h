// Copyright Epic Games, Inc. All Rights Reserved.
//
// rope 디버그의 한 프레임 스냅샷. sim tick(GT)이 채워 URopeDebugSubsystem에 제출하고,
// FGameplayDebuggerCategory_Rope가 읽어 AddShape/AddTextLine으로 그린다. 즉시모드 DrawDebug*를
// 대체하는 데이터 운반체 — 모두 POD라 UObject 결합이 없다(솔버/로직 철학과 동일).

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

// flight 노드별 디버그: 이전→현재 이동 + (필요 시) 접촉. 캡처 대상 로프에서만 채워진다.
struct FRopeFlightNodeDebug
{
	int32 NodeIndex = INDEX_NONE;
	FVector PrevPosition = FVector::ZeroVector;
	FVector Position = FVector::ZeroVector;
	float NodeSpeed = 0.0f;
	bool bFast = false;
	bool bNearBody = false;
	FRopeContact Contact;
};

// collider 시각화 한 개. analytic capsule이면 A-B 세그먼트+반지름, 그 외(SDF 등)는 월드 bounds 박스.
struct FRopeDebugCollider
{
	bool bIsCapsule = false;
	FVector A = FVector::ZeroVector;
	FVector B = FVector::ZeroVector;
	float Radius = 0.0f;
	FBox Bounds = FBox(ForceInit);
};

// 한 로프의 한 프레임 디버그 스냅샷. centerline/flight/whip/wrapped/collider를 한데 담는다.
// 비어 있는 섹션은 b*Has 플래그로 구분(예: bHasFlight는 Flight phase에서만 true).
struct FRopeDebugSnapshot
{
	// 제출 프레임(GFrameCounter). 카테고리는 너무 오래된 스냅샷을 무시한다(대상 해제 후 잔상 방지).
	uint64 FrameStamp = 0;

	//~ centerline(항상) ---------------------------------------------------
	ERopePhase Phase = ERopePhase::Free;
	TArray<FVector> Positions;
	TArray<int32> LatchedNodes; // centerline 상에서 강조할 latch 노드 인덱스.

	//~ flight(Flight phase에서만) ----------------------------------------
	bool bHasFlight = false;
	bool bSolveThisFrame = false;
	bool bShouldCapture = false;
	int32 FrameColliderCount = 0;
	int32 MinLatchNodes = 0;
	FName TrackerBone = NAME_None;
	TArray<int32> TrackerNodes;
	TArray<FRopeFlightNodeDebug> NodeDebug;
	TArray<FRopeContactCandidate> Candidates;

	//~ whip guide(whip 활성 시) ------------------------------------------
	bool bWhipActive = false;
	float WhipGuidedEnd = 0.0f;
	TArray<int32> WhipGuideNodeIndices;
	TArray<FVector> WhipGuideTargets;

	//~ wrapped(Wrapped phase에서만) --------------------------------------
	bool bHasWrapped = false;
	FName WrapBone = NAME_None;
	FString MeshName;
	TArray<FRopeLatchNode> Latched;

	//~ colliders(이 로프가 이번 프레임 질의한 collider들) ----------------
	TArray<FRopeDebugCollider> Colliders;
};
