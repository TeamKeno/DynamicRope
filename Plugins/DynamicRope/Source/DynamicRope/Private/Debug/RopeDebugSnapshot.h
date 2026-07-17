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

// collider 시각화 형상 종류. 채우기(FillDebugSnapshot)가 상호 배타 accessor로 분류한다.
enum class ERopeDebugColliderShape : uint8
{
	// A-B 세그먼트 + 반지름(스켈레탈 본 / 정적 스피어·스필)
	Capsule,
	// 회전 OBB(Center/Rot/HalfExtents) — 정적 박스
	Box,
	// 헐 와이어프레임(ConvexEdges: 연속 2개가 한 엣지) — 정적 컨벡스/전단 박스
	Convex,
	// 월드 AABB 폴백(SDF 등 형상 미상)
	Bounds,
};

// 노드별 접촉 진단: post-solve 노드 위치를 collider에 다시 질의해 "지금 이 노드가 어느 면에, 어느 법선으로
// 닿았나"를 데이터로 남긴다(GPU 런타임은 접촉을 리드백하지 않으므로 디버그 전용 CPU 질의). 노드가 벽/면에
// 붙는 증상을 눈으로 확정하기 위한 것 — 화살표(법선) + 텍스트(면 축/collider 종류)로 그린다.
struct FRopeNodeContactDebug
{
	int32   NodeIndex = INDEX_NONE;
	// 노드 월드 위치(화살표 시작)
	FVector Position = FVector::ZeroVector;
	// 접촉 바깥 법선(단위) — 어느 면인지 = 이 방향
	FVector Normal = FVector::ZeroVector;
	// 질의 반경 대비 침투(>0=밴드 안). 붙음 정도.
	float   Penetration = 0.0f;
	// 정적 월드(박스/컨벡스 등) vs 스켈레탈 — 색 구분.
	bool    bWorldStatic = false;
	// 스켈레탈이면 본 이름(없으면 None=정적).
	FName   Bone = NAME_None;
};

// collider 시각화 한 개. Shape에 따라 해당 필드만 유효하다.
struct FRopeDebugCollider
{
	ERopeDebugColliderShape Shape = ERopeDebugColliderShape::Bounds;
	// 정적 월드(박스/컨벡스/정적 캡슐) vs 스켈레탈 — 색 구분용.
	bool bWorldStatic = false;
	// 정적 메시 랩 대상(URopeWrapTargetComponent가 서빙): 가상 본은 있지만(감지 참여) SourceMesh가 스켈레탈이
	// 아니다. 스켈레탈 본(초록)/정적 월드(cyan)와 별색으로 그려 "로프가 실제로 감길 추출 셰이프"를 눈에 띄게 한다.
	bool bWrapTarget = false;

	// Capsule
	FVector A = FVector::ZeroVector;
	FVector B = FVector::ZeroVector;
	float   Radius = 0.0f;

	// Box (OBB)
	FVector Center = FVector::ZeroVector;
	FQuat   Rot = FQuat::Identity;
	FVector HalfExtents = FVector::ZeroVector;

	// Convex: 월드 공간 엣지 끝점(연속 2개 = 한 엣지). 디버그 그리기 전용(런타임 콜라이더엔 저장 안 함).
	TArray<FVector> ConvexEdges;

	// Bounds 폴백
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
	// centerline 상에서 강조할 latch 노드 인덱스.
	TArray<int32> LatchedNodes;

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
	// 최대 세그먼트 장력(FRopeWrapState::Tension 미러)
	float WrapTension = 0.0f;
	// 임계 장력(0=비활성) — 표시용
	float TensionReleaseForce = 0.0f;
	// ComputePull 성공(앵커/방향 유효 — 장력 0이어도 true)
	bool bPullValid = false;
	// 힘 인가점(앵커 월드)
	FVector PullPoint = FVector::ZeroVector;
	// 당김 단위 방향(EMA 스무딩 후 — 실제 인가 방향)
	FVector PullDirection = FVector::ZeroVector;
	// 스무딩 전 look-ahead 방향(원본) — 지터 진단용(스무딩 대비)
	FVector PullDirRaw = FVector::ZeroVector;
	// 첫 직선 다리 끝(walk가 멈춘 노드 월드) — 방향 조준점
	FVector PullAimPoint = FVector::ZeroVector;
	// 위 조준 노드 인덱스(프레임마다 튀면 방향 불안정 신호)
	int32 PullAimNode = INDEX_NONE;
	// 앵커 세그먼트 장력
	float PullTension = 0.0f;
	// 테더 반응(0=비활성) — 표시용
	float TetherResponse = 0.0f;
	// 가용 로프 길이 초과분(cm, 0=팽팽하지 않음)
	float TetherOvershoot = 0.0f;
	// 능동 Pull 힘(0=입력 없음)
	float ActivePullForce = 0.0f;
	// 팽팽(taut) 게이트 상태 — 능동 Pull 인가 조건(IsPullTaut와 동일 래치)
	bool bPullTaut = false;
	// 전 체인 팽팽(기하) 게이트 — 견인(테더+능동 Pull) 공용 선행 조건(코너-다리 chord 합 vs rest 길이)
	bool bChainTaut = false;
	// 앵커→손 코너-다리 chord 합(cm) — bChainTaut의 관측치
	float TautChordLen = 0.0f;
	// 자유 구간(손~앵커) rest 길이(cm) = AnchorNode × SegmentLength
	float FreeRestLen = 0.0f;
	// 거리 release 한계(cm, 0=비활성) — 표시용
	float DistanceReleaseSlack = 0.0f;

	//~ colliders(이 로프가 이번 프레임 질의한 collider들) ----------------
	TArray<FRopeDebugCollider> Colliders;

	//~ 노드별 접촉(디버그 CPU 재질의) — 붙는 노드 진단 -------------------
	TArray<FRopeNodeContactDebug> NodeContacts;

	//~ 감김 축(Wrapping에서 ResolveWrappingAxis가 정한 경로 축) — [O] 뷰 선 시각화용 ---
	// Wrapping 페이즈에서만 유효(bHasWrapAxis). 어느 축으로 감기는지 눈으로 확인하기 위한 것.
	bool    bHasWrapAxis = false;
	FVector WrapAxisOrigin = FVector::ZeroVector;
	FVector WrapAxisDirection = FVector::ForwardVector;
};
