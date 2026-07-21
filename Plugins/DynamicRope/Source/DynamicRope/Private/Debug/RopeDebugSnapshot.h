// Copyright Epic Games, Inc. All Rights Reserved.
//
// rope 디버그의 한 프레임 스냅샷. sim tick(GT)이 채워 URopeDebugSubsystem에 제출하고,
// FGameplayDebuggerCategory_Rope가 읽어 AddShape/AddTextLine으로 그린다. 즉시모드 DrawDebug*를
// 대체하는 데이터 운반체 — 모두 POD라 UObject 결합이 없다(솔버/로직 철학과 동일).

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

/**
 * 캡처 범위 비트. 카테고리의 보기 토글에서 만들어져 **캡처 측까지** 전달된다 — 그리기에서만 막으면
 * 꺼진 보기의 수집 비용(노드×콜라이더 재질의, Flight 추가 sweep, collider 형상/컨벡스 헐 재구성)이
 * 그대로 남아 디버거가 시뮬레이션을 계속 무겁게 한다.
 * Aim은 Wielder를 라이브로 읽고 Advanced는 표시 상세도라 캡처 비용이 없다 — 비트를 두지 않는다.
 * 헤더 요약(phase/노드 위치/솔브 경로)은 항상 채운다: 어느 보기를 켜든 필요하고 값도 싸다.
 */
enum class ERopeDebugCapture : uint8
{
	None      = 0,
	// 노드별 근접 재질의 — 노드 수 × collider 수의 CPU Query라 가장 비싸다.
	Nodes     = 1 << 0,
	// Flight 노드 재스윕 + 후보/whip 가이드 사본.
	Flight    = 1 << 1,
	// Wrapped 상세(latch 사본/pull 관측치)와 감김 축.
	Wrap      = 1 << 2,
	// collider 형상 사본 + 컨벡스 헐 엣지 재구성(O(plane³)).
	Colliders = 1 << 3,
};
ENUM_CLASS_FLAGS(ERopeDebugCapture);

// flight 노드별 디버그: 이전→현재 이동 + (필요 시) 접촉. 캡처 대상 로프에서만 채워진다.
struct FRopeFlightNodeDebug
{
	int32 NodeIndex = INDEX_NONE;
	FVector PrevPosition = FVector::ZeroVector;
	FVector Position = FVector::ZeroVector;
	// NodeSpeed/bFast는 캡처 루프의 게이팅 중간값이다 — 표시는 읽지 않는다(bNearBody만 노란 점으로 나간다).
	// 셋이 같은 게이팅 식에 함께 쓰이므로 출처를 가르지 않고 구조체에 나란히 둔다.
	float NodeSpeed = 0.0f;
	bool bFast = false;
	bool bNearBody = false;
	FRopeContact Contact;
	// Contact.SourceMesh를 캡처 시점에 굳힌 비교 전용 키(후보와 같은 대상인지 판정). 스냅샷은 몇 프레임
	// 살아남으므로 그리기 시점에 raw 포인터로 키를 만들면 이미 파괴된 컴포넌트를 역참조한다.
	FObjectKey ContactMeshKey;
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

// 노드별 **근접** 진단: post-solve 노드 위치를 collider에 다시 질의해 "이 노드가 어느 면을 어느 법선으로
// 마주하고 있나"를 데이터로 남긴다(GPU 런타임은 접촉을 리드백하지 않으므로 디버그 전용 CPU 질의).
// solver가 실제로 처리한 접촉이 **아니다** — 질의 반경을 CollisionRadius보다 넓혀 던지므로 닿지 않은
// 근처 노드도 잡힌다. 노드가 벽/면에 붙는 증상을 눈으로 확정하기 위한 것이라 그 편이 유용하다.
struct FRopeNodeProximityDebug
{
	int32   NodeIndex = INDEX_NONE;
	// 노드 월드 위치(화살표 시작)
	FVector Position = FVector::ZeroVector;
	// 접촉 바깥 법선(단위) — 어느 면인지 = 이 방향
	FVector Normal = FVector::ZeroVector;
	// 이 접촉을 낸 collider의 IsWorldStatic() — 색 구분용. 본 유무로 추론하지 않는다(본을 보고하지 않는
	// 커스텀 non-static collider와, 가상 본을 가진 정적 프롭이 둘 다 반례다).
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
	// 아니다. 감김 가능 대상 전체가 아니라 이 정적 opt-in 대상만 세는 값이라 화면 라벨도 staticWrapTargets다.
	bool bWrapTarget = false;
	// 이 collider에 실제로 감길 수 있는가 = 귀속(본+메시)이 유효하고 CanWrapTarget() 게이트를 통과했는가.
	// IsWorldStatic()과는 별개다 — 감지에는 들어오지만 게이트가 거부하는 대상이 있다.
	bool bWrapAllowed = false;

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
	// 프레임 종료 시점 phase(제출 직전). 아래 PhaseAtFrameStart와 다르면 이번 프레임에 전이한 것이다.
	ERopePhase Phase = ERopePhase::Free;
	// 프레임 시작(Prepare 진입) 시점 phase. 전이는 Prepare/Finalize 안에서 일어나므로, 한 스냅샷이
	// "Flight에서 시작해 Contacting으로 끝난 프레임"처럼 전이 전후를 함께 담는다 — 이때 flight 오버레이는
	// 전이를 일으킨 바로 그 관측이라 유효하다(phase가 다르다는 이유로 숨기면 전이 원인을 잃는다).
	ERopePhase PhaseAtFrameStart = ERopePhase::Free;
	TArray<FVector> Positions;
	// centerline 상에서 강조할 latch 노드 인덱스.
	TArray<int32> LatchedNodes;

	//~ 헤더 표시용 프레임 상태 -------------------------------------------
	// 화면 한 장이 하나의 시간 기준만 쓰도록, 헤더도 라이브 컴포넌트 대신 이 값들을 읽는다. 라이브와
	// 섞으면 같은 노드가 두 시점에 겹쳐 그려져 시뮬 떨림처럼 보인다.
	// (주의: GPU 경로에서는 Sim.Positions 자체가 리드백 미러라 1~2프레임 지연된다. 여기서 맞추는 것은
	//  디버거 내부의 일관성이지, 실제 GPU 버퍼로 그려지는 튜브와의 일치가 아니다.)
	FName WrapBoneName = NAME_None;
	bool  bSleeping = false;
	float LodScale = 1.0f;
	// tube 적격성 계산 입력(씬 프록시와 같은 소스인 설정값).
	int32 NumParticles = 0;
	int32 TubeSmoothingSubdiv = 1;
	// 솔브 경로 토큰 산출 입력 — 세 값의 조합이 6종을 가른다(bSleeping 포함).
	bool  bSolveThisFrame = false;
	bool  bGpuStepped = false;
	bool  bLogicOverride = false;

	//~ flight(Flight phase에서만) ----------------------------------------
	// (캡처 판정 3종은 담지 않는다: MinLatchNodes는 런타임에 안 바뀌는 config 상수고, bShouldCapture는
	//  true가 되는 즉시 같은 프레임에 Contacting으로 전이해 헤더의 `Flight→Contacting`과 중복이며,
	//  TrackerNodes의 위치는 후보 박스가 이미 그린다. 셋 다 찰나의 값이라 누적해서 읽는 stat RopeFlight의
	//  Candidate Nodes / Capture Decisions가 유효한 형태다.)
	bool bHasFlight = false;
	FName TrackerBone = NAME_None;
	// dominant 대상의 mesh를 캡처 시점에 변환한 키. 접촉 대상의 식별 계약은 (Mesh, Bone) 쌍이다
	// (FRopeContactTracker 주석 참조) — 본 이름만 비교하면 같은 스켈레톤을 쓰는 두 액터가 붙어 있을 때
	// 엉뚱한 후보가 dominant처럼 강조된다.
	FObjectKey TrackerMeshKey;
	TArray<FRopeFlightNodeDebug> NodeDebug;
	// 주의: Candidates[].Mesh는 raw 포인터다. 스냅샷이 몇 프레임 살아남으므로 **역참조 금지** —
	// 대상 일치 판정은 아래 CandidateMeshKeys(캡처 시 변환)로 한다. 인덱스는 Candidates와 1:1이다.
	TArray<FRopeContactCandidate> Candidates;
	TArray<FObjectKey> CandidateMeshKeys;

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
	// 이 로프의 도달 모드 — 자동 해제 문구에 함께 낸다.
	ERopeWrapResolveMode ResolveMode = ERopeWrapResolveMode::AssistedJudged;
	// 장력/거리 자동 해제가 실제로 동작하는가. GuaranteedWrap은 CheckWrappedAutoRelease가 조기 반환해
	// 임계치를 보지 않는다(보장 계약은 해제에도 대칭이라 명시 해제만 유효).
	bool bAutoReleaseEnabled = true;
	// 임계 장력을 연속 초과한 시간과 발동까지 필요한 시간(0 = 임계 release 비활성). 임계를 넘어도
	// TensionReleaseTime 동안 지속돼야 풀리므로 그 진행도를 표시한다.
	float TensionOverTime = 0.0f;
	float TensionReleaseTime = 0.0f;
	// ComputePull 성공(앵커/방향 유효 — 장력 0이어도 true)
	bool bPullValid = false;
	// 힘 인가점(앵커 월드)
	FVector PullPoint = FVector::ZeroVector;
	// 당김 단위 방향(EMA 스무딩 후 — 실제 인가 방향)
	FVector PullDirection = FVector::ZeroVector;
	// 스무딩 전 look-ahead 방향(원본) — 지터 진단용(스무딩 대비)
	FVector PullDirRaw = FVector::ZeroVector;
	// 첫 직선 다리 끝 = 스무딩된 fractional 조준 위치(AimPos) — 방향 EMA의 입력.
	// 정수 노드 위치가 아니다: 조준 인덱스를 EMA로 다듬은 뒤 노드 사이를 보간한 값이라 노드에 놓이지 않는다.
	FVector PullAimPoint = FVector::ZeroVector;
	// 스무딩 전 정수 조준 노드 인덱스(프레임마다 튀면 방향 불안정 신호) — 위 fractional 위치의 원본
	int32 PullAimNode = INDEX_NONE;
	// 앵커 세그먼트 장력
	float PullTension = 0.0f;
	// 테더 반응(0=비활성) — 표시용
	// 가용 로프 길이 초과분(cm, 0=팽팽하지 않음)
	float TetherOvershoot = 0.0f;
	// 이번 프레임 테더 장력(λ/dt 또는 랙돌 물리 제약 실측력, kg·cm/s²)과 그 상한 — 디버거 표시용.
	float TetherTension = 0.0f;
	float MaxTetherTension = 0.0f;
	// 능동 Pull **요청** 힘(0=입력 없음). SetActivePull이 저장한 값이라 인가 여부와는 별개다.
	float ActivePullForce = 0.0f;
	// 그 요청이 이번 프레임 팽팽 게이트를 통과해 실제로 인가됐는가. 요청값만 내면 "taut=N인데 우회 설정으로
	// 인가된" 경우와 "입력은 있으나 막힌" 경우가 같은 숫자로 보인다.
	bool bActivePullApplied = false;
	// 팽팽(taut) 게이트 상태 — 능동 Pull 인가 조건(IsPullTaut와 동일 래치)
	bool bPullTaut = false;
	// 전 체인 팽팽(기하) 게이트 — 견인(테더+능동 Pull) 공용 선행 조건(코너-다리 chord 합 vs rest 길이)
	bool bChainTaut = false;
	// 앵커→손 코너-다리 chord 합(cm) — bChainTaut의 관측치
	float TautChordLen = 0.0f;
	// 자유 구간(손~앵커) rest 길이(cm) = AnchorNode × SegmentLength
	float FreeRestLen = 0.0f;
	// 자유 구간 세그먼트 장력 최솟값 — 0이면 어딘가 슬랙(장력이 손까지 전달 안 됨 = 구김/부분 스트레치)
	float MinFreeTension = 0.0f;
	// 다리별 최대 처짐(cm) — 내부 노드의 다리 chord 직선 이탈. 시각적 "펴짐"의 직접 관측치
	float MaxLegSag = 0.0f;
	// 거리 release 한계(cm, 0=비활성) — 표시용
	float DistanceReleaseSlack = 0.0f;

	//~ colliders(이 로프가 이번 프레임 질의한 collider들) ----------------
	TArray<FRopeDebugCollider> Colliders;

	//~ 노드별 근접(디버그 CPU 재질의) — 붙는 노드 진단 -------------------
	TArray<FRopeNodeProximityDebug> NodeProximity;
	// 재질의에 더한 여유 반경(cm). 화면이 "이건 solver 접촉이 아니라 r+N cm 질의 결과"라고 밝히는 데 쓴다.
	float ProximityQueryMargin = 0.0f;

	//~ 감김 축(Wrapping에서 ResolveWrappingAxis가 정한 경로 축) — [I] wrap 뷰 선 시각화용 ---
	// Wrapping 페이즈에서만 유효(bHasWrapAxis). 어느 축으로 감기는지 눈으로 확인하기 위한 것.
	bool    bHasWrapAxis = false;
	FVector WrapAxisOrigin = FVector::ZeroVector;
	FVector WrapAxisDirection = FVector::ForwardVector;
	// 축 시각화 길이 도출용 세그먼트 길이. 디버거가 max(80, ×6) 수식으로 로프 스케일에 비례한 축을 그린다.
	float   WrapAxisSegmentLength = 0.0f;
};
