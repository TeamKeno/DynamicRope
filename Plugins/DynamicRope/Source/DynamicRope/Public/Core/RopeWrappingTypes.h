// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtrTemplates.h"

class USceneComponent;

/** latch된 wrap 노드. bone-local 공간에 고정되어 재충돌 없이 skinning을 따라간다. */
struct FRopeLatchNode
{
	int32   NodeIndex = INDEX_NONE;
	FName   Bone = NAME_None;
	FVector BoneLocalPos = FVector::ZeroVector;
};

/** Wrapping/Wrapped가 노드를 표면에 고정할 때 쓰는 표면 앵커(bone-local 프레임 + 진행 눈금). */
struct FRopeSurfaceAnchor
{
	int32 NodeIndex = INDEX_NONE;

	FName Bone = NAME_None;
	TWeakObjectPtr<const USceneComponent> Mesh = nullptr;

	/** SDF 표면 기준 bone-local 앵커 프레임(위치/법선/접선). */
	FVector LocalSurfacePosition = FVector::ZeroVector;
	FVector LocalNormal = FVector::UpVector;
	FVector LocalTangent = FVector::ForwardVector;

	/** Composite Wrapping front 뒤 tail의 연장 방향. 표면 tangent와 분리된 ideal helix guide를
	 *  bone-local로 저장하며 Wrapped 물리/표면 프레임에는 사용하지 않는다. */
	FVector LocalWrappingGuideTangent = FVector::ForwardVector;
	bool bHasWrappingGuideTangent = false;

	/** Wrapping 시작 순간의 월드 위치. front 모션의 Lerp 시작점으로 사용한다. */
	FVector StartWorldPosition = FVector::ZeroVector;

	/** 여러 번 감김/길이 계산이 쓰는 로프 진행 눈금. 커밋 시점 눈금으로 저장되므로 reel(길이 변경) 후엔 stale. */
	float RopeDistance = 0.f;

	/** 표면에서 로프 중심선을 얼마나 띄울지(보통 rope radius). */
	float SurfaceOffset = 0.0f;

	/** Pierce 전용 — 꽂힌 순간 얼린 팁 메쉬 원점의 bone-local 트랜스폼(스케일 포함).
	 *  Wrapped 동안 팁 렌더 자세의 단일 소스(회전 freeze + 본 추종). 비-Pierce는 Identity로 미사용. */
	FTransform LocalMeshTransform = FTransform::Identity;
};

/** Wrapping 경로의 점 하나(월드 기준 프레임 + 귀속 본/메시 + latch로부터의 진행 거리). */
struct FRopeWrapPathPoint
{
	/**
	 * bVirtual=false이면 rope centerline의 normal offset 이전 기준 위치다. 일반 점은 실제 투영 표면,
	 * bBridge 점은 허공 centerline에서 offset을 뺀 가상 기준 위치다. bVirtual=true이면 표면이 없으므로
	 * 이미 완성된 rope centerline을 저장한다. centerline 소비자는 이 규약을 직접 재구현하지 말고
	 * FRopeWrappingPhase의 공용 변환을 사용해야 한다.
	 */
	FVector SurfaceWorld = FVector::ZeroVector;
	FVector NormalWorld = FVector::UpVector;
	FVector TangentWorld = FVector::ForwardVector;

	/** Composite Wrapping tail 전용 ideal helix 방향. SDF normal의 미세 불규칙성을 따르지 않는다. */
	FVector WrappingGuideTangentWorld = FVector::ForwardVector;
	bool bHasWrappingGuideTangent = false;

	/**
	 * 이 path point가 투영된 실제 표면 본.
	 * Composite AnalyticHelix와 Sequential SurfaceVectorField 모두 실제 projection 결과를 넣는다.
	 * Composite radial ray가 빗나간 virtual point는 NAME_None이다.
	 * 이후 ProcessPathPointForAnchoring이 이 값을 기준으로 bone-local anchor를 저장한다.
	 */
	FName Bone = NAME_None;
	TWeakObjectPtr<const USceneComponent> Mesh = nullptr;

	float DistanceFromLatch = 0.0f;

	/**
	 * latch에서 이 path point까지의 wrapping animation 위상(rad). Composite는 ideal helix angle,
	 * Sequential SurfaceVectorField는 winding 방향 forward signed angle을 실제 centerline arc
	 * 재샘플 alpha로 보간해 저장한다. DistanceFromLatch는 물리 rope spacing을 유지하고,
	 * 이 값은 두 모드가 공유하는 front angle -> distance 매핑에 사용한다.
	 */
	float WrapAngleFromLatchRad = 0.0f;

	/**
	 * 허공 브리지(chord) 경로점(WrappingMaxGapBridgeDistance > 0에서만 발생): 표면 투영 없이
	 * tangent 직진으로 만들어졌다. front 모션의 위치 목표로는 참여하지만 앵커는 만들지 않는다 —
	 * 커밋 후 이 구간 노드는 자유 로프로 남는다.
	 */
	bool bBridge = false;

	/**
	 * Composite Analytic Helix의 no-anchor 점. radial SDF ray가 표면과 교차하지 않을 때
	 * IdealHelixWorld는 경로/디버그 위상으로만 보존한다. 이 점에 대응하는 노드는 Wrapping부터
	 * 위치 override와 surface anchor를 받지 않고 즉시 solver가 처리한다.
	 */
	bool bVirtual = false;
};

/**
 * Composite path에서 양쪽 실제 surface point로 닫힌 연속 virtual 구간.
 * 경로 빌드가 한 번만 산출하고 URopeComponent가 Wrapping~Wrapped 동안 같은 bridge를 이어서 소유한다.
 */
struct FRopeVirtualBridgeRun
{
	int32 LeftNodeIndex = INDEX_NONE;
	int32 RightNodeIndex = INDEX_NONE;
	TArray<int32> VirtualNodeIndices;
};

/** 접촉 시 실제로 평가한 pose-space gap의 판정 결과. island 연결 판정과 실패 진단이 함께 읽는다. */
enum class ERopeWrapIslandPortalState : uint8
{
	Open,
	ClosedGeometry,
	ClosedReachability
};

/** island 후보 SDF collider의 접촉 시점 volume 스냅샷. 런타임 복합 단면 계산이 추가 샘플링 없이 쓴다. */
struct FRopeWrapIslandDebugMember
{
	FName Bone = NAME_None;
	FBox WorldBounds = FBox(EForceInit::ForceInit);

	/** SDF grid의 실제 oriented bounds. SDF accessor가 없는 collider면 WorldBounds로 폴백한다. */
	bool bHasOrientedSDFBounds = false;
	FVector SDFCenter = FVector::ZeroVector;
	FVector SDFHalfExtent = FVector::ZeroVector;
	FQuat SDFRotation = FQuat::Identity;
};

/** island 생성 중 이미 계산한 두 표면 projection과 길이 판정값의 스냅샷. */
struct FRopeWrapIslandDebugPortal
{
	FName BoneA = NAME_None;
	FName BoneB = NAME_None;
	FVector SurfacePointA = FVector::ZeroVector;
	FVector SurfacePointB = FVector::ZeroVector;
	ERopeWrapIslandPortalState State = ERopeWrapIslandPortalState::Open;
	float SurfaceGap = 0.0f;
};

/** Wrapping 페이즈의 작업 상태(FRopeWrappingPhase::State). 경로 빌드 진행/앵커 축적/커밋 판정 재료. */
struct FRopeWrappingState
{
	FName BoneName = NAME_None;
	TWeakObjectPtr<const USceneComponent> Mesh = nullptr;

	TArray<FRopeSurfaceAnchor> Anchors;
	FRopeSurfaceAnchor LatchAnchor;
	TArray<FRopeWrapPathPoint> Path;

	/** Path 생성 중 한 번만 발견해 누적하는 bounded virtual run 목록. */
	TArray<FRopeVirtualBridgeRun> VirtualBridgeRuns;
	/** Path가 프레임마다 뒤에 붙으므로 새 구간만 검사하기 위한 점진 scan 상태. */
	int32 VirtualBridgeScanPathIndex = 0;
	int32 VirtualRunStartPathIndex = INDEX_NONE;
	int32 VirtualRunLeftPathIndex = INDEX_NONE;

	/**
	 * 보조 시드 앵커(시드 다중화, MaxWrapSeeds > 1): dominant latch보다 tail 쪽에서 *다른* 본에
	 * 접촉해 채택된 앵커들. 경로 빌드/front 모션의 대상이 아니라 Wrapping 내내 자기 본에 hold되고,
	 * BuildCommitSeed에서 경로 앵커들과 합류한다(BeginWrap/Hold는 앵커별 Bone/Mesh를 이미 지원).
	 * Begin *이전에* 채워야 한다 — BeginProgressiveWrapPathBuild가 경로 길이(NumTailNodes)를
	 * 첫 보조 시드 노드 앞까지로 클램프하는 데 읽는다(경로 앵커와 보조 앵커의 노드 중복 방지).
	 */
	TArray<FRopeSurfaceAnchor> SecondarySeedAnchors;

	int32 NumTailNodes = 0;
	int32 LastAnchoredPathPointCount = 0;
	bool bPathBuildActive = false;
	bool bPathBuildComplete = false;
	bool bPathBuildFailed = false;

	/** 마지막 path build 실패의 기계 판독 가능한 원인. 최종 abort 로그가 소비한다. */
	FString PathBuildFailureReason;

	/** projection 결과로 확정된 rope centerline polyline의 실제 누적 길이(cm).
	 *  Path의 다음 node는 이 거리에서 SegmentLength 경계를 통과할 때 재샘플링해 생성한다. */
	float PathCurrentDistance = 0.0f;

	/** predictor/composite sweep가 시도한 명목 진행 거리(cm).
	 *  projection이 제자리여도 증가해 표면 탐색 위상이 멈추지 않게 한다. */
	float PathSweepDistance = 0.0f;

	/** Composite Analytic Helix의 직전 raw probe가 SDF projection에 실패한 virtual 점인가.
	 *  출력 Path와 독립된 raw polyline의 arc-length 재샘플링에서 구간 종류를 보존한다. */
	bool bPathCompositeRawPointVirtual = false;

	/** 직전 raw probe의 projection 전 ideal helix tangent. 다음 arc-length 출력점 guide 보간용. */
	FVector PathCompositeRawGuideTangentWorld = FVector::ForwardVector;
	float FrontDistance = 0.0f;
	/** angle -> distance front가 현재까지 진행한 공통 animation 위상(rad).
	 *  물리 rope distance와 분리되며 Single/Composite가 같은 각속도 정책을 사용한다. */
	float FrontWrapAngleRad = 0.0f;
	/** 현재 최종 path가 요구하는 물리적 front 이동 거리(cm).
	 *  진행 중인 path build의 임시 끝점이 아니라 commit 시점에 검사할 최종 거리 목표다. */
	float FrontTargetDistance = 0.0f;
	/** 현재 최종 path가 요구하는 animation 위상(rad).
	 *  거리와 별도로 저장해 투영 후 path 밀도가 달라도 한 바퀴의 완료 시점을 동일하게 판정한다. */
	float FrontTargetWrapAngleRad = 0.0f;
	/** 현재 front가 point별 angle -> distance 매핑을 사용하는지 여부.
	 *  false인 degenerate/legacy path는 기존 거리 기반 완료 정책으로 안전하게 폴백한다. */
	bool bFrontUsesAngleMapping = false;
	/** angle과 distance가 모두 최종 목표에 처음 도달한 Wrapping phase 경과 시간.
	 *  이 시각부터 post-front 안정화 시간을 재며, 음수면 아직 목표에 도달하지 않은 상태다. */
	float FrontReachedTargetElapsed = -1.0f;

	/** front가 실제로 처음 전진한 Wrapping phase 경과 시간. angle-mapped front의
	 *  모드별 초/회전 로그를 계산하며, 음수면 아직 front가 출발하지 않은 상태다. */
	float FrontMotionStartElapsed = -1.0f;
	bool bFrontMotionStartLogged = false;
	bool bFrontMotionCompletionLogged = false;

	FVector PathSurfaceWorld = FVector::ZeroVector;
	FVector PathNormalWorld = FVector::UpVector;
	FVector PathTangentWorld = FVector::ForwardVector;
	FVector PathCircumferenceDir = FVector::ForwardVector;
	FVector PathAxisOrigin = FVector::ZeroVector;
	FVector PathAxisDirection = FVector::ForwardVector;
	FVector PathLatchRadial = FVector::ForwardVector;

	/**
	 * SurfaceVectorField 적분 중 현재 surface point가 어느 본 위에 있는지 추적한다.
	 * 다음 step의 후보 본은 이 값을 중심으로 skeleton graph 근방에서 고른다.
	 */
	FName PathCurrentBone = NAME_None;

	/**
	 * 마지막으로 떠난 본. 새 후보가 바로 이 본이면 A->B->A 왕복 가능성이 높으므로
	 * scoring 단계에서 ImmediateBoneReturnPenalty를 더해 전환 떨림을 줄인다.
	 */
	FName PathPreviousBone = NAME_None;
	TWeakObjectPtr<const USceneComponent> PathCurrentMesh = nullptr;

	/**
	 * 접촉 순간 현재 포즈에서 구성한 복합 감김 대상의 본 목록.
	 * 이 목록은 skeleton 계보/전환 깊이가 아니라, 투척 slab 안에서 로프 두께로 확장한 표면끼리
	 * 실제로 이어지거나 현재 가용 slack으로 사이를 통과할 수 없는 collider island다.
	 * SurfaceVectorField 경로는 매 step마다 이 목록 전체를 lazy projection해 하나의 기둥 표면처럼 다룬다.
	 */
	TArray<FName> PathWrapIslandBones;

	/** 접촉 시 실제 island 판정에서 나온 스냅샷. member는 런타임 복합 단면 계산에, portal은 실패 진단에
	 *  사용하며 별도 경로/SDF를 만들지 않는다. */
	TArray<FRopeWrapIslandDebugMember> PathWrapIslandDebugMembers;
	TArray<FRopeWrapIslandDebugPortal> PathWrapIslandDebugPortals;

	/** 복합 island 구성 시 계산한 미고정 로프의 가용 여유 길이(cm). 디버그/portal 판정 재현용. */
	float PathAvailableSlack = 0.0f;

	/** true면 순차 Surface Vector Field 대신 pose-space island 기반 Composite Analytic Helix를 사용한다. */
	bool bPathUsesPoseSpaceIsland = false;

	/** 복합 island 경로가 실패해 최초 latch 본 하나로 경로를 처음부터 다시 만드는 중인가. */
	bool bPathUsesSingleBoneFallback = false;

	/**
	 * 선택된 개별 bone SDF의 local tangent와 무관하게 복합 island 외곽을 훑는 축 수직 방향.
	 * 팔 표면 normal이 원주 tangent를 지워도 이 방향은 매 step 독립적으로 회전한다.
	 */
	FVector PathCompositeSweepRadial = FVector::ForwardVector;

	/** 복합 단면 전체 바깥에서 SDF support projection을 시작할 축 반지름(cm). */
	float PathCompositeProbeRadius = 0.0f;

	/** composite 축에서 island 전체 SDF bounds 외곽까지의 최대 투영 반지름(cm, probe margin 제외). */
	float PathCompositeHelixRadius = 0.0f;

	/** Contacting 순간 tail 방향의 axis/circumference 비율로 자동 산출한 composite helix pitch. */
	float PathCompositeHelixPitchScale = 0.0f;

	/** composite 축 원점 기준 island 전체 SDF bounds의 축 방향 범위(cm). */
	float PathCompositeAxisMinDistance = 0.0f;
	float PathCompositeAxisMaxDistance = 0.0f;
	bool bPathCompositeAxisRangeValid = false;

	/** 독립 sweep radial이 시작점에서 누적 회전한 양(라디안, 진행 상태 로그용). */
	float PathCompositeSweepAngleRad = 0.0f;

	/** Composite Analytic Helix의 개별 radial projection이 실패해 virtual point를 만든 횟수. */
	int32 PathCompositeProjectionFailureCount = 0;

	/**
	 * 마지막 본 전환 이후 path가 표면을 따라 진행한 거리(cm).
	 * 새 본 후보가 좋아 보여도 MinBoneTransitionPathDistance 전에는 현재 본을 유지해
	 * 한두 step마다 본이 바뀌는 flicker를 막는다.
	 */
	float PathDistanceSinceBoneTransition = 0.0f;

	float PathWindingSign = 1.0f;

	/**
	 * 현재 이어지고 있는 허공 브리지(chord)의 누적 길이(cm). 표면 재진입(스냅 수용) 시 0으로 리셋.
	 * WrappingMaxGapBridgeDistance를 넘기면 경로 빌드가 실패 처리된다(브리징 비활성이면 항상 0).
	 */
	float PathBridgeDistance = 0.0f;

	/**
	 * 경로 빌드 중 적분한 누적 감싼 각도(라디안). Sequential SurfaceVectorField는 걷기 스텝마다
	 * rolling axis 기준 radial 회전량을 더하고, Composite AnalyticHelix는 노드별 나선 위상을 저장한다.
	 */
	float PathAccumulatedAngleRad = 0.0f;

	/** Sequential SurfaceVectorField 각도 진단. 기존 PathAccumulatedAngleRad는 acos 기반 절댓값
	 *  누적을 유지하고, 아래 값은 winding 방향을 +로 둔 signed/forward/reverse 성분을 병렬 기록한다. */
	float PathSignedNetAngleRad = 0.0f;
	float PathForwardAngleRad = 0.0f;
	float PathReverseAngleRad = 0.0f;

	/** Sequential step이 실제 사용한 rolling axis 기준 surface radius 통계. */
	float PathRadiusSum = 0.0f;
	float PathRadiusMin = 0.0f;
	float PathRadiusMax = 0.0f;
	int32 PathRadiusSampleCount = 0;
	int32 PathBoneTransitionCount = 0;

	float Elapsed = 0.0f;
	float Duration = 0.16f;
	float StableTime = 0.0f;

	int32 FirstNode = INDEX_NONE;
	int32 LastNode = INDEX_NONE;
	int32 LastStableFirstNode = INDEX_NONE;
	int32 LastStableLastNode = INDEX_NONE;
	int32 LastStableAnchorCount = 0;

	void Reset()
	{
		*this = FRopeWrappingState();
	}

	bool IsActive() const
	{
		return !BoneName.IsNone() && Anchors.Num() > 0;
	}
};

/**
 * wrap 이후 데이터 모델(바인딩 시맨틱). 물리→로직 핸드오프 시점에 생성된다.
 * TODO: Wrapped까지 surface anchor 기반 정리가 끝나면 legacy Latched 경로를 제거하고 Anchors로 일원화한다.
 */
struct FRopeWrapState
{
	FName                   BoneName = NAME_None;

	/** legacy fallback(bone-local 점 고정, 구 방식). 현행 커밋 경로는 Anchors를 채운다. */
	TArray<FRopeLatchNode>  Latched;

	/** 표면 앵커(현행 방식). */
	TArray<FRopeSurfaceAnchor> Anchors;

	/** 최대 세그먼트 장력(Wrapped 중 매 프레임 갱신). */
	float                   Tension = 0.0f;

	// BoneName을 소유한 Mesh. wrap은 이 mesh에 대해 유지/추적된다(rope 소유자와 다른
	// 액터일 수 있음). 결정 시점에 컨택트로부터 해석된다.
	// cross-actor wrap에서는 대상 액터가 Wrapped 도중 파괴될 수 있다. raw 포인터로 보관하면
	// Hold가 매 프레임 dangling 포인터를 역참조(use-after-free)하므로, 파괴 시 안전하게 null이
	// 되는 weak 포인터로 보관한다(POD 유지: hard 레퍼런스가 아니라 GC를 막지 않는다).
	TWeakObjectPtr<const USceneComponent> Mesh = nullptr;

	bool IsWrapped() const { return Anchors.Num() > 0 || Latched.Num() > 0; }
	void Reset() { *this = FRopeWrapState(); }
};
