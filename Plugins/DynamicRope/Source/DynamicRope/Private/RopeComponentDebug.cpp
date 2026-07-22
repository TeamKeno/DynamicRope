// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeComponentInternal.h"

#include "Collision/RopeCollider.h"
#include "Debug/RopeDebugDraw.h"
#include "Debug/RopeDebugSnapshot.h"
#include "DynamicRopeLog.h"
#include "Engine/World.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

#pragma region Wrapping_Debug_And_Diagnostics

namespace RopeComponentPrivate
{
	void LogWrappingFailureState(const FString& OwnerName, const TCHAR* FailureSite,
		const FRopeWrappingState& State, const FRopeSimState& Sim)
	{
		FString IslandBones;
		for (const FName Bone : State.PathWrapIslandBones)
		{
			if (!IslandBones.IsEmpty())
			{
				IslandBones += TEXT(",");
			}
			IslandBones += Bone.ToString();
		}

		int32 ClosedGeometryCount = 0;
		int32 ClosedReachabilityCount = 0;
		int32 OpenPortalCount = 0;
		for (const FRopeWrapIslandPortal& Portal : State.PathWrapIslandPortals)
		{
			switch (Portal.State)
			{
			case ERopeWrapIslandPortalState::ClosedGeometry: ++ClosedGeometryCount; break;
			case ERopeWrapIslandPortalState::ClosedReachability: ++ClosedReachabilityCount; break;
			default: ++OpenPortalCount; break;
			}
		}

		const FRopeWrapPathPoint* LastPathPoint = State.Path.Num() > 0 ? &State.Path.Last() : nullptr;
		UE_LOG(LogRopeWrap, Error,
			TEXT("[%s] WRAP FAILURE: site=%s reason=%s active=%d complete=%d failed=%d algorithm=%s latch=%s[%d] current=%s previous=%s path=%d/%d anchors=%d secondary=%d currentDistance=%.2fcm bridge=%.2fcm sweep=%.1fdeg accumulated=%.1fdeg projectionMisses=%d island=[%s] portals(geometry=%d reachability=%d open=%d)"),
			*OwnerName, FailureSite ? FailureSite : TEXT("Unknown"),
			State.PathBuildFailureReason.IsEmpty() ? TEXT("Unspecified") : *State.PathBuildFailureReason,
			State.bPathBuildActive ? 1 : 0, State.bPathBuildComplete ? 1 : 0,
			State.bPathBuildFailed ? 1 : 0,
			State.bPathUsesPoseSpaceIsland ? TEXT("CompositeAnalyticHelix") :
				(State.bPathUsesSingleBoneFallback ? TEXT("SingleBoneFallback") : TEXT("SequentialSurfaceVectorField")),
			*State.LatchAnchor.Bone.ToString(), State.LatchAnchor.NodeIndex,
			*State.PathCurrentBone.ToString(), *State.PathPreviousBone.ToString(),
			State.Path.Num(), State.NumTailNodes, State.Anchors.Num(), State.SecondarySeedAnchors.Num(),
			State.PathCurrentDistance, State.PathBridgeDistance,
			FMath::RadiansToDegrees(State.PathCompositeSweepAngleRad),
			FMath::RadiansToDegrees(State.PathAccumulatedAngleRad),
			State.PathCompositeProjectionFailureCount, *IslandBones,
			ClosedGeometryCount, ClosedReachabilityCount, OpenPortalCount);

		UE_LOG(LogRopeWrap, Error,
			TEXT("[%s] WRAP FAILURE GEOMETRY: simNodes=%d segment=%.2fcm surface=%s normal=%s tangent=%s axisOrigin=%s axisDirection=%s sweepRadial=%s probeRadius=%.2fcm slack=%.2fcm lastPathBone=%s lastPathSurface=%s lastPathDistance=%.2fcm"),
			*OwnerName, Sim.Num(), Sim.SegmentLength, *State.PathSurfaceWorld.ToString(),
			*State.PathNormalWorld.ToString(), *State.PathTangentWorld.ToString(),
			*State.PathAxisOrigin.ToString(), *State.PathAxisDirection.ToString(),
			*State.PathCompositeSweepRadial.ToString(), State.PathCompositeProbeRadius,
			State.PathAvailableSlack,
			LastPathPoint ? *LastPathPoint->Bone.ToString() : TEXT("None"),
			LastPathPoint ? *LastPathPoint->SurfaceWorld.ToString() : TEXT("None"),
			LastPathPoint ? LastPathPoint->DistanceFromLatch : 0.0f);
	}
}

#pragma endregion Wrapping_Debug_And_Diagnostics

#pragma region Gameplay_Debugger_Snapshot

#if WITH_GAMEPLAY_DEBUGGER
namespace
{
	// 컨벡스 헐 와이어프레임 엣지를 바디-로컬 평면 집합에서 계산(디버그 그리기 전용). 평면-쌍 클리핑:
	// 두 면(i,j)의 교선을 나머지 halfspace로 클립해 [tmin,tmax] 구간이 남으면 그게 실제 헐 엣지다(인접
	// 면 쌍만 비어있지 않게 남음). 로컬 엣지 끝점을 강체(Rot*p+Trans)로 월드 변환해 OutWorldEdges에 쌍으로
	// 추가한다. 평면 규약: PlaneDot(p)=dot(N,p)-W, 내부는 모든 면에서 <0(halfspace dot(N,p)<=W). O(평면^3)이나
	// 평면 상한 32 + 디버그 대상 1액터라 무해.
	void BuildConvexHullEdges(const TConstArrayView<FPlane>& Planes, const FQuat& Rot, const FVector& Trans,
		TArray<FVector>& OutWorldEdges)
	{
		const int32 N = Planes.Num();
		for (int32 i = 0; i < N; ++i)
		{
			const FVector Ni(Planes[i].X, Planes[i].Y, Planes[i].Z);
			const double  Wi = Planes[i].W;
			for (int32 j = i + 1; j < N; ++j)
			{
				const FVector Nj(Planes[j].X, Planes[j].Y, Planes[j].Z);
				const double  Wj = Planes[j].W;
				const FVector Dir = FVector::CrossProduct(Ni, Nj);
				const double  DirLenSq = Dir.SizeSquared();
				if (DirLenSq < 1e-8)
				{
					// 평행 면 — 교선 없음.
					continue;
				}
				// 교선 위 한 점 p0 = (Wi·(Nj×Dir) + Wj·(Dir×Ni)) / |Dir|² — 두 평면 교선의 표준 점 공식.
				// (외적 인자 순서가 load-bearing: 뒤바뀌면 P0가 반사돼 비대칭 컨벡스에서 엣지가 대량 누락된다.)
				const FVector P0 = (FVector::CrossProduct(Nj, Dir) * Wi + FVector::CrossProduct(Dir, Ni) * Wj) / DirLenSq;

				// 나머지 평면으로 무한선을 클립: dot(N_k, p0 + t*Dir) <= W_k.
				double TMin = -DBL_MAX, TMax = DBL_MAX;
				bool bValid = true;
				for (int32 k = 0; k < N; ++k)
				{
					if (k == i || k == j)
					{
						continue;
					}
					const FVector Nk(Planes[k].X, Planes[k].Y, Planes[k].Z);
					const double  Denom = FVector::DotProduct(Nk, Dir);
					// W_k - N_k·p0
					const double  Num = static_cast<double>(Planes[k].W) - FVector::DotProduct(Nk, P0);
					if (FMath::Abs(Denom) < 1e-8)
					{
						// 선이 이 면 바깥 → 엣지 없음.
						if (Num < -1e-6) { bValid = false; break; }
						// 선이 면과 평행하고 안쪽 — 제약 없음.
						continue;
					}
					const double T = Num / Denom;
					if (Denom > 0.0) { TMax = FMath::Min(TMax, T); }
					else             { TMin = FMath::Max(TMin, T); }
				}
				if (!bValid || TMin >= TMax - 1e-4)
				{
					// 인접 면이 아니거나 구간 소멸 — 헐 엣지 아님.
					continue;
				}
				const FVector L0 = P0 + Dir * TMin;
				const FVector L1 = P0 + Dir * TMax;
				// 로컬 → 월드(강체).
				OutWorldEdges.Add(Rot.RotateVector(L0) + Trans);
				OutWorldEdges.Add(Rot.RotateVector(L1) + Trans);
			}
		}
	}
}

void URopeComponent::FillDebugSnapshot(FRopeDebugSnapshot& Snapshot, ERopeDebugCapture CaptureMask) const
{
	Snapshot.Phase = Phase;
	Snapshot.PhaseAtFrameStart = DebugPhaseAtFrameStart;
	// centerline 위치는 [P]nodes/[U]flight/[I]wrap 오버레이만 읽는다 — 기본(aim만) 상태에선 헤더 nodes=만
	// 필요하므로 전체 배열을 복사하지 않고 개수 스칼라만 담아 매 프레임 배열 할당을 없앤다.
	Snapshot.NodeCount = Sim.Positions.Num();
	if (EnumHasAnyFlags(CaptureMask, ERopeDebugCapture::Nodes | ERopeDebugCapture::Flight | ERopeDebugCapture::Wrap))
	{
		Snapshot.Positions = Sim.Positions;
	}

	// 정체성과 모드는 어느 보기를 켜든 헤더가 낸다 — 질의가 아니라 이름 복사라 게이트를 두지 않는다.
	Snapshot.ComponentName = GetName();
	Snapshot.OwnerActorName = GetOwner() ? GetOwner()->GetName() : TEXT("None");
	Snapshot.ResolveMode = ResolveMode;

	// 헤더 표시용 프레임 상태 — 화면이 라이브 대신 여기서 읽어 한 시간 기준을 유지한다.
	// wrapBone은 Wrapped 전용 블록과 달리 phase 무관하게 담는다(헤더가 항상 낸다).
	Snapshot.WrapBoneName = WrapController.State.BoneName;
	Snapshot.bSleeping = Throttle.IsAsleep();
	Snapshot.LodScale = Throttle.GetSolverLODScale();
	Snapshot.NumParticles = NumParticles;
	Snapshot.TubeSmoothingSubdiv = TubeSmoothingSubdiv;
	Snapshot.NodeCollisionRadius = GetEffectiveCollisionRadius();
	Snapshot.bSolveThisFrame = SimFrame.bSolveThisFrame;
	Snapshot.bGpuStepped = SimFrame.bGpuSteppedThisFrame;
	Snapshot.bLogicOverride = SimFrame.OverrideFrame.HasAny();

	// centerline 상에서 강조할 latch 노드 인덱스. [P] nodes의 latch 강조와 [I] wrap만 읽으므로,
	// 기본(aim만) 상태에서는 순회·복사할 이유가 없다.
	const FRopeWrapState& Wrap = WrapController.State;
	Snapshot.LatchedNodes.Reset();
	if (EnumHasAnyFlags(CaptureMask, ERopeDebugCapture::Nodes | ERopeDebugCapture::Wrap))
	{
		Snapshot.LatchedNodes.Reserve(Wrap.Latched.Num());
		for (const FRopeLatchNode& Latch : Wrap.Latched)
		{
			Snapshot.LatchedNodes.Add(Latch.NodeIndex);
		}
	}

	// wrapped 상세(테이블용)는 Wrapped phase + [I] wrap 보기일 때만.
	if (EnumHasAnyFlags(CaptureMask, ERopeDebugCapture::Wrap) && Phase == ERopePhase::Wrapped && Wrap.IsWrapped())
	{
		Snapshot.bHasWrapped = true;
		const USceneComponent* Mesh = Wrap.Mesh.Get();
		Snapshot.MeshName = Mesh ? Mesh->GetName() : TEXT("None");
		Snapshot.Latched = Wrap.Latched;
		Snapshot.WrapTension = Wrap.Tension;
		Snapshot.TensionReleaseForce = HoldConfig.TensionReleaseForce;
		// 자동 해제의 실효 여부 — 임계치 값과는 별개다. 판정식을 여기 한 곳에만 두고 화면은 이 불리언만
		// 읽어, CheckWrappedAutoRelease의 모드 게이트와 표시가 갈라지지 않게 한다.
		Snapshot.bAutoReleaseEnabled = (ResolveMode != ERopeWrapResolveMode::GuaranteedWrap);
		Snapshot.TensionOverTime = TensionOverTime;
		Snapshot.TensionReleaseTime = HoldConfig.TensionReleaseTime;
		Snapshot.bPullValid = PullDrive.LastPullSample.bValid;
		Snapshot.PullPoint = PullDrive.LastPullSample.WorldPoint;
		// 스무딩된(실제 인가) 방향
		Snapshot.PullDirection = PullDrive.LastPullSample.Direction;
		// 스무딩 전 look-ahead(지터 진단)
		Snapshot.PullDirRaw = PullDrive.LastPullDirRaw;
		// raw 정수 조준(홉 진단용 텍스트)
		Snapshot.PullAimNode = PullDrive.LastPullSample.AimNode;
		// 청록 = 스무딩된 fractional 조준 위치(방향 EMA의 입력). 실제 인가 방향은 위 PullDirection이다.
		Snapshot.PullAimPoint = PullDrive.LastPullSample.bValid ? PullDrive.LastPullSample.AimPos
			: PullDrive.LastPullSample.WorldPoint;
		Snapshot.PullTension = PullDrive.LastPullSample.Tension;
		Snapshot.TetherOvershoot = PullDrive.LastTetherOvershoot;
		Snapshot.TetherTension = GetTetherTension();
		Snapshot.MaxTetherTension = HoldConfig.MaxTetherTension;
		Snapshot.ActivePullForce = PullDrive.ActivePullForce;
		Snapshot.bActivePullApplied = DebugActivePullPassedGate;
		Snapshot.bPullTaut = PullDrive.bPullTaut;
		Snapshot.bChainTaut = PullDrive.bChainTaut;
		Snapshot.TautChordLen = PullDrive.LastPullSample.TautChordLen;
		Snapshot.FreeRestLen = PullDrive.LastPullSample.FreeRestLen;
		Snapshot.MinFreeTension = PullDrive.LastPullSample.MinFreeTension;
		Snapshot.MaxLegSag = PullDrive.LastPullSample.MaxLegSag;
		Snapshot.DistanceReleaseSlack = HoldConfig.DistanceReleaseSlack;
	}

	// 감김 축 시각화: Wrapping 페이즈에서 ResolveWrappingAxis가 정한 경로 축(원점+방향)을 담는다 —
	// [I] wrap 뷰가 선으로 그려 "이번 wrap이 어느 축으로 감기는지"를 눈으로 확인하게 한다.
	if (EnumHasAnyFlags(CaptureMask, ERopeDebugCapture::Wrap)
		&& Phase == ERopePhase::Wrapping && WrappingPhase.State.IsActive())
	{
		Snapshot.bHasWrapAxis = true;
		Snapshot.WrapAxisOrigin = WrappingPhase.State.PathAxisOrigin;
		Snapshot.WrapAxisDirection = WrappingPhase.State.PathAxisDirection;
		Snapshot.WrapAxisSegmentLength = Sim.SegmentLength;
	}

	// colliders 개수는 별도 필드 없이 아래 Snapshot.Colliders 배열 크기가 단일 소스다([O] 섹션 표시용).

	// 이 로프가 이번 프레임 질의한 collider 시각화(provider bDrawDebug 대체). 상호 배타 accessor 순서로
	// 실제 형상 분류: 캡슐(세그먼트) / 박스(회전 OBB) / 컨벡스(헐 와이어) / 그 외(SDF 등 월드 AABB 폴백).
	// FrameColliders는 provider 소유라 이 프레임 동안만 유효(GT Phase-3 직렬 실행이라 스레딩 무관).
	// [O] colliders 보기일 때만 — 형상 사본과 컨벡스 헐 엣지 재구성(O(plane³) + 매 프레임 배열 할당)이
	// 여기 전부 들어 있어, 보기가 꺼져 있으면 통째로 건너뛰는 것이 이 게이트의 요지다.
	Snapshot.Colliders.Reset();
	if (EnumHasAnyFlags(CaptureMask, ERopeDebugCapture::Colliders))
	for (const IRopeCollider* Collider : SimFrame.FrameColliders)
	{
		if (!Collider)
		{
			continue;
		}
		FRopeDebugCollider DC;
		DC.bWorldStatic = Collider->IsWorldStatic();

		// 귀속(본+메시) 한 번으로 두 값을 낸다.
		//  - bWrapTarget : 정적 메시 랩 대상(URopeWrapTargetComponent가 서빙). 가상 본은 있지만
		//                  SourceMesh가 스켈레탈이 아니다. 개수 표시(staticWrapTargets)용.
		//  - bWrapAllowed: 실제로 감길 수 있는가. 후보 산출(RemoveNonWrappableCandidates)과 같은
		//                  CanWrapTarget 게이트를 태워 표시와 판정을 한 기준으로 맞춘다.
		{
			FName AttribBone = NAME_None;
			const USceneComponent* AttribMesh = nullptr;
			Collider->GetGPUAttribution(AttribBone, AttribMesh);
			const bool bAttributed = !AttribBone.IsNone() && AttribMesh != nullptr;
			DC.bWrapTarget = bAttributed && !RopeWrapTargets::IsSkeletalTarget(AttribMesh);
			DC.bWrapAllowed = bAttributed && CanWrapTarget(AttribMesh, AttribBone);
		}

		TConstArrayView<FPlane> LocalPlanes;
		FBox LocalBounds(ForceInit);
		FQuat CvRot, CvPrevRot;
		FVector CvTrans, CvPrevTrans;
		float CvInvDt = 0.0f;
		if (Collider->GetGPUCapsule(DC.A, DC.B, DC.Radius))
		{
			DC.Shape = ERopeDebugColliderShape::Capsule;
		}
		else if (Collider->GetGPUBox(DC.Center, DC.Rot, DC.HalfExtents))
		{
			DC.Shape = ERopeDebugColliderShape::Box;
		}
		else if (Collider->GetGPUConvex(LocalPlanes, LocalBounds, CvRot, CvTrans, CvPrevRot, CvPrevTrans, CvInvDt)
			&& LocalPlanes.Num() >= 4)
		{
			DC.Shape = ERopeDebugColliderShape::Convex;
			BuildConvexHullEdges(LocalPlanes, CvRot, CvTrans, DC.ConvexEdges);
		}
		else
		{
			DC.Shape = ERopeDebugColliderShape::Bounds;
			DC.Bounds = Collider->GetWorldBounds();
		}
		Snapshot.Colliders.Add(MoveTemp(DC));
	}

	// 노드별 근접 재질의(디버그 전용): post-solve 노드 위치를 FrameColliders에 다시 질의해 각 노드가 어느 면을
	// 마주하는지(법선)를 기록한다. GPU 런타임은 접촉을 리드백하지 않으므로 여기서 CPU로 다시 질의한다. 질의
	// 반경 = CollisionRadius + 여유라 정착(표면에서 ~반경 떨어져 쉬는) 노드도 잡힌다 — solver가 실제로 처리한
	// 접촉 집합이 아니라는 뜻이고, 화면도 그 여유를 함께 밝힌다. 노드당 가장 깊은 것 1개만.
	// 노드 수 × collider 수의 CPU Query라 캡처 항목 중 가장 비싸다 — [P] nodes 보기일 때만 돈다.
	constexpr float ProximityQueryMargin = 4.0f;
	Snapshot.NodeProximity.Reset();
	Snapshot.ProximityQueryMargin = ProximityQueryMargin;
	const float DebugQueryRadius = GetEffectiveCollisionRadius() + ProximityQueryMargin;
	const int32 ProximityNodeCount = EnumHasAnyFlags(CaptureMask, ERopeDebugCapture::Nodes)
		? Sim.Positions.Num() : 0;
	for (int32 i = 0; i < ProximityNodeCount; ++i)
	{
		const FVector NodePos = Sim.Positions[i];
		FRopeContact Best;
		bool bAny = false;
		// 이긴 접촉을 낸 collider의 정적 여부를 함께 들고 간다(FRopeContact에는 없는 정보라 여기서 보존).
		bool bBestWorldStatic = false;
		for (const IRopeCollider* Collider : SimFrame.FrameColliders)
		{
			if (!Collider)
			{
				continue;
			}
			const FRopeContact C = Collider->Query(NodePos, DebugQueryRadius);
			if (C.bHit && (!bAny || C.Penetration > Best.Penetration))
			{
				Best = C;
				bBestWorldStatic = Collider->IsWorldStatic();
				bAny = true;
			}
		}
		if (bAny)
		{
			FRopeNodeProximityDebug NP;
			NP.NodeIndex = i;
			NP.Position = NodePos;
			NP.Normal = Best.Normal;
			NP.Bone = Best.Bone;
			NP.bWorldStatic = bBestWorldStatic;
			Snapshot.NodeProximity.Add(MoveTemp(NP));
		}
	}
}
#endif

#pragma endregion Gameplay_Debugger_Snapshot

#pragma region Flight_Observation

void URopeComponent::RecordFlightObservation(const FRopeFlightContactDetector::FParams& DetectParams,
	const TArray<FRopeContactCandidate>& Candidates, const FRopeContactTracker& FrameTracker,
	bool bShouldCapture, FRopeDebugSnapshot* OutSnapshot)
{
	// ③ 관측 전용 — 판정(①②)에 관여하지 않는 읽기 소비만 모아둔다. 스탯은 stat 시스템이 수집 중일
	// 때만 실제 비용이 들고, 스냅샷은 디버거 대상 로프만 OutSnapshot으로 넘어온다(그 외 null).
	const bool bNeedStats = RopeDebug::IsFlightStatEnabled();
	const bool bNeedSnapshot = OutSnapshot != nullptr;
	if (!bNeedStats && !bNeedSnapshot)
	{
		return;
	}

	const float WhipGuidedEnd = FMath::Clamp(WhipConfig.GuidedLength, 0.05f, 0.95f);
	int32 WhipGuidedNodeCount = 0;
#if WITH_GAMEPLAY_DEBUGGER
	if (OutSnapshot)
	{
		WhipGuide.CopyGuidedTargetsForDebug(
			OutSnapshot->WhipGuideNodeIndices, OutSnapshot->WhipGuideTargets);
		WhipGuidedNodeCount = OutSnapshot->WhipGuideNodeIndices.Num();
	}
#endif
	if (WhipGuidedNodeCount == 0 && bNeedStats)
	{
		WhipGuidedNodeCount = WhipGuide.GetGuidedNodeCountThisFrame();
	}
	const bool bWhipActive = WhipGuidedNodeCount > 0;
	RopeDebug::RecordFlightStats(Sim, SimFrame.bSolveThisFrame, SimFrame.FrameColliders.Num(), Candidates,
		FrameTracker, bShouldCapture);
	RopeDebug::RecordWhipStats(Sim, WhipGuidedNodeCount, WhipGuidedEnd);

#if WITH_GAMEPLAY_DEBUGGER
	if (OutSnapshot)
	{
		GatherFlightNodeDebug(DetectParams, OutSnapshot->NodeDebug);
		OutSnapshot->bHasFlight = true;
		// bSolveThisFrame / colliders 수는 FillDebugSnapshot(항상 실행)이 단일 소스로 채운다 — 여기선 안 쓴다.
		// 캡처 판정값(bShouldCapture / MinLatchNodes / TrackerNodes)은 담지 않는다 — 사유는 스냅샷 구조체 주석.
		OutSnapshot->TrackerBone = FrameTracker.CandidateBone;
		OutSnapshot->Candidates = Candidates;
		// 대상 식별은 (Mesh, Bone) 쌍이다(FRopeContactTracker 계약). 메시 포인터는 스냅샷 수명(수 프레임)
		// 뒤 죽어 있을 수 있으므로 비교 전용 키로 지금 굳힌다.
		OutSnapshot->TrackerMeshKey = FObjectKey(FrameTracker.CandidateMesh);
		OutSnapshot->CandidateMeshKeys.Reset(Candidates.Num());
		for (const FRopeContactCandidate& Candidate : Candidates)
		{
			OutSnapshot->CandidateMeshKeys.Add(FObjectKey(Candidate.Mesh));
		}
		OutSnapshot->bWhipActive = bWhipActive;
		OutSnapshot->WhipGuidedEnd = WhipGuidedEnd;
	}
#endif
}

#if WITH_GAMEPLAY_DEBUGGER
void URopeComponent::GatherFlightNodeDebug(const FRopeFlightContactDetector::FParams& DetectParams,
	TArray<FRopeFlightNodeDebug>& OutNodeDebug) const
{
	// 디버거 대상 로프 전용 시각화 수집. 본 감지 파이프라인과 별개로 노드마다 감지기를 재질의하는
	// 의도된 중복 — 판정에 안 걸린 노드(느림/원거리)의 "왜 안 걸렸나"까지 보여주는 것이 목적이라
	// 판정 산출물 재사용으로는 대체가 안 된다. 비용은 디버거 대상 1개 로프만 부담.
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightDebugGather);
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		if (!Sim.PrevPositions.IsValidIndex(i) || !Sim.Positions.IsValidIndex(i))
		{
			continue;
		}

		FRopeFlightNodeDebug NodeDebug;
		NodeDebug.NodeIndex = i;
		NodeDebug.PrevPosition = Sim.PrevPositions[i];
		NodeDebug.Position = Sim.Positions[i];
		const bool bFast = FRopeFlightContactDetector::IsTailNode(Sim, i) || Sim.NodeSpeed(i) > Sim.SegmentLength;
		NodeDebug.bNearBody = FRopeFlightContactDetector::IsNearAnyColliderSegment(
			NodeDebug.PrevPosition, NodeDebug.Position, SimFrame.FrameColliders, DetectParams);
		if (bFast || NodeDebug.bNearBody)
		{
			NodeDebug.Contact = FRopeFlightContactDetector::SweepOrSampleContact(
				Sim, NodeDebug.PrevPosition, NodeDebug.Position, SimFrame.FrameColliders, DetectParams);
			// 같은 노드라도 후보와 다른 대상에 닿은 접촉일 수 있다. 대상 일치 판정용 키를 지금 굳힌다
			// (스냅샷 수명이 지나면 raw 포인터는 역참조할 수 없다).
			NodeDebug.ContactMeshKey = FObjectKey(NodeDebug.Contact.SourceMesh);
		}

		if (bFast || NodeDebug.bNearBody || NodeDebug.Contact.bHit)
		{
			OutNodeDebug.Add(NodeDebug);
		}
	}
}
#endif // WITH_GAMEPLAY_DEBUGGER

#pragma endregion Flight_Observation

