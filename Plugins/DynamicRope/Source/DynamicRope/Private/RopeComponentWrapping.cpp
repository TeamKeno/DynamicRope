// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeComponent.h"

#include "Core/RopeWrapTarget.h"
#include "DynamicRopeLog.h"
#include "Engine/World.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"
#include "RopeComponentInternal.h"
#include "RopeMathHelpers.h"
#include "Subsystem/RopeSimSubsystem.h"

using RopeComponentPrivate::LogWrappingFailureState;
using RopeComponentPrivate::ReleaseCooldownSeconds;

#pragma region File_Local_Helpers

namespace
{
	constexpr float KinematicBridgeStretchWarningRatio = 1.25f;

	const FRopeSurfaceAnchor* FindSurfaceAnchorByNode(
		const TArray<FRopeSurfaceAnchor>& Anchors, int32 NodeIndex)
	{
		return Anchors.FindByPredicate(
			[NodeIndex](const FRopeSurfaceAnchor& Anchor)
			{
				return Anchor.NodeIndex == NodeIndex;
			});
	}

	bool ResolveAnchorCenterlineWorld(const FRopeSurfaceAnchor& Anchor, FVector& OutWorld)
	{
		const USceneComponent* Mesh = Anchor.Mesh.Get();
		if (!Mesh)
		{
			return false;
		}

		const FTransform BindingWorld = ResolveBindingWorld(Mesh, Anchor.Bone);
		const FVector SurfaceWorld =
			BindingWorld.TransformPosition(Anchor.LocalSurfacePosition);
		const FVector NormalWorld =
			BindingWorld.TransformVectorNoScale(Anchor.LocalNormal)
			.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		OutWorld = SurfaceWorld + NormalWorld * Anchor.SurfaceOffset;
		return true;
	}

}

#pragma endregion File_Local_Helpers_And_Debug

#pragma region Wrapping_Public_API

void URopeComponent::FinishWrapRelease(FName Bone, ERopeReleaseReason Reason, const FString& ReasonLog)
{
	// 모든 release 트리거(수동/절단/장력/거리/대상 소실)의 공용 마무리: 페이즈 전환 + 노드 반환 +
	// 일시 상태 폐기 + 쿨다운 + 이벤트. 사유별 차이는 호출자에서 끝난 상태로 들어온다.
	// 감겼던 mesh는 Release/Reset이 wrap 상태를 비우기 전에 캡처한다 — 중앙 release 신호가 실어 보내
	// 대상 반응 컴포넌트가 "내 메시가 풀렸나"를 판별하게 한다(release BP 델리게이트는 mesh 미포함).
	const USceneComponent* WrappedMesh = WrapController.State.Mesh.Get();
	// ReleaseWrapAs는 Wrapped뿐 아니라 Contacting/Wrapping/GuidedThrow(전부 커밋 전)에서도 들어온다 —
	// 커밋 전이면 중앙 신호를 쏘면 안 된다(아래 DispatchReleased 주석: 다른 로프가 감아 랙돌시킨 대상을
	// 이 로프의 abort가 잘못 복구시킨다). WrapController.Release가 상태를 비우기 전에 잡는다.
	const bool bWasWrapped = WrapController.IsActive();
	SetPhase(ERopePhase::Releasing, *ReasonLog);
	WrapController.Release(Reason);
	ReleaseKinematicVirtualBridgesToSolver();
	ResetTransientPhaseState();
	ReleaseCooldown = ReleaseCooldownSeconds;
	DispatchReleased(WrappedMesh, Bone, Reason, bWasWrapped);
	// 팁 부착물은 여기서 파괴하지 않는다 — 수명은 전 모드 BeginPlay~EndPlay다(2026-07-17 모드 무관화).
	// 종전엔 ①②만 release에 파괴했으나, 그러면 Free에 팁이 없어 bSyncTipMeshOnFree가 무의미해진다.
}

void URopeComponent::FinishPreCommitReleaseToFlight(FName Bone, const TCHAR* PhaseLog)
{
	// 성립 전(Captured~Wrapping) 이탈의 공용 마무리 — 정리(Flight 전이 + 일시 상태 폐기) 후에 통지한다.
	// 정리 전에 쏘면(구 버그) 아직 Contacting인 게이트를 핸들러의 ReleaseWrap()이 통과해 이중 release +
	// 핸들러가 확정한 Releasing을 곧바로 이 SetPhase(Flight)가 덮어썼다(AbortGuidedThrow와 같은 계약으로 통일).
	// Bone은 인자로 값 캡처되어 ResetTransientPhaseState(트래커 비움) 이후에도 유효하다. 커밋 전이라
	// bWasWrapped=false — 중앙 신호(OnAnyRopeReleased) 없이 per-instance만(다른 로프가 감은 대상 오복구 방지).
	SetPhase(ERopePhase::Flight, PhaseLog);
	ReleaseKinematicVirtualBridgesToSolver();
	ResetTransientPhaseState();
	DispatchReleased(nullptr, Bone, ERopeReleaseReason::Broken, /*bWasWrapped*/ false);
}

void URopeComponent::DispatchReleased(const USceneComponent* WrappedMesh, FName Bone, ERopeReleaseReason Reason, bool bWasWrapped)
{
	// Wrapped 통지가 아직 진행 중이면(핸들러가 그 안에서 ReleaseWrap을 불렀다) 통지를 미룬다 —
	// 지금 쏘면 구독자가 Released를 Wrapped보다 먼저 받는다(FDeferredReleaseNotice 주석 참고).
	// 상태는 이미 정리된 뒤이므로 미루는 것은 통지뿐이고, 큐는 Wrapped 통지가 끝나는 즉시 비워진다.
	if (WrappedDispatchDepth > 0)
	{
		FDeferredReleaseNotice& Notice = DeferredReleaseNotices.AddDefaulted_GetRef();
		Notice.WrappedMesh = const_cast<USceneComponent*>(WrappedMesh);
		Notice.Bone = Bone;
		Notice.Reason = Reason;
		Notice.bWasWrapped = bWasWrapped;
		return;
	}

	// per-instance: Captured/Wrapped로 시작된 engagement의 종료를 항상 알린다(짝 맞춤).
	NotifyReleased(Bone, Reason);
	OnRopeReleased.Broadcast(Bone, Reason);
	// 중앙 신호는 OnAnyRopeWrapped와 짝 — 실제 성립(Wrapped)이 있었을 때만. 성립 전 abort(bWasWrapped=false)에서
	// 쏘면, 다른 로프가 감아 랙돌시킨 같은 대상을 이 로프의 abort가 잘못 복구시킨다.
	if (bWasWrapped)
	{
		if (URopeSimSubsystem* SimSubsystem = URopeSimSubsystem::Get(GetWorld()))
		{
			// 로프 자신을 함께 싣는다 — 대상을 여러 로프가 감았을 때 구독자가 "내 engagement 중 어느
			// 것이 끝났나"를 mesh만으로는 구분할 수 없다(FRopeWrappedEventInfo::Rope와 짝).
			SimSubsystem->OnAnyRopeReleased.Broadcast(this, WrappedMesh, Bone, Reason);
		}
	}
}

void URopeComponent::ReleaseWrap()
{
	ReleaseWrapAs(ERopeReleaseReason::Manual);
}

void URopeComponent::CutRope()
{
	ReleaseWrapAs(ERopeReleaseReason::Cut);
}

void URopeComponent::ReleaseWrapAs(ERopeReleaseReason Reason)
{
	if (Phase != ERopePhase::Wrapped && Phase != ERopePhase::Contacting && Phase != ERopePhase::Wrapping &&
		Phase != ERopePhase::GuidedThrow)
		return;

	FName Bone = NAME_None;

	if (Phase == ERopePhase::Wrapped)
	{
		Bone = WrapController.State.BoneName;
	}
	else if (Phase == ERopePhase::Wrapping)
	{
		Bone = WrappingPhase.State.BoneName;
	}
	else if (Phase == ERopePhase::GuidedThrow)
	{
		Bone = GuidedThrowState.Prepared.Bone;
	}
	else
	{
		Bone = ContactTracker.CandidateBone;
	}

	FinishWrapRelease(Bone, Reason, FString::Printf(TEXT("%s, bone=%s"),
		Reason == ERopeReleaseReason::Cut ? TEXT("cut") : TEXT("manual"), *Bone.ToString()));
}

#pragma endregion Wrapping_Public_API

#pragma region Contacting

// ===== Contacting ===========================================================

void URopeComponent::UpdateContacting(float DeltaTime)
{
	// 총 체류(아래 정체 안전망 판단용 — 감김 판정 자체는 트래커 dwell).
	ContactingElapsed += DeltaTime;

	// 매 프레임 실제 접촉을 재수집한다 — 캡처 순간의 1회 스냅샷만 믿고 타이머를 돌리던 이전 구조는
	// (1) dismiss가 사실상 불발이었고(트래커 미갱신) (2) 움직이는 대상(랙돌/드래곤)에서 시드와 실제
	// 지오메트리의 어긋남이 WrapDecisionTime 동안 누적됐다. Contacting은 솔브가 없어 노드가 정지
	// 상태라 스윕은 점 질의로 축퇴하고, 대상 이탈은 collider 쪽 이동으로 감지된다.
	// 예측/whip 분기는 Flight 전용이므로 여기서는 actual 접촉만 수집한다(비용: 근접 노드 점 질의뿐).
	const FRopeFlightContactDetector::FParams DetectParams = MakeFlightDetectParams(DeltaTime);
	TArray<FRopeContactCandidate>& Candidates = ContactCandidateScratch;
	Candidates.Reset();
	FRopeFlightContactDetector::DetectContactCandidates(Sim, SimFrame.FrameColliders, DetectParams, Candidates);
	FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, DetectParams, Candidates);
	// CanWrapTarget 게이트(Flight 후보 산출과 공용 헬퍼).
	RemoveNonWrappableCandidates(Candidates);

	// 트래커 갱신: 같은 본이면 dwell 누적, 지배 본이 바뀌면 dwell 리셋(전이 프레임 오탐 방어 —
	// dwell 재시작 계약을 캡처 후 구간에도 실제로 적용), 접촉이 끊기면 dwell이 소진되며 트래커가
	// 비워져 아래 dismiss로 떨어진다(짧은 플리커는 그동안 쌓인 dwell만큼 관용).
	const bool bRequireAimPrimary = ResolveMode == ERopeWrapResolveMode::AssistedJudged
		&& AimTargeting.IsLockActive(Phase);
	ContactTracker.Update(Candidates, DeltaTime,
		bRequireAimPrimary ? AimTargeting.GetLockedTargetMesh() : nullptr,
		bRequireAimPrimary ? AimTargeting.GetLockedTargetBone() : NAME_None,
		bRequireAimPrimary);

	if (ShouldDismissContacting())
	{
		UE_LOG(LogRopeWrap, Warning,
			TEXT("[%s] Contacting dismissed before wrap: reason=ContactTrackerEmpty candidates=%d trackerBone=%s trackerNodes=%d dwell=%.3fs required=%.3fs elapsed=%.3fs colliders=%d"),
			*GetName(), Candidates.Num(), *ContactTracker.CandidateBone.ToString(),
			ContactTracker.CandidateNodes.Num(), ContactTracker.DwellTime,
			DetectConfig.WrapDecisionTime, ContactingElapsed, SimFrame.FrameColliders.Num());
		// 성립 전 이탈 — 정리 후 per-instance 통지(FinishPreCommitReleaseToFlight: 재진입 계약).
		FinishPreCommitReleaseToFlight(ContactTracker.CandidateBone, TEXT("contact lost before wrapping"));
		return;
	}

	// 시드 갱신: Wrapping이 시작되는 프레임의 최신 접촉 지오메트리에서 경로 생성이 출발하게 한다.
	if (Candidates.Num() > 0 && !ContactTracker.CandidateBone.IsNone())
	{
		PendingWrapSeed = BuildWrapSeedFromContactingState(Candidates);
	}

	if (ShouldStartWrapping())
	{
		StartWrappingFromContacting();
		return;
	}

	// 정체 안전망: 접촉이 깜빡여 dwell이 임계에 못 미친 채 오래 머물면(커밋도 dismiss도 안 됨)
	// Flight로 돌려보낸다. Flight에서 재캡처는 자유이므로 잃는 것 없이 무한 체류만 막는다.
	const float StallTimeout = FMath::Max(DetectConfig.WrapDecisionTime * 10.0f, 1.0f);
	if (ContactingElapsed >= StallTimeout)
	{
		UE_LOG(LogRopeWrap, Warning,
			TEXT("[%s] Contacting stalled before wrap: candidates=%d trackerBone=%s trackerNodes=%d targets=%d dwell=%.3fs required=%.3fs elapsed=%.3fs timeout=%.3fs"),
			*GetName(), Candidates.Num(), *ContactTracker.CandidateBone.ToString(),
			ContactTracker.CandidateNodes.Num(), ContactTracker.Targets.Num(),
			ContactTracker.DwellTime, DetectConfig.WrapDecisionTime, ContactingElapsed, StallTimeout);
		// 성립 전 이탈 — 정리 후 per-instance 통지(FinishPreCommitReleaseToFlight).
		FinishPreCommitReleaseToFlight(ContactTracker.CandidateBone,
			*FString::Printf(TEXT("contacting stalled %.2fs (dwell %.2fs < %.2fs)"),
				ContactingElapsed, ContactTracker.DwellTime, DetectConfig.WrapDecisionTime));
	}
}

bool URopeComponent::ShouldDismissContacting() const
{
	return ContactTracker.CandidateBone.IsNone() || ContactTracker.CandidateNodes.Num() == 0;
}

bool URopeComponent::ShouldStartWrapping() const
{
	// 판정은 "한 본과의 지속 접촉"(트래커 dwell — 지배 본이 바뀌면 0부터) 기준. 총 경과가 아니라
	// dwell을 쓰는 것이 원 설계 의도(노드들이 WrapDecisionTime 동안 한 본에 유지)와 일치한다.
	// 안정 접촉에서는 dwell == 총 경과라 기존과 동일하고, 본이 튀는 전이 프레임에서만 엄격해진다.
	return ContactTracker.DwellTime >= DetectConfig.WrapDecisionTime
		&& PendingWrapSeed.Latched.Num() > 0
		&& !PendingWrapSeed.BoneName.IsNone();
}

FRopeWrapState URopeComponent::BuildWrapSeedFromContactingState(const TArray<FRopeContactCandidate>& Candidates) const
{
	FRopeWrapState Seed;
	Seed.BoneName = ContactTracker.CandidateBone;
	Seed.Mesh = ContactTracker.CandidateMesh;
	const int32 NodeIndex = RopeMath::HeadValidNodeIndex(ContactTracker.CandidateNodes, Sim.Positions);
	if (NodeIndex == INDEX_NONE)
	{
		return Seed;
	}

	// dominant 시드: 시드의 [0]번 latch/anchor 자리다(StartWrappingFromContacting이 [0]을 경로
	// 빌드 출발점으로 소비하는 계약). anchor 구성이 실패해도 latch는 남긴다(종전 동작).
	{
		FRopeLatchNode Latch;
		FRopeSurfaceAnchor Anchor;
		const USceneComponent* ResolvedMesh = nullptr;
		const bool bAnchorBuilt = BuildSeedLatchForTarget(Candidates,
			ContactTracker.CandidateBone, ContactTracker.CandidateMesh, NodeIndex,
			/*RopeDistance*/ 0.0f, Latch, Anchor, ResolvedMesh);
		Seed.Latched.Add(Latch);
		if (!Seed.Mesh.IsValid() && ResolvedMesh)
		{
			Seed.Mesh = ResolvedMesh;
		}
		if (bAnchorBuilt)
		{
			Seed.Anchors.Add(Anchor);
		}
	}

	// 보조 시드(시드 다중화, MaxWrapSeeds > 1): dominant보다 tail 쪽에서 *다른* (mesh, bone)에
	// dwell을 채운 대상을 추가 시드로 채택한다(예: 양다리 — 반대쪽 다리). head 쪽 대상은 받지
	// 않는다: Wrapping의 경로/마스크가 latch 이후(tail) 구간만 소유하므로 head 쪽 노드는 고정할
	// 통로가 없다. 보조 시드는 anchor까지 만들어졌을 때만 유효하다(경로 없이 본에 hold만 하므로
	// 표면 프레임이 필수). dominant anchor가 없으면 보조도 받지 않는다 — Anchors[0]은 dominant
	// 자리라는 계약(StartWrappingFromContacting의 경로 출발점)이 보조 anchor로 오염되면 안 된다.
	if (WrapConfig.MaxWrapSeeds > 1 && Seed.Anchors.Num() > 0)
	{
		// 노드 간 최소 이격(세그먼트 수): dominant 나선이 쓸 최소 구간을 보장하고, 이웃 노드가
		// 서로 다른 시드로 갈라지는 것을 막는다.
		constexpr int32 MinSeedNodeSeparation = 2;

		TArray<const FRopeTrackedContactTarget*> Sorted;
		for (const FRopeTrackedContactTarget& Target : ContactTracker.Targets)
		{
			const bool bDominant = Target.Bone == ContactTracker.CandidateBone &&
				Target.Mesh == ContactTracker.CandidateMesh;
			if (!bDominant && Target.DwellTime >= DetectConfig.WrapDecisionTime)
			{
				Sorted.Add(&Target);
			}
		}
		// dominant에 가까운(head 쪽) 대상부터 — 프레임 간 안정적 채택 순서.
		Sorted.Sort([this](const FRopeTrackedContactTarget& A, const FRopeTrackedContactTarget& B)
		{
			return RopeMath::HeadValidNodeIndex(A.Nodes, Sim.Positions)
				< RopeMath::HeadValidNodeIndex(B.Nodes, Sim.Positions);
		});

		for (const FRopeTrackedContactTarget* Target : Sorted)
		{
			if (Seed.Latched.Num() >= WrapConfig.MaxWrapSeeds)
			{
				break;
			}

			const int32 SecondaryNode = RopeMath::HeadValidNodeIndex(Target->Nodes, Sim.Positions);
			if (SecondaryNode == INDEX_NONE)
			{
				continue;
			}

			bool bTooClose = false;
			for (const FRopeLatchNode& Existing : Seed.Latched)
			{
				if (SecondaryNode < Existing.NodeIndex + MinSeedNodeSeparation)
				{
					bTooClose = true;
					break;
				}
			}
			if (bTooClose)
			{
				continue;
			}

			FRopeLatchNode Latch;
			FRopeSurfaceAnchor Anchor;
			const USceneComponent* ResolvedMesh = nullptr;
			if (BuildSeedLatchForTarget(Candidates, Target->Bone, Target->Mesh, SecondaryNode,
				static_cast<float>(SecondaryNode - NodeIndex) * Sim.SegmentLength, Latch, Anchor, ResolvedMesh))
			{
				Seed.Latched.Add(Latch);
				Seed.Anchors.Add(Anchor);
			}
		}
	}

	return Seed;
}

bool URopeComponent::BuildSeedLatchForTarget(const TArray<FRopeContactCandidate>& Candidates,
	FName Bone, const USceneComponent* TrackedMesh, int32 NodeIndex, float RopeDistance,
	FRopeLatchNode& OutLatch, FRopeSurfaceAnchor& OutAnchor, const USceneComponent*& OutMesh) const
{
	OutLatch.NodeIndex = NodeIndex;
	OutLatch.Bone = Bone;
	OutMesh = TrackedMesh;

	const FRopeContactCandidate* LatchCandidate = nullptr;
	for (const FRopeContactCandidate& Candidate : Candidates)
	{
		if (!Candidate.bValid ||
			Candidate.NodeIndex != NodeIndex ||
			Candidate.Bone != Bone)
		{
			continue;
		}

		// 같은 본 이름을 쓰는 두 액터가 함께 닿는 프레임의 오귀속 방어(트래커의 (Mesh, Bone) 키와
		// 같은 이유). 트래커 mesh가 없을 때만 후보 mesh를 그대로 받는다(종전 폴백 유지).
		if (TrackedMesh && Candidate.Mesh && Candidate.Mesh != TrackedMesh)
		{
			continue;
		}

		if (!LatchCandidate || Candidate.Penetration > LatchCandidate->Penetration)
		{
			LatchCandidate = &Candidate;
		}
	}

	if (!OutMesh && LatchCandidate)
	{
		OutMesh = LatchCandidate->Mesh;
	}

	if (!LatchCandidate || !OutMesh)
	{
		return false;
	}

	const FVector NormalWorld = LatchCandidate->Normal.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	FVector TangentWorld = FRopeFlightContactDetector::ExpectedWrapTangent(Sim, *LatchCandidate, GetForwardVector());
	if (Sim.Positions.IsValidIndex(NodeIndex + 1))
	{
		TangentWorld = Sim.Positions[NodeIndex + 1] - Sim.Positions[NodeIndex];
	}
	TangentWorld = (TangentWorld - FVector::DotProduct(TangentWorld, NormalWorld) * NormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, RopeMath::AnyTangentFromNormal(NormalWorld));

	const FTransform BoneXform = ResolveBindingWorld(OutMesh, Bone);

	OutAnchor.NodeIndex = NodeIndex;
	OutAnchor.Bone = Bone;
	OutAnchor.Mesh = OutMesh;
	OutAnchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(LatchCandidate->WorldPoint);
	OutAnchor.LocalNormal = BoneXform.InverseTransformVectorNoScale(NormalWorld).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	OutAnchor.LocalTangent = BoneXform.InverseTransformVectorNoScale(TangentWorld).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
	OutAnchor.StartWorldPosition = Sim.Positions[NodeIndex];
	OutAnchor.SurfaceOffset = FMath::Max(0.0f, Radius);
	OutAnchor.RopeDistance = RopeDistance;
	return true;
}

#pragma endregion Contacting

#pragma region Wrapping

// ===== Wrapping =============================================================

void URopeComponent::StartWrappingFromContacting()
{
	// PendingWrapSeed를 바로 BeginWrap에 넣지 않고, WrappingPhase 상태로 변환한다.
	// 새 시도는 이전 Composite virtual run의 점진 scan 상태를 절대 이어받지 않는다.
	ResetKinematicVirtualBridges();
	WrappingPhase.State.Reset();

	// 감길 메시는 접촉에서 확정된다(FRopeContact.SourceMesh → seed). 여기 비어 있으면 시드가
	// 비정상인 것 — owner 메시로 때우면 cross-actor에서 엉뚱한 본에 붙으므로 폴백 없이 복귀한다.
	const USceneComponent* Mesh = PendingWrapSeed.Mesh.Get();
	if (!Mesh || PendingWrapSeed.BoneName.IsNone() || PendingWrapSeed.Latched.Num() == 0)
	{
		UE_LOG(LogRopeWrap, Error,
			TEXT("[%s] WRAP FAILURE: site=StartWrapping reason=InvalidWrappingSeed mesh=%s bone=%s latched=%d anchors=%d trackerBone=%s trackerNodes=%d dwell=%.3fs contacting=%.3fs simNodes=%d"),
			*GetName(), Mesh ? *Mesh->GetName() : TEXT("None"), *PendingWrapSeed.BoneName.ToString(),
			PendingWrapSeed.Latched.Num(), PendingWrapSeed.Anchors.Num(),
			*ContactTracker.CandidateBone.ToString(), ContactTracker.CandidateNodes.Num(),
			ContactTracker.DwellTime, ContactingElapsed, Sim.Num());
		// 성립 전 이탈 — 정리 후 per-instance 통지(FinishPreCommitReleaseToFlight).
		FinishPreCommitReleaseToFlight(ContactTracker.CandidateBone, TEXT("invalid wrapping seed"));
		return;
	}

	// M5c: GPU 상주 로프의 CPU 미러는 1~2프레임 낡다 — wrap 핸드오프 순간만 1회 동기 리드백으로
	// 최신 위치를 받아 시드(StartWorldPosition/fallback 앵커)의 정밀도를 확보한다(이벤트당 1회, 블로킹).
	if (URopeSimSubsystem* SimSubsystem = URopeSimSubsystem::Get(GetWorld()))
	{
		SimSubsystem->SyncGpuPositionsForHandoff(*this);
	}

	// 무조건 첫 번째 latch node 하나만 기준으로 잡는다
	const FRopeLatchNode& Latch = PendingWrapSeed.Latched[0];
	FRopeSurfaceAnchor LatchAnchor;

	// 정상 경로: BuildWrapSeedFromContactingState()가 실제 contact candidate 기반으로
	// surface anchor를 이미 만들어 둔 경우 — 그대로 사용한다.
	if (PendingWrapSeed.Anchors.Num() > 0)
	{
		LatchAnchor = PendingWrapSeed.Anchors[0];
		LatchAnchor.Mesh = Mesh;
	}
	// 비상비상: 아래 fallback은 contact candidate 기반의 정확한 SDF surface anchor가 없을 때만 쓰는 임시 anchor 경로다.
	// 현재 rope particle 위치와 임시 normal/tangent로 시작점을 때우므로, wrapping 품질/방향이 흔들릴 수 있다.
	// 정상 경로는 PendingWrapSeed.Anchors[0]에 실제 contact surface point/normal/tangent가 들어오는 것이다.
	// Contacting이 매 프레임 시드를 최신 후보로 재조립하게 된 뒤로는(개선 2호) 후보가 있는 한 Anchors[0]가
	// 항상 채워져 이 경로는 사실상 도달 불가로 추정된다 — 아래 경고로 실전 도달 여부를 관측한 뒤 제거 후보.
	else if (Sim.Positions.IsValidIndex(Latch.NodeIndex))
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] StartWrapping fell back to the synthetic latch anchor (no contact-based anchor in seed, bone=%s) — wrap quality may wobble. Thought unreachable; report if seen."),
			*GetName(), *Latch.Bone.ToString());

		const FVector NormalWorld = FVector::UpVector;
		FVector TangentWorld = FVector::ForwardVector;

		if (Sim.Positions.IsValidIndex(Latch.NodeIndex + 1))
		{
			// tangent는 가능하면 다음 rope node 방향을 쓴다 — "로프가 tail 방향으로 어느 쪽으로
			// 뻗어 있는가"를 잡기 위한 값으로, 이후 Composite Analytic Helix / Sequential Surface Vector Field에서
			// 감기는 방향(WindingSign)을 정할 때 중요하다.
			TangentWorld = (Sim.Positions[Latch.NodeIndex + 1] - Sim.Positions[Latch.NodeIndex])
				.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
		}

		const FTransform BoneXform = ResolveBindingWorld(Mesh, Latch.Bone);
		LatchAnchor.NodeIndex = Latch.NodeIndex;
		LatchAnchor.Bone = Latch.Bone;
		LatchAnchor.Mesh = Mesh;
		// 현재 latch node 위치를 bone-local surface position처럼 저장하고,
		// normal은 실제 SDF normal이 아니라 임시로 UpVector를 쓴다.
		LatchAnchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(Sim.Positions[Latch.NodeIndex]);
		LatchAnchor.LocalNormal = BoneXform.InverseTransformVectorNoScale(NormalWorld).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		LatchAnchor.LocalTangent = BoneXform.InverseTransformVectorNoScale(TangentWorld).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
		LatchAnchor.StartWorldPosition = Sim.Positions[Latch.NodeIndex];
		LatchAnchor.SurfaceOffset = FMath::Max(0.0f, Radius);
		LatchAnchor.RopeDistance = 0.0f;
	}

	// 시드 다중화: Anchors[0]은 dominant(경로 출발점), [1..]는 보조 시드 앵커다. Begin *전에*
	// 상태에 실어야 경로 빌드가 NumTailNodes를 첫 보조 노드 앞까지로 클램프한다(필드 주석 참고).
	// dominant latch보다 tail 쪽 노드만 유효하다(시드 조립이 보장하지만, 시드가 오래된 프레임일
	// 가능성에 대비해 한 번 더 거른다).
	for (int32 AnchorIndex = 1; AnchorIndex < PendingWrapSeed.Anchors.Num(); ++AnchorIndex)
	{
		const FRopeSurfaceAnchor& Secondary = PendingWrapSeed.Anchors[AnchorIndex];
		if (Secondary.NodeIndex > LatchAnchor.NodeIndex && Sim.Positions.IsValidIndex(Secondary.NodeIndex))
		{
			WrappingPhase.State.SecondarySeedAnchors.Add(Secondary);
		}
	}

	if (!WrappingPhase.Begin(LatchAnchor,
		FMath::Max(0.01f, WrapConfig.WrappingMotionDuration), Sim, MakeWrappingContext()))
	{
		LogWrappingFailureState(GetName(), TEXT("StartWrapping.Begin"), WrappingPhase.State, Sim);
		// 성립 전 이탈 — 정리 후 per-instance 통지(FinishPreCommitReleaseToFlight).
		FinishPreCommitReleaseToFlight(ContactTracker.CandidateBone, TEXT("no valid wrapping anchors"));
		return;
	}

	SetPhase(ERopePhase::Wrapping, *FString::Printf(TEXT("bone=%s, %d anchor(s), %d secondary seed(s)"),
		*WrappingPhase.State.BoneName.ToString(), WrappingPhase.State.Anchors.Num(),
		WrappingPhase.State.SecondarySeedAnchors.Num()));
}

void URopeComponent::UpdateWrapping(float DeltaTime)
{
	WrappingPhase.State.Elapsed += DeltaTime;

	if (!WrappingPhase.IsStillValid())
	{
		if (WrappingPhase.State.PathBuildFailureReason.IsEmpty())
		{
			WrappingPhase.State.PathBuildFailureReason = TEXT("InvalidWrappingState");
		}
		LogWrappingFailureState(GetName(), TEXT("UpdateWrapping.IsStillValid"), WrappingPhase.State, Sim);
		SetPhase(ERopePhase::Releasing, TEXT("invalid wrapping state"));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	const FRopeWrappingPhase::FContext WrappingCtx = MakeWrappingContext();
	WrappingPhase.AdvancePathBuild(Sim, WrappingCtx);

	// Composite Analytic Helix가 terminal failure 뒤 SingleBone으로 fallback하면 이미 고정한 virtual node를 즉시
	// solver에 돌려준다. 새 Single 경로의 front/mass override가 아래에서 같은 프레임에 다시 적용된다.
	if (!WrappingPhase.State.bPathUsesPoseSpaceIsland &&
		(KinematicVirtualBridges.Num() > 0 || KinematicVirtualBridgeRunCursor > 0))
	{
		ReleaseKinematicVirtualBridgesToSolver();
	}

	// fallback 초기화 자체가 실패한 경우를 위한 마지막 안전망이다. 이후 SingleBone 진행 중 생긴
	// projection failure는 기존 partial-path 품질 판정에 맡긴다.
	if (WrappingPhase.State.bPathBuildFailed &&
		WrappingPhase.State.PathBuildFailureReason == TEXT("SingleBoneFallbackInitializationFailure"))
	{
		LogWrappingFailureState(GetName(), TEXT("UpdateWrapping.CompositeFallbackExhausted"),
			WrappingPhase.State, Sim);
		UE_LOG(LogRopeWrap, Warning,
			TEXT("[%s] Wrap cancelled: algorithm=CompositeAnalyticHelix->SingleBone reason=FallbackInitializationFailed "
				"path=%d/%d anchors=%d"),
			*GetName(), WrappingPhase.State.Path.Num(), WrappingPhase.State.NumTailNodes,
			WrappingPhase.State.Anchors.Num());
		SetPhase(ERopePhase::Releasing, TEXT("Composite analytic helix and fallback initialization both failed"));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	// 안전장치: 표면 경로 생성이 중간에 실패했을 때, 그때까지 감싼 각도가 임계 미만이면 "조금 닿았는데
	// 바로 wrapped로 철썩 붙는" 상태를 만들지 않고 release한다. 기준은 회전 수가 아니라 감싼 각도(도,
	// FailedWrapMinAngleDeg — 0이면 가드 끔): 회전 수는 로프 2πr을 요구해 큰 대상(드래곤 몸통)에서
	// 물리적으로 도달 불가능한 기준이 됐다. 상세는 config 주석.
	float FailedWrapAngleDeg = 0.0f;
	if (WrappingPhase.ShouldAbortFailedShortWrap(Sim, WrappingCtx, WrapConfig.FailedWrapMinAngleDeg, FailedWrapAngleDeg))
	{
		LogWrappingFailureState(GetName(), TEXT("UpdateWrapping.FailedShortWrap"), WrappingPhase.State, Sim);
		SetPhase(ERopePhase::Releasing, *FString::Printf(TEXT("wrap path failed early, angle=%.0fdeg < %.0fdeg"),
			FailedWrapAngleDeg, WrapConfig.FailedWrapMinAngleDeg));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	WrappingPhase.ApplyWrappingMotionOverrides(Sim, DeltaTime, WrappingCtx, SimFrame.OverrideFrame);

	WrappingPhase.ApplyWrappingKinematicMask(Sim, SimFrame.OverrideFrame);

	if (WrappingPhase.State.bPathUsesPoseSpaceIsland)
	{
		// ApplyWrappingMotionOverrides가 오른쪽 실제 표면 노드를 먼저 붙이고 ApplyWrappingKinematicMask가 virtual node를 기본
		// 동적 상태로 만든 뒤 실행한다. 따라서 front가 닫은 구간만 이 마지막 override로 즉시 조인다.
		UpdateWrappingKinematicVirtualBridges(
			WrappingPhase.State.VirtualBridgeRuns,
			WrappingPhase.State.Anchors,
			WrappingPhase.State.FrontDistance);
		HoldKinematicVirtualBridges();
	}

	WrappingPhase.UpdateAnchorSpanStability(DeltaTime);

	if (WrappingPhase.IsReadyToCommit(Sim, WrapConfig))
	{
		CommitWrapping();
		return;
	}
}

FRopeWrappingPhase::FContext URopeComponent::MakeWrappingContext() const
{
	// CaptureTravelPlane 전용 폴백: whip guide 평면이 없는 던지기(BP 직행 등)에서는 캡처 순간
	// 스냅샷(속도×누운 방향)으로 유도한 진행 평면 normal을 대신 싣는다. 기본값(BoneCenteredGuidePlane)
	// 에서는 주입하지 않는다 — BoneCenteredGuidePlane은 whip guide에서 얻은 normal만 사용한다.
	bool bGuidePlane = bHasFlightGuidePlaneNormal;
	FVector GuidePlane = FlightGuidePlaneNormal;
	if (!bGuidePlane &&
		WrapConfig.WrappingAxisSource == ERopeWrappingAxisSource::CaptureTravelPlane &&
		CaptureTravelFrame.bValid && CaptureTravelFrame.bHasPlaneNormal)
	{
		bGuidePlane = true;
		GuidePlane = CaptureTravelFrame.PlaneNormal;
	}

	// wrap 대상 게이트(CanWrapTarget)를 감김 경로에도 적용한다 — 조준/preview/판정이 이미 거른 대상을
	// 경로 빌드만 모르고 주워 앵커를 까는 불일치를 막는다. 게이트 기본값(전부 허용)이면 결과는
	// FrameColliders와 동일하다.
	RopeWrapTargets::FilterWrappableColliders(SimFrame.FrameColliders,
		[this](const USceneComponent* Mesh, FName Bone) { return CanWrapTarget(Mesh, Bone); },
		WrappableColliders);

	FRopeWrappingPhase::FContext Ctx{
		WrapConfig,
		WrappableColliders,
		Radius,
		GetName(),
		false,
		bGuidePlane,
		GuidePlane,
		CaptureTravelFrame.bValid ? &CaptureTravelFrame : nullptr
	};
	// 0=auto 해석은 컴포넌트 경계 책임 — preview 경로(FInput 값 사본에 덮어씀)와 달리 여기는
	// Config가 참조 전달이라 해석값을 별도 필드로 싣는다. 미주입 시 ContactQueryRadius=0(auto)
	// 로프만 wrapping 경로에서 질의 반경 0으로 떨어지는 갭이 있었다.
	Ctx.ResolvedContactRadius = GetEffectiveContactQueryRadius();
	Ctx.ResolveMode = ResolveMode;
	return Ctx;
}

void URopeComponent::CommitWrapping()
{
	const USceneComponent* Mesh = WrappingPhase.State.Mesh.Get();

	//Wrapping 정보가 적절하지 않으면 바로 releasing
	if (!Mesh || WrappingPhase.State.BoneName.IsNone() || WrappingPhase.State.Anchors.Num() == 0)
	{
		WrappingPhase.State.PathBuildFailureReason = TEXT("CommitStateInvalid");
		LogWrappingFailureState(GetName(), TEXT("CommitWrapping.StateValidation"), WrappingPhase.State, Sim);
		SetPhase(ERopePhase::Releasing, TEXT("commit failed"));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	// 커밋 시점 감싼 각도(도): 커밋 품질 관문(아래)과 전이 로그가 공용으로 쓴다. 실패 조기 abort와
	// 같은 척도라 로그의 angle 수치를 그대로 비교/튜닝에 쓸 수 있다. 계산 불가(축 축퇴 등)면 -1 표기.
	float CommitAngleDeg = -1.0f;
	WrappingPhase.ComputeBuiltPathWrapAngle(Sim, MakeWrappingContext(), CommitAngleDeg);

	// 커밋 품질 관문(opt-in — CommitMinWrapAngleDeg 0이면 기존 동작 그대로): 경로가 정상 완료됐거나
	// settle 타임아웃으로 왔어도, 감은 각도가 하한 미만인 부실 랩은 Wrapped로 확정하지 않는다.
	if (WrapConfig.CommitMinWrapAngleDeg > 0.0f && CommitAngleDeg >= 0.0f
		&& CommitAngleDeg < WrapConfig.CommitMinWrapAngleDeg)
	{
		WrappingPhase.State.PathBuildFailureReason = FString::Printf(
			TEXT("CommitAngleBelowThreshold(%.1f<%.1f)"), CommitAngleDeg, WrapConfig.CommitMinWrapAngleDeg);
		LogWrappingFailureState(GetName(), TEXT("CommitWrapping.AngleQualityGate"), WrappingPhase.State, Sim);
		SetPhase(ERopePhase::Releasing, *FString::Printf(TEXT("commit quality below threshold, angle=%.0fdeg < %.0fdeg"),
			CommitAngleDeg, WrapConfig.CommitMinWrapAngleDeg));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	// 형상 기준 묶임 관문(opt-in — CommitMinWrapCoverageDeg 0이면 기존 동작 그대로): 축 둘레 각도
	// 커버리지(진동으로 부풀지 않는 기하 척도 — FRopeWrapConfig 주석 참고)가 하한 미만이면 대상을
	// 둘러싸지 못한 랩이다 — 커밋하지 않는다. 계산 불가(경로점 부족/축 축퇴, -1 표기)면 관문을
	// 건너뛴다(계산 가능성으로 벌하지 않는다). 전이 로그에 항상 실려 실측 튜닝의 관측값이 된다.
	float CommitCoverageDeg = -1.0f;
	if (!WrappingPhase.ComputeWrapEnclosureCoverage(CommitCoverageDeg))
	{
		CommitCoverageDeg = -1.0f;
	}
	if (WrapConfig.CommitMinWrapCoverageDeg > 0.0f && CommitCoverageDeg >= 0.0f
		&& CommitCoverageDeg < WrapConfig.CommitMinWrapCoverageDeg)
	{
		WrappingPhase.State.PathBuildFailureReason = FString::Printf(
			TEXT("CommitCoverageBelowThreshold(%.1f<%.1f)"), CommitCoverageDeg,
			WrapConfig.CommitMinWrapCoverageDeg);
		LogWrappingFailureState(GetName(), TEXT("CommitWrapping.CoverageQualityGate"), WrappingPhase.State, Sim);
		SetPhase(ERopePhase::Releasing, *FString::Printf(TEXT("commit enclosure below threshold, coverage=%.0fdeg < %.0fdeg"),
			CommitCoverageDeg, WrapConfig.CommitMinWrapCoverageDeg));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	if (WrappingPhase.State.bPathUsesPoseSpaceIsland)
	{
		int32 BridgePointCount = 0;
		int32 VirtualPointCount = 0;
		int32 SurfaceSwitchCount = 0;
		TArray<FName, TInlineAllocator<16>> VisitedBones;
		FName PreviousSurfaceBone = NAME_None;
		for (int32 PathIndex = 0; PathIndex < WrappingPhase.State.Path.Num(); ++PathIndex)
		{
			const FRopeWrapPathPoint& Point = WrappingPhase.State.Path[PathIndex];
			BridgePointCount += Point.bBridge ? 1 : 0;
			VirtualPointCount += Point.bVirtual ? 1 : 0;
			if (!Point.bBridge && !Point.bVirtual && !Point.Bone.IsNone())
			{
				VisitedBones.AddUnique(Point.Bone);
				if (!PreviousSurfaceBone.IsNone() && Point.Bone != PreviousSurfaceBone)
				{
					++SurfaceSwitchCount;
				}
				PreviousSurfaceBone = Point.Bone;
			}
		}

		FString VisitedBoneList;
		for (const FName Bone : VisitedBones)
		{
			if (!VisitedBoneList.IsEmpty())
			{
				VisitedBoneList += TEXT(",");
			}
			VisitedBoneList += Bone.ToString();
		}
		const float BuiltDistance = WrappingPhase.State.Path.Num() > 0
			? WrappingPhase.State.Path.Last().DistanceFromLatch
			: 0.0f;
		UE_LOG(LogRopeWrap, Log,
			TEXT("[%s] Composite wrap path summary: points=%d surfacePoints=%d bridgePoints=%d virtualPoints=%d "
				"switches=%d visitedBones=%d builtDistance=%.2fcm angle=%.0fdeg coverage=%.0fdeg "
				"projectionMisses=%d bones=[%s]"),
			*GetName(), WrappingPhase.State.Path.Num(),
			WrappingPhase.State.Path.Num() - BridgePointCount - VirtualPointCount,
			BridgePointCount, VirtualPointCount,
			SurfaceSwitchCount, VisitedBones.Num(), BuiltDistance,
			CommitAngleDeg, CommitCoverageDeg,
			WrappingPhase.State.PathCompositeProjectionFailureCount, *VisitedBoneList);
	}

	const FRopeWrapState Seed = WrappingPhase.BuildCommitSeed(Sim);
	if (Seed.Anchors.Num() == 0)
	{
		WrappingPhase.State.PathBuildFailureReason = TEXT("CommitSeedHasNoValidAnchors");
		LogWrappingFailureState(GetName(), TEXT("CommitWrapping.BuildCommitSeed"), WrappingPhase.State, Sim);
		SetPhase(ERopePhase::Releasing, TEXT("no valid latches"));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	// Wrapping 중 만든 bridge를 버리고 Path를 다시 스캔하지 않는다. 최종 commit seed의 anchor를
	// 정본으로 재해석해 같은 bridge의 binding만 갱신하고 모두 활성화한다.
	if (!FinalizeKinematicVirtualBridges(WrappingPhase.State.VirtualBridgeRuns, Seed.Anchors))
	{
		WrappingPhase.State.PathBuildFailureReason = TEXT("KinematicVirtualBridgeFinalizeFailed");
		LogWrappingFailureState(GetName(), TEXT("CommitWrapping.FinalizeVirtualBridges"),
			WrappingPhase.State, Sim);
		SetPhase(ERopePhase::Releasing, TEXT("virtual bridge finalization failed"));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	// 감길 mesh는 Seed.Mesh로 전파(접촉 유래, cross-actor 포함).
	WrapController.BeginWrap(Sim, Seed, SimFrame.OverrideFrame);
	HoldKinematicVirtualBridges();
	ApplyWrappedMassMask(/*bResetDynamicNodeVelocity*/ true);

	SetPhase(ERopePhase::Wrapped, *FString::Printf(TEXT("bone=%s, %d latched node(s), angle=%.0fdeg, coverage=%.0fdeg"),
		*Seed.BoneName.ToString(), Seed.Latched.Num(), CommitAngleDeg, CommitCoverageDeg));
	ResetTransientPhaseState();
	const FRopeWrappedEventInfo WrappedInfo = MakeWrappedEventInfo(Seed, CommitAngleDeg, CommitCoverageDeg);
	DispatchWrapped(WrappedInfo);
}

void URopeComponent::DispatchWrapped(const FRopeWrappedEventInfo& Info)
{
	// wrap 성립의 단일 브로드캐스트 지점: 네이티브 훅(서브클래스) → per-instance BP 델리게이트 →
	// 월드 중앙 신호(대상 반응 컴포넌트가 자기 로프를 몰라도 구독으로 반응). 두 성립 경로(③ preview /
	// 판정) 공용.
	// 이 구간 동안 release 통지는 큐로 간다 — 핸들러가 곧바로 ReleaseWrap을 불러도 구독자가 보는 순서는
	// 항상 Wrapped → Released다(FDeferredReleaseNotice 주석). 통지 중 예외는 없는 코드지만, 깊이를
	// 짝지어 내리는 것은 이 함수의 단일 책임이므로 마지막에 반드시 내린다.
	++WrappedDispatchDepth;
	NotifyWrapped(Info);
	OnRopeWrapped.Broadcast(Info);
	if (URopeSimSubsystem* SimSubsystem = URopeSimSubsystem::Get(GetWorld()))
	{
		SimSubsystem->OnAnyRopeWrapped.Broadcast(Info);
	}
	--WrappedDispatchDepth;

	FlushDeferredReleaseNotices();
}

void URopeComponent::FlushDeferredReleaseNotices()
{
	// 중첩된 Wrapped 통지가 아직 남아 있으면 가장 바깥에서만 흘린다(순서 보장의 유일한 지점).
	if (WrappedDispatchDepth > 0 || DeferredReleaseNotices.Num() == 0)
	{
		return;
	}

	// 흘리는 도중 핸들러가 또 release를 부를 수 있으므로 큐를 먼저 비우고 사본을 돈다.
	TArray<FDeferredReleaseNotice> Pending = MoveTemp(DeferredReleaseNotices);
	DeferredReleaseNotices.Reset();
	for (const FDeferredReleaseNotice& Notice : Pending)
	{
		DispatchReleased(Notice.WrappedMesh.Get(), Notice.Bone, Notice.Reason, Notice.bWasWrapped);
	}
}

FRopeWrappedEventInfo URopeComponent::MakeWrappedEventInfo(const FRopeWrapState& Seed,
	float AngleDeg, float CoverageDeg) const
{
	FRopeWrappedEventInfo Info;
	Info.Bone = Seed.BoneName;
	// 성립 주체(이 로프) — 중앙 신호 구독자의 engagement 집합 키. release 신호도 같은 포인터를 싣는다.
	Info.Rope = const_cast<URopeComponent*>(this);
	// 이벤트 페이로드는 읽기 전용 의미라 대상 mesh의 const를 벗겨 BP에 노출한다(수정 계약 아님).
	Info.Mesh = const_cast<USceneComponent*>(Seed.Mesh.Get());
	Info.ResolveMode = ResolveMode;
	Info.AngleDeg = AngleDeg;
	Info.CoverageDeg = CoverageDeg;
	Info.AnchorCount = Seed.Anchors.Num();

	// 앵커가 걸친 본 전체(대표 본 우선, 중복 제거) — 양다리처럼 복수 본 성립의 전체 정보.
	if (!Seed.BoneName.IsNone())
	{
		Info.Bones.Add(Seed.BoneName);
	}
	for (const FRopeSurfaceAnchor& Anchor : Seed.Anchors)
	{
		if (!Anchor.Bone.IsNone())
		{
			Info.Bones.AddUnique(Anchor.Bone);
		}
	}
	return Info;
}

void URopeComponent::AbortWrapping(ERopeReleaseReason Reason)
{
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] AbortWrapping reason=%d"),
		*GetName(), static_cast<int32>(Reason));

	// Captured 짝 맞춤: Wrapping은 Contacting(Captured 발화)에서만 진입하므로 이 abort는 항상 앞선 Captured와
	// 짝이다. 성립 전이라 per-instance만(중앙 신호는 커밋된 wrap 전용). 본 이름은 값으로 잡아 두고
	// **정리가 다 끝난 뒤에** 통지한다 — FinishPreCommitReleaseToFlight와 같은 계약(정리 전에 쏘면
	// 핸들러의 ReleaseWrap/재던지기가 아직 살아 있는 wrapping 상태 위에서 돌고, 그 결과를 이 아래
	// 정리가 덮어쓴다). 호출자는 전부 SetPhase(Releasing)을 마친 뒤 들어온다.
	const FName AbortedBone = WrappingPhase.State.BoneName;

	WrappingPhase.ReleaseAnchoredNodesToSolver(Sim, SimFrame.OverrideFrame);
	ReleaseKinematicVirtualBridgesToSolver();

	ResetTransientPhaseState();
	ReleaseCooldown = ReleaseCooldownSeconds;

	DispatchReleased(nullptr, AbortedBone, Reason, /*bWasWrapped*/ false);
}

void URopeComponent::UpdateWrappingKinematicVirtualBridges(
	const TArray<FRopeVirtualBridgeRun>& Runs,
	const TArray<FRopeSurfaceAnchor>& Anchors, float FrontDistance)
{
	// Path의 virtual run 탐색은 WrappingPhase가 한 번만 수행한다. 컴포넌트는 아직 소비하지 않은 run에
	// 양쪽 anchor를 연결해 runtime bridge를 한 번 만들고, anchor가 늦으면 같은 run부터 다음 프레임 재시도한다.
	KinematicVirtualBridgeRunCursor = FMath::Clamp(
		KinematicVirtualBridgeRunCursor, 0, Runs.Num());
	while (KinematicVirtualBridgeRunCursor < Runs.Num())
	{
		const FRopeVirtualBridgeRun& Run = Runs[KinematicVirtualBridgeRunCursor];
		const FRopeSurfaceAnchor* LeftAnchor = FindSurfaceAnchorByNode(Anchors, Run.LeftNodeIndex);
		const FRopeSurfaceAnchor* RightAnchor = FindSurfaceAnchorByNode(Anchors, Run.RightNodeIndex);
		if (!LeftAnchor || !RightAnchor)
		{
			// 경로와 anchor 추가 순서가 갈린 경우 현재 run을 소비하지 않고 다음 프레임에 재시도한다.
			break;
		}

		const USceneComponent* LeftMesh = LeftAnchor->Mesh.Get();
		const USceneComponent* RightMesh = RightAnchor->Mesh.Get();
		if (LeftMesh && LeftMesh == RightMesh)
		{
			FKinematicVirtualBridge& Bridge = KinematicVirtualBridges.AddDefaulted_GetRef();
			Bridge.NodeIndices = Run.VirtualNodeIndices;
			Bridge.LeftAnchor = *LeftAnchor;
			Bridge.RightAnchor = *RightAnchor;
			Bridge.RestSpanLength = static_cast<float>(Run.RightNodeIndex - Run.LeftNodeIndex) * Sim.SegmentLength;
			Bridge.ActivationFrontDistance = RightAnchor->RopeDistance;
			Bridge.bActive = false;

			UE_LOG(LogRopeWrap, Verbose,
				TEXT("[%s] Wrapping virtual bridge registered: leftNode=%d rightNode=%d "
					"virtualNodes=%d activationDistance=%.2fcm"),
				*GetName(), Run.LeftNodeIndex, Run.RightNodeIndex, Bridge.NodeIndices.Num(),
				Bridge.ActivationFrontDistance);
		}
		++KinematicVirtualBridgeRunCursor;
	}

	// 오른쪽 anchor 거리까지 front가 도달했다는 것은 ApplyWrappingMotionOverrides가 양쪽 경계를 실제 표면 위치로
	// 고정했다는 뜻이다. 그 프레임부터 내부 virtual node를 직선으로 묶어 Wrapped 전 출렁임을 없앤다.
	const float FrontTolerance = FMath::Max(0.01f, Sim.SegmentLength * 0.001f);
	for (FKinematicVirtualBridge& Bridge : KinematicVirtualBridges)
	{
		if (!Bridge.bActive && FrontDistance + FrontTolerance >= Bridge.ActivationFrontDistance)
		{
			Bridge.bActive = true;
			UE_LOG(LogRopeWrap, Log,
				TEXT("[%s] Wrapping virtual bridge activated: leftNode=%d rightNode=%d "
					"virtualNodes=%d front=%.2fcm activation=%.2fcm"),
				*GetName(), Bridge.LeftAnchor.NodeIndex, Bridge.RightAnchor.NodeIndex,
				Bridge.NodeIndices.Num(), FrontDistance, Bridge.ActivationFrontDistance);
		}
	}
}

bool URopeComponent::FinalizeKinematicVirtualBridges(
	const TArray<FRopeVirtualBridgeRun>& Runs,
	const TArray<FRopeSurfaceAnchor>& CommitAnchors)
{
	// Commit은 같은 run을 다시 찾거나 bridge를 재생성하지 않는다. Wrapping 중 산출/등록된 목록이
	// 완전한지 먼저 확인한 뒤 최종 seed anchor 사본으로 binding만 교체한다.
	if (KinematicVirtualBridgeRunCursor != Runs.Num() || KinematicVirtualBridges.Num() != Runs.Num())
	{
		UE_LOG(LogRopeWrap, Error,
			TEXT("[%s] Virtual bridge finalization rejected: discoveredRuns=%d consumedRuns=%d bridges=%d"),
			*GetName(), Runs.Num(), KinematicVirtualBridgeRunCursor, KinematicVirtualBridges.Num());
		return false;
	}

	for (int32 RunIndex = 0; RunIndex < Runs.Num(); ++RunIndex)
	{
		const FRopeVirtualBridgeRun& Run = Runs[RunIndex];
		FKinematicVirtualBridge& Bridge = KinematicVirtualBridges[RunIndex];
		if (Bridge.LeftAnchor.NodeIndex != Run.LeftNodeIndex ||
			Bridge.RightAnchor.NodeIndex != Run.RightNodeIndex ||
			Bridge.NodeIndices != Run.VirtualNodeIndices)
		{
			UE_LOG(LogRopeWrap, Error,
				TEXT("[%s] Virtual bridge finalization rejected: run %d no longer matches registered bridge"),
				*GetName(), RunIndex);
			return false;
		}

		const FRopeSurfaceAnchor* LeftAnchor = FindSurfaceAnchorByNode(CommitAnchors, Run.LeftNodeIndex);
		const FRopeSurfaceAnchor* RightAnchor = FindSurfaceAnchorByNode(CommitAnchors, Run.RightNodeIndex);
		if (!LeftAnchor || !RightAnchor)
		{
			UE_LOG(LogRopeWrap, Error,
				TEXT("[%s] Virtual bridge finalization rejected: run=%d missing anchor left=%d right=%d"),
				*GetName(), RunIndex, Run.LeftNodeIndex, Run.RightNodeIndex);
			return false;
		}

		const USceneComponent* LeftMesh = LeftAnchor->Mesh.Get();
		const USceneComponent* RightMesh = RightAnchor->Mesh.Get();
		if (!LeftMesh || LeftMesh != RightMesh)
		{
			UE_LOG(LogRopeWrap, Error,
				TEXT("[%s] Virtual bridge finalization rejected: run=%d crosses components left=%s right=%s"),
				*GetName(), RunIndex, *GetNameSafe(LeftMesh), *GetNameSafe(RightMesh));
			return false;
		}

		Bridge.LeftAnchor = *LeftAnchor;
		Bridge.RightAnchor = *RightAnchor;
		Bridge.ActivationFrontDistance = RightAnchor->RopeDistance;
		Bridge.bActive = true;
	}

	if (Runs.Num() > 0)
	{
		UE_LOG(LogRopeWrap, Log,
			TEXT("[%s] Kinematic virtual bridges finalized in place: runs=%d"),
			*GetName(), KinematicVirtualBridges.Num());
	}
	return true;
}

void URopeComponent::HoldKinematicVirtualBridges()
{
	if (KinematicVirtualBridges.Num() == 0)
	{
		return;
	}

	SimFrame.OverrideFrame.EnsureSize(Sim.Num());
	for (FKinematicVirtualBridge& Bridge : KinematicVirtualBridges)
	{
		if (!Bridge.bActive)
		{
			// 경로 생성만 완료되고 front가 아직 오른쪽 anchor에 도달하지 않은 구간은 solver에 맡긴다.
			continue;
		}

		FVector LeftWorld;
		FVector RightWorld;
		if (!ResolveAnchorCenterlineWorld(Bridge.LeftAnchor, LeftWorld) ||
			!ResolveAnchorCenterlineWorld(Bridge.RightAnchor, RightWorld))
		{
			continue;
		}

		const int32 NumBridgeSegments = Bridge.NodeIndices.Num() + 1;
		if (NumBridgeSegments <= 1)
		{
			continue;
		}

		const float ChordLength = static_cast<float>(FVector::Dist(LeftWorld, RightWorld));
		const float ChordSegmentLength = ChordLength / static_cast<float>(NumBridgeSegments);
		const float StretchRatio = ChordLength /
			FMath::Max(Bridge.RestSpanLength, KINDA_SMALL_NUMBER);
		if (StretchRatio > KinematicBridgeStretchWarningRatio && !Bridge.bLoggedStretchWarning)
		{
			Bridge.bLoggedStretchWarning = true;
			UE_LOG(LogRopeWrap, Warning,
				TEXT("[%s] Kinematic virtual bridge stretched: nodes=%d chord=%.2fcm "
					"restSpan=%.2fcm segment=%.2fcm ratio=%.2f"),
				*GetName(), Bridge.NodeIndices.Num(), ChordLength,
				Bridge.RestSpanLength, ChordSegmentLength, StretchRatio);
		}

		for (int32 BridgeNodeIndex = 0; BridgeNodeIndex < Bridge.NodeIndices.Num(); ++BridgeNodeIndex)
		{
			const int32 NodeIndex = Bridge.NodeIndices[BridgeNodeIndex];
			if (!Sim.Positions.IsValidIndex(NodeIndex) || !Sim.InvMass.IsValidIndex(NodeIndex))
			{
				continue;
			}

			const float Alpha = static_cast<float>(BridgeNodeIndex + 1) /
				static_cast<float>(NumBridgeSegments);
			const FVector TargetWorld = FMath::Lerp(LeftWorld, RightWorld, Alpha);
			// Pos와 Prev를 함께 덮어 보정 속도 주입을 없애고, solver가 다시 밀지 못하게 hard pin한다.
			SimFrame.OverrideFrame.SetPosition(NodeIndex, TargetWorld, /*bZeroVelocity*/ true);
			SimFrame.OverrideFrame.SetInvMass(NodeIndex, 0.0f);
		}
	}
}

void URopeComponent::ResetKinematicVirtualBridges()
{
	// bridge binding과 WrappingPhase 산출물 소비 cursor를 함께 비워야 다음 wrap이 이전 run을 이어 읽지
	// 않는다. 이 함수 자체는 질량을 복구하지 않으므로 활성 bridge 해제에는 Release*를 쓴다.
	KinematicVirtualBridges.Reset();
	KinematicVirtualBridgeRunCursor = 0;
	bWrappedMassMaskDirty = true;
}

void URopeComponent::ReleaseKinematicVirtualBridgesToSolver()
{
	// Composite→Single fallback, abort, 정상 release 공용 경로. 활성 여부와 관계없이 등록된 모든
	// virtual node를 동적 질량으로 돌려 stale hard pin이 다음 phase까지 남지 않게 한다.
	if (KinematicVirtualBridges.Num() > 0)
	{
		SimFrame.OverrideFrame.EnsureSize(Sim.Num());
		for (const FKinematicVirtualBridge& Bridge : KinematicVirtualBridges)
		{
			for (const int32 NodeIndex : Bridge.NodeIndices)
			{
				if (!Sim.InvMass.IsValidIndex(NodeIndex))
				{
					continue;
				}

				// bridge 해제 프레임에 Pos-Prev 차이가 속도로 튀지 않도록 현재 위치에서 정지시킨다.
				const bool bStartPin = NodeIndex == 0 && Sim.bStartPinned;
				SimFrame.OverrideFrame.SetInvMass(NodeIndex, bStartPin ? 0.0f : 1.0f);
				SimFrame.OverrideFrame.SetPrevFromPosition(NodeIndex);
			}
		}
	}

	ResetKinematicVirtualBridges();
}

#pragma endregion Wrapping

#pragma region Wrapped_Hold_And_Pull_Sampling

// ===== Wrapped ==============================================================
// PrepareSimFrame의 Wrapped 케이스는 아래 4단계 헬퍼의 고정 순서로 돈다:
// ① HoldWrappedNodesToBone → ② UpdateWrappedPullSample → ③ ApplyWrappedTraction → ④ CheckWrappedAutoRelease

bool URopeComponent::HoldWrappedNodesToBone(float DeltaTime)
{
	// ① latch된 node는 skinned bone을 따라간다(GT). latch 노드는 InvMass=0이라 솔브는 자유 구간만.
	// Hold가 false면 wrap 대상 mesh가 사라진 것(예: cross-actor 대상 액터 파괴) →
	// 노드를 솔버에 되돌려 안전하게 release한다(dangling 포인터 역참조 방지는 Hold 내부에서).
	if (!WrapController.Hold(Sim, DeltaTime, SimFrame.OverrideFrame))
	{
		const FName Bone = WrapController.State.BoneName;
		FinishWrapRelease(Bone, ERopeReleaseReason::Broken,
			FString::Printf(TEXT("wrap target mesh lost, bone=%s"), *Bone.ToString()));
		return false;
	}
	HoldKinematicVirtualBridges();
	if (bWrappedMassMaskDirty)
	{
		ApplyWrappedMassMask();
	}
	return true;
}

bool URopeComponent::CheckWrappedAutoRelease(float DeltaTime)
{
	// ③ GuaranteedWrap: 자동 release(장력/거리)는 무효 — "무조건 성립"의 보장은 해제에도 대칭이라
	// 명시 해제(ReleaseWrap/CutRope/게임 이벤트)만 유효하다(2026-07-13 회의 결정 G). ①②는 종전대로.
	// (대상 mesh 소실 release는 자동 release가 아니라 안전 계약이라 모드 무관 — HoldWrappedNodesToBone.)
	if (ResolveMode == ERopeWrapResolveMode::GuaranteedWrap)
	{
		return false;
	}

	// ④-1 임계 장력 release: 최대 장력이 TensionReleaseForce를 TensionReleaseTime 동안 지속해 넘으면
	// 풀린다(순간 스파이크 무시). 0 = 비활성. 흐름은 mesh-lost release와 동일, 사유만 Tension.
	if (HoldConfig.TensionReleaseForce > 0.0f)
	{
		TensionOverTime = (WrapController.State.Tension > HoldConfig.TensionReleaseForce)
			? TensionOverTime + DeltaTime : 0.0f;
		if (TensionOverTime >= HoldConfig.TensionReleaseTime)
		{
			const FName Bone = WrapController.State.BoneName;
			FinishWrapRelease(Bone, ERopeReleaseReason::Tension,
				FString::Printf(TEXT("tension release %.0f > %.0f, bone=%s"),
					WrapController.State.Tension, HoldConfig.TensionReleaseForce, *Bone.ToString()));
			return true;
		}
	}

	// ④-2 거리 release: 손~앵커 직선 거리의 가용 로프 길이 초과분(테더 초과분과 동일 소스 —
	// UpdateTether가 이번 프레임 갱신한 PullDrive.LastTetherOvershoot)이 한계를 넘으면 놓친다.
	// 기하 기반이라 지속 시간 없이 즉시 판정(장력처럼 노이즈가 없다).
	if (HoldConfig.DistanceReleaseSlack > 0.0f && PullDrive.LastTetherOvershoot > HoldConfig.DistanceReleaseSlack)
	{
		const FName Bone = WrapController.State.BoneName;
		FinishWrapRelease(Bone, ERopeReleaseReason::Distance,
			FString::Printf(TEXT("distance release overshoot %.0f > %.0f, bone=%s"),
				PullDrive.LastTetherOvershoot, HoldConfig.DistanceReleaseSlack, *Bone.ToString()));
		return true;
	}
	return false;
}

void URopeComponent::ApplyWrappedMassMask(bool bResetDynamicNodeVelocity)
{
	TSet<int32> AnchorNodes;

	for (const FRopeSurfaceAnchor& Anchor : WrapController.State.Anchors)
	{
		if (Sim.InvMass.IsValidIndex(Anchor.NodeIndex))
		{
			AnchorNodes.Add(Anchor.NodeIndex);
		}
	}

	for (const FRopeLatchNode& Latch : WrapController.State.Latched)
	{
		if (Sim.InvMass.IsValidIndex(Latch.NodeIndex))
		{
			AnchorNodes.Add(Latch.NodeIndex);
		}
	}

	for (const FKinematicVirtualBridge& Bridge : KinematicVirtualBridges)
	{
		if (!Bridge.bActive)
		{
			continue;
		}

		for (const int32 NodeIndex : Bridge.NodeIndices)
		{
			if (Sim.InvMass.IsValidIndex(NodeIndex))
			{
				AnchorNodes.Add(NodeIndex);
			}
		}
	}

	SimFrame.OverrideFrame.EnsureSize(Sim.Num());
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		const bool bStartPin = (i == 0 && Sim.bStartPinned);
		const bool bAnchor = AnchorNodes.Contains(i);
		const bool bFixed = bStartPin || bAnchor;
		SimFrame.OverrideFrame.SetInvMass(i, bFixed ? 0.0f : 1.0f);
		if (bResetDynamicNodeVelocity && !bFixed)
		{
			SimFrame.OverrideFrame.SetPrevFromPosition(i);
		}
	}
	bWrappedMassMaskDirty = false;
}

#pragma endregion Wrapped_Hold_And_Pull_Sampling

