// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeComponent.h"
#include "DynamicRopeLog.h"
#include "Collision/RopeCollider.h"
#include "Render/RopeSceneProxy.h"
#include "Debug/RopeDebugDraw.h"       // stat 카운터(RopeDebug::Record*)
#include "Debug/RopeDebugSnapshot.h"   // 게이트플레이 디버거용 한 프레임 디버그 스냅샷
#include "Camera/CameraComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "ProfilingDebugging/CpuProfilerTrace.h" // TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)
#include "Subsystem/RopeSimSubsystem.h"
#include "Subsystem/RopeDebugSubsystem.h" // 디버그 캡처 게이트 + 스냅샷 보관소
#include "Settings/DynamicRopeSettings.h"
#include "RopeMathHelpers.h" // RopeMath::SmoothStep / AnyTangentFromNormal (unity 빌드 중복 정의 방지)
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h" // 길이 비례 파라미터용 런타임 인스턴스
#include "UObject/ConstructorHelpers.h" // 기본 머티리얼 로드(FObjectFinder)

namespace
{
	// 접촉 후보 노드들 중 가장 손(node 0)에 가까운 유효 인덱스.
	int32 FindHeadValidNodeIndex(const TArray<int32>& NodeIndices, const FRopeSimState& Sim)
	{
		int32 HeadNodeIndex = INDEX_NONE;
		for (const int32 NodeIndex : NodeIndices)
		{
			if (!Sim.Positions.IsValidIndex(NodeIndex))
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

	// Releasing 진입 시 Free 복귀까지의 쿨다운(초). Abort/Hold 실패/수동 해제 공통.
	constexpr float ReleaseCooldownSeconds = 0.08f;

	// phase 전이 로그용 짧은 이름(UEnum 리플렉션 없이 hot-path에서도 안전).
	const TCHAR* PhaseName(ERopePhase Phase)
	{
		switch (Phase)
		{
		case ERopePhase::Free:       return TEXT("Free");
		case ERopePhase::Flight:     return TEXT("Flight");
		case ERopePhase::Contacting: return TEXT("Contacting");
		case ERopePhase::Wrapping:   return TEXT("Wrapping");
		case ERopePhase::Wrapped:    return TEXT("Wrapped");
		case ERopePhase::Releasing:  return TEXT("Releasing");
		default:                     return TEXT("?");
		}
	}

}

URopeComponent::URopeComponent()
{
	// 컴포넌트는 스스로 tick하지 않는다 — URopeSimSubsystem이 모든 로프를
	// Prepare/Solve/Finalize 3단계로 한 곳에서 구동한다.
	PrimaryComponentTick.bCanEverTick = false;

	// primitive가 motion vector를 출력하도록 Movable로 설정한다(TAA/TSR가 움직이는 rope를 유지하게 한다).
	Mobility = EComponentMobility::Movable;

	// 플러그인 제공 기본 머티리얼(헴프 밧줄). 설정 안 하면 씬 프록시가 엔진 기본(회색)으로 폴백하므로
	// 여기서 기본값을 채운다 — 인스턴스/BP에서 RopeMaterial을 바꾸면 그대로 오버라이드된다.
	// 에셋이 없으면(.Succeeded()==false) null 유지 → 회색 폴백(빌드/쿠킹 안전).
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> DefaultRopeMaterial(
		TEXT("/DynamicRope/Materials/M_RopeDefault.M_RopeDefault"));
	if (DefaultRopeMaterial.Succeeded())
	{
		RopeMaterial = DefaultRopeMaterial.Object;
	}
}

// ===== API ==================================================================

void URopeComponent::Throw(const FVector& AimDir)
{
	ThrowWithContext(MakeDefaultThrowContext(AimDir));
}

void URopeComponent::ThrowWithContext(const FRopeThrowContext& ThrowContext)
{
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] Throw requested (phase=%s, forward=%s)"),
		*GetName(), PhaseName(Phase), *ThrowContext.FrameForward.GetSafeNormal().ToCompactString());

	EnsureRopeInitialized();
	StartFreshThrow(ThrowContext);
}

void URopeComponent::ReleaseWrap()
{
	if (Phase != ERopePhase::Wrapped && Phase != ERopePhase::Contacting && Phase != ERopePhase::Wrapping)
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
	else
	{
		Bone = ContactTracker.CandidateBone;
	}

	SetPhase(ERopePhase::Releasing, *FString::Printf(TEXT("manual, bone=%s"), *Bone.ToString()));
	WrapController.Release(ERopeReleaseReason::Manual);
	ResetTransientPhaseState();
	ReleaseCooldown = ReleaseCooldownSeconds;
	OnRopeReleased.Broadcast(Bone, ERopeReleaseReason::Manual);
}

// ===== 시뮬레이션 프레임(서브시스템이 3단계로 구동) ===========================

void URopeComponent::PrepareSimFrame(float DeltaTime)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_Prepare);

	EnsureRopeInitialized();
	OverrideFrame.Reset(); // 프레임 스코프 — 이번 프레임 로직 산출물을 새로 모은다(G2).

	// pinned-start target을 전진시킨다; solver가 substep에 걸쳐 Prev->Target을 sweep하므로 빠른
	// 캐릭터 이동이 chain을 홱 잡아당겨(폭주시켜) 버리지 않는다.
	if (Sim.bStartPinned)
	{
		Sim.StartPinPrev = Sim.StartPinTarget;
		Sim.StartPinTarget = GetComponentLocation();
	}

	// collider 스냅샷은 RopeSimSubsystem이 Tick의 collider 단계에서 중앙 수집해 FrameColliders에 채워둔다
	// (Prepare 이전). 여기서 로프마다 provider를 탐색/gather하지 않는다.

	bSolveThisFrame = false;

	switch (Phase)
	{
	case ERopePhase::Free:        // 손에서 늘어뜨려진 채 캐릭터를 따라간다
		bSolveThisFrame = true;
		break;

	case ERopePhase::Flight:
	{
		if (WhipGuide.IsActive())
		{
			// whip 가이드 타깃 캡처: 이 로프가 디버거 대상이거나(시각화) stat 수집 중일 때만(비용 절약).
#if WITH_GAMEPLAY_DEBUGGER
			const URopeDebugSubsystem* WhipDebugSub = URopeDebugSubsystem::Get(GetWorld());
			const bool bCaptureGuideTargets = (WhipDebugSub && WhipDebugSub->ShouldCapture(this)) || RopeDebug::IsFlightStatEnabled();
#else
			const bool bCaptureGuideTargets = RopeDebug::IsFlightStatEnabled();
#endif
			// 타깃/마스크 계산만(Sim 불변) — 적용은 CPU 경로 SolveSimFrame(ApplyToSim) 또는
			// GPU 상주 경로의 override 패스(서브시스템이 step에 실음)가 담당한다(G1).
			WhipGuide.Advance(DeltaTime, Sim, MakeWhipGuideConfig(), bCaptureGuideTargets);
			WhipElapsed = WhipGuide.GetElapsed(); // BP 노출용 미러.
		}
		else
		{
			WhipGuide.ResetFrameOutputs();
		}
		bSolveThisFrame = true; // 솔브 후 접촉 감지는 FinalizeSimFrame에서.
		break;
	}

	case ERopePhase::Contacting:
		UpdateContacting(DeltaTime);
		break;

	case ERopePhase::Wrapping:
		UpdateWrapping(DeltaTime);
		break;

	case ERopePhase::Wrapped:
	{
		// latch된 node는 skinned bone을 따라간다(GT). latch 노드는 InvMass=0이라 솔브는 자유 구간만.
		// Hold가 false면 wrap 대상 mesh가 사라진 것(예: cross-actor 대상 액터 파괴) →
		// 노드를 솔버에 되돌려 안전하게 release한다(dangling 포인터 역참조 방지는 Hold 내부에서).
		if (!WrapController.Hold(Sim, DeltaTime, OverrideFrame))
		{
			const FName Bone = WrapController.State.BoneName;
			SetPhase(ERopePhase::Releasing, *FString::Printf(TEXT("wrap target mesh lost, bone=%s"), *Bone.ToString()));
			WrapController.Release(ERopeReleaseReason::Broken);
			ResetTransientPhaseState();
			ReleaseCooldown = ReleaseCooldownSeconds;
			OnRopeReleased.Broadcast(Bone, ERopeReleaseReason::Broken);
			break;
		}
		ApplyWrappedMassMask();
		bSolveThisFrame = true;
		break;
	}

	case ERopePhase::Releasing:
		// 모든 node를 solver에 다시 넘긴다(hand pin만 유지) — InvMass 복원 + Prev=Pos(튐 방지)를
		// 프레임 산출물로 담고, cooldown이 끝나면 free simulation을 재개한다.
		OverrideFrame.EnsureSize(Sim.Num());
		for (int32 i = 0; i < Sim.Num(); ++i)
		{
			OverrideFrame.SetInvMass(i, (i == 0 && Sim.bStartPinned) ? 0.0f : 1.0f);
			OverrideFrame.SetPrevFromPosition(i);
		}
		ReleaseCooldown -= DeltaTime;
		if (ReleaseCooldown <= 0)
		{
			SetPhase(ERopePhase::Free);
		}
		break;

	default:
		break;
	}

	// 로직 페이즈의 프레임 산출물을 CPU Sim에 1회 적용한다 — 기존 "핸들러 안에서 직접 쓰기"와
	// 같은 결과(같은 노드 중복 시 나중 fill이 이김 = 순차 쓰기와 동일). GPU 상주 로프에는
	// 서브시스템이 같은 프레임을 override 패스로 실어 커널에서 적용한다(G2).
	// 로직 페이즈 재시드(SimGeneration 증가)는 소멸 — 재시드는 진짜 시드(Init/Throw)뿐이다.
	if (OverrideFrame.HasAny())
	{
		OverrideFrame.ApplyToSim(Sim);
	}
}

void URopeComponent::SolveSimFrame(float DeltaTime)
{
	// 병렬 단계: POD 상태(Sim) + collider 스냅샷(FrameColliders)만 만진다. Query는 const → 스레드 안전.
	// bSolveThisFrame(Free/Flight/Wrapped)일 때만 물리 솔브 — Wrapped는 latch 노드가 InvMass=0이라
	// 자유 구간만 움직이고, Contacting/Wrapping/Releasing은 로직 구동(Prepare에서 GT 처리)이라 스킵.
	if (!bSolveThisFrame)
	{
		return;
	}
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_Solve);

	// whip 타깃 적용(CPU 경로, POD만 — 스레드 안전). Prepare의 Advance가 계산한 산출물을 솔브 시작
	// 위치로 기록한다. GPU 로프는 이 함수 대신 override 패스가 같은 데이터를 커널에서 적용한다(G1).
	// Flight 게이트: 다른 페이즈에 남은 stale 마스크(직전 whip 프레임 산출물)가 적용되는 것을 막는다.
	if (Phase == ERopePhase::Flight)
	{
		WhipGuide.ApplyToSim(Sim);
	}

	Solver.Step(Sim, SolverConfig, /*optional*/ FrameColliders, DeltaTime);
}

void URopeComponent::FinalizeSimFrame(float DeltaTime)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_Finalize);

	// 디버그 캡처 게이트: 이 로프가 게이트플레이 디버거의 대상 액터일 때만 비주얼 데이터를 모은다.
	// 대상이 아닌 로프는 아래 flight sweep 등 캡처 비용을 전혀 내지 않는다(타깃 1개 로프만 부담).
#if WITH_GAMEPLAY_DEBUGGER
	URopeDebugSubsystem* DebugSub = URopeDebugSubsystem::Get(GetWorld());
	const bool bDebugCapture = DebugSub && DebugSub->ShouldCapture(this);
	FRopeDebugSnapshot DebugSnapshot;
#else
	constexpr bool bDebugCapture = false;
#endif

	// Flight: 솔브 후 이동 경로 기반 접촉 후보 감지 → 캡처. 파이프라인 자체는
	// FRopeFlightContactDetector(UObject 비의존)이고, 여기서는 입력 조립 + 전이/이벤트만 한다.
	if (Phase == ERopePhase::Flight)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FinalizeFlight);
		const FRopeFlightContactDetector::FParams DetectParams = MakeFlightDetectParams();
		TArray<FRopeContactCandidate> Candidates;
		TArray<FRopeFlightNodeDebug> FlightNodeDebug;
		if (bDebugCapture)
		{
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
				NodeDebug.NodeSpeed = FRopeFlightContactDetector::NodeSpeed(Sim, i);
				NodeDebug.bFast = FRopeFlightContactDetector::IsTailNode(Sim, i) || NodeDebug.NodeSpeed > Sim.SegmentLength;
				NodeDebug.bNearBody = FRopeFlightContactDetector::IsNearAnyColliderSegment(
					NodeDebug.PrevPosition, NodeDebug.Position, FrameColliders, DetectParams);
				if (NodeDebug.bFast || NodeDebug.bNearBody)
				{
					NodeDebug.Contact = FRopeFlightContactDetector::SweepOrSampleContact(
						Sim, NodeDebug.PrevPosition, NodeDebug.Position, FrameColliders, DetectParams);
				}

				if (NodeDebug.bFast || NodeDebug.bNearBody || NodeDebug.Contact.bHit)
				{
					FlightNodeDebug.Add(NodeDebug);
				}
			}
		}

		// whip 가이드 활성 프레임엔 예측 접촉용 데이터 뷰를 구성한다(다음 프레임 타깃 미리보기 포함).
		// 예측이 꺼져 있으면(PredictiveContactFrames<=0) 검출기가 어차피 early-out이라 미리보기를 만들지 않는다.
		FRopeFlightContactDetector::FWhipGuideView WhipView;
		TArray<FVector> NextGuideTargets;
		if (WrapConfig.PredictiveContactFrames > KINDA_SMALL_NUMBER && WhipGuide.GetGuidedNodeMask().Num() > 0)
		{
			WhipGuide.PreviewNextTargets(DeltaTime, Sim, MakeWhipGuideConfig(), NextGuideTargets);
			WhipView.GuidedNodeMask = &WhipGuide.GetGuidedNodeMask();
			WhipView.CurrentTargets = &WhipGuide.GetCurrentTargets();
			WhipView.PrevTargets = &WhipGuide.GetPrevTargets();
			WhipView.NextTargets = &NextGuideTargets;
		}

		if (bGpuContactsThisFrame)
		{
			// GPU 감지 경로(G3): actual+predictive 후보 모두 GPU 커널이 산출한 것을 쓴다(귀속·중복제거는
			// 서브시스템이 복원). 상대운동 평가(ExpectedWrapTangent는 hand=node0 위치 필요)만 GT에서 돌린다.
			TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightGpuContacts);
			Candidates = GpuFlightCandidates;
			FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, DetectParams, Candidates);
		}
		else
		{
			{
				TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightActualContacts);
				FRopeFlightContactDetector::DetectContactCandidates(Sim, FrameColliders, DetectParams, Candidates);
			}
			{
				TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightPredictiveContacts);
				FRopeFlightContactDetector::AddPredictedContactCandidates(Sim, FrameColliders, DetectParams, WhipView, Candidates);
			}
			{
				TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightEvaluateCandidates);
				FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, DetectParams, Candidates);
			}
		}

		FRopeContactTracker FlightDebugTracker;
		{
			TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightTrackerUpdate);
			FlightDebugTracker.Update(Candidates, 0.0f);
		}
		bool bShouldCapture = false;
		{
			TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightShouldCapture);
			bShouldCapture = FRopeFlightContactDetector::ShouldCapture(Candidates, DetectParams);
		}
		if (bShouldCapture)
		{
			FlightNoContactElapsed = 0.0f;
			TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightBuildContactingState);
			BuildContactingState(Candidates);
			SetPhase(ERopePhase::Contacting, *FString::Printf(TEXT("bone=%s, %d node(s)"),
				*ContactTracker.CandidateBone.ToString(), ContactTracker.CandidateNodes.Num()));
			OnRopeCaptured.Broadcast(ContactTracker.CandidateBone);
		}
		else
		{
			// Whip이 끝난 뒤 캡처하지 못하고 남아 있으면 실패로 보고 Free로 복귀한다.
			// 후보가 계속 있어도 MinLatchNodes/품질 조건을 넘지 못하면 Flight에 갇힐 수 있으므로 리셋하지 않는다.
			if (!WhipGuide.IsActive())
			{
				const float FlightReturnTime = WrapConfig.FlightNoContactReturnTime > 0.0f
					? WrapConfig.FlightNoContactReturnTime
					: ReleaseCooldownSeconds;
				FlightNoContactElapsed += DeltaTime;
				if (FlightNoContactElapsed >= FlightReturnTime)
				{
					SetPhase(ERopePhase::Free, *FString::Printf(TEXT("flight failed %.3fs"), FlightNoContactElapsed));
					ResetTransientPhaseState();
				}
			}
			else
			{
				FlightNoContactElapsed = 0.0f;
			}
		}

		// stat 카운터(stat 시스템이 수집 중일 때만; 디버그 캡처와 독립).
		const FRopeContactTracker& DebugTracker = bShouldCapture ? ContactTracker : FlightDebugTracker;
		const float WhipGuidedEnd = FMath::Clamp(WhipConfig.GuidedLength, 0.05f, 0.95f);
		const bool bWhipActive = WhipGuide.GetDebugGuideTargets().Num() > 0;
		RopeDebug::RecordFlightStats(Sim, bSolveThisFrame, FrameColliders.Num(), Candidates,
			DebugTracker, WrapConfig, bShouldCapture);
		RopeDebug::RecordWhipStats(Sim, WhipGuide.GetDebugGuideNodeIndices(), WhipGuide.GetDebugGuideTargets(),
			WhipGuidedEnd, bWhipActive);

#if WITH_GAMEPLAY_DEBUGGER
		if (bDebugCapture)
		{
			DebugSnapshot.bHasFlight = true;
			DebugSnapshot.bSolveThisFrame = bSolveThisFrame;
			DebugSnapshot.bShouldCapture = bShouldCapture;
			DebugSnapshot.FrameColliderCount = FrameColliders.Num();
			DebugSnapshot.MinLatchNodes = WrapConfig.MinLatchNodes;
			DebugSnapshot.TrackerBone = DebugTracker.CandidateBone;
			DebugSnapshot.TrackerNodes = DebugTracker.CandidateNodes;
			DebugSnapshot.NodeDebug = MoveTemp(FlightNodeDebug);
			DebugSnapshot.Candidates = Candidates;
			DebugSnapshot.bWhipActive = bWhipActive;
			DebugSnapshot.WhipGuidedEnd = WhipGuidedEnd;
			DebugSnapshot.WhipGuideNodeIndices = WhipGuide.GetDebugGuideNodeIndices();
			DebugSnapshot.WhipGuideTargets = WhipGuide.GetDebugGuideTargets();
		}
#endif
	}

	// 새 centerline을 render proxy로 push하고 bounds를 갱신한다.
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_MarkRenderDirty);
		MarkRenderDynamicDataDirty();
		MarkRenderTransformDirty();
	}

	// wrapped stat 카운터(독립).
	if (Phase == ERopePhase::Wrapped)
	{
		RopeDebug::RecordWrappedStats(Sim, WrapController.State);
	}

	// 디버그 스냅샷 제출: centerline/collider/wrapped 공통 필드를 채워 디버거 보관소로 넘긴다.
#if WITH_GAMEPLAY_DEBUGGER
	if (bDebugCapture)
	{
		FillDebugSnapshot(DebugSnapshot);
		DebugSub->SubmitSnapshot(this, MoveTemp(DebugSnapshot));
	}
#endif
}

// ===== UActorComponent ======================================================

void URopeComponent::BeginPlay()
{
	Super::BeginPlay();
	if (URopeSimSubsystem* SimSubsystem = URopeSimSubsystem::Get(GetWorld()))
	{
		SimSubsystem->RegisterRope(this);
	}
	else
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] BeginPlay: RopeSimSubsystem unavailable — rope will not be simulated."),
			*GetName());
	}
}

void URopeComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (URopeSimSubsystem* SimSubsystem = URopeSimSubsystem::Get(GetWorld()))
	{
		SimSubsystem->UnregisterRope(this);
	}
	Super::EndPlay(EndPlayReason);
}

void URopeComponent::SendRenderDynamicData_Concurrent()
{
	Super::SendRenderDynamicData_Concurrent();

	if (!SceneProxy || Sim.Num() < 2)
	{
		return;
	}

	// centerline을 component-local 공간으로 보낸다; proxy는 GetLocalToWorld()를 통해 렌더링한다.
	const FTransform Xform = GetComponentTransform();
	FRopeDynamicData* DynamicData = new FRopeDynamicData;
	DynamicData->bGpuResident = bGpuSteppedThisFrame; // M5b: GPU step된 프레임만 resident PosBuf 직접 렌더 허용.
	DynamicData->Points.SetNumUninitialized(Sim.Num());
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		DynamicData->Points[i] = Xform.InverseTransformPosition(Sim.Positions[i]);
	}

	FRopeSceneProxy* Proxy = static_cast<FRopeSceneProxy*>(SceneProxy);
	ENQUEUE_RENDER_COMMAND(RopeUpdateCenterline)(
		[Proxy, DynamicData](FRHICommandListBase& RHICmdList)
		{
			Proxy->SetDynamicData_RenderThread(RHICmdList, DynamicData);
		});
}

void URopeComponent::OnRegister()
{
	Super::OnRegister();
	// 에디터에서도 Sim에 기본 직선 포즈를 채워 둔다(서브시스템 틱은 PIE에서만 돌기 때문).
	// 이미 채워져 있으면(InitRope 후/PIE 진행 중) 그대로 둔다.
	EnsureRopeInitialized();
	// 길이 의존 머티리얼 파라미터 초기 세팅(EnsureRopeInitialized가 Sim만 채우고 재init 안 하는 재등록 케이스 포함).
	UpdateRopeMaterialDynamicParams();
}

void URopeComponent::CreateRenderState_Concurrent(FRegisterComponentContext* Context)
{
	Super::CreateRenderState_Concurrent(Context);
	// 프록시가 막 생성됐다. 틱이 없는 에디터/스폰 직후에도 한 번은 센터라인을 밀어 BuildTube가 돌게 한다
	// (그래야 bHasData=true가 되어 정적 드로우가 유효 지오메트리를 그린다). SendRenderDynamicData_Concurrent는
	// SceneProxy/Sim 유효성을 자체 검사하고 렌더 커맨드만 enqueue하므로 이 시점 호출이 안전하다.
	SendRenderDynamicData_Concurrent();
}

#if WITH_EDITOR
void URopeComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	// NumParticles/RopeLength가 바뀌면 프록시는 새 토폴로지(NumRings)로 재생성되지만 Sim은 옛 개수라
	// BuildTube가 Points.Num()!=NumRings로 건너뛰어 미리보기가 사라진다. EnsureRopeInitialized는 비어있을
	// 때만 init하므로, 여기선 Sim을 새 값으로 강제 재구성해 토폴로지를 맞춘다. 이후 Super가 렌더 상태를
	// 재생성하며 CreateRenderState_Concurrent에서 센터라인을 다시 푸시한다.
	const FName PropertyName = PropertyChangedEvent.GetPropertyName();
	if (PropertyName == GET_MEMBER_NAME_CHECKED(URopeComponent, NumParticles) ||
		PropertyName == GET_MEMBER_NAME_CHECKED(URopeComponent, RopeLength))
	{
		InitRope();
	}

	// RopeMaterial/RopeLength/bScaleTwistByLength 변경 시 dynamic material 파라미터 갱신(에디터 미리보기 즉시 반영).
	UpdateRopeMaterialDynamicParams();

	Super::PostEditChangeProperty(PropertyChangedEvent);
}
#endif

// ===== UPrimitiveComponent / UMeshComponent =================================

FPrimitiveSceneProxy* URopeComponent::CreateSceneProxy()
{
	return new FRopeSceneProxy(this);
}

int32 URopeComponent::GetNumMaterials() const
{
	return 1;
}

UMaterialInterface* URopeComponent::GetMaterial(int32 /*ElementIndex*/) const
{
	// 길이 비례 파라미터를 실은 MID가 있으면 그것을 반환(없으면 원본 머티리얼).
	if (RopeMID)
	{
		return RopeMID;
	}
	return RopeMaterial;
}

void URopeComponent::SetMaterial(int32 /*ElementIndex*/, UMaterialInterface* Material)
{
	RopeMaterial = Material;
	UpdateRopeMaterialDynamicParams(); // 새 부모로 MID 재생성 + 길이 파라미터 재적용(MarkRenderStateDirty 포함).
}

void URopeComponent::UpdateRopeMaterialDynamicParams()
{
	// 길이 스케일 비활성 또는 머티리얼 없음 → MID 불필요. 있으면 버려 원본을 그대로 쓴다.
	auto DropMID = [this]()
	{
		if (RopeMID)
		{
			RopeMID = nullptr;
			MarkRenderStateDirty();
		}
	};

	if (!bScaleTwistByLength || !RopeMaterial)
	{
		DropMID();
		return;
	}

	// 머티리얼(또는 프리셋)이 저작한 기준 TwistTurns를 읽는다. 파라미터가 없는 커스텀 머티리얼이면 대상 아님.
	float AuthoredTwist = 0.0f;
	if (!RopeMaterial->GetScalarParameterValue(FMaterialParameterInfo(TEXT("TwistTurns")), AuthoredTwist))
	{
		DropMID();
		return;
	}

	// 부모(RopeMaterial/프리셋)가 바뀌었으면 MID 재생성.
	if (!RopeMID || RopeMID->Parent != RopeMaterial)
	{
		RopeMID = UMaterialInstanceDynamic::Create(RopeMaterial, this);
	}

	// 저작된 TwistTurns × (RopeLength / 기준 200cm) → 길이에 비례해 꼬임 간격 일정, 프리셋 상대 밀도 보존.
	// 기준 200cm = 기본 RopeLength라 기본 길이에선 저작값 그대로(시각적 회귀 없음).
	constexpr float ReferenceLengthCm = 200.0f;
	RopeMID->SetScalarParameterValue(TEXT("TwistTurns"), AuthoredTwist * (RopeLength / ReferenceLengthCm));
	MarkRenderStateDirty();
}

FBoxSphereBounds URopeComponent::CalcBounds(const FTransform& LocalToWorld) const
{
	// bounds를 컴포넌트(pinned start)에 anchor하되, rope가 어떻게 변형되든 항상 rope를 포함하는 반지름을
	// 사용한다: chain은 inextensible하므로 어떤 particle도 pin으로부터 RopeLength(+ tube radius)보다 멀리
	// 떨어지지 않는다. 대신 per-frame sim point로부터 bounds를 도출하면 render thread보다 한 frame 뒤처지며;
	// 빠른 캐릭터 모션 중에는 rope가 그 tight box를 앞질러 shadow/main pass에서 cull된다 -> 움직이는 동안
	// shadow가 사라지고 VSM cache는 오래된 afterimage를 유지한다. component transform에 anchor하면 엔진의
	// 추적되는 transform을 통해 bounds가 캐릭터와 함께 움직이므로, lag도 없고 잘못된 culling도 없다.
	const float Reach = RopeLength + Radius + 1.0f;
	return FBoxSphereBounds(LocalToWorld.GetLocation(), FVector(Reach), Reach);
}

// ===== 페이즈 상태 머신 ======================================================

void URopeComponent::SetPhase(ERopePhase NewPhase, const TCHAR* Reason)
{
	if (Reason)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] %s -> %s (%s)"),
			*GetName(), PhaseName(Phase), PhaseName(NewPhase), Reason);
	}
	else
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] %s -> %s"),
			*GetName(), PhaseName(Phase), PhaseName(NewPhase));
	}
	Phase = NewPhase;
}

void URopeComponent::ResetTransientPhaseState()
{
	ContactTracker.Reset();
	PendingWrapSeed.Reset();
	WrappingPhase.State.Reset();
	ContactingElapsed = 0.0f;
	FlightNoContactElapsed = 0.0f;
}

// ===== 초기화/유틸 ===========================================================

void URopeComponent::InitRope()
{
	const int32 N = FMath::Max(2, NumParticles);
	Sim.Positions.SetNum(N);
	Sim.PrevPositions.SetNum(N);
	Sim.InvMass.SetNum(N);
	Sim.RopeLength = RopeLength;
	Sim.SegmentLength = RopeLength / static_cast<float>(N - 1);

	const FVector Start = GetComponentLocation();
	const FVector End = Start + GetForwardVector() * RopeLength;
	for (int32 i = 0; i < N; ++i)
	{
		const float Alpha = static_cast<float>(i) / static_cast<float>(N - 1);
		Sim.Positions[i] = FMath::Lerp(Start, End, Alpha);
		Sim.PrevPositions[i] = Sim.Positions[i];
		Sim.InvMass[i] = 1.0f;
	}

	// 시작점을 컴포넌트(hand/socket)에 pin한다; solver가 substep에 걸쳐 이를 sweep한다.
	Sim.InvMass[0] = 0.0f;
	Sim.bStartPinned = true;
	Sim.StartPinTarget = Start;
	Sim.StartPinPrev = Start;

	++SimGeneration; // Sim 전면 재구성 → GPU 상주 버퍼 재시드(M5).

	// 길이가 확정되는 지점 — 꼬임 밀도(TwistTurns)를 새 RopeLength에 맞춰 갱신(런타임 길이 변경/재throw 포함).
	UpdateRopeMaterialDynamicParams();

	UE_LOG(LogDynamicRope, Verbose, TEXT("[%s] InitRope: %d particles, length=%.1f, segment=%.2f"),
		*GetName(), N, Sim.RopeLength, Sim.SegmentLength);
}

#if WITH_GAMEPLAY_DEBUGGER
void URopeComponent::FillDebugSnapshot(FRopeDebugSnapshot& Snapshot) const
{
	Snapshot.Phase = Phase;
	Snapshot.Positions = Sim.Positions;

	// centerline 상에서 강조할 latch 노드 인덱스.
	const FRopeWrapState& Wrap = WrapController.State;
	Snapshot.LatchedNodes.Reset();
	for (const FRopeLatchNode& Latch : Wrap.Latched)
	{
		Snapshot.LatchedNodes.Add(Latch.NodeIndex);
	}

	// wrapped 상세(테이블용)는 Wrapped phase일 때만.
	if (Phase == ERopePhase::Wrapped && Wrap.IsWrapped())
	{
		Snapshot.bHasWrapped = true;
		Snapshot.WrapBone = Wrap.BoneName;
		const USkeletalMeshComponent* Mesh = Wrap.Mesh.Get();
		Snapshot.MeshName = Mesh ? Mesh->GetName() : TEXT("None");
		Snapshot.Latched = Wrap.Latched;
	}

	// 이 로프가 이번 프레임 질의한 collider 시각화(provider bDrawDebug 대체). capsule이면 세그먼트,
	// 그 외(SDF 등)는 월드 bounds 박스. FrameColliders는 provider 소유라 이 프레임 동안만 유효.
	Snapshot.Colliders.Reset();
	for (const IRopeCollider* Collider : FrameColliders)
	{
		if (!Collider)
		{
			continue;
		}
		FRopeDebugCollider DC;
		if (Collider->GetGPUCapsule(DC.A, DC.B, DC.Radius))
		{
			DC.bIsCapsule = true;
		}
		else
		{
			DC.bIsCapsule = false;
			DC.Bounds = Collider->GetWorldBounds();
		}
		Snapshot.Colliders.Add(DC);
	}
}
#endif

// ===== Throw ================================================================

FRopeThrowContext URopeComponent::MakeDefaultThrowContext(const FVector& /*AimDir*/) const
{
	FRopeThrowContext Context;
	Context.Origin = GetComponentLocation();
	Context.FrameMode = ThrowParams.FrameMode;
	Context.ThrowSpeed = ThrowParams.ThrowSpeed;
	Context.FrameForward = GetForwardVector();
	Context.FrameUp = ThrowParams.FrameMode == ERopeThrowFrameMode::World ? FVector::UpVector : GetUpVector();
	Context.FrameRight = ThrowParams.FrameMode == ERopeThrowFrameMode::World ? FVector::RightVector : GetRightVector();
	if (ThrowParams.FrameMode == ERopeThrowFrameMode::World)
	{
		Context.FrameForward = FVector::ForwardVector;
		Context.FrameRight = FVector::RightVector;
	}
	if (ThrowParams.FrameMode == ERopeThrowFrameMode::OwnerCamera)
	{
		if (const AActor* Owner = GetOwner())
		{
			if (const UCameraComponent* Camera = Owner->FindComponentByClass<UCameraComponent>())
			{
				Context.FrameForward = Camera->GetForwardVector();
				Context.FrameUp = Camera->GetUpVector();
				Context.FrameRight = Camera->GetRightVector();
			}
		}
	}
	if (ThrowParams.FrameMode == ERopeThrowFrameMode::Custom)
	{
		Context.FrameForward = ThrowParams.CustomFrameForward;
		Context.FrameUp = ThrowParams.CustomFrameUp;
		Context.FrameRight = ThrowParams.CustomFrameRight;
	}
	Context.SwingPlane = ThrowParams.SwingPlane;
	Context.CustomSwingPlaneNormal = ThrowParams.CustomSwingPlaneNormal;
	Context.AimDirection = Context.FrameForward;
	return Context;
}

FRopeThrowContext URopeComponent::ResolveThrowContext(const FRopeThrowContext& ThrowContext) const
{
	FRopeThrowContext Resolved = ThrowContext;

	Resolved.FrameForward = FRopeWhipGuide::SafeNormalOr(Resolved.FrameForward, GetForwardVector());
	Resolved.FrameUp = FRopeWhipGuide::SafeNormalOr(Resolved.FrameUp, FVector::UpVector);
	Resolved.FrameRight = FRopeWhipGuide::SafeNormalOr(Resolved.FrameRight, FVector::CrossProduct(Resolved.FrameUp, Resolved.FrameForward));
	Resolved.AimDirection = Resolved.FrameForward;
	if (Resolved.ThrowSpeed <= 0.0f)
	{
		Resolved.ThrowSpeed = ThrowParams.ThrowSpeed;
	}
	if (Resolved.Origin.IsNearlyZero())
	{
		Resolved.Origin = GetComponentLocation();
	}

	return Resolved;
}

FVector URopeComponent::ComputeThrowInheritedVelocity(const FRopeThrowContext& ThrowContext) const
{
	return ThrowContext.OwnerVelocity * ThrowParams.OwnerVelocityScale +
		ThrowContext.SocketVelocity * ThrowParams.SocketVelocityScale;
}

void URopeComponent::StartFreshThrow(const FRopeThrowContext& ThrowContext)
{
	const FRopeThrowContext ResolvedThrow = ResolveThrowContext(ThrowContext);
	const FRopeWhipGuide::FSwingBasis SwingBasis = FRopeWhipGuide::ResolveSwingBasis(
		ResolvedThrow, ResolvedThrow.SwingPlane, ResolvedThrow.CustomSwingPlaneNormal);

	// 채찍 스윙 가이드 좌표계 구성 + 활성화(퇴화 케이스 fallback은 컴포넌트 축).
	WhipGuide.Begin(SwingBasis.AimDir, ResolvedThrow.Origin,
		ResolvedThrow.FrameForward, SwingBasis.GuideUp, SwingBasis.GuideRight, ResolvedThrow.ThrowSpeed);
	WhipElapsed = WhipGuide.GetElapsed();

	++SimGeneration; // throw로 tail 위치를 재설정 → GPU 상주 버퍼 재시드(M5).

	if (WrapController.IsActive())
	{
		WrapController.Release(ERopeReleaseReason::Manual);
	}
	ResetTransientPhaseState();
	ReleaseCooldown = 0.0f;

	const int32 LastNode = Sim.Num() - 1;
	if (LastNode >= 1)
	{
		const FVector Start = ResolvedThrow.Origin;
		Sim.bStartPinned = true;
		Sim.StartPinPrev = Start;
		Sim.StartPinTarget = Start;
		Sim.Positions[0] = Start;
		Sim.PrevPositions[0] = Start;

		for (int32 i = 0; i < Sim.Num(); ++i)
		{
			Sim.InvMass[i] = (i == 0) ? 0.0f : 1.0f;
			Sim.PrevPositions[i] = Sim.Positions[i];
		}

		// 가이드 구간 노드를 T=0 가이드 곡선 위에 스냅(속도 0).
		WhipGuide.SnapToInitialPose(Sim, MakeWhipGuideConfig());

		// 던지기 임펄스: PrevPositions를 조준 반대 방향으로 밀어 Verlet 속도를 주입한다.
		// tail로 갈수록 가중치를 높이고 TipMass로 끝부분을 부스트한다.
		const FVector ThrowDir = WhipGuide.GetAimDir();
		const float ReferenceDt = 1.0f / 60.0f;
		const float BaseImpulse = ResolvedThrow.ThrowSpeed * ReferenceDt;
		const float TipBoost = FMath::Clamp(ThrowParams.TipMass / 5.0f, 0.25f, 3.0f);
		const FVector InheritedVelocityImpulse = ComputeThrowInheritedVelocity(ResolvedThrow) * ReferenceDt;
		const int32 FirstTailNode = FMath::Clamp(FMath::FloorToInt(static_cast<float>(LastNode) * WhipConfig.GuidedLength), 1, LastNode);
		for (int32 i = 1; i <= LastNode; ++i)
		{
			const float AlongRope = static_cast<float>(i) / static_cast<float>(LastNode);
			const float TailWeight = TailWeightByIndex(i, FirstTailNode, LastNode);
			const float Weight = FMath::Lerp(RopeMath::SmoothStep(AlongRope), 1.0f, TailWeight * 0.5f);
			const float Impulse = BaseImpulse * Weight * FMath::Lerp(1.0f, TipBoost, TailWeight);
			Sim.PrevPositions[i] -= ThrowDir * Impulse + InheritedVelocityImpulse;
		}
	}

	SetPhase(ERopePhase::Flight, *FString::Printf(TEXT("fresh throw impulse, aim=%s, speed=%.1f"),
		*WhipGuide.GetAimDir().ToCompactString(), ResolvedThrow.ThrowSpeed));
}

FRopeWhipGuide::FConfig URopeComponent::MakeWhipGuideConfig() const
{
	FRopeWhipGuide::FConfig Config;
	Config.Duration = WhipConfig.Duration;
	Config.GuidedLength = WhipConfig.GuidedLength;
	Config.SweepAngleDegrees = WhipConfig.SweepAngleDegrees;
	Config.ReferenceThrowSpeed = ThrowParams.ThrowSpeed;
	Config.ComponentRopeLength = RopeLength;
	return Config;
}

float URopeComponent::TailWeightByIndex(int32 NodeIndex, int32 FirstTailNode, int32 LastNode) const
{
	if (LastNode <= FirstTailNode)
	{
		return NodeIndex >= LastNode ? 1.0f : 0.0f;
	}

	const float T = static_cast<float>(NodeIndex - FirstTailNode) / static_cast<float>(LastNode - FirstTailNode);
	return RopeMath::SmoothStep(T);
}

// ===== Flight ===============================================================

FRopeFlightContactDetector::FParams URopeComponent::MakeFlightDetectParams() const
{
	FRopeFlightContactDetector::FParams Params;
	Params.ContactRadius = WrapConfig.ContactRadius;
	Params.RopeRadius = Radius;
	Params.PredictiveContactFrames = WrapConfig.PredictiveContactFrames;
	Params.MinLatchNodes = WrapConfig.MinLatchNodes;
	Params.FallbackForward = GetForwardVector();
	return Params;
}

void URopeComponent::BuildContactingState(const TArray<FRopeContactCandidate>& Candidates)
{
	ContactTracker.Reset();
	ContactTracker.Update(Candidates, 0.0f);
	ContactingElapsed = 0.0f;
	PendingWrapSeed = BuildWrapSeedFromContactingState(Candidates);
}

// ===== Contacting ===========================================================

void URopeComponent::UpdateContacting(float DeltaTime)
{
	// 현재는 체류 타이머만 전진시켜 판정한다.
	// TODO: 매 프레임 접촉 후보를 재수집하고 tangential speed / winding angle까지 갱신.
	AdvanceWrappingMotion(DeltaTime);

	if (ShouldDismissContacting())
	{
		SetPhase(ERopePhase::Flight, TEXT("contact lost before wrapping"));
		ResetTransientPhaseState();
		return;
	}

	if (ShouldStartWrapping())
	{
		StartWrappingFromContacting();
		return;
	}
}

void URopeComponent::AdvanceWrappingMotion(float DeltaTime)
{
	ContactingElapsed += DeltaTime;
}

bool URopeComponent::ShouldDismissContacting() const
{
	return ContactTracker.CandidateBone.IsNone() || ContactTracker.CandidateNodes.Num() == 0;
}

bool URopeComponent::ShouldStartWrapping() const
{
	return ContactingElapsed >= WrapConfig.WrapDecisionTime
		&& PendingWrapSeed.Latched.Num() > 0
		&& !PendingWrapSeed.BoneName.IsNone();
}

FRopeWrapState URopeComponent::BuildWrapSeedFromContactingState(const TArray<FRopeContactCandidate>& Candidates) const
{
	FRopeWrapState Seed;
	Seed.BoneName = ContactTracker.CandidateBone;
	Seed.Mesh = ContactTracker.CandidateMesh;
	const int32 NodeIndex = FindHeadValidNodeIndex(ContactTracker.CandidateNodes, Sim);
	if (NodeIndex != INDEX_NONE)
	{
		FRopeLatchNode Latch;
		Latch.NodeIndex = NodeIndex;
		Latch.Bone = ContactTracker.CandidateBone;
		Seed.Latched.Add(Latch);

		const FRopeContactCandidate* LatchCandidate = nullptr;
		for (const FRopeContactCandidate& Candidate : Candidates)
		{
			if (!Candidate.bValid ||
				Candidate.NodeIndex != NodeIndex ||
				Candidate.Bone != ContactTracker.CandidateBone)
			{
				continue;
			}

			if (!LatchCandidate || Candidate.Penetration > LatchCandidate->Penetration)
			{
				LatchCandidate = &Candidate;
			}
		}

		const USkeletalMeshComponent* Mesh = Seed.Mesh.Get();
		if (!Mesh && LatchCandidate)
		{
			Mesh = LatchCandidate->Mesh;
			Seed.Mesh = Mesh;
		}

		if (LatchCandidate && Mesh)
		{
			const FVector NormalWorld = LatchCandidate->Normal.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
			FVector TangentWorld = FRopeFlightContactDetector::ExpectedWrapTangent(Sim, *LatchCandidate, GetForwardVector());
			if (Sim.Positions.IsValidIndex(NodeIndex + 1))
			{
				TangentWorld = Sim.Positions[NodeIndex + 1] - Sim.Positions[NodeIndex];
			}
			TangentWorld = (TangentWorld - FVector::DotProduct(TangentWorld, NormalWorld) * NormalWorld)
				.GetSafeNormal(KINDA_SMALL_NUMBER, RopeMath::AnyTangentFromNormal(NormalWorld));

			const FTransform BoneXform = Mesh->GetSocketTransform(ContactTracker.CandidateBone);

			FRopeSurfaceAnchor Anchor;
			Anchor.NodeIndex = NodeIndex;
			Anchor.Bone = ContactTracker.CandidateBone;
			Anchor.Mesh = Mesh;
			Anchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(LatchCandidate->WorldPoint);
			Anchor.LocalNormal = BoneXform.InverseTransformVectorNoScale(NormalWorld).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
			Anchor.LocalTangent = BoneXform.InverseTransformVectorNoScale(TangentWorld).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
			Anchor.StartWorldPosition = Sim.Positions[NodeIndex];
			Anchor.SurfaceOffset = FMath::Max(0.0f, Radius);
			Anchor.RopeDistance = 0.0f;
			Seed.Anchors.Add(Anchor);
		}
	}
	return Seed;
}

// ===== Wrapping =============================================================

void URopeComponent::StartWrappingFromContacting()
{
	// PendingWrapSeed를 바로 BeginWrap에 넣지 않고, WrappingPhase 상태로 변환한다.
	WrappingPhase.State.Reset();

	// 감길 메시는 접촉에서 확정된다(FRopeContact.SourceMesh → seed). 여기 비어 있으면 시드가
	// 비정상인 것 — owner 메시로 때우면 cross-actor에서 엉뚱한 본에 붙으므로 폴백 없이 복귀한다.
	const USkeletalMeshComponent* Mesh = PendingWrapSeed.Mesh.Get();
	if (!Mesh || PendingWrapSeed.BoneName.IsNone() || PendingWrapSeed.Latched.Num() == 0)
	{
		SetPhase(ERopePhase::Flight, TEXT("invalid wrapping seed"));
		ResetTransientPhaseState();
		return;
	}

	// M5c: GPU 상주 로프의 CPU 미러는 1~2프레임 낡다 — wrap 핸드오프 순간만 1회 동기 리드백으로
	// 최신 위치를 받아 시드(StartWorldPosition/fallback 앵커)의 정밀도를 확보한다(이벤트당 1회, 블로킹).
	if (URopeSimSubsystem* SimSubsystem = URopeSimSubsystem::Get(GetWorld()))
	{
		SimSubsystem->SyncGpuPositionsForHandoff(*this);
	}

	const FRopeLatchNode& Latch = PendingWrapSeed.Latched[0];	//무조건 첫 번째 latch node 하나만 기준으로 잡는다
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
	else if (Sim.Positions.IsValidIndex(Latch.NodeIndex))
	{
		const FVector NormalWorld = FVector::UpVector;
		FVector TangentWorld = FVector::ForwardVector;

		if (Sim.Positions.IsValidIndex(Latch.NodeIndex + 1))
		{
			// tangent는 가능하면 다음 rope node 방향을 쓴다 — "로프가 tail 방향으로 어느 쪽으로
			// 뻗어 있는가"를 잡기 위한 값으로, 이후 Analytic Helix / Surface Vector Field에서
			// 감기는 방향(WindingSign)을 정할 때 중요하다.
			TangentWorld = (Sim.Positions[Latch.NodeIndex + 1] - Sim.Positions[Latch.NodeIndex])
				.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
		}

		const FTransform BoneXform = Mesh->GetSocketTransform(Latch.Bone);
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

	if (!WrappingPhase.Begin(LatchAnchor, Mesh, PendingWrapSeed.BoneName,
		FMath::Max(0.01f, WrapConfig.WrappingMotionDuration), Sim, MakeWrappingContext()))
	{
		SetPhase(ERopePhase::Flight, TEXT("no valid wrapping anchors"));
		ResetTransientPhaseState();
		return;
	}

	SetPhase(ERopePhase::Wrapping, *FString::Printf(TEXT("bone=%s, %d anchor(s)"),
		*WrappingPhase.State.BoneName.ToString(), WrappingPhase.State.Anchors.Num()));
}

void URopeComponent::UpdateWrapping(float DeltaTime)
{
	WrappingPhase.State.Elapsed += DeltaTime;

	if (!WrappingPhase.IsStillValid())
	{
		SetPhase(ERopePhase::Releasing, TEXT("invalid wrapping state"));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	WrappingPhase.State.LostContactTime = 0.0f;
	const FRopeWrappingPhase::FContext WrappingCtx = MakeWrappingContext();
	WrappingPhase.AdvancePathBuild(Sim, WrappingCtx);
	WrappingPhase.ApplyFrontMotion(Sim, DeltaTime, WrappingCtx, OverrideFrame);

	WrappingPhase.ApplyMassMask(Sim, OverrideFrame);

	WrappingPhase.UpdateStability(DeltaTime);

	if (WrappingPhase.IsReadyToCommit(Sim, WrapConfig))
	{
		CommitWrapping();
		return;
	}
}

ERopeWrappingPathMode URopeComponent::GetWrappingPathMode() const
{
	const UDynamicRopeSettings* Settings = UDynamicRopeSettings::Get();
	return Settings ? Settings->WrappingPathMode : ERopeWrappingPathMode::SurfaceVectorField;
}

FRopeWrappingPhase::FContext URopeComponent::MakeWrappingContext() const
{
	return FRopeWrappingPhase::FContext{ WrapConfig, FrameColliders, GetWrappingPathMode(), Radius, GetName() };
}

void URopeComponent::CommitWrapping()
{
	const USkeletalMeshComponent* Mesh = WrappingPhase.State.Mesh.Get();

	//Wrapping 정보가 적절하지 않으면 바로 releasing
	if (!Mesh || WrappingPhase.State.BoneName.IsNone() || WrappingPhase.State.Anchors.Num() == 0)
	{
		SetPhase(ERopePhase::Releasing, TEXT("commit failed"));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	const FRopeWrapState Seed = WrappingPhase.BuildCommitSeed(Sim, Mesh);
	if (Seed.Anchors.Num() == 0)
	{
		SetPhase(ERopePhase::Releasing, TEXT("no valid latches"));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	WrapController.BeginWrap(Sim, Seed, OverrideFrame); // 감길 mesh는 Seed.Mesh로 전파(접촉 유래, cross-actor 포함).

	SetPhase(ERopePhase::Wrapped, *FString::Printf(TEXT("bone=%s, %d latched node(s)"),
		*Seed.BoneName.ToString(), Seed.Latched.Num()));
	ResetTransientPhaseState();
	OnRopeWrapped.Broadcast(Seed.BoneName);
}

void URopeComponent::AbortWrapping(ERopeReleaseReason Reason)
{
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] AbortWrapping reason=%d"),
		*GetName(), static_cast<int32>(Reason));

	WrappingPhase.ReturnNodesToSolver(Sim, OverrideFrame);

	ResetTransientPhaseState();
	ReleaseCooldown = ReleaseCooldownSeconds;
}

// ===== Wrapped ==============================================================

void URopeComponent::ApplyWrappedMassMask()
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

	OverrideFrame.EnsureSize(Sim.Num());
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		const bool bStartPin = (i == 0 && Sim.bStartPinned);
		const bool bAnchor = AnchorNodes.Contains(i);
		OverrideFrame.SetInvMass(i, (bStartPin || bAnchor) ? 0.0f : 1.0f);
	}
}
