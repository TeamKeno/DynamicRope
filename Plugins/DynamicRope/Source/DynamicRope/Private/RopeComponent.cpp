// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeComponent.h"

#include "Collision/RopeCollider.h"
#include "Debug/RopeDebugDraw.h"
#include "Debug/RopeDebugSnapshot.h"
#include "DynamicRopeLog.h"
#include "Engine/World.h"
#include "Materials/MaterialInterface.h"
#include "Preset/RopePreset.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"
#include "Render/RopeSceneProxy.h"
#include "RopeComponentInternal.h"
#include "RopeGPUSolver.h"
#include "Subsystem/RopeDebugSubsystem.h"
#include "Subsystem/RopeSimSubsystem.h"
#include "UObject/ConstructorHelpers.h"

using RopeComponentPrivate::PhaseName;

#pragma region Construction

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

#pragma endregion Construction

#pragma region Public_API

void URopeComponent::EnterLoaded()
{
	// ③(GuaranteedWrap) 전용 던지기 준비 상태. 창(팁)을 손 소켓에 들고, 로프 튜브 표시는 bShowRopeWhenLoaded을 따른다.
	// 꽂힌 뒤 release로 Free가 된 상태에서만 진입한다(초기 BeginPlay 진입은 예외).
	if (ResolveMode != ERopeWrapResolveMode::GuaranteedWrap)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] EnterLoaded ignored: Loaded은 GuaranteedWrap(③) 전용이다."), *GetName());
		return;
	}
	if (Phase != ERopePhase::Free && Phase != ERopePhase::Loaded)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] EnterLoaded ignored: phase=%s (Free/Loaded에서만 장전 가능)."),
			*GetName(), PhaseName(Phase));
		return;
	}

	// 이미 Loaded이면 재장전(상태 리셋)은 하되 연출 훅은 다시 부르지 않는다 — 프리셋 적용이 ③ 로프에
	// EnterLoaded()을 무조건 호출하는데(ApplyPreset [7]), 그때 SetPhase는 no-op이라 페이즈 이벤트는 안 나가면서
	// OnEnterLoaded만 재발화해 enter/deploy 짝이 어긋났다. 오버라이드가 VFX를 스폰하면 중복 스폰이 된다.
	const bool bAlreadyLoaded = (Phase == ERopePhase::Loaded);

	EnsureRopeInitialized();
	ResetTransientPhaseState();
	ReleaseCooldown = 0.0f;
	EnsureTipMesh();      // 확보 보험 — 정상 경로는 BeginPlay가 이미 잡았다(이미 있으면 no-op).
	if (!bAlreadyLoaded)
	{
		OnEnterLoaded();    // 기본: 로프 튜브 숨김(override 가능). 진입 에지에서만.
	}
	SetPhase(ERopePhase::Loaded, TEXT("reload"));
}

void URopeComponent::SetShowRopeWhenLoaded(bool bShow)
{
	if (bShowRopeWhenLoaded == bShow)
	{
		return;
	}
	bShowRopeWhenLoaded = bShow;

	// Loaded 중이면 즉시 반영한다(가시성 적용 시점이 진입 에지뿐이라, 없으면 다음 장전까지 안 바뀐다).
	// 그 외 페이즈는 전개 상태(항상 표시)라 건드리지 않는다 — 다음 OnEnterLoaded()이 이 값을 소비한다.
	if (Phase == ERopePhase::Loaded)
	{
		SetVisibility(bShowRopeWhenLoaded, /*bPropagateToChildren*/ false);
	}
}

bool URopeComponent::ToggleShowRopeWhenLoaded()
{
	SetShowRopeWhenLoaded(!bShowRopeWhenLoaded);
	return bShowRopeWhenLoaded;
}

void URopeComponent::OnEnterLoaded()
{
	// 기본 구현: bShowRopeWhenLoaded에 따라 로프 튜브 렌더를 켜고 끈다(끔 = 창만 손 소켓에 보인다).
	// 창 위치는 UpdateTipMeshTransform이 Loaded 분기로 처리.
	SetVisibility(bShowRopeWhenLoaded, /*bPropagateToChildren*/ false);
}

void URopeComponent::OnDeployFromLoaded()
{
	// 기본 구현: 로프 튜브를 다시 표시하고, 전개용으로 전체 길이를 복원한다.
	SetVisibility(true, /*bPropagateToChildren*/ false);
	SetRopeLength(RopeLength);
}

bool URopeComponent::ApplyPreset(const URopePreset* Preset)
{
	// [1] 게이트 — 유휴 페이즈(Free/Loaded)에서만 통째 적용이 성립한다. 날아가거나 감고 있는 중의
	// 재구성은 지원 범위 밖(시드/래치/경로가 옛 토폴로지를 물고 있다) — 거부하고 아무것도 안 바꾼다.
	if (!Preset)
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] ApplyPreset ignored: preset이 null이다."), *GetName());
		return false;
	}
	if (Phase != ERopePhase::Free && Phase != ERopePhase::Loaded)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] ApplyPreset('%s') ignored: phase=%s (Free/Loaded에서만 적용 가능)."),
			*GetName(), *Preset->GetName(), PhaseName(Phase));
		return false;
	}
	const bool bWasLoaded = (Phase == ERopePhase::Loaded);

	// [2] 값 스탬프 — RopeMaterial만 세터(SetMaterial) 경유가 필요해 [5]로 미룬다.
	// (인스턴스 배선 값 TipMeshComponentTag/LoadedHandSocket은 프리셋에 없다 — 헤더 주석 참조.)
	ResolveMode = Preset->ResolveMode;
	NumParticles = Preset->NumParticles;
	RopeLength = Preset->RopeLength;
	MinRopeLength = Preset->MinRopeLength;
	ReelSpeed = Preset->ReelSpeed;
	SolverConfig = Preset->SolverConfig;
	ThrowParams = Preset->ThrowParams;
	WrapConfig = Preset->WrapConfig;
	HoldConfig = Preset->HoldConfig;
	WhipConfig = Preset->WhipConfig;
	bUseTipMesh = Preset->bUseTipMesh;
	TipMesh = Preset->TipMesh;
	TipMeshRelativeTransform = Preset->TipMeshRelativeTransform;
	bTipMeshCollision = Preset->bTipMeshCollision;
	bSyncTipMeshOnFree = Preset->bSyncTipMeshOnFree;
	bUseTipMeshSockets = Preset->bUseTipMeshSockets;
	TipSocketName = Preset->TipSocketName;
	TipRopeSocketName = Preset->TipRopeSocketName;
	Radius = Preset->Radius;
	NumSides = Preset->NumSides;
	TubeSmoothingSubdiv = Preset->TubeSmoothingSubdiv;
	TubeSmoothingAlpha = Preset->TubeSmoothingAlpha;
	bIncludeOwnerColliders = Preset->bIncludeOwnerColliders;
	bUseWorldGDF = Preset->bUseWorldGDF;
	PreviewReachScale = Preset->PreviewReachScale;
	PreviewSegmentCount = Preset->PreviewSegmentCount;
	PreviewSampleStep = Preset->PreviewSampleStep;
	PreviewQueryRadius = Preset->PreviewQueryRadius;

	// [4] Sim 재시드 — 항상 호출(분기 없는 단일 경로). NumParticles/RopeLength 소비 + GPU 상주 버퍼
	// 재시드 세대 증가까지 포함한다. EnsureRopeInitialized는 비었을 때만이라 여기서는 부적합.
	InitRope();

	// [5] 렌더 — RopeMaterial은 SetMaterial 경유가 계약(씬 프록시가 생성 시점에 머티리얼을 캡처하므로
	// MarkRenderStateDirty로 프록시를 재생성해야 반영된다). 프록시 1회 소비 값(Radius/NumSides/
	// TubeSmoothing*)도 같은 MarkRenderStateDirty로 반영 — 에디터 PostEditChangeProperty의 런타임 등가물.
	SetMaterial(0, Preset->RopeMaterial);

	// [6] 팁 재구성 — EnsureTipMesh는 기존 컴포넌트가 있으면 no-op이라, 에셋 교체/on↔off를 반영하려면
	// 먼저 내려야 한다(스폰분만 파괴 — 태그 재사용 컴포넌트는 보존, 필요하면 Ensure가 재획득).
	TeardownSpawnedTipMesh();
	EnsureTipMesh();

	// [7] 모드-페이즈 정합 — ③은 Loaded(장전)에서만 던질 수 있으므로 즉시 장전한다(BeginPlay와 같은 규약).
	// 반대로 Loaded이었는데 ①②가 되면 Reel이 무의미해지므로 전개(가시성/길이 복원) 후 Free로 돌린다.
	if (ResolveMode == ERopeWrapResolveMode::GuaranteedWrap)
	{
		EnterLoaded();
	}
	else if (bWasLoaded)
	{
		OnDeployFromLoaded();
		SetPhase(ERopePhase::Free, TEXT("preset applied"));
	}

	// [8] 통지 — 네이티브 훅 먼저, 그다음 BP 델리게이트(엔진 Notify 관례).
	NotifyPresetApplied(Preset);
	OnPresetApplied.Broadcast(Preset);
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] preset '%s' applied (mode=%d, N=%d, L=%.0f)."),
		*GetName(), *Preset->GetName(), (int32)ResolveMode, NumParticles, RopeLength);
	return true;
}

#pragma endregion Public_API

#pragma region Simulation_Frame_Pipeline

// ===== 시뮬레이션 프레임(서브시스템이 3단계로 구동) ===========================

void URopeComponent::PrepareSimFrame(float DeltaTime, const TOptional<FVector>& LODCameraLocation)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_Prepare);
	// (분리 계약 — 이 함수는 "솔브 입력 생산" 단계다: 솔브 결과가 필요 없는 로직은 전부 여기.
	//  근거와 3단계 역할 분담은 헤더의 Prepare/Solve/Finalize 선언부 주석 참고.)

	EnsureRopeInitialized();
	// 프레임 스코프 — 이번 프레임 로직 산출물을 새로 모은다(G2).
	SimFrame.OverrideFrame.Reset();
	SimFrame.bForceNonStretchThisFrame = false;
	const ERopePhase PhaseAtPrepareStart = Phase;
	bEnteredFlightDuringPrepareThisFrame = false;
#if WITH_GAMEPLAY_DEBUGGER
	// 폴백 — 보통은 서브시스템이 Phase 1a에서 이미 잡았고(그쪽이 pending aim throw보다 앞선다) 이 호출은
	// no-op이다. Phase 1a를 건너뛴 로프만 여기서 처음 기록된다.
	CaptureDebugFrameStartPhase();
#endif

	// pinned-start target을 전진시킨다; solver가 substep에 걸쳐 Prev->Target을 sweep하므로 빠른
	// 캐릭터 이동이 chain을 홱 잡아당겨(폭주시켜) 버리지 않는다.
	if (Sim.bStartPinned)
	{
		Sim.StartPinPrev = Sim.StartPinTarget;
		Sim.StartPinTarget = GetComponentLocation();
	}

	// collider 스냅샷은 RopeSimSubsystem이 Tick의 collider 단계에서 중앙 수집해 FrameColliders에 채워둔다
	// (Prepare 이전). 여기서 로프마다 provider를 탐색/gather하지 않는다.

	// 되감기(reel): 솔브 전에 이번 프레임 길이 변화를 반영한다(세그먼트 rest 길이 균일 변경 —
	// 재시드 없이 CPU/GPU 동일 적용). Wrapped에서는 가용 로프 길이가 줄어 테더/장력으로 전파된다.
	UpdateReel(DeltaTime);

	// 거리 LOD 배율(iteration 감쇠) — 솔브 판정 전에 이번 프레임 값 확정(GPU 스텝/CPU 솔브 공용).
	ComputeSolverLOD(LODCameraLocation);

	// 슬립은 Free/Wrapped 전용 — 그 외 페이즈로 넘어가면 즉시 해제(전이 자체가 활동).
	if (Phase != ERopePhase::Free && Phase != ERopePhase::Wrapped && Throttle.IsAsleep())
	{
		Throttle.Wake();
	}

	SimFrame.bSolveThisFrame = false;
	SimFrame.bSolveCollisionsThisFrame = true;

	switch (Phase)
	{
	// 손에서 늘어뜨려진 채 캐릭터를 따라간다
	case ERopePhase::Free:
		if (Throttle.IsAsleep() && Throttle.ShouldWakeFromSleep(Sim, SolverConfig, ReelRate, SimFrame.FrameColliders))
		{
			Throttle.Wake();
		}
		// 슬립 중엔 솔브 스킵(GPU 로프는 dispatch 자체가 없음).
		SimFrame.bSolveThisFrame = !Throttle.IsAsleep();
		break;

	case ERopePhase::Flight:
	{
		if (WhipGuide.IsActive())
		{
			// 타깃/마스크 계산만(Sim 불변) — 적용은 CPU 경로 SolveSimFrame(ApplyToSim) 또는
			// GPU 상주 경로의 override 패스(서브시스템이 step에 실음)가 담당한다(G1).
			WhipGuide.Advance(DeltaTime, Sim, MakeWhipGuideConfig());
		}
		else
		{
			WhipGuide.ResetFrameOutputs();
		}

		if (AimTargeting.IsLockActive(Phase))
		{
			// 중앙 guide를 적용한 뒤 양끝은 XPBD 거리/굽힘/감쇠로 자연스럽게 연결한다.
			// collider push-out은 별도 게이트로 꺼서 이전의 충돌 순간이동을 재발시키지 않는다.
			SimFrame.bSolveThisFrame = true;
			SimFrame.bSolveCollisionsThisFrame = !WhipConfig.bAimHitCollisionFreeSolve;
		}
		else
		{
			// 일반 Flight는 기존처럼 solver 후 Finalize에서 접촉을 감지한다.
			SimFrame.bSolveThisFrame = true;
		}
		break;
	}

	case ERopePhase::Contacting:
		UpdateContacting(DeltaTime);
		break;

	case ERopePhase::Wrapping:
		UpdateWrapping(DeltaTime);
		// 실제 wrapping position override/anchor 노드만 mass mask로 고정한다. 아직 경로가 닿지 않은
		// tail은 solver가 계속 처리해 Flight에서 남은 strain을 Wrapping 중 해소한다.
		SimFrame.bSolveThisFrame = (Phase == ERopePhase::Wrapping || Phase == ERopePhase::Wrapped);
		break;

	case ERopePhase::Wrapped:
	{
		// Wrapped 틱 = 4단계 고정 순서: ① 본 추종(Hold + 질량 마스크 — 대상 소실 시 release)
		// → ② 관측치 산출(장력 + Pull 샘플/스무딩 — ③④의 공용 입력) → ③ 견인 인가(테더 + 능동 Pull)
		// → ④ 자동 release 판정(장력 지속 초과 / 거리 초과 — ②③의 산출물을 소비).
		if (!HoldWrappedNodesToBone(DeltaTime))
		{
			// 대상 mesh 소실 — release 완료(솔브 없음).
			break;
		}
		// Hold는 current bone 위치를 OverrideFrame에만 기록하고 실제 Sim 적용은 Prepare 끝의 단일
		// ApplyToSim 계약까지 미룬다. Pull/tether가 직전 solve pose를 읽지 않도록 별도 관측 view에
		// 현재 손 pin + Hold override를 먼저 합성한다. GPU 내부 자유 노드는 mirror일 수 있지만,
		// movement hard constraint는 이 view가 아니라 live CPU binding descriptor를 사용한다.
		PullObservationSim = Sim;
		if (PullObservationSim.Positions.IsValidIndex(0))
		{
			PullObservationSim.Positions[0] = Sim.StartPinTarget;
			if (PullObservationSim.PrevPositions.IsValidIndex(0))
			{
				PullObservationSim.PrevPositions[0] = Sim.StartPinTarget;
			}
		}
		if (SimFrame.OverrideFrame.HasAny())
		{
			SimFrame.OverrideFrame.ApplyToSim(PullObservationSim);
		}
		UpdateWrappedPullSample(DeltaTime, PullObservationSim);
		ApplyWrappedTraction(DeltaTime);
		if (CheckWrappedAutoRelease(DeltaTime))
		{
			// 장력/거리 release 발생(솔브 없음).
			break;
		}
		// Wrapped 정지 스로틀(Free 슬립의 확장): 핀·랩 본·근접 콜라이더가 모두 정지하면 자유 구간 솔브를
		// 쉰다. 위 ①~④(Hold/관측/견인/자동 release)는 수면 중에도 그대로 돈다 — 스킵되는 건 solver
		// substep뿐이라 GPU는 override-only dispatch(NumSub=0)로 줄어든다. 진입은 Finalize의
		// UpdateSleepState(속도 침전, 능동 Pull 장전 중엔 bHoldAwake로 차단)가 공용 처리. 랩 본 이동은
		// Hold가 쓴 노드 드리프트로 깨우는데, 관측 view(PullObservationSim)가 이번 프레임 핀+override를
		// 이미 합성했으므로 지연 없이 감지된다(엘리베이터가 출발하는 그 프레임에 wake).
		if (Throttle.IsAsleep()
			&& (PullDrive.ActivePullForce > 0.0f
				|| Throttle.ShouldWakeFromSleep(PullObservationSim, SolverConfig, ReelRate, SimFrame.FrameColliders)))
		{
			Throttle.Wake();
		}
		SimFrame.bSolveThisFrame = !Throttle.IsAsleep();
		break;
	}

	case ERopePhase::GuidedThrow:
		// ③ 전용 phase. 물리 solver/contact detector를 건너뛰고 확정 path(조준) 또는 아치(허공)만 따른다.
		UpdateGuidedThrow(DeltaTime);
		SimFrame.bSolveThisFrame = false;
		break;

	case ERopePhase::Loaded:
	{
		// 던지기 준비 상태(③ 전용): 창(팁)은 손 소켓에 고정하고, 그 뒤 로프는 물리로 자연스럽게 늘어뜨린다.
		// node 0은 위의 pin 로직(StartPinTarget)이 손을 따라가므로, 팁만 소켓에 고정하면 사이 로프가 처진다.
		// 팁 고정이 필요한 이유: 던지는 순간 팁 렌더가 소켓 → 마지막 노드로 바뀌므로(UpdateTipMeshTransform),
		// 마지막 노드가 소켓에 있어야 창이 튀지 않는다. 솔브를 켜야 프리즈 없이 캐릭터를 따라간다.
		// (Wrapped의 Hold와 동일 패턴: 위치 + InvMass=0 override → 나머지 노드는 솔버가 굴린다.)
		const int32 LoadedTipNode = Sim.Num() - 1;
		const FTransform LoadedTipWorld = GetLoadedTipTransform();
		SimFrame.OverrideFrame.EnsureSize(Sim.Num());
		SimFrame.OverrideFrame.SetPosition(LoadedTipNode, ResolveTipRopeAttachWorld(LoadedTipWorld), /*bZeroVelocity*/ true);
		SimFrame.OverrideFrame.SetInvMass(LoadedTipNode, 0.0f);
		SimFrame.bSolveThisFrame = true;
		break;
	}

	case ERopePhase::Releasing:
		// 모든 node를 solver에 다시 넘긴다(hand pin만 유지) — InvMass 복원 + Prev=Pos(튐 방지)를
		// 프레임 산출물로 담고, cooldown이 끝나면 free simulation을 재개한다.
		SimFrame.OverrideFrame.EnsureSize(Sim.Num());
		for (int32 i = 0; i < Sim.Num(); ++i)
		{
			SimFrame.OverrideFrame.SetInvMass(i, (i == 0 && Sim.bStartPinned) ? 0.0f : 1.0f);
			SimFrame.OverrideFrame.SetPrevFromPosition(i);
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

	// Prepare 시작부터 Flight였던 프레임만 정상적인 Flight Advance/Solve 입력을 가졌다.
	// 로직 처리 중 Flight로 돌아온 경우 Finalize 재캡처를 미뤄 다음 프레임에 guide를 먼저 재개한다.
	bEnteredFlightDuringPrepareThisFrame =
		PhaseAtPrepareStart != ERopePhase::Flight && Phase == ERopePhase::Flight;

	// 로직 페이즈의 프레임 산출물을 CPU Sim에 1회 적용한다 — 기존 "핸들러 안에서 직접 쓰기"와
	// 같은 결과(같은 노드 중복 시 나중 fill이 이김 = 순차 쓰기와 동일). GPU 상주 로프에는
	// 서브시스템이 같은 프레임을 override 패스로 실어 커널에서 적용한다(G2).
	// 로직 페이즈 재시드(SimFrame.SimGeneration 증가)는 소멸 — 재시드는 진짜 시드(Init/Throw)뿐이다.
	if (SimFrame.OverrideFrame.HasAny())
	{
		SimFrame.OverrideFrame.ApplyToSim(Sim);
	}
}

void URopeComponent::SolveSimFrame(float DeltaTime)
{
	// 병렬 단계: POD 상태(Sim) + collider 스냅샷(SimFrame.FrameColliders)만 만진다. Query는 const → 스레드 안전.
	// SimFrame.bSolveThisFrame(Free/Flight/Wrapping/Wrapped)일 때만 물리 솔브 — Wrapping/Wrapped는
	// 고정 노드가 InvMass=0이라 자유 구간만 움직이고, Contacting/Releasing은 로직 구동이라 스킵.
	if (!SimFrame.bSolveThisFrame)
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

	// 거리 LOD: 원거리에서 constraint iteration만 감쇠(substep은 유지 — 안정성은 substep이 지배).
	FRopeSolverConfig LODConfig = SolverConfig;
	LODConfig.Iterations = GetLODScaledIterations();
	LODConfig.MaxStretchRatio = GetEffectiveMaxStretchRatio();
	// 반지름 auto(0=렌더 Radius) 해석 — 솔버는 항상 해석된 값만 받는다(GPU step은 서브시스템이 동일 처리).
	LODConfig.CollisionRadius = GetEffectiveCollisionRadius();
	// Aim-hit collision-free solve도 solver 자체는 실행하되 빈 목록을 넘겨 push-out만 제외한다.
	const TArray<IRopeCollider*> NoSolveColliders;
	const TArray<IRopeCollider*>& SolveColliders = SimFrame.bSolveCollisionsThisFrame
		? SimFrame.FrameColliders
		: NoSolveColliders;
	Solver.Step(Sim, LODConfig, SolveColliders, DeltaTime);
}

float URopeComponent::GetEffectiveMaxStretchRatio() const
{
	// ①/②의 물리 던지기는 가이드 target과 wrapping position override를 solver 입력으로 쓴다.
	// 이 구간에서 허용 신장을 남기면 kinematic 노드가 풀리는 순간 rest length로 되감기므로,
	// 실제 resident pose를 푸는 CPU/GPU strain-limit 단계에서 완전 비신축으로 제한한다.
	const bool bPhysicalResolveMode = ResolveMode != ERopeWrapResolveMode::GuaranteedWrap;
	const bool bThrowOrWrapFrame =
		Phase == ERopePhase::Flight ||
		Phase == ERopePhase::Wrapping ||
		SimFrame.bForceNonStretchThisFrame;
	// Once held, the visual solver must obey the same material contract as movement and
	// reaction tension. A rigid authoritative hold cannot leave MaxStretchRatio=1.5 as a
	// second, hidden elasticity source. Positive TetherCompliance explicitly opts back into
	// the configured visual stretch policy.
	const bool bRigidAuthoritativeHold =
		HoldConfig.bEnforceWielderLengthConstraint &&
		HoldConfig.TetherCompliance <= KINDA_SMALL_NUMBER &&
		(Phase == ERopePhase::Wrapping || Phase == ERopePhase::Wrapped);
	return (bPhysicalResolveMode && bThrowOrWrapFrame) || bRigidAuthoritativeHold
		? 1.0f
		: SolverConfig.MaxStretchRatio;
}

void URopeComponent::FinalizeSimFrame(float DeltaTime)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_Finalize);

	// (분리 계약 — 이 함수는 "솔브 출력 소비" 단계다. 근거는 헤더의 3단계 선언부 주석 참고.)
	// 디버그 캡처 게이트: 이 로프가 게이트플레이 디버거의 대상 액터일 때만 비주얼 데이터를 모은다.
	// 대상이 아닌 로프는 flight sweep 등 캡처 비용을 전혀 내지 않는다(타깃 1개 로프만 부담).
#if WITH_GAMEPLAY_DEBUGGER
	URopeDebugSubsystem* DebugSub = URopeDebugSubsystem::Get(GetWorld());
	const bool bDebugCapture = DebugSub && DebugSub->ShouldCapture(this);
	// 켜진 보기만 수집한다. 특히 Flight 비트가 없으면 아래 관측 단계가 노드마다 돌리는 재스윕 자체를
	// 건너뛴다 — 그리기에서만 막으면 이 비용이 그대로 남는다.
	const ERopeDebugCapture CaptureMask = bDebugCapture ? DebugSub->GetCaptureMask() : ERopeDebugCapture::None;
	FRopeDebugSnapshot DebugSnapshot;
	FRopeDebugSnapshot* const FlightSnapshot =
		EnumHasAnyFlags(CaptureMask, ERopeDebugCapture::Flight) ? &DebugSnapshot : nullptr;
#else
	constexpr bool bDebugCapture = false;
	FRopeDebugSnapshot* const FlightSnapshot = nullptr;
#endif

	// Flight: 솔브 후 이동 경로 기반 접촉 후보 감지 → 캡처. 파이프라인 자체는
	// FRopeFlightContactDetector(UObject 비의존)이고, 여기서는 3단계 오케스트레이션만 한다.
	// 스탯/디버거 소비는 전부 ③ 안에 있다 — 본문에는 판정 흐름만 남긴다.
	if (Phase == ERopePhase::Flight && !bEnteredFlightDuringPrepareThisFrame)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FinalizeFlight);
		const FRopeFlightContactDetector::FParams DetectParams = MakeFlightDetectParams(DeltaTime);

		// ① 후보 산출
		TArray<FRopeContactCandidate>& Candidates = GetOrBuildFlightContactCandidates(DeltaTime, DetectParams);

		// ② 판정 결과를 한 번 만들고 게임 전이와 관측이 같은 tracker를 소비한다.
		FRopeFlightCaptureEvaluation CaptureEvaluation = EvaluateFlightCapture(Candidates, DetectParams);
		const bool bShouldCapture = ApplyFlightCaptureEvaluation(DeltaTime, Candidates, CaptureEvaluation);
		const FRopeContactTracker& FrameTracker = bShouldCapture
			? ContactTracker
			: CaptureEvaluation.Tracker;
		// ③ 관측
		RecordFlightObservation(DetectParams, Candidates, FrameTracker, bShouldCapture, FlightSnapshot);
	}

	// 실제 centerline/GPU 소스/component transform이 달라진 프레임만 render data를 다시 민다.
	// 정지 Free/Contacting처럼 solve·override가 없고 transform도 그대로인 로프는 render command를 만들지 않는다.
	const FTransform CurrentComponentTransform = GetComponentTransform();
	const bool bRenderTransformChanged = !bHasLastRenderDataComponentTransform ||
		!LastRenderDataComponentTransform.Equals(CurrentComponentTransform);
	const bool bGpuResidentChanged = bLastRenderDataGpuResident != SimFrame.bGpuSteppedThisFrame;
	const bool bRenderDataChanged = SimFrame.bSolveThisFrame || SimFrame.OverrideFrame.HasAny() ||
		bGpuResidentChanged || bRenderTransformChanged;
	if (bRenderDataChanged)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_MarkRenderDynamicDataDirty);
		MarkRenderDynamicDataDirty();
	}
	if (bRenderTransformChanged)
	{
		MarkRenderTransformDirty();
	}
	LastRenderDataComponentTransform = CurrentComponentTransform;
	bHasLastRenderDataComponentTransform = true;
	bLastRenderDataGpuResident = SimFrame.bGpuSteppedThisFrame;

	// 팁 부착물(창날/작살)을 확정된 자유단 위치로 추종시킨다(솔브 출력 소비 단계라 여기).
	UpdateTipMeshTransform();

	// wrapped stat 카운터(독립).
	if (Phase == ERopePhase::Wrapped)
	{
		RopeDebug::RecordWrappedStats(Sim, WrapController.State);
	}

	// 슬립 전이 측정(Free/Wrapped — 프레임간 노드 변위 기반이라 이번 프레임 결과가 확정된 여기서).
	// Wrapped에서 능동 Pull이 장전된 동안은 진입을 막는다(bHoldAwake — 견인 임펄스/climb-in은 적분 전제).
	if (Throttle.UpdateSleepState(Phase, Sim, SolverConfig, DeltaTime,
		/*bHoldAwake*/ Phase == ERopePhase::Wrapped && PullDrive.ActivePullForce > 0.0f))
	{
		UE_LOG(LogDynamicRope, Verbose, TEXT("[%s] rope asleep (max speed < %.1f cm/s for %.2fs)"),
			*GetName(), SolverConfig.SleepVelocityThreshold, SolverConfig.SleepDelay);
	}

	// 디버그 스냅샷 제출: centerline/collider/wrapped 공통 필드를 채워 디버거 보관소로 넘긴다.
#if WITH_GAMEPLAY_DEBUGGER
	if (bDebugCapture)
	{
		FillDebugSnapshot(DebugSnapshot, CaptureMask);
		DebugSub->SubmitSnapshot(this, MoveTemp(DebugSnapshot));
	}
#endif
}

#pragma endregion Simulation_Frame_Pipeline

#pragma region Component_Lifecycle

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

	// 팁 부착물 확보(bUseTipMesh가 켜진 경우만). 수명 = BeginPlay~EndPlay, 결착 모델·도달 모드 무관 —
	// Free에서도 팁이 로프 끝에 보이려면 여기서 확보돼 있어야 한다. Sim을 읽지 않아 초기화 순서 의존이 없다.
	EnsureTipMesh();

	// ③ Guaranteed 로프는 던지기 준비(Loaded) 상태로 시작한다 — 창을 손에 든 채 대기(로프 숨김).
	if (ResolveMode == ERopeWrapResolveMode::GuaranteedWrap)
	{
		EnterLoaded();
	}
}

void URopeComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// 부착 상태로 파괴/제거되면 release 이벤트를 발화해 소비자(잡힘 카운터·랙돌 오복구)가 상태에 고착되지
	// 않게 한다. 월드 통째 teardown(Quit/LevelTransition/에디터 종료)에선 구독자도 함께 죽으므로 생략한다.
	if (EndPlayReason == EEndPlayReason::Destroyed || EndPlayReason == EEndPlayReason::RemovedFromWorld)
	{
		if (Phase == ERopePhase::Wrapped)
		{
			// 커밋된 wrap이 파괴됨 → per-instance + 중앙 신호(cross-actor 대상이 살아 있으면 랙돌 복구가 필요).
			DispatchReleased(WrapController.State.Mesh.Get(), WrapController.State.BoneName,
				ERopeReleaseReason::Broken, /*bWasWrapped*/ true);
		}
		else if (Phase == ERopePhase::Contacting || Phase == ERopePhase::Wrapping ||
			(Phase == ERopePhase::GuidedThrow && !GuidedThrowState.bFreeThrow))
		{
			// 성립 전 engagement가 파괴됨 → per-instance만(짝 맞춤). 조준된 ③ 던지기도 engagement를 연
			// 것으로 본다(대상을 잡은 시점부터 — DispatchReleased 계약). 허공 던지기는 대상이 없어 제외.
			FName Bone = NAME_None;
			if (Phase == ERopePhase::Wrapping)          { Bone = WrappingPhase.State.BoneName; }
			else if (Phase == ERopePhase::GuidedThrow)  { Bone = GuidedThrowState.Prepared.Bone; }
			else                                        { Bone = ContactTracker.CandidateBone; }
			DispatchReleased(nullptr, Bone, ERopeReleaseReason::Broken, /*bWasWrapped*/ false);
		}
	}

	// 우리가 스폰한 팁 부착물 정리(외부 컴포넌트는 보존). 수명 = BeginPlay~EndPlay라 파괴는 여기 한 곳뿐이다.
	TeardownSpawnedTipMesh();
	TeardownPhysicalTether(); // 물리 제약 테더 정리(phase 전이를 안 거치는 파괴 경로 대비).

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
	// M5b: GPU step된 프레임만 resident PosBuf 직접 렌더 허용.
	DynamicData->bGpuResident = SimFrame.bGpuSteppedThisFrame;
	// 프록시가 resident 버퍼의 세대와 대조한다(같은 노드 수 재시드의 한 프레임 유령 방지).
	DynamicData->SimGeneration = SimFrame.SimGeneration;
	// resident 튜브의 월드→로컬 변환도 이 GT 트랜스폼으로 — Points 로컬화와 같은 프레임의 값이라 드로우
	// 트랜스폼과 일치한다(프록시 GetLocalToWorld()는 SetDynamicData 시점에 한 프레임 이전 값 — 헤더 주석 참고).
	DynamicData->WorldToLocal = FMatrix44f(Xform.ToInverseMatrixWithScale());
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
	else if (PropertyName == GET_MEMBER_NAME_CHECKED(URopeComponent, bTipMeshCollision))
	{
		// PIE 중 토글 시 현재 팁에 즉시 반영(팁이 없으면 no-op — 다음 확보 때 적용된다).
		ApplyTipMeshCollision();
	}

	Super::PostEditChangeProperty(PropertyChangedEvent);
}
#endif

#pragma endregion Component_Lifecycle

#pragma region Rendering

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
	return RopeMaterial;
}

void URopeComponent::SetMaterial(int32 /*ElementIndex*/, UMaterialInterface* Material)
{
	RopeMaterial = Material;
	// 씬 프록시가 생성 시점에 머티리얼을 캡처하므로, 교체를 반영하려면 프록시를 재생성한다.
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

#pragma endregion Rendering

#pragma region Phase_State_Machine

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
	const ERopePhase OldPhase = Phase;
	Phase = NewPhase;

	// 확장 훅 + BP 이벤트(실제 전이만 — 같은 페이즈 재설정은 알리지 않는다).
	if (OldPhase != NewPhase)
	{
		OnPhaseChanged(OldPhase, NewPhase);
		OnRopePhaseChanged.Broadcast(OldPhase, NewPhase);
	}
}

void URopeComponent::ResetTransientPhaseState(bool bPreservePhysicalTether)
{
	AimTargeting.ResetPendingThrow();
	PendingGuaranteedAimThrow.Reset();
	ContactTracker.Reset();
	PendingWrapSeed.Reset();
	bPendingGpuCaptureHandoff = false;
	if (!bPreservePhysicalTether)
	{
		TeardownPhysicalTether(); // abort/release/new throw/end play: attempt-scoped constraint.
	}
	CaptureTravelFrame.Reset();
	WrappingPhase.State.Reset();
	GuidedThrowState.Reset();
	ContactingElapsed = 0.0f;
	FlightNoContactElapsed = 0.0f;
	TensionOverTime = 0.0f;
	// Pull 샘플/EMA 3종/경고 래치만. 생존 필드는 FRopePullDriveState 주석 참조.
	PullDrive.ResetTransient();
	LengthConstraintState.ResetTransient();
}

#pragma endregion Phase_State_Machine

#pragma region Initialization_And_Debug

// ===== 초기화/유틸 ===========================================================

void URopeComponent::InitRope()
{
	// GPU 솔버 상한(스레드그룹 = MaxNodes)을 넘으면 조용히 CPU 솔브+튜브 폴백이 되어 성능 절벽이 된다.
	// 에디터 ClampMax와 별개로 BP/코드 경로도 하드 클램프한다 — 값을 써 넣어 프록시 NumNodes(= NumParticles)와
	// Sim 크기가 일치하도록(불일치 시 BuildTube가 스킵된다). 초과 시 1회 경고.
	if (NumParticles > FRopeGPUSolver::MaxNodes)
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] NumParticles %d exceeds the GPU solver cap %d; clamping (values above the cap fall back to CPU solve+tube)."),
			*GetName(), NumParticles, FRopeGPUSolver::MaxNodes);
		NumParticles = FRopeGPUSolver::MaxNodes;
	}
	const int32 N = FMath::Max(2, NumParticles);
	Sim.Reset();
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
		Sim.SetStill(i);
		Sim.InvMass[i] = 1.0f;
	}

	// 시작점을 컴포넌트(hand/socket)에 pin한다; solver가 substep에 걸쳐 이를 sweep한다.
	Sim.InvMass[0] = 0.0f;
	Sim.bStartPinned = true;
	Sim.StartPinTarget = Start;
	Sim.StartPinPrev = Start;

	// Sim 전면 재구성 → GPU 상주 버퍼 재시드(M5).
	++SimFrame.SimGeneration;
	bWrappedMassMaskDirty = true;

	UE_LOG(LogDynamicRope, Verbose, TEXT("[%s] InitRope: %d particles, length=%.1f, segment=%.2f"),
		*GetName(), N, Sim.RopeLength, Sim.SegmentLength);
}

void URopeComponent::EnsureRopeInitialized()
{
	if (Sim.Num() == 0)
	{
		InitRope();
	}
}

#pragma endregion Initialization_And_Debug

#pragma region Rope_Length_Reel_And_LOD

void URopeComponent::SetRopeLength(float NewLength)
{
	if (Sim.Num() < 2)
	{
		return;
	}
	// 상한 = 초기(디자이너) 길이 — 풀기는 감았던 만큼만 되돌린다. 하한 = MinRopeLength.
	const float MaxLen = FMath::Max(RopeLength, MinRopeLength);
	const float Clamped = FMath::Clamp(NewLength, FMath::Min(MinRopeLength, MaxLen), MaxLen);
	if (FMath::IsNearlyEqual(Clamped, Sim.RopeLength))
	{
		return;
	}
	Sim.RopeLength = Clamped;
	Sim.SegmentLength = Clamped / static_cast<float>(Sim.Num() - 1);
}

void URopeComponent::SetReelRate(float CmPerSecond)
{
	ReelRate = CmPerSecond;
}

void URopeComponent::ComputeSolverLOD(const TOptional<FVector>& CameraLocation)
{
	// 카메라 UObject 조회는 RopeSimSubsystem이 프레임당 한 번 수행한다. 여기서는 로프별 거리 계산만 한다.
	// 로컬 플레이어 0번 기준이며 서버/카메라 없음 = 풀 품질.
	TOptional<float> CameraDist;
	if (SolverConfig.bEnableDistanceLOD && SolverConfig.LODStartDistance > 0.0f && CameraLocation.IsSet())
	{
		CameraDist = static_cast<float>(FVector::Dist(CameraLocation.GetValue(), GetComponentLocation()));
	}
	Throttle.ComputeSolverLOD(SolverConfig, CameraDist);
}

void URopeComponent::UpdateReel(float DeltaTime)
{
	if (FMath::IsNearlyZero(ReelRate))
	{
		return;
	}
	// Contacting/Wrapping/Releasing은 보류: wrapping 경로 생성/커밋이 SegmentLength 기반 거리
	// (RopeDistance = idx × SegmentLength)를 쓰는 중이라 밑에서 눈금을 바꾸면 경로가 뒤틀린다.
	if (Phase != ERopePhase::Free && Phase != ERopePhase::Flight && Phase != ERopePhase::Wrapped)
	{
		return;
	}
	float NewLength = Sim.RopeLength - ReelRate * DeltaTime;
	// 감기(+)의 실현 가능성 실속(stall): 비신축 로프의 재질 길이는 손↔앵커 직선 거리(필요 길이의 하한)보다
	// 짧아질 수 없다. 대상이 무겁거나 걸려 안 끌려오는데 릴이 계속 감으면 위반 C가 무한 누적되고, 시뮬
	// 대상의 하드 Chaos 리밋이 매 서브스텝 그 대형 위반을 관절과 싸우며 닫으려 해 랙돌이 요동한다
	// (스네어 양팔 결박 바둥거림, 2026-07-24). 윈치가 하중에 실속하듯 "대상이 실제로 끌려온 만큼만" 따라
	// 감는다 — 위반이 릴 프레임 스텝 수준(슬랙)으로 유계가 되어 당김 바이어스는 남고 싸움은 사라진다.
	// 풀기(-)와 자기 랩/제약 비활성(live 바인딩 없음)은 종전 그대로.
	if (ReelRate > 0.0f && Phase == ERopePhase::Wrapped)
	{
		FRopeWielderMovementConstraint LiveConstraint;
		if (BuildWielderMovementConstraint(LiveConstraint))
		{
			const float RequiredLength = static_cast<float>(
				FVector::Distance(GetComponentLocation(), LiveConstraint.PivotWorld));
			// 슬랙 = 릴 2프레임 스텝(최소 1cm): 정상 견인(대상이 릴 속도로 따라오는 중)은 건드리지 않는다.
			const float StallSlack = FMath::Max(ReelRate * DeltaTime * 2.0f, 1.0f);
			// 하한은 "더 못 감"이지 "되풀기"가 아니다 — 위반이 이미 커도 길이를 늘리진 않는다(현재 길이 상한).
			NewLength = FMath::Max(NewLength, FMath::Min(RequiredLength - StallSlack, Sim.RopeLength));
		}
	}
	SetRopeLength(NewLength);
}

#pragma endregion Rope_Length_Reel_And_LOD
