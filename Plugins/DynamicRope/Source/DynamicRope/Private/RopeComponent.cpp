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
#include "GameFramework/Character.h"                    // Pull: 캐릭터 견인(CharacterMovement AddForce)
#include "GameFramework/CharacterMovementComponent.h"
#include "Camera/PlayerCameraManager.h"                 // 거리 LOD(카메라 거리 기준 iteration 감쇠)
#include "Kismet/GameplayStatics.h"
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

	void InjectPinnedFrameVelocityForFreeReturn(FRopeSimState& Sim)
	{
		if (!Sim.bStartPinned || Sim.Num() < 2)
		{
			return;
		}

		const FVector PinDelta = Sim.StartPinTarget - Sim.StartPinPrev;
		if (PinDelta.IsNearlyZero())
		{
			return;
		}

		for (int32 i = 1; i < Sim.Num(); ++i)
		{
			if (!Sim.PrevPositions.IsValidIndex(i) || !Sim.InvMass.IsValidIndex(i) || Sim.InvMass[i] <= 0.0f)
			{
				continue;
			}

			Sim.PrevPositions[i] -= PinDelta;
		}
	}

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

	FVector ArcPreviewDirectionAtAlpha(const FRopeArcPreviewData& Preview, float Alpha)
	{
		const FVector Aim = FRopeWhipGuide::SafeNormalOr(Preview.AimDir, FVector::ForwardVector);
		FVector Up = Preview.GuideUp - FVector::DotProduct(Preview.GuideUp, Aim) * Aim;
		Up = FRopeWhipGuide::SafeNormalOr(Up, FVector::UpVector);

		const float ClampedAlpha = FMath::Clamp(Alpha, 0.0f, 1.0f);
		const float SweepRadians = FMath::DegreesToRadians(FMath::Clamp(Preview.SweepAngleDegrees, 1.0f, 180.0f));
		const float Angle = SweepRadians * (1.0f - ClampedAlpha);
		return (Aim * FMath::Cos(Angle) + Up * FMath::Sin(Angle)).GetSafeNormal();
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

float URopeComponent::GetSegmentTension(int32 SegmentIndex) const
{
	return Sim.SegmentTension.IsValidIndex(SegmentIndex) ? Sim.SegmentTension[SegmentIndex] : 0.0f;
}

float URopeComponent::GetMaxTension() const
{
	float MaxTension = 0.0f;
	for (const float T : Sim.SegmentTension)
	{
		MaxTension = FMath::Max(MaxTension, T);
	}
	return MaxTension;
}

bool URopeComponent::IsTensioned(float SlackTolerance) const
{
	float Slack = 0.0f;
	float StraightDistance = 0.0f;
	float AvailableLength = 0.0f;
	if (!ComputeTensionSlack(Slack, StraightDistance, AvailableLength))
	{
		return false;
	}

	return Slack <= FMath::Max(0.0f, SlackTolerance);
}

bool URopeComponent::BuildThrowArcPreview(const FRopeThrowContext& ThrowContext, float ReachScale, int32 SegmentCount,
	FRopeArcPreviewData& OutPreview) const
{
	OutPreview = FRopeArcPreviewData();

	const float ClampedReachScale = FMath::Max(ReachScale, 0.0f);
	if (ClampedReachScale <= KINDA_SMALL_NUMBER)
	{
		return false;
	}

	const FRopeThrowContext ResolvedThrow = ResolveThrowContext(ThrowContext);
	const FRopeWhipGuide::FSwingBasis SwingBasis = FRopeWhipGuide::ResolveSwingBasis(
		ResolvedThrow, ResolvedThrow.SwingPlane, ResolvedThrow.CustomSwingPlaneNormal);
	const FRopeWhipGuide::FConfig WhipGuideConfig = MakeWhipGuideConfig();
	const float SourceRopeLength = FMath::Max(Sim.RopeLength, RopeLength);

	OutPreview.Origin = ResolvedThrow.Origin;
	OutPreview.AimDir = SwingBasis.AimDir;
	OutPreview.GuideUp = SwingBasis.GuideUp;
	OutPreview.Radius = SourceRopeLength * ClampedReachScale;
	OutPreview.SweepAngleDegrees = WhipGuideConfig.SweepAngleDegrees;
	OutPreview.SegmentCount = FMath::Clamp(SegmentCount, 1, 128);
	return OutPreview.Radius > KINDA_SMALL_NUMBER;
}

bool URopeComponent::FindThrowArcPreviewHit(const FRopeArcPreviewData& Preview, float SampleStep, float QueryRadius,
	FRopeArcPreviewHitResult& OutHit) const
{
	OutHit = FRopeArcPreviewHitResult();
	if (Preview.Radius <= KINDA_SMALL_NUMBER || FrameColliders.Num() == 0)
	{
		return false;
	}

	// Preview는 게임플레이 확정 판정이 아니라 조준 보조라서, SDF query가 프레임을 먹지 않도록 상한을 둔다.
	constexpr int32 MaxPreviewAngleSamples = 24;
	constexpr int32 MaxPreviewRadialSamples = 16;
	constexpr int32 MaxPreviewTotalSamples = 256;

	const int32 AngleSamples = FMath::Clamp(Preview.SegmentCount, 1, MaxPreviewAngleSamples);
	const float RadialStep = FMath::Max(SampleStep, 1.0f);
	const int32 RequestedRadialSamples = FMath::Max(1, FMath::CeilToInt(Preview.Radius / RadialStep));
	const int32 TotalLimitedRadialSamples = FMath::Max(1, MaxPreviewTotalSamples / (AngleSamples + 1));
	const int32 RadialSamples = FMath::Clamp(RequestedRadialSamples, 1,
		FMath::Min(MaxPreviewRadialSamples, TotalLimitedRadialSamples));
	const float EffectiveQueryRadius = QueryRadius > KINDA_SMALL_NUMBER
		? QueryRadius
		: FMath::Max(Radius, WrapConfig.ContactRadius);

	FBox PreviewBounds(EForceInit::ForceInit);
	PreviewBounds += Preview.Origin;
	for (int32 AngleIndex = 0; AngleIndex <= AngleSamples; ++AngleIndex)
	{
		const float AngleAlpha = static_cast<float>(AngleIndex) / static_cast<float>(AngleSamples);
		const FVector Direction = ArcPreviewDirectionAtAlpha(Preview, AngleAlpha);
		if (!Direction.IsNearlyZero())
		{
			PreviewBounds += Preview.Origin + Direction * Preview.Radius;
		}
	}
	PreviewBounds = PreviewBounds.ExpandBy(EffectiveQueryRadius);

	TArray<const IRopeCollider*, TInlineAllocator<8>> CandidateColliders;
	for (const IRopeCollider* Collider : FrameColliders)
	{
		if (Collider && Collider->GetWorldBounds().ExpandBy(EffectiveQueryRadius).Intersect(PreviewBounds))
		{
			CandidateColliders.Add(Collider);
		}
	}
	if (CandidateColliders.Num() == 0)
	{
		return false;
	}

	for (int32 AngleIndex = 0; AngleIndex <= AngleSamples; ++AngleIndex)
	{
		const float AngleAlpha = static_cast<float>(AngleIndex) / static_cast<float>(AngleSamples);
		const FVector Direction = ArcPreviewDirectionAtAlpha(Preview, AngleAlpha);
		if (Direction.IsNearlyZero())
		{
			continue;
		}

		for (int32 RadialIndex = 1; RadialIndex <= RadialSamples; ++RadialIndex)
		{
			const float DistanceAlpha = static_cast<float>(RadialIndex) / static_cast<float>(RadialSamples);
			const FVector SamplePoint = Preview.Origin + Direction * (Preview.Radius * DistanceAlpha);

			for (const IRopeCollider* Collider : CandidateColliders)
			{
				const FRopeContact Contact = Collider->Query(SamplePoint, EffectiveQueryRadius);
				if (Contact.bHit)
				{
					OutHit.bHit = true;
					OutHit.HitPoint = Contact.SurfacePoint;
					OutHit.AngleAlpha = AngleAlpha;
					OutHit.DistanceAlpha = DistanceAlpha;
					return true;
				}
			}
		}
	}

	return false;
}

void URopeComponent::FinishWrapRelease(FName Bone, ERopeReleaseReason Reason, const FString& ReasonLog)
{
	// 모든 release 트리거(수동/절단/장력/거리/대상 소실)의 공용 마무리: 페이즈 전환 + 노드 반환 +
	// 일시 상태 폐기 + 쿨다운 + 이벤트. 사유별 차이는 호출자에서 끝난 상태로 들어온다.
	SetPhase(ERopePhase::Releasing, *ReasonLog);
	WrapController.Release(Reason);
	ResetTransientPhaseState();
	ReleaseCooldown = ReleaseCooldownSeconds;
	OnRopeReleased.Broadcast(Bone, Reason);
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

	FinishWrapRelease(Bone, Reason, FString::Printf(TEXT("%s, bone=%s"),
		Reason == ERopeReleaseReason::Cut ? TEXT("cut") : TEXT("manual"), *Bone.ToString()));
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

	// 되감기(reel): 솔브 전에 이번 프레임 길이 변화를 반영한다(세그먼트 rest 길이 균일 변경 —
	// 재시드 없이 CPU/GPU 동일 적용). Wrapped에서는 가용 로프 길이가 줄어 테더/장력으로 전파된다.
	UpdateReel(DeltaTime);

	// 거리 LOD 배율(iteration 감쇠) — 솔브 판정 전에 이번 프레임 값 확정(GPU 스텝/CPU 솔브 공용).
	ComputeSolverLOD();

	// 슬립은 Free 전용 — 다른 페이즈로 넘어가면 즉시 해제(전이 자체가 활동).
	if (Phase != ERopePhase::Free && bAsleep)
	{
		bAsleep = false;
		SleepTimer = 0.0f;
	}

	bSolveThisFrame = false;

	switch (Phase)
	{
	case ERopePhase::Free:        // 손에서 늘어뜨려진 채 캐릭터를 따라간다
		if (bAsleep && ShouldWakeFromSleep())
		{
			bAsleep = false;
			SleepTimer = 0.0f;
		}
		bSolveThisFrame = !bAsleep; // 슬립 중엔 솔브 스킵(GPU 로프는 dispatch 자체가 없음).
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
		// latch 이전 구간은 solver가 계속 처리한다. latch~tail은 OverrideFrame mass mask로 고정된다.
		bSolveThisFrame = (Phase == ERopePhase::Wrapping || Phase == ERopePhase::Wrapped);
		break;

	case ERopePhase::Wrapped:
	{
		// latch된 node는 skinned bone을 따라간다(GT). latch 노드는 InvMass=0이라 솔브는 자유 구간만.
		// Hold가 false면 wrap 대상 mesh가 사라진 것(예: cross-actor 대상 액터 파괴) →
		// 노드를 솔버에 되돌려 안전하게 release한다(dangling 포인터 역참조 방지는 Hold 내부에서).
		if (!WrapController.Hold(Sim, DeltaTime, OverrideFrame))
		{
			const FName Bone = WrapController.State.BoneName;
			FinishWrapRelease(Bone, ERopeReleaseReason::Broken,
				FString::Printf(TEXT("wrap target mesh lost, bone=%s"), *Bone.ToString()));
			break;
		}
		ApplyWrappedMassMask();

		// 장력 모델: 솔버가 채운 세그먼트 장력(F=λ/h², GPU 로프는 1~2프레임 지연 미러)의 최대치를
		// wrap 상태에 반영한다. 게임플레이(당김/절단 판정)와 디버거가 이 값을 읽는다.
		WrapController.State.Tension = GetMaxTension();

		// Pull 샘플 산출(항상 — 디버거/BP 관찰 + 아래 두 동작의 공용 입력).
		LastPullSample = FRopePullSample();
		WrapController.ComputePull(Sim, LastPullSample);

		// 동작 1 — 자동 견인(테더, 위치/속도 동기): 가용 로프 길이 초과분만큼 대상을 되돌린다.
		// 장력 비례 힘(폭주: 힘→스트레치→장력↑→힘↑)을 대체 — 초과분 기반이라 수렴한다.
		UpdateTether(DeltaTime);

		// 동작 2 — 능동 Pull(상수 힘): 사용자 입력(SetActivePull/Wielder)이 준 힘을 팽팽할 때만
		// 인가한다. 장력과 무관한 상수라 피드백 폭주가 없다.
		if (ActivePullForce > 0.0f && LastPullSample.bValid && LastPullSample.Tension > KINDA_SMALL_NUMBER)
		{
			ApplyPullForce(LastPullSample.Direction * ActivePullForce, LastPullSample);
		}

		// 임계 장력 release: 최대 장력이 TensionReleaseForce를 TensionReleaseTime 동안 지속해 넘으면
		// 풀린다(순간 스파이크 무시). 0 = 비활성. 흐름은 위 mesh-lost release와 동일, 사유만 Tension.
		if (WrapConfig.TensionReleaseForce > 0.0f)
		{
			TensionOverTime = (WrapController.State.Tension > WrapConfig.TensionReleaseForce)
				? TensionOverTime + DeltaTime : 0.0f;
			if (TensionOverTime >= WrapConfig.TensionReleaseTime)
			{
				const FName Bone = WrapController.State.BoneName;
				FinishWrapRelease(Bone, ERopeReleaseReason::Tension,
					FString::Printf(TEXT("tension release %.0f > %.0f, bone=%s"),
						WrapController.State.Tension, WrapConfig.TensionReleaseForce, *Bone.ToString()));
				break;
			}
		}

		// 거리 release: 손~앵커 직선 거리의 가용 로프 길이 초과분(테더 초과분과 동일 소스)이 한계를
		// 넘으면 놓친다. 기하 기반이라 지속 시간 없이 즉시 판정(장력처럼 노이즈가 없다).
		if (WrapConfig.DistanceReleaseSlack > 0.0f && LastTetherOvershoot > WrapConfig.DistanceReleaseSlack)
		{
			const FName Bone = WrapController.State.BoneName;
			FinishWrapRelease(Bone, ERopeReleaseReason::Distance,
				FString::Printf(TEXT("distance release overshoot %.0f > %.0f, bone=%s"),
					LastTetherOvershoot, WrapConfig.DistanceReleaseSlack, *Bone.ToString()));
			break;
		}
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
	// bSolveThisFrame(Free/Flight/Wrapping/Wrapped)일 때만 물리 솔브 — Wrapping/Wrapped는
	// 고정 노드가 InvMass=0이라 자유 구간만 움직이고, Contacting/Releasing은 로직 구동이라 스킵.
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

	// 거리 LOD: 원거리에서 constraint iteration만 감쇠(substep은 유지 — 안정성은 substep이 지배).
	FRopeSolverConfig LODConfig = SolverConfig;
	LODConfig.Iterations = GetLODScaledIterations();
	Solver.Step(Sim, LODConfig, /*optional*/ FrameColliders, DeltaTime);
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

	// Test log for per-tick tension checks. Re-enable while tuning if needed.
	// if (Phase == ERopePhase::Wrapping || Phase == ERopePhase::Wrapped)
	// {
	// 	constexpr float TensionLogTolerance = 5.0f;
	// 	float Slack = 0.0f;
	// 	float StraightDistance = 0.0f;
	// 	float AvailableLength = 0.0f;
	// 	const bool bHasTensionData = ComputeTensionSlack(Slack, StraightDistance, AvailableLength);
	// 	const bool bTensioned = bHasTensionData && Slack <= TensionLogTolerance;
	// 	UE_LOG(LogDynamicRope, Log,
	// 		TEXT("[%s] TensionTick phase=%s valid=%s tension=%s slack=%.2fcm straight=%.2fcm available=%.2fcm tolerance=%.2fcm"),
	// 		*GetName(),
	// 		PhaseName(Phase),
	// 		bHasTensionData ? TEXT("true") : TEXT("false"),
	// 		bTensioned ? TEXT("true") : TEXT("false"),
	// 		Slack,
	// 		StraightDistance,
	// 		AvailableLength,
	// 		TensionLogTolerance);
	// }

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
					InjectPinnedFrameVelocityForFreeReturn(Sim);
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

	// 슬립 전이 측정(Free 전용 — 프레임간 노드 변위 기반이라 이번 프레임 결과가 확정된 여기서).
	UpdateSleepState(DeltaTime);

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
	TensionOverTime = 0.0f;
	LastPullSample = FRopePullSample();
	bLoggedPullNoReceiver = false;
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
		Snapshot.WrapTension = Wrap.Tension;
		Snapshot.TensionReleaseForce = WrapConfig.TensionReleaseForce;
		Snapshot.bPullValid = LastPullSample.bValid;
		Snapshot.PullPoint = LastPullSample.WorldPoint;
		Snapshot.PullDirection = LastPullSample.Direction;
		Snapshot.PullTension = LastPullSample.Tension;
		Snapshot.TetherResponse = WrapConfig.TetherResponse;
		Snapshot.TetherOvershoot = LastTetherOvershoot;
		Snapshot.ActivePullForce = ActivePullForce;
		Snapshot.DistanceReleaseSlack = WrapConfig.DistanceReleaseSlack;
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
	if (const AActor* Owner = GetOwner())
	{
		Context.OwnerVelocity = Owner->GetVelocity();
		Context.SocketVelocity = Context.OwnerVelocity;
	}
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
	const FVector InheritedVelocity = ComputeThrowInheritedVelocity(ResolvedThrow);

	// 채찍 스윙 가이드 좌표계 구성 + 활성화(퇴화 케이스 fallback은 컴포넌트 축).
	WhipGuide.Begin(SwingBasis.AimDir, ResolvedThrow.Origin,
		ResolvedThrow.FrameForward, SwingBasis.GuideUp, SwingBasis.GuideRight,
		ResolvedThrow.ThrowSpeed, InheritedVelocity);
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
		const FVector InheritedVelocityImpulse = InheritedVelocity * ReferenceDt;
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

	// 개발용 안전장치: 표면 경로 생성이 중간에 실패했을 때, 마지막 성공 지점이 helix 기준으로
	// 한 바퀴도 감기지 않았다면 "조금 닿았는데 바로 wrapped로 철썩 붙는" 상태를 만들지 않고 release한다.
	// 값 조절 중에는 아래 bool만 false로 바꾸면 기능을 바로 꺼서 A/B 테스트할 수 있고,
	// MinFailedWrapTurns는 감각이 맞으면 나중에 FRopeWrapConfig UPROPERTY로 승격하면 된다.
	constexpr bool bEnableShortFailedWrapAbort = true;
	constexpr float MinFailedWrapTurns = 1.0f; // 테스트 후 2.0~3.0으로 올릴 수 있음
	float FailedWrapTurns = 0.0f;
	if (bEnableShortFailedWrapAbort &&
		WrappingPhase.ShouldAbortFailedShortWrap(Sim, WrappingCtx, MinFailedWrapTurns, FailedWrapTurns))
	{
		SetPhase(ERopePhase::Releasing, *FString::Printf(TEXT("wrap path failed early, turns=%.2f < %.2f"),
			FailedWrapTurns, MinFailedWrapTurns));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

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
	ApplyWrappedMassMask(/*bResetDynamicNodeVelocity*/ true);

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

	OverrideFrame.EnsureSize(Sim.Num());
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		const bool bStartPin = (i == 0 && Sim.bStartPinned);
		const bool bAnchor = AnchorNodes.Contains(i);
		const bool bFixed = bStartPin || bAnchor;
		OverrideFrame.SetInvMass(i, bFixed ? 0.0f : 1.0f);
		if (bResetDynamicNodeVelocity && !bFixed)
		{
			OverrideFrame.SetPrevFromPosition(i);
		}
	}
}

void URopeComponent::SetActivePull(float Force)
{
	ActivePullForce = FMath::Max(0.0f, Force);
}

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
	// 길이 의존 머티리얼 파라미터(꼬임 밀도) 갱신 — 감아도 꼬임 간격이 일정하게 유지된다.
	UpdateRopeMaterialDynamicParams();
}

void URopeComponent::SetReelRate(float CmPerSecond)
{
	ReelRate = CmPerSecond;
}

void URopeComponent::ComputeSolverLOD()
{
	SolverLODScale = 1.0f;
	const FRopeSolverConfig& Cfg = SolverConfig;
	if (!Cfg.bEnableDistanceLOD || Cfg.LODStartDistance <= 0.0f)
	{
		return;
	}
	// 로컬 플레이어 카메라 기준(멀티 로컬 플레이어는 0번만 — LOD는 근사여도 무방). 서버/카메라 없음 = 풀 품질.
	const APlayerCameraManager* Camera = UGameplayStatics::GetPlayerCameraManager(GetWorld(), 0);
	if (!Camera)
	{
		return;
	}
	const float Dist = static_cast<float>(FVector::Dist(Camera->GetCameraLocation(), GetComponentLocation()));
	const float Range = FMath::Max(Cfg.LODEndDistance - Cfg.LODStartDistance, 1.0f);
	const float Alpha = FMath::Clamp((Dist - Cfg.LODStartDistance) / Range, 0.0f, 1.0f);
	SolverLODScale = FMath::Lerp(1.0f, FMath::Clamp(Cfg.LODMinIterationScale, 0.05f, 1.0f), Alpha);
}

void URopeComponent::UpdateSleepState(float DeltaTime)
{
	// Free + 슬립 허용에서만 측정. 그 외에는 누적을 버려 상태 오염을 막는다(캐시는 다음 Free 진입 시 재구축).
	if (Phase != ERopePhase::Free || !SolverConfig.bAllowSleep || bAsleep || DeltaTime <= KINDA_SMALL_NUMBER)
	{
		SleepTimer = 0.0f;
		SleepPrevFramePositions.Reset();
		return;
	}

	// 프레임간 최대 노드 변위 → 속도. Verlet substep 변위가 아니라 프레임 캐시 비교라 substep 수/GPU
	// 미러 지연과 무관하게 동작한다.
	if (SleepPrevFramePositions.Num() == Sim.Num())
	{
		float MaxDistSq = 0.0f;
		for (int32 i = 0; i < Sim.Num(); ++i)
		{
			MaxDistSq = FMath::Max(MaxDistSq, static_cast<float>(FVector::DistSquared(Sim.Positions[i], SleepPrevFramePositions[i])));
		}
		const float MaxSpeed = FMath::Sqrt(MaxDistSq) / DeltaTime;
		SleepTimer = (MaxSpeed < SolverConfig.SleepVelocityThreshold) ? SleepTimer + DeltaTime : 0.0f;
		if (SleepTimer >= SolverConfig.SleepDelay)
		{
			bAsleep = true;
			SleepPinPos = Sim.StartPinTarget;
			UE_LOG(LogDynamicRope, Verbose, TEXT("[%s] rope asleep (max speed < %.1f cm/s for %.2fs)"),
				*GetName(), SolverConfig.SleepVelocityThreshold, SolverConfig.SleepDelay);
		}
	}
	SleepPrevFramePositions = Sim.Positions;
}

bool URopeComponent::ShouldWakeFromSleep() const
{
	if (!SolverConfig.bAllowSleep)
	{
		return true;
	}
	// 핀(손)이 슬립 시점에서 이동 — 캐릭터가 움직였다.
	if (FVector::DistSquared(Sim.StartPinTarget, SleepPinPos) > FMath::Square(1.0f))
	{
		return true;
	}
	// 되감기/풀기 중.
	if (!FMath::IsNearlyZero(ReelRate))
	{
		return true;
	}
	// 움직이는 collider 근접: FrameColliders는 이미 로프 bounds로 컬링돼 있어(서브시스템) 근접분만 남는다.
	// 정지 본(prev==curr)은 무시 — 애니 idle 미세 흔들림은 0.5cm 임계로 걸러진다.
	for (const IRopeCollider* Collider : FrameColliders)
	{
		if (!Collider)
		{
			continue;
		}
		FTransform PrevX, CurrX;
		if (Collider->GetFrameMotion(PrevX, CurrX) && !PrevX.Equals(CurrX, 0.5f))
		{
			return true;
		}
		FVector PrevA, PrevB;
		float InvDt = 0.0f;
		if (Collider->GetGPUCapsuleMotion(PrevA, PrevB, InvDt))
		{
			FVector A, B;
			float R = 0.0f;
			if (Collider->GetGPUCapsule(A, B, R)
				&& (FVector::DistSquared(A, PrevA) > 0.25 || FVector::DistSquared(B, PrevB) > 0.25))
			{
				return true;
			}
		}
	}
	return false;
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
	SetRopeLength(Sim.RopeLength - ReelRate * DeltaTime);
}

void URopeComponent::UpdateTether(float DeltaTime)
{
	// 가용 로프 길이(손→앵커 세그먼트 수 × 길이 + 여유) 대비 실제 직선 거리의 초과분(overshoot).
	// 스냅샷/BP 관찰을 위해 테더가 꺼져 있어도 초과분은 항상 계산한다.
	LastTetherOvershoot = 0.0f;
	if (!LastPullSample.bValid)
	{
		return;
	}
	const FVector Hand = Sim.bStartPinned ? Sim.StartPinTarget : (Sim.Positions.Num() > 0 ? Sim.Positions[0] : FVector::ZeroVector);
	const FVector Span = LastPullSample.WorldPoint - Hand;
	const float Dist = static_cast<float>(Span.Size());
	const float AvailLen = static_cast<float>(LastPullSample.AnchorNode) * Sim.SegmentLength + WrapConfig.TetherSlack;
	const float Overshoot = Dist - AvailLen;
	LastTetherOvershoot = FMath::Max(0.0f, Overshoot);
	if (WrapConfig.TetherResponse <= 0.0f || Overshoot <= 0.0f || Dist <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	// 초과분의 일부를 이번 프레임에 회수(위치/속도 동기). 남은 초과분이 다음 프레임 입력이므로 수렴한다.
	const FVector DirToHand = -Span / Dist;
	const FVector Correction = DirToHand * (Overshoot * FMath::Clamp(WrapConfig.TetherResponse, 0.0f, 1.0f));

	USkeletalMeshComponent* Mesh = const_cast<USkeletalMeshComponent*>(WrapController.State.Mesh.Get());
	if (!Mesh)
	{
		return;
	}

	// 물리 시뮬 대상(본/루트): 텔레포트 대신 질량 무관 속도 변경으로 같은 프레임 변위를 만든다.
	if (Mesh->IsSimulatingPhysics(LastPullSample.Bone))
	{
		const FVector VelChange = Correction / FMath::Max(DeltaTime, 1e-4f);
		Mesh->AddImpulse(VelChange, LastPullSample.Bone, /*bVelChange*/ true);
		return;
	}
	AActor* Owner = Mesh->GetOwner();
	if (UPrimitiveComponent* Root = Owner ? Cast<UPrimitiveComponent>(Owner->GetRootComponent()) : nullptr)
	{
		if (Root->IsSimulatingPhysics())
		{
			Root->AddImpulse(Correction / FMath::Max(DeltaTime, 1e-4f), NAME_None, /*bVelChange*/ true);
			return;
		}
	}

	// 캐릭터/비시뮬 대상: 위치 보정(스윕 — 벽 통과 방지). 캐릭터 캡슐도 이 경로로 끌려온다.
	if (Owner)
	{
		Owner->AddActorWorldOffset(Correction, /*bSweep*/ true);
	}
}

void URopeComponent::ApplyPullForce(const FVector& Force, const FRopePullSample& Pull)
{
	// wrap 대상 mesh(cross-actor 가능). 계약상 로프는 대상을 읽기만 하므로 weak가 const지만,
	// Pull은 의도된 게임플레이 개입(힘 인가)이라 여기서만 명시적으로 non-const로 푼다.
	USkeletalMeshComponent* Mesh = const_cast<USkeletalMeshComponent*>(WrapController.State.Mesh.Get());
	if (!Mesh)
	{
		return;
	}

	// 1) 감긴 본이 물리 시뮬 중(래그돌/물리 프랍)이면 그 본에 직접 — 가장 정확한 인가점.
	if (Mesh->IsSimulatingPhysics(Pull.Bone))
	{
		Mesh->AddForceAtLocation(Force, Pull.WorldPoint, Pull.Bone);
		return;
	}

	// 2) 캐릭터면 무브먼트에 힘 — 애니메이션 구동 본에는 힘을 줄 수 없으므로 이동체 전체를 견인한다
	//    (PoC 4.2: 본/루트에 단순 힘 전달까지. 팔다리 IK/래그돌 반응은 후속).
	if (ACharacter* Character = Cast<ACharacter>(Mesh->GetOwner()))
	{
		if (UCharacterMovementComponent* Movement = Character->GetCharacterMovement())
		{
			Movement->AddForce(Force);
			return;
		}
	}

	// 3) 그 외: 시뮬 중인 루트 프리미티브(물리 액터에 붙은 skeletal mesh 구성).
	AActor* Owner = Mesh->GetOwner();
	if (UPrimitiveComponent* Root = Owner ? Cast<UPrimitiveComponent>(Owner->GetRootComponent()) : nullptr)
	{
		if (Root->IsSimulatingPhysics())
		{
			Root->AddForceAtLocation(Force, Pull.WorldPoint);
			return;
		}
	}

	// 수신자 없음(애니메이션 구동 본 + 비캐릭터 + 비시뮬 루트): 힘이 조용히 사라지는 걸 wrap당 1회 알린다.
	if (!bLoggedPullNoReceiver)
	{
		bLoggedPullNoReceiver = true;
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] Pull has no force receiver: mesh=%s bone=%s is not simulating, owner=%s is not a Character and its root is not simulating — pull force is dropped."),
			*GetName(), *Mesh->GetName(), *Pull.Bone.ToString(), *GetNameSafe(Owner));
	}
}

bool URopeComponent::ComputeTensionSlack(float& OutSlack, float& OutStraightDistance, float& OutAvailableLength) const
{
	OutSlack = 0.0f;
	OutStraightDistance = 0.0f;
	OutAvailableLength = 0.0f;

	if (Phase != ERopePhase::Wrapping && Phase != ERopePhase::Wrapped)
	{
		return false;
	}

	if (!Sim.Positions.IsValidIndex(0))
	{
		return false;
	}

	int32 AnchorNodeIndex = INDEX_NONE;
	FVector AnchorWorld = FVector::ZeroVector;
	bool bHasAnchor = false;

	const auto ResolveSurfaceAnchor = [this](const FRopeSurfaceAnchor& Anchor, FVector& OutWorld, int32& OutNodeIndex) -> bool
	{
		OutNodeIndex = Anchor.NodeIndex;
		if (!Sim.Positions.IsValidIndex(OutNodeIndex))
		{
			return false;
		}

		OutWorld = Sim.Positions[OutNodeIndex];
		const USkeletalMeshComponent* Mesh = Anchor.Mesh.Get();
		if (Mesh && !Anchor.Bone.IsNone())
		{
			const FTransform BoneXform = Mesh->GetSocketTransform(Anchor.Bone);
			const FVector SurfaceWorld = BoneXform.TransformPosition(Anchor.LocalSurfacePosition);
			const FVector NormalWorld = BoneXform.TransformVectorNoScale(Anchor.LocalNormal)
				.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
			OutWorld = SurfaceWorld + NormalWorld * Anchor.SurfaceOffset;
		}

		return true;
	};

	if (Phase == ERopePhase::Wrapped)
	{
		if (WrapController.State.Anchors.Num() > 0)
		{
			bHasAnchor = ResolveSurfaceAnchor(WrapController.State.Anchors[0], AnchorWorld, AnchorNodeIndex);
		}
		else if (WrapController.State.Latched.Num() > 0)
		{
			const FRopeLatchNode& Latch = WrapController.State.Latched[0];
			AnchorNodeIndex = Latch.NodeIndex;
			if (Sim.Positions.IsValidIndex(AnchorNodeIndex))
			{
				AnchorWorld = Sim.Positions[AnchorNodeIndex];
				if (const USkeletalMeshComponent* Mesh = WrapController.State.Mesh.Get())
				{
					const FName Bone = Latch.Bone.IsNone() ? WrapController.State.BoneName : Latch.Bone;
					if (!Bone.IsNone())
					{
						AnchorWorld = Mesh->GetSocketTransform(Bone).TransformPosition(Latch.BoneLocalPos);
					}
				}
				bHasAnchor = true;
			}
		}
	}
	else
	{
		if (WrappingPhase.State.Anchors.Num() > 0)
		{
			bHasAnchor = ResolveSurfaceAnchor(WrappingPhase.State.Anchors[0], AnchorWorld, AnchorNodeIndex);
		}
		else if (WrappingPhase.State.LatchAnchor.NodeIndex != INDEX_NONE)
		{
			bHasAnchor = ResolveSurfaceAnchor(WrappingPhase.State.LatchAnchor, AnchorWorld, AnchorNodeIndex);
		}
	}

	if (!bHasAnchor || AnchorNodeIndex <= 0)
	{
		return false;
	}

	const FVector PinWorld = Sim.bStartPinned ? Sim.StartPinTarget : Sim.Positions[0];
	OutStraightDistance = FVector::Dist(PinWorld, AnchorWorld);
	OutAvailableLength = static_cast<float>(AnchorNodeIndex) * FMath::Max(Sim.SegmentLength, 0.0f);
	OutSlack = OutAvailableLength - OutStraightDistance;
	return OutAvailableLength > KINDA_SMALL_NUMBER;
}
