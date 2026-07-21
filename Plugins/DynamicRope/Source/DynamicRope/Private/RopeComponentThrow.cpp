// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeComponent.h"

#include "Collision/RopeCollider.h"
#include "Core/RopeWrapTarget.h"
#include "Debug/RopeDebugSnapshot.h"
#include "DynamicRopeLog.h"
#include "Engine/World.h"
#include "Logic/RopeThrowPreviewBuilder.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"
#include "RopeComponentInternal.h"
#include "RopeMathHelpers.h"
#include "Subsystem/RopeSimSubsystem.h"

using RopeComponentPrivate::PhaseName;
using RopeComponentPrivate::ReleaseCooldownSeconds;
using RopeComponentPrivate::ResolveAimGuideHitWorld;

namespace
{
	void ClearPreparedGuideFrameLocal(FRopePreparedThrowPreview& Prepared)
	{
		Prepared.bUseGuideFrameLocal = false;
		Prepared.GuideFrameComponent = nullptr;
		Prepared.GuideFrameLocalPoints.Reset();
		Prepared.GuideFrameLocalOrigin = FVector::ZeroVector;
	}

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
}

#pragma region Throw_Public_API

void URopeComponent::Throw()
{
	// 프레임 기저 규약은 FRopeThrowContext::MakeDefault(RopeTypes.cpp) 주석 참고. 커스텀 지점은
	// ResolveThrowContext 하나다 — ThrowWithContext가 그 관문을 태운다.
	ThrowWithContext(FRopeThrowContext::MakeDefault(*this, ThrowParams));
}

void URopeComponent::ThrowWithContext(const FRopeThrowContext& ThrowContext)
{
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] Throw requested (phase=%s, mode=%d, forward=%s)"),
		*GetName(), PhaseName(Phase), static_cast<int32>(ResolveMode),
		*ThrowContext.FrameForward.GetSafeNormal().ToCompactString());

	EnsureRopeInitialized();

	// 조합 제약 강제(런타임 쓰기 방어 — 에디터 편집은 PostEditChangeProperty가 이미 보정):
	// ①②=BareWrap만, ③=Pierce/Cinch만. 무효 조합은 던지기 시점에 보정하고 경고를 남긴다.
	const ERopeTipEngagement ClampedEngagement = RopeWrapModes::ClampEngagement(ResolveMode, TipEngagement);
	if (ClampedEngagement != TipEngagement)
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] TipEngagement %d not allowed with ResolveMode %d — clamped to %d (FullSim/Assisted=BareWrap only, Guaranteed=Pierce/Cinch only)."),
			*GetName(), static_cast<int32>(TipEngagement), static_cast<int32>(ResolveMode),
			static_cast<int32>(ClampedEngagement));
		TipEngagement = ClampedEngagement;
	}

	// Cinch는 계약상 유효한 조합이지만(③ 전용) 아직 구현이 없어 실제로는 BareWrap 감김 경로로 떨어진다.
	// 이벤트 페이로드에는 저작값 그대로 Cinch가 실리므로, 로그가 없으면 소비자는 Cinch가 성립한 줄 안다.
	// 던지기당이 아니라 로프당 1회만 남긴다(연사 시 로그 홍수 방지).
	if (TipEngagement == ERopeTipEngagement::Cinch && !bWarnedCinchUnimplemented)
	{
		bWarnedCinchUnimplemented = true;
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] TipEngagement=Cinch는 아직 미구현이다 — 실제 동작은 BareWrap 감김이고 이벤트에만 Cinch로 보고된다."),
			*GetName());
	}

	// ③ GuaranteedWrap의 BP 직행/AI 경로: Wielder의 조준 흐름 없이 Throw가 불려도 보장 계약을 지킨다 —
	// 컴포넌트가 스스로 prepared preview를 빌드해 구속 경로로 던진다. 빌드 성공 = 조준한 대상에 무조건 꽂힘,
	// 빌드 실패(대상 없음/사거리 밖) = 거부가 아니라 레이 끝점 아치 투척으로 폴백(2026-07-14 보장 재정의).
	// 빌드 파라미터(Arc Search)는 로프 멤버가 단일 소스라 Wielder 경로와 항상 일치한다.
	if (ResolveMode == ERopeWrapResolveMode::GuaranteedWrap)
	{
		// ③는 Reel(장전) 상태에서만 throw가 성립한다. 꽂힌 뒤 release로 Free가 된 상태에서는 EnterReel() 후에야 던진다.
		if (!CanThrowNow())
		{
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] Guaranteed throw rejected: not in Reel (phase=%s). Call EnterReel() first."),
				*GetName(), PhaseName(Phase));
			return;
		}

		FRopePreparedThrowPreview Prepared;
		FString FailureReason;
		const FRopeThrowContext ResolvedThrow = ResolveThrowContext(ThrowContext);
		if (BuildPreparedWrappingPreviewFromResolvedContext(ResolvedThrow, Prepared, &FailureReason))
		{
			// 대상 조준 성공 → 무조건 꽂힘(GuidedThrow, 내부에서 OnDeployFromReel).
			if (!ThrowWithPreparedPreview(Prepared))
			{
				UE_LOG(LogDynamicRope, Log, TEXT("[%s] Guaranteed prepared throw failed after build: %s"),
					*GetName(), FailureReason.IsEmpty() ? TEXT("prepared throw failed") : *FailureReason);
			}
			return;
		}

		// preview 실패(대상 없음/사거리 밖) → 거부가 아니라 레이 끝점을 향한 아치 던지기로 폴백(꽂힘 없이 Free 낙하).
		// 조준 던지기와 아치를 통일한다(2026-07-14 보장 재정의: 보장은 '조준한 대상'에 대한 것).
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] Guaranteed throw: no aim target (%s) — arc toss toward ray-end."),
			*GetName(), FailureReason.IsEmpty() ? TEXT("no preview") : *FailureReason);
		const float FreeLen = FMath::Max(Sim.RopeLength, RopeLength);
		const FVector FreeEndpoint = ResolvedThrow.Origin + ResolvedThrow.FrameForward.GetSafeNormal() * FreeLen;
		OnDeployFromReel();
		StartFreeGuidedThrow(ResolvedThrow, FreeEndpoint);
		return;
	}

	StartFreshThrow(ThrowContext);
}

bool URopeComponent::ThrowWithPreparedPreview(const FRopePreparedThrowPreview& Prepared)
{
	// ③의 핵심 진입점: 여기서는 StartFreshThrow처럼 Flight로 보내지 않는다.
	// preview build가 고른 path/contact/anchor를 authoritative하게 사용해야 실제 결과가 preview와 갈라지지 않는다.
	EnsureRopeInitialized();
	if (!Prepared.IsValid() || Prepared.RenderPreview.Points.Num() < 2 || Sim.Num() < 2)
	{
		return false;
	}

	// ③ Guaranteed는 Reel(장전) 상태에서만 throw가 성립한다(Wielder 직행 방어 — ThrowWithContext와 동일 게이트).
	if (!CanThrowNow())
	{
		return false;
	}

	// 조합 제약 강제(Wielder 직행 진입점도 동일 방어 — ThrowWithContext의 보정과 같은 규칙).
	TipEngagement = RopeWrapModes::ClampEngagement(ResolveMode, TipEngagement);

	// 서브클래스 wrap 대상 게이트: preview 빌드는 이 게이트를 모르므로(정적 빌더) 진입점에서 거른다.
	if (!CanWrapTarget(Prepared.Mesh.Get(), Prepared.Bone))
	{
		return false;
	}

	ResetStateForNewThrow();
	// 팁 부착물 확보 보험 — 정상 경로는 BeginPlay가 이미 잡았다(런타임 bUseTipMesh 토글 대비; 이미 있으면 no-op).
	EnsureTipMesh();

	FRopePreparedThrowPreview ResolvedPrepared = Prepared;
	// 입력 때 저장한 owner-local path를 throw 실행 시점의 owner transform으로 다시 해석한다.
	ResolvedPrepared.RenderPreview = Prepared.ResolveRenderPreviewWorld();
	ResolvedPrepared.ThrowContext.Origin = Prepared.ResolveGuideOriginWorld();
	ClearPreparedGuideFrameLocal(ResolvedPrepared);
	ApplyPierceSocketTargetsToPrepared(ResolvedPrepared);
	AimTargeting.SetWrapTargetLock(ResolvedPrepared.ThrowContext);

	const FString PhaseReason = FString::Printf(TEXT("prepared points=%d, bone=%s"),
		ResolvedPrepared.RenderPreview.Points.Num(), *ResolvedPrepared.Bone.ToString());
	if (!BeginGuidedThrowState(MoveTemp(ResolvedPrepared), /*bFreeThrow*/ false))
	{
		return false;
	}

	// Reel에서 나가는 순간 전개 — 로프 표시 복원 + 전체 길이 복원(기본 구현, override 가능).
	OnDeployFromReel();

	SetPhase(ERopePhase::GuidedThrow, *PhaseReason);
	return true;
}

FRopeAimTargeting::FQueryContext URopeComponent::MakeAimQueryContext() const
{
	// 폴백 치수 규약: ray 길이 = 현재/초기 로프 길이 중 큰 값, 질의 반경 = 튜브/접촉 반경 중 큰 값.
	FRopeAimTargeting::FQueryContext Ctx;
	Ctx.Colliders = &SimFrame.FrameColliders;
	Ctx.FallbackRayLength = FMath::Max(Sim.RopeLength, RopeLength);
	Ctx.FallbackQueryRadius = FMath::Max(Radius, GetEffectiveContactQueryRadius());
	return Ctx;
}

float URopeComponent::GetAimRayEffectiveQueryRadius(float RequestedRadius) const
{
	return FRopeAimTargeting::ResolveEffectiveQueryRadius(MakeAimQueryContext(), RequestedRadius);
}

void URopeComponent::SetAimRayColliderQueryBounds(const FVector& Origin, const FVector& AimDir,
	float RayLength, float QueryRadius)
{
	// 무효 입력이면 FBox(ForceInit) 반환 = clear와 동일(수집 확장 없음).
	SimFrame.AimRayColliderQueryBounds =
		FRopeAimTargeting::MakeAimRayQueryBounds(MakeAimQueryContext(), Origin, AimDir, RayLength, QueryRadius);
}

void URopeComponent::ClearAimRayColliderQueryBounds()
{
	SimFrame.AimRayColliderQueryBounds = FBox(ForceInit);
}

bool URopeComponent::RefreshAimRayQueryColliders(const FRopeAimRayThrowRequest& Request)
{
	if (!Request.IsValid())
	{
		ClearAimRayColliderQueryBounds();
		return false;
	}

	SetAimRayColliderQueryBounds(
		Request.RayOrigin, Request.RayDirection, Request.RayLength, Request.QueryRadius);
	if (URopeSimSubsystem* RopeSim = URopeSimSubsystem::Get(GetWorld()))
	{
		return RopeSim->RefreshFrameCollidersForImmediateQuery(*this);
	}
	return false;
}

bool URopeComponent::ResolveAimRayThrowContext(const FRopeAimRayThrowRequest& Request,
	FRopeThrowContext& OutContext, FRopeAimRayHitResult* OutHit, FRopeAimRayHitResult* OutBlockedHit) const
{
	return FRopeAimTargeting::ResolveAimRayThrowContext(MakeAimQueryContext(), Request,
		[this](const USceneComponent* Mesh, FName Bone) { return CanWrapTarget(Mesh, Bone); },
		OutContext, OutHit, OutBlockedHit);
}

void URopeComponent::QueueAimRayThrow(const FRopeAimRayThrowRequest& Request)
{
	// 무효 요청은 즉시 fallback throw — StartFreshThrow 전이라 컴포넌트가 직접 처리한다(F-클래스 밖).
	if (!Request.IsValid())
	{
		StartFreshThrow(Request.BaseContext);
		Request.OnResolved.ExecuteIfBound();
		return;
	}

	AimTargeting.QueuePendingThrow(Request);
	SetAimRayColliderQueryBounds(
		Request.RayOrigin, Request.RayDirection, Request.RayLength, Request.QueryRadius);
}

bool URopeComponent::BuildPreparedWrappingPreview(const FRopeThrowContext& ThrowContext,
	FRopePreparedThrowPreview& OutPrepared, FString* OutFailureReason) const
{
	// Prepared preview는 아직 던지기 전인 Free/Releasing/Reel에서만 의미가 있다(Reel=GuaranteedWrap 장전
	// 준비 상태 — 조준 preview 표시 + Reel에서의 던지기 진입이 이 빌드를 쓴다). Flight 이후 phase는 이미
	// 실제 접촉/감김 상태라 preview가 없다(FullSimulation/AssistedJudged는 preview 자체가 없고,
	// GuaranteedWrap은 이 prepared 경로가 유일한 preview다).
	if (Phase != ERopePhase::Free && Phase != ERopePhase::Releasing && Phase != ERopePhase::Reel)
	{
		OutPrepared.Reset();
		RopeMath::SetPreviewFailureReason(OutFailureReason,
			FString::Printf(TEXT("prepared preview rejected: phase=%s"), PhaseName(Phase)));
		return false;
	}
	return BuildPreparedWrappingPreviewFromResolvedContext(
		ResolveThrowContext(ThrowContext), OutPrepared, OutFailureReason);
}

bool URopeComponent::BuildPreparedWrappingPreviewFromResolvedContext(
	const FRopeThrowContext& ResolvedThrowContext,
	FRopePreparedThrowPreview& OutPrepared, FString* OutFailureReason) const
{
	OutPrepared.Reset();

	FRopeThrowPreviewBuilder::FInput Input;
	Input.Sim = &Sim;
	Input.Colliders = &SimFrame.FrameColliders;
	// wrap 대상 게이트 주입(aim 경로의 ResolveAimRayThrowContext와 같은 패턴) — arc 탐색이 aim과 같은
	// 기준으로 후보를 거르게 한다. 주입 전에는 금지 대상이 preview에만 보이고 throw 진입점에서 거부됐다.
	Input.CanWrapTarget = [this](const USceneComponent* Mesh, FName Bone) { return CanWrapTarget(Mesh, Bone); };
	Input.ThrowContext = ResolvedThrowContext;
	Input.WrapConfig = WrapConfig;
	Input.WrapConfig.ContactQueryRadius = GetEffectiveContactQueryRadius(); // 0=auto 해석 승계
	// 결착 모델을 preview 빌더로 전파 — Pierce면 감김 나선 대신 단일 앵커 꽂힘 경로를 탄다.
	Input.TipEngagement = TipEngagement;
	Input.ResolveMode = ResolveMode;
	Input.RopeRadius = Radius;
	Input.RopeNumSides = NumSides;
	Input.RopeLength = FMath::Max(Sim.RopeLength, RopeLength);
	Input.SweepAngleDegrees = MakeWhipGuideConfig().SweepAngleDegrees;
	Input.FallbackForward = GetForwardVector();
	Input.OwnerName = GetName();
	// 아크 탐색 튜닝은 로프 멤버가 단일 소스 — Wielder 경로와 BP 직행 Throw() 경로가 항상 같은 값을 본다.
	Input.ReachScale = PreviewReachScale;
	Input.SegmentCount = PreviewSegmentCount;
	Input.SampleStep = PreviewSampleStep;
	Input.QueryRadius = PreviewQueryRadius;
	const bool bBuilt = FRopeThrowPreviewBuilder::BuildFreePreparedPreview(Input, OutPrepared, OutFailureReason);
	if (bBuilt)
	{
		ApplyPierceSocketTargetsToPrepared(OutPrepared);
	}
	return bBuilt;
}

void URopeComponent::DispatchCaptured(FName Bone)
{
	// 통지 순서는 엔진 Notify 관례 — 네이티브 훅 먼저, 그다음 BP 델리게이트.
	NotifyCaptured(Bone);
	OnRopeCaptured.Broadcast(Bone);
}

#pragma endregion Throw_Public_API

#pragma region Throw_Phase_State

void URopeComponent::ResolvePendingAimThrow()
{
	// StartFreshThrow가 transient state를 초기화하므로 요청을 먼저 값으로 꺼내고 pending 상태를 비운다.
	FRopeAimRayThrowRequest Request;
	if (!AimTargeting.TakePendingThrow(Request))
	{
		return;
	}

	FRopeThrowContext ResolvedContext;
	ResolveAimRayThrowContext(Request, ResolvedContext);
	StartFreshThrow(ResolvedContext);
	Request.OnResolved.ExecuteIfBound();
}

void URopeComponent::FilterFrameCollidersForAimWrapTarget()
{
	AimTargeting.FilterCollidersToTarget(Phase, ResolveMode, SimFrame.FrameColliders);
}

#pragma endregion Throw_Phase_State

#pragma region Throw

// ===== Throw ================================================================

FRopeThrowContext URopeComponent::ResolveThrowContext(const FRopeThrowContext& ThrowContext) const
{
	FRopeThrowContext Resolved = ThrowContext;

	// 최종 프레임은 정규직교(오른손계)를 보증한다 — 생산자(MakeDefault/Wielder/BP 직접 호출)가 무엇을
	// 넣었든 하류(WhipGuide 스윙 기저, 프리뷰 빌더)는 이 결과만 믿는다. 이전 구현은 세 축을 각각
	// 정규화만 해서 비직교 Custom 축이나 폴백으로 교체된 축이 서로 안 맞는 프레임으로 통과했다.
	// 규약: Forward가 기준축(방향 보존, 정규화만). Up은 Forward에 직교화(Gram-Schmidt) — 입력 Up이
	// Forward와 평행하면 월드 Up → 컴포넌트 Up 순으로 폴백, 전부 평행하면 임의 수직축.
	// Right는 항상 Up×Forward로 재유도하고 입력 Right는 무시한다 — 반대편 스윙 의도는 뒤집힌 Right가
	// 아니라 SwingPlane의 AimAndFrameLeft/Right로 표현하는 것이 지원 계약이다.
	Resolved.FrameForward = FRopeWhipGuide::SafeNormalOr(Resolved.FrameForward, GetForwardVector());
	const FVector Forward = Resolved.FrameForward;

	FVector Up = FVector::ZeroVector;
	const FVector UpCandidates[] = { ThrowContext.FrameUp, FVector::UpVector, GetUpVector() };
	for (const FVector& Candidate : UpCandidates)
	{
		FVector Projected = Candidate - FVector::DotProduct(Candidate, Forward) * Forward;
		if (Projected.Normalize(KINDA_SMALL_NUMBER))
		{
			Up = Projected;
			break;
		}
	}
	if (Up.IsNearlyZero())
	{
		// 수직 던지기 + 수직 컴포넌트 축 — 임의 수직축 폴백.
		Up = RopeMath::AnyTangentFromNormal(Forward);
	}
	Resolved.FrameUp = Up;
	Resolved.FrameRight = FVector::CrossProduct(Up, Forward);

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

	// 던지기 시작 = 4단계 고정 순서: ① 이전 상태 정리 → ② 체인 리셋(+GPU 재시드) → ③ 채찍 스윙 시작
	// → ④ Verlet 속도 주입. ④는 ③이 확정한 조준 방향(WhipGuide.GetAimDir)을 쓰므로 순서가 계약이다.
	ResetStateForNewThrow();
	// ray가 확정한 mesh+bone을 primary로 저장한다. Assisted는 같은 mesh의 다른 본도 후보/경로에
	// 허용하고, Guaranteed만 Flight/Contacting/Wrapping 전체를 exact bone으로 제한한다.
	AimTargeting.SetWrapTargetLock(ResolvedThrow);
	ResetChainForThrow(ResolvedThrow.Origin);
	BeginWhipSwingFromThrow(ResolvedThrow);
	InjectThrowVelocityIntoVerlet(ResolvedThrow);
	// 팁 부착물 확보 보험 — 정상 경로는 BeginPlay가 이미 잡았다(이미 있으면 no-op).
	EnsureTipMesh();

	SetPhase(ERopePhase::Flight, *FString::Printf(TEXT("fresh throw impulse, aim=%s, speed=%.1f"),
		*WhipGuide.GetAimDir().ToCompactString(), ResolvedThrow.ThrowSpeed));
}

void URopeComponent::ResetStateForNewThrow()
{
	// 새 throw: 잡고 있던 wrap은 수동 해제, 진행 중 페이즈 일시 상태는 폐기, 쿨다운 없이 즉시 던진다.
	// 커밋된 wrap이었으면 해제를 알려야 한다(FinishWrapRelease와 같은 짝 맞춤) — 안 그러면
	// OnAnyRopeReleased가 안 나가 cross-actor 대상(랙돌 등)이 로프가 풀렸는데도 영구 고착된다.
	// mesh/본은 Release가 상태를 비우기 전에 잡고, 통지는 상태 정리 후에 쏜다(DispatchReleased 재진입 계약).
	const USceneComponent* WrappedMesh = WrapController.State.Mesh.Get();
	const FName WrappedBone = WrapController.State.BoneName;
	const bool bWasWrapped = WrapController.IsActive();
	if (bWasWrapped)
	{
		WrapController.Release(ERopeReleaseReason::Manual);
	}
	ResetKinematicVirtualBridges();
	ResetTransientPhaseState();
	ReleaseCooldown = 0.0f;
	if (bWasWrapped)
	{
		DispatchReleased(WrappedMesh, WrappedBone, ERopeReleaseReason::Manual, /*bWasWrapped*/ true);
	}
}

bool URopeComponent::BeginGuidedThrowState(FRopePreparedThrowPreview&& Prepared, bool bFreeThrow)
{
	if (Sim.Num() < 2)
	{
		return false;
	}

	const FVector Origin = Prepared.ThrowContext.Origin;
	GuidedThrowState.bActive = true;
	GuidedThrowState.bFreeThrow = bFreeThrow;
	GuidedThrowState.Prepared = MoveTemp(Prepared);
	GuidedThrowState.StartPositions = Sim.Positions;
	GuidedThrowState.Elapsed = 0.0f;
	GuidedThrowState.Duration = FMath::Max(0.01f, WrapConfig.WrappingMotionDuration);

	Sim.bStartPinned = true;
	Sim.StartPinPrev = Origin;
	Sim.StartPinTarget = Origin;
	if (Sim.Positions.IsValidIndex(0))
	{
		Sim.Positions[0] = Origin;
		Sim.PrevPositions[0] = Origin;
	}
	return true;
}

void URopeComponent::ResetChainForThrow(const FVector& HandOrigin)
{
	// 체인 위치를 통째로 재설정하는 곳이므로 GPU 상주 버퍼 재시드 세대(M5)도 여기서 함께 올린다 —
	// 리셋과 재시드는 한 몸이다(따로 두면 한쪽만 하는 버그가 생긴다).
	++SimFrame.SimGeneration;
	bWrappedMassMaskDirty = true;

	if (Sim.Num() < 2)
	{
		return;
	}

	// 손(노드 0)을 던지기 원점에 핀. 전 노드 Prev=Pos(속도 0) — 던지기 속도는 ④가 따로 싣는다.
	Sim.bStartPinned = true;
	Sim.StartPinPrev = HandOrigin;
	Sim.StartPinTarget = HandOrigin;
	Sim.Positions[0] = HandOrigin;
	Sim.PrevPositions[0] = HandOrigin;
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		Sim.InvMass[i] = (i == 0) ? 0.0f : 1.0f;
		Sim.SetStill(i);
	}
}

void URopeComponent::BeginWhipSwingFromThrow(const FRopeThrowContext& ResolvedThrow)
{
	// WhipGuide.Begin의 입력(조준/가이드 축, 상속 속도)은 전부 ResolvedThrow에서 파생된다 —
	// 파생 값 조립을 여기 가둬서 호출부(StartFreshThrow)에는 단계 이름만 남긴다.
	const FRopeWhipGuide::FSwingBasis SwingBasis = FRopeWhipGuide::ResolveSwingBasis(
		ResolvedThrow, ResolvedThrow.SwingPlane, ResolvedThrow.CustomSwingPlaneNormal);
	const FVector InheritedVelocity = ComputeThrowInheritedVelocity(ResolvedThrow);
	FlightGuidePlaneNormal = SwingBasis.GuideRight.GetSafeNormal();
	bHasFlightGuidePlaneNormal = !FlightGuidePlaneNormal.IsNearlyZero();

	// 채찍 스윙 가이드 좌표계 구성 + 활성화(퇴화 케이스 fallback은 컴포넌트 축).
	WhipGuide.Begin(SwingBasis.AimDir, ResolvedThrow.Origin,
		ResolvedThrow.FrameForward, SwingBasis.GuideUp, SwingBasis.GuideRight,
		ResolvedThrow.ThrowSpeed, InheritedVelocity,
		ResolvedThrow.bHasAimGuideHit, ResolvedThrow.AimGuideHitWorldPos,
		ResolvedThrow.AimGuideSteerStartAlpha, ResolvedThrow.AimGuideLockAlpha);

	if (Sim.Num() >= 2)
	{
		// 가이드 구간 노드를 T=0 가이드 곡선 위에 스냅(속도 0).
		WhipGuide.SnapToInitialPose(Sim, MakeWhipGuideConfig());
	}
}

void URopeComponent::InjectThrowVelocityIntoVerlet(const FRopeThrowContext& ResolvedThrow)
{
	const int32 LastNode = Sim.Num() - 1;
	if (LastNode < 1)
	{
		return;
	}

	// Verlet 적분에서 속도는 (Pos - Prev)/dt 로 암묵 표현된다. Prev를 원하는 속도의 반대 방향으로
	// v·dt만큼 밀면 위치는 그대로인 채 다음 스텝부터 그 속도가 실린다(순수 속도 주입).
	// 분배: 손→끝으로 갈수록 가중(SmoothStep + tail 가중)하고 TipVelocityBoost로 끝을 부스트해 채찍처럼
	// 끝이 앞서 나가게 한다. 상속 속도(owner/socket)는 전 노드 균일. ReferenceDt는 첫 스텝 실제 dt와
	// 무관한 고정 환산 기준(프레임레이트에 따라 던지기 세기가 변하지 않게).
	const FVector ThrowDir = WhipGuide.GetAimDir();
	const float ReferenceDt = 1.0f / 60.0f;
	const float BaseImpulse = ResolvedThrow.ThrowSpeed * ReferenceDt;
	const float TipBoost = FMath::Clamp(ThrowParams.TipVelocityBoost, 0.25f, 3.0f);
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

void URopeComponent::AbortGuidedThrow(ERopeReleaseReason Reason, const TCHAR* ReasonLog)
{
	// 허공 던지기는 대상이 없어 engagement를 연 적이 없다 → release를 쏘면 짝이 안 맞는 유령 신호가 된다
	// (DispatchReleased 계약 참고). 조준 던지기만 알린다.
	const bool bNotify = !GuidedThrowState.bFreeThrow;
	// ResetTransientPhaseState가 GuidedThrowState를 비우므로 먼저 복사한다.
	const FName Bone = GuidedThrowState.Prepared.Bone;

	SetPhase(ERopePhase::Releasing, ReasonLog);
	ResetTransientPhaseState();
	ReleaseCooldown = ReleaseCooldownSeconds;

	// 상태를 모두 정리한 뒤에 알린다 — 핸들러가 ReleaseWrap 등을 다시 부를 수 있다(OnRopeReleased 계약).
	// 커밋 전이므로 중앙 신호는 나가지 않는다(bWasWrapped=false).
	if (bNotify)
	{
		DispatchReleased(nullptr, Bone, Reason, /*bWasWrapped*/ false);
	}
}

void URopeComponent::UpdateGuidedThrow(float DeltaTime)
{
	// 허공(free) 던지기는 대상 mesh/bone 없이 RenderPreview 직선만 따라가므로 IsValid(대상 요구) 대신 경로만 확인한다.
	const bool bFree = GuidedThrowState.bFreeThrow;
	const bool bValidPath = bFree ? GuidedThrowState.Prepared.RenderPreview.IsValid()
		: GuidedThrowState.Prepared.IsValid();
	if (!GuidedThrowState.bActive || !bValidPath || Sim.Num() < 2)
	{
		// 대상 mesh 소실/경로 무효 — 비자발적 실패라 Broken.
		AbortGuidedThrow(ERopeReleaseReason::Broken, TEXT("guided throw invalid"));
		return;
	}

	// ③ 연출 중 인터럽트 훅(기본 false = "그래도 보장"): 대상 사망/텔레포트 등 게임 규칙이 보장을
	// 깨야 할 때만 서브클래스가 true를 반환한다(2026-07-13 회의 결정 G — 깡통 오버라이드).
	// 허공 던지기는 폴링하지 않는다 — 깰 보장이 없고, Prepared가 stub(Mesh=null)이라 훅이 문서화한
	// 용례(대상 사망/텔레포트)를 판단할 수 없다. 취소가 필요하면 ReleaseWrap()이 있다.
	if (!bFree && ShouldAbortGuaranteedThrow(GuidedThrowState.Prepared))
	{
		// 게임 규칙이 의도적으로 깬 것 — 내부 실패(Broken)와 구분해 소비자가 다르게 반응할 수 있게 한다.
		AbortGuidedThrow(ERopeReleaseReason::ThrowAborted, TEXT("guided throw aborted by game rule"));
		return;
	}

	GuidedThrowState.Elapsed += DeltaTime;
	const float Alpha = FMath::Clamp(GuidedThrowState.Elapsed / FMath::Max(GuidedThrowState.Duration, 0.01f), 0.0f, 1.0f);
	const float EasedAlpha = Alpha * Alpha * (3.0f - 2.0f * Alpha);
	const FRopePreparedThrowPreview& Prepared = GuidedThrowState.Prepared;

	// 상향 포물선 아치(조준·허공 공통): 팁이 손→목표 직선 위로 부풀었다 착지한다. Alpha=0.5 정점, 0·1에서 0.
	// NodeFrac 선형이라 매 순간 로프는 일직선이고 팁 궤적만 포물선. Alpha=1에서 오프셋 0이라 착지 지점은 정확히 유지.
	const FVector ArcOriginW = Prepared.ResolveGuideOriginWorld();
	const FVector ArcTipW = Prepared.ResolveGuidePointWorld(Sim.Num() - 1);
	const float ArcHeight = ThrowParams.GuidedThrowArcHeightRatio * static_cast<float>((ArcTipW - ArcOriginW).Size());
	const float ArcT = 4.0f * Alpha * (1.0f - Alpha);
	const int32 LastNode = Sim.Num() - 1;

	// 조준 던지기(비-허공)면 팁 목표를 매 프레임 현재 대상 본 위치로 재조준한다 — 비행 중 대상이
	// 움직여도 팁 궤적이 조준한 신체 지점으로 수렴하고 착지 순간 튐(pop)이 없다. 프리뷰 가이드 점은
	// thrower-local이라 대상 이동을 반영하지 못하므로 끝점만 실시간 대상으로 대체한다.
	const bool bTrackAimTarget = !bFree && Prepared.ThrowContext.bHasAimGuideLocalHit;
	const FVector AimTargetWorld = bTrackAimTarget
		? ResolveAimGuideHitWorld(Prepared.ThrowContext)
		: FVector::ZeroVector;

	SimFrame.OverrideFrame.EnsureSize(Sim.Num());
	for (int32 NodeIndex = 0; NodeIndex < Sim.Num(); ++NodeIndex)
	{
		// 손 앵커(node 0)는 보간하지 않고 항상 "현재" 손 위치에 붙어 있어야 한다. 가이드 원점(ResolveGuideOriginWorld)은
		// 던진 순간의 좌표라(허공 던지기는 guide-frame-local이 없어 ThrowContext.Origin에 완전 고정) 그걸 목표로
		// 삼으면 던지는 동안 캐릭터가 움직일 때 0번 노드가 손에서 떨어진다. GuidedThrow는 솔브를 끄므로 솔버의
		// 손 핀도 걸리지 않는다 → 여기서 직접 현재 손에 고정한다.
		// StartPinTarget은 PrepareSimFrame이 이번 프레임 GetComponentLocation()으로 이미 갱신했다.
		if (NodeIndex == 0 && Sim.bStartPinned)
		{
			SimFrame.OverrideFrame.SetPosition(0, Sim.StartPinTarget, /*bZeroVelocity*/ true);
			SimFrame.OverrideFrame.SetInvMass(0, 0.0f);
			continue;
		}

		// 나머지 노드는 시작 위치 → preview 결과 위치로 보간하고, 시간 기반 상향 아치 오프셋을 더한다.
		// 조준 던지기의 팁 노드만 실시간 대상 위치로 재조준한다(나머지는 프리뷰 가이드 점 유지).
		const FVector Target = (bTrackAimTarget && NodeIndex == LastNode)
			? AimTargetWorld
			: Prepared.ResolveGuidePointWorld(NodeIndex);
		const FVector Start = GuidedThrowState.StartPositions.IsValidIndex(NodeIndex)
			? GuidedThrowState.StartPositions[NodeIndex]
			: Sim.Positions[NodeIndex];
		FVector Position = FMath::Lerp(Start, Target, EasedAlpha);

		const float NodeFrac = (LastNode > 0) ? static_cast<float>(NodeIndex) / static_cast<float>(LastNode) : 0.0f;
		Position += FVector::UpVector * (ArcHeight * ArcT * NodeFrac);

		SimFrame.OverrideFrame.SetPosition(NodeIndex, Position, /*bZeroVelocity*/ true);
		SimFrame.OverrideFrame.SetInvMass(NodeIndex, 0.0f);
	}

	if (Alpha >= 1.0f)
	{
		if (bFree)
		{
			// 허공 던지기: 꽂힘 없이 완료 → 비-핀 노드를 물리로 되돌리고 Free로 낙하시킨다.
			for (int32 NodeIndex = 0; NodeIndex < Sim.Num(); ++NodeIndex)
			{
				SimFrame.OverrideFrame.SetInvMass(NodeIndex, (NodeIndex == 0 && Sim.bStartPinned) ? 0.0f : 1.0f);
				SimFrame.OverrideFrame.SetPrevFromPosition(NodeIndex);
			}
			SetPhase(ERopePhase::Free, TEXT("free guided throw landed"));
			ResetTransientPhaseState();
		}
		else
		{
			FinishGuidedThrow();
		}
	}
}

void URopeComponent::FinishGuidedThrow()
{
	// 여기는 조준 던지기 전용이다 — 허공 던지기는 UpdateGuidedThrow가 Free로 착지시키고 오지 않는다.
	const FRopePreparedThrowPreview Prepared = GuidedThrowState.Prepared;
	if (!Prepared.IsValid() || Prepared.Anchors.Num() == 0 || !Prepared.Mesh.IsValid())
	{
		AbortGuidedThrow(ERopeReleaseReason::Broken, TEXT("guided throw commit failed"));
		return;
	}

	// preview builder가 만든 anchor들을 그대로 wrapped seed로 승격한다.
	// 그래서 완료 시점에 contact를 다시 찾지 않고, preview와 동일한 bone/local anchor에 고정된다.
	FRopeWrapState Seed;
	Seed.BoneName = Prepared.Bone;
	Seed.Mesh = Prepared.Mesh;
	Seed.Anchors = Prepared.Anchors;
	for (const FRopeSurfaceAnchor& Anchor : Seed.Anchors)
	{
		FRopeLatchNode Latch;
		Latch.NodeIndex = Anchor.NodeIndex;
		Latch.Bone = Anchor.Bone;
		Seed.Latched.Add(Latch);
	}

	// Pierce: 팁 소켓이 조준 히트점에 박히도록 메쉬 자세를 얼려 앵커에 싣는다(회전 freeze + 꼬리 연결).
	// LocalMeshTransform = 팁 렌더 자세(bone-local), LocalSurfacePosition = 로프 연결점(꼬리, bone-local).
	// 소켓 미설정이면 앵커를 그대로 둬 현행(원점=히트점, 세그먼트 추종) 폴백.
	if (TipEngagement == ERopeTipEngagement::Pierce && Seed.Anchors.Num() > 0)
	{
		FRopeSurfaceAnchor& Anchor = Seed.Anchors[0];
		const USceneComponent* Mesh = Seed.Mesh.Get();
		FVector HitPoint = Anchor.StartWorldPosition; // 빌더가 꽂힘 지점으로 세팅.
		ResolvePreparedPierceHitPoint(Prepared, HitPoint);
		FVector PierceDir = (HitPoint - Sim.Positions[0]).GetSafeNormal();
		if (PierceDir.IsNearlyZero())
		{
			PierceDir = Prepared.ThrowContext.FrameForward.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
		}

		FTransform ComponentWorld;
		FVector TailWorld;
		if (Mesh && !Anchor.Bone.IsNone() && ComputePierceEmbed(HitPoint, PierceDir, ComponentWorld, TailWorld))
		{
			const FTransform BoneXform = ResolveBindingWorld(Mesh, Anchor.Bone);
			Anchor.LocalMeshTransform = ComponentWorld.GetRelativeTransform(BoneXform);
			Anchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(TailWorld);
		}
	}

	ResetKinematicVirtualBridges();
	WrapController.BeginWrap(Sim, Seed, SimFrame.OverrideFrame);
	if (!WrapController.State.IsWrapped())
	{
		AbortGuidedThrow(ERopeReleaseReason::Broken, TEXT("guided throw begin wrap failed"));
		return;
	}

	ApplyWrappedMassMask(/*bResetDynamicNodeVelocity*/ true);

	// ③은 Flight/Contacting을 거치지 않아 Captured가 한 번도 발화하지 않았다 — 그런데 release는 발화하므로
	// 소비자 입장에선 "Captured 없는 Released"라는 짝 안 맞는 이벤트 쌍이 됐다. 도달 = 잡힘이므로 여기서
	// 발화해 ①②와 같은 (Captured → Wrapped) 순서를 만든다. BeginWrap 성공 뒤에 두어, Captured만 나가고
	// Wrapped가 안 나오는 중간 실패 구간이 생기지 않게 한다.
	DispatchCaptured(Seed.BoneName);

	SetPhase(ERopePhase::Wrapped, *FString::Printf(TEXT("guided throw bone=%s, %d anchor(s)"),
		*Seed.BoneName.ToString(), Seed.Anchors.Num()));
	ResetTransientPhaseState();
	// ③ preview 기반 성립은 판정을 거치지 않으므로 판정값은 -1(미측정) 계약이다.
	const FRopeWrappedEventInfo WrappedInfo = MakeWrappedEventInfo(Seed, /*AngleDeg*/ -1.0f, /*CoverageDeg*/ -1.0f);
	DispatchWrapped(WrappedInfo);
}

void URopeComponent::StartFreeGuidedThrow(const FRopeThrowContext& ThrowContext, const FVector& EndpointWorld)
{
	// 허공(대상 없음) 던지기: 손 원점 → 레이 끝점 직선을 아치로 재생하고, 완료 시 꽂힘 없이 Free로 낙하한다.
	// preview는 straight 월드 라인만 채운다 — ResolveGuidePointWorld는 owner-local이 없으면 월드 Points로 폴백한다.
	EnsureRopeInitialized();
	if (Sim.Num() < 2)
	{
		return;
	}

	const FVector Origin = ThrowContext.Origin;

	FRopePreparedThrowPreview Free;
	Free.bValid = false; // 대상 없음 — free 경로는 GuidedThrowState.bFreeThrow로 진행한다(IsValid 불요).
	Free.ThrowContext = ThrowContext;
	Free.RenderPreview.Points.SetNum(Sim.Num());
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		const float Alpha = static_cast<float>(i) / static_cast<float>(Sim.Num() - 1);
		Free.RenderPreview.Points[i] = FMath::Lerp(Origin, EndpointWorld, Alpha);
	}
	Free.RenderPreview.Radius = FMath::Max(0.1f, Radius * 1.05f);
	Free.RenderPreview.NumSides = FMath::Clamp(NumSides, 3, 32);

	ResetStateForNewThrow();
	const FString PhaseReason = FString::Printf(TEXT("free throw to ray-end, len=%.0f"),
		static_cast<float>((EndpointWorld - Origin).Size()));
	if (!BeginGuidedThrowState(MoveTemp(Free), /*bFreeThrow*/ true))
	{
		return;
	}

	SetPhase(ERopePhase::GuidedThrow, *PhaseReason);
}

FRopeWhipGuide::FConfig URopeComponent::MakeWhipGuideConfig() const
{
	FRopeWhipGuide::FConfig Config;
	Config.Duration = WhipConfig.Duration;
	Config.GuidedLength = WhipConfig.GuidedLength;
	Config.SweepAngleDegrees = WhipConfig.SweepAngleDegrees;
	Config.ReferenceThrowSpeed = ThrowParams.ThrowSpeed;
	Config.ComponentRopeLength = RopeLength;
	// CPU/GPU/preview가 동일한 Aim-hit endpoint envelope와 방향 bias를 사용하도록 component 설정을 전달한다.
	Config.AimHitRootSolverFraction = WhipConfig.AimHitRootSolverFraction;
	Config.AimHitTipSolverFraction = WhipConfig.AimHitTipSolverFraction;
	Config.AimHitDirectionBias = WhipConfig.AimHitDirectionBias;
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

#pragma endregion Throw

#pragma region Flight

// ===== Flight ===============================================================

FRopeFlightContactDetector::FParams URopeComponent::MakeFlightDetectParams(float DeltaTime) const
{
	FRopeFlightContactDetector::FParams Params;
	Params.ContactRadius = GetEffectiveContactQueryRadius();
	Params.RopeRadius = Radius;
	Params.PredictiveContactFrames = DetectConfig.PredictiveContactFrames;
	Params.MinLatchNodes = DetectConfig.MinLatchNodes;
	Params.FallbackForward = GetForwardVector();
	// 감지 스윕 해상도(터널링 방지) — GPU step에도 같은 값이 실린다(RequestContactDetection).
	Params.ContactSweepStep = DetectConfig.ContactSweepStep;
	Params.ContactMaxSweepSamples = DetectConfig.ContactMaxSweepSamples;
	// substep dt = FixedDt(=(1/60)/Substeps) — 로프 Verlet 변위(마지막 substep 델타)와 표면속도(cm/s)를 같은
	// 단위로 맞추는 다리(RopeSolverSubsteps의 FixedDt와 동일 식). 프레임 dt가 아님 — 자세한 이유는 FParams 주석.
	Params.SubstepDeltaTime = (1.0f / 60.0f) / static_cast<float>(FMath::Clamp(SolverConfig.Substeps, 1, 16));
	// 프레임 dt: 예측 접촉 외삽이 substep 변위를 프레임 변위로 환산하는 데 쓴다(FParams::FrameDeltaTime 주석).
	Params.FrameDeltaTime = DeltaTime;
	return Params;
}

void URopeComponent::RemoveNonWrappableCandidates(TArray<FRopeContactCandidate>& Candidates) const
{
	// 서브클래스 wrap 대상 게이트(CanWrapTarget): 거른 대상은 트래커/캡처 판정에서 아예 안 보이게
	// 제거한다 — 금지 대상에 트래커가 고착돼 페이즈가 정체되는 것을 막는다. 기본 구현은 전부 true라
	// 필터가 no-op이고, 후보 수가 적어(프레임당 수십 개 상한) 비용은 무시 가능.
	// Flight(후보 산출)와 Contacting(재수집)이 공용 — 조건이 한쪽만 바뀌면 두 페이즈가 서로 다른
	// 후보 집합으로 판정하는 미묘한 버그가 되므로 반드시 이 헬퍼를 거친다.
	Candidates.RemoveAll([this](const FRopeContactCandidate& Candidate)
	{
		// GPU 지연/외부 주입 후보도 모드 정책을 통과시킨다. Assisted는 같은 mesh의 다른 본을
		// secondary/multi-bone 재료로 유지하고, Guaranteed만 exact mesh+bone으로 제한한다.
		return !AimTargeting.IsWrapTarget(Phase, ResolveMode, Candidate.Mesh, Candidate.Bone) ||
			!CanWrapTarget(Candidate.Mesh, Candidate.Bone);
	});
}

TArray<FRopeContactCandidate>& URopeComponent::GetOrBuildFlightContactCandidates(float DeltaTime,
	const FRopeFlightContactDetector::FParams& DetectParams)
{
	TArray<FRopeContactCandidate>& Candidates = SimFrame.bGpuContactsThisFrame
		? SimFrame.GpuFlightCandidates
		: ContactCandidateScratch;

	if (SimFrame.bGpuContactsThisFrame)
	{
		// GPU 감지 경로(G3): actual+predictive 후보 모두 GPU 커널이 산출한 것을 쓴다(귀속·중복제거는
		// 서브시스템이 복원). SimFrame 배열을 직접 후처리해 frame-local 복사/할당을 만들지 않는다.
		// 상대운동 평가(ExpectedWrapTangent는 hand=node0 위치 필요)만 GT에서 돌린다.
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightGpuContacts);
		FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, DetectParams, Candidates);
	}
	else
	{
		Candidates.Reset();
		NextGuideTargetScratch.Reset();
		BuildCpuFlightContactCandidates(DeltaTime, DetectParams, Candidates);
	}

	// CanWrapTarget 게이트(Contacting 재수집과 공용 헬퍼).
	RemoveNonWrappableCandidates(Candidates);
	return Candidates;
}

void URopeComponent::BuildCpuFlightContactCandidates(float DeltaTime,
	const FRopeFlightContactDetector::FParams& DetectParams, TArray<FRopeContactCandidate>& OutCandidates)
{
	// CPU 예측 접촉에만 whip 데이터 뷰가 필요하다. GPU 경로는 subsystem이 dispatch 전에 같은
	// 다음 프레임 타깃을 이미 계산해 GPU step에 실었으므로 Finalize에서 다시 만들지 않는다.
	// 예측이 꺼져 있으면(PredictiveContactFrames<=0) 검출기가 어차피 early-out이라 미리보기를 만들지 않는다.
	// NextGuideTargetScratch는 뷰가 가리키는 멤버 버퍼 — 감지가 끝난 뒤 다음 CPU 사용 때 Reset한다.
	FRopeFlightContactDetector::FWhipGuideView WhipView;
	if (DetectConfig.PredictiveContactFrames > KINDA_SMALL_NUMBER && WhipGuide.GetGuidedNodeMask().Num() > 0)
	{
		WhipGuide.PreviewNextTargets(DeltaTime, Sim, MakeWhipGuideConfig(), NextGuideTargetScratch);
		WhipView.GuidedNodeMask = &WhipGuide.GetGuidedNodeMask();
		WhipView.CurrentTargets = &WhipGuide.GetCurrentTargets();
		WhipView.PrevTargets = &WhipGuide.GetPrevTargets();
		WhipView.NextTargets = &NextGuideTargetScratch;
	}

	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightActualContacts);
		FRopeFlightContactDetector::DetectContactCandidates(Sim, SimFrame.FrameColliders, DetectParams, OutCandidates);
	}
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightPredictiveContacts);
		FRopeFlightContactDetector::AddPredictedContactCandidates(Sim, SimFrame.FrameColliders, DetectParams, WhipView, OutCandidates);
	}
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightEvaluateCandidates);
		FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, DetectParams, OutCandidates);
	}
}

FRopeFlightCaptureEvaluation URopeComponent::EvaluateFlightCapture(
	const TArray<FRopeContactCandidate>& Candidates,
	const FRopeFlightContactDetector::FParams& DetectParams) const
{
	// Assisted aim lock은 dominant만 조준 본으로 고정한다. Tracker.Update는 전체 후보를 집계하므로
	// 같은 mesh의 다른 본은 Targets에 남아 secondary dwell과 multi-bone 경로 재료가 된다.
	const bool bRequireAimPrimary = ResolveMode == ERopeWrapResolveMode::AssistedJudged
		&& AimTargeting.IsLockActive(Phase);
	FRopeFlightCapturePolicy Policy;
	if (bRequireAimPrimary)
	{
		Policy.PreferredMesh = AimTargeting.GetLockedTargetMesh();
		Policy.PreferredBone = AimTargeting.GetLockedTargetBone();
		Policy.bRequirePreferred = true;
	}

	FRopeFlightCaptureEvaluation Evaluation;
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightEvaluateCapture);
		Evaluation = FRopeFlightContactDetector::EvaluateCapture(Candidates, DetectParams, Policy);
	}

	// ③ GuaranteedWrap은 정상 경로로는 Flight를 타지 않는다 — ThrowWithContext가 조준 던지기(prepared)와
	// 허공 던지기(레이 끝점 아치) 양쪽 모두 GuidedThrow로 보낸다. 그래도 어떤 경로로든 Flight에 들어왔다면
	// 캡처는 금지한다: ③의 성립은 GuidedThrow가 확정한 앵커로만 이뤄진다(방어적 백스톱).
	if (ResolveMode == ERopeWrapResolveMode::GuaranteedWrap)
	{
		Evaluation.bShouldCapture = false;
	}
	return Evaluation;
}

bool URopeComponent::ApplyFlightCaptureEvaluation(float DeltaTime,
	const TArray<FRopeContactCandidate>& Candidates, FRopeFlightCaptureEvaluation& Evaluation)
{
	if (Evaluation.bShouldCapture)
	{
		FlightNoContactElapsed = 0.0f;
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightBuildContactingState);
		BuildContactingState(MoveTemp(Evaluation.Tracker), Candidates, DeltaTime);
		SetPhase(ERopePhase::Contacting, *FString::Printf(TEXT("bone=%s, %d node(s)"),
			*ContactTracker.CandidateBone.ToString(), ContactTracker.CandidateNodes.Num()));
		DispatchCaptured(ContactTracker.CandidateBone);
		// 캡처 프레임 자체도 실제 접촉 1프레임이다. 기본 WrapDecisionTime(약 1프레임)을 이미 채웠다면
		// 다음 프레임 재검출을 기다리지 않고 즉시 Wrapping으로 넘겨 움직이는 대상에서 튕김을 줄인다.
		if (ShouldStartWrapping())
		{
			StartWrappingFromContacting();
		}
		return true;
	}

	// Whip이 끝난 뒤 캡처하지 못하고 남아 있으면 실패로 보고 Free로 복귀한다.
	// 후보가 계속 있어도 MinLatchNodes/품질 조건을 넘지 못하면 Flight에 갇힐 수 있으므로 리셋하지 않는다.
	if (!WhipGuide.IsActive())
	{
		const float FlightReturnTime = DetectConfig.FlightNoContactReturnTime > 0.0f
			? DetectConfig.FlightNoContactReturnTime
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
	return false;
}

void URopeComponent::BuildContactingState(FRopeContactTracker&& EvaluatedTracker,
	const TArray<FRopeContactCandidate>& Candidates, float DeltaTime)
{
	ContactTracker = MoveTemp(EvaluatedTracker);
	// 평가 단계의 0초 집계는 대상 선택만 수행하므로 캡처 프레임의 실제 접촉 시간은 여기서 반영한다.
	// 이 프레임을 dwell에 반영해 기본 1-frame decision이 진짜로 같은 프레임에 성립하게 한다.
	const float CapturedFrameDwell = FMath::Max(0.0f, DeltaTime);
	ContactTracker.DwellTime = FMath::Max(ContactTracker.DwellTime, CapturedFrameDwell);
	for (FRopeTrackedContactTarget& Target : ContactTracker.Targets)
	{
		if (Target.Nodes.Num() > 0)
		{
			Target.DwellTime = FMath::Max(Target.DwellTime, CapturedFrameDwell);
		}
	}
	ContactingElapsed = 0.0f;
	PendingWrapSeed = BuildWrapSeedFromContactingState(Candidates);

	// 진행 좌표계 스냅샷은 이 순간이 마지막 기회다 — Contacting부터는 솔브가 없어 노드가 정지하고
	// (Pos==Prev로 수렴) 속도 정보가 죽는다. Wrapping의 CaptureTravelPlane 축이 소비한다.
	CaptureTravelFrame = FRopeCaptureTravelFrame::Compute(Sim, Candidates, DeltaTime);
}

#pragma endregion Flight

