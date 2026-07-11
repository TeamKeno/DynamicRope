// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeComponent.h"
// ResolveBindingWorld — 랩 바인딩(본/소켓/컴포넌트) 트랜스폼 해석의 단일 지점(seam A).
#include "Core/RopeWrapTarget.h"
#include "DynamicRopeLog.h"
#include "Collision/RopeCollider.h"
#include "Render/RopeSceneProxy.h"
// stat 카운터(RopeDebug::Record*)
#include "Debug/RopeDebugDraw.h"
// 게이트플레이 디버거용 한 프레임 디버그 스냅샷
#include "Debug/RopeDebugSnapshot.h"
#include "Camera/CameraComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
// Pull: 캐릭터 견인(CharacterMovement AddForce)
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
// 테더 자동 분배: 물리 바디 질량 조회(본별 GetBodyMass)
#include "PhysicsEngine/BodyInstance.h"
// 거리 LOD(카메라 거리 기준 iteration 감쇠)
#include "Camera/PlayerCameraManager.h"
#include "Kismet/GameplayStatics.h"
// TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)
#include "ProfilingDebugging/CpuProfilerTrace.h"
#include "Subsystem/RopeSimSubsystem.h"
// 디버그 캡처 게이트 + 스냅샷 보관소
#include "Subsystem/RopeDebugSubsystem.h"
#include "Logic/RopeThrowPreviewBuilder.h"
// FRopeGPUSolver::MaxNodes — NumParticles 상한(GPU 솔버 스레드그룹 한도)
#include "RopeGPUSolver.h"
#include "Settings/DynamicRopeSettings.h"
// RopeMath:: 공용 헬퍼 (unity 빌드 익명 네임스페이스 중복 정의 방지)
#include "RopeMathHelpers.h"
#include "DrawDebugHelpers.h"
#include "Materials/MaterialInterface.h"
// 길이 비례 파라미터용 런타임 인스턴스
#include "Materials/MaterialInstanceDynamic.h"
// 기본 머티리얼 로드(FObjectFinder)
#include "UObject/ConstructorHelpers.h"

namespace
{
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
		case ERopePhase::GuidedThrow:return TEXT("GuidedThrow");
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

bool URopeComponent::ThrowWithPreparedPreview(const FRopePreparedThrowPreview& Prepared)
{
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] Prepared preview throw requested (phase=%s, valid=%d, points=%d)"),
		*GetName(), PhaseName(Phase), Prepared.IsValid() ? 1 : 0, Prepared.RenderPreview.Points.Num());

	// PreviewPathLocked의 핵심 진입점: 여기서는 StartFreshThrow처럼 Flight로 보내지 않는다.
	// preview build가 고른 path/contact/anchor를 authoritative하게 사용해야 실제 결과가 preview와 갈라지지 않는다.
	EnsureRopeInitialized();
	if (!Prepared.IsValid() || Prepared.RenderPreview.Points.Num() < 2 || Sim.Num() < 2)
	{
		return false;
	}

	// 서브클래스 wrap 대상 게이트: preview 빌드는 이 게이트를 모르므로(정적 빌더) 진입점에서 거른다.
	if (!CanWrapTarget(Prepared.Mesh.Get(), Prepared.Bone))
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] Prepared preview throw rejected by CanWrapTarget (bone=%s)"),
			*GetName(), *Prepared.Bone.ToString());
		return false;
	}

	if (WrapController.IsActive())
	{
		WrapController.Release(ERopeReleaseReason::Manual);
	}
	ResetTransientPhaseState();
	ReleaseCooldown = 0.0f;

	FRopePreparedThrowPreview ResolvedPrepared = Prepared;
	// 입력 때 저장한 owner-local path를 throw 실행 시점의 owner transform으로 다시 해석한다.
	ResolvedPrepared.RenderPreview = Prepared.ResolveRenderPreviewWorld();
	ResolvedPrepared.ThrowContext.Origin = Prepared.ResolveGuideOriginWorld();
	AimTargeting.SetWrapTargetLock(ResolvedPrepared.ThrowContext);

	// 다음 PrepareSimFrame부터 GuidedThrow가 StartPositions -> RenderPreview.Points로 노드를 구동한다.
	// 완료 시 Prepared.Anchors를 그대로 Wrapped seed로 사용한다.
	GuidedThrowState.Reset();
	GuidedThrowState.bActive = true;
	GuidedThrowState.Prepared = ResolvedPrepared;
	GuidedThrowState.StartPositions = Sim.Positions;
	GuidedThrowState.Elapsed = 0.0f;
	GuidedThrowState.Duration = FMath::Max(0.01f, WrapConfig.WrappingMotionDuration);

	Sim.bStartPinned = true;
	Sim.StartPinPrev = ResolvedPrepared.ThrowContext.Origin;
	Sim.StartPinTarget = ResolvedPrepared.ThrowContext.Origin;
	if (Sim.Positions.IsValidIndex(0))
	{
		Sim.Positions[0] = ResolvedPrepared.ThrowContext.Origin;
		Sim.PrevPositions[0] = ResolvedPrepared.ThrowContext.Origin;
	}

	SetPhase(ERopePhase::GuidedThrow, *FString::Printf(TEXT("prepared points=%d, bone=%s"),
		ResolvedPrepared.RenderPreview.Points.Num(), *ResolvedPrepared.Bone.ToString()));
	return true;
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
	if (Preview.Radius <= KINDA_SMALL_NUMBER || SimFrame.FrameColliders.Num() == 0)
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
		const FVector Direction = RopeMath::ArcDirectionAtAlpha(Preview.AimDir, Preview.GuideUp, Preview.SweepAngleDegrees, AngleAlpha);
		if (!Direction.IsNearlyZero())
		{
			PreviewBounds += Preview.Origin + Direction * Preview.Radius;
		}
	}
	PreviewBounds = PreviewBounds.ExpandBy(EffectiveQueryRadius);

	TArray<const IRopeCollider*, TInlineAllocator<8>> CandidateColliders;
	for (const IRopeCollider* Collider : SimFrame.FrameColliders)
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
		const FVector Direction = RopeMath::ArcDirectionAtAlpha(Preview.AimDir, Preview.GuideUp, Preview.SweepAngleDegrees, AngleAlpha);
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

FRopeAimTargeting::FQueryContext URopeComponent::MakeAimQueryContext() const
{
	// 폴백 치수 규약: ray 길이 = 현재/초기 로프 길이 중 큰 값, 질의 반경 = 튜브/접촉 반경 중 큰 값.
	FRopeAimTargeting::FQueryContext Ctx;
	Ctx.Colliders = &SimFrame.FrameColliders;
	Ctx.FallbackRayLength = FMath::Max(Sim.RopeLength, RopeLength);
	Ctx.FallbackQueryRadius = FMath::Max(Radius, WrapConfig.ContactRadius);
	return Ctx;
}

bool URopeComponent::FindAimRayBoneHit(const FVector& Origin, const FVector& AimDir, float RayLength,
	float QueryRadius, float SweepStep, bool bDrawDebug, FRopeAimRayHitResult& OutHit) const
{
	// 질의 본체는 FRopeAimTargeting(UObject-free). 여기선 컨텍스트 조립 + CanWrapTarget(virtual 게이트) 주입만.
	return FRopeAimTargeting::FindAimRayBoneHit(MakeAimQueryContext(), Origin, AimDir, RayLength,
		QueryRadius, SweepStep, bDrawDebug, GetWorld(),
		[this](const USceneComponent* Mesh, FName Bone) { return CanWrapTarget(Mesh, Bone); }, OutHit);
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

bool URopeComponent::ResolveAimRayThrowContext(const FRopeAimRayThrowRequest& Request,
	FRopeThrowContext& OutContext) const
{
	return FRopeAimTargeting::ResolveAimRayThrowContext(MakeAimQueryContext(), Request, GetWorld(),
		[this](const USceneComponent* Mesh, FName Bone) { return CanWrapTarget(Mesh, Bone); }, OutContext);
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

bool URopeComponent::BuildPreviewContext(const FRopeThrowContext& ThrowContext, FRopePreviewBuildContext& OutContext) const
{
	OutContext = FRopePreviewBuildContext();
	if (Sim.Num() < 2)
	{
		return false;
	}

	// preview가 실제 Flight와 같은 basis/config/속도/collider를 소비하도록 한 번에 스냅샷한다.
	OutContext.ThrowContext = ResolveThrowContext(ThrowContext);
	OutContext.SwingBasis = FRopeWhipGuide::ResolveSwingBasis(
		OutContext.ThrowContext, OutContext.ThrowContext.SwingPlane, OutContext.ThrowContext.CustomSwingPlaneNormal);
	OutContext.WhipConfig = MakeWhipGuideConfig();
	OutContext.Colliders = &SimFrame.FrameColliders;
	OutContext.InheritedVelocity = ComputeThrowInheritedVelocity(OutContext.ThrowContext);
	OutContext.RopeLength = FMath::Max(Sim.RopeLength, RopeLength);
	OutContext.SegmentLength = Sim.SegmentLength;
	OutContext.RopeRadius = Radius;
	OutContext.RopeNumSides = NumSides;
	OutContext.NodeCount = Sim.Num();
	OutContext.Phase = Phase;
	return OutContext.RopeLength > KINDA_SMALL_NUMBER && OutContext.SegmentLength > KINDA_SMALL_NUMBER;
}

bool URopeComponent::BuildWrappingPreview(FRopeWrapPreviewData& OutPreview) const
{
	OutPreview = FRopeWrapPreviewData();
	if (Sim.Num() < 2)
	{
		return false;
	}

	if (Phase == ERopePhase::Flight)
	{
		FRopeThrowPreviewBuilder::FInput Input;
		Input.Sim = &Sim;
		Input.Colliders = &SimFrame.FrameColliders;
		Input.WrapConfig = WrapConfig;
		Input.PathMode = GetWrappingPathMode();
		Input.RopeRadius = Radius;
		Input.RopeNumSides = NumSides;
		Input.FallbackForward = GetForwardVector();
		Input.OwnerName = GetName();
		return FRopeThrowPreviewBuilder::BuildFlightWrappingPreview(Input, OutPreview);
	}

	const USceneComponent* Mesh = nullptr;
	FName Bone = NAME_None;
	FRopeSurfaceAnchor LatchAnchor;

	if (Phase == ERopePhase::Wrapping && WrappingPhase.State.IsActive())
	{
		Mesh = WrappingPhase.State.Mesh.Get();
		Bone = WrappingPhase.State.BoneName;
		LatchAnchor = WrappingPhase.State.LatchAnchor;
	}
	else if (Phase == ERopePhase::Contacting)
	{
		Mesh = PendingWrapSeed.Mesh.Get();
		Bone = PendingWrapSeed.BoneName;
		if (PendingWrapSeed.Anchors.Num() > 0)
		{
			LatchAnchor = PendingWrapSeed.Anchors[0];
		}
		else if (PendingWrapSeed.Latched.Num() > 0)
		{
			const FRopeLatchNode& Latch = PendingWrapSeed.Latched[0];
			if (Mesh && Sim.Positions.IsValidIndex(Latch.NodeIndex))
			{
				const FTransform BoneXform = ResolveBindingWorld(Mesh, Latch.Bone);
				const FVector NormalWorld = FVector::UpVector;
				FVector TangentWorld = FVector::ForwardVector;
				if (Sim.Positions.IsValidIndex(Latch.NodeIndex + 1))
				{
					TangentWorld = (Sim.Positions[Latch.NodeIndex + 1] - Sim.Positions[Latch.NodeIndex])
						.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
				}

				LatchAnchor.NodeIndex = Latch.NodeIndex;
				LatchAnchor.Bone = Latch.Bone;
				LatchAnchor.Mesh = Mesh;
				LatchAnchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(Sim.Positions[Latch.NodeIndex]);
				LatchAnchor.LocalNormal = BoneXform.InverseTransformVectorNoScale(NormalWorld)
					.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
				LatchAnchor.LocalTangent = BoneXform.InverseTransformVectorNoScale(TangentWorld)
					.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
				LatchAnchor.StartWorldPosition = Sim.Positions[Latch.NodeIndex];
				LatchAnchor.SurfaceOffset = FMath::Max(0.0f, Radius);
				LatchAnchor.RopeDistance = 0.0f;
			}
		}
	}
	else
	{
		return false;
	}

	if (!Mesh)
	{
		Mesh = LatchAnchor.Mesh.Get();
	}
	if (!Mesh || Bone.IsNone() || LatchAnchor.Bone.IsNone() ||
		!Sim.Positions.IsValidIndex(LatchAnchor.NodeIndex))
	{
		return false;
	}

	LatchAnchor.Mesh = Mesh;
	LatchAnchor.SurfaceOffset = FMath::Max(0.0f, Radius);

	TArray<FVector> PreviewPoints;
	if (!WrappingPhase.BuildPreviewCenterline(LatchAnchor, Mesh, Bone, Sim, MakeWrappingContext(), PreviewPoints))
	{
		return false;
	}

	OutPreview.Points = MoveTemp(PreviewPoints);
	OutPreview.Radius = FMath::Max(0.1f, Radius * 1.05f);
	OutPreview.NumSides = FMath::Clamp(NumSides, 3, 32);
	return OutPreview.IsValid();
}

bool URopeComponent::BuildWrappingPreview(const FRopeThrowContext& ThrowContext, float ReachScale, int32 SegmentCount,
	float SampleStep, float QueryRadius, FRopeWrapPreviewData& OutPreview, FString* OutFailureReason) const
{
	OutPreview = FRopeWrapPreviewData();

	if (Phase == ERopePhase::Free || Phase == ERopePhase::Releasing)
	{
		FRopeThrowPreviewBuilder::FInput Input;
		Input.Sim = &Sim;
		Input.Colliders = &SimFrame.FrameColliders;
		Input.ThrowContext = ResolveThrowContext(ThrowContext);
		Input.WrapConfig = WrapConfig;
		Input.PathMode = GetWrappingPathMode();
		Input.RopeRadius = Radius;
		Input.RopeNumSides = NumSides;
		Input.RopeLength = FMath::Max(Sim.RopeLength, RopeLength);
		Input.SweepAngleDegrees = MakeWhipGuideConfig().SweepAngleDegrees;
		Input.FallbackForward = GetForwardVector();
		Input.OwnerName = GetName();
		Input.ReachScale = ReachScale;
		Input.SegmentCount = SegmentCount;
		Input.SampleStep = SampleStep;
		Input.QueryRadius = QueryRadius;
		return FRopeThrowPreviewBuilder::BuildFreeWrappingPreview(Input, OutPreview, OutFailureReason);
	}

	if (Phase == ERopePhase::Flight)
	{
		FRopeThrowPreviewBuilder::FInput Input;
		Input.Sim = &Sim;
		Input.Colliders = &SimFrame.FrameColliders;
		Input.WrapConfig = WrapConfig;
		Input.PathMode = GetWrappingPathMode();
		Input.RopeRadius = Radius;
		Input.RopeNumSides = NumSides;
		Input.FallbackForward = GetForwardVector();
		Input.OwnerName = GetName();
		return FRopeThrowPreviewBuilder::BuildFlightWrappingPreview(Input, OutPreview, OutFailureReason);
	}

	const bool bBuilt = BuildWrappingPreview(OutPreview);
	if (!bBuilt)
	{
		RopeMath::SetPreviewFailureReason(OutFailureReason,
			FString::Printf(TEXT("active phase preview failed (phase=%s)"), PhaseName(Phase)));
	}
	return bBuilt;
}

bool URopeComponent::BuildPreparedWrappingPreview(const FRopeThrowContext& ThrowContext, float ReachScale,
	int32 SegmentCount, float SampleStep, float QueryRadius, FRopePreparedThrowPreview& OutPrepared,
	FString* OutFailureReason) const
{
	OutPrepared.Reset();
	// Prepared preview는 아직 던지기 전인 Free/Releasing에서만 의미가 있다.
	// Flight 이후 phase는 이미 실제 접촉/감김 상태가 있으므로 기존 표시용 BuildWrappingPreview 경로를 쓴다.
	if (Phase != ERopePhase::Free && Phase != ERopePhase::Releasing)
	{
		RopeMath::SetPreviewFailureReason(OutFailureReason,
			FString::Printf(TEXT("prepared preview rejected: phase=%s"), PhaseName(Phase)));
		return false;
	}

	FRopeThrowPreviewBuilder::FInput Input;
	Input.Sim = &Sim;
	Input.Colliders = &SimFrame.FrameColliders;
	Input.ThrowContext = ResolveThrowContext(ThrowContext);
	Input.WrapConfig = WrapConfig;
	Input.PathMode = GetWrappingPathMode();
	Input.RopeRadius = Radius;
	Input.RopeNumSides = NumSides;
	Input.RopeLength = FMath::Max(Sim.RopeLength, RopeLength);
	Input.SweepAngleDegrees = MakeWhipGuideConfig().SweepAngleDegrees;
	Input.FallbackForward = GetForwardVector();
	Input.OwnerName = GetName();
	Input.ReachScale = ReachScale;
	Input.SegmentCount = SegmentCount;
	Input.SampleStep = SampleStep;
	Input.QueryRadius = QueryRadius;
	return FRopeThrowPreviewBuilder::BuildFreePreparedPreview(Input, OutPrepared, OutFailureReason);
}

void URopeComponent::FinishWrapRelease(FName Bone, ERopeReleaseReason Reason, const FString& ReasonLog)
{
	// 모든 release 트리거(수동/절단/장력/거리/대상 소실)의 공용 마무리: 페이즈 전환 + 노드 반환 +
	// 일시 상태 폐기 + 쿨다운 + 이벤트. 사유별 차이는 호출자에서 끝난 상태로 들어온다.
	SetPhase(ERopePhase::Releasing, *ReasonLog);
	WrapController.Release(Reason);
	ResetTransientPhaseState();
	ReleaseCooldown = ReleaseCooldownSeconds;
	NotifyReleased(Bone, Reason);
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

// ===== 시뮬레이션 프레임(서브시스템이 3단계로 구동) ===========================

void URopeComponent::PrepareSimFrame(float DeltaTime)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_Prepare);
	// (분리 계약 — 이 함수는 "솔브 입력 생산" 단계다: 솔브 결과가 필요 없는 로직은 전부 여기.
	//  근거와 3단계 역할 분담은 헤더의 Prepare/Solve/Finalize 선언부 주석 참고.)

	EnsureRopeInitialized();
	// 프레임 스코프 — 이번 프레임 로직 산출물을 새로 모은다(G2).
	SimFrame.OverrideFrame.Reset();

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
	if (Phase != ERopePhase::Free && Throttle.IsAsleep())
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
		// latch 이전 구간은 solver가 계속 처리한다. latch~tail은 SimFrame.OverrideFrame mass mask로 고정된다.
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
		UpdateWrappedPullSample(DeltaTime);
		ApplyWrappedTraction(DeltaTime);
		if (CheckWrappedAutoRelease(DeltaTime))
		{
			// 장력/거리 release 발생(솔브 없음).
			break;
		}
		SimFrame.bSolveThisFrame = true;
		break;
	}

	case ERopePhase::GuidedThrow:
		// PreviewPathLocked 전용 phase. 물리 solver/contact detector를 건너뛰고 cached preview path만 따른다.
		UpdateGuidedThrow(DeltaTime);
		SimFrame.bSolveThisFrame = false;
		break;

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
	// Aim-hit collision-free solve도 solver 자체는 실행하되 빈 목록을 넘겨 push-out만 제외한다.
	const TArray<IRopeCollider*> NoSolveColliders;
	const TArray<IRopeCollider*>& SolveColliders = SimFrame.bSolveCollisionsThisFrame
		? SimFrame.FrameColliders
		: NoSolveColliders;
	Solver.Step(Sim, LODConfig, SolveColliders, DeltaTime);
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
	FRopeDebugSnapshot DebugSnapshot;
	FRopeDebugSnapshot* const FlightSnapshot = bDebugCapture ? &DebugSnapshot : nullptr;
#else
	constexpr bool bDebugCapture = false;
	FRopeDebugSnapshot* const FlightSnapshot = nullptr;
#endif

	// Flight: 솔브 후 이동 경로 기반 접촉 후보 감지 → 캡처. 파이프라인 자체는
	// FRopeFlightContactDetector(UObject 비의존)이고, 여기서는 3단계 오케스트레이션만 한다.
	// 스탯/디버거 소비는 전부 ③ 안에 있다 — 본문에는 판정 흐름만 남긴다.
	if (Phase == ERopePhase::Flight)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FinalizeFlight);
		const FRopeFlightContactDetector::FParams DetectParams = MakeFlightDetectParams(DeltaTime);

		TArray<FRopeContactCandidate> Candidates;
		// ① 후보 산출
		BuildFlightContactCandidates(DeltaTime, DetectParams, Candidates);

		// ② 판정/전이
		const bool bShouldCapture = TryCaptureFlightContacts(DeltaTime, Candidates, DetectParams);
		// ③ 관측
		RecordFlightObservation(DetectParams, Candidates, bShouldCapture, FlightSnapshot);
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
	if (Throttle.UpdateSleepState(Phase, Sim, SolverConfig, DeltaTime))
	{
		UE_LOG(LogDynamicRope, Verbose, TEXT("[%s] rope asleep (max speed < %.1f cm/s for %.2fs)"),
			*GetName(), SolverConfig.SleepVelocityThreshold, SolverConfig.SleepDelay);
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
	// M5b: GPU step된 프레임만 resident PosBuf 직접 렌더 허용.
	DynamicData->bGpuResident = SimFrame.bGpuSteppedThisFrame;
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
	// 새 부모로 MID 재생성 + 길이 파라미터 재적용(MarkRenderStateDirty 포함).
	UpdateRopeMaterialDynamicParams();
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
	const ERopePhase OldPhase = Phase;
	Phase = NewPhase;

	// 확장 훅 + BP 이벤트(실제 전이만 — 같은 페이즈 재설정은 알리지 않는다).
	if (OldPhase != NewPhase)
	{
		OnPhaseChanged(OldPhase, NewPhase);
		OnRopePhaseChanged.Broadcast(OldPhase, NewPhase);
	}
}

void URopeComponent::ResetTransientPhaseState()
{
	AimTargeting.ResetPendingThrow();
	ContactTracker.Reset();
	PendingWrapSeed.Reset();
	WrappingPhase.State.Reset();
	GuidedThrowState.Reset();
	ContactingElapsed = 0.0f;
	FlightNoContactElapsed = 0.0f;
	TensionOverTime = 0.0f;
	// Pull 샘플/EMA 3종/경고 래치만. 생존 필드는 FRopePullDriveState 주석 참조.
	PullDrive.ResetTransient();
}

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
	AimTargeting.FilterCollidersToTarget(Phase, SimFrame.FrameColliders);
}

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

	// 길이가 확정되는 지점 — 꼬임 밀도(TwistTurns)를 새 RopeLength에 맞춰 갱신(런타임 길이 변경/재throw 포함).
	UpdateRopeMaterialDynamicParams();

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
		const USceneComponent* Mesh = Wrap.Mesh.Get();
		Snapshot.MeshName = Mesh ? Mesh->GetName() : TEXT("None");
		Snapshot.Latched = Wrap.Latched;
		Snapshot.WrapTension = Wrap.Tension;
		Snapshot.TensionReleaseForce = WrapConfig.TensionReleaseForce;
		Snapshot.bPullValid = PullDrive.LastPullSample.bValid;
		Snapshot.PullPoint = PullDrive.LastPullSample.WorldPoint;
		// 스무딩된(실제 인가) 방향
		Snapshot.PullDirection = PullDrive.LastPullSample.Direction;
		// 스무딩 전 look-ahead(지터 진단)
		Snapshot.PullDirRaw = PullDrive.LastPullDirRaw;
		// raw 정수 조준(홉 진단용 텍스트)
		Snapshot.PullAimNode = PullDrive.LastPullSample.AimNode;
		// 청록 = 스무딩된 fractional 조준(실제 인가)
		Snapshot.PullAimPoint = PullDrive.LastPullSample.bValid ? PullDrive.LastPullSample.AimPos
			: PullDrive.LastPullSample.WorldPoint;
		Snapshot.PullTension = PullDrive.LastPullSample.Tension;
		Snapshot.TetherResponse = WrapConfig.TetherResponse;
		Snapshot.TetherOvershoot = PullDrive.LastTetherOvershoot;
		Snapshot.ActivePullForce = PullDrive.ActivePullForce;
		Snapshot.DistanceReleaseSlack = WrapConfig.DistanceReleaseSlack;
	}

	// 감김 축 시각화: Wrapping 페이즈에서 ResolveWrappingAxis가 정한 경로 축(원점+방향)을 담는다 —
	// [O] 뷰가 선으로 그려 "이번 wrap이 어느 축으로 감기는지"를 눈으로 확인하게 한다.
	if (Phase == ERopePhase::Wrapping && WrappingPhase.State.IsActive())
	{
		Snapshot.bHasWrapAxis = true;
		Snapshot.WrapAxisOrigin = WrappingPhase.State.PathAxisOrigin;
		Snapshot.WrapAxisDirection = WrappingPhase.State.PathAxisDirection;
	}

	// 이 로프가 이번 프레임 질의한 collider 시각화(provider bDrawDebug 대체). 상호 배타 accessor 순서로
	// 실제 형상 분류: 캡슐(세그먼트) / 박스(회전 OBB) / 컨벡스(헐 와이어) / 그 외(SDF 등 월드 AABB 폴백).
	// FrameColliders는 provider 소유라 이 프레임 동안만 유효(GT Phase-3 직렬 실행이라 스레딩 무관).
	Snapshot.Colliders.Reset();
	for (const IRopeCollider* Collider : SimFrame.FrameColliders)
	{
		if (!Collider)
		{
			continue;
		}
		FRopeDebugCollider DC;
		DC.bWorldStatic = Collider->IsWorldStatic();

		// 정적 메시 랩 대상 식별: 가상 본은 있지만(감지 참여) SourceMesh가 스켈레탈이 아니면 랩 대상 셰이프
		// (URopeWrapTargetComponent가 서빙한 박스/캡슐). [O] 뷰에서 스켈레탈 본과 다른 색으로 표시한다.
		{
			FName AttribBone = NAME_None;
			const USceneComponent* AttribMesh = nullptr;
			Collider->GetGPUAttribution(AttribBone, AttribMesh);
			DC.bWrapTarget = !AttribBone.IsNone() && AttribMesh != nullptr
				&& !RopeWrapTargets::IsSkeletalTarget(AttribMesh);
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

	// 노드별 접촉 재질의(디버그 전용): post-solve 노드 위치를 FrameColliders에 다시 질의해 각 노드가 어느 면에
	// 닿았는지(법선)를 기록한다. GPU 런타임은 접촉을 리드백하지 않으므로 여기서 CPU로 다시 질의한다. 질의 반경
	// = CollisionRadius + 여유라 정착(표면에서 ~반경 떨어져 쉬는) 노드도 잡힌다. 노드당 가장 깊은 접촉 1개만.
	Snapshot.NodeContacts.Reset();
	const float DebugQueryRadius = SolverConfig.CollisionRadius + 4.0f;
	for (int32 i = 0; i < Sim.Positions.Num(); ++i)
	{
		const FVector NodePos = Sim.Positions[i];
		FRopeContact Best;
		bool bAny = false;
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
				bAny = true;
			}
		}
		if (bAny)
		{
			FRopeNodeContactDebug NC;
			NC.NodeIndex = i;
			NC.Position = NodePos;
			NC.Normal = Best.Normal;
			NC.Penetration = Best.Penetration;
			NC.Bone = Best.Bone;
			// 정적 월드(박스/컨벡스)는 Bone=None, 스켈레탈은 본 이름 있음.
			NC.bWorldStatic = Best.Bone.IsNone();
			Snapshot.NodeContacts.Add(MoveTemp(NC));
		}
	}
}
#endif

// ===== Throw ================================================================

FRopeThrowContext URopeComponent::MakeDefaultThrowContext(const FVector& /*AimDir*/) const
{
	// 조립 로직은 FRopeThrowContext::MakeDefault(RopeTypes.cpp — 프레임 기저 규약 포함)로 이동.
	// 이 함수는 서브클래스가 조준 규약을 바꾸는 확장 훅으로 남는다(기본 구현 = 공용 조립 위임).
	return FRopeThrowContext::MakeDefault(*this, ThrowParams);
}

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
	AbandonActiveStateForRethrow();
	// ray가 확정한 mesh+bone을 이 throw의 Flight/Contacting/Wrapping 전체에 고정한다.
	AimTargeting.SetWrapTargetLock(ResolvedThrow);
	ResetChainForThrow(ResolvedThrow.Origin);
	BeginWhipSwingFromThrow(ResolvedThrow);
	InjectThrowVelocityIntoVerlet(ResolvedThrow);

	SetPhase(ERopePhase::Flight, *FString::Printf(TEXT("fresh throw impulse, aim=%s, speed=%.1f"),
		*WhipGuide.GetAimDir().ToCompactString(), ResolvedThrow.ThrowSpeed));
}

void URopeComponent::AbandonActiveStateForRethrow()
{
	// 재던지기: 잡고 있던 wrap은 수동 해제, 진행 중 페이즈 일시 상태는 폐기, 쿨다운 없이 즉시 던진다.
	if (WrapController.IsActive())
	{
		WrapController.Release(ERopeReleaseReason::Manual);
	}
	ResetTransientPhaseState();
	ReleaseCooldown = 0.0f;
}

void URopeComponent::ResetChainForThrow(const FVector& HandOrigin)
{
	// 체인 위치를 통째로 재설정하는 곳이므로 GPU 상주 버퍼 재시드 세대(M5)도 여기서 함께 올린다 —
	// 리셋과 재시드는 한 몸이다(따로 두면 한쪽만 하는 버그가 생긴다).
	++SimFrame.SimGeneration;

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
	// 분배: 손→끝으로 갈수록 가중(SmoothStep + tail 가중)하고 TipMass로 끝을 부스트해 채찍처럼 끝이
	// 앞서 나가게 한다. 상속 속도(owner/socket)는 전 노드 균일. ReferenceDt는 첫 스텝 실제 dt와
	// 무관한 고정 환산 기준(프레임레이트에 따라 던지기 세기가 변하지 않게).
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

void URopeComponent::UpdateGuidedThrow(float DeltaTime)
{
	if (!GuidedThrowState.bActive || !GuidedThrowState.Prepared.IsValid() || Sim.Num() < 2)
	{
		SetPhase(ERopePhase::Releasing, TEXT("guided throw invalid"));
		ResetTransientPhaseState();
		ReleaseCooldown = ReleaseCooldownSeconds;
		return;
	}

	GuidedThrowState.Elapsed += DeltaTime;
	const float Alpha = FMath::Clamp(GuidedThrowState.Elapsed / FMath::Max(GuidedThrowState.Duration, 0.01f), 0.0f, 1.0f);
	const float EasedAlpha = Alpha * Alpha * (3.0f - 2.0f * Alpha);
	const FRopePreparedThrowPreview& Prepared = GuidedThrowState.Prepared;

	SimFrame.OverrideFrame.EnsureSize(Sim.Num());
	for (int32 NodeIndex = 0; NodeIndex < Sim.Num(); ++NodeIndex)
	{
		// 1차 구현은 전체 노드를 시작 위치에서 preview 결과 위치로 부드럽게 보간한다.
		// 나중에 모션 품질을 높이면 여기만 front-follow/arc-length sampling 방식으로 교체하면 된다.
		// 매 프레임 현재 owner transform으로 복원하므로 손 소켓 애니메이션에는 종속되지 않고 owner 이동은 따른다.
		FVector Target = Prepared.ResolveGuidePointWorld(NodeIndex);
		if (NodeIndex == 0 && Sim.bStartPinned)
		{
			Target = Prepared.ResolveGuideOriginWorld();
			Sim.StartPinTarget = Target;
		}

		const FVector Start = GuidedThrowState.StartPositions.IsValidIndex(NodeIndex)
			? GuidedThrowState.StartPositions[NodeIndex]
			: Sim.Positions[NodeIndex];
		const FVector Position = FMath::Lerp(Start, Target, EasedAlpha);
		SimFrame.OverrideFrame.SetPosition(NodeIndex, Position, /*bZeroVelocity*/ true);
		SimFrame.OverrideFrame.SetInvMass(NodeIndex, 0.0f);
	}

	if (Alpha >= 1.0f)
	{
		FinishGuidedThrow();
	}
}

void URopeComponent::FinishGuidedThrow()
{
	const FRopePreparedThrowPreview Prepared = GuidedThrowState.Prepared;
	if (!Prepared.IsValid() || Prepared.Anchors.Num() == 0 || !Prepared.Mesh.IsValid())
	{
		SetPhase(ERopePhase::Releasing, TEXT("guided throw commit failed"));
		ResetTransientPhaseState();
		ReleaseCooldown = ReleaseCooldownSeconds;
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

	WrapController.BeginWrap(Sim, Seed, SimFrame.OverrideFrame);
	if (!WrapController.State.IsWrapped())
	{
		SetPhase(ERopePhase::Releasing, TEXT("guided throw begin wrap failed"));
		ResetTransientPhaseState();
		ReleaseCooldown = ReleaseCooldownSeconds;
		return;
	}

	ApplyWrappedMassMask(/*bResetDynamicNodeVelocity*/ true);
	SetPhase(ERopePhase::Wrapped, *FString::Printf(TEXT("guided throw bone=%s, %d anchor(s)"),
		*Seed.BoneName.ToString(), Seed.Anchors.Num()));
	ResetTransientPhaseState();
	NotifyWrapped(Seed.BoneName);
	OnRopeWrapped.Broadcast(Seed.BoneName);
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

// ===== Flight ===============================================================

FRopeFlightContactDetector::FParams URopeComponent::MakeFlightDetectParams(float DeltaTime) const
{
	FRopeFlightContactDetector::FParams Params;
	Params.ContactRadius = WrapConfig.ContactRadius;
	Params.RopeRadius = Radius;
	Params.PredictiveContactFrames = WrapConfig.PredictiveContactFrames;
	Params.MinLatchNodes = WrapConfig.MinLatchNodes;
	Params.FallbackForward = GetForwardVector();
	Params.DeltaTime = DeltaTime;
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
		// GPU 지연 후보나 외부 주입 후보도 ray가 잠근 mesh+bone 이외에는 다음 단계로 넘기지 않는다.
		return !AimTargeting.IsWrapTarget(Phase, Candidate.Mesh, Candidate.Bone) ||
			!CanWrapTarget(Candidate.Mesh, Candidate.Bone);
	});
}

void URopeComponent::BuildFlightContactCandidates(float DeltaTime,
	const FRopeFlightContactDetector::FParams& DetectParams, TArray<FRopeContactCandidate>& OutCandidates)
{
	// whip 가이드 활성 프레임엔 예측 접촉용 데이터 뷰를 구성한다(다음 프레임 타깃 미리보기 포함).
	// 예측이 꺼져 있으면(PredictiveContactFrames<=0) 검출기가 어차피 early-out이라 미리보기를 만들지 않는다.
	// NextGuideTargets는 뷰가 가리키는 로컬 버퍼 — 감지가 이 함수 안에서 끝나므로 수명이 충분하다.
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

	if (SimFrame.bGpuContactsThisFrame)
	{
		// GPU 감지 경로(G3): actual+predictive 후보 모두 GPU 커널이 산출한 것을 쓴다(귀속·중복제거는
		// 서브시스템이 복원). 상대운동 평가(ExpectedWrapTangent는 hand=node0 위치 필요)만 GT에서 돌린다.
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightGpuContacts);
		OutCandidates = SimFrame.GpuFlightCandidates;
		FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, DetectParams, OutCandidates);
	}
	else
	{
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

	// CanWrapTarget 게이트(Contacting 재수집과 공용 헬퍼).
	RemoveNonWrappableCandidates(OutCandidates);
}

bool URopeComponent::TryCaptureFlightContacts(float DeltaTime,
	const TArray<FRopeContactCandidate>& Candidates, const FRopeFlightContactDetector::FParams& DetectParams)
{
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
		NotifyCaptured(ContactTracker.CandidateBone);
		OnRopeCaptured.Broadcast(ContactTracker.CandidateBone);
		return true;
	}

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
	return false;
}

void URopeComponent::RecordFlightObservation(const FRopeFlightContactDetector::FParams& DetectParams,
	const TArray<FRopeContactCandidate>& Candidates, bool bShouldCapture, FRopeDebugSnapshot* OutSnapshot)
{
	// ③ 관측 전용 — 판정(①②)에 관여하지 않는 읽기 소비만 모아둔다. 스탯은 stat 시스템이 수집 중일
	// 때만 실제 비용이 들고, 스냅샷은 디버거 대상 로프만 OutSnapshot으로 넘어온다(그 외 null).
	// 캡처 프레임엔 방금 채워진 ContactTracker를, 아니면 이번 후보로 만든 관측 전용 트래커를 보여준다.
	FRopeContactTracker FlightObserveTracker;
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FlightTrackerUpdate);
		FlightObserveTracker.Update(Candidates, 0.0f);
	}
	const FRopeContactTracker& DebugTracker = bShouldCapture ? ContactTracker : FlightObserveTracker;
	const float WhipGuidedEnd = FMath::Clamp(WhipConfig.GuidedLength, 0.05f, 0.95f);
	const bool bWhipActive = WhipGuide.GetDebugGuideTargets().Num() > 0;
	RopeDebug::RecordFlightStats(Sim, SimFrame.bSolveThisFrame, SimFrame.FrameColliders.Num(), Candidates,
		DebugTracker, WrapConfig, bShouldCapture);
	RopeDebug::RecordWhipStats(Sim, WhipGuide.GetDebugGuideNodeIndices(), WhipGuide.GetDebugGuideTargets(),
		WhipGuidedEnd, bWhipActive);

#if WITH_GAMEPLAY_DEBUGGER
	if (OutSnapshot)
	{
		GatherFlightNodeDebug(DetectParams, OutSnapshot->NodeDebug);
		OutSnapshot->bHasFlight = true;
		OutSnapshot->bSolveThisFrame = SimFrame.bSolveThisFrame;
		OutSnapshot->bShouldCapture = bShouldCapture;
		OutSnapshot->FrameColliderCount = SimFrame.FrameColliders.Num();
		OutSnapshot->MinLatchNodes = WrapConfig.MinLatchNodes;
		OutSnapshot->TrackerBone = DebugTracker.CandidateBone;
		OutSnapshot->TrackerNodes = DebugTracker.CandidateNodes;
		OutSnapshot->Candidates = Candidates;
		OutSnapshot->bWhipActive = bWhipActive;
		OutSnapshot->WhipGuidedEnd = WhipGuidedEnd;
		OutSnapshot->WhipGuideNodeIndices = WhipGuide.GetDebugGuideNodeIndices();
		OutSnapshot->WhipGuideTargets = WhipGuide.GetDebugGuideTargets();
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
		NodeDebug.NodeSpeed = Sim.NodeSpeed(i);
		NodeDebug.bFast = FRopeFlightContactDetector::IsTailNode(Sim, i) || NodeDebug.NodeSpeed > Sim.SegmentLength;
		NodeDebug.bNearBody = FRopeFlightContactDetector::IsNearAnyColliderSegment(
			NodeDebug.PrevPosition, NodeDebug.Position, SimFrame.FrameColliders, DetectParams);
		if (NodeDebug.bFast || NodeDebug.bNearBody)
		{
			NodeDebug.Contact = FRopeFlightContactDetector::SweepOrSampleContact(
				Sim, NodeDebug.PrevPosition, NodeDebug.Position, SimFrame.FrameColliders, DetectParams);
		}

		if (NodeDebug.bFast || NodeDebug.bNearBody || NodeDebug.Contact.bHit)
		{
			OutNodeDebug.Add(NodeDebug);
		}
	}
}
#endif // WITH_GAMEPLAY_DEBUGGER

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
	// 총 체류(아래 정체 안전망 판단용 — 감김 판정 자체는 트래커 dwell).
	ContactingElapsed += DeltaTime;

	// 매 프레임 실제 접촉을 재수집한다 — 캡처 순간의 1회 스냅샷만 믿고 타이머를 돌리던 이전 구조는
	// (1) dismiss가 사실상 불발이었고(트래커 미갱신) (2) 움직이는 대상(랙돌/드래곤)에서 시드와 실제
	// 지오메트리의 어긋남이 WrapDecisionTime 동안 누적됐다. Contacting은 솔브가 없어 노드가 정지
	// 상태라 스윕은 점 질의로 축퇴하고, 대상 이탈은 collider 쪽 이동으로 감지된다.
	// 예측/whip 분기는 Flight 전용이므로 여기서는 actual 접촉만 수집한다(비용: 근접 노드 점 질의뿐).
	const FRopeFlightContactDetector::FParams DetectParams = MakeFlightDetectParams(DeltaTime);
	TArray<FRopeContactCandidate> Candidates;
	FRopeFlightContactDetector::DetectContactCandidates(Sim, SimFrame.FrameColliders, DetectParams, Candidates);
	FRopeFlightContactDetector::EvaluateRelativeMotion(Sim, DetectParams, Candidates);
	// CanWrapTarget 게이트(Flight 후보 산출과 공용 헬퍼).
	RemoveNonWrappableCandidates(Candidates);

	// 트래커 갱신: 같은 본이면 dwell 누적, 지배 본이 바뀌면 dwell 리셋(전이 프레임 오탐 방어 —
	// dwell 재시작 계약을 캡처 후 구간에도 실제로 적용), 접촉이 끊기면 dwell이 소진되며 트래커가
	// 비워져 아래 dismiss로 떨어진다(짧은 플리커는 그동안 쌓인 dwell만큼 관용).
	ContactTracker.Update(Candidates, DeltaTime);

	if (ShouldDismissContacting())
	{
		SetPhase(ERopePhase::Flight, TEXT("contact lost before wrapping"));
		ResetTransientPhaseState();
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
	const float StallTimeout = FMath::Max(WrapConfig.WrapDecisionTime * 10.0f, 1.0f);
	if (ContactingElapsed >= StallTimeout)
	{
		SetPhase(ERopePhase::Flight, *FString::Printf(TEXT("contacting stalled %.2fs (dwell %.2fs < %.2fs)"),
			ContactingElapsed, ContactTracker.DwellTime, WrapConfig.WrapDecisionTime));
		ResetTransientPhaseState();
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
	return ContactTracker.DwellTime >= WrapConfig.WrapDecisionTime
		&& PendingWrapSeed.Latched.Num() > 0
		&& !PendingWrapSeed.BoneName.IsNone();
}

FRopeWrapState URopeComponent::BuildWrapSeedFromContactingState(const TArray<FRopeContactCandidate>& Candidates) const
{
	FRopeWrapState Seed;
	Seed.BoneName = ContactTracker.CandidateBone;
	Seed.Mesh = ContactTracker.CandidateMesh;
	const int32 NodeIndex = RopeMath::HeadValidNodeIndex(ContactTracker.CandidateNodes, Sim.Positions);
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

		const USceneComponent* Mesh = Seed.Mesh.Get();
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

			const FTransform BoneXform = ResolveBindingWorld(Mesh, ContactTracker.CandidateBone);

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
	const USceneComponent* Mesh = PendingWrapSeed.Mesh.Get();
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
			// 뻗어 있는가"를 잡기 위한 값으로, 이후 Analytic Helix / Surface Vector Field에서
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

	const FRopeWrappingPhase::FContext WrappingCtx = MakeWrappingContext();
	WrappingPhase.AdvancePathBuild(Sim, WrappingCtx);

	// 안전장치: 표면 경로 생성이 중간에 실패했을 때, 그때까지 감싼 각도가 임계 미만이면 "조금 닿았는데
	// 바로 wrapped로 철썩 붙는" 상태를 만들지 않고 release한다. 기준은 회전 수가 아니라 감싼 각도(도,
	// FailedWrapMinAngleDeg — 0이면 가드 끔): 회전 수는 로프 2πr을 요구해 큰 대상(드래곤 몸통)에서
	// 물리적으로 도달 불가능한 기준이 됐다. 상세는 config 주석.
	float FailedWrapAngleDeg = 0.0f;
	if (WrappingPhase.ShouldAbortFailedShortWrap(Sim, WrappingCtx, WrapConfig.FailedWrapMinAngleDeg, FailedWrapAngleDeg))
	{
		SetPhase(ERopePhase::Releasing, *FString::Printf(TEXT("wrap path failed early, angle=%.0fdeg < %.0fdeg"),
			FailedWrapAngleDeg, WrapConfig.FailedWrapMinAngleDeg));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	WrappingPhase.ApplyFrontMotion(Sim, DeltaTime, WrappingCtx, SimFrame.OverrideFrame);

	WrappingPhase.ApplyMassMask(Sim, SimFrame.OverrideFrame);

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
	return FRopeWrappingPhase::FContext{ WrapConfig, SimFrame.FrameColliders, GetWrappingPathMode(), Radius, GetName() };
}

void URopeComponent::CommitWrapping()
{
	const USceneComponent* Mesh = WrappingPhase.State.Mesh.Get();

	//Wrapping 정보가 적절하지 않으면 바로 releasing
	if (!Mesh || WrappingPhase.State.BoneName.IsNone() || WrappingPhase.State.Anchors.Num() == 0)
	{
		SetPhase(ERopePhase::Releasing, TEXT("commit failed"));
		AbortWrapping(ERopeReleaseReason::Broken);
		return;
	}

	// 커밋 시점 감싼 각도(도): 커밋 품질 관문(아래)과 전이 로그가 공용으로 쓴다. 실패 조기 abort와
	// 같은 척도라 로그의 angle 수치를 그대로 비교/튜닝에 쓸 수 있다. 계산 불가(축 축퇴 등)면 -1 표기.
	float CommitAngleDeg = -1.0f;
	WrappingPhase.ComputeWrappedAngleAtLastBuiltPoint(Sim, MakeWrappingContext(), CommitAngleDeg);

	// 커밋 품질 관문(opt-in — CommitMinWrapAngleDeg 0이면 기존 동작 그대로): 경로가 정상 완료됐거나
	// settle 타임아웃으로 왔어도, 감은 각도가 하한 미만인 부실 랩은 Wrapped로 확정하지 않는다.
	if (WrapConfig.CommitMinWrapAngleDeg > 0.0f && CommitAngleDeg >= 0.0f
		&& CommitAngleDeg < WrapConfig.CommitMinWrapAngleDeg)
	{
		SetPhase(ERopePhase::Releasing, *FString::Printf(TEXT("commit quality below threshold, angle=%.0fdeg < %.0fdeg"),
			CommitAngleDeg, WrapConfig.CommitMinWrapAngleDeg));
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

	// 감길 mesh는 Seed.Mesh로 전파(접촉 유래, cross-actor 포함).
	WrapController.BeginWrap(Sim, Seed, SimFrame.OverrideFrame);
	ApplyWrappedMassMask(/*bResetDynamicNodeVelocity*/ true);

	SetPhase(ERopePhase::Wrapped, *FString::Printf(TEXT("bone=%s, %d latched node(s), angle=%.0fdeg"),
		*Seed.BoneName.ToString(), Seed.Latched.Num(), CommitAngleDeg));
	ResetTransientPhaseState();
	NotifyWrapped(Seed.BoneName);
	OnRopeWrapped.Broadcast(Seed.BoneName);
}

void URopeComponent::AbortWrapping(ERopeReleaseReason Reason)
{
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] AbortWrapping reason=%d"),
		*GetName(), static_cast<int32>(Reason));

	WrappingPhase.ReturnNodesToSolver(Sim, SimFrame.OverrideFrame);

	ResetTransientPhaseState();
	ReleaseCooldown = ReleaseCooldownSeconds;
}

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
	ApplyWrappedMassMask();
	return true;
}

void URopeComponent::UpdateWrappedPullSample(float DeltaTime)
{
	// ② 장력 모델: 솔버가 채운 세그먼트 장력(F=λ/h², GPU 로프는 1~2프레임 지연 미러)의 최대치를
	// wrap 상태에 반영한다. 게임플레이(당김/절단 판정)와 디버거가 이 값을 읽는다.
	WrapController.State.Tension = GetMaxTension();

	// Pull 샘플 산출(항상 — 디버거/BP 관찰 + 견인/release의 공용 입력). 방향은 첫 직선 다리 추종(공간).
	PullDrive.LastPullSample = FRopePullSample();
	WrapController.ComputePull(Sim, WrapConfig.PullBendThresholdDeg, PullDrive.LastPullSample);

	// Pull 스무딩(2단): (1) 조준 노드 fractional 스무딩 — 정수 AimNode의 프레임 간 이산 홉(방향 통째 점프
	// + tether 초과분 불연속)을 float EMA + 노드 사이 보간으로 없앤다. (2) 방향 EMA — 그 위에 남는 노드 위치
	// 노이즈(GPU 미러 지연 등)를 다듬는다. wrap 시작 후 첫 유효 프레임은 측정값으로 시드(래그 없음).
	if (!PullDrive.LastPullSample.bValid)
	{
		return;
	}

	// 스무딩 전 raw look-ahead(정수 조준) — 디버거 raw vs smoothed 비교.
	PullDrive.LastPullDirRaw = PullDrive.LastPullSample.Direction;

	// (1) 조준 인덱스 시간 스무딩 → fractional 조준 위치 보간.
	const float RawAimF = static_cast<float>(PullDrive.LastPullSample.AimNode);
	if (PullDrive.SmoothedAimNodeF < 0.0f)
	{
		PullDrive.SmoothedAimNodeF = RawAimF;
	}
	else
	{
		const float TauA = WrapConfig.PullAimSmoothTime;
		const float AlphaA = (TauA > KINDA_SMALL_NUMBER) ? (1.0f - FMath::Exp(-DeltaTime / TauA)) : 1.0f;
		PullDrive.SmoothedAimNodeF = FMath::Lerp(PullDrive.SmoothedAimNodeF, RawAimF, AlphaA);
	}
	const float AimF = FMath::Clamp(PullDrive.SmoothedAimNodeF, 0.0f, static_cast<float>(PullDrive.LastPullSample.AnchorNode));
	const int32 A0 = FMath::FloorToInt(AimF);
	const int32 A1 = FMath::Min(A0 + 1, PullDrive.LastPullSample.AnchorNode);
	const FVector AimPos = FMath::Lerp(Sim.Positions[A0], Sim.Positions[A1], AimF - static_cast<float>(A0));
	PullDrive.LastPullSample.AimNodeF = AimF;
	PullDrive.LastPullSample.AimPos = AimPos;

	// (2) 연속 조준으로 방향 재계산 후 방향 EMA. 축퇴(조준=앵커)면 raw 방향 유지.
	const FVector DirF = (AimPos - Sim.Positions[PullDrive.LastPullSample.AnchorNode]).GetSafeNormal();
	const FVector DirIn = DirF.IsNearlyZero() ? PullDrive.LastPullSample.Direction : DirF;
	if (PullDrive.SmoothedPullDir.IsNearlyZero())
	{
		PullDrive.SmoothedPullDir = DirIn;
	}
	else
	{
		const float Tau = WrapConfig.PullDirSmoothTime;
		const float Alpha = (Tau > KINDA_SMALL_NUMBER) ? (1.0f - FMath::Exp(-DeltaTime / Tau)) : 1.0f;
		PullDrive.SmoothedPullDir = FMath::Lerp(PullDrive.SmoothedPullDir, DirIn, Alpha).GetSafeNormal();
	}
	PullDrive.LastPullSample.Direction = PullDrive.SmoothedPullDir;
}

void URopeComponent::ApplyWrappedTraction(float DeltaTime)
{
	// ③-1 자동 견인(테더, 위치/속도 동기): 가용 로프 길이 초과분만큼 양끝(TetherTargetShare 분배)을
	// 되돌린다. 장력 비례 힘(폭주: 힘→스트레치→장력↑→힘↑)을 대체 — 초과분 기반이라 수렴한다.
	UpdateTether(DeltaTime);

	// ③-2 능동 Pull(상수 힘): 사용자 입력(SetActivePull/Wielder)이 준 힘을 팽팽할 때만 인가한다.
	// 장력과 무관한 상수라 피드백 폭주가 없다.
	if (PullDrive.ActivePullForce > 0.0f && PullDrive.LastPullSample.bValid && PullDrive.LastPullSample.Tension > KINDA_SMALL_NUMBER)
	{
		ApplyPullForce(PullDrive.LastPullSample.Direction * PullDrive.ActivePullForce, PullDrive.LastPullSample);
	}
}

bool URopeComponent::CheckWrappedAutoRelease(float DeltaTime)
{
	// ④-1 임계 장력 release: 최대 장력이 TensionReleaseForce를 TensionReleaseTime 동안 지속해 넘으면
	// 풀린다(순간 스파이크 무시). 0 = 비활성. 흐름은 mesh-lost release와 동일, 사유만 Tension.
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
			return true;
		}
	}

	// ④-2 거리 release: 손~앵커 직선 거리의 가용 로프 길이 초과분(테더 초과분과 동일 소스 —
	// UpdateTether가 이번 프레임 갱신한 PullDrive.LastTetherOvershoot)이 한계를 넘으면 놓친다.
	// 기하 기반이라 지속 시간 없이 즉시 판정(장력처럼 노이즈가 없다).
	if (WrapConfig.DistanceReleaseSlack > 0.0f && PullDrive.LastTetherOvershoot > WrapConfig.DistanceReleaseSlack)
	{
		const FName Bone = WrapController.State.BoneName;
		FinishWrapRelease(Bone, ERopeReleaseReason::Distance,
			FString::Printf(TEXT("distance release overshoot %.0f > %.0f, bone=%s"),
				PullDrive.LastTetherOvershoot, WrapConfig.DistanceReleaseSlack, *Bone.ToString()));
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
}

void URopeComponent::SetActivePull(float Force)
{
	PullDrive.ActivePullForce = FMath::Max(0.0f, Force);
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
	// 카메라 거리 산출만 GT/UObject 접근 — 배율 계산은 Throttle(순수)에 위임.
	// 로컬 플레이어 카메라 기준(멀티 로컬 플레이어는 0번만 — LOD는 근사여도 무방). 서버/카메라 없음 = 풀 품질.
	TOptional<float> CameraDist;
	if (SolverConfig.bEnableDistanceLOD && SolverConfig.LODStartDistance > 0.0f)
	{
		if (const APlayerCameraManager* Camera = UGameplayStatics::GetPlayerCameraManager(GetWorld(), 0))
		{
			CameraDist = static_cast<float>(FVector::Dist(Camera->GetCameraLocation(), GetComponentLocation()));
		}
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
	SetRopeLength(Sim.RopeLength - ReelRate * DeltaTime);
}

namespace
{
	// 본에서 부모 체인을 올라가 가장 가까운 "시뮬 중인 피직스 바디"의 본을 찾는다(없으면 None).
	// 감긴 본이 트위스트 본 등 피직스 에셋에 바디가 없는 본일 수 있다 — 그 경우 본 이름만 보고
	// 비시뮬 판정해 캐릭터 무브먼트 분기로 빠지면, 랙돌 셋업이 무브먼트를 꺼둔 상태(MOVE_None)라
	// AddForce가 조용히 버려진다. 힘/속도 인가 본은 이 함수로 승격해 찾는다.
	FName FindNearestSimulatingBone(const USkeletalMeshComponent* Mesh, FName Bone)
	{
		while (!Bone.IsNone())
		{
			if (Mesh->IsSimulatingPhysics(Bone))
			{
				return Bone;
			}
			Bone = Mesh->GetParentBone(Bone);
		}
		return NAME_None;
	}

	// 캐릭터 무브먼트가 지금 힘을 소비할 수 있는가. MOVE_None(DisableMovement — 랙돌 셋업 관례)이면
	// AddForce가 누적만 되고 소비되지 않아 "성공한 척" 힘이 사라진다 — 그 경우 다른 수신자로 넘긴다.
	// wrap 대상 액터를 직접 받는다(대상이 스켈레탈/정적/물리프랍 무엇이든 무관 — 소유 액터 기준 판정).
	UCharacterMovementComponent* GetForceConsumingMovement(const AActor* Owner)
	{
		const ACharacter* Character = Cast<ACharacter>(Owner);
		UCharacterMovementComponent* Movement = Character ? Character->GetCharacterMovement() : nullptr;
		return (Movement && Movement->MovementMode != MOVE_None) ? Movement : nullptr;
	}

	// 테더 자동 분배용 유효 역질량(w = 1/유효질량). 0 = 앵커(무한질량). 물리 바디 질량은 UE가 콜리전
	// 볼륨×밀도로 자동 유지하는 값을 읽으므로 별도 세팅이 필요 없다. 판정 순서는 UpdateTether의 수신자
	// 체인(스켈레탈 본 → 시뮬 프리미티브 → 소유 루트 → 캐릭터)과 동일 — 질량과 실제 인가점이 일치한다.
	//  - 스켈레탈 풀 랙돌: 승격된 시뮬 본의 바디 질량.
	//  - 시뮬 프리미티브(대상 자체/루트): GetMass().
	//  - 캐릭터: 접지=유한 브레이스(Mass×GroundBraceFactor — 발 디딤 저항), 공중=Mass, MOVE_None=앵커.
	//  - 그 외(정적/키네마틱/비시뮬 비캐릭터): 앵커(0). wielder는 MeshComp=null로 호출(루트/캐릭터만 해석).
	float ResolveEndpointInvMass(const USceneComponent* MeshComp, const AActor* Owner, FName WrappedBone, float GroundBraceFactor)
	{
		auto InvFromMass = [](float Mass) -> float
		{
			return (Mass > KINDA_SMALL_NUMBER) ? (1.0f / Mass) : 0.0f;
		};

		// (1) 스켈레탈 풀 랙돌: 감긴 본(부모 체인 승격)의 물리 바디 질량.
		if (const USkeletalMeshComponent* Skel = Cast<USkeletalMeshComponent>(MeshComp))
		{
			if (Skel->IsSimulatingPhysics())
			{
				const FName SimBone = FindNearestSimulatingBone(Skel, WrappedBone);
				if (!SimBone.IsNone())
				{
					if (const FBodyInstance* Body = Skel->GetBodyInstance(SimBone))
					{
						return InvFromMass(static_cast<float>(Body->GetBodyMass()));
					}
				}
			}
		}
		// (2) 대상 컴포넌트 자체가 시뮬 중인 프리미티브(가벼운 물리 프랍 등).
		if (const UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(MeshComp))
		{
			if (Prim->IsSimulatingPhysics())
			{
				return InvFromMass(static_cast<float>(Prim->GetMass()));
			}
		}
		// (3) 소유 액터 루트 프리미티브가 시뮬 중.
		if (Owner)
		{
			if (const UPrimitiveComponent* Root = Cast<UPrimitiveComponent>(Owner->GetRootComponent()))
			{
				if (Root->IsSimulatingPhysics())
				{
					return InvFromMass(static_cast<float>(Root->GetMass()));
				}
			}
		}
		// (4) 캐릭터: 접지=유한 브레이스, 공중=Mass, MOVE_None=앵커.
		if (const ACharacter* Character = Cast<ACharacter>(Owner))
		{
			if (const UCharacterMovementComponent* Movement = Character->GetCharacterMovement())
			{
				if (Movement->MovementMode == MOVE_None)
				{
					return 0.0f;
				}
				const float BraceScale = Movement->IsMovingOnGround() ? FMath::Max(GroundBraceFactor, 1.0f) : 1.0f;
				return InvFromMass(Movement->Mass * BraceScale);
			}
		}
		// (5) 정적/키네마틱/비시뮬 비캐릭터 → 앵커.
		return 0.0f;
	}
}

void URopeComponent::UpdateTether(float DeltaTime)
{
	// 초과분(overshoot) = 앵커에서 "조준 노드"(walk가 찾은 첫 직선 다리 끝 = 손 또는 벽 모서리)까지의 실제
	// 직선 거리가 그 구간의 가용 로프 길이를 넘는 양. 손이 아니라 조준 노드를 기준으로 삼는 이유: 로프가 벽에
	// 걸려 우회하면 손 직선은 장애물 뒤라 영영 안 터지지만(무반응), 모서리(조준) 기준이면 그 다리로 제대로
	// 발화한다. 곧은 로프는 조준=손(노드 0)이라 기존과 등가. 스냅샷/BP 관찰을 위해 꺼져 있어도 항상 계산.
	PullDrive.LastTetherOvershoot = 0.0f;
	if (!PullDrive.LastPullSample.bValid || PullDrive.LastPullSample.AimNodeF < 0.0f)
	{
		return;
	}
	// fractional 조준(연속): 정수 AimNode 대신 스무딩된 조준 위치/세그먼트 수를 써 초과분이 노드 단위로 뚝뚝
	// 튀지 않고 연속으로 변한다 → 견인이 매끈해진다(어제 "뚝뚝 끊김"의 원인이 이 이산 참조였다).
	// 끌 지점(대상 쪽 앵커)
	const FVector Anchor = PullDrive.LastPullSample.WorldPoint;
	// 보간된 조준(손 또는 벽 모서리)
	const FVector Aim    = PullDrive.LastPullSample.AimPos;
	// 연속 세그먼트 수
	const float   LegSegs = static_cast<float>(PullDrive.LastPullSample.AnchorNode) - PullDrive.LastPullSample.AimNodeF;
	const FVector Span = Aim - Anchor;
	const float Dist = static_cast<float>(Span.Size());
	const float AvailLen = LegSegs * Sim.SegmentLength + WrapConfig.TetherSlack;
	const float Overshoot = Dist - AvailLen;
	PullDrive.LastTetherOvershoot = FMath::Max(0.0f, Overshoot);
	if (WrapConfig.TetherResponse <= 0.0f || Overshoot <= 0.0f || Dist <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	// 방향 = 앵커에서 조준(모서리/손) 쪽 = 스무딩된 look-ahead 방향(둘 다 로프 경로 추종). 폴백은 이 구간 직선.
	const FVector DirToAim = PullDrive.SmoothedPullDir.IsNearlyZero() ? (Span / Dist) : PullDrive.SmoothedPullDir;
	// 이번 프레임 회수량 = 초과분 × 반응(위치 동기 — 남은 초과분이 다음 입력이라 수렴). 최대 속도로 클램프해
	// 초과분 스파이크(코너 전이 등)에도 대상이 튕겨나가지 않게 한다.
	const float Response = FMath::Clamp(WrapConfig.TetherResponse, 0.0f, 1.0f);
	// 이번 프레임 최대 이동(cm)
	const float MaxStep = FMath::Max(WrapConfig.TetherMaxSpeed, 0.0f) * DeltaTime;
	const float StepLen = (MaxStep > 0.0f) ? FMath::Min(Overshoot * Response, MaxStep) : (Overshoot * Response);

	// State.Mesh는 이제 USceneComponent(정적 랩 대비 일반화). 대상 타입을 가리지 않고 아래 수신자 체인
	// (스켈레탈 본 → 시뮬 프리미티브 → 캐릭터 무브먼트/스윕)으로 견인한다 — 가벼운 물리 프랍/정적 대상도
	// 스켈레탈과 동일 로직으로 끌린다. null은 대상 컴포넌트가 파괴로 소실된 경우뿐(그땐 Hold가 이미 release).
	USceneComponent* MeshComp = const_cast<USceneComponent*>(WrapController.State.Mesh.Get());
	if (!MeshComp)
	{
		return;
	}

	// 초과분 회수를 대상/wielder(로프 owner) 양끝에 분배한다(양끝이 서로 각자 몫만큼 움직여 합이 초과분을
	// 넘지 않음 — 과수렴 없음). 몫 산출:
	//  - 자기 자신에 감긴 로프(owner==대상): 분배 무의미 → 전량 대상.
	//  - 자동(bAutoTetherShare): 양끝 유효 역질량으로 나눈다(무거울수록/앵커일수록 덜 움직임). 접지↔공중/
	//    질량 변화로 프레임 간 튀는 것은 EMA로 흡수. 양끝 다 앵커면 아무도 안 움직임(로프가 한계에서 버팀 —
	//    거리 release가 처리).
	//  - 수동(오버라이드 우선): TetherTargetShare 고정 비율. 1=전량 대상(질량 무관 강제, wielder가 대상을
	//    전부 끌고 옴), 0=전량 wielder(앵커 매달리기/등반).
	const bool bSelfWrap = (GetOwner() != nullptr && MeshComp->GetOwner() == GetOwner());
	float ShareT = 1.0f; // 대상 몫 [0..1]
	float ShareW = 0.0f; // wielder 몫
	if (bSelfWrap)
	{
		ShareT = 1.0f;
		ShareW = 0.0f;
	}
	else if (WrapConfig.bAutoTetherShare)
	{
		const float WT = ResolveEndpointInvMass(MeshComp, MeshComp->GetOwner(), PullDrive.LastPullSample.Bone, WrapConfig.GroundBraceFactor);
		const float WW = ResolveEndpointInvMass(nullptr, GetOwner(), NAME_None, WrapConfig.GroundBraceFactor);
		const float Total = WT + WW;
		if (Total <= KINDA_SMALL_NUMBER)
		{
			// 양끝 다 앵커(정적/MOVE_None) — 아무도 안 움직임.
			ShareT = 0.0f;
			ShareW = 0.0f;
		}
		else
		{
			const float RawShareT = WT / Total; // 무거운 쪽 = 작은 w → 작은 몫.
			if (PullDrive.SmoothedTargetShare < 0.0f)
			{
				PullDrive.SmoothedTargetShare = RawShareT; // 첫 유효 프레임은 측정값으로 시드(래그 없음).
			}
			else
			{
				const float Tau = WrapConfig.PullDirSmoothTime;
				const float Alpha = (Tau > KINDA_SMALL_NUMBER) ? (1.0f - FMath::Exp(-DeltaTime / Tau)) : 1.0f;
				PullDrive.SmoothedTargetShare = FMath::Lerp(PullDrive.SmoothedTargetShare, RawShareT, Alpha);
			}
			ShareT = FMath::Clamp(PullDrive.SmoothedTargetShare, 0.0f, 1.0f);
			ShareW = 1.0f - ShareT;
		}
	}
	else
	{
		ShareT = FMath::Clamp(WrapConfig.TetherTargetShare, 0.0f, 1.0f);
		ShareW = 1.0f - ShareT;
	}
	PullDrive.LastTargetShare = ShareT; // wielder 게이트/디버그가 읽는 유효 대상 몫.
	const float TargetStep = StepLen * ShareT;
	const float WielderStep = StepLen * ShareW;

	// 물리 시뮬 대상(본/루트): 속도를 *누적하지 않고* 목표 속도(step/dt)까지만 톱업한다 — 이미 그 방향으로
	// 충분히 빠르면 아무것도 더하지 않는다. bVelChange로 매 프레임 임펄스를 더하던 기존 방식은 물리 운동량이
	// 이월돼 속도가 누적 → 발산(맵 밖)했다. 여기서는 목표를 넘지 않게 차분만 주므로 수렴하고, 수직 성분(중력
	// 등)은 보존된다. step이 이미 최대 속도로 클램프돼 있어 상한도 보장.
	const float InvDt = 1.0f / FMath::Max(DeltaTime, 1e-4f);
	auto TopUpVelocity = [&](UPrimitiveComponent* Prim, FName BoneName, const FVector& Dir, float TargetSpeed)
	{
		const FVector CurVel = Prim->GetPhysicsLinearVelocity(BoneName);
		const float CurAlong = static_cast<float>(FVector::DotProduct(CurVel, Dir));
		if (TargetSpeed > CurAlong)
		{
			Prim->AddImpulse(Dir * (TargetSpeed - CurAlong), BoneName, /*bVelChange*/ true);
		}
	};

	// 비시뮬 수신자: 캐릭터 무브먼트가 살아 있으면 위치 오프셋 대신 *무브먼트 속도* 톱업으로 끈다.
	// 프레임당 AddActorWorldOffset은 위치 계단이라 견인이 틱틱 끊기고(초과분 쌓임→보정→슬랙 반복)
	// wielder 수신에서는 카메라 흔들림으로 도드라졌다. 속도로 주면 무브먼트가 자체 스윕/보간으로
	// 통합해 부드럽다. 물리 경로와 같은 "목표 속도까지 차분만" 원칙이라 누적/발산이 없고, walking은
	// 수직 성분을 무브먼트가 버리므로 상향 견인은 지상 이탈(Wielder의 bAutoGroundExitOnUpwardPull)이
	// 선행된다. 무브먼트가 없거나 꺼진(MOVE_None) 액터는 기존 스윕 오프셋 폴백(벽 통과 방지).
	auto ApplyNonSimCorrection = [&](AActor* Actor, const FVector& Dir, float Step)
	{
		if (const ACharacter* Character = Cast<ACharacter>(Actor))
		{
			UCharacterMovementComponent* Movement = Character->GetCharacterMovement();
			if (Movement && Movement->MovementMode != MOVE_None)
			{
				const float TargetSpeed = Step * InvDt;
				const float CurAlong = static_cast<float>(FVector::DotProduct(Movement->Velocity, Dir));
				if (TargetSpeed > CurAlong)
				{
					FVector NewVel = Movement->Velocity + Dir * (TargetSpeed - CurAlong);
					// 주입 결과 속력의 절대 상한 = max(TetherMaxSpeed, 기존 속력): 방향이 흔들리면 톱업이
					// 프레임마다 다른 축으로 들어가 감쇠 없는 Falling에서 벡터가 계속 커질 수 있다(폭주의
					// 2차 방어 — 1차는 방향 EMA). 주입은 절대 속력을 이 상한 너머로 못 키우고, 기존에 더
					// 빠른 외부 운동(자유낙하 등)은 보존한다. TetherMaxSpeed=0(클램프 없음 설정)이면 생략.
					const float SpeedCap = FMath::Max(WrapConfig.TetherMaxSpeed, 0.0f);
					if (SpeedCap > 0.0f)
					{
						const float MaxAllowed = FMath::Max(SpeedCap, static_cast<float>(Movement->Velocity.Size()));
						NewVel = NewVel.GetClampedToMaxSize(MaxAllowed);
					}
					Movement->Velocity = NewVel;
				}
				return;
			}
		}
		Actor->AddActorWorldOffset(Dir * Step, /*bSweep*/ true);
	};

	// ---- 대상 몫 ----
	if (TargetStep > KINDA_SMALL_NUMBER)
	{
		bool bHandled = false;
		// (1) 스켈레탈 + 풀 랙돌: 감긴 본이 시뮬 중이어도 메시 전체가 시뮬(풀 랙돌)일 때만 본 속도 톱업으로
		// 처리한다. 부분 랙돌(메시 루트 바디는 키네마틱, 서브트리만 시뮬)은 시뮬 본이 키네마틱 부모에
		// 구속돼 있어 — 키네마틱은 사실상 무한질량 — 본에 준 속도가 구속에 다시 잡아먹혀 액터가 끌려오지
		// 않는다. 그 경우 아래 프리미티브/액터 오프셋 경로로 떨어뜨려 이동체를 직접 회수한다(시뮬 팔다리는
		// 구속으로 따라온다). 인가 본은 감긴 본에서 부모 체인 승격(FindNearestSimulatingBone) — 바디 없는
		// 본(트위스트 등) 대응.
		if (USkeletalMeshComponent* Skel = Cast<USkeletalMeshComponent>(MeshComp))
		{
			if (Skel->IsSimulatingPhysics())
			{
				const FName SimBone = FindNearestSimulatingBone(Skel, PullDrive.LastPullSample.Bone);
				if (!SimBone.IsNone())
				{
					TopUpVelocity(Skel, SimBone, DirToAim, TargetStep * InvDt);
					bHandled = true;
				}
			}
		}
		// (2) wrap 대상 컴포넌트 자체가 시뮬 중인 프리미티브(가벼운 물리 프랍 등): 컴포넌트 속도 톱업으로 견인.
		if (!bHandled)
		{
			if (UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(MeshComp))
			{
				if (Prim->IsSimulatingPhysics())
				{
					TopUpVelocity(Prim, NAME_None, DirToAim, TargetStep * InvDt);
					bHandled = true;
				}
			}
		}
		AActor* TargetOwner = MeshComp->GetOwner();
		// (3) 소유 액터 루트 프리미티브가 시뮬 중(물리 액터에 붙은 컴포넌트 구성).
		if (!bHandled && TargetOwner)
		{
			if (UPrimitiveComponent* Root = Cast<UPrimitiveComponent>(TargetOwner->GetRootComponent()))
			{
				if (Root->IsSimulatingPhysics())
				{
					TopUpVelocity(Root, NAME_None, DirToAim, TargetStep * InvDt);
					bHandled = true;
				}
			}
		}
		// (4) 캐릭터/비시뮬 대상: 속도 톱업(캐릭터) 또는 스윕 위치 보정 폴백. step이 클램프돼 텔레포트 없음.
		if (!bHandled && TargetOwner)
		{
			ApplyNonSimCorrection(TargetOwner, DirToAim, TargetStep);
		}
	}

	// ---- wielder 몫: 같은 초과분을 로프 owner를 로프 쪽으로 당겨 회수한다 ----
	if (WielderStep > KINDA_SMALL_NUMBER)
	{
		// 방향 = 손(노드 0)에서 로프의 첫 직선 다리를 따라. 조준(AimPos)이 벽 모서리면 모서리를 향하고,
		// 로프가 곧아 조준=손이면(chord ~0) 앵커→조준의 역방향(=손→앵커)으로 폴백한다.
		const FVector HandPos = Sim.Positions.IsValidIndex(0) ? Sim.Positions[0] : Aim;
		FVector WielderDirRaw = Aim - HandPos;
		if (!WielderDirRaw.Normalize(KINDA_SMALL_NUMBER))
		{
			WielderDirRaw = -DirToAim;
		}
		// 방향 EMA(대상 쪽 SmoothedPullDir과 동일 상수): AimPos 노드 노이즈/모서리 전환/근접 축퇴로
		// raw 방향이 프레임마다 튀면 속도 톱업이 매번 다른 축으로 들어가 벡터가 랜덤워크로 불어난다(폭주).
		if (PullDrive.SmoothedWielderPullDir.IsNearlyZero())
		{
			PullDrive.SmoothedWielderPullDir = WielderDirRaw;
		}
		else
		{
			const float Tau = WrapConfig.PullDirSmoothTime;
			const float Alpha = (Tau > KINDA_SMALL_NUMBER) ? (1.0f - FMath::Exp(-DeltaTime / Tau)) : 1.0f;
			PullDrive.SmoothedWielderPullDir = FMath::Lerp(PullDrive.SmoothedWielderPullDir, WielderDirRaw, Alpha).GetSafeNormal();
			if (PullDrive.SmoothedWielderPullDir.IsNearlyZero())
			{
				// 정반대 방향 상쇄 축퇴(180° 반전 순간) — raw로 재시드.
				PullDrive.SmoothedWielderPullDir = WielderDirRaw;
			}
		}
		const FVector WielderDir = PullDrive.SmoothedWielderPullDir;

		AActor* RopeOwner = GetOwner();
		bool bHandled = false;
		if (UPrimitiveComponent* Root = RopeOwner ? Cast<UPrimitiveComponent>(RopeOwner->GetRootComponent()) : nullptr)
		{
			if (Root->IsSimulatingPhysics())
			{
				TopUpVelocity(Root, NAME_None, WielderDir, WielderStep * InvDt);
				bHandled = true;
			}
		}
		if (!bHandled && RopeOwner)
		{
			ApplyNonSimCorrection(RopeOwner, WielderDir, WielderStep);
		}
	}
}

USkeletalMeshComponent* URopeComponent::GetWrappedMesh() const
{
	// State.Mesh는 USceneComponent(정적 랩 대비 일반화). "스켈레탈 메시" 반환 계약 유지 —
	// 정적 대상이면 Cast 실패로 null(대상 액터 반응은 호출자가 GetOwner로 이어감).
	return const_cast<USkeletalMeshComponent*>(Cast<USkeletalMeshComponent>(WrapController.State.Mesh.Get()));
}

void URopeComponent::ApplyPullForce(const FVector& Force, const FRopePullSample& Pull)
{
	// wrap 대상 컴포넌트(cross-actor 가능). 계약상 로프는 대상을 읽기만 하므로 weak가 const지만,
	// Pull은 의도된 게임플레이 개입(힘 인가)이라 여기서만 명시적으로 non-const로 푼다.
	// State.Mesh는 이제 USceneComponent(정적 랩 대비 일반화) — 대상 타입을 가리지 않고 수신자 체인으로
	// 힘을 인가한다(가벼운 물리 프랍/정적 대상도 스켈레탈과 동일 로직). null은 대상 소실(파괴)일 때뿐.
	USceneComponent* MeshComp = const_cast<USceneComponent*>(WrapController.State.Mesh.Get());
	if (!MeshComp)
	{
		return;
	}
	AActor* Owner = MeshComp->GetOwner();

	// 1) 스켈레탈: 감긴 본(부모 체인 승격 포함)이 물리 시뮬 중(래그돌/물리 프랍)이면 그 바디에 직접 —
	//    가장 정확한 인가점. 감긴 본 자체에 바디가 없으면(트위스트 본 등) 가장 가까운 시뮬 부모 바디로.
	if (USkeletalMeshComponent* Skel = Cast<USkeletalMeshComponent>(MeshComp))
	{
		const FName SimBone = FindNearestSimulatingBone(Skel, Pull.Bone);
		if (!SimBone.IsNone())
		{
			Skel->AddForceAtLocation(Force, Pull.WorldPoint, SimBone);
			// 부분 랙돌(메시 루트 바디는 키네마틱): 시뮬 본에 준 힘은 키네마틱 부모 구속(무한질량)이 흡수해
			// 액터로 전달되지 않는다. 캐릭터가 여전히 무브먼트로 구동 중이면 이동체에도 같은 힘을 줘 실제로
			// 끌리게 한다(본 인가는 팔다리가 당겨지는 시각 반응, 무브먼트 인가는 몸통 견인 — 역할이 다르다).
			// 풀 랙돌은 루트 바디가 시뮬이라 여기로 들어오지 않는다(이중 인가 없음).
			if (!Skel->IsSimulatingPhysics())
			{
				if (UCharacterMovementComponent* Movement = GetForceConsumingMovement(Owner))
				{
					Movement->AddForce(Force);
				}
			}
			return;
		}
	}

	// 2) 캐릭터면 무브먼트에 힘 — 애니메이션 구동 본에는 힘을 줄 수 없으므로 이동체 전체를 견인한다
	//    (PoC 4.2: 본/루트에 단순 힘 전달까지. 팔다리 IK/래그돌 반응은 후속).
	//    무브먼트가 힘을 실제로 소비할 때만(MOVE_None 제외) — 아니면 3)/무수신 경고로 떨어져 원인이 보인다.
	if (UCharacterMovementComponent* Movement = GetForceConsumingMovement(Owner))
	{
		Movement->AddForce(Force);
		return;
	}

	// 3) wrap 대상 컴포넌트 자체가 시뮬 중인 프리미티브(가벼운 물리 프랍 등)면 그 바디에 직접.
	if (UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(MeshComp))
	{
		if (Prim->IsSimulatingPhysics())
		{
			Prim->AddForceAtLocation(Force, Pull.WorldPoint);
			return;
		}
	}

	// 4) 그 외: 시뮬 중인 루트 프리미티브(물리 액터에 붙은 컴포넌트 구성).
	if (UPrimitiveComponent* Root = Owner ? Cast<UPrimitiveComponent>(Owner->GetRootComponent()) : nullptr)
	{
		if (Root->IsSimulatingPhysics())
		{
			Root->AddForceAtLocation(Force, Pull.WorldPoint);
			return;
		}
	}

	// 수신자 없음(시뮬 바디 없는 본 체인/비시뮬 컴포넌트 + 무브먼트 비활성/비캐릭터 + 비시뮬 루트): 힘이
	// 조용히 사라지는 걸 wrap당 1회 알린다.
	if (!PullDrive.bLoggedPullNoReceiver)
	{
		PullDrive.bLoggedPullNoReceiver = true;
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] Pull has no force receiver: target=%s bone=%s has no simulating body up its parent chain, owner=%s has no force-consuming CharacterMovement (not a Character, or movement disabled) and its root is not simulating — pull force is dropped."),
			*GetName(), *MeshComp->GetName(), *Pull.Bone.ToString(), *GetNameSafe(Owner));
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
		const USceneComponent* Mesh = Anchor.Mesh.Get();
		if (Mesh && !Anchor.Bone.IsNone())
		{
			const FTransform BoneXform = ResolveBindingWorld(Mesh, Anchor.Bone);
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
				if (const USceneComponent* Mesh = WrapController.State.Mesh.Get())
				{
					const FName Bone = Latch.Bone.IsNone() ? WrapController.State.BoneName : Latch.Bone;
					if (!Bone.IsNone())
					{
						AnchorWorld = ResolveBindingWorld(Mesh, Bone).TransformPosition(Latch.BoneLocalPos);
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
