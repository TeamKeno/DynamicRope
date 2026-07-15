// Copyright Epic Games, Inc. All Rights Reserved.

#include "Render/RopePreviewComponent.h"

#include "Collision/RopeCollider.h"
#include "DynamicMeshBuilder.h"
#include "Materials/Material.h"
#include "Materials/MaterialRenderProxy.h"
#include "PrimitiveSceneProxy.h"
#include "PrimitiveViewRelevance.h"
#include "RopeMathHelpers.h"
#include "SceneManagement.h"
#include "SceneView.h"

namespace
{
constexpr int32 RopePreviewMaterialCount = 1;

int32 ToMaterialIndex(ERopePreviewMaterialSlot Slot)
{
	return static_cast<int32>(Slot);
}

FVector AnyNormalForTangent(const FVector& Tangent)
{
	const FVector Axis = FMath::Abs(FVector::DotProduct(Tangent, FVector::UpVector)) < 0.85f
		? FVector::UpVector
		: FVector::RightVector;
	return FVector::CrossProduct(Axis, Tangent).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
}

void SetFailureReason(FString* OutFailureReason, const TCHAR* Reason)
{
	if (OutFailureReason)
	{
		*OutFailureReason = Reason;
	}
}

float SmoothStep01(float Value)
{
	const float T = FMath::Clamp(Value, 0.0f, 1.0f);
	return T * T * (3.0f - 2.0f * T);
}

// 실제 WhipGuide와 같은 속도 보정 규칙으로 preview 애니메이션 길이를 계산한다.
float ResolvePreviewGuideDuration(const FRopeWhipGuide::FConfig& Config, float ThrowSpeed)
{
	const float BaseDuration = FMath::Max(Config.Duration, KINDA_SMALL_NUMBER);
	if (ThrowSpeed <= KINDA_SMALL_NUMBER)
	{
		return BaseDuration;
	}

	const float ReferenceSpeed = FMath::Max(Config.ReferenceThrowSpeed, KINDA_SMALL_NUMBER);
	return FMath::Clamp(BaseDuration * ReferenceSpeed / ThrowSpeed, KINDA_SMALL_NUMBER, 10.0f);
}

// 곡선을 rope segment 간격으로 다시 샘플링해 실제 solver node 수와 preview 점을 맞춘다.
void ResampleBySpacing(const TArray<FVector>& SourcePoints, float NodeSpacing,
	int32 DesiredPointCount, TArray<FVector>& OutPoints)
{
	OutPoints.Reset();
	if (SourcePoints.Num() == 0 || DesiredPointCount <= 0)
	{
		return;
	}

	OutPoints.SetNum(DesiredPointCount);
	OutPoints[0] = SourcePoints[0];
	if (DesiredPointCount == 1)
	{
		return;
	}

	TArray<float> Accumulated;
	Accumulated.SetNum(SourcePoints.Num());
	Accumulated[0] = 0.0f;
	for (int32 i = 1; i < SourcePoints.Num(); ++i)
	{
		Accumulated[i] = Accumulated[i - 1] + FVector::Dist(SourcePoints[i - 1], SourcePoints[i]);
	}

	const float SegmentLength = FMath::Max(NodeSpacing, KINDA_SMALL_NUMBER);
	int32 SourceIndex = 1;
	for (int32 PointIndex = 1; PointIndex < DesiredPointCount; ++PointIndex)
	{
		const float TargetDistance = SegmentLength * static_cast<float>(PointIndex);
		while (SourceIndex < Accumulated.Num() - 1 && Accumulated[SourceIndex] < TargetDistance)
		{
			++SourceIndex;
		}

		const int32 PrevIndex = FMath::Max(SourceIndex - 1, 0);
		const float SegmentDistance = FMath::Max(Accumulated[SourceIndex] - Accumulated[PrevIndex], KINDA_SMALL_NUMBER);
		const float Alpha = FMath::Clamp((TargetDistance - Accumulated[PrevIndex]) / SegmentDistance, 0.0f, 1.0f);
		OutPoints[PointIndex] = FMath::Lerp(SourcePoints[PrevIndex], SourcePoints[SourceIndex], Alpha);
	}
}

void DrawMesh(FDynamicMeshBuilder& MeshBuilder, const FMatrix& LocalToWorld,
	const FMaterialRenderProxy* MaterialProxy, int32 ViewIndex, FMeshElementCollector& Collector)
{
	FDynamicMeshBuilderSettings Settings;
	Settings.bDisableBackfaceCulling = true;
	Settings.bReceivesDecals = false;
	Settings.bUseSelectionOutline = false;
	Settings.CastShadow = false;

	MeshBuilder.GetMesh(LocalToWorld, MaterialProxy, SDPG_World, Settings, nullptr, ViewIndex, Collector);
}

void DrawPreviewTube(const FSceneView* View, const FRopeWrapPreviewData& Preview, UMaterialInterface* Material,
	const FMatrix& LocalToWorld, int32 ViewIndex, FMeshElementCollector& Collector)
{
	if (!Material || !Preview.IsValid())
	{
		return;
	}

	const int32 NumPoints = Preview.Points.Num();
	const int32 NumSides = FMath::Clamp(Preview.NumSides, 3, 32);
	const float Radius = FMath::Max(0.1f, Preview.Radius);
	const FColor PreviewColor(80, 220, 255, 96);

	TArray<FDynamicMeshVertex> Vertices;
	Vertices.Reserve(NumPoints * NumSides);
	TArray<uint32> Indices;
	Indices.Reserve((NumPoints - 1) * NumSides * 6);

	// 매번 ring vertex 생성
	FVector PreviousNormal = FVector::ZeroVector;
	for (int32 PointIndex = 0; PointIndex < NumPoints; ++PointIndex)
	{
		FVector Tangent = FVector::ForwardVector;
		if (PointIndex == 0)
		{
			Tangent = Preview.Points[1] - Preview.Points[0];
		}
		else if (PointIndex == NumPoints - 1)
		{
			Tangent = Preview.Points[PointIndex] - Preview.Points[PointIndex - 1];
		}
		else
		{
			Tangent = Preview.Points[PointIndex + 1] - Preview.Points[PointIndex - 1];
		}
		Tangent = Tangent.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);

		FVector Normal = PreviousNormal.IsNearlyZero()
			? AnyNormalForTangent(Tangent)
			: (PreviousNormal - FVector::DotProduct(PreviousNormal, Tangent) * Tangent)
				.GetSafeNormal(KINDA_SMALL_NUMBER, AnyNormalForTangent(Tangent));
		const FVector Binormal = FVector::CrossProduct(Tangent, Normal).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::RightVector);
		PreviousNormal = Normal;

		for (int32 SideIndex = 0; SideIndex < NumSides; ++SideIndex)
		{
			const float Angle = (2.0f * UE_PI) * static_cast<float>(SideIndex) / static_cast<float>(NumSides);
			const FVector RingOffset = (Normal * FMath::Cos(Angle) + Binormal * FMath::Sin(Angle)) * Radius;
			FDynamicMeshVertex Vertex(FVector3f(Preview.Points[PointIndex] + RingOffset));
			Vertex.Color = PreviewColor;
			Vertices.Add(Vertex);
		}
	}

	// 매번 triangle index 생성
	for (int32 PointIndex = 0; PointIndex < NumPoints - 1; ++PointIndex)
	{
		const uint32 BaseA = static_cast<uint32>(PointIndex * NumSides);
		const uint32 BaseB = static_cast<uint32>((PointIndex + 1) * NumSides);
		for (int32 SideIndex = 0; SideIndex < NumSides; ++SideIndex)
		{
			const uint32 A0 = BaseA + static_cast<uint32>(SideIndex);
			const uint32 A1 = BaseA + static_cast<uint32>((SideIndex + 1) % NumSides);
			const uint32 B0 = BaseB + static_cast<uint32>(SideIndex);
			const uint32 B1 = BaseB + static_cast<uint32>((SideIndex + 1) % NumSides);
			Indices.Add(A0); Indices.Add(B0); Indices.Add(A1);
			Indices.Add(A1); Indices.Add(B0); Indices.Add(B1);
		}
	}

	FDynamicMeshBuilder MeshBuilder(View->GetFeatureLevel());
	MeshBuilder.AddVertices(Vertices);
	MeshBuilder.AddTriangles(Indices);
	DrawMesh(MeshBuilder, LocalToWorld, Material->GetRenderProxy(), ViewIndex, Collector);
}
}

struct FRopePreviewDynamicData
{
	FRopeWrapPreviewData WrapPreview;
	bool bVisible = false;
};

class FRopePreviewSceneProxy final : public FPrimitiveSceneProxy
{
public:
	explicit FRopePreviewSceneProxy(const URopePreviewComponent* Component)
		: FPrimitiveSceneProxy(Component)
		, MaterialRelevance(Component->GetMaterialRelevance(GetScene().GetShaderPlatform()))
		, WrapPreviewMaterial(Component->GetMaterial(ToMaterialIndex(ERopePreviewMaterialSlot::WrapPreview)))
	{
		if (!WrapPreviewMaterial)
		{
			WrapPreviewMaterial = UMaterial::GetDefaultMaterial(MD_Surface);
		}
	}

	SIZE_T GetTypeHash() const override
	{
		static size_t UniquePointer;
		return reinterpret_cast<size_t>(&UniquePointer);
	}

	void SetDynamicData_RenderThread(FRopePreviewDynamicData* NewData)
	{
		check(IsInRenderingThread());
		if (!NewData)
		{
			return;
		}

		WrapPreview = MoveTemp(NewData->WrapPreview);
		bVisible = NewData->bVisible;
		delete NewData;
	}

	virtual void GetDynamicMeshElements(const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily,
		uint32 VisibilityMap, FMeshElementCollector& Collector) const override
	{
		if (!bVisible || !WrapPreview.IsValid())
		{
			return;
		}

		for (int32 ViewIndex = 0; ViewIndex < Views.Num(); ++ViewIndex)
		{
			if (!(VisibilityMap & (1 << ViewIndex)))
			{
				continue;
			}

			DrawPreviewTube(Views[ViewIndex], WrapPreview, WrapPreviewMaterial,
				GetLocalToWorld(), ViewIndex, Collector);
		}
	}

	virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView* View) const override
	{
		FPrimitiveViewRelevance Result;
		Result.bDrawRelevance = IsShown(View);
		Result.bDynamicRelevance = true;
		Result.bShadowRelevance = false;
		Result.bRenderInMainPass = ShouldRenderInMainPass();
		Result.bRenderCustomDepth = ShouldRenderCustomDepth();
		Result.bUsesLightingChannels = GetLightingChannelMask() != GetDefaultLightingChannelMask();
		MaterialRelevance.SetPrimitiveViewRelevance(Result);
		Result.bVelocityRelevance = false;
		return Result;
	}

	virtual uint32 GetMemoryFootprint() const override
	{
		return sizeof(*this) + GetAllocatedSize();
	}

	uint32 GetAllocatedSize() const
	{
		return FPrimitiveSceneProxy::GetAllocatedSize() + WrapPreview.Points.GetAllocatedSize();
	}

private:
	FMaterialRelevance MaterialRelevance;
	UMaterialInterface* WrapPreviewMaterial = nullptr;

	FRopeWrapPreviewData WrapPreview;
	bool bVisible = false;
};

URopePreviewComponent::URopePreviewComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = false;
	Mobility = EComponentMobility::Movable;
	SetCastShadow(false);
	LocalPreviewBounds = FBoxSphereBounds(FVector::ZeroVector, FVector(1.0f), 1.0f);
}

bool URopePreviewComponent::ShowWhipGuideAnimation(const URopeComponent& Rope, const FRopeThrowContext& ThrowContext,
	FString* OutFailureReason)
{
	// 표시 전용: 화면용 whip 프레임만 만든다. 게임플레이 데이터(prepared contact/anchor)는 호출자(Wielder)가
	// 로프에서 직접 빌드해 소유하므로, 여기서 실패해도 던지기에는 영향이 없다.
	FRopePreviewBuildContext Source;
	if (!Rope.BuildPreviewContext(ThrowContext, Source))
	{
		SetFailureReason(OutFailureReason, TEXT("whip guide preview rejected: no valid rope preview context"));
		return false;
	}

	RebuildWhipPreviewFrames(Source);
	if (WhipPreviewFramesLocal.Num() == 0)
	{
		SetFailureReason(OutFailureReason, TEXT("whip guide preview rejected: no generated frames"));
		return false;
	}

	WhipPreviewFrameIndex = FMath::Clamp(WhipPreviewFrameIndex, 0, WhipPreviewFramesLocal.Num() - 1);
	SetWrapPreviewLocal(WhipPreviewFramesLocal[WhipPreviewFrameIndex]);
	SetComponentTickEnabled(true);
	return true;
}

void URopePreviewComponent::RebuildWhipPreviewFrames(const FRopePreviewBuildContext& Source)
{
	const int32 PreviousFrameIndex = WhipPreviewFrameIndex;
	WhipPreviewFramesLocal.Reset();

	const float SweepDegrees = FMath::Clamp(Source.WhipConfig.SweepAngleDegrees, 1.0f, 180.0f);
	const float StepDegrees = FMath::Clamp(WhipPreviewAngleStepDegrees, 1.0f, 45.0f);
	const int32 StepCount = FMath::Clamp(FMath::CeilToInt(SweepDegrees / StepDegrees), 1, 180);
	// 실제 Aim hit Flight와 동일하게 preview도 로프 전체 길이의 spline을 표시한다.
	const float GuidedEnd = Source.ThrowContext.bHasAimGuideHit
		? 1.0f
		: FMath::Clamp(Source.WhipConfig.GuidedLength, 0.05f, 0.95f);
	const int32 LastGuidedNode = FMath::Clamp(
		FMath::CeilToInt(static_cast<float>(Source.NodeCount - 1) * GuidedEnd), 1, Source.NodeCount - 1);
	const int32 DesiredPointCount = FMath::Clamp(LastGuidedNode + 1, 2, Source.NodeCount);
	const int32 RawSampleCount = FMath::Max(DesiredPointCount * 4, 16);
	const float GuideLength = Source.RopeLength * GuidedEnd * FMath::Max(Source.PreviewReachScale, 0.0f);
	const float SweepRadians = FMath::DegreesToRadians(SweepDegrees);
	const float GuideDuration = ResolvePreviewGuideDuration(Source.WhipConfig, Source.ThrowContext.ThrowSpeed);

	if (GuideLength <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	WhipPreviewFramesLocal.Reserve(StepCount + 1);
	for (int32 StepIndex = 0; StepIndex <= StepCount; ++StepIndex)
	{
		const float T = static_cast<float>(StepIndex) / static_cast<float>(StepCount);
		const float AngleFromAim = SweepRadians * (1.0f - T);
		FVector SweepDir = (Source.SwingBasis.AimDir * FMath::Cos(AngleFromAim) +
			Source.SwingBasis.GuideUp * FMath::Sin(AngleFromAim)).GetSafeNormal();
		if (T >= 1.0f - KINDA_SMALL_NUMBER)
		{
			SweepDir = Source.SwingBasis.AimDir;
		}
		if (SweepDir.IsNearlyZero())
		{
			continue;
		}

		const FVector InheritedDrift = Source.InheritedVelocity * (GuideDuration * T);
		const bool bHasAimTarget = Source.ThrowContext.bHasAimGuideHit &&
			!(Source.ThrowContext.AimGuideHitWorldPos - Source.ThrowContext.Origin).IsNearlyZero();
		const FVector LockedAimDir = bHasAimTarget
			? (Source.ThrowContext.AimGuideHitWorldPos - Source.ThrowContext.Origin)
				.GetSafeNormal(KINDA_SMALL_NUMBER, Source.SwingBasis.AimDir)
			: Source.SwingBasis.AimDir;
		const float AimSteerStartAlpha = FMath::Clamp(Source.ThrowContext.AimGuideSteerStartAlpha, 0.0f, 0.9f);
		const float AimLockAlpha = FMath::Clamp(Source.ThrowContext.AimGuideLockAlpha, 0.05f, 1.0f);
		TArray<FVector> RawPoints;
		// 실제 Flight와 같은 공통 곡선 생성기를 써서 preview와 throw의 형상이 완전히 일치하게 한다.
		RopeMath::BuildWhipGuideRawPoints(Source.ThrowContext.Origin, SweepDir, LockedAimDir,
			bHasAimTarget, T, GuideLength, InheritedDrift, AimSteerStartAlpha,
			AimLockAlpha, Source.WhipConfig.AimHitDirectionBias, RawSampleCount, RawPoints);

		FRopeWrapPreviewData FrameWorld;
		ResampleBySpacing(RawPoints, Source.SegmentLength, DesiredPointCount, FrameWorld.Points);
		FrameWorld.Radius = FMath::Max(Source.RopeRadius, WrapPreviewRadius);
		FrameWorld.NumSides = FMath::Clamp(Source.RopeNumSides > 0 ? Source.RopeNumSides : WrapPreviewSides, 3, 32);
		if (FrameWorld.IsValid())
		{
			FRopeWrapPreviewData FrameLocal = ConvertWrapPreviewToLocal(FrameWorld);
			// provider 소유 collider가 유효한 이 호출 안에서 충돌 경계를 확정한다.
			// 재생 tick에는 안전한 local point 배열만 남기고 raw collider 포인터는 넘기지 않는다.
			if (DoesWhipFrameHit(FrameLocal, Source))
			{
				break;
			}
			WhipPreviewFramesLocal.Add(MoveTemp(FrameLocal));
		}
	}

	WhipPreviewFrameIndex = WhipPreviewFramesLocal.Num() > 0
		? FMath::Clamp(PreviousFrameIndex, 0, WhipPreviewFramesLocal.Num() - 1)
		: 0;
}

bool URopePreviewComponent::DoesWhipFrameHit(const FRopeWrapPreviewData& Frame,
	const FRopePreviewBuildContext& Source) const
{
	if (!Source.Colliders || Source.Colliders->Num() == 0 || !Frame.IsValid())
	{
		return false;
	}

	// 프레임은 component-local이므로 collider 질의 직전에 현재 transform으로 월드 좌표를 복원한다.
	const FTransform Xform = GetComponentTransform();
	const float QueryRadius = Source.PreviewQueryRadius > KINDA_SMALL_NUMBER
		? Source.PreviewQueryRadius
		: FMath::Max(Source.RopeRadius, WrapPreviewRadius);

	for (int32 PointIndex = 1; PointIndex < Frame.Points.Num(); ++PointIndex)
	{
		const FVector WorldPoint = Xform.TransformPosition(Frame.Points[PointIndex]);
		for (const IRopeCollider* Collider : *Source.Colliders)
		{
			if (Collider && Collider->Query(WorldPoint, QueryRadius).bHit)
			{
				return true;
			}
		}
	}

	return false;
}

void URopePreviewComponent::AdvanceWhipPreview(float DeltaTime)
{
	if (!bPreviewVisible || WhipPreviewFramesLocal.Num() == 0)
	{
		return;
	}

	WhipPreviewPlaybackTimer += DeltaTime;
	if (WhipPreviewPlaybackTimer < FMath::Max(WhipPreviewSecondsPerStep, 0.01f))
	{
		return;
	}

	WhipPreviewPlaybackTimer = 0.0f;
	int32 NextFrameIndex = WhipPreviewFrameIndex + 1;
	if (NextFrameIndex >= WhipPreviewFramesLocal.Num())
	{
		NextFrameIndex = 0;
	}
	WhipPreviewFrameIndex = NextFrameIndex;
	SetWrapPreviewLocal(WhipPreviewFramesLocal[WhipPreviewFrameIndex]);
}

void URopePreviewComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (PreviewMode == ERopePreviewMode::WhipGuideAnimation)
	{
		AdvanceWhipPreview(DeltaTime);
	}
}

void URopePreviewComponent::SetWrapPreviewWorld(const FRopeWrapPreviewData& InPreview)
{
	// 표시 전용 진입점: 주어진 월드 centerline을 그대로 그린다. whip 애니 재생 중이었다면 정적 표시로 전환한다.
	WhipPreviewFramesLocal.Reset();
	SetComponentTickEnabled(false);
	SetWrapPreviewLocal(ConvertWrapPreviewToLocal(InPreview));
}

FRopeWrapPreviewData URopePreviewComponent::ConvertWrapPreviewToLocal(const FRopeWrapPreviewData& InPreview) const
{
	// scene proxy에는 local 좌표만 넘겨 component transform이 렌더 단계에서 한 번만 적용되게 한다.
	const FTransform Xform = GetComponentTransform();
	FRopeWrapPreviewData LocalPreview = InPreview;
	LocalPreview.Points.Reset(InPreview.Points.Num());
	for (const FVector& Point : InPreview.Points)
	{
		LocalPreview.Points.Add(Xform.InverseTransformPosition(Point));
	}
	LocalPreview.Radius = FMath::Max(InPreview.Radius, WrapPreviewRadius);
	LocalPreview.NumSides = FMath::Clamp(InPreview.NumSides > 0 ? InPreview.NumSides : WrapPreviewSides, 3, 32);
	return LocalPreview;
}

void URopePreviewComponent::SetWrapPreviewLocal(const FRopeWrapPreviewData& InPreview)
{
	WrapPreviewLocal = InPreview;
	bPreviewVisible = WrapPreviewLocal.IsValid();

	RebuildLocalBounds();
	UpdateBounds();
	MarkRenderTransformDirty();
	MarkRenderDynamicDataDirty();
}

void URopePreviewComponent::ClearPreview()
{
	if (!bPreviewVisible && !WrapPreviewLocal.IsValid() && WhipPreviewFramesLocal.Num() == 0)
	{
		return;
	}

	bPreviewVisible = false;
	WrapPreviewLocal = FRopeWrapPreviewData();
	WhipPreviewFramesLocal.Reset();
	WhipPreviewFrameIndex = 0;
	WhipPreviewPlaybackTimer = 0.0f;
	SetComponentTickEnabled(false);
	RebuildLocalBounds();
	UpdateBounds();
	MarkRenderDynamicDataDirty();
}

FPrimitiveSceneProxy* URopePreviewComponent::CreateSceneProxy()
{
	return new FRopePreviewSceneProxy(this);
}

void URopePreviewComponent::SendRenderDynamicData_Concurrent()
{
	Super::SendRenderDynamicData_Concurrent();

	if (!SceneProxy)
	{
		return;
	}

	FRopePreviewDynamicData* DynamicData = new FRopePreviewDynamicData;
	DynamicData->WrapPreview = WrapPreviewLocal;
	DynamicData->bVisible = bPreviewVisible;

	FRopePreviewSceneProxy* Proxy = static_cast<FRopePreviewSceneProxy*>(SceneProxy);
	ENQUEUE_RENDER_COMMAND(RopePreviewUpdate)(
		[Proxy, DynamicData](FRHICommandListBase& RHICmdList)
		{
			Proxy->SetDynamicData_RenderThread(DynamicData);
		});
}

FBoxSphereBounds URopePreviewComponent::CalcBounds(const FTransform& LocalToWorld) const
{
	return LocalPreviewBounds.TransformBy(LocalToWorld);
}

int32 URopePreviewComponent::GetNumMaterials() const
{
	return RopePreviewMaterialCount;
}

UMaterialInterface* URopePreviewComponent::GetMaterial(int32 ElementIndex) const
{
	switch (ElementIndex)
	{
	case 0:
		return WrapPreviewMaterial;
	default:
		return nullptr;
	}
}

void URopePreviewComponent::SetMaterial(int32 ElementIndex, UMaterialInterface* Material)
{
	switch (ElementIndex)
	{
	case 0:
		WrapPreviewMaterial = Material;
		break;
	default:
		return;
	}
	MarkRenderStateDirty();
}

void URopePreviewComponent::RebuildLocalBounds()
{
	if (!bPreviewVisible || !WrapPreviewLocal.IsValid())
	{
		LocalPreviewBounds = FBoxSphereBounds(FVector::ZeroVector, FVector(1.0f), 1.0f);
		return;
	}

	FBox LocalBoundsBox(ForceInit);
	const float BoundsRadius = FMath::Max(WrapPreviewLocal.Radius, WrapPreviewRadius);
	for (const FVector& Point : WrapPreviewLocal.Points)
	{
		LocalBoundsBox += Point + FVector(BoundsRadius);
		LocalBoundsBox += Point - FVector(BoundsRadius);
	}
	LocalPreviewBounds = FBoxSphereBounds(LocalBoundsBox);
}
