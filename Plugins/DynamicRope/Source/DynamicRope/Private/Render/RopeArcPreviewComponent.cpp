// Copyright Epic Games, Inc. All Rights Reserved.

#include "Render/RopeArcPreviewComponent.h"

#include "DynamicMeshBuilder.h"
#include "Materials/Material.h"
#include "Materials/MaterialRenderProxy.h"
#include "PrimitiveSceneProxy.h"
#include "PrimitiveViewRelevance.h"
#include "SceneManagement.h"
#include "SceneView.h"

namespace
{
constexpr int32 RopeArcPreviewMaterialCount = 5;

int32 ToMaterialIndex(ERopeArcPreviewMaterialSlot Slot)
{
	return static_cast<int32>(Slot);
}

FVector SafeDir(const FVector& Value, const FVector& Fallback)
{
	const FVector Normalized = Value.GetSafeNormal();
	return Normalized.IsNearlyZero() ? Fallback.GetSafeNormal() : Normalized;
}

FVector DirectionAtAlpha(const FRopeArcPreviewData& Preview, float Alpha)
{
	const FVector Aim = SafeDir(Preview.AimDir, FVector::ForwardVector);
	FVector Up = Preview.GuideUp - FVector::DotProduct(Preview.GuideUp, Aim) * Aim;
	Up = SafeDir(Up, FVector::UpVector);

	const float ClampedAlpha = FMath::Clamp(Alpha, 0.0f, 1.0f);
	const float SweepRadians = FMath::DegreesToRadians(FMath::Clamp(Preview.SweepAngleDegrees, 1.0f, 180.0f));
	const float Angle = SweepRadians * (1.0f - ClampedAlpha);
	return (Aim * FMath::Cos(Angle) + Up * FMath::Sin(Angle)).GetSafeNormal();
}

FVector PlaneNormal(const FRopeArcPreviewData& Preview)
{
	const FVector Aim = SafeDir(Preview.AimDir, FVector::ForwardVector);
	FVector Up = Preview.GuideUp - FVector::DotProduct(Preview.GuideUp, Aim) * Aim;
	Up = SafeDir(Up, FVector::UpVector);
	FVector Normal = FVector::CrossProduct(Aim, Up).GetSafeNormal();
	if (Normal.IsNearlyZero())
	{
		Normal = FVector::UpVector;
	}
	return Normal;
}

void BuildAlphaRange(float StartAlpha, float EndAlpha, int32 SegmentCount, TArray<float>& OutAlphas)
{
	OutAlphas.Reset();
	const float Start = FMath::Clamp(StartAlpha, 0.0f, 1.0f);
	const float End = FMath::Clamp(EndAlpha, 0.0f, 1.0f);
	if (End - Start <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	const int32 Segments = FMath::Clamp(SegmentCount, 1, 128);
	OutAlphas.Add(Start);
	for (int32 SegmentIndex = 1; SegmentIndex < Segments; ++SegmentIndex)
	{
		const float Alpha = static_cast<float>(SegmentIndex) / static_cast<float>(Segments);
		if (Alpha > Start + KINDA_SMALL_NUMBER && Alpha < End - KINDA_SMALL_NUMBER)
		{
			OutAlphas.Add(Alpha);
		}
	}
	OutAlphas.Add(End);
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

void DrawFillRange(const FSceneView* View, const FRopeArcPreviewData& Preview,
	float StartAlpha, float EndAlpha, UMaterialInterface* Material,
	const FMatrix& LocalToWorld, int32 ViewIndex, FMeshElementCollector& Collector)
{
	if (!Material || Preview.Radius <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	TArray<float> Alphas;
	BuildAlphaRange(StartAlpha, EndAlpha, Preview.SegmentCount, Alphas);
	if (Alphas.Num() < 2)
	{
		return;
	}

	TArray<FDynamicMeshVertex> Vertices;
	Vertices.Reserve(Alphas.Num() + 1);
	Vertices.Add(FDynamicMeshVertex(FVector3f(Preview.Origin)));
	for (const float Alpha : Alphas)
	{
		Vertices.Add(FDynamicMeshVertex(FVector3f(Preview.Origin + DirectionAtAlpha(Preview, Alpha) * Preview.Radius)));
	}

	TArray<uint32> Indices;
	Indices.Reserve((Alphas.Num() - 1) * 3);
	for (int32 PointIndex = 1; PointIndex < Vertices.Num() - 1; ++PointIndex)
	{
		Indices.Add(0);
		Indices.Add(PointIndex);
		Indices.Add(PointIndex + 1);
	}

	FDynamicMeshBuilder MeshBuilder(View->GetFeatureLevel());
	MeshBuilder.AddVertices(Vertices);
	MeshBuilder.AddTriangles(Indices);
	DrawMesh(MeshBuilder, LocalToWorld, Material->GetRenderProxy(), ViewIndex, Collector);
}

void DrawRimRange(const FSceneView* View, const FRopeArcPreviewData& Preview,
	float StartAlpha, float EndAlpha, float RimThickness, float RimPlaneOffset, UMaterialInterface* Material,
	const FMatrix& LocalToWorld, int32 ViewIndex, FMeshElementCollector& Collector)
{
	if (!Material || Preview.Radius <= KINDA_SMALL_NUMBER || RimThickness <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	TArray<float> Alphas;
	BuildAlphaRange(StartAlpha, EndAlpha, Preview.SegmentCount, Alphas);
	if (Alphas.Num() < 2)
	{
		return;
	}

	const float HalfThickness = RimThickness * 0.5f;
	const float InnerRadius = FMath::Max(Preview.Radius - HalfThickness, 0.0f);
	const float OuterRadius = Preview.Radius + HalfThickness;
	const FVector Offset = PlaneNormal(Preview) * RimPlaneOffset;

	TArray<FDynamicMeshVertex> Vertices;
	Vertices.Reserve(Alphas.Num() * 2);
	for (const float Alpha : Alphas)
	{
		const FVector Dir = DirectionAtAlpha(Preview, Alpha);
		Vertices.Add(FDynamicMeshVertex(FVector3f(Preview.Origin + Dir * InnerRadius + Offset)));
		Vertices.Add(FDynamicMeshVertex(FVector3f(Preview.Origin + Dir * OuterRadius + Offset)));
	}

	TArray<uint32> Indices;
	Indices.Reserve((Alphas.Num() - 1) * 6);
	for (int32 PointIndex = 0; PointIndex < Alphas.Num() - 1; ++PointIndex)
	{
		const uint32 A = static_cast<uint32>(PointIndex * 2);
		const uint32 B = A + 1;
		const uint32 C = A + 2;
		const uint32 D = A + 3;
		Indices.Add(A); Indices.Add(C); Indices.Add(B);
		Indices.Add(B); Indices.Add(C); Indices.Add(D);
	}

	FDynamicMeshBuilder MeshBuilder(View->GetFeatureLevel());
	MeshBuilder.AddVertices(Vertices);
	MeshBuilder.AddTriangles(Indices);
	DrawMesh(MeshBuilder, LocalToWorld, Material->GetRenderProxy(), ViewIndex, Collector);
}

void DrawHitPoint(const FSceneView* View, const FVector& Center, float Radius, UMaterialInterface* Material,
	const FMatrix& LocalToWorld, int32 ViewIndex, FMeshElementCollector& Collector)
{
	if (!Material || Radius <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	TArray<FDynamicMeshVertex> Vertices;
	Vertices.Reserve(6);
	Vertices.Add(FDynamicMeshVertex(FVector3f(Center + FVector(Radius, 0.0f, 0.0f))));
	Vertices.Add(FDynamicMeshVertex(FVector3f(Center - FVector(Radius, 0.0f, 0.0f))));
	Vertices.Add(FDynamicMeshVertex(FVector3f(Center + FVector(0.0f, Radius, 0.0f))));
	Vertices.Add(FDynamicMeshVertex(FVector3f(Center - FVector(0.0f, Radius, 0.0f))));
	Vertices.Add(FDynamicMeshVertex(FVector3f(Center + FVector(0.0f, 0.0f, Radius))));
	Vertices.Add(FDynamicMeshVertex(FVector3f(Center - FVector(0.0f, 0.0f, Radius))));

	TArray<uint32> Indices;
	Indices.Append({ 4, 0, 2, 4, 2, 1, 4, 1, 3, 4, 3, 0 });
	Indices.Append({ 5, 2, 0, 5, 1, 2, 5, 3, 1, 5, 0, 3 });

	FDynamicMeshBuilder MeshBuilder(View->GetFeatureLevel());
	MeshBuilder.AddVertices(Vertices);
	MeshBuilder.AddTriangles(Indices);
	DrawMesh(MeshBuilder, LocalToWorld, Material->GetRenderProxy(), ViewIndex, Collector);
}
}

struct FRopeArcPreviewDynamicData
{
	FRopeArcPreviewData Preview;
	bool bVisible = false;
	float RimThickness = 3.0f;
	float HitPointRadius = 6.0f;
	float RimPlaneOffset = 0.25f;
};

class FRopeArcPreviewSceneProxy final : public FPrimitiveSceneProxy
{
public:
	explicit FRopeArcPreviewSceneProxy(const URopeArcPreviewComponent* Component)
		: FPrimitiveSceneProxy(Component)
		, MaterialRelevance(Component->GetMaterialRelevance(GetScene().GetShaderPlatform()))
		, ArcFillMaterial(Component->GetMaterial(ToMaterialIndex(ERopeArcPreviewMaterialSlot::ArcFill)))
		, ArcRimMaterial(Component->GetMaterial(ToMaterialIndex(ERopeArcPreviewMaterialSlot::ArcRim)))
		, BlockedArcFillMaterial(Component->GetMaterial(ToMaterialIndex(ERopeArcPreviewMaterialSlot::BlockedArcFill)))
		, BlockedArcRimMaterial(Component->GetMaterial(ToMaterialIndex(ERopeArcPreviewMaterialSlot::BlockedArcRim)))
		, HitPointMaterial(Component->GetMaterial(ToMaterialIndex(ERopeArcPreviewMaterialSlot::HitPoint)))
	{
		if (!ArcFillMaterial)
		{
			ArcFillMaterial = UMaterial::GetDefaultMaterial(MD_Surface);
		}
		if (!ArcRimMaterial)
		{
			ArcRimMaterial = ArcFillMaterial;
		}
		if (!BlockedArcFillMaterial)
		{
			BlockedArcFillMaterial = ArcFillMaterial;
		}
		if (!BlockedArcRimMaterial)
		{
			BlockedArcRimMaterial = BlockedArcFillMaterial;
		}
		if (!HitPointMaterial)
		{
			HitPointMaterial = BlockedArcRimMaterial;
		}
	}

	SIZE_T GetTypeHash() const override
	{
		static size_t UniquePointer;
		return reinterpret_cast<size_t>(&UniquePointer);
	}

	void SetDynamicData_RenderThread(FRopeArcPreviewDynamicData* NewData)
	{
		check(IsInRenderingThread());
		if (!NewData)
		{
			return;
		}

		Preview = NewData->Preview;
		bVisible = NewData->bVisible;
		RimThickness = NewData->RimThickness;
		HitPointRadius = NewData->HitPointRadius;
		RimPlaneOffset = NewData->RimPlaneOffset;
		delete NewData;
	}

	virtual void GetDynamicMeshElements(const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily,
		uint32 VisibilityMap, FMeshElementCollector& Collector) const override
	{
		if (!bVisible || Preview.Radius <= KINDA_SMALL_NUMBER)
		{
			return;
		}

		const float SplitAlpha = Preview.bBlocked ? FMath::Clamp(Preview.BlockedStartAlpha, 0.0f, 1.0f) : 1.0f;
		for (int32 ViewIndex = 0; ViewIndex < Views.Num(); ++ViewIndex)
		{
			if (!(VisibilityMap & (1 << ViewIndex)))
			{
				continue;
			}

			const FSceneView* View = Views[ViewIndex];
			const FMatrix& PreviewLocalToWorld = GetLocalToWorld();

			DrawFillRange(View, Preview, 0.0f, SplitAlpha, ArcFillMaterial, PreviewLocalToWorld, ViewIndex, Collector);
			DrawRimRange(View, Preview, 0.0f, SplitAlpha, RimThickness, RimPlaneOffset, ArcRimMaterial, PreviewLocalToWorld, ViewIndex, Collector);

			if (Preview.bBlocked && SplitAlpha < 1.0f - KINDA_SMALL_NUMBER)
			{
				DrawFillRange(View, Preview, SplitAlpha, 1.0f, BlockedArcFillMaterial, PreviewLocalToWorld, ViewIndex, Collector);
				DrawRimRange(View, Preview, SplitAlpha, 1.0f, RimThickness, RimPlaneOffset, BlockedArcRimMaterial, PreviewLocalToWorld, ViewIndex, Collector);
				DrawHitPoint(View, Preview.HitPoint, HitPointRadius, HitPointMaterial, PreviewLocalToWorld, ViewIndex, Collector);
			}
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
		return FPrimitiveSceneProxy::GetAllocatedSize();
	}

private:
	FMaterialRelevance MaterialRelevance;
	UMaterialInterface* ArcFillMaterial = nullptr;
	UMaterialInterface* ArcRimMaterial = nullptr;
	UMaterialInterface* BlockedArcFillMaterial = nullptr;
	UMaterialInterface* BlockedArcRimMaterial = nullptr;
	UMaterialInterface* HitPointMaterial = nullptr;

	FRopeArcPreviewData Preview;
	bool bVisible = false;
	float RimThickness = 3.0f;
	float HitPointRadius = 6.0f;
	float RimPlaneOffset = 0.25f;
};

URopeArcPreviewComponent::URopeArcPreviewComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
	Mobility = EComponentMobility::Movable;
	SetCastShadow(false);
	LocalPreviewBounds = FBoxSphereBounds(FVector::ZeroVector, FVector(1.0f), 1.0f);
}

void URopeArcPreviewComponent::SetArcPreviewWorld(const FRopeArcPreviewData& InPreview)
{
	const FTransform Xform = GetComponentTransform();
	PreviewLocal = InPreview;
	PreviewLocal.Origin = Xform.InverseTransformPosition(InPreview.Origin);
	PreviewLocal.AimDir = Xform.InverseTransformVectorNoScale(InPreview.AimDir).GetSafeNormal();
	PreviewLocal.GuideUp = Xform.InverseTransformVectorNoScale(InPreview.GuideUp).GetSafeNormal();
	PreviewLocal.HitPoint = Xform.InverseTransformPosition(InPreview.HitPoint);
	PreviewLocal.SegmentCount = FMath::Clamp(InPreview.SegmentCount, 1, 128);
	PreviewLocal.Radius = FMath::Max(InPreview.Radius, 0.0f);
	PreviewLocal.SweepAngleDegrees = FMath::Clamp(InPreview.SweepAngleDegrees, 1.0f, 180.0f);
	PreviewLocal.BlockedStartAlpha = FMath::Clamp(InPreview.BlockedStartAlpha, 0.0f, 1.0f);
	bPreviewVisible = PreviewLocal.Radius > KINDA_SMALL_NUMBER;

	RebuildLocalBounds();
	UpdateBounds();
	MarkRenderTransformDirty();
	MarkRenderDynamicDataDirty();
}

void URopeArcPreviewComponent::ClearArcPreview()
{
	if (!bPreviewVisible && PreviewLocal.Radius <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	bPreviewVisible = false;
	PreviewLocal = FRopeArcPreviewData();
	RebuildLocalBounds();
	UpdateBounds();
	MarkRenderDynamicDataDirty();
}

FPrimitiveSceneProxy* URopeArcPreviewComponent::CreateSceneProxy()
{
	return new FRopeArcPreviewSceneProxy(this);
}

void URopeArcPreviewComponent::SendRenderDynamicData_Concurrent()
{
	Super::SendRenderDynamicData_Concurrent();

	if (!SceneProxy)
	{
		return;
	}

	FRopeArcPreviewDynamicData* DynamicData = new FRopeArcPreviewDynamicData;
	DynamicData->Preview = PreviewLocal;
	DynamicData->bVisible = bPreviewVisible;
	DynamicData->RimThickness = RimThickness;
	DynamicData->HitPointRadius = HitPointRadius;
	DynamicData->RimPlaneOffset = RimPlaneOffset;

	FRopeArcPreviewSceneProxy* Proxy = static_cast<FRopeArcPreviewSceneProxy*>(SceneProxy);
	ENQUEUE_RENDER_COMMAND(RopeArcPreviewUpdate)(
		[Proxy, DynamicData](FRHICommandListBase& RHICmdList)
		{
			Proxy->SetDynamicData_RenderThread(DynamicData);
		});
}

FBoxSphereBounds URopeArcPreviewComponent::CalcBounds(const FTransform& LocalToWorld) const
{
	return LocalPreviewBounds.TransformBy(LocalToWorld);
}

int32 URopeArcPreviewComponent::GetNumMaterials() const
{
	return RopeArcPreviewMaterialCount;
}

UMaterialInterface* URopeArcPreviewComponent::GetMaterial(int32 ElementIndex) const
{
	switch (ElementIndex)
	{
	case 0:
		return ArcFillMaterial;
	case 1:
		return ArcRimMaterial;
	case 2:
		return BlockedArcFillMaterial;
	case 3:
		return BlockedArcRimMaterial;
	case 4:
		return HitPointMaterial;
	default:
		return nullptr;
	}
}

void URopeArcPreviewComponent::SetMaterial(int32 ElementIndex, UMaterialInterface* Material)
{
	switch (ElementIndex)
	{
	case 0:
		ArcFillMaterial = Material;
		break;
	case 1:
		ArcRimMaterial = Material;
		break;
	case 2:
		BlockedArcFillMaterial = Material;
		break;
	case 3:
		BlockedArcRimMaterial = Material;
		break;
	case 4:
		HitPointMaterial = Material;
		break;
	default:
		return;
	}
	MarkRenderStateDirty();
}

void URopeArcPreviewComponent::RebuildLocalBounds()
{
	if (!bPreviewVisible || PreviewLocal.Radius <= KINDA_SMALL_NUMBER)
	{
		LocalPreviewBounds = FBoxSphereBounds(FVector::ZeroVector, FVector(1.0f), 1.0f);
		return;
	}

	FBox LocalBoundsBox(ForceInit);
	LocalBoundsBox += PreviewLocal.Origin;
	const int32 Segments = FMath::Clamp(PreviewLocal.SegmentCount, 1, 128);
	for (int32 SegmentIndex = 0; SegmentIndex <= Segments; ++SegmentIndex)
	{
		const float Alpha = static_cast<float>(SegmentIndex) / static_cast<float>(Segments);
		LocalBoundsBox += PreviewLocal.Origin + DirectionAtAlpha(PreviewLocal, Alpha) * (PreviewLocal.Radius + RimThickness);
	}
	if (PreviewLocal.bBlocked)
	{
		LocalBoundsBox += PreviewLocal.HitPoint + FVector(HitPointRadius);
		LocalBoundsBox += PreviewLocal.HitPoint - FVector(HitPointRadius);
	}
	LocalPreviewBounds = FBoxSphereBounds(LocalBoundsBox);
}
