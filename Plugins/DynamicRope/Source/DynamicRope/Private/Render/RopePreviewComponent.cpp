// Copyright Epic Games, Inc. All Rights Reserved.

#include "Render/RopePreviewComponent.h"

#include "DynamicMeshBuilder.h"
#include "Materials/Material.h"
#include "Materials/MaterialRenderProxy.h"
#include "PrimitiveSceneProxy.h"
#include "PrimitiveViewRelevance.h"
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
	PrimaryComponentTick.bCanEverTick = false;
	Mobility = EComponentMobility::Movable;
	SetCastShadow(false);
	LocalPreviewBounds = FBoxSphereBounds(FVector::ZeroVector, FVector(1.0f), 1.0f);
}

void URopePreviewComponent::SetArcPreviewWorld(const FRopeArcPreviewData& InPreview)
{
	// Legacy throw arc preview is intentionally disabled; this component now renders wrap previews only.
	ClearPreview();
}

void URopePreviewComponent::SetWrapPreviewWorld(const FRopeWrapPreviewData& InPreview)
{
	const FTransform Xform = GetComponentTransform();
	WrapPreviewLocal = InPreview;
	WrapPreviewLocal.Points.Reset(InPreview.Points.Num());
	for (const FVector& Point : InPreview.Points)
	{
		WrapPreviewLocal.Points.Add(Xform.InverseTransformPosition(Point));
	}
	WrapPreviewLocal.Radius = FMath::Max(InPreview.Radius, WrapPreviewRadius);
	WrapPreviewLocal.NumSides = FMath::Clamp(InPreview.NumSides > 0 ? InPreview.NumSides : WrapPreviewSides, 3, 32);
	bPreviewVisible = WrapPreviewLocal.IsValid();

	RebuildLocalBounds();
	UpdateBounds();
	MarkRenderTransformDirty();
	MarkRenderDynamicDataDirty();
}

void URopePreviewComponent::ClearPreview()
{
	if (!bPreviewVisible && !WrapPreviewLocal.IsValid())
	{
		return;
	}

	bPreviewVisible = false;
	PreviewLocal = FRopeArcPreviewData();
	WrapPreviewLocal = FRopeWrapPreviewData();
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

URopeArcPreviewComponent::URopeArcPreviewComponent()
	: Super()
{
}
