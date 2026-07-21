// Copyright Epic Games, Inc. All Rights Reserved.

#include "Core/RopeThrowTypes.h"

#include "Components/SceneComponent.h"

namespace
{
FRopeWrapPreviewData TransformPreviewFromGuideLocal(const FRopeWrapPreviewData& Source,
	const TArray<FVector>& LocalPoints, const USceneComponent* GuideFrame)
{
	if (!GuideFrame || LocalPoints.Num() == 0)
	{
		return Source;
	}

	const FTransform GuideTransform = GuideFrame->GetComponentTransform();
	FRopeWrapPreviewData WorldPreview = Source;
	WorldPreview.Points.Reset(LocalPoints.Num());
	for (const FVector& LocalPoint : LocalPoints)
	{
		WorldPreview.Points.Add(GuideTransform.TransformPosition(LocalPoint));
	}
	return WorldPreview;
}
}

void FRopePreparedThrowPreview::StoreGuideFrameLocal(const USceneComponent* InGuideFrame)
{
	bUseGuideFrameLocal = false;
	GuideFrameComponent = nullptr;
	GuideFrameLocalPoints.Reset();
	GuideFrameLocalOrigin = FVector::ZeroVector;

	if (!InGuideFrame || !RenderPreview.IsValid())
	{
		return;
	}

	// spline 점과 throw origin을 같은 owner frame에 저장해야 애니메이션 소켓 이동과 분리된다.
	const FTransform GuideTransform = InGuideFrame->GetComponentTransform();
	GuideFrameLocalPoints.Reserve(RenderPreview.Points.Num());
	for (const FVector& Point : RenderPreview.Points)
	{
		GuideFrameLocalPoints.Add(GuideTransform.InverseTransformPosition(Point));
	}

	GuideFrameLocalOrigin = GuideTransform.InverseTransformPosition(ThrowContext.Origin);
	GuideFrameComponent = InGuideFrame;
	bUseGuideFrameLocal = GuideFrameLocalPoints.Num() == RenderPreview.Points.Num();
}

bool FRopePreparedThrowPreview::HasGuideFrameLocal() const
{
	return bUseGuideFrameLocal && GuideFrameComponent.IsValid() && GuideFrameLocalPoints.Num() > 0;
}

FVector FRopePreparedThrowPreview::ResolveGuideOriginWorld() const
{
	if (!HasGuideFrameLocal())
	{
		return ThrowContext.Origin;
	}

	return GuideFrameComponent->GetComponentTransform().TransformPosition(GuideFrameLocalOrigin);
}

FVector FRopePreparedThrowPreview::ResolveGuidePointWorld(int32 PointIndex) const
{
	if (!HasGuideFrameLocal())
	{
		if (RenderPreview.Points.IsValidIndex(PointIndex))
		{
			return RenderPreview.Points[PointIndex];
		}
		return RenderPreview.Points.Num() > 0 ? RenderPreview.Points.Last() : ThrowContext.Origin;
	}

	const int32 ClampedIndex = FMath::Clamp(PointIndex, 0, GuideFrameLocalPoints.Num() - 1);
	return GuideFrameComponent->GetComponentTransform().TransformPosition(GuideFrameLocalPoints[ClampedIndex]);
}

FRopeWrapPreviewData FRopePreparedThrowPreview::ResolveRenderPreviewWorld() const
{
	return TransformPreviewFromGuideLocal(
		RenderPreview, GuideFrameLocalPoints, HasGuideFrameLocal() ? GuideFrameComponent.Get() : nullptr);
}
