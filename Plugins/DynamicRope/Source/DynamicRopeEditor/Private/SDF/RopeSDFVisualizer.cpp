// Copyright Epic Games, Inc. All Rights Reserved.

#include "SDF/RopeSDFVisualizer.h"
#include "SDF/RopeSDFDraw.h"
#include "Collision/SDF/RopeSDFProvider.h"
#include "Collision/SDF/RopeSDFData.h"
#include "Collision/SDF/RopeSDFSynthetic.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "SceneManagement.h"

namespace
{
	USkeletalMeshComponent* ResolveMesh(const URopeSDFProvider* Provider)
	{
		if (Provider->SkeletalMesh)
		{
			return Provider->SkeletalMesh;
		}
		if (const AActor* Owner = Provider->GetOwner())
		{
			return Owner->FindComponentByClass<USkeletalMeshComponent>();
		}
		return nullptr;
	}
}

void FRopeSDFVisualizer::DrawVisualization(const UActorComponent* Component, const FSceneView* View,
	FPrimitiveDrawInterface* PDI)
{
	const URopeSDFProvider* Provider = Cast<URopeSDFProvider>(Component);
	if (!Provider || !PDI)
	{
		return;
	}
	if (!Provider->bDrawSDFBounds && !Provider->bDrawSDFGrid && !Provider->bDrawSDFVoxels
		&& !Provider->bDrawSDFSlice && !Provider->bDrawSDFGradient)
	{
		return;
	}

	USkeletalMeshComponent* Mesh = ResolveMesh(Provider);
	if (!Mesh || !Provider->SDFData)
	{
		return;
	}

	for (const FRopeBoneSDFVolume& Vol : Provider->SDFData->BoneVolumes)
	{
		if (Vol.Bone.IsNone())
		{
			continue;
		}
		const FTransform Xform = Mesh->GetSocketTransform(Vol.Bone);

		// Synthetic preview substitutes an analytic sphere so the viz works before B3 bake exists.
		const FRopeBoneSDFVolume* DrawVol = &Vol;
		FRopeBoneSDFVolume Synthetic;
		if (Provider->bSDFSyntheticPreview)
		{
			Synthetic = RopeSDFSynthetic::MakeSphere(Vol.Bone, FVector::ZeroVector,
				Provider->SDFSyntheticRadius, FIntVector(17), 6.0f);
			DrawVol = &Synthetic;
		}

		if (Provider->bDrawSDFBounds)
		{
			RopeSDFDraw::DrawBounds(PDI, DrawVol->LocalBounds, Xform, FLinearColor(1.0f, 0.6f, 0.0f));
		}
		if (Provider->bDrawSDFGrid)
		{
			RopeSDFDraw::DrawCoarseGrid(PDI, DrawVol->LocalBounds, Xform, FLinearColor(0.3f, 0.3f, 0.3f), 4);
		}
		if (Provider->bDrawSDFVoxels && DrawVol->IsBaked())
		{
			RopeSDFDraw::DrawVoxels(PDI, *DrawVol, Xform, Provider->SDFBandThreshold);
		}
		if (Provider->bDrawSDFSlice && DrawVol->IsBaked())
		{
			RopeSDFDraw::DrawSlice(PDI, *DrawVol, Xform, Provider->SDFSliceAxis, Provider->SDFSlicePosition,
				Provider->SDFSliceResolution, Provider->SDFSliceColorScale);
		}
		if (Provider->bDrawSDFGradient && DrawVol->IsBaked())
		{
			RopeSDFDraw::DrawGradients(PDI, *DrawVol, Xform, Provider->SDFBandThreshold, Provider->SDFGradientLength);
		}
	}
}
