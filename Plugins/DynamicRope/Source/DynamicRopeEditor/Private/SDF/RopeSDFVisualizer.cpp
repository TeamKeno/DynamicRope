// Copyright Epic Games, Inc. All Rights Reserved.

#include "SDF/RopeSDFVisualizer.h"
#include "Collision/SDF/RopeSDFProvider.h"
#include "Collision/SDF/RopeSDFData.h"
#include "Collision/SDF/RopeSDFSynthetic.h"
#include "Collision/SDF/RopeSDFSampler.h"
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

	// Local-space AABB -> 12 world edges.
	void DrawBoundsBox(FPrimitiveDrawInterface* PDI, const FBox& Local, const FTransform& Xform, const FLinearColor& Color)
	{
		const FVector Mn = Local.Min;
		const FVector Mx = Local.Max;
		const FVector V[8] = {
			Xform.TransformPosition(FVector(Mn.X, Mn.Y, Mn.Z)),
			Xform.TransformPosition(FVector(Mx.X, Mn.Y, Mn.Z)),
			Xform.TransformPosition(FVector(Mx.X, Mx.Y, Mn.Z)),
			Xform.TransformPosition(FVector(Mn.X, Mx.Y, Mn.Z)),
			Xform.TransformPosition(FVector(Mn.X, Mn.Y, Mx.Z)),
			Xform.TransformPosition(FVector(Mx.X, Mn.Y, Mx.Z)),
			Xform.TransformPosition(FVector(Mx.X, Mx.Y, Mx.Z)),
			Xform.TransformPosition(FVector(Mn.X, Mx.Y, Mx.Z)),
		};
		static const int32 Edges[12][2] = {
			{0,1},{1,2},{2,3},{3,0}, {4,5},{5,6},{6,7},{7,4}, {0,4},{1,5},{2,6},{3,7} };
		for (const auto& E : Edges)
		{
			PDI->DrawLine(V[E[0]], V[E[1]], Color, SDPG_World, 0.5f);
		}
	}

	// Coarse lattice (Div subdivisions per axis) inside the local AABB.
	void DrawCoarseGrid(FPrimitiveDrawInterface* PDI, const FBox& Local, const FTransform& Xform,
		const FLinearColor& Color, int32 Div)
	{
		Div = FMath::Max(1, Div);
		const FVector Mn = Local.Min;
		const FVector Sz = Local.GetSize();
		auto P = [&](double tx, double ty, double tz)
		{
			return Xform.TransformPosition(FVector(Mn.X + Sz.X * tx, Mn.Y + Sz.Y * ty, Mn.Z + Sz.Z * tz));
		};
		for (int32 i = 0; i <= Div; ++i)
		{
			const double ti = static_cast<double>(i) / Div;
			for (int32 j = 0; j <= Div; ++j)
			{
				const double tj = static_cast<double>(j) / Div;
				PDI->DrawLine(P(ti, tj, 0.0), P(ti, tj, 1.0), Color, SDPG_World, 0.25f); // along Z
				PDI->DrawLine(P(ti, 0.0, tj), P(ti, 1.0, tj), Color, SDPG_World, 0.25f); // along Y
				PDI->DrawLine(P(0.0, ti, tj), P(1.0, ti, tj), Color, SDPG_World, 0.25f); // along X
			}
		}
	}

	// Narrow-band voxels colored by sign.
	void DrawVoxels(FPrimitiveDrawInterface* PDI, const FRopeBoneSDFVolume& V, const FTransform& Xform, float Band)
	{
		const FVector Mn = V.LocalBounds.Min;
		const FVector Sz = V.LocalBounds.GetSize();
		const int32 NX = V.Resolution.X;
		const int32 NY = V.Resolution.Y;
		const int32 NZ = V.Resolution.Z;
		if (NX < 2 || NY < 2 || NZ < 2)
		{
			return;
		}

		// Cap total iteration with a stride for very high resolutions.
		const int64 Total = static_cast<int64>(NX) * NY * NZ;
		int32 Stride = 1;
		while ((Total / (static_cast<int64>(Stride) * Stride * Stride)) > 200000)
		{
			++Stride;
		}

		for (int32 Z = 0; Z < NZ; Z += Stride)
		{
			for (int32 Y = 0; Y < NY; Y += Stride)
			{
				for (int32 X = 0; X < NX; X += Stride)
				{
					const int32 Idx = X + Y * NX + Z * NX * NY;
					if (!V.Distances.IsValidIndex(Idx))
					{
						continue;
					}
					const float D = V.Distances[Idx];
					if (FMath::Abs(D) > Band)
					{
						continue;
					}
					const FVector L(
						Mn.X + Sz.X * (static_cast<double>(X) / (NX - 1)),
						Mn.Y + Sz.Y * (static_cast<double>(Y) / (NY - 1)),
						Mn.Z + Sz.Z * (static_cast<double>(Z) / (NZ - 1)));
					const FLinearColor C = (D < -0.01f) ? FLinearColor::Red
						: (D > 0.01f) ? FLinearColor(0.0f, 0.4f, 1.0f)
						: FLinearColor::White;
					PDI->DrawPoint(Xform.TransformPosition(L), C, 4.0f, SDPG_World);
				}
			}
		}
	}

	// 정규화 좌표(각 0~1)를 본 로컬 위치로 변환.
	FORCEINLINE FVector LocalFromNorm(const FBox& Local, double Tx, double Ty, double Tz)
	{
		const FVector Mn = Local.Min;
		const FVector Sz = Local.GetSize();
		return FVector(Mn.X + Sz.X * Tx, Mn.Y + Sz.Y * Ty, Mn.Z + Sz.Z * Tz);
	}

	// 발산형 heatmap: 음=파랑, 0=흰, 양=빨강. Scale(cm)에서 포화.
	FLinearColor HeatColor(float D, float Scale)
	{
		const float T = FMath::Clamp(D / FMath::Max(Scale, KINDA_SMALL_NUMBER), -1.0f, 1.0f);
		return (T >= 0.0f)
			? FMath::Lerp(FLinearColor::White, FLinearColor::Red, T)
			: FMath::Lerp(FLinearColor::White, FLinearColor(0.0f, 0.4f, 1.0f), -T);
	}

	// 한 축을 가로지르는 평면에 distance를 색 점으로.
	void DrawSlice(FPrimitiveDrawInterface* PDI, const FRopeBoneSDFVolume& V, const FTransform& Xform,
		ERopeSDFSliceAxis Axis, float Pos01, int32 Res, float Scale)
	{
		Res = FMath::Max(2, Res);
		for (int32 I = 0; I < Res; ++I)
		{
			const double U = static_cast<double>(I) / (Res - 1);
			for (int32 J = 0; J < Res; ++J)
			{
				const double W = static_cast<double>(J) / (Res - 1);
				double Tx, Ty, Tz;
				switch (Axis)
				{
				case ERopeSDFSliceAxis::X: Tx = Pos01; Ty = U; Tz = W; break;
				case ERopeSDFSliceAxis::Y: Tx = U; Ty = Pos01; Tz = W; break;
				default:                   Tx = U; Ty = W; Tz = Pos01; break; // Z
				}
				const FVector L = LocalFromNorm(V.LocalBounds, Tx, Ty, Tz);
				const float D = RopeSDFSampler::SampleTrilinear(V, L);
				PDI->DrawPoint(Xform.TransformPosition(L), HeatColor(D, Scale), 5.0f, SDPG_World);
			}
		}
	}

	// 좁은밴드 샘플에서 gradient(바깥쪽 = Query 법선) 방향을 화살표(선분)로.
	void DrawGradients(FPrimitiveDrawInterface* PDI, const FRopeBoneSDFVolume& V, const FTransform& Xform,
		float Band, float Length)
	{
		const int32 NX = V.Resolution.X;
		const int32 NY = V.Resolution.Y;
		const int32 NZ = V.Resolution.Z;
		if (NX < 2 || NY < 2 || NZ < 2)
		{
			return;
		}
		// 축당 ~8개로 스트라이드(빽빽함 방지).
		const int32 SX = FMath::Max(1, (NX - 1) / 8);
		const int32 SY = FMath::Max(1, (NY - 1) / 8);
		const int32 SZ = FMath::Max(1, (NZ - 1) / 8);
		for (int32 Z = 0; Z < NZ; Z += SZ)
		{
			for (int32 Y = 0; Y < NY; Y += SY)
			{
				for (int32 X = 0; X < NX; X += SX)
				{
					const int32 Idx = X + Y * NX + Z * NX * NY;
					if (!V.Distances.IsValidIndex(Idx) || FMath::Abs(V.Distances[Idx]) > Band)
					{
						continue;
					}
					const FVector L = LocalFromNorm(V.LocalBounds,
						static_cast<double>(X) / (NX - 1),
						static_cast<double>(Y) / (NY - 1),
						static_cast<double>(Z) / (NZ - 1));
					const FVector G = RopeSDFSampler::SampleGradient(V, L);
					PDI->DrawLine(Xform.TransformPosition(L), Xform.TransformPosition(L + G * Length),
						FLinearColor::Green, SDPG_World, 0.5f);
				}
			}
		}
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
			DrawBoundsBox(PDI, DrawVol->LocalBounds, Xform, FLinearColor(1.0f, 0.6f, 0.0f));
		}
		if (Provider->bDrawSDFGrid)
		{
			DrawCoarseGrid(PDI, DrawVol->LocalBounds, Xform, FLinearColor(0.3f, 0.3f, 0.3f), 4);
		}
		if (Provider->bDrawSDFVoxels && DrawVol->IsBaked())
		{
			DrawVoxels(PDI, *DrawVol, Xform, Provider->SDFBandThreshold);
		}
		if (Provider->bDrawSDFSlice && DrawVol->IsBaked())
		{
			DrawSlice(PDI, *DrawVol, Xform, Provider->SDFSliceAxis, Provider->SDFSlicePosition,
				Provider->SDFSliceResolution, Provider->SDFSliceColorScale);
		}
		if (Provider->bDrawSDFGradient && DrawVol->IsBaked())
		{
			DrawGradients(PDI, *DrawVol, Xform, Provider->SDFBandThreshold, Provider->SDFGradientLength);
		}
	}
}
