// Copyright Epic Games, Inc. All Rights Reserved.

#include "SDF/RopeSDFDraw.h"
#include "Collision/SDF/RopeSDFData.h"
#include "Collision/SDF/RopeSDFSampler.h"
#include "Collision/SDF/RopeSDFProvider.h" // ERopeSDFSliceAxis
#include "SceneManagement.h"               // FPrimitiveDrawInterface, SDPG_*

namespace
{
	// 정규화 좌표(각 0~1)를 본 로컬 위치로 변환.
	FORCEINLINE FVector LocalFromNorm(const FBox& Local, double Tx, double Ty, double Tz)
	{
		const FVector Mn = Local.Min;
		const FVector Sz = Local.GetSize();
		return FVector(Mn.X + Sz.X * Tx, Mn.Y + Sz.Y * Ty, Mn.Z + Sz.Z * Tz);
	}

	// 발산형 heatmap: 음(안)=빨강, 0=흰, 양(밖)=파랑. Scale(cm)에서 포화.
	FLinearColor HeatColor(float D, float Scale)
	{
		const float T = FMath::Clamp(D / FMath::Max(Scale, KINDA_SMALL_NUMBER), -1.0f, 1.0f);
		return (T >= 0.0f)
			? FMath::Lerp(FLinearColor::White, FLinearColor(0.0f, 0.4f, 1.0f), T)
			: FMath::Lerp(FLinearColor::White, FLinearColor::Red, -T);
	}
}

void RopeSDFDraw::DrawBounds(FPrimitiveDrawInterface* PDI, const FBox& Local, const FTransform& Xform, const FLinearColor& Color)
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

void RopeSDFDraw::DrawVoxels(FPrimitiveDrawInterface* PDI, const FRopeBoneSDFVolume& V, const FTransform& Xform, float Band, float NarrowBand)
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
				const float D = V.DecodeDistance(Idx);
				if (FMath::Abs(D) > Band)
				{
					continue;
				}
				// 포화(±NarrowBand 도달) 샘플은 클램프된 placeholder라 스킵 — Band 최댓값에서 plateau가
				// 통째로 들어와 박스를 채우며 튀는 현상을 막는다. NarrowBand <= 0이면 미상 → 스킵 비활성.
				if (NarrowBand > 0.0f && FMath::Abs(D) >= NarrowBand - KINDA_SMALL_NUMBER)
				{
					continue;
				}
				const FVector L(
					Mn.X + Sz.X * (static_cast<double>(X) / (NX - 1)),
					Mn.Y + Sz.Y * (static_cast<double>(Y) / (NY - 1)),
					Mn.Z + Sz.Z * (static_cast<double>(Z) / (NZ - 1)));
				// slice heatmap(HeatColor)과 동일 규약: 안(음)=빨강, 밖(양)=파랑, ≈0=흰색.
				const FLinearColor C = (D < -0.01f) ? FLinearColor::Red
					: (D > 0.01f) ? FLinearColor(0.0f, 0.4f, 1.0f)
					: FLinearColor::White;
				PDI->DrawPoint(Xform.TransformPosition(L), C, 4.0f, SDPG_World);
			}
		}
	}
}

void RopeSDFDraw::DrawSlice(FPrimitiveDrawInterface* PDI, const FRopeBoneSDFVolume& V, const FTransform& Xform,
	ERopeSDFSliceAxis Axis, float Pos01, int32 Res, float Scale, float NarrowBand)
{
	// 포화(±NarrowBand 도달) 샘플용 흐린 회색 — 무의미 plateau를 유의미 밴드와 시각적으로 분리한다.
	static const FLinearColor SaturatedColor(0.15f, 0.15f, 0.15f);
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
			// ±NarrowBand로 포화된 샘플은 실제 거리 정보가 없는 상수 plateau → 회색으로 그려 유의미
			// 밴드(표면·연속장)와 구분한다. NarrowBand <= 0(미상/구 에셋)이면 기존대로 전부 heatmap.
			const bool bSaturated = (NarrowBand > 0.0f) && (FMath::Abs(D) >= NarrowBand - KINDA_SMALL_NUMBER);
			const FLinearColor C = bSaturated ? SaturatedColor : HeatColor(D, Scale);
			PDI->DrawPoint(Xform.TransformPosition(L), C, 5.0f, SDPG_World);
		}
	}
}

void RopeSDFDraw::DrawGradients(FPrimitiveDrawInterface* PDI, const FRopeBoneSDFVolume& V, const FTransform& Xform,
	float Band, float Length, float NarrowBand)
{
	const int32 NX = V.Resolution.X;
	const int32 NY = V.Resolution.Y;
	const int32 NZ = V.Resolution.Z;
	if (NX < 2 || NY < 2 || NZ < 2)
	{
		return;
	}
	// 축당 ~6개로 스트라이드(빽빽함 방지).
	const int32 SX = FMath::Max(1, (NX - 1) / 6);
	const int32 SY = FMath::Max(1, (NY - 1) / 6);
	const int32 SZ = FMath::Max(1, (NZ - 1) / 6);
	for (int32 Z = 0; Z < NZ; Z += SZ)
	{
		for (int32 Y = 0; Y < NY; Y += SY)
		{
			for (int32 X = 0; X < NX; X += SX)
			{
				const int32 Idx = X + Y * NX + Z * NX * NY;
				if (!V.Distances.IsValidIndex(Idx))
				{
					continue;
				}
				const float D = V.DecodeDistance(Idx);
				if (FMath::Abs(D) > Band)
				{
					continue;
				}
				// 포화(±NarrowBand 도달) 샘플은 방향 정보가 없으므로(평탄=up 폴백, 경계=노이즈) 스킵한다.
				// NarrowBand <= 0이면 미상(구 에셋 등) → 스킵 비활성, 기존대로 그린다.
				if (NarrowBand > 0.0f && FMath::Abs(D) >= NarrowBand - KINDA_SMALL_NUMBER)
				{
					continue;
				}
				const FVector L = LocalFromNorm(V.LocalBounds,
					static_cast<double>(X) / (NX - 1),
					static_cast<double>(Y) / (NY - 1),
					static_cast<double>(Z) / (NZ - 1));
				const FVector G = RopeSDFSampler::SampleGradient(V, L);
				// 머리 달린 화살표로 그려 push-out(바깥) 방향이 보이게 한다. +X축을 그라디언트 방향으로
				// 회전시킨 행렬을 만들고 샘플 위치를 원점으로 둔다(스케일 비균등 대비 NoScale 변환).
				const FVector WorldDir = Xform.TransformVectorNoScale(G).GetSafeNormal();
				if (WorldDir.IsNearlyZero())
				{
					continue;
				}
				FMatrix ArrowToWorld = FRotationMatrix::MakeFromX(WorldDir);
				ArrowToWorld.SetOrigin(Xform.TransformPosition(L));
				DrawDirectionalArrow(PDI, ArrowToWorld, FLinearColor::Green,
					Length /*길이(cm)*/, Length * 0.05f /*화살촉 크기*/, SDPG_World, 0.2f /*두께*/);
			}
		}
	}
}
