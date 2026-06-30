// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/SDF/RopeSDFSampler.h"
#include "Collision/SDF/RopeSDFData.h"

namespace
{
	// 샘플 규약: 그리드 노드는 LocalBounds를 Min..Max 포함으로 균등 분할한다. 노드 i의 정규화 좌표는
	// i/(Res-1), 따라서 첫 노드=Min, 마지막 노드=Max. 베이커(B3)도 반드시 이 규약을 따라야 한다.
	FORCEINLINE float SampleAt(const FRopeBoneSDFVolume& V, int32 X, int32 Y, int32 Z)
	{
		X = FMath::Clamp(X, 0, V.Resolution.X - 1);
		Y = FMath::Clamp(Y, 0, V.Resolution.Y - 1);
		Z = FMath::Clamp(Z, 0, V.Resolution.Z - 1);
		const int32 Idx = X + Y * V.Resolution.X + Z * V.Resolution.X * V.Resolution.Y;
		return V.DecodeDistance(Idx); // uint8 코드 → cm(범위 밖/무효 인덱스는 0)
	}

	FORCEINLINE double GridCoord(double P, double Mn, double Size, int32 Res)
	{
		if (Size <= KINDA_SMALL_NUMBER || Res < 2)
		{
			return 0.0;
		}
		const double T = FMath::Clamp((P - Mn) / Size, 0.0, 1.0);
		return T * (Res - 1);
	}
}

float RopeSDFSampler::SampleTrilinear(const FRopeBoneSDFVolume& V, const FVector& LocalPos)
{
	if (!V.IsBaked())
	{
		return 0.0f;
	}

	const FVector Min = V.LocalBounds.Min;
	const FVector Size = V.LocalBounds.GetSize();

	const double Gx = GridCoord(LocalPos.X, Min.X, Size.X, V.Resolution.X);
	const double Gy = GridCoord(LocalPos.Y, Min.Y, Size.Y, V.Resolution.Y);
	const double Gz = GridCoord(LocalPos.Z, Min.Z, Size.Z, V.Resolution.Z);

	const int32 X0 = FMath::FloorToInt(Gx);
	const int32 Y0 = FMath::FloorToInt(Gy);
	const int32 Z0 = FMath::FloorToInt(Gz);
	const float Fx = static_cast<float>(Gx - X0);
	const float Fy = static_cast<float>(Gy - Y0);
	const float Fz = static_cast<float>(Gz - Z0);

	const float C000 = SampleAt(V, X0,     Y0,     Z0);
	const float C100 = SampleAt(V, X0 + 1, Y0,     Z0);
	const float C010 = SampleAt(V, X0,     Y0 + 1, Z0);
	const float C110 = SampleAt(V, X0 + 1, Y0 + 1, Z0);
	const float C001 = SampleAt(V, X0,     Y0,     Z0 + 1);
	const float C101 = SampleAt(V, X0 + 1, Y0,     Z0 + 1);
	const float C011 = SampleAt(V, X0,     Y0 + 1, Z0 + 1);
	const float C111 = SampleAt(V, X0 + 1, Y0 + 1, Z0 + 1);

	const float X00 = FMath::Lerp(C000, C100, Fx);
	const float X10 = FMath::Lerp(C010, C110, Fx);
	const float X01 = FMath::Lerp(C001, C101, Fx);
	const float X11 = FMath::Lerp(C011, C111, Fx);
	const float Y0V = FMath::Lerp(X00, X10, Fy);
	const float Y1V = FMath::Lerp(X01, X11, Fy);
	return FMath::Lerp(Y0V, Y1V, Fz);
}

FVector RopeSDFSampler::SampleGradient(const FRopeBoneSDFVolume& V, const FVector& LocalPos)
{
	if (!V.IsBaked())
	{
		return FVector::UpVector;
	}

	const FVector Size = V.LocalBounds.GetSize();
	const double Hx = (V.Resolution.X > 1) ? (Size.X / (V.Resolution.X - 1)) : 1.0;
	const double Hy = (V.Resolution.Y > 1) ? (Size.Y / (V.Resolution.Y - 1)) : 1.0;
	const double Hz = (V.Resolution.Z > 1) ? (Size.Z / (V.Resolution.Z - 1)) : 1.0;

	// Forward difference(center + 축당 1샘플 = 4) — 기존 central(6)보다 trilinear 2회 적다(핫패스 비용↓).
	// SDF 내부는 단조로워 push-out 법선 방향엔 충분(완전 대칭 정확도는 약간 손해).
	const float C  = SampleTrilinear(V, LocalPos);
	const float Dx = SampleTrilinear(V, LocalPos + FVector(Hx, 0, 0)) - C;
	const float Dy = SampleTrilinear(V, LocalPos + FVector(0, Hy, 0)) - C;
	const float Dz = SampleTrilinear(V, LocalPos + FVector(0, 0, Hz)) - C;

	const FVector Grad(Dx / Hx, Dy / Hy, Dz / Hz);
	const FVector N = Grad.GetSafeNormal();
	return N.IsNearlyZero() ? FVector::UpVector : N;
}
