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
		// 양자화 코드 → cm(범위 밖/무효 인덱스는 0).
		return V.DecodeDistance(Idx);
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
	// 단 +H 프로브가 그리드 Max 경계 밖이면 SampleTrilinear가 경계면으로 클램프돼 그 축 차분이 0으로
	// 축퇴한다(경계 법선 성분 소실 → 접선 방향 법선). 절단면(본 이음매) 밖에서 노드가 옆으로 밀리는 #4를
	// 막기 위해, 그 축만 후방 차분으로 대체한다(내부 점은 종전 순방향 그대로 — 핫패스 4샘플 유지).
	const float C = SampleTrilinear(V, LocalPos);
	const auto AxisDeriv = [&V, &LocalPos, C](int32 Axis, double H) -> float
	{
		FVector Pp = LocalPos;
		Pp[Axis] += H;
		if (Pp[Axis] <= V.LocalBounds.Max[Axis])
		{
			return static_cast<float>((SampleTrilinear(V, Pp) - C) / H);
		}
		FVector Pm = LocalPos;
		Pm[Axis] -= H;
		return static_cast<float>((C - SampleTrilinear(V, Pm)) / H);
	};

	const FVector Grad(AxisDeriv(0, Hx), AxisDeriv(1, Hy), AxisDeriv(2, Hz));
	const FVector N = Grad.GetSafeNormal();
	return N.IsNearlyZero() ? FVector::UpVector : N;
}

FVector RopeSDFSampler::SampleProjectionGradient(const FRopeBoneSDFVolume& V, const FVector& LocalPos)
{
	if (!V.IsBaked())
	{
		return FVector::ZeroVector;
	}

	const FVector Size = V.LocalBounds.GetSize();
	const FVector H(
		(V.Resolution.X > 1) ? (Size.X / (V.Resolution.X - 1)) : 1.0,
		(V.Resolution.Y > 1) ? (Size.Y / (V.Resolution.Y - 1)) : 1.0,
		(V.Resolution.Z > 1) ? (Size.Z / (V.Resolution.Z - 1)) : 1.0);

	auto AxisDerivative = [&](int32 Axis, float Step)
	{
		FVector Minus = LocalPos;
		FVector Plus = LocalPos;
		Minus[Axis] = FMath::Max(LocalPos[Axis] - Step, V.LocalBounds.Min[Axis]);
		Plus[Axis] = FMath::Min(LocalPos[Axis] + Step, V.LocalBounds.Max[Axis]);

		const float Span = static_cast<float>(Plus[Axis] - Minus[Axis]);
		if (Span <= KINDA_SMALL_NUMBER)
		{
			return 0.0f;
		}

		return (SampleTrilinear(V, Plus) - SampleTrilinear(V, Minus)) / Span;
	};

	return FVector(
		AxisDerivative(0, H.X),
		AxisDerivative(1, H.Y),
		AxisDerivative(2, H.Z)).GetSafeNormal();
}
