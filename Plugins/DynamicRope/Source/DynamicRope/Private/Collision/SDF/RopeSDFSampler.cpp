// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/SDF/RopeSDFSampler.h"
#include "Collision/SDF/RopeSDFData.h"

namespace
{
	// 샘플 규약: 그리드 노드는 LocalBounds를 Min..Max 포함으로 균등 분할한다. 노드 i의 정규화 좌표는
	// i/(Res-1), 따라서 첫 노드=Min, 마지막 노드=Max. 베이커(B3)도 반드시 이 규약을 따라야 한다.

	/**
	 * 볼륨 1개의 디코드/인덱싱 상수를 미리 뽑아 둔 뷰.
	 *
	 * 기존에는 탭 하나마다 FRopeBoneSDFVolume::DecodeDistance가 QuantRange() 재계산, BytesPerCode() 분기,
	 * Range/MaxCode 나눗셈, Distances.IsValidIndex() 경계검사를 다시 했다. trilinear 1회가 8탭이고
	 * SampleGradient는 trilinear를 4회 부르므로 접촉 1회당 40탭 — 전부 같은 볼륨에 대한 같은 계산이다.
	 * 그 상수들을 샘플 진입 시 1회만 만들어 돌려 쓴다.
	 *
	 * 값은 한 비트도 바뀌지 않는다: 나눗셈 Range/MaxCodeF는 결정적이라 1회 계산이든 8회 계산이든 같은
	 * float이고, `- NarrowBandInner`를 `+ (-NarrowBandInner)`로 쓴 것도 IEEE754에서 동일하다.
	 * 경계검사를 생략할 수 있는 근거는 IsBaked()다 — Distances.Num() == ResX*ResY*ResZ*BytesPerCode를
	 * 보장하므로, 축별로 [0,Res-1]에 갇힌 인덱스는 항상 유효하다. 공개 진입점은 모두 IsBaked()를 먼저 본다.
	 */
	struct FVolumeReader
	{
		const uint8* Data = nullptr;
		int32 ResX = 0;
		int32 ResY = 0;
		int32 ResZ = 0;
		/** 인덱싱 스트라이드(= ResX, ResX*ResY). 탭마다 곱셈을 되풀이하지 않으려고 뽑아 둔다. */
		int32 StrideY = 0;
		int32 StrideZ = 0;
		int32 Bpc = 1;
		/** 코드 → cm 선형 복원: Code * DecodeScale + DecodeBias (DecodeDistance와 동일 식). */
		float DecodeScale = 0.0f;
		float DecodeBias = 0.0f;

		explicit FVolumeReader(const FRopeBoneSDFVolume& V)
			: Data(V.Distances.GetData())
			, ResX(V.Resolution.X)
			, ResY(V.Resolution.Y)
			, ResZ(V.Resolution.Z)
			, StrideY(V.Resolution.X)
			, StrideZ(V.Resolution.X * V.Resolution.Y)
			, Bpc(V.BytesPerCode())
			, DecodeBias(-V.NarrowBandInner)
		{
			const float MaxCodeF = (Bpc >= 2) ? 65535.0f : 255.0f;
			DecodeScale = V.QuantRange() / MaxCodeF;
		}

		/** 복셀 인덱스 → cm. 인덱스가 유효하다는 전제(IsBaked + 축별 clamp)라 경계검사 없음. */
		FORCEINLINE float Decode(int32 Index) const
		{
			const int32 Base = Index * Bpc;
			uint32 Code = Data[Base];
			if (Bpc >= 2)
			{
				// 리틀엔디안.
				Code |= static_cast<uint32>(Data[Base + 1]) << 8;
			}
			return static_cast<float>(Code) * DecodeScale + DecodeBias;
		}

		/** 격자 밖 좌표를 경계면으로 클램프해 읽는다(그리드 Max 면에 정확히 걸친 탭 전용 경로). */
		FORCEINLINE float FetchClamped(int32 X, int32 Y, int32 Z) const
		{
			X = FMath::Clamp(X, 0, ResX - 1);
			Y = FMath::Clamp(Y, 0, ResY - 1);
			Z = FMath::Clamp(Z, 0, ResZ - 1);
			return Decode(X + Y * StrideY + Z * StrideZ);
		}
	};

	FORCEINLINE double GridCoord(double P, double Mn, double Size, int32 Res)
	{
		if (Size <= KINDA_SMALL_NUMBER || Res < 2)
		{
			return 0.0;
		}
		const double T = FMath::Clamp((P - Mn) / Size, 0.0, 1.0);
		return T * (Res - 1);
	}

	/** 리더를 이미 만든 뒤의 trilinear 본체. SampleGradient처럼 한 볼륨을 여러 번 찌르는 쪽이 재사용한다. */
	float SampleTrilinearWith(const FVolumeReader& R, const FRopeBoneSDFVolume& V, const FVector& LocalPos)
	{
		const FVector Min = V.LocalBounds.Min;
		const FVector Size = V.LocalBounds.GetSize();

		const double Gx = GridCoord(LocalPos.X, Min.X, Size.X, R.ResX);
		const double Gy = GridCoord(LocalPos.Y, Min.Y, Size.Y, R.ResY);
		const double Gz = GridCoord(LocalPos.Z, Min.Z, Size.Z, R.ResZ);

		const int32 X0 = FMath::FloorToInt(Gx);
		const int32 Y0 = FMath::FloorToInt(Gy);
		const int32 Z0 = FMath::FloorToInt(Gz);
		const float Fx = static_cast<float>(Gx - X0);
		const float Fy = static_cast<float>(Gy - Y0);
		const float Fz = static_cast<float>(Gz - Z0);

		float C000, C100, C010, C110, C001, C101, C011, C111;
		// 고속 경로: 셀이 격자 내부면 +1 탭이 범위를 못 넘으므로 축별 클램프(8탭 × 3축)를 통째로 생략하고
		// 코너 인덱스를 스트라이드 오프셋으로 바로 짚는다. 클램프가 실제로 무는 경우는 GridCoord가 정확히
		// Res-1을 뱉는 격자 Max 면뿐이라, 아래 느린 경로는 거의 타지 않는다.
		if (X0 + 1 < R.ResX && Y0 + 1 < R.ResY && Z0 + 1 < R.ResZ && X0 >= 0 && Y0 >= 0 && Z0 >= 0)
		{
			const int32 I = X0 + Y0 * R.StrideY + Z0 * R.StrideZ;
			C000 = R.Decode(I);
			C100 = R.Decode(I + 1);
			C010 = R.Decode(I + R.StrideY);
			C110 = R.Decode(I + R.StrideY + 1);
			C001 = R.Decode(I + R.StrideZ);
			C101 = R.Decode(I + R.StrideZ + 1);
			C011 = R.Decode(I + R.StrideZ + R.StrideY);
			C111 = R.Decode(I + R.StrideZ + R.StrideY + 1);
		}
		else
		{
			C000 = R.FetchClamped(X0,     Y0,     Z0);
			C100 = R.FetchClamped(X0 + 1, Y0,     Z0);
			C010 = R.FetchClamped(X0,     Y0 + 1, Z0);
			C110 = R.FetchClamped(X0 + 1, Y0 + 1, Z0);
			C001 = R.FetchClamped(X0,     Y0,     Z0 + 1);
			C101 = R.FetchClamped(X0 + 1, Y0,     Z0 + 1);
			C011 = R.FetchClamped(X0,     Y0 + 1, Z0 + 1);
			C111 = R.FetchClamped(X0 + 1, Y0 + 1, Z0 + 1);
		}

		const float X00 = FMath::Lerp(C000, C100, Fx);
		const float X10 = FMath::Lerp(C010, C110, Fx);
		const float X01 = FMath::Lerp(C001, C101, Fx);
		const float X11 = FMath::Lerp(C011, C111, Fx);
		const float Y0V = FMath::Lerp(X00, X10, Fy);
		const float Y1V = FMath::Lerp(X01, X11, Fy);
		return FMath::Lerp(Y0V, Y1V, Fz);
	}
}

float RopeSDFSampler::SampleTrilinear(const FRopeBoneSDFVolume& V, const FVector& LocalPos)
{
	if (!V.IsBaked())
	{
		return 0.0f;
	}
	return SampleTrilinearWith(FVolumeReader(V), V, LocalPos);
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
	// 리더는 4번의 trilinear가 공유한다(볼륨이 같으므로 디코드 상수도 같다).
	const FVolumeReader R(V);
	const float C = SampleTrilinearWith(R, V, LocalPos);
	const auto AxisDeriv = [&R, &V, &LocalPos, C](int32 Axis, double H) -> float
	{
		FVector Pp = LocalPos;
		Pp[Axis] += H;
		if (Pp[Axis] <= V.LocalBounds.Max[Axis])
		{
			return static_cast<float>((SampleTrilinearWith(R, V, Pp) - C) / H);
		}
		FVector Pm = LocalPos;
		Pm[Axis] -= H;
		return static_cast<float>((C - SampleTrilinearWith(R, V, Pm)) / H);
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

	// 6번의 trilinear가 리더 하나를 공유한다.
	const FVolumeReader R(V);
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

		return (SampleTrilinearWith(R, V, Plus) - SampleTrilinearWith(R, V, Minus)) / Span;
	};

	return FVector(
		AxisDerivative(0, H.X),
		AxisDerivative(1, H.Y),
		AxisDerivative(2, H.Z)).GetSafeNormal();
}
