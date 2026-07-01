// Copyright Epic Games, Inc. All Rights Reserved.
//
// 해석적 SDF로 FRopeBoneSDFVolume를 채우는 헬퍼. 진짜 베이커(B3)가 없어도 샘플러/시각화를
// 개발·검증할 수 있게 해 준다(병렬 트랙의 핵심). 테스트 fixture로도 쓴다. 헤더 전용(inline).

#pragma once

#include "CoreMinimal.h"
#include "Collision/SDF/RopeSDFData.h"

namespace RopeSDFSynthetic
{
	/**
	 * 본 로컬 공간에 해석적 구 SDF를 굽는다: distance = |P - Center| - Radius (바깥 +, 안 -).
	 * 샘플은 그리드 노드(Min..Max 포함 균등 분할, RopeSDFSampler 규약)에 위치한다.
	 */
	inline FRopeBoneSDFVolume MakeSphere(FName Bone, const FVector& Center, float Radius,
		const FIntVector& Resolution, float Padding = 8.0f)
	{
		FRopeBoneSDFVolume V;
		V.Bone = Bone;
		V.Resolution = FIntVector(
			FMath::Max(2, Resolution.X), FMath::Max(2, Resolution.Y), FMath::Max(2, Resolution.Z));

		const float R = Radius + Padding;
		V.LocalBounds = FBox(Center - FVector(R), Center + FVector(R));

		const FVector Min = V.LocalBounds.Min;
		const FVector Size = V.LocalBounds.GetSize();
		V.VoxelSize = static_cast<float>(Size.X / (V.Resolution.X - 1));

		const int32 NX = V.Resolution.X;
		const int32 NY = V.Resolution.Y;
		const int32 NZ = V.Resolution.Z;
		const int32 N = NX * NY * NZ;

		// 1패스: 해석적 거리(float)를 임시로 모으며 최대 |거리|를 구한다. 합성 데이터는 클램프되지 않으므로
		// 양자화 범위(NarrowBand)를 데이터 최댓값에 맞춰, 클램프 없이 양자화 rounding 손실만 남긴다.
		TArray<float> Raw;
		Raw.SetNumUninitialized(N);
		float MaxAbs = KINDA_SMALL_NUMBER;
		for (int32 Z = 0; Z < NZ; ++Z)
		{
			for (int32 Y = 0; Y < NY; ++Y)
			{
				for (int32 X = 0; X < NX; ++X)
				{
					const FVector P(
						Min.X + Size.X * (static_cast<double>(X) / (NX - 1)),
						Min.Y + Size.Y * (static_cast<double>(Y) / (NY - 1)),
						Min.Z + Size.Z * (static_cast<double>(Z) / (NZ - 1)));
					const int32 Idx = X + Y * NX + Z * NX * NY;
					const float D = static_cast<float>((P - Center).Size()) - Radius;
					Raw[Idx] = D;
					MaxAbs = FMath::Max(MaxAbs, FMath::Abs(D));
				}
			}
		}

		// 2패스: uint8로 양자화. 구는 대칭이라 안쪽/바깥 밴드 모두 데이터 최대 |거리|로 둔다(대칭 [-MaxAbs,+MaxAbs]).
		V.NarrowBandInner = MaxAbs;
		V.NarrowBandOuter = MaxAbs;
		V.Distances.SetNumUninitialized(N);
		for (int32 i = 0; i < N; ++i)
		{
			V.Distances[i] = FRopeBoneSDFVolume::EncodeDistance(Raw[i], V.NarrowBandInner, V.NarrowBandOuter);
		}
		return V;
	}
}
