// Copyright Epic Games, Inc. All Rights Reserved.
//
// 솔버/로직 단위 테스트용 헬퍼 — POD FRopeSimState fixture 빌더와 스크립트된 IRopeCollider mock.
// UObject 의존이 없어 월드 없이도 순수 단위 테스트가 가능하다(솔버가 POD로 설계된 이유).

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"
#include "Collision/RopeCollider.h"

namespace RopeTest
{
	/** 직선 체인 fixture. InvMass 1, 속도 0(Prev=Pos), 기본은 핀 없음(자유 체인). */
	inline FRopeSimState MakeStraightRope(int32 NumNodes, float Length,
		const FVector& Start = FVector::ZeroVector, const FVector& Dir = FVector(1, 0, 0))
	{
		FRopeSimState S;
		const int32 N = FMath::Max(2, NumNodes);
		S.Positions.SetNum(N);
		S.PrevPositions.SetNum(N);
		S.InvMass.SetNum(N);
		S.RopeLength = Length;
		S.SegmentLength = Length / static_cast<float>(N - 1);
		const FVector D = Dir.GetSafeNormal();
		for (int32 i = 0; i < N; ++i)
		{
			const float Alpha = static_cast<float>(i) / static_cast<float>(N - 1);
			const FVector P = Start + D * (Length * Alpha);
			S.Positions[i] = P;
			S.PrevPositions[i] = P;
			S.InvMass[i] = 1.0f;
		}
		return S;
	}

	/** 최대 세그먼트 길이 오차 |len - SegmentLength|. */
	inline float MaxSegmentError(const FRopeSimState& S)
	{
		float MaxErr = 0.0f;
		for (int32 i = 0; i + 1 < S.Num(); ++i)
		{
			const float Len = static_cast<float>(FVector::Dist(S.Positions[i], S.Positions[i + 1]));
			MaxErr = FMath::Max(MaxErr, FMath::Abs(Len - S.SegmentLength));
		}
		return MaxErr;
	}

	inline bool AnyNaN(const FRopeSimState& S)
	{
		for (const FVector& P : S.Positions)
		{
			if (P.ContainsNaN())
			{
				return true;
			}
		}
		return false;
	}

	/**
	 * 스크립트된 구체 collider. center 반경 Radius 안의 query에 바깥쪽 normal로 컨택트를 보고한다.
	 * SourceMesh는 nullptr — DecideWrap은 이를 저장만 하고 역참조하지 않으므로 단위 테스트에 안전하다.
	 */
	class FSphereMockCollider : public IRopeCollider
	{
	public:
		FVector Center = FVector::ZeroVector;
		float   Radius = 0.0f;
		FName   Bone = NAME_None;

		FSphereMockCollider() = default;
		FSphereMockCollider(const FVector& InCenter, float InRadius, FName InBone)
			: Center(InCenter), Radius(InRadius), Bone(InBone) {}

		virtual FRopeContact Query(const FVector& WorldPos, float NodeRadius) const override
		{
			FRopeContact C;
			const FVector D = WorldPos - Center;
			const float Dist = static_cast<float>(D.Size());
			const float Pen = (Radius + NodeRadius) - Dist;
			if (Pen > 0.0f)
			{
				C.bHit = true;
				C.Penetration = Pen;
				C.Normal = (Dist > KINDA_SMALL_NUMBER) ? (D / Dist) : FVector::UpVector;
				C.SurfacePoint = Center + C.Normal * Radius;
				C.Bone = Bone;
				C.SourceMesh = nullptr;
			}
			return C;
		}

		virtual FBox GetWorldBounds() const override
		{
			return FBox(Center - FVector(Radius), Center + FVector(Radius));
		}
	};
}
