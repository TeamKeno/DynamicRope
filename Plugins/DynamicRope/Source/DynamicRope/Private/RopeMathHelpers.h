// Copyright Epic Games, Inc. All Rights Reserved.
//
// 모듈 내부 공용 수학/유틸 헬퍼. 여러 .cpp가 같은 헬퍼를 익명 네임스페이스에 각자 정의하면
// unity 빌드(여러 .cpp가 한 TU로 병합)에서 익명 네임스페이스가 합쳐져 중복 정의 오류가
// 나므로, 공유가 필요한 헬퍼는 여기 inline으로 두고 RopeMath:: 로 사용한다.

#pragma once

#include "CoreMinimal.h"

namespace RopeMath
{
	/** 0~1 구간의 부드러운 보간 가중치(3t^2 - 2t^3). */
	inline float SmoothStep(float T)
	{
		T = FMath::Clamp(T, 0.0f, 1.0f);
		return T * T * (3.0f - 2.0f * T);
	}

	/** 점 P의 세그먼트(SegA-SegB) 위 최근접 파라미터 t(0..1 clamp). 캡슐 접촉 재질점 식별 등에 쓴다. */
	inline float ClosestSegmentParam(const FVector& P, const FVector& SegA, const FVector& SegB)
	{
		const FVector Seg = SegB - SegA;
		const float SegSq = static_cast<float>(Seg.SizeSquared());
		return (SegSq > KINDA_SMALL_NUMBER) ? FMath::Clamp(static_cast<float>((P - SegA) | Seg) / SegSq, 0.0f, 1.0f) : 0.0f;
	}

	/** normal에 수직인 임의의 안정적인 tangent(퇴화 케이스 fallback 내장). */
	inline FVector AnyTangentFromNormal(const FVector& Normal)
	{
		const FVector N = Normal.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		const FVector Reference = FMath::Abs(FVector::DotProduct(N, FVector::UpVector)) < 0.9f
			? FVector::UpVector
			: FVector::RightVector;
		return FVector::CrossProduct(Reference, N).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
	}

	/** Value의 정규화. 퇴화(0 벡터)면 Fallback의 정규화를 반환(FRopeWhipGuide::SafeNormalOr와 동일 의미). */
	inline FVector SafeNormalOr(const FVector& Value, const FVector& Fallback)
	{
		const FVector Normalized = Value.GetSafeNormal();
		return Normalized.IsNearlyZero() ? Fallback.GetSafeNormal() : Normalized;
	}

	/**
	 * 던지기 미리보기 호(arc)에서 Alpha(0=스윕 시작=조준 반대편, 1=조준 방향) 지점의 단위 방향.
	 * AimDirRaw/GuideUpRaw는 호 평면 기준축(비정규화/비직교 허용 — 내부에서 정규화·직교화).
	 */
	inline FVector ArcDirectionAtAlpha(const FVector& AimDirRaw, const FVector& GuideUpRaw,
		float SweepAngleDegrees, float Alpha)
	{
		const FVector Aim = SafeNormalOr(AimDirRaw, FVector::ForwardVector);
		FVector Up = GuideUpRaw - FVector::DotProduct(GuideUpRaw, Aim) * Aim;
		Up = SafeNormalOr(Up, FVector::UpVector);

		const float ClampedAlpha = FMath::Clamp(Alpha, 0.0f, 1.0f);
		const float SweepRadians = FMath::DegreesToRadians(FMath::Clamp(SweepAngleDegrees, 1.0f, 180.0f));
		const float Angle = SweepRadians * (1.0f - ClampedAlpha);
		return (Aim * FMath::Cos(Angle) + Up * FMath::Sin(Angle)).GetSafeNormal();
	}

	/** 노드 인덱스 목록 중 Positions 범위 내에서 가장 손(node 0)에 가까운 인덱스. 없으면 INDEX_NONE. */
	inline int32 HeadValidNodeIndex(const TArray<int32>& NodeIndices, const TArray<FVector>& Positions)
	{
		int32 HeadNodeIndex = INDEX_NONE;
		for (const int32 NodeIndex : NodeIndices)
		{
			if (!Positions.IsValidIndex(NodeIndex))
			{
				continue;
			}

			if (HeadNodeIndex == INDEX_NONE || NodeIndex < HeadNodeIndex)
			{
				HeadNodeIndex = NodeIndex;
			}
		}
		return HeadNodeIndex;
	}

	/** 선택적 실패 사유 out 파라미터 세터(null 허용). 미리보기 API들의 공용 패턴. */
	inline void SetPreviewFailureReason(FString* OutFailureReason, const FString& Reason)
	{
		if (OutFailureReason)
		{
			*OutFailureReason = Reason;
		}
	}
}
