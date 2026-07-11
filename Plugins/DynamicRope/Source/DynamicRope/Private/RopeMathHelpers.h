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

	/**
	 * Whip guide의 시간 가변 원시 중심선을 만든다.
	 * Aim-hit은 현재 sweep 방향 하나로 직선을 만들며, 호출자가 throw frame/swing plane으로 그 방향을
	 * 시간에 따라 회전시킨다. 일반 whip은 기존 공간/시간 보간 경로를 사용한다.
	 */
	inline void BuildWhipGuideRawPoints(const FVector& Origin, const FVector& SweepDirection,
		const FVector& AimDirection, bool bHasAimTarget, float NormalizedTime,
		float GuideLength, const FVector& InheritedDrift, float AimSteerStartAlpha,
		float AimLockAlpha, float AimDirectionBias, int32 RequestedSampleCount, TArray<FVector>& OutPoints)
	{
		// 1. 기본값 정리
		OutPoints.Reset();
		const int32 SampleCount = FMath::Max(RequestedSampleCount, 4);
		const float PathLength = FMath::Max(GuideLength, KINDA_SMALL_NUMBER);
		// 두 방향을 정규화한다. SweepDirection이 퇴화면 AimDirection을, AimDirection이 퇴화면 SweepDir을 쓴다.
		const FVector SweepDir = SafeNormalOr(SweepDirection, AimDirection);
		const FVector AimDir = SafeNormalOr(AimDirection, SweepDir);

		// Aim-hit keeps each frame's spline straight. SweepDir is computed from the
		// wielder throw frame and swing plane, so the whole line sweeps an arc over time
		// and reaches the Origin->Hit direction at the end of the swing.
		if (bHasAimTarget)
		{
			OutPoints.Reserve(SampleCount);
			for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
			{
				const float RopeAlpha = static_cast<float>(SampleIndex) /
					static_cast<float>(SampleCount - 1);
				OutPoints.Add(Origin + SweepDir * (RopeAlpha * PathLength));
			}
			return;
		}

		// 2. 공간 보간 구간: 0% ─── SteerStart ─── FullSteer ─── 100%
		//                    Sweep 유지   Hit 방향으로 보간   Hit 방향 영향 최대
		const float SteerStart = FMath::Clamp(AimSteerStartAlpha, 0.0f, 0.95f);
		const float FullSteer = FMath::Clamp(FMath::Max(AimLockAlpha, SteerStart + 0.01f), 0.01f, 1.0f);
		const float DirectionBias = FMath::Clamp(AimDirectionBias, 1.0f, 4.0f);

		// 3. 시간 보간. DirectionBias가 클수록 hit 방향 영향이 빠르게 강해진다.
		// bias는 hit 방향 전환을 앞당기지만 T=0에서는 반드시 0이라 throw 시작 순간 점프가 없다.
		const float TemporalBase = SmoothStep(NormalizedTime);
		const float TemporalAimBlend = bHasAimTarget
			? 1.0f - FMath::Pow(1.0f - TemporalBase, DirectionBias)
			: 0.0f;

		OutPoints.Reserve(SampleCount);
		for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
		{
			// 4. 로프 위치별 공간 보간: 각 점이 로프의 몇 퍼센트 지점인지 계산한다.
			const float RopeAlpha = static_cast<float>(SampleIndex) / static_cast<float>(SampleCount - 1);
			const float SpatialBase = SmoothStep(
				(RopeAlpha - SteerStart) / FMath::Max(FullSteer - SteerStart, KINDA_SMALL_NUMBER));
			const float SpatialAimBlend = bHasAimTarget
				? 1.0f - FMath::Pow(1.0f - SpatialBase, DirectionBias)
				: 0.0f;
			// 5. 공간 보간만으로 자유단을 미리 고정하지 않도록 Flight 시간 보간을 반드시 곱한다.
			//    (자유단의 공간 보간이 1이어도 Flight 시작 시점에는 hit 방향에 붙지 않는다.)
			const float AimBlend = SpatialAimBlend * TemporalAimBlend;

			// 6. 최종 방향
			const FVector CurveDirection = SafeNormalOr(FMath::Lerp(SweepDir, AimDir, AimBlend), SweepDir);

			// 7. 상속 이동량. 손 근처(RopeAlpha≈0)는 drift 거의 없음, 자유단(RopeAlpha≈1)은 drift 영향 증가.
			const float DriftWeight = SmoothStep(RopeAlpha) * (1.0f - AimBlend);

			// 8. 점 생성
			OutPoints.Add(Origin + CurveDirection * (RopeAlpha * PathLength) +
				InheritedDrift * DriftWeight);
		}
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
