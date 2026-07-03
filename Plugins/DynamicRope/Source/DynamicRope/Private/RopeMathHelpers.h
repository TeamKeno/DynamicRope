// Copyright Epic Games, Inc. All Rights Reserved.
//
// 모듈 내부 공용 수학 헬퍼. 여러 .cpp가 같은 헬퍼를 익명 네임스페이스에 각자 정의하면
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

	/** normal에 수직인 임의의 안정적인 tangent(퇴화 케이스 fallback 내장). */
	inline FVector AnyTangentFromNormal(const FVector& Normal)
	{
		const FVector N = Normal.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		const FVector Reference = FMath::Abs(FVector::DotProduct(N, FVector::UpVector)) < 0.9f
			? FVector::UpVector
			: FVector::RightVector;
		return FVector::CrossProduct(Reference, N).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
	}
}
