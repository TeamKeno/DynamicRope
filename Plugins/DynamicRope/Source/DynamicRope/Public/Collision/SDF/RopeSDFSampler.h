// Copyright Epic Games, Inc. All Rights Reserved.
//
// FRopeBoneSDFVolume를 샘플링하는 순수 유틸. 시각화와 장차 FRopeSDFCollider::Query가 공유한다
// (지금은 후자가 스텁). UObject 의존 없음 → 단위 테스트 가능. 좌표는 모두 본 로컬 공간.

#pragma once

#include "CoreMinimal.h"

struct FRopeBoneSDFVolume;

namespace RopeSDFSampler
{
	/** 본 로컬 위치에서 trilinear 보간한 signed distance(cm, 바깥 +). 미베이크면 0. */
	DYNAMICROPE_API float SampleTrilinear(const FRopeBoneSDFVolume& Volume, const FVector& LocalPos);

	/** central-difference gradient를 정규화한 바깥쪽 방향(= Query가 반환할 법선). 축퇴 시 +Z. */
	DYNAMICROPE_API FVector SampleGradient(const FRopeBoneSDFVolume& Volume, const FVector& LocalPos);

	/** projection용 경계 대응 gradient. LocalBounds 가장자리에서는 가능한 쪽의 차분을 사용한다. */
	DYNAMICROPE_API FVector SampleProjectionGradient(const FRopeBoneSDFVolume& Volume, const FVector& LocalPos);
}
