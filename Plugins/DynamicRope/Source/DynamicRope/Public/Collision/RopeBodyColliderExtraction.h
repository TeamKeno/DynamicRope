// Copyright Epic Games, Inc. All Rights Reserved.
//
// UBodySetup 심플 콜리전(sphyl/sphere/box/convex)을 월드 공간 push-out 콜라이더로 추출하는 공용 헬퍼.
// URopeStaticBodyProvider(채널 오버랩 경로)와 URopeWrapTargetComponent(대상 직접 추출 경로)가 공유한다 —
// 추출된 콜라이더는 Bone=None → IsWorldStatic()=true → detect 제외(push-out 전용).

#pragma once

#include "CoreMinimal.h"
#include "Templates/Function.h"
#include "Collision/RopeStaticCollider.h"

class UBodySetup;

namespace RopeBodyColliderExtraction
{
	/**
	 * Setup의 심플 콜리전을 타입별 out-배열에 월드 공간 콜라이더로 append한다. 예산(MaxColliders)은 세 배열의
	 * 합으로 세고, 초과하면 false를 반환한다(부분 추출). PrevCompTM/InvDeltaTime은 동적 바디 표면 속도용 —
	 * 정적이면 PrevCompTM=CompTM, InvDeltaTime=0을 넘긴다. MaxConvexPlanes 초과/미쿡 컨벡스는 ElemBox OBB로
	 * 폴백하며 OnConvexFallback(평면 수)로 통지한다(호출자 로깅용). 콜라이더는 Bone=None(push-out 전용)으로 만든다.
	 */
	DYNAMICROPE_API bool AppendBodyColliders(
		const UBodySetup& Setup, const FTransform& CompTM, const FTransform& PrevCompTM,
		float InvDeltaTime, int32 MaxColliders, int32 MaxConvexPlanes,
		TArray<FRopeBoxCollider>& OutBoxes,
		TArray<FRopeStaticCapsuleCollider>& OutCapsules,
		TArray<FRopeConvexCollider>& OutConvexes,
		const TFunctionRef<void(int32 NumPlanes)>& OnConvexFallback);
}
