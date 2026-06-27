// Copyright Epic Games, Inc. All Rights Reserved.
//
// SDF 볼륨 시각화 드로잉 헬퍼(에디터 전용, PDI 기반). 본별 SDF 볼륨을 bounds/grid/voxel/slice/gradient로
// 그린다. FRopeSDFVisualizer(레벨 에디터 컴포넌트 비주얼라이저)와 장차 오써링 패널 프리뷰 뷰포트가
// 동일 코드로 그리도록 분리. 모든 좌표는 본 로컬 → Xform으로 월드 배치한다.

#pragma once

#include "CoreMinimal.h"

struct FRopeBoneSDFVolume;
class FPrimitiveDrawInterface;
enum class ERopeSDFSliceAxis : uint8;

namespace RopeSDFDraw
{
	/** 로컬 AABB의 12개 모서리. */
	void DrawBounds(FPrimitiveDrawInterface* PDI, const FBox& LocalBounds, const FTransform& Xform, const FLinearColor& Color);

	/** 로컬 AABB 내부 coarse 격자(축당 Div 분할). */
	void DrawCoarseGrid(FPrimitiveDrawInterface* PDI, const FBox& LocalBounds, const FTransform& Xform, const FLinearColor& Color, int32 Div);

	/** 좁은밴드 voxel을 부호별 색 점으로(안=빨강, 밖=파랑, ≈0=흰색). |distance| <= Band 인 것만. */
	void DrawVoxels(FPrimitiveDrawInterface* PDI, const FRopeBoneSDFVolume& Volume, const FTransform& Xform, float Band);

	/** 한 축 슬라이스 평면의 distance heatmap(음=파랑, 0=흰, 양=빨강). Pos01 0~1, Res 격자, Scale(cm)에서 포화. */
	void DrawSlice(FPrimitiveDrawInterface* PDI, const FRopeBoneSDFVolume& Volume, const FTransform& Xform,
		ERopeSDFSliceAxis Axis, float Pos01, int32 Res, float Scale);

	/** 좁은밴드 샘플의 gradient(바깥쪽 = Query 법선) 방향을 화살표(선분)로. */
	void DrawGradients(FPrimitiveDrawInterface* PDI, const FRopeBoneSDFVolume& Volume, const FTransform& Xform, float Band, float Length);
}
