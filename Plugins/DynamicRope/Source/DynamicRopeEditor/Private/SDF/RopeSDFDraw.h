// Copyright Epic Games, Inc. All Rights Reserved.
//
// SDF 볼륨 시각화 드로잉 헬퍼(에디터 전용, PDI 기반). 본별 SDF 볼륨을 bounds/grid/voxel/slice/gradient로
// 그린다. 오써링 패널 프리뷰 뷰포트가 이 헬퍼로 그린다. 모든 좌표는 본 로컬 → Xform으로 월드 배치한다.

#pragma once

#include "CoreMinimal.h"

struct FRopeBoneSDFVolume;
class FPrimitiveDrawInterface;
enum class ERopeSDFSliceAxis : uint8;

namespace RopeSDFDraw
{
	/** 로컬 AABB의 12개 모서리. */
	void DrawBounds(FPrimitiveDrawInterface* PDI, const FBox& LocalBounds, const FTransform& Xform, const FLinearColor& Color);

	/**
	 * 좁은밴드 voxel을 부호별 색 점으로(안=빨강, 밖=파랑, ≈0=흰색 — slice heatmap과 동일 규약). |distance| <= Band 인 것만.
	 * NarrowBand > 0이면 |distance| >= NarrowBand 인 포화 샘플은 스킵한다 — 그 영역은 ±NarrowBand로
	 * 클램프된 placeholder라, Band를 최댓값까지 올렸을 때 plateau가 통째로 들어와 바운드 박스를 채우며
	 * 튀어 보이기 때문(도메인 확인은 Bounds 오버레이가 담당). 0이면 비활성.
	 */
	void DrawVoxels(FPrimitiveDrawInterface* PDI, const FRopeBoneSDFVolume& Volume, const FTransform& Xform, float Band, float NarrowBand);

	/**
	 * 한 축 슬라이스 평면의 distance heatmap(음=빨강, 0=흰, 양=파랑). Pos01 0~1, Res 격자, Scale(cm)에서 포화.
	 * NarrowBand > 0이면 |distance| >= NarrowBand 인 포화 샘플은 heatmap 대신 흐린 회색으로 그린다 —
	 * 그 영역은 ±NarrowBand로 클램프된 무의미 plateau라, 유의미한 밴드(표면·연속장)와 구분해 보여준다
	 * (Voxels/Gradients의 포화 스킵과 동일 규약). 0이면 비활성 → 기존대로 전체를 heatmap으로 채운다.
	 */
	void DrawSlice(FPrimitiveDrawInterface* PDI, const FRopeBoneSDFVolume& Volume, const FTransform& Xform,
		ERopeSDFSliceAxis Axis, float Pos01, int32 Res, float Scale, float NarrowBand);

	/**
	 * 좁은밴드 샘플의 gradient(바깥쪽 = Query 법선) 방향을 화살표로. |distance| <= Band 인 것만 그린다.
	 * NarrowBand > 0이면 |distance| >= NarrowBand 인 포화 샘플은 스킵한다 — 그 영역은 ±NarrowBand로
	 * 클램프돼 방향 정보가 없어(평탄 → up 폴백, 경계 → 노이즈) 가짜 화살표만 만들기 때문. 0이면 비활성.
	 */
	void DrawGradients(FPrimitiveDrawInterface* PDI, const FRopeBoneSDFVolume& Volume, const FTransform& Xform,
		float Band, float Length, float NarrowBand);
}
