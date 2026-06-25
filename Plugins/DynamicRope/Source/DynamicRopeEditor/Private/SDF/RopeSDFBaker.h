// Copyright Epic Games, Inc. All Rights Reserved.
//
// URopeSDFData를 본별 signed distance 볼륨으로 채우는 에디터 전용 CPU 베이커.
// 스켈레탈 메시의 에디터 소스 모델(WITH_EDITOR)을 읽어, 삼각형을 스킨 가중치로 본에 배정하고,
// 본 로컬 공간으로 변환한 뒤, generalized winding number로 부호를 매긴 좁은밴드 SDF를 voxel화한다
// (닫히지 않은 본별 삼각형 패치에서도 강건한 부호 판정).
//
// 규약(런타임 FRopeSDFCollider::Query 샘플러와 반드시 일치): 샘플은 grid 코너에 놓인다.
// 즉 인덱스 (x,y,z)의 샘플 위치 = LocalBounds.Min + (x,y,z) * VoxelSize,
// LocalBounds.Max == Min + (Resolution - 1) * VoxelSize. 거리 단위 cm, 바깥쪽 양수.

#pragma once

#include "CoreMinimal.h"

class USkeletalMesh;
struct FRopeBoneSDFVolume;

/** 베이크 1회에 대한 디자이너용 설정 값. */
struct FRopeSDFBakeSettings
{
	/** 샘플 간격(cm, 큐브 voxel). 작을수록 표면이 선명해지고 메모리/시간이 늘어난다. */
	float VoxelSize = 1.5f;

	/** 축당 샘플 상한. 본 grid가 이를 넘으면 VoxelSize를 키워 맞춘다. */
	int32 MaxResolution = 48;

	/** |거리|를 이 밴드(cm)로 clamp. 밴드 밖 값은 충돌과 무관하다. */
	float NarrowBand = 6.0f;

	/** 삼각형을 본에 배정하기 위한 최소 평균 스킨 가중치 [0..1]. */
	float WeightThreshold = 0.2f;

	/** voxel화 전 본 삼각형 AABB를 확장(cm) — 스킨 바깥에도 밴드 여유를 둔다. */
	float BoundsPadding = 3.0f;
};

/** 무상태 본별 SDF 베이커. 에디터 전용(임포트 소스 모델 사용). */
class FRopeSDFBaker
{
public:
	/**
	 * 요청된 본마다 본 로컬 볼륨 하나를 OutVolumes에 굽는다.
	 *  - Bones가 비면 => 스킨 지오메트리가 있는 모든 본.
	 *  - 자격 삼각형이 없는 본은 조용히 건너뛴다.
	 * 메시에 CPU 지오메트리가 없을 때만(예: 쿡/스트립) false를 반환한다.
	 */
	static bool BakeMesh(USkeletalMesh* Mesh, const TArray<FName>& Bones,
		const FRopeSDFBakeSettings& Settings, TArray<FRopeBoneSDFVolume>& OutVolumes);
};
