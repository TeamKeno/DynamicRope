// Copyright Epic Games, Inc. All Rights Reserved.
//
// URopeSDFData를 본별 signed distance 볼륨으로 채우는 에디터 전용 CPU 베이커.
// 스켈레탈 메시의 에디터 소스 모델(WITH_EDITOR)을 읽어, 삼각형을 스킨 가중치로 본에 배정하고,
// 본 로컬 공간으로 변환한 뒤 좁은밴드 SDF를 voxel화한다. 거리(unsigned)는 본별 삼각형으로 재서 본 귀속을
// 유지하되, 부호(안/밖)는 메시 전체(닫힌 표면)에 대한 generalized winding number(GeometryCore fast
// winding)로 매긴다 — 본별 열린 패치로 적분하면 짧고 넓은 본 토막의 내부가 w<0.5로 바깥 오판되므로,
// 전역 메시로 적분해야 강건하다.
//
// 규약(런타임 FRopeSDFCollider::Query 샘플러와 반드시 일치): 샘플은 grid 코너에 놓인다.
// 즉 인덱스 (x,y,z)의 샘플 위치 = LocalBounds.Min + (x,y,z) * VoxelSize,
// LocalBounds.Max == Min + (Resolution - 1) * VoxelSize. 거리 단위 cm, 바깥쪽 양수.

#pragma once

#include "CoreMinimal.h"
// FRopeSDFBakeProgress(TFunction) 진행 콜백
#include "Templates/Function.h"
// FRopeSDFBakeSettings(런타임 USTRUCT, 에셋에 저장)
#include "Collision/SDF/RopeSDFData.h"

class USkeletalMesh;
struct FRopeBoneSDFVolume;

// FRopeSDFBakeSettings는 런타임 모듈(RopeSDFData.h)로 승격되었다 — 베이크 설정을 에셋에 함께
// 저장(URopeSDFData::LastBakeSettings)해 재오써링 시 비교 기준으로 쓰기 위함. 여기서는 그 타입을
// 그대로 입력으로 받는다.

/**
 * 본 단위 진행/취소 콜백. 타깃 본 하나의 voxel화를 시작할 때마다 한 번 호출된다.
 *  - Done : 지금 시작하는 본의 0-기반 순번(0..Total).
 *  - Total: 전체 타깃 본 수.
 *  - Bone : 지금 시작하는 본 이름.
 * 반환값이 false면 베이크를 즉시 중단한다(true=계속). 비어 있으면(기본) 보고/취소 없이 끝까지 굽는다.
 */
using FRopeSDFBakeProgress = TFunction<bool(int32 /*Done*/, int32 /*Total*/, const FName& /*Bone*/)>;

/**
 * 베이크 도중 자주(본 내부 voxel 배치 사이마다) 호출되는 취소 폴. true면 즉시 중단한다.
 * 무거운 본의 voxel화가 게임 스레드를 오래 점유하지 않도록, 배치 사이에서 이 폴을 통해 슬로우 태스크
 * UI를 펌프하고 취소 버튼 입력을 처리한다. 비어 있으면(기본) 본 단위 취소(Progress 반환값)만 동작한다.
 */
using FRopeSDFBakeCancelPoll = TFunction<bool()>;

/** BakeMesh 결과. */
enum class ERopeSDFBakeResult : uint8
{
	// 정상 완료(결과가 0개 본일 수도 있음).
	Success,
	// CPU 지오메트리 없음(쿡/스트립) 또는 null 메시 — 베이크 불가.
	NoGeometry,
	// 진행 콜백이 중단 요청 — OutVolumes는 미완성이므로 자산에 반영하지 말 것.
	Cancelled,
};

/**
 * 요청 VoxelSize가 MaxResolution 상한 때문에 키워진(coarsen된) 본 하나의 기록.
 * 사용자가 "내가 넣은 간격이 왜 더 굵게 구워졌나"를 Message Log로 확인할 수 있게 한다.
 */
struct FRopeSDFCoarsenedBone
{
	// 해당 본 이름
	FName      Bone;
	// 사용자가 입력한 S.VoxelSize(cm)
	float      RequestedVoxelSize;
	// 상한에 맞추느라 키워진 실제 voxel 크기(cm)
	float      ActualVoxelSize;
	// 최종 grid 해상도(축별 샘플 수)
	FIntVector Resolution;
};

/** BakeMesh 한 번의 집계 통계(에디터 보고용, 에셋에 저장하지 않는 plain 타입). */
struct FRopeSDFBakeStats
{
	// 실제로 볼륨이 구워진 본 수
	int32 BonesBaked = 0;
	// coarsening이 발생한 본만 기록
	TArray<FRopeSDFCoarsenedBone> CoarsenedBones;
	// girth < MinBoneGirth 로 제외(drop)된 본
	TArray<FName> DroppedThinBones;
};

/** 무상태 본별 SDF 베이커. 에디터 전용(임포트 소스 모델 사용). */
class FRopeSDFBaker
{
public:
	/**
	 * 요청된 본마다 본 로컬 볼륨 하나를 OutVolumes에 굽는다.
	 *  - Bones가 비면 => 스킨 지오메트리가 있는 모든 본.
	 *  - 자격 삼각형이 없는 본은 조용히 건너뛴다.
	 *  - Progress가 있으면 본 하나를 처리하기 직전마다 호출한다(진행률 표시 + 본 단위 취소, 옵션).
	 *  - CancelPoll이 있으면 본 내부 voxel 배치 사이마다 호출해 무거운 본 도중에도 취소를 받는다(옵션).
	 *    Progress나 CancelPoll이 취소를 신호하면 Cancelled를 반환하며 OutVolumes는 미완성 상태로 남는다.
	 *  - OutStats가 있으면 구워진 본 수와 coarsening이 발생한 본 목록을 채운다(옵션, 보고용).
	 * 메시에 CPU 지오메트리가 없으면(예: 쿡/스트립) NoGeometry를 반환한다.
	 */
	static ERopeSDFBakeResult BakeMesh(USkeletalMesh* Mesh, const TArray<FName>& Bones,
		const FRopeSDFBakeSettings& Settings, TArray<FRopeBoneSDFVolume>& OutVolumes,
		const FRopeSDFBakeProgress& Progress = FRopeSDFBakeProgress(),
		const FRopeSDFBakeCancelPoll& CancelPoll = FRopeSDFBakeCancelPoll(),
		FRopeSDFBakeStats* OutStats = nullptr);
};
