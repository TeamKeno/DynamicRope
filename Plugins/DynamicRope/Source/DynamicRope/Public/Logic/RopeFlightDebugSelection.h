// Copyright Epic Games, Inc. All Rights Reserved.
//
// Flight 디버거의 후보 박스 선택 — 순수 함수라 월드/UObject 없이 단위 테스트한다. 디버거 그리기
// (WITH_GAMEPLAY_DEBUGGER)와 자동화 테스트가 공유한다. 스냅샷이 이미 모은 배열만 소비하고 아무것도
// 추가로 수집하지 않는다. Mesh 식별은 raw 포인터가 아니라 FObjectKey로 — 스냅샷 수명(수 프레임) 뒤
// 포인터가 죽어 있을 수 있어 비교/해석 모두 키로 한다.

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectKey.h"
#include "Core/RopeContactTrackingTypes.h"

namespace RopeFlightDebug
{
	// 후보 박스 선택 결과. 인덱스는 입력 Candidates 배열의 원본 인덱스다.
	struct FCandidateSelection
	{
		// 포착 대상(tracker와 (Mesh,Bone)이 일치하는 대표) 후보의 원본 인덱스. 없으면 INDEX_NONE.
		int32 CaptureTargetIndex = INDEX_NONE;
		// 그릴 박스들의 원본 인덱스. 대표가 있으면 [0]이 대표, 그 뒤가 penetration 상위 일반 후보.
		TArray<int32> BoxIndices;
		// 유효(bValid) 후보 총수 — 요약의 candidates=.
		int32 TotalValid = 0;
		// 실제 그리는 박스 수(BoxIndices.Num())와, 그중 제외된 유효 후보 수.
		int32 Shown = 0;
		int32 Hidden = 0;
	};

	/**
	 * 그릴 후보 박스를 고른다. 대표(포착 대상)는 penetration top-N 밖이어도 항상 포함한다 — "곧 무엇에
	 * 걸리려 하나"가 predictive 대표여서 penetration이 낮을 때가 정작 가장 보고 싶은 순간이기 때문.
	 *  - MeshKeys: Candidates와 1:1(같은 인덱스). 짧으면(구 스냅샷) 그 인덱스는 본만으로 tracker와 대조한다.
	 *  - MaxBoxes: 그릴 박스 상한(기본 [U]=1, [U]+[K]=5).
	 *  - bFillWithGeneral: true면 남는 칸을 penetration 상위 일반 후보로 채운다(Advanced). false면 대표만(기본).
	 * 순위 규칙: penetration 내림차순, 동률이면 NodeIndex 오름차순, 그래도 동률이면 원본 인덱스 오름차순 — 결정적.
	 */
	DYNAMICROPE_API FCandidateSelection SelectCandidateBoxes(
		const TArray<FRopeContactCandidate>& Candidates,
		const TArray<FObjectKey>& MeshKeys,
		FName TrackerBone,
		FObjectKey TrackerMeshKey,
		int32 MaxBoxes,
		bool bFillWithGeneral);
}
