// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

class USceneComponent;
struct FRopeSimState;

/**
 * Flight 접촉 후보의 출처(비트 조합 가능 — 같은 노드×본이 여러 경로로 잡히면 SourceMask에 OR).
 * Actual = 이번 프레임 이동 경로(Prev→Pos)의 실제 스윕 접촉,
 * PredictiveFree = 자유 노드의 관성 외삽 예측 접촉, PredictiveGuided = whip 가이드 타깃 외삽 예측 접촉.
 */
enum class ERopeContactCandidateSource : uint8
{
	Actual = 1,
	PredictiveFree = 2,
	PredictiveGuided = 4
};

/** Flight/Contacting이 소비하는 접촉 후보 1건(노드×본). FRopeContact + 상대운동 평가 산출물. */
struct FRopeContactCandidate
{
	bool bValid = false;
	int32 NodeIndex = INDEX_NONE;
	FName Bone = NAME_None;
	const USceneComponent* Mesh = nullptr;
	ERopeContactCandidateSource Source = ERopeContactCandidateSource::Actual;
	uint8 SourceMask = static_cast<uint8>(ERopeContactCandidateSource::Actual);

	FVector WorldPoint = FVector::ZeroVector;
	FVector Normal = FVector::UpVector;
	FVector SurfaceVelocity = FVector::ZeroVector;

	float Penetration = 0.0f;
	float RelativeTangentialSpeed = 0.0f;

	/** 감김 방향이면 +, 반대면 -. */
	float WrapDirectionScore = 0.0f;
};

/**
 * 캡처(Flight→Contacting) 순간의 로프 진행 좌표계 스냅샷(진행 방향 기반 wrap 2단계).
 * Contacting부터는 솔브가 없어 노드가 정지하므로, "로프가 어느 방향으로 날아와 어떻게 누웠는가"는
 * 이 순간에만 잴 수 있다 — BuildContactingState가 채우고 ResetTransientPhaseState가 폐기한다.
 * 소비자: CaptureTravelPlane 축(가이드 평면이 없는 던지기의 폴백 normal, 3단계에서 축 origin으로
 * RegionCenter 사용 예정). GPU 상주 로프는 CPU 미러가 1~2프레임 낡을 수 있으나 방향 성분은 충분하다.
 */
struct DYNAMICROPE_API FRopeCaptureTravelFrame
{
	bool bValid = false;

	/** 접촉 후보 표면점(WorldPoint)들의 평균 — 접촉 영역 중심(월드). */
	FVector RegionCenter = FVector::ZeroVector;

	/** 접촉 노드들의 평균 Verlet 속도(cm/s). dt<=0이면 Zero. */
	FVector AverageVelocity = FVector::ZeroVector;

	/** 접촉 span의 head→tail 단위 방향(로프가 누운 방향). 접촉이 한 노드뿐이면 이웃 노드로 넓혀 잰다. */
	FVector SpanDirection = FVector::ZeroVector;

	/** AverageVelocity × SpanDirection 유도 성공 여부(속도 0/평행이면 false — 자연 폴백 신호). */
	bool bHasPlaneNormal = false;

	/** 유도된 진행 평면 normal(단위). 부호는 소비자(winding/OrientAxisByTail)가 해석한다. */
	FVector PlaneNormal = FVector::ZeroVector;

	void Reset()
	{
		*this = FRopeCaptureTravelFrame();
	}

	/** 캡처 순간의 Sim/후보에서 스냅샷을 계산한다(UObject-free). 구현은 RopeTypes.cpp. */
	static FRopeCaptureTravelFrame Compute(const FRopeSimState& Sim,
		const TArray<FRopeContactCandidate>& Candidates, float DeltaTime);
};

/** 트래커가 dominant 외에도 유지하는 (Mesh, Bone) 대상별 접촉 집계(시드 다중화 재료). */
struct FRopeTrackedContactTarget
{
	FName Bone = NAME_None;
	const USceneComponent* Mesh = nullptr;

	/** 이번 프레임 이 대상에 닿은 노드들(매 갱신 최신 후보로 교체). */
	TArray<int32> Nodes;

	/** 이 대상의 지속 접촉 시간. 접촉이 끊긴 프레임에는 같은 양만큼 감쇠하고 0이 되면 목록에서 빠진다. */
	float DwellTime = 0.0f;
};

/**
 * 접촉 후보들에서 dominant 대상 — (Mesh, Bone) 쌍 — 을 추적하는 POD 트래커. 본 이름만 키로 쓰면
 * 같은 스켈레톤을 쓰는 두 액터가 동시에 닿을 때 후보가 합산/오귀속되므로 mesh까지 키에 포함한다.
 * Flight의 캡처 판정(ShouldCapture)과 Contacting의 체류 추적이 공용으로 쓴다.
 * 동률은 head(손 쪽) 노드가 앞선 대상 → 점수(관통+감김 방향) 순으로 깨져 프레임 간 안정적이다.
 * 대상이 바뀌면(본 또는 mesh) DwellTime이 0부터 다시 쌓인다(전이 프레임 오탐 방어 — 랙돌 테스트 (c)가
 * 고정하는 계약).
 * dominant와 별개로 접촉 중인 모든 (Mesh, Bone) 대상을 Targets에 dwell과 함께 유지한다 —
 * 시드 다중화(MaxWrapSeeds > 1)가 보조 시드 후보를 고르는 재료다. dominant 선정/리셋 계약은
 * Targets 도입과 무관하게 종전과 동일하다.
 */
struct DYNAMICROPE_API FRopeContactTracker
{
	FName CandidateBone = NAME_None;
	const USceneComponent* CandidateMesh = nullptr;

	TArray<int32> CandidateNodes;
	float DwellTime = 0.0f;

	/** 접촉 중인 모든 대상의 (Mesh, Bone)별 집계. dominant도 포함된다(같은 키로 조회 가능). */
	TArray<FRopeTrackedContactTarget> Targets;

	void Reset()
	{
		CandidateBone = NAME_None;
		CandidateMesh = nullptr;
		CandidateNodes.Reset();
		DwellTime = 0.0f;
		Targets.Reset();
	}

	UE_DEPRECATED(5.7, "Use Update(Candidates, DeltaTime) instead.")
	void BeginOrUpdate(const TArray<FRopeContactCandidate>& Candidates)
	{
		Update(Candidates, 0.0f);
	}

	void Decay(float DeltaTime)
	{
		DwellTime = FMath::Max(0.0f, DwellTime - DeltaTime);

		// 접촉이 전무한 프레임: 모든 대상의 dwell을 같은 비율로 소진시킨다(짧은 플리커 관용은 동일).
		// 노드 목록은 이번 프레임 접촉이 아니므로 비워 stale 소비를 막는다.
		for (int32 Index = Targets.Num() - 1; Index >= 0; --Index)
		{
			Targets[Index].DwellTime -= DeltaTime;
			Targets[Index].Nodes.Reset();
			if (Targets[Index].DwellTime <= 0.0f)
			{
				Targets.RemoveAt(Index);
			}
		}

		if (DwellTime <= 0.0f)
		{
			Reset();
		}
	}

	// [Assisted 멀티 본 계약] Targets에는 모든 허용 본을 보존하면서 CandidateBone만 조준 본으로
	// 고정할 수 있어야 한다. 그래서 일반 rank를 덮어쓰는 preferred target 입력을 이 공용 tracker에 둔다.
	/** 후보를 (Mesh, Bone) 쌍별 집계해 dominant 대상/노드/체류 시간을 갱신한다.
	 *  Preferred target이 현재 후보에 있으면 일반 rank보다 우선한다. bRequirePreferred인데 없으면
	 *  dominant를 비우되 Targets의 secondary 집계는 유지한다. 구현은 RopeTypes.cpp. */
	void Update(const TArray<FRopeContactCandidate>& Candidates, float DeltaTime,
		const USceneComponent* PreferredMesh = nullptr, FName PreferredBone = NAME_None,
		bool bRequirePreferred = false);
};
