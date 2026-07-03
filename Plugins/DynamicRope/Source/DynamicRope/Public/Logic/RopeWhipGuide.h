// Copyright Epic Games, Inc. All Rights Reserved.
//
// 던지기 초반의 "채찍 스윙" 연출 로직. 조준 반대편에서 시작해 조준 방향까지 스윕하는 가이드
// 곡선을 시간에 따라 회전시키고, 로프 앞부분(가이드 구간) 노드들을 그 곡선 위에 강제 배치한다.
// Flight 동안만 활성. 솔버/랩 컨트롤러와 같은 패턴의 UObject 비의존 클래스다.
//
// GPU 전환(M5 이후) 대비 설계: 타깃 *계산*(스윕 각도 → 가이드 곡선 → 노드 간격 리샘플)은
// 게임 스레드에 남고, *적용*(Positions/PrevPositions 덮어쓰기)은 추후 GPU 커널로 옮긴다.
// 그래서 프레임 산출물(CurrentTargets/PrevTargets/GuidedNodeMask)을 데이터 계약으로 노출한다
// — GPU 포팅 시 이 데이터를 업로드하고 Advance의 적용 루프만 커널로 대체하면 된다.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

class DYNAMICROPE_API FRopeWhipGuide
{
public:
	// 디자이너 설정 스냅샷. UPROPERTY 직렬화 경로를 지키기 위해 원본 프로퍼티는
	// URopeComponent(Rope|Whip 카테고리)에 남고, 호출할 때마다 여기로 복사해 넘긴다.
	struct FConfig
	{
		float Duration = 0.35f;           // 스윙 전체 시간(s)
		float GuidedLength = 0.65f;       // 가이드가 잡는 로프 길이 비율(0~1)
		float SweepAngleDegrees = 180.0f; // 시작 각도(조준 반대편)에서 조준 방향까지의 스윕 각
		float ComponentRopeLength = 0.0f; // 가이드 길이 산정용: max(Sim.RopeLength, 이 값) 사용
	};

	/**
	 * throw 시 호출: 조준 방향 기준의 가이드 좌표계(Forward/Up)를 구성하고 스윙을 활성화한다.
	 * Fallback* 벡터들은 퇴화 케이스(조준이 0이거나 수직에 가까울 때)에 쓸 컴포넌트 축.
	 */
	void Begin(const FVector& InAimDir, const FVector& InOrigin,
		const FVector& FallbackAim, const FVector& FallbackUp, const FVector& FallbackSide);

	/**
	 * throw 직후 초기 포즈(T=0) 스냅: 가이드 타깃을 계산해 프레임 산출물을 채우고, 가이드 구간
	 * 노드의 Positions/PrevPositions를 타깃에 스냅한다(속도 0). StartFreshThrow에서 1회 호출.
	 */
	void SnapToInitialPose(FRopeSimState& Sim, const FConfig& Config);

	/**
	 * 매 프레임(Flight, GT): Elapsed 전진 → 가이드 타깃/마스크 계산 → Sim에 적용.
	 * 스윙이 끝나면(Elapsed >= Duration) 스스로 비활성화된다.
	 * bCaptureDebugTargets가 참일 때만 디버그 배열(GetDebugGuide*)을 채운다(비용 절약).
	 */
	void Advance(float DeltaTime, FRopeSimState& Sim, const FConfig& Config, bool bCaptureDebugTargets);

	/** 예측 접촉용: 다음 프레임 시점(Elapsed + DeltaTime)의 가이드 타깃 미리보기(상태 불변). */
	void PreviewNextTargets(float DeltaTime, const FRopeSimState& Sim, const FConfig& Config,
		TArray<FVector>& OutTargets) const;

	/** 이 프레임의 산출물만 비운다(비활성 프레임에 stale 데이터가 남지 않도록). */
	void ResetFrameOutputs();

	bool IsActive() const { return bActive; }
	float GetElapsed() const { return Elapsed; }
	/** 정규화된 조준 방향(퇴화 시 fallback 적용 후). throw 임펄스 주입에도 쓰인다. */
	const FVector& GetAimDir() const { return AimDir; }

	//~ 프레임 산출물(데이터 계약) — SnapToInitialPose/Advance가 채우고 다음 갱신까지 유효.
	const TArray<FVector>& GetCurrentTargets() const { return CurrentTargetsThisFrame; }
	const TArray<FVector>& GetPrevTargets() const { return PrevTargetsThisFrame; }
	const TArray<uint8>& GetGuidedNodeMask() const { return GuidedNodesThisFrame; }
	bool IsGuidedNodeThisFrame(int32 NodeIndex) const
	{
		return GuidedNodesThisFrame.IsValidIndex(NodeIndex) && GuidedNodesThisFrame[NodeIndex] != 0;
	}

	//~ 디버그 캡처 산출물(Advance에 bCaptureDebugTargets를 줬을 때만 채워짐)
	const TArray<int32>& GetDebugGuideNodeIndices() const { return DebugGuideNodeIndices; }
	const TArray<FVector>& GetDebugGuideTargets() const { return DebugGuideTargets; }

private:
	/** NormalizedTime(0~1) 시점의 가이드 곡선을 만들고 노드 간격으로 리샘플해 타깃을 채운다. */
	void BuildGuideTargets(float NormalizedTime, int32 LastGuidedNode,
		const FRopeSimState& Sim, const FConfig& Config, TArray<FVector>& OutTargets) const;

	/** 원시 곡선 점들을 로프 노드 간격(세그먼트 길이)에 맞춰 등간격 리샘플한다. */
	void ResampleGuideByNodeSpacing(const TArray<FVector>& SourcePoints, float TotalLength, int32 NodeCount,
		int32 DesiredPointCount, float FallbackSegmentLength, TArray<FVector>& OutPoints) const;

	bool bActive = false;
	float Elapsed = 0.0f;

	FVector AimDir = FVector::ForwardVector;
	FVector Origin = FVector::ZeroVector;
	FVector GuideForward = FVector::ForwardVector;
	FVector GuideUp = FVector::UpVector;

	// 직전 프레임의 가이드 타깃(가이드 노드의 Verlet 속도 주입: PrevPositions ← 이 값).
	TArray<FVector> PreviousTargets;

	//~ 프레임 산출물
	TArray<FVector> PrevTargetsThisFrame;
	TArray<FVector> CurrentTargetsThisFrame;
	TArray<uint8> GuidedNodesThisFrame;
	TArray<int32> DebugGuideNodeIndices;
	TArray<FVector> DebugGuideTargets;
};
