// Copyright Epic Games, Inc. All Rights Reserved.
//
// 던지기 초반의 "채찍 스윙" 연출 로직. 조준 반대편에서 시작해 조준 방향까지 스윕하는 가이드
// 곡선을 시간에 따라 회전시킨다. 일반 throw는 앞쪽 가이드 구간을 잡고, Aim-hit throw는 중앙을
// 강하게 잡되 손/자유단으로 갈수록 solver 상태와 부드럽게 섞는다. Flight 동안만 활성이다.
//
// 타깃 계산(스윕 각도 → 가이드 곡선 → 노드 간격 리샘플)은 게임 스레드에 남고, 적용은 CPU의
// ApplyToSim 또는 GPU resident override가 같은 프레임 산출물을 소비한다. CurrentTargets/
// PrevTargets/GuidedNodeMask가 두 경로의 공통 데이터 계약이다.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeTypes.h"

class DYNAMICROPE_API FRopeWhipGuide
{
public:
	/**
	 * 디자이너 설정 스냅샷. UPROPERTY 직렬화 경로를 지키기 위해 원본 프로퍼티는
	 * URopeComponent(Rope|Whip 카테고리)에 남고, 호출할 때마다 여기로 복사해 넘긴다.
	 */
	struct FConfig
	{
		/** 스윙 전체 시간(s). */
		float Duration = 0.35f;

		/** 가이드가 잡는 로프 길이 비율(0~1). */
		float GuidedLength = 0.65f;

		/** 시작 각도(조준 반대편)에서 조준 방향까지의 스윕 각. */
		float SweepAngleDegrees = 180.0f;

		/** 이 속도일 때 Duration 그대로 사용한다. */
		float ReferenceThrowSpeed = 1500.0f;

		/** 가이드 길이 산정용: max(Sim.RopeLength, 이 값) 사용. */
		float ComponentRopeLength = 0.0f;

		/** Aim-hit에서 손 쪽/자유단 쪽 guide 완화 구간과 hit direction 보간을 앞당기는 지수. */
		float AimHitRootSolverFraction = 0.20f;
		float AimHitTipSolverFraction = 0.25f;
		float AimHitDirectionBias = 2.0f;
	};

	struct FSwingBasis
	{
		FVector AimDir = FVector::ForwardVector;
		FVector GuideUp = FVector::UpVector;
		FVector GuideRight = FVector::RightVector;
	};

	/** 퇴화 벡터를 fallback으로 정규화한다. throw frame/swing basis 해석 공용. */
	static FVector SafeNormalOr(const FVector& Value, const FVector& Fallback);

	/** ThrowContext와 SwingPlane 설정을 WhipGuide가 실제로 쓰는 Aim/Up/Right 기준축으로 해석한다. */
	static FSwingBasis ResolveSwingBasis(const FRopeThrowContext& ThrowContext,
		ERopeSwingPlane SwingPlane, const FVector& CustomPlaneNormal);

	/**
	 * throw 시 호출: 조준 방향 기준의 가이드 좌표계(Forward/Up)를 구성하고 스윙을 활성화한다.
	 * Fallback* 벡터들은 퇴화 케이스(조준이 0이거나 수직에 가까울 때)에 쓸 컴포넌트 축.
	 */
	void Begin(const FVector& InAimDir, const FVector& InOrigin,
		const FVector& FallbackAim, const FVector& FallbackUp, const FVector& FallbackSide,
		float InThrowSpeed = 0.0f, const FVector& InInheritedVelocity = FVector::ZeroVector,
		bool bInHasAimTarget = false, const FVector& InAimTarget = FVector::ZeroVector,
		float InAimSteerStartAlpha = 0.25f, float InAimLockAlpha = 0.50f);

	/**
	 * throw 직후 초기 포즈(T=0) 스냅: 일반 가이드 구간은 타깃에 놓고, Aim-hit은 중앙만 강하게
	 * 배치하며 양끝 envelope는 기존 solver 위치와 섞는다. StartFreshThrow에서 1회 호출.
	 */
	void SnapToInitialPose(FRopeSimState& Sim, const FConfig& Config);

	/**
	 * 매 프레임(Flight, GT): Elapsed 전진 → 가이드 타깃/마스크 *계산만* 한다(Sim 불변).
	 * 적용은 두 갈래가 같은 산출물을 소비한다: CPU 솔브 경로는 ApplyToSim, GPU 상주 경로는
	 * override 패스(ERopeGPUOverride::Position|Prev — 서브시스템이 step에 실어 보냄).
	 * 스윙이 끝나면(Elapsed >= Duration) 스스로 비활성화된다.
	 * bCaptureDebugTargets가 참일 때만 디버그 배열(GetDebugGuide*)을 채운다(비용 절약).
	 */
	void Advance(float DeltaTime, const FRopeSimState& Sim, const FConfig& Config, bool bCaptureDebugTargets);

	/**
	 * CPU 경로의 적용 절반: Advance가 계산한 타깃/마스크를 Sim에 기록한다(가이드 노드만,
	 * Pos=현재 타깃 / Prev=직전 타깃 → 차이가 Verlet 속도). GPU 로프에는 호출하지 않는다.
	 */
	void ApplyToSim(FRopeSimState& Sim) const;

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
	void ResampleGuideByNodeSpacing(const TArray<FVector>& SourcePoints, float NodeSpacing,
		int32 DesiredPointCount, TArray<FVector>& OutPoints) const;

	bool bActive = false;
	float Elapsed = 0.0f;

	FVector AimDir = FVector::ForwardVector;
	FVector Origin = FVector::ZeroVector;
	FVector GuideForward = FVector::ForwardVector;
	FVector GuideUp = FVector::UpVector;
	FVector GuideInheritedVelocity = FVector::ZeroVector;
	float GuideThrowSpeed = 0.0f;

	/** Aim target은 노드 고정점이 아니라 최종 방향과 공간 보간 파라미터로만 보관한다. */
	bool bHasAimTarget = false;
	FVector AimTarget = FVector::ZeroVector;
	float AimSteerStartAlpha = 0.25f;
	float AimLockAlpha = 0.50f;

	/** 직전 프레임의 가이드 타깃(가이드 노드의 Verlet 속도 주입: PrevPositions ← 이 값). */
	TArray<FVector> PreviousTargets;

	//~ 프레임 산출물
	TArray<FVector> PrevTargetsThisFrame;
	TArray<FVector> CurrentTargetsThisFrame;
	TArray<uint8> GuidedNodesThisFrame;
	TArray<int32> DebugGuideNodeIndices;
	TArray<FVector> DebugGuideTargets;
};
