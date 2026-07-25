// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "Core/RopeWrappingTypes.h"
#include "RopeThrowTypes.generated.h"

class USceneComponent;

/** 던질 때 기준축을 어느 좌표계에서 가져올지. */
UENUM(BlueprintType)
enum class ERopeThrowFrameMode : uint8
{
	World = 0 UMETA(DisplayName = "World"),
	Owner = 1 UMETA(DisplayName = "Owner"),
	OwnerCamera = 3 UMETA(DisplayName = "Owner Camera"),
	Socket = 2 UMETA(DisplayName = "Socket"),
	Custom = 4 UMETA(DisplayName = "Custom")
};

/** AimDir과 조합해 스윙 호가 놓일 평면/방향을 고르는 5개 드롭다운. */
UENUM(BlueprintType)
enum class ERopeSwingPlane : uint8
{
	AimAndFrameUp = 0 UMETA(DisplayName = "Aim + Frame Up"),
	AimAndFrameDown = 1 UMETA(DisplayName = "Aim + Frame Down"),
	AimAndFrameRight = 2 UMETA(DisplayName = "Aim + Frame Right"),
	AimAndFrameLeft = 3 UMETA(DisplayName = "Aim + Frame Left"),
	CustomNormal = 4 UMETA(DisplayName = "Custom Plane Normal")
};

// 아래 정의 — MakeDefault가 설정 스냅샷으로 받는다.
struct FRopeThrowParams;

/** throw 순간 Wielder/Component가 계산해 넘기는 런타임 값. 설정값(FRopeThrowParams)과 분리한다. */
USTRUCT(BlueprintType)
struct DYNAMICROPE_API FRopeThrowContext
{
	GENERATED_BODY()

	/**
	 * 컴포넌트 트랜스폼 + 던지기 설정에서 기본 컨텍스트를 조립한다(throw당 1회, GT).
	 * URopeComponent::Throw() 편의 진입점의 기본 구현이 사용한다 — Wielder처럼 컨텍스트를
	 * 직접 만드는 호출자는 무관. 프레임 기저 규약(FrameMode별):
	 *   World = 월드 축 · Owner/Socket = 컴포넌트 기저 · OwnerCamera = owner의 첫 카메라
	 *   (없으면 컴포넌트 기저 폴백) · Custom = Params의 커스텀 축(원값 — 정규화/직교
	 *   폴백은 ResolveThrowContext 책임). 구현은 RopeTypes.cpp.
	 */
	static FRopeThrowContext MakeDefault(const USceneComponent& RopeComponent, const FRopeThrowParams& Params);

	// 이 struct는 throw 순간 계산되는 런타임 스냅샷이라 저장형 UPROPERTY 멤버로 노출되지 않는다 —
	// 디테일 패널 편집(EditAnywhere)은 매 던지기마다 덮어써져 의미가 없다. BP Make/Break로 컨텍스트를
	// 조립하는 호출자를 위해 BlueprintReadWrite만 남긴다.
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw")
	FVector Origin = FVector::ZeroVector;

	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw")
	FVector FrameForward = FVector::ForwardVector;

	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw")
	FVector FrameUp = FVector::UpVector;

	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw")
	FVector FrameRight = FVector::RightVector;

	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw")
	FVector OwnerVelocity = FVector::ZeroVector;

	/** 손 소켓의 애니메이션 상대 속도(월드, 캐릭터 이동 제외) — 정지 상태에서 팔만 휘둘러도 그 스윙이
	 *  던지기에 실린다. Wielder가 컴포넌트-로컬 소켓 위치 델타로 측정해 채운다(GetPhysicsLinearVelocity의
	 *  물리 바디 의존/역방향 문제를 피한다). 소켓 추적이 없는 경로(BP 직접 Throw)는 0. */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw")
	FVector HandAnimationVelocity = FVector::ZeroVector;

	/** Wielder가 소유하는 던지기 속도. 0 이하이면 RopeComponent의 fallback 값을 사용한다. */
	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw", meta = (ClampMin = "0.0"))
	float ThrowSpeed = 0.0f;

	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw")
	ERopeThrowFrameMode FrameMode = ERopeThrowFrameMode::Owner;

	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw")
	ERopeSwingPlane SwingPlane = ERopeSwingPlane::AimAndFrameUp;

	UPROPERTY(BlueprintReadWrite, Category = "Rope|Throw")
	FVector CustomSwingPlaneNormal = FVector::RightVector;

	/** 이 컨텍스트를 조준 ray 경로가 만들었는지 나타낸다 — hit/miss와 무관하게 참이다.
	 *  아래 bHasAimGuideHit이 false인 두 상황을 가르는 유일한 표시다: "조준했는데 빗나감"(true)
	 *  vs "조준 자체가 없음"(false — Wielder 없는 BP 직행/AI의 Throw()). preview 빌더가 이걸로
	 *  arc 재탐색 허용 여부를 정한다 — 조준이 빗나간 경우까지 arc로 대상을 주우면 조준하지 않은
	 *  옆 대상에 꽂혀 ③ 계약("보장 = 조준한 대상")이 깨진다. */
	bool bAimRayEvaluated = false;

	/** aim ray가 유효한 본 hit을 확보했는지 나타낸다. */
	bool bHasAimGuideHit = false;

	/** Aim ray가 고른 primary 대상 본. Assisted에서는 첫 캡처/dominant만 이 본으로 고정하고,
	 *  같은 mesh의 다른 본은 multi-bone 후보로 허용한다. Guaranteed에서는 exact target으로 유지된다. */
	FName AimGuideBone = NAME_None;

	/** 대상 본의 transform과 SDF를 해석할 mesh/component이다. */
	TWeakObjectPtr<const USceneComponent> AimGuideMesh = nullptr;

	/** ray 중심선이 처음 target SDF 안으로 들어간 월드 위치이다. */
	FVector AimGuideHitWorldPos = FVector::ZeroVector;

	/** 조준 hit을 대상 본(AimGuideBone) 기준으로 표현한 로컬 위치이다. 조준 순간의 월드 hit을 그때의
	 *  본 트랜스폼으로 역변환해 저장한다. 대상이 움직여도 소비 시점의 본 트랜스폼으로 현재 월드를
	 *  복원하면(대칭) 팁이 조준한 신체 지점을 그대로 따라간다 — AimGuideHitWorldPos(월드 고정)는
	 *  대상 이동을 반영하지 못해 커밋 시 팁이 허공에 뜨는 문제가 있었다. */
	FVector AimGuideLocalHitPos = FVector::ZeroVector;

	/** 위 AimGuideLocalHitPos가 유효한 본-로컬 값을 담고 있는지 나타낸다. false면 소비부가
	 *  AimGuideHitWorldPos(월드) 폴백을 사용한다 — 조준 본/mesh가 없거나 구 컨텍스트 하위호환용. */
	bool bHasAimGuideLocalHit = false;

	/** 표면점에서 얻은 바깥쪽 법선이다. */
	FVector AimGuideNormal = FVector::UpVector;

	/**
	 * 로프 길이상 hit 방향 보간을 시작/완료할 구간이다.
	 * 공간 보간은 Flight 시간 보간과 곱하므로 throw가 끝나기 전에 hit 방향에 고정되는 노드는 없다.
	 */
	float AimGuideSteerStartAlpha = 0.25f;
	float AimGuideLockAlpha = 0.50f;
};

/** Runtime centerline data for the pre-wrapped rope preview. */
USTRUCT(BlueprintType)
struct FRopeWrapPreviewData
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview")
	TArray<FVector> Points;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview", meta = (ClampMin = "0.1", Units = "cm"))
	float Radius = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Preview", meta = (ClampMin = "3", ClampMax = "32"))
	int32 NumSides = 8;

	bool IsValid() const
	{
		return Points.Num() >= 2 && Radius > KINDA_SMALL_NUMBER;
	}
};

/** flight 단계의 Throw / launch 파라미터. */
USTRUCT(BlueprintType)
struct FRopeThrowParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	ERopeThrowFrameMode FrameMode = ERopeThrowFrameMode::Owner;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw")
	ERopeSwingPlane SwingPlane = ERopeSwingPlane::AimAndFrameUp;

	/** Wielder를 거치지 않고 RopeComponent::Throw를 직접 호출할 때 쓰는 fallback 속도. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw", meta = (ClampMin = "1.0", UIMin = "100.0", UIMax = "5000.0", Units = "cm/s", DisplayName = "Throw Speed"))
	float ThrowSpeed = 1500.0f;

	/**
	 * 던지기 속도 주입의 팁 부스트 배율(1 = 균등, >1 = 끝으로 갈수록 세게 — 채찍처럼 끝이 앞서 나감).
	 * 질량이 아니다: 솔버에는 어떤 질량도 반영되지 않고(2026-07-13 회의 — 팁 질량 솔버 무반영),
	 * Throw 순간 Verlet 속도 분배에만 쓰는 연출 배율이다. 종전 이름 TipMass(기본 5 = 1배라는 숨은
	 * 정규화)에서 개명·정규화(표면 감사 B-2) — 이제 값이 곧 배율이고 소비처에서 [0.25, 3]으로 클램프.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw|Tuning", meta = (ClampMin = "0.25", ClampMax = "3.0"))
	float TipVelocityBoost = 1.0f;

	/**
	 * ③ GuidedThrow 비행 아치의 정점 높이 = 손→목표 거리 × 이 비율. 팁이 위쪽 포물선을 그리며 목표
	 * (꽂힘 지점 또는 허공 던지기의 레이 끝점)에 도달한다. 0 = 아치 없음(장전 포즈 → 목표로 곧장 보간).
	 * 아치 오프셋은 팁으로 갈수록 선형으로 커진다. Alpha=1에서 오프셋 0이라 착지 지점은 정확히 유지된다.
	 * 상한 0.5 = 정점이 손→목표 거리의 절반까지. 그 이상은 목표로 날아간다기보다 위로 쏘아 올렸다
	 * 떨어지는 궤적이 되어, 조준선과 실제로 보이는 비행이 눈에 띄게 어긋난다.
	 * BP 런타임 쓰기는 이 meta를 우회하므로 소비처에서도 같은 범위로 클램프한다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw|Tuning",
		meta = (ClampMin = "0.0", ClampMax = "0.5"))
	float GuidedThrowArcHeightRatio = 0.25f;

	/**
	 * 던질 때 캐릭터 이동 속도를 로프에 상속시키는 배율("관성 과장" 게임필). 물리 사실값은 1이지만 기본 5 —
	 * 달리며 던질 때 로프가 눈에 띄게 앞서 나간다. 0 = 상속 없음(제자리 던지기와 동일).
	 * 손 소켓의 애니메이션 스윙(캐릭터 이동을 뺀 손 상대 속도)은 이 배율과 무관하게 항상 1배 실린다 —
	 * 소켓 월드 속도가 이미 캐릭터 이동을 포함하므로 그 몫을 빼(ComputeThrowInheritedVelocity) 이중 반영을 막는다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw|Tuning", meta = (ClampMin = "0.0", DisplayName = "Motion Inheritance"))
	float MotionInheritance = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw|Tuning", meta = (EditCondition = "FrameMode == ERopeThrowFrameMode::Custom"))
	FVector CustomFrameForward = FVector::ForwardVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw|Tuning", meta = (EditCondition = "FrameMode == ERopeThrowFrameMode::Custom"))
	FVector CustomFrameUp = FVector::UpVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw|Tuning", meta = (EditCondition = "FrameMode == ERopeThrowFrameMode::Custom"))
	FVector CustomFrameRight = FVector::RightVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Throw|Tuning", meta = (EditCondition = "SwingPlane == ERopeSwingPlane::CustomNormal"))
	FVector CustomSwingPlaneNormal = FVector::RightVector;
};

/** 던지기 초반 채찍 스윙(FRopeWhipGuide) 튜닝 값. 런타임 상태는 URopeComponent::WhipGuide가 소유한다.
 *  스윙 시간은 Throw Speed에서 파생한다 — 내부 기준(1500cm/s에서 0.35s)으로
 *  EffectiveDuration = 0.35 × 1500 / ThrowSpeed(MakeWhipGuideConfig + RopeWhipGuide::ResolveGuideDuration).
 *  빠를수록 짧게 휘둘러, Throw Speed 하나가 휘두름·비행 세기를 함께 정한다(별도 duration 노브 없음). */
USTRUCT(BlueprintType)
struct FRopeWhipConfig
{
	GENERATED_BODY()

	/** 가이드가 잡는 로프 길이 비율(0~1) — 스윙 궤적 형태 조절. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip", meta = (ClampMin = "0.1", ClampMax = "0.95"))
	float GuidedLength = 0.65f;

	/** 시작 각도(조준 반대편)에서 조준 방향까지의 스윕 각. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip", meta = (ClampMin = "1.0", ClampMax = "180.0", Units = "deg", DisplayName = "Sweep Angle"))
	float SweepAngleDegrees = 180.0f;

	/**
	 * Aim-hit Flight에서 손 쪽 guide를 solver에 넘기는 로프 길이 비율. 0이면 중앙 spline이 손 바로 옆까지 지배한다.
	 * 같은 값이 손 소켓 오프셋을 뿌리 구간에 섞는 범위이기도 해서(FRopeWhipGuide의 AimRootSocketInfluence),
	 * 0에서는 guide가 던진 순간의 Origin에 고정돼 손 애니메이션을 따라가지 않는다.
	 * 디자이너 노출 없이 이 기본값으로 고정한다. 소비처는 0~0.45로 클램프한다.
	 */
	float AimHitRootSolverFraction = 0.20f;

	/**
	 * Hit direction 보간 편향. 1은 선형 강도, 클수록 같은 Flight 시점에서 spline이 더 빨리 hit 방향을 향한다.
	 * RopeMath::BuildWhipGuideRawPoints의 aim-hit 분기는 직선 spline이라 이 값을 읽지 않는다.
	 * 그 분기에 곡선 보간을 넣을 때 함께 살릴 자리로 남겨둔다 — 디자이너 노출 없음.
	 */
	float AimHitDirectionBias = 2.0f;

	/** Aim-hit Flight에서 자유단 쪽 guide를 solver에 넘기는 로프 길이 비율. 클수록 끝이 더 관성적으로 움직인다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip|Tuning|Aim Hit", meta = (ClampMin = "0.0", ClampMax = "0.45"))
	float AimHitTipSolverFraction = 0.25f;

	/** Aim-hit Flight에서 거리/굽힘/감쇠 solver는 유지하고 collider push-out만 끈다. 접촉 감지는 계속 동작한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Whip|Tuning|Aim Hit")
	bool bAimHitCollisionFreeSolve = true;
};

/** ③이 입력 순간 확정하는 prepared preview(렌더 + GuidedThrow/Wrapped 진입 재료). */
struct DYNAMICROPE_API FRopePreparedThrowPreview
{
	bool bValid = false;

	/** preview를 만들 때 쓴 throw 기준. montage가 있어도 입력 순간의 frame/origin을 보존하기 위해 저장한다. */
	FRopeThrowContext ThrowContext;

	/** 화면에 보이는 preview centerline. GuidedThrow에서는 이 점들을 실제 노드 목표 위치로도 사용한다. */
	FRopeWrapPreviewData RenderPreview;

	/** aim ray 조준처럼 소켓 애니메이션에서 독립시킬 필요가 있는 path는 owner 기준 로컬로도 보관한다. */
	bool bUseGuideFrameLocal = false;
	TWeakObjectPtr<const USceneComponent> GuideFrameComponent = nullptr;
	TArray<FVector> GuideFrameLocalPoints;
	FVector GuideFrameLocalOrigin = FVector::ZeroVector;

	/** 최종 Wrapped 진입에 필요한 bone-local 고정 정보. Points만으로는 캐릭터 움직임을 따라갈 수 없다. */
	FRopeSurfaceAnchor LatchAnchor;
	TArray<FRopeSurfaceAnchor> Anchors;

	TWeakObjectPtr<const USceneComponent> Mesh = nullptr;
	FName Bone = NAME_None;

	void Reset()
	{
		*this = FRopePreparedThrowPreview();
	}

	bool IsValid() const
	{
		return bValid && RenderPreview.IsValid() && Mesh.IsValid() && !Bone.IsNone() && LatchAnchor.NodeIndex != INDEX_NONE;
	}

	/** 생성 당시 월드 preview를 wielder owner 기준 로컬 좌표로 저장해 소켓 애니메이션에서 분리한다. */
	void StoreGuideFrameLocal(const USceneComponent* InGuideFrame);

	/** owner-local guide frame과 로컬 점 데이터가 모두 유효한지 확인한다. */
	bool HasGuideFrameLocal() const;

	/** 저장한 owner-local origin을 현재 owner transform 기준 월드 좌표로 복원한다. */
	FVector ResolveGuideOriginWorld() const;

	/** 지정한 owner-local spline 점을 현재 owner transform 기준 월드 좌표로 복원한다. */
	FVector ResolveGuidePointWorld(int32 PointIndex) const;

	/** 렌더용 전체 preview를 현재 owner transform에 맞춘 월드 데이터로 해석한다. */
	FRopeWrapPreviewData ResolveRenderPreviewWorld() const;
};

/** GuidedThrow 페이즈의 작업 상태: cached preview path를 authoritative하게 따라가는 진행분. */
struct FRopeGuidedThrowState
{
	bool bActive = false;

	/** Wielder가 확정한 prepared preview. 이 phase에서는 접촉 탐색을 다시 하지 않고 이 데이터만 따른다. */
	FRopePreparedThrowPreview Prepared;

	/**
	 * 허공(대상 없음) 던지기: 레이 끝점을 향한 아치 비행. 대상 mesh/bone/anchor 없이 RenderPreview 직선만
	 * 따라가고, 완료 시 꽂힘(Wrapped)이 아니라 Free로 낙하한다. false면 종전 조준 던지기(꽂힘).
	 */
	bool bFreeThrow = false;

	/** GuidedThrow 시작 순간의 실제 rope 위치. RenderPreview.Points로 전체 노드를 lerp하는 시작점이다. */
	TArray<FVector> StartPositions;
	float Elapsed = 0.0f;
	float Duration = 0.18f;

	void Reset()
	{
		*this = FRopeGuidedThrowState();
	}
};
