// Copyright Epic Games, Inc. All Rights Reserved.
//
// Aim-ray 조준 로직/상태(UObject-free F-클래스). Wielder의 조준 흐름이 쓰는 swept ray 본 질의
// (FindAimRayBoneHit), aim throw 컨텍스트 해석, collider 수집 확장 AABB 계산, 그리고 throw당
// wrap primary 잠금(mesh+bone), resolve mode별 허용 범위 + pending aim throw 큐를 담당한다.
// UObject 컨텍스트(collider 스냅샷/폴백 치수/CanWrapTarget 게이트/디버그 월드)는 호출마다
// 파라미터로 주입된다 — 월드 없이 단위 테스트 가능. StartFreshThrow *전이*가 걸린 진입점
// (QueueAimRayThrow/ResolvePendingAimThrow)은 URopeComponent에 남는다(오케스트레이션).

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "Core/RopeTypes.h"

class IRopeCollider;
class USceneComponent;
class UWorld;

/** Wielder aim ray가 rope collider/SDF에서 찾은 가장 가까운 wrap 가능 본 hit. */
struct FRopeAimRayHitResult
{
	// 아래 mesh/bone/표면 데이터가 모두 유효한 결과인지 나타낸다.
	bool bHit = false;
	// 이 throw에서 wrap 대상으로 잠글 bone 이름이다.
	FName Bone = NAME_None;
	// 같은 이름의 bone을 가진 다른 actor와 구분하기 위한 component이다.
	const USceneComponent* Mesh = nullptr;
	// swept ray가 collider에 처음 진입한 월드 위치이다.
	FVector HitWorldPos = FVector::ZeroVector;
	// SDF/contact projection으로 얻은 실제 표면점이다.
	FVector SurfacePoint = FVector::ZeroVector;
	// 표면점의 바깥 방향 법선이다.
	FVector Normal = FVector::UpVector;
	// ray origin에서 HitWorldPos까지의 투영 거리이다.
	float Distance = 0.0f;
	// 맞은 콜라이더의 월드 bounds 반경 근사(반대각 길이). 조준 HUD 강조 링 크기 산정용(가산 필드).
	float TargetBoundsRadius = 0.0f;
};

/** Wielder가 입력 순간 고정하고 RopeSimSubsystem의 최신 collider 수집 직후 해결할 Aim throw 요청. */
struct FRopeAimRayThrowRequest
{
	FRopeThrowContext BaseContext;
	FVector RayOrigin = FVector::ZeroVector;
	FVector RayDirection = FVector::ForwardVector;
	float RayLength = 0.0f;
	float QueryRadius = 0.0f;
	float SweepStep = 2.0f;
	bool bDrawDebug = false;
	// StartFreshThrow 완료 뒤 실행한다. Wielder를 직접 참조하지 않는 C++ 전용 완료 알림이다.
	FSimpleDelegate OnResolved;

	bool IsValid() const
	{
		return RayLength > KINDA_SMALL_NUMBER && !RayDirection.IsNearlyZero();
	}
};

class DYNAMICROPE_API FRopeAimTargeting
{
public:
	/** 질의 공통 컨텍스트. 호출자(URopeComponent)가 프레임 값으로 조립해 넘긴다. */
	struct FQueryContext
	{
		// 이번 프레임 collider 스냅샷(SimFrame.FrameColliders). 호출 범위 동안만 유효.
		const TArray<IRopeCollider*>* Colliders = nullptr;
		// RayLength 미지정(<=0) 시 폴백 길이: max(현재 Sim.RopeLength, 초기 RopeLength).
		float FallbackRayLength = 0.0f;
		// QueryRadius 미지정(<=0) 시 폴백 반경: max(튜브 Radius, WrapConfig.ContactQueryRadius).
		float FallbackQueryRadius = 0.0f;
	};

	//~ 질의(상태 불변 — static) --------------------------------------------
	/** swept SDF 질의로 ray에서 가장 가까운 wrap 가능 mesh+bone을 찾는다(broad phase → QuerySwept →
	 *  ray 진행 거리 최솟값). CanWrapTarget 게이트를 통과 못 한 후보는 없는 것으로 취급.
	 *  bDrawDebug면 DebugWorld에 실제 질의 치수를 시각화(cyan=미스/red=히트).
	 *  OutBlockedHit(옵션): ray가 콜라이더에 맞았지만 wrap은 불가능한(본 없음/SourceMesh 없음/게이트 거부)
	 *  가장 가까운 hit. 반환값(wrap 가능 hit 유무)과 독립 — 조준 HUD의 "빨강" 표시용. */
	static bool FindAimRayBoneHit(const FQueryContext& Ctx,
		const FVector& Origin, const FVector& AimDir, float RayLength, float QueryRadius, float SweepStep,
		bool bDrawDebug, const UWorld* DebugWorld,
		TFunctionRef<bool(const USceneComponent*, FName)> CanWrapTarget,
		FRopeAimRayHitResult& OutHit,
		FRopeAimRayHitResult* OutBlockedHit = nullptr);

	/** Aim 요청을 hit 컨텍스트(FrameForward/AimGuide*)로 해석한다. hit이 없으면 OutContext는
	 *  BaseContext fallback(반환 false). 디버그 드로우 여부는 Request.bDrawDebug를 따른다. */
	static bool ResolveAimRayThrowContext(const FQueryContext& Ctx, const FRopeAimRayThrowRequest& Request,
		const UWorld* DebugWorld,
		TFunctionRef<bool(const USceneComponent*, FName)> CanWrapTarget,
		FRopeThrowContext& OutContext);

	/** Aim ray가 검사할 collider 수집 확장 AABB를 만든다. 무효 입력이면 FBox(ForceInit)
	 *  (= 수집 확장 없음 — SimFrame.AimRayColliderQueryBounds의 clear와 동일 의미). */
	static FBox MakeAimRayQueryBounds(const FQueryContext& Ctx,
		const FVector& Origin, const FVector& AimDir, float RayLength, float QueryRadius);

	//~ wrap 대상 잠금(throw당) ---------------------------------------------
	/** throw 컨텍스트의 AimGuide hit로 잠금을 설정/해제한다(StartFreshThrow/prepared throw 진입 시 1회). */
	void SetWrapTargetLock(const FRopeThrowContext& ThrowContext);

	/** 잠금이 이 페이즈에 적용 중인가. 한 throw의 접근/접촉/감김(Flight/Contacting/Wrapping)에만
	 *  적용한다 — Free preview와 Wrapped 이후의 일반 충돌은 유지한다. */
	bool IsLockActive(ERopePhase Phase) const;

	/** 정확히 aim ray가 잠근 primary (Mesh, Bone)인가. Assisted의 캡처/dominant 고정이 소비한다. */
	bool IsPrimaryTarget(const USceneComponent* Mesh, FName Bone) const;

	// [Assisted 멀티 본 계약] 조준 본은 primary 판정용이고, 같은 mesh의 다른 본은 감김 경로 후보용이다.
	// 이 둘을 하나의 exact-bone 조건으로 합치면 Assisted에서 다른 본 collider가 다시 사라진다.
	/** (Mesh, Bone)이 현재 resolve mode에서 허용되는 wrap 대상인가(잠금 비활성이면 모두 통과).
	 *  Assisted는 primary와 같은 mesh의 다른 본도 후보/경로 투영에 허용하고, Guaranteed는 exact bone만 허용한다. */
	bool IsWrapTarget(ERopePhase Phase, ERopeWrapResolveMode ResolveMode,
		const USceneComponent* Mesh, FName Bone) const;

	/** 잠금 활성 시 resolve mode 정책에 맞지 않는 skeletal collider를 제거한다.
	 *  Assisted는 같은 mesh의 모든 본을 유지하고 Guaranteed는 exact bone만 유지한다.
	 *  월드 정적 형상은 궤적/환경 충돌용이므로 항상 유지한다. 잠금 비활성이면 no-op. */
	void FilterCollidersToTarget(ERopePhase Phase, ERopeWrapResolveMode ResolveMode,
		TArray<IRopeCollider*>& Colliders) const;

	const USceneComponent* GetLockedTargetMesh() const { return TargetMesh.Get(); }
	FName GetLockedTargetBone() const { return TargetBone; }

	//~ pending aim throw(입력 순간 고정 → collider gather 직후 소비) ---------
	void QueuePendingThrow(const FRopeAimRayThrowRequest& Request) { PendingThrow = Request; }

	/** pending을 값으로 꺼내며 비운다(없으면 false). 호출자(ResolvePendingAimThrow)의 StartFreshThrow가
	 *  transient 상태를 리셋하므로, 요청은 반드시 꺼낸 값으로 이어서 처리한다. */
	bool TakePendingThrow(FRopeAimRayThrowRequest& OutRequest);

	void ResetPendingThrow() { PendingThrow.Reset(); }

private:
	// throw 시작 때 ray hit로 확정한 대상. 같은 bone 이름을 가진 다른 액터를 막기 위해 mesh도 함께 저장한다.
	bool bLocked = false;
	FName TargetBone = NAME_None;
	TWeakObjectPtr<const USceneComponent> TargetMesh = nullptr;

	// 입력 순간의 ray/frame을 보존하며, Subsystem collider gather 직후 한 번 소비한다.
	TOptional<FRopeAimRayThrowRequest> PendingThrow;
};
