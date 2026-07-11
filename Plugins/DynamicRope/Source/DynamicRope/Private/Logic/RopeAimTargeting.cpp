// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeAimTargeting.h"
#include "Collision/RopeCollider.h"
#include "DrawDebugHelpers.h"

bool FRopeAimTargeting::FindAimRayBoneHit(const FQueryContext& Ctx,
	const FVector& Origin, const FVector& AimDir, float RayLength, float QueryRadius, float SweepStep,
	bool bDrawDebug, const UWorld* DebugWorld,
	TFunctionRef<bool(const USceneComponent*, FName)> CanWrapTarget,
	FRopeAimRayHitResult& OutHit)
{
	OutHit = FRopeAimRayHitResult();
	if (!Ctx.Colliders)
	{
		return false;
	}

	const FVector RayDir = AimDir.GetSafeNormal();
	const float EffectiveRayLength = RayLength > KINDA_SMALL_NUMBER ? RayLength : Ctx.FallbackRayLength;
	if (EffectiveRayLength <= KINDA_SMALL_NUMBER || RayDir.IsNearlyZero())
	{
		return false;
	}

	const FVector RayStart = Origin;
	const FVector RayEnd = RayStart + RayDir * EffectiveRayLength;
	// 0 설정은 선 ray가 아니라 rope/contact 기본 두께를 사용한다. 명시값이 있으면 그 반경으로 sweep한다.
	const float EffectiveQueryRadius = QueryRadius > KINDA_SMALL_NUMBER ? QueryRadius : Ctx.FallbackQueryRadius;
	const float EffectiveSweepStep = FMath::Clamp(SweepStep > KINDA_SMALL_NUMBER ? SweepStep : 2.0f, 0.5f, 10.0f);

	FRopeSweptQuery Query;
	Query.WorldStart = RayStart;
	Query.WorldEnd = RayEnd;
	Query.NodeRadius = EffectiveQueryRadius;
	Query.SweepStep = EffectiveSweepStep;
	Query.MaxSamples = FMath::Clamp(1 + FMath::CeilToInt(EffectiveRayLength / EffectiveSweepStep), 2, 4096);

	const FBox RayBounds(RayStart.ComponentMin(RayEnd), RayStart.ComponentMax(RayEnd));
	const FBox ExpandedRayBounds = RayBounds.ExpandBy(EffectiveQueryRadius);
	bool bFoundHit = false;
	FRopeAimRayHitResult BestHit;

	// broad phase bounds를 통과한 collider만 같은 swept query로 검사하고 ray 진행 거리의 최솟값을 고른다.
	for (const IRopeCollider* Collider : *Ctx.Colliders)
	{
		if (!Collider || !Collider->GetWorldBounds().Intersect(ExpandedRayBounds))
		{
			continue;
		}

		FVector HitWorldPos = FVector::ZeroVector;
		const FRopeContact Contact = Collider->QuerySwept(Query, HitWorldPos);
		if (!Contact.bHit || Contact.Bone.IsNone() || !Contact.SourceMesh ||
			!CanWrapTarget(Contact.SourceMesh, Contact.Bone))
		{
			continue;
		}

		FRopeAimRayHitResult Candidate;
		Candidate.bHit = true;
		Candidate.Bone = Contact.Bone;
		Candidate.Mesh = Contact.SourceMesh;
		Candidate.HitWorldPos = HitWorldPos;
		Candidate.SurfacePoint = Contact.SurfacePoint;
		Candidate.Normal = Contact.Normal;
		Candidate.Distance = FVector::DotProduct(HitWorldPos - RayStart, RayDir);
		if (!bFoundHit || Candidate.Distance < BestHit.Distance)
		{
			BestHit = Candidate;
			bFoundHit = true;
		}
	}

	if (bDrawDebug && DebugWorld)
	{
		// cyan/red capsule은 실제 QuerySwept에 전달한 길이와 반경을 그대로 시각화한다.
		constexpr float LifeTime = 0.05f;
		const bool bHit = bFoundHit && BestHit.bHit;
		const FVector RayStop = bHit ? BestHit.HitWorldPos : RayEnd;
		const FColor MainColor = bHit ? FColor::Red : FColor::Cyan;
		const FQuat CapsuleRotation = FRotationMatrix::MakeFromZ(RayDir).ToQuat();
		DrawDebugCapsule(DebugWorld, (RayStart + RayEnd) * 0.5f,
			EffectiveRayLength * 0.5f + EffectiveQueryRadius, EffectiveQueryRadius,
			CapsuleRotation, MainColor, false, LifeTime, 0, 1.0f);
		DrawDebugLine(DebugWorld, RayStart, RayStop, MainColor, false, LifeTime, 0, 2.0f);
		if (bHit)
		{
			DrawDebugLine(DebugWorld, RayStop, RayEnd, FColor(96, 0, 0), false, LifeTime, 0, 1.0f);
			DrawDebugSphere(DebugWorld, BestHit.HitWorldPos, 8.0f, 12, FColor::Yellow, false, LifeTime, 0, 2.0f);
			DrawDebugString(DebugWorld, BestHit.HitWorldPos + FVector(0.0f, 0.0f, 14.0f),
				BestHit.Bone.ToString(), nullptr, FColor::Yellow, LifeTime, false, 1.0f);
		}
	}

	if (!bFoundHit)
	{
		return false;
	}

	OutHit = BestHit;
	return true;
}

bool FRopeAimTargeting::ResolveAimRayThrowContext(const FQueryContext& Ctx, const FRopeAimRayThrowRequest& Request,
	const UWorld* DebugWorld,
	TFunctionRef<bool(const USceneComponent*, FName)> CanWrapTarget,
	FRopeThrowContext& OutContext)
{
	OutContext = Request.BaseContext;
	if (!Request.IsValid())
	{
		return false;
	}

	FRopeAimRayHitResult Hit;
	if (!FindAimRayBoneHit(Ctx, Request.RayOrigin, Request.RayDirection, Request.RayLength,
		Request.QueryRadius, Request.SweepStep, Request.bDrawDebug, DebugWorld, CanWrapTarget, Hit))
	{
		return false;
	}

	// ray 시작점이 아니라 실제 throw origin에서 hit으로 향하는 벡터가 최종 guide forward다.
	const FVector HitAimDir = (Hit.HitWorldPos - OutContext.Origin).GetSafeNormal();
	if (HitAimDir.IsNearlyZero())
	{
		return false;
	}

	OutContext.FrameForward = HitAimDir;
	OutContext.bHasAimGuideHit = true;
	OutContext.AimGuideBone = Hit.Bone;
	OutContext.AimGuideMesh = Hit.Mesh;
	OutContext.AimGuideHitWorldPos = Hit.HitWorldPos;
	OutContext.AimGuideSurfacePoint = Hit.SurfacePoint;
	OutContext.AimGuideNormal = Hit.Normal;
	OutContext.AimGuideDistance = Hit.Distance;
	return true;
}

FBox FRopeAimTargeting::MakeAimRayQueryBounds(const FQueryContext& Ctx,
	const FVector& Origin, const FVector& AimDir, float RayLength, float QueryRadius)
{
	const FVector RayDir = AimDir.GetSafeNormal();
	const float EffectiveRayLength = RayLength > KINDA_SMALL_NUMBER ? RayLength : Ctx.FallbackRayLength;
	if (RayDir.IsNearlyZero() || EffectiveRayLength <= KINDA_SMALL_NUMBER)
	{
		return FBox(ForceInit); // 무효 = 수집 확장 없음(clear와 동일).
	}

	const float EffectiveQueryRadius = QueryRadius > KINDA_SMALL_NUMBER ? QueryRadius : Ctx.FallbackQueryRadius;
	const FVector RayEnd = Origin + RayDir * EffectiveRayLength;
	return FBox(Origin.ComponentMin(RayEnd), Origin.ComponentMax(RayEnd))
		.ExpandBy(EffectiveQueryRadius);
}

void FRopeAimTargeting::SetWrapTargetLock(const FRopeThrowContext& ThrowContext)
{
	bLocked = ThrowContext.bHasAimGuideHit &&
		!ThrowContext.AimGuideBone.IsNone() && ThrowContext.AimGuideMesh.IsValid();
	TargetBone = bLocked ? ThrowContext.AimGuideBone : NAME_None;
	TargetMesh = bLocked ? ThrowContext.AimGuideMesh : nullptr;
}

bool FRopeAimTargeting::IsLockActive(ERopePhase Phase) const
{
	// 잠금은 한 throw의 접근/접촉/감김 경로에만 적용한다. Free preview와 Wrapped 이후의 일반 충돌은 유지한다.
	const bool bLockingPhase = Phase == ERopePhase::Flight ||
		Phase == ERopePhase::Contacting || Phase == ERopePhase::Wrapping;
	return bLockingPhase && bLocked && !TargetBone.IsNone() && TargetMesh.IsValid();
}

bool FRopeAimTargeting::IsWrapTarget(ERopePhase Phase, const USceneComponent* Mesh, FName Bone) const
{
	return !IsLockActive(Phase) || (Mesh == TargetMesh.Get() && Bone == TargetBone);
}

void FRopeAimTargeting::FilterCollidersToTarget(ERopePhase Phase, TArray<IRopeCollider*>& Colliders) const
{
	if (!IsLockActive(Phase))
	{
		return;
	}

	Colliders.RemoveAll([this](const IRopeCollider* Collider)
	{
		if (!Collider)
		{
			return true;
		}
		if (Collider->IsWorldStatic())
		{
			// 월드 정적 형상은 궤적/환경 충돌용이므로 유지하고 skeletal 본 collider만 target으로 제한한다.
			return false;
		}

		FName ColliderBone = NAME_None;
		const USceneComponent* ColliderMesh = nullptr;
		Collider->GetGPUAttribution(ColliderBone, ColliderMesh);
		return ColliderMesh != TargetMesh.Get() || ColliderBone != TargetBone;
	});
}

bool FRopeAimTargeting::TakePendingThrow(FRopeAimRayThrowRequest& OutRequest)
{
	if (!PendingThrow.IsSet())
	{
		return false;
	}
	OutRequest = PendingThrow.GetValue();
	PendingThrow.Reset();
	return true;
}
