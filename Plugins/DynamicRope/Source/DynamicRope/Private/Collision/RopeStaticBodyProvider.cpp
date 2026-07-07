// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeStaticBodyProvider.h"
#include "DynamicRopeLog.h"
#include "Subsystem/RopeSimSubsystem.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/OverlapResult.h"
#include "Engine/World.h"
#include "PhysicsEngine/BodySetup.h"

URopeStaticBodyProvider::URopeStaticBodyProvider()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void URopeStaticBodyProvider::BeginPlay()
{
	Super::BeginPlay();
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->RegisterColliderProvider(this);
	}
}

void URopeStaticBodyProvider::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->UnregisterColliderProvider(this);
	}
	Super::EndPlay(EndPlayReason);
}

void URopeStaticBodyProvider::GatherColliders(const FBox& RopeBounds, TArray<IRopeCollider*>& OutColliders)
{
	// 프레임당 1회만 빌드(디둡). RopeBounds는 서브시스템이 전 로프 union AABB(+여유)로 프레임 내내
	// 동일하게 넘기므로, 첫 호출의 오버랩 결과를 그 프레임의 모든 로프가 공유한다(per-rope 컬링은
	// 서브시스템의 collider AABB 컬링이 담당).
	const uint64 Frame = GFrameCounter;
	if (BuiltFrame != Frame)
	{
		BuiltFrame = Frame;
		Boxes.Reset();
		Capsules.Reset();
		BuildColliders(RopeBounds);
	}

	OutColliders.Reserve(OutColliders.Num() + Boxes.Num() + Capsules.Num());
	for (FRopeBoxCollider& Box : Boxes)
	{
		OutColliders.Add(&Box);
	}
	for (FRopeStaticCapsuleCollider& Cap : Capsules)
	{
		OutColliders.Add(&Cap);
	}
}

void URopeStaticBodyProvider::BuildColliders(const FBox& RopeBounds)
{
	UWorld* World = GetWorld();
	if (!World || !RopeBounds.IsValid)
	{
		return; // 활성 로프가 없으면(무효 bounds) 스캔할 이유가 없다.
	}

	// 브로드페이즈: 로프 활성 영역과 겹치는 정적 오브젝트를 GT에서 1회 오버랩. 오브젝트 타입 기준이라
	// WorldStatic 타입의 movable 액터도 잡힌다 — 트랜스폼은 매 프레임 다시 읽으므로 위치는 따라가고,
	// 표면 속도만 0(정적 응답) 근사가 된다.
	TArray<FOverlapResult> Overlaps;
	FCollisionObjectQueryParams ObjParams(ECC_WorldStatic);
	FCollisionQueryParams QueryParams(FName(TEXT("RopeStaticBodyGather")), /*bInTraceComplex*/ false);
	World->OverlapMultiByObjectType(Overlaps, RopeBounds.GetCenter(), FQuat::Identity, ObjParams,
		FCollisionShape::MakeBox(RopeBounds.GetExtent()), QueryParams);

	TSet<const UPrimitiveComponent*> Seen; // 오버랩은 바디별로 나올 수 있어 컴포넌트 단위로 디둡.
	int32 NumConvexSkipped = 0;
	bool bBudgetClipped = false;
	for (const FOverlapResult& Overlap : Overlaps)
	{
		UPrimitiveComponent* Prim = Overlap.Component.Get();
		if (!Prim || Seen.Contains(Prim))
		{
			continue;
		}
		Seen.Add(Prim);
		if (IgnoredComponents.Contains(Prim))
		{
			continue;
		}
		// ISM/HISM은 v1 범위 밖 — GetBodySetup이 인스턴스 트랜스폼을 모르는 원본 셰이프를 주므로
		// (컴포넌트 원점에 콜라이더 1개 = 오답) 통째로 건너뛴다. M3에서 인스턴스별 처리 예정.
		if (Prim->IsA<UInstancedStaticMeshComponent>())
		{
			continue;
		}
		const UBodySetup* Setup = Prim->GetBodySetup();
		if (!Setup)
		{
			continue;
		}
		NumConvexSkipped += Setup->AggGeom.ConvexElems.Num(); // M2 전까지 컨벡스는 미지원.
		if (!AppendBodyColliders(*Setup, Prim->GetComponentTransform()))
		{
			bBudgetClipped = true;
			break;
		}
	}

	if (bBudgetClipped)
	{
		UE_LOG(LogRopeCollision, Verbose,
			TEXT("StaticBodyProvider on %s: collider budget (%d) exceeded — remaining static bodies dropped this frame."),
			*GetNameSafe(GetOwner()), MaxColliders);
	}
	if (NumConvexSkipped > 0)
	{
		UE_LOG(LogRopeCollision, VeryVerbose,
			TEXT("StaticBodyProvider on %s: skipped %d convex elem(s) — convex colliders land in M2."),
			*GetNameSafe(GetOwner()), NumConvexSkipped);
	}
	UE_LOG(LogRopeCollision, VeryVerbose, TEXT("StaticBodyProvider on %s: built %d box(es) + %d capsule(s) from %d overlapped component(s)."),
		*GetNameSafe(GetOwner()), Boxes.Num(), Capsules.Num(), Seen.Num());
}

bool URopeStaticBodyProvider::AppendBodyColliders(const UBodySetup& Setup, const FTransform& CompTM)
{
	const FVector Scale3D = CompTM.GetScale3D();
	const auto BudgetLeft = [this]() { return Boxes.Num() + Capsules.Num() < MaxColliders; };

	// sphyl: 스킨 캡슐 provider와 동일한 스케일 규약(GetScaledRadius/CylinderLength).
	for (const FKSphylElem& Sphyl : Setup.AggGeom.SphylElems)
	{
		if (!BudgetLeft())
		{
			return false;
		}
		const FTransform ElemTM = Sphyl.GetTransform() * CompTM;
		const FVector Axis = ElemTM.GetUnitAxis(EAxis::Z); // sphyl 축 = 로컬 Z
		const FVector Center = ElemTM.GetLocation();
		const float HalfLen = Sphyl.GetScaledCylinderLength(Scale3D) * 0.5f;
		Capsules.Add(FRopeStaticCapsuleCollider(Center + Axis * HalfLen, Center - Axis * HalfLen,
			Sphyl.GetScaledRadius(Scale3D)));
	}

	// sphere: A==B 축퇴 캡슐.
	for (const FKSphereElem& Sphere : Setup.AggGeom.SphereElems)
	{
		if (!BudgetLeft())
		{
			return false;
		}
		const FVector Center = CompTM.TransformPosition(Sphere.Center);
		const float ScaledRadius = Sphere.Radius * static_cast<float>(Scale3D.GetAbsMin());
		Capsules.Add(FRopeStaticCapsuleCollider(Center, Center, ScaledRadius));
	}

	// box: 해석적 OBB — 모서리 정확 처리의 본체. X/Y/Z는 전체 길이.
	for (const FKBoxElem& Box : Setup.AggGeom.BoxElems)
	{
		if (!BudgetLeft())
		{
			return false;
		}
		const FTransform ElemTM = Box.GetTransform() * CompTM;
		FVector Half(Box.X * 0.5, Box.Y * 0.5, Box.Z * 0.5);
		const FVector AbsScale = Scale3D.GetAbs();
		if (FMath::IsNearlyEqual(AbsScale.GetMax(), AbsScale.GetMin(), UE_KINDA_SMALL_NUMBER))
		{
			Half *= AbsScale.X; // 균등 스케일: 정확.
		}
		else if (Box.Rotation.IsNearlyZero())
		{
			Half *= AbsScale; // elem 회전 identity: 컴포넌트 축 정렬이라 축별 스케일이 정확.
		}
		else
		{
			// 비균등 스케일 × 회전된 elem = 전단(shear) — OBB로 정확히 표현 불가. M2에서 6평면
			// 컨벡스로 라우팅 예정. 그때까지 최소 스케일 보수 근사(관통보다 얇게 잡는 쪽) + 로그.
			Half *= AbsScale.GetAbsMin();
			UE_LOG(LogRopeCollision, Verbose,
				TEXT("StaticBodyProvider on %s: rotated box elem under non-uniform scale — using min-scale approximation (exact convex lands in M2)."),
				*GetNameSafe(GetOwner()));
		}
		Boxes.Add(FRopeBoxCollider(ElemTM.GetLocation(), ElemTM.GetRotation(), Half));
	}

	return true;
}
