// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeStaticBodyProvider.h"
#include "DynamicRopeLog.h"
#include "Subsystem/RopeSimSubsystem.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/OverlapResult.h"
#include "Engine/World.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/ConvexElem.h" // FKConvexElem::GetPlanes(월드 평면 추출)

namespace
{
	// elem 로컬 → 월드 변환 행렬. UE 컨벡스 규약(FKConvexElem::CalcAABB): WT = ElemTM * Scale * Comp(무스케일).
	// 스케일을 회전/이동과 분리해 매트릭스로 합성한다(FTransform은 전단 표현 불가) — 비균등 스케일 × 회전에서도
	// 평면이 정확히 변환된다.
	FMatrix ComposeConvexToWorld(const FTransform& ElemTM, const FVector& Scale3D, const FTransform& CompTM)
	{
		FTransform CompNoScale = CompTM;
		CompNoScale.SetScale3D(FVector::OneVector);
		return ElemTM.ToMatrixWithScale() * FScaleMatrix(Scale3D) * CompNoScale.ToMatrixWithScale();
	}

	// 로컬 평면 집합을 월드로 변환 + 정규화(단위 법선·바깥). FPlane::TransformBy가 역전치로 법선을 올바르게
	// 변환하므로 전단에서도 정확 — 단 길이가 변하므로 (N,W)를 |N|으로 나눠 정규화한다.
	void TransformPlanesToWorld(const TArray<FPlane>& Local, const FMatrix& M, TArray<FPlane>& OutWorld)
	{
		OutWorld.Reset(Local.Num());
		for (const FPlane& LP : Local)
		{
			FPlane WP = LP.TransformBy(M);
			const double NLen = FMath::Sqrt(WP.X * WP.X + WP.Y * WP.Y + WP.Z * WP.Z);
			if (NLen > UE_SMALL_NUMBER)
			{
				WP.X /= NLen; WP.Y /= NLen; WP.Z /= NLen; WP.W /= NLen;
				OutWorld.Add(WP);
			}
		}
	}

	// 로컬 박스(중심 원점, 반폭 Half)의 6평면(바깥 법선 ±축). 전단 박스를 컨벡스로 정확히 라우팅할 때 쓴다.
	TArray<FPlane> MakeBoxLocalPlanes(const FVector& Half)
	{
		TArray<FPlane> P;
		P.Reserve(6);
		P.Add(FPlane(FVector(1, 0, 0), Half.X));
		P.Add(FPlane(FVector(-1, 0, 0), Half.X));
		P.Add(FPlane(FVector(0, 1, 0), Half.Y));
		P.Add(FPlane(FVector(0, -1, 0), Half.Y));
		P.Add(FPlane(FVector(0, 0, 1), Half.Z));
		P.Add(FPlane(FVector(0, 0, -1), Half.Z));
		return P;
	}
}

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
		Convexes.Reset();
		BuildColliders(RopeBounds);
	}

	OutColliders.Reserve(OutColliders.Num() + Boxes.Num() + Capsules.Num() + Convexes.Num());
	for (FRopeBoxCollider& Box : Boxes)
	{
		OutColliders.Add(&Box);
	}
	for (FRopeStaticCapsuleCollider& Cap : Capsules)
	{
		OutColliders.Add(&Cap);
	}
	for (FRopeConvexCollider& Convex : Convexes)
	{
		OutColliders.Add(&Convex);
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
	UE_LOG(LogRopeCollision, VeryVerbose, TEXT("StaticBodyProvider on %s: built %d box(es) + %d capsule(s) + %d convex from %d overlapped component(s)."),
		*GetNameSafe(GetOwner()), Boxes.Num(), Capsules.Num(), Convexes.Num(), Seen.Num());
}

bool URopeStaticBodyProvider::AppendBodyColliders(const UBodySetup& Setup, const FTransform& CompTM)
{
	const FVector Scale3D = CompTM.GetScale3D();
	const auto BudgetLeft = [this]() { return Boxes.Num() + Capsules.Num() + Convexes.Num() < MaxColliders; };

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

	// box: 해석적 OBB — 모서리 정확 처리의 본체. X/Y/Z는 전체 길이. 전단(비균등 스케일 × 회전 elem)만
	// 6평면 컨벡스로 정확히 라우팅한다(OBB로는 표현 불가). 나머지는 정확한 OBB.
	for (const FKBoxElem& Box : Setup.AggGeom.BoxElems)
	{
		if (!BudgetLeft())
		{
			return false;
		}
		const FVector HalfLocal(Box.X * 0.5, Box.Y * 0.5, Box.Z * 0.5); // elem 공간 반폭(스케일 전).
		const FVector AbsScale = Scale3D.GetAbs();
		const bool bUniform = FMath::IsNearlyEqual(AbsScale.GetMax(), AbsScale.GetMin(), UE_KINDA_SMALL_NUMBER);
		if (bUniform || Box.Rotation.IsNearlyZero())
		{
			// 정확 OBB: 균등 스케일(전 축 동일) 또는 elem 회전 identity(컴포넌트 축 정렬 → 축별 스케일 정확).
			const FTransform ElemTM = Box.GetTransform() * CompTM;
			const FVector Half = bUniform ? HalfLocal * AbsScale.X : HalfLocal * AbsScale;
			Boxes.Add(FRopeBoxCollider(ElemTM.GetLocation(), ElemTM.GetRotation(), Half));
		}
		else
		{
			// 전단: 6평면 컨벡스로 정확히(평면은 전단 행렬로도 정확 변환). M1의 min-scale 근사를 대체.
			TArray<FPlane> WorldPlanes;
			const FMatrix M = ComposeConvexToWorld(Box.GetTransform(), Scale3D, CompTM);
			TransformPlanesToWorld(MakeBoxLocalPlanes(HalfLocal), M, WorldPlanes);
			const FBox WB = FBox(-HalfLocal, HalfLocal).TransformBy(M);
			if (WorldPlanes.Num() == 6 && WB.IsValid)
			{
				Convexes.Add(FRopeConvexCollider(MoveTemp(WorldPlanes), WB));
			}
		}
	}

	// convex: Chaos convex의 평면 집합을 월드로 변환해 해석적 컨벡스 collider로. 미쿡(빈 평면)이거나
	// 평면 과다(>상한)면 ElemBox를 OBB 근사로 폴백해 충돌을 통째로 잃지 않는다.
	for (const FKConvexElem& Convex : Setup.AggGeom.ConvexElems)
	{
		if (!BudgetLeft())
		{
			return false;
		}
		TArray<FPlane> LocalPlanes;
		Convex.GetPlanes(LocalPlanes);
		const FMatrix M = ComposeConvexToWorld(Convex.GetTransform(), Scale3D, CompTM);

		if (LocalPlanes.Num() >= 4 && LocalPlanes.Num() <= MaxConvexPlanes && Convex.ElemBox.IsValid)
		{
			TArray<FPlane> WorldPlanes;
			TransformPlanesToWorld(LocalPlanes, M, WorldPlanes);
			const FBox WB = Convex.ElemBox.TransformBy(M);
			if (WorldPlanes.Num() >= 4 && WB.IsValid)
			{
				Convexes.Add(FRopeConvexCollider(MoveTemp(WorldPlanes), WB));
				continue;
			}
		}

		// 폴백: ElemBox OBB 근사(미쿡/평면 과다/무효). 거칠지만 충돌 유지 > 통째 누락.
		if (Convex.ElemBox.IsValid)
		{
			const FVector CenterW = M.TransformPosition(Convex.ElemBox.GetCenter());
			const FQuat   RotW = M.GetMatrixWithoutScale().ToQuat();
			const FVector HalfW = Convex.ElemBox.GetExtent() * static_cast<float>(Scale3D.GetAbsMin());
			Boxes.Add(FRopeBoxCollider(CenterW, RotW, HalfW));
			UE_LOG(LogRopeCollision, Verbose,
				TEXT("StaticBodyProvider on %s: convex elem unusable (%d planes) — falling back to ElemBox OBB."),
				*GetNameSafe(GetOwner()), LocalPlanes.Num());
		}
	}

	return true;
}
