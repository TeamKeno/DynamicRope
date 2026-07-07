// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeStaticBodyProvider.h"
#include "DynamicRopeLog.h"
#include "Subsystem/RopeSimSubsystem.h"
#include "Settings/DynamicRopeSettings.h" // 콜라이더 예산/컨벡스 평면 상한(단일 소스)
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

	// 바디-로컬 변환(elem + 스케일, 컴포넌트 강체 rot/trans 제외). 월드 = 바디로컬 ∘ 강체(컴포넌트 rot/trans).
	// 강체만 프레임 간 움직이므로(스케일 불변 가정) 바디-로컬 평면은 불변 → 동적 바디의 sub-포즈 강체 보간용.
	FMatrix ComposeConvexBodyLocal(const FTransform& ElemTM, const FVector& Scale3D)
	{
		return ElemTM.ToMatrixWithScale() * FScaleMatrix(Scale3D);
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

	// 콜라이더 예산/컨벡스 평면 상한/동적 포함 여부는 Project Settings에서 단일 관리.
	const UDynamicRopeSettings* Settings = UDynamicRopeSettings::Get();
	const int32 MaxColliders = Settings ? FMath::Max(1, Settings->StaticBodyMaxColliders) : 128;
	const int32 MaxConvexPlanes = Settings ? FMath::Max(4, Settings->StaticBodyMaxConvexPlanes) : 32;
	const bool  bIncludeDynamic = Settings ? Settings->bIncludeWorldDynamic : false;

	// 브로드페이즈: 로프 활성 영역과 겹치는 오브젝트를 GT에서 1회 오버랩. 기본 WorldStatic, 옵션으로 WorldDynamic
	// (움직이는 플랫폼/문 등)도 포함. 움직이는 바디는 아래에서 이전 프레임 트랜스폼으로 표면 속도를 산출한다.
	TArray<FOverlapResult> Overlaps;
	FCollisionObjectQueryParams ObjParams(ECC_WorldStatic);
	if (bIncludeDynamic)
	{
		ObjParams.AddObjectTypesToQuery(ECC_WorldDynamic);
	}
	FCollisionQueryParams QueryParams(FName(TEXT("RopeStaticBodyGather")), /*bInTraceComplex*/ false);
	World->OverlapMultiByObjectType(Overlaps, RopeBounds.GetCenter(), FQuat::Identity, ObjParams,
		FCollisionShape::MakeBox(RopeBounds.GetExtent()), QueryParams);

	// 표면 속도(드래그/CCD)용 프레임 dt. 이번 프레임 처리한 컴포넌트의 (curr - prev)/dt 로 산출한다.
	const float FrameDt = World->GetDeltaSeconds();
	const float InvDt = (FrameDt > KINDA_SMALL_NUMBER) ? (1.0f / FrameDt) : 0.0f;
	// 이번 프레임 컴포넌트 트랜스폼(다음 프레임 prev 소스). 처리한 것만 담아 파괴/이탈 항목은 자연히 만료.
	TMap<TWeakObjectPtr<UPrimitiveComponent>, FTransform> CurrCompXforms;
	CurrCompXforms.Reserve(Overlaps.Num());

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
		// ISM/HISM(M3): 인스턴스별 처리 — 한 컴포넌트가 공유 메시 콜리전을 여러 인스턴스에 배치한다.
		// GetBodySetup은 인스턴스 트랜스폼을 모르는 원본(로컬) 셰이프를 주므로, 근접 인스턴스마다 그
		// 월드 트랜스폼으로 추출해야 한다(HISM도 이 베이스로 캐치). 인스턴스별 prev 추적은 미지원 →
		// 인스턴스는 정적 스냅샷으로 처리(내부에서 prev=curr, InvDt=0).
		if (UInstancedStaticMeshComponent* ISM = Cast<UInstancedStaticMeshComponent>(Prim))
		{
			if (!AppendInstancedBodyColliders(*ISM, RopeBounds, MaxColliders, MaxConvexPlanes))
			{
				bBudgetClipped = true;
				break;
			}
			continue;
		}
		const UBodySetup* Setup = Prim->GetBodySetup();
		if (!Setup)
		{
			continue;
		}
		// 이 컴포넌트의 이전 프레임 트랜스폼 조회(없으면 이번 프레임은 정적 취급 — InvDt 0).
		const FTransform CompTM = Prim->GetComponentTransform();
		const FTransform* PrevPtr = PrevCompXforms.Find(Prim);
		const FTransform PrevTM = PrevPtr ? *PrevPtr : CompTM;
		const float CompInvDt = PrevPtr ? InvDt : 0.0f;
		CurrCompXforms.Add(Prim, CompTM);

		if (!AppendBodyColliders(*Setup, CompTM, PrevTM, CompInvDt, MaxColliders, MaxConvexPlanes))
		{
			bBudgetClipped = true;
			break;
		}
	}

	PrevCompXforms = MoveTemp(CurrCompXforms); // 다음 프레임 prev 소스로 교체.

	if (bBudgetClipped)
	{
		UE_LOG(LogRopeCollision, Verbose,
			TEXT("StaticBodyProvider on %s: collider budget (%d) exceeded — remaining static bodies dropped this frame."),
			*GetNameSafe(GetOwner()), MaxColliders);
	}
	UE_LOG(LogRopeCollision, VeryVerbose, TEXT("StaticBodyProvider on %s: built %d box(es) + %d capsule(s) + %d convex from %d overlapped component(s)."),
		*GetNameSafe(GetOwner()), Boxes.Num(), Capsules.Num(), Convexes.Num(), Seen.Num());
}

bool URopeStaticBodyProvider::AppendBodyColliders(const UBodySetup& Setup, const FTransform& CompTM,
	const FTransform& PrevCompTM, float InvDeltaTime, int32 MaxColliders, int32 MaxConvexPlanes)
{
	const FVector Scale3D = CompTM.GetScale3D();
	const auto BudgetLeft = [this, MaxColliders]() { return Boxes.Num() + Capsules.Num() + Convexes.Num() < MaxColliders; };

	// sphyl: 스킨 캡슐 provider와 동일한 스케일 규약(GetScaledRadius/CylinderLength). 동적이면 prev 끝점도
	// 채워 FCapsuleCollider의 표면 속도/substep CCD machinery를 그대로 탄다(셰이더/GPU 변경 불필요).
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
		FRopeStaticCapsuleCollider Cap(Center + Axis * HalfLen, Center - Axis * HalfLen, Sphyl.GetScaledRadius(Scale3D));
		if (InvDeltaTime > 0.0f)
		{
			const FTransform PrevElemTM = Sphyl.GetTransform() * PrevCompTM;
			const FVector PrevAxis = PrevElemTM.GetUnitAxis(EAxis::Z);
			const FVector PrevCenter = PrevElemTM.GetLocation();
			Cap.PrevA = PrevCenter + PrevAxis * HalfLen;
			Cap.PrevB = PrevCenter - PrevAxis * HalfLen;
			Cap.InvDeltaTime = InvDeltaTime;
		}
		Capsules.Add(MoveTemp(Cap));
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
		FRopeStaticCapsuleCollider Cap(Center, Center, ScaledRadius);
		if (InvDeltaTime > 0.0f)
		{
			const FVector PrevCenter = PrevCompTM.TransformPosition(Sphere.Center);
			Cap.PrevA = PrevCenter;
			Cap.PrevB = PrevCenter;
			Cap.InvDeltaTime = InvDeltaTime;
		}
		Capsules.Add(MoveTemp(Cap));
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
			FRopeBoxCollider BoxCol(ElemTM.GetLocation(), ElemTM.GetRotation(), Half);
			if (InvDeltaTime > 0.0f)
			{
				const FTransform PrevElemTM = Box.GetTransform() * PrevCompTM;
				BoxCol.PrevCenter = PrevElemTM.GetLocation();
				BoxCol.PrevRot = PrevElemTM.GetRotation();
				BoxCol.InvDeltaTime = InvDeltaTime;
			}
			Boxes.Add(MoveTemp(BoxCol));
		}
		else
		{
			// 전단: 6평면 컨벡스로 정확히(평면은 전단 행렬로도 정확 변환). 바디-로컬 평면 + 컴포넌트 강체로
			// 저장해 동적(움직이는 전단 박스)도 지원. M1의 min-scale 근사를 대체.
			TArray<FPlane> LocalPlanes;
			const FMatrix BodyLocalM = ComposeConvexBodyLocal(Box.GetTransform(), Scale3D);
			TransformPlanesToWorld(MakeBoxLocalPlanes(HalfLocal), BodyLocalM, LocalPlanes);
			const FBox LB = FBox(-HalfLocal, HalfLocal).TransformBy(BodyLocalM);
			if (LocalPlanes.Num() == 6 && LB.IsValid)
			{
				FRopeConvexCollider Cv(MoveTemp(LocalPlanes), LB, CompTM.GetRotation(), CompTM.GetTranslation());
				if (InvDeltaTime > 0.0f)
				{
					Cv.PrevRot = PrevCompTM.GetRotation();
					Cv.PrevTrans = PrevCompTM.GetTranslation();
					Cv.InvDeltaTime = InvDeltaTime;
				}
				Convexes.Add(MoveTemp(Cv));
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
		TArray<FPlane> ElemPlanes;
		Convex.GetPlanes(ElemPlanes);
		const FMatrix BodyLocalM = ComposeConvexBodyLocal(Convex.GetTransform(), Scale3D);

		if (ElemPlanes.Num() >= 4 && ElemPlanes.Num() <= MaxConvexPlanes && Convex.ElemBox.IsValid)
		{
			TArray<FPlane> LocalPlanes;
			TransformPlanesToWorld(ElemPlanes, BodyLocalM, LocalPlanes); // elem->바디로컬(강체 제외)
			const FBox LB = Convex.ElemBox.TransformBy(BodyLocalM);
			if (LocalPlanes.Num() >= 4 && LB.IsValid)
			{
				FRopeConvexCollider Cv(MoveTemp(LocalPlanes), LB, CompTM.GetRotation(), CompTM.GetTranslation());
				if (InvDeltaTime > 0.0f)
				{
					Cv.PrevRot = PrevCompTM.GetRotation();
					Cv.PrevTrans = PrevCompTM.GetTranslation();
					Cv.InvDeltaTime = InvDeltaTime;
				}
				Convexes.Add(MoveTemp(Cv));
				continue;
			}
		}

		// 폴백: ElemBox OBB 근사(미쿡/평면 과다/무효). 거칠지만 충돌 유지 > 통째 누락. 정적으로 처리(드문 경로).
		if (Convex.ElemBox.IsValid)
		{
			const FMatrix M = ComposeConvexToWorld(Convex.GetTransform(), Scale3D, CompTM);
			const FVector CenterW = M.TransformPosition(Convex.ElemBox.GetCenter());
			const FQuat   RotW = M.GetMatrixWithoutScale().ToQuat();
			const FVector HalfW = Convex.ElemBox.GetExtent() * static_cast<float>(Scale3D.GetAbsMin());
			Boxes.Add(FRopeBoxCollider(CenterW, RotW, HalfW));
			UE_LOG(LogRopeCollision, Verbose,
				TEXT("StaticBodyProvider on %s: convex elem unusable (%d planes) — falling back to ElemBox OBB."),
				*GetNameSafe(GetOwner()), ElemPlanes.Num());
		}
	}

	return true;
}

bool URopeStaticBodyProvider::AppendInstancedBodyColliders(UInstancedStaticMeshComponent& ISM,
	const FBox& RopeBounds, int32 MaxColliders, int32 MaxConvexPlanes)
{
	// 모든 인스턴스가 공유하는 메시 콜리전(로컬 셰이프). ISM은 GetBodySetup을 오버라이드하지 않아
	// UStaticMeshComponent의 것(= 메시 BodySetup)을 상속한다.
	const UBodySetup* Setup = ISM.GetBodySetup();
	if (!Setup)
	{
		return true; // 콜리전 없음 — 스킵(예산 소진 아님).
	}

	// 로프 bounds와 겹치는 인스턴스만 열거(월드 공간 박스) — 밀집 폴리지에서도 근접분만 추린다.
	const TArray<int32> Indices = ISM.GetInstancesOverlappingBox(RopeBounds, /*bBoxInWorldSpace=*/true);
	for (int32 Index : Indices)
	{
		FTransform InstanceTM;
		if (!ISM.GetInstanceTransform(Index, InstanceTM, /*bWorldSpace=*/true))
		{
			continue;
		}
		// 인스턴스 월드 트랜스폼(= 인스턴스 로컬 × 컴포넌트→월드)으로 공유 콜리전을 배치한다 — 일반 스태틱
		// 메시가 ComponentTransform으로 배치하는 것과 동일하므로 AppendBodyColliders를 그대로 재사용.
		// 인스턴스별 prev 추적은 미지원 → 정적(prev=curr, InvDt=0)으로 처리.
		if (!AppendBodyColliders(*Setup, InstanceTM, InstanceTM, 0.0f, MaxColliders, MaxConvexPlanes))
		{
			return false; // 예산 소진(인스턴스는 다른 바디와 같은 MaxColliders 예산을 공유).
		}
	}
	return true;
}
