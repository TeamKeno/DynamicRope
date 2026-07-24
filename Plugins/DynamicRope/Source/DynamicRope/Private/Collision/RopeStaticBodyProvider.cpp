// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeStaticBodyProvider.h"
#include "DynamicRopeLog.h"
#include "Subsystem/RopeSimSubsystem.h"
// 콜라이더 예산/컨벡스 평면 상한(단일 소스)
#include "Settings/DynamicRopeSettings.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/OverlapResult.h"
#include "Engine/World.h"
#include "PhysicsEngine/BodySetup.h"
// FKConvexElem::GetPlanes(월드 평면 추출)
#include "PhysicsEngine/ConvexElem.h"
// 심플 콜리전 → push-out 콜라이더 추출(WrapTarget provider와 공용 헬퍼). 지오메트리 수학 free 함수도 이리로 이동.
#include "Collision/RopeBodyColliderExtraction.h"

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

void URopeStaticBodyProvider::GatherColliders(FRopeColliderGatherContext& Gather)
{
	// 프레임당 1회만 빌드(디둡). 물리/조준 RopeRegions를 한 번에 받아 첫 중앙 수집 pass에서 추출한
	// 오버랩 결과(+추출 그룹)를 그 프레임의 모든 region이 공유한다. 같은 프레임의 후속 pass는 현재
	// region에 대한 매핑만 다시 만들고, backing pool은 재사용한다.
	const uint64 Frame = GFrameCounter;
	if (BuiltFrame != Frame)
	{
		BuiltFrame = Frame;
		Boxes.Reset();
		Capsules.Reset();
		Convexes.Reset();
		Groups.Reset();
		BoxWorldBounds.Reset();
		CapWorldBounds.Reset();
		CvxWorldBounds.Reset();
		BoxSourceActors.Reset();
		CapSourceActors.Reset();
		CvxSourceActors.Reset();
		BuildColliders(Gather);
	}

	const int32 PoolBase = Gather.Colliders.Num();
	Gather.Colliders.Reserve(PoolBase + Boxes.Num() + Capsules.Num() + Convexes.Num());
	for (FRopeBoxCollider& Box : Boxes)
	{
		Gather.Colliders.Add(&Box);
	}
	for (FRopeStaticCapsuleCollider& Cap : Capsules)
	{
		Gather.Colliders.Add(&Cap);
	}
	for (FRopeConvexCollider& Convex : Convexes)
	{
		Gather.Colliders.Add(&Convex);
	}

	// 콜라이더별 출처 액터를 풀과 같은 순서(boxes → capsules → convexes)로 흘려보낸다 — 서브시스템이
	// 소유자 제외를 provider가 아니라 바디 단위로 판정하는 근거다. 앞선 append분(PoolBase)은 이 provider
	// 소관이 아니므로 nullptr로 자리만 채워 배열을 풀과 평행하게 유지한다.
	Gather.ColliderSourceActors.Reset();
	Gather.ColliderSourceActors.SetNumZeroed(PoolBase);
	Gather.ColliderSourceActors.Reserve(Gather.Colliders.Num());
	Gather.ColliderSourceActors.Append(BoxSourceActors);
	Gather.ColliderSourceActors.Append(CapSourceActors);
	Gather.ColliderSourceActors.Append(CvxSourceActors);

	// region 매핑: 추출 그룹(=오버랩이 이미 판정한 컴포넌트 단위) 유니언 선-거절 → 히트한 그룹만
	// 콜라이더별 bounds로 정밀 배정. 풀 순서는 위 append와 동일(boxes → capsules → convexes)이라
	// 타입별 로컬 인덱스 + 오프셋으로 flat 인덱스를 만든다.
	Gather.bHasRegionMapping = true;
	Gather.RegionColliderIndices.SetNum(Gather.RopeRegions.Num());
	const int32 CapFlatBase = PoolBase + Boxes.Num();
	const int32 CvxFlatBase = CapFlatBase + Capsules.Num();
	for (int32 r = 0; r < Gather.RopeRegions.Num(); ++r)
	{
		const FBox& Region = Gather.RopeRegions[r];
		if (!Region.IsValid)
		{
			continue;
		}
		TArray<int32>& Out = Gather.RegionColliderIndices[r];
		for (const FExtractedGroup& Group : Groups)
		{
			if (!Group.Bounds.IsValid || !Group.Bounds.Intersect(Region))
			{
				continue;
			}
			for (int32 i = Group.BoxStart; i < Group.BoxStart + Group.BoxCount; ++i)
			{
				if (BoxWorldBounds[i].Intersect(Region))
				{
					Out.Add(PoolBase + i);
				}
			}
			for (int32 i = Group.CapStart; i < Group.CapStart + Group.CapCount; ++i)
			{
				if (CapWorldBounds[i].Intersect(Region))
				{
					Out.Add(CapFlatBase + i);
				}
			}
			for (int32 i = Group.CvxStart; i < Group.CvxStart + Group.CvxCount; ++i)
			{
				if (CvxWorldBounds[i].Intersect(Region))
				{
					Out.Add(CvxFlatBase + i);
				}
			}
		}
	}
}

void URopeStaticBodyProvider::RecordExtractedGroup(int32 BoxStart, int32 CapStart, int32 CvxStart,
	const AActor* SourceActor)
{
	FExtractedGroup Group;
	Group.BoxStart = BoxStart;
	Group.BoxCount = Boxes.Num() - BoxStart;
	Group.CapStart = CapStart;
	Group.CapCount = Capsules.Num() - CapStart;
	Group.CvxStart = CvxStart;
	Group.CvxCount = Convexes.Num() - CvxStart;
	if (Group.BoxCount + Group.CapCount + Group.CvxCount <= 0)
	{
		return;
	}
	// 월드 AABB를 collider당 1회 계산해 Group.Bounds와 캐시에 함께 넣는다 — 아래 GatherColliders의 region
	// 매핑이 collider×region마다 GetWorldBounds를 재계산하지 않게 한다(#10). 캐시는 collider 배열과 평행.
	// 출처 액터도 같은 루프에서 collider별로 채운다(배열은 collider 배열과 평행).
	BoxWorldBounds.SetNum(Boxes.Num());
	BoxSourceActors.SetNumZeroed(Boxes.Num());
	for (int32 i = Group.BoxStart; i < Group.BoxStart + Group.BoxCount; ++i)
	{
		const FBox WB = Boxes[i].GetWorldBounds();
		BoxWorldBounds[i] = WB;
		BoxSourceActors[i] = SourceActor;
		Group.Bounds += WB;
	}
	CapWorldBounds.SetNum(Capsules.Num());
	CapSourceActors.SetNumZeroed(Capsules.Num());
	for (int32 i = Group.CapStart; i < Group.CapStart + Group.CapCount; ++i)
	{
		const FBox WB = Capsules[i].GetWorldBounds();
		CapWorldBounds[i] = WB;
		CapSourceActors[i] = SourceActor;
		Group.Bounds += WB;
	}
	CvxWorldBounds.SetNum(Convexes.Num());
	CvxSourceActors.SetNumZeroed(Convexes.Num());
	for (int32 i = Group.CvxStart; i < Group.CvxStart + Group.CvxCount; ++i)
	{
		const FBox WB = Convexes[i].GetWorldBounds();
		CvxWorldBounds[i] = WB;
		CvxSourceActors[i] = SourceActor;
		Group.Bounds += WB;
	}
	Groups.Add(Group);
}

void URopeStaticBodyProvider::BuildColliders(const FRopeColliderGatherContext& Gather)
{
	TArrayView<const FBox> RopeRegions = Gather.RopeRegions;
	UWorld* World = GetWorld();
	if (!World || RopeRegions.Num() == 0)
	{
		// 활성 로프가 없으면(빈 region 리스트) 스캔할 이유가 없다.
		return;
	}

	// 콜라이더 예산/컨벡스 평면 상한/동적 포함 여부는 Project Settings에서 단일 관리.
	const UDynamicRopeSettings* Settings = UDynamicRopeSettings::Get();
	const int32 MaxColliders = Settings ? FMath::Max(1, Settings->StaticBodyMaxColliders) : 128;
	const int32 MaxConvexPlanes = Settings ? FMath::Max(4, Settings->StaticBodyMaxConvexPlanes) : 32;
	const bool  bIncludeDynamic = Settings ? Settings->bIncludeWorldDynamic : true;

	FCollisionObjectQueryParams ObjParams(ECC_WorldStatic);
	if (bIncludeDynamic)
	{
		ObjParams.AddObjectTypesToQuery(ECC_WorldDynamic);
	}
	const FCollisionQueryParams QueryParams(FName(TEXT("RopeStaticBodyGather")), /*bInTraceComplex*/ false);

	// 표면 속도(드래그/CCD)용 프레임 dt. 이번 프레임 처리한 컴포넌트의 (curr - prev)/dt 로 산출한다.
	const float FrameDt = World->GetDeltaSeconds();
	const float InvDt = (FrameDt > KINDA_SMALL_NUMBER) ? (1.0f / FrameDt) : 0.0f;
	// 이번 프레임 컴포넌트 트랜스폼(다음 프레임 prev 소스). 처리한 것만 담아 파괴/이탈 항목은 자연히 만료.
	// 프레임 전역 — 여러 region에 걸쳐 축적하고 루프 종료 후 딱 1회 스왑(표면 속도 continuity 불변식).
	TMap<TWeakObjectPtr<UPrimitiveComponent>, FTransform> CurrCompXforms;

	// 프레임 전역 디둡 상태(region 루프 바깥). 일반 컴포넌트는 컴포넌트 단위로 1회만 추출.
	TSet<const UPrimitiveComponent*> Seen;
	// ISM은 region마다 다른 인스턴스가 걸릴 수 있어 컴포넌트가 아니라 인스턴스 인덱스 단위로 디둡한다.
	TMap<UInstancedStaticMeshComponent*, TSet<int32>> SeenInstances;

	// 브로드페이즈: 로프별 활성 region마다 오버랩(멀리 떨어진 로프 사이 빈 공간은 스캔에서 배제 —
	// 전 로프 union AABB의 낭비/예산 경합 제거). region 간 중복 결과는 위 디둡 상태로 걸러 프레임당 1회만 추출.
	// 처리 순서는 RegionGatherOrder(활성 로프 먼저 — 서브시스템이 정렬): 전역 상한(MaxColliders)이 걸리는
	// 프레임에 뒤로 밀려 스캔을 못 받는 쪽이 한가한/잠든 로프가 되게 한다. 순서일 뿐 region 인덱스는
	// 불변이라 추출 그룹/매핑에는 영향이 없다. 순서 리스트가 비었거나 길이가 다르면 인덱스 순서 폴백.
	const bool bUseGatherOrder = Gather.RegionGatherOrder.Num() == RopeRegions.Num();
	TArray<FOverlapResult> Overlaps;
	bool bBudgetClipped = false;
	for (int32 OrderSlot = 0; OrderSlot < RopeRegions.Num(); ++OrderSlot)
	{
		const int32 RegionIndex = bUseGatherOrder ? Gather.RegionGatherOrder[OrderSlot] : OrderSlot;
		if (!RopeRegions.IsValidIndex(RegionIndex))
		{
			continue;
		}
		const FBox& Region = RopeRegions[RegionIndex];
		if (bBudgetClipped)
		{
			break;
		}
		if (!Region.IsValid)
		{
			continue;
		}
		Overlaps.Reset();
		World->OverlapMultiByObjectType(Overlaps, Region.GetCenter(), FQuat::Identity, ObjParams,
			FCollisionShape::MakeBox(Region.GetExtent()), QueryParams);

		for (const FOverlapResult& Overlap : Overlaps)
		{
			UPrimitiveComponent* Prim = Overlap.Component.Get();
			if (!Prim)
			{
				continue;
			}
			// 트리거/오버랩 볼륨 배제: 오브젝트 타입 오버랩은 상대의 채널 응답을 안 보고 QueryOnly 바디도
			// 잡으므로, 감지 전용 볼륨(압력판 Trigger 등 눈에 안 보이는 QueryOnly 박스)이 그대로 solid
			// 콜라이더가 되어 로프를 민다(2026-07-24 스네어 끌어올림 떨림). 로프는 물리 오브젝트처럼
			// 행동한다는 계약으로 걸러낸다 — "물리 충돌이 켜져 있고(ECollisionEnabled에 Physics 포함)
			// PhysicsBody 채널을 Block하는" 셰이프만 밀어낼 자격이 있다. 보이지 않아도 물리로 막는
			// BlockingVolume류는 통과(랙돌/프랍을 막으니 로프도 막는 게 일관) — 의도적 예외는
			// IgnoredComponents가 담당한다.
			if (!Prim->IsPhysicsCollisionEnabled()
				|| Prim->GetCollisionResponseToChannel(ECC_PhysicsBody) != ECR_Block)
			{
				continue;
			}
			// ISM/HISM(M3): 인스턴스별 처리 — 한 컴포넌트가 공유 메시 콜리전을 여러 인스턴스에 배치한다.
			// GetBodySetup은 인스턴스 트랜스폼을 모르는 원본(로컬) 셰이프를 주므로, 근접 인스턴스마다 그
			// 월드 트랜스폼으로 추출해야 한다(HISM도 이 베이스로 캐치). 컴포넌트 단위 Seen에 넣지 않고
			// 인스턴스 인덱스 단위(SeenInstances)로 디둡 — 다른 region의 다른 인스턴스를 놓치지 않도록.
			// 인스턴스별 prev 추적은 미지원 → 인스턴스는 정적 스냅샷으로 처리(내부에서 prev=curr, InvDt=0).
			if (UInstancedStaticMeshComponent* ISM = Cast<UInstancedStaticMeshComponent>(Prim))
			{
				if (IgnoredComponents.Contains(Prim))
				{
					continue;
				}
				// 그룹 기록: 이 호출이 추가한 인스턴스 콜라이더 묶음(예산 클립으로 부분 추출이어도
				// 추가된 만큼은 기록해 매핑에서 빠지지 않게 한다).
				const int32 BoxStart = Boxes.Num(), CapStart = Capsules.Num(), CvxStart = Convexes.Num();
				const bool bWithinBudget = AppendInstancedBodyColliders(*ISM, Region, SeenInstances.FindOrAdd(ISM), MaxColliders, MaxConvexPlanes);
				RecordExtractedGroup(BoxStart, CapStart, CvxStart, ISM->GetOwner());
				if (!bWithinBudget)
				{
					bBudgetClipped = true;
					break;
				}
				continue;
			}
			// 일반 컴포넌트: 프레임 전역 Seen으로 여러 region에 걸쳐도 1회만 추출(오버랩이 바디별로 중복 보고돼도 디둡).
			if (Seen.Contains(Prim))
			{
				continue;
			}
			Seen.Add(Prim);
			if (IgnoredComponents.Contains(Prim))
			{
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

			{
				// 그룹 기록: 이 컴포넌트가 추가한 콜라이더 묶음. region 오버랩이 준 근접 정보를 보존해
				// 서브시스템 재-컬 없이 로프별 배정에 쓴다(겹치는 region은 매핑 단계에서 양쪽에 배정).
				const int32 BoxStart = Boxes.Num(), CapStart = Capsules.Num(), CvxStart = Convexes.Num();
				const bool bWithinBudget = AppendBodyColliders(*Setup, CompTM, PrevTM, CompInvDt, MaxColliders, MaxConvexPlanes);
				RecordExtractedGroup(BoxStart, CapStart, CvxStart, Prim->GetOwner());
				if (!bWithinBudget)
				{
					bBudgetClipped = true;
					break;
				}
			}
		}
	}

	// 다음 프레임 prev 소스로 교체(프레임당 1회 스왑).
	PrevCompXforms = MoveTemp(CurrCompXforms);

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
	// 실제 추출은 공용 헬퍼(WrapTarget provider와 공유)로 위임한다. 이 provider의 멤버 배열을 out으로 넘기고,
	// convex 폴백 로그만 이 provider 문맥(owner 이름)으로 남긴다. 예산/prev-트랜스폼 계약은 헬퍼가 그대로 유지.
	return RopeBodyColliderExtraction::AppendBodyColliders(
		Setup, CompTM, PrevCompTM, InvDeltaTime, MaxColliders, MaxConvexPlanes,
		Boxes, Capsules, Convexes,
		[this](int32 NumPlanes)
		{
			UE_LOG(LogRopeCollision, Verbose,
				TEXT("StaticBodyProvider on %s: convex elem unusable (%d planes) — falling back to ElemBox OBB."),
				*GetNameSafe(GetOwner()), NumPlanes);
		});
}

bool URopeStaticBodyProvider::AppendInstancedBodyColliders(UInstancedStaticMeshComponent& ISM,
	const FBox& Region, TSet<int32>& SeenIndices, int32 MaxColliders, int32 MaxConvexPlanes)
{
	// 모든 인스턴스가 공유하는 메시 콜리전(로컬 셰이프). ISM은 GetBodySetup을 오버라이드하지 않아
	// UStaticMeshComponent의 것(= 메시 BodySetup)을 상속한다.
	const UBodySetup* Setup = ISM.GetBodySetup();
	if (!Setup)
	{
		// 콜리전 없음 — 스킵(예산 소진 아님).
		return true;
	}

	// region과 겹치는 인스턴스만 열거(월드 공간 박스) — 밀집 폴리지에서도 근접분만 추린다.
	// 여러 region에 걸치는 ISM은 이미 추출한 인덱스(SeenIndices)를 건너뛰어 콜라이더 중복을 막는다.
	const TArray<int32> Indices = ISM.GetInstancesOverlappingBox(Region, /*bBoxInWorldSpace=*/true);
	for (int32 Index : Indices)
	{
		bool bAlreadySeen = false;
		SeenIndices.Add(Index, &bAlreadySeen);
		if (bAlreadySeen)
		{
			continue;
		}
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
			// 예산 소진(인스턴스는 다른 바디와 같은 MaxColliders 예산을 공유).
			return false;
		}
	}
	return true;
}
