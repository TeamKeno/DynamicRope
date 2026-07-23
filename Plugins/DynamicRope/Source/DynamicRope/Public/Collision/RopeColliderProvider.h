// Copyright Epic Games, Inc. All Rights Reserved.
//
// 매 프레임 rope solver에 collider를 공급하는 컴포넌트/오브젝트. skeletal provider는
// 후보 bone들에 대해 per-bone collider(capsule/SDF)를 빌드한다.
//
// 2026-07 수집 방식 변경(노션 "GatherCollidersForRope O(로프×풀)" 이슈): 이전 계약(flat 배열 반환)은
// provider가 gather 시점에 이미 아는 "region(=로프)↔collider" 매핑을 버렸고, 서브시스템이 로프마다
// 풀 전체를 bounds 재테스트해 O(로프 수 × 전체 콜라이더)로 매핑을 재구성했다. 이제 provider가
// flat 디둡 풀 + region별 인덱스 매핑을 함께 돌려줘 그 재-컬을 없앤다.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
// RopeColliderGather::MapCollidersToRegionsByBounds가 GetWorldBounds를 쓴다.
#include "Collision/RopeCollider.h"
#include "RopeColliderProvider.generated.h"

class AActor;

/**
 * 프레임당 1회의 collider 수집 입출력 묶음. 서브시스템(BuildFrameColliders)이 provider마다 하나씩 만들어
 * 넘기고, provider는 풀(Colliders)과 — 가능하면 — region별 인덱스 매핑을 채운다.
 */
struct FRopeColliderGatherContext
{
	/**
	 * 입력: 물리/조준 활성 영역. 로프 수를 N이라 할 때 레이아웃은 [0, N) 물리 region,
	 * [N, 2N) 같은 로프의 조준 region이다. 조준 중이 아닌 로프의 조준 region과 region이 없는 로프는
	 * !IsValid 박스로 자리를 유지한다 — provider는 !IsValid를 건너뛴다.
	 * bounds-aware provider는 이 리스트로 멀리 동떨어진 로프 사이 빈 공간을 스캔에서 배제한다.
	 */
	TArrayView<const FBox> RopeRegions;

	/** RopeRegions 앞쪽에 있는 물리 region 수(N). 뒤쪽 N개는 같은 순서의 조준 region이다. */
	int32 NumPhysicsRegions = 0;

	/**
	 * 입력(선택): region 처리 우선순위 — 앞에 오는 region 인덱스부터 스캔한다. 전역 추출 상한이 있는
	 * provider(정적 바디)용: 선착순 소진이면 앞 순서의 한가한 로프 주변 잡동사니가 상한을 먼저 먹어
	 * 활성 로프가 충돌을 굶을 수 있다 — 서브시스템이 활성(사용 중 페이즈, 비슬립) 물리 region을
	 * 먼저 두고, 조준 region은 모든 물리 region 뒤에 둔다.
	 * 순서만 바꿀 뿐 region 인덱스 자체는 불변이라 RegionColliderIndices 매핑에는 영향이 없다.
	 * 비어 있으면 인덱스 순서(0..N-1)로 처리한다. 상한 없는 provider는 무시해도 된다.
	 */
	TArrayView<const int32> RegionGatherOrder;

	/**
	 * 출력: flat 디둡 풀(이전 계약과 동일). collider↔region은 다대다(겹치는 영역)라 풀은 provider가
	 * 컴포넌트/본 단위로 디둡해 1회만 담고, 다중 소속은 아래 매핑으로 표현한다. 가리키는 collider들은
	 * provider 소유 스토리지이며 이번 프레임 solve가 끝날 때까지 유효해야 한다.
	 */
	TArray<IRopeCollider*> Colliders;

	/**
	 * 출력(권장): region r이 문 콜라이더의 Colliders 인덱스 리스트. bHasRegionMapping=true일 때만
	 * 소비되며, 그때 길이는 RopeRegions.Num()과 같아야 한다(서브시스템이 불일치 시 폴백으로 강등).
	 */
	TArray<TArray<int32>> RegionColliderIndices;

	/**
	 * true면 서브시스템이 RegionColliderIndices로 로프별 배정을 바로 끝낸다(bounds 재테스트 없음 —
	 * 이 변경의 핵심). false(기본)면 이전 방식대로 서브시스템이 collider bounds로 로프별 재-컬한다 —
	 * 매핑을 만들 수 없는 provider의 합법적 폴백 경로.
	 */
	bool bHasRegionMapping = false;

	/**
	 * 출력(선택): Colliders와 **평행한** 콜라이더별 출처 액터 — 그 셰이프를 소유한 컴포넌트의 owner.
	 * 채우면 서브시스템의 "자기 owner 제외"가 provider 단위가 아니라 **콜라이더(=바디) 단위**로
	 * 판정된다. 월드 지오메트리를 서빙하면서 로프 소유 액터에 붙은 셰이프까지 함께 긁을 수 있는
	 * provider(정적 바디)에 필요하다 — provider 단위로 면제하면 그런 셰이프가 제 로프를 미는 push-out
	 * 콜라이더가 되기 때문이다(ProvidesWorldStaticColliders 주석 참조).
	 * 길이가 Colliders와 다르면 신뢰하지 않고 provider 단위 판정으로 폴백한다. 출처를 모르는 자리는
	 * nullptr로 둘 것. 수명은 collider 포인터와 같다(해당 프레임).
	 */
	TArray<const AActor*> ColliderSourceActors;
};

UINTERFACE(MinimalAPI)
class URopeColliderProvider : public UInterface
{
	GENERATED_BODY()
};

class IRopeColliderProvider
{
	GENERATED_BODY()

public:
	/**
	 * 이번 프레임의 collider를 수집한다(broad phase는 여기서 수행). 서브시스템이 프레임당 1회,
	 * 물리/조준 region 리스트(Gather.RopeRegions)와 함께 호출한다. 공개 레이아웃 계약은
	 * Gather.NumPhysicsRegions 주석을 따른다.
	 *  - region을 실제로 쓰는 provider(정적 바디): region별 오버랩 결과를 풀에 디둡해 담고, 어느
	 *    region이 어느 콜라이더를 물었는지 RegionColliderIndices로 함께 돌려준다.
	 *  - region 무시 provider(스켈레톤): 전 collider를 빌드해 풀에 담은 뒤
	 *    RopeColliderGather::MapCollidersToRegionsByBounds로 매핑을 만들면 된다(유니언 선-거절이라
	 *    원거리 로프는 O(1)에 통째로 떨어져 나간다).
	 *  - 매핑을 못 만들면 bHasRegionMapping=false로 두면 된다 — 서브시스템이 bounds 재-컬로 폴백.
	 */
	virtual void GatherColliders(FRopeColliderGatherContext& Gather) = 0;

	/**
	 * 이 provider가 정적 월드 지오메트리 collider를 공급하는지(예: URopeStaticBodyProvider). true면
	 * 서브시스템의 로프별 "자기 owner provider 제외"에서 면제된다 — 정적 월드는 "던진 본인의 몸"이
	 * 될 수 없는데, 로프 소유 액터에 붙였다는 이유만으로 월드 충돌이 조용히 사라지는 것을 막는다.
	 *
	 * 주의: 이 면제는 provider 단위라 그 자체로는 너무 넓다. 이런 provider는 월드를 훑으면서 로프 소유
	 * 액터에 붙은 셰이프(테더 프록시·팁 메쉬·든 무기 등)도 같이 긁을 수 있고, 그것이 그대로 제 로프를
	 * 미는 push-out 콜라이더가 된다. 그래서 Gather.ColliderSourceActors로 콜라이더별 출처를 함께
	 * 돌려줘야 서브시스템이 바디 단위로 소유자를 걸러낼 수 있다(월드 지오메트리는 출처가 다른 액터라
	 * 그대로 남는다).
	 */
	virtual bool ProvidesWorldStaticColliders() const { return false; }
};

namespace RopeColliderGather
{
	/**
	 * 스켈레톤형(전 콜라이더 빌드 후 배정) provider 공용 매핑 헬퍼: 풀의 [StartIndex, Colliders.Num())
	 * 구간 — 이 provider가 이번 호출에 추가한 콜라이더들 — 을 각 region에 배정한다.
	 * 그룹 유니언 bounds로 먼저 거절하므로 멀리 있는 영역은 region당 1회 비교로 끝나고(메시당 O(region)),
	 * 유니언에 걸린 가까운 로프만 콜라이더별 bounds로 정밀 배정한다(이전 서브시스템 재-컬과 동일 판정).
	 */
	inline void MapCollidersToRegionsByBounds(FRopeColliderGatherContext& Gather, int32 StartIndex)
	{
		Gather.bHasRegionMapping = true;
		Gather.RegionColliderIndices.SetNum(Gather.RopeRegions.Num());
		const int32 EndIndex = Gather.Colliders.Num();
		if (StartIndex >= EndIndex)
		{
			return;
		}

		// collider bounds 1회 캐시 + 그룹 유니언(선-거절용).
		TArray<FBox, TInlineAllocator<64>> Bounds;
		Bounds.Reserve(EndIndex - StartIndex);
		FBox GroupBounds(ForceInit);
		for (int32 i = StartIndex; i < EndIndex; ++i)
		{
			const FBox Box = Gather.Colliders[i] ? Gather.Colliders[i]->GetWorldBounds() : FBox(ForceInit);
			Bounds.Add(Box);
			if (Box.IsValid)
			{
				GroupBounds += Box;
			}
		}
		if (!GroupBounds.IsValid)
		{
			return;
		}

		for (int32 r = 0; r < Gather.RopeRegions.Num(); ++r)
		{
			const FBox& Region = Gather.RopeRegions[r];
			if (!Region.IsValid || !GroupBounds.Intersect(Region))
			{
				continue;
			}
			TArray<int32>& Out = Gather.RegionColliderIndices[r];
			for (int32 i = StartIndex; i < EndIndex; ++i)
			{
				const FBox& Box = Bounds[i - StartIndex];
				if (Box.IsValid && Box.Intersect(Region))
				{
					Out.Add(i);
				}
			}
		}
	}
}
