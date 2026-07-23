// Copyright Epic Games, Inc. All Rights Reserved.
//
// 콜라이더 수집의 소유자 제외 판정(RopeColliderGather::IsExcludedOwnerBody) 단위 테스트.
// 이 판정은 정적 월드 provider처럼 provider 단위 제외를 면제받는 경로에서 "월드 지오메트리는 남기고
// 로프 소유 액터의 셰이프만 뺀다"를 담당한다 — 잘못 넓히면 바닥이 사라지고, 잘못 좁히면 로프를
// 따라다니는 자기 셰이프(테더 프록시/팁/무기)가 제 로프를 밀어 요동이 재발한다. 양쪽 실패를 다 고정한다.
// 액터 포인터는 식별(비교)에만 쓰이므로 트랜지언트 객체로 충분하다 — 역참조하지 않는다.

#include "Misc/AutomationTest.h"
#include "Collision/RopeColliderProvider.h"
#include "GameFramework/Actor.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	// 비교 전용 더미 액터(월드 불필요). 이름만 다르면 포인터가 달라 식별에 충분하다.
	AActor* MakeIdentityActor()
	{
		return NewObject<AActor>(GetTransientPackage());
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeColliderGatherOwnerBodyTest,
	"DynamicRope.Collision.Gather.OwnerBodyExclusion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeColliderGatherOwnerBodyTest::RunTest(const FString& Parameters)
{
	AActor* RopeOwner = MakeIdentityActor();
	AActor* WorldProp = MakeIdentityActor();
	AActor* WrapTarget = MakeIdentityActor();

	// 풀: [0] 로프 소유 액터의 셰이프(테더 프록시/팁), [1] 바닥 같은 월드 지오메트리,
	//     [2] 감는 대상(다른 액터), [3] 출처 미상(nullptr).
	const TArray<const AActor*> SourceActors = { RopeOwner, WorldProp, WrapTarget, nullptr };

	// 기본(bIncludeOwnerColliders = false): owner 몸만 빠지고 나머지는 전부 남는다.
	TestTrue(TEXT("로프 소유 액터의 셰이프는 제외된다"),
		RopeColliderGather::IsExcludedOwnerBody(SourceActors, 0, RopeOwner));
	TestFalse(TEXT("다른 액터의 월드 지오메트리(바닥/기둥)는 남는다"),
		RopeColliderGather::IsExcludedOwnerBody(SourceActors, 1, RopeOwner));
	TestFalse(TEXT("감는 대상(cross-actor)은 남는다"),
		RopeColliderGather::IsExcludedOwnerBody(SourceActors, 2, RopeOwner));
	TestFalse(TEXT("출처 미상(nullptr) 항목은 제외되지 않는다"),
		RopeColliderGather::IsExcludedOwnerBody(SourceActors, 3, RopeOwner));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeColliderGatherOwnerBodyFallbackTest,
	"DynamicRope.Collision.Gather.OwnerBodyExclusionFallbacks",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeColliderGatherOwnerBodyFallbackTest::RunTest(const FString& Parameters)
{
	AActor* RopeOwner = MakeIdentityActor();
	const TArray<const AActor*> SourceActors = { RopeOwner, RopeOwner };

	// 옵트인(bIncludeOwnerColliders = true → OwnerToExclude = nullptr): 아무것도 제외하지 않는다.
	// 프롭에 얹은 로프가 자기 받침대와 계속 충돌하게 하는 탈출구라, 이 성질이 깨지면 그 구성이 조용히 망가진다.
	TestFalse(TEXT("owner 콜라이더 옵트인이면 owner 몸도 남는다"),
		RopeColliderGather::IsExcludedOwnerBody(SourceActors, 0, nullptr));

	// 출처를 안 주는 provider(스켈레톤/랩 대상): 빈 배열 → 아무것도 제외하지 않고 provider 단위 판정에 맡긴다.
	const TArray<const AActor*> NoAttribution;
	TestFalse(TEXT("출처 미제공 provider는 콜라이더 단위로 제외하지 않는다"),
		RopeColliderGather::IsExcludedOwnerBody(NoAttribution, 0, RopeOwner));

	// 길이가 어긋난 배열(provider 버그): 범위 밖 인덱스는 제외하지 않는다 — 충돌이 사라지는 쪽으로 실패하지 않는다.
	TestFalse(TEXT("길이 불일치 시 범위 밖 인덱스는 제외되지 않는다"),
		RopeColliderGather::IsExcludedOwnerBody(SourceActors, 5, RopeOwner));
	TestFalse(TEXT("음수 인덱스는 제외되지 않는다"),
		RopeColliderGather::IsExcludedOwnerBody(SourceActors, -1, RopeOwner));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
