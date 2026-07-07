// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeController.h"
#include "Collision/RopeStaticBodyProvider.h"

ARopeController::ARopeController()
{
	// 매니저 액터 — 틱 없음(프로바이더는 서브시스템이 프레임당 1회 pull), 리플리케이트 없음
	// (클라/서버가 각자 스폰 — 서버는 CPU 폴백 sim에서도 정적 충돌이 필요하므로 양쪽 스폰이 옳다).
	PrimaryActorTick.bCanEverTick = false;
	SetReplicates(false);

	// 정적 바디 프로바이더는 비-scene UActorComponent라 RootComponent가 될 수 없다 — 서브오브젝트로만 붙인다.
	// 컴포넌트가 BeginPlay에서 URopeSimSubsystem에 자동 등록된다(ProvidesWorldStaticColliders=true).
	StaticBodyProvider = CreateDefaultSubobject<URopeStaticBodyProvider>(TEXT("StaticBodyProvider"));
}
