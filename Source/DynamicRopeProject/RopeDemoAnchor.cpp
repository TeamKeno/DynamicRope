// Fill out your copyright notice in the Description page of Project Settings.

#include "RopeDemoAnchor.h"
#include "RopeTetherTarget.h"
#include "RopeComponent.h"
#include "Components/InputComponent.h"
#include "GameFramework/PlayerController.h"
#include "Engine/World.h"

ARopeDemoAnchor::ARopeDemoAnchor()
{
	PrimaryActorTick.bCanEverTick = false;

	Rope = CreateDefaultSubobject<URopeComponent>(TEXT("Rope"));
	RootComponent = Rope;
	Rope->bDrawDebugCenterline = true; // 데모: 센터라인/바운드를 화면에 표시.
}

void ARopeDemoAnchor::BeginPlay()
{
	Super::BeginPlay();

	if (Target)
	{
		// cross-actor 배선: 다른 액터(타깃)의 provider를 로프 충돌 소스로 등록.
		Rope->ColliderSourceActors.AddUnique(Target);
		// 폴백 메시 명시(잡힌 본의 메시는 contact로 자동 스레딩되지만, 폴백 대비).
		Rope->WrapTargetMesh = Target->GetMesh();
	}

	// wrap/release 이벤트 → 타깃 행동 제한 토글.
	Rope->OnRopeWrapped.AddDynamic(this, &ARopeDemoAnchor::HandleWrapped);
	Rope->OnRopeReleased.AddDynamic(this, &ARopeDemoAnchor::HandleReleased);

	// 데모 입력: 첫 번째 플레이어 컨트롤러가 이 액터로 키 입력을 보내게 한다.
	if (APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr)
	{
		EnableInput(PC);
		if (InputComponent)
		{
			InputComponent->BindKey(EKeys::T, IE_Pressed, this, &ARopeDemoAnchor::DoThrow);
			InputComponent->BindKey(EKeys::F, IE_Pressed, this, &ARopeDemoAnchor::DoForceWrap);
			InputComponent->BindKey(EKeys::R, IE_Pressed, this, &ARopeDemoAnchor::DoRelease);
		}
	}
}

void ARopeDemoAnchor::DoThrow()
{
	const FVector Dir = Target
		? (Target->GetActorLocation() - GetActorLocation()).GetSafeNormal()
		: GetActorForwardVector();
	Rope->Throw(Dir);
}

void ARopeDemoAnchor::DoForceWrap()
{
	Rope->DebugForceWrap();
}

void ARopeDemoAnchor::DoRelease()
{
	Rope->ReleaseWrap();
}

void ARopeDemoAnchor::HandleWrapped(FName Bone)
{
	if (Target)
	{
		Target->SetTethered(true); // 감김 성립 → 행동 제한 발동.
	}
}

void ARopeDemoAnchor::HandleReleased(FName Bone, ERopeReleaseReason Reason)
{
	if (Target)
	{
		Target->SetTethered(false); // 풀림 → 제한 해제, 배회 재개.
	}
}
