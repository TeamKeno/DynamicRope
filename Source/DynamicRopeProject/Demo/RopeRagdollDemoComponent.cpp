// Fill out your copyright notice in the Description page of Project Settings.

#include "RopeRagdollDemoComponent.h"

#include "RopeComponent.h"
#include "Components/CapsuleComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "HAL/IConsoleManager.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "UObject/UObjectIterator.h"

DEFINE_LOG_CATEGORY_STATIC(LogRopeRagdollDemo, Log, All);

URopeRagdollDemoComponent::URopeRagdollDemoComponent()
{
	// 자동 전환 감시(bRagdollOnWrapped)용. 꺼져 있으면 틱 초입에서 바로 나간다(데모라 비용 무시).
	PrimaryComponentTick.bCanEverTick = true;
}

void URopeRagdollDemoComponent::BeginPlay()
{
	Super::BeginPlay();

	if (!ResolveMesh())
	{
		UE_LOG(LogRopeRagdollDemo, Warning, TEXT("[%s] 스켈레탈 메시를 찾지 못했다 — 랙돌 데모 비활성."),
			*GetNameSafe(GetOwner()));
	}
}

void URopeRagdollDemoComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bRagdollOnWrapped)
	{
		return;
	}

	FName WrappedBone = NAME_None;
	URopeComponent* Rope = FindRopeWrappingUs(WrappedBone);
	if (!Rope)
	{
		// 감김이 풀리면 다음 wrap에서 다시 발화할 수 있게 리셋.
		PendingAutoRagdollTime = -1.0f;
		bAutoFiredThisWrap = false;
		return;
	}

	if (bRagdolled || bAutoFiredThisWrap)
	{
		return;
	}

	PendingAutoRagdollTime = (PendingAutoRagdollTime < 0.0f) ? DeltaTime : PendingAutoRagdollTime + DeltaTime;
	if (PendingAutoRagdollTime < RagdollOnWrappedDelay)
	{
		return;
	}

	bAutoFiredThisWrap = true;
	PendingAutoRagdollTime = -1.0f;

	if (bOnlyBelowWrappedBone && WrappedBone != NAME_None)
	{
		UE_LOG(LogRopeRagdollDemo, Log, TEXT("[%s] wrap 감지 → 감긴 본(%s) 이하 부분 랙돌 자동 전환."),
			*GetNameSafe(GetOwner()), *WrappedBone.ToString());
		EnterPartialRagdoll(WrappedBone);
	}
	else
	{
		UE_LOG(LogRopeRagdollDemo, Log, TEXT("[%s] wrap 감지 → 풀 랙돌 자동 전환."), *GetNameSafe(GetOwner()));
		EnterRagdoll();
	}
}

void URopeRagdollDemoComponent::EnterRagdoll()
{
	USkeletalMeshComponent* Mesh = ResolveMesh();
	if (!Mesh || bRagdolled)
	{
		return;
	}

	SaveRestoreState(Mesh);

	if (ACharacter* Character = Cast<ACharacter>(GetOwner()))
	{
		Character->GetCharacterMovement()->DisableMovement();
		SavedCapsuleCollision = Character->GetCapsuleComponent()->GetCollisionEnabled();
		Character->GetCapsuleComponent()->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	}

	Mesh->SetCollisionProfileName(RagdollCollisionProfileName);
	Mesh->SetSimulatePhysics(true);

	bRagdolled = true;
	bPartial = false;
	UE_LOG(LogRopeRagdollDemo, Log, TEXT("[%s] 풀 랙돌 진입."), *GetNameSafe(GetOwner()));
}

void URopeRagdollDemoComponent::EnterPartialRagdoll(FName BoneName)
{
	USkeletalMeshComponent* Mesh = ResolveMesh();
	if (!Mesh || bRagdolled)
	{
		return;
	}
	if (BoneName == NAME_None || Mesh->GetBoneIndex(BoneName) == INDEX_NONE)
	{
		UE_LOG(LogRopeRagdollDemo, Warning, TEXT("[%s] 본 '%s'을(를) 찾지 못해 부분 랙돌을 건너뛴다."),
			*GetNameSafe(GetOwner()), *BoneName.ToString());
		return;
	}

	// 스켈레톤에는 있어도 피직스 에셋에 바디가 없는 본(트위스트/IK 등)이면 SetAllBodiesBelow*가
	// 잡을 바디가 없어 조용히 무시된다(감긴 본을 그대로 넘기는 bOnlyBelowWrappedBone 자동 경로에서
	// 특히 잘 걸린다). 부모 체인을 올라가 바디가 있는 본으로 승격한다 — 플러그인 Pull 쪽
	// FindNearestSimulatingBone과 같은 원리(여기는 "시뮬 중"이 아니라 "바디 존재"가 기준).
	UPhysicsAsset* PhysAsset = Mesh->GetPhysicsAsset();
	if (!PhysAsset)
	{
		UE_LOG(LogRopeRagdollDemo, Warning, TEXT("[%s] 피직스 에셋이 없어 부분 랙돌을 건너뛴다."),
			*GetNameSafe(GetOwner()));
		return;
	}
	FName BodyBone = BoneName;
	while (!BodyBone.IsNone() && PhysAsset->FindBodyIndex(BodyBone) == INDEX_NONE)
	{
		BodyBone = Mesh->GetParentBone(BodyBone);
	}
	if (BodyBone.IsNone())
	{
		UE_LOG(LogRopeRagdollDemo, Warning,
			TEXT("[%s] 본 '%s' 및 부모 체인에 피직스 바디가 없어 부분 랙돌을 건너뛴다."),
			*GetNameSafe(GetOwner()), *BoneName.ToString());
		return;
	}
	if (BodyBone != BoneName)
	{
		UE_LOG(LogRopeRagdollDemo, Log, TEXT("[%s] 본 '%s'에 바디가 없어 '%s'(으)로 승격."),
			*GetNameSafe(GetOwner()), *BoneName.ToString(), *BodyBone.ToString());
	}

	SaveRestoreState(Mesh);

	// 캡슐/무브먼트는 유지 — 시뮬 안 하는 나머지 본은 애니메이션을 계속 탄다.
	Mesh->SetCollisionProfileName(RagdollCollisionProfileName);
	Mesh->SetAllBodiesBelowSimulatePhysics(BodyBone, true, /*bIncludeSelf*/ true);
	Mesh->SetAllBodiesBelowPhysicsBlendWeight(BodyBone, 1.0f);

	bRagdolled = true;
	bPartial = true;
	UE_LOG(LogRopeRagdollDemo, Log, TEXT("[%s] 부분 랙돌 진입(본 %s 이하)."),
		*GetNameSafe(GetOwner()), *BodyBone.ToString());
}

void URopeRagdollDemoComponent::RecoverFromRagdoll()
{
	USkeletalMeshComponent* Mesh = ResolveMesh();
	if (!Mesh || !bRagdolled)
	{
		return;
	}

	Mesh->SetAllBodiesSimulatePhysics(false);
	Mesh->SetAllBodiesPhysicsBlendWeight(0.0f);
	Mesh->SetSimulatePhysics(false);
	Mesh->SetCollisionProfileName(SavedCollisionProfile);

	if (!bPartial)
	{
		// 풀 랙돌은 메시가 물리를 따라 캡슐에서 떨어져 나갔으므로 원래 부착 관계로 되돌린다.
		// 캡슐 위치는 그대로 두므로 큰 포즈 팝이 생긴다 — wrap 중이면 로프 무속도 추종 관찰 재료(의도).
		if (USceneComponent* Parent = SavedAttachParent.Get())
		{
			Mesh->AttachToComponent(Parent, FAttachmentTransformRules::KeepRelativeTransform, SavedAttachSocket);
		}
		Mesh->SetRelativeTransform(SavedMeshRelative);

		if (ACharacter* Character = Cast<ACharacter>(GetOwner()))
		{
			Character->GetCapsuleComponent()->SetCollisionEnabled(SavedCapsuleCollision);
			Character->GetCharacterMovement()->SetMovementMode(MOVE_Walking);
		}
	}

	bRagdolled = false;
	bPartial = false;
	UE_LOG(LogRopeRagdollDemo, Log, TEXT("[%s] 랙돌 복귀."), *GetNameSafe(GetOwner()));
}

USkeletalMeshComponent* URopeRagdollDemoComponent::ResolveMesh() const
{
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return nullptr;
	}
	if (const ACharacter* Character = Cast<ACharacter>(Owner))
	{
		return Character->GetMesh();
	}
	return Owner->FindComponentByClass<USkeletalMeshComponent>();
}

void URopeRagdollDemoComponent::SaveRestoreState(USkeletalMeshComponent* Mesh)
{
	SavedCollisionProfile = Mesh->GetCollisionProfileName();
	SavedMeshRelative = Mesh->GetRelativeTransform();
	SavedAttachParent = Mesh->GetAttachParent();
	SavedAttachSocket = Mesh->GetAttachSocketName();
}

URopeComponent* URopeRagdollDemoComponent::FindRopeWrappingUs(FName& OutWrappedBone) const
{
	OutWrappedBone = NAME_None;
	const USkeletalMeshComponent* MyMesh = ResolveMesh();
	if (!MyMesh)
	{
		return nullptr;
	}

	// 데모 규모(월드당 로프 몇 개)라 전수 순회로 충분하다.
	for (TObjectIterator<URopeComponent> It; It; ++It)
	{
		URopeComponent* Rope = *It;
		if (!IsValid(Rope) || Rope->GetWorld() != GetWorld() || !Rope->IsRegistered())
		{
			continue;
		}
		if (Rope->GetPhase() == ERopePhase::Wrapped && Rope->GetWrappedMesh() == MyMesh)
		{
			OutWrappedBone = Rope->GetWrappedBoneName();
			return Rope;
		}
	}
	return nullptr;
}

//======================================================================================
// PIE 콘솔 명령 — 이 컴포넌트가 붙은 월드 내 모든 액터에 일괄 적용.
//======================================================================================

namespace RopeRagdollDemoCommands
{
	static void ForEach(UWorld* World, TFunctionRef<void(URopeRagdollDemoComponent&)> Fn)
	{
		int32 Count = 0;
		for (TObjectIterator<URopeRagdollDemoComponent> It; It; ++It)
		{
			URopeRagdollDemoComponent* Comp = *It;
			if (IsValid(Comp) && Comp->GetWorld() == World && IsValid(Comp->GetOwner()))
			{
				Fn(*Comp);
				++Count;
			}
		}
		if (Count == 0)
		{
			UE_LOG(LogRopeRagdollDemo, Warning,
				TEXT("RopeRagdollDemoComponent가 붙은 액터가 없다 — 대상 캐릭터 BP(또는 레벨 인스턴스)에 컴포넌트를 추가할 것."));
		}
	}

	static FAutoConsoleCommandWithWorldAndArgs GRagdollCmd(
		TEXT("Rope.Demo.Ragdoll"),
		TEXT("랙돌 데모 토글. 인자 없음: 풀 랙돌 토글(랙돌 중이면 복귀). 본 이름 인자: 그 본 이하 부분 랙돌."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
		{
			ForEach(World, [&Args](URopeRagdollDemoComponent& Comp)
			{
				if (Comp.IsRagdolled())
				{
					Comp.RecoverFromRagdoll();
				}
				else if (Args.Num() > 0)
				{
					Comp.EnterPartialRagdoll(FName(*Args[0]));
				}
				else
				{
					Comp.EnterRagdoll();
				}
			});
		}));

	static FAutoConsoleCommandWithWorldAndArgs GRecoverCmd(
		TEXT("Rope.Demo.RagdollRecover"),
		TEXT("랙돌 데모: 애니메이션 복귀(풀 랙돌은 포즈 팝 — wrap 중이면 로프 무속도 추종 확인)."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
		{
			ForEach(World, [](URopeRagdollDemoComponent& Comp)
			{
				Comp.RecoverFromRagdoll();
			});
		}));

	static FAutoConsoleCommandWithWorldAndArgs GDestroyCmd(
		TEXT("Rope.Demo.RagdollDestroy"),
		TEXT("랙돌 데모: 대상 액터 파괴(감긴 중 파괴 → 로프 weak mesh 경로 release 확인)."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
		{
			ForEach(World, [](URopeRagdollDemoComponent& Comp)
			{
				if (AActor* Owner = Comp.GetOwner())
				{
					UE_LOG(LogRopeRagdollDemo, Log, TEXT("[%s] 파괴(대상 소실 release 테스트)."), *GetNameSafe(Owner));
					Owner->Destroy();
				}
			});
		}));
}
