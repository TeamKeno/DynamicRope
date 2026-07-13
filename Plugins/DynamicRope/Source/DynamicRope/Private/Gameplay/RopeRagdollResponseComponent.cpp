// Copyright Epic Games, Inc. All Rights Reserved.

#include "Gameplay/RopeRagdollResponseComponent.h"

#include "DynamicRopeLog.h"
#include "Subsystem/RopeSimSubsystem.h"
#include "Components/CapsuleComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/World.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "TimerManager.h"
#include "UObject/UObjectIterator.h"

URopeRagdollResponseComponent::URopeRagdollResponseComponent()
{
	// 반응은 전적으로 중앙 wrap/release 신호 + 타이머로 구동한다 — 틱 불필요.
	PrimaryComponentTick.bCanEverTick = false;
}

void URopeRagdollResponseComponent::BeginPlay()
{
	Super::BeginPlay();

	if (!ResolveMesh())
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] RopeRagdollResponse: 스켈레탈 메시를 찾지 못했다 — 랙돌 반응 비활성."),
			*GetNameSafe(GetOwner()));
	}

	// 월드 어느 로프든 wrap/release되면 알림을 받는다(자기를 감을 로프를 미리 몰라도 반응 가능).
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		WrappedHandle = Sim->OnAnyRopeWrapped.AddUObject(this, &URopeRagdollResponseComponent::HandleAnyRopeWrapped);
		ReleasedHandle = Sim->OnAnyRopeReleased.AddUObject(this, &URopeRagdollResponseComponent::HandleAnyRopeReleased);
	}
}

void URopeRagdollResponseComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->OnAnyRopeWrapped.Remove(WrappedHandle);
		Sim->OnAnyRopeReleased.Remove(ReleasedHandle);
	}
	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().ClearTimer(AutoRagdollTimer);
	}
	Super::EndPlay(EndPlayReason);
}

void URopeRagdollResponseComponent::HandleAnyRopeWrapped(const FRopeWrappedEventInfo& Info)
{
	if (!bRagdollOnWrapped || bRagdolled)
	{
		return;
	}
	const USkeletalMeshComponent* MyMesh = ResolveMesh();
	if (!MyMesh || Info.Mesh.Get() != MyMesh)
	{
		// 다른 메시가 감긴 이벤트 — 무시.
		return;
	}

	PendingWrappedBone = Info.Bone;
	if (RagdollOnWrappedDelay <= 0.0f)
	{
		FireAutoRagdoll();
	}
	else if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().SetTimer(AutoRagdollTimer, this,
			&URopeRagdollResponseComponent::FireAutoRagdoll, RagdollOnWrappedDelay, /*bLoop*/ false);
	}
}

void URopeRagdollResponseComponent::HandleAnyRopeReleased(const USceneComponent* WrappedMesh, FName Bone, ERopeReleaseReason Reason)
{
	const USkeletalMeshComponent* MyMesh = ResolveMesh();
	if (!MyMesh || WrappedMesh != MyMesh)
	{
		return;
	}

	// 지연 대기 중 풀렸다면(빠른 wrap→release) 예약된 자동 전환을 취소한다.
	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().ClearTimer(AutoRagdollTimer);
	}

	// 자동 전환된 랙돌만 자동 복귀(수동/치트 진입은 유지). RecoverFromRagdoll이 메시 유효성/랙돌 여부를
	// 다시 가드하므로 대상 소실(Broken) 등으로 메시가 없으면 조용히 no-op.
	if (bRecoverRagdollOnRopeRelease && bRagdolled && bRagdollWasAutoTriggered)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeRagdollResponse: 로프 release(%s) → 자동 랙돌 복귀."),
			*GetNameSafe(GetOwner()), *UEnum::GetValueAsString(Reason));
		RecoverFromRagdoll();
	}
}

void URopeRagdollResponseComponent::FireAutoRagdoll()
{
	if (bRagdolled)
	{
		return;
	}

	if (bOnlyBelowWrappedBone && !PendingWrappedBone.IsNone())
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeRagdollResponse: wrap → 감긴 본(%s) 이하 부분 랙돌 자동 전환."),
			*GetNameSafe(GetOwner()), *PendingWrappedBone.ToString());
		EnterPartialRagdoll(PendingWrappedBone);
	}
	else
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeRagdollResponse: wrap → 풀 랙돌 자동 전환."), *GetNameSafe(GetOwner()));
		EnterRagdoll();
	}

	// Enter*가 성공해 랙돌에 들어갔으면 이 전환은 "자동"으로 표시한다(Enter*는 수동 기준으로 false를
	// 세팅하므로 여기서 덮어써야 한다 — 자동 복귀 게이트가 이 플래그를 본다).
	if (bRagdolled)
	{
		bRagdollWasAutoTriggered = true;
	}
}

void URopeRagdollResponseComponent::EnterRagdoll()
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
	// 기본은 수동 진입 — 자동 경로(FireAutoRagdoll)가 성공 후 true로 덮는다.
	bRagdollWasAutoTriggered = false;
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeRagdollResponse: 풀 랙돌 진입."), *GetNameSafe(GetOwner()));
}

void URopeRagdollResponseComponent::EnterPartialRagdoll(FName BoneName)
{
	USkeletalMeshComponent* Mesh = ResolveMesh();
	if (!Mesh || bRagdolled)
	{
		return;
	}
	if (BoneName == NAME_None || Mesh->GetBoneIndex(BoneName) == INDEX_NONE)
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] RopeRagdollResponse: 본 '%s'을(를) 찾지 못해 부분 랙돌을 건너뛴다."),
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
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] RopeRagdollResponse: 피직스 에셋이 없어 부분 랙돌을 건너뛴다."),
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
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] RopeRagdollResponse: 본 '%s' 및 부모 체인에 피직스 바디가 없어 부분 랙돌을 건너뛴다."),
			*GetNameSafe(GetOwner()), *BoneName.ToString());
		return;
	}
	if (BodyBone != BoneName)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeRagdollResponse: 본 '%s'에 바디가 없어 '%s'(으)로 승격."),
			*GetNameSafe(GetOwner()), *BoneName.ToString(), *BodyBone.ToString());
	}

	SaveRestoreState(Mesh);

	// 캡슐/무브먼트는 유지 — 시뮬 안 하는 나머지 본은 애니메이션을 계속 탄다.
	Mesh->SetCollisionProfileName(RagdollCollisionProfileName);
	Mesh->SetAllBodiesBelowSimulatePhysics(BodyBone, true, /*bIncludeSelf*/ true);
	Mesh->SetAllBodiesBelowPhysicsBlendWeight(BodyBone, 1.0f);

	bRagdolled = true;
	bPartial = true;
	bRagdollWasAutoTriggered = false;
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeRagdollResponse: 부분 랙돌 진입(본 %s 이하)."),
		*GetNameSafe(GetOwner()), *BodyBone.ToString());
}

void URopeRagdollResponseComponent::RecoverFromRagdoll()
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
	bRagdollWasAutoTriggered = false;
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeRagdollResponse: 랙돌 복귀."), *GetNameSafe(GetOwner()));
}

USkeletalMeshComponent* URopeRagdollResponseComponent::ResolveMesh() const
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

void URopeRagdollResponseComponent::SaveRestoreState(USkeletalMeshComponent* Mesh)
{
	SavedCollisionProfile = Mesh->GetCollisionProfileName();
	SavedMeshRelative = Mesh->GetRelativeTransform();
	SavedAttachParent = Mesh->GetAttachParent();
	SavedAttachSocket = Mesh->GetAttachSocketName();
}

#if !UE_BUILD_SHIPPING
//======================================================================================
// 개발 확인용 콘솔 명령 — 이 컴포넌트가 붙은 월드 내 모든 액터에 일괄 적용.
//======================================================================================

namespace RopeRagdollConsole
{
	static void ForEach(UWorld* World, TFunctionRef<void(URopeRagdollResponseComponent&)> Fn)
	{
		int32 Count = 0;
		for (TObjectIterator<URopeRagdollResponseComponent> It; It; ++It)
		{
			URopeRagdollResponseComponent* Comp = *It;
			if (IsValid(Comp) && Comp->GetWorld() == World && IsValid(Comp->GetOwner()))
			{
				Fn(*Comp);
				++Count;
			}
		}
		if (Count == 0)
		{
			UE_LOG(LogDynamicRope, Warning,
				TEXT("RopeRagdollResponseComponent가 붙은 액터가 없다 — 대상 캐릭터 BP(또는 레벨 인스턴스)에 컴포넌트를 추가할 것."));
		}
	}

	static FAutoConsoleCommandWithWorldAndArgs GRagdollCmd(
		TEXT("Rope.Ragdoll"),
		TEXT("랙돌 반응 토글. 인자 없음: 풀 랙돌 토글(랙돌 중이면 복귀). 본 이름 인자: 그 본 이하 부분 랙돌."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
		{
			ForEach(World, [&Args](URopeRagdollResponseComponent& Comp)
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
		TEXT("Rope.Ragdoll.Recover"),
		TEXT("랙돌 반응: 애니메이션 복귀(풀 랙돌은 포즈 팝 — wrap 중이면 로프 무속도 추종 확인)."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
		{
			ForEach(World, [](URopeRagdollResponseComponent& Comp)
			{
				Comp.RecoverFromRagdoll();
			});
		}));

	static FAutoConsoleCommandWithWorldAndArgs GDestroyCmd(
		TEXT("Rope.Ragdoll.Destroy"),
		TEXT("랙돌 반응: 대상 액터 파괴(감긴 중 파괴 → 로프 weak mesh 경로 release 확인)."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
		{
			ForEach(World, [](URopeRagdollResponseComponent& Comp)
			{
				if (AActor* Owner = Comp.GetOwner())
				{
					UE_LOG(LogDynamicRope, Log, TEXT("[%s] 파괴(대상 소실 release 테스트)."), *GetNameSafe(Owner));
					Owner->Destroy();
				}
			});
		}));
}
#endif // !UE_BUILD_SHIPPING
