// Copyright Epic Games, Inc. All Rights Reserved.

#include "Gameplay/RopeRagdollResponseComponent.h"

#include "DynamicRopeLog.h"
// URopeComponent 완전정의 — WrappingRopes(engagement 집합)의 weak 키 타입.
#include "RopeComponent.h"
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
	UWorld* World = GetWorld();
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(World))
	{
		Sim->OnAnyRopeWrapped.Remove(WrappedHandle);
		Sim->OnAnyRopeReleased.Remove(ReleasedHandle);
	}
	if (World)
	{
		World->GetTimerManager().ClearTimer(AutoRagdollTimer);
	}
	WrappingRopes.Reset();

	// 컴포넌트만 떼는 경우(DestroyComponent/UnregisterComponent)의 원복: 랙돌 상태는 이 컴포넌트가 만든
	// 것이고 복구에 필요한 저장값(프로파일/부착/무브먼트 모드)도 이 컴포넌트에만 있다 — 그대로 사라지면
	// 대상은 시뮬 켜진 채 무브먼트가 꺼져 영구히 조작 불능이 된다. 액터/월드가 함께 죽는 경우는 원복해도
	// 의미가 없고 죽어가는 객체를 건드리는 위험만 있으므로 제외한다.
	AActor* Owner = GetOwner();
	const bool bComponentOnlyTeardown = bRagdolled
		&& IsValid(Owner) && !Owner->IsActorBeingDestroyed()
		&& World && !World->bIsTearingDown;
	if (bComponentOnlyTeardown)
	{
		UE_LOG(LogDynamicRope, Log,
			TEXT("[%s] RopeRagdollResponse: 컴포넌트 제거 — 랙돌 상태를 원복하고 나간다."), *GetNameSafe(Owner));
		RecoverFromRagdoll();
	}
	Super::EndPlay(EndPlayReason);
}

void URopeRagdollResponseComponent::HandleAnyRopeWrapped(const FRopeWrappedEventInfo& Info)
{
	const USkeletalMeshComponent* MyMesh = ResolveMesh();
	if (!MyMesh || Info.Mesh.Get() != MyMesh)
	{
		// 다른 메시가 감긴 이벤트 — 무시.
		return;
	}

	// engagement 등록은 랙돌 게이트보다 **먼저** 한다 — 이미 랙돌 중이어도 두 번째 로프를 세어야
	// 하나가 풀렸을 때 조기 복구를 막을 수 있다(종전엔 bRagdolled면 곧장 return이라 아예 기록되지 않았다).
	if (Info.Rope.IsValid())
	{
		WrappingRopes.Add(Info.Rope);
	}

	if (!bRagdollOnWrapped || bRagdolled)
	{
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

void URopeRagdollResponseComponent::HandleAnyRopeReleased(const URopeComponent* Rope,
	const USceneComponent* WrappedMesh, FName Bone, ERopeReleaseReason Reason)
{
	const USkeletalMeshComponent* MyMesh = ResolveMesh();
	if (!MyMesh || WrappedMesh != MyMesh)
	{
		return;
	}

	WrappingRopes.Remove(const_cast<URopeComponent*>(Rope));
	// 감긴 채 파괴된 로프는 release 신호를 못 쏘므로 만료 weak로만 남는다 — 여기서 걷어내지 않으면
	// 집합이 영영 비지 않아 자동 복귀가 죽는다.
	const int32 RemainingRopes = PruneWrappingRopes();

	// 지연 대기 중 풀렸다면(빠른 wrap→release) 예약된 자동 전환을 취소한다. 단 다른 로프가 아직 감고
	// 있으면 그 로프의 예약이므로 유지한다.
	if (RemainingRopes == 0)
	{
		if (UWorld* World = GetWorld())
		{
			World->GetTimerManager().ClearTimer(AutoRagdollTimer);
		}
	}

	// 한 대상을 여러 로프가 감을 수 있다 — 마지막 로프가 풀렸을 때만 일으킨다(하나만 풀렸는데 복구하면
	// 남은 로프에 감긴 채 서 있게 된다).
	if (RemainingRopes > 0)
	{
		UE_LOG(LogDynamicRope, Verbose,
			TEXT("[%s] RopeRagdollResponse: 로프 release(%s) — 아직 %d개 로프가 감고 있어 복귀 보류."),
			*GetNameSafe(GetOwner()), *UEnum::GetValueAsString(Reason), RemainingRopes);
		return;
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

bool URopeRagdollResponseComponent::RecoverFromRagdollIfUnheld()
{
	// 자동 복귀(HandleAnyRopeReleased)와 동일 게이트 — 이 API는 "release 이벤트가 영영 안 오는" 경로의
	// 명시 트리거일 뿐, 복귀 자격 규칙을 넓히지 않는다(수동/치트 랙돌·복귀 옵트아웃은 그대로 존중).
	if (!bRecoverRagdollOnRopeRelease || !bRagdolled || !bRagdollWasAutoTriggered)
	{
		return false;
	}
	// 아직 감고 있는 로프가 있으면 그 마지막 release의 자동 복귀 몫이다(하나만 풀렸는데 일으키기 방지).
	if (PruneWrappingRopes() > 0)
	{
		return false;
	}
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeRagdollResponse: 감은 로프 없음 → 로프 구동 랙돌 명시 복귀(IfUnheld)."),
		*GetNameSafe(GetOwner()));
	RecoverFromRagdoll();
	return true;
}

int32 URopeRagdollResponseComponent::PruneWrappingRopes()
{
	for (auto It = WrappingRopes.CreateIterator(); It; ++It)
	{
		if (!It->IsValid())
		{
			It.RemoveCurrent();
		}
	}
	return WrappingRopes.Num();
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
		EnterPartialRagdoll(PendingWrappedBone, /*bAutoRecoverOnRelease*/ true);
	}
	else
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeRagdollResponse: wrap → 풀 랙돌 자동 전환."), *GetNameSafe(GetOwner()));
		EnterRagdoll(/*bAutoRecoverOnRelease*/ true);
	}
}

void URopeRagdollResponseComponent::EnterRagdoll(bool bAutoRecoverOnRelease)
{
	USkeletalMeshComponent* Mesh = ResolveMesh();
	if (!Mesh || bRagdolled)
	{
		return;
	}

	// 피직스 에셋 검증은 **아무것도 바꾸기 전에** 한다. 바디가 없으면 SetSimulatePhysics(true)가 조용히
	// 무효가 되는데, 그 전에 무브먼트/캡슐 콜리전을 먼저 껐다면 대상은 "랙돌도 아니고 걷지도 못하는"
	// 상태로 bRagdolled=true인 채 고착된다(조작 불능). 부분 랙돌 경로는 원래 이 순서를 지키고 있었다 —
	// 풀 랙돌만 비대칭이었다.
	const UPhysicsAsset* PhysAsset = Mesh->GetPhysicsAsset();
	if (!PhysAsset || PhysAsset->SkeletalBodySetups.Num() == 0)
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] RopeRagdollResponse: 메시에 피직스 바디가 없어(에셋 %s) 풀 랙돌을 건너뛴다."),
			*GetNameSafe(GetOwner()), PhysAsset ? TEXT("바디 0개") : TEXT("없음"));
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
	ApplyRagdollOverlapEvents(Mesh);
	Mesh->SetSimulatePhysics(true);
	ApplyRagdollCCD(Mesh, true);

	bRagdolled = true;
	bPartial = false;
	// 로프 구동 진입(wrap 자동 전환·스네어 강제 랙돌)은 true — release 자동 복귀 게이트 대상.
	// false = 수동/치트 진입(로프가 멋대로 일으키지 않는다).
	bRagdollWasAutoTriggered = bAutoRecoverOnRelease;
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeRagdollResponse: 풀 랙돌 진입."), *GetNameSafe(GetOwner()));
}

void URopeRagdollResponseComponent::EnterPartialRagdoll(FName BoneName, bool bAutoRecoverOnRelease)
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
	ApplyRagdollOverlapEvents(Mesh);
	Mesh->SetAllBodiesBelowSimulatePhysics(BodyBone, true, /*bIncludeSelf*/ true);
	Mesh->SetAllBodiesBelowPhysicsBlendWeight(BodyBone, 1.0f);
	ApplyRagdollCCD(Mesh, true);

	bRagdolled = true;
	bPartial = true;
	bRagdollWasAutoTriggered = bAutoRecoverOnRelease;
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

	// 캡슐 재정렬(풀 랙돌 한정): 시뮬을 끄기 전(= 아직 랙돌 포즈일 때) 앵커 본의 월드 위치를 캡처한다.
	// 아래에서 시뮬을 끄고 메시를 ref 포즈로 리셋하면 이 위치 정보가 사라지므로 여기서 미리 잡아둔다.
	FName RealignAnchor = NAME_None;
	FVector RagdollAnchorWorld = FVector::ZeroVector;
	if (bMoveCapsuleToMeshOnRecover && !bPartial)
	{
		if (!RecoverAnchorBoneName.IsNone() && Mesh->GetBoneIndex(RecoverAnchorBoneName) != INDEX_NONE)
		{
			RealignAnchor = RecoverAnchorBoneName;
			RagdollAnchorWorld = Mesh->GetSocketLocation(RealignAnchor);
		}
		else
		{
			UE_LOG(LogDynamicRope, Warning,
				TEXT("[%s] RopeRagdollResponse: 재정렬 앵커 본 '%s'을(를) 스켈레톤에서 찾지 못해 캡슐 재정렬을 건너뛴다."),
				*GetNameSafe(GetOwner()), *RecoverAnchorBoneName.ToString());
		}
	}

	Mesh->SetAllBodiesSimulatePhysics(false);
	Mesh->SetAllBodiesPhysicsBlendWeight(0.0f);
	Mesh->SetSimulatePhysics(false);
	Mesh->SetCollisionProfileName(SavedCollisionProfile);
	Mesh->SetGenerateOverlapEvents(bSavedMeshOverlapEvents);
	ApplyRagdollCCD(Mesh, false);

	if (!bPartial)
	{
		// 풀 랙돌은 메시가 물리를 따라 캡슐에서 떨어져 나갔으므로 원래 부착 관계로 되돌린다.
		if (USceneComponent* Parent = SavedAttachParent.Get())
		{
			Mesh->AttachToComponent(Parent, FAttachmentTransformRules::KeepRelativeTransform, SavedAttachSocket);
		}
		Mesh->SetRelativeTransform(SavedMeshRelative);

		// 리셋으로 메시가 캡슐 위치의 ref 포즈로 돌아왔다. 캡슐(액터)을 랙돌이 멈춘 곳으로 수평 이동해,
		// 그 되돌아감이 시각적 순간이동이 아니게 만든다. 이동량 = (랙돌 앵커 - 현재 앵커 월드), Z는 0으로
		// 눌러 지면 높이를 유지(캡슐이 pelvis 높이만큼 가라앉는 것 방지 — 지면 스냅은 이어지는 무브먼트가 처리).
		if (!RealignAnchor.IsNone())
		{
			if (AActor* Owner = GetOwner())
			{
				FVector Delta = RagdollAnchorWorld - Mesh->GetSocketLocation(RealignAnchor);
				Delta.Z = 0.0f;
				Owner->AddActorWorldOffset(Delta, /*bSweep*/ false);
			}
		}

		if (ACharacter* Character = Cast<ACharacter>(GetOwner()))
		{
			Character->GetCapsuleComponent()->SetCollisionEnabled(SavedCapsuleCollision);
			// 진입 시 모드로 복귀한다. 단 MOVE_None(이미 무브먼트가 꺼진 상태에서 진입)으로 되돌리면
			// 조작 불능이 그대로 남으므로 그 한 경우만 Walking으로 구제한다.
			const EMovementMode RestoreMode = (SavedMovementMode == MOVE_None) ? MOVE_Walking : SavedMovementMode.GetValue();
			Character->GetCharacterMovement()->SetMovementMode(RestoreMode, SavedCustomMovementMode);
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

void URopeRagdollResponseComponent::ApplyRagdollOverlapEvents(USkeletalMeshComponent* Mesh)
{
	// 랙돌 중 오버랩 이벤트를 낼 수 있는 컴포넌트가 하나도 없는 상태를 막는다: 풀 랙돌은 캡슐을 끄고,
	// ACharacter의 메시는 애초에 GenerateOverlapEvents가 꺼져 있다(엔진 기본). 프로파일 전환은 이
	// 플래그를 건드리지 않으므로 여기서 명시적으로 켠다 — 안 켜면 트리거 볼륨이 랙돌을 못 본다.
	if (bGenerateOverlapEventsWhileRagdolled)
	{
		Mesh->SetGenerateOverlapEvents(true);
	}
}

void URopeRagdollResponseComponent::ApplyRagdollCCD(USkeletalMeshComponent* Mesh, bool bEnable)
{
	// 얇은 바닥(엔진 기본 Plane 등 두께 0 콜리전)은 랙돌 바디가 한 스텝에 면을 넘어가면 접촉이 생성되지
	// 않아 뚫린다 — 랙돌 동안만 전 바디 CCD로 스텝 사이를 스윕한다. 원복은 일괄 false: 피직스 에셋이
	// 저작 시점에 켜둔 바디까지 함께 꺼지는 한계는 감수한다(복귀 후엔 시뮬 off라 실효가 없고, 이
	// 컴포넌트로 재진입하면 다시 켠다).
	if (bUseCCDWhileRagdolled)
	{
		Mesh->SetAllUseCCD(bEnable);
	}
}

void URopeRagdollResponseComponent::SaveRestoreState(USkeletalMeshComponent* Mesh)
{
	SavedCollisionProfile = Mesh->GetCollisionProfileName();
	bSavedMeshOverlapEvents = Mesh->GetGenerateOverlapEvents();
	SavedMeshRelative = Mesh->GetRelativeTransform();
	SavedAttachParent = Mesh->GetAttachParent();
	SavedAttachSocket = Mesh->GetAttachSocketName();

	// 무브먼트 모드도 저장한다 — 복귀 때 되돌리기 위함(비행/수영/커스텀 중 감긴 대상이 걸어 나오면 안 된다).
	SavedMovementMode = MOVE_Walking;
	SavedCustomMovementMode = 0;
	if (const ACharacter* Character = Cast<ACharacter>(GetOwner()))
	{
		if (const UCharacterMovementComponent* Movement = Character->GetCharacterMovement())
		{
			SavedMovementMode = Movement->MovementMode;
			SavedCustomMovementMode = Movement->CustomMovementMode;
		}
	}
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
