// Copyright Epic Games, Inc. All Rights Reserved.

#include "Gameplay/RopeRagdollResponseComponent.h"

#include "DynamicRopeLog.h"
// URopeComponent 완전정의 — WrappingRopes(engagement 집합)의 weak 키 타입.
#include "RopeComponent.h"
#include "Subsystem/RopeSimSubsystem.h"
#include "Camera/CameraActor.h"
#include "CollisionQueryParams.h"
#include "Camera/CameraComponent.h"
#include "Camera/PlayerCameraManager.h"
#include "Components/CapsuleComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/World.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/PlayerController.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "TimerManager.h"
#include "UObject/UObjectIterator.h"

URopeRagdollResponseComponent::URopeRagdollResponseComponent()
{
	// 반응은 중앙 wrap/release 신호 + 타이머로 구동한다. 틱은 랙돌 카메라 추적(풀 랙돌 동안만) 전용 —
	// 평시에는 꺼 둔다. PostPhysics = 랙돌 본 최종 포즈 이후, 카메라 매니저 갱신 이전.
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = false;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;
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
	// 카메라 추적 정리 — 컴포넌트만 떼는 경우의 원복(RecoverFromRagdoll)보다 먼저 불려도, 나중에
	// 중복으로 불려도 무해(내부 가드). 월드 소멸 중엔 블렌드 없이 정리만 된다.
	EndRagdollCameraFollow();

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
	BeginRagdollCameraFollow();
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

		// 리셋으로 메시가 캡슐 위치의 ref 포즈로 돌아왔다. 캡슐(액터)을 랙돌이 멈춘 곳으로 이동해 그
		// 되돌아감이 시각적 순간이동이 아니게 만든다. 이동량 = (랙돌 앵커 - 현재 앵커 월드). 높이는 캐릭터
		// + RecoverGroundSearchDistance > 0일 때만 반영한다(수직 수송 — 헬기 캐리 — 후 옛 높이로 되돌아가는
		// 것 방지): 앵커 아래로 바닥을 트레이스해 캡슐 바닥을 그 위에 세우고, 못 찾으면(공중 하차) 앵커
		// 높이에서 낙하로 잇는다. 그 외(비캐릭터/탐색 0)는 종전대로 Z를 눌러 지면 높이 유지.
		bool bRecoveredAirborne = false;
		if (!RealignAnchor.IsNone())
		{
			if (AActor* Owner = GetOwner())
			{
				FVector Delta = RagdollAnchorWorld - Mesh->GetSocketLocation(RealignAnchor);
				const ACharacter* OwnerCharacter = Cast<ACharacter>(Owner);
				const UCapsuleComponent* Capsule =
					OwnerCharacter ? OwnerCharacter->GetCapsuleComponent() : nullptr;
				if (Capsule && RecoverGroundSearchDistance > 0.0f && GetWorld())
				{
					// 캡슐 콜리전은 아직 꺼져 있고(진입 시 NoCollision) 랙돌 메시는 남아 있으므로 자기
					// 액터를 명시 제외한다. 채널은 캡슐이 걷는 것과 같은 Pawn 기준.
					FCollisionQueryParams Params(SCENE_QUERY_STAT(RopeRagdollRecoverGround),
						/*bTraceComplex*/ false, Owner);
					const float HalfHeight = Capsule->GetScaledCapsuleHalfHeight();
					const FVector TraceStart = RagdollAnchorWorld + FVector(0.0f, 0.0f, HalfHeight);
					const FVector TraceEnd =
						RagdollAnchorWorld - FVector(0.0f, 0.0f, RecoverGroundSearchDistance);
					FHitResult Hit;
					float TargetCenterZ;
					if (GetWorld()->LineTraceSingleByChannel(Hit, TraceStart, TraceEnd, ECC_Pawn, Params))
					{
						TargetCenterZ = static_cast<float>(Hit.ImpactPoint.Z) + HalfHeight;
					}
					else
					{
						// 탐색 거리 안에 바닥 없음 = 공중 하차 — 앵커 높이에서 낙하로 복귀.
						TargetCenterZ = static_cast<float>(RagdollAnchorWorld.Z);
						bRecoveredAirborne = true;
					}
					Delta.Z = TargetCenterZ - static_cast<float>(Capsule->GetComponentLocation().Z);
				}
				else
				{
					Delta.Z = 0.0f;
				}
				Owner->AddActorWorldOffset(Delta, /*bSweep*/ false);
			}
		}

		if (ACharacter* Character = Cast<ACharacter>(GetOwner()))
		{
			Character->GetCapsuleComponent()->SetCollisionEnabled(SavedCapsuleCollision);
			// 진입 시 모드로 복귀한다. 단 MOVE_None(이미 무브먼트가 꺼진 상태에서 진입)으로 되돌리면
			// 조작 불능이 그대로 남으므로 그 한 경우만 Walking으로 구제하고, 공중 하차(위 트레이스 실패)는
			// 접지 모드 대신 낙하로 잇는다(Walking 복귀는 바닥 탐색 실패 시 첫 틱에 어차피 낙하 전환되지만,
			// 명시해 한 프레임 바닥 스냅 시도를 없앤다). 비행/수영 저장 모드는 그대로 존중.
			EMovementMode RestoreMode = (SavedMovementMode == MOVE_None) ? MOVE_Walking : SavedMovementMode.GetValue();
			if (bRecoveredAirborne && (RestoreMode == MOVE_Walking || RestoreMode == MOVE_NavWalking))
			{
				RestoreMode = MOVE_Falling;
			}
			Character->GetCharacterMovement()->SetMovementMode(RestoreMode, SavedCustomMovementMode);
		}
	}

	// 캡슐 재정렬/무브먼트 복구가 끝난 뒤에 카메라를 폰으로 블렌드 백한다 — 목적지(폰 카메라)가
	// 최종 위치에 있어야 블렌드가 헛돌지 않는다.
	EndRagdollCameraFollow();

	bRagdolled = false;
	bPartial = false;
	bRagdollWasAutoTriggered = false;
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeRagdollResponse: 랙돌 복귀."), *GetNameSafe(GetOwner()));
}

void URopeRagdollResponseComponent::BeginRagdollCameraFollow()
{
	// 플레이어가 보고 있는 폰의 풀 랙돌만 대상. 실패 조건은 전부 조용한 no-op — 카메라는 편의 기능이고
	// 랙돌 본체(물리 전환)의 성패와 무관해야 한다.
	if (!bViewTargetFollowRagdoll || FollowCamera.IsValid())
	{
		return;
	}
	APawn* Pawn = Cast<APawn>(GetOwner());
	APlayerController* PC = Pawn ? Cast<APlayerController>(Pawn->GetController()) : nullptr;
	USkeletalMeshComponent* Mesh = ResolveMesh();
	UWorld* World = GetWorld();
	if (!PC || !PC->PlayerCameraManager || !Mesh || !World)
	{
		return;
	}
	// 이미 다른 뷰타깃(시네마틱 등)을 보고 있으면 빼앗지 않는다.
	if (PC->GetViewTarget() != Pawn)
	{
		return;
	}

	// 추적 기준점: 캡슐 재정렬과 같은 앵커 본(스켈레톤에 없으면 메시 원점).
	const bool bHasAnchorBone =
		!RecoverAnchorBoneName.IsNone() && Mesh->GetBoneIndex(RecoverAnchorBoneName) != INDEX_NONE;
	const FVector AnchorPos =
		bHasAnchorBone ? Mesh->GetSocketLocation(RecoverAnchorBoneName) : Mesh->GetComponentLocation();

	// 현재 POV 그 자리에 스폰 + 무블렌드 전환 = 화면상 이음새 없음.
	const FVector CamLoc = PC->PlayerCameraManager->GetCameraLocation();
	const FRotator CamRot = PC->PlayerCameraManager->GetCameraRotation();
	FActorSpawnParameters SpawnParams;
	SpawnParams.Owner = GetOwner();
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	ACameraActor* Cam = World->SpawnActor<ACameraActor>(CamLoc, CamRot, SpawnParams);
	if (!Cam)
	{
		return;
	}
	if (UCameraComponent* CamComp = Cam->GetCameraComponent())
	{
		CamComp->SetFieldOfView(PC->PlayerCameraManager->GetFOVAngle());
		CamComp->bConstrainAspectRatio = false;
	}
	PC->SetViewTargetWithBlend(Cam, 0.0f);

	FollowController = PC;
	FollowCamera = Cam;
	FollowCameraOffset = CamLoc - AnchorPos;
	// 본 최종 포즈(물리 블렌드) 이후에 읽도록 메시를 선행조건으로 — 한 프레임 지연 흔들림 방지.
	AddTickPrerequisiteComponent(Mesh);
	SetComponentTickEnabled(true);
}

void URopeRagdollResponseComponent::EndRagdollCameraFollow()
{
	SetComponentTickEnabled(false);
	APlayerController* PC = FollowController.Get();
	ACameraActor* Cam = FollowCamera.Get();
	FollowController.Reset();
	FollowCamera.Reset();
	if (!Cam)
	{
		return;
	}

	AActor* Owner = GetOwner();
	UWorld* World = GetWorld();
	const bool bWorldAlive = World && !World->bIsTearingDown;
	// 우리가 아직 뷰타깃일 때만 폰으로 블렌드 백(그 사이 시네마틱 등이 가져갔으면 존중). 소유자가
	// 죽는 중이면 목적지가 없다 — 카메라를 그대로 두고 수명만 걸어 카메라 매니저 폴백에 맡긴다.
	if (PC && bWorldAlive && PC->GetViewTarget() == Cam
		&& IsValid(Owner) && !Owner->IsActorBeingDestroyed())
	{
		PC->SetViewTargetWithBlend(Owner, FMath::Max(RecoverCameraBlendTime, 0.0f),
			VTBlend_Cubic);
	}
	// 블렌드가 끝날 때까지 원본 뷰타깃이 살아 있어야 한다 — 즉시 파괴 대신 수명으로 정리.
	if (bWorldAlive)
	{
		Cam->SetLifeSpan(FMath::Max(RecoverCameraBlendTime, 0.0f) + 0.5f);
	}
	else
	{
		Cam->Destroy();
	}
}

void URopeRagdollResponseComponent::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	ACameraActor* Cam = FollowCamera.Get();
	USkeletalMeshComponent* Mesh = ResolveMesh();
	APlayerController* PC = FollowController.Get();
	// 추적 전제가 무너지면(복귀 외 경로: 메시 소실/컨트롤러 소멸/뷰타깃 피탈) 정리하고 끝낸다.
	if (!Cam || !Mesh || !bRagdolled || bPartial || !PC || PC->GetViewTarget() != Cam)
	{
		EndRagdollCameraFollow();
		return;
	}

	const bool bHasAnchorBone =
		!RecoverAnchorBoneName.IsNone() && Mesh->GetBoneIndex(RecoverAnchorBoneName) != INDEX_NONE;
	const FVector AnchorPos =
		bHasAnchorBone ? Mesh->GetSocketLocation(RecoverAnchorBoneName) : Mesh->GetComponentLocation();
	const FVector Desired = AnchorPos + FollowCameraOffset;
	// 위치는 일방향 러그 추적(카메라→본 피드백 없음 = 폭주 불가), 시선은 항상 랙돌을 향한다.
	const FVector NewLoc = FollowCameraLagSpeed > 0.0f
		? FMath::VInterpTo(Cam->GetActorLocation(), Desired, DeltaTime, FollowCameraLagSpeed)
		: Desired;
	Cam->SetActorLocation(NewLoc);
	const FVector ToAnchor = AnchorPos - NewLoc;
	if (!ToAnchor.IsNearlyZero())
	{
		Cam->SetActorRotation(ToAnchor.Rotation());
	}
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
