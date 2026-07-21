// Copyright Epic Games, Inc. All Rights Reserved.

#include "Gameplay/RopeWielderComponent.h"
#include "RopeComponent.h"
#include "Materials/MaterialInstanceDynamic.h"
// ResolveBindingWorld — 조준 HUD 샘플의 본 위치(링 중심) 해석.
#include "Core/RopeWrapTarget.h"
#include "DynamicRopeLog.h"
#include "Render/RopePreviewComponent.h"
// 조준 HUD 위젯(생성은 프로젝트 세팅의 클래스, 수명은 이 컴포넌트가 관리).
#include "Settings/DynamicRopeSettings.h"
#include "UI/RopeAimWidget.h"
#include "UI/RopePullGaugeWidget.h"
#include "Blueprint/UserWidget.h"

#include "Components/SceneComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "Animation/AnimInstance.h"
#include "Animation/AnimMontage.h"

#include "Components/InputComponent.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"

namespace
{
	void SetThrowContextForward(FRopeThrowContext& Context, const FVector& Forward)
	{
		const FVector SafeForward = Forward.GetSafeNormal();
		if (SafeForward.IsNearlyZero())
		{
			return;
		}

		FVector Up = Context.FrameUp.GetSafeNormal();
		if (Up.IsNearlyZero() || FMath::Abs(FVector::DotProduct(Up, SafeForward)) > 0.98f)
		{
			Up = FMath::Abs(FVector::DotProduct(FVector::UpVector, SafeForward)) < 0.98f
				? FVector::UpVector
				: FVector::RightVector;
		}

		FVector Right = FVector::CrossProduct(Up, SafeForward).GetSafeNormal();
		if (Right.IsNearlyZero())
		{
			Right = FVector::CrossProduct(FVector::RightVector, SafeForward).GetSafeNormal();
		}
		if (Right.IsNearlyZero())
		{
			return;
		}

		Context.FrameForward = SafeForward;
		Context.FrameRight = Right;
		Context.FrameUp = FVector::CrossProduct(SafeForward, Right).GetSafeNormal();
	}
}

URopeWielderComponent::URopeWielderComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = false;
}

void URopeWielderComponent::BeginPlay()
{
	Super::BeginPlay();

	ResolveRefs();

	if (!Rope)
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("RopeWielder on %s: no URopeComponent found (set Rope or add one to the actor)."),
			*GetNameSafe(GetOwner()));
	}

	if (bAttachOnBeginPlay)
	{
		AttachRopeToSocket();
	}

	if (bAutoBindInput)
	{
		// 늦은/재 빙의 대응: 지금 되면 지금 걸고, 안 되면 possession/restart 훅이 다시 시도한다.
		// (컨트롤러 없이 스폰된 폰, 클라이언트 지연 빙의, unpossess 후 재빙의 — 종전엔 전부 영구 누락)
		if (APawn* OwnerPawn = Cast<APawn>(GetOwner()))
		{
			OwnerPawn->ReceiveControllerChangedDelegate.AddDynamic(this, &URopeWielderComponent::HandlePawnControllerChanged);
			OwnerPawn->ReceiveRestartedDelegate.AddDynamic(this, &URopeWielderComponent::HandlePawnRestarted);
		}
		RefreshInputRegistration();
	}

	// 모드 유도 상태(preview 생성/틱 활성/조준 샘플)는 RefreshModeDerivedState 한 곳으로 통일 —
	// 런타임 프리셋 적용(ApplyPreset → OnPresetApplied)이 같은 경로를 재사용한다. Rope가 BeginPlay
	// 이후에 해석되는 비정상 순서라면 구독이 빠진다 — 그때는 게임 코드가 Refresh를 수동 호출한다.
	if (Rope)
	{
		Rope->OnPresetApplied.AddUniqueDynamic(this, &URopeWielderComponent::HandleRopePresetApplied);
	}
	RefreshModeDerivedState();
}

void URopeWielderComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// 입력 바인딩/매핑 정리(컴포넌트 파괴 후 댕글링 델리게이트 방지).
	if (APawn* Pawn = Cast<APawn>(GetOwner()))
	{
		Pawn->ReceiveControllerChangedDelegate.RemoveDynamic(this, &URopeWielderComponent::HandlePawnControllerChanged);
		Pawn->ReceiveRestartedDelegate.RemoveDynamic(this, &URopeWielderComponent::HandlePawnRestarted);
		if (UEnhancedInputComponent* EIC = Cast<UEnhancedInputComponent>(Pawn->InputComponent))
		{
			EIC->ClearBindingsForObject(this);
		}
	}
	RemoveMappingContext();
	bInputBound = false;
	BoundInputComponent.Reset();
	ClearThrowPreview();
	if (PreviewComponent)
	{
		PreviewComponent->ReleasePreviewOwner(this);
		PreviewComponent = nullptr;
	}
	if (AimHudWidget)
	{
		AimHudWidget->RemoveFromParent();
		AimHudWidget = nullptr;
	}
	if (PullGaugeWidget)
	{
		PullGaugeWidget->RemoveFromParent();
		PullGaugeWidget = nullptr;
	}
	if (Rope)
	{
		Rope->CancelQueuedGuaranteedAimThrow();
		bGuaranteedAimThrowQueued = false;
		// Wielder가 사라진 뒤에도 로프의 collider 수집 범위가 조준 ray 방향으로 남지 않게 정리한다.
		Rope->ClearAimRayColliderQueryBounds();
		Rope->OnPresetApplied.RemoveDynamic(this, &URopeWielderComponent::HandleRopePresetApplied);
	}

	// 스윙 중 파괴/레벨 전환 시 AirControl 원복 누락 방지.
	if (bAirControlBoosted)
	{
		if (const ACharacter* Character = Cast<ACharacter>(GetOwner()))
		{
			if (UCharacterMovementComponent* Movement = Character->GetCharacterMovement())
			{
				Movement->AirControl = SavedAirControl;
			}
		}
		bAirControlBoosted = false;
	}

	Super::EndPlay(EndPlayReason);
}

void URopeWielderComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	UpdateGroundExit();
	UpdateSwingAirControl();
	UpdatePullEngage();
	UpdatePullGlowMaterial();
	UpdatePullGaugeWidget();
	UpdateAimHudSample();
	UpdateAimHudWidget();
	UpdateThrowPreview();
}

void URopeWielderComponent::UpdateAimHudSample()
{
	if (!Rope)
	{
		ResolveRefs();
	}

	const bool bHadTarget = AimHudSample.bHasTarget;
	USceneComponent* PrevMesh = AimHudSample.Mesh;
	const FName PrevBone = AimHudSample.Bone;

	AimHudSample = FRopeAimHudSample();
	bHasAimRayFrameThrowContext = false;
	AimRayFrameContextStamp = GFrameCounter;
	// 던질 수 없는 phase에서는 조준 스윕 자체를 돌리지 않는다 — 샘플이 비면 위젯/디버거가 알아서 숨는다.
	// (③ 비-Reel에서 이 스윕이 유일한 SDF 비용이었다: UpdateThrowPreview는 이미 prepared를 안 만든다.)
	if (IsAimActive())
	{
		const FRopeAimRayThrowRequest CurrentRequest = BuildAimRayThrowRequest(FVector::ZeroVector);
		// PrePhysics에서는 요청만 등록한다. PostPhysics의 정상 collider gather 직후 Rope가 해석하고,
		// 여기서는 직전 gather 결과를 소비한다(최대 1프레임 지연, 전역 provider 수집은 프레임당 1회).
		Rope->QueueAimRayQuery(CurrentRequest);

		FRopeAimRayQueryResult QueryResult;
		const bool bHasResolvedQuery = Rope->GetLatestAimRayQueryResult(QueryResult) && QueryResult.IsValid();
		const FRopeAimRayThrowRequest& DisplayRequest = bHasResolvedQuery ? QueryResult.Request : CurrentRequest;
		const FRopeThrowContext& ResolvedContext = bHasResolvedQuery
			? QueryResult.ResolvedContext
			: CurrentRequest.BaseContext;
		const FVector RayDirection = DisplayRequest.RayDirection.GetSafeNormal();
		AimHudSample.RayOrigin = DisplayRequest.RayOrigin;
		AimHudSample.RayDirection = RayDirection;
		AimHudSample.RayLength = DisplayRequest.RayLength;
		// 설정 반경이 0(기본)이면 로프/접촉 폴백이 걸린다 — 질의가 실제로 쓴 값을 그대로 담아야
		// HUD/디버거가 검사 두께를 정확히 그린다.
		AimHudSample.QueryRadius = Rope->GetAimRayEffectiveQueryRadius(DisplayRequest.QueryRadius);
		AimHudSample.AimWorldPos = DisplayRequest.RayOrigin + RayDirection * DisplayRequest.RayLength;
		// 이 샘플이 조준 시각화의 단일 소스다 — HUD 위젯과 Gameplay Debugger([J]aim)가 함께 읽는다.
		if (bHasResolvedQuery && QueryResult.bHitTarget && QueryResult.Hit.bHit && QueryResult.Hit.Mesh)
		{
			const FRopeAimRayHitResult& Hit = QueryResult.Hit;
			const USceneComponent* HitMesh = Hit.Mesh;
			AimHudSample.bHasTarget = true;
			AimHudSample.Bone = Hit.Bone;
			// 샘플은 읽기 전용 계약(헤더 주석) — BP 노출을 위해 non-const로 보관만 한다.
			AimHudSample.Mesh = const_cast<USceneComponent*>(HitMesh);
			AimHudSample.TargetWorldPos = ResolveBindingWorld(HitMesh, Hit.Bone).GetLocation();
			AimHudSample.HitWorldPos = Hit.HitWorldPos;
			AimHudSample.TargetRadius = Hit.TargetBoundsRadius;
			AimHudSample.Distance = Hit.Distance;
			AimHudSample.AimWorldPos = Hit.HitWorldPos;
		}
		else if (bHasResolvedQuery && QueryResult.BlockedHit.bHit)
		{
			const FRopeAimRayHitResult& Blocked = QueryResult.BlockedHit;
			// ray는 맞았지만 wrap 불가 — 빨강 표시. 본 바인딩이 없을 수 있어 걸린 지점을 링 중심으로 쓴다.
			AimHudSample.bBlocked = true;
			AimHudSample.Bone = Blocked.Bone;
			AimHudSample.Mesh = const_cast<USceneComponent*>(Blocked.Mesh);
			AimHudSample.TargetWorldPos = Blocked.HitWorldPos;
			AimHudSample.HitWorldPos = Blocked.HitWorldPos;
			AimHudSample.TargetRadius = Blocked.TargetBoundsRadius;
			AimHudSample.Distance = Blocked.Distance;
			AimHudSample.AimWorldPos = Blocked.HitWorldPos;
		}

		if (bHasResolvedQuery && DisplayRequest.IsValid())
		{
			AimRayFrameThrowContext = ResolvedContext;
			bHasAimRayFrameThrowContext = true;
		}
	}
	else if (Rope)
	{
		if (bGuaranteedAimThrowQueued)
		{
			Rope->CancelQueuedGuaranteedAimThrow();
			bGuaranteedAimThrowQueued = false;
		}
		// 조준 불가능한 phase에서는 이전 ray bounds가 collider 수집 범위를 계속 넓히지 않게 한다.
		Rope->ClearAimRayColliderQueryBounds();
	}

	// 대상 (Mesh, Bone) 변화 통지 — 진입/전환은 Changed, 이탈은 Lost.
	if (AimHudSample.bHasTarget && (!bHadTarget || AimHudSample.Mesh != PrevMesh || AimHudSample.Bone != PrevBone))
	{
		OnAimTargetChanged.Broadcast(AimHudSample.Mesh, AimHudSample.Bone);
	}
	else if (!AimHudSample.bHasTarget && bHadTarget)
	{
		OnAimTargetLost.Broadcast();
	}
}

void URopeWielderComponent::UpdateAimHudWidget()
{
	const bool bWantWidget = bShowAimHudWidget && UsesAimRay();
	if (!bWantWidget)
	{
		if (AimHudWidget)
		{
			AimHudWidget->RemoveFromParent();
			AimHudWidget = nullptr;
		}
		return;
	}
	if (AimHudWidget)
	{
		return;
	}

	// 로컬 플레이어 컨트롤러가 준비된 뒤에만 생성한다(지연 빙의 대비 — 준비 전에는 다음 틱 재시도).
	const APawn* Pawn = Cast<APawn>(GetOwner());
	APlayerController* PC = Pawn ? Cast<APlayerController>(Pawn->GetController()) : nullptr;
	if (!PC || !PC->IsLocalController())
	{
		return;
	}

	// 위젯 클래스는 프로젝트 세팅 단일 소스(기본 = C++ URopeAimWidget, WBP로 교체 가능). 비우면 HUD 없음.
	// 데모 HUD 규모라 동기 로드를 허용한다(최초 1회).
	UClass* WidgetClass = UDynamicRopeSettings::Get()->AimHudWidgetClass.LoadSynchronous();
	if (!WidgetClass)
	{
		return;
	}
	AimHudWidget = CreateWidget<URopeAimWidget>(PC, WidgetClass);
	if (AimHudWidget)
	{
		AimHudWidget->AddToViewport();
	}
}

void URopeWielderComponent::UpdatePullGaugeWidget()
{
	// 게이지는 aim 모드와 무관하다(①에서도 감고 당길 수 있다) — 토글만 보고 유지한다.
	// 장전 전에는 위젯이 아무것도 그리지 않으므로 상태별 생성/파괴를 하지 않는다(깜빡임 방지).
	if (!bShowPullGaugeWidget)
	{
		if (PullGaugeWidget)
		{
			PullGaugeWidget->RemoveFromParent();
			PullGaugeWidget = nullptr;
		}
		return;
	}
	if (PullGaugeWidget)
	{
		return;
	}

	// 로컬 플레이어 컨트롤러가 준비된 뒤에만 생성한다(지연 빙의 대비 — 준비 전에는 다음 틱 재시도).
	const APawn* Pawn = Cast<APawn>(GetOwner());
	APlayerController* PC = Pawn ? Cast<APlayerController>(Pawn->GetController()) : nullptr;
	if (!PC || !PC->IsLocalController())
	{
		return;
	}

	// 위젯 클래스는 프로젝트 세팅 단일 소스(기본 = C++ URopePullGaugeWidget). 비우면 게이지 없음.
	UClass* WidgetClass = UDynamicRopeSettings::Get()->PullGaugeWidgetClass.LoadSynchronous();
	if (!WidgetClass)
	{
		return;
	}
	PullGaugeWidget = CreateWidget<URopePullGaugeWidget>(PC, WidgetClass);
	if (PullGaugeWidget)
	{
		// 위젯이 소유 폰에서 wielder를 스스로 찾지만, 여기서는 확정적으로 배선해 준다.
		PullGaugeWidget->SetWielder(this);
		PullGaugeWidget->AddToViewport();
	}
}

bool URopeWielderComponent::IsWielderTetherActive() const
{
	if (!Rope || Rope->GetPhase() != ERopePhase::Wrapped)
	{
		return false;
	}
	// wielder가 실제로 테더 몫을 받을 때만 — 이번 프레임 유효 몫(λ 역질량비/끌림 판정 이진값)이
	// 전량 대상(=1)이면 wielder는 자유끝이라 스윙/지상이탈 반응이 무의미하다.
	if (Rope->GetEffectiveTetherTargetShare() >= 1.0f - KINDA_SMALL_NUMBER)
	{
		return false;
	}
	// 셀프랩(자기 자신에 감김)은 테더가 wielder 몫을 주지 않는다.
	if (const USkeletalMeshComponent* WrappedMesh = Rope->GetWrappedMesh())
	{
		if (WrappedMesh->GetOwner() == GetOwner())
		{
			return false;
		}
	}
	return true;
}

void URopeWielderComponent::UpdateGroundExit()
{
	if (!bAutoGroundExitOnUpwardPull || !IsWielderTetherActive())
	{
		return;
	}
	// 전 체인이 팽팽해야 테더가 실제로 인가된다 — overshoot는 슬랙 체인에서도 sub-leg 스트레치로
	// >0일 수 있으므로(움직이는 앵커), 그것만 보고 지상을 이탈하면 견인 없는 헛 낙하가 된다.
	if (!Rope->IsChainTaut() || Rope->GetTetherOvershoot() < GroundExitMinOvershoot)
	{
		return;
	}

	const ACharacter* Character = Cast<ACharacter>(GetOwner());
	UCharacterMovementComponent* Movement = Character ? Character->GetCharacterMovement() : nullptr;
	if (!Movement || !Movement->IsMovingOnGround())
	{
		return;
	}

	// 견인 방향(손→앵커) = Pull 샘플 방향(앵커→손 다리 추종)의 역. 상향 성분이 충분할 때만 이탈 —
	// 수평 견인은 walking 그대로 끌리는 게 자연스럽다. 착지 시 walking 복귀는 엔진이 처리한다.
	FVector DirToHand = FVector::ZeroVector;
	float Tension = 0.0f;
	if (!Rope->GetPullSample(DirToHand, Tension))
	{
		return;
	}
	const FVector WielderDir = -DirToHand;
	if (WielderDir.Z >= GroundExitUpDot)
	{
		Movement->SetMovementMode(MOVE_Falling);
	}
}

void URopeWielderComponent::UpdateSwingAirControl()
{
	ACharacter* Character = Cast<ACharacter>(GetOwner());
	UCharacterMovementComponent* Movement = Character ? Character->GetCharacterMovement() : nullptr;
	if (!Movement)
	{
		return;
	}

	// 스윙 = 공중 + wielder 몫 테더 활성. 끝나면(착지/release/설정 변경) 저장해 둔 원래 값으로 복원.
	// 부스트 중 외부에서 AirControl을 바꾸면 복원 시 덮어쓴다(데모 수준 한계 — 주석으로 계약).
	const bool bSwinging = bBoostAirControlWhileSwinging && Movement->IsFalling() && IsWielderTetherActive();
	if (bSwinging && !bAirControlBoosted)
	{
		SavedAirControl = Movement->AirControl;
		Movement->AirControl = SwingAirControl;
		bAirControlBoosted = true;
	}
	else if (!bSwinging && bAirControlBoosted)
	{
		Movement->AirControl = SavedAirControl;
		bAirControlBoosted = false;
	}
}

void URopeWielderComponent::ResolveRefs()
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}
	if (!Rope)
	{
		Rope = Owner->FindComponentByClass<URopeComponent>();
	}
	if (!AttachMesh)
	{
		AttachMesh = Owner->FindComponentByClass<USkeletalMeshComponent>();
	}
}

// 조준/던지기 방식은 로프 ResolveMode에서 유도된다(2026-07-13 회의 결정 F — 종전 AimMode/
// ThrowMode 스위치 대체). 로프가 없으면 조준 보정도 preview 구속도 없다(①과 동일하게 동작).
bool URopeWielderComponent::UsesAimRay() const
{
	return Rope && Rope->ResolveMode != ERopeWrapResolveMode::FullSimulation;
}

bool URopeWielderComponent::IsAimActive() const
{
	// 던질 수 없는 phase에서는 조준할 이유가 없다 — ③는 Reel 전용이라 Free/Wrapped 등에서 HUD가 꺼진다.
	// 게이트는 로프가 소유(CanThrowNow) — 던지기 진입과 같은 술어를 봐야 HUD와 실제 가능 여부가 갈리지 않는다.
	return UsesAimRay() && Rope->CanThrowNow();
}

bool URopeWielderComponent::UsesLockedPreview() const
{
	return Rope && Rope->ResolveMode == ERopeWrapResolveMode::GuaranteedWrap;
}

void URopeWielderComponent::ResolvePreviewComponent(bool bAllowAutoCreate)
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	if (PreviewComponent && !IsValid(PreviewComponent))
	{
		PreviewComponent = nullptr;
	}
	if (PreviewComponent)
	{
		return;
	}

	URopePreviewComponent* ReferencedPreviewComponent = nullptr;
	if (UActorComponent* ReferencedComponent = PreviewComponentReference.GetComponent(Owner))
	{
		ReferencedPreviewComponent = Cast<URopePreviewComponent>(ReferencedComponent);
		if (ReferencedPreviewComponent)
		{
			if (ReferencedPreviewComponent->TryClaimPreviewOwner(this))
			{
				PreviewComponent = ReferencedPreviewComponent;
			}
			else
			{
				UE_LOG(LogDynamicRope, Warning,
					TEXT("RopeWielder on %s: referenced RopePreviewComponent '%s' is already used by another wielder."),
					*GetNameSafe(Owner), *GetNameSafe(ReferencedPreviewComponent));
			}
		}
	}

	if (!PreviewComponent)
	{
		TArray<URopePreviewComponent*> PreviewComponents;
		Owner->GetComponents(PreviewComponents);
		for (URopePreviewComponent* Candidate : PreviewComponents)
		{
			if (!Candidate || Candidate == ReferencedPreviewComponent)
			{
				continue;
			}

			if (Candidate->TryClaimPreviewOwner(this))
			{
				PreviewComponent = Candidate;
				break;
			}
		}
	}

	if (!PreviewComponent && bAllowAutoCreate)
	{
		// 편의 자동 생성 — 표시를 원하는 ③에서만(BeginPlay가 그렇게 호출한다). 게임플레이 필수는 아니다:
		// prepared 계산은 Rope/Wielder가 하므로 이 컴포넌트가 없어도 ③은 정상적으로 던져진다.
		// 이미 레벨/BP에 배치된 PreviewComponent가 있으면 그 설정을 우선 사용하고 여기로 오지 않는다.
		const FName PreviewName = MakeUniqueObjectName(Owner, URopePreviewComponent::StaticClass(), TEXT("RopePreviewComponent"));
		PreviewComponent = NewObject<URopePreviewComponent>(Owner, URopePreviewComponent::StaticClass(), PreviewName);
		if (PreviewComponent)
		{
			PreviewComponent->TryClaimPreviewOwner(this);
			Owner->AddInstanceComponent(PreviewComponent);
			if (USceneComponent* Root = Owner->GetRootComponent())
			{
				PreviewComponent->SetupAttachment(Root);
			}
			PreviewComponent->RegisterComponent();
		}
	}
}

void URopeWielderComponent::AttachRopeToSocket()
{
	if (!Rope || !AttachMesh)
	{
		return;
	}
	if (!HandSocketName.IsNone() && !AttachMesh->DoesSocketExist(HandSocketName))
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("RopeWielder on %s: socket '%s' not found on %s — attaching to component root."),
			*GetNameSafe(GetOwner()), *HandSocketName.ToString(), *AttachMesh->GetName());
	}
	Rope->AttachToComponent(AttachMesh, FAttachmentTransformRules::SnapToTargetNotIncludingScale, HandSocketName);
}

void URopeWielderComponent::HandlePawnControllerChanged(APawn* OwnerPawn, AController* OldController,
	AController* NewController)
{
	if (!NewController)
	{
		// unpossess — IMC를 그 로컬 플레이어에서 떼어 둔다(다음 빙의 때 새 플레이어에 다시 꽂는다).
		// 바인딩은 InputComponent에 붙어 있으므로 여기서 건드리지 않는다(같은 컴포넌트로 돌아오면 그대로 유효).
		RemoveMappingContext();
		return;
	}
	// 빙의 직후엔 InputComponent가 아직 없을 수 있다 — 그 경우 아래 restart 훅이 마저 성사시킨다.
	RefreshInputRegistration();
}

void URopeWielderComponent::HandlePawnRestarted(APawn* OwnerPawn)
{
	// PawnClientRestart(→ SetupPlayerInputComponent) 이후라 InputComponent가 준비돼 있다.
	RefreshInputRegistration();
}

void URopeWielderComponent::RefreshInputRegistration()
{
	// 빙의가 바뀌면 IMC가 붙어야 할 로컬 플레이어도 바뀔 수 있다 — 옛 곳에서 떼고 새 곳에 꽂는다.
	RemoveMappingContext();
	AddMappingContext();
	BindInput();
}

void URopeWielderComponent::RemoveMappingContext()
{
	// IMC는 Pawn이 아니라 LocalPlayer에 등록됐다 — 폰이 먼저 unpossess된 뒤 파괴되면 GetController()가 null이라
	// 종전엔 제거가 건너뛰어져 IMC가 로컬 플레이어에 영구 잔류했다(#11). 추가 시점에 캐시한 서브시스템으로
	// possession 상태와 무관하게 제거한다(LocalPlayer가 이미 파괴됐으면 weak가 null → 제거 불필요).
	if (!MappingContext)
	{
		return;
	}
	if (UEnhancedInputLocalPlayerSubsystem* Sub = MappedInputSubsystem.Get())
	{
		Sub->RemoveMappingContext(MappingContext);
	}
	MappedInputSubsystem.Reset();
}

void URopeWielderComponent::AddMappingContext()
{
	if (!MappingContext)
	{
		return;
	}
	APawn* Pawn = Cast<APawn>(GetOwner());
	APlayerController* PC = Pawn ? Cast<APlayerController>(Pawn->GetController()) : nullptr;
	ULocalPlayer* LP = PC ? PC->GetLocalPlayer() : nullptr;
	if (UEnhancedInputLocalPlayerSubsystem* Sub = LP ? LP->GetSubsystem<UEnhancedInputLocalPlayerSubsystem>() : nullptr)
	{
		Sub->AddMappingContext(MappingContext, MappingPriority);
		// EndPlay가 possession 무관하게 제거하도록 서브시스템을 캐시(#11).
		MappedInputSubsystem = Sub;
	}
}

void URopeWielderComponent::BindInput()
{
	APawn* Pawn = Cast<APawn>(GetOwner());
	if (!Pawn)
	{
		// 입력은 Pawn 전용.
		return;
	}
	UEnhancedInputComponent* EIC = Cast<UEnhancedInputComponent>(Pawn->InputComponent);
	if (!EIC)
	{
		// 아직 빙의/입력 셋업 전일 수 있다 — bAutoBindInput이면 possession/restart 훅이 다시 부른다.
		// 수동 모드라면 Pawn의 SetupPlayerInputComponent에서 BindInput()을 호출하면 된다.
		UE_LOG(LogDynamicRope, Verbose, TEXT("RopeWielder on %s: EnhancedInputComponent not ready — will retry on possess/restart (or call BindInput() from SetupPlayerInputComponent)."),
			*GetNameSafe(GetOwner()));
		return;
	}

	// 중복 방지는 "이미 걸었나"가 아니라 "**이** 컴포넌트에 걸었나"로 판정한다 — 재빙의로 새 InputComponent가
	// 생기면 옛 플래그만 보고 건너뛰어 입력이 영영 안 걸렸다.
	if (bInputBound && BoundInputComponent.Get() == EIC)
	{
		return;
	}
	if (UInputComponent* Old = BoundInputComponent.Get())
	{
		// 옛 컴포넌트가 아직 살아 있으면 이중 발화하지 않게 우리 바인딩만 걷어낸다.
		if (UEnhancedInputComponent* OldEIC = Cast<UEnhancedInputComponent>(Old))
		{
			OldEIC->ClearBindingsForObject(this);
		}
	}

	if (ThrowAction)
	{
		EIC->BindAction(ThrowAction, ETriggerEvent::Started, this, &URopeWielderComponent::OnThrowInput);
	}
	if (ReleaseAction)
	{
		EIC->BindAction(ReleaseAction, ETriggerEvent::Started, this, &URopeWielderComponent::OnReleaseInput);
	}
	if (PullAction)
	{
		// 토글 시맨틱: 누를 때마다 장전↔해제. 발동 시점은 장력 임계가 정한다(UpdatePullEngage) —
		// Completed/Canceled 바인딩 불필요(홀드 아님).
		EIC->BindAction(PullAction, ETriggerEvent::Started, this, &URopeWielderComponent::OnPullInputStarted);
	}
	if (ReelInAction)
	{
		EIC->BindAction(ReelInAction, ETriggerEvent::Started,   this, &URopeWielderComponent::OnReelInStarted);
		EIC->BindAction(ReelInAction, ETriggerEvent::Completed, this, &URopeWielderComponent::OnReelCompleted);
		EIC->BindAction(ReelInAction, ETriggerEvent::Canceled,  this, &URopeWielderComponent::OnReelCompleted);
	}
	if (ReelOutAction)
	{
		EIC->BindAction(ReelOutAction, ETriggerEvent::Started,   this, &URopeWielderComponent::OnReelOutStarted);
		EIC->BindAction(ReelOutAction, ETriggerEvent::Completed, this, &URopeWielderComponent::OnReelCompleted);
		EIC->BindAction(ReelOutAction, ETriggerEvent::Canceled,  this, &URopeWielderComponent::OnReelCompleted);
	}
	if (ReloadAction)
	{
		// 장전은 단발(누름) — ③ 로프를 던지기 준비(Reel) 상태로 전환.
		EIC->BindAction(ReloadAction, ETriggerEvent::Started, this, &URopeWielderComponent::OnReloadInput);
	}
	bInputBound = true;
	BoundInputComponent = EIC;
}

void URopeWielderComponent::OnThrowInput()
{
	// ReleaseAction이 없고 토글 모드면 한 버튼으로 던지기/해제.
	if (!ReleaseAction && bThrowActionToggles)
	{
		ToggleThrow();
	}
	else
	{
		Throw();
	}
}

void URopeWielderComponent::OnReleaseInput()
{
	Release();
}

void URopeWielderComponent::StartPull()
{
	// 장전(토글 on): 힘/몽타주는 여기서 시작하지 않는다 — 발동은 UpdatePullEngage가 "Wrapped + 장력이
	// PullEngageTension을 처음 넘는 순간" 1회 수행한다. 감기 전에 장전해 두면 감겨서 당겨지는 순간 발동한다.
	SetPullArmed(true);
}

void URopeWielderComponent::UpdatePullEngage()
{
	// 장전(bPullArmed)된 Pull의 발동 판정: Wrapped + 장력 조건을 처음 만족하는 **순간** 발동하는 래치
	// (wrap당 1회). 애니 window(UAnimNotifyState_RopePull)가 정하던 "언제 힘을 싣나"를 장력 임계가 대신하고,
	// 몽타주 셋업이면 그 window를 품은 몽타주를 **단일 재생**한다(반복/중단 관리 없음 — 자연 종료).
	// 무애니 셋업이면 즉시 힘을 장전한다. 발동 후 프레임별 인가 게이트(bActivePullRequiresTaut 등)는
	// 로프가 동일하게 판정하고, wrap이 풀리면 힘을 끄고 재무장한다(장전 유지 — 다음 wrap에서 재발동).
	if (!bPullArmed || !Rope)
	{
		return;
	}
	if (Rope->GetPhase() != ERopePhase::Wrapped)
	{
		if (bPullEngaged)
		{
			// 재무장 — 장전은 유지된 채 다음 wrap에서 다시 발동한다. UI가 발동 표시를 끄도록 알린다.
			SetPullEngaged(false, 0.0f);
			StopPullNow();
		}
		return;
	}
	if (bPullEngaged)
	{
		return; // wrap당 1회 — 유지/해제는 로프 게이트와 wrap 수명이 담당.
	}
	// 발동 판정: 임계 0 = 팽팽 래치(IsPullTaut — 로프 게이트와 동일 판정), > 0 = 최대 장력 임계.
	const bool bEngage = (PullEngageTension <= 0.0f)
		? Rope->IsPullTaut()
		: (Rope->GetMaxTension() >= PullEngageTension);
	if (!bEngage)
	{
		return;
	}
	SetPullEngaged(true, Rope->GetMaxTension());
	// 발동 순간 스냅샷(원샷) — PullEngageTension 튜닝용 관측.
	UE_LOG(LogDynamicRope, Log,
		TEXT("[PullEngage] share=%.2f tautT=%.0f tetherT=%.0f overshoot=%.0f"),
		Rope->GetEffectiveTetherTargetShare(),
		Rope->GetMaxTension(), Rope->GetTetherTension(), Rope->GetTetherOvershoot());
	if (PullMontage)
	{
		// 몽타주 단일 재생 — 힘은 안의 window notify(StartPullNow/StopPullNow)가 싣는다.
		PlayPullMontage();
	}
	else
	{
		StartPullNow();
	}
}

void URopeWielderComponent::StartPullNow(bool bIgnoreTautGate)
{
	if (Rope)
	{
		Rope->SetActivePull(Rope->HoldConfig.PullForce, bIgnoreTautGate);
	}
}

void URopeWielderComponent::StopPullNow()
{
	if (Rope)
	{
		Rope->SetActivePull(0.0f);
	}
}

void URopeWielderComponent::StopPull()
{
	// 장전 해제(토글 off) + 힘 정지 + 발동 래치 리셋. 몽타주는 중단하지 않는다 — 단일 재생 계약이라 재생
	// 수명을 여기서 관리하지 않고 자연 종료에 맡긴다(해제 순간 모션이 뚝 끊기는 것 방지).
	SetPullArmed(false);
	SetPullEngaged(false, 0.0f);
	StopPullNow();
}

void URopeWielderComponent::SetPullArmed(bool bNewArmed)
{
	if (bPullArmed == bNewArmed)
	{
		return;
	}
	bPullArmed = bNewArmed;
	OnPullArmedChanged.Broadcast(bPullArmed);
}

void URopeWielderComponent::SetPullEngaged(bool bNewEngaged, float Tension)
{
	if (bPullEngaged == bNewEngaged)
	{
		return;
	}
	bPullEngaged = bNewEngaged;
	OnPullEngagedChanged.Broadcast(bPullEngaged, bPullEngaged ? Tension : 0.0f);
}

float URopeWielderComponent::GetPullEngageProgress() const
{
	if (!Rope || Rope->GetPhase() != ERopePhase::Wrapped)
	{
		// 감기기 전에는 임계 자체가 성립하지 않는다 — 게이지를 0으로 두면 "아직 감아야 한다"가 읽힌다.
		return 0.0f;
	}
	if (bPullEngaged)
	{
		return 1.0f;
	}
	if (PullEngageTension <= 0.0f)
	{
		// 임계 0 = 팽팽 판정만으로 발동 — 연속값이 없으므로 게이트와 같은 판정을 0/1로 돌려준다.
		return Rope->IsPullTaut() ? 1.0f : 0.0f;
	}
	return FMath::Clamp(Rope->GetMaxTension() / PullEngageTension, 0.0f, 1.0f);
}

void URopeWielderComponent::UpdatePullGlowMaterial()
{
	if (!bDrivePullGlowMaterial || !Rope)
	{
		return;
	}

	// 한 번도 장전한 적 없으면 머티리얼 배선을 건드리지 않는다 — pull을 안 쓰는 로프의 렌더 상태를
	// 이 기능이 조용히 바꾸지 않게 하는 게 목적이다.
	UMaterialInstanceDynamic* MID = PullGlowMID.Get();
	if (!MID && !bPullArmed)
	{
		return;
	}

	// 프리셋 적용 등으로 로프 머티리얼이 교체되면 우리 MID는 더 이상 로프에 붙어 있지 않다 — 다시 만든다.
	if (!MID || Rope->GetMaterial(0) != MID)
	{
		MID = Rope->CreateAndSetMaterialInstanceDynamic(0);
		PullGlowMID = MID;
		if (!MID)
		{
			return;
		}
	}

	const float Progress = GetPullEngageProgress();
	const float GlowValue = bPullEngaged ? PullGlowEngagedValue : Progress;
	// 파라미터가 없는 머티리얼이면 이 호출은 무해한 no-op다(경고도 없다).
	MID->SetScalarParameterValue(PullGlowParameterName, bPullArmed ? GlowValue : 0.0f);
}

void URopeWielderComponent::Cut()
{
	if (Rope)
	{
		Rope->CutRope();
	}
}

void URopeWielderComponent::StartReelIn()
{
	if (Rope)
	{
		Rope->SetReelRate(Rope->ReelSpeed);
	}
}

void URopeWielderComponent::StartReelOut()
{
	if (Rope)
	{
		Rope->SetReelRate(-Rope->ReelSpeed);
	}
}

void URopeWielderComponent::StopReel()
{
	if (Rope)
	{
		Rope->SetReelRate(0.0f);
	}
}

void URopeWielderComponent::OnReelInStarted()
{
	StartReelIn();
}

void URopeWielderComponent::OnReelOutStarted()
{
	StartReelOut();
}

void URopeWielderComponent::OnReelCompleted()
{
	StopReel();
}

void URopeWielderComponent::OnReloadInput()
{
	if (Rope)
	{
		Rope->EnterReel();
	}
}

void URopeWielderComponent::OnPullInputStarted()
{
	// 토글: 누를 때마다 장전 ↔ 해제. 발동(힘/몽타주 단일 재생)은 장전 상태에서 장력 임계가 정한다
	// (UpdatePullEngage — PullEngageTension).
	if (bPullArmed)
	{
		StopPull();
	}
	else
	{
		StartPull();
	}
}

FRopeThrowContext URopeWielderComponent::BuildThrowContext(const FVector& AimDir) const
{
	return BuildThrowContextInternal(AimDir);
}

FVector URopeWielderComponent::GetAimRayOrigin() const
{
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return FVector::ZeroVector;
	}

	const USkeletalMeshComponent* OriginMesh = AttachMesh ? AttachMesh.Get() : Owner->FindComponentByClass<USkeletalMeshComponent>();

	// AimRay는 로프/손 소켓 위치가 아니라 Wielder가 고른 조준 기준 위치에서 쏜다.
	// 최종 투척 방향은 아래에서 Hit - ThrowOrigin으로 다시 계산한다.
	switch (AimRayOriginMode)
	{
	case ERopeAimRayOriginMode::AttachSocketOrBone:
		if (OriginMesh && !AimRayOriginSocketName.IsNone() && OriginMesh->DoesSocketExist(AimRayOriginSocketName))
		{
			return OriginMesh->GetSocketLocation(AimRayOriginSocketName);
		}
		[[fallthrough]];

	case ERopeAimRayOriginMode::AttachMeshBoundsCenter:
		if (OriginMesh)
		{
			return OriginMesh->Bounds.Origin;
		}
		break;

	case ERopeAimRayOriginMode::ViewLocation:
		// 방향을 카메라에서 가져오는 설정이면 원점도 카메라에 맞춘다 — ray의 원점과 방향이 서로 다른 기준을
		// 쓰면 조준선이 화면과 어긋난다. 그 외에는 눈높이(PawnViewLocation).
		if (Rope && Rope->ThrowParams.FrameMode == ERopeThrowFrameMode::OwnerCamera)
		{
			if (const UCameraComponent* Camera = Owner->FindComponentByClass<UCameraComponent>())
			{
				return Camera->GetComponentLocation();
			}
		}
		if (const APawn* Pawn = Cast<APawn>(Owner))
		{
			return Pawn->GetPawnViewLocation();
		}
		break;

	case ERopeAimRayOriginMode::OwnerActorLocation:
	default:
		break;
	}

	return Owner->GetActorLocation();
}

float URopeWielderComponent::GetAimReachLength() const
{
	return Rope ? FMath::Max(Rope->GetCurrentRopeLength(), Rope->RopeLength) : 0.0f;
}

FRopeThrowContext URopeWielderComponent::BuildBaseThrowContext(const FVector& AimDir) const
{
	FRopeThrowContext Context;

	const AActor* Owner = GetOwner();

	// 던지기 파라미터의 단일 소스는 로프의 ThrowParams다(2026-07-13 표면 감사 A-1 — Wielder 사본
	// 7종 제거). Wielder는 손 소켓 원점/소켓 속도/조준 유도 등 "출처"만 컨텍스트에 얹는다.
	const FRopeThrowParams DefaultParams;
	const FRopeThrowParams& Params = Rope ? Rope->ThrowParams : DefaultParams;

	Context.Origin = Rope ? Rope->GetComponentLocation() : (Owner ? Owner->GetActorLocation() : FVector::ZeroVector);
	Context.FrameForward = Owner ? Owner->GetActorForwardVector() : FVector::ForwardVector;
	Context.FrameUp = Owner ? Owner->GetActorUpVector() : FVector::UpVector;
	Context.FrameRight = Owner ? Owner->GetActorRightVector() : FVector::RightVector;
	Context.OwnerVelocity = Owner ? Owner->GetVelocity() : FVector::ZeroVector;
	Context.SocketVelocity = Context.OwnerVelocity;
	Context.FrameMode = Params.FrameMode;
	Context.SwingPlane = Params.SwingPlane;
	Context.CustomSwingPlaneNormal = Params.CustomSwingPlaneNormal;
	Context.ThrowSpeed = Params.ThrowSpeed;
	Context.AimGuideSteerStartAlpha = FMath::Clamp(AimRayGuideSteerStartAlpha, 0.0f, 0.9f);
	Context.AimGuideLockAlpha = FMath::Clamp(
		FMath::Max(AimRayGuideLockAlpha, Context.AimGuideSteerStartAlpha + 0.01f), 0.05f, 1.0f);

	if (AttachMesh)
	{
		Context.Origin = AttachMesh->GetSocketLocation(HandSocketName);
		Context.SocketVelocity = AttachMesh->GetPhysicsLinearVelocity(HandSocketName);
	}

	switch (Params.FrameMode)
	{
	case ERopeThrowFrameMode::Owner:
		if (Owner)
		{
			Context.FrameForward = Owner->GetActorForwardVector();
			Context.FrameUp = Owner->GetActorUpVector();
			Context.FrameRight = Owner->GetActorRightVector();
		}
		break;

	case ERopeThrowFrameMode::OwnerCamera:
		if (const UCameraComponent* Camera = Owner ? Owner->FindComponentByClass<UCameraComponent>() : nullptr)
		{
			Context.FrameForward = Camera->GetForwardVector();
			Context.FrameUp = Camera->GetUpVector();
			Context.FrameRight = Camera->GetRightVector();
		}
		break;

	case ERopeThrowFrameMode::Socket:
		if (AttachMesh)
		{
			const FTransform SocketTransform = AttachMesh->GetSocketTransform(HandSocketName);
			Context.FrameForward = SocketTransform.GetUnitAxis(EAxis::X);
			Context.FrameRight = SocketTransform.GetUnitAxis(EAxis::Y);
			Context.FrameUp = SocketTransform.GetUnitAxis(EAxis::Z);
		}
		break;

	case ERopeThrowFrameMode::Custom:
		Context.FrameForward = Params.CustomFrameForward;
		Context.FrameUp = Params.CustomFrameUp;
		Context.FrameRight = Params.CustomFrameRight;
		break;

	case ERopeThrowFrameMode::World:
	default:
		Context.FrameForward = FVector::ForwardVector;
		Context.FrameUp = FVector::UpVector;
		Context.FrameRight = FVector::RightVector;
		break;
	}

	// ThrowInDirection의 명시적 입력이 있으면 ray와 throw가 같은 방향을 사용해야 한다.
	// 기존 Throw()/ThrowNow()는 ZeroVector를 넘기므로 설정된 frame forward 동작을 그대로 유지한다.
	const FVector ExplicitAimDir = AimDir.GetSafeNormal();
	if (!ExplicitAimDir.IsNearlyZero())
	{
		SetThrowContextForward(Context, ExplicitAimDir);
	}

	return Context;
}

FRopeThrowContext URopeWielderComponent::BuildThrowContextInternal(const FVector& AimDir) const
{
	if (!UsesAimRay())
	{
		return BuildBaseThrowContext(AimDir);
	}

	if (FRopeThrowContext CachedContext; TryGetCachedAimRayThrowContext(AimDir, CachedContext))
	{
		return CachedContext;
	}

	const FRopeAimRayThrowRequest Request = BuildAimRayThrowRequest(AimDir);
	// 첫 조준 프레임 또는 명시 AimDir은 아직 정상 gather에서 확정한 캐시가 없다. 여기서 provider를
	// 즉시 재수집하지 않고 base fallback을 반환한다. 실제 throw는 QueueAimRayThrow가 같은 프레임의
	// PostPhysics gather 직후 확정하며, HUD/preview는 다음 틱부터 위 캐시를 사용한다.
	return Request.BaseContext;
}

bool URopeWielderComponent::TryGetCachedAimRayThrowContext(const FVector& AimDir, FRopeThrowContext& OutContext) const
{
	if (!AimDir.GetSafeNormal().IsNearlyZero())
	{
		return false;
	}
	if (!bHasAimRayFrameThrowContext || AimRayFrameContextStamp != GFrameCounter)
	{
		return false;
	}

	OutContext = AimRayFrameThrowContext;
	return true;
}

FRopeAimRayThrowRequest URopeWielderComponent::BuildAimRayThrowRequest(const FVector& AimDir) const
{
	FRopeAimRayThrowRequest Request;
	Request.BaseContext = BuildBaseThrowContext(AimDir);
	// 조준 ray 경로가 만든 컨텍스트임을 표시한다 — preview 빌더가 "조준 miss"와 "조준 자체가 없음
	// (BP 직행/AI)"을 구분하는 근거다. 조준 컨텍스트의 단일 팩토리인 여기서 한 번만 찍으면 hit/miss는
	// 물론 ray가 무효(RayLength=0 — reach 구를 안 지남)인 경우까지 덮인다: ResolveAimRayThrowContext와
	// BuildThrowContextInternal 둘 다 결과를 BaseContext에서 출발시키므로 플래그가 살아남는다.
	Request.BaseContext.bAimRayEvaluated = true;
	Request.RayOrigin = GetAimRayOrigin();
	Request.RayDirection = Request.BaseContext.FrameForward;
	Request.ReachOrigin = Request.BaseContext.Origin;
	Request.ReachLength = GetAimReachLength();
	Request.RayLength = FRopeAimTargeting::ResolveRayLengthForReach(
		Request.RayOrigin, Request.RayDirection, Request.ReachOrigin, Request.ReachLength);
	Request.QueryRadius = AimRayQueryRadius;
	Request.SweepStep = AimRaySweepStep;
	return Request;
}

bool URopeWielderComponent::QueueGuaranteedAimThrow(const FVector& AimDir, bool bExecuteWhenReady)
{
	if (bGuaranteedAimThrowQueued)
	{
		// 같은 입력/몽타주 요청이 이미 정상 gather 또는 notify를 기다리는 중이다. 기존 요청을 보존한다.
		return false;
	}
	if (!Rope || !Rope->CanThrowNow())
	{
		bGuaranteedAimThrowQueued = false;
		NotifyThrowRejected(ERopeThrowRejectReason::NotInReel);
		OnThrowRejected.Broadcast(ERopeThrowRejectReason::NotInReel);
		return false;
	}

	FRopeAimRayThrowRequest Request = BuildAimRayThrowRequest(AimDir);
	Request.OnPrepared = FRopeAimPreparedDelegate::CreateUObject(
		this, &URopeWielderComponent::OnGuaranteedAimPrepared);
	Request.OnResolved = FSimpleDelegate::CreateUObject(this, &URopeWielderComponent::OnAimRayThrowResolved);
	Request.OnRejected = FSimpleDelegate::CreateUObject(this, &URopeWielderComponent::OnAimRayThrowRejected);
	if (!Rope->QueueGuaranteedAimThrow(Request, bExecuteWhenReady))
	{
		bGuaranteedAimThrowQueued = false;
		NotifyThrowRejected(ERopeThrowRejectReason::RopeRejected);
		OnThrowRejected.Broadcast(ERopeThrowRejectReason::RopeRejected);
		return false;
	}

	bGuaranteedAimThrowQueued = true;
	ClearPreviewDisplay();
	return true;
}

void URopeWielderComponent::Throw()
{
	// 서브클래스 게임 규칙 게이트(스태미나/상태 등). 몽타주 경로의 ThrowNow는 재검사하지 않는다(헤더 계약).
	if (!CanThrow())
	{
		NotifyThrowRejected(ERopeThrowRejectReason::Gated);
		OnThrowRejected.Broadcast(ERopeThrowRejectReason::Gated);
		return;
	}

	if (UsesLockedPreview())
	{
		// ③ 실제 발사는 화면용 1프레임 캐시를 쓰지 않는다. 입력 순간 ray를 같은 프레임 정상 gather에서
		// prepared로 확정하고, 몽타주가 있으면 그 결과만 notify까지 보관한다.
		if (!QueueGuaranteedAimThrow(FVector::ZeroVector, /*bExecuteWhenReady*/ !ThrowMontage))
		{
			return;
		}
		if (ThrowMontage)
		{
			PlayThrowMontage();
		}
		return;
	}

	if (ThrowMontage)
	{
		// 실제 던지기는 몽타주의 UAnimNotify_RopeThrow → ThrowNow().
		PlayThrowMontage();
	}
	else
	{
		ThrowNow();
	}
}

void URopeWielderComponent::ThrowNow()
{
	ThrowInDirection(FVector::ZeroVector);
}

void URopeWielderComponent::ThrowInDirection(const FVector& AimDir)
{
	if (Rope)
	{
		if (UsesLockedPreview())
		{
			// 몽타주 입력이 이미 큐를 만들었다면 notify는 실행 의사만 전달한다. gather가 아직이면 준비 직후,
			// 이미 prepared가 준비됐으면 지금 실행된다.
			if (Rope->RequestExecuteQueuedGuaranteedAimThrow())
			{
				return;
			}

			// ThrowInDirection 직행(BP/코드)처럼 선행 Throw()가 없으면 여기서 현재 ray 요청을 만들고
			// 정상 gather 직후 실행한다.
			QueueGuaranteedAimThrow(AimDir, /*bExecuteWhenReady*/ true);
			return;
		}

		ClearThrowPreview();
		if (UsesAimRay())
		{
			// 최신 collider 수집 직후 hit/fallback을 확정하도록 값 타입 요청만 큐에 넣는다.
			FRopeAimRayThrowRequest Request = BuildAimRayThrowRequest(AimDir);
			Request.OnResolved = FSimpleDelegate::CreateUObject(this, &URopeWielderComponent::OnAimRayThrowResolved);
			Rope->QueueAimRayThrow(Request);
			return;
		}

		Rope->ThrowWithContext(BuildBaseThrowContext(AimDir));
		NotifyThrown();
		OnThrown.Broadcast();
	}
}

void URopeWielderComponent::OnAimRayThrowResolved()
{
	bGuaranteedAimThrowQueued = false;
	LastPreparedPreview.Reset();
	ClearPreviewDisplay();
	// Rope가 실제 실행 페이즈(Assisted=Flight, Guaranteed=GuidedThrow)에 진입한 뒤 성공 알림을 보낸다.
	NotifyThrown();
	OnThrown.Broadcast();
}

void URopeWielderComponent::OnGuaranteedAimPrepared(FRopePreparedThrowPreview& Prepared)
{
	// 입력 프레임에 확정한 world path를 owner-local로 바꿔, 몽타주 동안 캐릭터가 움직여도 notify 실행 시
	// 기존 ③ 계약처럼 현재 owner transform 기준으로 복원한다. 실제 실행 전에 같은 값을 직접 수정한다.
	StoreAimGuideFrameIfNeeded(Prepared);
	HeldPreparedPreview = Prepared.IsValid()
		? ResolvePreparedPreviewForDisplay(Prepared)
		: FRopeWrapPreviewData();
	HeldPreviewExpireTimeSeconds = 0.0f;
	ClearPreviewDisplay();
}

void URopeWielderComponent::OnAimRayThrowRejected()
{
	bGuaranteedAimThrowQueued = false;
	ClearThrowPreview();
	NotifyThrowRejected(ERopeThrowRejectReason::RopeRejected);
	OnThrowRejected.Broadcast(ERopeThrowRejectReason::RopeRejected);
}

void URopeWielderComponent::PlayThrowMontage()
{
	if (!ThrowMontage)
	{
		return;
	}
	if (!AttachMesh)
	{
		ResolveRefs();
	}
	UAnimInstance* Anim = AttachMesh ? AttachMesh->GetAnimInstance() : nullptr;
	if (Anim)
	{
		Anim->Montage_Play(ThrowMontage, ThrowMontagePlayRate);
	}
	else
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("RopeWielder on %s: no AnimInstance to play ThrowMontage."),
			*GetNameSafe(GetOwner()));
	}
}

void URopeWielderComponent::PlayPullMontage()
{
	if (!PullMontage)
	{
		return;
	}
	if (!AttachMesh)
	{
		ResolveRefs();
	}
	UAnimInstance* Anim = AttachMesh ? AttachMesh->GetAnimInstance() : nullptr;
	if (Anim)
	{
		Anim->Montage_Play(PullMontage, PullMontagePlayRate);
	}
	else
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("RopeWielder on %s: no AnimInstance to play PullMontage."),
			*GetNameSafe(GetOwner()));
	}
}


void URopeWielderComponent::Release()
{
	if (Rope)
	{
		Rope->ReleaseWrap();
	}
}

void URopeWielderComponent::ToggleThrow()
{
	if (!Rope)
	{
		return;
	}
	const ERopePhase Phase = Rope->GetPhase();
	if (Phase == ERopePhase::Wrapped || Phase == ERopePhase::Contacting || Phase == ERopePhase::Wrapping ||
		Phase == ERopePhase::GuidedThrow)
	{
		Release();
	}
	else
	{
		Throw();
	}
}

void URopeWielderComponent::SetThrowPreviewEnabled(bool bEnabled)
{
	bShowThrowPreview = bEnabled;
	if (bShowThrowPreview)
	{
		ResolveRefs();
		ResolvePreviewComponent(/*bAllowAutoCreate*/ true);
		SetComponentTickEnabled(true);
		UpdateThrowPreview();
	}
	else
	{
		// 표시만 끈다 — prepared(던지기용)는 그대로 두고, ③/aim ray는 계산 틱을 계속 돌린다.
		ClearPreviewDisplay();
		SetComponentTickEnabled(ComputeDesiredTickEnabled());
	}
}

bool URopeWielderComponent::ComputeDesiredTickEnabled() const
{
	// Guaranteed는 표시를 꺼도 던지기용 prepared 계산에 틱이 필요하다(UsesLockedPreview로 켜진다 —
	// bShowThrowPreview는 표시 on/off일 뿐 계산 게이트가 아니다). preview 외에 지상 이탈/스윙
	// 에어컨트롤 감시도 틱이 필요하다 — 전부 꺼져야 틱 정지. Aim ray 모드(Assisted)는 preview
	// component가 없어도 collider 수집 bounds를 매 프레임 갱신해야 한다.
	return UsesLockedPreview() || bAutoGroundExitOnUpwardPull ||
		bBoostAirControlWhileSwinging || UsesAimRay();
}

void URopeWielderComponent::RefreshModeDerivedState()
{
	// preview는 Guaranteed 모드 전용이다 — 표시를 원하는 Guaranteed에서만 런타임 컴포넌트를 만들어준다
	// (수동 배치가 있으면 그걸 쓴다). ③이 아니게 되면 잔류 preview 표시만 지운다(컴포넌트는 유휴로
	// 남긴다 — 다시 ③이 되면 재사용). HUD 위젯은 UpdateAimHudWidget이 틱마다 재유도하지만, ①로
	// 바뀌며 틱 자체가 꺼질 수 있으므로 꺼지기 전에 위젯 생성/제거를 한 번 정리하고 나간다.
	ResolvePreviewComponent(/*bAllowAutoCreate*/ bShowThrowPreview && UsesLockedPreview());
	if (!UsesLockedPreview())
	{
		if (Rope)
		{
			Rope->CancelQueuedGuaranteedAimThrow();
		}
		bGuaranteedAimThrowQueued = false;
		ClearPreviewDisplay();
	}
	SetComponentTickEnabled(ComputeDesiredTickEnabled());
	UpdateAimHudWidget();
	UpdateAimHudSample();
	if (UsesLockedPreview())
	{
		UpdateThrowPreview();
	}
}

void URopeWielderComponent::HandleRopePresetApplied(const URopePreset* Preset)
{
	RefreshModeDerivedState();
}

bool URopeWielderComponent::ShouldHoldPreparedPreview()
{
	if (!UsesLockedPreview() || !bGuaranteedAimThrowQueued)
	{
		return false;
	}
	if (!ThrowMontage)
	{
		// 즉시 실행도 PostPhysics gather까지는 큐 상태다. 그 사이 직전 HUD 캐시로 preview를 다시 만들지 않는다.
		ClearPreviewDisplay();
		return true;
	}

	if (!AttachMesh)
	{
		ResolveRefs();
	}

	const UAnimInstance* Anim = AttachMesh ? AttachMesh->GetAnimInstance() : nullptr;
	if (Anim && Anim->Montage_IsPlaying(ThrowMontage))
	{
		// 입력 프레임 PostPhysics에서 확정된 결과는 Rope가 notify까지 보관한다. 프리뷰는 Reel 조준에서만
		// 보이면 되므로 윈드업 중에는 숨기고, 직전 HUD 캐시로 새 path를 만들지 않는다.
		ClearPreviewDisplay();
		return true;
	}

	// 몽타주가 notify 없이 끝났거나 재생에 실패했다면 예약된 실제 throw도 함께 취소한다.
	if (Rope)
	{
		Rope->CancelQueuedGuaranteedAimThrow();
	}
	bGuaranteedAimThrowQueued = false;
	return false;
}

bool URopeWielderComponent::ShouldUpdateThrowPreviewForPhase(ERopePhase Phase) const
{
	// Guaranteed 모드는 "던지기 전 성공한 preview path"만 새로 만든다. GuidedThrow/Wrapped에서는 이미
	// 확정된 HeldPreparedPreview를 쓰므로 build를 다시 시도하지 않는다(호출자에서 먼저 걸러진다).
	// Reel(장전 준비) 상태에서만 조준 preview를 만든다 — Reel에서만 던질 수 있으므로. 던지기 게이트와
	// 같은 술어(CanThrowInPhase)를 봐야 "보이는 것 = 던질 수 있는 것"이 유지된다. 라이브 phase가 아니라
	// 인자 Phase로 물어야 이 함수의 시그니처 계약과 어긋나지 않는다.
	return RopeWrapModes::CanThrowInPhase(Rope->ResolveMode, Phase);
}

bool URopeWielderComponent::UpdateHeldPreparedPreviewForPhase(ERopePhase Phase)
{
	if (!UsesLockedPreview() || !HeldPreparedPreview.IsValid())
	{
		return false;
	}

	if (Phase == ERopePhase::GuidedThrow)
	{
		// GuidedThrow는 cached preview path를 authoritative하게 따라가는 상태다. 새 path를 build하지
		// 않는다. 프리뷰는 Reel(조준)에서만 보이면 되므로 발사 후에는 표시를 지운다(HeldPreparedPreview
		// 데이터는 보존 — phase-gate 유효성 검사와 Wrapped-hold 옵션이 참조한다). return true로 이
		// phase에서 preview 재빌드로 떨어지지 않게 한다.
		ClearPreviewDisplay();
		LastPreviewPhase = Phase;
		return true;
	}

	if (Phase == ERopePhase::Wrapped)
	{
		// Wrapped 진입 후에도 옵션 시간만큼 path를 남길 수 있다.
		// 기본값 0초에서는 여기서 바로 ClearThrowPreview()로 떨어진다.
		const float NowSeconds = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
		if (LastPreviewPhase != ERopePhase::Wrapped)
		{
			HeldPreviewExpireTimeSeconds = NowSeconds + FMath::Max(0.0f, LockedWrappedPreviewHoldTime);
		}

		if (LockedWrappedPreviewHoldTime > KINDA_SMALL_NUMBER && NowSeconds < HeldPreviewExpireTimeSeconds)
		{
			DisplayHeldPreparedPreview();
			LastPreviewPhase = Phase;
			return true;
		}

		ClearThrowPreview();
		LastPreviewPhase = Phase;
		return true;
	}

	return false;
}

void URopeWielderComponent::DisplayHeldPreparedPreview()
{
	// 이미 확정된 path를 그대로 유지 표시한다(새 build 없음). 표시가 꺼져 있으면 조용히 넘어간다.
	if (bShowThrowPreview && PreviewComponent && HeldPreparedPreview.IsValid())
	{
		PreviewComponent->SetWrapPreviewWorld(HeldPreparedPreview);
	}
}

void URopeWielderComponent::DisplayPreviewCenterline(const FRopeWrapPreviewData& Centerline)
{
	// 표시는 전적으로 선택 사항 — 계산과 분리돼 있어 여기서 실패해도 prepared(던지기용)는 건드리지 않는다.
	if (!bShowThrowPreview || !PreviewComponent || !Rope)
	{
		return;
	}

	PreviewComponent->SetWrapPreviewWorld(Centerline);
}

void URopeWielderComponent::UpdateThrowPreview()
{
	// preview는 GuaranteedWrap 모드 전용이다 — Reel(장전)에서 조준한 대상을 확정 throw로 던지기 위한
	// prepared path(contact/anchor 포함)를 만든다. 계산(prepared)과 표시(preview 컴포넌트)는 분리돼
	// 있어, 표시를 꺼도·컴포넌트가 없어도 prepared는 만들어야 던질 수 있다.
	// FullSimulation/AssistedJudged는 preview를 쓰지 않는다 — 감김이 판정/창발이라 던지기 전에 확정할
	// 경로가 없다. AssistedJudged의 조준 표시는 aim ray HUD(UpdateAimHudSample)가 따로 담당한다.
	if (!Rope)
	{
		ResolveRefs();
	}
	if (!Rope || !UsesLockedPreview())
	{
		ClearThrowPreview();
		return;
	}
	if (!PreviewComponent)
	{
		ResolvePreviewComponent(/*bAllowAutoCreate*/ false);
	}
	if (ShouldHoldPreparedPreview())
	{
		return;
	}

	const ERopePhase RopePhase = Rope->GetPhase();
	if (UpdateHeldPreparedPreviewForPhase(RopePhase))
	{
		return;
	}
	if (!ShouldUpdateThrowPreviewForPhase(RopePhase))
	{
		// 이 phase에서는 preview build 자체가 의미 없으므로 실패 로그를 만들지 않고 조용히 정리한다.
		ClearThrowPreview();
		LastPreviewPhase = RopePhase;
		return;
	}
	if (!bHasAimRayFrameThrowContext)
	{
		// 첫 조준 프레임은 아직 정상 gather 결과가 없다. base fallback으로 허공 preview를 한 프레임
		// 그렸다가 target path로 바뀌는 깜빡임을 만들지 않고, 다음 틱의 확정 결과를 기다린다.
		ClearThrowPreview();
		LastPreviewPhase = RopePhase;
		return;
	}

	FString PreviewBuildReason;
	const FRopeThrowContext ThrowContext = BuildThrowContext(FVector::ZeroVector);
	FRopePreparedThrowPreview Prepared;
	if (!Rope->BuildPreparedWrappingPreview(ThrowContext, Prepared, &PreviewBuildReason))
	{
		ClearThrowPreview();
		LastPreviewPhase = RopePhase;
		return;
	}

	LastPreparedPreview = Prepared;
	StoreAimGuideFrameIfNeeded(LastPreparedPreview);
	HeldPreparedPreview = ResolvePreparedPreviewForDisplay(LastPreparedPreview);
	HeldPreviewExpireTimeSeconds = 0.0f;
	DisplayPreviewCenterline(HeldPreparedPreview);

	LastPreviewPhase = RopePhase;
}

void URopeWielderComponent::ClearPreviewDisplay()
{
	// 표시만 정리 — prepared(던지기용)는 건드리지 않는다.
	if (PreviewComponent && PreviewComponent->IsPreviewOwner(this))
	{
		PreviewComponent->ClearPreview();
	}
}

void URopeWielderComponent::ClearPreparedThrow()
{
	LastPreparedPreview.Reset();
	HeldPreparedPreview = FRopeWrapPreviewData();
	HeldPreviewExpireTimeSeconds = 0.0f;
}

void URopeWielderComponent::ClearThrowPreview()
{
	ClearPreparedThrow();
	ClearPreviewDisplay();
}

void URopeWielderComponent::StoreAimGuideFrameIfNeeded(FRopePreparedThrowPreview& Prepared) const
{
	if (!UsesAimRay() || !Prepared.IsValid())
	{
		return;
	}

	const AActor* Owner = GetOwner();
	const USceneComponent* OwnerRoot = Owner ? Owner->GetRootComponent() : nullptr;
	if (!OwnerRoot)
	{
		return;
	}

	// aim ray 조준 path는 소켓/로프 컴포넌트 로컬이 아니라 wielder owner 로컬 기준으로 고정한다.
	Prepared.StoreGuideFrameLocal(OwnerRoot);
}

FRopeWrapPreviewData URopeWielderComponent::ResolvePreparedPreviewForDisplay(const FRopePreparedThrowPreview& Prepared) const
{
	// owner-local로 저장되지 않은 일반 preview는 원래 월드 점을 그대로 반환한다.
	FRopeWrapPreviewData Preview = Prepared.ResolveRenderPreviewWorld();
	if (!Rope || Rope->ResolveMode != ERopeWrapResolveMode::GuaranteedWrap || !Preview.IsValid())
	{
		return Preview;
	}

	// Pierce의 throw용 prepared 경로는 마지막 노드가 Tail 소켓에 오도록 RopeComponent가 TailWorld까지 줄인다.
	// 표시는 플레이어가 조준한 Head/Hit 지점까지 이어져야 하므로, 렌더 전용 centerline만 HitPoint까지 다시 편다.
	const FRopeSurfaceAnchor* Anchor = Prepared.Anchors.Num() > 0 ? &Prepared.Anchors[0] : &Prepared.LatchAnchor;
	if (!Anchor || Anchor->NodeIndex == INDEX_NONE)
	{
		return Preview;
	}

	FVector HitPoint = Anchor->StartWorldPosition;
	if (Prepared.ThrowContext.bHasAimGuideHit)
	{
		// Aim guide preview의 표시 끝점은 anchor local을 다시 푼 값이 아니라, 조준 레이가 실제로 선택한 hit이다.
		HitPoint = Prepared.ThrowContext.AimGuideHitWorldPos;
	}
	else
	{
		const USceneComponent* Mesh = Anchor->Mesh.IsValid() ? Anchor->Mesh.Get() : Prepared.Mesh.Get();
		const FName Bone = Anchor->Bone.IsNone() ? Prepared.Bone : Anchor->Bone;
		if (Mesh && !Bone.IsNone())
		{
			HitPoint = ResolveBindingWorld(Mesh, Bone).TransformPosition(Anchor->LocalSurfacePosition);
		}
	}

	const FVector Origin = Preview.Points[0];
	const int32 LastPoint = Preview.Points.Num() - 1;
	for (int32 PointIndex = 0; PointIndex <= LastPoint; ++PointIndex)
	{
		const float Alpha = static_cast<float>(PointIndex) / static_cast<float>(LastPoint);
		Preview.Points[PointIndex] = FMath::Lerp(Origin, HitPoint, Alpha);
	}
	return Preview;
}
