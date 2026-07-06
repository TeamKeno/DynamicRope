// Copyright Epic Games, Inc. All Rights Reserved.

#include "Gameplay/RopeWielderComponent.h"
#include "RopeComponent.h"
#include "DynamicRopeLog.h"
#include "Render/RopePreviewComponent.h"

#include "Components/SceneComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "Animation/AnimInstance.h"
#include "Animation/AnimMontage.h"

#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"

URopeWielderComponent::URopeWielderComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = false;
}

void URopeWielderComponent::BeginPlay()
{
	Super::BeginPlay();

	ResolveRefs();
	ResolvePreviewComponent();

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
		AddMappingContext();
		BindInput();
	}

	bShowThrowPreviewArc = PreviewComponent != nullptr;
	SetComponentTickEnabled(bShowThrowPreviewArc);
	if (bShowThrowPreviewArc)
	{
		UpdateThrowPreview();
	}
}

void URopeWielderComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// 입력 바인딩/매핑 정리(컴포넌트 파괴 후 댕글링 델리게이트 방지).
	if (APawn* Pawn = Cast<APawn>(GetOwner()))
	{
		if (UEnhancedInputComponent* EIC = Cast<UEnhancedInputComponent>(Pawn->InputComponent))
		{
			EIC->ClearBindingsForObject(this);
		}
		if (MappingContext)
		{
			if (APlayerController* PC = Cast<APlayerController>(Pawn->GetController()))
			{
				if (ULocalPlayer* LP = PC->GetLocalPlayer())
				{
					if (UEnhancedInputLocalPlayerSubsystem* Sub = LP->GetSubsystem<UEnhancedInputLocalPlayerSubsystem>())
					{
						Sub->RemoveMappingContext(MappingContext);
					}
				}
			}
		}
	}
	bInputBound = false;
	ClearThrowPreview();

	Super::EndPlay(EndPlayReason);
}

void URopeWielderComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	UpdateThrowPreview();
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

void URopeWielderComponent::ResolvePreviewComponent()
{
	AActor* Owner = GetOwner();
	if (!Owner || PreviewComponent)
	{
		return;
	}

	if (UActorComponent* ReferencedComponent = PreviewComponentReference.GetComponent(Owner))
	{
		PreviewComponent = Cast<URopePreviewComponent>(ReferencedComponent);
	}

	if (!PreviewComponent)
	{
		PreviewComponent = Owner->FindComponentByClass<URopePreviewComponent>();
	}
	if (!PreviewComponent && bAutoCreatePreviewComponent)
	{
		PreviewComponent = NewObject<URopePreviewComponent>(Owner, TEXT("RopePreviewComponent"));
		Owner->AddInstanceComponent(PreviewComponent);
		if (USceneComponent* RootComponent = Owner->GetRootComponent())
		{
			PreviewComponent->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepWorldTransform);
		}
		PreviewComponent->RegisterComponent();
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
	}
}

void URopeWielderComponent::BindInput()
{
	if (bInputBound)
	{
		return;
	}
	APawn* Pawn = Cast<APawn>(GetOwner());
	if (!Pawn)
	{
		return; // 입력은 Pawn 전용.
	}
	UEnhancedInputComponent* EIC = Cast<UEnhancedInputComponent>(Pawn->InputComponent);
	if (!EIC)
	{
		// 아직 빙의/입력 셋업 전일 수 있다 — Pawn의 SetupPlayerInputComponent에서 BindInput()을 호출하면 된다.
		UE_LOG(LogDynamicRope, Verbose, TEXT("RopeWielder on %s: EnhancedInputComponent not ready — call BindInput() from SetupPlayerInputComponent."),
			*GetNameSafe(GetOwner()));
		return;
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
		// 홀드 시맨틱: 누르면 시작, 떼거나(Completed) 중단되면(Canceled) 정지.
		EIC->BindAction(PullAction, ETriggerEvent::Started,   this, &URopeWielderComponent::OnPullInputStarted);
		EIC->BindAction(PullAction, ETriggerEvent::Completed, this, &URopeWielderComponent::OnPullInputCompleted);
		EIC->BindAction(PullAction, ETriggerEvent::Canceled,  this, &URopeWielderComponent::OnPullInputCompleted);
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
	bInputBound = true;
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
	if (Rope)
	{
		Rope->SetActivePull(PullForce);
	}
}

void URopeWielderComponent::StopPull()
{
	if (Rope)
	{
		Rope->SetActivePull(0.0f);
	}
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
		Rope->SetReelRate(ReelSpeed);
	}
}

void URopeWielderComponent::StartReelOut()
{
	if (Rope)
	{
		Rope->SetReelRate(-ReelSpeed);
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

void URopeWielderComponent::OnPullInputStarted()
{
	StartPull();
}

void URopeWielderComponent::OnPullInputCompleted()
{
	StopPull();
}

FVector URopeWielderComponent::GetAimDirection() const
{
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return FVector::ForwardVector;
	}

	switch (AimSource)
	{
	case ERopeAimSource::ActorForward:
		return Owner->GetActorForwardVector();

	case ERopeAimSource::CameraForward:
		if (const UCameraComponent* Cam = Owner->FindComponentByClass<UCameraComponent>())
		{
			return Cam->GetForwardVector();
		}
		// 카메라 없으면 ControlRotation 폴백.
		[[fallthrough]];

	case ERopeAimSource::ControlRotation:
	default:
		if (const APawn* Pawn = Cast<APawn>(Owner))
		{
			if (Pawn->Controller)
			{
				return Pawn->GetControlRotation().Vector();
			}
		}
		return Owner->GetActorForwardVector(); // 컨트롤러 없으면 액터 forward.
	}
}

FRopeThrowContext URopeWielderComponent::BuildThrowContext(const FVector& /*AimDir*/) const
{
	FRopeThrowContext Context;

	const AActor* Owner = GetOwner();

	Context.Origin = Rope ? Rope->GetComponentLocation() : (Owner ? Owner->GetActorLocation() : FVector::ZeroVector);
	Context.FrameForward = Owner ? Owner->GetActorForwardVector() : FVector::ForwardVector;
	Context.FrameUp = Owner ? Owner->GetActorUpVector() : FVector::UpVector;
	Context.FrameRight = Owner ? Owner->GetActorRightVector() : FVector::RightVector;
	Context.OwnerVelocity = Owner ? Owner->GetVelocity() : FVector::ZeroVector;
	Context.SocketVelocity = Context.OwnerVelocity;
	Context.FrameMode = ThrowFrameMode;
	Context.SwingPlane = SwingPlane;
	Context.CustomSwingPlaneNormal = CustomSwingPlaneNormal;
	Context.ThrowSpeed = ThrowSpeed;

	if (AttachMesh)
	{
		Context.Origin = AttachMesh->GetSocketLocation(HandSocketName);
		Context.SocketVelocity = AttachMesh->GetPhysicsLinearVelocity(HandSocketName);
	}

	switch (ThrowFrameMode)
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
		Context.FrameForward = CustomFrameForward;
		Context.FrameUp = CustomFrameUp;
		Context.FrameRight = CustomFrameRight;
		break;

	case ERopeThrowFrameMode::World:
	default:
		Context.FrameForward = FVector::ForwardVector;
		Context.FrameUp = FVector::UpVector;
		Context.FrameRight = FVector::RightVector;
		break;
	}

	// AimDir는 legacy 입력값으로만 남긴다. 실제 던지는 방향은 선택한 frame의 forward다.
	Context.AimDirection = Context.FrameForward;

	return Context;
}

void URopeWielderComponent::Throw()
{
	if (ThrowMontage)
	{
		PlayThrowMontage(); // 실제 던지기는 몽타주의 UAnimNotify_RopeThrow → ThrowNow().
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
		ClearThrowPreview();
		Rope->ThrowWithContext(BuildThrowContext(AimDir));
	}
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
	if (Phase == ERopePhase::Wrapped || Phase == ERopePhase::Contacting)
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
	bShowThrowPreviewArc = bEnabled;
	if (bShowThrowPreviewArc)
	{
		ResolveRefs();
		ResolvePreviewComponent();
		PreviewUpdateCooldown = 0.0f;
		SetComponentTickEnabled(true);
		UpdateThrowPreview();
	}
	else
	{
		ClearThrowPreview();
		SetComponentTickEnabled(false);
	}
}

void URopeWielderComponent::UpdateThrowPreview()
{
	if (!bShowThrowPreviewArc)
	{
		ClearThrowPreview();
		return;
	}

	if (!Rope)
	{
		ResolveRefs();
	}
	if (!PreviewComponent)
	{
		ResolvePreviewComponent();
	}
	if (!Rope || !PreviewComponent)
	{
		ClearThrowPreview();
		return;
	}

	FRopeWrapPreviewData Preview;
	if (!Rope->BuildWrappingPreview(BuildThrowContext(FVector::ZeroVector),
		PreviewReachScale, PreviewSegmentCount, PreviewSampleStep, PreviewQueryRadius, Preview))
	{
		ClearThrowPreview();
		return;
	}

	bLastPreviewBlocked = false;
	LastPreviewHitPoint = FVector::ZeroVector;
	PreviewComponent->SetWrapPreviewWorld(Preview);
}

void URopeWielderComponent::ClearThrowPreview()
{
	bLastPreviewBlocked = false;
	LastPreviewHitPoint = FVector::ZeroVector;
	PreviewUpdateCooldown = 0.0f;
	if (PreviewComponent)
	{
		PreviewComponent->ClearPreview();
	}
}
