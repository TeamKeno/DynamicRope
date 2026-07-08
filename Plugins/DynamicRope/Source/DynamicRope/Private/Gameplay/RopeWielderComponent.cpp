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

	// PreviewPathLocked는 preview 성공 여부가 던지기 가능 여부 자체를 결정한다.
	// 수동으로 배치한 PreviewComponent가 없으면 BeginPlay에서 런타임 컴포넌트를 만들어 preview tick을 보장한다.
	ResolvePreviewComponent(/*bAllowAutoCreate*/ ThrowMode == ERopeWielderThrowMode::PreviewPathLocked);

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

	bShowThrowPreview = PreviewComponent != nullptr;
	SetComponentTickEnabled(bShowThrowPreview);
	if (bShowThrowPreview)
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

void URopeWielderComponent::ResolvePreviewComponent(bool bAllowAutoCreate)
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

	if (!PreviewComponent && bAllowAutoCreate)
	{
		// 자동 생성은 PreviewPathLocked처럼 preview가 필수인 모드에서만 허용한다.
		// 이미 레벨/BP에 배치된 PreviewComponent가 있으면 그 설정을 우선 사용하고 여기로 오지 않는다.
		const FName PreviewName = MakeUniqueObjectName(Owner, URopePreviewComponent::StaticClass(), TEXT("RopePreviewComponent"));
		PreviewComponent = NewObject<URopePreviewComponent>(Owner, URopePreviewComponent::StaticClass(), PreviewName);
		if (PreviewComponent)
		{
			Owner->AddInstanceComponent(PreviewComponent);
			if (USceneComponent* Root = Owner->GetRootComponent())
			{
				PreviewComponent->SetupAttachment(Root);
			}
			PreviewComponent->RegisterComponent();
			UE_LOG(LogDynamicRope, Log, TEXT("RopeWielder on %s: auto-created RopePreviewComponent for PreviewPathLocked mode."),
				*GetNameSafe(Owner));
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
	if (ThrowMode == ERopeWielderThrowMode::PreviewPathLocked)
	{
		// Locked 모드는 "보이는 preview대로만 던진다"가 계약이다.
		// 따라서 마지막 prepared preview가 없으면 물리 throw로 fallback하지 않고 입력을 버린다.
		if (!LastPreparedPreview.IsValid())
		{
			UE_LOG(LogDynamicRope, Log, TEXT("RopeWielder on %s: preview path locked throw rejected (no valid prepared preview)."),
				*GetNameSafe(GetOwner()));
			return;
		}

		// 몽타주가 있으면 손을 놓는 AnimNotify까지 시간이 지나므로, 입력 순간 플레이어가 본 preview를 보존한다.
		// notify 시점에 새로 build하면 손/카메라/타겟 포즈 변화로 결과가 달라질 수 있다.
		PendingPreparedThrow = LastPreparedPreview;
		HeldPreparedPreview = LastPreparedPreview.RenderPreview;
		if (ThrowMontage)
		{
			PlayThrowMontage();
		}
		else
		{
			ThrowNow();
		}
		return;
	}

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
		if (ThrowMode == ERopeWielderThrowMode::PreviewPathLocked)
		{
			// ThrowNow는 즉시 throw와 AnimNotify throw가 모두 들어오는 실제 실행 지점이다.
			// 몽타주 경로에서는 PendingPreparedThrow를 우선 소비하고, 즉시 throw에서는 LastPreparedPreview를 쓴다.
			const FRopePreparedThrowPreview Prepared = PendingPreparedThrow.IsValid()
				? PendingPreparedThrow
				: LastPreparedPreview;
			if (!Prepared.IsValid())
			{
				UE_LOG(LogDynamicRope, Log, TEXT("RopeWielder on %s: prepared throw ignored (preview is not valid)."),
					*GetNameSafe(GetOwner()));
				return;
			}

			HeldPreparedPreview = Prepared.RenderPreview;
			if (PreviewComponent && HeldPreparedPreview.IsValid())
			{
				PreviewComponent->SetWrapPreviewWorld(HeldPreparedPreview);
			}
			PendingPreparedThrow.Reset();
			LastPreparedPreview.Reset();
			if (!Rope->ThrowWithPreparedPreview(Prepared))
			{
				ClearThrowPreview();
				UE_LOG(LogDynamicRope, Warning, TEXT("RopeWielder on %s: Rope rejected prepared preview throw."),
					*GetNameSafe(GetOwner()));
			}
			return;
		}

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

bool URopeWielderComponent::ShouldHoldPreparedPreview()
{
	if (ThrowMode != ERopeWielderThrowMode::PreviewPathLocked ||
		!PendingPreparedThrow.IsValid() ||
		!ThrowMontage)
	{
		return false;
	}

	if (!AttachMesh)
	{
		ResolveRefs();
	}

	const UAnimInstance* Anim = AttachMesh ? AttachMesh->GetAnimInstance() : nullptr;
	if (Anim && Anim->Montage_IsPlaying(ThrowMontage))
	{
		if (PreviewComponent && HeldPreparedPreview.IsValid())
		{
			PreviewComponent->SetWrapPreviewWorld(HeldPreparedPreview);
		}
		return true;
	}

	PendingPreparedThrow.Reset();
	return false;
}

void URopeWielderComponent::UpdateThrowPreview()
{
	if (!bShowThrowPreview)
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
		ResolvePreviewComponent(/*bAllowAutoCreate*/ false);
	}
	if (!Rope || !PreviewComponent)
	{
		ClearThrowPreview();
		return;
	}
	if (ShouldHoldPreparedPreview())
	{
		return;
	}

	FRopeWrapPreviewData Preview;
	FString PreviewBuildReason;
	const FRopeThrowContext ThrowContext = BuildThrowContext(FVector::ZeroVector);
	const ERopePhase RopePhase = Rope->GetPhase();
	if (ThrowMode == ERopeWielderThrowMode::PreviewPathLocked &&
		RopePhase == ERopePhase::GuidedThrow &&
		HeldPreparedPreview.IsValid())
	{
		PreviewComponent->SetWrapPreviewWorld(HeldPreparedPreview);
		return;
	}
	FRopePreparedThrowPreview Prepared;
	const bool bShouldBuildPrepared = ThrowMode == ERopeWielderThrowMode::PreviewPathLocked &&
		(RopePhase == ERopePhase::Free || RopePhase == ERopePhase::Releasing);

	// PreviewPathLocked의 Free/Releasing preview는 렌더용 centerline뿐 아니라 실제 throw에 쓸 contact/anchor까지 만든다.
	// 그 외 모드/phase에서는 기존처럼 표시용 preview만 만든다.
	const bool bBuiltPreview = bShouldBuildPrepared
		? Rope->BuildPreparedWrappingPreview(ThrowContext,
			PreviewComponent->PreviewReachScale, PreviewComponent->PreviewSegmentCount,
			PreviewComponent->PreviewSampleStep, PreviewComponent->PreviewQueryRadius, Prepared, &PreviewBuildReason)
		: Rope->BuildWrappingPreview(ThrowContext,
			PreviewComponent->PreviewReachScale, PreviewComponent->PreviewSegmentCount,
			PreviewComponent->PreviewSampleStep, PreviewComponent->PreviewQueryRadius, Preview, &PreviewBuildReason);
	if (!bBuiltPreview)
	{
		LogPreviewBuildResult(false, PreviewBuildReason.IsEmpty()
			? TEXT("preview build failed without a specific reason") : PreviewBuildReason);
		ClearThrowPreview();
		return;
	}
	if (bShouldBuildPrepared)
	{
		LastPreparedPreview = Prepared;
		Preview = Prepared.RenderPreview;
	}
	else
	{
		LastPreparedPreview.Reset();
	}

	LogPreviewBuildResult(true, FString::Printf(TEXT("preview built (points=%d, radius=%.2f, sides=%d)"),
		Preview.Points.Num(), Preview.Radius, Preview.NumSides));
	bLastPreviewBlocked = false;
	LastPreviewHitPoint = FVector::ZeroVector;
	PreviewComponent->SetWrapPreviewWorld(Preview);
	if (bShouldBuildPrepared)
	{
		HeldPreparedPreview = Preview;
	}
}

void URopeWielderComponent::ClearThrowPreview()
{
	bLastPreviewBlocked = false;
	LastPreviewHitPoint = FVector::ZeroVector;
	LastPreparedPreview.Reset();
	HeldPreparedPreview = FRopeWrapPreviewData();
	PreviewUpdateCooldown = 0.0f;
	if (PreviewComponent)
	{
		PreviewComponent->ClearPreview();
	}
}

void URopeWielderComponent::LogPreviewBuildResult(bool bSucceeded, const FString& Reason)
{
	const bool bChanged = !bHasLastPreviewBuildResult ||
		bLastPreviewBuildSucceeded != bSucceeded ||
		LastPreviewBuildReason != Reason;
	if (!bLogPreviewBuildAttempts && !bChanged)
	{
		return;
	}

	UE_LOG(LogDynamicRope, Log, TEXT("Rope preview %s on %s: %s"),
		bSucceeded ? TEXT("succeeded") : TEXT("failed"),
		*GetNameSafe(GetOwner()),
		*Reason);

	bHasLastPreviewBuildResult = true;
	bLastPreviewBuildSucceeded = bSucceeded;
	LastPreviewBuildReason = Reason;
}
