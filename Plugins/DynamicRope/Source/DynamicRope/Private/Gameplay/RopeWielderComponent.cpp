// Copyright Epic Games, Inc. All Rights Reserved.

#include "Gameplay/RopeWielderComponent.h"
#include "RopeComponent.h"
// ResolveBindingWorld — 조준 HUD 샘플의 본 위치(링 중심) 해석.
#include "Core/RopeWrapTarget.h"
#include "DynamicRopeLog.h"
#include "Render/RopePreviewComponent.h"
// 조준 HUD 위젯(생성은 프로젝트 세팅의 클래스, 수명은 이 컴포넌트가 관리).
#include "Settings/DynamicRopeSettings.h"
#include "UI/RopeAimWidget.h"
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
	ResolvePreviewComponent(/*bAllowAutoCreate*/ UsesLockedPreview());

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
	// preview 외에 지상 이탈/스윙 에어컨트롤 감시도 틱이 필요하다 — 전부 꺼져야 틱 정지.
	// Aim ray 모드는 preview component가 없어도 collider 수집 bounds를 매 프레임 갱신해야 한다.
	SetComponentTickEnabled(bShowThrowPreview || bAutoGroundExitOnUpwardPull || bBoostAirControlWhileSwinging ||
		UsesAimRay());
	UpdateAimRayColliderQueryBounds();
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
	if (AimHudWidget)
	{
		AimHudWidget->RemoveFromParent();
		AimHudWidget = nullptr;
	}
	if (Rope)
	{
		// Wielder가 사라진 뒤에도 로프의 collider 수집 범위가 조준 ray 방향으로 남지 않게 정리한다.
		Rope->ClearAimRayColliderQueryBounds();
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
	UpdateAimRayColliderQueryBounds();
	UpdateAimHudSample();
	UpdateAimHudWidget();
	UpdateThrowPreview();
}

void URopeWielderComponent::UpdateAimHudSample()
{
	const bool bHadTarget = AimHudSample.bHasTarget;
	USceneComponent* PrevMesh = AimHudSample.Mesh;
	const FName PrevBone = AimHudSample.Bone;

	AimHudSample = FRopeAimHudSample();
	if (UsesAimRay() && Rope)
	{
		const FRopeAimRayThrowRequest Request = BuildAimRayThrowRequest(FVector::ZeroVector);
		const FVector RayDirection = Request.RayDirection.GetSafeNormal();
		AimHudSample.RayOrigin = Request.RayOrigin;
		AimHudSample.RayDirection = RayDirection;
		AimHudSample.RayLength = Request.RayLength;
		AimHudSample.AimWorldPos = Request.RayOrigin + RayDirection * Request.RayLength;
		FRopeAimRayHitResult Hit;
		FRopeAimRayHitResult Blocked;
		// HUD 전용 스윕 — 월드 디버그 캡슐은 그리지 않는다(HUD 자체가 시각화이고, bDrawAimRayDebug는
		// preview/throw 해석 경로에서 이미 그린다 — 중복 드로우 방지).
		const bool bHitTarget = Rope->FindAimRayBoneHit(Request.RayOrigin, Request.RayDirection, Request.RayLength,
			Request.QueryRadius, Request.SweepStep, /*bDrawDebug*/ false, Hit, &Blocked);
		if (bHitTarget && Hit.bHit)
		{
			AimHudSample.bHasTarget = true;
			AimHudSample.Bone = Hit.Bone;
			// 샘플은 읽기 전용 계약(헤더 주석) — BP 노출을 위해 non-const로 보관만 한다.
			AimHudSample.Mesh = const_cast<USceneComponent*>(Hit.Mesh);
			AimHudSample.TargetWorldPos = ResolveBindingWorld(Hit.Mesh, Hit.Bone).GetLocation();
			AimHudSample.HitWorldPos = Hit.HitWorldPos;
			AimHudSample.TargetRadius = Hit.TargetBoundsRadius;
			AimHudSample.Distance = Hit.Distance;
			AimHudSample.AimWorldPos = Hit.HitWorldPos;
		}
		else if (Blocked.bHit)
		{
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

bool URopeWielderComponent::IsWielderTetherActive() const
{
	if (!Rope || Rope->GetPhase() != ERopePhase::Wrapped)
	{
		return false;
	}
	// wielder가 실제로 테더 몫을 받을 때만(테더 자체가 꺼졌거나 이번 프레임 유효 몫이 전량 대상이면 무의미).
	// 유효 대상 몫은 자동(질량 기반)/수동 공통 최종값이라 auto·override 모두에서 일관되게 판정된다.
	if (Rope->HoldConfig.TetherResponse <= 0.0f || Rope->GetEffectiveTetherTargetShare() >= 1.0f - KINDA_SMALL_NUMBER)
	{
		return false;
	}
	// 셀프랩(자기 자신에 감김)은 UpdateTether가 wielder 몫을 주지 않는다.
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
	if (Rope->GetTetherOvershoot() < GroundExitMinOvershoot)
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

bool URopeWielderComponent::UsesLockedPreview() const
{
	return Rope && Rope->ResolveMode == ERopeWrapResolveMode::GuaranteedWrap;
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
		// 입력은 Pawn 전용.
		return;
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
	if (ReloadAction)
	{
		// 장전은 단발(누름) — ③ 로프를 던지기 준비(Reel) 상태로 전환.
		EIC->BindAction(ReloadAction, ETriggerEvent::Started, this, &URopeWielderComponent::OnReloadInput);
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
		Rope->SetActivePull(Rope->HoldConfig.PullForce);
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
		// 컨트롤러 없으면 액터 forward.
		return Owner->GetActorForwardVector();
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
		if (AimSource == ERopeAimSource::CameraForward)
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

float URopeWielderComponent::GetAimRayLength() const
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
		Context.FrameForward = ExplicitAimDir;
	}

	return Context;
}

FRopeThrowContext URopeWielderComponent::BuildThrowContextInternal(const FVector& AimDir) const
{
	if (!UsesAimRay())
	{
		return BuildBaseThrowContext(AimDir);
	}

	const FRopeAimRayThrowRequest Request = BuildAimRayThrowRequest(AimDir);
	FRopeThrowContext Context = Request.BaseContext;
	if (Rope)
	{
		// Preview context는 현재 frame snapshot으로 즉시 해석한다. 실제 throw는 QueueAimRayThrow 경로를 쓴다.
		Rope->SetAimRayColliderQueryBounds(
			Request.RayOrigin, Request.RayDirection, Request.RayLength, Request.QueryRadius);
		Rope->ResolveAimRayThrowContext(Request, Context);
	}
	return Context;
}

FRopeAimRayThrowRequest URopeWielderComponent::BuildAimRayThrowRequest(const FVector& AimDir) const
{
	FRopeAimRayThrowRequest Request;
	Request.BaseContext = BuildBaseThrowContext(AimDir);
	Request.RayOrigin = GetAimRayOrigin();
	Request.RayDirection = Request.BaseContext.FrameForward;
	Request.RayLength = GetAimRayLength();
	Request.QueryRadius = AimRayQueryRadius;
	Request.SweepStep = AimRaySweepStep;
	Request.bDrawDebug = bDrawAimRayDebug;
	return Request;
}

void URopeWielderComponent::UpdateAimRayColliderQueryBounds()
{
	if (!Rope)
	{
		ResolveRefs();
	}
	if (!Rope)
	{
		return;
	}

	if (!UsesAimRay())
	{
		// 런타임 모드 변경 시 이전 ray AABB가 collider 수집 범위에 남지 않게 즉시 제거한다.
		Rope->ClearAimRayColliderQueryBounds();
		return;
	}

	// 실제 SDF query 없이 입력 값과 동일한 request를 만들어 다음 subsystem 수집 범위만 갱신한다.
	const FRopeAimRayThrowRequest Request = BuildAimRayThrowRequest(FVector::ZeroVector);
	Rope->SetAimRayColliderQueryBounds(
		Request.RayOrigin, Request.RayDirection, Request.RayLength, Request.QueryRadius);
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
		// ③: 유효한 prepared preview가 있으면(대상 조준 성공) 그 경로대로 무조건 꽂고(GuidedThrow),
		// 없으면(허공/사거리 밖) ThrowInDirection이 물리 탄도 투척으로 폴백한다 — 던지기 입력을 버리지 않는다
		// (2026-07-14 보장 재정의: 보장은 '조준한 대상'에 대한 것).
		if (LastPreparedPreview.IsValid())
		{
			// 몽타주가 있으면 손을 놓는 AnimNotify까지 시간이 지나므로, 입력 순간 플레이어가 본 preview를 보존한다.
			// notify 시점에 새로 build하면 손/카메라/타겟 포즈 변화로 결과가 달라질 수 있다.
			PendingPreparedThrow = LastPreparedPreview;
			HeldPreparedPreview = ResolvePreparedPreviewForDisplay(LastPreparedPreview);
			HeldPreviewExpireTimeSeconds = 0.0f;
		}
		else
		{
			PendingPreparedThrow.Reset();
		}
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
			// ThrowNow는 즉시 throw와 AnimNotify throw가 모두 들어오는 실제 실행 지점이다.
			// 몽타주 경로에서는 PendingPreparedThrow를 우선 소비하고, 즉시 throw에서는 LastPreparedPreview를 쓴다.
			const FRopePreparedThrowPreview Prepared = PendingPreparedThrow.IsValid()
				? PendingPreparedThrow
				: LastPreparedPreview;
			if (!Prepared.IsValid())
			{
				// 허공(대상 없음/사거리 밖): 거부 대신 물리 탄도 투척으로 폴백한다(2026-07-14 보장 재정의).
				// 로프의 ThrowWithContext ③ 경로가 Reel 게이트 + preview 재빌드 실패 → 탄도 Flight로 마무리한다
				// (③ Flight는 캡처 안 함 → 안 꽂히고 Free). 던지기 방향은 조준 컨텍스트(BuildThrowContext)로 해석.
				ClearThrowPreview();
				PendingPreparedThrow.Reset();
				LastPreparedPreview.Reset();
				Rope->ThrowWithContext(BuildThrowContext(AimDir));
				NotifyThrown();
				OnThrown.Broadcast();
				return;
			}

			HeldPreparedPreview = ResolvePreparedPreviewForDisplay(Prepared);
			HeldPreviewExpireTimeSeconds = 0.0f;
			if (PreviewComponent && HeldPreparedPreview.IsValid())
			{
				PreviewComponent->SetWrapPreviewWorld(HeldPreparedPreview);
			}
			PendingPreparedThrow.Reset();
			LastPreparedPreview.Reset();
			if (!Rope->ThrowWithPreparedPreview(Prepared))
			{
				ClearThrowPreview();
				NotifyThrowRejected(ERopeThrowRejectReason::RopeRejected);
				OnThrowRejected.Broadcast(ERopeThrowRejectReason::RopeRejected);
				return;
			}
			NotifyThrown();
			OnThrown.Broadcast();
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
	// 기존 계약대로 Rope가 Flight에 진입한 뒤 성공 알림을 보낸다.
	NotifyThrown();
	OnThrown.Broadcast();
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
		SetComponentTickEnabled(bAutoGroundExitOnUpwardPull || bBoostAirControlWhileSwinging);
	}
}

bool URopeWielderComponent::ShouldHoldPreparedPreview()
{
	if (!UsesLockedPreview() ||
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
		HeldPreparedPreview = ResolvePreparedPreviewForDisplay(PendingPreparedThrow);
		if (PreviewComponent && HeldPreparedPreview.IsValid())
		{
			PreviewComponent->SetWrapPreviewWorld(HeldPreparedPreview);
		}
		return true;
	}

	PendingPreparedThrow.Reset();
	return false;
}

bool URopeWielderComponent::ShouldUpdateThrowPreviewForPhase(ERopePhase Phase) const
{
	// PreviewPathLocked는 "던지기 전 성공한 preview path"만 새로 만든다.
	// GuidedThrow/Wrapped에서는 이미 확정된 HeldPreparedPreview를 사용하므로 build를 다시 시도하지 않는다.
	if (UsesLockedPreview())
	{
		// ③(Guaranteed)는 Reel(장전 준비) 상태에서만 조준 preview를 만든다 — Reel에서만 던질 수 있으므로.
		// Free(release 후 늘어진 상태)에서는 장전 전이라 preview를 보이지 않는다.
		return Phase == ERopePhase::Reel;
	}

	// 일반 preview도 idle 전용 설정이면 조준 전 상태에서만 계산한다.
	if (bPreviewOnlyWhenIdle)
	{
		return Phase == ERopePhase::Free || Phase == ERopePhase::Releasing;
	}

	// idle 전용이 아니면 실제 접촉/감김 진행 중 표시용 preview까지 허용한다.
	return Phase == ERopePhase::Free ||
		Phase == ERopePhase::Releasing ||
		Phase == ERopePhase::Flight ||
		Phase == ERopePhase::Contacting ||
		Phase == ERopePhase::Wrapping;
}

bool URopeWielderComponent::UpdateHeldPreparedPreviewForPhase(ERopePhase Phase)
{
	if (!UsesLockedPreview() || !HeldPreparedPreview.IsValid())
	{
		return false;
	}

	if (Phase == ERopePhase::GuidedThrow)
	{
		// GuidedThrow는 cached preview path를 authoritative하게 따라가는 상태다.
		// 새 path를 build하지 않고, 플레이어가 보고 확정한 path를 그대로 렌더 유지한다.
		if (PreviewComponent)
		{
			PreviewComponent->SetWrapPreviewWorld(HeldPreparedPreview);
		}
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
			if (PreviewComponent)
			{
				PreviewComponent->SetWrapPreviewWorld(HeldPreparedPreview);
			}
			LastPreviewPhase = Phase;
			return true;
		}

		ClearThrowPreview();
		LastPreviewPhase = Phase;
		return true;
	}

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

	FString PreviewBuildReason;
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

	const FRopeThrowContext ThrowContext = BuildThrowContext(FVector::ZeroVector);
	// ③ prepared preview(contact/anchor 포함 — 실제 throw에 쓰임)는 Reel(장전 준비)에서만 만들어 LastPreparedPreview에 보관한다.
	const bool bShouldBuildPrepared = UsesLockedPreview() && RopePhase == ERopePhase::Reel;

	// PreviewPathLocked의 Free/Releasing preview는 렌더용 centerline뿐 아니라 실제 throw에 쓸 contact/anchor까지 만든다.
	// 그 외 모드/phase에서는 기존처럼 표시용 preview만 만든다.
	// preview 모드별 build와 prepared 결과 보관은 PreviewComponent가 일관되게 소유한다.
	const bool bBuiltPreview = PreviewComponent->UpdatePreviewFromRope(
		*Rope, ThrowContext, bShouldBuildPrepared, &PreviewBuildReason);
	if (!bBuiltPreview)
	{
		LogPreviewBuildResult(false, PreviewBuildReason.IsEmpty()
			? TEXT("preview build failed without a specific reason") : PreviewBuildReason);
		ClearThrowPreview();
		LastPreviewPhase = RopePhase;
		return;
	}
	if (bShouldBuildPrepared)
	{
		LastPreparedPreview = PreviewComponent->GetPreparedPreview();
		StoreAimGuideFrameIfNeeded(LastPreparedPreview);
	}
	else
	{
		LastPreparedPreview.Reset();
	}

	LogPreviewBuildResult(true, TEXT("preview built"));
	bLastPreviewBlocked = false;
	LastPreviewHitPoint = FVector::ZeroVector;
	if (bShouldBuildPrepared)
	{
		HeldPreparedPreview = ResolvePreparedPreviewForDisplay(LastPreparedPreview);
		HeldPreviewExpireTimeSeconds = 0.0f;
	}
	LastPreviewPhase = RopePhase;
}

void URopeWielderComponent::ClearThrowPreview()
{
	bLastPreviewBlocked = false;
	LastPreviewHitPoint = FVector::ZeroVector;
	LastPreparedPreview.Reset();
	HeldPreparedPreview = FRopeWrapPreviewData();
	HeldPreviewExpireTimeSeconds = 0.0f;
	PreviewUpdateCooldown = 0.0f;
	if (PreviewComponent)
	{
		PreviewComponent->ClearPreview();
	}
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

	// AimRayHitDirection은 소켓/로프 컴포넌트 로컬이 아니라 wielder owner 로컬 기준으로 path를 고정한다.
	Prepared.StoreGuideFrameLocal(OwnerRoot);
}

FRopeWrapPreviewData URopeWielderComponent::ResolvePreparedPreviewForDisplay(const FRopePreparedThrowPreview& Prepared) const
{
	// owner-local로 저장되지 않은 일반 preview는 원래 월드 점을 그대로 반환한다.
	return Prepared.ResolveRenderPreviewWorld();
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
