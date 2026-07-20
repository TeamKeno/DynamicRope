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
	if (Rope)
	{
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
	UpdatePullMontage();
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
		const FRopeAimRayThrowRequest Request = BuildAimRayThrowRequest(FVector::ZeroVector);
		Rope->RefreshAimRayQueryColliders(Request);
		FRopeThrowContext ResolvedContext = Request.BaseContext;
		const FVector RayDirection = Request.RayDirection.GetSafeNormal();
		AimHudSample.RayOrigin = Request.RayOrigin;
		AimHudSample.RayDirection = RayDirection;
		AimHudSample.RayLength = Request.RayLength;
		// 설정 반경이 0(기본)이면 로프/접촉 폴백이 걸린다 — 질의가 실제로 쓴 값을 그대로 담아야
		// HUD/디버거가 검사 두께를 정확히 그린다.
		AimHudSample.QueryRadius = Rope->GetAimRayEffectiveQueryRadius(Request.QueryRadius);
		AimHudSample.AimWorldPos = Request.RayOrigin + RayDirection * Request.RayLength;
		FRopeAimRayHitResult Hit;
		FRopeAimRayHitResult Blocked;
		// 이 샘플이 조준 시각화의 단일 소스다 — HUD 위젯과 Gameplay Debugger([J]aim)가 함께 읽는다.
		const bool bHitTarget = Rope->ResolveAimRayThrowContext(
			Request, ResolvedContext, &Hit, &Blocked);
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

		if (Request.IsValid())
		{
			AimRayFrameThrowContext = ResolvedContext;
			bHasAimRayFrameThrowContext = true;
		}
	}
	else if (Rope)
	{
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
	bPullHeld = true;
	if (PullMontage)
	{
		// 몽타주 모드: 힘은 window notify만 싣는다(비Wrapped 힘 장전 없음 — window 밖 pull이 새는 것을 막는다).
		// 재생 조건은 UpdatePullMontage가 굴린다 — 여기서 즉시 1회 돌려 Wrapped면 지연 없이 시작.
		UpdatePullMontage();
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
	bPullHeld = false;
	StopPullNow();
	// 몽타주 경로로 시작했다면 연출도 함께 끝낸다(입력을 뗀 순간). 다른 경로였어도 무해 — 재생 중일 때만 중단.
	if (PullMontage && AttachMesh)
	{
		if (UAnimInstance* Anim = AttachMesh->GetAnimInstance())
		{
			if (Anim->Montage_IsPlaying(PullMontage))
			{
				Anim->Montage_Stop(PullMontage->BlendOut.GetBlendTime(), PullMontage);
			}
		}
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
	FRopeThrowContext Context = Request.BaseContext;
	if (Rope)
	{
		// Preview context는 current ray bounds로 snapshot을 즉시 갱신한 뒤 해석한다.
		// 실제 throw는 QueueAimRayThrow 경로를 써서 SimTick의 중앙 수집 직후 확정한다.
		if (Request.IsValid())
		{
			Rope->RefreshAimRayQueryColliders(Request);
			Rope->ResolveAimRayThrowContext(Request, Context);
		}
		else
		{
			Rope->ClearAimRayColliderQueryBounds();
		}
	}
	return Context;
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
		// 없으면(허공/사거리 밖) ThrowInDirection이 레이 끝점을 향한 아치 던지기로 폴백한다 — 던지기 입력을
		// 버리지 않는다(2026-07-14 보장 재정의: 보장은 '조준한 대상'에 대한 것).
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
				ClearThrowPreview();
				PendingPreparedThrow.Reset();
				LastPreparedPreview.Reset();

				// 던질 수 없는 phase(③ 비-Reel)면 여기서 끝낸다. 로프의 게이트(ThrowWithContext)는 void라
				// 거절을 알릴 수 없어서, 이 검사가 없으면 아무 일도 안 한 던지기에 OnThrown이 발화한다
				// (몽타주 경로면 와인드업이 다 돌아간 뒤에).
				if (!Rope->CanThrowNow())
				{
					NotifyThrowRejected(ERopeThrowRejectReason::NotInReel);
					OnThrowRejected.Broadcast(ERopeThrowRejectReason::NotInReel);
					return;
				}

				// 허공(대상 없음/사거리 밖): 거부 대신 레이 끝점을 향한 아치 던지기로 폴백한다(2026-07-14 보장
				// 재정의). 로프의 ThrowWithContext ③ 경로가 preview 재빌드에 실패하면 손 원점 → 레이 끝점 직선을
				// 아치로 재생하고(StartFreeGuidedThrow — 조준 던지기와 같은 GuidedThrow, 대상/앵커만 없다),
				// 꽂을 대상이 없으므로 아치 완료 시 Free로 낙하한다. 방향은 조준 컨텍스트(BuildThrowContext)로 해석.
				Rope->ThrowWithContext(BuildThrowContext(AimDir));
				NotifyThrown();
				OnThrown.Broadcast();
				return;
			}

			HeldPreparedPreview = ResolvePreparedPreviewForDisplay(Prepared);
			HeldPreviewExpireTimeSeconds = 0.0f;
			// 프리뷰는 Reel(조준)에서만 보인다 — 발사 즉시 표시를 지운다(HeldPreparedPreview 데이터는
			// 유지: phase-gate 유효성 검사와 Wrapped-hold 옵션이 참조). 이후 GuidedThrow 분기가 계속
			// 지운 상태를 유지한다.
			ClearPreviewDisplay();
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

void URopeWielderComponent::UpdatePullMontage()
{
	if (!PullMontage || !AttachMesh)
	{
		return; // 몽타주 모드 아님(또는 메시 미해석 — 다음 틱에 재시도할 것 없이 no-op).
	}
	UAnimInstance* Anim = AttachMesh->GetAnimInstance();
	if (!Anim)
	{
		return;
	}

	const bool bPlaying = Anim->Montage_IsPlaying(PullMontage);
	const bool bWrapped = Rope && Rope->GetPhase() == ERopePhase::Wrapped;
	if (bPullHeld && bWrapped && !bPlaying)
	{
		// 홀드 중 재생 보장: 홀드 중 wrap 성립(그때 시작)과 비루프 몽타주의 자연 종료(반복 재생 =
		// 연속 당기기 사이클)를 조건 하나로 잇는다. 힘은 몽타주 안의 window notify가 싣는다.
		PlayPullMontage();
	}
	else if (bPlaying && !bWrapped)
	{
		// wrap이 풀리면(release/cut/대상 소실) 당기는 모션도 끝낸다 — 힘은 NotifyEnd 캐스케이드가 끈다.
		// 직접 재생(BP의 PlayPullMontage)도 같은 규칙: 감긴 게 없는 pull 모션은 두지 않는다.
		Anim->Montage_Stop(PullMontage->BlendOut.GetBlendTime(), PullMontage);
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
		// 던지기 입력 순간 PendingPreparedThrow에 경로를 고정한다(실제 던지기 정확도용) — 유지한다.
		// 다만 프리뷰는 Reel(조준)에서만 보이면 되므로, 윈드업 몽타주 재생 중에는 표시를 끈다.
		// 이전엔 고정 경로를 매 틱 그려, 윈드업 동안 캐릭터가 이동하면 지나간 자리에 프리뷰가 남았다.
		HeldPreparedPreview = ResolvePreparedPreviewForDisplay(PendingPreparedThrow);
		ClearPreviewDisplay();
		return true;
	}

	PendingPreparedThrow.Reset();
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
	if (!Rope || Rope->TipEngagement != ERopeTipEngagement::Pierce || !Preview.IsValid())
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
