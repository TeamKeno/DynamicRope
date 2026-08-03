// Copyright Epic Games, Inc. All Rights Reserved.

#include "Gameplay/RopeWielderComponent.h"
#include "RopeComponent.h"
#include "Materials/MaterialInstanceDynamic.h"
// ResolveBindingWorld, which resolves the bone position, that is the ring centre, of the aiming HUD sample.
#include "Core/RopeWrapTarget.h"
#include "DynamicRopeLog.h"
#include "Render/RopePreviewComponent.h"
// IsRagdolled, the owner state the automatic input suppression reads.
#include "Gameplay/RopeRagdollResponseComponent.h"
// The aiming HUD widget. Its class comes from the project settings and this component manages its lifetime.
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
#include "GameFramework/MovementComponent.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "Animation/AnimInstance.h"
#include "Animation/AnimMontage.h"

#include "Components/InputComponent.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "InputMappingContext.h"

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
	// Hard leash must run after Pawn movement but before Chaos/Rope PostPhysics.
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
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
		// Cover late and repeated possession: bind now if it is possible now, and let the possession and
		// restart hooks try again if it is not. Without that, a pawn spawned with no controller, a client
		// possessing late, or a repossession after unpossessing would all leave input permanently unbound.
		if (APawn* OwnerPawn = Cast<APawn>(GetOwner()))
		{
			OwnerPawn->ReceiveControllerChangedDelegate.AddDynamic(this, &URopeWielderComponent::HandlePawnControllerChanged);
			OwnerPawn->ReceiveRestartedDelegate.AddDynamic(this, &URopeWielderComponent::HandlePawnRestarted);
		}
		RefreshInputRegistration();
	}

	// The mode-derived state, meaning preview creation, tick activation and the aiming sample, is unified
	// in RefreshModeDerivedState, which applying a preset at runtime reuses through OnPresetApplied. In
	// the abnormal order where the rope is resolved after BeginPlay the subscription is missed, and game
	// code has to call Refresh by hand.
	if (Rope)
	{
		Rope->OnPresetApplied.AddUniqueDynamic(this, &URopeWielderComponent::HandleRopePresetApplied);
		Rope->OnRopePhaseChanged.AddUniqueDynamic(this, &URopeWielderComponent::HandleRopePhaseChanged);
	}
	RegisterMovementConstraintHooks();
	RefreshModeDerivedState();
}

void URopeWielderComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Clear the input bindings and mapping, which stops delegates dangling after the component is destroyed.
	if (APawn* Pawn = Cast<APawn>(GetOwner()))
	{
		Pawn->ReceiveControllerChangedDelegate.RemoveDynamic(this, &URopeWielderComponent::HandlePawnControllerChanged);
		Pawn->ReceiveRestartedDelegate.RemoveDynamic(this, &URopeWielderComponent::HandlePawnRestarted);
	}
	ClearBoundInput();
	RemoveMappingContext();
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
	// Make sure the rope's collider gather region does not stay widened along the aim ray after the
	// wielder is gone.
		Rope->ClearAimRayColliderQueryBounds();
		Rope->OnPresetApplied.RemoveDynamic(this, &URopeWielderComponent::HandleRopePresetApplied);
		Rope->OnRopePhaseChanged.RemoveDynamic(this, &URopeWielderComponent::HandleRopePhaseChanged);
	}
	UnregisterMovementConstraintHooks();

	// Prevent air control from being left boosted if the character is destroyed or the level changes
	// mid-swing.
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

	// Measure the hand socket's animation-relative velocity, which is carried as HandAnimationVelocity on
	// a throw.
	UpdateHandAnimVelocity(DeltaTime);

	// Ahead of UpdatePullEngage, so an armed pull cannot engage and play its montage on the very frame
	// suppression begins, and ahead of UpdateAimHudSample, so the aiming path sees the settled state.
	// Hand velocity sampling deliberately stays above this and keeps running while suppressed, so the
	// sample is not stale on recovery.
	UpdateInputSuppression();

	// This runs in the same PrePhysics tick, after the movement prerequisite. A delegate inside the
	// character movement component can fire while the skeletal mesh child update is still scoped or
	// deferred, which leaves the real hand socket position stale.
	// By this point the movement tick has fully finished, so the mesh and socket transforms and the final
	// root motion and slide results are all settled.
	if (Cast<APawn>(GetOwner()))
	{
		EnforceWielderLengthConstraint(DeltaTime);
	}
	UpdateGroundExit();
	UpdateSwingAirControl();
	UpdatePullEngage();
	UpdatePullGlowMaterial();
	UpdatePullGaugeWidget();
	UpdateAimHudSample();
	UpdateAimHudWidget();
	UpdateThrowPreview();
}

void URopeWielderComponent::UpdateHandAnimVelocity(float DeltaTime)
{
	// Measure the hand socket's animation-relative velocity from its component-local position delta and
	// convert it to world space, excluding character movement.
	// That avoids the pitfalls of reading a physics linear velocity, which depends on a physics body and
	// is zero without one, along with its sign problem. It is carried as HandAnimationVelocity on a
	// throw, so swinging the arm while standing still is reflected.
	// Only an extreme hitch or a paused frame, below two frames per second, breaks the delta.
	// Reactivation and rig changes are handled by an explicit reseed instead, so a sustained low
	// framerate of 15 to 30 still uses the real delta time and the velocity does not vary with it.
	constexpr float MaxSampleDeltaTime = 0.5f;
	constexpr float MaxHandAnimSpeed = 2000.0f;  // cm/s. Caps an oversized change from a teleport or an animation reset, which would otherwise fling the rope.

	if (!AttachMesh || DeltaTime <= KINDA_SMALL_NUMBER || DeltaTime > MaxSampleDeltaTime)
	{
		ResetHandAnimVelocitySample();
		return;
	}

	// A changed rig, meaning the mesh or socket, puts the previous position in a different space and
	// makes the delta meaningless, so this frame only reseeds.
	const bool bRigChanged =
		PreviousHandSampleMesh.Get() != AttachMesh || PreviousHandSampleSocket != HandSocketName;

	const FVector SocketLocCS =
		AttachMesh->GetSocketTransform(HandSocketName, RTS_Component).GetLocation();
	MeasuredHandAnimVelocityWorld = (bHasHandSocketSample && !bRigChanged)
		? ComputeHandSwingVelocityWorld(PreviousHandSocketLocationCS, SocketLocCS, DeltaTime,
			AttachMesh->GetComponentTransform(), MaxHandAnimSpeed)
		: FVector::ZeroVector;
	PreviousHandSocketLocationCS = SocketLocCS;
	PreviousHandSampleMesh = AttachMesh;
	PreviousHandSampleSocket = HandSocketName;
	bHasHandSocketSample = true;
}

FVector URopeWielderComponent::ComputeHandSwingVelocityWorld(
	const FVector& PrevSocketCS, const FVector& CurSocketCS, float DeltaTime,
	const FTransform& ComponentXform, float MaxSpeed)
{
	if (DeltaTime <= KINDA_SMALL_NUMBER)
	{
		return FVector::ZeroVector;
	}
	const FVector RelVelCS = (CurSocketCS - PrevSocketCS) / DeltaTime;
	const FVector RelVelWorld = ComponentXform.TransformVectorNoScale(RelVelCS);
	return RelVelWorld.GetClampedToMaxSize(MaxSpeed);
}

void URopeWielderComponent::ResetHandAnimVelocitySample()
{
	bHasHandSocketSample = false;
	PreviousHandSocketLocationCS = FVector::ZeroVector;
	MeasuredHandAnimVelocityWorld = FVector::ZeroVector;
	PreviousHandSampleMesh = nullptr;
	PreviousHandSampleSocket = NAME_None;
}

void URopeWielderComponent::RefreshTickEnabled()
{
	// On an off-to-on reactivation, the delta against the previous, now stale, socket position would
	// produce a large spike, so the sample is broken and reseeded.
	const bool bWant = ComputeDesiredTickEnabled();
	if (bWant && !IsComponentTickEnabled())
	{
		ResetHandAnimVelocitySample();
	}
	SetComponentTickEnabled(bWant);
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
	// In a phase the rope cannot be thrown from, or while rope input is suppressed, the aiming sweep does
	// not run at all; an empty sample makes the widgets and the debugger hide themselves.
	// For GuaranteedWrap outside Loaded this sweep was the only remaining SDF cost, since
	// UpdateThrowPreview already builds no prepared preview there.
	if (IsAimActive())
	{
		const FRopeAimRayThrowRequest CurrentRequest = BuildAimRayThrowRequest(FVector::ZeroVector);
		// PrePhysics only registers the request. The rope resolves it right after the normal collider
		// gather in PostPhysics, and what is consumed here is the previous gather's result. That costs at
		// most one frame of latency, and the global provider gather stays at once per frame.
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
		// A configured radius of 0, the default, engages the rope or contact fallback, so recording the
		// value the query actually used is what lets the HUD and the debugger draw the real tested
		// thickness.
		AimHudSample.QueryRadius = Rope->GetAimRayEffectiveQueryRadius(DisplayRequest.QueryRadius);
		AimHudSample.AimWorldPos = DisplayRequest.RayOrigin + RayDirection * DisplayRequest.RayLength;
		// This sample is the single source for aiming visualization, read by both the HUD widget and the
		// Gameplay Debugger's aim view.
		if (bHasResolvedQuery && QueryResult.bHitTarget && QueryResult.Hit.bHit && QueryResult.Hit.Mesh)
		{
			const FRopeAimRayHitResult& Hit = QueryResult.Hit;
			const USceneComponent* HitMesh = Hit.Mesh;
			AimHudSample.bHasTarget = true;
			AimHudSample.Bone = Hit.Bone;
			// The sample is read-only by contract, as documented in the header; it is held non-const only
			// so it can be exposed to Blueprint.
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
			// The ray hit something that cannot be wrapped, which is shown as blocked. There may be no bone
			// binding, so the point that was hit is used as the ring centre.
			AimHudSample.bBlocked = true;
			AimHudSample.bBlockedByTarget = Blocked.bWrapCandidate;
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
		// In a phase that cannot aim, stop the previous ray bounds from continuing to widen the collider
		// gather region.
		Rope->ClearAimRayColliderQueryBounds();
	}

	// Report changes of the (mesh, bone) target: entering or switching fires Changed and leaving fires
	// Lost.
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

	// Create it only once the local player controller is ready, which covers late possession; before then
	// it is retried on the next tick.
	const APawn* Pawn = Cast<APawn>(GetOwner());
	APlayerController* PC = Pawn ? Cast<APlayerController>(Pawn->GetController()) : nullptr;
	if (!PC || !PC->IsLocalController())
	{
		return;
	}

	// The widget class has a single source in the project settings, defaulting to the C++ URopeAimWidget
	// and replaceable with a Blueprint. Leaving it empty means no HUD.
	// It is the size of a demo HUD, so a synchronous load is acceptable for the one-off.
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
	// The gauge is independent of the aiming mode, since a rope can be wrapped and pulled in
	// FullSimulation too, so it follows the toggle alone.
	// Before pull is armed the widget draws nothing, so it is not created and destroyed per state, which
	// would flicker.
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

	// Create it only once the local player controller is ready, which covers late possession; before then
	// it is retried on the next tick.
	const APawn* Pawn = Cast<APawn>(GetOwner());
	APlayerController* PC = Pawn ? Cast<APlayerController>(Pawn->GetController()) : nullptr;
	if (!PC || !PC->IsLocalController())
	{
		return;
	}

	// The widget class has a single source in the project settings, defaulting to the C++
	// URopePullGaugeWidget. Leaving it empty means no gauge.
	UClass* WidgetClass = UDynamicRopeSettings::Get()->PullGaugeWidgetClass.LoadSynchronous();
	if (!WidgetClass)
	{
		return;
	}
	PullGaugeWidget = CreateWidget<URopePullGaugeWidget>(PC, WidgetClass);
	if (PullGaugeWidget)
	{
		// The widget finds the wielder on its owning pawn by itself, but wiring it explicitly here is
		// deterministic.
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
	// Only while the wielder actually receives a tether share. If this frame's effective share, whether
	// the inverse-mass ratio of the lambda solve or the binary result of the drag test, is entirely the
	// target's, the wielder is a free end and swing and ground-exit responses are meaningless.
	if (Rope->GetEffectiveTetherTargetShare() >= 1.0f - KINDA_SMALL_NUMBER)
	{
		return false;
	}
	// A self-wrap, where the rope wraps its own owner, never gives the wielder a tether share, whatever
	// kind of component the target is.
	if (const USceneComponent* WrappedComponent = Rope->GetWrappedComponent())
	{
		if (WrappedComponent->GetOwner() == GetOwner())
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
	// The whole chain has to be taut for the tether to actually apply. An overshoot can exceed zero on a
	// slack chain through a sub-leg stretch, as with a moving anchor, so leaving the ground on that alone
	// would produce a fall with no traction behind it.
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

	// The traction direction, from the hand to the anchor, is the reverse of the pull sample's direction,
	// which follows the leg from the anchor to the hand. The ground exit only triggers with enough upward
	// component, since horizontal traction reads more naturally as being dragged while walking. Returning
	// to walking on landing is handled by the engine.
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

	// Swinging means airborne, receiving a wielder tether share, and the whole chain taut. When it ends,
	// on landing, on release, on going slack or on a settings change, the saved original value is
	// restored. Changing air control externally while the boost is active is overwritten on restore,
	// which is a demo-level limitation stated here as the contract.
	// The taut gate is the same test UpdateGroundExit uses: steering is only enabled while the tether is
	// genuinely pulling the wielder. On a slack chain there is no tether force, so raising air control
	// would not steer a swing, it would simply let the character float.
	const bool bSwinging = bBoostAirControlWhileSwinging && Movement->IsFalling()
		&& IsWielderTetherActive() && Rope->IsChainTaut();
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
	if (!RagdollResponse.IsValid())
	{
		// Resolved here rather than searched per frame: the suppression check only asks the cached
		// component whether it is ragdolled. An owner with no ragdoll response leaves this null, which is
		// the "never suppressed automatically" case.
		RagdollResponse = Owner->FindComponentByClass<URopeRagdollResponseComponent>();
	}
}

// Aiming and throwing behaviour are derived from the rope's ResolveMode. With no rope there is neither
// aim assistance nor a preview-locked throw, which behaves the same as FullSimulation.
bool URopeWielderComponent::UsesAimRay() const
{
	return Rope && Rope->ResolveMode != ERopeWrapResolveMode::FullSimulation;
}

bool URopeWielderComponent::IsAimActive() const
{
	// There is no reason to aim in a phase the rope cannot be thrown from. GuaranteedWrap is restricted
	// to Loaded, so the HUD is off in Free, Wrapped and elsewhere.
	// The rope owns the gate, through CanThrowNow: sharing the predicate with the throw entry point is
	// what keeps the HUD and actual throwability from diverging.
	// Suppressed input turns aiming off through this same call, which takes the HUD widget, the aim ray
	// sweep and the throw preview with it. UsesAimRay() stays first because it carries the null check
	// CanThrowNow() relies on.
	return UsesAimRay() && !IsRopeInputSuppressed() && Rope->CanThrowNow();
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
		// A convenience automatic creation, only for GuaranteedWrap when display is wanted, which is how
		// BeginPlay calls it. It is not required for gameplay: the prepared calculation belongs to the
		// rope and the wielder, so GuaranteedWrap throws correctly without this component.
		// A preview component already placed in the level or Blueprint takes priority and never reaches
		// here.
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
		// Unpossessed: the Completed and Canceled events of held inputs can no longer arrive, so any
		// continuous state is cleared first.
		StopPull();
		StopReel();
		// Detach the mapping context from that local player; the next possession attaches it to the new
		// one.
		// The bindings live on the input component and are left alone here, so returning to the same
		// component keeps them valid.
		RemoveMappingContext();
		return;
	}
	// The input component may not exist yet immediately after possession, in which case the restart hook
	// below completes the job.
	RefreshInputRegistration();
}

void URopeWielderComponent::HandlePawnRestarted(APawn* OwnerPawn)
{
	// This is after PawnClientRestart, and therefore after SetupPlayerInputComponent, so the input
	// component is ready.
	RefreshInputRegistration();
}

void URopeWielderComponent::RefreshInputRegistration()
{
	// A change of possession can also change which local player the mapping context belongs on, so it is
	// detached from the old one and attached to the new.
	RemoveMappingContext();
	AddMappingContext();
	BindInput();
}

void URopeWielderComponent::RemoveMappingContext()
{
	// The mapping context is registered on the local player rather than the pawn. If the pawn is
	// unpossessed and then destroyed, GetController() returns null, and removing it through the pawn's
	// controller would be skipped, leaving the context on the local player permanently. Removing it
	// through the subsystem cached when it was added works regardless of possession; a local player
	// already destroyed leaves the weak pointer null and nothing needs removing.
	if (UEnhancedInputLocalPlayerSubsystem* Sub = MappedInputSubsystem.Get())
	{
		if (UInputMappingContext* AddedContext = MappedInputContext.Get())
		{
			Sub->RemoveMappingContext(AddedContext);
		}
	}
	MappedInputSubsystem.Reset();
	MappedInputContext = nullptr;
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
		// Cached so that EndPlay removes exactly the pair that was registered, independently of possession
		// and of any later property change.
		MappedInputSubsystem = Sub;
		MappedInputContext = MappingContext;
	}
}

void URopeWielderComponent::ClearBoundInput()
{
	if (UEnhancedInputComponent* EIC = Cast<UEnhancedInputComponent>(BoundInputComponent.Get()))
	{
		EIC->ClearBindingsForObject(this);
	}
	BoundInputComponent.Reset();
}

void URopeWielderComponent::BindInput()
{
	APawn* Pawn = Cast<APawn>(GetOwner());
	if (!Pawn)
	{
		// Input is for pawns only.
		return;
	}
	UEnhancedInputComponent* EIC = Cast<UEnhancedInputComponent>(Pawn->InputComponent);
	if (!EIC)
	{
		// Possession or input setup may not have happened yet. With bAutoBindInput the possession and
		// restart hooks call back in.
		// In manual mode, call BindInput() from the pawn's SetupPlayerInputComponent.
		UE_LOG(LogDynamicRope, Verbose, TEXT("RopeWielder on %s: EnhancedInputComponent not ready — will retry on possess/restart (or call BindInput() from SetupPlayerInputComponent)."),
			*GetNameSafe(GetOwner()));
		return;
	}

	// It only counts as a duplicate when the binding target is the same component. When repossession
	// replaces the input component, the existing bindings are cleared below and rebound onto the new one.
	if (BoundInputComponent.Get() == EIC)
	{
		return;
	}
	if (UInputComponent* Old = BoundInputComponent.Get())
	{
		// If the old component is still alive, remove only our bindings so nothing fires twice.
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
		EIC->BindAction(ReleaseAction, ETriggerEvent::Started, this, &URopeWielderComponent::Release);
	}
	if (PullAction)
	{
		// Toggle semantics: each press arms or disarms. When it engages is decided by the tension
		// threshold, in UpdatePullEngage, so Completed and Canceled need no binding since this is not a
		// hold.
		EIC->BindAction(PullAction, ETriggerEvent::Started, this, &URopeWielderComponent::OnPullInputStarted);
	}
	if (ReelInAction)
	{
		EIC->BindAction(ReelInAction, ETriggerEvent::Started,   this, &URopeWielderComponent::StartReelIn);
		EIC->BindAction(ReelInAction, ETriggerEvent::Completed, this, &URopeWielderComponent::StopReel);
		EIC->BindAction(ReelInAction, ETriggerEvent::Canceled,  this, &URopeWielderComponent::StopReel);
	}
	if (ReelOutAction)
	{
		EIC->BindAction(ReelOutAction, ETriggerEvent::Started,   this, &URopeWielderComponent::StartReelOut);
		EIC->BindAction(ReelOutAction, ETriggerEvent::Completed, this, &URopeWielderComponent::StopReel);
		EIC->BindAction(ReelOutAction, ETriggerEvent::Canceled,  this, &URopeWielderComponent::StopReel);
	}
	if (ReloadAction)
	{
		// Loading is a single press that moves a GuaranteedWrap rope into the ready-to-throw Loaded phase.
		EIC->BindAction(ReloadAction, ETriggerEvent::Started, this, &URopeWielderComponent::OnReloadInput);
	}
	BoundInputComponent = EIC;
}

bool URopeWielderComponent::IsRopeInputSuppressed() const
{
	if (bRopeInputSuppressedExternally)
	{
		return true;
	}
	// Any ragdoll counts, partial included: the aim frame and the hand socket both come from a body that
	// is no longer under animation control.
	const URopeRagdollResponseComponent* Response = RagdollResponse.Get();
	return bSuppressInputWhileRagdolled && Response && Response->IsRagdolled();
}

void URopeWielderComponent::SetRopeInputSuppressed(bool bSuppressed)
{
	bRopeInputSuppressedExternally = bSuppressed;
	// The setter resynchronizes the state derived from the flag instead of waiting for the next tick, the
	// same arrangement as SetThrowPreviewEnabled.
	UpdateInputSuppression();
}

void URopeWielderComponent::UpdateInputSuppression()
{
	const bool bSuppressed = IsRopeInputSuppressed();
	if (bSuppressed == bWasRopeInputSuppressed)
	{
		return;
	}
	bWasRopeInputSuppressed = bSuppressed;
	if (!bSuppressed)
	{
		// Nothing is restored when suppression ends: a key still held produces no new Started event, so
		// rope input resumes on the next press, which is what discarding the input has to mean.
		return;
	}
	// Held input is cancelled exactly as an unpossession cancels it, in HandlePawnControllerChanged: the
	// pull toggle would otherwise stay latched and engage by itself, and a reel key held across the
	// transition would keep shortening the rope. The wrap itself is deliberately left alone, so the rope
	// keeps holding and its tether keeps dragging the limp body.
	StopPull();
	StopReel();
}

void URopeWielderComponent::OnThrowInput()
{
	// With no release action and toggle mode enabled, one button both throws and releases.
	if (!ReleaseAction && bThrowActionToggles)
	{
		ToggleThrow();
	}
	else
	{
		Throw();
	}
}

void URopeWielderComponent::StartPull()
{
	// Arming, as a toggle. Neither the force nor the montage starts here: UpdatePullEngage engages once,
	// the first moment the rope is wrapped and the tension crosses PullEngageTension. Arming before the
	// wrap lands means it engages by itself the moment the rope goes tight.
	if (IsRopeInputSuppressed())
	{
		return;
	}
	SetPullArmed(true);
	RefreshTickEnabled();
}

void URopeWielderComponent::UpdatePullEngage()
{
	// The engage decision for an armed pull: a latch that fires the moment the rope is wrapped and the
	// tension condition is first satisfied, once per wrap. The tension threshold takes over the question
	// of when force is applied, which used to belong to the animation window, and with a montage set up
	// the montage carrying that window is played once, with no repeat or interruption management, and
	// left to finish naturally.
	// Without a montage the force is applied immediately. After engaging, the per-frame application gates,
	// such as bActivePullRequiresTaut, are judged by the rope exactly as before, and releasing the wrap
	// stops the force and rearms, keeping the armed state so the next wrap can engage again.
	if (!bPullArmed || !Rope)
	{
		return;
	}
	if (Rope->GetPhase() != ERopePhase::Wrapped)
	{
		if (bPullEngaged)
		{
			// Rearm: the armed state is kept and the next wrap engages again. The UI is told so it can
			// clear its engagement indicator.
			SetPullEngaged(false, 0.0f);
			StopPullNow();
		}
		return;
	}
	if (bPullEngaged)
	{
		return; // Once per wrap; maintaining and clearing it belong to the rope's gates and the wrap's lifetime.
	}
	// The engage test: a threshold of 0 uses the taut latch, IsPullTaut, which is the same judgement the
	// rope's gate makes, and above 0 it uses the maximum tension threshold.
	const bool bEngage = (PullEngageTension <= 0.0f)
		? Rope->IsPullTaut()
		: (Rope->GetConstraintTension() >= PullEngageTension);
	if (!bEngage)
	{
		return;
	}
	SetPullEngaged(true, Rope->GetConstraintTension());
	// A one-shot snapshot at the moment of engagement, as an observation for tuning PullEngageTension.
	UE_LOG(LogDynamicRope, Log,
		TEXT("[PullEngage] share=%.2f tautT=%.0f tetherT=%.0f overshoot=%.0f"),
		Rope->GetEffectiveTetherTargetShare(),
		Rope->GetConstraintTension(), Rope->GetTetherTension(), Rope->GetTetherOvershoot());
	if (PullMontage)
	{
		// The montage is played once and the force is carried by the window notify inside it, through
		// StartPullNow and StopPullNow.
		// If playback fails, fall back immediately to the montage-free path rather than leaving a dead
		// latch that is engaged with nothing driving it.
		if (!TryPlayPullMontage())
		{
			StartPullNow();
		}
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
	// Disarm, stop the force and reset the engage latch. The montage is not interrupted: it is played
	// once by contract, so its lifetime is not managed here and it is left to finish, which avoids the
	// motion cutting off abruptly on release.
	SetPullArmed(false);
	SetPullEngaged(false, 0.0f);
	StopPullNow();
	RefreshTickEnabled();
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
		// The threshold does not apply before the wrap lands, so leaving the gauge at zero reads as "you
		// still have to wrap something".
		return 0.0f;
	}
	if (bPullEngaged)
	{
		return 1.0f;
	}
	if (PullEngageTension <= 0.0f)
	{
		// A threshold of 0 engages on the taut test alone, which has no continuous value, so the same
		// judgement as the gate is returned as 0 or 1.
		return Rope->IsPullTaut() ? 1.0f : 0.0f;
	}
	return FMath::Clamp(Rope->GetConstraintTension() / PullEngageTension, 0.0f, 1.0f);
}

void URopeWielderComponent::UpdatePullGlowMaterial()
{
	if (!bDrivePullGlowMaterial || !Rope)
	{
		return;
	}

	// While pull has never been armed, the material wiring is left untouched. The point is that this
	// feature does not quietly change the render state of a rope that never uses pull.
	UMaterialInstanceDynamic* MID = PullGlowMID.Get();
	if (!MID && !bPullArmed)
	{
		return;
	}

	// Applying a preset can replace the rope material, which leaves our dynamic instance no longer
	// attached to the rope, so it is recreated.
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
	// On a material without the parameter this call is a harmless no-op, with no warning.
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
	if (Rope && !IsRopeInputSuppressed())
	{
		Rope->SetReelRate(Rope->ReelSpeed);
	}
}

void URopeWielderComponent::StartReelOut()
{
	if (Rope && !IsRopeInputSuppressed())
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

void URopeWielderComponent::OnReloadInput()
{
	if (Rope && !IsRopeInputSuppressed())
	{
		Rope->EnterLoaded();
	}
}

void URopeWielderComponent::OnPullInputStarted()
{
	// A toggle: each press arms or disarms. Engaging, which plays the montage once or applies the force,
	// is decided by the tension threshold while armed.
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
	// The public virtual extension hook produces the original context for every real throw request.
	// Applying the aim hit cache is handled separately by the preview-only internal path,
	// BuildThrowContextInternal.
	return BuildBaseThrowContext(AimDir);
}

FVector URopeWielderComponent::GetAimRayOrigin() const
{
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return FVector::ZeroVector;
	}

	const USkeletalMeshComponent* OriginMesh = AttachMesh ? AttachMesh.Get() : Owner->FindComponentByClass<USkeletalMeshComponent>();

	// The aim ray is cast from the aiming reference position the wielder chose, not from the rope or the
	// hand socket.
	// The final throw direction is recomputed below as the hit minus the throw origin.
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
		// When the direction comes from the camera, the origin follows the camera too: a ray whose origin
		// and direction use different references produces an aiming line that does not match the screen.
		// Otherwise it uses eye height, from the pawn's view location.
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

	// The single source for throw parameters is the rope's ThrowParams. The wielder only contributes the
	// origin at the hand socket, the socket velocity and the aim guidance, that is where the throw comes
	// from.
	const FRopeThrowParams DefaultParams;
	const FRopeThrowParams& Params = Rope ? Rope->ThrowParams : DefaultParams;

	Context.Origin = Rope ? Rope->GetComponentLocation() : (Owner ? Owner->GetActorLocation() : FVector::ZeroVector);
	Context.FrameForward = Owner ? Owner->GetActorForwardVector() : FVector::ForwardVector;
	Context.FrameUp = Owner ? Owner->GetActorUpVector() : FVector::UpVector;
	Context.FrameRight = Owner ? Owner->GetActorRightVector() : FVector::RightVector;
	Context.OwnerVelocity = Owner ? Owner->GetVelocity() : FVector::ZeroVector;
	// The hand socket's animation-relative velocity, excluding character movement, is measured in Tick
	// from a component-local position delta.
	Context.HandAnimationVelocity = MeasuredHandAnimVelocityWorld;
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

	// When ThrowInDirection supplies an explicit direction, the ray and the throw have to use the same
	// one.
	// The existing Throw() and ThrowNow() pass a zero vector, which preserves the configured frame
	// forward behaviour.
	const FVector ExplicitAimDir = AimDir.GetSafeNormal();
	if (!ExplicitAimDir.IsNearlyZero())
	{
		SetThrowContextForward(Context, ExplicitAimDir);
	}

	return Context;
}

FRopeThrowContext URopeWielderComponent::BuildThrowContextInternal(const FVector& AimDir) const
{
	if (UsesAimRay())
	{
		if (FRopeThrowContext CachedContext; TryGetCachedAimRayThrowContext(AimDir, CachedContext))
		{
			return CachedContext;
		}
	}

	// The first aiming frame, and an explicit aim direction, have no cached result from a normal gather
	// yet. The original context is returned and the real throw is settled by BuildAimRayThrowRequest from
	// the same frame's PostPhysics gather.
	return BuildThrowContext(AimDir);
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
	Request.BaseContext = BuildThrowContext(AimDir);
	// Mark the context as having come from the aim ray path, which is how the preview builder
	// distinguishes an aiming miss from there being no aiming at all, as on a direct Blueprint or AI
	// call. Setting it once here, in the single factory for aiming contexts, covers hits and misses and
	// also the case of an invalid ray, with a length of 0 because it never crosses the reach sphere: both
	// ResolveAimRayThrowContext and BuildThrowContextInternal start their result from the base context,
	// so the flag survives.
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
		// The same input or montage request is already waiting for the normal gather or for the notify, so
		// the existing request is preserved.
		return false;
	}
	if (!Rope || !Rope->CanThrowNow())
	{
		bGuaranteedAimThrowQueued = false;
		NotifyThrowRejected(ERopeThrowRejectReason::NotLoaded);
		OnThrowRejected.Broadcast(ERopeThrowRejectReason::NotLoaded);
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
	// The plugin's own input gate, tested before the subclass hook so that an override cannot be reached,
	// and therefore cannot be relied on, to enforce it.
	if (IsRopeInputSuppressed())
	{
		NotifyThrowRejected(ERopeThrowRejectReason::InputSuppressed);
		OnThrowRejected.Broadcast(ERopeThrowRejectReason::InputSuppressed);
		return;
	}

	// The subclass game rule gate, such as stamina or character state. ThrowNow on the montage path is not
	// re-tested; see the contract in the header.
	if (!CanThrow())
	{
		NotifyThrowRejected(ERopeThrowRejectReason::Gated);
		OnThrowRejected.Broadcast(ERopeThrowRejectReason::Gated);
		return;
	}

	if (UsesLockedPreview())
	{
		// A real GuaranteedWrap throw does not use the one-frame cache kept for display. The ray captured
		// at the moment of input is resolved into a prepared throw by the same frame's normal gather, and
		// with a montage that result alone is held until the notify.
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
		// The actual throw happens through the montage's UAnimNotify_RopeThrow calling ThrowNow().
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
	// The montage path arrives here through the throw notify, and a ragdoll beginning mid-montage does not
	// stop that notify from firing. A committed throw is discarded rather than thrown from a limp hand;
	// GuaranteedWrap would otherwise cast a fresh aim ray right here, since the queued throw was already
	// cancelled when aiming went inactive.
	if (IsRopeInputSuppressed())
	{
		NotifyThrowRejected(ERopeThrowRejectReason::InputSuppressed);
		OnThrowRejected.Broadcast(ERopeThrowRejectReason::InputSuppressed);
		return;
	}

	if (Rope)
	{
		if (UsesLockedPreview())
		{
			// If the montage input already created a queue entry, the notify only conveys the intent to
			// execute: before the gather it runs as soon as the prepared result is ready, and afterwards it
			// runs now.
			if (Rope->RequestExecuteQueuedGuaranteedAimThrow())
			{
				return;
			}

			// Where there was no preceding Throw(), as with a direct ThrowInDirection from Blueprint or
			// code, the current ray request is created here and executed right after the normal gather.
			QueueGuaranteedAimThrow(AimDir, /*bExecuteWhenReady*/ true);
			return;
		}

		ClearThrowPreview();
		if (UsesAimRay())
		{
		// Only a value-type request is queued, so the hit or fallback is settled right after the latest
		// collider gather.
			FRopeAimRayThrowRequest Request = BuildAimRayThrowRequest(AimDir);
			Request.OnResolved = FSimpleDelegate::CreateUObject(this, &URopeWielderComponent::OnAimRayThrowResolved);
			Rope->QueueAimRayThrow(Request);
			return;
		}

		Rope->ThrowWithContext(BuildThrowContext(AimDir));
		NotifyThrown();
		OnThrown.Broadcast();
	}
}

void URopeWielderComponent::OnAimRayThrowResolved()
{
	bGuaranteedAimThrowQueued = false;
	ClearPreviewDisplay();
	// The success notification is sent after the rope has entered its real execution phase, which is
	// Flight for AssistedJudged and GuidedThrow for GuaranteedWrap.
	NotifyThrown();
	OnThrown.Broadcast();
}

void URopeWielderComponent::OnGuaranteedAimPrepared(FRopePreparedThrowPreview& Prepared)
{
	// Convert the world path settled on the input frame into owner-local space, so that when the notify
	// executes it is restored against the current owner transform, as the GuaranteedWrap contract
	// requires, even if the character moved during the montage. The same value is modified directly
	// before execution.
	StoreAimGuideFrameIfNeeded(Prepared);
	HeldPreparedPreview = Prepared.IsValid()
		? ResolvePreparedPreviewForDisplay(Prepared)
		: FRopeWrapPreviewData();
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

bool URopeWielderComponent::TryPlayPullMontage()
{
	if (!PullMontage)
	{
		return false;
	}
	if (!AttachMesh)
	{
		ResolveRefs();
	}
	UAnimInstance* Anim = AttachMesh ? AttachMesh->GetAnimInstance() : nullptr;
	if (Anim)
	{
		return Anim->Montage_Play(PullMontage, PullMontagePlayRate) > 0.0f;
	}

	UE_LOG(LogDynamicRope, Warning, TEXT("RopeWielder on %s: no AnimInstance to play PullMontage."),
		*GetNameSafe(GetOwner()));
	return false;
}

void URopeWielderComponent::PlayPullMontage()
{
	TryPlayPullMontage();
}


void URopeWielderComponent::Release()
{
	if (Rope && !IsRopeInputSuppressed())
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
		// Off to on: a preview forces the tick on regardless of ComputeDesiredTickEnabled and therefore
		// does not go through RefreshTickEnabled, so the hand velocity sample is reseeded directly here to
		// avoid a stale socket delta spike on reactivation.
		if (!IsComponentTickEnabled())
		{
			ResetHandAnimVelocitySample();
		}
		SetComponentTickEnabled(true);
		UpdateThrowPreview();
	}
	else
	{
		// Only the display is turned off. The prepared data used for throwing is left alone, and
		// GuaranteedWrap and the aim ray keep their calculation tick running.
		ClearPreviewDisplay();
		RefreshTickEnabled();
	}
}

bool URopeWielderComponent::ComputeDesiredTickEnabled() const
{
	// The engage decision for an armed pull, and restoring an air control boost that was already applied,
	// both need a tick regardless of the settings toggles.
	// Aim ray modes refresh the collider gather bounds and the HUD sample every frame, independently of
	// preview display.
	const bool bNeedsPawnLeash =
		Cast<APawn>(GetOwner()) && Rope &&
		Rope->HoldConfig.bEnforceWielderLengthConstraint &&
		(Rope->GetPhase() == ERopePhase::Wrapping || Rope->GetPhase() == ERopePhase::Wrapped);
	// In a throwable state such as Loaded, the hand's animation swing has to be measured every frame to be
	// carried into the throw. Even with FullSimulation and every movement assist disabled, this sampling
	// alone keeps the tick alive.
	const bool bNeedsHandVelocitySampling = Rope && AttachMesh && Rope->CanThrowNow();
	return bNeedsHandVelocitySampling || bNeedsPawnLeash || bPullArmed || bAirControlBoosted ||
		bAutoGroundExitOnUpwardPull || bBoostAirControlWhileSwinging || UsesAimRay();
}

void URopeWielderComponent::HandleRopePhaseChanged(
	ERopePhase /*OldPhase*/, ERopePhase NewPhase)
{
	RefreshTickEnabled();
	if (Rope &&
		(NewPhase == ERopePhase::Wrapping ||
			NewPhase == ERopePhase::Wrapped))
	{
		FRopeWielderMovementConstraint Constraint;
		if (Rope->BuildWielderMovementConstraint(Constraint))
		{
			RefreshTargetMovementConstraintHooks(Constraint);
			return;
		}
	}
	ClearTargetMovementConstraintHooks();
}

void URopeWielderComponent::RegisterMovementConstraintHooks()
{
	UnregisterMovementConstraintHooks();

	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	UMovementComponent* Movement = nullptr;
	if (ACharacter* Character = Cast<ACharacter>(Owner))
	{
		Movement = Character->GetCharacterMovement();
	}
	else if (APawn* Pawn = Cast<APawn>(Owner))
	{
		Movement = Pawn->GetMovementComponent();
	}
	if (!Movement)
	{
		Movement = Owner->FindComponentByClass<UMovementComponent>();
	}
	ConstraintMovementComponent = Movement;
	if (Movement)
	{
		// Wielder must observe the completed movement (including Character mesh child propagation),
		// then project before Chaos and the Rope subsystem's PostPhysics simulation.
		AddTickPrerequisiteComponent(Movement);
	}
}

void URopeWielderComponent::UnregisterMovementConstraintHooks()
{
	ClearTargetMovementConstraintHooks();
	if (UMovementComponent* Movement = ConstraintMovementComponent.Get())
	{
		RemoveTickPrerequisiteComponent(Movement);
	}
	ConstraintMovementComponent.Reset();
	bLeashCorrectionBlockedLogged = false;
}

void URopeWielderComponent::RefreshTargetMovementConstraintHooks(
	const FRopeWielderMovementConstraint& Constraint)
{
	USceneComponent* TargetComponent =
		const_cast<USceneComponent*>(Constraint.TargetComponent.Get());
	UMovementComponent* TargetMovement = nullptr;
	AActor* TargetOwner =
		TargetComponent ? TargetComponent->GetOwner() : nullptr;
	if (ACharacter* Character = Cast<ACharacter>(TargetOwner))
	{
		TargetMovement = Character->GetCharacterMovement();
	}
	else if (APawn* Pawn = Cast<APawn>(TargetOwner))
	{
		TargetMovement = Pawn->GetMovementComponent();
	}
	if (!TargetMovement && TargetOwner)
	{
		TargetMovement =
			TargetOwner->FindComponentByClass<UMovementComponent>();
	}

	if (ConstraintTargetActor.Get() == TargetOwner &&
		ConstraintTargetComponent.Get() == TargetComponent &&
		ConstraintTargetMovementComponent.Get() == TargetMovement)
	{
		return;
	}
	ClearTargetMovementConstraintHooks();
	if (TargetOwner && TargetOwner != GetOwner())
	{
		AddTickPrerequisiteActor(TargetOwner);
		ConstraintTargetActor = TargetOwner;
	}
	if (TargetMovement &&
		TargetMovement != ConstraintMovementComponent.Get())
	{
		AddTickPrerequisiteComponent(TargetMovement);
		ConstraintTargetMovementComponent = TargetMovement;
	}
	if (TargetComponent)
	{
		AddTickPrerequisiteComponent(TargetComponent);
		ConstraintTargetComponent = TargetComponent;
	}
}

void URopeWielderComponent::ClearTargetMovementConstraintHooks()
{
	if (AActor* TargetActor = ConstraintTargetActor.Get())
	{
		RemoveTickPrerequisiteActor(TargetActor);
	}
	if (UMovementComponent* TargetMovement =
		ConstraintTargetMovementComponent.Get())
	{
		RemoveTickPrerequisiteComponent(TargetMovement);
	}
	if (USceneComponent* TargetComponent =
		ConstraintTargetComponent.Get())
	{
		RemoveTickPrerequisiteComponent(TargetComponent);
	}
	ConstraintTargetActor.Reset();
	ConstraintTargetMovementComponent.Reset();
	ConstraintTargetComponent.Reset();
}

void URopeWielderComponent::EnforceWielderLengthConstraint(float DeltaTime)
{
	if (!Rope || DeltaTime <= KINDA_SMALL_NUMBER)
	{
		return;
	}
	// Replicated simulated proxies must follow authoritative movement snapshots. The plugin does not
	// replicate wrap descriptors yet; projecting a proxy locally would fight network smoothing.
	if (const APawn* Pawn = Cast<APawn>(GetOwner());
		Pawn && Pawn->GetLocalRole() == ROLE_SimulatedProxy)
	{
		return;
	}

	FRopeWielderMovementConstraint Constraint;
	if (!Rope->BuildWielderMovementConstraint(Constraint, DeltaTime))
	{
		bLeashCorrectionBlockedLogged = false;
		return;
	}
	RefreshTargetMovementConstraintHooks(Constraint);

	const FVector DesiredPinWorld = Rope->GetComponentLocation();
	const RopeMovementConstraint::FProjectionResult Projection =
		RopeMovementConstraint::ProjectPoint(
			DesiredPinWorld, Constraint.PivotWorld, Constraint.MaxDistance);
	// Hard projection is a *movement adapter* correction: it assumes the pawn's authoritative position is the
	// one the game thread just wrote. A simulating owner — a Chaos vehicle, a physics prop — breaks that
	// assumption in both directions. Moving it here is discarded by the next physics sync (and warns in the
	// editor: "Attempting to move a fully simulated skeletal mesh"), and UMovementComponent::Velocity is an
	// output mirror such a movement component never reads back, so the velocity share below would be lost too.
	// Worse, recording the attempt would tell the tether that this frame's outward velocity had already been
	// removed from the Wielder, suppressing the reaction it never actually received.
	// The boundary for that owner is enforced by the physical tether instead, which PrepareWielderLengthConstraint
	// binds directly to its body so Chaos resolves both ends together inside the substep.
	const bool bHardProjectionApplied =
		Rope->HoldConfig.TetherCompliance <= KINDA_SMALL_NUMBER &&
		!Rope->IsWielderPhysicallySimulated();
	const float WielderPositionCorrectionShare =
		(bHardProjectionApplied && Projection.bConstrained)
			? Rope->ComputeWielderLengthPositionCorrectionShare(
				Constraint)
			: 1.0f;

	UMovementComponent* Movement = ConstraintMovementComponent.Get();
	USceneComponent* MovementUpdatedComponent = Movement ? Movement->UpdatedComponent.Get() : nullptr;
	USceneComponent* UpdatedComponent = MovementUpdatedComponent;
	if (!UpdatedComponent && GetOwner())
	{
		UpdatedComponent = GetOwner()->GetRootComponent();
	}
	const FVector AttemptedVelocity = Movement ? Movement->Velocity : FVector::ZeroVector;

	if (bHardProjectionApplied && Projection.bConstrained && UpdatedComponent)
	{
		// A movable physical target receives the complementary correction through the
		// same-frame Chaos constraint. Keep only the Wielder's generalized share here;
		// projecting the hand 100% and then moving the target would shorten the real span.
		const FVector Correction =
			(Projection.Position - DesiredPinWorld) *
			WielderPositionCorrectionShare;
		const FVector ExpectedPinWorld =
			DesiredPinWorld + Correction;
		FHitResult Hit;
		if (Movement && MovementUpdatedComponent == UpdatedComponent)
		{
			Movement->SafeMoveUpdatedComponent(
				Correction, UpdatedComponent->GetComponentQuat(), /*bSweep*/ true, Hit);
		}
		else
		{
			UpdatedComponent->MoveComponent(
				Correction,
				UpdatedComponent->GetComponentQuat(),
				/*bSweep*/ true,
				&Hit,
				MOVECOMP_NoFlags,
				ETeleportType::None);
		}

		const float Residual =
			static_cast<float>(FVector::Distance(
				Rope->GetComponentLocation(),
				ExpectedPinWorld));
		if (Residual > 0.5f)
		{
			if (!bLeashCorrectionBlockedLogged)
			{
				UE_LOG(LogDynamicRope, Warning,
					TEXT("[%s] hard rope leash correction blocked by collision (residual %.2f cm); ")
					TEXT("world collision and fixed cable length are simultaneously infeasible."),
					*GetName(), Residual);
				bLeashCorrectionBlockedLogged = true;
			}
		}
		else
		{
			bLeashCorrectionBlockedLogged = false;
		}
	}
	else
	{
		bLeashCorrectionBlockedLogged = false;
	}

	FVector BoundaryNormal = Projection.OutwardNormal;
	FVector ConstrainedVelocity = AttemptedVelocity;
	FVector FullyConstrainedVelocity = AttemptedVelocity;
	if (bHardProjectionApplied && Movement && Projection.bAtLimit)
	{
		BoundaryNormal =
			(Rope->GetComponentLocation() - Constraint.PivotWorld).GetSafeNormal();
		if (BoundaryNormal.IsNearlyZero())
		{
			BoundaryNormal = Projection.OutwardNormal;
		}
		FullyConstrainedVelocity = RopeMovementConstraint::RemoveOutwardVelocity(
			AttemptedVelocity, Constraint.PivotVelocity, BoundaryNormal);
		// Velocity uses the same generalized split as position. A simulated target receives
		// its complementary share in the same Chaos step; fixed/kinematic targets make this
		// share 1 so the Wielder alone closes the boundary.
		const float WielderReactionShare =
			Rope->ComputeWielderLengthReactionShare(Constraint);
		ConstrainedVelocity = FMath::Lerp(
			AttemptedVelocity,
			FullyConstrainedVelocity,
			WielderReactionShare);
		Movement->Velocity = ConstrainedVelocity;

		if (UCharacterMovementComponent* CharacterMovement =
			Cast<UCharacterMovementComponent>(Movement))
		{
			CharacterMovement->bForceNextFloorCheck = true;
		}
	}

	// Preserve the motion rejected by the coupled hard correction. A physical target is not
	// position-corrected until the same-frame Chaos step, while a fixed target is already at
	// C=0 here; both still need the original attempted load for authoritative tension.
	// PositionViolation also covers teleport/custom movement without Velocity.
	const float RejectedSeparatingSpeed =
		RopeMovementConstraint::ComputeRejectedSeparatingSpeed(
			AttemptedVelocity,
			FullyConstrainedVelocity,
			BoundaryNormal,
			Projection.Violation,
			DeltaTime);
	Rope->PrepareWielderLengthConstraint(
		Constraint,
		BoundaryNormal,
		RejectedSeparatingSpeed,
		Projection.Violation,
		Projection.bAtLimit,
		bHardProjectionApplied,
		DeltaTime);
}

void URopeWielderComponent::RefreshModeDerivedState()
{
	// The preview belongs to GuaranteedWrap alone, so a runtime component is created only for that mode
	// when display is wanted; a manually placed one is used instead where it exists. Leaving that mode
	// clears any lingering preview display while keeping the component idle for reuse if the mode returns.
	// The HUD widget is re-derived every tick by UpdateAimHudWidget, but switching to FullSimulation can
	// disable the tick itself, so widget creation and destruction are settled once here before that
	// happens.
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
	RefreshTickEnabled();
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
		// Even an immediate execution stays queued until the PostPhysics gather. The preview is not rebuilt
		// from the previous HUD cache in the meantime.
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
		// The result settled by the input frame's PostPhysics is held by the rope until the notify. The
		// preview only needs to be visible while aiming from Loaded, so it is hidden during the windup and
		// no new path is built from the previous HUD cache.
		ClearPreviewDisplay();
		return true;
	}

	// If the montage ended without a notify, or failed to play, the scheduled real throw is cancelled too.
	if (Rope)
	{
		Rope->CancelQueuedGuaranteedAimThrow();
	}
	bGuaranteedAimThrowQueued = false;
	return false;
}

bool URopeWielderComponent::ShouldUpdateThrowPreviewForPhase(ERopePhase Phase) const
{
	// GuaranteedWrap only builds a fresh preview path for a throw that has not happened yet. During
	// GuidedThrow and Wrapped the already-settled HeldPreparedPreview is used and no build is attempted;
	// the caller filters those out first.
	// The aiming preview is built only in the ready-to-throw Loaded phase, because that is the only phase
	// it can be thrown from. Sharing the predicate with the throw gate, CanThrowInPhase, is what keeps
	// "what you see" and "what you can throw" aligned. Asking with the Phase argument rather than the live
	// phase is what keeps this function consistent with its own signature.
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
		// GuidedThrow authoritatively follows the cached preview path and builds no new one. The preview
		// only needs to be visible while aiming from Loaded, so the display is cleared after the throw
		// while the HeldPreparedPreview data is preserved, since the phase-gate validity check reads it.
		// Returning true stops this phase falling through to a preview rebuild.
		ClearPreviewDisplay();
		return true;
	}

	if (Phase == ERopePhase::Wrapped)
	{
		// Once the embed is committed the aiming path has no meaning, so the data is discarded as well.
		ClearThrowPreview();
		return true;
	}

	return false;
}

void URopeWielderComponent::DisplayPreviewCenterline(const FRopeWrapPreviewData& Centerline)
{
	// Display is entirely optional and separate from the calculation, so failing here never touches the
	// prepared data used for throwing.
	if (!bShowThrowPreview || !PreviewComponent || !Rope)
	{
		return;
	}

	PreviewComponent->SetWrapPreviewWorld(Centerline);
}

void URopeWielderComponent::UpdateThrowPreview()
{
	// The on-screen preview is built only while display is enabled. The prepared data for a real
	// GuaranteedWrap throw is settled at the moment of input by QueueGuaranteedAimThrow from a fresh
	// normal gather, so there is no need to build it every tick while display is off.
	// FullSimulation and AssistedJudged use no preview: their wrap is judged or emergent, so there is no
	// path to settle before the throw. Aiming feedback for AssistedJudged is handled separately by the aim
	// ray HUD, in UpdateAimHudSample.
	if (!bShowThrowPreview)
	{
		ClearPreviewDisplay();
		return;
	}
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
		// A preview build is meaningless in this phase, so it is cleared quietly without producing a
		// failure log.
		ClearThrowPreview();
		return;
	}
	if (!bHasAimRayFrameThrowContext)
	{
		// The first aiming frame has no result from a normal gather yet. Rather than drawing an
		// open-space preview from the base fallback for one frame and then flickering to the target path,
		// it waits for the settled result on the next tick.
		ClearThrowPreview();
		return;
	}

	FString PreviewBuildReason;
	const FRopeThrowContext ThrowContext = BuildThrowContextInternal(FVector::ZeroVector);
	FRopePreparedThrowPreview Prepared;
#if WITH_DEV_AUTOMATION_TESTS
	++TestPreparedPreviewBuildCount;
#endif
	if (!Rope->BuildPreparedWrappingPreview(ThrowContext, Prepared, &PreviewBuildReason))
	{
		ClearThrowPreview();
		return;
	}

	StoreAimGuideFrameIfNeeded(Prepared);
	HeldPreparedPreview = ResolvePreparedPreviewForDisplay(Prepared);
	DisplayPreviewCenterline(HeldPreparedPreview);
}

void URopeWielderComponent::ClearPreviewDisplay()
{
	// Clear the display only; the prepared data used for throwing is left alone.
	if (PreviewComponent && PreviewComponent->IsPreviewOwner(this))
	{
		PreviewComponent->ClearPreview();
	}
}

void URopeWielderComponent::ClearPreparedThrow()
{
	HeldPreparedPreview = FRopeWrapPreviewData();
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

	// An aim ray path is fixed in the wielder's owner-local space rather than that of the socket or the
	// rope component.
	Prepared.StoreGuideFrameLocal(OwnerRoot);
}

FRopeWrapPreviewData URopeWielderComponent::ResolvePreparedPreviewForDisplay(const FRopePreparedThrowPreview& Prepared) const
{
	// An ordinary preview, not stored in owner-local space, returns its original world points unchanged.
	FRopeWrapPreviewData Preview = Prepared.ResolveRenderPreviewWorld();
	if (!Rope || Rope->ResolveMode != ERopeWrapResolveMode::GuaranteedWrap || !Preview.IsValid())
	{
		return Preview;
	}

	// On the prepared throw path for Pierce, the rope component shortens the path to the tail world
	// position so the last node lands on the tail socket.
	// The display has to reach the head, that is the point the player aimed at, so the render-only
	// centreline alone is extended back out to the hit point.
	const FRopeSurfaceAnchor* Anchor = Prepared.Anchors.Num() > 0 ? &Prepared.Anchors[0] : &Prepared.LatchAnchor;
	if (!Anchor || Anchor->NodeIndex == INDEX_NONE)
	{
		return Preview;
	}

	FVector HitPoint = Anchor->StartWorldPosition;
	if (Prepared.ThrowContext.bHasAimGuideHit)
	{
		// The display endpoint of an aim guide preview is the hit the aiming ray actually selected, not a
		// value unpacked again from the anchor's local space.
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
