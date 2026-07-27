// Copyright Epic Games, Inc. All Rights Reserved.

#include "Demo/RopeDemoSnare.h"
#include "Demo/RopeDemoPressurePlate.h"
#include "Gameplay/RopeRagdollResponseComponent.h"
#include "RopeComponent.h"
#include "Logic/RopeAimTargeting.h"
#include "Core/RopeLifecycleTypes.h"
#include "Core/RopeThrowTypes.h"
#include "DynamicRopeLog.h"

#include "Components/SkeletalMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "UObject/ConstructorHelpers.h"

ARopeDemoSnare::ARopeDemoSnare()
{
	// A tick is needed to poll for the snare being established and to drive the reel.
	PrimaryActorTick.bCanEverTick = true;

	Base = CreateDefaultSubobject<USceneComponent>(TEXT("Base"));
	SetRootComponent(Base);

	// Only engine basic shapes are used, since the plugin never references /Game content.
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));

	AnchorMarkers.Reserve(MaxBindings);
	Ropes.Reserve(MaxBindings);
	for (int32 Index = 0; Index < MaxBindings; ++Index)
	{
		UStaticMeshComponent* Marker = CreateDefaultSubobject<UStaticMeshComponent>(*FString::Printf(TEXT("Anchor%d"), Index));
		if (Marker)
		{
			Marker->SetupAttachment(Base);
			// A 15 cm marker. Only the anchor position needs to be visible, so collision is disabled to keep
			// it from interfering with the rope or the character.
			Marker->SetRelativeScale3D(FVector(0.15f));
			Marker->SetCollisionEnabled(ECollisionEnabled::NoCollision);
			if (CubeMesh.Succeeded())
			{
				Marker->SetStaticMesh(CubeMesh.Object);
			}
			AnchorMarkers.Add(Marker);
		}

		URopeComponent* Cable = CreateDefaultSubobject<URopeComponent>(*FString::Printf(TEXT("Rope%d"), Index));
		if (Cable)
		{
			Cable->SetupAttachment(Marker ? static_cast<USceneComponent*>(Marker) : static_cast<USceneComponent*>(Base));
			// The demo has to wrap the named limbs without fail, so it is fixed to GuaranteedWrap.
			Cable->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
			Ropes.Add(Cable);
		}
	}

	// The default fills all four limbs for a full spread. It assumes the target stands at the origin facing
	// +X, which puts its left on -Y.
	// The hands take the upper diagonal anchors and the feet the lower ones. Reduce it freely in the level
	// by clearing a slot's bone, deleting an entry, or adjusting the anchor offsets; leaving two bound
	// restrains the arms alone.
	Bindings.Reserve(MaxBindings);
	{
		FRopeDemoSnareBinding LeftHand;
		LeftHand.Bone = TEXT("hand_l");
		LeftHand.AnchorOffset = FVector(0.0f, -250.0f, 200.0f);
		Bindings.Add(LeftHand);

		FRopeDemoSnareBinding RightHand;
		RightHand.Bone = TEXT("hand_r");
		RightHand.AnchorOffset = FVector(0.0f, 250.0f, 200.0f);
		Bindings.Add(RightHand);

		FRopeDemoSnareBinding LeftFoot;
		LeftFoot.Bone = TEXT("foot_l");
		LeftFoot.AnchorOffset = FVector(0.0f, -250.0f, 30.0f);
		Bindings.Add(LeftFoot);

		FRopeDemoSnareBinding RightFoot;
		RightFoot.Bone = TEXT("foot_r");
		RightFoot.AnchorOffset = FVector(0.0f, 250.0f, 30.0f);
		Bindings.Add(RightFoot);
	}
}

void ARopeDemoSnare::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	// Reapply the layout so the markers and ropes follow the moment an anchor offset is edited in the
	// editor.
	ApplyBindingLayout();
}

void ARopeDemoSnare::BeginPlay()
{
	Super::BeginPlay();

	ApplyBindingLayout();

	if (TriggerPlate)
	{
		TriggerPlate->OnPlatePressedChanged.AddDynamic(this, &ARopeDemoSnare::HandleTriggerPlateChanged);
	}

	if (!TargetActor)
	{
	// With a plate assigned this is trap mode, which picks up the occupying actor the moment it is pressed,
	// so having no target here is normal.
		if (!TriggerPlate)
		{
			UE_LOG(LogDynamicRope, Warning,
				TEXT("[%s] demo snare has no TargetActor and no TriggerPlate — set a target, or wire a plate for trap mode, or it will never fire."),
				*GetName());
		}
	}
	else if (!ResolveTargetMesh())
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] demo snare target '%s' has no skeletal mesh — nothing to wrap."),
			*GetName(), *TargetActor->GetName());
	}

	if (GetActiveBindingCount() == 0)
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] demo snare has no active bindings — fill Bindings with limb bone names."), *GetName());
	}

	if (bSnareOnBeginPlay)
	{
		TriggerSnare();
	}
}

void ARopeDemoSnare::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (TriggerPlate)
	{
		TriggerPlate->OnPlatePressedChanged.RemoveDynamic(this, &ARopeDemoSnare::HandleTriggerPlateChanged);
	}

	Super::EndPlay(EndPlayReason);
}

void ARopeDemoSnare::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (!bTriggered || GetActiveBindingCount() == 0)
	{
		return;
	}

	//~ 1) Establishing the snare: fire again periodically until every active slot is wrapped.
	if (!bSnared)
	{
		if (AreAllBoundRopesWrapped())
		{
			SetSnared(true);
			return;
		}

		FireRetryRemaining -= DeltaSeconds;
		if (FireRetryRemaining <= 0.0f)
		{
			FireSnareRopes();
			FireRetryRemaining = 1.0f; // Retry every second, which also settles the gather and the arming edge.
		}
		return;
	}

	//~ 2) If any rope released, whether the target was lost or the distance limit was hit, return to the
	//~    establishing stage.
	if (!AreAllBoundRopesWrapped())
	{
		for (URopeComponent* Cable : Ropes)
		{
			if (Cable)
			{
				Cable->SetReelRate(0.0f);
				Cable->SetActivePull(0.0f);
			}
		}
		SetSnared(false);
		FireRetryRemaining = 0.5f;
		return;
	}

	//~ 2.5) Automatic release: let go once AutoReleaseDelay has elapsed since the snare was established,
	//~    where 0 disables it. A target still standing on the plate is not snared again; rearming requires
	//~    the plate's next pressing edge or a manual TriggerSnare.
	if (AutoReleaseDelay > 0.0f)
	{
		AutoReleaseRemaining -= DeltaSeconds;
		if (AutoReleaseRemaining <= 0.0f)
		{
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] snare auto-released after %.1fs."),
				*GetName(), AutoReleaseDelay);
			ReleaseSnare();
			return;
		}
	}

	//~ 3) Spread the limbs: reel in to the target length, plus the per-slot active pull. The tether drags
	//~    each bone towards its anchor.
	for (int32 Index = 0; Index < Ropes.Num(); ++Index)
	{
		if (!IsSlotActive(Index))
		{
			continue;
		}
		URopeComponent* Cable = Ropes[Index];
		const float MaxLength = FMath::Max(Cable->RopeLength, Cable->MinRopeLength);
		const float TargetLength = (SnareLength > 0.0f)
			? FMath::Clamp(SnareLength, Cable->MinRopeLength, MaxLength)
			: Cable->MinRopeLength;

		Cable->SetReelRate(Cable->GetCurrentRopeLength() > TargetLength + ArrivalTolerance ? SnareReelSpeed : 0.0f);
		Cable->SetActivePull(LimbPullForce);
	}
}

void ARopeDemoSnare::TriggerSnare()
{
	if (bTriggered)
	{
		return;
	}
	// Trap mode, meaning no assigned target: even when triggered manually or from Blueprint, an occupying
	// actor is picked up if the plate is already pressed. A call arriving through the plate path is a no-op
	// here, since the plate handler already resolved it.
	if (!TargetActor && !AutoTargetActor.IsValid())
	{
		ResolveAutoTargetFromPlate();
	}
	bTriggered = true;
	FireRetryRemaining = 0.0f; // Fire for the first time on the very next tick.

	ForceTargetRagdoll();
}

void ARopeDemoSnare::ReleaseSnare()
{
	if (!bTriggered)
	{
		return;
	}
	bTriggered = false;

	for (URopeComponent* Cable : Ropes)
	{
		if (!Cable)
		{
			continue;
		}
		Cable->SetReelRate(0.0f);
		Cable->SetActivePull(0.0f);
		// Discard any throw still queued: without that, the queue would execute after the release and a
		// cable would fly off towards a target that no longer exists.
		Cable->CancelQueuedGuaranteedAimThrow();
		// Releasing only wrapped ropes left a hole: a rope in flight, during a guided throw, or in the
		// middle of catching, during contact or wrapping, would finish wrapping after the release, and with
		// the snare no longer triggered the tick would not manage it, while GuaranteedWrap has no automatic
		// release either, leaving a permanent snare. It reproduced on the fast edge of stepping on the plate
		// and immediately off. ReleaseWrap gates all four phases internally, so it is called
		// unconditionally and is a no-op in any other phase.
		Cable->ReleaseWrap();
		// Restore whatever was reeled in so the next snare starts from the same length.
		Cable->SetRopeLength(Cable->RopeLength);
	}

	// Undoing the forced ragdoll on the release-before-wrap path: automatic recovery only fires from the
	// release event of a rope that was actually wrapped, so releasing with no rope ever wrapped, as when
	// stepping on the plate and immediately off, left the target limp forever. A wrapped rope is handled by
	// the release event above and the component's automatic recovery; only the "nothing ever wrapped" case
	// is recovered explicitly here, and the helper's internal gate defers when another rope still holds the
	// target.
	if (AActor* Target = GetEffectiveTargetActor())
	{
		if (URopeRagdollResponseComponent* Response = Target->FindComponentByClass<URopeRagdollResponseComponent>())
		{
			Response->RecoverFromRagdollIfUnheld();
		}
	}

	SetSnared(false);
	// A trap-mode target's lifetime matches the snare's: releasing clears it so the next press acquires a
	// new one.
	AutoTargetActor.Reset();
}

void ARopeDemoSnare::ToggleSnare()
{
	if (bTriggered)
	{
		ReleaseSnare();
	}
	else
	{
		TriggerSnare();
	}
}

int32 ARopeDemoSnare::GetActiveBindingCount() const
{
	int32 Count = 0;
	for (int32 Index = 0; Index < Ropes.Num(); ++Index)
	{
		if (IsSlotActive(Index))
		{
			++Count;
		}
	}
	return Count;
}

int32 ARopeDemoSnare::GetBoundRopeCount() const
{
	int32 Count = 0;
	for (int32 Index = 0; Index < Ropes.Num(); ++Index)
	{
		if (IsSlotActive(Index) && Ropes[Index]->GetPhase() == ERopePhase::Wrapped)
		{
			++Count;
		}
	}
	return Count;
}

bool ARopeDemoSnare::IsSlotActive(int32 SlotIndex) const
{
	return Ropes.IsValidIndex(SlotIndex)
		&& Ropes[SlotIndex] != nullptr
		&& Bindings.IsValidIndex(SlotIndex)
		&& !Bindings[SlotIndex].Bone.IsNone();
}

void ARopeDemoSnare::ApplyBindingLayout()
{
	for (int32 Index = 0; Index < MaxBindings; ++Index)
	{
		const bool bActive = IsSlotActive(Index);
		const FVector Offset = Bindings.IsValidIndex(Index) ? Bindings[Index].AnchorOffset : FVector::ZeroVector;

		if (AnchorMarkers.IsValidIndex(Index) && AnchorMarkers[Index])
		{
			AnchorMarkers[Index]->SetRelativeLocation(Offset);
			AnchorMarkers[Index]->SetVisibility(bActive);
		}
		if (Ropes.IsValidIndex(Index) && Ropes[Index])
		{
			// The ropes are attached to their markers, so the relative position is zero; only unused slots
			// are hidden.
			Ropes[Index]->SetVisibility(bActive);
		}
	}
}

bool ARopeDemoSnare::AreAllBoundRopesWrapped() const
{
	const int32 ActiveCount = GetActiveBindingCount();
	return ActiveCount > 0 && GetBoundRopeCount() == ActiveCount;
}

AActor* ARopeDemoSnare::GetEffectiveTargetActor() const
{
	return TargetActor ? TargetActor.Get() : AutoTargetActor.Get();
}

USkeletalMeshComponent* ARopeDemoSnare::ResolveTargetMesh() const
{
	AActor* Target = GetEffectiveTargetActor();
	return Target ? Target->FindComponentByClass<USkeletalMeshComponent>() : nullptr;
}

void ARopeDemoSnare::ResolveAutoTargetFromPlate()
{
	AutoTargetActor.Reset();
	if (!TriggerPlate)
	{
		return;
	}
	// The demo wraps bones, so only an owner with a skeletal mesh can be a target, which skips things like
	// physics boxes. With several occupants the first eligible one is taken; what counts as an eligible
	// occupant, whether by tag or by simulating physics, is decided by the plate's own settings.
	for (AActor* Occupant : TriggerPlate->GetQualifyingOccupants())
	{
		if (Occupant && Occupant != this && Occupant->FindComponentByClass<USkeletalMeshComponent>())
		{
			AutoTargetActor = Occupant;
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] trap target acquired from plate: %s"),
				*GetName(), *Occupant->GetName());
			return;
		}
	}
}

void ARopeDemoSnare::ForceTargetRagdoll()
{
	AActor* Target = GetEffectiveTargetActor();
	if (!bForceRagdollOnSnare || !Target)
	{
		return;
	}
	// The limbs only spread freely if the target is already under physics before the wrap lands. With no
	// response component there is also no automatic transition from the wrap event, so it is left alone, as
	// with a static mesh target.
	if (URopeRagdollResponseComponent* Response = Target->FindComponentByClass<URopeRagdollResponseComponent>())
	{
		if (!Response->IsRagdolled())
		{
			// Marked as a rope-driven entry, which makes it eligible for automatic recovery once the snare
			// lets go and the last rope releases. Marking it as manual would exclude it from the automatic
			// recovery gate and leave the target limp forever.
			Response->EnterRagdoll(/*bAutoRecoverOnRelease*/ true);
		}
	}
}

void ARopeDemoSnare::SetSnared(bool bNewSnared)
{
	if (bSnared == bNewSnared)
	{
		return;
	}
	bSnared = bNewSnared;

	if (bSnared)
	{
		// Which bone was actually caught can differ from the one aimed at, when a bone higher up blocked the
		// way, so it is logged for inspection.
		FString BoundBones;
		for (int32 Index = 0; Index < Ropes.Num(); ++Index)
		{
			if (IsSlotActive(Index))
			{
				BoundBones += FString::Printf(TEXT("%s%s->%s"), BoundBones.IsEmpty() ? TEXT("") : TEXT(", "),
					*Bindings[Index].Bone.ToString(), *Ropes[Index]->GetWrappedBoneName().ToString());
			}
		}
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] snare bound (%d cables): %s"), *GetName(), GetBoundRopeCount(), *BoundBones);
		// The automatic release countdown starts from the moment the snare is established, excluding the
		// time spent firing and retrying.
		AutoReleaseRemaining = AutoReleaseDelay;
	}
	else
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] snare released."), *GetName());
	}

	OnSnareStateChanged.Broadcast(this, bSnared);
}

void ARopeDemoSnare::HandleTriggerPlateChanged(ARopeDemoPressurePlate* /*Plate*/, bool bPressed)
{
	// The trap trigger: stepping on it snares and stepping off releases.
	if (bPressed)
	{
	// Trap mode: with no assigned target, the actor that stepped on the plate is settled once at the moment
	// it is pressed.
		if (!TargetActor)
		{
			ResolveAutoTargetFromPlate();
			if (!AutoTargetActor.IsValid())
			{
				UE_LOG(LogDynamicRope, Warning,
					TEXT("[%s] plate pressed but no occupant has a skeletal mesh — snare not triggered."),
					*GetName());
				return;
			}
		}
		TriggerSnare();
	}
	else
	{
		ReleaseSnare();
	}
}

void ARopeDemoSnare::FireSnareRopes()
{
	for (int32 Index = 0; Index < Ropes.Num(); ++Index)
	{
		if (IsSlotActive(Index) && Ropes[Index]->GetPhase() != ERopePhase::Wrapped)
		{
			FireSnareRopeFor(Index);
		}
	}
}

bool ARopeDemoSnare::FireSnareRopeFor(int32 SlotIndex)
{
	if (!IsSlotActive(SlotIndex))
	{
		return false;
	}
	USkeletalMeshComponent* Mesh = ResolveTargetMesh();
	if (!Mesh)
	{
		return false;
	}

	URopeComponent* Cable = Ropes[SlotIndex];
	const FName Bone = Bindings[SlotIndex].Bone;
	if (Mesh->GetBoneIndex(Bone) == INDEX_NONE)
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] snare slot %d bone '%s' not found on '%s'."),
			*GetName(), SlotIndex, *Bone.ToString(), *Mesh->GetName());
		return false;
	}

	// GuaranteedWrap can only be thrown from Loaded, so otherwise it only loads and fires on the next
	// attempt, which settles the arming edge.
	if (Cable->GetPhase() != ERopePhase::Loaded)
	{
		Cable->EnterLoaded();
		return false;
	}

	const FVector Origin = Cable->GetComponentLocation();
	const FVector BoneWorld = Mesh->GetSocketLocation(Bone);
	const FVector ToBone = BoneWorld - Origin;
	const float Dist = ToBone.Size();
	if (Dist < KINDA_SMALL_NUMBER)
	{
		return false;
	}
	const FVector AimDir = ToBone / Dist;

	// A minimal reproduction of the wielder's aim ray request, aiming at a fixed bone with no input.
	FRopeAimRayThrowRequest Request;
	FRopeThrowContext& Ctx = Request.BaseContext;
	Ctx.Origin = Origin;
	Ctx.FrameForward = AimDir;
	Ctx.FrameUp = FVector::UpVector;
	Ctx.FrameRight = FVector::CrossProduct(AimDir, FVector::UpVector).GetSafeNormal();
	if (Ctx.FrameRight.IsNearlyZero())
	{
		Ctx.FrameRight = FVector::RightVector; // The degenerate fallback for a vertical aim, where the direction is parallel to up.
	}
	Ctx.FrameMode = ERopeThrowFrameMode::Custom;
	Ctx.bAimRayEvaluated = true; // Marks the context as having come from an aim ray, as the preview builder's contract requires.

	Request.RayOrigin = Origin;
	Request.RayDirection = AimDir;
	Request.ReachOrigin = Origin;
	Request.ReachLength = Dist + 200.0f; // Reach margin, which makes sure the bone is covered.
	Request.RayLength = FRopeAimTargeting::ResolveRayLengthForReach(
		Request.RayOrigin, Request.RayDirection, Request.ReachOrigin, Request.ReachLength);
	Request.QueryRadius = 0.0f;
	Request.SweepStep = 2.0f;

	// Executed immediately after the normal gather, with no montage.
	return Cable->QueueGuaranteedAimThrow(Request, /*bExecuteWhenReady*/ true);
}
