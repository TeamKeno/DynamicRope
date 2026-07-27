// Copyright Epic Games, Inc. All Rights Reserved.

#include "Gameplay/RopeRagdollResponseComponent.h"

#include "DynamicRopeLog.h"
// The full definition of URopeComponent, which is the weak key type of the WrappingRopes engagement set.
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
	// The response is driven by the central wrap and release signals plus a timer. The tick exists only
	// for the ragdoll camera follow, during a full ragdoll, and is disabled the rest of the time.
	// PostPhysics puts it after the ragdoll bones reach their final pose and before the camera manager
	// updates.
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = false;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;
}

void URopeRagdollResponseComponent::BeginPlay()
{
	Super::BeginPlay();

	if (!ResolveMesh())
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] RopeRagdollResponse: no skeletal mesh found - the ragdoll response is disabled."),
			*GetNameSafe(GetOwner()));
	}

	// Receive a notification whenever any rope in the world wraps or releases, which allows a reaction
	// without knowing in advance which rope will wrap this actor.
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
	// Clean up the camera follow. It is harmless whether it runs before the recovery path that fires when
	// only the component is removed, or a second time afterwards, thanks to its internal guard. While the
	// world is being torn down it only cleans up, with no blend.
	EndRagdollCameraFollow();

	// Recovery for the case where only the component is removed, through DestroyComponent or
	// UnregisterComponent. The ragdoll state was created by this component, and the values needed to undo
	// it, namely the collision profile, the attachment and the movement mode, exist only here, so
	// disappearing would leave the target simulating with movement disabled and permanently uncontrollable.
	// Where the actor or the world is dying too, recovering means nothing and only risks touching an object
	// already being destroyed, so it is skipped.
	AActor* Owner = GetOwner();
	const bool bComponentOnlyTeardown = bRagdolled
		&& IsValid(Owner) && !Owner->IsActorBeingDestroyed()
		&& World && !World->bIsTearingDown;
	if (bComponentOnlyTeardown)
	{
		UE_LOG(LogDynamicRope, Log,
			TEXT("[%s] RopeRagdollResponse: component removed - restoring the ragdoll state before leaving."), *GetNameSafe(Owner));
		RecoverFromRagdoll();
	}
	Super::EndPlay(EndPlayReason);
}

void URopeRagdollResponseComponent::HandleAnyRopeWrapped(const FRopeWrappedEventInfo& Info)
{
	const USkeletalMeshComponent* MyMesh = ResolveMesh();
	if (!MyMesh || Info.Mesh.Get() != MyMesh)
	{
		// A different mesh was wrapped, so this is ignored.
		return;
	}

	// The engagement is registered before the ragdoll gate: a second rope has to be counted even while
	// already limp, or releasing one of them would recover the target early. Returning immediately when
	// already limp would mean it was never recorded at all.
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
	// A rope destroyed while still wrapped can never fire a release signal and remains only as an expired
	// weak pointer. Without pruning them here the set would never empty and automatic recovery would die.
	const int32 RemainingRopes = PruneWrappingRopes();

	// If the wrap released while the delay was still pending, as in a fast wrap and release, the scheduled
	// automatic transition is cancelled, unless another rope is still wrapped, in which case the schedule
	// belongs to that rope and is kept.
	if (RemainingRopes == 0)
	{
		if (UWorld* World = GetWorld())
		{
			World->GetTimerManager().ClearTimer(AutoRagdollTimer);
		}
	}

	// Several ropes can wrap one target, so the target is stood up only once the last of them releases;
	// recovering when just one released would leave it standing while still wrapped by the others.
	if (RemainingRopes > 0)
	{
		UE_LOG(LogDynamicRope, Verbose,
			TEXT("[%s] RopeRagdollResponse: rope released (%s) - %d rope(s) still wrapped, so recovery is deferred."),
			*GetNameSafe(GetOwner()), *UEnum::GetValueAsString(Reason), RemainingRopes);
		return;
	}

	// Only an automatically entered ragdoll recovers automatically; a manual entry is kept. RecoverFromRagdoll
	// guards the mesh validity and the ragdoll state again, so a lost target, as with a Broken release,
	// simply becomes a quiet no-op.
	if (bRecoverRagdollOnRopeRelease && bRagdolled && bRagdollWasAutoTriggered)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeRagdollResponse: rope released (%s) - recovering the automatic ragdoll."),
			*GetNameSafe(GetOwner()), *UEnum::GetValueAsString(Reason));
		RecoverFromRagdoll();
	}
}

bool URopeRagdollResponseComponent::RecoverFromRagdollIfUnheld()
{
	// The same gate as the automatic recovery in HandleAnyRopeReleased. This API is only an explicit
	// trigger for paths where the release event never arrives, and does not widen the eligibility rules: a
	// manual ragdoll and an opt-out of recovery are both still respected.
	if (!bRecoverRagdollOnRopeRelease || !bRagdolled || !bRagdollWasAutoTriggered)
	{
		return false;
	}
	// While any rope is still wrapped, recovery belongs to the automatic path on that last release, which
	// prevents standing the target up when only one of several has let go.
	if (PruneWrappingRopes() > 0)
	{
		return false;
	}
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeRagdollResponse: no rope still wrapped - explicitly recovering the rope-driven ragdoll."),
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
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeRagdollResponse: wrapped - going partially limp below the wrapped bone (%s)."),
			*GetNameSafe(GetOwner()), *PendingWrappedBone.ToString());
		EnterPartialRagdoll(PendingWrappedBone, /*bAutoRecoverOnRelease*/ true);
	}
	else
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeRagdollResponse: wrapped - going fully limp."), *GetNameSafe(GetOwner()));
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

	// Validate the physics asset before changing anything. Without bodies, SetSimulatePhysics(true) fails
	// silently, and having already disabled movement and capsule collision first would leave the target
	// neither limp nor able to walk, stuck with the ragdoll flag set and no way to control it. The partial
	// ragdoll path already followed this order; only the full ragdoll was asymmetric.
	const UPhysicsAsset* PhysAsset = Mesh->GetPhysicsAsset();
	if (!PhysAsset || PhysAsset->SkeletalBodySetups.Num() == 0)
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] RopeRagdollResponse: the mesh has no physics bodies (asset: %s), so the full ragdoll is skipped."),
			*GetNameSafe(GetOwner()), PhysAsset ? TEXT("zero bodies") : TEXT("none"));
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
	// A rope-driven entry, whether the automatic transition on a wrap or a snare forcing it, passes true and
	// is eligible for the automatic recovery gate on release.
	// False marks a manual entry, which a rope must never stand up on its own.
	bRagdollWasAutoTriggered = bAutoRecoverOnRelease;
	BeginRagdollCameraFollow();
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeRagdollResponse: entering a full ragdoll."), *GetNameSafe(GetOwner()));
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
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] RopeRagdollResponse: bone '%s' not found, so the partial ragdoll is skipped."),
			*GetNameSafe(GetOwner()), *BoneName.ToString());
		return;
	}

	// A bone that exists in the skeleton but has no body in the physics asset, such as a twist or IK bone,
	// gives SetAllBodiesBelow nothing to act on and is silently ignored. That is particularly easy to hit on
	// the automatic bOnlyBelowWrappedBone path, which passes the wrapped bone straight through. The parent
	// chain is walked up to a bone that does have a body. It is the same principle as
	// FindNearestSimulatingBone on the pull side, except that the criterion here is having a body rather
	// than currently simulating.
	UPhysicsAsset* PhysAsset = Mesh->GetPhysicsAsset();
	if (!PhysAsset)
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] RopeRagdollResponse: no physics asset, so the partial ragdoll is skipped."),
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
			TEXT("[%s] RopeRagdollResponse: neither bone '%s' nor its parent chain has a physics body, so the partial ragdoll is skipped."),
			*GetNameSafe(GetOwner()), *BoneName.ToString());
		return;
	}
	if (BodyBone != BoneName)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeRagdollResponse: bone '%s' has no body, so it was promoted to '%s'."),
			*GetNameSafe(GetOwner()), *BoneName.ToString(), *BodyBone.ToString());
	}

	SaveRestoreState(Mesh);

	// The capsule and movement are kept: the remaining bones, which do not simulate, keep animating.
	Mesh->SetCollisionProfileName(RagdollCollisionProfileName);
	ApplyRagdollOverlapEvents(Mesh);
	Mesh->SetAllBodiesBelowSimulatePhysics(BodyBone, true, /*bIncludeSelf*/ true);
	Mesh->SetAllBodiesBelowPhysicsBlendWeight(BodyBone, 1.0f);
	ApplyRagdollCCD(Mesh, true);

	bRagdolled = true;
	bPartial = true;
	bRagdollWasAutoTriggered = bAutoRecoverOnRelease;
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeRagdollResponse: entering a partial ragdoll below bone %s."),
		*GetNameSafe(GetOwner()), *BodyBone.ToString());
}

void URopeRagdollResponseComponent::RecoverFromRagdoll()
{
	USkeletalMeshComponent* Mesh = ResolveMesh();
	if (!Mesh || !bRagdolled)
	{
		return;
	}

	// Capsule realignment, for a full ragdoll only: capture the anchor bone's world position before the
	// simulation is disabled, while the ragdoll pose still exists. Disabling the simulation below resets the
	// mesh to its reference pose and destroys that information, so it is taken in advance here.
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
				TEXT("[%s] RopeRagdollResponse: realignment anchor bone '%s' not found in the skeleton, so the capsule realignment is skipped."),
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
		// A full ragdoll let the mesh follow physics away from the capsule, so the original attachment is
		// restored.
		if (USceneComponent* Parent = SavedAttachParent.Get())
		{
			Mesh->AttachToComponent(Parent, FAttachmentTransformRules::KeepRelativeTransform, SavedAttachSocket);
		}
		Mesh->SetRelativeTransform(SavedMeshRelative);

		// The reset returned the mesh to its reference pose at the capsule's location. Moving the capsule,
		// that is the actor, to where the ragdoll came to rest makes that return invisible rather than a
		// teleport. The offset is the ragdoll anchor minus the current anchor in world space. The height is
		// only applied on a character and when the ground search distance is above 0, which prevents
		// returning to the old height after vertical transport such as a helicopter carry: the ground is
		// traced for below the anchor and the capsule is stood on it, and where none is found, meaning it
		// was dropped in mid-air, it continues falling from the anchor height. Otherwise, on a non-character
		// or with a search distance of 0, Z is held as before to keep the ground height.
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
					// Capsule collision is still disabled, having been set to none on entry, and the ragdoll
					// mesh is still present, so this actor is explicitly excluded. The channel matches the one
					// the capsule walks on.
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
						// No ground within the search distance, meaning it was dropped in mid-air, so it
						// continues falling from the anchor height.
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
			// Restore the movement mode recorded on entry. Restoring MOVE_None, which happens when it was
			// entered with movement already disabled, would leave the target uncontrollable, so that one case
			// is rescued to walking; and being dropped in mid-air, meaning the trace above failed, continues
			// falling rather than returning to a grounded mode. Returning to walking would fall on the first
			// tick anyway once the ground search failed, but stating it explicitly removes one frame's
			// attempt at a ground snap. Saved flying and swimming modes are respected as they are.
			EMovementMode RestoreMode = (SavedMovementMode == MOVE_None) ? MOVE_Walking : SavedMovementMode.GetValue();
			if (bRecoveredAirborne && (RestoreMode == MOVE_Walking || RestoreMode == MOVE_NavWalking))
			{
				RestoreMode = MOVE_Falling;
			}
			Character->GetCharacterMovement()->SetMovementMode(RestoreMode, SavedCustomMovementMode);
		}
	}

	// The camera blends back to the pawn only after the capsule realignment and the movement recovery are
	// complete, so its destination, the pawn camera, is already at its final position and the blend does not
	// chase it.
	EndRagdollCameraFollow();

	bRagdolled = false;
	bPartial = false;
	bRagdollWasAutoTriggered = false;
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] RopeRagdollResponse: recovered from the ragdoll."), *GetNameSafe(GetOwner()));
}

void URopeRagdollResponseComponent::BeginRagdollCameraFollow()
{
	// Only a full ragdoll on the pawn the player is watching. Every failure condition is a quiet no-op: the
	// camera is a convenience and must not affect whether the ragdoll itself, meaning the physics
	// transition, succeeded.
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
	// If another view target, such as a cinematic, is already active, it is not taken over.
	if (PC->GetViewTarget() != Pawn)
	{
		return;
	}

	// The follow reference is the same anchor bone the capsule realignment uses, falling back to the mesh
	// origin when the skeleton lacks it.
	const bool bHasAnchorBone =
		!RecoverAnchorBoneName.IsNone() && Mesh->GetBoneIndex(RecoverAnchorBoneName) != INDEX_NONE;
	const FVector AnchorPos =
		bHasAnchorBone ? Mesh->GetSocketLocation(RecoverAnchorBoneName) : Mesh->GetComponentLocation();

	// Spawning at the current point of view and switching with no blend makes it seamless on screen.
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
	// The mesh is a prerequisite so this reads the bones after their final pose, including the physics
	// blend, which prevents a one-frame lag from showing as jitter.
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
	// Blend back to the pawn only while we are still the view target, respecting anything, such as a
	// cinematic, that took it in the meantime. If the owner is dying there is no destination, so the camera
	// is left alone and only given a lifetime, leaving it to the camera manager's fallback.
	if (PC && bWorldAlive && PC->GetViewTarget() == Cam
		&& IsValid(Owner) && !Owner->IsActorBeingDestroyed())
	{
		PC->SetViewTargetWithBlend(Owner, FMath::Max(RecoverCameraBlendTime, 0.0f),
			VTBlend_Cubic);
	}
	// The original view target has to survive until the blend finishes, so it is given a lifetime rather
	// than being destroyed immediately.
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
	// If a precondition of following breaks, through a path other than recovery such as a lost mesh, a
	// destroyed controller or the view target being taken, it cleans up and stops.
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
	// The position follows with a one-way lag, with no feedback from the camera back to the bone, which
	// makes a runaway impossible, while the view always points at the ragdoll.
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
	// Prevent the state where no component at all can raise overlap events: a full ragdoll disables the
	// capsule, and an ACharacter's mesh has overlap events disabled to begin with, which is the engine
	// default. Switching the collision profile does not touch that flag, so it is enabled explicitly here;
	// without it a trigger volume cannot see the ragdoll.
	if (bGenerateOverlapEventsWhileRagdolled)
	{
		Mesh->SetGenerateOverlapEvents(true);
	}
}

void URopeRagdollResponseComponent::ApplyRagdollCCD(USkeletalMeshComponent* Mesh, bool bEnable)
{
	// A thin floor, such as the engine's default plane with zero collision thickness, is passed through when
	// a ragdoll body crosses the surface within one step, because no contact is generated. Continuous
	// collision is therefore enabled on every body for the duration of the ragdoll, sweeping between steps.
	// Restoring sets them all to false, which accepts the limitation that a body the physics asset had
	// enabled at authoring time is disabled too: after recovery the simulation is off so it has no effect,
	// and re-entering through this component enables them again.
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

	// The movement mode is saved as well, so it can be restored on recovery: a target wrapped while flying,
	// swimming or in a custom mode must not walk out of the ragdoll.
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
// Console commands for development checks, applied to every actor in the world carrying this component.
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
				TEXT("No actor has a RopeRagdollResponseComponent - add the component to the target character Blueprint, or to the level instance."));
		}
	}

	static FAutoConsoleCommandWithWorldAndArgs GRagdollCmd(
		TEXT("Rope.Ragdoll"),
		TEXT("Toggles the ragdoll response. With no argument it toggles a full ragdoll, recovering if already limp. With a bone name it goes partially limp below that bone."),
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
		TEXT("Ragdoll response: recover to animation. A full ragdoll produces a pose pop; while wrapped, this checks the rope follows with no velocity."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
		{
			ForEach(World, [](URopeRagdollResponseComponent& Comp)
			{
				Comp.RecoverFromRagdoll();
			});
		}));

	static FAutoConsoleCommandWithWorldAndArgs GDestroyCmd(
		TEXT("Rope.Ragdoll.Destroy"),
		TEXT("Ragdoll response: destroy the target actor, which checks the rope's weak-mesh release path when the target dies mid-wrap."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
		{
			ForEach(World, [](URopeRagdollResponseComponent& Comp)
			{
				if (AActor* Owner = Comp.GetOwner())
				{
					UE_LOG(LogDynamicRope, Log, TEXT("[%s] Destroyed, to test the lost-target release."), *GetNameSafe(Owner));
					Owner->Destroy();
				}
			});
		}));
}
#endif // !UE_BUILD_SHIPPING
