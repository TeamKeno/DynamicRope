// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Demo/RopeDemoHelicopter.h"
#include "RopeComponent.h"
#include "Logic/RopeAimTargeting.h"
#include "Core/RopeLifecycleTypes.h"
#include "Core/RopeThrowTypes.h"
#include "DynamicRopeLog.h"

#include "Camera/CameraComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/SphereComponent.h"
#include "Components/StaticMeshComponent.h"

ARopeDemoHelicopter::ARopeDemoHelicopter()
{
	// Hovering, flying, the rotor and the grab retry all need a tick.
	PrimaryActorTick.bCanEverTick = true;

	Base = CreateDefaultSubobject<USceneComponent>(TEXT("Base"));
	SetRootComponent(Base);

	// The mesh is assigned in the level or a Blueprint, since a plugin must not reference /Game. The rotor is optional, leaving an empty component if there is none.
	BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
	BodyMesh->SetupAttachment(Base);
	RotorMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RotorMesh"));
	RotorMesh->SetupAttachment(BodyMesh);

	// The cable hangs from a point under the airframe, whose position is adjusted in the details panel.
	RopeAttach = CreateDefaultSubobject<USceneComponent>(TEXT("RopeAttach"));
	RopeAttach->SetupAttachment(Base);
	RopeAttach->SetRelativeLocation(FVector(0.0f, 0.0f, -100.0f));

	Rope = CreateDefaultSubobject<URopeComponent>(TEXT("Rope"));
	Rope->SetupAttachment(RopeAttach);
	// The demo has to wrap the nominated passenger bone without fail, so it is fixed to GuaranteedWrap and waits in Loaded from BeginPlay.
	Rope->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
	Rope->RopeLength = 700.0f;

	// The grab volume is for detection alone, being query-only and overlapping pawns alone, so it fails the collection
	// eligibility filter, which requires physical collision and a blocking response to PhysicsBody, and is never
	// gathered as a rope collider. That stops the rope being pushed aside by an invisible sphere.
	GrabVolume = CreateDefaultSubobject<USphereComponent>(TEXT("GrabVolume"));
	GrabVolume->SetupAttachment(Base);
	GrabVolume->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	GrabVolume->SetCollisionObjectType(ECC_WorldDynamic);
	GrabVolume->SetCollisionResponseToAllChannels(ECR_Ignore);
	GrabVolume->SetCollisionResponseToChannel(ECC_Pawn, ECR_Overlap);
	GrabVolume->SetGenerateOverlapEvents(true);

	// The ride camera. Only used when bSwitchPlayerViewTarget switches a grabbed player's view here.
	// The default looks down at the hanging cable from behind the body; reframe it in the Blueprint
	// or level.
	ViewCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("ViewCamera"));
	ViewCamera->SetupAttachment(Base);
	ViewCamera->SetRelativeLocation(FVector(-600.0f, 0.0f, 150.0f));
	ViewCamera->SetRelativeRotation(FRotator(-40.0f, 0.0f, 0.0f));
}

void ARopeDemoHelicopter::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	// Repositions the grab volume to match the knobs, so a value edited in the editor takes effect immediately.
	if (GrabVolume)
	{
		GrabVolume->SetRelativeLocation(FVector(0.0f, 0.0f, -GrabZoneDrop));
		GrabVolume->SetSphereRadius(GrabZoneRadius);
	}
}

void ARopeDemoHelicopter::BeginPlay()
{
	Super::BeginPlay();

	IdleAnchor = GetActorLocation();
	NavPos = IdleAnchor;
	NavYaw = static_cast<float>(GetActorRotation().Yaw);

	if (GrabVolume)
	{
		GrabVolume->OnComponentBeginOverlap.AddDynamic(this, &ARopeDemoHelicopter::HandleGrabZoneBeginOverlap);
	}
	if (!Rope)
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] demo helicopter has no rope component — it will never grab."),
			*GetName());
	}
}

void ARopeDemoHelicopter::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// An instant hand-back, so a destroyed or streamed-out helicopter does not strand the view on a
	// dying actor. The switcher itself is a no-op during world teardown.
	ViewSwitcher.Deactivate(0.0f);

	Super::EndPlay(EndPlayReason);
}

void ARopeDemoHelicopter::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// The rotor always turns, regardless of state. Revolutions per minute become degrees per second by multiplying by six.
	if (RotorMesh && RotorRPM > 0.0f)
	{
		RotorMesh->AddLocalRotation(FRotator(0.0f, RotorRPM * 6.0f * DeltaSeconds, 0.0f));
	}
	BobTime += DeltaSeconds;

	switch (State)
	{
	case EState::Idle:
		// Hovering at home, or at the last place it stopped.
		MoveTowards(IdleAnchor, DeltaSeconds);
		break;

	case EState::Grabbing:
	{
		if (!CarryTarget.IsValid() || !ResolveTargetMesh())
		{
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] grab target lost — cancelling."), *GetName());
			CancelGrab();
			break;
		}
		if (GrabTimeout > 0.0f)
		{
			GrabElapsed += DeltaSeconds;
			if (GrabElapsed >= GrabTimeout)
			{
				UE_LOG(LogDynamicRope, Log, TEXT("[%s] grab timed out (%.1fs) — cancelling."), *GetName(), GrabTimeout);
				CancelGrab();
				break;
			}
		}
		if (Rope && Rope->GetPhase() == ERopePhase::Wrapped)
		{
			// The wrap took, so transport begins.
			State = EState::Carrying;
			WaypointIndex = 0;
			bCarryBroadcast = true;
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] passenger grabbed: %s (%s -> %s)."), *GetName(),
				*GetNameSafe(CarryTarget.Get()), *GrabBone.ToString(), *Rope->GetWrappedBoneName().ToString());
			OnCarryStateChanged.Broadcast(this, true);
			break;
		}
		// It did not take, so it fires again periodically, including the load edge, on the same rhythm as the snare.
		GrabRetryRemaining -= DeltaSeconds;
		if (GrabRetryRemaining <= 0.0f)
		{
			FireRopeAtTarget();
			GrabRetryRemaining = GrabRetryPeriod;
		}
		MoveTowards(IdleAnchor, DeltaSeconds);
		break;
	}

	case EState::Carrying:
	{
		if (!Rope || Rope->GetPhase() != ERopePhase::Wrapped)
		{
			// An external release, a cut or the target being destroyed, so it ends the transport and returns.
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] carried passenger lost mid-flight."), *GetName());
			RecallRope();
			if (bCarryBroadcast)
			{
				bCarryBroadcast = false;
				OnCarryStateChanged.Broadcast(this, false);
			}
			CarryTarget.Reset();
			State = bReturnHomeAfterRelease ? EState::Returning : EState::Idle;
			if (!bReturnHomeAfterRelease)
			{
				IdleAnchor = NavPos;
			}
			break;
		}

		// Reels in to the target length, optionally with an active pull as well.
		const float MaxLength = FMath::Max(Rope->RopeLength, Rope->MinRopeLength);
		const float TargetLength = FMath::Clamp(CarryRopeLength, Rope->MinRopeLength, MaxLength);
		const float CurrentLength = Rope->GetCurrentRopeLength();
		Rope->SetReelRate(CurrentLength > TargetLength + LiftTolerance ? LiftReelSpeed : 0.0f);
		Rope->SetActivePull(CarryPullForce);

		// It hovers in place while reeling in, and tours the waypoints once the load is all the way up.
		const bool bLifted = CurrentLength <= TargetLength + LiftTolerance;
		if (!bLifted || !Waypoints.IsValidIndex(WaypointIndex) || !Waypoints[WaypointIndex])
		{
		// Either there are no waypoints, meaning a hovering crane, or it is still reeling in, so it holds position.
			MoveTowards(NavPos, DeltaSeconds);
			break;
		}
		const float Remaining = MoveTowards(Waypoints[WaypointIndex]->GetActorLocation(), DeltaSeconds);
		if (Remaining <= WaypointTolerance)
		{
			++WaypointIndex;
			if (WaypointIndex >= Waypoints.Num() && bReleaseAtLastWaypoint)
			{
				ReleaseCarried();
			}
		}
		break;
	}

	case EState::Returning:
		if (MoveTowards(IdleAnchor, DeltaSeconds) <= WaypointTolerance)
		{
			State = EState::Idle;
		}
		break;
	}
}

bool ARopeDemoHelicopter::Grab(AActor* Passenger)
{
	if (State != EState::Idle || !Rope)
	{
		return false;
	}
	if (!Passenger || Passenger == this
		|| !Passenger->FindComponentByClass<USkeletalMeshComponent>())
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] grab rejected: '%s' has no skeletal mesh."),
			*GetName(), *GetNameSafe(Passenger));
		return false;
	}
	CarryTarget = Passenger;
	State = EState::Grabbing;
	GrabRetryRemaining = 0.0f; // Fire for the first time on the very next tick.
	GrabElapsed = 0.0f;
	// The ride camera engages the moment the grab starts, so the whole cable drop is on screen. An AI
	// passenger is a quiet no-op inside the switcher.
	if (bSwitchPlayerViewTarget)
	{
		ViewSwitcher.Activate(this, Passenger, ViewBlendInTime);
	}
	return true;
}

void ARopeDemoHelicopter::CancelGrab()
{
	if (State != EState::Grabbing)
	{
		return;
	}
	RecallRope();
	CarryTarget.Reset();
	State = EState::Idle;
}

void ARopeDemoHelicopter::ReleaseCarried()
{
	if (State != EState::Carrying)
	{
		return;
	}
	RecallRope();
	if (bCarryBroadcast)
	{
		bCarryBroadcast = false;
		OnCarryStateChanged.Broadcast(this, false);
	}
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] passenger released: %s."), *GetName(), *GetNameSafe(CarryTarget.Get()));
	CarryTarget.Reset();
	State = bReturnHomeAfterRelease ? EState::Returning : EState::Idle;
	if (!bReturnHomeAfterRelease)
	{
	// This position becomes the new standby point, where it waits for the next passenger.
		IdleAnchor = NavPos;
	}
}

bool ARopeDemoHelicopter::IsCarrying() const
{
	return State == EState::Carrying;
}

void ARopeDemoHelicopter::RecallRope()
{
	// Every path out of a grab or a carry recalls the cable, so the view is handed back here: cancel,
	// timeout, drop-off and mid-flight loss all pass through.
	ViewSwitcher.Deactivate(ViewBlendOutTime);

	if (!Rope)
	{
		return;
	}
	Rope->SetReelRate(0.0f);
	Rope->SetActivePull(0.0f);
	// Discards the firing queue that has not gone out yet and recovers from every phase, including in flight and mid
	// grab, since ReleaseWrapAs is the gate and the rest is a no-op.
	// The same hygiene the snare needs: without it, the cable finishes flying out and wraps after the cancellation.
	Rope->CancelQueuedGuaranteedAimThrow();
	Rope->ReleaseWrap();
	// Unwinds by as much as was reeled in, so the next grab starts from the same length.
	Rope->SetRopeLength(Rope->RopeLength);
}

USkeletalMeshComponent* ARopeDemoHelicopter::ResolveTargetMesh() const
{
	AActor* Target = CarryTarget.Get();
	return Target ? Target->FindComponentByClass<USkeletalMeshComponent>() : nullptr;
}

bool ARopeDemoHelicopter::FireRopeAtTarget()
{
	USkeletalMeshComponent* Mesh = ResolveTargetMesh();
	if (!Rope || !Mesh)
	{
		return false;
	}
	if (Mesh->GetBoneIndex(GrabBone) == INDEX_NONE)
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] grab bone '%s' not found on '%s' — cancelling."),
			*GetName(), *GrabBone.ToString(), *Mesh->GetName());
		return false;
	}

	// GuaranteedWrap can be thrown from Loaded alone, so otherwise it only loads and fires on the next retry, which keeps the load edge stable.
	if (Rope->GetPhase() != ERopePhase::Loaded)
	{
		Rope->EnterLoaded();
		return false;
	}

	const FVector Origin = Rope->GetComponentLocation();
	const FVector BoneWorld = Mesh->GetSocketLocation(GrabBone);
	const FVector ToBone = BoneWorld - Origin;
	const float Dist = static_cast<float>(ToBone.Size());
	if (Dist < KINDA_SMALL_NUMBER)
	{
		return false;
	}
	const FVector AimDir = ToBone / Dist;

	// A minimal copy of the wielder's BuildAimRayThrowRequest, aiming at a fixed bone with no input, under the same contract as the snare's FireSnareRopeFor.
	FRopeAimRayThrowRequest Request;
	FRopeThrowContext& Ctx = Request.BaseContext;
	Ctx.Origin = Origin;
	Ctx.FrameForward = AimDir;
	Ctx.FrameUp = FVector::UpVector;
	Ctx.FrameRight = FVector::CrossProduct(AimDir, FVector::UpVector).GetSafeNormal();
	if (Ctx.FrameRight.IsNearlyZero())
	{
	// A vertical drop aim, with the aim direction parallel to the up vector. A helicopter shooting straight down is the norm, so this degenerate case is the usual path.
		Ctx.FrameRight = FVector::RightVector;
	}
	Ctx.FrameMode = ERopeThrowFrameMode::Custom;
	Ctx.bAimRayEvaluated = true;

	Request.RayOrigin = Origin;
	Request.RayDirection = AimDir;
	Request.ReachOrigin = Origin;
	Request.ReachLength = Dist + 200.0f; // Reach margin, so the bone is certainly included.
	Request.RayLength = FRopeAimTargeting::ResolveRayLengthForReach(
		Request.RayOrigin, Request.RayDirection, Request.ReachOrigin, Request.ReachLength);
	Request.QueryRadius = 0.0f;
	Request.SweepStep = 2.0f;

	return Rope->QueueGuaranteedAimThrow(Request, /*bExecuteWhenReady*/ true);
}

float ARopeDemoHelicopter::MoveTowards(const FVector& Dest, float DeltaSeconds)
{
	const FVector Delta = Dest - NavPos;
	const float Remaining = static_cast<float>(Delta.Size());
	if (Remaining > KINDA_SMALL_NUMBER)
	{
		NavPos = FMath::VInterpConstantTo(NavPos, Dest, DeltaSeconds, FlySpeed);
		if (bFaceTravelDirection && Remaining > WaypointTolerance)
		{
			const float DesiredYaw = static_cast<float>(Delta.Rotation().Yaw);
			NavYaw = FMath::FixedTurn(NavYaw, DesiredYaw, TurnRateDeg * DeltaSeconds);
		}
	}
	ApplyPose();
	return Remaining;
}

void ARopeDemoHelicopter::ApplyPose()
{
	// The sway is applied to the presentation alone and is never mixed into the navigation position, so that waypoint arrival does not wobble with it.
	FVector Shown = NavPos;
	if (HoverBobAmplitude > 0.0f && HoverBobPeriod > KINDA_SMALL_NUMBER)
	{
		Shown.Z += HoverBobAmplitude * FMath::Sin(BobTime * (2.0f * UE_PI / HoverBobPeriod));
	}
	SetActorLocationAndRotation(Shown, FRotator(0.0f, NavYaw, 0.0f));
}

void ARopeDemoHelicopter::HandleGrabZoneBeginOverlap(UPrimitiveComponent* /*OverlappedComponent*/,
	AActor* OtherActor, UPrimitiveComponent* /*OtherComp*/, int32 /*OtherBodyIndex*/, bool /*bFromSweep*/,
	const FHitResult& /*SweepResult*/)
{
	if (!bAutoGrab || State != EState::Idle)
	{
		return;
	}
	Grab(OtherActor);
}
