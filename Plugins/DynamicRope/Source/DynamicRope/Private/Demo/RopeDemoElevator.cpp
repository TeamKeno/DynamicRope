// Copyright Epic Games, Inc. All Rights Reserved.

#include "Demo/RopeDemoElevator.h"
#include "Demo/RopeDemoPressurePlate.h"
#include "RopeComponent.h"
#include "Logic/RopeAimTargeting.h"
#include "Core/RopeLifecycleTypes.h"
#include "Core/RopeThrowTypes.h"
#include "DynamicRopeLog.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "PhysicsEngine/BodyInstance.h"
#include "UObject/ConstructorHelpers.h"

ARopeDemoElevator::ARopeDemoElevator()
{
	// Driving the lift and polling for the grapples to be established both need a tick.
	PrimaryActorTick.bCanEverTick = true;

	Platform = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Platform"));
	SetRootComponent(Platform);
	// A 200 x 200 x 20 cm plate, based on the engine's 100 cm cube. Its physics body is the receiver for climb-in traction, being a simulating body.
	Platform->SetRelativeScale3D(FVector(2.0f, 2.0f, 0.2f));
	Platform->SetMobility(EComponentMobility::Movable);
	Platform->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	Platform->SetCollisionObjectType(ECC_PhysicsBody);
	Platform->SetSimulatePhysics(true);

	// Engine primitive shapes alone, to avoid creating a content dependency, since a plugin must not reference /Game.
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (CubeMesh.Succeeded())
	{
		Platform->SetStaticMesh(CubeMesh.Object);
	}

	// The four corner cables, each of which wraps a ceiling anchor from the platform's top face at plus or minus 80 in X and Y and 10 cm up.
	const FVector Corners[NumRopes] = {
		FVector( 80.0f,  80.0f, 10.0f),
		FVector( 80.0f, -80.0f, 10.0f),
		FVector(-80.0f,  80.0f, 10.0f),
		FVector(-80.0f, -80.0f, 10.0f),
	};
	Ropes.Reserve(NumRopes);
	for (int32 Index = 0; Index < NumRopes; ++Index)
	{
		URopeComponent* Cable = CreateDefaultSubobject<URopeComponent>(*FString::Printf(TEXT("Rope%d"), Index));
		if (Cable)
		{
			Cable->SetupAttachment(Platform);
			Cable->SetRelativeLocation(Corners[Index]);
			// The demo has to wrap the ceiling anchors without fail, so it is fixed to GuaranteedWrap.
			Cable->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
			Ropes.Add(Cable);
		}
	}
}

void ARopeDemoElevator::BeginPlay()
{
	Super::BeginPlay();

	if (CallPlate)
	{
		CallPlate->OnPlatePressedChanged.AddDynamic(this, &ARopeDemoElevator::HandleCallPlateChanged);
	}

	if (!AnchorTarget)
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] demo elevator has no AnchorTarget — set a wrappable ceiling anchor (skeletal bone collider / SDF) or it will never move."),
			*GetName());
	}

	ApplyPlatformStability();

	// It starts on the lower floor, waiting on the ground. The grapples try to establish themselves from the first tick.
	bTargetTop = false;
	bArrivedBroadcast = false;
}

void ARopeDemoElevator::ApplyPlatformStability()
{
	if (!Platform)
	{
		return;
	}

	if (PlatformMass > 0.0f)
	{
		Platform->SetMassOverrideInKg(NAME_None, PlatformMass, /*bOverrideMass*/ true);
	}
	Platform->SetAngularDamping(PlatformAngularDamping);

	// Removing the pitch and roll degrees of freedom, which would let it capsize, keeps the platform level when a
	// character stands on one side, while preserving vertical physical movement for climb-in. Yaw is optional, through
	// bLockPlatformYaw. With every lock off, the six-degree-of-freedom constraint behaves exactly like free physics.
	if (FBodyInstance* Body = Platform->GetBodyInstance())
	{
		Body->bLockXRotation = bLockPlatformTilt;
		Body->bLockYRotation = bLockPlatformTilt;
		Body->bLockZRotation = bLockPlatformYaw;
		Body->SetDOFLock(EDOFMode::SixDOF);
	}
}

void ARopeDemoElevator::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (CallPlate)
	{
		CallPlate->OnPlatePressedChanged.RemoveDynamic(this, &ARopeDemoElevator::HandleCallPlateChanged);
	}

	Super::EndPlay(EndPlayReason);
}

void ARopeDemoElevator::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (Ropes.Num() == 0)
	{
		return;
	}

	//~ 1) Establishing the grapples: it fires again periodically until all four cables have wrapped a ceiling anchor.
	if (!bGrappleReady)
	{
		if (AreAllRopesWrapped())
		{
			bGrappleReady = true;
			bArrivedBroadcast = false;
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] elevator grapple established (%d cables)."), *GetName(), NumRopes);
			return;
		}

		if (AnchorTarget)
		{
			EstablishRetryRemaining -= DeltaSeconds;
			if (EstablishRetryRemaining <= 0.0f)
			{
				FireGrapples();
				EstablishRetryRemaining = 1.0f; // Retry at one-second intervals, leaving room for the gather and the load edge to settle.
			}
		}
		return;
	}

	//~ 2) If any cable has come loose, as when its target is lost, it returns to the establishing stage and refires that cable alone.
	if (!AreAllRopesWrapped())
	{
		bGrappleReady = false;
		EstablishRetryRemaining = 0.5f;
		for (URopeComponent* Cable : Ropes)
		{
			if (Cable)
			{
				Cable->SetReelRate(0.0f);
				Cable->SetActivePull(0.0f);
			}
		}
		return;
	}

	//~ 3) Driving the reel to the target floor. Going up reels in, with climb-in per cable, and going down reels out
	//     under gravity. Arrival requires every cable to have reached its target, so all four lengths converge together.
	bool bAllArrived = true;
	for (URopeComponent* Cable : Ropes)
	{
		if (!Cable)
		{
			continue;
		}
		const float Length = Cable->GetCurrentRopeLength();
		const float MinLength = Cable->MinRopeLength;
		const float MaxLength = FMath::Max(Cable->RopeLength, MinLength);

		if (bTargetTop)
		{
			if (Length > MinLength + ArrivalTolerance)
			{
				Cable->SetReelRate(AscendReelSpeed);
				if (ClimbForce > 0.0f)
				{
					Cable->SetActivePull(ClimbForce);
				}
				bAllArrived = false;
			}
			else
			{
				Cable->SetReelRate(0.0f);
				Cable->SetActivePull(0.0f);
			}
		}
		else
		{
			if (Length < MaxLength - ArrivalTolerance)
			{
				Cable->SetReelRate(-DescendReelSpeed);
				Cable->SetActivePull(0.0f);
				bAllArrived = false;
			}
			else
			{
				Cable->SetReelRate(0.0f);
				Cable->SetActivePull(0.0f);
			}
		}
	}

	if (bAllArrived && !bArrivedBroadcast)
	{
		bArrivedBroadcast = true;
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] elevator arrived at %s."), *GetName(), bTargetTop ? TEXT("top") : TEXT("bottom"));
		OnElevatorArrived.Broadcast(this, bTargetTop);
	}
}

void ARopeDemoElevator::SetTargetTop(bool bNewTargetTop)
{
	if (bTargetTop == bNewTargetTop)
	{
		return;
	}
	bTargetTop = bNewTargetTop;
	bArrivedBroadcast = false; // A new target, so arrival detection resumes.
}

void ARopeDemoElevator::HandleCallPlateChanged(ARopeDemoPressurePlate* /*Plate*/, bool bPressed)
{
	// The call button: pressed goes up, released goes down.
	SetTargetTop(bPressed);
}

bool ARopeDemoElevator::AreAllRopesWrapped() const
{
	if (Ropes.Num() == 0)
	{
		return false;
	}
	for (const URopeComponent* Cable : Ropes)
	{
		if (!Cable || Cable->GetPhase() != ERopePhase::Wrapped)
		{
			return false;
		}
	}
	return true;
}

FVector ARopeDemoElevator::ResolveAnchorAimWorld() const
{
	// Aims at the anchor actor's position, so the swept aim ray sweeps that direction for a wrappable bone and locks onto it.
	return AnchorTarget ? AnchorTarget->GetActorLocation() : GetActorLocation();
}

void ARopeDemoElevator::FireGrapples()
{
	for (URopeComponent* Cable : Ropes)
	{
		if (Cable && Cable->GetPhase() != ERopePhase::Wrapped)
		{
			FireGrappleFor(Cable);
		}
	}
}

bool ARopeDemoElevator::FireGrappleFor(URopeComponent* InRope)
{
	if (!InRope || !AnchorTarget)
	{
		return false;
	}

	// GuaranteedWrap can be thrown from Loaded alone, so otherwise it only loads and fires on the next attempt, which keeps the load edge stable.
	if (InRope->GetPhase() != ERopePhase::Loaded)
	{
		InRope->EnterLoaded();
		return false;
	}

	const FVector Origin = InRope->GetComponentLocation();
	const FVector AnchorWorld = ResolveAnchorAimWorld();
	const FVector ToAnchor = AnchorWorld - Origin;
	const float Dist = ToAnchor.Size();
	if (Dist < KINDA_SMALL_NUMBER)
	{
		return false;
	}
	const FVector AimDir = ToAnchor / Dist;

	// A minimal copy of the wielder's BuildAimRayThrowRequest, aiming at the anchor with no input.
	FRopeAimRayThrowRequest Request;
	FRopeThrowContext& Ctx = Request.BaseContext;
	Ctx.Origin = Origin;
	Ctx.FrameForward = AimDir;
	Ctx.FrameUp = FVector::UpVector;
	Ctx.FrameRight = FVector::CrossProduct(AimDir, FVector::UpVector).GetSafeNormal();
	if (Ctx.FrameRight.IsNearlyZero())
	{
		Ctx.FrameRight = FVector::RightVector; // The fallback for the degenerate case of a vertical aim, where the aim direction is parallel to the up vector.
	}
	Ctx.FrameMode = ERopeThrowFrameMode::Custom;
	Ctx.bAimRayEvaluated = true; // Marks this as a context produced by an aim ray, which is the preview builder's contract.

	Request.RayOrigin = Origin;
	Request.RayDirection = AimDir;
	Request.ReachOrigin = Origin;
	Request.ReachLength = Dist + 200.0f; // Reach margin, so the anchor is certainly included.
	Request.RayLength = FRopeAimTargeting::ResolveRayLengthForReach(
		Request.RayOrigin, Request.RayDirection, Request.ReachOrigin, Request.ReachLength);
	Request.QueryRadius = 0.0f;
	Request.SweepStep = 2.0f;

	// Executed immediately after a normal gather, with no montage.
	return InRope->QueueGuaranteedAimThrow(Request, /*bExecuteWhenReady*/ true);
}
