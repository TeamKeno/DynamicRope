// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Demo/RopeDemoElevator.h"
#include "Demo/RopeDemoPressurePlate.h"
#include "RopeComponent.h"
#include "Logic/RopeAimTargeting.h"
#include "Core/RopeLifecycleTypes.h"
#include "Core/RopeThrowTypes.h"
#include "DynamicRopeLog.h"

#include "Components/SceneComponent.h"
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

	// The visual winch drum: a cube above the platform's centre that spins while the elevator travels.
	// A cube rather than a cylinder, since a featureless cylinder spinning about its own axis shows no
	// motion. It keeps its own scale: inheriting the platform's non-uniform scale would both squash it
	// and shear it as it rotates.
	Winch = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Winch"));
	if (Winch)
	{
		Winch->SetupAttachment(Platform);
		Winch->SetUsingAbsoluteScale(true);
		Winch->SetRelativeScale3D(FVector(0.5f, 0.5f, 0.5f));
		// The relative location is still scaled by the parent's Z of 0.2, so 200 sits the drum's centre 40 cm above the platform's centre in world space, clear of the 10 cm top face.
		Winch->SetRelativeLocation(FVector(0.0f, 0.0f, 200.0f));
		Winch->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		if (CubeMesh.Succeeded())
		{
			Winch->SetStaticMesh(CubeMesh.Object);
		}
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

	//~ 4) The winch drum visual: it spins while any cable is still reeling, forward when ascending
	//     and backward when descending, and stands still on arrival.
	if (Winch && !bAllArrived && WinchSpinSpeed > 0.0f)
	{
		const FVector Axis = WinchSpinAxis.GetSafeNormal();
		if (!Axis.IsNearlyZero())
		{
			float Direction = bTargetTop ? 1.0f : -1.0f;
			if (bReverseWinchSpin)
			{
				Direction = -Direction;
			}
			Winch->AddLocalRotation(FQuat(Axis, FMath::DegreesToRadians(WinchSpinSpeed * Direction * DeltaSeconds)));
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

void ARopeDemoElevator::AssignAnchorSockets()
{
	AssignedAnchorSockets.Init(NAME_None, Ropes.Num());
	if (!AnchorTarget)
	{
		return;
	}

	//~ Explicit listing: index-matched to the cables, taken as authored.
	if (AnchorSocketNames.Num() > 0)
	{
		for (int32 Index = 0; Index < Ropes.Num() && Index < AnchorSocketNames.Num(); ++Index)
		{
			AssignedAnchorSockets[Index] = AnchorSocketNames[Index];
		}
		return;
	}

	//~ Auto-discovery: every true socket on the anchor's components. Bones are excluded, since on a
	//  skeletal anchor they would flood the list with entries the author never meant as aim points.
	TArray<FName> SocketNames;
	TArray<FVector> SocketLocations;
	TInlineComponentArray<USceneComponent*> Components(AnchorTarget);
	for (USceneComponent* Component : Components)
	{
		TArray<FComponentSocketDescription> Sockets;
		Component->QuerySupportedSockets(Sockets);
		for (const FComponentSocketDescription& Socket : Sockets)
		{
			if (Socket.Type == EComponentSocketType::Socket)
			{
				SocketNames.Add(Socket.Name);
				SocketLocations.Add(Component->GetSocketLocation(Socket.Name));
			}
		}
	}

	// Greedy nearest-unclaimed matching, in cable order: each corner takes the socket closest to it,
	// which keeps the four cables spread instead of crossing, and is deterministic across refires.
	TArray<bool> Claimed;
	Claimed.Init(false, SocketNames.Num());
	for (int32 Index = 0; Index < Ropes.Num(); ++Index)
	{
		const FVector Origin = Ropes[Index] ? Ropes[Index]->GetComponentLocation() : GetActorLocation();
		int32 Best = INDEX_NONE;
		float BestDistSq = TNumericLimits<float>::Max();
		for (int32 SocketIndex = 0; SocketIndex < SocketNames.Num(); ++SocketIndex)
		{
			const float DistSq = float(FVector::DistSquared(Origin, SocketLocations[SocketIndex]));
			if (!Claimed[SocketIndex] && DistSq < BestDistSq)
			{
				BestDistSq = DistSq;
				Best = SocketIndex;
			}
		}
		if (Best != INDEX_NONE)
		{
			AssignedAnchorSockets[Index] = SocketNames[Best];
			Claimed[Best] = true;
		}
	}
}

FVector ARopeDemoElevator::ResolveAnchorAimWorld(int32 CableIndex) const
{
	if (!AnchorTarget)
	{
		return GetActorLocation();
	}

	// The cable's assigned socket steers the aim ray; the wrap still resolves onto the wrappable bone
	// or SDF surface that ray sweeps.
	if (AssignedAnchorSockets.IsValidIndex(CableIndex) && !AssignedAnchorSockets[CableIndex].IsNone())
	{
		const FName SocketName = AssignedAnchorSockets[CableIndex];
		TInlineComponentArray<USceneComponent*> Components(AnchorTarget);
		for (USceneComponent* Component : Components)
		{
			if (Component->DoesSocketExist(SocketName))
			{
				return Component->GetSocketLocation(SocketName);
			}
		}
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] anchor socket '%s' for cable %d not found on %s — aiming at the actor's location instead."),
			*GetName(), *SocketName.ToString(), CableIndex, *AnchorTarget->GetName());
	}

	// The fallback: the anchor actor's position, so the swept aim ray sweeps that direction for a wrappable bone and locks onto it.
	return AnchorTarget->GetActorLocation();
}

void ARopeDemoElevator::FireGrapples()
{
	// Reassigned every round: the elevator or a movable anchor may have shifted since the last
	// attempt, and a refire after losing one cable re-resolves against the current pose.
	AssignAnchorSockets();

	for (int32 Index = 0; Index < Ropes.Num(); ++Index)
	{
		URopeComponent* Cable = Ropes[Index];
		if (Cable && Cable->GetPhase() != ERopePhase::Wrapped)
		{
			FireGrappleFor(Cable, Index);
		}
	}
}

bool ARopeDemoElevator::FireGrappleFor(URopeComponent* InRope, int32 CableIndex)
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
	const FVector AnchorWorld = ResolveAnchorAimWorld(CableIndex);
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
