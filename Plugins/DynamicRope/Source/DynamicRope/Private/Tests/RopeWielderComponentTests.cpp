// Copyright Epic Games, Inc. All Rights Reserved.
//
// Regression tests for URopeWielderComponent's input, animation and preview lifetimes.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "RopeWielderComponentTestTypes.h"

#include "Gameplay/AnimNotifyState_RopePull.h"
#include "Gameplay/RopeWielderComponent.h"
#include "RopeComponent.h"
#include "RopeTestHelpers.h"

#include "Animation/AnimMontage.h"
#include "Components/SceneComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/SphereComponent.h"
#include "Engine/Engine.h"
#include "Engine/LocalPlayer.h"
#include "Engine/World.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "EnhancedPlayerInput.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "InputAction.h"
#include "InputMappingContext.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "Tests/AutomationCommon.h"

struct FRopeWielderComponentTestSeam
{
	static void SetAirControlBoosted(URopeWielderComponent& Wielder, bool bBoosted)
	{
		Wielder.bAirControlBoosted = bBoosted;
	}

	static bool ComputeDesiredTickEnabled(const URopeWielderComponent& Wielder)
	{
		return Wielder.ComputeDesiredTickEnabled();
	}

	static FVector ComputeHandSwingVelocityWorld(const FVector& Prev, const FVector& Cur, float Dt,
		const FTransform& Xform, float MaxSpeed)
	{
		return URopeWielderComponent::ComputeHandSwingVelocityWorld(Prev, Cur, Dt, Xform, MaxSpeed);
	}

	static void ForceWrappedTaut(URopeComponent& Rope)
	{
		Rope.Phase = ERopePhase::Wrapped;
		Rope.PullDrive.bPullTaut = true;
	}

	static void SetConstraintTension(URopeComponent& Rope, float Tension)
	{
		Rope.LengthConstraintState.LastLambdaDt = 1.0f / 60.0f;
		Rope.LengthConstraintState.LastLambda =
			FMath::Max(Tension, 0.0f) * Rope.LengthConstraintState.LastLambdaDt;
	}

	static void SetVisualSegmentTension(URopeComponent& Rope, float Tension)
	{
		Rope.Sim.SegmentTension = { FMath::Max(Tension, 0.0f) };
	}

	static void SetPullEngaged(URopeWielderComponent& Wielder, bool bEngaged)
	{
		Wielder.SetPullEngaged(bEngaged, 0.0f);
	}

	static void UpdatePullEngage(URopeWielderComponent& Wielder)
	{
		Wielder.UpdatePullEngage();
	}

	static float GetActivePullForce(const URopeComponent& Rope)
	{
		return Rope.PullDrive.ActivePullForce;
	}

	static float GetReelRate(const URopeComponent& Rope)
	{
		return Rope.ReelRate;
	}

	static void HandleUnpossess(URopeWielderComponent& Wielder)
	{
		Wielder.HandlePawnControllerChanged(nullptr, nullptr, nullptr);
	}

	static void SetMappedInputSubsystem(URopeWielderComponent& Wielder,
		UEnhancedInputLocalPlayerSubsystem* Subsystem)
	{
		Wielder.MappedInputSubsystem = Subsystem;
		Wielder.MappedInputContext = Wielder.MappingContext;
	}

	static void RemoveMappingContext(URopeWielderComponent& Wielder)
	{
		Wielder.RemoveMappingContext();
	}

	static void SetAimFrameContext(URopeWielderComponent& Wielder, const FRopeThrowContext& Context)
	{
		Wielder.AimRayFrameThrowContext = Context;
		Wielder.AimRayFrameContextStamp = GFrameCounter;
		Wielder.bHasAimRayFrameThrowContext = true;
	}

	static void UpdateThrowPreview(URopeWielderComponent& Wielder)
	{
		Wielder.UpdateThrowPreview();
	}

	static int32 GetPreparedPreviewBuildCount(const URopeWielderComponent& Wielder)
	{
		return Wielder.TestPreparedPreviewBuildCount;
	}

	static void ForceStaticSelfWrap(URopeComponent& Rope, const USceneComponent* WrappedComponent)
	{
		Rope.Phase = ERopePhase::Wrapped;
		Rope.PullDrive.LastTargetShare = 0.0f;
		Rope.WrapController.State.Mesh = WrappedComponent;
	}

	static bool IsWielderTetherActive(const URopeWielderComponent& Wielder)
	{
		return Wielder.IsWielderTetherActive();
	}

	static void ConfigureExternalHardLeash(
		URopeComponent& Rope, const USceneComponent* Target, int32 AnchorNode, float RopeLength)
	{
		Rope.Sim = RopeTest::MakeStraightRope(FMath::Max(AnchorNode + 1, 2), RopeLength);
		Rope.Sim.bStartPinned = true;
		Rope.Sim.StartPinPrev = Rope.Sim.Positions[0];
		Rope.Sim.StartPinTarget = Rope.Sim.Positions[0];
		Rope.Sim.InvMass[0] = 0.0f;
		Rope.HoldConfig.bEnforceWielderLengthConstraint = true;

		FRopeSurfaceAnchor Anchor;
		Anchor.NodeIndex = AnchorNode;
		Anchor.Mesh = Target;
		Anchor.LocalSurfacePosition = FVector::ZeroVector;
		Anchor.LocalNormal = FVector::UpVector;
		Rope.WrapController.State.Mesh = Target;
		Rope.WrapController.State.Anchors = { Anchor };
		Rope.Phase = ERopePhase::Wrapped;
		Rope.bWrappedMassMaskDirty = true;
	}

	static bool HasPhysicalTether(const URopeComponent& Rope)
	{
		return Rope.PhysicalTetherConstraint != nullptr && Rope.PhysicalTetherProxy != nullptr;
	}

	static FVector GetPhysicalTetherProxyWorld(const URopeComponent& Rope)
	{
		return Rope.PhysicalTetherProxy
			? Rope.PhysicalTetherProxy->GetComponentLocation()
			: FVector::ZeroVector;
	}

	static float GetPhysicalTetherLimit(const URopeComponent& Rope)
	{
		return Rope.PhysicalTetherLimit;
	}

	static ERopeLengthConstraintBackend GetLengthConstraintBackend(const URopeComponent& Rope)
	{
		return Rope.LengthConstraintState.Backend;
	}

	static bool IsPhysicalTetherSoft(const URopeComponent& Rope)
	{
		return Rope.PhysicalTetherConstraint &&
			Rope.PhysicalTetherConstraint->ConstraintInstance.ProfileInstance
				.LinearLimit.bSoftConstraint;
	}

	static bool IsPhysicalTetherLinearLocked(const URopeComponent& Rope)
	{
		if (!Rope.PhysicalTetherConstraint)
		{
			return false;
		}
		const FLinearConstraint& Limit =
			Rope.PhysicalTetherConstraint->ConstraintInstance
				.ProfileInstance.LinearLimit;
		return Limit.XMotion == LCM_Locked &&
			Limit.YMotion == LCM_Locked &&
			Limit.ZMotion == LCM_Locked;
	}

	static bool IsPhysicalTetherLinearLimited(const URopeComponent& Rope)
	{
		if (!Rope.PhysicalTetherConstraint)
		{
			return false;
		}
		const FLinearConstraint& Limit =
			Rope.PhysicalTetherConstraint->ConstraintInstance
				.ProfileInstance.LinearLimit;
		return Limit.XMotion == LCM_Limited &&
			Limit.YMotion == LCM_Limited &&
			Limit.ZMotion == LCM_Limited;
	}

	static void UpdatePhysicalTether(
		URopeComponent& Rope,
		UPrimitiveComponent* Target,
		const FVector& AnchorWorld,
		const FVector& ProxyWorld,
		float Limit,
		float DeltaTime)
	{
		Rope.UpdatePhysicalTether(
			Target,
			NAME_None,
			AnchorWorld,
			ProxyWorld,
			Limit,
			DeltaTime);
	}

	static float GetPhysicalTetherStiffness(const URopeComponent& Rope)
	{
		return Rope.PhysicalTetherConstraint
			? Rope.PhysicalTetherConstraint->ConstraintInstance.ProfileInstance
				.LinearLimit.Stiffness
			: 0.0f;
	}

	static float ComputeWielderReactionShare(
		const URopeComponent& Rope,
		const FRopeWielderMovementConstraint& Constraint)
	{
		return Rope.ComputeWielderLengthReactionShare(Constraint);
	}

	static float ComputeWielderPositionCorrectionShare(
		const URopeComponent& Rope,
		const FRopeWielderMovementConstraint& Constraint)
	{
		return Rope.ComputeWielderLengthPositionCorrectionShare(
			Constraint);
	}

	static void SeedPreviousLengthGeometry(
		URopeComponent& Rope,
		int32 AnchorNode,
		float MaterialLength,
		const FVector& AnchorWorld)
	{
		Rope.LengthConstraintState.PrevAnchorNode = AnchorNode;
		Rope.LengthConstraintState.PrevMaterialLength = MaterialLength;
		Rope.LengthConstraintState.PrevAnchorWorldPoint = AnchorWorld;
		Rope.LengthConstraintState.bPrevGeometryValid = true;
	}

	static float GetMaterialRopeLength(const URopeComponent& Rope)
	{
		return Rope.Sim.RopeLength;
	}

};

namespace
{
	USphereComponent* AddTestTargetRoot(
		AActor& Actor,
		const FVector& Location,
		bool bSimulatePhysics)
	{
		USphereComponent* Root = NewObject<USphereComponent>(&Actor);
		Actor.AddInstanceComponent(Root);
		Actor.SetRootComponent(Root);
		Root->InitSphereRadius(10.0f);
		Root->SetMobility(EComponentMobility::Movable);
		Root->RegisterComponent();
		Actor.SetActorLocation(Location);
		Root->SetCollisionEnabled(
			bSimulatePhysics
				? ECollisionEnabled::PhysicsOnly
				: ECollisionEnabled::NoCollision);
		Root->SetSimulatePhysics(bSimulatePhysics);
		Root->SetEnableGravity(false);
		return Root;
	}

	URopeWielderComponent* AddTestWielder(
		ACharacter& Character,
		URopeComponent& Rope)
	{
		URopeWielderComponent* Wielder =
			NewObject<URopeWielderComponent>(&Character);
		Character.AddInstanceComponent(Wielder);
		Wielder->Rope = &Rope;
		Wielder->bAutoBindInput = false;
		Wielder->bAttachOnBeginPlay = false;
		Wielder->bShowThrowPreview = false;
		Wielder->bAutoGroundExitOnUpwardPull = false;
		Wielder->bBoostAirControlWhileSwinging = false;
		Wielder->RegisterComponent();
		return Wielder;
	}

	UCharacterMovementComponent* ConfigureFlyingMovement(
		ACharacter& Character,
		const FVector& Velocity)
	{
		UCharacterMovementComponent* Movement =
			Character.GetCharacterMovement();
		Movement->GravityScale = 0.0f;
		Movement->BrakingDecelerationFlying = 0.0f;
		Movement->bUseSeparateBrakingFriction = true;
		Movement->BrakingFriction = 0.0f;
		Movement->bRunPhysicsWithNoController = true;
		Movement->SetMovementMode(MOVE_Flying);
		Movement->Velocity = Velocity;
		return Movement;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderPullArmingEnablesTickTest,
	"DynamicRope.Wielder.Pull.ArmingEnablesTick",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderPullArmingEnablesTickTest::RunTest(const FString& Parameters)
{
	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>();
	Wielder->bAutoGroundExitOnUpwardPull = false;
	Wielder->bBoostAirControlWhileSwinging = false;
	Wielder->SetComponentTickEnabled(false);

	TestFalse(TEXT("baseline tick is disabled"), Wielder->IsComponentTickEnabled());
	Wielder->StartPull();
	TestTrue(TEXT("arming Pull enables the tick that evaluates engagement"), Wielder->IsComponentTickEnabled());

	Wielder->StopPull();
	TestFalse(TEXT("disarming the only tick consumer disables tick again"), Wielder->IsComponentTickEnabled());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderActiveAirControlKeepsTickTest,
	"DynamicRope.Wielder.Movement.ActiveAirControlKeepsTick",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderActiveAirControlKeepsTickTest::RunTest(const FString& Parameters)
{
	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>();
	Wielder->bAutoGroundExitOnUpwardPull = false;
	Wielder->bBoostAirControlWhileSwinging = false;
	FRopeWielderComponentTestSeam::SetAirControlBoosted(*Wielder, true);

	TestTrue(TEXT("an applied AirControl override keeps tick alive until restoration"),
		FRopeWielderComponentTestSeam::ComputeDesiredTickEnabled(*Wielder));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderCancelledPullRejectsLateNotifyTest,
	"DynamicRope.Wielder.Pull.CancelledPullRejectsLateNotify",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderCancelledPullRejectsLateNotifyTest::RunTest(const FString& Parameters)
{
	AActor* Owner = NewObject<AActor>();
	URopeComponent* Rope = NewObject<URopeComponent>(Owner);
	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>(Owner);
	USkeletalMeshComponent* Mesh = NewObject<USkeletalMeshComponent>(Owner);
	Wielder->Rope = Rope;
	Wielder->AttachMesh = Mesh;

	Wielder->StartPull();
	FRopeWielderComponentTestSeam::SetPullEngaged(*Wielder, true);
	Wielder->StartPullNow(/*bIgnoreTautGate*/ true);
	Wielder->StopPull();
	TestTrue(TEXT("cancel clears Pull force"),
		FMath::IsNearlyZero(FRopeWielderComponentTestSeam::GetActivePullForce(*Rope)));

	UAnimNotifyState_RopePull* Notify = NewObject<UAnimNotifyState_RopePull>();
	Notify->NotifyBegin(Mesh, nullptr, 0.1f, FAnimNotifyEventReference());

	TestTrue(TEXT("a late NotifyBegin cannot restart a cancelled Pull"),
		FMath::IsNearlyZero(FRopeWielderComponentTestSeam::GetActivePullForce(*Rope)));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderPullMontageFailureDoesNotDeadLatchTest,
	"DynamicRope.Wielder.Pull.MontageFailureDoesNotLeaveDeadLatch",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderPullMontageFailureDoesNotDeadLatchTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>();
	Wielder->Rope = Rope;
	Wielder->PullMontage = NewObject<UAnimMontage>();
	FRopeWielderComponentTestSeam::ForceWrappedTaut(*Rope);
	Wielder->StartPull();

	AddExpectedError(TEXT("no AnimInstance to play PullMontage"),
		EAutomationExpectedErrorFlags::Contains, 1);
	FRopeWielderComponentTestSeam::UpdatePullEngage(*Wielder);

	const bool bDeadLatch = Wielder->IsPullEngaged() &&
		FRopeWielderComponentTestSeam::GetActivePullForce(*Rope) <= KINDA_SMALL_NUMBER;
	TestFalse(TEXT("failed montage must either roll back engagement or fall back to immediate Pull"), bDeadLatch);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderPullUsesConstraintTensionTest,
	"DynamicRope.Wielder.Pull.EngageUsesConstraintTension",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderPullUsesConstraintTensionTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>();
	Wielder->Rope = Rope;
	Wielder->PullEngageTension = 1000.0f;
	FRopeWielderComponentTestSeam::ForceWrappedTaut(*Rope);

	// A large visual XPBD diagnostic must not engage gameplay while authoritative load is low.
	FRopeWielderComponentTestSeam::SetVisualSegmentTension(*Rope, 1000000.0f);
	FRopeWielderComponentTestSeam::SetConstraintTension(*Rope, 500.0f);
	Wielder->StartPull();
	FRopeWielderComponentTestSeam::UpdatePullEngage(*Wielder);
	TestFalse(TEXT("visual SegmentTension cannot satisfy PullEngageTension"),
		Wielder->IsPullEngaged());

	FRopeWielderComponentTestSeam::SetConstraintTension(*Rope, 1500.0f);
	FRopeWielderComponentTestSeam::UpdatePullEngage(*Wielder);
	TestTrue(TEXT("authoritative constraint tension engages Pull"),
		Wielder->IsPullEngaged());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderDirectThrowUsesVirtualContextTest,
	"DynamicRope.Wielder.Throw.DirectUsesVirtualBuildContext",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderDirectThrowUsesVirtualContextTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	Rope->ResolveMode = ERopeWrapResolveMode::FullSimulation;
	URopeWielderBuildContextProbe* Wielder = NewObject<URopeWielderBuildContextProbe>();
	Wielder->Rope = Rope;

	Wielder->ThrowInDirection(FVector::ForwardVector);
	TestEqual(TEXT("direct actual throw passes through the public virtual BuildThrowContext hook"),
		Wielder->BuildContextCalls, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderAimThrowUsesVirtualContextTest,
	"DynamicRope.Wielder.Throw.AimRequestUsesVirtualBuildContext",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderAimThrowUsesVirtualContextTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	Rope->ResolveMode = ERopeWrapResolveMode::AssistedJudged;
	URopeWielderBuildContextProbe* Wielder = NewObject<URopeWielderBuildContextProbe>();
	Wielder->Rope = Rope;

	Wielder->ThrowInDirection(FVector::ForwardVector);
	TestEqual(TEXT("aim throw request passes through the public virtual BuildThrowContext hook"),
		Wielder->BuildContextCalls, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderUnpossessClearsHeldInputTest,
	"DynamicRope.Wielder.Input.UnpossessClearsHeldState",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderUnpossessClearsHeldInputTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>();
	Wielder->Rope = Rope;

	Wielder->StartPull();
	Wielder->StartPullNow(/*bIgnoreTautGate*/ true);
	Wielder->StartReelIn();
	FRopeWielderComponentTestSeam::HandleUnpossess(*Wielder);

	TestFalse(TEXT("unpossess disarms held Pull input"), Wielder->IsPullArmed());
	TestTrue(TEXT("unpossess clears active Pull force"),
		FMath::IsNearlyZero(FRopeWielderComponentTestSeam::GetActivePullForce(*Rope)));
	TestTrue(TEXT("unpossess clears held Reel rate"),
		FMath::IsNearlyZero(FRopeWielderComponentTestSeam::GetReelRate(*Rope)));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderRemovesAddedMappingContextTest,
	"DynamicRope.Wielder.Input.RemovesActuallyAddedMappingContext",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderRemovesAddedMappingContextTest::RunTest(const FString& Parameters)
{
	ULocalPlayer* LocalPlayer = NewObject<ULocalPlayer>(GEngine);
	APlayerController* PlayerController = NewObject<APlayerController>();
	PlayerController->PlayerInput = NewObject<UEnhancedPlayerInput>(PlayerController);
	LocalPlayer->SwitchController(PlayerController);
	UEnhancedInputLocalPlayerSubsystem* Subsystem =
		NewObject<UEnhancedInputLocalPlayerSubsystem>(LocalPlayer);

	UInputMappingContext* AddedContext = NewObject<UInputMappingContext>();
	UInputMappingContext* CurrentPropertyContext = NewObject<UInputMappingContext>();
	Subsystem->AddMappingContext(AddedContext, 0);
	Subsystem->AddMappingContext(CurrentPropertyContext, 1);
	if (!TestTrue(TEXT("fixture installed the originally added context"), Subsystem->HasMappingContext(AddedContext)) ||
		!TestTrue(TEXT("fixture installed the current-property context"), Subsystem->HasMappingContext(CurrentPropertyContext)))
	{
		return false;
	}

	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>();
	Wielder->MappingContext = AddedContext;
	FRopeWielderComponentTestSeam::SetMappedInputSubsystem(*Wielder, Subsystem);
	Wielder->MappingContext = CurrentPropertyContext;
	FRopeWielderComponentTestSeam::RemoveMappingContext(*Wielder);

	TestFalse(TEXT("removal targets the context that Wielder actually added"),
		Subsystem->HasMappingContext(AddedContext));
	TestTrue(TEXT("removal does not remove a later property value owned elsewhere"),
		Subsystem->HasMappingContext(CurrentPropertyContext));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderEndPlayClearsBoundInputTest,
	"DynamicRope.Wielder.Input.EndPlayClearsOriginallyBoundComponent",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderEndPlayClearsBoundInputTest::RunTest(const FString& Parameters)
{
	FTestWorldWrapper WorldWrapper;
	if (!WorldWrapper.CreateTestWorld(EWorldType::Game) || !WorldWrapper.BeginPlayInTestWorld())
	{
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	UWorld* World = WorldWrapper.GetTestWorld();
	APawn* Pawn = World->SpawnActor<APawn>();
	if (!TestNotNull(TEXT("lifecycle fixture spawned a Pawn"), Pawn))
	{
		WorldWrapper.DestroyTestWorld(false);
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	URopeComponent* Rope = NewObject<URopeComponent>(Pawn);
	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>(Pawn);
	Pawn->AddInstanceComponent(Wielder);
	Wielder->Rope = Rope;
	Wielder->bAutoBindInput = false;
	Wielder->bAttachOnBeginPlay = false;
	Wielder->bShowThrowPreview = false;
	Wielder->ThrowAction = NewObject<UInputAction>(Wielder);
	Wielder->RegisterComponent();
	if (!TestTrue(TEXT("Wielder entered the real component lifecycle"), Wielder->HasBegunPlay()))
	{
		WorldWrapper.DestroyTestWorld(false);
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	UEnhancedInputComponent* OriginallyBoundInput = NewObject<UEnhancedInputComponent>(Pawn);
	UEnhancedInputComponent* CurrentPawnInput = NewObject<UEnhancedInputComponent>(Pawn);
	Pawn->InputComponent = OriginallyBoundInput;
	Wielder->BindInput();
	if (!TestEqual(TEXT("fixture created one binding on the original InputComponent"),
		OriginallyBoundInput->GetActionEventBindings().Num(), 1))
	{
		WorldWrapper.DestroyTestWorld(false);
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	// Pawn restart/re-possession may replace InputComponent before component teardown.
	Pawn->InputComponent = CurrentPawnInput;
	Wielder->EndPlay(EEndPlayReason::Destroyed);

	TestEqual(TEXT("EndPlay clears the component that actually owns Wielder's bindings"),
		OriginallyBoundInput->GetActionEventBindings().Num(), 0);
	TestEqual(TEXT("EndPlay does not touch an unrelated replacement InputComponent"),
		CurrentPawnInput->GetActionEventBindings().Num(), 0);

	WorldWrapper.DestroyTestWorld(false);
	WorldWrapper.ForwardErrorMessages(this);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderCharacterHardLeashSameFrameTest,
	"DynamicRope.Movement.HardLeash.CharacterSameFrame",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderCharacterHardLeashSameFrameTest::RunTest(const FString& Parameters)
{
	FTestWorldWrapper WorldWrapper;
	if (!WorldWrapper.CreateTestWorld(EWorldType::Game) || !WorldWrapper.BeginPlayInTestWorld())
	{
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	UWorld* World = WorldWrapper.GetTestWorld();
	AActor* TargetActor = World->SpawnActor<AActor>();
	ACharacter* Character = World->SpawnActor<ACharacter>();
	if (!TestNotNull(TEXT("hard-leash fixture spawned target"), TargetActor) ||
		!TestNotNull(TEXT("hard-leash fixture spawned character"), Character))
	{
		WorldWrapper.DestroyTestWorld(false);
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	USphereComponent* TargetRoot = NewObject<USphereComponent>(TargetActor);
	TargetActor->AddInstanceComponent(TargetRoot);
	TargetActor->SetRootComponent(TargetRoot);
	TargetRoot->InitSphereRadius(10.0f);
	TargetRoot->SetMobility(EComponentMobility::Movable);
	TargetRoot->RegisterComponent();
	TargetActor->SetActorLocation(FVector(60.0f, 0.0f, 0.0f));
	TargetRoot->SetCollisionEnabled(ECollisionEnabled::PhysicsOnly);
	TargetRoot->SetSimulatePhysics(true);
	TargetRoot->SetEnableGravity(false);
	// Start 5 cm inside the boundary. A -10 cm CMC step crosses it; the Wielder and
	// simulated target must split the active 5 cm correction by generalized mass.
	Character->SetActorLocation(FVector(5.0f, 0.0f, 0.0f));

	URopeComponent* Rope = NewObject<URopeComponent>(Character);
	Character->AddInstanceComponent(Rope);
	// Match the production Blueprint: the rope is a child of the Character mesh/socket hierarchy,
	// whose transform propagation is deferred inside CMC's scoped movement update.
	Rope->SetupAttachment(Character->GetMesh());
	FRopeWielderComponentTestSeam::ConfigureExternalHardLeash(
		*Rope, TargetRoot, /*AnchorNode*/ 3, /*RopeLength*/ 60.0f);
	Rope->RegisterComponent();

	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>(Character);
	Character->AddInstanceComponent(Wielder);
	Wielder->Rope = Rope;
	Wielder->bAutoBindInput = false;
	Wielder->bAttachOnBeginPlay = false;
	Wielder->bShowThrowPreview = false;
	Wielder->bAutoGroundExitOnUpwardPull = false;
	Wielder->bBoostAirControlWhileSwinging = false;
	Wielder->RegisterComponent();
	TestTrue(TEXT("Wrapped Character keeps the post-movement PrePhysics leash tick enabled"),
		Wielder->IsComponentTickEnabled());

	UCharacterMovementComponent* Movement = Character->GetCharacterMovement();
	Movement->GravityScale = 0.0f;
	Movement->BrakingDecelerationFlying = 0.0f;
	Movement->bUseSeparateBrakingFriction = true;
	Movement->BrakingFriction = 0.0f;
	Movement->bRunPhysicsWithNoController = true;
	Movement->SetMovementMode(MOVE_Flying);
	Movement->Velocity = FVector(-600.0f, 0.0f, 0.0f);

	// One frame is intentional: a PostPhysics/next-tick correction would leave X=-5 here.
	World->Tick(LEVELTICK_All, 1.0f / 60.0f);
	FRopeWielderMovementConstraint ReactionConstraint;
	TestTrue(TEXT("physical fixture exposes a live material constraint"),
		Rope->BuildWielderMovementConstraint(ReactionConstraint));
	const float WielderReactionShare =
		FRopeWielderComponentTestSeam::ComputeWielderReactionShare(
			*Rope, ReactionConstraint);
	const float WielderPositionCorrectionShare =
		FRopeWielderComponentTestSeam::ComputeWielderPositionCorrectionShare(
			*Rope, ReactionConstraint);

	const float FinalDistance =
		static_cast<float>(FVector::Distance(Rope->GetComponentLocation(), TargetRoot->GetComponentLocation()));
	TestEqual(TEXT("same-frame coupled endpoints preserve exact material length"),
		FinalDistance, 60.0f, 0.1f);
	const float ExpectedWielderX =
		-5.0f * (1.0f - WielderPositionCorrectionShare);
	TestEqual(TEXT("inside-to-outside CMC move keeps only the Wielder position share"),
		static_cast<float>(Character->GetActorLocation().X),
		ExpectedWielderX,
		0.1f);
	const float ExpectedWielderVelocityX =
		-600.0f * (1.0f - WielderReactionShare);
	TestEqual(TEXT("Wielder velocity receives exactly its generalized-mass reaction share"),
		static_cast<float>(Movement->Velocity.X),
		ExpectedWielderVelocityX,
		2.0f);
	TestTrue(TEXT("simulated wrapped target receives a Chaos tether before physics"),
		FRopeWielderComponentTestSeam::HasPhysicalTether(*Rope));
	const float ProxyX =
		FRopeWielderComponentTestSeam::GetPhysicalTetherProxyWorld(*Rope).X;
	TestEqual(TEXT("Chaos proxy is the actual shared-correction hand point"),
		ProxyX,
		static_cast<float>(Rope->GetComponentLocation().X),
		0.1f);
	TestTrue(TEXT("physical target uses the hard material limit"),
		FMath::IsNearlyEqual(
			FRopeWielderComponentTestSeam::GetPhysicalTetherLimit(*Rope), 60.0f, 0.1f));
	TestTrue(TEXT("cancelled outward hand motion pulls the simulated target in the same Chaos step"),
		TargetRoot->GetComponentLocation().X < 59.99f);
	TestTrue(TEXT("physical backend publishes authoritative tension without visual stretch"),
		Rope->GetConstraintTension() > 0.0f);
	TestEqual(TEXT("simulated target uses only the Chaos constraint backend"),
		FRopeWielderComponentTestSeam::GetLengthConstraintBackend(*Rope),
		ERopeLengthConstraintBackend::Chaos);
	TestFalse(TEXT("zero material compliance creates a hard Chaos limit"),
		FRopeWielderComponentTestSeam::IsPhysicalTetherSoft(*Rope));
	TestTrue(TEXT("a hard Chaos limit carries no linear-limit stiffness"),
		FMath::IsNearlyZero(
			FRopeWielderComponentTestSeam::GetPhysicalTetherStiffness(*Rope)));

	WorldWrapper.DestroyTestWorld(false);
	WorldWrapper.ForwardErrorMessages(this);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderHardReactionTensionTest,
	"DynamicRope.Movement.HardLeash.KinematicReactionTension",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderHardReactionTensionTest::RunTest(const FString& Parameters)
{
	FTestWorldWrapper WorldWrapper;
	if (!WorldWrapper.CreateTestWorld(EWorldType::Game) || !WorldWrapper.BeginPlayInTestWorld())
	{
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	UWorld* World = WorldWrapper.GetTestWorld();
	AActor* TargetActor = World->SpawnActor<AActor>();
	ACharacter* Character = World->SpawnActor<ACharacter>();
	if (!TestNotNull(TEXT("reaction fixture spawned target"), TargetActor) ||
		!TestNotNull(TEXT("reaction fixture spawned character"), Character))
	{
		WorldWrapper.DestroyTestWorld(false);
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	USphereComponent* TargetRoot = NewObject<USphereComponent>(TargetActor);
	TargetActor->AddInstanceComponent(TargetRoot);
	TargetActor->SetRootComponent(TargetRoot);
	TargetRoot->InitSphereRadius(10.0f);
	TargetRoot->SetMobility(EComponentMobility::Movable);
	TargetRoot->RegisterComponent();
	TargetActor->SetActorLocation(FVector(60.0f, 0.0f, 0.0f));
	// Deliberately kinematic: this exercises the hard-reaction backend, not Chaos.
	TargetRoot->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	TargetRoot->SetSimulatePhysics(false);
	Character->SetActorLocation(FVector(5.0f, 0.0f, 0.0f));

	URopeComponent* Rope = NewObject<URopeComponent>(Character);
	Character->AddInstanceComponent(Rope);
	Rope->SetupAttachment(Character->GetMesh());
	FRopeWielderComponentTestSeam::ConfigureExternalHardLeash(
		*Rope, TargetRoot, /*AnchorNode*/ 3, /*RopeLength*/ 60.0f);
	Rope->RegisterComponent();

	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>(Character);
	Character->AddInstanceComponent(Wielder);
	Wielder->Rope = Rope;
	Wielder->bAutoBindInput = false;
	Wielder->bAttachOnBeginPlay = false;
	Wielder->bShowThrowPreview = false;
	Wielder->bAutoGroundExitOnUpwardPull = false;
	Wielder->bBoostAirControlWhileSwinging = false;
	Wielder->RegisterComponent();

	UCharacterMovementComponent* Movement = Character->GetCharacterMovement();
	Movement->GravityScale = 0.0f;
	Movement->BrakingDecelerationFlying = 0.0f;
	Movement->bUseSeparateBrakingFriction = true;
	Movement->BrakingFriction = 0.0f;
	Movement->bRunPhysicsWithNoController = true;
	Movement->SetMovementMode(MOVE_Flying);
	Movement->Velocity = FVector(-600.0f, 0.0f, 0.0f);

	World->Tick(LEVELTICK_All, 1.0f / 60.0f);
	TestTrue(TEXT("kinematic boundary crossing is still hard-clamped"),
		FMath::IsNearlyEqual(Character->GetActorLocation().X, 0.0f, 0.1f));
	TestTrue(TEXT("rejected movement creates authoritative tension at zero extension"),
		Rope->GetConstraintTension() > 0.0f);
	TestEqual(TEXT("kinematic target uses only the hard-reaction backend"),
		FRopeWielderComponentTestSeam::GetLengthConstraintBackend(*Rope),
		ERopeLengthConstraintBackend::HardReaction);
	TestTrue(TEXT("reaction tension does not require material extension"),
		FVector::Distance(Rope->GetComponentLocation(), TargetRoot->GetComponentLocation()) <= 60.1f);

	WorldWrapper.DestroyTestWorld(false);
	WorldWrapper.ForwardErrorMessages(this);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderNodeZeroReactionTensionTest,
	"DynamicRope.Movement.HardLeash.NodeZeroPublishesReactionTension",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderNodeZeroReactionTensionTest::RunTest(const FString& Parameters)
{
	FTestWorldWrapper WorldWrapper;
	if (!WorldWrapper.CreateTestWorld(EWorldType::Game) ||
		!WorldWrapper.BeginPlayInTestWorld())
	{
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	UWorld* World = WorldWrapper.GetTestWorld();
	AActor* TargetActor = World->SpawnActor<AActor>();
	ACharacter* Character = World->SpawnActor<ACharacter>();
	if (!TestNotNull(TEXT("node-zero fixture target spawned"), TargetActor) ||
		!TestNotNull(TEXT("node-zero fixture character spawned"), Character))
	{
		WorldWrapper.DestroyTestWorld(false);
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	USphereComponent* TargetRoot =
		AddTestTargetRoot(*TargetActor, FVector::ZeroVector, false);
	Character->SetActorLocation(FVector::ZeroVector);
	URopeComponent* Rope = NewObject<URopeComponent>(Character);
	Character->AddInstanceComponent(Rope);
	Rope->SetupAttachment(Character->GetMesh());
	FRopeWielderComponentTestSeam::ConfigureExternalHardLeash(
		*Rope, TargetRoot, /*AnchorNode*/ 0, /*RopeLength*/ 60.0f);
	Rope->RegisterComponent();
	AddTestWielder(*Character, *Rope);
	ConfigureFlyingMovement(
		*Character, FVector(-600.0f, 0.0f, 0.0f));

	World->Tick(LEVELTICK_All, 1.0f / 60.0f);
	TestTrue(TEXT("node-zero hard boundary keeps the hand on the live anchor"),
		Rope->GetComponentLocation().Equals(
			TargetRoot->GetComponentLocation(), 0.1f));
	TestTrue(TEXT("node-zero publishes tension without a valid pull-lookahead segment"),
		Rope->GetConstraintTension() > 0.0f);
	TestEqual(TEXT("node-zero kinematic target uses the hard-reaction backend"),
		FRopeWielderComponentTestSeam::GetLengthConstraintBackend(*Rope),
		ERopeLengthConstraintBackend::HardReaction);

	WorldWrapper.DestroyTestWorld(false);
	WorldWrapper.ForwardErrorMessages(this);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderPhysicalNodeZeroTest,
	"DynamicRope.Movement.HardLeash.PhysicalNodeZeroHasExactLimit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderPhysicalNodeZeroTest::RunTest(const FString& Parameters)
{
	FTestWorldWrapper WorldWrapper;
	if (!WorldWrapper.CreateTestWorld(EWorldType::Game) ||
		!WorldWrapper.BeginPlayInTestWorld())
	{
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	UWorld* World = WorldWrapper.GetTestWorld();
	AActor* TargetActor = World->SpawnActor<AActor>();
	ACharacter* Character = World->SpawnActor<ACharacter>();
	if (!TestNotNull(TEXT("physical node-zero target spawned"), TargetActor) ||
		!TestNotNull(TEXT("physical node-zero character spawned"), Character))
	{
		WorldWrapper.DestroyTestWorld(false);
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	USphereComponent* TargetRoot =
		AddTestTargetRoot(*TargetActor, FVector::ZeroVector, true);
	Character->SetActorLocation(FVector::ZeroVector);
	URopeComponent* Rope = NewObject<URopeComponent>(Character);
	Character->AddInstanceComponent(Rope);
	Rope->SetupAttachment(Character->GetMesh());
	FRopeWielderComponentTestSeam::ConfigureExternalHardLeash(
		*Rope, TargetRoot, /*AnchorNode*/ 0, /*RopeLength*/ 60.0f);
	Rope->RegisterComponent();
	AddTestWielder(*Character, *Rope);
	ConfigureFlyingMovement(
		*Character, FVector(-600.0f, 0.0f, 0.0f));

	World->Tick(LEVELTICK_All, 1.0f / 60.0f);
	TestTrue(TEXT("physical node-zero creates the Chaos backend"),
		FRopeWielderComponentTestSeam::HasPhysicalTether(*Rope));
	TestTrue(TEXT("physical node-zero stores an exact zero material limit"),
		FMath::IsNearlyZero(
			FRopeWielderComponentTestSeam::GetPhysicalTetherLimit(*Rope)));
	TestTrue(TEXT("Chaos represents zero radius with XYZ Locked"),
		FRopeWielderComponentTestSeam::IsPhysicalTetherLinearLocked(*Rope));
	TestEqual(TEXT("node-zero coupled correction keeps hand and target coincident"),
		static_cast<float>(FVector::Distance(
			Rope->GetComponentLocation(),
			TargetRoot->GetComponentLocation())),
		0.0f,
		0.1f);
	TestTrue(TEXT("physical node-zero publishes reaction tension"),
		Rope->GetConstraintTension() > 0.0f);

	// The tolerance used for numeric limit updates must not hide the categorical
	// Locked -> Limited transition for a short, non-zero material leg.
	FRopeWielderComponentTestSeam::UpdatePhysicalTether(
		*Rope,
		TargetRoot,
		TargetRoot->GetComponentLocation(),
		Rope->GetComponentLocation(),
		/*Limit*/ 0.25f,
		1.0f / 60.0f);
	TestEqual(TEXT("sub-centimeter non-zero limit is stored"),
		FRopeWielderComponentTestSeam::GetPhysicalTetherLimit(*Rope),
		0.25f,
		0.001f);
	TestTrue(TEXT("zero-to-nonzero transition switches XYZ to Limited"),
		FRopeWielderComponentTestSeam::IsPhysicalTetherLinearLimited(*Rope));

	WorldWrapper.DestroyTestWorld(false);
	WorldWrapper.ForwardErrorMessages(this);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderReelInSameFrameTest,
	"DynamicRope.Movement.HardLeash.ReelInSameFrameNoDoubleReaction",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderReelInSameFrameTest::RunTest(const FString& Parameters)
{
	constexpr float Dt = 1.0f / 60.0f;
	constexpr float ReelSpeed = 600.0f;
	FTestWorldWrapper WorldWrapper;
	if (!WorldWrapper.CreateTestWorld(EWorldType::Game) ||
		!WorldWrapper.BeginPlayInTestWorld())
	{
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	UWorld* World = WorldWrapper.GetTestWorld();
	AActor* TargetActor = World->SpawnActor<AActor>();
	ACharacter* Character = World->SpawnActor<ACharacter>();
	if (!TestNotNull(TEXT("reel fixture target spawned"), TargetActor) ||
		!TestNotNull(TEXT("reel fixture character spawned"), Character))
	{
		WorldWrapper.DestroyTestWorld(false);
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	USphereComponent* TargetRoot =
		AddTestTargetRoot(*TargetActor, FVector(60.0f, 0.0f, 0.0f), false);
	Character->SetActorLocation(FVector::ZeroVector);
	URopeComponent* Rope = NewObject<URopeComponent>(Character);
	Character->AddInstanceComponent(Rope);
	Rope->SetupAttachment(Character->GetMesh());
	FRopeWielderComponentTestSeam::ConfigureExternalHardLeash(
		*Rope, TargetRoot, /*AnchorNode*/ 3, /*RopeLength*/ 60.0f);
	Rope->RopeLength = 60.0f;
	Rope->MinRopeLength = 0.0f;
	Rope->SetReelRate(ReelSpeed);
	FRopeWielderComponentTestSeam::SeedPreviousLengthGeometry(
		*Rope, 3, 60.0f, TargetRoot->GetComponentLocation());
	Rope->RegisterComponent();
	AddTestWielder(*Character, *Rope);
	UCharacterMovementComponent* Movement =
		ConfigureFlyingMovement(*Character, FVector::ZeroVector);

	World->Tick(LEVELTICK_All, Dt);
	TestTrue(TEXT("PrePhysics predicts the pending reel-in material boundary"),
		FVector::Distance(
			Rope->GetComponentLocation(),
			TargetRoot->GetComponentLocation()) <= 50.1f);
	TestTrue(TEXT("PostPhysics commits exactly the predicted material length"),
		FMath::IsNearlyEqual(
			FRopeWielderComponentTestSeam::GetMaterialRopeLength(*Rope),
			50.0f,
			0.01f));
	const float ExpectedTension =
		ReelSpeed * Movement->Mass / Dt;
	TestTrue(TEXT("reel-in reaction is counted once, not once in projection and again in rest-rate"),
		FMath::IsNearlyEqual(
			Rope->GetConstraintTension(),
			ExpectedTension,
			ExpectedTension * 0.01f));
	TestEqual(TEXT("rigid reel-in uses the hard-reaction backend"),
		FRopeWielderComponentTestSeam::GetLengthConstraintBackend(*Rope),
		ERopeLengthConstraintBackend::HardReaction);

	WorldWrapper.DestroyTestWorld(false);
	WorldWrapper.ForwardErrorMessages(this);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderCharacterTargetReactionShareTest,
	"DynamicRope.Movement.HardLeash.CharacterTargetClosesRelativeVelocity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderCharacterTargetReactionShareTest::RunTest(const FString& Parameters)
{
	FTestWorldWrapper WorldWrapper;
	if (!WorldWrapper.CreateTestWorld(EWorldType::Game) ||
		!WorldWrapper.BeginPlayInTestWorld())
	{
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	UWorld* World = WorldWrapper.GetTestWorld();
	ACharacter* TargetCharacter = World->SpawnActor<ACharacter>();
	ACharacter* WielderCharacter = World->SpawnActor<ACharacter>();
	if (!TestNotNull(TEXT("character-target fixture target spawned"), TargetCharacter) ||
		!TestNotNull(TEXT("character-target fixture wielder spawned"), WielderCharacter))
	{
		WorldWrapper.DestroyTestWorld(false);
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	TargetCharacter->SetActorLocation(FVector(60.0f, 0.0f, 0.0f));
	WielderCharacter->SetActorLocation(FVector(5.0f, 0.0f, 0.0f));
	if (UPrimitiveComponent* TargetRoot =
		Cast<UPrimitiveComponent>(TargetCharacter->GetRootComponent()))
	{
		TargetRoot->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	}
	if (UPrimitiveComponent* WielderRoot =
		Cast<UPrimitiveComponent>(WielderCharacter->GetRootComponent()))
	{
		WielderRoot->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	}
	URopeComponent* Rope = NewObject<URopeComponent>(WielderCharacter);
	WielderCharacter->AddInstanceComponent(Rope);
	Rope->SetupAttachment(WielderCharacter->GetMesh());
	FRopeWielderComponentTestSeam::ConfigureExternalHardLeash(
		*Rope,
		TargetCharacter->GetRootComponent(),
		/*AnchorNode*/ 3,
		/*RopeLength*/ 60.0f);
	Rope->RegisterComponent();
	AddTestWielder(*WielderCharacter, *Rope);
	UCharacterMovementComponent* TargetMovement =
		ConfigureFlyingMovement(*TargetCharacter, FVector::ZeroVector);
	UCharacterMovementComponent* WielderMovement =
		ConfigureFlyingMovement(
			*WielderCharacter, FVector(-600.0f, 0.0f, 0.0f));

	World->Tick(LEVELTICK_All, 1.0f / 60.0f);
	TestTrue(TEXT("two Character endpoints receive matching post-reaction rope-axis velocity"),
		FMath::IsNearlyEqual(
			WielderMovement->Velocity.X,
			TargetMovement->Velocity.X,
			1.0f));
	TestTrue(TEXT("the shared reaction removes separating relative velocity"),
		FMath::Abs(
			WielderMovement->Velocity.X -
			TargetMovement->Velocity.X) <= 1.0f);
	TestTrue(TEXT("Character target reaction still publishes rigid tension"),
		Rope->GetConstraintTension() > 0.0f);
	TestEqual(TEXT("nonphysical Character target uses one hard-reaction backend"),
		FRopeWielderComponentTestSeam::GetLengthConstraintBackend(*Rope),
		ERopeLengthConstraintBackend::HardReaction);

	WorldWrapper.DestroyTestWorld(false);
	WorldWrapper.ForwardErrorMessages(this);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderCoMovingTargetTest,
	"DynamicRope.Movement.HardLeash.CoMovingTargetUsesCurrentPivotVelocity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderCoMovingTargetTest::RunTest(const FString& Parameters)
{
	FTestWorldWrapper WorldWrapper;
	if (!WorldWrapper.CreateTestWorld(EWorldType::Game) ||
		!WorldWrapper.BeginPlayInTestWorld())
	{
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	UWorld* World = WorldWrapper.GetTestWorld();
	ACharacter* TargetCharacter = World->SpawnActor<ACharacter>();
	ACharacter* WielderCharacter = World->SpawnActor<ACharacter>();
	if (!TestNotNull(TEXT("co-moving target spawned"), TargetCharacter) ||
		!TestNotNull(TEXT("co-moving Wielder spawned"), WielderCharacter))
	{
		WorldWrapper.DestroyTestWorld(false);
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	TargetCharacter->SetActorLocation(FVector(60.0f, 0.0f, 0.0f));
	WielderCharacter->SetActorLocation(FVector::ZeroVector);
	for (ACharacter* Character : { TargetCharacter, WielderCharacter })
	{
		if (UPrimitiveComponent* Root =
			Cast<UPrimitiveComponent>(Character->GetRootComponent()))
		{
			Root->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		}
	}

	URopeComponent* Rope = NewObject<URopeComponent>(WielderCharacter);
	WielderCharacter->AddInstanceComponent(Rope);
	Rope->SetupAttachment(WielderCharacter->GetMesh());
	FRopeWielderComponentTestSeam::ConfigureExternalHardLeash(
		*Rope,
		TargetCharacter->GetRootComponent(),
		/*AnchorNode*/ 3,
		/*RopeLength*/ 60.0f);
	FRopeWielderComponentTestSeam::SeedPreviousLengthGeometry(
		*Rope,
		3,
		60.0f,
		FVector(60.0f, 0.0f, 0.0f));
	Rope->RegisterComponent();
	AddTestWielder(*WielderCharacter, *Rope);
	const FVector CoMovingVelocity(-300.0f, 0.0f, 0.0f);
	ConfigureFlyingMovement(*TargetCharacter, CoMovingVelocity);
	UCharacterMovementComponent* WielderMovement =
		ConfigureFlyingMovement(*WielderCharacter, CoMovingVelocity);

	World->Tick(LEVELTICK_All, 1.0f / 60.0f);
	TestEqual(TEXT("co-moving Wielder keeps its world velocity"),
		static_cast<float>(WielderMovement->Velocity.X),
		-300.0f,
		1.0f);
	TestTrue(TEXT("equal current pivot/hand velocity does not create false reaction tension"),
		Rope->GetConstraintTension() <= 1.0f);
	TestTrue(TEXT("co-moving endpoints preserve exact material separation"),
		FMath::IsNearlyEqual(
			static_cast<float>(FVector::Distance(
				Rope->GetComponentLocation(),
				TargetCharacter->GetActorLocation())),
			60.0f,
			0.1f));

	WorldWrapper.DestroyTestWorld(false);
	WorldWrapper.ForwardErrorMessages(this);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderPhysicalSlackTest,
	"DynamicRope.Movement.HardLeash.PhysicalTargetPreservesSlack",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderPhysicalSlackTest::RunTest(const FString& Parameters)
{
	FTestWorldWrapper WorldWrapper;
	if (!WorldWrapper.CreateTestWorld(EWorldType::Game) ||
		!WorldWrapper.BeginPlayInTestWorld())
	{
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	UWorld* World = WorldWrapper.GetTestWorld();
	AActor* TargetActor = World->SpawnActor<AActor>();
	ACharacter* Character = World->SpawnActor<ACharacter>();
	if (!TestNotNull(TEXT("slack fixture target spawned"), TargetActor) ||
		!TestNotNull(TEXT("slack fixture character spawned"), Character))
	{
		WorldWrapper.DestroyTestWorld(false);
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	USphereComponent* TargetRoot =
		AddTestTargetRoot(*TargetActor, FVector(60.0f, 0.0f, 0.0f), true);
	Character->SetActorLocation(FVector(20.0f, 0.0f, 0.0f));
	URopeComponent* Rope = NewObject<URopeComponent>(Character);
	Character->AddInstanceComponent(Rope);
	Rope->SetupAttachment(Character->GetMesh());
	FRopeWielderComponentTestSeam::ConfigureExternalHardLeash(
		*Rope, TargetRoot, /*AnchorNode*/ 3, /*RopeLength*/ 60.0f);
	Rope->RegisterComponent();
	AddTestWielder(*Character, *Rope);
	ConfigureFlyingMovement(*Character, FVector::ZeroVector);
	const FVector InitialTarget = TargetRoot->GetComponentLocation();

	World->Tick(LEVELTICK_All, 1.0f / 60.0f);
	TestTrue(TEXT("slack physical proxy stays at the actual hand, not the material boundary"),
		FRopeWielderComponentTestSeam::GetPhysicalTetherProxyWorld(*Rope)
			.Equals(Rope->GetComponentLocation(), 0.1f));
	TestTrue(TEXT("unused material slack produces no authoritative load"),
		Rope->GetConstraintTension() <= 1.0f);
	TestTrue(TEXT("a stationary physical target is not pulled across available slack"),
		TargetRoot->GetComponentLocation().Equals(InitialTarget, 0.1f));

	WorldWrapper.DestroyTestWorld(false);
	WorldWrapper.ForwardErrorMessages(this);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderCompliantPhysicalTargetTest,
	"DynamicRope.Movement.HardLeash.ComplianceUsesCommonAnalyticBackend",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderCompliantPhysicalTargetTest::RunTest(const FString& Parameters)
{
	FTestWorldWrapper WorldWrapper;
	if (!WorldWrapper.CreateTestWorld(EWorldType::Game) ||
		!WorldWrapper.BeginPlayInTestWorld())
	{
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	UWorld* World = WorldWrapper.GetTestWorld();
	AActor* TargetActor = World->SpawnActor<AActor>();
	ACharacter* Character = World->SpawnActor<ACharacter>();
	if (!TestNotNull(TEXT("compliance fixture target spawned"), TargetActor) ||
		!TestNotNull(TEXT("compliance fixture character spawned"), Character))
	{
		WorldWrapper.DestroyTestWorld(false);
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	USphereComponent* TargetRoot =
		AddTestTargetRoot(*TargetActor, FVector(60.0f, 0.0f, 0.0f), true);
	Character->SetActorLocation(FVector(-5.0f, 0.0f, 0.0f));
	URopeComponent* Rope = NewObject<URopeComponent>(Character);
	Character->AddInstanceComponent(Rope);
	Rope->SetupAttachment(Character->GetMesh());
	FRopeWielderComponentTestSeam::ConfigureExternalHardLeash(
		*Rope, TargetRoot, /*AnchorNode*/ 3, /*RopeLength*/ 60.0f);
	Rope->HoldConfig.TetherCompliance = 0.0005f;
	Rope->HoldConfig.MaxTetherTension = 60000.0f;
	Rope->RegisterComponent();
	AddTestWielder(*Character, *Rope);
	ConfigureFlyingMovement(*Character, FVector::ZeroVector);

	World->Tick(LEVELTICK_All, 1.0f / 60.0f);
	TestTrue(TEXT("positive compliance does not invoke a separate hard Pawn projection"),
		Character->GetActorLocation().X < -4.9f);
	TestFalse(TEXT("compliant physical target does not create a second Chaos spring"),
		FRopeWielderComponentTestSeam::HasPhysicalTether(*Rope));
	TestEqual(TEXT("all compliant endpoints use the common analytic lambda backend"),
		FRopeWielderComponentTestSeam::GetLengthConstraintBackend(*Rope),
		ERopeLengthConstraintBackend::Analytic);
	TestTrue(TEXT("compliant reaction is positive and respects its force cap"),
		Rope->GetConstraintTension() > 0.0f &&
		Rope->GetConstraintTension() <= 60000.1f);

	WorldWrapper.DestroyTestWorld(false);
	WorldWrapper.ForwardErrorMessages(this);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderHiddenPreviewDoesNotBuildTest,
	"DynamicRope.Wielder.Preview.HiddenIdlePreviewDoesNotBuild",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderHiddenPreviewDoesNotBuildTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	Rope->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
	Rope->EnterLoaded();

	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>();
	Wielder->Rope = Rope;
	Wielder->bShowThrowPreview = false;

	FRopeThrowContext Context;
	Context.FrameForward = FVector::ForwardVector;
	Context.FrameRight = FVector::RightVector;
	Context.FrameUp = FVector::UpVector;
	Context.bAimRayEvaluated = true;
	FRopeWielderComponentTestSeam::SetAimFrameContext(*Wielder, Context);
	FRopeWielderComponentTestSeam::UpdateThrowPreview(*Wielder);

	TestEqual(TEXT("hidden idle preview does not attempt a prepared path build"),
		FRopeWielderComponentTestSeam::GetPreparedPreviewBuildCount(*Wielder), 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderStaticSelfWrapIsNotTetherTest,
	"DynamicRope.Wielder.Movement.StaticSelfWrapIsNotExternalTether",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderStaticSelfWrapIsNotTetherTest::RunTest(const FString& Parameters)
{
	AActor* Owner = NewObject<AActor>();
	URopeComponent* Rope = NewObject<URopeComponent>(Owner);
	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>(Owner);
	USceneComponent* StaticWrappedComponent = NewObject<USceneComponent>(Owner);
	Wielder->Rope = Rope;
	FRopeWielderComponentTestSeam::ForceStaticSelfWrap(*Rope, StaticWrappedComponent);

	TestFalse(TEXT("wrapping any component owned by the wielder is a self-wrap, not an external tether"),
		FRopeWielderComponentTestSeam::IsWielderTetherActive(*Wielder));
	return true;
}

// The hand swing's relative velocity is divided by the real delta time and is therefore independent of the frame
// rate: the same hand movement in world space gives the same velocity at 15, 20 and 30 fps, which guards against the
// velocity dying at a low frame rate. The upper clamp and the guard against dividing by zero are checked too.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeHandSwingVelocityFrameRateTest,
	"DynamicRope.Wielder.HandSwingVelocityFrameRateConsistent",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeHandSwingVelocityFrameRateTest::RunTest(const FString& Parameters)
{
	const FTransform Identity = FTransform::Identity;
	const FVector V(300.0f, 0.0f, 0.0f);   // The target world hand velocity in cm/s, below the limit of 2000.
	constexpr float MaxSpeed = 2000.0f;

	for (float F : { 15.0f, 20.0f, 30.0f })
	{
		const float Dt = 1.0f / F;
		const FVector Cur = V * Dt;         // The displacement over the delta, being the velocity times the delta, with the previous position at zero.
		const FVector Vel = FRopeWielderComponentTestSeam::ComputeHandSwingVelocityWorld(
			FVector::ZeroVector, Cur, Dt, Identity, MaxSpeed);
		TestTrue(FString::Printf(TEXT("the velocity at %.0f fps is frame-rate independent and about V"), F), Vel.Equals(V, 0.1f));
	}

	// The upper limit: a displacement equivalent to 3000 cm/s is clamped to the maximum speed.
	{
		const float Dt = 1.0f / 30.0f;
		const FVector Vel = FRopeWielderComponentTestSeam::ComputeHandSwingVelocityWorld(
			FVector::ZeroVector, FVector(3000.0f, 0.0f, 0.0f) * Dt, Dt, Identity, MaxSpeed);
		TestEqual(TEXT("the upper clamp is 2000"), static_cast<float>(Vel.Size()), 2000.0f, 0.5f);
	}

	// The guard for a delta of about zero: it returns zero, with no division by zero.
	{
		const FVector Vel = FRopeWielderComponentTestSeam::ComputeHandSwingVelocityWorld(
			FVector::ZeroVector, FVector(10.0f, 0.0f, 0.0f), 0.0f, Identity, MaxSpeed);
		TestTrue(TEXT("a delta of zero gives zero"), Vel.IsNearlyZero());
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
