// Copyright Epic Games, Inc. All Rights Reserved.
//
// URopeWielderComponent 입력/애니메이션/preview 수명주기 회귀 테스트.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "RopeWielderComponentTestTypes.h"

#include "Gameplay/AnimNotifyState_RopePull.h"
#include "Gameplay/RopeWielderComponent.h"
#include "RopeComponent.h"

#include "Animation/AnimMontage.h"
#include "Components/SceneComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/Engine.h"
#include "Engine/LocalPlayer.h"
#include "Engine/World.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "EnhancedPlayerInput.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "InputAction.h"
#include "InputMappingContext.h"
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

	static void ForceWrappedTaut(URopeComponent& Rope)
	{
		Rope.Phase = ERopePhase::Wrapped;
		Rope.PullDrive.bPullTaut = true;
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
};

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

#endif // WITH_DEV_AUTOMATION_TESTS
