// Copyright 2026 TeamKeno. All Rights Reserved.
//
// Regression tests for URopeWielderComponent's input, animation and preview lifetimes.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "RopeWielderComponentTestTypes.h"

#include "Gameplay/AnimNotifyState_RopePull.h"
#include "Gameplay/RopeRagdollResponseComponent.h"
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
#include "PhysicsEngine/PhysicsSettings.h"
#include "Tests/AutomationCommon.h"

// The HardLeash Chaos-backend tests assert synchronous same-frame semantics: create the constraint, tick the
// world once, observe the reaction on that very tick. A project running async fixed-tick physics (the
// frame-rate-independence setting) defers simulation off the game thread, which breaks that contract by
// design rather than by bug. These tests specify the synchronous mode, so each pins the setting while its
// physics scene exists; the project value is restored on scope exit.
struct FScopedSyncPhysicsForTest
{
	bool bSavedTickPhysicsAsync;

	FScopedSyncPhysicsForTest()
	{
		UPhysicsSettings* Settings = UPhysicsSettings::Get();
		bSavedTickPhysicsAsync = Settings->bTickPhysicsAsync;
		Settings->bTickPhysicsAsync = false;
	}

	~FScopedSyncPhysicsForTest()
	{
		UPhysicsSettings::Get()->bTickPhysicsAsync = bSavedTickPhysicsAsync;
	}
};

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

	static FVector SampleCenterlineAtArcLength(const TArray<FVector>& Positions, float ArcLength)
	{
		return URopeWielderComponent::SampleCenterlineAtArcLength(Positions, ArcLength);
	}

	static void SetSimPositions(URopeComponent& Rope, const TArray<FVector>& Positions)
	{
		Rope.Sim.Positions = Positions;
		Rope.Sim.PrevPositions = Positions;
		Rope.Sim.InvMass.Init(1.0f, Positions.Num());
		Rope.Sim.SegmentLength = Positions.Num() >= 2
			? static_cast<float>(FVector::Dist(Positions[0], Positions[1]))
			: 0.0f;
	}

	static void SetSimPrevPositions(URopeComponent& Rope, const TArray<FVector>& Prev)
	{
		Rope.Sim.PrevPositions = Prev;
	}

	// The minimal state under which ApplyTautPresentationShaping is live: a taut Wrapped hold with a
	// wrap anchor bounding the free span, and the presentation blend already faded in.
	static void SetTautPresentationState(URopeComponent& Rope, int32 AnchorNode, float Blend)
	{
		Rope.Phase = ERopePhase::Wrapped;
		Rope.PullDrive.bChainTaut = true;
		FRopeSurfaceAnchor Anchor;
		Anchor.NodeIndex = AnchorNode;
		Rope.WrapController.State.Anchors = { Anchor };
		Rope.TautPresentationBlend = Blend;
	}

	static void ApplyHangGripPin(URopeComponent& Rope)
	{
		Rope.ApplyHangGripPinOverride();
	}

	static float GetSegmentLength(const URopeComponent& Rope)
	{
		return Rope.Sim.SegmentLength;
	}

	static float GetOverrideInvMass(const URopeComponent& Rope, int32 Node)
	{
		return Rope.SimFrame.OverrideFrame.InvMass.IsValidIndex(Node)
			? Rope.SimFrame.OverrideFrame.InvMass[Node]
			: -1.0f;
	}

	static FVector GetOverridePosition(const URopeComponent& Rope, int32 Node)
	{
		return Rope.SimFrame.OverrideFrame.Positions.IsValidIndex(Node)
			? Rope.SimFrame.OverrideFrame.Positions[Node]
			: FVector(TNumericLimits<float>::Max());
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

	// EnterRagdoll needs a skeletal mesh with a populated physics asset, so the automatic suppression rule
	// is driven through the flag it ends up setting instead.
	static void SetRagdolled(URopeRagdollResponseComponent& Response, bool bRagdolled)
	{
		Response.bRagdolled = bRagdolled;
	}

	static void SetRagdollResponse(URopeWielderComponent& Wielder, URopeRagdollResponseComponent* Response)
	{
		Wielder.RagdollResponse = Response;
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

	static void ForceWrappingPhase(URopeComponent& Rope, const USceneComponent* CandidateMesh)
	{
		Rope.Phase = ERopePhase::Wrapping;
		Rope.ContactTracker.CandidateMesh = CandidateMesh;
	}

	static bool IsWielderTetherActive(const URopeWielderComponent& Wielder)
	{
		return Wielder.IsWielderTetherActive();
	}

	// The minimal Wrapped state under which the hang test reaches its direction and load gates: an
	// external tether share, a latched taut chain and a valid pull sample. DirToHand is the sample's
	// anchor-to-hand direction, so an anchor straight above the hand is (0, 0, -1).
	static void ForceExternalWrapHangState(URopeComponent& Rope, const FVector& DirToHand, float Tension)
	{
		Rope.Phase = ERopePhase::Wrapped;
		Rope.PullDrive.LastTargetShare = 0.0f;
		Rope.PullDrive.bChainTaut = true;
		Rope.PullDrive.LastPullSample.bValid = true;
		Rope.PullDrive.LastPullSample.Direction = DirToHand;
		SetConstraintTension(Rope, Tension);
	}

	static void UpdateHangLatch(URopeWielderComponent& Wielder, float DeltaTime)
	{
		Wielder.UpdateHangLatch(DeltaTime);
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

	// The directly bound hand side: a constraint whose Frame1 is the owner's own body, with no proxy at all.
	static const UPrimitiveComponent* GetPhysicalTetherWielderBody(const URopeComponent& Rope)
	{
		return Rope.PhysicalTetherConstraint ? Rope.PhysicalTetherWielder.Get() : nullptr;
	}

	static bool HasPhysicalTetherProxy(const URopeComponent& Rope)
	{
		return Rope.PhysicalTetherProxy != nullptr;
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

	// MAX_uint64 while no hard Wielder projection has ever been recorded. Reading the raw stamp rather than
	// HasWielderAttempt keeps the assertion independent of GFrameCounter inside a hand-driven World->Tick.
	static uint64 GetWielderAttemptFrame(const URopeComponent& Rope)
	{
		return Rope.LengthConstraintState.WielderAttemptFrame;
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
			DeltaTime,
			/*bCornerIsOwnerAttachPoint*/ true);
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

// Rope input is discarded while suppressed. The gate sits on the public API rather than on the private
// input handlers, because four of the bindings in BindInput go straight to that API and a game is
// invited to bind its own keys to it as well.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderSuppressedInputIsDiscardedTest,
	"DynamicRope.Wielder.Input.SuppressedInputIsDiscarded",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderSuppressedInputIsDiscardedTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>();
	Wielder->Rope = Rope;

	Wielder->SetRopeInputSuppressed(true);
	TestTrue(TEXT("the external latch suppresses rope input"), Wielder->IsRopeInputSuppressed());

	Wielder->StartPull();
	TestFalse(TEXT("suppressed Pull input does not arm"), Wielder->IsPullArmed());
	Wielder->StartReelIn();
	TestTrue(TEXT("suppressed Reel input does not start reeling"),
		FMath::IsNearlyZero(FRopeWielderComponentTestSeam::GetReelRate(*Rope)));

	Wielder->SetRopeInputSuppressed(false);
	TestFalse(TEXT("clearing the latch resumes rope input"), Wielder->IsRopeInputSuppressed());
	Wielder->StartReelIn();
	TestFalse(TEXT("Reel input works again once suppression ends"),
		FMath::IsNearlyZero(FRopeWielderComponentTestSeam::GetReelRate(*Rope)));
	return true;
}

// Held input is cancelled the moment suppression begins, the same way an unpossession cancels it: a pull
// left armed would engage by itself, and a reel key held across the transition would keep shortening the
// rope because its Completed event arrives while the input is being discarded.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderSuppressionCancelsHeldInputTest,
	"DynamicRope.Wielder.Input.SuppressionCancelsHeldInput",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderSuppressionCancelsHeldInputTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>();
	Wielder->Rope = Rope;

	Wielder->StartPull();
	Wielder->StartPullNow(/*bIgnoreTautGate*/ true);
	Wielder->StartReelIn();
	Wielder->SetRopeInputSuppressed(true);

	TestFalse(TEXT("suppression disarms held Pull input"), Wielder->IsPullArmed());
	TestTrue(TEXT("suppression clears active Pull force"),
		FMath::IsNearlyZero(FRopeWielderComponentTestSeam::GetActivePullForce(*Rope)));
	TestTrue(TEXT("suppression clears held Reel rate"),
		FMath::IsNearlyZero(FRopeWielderComponentTestSeam::GetReelRate(*Rope)));
	return true;
}

// Suppression turns aiming off through IsAimActive, the single gate the HUD widget and the Gameplay
// Debugger both read, so asserting that one call covers all of them.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderSuppressionDisablesAimTest,
	"DynamicRope.Wielder.Aim.SuppressionDisablesAim",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderSuppressionDisablesAimTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	// AssistedJudged has no phase gate, so aiming is live from the start and the only thing the assertions
	// below can be measuring is the suppression term.
	Rope->ResolveMode = ERopeWrapResolveMode::AssistedJudged;
	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>();
	Wielder->Rope = Rope;

	TestTrue(TEXT("an unsuppressed aim ray mode aims"), Wielder->IsAimActive());

	Wielder->SetRopeInputSuppressed(true);
	TestFalse(TEXT("suppression stops aiming"), Wielder->IsAimActive());
	TestTrue(TEXT("suppression does not change the mode's use of an aim ray"), Wielder->UsesAimRay());

	Wielder->SetRopeInputSuppressed(false);
	TestTrue(TEXT("aiming resumes once suppression ends"), Wielder->IsAimActive());
	return true;
}

// The throw path, including the montage entry point. ThrowNow is the regression guard: a throw montage
// started a frame before the owner goes limp still reaches its notify, and in GuaranteedWrap the queued
// throw has been cancelled by then, so an ungated ThrowInDirection would cast a fresh aim ray from the
// limp body instead of doing nothing.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderSuppressedThrowIsDiscardedTest,
	"DynamicRope.Wielder.Throw.SuppressedThrowIsDiscarded",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderSuppressedThrowIsDiscardedTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	Rope->ResolveMode = ERopeWrapResolveMode::FullSimulation;
	URopeWielderBuildContextProbe* Wielder = NewObject<URopeWielderBuildContextProbe>();
	Wielder->Rope = Rope;

	Wielder->SetRopeInputSuppressed(true);
	Wielder->Throw();
	TestEqual(TEXT("a suppressed Throw never builds a throw context"), Wielder->BuildContextCalls, 0);
	TestEqual(TEXT("a suppressed Throw reports the InputSuppressed reason"),
		Wielder->LastThrowRejectReason, ERopeThrowRejectReason::InputSuppressed);

	Wielder->ThrowNow();
	TestEqual(TEXT("a committed montage-path throw is discarded too"), Wielder->BuildContextCalls, 0);

	Wielder->SetRopeInputSuppressed(false);
	Wielder->Throw();
	TestEqual(TEXT("clearing suppression restores the throw"), Wielder->BuildContextCalls, 1);
	return true;
}

// The actual user-facing rule: a wielder whose owner is ragdolled, as a snare leaves it, discards rope
// input without the game having to call anything.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderRagdollSuppressesRopeInputTest,
	"DynamicRope.Wielder.Input.RagdollSuppressesRopeInput",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderRagdollSuppressesRopeInputTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	Rope->ResolveMode = ERopeWrapResolveMode::AssistedJudged;
	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>();
	Wielder->Rope = Rope;
	URopeRagdollResponseComponent* Response = NewObject<URopeRagdollResponseComponent>();
	FRopeWielderComponentTestSeam::SetRagdollResponse(*Wielder, Response);

	TestFalse(TEXT("an upright owner is not suppressed"), Wielder->IsRopeInputSuppressed());

	FRopeWielderComponentTestSeam::SetRagdolled(*Response, true);
	TestTrue(TEXT("a ragdolled owner suppresses rope input"), Wielder->IsRopeInputSuppressed());
	TestFalse(TEXT("a ragdolled owner cannot aim"), Wielder->IsAimActive());
	Wielder->StartPull();
	TestFalse(TEXT("a ragdolled owner cannot arm Pull"), Wielder->IsPullArmed());

	// Opting out returns a game that has its own held-state rules to the previous behaviour.
	Wielder->bSuppressInputWhileRagdolled = false;
	TestFalse(TEXT("opting out ignores the ragdoll"), Wielder->IsRopeInputSuppressed());
	TestTrue(TEXT("opting out restores aiming"), Wielder->IsAimActive());

	Wielder->bSuppressInputWhileRagdolled = true;
	FRopeWielderComponentTestSeam::SetRagdolled(*Response, false);
	TestFalse(TEXT("recovering from the ragdoll resumes rope input"), Wielder->IsRopeInputSuppressed());
	TestTrue(TEXT("recovering from the ragdoll resumes aiming"), Wielder->IsAimActive());
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
	const FScopedSyncPhysicsForTest SyncPhysics;
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderSimulatedOwnerBindsChaosDirectlyTest,
	"DynamicRope.Movement.HardLeash.SimulatedOwnerBindsChaosDirectly",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderSimulatedOwnerBindsChaosDirectlyTest::RunTest(const FString& Parameters)
{
	// A Wielder whose own root simulates — a Chaos vehicle, a physics prop — is not a movement adapter. The
	// hard projection must stand down for it (moving a simulated body from the game thread is discarded by the
	// next physics sync, and recording the attempt would suppress a reaction it never received), and the
	// tether must bind that body directly instead of hiding behind an infinite-mass kinematic proxy.
	const FScopedSyncPhysicsForTest SyncPhysics;
	FTestWorldWrapper WorldWrapper;
	if (!WorldWrapper.CreateTestWorld(EWorldType::Game) || !WorldWrapper.BeginPlayInTestWorld())
	{
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	UWorld* World = WorldWrapper.GetTestWorld();
	AActor* TargetActor = World->SpawnActor<AActor>();
	APawn* Vehicle = World->SpawnActor<APawn>();
	if (!TestNotNull(TEXT("simulated-owner fixture spawned target"), TargetActor) ||
		!TestNotNull(TEXT("simulated-owner fixture spawned pawn"), Vehicle))
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

	USphereComponent* VehicleRoot = NewObject<USphereComponent>(Vehicle);
	Vehicle->AddInstanceComponent(VehicleRoot);
	Vehicle->SetRootComponent(VehicleRoot);
	VehicleRoot->InitSphereRadius(10.0f);
	VehicleRoot->SetMobility(EComponentMobility::Movable);
	VehicleRoot->RegisterComponent();
	// Start 10 cm outside the 60 cm boundary, so a hard projection would fire on the very first tick.
	Vehicle->SetActorLocation(FVector(-10.0f, 0.0f, 0.0f));
	VehicleRoot->SetCollisionEnabled(ECollisionEnabled::PhysicsOnly);
	VehicleRoot->SetSimulatePhysics(true);
	VehicleRoot->SetEnableGravity(false);

	URopeComponent* Rope = NewObject<URopeComponent>(Vehicle);
	Vehicle->AddInstanceComponent(Rope);
	Rope->SetupAttachment(VehicleRoot);
	FRopeWielderComponentTestSeam::ConfigureExternalHardLeash(
		*Rope, TargetRoot, /*AnchorNode*/ 3, /*RopeLength*/ 60.0f);
	Rope->RegisterComponent();

	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>(Vehicle);
	Vehicle->AddInstanceComponent(Wielder);
	Wielder->Rope = Rope;
	Wielder->bAutoBindInput = false;
	Wielder->bAttachOnBeginPlay = false;
	Wielder->bShowThrowPreview = false;
	Wielder->bAutoGroundExitOnUpwardPull = false;
	Wielder->bBoostAirControlWhileSwinging = false;
	Wielder->RegisterComponent();

	TestTrue(TEXT("a simulating owner root is recognised as a physical Wielder end"),
		Rope->IsWielderPhysicallySimulated());

	World->Tick(LEVELTICK_All, 1.0f / 60.0f);

	TestTrue(TEXT("a simulated Wielder is never hard-projected, so no attempt is ever recorded"),
		FRopeWielderComponentTestSeam::GetWielderAttemptFrame(*Rope) == MAX_uint64);
	TestTrue(TEXT("the tether's hand side is the owner's own simulating body"),
		FRopeWielderComponentTestSeam::GetPhysicalTetherWielderBody(*Rope) ==
			static_cast<const UPrimitiveComponent*>(VehicleRoot));
	TestFalse(TEXT("a directly bound hand side creates no kinematic proxy"),
		FRopeWielderComponentTestSeam::HasPhysicalTetherProxy(*Rope));
	TestTrue(TEXT("the directly bound tether still uses the hard material limit"),
		FMath::IsNearlyEqual(
			FRopeWielderComponentTestSeam::GetPhysicalTetherLimit(*Rope), 60.0f, 0.1f));
	TestEqual(TEXT("a simulated pair resolves through the Chaos backend"),
		FRopeWielderComponentTestSeam::GetLengthConstraintBackend(*Rope),
		ERopeLengthConstraintBackend::Chaos);
	// The point of binding directly: both ends are dynamic, so the owner is drawn in too instead of standing
	// still while only the target is winched, which is all an infinite-mass proxy could ever produce.
	TestTrue(TEXT("a directly bound owner is actually pulled toward the anchor"),
		Vehicle->GetActorLocation().X > -9.99f);

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
	const FScopedSyncPhysicsForTest SyncPhysics;
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderWrappingTetherActiveTest,
	"DynamicRope.Wielder.Movement.WrappingCountsAsExternalTether",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderWrappingTetherActiveTest::RunTest(const FString& Parameters)
{
	// An in-progress wrap already clamps the wielder through the hand-side hard boundary, so the
	// tether gate accepts Wrapping. The target share is a Wrapped-phase observation and stays at its
	// default of 1 here, which must not block the gate; a wrap forming on the wielder's own actor is
	// still a self-wrap, read from the capture candidate because the wrap state has not committed.
	AActor* Owner = NewObject<AActor>();
	AActor* Other = NewObject<AActor>();
	URopeComponent* Rope = NewObject<URopeComponent>(Owner);
	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>(Owner);
	Wielder->Rope = Rope;

	USceneComponent* ExternalMesh = NewObject<USceneComponent>(Other);
	FRopeWielderComponentTestSeam::ForceWrappingPhase(*Rope, ExternalMesh);
	TestTrue(TEXT("a wrap in progress on another actor is already an external tether"),
		FRopeWielderComponentTestSeam::IsWielderTetherActive(*Wielder));

	USceneComponent* OwnMesh = NewObject<USceneComponent>(Owner);
	FRopeWielderComponentTestSeam::ForceWrappingPhase(*Rope, OwnMesh);
	TestFalse(TEXT("a wrap forming on the wielder's own actor is a self-wrap, not an external tether"),
		FRopeWielderComponentTestSeam::IsWielderTetherActive(*Wielder));
	return true;
}

// The hang verdict must enter the instant its gates hold, ride out brief gate flicker (the mid-swing
// Hang -> Falling -> Hang animation pop), reject the jump-while-dragging false positive at entry, and
// still end immediately when the character stops falling. The latch is driven directly through the
// seam; the wielder tick is not simulated here.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderHangLatchTest,
	"DynamicRope.Wielder.HangEntersInstantlyAndExitsWithGrace",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderHangLatchTest::RunTest(const FString& Parameters)
{
	FTestWorldWrapper WorldWrapper;
	if (!WorldWrapper.CreateTestWorld(EWorldType::Game) ||
		!WorldWrapper.BeginPlayInTestWorld())
	{
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}

	UWorld* World = WorldWrapper.GetTestWorld();
	ACharacter* Character = World->SpawnActor<ACharacter>();
	if (!TestNotNull(TEXT("hang fixture character spawned"), Character))
	{
		WorldWrapper.DestroyTestWorld(false);
		WorldWrapper.ForwardErrorMessages(this);
		return false;
	}
	URopeComponent* Rope = NewObject<URopeComponent>(Character);
	Character->AddInstanceComponent(Rope);
	Rope->RegisterComponent();
	URopeWielderComponent* Wielder = AddTestWielder(*Character, *Rope);
	Character->GetCharacterMovement()->SetMovementMode(MOVE_Falling);
	Wielder->HangExitGraceTime = 0.3f;

	// A taut horizontal leg while airborne is the jump-while-dragging case: never an entry.
	FRopeWielderComponentTestSeam::ForceExternalWrapHangState(
		*Rope, /*DirToHand*/ FVector(-1.0f, 0.0f, 0.0f), /*Tension*/ 50000.0f);
	FRopeWielderComponentTestSeam::UpdateHangLatch(*Wielder, 0.016f);
	TestFalse(TEXT("a taut horizontal rope does not become a hang when the character jumps"),
		Wielder->IsHangingOnRope());

	// An upward leg enters on the very first update — the catch of a fall must not lag.
	FRopeWielderComponentTestSeam::ForceExternalWrapHangState(
		*Rope, FVector(0.0f, 0.0f, -1.0f), 50000.0f);
	FRopeWielderComponentTestSeam::UpdateHangLatch(*Wielder, 0.016f);
	TestTrue(TEXT("airborne under an upward taut rope is a hang, immediately"),
		Wielder->IsHangingOnRope());

	// A swing extreme dips the direction gate for a moment: inside the grace the verdict holds…
	FRopeWielderComponentTestSeam::ForceExternalWrapHangState(
		*Rope, FVector(-1.0f, 0.0f, 0.0f), 50000.0f);
	FRopeWielderComponentTestSeam::UpdateHangLatch(*Wielder, 0.1f);
	TestTrue(TEXT("a brief gate dip inside the exit grace keeps the hang"),
		Wielder->IsHangingOnRope());
	// …and only a sustained false ends it.
	FRopeWielderComponentTestSeam::UpdateHangLatch(*Wielder, 0.4f);
	TestFalse(TEXT("gates false past the exit grace end the hang"),
		Wielder->IsHangingOnRope());

	// Re-entry after a full exit is again immediate.
	FRopeWielderComponentTestSeam::ForceExternalWrapHangState(
		*Rope, FVector(0.0f, 0.0f, -1.0f), 50000.0f);
	FRopeWielderComponentTestSeam::UpdateHangLatch(*Wielder, 0.016f);
	TestTrue(TEXT("the hang re-enters immediately after a full exit"),
		Wielder->IsHangingOnRope());

	// The optional load gate: enabled, an unloaded rope is fallen past, not hung from.
	Wielder->HangMinTension = 1000.0f;
	FRopeWielderComponentTestSeam::ForceExternalWrapHangState(
		*Rope, FVector(0.0f, 0.0f, -1.0f), 0.0f);
	FRopeWielderComponentTestSeam::UpdateHangLatch(*Wielder, 0.4f);
	TestFalse(TEXT("with the load gate enabled an unloaded rope is not hung from"),
		Wielder->IsHangingOnRope());
	Wielder->HangMinTension = 0.0f;

	// No longer falling ends the hang immediately, with no grace. Flying stands in for a landing:
	// SetMovementMode(MOVE_Walking) can bounce straight back to Falling without a floor under the capsule.
	FRopeWielderComponentTestSeam::ForceExternalWrapHangState(
		*Rope, FVector(0.0f, 0.0f, -1.0f), 50000.0f);
	FRopeWielderComponentTestSeam::UpdateHangLatch(*Wielder, 0.016f);
	TestTrue(TEXT("hanging again ahead of the landing check"),
		Wielder->IsHangingOnRope());
	Character->GetCharacterMovement()->SetMovementMode(MOVE_Flying);
	TestFalse(TEXT("leaving Falling ends the hang immediately, with no grace"),
		Wielder->IsHangingOnRope());

	WorldWrapper.DestroyTestWorld(false);
	WorldWrapper.ForwardErrorMessages(this);
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

// The grip point sampler behind GetHangAnimSample: an arc-length walk along the centerline from node
// zero, which is what places the free hand's IK effector on the actual rope curve rather than on a
// straight-line offset from the hand.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderHangGripSampleTest,
	"DynamicRope.Wielder.HangGripSampleWalksArcLength",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderHangGripSampleTest::RunTest(const FString& Parameters)
{
	// A bent centerline: 50 cm straight down, then 70 cm sideways. Segment lengths differ on purpose so
	// a walk that assumed uniform spacing would land on the wrong point.
	const TArray<FVector> Positions = {
		FVector(0.0f, 0.0f, 0.0f),
		FVector(0.0f, 0.0f, -50.0f),
		FVector(70.0f, 0.0f, -50.0f)
	};

	TestTrue(TEXT("a mid-segment arc length lerps inside the first segment"),
		FRopeWielderComponentTestSeam::SampleCenterlineAtArcLength(Positions, 25.0f)
			.Equals(FVector(0.0f, 0.0f, -25.0f), 0.01f));
	TestTrue(TEXT("an arc length past the first segment continues into the second"),
		FRopeWielderComponentTestSeam::SampleCenterlineAtArcLength(Positions, 60.0f)
			.Equals(FVector(10.0f, 0.0f, -50.0f), 0.01f));
	TestTrue(TEXT("an arc length beyond the rope clamps to the last node"),
		FRopeWielderComponentTestSeam::SampleCenterlineAtArcLength(Positions, 999.0f)
			.Equals(Positions.Last(), 0.01f));
	TestTrue(TEXT("a negative arc length clamps to node zero"),
		FRopeWielderComponentTestSeam::SampleCenterlineAtArcLength(Positions, -5.0f)
			.Equals(Positions[0], 0.01f));

	// Degenerate inputs: an empty centerline yields zero, and a chain of coincident nodes returns that
	// point instead of dividing by a zero segment length.
	TestTrue(TEXT("an empty centerline yields zero"),
		FRopeWielderComponentTestSeam::SampleCenterlineAtArcLength({}, 10.0f).IsNearlyZero());
	const TArray<FVector> Coincident = { FVector(5.0f, 5.0f, 5.0f), FVector(5.0f, 5.0f, 5.0f) };
	TestTrue(TEXT("coincident nodes return the point itself"),
		FRopeWielderComponentTestSeam::SampleCenterlineAtArcLength(Coincident, 0.0f)
			.Equals(Coincident[0], 0.01f));

	return true;
}

// A taut Wrapped hold renders through the taut presentation shaping (straightening plus thrum), so the
// hang sample must place the free hand on the shaped curve. Sampling the solved positions instead left
// the hand hanging visibly short of the tube — off by exactly the straightened sag.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderHangSampleMatchesRenderedRopeTest,
	"DynamicRope.Wielder.HangSampleMatchesRenderedRope",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderHangSampleMatchesRenderedRopeTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>();
	Wielder->Rope = Rope;

	// A sagging span from the hand to the anchor at node 4, plus a free tail node. The sag stays under
	// the corner-guard fade so the straightening runs at full strength.
	const TArray<FVector> Sagging = {
		FVector(0.0f, 0.0f, 0.0f),
		FVector(25.0f, 0.0f, -6.0f),
		FVector(50.0f, 0.0f, -8.0f),
		FVector(75.0f, 0.0f, -6.0f),
		FVector(100.0f, 0.0f, 0.0f),
		FVector(120.0f, 0.0f, -10.0f)
	};
	FRopeWielderComponentTestSeam::SetSimPositions(*Rope, Sagging);
	const FVector RawGrip = FRopeWielderComponentTestSeam::SampleCenterlineAtArcLength(
		Sagging, Wielder->OffHandGripDistance);

	// Before the hold: no shaping, the sample rides the solved curve.
	TestTrue(TEXT("without a taut hold the grip rides the solved curve"),
		Wielder->GetHangAnimSample().OffHandGripWorld.Equals(RawGrip, 0.01f));

	FRopeWielderComponentTestSeam::SetTautPresentationState(*Rope, /*AnchorNode*/ 4, /*Blend*/ 1.0f);
	TArray<FVector> Shaped = Sagging;
	TestTrue(TEXT("the fixture's shaping is active"), Rope->ApplyTautPresentationShaping(Shaped));
	const FVector ShapedGrip = FRopeWielderComponentTestSeam::SampleCenterlineAtArcLength(
		Shaped, Wielder->OffHandGripDistance);

	const FRopeHangAnimSample Sample = Wielder->GetHangAnimSample();
	TestTrue(TEXT("under a taut hold the grip rides the shaped curve"),
		Sample.OffHandGripWorld.Equals(ShapedGrip, 0.01f));
	TestTrue(TEXT("the shaped grip actually differs from the solved curve"),
		!Sample.OffHandGripWorld.Equals(RawGrip, 1.0f));
	TestTrue(TEXT("the hand end is unmoved by the shaping"), Sample.HandWorld.Equals(Sagging[0], 0.001f));

	return true;
}

// The hang sample's one-frame prediction: the anim update runs before the rope's own tick, so the
// sample extrapolates every node by its last Verlet displacement. Without it the grip hand trails the
// drawn rope through a swing by exactly one frame of rope motion.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWielderHangSamplePredictsOneFrameTest,
	"DynamicRope.Wielder.HangSamplePredictsOneFrame",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWielderHangSamplePredictsOneFrameTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	URopeWielderComponent* Wielder = NewObject<URopeWielderComponent>();
	Wielder->Rope = Rope;

	// A straight vertical rope that moved +5 in X over the last frame.
	const TArray<FVector> Positions = {
		FVector(0.0f, 0.0f, 0.0f),
		FVector(0.0f, 0.0f, 25.0f),
		FVector(0.0f, 0.0f, 50.0f),
		FVector(0.0f, 0.0f, 75.0f)
	};
	FRopeWielderComponentTestSeam::SetSimPositions(*Rope, Positions);
	TArray<FVector> Prev = Positions;
	for (FVector& P : Prev)
	{
		P.X -= 5.0f;
	}
	FRopeWielderComponentTestSeam::SetSimPrevPositions(*Rope, Prev);

	const FRopeHangAnimSample Sample = Wielder->GetHangAnimSample();
	// Half-strength prediction: half of the 5 cm displacement.
	TestTrue(TEXT("the hand end is extrapolated half a displacement forward"),
		Sample.HandWorld.Equals(FVector(2.5f, 0.0f, 0.0f), 0.01f));
	TestTrue(TEXT("the grip point is extrapolated the same way"),
		Sample.OffHandGripWorld.Equals(
			FVector(2.5f, 0.0f, Wielder->OffHandGripDistance), 0.01f));

	// A still rope (Prev == Pos) is sampled exactly — prediction adds nothing at rest.
	FRopeWielderComponentTestSeam::SetSimPositions(*Rope, Positions);
	TestTrue(TEXT("a still rope is sampled exactly"),
		Wielder->GetHangAnimSample().HandWorld.Equals(Positions[0], 0.001f));

	return true;
}

// The hang grip pin: the rope follows the animated hand rather than the hand chasing simulated nodes,
// via the same per-frame override the wrapped hold uses. The pinned node also bounds the taut
// presentation span from below, so the drawn rope keeps passing through the gripping hand.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeHangGripPinFollowsAnimatedHandTest,
	"DynamicRope.Wielder.HangGripPinFollowsAnimatedHand",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeHangGripPinFollowsAnimatedHandTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	USceneComponent* Hand = NewObject<USceneComponent>();
	Hand->SetRelativeLocation(FVector(30.0f, 10.0f, 5.0f));

	const TArray<FVector> Sagging = {
		FVector(0.0f, 0.0f, 0.0f),
		FVector(25.0f, 0.0f, -6.0f),
		FVector(50.0f, 0.0f, -8.0f),
		FVector(75.0f, 0.0f, -6.0f),
		FVector(100.0f, 0.0f, 0.0f),
		FVector(120.0f, 0.0f, -10.0f)
	};
	FRopeWielderComponentTestSeam::SetSimPositions(*Rope, Sagging);
	FRopeWielderComponentTestSeam::SetTautPresentationState(*Rope, /*AnchorNode*/ 4, /*Blend*/ 1.0f);

	// Pin node 2: the override carries the hand's socket position with zero inverse mass.
	Rope->SetHangGripPin(Hand, NAME_None, 2);
	FRopeWielderComponentTestSeam::ApplyHangGripPin(*Rope);
	TestEqual(TEXT("the requested node is pinned"), Rope->GetHangGripPinNode(), 2);
	TestTrue(TEXT("the pinned node is overridden to the hand socket"),
		FRopeWielderComponentTestSeam::GetOverridePosition(*Rope, 2)
			.Equals(Hand->GetSocketLocation(NAME_None), 0.01f));
	TestEqual(TEXT("the pinned node's inverse mass is zero"),
		FRopeWielderComponentTestSeam::GetOverrideInvMass(*Rope, 2), 0.0f, 0.0001f);

	// The interior between the hands is draped kinematically: on the hand-to-grip chord, dropped by
	// the slack-derived parabolic sag, with zero inverse mass — not left to the solver.
	{
		const FVector HandWorld = Rope->GetComponentLocation();
		const FVector GripWorld = Hand->GetSocketLocation(NAME_None);
		const float SpanRest = 2.0f * FRopeWielderComponentTestSeam::GetSegmentLength(*Rope);
		const float ChordLen = FVector::Dist(HandWorld, GripWorld);
		const float Sag = FMath::Sqrt(3.0f * ChordLen * FMath::Max(SpanRest - ChordLen, 0.0f) / 8.0f);
		FVector ExpectedDrape = FMath::Lerp(HandWorld, GripWorld, 0.5f);
		ExpectedDrape.Z -= Sag;
		TestTrue(TEXT("the interior node is draped on the sagging chord"),
			FRopeWielderComponentTestSeam::GetOverridePosition(*Rope, 1).Equals(ExpectedDrape, 0.01f));
		TestEqual(TEXT("the draped node's inverse mass is zero"),
			FRopeWielderComponentTestSeam::GetOverrideInvMass(*Rope, 1), 0.0f, 0.0001f);
	}

	// The shaped span starts at the pin: below it the drape stands, above it the interior node lands
	// on the pin-to-anchor chord.
	TArray<FVector> Shaped = Sagging;
	TestTrue(TEXT("shaping still runs with a pin"), Rope->ApplyTautPresentationShaping(Shaped));
	TestTrue(TEXT("the node below the pin is untouched by shaping"), Shaped[1].Equals(Sagging[1], 0.001f));
	TestTrue(TEXT("the pinned node itself is untouched by shaping"), Shaped[2].Equals(Sagging[2], 0.001f));
	TestTrue(TEXT("the node above the pin lands on the pin-to-anchor chord"),
		Shaped[3].Equals(FVector(75.0f, 0.0f, -4.0f), 0.01f));

	// An out-of-range request clamps against the wrap; the old span is rewritten as part of the new
	// one, so node 2 turns from the grip into a draped interior.
	Rope->SetHangGripPin(Hand, NAME_None, 9);
	FRopeWielderComponentTestSeam::ApplyHangGripPin(*Rope);
	TestEqual(TEXT("the request clamps below the first wrapped node"), Rope->GetHangGripPinNode(), 3);
	TestEqual(TEXT("the old grip node becomes a draped interior"),
		FRopeWielderComponentTestSeam::GetOverrideInvMass(*Rope, 2), 0.0f, 0.0001f);

	// Clearing hands the whole span back to the solver on the next frame.
	Rope->ClearHangGripPin();
	FRopeWielderComponentTestSeam::ApplyHangGripPin(*Rope);
	TestEqual(TEXT("clearing deactivates the pin"), Rope->GetHangGripPinNode(),
		static_cast<int32>(INDEX_NONE));
	TestEqual(TEXT("the cleared interior gets its mass back"),
		FRopeWielderComponentTestSeam::GetOverrideInvMass(*Rope, 1), 1.0f, 0.0001f);
	TestEqual(TEXT("the cleared grip gets its mass back"),
		FRopeWielderComponentTestSeam::GetOverrideInvMass(*Rope, 3), 1.0f, 0.0001f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
