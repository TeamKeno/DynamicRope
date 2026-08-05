// Copyright 2026 TeamKeno. All Rights Reserved.

#include "RopeDragonFlightDemoComponent.h"

#include "Animation/AnimSequenceBase.h"
#include "Components/SkeletalMeshComponent.h"
#include "Core/RopeTypes.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Subsystem/RopeSimSubsystem.h"
#include "UObject/UObjectIterator.h"

// A demo-only log category, since the plugin's LogDynamicRope is not exported and cannot be used from the game module.
DEFINE_LOG_CATEGORY_STATIC(LogRopeDragonDemo, Log, All);

namespace RopeDragonDemo
{
	/** The howl duration, in seconds, used when no howl animation is assigned. */
	static constexpr float FallbackHowlDuration = 1.5f;

	/** Eases the range from zero to one in and out, which prevents a pop at the start and end of the ascent and descent. */
	static float Ease(float Alpha)
	{
		const float T = FMath::Clamp(Alpha, 0.0f, 1.0f);
		return T * T * (3.0f - 2.0f * T);
	}
}

URopeDragonFlightDemoComponent::URopeDragonFlightDemoComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void URopeDragonFlightDemoComponent::BeginPlay()
{
	Super::BeginPlay();

	if (const AActor* Owner = GetOwner())
	{
		AnchorLocation = Owner->GetActorLocation();
		AnchorYawDeg = static_cast<float>(Owner->GetActorRotation().Yaw);
	}

	if (!ResolveMesh())
	{
		UE_LOG(LogRopeDragonDemo, Warning, TEXT("[%s] DragonDemo: no skeletal mesh was found, so it moves without switching animations."),
			*GetNameSafe(GetOwner()));
	}

	PlayAnim(IdleAnim, /*bLoop*/ true);

	// Subscribes to the central world signal so the rope that will wrap it need not be known in advance, the same pattern as the ragdoll response component.
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		WrappedHandle = Sim->OnAnyRopeWrapped.AddUObject(this, &URopeDragonFlightDemoComponent::HandleAnyRopeWrapped);
	}
}

void URopeDragonFlightDemoComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->OnAnyRopeWrapped.Remove(WrappedHandle);
	}
	Super::EndPlay(EndPlayReason);
}

void URopeDragonFlightDemoComponent::HandleAnyRopeWrapped(const FRopeWrappedEventInfo& Info)
{
	const USkeletalMeshComponent* MyMesh = ResolveMesh();
	if (!bStartOnWrapped || !MyMesh || Info.Mesh.Get() != MyMesh)
	{
		return;
	}
	if (State != ERopeDragonDemoState::Idle || PendingTriggerTime >= 0.0f)
	{
		// Already triggered or waiting, so a second rope catching it does not restart the sequence.
		return;
	}

	UE_LOG(LogRopeDragonDemo, Log, TEXT("[%s] DragonDemo: a rope wrapped (bone %s); howling starts in %.2f s."),
		*GetNameSafe(GetOwner()), *Info.Bone.ToString(), TriggerDelay);
	PendingTriggerTime = FMath::Max(TriggerDelay, 0.0f);
}

void URopeDragonFlightDemoComponent::StartSequence()
{
	if (State != ERopeDragonDemoState::Idle)
	{
		return;
	}

	// The transform at the moment of triggering is the origin and basis for the whole sequence. Only the yaw is used, so the figure of eight is always parallel to the ground.
	if (const AActor* Owner = GetOwner())
	{
		AnchorLocation = Owner->GetActorLocation();
		AnchorYawDeg = static_cast<float>(Owner->GetActorRotation().Yaw);
	}
	PendingTriggerTime = -1.0f;
	AscendEndOffset = FVector::ZeroVector;

	EnterState(ERopeDragonDemoState::Howl);
}

void URopeDragonFlightDemoComponent::ResetSequence()
{
	PendingTriggerTime = -1.0f;
	AscendEndOffset = FVector::ZeroVector;
	State = ERopeDragonDemoState::Idle;
	TimeInState = 0.0f;
	ApplyPose(FVector::ZeroVector, 0.0f, 0.0f, 0.0f);
	PlayAnim(IdleAnim, /*bLoop*/ true);
	UE_LOG(LogRopeDragonDemo, Log, TEXT("[%s] DragonDemo: reset, returning to the trigger point."), *GetNameSafe(GetOwner()));
}

void URopeDragonFlightDemoComponent::EnterState(ERopeDragonDemoState NewState)
{
	State = NewState;
	TimeInState = 0.0f;

	switch (NewState)
	{
	case ERopeDragonDemoState::Howl:
		PlayAnim(HowlAnim, /*bLoop*/ false);
		break;
	case ERopeDragonDemoState::Thrash:
		PlayAnim(FlyAnim, /*bLoop*/ true);
		break;
	case ERopeDragonDemoState::Finished:
		PlayAnim(IdleAnim, /*bLoop*/ true);
		break;
	default:
		// The ascent and descent continue the flight loop started during the thrash, which avoids resetting its playback position.
		break;
	}

	UE_LOG(LogRopeDragonDemo, Log, TEXT("[%s] DragonDemo: stage -> %s"),
		*GetNameSafe(GetOwner()), *UEnum::GetValueAsString(NewState));
}

void URopeDragonFlightDemoComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!GetOwner() || DeltaTime <= 0.0f)
	{
		return;
	}

	if (PendingTriggerTime >= 0.0f)
	{
		PendingTriggerTime -= DeltaTime;
		if (PendingTriggerTime <= 0.0f)
		{
			StartSequence();
		}
		return;
	}

	if (State == ERopeDragonDemoState::Idle || State == ERopeDragonDemoState::Finished)
	{
		return;
	}

	TimeInState += DeltaTime;

	switch (State)
	{
	case ERopeDragonDemoState::Howl:
	{
	// Howling in place alone; the position stays pinned at the trigger point.
		ApplyPose(FVector::ZeroVector, 0.0f, 0.0f, 0.0f);
		if (TimeInState >= ResolveHowlDuration())
		{
			EnterState(ERopeDragonDemoState::Thrash);
		}
		break;
	}

	case ERopeDragonDemoState::Thrash:
	{
		// The figure of eight is a 1:2 Lissajous curve, sin u against sin 2u. Its long axis is the forward direction at
		// the moment of triggering and its crossing point is the trigger point.
		// The loop count is an integer, so u ends on a multiple of 2 pi and both the start and the end are at the origin, which joins smoothly to the stages before and after.
		const float Alpha = FMath::Clamp(TimeInState / FMath::Max(ThrashDuration, KINDA_SMALL_NUMBER), 0.0f, 1.0f);
		const float U = Alpha * 2.0f * PI * FMath::Max(ThrashLoops, 1);

		const float SinU = FMath::Sin(U);
		const float CosU = FMath::Cos(U);
		const float Sin2U = FMath::Sin(2.0f * U);
		const float Cos2U = FMath::Cos(2.0f * U);

		FVector Offset;
		Offset.X = ThrashLength * SinU;						// The forward axis.
		Offset.Y = ThrashWidth * Sin2U;						// The lateral axis, at twice the frequency, which makes the figure of eight.
		Offset.Z = ThrashBobHeight * (0.5f - 0.5f * Cos2U);	// A vertical bob starting and ending at zero.

		// The heading is the curve's tangent, so the body always faces the way it is going.
		const float TangentX = ThrashLength * CosU;
		const float TangentY = 2.0f * ThrashWidth * Cos2U;
		const float LocalYaw = FMath::RadiansToDegrees(FMath::Atan2(TangentY, TangentX));

	// The bank is greatest through the crossing, and its sign matches the turn direction, since the lateral term is positive through a right turn.
		const float Roll = ThrashBankDeg * Sin2U;

		ApplyPose(Offset, LocalYaw, 0.0f, Roll);

		if (Alpha >= 1.0f)
		{
			EnterState(ERopeDragonDemoState::Ascend);
		}
		break;
	}

	case ERopeDragonDemoState::Ascend:
	{
		const float Alpha = FMath::Clamp(TimeInState / FMath::Max(AscendDuration, KINDA_SMALL_NUMBER), 0.0f, 1.0f);
		const float E = RopeDragonDemo::Ease(Alpha);

		const FVector Offset(AscendForward * E, 0.0f, AscendHeight * E);
	// The pitch peaks in the middle and is zero at both ends, so the angle does not jump when it joins the descent.
		const float Pitch = ClimbPitchDeg * FMath::Sin(PI * Alpha);
		ApplyPose(Offset, 0.0f, Pitch, 0.0f);

		if (Alpha >= 1.0f)
		{
			AscendEndOffset = Offset;
			EnterState(ERopeDragonDemoState::Descend);
		}
		break;
	}

	case ERopeDragonDemoState::Descend:
	{
		const float Alpha = FMath::Clamp(TimeInState / FMath::Max(DescendDuration, KINDA_SMALL_NUMBER), 0.0f, 1.0f);
		const float E = RopeDragonDemo::Ease(Alpha);

		const FVector LandOffset = bReturnToStartOnLand
			? FVector::ZeroVector
			: FVector(AscendEndOffset.X, AscendEndOffset.Y, 0.0f);
		const FVector Offset = FMath::Lerp(AscendEndOffset, LandOffset, E);
		const float Pitch = -ClimbPitchDeg * FMath::Sin(PI * Alpha);
		ApplyPose(Offset, 0.0f, Pitch, 0.0f);

		if (Alpha >= 1.0f)
		{
			EnterState(ERopeDragonDemoState::Finished);
		}
		break;
	}

	default:
		break;
	}
}

void URopeDragonFlightDemoComponent::ApplyPose(const FVector& LocalOffset, float YawDeg, float PitchDeg, float RollDeg)
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	const FRotator AnchorBasis(0.0f, AnchorYawDeg, 0.0f);
	const FVector World = AnchorLocation + AnchorBasis.RotateVector(LocalOffset);
	const FRotator Rotation(PitchDeg, AnchorYawDeg + YawDeg, RollDeg);

	// Presentation only: the path is written over directly, with no sweep, so displacement from the tether or a collision does not persist.
	Owner->SetActorLocationAndRotation(World, Rotation, /*bSweep*/ false);
}

float URopeDragonFlightDemoComponent::ResolveHowlDuration() const
{
	if (HowlDuration > 0.0f)
	{
		return HowlDuration;
	}
	if (HowlAnim && HowlAnim->GetPlayLength() > 0.0f)
	{
		return HowlAnim->GetPlayLength();
	}
	return RopeDragonDemo::FallbackHowlDuration;
}

void URopeDragonFlightDemoComponent::PlayAnim(UAnimSequenceBase* Anim, bool bLoop)
{
	USkeletalMeshComponent* Mesh = ResolveMesh();
	if (!Mesh || !Anim)
	{
		return;
	}
	// Played directly in single node mode with no animation Blueprint; PlayAnimation switches the mode as well.
	Mesh->PlayAnimation(Anim, bLoop);
}

USkeletalMeshComponent* URopeDragonFlightDemoComponent::ResolveMesh() const
{
	const AActor* Owner = GetOwner();
	return Owner ? Owner->FindComponentByClass<USkeletalMeshComponent>() : nullptr;
}

#if !UE_BUILD_SHIPPING
//======================================================================================
// Console commands for filming, applied to every actor in the world carrying this component.
//======================================================================================

namespace RopeDragonDemoConsole
{
	static void ForEach(UWorld* World, TFunctionRef<void(URopeDragonFlightDemoComponent&)> Fn)
	{
		int32 Count = 0;
		for (TObjectIterator<URopeDragonFlightDemoComponent> It; It; ++It)
		{
			URopeDragonFlightDemoComponent* Comp = *It;
			if (IsValid(Comp) && Comp->GetWorld() == World && IsValid(Comp->GetOwner()))
			{
				Fn(*Comp);
				++Count;
			}
		}
		if (Count == 0)
		{
			UE_LOG(LogRopeDragonDemo, Warning,
				TEXT("No actor has a URopeDragonFlightDemoComponent. Add the component to the dragon Blueprint."));
		}
	}

	static FAutoConsoleCommandWithWorldAndArgs GStartCmd(
		TEXT("Rope.Demo.Dragon"),
		TEXT("Starts the dragon demo sequence: howl, then a figure-of-eight thrash, then ascend and descend."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
		{
			ForEach(World, [](URopeDragonFlightDemoComponent& Comp)
			{
				Comp.StartSequence();
			});
		}));

	static FAutoConsoleCommandWithWorldAndArgs GResetCmd(
		TEXT("Rope.Demo.Dragon.Reset"),
		TEXT("Resets the dragon demo sequence, returning to the trigger point and idle, for repeated takes."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
		{
			ForEach(World, [](URopeDragonFlightDemoComponent& Comp)
			{
				Comp.ResetSequence();
			});
		}));
}
#endif // !UE_BUILD_SHIPPING
