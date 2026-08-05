// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Subsystem/RopeDebugSubsystem.h"
#include "RopeComponent.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"

URopeDebugSubsystem* URopeDebugSubsystem::Get(const UWorld* World)
{
	return World ? World->GetSubsystem<URopeDebugSubsystem>() : nullptr;
}

bool URopeDebugSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
	// In a game or in play-in-editor alone, as with the simulation subsystem, which excludes editor preview and inspector worlds.
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

void URopeDebugSubsystem::Tick(float DeltaTime)
{
#if WITH_GAMEPLAY_DEBUGGER
	if (Snapshots.Num() == 0 && HeldFlight.Num() == 0)
	{
		return;
	}

	// If the category has not drawn recently, meaning within the active frame window, it is inactive and the whole
	// store, including the hold store, is cleared so that the last array does not survive until the world shuts down
	// after submissions stop.
	if (GFrameCounter - LastActiveFrame > ActiveFrameWindow)
	{
		Snapshots.Reset();
		HeldFlight.Reset();
		return;
	}

	// Cleaning up stale and invalid entries while active happens here alone, once per frame, which removes the walk of
	// the whole map on every SubmitSnapshot.
	for (auto It = Snapshots.CreateIterator(); It; ++It)
	{
		if (!It.Key().IsValid() || GFrameCounter - It.Value().FrameStamp > ActiveFrameWindow)
		{
			It.RemoveCurrent();
		}
	}

	// The hold store expires against the real-time window of FlightHoldSeconds, and invalid keys are removed with it.
	if (HeldFlight.Num() > 0)
	{
		const UWorld* World = GetWorld();
		const double Now = World ? World->GetRealTimeSeconds() : 0.0;
		for (auto It = HeldFlight.CreateIterator(); It; ++It)
		{
			if (!It.Key().IsValid() || Now - It.Value().RealTimeSeconds > FlightHoldSeconds)
			{
				It.RemoveCurrent();
			}
		}
	}
#endif
}

TStatId URopeDebugSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(URopeDebugSubsystem, STATGROUP_Tickables);
}

bool URopeDebugSubsystem::IsTickable() const
{
	// Debug-only lifetime management, so there is nothing to tick in a build where WITH_GAMEPLAY_DEBUGGER is off.
#if WITH_GAMEPLAY_DEBUGGER
	return true;
#else
	return false;
#endif
}

#if WITH_GAMEPLAY_DEBUGGER

void URopeDebugSubsystem::SetTarget(AActor* InActor, ERopeDebugCapture InCaptureMask)
{
	TargetActor = InActor;
	CaptureMask = InCaptureMask;
	LastActiveFrame = GFrameCounter;
}

ERopeDebugCapture URopeDebugSubsystem::GetCaptureMask() const
{
	return CaptureMask;
}

bool URopeDebugSubsystem::ShouldCapture(const URopeComponent* Rope) const
{
	if (!Rope)
	{
		return false;
	}

	// Whether the category drew recently, meaning within the active frame window, which is what marks the debugger and the category active.
	if (GFrameCounter - LastActiveFrame > ActiveFrameWindow)
	{
		return false;
	}

	const AActor* Target = TargetActor.Get();
	return Target != nullptr && Rope->GetOwner() == Target;
}

void URopeDebugSubsystem::SubmitSnapshot(const URopeComponent* Rope, FRopeDebugSnapshot&& Snapshot)
{
	if (!Rope)
	{
		return;
	}
	Snapshot.FrameStamp = GFrameCounter;
	// The flight overlay is overwritten and lost by the next frame's snapshot, taken while wrapping, so a snapshot
	// carrying flight data is copied and held separately for FlightHoldSeconds of real time, which keeps what the rope
	// decided to catch visible just after the decision. The copy is taken first because the MoveTemp below consumes
	// the original.
	if (Snapshot.bHasFlight)
	{
		FHeldFlightSnapshot& Held = HeldFlight.FindOrAdd(Rope);
		Held.Snapshot = Snapshot;
		const UWorld* World = GetWorld();
		Held.RealTimeSeconds = World ? World->GetRealTimeSeconds() : 0.0;
	}
	Snapshots.Add(Rope, MoveTemp(Snapshot));
	// Stale and invalid entries are cleaned up once per frame by the tick, so the whole map is not walked on every submission here.
}

const FRopeDebugSnapshot* URopeDebugSubsystem::GetSnapshot(const URopeComponent* Rope) const
{
	const FRopeDebugSnapshot* Found = Snapshots.Find(Rope);
	if (!Found || GFrameCounter - Found->FrameStamp > ActiveFrameWindow)
	{
		return nullptr;
	}
	return Found;
}

const FRopeDebugSnapshot* URopeDebugSubsystem::GetHeldFlightSnapshot(const URopeComponent* Rope, float& OutAgeSeconds) const
{
	OutAgeSeconds = 0.0f;
	const FHeldFlightSnapshot* Found = HeldFlight.Find(Rope);
	if (!Found)
	{
		return nullptr;
	}
	const UWorld* World = GetWorld();
	const double Now = World ? World->GetRealTimeSeconds() : 0.0;
	const double Age = Now - Found->RealTimeSeconds;
	if (Age < 0.0 || Age > FlightHoldSeconds)
	{
		return nullptr;
	}
	OutAgeSeconds = static_cast<float>(Age);
	return &Found->Snapshot;
}

#else // !WITH_GAMEPLAY_DEBUGGER — in a build with debugging disabled, everything is a no-op.

void URopeDebugSubsystem::SetTarget(AActor*, ERopeDebugCapture) {}
bool URopeDebugSubsystem::ShouldCapture(const URopeComponent*) const { return false; }
ERopeDebugCapture URopeDebugSubsystem::GetCaptureMask() const { return ERopeDebugCapture::None; }
void URopeDebugSubsystem::SubmitSnapshot(const URopeComponent*, FRopeDebugSnapshot&&) {}
const FRopeDebugSnapshot* URopeDebugSubsystem::GetSnapshot(const URopeComponent*) const { return nullptr; }
const FRopeDebugSnapshot* URopeDebugSubsystem::GetHeldFlightSnapshot(const URopeComponent*, float& OutAgeSeconds) const { OutAgeSeconds = 0.0f; return nullptr; }

#endif // WITH_GAMEPLAY_DEBUGGER
