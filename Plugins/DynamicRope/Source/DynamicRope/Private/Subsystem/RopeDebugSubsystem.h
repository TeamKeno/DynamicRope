// Copyright Epic Games, Inc. All Rights Reserved.
//
// The single store for rope debugging. Every debug entry point is FGameplayDebuggerCategory_Rope, and this subsystem
// relays between that category and the simulation tick: the category registers the actor being debugged through
// SetTarget, the simulation tick on the game thread captures that actor's ropes alone, deciding through
// ShouldCapture, and submits a snapshot through SubmitSnapshot, which the category reads back through GetSnapshot and
// draws. A rope that is not the target never pays a capture cost such as the flight sweep at all.
//
// Being a debug-only feature, the substance compiles only under WITH_GAMEPLAY_DEBUGGER, leaving an empty shell in a
// shipping build.
//
// It lives in Private because it is internal to the module. Its API surface exposes FRopeDebugSnapshot and
// ERopeDebugCapture from Private/Debug directly, so making it public would mean publishing those types too, while its
// only users are this module's private sources: the category, the rope component and this subsystem's implementation.
// If an external module ever needs it, whether the snapshot types should be public has to be decided first, since
// publishing them now would make the debug data structures a de facto part of the plugin's API.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Debug/RopeDebugSnapshot.h"
#include "RopeDebugSubsystem.generated.h"

class URopeComponent;
class AActor;

UCLASS()
class URopeDebugSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	/** The world's rope debug subsystem, which is valid in a game or in play-in-editor and null otherwise. */
	static URopeDebugSubsystem* Get(const UWorld* World);

	/** Called every time the gameplay debugger category draws: it registers the actor currently being debugged and
	 *  the capture scope of the enabled views, and marks the category active for this frame. The mask is read by the
	 *  next simulation tick, so a toggle takes effect one frame later. */
	void SetTarget(AActor* InActor, ERopeDebugCapture InCaptureMask);

	/** Whether to capture this rope for debugging this frame, which is true only when the category was recently active and the rope's owner is the target actor. */
	bool ShouldCapture(const URopeComponent* Rope) const;

	/** The sections to fill this frame, used to decide which collections a captured rope may skip. */
	ERopeDebugCapture GetCaptureMask() const;

	/** Submits a snapshot filled in by the simulation tick on the game thread, called for target ropes alone. */
	void SubmitSnapshot(const URopeComponent* Rope, FRopeDebugSnapshot&& Snapshot);

	/** Read by the category when it draws. Null if there is no snapshot or it is too old, as after the target is cleared. */
	const FRopeDebugSnapshot* GetSnapshot(const URopeComponent* Rope) const;

	/** The last held flight snapshot, within a real-time window of FlightHoldSeconds. Null if there is none, otherwise
	 *  the elapsed seconds are written to OutAgeSeconds. Used to keep the flight overlay on screen just after leaving Flight. */
	const FRopeDebugSnapshot* GetHeldFlightSnapshot(const URopeComponent* Rope, float& OutAgeSeconds) const;

	//~ UWorldSubsystem
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;

	//~ FTickableGameObject (UTickableWorldSubsystem) — runs snapshot lifetime management once per frame: while active
	//   it cleans up stale and invalid entries alone, which removes the walk of the whole map on every submission,
	//   and when the category is inactive it clears the store entirely, so that the last array does not survive until
	//   the world shuts down after submissions stop. It ticks in debug builds alone.
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;
	virtual bool IsTickable() const override;

private:
#if WITH_GAMEPLAY_DEBUGGER
	// The window, in frames, for deciding activity and staleness. The order of the category's CollectData and this
	// subsystem's tick within a frame is not determined, so a frame or two of slack keeps the capture from breaking.
	static constexpr uint64 ActiveFrameWindow = 4;

	TWeakObjectPtr<AActor> TargetActor;
	uint64 LastActiveFrame = 0;
	ERopeDebugCapture CaptureMask = ERopeDebugCapture::None;
	TMap<TWeakObjectPtr<const URopeComponent>, FRopeDebugSnapshot> Snapshots;

	// Holding the last flight snapshot: a frame with flight data is overwritten and lost by the next frame's snapshot,
	// taken while wrapping, so it is held separately for FlightHoldSeconds of real time to keep the flight overlay at
	// the position where the decision was made.
	struct FHeldFlightSnapshot
	{
		FRopeDebugSnapshot Snapshot;
		// The capture time from World->GetRealTimeSeconds(), which is real time independent of slow motion and pausing, so the hold window is measured against the wall clock.
		double RealTimeSeconds = 0.0;
	};
	TMap<TWeakObjectPtr<const URopeComponent>, FHeldFlightSnapshot> HeldFlight;
	static constexpr double FlightHoldSeconds = 2.5;
#endif
};
