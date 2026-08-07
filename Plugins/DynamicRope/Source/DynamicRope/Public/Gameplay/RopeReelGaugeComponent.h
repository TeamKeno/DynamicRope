// Copyright 2026 TeamKeno. All Rights Reserved.
//
// A world-space widget component that shows the reel gauge over the character while the rope's length
// is actually changing, and fades it out once it stops. On camera, a reel-in or reel-out with no
// visible input reads as a bug; this indicator marks the length change as intentional wherever the
// shot's camera is, which a viewport HUD cannot (view-target blends and cinematic cameras never render
// the local HUD).
//
// The trigger is the observed length change (GetCurrentRopeLength delta), not the reel input: gameplay
// reeling — a helicopter winch, a snare, a direct SetRopeLength — must light the gauge exactly like a
// held reel key, and a reel key held in a phase that defers reeling (Contacting/Wrapping/Releasing)
// must not claim the length is changing when it is not.
//
// Add it to the character (Blueprint or C++), position it like any scene component — the default sits
// off the right shoulder — and it wires itself: the rope resolves through the owner's URopeWielderComponent,
// with a plain component search as the fallback. The widget class defaults to the asset-free
// URopeReelGaugeWidget; point WidgetClass at a Blueprint subclass to restyle.

#pragma once

#include "CoreMinimal.h"
#include "Components/WidgetComponent.h"
#include "RopeReelGaugeComponent.generated.h"

class URopeComponent;
class URopePreset;
class URopeWielderComponent;

/**
 * The activity latch and fade behind the reel gauge, as pure data with no UObject involved so it can
 * be unit tested. Activity snaps the alpha to 1; from the last active frame the alpha blends linearly
 * to 0 over FadeOutTime, and stays 0 after.
 */
struct FRopeReelGaugeFadeState
{
	/** Seconds since the last observed activity. Starts far in the past, so a gauge that has never
	 *  seen activity is fully faded. */
	float TimeSinceActivity = BIG_NUMBER;

	/** Advances the latch one frame and returns the display alpha in [0, 1]. */
	float Update(bool bActiveThisFrame, float DeltaTime, float FadeOutTime)
	{
		if (bActiveThisFrame)
		{
			TimeSinceActivity = 0.0f;
			return 1.0f;
		}
		TimeSinceActivity = FMath::Min(TimeSinceActivity + FMath::Max(DeltaTime, 0.0f), BIG_NUMBER);
		if (FadeOutTime <= 0.0f)
		{
			return 0.0f;
		}
		return FMath::Clamp(1.0f - TimeSinceActivity / FadeOutTime, 0.0f, 1.0f);
	}
};

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeReelGaugeComponent : public UWidgetComponent
{
	GENERATED_BODY()

public:
	URopeReelGaugeComponent(const FObjectInitializer& ObjectInitializer);

	//~ UActorComponent
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/** How long the gauge blends out after the length stops changing (s). Reel activity during the
	 *  blend snaps it back to fully visible. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge", meta = (ClampMin = "0.0", Units = "s"))
	float FadeOutTime = 3.0f;

	/** Whether the gauge turns to face the local player's camera each frame. The camera manager's
	 *  location follows view-target blends, so the gauge stays readable through the demo's helicopter
	 *  and snare cameras. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge")
	bool bFaceCamera = true;

	/** Per-frame length change (cm) below which the rope counts as not reeling. Filters solver-scale
	 *  noise; a real reel moves centimetres per frame. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Reel Gauge|Tuning", meta = (ClampMin = "0.0", Units = "cm"))
	float ActivityEpsilon = 0.01f;

	/** The current display alpha, 0 to 1 — what the fade latch decided this frame. For Blueprint
	 *  restyles that want their own show/hide effects on top. */
	UFUNCTION(BlueprintPure, Category = "Rope|Reel Gauge")
	float GetDisplayAlpha() const { return DisplayAlpha; }

private:
	/** The rope this gauge watches: the owner's wielder-held rope, else the first rope component on the
	 *  owner. Resolved every tick so a swapped pawn or rope is picked up without wiring. */
	URopeComponent* ResolveRope();

	/** Moves the preset subscription to the rope currently watched, unbinding the previous one. */
	void BindRope(URopeComponent* NewRope);

	/** Handler for the rope's OnPresetApplied signal. A preset rewrites the length and its domain in
	 *  one go — a settings change, not a reel — so the activity baseline reseeds and the widget's
	 *  displayed fill snaps to the new domain instead of the jump reading as reeling. */
	UFUNCTION()
	void HandlePresetApplied(const URopePreset* Preset);

	/** Turns the widget plane toward the local player's camera. */
	void FaceCamera();

	/** The wielder found on the owner, kept only to skip the component search in the normal case. */
	TWeakObjectPtr<URopeWielderComponent> CachedWielder;

	/** The rope whose OnPresetApplied is currently bound, kept to move the subscription when the
	 *  watched rope changes. */
	TWeakObjectPtr<URopeComponent> BoundRope;

	/** Set by the preset handler, consumed on the next tick: the widget snaps its displayed fill
	 *  rather than animating across the rewritten domain. */
	bool bPendingDisplaySnap = false;

	FRopeReelGaugeFadeState Fade;

	/** Last frame's rope length, the baseline of the activity observation, and whether it has been
	 *  seeded — the seeding frame is never counted as activity. */
	float PrevLength = 0.0f;
	bool bHasPrevLength = false;

	float DisplayAlpha = 0.0f;
};
