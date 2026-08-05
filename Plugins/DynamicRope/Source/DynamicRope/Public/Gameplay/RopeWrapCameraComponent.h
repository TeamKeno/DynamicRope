// Copyright 2026 TeamKeno. All Rights Reserved.
//
// Opt-in camera framing marker for a rope wrap target. Add it to anything wrappable, place it in the
// Blueprint viewport until the frustum preview shows the shot you want, and URopeWrapCameraDirectorComponent
// on the wielder cuts to it for a beat when a rope wraps this actor. A target without one of these is never
// affected, so the whole feature is opt-in.
//
// Attach it to a bone socket rather than the actor root when the shot should follow the target's animation
// or ragdoll; the camera then rides the body instead of watching an empty spot once the target is dragged
// away.
//
// It is a UCameraComponent purely for the authoring surface, that is the viewport preview and the FOV and
// post process settings, and is deliberately kept deactivated for its whole lifetime. AActor::CalcCamera
// picks the first *active* camera component on a view target, so an active marker would fight the target's
// own gameplay camera when the target is a player pawn. The director never makes this actor the view target
// either: it spawns an ARopeWrapCameraRig that delegates its CalcCamera to this component, which reproduces
// everything authored here without touching the target's camera state. Do not activate it.

#pragma once

#include "CoreMinimal.h"
#include "Camera/CameraComponent.h"
// FViewTargetTransitionParams, the engine's blend descriptor, used as-is for the two blend properties.
#include "Camera/PlayerCameraManager.h"
#include "RopeWrapCameraComponent.generated.h"

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent), HideCategories = (Activation))
class DYNAMICROPE_API URopeWrapCameraComponent : public UCameraComponent
{
	GENERATED_BODY()

public:
	URopeWrapCameraComponent();

	/**
	 * The bones this marker frames. Leave it empty for a catch-all marker that accepts any wrap on this
	 * actor. Filling it lets one target carry several shots, for example a dragon framing a wrapped wing
	 * differently from a wrapped tail; the marker is only considered when the wrap touches one of these
	 * bones.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Camera")
	TArray<FName> BoneFilter;

	/** Tie-break between markers that are equally specific about the wrapped bone; the highest wins, and a
	 *  remaining tie goes to the marker nearest the wrap. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Camera")
	int32 Priority = 0;

	/** How the wielder's camera blends into this shot. BlendTime 0 cuts instantly. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Camera")
	FViewTargetTransitionParams BlendIn;

	/** How long the shot is held after the blend in finishes (s), before blending back to the wielder. The
	 *  cut ends early regardless if the rope releases or is cut first. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Camera", meta = (ClampMin = "0.0", Units = "s"))
	float HoldTime = 1.0f;

	/** How the shot blends back to the wielder's camera. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Camera")
	FViewTargetTransitionParams BlendOut;

	/** Whether this marker accepts a wrap spanning the given bones: true for a catch-all marker, and for a
	 *  filtered one only when the filter and the wrapped bones intersect. */
	UFUNCTION(BlueprintPure, Category = "Rope|Wrap Camera")
	bool MatchesBones(const TArray<FName>& WrappedBones) const;

	/**
	 * The marker on TargetActor that should frame a wrap spanning WrappedBones near WrapLocation, or null
	 * when the actor carries none that accepts it.
	 *
	 * A marker naming one of the wrapped bones always beats a catch-all marker, whatever their priorities,
	 * since naming the bone is the more specific statement of intent and a catch-all is meant as the
	 * fallback. Within one specificity level the highest Priority wins, and a tie goes to the marker
	 * closest to the wrap, which is the useful answer for a symmetric skeleton carrying a marker per side.
	 */
	static URopeWrapCameraComponent* SelectForWrap(const AActor* TargetActor, const TArray<FName>& WrappedBones,
		const FVector& WrapLocation);
};
