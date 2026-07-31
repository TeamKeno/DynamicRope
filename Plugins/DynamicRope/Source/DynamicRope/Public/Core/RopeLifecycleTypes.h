// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "RopeLifecycleTypes.generated.h"

class USceneComponent;
class URopeComponent;

/**
 * Lifecycle phases. Physics, that is the solver, runs the whole rope in Free and Flight, and only the
 * unmasked free stretches in Wrapping and Wrapped. The decisions and driving for Contacting,
 * Wrapping, Wrapped and Releasing belong to the logic classes under Logic/.
 */
UENUM(BlueprintType)
enum class ERopePhase : uint8
{
	Free = 0,
	Flight = 1,

	/** Re-collecting contact candidates every frame and deciding, from the tracker's dwell time,
	 *  whether to begin wrapping. */
	Contacting = 2,

	/** Wrapping: progressively building the surface path, moving the front along it, and masking
	 *  mass. */
	Wrapping = 3,

	Wrapped = 4,

	/** GuaranteedWrap only. It does not go through the physical Flight phase: an aimed throw follows
	 *  the committed preview path, and a throw into open space follows an arc towards the far end of
	 *  the ray (bFreeThrow). The former ends in Wrapped and the latter in Free. */
	GuidedThrow = 5,

	Releasing = 6,

	/** GuaranteedWrap only: ready to throw with the tip held in the hand. The rope is hidden, and a
	 *  throw is only valid from this phase. It is entered through EnterLoaded() from Free, including
	 *  after a release from an embedded state. */
	Loaded = 7
};

/** Why this rope's engagement ended, covering contact, an established wrap, and a GuaranteedWrap
 *  aimed throw. It includes aborts from before a wrap was established: OnRopeReleased fires without
 *  a wrap too, in which case the bone can be None. Only the central OnAnyRopeReleased is restricted
 *  to committed wraps. */
UENUM(BlueprintType)
enum class ERopeReleaseReason : uint8
{
	/** Gameplay released it explicitly, through URopeComponent::ReleaseWrap. */
	Manual = 0,

	/** The hand-to-anchor distance exceeded the available rope length plus DistanceReleaseSlack,
	 *  automatically. */
	Distance = 1,

	/** The maximum tension stayed above TensionReleaseForce, automatically. */
	Tension = 2,

	/** An internal cause such as the target being lost or the wrap failing. Aborts before the wrap is
	 *  established, where the target is lost during contact, wrapping or a guaranteed throw, also land
	 *  here. */
	Broken = 3,

	/** External gameplay cut the rope, through URopeComponent::CutRope. */
	Cut = 4,

	/**
	 * A game rule broke the guarantee during a guaranteed throw, meaning the ShouldAbortGuaranteedThrow
	 * override returned true. Unlike an internal failure it is an intended gameplay outcome, such as
	 * the target dodging or teleporting away. Consumers need the two distinguished so they can react
	 * differently to an engine problem and to a designed evasion.
	 */
	ThrowAborted = 5
};

/**
 * The wrap resolve mode: the contract for what this rope guarantees between the throw and the bind.
 * It decides the standing of aiming and the preview, and whether the judgement gates are used, and
 * the wielder's aiming and throwing behaviour is derived from it.
 * Everything after a wrap is established, that is holding, pulling, tethering and releasing, is
 * common to all modes.
 */

UENUM(BlueprintType)
enum class ERopeWrapResolveMode : uint8
{
	// Everything from the throw to the bind is emergent. There is no aim assistance and no preview,
	// so missing, grazing and falling short of the judgement gates are all normal outcomes, matching
	// reality.

	/** Full simulation, guaranteeing nothing. Missing is a normal result, for sandboxes and
	 *  research. */
	FullSimulation = 0 UMETA(DisplayName = "Full Simulation"),

	// An aim ray locks the target, so the hit is guaranteed while whether a bind is established is
	// decided by judgement, through the wrapped angle and coverage gates. The preview is display only
	// and does not constrain the throw.

	/** Assisted and judged: the hit is guaranteed, the bind is judged. Failure, that is a release, is
	 *  a normal result, for combat and skills. */
	AssistedJudged = 1 UMETA(DisplayName = "Assisted (Judged)"),

	// The preview committed to at the moment of the throw is the execution path itself, so there is no
	// failure after the throw plays out. When aiming does not resolve, because there is no target or
	// it is out of range, the throw is not refused: the rope arcs towards the far end of the ray,
	// embeds in nothing and falls to Free. The guarantee applies to the target that was aimed at, so
	// that is a normal result too. Automatic releases from tension and distance do not apply; only an
	// explicit release does.

	/** Guaranteed: it binds the aimed target without fail. It can only be thrown from the Loaded
	 *  phase. For demos, scripted sequences and traversal. */
	GuaranteedWrap = 2 UMETA(DisplayName = "Guaranteed")
};

/** The single source for the constraints a resolve mode imposes, namely the phase gate
 *  CanThrowInPhase. Shared by the throw entry points, the aiming HUD and the tests. It has no
 *  UObject or world dependency, so it can be inlined in the header and unit tested. */
namespace RopeWrapModes
{
	/**
	 * Whether a throw is valid in this phase for this mode. GuaranteedWrap is restricted to the Loaded
	 * phase, while the other modes have no phase gate and are always true.
	 *
	 * Careful: this means "the throw gate does not block", not "the mode is GuaranteedWrap and the
	 * phase is Loaded". Replacing an expression of the form `X && Phase == Loaded` with this function
	 * alone would let the other two modes leak through as true; such places must keep the form
	 * `X && CanThrowInPhase(...)`.
	 */
	inline bool CanThrowInPhase(ERopeWrapResolveMode Mode, ERopePhase Phase)
	{
		return Mode != ERopeWrapResolveMode::GuaranteedWrap || Phase == ERopePhase::Loaded;
	}
}

/**
 * The payload of the wrap established event, delivered through OnRopeWrapped and NotifyWrapped. It
 * carries more than the dominant bone so that game rules such as pierce damage hooks and capture
 * strength have the information they need.
 */
USTRUCT(BlueprintType)
struct FRopeWrappedEventInfo
{
	GENERATED_BODY()

	/** The representative, that is dominant, bone. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope")
	FName Bone;

	/** Every bone the anchors span, with the dominant bone first and duplicates removed. This is the
	 *  full picture when a wrap establishes across several bones, as when catching both legs. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope")
	TArray<FName> Bones;

	/** The mesh that was wrapped, which can belong to another actor. Weak, since it can be destroyed
	 *  after the event. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope")
	TWeakObjectPtr<USceneComponent> Mesh;

	/** The rope's resolve mode when the wrap was established. A GuaranteedWrap is preview based, so
	 *  its judgement values are -1. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope")
	ERopeWrapResolveMode ResolveMode = ERopeWrapResolveMode::AssistedJudged;

	/** The accumulated wrapped angle at commit time (degrees), or -1 when it cannot be computed or the
	 *  wrap was preview based. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope")
	float AngleDeg = -1.0f;

	/** The angular coverage about the axis at commit time (degrees, 0 to 360), which answers whether a
	 *  gap remains to escape through. -1 when it cannot be computed or the wrap was preview based. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope")
	float CoverageDeg = -1.0f;

	/** The number of anchors, that is latched nodes, established. A supporting measure of capture
	 *  strength. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope")
	int32 AnchorCount = 0;

	/**
	 * The rope that established this wrap, which identifies it to subscribers of the central
	 * OnAnyRopeWrapped signal. Several ropes can wrap one target at the same time, as when binding both
	 * arms, so subscribers must maintain their set of active engagements keyed by this value.
	 * Reacting to the mesh alone would revert the reaction when one rope released even though others
	 * remain attached. The matching release signal, OnAnyRopeReleased, carries the same rope pointer.
	 * Weak, since it can be destroyed after the event.
	 */
	UPROPERTY(BlueprintReadOnly, Category = "Rope")
	TWeakObjectPtr<URopeComponent> Rope;
};
