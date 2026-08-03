// Copyright Epic Games, Inc. All Rights Reserved.
//
// A rope preset data asset: a bundle of values making up a URopeComponent's behavioural identity, covering its mode,
// material properties, detection, hold, tip and rendering.
// It is a stamp, meaning copy-on-apply: URopeComponent::ApplyPreset() copies the values into the component and is
// done. Tuning the component individually afterwards is free, and there is no live link to the preset asset. This
// deliberately avoids the explosion of per-field override surface that a reference-style preset would bring.
//
// What it does not carry is the instance plumbing, meaning values bound to a particular actor or skeleton, which a
// preset would break by overwriting: TipMeshComponentTag, being a component tag on the owning actor, and everything
// on the wielder, being the attach mesh, the input and the movement, which are outside the scope of this version.
// LoadedHandSocket, being a socket on the owning skeleton, is the one exception, and is overwritten only by a preset
// that has bOverrideLoadedHandSocket enabled; it is off by default, which preserves the plumbing.
//
// The fields are a one-to-one mirror of URopeComponent's identically named properties. Keep the defaults, the clamp
// metadata and the tooltips the same as the component, and the details panel surface too: the same category names
// (Rope|…), the same display names, the same edit conditions, ShowOnlyInnerProperties on the configuration structs,
// and the same declaration order. That is what makes a preset asset read with the same shape and order as the
// component's Rope section. The exposure and display names of the fields inside the structs follow automatically,
// since the definitions in RopeConfigTypes.h and RopeThrowTypes.h are shared, and the single gate that crosses a
// struct boundary, being the three automatic release fields in HoldConfig, is handled by the editor customization in
// DynamicRopeEditor's RopeResolveModeDetails.h, which shares the same class with the component.
// (The contract is that applying a default preset gives the default rope, with no regression.)

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "Core/RopeTypes.h"
#include "RopePreset.generated.h"

class UMaterialInterface;
class UStaticMesh;

/**
 * A rope preset. It holds one verified rope configuration, such as a grappling hook, a capture rope or a free
 * simulation, as an asset, applied wholesale through URopeComponent::ApplyPreset() in the Free and Loaded phases alone.
 */
UCLASS(BlueprintType, meta = (ToolTip = "A complete rope configuration (mode, physics, detection, hold, tip, render) applied to a URopeComponent as one stamp via ApplyPreset(). Values are copied — no live link. Only applies while the rope is in Free or Loaded phase."))
class DYNAMICROPE_API URopePreset : public UDataAsset
{
	GENERATED_BODY()

public:
	// The defaults are kept synchronized with the component's constructor: the rope material is the plugin's default
	// hemp rope, found through the same FObjectFinder as URopeComponent's constructor. Leaving it as none would break
	// the "an empty preset gives the default rope" contract for the material, since the stamp would strip the default
	// material and leave the grey fallback.
	URopePreset();

	//~ Setup ---------------------------------------------------------------

	/** The wrap resolve mode, meaning what is guaranteed from the throw through to the binding. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (DisplayName = "Wrap Mode"))
	ERopeWrapResolveMode ResolveMode = ERopeWrapResolveMode::AssistedJudged;

	//~ Tip attachment -------------------------------------------------------
	// TipMeshComponentTag is instance plumbing and is absent from the preset, and LoadedHandSocket is an opt-in stamp;
	// see the comment at the top of this file. On application the existing tip, if we spawned it, is destroyed and reacquired under the new settings.

	/** Use a tip attachment. Turning it off ignores every tip setting below and gives an ordinary rope with no tip. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip")
	bool bUseTipMesh = false;

	/** The static mesh spawned as the tip. Empty means no tip, unless a tagged component on the target is reused. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh", DisplayName = "Mesh"))
	TObjectPtr<UStaticMesh> TipMesh = nullptr;

	/** The tip's placement offset, in the tip node's frame. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh", DisplayName = "Relative Transform"))
	FTransform TipMeshRelativeTransform = FTransform::Identity;

	/** Enables collision on the tip mesh. Off by default, which stops a display-only tip interfering with the rope and the character. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh", DisplayName = "Enable Collision"))
	bool bTipMeshCollision = false;

	/** Keeps the tip aligned to the rope's end every frame while Free. Turning it off leaves the tip alone while Free. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh", DisplayName = "Sync While Free"))
	bool bSyncTipMeshOnFree = true;

	// Loaded (hand) placement. The socket name is wiring bound to the owning skeleton, so it is stamped
	// only when opted in; the offset defaults to Identity - which is exactly the current behaviour - and
	// is stamped unconditionally like the rest of the Tip fields. The opt-in toggle has no component
	// counterpart (it governs the stamp, not the rope), so it renders under its own name.

	/** Stamp Loaded Hand Socket onto the component. Leave off to keep the instance's own socket wiring. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip", meta = (EditCondition = "bUseTipMesh"))
	bool bOverrideLoadedHandSocket = false;

	/** Socket on the owner's skeletal mesh the tip is held at while Loaded. Empty falls back to the component transform. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh && bOverrideLoadedHandSocket"))
	FName LoadedHandSocket = NAME_None;

	/** Offset applied to the tip while Loaded, expressed in the hand socket's frame. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh", DisplayName = "Loaded Relative Transform"))
	FTransform LoadedTipRelativeTransform = FTransform::Identity;

	/** For GuaranteedWrap alone: places the tip precisely using the head and tail sockets. Turning it off puts the mesh origin at the rope's end. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh && ResolveMode == ERopeWrapResolveMode::GuaranteedWrap", DisplayName = "Use Sockets"))
	bool bUseTipMeshSockets = false;

	/** The head socket, being the tip's point, which is what embeds at the aim hit. With none, the socket correction is disabled. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh && bUseTipMeshSockets && ResolveMode == ERopeWrapResolveMode::GuaranteedWrap", DisplayName = "Tip Socket"))
	FName TipSocketName = NAME_None;

	/** The tail socket, where the rope's free end attaches. With none, it attaches at the mesh origin. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Tip",
		meta = (EditCondition = "bUseTipMesh && bUseTipMeshSockets && ResolveMode == ERopeWrapResolveMode::GuaranteedWrap", DisplayName = "Rope Socket"))
	FName TipRopeSocketName = NAME_None;

	//~ Rope, being the basic material properties ----------------------------

	/** The number of nodes, meaning particles. Applying it reinitializes the rope. The limit of 512 is the GPU solver's thread group limit. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "2", ClampMax = "512", DisplayName = "Node Count"))
	int32 NumParticles = 64;

	/** The initial, and maximum, rope length in centimetres. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "1.0", Units = "cm"))
	float RopeLength = 600.0f;

	/** The minimum length, in centimetres, that reeling in can reach. The initial rope length is the upper bound. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "10.0", Units = "cm"))
	float MinRopeLength = 100.0f;

	/** The default reel speed, in centimetres per second, used by the reel-in and pay-out inputs. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "0.0", Units = "cm/s"))
	float ReelSpeed = 300.0f;

	/** Shows the rope tube while Loaded. A presentation switch for GuaranteedWrap alone. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope",
		meta = (EditCondition = "ResolveMode == ERopeWrapResolveMode::GuaranteedWrap"))
	bool bShowRopeWhenLoaded = false;

	//~ The per-phase configuration structs, split by the same domains as the component ---
	// Exposed with ShowOnlyInnerProperties as on the component, so the fields unfold directly beneath the category
	// header (Rope|Solver, Rope|Throw and so on) with no second expansion from a category to a struct name to a field,
	// while each field's sub-category, such as Rope|Solver|Tuning, is preserved by the definition inside the struct.

	/** The XPBD solver tuning: substeps, iterations, compliance, friction, gravity, sleeping and LOD. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Solver", meta = (ShowOnlyInnerProperties))
	FRopeSolverConfig SolverConfig;

	/** The throw parameters. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Throw", meta = (ShowOnlyInnerProperties))
	FRopeThrowParams ThrowParams;

	// GuaranteedWrap is preview-based and therefore has no wrap decision or path build, and takes the guided throw arc
	// rather than the whip flight, so WrapConfig and WhipConfig are meaningless in a GuaranteedWrap preset and are
	// greyed out by the same struct-member EditCondition as on the component, which can see the resolve mode as a
	// sibling member of the same class. The edit-const propagates to the inline children promoted by
	// ShowOnlyInnerProperties, and the values are preserved while editing alone is blocked.

	/** The physics-to-logic wrap handoff: the capture decision thresholds and the tuning of establishing it, meaning the path build, the decision and the commit. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Wrap",
		meta = (ShowOnlyInnerProperties, EditCondition = "ResolveMode != ERopeWrapResolveMode::GuaranteedWrap"))
	FRopeWrapConfig WrapConfig;

	/** Tuning applied after the wrap: holding, pulling and releasing. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Hold", meta = (ShowOnlyInnerProperties))
	FRopeHoldConfig HoldConfig;

	//~ Collision ------------------------------------------------------------

	/** Whether the owner's own collider provider takes part in collision. Excluded by default, which stops a thrown rope tangling on the thrower. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Collision",
		meta = (ToolTip = "When off, the default, the owner's own colliders are excluded, and shapes owned by the owner that the static world provider picked up (the tether proxy, the tip, a weapon) drop out per body as well. Turn it on when the rope is attached to a prop actor such as a pillar; otherwise it passes through its own base.", DisplayName = "Collide With Owner"))
	bool bIncludeOwnerColliders = false;

	/** Whether the engine's global distance field pushes the rope out of static world geometry such as walls and floors. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Collision", meta = (DisplayName = "Use World Distance Field"))
	bool bUseWorldGDF = true;

	//~ Whip, being the throw's swing ----------------------------------------

	/** Tuning of the whip swing at the start of a throw. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Whip",
		meta = (ShowOnlyInnerProperties, EditCondition = "ResolveMode != ERopeWrapResolveMode::GuaranteedWrap"))
	FRopeWhipConfig WhipConfig;

	//~ Render ---------------------------------------------------------------
	// On the component these are consumed once when the proxy is created, so ApplyPreset is responsible for recreating the render state too.

	/** The visual tube radius, in centimetres. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render", meta = (ClampMin = "0.1", Units = "cm", DisplayName = "Rope Radius"))
	float Radius = 2.0f;

	/** The material applied to the rope tube. Leaving it empty uses the engine's default material. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render", meta = (DisplayName = "Material"))
	TObjectPtr<UMaterialInterface> RopeMaterial = nullptr;

	/** The number of sides on the tube's cross-section. Higher is rounder. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render|Tuning", meta = (ClampMin = "3", ClampMax = "32", DisplayName = "Sides"))
	int32 NumSides = 8;

	/** Render tube smoothing: the number of Catmull-Rom subdivisions per segment, where one disables it. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render|Tuning", meta = (ClampMin = "1", ClampMax = "8", DisplayName = "Smoothing Subdivisions"))
	int32 TubeSmoothingSubdiv = 1;

	/** The Catmull-Rom knot alpha for the render tube smoothing: zero is uniform, 0.5 centripetal and one chordal. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Render|Tuning", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Smoothing Strength"))
	float TubeSmoothingAlpha = 0.5f;

#if WITH_EDITOR
	//~ Editor validation, which reports authoring mistakes such as an inverted length range when the asset is saved.
	virtual EDataValidationResult IsDataValid(FDataValidationContext& Context) const override;
#endif
};
