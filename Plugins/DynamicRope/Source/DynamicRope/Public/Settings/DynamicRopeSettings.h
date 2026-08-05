// Copyright 2026 TeamKeno. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Core/RopeConfigTypes.h"
#include "Engine/DeveloperSettings.h"
class URopePullGaugeWidget;

#include "DynamicRopeSettings.generated.h"

class ARopeController;
class URopeAimWidget;
class URopePreset;

/**
 * Project-wide settings for the Dynamic Rope plugin.
 * Editable under Project Settings > Plugins > Dynamic Rope and saved to DefaultGame.ini.
 */
UCLASS(config = Game, defaultconfig, meta = (DisplayName = "Dynamic Rope"))
class DYNAMICROPE_API UDynamicRopeSettings : public UDeveloperSettings
{
	GENERATED_BODY()

public:
	UDynamicRopeSettings();

	/** Convenience accessor for the active settings object, which is the class default object. */
	static const UDynamicRopeSettings* Get();

	// Every global setting must have a consumer wired up when it is added, reached through Get().
	// A config field nothing reads is dead weight that looks configurable and changes nothing.

	/**
	 * The rope manager actor class URopeSimSubsystem spawns automatically when a game or PIE world
	 * starts. That actor hosts the URopeStaticBodyProvider used for static world collision, which
	 * guarantees exactly one per world without placing a provider by hand in every level. Subclass
	 * ARopeController to adjust MaxColliders and similar settings.
	 * Clear it to None to disable the automatic spawn, for projects that place the provider
	 * themselves.
	 */
	UPROPERTY(config, EditAnywhere, Category = "Collision", meta = (ToolTip = "Rope manager actor class spawned automatically when the game starts; it hosts the static world collision provider. Clear it to disable the automatic spawn."))
	TSoftClassPtr<ARopeController> StaticBodyControllerClass;

	/**
	 * The global per-frame limit on how many colliders the static body provider extracts. It is a
	 * safety valve against extraction and culling costs exploding in dense collision areas, and a
	 * normal scene should never reach it. The per-rope solve budget is StaticBodyMaxCollidersPerRope
	 * below, so this should be set comfortably above the expected maximum rope count multiplied by
	 * that per-rope budget, which keeps a rope from being starved during global extraction. The
	 * provider reads it directly every frame as the single source.
	 */
	UPROPERTY(config, EditAnywhere, Category = "Collision", meta = (ClampMin = "1", ToolTip = "Global per-frame extraction limit for the static body provider, a safety valve against dense scenes. The per-rope budget is StaticBodyMaxCollidersPerRope."))
	int32 StaticBodyMaxColliders = 256;

	/**
	 * The maximum number of static world colliders one rope can carry into its solve. After per-rope
	 * culling, anything above this count is dropped starting with the colliders furthest from that
	 * rope. The GPU kernel loops over colliders for every node and substep, so this bounds the solve
	 * cost per rope directly. Unlike the global limit it is independent per rope, so a distant rope's
	 * colliders cannot consume a nearby rope's budget, which makes it order independent. Skeleton
	 * colliders are always included and ignore this budget.
	 */
	UPROPERTY(config, EditAnywhere, Category = "Collision", meta = (ClampMin = "1", ToolTip = "Maximum static world colliders one rope carries into its solve. Anything above the limit is dropped furthest first. Skeleton colliders are always included."))
	int32 StaticBodyMaxCollidersPerRope = 32;

	/**
	 * The maximum number of planes per convex in the static body provider. A convex more complex than
	 * this falls back to the element box OBB. Like StaticBodyMaxColliders, the provider reads it
	 * directly as the single source; the total collider count is limited separately above.
	 */
	UPROPERTY(config, EditAnywhere, Category = "Collision", meta = (ClampMin = "4", ToolTip = "Maximum planes per convex; anything above it falls back to an OBB. Read directly by the provider as the single source."))
	int32 StaticBodyMaxConvexPlanes = 32;

	/**
	 * Whether the static body provider also collects WorldDynamic objects alongside WorldStatic ones.
	 * With this on, moving physical and kinematic bodies such as elevators, doors and platforms take
	 * part in rope collision. The provider tracks the previous frame's transform to derive a surface
	 * velocity, so a moving surface drags the rope and substep continuous collision prevents
	 * tunnelling.
	 */
	UPROPERTY(config, EditAnywhere, Category = "Collision", meta = (ToolTip = "Also collect WorldDynamic bodies alongside WorldStatic ones, such as moving platforms and doors. Surface velocity and continuous collision come from the previous frame's transform."))
	bool bIncludeWorldDynamic = true;

	/**
	 * The widget class for the aim ray demo HUD, a crosshair plus a highlight ring on the wrappable
	 * bone, used by the AssistedJudged and GuaranteedWrap modes.
	 * URopeWielderComponent creates it and adds it to the local player viewport while
	 * bShowAimHudWidget is on. It defaults to the C++ URopeAimWidget, which works without any assets,
	 * and can be restyled by pointing it at a Blueprint widget subclassing URopeAimWidget, optionally
	 * disabling the built-in painting and drawing its own visuals; see RopeAimWidget.h. Clear it to
	 * show no HUD.
	 */
	UPROPERTY(config, EditAnywhere, Category = "Demo", meta = (ToolTip = "Aim-ray demo HUD widget class (crosshair + wrappable-bone highlight ring). Spawned by URopeWielderComponent for the local player when bShowAimHudWidget is on. Defaults to the C++ URopeAimWidget (works with no assets); point it at a WBP subclass to restyle. Clear it to disable the HUD."))
	TSoftClassPtr<URopeAimWidget> AimHudWidgetClass;

	/**
	 * The widget class for the pull arming and engagement gauge, a ring showing progress towards the
	 * engage threshold.
	 * URopeWielderComponent creates it and adds it to the local player viewport while
	 * bShowPullGaugeWidget is on. It defaults to the C++ URopePullGaugeWidget, which works without any
	 * assets, and can be restyled with a Blueprint widget subclass. Clear it to show no gauge; the
	 * getters and events remain usable either way.
	 */
	UPROPERTY(config, EditAnywhere, Category = "Demo", meta = (ToolTip = "Pull gauge widget class (ring showing progress toward the pull engage tension). Spawned by URopeWielderComponent for the local player when bShowPullGaugeWidget is on. Defaults to the C++ URopePullGaugeWidget (works with no assets); point it at a WBP subclass to restyle. Clear it to disable the gauge."))
	TSoftClassPtr<URopePullGaugeWidget> PullGaugeWidgetClass;

	/**
	 * The demo preset cycle list, consumed only by the console commands Rope.Preset.Cycle,
	 * Rope.Preset.Apply and Rope.Preset.List in RopePresetDemoCommands.cpp, which exist in
	 * non-shipping builds only. It registers the URopePreset assets to apply in order to the ropes in
	 * the world during demos and feature testing, held as soft references so they load only when a
	 * command runs. In a shipping build the commands are compiled out and this setting has no
	 * consumer, which is the one intended exception to the rule that every setting must be read. Game
	 * code should call URopeComponent::ApplyPreset directly instead.
	 */
	UPROPERTY(config, EditAnywhere, Category = "Demo", meta = (ToolTip = "Demo preset cycle list consumed only by the non-shipping console commands Rope.Preset.Cycle / .Apply / .List. Soft references - loaded when a command runs. Game code should call URopeComponent::ApplyPreset directly."))
	TArray<TSoftObjectPtr<URopePreset>> DemoPresets;

	/**
	 * Whether the rope tube outputs per-vertex velocity. The tube keeps last frame's vertex
	 * positions alongside the current ones, so its motion vectors describe the actual deformation:
	 * temporal upscalers (TSR/TAA) reproject the rope correctly instead of ghosting it as static
	 * geometry, and motion blur blurs along the real motion instead of smearing. Requires a platform
	 * with GPU-skin passthrough shader support; elsewhere, and when disabled, the rope writes no
	 * velocity and is excluded from motion blur - a transform-only velocity would be wrong for a
	 * deforming mesh, so there is no in-between mode. It is read once when the scene proxy is
	 * created, so a change takes effect after restarting PIE or recreating render state.
	 */
	UPROPERTY(config, EditAnywhere, Category = "Rendering", meta = (ToolTip = "Whether the rope tube outputs per-vertex velocity (default on). Gives correct motion vectors for the deforming tube: no TSR/TAA ghosting and accurate motion blur. Off, or on platforms without GPU-skin passthrough support, the rope writes no velocity and is excluded from motion blur."))
	bool bWriteVelocity = true;
};
