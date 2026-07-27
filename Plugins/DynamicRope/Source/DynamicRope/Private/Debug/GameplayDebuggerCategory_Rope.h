// Copyright Epic Games, Inc. All Rights Reserved.
//
// The single entry point for rope debugging. Enabling the gameplay debugger in game and toggling the
// Rope category displays the URopeComponents of the debugged actor. Registering that actor with
// URopeDebugSubsystem every frame makes the sim tick capture only those ropes and leave a snapshot,
// which is read back here and drawn through AddShape and AddTextLine.
// The sub-views, covering nodes, flight, wrap, colliders and aim, and the detail level are toggled with
// the category's input binding keys.
// The aim view alone is drawn live from the FRopeAimHudSample the actor's URopeWielderComponent caches
// each tick, rather than from the rope: aiming belongs to the wielder and cannot be carried on a rope
// snapshot.
// The whole file is compiled out in builds without the gameplay debugger, such as shipping.
//
// Supported scope: standalone and local play. Many of the 3D overlays, namely the colliders, the aim,
// the wrap axis, the node proximity and the pull leg, need to draw in the foreground and therefore call
// DrawDebug* directly rather than AddShape, because AddShape hardcodes its depth priority to world and
// something like an anchor inside a character mesh would be buried.
// The cost is that those shapes are not replicated to a remote client: data collection runs on the
// authority and DrawDebug* draws only into that world. Text lines, shapes and the replicated point data
// below do reach the client.
// This is accepted technical debt within the current supported scope rather than a claim that the
// debugger is network-agnostic: the plugin's simulation is local-only today so nothing is lost, but
// supporting multiplayer would also require serializing the endpoints and drawing them during the
// client's render step.
// The point display already works that way: it is replicated as data and drawn in DrawData().

#pragma once

#include "CoreMinimal.h"

#if WITH_GAMEPLAY_DEBUGGER

#include "GameplayDebuggerCategory.h"
// ERopeDebugCapture, the scope bits that carry the enabled views to the capture side.
#include "Debug/RopeDebugSnapshot.h"

class APlayerController;
class AActor;
class URopeComponent;
class URopeWielderComponent;
struct FRopeDebugSnapshot;

class FGameplayDebuggerCategory_Rope : public FGameplayDebuggerCategory
{
public:
	FGameplayDebuggerCategory_Rope();

	virtual void CollectData(APlayerController* OwnerPC, AActor* DebugActor) override;
	virtual void DrawData(APlayerController* OwnerPC, FGameplayDebuggerCanvasContext& CanvasContext) override;

	static TSharedRef<FGameplayDebuggerCategory> MakeInstance();

private:
	// Replicated data for point display alone. AddShape's point is avoided for performance: the engine's
	// point shape is not a real point but a sphere drawn as sixteen wire segments, so one point becomes
	// 512 batched lines. A single rope with the default node count exceeds ten thousand lines per frame,
	// and the latch highlight and the number of ropes multiply that further.
	// Instead only the position, colour and size are replicated and DrawData() draws them with
	// DrawDebugPoint, where a point costs no lines at all.
	// Because the drawing happens locally, on the viewing client, this path keeps its replication, unlike
	// calling DrawDebug* directly.
	struct FRepData
	{
		struct FPoint
		{
			FVector Location = FVector::ZeroVector;
			FColor  Color = FColor::White;
			// The size in screen pixels, not a world radius, which is what DrawDebugPoint expects.
			float   Size = 6.0f;
		};

		TArray<FPoint> Points;

		void Serialize(FArchive& Ar);
	};

	/** One point from this collection frame. The size is in screen pixels rather than a world radius. */
	void AddPoint(const FVector& Location, float PixelSize, const FColor& Color);

	FRepData DataPack;

	// The sub-view toggle bits, switched with the input binding keys. They are classified by what the
	// diagnosis is about: node state is Nodes, the wrap path and result are Wrap, and collision shapes are
	// Colliders. The summary, meaning the two header lines, has no bit and is always shown.
	enum class EView : uint8
	{
		// Node points, the latch highlight, and the per-node proximity re-query drawn as normal arrows.
		Nodes     = 1 << 0,
		// Flight candidates, sweeps and the whip guide.
		Flight    = 1 << 1,
		// The wrapping path axis plus the wrapped result, covering latches, tension and pull.
		Wrap      = 1 << 2,
		// Collider shapes only.
		Colliders = 1 << 3,
		Aim       = 1 << 4,
		// Adds the detailed numbers of whichever views are enabled, for tuning the algorithms. Without it
		// only the summary an integrating user needs is shown.
		Advanced  = 1 << 5,
	};

	// The defaults are minimal so the screen is not covered the first time an integrating user enables it.
	// Detail is a key away when it is wanted.
	static constexpr uint8 DefaultViewMask = static_cast<uint8>(EView::Aim);

	bool HasView(EView Flag) const { return (ViewMask & static_cast<uint8>(Flag)) != 0; }

	/** Converts the enabled views into a capture scope. The bits are mapped explicitly rather than passed
	 *  through, because the two enumerations serve different purposes, display toggles and collection
	 *  scope, and relying on them happening to share a layout would break silently when only one changes. */
	ERopeDebugCapture BuildCaptureMask() const;

	// The key handlers, which toggle their view while the category is active.
	void OnToggleNodes();
	void OnToggleFlight();
	void OnToggleWrap();
	void OnToggleColliders();
	void OnToggleAim();
	void OnToggleAdvanced();

	// Draws one rope. One screen uses a single point in time: the header, covering the phase, node count,
	// wrapped bone and solve path, along with the centreline and the diagnostic overlays, all come from the
	// same snapshot. Leaving the header live would draw the same node at two different moments and be
	// misread as simulation jitter or an unstable latch. The snapshot's age is reported in the header.
	// Only on the first capture frame, where there is no snapshot, is the header taken live and labelled
	// as such, with the overlays omitted.
	// The flight overlay is drawn from the last held flight snapshot, within a half-second window, whenever
	// the live rope is not in Flight, which keeps it visible for a moment after the capture decision. The
	// held age in seconds is what the held label reports.
	void DrawRope(const URopeComponent& Rope, const FRopeDebugSnapshot* Snap,
		const FRopeDebugSnapshot* HeldFlight, float HeldFlightAgeSeconds);

	// Draws the aim ray. It performs no query and only reads the sample the wielder already swept and
	// cached this tick, using green for wrappable, red for hit but not wrappable, and cyan for a miss.
	// Outside an aim ray mode the sample is empty and it reports no target.
	void DrawAim(const URopeWielderComponent& Wielder);

	uint8 ViewMask = DefaultViewMask;
};

#endif // WITH_GAMEPLAY_DEBUGGER
