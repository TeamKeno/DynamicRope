// Copyright Epic Games, Inc. All Rights Reserved.
//
// The two pieces of infrastructure supporting global distance field world collision, kept together:
//  1) The scene-to-solver registry. The GPU solver is per world, as a member of URopeSimSubsystem,
//     while the view extension is global, so the render thread has to find the solver belonging to
//     the scene being rendered before it can dispatch. That is what keeps the global distance field
//     consistent across multiple PIE worlds.
//  2) The consumer gate. A flag saying whether this scene has any rope using the global distance
//     field, read by the custom FFXSystem, FRopeGDFFXSystem, in UsesGlobalDistanceField(). It is the
//     signal that makes the engine build the field on demand.
// Everything is keyed by FSceneInterface*, which is the same type as the world's scene,
// View.Family->Scene and FFXSystem->GetSceneInterface().

#pragma once

#include "CoreMinimal.h"

class FRopeGPUSolver;
class FSceneInterface;
class FRDGBuilder;

namespace RopeGDF
{
	//~ The scene-to-solver registry.
	/** Registers a solver against a scene, on the game thread from the subsystem's Initialize. */
	DYNAMICROPESHADERS_API void RegisterSolver(FSceneInterface* Scene, FRopeGPUSolver* Solver);
	/** Unregisters a scene's solver, on the game thread from the subsystem's Deinitialize. */
	DYNAMICROPESHADERS_API void UnregisterSolver(FSceneInterface* Scene);
	/** Finds a solver by scene, on the render thread from the view extension. Null when there is
	 *  none. */
	FRopeGPUSolver* FindSolver(FSceneInterface* Scene);

	//~ The consumer gate.
	/** Sets how many ropes in this scene are using the global distance field, on the game thread from
	 *  the subsystem's Tick, which is authoritative every frame. */
	DYNAMICROPESHADERS_API void SetGDFActiveCount(FSceneInterface* Scene, int32 Count);
	/** Whether this scene needs the global distance field, meaning the active count is above zero. Read
	 *  by FFXSystem::UsesGlobalDistanceField(). */
	bool IsGDFActive(FSceneInterface* Scene);
}
