// Copyright Epic Games, Inc. All Rights Reserved.
//
// The 'stat DynamicRope' dashboard group, which gathers the per-frame simulation cost and load onto one
// screen, serving both performance tracking and a runtime state check. The per-component
// 'stat RopeFlight' and 'stat RopeWrapped' groups in RopeDebugDraw are left as they are; this group
// takes in the whole of URopeSimSubsystem::Tick.
//   - Cycle statistics, timed on the game thread, report each tick stage in milliseconds: the gather,
//     Prepare, Solve and Finalize.
//   - Counters report load: the number of active ropes, the solve path split, the total particle count,
//     the collider count, and the physics against logic distribution.
//
// Scope contract: this group is the game thread frame dashboard alone. GPU render thread timings, video
// memory and bandwidth belong to the separate 'stat DynamicRopeGPU' group, in the DynamicRopeShaders
// module. Putting them all in one group produced 53 rows and overflowed the stat HUD, which draws
// cycles, then memory, then counters, and does not scroll, so the counter section that mattered was cut
// off below the screen. Preserve that split when adding a statistic: game thread frame cost here, GPU
// and render thread there.
//
// The solve path split, where the active count equals the GPU-stepped plus the CPU-solved plus the rest,
// which did not solve:
//   - GPU Stepped is a rope dispatched as a GPU resident step this frame, whether a solve or a logic
//                 override.
//   - CPU Solved  is a rope that solved but not on the GPU, which is a genuine CPU fallback, caused by
//                 exceeding the maximum node count or by there being no renderable RHI, as under
//                 -nullrhi or on a server.
//   - The rest are idle: neither of the above, such as a frozen Contacting rope, one asleep in Free, or
//                 one with no logic override at all. It is the active count minus the other two, so it
//                 has no row of its own; the RopePerf debugger shows the per-rope breakdown.
// Sleeping is a state counter orthogonal to that split, counting ropes asleep because they came to rest
// in Free; they are usually part of the idle group.
// The world distance field dispatch count is how many were dispatched together with the field this
// frame, and it fluctuating between frames with the phase, sleep and collision gates is normal; it
// matches the engine's on-demand build signal. The setting itself defaults to enabled, so counting it
// would simply equal the active count and it is not a statistic; the RopePerf debugger shows the
// per-rope setting.
//
// Phases are summarized as two rows along the design's physics and logic boundary, where Free and Flight
// are the solver and the rest are logic. The full breakdown of all eight phases is already shown per
// rope by the RopePerf gameplay debugger category, so eight rows are not spent on the HUD.
//
// The statistic macros compile to nothing in builds with statistics disabled, such as shipping, so no
// separate guard is needed. The declarations live here and the definitions in RopeStats.cpp, which is
// what lets the scope and counter macros be used directly from other translation units such as the
// subsystem.

#pragma once

#include "CoreMinimal.h"
#include "Containers/ArrayView.h"
#include "Stats/Stats.h"
#include "UObject/ObjectPtr.h"

class URopeComponent;

// 'stat DynamicRope': the runtime simulation frame cost and load dashboard.
DECLARE_STATS_GROUP(TEXT("DynamicRope"), STATGROUP_DynamicRope, STATCAT_Advanced);

// Game thread timings for each stage of URopeSimSubsystem::Tick, as wall clock. On the GPU path the
// solve captures only the cost of enqueueing the dispatch, since the real GPU solve and detection happen
// on the render thread and are visible in Insights. On the CPU fallback path it is the real solve cost
// of the parallel loop.
DECLARE_CYCLE_STAT_EXTERN(TEXT("Tick (total)"), STAT_RopeSim_Tick, STATGROUP_DynamicRope, );
DECLARE_CYCLE_STAT_EXTERN(TEXT("Gather Colliders"), STAT_RopeSim_Gather, STATGROUP_DynamicRope, );
DECLARE_CYCLE_STAT_EXTERN(TEXT("Prepare"), STAT_RopeSim_Prepare, STATGROUP_DynamicRope, );
DECLARE_CYCLE_STAT_EXTERN(TEXT("Solve (enqueue/parallel)"), STAT_RopeSim_Solve, STATGROUP_DynamicRope, );
DECLARE_CYCLE_STAT_EXTERN(TEXT("Finalize"), STAT_RopeSim_Finalize, STATGROUP_DynamicRope, );

// Load: this frame's rope scale and solve path split. The active count minus the GPU-stepped and
// CPU-solved counts gives the ropes that did not solve.
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Active Ropes"), STAT_Rope_Active, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Total Particles"), STAT_Rope_TotalParticles, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Frame Colliders"), STAT_Rope_FrameColliders, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("GPU Stepped Ropes"), STAT_Rope_GpuStepped, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("CPU Solved Ropes (fallback)"), STAT_Rope_CpuSolved, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Sleeping Ropes"), STAT_Rope_Sleeping, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("World GDF Dispatches"), STAT_Rope_GdfDispatched, STATGROUP_DynamicRope, );

// Phase distribution, summarized along the physics and logic boundary alone, summing to the active
// count. It shows at a glance whether the load comes from the solver or from logic; the full breakdown
// of all eight phases is shown per rope by the RopePerf gameplay debugger.
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Phase: Physics (Free/Flight)"), STAT_Rope_PhasePhysics, STATGROUP_DynamicRope, );
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Phase: Logic (Contacting..Loaded)"), STAT_Rope_PhaseLogic, STATGROUP_DynamicRope, );

namespace RopeStats
{
	// Frame scalars, which only the subsystem knows because the frame state is private and the GPU path
	// branches there, are passed to the helper. Every other per-rope counter, covering the phase, GPU
	// steps, CPU solves, sleep and particles, is aggregated by RecordFrameStats through the public getters.
	struct FRopeFrameCounters
	{
		int32 NumGdfDispatched = 0;  // GPU steps dispatched together with the distance field this frame, matching the engine's on-demand build signal.
		int32 FrameColliders = 0;    // The sum of every rope's frame colliders, aggregated by the subsystem because the frame state is private.
	};

	/** Updates the frame load and phase counters of 'stat DynamicRope', only while the group is
	 *  collecting; otherwise the walk is skipped. Stage timings are recorded directly by the scope macros
	 *  in each stage. */
	void RecordFrameStats(TConstArrayView<TObjectPtr<URopeComponent>> Ropes, const FRopeFrameCounters& Frame);
}
