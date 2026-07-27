// Copyright Epic Games, Inc. All Rights Reserved.
//
// The declaration of the 'stat DynamicRopeGPU' group, owned by the DynamicRopeShaders module. The GPU render-thread
// timing, VRAM and transfer bandwidth stats are collected here. It is a separate group from the runtime game-thread
// frame dashboard, 'stat DynamicRope', declared in the runtime module's RopeStats.h.
// They were originally one group, which grew past what a single stat HUD screen can show: the HUD draws cycles, then
// memory, then counters, with no scrolling, so the counter section at the bottom was cut off, and the default
// stats.MaxPerGroup limit of 25 was close as well. Game-thread frame costs therefore belong to that group and
// GPU and render-thread costs to this one, and new stats must respect that boundary. To see both, enable both groups
// ('stat DynamicRope' and 'stat DynamicRopeGPU').
//
// Note that every 'GPU *' cycle stat in this group is render-thread CPU time. The time the GPU actually spent running
// the kernels belongs to the engine's GPU group and is seen in 'stat gpu' as DynamicRope Solve, Detect and Tube,
// declared through DECLARE_GPU_STAT_NAMED in each .cpp.
//
// The group struct declared by DECLARE_STATS_GROUP must be defined exactly once per translation unit. A unity build
// merges several .cpp files into one translation unit, so declaring it inline in each .cpp would redefine the struct,
// which is why it lives in a header behind an include guard and is included by every shaders .cpp that declares a
// stat in this group, namely RopeGPUSolver.cpp and RopeTubeBuilder.cpp. The individual stats declared through
// DECLARE_*_STAT are static and therefore local to a translation unit, so they stay in each .cpp.

#pragma once

#include "Stats/Stats.h"

DECLARE_STATS_GROUP(TEXT("DynamicRopeGPU"), STATGROUP_DynamicRopeGPU, STATCAT_Advanced);
