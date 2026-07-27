// Copyright Epic Games, Inc. All Rights Reserved.
//
// Log categories for the DynamicRope runtime module, split along the boundaries of the architecture:
// the facade, the solver, the logic, and collision and rendering.
// Including this header alone is enough for any .cpp to use UE_LOG(LogRope..., ...); the definitions
// live in DynamicRope.cpp.
//
// Which to use:
//   LogDynamicRope     high-level flow, such as the lifecycles of components, subsystems and modules,
//                      and phase transitions.
//   LogRopeSolver      the XPBD solver, meaning physics. Hot-path diagnostics for substeps,
//                      constraints and convergence.
//   LogRopeWrap        the wrap controller, meaning logic: BeginWrap, Hold, Pull and Release.
//   LogRopeCollision   runtime collider, provider and SDF queries, in both the broad and narrow
//                      phases.

#pragma once

#include "Logging/LogMacros.h"

/** High-level flow: the component facade, the subsystem, module lifecycles and phase transitions. */
DECLARE_LOG_CATEGORY_EXTERN(LogDynamicRope, Log, All);

/** Physics: the XPBD solver. Diagnostics for substeps and the distance, bending and collision
 *  constraints. */
DECLARE_LOG_CATEGORY_EXTERN(LogRopeSolver, Log, All);

/** Logic: the wrap controller, covering BeginWrap, Hold, Pull and Release. */
DECLARE_LOG_CATEGORY_EXTERN(LogRopeWrap, Log, All);

/** Collision: runtime collider, provider and SDF queries in the broad and narrow phases. */
DECLARE_LOG_CATEGORY_EXTERN(LogRopeCollision, Log, All);
