// Copyright 2026 TeamKeno. All Rights Reserved.
//
// The DynamicRopeShaders module's log category. This module does not depend on DynamicRope, so it has its own category.

#pragma once

#include "Logging/LogMacros.h"

/** The GPU solver and shader plumbing: path mapping and dispatch or readback diagnostics. */
DECLARE_LOG_CATEGORY_EXTERN(LogDynamicRopeGPU, Log, All);
