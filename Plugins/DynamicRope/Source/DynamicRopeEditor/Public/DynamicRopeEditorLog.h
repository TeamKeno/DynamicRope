// Copyright Epic Games, Inc. All Rights Reserved.
//
// DynamicRopeEditor log categories. Include this header in any editor .cpp to use UE_LOG.
// Definitions (DEFINE) live once in DynamicRopeEditorModule.cpp.
//
// Usage:
//   LogDynamicRopeEditor — editor module lifecycle, tabs/menus, visualizers, asset tooling.
//   LogRopeSDFBake       — the SDF authoring/baking pipeline (the heavy offline editor op).

#pragma once

#include "Logging/LogMacros.h"

/** General editor tooling: module startup, tab/menu registration, component visualizers, factories. */
DECLARE_LOG_CATEGORY_EXTERN(LogDynamicRopeEditor, Log, All);

/** SDF authoring/baking pipeline (sampling, bake progress, asset write-out). */
DECLARE_LOG_CATEGORY_EXTERN(LogRopeSDFBake, Log, All);
