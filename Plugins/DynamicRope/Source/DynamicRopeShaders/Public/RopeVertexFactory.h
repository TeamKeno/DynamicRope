// Copyright Epic Games, Inc. All Rights Reserved.
//
// The rope tube's vertex factory: FLocalVertexFactory under its own vertex factory type.
//
// A rope tube rewrites its vertex buffers in place every frame, so the engine can derive its motion
// vectors only from the component transform, which is wrong for a deforming mesh: temporal
// upscalers reproject the tube as static geometry (ghosting) and per-object motion blur smears it.
// LocalVertexFactory.ush already contains the fix - the GPU-skin passthrough branch reads a
// per-vertex previous position from a loose-parameter buffer and falls back to zero deformation
// velocity when the buffer's frame number does not match the view - but driving that branch from
// engine code requires FGPUSkinPassthroughVertexFactory, which is not exported from the Engine
// module. Everything the branch actually needs is public, though: the loose parameter struct, the
// shader define hook, and the parameter-binding base class. This factory therefore reproduces the
// small C++ side under a rope-owned type and reuses the engine .ush unmodified; there is no forked
// shader to maintain.
//
// A dedicated type also means dedicated material shader permutations, so compilation is gated to
// the materials a rope can actually use; see ShouldCompilePermutation.

#pragma once

#include "CoreMinimal.h"
#include "LocalVertexFactory.h"

/**
 * The vertex factory the rope's scene proxy renders with. It behaves exactly like
 * FLocalVertexFactory: the passthrough branch is compiled in (the type advertises
 * SupportsGPUSkinPassThrough) but the parameter class binds it inactive, with the null loose
 * parameters, until the proxy supplies a previous-position buffer for velocity output.
 */
class FRopeVertexFactory : public FLocalVertexFactory
{
	DECLARE_VERTEX_FACTORY_TYPE_API(FRopeVertexFactory, DYNAMICROPESHADERS_API);

public:
	DYNAMICROPESHADERS_API FRopeVertexFactory(ERHIFeatureLevel::Type InFeatureLevel, const char* InDebugName);

	/**
	 * Gates this type's material permutations to rope-capable materials: ones flagged
	 * "Used with Skeletal Mesh" plus the special engine materials, which keeps the default-material
	 * fallback compiled. The rope is a deforming mesh, and piggybacking on the skeletal usage flag
	 * avoids a custom usage bit, which a plugin cannot add; URopeComponent's proxy creation checks
	 * the usage, setting the flag automatically in the editor.
	 */
	static bool ShouldCompilePermutation(const FVertexFactoryShaderPermutationParameters& Parameters);
};
