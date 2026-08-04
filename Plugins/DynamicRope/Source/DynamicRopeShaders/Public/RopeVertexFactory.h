// Copyright Epic Games, Inc. All Rights Reserved.
//
// The rope tube's vertex factory: FLocalVertexFactory under its own vertex factory type.
//
// A rope tube rewrites its vertex buffers in place every frame, so without help the engine can
// derive its motion vectors only from the component transform, which is wrong for a deforming mesh:
// temporal upscalers reproject the tube as static geometry (ghosting) and per-object motion blur
// smears it. LocalVertexFactory.ush already contains the fix - the GPU-skin passthrough branch
// reads a per-vertex previous position from a loose-parameter buffer and falls back to zero
// deformation velocity when the buffer's frame number does not match the view - but driving that
// branch from engine code requires FGPUSkinPassthroughVertexFactory, which is not exported from the
// Engine module. Everything the branch actually needs is public, though: the loose parameter
// struct, the shader define hook, and the parameter-binding base class. This factory therefore
// reproduces the small C++ side under a rope-owned type and reuses the engine .ush unmodified;
// there is no forked shader to maintain.
//
// A dedicated type also means dedicated material shader permutations, so compilation is gated to
// the materials a rope can actually use; see ShouldCompilePermutation.
//
// All of that needs UE 5.6 or newer; see ROPE_WITH_VELOCITY_PASSTHROUGH below for the 5.5 fallback.

#pragma once

#include "CoreMinimal.h"
#include "LocalVertexFactory.h"
#include "Misc/EngineVersionComparison.h"

// Whether this engine version can host the rope's own vertex factory type.
//
// Two things it needs are exported from the Engine module only from 5.6 on:
// FLocalVertexFactoryShaderParametersBase, which any LocalVertexFactory-derived type outside the
// Engine module must derive its parameter class from, and FGPUSkinPassThroughFactoryLooseParameters,
// the struct the passthrough shader branch reads. UE 5.5 exports neither - its parameter base has no
// ENGINE_API at all and its loose parameters live in the unexported FLocalVertexFactoryLooseParameters
// - so on 5.5 no plugin can register a type of its own here, whatever it is willing to reimplement.
//
// On 5.5 FRopeVertexFactory is therefore a thin FLocalVertexFactory that adds no vertex factory type
// of its own: the tube renders through the engine's local vertex factory exactly as it did before the
// velocity path existed, with motion vectors derived from the component transform. Everything below
// keeps its signature there, so the scene proxy compiles unchanged on every supported version.
#define ROPE_WITH_VELOCITY_PASSTHROUGH (!UE_VERSION_OLDER_THAN(5, 6, 0))

#if ROPE_WITH_VELOCITY_PASSTHROUGH

// FGPUSkinPassThroughFactoryLooseParameters, the engine-declared loose parameter struct the
// passthrough branch of LocalVertexFactory.ush reads. Declared ENGINE_API, so a plugin can create
// and bind uniform buffers of it even though the engine's passthrough vertex factory itself is not
// exported.
#include "GPUSkinVertexFactory.h"

/**
 * The vertex factory the rope's scene proxy renders with.
 *
 * With the velocity passthrough disabled it behaves exactly like FLocalVertexFactory: the
 * passthrough branch is compiled in (the type advertises SupportsGPUSkinPassThrough) but the
 * parameter class binds it inactive, with the null loose parameters.
 *
 * With it enabled, the proxy hands the factory the current/previous position SRVs and the
 * game-thread frame number once per frame through UpdateLooseParameters, and the parameter class
 * binds the passthrough branch active. The shader then reads the previous position per vertex for
 * the velocity pass, and degrades to zero deformation velocity on any frame whose number does not
 * match the view - which makes stale data (the first frame, a reseed, a paused world) safe by
 * construction. Enablement is decided once, before InitResource, because the shader bindings are
 * captured into cached mesh draw commands; per-frame data flows only through the uniform buffer,
 * which is updated in place.
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

	/**
	 * Enables the velocity passthrough. Must be called before InitResource: the decision is baked
	 * into the cached mesh draw command bindings, so it is fixed for the factory's lifetime.
	 * The caller is responsible for checking IsGPUSkinPassThroughSupported for the platform.
	 */
	DYNAMICROPESHADERS_API void SetVelocityPassThroughEnabled(bool bEnabled);
	bool IsVelocityPassThroughEnabled() const { return bVelocityPassThrough; }

	/** Whether this engine version can drive the passthrough at all; see the macro above. */
	static constexpr bool IsVelocityPassThroughAvailable() { return true; }

	/** Whether the rope's material must carry the skeletal-mesh usage; see ShouldCompilePermutation. */
	static constexpr bool RequiresSkeletalMeshUsage() { return true; }

	/**
	 * Render thread, once per frame before the frame's positions are written: points the loose
	 * parameters at the current and previous position buffers and stamps the frame number.
	 * Pass MAX_uint32 as the frame number to mark the previous positions invalid for this frame;
	 * the shader then outputs zero deformation velocity.
	 * The uniform buffer contents are updated in place, so bindings captured in cached mesh draw
	 * commands stay valid.
	 */
	DYNAMICROPESHADERS_API void UpdateLooseParameters(FRHICommandListBase& RHICmdList, uint32 FrameNumber,
		FRHIShaderResourceView* PositionSRV, FRHIShaderResourceView* PreviousPositionSRV,
		FRHIShaderResourceView* PreSkinnedTangentSRV);

	const TUniformBufferRef<FGPUSkinPassThroughFactoryLooseParameters>& GetLooseParametersUniformBuffer() const
	{
		return LooseParametersUniformBuffer;
	}

private:
	virtual void InitRHI(FRHICommandListBase& RHICmdList) override;
	virtual void ReleaseRHI() override;

	// Whether the velocity passthrough is bound active; fixed before InitResource.
	bool bVelocityPassThrough = false;

	// The loose parameters the passthrough branch reads. Created once in InitRHI, so it exists by
	// the time the first mesh draw command is cached, and updated in place afterwards.
	TUniformBufferRef<FGPUSkinPassThroughFactoryLooseParameters> LooseParametersUniformBuffer;
};

#else

/**
 * UE 5.5: the rope tube renders through the engine's own local vertex factory.
 *
 * No vertex factory type is declared here, so GetType stays FLocalVertexFactory's and every engine
 * path - permutation compilation, cached mesh draw commands, the parameter bindings - behaves as it
 * does for any other local vertex factory. The velocity entry points are kept so the scene proxy
 * needs no version branch of its own; they do nothing, and IsVelocityPassThroughAvailable reports
 * that, which is what keeps the proxy from advertising velocity it cannot write.
 */
class FRopeVertexFactory : public FLocalVertexFactory
{
public:
	FRopeVertexFactory(ERHIFeatureLevel::Type InFeatureLevel, const char* InDebugName)
		: FLocalVertexFactory(InFeatureLevel, InDebugName)
	{
	}

	static constexpr bool IsVelocityPassThroughAvailable() { return false; }
	/** No rope-owned type here, so the engine's own permutations apply and any material draws. */
	static constexpr bool RequiresSkeletalMeshUsage() { return false; }
	bool IsVelocityPassThroughEnabled() const { return false; }
	void SetVelocityPassThroughEnabled(bool) {}

	void UpdateLooseParameters(FRHICommandListBase&, uint32,
		FRHIShaderResourceView*, FRHIShaderResourceView*, FRHIShaderResourceView*)
	{
	}
};

#endif