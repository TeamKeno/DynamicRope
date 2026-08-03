// Copyright Epic Games, Inc. All Rights Reserved.

#include "Render/RopeSceneProxy.h"
#include "RopeComponent.h"

#include "Materials/Material.h"
// CheckMaterialUsage_Concurrent + MATUSAGE_SkeletalMesh, the usage FRopeVertexFactory's material
// permutations are gated on.
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialRenderProxy.h"
#include "MaterialDomain.h"
#include "DynamicRopeLog.h"
#include "SceneInterface.h"
#include "SceneManagement.h"
#include "SceneView.h"
#include "PrimitiveViewRelevance.h"
#include "PrimitiveUniformShaderParametersBuilder.h"
#include "Engine/Engine.h"
// RopeGPU::BuildTube_RenderThread / BuildTubeFromResident_RenderThread
#include "RopeTubeBuilder.h"
// FRopeGPUSolver::GetResidentPositionSRV_RenderThread — the resident centerline the GPU tube reads directly
#include "RopeGPUSolver.h"
#include "Subsystem/RopeSimSubsystem.h"
// FRHITransitionInfo
#include "RHICommandList.h"
// GDynamicRHI
#include "RHI.h"
// FApp::CanEverRender
#include "Misc/App.h"
// UE_VERSION_OLDER_THAN, the engine version guard, since the GetMaterialRelevance signature changed in 5.7.
#include "Misc/EngineVersionComparison.h"
// The wrapper covering the RHI buffer creation difference between engine versions.
#include "RopeRHICompat.h"
// bWriteVelocity, snapshotted once when the proxy is created.
#include "Settings/DynamicRopeSettings.h"
// IsGPUSkinPassThroughSupported, the platform gate for the per-vertex velocity path.
#include "RenderUtils.h"

// Where the render tuning comes from: the tube smoothing, meaning the subdivision and the knot
// parameter, is a per-rope property on URopeComponent, while whether velocity is written is a project
// setting on UDynamicRopeSettings.
// All three are snapshotted once when the proxy is created, so a change takes effect after the render
// state is recreated, whether by editing the property in the editor or restarting PIE.
// The resident tube, which reads the nodes directly, smooths on the GPU and therefore still applies with
// a subdivision above 1; only non-resident frames smooth on the CPU before uploading.

void FRopeIndexBuffer::InitRHI(FRHICommandListBase& RHICmdList)
{
	IndexBufferRHI = RopeRHI::CreateIndexBuffer(RHICmdList, TEXT("FRopeIndexBuffer"), sizeof(int32), NumIndices,
		EBufferUsageFlags::Dynamic | EBufferUsageFlags::ShaderResource);

	// Before BuildTube fills them in, as outside PIE where there is no subsystem tick and no centreline
	// has been uploaded, a cached static draw would render uninitialized, that is garbage, indices,
	// producing degenerate triangles across the origin and a black smear across the world. Initializing
	// them to zero collapses every triangle onto vertex 0 with zero area, so nothing is drawn, and real
	// data renders normally once it arrives. Static draws are designed to update their buffers in place,
	// so guarding DrawStaticElements on whether data exists would stop the command being recached and it
	// would never draw at all; a safe initial state is used instead of a guard.
	if (NumIndices > 0)
	{
		void* Dst = RHICmdList.LockBuffer(IndexBufferRHI, 0, NumIndices * sizeof(int32), RLM_WriteOnly);
		FMemory::Memzero(Dst, NumIndices * sizeof(int32));
		RHICmdList.UnlockBuffer(IndexBufferRHI);
	}
}

// The UAV-capable position vertex buffer. The compute shader writes it through an R32_FLOAT UAV and the
// vertex factory reads it as an R32_FLOAT SRV and stream.
void FRopeGpuPositionBuffer::InitRHI(FRHICommandListBase& RHICmdList)
{
	const uint32 Bytes = static_cast<uint32>(NumVertices) * sizeof(FVector3f);
	VertexBufferRHI = RopeRHI::CreateVertexBuffer(RHICmdList, TEXT("FRopeGpuPositionBuffer"), Bytes,
		EBufferUsageFlags::ShaderResource | EBufferUsageFlags::UnorderedAccess);

	SRV = RHICmdList.CreateShaderResourceView(VertexBufferRHI,
		FRHIViewDesc::CreateBufferSRV().SetType(FRHIViewDesc::EBufferType::Typed).SetFormat(PF_R32_FLOAT));
	UAV = RHICmdList.CreateUnorderedAccessView(VertexBufferRHI,
		FRHIViewDesc::CreateBufferUAV().SetType(FRHIViewDesc::EBufferType::Typed).SetFormat(PF_R32_FLOAT));
}

void FRopeGpuPositionBuffer::ReleaseRHI()
{
	SRV.SafeRelease();
	UAV.SafeRelease();
	FVertexBuffer::ReleaseRHI();
}

// The buffer the compute shader reads, filled each frame by uploading the CPU centreline, with an
// R32_FLOAT SRV.
// It is deliberately not dynamic: a dynamic buffer can orphan its backing store on lock and invalidate a
// persistent SRV, whereas locking a static buffer with the shader resource flag every frame, as
// FPositionVertexBuffer does, keeps the SRV valid.
void FRopeCenterlineBuffer::InitRHI(FRHICommandListBase& RHICmdList)
{
	const uint32 Bytes = static_cast<uint32>(NumFloats) * sizeof(float);
	VertexBufferRHI = RopeRHI::CreateVertexBuffer(RHICmdList, TEXT("FRopeCenterlineBuffer"), Bytes,
		EBufferUsageFlags::ShaderResource);

	SRV = RHICmdList.CreateShaderResourceView(VertexBufferRHI,
		FRHIViewDesc::CreateBufferSRV().SetType(FRHIViewDesc::EBufferType::Typed).SetFormat(PF_R32_FLOAT));
}

void FRopeCenterlineBuffer::ReleaseRHI()
{
	SRV.SafeRelease();
	FVertexBuffer::ReleaseRHI();
}

// The tangent basis buffer. The compute shader writes a uint4 of packed SNORM16 values per vertex through
// an R32_UINT UAV, and the vertex factory reads it as VET_Short4N streams, with tangent X at offset 0 and
// tangent Z at offset 8 and a stride of 16, plus an R16G16B16A16_SNORM SRV for the manual fetch.
void FRopeGpuTangentBuffer::InitRHI(FRHICommandListBase& RHICmdList)
{
	// TangentX(8) + TangentZ(8)
	const uint32 Bytes = static_cast<uint32>(NumVertices) * 16;
	VertexBufferRHI = RopeRHI::CreateVertexBuffer(RHICmdList, TEXT("FRopeGpuTangentBuffer"), Bytes,
		EBufferUsageFlags::ShaderResource | EBufferUsageFlags::UnorderedAccess);

	SRV = RHICmdList.CreateShaderResourceView(VertexBufferRHI,
		FRHIViewDesc::CreateBufferSRV().SetType(FRHIViewDesc::EBufferType::Typed).SetFormat(PF_R16G16B16A16_SNORM));
	UAV = RHICmdList.CreateUnorderedAccessView(VertexBufferRHI,
		FRHIViewDesc::CreateBufferUAV().SetType(FRHIViewDesc::EBufferType::Typed).SetFormat(PF_R32_UINT));
}

void FRopeGpuTangentBuffer::ReleaseRHI()
{
	SRV.SafeRelease();
	UAV.SafeRelease();
	FVertexBuffer::ReleaseRHI();
}

// The UV buffer. The compute shader writes a float2 per vertex through an R32_FLOAT UAV, and the vertex
// factory reads it as a VET_Float2 stream plus a G32R32F SRV for the manual fetch.
void FRopeGpuTexCoordBuffer::InitRHI(FRHICommandListBase& RHICmdList)
{
	const uint32 Bytes = static_cast<uint32>(NumVertices) * sizeof(FVector2f);
	VertexBufferRHI = RopeRHI::CreateVertexBuffer(RHICmdList, TEXT("FRopeGpuTexCoordBuffer"), Bytes,
		EBufferUsageFlags::ShaderResource | EBufferUsageFlags::UnorderedAccess);

	SRV = RHICmdList.CreateShaderResourceView(VertexBufferRHI,
		FRHIViewDesc::CreateBufferSRV().SetType(FRHIViewDesc::EBufferType::Typed).SetFormat(PF_G32R32F));
	UAV = RHICmdList.CreateUnorderedAccessView(VertexBufferRHI,
		FRHIViewDesc::CreateBufferUAV().SetType(FRHIViewDesc::EBufferType::Typed).SetFormat(PF_R32_FLOAT));
}

void FRopeGpuTexCoordBuffer::ReleaseRHI()
{
	SRV.SafeRelease();
	UAV.SafeRelease();
	FVertexBuffer::ReleaseRHI();
}

SIZE_T FRopeSceneProxy::GetTypeHash() const
{
	static size_t UniquePointer;
	return reinterpret_cast<size_t>(&UniquePointer);
}

FRopeSceneProxy::FRopeSceneProxy(URopeComponent* Component)
	: FPrimitiveSceneProxy(Component)
	, Material(Component->GetMaterial(0))
	, VertexFactory(GetScene().GetFeatureLevel(), "FRopeSceneProxy")
	// In 5.7 the GetMaterialRelevance argument changed from a feature level to a shader platform, hence
	// the version guard.
	, MaterialRelevance(Component->GetMaterialRelevance(
#if UE_VERSION_OLDER_THAN(5, 7, 0)
		GetScene().GetFeatureLevel()
#else
		GetScene().GetShaderPlatform()
#endif
	))
	, NumNodes(FMath::Max(2, Component->NumParticles))
	// Reduced automatically to fit the GPU tube's ring limit, through RopeGPU::ComputeTubeSubdiv, which is
	// the single source shared with the debug overlay.
	, Subdiv(RopeGPU::ComputeTubeSubdiv(NumNodes, Component->TubeSmoothingSubdiv))
	// The number of smoothed render rings, which equals the node count when there is no subdivision.
	, NumRings((NumNodes - 1) * Subdiv + 1)
	, NumSides(FMath::Max(3, Component->NumSides))
	, Radius(Component->Radius)
	, SmoothParam(FMath::Clamp(Component->TubeSmoothingAlpha, 0.0f, 1.0f))
	// The per-vertex velocity path needs the passthrough branch compiled into the local vertex factory
	// shaders, which the engine only does on platforms with GPU-skin passthrough support; elsewhere the
	// rope simply writes no velocity, as before.
	, bVelocityActive(UDynamicRopeSettings::Get()->bWriteVelocity && IsGPUSkinPassThroughSupported(GMaxRHIShaderPlatform))
{
	// Decided before InitWithDummyData triggers InitResource: the flag is baked into the cached mesh
	// draw command bindings, so it cannot change for the proxy's lifetime.
	VertexFactory.SetVelocityPassThroughEnabled(bVelocityActive);
	// A deforming mesh has per-vertex motion every frame regardless of its transform, so the velocity
	// pass must include the rope even on frames where the component did not move; without this a rope
	// whipped from a stationary component drops out of the velocity pass entirely.
	bAlwaysHasVelocity = bVelocityActive;

	VertexBuffers.InitWithDummyData(&VertexFactory, GetRequiredVertexCount());
	IndexBuffer.NumIndices = GetRequiredIndexCount();

	// The GPU tube is the default path: with a renderable RHI and a ring count within the maximum, which
	// is the largest thread group bucket, the positions, tangents and UVs are produced by compute
	// shaders; otherwise, as in a cook, under -nullrhi, on a server, or when the rings exceed the limit,
	// it falls back to the CPU tube build. There is no console variable, and the selection is automatic
	// exactly as it is for the solver. The limit references the bucket definition in RopeTubeBuilder as a
	// single source, which prevents the two drifting apart.
	// It is decided once at creation, since the ring count is fixed for the proxy's lifetime.
	// The runtime support test uses the same function as the solver's gate, RopeGPU::IsRuntimeSupported,
	// which checks for an RHI and at least SM5. Letting the two gates diverge would produce the half state
	// of a GPU solver with a CPU tube.
	bUseGpuTube = RopeGPU::IsRuntimeSupported() && NumRings <= RopeGPU::MaxTubeRings();

	// The handle used to read the solver's resident position buffer directly, captured on the game thread.
	// The solver lives as long as the world, so it stays valid for the proxy's lifetime.
	RopeId = Component->GetUniqueID();
	if (URopeSimSubsystem* Sub = URopeSimSubsystem::Get(Component->GetWorld()))
	{
		SolverPtr = Sub->GetGpuSolver();
	}

	// FRopeVertexFactory compiles material shader permutations only for materials flagged
	// "Used with Skeletal Mesh", plus the engine's special materials; see its ShouldCompilePermutation.
	// Checking the usage here, on the game thread that creates the proxy, sets the flag automatically
	// in the editor, which triggers that compile. In a cooked game a material shipped without the flag
	// has no shaders for this factory, the check fails, and the default material is drawn instead of
	// nothing.
	if (Material && !Material->CheckMaterialUsage_Concurrent(MATUSAGE_SkeletalMesh))
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("%s: material '%s' is not marked 'Used with Skeletal Mesh', which the rope's vertex factory requires; drawing the default material instead."),
			*GetOwnerName().ToString(), *Material->GetName());
		Material = nullptr;
	}

	if (!Material)
	{
		Material = UMaterial::GetDefaultMaterial(MD_Surface);
	}

	// We rewrite the vertex buffer directly every frame, through an RHI lock. The renderer's automatic
	// shadow cache heuristics, which look at world position offset and transform deltas, cannot detect
	// that, so a virtual shadow map keeps its stale cached page and the rope's shadow leaves a duplicated
	// afterimage on the floor until movement nearby happens to invalidate the page. Setting this to always
	// puts the primitive into the shadow scene's always-invalidating list, which makes the virtual shadow
	// map invalidate it unconditionally every frame.
	// (VirtualShadowMapCacheManager: GetAlwaysInvalidatingPrimitives -> UpdatedTransform).
	bHasDeformableMesh = true;
	ShadowCacheInvalidationBehavior = EShadowCacheInvalidationBehavior::Always;
	// A rope deforms every frame through direct buffer writes, so its shadow must never be cached.
	// Forcing this to false makes IsMeshShapeOftenMoving() true regardless of the component's mobility,
	// which a Blueprint or an instance could quietly have set to static, and keeps the rope on the
	// uncached dynamic virtual shadow map path; the cache manager derives its dynamic-primitive decision
	// from it.
	bGoodCandidateForCachedShadowmap = false;

	ENQUEUE_RENDER_COMMAND(InitRopeResources)(
		[this](FRHICommandListBase& RHICmdList)
		{
			IndexBuffer.InitResource(RHICmdList);

			// The previous-position buffer serves both tube paths, so it is sized and initialized
			// independently of the GPU-tube decision.
			if (bVelocityActive)
			{
				PrevPositionBuffer.NumVertices = GetRequiredVertexCount();
				PrevPositionBuffer.InitResource(RHICmdList);
			}

			if (bUseGpuTube)
			{
				// Every stream, meaning the positions, tangents and UVs, is replaced by a UAV buffer the
				// compute shader writes; only the colour comes from the standard vertex buffers.
				const int32 VertCount = GetRequiredVertexCount();
				GpuPositionBuffer.NumVertices = VertCount;
				GpuPositionBuffer.InitResource(RHICmdList);
				GpuTangentBuffer.NumVertices = VertCount;
				GpuTangentBuffer.InitResource(RHICmdList);
				GpuTexCoordBuffer.NumVertices = VertCount;
				GpuTexCoordBuffer.InitResource(RHICmdList);
				CenterlineBuffer.NumFloats = NumRings * 3;
				CenterlineBuffer.InitResource(RHICmdList);

				// The data type is constructed directly: the positions, tangents and texture coordinates use
				// the custom UAV buffers, as streams plus manual-fetch SRVs, while the colour uses the
				// standard colour vertex buffer, which is already initialized at this point because the
				// dummy-data initialization was enqueued first.
				FLocalVertexFactory::FDataType Data;
				Data.PositionComponent = FVertexStreamComponent(&GpuPositionBuffer, 0, sizeof(FVector3f), VET_Float3);
				Data.PositionComponentSRV = GpuPositionBuffer.SRV;

				// tangent basis: TangentX@0, TangentZ@8, stride 16, VET_Short4N(SNORM16). SRV = R16G16B16A16_SNORM.
				Data.TangentBasisComponents[0] = FVertexStreamComponent(&GpuTangentBuffer, 0, 16, VET_Short4N);
				Data.TangentBasisComponents[1] = FVertexStreamComponent(&GpuTangentBuffer, 8, 16, VET_Short4N);
				Data.TangentsSRV = GpuTangentBuffer.SRV;

				// UVs: VET_Float2 with a stride of 8, an SRV of G32R32F, and one texture coordinate set.
				Data.TextureCoordinates.Empty();
				Data.TextureCoordinates.Add(FVertexStreamComponent(&GpuTexCoordBuffer, 0, sizeof(FVector2f), VET_Float2));
				Data.TextureCoordinatesSRV = GpuTexCoordBuffer.SRV;
				Data.NumTexCoords = 1;
				Data.LightMapCoordinateIndex = 0;

				VertexBuffers.ColorVertexBuffer.BindColorVertexBuffer(&VertexFactory, Data);
				VertexFactory.SetData(RHICmdList, Data);
			}
		});
}

FRopeSceneProxy::~FRopeSceneProxy()
{
	VertexBuffers.PositionVertexBuffer.ReleaseResource();
	VertexBuffers.StaticMeshVertexBuffer.ReleaseResource();
	VertexBuffers.ColorVertexBuffer.ReleaseResource();
	IndexBuffer.ReleaseResource();
	VertexFactory.ReleaseResource();
	// The GPU tube buffers; releasing them is safe even when they were never initialized.
	GpuPositionBuffer.ReleaseResource();
	GpuTangentBuffer.ReleaseResource();
	GpuTexCoordBuffer.ReleaseResource();
	CenterlineBuffer.ReleaseResource();
	PrevPositionBuffer.ReleaseResource();
}

void FRopeSceneProxy::BuildSmoothedCenterline(const TArray<FVector>& Nodes, TArray<FVector>& Out) const
{
	Out.SetNumUninitialized(NumRings);

	// With no subdivision there is no smoothing, so the nodes are copied one-to-one, clamping
	// defensively if the counts disagree.
	if (Subdiv <= 1 || Nodes.Num() < 2)
	{
		for (int32 r = 0; r < NumRings; ++r)
		{
			Out[r] = Nodes[FMath::Clamp(r, 0, Nodes.Num() - 1)];
		}
		return;
	}

	const int32 LastNode = NumNodes - 1;
	for (int32 r = 0; r < NumRings; ++r)
	{
		// The segment this ring belongs to, spanning nodes Seg and Seg + 1.
		const int32 Seg = r / Subdiv;
		// The sub-index within that segment.
		const int32 Sub = r % Subdiv;
		if (Seg >= LastNode)
		{
			// The final ring, which lands exactly on the last node.
			Out[r] = Nodes[LastNode];
			continue;
		}
		const float T = static_cast<float>(Sub) / static_cast<float>(Subdiv);

		// The Catmull-Rom control points, clamped at the ends. The tangents come from the neighbouring
		// nodes, which estimates the local curvature.
		const FVector P0 = Nodes[FMath::Max(Seg - 1, 0)];
		const FVector P1 = Nodes[Seg];
		const FVector P2 = Nodes[Seg + 1];
		const FVector P3 = Nodes[FMath::Min(Seg + 2, LastNode)];

		// A parameterized Catmull-Rom, mirroring RopeCatmullSmooth on the GPU. The knot parameter is 0 for
		// uniform, the older standard with a tension of 0.5, and 0.5 for centripetal, which reduces tangent
		// overshoot at sharp corners so the middle rings hug a wall more closely. The knot is the distance
		// between points raised to that power, with a lower bound.
		const double EPS = 1e-4;
		const double t01 = FMath::Pow(FMath::Max((P1 - P0).Size(), EPS), (double)SmoothParam);
		const double t12 = FMath::Pow(FMath::Max((P2 - P1).Size(), EPS), (double)SmoothParam);
		const double t23 = FMath::Pow(FMath::Max((P3 - P2).Size(), EPS), (double)SmoothParam);
		const FVector M1 = (P2 - P1) + t12 * ((P1 - P0) / t01 - (P2 - P0) / (t01 + t12));
		const FVector M2 = (P2 - P1) + t12 * ((P3 - P2) / t23 - (P3 - P1) / (t12 + t23));
		const FVector A =  2.0 * (P1 - P2) + M1 + M2;
		const FVector B = -3.0 * (P1 - P2) - 2.0 * M1 - M2;
		Out[r] = ((A * T + B) * T + M1) * T + P1;
	}
}

void FRopeSceneProxy::BuildTube(FRHICommandListBase& RHICmdList, const FRopeDynamicData& Data)
{
	if (Data.Points.Num() != NumNodes)
	{
		// The centreline, meaning the simulation nodes, has to match the node count the proxy was created
		// with.
		return;
	}
	// Simulation nodes become the smoothed render centreline. With no subdivision the nodes are used
	// as-is. Everything below builds rings from the smoothed points.
	TArray<FVector> Points;
	BuildSmoothedCenterline(Data.Points, Points);

	// Seed a frame perpendicular to the first tangent and then parallel-transport it ring by ring, which
	// is the minimum rotation. That stops the tube twist-popping the way a Frenet frame does.
	FVector3f PrevTangent = FVector3f(Points[1] - Points[0]).GetSafeNormal();
	if (PrevTangent.IsNearlyZero())
	{
		PrevTangent = FVector3f::XAxisVector;
	}
	const FVector3f SeedUp = (FMath::Abs(PrevTangent.Z) < 0.99f) ? FVector3f::ZAxisVector : FVector3f::XAxisVector;
	FVector3f U = (SeedUp ^ PrevTangent).GetSafeNormal();
	FVector3f V = (PrevTangent ^ U).GetSafeNormal();

	// U is the accumulated arc length divided by the circumference, so U and V share a physical scale and
	// the twist density stays constant regardless of the rope's length.
	const float InvCirc = 1.0f / FMath::Max(2.0f * PI * Radius, KINDA_SMALL_NUMBER);
	float AlongLen = 0.0f;

	uint32 VertIdx = 0;
	for (int32 i = 0; i < NumRings; ++i)
	{
		FVector3f Tangent = (i < NumRings - 1)
			? FVector3f(Points[i + 1] - Points[i]).GetSafeNormal()
			: FVector3f(Points[i] - Points[i - 1]).GetSafeNormal();
		if (Tangent.IsNearlyZero())
		{
			Tangent = PrevTangent;
		}

		if (i > 0)
		{
			const FQuat4f Turn = FQuat4f::FindBetweenNormals(PrevTangent, Tangent);
			U = Turn.RotateVector(U);
			V = Turn.RotateVector(V);
		}
		PrevTangent = Tangent;

		const FVector3f Center(Points[i]);
		if (i > 0)
		{
			AlongLen += static_cast<float>((Points[i] - Points[i - 1]).Size());
		}
		const float AlongFrac = AlongLen * InvCirc;

		for (int32 s = 0; s <= NumSides; ++s)
		{
			const float AroundFrac = static_cast<float>(s) / static_cast<float>(NumSides);
			const float Angle = 2.0f * PI * AroundFrac;
			const FVector3f Radial = (U * FMath::Cos(Angle) + V * FMath::Sin(Angle)).GetSafeNormal();

			VertexBuffers.PositionVertexBuffer.VertexPosition(VertIdx) = Center + Radial * Radius;
			VertexBuffers.StaticMeshVertexBuffer.SetVertexUV(VertIdx, 0, FVector2f(AlongFrac, AroundFrac));
			VertexBuffers.ColorVertexBuffer.VertexColor(VertIdx) = FColor::White;
			// The bitangent is the tangent crossed with the radial, which points along increasing V, that is
			// around the circumference, and matches normal map conventions. The opposite order has the
			// opposite sign and would invert the circumferential normal.
			VertexBuffers.StaticMeshVertexBuffer.SetVertexTangents(VertIdx, Tangent, FVector3f(Tangent ^ Radial), Radial);
			++VertIdx;
		}
	}
	check(VertIdx == static_cast<uint32>(GetRequiredVertexCount()));

	// Upload the vertex streams.
	{
		FPositionVertexBuffer& VB = VertexBuffers.PositionVertexBuffer;
		void* Dst = RHICmdList.LockBuffer(VB.VertexBufferRHI, 0, VB.GetNumVertices() * VB.GetStride(), RLM_WriteOnly);
		FMemory::Memcpy(Dst, VB.GetVertexData(), VB.GetNumVertices() * VB.GetStride());
		RHICmdList.UnlockBuffer(VB.VertexBufferRHI);
	}
	{
		FColorVertexBuffer& CB = VertexBuffers.ColorVertexBuffer;
		void* Dst = RHICmdList.LockBuffer(CB.VertexBufferRHI, 0, CB.GetNumVertices() * CB.GetStride(), RLM_WriteOnly);
		FMemory::Memcpy(Dst, CB.GetVertexData(), CB.GetNumVertices() * CB.GetStride());
		RHICmdList.UnlockBuffer(CB.VertexBufferRHI);
	}
	{
		FStaticMeshVertexBuffer& SB = VertexBuffers.StaticMeshVertexBuffer;
		void* Dst = RHICmdList.LockBuffer(SB.TangentsVertexBuffer.VertexBufferRHI, 0, SB.GetTangentSize(), RLM_WriteOnly);
		FMemory::Memcpy(Dst, SB.GetTangentData(), SB.GetTangentSize());
		RHICmdList.UnlockBuffer(SB.TangentsVertexBuffer.VertexBufferRHI);
	}
	{
		FStaticMeshVertexBuffer& SB = VertexBuffers.StaticMeshVertexBuffer;
		void* Dst = RHICmdList.LockBuffer(SB.TexCoordVertexBuffer.VertexBufferRHI, 0, SB.GetTexCoordSize(), RLM_WriteOnly);
		FMemory::Memcpy(Dst, SB.GetTexCoordData(), SB.GetTexCoordSize());
		RHICmdList.UnlockBuffer(SB.TexCoordVertexBuffer.VertexBufferRHI);
	}

	// Build and upload the indices. The topology is constant, but refilling it is cheap.
	int32* Indices = static_cast<int32*>(RHICmdList.LockBuffer(IndexBuffer.IndexBufferRHI, 0, GetRequiredIndexCount() * sizeof(int32), RLM_WriteOnly));
	uint32 Out = 0;
	for (int32 i = 0; i < NumRings - 1; ++i)
	{
		for (int32 s = 0; s < NumSides; ++s)
		{
			const int32 A = GetVertIndex(i, s);
			const int32 B = GetVertIndex(i, s + 1);
			const int32 C = GetVertIndex(i + 1, s);
			const int32 D = GetVertIndex(i + 1, s + 1);
			Indices[Out++] = A; Indices[Out++] = C; Indices[Out++] = B;
			Indices[Out++] = B; Indices[Out++] = C; Indices[Out++] = D;
		}
	}
	RHICmdList.UnlockBuffer(IndexBuffer.IndexBufferRHI);
	check(Out == static_cast<uint32>(GetRequiredIndexCount()));

	bHasData = true;
}

void FRopeSceneProxy::SetDynamicData_RenderThread(FRHICommandListBase& RHICmdList, FRopeDynamicData* NewData)
{
	check(IsInRenderingThread());
	if (NewData)
	{
		if (bVelocityActive)
		{
			// Before the builds overwrite the position buffers: at this point they still hold last
			// frame's tube, which is exactly the previous-position data the velocity pass needs.
			UpdatePreviousPositions(FRHICommandListExecutor::GetImmediateCommandList(), *NewData);
		}

		if (bUseGpuTube)
		{
			BuildTubeGPU(RHICmdList, *NewData);
		}
		else
		{
			BuildTube(RHICmdList, *NewData);
		}
		delete NewData;
	}
}

void FRopeSceneProxy::UpdatePreviousPositions(FRHICommandListImmediate& RHICmdList, const FRopeDynamicData& Data)
{
	FRHIBuffer* Current = bUseGpuTube ? GpuPositionBuffer.VertexBufferRHI : VertexBuffers.PositionVertexBuffer.VertexBufferRHI;
	FRHIBuffer* Previous = PrevPositionBuffer.VertexBufferRHI;
	if (!Current || !Previous)
	{
		return;
	}

	{
		FRHITransitionInfo ToCopy[2] = {
			FRHITransitionInfo(Current,  ERHIAccess::Unknown, ERHIAccess::CopySrc),
			FRHITransitionInfo(Previous, ERHIAccess::Unknown, ERHIAccess::CopyDest),
		};
		RHICmdList.Transition(MakeArrayView(ToCopy, 2));
	}
	RHICmdList.CopyBufferRegion(Previous, 0, Current, 0,
		static_cast<uint64>(GetRequiredVertexCount()) * sizeof(FVector3f));
	{
		// The current buffer returns to its read state; the tube build that follows transitions it
		// itself on the GPU path, and the CPU path's lock-update manages its own upload copy.
		FRHITransitionInfo FromCopy[2] = {
			FRHITransitionInfo(Current,  ERHIAccess::CopySrc,  ERHIAccess::SRVGraphics | ERHIAccess::VertexOrIndexBuffer),
			FRHITransitionInfo(Previous, ERHIAccess::CopyDest, ERHIAccess::SRVGraphics),
		};
		RHICmdList.Transition(MakeArrayView(FromCopy, 2));
	}

	// The copy is genuinely last frame's pose only if a tube has been built at all and the sim was not
	// reseeded since; otherwise a stale frame number keeps the shader on zero deformation velocity for
	// this frame. The position and tangent SRVs come from the factory's data, which resolves the GPU
	// tube's UAV streams and the CPU fallback's vertex buffers alike.
	const bool bPrevIsLastPose = bHasData && Data.SimGeneration == LastVelocitySimGeneration;
	LastVelocitySimGeneration = Data.SimGeneration;
	VertexFactory.UpdateLooseParameters(RHICmdList,
		bPrevIsLastPose ? Data.FrameNumber : MAX_uint32,
		VertexFactory.GetPositionsSRV(),
		PrevPositionBuffer.SRV,
		VertexFactory.GetTangentsSRV());
}

void FRopeSceneProxy::BuildGpuStaticBuffers(FRHICommandListBase& RHICmdList)
{
	// Fill the index topology, which never changes between frames, and the constant white colour once,
	// replacing the CPU tube build.
	{
		int32* Indices = static_cast<int32*>(RHICmdList.LockBuffer(IndexBuffer.IndexBufferRHI, 0, GetRequiredIndexCount() * sizeof(int32), RLM_WriteOnly));
		uint32 Out = 0;
		for (int32 i = 0; i < NumRings - 1; ++i)
		{
			for (int32 s = 0; s < NumSides; ++s)
			{
				const int32 A = GetVertIndex(i, s);
				const int32 B = GetVertIndex(i, s + 1);
				const int32 C = GetVertIndex(i + 1, s);
				const int32 D = GetVertIndex(i + 1, s + 1);
				Indices[Out++] = A; Indices[Out++] = C; Indices[Out++] = B;
				Indices[Out++] = B; Indices[Out++] = C; Indices[Out++] = D;
			}
		}
		RHICmdList.UnlockBuffer(IndexBuffer.IndexBufferRHI);
	}
	{
		FColorVertexBuffer& CB = VertexBuffers.ColorVertexBuffer;
		for (uint32 v = 0; v < CB.GetNumVertices(); ++v)
		{
			CB.VertexColor(v) = FColor::White;
		}
		void* Dst = RHICmdList.LockBuffer(CB.VertexBufferRHI, 0, CB.GetNumVertices() * CB.GetStride(), RLM_WriteOnly);
		FMemory::Memcpy(Dst, CB.GetVertexData(), CB.GetNumVertices() * CB.GetStride());
		RHICmdList.UnlockBuffer(CB.VertexBufferRHI);
	}
	bGpuStaticsBuilt = true;
}

void FRopeSceneProxy::BuildTubeGPU(FRHICommandListBase& /*RHICmdListBase*/, const FRopeDynamicData& Data)
{
	// Render thread. Dispatching compute work and issuing transitions need an immediate command list,
	// which is the same object as the base list passed in.
	FRHICommandListImmediate& RHICmdList = FRHICommandListExecutor::GetImmediateCommandList();

	if (Data.Points.Num() != NumNodes)
	{
		return;
	}

	// Fill the index topology and the colour once; later frames regenerate only the positions, tangents
	// and UVs.
	if (!bGpuStaticsBuilt)
	{
		BuildGpuStaticBuffers(RHICmdList);
	}

	// Read the solver's resident position buffer, holding the simulation nodes in world space, directly
	// and smooth it on the GPU.
	// Only for a rope actually stepped on the GPU this frame: on a whip, throw or CPU fallback frame that
	// buffer is stale, so the CPU mirror is smoothed on the CPU and uploaded instead.
	// On the global distance field path, where the solve runs during PreRenderBasePass, this command runs
	// before the solve and therefore reads the previous frame's result, which is an intended one-frame
	// render delay. Rebuilding the tube after the solve would leave the depth prepass, which used this
	// build, inconsistent with the base pass geometry and the pixels would fail the equal-depth test,
	// turning the rope black, so this single build per frame is kept and every pass sees the same
	// geometry.
	int32 ResidentNodes = 0;
	uint32 ResidentGeneration = 0;
	FRHIShaderResourceView* ResidentSRV = (Data.bGpuResident && SolverPtr)
		? SolverPtr->GetResidentPositionSRV_RenderThread(RopeId, ResidentNodes, ResidentGeneration) : nullptr;
	// Both the node count and the seed generation have to match before reading directly. On a reseed frame
	// the buffer is still on the old generation, because the reseed happens in that frame's dispatch, so
	// checking the node count alone would draw the previous rope pose for one frame.
	// That frame falls back to the CPU mirror, which already holds the new seed and is therefore correct.
	const bool bResident = (ResidentSRV != nullptr && ResidentNodes == NumNodes
		&& ResidentGeneration == Data.SimGeneration);

	if (!bResident)
	{
		// The non-resident fallback: the CPU mirror is smoothed with Catmull-Rom interpolation and uploaded
		// to the centreline buffer.
		TArray<FVector> Smoothed;
		BuildSmoothedCenterline(Data.Points, Smoothed);
		const int32 NumFloats = NumRings * 3;
		float* Dst = static_cast<float*>(RHICmdList.LockBuffer(CenterlineBuffer.VertexBufferRHI, 0, NumFloats * sizeof(float), RLM_WriteOnly));
		for (int32 i = 0; i < NumRings; ++i)
		{
			Dst[i * 3 + 0] = static_cast<float>(Smoothed[i].X);
			Dst[i * 3 + 1] = static_cast<float>(Smoothed[i].Y);
			Dst[i * 3 + 2] = static_cast<float>(Smoothed[i].Z);
		}
		RHICmdList.UnlockBuffer(CenterlineBuffer.VertexBufferRHI);
	}

	// Generate the tube into the position, tangent and UV UAVs, with a barrier between the UAV write and
	// the vertex stream read for all three buffers.
	FRHITransitionInfo ToUAV[3] = {
		FRHITransitionInfo(GpuPositionBuffer.VertexBufferRHI, ERHIAccess::Unknown, ERHIAccess::UAVCompute),
		FRHITransitionInfo(GpuTangentBuffer.VertexBufferRHI,  ERHIAccess::Unknown, ERHIAccess::UAVCompute),
		FRHITransitionInfo(GpuTexCoordBuffer.VertexBufferRHI, ERHIAccess::Unknown, ERHIAccess::UAVCompute),
	};
	RHICmdList.Transition(MakeArrayView(ToUAV, 3));

	if (bResident)
	{
		// Convert the world-space position buffer into component-local space, using the inverse matrix the
		// game thread built from this frame's component transform. Reading GetLocalToWorld().Inverse() here
		// would be wrong: this command runs before UpdateAllPrimitiveSceneInfos applies this frame's
		// transform, so it would be one frame behind, and building at frame N-1 while drawing at frame N
		// makes a world-fixed point, such as a wrap node, jitter by exactly the component's movement.
		// The GPU smooths the simulation nodes with Catmull-Rom interpolation at the subdivision factor to
		// produce the render centreline, and then builds the tube.
		RopeGPU::BuildTubeFromResident_RenderThread(RHICmdList, ResidentSRV,
			GpuPositionBuffer.UAV, GpuTangentBuffer.UAV, GpuTexCoordBuffer.UAV,
			NumRings, NumSides, Radius, NumNodes, Subdiv, SmoothParam, Data.WorldToLocal);
	}
	else
	{
		RopeGPU::BuildTube_RenderThread(RHICmdList, CenterlineBuffer.SRV,
			GpuPositionBuffer.UAV, GpuTangentBuffer.UAV, GpuTexCoordBuffer.UAV, NumRings, NumSides, Radius);
	}

	const ERHIAccess ToRead = ERHIAccess::SRVGraphics | ERHIAccess::VertexOrIndexBuffer;
	FRHITransitionInfo ToVtx[3] = {
		FRHITransitionInfo(GpuPositionBuffer.VertexBufferRHI, ERHIAccess::UAVCompute, ToRead),
		FRHITransitionInfo(GpuTangentBuffer.VertexBufferRHI,  ERHIAccess::UAVCompute, ToRead),
		FRHITransitionInfo(GpuTexCoordBuffer.VertexBufferRHI, ERHIAccess::UAVCompute, ToRead),
	};
	RHICmdList.Transition(MakeArrayView(ToVtx, 3));

	bHasData = true;
}

void FRopeSceneProxy::DrawStaticElements(FStaticPrimitiveDrawInterface* PDI)
{
	// The cable-style static draw path: a cached mesh draw command against the persistent vertex factory.
	// BuildTube() updates the vertex buffer in place every frame, so the cached command renders the current
	// geometry. The velocity pass reuses the same cached draw: the per-vertex previous positions flow
	// through the factory's loose parameter uniform buffer, updated in place, so caching stays valid.
	if (HasViewDependentDPG())
	{
		return;
	}

	FMeshBatch Mesh;
	Mesh.VertexFactory = &VertexFactory;
	Mesh.MaterialRenderProxy = Material->GetRenderProxy();
	Mesh.ReverseCulling = IsLocalToWorldDeterminantNegative();
	Mesh.Type = PT_TriangleList;
	Mesh.DepthPriorityGroup = SDPG_World;
	Mesh.LODIndex = 0;
	Mesh.MeshIdInPrimitive = 0;
	Mesh.SegmentIndex = 0;

	FMeshBatchElement& BatchElement = Mesh.Elements[0];
	BatchElement.IndexBuffer = &IndexBuffer;
	BatchElement.FirstIndex = 0;
	BatchElement.NumPrimitives = GetRequiredIndexCount() / 3;
	BatchElement.MinVertexIndex = 0;
	BatchElement.MaxVertexIndex = GetRequiredVertexCount() - 1;

	PDI->DrawMesh(Mesh, FLT_MAX);
}

void FRopeSceneProxy::GetDynamicMeshElements(const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily,
	uint32 VisibilityMap, FMeshElementCollector& Collector) const
{
	if (!bHasData)
	{
		return;
	}

	const bool bWireframe = AllowDebugViewmodes() && ViewFamily.EngineShowFlags.Wireframe;

	FColoredMaterialRenderProxy* WireframeMaterial = new FColoredMaterialRenderProxy(
		GEngine->WireframeMaterial ? GEngine->WireframeMaterial->GetRenderProxy() : nullptr,
		FLinearColor(0.0f, 0.5f, 1.0f));
	Collector.RegisterOneFrameMaterialProxy(WireframeMaterial);

	FMaterialRenderProxy* MaterialProxy = bWireframe ? WireframeMaterial : Material->GetRenderProxy();

	for (int32 ViewIndex = 0; ViewIndex < Views.Num(); ++ViewIndex)
	{
		if (!(VisibilityMap & (1 << ViewIndex)))
		{
			continue;
		}

		FMeshBatch& Mesh = Collector.AllocateMesh();
		Mesh.bWireframe = bWireframe;
		Mesh.VertexFactory = &VertexFactory;
		Mesh.MaterialRenderProxy = MaterialProxy;
		Mesh.ReverseCulling = IsLocalToWorldDeterminantNegative();
		Mesh.Type = PT_TriangleList;
		Mesh.DepthPriorityGroup = SDPG_World;
		Mesh.bCanApplyViewModeOverrides = false;

		FDynamicPrimitiveUniformBuffer& DynamicUniformBuffer = Collector.AllocateOneFrameResource<FDynamicPrimitiveUniformBuffer>();
		FPrimitiveUniformShaderParametersBuilder Builder;
		BuildUniformShaderParameters(Builder);
		DynamicUniformBuffer.Set(Collector.GetRHICommandList(), Builder);

		FMeshBatchElement& BatchElement = Mesh.Elements[0];
		BatchElement.IndexBuffer = &IndexBuffer;
		BatchElement.PrimitiveUniformBufferResource = &DynamicUniformBuffer.UniformBuffer;
		BatchElement.FirstIndex = 0;
		BatchElement.NumPrimitives = GetRequiredIndexCount() / 3;
		BatchElement.MinVertexIndex = 0;
		BatchElement.MaxVertexIndex = GetRequiredVertexCount() - 1;

		Collector.AddMesh(ViewIndex, Mesh);
	}
}

FPrimitiveViewRelevance FRopeSceneProxy::GetViewRelevance(const FSceneView* View) const
{
	// This follows FCableSceneProxy exactly: static relevance for ordinary views, through the cached draw
	// in DrawStaticElements, and dynamic only for wireframe, rich and debug views, handled in
	// GetDynamicMeshElements.
	FPrimitiveViewRelevance Result;
	Result.bDrawRelevance = IsShown(View);
	Result.bShadowRelevance = IsShadowCast(View);
	Result.bRenderInMainPass = ShouldRenderInMainPass();
	Result.bRenderCustomDepth = ShouldRenderCustomDepth();
	Result.bUsesLightingChannels = GetLightingChannelMask() != GetDefaultLightingChannelMask();

	const bool bWireframe = AllowDebugViewmodes() && View->Family->EngineShowFlags.Wireframe;
	if (IsRichView(*View->Family) || bWireframe || View->Family->EngineShowFlags.Bounds || HasViewDependentDPG())
	{
		Result.bDynamicRelevance = true;
	}
	else
	{
		Result.bStaticRelevance = true;
	}

	MaterialRelevance.SetPrimitiveViewRelevance(Result);
	// With the per-vertex velocity path active the rope joins the velocity pass, and its motion vectors
	// are correct because the vertex factory supplies last frame's vertex positions - temporal upscalers
	// reproject the rope instead of ghosting it, and motion blur blurs along the real motion. With it
	// off, whether by the project setting or a platform without the passthrough shader branch, no
	// velocity is written at all: a transform-only velocity would be wrong for a deforming mesh, smearing
	// under motion blur without fixing the ghosting.
	Result.bVelocityRelevance = bVelocityActive
		&& DrawsVelocity() && Result.bOpaque && Result.bRenderInMainPass;
	return Result;
}
