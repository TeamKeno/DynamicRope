// Copyright Epic Games, Inc. All Rights Reserved.

#include "Render/RopeSceneProxy.h"
#include "RopeComponent.h"

#include "Materials/Material.h"
#include "Materials/MaterialRenderProxy.h"
#include "MaterialDomain.h"
#include "SceneInterface.h"
#include "SceneManagement.h"
#include "SceneView.h"
#include "PrimitiveViewRelevance.h"
#include "PrimitiveUniformShaderParametersBuilder.h"
#include "Engine/Engine.h"

void FRopeIndexBuffer::InitRHI(FRHICommandListBase& RHICmdList)
{
	const FRHIBufferCreateDesc CreateDesc =
		FRHIBufferCreateDesc::CreateIndex<int32>(TEXT("FRopeIndexBuffer"), NumIndices)
		.AddUsage(EBufferUsageFlags::Dynamic | EBufferUsageFlags::ShaderResource)
		.DetermineInitialState();

	IndexBufferRHI = RHICmdList.CreateBuffer(CreateDesc);
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
	, MaterialRelevance(Component->GetMaterialRelevance(GetScene().GetShaderPlatform()))
	, NumRings(FMath::Max(2, Component->NumParticles))
	, NumSides(FMath::Max(3, Component->NumSides))
	, Radius(Component->Radius)
{
	VertexBuffers.InitWithDummyData(&VertexFactory, GetRequiredVertexCount());
	IndexBuffer.NumIndices = GetRequiredIndexCount();

	if (!Material)
	{
		Material = UMaterial::GetDefaultMaterial(MD_Surface);
	}

	ENQUEUE_RENDER_COMMAND(InitRopeResources)(
		[this](FRHICommandListBase& RHICmdList)
		{
			IndexBuffer.InitResource(RHICmdList);
		});
}

FRopeSceneProxy::~FRopeSceneProxy()
{
	VertexBuffers.PositionVertexBuffer.ReleaseResource();
	VertexBuffers.StaticMeshVertexBuffer.ReleaseResource();
	VertexBuffers.ColorVertexBuffer.ReleaseResource();
	IndexBuffer.ReleaseResource();
	VertexFactory.ReleaseResource();
}

void FRopeSceneProxy::BuildTube(FRHICommandListBase& RHICmdList, const FRopeDynamicData& Data)
{
	const TArray<FVector>& Points = Data.Points;
	if (Points.Num() != NumRings)
	{
		// Centerline must match the fixed topology this proxy was built for.
		return;
	}

	// Seed a frame perpendicular to the first tangent, then parallel-transport it ring to ring
	// (minimal rotation) so the tube doesn't twist-pop like a Frenet frame would.
	FVector3f PrevTangent = FVector3f(Points[1] - Points[0]).GetSafeNormal();
	if (PrevTangent.IsNearlyZero())
	{
		PrevTangent = FVector3f::XAxisVector;
	}
	const FVector3f SeedUp = (FMath::Abs(PrevTangent.Z) < 0.99f) ? FVector3f::ZAxisVector : FVector3f::XAxisVector;
	FVector3f U = (SeedUp ^ PrevTangent).GetSafeNormal();
	FVector3f V = (PrevTangent ^ U).GetSafeNormal();

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
		const float AlongFrac = static_cast<float>(i) / static_cast<float>(NumRings - 1);

		for (int32 s = 0; s <= NumSides; ++s)
		{
			const float AroundFrac = static_cast<float>(s) / static_cast<float>(NumSides);
			const float Angle = 2.0f * PI * AroundFrac;
			const FVector3f Radial = (U * FMath::Cos(Angle) + V * FMath::Sin(Angle)).GetSafeNormal();

			VertexBuffers.PositionVertexBuffer.VertexPosition(VertIdx) = Center + Radial * Radius;
			VertexBuffers.StaticMeshVertexBuffer.SetVertexUV(VertIdx, 0, FVector2f(AlongFrac, AroundFrac));
			VertexBuffers.ColorVertexBuffer.VertexColor(VertIdx) = FColor::White;
			VertexBuffers.StaticMeshVertexBuffer.SetVertexTangents(VertIdx, Tangent, FVector3f(Radial ^ Tangent), Radial);
			++VertIdx;
		}
	}
	check(VertIdx == static_cast<uint32>(GetRequiredVertexCount()));

	// Upload vertex streams.
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

	// Build + upload indices (topology is constant, but cheap to refill).
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
		BuildTube(RHICmdList, *NewData);
		delete NewData;
	}
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
	FPrimitiveViewRelevance Result;
	Result.bDrawRelevance = IsShown(View);
	Result.bShadowRelevance = IsShadowCast(View);
	Result.bDynamicRelevance = true;
	Result.bRenderInMainPass = ShouldRenderInMainPass();
	Result.bRenderCustomDepth = ShouldRenderCustomDepth();
	MaterialRelevance.SetPrimitiveViewRelevance(Result);
	Result.bVelocityRelevance = DrawsVelocity() && Result.bOpaque && Result.bRenderInMainPass;
	return Result;
}
