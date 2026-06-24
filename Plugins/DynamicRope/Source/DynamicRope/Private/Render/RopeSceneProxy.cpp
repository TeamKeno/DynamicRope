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

	// 우리는 매 프레임 vertex buffer를 직접 다시 쓴다(RHI lock). 렌더러의 Auto
	// shadow-cache 휴리스틱(WPO / transform 델타)은 이를 감지하지 못하므로, Virtual Shadow Map은
	// 오래된 cached page를 유지하고, 근처에서 움직임이 발생해 page가 무효화될 때까지 rope의 그림자가
	// 바닥에 중복된 잔상을 남긴다. `Always`는 이 primitive를 ShadowScene의
	// AlwaysInvalidatingPrimitives에 넣어, VSM이 매 프레임 무조건 무효화하게 한다
	// (VirtualShadowMapCacheManager: GetAlwaysInvalidatingPrimitives -> UpdatedTransform).
	bHasDeformableMesh = true;
	ShadowCacheInvalidationBehavior = EShadowCacheInvalidationBehavior::Always;
	// rope는 직접 buffer 쓰기로 매 프레임 변형되므로 그 그림자는 절대 캐싱되어서는 안 된다.
	// 이를 false로 강제하면 component의 Mobility(블루프린트/인스턴스가 조용히 Static으로 설정할 수 있음)와
	// 무관하게 IsMeshShapeOftenMoving() == true가 되어, rope를 캐싱되지 않는
	// dynamic VSM shadow 경로에 유지한다(VirtualShadowMapCacheManager가 이로부터 CachePrimitiveAsDynamic를 설정).
	bGoodCandidateForCachedShadowmap = false;

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
		// centerline은 이 proxy가 생성된 기준인 고정 topology와 일치해야 한다.
		return;
	}

	// 첫 tangent에 수직인 frame을 시드한 뒤, ring 단위로 parallel-transport한다
	// (최소 회전). 그러면 tube가 Frenet frame처럼 twist-pop하지 않는다.
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

	// vertex stream을 업로드한다.
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

	// index를 만들고 업로드한다(topology은 일정하지만 다시 채우는 비용이 저렴하다).
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

void FRopeSceneProxy::DrawStaticElements(FStaticPrimitiveDrawInterface* PDI)
{
	// Cable 스타일의 static draw 경로: 지속적인 vertex factory에 대한 cached mesh draw command.
	// 매 프레임 BuildTube()가 vertex buffer를 제자리에서 갱신하므로, cached command가
	// 현재 geometry를 렌더링한다. static relevance(Movable/dynamic이 아님)는 잘못된 motion-vector ghosting을 피한다.
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
	// FCableSceneProxy를 그대로 따른다: 일반 view에는 static relevance(DrawStaticElements를 통한 cached draw),
	// wireframe / rich / debug view에만 dynamic(GetDynamicMeshElements에서 처리).
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
	Result.bVelocityRelevance = DrawsVelocity() && Result.bOpaque && Result.bRenderInMainPass;
	return Result;
}
