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
#include "RopeTubeBuilder.h"        // RopeGPU::BuildTube_RenderThread (DynamicRopeShaders 모듈) — M5b
#include "RopeGPUSolver.h"          // FRopeGPUSolver::GetResidentPositionSRV_RenderThread — M5b B2-lite
#include "Subsystem/RopeSimSubsystem.h"
#include "RHICommandList.h"         // FRHITransitionInfo
#include "HAL/IConsoleManager.h"

// M5b: 1이면 튜브 position을 GPU 컴퓨트로 생성(UAV vertex buffer)해 렌더 리드백을 줄인다(검증 단계 B1: position만,
// tangent/UV/color는 CPU 유지). 0이면 기존 CPU BuildTube. proxy 생성 시점에 한 번 읽으므로, 토글 후엔 렌더 상태
// 재생성(예: 재PIE/가시성 토글)이 필요하다.
static TAutoConsoleVariable<int32> CVarRopeGPUTube(
	TEXT("r.DynamicRope.GPUTube"),
	0,
	TEXT("DynamicRope: 0=CPU 튜브 빌드(기본), 1=GPU 컴퓨트로 position 생성(M5b UAV vertex buffer 검증)."),
	ECVF_Default);

void FRopeIndexBuffer::InitRHI(FRHICommandListBase& RHICmdList)
{
	const FRHIBufferCreateDesc CreateDesc =
		FRHIBufferCreateDesc::CreateIndex<int32>(TEXT("FRopeIndexBuffer"), NumIndices)
		.AddUsage(EBufferUsageFlags::Dynamic | EBufferUsageFlags::ShaderResource)
		.DetermineInitialState();

	IndexBufferRHI = RHICmdList.CreateBuffer(CreateDesc);
}

// M5b: UAV 가능 position vertex buffer. 컴퓨트가 R32_FLOAT UAV로 쓰고, VF가 R32_FLOAT SRV/stream으로 읽는다.
void FRopeGpuPositionBuffer::InitRHI(FRHICommandListBase& RHICmdList)
{
	const uint32 Bytes = static_cast<uint32>(NumVertices) * sizeof(FVector3f);
	const FRHIBufferCreateDesc CreateDesc =
		FRHIBufferCreateDesc::CreateVertex(TEXT("FRopeGpuPositionBuffer"), Bytes)
		.AddUsage(EBufferUsageFlags::ShaderResource | EBufferUsageFlags::UnorderedAccess)
		.DetermineInitialState();
	VertexBufferRHI = RHICmdList.CreateBuffer(CreateDesc);

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

// M5b(B1 임시): 매 프레임 CPU centerline을 올려 컴퓨트가 읽는 버퍼. R32_FLOAT SRV.
// Dynamic 미사용 — Dynamic은 lock 시 backing을 orphan해 영속 SRV를 무효화할 수 있다(FPositionVertexBuffer와
// 동일하게 static+ShaderResource를 매 프레임 lock하면 SRV가 유지된다).
void FRopeCenterlineBuffer::InitRHI(FRHICommandListBase& RHICmdList)
{
	const uint32 Bytes = static_cast<uint32>(NumFloats) * sizeof(float);
	const FRHIBufferCreateDesc CreateDesc =
		FRHIBufferCreateDesc::CreateVertex(TEXT("FRopeCenterlineBuffer"), Bytes)
		.AddUsage(EBufferUsageFlags::ShaderResource)
		.DetermineInitialState();
	VertexBufferRHI = RHICmdList.CreateBuffer(CreateDesc);

	SRV = RHICmdList.CreateShaderResourceView(VertexBufferRHI,
		FRHIViewDesc::CreateBufferSRV().SetType(FRHIViewDesc::EBufferType::Typed).SetFormat(PF_R32_FLOAT));
}

void FRopeCenterlineBuffer::ReleaseRHI()
{
	SRV.SafeRelease();
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
	, MaterialRelevance(Component->GetMaterialRelevance(GetScene().GetShaderPlatform()))
	, NumRings(FMath::Max(2, Component->NumParticles))
	, NumSides(FMath::Max(3, Component->NumSides))
	, Radius(Component->Radius)
{
	VertexBuffers.InitWithDummyData(&VertexFactory, GetRequiredVertexCount());
	IndexBuffer.NumIndices = GetRequiredIndexCount();

	// M5b: GPU 튜브 경로 여부를 생성 시점에 한 번 결정(런타임 토글은 렌더 상태 재생성 후 반영).
	bUseGpuTube = CVarRopeGPUTube.GetValueOnGameThread() != 0 && NumRings <= 256;

	// B2-lite: 솔버 resident PosBuf를 직접 읽기 위한 핸들(GT에서 캡처). 솔버는 월드 수명이라 proxy 동안 유효.
	RopeId = Component->GetUniqueID();
	if (URopeSimSubsystem* Sub = URopeSimSubsystem::Get(Component->GetWorld()))
	{
		SolverPtr = Sub->GetGpuSolver();
	}

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

			if (bUseGpuTube)
			{
				// position stream을 GPU 컴퓨트가 쓰는 UAV 버퍼로 교체(tangent/UV/color는 VertexBuffers에서 바인딩).
				GpuPositionBuffer.NumVertices = GetRequiredVertexCount();
				GpuPositionBuffer.InitResource(RHICmdList);
				CenterlineBuffer.NumFloats = NumRings * 3;
				CenterlineBuffer.InitResource(RHICmdList);

				// GetData()는 protected라 FDataType을 새로 구성: position만 커스텀 UAV 버퍼, 나머지는 VertexBuffers
				// (InitWithDummyData가 먼저 enqueue돼 이 커맨드 시점엔 초기화됨)에서 public Bind* 헬퍼로 채운다.
				FLocalVertexFactory::FDataType Data;
				Data.PositionComponent = FVertexStreamComponent(&GpuPositionBuffer, 0, sizeof(FVector3f), VET_Float3);
				Data.PositionComponentSRV = GpuPositionBuffer.SRV;
				VertexBuffers.StaticMeshVertexBuffer.BindTangentVertexBuffer(&VertexFactory, Data);
				VertexBuffers.StaticMeshVertexBuffer.BindTexCoordVertexBuffer(&VertexFactory, Data);
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
	// M5b: GPU 튜브 버퍼(미초기화여도 ReleaseResource는 안전).
	GpuPositionBuffer.ReleaseResource();
	CenterlineBuffer.ReleaseResource();
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

void FRopeSceneProxy::BuildTubeGPU(FRHICommandListBase& /*RHICmdListBase*/, const FRopeDynamicData& Data)
{
	// 렌더 스레드. 컴퓨트 디스패치/transition엔 즉시 커맨드리스트가 필요(전달된 base list와 동일 객체).
	FRHICommandListImmediate& RHICmdList = FRHICommandListExecutor::GetImmediateCommandList();

	// tangent/UV/color/index는 CPU 경로 그대로 사용(positions는 unbound VertexBuffers.Position으로 가 낭비되지만 무해).
	BuildTube(RHICmdList, Data);
	if (Data.Points.Num() != NumRings)
	{
		return;
	}

	// B2-lite: 솔버 resident PosBuf(월드)를 직접 읽어 위치 무지연. 단 이번 프레임에 실제로 GPU step된 로프만
	// (Data.bGpuResident) — whip/throw/CPU-폴백 프레임엔 PosBuf가 stale이라 CPU 미러(B1: Data.Points)로 그린다.
	int32 ResidentNodes = 0;
	FRHIShaderResourceView* ResidentSRV = (Data.bGpuResident && SolverPtr)
		? SolverPtr->GetResidentPositionSRV_RenderThread(RopeId, ResidentNodes) : nullptr;
	const bool bResident = (ResidentSRV != nullptr && ResidentNodes == NumRings);

	if (!bResident)
	{
		const int32 NumFloats = NumRings * 3;
		float* Dst = static_cast<float*>(RHICmdList.LockBuffer(CenterlineBuffer.VertexBufferRHI, 0, NumFloats * sizeof(float), RLM_WriteOnly));
		for (int32 i = 0; i < NumRings; ++i)
		{
			Dst[i * 3 + 0] = static_cast<float>(Data.Points[i].X);
			Dst[i * 3 + 1] = static_cast<float>(Data.Points[i].Y);
			Dst[i * 3 + 2] = static_cast<float>(Data.Points[i].Z);
		}
		RHICmdList.UnlockBuffer(CenterlineBuffer.VertexBufferRHI);
	}

	// position UAV에 GPU 튜브 생성. UAV write → vertex stream read 사이 배리어.
	RHICmdList.Transition(FRHITransitionInfo(GpuPositionBuffer.VertexBufferRHI, ERHIAccess::Unknown, ERHIAccess::UAVCompute));
	if (bResident)
	{
		// 월드 PosBuf → component-local 변환. proxy는 GetLocalToWorld()로 렌더하므로 WorldToLocal = inverse.
		const FMatrix44f WorldToLocal(GetLocalToWorld().Inverse());
		RopeGPU::BuildTubeFromResident_RenderThread(RHICmdList, ResidentSRV, GpuPositionBuffer.UAV, NumRings, NumSides, Radius, WorldToLocal);
	}
	else
	{
		RopeGPU::BuildTube_RenderThread(RHICmdList, CenterlineBuffer.SRV, GpuPositionBuffer.UAV, NumRings, NumSides, Radius);
	}
	RHICmdList.Transition(FRHITransitionInfo(GpuPositionBuffer.VertexBufferRHI, ERHIAccess::UAVCompute, ERHIAccess::SRVGraphics | ERHIAccess::VertexOrIndexBuffer));

	bHasData = true;
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
