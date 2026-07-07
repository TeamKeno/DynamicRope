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
#include "RopeTubeBuilder.h"        // RopeGPU::BuildTube_RenderThread / BuildTubeFromResident_RenderThread — M5b
#include "RopeGPUSolver.h"          // FRopeGPUSolver::GetResidentPositionSRV_RenderThread — M5b B2-lite
#include "Subsystem/RopeSimSubsystem.h"
#include "RHICommandList.h"         // FRHITransitionInfo
#include "RHI.h"                    // GDynamicRHI
#include "HAL/IConsoleManager.h"
#include "Misc/App.h"               // FApp::CanEverRender

// 모션블러 잔상 대응. 로프는 매 프레임 정점을 in-place로 갱신하지만 per-vertex 변형 velocity를 만들지
// 못한다(prev-position 스트림 없음). Movable이라 DrawsVelocity()==true가 되면 transform 기반 velocity만
// 찍혀 velocity 패스에 들어가고, 빠르게 이동하는 프레임에 per-object 모션블러가 로프를 번지게 한다(잔상).
// 0(기본)=velocity 미출력 → 모션블러 대상에서 제외. 1=레거시(velocity 출력, 빠른 이동 시 모션블러).
// 트레이드오프: 0이면 TSR이 이 픽셀을 카메라 재투영으로 처리하므로 정지 카메라 + 빠른 로프에서 약한
// TSR 고스팅이 생길 수 있다 → 1로 토글해 A/B 비교 가능. GetViewRelevance(렌더 스레드)에서 읽는다.
static TAutoConsoleVariable<int32> CVarRopeWriteVelocity(
	TEXT("r.DynamicRope.WriteVelocity"),
	0,
	TEXT("DynamicRope: 0=velocity 미출력(모션블러 잔상 제거, 기본), 1=velocity 출력(레거시, 빠른 이동 시 모션블러)."),
	ECVF_RenderThreadSafe);

// 렌더 튜브 스무딩: 세그먼트당 Catmull-Rom 서브분할 수(1=off=노드당 링 1개, 기본 3). 시뮬 노드는 그대로 두고
// 렌더 센터라인만 이웃 노드로 곡률을 추정해 매끄럽게 편다(물리와 분리 → 리스크 0). 링 수/토폴로지가 바뀌므로
// proxy 생성 시 1회 읽는다 → 런타임 토글은 렌더 상태 재생성(재PIE/가시성 토글) 후 반영. Subdiv>1이면 GPU
// resident 튜브(노드 직독)는 링수 불일치로 자동 비활성 → 스무딩된 CPU 센터라인 업로드 경로로 폴백한다.
static TAutoConsoleVariable<int32> CVarRopeTubeSmoothing(
	TEXT("r.DynamicRope.TubeSmoothing"),
	3,
	TEXT("DynamicRope: 렌더 튜브 Catmull-Rom 서브분할(세그먼트당). 1=off. 물리 무관(렌더 전용)."),
	ECVF_Default);

void FRopeIndexBuffer::InitRHI(FRHICommandListBase& RHICmdList)
{
	const FRHIBufferCreateDesc CreateDesc =
		FRHIBufferCreateDesc::CreateIndex<int32>(TEXT("FRopeIndexBuffer"), NumIndices)
		.AddUsage(EBufferUsageFlags::Dynamic | EBufferUsageFlags::ShaderResource)
		.DetermineInitialState();

	IndexBufferRHI = RHICmdList.CreateBuffer(CreateDesc);

	// BuildTube가 채우기 전(예: PIE 밖, 서브시스템 틱이 없어 센터라인이 안 올라온 상태)에 캐시된 정적
	// 드로우가 미초기화 인덱스(쓰레기 값)를 그리면 원점을 가로지르는 degenerate 삼각형 = 월드를 가르는
	// 검은 번짐이 생긴다. 0으로 초기화하면 모든 삼각형이 정점 0으로 수축한 zero-area라 아무것도 안 그려진다
	// (정상 데이터가 채워지면 그대로 렌더). 정적 드로우는 버퍼를 in-place로 갱신하는 설계라 DrawStaticElements를
	// bHasData로 가드하면 커맨드가 재캐싱되지 않아 영영 안 그려지므로, 가드 대신 안전한 초기 상태로 둔다.
	if (NumIndices > 0)
	{
		void* Dst = RHICmdList.LockBuffer(IndexBufferRHI, 0, NumIndices * sizeof(int32), RLM_WriteOnly);
		FMemory::Memzero(Dst, NumIndices * sizeof(int32));
		RHICmdList.UnlockBuffer(IndexBufferRHI);
	}
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

// B2-full: tangent basis 버퍼. 컴퓨트는 R32_UINT UAV로 v당 uint4(SNORM16 packed)를 쓰고, VF는 VET_Short4N
// 스트림(TangentX@0, TangentZ@8, stride 16) + R16G16B16A16_SNORM SRV(매뉴얼 페치)로 읽는다.
void FRopeGpuTangentBuffer::InitRHI(FRHICommandListBase& RHICmdList)
{
	const uint32 Bytes = static_cast<uint32>(NumVertices) * 16; // TangentX(8) + TangentZ(8)
	const FRHIBufferCreateDesc CreateDesc =
		FRHIBufferCreateDesc::CreateVertex(TEXT("FRopeGpuTangentBuffer"), Bytes)
		.AddUsage(EBufferUsageFlags::ShaderResource | EBufferUsageFlags::UnorderedAccess)
		.DetermineInitialState();
	VertexBufferRHI = RHICmdList.CreateBuffer(CreateDesc);

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

// B2-full: UV 버퍼. 컴퓨트는 R32_FLOAT UAV로 v당 float2, VF는 VET_Float2 스트림 + G32R32F SRV(매뉴얼 페치)로 읽는다.
void FRopeGpuTexCoordBuffer::InitRHI(FRHICommandListBase& RHICmdList)
{
	const uint32 Bytes = static_cast<uint32>(NumVertices) * sizeof(FVector2f);
	const FRHIBufferCreateDesc CreateDesc =
		FRHIBufferCreateDesc::CreateVertex(TEXT("FRopeGpuTexCoordBuffer"), Bytes)
		.AddUsage(EBufferUsageFlags::ShaderResource | EBufferUsageFlags::UnorderedAccess)
		.DetermineInitialState();
	VertexBufferRHI = RHICmdList.CreateBuffer(CreateDesc);

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

// 렌더 튜브 Subdiv 결정: CVar 값(1..8)을 쓰되, NumRings=(NumNodes-1)*Subdiv+1이 GPU 튜브 링 상한
// (MaxTubeRings)을 넘지 않도록 자동으로 낮춘다. GPU-솔브 가능한(NumNodes ≤ MaxNodes) 로프는 노드 수와
// 무관하게 GPU 튜브를 유지하고, "크기 때문에 CPU 튜브로 떨어지는" 구간(GPU-솔브 + CPU-튜브 = 리드백 지연
// 부활 + 렌더 스레드 비용)이 사라진다. 커질수록 렌더 스무딩만 완만히 감소한다(512노드에서 Subdiv=1).
static int32 RopeComputeTubeSubdiv(int32 NumNodes)
{
	const int32 Wanted = FMath::Clamp(CVarRopeTubeSmoothing.GetValueOnGameThread(), 1, 8);
	if (NumNodes <= 2)
	{
		return Wanted;
	}
	const int32 MaxForGpu = FMath::Max(1, (RopeGPU::MaxTubeRings() - 1) / (NumNodes - 1));
	return FMath::Min(Wanted, MaxForGpu);
}

FRopeSceneProxy::FRopeSceneProxy(URopeComponent* Component)
	: FPrimitiveSceneProxy(Component)
	, Material(Component->GetMaterial(0))
	, VertexFactory(GetScene().GetFeatureLevel(), "FRopeSceneProxy")
	, MaterialRelevance(Component->GetMaterialRelevance(GetScene().GetShaderPlatform()))
	, NumNodes(FMath::Max(2, Component->NumParticles))
	, Subdiv(RopeComputeTubeSubdiv(NumNodes)) // 링 상한에 맞춰 자동 하향(위 헬퍼 주석 참고).
	, NumRings((NumNodes - 1) * Subdiv + 1) // 스무딩된 렌더 링 수(Subdiv=1이면 NumNodes와 동일).
	, NumSides(FMath::Max(3, Component->NumSides))
	, Radius(Component->Radius)
{
	VertexBuffers.InitWithDummyData(&VertexFactory, GetRequiredVertexCount());
	IndexBuffer.NumIndices = GetRequiredIndexCount();

	// GPU 튜브 상시화: 렌더 가능 RHI + NumRings<=MaxTubeRings(최상단 스레드그룹 버킷)이면 GPU 튜브
	// (pos/tangent/UV 컴퓨트), 아니면(쿡/-nullrhi/서버, 또는 링>상한) CPU BuildTube 폴백. CVar 토글 없음 —
	// G4 솔버와 동일한 자동 선택. 상한은 RopeTubeBuilder의 버킷 정의를 단일 소스로 참조(드리프트 방지).
	// 생성 시점에 한 번 결정(링 수는 proxy 수명 동안 고정).
	bUseGpuTube = (GDynamicRHI != nullptr && FApp::CanEverRender()) && NumRings <= RopeGPU::MaxTubeRings();

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
				// B2-full: position/tangent/UV 스트림을 모두 GPU 컴퓨트가 쓰는 UAV 버퍼로 교체(color만 VertexBuffers).
				const int32 VertCount = GetRequiredVertexCount();
				GpuPositionBuffer.NumVertices = VertCount;
				GpuPositionBuffer.InitResource(RHICmdList);
				GpuTangentBuffer.NumVertices = VertCount;
				GpuTangentBuffer.InitResource(RHICmdList);
				GpuTexCoordBuffer.NumVertices = VertCount;
				GpuTexCoordBuffer.InitResource(RHICmdList);
				CenterlineBuffer.NumFloats = NumRings * 3;
				CenterlineBuffer.InitResource(RHICmdList);

				// FDataType 직접 구성: position/tangent/texcoord는 커스텀 UAV 버퍼(스트림 + 매뉴얼 페치 SRV),
				// color는 VertexBuffers.ColorVertexBuffer(InitWithDummyData가 먼저 enqueue돼 이 시점 초기화됨).
				FLocalVertexFactory::FDataType Data;
				Data.PositionComponent = FVertexStreamComponent(&GpuPositionBuffer, 0, sizeof(FVector3f), VET_Float3);
				Data.PositionComponentSRV = GpuPositionBuffer.SRV;

				// tangent basis: TangentX@0, TangentZ@8, stride 16, VET_Short4N(SNORM16). SRV = R16G16B16A16_SNORM.
				Data.TangentBasisComponents[0] = FVertexStreamComponent(&GpuTangentBuffer, 0, 16, VET_Short4N);
				Data.TangentBasisComponents[1] = FVertexStreamComponent(&GpuTangentBuffer, 8, 16, VET_Short4N);
				Data.TangentsSRV = GpuTangentBuffer.SRV;

				// UV: VET_Float2, stride 8. SRV = G32R32F. TexCoord 1개.
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
	// M5b: GPU 튜브 버퍼(미초기화여도 ReleaseResource는 안전).
	GpuPositionBuffer.ReleaseResource();
	GpuTangentBuffer.ReleaseResource();
	GpuTexCoordBuffer.ReleaseResource();
	CenterlineBuffer.ReleaseResource();
}

void FRopeSceneProxy::BuildSmoothedCenterline(const TArray<FVector>& Nodes, TArray<FVector>& Out) const
{
	Out.SetNumUninitialized(NumRings);

	// Subdiv=1: 스무딩 없음 → 노드를 그대로 복사(1:1). (방어적으로 개수 불일치 시 clamp)
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
		const int32 Seg = r / Subdiv;      // 이 링이 속한 세그먼트(노드 Seg..Seg+1).
		const int32 Sub = r % Subdiv;      // 세그먼트 내 서브 인덱스.
		if (Seg >= LastNode)
		{
			Out[r] = Nodes[LastNode];      // 마지막 노드에 정확히 놓이는 끝 링.
			continue;
		}
		const float T = static_cast<float>(Sub) / static_cast<float>(Subdiv);

		// Catmull-Rom 제어점(끝에서 clamp). 접선은 이웃 노드로부터 → 국소 곡률 추정.
		const FVector P0 = Nodes[FMath::Max(Seg - 1, 0)];
		const FVector P1 = Nodes[Seg];
		const FVector P2 = Nodes[Seg + 1];
		const FVector P3 = Nodes[FMath::Min(Seg + 2, LastNode)];

		const float T2 = T * T;
		const float T3 = T2 * T;
		// 표준 Catmull-Rom(장력 0.5): 0.5*(2P1 + (P2-P0)t + (2P0-5P1+4P2-P3)t^2 + (-P0+3P1-3P2+P3)t^3).
		Out[r] = (P1 * 2.0
			+ (P2 - P0) * T
			+ (P0 * 2.0 - P1 * 5.0 + P2 * 4.0 - P3) * T2
			+ (P1 * 3.0 - P0 - P2 * 3.0 + P3) * T3) * 0.5;
	}
}

void FRopeSceneProxy::BuildTube(FRHICommandListBase& RHICmdList, const FRopeDynamicData& Data)
{
	if (Data.Points.Num() != NumNodes)
	{
		// centerline(시뮬 노드)은 proxy 생성 기준 노드 수와 일치해야 한다.
		return;
	}
	// 시뮬 노드 → 스무딩된 렌더 센터라인(NumRings). Subdiv=1이면 노드 그대로(1:1). 이하 링 빌드는 스무딩 점을 쓴다.
	TArray<FVector> Points;
	BuildSmoothedCenterline(Data.Points, Points);

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

void FRopeSceneProxy::BuildGpuStaticBuffers(FRHICommandListBase& RHICmdList)
{
	// B2-full: 매 프레임 불변인 index topology + 상수 color(white)를 1회만 채운다(CPU BuildTube 대체).
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
	// 렌더 스레드. 컴퓨트 디스패치/transition엔 즉시 커맨드리스트가 필요(전달된 base list와 동일 객체).
	FRHICommandListImmediate& RHICmdList = FRHICommandListExecutor::GetImmediateCommandList();

	if (Data.Points.Num() != NumNodes)
	{
		return;
	}

	// B2-full: index topology + color를 1회 채운다(이후 프레임은 위치/tangent/UV만 GPU 재생성).
	if (!bGpuStaticsBuilt)
	{
		BuildGpuStaticBuffers(RHICmdList);
	}

	// B2-full: 솔버 resident PosBuf(월드, 시뮬 노드 NumNodes개)를 직접 읽어 GPU에서 스무딩한다.
	// 단 이번 프레임에 실제로 GPU step된 로프만(Data.bGpuResident) — whip/throw/CPU-폴백 프레임엔 PosBuf가
	// stale이라 CPU 미러(Data.Points)를 CPU 스무딩해 업로드하는 B1 경로로 그린다.
	// GDF 모드(솔브가 PreRenderBasePass)에서 이 커맨드는 솔브보다 먼저 실행되므로 직전 프레임 솔브 결과를
	// 읽는다(의도된 1프레임 렌더 지연). 튜브를 솔브 뒤에 다시 빌드하면 depth prepass(이 빌드 결과)와 base
	// pass 지오메트리가 어긋나 EQUAL 깊이 테스트에서 픽셀이 탈락한다(로프가 검게 탐) — 그래서 프레임당 이
	// 1회 빌드만 유지하고 모든 패스가 같은 지오메트리를 보게 한다.
	int32 ResidentNodes = 0;
	FRHIShaderResourceView* ResidentSRV = (Data.bGpuResident && SolverPtr)
		? SolverPtr->GetResidentPositionSRV_RenderThread(RopeId, ResidentNodes) : nullptr;
	const bool bResident = (ResidentSRV != nullptr && ResidentNodes == NumNodes);

	if (!bResident)
	{
		// 비-resident 폴백: CPU 미러(Data.Points)를 Catmull-Rom 스무딩해 CenterlineBuffer에 업로드(B1).
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

	// pos/tangent/UV UAV에 GPU 튜브 생성. UAV write → vertex stream read 사이 배리어(세 버퍼 모두).
	FRHITransitionInfo ToUAV[3] = {
		FRHITransitionInfo(GpuPositionBuffer.VertexBufferRHI, ERHIAccess::Unknown, ERHIAccess::UAVCompute),
		FRHITransitionInfo(GpuTangentBuffer.VertexBufferRHI,  ERHIAccess::Unknown, ERHIAccess::UAVCompute),
		FRHITransitionInfo(GpuTexCoordBuffer.VertexBufferRHI, ERHIAccess::Unknown, ERHIAccess::UAVCompute),
	};
	RHICmdList.Transition(MakeArrayView(ToUAV, 3));

	if (bResident)
	{
		// 월드 PosBuf → component-local 변환. GT가 이번 프레임 GetComponentTransform()으로 만든 역행렬을
		// 쓴다(Data.WorldToLocal) — 여기서 GetLocalToWorld().Inverse()를 읽으면 안 된다: 이 커맨드는
		// UpdateAllPrimitiveSceneInfos(이번 프레임 트랜스폼 적용)보다 먼저 실행돼 한 프레임 이전 값이라,
		// 빌드(N-1)/드로우(N) 불일치로 월드 고정점(wrap 노드)이 컴포넌트 이동량만큼 떨린다.
		// GPU가 시뮬 노드(NumNodes)를 Subdiv로 Catmull-Rom 스무딩해 NumRings 센터라인 → 튜브 생성.
		RopeGPU::BuildTubeFromResident_RenderThread(RHICmdList, ResidentSRV,
			GpuPositionBuffer.UAV, GpuTangentBuffer.UAV, GpuTexCoordBuffer.UAV,
			NumRings, NumSides, Radius, NumNodes, Subdiv, Data.WorldToLocal);
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
	// (A) 모션블러 잔상 제거: 기본적으로 velocity를 출력하지 않아 per-object 모션블러 대상에서 제외한다.
	// r.DynamicRope.WriteVelocity 1로 레거시(velocity 출력) 동작과 A/B 비교 가능.
	Result.bVelocityRelevance = (CVarRopeWriteVelocity.GetValueOnRenderThread() != 0)
		&& DrawsVelocity() && Result.bOpaque && Result.bRenderInMainPass;
	return Result;
}
