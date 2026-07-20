// Copyright Epic Games, Inc. All Rights Reserved.
//
// GPU tube generation and resident-centerline smoothing tests.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "RopeTubeBuilder.h"
// FRHIGPUBufferReadback 완전정의 + RHI 버퍼 생성 버전차 래퍼
#include "RHIGPUReadback.h"
#include "RopeRHICompat.h"
#include "RHI.h"
#include "RHIGPUReadback.h"
#include "RHICommandList.h"
#include "RenderingThread.h"
#include "Misc/App.h"

// B2-full 튜브 컴퓨트: 셰이더가 컴파일되고 (1) 위치가 CPU parallel-transport 결과와 일치, (2) tangent(SNORM16
// 언팩)가 단위이며 TangentX=전방접선/TangentZ=radial, (3) UV가 (누적 호길이/원주, side/NumSides)인지 검증.
// 직선 센터라인(전방=+X)이라 프레임이 상수(U=+Y, V=+Z)여서 기대값이 결정적이다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUTubeTangentUVTest,
	"DynamicRope.Solver.GPUTubeTangentUV",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUTubeTangentUVTest::RunTest(const FString& Parameters)
{
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU 튜브 tangent/UV 테스트 스킵: 렌더 가능한 RHI가 없음(헤드리스)."));
		return true;
	}

	const int32 NumRings = 4;
	const int32 NumSides = 6;
	const float Radius = 5.0f;
	const int32 VertsPerRing = NumSides + 1;
	const int32 NumVerts = NumRings * VertsPerRing;

	// 직선 센터라인(+X). 전방접선=+X, U=cross(+Z,+X)=+Y, V=cross(+X,+Y)=+Z.
	TArray<FVector3f> Centerline;
	Centerline.SetNum(NumRings);
	for (int32 i = 0; i < NumRings; ++i) { Centerline[i] = FVector3f(10.0f * i, 0, 0); }

	// GPU 디스패치 + 리드백(렌더 스레드).
	TArray<float> OutPos, OutUV;
	TArray<uint32> OutTan;
	OutPos.SetNumZeroed(NumVerts * 3);
	OutTan.SetNumZeroed(NumVerts * 4);
	OutUV.SetNumZeroed(NumVerts * 2);

	ENQUEUE_RENDER_COMMAND(RopeTubeTest)(
		[&](FRHICommandListImmediate& RHICmdList)
		{
			auto MakeBuf = [&](const TCHAR* Name, uint32 Bytes, EPixelFormat Fmt, bool bUAV,
				FShaderResourceViewRHIRef& OutSRV, FUnorderedAccessViewRHIRef& OutUAV) -> FBufferRHIRef
			{
				FBufferRHIRef Buf = RopeRHI::CreateVertexBuffer(RHICmdList, Name, Bytes,
					EBufferUsageFlags::ShaderResource | (bUAV ? EBufferUsageFlags::UnorderedAccess : EBufferUsageFlags::None));
				OutSRV = RHICmdList.CreateShaderResourceView(Buf,
					FRHIViewDesc::CreateBufferSRV().SetType(FRHIViewDesc::EBufferType::Typed).SetFormat(Fmt));
				if (bUAV)
				{
					OutUAV = RHICmdList.CreateUnorderedAccessView(Buf,
						FRHIViewDesc::CreateBufferUAV().SetType(FRHIViewDesc::EBufferType::Typed).SetFormat(Fmt));
				}
				return Buf;
			};

			FShaderResourceViewRHIRef InSRV, PosSRV, TanSRV, UVSRV;
			FUnorderedAccessViewRHIRef DummyUAV, PosUAV, TanUAV, UVUAV;
			FBufferRHIRef InBuf  = MakeBuf(TEXT("TubeTest.In"),  NumRings * 3 * sizeof(float), PF_R32_FLOAT, false, InSRV, DummyUAV);
			FBufferRHIRef PosBuf = MakeBuf(TEXT("TubeTest.Pos"), NumVerts * 3 * sizeof(float), PF_R32_FLOAT, true,  PosSRV, PosUAV);
			FBufferRHIRef TanBuf = MakeBuf(TEXT("TubeTest.Tan"), NumVerts * 4 * sizeof(uint32), PF_R32_UINT,  true,  TanSRV, TanUAV);
			FBufferRHIRef UVBuf  = MakeBuf(TEXT("TubeTest.UV"),  NumVerts * 2 * sizeof(float), PF_R32_FLOAT, true,  UVSRV, UVUAV);

			// 입력 센터라인 업로드.
			{
				float* Dst = static_cast<float*>(RHICmdList.LockBuffer(InBuf, 0, NumRings * 3 * sizeof(float), RLM_WriteOnly));
				for (int32 i = 0; i < NumRings; ++i) { Dst[i * 3 + 0] = Centerline[i].X; Dst[i * 3 + 1] = Centerline[i].Y; Dst[i * 3 + 2] = Centerline[i].Z; }
				RHICmdList.UnlockBuffer(InBuf);
			}

			RopeGPU::BuildTube_RenderThread(RHICmdList, InSRV, PosUAV, TanUAV, UVUAV, NumRings, NumSides, Radius);
			RHICmdList.BlockUntilGPUIdle();

			auto Read = [&](FBufferRHIRef Buf, uint32 Bytes, void* Dst)
			{
				FRHIGPUBufferReadback RB(TEXT("TubeTest.RB"));
				RB.EnqueueCopy(RHICmdList, Buf, Bytes);
				RHICmdList.BlockUntilGPUIdle();
				if (const void* Src = RB.Lock(Bytes)) { FMemory::Memcpy(Dst, Src, Bytes); RB.Unlock(); }
			};
			Read(PosBuf, NumVerts * 3 * sizeof(float),  OutPos.GetData());
			Read(TanBuf, NumVerts * 4 * sizeof(uint32), OutTan.GetData());
			Read(UVBuf,  NumVerts * 2 * sizeof(float),  OutUV.GetData());
		});
	FlushRenderingCommands();

	// SNORM16 언팩.
	auto Snorm = [](uint32 packed, int half) -> float
	{
		const int16 s = static_cast<int16>((packed >> (half * 16)) & 0xFFFF);
		return FMath::Clamp(static_cast<float>(s) / 32767.0f, -1.0f, 1.0f);
	};

	float MaxPosDev = 0.0f, MaxTanLenDev = 0.0f, MaxTxDev = 0.0f, MaxUVDev = 0.0f;
	for (int32 ring = 0; ring < NumRings; ++ring)
	{
		for (int32 side = 0; side < VertsPerRing; ++side)
		{
			const int32 v = ring * VertsPerRing + side;
			const float Angle = 2.0f * PI * static_cast<float>(side) / static_cast<float>(NumSides);
			// U=+Y, V=+Z
			const FVector3f Radial(0.0f, FMath::Cos(Angle), FMath::Sin(Angle));
			const FVector3f ExpPos = Centerline[ring] + Radial * Radius;
			const FVector3f GpuPos(OutPos[v * 3 + 0], OutPos[v * 3 + 1], OutPos[v * 3 + 2]);
			MaxPosDev = FMath::Max(MaxPosDev, (GpuPos - ExpPos).Size());

			// TangentX = uint[0..1], TangentZ = uint[2..3].
			const FVector3f TX(Snorm(OutTan[v * 4 + 0], 0), Snorm(OutTan[v * 4 + 0], 1), Snorm(OutTan[v * 4 + 1], 0));
			const FVector3f TZ(Snorm(OutTan[v * 4 + 2], 0), Snorm(OutTan[v * 4 + 2], 1), Snorm(OutTan[v * 4 + 3], 0));
			MaxTanLenDev = FMath::Max(MaxTanLenDev, FMath::Abs(TZ.Size() - 1.0f));
			// 전방접선 +X
			MaxTxDev = FMath::Max(MaxTxDev, (TX - FVector3f(1, 0, 0)).Size());
			// 법선 = radial
			MaxTanLenDev = FMath::Max(MaxTanLenDev, (TZ - Radial).Size());

			// UV.x = 누적 호길이(직선 10cm 간격이라 10*ring) / 원주(2πR).
			const float ExpU = (10.0f * static_cast<float>(ring)) / (2.0f * PI * Radius);
			const float ExpV = static_cast<float>(side) / static_cast<float>(NumSides);
			MaxUVDev = FMath::Max(MaxUVDev, FMath::Abs(OutUV[v * 2 + 0] - ExpU));
			MaxUVDev = FMath::Max(MaxUVDev, FMath::Abs(OutUV[v * 2 + 1] - ExpV));
		}
	}

	AddInfo(FString::Printf(TEXT("pos dev %.4f, tangentX dev %.4f, normal/len dev %.4f, UV dev %.5f"),
		MaxPosDev, MaxTxDev, MaxTanLenDev, MaxUVDev));
	TestTrue(TEXT("위치가 CPU parallel-transport와 일치"), MaxPosDev < 0.01f);
	TestTrue(TEXT("TangentX = 전방접선(+X)"), MaxTxDev < 0.01f);
	TestTrue(TEXT("TangentZ = radial 법선(단위)"), MaxTanLenDev < 0.01f);
	TestTrue(TEXT("UV = (누적 호길이/원주, side/NumSides)"), MaxUVDev < 0.001f);
	return true;
}

// GPU 튜브 스무딩(B2-full): resident 엔트리가 시뮬 노드(NumNodes)를 GPU에서 Catmull-Rom 스무딩한 결과가
// CPU 스무딩 후 B1 엔트리로 만든 튜브와 일치하는가(정점 위치 비교). 프레임/정점 수식은 공유하므로 편차는
// 스무딩 포팅의 정확도만 반영한다. 곡선 노드로 스무딩이 실제로 작동하는 케이스.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUTubeSmoothingTest,
	"DynamicRope.Solver.GPUTubeSmoothing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUTubeSmoothingTest::RunTest(const FString& Parameters)
{
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU 튜브 스무딩 테스트 스킵: 렌더 가능한 RHI가 없음(헤드리스)."));
		return true;
	}

	const int32 NumNodes = 5;
	const int32 Subdiv = 3;
	// 13
	const int32 NumRings = (NumNodes - 1) * Subdiv + 1;
	const int32 NumSides = 6;
	const float Radius = 4.0f;
	const int32 VertsPerRing = NumSides + 1;
	const int32 NumVerts = NumRings * VertsPerRing;

	// 곡선 시뮬 노드(월드=로컬, WorldToLocal=identity). 지그재그로 곡률을 준다.
	TArray<FVector3f> Nodes;
	Nodes.Add(FVector3f(0, 0, 0));
	Nodes.Add(FVector3f(10, 8, 0));
	Nodes.Add(FVector3f(20, -6, 4));
	Nodes.Add(FVector3f(30, 5, -3));
	Nodes.Add(FVector3f(40, 0, 6));

	// CPU 스무딩(FRopeSceneProxy::BuildSmoothedCenterline / GPU RopeCatmullSmooth 미러). α=centripetal.
	const float SmoothParam = 0.5f;
	TArray<FVector3f> Smoothed;
	Smoothed.SetNum(NumRings);
	{
		const int32 LastNode = NumNodes - 1;
		for (int32 r = 0; r < NumRings; ++r)
		{
			const int32 Seg = r / Subdiv;
			if (Seg >= LastNode) { Smoothed[r] = Nodes[LastNode]; continue; }
			const float T = static_cast<float>(r % Subdiv) / static_cast<float>(Subdiv);
			const FVector3f P0 = Nodes[FMath::Max(Seg - 1, 0)];
			const FVector3f P1 = Nodes[Seg];
			const FVector3f P2 = Nodes[Seg + 1];
			const FVector3f P3 = Nodes[FMath::Min(Seg + 2, LastNode)];
			const float EPS = 1e-4f;
			const float t01 = FMath::Pow(FMath::Max((P1 - P0).Size(), EPS), SmoothParam);
			const float t12 = FMath::Pow(FMath::Max((P2 - P1).Size(), EPS), SmoothParam);
			const float t23 = FMath::Pow(FMath::Max((P3 - P2).Size(), EPS), SmoothParam);
			const FVector3f M1 = (P2 - P1) + t12 * ((P1 - P0) / t01 - (P2 - P0) / (t01 + t12));
			const FVector3f M2 = (P2 - P1) + t12 * ((P3 - P2) / t23 - (P3 - P1) / (t12 + t23));
			const FVector3f A =  2.0f * (P1 - P2) + M1 + M2;
			const FVector3f B = -3.0f * (P1 - P2) - 2.0f * M1 - M2;
			Smoothed[r] = ((A * T + B) * T + M1) * T + P1;
		}
	}

	// A=B1(CPU 스무딩), B=resident(GPU 스무딩)
	TArray<float> PosA, PosB;
	PosA.SetNumZeroed(NumVerts * 3);
	PosB.SetNumZeroed(NumVerts * 3);

	ENQUEUE_RENDER_COMMAND(RopeTubeSmoothTest)(
		[&](FRHICommandListImmediate& RHICmdList)
		{
			auto MakeTyped = [&](const TCHAR* Name, uint32 Bytes, EPixelFormat Fmt, bool bUAV,
				FShaderResourceViewRHIRef& OutSRV, FUnorderedAccessViewRHIRef& OutUAV) -> FBufferRHIRef
			{
				FBufferRHIRef Buf = RopeRHI::CreateVertexBuffer(RHICmdList, Name, Bytes,
					EBufferUsageFlags::ShaderResource | (bUAV ? EBufferUsageFlags::UnorderedAccess : EBufferUsageFlags::None));
				OutSRV = RHICmdList.CreateShaderResourceView(Buf,
					FRHIViewDesc::CreateBufferSRV().SetType(FRHIViewDesc::EBufferType::Typed).SetFormat(Fmt));
				if (bUAV) { OutUAV = RHICmdList.CreateUnorderedAccessView(Buf,
					FRHIViewDesc::CreateBufferUAV().SetType(FRHIViewDesc::EBufferType::Typed).SetFormat(Fmt)); }
				return Buf;
			};
			auto MakeStructured = [&](const TCHAR* Name, uint32 Stride, uint32 Count, FShaderResourceViewRHIRef& OutSRV) -> FBufferRHIRef
			{
				FBufferRHIRef Buf = RopeRHI::CreateStructuredBuffer(RHICmdList, Name, Stride, Count,
					EBufferUsageFlags::ShaderResource);
				OutSRV = RHICmdList.CreateShaderResourceView(Buf,
					FRHIViewDesc::CreateBufferSRV().SetType(FRHIViewDesc::EBufferType::Structured));
				return Buf;
			};

			FShaderResourceViewRHIRef InSRV, ResSRV, PSRVa, TSRVa, USRVa, PSRVb, TSRVb, USRVb;
			FUnorderedAccessViewRHIRef DummyUAV, PUAVa, TUAVa, UUAVa, PUAVb, TUAVb, UUAVb;
			FBufferRHIRef InBuf  = MakeTyped(TEXT("Sm.In"),  NumRings * 3 * sizeof(float), PF_R32_FLOAT, false, InSRV, DummyUAV);
			FBufferRHIRef ResBuf = MakeStructured(TEXT("Sm.Res"), sizeof(FVector4f), NumNodes, ResSRV);
			FBufferRHIRef PBufA  = MakeTyped(TEXT("Sm.PA"), NumVerts * 3 * sizeof(float), PF_R32_FLOAT, true, PSRVa, PUAVa);
			FBufferRHIRef TBufA  = MakeTyped(TEXT("Sm.TA"), NumVerts * 4 * sizeof(uint32), PF_R32_UINT, true, TSRVa, TUAVa);
			FBufferRHIRef UBufA  = MakeTyped(TEXT("Sm.UA"), NumVerts * 2 * sizeof(float), PF_R32_FLOAT, true, USRVa, UUAVa);
			FBufferRHIRef PBufB  = MakeTyped(TEXT("Sm.PB"), NumVerts * 3 * sizeof(float), PF_R32_FLOAT, true, PSRVb, PUAVb);
			FBufferRHIRef TBufB  = MakeTyped(TEXT("Sm.TB"), NumVerts * 4 * sizeof(uint32), PF_R32_UINT, true, TSRVb, TUAVb);
			FBufferRHIRef UBufB  = MakeTyped(TEXT("Sm.UB"), NumVerts * 2 * sizeof(float), PF_R32_FLOAT, true, USRVb, UUAVb);

			// A 입력: CPU 스무딩 센터라인(component-local).
			{
				float* Dst = static_cast<float*>(RHICmdList.LockBuffer(InBuf, 0, NumRings * 3 * sizeof(float), RLM_WriteOnly));
				for (int32 i = 0; i < NumRings; ++i) { Dst[i*3+0]=Smoothed[i].X; Dst[i*3+1]=Smoothed[i].Y; Dst[i*3+2]=Smoothed[i].Z; }
				RHICmdList.UnlockBuffer(InBuf);
			}
			// B 입력: resident 시뮬 노드(월드=로컬, float4).
			{
				FVector4f* Dst = static_cast<FVector4f*>(RHICmdList.LockBuffer(ResBuf, 0, NumNodes * sizeof(FVector4f), RLM_WriteOnly));
				for (int32 i = 0; i < NumNodes; ++i) { Dst[i] = FVector4f(Nodes[i].X, Nodes[i].Y, Nodes[i].Z, 0.0f); }
				RHICmdList.UnlockBuffer(ResBuf);
			}

			RopeGPU::BuildTube_RenderThread(RHICmdList, InSRV, PUAVa, TUAVa, UUAVa, NumRings, NumSides, Radius);
			RopeGPU::BuildTubeFromResident_RenderThread(RHICmdList, ResSRV, PUAVb, TUAVb, UUAVb,
				NumRings, NumSides, Radius, NumNodes, Subdiv, SmoothParam, FMatrix44f::Identity);
			RHICmdList.BlockUntilGPUIdle();

			auto Read = [&](FBufferRHIRef Buf, uint32 Bytes, void* Dst)
			{
				FRHIGPUBufferReadback RB(TEXT("Sm.RB"));
				RB.EnqueueCopy(RHICmdList, Buf, Bytes);
				RHICmdList.BlockUntilGPUIdle();
				if (const void* Src = RB.Lock(Bytes)) { FMemory::Memcpy(Dst, Src, Bytes); RB.Unlock(); }
			};
			Read(PBufA, NumVerts * 3 * sizeof(float), PosA.GetData());
			Read(PBufB, NumVerts * 3 * sizeof(float), PosB.GetData());
		});
	FlushRenderingCommands();

	float MaxDev = 0.0f;
	for (int32 v = 0; v < NumVerts; ++v)
	{
		const FVector3f A(PosA[v*3+0], PosA[v*3+1], PosA[v*3+2]);
		const FVector3f B(PosB[v*3+0], PosB[v*3+1], PosB[v*3+2]);
		MaxDev = FMath::Max(MaxDev, (A - B).Size());
	}
	AddInfo(FString::Printf(TEXT("GPU-스무딩 vs CPU-스무딩 최대 정점 편차 %.5f cm"), MaxDev));
	TestTrue(FString::Printf(TEXT("GPU Catmull-Rom 스무딩이 CPU와 일치(편차 %.5f)"), MaxDev), MaxDev < 0.05f);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS

