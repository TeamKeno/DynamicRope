// Copyright Epic Games, Inc. All Rights Reserved.
//
// GPU tube generation and resident-centerline smoothing tests.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "RopeTubeBuilder.h"
// The complete definition of FRHIGPUBufferReadback, plus the wrapper covering version differences in RHI buffer creation.
#include "RHIGPUReadback.h"
#include "RopeRHICompat.h"
#include "RHI.h"
#include "RHICommandList.h"
#include "RenderingThread.h"
#include "Misc/App.h"

// The tube compute pass: whether the shader compiles and whether (1) the positions match the CPU parallel transport
// result, (2) the tangents, unpacked from signed normalized 16-bit, are unit vectors with the tangent X being the
// forward tangent and the tangent Z the radial normal, and (3) the UVs are the accumulated arc length over the
// circumference and the side index over the side count.
// The centreline is straight, with its forward direction along positive X, so the frame is constant, with U along
// positive Y and V along positive Z, which makes the expected values deterministic.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUTubeTangentUVTest,
	"DynamicRope.Solver.GPUTubeTangentUV",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUTubeTangentUVTest::RunTest(const FString& Parameters)
{
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("Skipping the GPU tube tangent and UV test: there is no renderable RHI, meaning this is headless."));
		return true;
	}

	const int32 NumRings = 4;
	const int32 NumSides = 6;
	const float Radius = 5.0f;
	const int32 VertsPerRing = NumSides + 1;
	const int32 NumVerts = NumRings * VertsPerRing;

	// A straight centreline along positive X. The forward tangent is positive X, U is the cross product of positive Z and positive X, giving positive Y, and V is the cross product of positive X and positive Y, giving positive Z.
	TArray<FVector3f> Centerline;
	Centerline.SetNum(NumRings);
	for (int32 i = 0; i < NumRings; ++i) { Centerline[i] = FVector3f(10.0f * i, 0, 0); }

	// The GPU dispatch and readback, on the render thread.
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

			// Uploading the input centreline.
			{
				float* Dst = static_cast<float*>(RHICmdList.LockBuffer(InBuf, 0, NumRings * 3 * sizeof(float), RLM_WriteOnly));
				for (int32 i = 0; i < NumRings; ++i) { Dst[i * 3 + 0] = Centerline[i].X; Dst[i * 3 + 1] = Centerline[i].Y; Dst[i * 3 + 2] = Centerline[i].Z; }
				RHICmdList.UnlockBuffer(InBuf);
			}

			RopeGPU::BuildTube_RenderThread(RHICmdList, InSRV, PosUAV, TanUAV, UVUAV, NumRings, NumSides, Radius);
			RHICmdList.SubmitAndBlockUntilGPUIdle();

			auto Read = [&](FBufferRHIRef Buf, uint32 Bytes, void* Dst)
			{
				FRHIGPUBufferReadback RB(TEXT("TubeTest.RB"));
				RB.EnqueueCopy(RHICmdList, Buf, Bytes);
				RHICmdList.SubmitAndBlockUntilGPUIdle();
				if (const void* Src = RB.Lock(Bytes)) { FMemory::Memcpy(Dst, Src, Bytes); RB.Unlock(); }
			};
			Read(PosBuf, NumVerts * 3 * sizeof(float),  OutPos.GetData());
			Read(TanBuf, NumVerts * 4 * sizeof(uint32), OutTan.GetData());
			Read(UVBuf,  NumVerts * 2 * sizeof(float),  OutUV.GetData());
		});
	FlushRenderingCommands();

	// Unpacking the signed normalized 16-bit values.
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
			// The forward tangent, positive X.
			MaxTxDev = FMath::Max(MaxTxDev, (TX - FVector3f(1, 0, 0)).Size());
			// The normal, which is radial.
			MaxTanLenDev = FMath::Max(MaxTanLenDev, (TZ - Radial).Size());

			// The U coordinate is the accumulated arc length, which at a straight 10 cm spacing is ten per ring, over the circumference.
			const float ExpU = (10.0f * static_cast<float>(ring)) / (2.0f * PI * Radius);
			const float ExpV = static_cast<float>(side) / static_cast<float>(NumSides);
			MaxUVDev = FMath::Max(MaxUVDev, FMath::Abs(OutUV[v * 2 + 0] - ExpU));
			MaxUVDev = FMath::Max(MaxUVDev, FMath::Abs(OutUV[v * 2 + 1] - ExpV));
		}
	}

	AddInfo(FString::Printf(TEXT("pos dev %.4f, tangentX dev %.4f, normal/len dev %.4f, UV dev %.5f"),
		MaxPosDev, MaxTxDev, MaxTanLenDev, MaxUVDev));
	TestTrue(TEXT("the positions match the CPU parallel transport"), MaxPosDev < 0.01f);
	TestTrue(TEXT("the tangent X is the forward tangent, positive X"), MaxTxDev < 0.01f);
	TestTrue(TEXT("the tangent Z is the radial normal and is a unit vector"), MaxTanLenDev < 0.01f);
	TestTrue(TEXT("the UV is the accumulated arc length over the circumference and the side index over the side count"), MaxUVDev < 0.001f);
	return true;
}

// GPU tube smoothing: whether smoothing the simulation nodes of a resident entry with Catmull-Rom on the GPU matches
// the tube built from a non-resident entry after smoothing on the CPU, compared by vertex position. The frame and
// vertex expressions are shared, so any deviation reflects the accuracy of the smoothing port alone. Curved nodes
// make this a case where the smoothing actually does something.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUTubeSmoothingTest,
	"DynamicRope.Solver.GPUTubeSmoothing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUTubeSmoothingTest::RunTest(const FString& Parameters)
{
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("Skipping the GPU tube smoothing test: there is no renderable RHI, meaning this is headless."));
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

	// Curved simulation nodes, with world space equal to local space and an identity world-to-local transform. The zigzag gives it curvature.
	TArray<FVector3f> Nodes;
	Nodes.Add(FVector3f(0, 0, 0));
	Nodes.Add(FVector3f(10, 8, 0));
	Nodes.Add(FVector3f(20, -6, 4));
	Nodes.Add(FVector3f(30, 5, -3));
	Nodes.Add(FVector3f(40, 0, 6));

	// CPU smoothing, in FRopeSceneProxy::BuildSmoothedCenterline, mirrored by RopeCatmullSmooth on the GPU, with a centripetal alpha.
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

	// A is the non-resident entry with CPU smoothing and B is the resident entry with GPU smoothing.
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

			// The input to A: the CPU-smoothed centreline, in component-local space.
			{
				float* Dst = static_cast<float*>(RHICmdList.LockBuffer(InBuf, 0, NumRings * 3 * sizeof(float), RLM_WriteOnly));
				for (int32 i = 0; i < NumRings; ++i) { Dst[i*3+0]=Smoothed[i].X; Dst[i*3+1]=Smoothed[i].Y; Dst[i*3+2]=Smoothed[i].Z; }
				RHICmdList.UnlockBuffer(InBuf);
			}
			// The input to B: the resident simulation nodes, with world space equal to local space, as float4.
			{
				FVector4f* Dst = static_cast<FVector4f*>(RHICmdList.LockBuffer(ResBuf, 0, NumNodes * sizeof(FVector4f), RLM_WriteOnly));
				for (int32 i = 0; i < NumNodes; ++i) { Dst[i] = FVector4f(Nodes[i].X, Nodes[i].Y, Nodes[i].Z, 0.0f); }
				RHICmdList.UnlockBuffer(ResBuf);
			}

			RopeGPU::BuildTube_RenderThread(RHICmdList, InSRV, PUAVa, TUAVa, UUAVa, NumRings, NumSides, Radius);
			RopeGPU::BuildTubeFromResident_RenderThread(RHICmdList, ResSRV, PUAVb, TUAVb, UUAVb,
				NumRings, NumSides, Radius, NumNodes, Subdiv, SmoothParam, FMatrix44f::Identity);
			RHICmdList.SubmitAndBlockUntilGPUIdle();

			auto Read = [&](FBufferRHIRef Buf, uint32 Bytes, void* Dst)
			{
				FRHIGPUBufferReadback RB(TEXT("Sm.RB"));
				RB.EnqueueCopy(RHICmdList, Buf, Bytes);
				RHICmdList.SubmitAndBlockUntilGPUIdle();
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
	AddInfo(FString::Printf(TEXT("The maximum vertex deviation between GPU and CPU smoothing is %.5f cm"), MaxDev));
	TestTrue(FString::Printf(TEXT("the GPU Catmull-Rom smoothing matches the CPU, with a deviation of %.5f"), MaxDev), MaxDev < 0.05f);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS

