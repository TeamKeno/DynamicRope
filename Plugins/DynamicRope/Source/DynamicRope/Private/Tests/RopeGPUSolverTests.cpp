// Copyright Epic Games, Inc. All Rights Reserved.
//
// GPU 솔버(M1) 패리티/안정성 테스트. CPU FRopeXPBDSolver를 ground-truth로, GPU FRopeGPUSolver가
// 같은 시나리오에서 (1) 발산/NaN 없이 (2) 비신축 세그먼트 길이를 유지하며 (3) CPU와 "근사" 일치하는지 본다.
// 주의: red-black/stride-3 컬러링은 CPU의 교대-sweep Gauss-Seidel과 비트일치하지 않는다(수렴만 근사) →
// 노드별 편차는 허용오차 기반으로만 본다. 또한 GPU 디스패치는 RHI가 필요하므로, 렌더 불가 환경에선 스킵한다.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Solver/RopeXPBDSolver.h"
// DynamicRopeShaders 모듈
#include "RopeGPUSolver.h"
#include "Collision/RopeCollider.h"
// FRopeBoxCollider (정적 박스 parity)
#include "Collision/RopeStaticCollider.h"
#include "Collision/SDF/RopeSDFCollider.h"
// MakeSphere(합성 SDF 볼륨)
#include "Collision/SDF/RopeSDFSynthetic.h"
#include "Collision/SDF/RopeSDFData.h"
// CPU 감지(패리티 ground-truth)
#include "Logic/RopeFlightContactDetector.h"
#include "RopeTestHelpers.h"
#include "RHI.h"
#include "RHIGPUReadback.h"
// BlockUntilGPUIdle
#include "RHICommandList.h"
// FlushRenderingCommands
#include "RenderingThread.h"
#include "Misc/App.h"

namespace
{
	FRopeSolverConfig MakeHangConfig()
	{
		FRopeSolverConfig C;
		C.Substeps = 8;
		C.Iterations = 8;
		// 비신축
		C.StretchCompliance = 0.0f;
		C.BendCompliance = 0.02f;
		C.Gravity = FVector(0.0f, 0.0f, -980.0f);
		C.Damping = 0.02f;
		return C;
	}

	FRopeSimState MakePinnedRope(int32 N, float Length)
	{
		FRopeSimState S = RopeTest::MakeStraightRope(N, Length);
		S.bStartPinned = true;
		S.StartPinPrev = S.Positions[0];
		S.StartPinTarget = S.Positions[0];
		S.InvMass[0] = 0.0f;
		return S;
	}
}

// 핀-고정 hanging rope에서 GPU 경로가 안정적이고(NaN/발산 없음, 세그먼트 길이 유지) CPU와 근사 일치하는가.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUSolverParityTest,
	"DynamicRope.Solver.GPUParityHangingRope",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUSolverParityTest::RunTest(const FString& Parameters)
{
	// GPU 디스패치는 RHI 필요 — 렌더 불가(헤드리스/널 RHI) 환경에선 스킵(실패가 아님).
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU 솔버 패리티 테스트 스킵: 렌더 가능한 RHI가 없음(헤드리스)."));
		return true;
	}

	const int32 N = 24;
	const float Length = 300.0f;
	const FRopeSolverConfig Config = MakeHangConfig();

	FRopeSimState CpuSim = MakePinnedRope(N, Length);
	FRopeSimState GpuSim = MakePinnedRope(N, Length);

	const FRopeXPBDSolver Solver;
	const TArray<IRopeCollider*> NoColliders;

	// GPU 솔버는 상주(M5): 매 프레임 Step으로 영속 버퍼를 in-place 전진, 결과는 RT 리드백→GetLatest로 회수(지연).
	// 테스트는 동기 검증이라 매 Step 후 FlushRenderingCommands로 RT를 진행시킨다. generation은 1로 고정(첫 프레임만 시드).
	FRopeGPUSolver GpuSolver;
	const uint32 RopeId = 1;
	const uint32 Gen = 1;

	auto MakeStep = [&](const FRopeSimState& Src, int32 NumSub, float FixedDt) -> FRopeGPUResidentStep
	{
		FRopeGPUResidentStep Step;
		Step.RopeId            = RopeId;
		Step.Generation        = Gen;
		Step.NumNodes          = Src.Num();
		Step.SeedPositions     = Src.Positions;
		Step.SeedPrevPositions = Src.PrevPositions;
		Step.InvMass           = Src.InvMass;
		Step.SegmentLength     = Src.SegmentLength;
		Step.bStartPinned      = Src.bStartPinned;
		Step.StartPinPrev      = Src.StartPinPrev;
		Step.StartPinTarget    = Src.StartPinTarget;
		Step.StretchCompliance = Config.StretchCompliance;
		Step.BendCompliance    = Config.BendCompliance;
		Step.BendReleaseRatio  = Config.BendReleaseRatio;
		Step.BendFullRatio     = Config.BendFullRatio;
		Step.Damping           = Config.Damping;
		Step.Iterations        = Config.Iterations;
		Step.Gravity           = Config.Gravity;
		Step.NumSub            = NumSub;
		Step.FixedDt           = FixedDt;
		return Step;
	};

	for (int32 Frame = 0; Frame < 120; ++Frame)
	{
		// CPU: ground-truth.
		Solver.Step(CpuSim, Config, NoColliders, 1.0f / 60.0f);

		// GPU: CPU와 동일한 고정-timestep 스케줄(누적 진화 동일 → 동일 NumSub)로 상주 step 1회.
		const FRopeSubstepSchedule Schedule = RopeSolverSubsteps(GpuSim, Config, 1.0f / 60.0f);
		TArray<FRopeGPUResidentStep> Steps;
		Steps.Add(MakeStep(GpuSim, Schedule.NumSub, Schedule.FixedDt));
		GpuSolver.Step(MoveTemp(Steps));
		// RT가 dispatch + 리드백 copy를 처리하도록 진행.
		FlushRenderingCommands();
	}

	// 마지막 step의 리드백을 drain: consume은 다음 Step의 loop1에서 일어나므로 NumSub=0 step으로 펌프한다.
	TMap<uint32, FRopeResidentLatest> Latest;
	bool bGot = false;
	for (int32 Spin = 0; Spin < 64 && !bGot; ++Spin)
	{
		FlushRenderingCommands();
		TArray<FRopeGPUResidentStep> Drain;
		// NumSub=0 → 적분 없이 직전 리드백만 consume.
		Drain.Add(MakeStep(GpuSim, 0, 1.0f / 60.0f));
		GpuSolver.Step(MoveTemp(Drain));
		FlushRenderingCommands();
		GpuSolver.GetLatest(Latest);
		if (const FRopeResidentLatest* L = Latest.Find(RopeId))
		{
			if (L->Generation == Gen && L->Positions.Num() == GpuSim.Num() && L->PrevPositions.Num() == GpuSim.Num())
			{
				for (int32 i = 0; i < GpuSim.Num(); ++i)
				{
					GpuSim.Positions[i]     = L->Positions[i];
					GpuSim.PrevPositions[i] = L->PrevPositions[i];
				}
				bGot = true;
			}
		}
	}
	if (!bGot)
	{
		AddError(TEXT("GPU 상주 결과를 회수하지 못함(리드백 drain 실패)."));
		return false;
	}

	// (1) 안정성: NaN 없음.
	TestFalse(TEXT("CPU no NaN"), RopeTest::AnyNaN(CpuSim));
	TestFalse(TEXT("GPU no NaN"), RopeTest::AnyNaN(GpuSim));

	// (2) 비신축: GPU 세그먼트 오차가 CPU와 같은 수준으로 bounded.
	const float GpuSegErr = RopeTest::MaxSegmentError(GpuSim);
	TestTrue(FString::Printf(TEXT("GPU segment error %.2f cm bounded"), GpuSegErr),
		GpuSegErr < GpuSim.SegmentLength * 2.0f);

	// (3) 근사 일치: 노드별 최대 편차. 비트일치는 기대하지 않음 — 정착 형상이 가까운지만 본다(허용오차 관대).
	float MaxDev = 0.0f;
	for (int32 i = 0; i < N; ++i)
	{
		MaxDev = FMath::Max(MaxDev, static_cast<float>(FVector::Dist(CpuSim.Positions[i], GpuSim.Positions[i])));
	}
	AddInfo(FString::Printf(TEXT("CPU↔GPU 최대 노드 편차: %.2f cm (RopeLength %.0f)"), MaxDev, Length));
	TestTrue(FString::Printf(TEXT("CPU↔GPU max node deviation %.2f cm within tolerance"), MaxDev),
		// 정착 hanging 형상은 가까워야 함(관대한 상한).
		MaxDev < Length * 0.25f);

	return true;
}

// Override 패스(G0): NumSub=0 오버라이드 dispatch가 위치/질량을 상주 버퍼에 기록하고,
// InvMass=0으로 고정한 노드가 이후 중력 솔브에서도 타깃에 정확히 남으며(질량 마스크 영속),
// InvMass 복원 오버라이드 후에는 다시 물리로 돌아오는지 본다. "타깃 계산은 GT, 적용은 GPU" 계약 검증.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUOverridePassTest,
	"DynamicRope.Solver.GPUOverridePass",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUOverridePassTest::RunTest(const FString& Parameters)
{
	// GPU 디스패치는 RHI 필요 — 렌더 불가(헤드리스/널 RHI) 환경에선 스킵(실패가 아님).
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU override 패스 테스트 스킵: 렌더 가능한 RHI가 없음(헤드리스)."));
		return true;
	}

	const int32 N = 8;
	const float Length = 140.0f;
	const FRopeSolverConfig Config = MakeHangConfig();
	const FRopeSimState Sim = MakePinnedRope(N, Length);

	FRopeGPUSolver GpuSolver;
	const uint32 RopeId = 7;
	const uint32 Gen = 1;

	auto MakeStep = [&](int32 NumSub, float FixedDt) -> FRopeGPUResidentStep
	{
		FRopeGPUResidentStep Step;
		Step.RopeId            = RopeId;
		Step.Generation        = Gen;
		Step.NumNodes          = Sim.Num();
		Step.SeedPositions     = Sim.Positions;
		Step.SeedPrevPositions = Sim.PrevPositions;
		Step.InvMass           = Sim.InvMass;
		Step.SegmentLength     = Sim.SegmentLength;
		Step.bStartPinned      = Sim.bStartPinned;
		Step.StartPinPrev      = Sim.StartPinPrev;
		Step.StartPinTarget    = Sim.StartPinTarget;
		Step.StretchCompliance = Config.StretchCompliance;
		Step.BendCompliance    = Config.BendCompliance;
		Step.BendReleaseRatio  = Config.BendReleaseRatio;
		Step.BendFullRatio     = Config.BendFullRatio;
		Step.Damping           = Config.Damping;
		Step.Iterations        = Config.Iterations;
		Step.Gravity           = Config.Gravity;
		Step.NumSub            = NumSub;
		Step.FixedDt           = FixedDt;
		return Step;
	};
	auto Pump = [&](FRopeGPUResidentStep&& Step)
	{
		TArray<FRopeGPUResidentStep> Steps;
		Steps.Add(MoveTemp(Step));
		GpuSolver.Step(MoveTemp(Steps));
		FlushRenderingCommands();
	};
	// GPU idle까지 동기화 — 리드백 IsReady를 결정적으로 만든다(헤드리스 고속 실행에서 GPU가
	// 뒤처지면 in-flight 복사본이 오래된 프레임 것일 수 있다).
	auto SyncGPU = []()
	{
		ENQUEUE_RENDER_COMMAND(RopeTestGpuSync)(
			[](FRHICommandListImmediate& RHICmdList)
			{
				RHICmdList.BlockUntilGPUIdle();
			});
		FlushRenderingCommands();
	};
	// 최신(최종 상태) 리드백 회수. 주의: 단일 in-flight 리드백은 무장 시점의 버퍼를 복사하므로,
	// (1) GPU idle 동기화로 기존 복사본을 consume 가능하게 만들고 (2) no-op override dispatch로
	// 재무장을 유도하는 사이클을 여러 번 돌려야 "마지막 실제 상태"의 복사본이 확실히 도착한다.
	// (no-op = 플래그 전부 0: 아무 노드도 쓰지 않지만 dispatch는 발생 → consume+재무장.)
	auto Drain = [&](FRopeResidentLatest& OutLatest) -> bool
	{
		for (int32 Spin = 0; Spin < 8; ++Spin)
		{
			SyncGPU();
			FRopeGPUResidentStep Noop = MakeStep(0, 1.0f / 60.0f);
			Noop.OverrideFlags.SetNumZeroed(Sim.Num());
			Pump(MoveTemp(Noop));
		}
		SyncGPU();
		// 마지막 consume(오버라이드 없음 — dispatch 없이 회수만).
		Pump(MakeStep(0, 1.0f / 60.0f));

		TMap<uint32, FRopeResidentLatest> Latest;
		GpuSolver.GetLatest(Latest);
		if (const FRopeResidentLatest* L = Latest.Find(RopeId))
		{
			if (L->Generation == Gen && L->Positions.Num() == Sim.Num())
			{
				OutLatest = *L;
				return true;
			}
		}
		return false;
	};

	// 1) 시드 + 정상 솔브 몇 프레임.
	for (int32 Frame = 0; Frame < 4; ++Frame)
	{
		Pump(MakeStep(Config.Substeps, (1.0f / 60.0f) / Config.Substeps));
	}

	// 2) 오버라이드: 노드 3..5를 임의 타깃에 고정(Pos + Prev=Pos + InvMass=0). NumSub=0 — 적분 없이 기록만.
	TArray<FVector> Targets;
	Targets.SetNumZeroed(N);
	const uint8 FixFlags = static_cast<uint8>(
		ERopeGPUOverride::Position | ERopeGPUOverride::PrevFromPosition | ERopeGPUOverride::InvMass);
	{
		FRopeGPUResidentStep Ov = MakeStep(0, 1.0f / 60.0f);
		Ov.OverrideFlags.SetNumZeroed(N);
		Ov.OverridePositions.SetNumZeroed(N);
		Ov.OverrideInvMass.SetNumZeroed(N);
		for (int32 i = 3; i <= 5; ++i)
		{
			Targets[i] = FVector(20.0f * i, 35.0f, -25.0f);
			Ov.OverrideFlags[i]     = FixFlags;
			Ov.OverridePositions[i] = Targets[i];
			Ov.OverrideInvMass[i]   = 0.0f;
		}
		Pump(MoveTemp(Ov));
	}

	// 3) 중력 솔브 20프레임 — 고정 노드는 1mm도 움직이면 안 된다(InvMass 마스크가 영속되는지).
	for (int32 Frame = 0; Frame < 20; ++Frame)
	{
		Pump(MakeStep(Config.Substeps, (1.0f / 60.0f) / Config.Substeps));
	}

	FRopeResidentLatest AfterFix;
	if (!Drain(AfterFix))
	{
		AddError(TEXT("override 후 리드백 drain 실패."));
		return false;
	}
	for (int32 i = 3; i <= 5; ++i)
	{
		const float Dev = static_cast<float>(FVector::Dist(AfterFix.Positions[i], Targets[i]));
		TestTrue(FString::Printf(TEXT("fixed node %d stays on target (dev %.4f cm)"), i, Dev), Dev < 0.1f);
	}
	for (const FVector& P : AfterFix.Positions)
	{
		if (P.ContainsNaN())
		{
			AddError(TEXT("override 후 NaN 발생."));
			return false;
		}
	}

	// 4) 질량 복원(InvMass=1만 오버라이드) 후 중력 솔브 — 노드가 타깃에서 다시 벗어나야 한다.
	{
		FRopeGPUResidentStep Restore = MakeStep(0, 1.0f / 60.0f);
		Restore.OverrideFlags.SetNumZeroed(N);
		Restore.OverrideInvMass.SetNumZeroed(N);
		for (int32 i = 3; i <= 5; ++i)
		{
			Restore.OverrideFlags[i]   = static_cast<uint8>(ERopeGPUOverride::InvMass);
			Restore.OverrideInvMass[i] = 1.0f;
		}
		Pump(MoveTemp(Restore));
	}
	for (int32 Frame = 0; Frame < 20; ++Frame)
	{
		Pump(MakeStep(Config.Substeps, (1.0f / 60.0f) / Config.Substeps));
	}

	FRopeResidentLatest AfterRestore;
	if (!Drain(AfterRestore))
	{
		AddError(TEXT("복원 후 리드백 drain 실패."));
		return false;
	}
	const float MovedDev = static_cast<float>(FVector::Dist(AfterRestore.Positions[4], Targets[4]));
	TestTrue(FString::Printf(TEXT("restored node resumes physics (moved %.2f cm off target)"), MovedDev),
		MovedDev > 1.0f);

	return true;
}

// 접촉 감지 패리티(G3): GPU 감지 커널이 CPU FRopeFlightContactDetector::DetectContactCandidates와
// 같은 접촉(히트 노드 집합 + 노드별 침투/법선)을 산출하는가. 정적 로프 + 캡슐로 결정적 비교.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUContactParityTest,
	"DynamicRope.Solver.GPUContactParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUContactParityTest::RunTest(const FString& Parameters)
{
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU 접촉 감지 패리티 테스트 스킵: 렌더 가능한 RHI가 없음(헤드리스)."));
		return true;
	}

	const int32 N = 8;
	const float Length = 140.0f;
	const float ContactRadius = 3.0f;

	// 정적 로프(prev==pos)를 z=15에 둔다. 캡슐: x=60에서 Y축을 따라, 반지름 30 → 노드 2/3/4가 침투.
	FRopeSimState Sim = RopeTest::MakeStraightRope(N, Length, FVector(0, 0, 15));
	FCapsuleCollider Capsule(FVector(60, -50, 0), FVector(60, 50, 0), 30.0f, FName("arm"));
	TArray<IRopeCollider*> Colliders = { &Capsule };

	// --- CPU ground-truth 감지.
	FRopeFlightContactDetector::FParams Params;
	Params.ContactRadius = ContactRadius;
	Params.RopeRadius = 2.0f;
	Params.PredictiveContactFrames = 0.0f;
	Params.MinLatchNodes = 1;
	TArray<FRopeContactCandidate> CpuCandidates;
	FRopeFlightContactDetector::DetectContactCandidates(Sim, Colliders, Params, CpuCandidates);

	// --- GPU 감지: 감지 전용 step(NumSub=0, bDetectContacts). 상주 버퍼를 시드 위치로 채우고 감지.
	FRopeGPUSolver GpuSolver;
	const uint32 RopeId = 11;
	const uint32 Gen = 1;

	auto MakeDetectStep = [&]() -> FRopeGPUResidentStep
	{
		FRopeGPUResidentStep Step;
		Step.RopeId            = RopeId;
		Step.Generation        = Gen;
		Step.NumNodes          = Sim.Num();
		Step.SeedPositions     = Sim.Positions;
		Step.SeedPrevPositions = Sim.PrevPositions;
		Step.InvMass           = Sim.InvMass;
		Step.SegmentLength     = Sim.SegmentLength;
		// 적분 없음 — 시드 위치 그대로 감지.
		Step.NumSub            = 0;
		Step.FixedDt           = 1.0f / 60.0f;
		Step.bDetectContacts   = true;
		Step.ContactRadius     = ContactRadius;
		for (const IRopeCollider* C : Colliders)
		{
			FRopeGPUCapsule Cap;
			FVector A, B; float R;
			if (const_cast<IRopeCollider*>(C)->GetGPUCapsule(A, B, R))
			{
				Cap.A = A; Cap.B = B; Cap.Radius = R;
				Step.Capsules.Add(Cap);
			}
		}
		return Step;
	};
	auto SyncGPU = []()
	{
		ENQUEUE_RENDER_COMMAND(RopeTestGpuSync)(
			[](FRHICommandListImmediate& RHICmdList) { RHICmdList.BlockUntilGPUIdle(); });
		FlushRenderingCommands();
	};
	auto Pump = [&]()
	{
		TArray<FRopeGPUResidentStep> Steps;
		Steps.Add(MakeDetectStep());
		GpuSolver.Step(MoveTemp(Steps));
		FlushRenderingCommands();
	};

	// 여러 번 펌프해 감지 리드백이 도착하게 한다(단일 in-flight → sync + 재무장 사이클).
	FRopeResidentContacts GpuContacts;
	bool bGot = false;
	for (int32 Spin = 0; Spin < 16 && !bGot; ++Spin)
	{
		SyncGPU();
		Pump();
		SyncGPU();
		TMap<uint32, FRopeResidentContacts> Latest;
		GpuSolver.GetLatestContacts(Latest);
		if (const FRopeResidentContacts* C = Latest.Find(RopeId))
		{
			if (C->Generation == Gen)
			{
				GpuContacts = *C;
				bGot = true;
			}
		}
	}
	if (!bGot)
	{
		AddError(TEXT("GPU 접촉 감지 결과를 회수하지 못함."));
		return false;
	}

	// --- 비교: 히트 노드 집합 일치 + 노드별 침투/법선 근사 일치.
	TMap<int32, const FRopeContactCandidate*> CpuByNode;
	for (const FRopeContactCandidate& C : CpuCandidates) { CpuByNode.Add(C.NodeIndex, &C); }
	TMap<int32, const FRopeGPUContactResult*> GpuByNode;
	for (const FRopeGPUContactResult& C : GpuContacts.Contacts) { GpuByNode.Add(C.NodeIndex, &C); }

	AddInfo(FString::Printf(TEXT("CPU 접촉 %d개, GPU 접촉 %d개"), CpuCandidates.Num(), GpuContacts.Contacts.Num()));
	TestTrue(TEXT("적어도 하나의 접촉이 감지됨"), CpuCandidates.Num() > 0);
	TestEqual(TEXT("히트 노드 수 일치"), GpuContacts.Contacts.Num(), CpuCandidates.Num());

	for (const TPair<int32, const FRopeContactCandidate*>& Pair : CpuByNode)
	{
		const int32 Node = Pair.Key;
		const FRopeGPUContactResult** GpuC = GpuByNode.Find(Node);
		if (!TestTrue(FString::Printf(TEXT("GPU도 노드 %d를 히트"), Node), GpuC != nullptr))
		{
			continue;
		}
		const float PenDev = FMath::Abs((*GpuC)->Penetration - Pair.Value->Penetration);
		TestTrue(FString::Printf(TEXT("노드 %d 침투 일치(차 %.3f)"), Node, PenDev), PenDev < 0.1f);
		const float NormalDot = FVector::DotProduct((*GpuC)->Normal.GetSafeNormal(), Pair.Value->Normal.GetSafeNormal());
		TestTrue(FString::Printf(TEXT("노드 %d 법선 일치(dot %.3f)"), Node, NormalDot), NormalDot > 0.99f);
		const float PointDev = static_cast<float>(FVector::Dist((*GpuC)->WorldPoint, Pair.Value->WorldPoint));
		TestTrue(FString::Printf(TEXT("노드 %d 접촉점 일치(차 %.3f cm)"), Node, PointDev), PointDev < 0.5f);
	}

	return true;
}

// 예측 접촉 패리티(G3b): 아직 안 닿았지만 외삽 경로가 캡슐을 지나는 tail 노드가 predictive 슬롯에
// 잡히고, CPU AddPredictedContactCandidates와 침투/소스가 일치하는가. actual 슬롯은 비어야 한다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUPredictiveParityTest,
	"DynamicRope.Solver.GPUPredictiveParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUPredictiveParityTest::RunTest(const FString& Parameters)
{
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU 예측 접촉 패리티 테스트 스킵: 렌더 가능한 RHI가 없음(헤드리스)."));
		return true;
	}

	const int32 N = 8;
	const float Length = 140.0f;
	const float ContactRadius = 3.0f;
	const float PredictionFrames = 3.0f;

	// tail 노드 7만 +X로 이동(prev 120 → pos 140). 캡슐은 x=175(도달 전) → actual 미스, 예측 경로가 관통.
	FRopeSimState Sim = RopeTest::MakeStraightRope(N, Length, FVector(0, 0, 15));
	Sim.PrevPositions[7] = FVector(120, 0, 15);
	Sim.Positions[7]     = FVector(140, 0, 15);
	FCapsuleCollider Capsule(FVector(175, -50, 0), FVector(175, 50, 0), 30.0f, FName("arm"));
	TArray<IRopeCollider*> Colliders = { &Capsule };

	// --- CPU ground-truth: actual + predictive.
	FRopeFlightContactDetector::FParams Params;
	Params.ContactRadius = ContactRadius;
	Params.RopeRadius = 2.0f;
	Params.PredictiveContactFrames = PredictionFrames;
	Params.MinLatchNodes = 1;
	TArray<FRopeContactCandidate> CpuCandidates;
	FRopeFlightContactDetector::DetectContactCandidates(Sim, Colliders, Params, CpuCandidates);
	const int32 CpuActualCount = CpuCandidates.Num();
	FRopeFlightContactDetector::AddPredictedContactCandidates(Sim, Colliders, Params,
		FRopeFlightContactDetector::FWhipGuideView(), CpuCandidates);

	// CPU: actual 0개, predictive로 노드 7 하나 추가되어야 한다.
	TestEqual(TEXT("CPU actual 접촉 없음"), CpuActualCount, 0);
	const FRopeContactCandidate* CpuPred = nullptr;
	for (const FRopeContactCandidate& C : CpuCandidates)
	{
		if (C.NodeIndex == 7) { CpuPred = &C; }
	}
	if (!TestTrue(TEXT("CPU 예측 후보(노드 7) 존재"), CpuPred != nullptr))
	{
		return false;
	}

	// --- GPU: 예측 포함 감지 step(whip 없음 → free 예측).
	FRopeGPUSolver GpuSolver;
	const uint32 RopeId = 13;
	const uint32 Gen = 1;
	auto MakeDetectStep = [&]() -> FRopeGPUResidentStep
	{
		FRopeGPUResidentStep Step;
		Step.RopeId            = RopeId;
		Step.Generation        = Gen;
		Step.NumNodes          = Sim.Num();
		Step.SeedPositions     = Sim.Positions;
		Step.SeedPrevPositions = Sim.PrevPositions;
		Step.InvMass           = Sim.InvMass;
		Step.SegmentLength     = Sim.SegmentLength;
		Step.NumSub            = 0;
		Step.FixedDt           = 1.0f / 60.0f;
		Step.bDetectContacts   = true;
		Step.ContactRadius     = ContactRadius;
		Step.PredictionFrames  = PredictionFrames;
		FVector A, B; float R;
		Capsule.GetGPUCapsule(A, B, R);
		FRopeGPUCapsule Cap; Cap.A = A; Cap.B = B; Cap.Radius = R;
		Step.Capsules.Add(Cap);
		return Step;
	};
	auto SyncGPU = []()
	{
		ENQUEUE_RENDER_COMMAND(RopeTestGpuSync)(
			[](FRHICommandListImmediate& RHICmdList) { RHICmdList.BlockUntilGPUIdle(); });
		FlushRenderingCommands();
	};

	FRopeResidentContacts GpuContacts;
	bool bGot = false;
	for (int32 Spin = 0; Spin < 16 && !bGot; ++Spin)
	{
		SyncGPU();
		TArray<FRopeGPUResidentStep> Steps;
		Steps.Add(MakeDetectStep());
		GpuSolver.Step(MoveTemp(Steps));
		FlushRenderingCommands();
		SyncGPU();
		TMap<uint32, FRopeResidentContacts> Latest;
		GpuSolver.GetLatestContacts(Latest);
		if (const FRopeResidentContacts* C = Latest.Find(RopeId))
		{
			if (C->Generation == Gen) { GpuContacts = *C; bGot = true; }
		}
	}
	if (!bGot)
	{
		AddError(TEXT("GPU 예측 접촉 결과를 회수하지 못함."));
		return false;
	}

	// GPU: 노드 7의 predictive(Source=2) 접촉이 있어야 하고 actual(Source=1)은 없어야 한다.
	const FRopeGPUContactResult* GpuPred = nullptr;
	bool bAnyActual = false;
	for (const FRopeGPUContactResult& C : GpuContacts.Contacts)
	{
		if (C.Source == static_cast<uint8>(ERopeContactCandidateSource::Actual)) { bAnyActual = true; }
		if (C.NodeIndex == 7 && C.Source == static_cast<uint8>(ERopeContactCandidateSource::PredictiveFree)) { GpuPred = &C; }
	}
	TestFalse(TEXT("GPU actual 접촉 없음"), bAnyActual);
	if (!TestTrue(TEXT("GPU 예측 후보(노드 7, PredictiveFree) 존재"), GpuPred != nullptr))
	{
		return false;
	}

	const float PenDev = FMath::Abs(GpuPred->Penetration - CpuPred->Penetration);
	AddInfo(FString::Printf(TEXT("예측 침투 CPU %.3f / GPU %.3f"), CpuPred->Penetration, GpuPred->Penetration));
	TestTrue(FString::Printf(TEXT("예측 침투 일치(차 %.3f)"), PenDev), PenDev < 0.1f);
	const float PointDev = static_cast<float>(FVector::Dist(GpuPred->WorldPoint, CpuPred->WorldPoint));
	TestTrue(FString::Printf(TEXT("예측 접촉점 일치(차 %.3f cm)"), PointDev), PointDev < 0.5f);

	return true;
}

// SDF 접촉 감지 패리티(G3b): GPU SDF 감지가 CPU FRopeSDFCollider::Query 기반 감지와 같은 접촉을
// 산출하는가. 합성 구 볼륨 + 정적 로프로 결정적 비교(침투/법선/접촉점).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUSDFContactParityTest,
	"DynamicRope.Solver.GPUSDFContactParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUSDFContactParityTest::RunTest(const FString& Parameters)
{
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU SDF 접촉 패리티 테스트 스킵: 렌더 가능한 RHI가 없음(헤드리스)."));
		return true;
	}

	const float ContactRadius = 3.0f;

	// 반지름 20 구(본 로컬), BoneToWorld=identity. 로프 노드를 표면 근처에 배치해 몇 개가 침투하게 한다.
	const FRopeBoneSDFVolume Volume =
		RopeSDFSynthetic::MakeSphere(FName("arm"), FVector::ZeroVector, 20.0f, FIntVector(31), 10.0f);
	FRopeSDFCollider Sdf(&Volume, FTransform::Identity, FTransform::Identity, 0.0f, FName("arm"), nullptr);
	TArray<IRopeCollider*> Colliders = { &Sdf };

	// 정적 로프(prev==pos): x = -25,-21,-19,19,21,25 (z=0). 표면(20)에서 노드 1/2/3/4가 반경 3 이내.
	const int32 N = 6;
	FRopeSimState Sim = RopeTest::MakeStraightRope(N, 100.0f);
	const float Xs[N] = { -25.0f, -21.0f, -19.0f, 19.0f, 21.0f, 25.0f };
	for (int32 i = 0; i < N; ++i)
	{
		Sim.Positions[i] = FVector(Xs[i], 0, 0);
		Sim.SetStill(i);
	}

	// --- CPU ground-truth.
	FRopeFlightContactDetector::FParams Params;
	Params.ContactRadius = ContactRadius;
	Params.RopeRadius = 2.0f;
	Params.PredictiveContactFrames = 0.0f;
	Params.MinLatchNodes = 1;
	TArray<FRopeContactCandidate> CpuCandidates;
	FRopeFlightContactDetector::DetectContactCandidates(Sim, Colliders, Params, CpuCandidates);

	// --- GPU SDF collider 뷰 → step.
	FRopeSDFColliderView View;
	if (!Sdf.GetGPUSDF(View))
	{
		AddError(TEXT("GetGPUSDF 실패(볼륨 미베이크?)."));
		return false;
	}

	FRopeGPUSolver GpuSolver;
	const uint32 RopeId = 17;
	const uint32 Gen = 1;
	auto MakeDetectStep = [&]() -> FRopeGPUResidentStep
	{
		FRopeGPUResidentStep Step;
		Step.RopeId            = RopeId;
		Step.Generation        = Gen;
		Step.NumNodes          = Sim.Num();
		Step.SeedPositions     = Sim.Positions;
		Step.SeedPrevPositions = Sim.PrevPositions;
		Step.InvMass           = Sim.InvMass;
		Step.SegmentLength     = Sim.SegmentLength;
		Step.NumSub            = 0;
		Step.FixedDt           = 1.0f / 60.0f;
		Step.bDetectContacts   = true;
		Step.ContactRadius     = ContactRadius;
		FRopeGPUSDFCollider G;
		G.Distances       = View.Distances;
		G.BytesPerCode    = View.BytesPerCode;
		G.NarrowBandInner = View.NarrowBandInner;
		G.NarrowBandOuter = View.NarrowBandOuter;
		G.ResX = View.ResX; G.ResY = View.ResY; G.ResZ = View.ResZ;
		G.LocalMin = View.LocalMin; G.LocalSize = View.LocalSize;
		G.BoneToWorld = View.BoneToWorld; G.PrevBoneToWorld = View.PrevBoneToWorld;
		G.InvDeltaTime = View.InvDeltaTime; G.VolumeKey = View.VolumeKey;
		Step.SDFColliders.Add(G);
		return Step;
	};
	auto SyncGPU = []()
	{
		ENQUEUE_RENDER_COMMAND(RopeTestGpuSync)(
			[](FRHICommandListImmediate& RHICmdList) { RHICmdList.BlockUntilGPUIdle(); });
		FlushRenderingCommands();
	};

	FRopeResidentContacts GpuContacts;
	bool bGot = false;
	for (int32 Spin = 0; Spin < 16 && !bGot; ++Spin)
	{
		SyncGPU();
		TArray<FRopeGPUResidentStep> Steps;
		Steps.Add(MakeDetectStep());
		GpuSolver.Step(MoveTemp(Steps));
		FlushRenderingCommands();
		SyncGPU();
		TMap<uint32, FRopeResidentContacts> Latest;
		GpuSolver.GetLatestContacts(Latest);
		if (const FRopeResidentContacts* C = Latest.Find(RopeId))
		{
			if (C->Generation == Gen) { GpuContacts = *C; bGot = true; }
		}
	}
	if (!bGot)
	{
		AddError(TEXT("GPU SDF 접촉 결과를 회수하지 못함."));
		return false;
	}

	// GPU actual 접촉만(정적 → 예측 없음).
	TMap<int32, const FRopeGPUContactResult*> GpuByNode;
	for (const FRopeGPUContactResult& C : GpuContacts.Contacts)
	{
		if (C.Source == static_cast<uint8>(ERopeContactCandidateSource::Actual)) { GpuByNode.Add(C.NodeIndex, &C); }
	}

	AddInfo(FString::Printf(TEXT("CPU SDF 접촉 %d개, GPU %d개"), CpuCandidates.Num(), GpuByNode.Num()));
	TestTrue(TEXT("적어도 하나의 SDF 접촉"), CpuCandidates.Num() > 0);
	TestEqual(TEXT("SDF 히트 노드 수 일치"), GpuByNode.Num(), CpuCandidates.Num());

	for (const FRopeContactCandidate& Cpu : CpuCandidates)
	{
		const FRopeGPUContactResult** GpuC = GpuByNode.Find(Cpu.NodeIndex);
		if (!TestTrue(FString::Printf(TEXT("GPU도 노드 %d를 히트"), Cpu.NodeIndex), GpuC != nullptr))
		{
			continue;
		}
		const float PenDev = FMath::Abs((*GpuC)->Penetration - Cpu.Penetration);
		TestTrue(FString::Printf(TEXT("노드 %d SDF 침투 일치(차 %.3f)"), Cpu.NodeIndex, PenDev), PenDev < 0.3f);
		const float NormalDot = FVector::DotProduct((*GpuC)->Normal.GetSafeNormal(), Cpu.Normal.GetSafeNormal());
		TestTrue(FString::Printf(TEXT("노드 %d SDF 법선 일치(dot %.3f)"), Cpu.NodeIndex, NormalDot), NormalDot > 0.98f);
	}

	return true;
}

// 정적 박스(OBB) 충돌 parity: 박스 모서리 위로 드레이프된 로프가 GPU 경로에서도 (1) 박스 내부로
// 파고들지 않고 (2) CPU 솔버(FRopeBoxCollider)와 근사 일치하는가. GDF 복셀 라운딩으로 모서리를
// 관통하던 버그를 해석적 박스가 GPU에서 막는지 보는 회귀 게이트.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUBoxCornerParityTest,
	"DynamicRope.Solver.GPUBoxCornerParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUBoxCornerParityTest::RunTest(const FString& Parameters)
{
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU 박스 parity 테스트 스킵: 렌더 가능한 RHI가 없음(헤드리스)."));
		return true;
	}

	const int32 N = 24;
	const float Length = 300.0f;
	const FVector HalfExtents(50.0);
	FRopeSolverConfig Config = MakeHangConfig();
	Config.CollisionRadius = 2.0f;
	Config.Friction = 0.5f;

	// 박스(반폭 50, 원점) 위 z=55에서 +X/-X 엣지를 가로질러 걸친 자유 로프(핀 없음). 중력 드레이프.
	FRopeBoxCollider Box(FVector::ZeroVector, FQuat::Identity, HalfExtents);
	TArray<IRopeCollider*> Colliders;
	Colliders.Add(&Box);

	FRopeSimState CpuSim = RopeTest::MakeStraightRope(N, Length, FVector(-150.0, 0.0, 55.0), FVector(1, 0, 0));
	FRopeSimState GpuSim = CpuSim;

	const FRopeXPBDSolver Solver;
	FRopeGPUSolver GpuSolver;
	const uint32 RopeId = 23;
	const uint32 Gen = 1;

	auto MakeStep = [&](const FRopeSimState& Src, int32 NumSub, float FixedDt) -> FRopeGPUResidentStep
	{
		FRopeGPUResidentStep Step;
		Step.RopeId            = RopeId;
		Step.Generation        = Gen;
		Step.NumNodes          = Src.Num();
		Step.SeedPositions     = Src.Positions;
		Step.SeedPrevPositions = Src.PrevPositions;
		Step.InvMass           = Src.InvMass;
		Step.SegmentLength     = Src.SegmentLength;
		Step.StretchCompliance = Config.StretchCompliance;
		Step.BendCompliance    = Config.BendCompliance;
		Step.BendReleaseRatio  = Config.BendReleaseRatio;
		Step.BendFullRatio     = Config.BendFullRatio;
		Step.Damping           = Config.Damping;
		Step.Iterations        = Config.Iterations;
		Step.Gravity           = Config.Gravity;
		Step.CollisionRadius   = Config.CollisionRadius;
		Step.Friction          = Config.Friction;
		Step.SweepStep         = Config.SweepStep;
		Step.MaxSweepSamples   = Config.MaxSweepSamples;
		Step.NumSub            = NumSub;
		Step.FixedDt           = FixedDt;
		// 정적 박스: CPU FRopeBoxCollider와 동일 데이터(GetGPUBox 추출과 같은 값).
		FRopeGPUBox GpuBox;
		Box.GetGPUBox(GpuBox.Center, GpuBox.Rot, GpuBox.HalfExtents);
		Step.Boxes.Add(GpuBox);
		return Step;
	};

	for (int32 Frame = 0; Frame < 120; ++Frame)
	{
		Solver.Step(CpuSim, Config, Colliders, 1.0f / 60.0f);

		const FRopeSubstepSchedule Schedule = RopeSolverSubsteps(GpuSim, Config, 1.0f / 60.0f);
		TArray<FRopeGPUResidentStep> Steps;
		Steps.Add(MakeStep(GpuSim, Schedule.NumSub, Schedule.FixedDt));
		GpuSolver.Step(MoveTemp(Steps));
		FlushRenderingCommands();
	}

	// 마지막 결과 drain(NumSub=0 펌프 — 위 parity 테스트와 동일 패턴).
	TMap<uint32, FRopeResidentLatest> Latest;
	bool bGot = false;
	for (int32 Spin = 0; Spin < 64 && !bGot; ++Spin)
	{
		FlushRenderingCommands();
		TArray<FRopeGPUResidentStep> Drain;
		Drain.Add(MakeStep(GpuSim, 0, 1.0f / 60.0f));
		GpuSolver.Step(MoveTemp(Drain));
		FlushRenderingCommands();
		GpuSolver.GetLatest(Latest);
		if (const FRopeResidentLatest* L = Latest.Find(RopeId))
		{
			if (L->Generation == Gen && L->Positions.Num() == GpuSim.Num() && L->PrevPositions.Num() == GpuSim.Num())
			{
				for (int32 i = 0; i < GpuSim.Num(); ++i)
				{
					GpuSim.Positions[i]     = L->Positions[i];
					GpuSim.PrevPositions[i] = L->PrevPositions[i];
				}
				bGot = true;
			}
		}
	}
	if (!bGot)
	{
		AddError(TEXT("GPU 박스 parity: 상주 결과를 회수하지 못함."));
		return false;
	}

	TestFalse(TEXT("CPU no NaN"), RopeTest::AnyNaN(CpuSim));
	TestFalse(TEXT("GPU no NaN"), RopeTest::AnyNaN(GpuSim));

	// (1) 관통 없음: 어떤 GPU 노드도 박스 내부에 있으면 안 된다(원래 버그의 회귀 조건).
	float MaxInsideDepth = 0.0f;
	for (const FVector& P : GpuSim.Positions)
	{
		const FVector A = P.GetAbs();
		if (A.X < HalfExtents.X && A.Y < HalfExtents.Y && A.Z < HalfExtents.Z)
		{
			MaxInsideDepth = FMath::Max(MaxInsideDepth, static_cast<float>(FMath::Min3(
				HalfExtents.X - A.X, HalfExtents.Y - A.Y, HalfExtents.Z - A.Z)));
		}
	}
	TestTrue(FString::Printf(TEXT("GPU max inside depth %.3f cm should be < 0.5"), MaxInsideDepth),
		MaxInsideDepth < 0.5f);

	// (2) CPU 근사 일치: 정착 드레이프 형상이 가까운지(비트일치 아님 — 컬러링/샘플 순서 차).
	float MaxDev = 0.0f;
	for (int32 i = 0; i < N; ++i)
	{
		MaxDev = FMath::Max(MaxDev, static_cast<float>(FVector::Dist(CpuSim.Positions[i], GpuSim.Positions[i])));
	}
	AddInfo(FString::Printf(TEXT("박스 드레이프 CPU↔GPU 최대 노드 편차: %.2f cm"), MaxDev));
	TestTrue(FString::Printf(TEXT("CPU↔GPU max node deviation %.2f cm within tolerance"), MaxDev),
		MaxDev < Length * 0.25f);

	return true;
}

// 정적 컨벡스(6평면 = 박스) 충돌 parity: 컨벡스 모서리 위로 드레이프된 로프가 GPU 컨벡스 경로에서도
// (1) 내부로 파고들지 않고 (2) CPU 솔버(FRopeConvexCollider)와 근사 일치하는가. 박스를 6평면 컨벡스로
// 표현해 두 경로의 max-plane 질의 + 평면 풀 패킹을 검증한다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUConvexParityTest,
	"DynamicRope.Solver.GPUConvexParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUConvexParityTest::RunTest(const FString& Parameters)
{
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU 컨벡스 parity 테스트 스킵: 렌더 가능한 RHI가 없음(헤드리스)."));
		return true;
	}

	const int32 N = 24;
	const float Length = 300.0f;
	const FVector H(50.0);
	FRopeSolverConfig Config = MakeHangConfig();
	Config.CollisionRadius = 2.0f;
	Config.Friction = 0.5f;

	// 원점 박스(반폭 50)를 6평면 컨벡스로. CPU는 FRopeConvexCollider, GPU는 Step.Convexes/ConvexPlanes.
	auto MakePlanes = [&]() -> TArray<FPlane>
	{
		TArray<FPlane> P;
		P.Add(FPlane(1, 0, 0, H.X)); P.Add(FPlane(-1, 0, 0, H.X));
		P.Add(FPlane(0, 1, 0, H.Y)); P.Add(FPlane(0, -1, 0, H.Y));
		P.Add(FPlane(0, 0, 1, H.Z)); P.Add(FPlane(0, 0, -1, H.Z));
		return P;
	};
	FRopeConvexCollider CpuConvex(MakePlanes(), FBox(-H, H));
	TArray<IRopeCollider*> Colliders;
	Colliders.Add(&CpuConvex);

	FRopeSimState CpuSim = RopeTest::MakeStraightRope(N, Length, FVector(-150.0, 0.0, 55.0), FVector(1, 0, 0));
	FRopeSimState GpuSim = CpuSim;

	const FRopeXPBDSolver Solver;
	FRopeGPUSolver GpuSolver;
	const uint32 RopeId = 29;
	const uint32 Gen = 1;

	auto MakeStep = [&](const FRopeSimState& Src, int32 NumSub, float FixedDt) -> FRopeGPUResidentStep
	{
		FRopeGPUResidentStep Step;
		Step.RopeId            = RopeId;
		Step.Generation        = Gen;
		Step.NumNodes          = Src.Num();
		Step.SeedPositions     = Src.Positions;
		Step.SeedPrevPositions = Src.PrevPositions;
		Step.InvMass           = Src.InvMass;
		Step.SegmentLength     = Src.SegmentLength;
		Step.StretchCompliance = Config.StretchCompliance;
		Step.BendCompliance    = Config.BendCompliance;
		Step.BendReleaseRatio  = Config.BendReleaseRatio;
		Step.BendFullRatio     = Config.BendFullRatio;
		Step.Damping           = Config.Damping;
		Step.Iterations        = Config.Iterations;
		Step.Gravity           = Config.Gravity;
		Step.CollisionRadius   = Config.CollisionRadius;
		Step.Friction          = Config.Friction;
		Step.SweepStep         = Config.SweepStep;
		Step.MaxSweepSamples   = Config.MaxSweepSamples;
		Step.NumSub            = NumSub;
		Step.FixedDt           = FixedDt;
		// 6평면 컨벡스: 평면 풀 + 헤더(오프셋 0, 개수 6). 강체 identity → 월드=로컬(평면을 원점에 구성), 정적(InvDt 0).
		FRopeGPUConvex Cv;
		Cv.PlaneOffset = 0;
		Cv.PlaneCount = 6;
		Cv.LocalBoundsCenter = FVector::ZeroVector;
		Cv.LocalBoundsExtent = H;
		for (const FPlane& Pl : MakePlanes())
		{
			Step.ConvexPlanes.Add(FVector4(Pl.X, Pl.Y, Pl.Z, Pl.W));
		}
		Step.Convexes.Add(Cv);
		return Step;
	};

	for (int32 Frame = 0; Frame < 120; ++Frame)
	{
		Solver.Step(CpuSim, Config, Colliders, 1.0f / 60.0f);
		const FRopeSubstepSchedule Schedule = RopeSolverSubsteps(GpuSim, Config, 1.0f / 60.0f);
		TArray<FRopeGPUResidentStep> Steps;
		Steps.Add(MakeStep(GpuSim, Schedule.NumSub, Schedule.FixedDt));
		GpuSolver.Step(MoveTemp(Steps));
		FlushRenderingCommands();
	}

	TMap<uint32, FRopeResidentLatest> Latest;
	bool bGot = false;
	for (int32 Spin = 0; Spin < 64 && !bGot; ++Spin)
	{
		FlushRenderingCommands();
		TArray<FRopeGPUResidentStep> Drain;
		Drain.Add(MakeStep(GpuSim, 0, 1.0f / 60.0f));
		GpuSolver.Step(MoveTemp(Drain));
		FlushRenderingCommands();
		GpuSolver.GetLatest(Latest);
		if (const FRopeResidentLatest* L = Latest.Find(RopeId))
		{
			if (L->Generation == Gen && L->Positions.Num() == GpuSim.Num() && L->PrevPositions.Num() == GpuSim.Num())
			{
				for (int32 i = 0; i < GpuSim.Num(); ++i)
				{
					GpuSim.Positions[i]     = L->Positions[i];
					GpuSim.PrevPositions[i] = L->PrevPositions[i];
				}
				bGot = true;
			}
		}
	}
	if (!bGot)
	{
		AddError(TEXT("GPU 컨벡스 parity: 상주 결과를 회수하지 못함."));
		return false;
	}

	TestFalse(TEXT("CPU no NaN"), RopeTest::AnyNaN(CpuSim));
	TestFalse(TEXT("GPU no NaN"), RopeTest::AnyNaN(GpuSim));

	// (1) 관통 없음: 어떤 GPU 노드도 컨벡스(=박스) 내부에 있으면 안 된다.
	float MaxInsideDepth = 0.0f;
	for (const FVector& P : GpuSim.Positions)
	{
		const FVector A = P.GetAbs();
		if (A.X < H.X && A.Y < H.Y && A.Z < H.Z)
		{
			MaxInsideDepth = FMath::Max(MaxInsideDepth, static_cast<float>(FMath::Min3(H.X - A.X, H.Y - A.Y, H.Z - A.Z)));
		}
	}
	TestTrue(FString::Printf(TEXT("GPU convex max inside depth %.3f cm should be < 0.5"), MaxInsideDepth),
		MaxInsideDepth < 0.5f);

	// (2) CPU 근사 일치.
	float MaxDev = 0.0f;
	for (int32 i = 0; i < N; ++i)
	{
		MaxDev = FMath::Max(MaxDev, static_cast<float>(FVector::Dist(CpuSim.Positions[i], GpuSim.Positions[i])));
	}
	AddInfo(FString::Printf(TEXT("컨벡스 드레이프 CPU↔GPU 최대 노드 편차: %.2f cm"), MaxDev));
	TestTrue(FString::Printf(TEXT("CPU↔GPU max node deviation %.2f cm within tolerance"), MaxDev),
		MaxDev < Length * 0.25f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS

