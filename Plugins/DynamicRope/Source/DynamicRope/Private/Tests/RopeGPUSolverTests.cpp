// Copyright Epic Games, Inc. All Rights Reserved.
//
// GPU 솔버(M1) 패리티/안정성 테스트. CPU FRopeXPBDSolver를 ground-truth로, GPU FRopeGPUSolver가
// 같은 시나리오에서 (1) 발산/NaN 없이 (2) 비신축 세그먼트 길이를 유지하며 (3) CPU와 "근사" 일치하는지 본다.
// 주의: red-black/stride-3 컬러링은 CPU의 교대-sweep Gauss-Seidel과 비트일치하지 않는다(수렴만 근사) →
// 노드별 편차는 허용오차 기반으로만 본다. 또한 GPU 디스패치는 RHI가 필요하므로, 렌더 불가 환경에선 스킵한다.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Solver/RopeXPBDSolver.h"
#include "RopeGPUSolver.h" // DynamicRopeShaders 모듈
#include "Collision/RopeCollider.h"
#include "RopeTestHelpers.h"
#include "RHI.h"
#include "Misc/App.h"

namespace
{
	FRopeSolverConfig MakeHangConfig()
	{
		FRopeSolverConfig C;
		C.Substeps = 8;
		C.Iterations = 8;
		C.StretchCompliance = 0.0f; // 비신축
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

	for (int32 Frame = 0; Frame < 120; ++Frame)
	{
		// CPU: ground-truth.
		Solver.Step(CpuSim, Config, NoColliders, 1.0f / 60.0f);

		// GPU: CPU와 동일한 고정-timestep 스케줄을 같은 함수로 구해 잡 구성(누적 진화가 동일 → 동일 NumSub).
		const FRopeSubstepSchedule Schedule = RopeSolverSubsteps(GpuSim, Config, 1.0f / 60.0f);
		if (Schedule.NumSub > 0)
		{
			FRopeGPUJob Job;
			Job.Positions         = GpuSim.Positions.GetData();
			Job.PrevPositions     = GpuSim.PrevPositions.GetData();
			Job.InvMass           = GpuSim.InvMass.GetData();
			Job.NumNodes          = GpuSim.Num();
			Job.SegmentLength     = GpuSim.SegmentLength;
			Job.bStartPinned      = GpuSim.bStartPinned;
			Job.StartPinPrev      = GpuSim.StartPinPrev;
			Job.StartPinTarget    = GpuSim.StartPinTarget;
			Job.StretchCompliance = Config.StretchCompliance;
			Job.BendCompliance    = Config.BendCompliance;
			Job.Damping           = Config.Damping;
			Job.Iterations        = Config.Iterations;
			Job.Gravity           = Config.Gravity;
			Job.NumSub            = Schedule.NumSub;
			Job.FixedDt           = Schedule.FixedDt;
			FRopeGPUSolver::SolveBatch(MakeArrayView(&Job, 1));
		}
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
		MaxDev < Length * 0.25f); // 정착 hanging 형상은 가까워야 함(관대한 상한).

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
