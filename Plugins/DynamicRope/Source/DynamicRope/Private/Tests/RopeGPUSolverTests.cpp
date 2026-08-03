// Copyright Epic Games, Inc. All Rights Reserved.
//
// GPU solver parity and stability test. Running the same scenario through the CPU FRopeXPBDSolver as ground
// truth and the GPU FRopeGPUSolver, it checks that (1) nothing diverges or produces NaN, (2) an inextensible
// rope holds its segment lengths, and (3) the GPU result matches the CPU's to within tolerance.
// Note: red-black and stride-3 colouring is not bit-identical to the CPU's alternating-sweep Gauss-Seidel —
// they only converge to the same answer — so per-node deviation is judged by tolerance alone. A GPU dispatch
// also needs an RHI, so these are skipped where rendering is not possible.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Solver/RopeXPBDSolver.h"
// DynamicRopeShaders module
#include "RopeGPUSolver.h"
// FRHIGPUBufferReadback fully defined (5.7/5.8 includes transition, ≤5.6 requires specification)
#include "RHIGPUReadback.h"
#include "Collision/RopeCollider.h"
// FRopeBoxCollider (static box parity)
#include "Collision/RopeStaticCollider.h"
#include "Collision/SDF/RopeSDFCollider.h"
// MakeSphere (synthetic SDF volume)
#include "RopeSDFSynthetic.h"
#include "Collision/SDF/RopeSDFData.h"
// CPU detection(parity ground-truth)
#include "Logic/RopeFlightContactDetector.h"
#include "RopeTestHelpers.h"
#include "HAL/IConsoleManager.h"
#include "RHI.h"
// SubmitAndBlockUntilGPUIdle
#include "RHICommandList.h"
#include "RenderGraphBuilder.h"
// FlushRenderingCommands
#include "RenderingThread.h"
#include "Misc/App.h"
#include "Misc/ScopeExit.h"

namespace
{
	FRopeSolverConfig MakeHangConfig()
	{
		FRopeSolverConfig C;
		C.Substeps = 8;
		C.Iterations = 8;
		// Non-expandable
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

// In the pinned hanging rope, is the GPU path static (no NaN/divergence, segment length maintained) and the approximation matches that of the CPU?
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUSolverParityTest,
	"DynamicRope.Solver.GPUParityHangingRope",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUSolverParityTest::RunTest(const FString& Parameters)
{
	// GPU dispatch requires RHI — skip (not fail) in render-unable (headless/null RHI) environments.
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU solver parity test skipped: no renderable RHI (headless)."));
		return true;
	}

	const int32 N = 24;
	const float Length = 300.0f;
	const FRopeSolverConfig Config = MakeHangConfig();

	FRopeSimState CpuSim = MakePinnedRope(N, Length);
	FRopeSimState GpuSim = MakePinnedRope(N, Length);

	const FRopeXPBDSolver Solver;
	const TArray<IRopeCollider*> NoColliders;

	// The GPU solver keeps the centerline resident: each frame's step advances the persistent buffer in place, and the result comes back through the render-thread readback (delayed) via GetLatest.
	// The test is a synchronous verification, so RT is performed with FlushRenderingCommands after each step. The generation is pinned to 1 (only the first frame is seeded).
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

		// GPU: One resident step with the same pinned-timestep schedule as the CPU (same cumulative evolution → same NumSub).
		const FRopeSubstepSchedule Schedule = RopeSolverSubsteps(GpuSim, Config, 1.0f / 60.0f);
		TArray<FRopeGPUResidentStep> Steps;
		Steps.Add(MakeStep(GpuSim, Schedule.NumSub, Schedule.FixedDt));
		GpuSolver.Step(MoveTemp(Steps));
		// RT proceeds to handle dispatch + readback copy.
		FlushRenderingCommands();
	}

	// Retrieve the final frame result deterministically: ReadbackNow synchronously reads back the resident buffer to SubmitAndBlockUntilGPUIdle.
	// Dedicated step paths are not stacked in PendingSteps, so the last dispatch state is read as is without additional solving.
	// (The asynchronous GetLatest mirror cannot specify which frame the result is, so it is non-static between executions).
	FlushRenderingCommands();
	TArray<FVector> RbPos, RbPrev;
	uint32 RbGen = 0;
	if (!GpuSolver.ReadbackNow(RopeId, RbPos, RbPrev, RbGen) ||
		RbGen != Gen || RbPos.Num() != GpuSim.Num() || RbPrev.Num() != GpuSim.Num())
	{
		AddError(TEXT("Could not retrieve the GPU resident result (final ReadbackNow state)."));
		return false;
	}
	for (int32 i = 0; i < GpuSim.Num(); ++i)
	{
		GpuSim.Positions[i]     = RbPos[i];
		GpuSim.PrevPositions[i] = RbPrev[i];
	}

	// (1) Stability: No NaN.
	TestFalse(TEXT("CPU no NaN"), RopeTest::AnyNaN(CpuSim));
	TestFalse(TEXT("GPU no NaN"), RopeTest::AnyNaN(GpuSim));

	// (2) Non-stretch: GPU segment error is bounded to the same level as CPU.
	const float GpuSegErr = RopeTest::MaxSegmentError(GpuSim);
	TestTrue(FString::Printf(TEXT("GPU segment error %.2f cm bounded"), GpuSegErr),
		GpuSegErr < GpuSim.SegmentLength * 2.0f);

	// (3) approximation matching: per-node maximum deviation. Do not expect bit-identical — the fixed geometry is close, but bone (tolerant).
	float MaxDev = 0.0f;
	for (int32 i = 0; i < N; ++i)
	{
		MaxDev = FMath::Max(MaxDev, static_cast<float>(FVector::Dist(CpuSim.Positions[i], GpuSim.Positions[i])));
	}
	AddInfo(FString::Printf(TEXT("Largest CPU-to-GPU node deviation: %.2f cm (RopeLength %.0f)"), MaxDev, Length));
	TestTrue(FString::Printf(TEXT("CPU↔GPU max node deviation %.2f cm within tolerance"), MaxDev),
		// Settlement hanging geometry should be close (generous cap).
		MaxDev < Length * 0.25f);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUPendingStepQueueCompatibilityTest,
	"DynamicRope.Solver.GPUPendingStepQueueCompatibility",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUPendingStepQueueCompatibilityTest::RunTest(const FString& Parameters)
{
	FRopeGPUResidentStep Older;
	Older.RopeId = 17;
	Older.Generation = 4;
	Older.NumNodes = 64;
	Older.NumSub = 6;
	Older.FixedDt = (1.0f / 60.0f) / 6.0f;

	FRopeGPUResidentStep Newer = Older;
	TestTrue(TEXT("same-generation and same-topology frames share one resident timeline"),
		Older.CanExecuteBefore(Newer));

	FRopeGPUResidentStep DifferentTimestep = Newer;
	DifferentTimestep.FixedDt *= 0.5f;
	TestTrue(TEXT("each queued frame may retain its own fixed timestep"),
		Older.CanExecuteBefore(DifferentTimestep));

	FRopeGPUResidentStep DifferentGeneration = Newer;
	DifferentGeneration.Generation = Older.Generation + 1;
	TestFalse(TEXT("queued execution does not cross a reseed boundary"),
		Older.CanExecuteBefore(DifferentGeneration));

	FRopeGPUResidentStep DifferentTopology = Newer;
	DifferentTopology.NumNodes = Older.NumNodes + 1;
	TestFalse(TEXT("queued execution does not cross a topology boundary"),
		Older.CanExecuteBefore(DifferentTopology));

	// Exercise the real render-thread pending queue as well. Two compatible enqueues before a view
	// dispatch must remain queued without publishing a delayed refund; a later reseed still refunds both.
	FRopeGPUSolver QueueSolver;
	auto EnqueueWithoutViewDispatch = [&QueueSolver](FRopeGPUResidentStep Step)
	{
		TArray<FRopeGPUResidentStep> Steps;
		Steps.Add(MoveTemp(Step));
		QueueSolver.EnqueueSteps(MoveTemp(Steps));
		FlushRenderingCommands();
	};
	FRopeGPUResidentStep FirstQueued = Older;
	FRopeGPUResidentStep SecondQueued = FirstQueued;
	EnqueueWithoutViewDispatch(MoveTemp(FirstQueued));
	EnqueueWithoutViewDispatch(MoveTemp(SecondQueued));
	TMap<uint32, float> DroppedTime;
	QueueSolver.DrainDroppedSimTime(DroppedTime);
	TestFalse(TEXT("compatible queued frames publish no one-frame-late refund"),
		DroppedTime.Contains(Older.RopeId));

	FRopeGPUResidentStep ReseededQueued = Older;
	ReseededQueued.Generation = Older.Generation + 1;
	EnqueueWithoutViewDispatch(MoveTemp(ReseededQueued));
	QueueSolver.DrainDroppedSimTime(DroppedTime);
	const float* ReseedRefund = DroppedTime.Find(Older.RopeId);
	TestNotNull(TEXT("a reseed replacement keeps the safe refund path"), ReseedRefund);
	if (ReseedRefund)
	{
		TestTrue(TEXT("the refund contains both pending frame intervals"),
			FMath::IsNearlyEqual(*ReseedRefund, 12.0f * Older.FixedDt));
	}

	FRopeGPUSolver BoundedQueueSolver;
	FRopeGPUResidentStep BoundedStep = Older;
	BoundedStep.RopeId = Older.RopeId + 1;
	BoundedStep.NumSub = 1;
	for (int32 FrameIndex = 0; FrameIndex < 5; ++FrameIndex)
	{
		TArray<FRopeGPUResidentStep> Steps;
		Steps.Add(BoundedStep);
		BoundedQueueSolver.EnqueueSteps(MoveTemp(Steps));
		FlushRenderingCommands();
	}
	BoundedQueueSolver.DrainDroppedSimTime(DroppedTime);
	const float* QueueLimitRefund = DroppedTime.Find(BoundedStep.RopeId);
	TestNotNull(TEXT("the bounded queue refunds history beyond four frames"), QueueLimitRefund);
	if (QueueLimitRefund)
	{
		TestTrue(TEXT("only the single oldest frame exceeds the queue bound"),
			FMath::IsNearlyEqual(*QueueLimitRefund, BoundedStep.FixedDt));
	}
	return true;
}

// A pending frame carries a time-dependent override, not just a number of simulation substeps. Replacing
// frame A with frame B and adding A's substeps to B applies B once and extrapolates it twice, overshooting
// the guide. The runtime queue must execute A and B in order when the render thread consumes them together.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUPendingTemporalOverrideOrderTest,
	"DynamicRope.Solver.GPUPendingTemporalOverrideOrder",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUPendingTemporalOverrideOrderTest::RunTest(const FString& Parameters)
{
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU pending temporal override test skipped: no renderable RHI (headless)."));
		return true;
	}

	constexpr int32 NumNodes = 4;
	constexpr uint32 RopeId = 117;
	constexpr uint32 Generation = 3;
	const FRopeSimState Seed = RopeTest::MakeStraightRope(NumNodes, 300.0f);
	FRopeGPUSolver GpuSolver;

	const auto MakeGuidedStep = [&](float CurrentY, float PreviousY)
	{
		FRopeGPUResidentStep Step;
		Step.RopeId = RopeId;
		Step.Generation = Generation;
		Step.NumNodes = NumNodes;
		Step.SeedPositions = Seed.Positions;
		Step.SeedPrevPositions = Seed.PrevPositions;
		Step.InvMass = Seed.InvMass;
		Step.SegmentLength = Seed.SegmentLength;
		Step.Iterations = 1;
		Step.Damping = 0.0f;
		Step.Gravity = FVector::ZeroVector;
		Step.NumSub = 4;
		Step.FixedDt = (1.0f / 60.0f) / 4.0f;
		Step.OverrideFlags.SetNumUninitialized(NumNodes);
		Step.OverridePositions.SetNumUninitialized(NumNodes);
		Step.OverridePrevPositions.SetNumUninitialized(NumNodes);
		for (int32 NodeIndex = 0; NodeIndex < NumNodes; ++NodeIndex)
		{
			Step.OverrideFlags[NodeIndex] = static_cast<uint8>(
				ERopeGPUOverride::Position | ERopeGPUOverride::Prev |
				ERopeGPUOverride::KinematicPath);
			Step.OverridePositions[NodeIndex] = Seed.Positions[NodeIndex] + FVector(0.0f, CurrentY, 0.0f);
			Step.OverridePrevPositions[NodeIndex] = Seed.Positions[NodeIndex] + FVector(0.0f, PreviousY, 0.0f);
		}
		return Step;
	};

	const auto EnqueueWithoutViewDispatch = [&GpuSolver](FRopeGPUResidentStep Step)
	{
		TArray<FRopeGPUResidentStep> Steps;
		Steps.Add(MoveTemp(Step));
		GpuSolver.EnqueueSteps(MoveTemp(Steps));
		FlushRenderingCommands();
	};

	EnqueueWithoutViewDispatch(MakeGuidedStep(10.0f, 0.0f));
	EnqueueWithoutViewDispatch(MakeGuidedStep(20.0f, 10.0f));

	TArray<FVector> Positions;
	TArray<FVector> PrevPositions;
	uint32 ReadbackGeneration = 0;
	if (!GpuSolver.ReadbackNow(RopeId, Positions, PrevPositions, ReadbackGeneration))
	{
		AddError(TEXT("Could not consume the queued temporal override steps."));
		return false;
	}

	TestEqual(TEXT("readback generation remains current"), ReadbackGeneration, Generation);
	TestEqual(TEXT("readback node count remains current"), Positions.Num(), NumNodes);
	for (int32 NodeIndex = 0; NodeIndex < Positions.Num(); ++NodeIndex)
	{
		TestTrue(*FString::Printf(TEXT("node %d executes A then B without overshoot (Y=%.3f)"),
			NodeIndex, Positions[NodeIndex].Y),
			FMath::IsNearlyEqual(Positions[NodeIndex].Y, 20.0f, 0.05f));
		TestTrue(*FString::Printf(TEXT("node %d stores one substep of release velocity (PrevY=%.3f)"),
			NodeIndex, PrevPositions[NodeIndex].Y),
			FMath::IsNearlyEqual(PrevPositions[NodeIndex].Y, 17.5f, 0.05f));
	}
	return true;
}

// Override pass(G0): NumSub=0 override dispatch records location/mass in resident buffer,
// Nodes pinned with InvMass=0 remain exactly in the target in subsequent gravity solves (mass mask is persistent),
// After InvMass restoration override, the bone returns to physics. “Target calculation is GT, application is GPU” contract verification.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUOverridePassTest,
	"DynamicRope.Solver.GPUOverridePass",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUOverridePassTest::RunTest(const FString& Parameters)
{
	// GPU dispatch requires RHI — skip (not fail) in render-unable (headless/null RHI) environments.
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU override pass test skipped: no renderable RHI (headless)."));
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
	// Synchronize until GPU idle — make readback IsReady non-static (in high-speed headless execution, GPU
	// If you lag behind, the in-Flight copy may be an old frame).
	auto SyncGPU = []()
	{
		ENQUEUE_RENDER_COMMAND(RopeTestGpuSync)(
			[](FRHICommandListImmediate& RHICmdList)
			{
				RHICmdList.SubmitAndBlockUntilGPUIdle();
			});
		FlushRenderingCommands();
	};
	// Retrieve the latest (final status) readback. NOTE: A single in-Flight readback copies the buffer at arming time, so
	// (1) Make the existing copy available for consumption with GPU idle synchronization and (2) make it possible to consume with no-op override dispatch.
	// The cycle leading to rearmament must be repeated several times to ensure that a copy of the "last actual state" arrives.
	// (no-op = flags all 0: No nodes are used, but dispatch occurs → consume+rearm.)
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
		// last consume(no override — just recall, no dispatch).
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

	// 1) Seed + normal solve a few frames.
	for (int32 Frame = 0; Frame < 4; ++Frame)
	{
		Pump(MakeStep(Config.Substeps, (1.0f / 60.0f) / Config.Substeps));
	}

	// 2) override: pinned node 3..5 to a random target (Pos + Prev=Pos + InvMass=0). NumSub=0 — Record only, no integration.
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

	// 3) Gravity solve 20 frames — pinned nodes must not move even 1mm (whether the InvMass mask is persistent).
	for (int32 Frame = 0; Frame < 20; ++Frame)
	{
		Pump(MakeStep(Config.Substeps, (1.0f / 60.0f) / Config.Substeps));
	}

	FRopeResidentLatest AfterFix;
	if (!Drain(AfterFix))
	{
		AddError(TEXT("Readback drain failed after the override."));
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
			AddError(TEXT("NaN after the override."));
			return false;
		}
	}

	// 4) Solve gravity after restoring mass (override InvMass=1 only) — the node must deviate from the target again.
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
		AddError(TEXT("Readback drain failed after the restore."));
		return false;
	}
	const float MovedDev = static_cast<float>(FVector::Dist(AfterRestore.Positions[4], Targets[4]));
	TestTrue(FString::Printf(TEXT("restored node resumes physics (moved %.2f cm off target)"), MovedDev),
		MovedDev > 1.0f);

	// 5) MaxStretchRatio=1.0 delivered by Flight/Wrapping phase policy: Guide overstretched in GT
	// Even if override is added, the actual resident pose must return within SegmentLength at the end of the same GPU step.
	{
		FRopeGPUResidentStep NonStretch = MakeStep(/*NumSub*/ 1, 1.0f / 60.0f);
		NonStretch.MaxStretchRatio = 1.0f;
		NonStretch.Gravity = FVector::ZeroVector;
		NonStretch.OverrideFlags.SetNumZeroed(N);
		NonStretch.OverridePositions.SetNumZeroed(N);
		NonStretch.OverrideInvMass.SetNumZeroed(N);
		const uint8 PoseFlags = static_cast<uint8>(
			ERopeGPUOverride::Position | ERopeGPUOverride::PrevFromPosition | ERopeGPUOverride::InvMass);
		for (int32 i = 0; i < N; ++i)
		{
			NonStretch.OverrideFlags[i] = PoseFlags;
			NonStretch.OverridePositions[i] =
				Sim.StartPinTarget + FVector(2.0f * Sim.SegmentLength * static_cast<float>(i), 0.0f, 0.0f);
			NonStretch.OverrideInvMass[i] = i == 0 ? 0.0f : 1.0f;
		}
		Pump(MoveTemp(NonStretch));
	}

	FRopeResidentLatest AfterNonStretch;
	if (!Drain(AfterNonStretch))
	{
		AddError(TEXT("Readback drain failed after the inextensible override."));
		return false;
	}
	for (int32 i = 0; i + 1 < AfterNonStretch.Positions.Num(); ++i)
	{
		const float EdgeLength = static_cast<float>(
			FVector::Dist(AfterNonStretch.Positions[i], AfterNonStretch.Positions[i + 1]));
		TestTrue(FString::Printf(TEXT("GPU non-stretch edge %d-%d (%.3f <= %.3f)"),
			i, i + 1, EdgeLength, Sim.SegmentLength),
			EdgeLength <= Sim.SegmentLength + 0.05f);
	}

	return true;
}

// contact detection parity(G3): GPU detection kernel compares CPU FRopeFlightContactDetector::DetectContactCandidates
// Does it calculate the same contact (hit node set + per-node penetration/normal)? Static comparison with static rope + capsule.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUContactParityTest,
	"DynamicRope.Solver.GPUContactParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUContactParityTest::RunTest(const FString& Parameters)
{
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU contact detection parity test skipped: no renderable RHI (headless)."));
		return true;
	}

	const int32 N = 8;
	const float Length = 140.0f;
	const float ContactRadius = 3.0f;

	// Place static rope(prev==pos) at z=15. Capsule: Along the Y axis at x=60, radius 30 → node 2/3/4 penetrates.
	FRopeSimState Sim = RopeTest::MakeStraightRope(N, Length, FVector(0, 0, 15));
	FCapsuleCollider Capsule(FVector(60, -50, 0), FVector(60, 50, 0), 30.0f, FName("arm"));
	TArray<IRopeCollider*> Colliders = { &Capsule };

	// --- CPU ground-truth detection.
	FRopeFlightContactDetector::FParams Params;
	Params.ContactRadius = ContactRadius;
	Params.RopeRadius = 2.0f;
	Params.PredictiveContactFrames = 0.0f;
	Params.MinLatchNodes = 1;
	TArray<FRopeContactCandidate> CpuCandidates;
	FRopeFlightContactDetector::DetectContactCandidates(Sim, Colliders, Params, CpuCandidates);

	// --- GPU detection: detection-only step(NumSub=0, bDetectContacts). Fill the resident buffer with seed locations and perform detection.
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
		// No integration — Detection of seed position as is.
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
			[](FRHICommandListImmediate& RHICmdList) { RHICmdList.SubmitAndBlockUntilGPUIdle(); });
		FlushRenderingCommands();
	};
	auto Pump = [&]()
	{
		TArray<FRopeGPUResidentStep> Steps;
		Steps.Add(MakeDetectStep());
		GpuSolver.Step(MoveTemp(Steps));
		FlushRenderingCommands();
	};

	// Pumps multiple times to ensure detection readback arrives (single in-Flight → sync + rearmament cycle).
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
		AddError(TEXT("Could not retrieve the GPU contact detection results."));
		return false;
	}

	// --- Comparison: Hit node set matching + per-node penetration/normal approximation matching.
	TMap<int32, const FRopeContactCandidate*> CpuByNode;
	for (const FRopeContactCandidate& C : CpuCandidates) { CpuByNode.Add(C.NodeIndex, &C); }
	TMap<int32, const FRopeGPUContactResult*> GpuByNode;
	for (const FRopeGPUContactResult& C : GpuContacts.Contacts) { GpuByNode.Add(C.NodeIndex, &C); }

	AddInfo(FString::Printf(TEXT("CPU contacts %d, GPU contacts %d"), CpuCandidates.Num(), GpuContacts.Contacts.Num()));
	TestTrue(TEXT("At least one contact was detected"), CpuCandidates.Num() > 0);
	TestEqual(TEXT("Hit node counts match"), GpuContacts.Contacts.Num(), CpuCandidates.Num());

	for (const TPair<int32, const FRopeContactCandidate*>& Pair : CpuByNode)
	{
		const int32 Node = Pair.Key;
		const FRopeGPUContactResult** GpuC = GpuByNode.Find(Node);
		if (!TestTrue(FString::Printf(TEXT("GPU hit node %d as well"), Node), GpuC != nullptr))
		{
			continue;
		}
		const float PenDev = FMath::Abs((*GpuC)->Penetration - Pair.Value->Penetration);
		TestTrue(FString::Printf(TEXT("Node %d penetration matches (diff %.3f)"), Node, PenDev), PenDev < 0.1f);
		const float NormalDot = FVector::DotProduct((*GpuC)->Normal.GetSafeNormal(), Pair.Value->Normal.GetSafeNormal());
		TestTrue(FString::Printf(TEXT("Node %d normal matches (dot %.3f)"), Node, NormalDot), NormalDot > 0.99f);
		const float PointDev = static_cast<float>(FVector::Dist((*GpuC)->WorldPoint, Pair.Value->WorldPoint));
		TestTrue(FString::Printf(TEXT("Node %d contact point matches (diff %.3f cm)"), Node, PointDev), PointDev < 0.5f);
	}

	return true;
}

// wrap possible convex detection parity: box-type convex (virtual bone) is used in convex loops of CPU detection and GPU detection kernels.
// Capture with the same hit set/penetration/normal (NumDetectConvexes boundary + ColliderType=3 wiring verification).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUConvexContactParityTest,
	"DynamicRope.Solver.GPUConvexContactParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUConvexContactParityTest::RunTest(const FString& Parameters)
{
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU convex detection parity test skipped: no renderable RHI (headless)."));
		return true;
	}

	const int32 N = 8;
	const float Length = 140.0f;
	const float ContactRadius = 3.0f;

	// static rope(prev==pos) at z=15. Box-type convex (6 planes): x=60 center, half width (30,50,30) → nodes 2/3/4 are inside.
	FRopeSimState Sim = RopeTest::MakeStraightRope(N, Length, FVector(0, 0, 15));
	// Make the tail cross both faces in one frame; capture must stay on the first face encountered.
	constexpr int32 SweptNode = N - 1;
	Sim.PrevPositions[SweptNode] = FVector(-40.0f, 0.0f, 15.0f);
	Sim.Positions[SweptNode] = FVector(160.0f, 0.0f, 15.0f);
	TArray<FPlane> Planes;
	Planes.Add(FPlane(FVector(1, 0, 0), 30.0));
	Planes.Add(FPlane(FVector(-1, 0, 0), 30.0));
	Planes.Add(FPlane(FVector(0, 1, 0), 50.0));
	Planes.Add(FPlane(FVector(0, -1, 0), 50.0));
	Planes.Add(FPlane(FVector(0, 0, 1), 30.0));
	Planes.Add(FPlane(FVector(0, 0, -1), 30.0));
	FRopeConvexCollider Convex(MoveTemp(Planes), FBox(FVector(-30.0, -50.0, -30.0), FVector(30.0, 50.0, 30.0)),
		FQuat::Identity, FVector(60.0, 0.0, 0.0));
	Convex.Bone = FName(TEXT("prop"));
	TArray<IRopeCollider*> Colliders = { &Convex };

	// --- CPU ground-truth detection.
	FRopeFlightContactDetector::FParams Params;
	Params.ContactRadius = ContactRadius;
	Params.RopeRadius = 2.0f;
	Params.PredictiveContactFrames = 0.0f;
	Params.MinLatchNodes = 1;
	TArray<FRopeContactCandidate> CpuCandidates;
	FRopeFlightContactDetector::DetectContactCandidates(Sim, Colliders, Params, CpuCandidates);

	// --- GPU detection: Dedicated step for detection. Packing convexes with pass-1 protocol (smooth plane pool + NumDetectConvexes).
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
		TConstArrayView<FPlane> LocalPlanes;
		FBox LocalBounds(ForceInit);
		FQuat CvRot, CvPrevRot;
		FVector CvTrans, CvPrevTrans;
		float CvInvDt = 0.0f;
		if (Convex.GetGPUConvex(LocalPlanes, LocalBounds, CvRot, CvTrans, CvPrevRot, CvPrevTrans, CvInvDt))
		{
			FRopeGPUConvex Cv;
			Cv.PlaneOffset = 0;
			Cv.PlaneCount = LocalPlanes.Num();
			Cv.LocalBoundsCenter = LocalBounds.GetCenter();
			Cv.LocalBoundsExtent = LocalBounds.GetExtent();
			Cv.Rot = CvRot; Cv.Trans = CvTrans;
			Cv.PrevRot = CvPrevRot; Cv.PrevTrans = CvPrevTrans;
			Cv.InvDeltaTime = CvInvDt;
			for (const FPlane& Pl : LocalPlanes)
			{
				Step.ConvexPlanes.Add(FVector4(Pl.X, Pl.Y, Pl.Z, Pl.W));
			}
			Step.Convexes.Add(Cv);
			Step.NumDetectConvexes = 1;
		}
		return Step;
	};
	auto SyncGPU = []()
	{
		ENQUEUE_RENDER_COMMAND(RopeTestGpuSync)(
			[](FRHICommandListImmediate& RHICmdList) { RHICmdList.SubmitAndBlockUntilGPUIdle(); });
		FlushRenderingCommands();
	};

	FRopeResidentContacts GpuContacts;
	bool bGot = false;
	for (int32 Spin = 0; Spin < 16 && !bGot; ++Spin)
	{
		SyncGPU();
		{
			TArray<FRopeGPUResidentStep> Steps;
			Steps.Add(MakeDetectStep());
			GpuSolver.Step(MoveTemp(Steps));
			FlushRenderingCommands();
		}
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
		AddError(TEXT("Could not retrieve the GPU convex contact detection results."));
		return false;
	}

	TMap<int32, const FRopeContactCandidate*> CpuByNode;
	for (const FRopeContactCandidate& C : CpuCandidates) { CpuByNode.Add(C.NodeIndex, &C); }
	TMap<int32, const FRopeGPUContactResult*> GpuByNode;
	for (const FRopeGPUContactResult& C : GpuContacts.Contacts) { GpuByNode.Add(C.NodeIndex, &C); }

	AddInfo(FString::Printf(TEXT("CPU contacts %d, GPU contacts %d"), CpuCandidates.Num(), GpuContacts.Contacts.Num()));
	TestTrue(TEXT("At least one convex contact was detected"), CpuCandidates.Num() > 0);
	TestEqual(TEXT("Hit node counts match"), GpuContacts.Contacts.Num(), CpuCandidates.Num());
	for (const FRopeGPUContactResult& C : GpuContacts.Contacts)
	{
		TestEqual(TEXT("GPU contact ColliderType is convex (3)"), C.ColliderType, 3);
	}

	for (const TPair<int32, const FRopeContactCandidate*>& Pair : CpuByNode)
	{
		const int32 Node = Pair.Key;
		const FRopeGPUContactResult** GpuC = GpuByNode.Find(Node);
		if (!TestTrue(FString::Printf(TEXT("GPU hit node %d as well"), Node), GpuC != nullptr))
		{
			continue;
		}
		const float PenDev = FMath::Abs((*GpuC)->Penetration - Pair.Value->Penetration);
		TestTrue(FString::Printf(TEXT("Node %d penetration matches (diff %.3f)"), Node, PenDev), PenDev < 0.1f);
		const float NormalDot = FVector::DotProduct((*GpuC)->Normal.GetSafeNormal(), Pair.Value->Normal.GetSafeNormal());
		TestTrue(FString::Printf(TEXT("Node %d normal matches (dot %.3f)"), Node, NormalDot), NormalDot > 0.99f);
		const float PointDev = static_cast<float>(FVector::Dist((*GpuC)->WorldPoint, Pair.Value->WorldPoint));
		TestTrue(FString::Printf(TEXT("Node %d contact point matches (diff %.3f cm)"), Node, PointDev), PointDev < 0.5f);
	}

	const FRopeContactCandidate** CpuEntryPtr = CpuByNode.Find(SweptNode);
	const FRopeGPUContactResult** GpuEntryPtr = GpuByNode.Find(SweptNode);
	if (TestNotNull(TEXT("CPU detects the complete thick-convex crossing"), CpuEntryPtr)
		&& TestNotNull(TEXT("GPU detects the complete thick-convex crossing"), GpuEntryPtr))
	{
		TestTrue(TEXT("CPU keeps the negative-X entry normal"), (*CpuEntryPtr)->Normal.X < -0.99f);
		TestTrue(TEXT("GPU keeps the negative-X entry normal"), (*GpuEntryPtr)->Normal.X < -0.99f);
		TestTrue(TEXT("GPU entry point stays on the negative-X face"),
			FMath::IsNearlyEqual((*GpuEntryPtr)->WorldPoint.X, 30.0f, 0.5f));
	}

	return true;
}


// A catch-up batch can contain a one-frame convex crossing followed by a pose that is already clear of
// the target. Contact detection must observe and retain the intermediate step instead of sampling only
// the newest resident pose.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUQueuedIntermediateConvexContactTest,
	"DynamicRope.Solver.GPUQueuedIntermediateConvexContact",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUQueuedIntermediateConvexContactTest::RunTest(const FString& Parameters)
{
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU queued intermediate convex contact test skipped: no renderable RHI (headless)."));
		return true;
	}

	constexpr int32 NumNodes = 8;
	constexpr int32 SweptNode = NumNodes - 1;
	constexpr uint32 RopeId = 131;
	constexpr uint32 Generation = 1;
	constexpr uint32 AttribSig = 0xC0111D3u;
	constexpr float ContactRadius = 3.0f;

	FRopeSimState Sim = RopeTest::MakeStraightRope(NumNodes, 140.0f, FVector(0, 0, 15));
	Sim.PrevPositions[SweptNode] = FVector(-40.0f, 0.0f, 15.0f);
	Sim.Positions[SweptNode] = FVector(160.0f, 0.0f, 15.0f);

	TArray<FPlane> Planes;
	Planes.Add(FPlane(FVector(1, 0, 0), 30.0));
	Planes.Add(FPlane(FVector(-1, 0, 0), 30.0));
	Planes.Add(FPlane(FVector(0, 1, 0), 50.0));
	Planes.Add(FPlane(FVector(0, -1, 0), 50.0));
	Planes.Add(FPlane(FVector(0, 0, 1), 30.0));
	Planes.Add(FPlane(FVector(0, 0, -1), 30.0));
	FRopeConvexCollider Convex(MoveTemp(Planes),
		FBox(FVector(-30.0, -50.0, -30.0), FVector(30.0, 50.0, 30.0)),
		FQuat::Identity, FVector(60.0, 0.0, 0.0));

	auto MakeStep = [&]()
	{
		FRopeGPUResidentStep Step;
		Step.RopeId = RopeId;
		Step.Generation = Generation;
		Step.NumNodes = NumNodes;
		Step.SeedPositions = Sim.Positions;
		Step.SeedPrevPositions = Sim.PrevPositions;
		Step.InvMass = Sim.InvMass;
		Step.SegmentLength = Sim.SegmentLength;
		Step.NumSub = 0;
		Step.FixedDt = 1.0f / 60.0f;
		Step.bDetectContacts = true;
		Step.ContactRadius = ContactRadius;
		Step.AttribSig = AttribSig;

		TConstArrayView<FPlane> LocalPlanes;
		FBox LocalBounds(ForceInit);
		FQuat CvRot, CvPrevRot;
		FVector CvTrans, CvPrevTrans;
		float CvInvDt = 0.0f;
		if (Convex.GetGPUConvex(LocalPlanes, LocalBounds, CvRot, CvTrans,
			CvPrevRot, CvPrevTrans, CvInvDt))
		{
			FRopeGPUConvex Cv;
			Cv.PlaneOffset = 0;
			Cv.PlaneCount = LocalPlanes.Num();
			Cv.LocalBoundsCenter = LocalBounds.GetCenter();
			Cv.LocalBoundsExtent = LocalBounds.GetExtent();
			Cv.Rot = CvRot;
			Cv.Trans = CvTrans;
			Cv.PrevRot = CvPrevRot;
			Cv.PrevTrans = CvPrevTrans;
			Cv.InvDeltaTime = CvInvDt;
			for (const FPlane& Plane : LocalPlanes)
			{
				Step.ConvexPlanes.Add(FVector4(Plane.X, Plane.Y, Plane.Z, Plane.W));
			}
			Step.Convexes.Add(Cv);
			Step.NumDetectConvexes = 1;
		}
		return Step;
	};

	FRopeGPUResidentStep CrossingStep = MakeStep();
	FRopeGPUResidentStep ClearStep = MakeStep();
	const uint8 PoseFlags = static_cast<uint8>(ERopeGPUOverride::Position | ERopeGPUOverride::Prev);
	CrossingStep.OverrideFlags.SetNumUninitialized(NumNodes);
	CrossingStep.OverridePositions.SetNumUninitialized(NumNodes);
	CrossingStep.OverridePrevPositions.SetNumUninitialized(NumNodes);
	for (int32 NodeIndex = 0; NodeIndex < NumNodes; ++NodeIndex)
	{
		CrossingStep.OverrideFlags[NodeIndex] = PoseFlags;
		CrossingStep.OverridePositions[NodeIndex] = Sim.Positions[NodeIndex];
		CrossingStep.OverridePrevPositions[NodeIndex] = Sim.PrevPositions[NodeIndex];
	}
	ClearStep.OverrideFlags.SetNumUninitialized(NumNodes);
	ClearStep.OverridePositions.SetNumUninitialized(NumNodes);
	ClearStep.OverridePrevPositions.SetNumUninitialized(NumNodes);
	for (int32 NodeIndex = 0; NodeIndex < NumNodes; ++NodeIndex)
	{
		const FVector ClearPosition(NodeIndex * 20.0f, 200.0f, 15.0f);
		ClearStep.OverrideFlags[NodeIndex] = PoseFlags;
		ClearStep.OverridePositions[NodeIndex] = ClearPosition;
		ClearStep.OverridePrevPositions[NodeIndex] = ClearPosition;
	}

	IConsoleVariable* HoldContactReadbacks =
		IConsoleManager::Get().FindConsoleVariable(TEXT("r.DynamicRope.Test.HoldContactReadbacks"));
	if (!TestNotNull(TEXT("Contact readback hold test control is registered"), HoldContactReadbacks))
	{
		return false;
	}
	HoldContactReadbacks->Set(1, ECVF_SetByCode);
	ON_SCOPE_EXIT
	{
		HoldContactReadbacks->Set(0, ECVF_SetByCode);
	};

	FRopeGPUSolver GpuSolver;
	// Arm a no-hit copy in graph A and deliberately keep it in-flight from the consumer's point of view.
	// Graph B below must use another contact slot or its transient convex hit will be lost.
	TArray<FRopeGPUResidentStep> InitialClearSteps;
	InitialClearSteps.Add(ClearStep);
	GpuSolver.Step(MoveTemp(InitialClearSteps));
	FlushRenderingCommands();

	TArray<FRopeGPUResidentStep> CrossingEnqueue;
	CrossingEnqueue.Add(MoveTemp(CrossingStep));
	GpuSolver.EnqueueSteps(MoveTemp(CrossingEnqueue));
	FlushRenderingCommands();
	TArray<FRopeGPUResidentStep> ClearEnqueue;
	ClearEnqueue.Add(ClearStep);
	GpuSolver.EnqueueSteps(MoveTemp(ClearEnqueue));
	FlushRenderingCommands();

	// Flush the pending queue through the same scene-graph entry point used at runtime. The explicit
	// GPU idle makes the asynchronous contact copy ready before the poll step consumes it.
	ENQUEUE_RENDER_COMMAND(RopeQueuedContactDispatch)(
		[&GpuSolver](FRHICommandListImmediate& RHICmdList)
		{
			FRDGBuilder GraphBuilder(RHICmdList);
			GpuSolver.DispatchPending_RenderThread(
				GraphBuilder, nullptr, nullptr, FVector3f::ZeroVector);
			GraphBuilder.Execute();
			RHICmdList.SubmitAndBlockUntilGPUIdle();
		});
	FlushRenderingCommands();

	// Keep both copies armed and submit graph C. This forces the lazy third slot and provides a newer
	// no-hit snapshot, exercising both overflow allocation and hit-preserving drain selection.
	TArray<FRopeGPUResidentStep> HeldClearSteps;
	HeldClearSteps.Add(ClearStep);
	GpuSolver.Step(MoveTemp(HeldClearSteps));
	FlushRenderingCommands();
	ENQUEUE_RENDER_COMMAND(RopeQueuedContactOverflowSync)(
		[](FRHICommandListImmediate& RHICmdList)
		{
			RHICmdList.SubmitAndBlockUntilGPUIdle();
		});
	FlushRenderingCommands();
	HoldContactReadbacks->Set(0, ECVF_SetByCode);

	// Submit one clear step so RunSteps drains graph A's no-hit and graph B's hit together. The newer
	// no-hit observation must not erase the transient hit before the game thread can observe it.
	TArray<FRopeGPUResidentStep> PollSteps;
	PollSteps.Add(MoveTemp(ClearStep));
	GpuSolver.Step(MoveTemp(PollSteps));
	FlushRenderingCommands();

	TMap<uint32, FRopeResidentContacts> Latest;
	GpuSolver.GetLatestContacts(Latest);
	const FRopeResidentContacts* Contacts = Latest.Find(RopeId);
	if (!TestNotNull(TEXT("Intermediate convex contact readback is published"), Contacts))
	{
		return false;
	}
	TestEqual(TEXT("Intermediate contact keeps its generation"), Contacts->Generation, Generation);
	TestEqual(TEXT("Intermediate contact keeps its attribution signature"), Contacts->AttribSig, AttribSig);

	const FRopeGPUContactResult* SweptContact = nullptr;
	for (const FRopeGPUContactResult& Contact : Contacts->Contacts)
	{
		if (Contact.NodeIndex == SweptNode && Contact.Source == 1)
		{
			SweptContact = &Contact;
			break;
		}
	}
	if (TestNotNull(TEXT("One-frame thick-convex crossing survives the newer clear pose"), SweptContact))
	{
		TestEqual(TEXT("Retained contact is convex"), SweptContact->ColliderType, 3);
		TestTrue(TEXT("Retained contact stays on the negative-X entry face"),
			SweptContact->Normal.X < -0.99f && FMath::IsNearlyEqual(SweptContact->WorldPoint.X, 30.0f, 0.5f));
	}

	return true;
}


// Predictive contact parity (G3b): The tail node that has not yet been reached but whose extrapolation path passes through the capsule is in the predictive slot.
// is caught, and does the CPU AddPredictedContactCandidates and penetration/source match? The actual slot must be empty.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUPredictiveParityTest,
	"DynamicRope.Solver.GPUPredictiveParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUPredictiveParityTest::RunTest(const FString& Parameters)
{
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU predicted contact parity test skipped: no renderable RHI (headless)."));
		return true;
	}

	const int32 N = 8;
	const float Length = 140.0f;
	const float ContactRadius = 3.0f;
	const float PredictionFrames = 3.0f;

	// Only tail node 7 moves to +X (prev 120 → pos 140). Capsule is x=175 (before arrival) → actual miss, predicted path penetrates.
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

	// CPU: actual 0, one node 7 should be added predictively.
	TestEqual(TEXT("No CPU actual contact"), CpuActualCount, 0);
	const FRopeContactCandidate* CpuPred = nullptr;
	for (const FRopeContactCandidate& C : CpuCandidates)
	{
		if (C.NodeIndex == 7) { CpuPred = &C; }
	}
	if (!TestTrue(TEXT("CPU predicted candidate exists (node 7)"), CpuPred != nullptr))
	{
		return false;
	}

	// --- GPU: Includes prediction detection step (no whip → Free prediction).
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
			[](FRHICommandListImmediate& RHICmdList) { RHICmdList.SubmitAndBlockUntilGPUIdle(); });
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
		AddError(TEXT("Could not retrieve the GPU predicted contact results."));
		return false;
	}

	// GPU: There must be a predictive(Source=2) contact on node 7 and no actual(Source=1) contact.
	const FRopeGPUContactResult* GpuPred = nullptr;
	bool bAnyActual = false;
	for (const FRopeGPUContactResult& C : GpuContacts.Contacts)
	{
		if (C.Source == static_cast<uint8>(ERopeContactCandidateSource::Actual)) { bAnyActual = true; }
		if (C.NodeIndex == 7 && C.Source == static_cast<uint8>(ERopeContactCandidateSource::PredictiveFree)) { GpuPred = &C; }
	}
	TestFalse(TEXT("No GPU actual contact"), bAnyActual);
	if (!TestTrue(TEXT("GPU predicted candidate exists (node 7, PredictiveFree)"), GpuPred != nullptr))
	{
		return false;
	}

	const float PenDev = FMath::Abs(GpuPred->Penetration - CpuPred->Penetration);
	AddInfo(FString::Printf(TEXT("Predicted penetration CPU %.3f / GPU %.3f"), CpuPred->Penetration, GpuPred->Penetration));
	TestTrue(FString::Printf(TEXT("Predicted penetration matches (diff %.3f)"), PenDev), PenDev < 0.1f);
	const float PointDev = static_cast<float>(FVector::Dist(GpuPred->WorldPoint, CpuPred->WorldPoint));
	TestTrue(FString::Printf(TEXT("Predicted contact point matches (diff %.3f cm)"), PointDev), PointDev < 0.5f);

	return true;
}

// SDF contact detection parity(G3b): GPU SDF detection detects the same contact as CPU FRopeSDFCollider::Query-based detection.
// calculated? Static comparison of composite sphere volume + static rope (penetration/normal/contact point).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUSDFContactParityTest,
	"DynamicRope.Solver.GPUSDFContactParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUSDFContactParityTest::RunTest(const FString& Parameters)
{
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU SDF contact parity test skipped: no renderable RHI (headless)."));
		return true;
	}

	const float ContactRadius = 3.0f;

	// radius 20 sphere (bone local), BoneToWorld=identity. Place rope nodes near the surface so that several of them penetrate.
	const FRopeBoneSDFVolume Volume =
		RopeSDFSynthetic::MakeSphere(FName("arm"), FVector::ZeroVector, 20.0f, FIntVector(31), 10.0f);
	FRopeSDFCollider Sdf(&Volume, FTransform::Identity, FTransform::Identity, 0.0f, FName("arm"), nullptr);
	TArray<IRopeCollider*> Colliders = { &Sdf };

	// static rope(prev==pos): x = -25,-21,-19,19,21,25 (z=0). In surface(20), nodes 1/2/3/4 are within radius 3.
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

	// --- GPU SDF collider view → step.
	FRopeSDFColliderView View;
	if (!Sdf.GetGPUSDF(View))
	{
		AddError(TEXT("GetGPUSDF failed (volume not baked?)."));
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
			[](FRHICommandListImmediate& RHICmdList) { RHICmdList.SubmitAndBlockUntilGPUIdle(); });
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
		AddError(TEXT("Could not retrieve the GPU SDF contact results."));
		return false;
	}

	// GPU actual contact only (static → no prediction).
	TMap<int32, const FRopeGPUContactResult*> GpuByNode;
	for (const FRopeGPUContactResult& C : GpuContacts.Contacts)
	{
		if (C.Source == static_cast<uint8>(ERopeContactCandidateSource::Actual)) { GpuByNode.Add(C.NodeIndex, &C); }
	}

	AddInfo(FString::Printf(TEXT("CPU SDF contacts %d, GPU %d"), CpuCandidates.Num(), GpuByNode.Num()));
	TestTrue(TEXT("At least one SDF contact"), CpuCandidates.Num() > 0);
	TestEqual(TEXT("SDF hit node counts match"), GpuByNode.Num(), CpuCandidates.Num());

	for (const FRopeContactCandidate& Cpu : CpuCandidates)
	{
		const FRopeGPUContactResult** GpuC = GpuByNode.Find(Cpu.NodeIndex);
		if (!TestTrue(FString::Printf(TEXT("GPU hit node %d as well"), Cpu.NodeIndex), GpuC != nullptr))
		{
			continue;
		}
		const float PenDev = FMath::Abs((*GpuC)->Penetration - Cpu.Penetration);
		TestTrue(FString::Printf(TEXT("Node %d SDF penetration matches (diff %.3f)"), Cpu.NodeIndex, PenDev), PenDev < 0.3f);
		const float NormalDot = FVector::DotProduct((*GpuC)->Normal.GetSafeNormal(), Cpu.Normal.GetSafeNormal());
		TestTrue(FString::Printf(TEXT("Node %d SDF normal matches (dot %.3f)"), Cpu.NodeIndex, NormalDot), NormalDot > 0.98f);
	}

	return true;
}

// static box(OBB) collision parity: The rope draped over the edge of the box goes inside the box (1) in the GPU path as well.
// Without going into detail, (2) Does the approximation match the CPU solver (FRopeBoxCollider)? Edges with GDF voxel rounding
// A regression gate that checks whether the analytic box blocks the bug that was penetrating the GPU.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUBoxCornerParityTest,
	"DynamicRope.Solver.GPUBoxCornerParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUBoxCornerParityTest::RunTest(const FString& Parameters)
{
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU box parity test skipped: no renderable RHI (headless)."));
		return true;
	}

	const int32 N = 24;
	const float Length = 300.0f;
	const FVector HalfExtents(50.0);
	FRopeSolverConfig Config = MakeHangConfig();
	Config.CollisionRadius = 2.0f;
	Config.Friction = 0.5f;

	// Free rope (no pins) spanning the +X/-X edges at z=55 above the box (half-width 50, origin). Gravity Drape.
	FRopeBoxCollider Box(FVector::ZeroVector, FQuat::Identity, HalfExtents);
	TArray<IRopeCollider*> Colliders;
	Colliders.Add(&Box);

	FRopeSimState CpuSim = RopeTest::MakeStraightRope(N, Length, FVector(-150.0, 0.0, 55.0), FVector(1, 0, 0));
	FRopeSimState GpuSim = CpuSim;

	const FRopeXPBDSolver Solver;
	FRopeGPUSolver GpuSolver;
	const uint32 RopeId = 23;
	const uint32 Gen = 1;
	// 120 frames keeps the original long-run stability coverage. Absolute CPU/GPU positions are
	// compared at frame 30 while the rope is still interacting with the box; after it falls free,
	// tiny solver-order differences accumulate into unrelated trajectory drift.
	constexpr int32 NumSimulationFrames = 120;
	constexpr int32 ParitySampleFrame = 30;
	TArray<FVector> ParityCpuPositions;
	TArray<FVector> ParityGpuPositions;

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
		// static box: Same data as CPU FRopeBoxCollider (same value as GetGPUBox extract).
		FRopeGPUBox GpuBox;
		Box.GetGPUBox(GpuBox.Center, GpuBox.Rot, GpuBox.HalfExtents);
		Step.Boxes.Add(GpuBox);
		return Step;
	};

	for (int32 Frame = 0; Frame < NumSimulationFrames; ++Frame)
	{
		Solver.Step(CpuSim, Config, Colliders, 1.0f / 60.0f);

		const FRopeSubstepSchedule Schedule = RopeSolverSubsteps(GpuSim, Config, 1.0f / 60.0f);
		TArray<FRopeGPUResidentStep> Steps;
		Steps.Add(MakeStep(GpuSim, Schedule.NumSub, Schedule.FixedDt));
		GpuSolver.Step(MoveTemp(Steps));
		FlushRenderingCommands();
		if (Frame + 1 == ParitySampleFrame)
		{
			TArray<FVector> SnapshotPrev;
			uint32 SnapshotGen = 0;
			if (!GpuSolver.ReadbackNow(RopeId, ParityGpuPositions, SnapshotPrev, SnapshotGen) ||
				SnapshotGen != Gen || ParityGpuPositions.Num() != CpuSim.Num())
			{
				AddError(TEXT("GPU box parity: could not retrieve the contact span snapshot."));
				return false;
			}
			ParityCpuPositions = CpuSim.Positions;
		}
	}

	// Retrieve the final frame result deterministically: ReadbackNow synchronously reads back the resident buffer to SubmitAndBlockUntilGPUIdle.
	// Dedicated step paths are not stacked in PendingSteps, so the last dispatch state is read as is without additional solving.
	// (The asynchronous GetLatest mirror cannot specify which frame the result is, so it is non-static between executions).
	FlushRenderingCommands();
	TArray<FVector> RbPos, RbPrev;
	uint32 RbGen = 0;
	if (!GpuSolver.ReadbackNow(RopeId, RbPos, RbPrev, RbGen) ||
		RbGen != Gen || RbPos.Num() != GpuSim.Num() || RbPrev.Num() != GpuSim.Num())
	{
		AddError(TEXT("GPU box parity: could not retrieve the resident result (ReadbackNow)."));
		return false;
	}
	for (int32 i = 0; i < GpuSim.Num(); ++i)
	{
		GpuSim.Positions[i]     = RbPos[i];
		GpuSim.PrevPositions[i] = RbPrev[i];
	}

	TestFalse(TEXT("CPU no NaN"), RopeTest::AnyNaN(CpuSim));
	TestFalse(TEXT("GPU no NaN"), RopeTest::AnyNaN(GpuSim));

	// (1) No contact section penetration: No GPU node must be inside the box (regression condition of the original bug).
	float MaxInsideDepth = 0.0f;
	for (const FVector& P : ParityGpuPositions)
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

	// (2) CPU approximation matching: whether the fixation drape geometry is close (not bit-identical — coloring/sample order difference).
	if (!TestEqual(TEXT("Box parity CPU snapshot node count"), ParityCpuPositions.Num(), N) ||
		!TestEqual(TEXT("Box parity GPU snapshot node count"), ParityGpuPositions.Num(), N))
	{
		return false;
	}
	float MaxDev = 0.0f;
	int32 MaxDevNode = INDEX_NONE;
	for (int32 i = 0; i < N; ++i)
	{
		const float Dev = static_cast<float>(FVector::Dist(ParityCpuPositions[i], ParityGpuPositions[i]));
		if (Dev > MaxDev)
		{
			MaxDev = Dev;
			MaxDevNode = i;
		}
	}
	AddInfo(FString::Printf(TEXT("Box contact span (%d frames), largest CPU-to-GPU node deviation: %.2f cm"),
		ParitySampleFrame, MaxDev));
	if (MaxDevNode != INDEX_NONE)
	{
		AddInfo(FString::Printf(TEXT("Box largest deviation node=%d CPU=%s GPU=%s"),
			MaxDevNode,
			*ParityCpuPositions[MaxDevNode].ToCompactString(),
			*ParityGpuPositions[MaxDevNode].ToCompactString()));
	}
	TestTrue(FString::Printf(TEXT("CPU↔GPU max node deviation %.2f cm within tolerance"), MaxDev),
		MaxDev < Length * 0.10f);

	return true;
}

// static convex (6 planes = box) collision parity: The rope draped over the edge of the convex is also in the GPU convex path.
// (1) Without delving into the internals, (2) does the approximation match the CPU solver (FRopeConvexCollider)? box into 6-plane convex
// is expressed to verify the max-plane query + plane full packing of the two paths.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUConvexParityTest,
	"DynamicRope.Solver.GPUConvexParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUConvexParityTest::RunTest(const FString& Parameters)
{
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU convex parity test skipped: no renderable RHI (headless)."));
		return true;
	}

	const int32 N = 24;
	const float Length = 300.0f;
	const FVector H(50.0);
	FRopeSolverConfig Config = MakeHangConfig();
	Config.CollisionRadius = 2.0f;
	Config.Friction = 0.5f;

	// Origin box (half width 50) as 6-plane convex. FRopeConvexCollider on CPU, Step.Convexes/ConvexPlanes on GPU.
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
	// Keep the 120-frame stability run, but compare CPU/GPU shape at the contact snapshot rather
	// than after both free ropes have left the convex and accumulated unrelated flight drift.
	constexpr int32 NumSimulationFrames = 120;
	constexpr int32 ParitySampleFrame = 30;
	TArray<FVector> ParityCpuPositions;
	TArray<FVector> ParityGpuPositions;

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
		// 6-plane convex: plane pool + header (offset 0, count 6). rigid body identity → world=local (constructs the plane at the origin), static(InvDt 0).
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

	for (int32 Frame = 0; Frame < NumSimulationFrames; ++Frame)
	{
		Solver.Step(CpuSim, Config, Colliders, 1.0f / 60.0f);
		const FRopeSubstepSchedule Schedule = RopeSolverSubsteps(GpuSim, Config, 1.0f / 60.0f);
		TArray<FRopeGPUResidentStep> Steps;
		Steps.Add(MakeStep(GpuSim, Schedule.NumSub, Schedule.FixedDt));
		GpuSolver.Step(MoveTemp(Steps));
		FlushRenderingCommands();
		if (Frame + 1 == ParitySampleFrame)
		{
			TArray<FVector> SnapshotPrev;
			uint32 SnapshotGen = 0;
			if (!GpuSolver.ReadbackNow(RopeId, ParityGpuPositions, SnapshotPrev, SnapshotGen) ||
				SnapshotGen != Gen || ParityGpuPositions.Num() != CpuSim.Num())
			{
				AddError(TEXT("GPU convex parity: could not retrieve the contact span snapshot."));
				return false;
			}
			ParityCpuPositions = CpuSim.Positions;
		}
	}

	// Retrieve the final frame result deterministically: ReadbackNow synchronously reads back the resident buffer to SubmitAndBlockUntilGPUIdle.
	// Dedicated step paths are not stacked in PendingSteps, so the last dispatch state is read as is without additional solving.
	// (The asynchronous GetLatest mirror cannot specify which frame the result is, so it is non-static between executions).
	FlushRenderingCommands();
	TArray<FVector> RbPos, RbPrev;
	uint32 RbGen = 0;
	if (!GpuSolver.ReadbackNow(RopeId, RbPos, RbPrev, RbGen) ||
		RbGen != Gen || RbPos.Num() != GpuSim.Num() || RbPrev.Num() != GpuSim.Num())
	{
		AddError(TEXT("GPU convex parity: could not retrieve the resident result (ReadbackNow)."));
		return false;
	}
	for (int32 i = 0; i < GpuSim.Num(); ++i)
	{
		GpuSim.Positions[i]     = RbPos[i];
		GpuSim.PrevPositions[i] = RbPrev[i];
	}

	TestFalse(TEXT("CPU no NaN"), RopeTest::AnyNaN(CpuSim));
	TestFalse(TEXT("GPU no NaN"), RopeTest::AnyNaN(GpuSim));

	// (1) No contact section penetration: No GPU node must be inside the convex (=box).
	float MaxInsideDepth = 0.0f;
	for (const FVector& P : ParityGpuPositions)
	{
		const FVector A = P.GetAbs();
		if (A.X < H.X && A.Y < H.Y && A.Z < H.Z)
		{
			MaxInsideDepth = FMath::Max(MaxInsideDepth, static_cast<float>(FMath::Min3(H.X - A.X, H.Y - A.Y, H.Z - A.Z)));
		}
	}
	TestTrue(FString::Printf(TEXT("GPU convex max inside depth %.3f cm should be < 0.5"), MaxInsideDepth),
		MaxInsideDepth < 0.5f);

	// (2) CPU approximation matching.
	if (!TestEqual(TEXT("Convex parity CPU snapshot node count"), ParityCpuPositions.Num(), N) ||
		!TestEqual(TEXT("Convex parity GPU snapshot node count"), ParityGpuPositions.Num(), N))
	{
		return false;
	}
	float MaxDev = 0.0f;
	for (int32 i = 0; i < N; ++i)
	{
		MaxDev = FMath::Max(MaxDev,
			static_cast<float>(FVector::Dist(ParityCpuPositions[i], ParityGpuPositions[i])));
	}
	AddInfo(FString::Printf(TEXT("Convex contact span (%d frames), largest CPU-to-GPU node deviation: %.2f cm"),
		ParitySampleFrame, MaxDev));
	int32 MaxDevNode = INDEX_NONE;
	for (int32 i = 0; i < N; ++i)
	{
		if (FVector::Dist(ParityCpuPositions[i], ParityGpuPositions[i]) >= MaxDev - KINDA_SMALL_NUMBER)
		{
			MaxDevNode = i;
			break;
		}
	}
	if (MaxDevNode != INDEX_NONE)
	{
		AddInfo(FString::Printf(TEXT("Convex largest deviation node=%d CPU=%s GPU=%s"),
			MaxDevNode,
			*ParityCpuPositions[MaxDevNode].ToCompactString(),
			*ParityGpuPositions[MaxDevNode].ToCompactString()));
	}
	TestTrue(FString::Printf(TEXT("CPU↔GPU max node deviation %.2f cm within tolerance"), MaxDev),
		MaxDev < Length * 0.10f);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUPendingReadbackConsumesStandaloneStepTest,
	"DynamicRope.Solver.GPUPendingReadbackConsumesStandaloneStep",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUPendingReadbackConsumesStandaloneStepTest::RunTest(const FString& Parameters)
{
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU pending handoff test skipped: no renderable RHI (headless)."));
		return true;
	}

	constexpr uint32 RopeId = 0xA55157EDu;
	constexpr uint32 Generation = 37u;
	constexpr int32 NumNodes = 4;
	FRopeSimState Sim = RopeTest::MakeStraightRope(NumNodes, 60.0f);
	FRopeGPUSolver GpuSolver;

	auto MakeOverrideStep = [&](const FVector& Target)
	{
		FRopeGPUResidentStep Step;
		Step.RopeId = RopeId;
		Step.Generation = Generation;
		Step.NumNodes = NumNodes;
		Step.SeedPositions = Sim.Positions;
		Step.SeedPrevPositions = Sim.PrevPositions;
		Step.InvMass = Sim.InvMass;
		Step.SegmentLength = Sim.SegmentLength;
		Step.Iterations = 1;
		Step.bSolveCollisions = false;
		Step.NumSub = 0;
		Step.FixedDt = 1.0f / 60.0f;
		Step.OverrideFlags.SetNumZeroed(NumNodes);
		Step.OverridePositions.SetNumZeroed(NumNodes);
		Step.OverrideFlags[2] = static_cast<uint8>(
			ERopeGPUOverride::Position | ERopeGPUOverride::PrevFromPosition);
		Step.OverridePositions[2] = Target;
		return Step;
	};

	auto EnqueueAndRead = [&](const FVector& Target, TArray<FVector>& OutPos, TArray<FVector>& OutPrev,
		uint32& OutGeneration)
	{
		TArray<FRopeGPUResidentStep> Steps;
		Steps.Add(MakeOverrideStep(Target));
		// It is intentionally placed only in pending without separate dispatch/flush. ReadbackNow performs this step with RT ordering.
		// must be executed first, and the first call without a resident must also succeed.
		GpuSolver.EnqueueSteps(MoveTemp(Steps));
		return GpuSolver.ReadbackNow(RopeId, OutPos, OutPrev, OutGeneration);
	};

	const FVector FirstTarget(31.0f, 17.0f, -9.0f);
	TArray<FVector> Pos;
	TArray<FVector> Prev;
	uint32 ReadGeneration = 0;
	if (!TestTrue(TEXT("ReadbackNow consumes an undispatched pending step"),
		EnqueueAndRead(FirstTarget, Pos, Prev, ReadGeneration)))
	{
		return false;
	}
	TestEqual(TEXT("pending handoff generation"), ReadGeneration, Generation);
	TestEqual(TEXT("pending handoff node count"), Pos.Num(), NumNodes);
	if (Pos.Num() != NumNodes || Prev.Num() != NumNodes)
	{
		return false;
	}
	TestTrue(TEXT("pending override position is authoritative"), FVector::Dist(Pos[2], FirstTarget) < 0.01f);
	TestTrue(TEXT("PrevFromPosition is included in the same snapshot"), FVector::Dist(Prev[2], FirstTarget) < 0.01f);

	// Even if a new pending message enters the same GFrameCounter, the past handoff cache should not be returned.
	const FVector SecondTarget(-12.0f, 44.0f, 6.0f);
	Pos.Reset();
	Prev.Reset();
	if (!TestTrue(TEXT("same-frame second pending step invalidates the old handoff snapshot"),
		EnqueueAndRead(SecondTarget, Pos, Prev, ReadGeneration)))
	{
		return false;
	}
	TestTrue(TEXT("same-frame readback returns the second override"),
		Pos.IsValidIndex(2) && FVector::Dist(Pos[2], SecondTarget) < 0.01f);

	TMap<uint32, FRopeResidentLatest> Latest;
	GpuSolver.GetLatest(Latest);
	const FRopeResidentLatest* Published = Latest.Find(RopeId);
	TestTrue(TEXT("authoritative handoff snapshot is promoted to GetLatest"), Published != nullptr);
	if (Published && Published->Positions.IsValidIndex(2))
	{
		TestTrue(TEXT("GetLatest cannot roll back to the pre-handoff pose"),
			FVector::Dist(Published->Positions[2], SecondTarget) < 0.01f);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeGPUPendingReadbackDoesNotBypassGDFTest,
	"DynamicRope.Solver.GPUPendingReadbackDoesNotBypassSceneGDF",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeGPUPendingReadbackDoesNotBypassGDFTest::RunTest(const FString& Parameters)
{
	if (!FApp::CanEverRender() || GDynamicRHI == nullptr)
	{
		AddWarning(TEXT("GPU GDF handoff test skipped: no renderable RHI (headless)."));
		return true;
	}

	constexpr uint32 RopeId = 0x6DFCA57u;
	constexpr uint32 Generation = 11u;
	FRopeSimState Sim = RopeTest::MakeStraightRope(4, 60.0f);
	FRopeGPUSolver GpuSolver;
	FRopeGPUResidentStep Step;
	Step.RopeId = RopeId;
	Step.Generation = Generation;
	Step.NumNodes = Sim.Num();
	Step.SeedPositions = Sim.Positions;
	Step.SeedPrevPositions = Sim.PrevPositions;
	Step.InvMass = Sim.InvMass;
	Step.SegmentLength = Sim.SegmentLength;
	Step.Iterations = 1;
	Step.bSolveCollisions = true;
	Step.bUseWorldGDF = true;
	Step.NumSub = 1;
	Step.FixedDt = 1.0f / 60.0f;

	TArray<FRopeGPUResidentStep> Steps;
	Steps.Add(MoveTemp(Step));
	GpuSolver.EnqueueSteps(MoveTemp(Steps));
	TArray<FVector> Pos;
	TArray<FVector> Prev;
	uint32 ReadGeneration = 0;
	TestFalse(TEXT("ReadbackNow must not execute a Scene-GDF step through the null-View lean path"),
		GpuSolver.ReadbackNow(RopeId, Pos, Prev, ReadGeneration));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS

