// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeComponent.h"

#include "Collision/RopeCollider.h"
#include "Debug/RopeDebugDraw.h"
#include "Debug/RopeDebugSnapshot.h"
#include "DynamicRopeLog.h"
#include "Engine/World.h"
#include "Materials/MaterialInterface.h"
#include "Preset/RopePreset.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"
#include "Render/RopeSceneProxy.h"
#include "RopeComponentInternal.h"
#include "RopeGPUSolver.h"
#include "Subsystem/RopeDebugSubsystem.h"
#include "Subsystem/RopeSimSubsystem.h"
#include "UObject/ConstructorHelpers.h"

using RopeComponentPrivate::PhaseName;

#pragma region Construction

URopeComponent::URopeComponent()
{
	// component does not tick itself — URopeSimSubsystem ticks all ropes
	// Runs in three steps: Prepare/Solve/Finalize in one place.
	PrimaryComponentTick.bCanEverTick = false;

	// Set the primitive to Movable to output a motion vector (TAA/TSR maintains a moving rope).
	Mobility = EComponentMobility::Movable;

	// Plugin provided default material (hemp rope). If not set, the thin proxy falls back to the engine default (gray).
	// Fill in the default values here — if you change the RopeMaterial in the instance/BP, it will be overridden.
	// If asset does not exist (.Succeeded()==false), keep null → gray fallback (build/cook safe).
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> DefaultRopeMaterial(
		TEXT("/DynamicRope/Materials/M_RopeDefault.M_RopeDefault"));
	if (DefaultRopeMaterial.Succeeded())
	{
		RopeMaterial = DefaultRopeMaterial.Object;
	}
}

#pragma endregion Construction

#pragma region Public_API

void URopeComponent::EnterLoaded()
{
	// ③(GuaranteedWrap) Only throwing ready state. The spear (tip) is held in the hand socket, and the rope tube display follows bShowRopeWhenLoaded.
	// After plugging in, you can only enter when it is released and Free (the initial BeginPlay entry is an exception).
	if (ResolveMode != ERopeWrapResolveMode::GuaranteedWrap)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] EnterLoaded ignored: Loaded은 GuaranteedWrap(③) 전용이다."), *GetName());
		return;
	}
	if (Phase != ERopePhase::Free && Phase != ERopePhase::Loaded)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] EnterLoaded ignored: phase=%s (Free/Loaded에서만 장전 가능)."),
			*GetName(), PhaseName(Phase));
		return;
	}

	// If it is already Loaded, it is reloaded (state reset), but the presentation hook is not called again — the preset is applied to the ③ rope.
	// EnterLoaded() is unconditionally called (ApplyPreset [7]), but at that time, SetPhase is no-op, so the phase event does not occur.
	// Only OnEnterLoaded was fired again, and the enter/deploy pairing was misaligned. If override spawns VFX, it spawns duplicates.
	const bool bAlreadyLoaded = (Phase == ERopePhase::Loaded);

	EnsureRopeInitialized();
	ResetTransientPhaseState();
	ReleaseCooldown = 0.0f;
	EnsureTipMesh();      // Secured insurance — BeginPlay has already taken the normal path (no-op if already available).
	if (!bAlreadyLoaded)
	{
		OnEnterLoaded();    // Default: Hide the rope tube (can be overridden). Only at the entering edge.
	}
	SetPhase(ERopePhase::Loaded, TEXT("reload"));
}

void URopeComponent::SetShowRopeWhenLoaded(bool bShow)
{
	if (bShowRopeWhenLoaded == bShow)
	{
		return;
	}
	bShowRopeWhenLoaded = bShow;

	// If it is Loaded, it is reflected immediately (visibility is applied only at the entry edge, so if it is not present, it will not change until the next Loaded).
	// Other phases are in the deploy state (always visible) and are not touched — the next OnEnterLoaded() consumes this value.
	if (Phase == ERopePhase::Loaded)
	{
		SetVisibility(bShowRopeWhenLoaded, /*bPropagateToChildren*/ false);
	}
}

bool URopeComponent::ToggleShowRopeWhenLoaded()
{
	SetShowRopeWhenLoaded(!bShowRopeWhenLoaded);
	return bShowRopeWhenLoaded;
}

void URopeComponent::OnEnterLoaded()
{
	// Default implementation: bShowRopeWhenLoaded turns the rope tube render on and off (off = only the window is visible in the hand socket).
	// Window position is handled by UpdateTipMeshTransform in the Loaded branch.
	SetVisibility(bShowRopeWhenLoaded, /*bPropagateToChildren*/ false);
}

void URopeComponent::OnDeployFromLoaded()
{
	// Default implementation: Redisplay the rope tube and restore its full length for deploy.
	SetVisibility(true, /*bPropagateToChildren*/ false);
	SetRopeLength(RopeLength);
}

bool URopeComponent::ApplyPreset(const URopePreset* Preset)
{
	// [1] gate — Full application is established only in the idle phase (Free/Loaded). Flying or coiling
	// Reconfiguration is out of scope (seed/latch/path is dependent on old topology) — reject and change nothing.
	if (!Preset)
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] ApplyPreset ignored: preset이 null이다."), *GetName());
		return false;
	}
	if (Phase != ERopePhase::Free && Phase != ERopePhase::Loaded)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] ApplyPreset('%s') ignored: phase=%s (Free/Loaded에서만 적용 가능)."),
			*GetName(), *Preset->GetName(), PhaseName(Phase));
		return false;
	}
	const bool bWasLoaded = (Phase == ERopePhase::Loaded);

	// [2] Value stamps — the only two exceptions are those that require a setter pass: bShowRopeWhenLoaded at the end of this block.
	// Enter SetShowRopeWhenLoaded, and RopeMaterial is input from [5] to SetMaterial.
	// (instance wiring value TipMeshComponentTag is not in the preset, LoadedHandSocket is only covered when opt-in
	//  — see bOverrideLoadedHandSocket branch and header comments below.)
	ResolveMode = Preset->ResolveMode;
	NumParticles = Preset->NumParticles;
	RopeLength = Preset->RopeLength;
	MinRopeLength = Preset->MinRopeLength;
	ReelSpeed = Preset->ReelSpeed;
	SolverConfig = Preset->SolverConfig;
	ThrowParams = Preset->ThrowParams;
	WrapConfig = Preset->WrapConfig;
	HoldConfig = Preset->HoldConfig;
	WhipConfig = Preset->WhipConfig;
	bUseTipMesh = Preset->bUseTipMesh;
	TipMesh = Preset->TipMesh;
	TipMeshRelativeTransform = Preset->TipMeshRelativeTransform;
	bTipMeshCollision = Preset->bTipMeshCollision;
	bSyncTipMeshOnFree = Preset->bSyncTipMeshOnFree;
	LoadedTipRelativeTransform = Preset->LoadedTipRelativeTransform;
	if (Preset->bOverrideLoadedHandSocket)
	{
		// The hand socket is wiring bound to the owning skeleton, so preserving it is the default: an
		// unconditional stamp would let any preset that left the socket empty wipe the instance wiring.
		// Only authoring that wants the preset to pick the hand (per weapon) opts in.
		LoadedHandSocket = Preset->LoadedHandSocket;
	}
	bUseTipMeshSockets = Preset->bUseTipMeshSockets;
	TipSocketName = Preset->TipSocketName;
	TipRopeSocketName = Preset->TipRopeSocketName;
	Radius = Preset->Radius;
	NumSides = Preset->NumSides;
	TubeSmoothingSubdiv = Preset->TubeSmoothingSubdiv;
	TubeSmoothingAlpha = Preset->TubeSmoothingAlpha;
	bIncludeOwnerColliders = Preset->bIncludeOwnerColliders;
	bUseWorldGDF = Preset->bUseWorldGDF;

	// via setter — If reapplied during Loaded, EnterLoaded in [7] is not an entry edge and does not call OnEnterLoaded again.
	// (visibility is applied by the hook), direct assignment is buried until the next Loaded.
	SetShowRopeWhenLoaded(Preset->bShowRopeWhenLoaded);

	// [4] Sim reseed — Always call (single path without branches). NumParticles/RopeLength consumption + GPU resident buffer
	// Includes increased reseed generation. EnsureRopeInitialized is irrelevant here because it only works when empty.
	InitRope();

	// [5] render — RopeMaterial has a contract via SetMaterial (since the scene proxy captures the material at creation time)
	// The proxy must be regenerated with MarkRenderStateDirty to be reflected). Proxy one-time consumption value (Radius/NumSides/
	// TubeSmoothing*) is also reflected in the same MarkRenderStateDirty — the runtime equivalent of the editor PostEditChangeProperty.
	SetMaterial(0, Preset->RopeMaterial);

	// [6] Reconfigure tip — EnsureTipMesh is no-op if there is an existing component, so to reflect asset replacement/on↔off
	// must be unloaded first (only the spawn is destroyed - the tag reuse component is preserved, and Ensure reacquires it if necessary).
	TeardownSpawnedTipMesh();
	EnsureTipMesh();

	// [7] Mode-phase matching — ③ can only be thrown in Loaded (Loaded), so load it immediately (same convention as BeginPlay).
	// Conversely, it was Loaded, but when ①② happens, the Reel becomes meaningless, so deploy (restore visibility/length) and return to Free.
	if (ResolveMode == ERopeWrapResolveMode::GuaranteedWrap)
	{
		EnterLoaded();
	}
	else if (bWasLoaded)
	{
		OnDeployFromLoaded();
		SetPhase(ERopePhase::Free, TEXT("preset applied"));
	}

	// [8] Notification — native hook first, then BP delegate (engine Notify convention).
	NotifyPresetApplied(Preset);
	OnPresetApplied.Broadcast(Preset);
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] preset '%s' applied (mode=%d, N=%d, L=%.0f)."),
		*GetName(), *Preset->GetName(), (int32)ResolveMode, NumParticles, RopeLength);
	return true;
}

#pragma endregion Public_API

#pragma region Simulation_Frame_Pipeline

// ===== Simulation frame (subsystem runs in 3 stages) ===========================

void URopeComponent::PrepareSimFrame(float DeltaTime, const TOptional<FVector>& LODCameraLocation)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_Prepare);
	// (separate contract — this function is the "produce solve input" step: all logic that does not require solve results goes here.
	//  For the basis and three-stage role division, refer to the comments in the Prepare/Solve/Finalize declaration section of the header.)

	EnsureRopeInitialized();
	// frame scope — Gather new logic output for this frame (G2).
	SimFrame.OverrideFrame.Reset();
	SimFrame.bForceNonStretchThisFrame = false;
	const ERopePhase PhaseAtPrepareStart = Phase;
	bEnteredFlightDuringPrepareThisFrame = false;
#if WITH_GAMEPLAY_DEBUGGER
	// fallback — Normally, the subsystem has already caught in Phase 1a (it precedes the pending aim throw) and this call is
	// is a no-op. Only ropes that skipped Phase 1a are recorded here for the first time.
	CaptureDebugFrameStartPhase();
#endif

	// advance pinned-start target; Fast because the solver sweeps Prev->Target across substeps
	// Character movement does not yank (runaway) the chain.
	if (Sim.bStartPinned)
	{
		Sim.StartPinPrev = Sim.StartPinTarget;
		Sim.StartPinTarget = GetComponentLocation();
	}

	// Collider snapshots are collected centrally by RopeSimSubsystem at the collider stage of the tick and filled in FrameColliders.
	// (Before Prepare). Here, the provider is not searched/gathered for each rope.

	// Wrapping(reel): Reflects this frame length change before solving (segment rest length uniform change —
	// Same CPU/GPU without reseeding). In Wrapped, the available rope length is reduced and propagated to the tether/tension.
	UpdateReel(DeltaTime);

	// Distance LOD multiplier (iteration damping) — Confirm this frame value before solve check (GPU step/CPU solve shared).
	ComputeSolverLOD(LODCameraLocation);

	// Sleep is only for Free/Wrapped — immediately released when transitioning to another phase (the transition itself is active).
	if (Phase != ERopePhase::Free && Phase != ERopePhase::Wrapped && Throttle.IsAsleep())
	{
		Throttle.Wake();
	}

	SimFrame.bSolveThisFrame = false;
	SimFrame.bSolveCollisionsThisFrame = true;

	switch (Phase)
	{
	// Follows the character while hanging from his hand.
	case ERopePhase::Free:
		if (Throttle.IsAsleep() && Throttle.ShouldWakeFromSleep(Sim, SolverConfig, ReelRate, SimFrame.FrameColliders))
		{
			Throttle.Wake();
		}
		// Solve is skipped during sleep (GPU rope does not dispatch itself).
		SimFrame.bSolveThisFrame = !Throttle.IsAsleep();
		break;

	case ERopePhase::Flight:
	{
		if (WhipGuide.IsActive())
		{
			// target/mask calculation only (Sim invariant) — Apply is done using CPU path SolveSimFrame(ApplyToSim) or
			// The override pass (subsystem carried in step) of the GPU resident path is responsible (G1).
			WhipGuide.Advance(DeltaTime, Sim, MakeWhipGuideConfig());
		}
		else
		{
			WhipGuide.ResetFrameOutputs();
		}

		if (AimTargeting.IsLockActive(Phase))
		{
			// After applying the central guide, both ends are naturally connected using XPBD distance/bending/damping.
			// collider push-out is turned off by a separate gate so that previous collision instantaneous movement does not occur again.
			SimFrame.bSolveThisFrame = true;
			SimFrame.bSolveCollisionsThisFrame = !WhipConfig.bAimHitCollisionFreeSolve;
		}
		else
		{
			// In general Flight, contact is detected in Finalize after solving as before.
			SimFrame.bSolveThisFrame = true;
		}
		break;
	}

	case ERopePhase::Contacting:
		UpdateContacting(DeltaTime);
		break;

	case ERopePhase::Wrapping:
		UpdateWrapping(DeltaTime);
		// Only the actual Wrapping position override/anchor node is pinned with the mass mask. The path has not been reached yet
		// The solver continues to process the tail and resolves the strain remaining in the Flight during Wrapping.
		SimFrame.bSolveThisFrame = (Phase == ERopePhase::Wrapping || Phase == ERopePhase::Wrapped);
		break;

	case ERopePhase::Wrapped:
	{
		// Wrapped tick = 4-step pinned sequence: ① bone following(Hold + mass mask — release when target disappears)
		// → ② Calculate observed values (tension + Pull sample/smoothing — shared input of ③④) → ③ Apply traction (tether + active pull)
		// → ④ Automatic release check (exceeding tension / exceeding distance — consumes the output of ②③).
		if (!HoldWrappedNodesToBone(DeltaTime))
		{
			// Target mesh lost — release completed (no solve).
			break;
		}
		// Hold records the current bone position only in OverrideFrame, and actual Sim application is performed as a single bone position at the end of Prepare.
		// ApplyToSim contract is postponed. In a separate observation view so that pull/tether does not read the previous solve pose
		// First synthesize the current hand pin + Hold override. Free nodes inside the GPU can be mirrors, but
		// The movement hard constraint uses the live CPU binding descriptor, not this view.
		PullObservationSim = Sim;
		if (PullObservationSim.Positions.IsValidIndex(0))
		{
			PullObservationSim.Positions[0] = Sim.StartPinTarget;
			if (PullObservationSim.PrevPositions.IsValidIndex(0))
			{
				PullObservationSim.PrevPositions[0] = Sim.StartPinTarget;
			}
		}
		if (SimFrame.OverrideFrame.HasAny())
		{
			SimFrame.OverrideFrame.ApplyToSim(PullObservationSim);
		}
		UpdateWrappedPullSample(DeltaTime, PullObservationSim);
		ApplyWrappedTraction(DeltaTime);
		if (CheckWrappedAutoRelease(DeltaTime))
		{
			// Tension/distance release occurs (no solve).
			break;
		}
		// Wrapped stationary throttling (extension of Free slip): If the pin, Wrapped bone, and proximity collider are all stationary, Free span solve is performed.
		// Rest. The above ①~④ (Hold/observation/traction/automatic release) continues even during sleep — what is skipped is the solver
		// Since it is only a substep, the GPU is reduced to override-only dispatch (NumSub=0). Entry is through Finalize
		// UpdateSleepState (velocity settlement, blocked with bHoldAwake during active Pull Loaded) is shared processing. Wrapped bone movement
		// It wakes up due to node drift written by Hold, and the observation view (PullObservationSim) pin + override this frame.
		// Since it has already been synthesized, it is detected without delay (wake up at the frame when the elevator departs).
		if (Throttle.IsAsleep()
			&& (PullDrive.ActivePullForce > 0.0f
				|| Throttle.ShouldWakeFromSleep(PullObservationSim, SolverConfig, ReelRate, SimFrame.FrameColliders)))
		{
			Throttle.Wake();
		}
		SimFrame.bSolveThisFrame = !Throttle.IsAsleep();
		break;
	}

	case ERopePhase::GuidedThrow:
		// ③ Dedicated phase. Skip the physics solver/contact detector and only follow a deterministic path (aiming) or arch (air).
		UpdateGuidedThrow(DeltaTime);
		SimFrame.bSolveThisFrame = false;
		break;

	case ERopePhase::Loaded:
	{
		// Throwing preparation state (③ only): The spear (tip) is pinned to the hand socket, and then the rope is naturally stretched physically.
		// As for node 0, the above pin logic (StartPinTarget) follows the hand, so if only the tip is pinned to the socket, the rope in between will sag.
		// Reason why tip pinning is necessary: At the moment of throwing, the tip render changes from socket to last node (UpdateTipMeshTransform),
		// The last node must be in the socket to prevent the window from popping. You must turn on solve to follow the character without freeze.
		// (Same pattern as Hold in Wrapped: Position + InvMass=0 override → The solver rolls the remaining nodes.)
		// MakeLoadedTipBaseWorld = hand socket + LoadedTipRelativeTransform. This must use the *same* base
		// as the Loaded branch of UpdateTipMeshTransform - offsetting only one of the two would leave the
		// spear and the rope end apart.
		const int32 LoadedTipNode = Sim.Num() - 1;
		const FTransform LoadedTipWorld = MakeLoadedTipBaseWorld();
		SimFrame.OverrideFrame.EnsureSize(Sim.Num());
		SimFrame.OverrideFrame.SetPosition(LoadedTipNode, ResolveTipRopeAttachWorld(LoadedTipWorld), /*bZeroVelocity*/ true);
		SimFrame.OverrideFrame.SetInvMass(LoadedTipNode, 0.0f);
		SimFrame.bSolveThisFrame = true;
		break;
	}

	case ERopePhase::Releasing:
		// Hand all nodes back to the solver (keep only hand pins) — Restore InvMass + Prev=Pos (prevent splash)
		// is stored as frame output, and Free simulation is resumed when cooldown is completed.
		SimFrame.OverrideFrame.EnsureSize(Sim.Num());
		for (int32 i = 0; i < Sim.Num(); ++i)
		{
			SimFrame.OverrideFrame.SetInvMass(i, (i == 0 && Sim.bStartPinned) ? 0.0f : 1.0f);
			SimFrame.OverrideFrame.SetPrevFromPosition(i);
		}
		ReleaseCooldown -= DeltaTime;
		if (ReleaseCooldown <= 0)
		{
			SetPhase(ERopePhase::Free);
		}
		break;

	default:
		break;
	}

	// Only the frame that was Flight from the start of Prepare had normal Flight Advance/Solve input.
	// If you return to Flight while processing logic, Finalize recapture is postponed and the guide resumes first in the next frame.
	bEnteredFlightDuringPrepareThisFrame =
		PhaseAtPrepareStart != ERopePhase::Flight && Phase == ERopePhase::Flight;

	// Apply the frame output of the logic phase to the CPU Sim once — unlike the existing “write directly within the handler”
	// Same result (when overlapping the same node, later fill wins = same as sequential write). The GPU-resident rope has
	// The subsystem loads the same frame into the override pass and applies it in the kernel (G2).
	// Logic phase reseeding (increasing SimFrame.SimGeneration) destroys — the only reseeding is the real seed (Init/Throw).
	if (SimFrame.OverrideFrame.HasAny())
	{
		SimFrame.OverrideFrame.ApplyToSim(Sim);
	}
}

void URopeComponent::SolveSimFrame(float DeltaTime)
{
	// Parallel step: only touches the POD state (Sim) + collider snapshot (SimFrame.FrameColliders). Query is const → thread safe.
	// solve physics only when SimFrame.bSolveThisFrame(Free/Flight/Wrapping/Wrapped) — Wrapping/Wrapped is
	// Since the pinned node has InvMass=0, only the Free span moves, and Contacting/Releasing is logic driven, so it is skipped.
	if (!SimFrame.bSolveThisFrame)
	{
		return;
	}
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_Solve);

	// Apply whip target (CPU path, POD only — thread safe). Start solving the output calculated by Prepare's Advance
	// location. Instead of this function, GPU rope applies the same data as the override pass in the kernel (G1).
	// Flight gate: Prevents remaining stale masks (output of the previous whip frame) from being applied to other phases.
	if (Phase == ERopePhase::Flight)
	{
		WhipGuide.ApplyToSim(Sim);
	}

	// Distance LOD: Damping only constraint iteration at a distance (substep is maintained — stability is dominated by substep).
	FRopeSolverConfig LODConfig = SolverConfig;
	LODConfig.Iterations = GetLODScaledIterations();
	LODConfig.MaxStretchRatio = GetEffectiveMaxStretchRatio();
	// radius auto(0=render Radius) Analysis — The solver always receives only the interpreted value (GPU step is processed in the same way by the subsystem).
	LODConfig.CollisionRadius = GetEffectiveCollisionRadius();
	// Aim-hit collision-Free solve also executes the solver itself, but only excludes push-out by passing an empty list.
	const TArray<IRopeCollider*> NoSolveColliders;
	const TArray<IRopeCollider*>& SolveColliders = SimFrame.bSolveCollisionsThisFrame
		? SimFrame.FrameColliders
		: NoSolveColliders;
	Solver.Step(Sim, LODConfig, SolveColliders, DeltaTime);
}

float URopeComponent::GetEffectiveMaxStretchRatio() const
{
	// Physics throwing in ①/② uses guide target and Wrapping position override as solver input.
	// If the allowable length is left in this section, the moment the kinematic node is released, it will be Wrapped to the rest length.
	// In the CPU/GPU strain-limit step of solving the actual resident pose, it is limited to complete non-stretching.
	const bool bPhysicalResolveMode = ResolveMode != ERopeWrapResolveMode::GuaranteedWrap;
	const bool bThrowOrWrapFrame =
		Phase == ERopePhase::Flight ||
		Phase == ERopePhase::Wrapping ||
		SimFrame.bForceNonStretchThisFrame;
	// Once held, the visual solver must obey the same material contract as movement and
	// reaction tension. A rigid authoritative hold cannot leave MaxStretchRatio=1.5 as a
	// second, hidden elasticity source. Positive TetherCompliance explicitly opts back into
	// the configured visual stretch policy.
	const bool bRigidAuthoritativeHold =
		HoldConfig.bEnforceWielderLengthConstraint &&
		HoldConfig.TetherCompliance <= KINDA_SMALL_NUMBER &&
		(Phase == ERopePhase::Wrapping || Phase == ERopePhase::Wrapped);
	return (bPhysicalResolveMode && bThrowOrWrapFrame) || bRigidAuthoritativeHold
		? 1.0f
		: SolverConfig.MaxStretchRatio;
}

void URopeComponent::FinalizeSimFrame(float DeltaTime)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_Finalize);

	// (Separation contract — This function is the “solve output consumption” step. See the comment in the header’s step 3 declaration for rationale.)
	// Debug capture gate: Collect visual data only when this rope is the target actor of the gateplay debugger.
	// Ropes that are not the target do not pay any capture costs such as Flight sweep (only one target rope is charged).
#if WITH_GAMEPLAY_DEBUGGER
	URopeDebugSubsystem* DebugSub = URopeDebugSubsystem::Get(GetWorld());
	const bool bDebugCapture = DebugSub && DebugSub->ShouldCapture(this);
	// Collect only views that are turned on. In particular, if there is no Flight bit, the observation step below performs the resweep itself for each node.
	// Skip — If you only block from drawing, this cost remains.
	const ERopeDebugCapture CaptureMask = bDebugCapture ? DebugSub->GetCaptureMask() : ERopeDebugCapture::None;
	FRopeDebugSnapshot DebugSnapshot;
	FRopeDebugSnapshot* const FlightSnapshot =
		EnumHasAnyFlags(CaptureMask, ERopeDebugCapture::Flight) ? &DebugSnapshot : nullptr;
#else
	constexpr bool bDebugCapture = false;
	FRopeDebugSnapshot* const FlightSnapshot = nullptr;
#endif

	// Flight: After solving, movement path-based contact candidate detection → capture. The pipeline itself
	// FRopeFlightContactDetector (UObject-independent), which only does three-step orchestration.
	// All stat/debugger consumption is in ③ — only the check flow is left in the text.
	if (Phase == ERopePhase::Flight && !bEnteredFlightDuringPrepareThisFrame)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FinalizeFlight);
		const FRopeFlightContactDetector::FParams DetectParams = MakeFlightDetectParams(DeltaTime);

		// ① Candidate calculation
		TArray<FRopeContactCandidate>& Candidates = GetOrBuildFlightContactCandidates(DeltaTime, DetectParams);

		// ② The check result is created once and the game transition and observation consume the same tracker.
		FRopeFlightCaptureEvaluation CaptureEvaluation = EvaluateFlightCapture(Candidates, DetectParams);
		const bool bShouldCapture = ApplyFlightCaptureEvaluation(DeltaTime, Candidates, CaptureEvaluation);
		const FRopeContactTracker& FrameTracker = bShouldCapture
			? ContactTracker
			: CaptureEvaluation.Tracker;
		// ③ Observation
		RecordFlightObservation(DetectParams, Candidates, FrameTracker, bShouldCapture, FlightSnapshot);
	}

	// Only the frames where the actual centerline/GPU source/component transform has changed are pushed back to the render data.
	// Like stationary Free/Contacting, a rope without solve/override and with the same transform does not create a render command.
	const FTransform CurrentComponentTransform = GetComponentTransform();
	const bool bRenderTransformChanged = !bHasLastRenderDataComponentTransform ||
		!LastRenderDataComponentTransform.Equals(CurrentComponentTransform);
	const bool bGpuResidentChanged = bLastRenderDataGpuResident != SimFrame.bGpuSteppedThisFrame;
	const bool bRenderDataChanged = SimFrame.bSolveThisFrame || SimFrame.OverrideFrame.HasAny() ||
		bGpuResidentChanged || bRenderTransformChanged;
	if (bRenderDataChanged)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_MarkRenderDynamicDataDirty);
		MarkRenderDynamicDataDirty();
	}
	if (bRenderTransformChanged)
	{
		MarkRenderTransformDirty();
	}
	LastRenderDataComponentTransform = CurrentComponentTransform;
	bHasLastRenderDataComponentTransform = true;
	bLastRenderDataGpuResident = SimFrame.bGpuSteppedThisFrame;

	// Follows the tip attachment (spearhead/harpoon) to the determined Free end position (hereinafter referred to as the solve output consumption step).
	UpdateTipMeshTransform();

	// Wrapped stat counter (independent).
	if (Phase == ERopePhase::Wrapped)
	{
		RopeDebug::RecordWrappedStats(Sim, WrapController.State);
	}

	// Slip transition measurement (Free/Wrapped — This is where the results for this frame are confirmed because it is based on node displacement between frames).
	// In Wrapped, entry is prevented while the active pull is Loaded (bHoldAwake — traction impulse/climb-in is an integral premise).
	if (Throttle.UpdateSleepState(Phase, Sim, SolverConfig, DeltaTime,
		/*bHoldAwake*/ Phase == ERopePhase::Wrapped && PullDrive.ActivePullForce > 0.0f))
	{
		UE_LOG(LogDynamicRope, Verbose, TEXT("[%s] rope asleep (max speed < %.1f cm/s for %.2fs)"),
			*GetName(), SolverConfig.SleepVelocityThreshold, SolverConfig.SleepDelay);
	}

	// Submit a debug snapshot: Fill in the centerline/collider/Wrapped common fields and pass it to the debugger archive.
#if WITH_GAMEPLAY_DEBUGGER
	if (bDebugCapture)
	{
		FillDebugSnapshot(DebugSnapshot, CaptureMask);
		DebugSub->SubmitSnapshot(this, MoveTemp(DebugSnapshot));
	}
#endif
}

#pragma endregion Simulation_Frame_Pipeline

#pragma region Component_Lifecycle

// ===== UActorComponent ======================================================

void URopeComponent::BeginPlay()
{
	Super::BeginPlay();
	if (URopeSimSubsystem* SimSubsystem = URopeSimSubsystem::Get(GetWorld()))
	{
		SimSubsystem->RegisterRope(this);
	}
	else
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] BeginPlay: RopeSimSubsystem unavailable — rope will not be simulated."),
			*GetName());
	}

	// Obtain tip attachment (only when bUseTipMesh is enabled). Lifespan = BeginPlay~EndPlay, regardless of latching model or arrival mode —
	// Even in Free, the tip must be secured here to be visible at the end of the rope. Since the SIM is not read, there is no dependency on initialization order.
	EnsureTipMesh();

	// ③ Guaranteed rope starts in the throwing state (Loaded) — waiting with the spear in hand (rope hidden).
	if (ResolveMode == ERopeWrapResolveMode::GuaranteedWrap)
	{
		EnterLoaded();
	}
}

void URopeComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// When an attached state is destroyed/removed, a release event is fired so that the consumer (caught counter, ragdoll misrecovery) is not stuck in the state.
	// . In the world-wide teardown (Quit/LevelTransition/editor termination), subscribers also die, so it is omitted.
	if (EndPlayReason == EEndPlayReason::Destroyed || EndPlayReason == EEndPlayReason::RemovedFromWorld)
	{
		if (Phase == ERopePhase::Wrapped)
		{
			// Committed wrap is destroyed → per-instance + central signal (ragdoll recovery is required if cross-actor target is alive).
			DispatchReleased(WrapController.State.Mesh.Get(), WrapController.State.BoneName,
				ERopeReleaseReason::Broken, /*bWasWrapped*/ true);
		}
		else if (Phase == ERopePhase::Contacting || Phase == ERopePhase::Wrapping ||
			(Phase == ERopePhase::GuidedThrow && !GuidedThrowState.bFreeThrow))
		{
			// Engagement is destroyed before establishment → per-instance only (matching). Aimed ③ throwing also opened an engagement
			// (from the point of capturing the target — DispatchReleased contract). Except for throwing in the air as there is no target.
			FName Bone = NAME_None;
			if (Phase == ERopePhase::Wrapping)          { Bone = WrappingPhase.State.BoneName; }
			else if (Phase == ERopePhase::GuidedThrow)  { Bone = GuidedThrowState.Prepared.Bone; }
			else                                        { Bone = ContactTracker.CandidateBone; }
			DispatchReleased(nullptr, Bone, ERopeReleaseReason::Broken, /*bWasWrapped*/ false);
		}
	}

	// Clean up the tip attachment we spawned (preserving external components). Lifespan = BeginPlay~EndPlay, so there is only one destruction here.
	TeardownSpawnedTipMesh();
	TeardownPhysicalTether(); // Physical constraint tether summary (compared to a destruction path that does not undergo a phase transition).

	if (URopeSimSubsystem* SimSubsystem = URopeSimSubsystem::Get(GetWorld()))
	{
		SimSubsystem->UnregisterRope(this);
	}
	Super::EndPlay(EndPlayReason);
}

void URopeComponent::SendRenderDynamicData_Concurrent()
{
	Super::SendRenderDynamicData_Concurrent();

	if (!SceneProxy || Sim.Num() < 2)
	{
		return;
	}

	// send centerline to component-local space; The proxy renders through GetLocalToWorld().
	const FTransform Xform = GetComponentTransform();
	FRopeDynamicData* DynamicData = new FRopeDynamicData;
	// M5b: Only GPU stepped frames are allowed to render resident PosBuf directly.
	DynamicData->bGpuResident = SimFrame.bGpuSteppedThisFrame;
	// The proxy matches the generation of the resident buffer (prevents ghosting of one frame from reseeding on the same number of nodes).
	DynamicData->SimGeneration = SimFrame.SimGeneration;
	// The world → local transformation of the resident tube is also this GT transform — it is the same frame value as the Points localization, so it is drawn.
	// matches the transform (the proxy GetLocalToWorld() is one frame before the time of SetDynamicData — see header comments).
	DynamicData->WorldToLocal = FMatrix44f(Xform.ToInverseMatrixWithScale());
	DynamicData->Points.SetNumUninitialized(Sim.Num());
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		DynamicData->Points[i] = Xform.InverseTransformPosition(Sim.Positions[i]);
	}

	FRopeSceneProxy* Proxy = static_cast<FRopeSceneProxy*>(SceneProxy);
	ENQUEUE_RENDER_COMMAND(RopeUpdateCenterline)(
		[Proxy, DynamicData](FRHICommandListBase& RHICmdList)
		{
			Proxy->SetDynamicData_RenderThread(RHICmdList, DynamicData);
		});
}

void URopeComponent::OnRegister()
{
	Super::OnRegister();
	// The editor also fills in the default straight pose in the Sim (since subsystem ticks only run in PIE).
	// If it is already filled (after InitRope/while PIE is in progress), leave it as is.
	EnsureRopeInitialized();
}

void URopeComponent::CreateRenderState_Concurrent(FRegisterComponentContext* Context)
{
	Super::CreateRenderState_Concurrent(Context);
	// The proxy has just been created. Even right after editor/spawning without a tick, push the center line once to make BuildTube spin.
	// (so that bHasData=true and the static draw draws valid geometry). SendRenderDynamicData_Concurrent
	// SceneProxy/Sim self-checks for validity and enqueues only the render command, so calling at this point is safe.
	SendRenderDynamicData_Concurrent();
}

#if WITH_EDITOR
void URopeComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	// When NumParticles/RopeLength is changed, the proxy is regenerated with the new topology (NumRings), but the Sim is the old number.
	// BuildTube skips to Points.Num()!=NumRings and the preview disappears. EnsureRopeInitialized should be empty
	// , here, the topology is adjusted by forcibly reconfiguring the Sim to a new value. Afterwards, Super renders the state
	// Regenerate and push the centerline again in CreateRenderState_Concurrent.
	const FName PropertyName = PropertyChangedEvent.GetPropertyName();
	if (PropertyName == GET_MEMBER_NAME_CHECKED(URopeComponent, NumParticles) ||
		PropertyName == GET_MEMBER_NAME_CHECKED(URopeComponent, RopeLength))
	{
		InitRope();
	}
	else if (PropertyName == GET_MEMBER_NAME_CHECKED(URopeComponent, bTipMeshCollision))
	{
		// When toggled during PIE, it is immediately reflected in the current tip (if there is no tip, no-op — applied at the next acquisition).
		ApplyTipMeshCollision();
	}

	Super::PostEditChangeProperty(PropertyChangedEvent);
}
#endif

#pragma endregion Component_Lifecycle

#pragma region Rendering

// ===== UPrimitiveComponent / UMeshComponent =================================

FPrimitiveSceneProxy* URopeComponent::CreateSceneProxy()
{
	return new FRopeSceneProxy(this);
}

int32 URopeComponent::GetNumMaterials() const
{
	return 1;
}

UMaterialInterface* URopeComponent::GetMaterial(int32 /*ElementIndex*/) const
{
	return RopeMaterial;
}

void URopeComponent::SetMaterial(int32 /*ElementIndex*/, UMaterialInterface* Material)
{
	RopeMaterial = Material;
	// Since the scene proxy captures the material at creation time, regenerate the proxy to reflect the replacement.
	MarkRenderStateDirty();
}

FBoxSphereBounds URopeComponent::CalcBounds(const FTransform& LocalToWorld) const
{
	// Anchor the bounds to the component (pinned start), but always set a radius that includes the rope, no matter how the rope is transformed.
	// : The chain is inextensible, so no particle can move farther than RopeLength (+ tube radius) from the pin.
	// Does not fall. Instead, deriving bounds from a per-frame sim point lags the render thread by one frame;
	// During fast character motion, the rope overtakes the tight box and culls in the shadow/main pass -> while moving.
	// The shadow disappears and the VSM cache maintains the old afterimage. When anchoring a component transform, the engine
	// The bounds move with the character through tracked transforms, so there is no lag or false culling.
	const float Reach = RopeLength + Radius + 1.0f;
	return FBoxSphereBounds(LocalToWorld.GetLocation(), FVector(Reach), Reach);
}

#pragma endregion Rendering

#pragma region Phase_State_Machine

// ===== phase state machine =========================================================

void URopeComponent::SetPhase(ERopePhase NewPhase, const TCHAR* Reason)
{
	if (Reason)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] %s -> %s (%s)"),
			*GetName(), PhaseName(Phase), PhaseName(NewPhase), Reason);
	}
	else
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] %s -> %s"),
			*GetName(), PhaseName(Phase), PhaseName(NewPhase));
	}
	const ERopePhase OldPhase = Phase;
	Phase = NewPhase;

	// Extended hook + BP event (actual transition only — does not notify same phase reset).
	if (OldPhase != NewPhase)
	{
		OnPhaseChanged(OldPhase, NewPhase);
		OnRopePhaseChanged.Broadcast(OldPhase, NewPhase);
	}
}

void URopeComponent::ResetTransientPhaseState(bool bPreservePhysicalTether)
{
	AimTargeting.ResetPendingThrow();
	PendingGuaranteedAimThrow.Reset();
	ContactTracker.Reset();
	PendingWrapSeed.Reset();
	bPendingGpuCaptureHandoff = false;
	if (!bPreservePhysicalTether)
	{
		TeardownPhysicalTether(); // abort/release/new throw/end play: attempt-scoped constraint.
	}
	CaptureTravelFrame.Reset();
	WrappingPhase.State.Reset();
	GuidedThrowState.Reset();
	ContactingElapsed = 0.0f;
	FlightNoContactElapsed = 0.0f;
	TensionOverTime = 0.0f;
	// Pull sample/EMA 3 types/Warning latch only. See FRopePullDriveState comment for survival fields.
	PullDrive.ResetTransient();
	LengthConstraintState.ResetTransient();
}

#pragma endregion Phase_State_Machine

#pragma region Initialization_And_Debug

// ===== Initialization/Utility ============================================================================

void URopeComponent::InitRope()
{
	// If the GPU solver cap (thread group = MaxNodes) is exceeded, CPU solve+tube fallback quietly becomes a performance cliff.
	// Apart from the editor ClampMax, also hard clamps the BP/code path — write the values ​​to proxy NumNodes (= NumParticles) and
	// Make sure the Sim sizes match (if they don't, BuildTube is skipped). 1 warning if exceeded.
	if (NumParticles > FRopeGPUSolver::MaxNodes)
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] NumParticles %d exceeds the GPU solver cap %d; clamping (values above the cap fall back to CPU solve+tube)."),
			*GetName(), NumParticles, FRopeGPUSolver::MaxNodes);
		NumParticles = FRopeGPUSolver::MaxNodes;
	}
	const int32 N = FMath::Max(2, NumParticles);
	Sim.Reset();
	Sim.Positions.SetNum(N);
	Sim.PrevPositions.SetNum(N);
	Sim.InvMass.SetNum(N);
	Sim.RopeLength = RopeLength;
	Sim.SegmentLength = RopeLength / static_cast<float>(N - 1);

	const FVector Start = GetComponentLocation();
	const FVector End = Start + GetForwardVector() * RopeLength;
	for (int32 i = 0; i < N; ++i)
	{
		const float Alpha = static_cast<float>(i) / static_cast<float>(N - 1);
		Sim.Positions[i] = FMath::Lerp(Start, End, Alpha);
		Sim.SetStill(i);
		Sim.InvMass[i] = 1.0f;
	}

	// Pin the starting point to component(hand/socket); The solver sweeps this over substeps.
	Sim.InvMass[0] = 0.0f;
	Sim.bStartPinned = true;
	Sim.StartPinTarget = Start;
	Sim.StartPinPrev = Start;

	// Full Sim reconfiguration → GPU resident buffer reseeding (M5).
	++SimFrame.SimGeneration;
	bWrappedMassMaskDirty = true;

	UE_LOG(LogDynamicRope, Verbose, TEXT("[%s] InitRope: %d particles, length=%.1f, segment=%.2f"),
		*GetName(), N, Sim.RopeLength, Sim.SegmentLength);
}

void URopeComponent::EnsureRopeInitialized()
{
	if (Sim.Num() == 0)
	{
		InitRope();
	}
}

#pragma endregion Initialization_And_Debug

#pragma region Rope_Length_Reel_And_LOD

void URopeComponent::SetRopeLength(float NewLength)
{
	if (Sim.Num() < 2)
	{
		return;
	}
	// cap = initial (designer) length — Unwinding unwinds only the amount of winding. floor = MinRopeLength.
	const float MaxLen = FMath::Max(RopeLength, MinRopeLength);
	const float Clamped = FMath::Clamp(NewLength, FMath::Min(MinRopeLength, MaxLen), MaxLen);
	if (FMath::IsNearlyEqual(Clamped, Sim.RopeLength))
	{
		return;
	}
	Sim.RopeLength = Clamped;
	Sim.SegmentLength = Clamped / static_cast<float>(Sim.Num() - 1);
}

void URopeComponent::SetReelRate(float CmPerSecond)
{
	ReelRate = CmPerSecond;
}

void URopeComponent::ComputeSolverLOD(const TOptional<FVector>& CameraLocation)
{
	// Camera UObject search is performed once per frame by RopeSimSubsystem. Here, only the distance for each rope is calculated.
	// Based on local player 0, no server/camera = full quality.
	TOptional<float> CameraDist;
	if (SolverConfig.bEnableDistanceLOD && SolverConfig.LODStartDistance > 0.0f && CameraLocation.IsSet())
	{
		CameraDist = static_cast<float>(FVector::Dist(CameraLocation.GetValue(), GetComponentLocation()));
	}
	Throttle.ComputeSolverLOD(SolverConfig, CameraDist);
}

void URopeComponent::UpdateReel(float DeltaTime)
{
	if (FMath::IsNearlyZero(ReelRate))
	{
		return;
	}
	// Contacting/Wrapping/Releasing pending: Wrapping path creation/commit distance based on SegmentLength
	// (RopeDistance = idx × SegmentLength), if I change the scale below, the path becomes distorted.
	if (Phase != ERopePhase::Free && Phase != ERopePhase::Flight && Phase != ERopePhase::Wrapped)
	{
		return;
	}
	float NewLength = Sim.RopeLength - ReelRate * DeltaTime;
	// Feasibility of Wrapping(+) Stall: The material length of the non-stretchable rope is greater than the straight line distance between the hand and the anchor (the floor of the required length).
	// cannot be shortened. If the reel continues to wind even though the target is heavy or stuck and is not drawn, violation C accumulates infinitely, and the simulation
	// The target's hard Chaos limit breaks every substep, fighting the joints to close the formation, causing the ragdoll to fluctuate.
	// (Snare both arms tied and shaking, 2026-07-24). Like a winch stalling under a load, it follows “only as much as the object is actually pulled”.
	// Wind — The violation is bounded to the reel frame step level (slack), the pull bias remains and the fight disappears.
	// Unwind (-) and self wrap/constraint disabled (no live binding) are the same as before.
	if (ReelRate > 0.0f && Phase == ERopePhase::Wrapped)
	{
		FRopeWielderMovementConstraint LiveConstraint;
		if (BuildWielderMovementConstraint(LiveConstraint))
		{
			const float RequiredLength = static_cast<float>(
				FVector::Distance(GetComponentLocation(), LiveConstraint.PivotWorld));
			// slack = Reel 2 frame step (minimum 1cm): Normal traction (target following at reel velocity) is not touched.
			const float StallSlack = FMath::Max(ReelRate * DeltaTime * 2.0f, 1.0f);
			// floor is "no more", not "unwind" — it does not increase length if the violation is already large (currently length cap).
			NewLength = FMath::Max(NewLength, FMath::Min(RequiredLength - StallSlack, Sim.RopeLength));
		}
	}
	SetRopeLength(NewLength);
}

#pragma endregion Rope_Length_Reel_And_LOD
