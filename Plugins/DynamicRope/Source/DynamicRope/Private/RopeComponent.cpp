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
#include "RopeTautPresentation.h"
#include "Subsystem/RopeDebugSubsystem.h"
#include "Subsystem/RopeSimSubsystem.h"
#include "UObject/ConstructorHelpers.h"

using RopeComponentPrivate::PhaseName;

#pragma region Construction

URopeComponent::URopeComponent()
{
	// The component does not tick itself — URopeSimSubsystem ticks every rope, running Prepare, Solve and
	// Finalize from one place.
	PrimaryComponentTick.bCanEverTick = false;

	// Movable, so the primitive emits motion vectors and TAA/TSR keeps a moving rope stable.
	Mobility = EComponentMobility::Movable;

	// The plugin's default material (hemp rope). Filled in here as a default: changing RopeMaterial on the
	// instance or in a Blueprint overrides it. If the asset is missing (.Succeeded() == false) it stays null
	// and the scene proxy falls back to the engine's grey default, which keeps builds and cooks safe.
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
	// The ready state, GuaranteedWrap only: the tip is held in the hand socket, and the rope tube is shown or
	// hidden per bShowRopeWhenLoaded. Once thrown, the rope can only load again after it releases and returns
	// to Free — the initial entry at BeginPlay being the exception.
	if (ResolveMode != ERopeWrapResolveMode::GuaranteedWrap)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] EnterLoaded ignored: Loaded is for GuaranteedWrap only."), *GetName());
		return;
	}
	if (Phase != ERopePhase::Free && Phase != ERopePhase::Loaded)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] EnterLoaded ignored: phase=%s (can only load from Free or Loaded)."),
			*GetName(), PhaseName(Phase));
		return;
	}

	// Loading again while already Loaded resets the state but does not fire the presentation hook a second
	// time. It matters because applying a preset to a GuaranteedWrap rope always calls EnterLoaded(): SetPhase
	// is a no-op then, so no phase event fires, but OnEnterLoaded firing again would break the enter/deploy
	// pairing — and an override that spawns VFX would spawn them twice.
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

	// While Loaded it takes effect at once. Visibility is otherwise applied on the edge into Loaded, so
	// without this the change would wait for the next one. Other phases are deployed and always visible, so
	// they are left alone and the next OnEnterLoaded() consumes this value.
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
	// Default implementation: bShowRopeWhenLoaded shows or hides the rope tube. Off leaves only the tip
	// visible in the hand socket; the tip's placement is UpdateTipMeshTransform's Loaded branch.
	SetVisibility(bShowRopeWhenLoaded, /*bPropagateToChildren*/ false);
}

void URopeComponent::OnDeployFromLoaded()
{
	// Default implementation: show the rope tube again and restore its full length for the deploy.
	SetVisibility(true, /*bPropagateToChildren*/ false);
	SetRopeLength(RopeLength);
}

bool URopeComponent::ApplyPreset(const URopePreset* Preset)
{
	// [1] Gate — a preset only applies from an idle phase (Free or Loaded). Reconfiguring mid-flight or
	// mid-wrap is out of scope, because the seed, latches and path all depend on the old topology, so the
	// call is rejected and nothing changes.
	if (!Preset)
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] ApplyPreset ignored: preset is null."), *GetName());
		return false;
	}
	if (Phase != ERopePhase::Free && Phase != ERopePhase::Loaded)
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] ApplyPreset('%s') ignored: phase=%s (can only apply from Free or Loaded)."),
			*GetName(), *Preset->GetName(), PhaseName(Phase));
		return false;
	}
	const bool bWasLoaded = (Phase == ERopePhase::Loaded);

	// [2] Stamp the values. Only two need to go through a setter instead: bShowRopeWhenLoaded, at the end of
	// this block through SetShowRopeWhenLoaded, and RopeMaterial, through SetMaterial in step [5].
	// (Instance wiring such as TipMeshComponentTag is not part of a preset, and LoadedHandSocket is only
	//  overwritten on opt-in — see the bOverrideLoadedHandSocket branch below.)
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

	// Through the setter: reapplying while already Loaded means EnterLoaded in [7] is not an entry edge and
	// will not call OnEnterLoaded again, so a direct assignment would stay buried until the next Loaded.
	SetShowRopeWhenLoaded(Preset->bShowRopeWhenLoaded);

	// [4] Reseed the sim — always, with no branch. This consumes NumParticles and RopeLength and bumps the
	// GPU resident buffer's reseed generation. EnsureRopeInitialized is irrelevant here: it only acts when empty.
	InitRope();

	// [5] Render — RopeMaterial has to go through SetMaterial, because the scene proxy captures the material
	// when it is built and only a MarkRenderStateDirty rebuild shows the change. The other values the proxy
	// captures once (Radius, NumSides, TubeSmoothing*) ride the same MarkRenderStateDirty — this is the
	// runtime equivalent of the editor's PostEditChangeProperty.
	SetMaterial(0, Preset->RopeMaterial);

	// [6] Rebuild the tip — EnsureTipMesh is a no-op when a component already exists, so a swapped asset or an
	// on/off change needs a teardown first. Only a tip we spawned is destroyed; a component adopted by tag is
	// preserved, and Ensure re-adopts it if it is still wanted.
	TeardownSpawnedTipMesh();
	EnsureTipMesh();

	// [7] Align mode and phase — a GuaranteedWrap rope can only throw from Loaded, so load it immediately,
	// the same convention BeginPlay uses. Conversely, a rope that was Loaded but is switched to another mode
	// has nothing to be loaded for, so deploy it (restoring visibility and length) and return it to Free.
	if (ResolveMode == ERopeWrapResolveMode::GuaranteedWrap)
	{
		EnterLoaded();
	}
	else if (bWasLoaded)
	{
		OnDeployFromLoaded();
		SetPhase(ERopePhase::Free, TEXT("preset applied"));
	}

	// [8] Notify — native hook first, then the Blueprint delegate, following the engine's Notify convention.
	NotifyPresetApplied(Preset);
	OnPresetApplied.Broadcast(Preset);
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] preset '%s' applied (mode=%d, N=%d, L=%.0f)."),
		*GetName(), *Preset->GetName(), (int32)ResolveMode, NumParticles, RopeLength);
	return true;
}

#pragma endregion Public_API

#pragma region Simulation_Frame_Pipeline

// ===== Simulation frame (the subsystem runs these three stages) ================

void URopeComponent::PrepareSimFrame(float DeltaTime, const TOptional<FVector>& LODCameraLocation)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_Prepare);
	// (Stage contract — this is the "produce solve input" step, so everything that does not need the solve
	//  result belongs here. The reasoning and the three-way split are on the Prepare/Solve/Finalize
	//  declarations in the header.)

	EnsureRopeInitialized();
	// Frame scope — start collecting this frame's logic output.
	SimFrame.OverrideFrame.Reset();
	SimFrame.bForceNonStretchThisFrame = false;
	const ERopePhase PhaseAtPrepareStart = Phase;
	bEnteredFlightDuringPrepareThisFrame = false;
#if WITH_GAMEPLAY_DEBUGGER
	// Fallback — normally the subsystem already captured this in phase 1a, ahead of the pending aim throw, and
	// this call is a no-op. Only a rope that skipped phase 1a records it here for the first time.
	CaptureDebugFrameStartPhase();
#endif

	// Advance the pinned start target. The solver sweeps Prev → Target across the substeps, so fast character
	// movement cannot yank the chain into a runaway.
	if (Sim.bStartPinned)
	{
		Sim.StartPinPrev = Sim.StartPinTarget;
		Sim.StartPinTarget = GetComponentLocation();
	}

	// The collider snapshot is gathered centrally by RopeSimSubsystem during the tick's collider stage and
	// left in FrameColliders, before Prepare runs. Nothing here searches or gathers providers per rope.

	// Reeling: apply this frame's length change before the solve — the segment rest lengths change uniformly,
	// which needs no reseed and works the same on CPU and GPU. While Wrapped it shortens the available rope
	// length, which propagates into the tether and the tension.
	UpdateReel(DeltaTime);

	// Distance LOD multiplier (iteration falloff) — resolve this frame's value before the solve decision, since the GPU step and the CPU solve share it.
	ComputeSolverLOD(LODCameraLocation);

	// Sleep applies to Free and Wrapped only, and is dropped the moment the rope transitions — a transition is itself activity.
	if (Phase != ERopePhase::Free && Phase != ERopePhase::Wrapped && Throttle.IsAsleep())
	{
		Throttle.Wake();
	}

	SimFrame.bSolveThisFrame = false;
	SimFrame.bSolveCollisionsThisFrame = true;

	switch (Phase)
	{
	// Hangs from the character's hand and follows it.
	case ERopePhase::Free:
		if (Throttle.IsAsleep() && Throttle.ShouldWakeFromSleep(Sim, SolverConfig, ReelRate, SimFrame.FrameColliders))
		{
			Throttle.Wake();
		}
		// Asleep, so the solve is skipped; a GPU rope does not dispatch at all.
		SimFrame.bSolveThisFrame = !Throttle.IsAsleep();
		break;

	case ERopePhase::Flight:
	{
		if (WhipGuide.IsActive())
		{
			// Targets and masks only, leaving Sim untouched. Both ordinary and aim-hit guides preserve their
			// own target construction, then sweep those targets across the solver's fixed substeps.
			WhipGuide.Advance(DeltaTime, Sim, MakeWhipGuideConfig());
		}
		else
		{
			WhipGuide.ResetFrameOutputs();
		}

		if (AimTargeting.IsLockActive(Phase))
		{
			// Once the guided middle is applied, XPBD distance, bending and damping connect the two ends
			// naturally. Collider push-out stays off behind its own gate so the instantaneous displacement from
			// the earlier collision cannot reappear.
			SimFrame.bSolveThisFrame = true;
			SimFrame.bSolveCollisionsThisFrame = !WhipConfig.bAimHitCollisionFreeSolve;
		}
		else
		{
			// In an ordinary Flight, contact detection still happens in Finalize, after the solve.
			SimFrame.bSolveThisFrame = true;
		}
		break;
	}

	case ERopePhase::Contacting:
		UpdateContacting(DeltaTime);
		break;

	case ERopePhase::Wrapping:
		UpdateWrapping(DeltaTime);
		// Only the nodes the wrapping path has actually reached are pinned by position override and the mass
		// mask. The solver keeps working the tail beyond the front, which is what lets the strain left over
		// from Flight resolve during Wrapping.
		SimFrame.bSolveThisFrame = (Phase == ERopePhase::Wrapping || Phase == ERopePhase::Wrapped);
		break;

	case ERopePhase::Wrapped:
	{
		// A Wrapped tick is a fixed four-step sequence:
		//  1) follow the bone (Hold plus the mass mask — releases when the target is gone)
		//  2) compute the observations (tension, pull sample and smoothing — the shared input of 3 and 4)
		//  3) apply traction (tether plus active pull)
		//  4) check for auto-release (tension or distance over threshold — consumes the output of 2 and 3).
		if (!HoldWrappedNodesToBone(DeltaTime))
		{
			// The target mesh is gone and the release has completed. No solve.
			break;
		}
		// The hang grip rides the same override frame as the hold, so the observation view below and the
		// GPU packing both see it.
		ApplyHangGripPinOverride();
		// Hold records the current bone positions into OverrideFrame only; applying them to Sim is deferred to
		// the single ApplyToSim at the end of Prepare. So that pull and tether do not read the previous
		// frame's solved pose, a separate observation view is composed first from the current hand pin plus
		// the Hold override. The free nodes inside it may still be a GPU mirror, but the movement hard
		// constraint reads the live CPU binding rather than this view.
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
			// Released on tension or distance. No solve.
			break;
		}
		// Wrapped rest throttling, an extension of the Free sleep: when the pin, the wrapped bone and every
		// nearby collider are all still, the free span's solve pauses. Steps 1 to 4 above — hold, observation,
		// traction, auto-release — keep running through sleep; what is skipped is only the solver's substeps,
		// so the GPU drops to an override-only dispatch (NumSub = 0).
		// Entry is shared with Finalize's UpdateSleepState (velocity settling, held off by bHoldAwake while an
		// active pull is engaged). A wrapped bone moving wakes the rope through the node drift Hold writes,
		// and because the observation view (PullObservationSim) has already composed this frame's pin and
		// override, it is noticed without delay — the frame the lift departs is the frame it wakes.
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
		// GuaranteedWrap only. The physics solver and contact detector are skipped, and the rope follows either the resolved path (aimed) or an arc (open space).
		UpdateGuidedThrow(DeltaTime);
		SimFrame.bSolveThisFrame = false;
		break;

	case ERopePhase::Loaded:
	{
		// The ready state, GuaranteedWrap only: the tip is pinned to the hand socket and the rope between hangs
		// under physics. Node 0 already follows the hand through the pin logic above (StartPinTarget), so
		// pinning the tip as well leaves the rope sagging between the two.
		// Why the tip needs pinning at all: at the moment of the throw the tip's render source switches from
		// the socket to the last node (UpdateTipMeshTransform), so the last node must already be at the socket
		// or the tip pops. The solve stays on so the rope follows the character instead of freezing.
		// (Same pattern as Hold under Wrapped: a Position plus InvMass = 0 override, and the solver runs the rest.)
		// MakeLoadedTipBaseWorld is the hand socket composed with LoadedTipRelativeTransform. It must be the
		// *same* base UpdateTipMeshTransform's Loaded branch uses — offsetting only one of the two would leave
		// the tip and the rope end apart.
		const int32 LoadedTipNode = Sim.Num() - 1;
		const FTransform LoadedTipWorld = MakeLoadedTipBaseWorld();
		SimFrame.OverrideFrame.EnsureSize(Sim.Num());
		SimFrame.OverrideFrame.SetPosition(LoadedTipNode, ResolveTipRopeAttachWorld(LoadedTipWorld), /*bZeroVelocity*/ true);
		SimFrame.OverrideFrame.SetInvMass(LoadedTipNode, 0.0f);
		SimFrame.bSolveThisFrame = true;
		break;
	}

	case ERopePhase::Releasing:
		// Hand every node back to the solver, keeping only the hand pin: restore InvMass and set Prev = Pos so
		// nothing is flung, recorded as this frame's output. Free simulation resumes once the cooldown ends.
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

	// Only a frame that was already in Flight when Prepare began had a proper Flight advance and solve behind
	// it. Returning to Flight during the logic phases defers Finalize's recapture, so the guide restarts on
	// the next frame first.
	bEnteredFlightDuringPrepareThisFrame =
		PhaseAtPrepareStart != ERopePhase::Flight && Phase == ERopePhase::Flight;

	// Apply the logic phase's frame output to the CPU Sim in one pass. The result matches the old "write
	// directly from the handler" behaviour: where two writes hit the same node, the later one wins, exactly as
	// a sequential write would. For a GPU-resident rope the subsystem loads the same frame into the override
	// pass and the kernel applies it.
	// A logic phase must not reseed by bumping SimFrame.SimGeneration — the only reseeds are the real ones,
	// from Init and Throw.
	if (SimFrame.OverrideFrame.HasAny())
	{
		SimFrame.OverrideFrame.ApplyToSim(Sim);
	}
}

void URopeComponent::SolveSimFrame(float DeltaTime)
{
	// Parallel stage: it touches only POD state (Sim) and the collider snapshot (SimFrame.FrameColliders), and
	// Query is const, so it is thread-safe.
	// Physics solves only when SimFrame.bSolveThisFrame (Free, Flight, Wrapping, Wrapped). Under Wrapping and
	// Wrapped the pinned nodes have InvMass = 0, so only the free span moves; Contacting and Releasing are
	// logic-driven and skipped.
	if (!SimFrame.bSolveThisFrame)
	{
		return;
	}
	TRACE_CPUPROFILER_EVENT_SCOPE(Rope_Solve);

	// Carry the whip as a substep kinematic path on the CPU fallback, matching the GPU KinematicPath override.
	// Applying Current/Prev once before the loop would integrate one frame of guide displacement again on every
	// substep, overshoot the guide, and reset on the next frame.
	FRopeKinematicTargetFrame WhipKinematicTargets;
	const FRopeKinematicTargetFrame* KinematicTargets = nullptr;
	if (Phase == ERopePhase::Flight)
	{
		// Aim-hit keeps its endpoint solver-state blend in the targets themselves. Once built, those targets
		// follow the same per-substep kinematic path as the ordinary guide, preventing a frame displacement
		// from being integrated again by every substep.
		WhipGuide.ApplyToSim(Sim);
		WhipKinematicTargets.Mask = MakeArrayView(WhipGuide.GetGuidedNodeMask());
		WhipKinematicTargets.PrevTargets = MakeArrayView(WhipGuide.GetPrevTargets());
		WhipKinematicTargets.CurrentTargets = MakeArrayView(WhipGuide.GetCurrentTargets());
		if (WhipKinematicTargets.IsValidFor(Sim.Num()))
		{
			KinematicTargets = &WhipKinematicTargets;
		}
	}

	// Distance LOD: fall off the constraint iterations at range only. The substep count is untouched, because stability is dominated by the substeps.
	FRopeSolverConfig LODConfig = SolverConfig;
	LODConfig.Iterations = GetLODScaledIterations();
	LODConfig.MaxStretchRatio = GetEffectiveMaxStretchRatio();
	// Resolve the auto radius (0 = the render Radius). The solver only ever receives the resolved value; the subsystem does the same for the GPU step.
	LODConfig.CollisionRadius = GetEffectiveCollisionRadius();
	// A collision-free aim solve still runs the solver, and excludes only the push-out by passing an empty collider list.
	const TArray<IRopeCollider*> NoSolveColliders;
	const TArray<IRopeCollider*>& SolveColliders = SimFrame.bSolveCollisionsThisFrame
		? SimFrame.FrameColliders
		: NoSolveColliders;
	Solver.Step(Sim, LODConfig, SolveColliders, DeltaTime, KinematicTargets);
}

float URopeComponent::GetEffectiveMaxStretchRatio() const
{
	// A physical throw feeds the solver guide targets and wrapping position overrides as input.
	// Leaving any stretch allowance in this span would let the rope snap back to its rest length the moment
	// the kinematic nodes are released, so the strain-limit stage of the actual resident solve — CPU and GPU
	// alike — pins it to fully inextensible.
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

	// (Stage contract — this is the "consume solve output" step. The reasoning is on the third declaration in the header.)
	// Debug capture gate: visual data is only collected while this rope is the gameplay debugger's target
	// actor, so a rope that is not the target pays nothing for captures such as the Flight sweep — the cost is
	// charged to the single targeted rope.
#if WITH_GAMEPLAY_DEBUGGER
	URopeDebugSubsystem* DebugSub = URopeDebugSubsystem::Get(GetWorld());
	const bool bDebugCapture = DebugSub && DebugSub->ShouldCapture(this);
	// Collect only the views that are switched on. Without the Flight bit in particular, the observation step
	// below skips its own per-node resweep entirely; merely not drawing it would still leave that cost.
	const ERopeDebugCapture CaptureMask = bDebugCapture ? DebugSub->GetCaptureMask() : ERopeDebugCapture::None;
	FRopeDebugSnapshot DebugSnapshot;
	FRopeDebugSnapshot* const FlightSnapshot =
		EnumHasAnyFlags(CaptureMask, ERopeDebugCapture::Flight) ? &DebugSnapshot : nullptr;
#else
	constexpr bool bDebugCapture = false;
	FRopeDebugSnapshot* const FlightSnapshot = nullptr;
#endif

	// Flight: after the solve, detect contact candidates along the motion path and decide on a capture. The
	// pipeline itself is FRopeFlightContactDetector, which has no UObject dependency; what happens here is the
	// three-step orchestration around it. Every stat and debugger consumer lives in step 3, so the body below
	// carries only the decision flow.
	if (Phase == ERopePhase::Flight && !bEnteredFlightDuringPrepareThisFrame)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Rope_FinalizeFlight);
		const FRopeFlightContactDetector::FParams DetectParams = MakeFlightDetectParams(DeltaTime);

		// 1) Build the candidates.
		TArray<FRopeContactCandidate>& Candidates = GetOrBuildFlightContactCandidates(DeltaTime, DetectParams);

		// 2) Evaluate once, so the game transition and the observation read the same tracker.
		FRopeFlightCaptureEvaluation CaptureEvaluation = EvaluateFlightCapture(Candidates, DetectParams);
		const bool bShouldCapture = ApplyFlightCaptureEvaluation(DeltaTime, Candidates, CaptureEvaluation);
		const FRopeContactTracker& FrameTracker = bShouldCapture
			? ContactTracker
			: CaptureEvaluation.Tracker;
		// 3) Observe.
		RecordFlightObservation(DetectParams, Candidates, FrameTracker, bShouldCapture, FlightSnapshot);
	}

	// The taut-hold presentation oscillator has to advance even on a sleeping Wrapped frame, so the thrum
	// decays and the blend releases in real time rather than freezing with the solve.
	UpdateTautPresentation(DeltaTime);

	// Push render data only on frames where the centerline, the GPU source or the component transform actually
	// changed. A rope at rest — Free or Contacting, with no solve, no override and an unchanged transform —
	// issues no render command at all.
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

	// Follow the tip attachment to the resolved free-end position (still the consume-solve-output step).
	UpdateTipMeshTransform();

	// Wrapped stat counters, independent of the above.
	if (Phase == ERopePhase::Wrapped)
	{
		RopeDebug::RecordWrappedStats(Sim, WrapController.State);
	}

	// Sleep transition measurement (Free and Wrapped). It belongs here because it reads the node displacement
	// between frames, which is only final at this point.
	// Under Wrapped, entry is blocked while an active pull is engaged (bHoldAwake) — the traction impulse and
	// climb-in both assume continuous integration.
	if (Throttle.UpdateSleepState(Phase, Sim, SolverConfig, DeltaTime,
		/*bHoldAwake*/ Phase == ERopePhase::Wrapped && PullDrive.ActivePullForce > 0.0f))
	{
		UE_LOG(LogDynamicRope, Verbose, TEXT("[%s] rope asleep (max speed < %.1f cm/s for %.2fs)"),
			*GetName(), SolverConfig.SleepVelocityThreshold, SolverConfig.SleepDelay);
	}

	// Submit the debug snapshot: fill the centerline, collider and Wrapped fields shared by every view, then hand it to the debugger archive.
#if WITH_GAMEPLAY_DEBUGGER
	if (bDebugCapture)
	{
		FillDebugSnapshot(DebugSnapshot, CaptureMask);
		DebugSub->SubmitSnapshot(this, MoveTemp(DebugSnapshot));
	}
#endif
}

int32 URopeComponent::GetFirstWrappedNodeIndex() const
{
	int32 First = INDEX_NONE;
	for (const FRopeSurfaceAnchor& Anchor : WrapController.State.Anchors)
	{
		if (Anchor.NodeIndex != INDEX_NONE && (First == INDEX_NONE || Anchor.NodeIndex < First))
		{
			First = Anchor.NodeIndex;
		}
	}
	for (const FRopeLatchNode& Latch : WrapController.State.Latched)
	{
		if (Latch.NodeIndex != INDEX_NONE && (First == INDEX_NONE || Latch.NodeIndex < First))
		{
			First = Latch.NodeIndex;
		}
	}
	return First;
}

bool URopeComponent::ApplyTautPresentationShaping(TArray<FVector>& WorldPoints, bool bIncludeThrum) const
{
	// The blend can outlast the phase by a fade-out frame or two, hence the phase and span re-checks.
	const int32 EndNode = GetFirstWrappedNodeIndex();
	if (TautPresentationBlend <= UE_KINDA_SMALL_NUMBER || Phase != ERopePhase::Wrapped
		|| EndNode >= WorldPoints.Num())
	{
		return false;
	}
	RopeTautPresentation::FParams Present;
	// A hang grip pin bounds the straightened span from below: the rope between the two hands drapes as
	// solved, and only the leg from the gripping hand up to the wrap is presented taut.
	Present.StartNode = AppliedHangGripPinNode != INDEX_NONE ? AppliedHangGripPinNode : 0;
	Present.EndNode = EndNode;
	Present.Straighten = TautPresentationBlend * TautStraightening;
	Present.ThrumOffset = bIncludeThrum
		? TautPresentationBlend * TautThrumLevel * FMath::Sin(TautThrumPhase)
		: 0.0f;
	return RopeTautPresentation::Apply(WorldPoints, Present);
}

void URopeComponent::SetHangGripPin(USceneComponent* Target, FName Socket, int32 NodeIndex)
{
	// A change of target or node re-pins seamlessly: the override is rewritten from the new source next
	// frame anyway. Only a previously applied *different* node needs its mass handed back.
	if (AppliedHangGripPinNode != INDEX_NONE && AppliedHangGripPinNode != NodeIndex)
	{
		PendingHangGripUnpinNode = AppliedHangGripPinNode;
		AppliedHangGripPinNode = INDEX_NONE;
	}
	HangGripPinTarget = Target;
	HangGripPinSocket = Socket;
	HangGripPinNode = NodeIndex;
}

void URopeComponent::ClearHangGripPin()
{
	if (AppliedHangGripPinNode != INDEX_NONE)
	{
		PendingHangGripUnpinNode = AppliedHangGripPinNode;
	}
	HangGripPinTarget = nullptr;
	HangGripPinSocket = NAME_None;
	HangGripPinNode = INDEX_NONE;
	AppliedHangGripPinNode = INDEX_NONE;
}

void URopeComponent::ApplyHangGripPinOverride()
{
	// The one-shot restore after a clear mid-Wrapped: hand the whole between-hands span (the draped
	// interiors plus the grip node) back to the solver with zero velocity, so the release does not
	// fling it. A release out of Wrapped restores every node's mass anyway. The override frame's own
	// index guard covers a node count that shrank in between.
	if (PendingHangGripUnpinNode != INDEX_NONE)
	{
		SimFrame.OverrideFrame.EnsureSize(Sim.Num());
		for (int32 Index = 1; Index <= PendingHangGripUnpinNode; ++Index)
		{
			SimFrame.OverrideFrame.SetInvMass(Index, 1.0f);
			SimFrame.OverrideFrame.SetPrevFromPosition(Index);
		}
		PendingHangGripUnpinNode = INDEX_NONE;
	}

	USceneComponent* Target = HangGripPinTarget.Get();
	if (!Target || HangGripPinNode == INDEX_NONE)
	{
		AppliedHangGripPinNode = INDEX_NONE;
		return;
	}
	// Strictly between the hand and the first wrapped node, or there is nothing to pin. Clamped every
	// frame because a reel or a re-wrap moves the first wrapped node.
	const int32 FirstWrapped = GetFirstWrappedNodeIndex();
	const int32 Node = FMath::Clamp(HangGripPinNode, 1, FirstWrapped - 1);
	if (FirstWrapped <= 1 || !Sim.Positions.IsValidIndex(Node))
	{
		AppliedHangGripPinNode = INDEX_NONE;
		return;
	}
	SimFrame.OverrideFrame.EnsureSize(Sim.Num());
	// A clamp that lands on a different node than last frame frees the old span first; the new one is
	// rewritten below in the same frame, so only the difference actually changes hands.
	if (AppliedHangGripPinNode != INDEX_NONE && AppliedHangGripPinNode != Node)
	{
		for (int32 Index = 1; Index <= AppliedHangGripPinNode; ++Index)
		{
			SimFrame.OverrideFrame.SetInvMass(Index, 1.0f);
			SimFrame.OverrideFrame.SetPrevFromPosition(Index);
		}
	}
	const FVector GripWorld = Target->GetSocketLocation(HangGripPinSocket);
	SimFrame.OverrideFrame.SetPosition(Node, GripWorld, /*bZeroVelocity*/ true);
	SimFrame.OverrideFrame.SetInvMass(Node, 0.0f);

	// The interiors between the two hands are draped kinematically rather than simulated: a nearly
	// taut sub-arm's-length chain pinched between two animation-driven ends has no resolvable
	// dynamics at this node density — the solver only shivers there, and the tube renders every
	// twitch. Each node sits on the hand-to-grip chord plus a parabolic sag (the hanging-chain
	// approximation sag = sqrt(3·chord·slack/8)), so the span follows the hands exactly and settles
	// the moment they do.
	const FVector HandWorld = GetComponentLocation();
	const float SpanRest = Node * Sim.SegmentLength;
	const float ChordLen = FVector::Dist(HandWorld, GripWorld);
	const float Slack = FMath::Max(SpanRest - ChordLen, 0.0f);
	const float Sag = FMath::Sqrt(3.0f * ChordLen * Slack / 8.0f);
	for (int32 Index = 1; Index < Node; ++Index)
	{
		const float Frac = static_cast<float>(Index) / static_cast<float>(Node);
		FVector Draped = FMath::Lerp(HandWorld, GripWorld, Frac);
		Draped.Z -= Sag * FMath::Sin(UE_PI * Frac);
		SimFrame.OverrideFrame.SetPosition(Index, Draped, /*bZeroVelocity*/ true);
		SimFrame.OverrideFrame.SetInvMass(Index, 0.0f);
	}
	AppliedHangGripPinNode = Node;
}

void URopeComponent::UpdateTautPresentation(float DeltaTime)
{
	// Active only while a Wrapped hold is taut and there is a shapeable span: at least one interior node
	// between the hand and the first wrapped node. bChainTaut already carries the whole-chain taut latch
	// (chord sum against rest plus the leg sag limit), so by construction the span is near its chord and
	// the straightening moves nodes at most a few centimetres.
	const bool bActive = bTautPresentation && Phase == ERopePhase::Wrapped
		&& PullDrive.bChainTaut && GetFirstWrappedNodeIndex() >= 2;

	// The pluck: the frame the chain snaps taut starts the thrum at full amplitude.
	if (bActive && !bWasTautPresentationActive)
	{
		TautThrumLevel = TautThrumAmplitude;
		TautThrumPhase = 0.0f;
	}
	bWasTautPresentationActive = bActive;

	// Engage faster than release: the snap should read immediately, the let-off softly. The release
	// rate is also a stability rate: the taut gate flickers off for a tenth of a second at a swing
	// apex, and everything sitting on the shaped curve — the tube and the hang-pose grip targets —
	// slides toward the solved curve for exactly as far as the blend falls in that window. 3/s keeps
	// the dip under a tenth of the sag instead of half of it.
	TautPresentationBlend = FMath::FInterpTo(TautPresentationBlend, bActive ? 1.0f : 0.0f,
		DeltaTime, bActive ? 10.0f : 3.0f);
	TautThrumPhase += DeltaTime * 2.0f * UE_PI * RopeTautPresentation::ThrumFrequencyHz;
	TautThrumLevel *= FMath::Exp(-RopeTautPresentation::ThrumDecayPerSecond * DeltaTime);
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

	// Acquire the tip attachment, only while bUseTipMesh is on. Its lifetime is BeginPlay to EndPlay
	// regardless of latching model or resolve mode — even in Free the tip has to be acquired here to be
	// visible on the rope's end. It does not read the sim, so it has no ordering dependency on initialization.
	EnsureTipMesh();

	// A GuaranteedWrap rope starts ready (Loaded), waiting with the tip in hand and the rope hidden.
	if (ResolveMode == ERopeWrapResolveMode::GuaranteedWrap)
	{
		EnterLoaded();
	}
}

void URopeComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// When a rope holding something is destroyed or removed, a release event fires so consumers — a capture
	// counter, a ragdoll recovery — are not left stuck. On a world-wide teardown (quit, level transition,
	// closing the editor) the subscribers are dying too, so it is skipped.
	if (EndPlayReason == EEndPlayReason::Destroyed || EndPlayReason == EEndPlayReason::RemovedFromWorld)
	{
		if (Phase == ERopePhase::Wrapped)
		{
			// A committed wrap is being destroyed: per-instance plus the central signal, since a cross-actor target may still be alive and need its ragdoll recovered.
			DispatchReleased(WrapController.State.Mesh.Get(), WrapController.State.BoneName,
				ERopeReleaseReason::Broken, /*bWasWrapped*/ true);
		}
		else if (Phase == ERopePhase::Contacting || Phase == ERopePhase::Wrapping ||
			(Phase == ERopePhase::GuidedThrow && !GuidedThrowState.bFreeThrow))
		{
			// An engagement is being destroyed before it was established: per-instance only, to close the pair.
			// An aimed GuaranteedWrap throw also opened an engagement, from the moment it captured its target,
			// per DispatchReleased's contract. A throw into open space has no target and so opened nothing.
			FName Bone = NAME_None;
			if (Phase == ERopePhase::Wrapping)          { Bone = WrappingPhase.State.BoneName; }
			else if (Phase == ERopePhase::GuidedThrow)  { Bone = GuidedThrowState.Prepared.Bone; }
			else                                        { Bone = ContactTracker.CandidateBone; }
			DispatchReleased(nullptr, Bone, ERopeReleaseReason::Broken, /*bWasWrapped*/ false);
		}
	}

	// Clean up a tip we spawned, leaving an adopted component alone. Its lifetime is BeginPlay to EndPlay, so this is the only teardown.
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

	// Send the centerline in component-local space; the proxy renders it through GetLocalToWorld().
	const FTransform Xform = GetComponentTransform();
	FRopeDynamicData* DynamicData = new FRopeDynamicData;
	// Only a frame the GPU actually stepped may render straight from the resident PosBuf.
	DynamicData->bGpuResident = SimFrame.bGpuSteppedThisFrame;
	// The proxy matches the resident buffer's generation, which is what stops a one-frame ghost after a reseed with an unchanged node count.
	DynamicData->SimGeneration = SimFrame.SimGeneration;
	// The resident tube's world-to-local transform is this same game-thread transform, so it is the same
	// frame's value as the localized points and matches what is drawn. (The proxy's GetLocalToWorld() is one
	// frame behind the moment SetDynamicData ran — see the header comment.)
	DynamicData->WorldToLocal = FMatrix44f(Xform.ToInverseMatrixWithScale());
	// The game-thread frame counter, which the view family also captures at creation; the velocity
	// shader compares the two and outputs deformation velocity only when they match (see
	// FRopeDynamicData::FrameNumber).
	DynamicData->FrameNumber = static_cast<uint32>(GFrameCounter);
	DynamicData->Points.SetNumUninitialized(Sim.Num());
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		DynamicData->Points[i] = Xform.InverseTransformPosition(Sim.Positions[i]);
	}

	// Taut-hold presentation: shape the copy just sent, never Sim itself. The shaping runs on world
	// positions (the thrum's world-up reference matters) through the same helper the hang-pose anim
	// sample uses, so the hand IK targets and the drawn tube stay on the same curve.
	TArray<FVector> ShapedWorld = Sim.Positions;
	if (ApplyTautPresentationShaping(ShapedWorld))
	{
		// A shaped frame must not render from the solver's resident buffer, which still holds the
		// unshaped pose — dropping the flag routes the tube through this CPU upload instead.
		DynamicData->bGpuResident = false;
		for (int32 i = 0; i < ShapedWorld.Num(); ++i)
		{
			DynamicData->Points[i] = Xform.InverseTransformPosition(ShapedWorld[i]);
		}
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
	// The editor fills the sim with a default straight pose too, since the subsystem only ticks in PIE.
	// If it is already filled — after InitRope, or during PIE — it is left alone.
	EnsureRopeInitialized();
}

void URopeComponent::CreateRenderState_Concurrent(FRegisterComponentContext* Context)
{
	Super::CreateRenderState_Concurrent(Context);
	// The proxy has just been created. Even in the editor, or right after a spawn with no tick yet, push the
	// centerline once so BuildTube runs and bHasData becomes true, letting the static draw produce valid
	// geometry. SendRenderDynamicData_Concurrent validates the scene proxy and the sim itself and only then
	// enqueues the render command, so calling it here is safe.
	SendRenderDynamicData_Concurrent();
}

#if WITH_EDITOR
void URopeComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	// Changing NumParticles or RopeLength rebuilds the proxy with the new topology (NumRings) while the sim
	// still holds the old count, so BuildTube would bail on Points.Num() != NumRings and the preview would
	// vanish. EnsureRopeInitialized only acts on an empty sim, so the sim is rebuilt outright here to match.
	// Super then recreates the render state and CreateRenderState_Concurrent pushes the centerline again.
	const FName PropertyName = PropertyChangedEvent.GetPropertyName();
	if (PropertyName == GET_MEMBER_NAME_CHECKED(URopeComponent, NumParticles) ||
		PropertyName == GET_MEMBER_NAME_CHECKED(URopeComponent, RopeLength))
	{
		InitRope();
	}
	else if (PropertyName == GET_MEMBER_NAME_CHECKED(URopeComponent, bTipMeshCollision))
	{
		// Toggled during PIE, this reaches the current tip at once. With no tip it is a no-op and applies on the next acquisition.
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
	// The scene proxy captures the material when it is built, so the proxy is rebuilt to show the change.
	MarkRenderStateDirty();
}

FBoxSphereBounds URopeComponent::CalcBounds(const FTransform& LocalToWorld) const
{
	// Anchor the bounds to the component (the pinned start) with a radius that contains the rope whatever it
	// does: the chain is inextensible, so no particle can be further from the pin than RopeLength plus the
	// tube radius.
	// Deriving the bounds from the per-frame sim points instead would leave the render thread a frame behind,
	// and during fast character motion the rope outruns its tight box and is culled from the shadow and main
	// passes — the shadow disappears while moving and the VSM cache holds a stale afterimage. Anchored to the
	// component transform, the engine moves the bounds with the character through the transform it already
	// tracks, so there is no lag and no false culling.
	const float Reach = RopeLength + Radius + 1.0f;
	return FBoxSphereBounds(LocalToWorld.GetLocation(), FVector(Reach), Reach);
}

#pragma endregion Rendering

#pragma region Phase_State_Machine

// ===== Phase state machine ==================================================

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

	// Extension hook, then the Blueprint event. Real transitions only — a same-phase reset notifies nothing.
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
	// The pull sample, the three EMAs and the warning latches only. For which fields survive, see FRopePullDriveState.
	PullDrive.ResetTransient();
	LengthConstraintState.ResetTransient();
}

#pragma endregion Phase_State_Machine

#pragma region Initialization_And_Debug

// ===== Initialization and utilities =========================================

void URopeComponent::InitRope()
{
	// Going over the GPU solver's cap (one thread group, MaxNodes) drops the rope to a CPU solve and CPU tube:
	// a quiet performance cliff. The editor clamps it through ClampMax, and this hard-clamps the Blueprint and
	// C++ paths as well, so the value written into the proxy's NumNodes matches the sim's size — if they
	// disagree, BuildTube is skipped entirely. Going over warns once.
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

	// Pin the start to the component (the hand or socket); the solver sweeps it across the substeps.
	Sim.InvMass[0] = 0.0f;
	Sim.bStartPinned = true;
	Sim.StartPinTarget = Start;
	Sim.StartPinPrev = Start;

	// A full sim rebuild, so the GPU resident buffer must reseed.
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
	// Cap is the initial designer length — reeling out only gives back what was reeled in. Floor is MinRopeLength.
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
	// The camera UObject lookup happens once per frame in RopeSimSubsystem; only the per-rope distance is
	// computed here, against local player 0. No server and no camera means full quality.
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
	// Held during Contacting, Wrapping and Releasing: the wrapping path's build and commit distances are
	// measured in SegmentLength (RopeDistance = index × SegmentLength), so changing the scale underneath
	// would distort the path.
	if (Phase != ERopePhase::Free && Phase != ERopePhase::Flight && Phase != ERopePhase::Wrapped)
	{
		return;
	}
	float NewLength = Sim.RopeLength - ReelRate * DeltaTime;
	// Reel-in stall: an inextensible rope's hand-side leg cannot become shorter than the straight-line
	// hand-to-anchor distance, which is the floor on how much length is needed. Keeping the reel winding
	// against a target too heavy or too stuck to move would accumulate the violation C without bound, and on
	// a simulating target the hard Chaos limit would break every substep, fighting the joints to close a gap
	// it cannot close and shaking the ragdoll — the snare with both arms tied shook exactly that way.
	// Instead it winds like a winch stalling under load: only as much as the object actually comes in. The
	// violation stays bounded to two reel frame steps of slack, the pull bias survives, and the fight is gone.
	// Reeling out, a self-wrap, and a disabled constraint (no live binding) all behave as before.
	if (ReelRate > 0.0f && Phase == ERopePhase::Wrapped)
	{
		FRopeWielderMovementConstraint LiveConstraint;
		if (BuildWielderMovementConstraint(LiveConstraint) && LiveConstraint.AnchorNode > 0 && Sim.Num() >= 2)
		{
			const float RequiredLength = static_cast<float>(
				FVector::Distance(GetComponentLocation(), LiveConstraint.PivotWorld));
			// The constraint limit spans only the hand-side leg — AnchorNode of the Num()-1 segments
			// (MaxDistance = AnchorNode × SegmentLength) — while the reel changes the *full* rope length. So
			// the stall is measured in leg units and converted back: reeling the full length by one step
			// shortens the leg by only LegFraction × step, and a floor stated as a full length must divide by
			// the fraction, or a mid-chain anchor keeps reeling until the leg limit sits chronically below the
			// measured distance — a hard-constraint violation that can never close, felt as shaking.
			const float LegFraction =
				static_cast<float>(LiveConstraint.AnchorNode) / static_cast<float>(Sim.Num() - 1);
			// Slack of two reel frame steps of leg length, at least 1 cm, so ordinary traction — the target following at reel speed — is untouched.
			const float StallSlack = FMath::Max(ReelRate * DeltaTime * 2.0f * LegFraction, 1.0f);
			// A floor meaning "no further", not "reel out": it never lengthens the rope when the violation is already large, since the current length is the cap.
			NewLength = FMath::Max(NewLength,
				FMath::Min((RequiredLength - StallSlack) / LegFraction, Sim.RopeLength));
		}
	}
	SetRopeLength(NewLength);
}

#pragma endregion Rope_Length_Reel_And_LOD
