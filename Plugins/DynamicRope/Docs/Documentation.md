# DynamicRope — Technical Documentation

Version 1.0 · Unreal Engine 5.5 – 5.8 · Win64

DynamicRope is a fully interactive rope for Unreal Engine: throw it, watch it fly and collide,
and it wraps around whatever it hits — a running character's limbs, a beam, a crate — then holds
on, follows the target's animation, and transmits tension both ways.

---

## Contents

1. [Requirements & Installation](#1-requirements--installation)
2. [Quick Start](#2-quick-start)
3. [Core Concepts](#3-core-concepts)
4. [URopeComponent Reference](#4-uropecomponent-reference)
5. [URopeWielderComponent Reference](#5-uropewieldercomponent-reference)
6. [Configuration](#6-configuration)
7. [Collision & Wrap Targets](#7-collision--wrap-targets)
8. [Pull & Tether](#8-pull--tether)
9. [Events](#9-events)
10. [Debugging & Profiling](#10-debugging--profiling)
11. [Performance Notes](#11-performance-notes)
12. [Automation Tests](#12-automation-tests)
13. [Limitations & FAQ](#13-limitations--faq)

---

## 1. Requirements & Installation

- **Engine**: Unreal Engine 5.5 – 5.8 (developed against 5.7).
- **Platform**: Win64. The GPU simulation/rendering path requires a renderable RHI; a CPU
  fallback is selected automatically when one is not available (dedicated servers, cooking,
  `-nullrhi`).
- **Dependencies**: Enhanced Input (engine built-in plugin; enabled automatically by the
  `.uplugin`).

Install the plugin into `<Project>/Plugins/DynamicRope/` (or engine `Marketplace` folder),
enable **DynamicRope** in *Edit → Plugins*, and restart the editor. Three modules are loaded:

| Module | Type | Purpose |
|---|---|---|
| `DynamicRope` | Runtime | Solver, wrap logic, collision, rendering, components, subsystem |
| `DynamicRopeShaders` | Runtime (PostConfigInit) | GPU compute: XPBD solver + tube builder (RDG) |
| `DynamicRopeEditor` | Editor | SDF authoring tab, `URopeSDFData` baker/factory |

---

## 2. Quick Start

### The two-component setup (recommended)

1. Open your character Blueprint and add two components:
   - **RopeComponent** — the rope itself.
   - **RopeWielderComponent** — gameplay glue: hand attachment, aiming, input.
2. On the wielder, set `Rope` to the RopeComponent (it auto-finds one on the same actor if left
   unset) and check `HandSocketName` (default `hand_r`). With `bAttachOnBeginPlay` enabled the
   rope is attached to that socket on the owner's skeletal mesh automatically.
3. Assign Enhanced Input assets: `MappingContext`, `ThrowAction`, `ReleaseAction`, and
   optionally `PullAction`, `ReelInAction`, `ReelOutAction`, `ReloadAction`. With
   `bAutoBindInput` enabled they are bound during BeginPlay.
4. Play. Press the throw input: the rope flies, and when it hits a character or prop it wraps
   and holds. Press release to let go.

### The minimal setup (no wielder)

Add only a **RopeComponent**, attach it where you want the rope anchored, and call `Throw()` /
`ReleaseWrap()` from your own gameplay code. The aim direction comes from
`ThrowParams.FrameMode`. Use `ThrowWithContext()` to supply an explicit throw frame.

### Presets

`URopePreset` is a data asset that stamps a complete rope setup (solver, throw, wrap, hold
config, tip mesh, length) onto a component in one call: `ApplyPreset(Preset)`. Three example
presets ship in `Content/Presets/`.

---

## 3. Core Concepts

### Phase state machine

Each rope is always in exactly one `ERopePhase`:

| Phase | Driven by | Meaning |
|---|---|---|
| `Free` | physics | Idle / dropped; hangs and collides |
| `Loaded` | — | GuaranteedWrap only: tip held in hand, rope hidden, ready to throw |
| `Flight` | physics | Airborne after `Throw()`; contact candidates are collected |
| `GuidedThrow` | logic | GuaranteedWrap only: follows the committed preview path |
| `Contacting` | logic | Re-collecting contacts each frame; deciding whether to begin a wrap |
| `Wrapping` | logic | Progressively building the wrap path around the target |
| `Wrapped` | logic | Latched; follows the bone, transmits tension, accepts pull |
| `Releasing` | logic | Handing nodes back to the solver |

The core performance idea: **during the wrap = physics, after the wrap = data + constraints.**
While flying, the rope is solved by a substep XPBD solver. The moment a wrap is decided, the
contact nodes are frozen into **bone-local space** and simply re-placed on the skinned bone
each frame — so a wrapped rope follows animation (and ragdoll) at a fraction of the cost of
brute-force physics, and cannot be pushed through the body it wraps.

### Wrap resolve modes

`ERopeWrapResolveMode` (property `ResolveMode` on the component) is the contract for what a
throw guarantees. Everything after a wrap is established (hold, pull, tether, release) is
common to all modes.

| Mode | Hit | Bind | Use for |
|---|---|---|---|
| `FullSimulation` | emergent | emergent | Sandboxes; missing is a normal outcome |
| `AssistedJudged` (default) | guaranteed (aim ray locks the target) | judged by wrap-angle / coverage gates | Combat, skills |
| `GuaranteedWrap` | guaranteed | guaranteed | Demos, scripted sequences, traversal; throw only from `Loaded` |

The wielder derives its aiming behaviour from the rope's mode: `UsesAimRay()` and
`UsesLockedPreview()`.

### GPU / CPU paths

The GPU compute path (XPBD solve + contact detection, and tube mesh generation) is the single
runtime path whenever a renderable RHI exists and the tube has ≤ 512 rings. Otherwise the CPU
solver and CPU tube builder run — same behaviour, chosen automatically, no console variables to
manage.

### Central ticking

Rope components do not tick themselves. `URopeSimSubsystem` drives every registered rope each
frame (prepare → solve, parallel across ropes → finalize) and gathers colliders **once per
frame for all ropes**. This is also where cross-actor collision rules live (see §7).

---

## 4. URopeComponent Reference

`URopeComponent` is the façade and the single UE integration point. Attach it to an actor;
everything else (solver, wrap controller, collision, rendering) hangs off it.

### Key properties

| Property | Default | Meaning |
|---|---|---|
| `ResolveMode` | `AssistedJudged` | Throw contract, see §3 |
| `NumParticles` | 64 | Node count of the simulated chain |
| `RopeLength` | 600 | Total rest length (cm) |
| `MinRopeLength` | 100 | Lower clamp for reel-in |
| `ReelSpeed` | 300 | Reel rate (cm/s) |
| `Radius` / `NumSides` | 2 / 8 | Rendered tube radius (cm) and cross-section sides |
| `TubeSmoothingSubdiv` / `TubeSmoothingAlpha` | 1 / 0.5 | Catmull-Rom centerline smoothing |
| `RopeMaterial` | plugin default | Tube material |
| `SolverConfig` / `ThrowParams` / `WrapConfig` / `HoldConfig` / `WhipConfig` | — | See §6 |
| `bIncludeOwnerColliders` | false | Let the rope collide with its own owner's colliders |
| `bUseWorldGDF` | true | Collide against the world global distance field |
| `bUseTipMesh` / `TipMesh` / `TipMeshRelativeTransform` | — | Optional tip mesh (hook, dart …) |
| `bUseTipMeshSockets` / `TipSocketName` / `TipRopeSocketName` | — | Socket-based tip alignment |
| `LoadedHandSocket` / `LoadedTipRelativeTransform` | — | Hand placement in the `Loaded` phase |
| `bShowRopeWhenLoaded` | false | Show the slack rope while `Loaded` |

### Key functions (BlueprintCallable unless noted)

| Function | Purpose |
|---|---|
| `Throw()` | Throw using `ThrowParams` |
| `ThrowWithContext(Context)` | Throw with an explicit frame/velocity context |
| `EnterLoaded()` | Enter the `Loaded` ready-to-throw phase (GuaranteedWrap flow) |
| `CanThrowNow()` (pure) | Phase/mode gate for throwing |
| `ReleaseWrap()` | Let go of an established wrap (reason `Manual`) |
| `CutRope()` | Sever the rope (reason `Cut`) |
| `ApplyPreset(Preset)` | Stamp a `URopePreset` onto this component |
| `SetActivePull(Force, bIgnoreTautGate)` | Constant pull force on the wrapped target, see §8 |
| `SetRopeLength(NewLength)` / `SetReelRate(CmPerSecond)` | Length control / reeling |
| `GetPhase()` (pure) | Current `ERopePhase` |
| `GetSegmentTension(Index)` / `GetMaxTension()` (pure) | Per-segment / peak tension |
| `GetConstraintTension()` / `GetTetherTension()` (pure) | Whole-chain tether tension |
| `IsPullTaut()` / `IsChainTaut()` (pure) | Taut gates (see §8) |
| `GetPullSample(OutDir, OutTension)` (pure) | Pull direction + tension as data |
| `GetWrappedBoneName()` / `GetWrappedMesh()` (pure) | What the rope is wrapped on |
| `GetCurrentRopeLength()` (pure) | Current rest length |
| `IsSleeping()` / `GetSolverLODScale()` (pure) | Throttling state (see §11) |
| `ConstrainWielderLocation(...)` (pure) | Clamp a proposed wielder location to rope reach |

### Extension hooks

The component exposes `protected virtual` hooks for game-side subclassing on game-thread,
cold-path moments (throw building, wrap commit, release, guaranteed-throw abort via
`ShouldAbortGuaranteedThrow`). Hot-path solver internals are intentionally not virtual.

---

## 5. URopeWielderComponent Reference

The gameplay wielder: attaches the rope to a hand socket, owns aiming, and binds input. A
character needs only this component plus a `URopeComponent` to be fully set up.

### Attachment

| Property | Default | Meaning |
|---|---|---|
| `Rope` | auto | The rope to wield (auto-found on the owner if unset) |
| `AttachMesh` | auto | Skeletal mesh to attach to (owner's mesh if unset) |
| `HandSocketName` | `hand_r` | Socket the rope root is attached to |
| `bAttachOnBeginPlay` | true | Perform the attachment automatically |

### Input (Enhanced Input)

With `bAutoBindInput` enabled, `MappingContext` (at `MappingPriority`) is added to the local
player and these actions are bound on BeginPlay:

| Action | Behaviour |
|---|---|
| `ThrowAction` | Throw. With `bThrowActionToggles`, press again to release |
| `ReleaseAction` | Release / recall |
| `PullAction` | **Armed toggle**: press to arm; the pull engages the moment tension first crosses `PullEngageTension`, playing `PullMontage` once if set |
| `ReelInAction` / `ReelOutAction` | Shorten / lengthen the rope at `ReelSpeed` |
| `ReloadAction` | Return to `Loaded` (GuaranteedWrap flow) |

All of these are also callable directly: `Throw()`, `ThrowNow()`, `ThrowInDirection(Dir)`,
`Release()`, `StartPull()` / `StopPull()` (armed) or `StartPullNow()` / `StopPullNow()`
(immediate), `Cut()`, `StartReelIn()` / `StartReelOut()`, `PlayThrowMontage()`,
`PlayPullMontage()`. `BuildThrowContext(AimDir)` is virtual for custom throw framing.

### Aiming & HUD

`AimRayOriginMode` / `AimRayOriginSocketName` control where the aim ray starts;
`AimRaySweepStep` its sampling. `bShowAimHudWidget` and `bShowPullGaugeWidget` toggle the
built-in HUD widgets, and `bDrivePullGlowMaterial` / `PullGlowParameterName` /
`PullGlowEngagedValue` drive a material scalar on the rope while a pull is engaged.

### Movement while swinging

`bAutoGroundExitOnUpwardPull` lets an upward pull lift the character off the ground;
`bBoostAirControlWhileSwinging` / `SwingAirControl` improve mid-air steering while tethered.

### Animation

`ThrowMontage` / `ThrowMontagePlayRate` and `PullMontage` / `PullMontagePlayRate`. For
animation-driven throws, place the **`AnimNotify_RopeThrow`** notify in a montage: the throw
fires at the notify frame. **`AnimNotifyState_RopePull`** scopes a pull window to a montage
section.

---

## 6. Configuration

All simulation tuning lives in four `USTRUCT(BlueprintType)` config blocks on the component
(and mirrored on `URopePreset`).

### FRopeSolverConfig — the physics

Substep XPBD solver: `Substeps`, `Iterations`, `CollisionPassesPerSubstep`,
`ContactSolveInterval`. Material response: `StretchCompliance`, `MaxStretchRatio` (strain
limiting, default 1.5), `BendCompliance` with `BendReleaseRatio` / `BendFullRatio`. Contact:
`Friction`, `TipFrictionScale`, `CollisionRadius`, plus swept-contact controls `SweepStep` /
`MaxSweepSamples`. World: `Gravity`, `Damping`.

Throttling (see §11): `bAllowSleep`, `SleepVelocityThreshold`, `SleepDelay`,
`bEnableDistanceLOD`, `LODStartDistance`, `LODEndDistance`, `LODMinIterationScale`.

### FRopeThrowParams — the throw

`ThrowSpeed`, `FrameMode` (`ERopeThrowFrameMode` — where the aim frame comes from),
`SwingPlane` / `CustomSwingPlaneNormal` (the whip swing plane), and `OwnerVelocity` /
`HandAnimationVelocity` injection. Aim-guide fields (`AimGuideHitWorldPos`,
`AimGuideSteerStartAlpha`, `AimGuideLockAlpha`, …) are filled by the wielder's aim ray and
steer the flight towards the locked target in the assisted modes.

### FRopeWrapConfig — contact and wrap decisions

- **Contact detection**: `ContactQueryRadius`, `PredictiveContactFrames`,
  `ContactSweepStep` / `ContactMaxSweepSamples`.
- **The physics→logic gate** (`DecideWrap`): a wrap begins only after `MinLatchNodes` nodes
  stay in contact with one bone for `WrapDecisionTime` seconds.
- **Wrapping motion**: the progressive path build and front motion — `WrappingAxisSource`,
  `WrappingMaxWrapAngleDeg`, `WrappingAngularSpeedDegPerSec`, `WrappingMotionDuration`,
  `WrappingHelixPitchScale`, gap bridging (`WrappingMaxGapBridgeDistance`), settle timing
  (`WrappingStableTime`, `WrappingPostFrontSettleTime`, `WrappingMaxSettleTime`),
  `MaxWrapSeeds`, `bEnableMultiBoneWrapping` and the bone-transition scoring weights.
- **Judgement gates** (AssistedJudged): `CommitMinWrapAngleDeg`, `CommitMinWrapCoverageDeg`,
  `FailedWrapMinAngleDeg`.

### FRopeHoldConfig — after the wrap

Length-constraint enforcement per end (`bEnforceWielderLengthConstraint`,
`bEnforceTargetLengthConstraint`, `LengthConstraintActivationSlop`) and the automatic release
thresholds referenced by `ERopeReleaseReason`: distance slack and sustained-tension limits.

### Project settings

`UDynamicRopeSettings` (*Project Settings → Plugins → Dynamic Rope*, persisted to
`DefaultGame.ini`) holds project-wide static-collision budgets: `StaticBodyMaxColliders`,
`StaticBodyMaxCollidersPerRope`, `StaticBodyMaxConvexPlanes`, `bIncludeWorldDynamic`.

---

## 7. Collision & Wrap Targets

The solver only ever talks to the `IRopeCollider` interface; providers implementing
`IRopeColliderProvider` supply colliders each frame. Pick per target:

| Provider | Target | Shape source |
|---|---|---|
| `URopeBoneCapsuleProvider` | skeletal characters | one analytic capsule per listed bone, rebuilt each frame |
| `URopeSDFProvider` | skeletal characters (high fidelity) | baked per-bone SDF volumes (`URopeSDFData`) |
| `URopeStaticBodyProvider` | static props / world | the actor's simple collision: boxes, spheres, capsules, convex |
| `URopeWrapTargetComponent` | any prop meant to be wrapped | simple collision + a derived wrap axis (union long-axis, origin at the shape's center) |

Rules enforced by `URopeSimSubsystem`:

- Providers register on BeginPlay / unregister on EndPlay; colliders are gathered **once per
  frame** for all ropes, with broad-phase AABB culling.
- A rope collides with **every registered provider except its own owner's** — so a thrown rope
  does not tangle on the thrower. Opt back in with `bIncludeOwnerColliders`.
- **Cross-actor wrap works by construction**: contacts carry the source mesh, so a rope owned
  by actor A can wrap and follow a bone on actor B. If B is destroyed mid-wrap the rope
  detects the loss and releases (`Broken`) instead of dereferencing a dead component.
- With `bUseWorldGDF`, ropes additionally collide with the world global distance field, which
  catches level geometry without any provider setup.

### SDF authoring (editor)

*Window → Rope SDF Authoring* opens the authoring panel. Pick a skeletal mesh, bake a
`URopeSDFData` asset (per-bone signed distance volumes), and assign it to a
`URopeSDFProvider` on the character. SDF colliders report accurate surface normals on concave
regions where capsules approximate, and derive per-contact surface velocity from bone motion
so a moving limb drags the rope realistically.

---

## 8. Pull & Tether

Once wrapped, the rope transmits force two ways:

### Passive tether (always on while Wrapped)

A single whole-chain length constraint: one tension impulse per frame, solved from the total
chord-length violation and applied as an equal/opposite impulse pair to both ends. Force
distribution follows inverse effective mass — a light target moves more, a heavy wielder moves
less — and neither end can be winched in or blown up. The `IsChainTaut()` /
`IsPullTaut()` gates expose whether the chain is actually taut.

**Ragdoll targets** are handled by a real engine physics constraint instead (a kinematic
proxy ↔ wrapped-bone anchor with a spherical distance limit), solved by Chaos together with
the ragdoll's own joints — so a tethered ragdoll cannot stretch through the constraint between
frames.

### Active pull

`SetActivePull(Force)` applies a constant, user-set force along the anchor→hand chord:

- If the target can move (simulating bone → character movement → simulating root, tried in
  that order), it is pulled toward the wielder.
- If the target is too heavy or anchored, the same force pulls the **wielder** toward the
  anchor instead — climb / zip-in for traversal.

On the wielder, `PullAction` is an armed toggle gated by `PullEngageTension`, so the pull
kicks in exactly when the rope first goes taut. Tension-proportional forces were deliberately
rejected (they run away); the force is constant by design.

### Interaction receivers

Wrapped bones on simulating bodies receive impulses directly; characters receive movement
input; `URopeRagdollResponseComponent` (optional, add to characters) reacts to wrap/release
events with configurable partial or full ragdoll, using the plugin-wide
`OnAnyRopeWrapped` / `OnAnyRopeReleased` signals.

---

## 9. Events

On `URopeComponent`:

| Delegate | Fires |
|---|---|
| `OnRopeCaptured (Bone)` | Contact captured, wrap attempt starting |
| `OnRopeWrapped (Info)` | Wrap established (bone, mesh, wrap data) |
| `OnRopeReleased (Bone, Reason)` | Engagement ended; `Reason` is an `ERopeReleaseReason` |
| `OnRopePhaseChanged (Old, New)` | Any phase transition |
| `OnPresetApplied (Preset)` | After `ApplyPreset` |

`ERopeReleaseReason`: `Manual`, `Distance` (exceeded length + slack), `Tension` (sustained
over-tension), `Broken` (target lost / wrap failed), `Cut`, `ThrowAborted` (game rule broke a
guaranteed throw — see the `ShouldAbortGuaranteedThrow` hook).

On `URopeWielderComponent`: `OnThrown`, `OnThrowRejected (Reason)`,
`OnAimTargetChanged (Mesh, Bone)`, `OnAimTargetLost`, `OnPullArmedChanged (bArmed)`,
`OnPullEngagedChanged (bEngaged, Tension)`.

---

## 10. Debugging & Profiling

- **Gameplay Debugger** (apostrophe key by default): the *Rope* category draws per-node
  proximity, flight contact candidates, wrap latches and pull state, and collider shapes, with
  per-view toggles. Views that are off cost nothing (capture is scoped, not just drawing).
- **`stat DynamicRope`**: a per-frame dashboard — CPU cost per stage, GPU solve/tube timings,
  render-target and buffer memory, upload bandwidth.
- **Logging**: rope lifecycle transitions are logged with the owner's name; look for the
  `SetPhase` transition lines.

---

## 11. Performance Notes

- The GPU path batches all ropes' solves per stage rather than dispatching per rope; a demo
  scene with dozens of active ropes holds 60 fps.
- **Sleep**: a `Free` rope at rest suspends solving (`bAllowSleep`).
- **Distance LOD**: solver iterations scale down between `LODStartDistance` and
  `LODEndDistance`.
- **Culling**: collider gathering is AABB-culled per rope.
- Tube meshes above **512 rings** (`NumParticles − 1` × `TubeSmoothingSubdiv` subdivision)
  fall back to the CPU tube builder; keep `NumParticles` and `TubeSmoothingSubdiv` moderate
  for very long ropes.
- The wrapped phase is data + constraints, not physics — many simultaneously wrapped ropes
  are cheap. The expensive phases are Flight/Contacting with many colliders.

---

## 12. Automation Tests

C++ automation tests cover the solver, the wrap controller, the flight contact detector, the
SDF sampler, and GPU/CPU parity. Run them from **Session Frontend → Automation**, filter
`DynamicRope.`. Note that headless runs must not use `-nullrhi` for the GPU tests.

---

## 13. Limitations & FAQ

**Is it replicated?** No. Simulation is local. For multiplayer, replicate your own gameplay
events (throw, wrap, release) and let each client simulate locally; the wrap logic is
deterministic enough for cosmetic agreement, but there is no built-in authority model.

**Can two ropes wrap the same target?** Yes; each rope simulates and wraps independently.

**Does it modify engine code?** No. The ragdoll tether uses a standard
`UPhysicsConstraintComponent`; everything else is plugin code.

**Why does my rope ignore its own character?** By design — the owner's colliders are excluded
so a throw does not tangle on the thrower. Set `bIncludeOwnerColliders` to opt back in.

**The rope passes through very fast geometry.** Swept contact detection is controlled by
`SweepStep` / `MaxSweepSamples` (solver) and `ContactSweepStep` / `ContactMaxSweepSamples`
(wrap detection); tighten these for extreme speeds.

**Which collision should wrap targets have?** Simple collision (any shape, convex included).
For skeletal characters, start with `URopeBoneCapsuleProvider` and switch to a baked SDF where
wrap fidelity matters.
