# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Unreal Engine **5.7** project (Windows) whose entire purpose is the **DynamicRope** plugin: a
throwable rope that flies, collides, *wraps* around a skeletal character's bones, and then holds /
pulls / releases. Code comments are written in **Korean** — match that when editing existing files.

Version control is **Perforce**, not git. The working tree is a Perforce client; there is no `.git`.
Do not assume git commands work. Files in the depot are LF but checked out as CRLF on Windows.

## Build & run

There is no lint step wired up. C++ automation tests live in
`Plugins/DynamicRope/Source/DynamicRope/Private/Tests/` (solver, wrap controller, SDF sampler, GPU
solver); run them from the editor's Session Frontend → Automation tab, filtering on `DynamicRope.`.
Build through Unreal Build Tool (adjust the engine path to your UE 5.7 install):

```sh
# Build the editor target (most common)
"<UE>/Engine/Build/BatchFiles/Build.bat" DynamicRopeProjectEditor Win64 Development \
  -Project="C:/Users/Shimwoojin/Perforce/Shimwoojin_Main/DynamicRopeProject/DynamicRopeProject.uproject"
```

Or open `DynamicRopeProject.sln` in Visual Studio and build the `DynamicRopeProjectEditor` config.
If headers/modules change, regenerate project files first (right-click the `.uproject` →
"Generate Visual Studio project files"). The `DynamicRopeProject.sln` is generated — don't hand-edit it.

To run/iterate behavior, open the `.uproject` in the editor and Play.

## Editing rules (enforced by `.editorconfig`)

- **C++ uses tabs, width 4**, and **CRLF** line endings. Saving `.cpp`/`.h` as LF produces phantom
  whole-file diffs and spurious Perforce merge conflicts on the shared stream — keep CRLF.
- `.md` files use LF.

## Module / source layout

- `Plugins/DynamicRope/` — the real work. Three modules (see `DynamicRope.uplugin`):
  - `DynamicRope` (Runtime) — solver, wrap logic, collision, rendering, the component, plus the
    `URopeSimSubsystem` that centrally ticks every rope, `URopeWielderComponent` (gameplay wielder +
    Enhanced Input), `UAnimNotify_RopeThrow`, and the Gameplay Debugger category.
  - `DynamicRopeShaders` (Runtime, loads at `PostConfigInit`) — the GPU compute path: XPBD solver and
    tube builder on RDG (`FRopeGPUSolver`, `FRopeTubeBuilder`, `.usf` in `Shaders/`). **GPU is the
    single runtime path for both solve+detect and tube rendering** — auto-selected when a renderable
    RHI exists (and, for the tube, `NumRings <= 256`), else CPU fallback (cook / `-nullrhi` / server,
    or oversized ropes). No `r.DynamicRope.GPUSolver` / `.GPUTube` toggles anymore. The GPU tube
    generates position + tangent basis + UV (B2-full), so CPU `BuildTube` runs only on the fallback.
    `FRopeXPBDSolver` + CPU `BuildTube` are kept as that fallback + parity/unit-test reference.
    (The CPU `Sim` mirror is still uploaded per frame as the tube's centerline source under Catmull-Rom
    smoothing — removing it needs GPU-side smoothing, a later step.)
  - `DynamicRopeEditor` (Editor) — SDF authoring: a nomad tab (`SRopeSDFAuthoringPanel`), the
    `URopeSDFData` baker/factory/asset-definition, and component visualizers. **Not** an empty stub.
- `Source/DynamicRopeProject/` — thin game module (game mode + module boilerplate). Depends only on
  `DynamicRope`.
- `Plugins/DynamicRope/Docs/PoC/` — design notes (Korean) defining the post-wrap behavior model. The
  throwaway PoC *code* (`Source/DynamicRope/{Public,Private}/PoC/`) has been removed; these notes are
  kept as production design rationale.

## Architecture (the big picture)

`URopeComponent` (`RopeComponent.h`) is the **Facade** and the single UE integration point. Attach it
to an actor, call `Throw()`. It owns the sim state, the solver, the logic-side F-classes, and a
**phase state machine** (`ERopePhase`) — all `Phase` assignments go through `SetPhase()` (uniform
transition log), and the transient per-attempt state (tracker/seed/wrapping state/timer) is dropped
together via `ResetTransientPhaseState()`. The component does **not** tick itself: `URopeSimSubsystem`
drives every registered rope each frame via `PrepareSimFrame` → `SolveSimFrame` (run in parallel
across ropes) → `FinalizeSimFrame`, and gathers colliders once per frame for all of them. Each phase
decides whether the rope is governed by *physics* or by *logic*:

```
Free → Flight → Contacting → Wrapping → Wrapped → Releasing → Free
└─ physics (solver) ─┘ └───────── logic (Logic/ F-classes) ─────────┘
```

The hard split — **"during the wrap = physics, after the wrap = data + constraints"** — is the
performance core of the design (see `Docs/PoC/01_PostWrapModel.md`).

**Physics side (`FRopeXPBDSolver`, `Solver/`)**: position-based XPBD solver operating *only* on
`FRopeSimState` (a POD chain of particle positions). It has **no UObject dependency** so it stays
unit-testable and portable to a compute shader (see `DynamicRopeShaders`). Runs only in Free/Flight;
Contacting/Wrapped/Releasing are logic-driven (no solve). Uses substeps ("small steps") +
distance/bending/collision constraints.

**Logic side (`Logic/`)** — one UObject-free F-class per phase behavior; the component only
orchestrates transitions/broadcasts. UObject context (config, collider snapshot, fallback axes,
owner name for logs) is injected per call as params/context structs, so all of these are
unit-testable without a world:
- `FRopeWhipGuide`: the throw's whip-swing presentation (Flight). Computes guide-curve targets +
  guided-node mask as *data* (GPU-port seam: only its apply-loop becomes a kernel later) and snaps
  guided nodes to them each frame while active.
- `FRopeFlightContactDetector` (stateless, all static): the Flight contact pipeline — actual +
  predicted contact candidates, relative-motion scoring, capture decision. Consumes positions only,
  so it survives the CPU→GPU solver switch. Unit tests in `Tests/RopeFlightContactDetectorTests.cpp`.
- `FRopeWrappingPhase`: the Wrapping phase — owns `FRopeWrappingState` (`.State`), progressive wrap
  path build (AnalyticHelix / SurfaceVectorField), front motion along the path, mass masking,
  commit-readiness, and commit-seed assembly for the handoff to the wrap controller.
- `FRopeWrapController`: everything *after* a wrap is decided. `DecideWrap` is the physics→logic
  gate (requires `MinLatchNodes` nodes in sustained contact with one bone for `WrapDecisionTime`).
  `BeginWrap` freezes the contact nodes into **bone-local** space; `Hold` re-places them on the
  skinned bone each frame so the wrap follows animation (returning `false` if the wrapped mesh was
  destroyed, so the caller releases); `Release` hands the nodes back to the solver. `Pull` is
  declared but currently a stub.

**Collision abstraction (`Collision/`)**: the solver only ever calls `IRopeCollider::Query()` — it
never knows whether the collider is a capsule, a per-bone SDF, or a world distance field.
`IRopeColliderProvider::GatherColliders()` supplies colliders per frame (broad phase happens there).
`URopeBoneCapsuleProvider` is the v1 provider: builds one capsule per listed bone each frame
(`FCapsuleCollider`, analytic). The per-bone SDF path now exists behind the same interface
(`Collision/SDF/`): `URopeSDFProvider` serves `FRopeSDFCollider`s sampling a baked `URopeSDFData`
volume (authored in the editor module). Pick the provider per rope; the solver is identical either way.

### Two contracts to respect when touching collision/wrap

1. **`FRopeContact` is a FROZEN contract** (`Core/RopeTypes.h`, frozen 2026-06-24; extended 2026-06-27
   with an additive `SurfaceVelocity` field — default `ZeroVector`, so backward-compatible). Every
   `IRopeCollider` must obey it exactly. Key invariants: `Normal` is unit and points *outward*
   (collider→node) — the sign is load-bearing, an inward normal sucks the rope into the body;
   `Penetration` is measured against the *query* radius; skeletal colliders must report a non-None
   `Bone` (that's how `DecideWrap` attributes the wrap); `SurfaceVelocity` is the collider surface's
   world velocity (cm/s) at the contact point — the solver uses it for *relative*-tangential friction so
   a moving body drags/sweeps the rope aside (leave it `0` for static colliders; the v1 capsule does,
   the SDF collider derives it from the bone's per-frame motion). Read the struct's comment block before changing it.

2. **Cross-actor wrap**: `FRopeContact::SourceMesh` (and `FCapsuleCollider::SourceMesh`) carry the
   `USkeletalMeshComponent` that owns the contacted bone. This propagates to `FRopeWrapState::Mesh`,
   so a rope owned by actor A can wrap and *follow* a bone on a different actor B (e.g. a rope pinned
   to a static prop tethering a moving character). Because B can be destroyed mid-wrap,
   `FRopeWrapState::Mesh` is a `TWeakObjectPtr` — `Hold` detects the loss and triggers a release
   rather than dereferencing a dangling pointer. Collider gathering is centralized in
   `URopeSimSubsystem` (providers register on BeginPlay/EndPlay; built once per frame). A rope collides
   with *every* registered provider **except its own owner's** (so a thrown rope doesn't tangle on the
   thrower) — cross-actor "just works" since actor B is included; opt back in with
   `URopeComponent::bIncludeOwnerColliders`.

### Data types worth knowing (`Core/RopeTypes.h`)

Hot-loop state (`FRopeSimState`, `FRopeContact`, `FRopeLatchNode`, `FRopeWrapState`) is plain POD —
not USTRUCT, not GC-tracked. (The one nuance: `FRopeWrapState::Mesh` is a `TWeakObjectPtr` for
cross-actor safety — a weak handle, so it still does **not** keep the mesh alive.) Only
designer-facing config (`FRopeSolverConfig`, `FRopeWrapConfig`, `FRopeThrowParams`) is
`USTRUCT(BlueprintType)`. `FRopeSimState` is the single source of truth shared by solver, logic,
and render.

Project-wide defaults live in `UDynamicRopeSettings` (`Settings/`), an `UDeveloperSettings` editable
under Project Settings → Plugins → Dynamic Rope, persisted to `DefaultGame.ini`.

Rendering: `FRopeSceneProxy` (`Render/`) builds the tube mesh from the centerline;
`URopeComponent::SendRenderDynamicData_Concurrent` pushes updated positions each frame.
