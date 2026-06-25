# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Unreal Engine **5.7** project (Windows) whose entire purpose is the **DynamicRope** plugin: a
throwable rope that flies, collides, *wraps* around a skeletal character's bones, and then holds /
pulls / releases. Code comments are written in **Korean** — match that when editing existing files.

Version control is **Perforce**, not git. The working tree is a Perforce client; there is no `.git`.
Do not assume git commands work. Files in the depot are LF but checked out as CRLF on Windows.

## Build & run

There is no test suite or lint step wired up. Build through Unreal Build Tool (adjust the engine
path to your UE 5.7 install):

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

- `Plugins/DynamicRope/` — the real work. Two modules:
  - `DynamicRope` (Runtime) — solver, wrap logic, collision, rendering, component.
  - `DynamicRopeEditor` (Editor, currently an empty module stub).
- `Source/DynamicRopeProject/` — thin game module (game mode + module boilerplate). Depends only on
  `DynamicRope`.
- `Plugins/DynamicRope/Source/DynamicRope/.../PoC/` — throwaway proof-of-concept actors/components.
  Not part of the production path; don't build new features on top of them.
- `Plugins/DynamicRope/Docs/PoC/` — design notes (Korean) defining the post-wrap behavior model.

## Architecture (the big picture)

`URopeComponent` (`RopeComponent.h`) is the **Facade** and the single UE integration point. Attach it
to an actor, call `Throw()`. It owns the sim state, the solver, the wrap controller, and a
**phase state machine** (`ERopePhase`) that decides each frame whether the rope is governed by
*physics* or by *logic*:

```
Free → Flight → Contacting → Wrapped → Releasing → Free
└──── physics (solver) ────┘ └──── logic (wrap controller) ────┘
```

The hard split — **"during the wrap = physics, after the wrap = data + constraints"** — is the
performance core of the design (see `Docs/PoC/01_PostWrapModel.md`).

**Physics side (`FRopeXPBDSolver`, `Solver/`)**: position-based XPBD solver operating *only* on
`FRopeSimState` (a POD chain of particle positions). It has **no UObject dependency** so it stays
unit-testable and portable to a compute shader. Runs only in Flight/Contacting. Uses substeps
("small steps") + distance/bending/collision constraints.

**Logic side (`FRopeWrapController`, `Logic/`)**: everything *after* a wrap is decided. `DecideWrap`
is the physics→logic gate (requires `MinLatchNodes` nodes in sustained contact with one bone for
`WrapDecisionTime`). `BeginWrap` freezes the contact nodes into **bone-local** space; `Hold`
re-places them on the skinned bone each frame so the wrap follows animation; `Pull`/`Release` finish it.

**Collision abstraction (`Collision/`)**: the solver only ever calls `IRopeCollider::Query()` — it
never knows whether the collider is a capsule, a per-bone SDF, or a world distance field.
`IRopeColliderProvider::GatherColliders()` supplies colliders per frame (broad phase happens there).
`URopeBoneCapsuleProvider` is the current v1 provider: builds one capsule per listed bone each frame.
`FCapsuleCollider` is the v1 analytic collider, slated for replacement by per-bone SDF behind the
same interface.

### Two contracts to respect when touching collision/wrap

1. **`FRopeContact` is a FROZEN contract** (`Core/RopeTypes.h`, frozen 2026-06-24). Every
   `IRopeCollider` must obey it exactly. Key invariants: `Normal` is unit and points *outward*
   (collider→node) — the sign is load-bearing, an inward normal sucks the rope into the body;
   `Penetration` is measured against the *query* radius; skeletal colliders must report a non-None
   `Bone` (that's how `DecideWrap` attributes the wrap). Read the struct's comment block before changing it.

2. **Cross-actor wrap**: `FRopeContact::SourceMesh` (and `FCapsuleCollider::SourceMesh`) carry the
   `USkeletalMeshComponent` that owns the contacted bone. This propagates to `FRopeWrapState::Mesh`,
   so a rope owned by actor A can wrap and *follow* a bone on a different actor B (e.g. a rope pinned
   to a static prop tethering a moving character). When wiring colliders from a separate actor, set
   `URopeComponent::ColliderSourceActors`.

### Data types worth knowing (`Core/RopeTypes.h`)

Hot-loop state (`FRopeSimState`, `FRopeContact`, `FRopeLatchNode`, `FRopeWrapState`) is plain POD —
not USTRUCT, not GC-tracked. Only designer-facing config (`FRopeSolverConfig`, `FRopeWrapConfig`,
`FRopeThrowParams`) is `USTRUCT(BlueprintType)`. `FRopeSimState` is the single source of truth shared
by solver, logic, and render.

Project-wide defaults live in `UDynamicRopeSettings` (`Settings/`), an `UDeveloperSettings` editable
under Project Settings → Plugins → Dynamic Rope, persisted to `DefaultGame.ini`.

Rendering: `FRopeSceneProxy` (`Render/`) builds the tube mesh from the centerline;
`URopeComponent::SendRenderDynamicData_Concurrent` pushes updated positions each frame.
