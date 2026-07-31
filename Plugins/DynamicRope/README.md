# DynamicRope

A throwable, GPU-simulated rope for Unreal Engine that wraps around characters and props —
then holds, pulls, and releases. Snare enemies, tether objects, crank levers, or reel
yourself in.

Version 1.0 · Unreal Engine 5.5 – 5.8 · Win64 · by Team Keno

---

## What it does

Throw the rope and it flies, collides, and **wraps around whatever it hits** — a running
character's limbs, a beam, a crate. Wrapped nodes are frozen into bone-local space, so the
rope follows the target's animation (and ragdoll) instead of fighting it, and transmits
tension both ways: drag light targets to you, or zip yourself toward heavy ones.

- **GPU XPBD solver + GPU tube rendering** (RDG compute), with an automatic CPU fallback for
  servers, cooking, and oversized ropes — nothing to configure.
- **Wraps skeletal characters** (per-bone capsules, or baked per-bone SDF volumes with the
  included in-editor authoring tool) **and static geometry** (every simple collision shape,
  convex included).
- **Stable tension model**: one whole-chain constraint impulse per frame — neither end can be
  winched in or blown up. Ragdoll targets are tethered through a real Chaos physics
  constraint.
- **Gameplay-ready wielder component**: hand-socket attachment, aim ray with HUD preview,
  Enhanced Input auto-binding (throw / release / pull / reel), throw & pull montages, an
  AnimNotify for animation-driven throws.
- **Three throw contracts** per rope: full simulation, assisted-but-judged, or guaranteed
  wrap — from sandbox physics to scripted traversal.
- Reel-in/out, cut, distance/tension auto-release, presets, sleep + distance LOD,
  a `stat DynamicRope` dashboard, and a Gameplay Debugger category.

## Quick start

1. Enable the plugin and restart the editor.
2. On your character Blueprint, add a **RopeComponent** and a **RopeWielderComponent**.
3. On the wielder, assign your Enhanced Input `MappingContext`, `ThrowAction`, and
   `ReleaseAction` (defaults handle hand attachment: socket `hand_r`).
4. Play and press throw. The rope flies, wraps what it hits, and holds. Press throw again
   (or release) to let go.

No wielder needed for scripted uses — call `Throw()` / `ReleaseWrap()` / `SetActivePull()`
directly on the RopeComponent, and listen to `OnRopeWrapped` / `OnRopeReleased`.

## Demo content

`Content/Demo/` ships two maps with small, readable gameplay examples wired in Blueprint:
snare trap, rescue helicopter grab, crankable lever, pressure plate, basket goal, and AI
wrap targets.

## Documentation

Full technical documentation (setup, phase model, configuration reference, collision
providers, pull/tether, debugging): see the hosted documentation linked on the product page.

## Modules

| Module | Type | Purpose |
|---|---|---|
| `DynamicRope` | Runtime | Solver, wrap logic, collision, rendering, components |
| `DynamicRopeShaders` | Runtime | GPU compute path (XPBD solve + tube build, RDG) |
| `DynamicRopeEditor` | Editor | Rope SDF authoring tool |

**Dependencies**: Enhanced Input (engine built-in). **Network**: local simulation only — no
built-in replication.

## Support

- C++ automation tests included: Session Frontend → Automation, filter `DynamicRope.`
- Issues and questions: [지원 링크/이메일]
