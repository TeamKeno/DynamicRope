# DynamicRope

A throwable, GPU-simulated rope for **Unreal Engine 5** that flies, wraps around skeletal
characters and props, then holds, pulls, or releases. Snare enemies, tether objects, crank
levers, or reel yourself in.

[**Fab**](https://www.fab.com/listings/17f62a8f-387b-48b0-9198-af7273dc3733) · [**Documentation**](https://teamkeno.github.io/DynamicRopeDocs/)

[![DynamicRope demo video](https://img.youtube.com/vi/iRLpnem_LfQ/maxresdefault.jpg)](https://www.youtube.com/watch?v=iRLpnem_LfQ)

---

## What it does

Throw the rope and it flies, collides, and **wraps around whatever it hits** — a running
character's limbs, a beam, a crate. Wrapped nodes are frozen into bone-local space, so the
rope follows the target's animation (and ragdoll) instead of fighting it, and transmits
tension both ways: drag light targets toward you, or zip yourself toward heavy ones.

- **GPU XPBD solver + GPU tube rendering** (RDG compute) with an automatic CPU fallback for
  servers, cooking, and oversized ropes — nothing to configure.
- **Wraps skeletal characters** (per-bone capsules, or baked per-bone SDF volumes authored
  with the included editor tool) **and static geometry**.
- **Stable tension model** — one whole-chain constraint impulse per frame, so neither end
  can be winched in or blown up. Ragdoll targets are tethered through a real Chaos
  physics constraint.
- **Gameplay-ready wielder component** — hand-socket attachment, aim preview, Enhanced Input
  auto-binding (throw / release / pull / reel), montages, and an AnimNotify for
  animation-driven throws.
- Reel in/out, cut, distance & tension auto-release, presets, sleep + distance LOD,
  a `stat DynamicRope` dashboard, and a Gameplay Debugger category.

## Repository layout

| Path | Contents |
|---|---|
| `Plugins/DynamicRope/` | The plugin itself — runtime, GPU shaders, and editor modules |
| `Content/DynamicRope/Demo/` | Demo maps: snare trap, helicopter grab, lever, pressure plate, basket goal |
| `Source/DynamicRopeProject/` | Thin host game module for the sample project |

This repository is the **sample project** that hosts the plugin. To use DynamicRope in your
own game, copy `Plugins/DynamicRope/` into your project's `Plugins/` folder (or install it
from Fab), enable it, and restart the editor.

## Quick start

1. Enable the plugin and restart the editor.
2. On your character Blueprint, add a **RopeComponent** and a **RopeWielderComponent**.
3. On the wielder, assign your Enhanced Input `MappingContext`, `ThrowAction`, and
   `ReleaseAction` (hand attachment defaults to socket `hand_r`).
4. Play and press throw — the rope flies, wraps what it hits, and holds.

No wielder is needed for scripted uses: call `Throw()` / `ReleaseWrap()` / `SetActivePull()`
on the RopeComponent and listen to `OnRopeWrapped` / `OnRopeReleased`.

See the [documentation](https://teamkeno.github.io/DynamicRopeDocs/) for the full setup
guide, phase model, configuration reference, and debugging tools.

## Requirements

Unreal Engine 5.5 – 5.8 · Win64 · Enhanced Input (engine built-in). Local simulation only —
no built-in replication.

## Support

Team Keno — teamkeno0824@gmail.com
