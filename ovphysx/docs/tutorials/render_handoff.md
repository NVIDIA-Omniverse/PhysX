<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# Rendering Handoff

ovphysx is a headless simulation SDK — it has no built-in renderer.
To visualize simulation state, you read data back through tensor bindings and hand it to an external tool such as [Rerun](https://rerun.io) or ovrtx.

This tutorial shows the pattern using Rerun: populate an ovstage scene from USD and attach it, step the simulation, and stream rigid body transforms to a browser-based viewer.

## Key Idea

Rerun does not read USD.
The sample explicitly logs all visual geometry to Rerun from the simulation data returned by ovphysx tensor bindings:

- **Falling cubes** — logged each frame as `rr.Boxes3D` with positions and quaternions from the `RIGID_BODY_POSE` tensor binding.
- **Ground plane** — logged once as a static flat box for visual context.

This is the core of the render handoff pattern: ovphysx owns the simulation, you log what you need to your visualization tool.

## Two ways to hand off, and what each populates

- **Tensor handoff (this tutorial).** The viewer never reads the Stage — Rerun
  receives transforms from tensor bindings and the sample supplies its own
  geometry. The sample USD is non-instanced, so `domains=PopulationDomain.PHYSICS`
  is enough here; for arbitrary content prefer `ALL`.
- **Shared Stage.** A renderer such as ovrtx draws from the same Stage, so
  populate `ALL` (or `PHYSICS | RENDERING`). See
  [Population domains](../ovstage_integration.md#population-domains)
  for the full contract, the native-instance caveat, sequencing, and cost.

## Prerequisites

- Install ovphysx: `pip install ovphysx`
- Install the [Rerun SDK](https://rerun.io): `pip install rerun-sdk`
- Get the sample from the [source repository](https://github.com/NVIDIA-Omniverse/PhysX/): the Rerun sample lives under `tests/python_samples_extra/visual_rerunio_sample/` and is not part of the `pip install` wheel.

## Sample

```{literalinclude} ../../tests/python_samples_extra/visual_rerunio_sample/visualize_rigid_bodies.py
:language: python
```

## Running

From your ovphysx source checkout:

```bash
cd tests/python_samples_extra/visual_rerunio_sample
uv run visualize_rigid_bodies.py --interactive
```

This installs dependencies, starts a Rerun web viewer at `http://localhost:9090`, pauses for 5 seconds so you can open the URL in your browser, then runs the simulation at real-time speed.

Without `--interactive`, the simulation runs as fast as possible — useful for CI or when you only need the recorded data in the Rerun viewer's timeline.

## Result

After this tutorial, you can stream simulation state from ovphysx to an external viewer — the same pattern applies to any visualization or rendering tool that accepts transform data.
