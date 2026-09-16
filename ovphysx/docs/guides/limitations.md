<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Known Physics Limitations

These are known limitations of the underlying PhysX engine that apply to scenes
simulated by ovphysx, with recommended workarounds. Limitations of Kit-only
features such as force fields or the asynchronous scene-update mode are omitted,
because they are not reachable through ovphysx.

Two solver names recur below. TGS (Temporal Gauss-Seidel, the default) and PGS
(Projected Gauss-Seidel) are the two PhysX solver types, selected per scene on
`physxScene:solverType`; refer to
[Physics Solver](../simulation_setup/physics_scene.md#physics-solver) for how to
author the choice and its iteration counts. "None known" in the last column means
no workaround exists today, not that the entry is unfinished.

Each feature name links to the page that covers how to author it:

| Feature | Limitation | Recommended workaround |
|---------|------------|------------------------|
| [Particle](../simulation_setup/particles.md) / [deformable](../simulation_setup/deformables.md) contact reports | Particles and deformable bodies do not support contact reports. | None known. |
| Simulation-resume determinacy | Replaying from an in-contact state saved mid-run can be nondeterministic; internal contact state persists across steps and is not serialized to USD. | Restart simulations from the beginning for determinism. |
| [Conveyor belts (surface velocity)](../simulation_setup/rigid_bodies.md#surface-velocity-conveyors) | Deformables and particles do not contact conveyor surfaces (fall through); conveyor against [SDF](../simulation_setup/collision.md#sdf-colliders) triangle-mesh dynamics is inadequate. | Use rigid bodies with non-SDF collision geometry for conveyors. |
| Deformable / particle static friction | Static friction is not supported. | Use a sufficiently high dynamic friction. |
| Deformable / particle [friction combine mode](../simulation_setup/rigid_bodies.md#combine-modes) | Friction combine mode is not supported; interactions use the actor's dynamic friction. | None known. |
| Particle roll-off | Particles can roll off flat surfaces perpendicular to gravity due to solver ghost forces. | None known. |
| [Articulation tendons](../simulation_setup/articulations.md#tendons) | Simulation fidelity can be inadequate; incoming joint force is excessively high with nonzero TGS velocity iterations. | Fixed tendons: use [mimic joints](../simulation_setup/articulations.md#mimic-joints). Spatial tendons: apply external forces to links. Force sensing: use TGS with zero velocity iterations. |
| [GPU convex hull vertex/face limit](../simulation_setup/collision.md#mesh-colliders) | GPU-compatible convex hulls are limited to 64 vertices and faces, which can yield a poor approximation. | Use convex decomposition or an [SDF triangle mesh](../simulation_setup/collision.md#sdf-colliders) to capture detail (at higher cost). |
| [Spherical articulation joints](../simulation_setup/joints.md#joint-types) with non-identity COM | Limits and drives may not respond correctly for some joint-state ranges when the link has a non-identity mass frame. | Transform the asset so the prim and [mass frames](../simulation_setup/rigid_bodies.md#mass-properties) coincide (identity mass frame). |
| [TGS velocity iterations](../simulation_setup/physics_scene.md#physics-solver) | The SDK no longer silently converts velocity iterations above four into position iterations (a warning is issued); this can change behavior. | Manually convert velocity iterations above four to position iterations, set zero or very few velocity iterations, or use PGS. |
| [D6 joint drive](../simulation_setup/joints.md#joint-drive) | Does not behave exactly as expected with the TGS solver, and works poorly with TGS plus velocity iterations. | Use PGS for drive issues; with TGS, favor position iterations and use zero velocity iterations. |
| [Articulation loop-closing](../simulation_setup/articulations.md#closed-loops) with D6 joints | Simulation can become unstable. | Increase timesteps-per-second and articulation solver iterations; try PGS and TGS; avoid high drive gains competing with stiff constraints. |
| [Articulation joint friction](../simulation_setup/articulations.md#articulation-joint-friction-and-armature) | Effective joint friction can differ between PGS and TGS; the friction model may not suit all applications. | Emulate velocity-proportional dynamic friction with a joint drive at zero target velocity and suitable damping. |
| [Joint properties (articulation compared to non-articulation)](../simulation_setup/articulations.md#limitations-and-differences) | Certain joint properties are available only when a joint is (or is not) part of an articulation; console warnings indicate mismatches. | Use or avoid articulations depending on the property your simulation relies on. |
| [Articulation joint drive](../simulation_setup/articulations.md#articulation-joint-drive-and-performance-envelope) with TGS | The drive can reach steady state but report a non-zero joint velocity. | Enable `physxScene:enableExternalForcesEveryIteration` (applies external forces per solver step). |

For tuning guidance that works around several of these at once, refer to
[Articulation Stability](articulation_stability.md) and
[Collision Behavior](collision_tuning.md).
