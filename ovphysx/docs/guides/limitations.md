<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# Known Physics Limitations

Known limitations of the underlying PhysX engine that apply to scenes simulated
by ovphysx, with recommended workarounds. (Limitations of Kit-only features such
as force fields or the asynchronous scene-update mode are omitted, as they are
not reachable through ovphysx.)

| Feature | Limitation | Recommended workaround |
|---------|------------|------------------------|
| Particle / deformable contact reports | Particles and deformable bodies do not support contact reports. | — |
| Simulation-resume determinacy | Replaying from an in-contact state saved mid-run can be nondeterministic; internal contact state persists across steps and is not serialized to USD. | Restart simulations from the beginning for determinism. |
| Conveyor belts (surface velocity) | Deformables and particles do not contact conveyor surfaces (fall through); conveyor vs SDF triangle-mesh dynamics is inadequate. | Use rigid bodies with non-SDF collision geometry for conveyors. |
| Deformable / particle static friction | Static friction is not supported. | Use a sufficiently high dynamic friction. |
| Deformable / particle friction combine mode | Friction combine mode is not supported; interactions use the actor's dynamic friction. | — |
| Particle roll-off | Particles can roll off flat surfaces perpendicular to gravity due to solver ghost forces. | — |
| Articulation tendons | Simulation fidelity can be inadequate; incoming joint force is excessively high with nonzero TGS velocity iterations. | Fixed tendons: use mimic joints. Spatial tendons: apply external forces to links. Force sensing: use TGS with zero velocity iterations. |
| GPU convex hull vertex/face limit | GPU-compatible convex hulls are limited to 64 vertices and faces, which can yield a poor approximation. | Use convex decomposition or an SDF triangle mesh to capture detail (at higher cost). |
| Spherical articulation joints with non-identity COM | Limits and drives may not respond correctly for some joint-state ranges when the link has a non-identity mass frame. | Transform the asset so the prim and mass frames coincide (identity mass frame). |
| TGS velocity iterations | The SDK no longer silently converts velocity iterations above four into position iterations (a warning is issued); this can change behavior. | Manually convert velocity iterations >4 to position iterations, set zero/very few velocity iterations, or use PGS. |
| D6 joint drive | Does not behave exactly as expected with the TGS solver, and works poorly with TGS plus velocity iterations. | Use PGS for drive issues; with TGS, favor position iterations and use zero velocity iterations. |
| Articulation loop-closing with D6 joints | Simulation can become unstable. | Increase timesteps-per-second and articulation solver iterations; try PGS and TGS; avoid high drive gains competing with stiff constraints. |
| Articulation joint friction | Effective joint friction can differ between PGS and TGS; the friction model may not suit all applications. | Emulate velocity-proportional dynamic friction with a joint drive at zero target velocity and suitable damping. |
| Joint properties (articulation vs non-articulation) | Certain joint properties are available only when a joint is (or is not) part of an articulation; console warnings indicate mismatches. | Use or avoid articulations depending on the property your simulation relies on. |
| Articulation joint drive with TGS | The drive can reach steady state but report a non-zero joint velocity. | Enable `physxScene:enableExternalForcesEveryIteration` (applies external forces per solver step). |
