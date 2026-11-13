<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Tuning Guides

In-depth advice for speed and stability. The [Stable Simulations](../common_tasks/stable_simulations.md)
and [Fast Simulations](../common_tasks/fast_simulations.md) guides give the short version.

| Page | What It Covers |
| --- | --- |
| [Simulation Performance Guide](../guides/performance.md) | Measuring and improving simulation speed. |
| [Collision Behavior Guide](../guides/collision_tuning.md) | Settings that affect contact quality, including SDF colliders. |
| [Articulation and Robot Simulation Stability Guide](../guides/articulation_stability.md) | Keeping robots stable: time steps, drives, mass ratios, armature and solver order. |
| [Joint Parameter Tuning Example: Robotiq 2F-85](../guides/gripper_tuning.md) | Tuning a real gripper step by step. |
| [Known Physics Limitations](../guides/limitations.md) | Physics features that are not supported yet. |

```{toctree}
:hidden:
:maxdepth: 1

/guides/performance
/guides/collision_tuning
/guides/articulation_stability
/guides/gripper_tuning
/guides/limitations
```
