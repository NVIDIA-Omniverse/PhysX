<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Detailed Reference

These pages describe every part of ovphysx in full detail: each setting, attribute, rule and
limit. They are written for experienced users and coding agents. If you are new to ovphysx,
start with [Install](../getting_started/install.md) and the guides, which link here when you
need more detail.

| Page | What It Covers |
| --- | --- |
| [Overview and Migration](overview.md) | How ovphysx is built and used in depth, and how to move code from one release to the next. |
| [ovstage Integration](../ovstage_integration.md) | How ovphysx shares a stage with your program and other libraries, step by step. |
| [Simulation Setup](simulation_setup.md) | Every USD setting for scenes, bodies, colliders, joints, articulations, soft bodies and particles. |
| [Schemas and Population](schemas.md) | The USD schemas ovphysx reads and how ovstage loads each kind of object. |
| [Read and Write Contract](read_write.md) | Every value you can read and write, with types, units, devices and limits. |
| [Tutorials](tutorials.md) | Complete programs for specific tasks, including older and deprecated APIs. |
| [Tuning Guides](tuning_guides.md) | Performance, collision, robot stability and gripper tuning in depth. |
| [Contributing](contributing.md) | Building ovphysx locally and the coding style of the project. |
| [Glossary](glossary.md) | The words the guides use, each with a short meaning and a link to the guide that explains it. |

The [Python API](../python_api.rst) and [C API](../api.md) references list every call.

```{toctree}
:hidden:
:maxdepth: 1

overview
/ovstage_integration
simulation_setup
schemas
read_write
tutorials
tuning_guides
contributing
glossary
```
