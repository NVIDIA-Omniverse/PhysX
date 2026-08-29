<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# ovphysx

ovphysx is a self-contained library package offering PhysX simulation through a C API and corresponding Python bindings for inclusion in user applications.
It consumes caller-owned ovstage data, runs simulation, and allows reading and writing simulation data with DLPack interoperability.

To get started, refer to the [Quickstart](tutorials/quickstart.md).

> **Note**
>
> - **Maturity**: ovphysx is pre-release software and not yet mature.
> - **USD coexistence**: ovphysx ships an OV namespaced monolithic OpenUSD runtime. It can reuse an already-loaded compatible OV namespaced USD runtime from another OV library, but it does not treat a classic `usd-core` or host USD import as its runtime. In mixed OV processes, register each subsystem's schema paths before the first USD stage open or schema-registry access.
> - **API stability**: Parts of the API are still being completed and may change before 1.0.

ovphysx packages a USD-aware PhysX simulation runtime with tensorized data access. Applications populate scenes through ovstage, attach the stage to ovphysx, and drain committed ordinal ranges into simulation.

The source repository of ovphysx is the [NVIDIA-Omniverse/PhysX GitHub repository](https://github.com/NVIDIA-Omniverse/PhysX/).

External references of contained functionality:
- [Omni PhysX User Guide](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/index.html)
- [PhysX SDK Documentation](https://nvidia-omniverse.github.io/PhysX/)

## Table of Contents

```{toctree}
:maxdepth: 1
:caption: Overview

ovphysx_overview
developer_guide
local_development
changelog
```

```{toctree}
:maxdepth: 1
:caption: ovstage Integration

ovstage_integration
```

```{toctree}
:maxdepth: 1
:caption: Simulation Setup

simulation_setup/physics_scene
simulation_setup/collision
simulation_setup/rigid_bodies
simulation_setup/joints
simulation_setup/articulations
simulation_setup/deformables
simulation_setup/particles
```

```{toctree}
:maxdepth: 1
:caption: Physics Schemas

physics_schemas
```

```{toctree}
:maxdepth: 1
:caption: Tutorials

tutorials/quickstart
tutorials/hello_world
tutorials/tensor_bindings
tutorials/contact_binding
tutorials/cloning
tutorials/render_handoff
tutorials/physx_interop
tutorials/omnipvd_recording
tutorials/source_link_build
```

```{toctree}
:maxdepth: 1
:caption: Guides

guides/performance
guides/collision_tuning
guides/articulation_stability
guides/gripper_tuning
guides/limitations
```

```{toctree}
:maxdepth: 1
:caption: API Reference

python_api
api
```

```{toctree}
:maxdepth: 1
:caption: Legal

SECURITY
```
