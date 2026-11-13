<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# ovphysx

ovphysx is a library for simulating physics in USD scenes, with a C API and a Python package. USD,
short for [Universal Scene Description](https://openusd.org/), is an open file format for 3D scenes. ovphysx is built
on [NVIDIA PhysX](https://nvidia-omniverse.github.io/PhysX/physx/latest/) and simulates on an NVIDIA GPU or on the CPU. Your program loads a scene,
moves time forward in steps, and reads the results as arrays.

The following animation shows boxes whose motion ovphysx simulated, drawn by an NVIDIA RTX
renderer:

![Six boxes, four of them green, fall onto a dark floor, tumble, and come to rest.](getting_started/images/getting_started_hero.png)

ovphysx is for programs that need physics as data rather than pictures, such as training
robots, running [virtual factories](common_tasks/fast_simulations.md), testing machines in simulation, and adding physics to your own application. It draws
nothing on screen. To see a scene, pass the results to a renderer.

To learn the basics, read [Install](getting_started/install.md),
[Your First Simulation](getting_started/first_simulation.md), and
[Concepts](getting_started/concepts.md) in that order.

```{eval-rst}
.. grid:: 1 1 3 3
   :gutter: 3

   .. grid-item-card:: Install
      :link: getting_started/install
      :link-type: doc
      :img-top: getting_started/images/getting_started_card_install.png
      :img-alt: The pip command that installs ovphysx.

      Install the Python package or the C SDK.

   .. grid-item-card:: Your First Simulation
      :link: getting_started/first_simulation
      :link-type: doc
      :img-top: getting_started/images/getting_started_card_first_simulation.png
      :img-alt: A box resting on the ground after its fall.

      Drop a box onto the ground and print its height.

   .. grid-item-card:: Concepts
      :link: getting_started/concepts
      :link-type: doc
      :img-top: getting_started/images/getting_started_card_concepts.png
      :img-alt: The order of a simulation, from scene to stage to steps to arrays.

      Learn the ideas behind every ovphysx program.
```

Then pick the topic you need from [Building Scenes](building_scenes/loading_scenes.md),
[Simulation Data](simulation_data/selecting_objects.md), and
[Common Tasks](common_tasks/changing_the_scene.md). Each guide is short and links to the
[Detailed Reference](reference/index.md), which describes every feature in full.

```{eval-rst}
.. grid:: 1 1 3 3
   :gutter: 3

   .. grid-item-card:: Building Scenes
      :link: building_scenes/loading_scenes
      :link-type: doc
      :img-top: building_scenes/images/guide_card_building_scenes.png
      :img-alt: A box resting on the ground, a pendulum, and a two-link articulation.

      Describe bodies, colliders, joints, and articulations in USD.

   .. grid-item-card:: Simulation Data
      :link: simulation_data/selecting_objects
      :link-type: doc
      :img-top: simulation_data/images/guide_card_simulation_data.png
      :img-alt: A path pattern selects three boxes on a shelf, and a read returns one row of x, y, and z per box.

      Select objects, then read and write their values.

   .. grid-item-card:: Common Tasks
      :link: common_tasks/changing_the_scene
      :link-type: doc
      :img-top: common_tasks/images/guide_card_common_tasks.png
      :img-alt: Four tiles: a box added to a running scene, a contact force graph, a rendered picture, and a recording timeline.

      Change a running scene, read contacts, debug, and speed up.
```

<div class="ai-ready">
<svg class="ai-ready-icon" viewBox="0 0 24 24" aria-hidden="true"><path d="M10 3.5l1.6 4.4a2 2 0 0 0 1.2 1.2l4.4 1.6-4.4 1.6a2 2 0 0 0-1.2 1.2L10 17.9l-1.6-4.4a2 2 0 0 0-1.2-1.2L2.8 10.7l4.4-1.6a2 2 0 0 0 1.2-1.2z"/><path d="M18.5 14.5l.7 1.8 1.8.7-1.8.7-.7 1.8-.7-1.8-1.8-.7 1.8-.7z"/></svg>
<div>
<p class="ai-ready-title">Ready for AI coding assistants</p>
<p>The Python package and the C SDK ship these guides as Markdown, together with skills: short playbooks that each teach an assistant one task. In Python, <code>python -m ovphysx --skills-path</code> prints where they are installed, so you can point your assistant at them.</p>
</div>
</div>

> **Note**
>
> ovphysx is prerelease software. Until version 1.0, its API can change from one release to the
> next.

The source code of ovphysx lives alongside the PhysX engine in the
[NVIDIA-Omniverse/PhysX GitHub repository](https://github.com/NVIDIA-Omniverse/PhysX/).
These guides describe the physics engine that ovphysx is built on:

- [Omni PhysX User Guide](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/index.html)
- [PhysX SDK Documentation](https://nvidia-omniverse.github.io/PhysX/)

```{toctree}
:hidden:
:maxdepth: 1
:caption: Getting Started

getting_started/install
getting_started/first_simulation
getting_started/concepts
```

```{toctree}
:hidden:
:maxdepth: 1
:caption: Building Scenes

building_scenes/loading_scenes
building_scenes/physics_scene
building_scenes/rigid_bodies
building_scenes/colliders_and_materials
building_scenes/joints
building_scenes/articulations
building_scenes/soft_bodies_and_particles
```

```{toctree}
:hidden:
:maxdepth: 1
:caption: Simulation Data

simulation_data/selecting_objects
simulation_data/reading_values
simulation_data/writing_values
simulation_data/gpu_data
```

```{toctree}
:hidden:
:maxdepth: 1
:caption: Common Tasks

common_tasks/changing_the_scene
common_tasks/contacts
common_tasks/showing_the_simulation
common_tasks/debugging
common_tasks/stable_simulations
common_tasks/fast_simulations
```

```{toctree}
:hidden:
:maxdepth: 1
:caption: Advanced

common_tasks/many_copies
```

```{toctree}
:hidden:
:maxdepth: 1
:caption: Reference

reference/index
python_api
api
changelog
SECURITY
```
