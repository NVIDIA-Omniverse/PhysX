<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Glossary

This page explains the words that the ovphysx guides use, grouped by topic. Each entry links to
the guide that explains it in full. The names of USD schema types, such as `Xformable` and
`Gprim`, are defined in [Population vocabulary](../population/glossary.md).

## Libraries

ovphysx works together with these libraries:

| Term | Meaning | Guide |
| --- | --- | --- |
| ovphysx | A library that simulates physics in USD scenes, used through its C API or its Python package. | [ovphysx](../index.md) |
| PhysX | NVIDIA's physics engine. ovphysx is built on it. | [PhysX SDK Documentation](https://nvidia-omniverse.github.io/PhysX/) |
| ovstage | The library that loads a USD scene once and keeps it in memory as a stage. ovphysx reads its scene from that stage. | [Stages and Ordinals](../getting_started/concepts.md#stages-and-ordinals) |
| ovrtx | NVIDIA's RTX rendering library. It can draw a stage that ovphysx writes its results into. | [Showing the Simulation](../common_tasks/showing_the_simulation.md) |
| NVIDIA Warp | The array library that holds the results of every Python read. | [Using the Results](../getting_started/concepts.md#using-the-results) |

## Scenes and Stages

These terms describe how a scene gets into ovphysx:

| Term | Meaning | Guide |
| --- | --- | --- |
| USD | Universal Scene Description, an open file format for 3D scenes. | [Loading Scenes](../building_scenes/loading_scenes.md) |
| Prim | One element of a USD scene, such as a box or a joint. | [Stages and Ordinals](../getting_started/concepts.md#stages-and-ordinals) |
| Path | The name of a prim, made of the names of its parents and its own name, such as `/World/Shelf/BoxA`. | [Path Patterns](../simulation_data/selecting_objects.md#path-patterns) |
| API schema | A label on a prim that adds behavior, such as `PhysicsRigidBodyAPI`, which makes the prim a moving body. | [Stages and Ordinals](../getting_started/concepts.md#stages-and-ordinals) |
| PhysX schemas | The definitions of the PhysX settings that a USD scene can hold, from the `physx-usd-schemas` package. Your program registers them before it loads a scene. | [Four Steps, in Order](../building_scenes/loading_scenes.md#four-steps-in-order) |
| Attribute | A named value. In a scene, an attribute such as `physics:mass` stores a setting on a prim. In a read or a write, an attribute such as `position` names the values for each object. | [Finding Attribute Names](../simulation_data/reading_values.md#finding-attribute-names) |
| Stage | A scene held in memory by ovstage, which your program loads and edits. | [Stages and Ordinals](../getting_started/concepts.md#stages-and-ordinals) |
| Ordinal | A number that your program gives each batch of edits to a stage, higher for each new batch. | [Stages and Ordinals](../getting_started/concepts.md#stages-and-ordinals) |
| Seal | To mark every edit up to an ordinal as complete, with `advance_write_floor`. ovphysx reads only sealed edits. | [Stages and Ordinals](../getting_started/concepts.md#stages-and-ordinals) |
| Attach | To connect ovphysx to a stage with `attach_ovstage`. ovphysx builds the simulation from the stage at a sealed ordinal. | [Four Steps, in Order](../building_scenes/loading_scenes.md#four-steps-in-order) |
| Scene update | A change to the scene itself while it runs: your program edits the stage, seals a new ordinal, and calls `update_from_ovstage`. | [Changing the Scene](../common_tasks/changing_the_scene.md) |
| Physics scene | The `PhysicsScene` prim, which holds the settings for the whole simulation, such as gravity. | [The Physics Scene](../building_scenes/physics_scene.md) |

## Bodies and Colliders

These terms describe the objects that ovphysx simulates:

| Term | Meaning | Guide |
| --- | --- | --- |
| Rigid body | An object that moves but never bends, such as a box or a wheel. | [Three Kinds of Objects](../building_scenes/rigid_bodies.md#three-kinds-of-objects) |
| Dynamic body | A rigid body that physics moves: gravity, contacts, joints, and forces. | [Three Kinds of Objects](../building_scenes/rigid_bodies.md#three-kinds-of-objects) |
| Kinematic body | A rigid body that your program moves. Gravity and contacts do not move it, but it moves the dynamic bodies it touches. | [Moving a Kinematic Body](../building_scenes/rigid_bodies.md#moving-a-kinematic-body) |
| Collider | A shape that other objects can hit. Applying `PhysicsCollisionAPI` to a shape prim makes one. | [Making an Object Solid](../building_scenes/colliders_and_materials.md#making-an-object-solid) |
| Static collider | A collider that is not part of a rigid body. It never moves, like the ground. | [Three Kinds of Objects](../building_scenes/rigid_bodies.md#three-kinds-of-objects) |
| Simple shape | A collider shaped as a cube, sphere, capsule, cylinder, or cone. These shapes cost the least and are the most stable. | [Simple Shapes](../building_scenes/colliders_and_materials.md#simple-shapes) |
| Approximation | The shape that ovphysx builds from the triangles of a mesh collider, such as one convex hull. | [Meshes](../building_scenes/colliders_and_materials.md#meshes) |
| Convex hull | The tightest shape without dents that wraps a mesh. | [Meshes](../building_scenes/colliders_and_materials.md#meshes) |
| Material | Settings for how a collider behaves when it touches something: how much it grips and how much it bounces. A material can also set the density that gives a body its mass. | [Materials](../building_scenes/colliders_and_materials.md#materials) |
| Soft body | An object that changes shape when it hits something and then returns toward its rest shape, such as a rubber toy. Also called a deformable body. It requires an NVIDIA GPU. | [Soft Bodies](../building_scenes/soft_bodies_and_particles.md#soft-bodies) |
| Rest shape | The shape that a soft body has when nothing pushes on it. | [Soft Bodies](../building_scenes/soft_bodies_and_particles.md#soft-bodies) |
| Particles | Small spheres that together simulate liquids and granular material, such as sand. They require an NVIDIA GPU. | [Particles](../building_scenes/soft_bodies_and_particles.md#particles) |

## Joints and Articulations

These terms describe how bodies are connected:

| Term | Meaning | Guide |
| --- | --- | --- |
| Joint | A connection between two bodies that limits how they move relative to each other, like the hinge of a door. | [Joints](../building_scenes/joints.md) |
| Limit | A stop at the minimum or maximum of a joint's motion, like a door stop. | [Limits](../building_scenes/joints.md#limits) |
| Drive | A motor on a joint. It pushes the joint toward a target position, a target velocity, or both. | [Drives](../building_scenes/joints.md#drives) |
| Articulation | A set of rigid parts, called links, connected by joints in a tree, such as a robot arm. ovphysx moves the links together, so the joints always hold exactly. | [Articulations](../building_scenes/articulations.md) |
| Link | One rigid part of an articulation. | [Links, Joints, and the Root](../building_scenes/articulations.md#links-joints-and-the-root) |
| Articulation root | `PhysicsArticulationRootAPI`, applied to the prim that holds an articulation. Without it, the links are separate rigid bodies. | [Links, Joints, and the Root](../building_scenes/articulations.md#links-joints-and-the-root) |
| Base | The link where the tree of an articulation starts. A fixed base is attached to the world, and a floating base can move and fall. | [Links, Joints, and the Root](../building_scenes/articulations.md#links-joints-and-the-root) |

## Running the Simulation

These terms describe how the simulation moves forward:

| Term | Meaning | Guide |
| --- | --- | --- |
| Step | One move of the simulation forward in time, made with `step` or `step_sync`. | [Steps and Tasks](../getting_started/concepts.md#steps-and-tasks) |
| Time step | The simulated time that one step covers, the `dt` that your program passes. These guides use 1/60 s. | [The Time Step](../building_scenes/physics_scene.md#the-time-step) |
| Task | A receipt for queued work, such as a step. Your program waits for each task once. | [Steps and Tasks](../getting_started/concepts.md#steps-and-tasks) |
| Solver iterations | The passes that PhysX makes in each step to correct positions. More iterations hold stacks and joints better but take more time. | [What to Change](../common_tasks/stable_simulations.md#what-to-change) |
| Environment | One self-contained setup, such as a robot and the objects it works with, under its own prim. | [Environments and Copies](../common_tasks/many_copies.md#environments-and-copies) |
| Clone | To copy an environment many times in the simulation only, for robot training. One read returns the state of every copy. | [Clone an Environment](../common_tasks/many_copies.md#clone-an-environment) |

## Reading and Writing

These terms describe how your program gets values out of the simulation and puts them in:

| Term | Meaning | Guide |
| --- | --- | --- |
| Object type | The kind of object that a query selects, a `SimObjectType` such as `RIGID_BODY`. | [Object Types](../simulation_data/selecting_objects.md#object-types) |
| Query | A selection of objects by type, and optionally by path pattern. Reads and writes act on a query, and your program can keep one to reuse it. | [Keeping a Query](../simulation_data/selecting_objects.md#keeping-a-query) |
| Path pattern | A path with wildcards, such as `/World/Box*`, that narrows a query to part of the scene. | [Path Patterns](../simulation_data/selecting_objects.md#path-patterns) |
| Read | A request for attributes of the objects that a query selected. It returns arrays with one row per object. | [Reading Values](../simulation_data/reading_values.md) |
| Group | One part of a read or write: the values of one attribute and the list of objects that they belong to. | [Reading and Writing State](../getting_started/concepts.md#reading-and-writing-state) |
| Write | A change that your program makes to the simulation, such as a velocity or a force. It has three steps: open, fill, and commit. | [Open, Fill, Commit](../simulation_data/writing_values.md#open-fill-commit) |
| Commit | The last step of a write. ovphysx applies the values of a group at once, before the next step. | [Open, Fill, Commit](../simulation_data/writing_values.md#open-fill-commit) |
| Mask | An array with one value for each row of a group. A commit with a mask skips the rows whose value is zero. | [Open, Fill, Commit](../simulation_data/writing_values.md#open-fill-commit) |
| Contact sensor | A body that reports the contact forces it receives, made with `PhysxContactReportAPI`. | [Contact Sensors](../common_tasks/contacts.md#contact-sensors) |

## CPU and GPU

These terms describe where the simulation runs and where its results are:

| Term | Meaning | Guide |
| --- | --- | --- |
| Default mode | With an NVIDIA GPU, physics runs on the GPU, and ovphysx copies most results to CPU memory after every step. | [Where Results Are](../simulation_data/gpu_data.md#where-results-are) |
| CPU mode | Physics and results both stay in CPU memory. On a computer with an NVIDIA GPU, set `OVPHYSX_DISABLE_GPU=1` to use it. | [CPU and GPU](../getting_started/concepts.md#cpu-and-gpu) |
| DirectGPU | A mode in which simulated values stay in GPU memory, ready for code that runs on the GPU, such as a PyTorch network. | [Turn On DirectGPU](../simulation_data/gpu_data.md#turn-on-directgpu) |
| Warp array | The `warp.array` that every Python read returns. For results in CPU memory, `numpy()` gives a NumPy array. | [Using the Results](../getting_started/concepts.md#using-the-results) |
| CUDA graph | A recording of GPU work, such as reads, a policy network, and writes, that replays with one launch. | [Replay a Loop With a CUDA Graph](../simulation_data/gpu_data.md#replay-a-loop-with-a-cuda-graph) |

## Seeing and Debugging

These terms describe the tools that show what the simulation does:

| Term | Meaning | Guide |
| --- | --- | --- |
| Debug view | Lines that show what PhysX works with, such as the collision shape of every object and every contact. Your program turns it on by writing to the `DEBUG_RENDER` target, and your renderer draws the lines. | [See What PhysX Sees](../common_tasks/debugging.md#see-what-physx-sees) |
| OmniPVD | The PhysX Omniverse Visual Debugger. It records what PhysX simulates at every step into a `.ovd` file, which a viewer replays. | [Record and Replay With OmniPVD](../common_tasks/debugging.md#record-and-replay-with-omnipvd) |
| Log level | How much ovphysx prints. By default, it prints messages of level `WARNING` and above. | [Turn Up the Log Level](../common_tasks/debugging.md#turn-up-the-log-level) |
