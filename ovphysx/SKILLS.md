<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# ovphysx Skills

This file indexes the available skills for ovphysx.
A skill is a focused playbook (Markdown + code snippets) that teaches an agent or human how to accomplish one task with ovphysx.

Installing a skill does NOT build ovphysx. It only provides docs and snippets.
Installing ovphysx itself is separate:
- Python: `pip install ovphysx`
- C/C++: download the OVPhysX SDK and matching native OVStage archive as
  described in the [SDK Quickstart](docs/tutorials/quickstart.md), or build from
  source (the build fetches OVStage automatically)

Notes:
- These skills cover public ovphysx authoring and runtime workflows, including ovstage integration and TensorBindingsAPI.
- The bundled samples are CI-tested and are the authoritative full examples;
  skills keep only the shortest task-oriented pattern.
- Each `SKILL.md` declares its required name and invocation description.
  Optional author, version, and tags belong under the standard `metadata` map;
  environment requirements belong in `compatibility`. Product-specific UI
  metadata may also live under `agents/openai.yaml`.

## Dependencies

- ovphysx does not need a separately installed classic USD runtime to simulate
  an ovstage scene. The optional Python route in `ovphysx-usd-authoring` does
  need caller-owned `pxr` modules: on supported PyPI platforms, install
  `usd-core` in the authoring or validation environment. It remains a tool
  dependency, never an ovphysx runtime dependency. Applications do not pass
  that USD runtime to ovphysx: ovstage owns population, and ovphysx attaches the
  resulting `ovstage.Stage`. PyPI currently provides no Linux aarch64
  `usd-core` wheel or source distribution; use the `.usda` text route, author
  on a supported host, or supply a compatible OpenUSD Python build.
- The Python output-read API returns `warp.array` on both CPU and CUDA.
  `warp-lang` is a required dependency of ovphysx and currently brings NumPy
  transitively. Warp owns downstream DLPack interoperability; add only the
  additional consumer framework your application needs, for example:
  ```toml
  dependencies = ["ovphysx", "torch"]
  ```

## Population domains

Every skill below populates with `domains=ovstage.PopulationDomain.PHYSICS`
because their sample USD is known not to put physics under native scene-graph
instances. For arbitrary content — including headless apps — prefer `ALL`
(equivalently `PHYSICS | RENDERING`); `PHYSICS` alone can silently omit
instanced colliders on the currently pinned ovstage. Refer to
[Population domains](docs/ovstage_integration.md#population-domains) before
copying a snippet into production code.

## Quick start

- Author a USD scene for ovstage to populate and ovphysx to simulate (rigid
  bodies, colliders): use `ovphysx-usd-authoring`.
- Smallest possible workflow: use `basic-workflow`.
- Read simulation output or write it back through ovstage: use `ovphysx-output-read`.
- Replicated environments for RL: use `clone-environments`.
- Bulk simulation I/O: use the session read/write API — `ovphysx-output-read` to read output back
  (ovstage-native, identity-preserving), and `ovphysx-session-write` to push control inputs and state
  in (`ovphysx_write` / `PhysX.write`). The tensor-binding skills `tensor-bindings-cpu` /
  `tensor-bindings-gpu` are deprecated (ovphysx 0.6) and kept only for maintaining existing
  caller-owned bulk NumPy/CUDA binding code.

## Skills

### `ovphysx-usd-authoring`
- **Goal**: Author USD physics content (rigid bodies, colliders, mass, physics
  scene) for ovstage to populate and ovphysx to simulate.
- **Skill version**: 0.1.1
- **APIs**: USD authoring via `.usda` text or Python
  (`UsdPhysics.RigidBodyAPI`); PhysX-specific schemas via
  `ovphysx.codeless_schema_paths()` + generic
  `prim.ApplyAPI("PhysxRigidBodyAPI")`.
- **Doc**: [skills/ovphysx-usd-authoring/SKILL.md](skills/ovphysx-usd-authoring/SKILL.md)
- **References**:
  - Physics scene and ground: `skills/ovphysx-usd-authoring/references/scene_setup.md`
  - Rigid body: `skills/ovphysx-usd-authoring/references/rigid_body.md`
- **Evaluation**: `skills/ovphysx-usd-authoring/evals/evals.json` covers `.usda` and Python authoring of scene/collider/rigid-body/mass, the codeless PhysX schema path for PhysX-only attributes, and routing away from runtime stepping, tensor I/O, cloning, and rendering.

### `basic-workflow`
- **Goal**: Create an instance, attach an ovstage scene, step simulation, clean up.
- **Skill version**: 0.1.3
- **APIs**: `PhysX()`, `attach_ovstage()`, `step_sync()` (default) / `step()`, `destroy()` / C: `ovphysx_create_instance()`, `ovphysx_attach_ovstage()`, `ovphysx_step_sync()` / `ovphysx_step()`, `ovphysx_destroy_instance()`
- **Doc**: [skills/basic-workflow/SKILL.md](skills/basic-workflow/SKILL.md)
- **Evaluation**: `skills/basic-workflow/evals/evals.json` covers the minimal Python/C lifecycle, post-attach population drains, and routing away from clone/tensor work.
- **References**:
  - Docs: `docs/tutorials/hello_world.md`
  - Python sample: `samples/python_samples/hello_world.py` (wheel; source: `tests/python_samples/hello_world.py`)
  - C sample: `samples/c_samples/hello_world_c/main.c` (SDK; source: `tests/c_samples/hello_world_c/main.c`)

### `ovphysx-output-read`
- **Goal**: Read simulation output by type and attribute, or write ovstage-native output groups back into the attached Stage with separate control/output ordinals.
- **Metadata**: Canonical skill version, author, tags, and ovphysx compatibility are declared in the skill frontmatter.
- **APIs**: Python: `PhysX.read()` / `PhysX.read_tokens()`; C: `ovphysx_query()`, `ovphysx_read()`, `ovphysx_fetch_read_next()`, and matching release calls
- **Doc**: [skills/ovphysx-output-read/SKILL.md](skills/ovphysx-output-read/SKILL.md)
- **Evaluation**: `skills/ovphysx-output-read/evals/evals.json` covers Python reads, C no-repack write-back, caller-owned CUDA-buffer routing, and `ACTIVE`/layout guardrails.
- **References**:
  - Bundled: `skills/ovphysx-output-read/references/{python,c,scope_and_layout,closed_loop}.md`
  - Public docs: <https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/index.html>
  - Python sample: `samples/python_samples/output_read.py` (wheel; source: `tests/python_samples/output_read.py`)
  - C sample: `samples/c_samples/output_read_c/main.c` (SDK; source: `tests/c_samples/output_read_c/main.c`)

### `ovphysx-session-write`
- **Goal**: Push caller-owned control inputs and state into the simulation by type and attribute -- fill borrowed write groups and commit them, then step. The successor to the deprecated tensor-binding write; read output back with `ovphysx-output-read`.
- **Metadata**: Canonical skill version, author, tags, and ovphysx compatibility are declared in the skill frontmatter.
- **APIs**: Python: `PhysX.write()`, `WriteSession`, `WriteGroup`; C: `ovphysx_query()`, `ovphysx_write()`, `ovphysx_fetch_write_next()`, `ovphysx_commit_group()`, and matching release calls
- **Doc**: [skills/ovphysx-session-write/SKILL.md](skills/ovphysx-session-write/SKILL.md)
- **Evaluation**: `skills/ovphysx-session-write/evals/evals.json` covers writing control targets then reading state, the refuse-before-first-step contract, force-vs-wrench placement, and read/deprecation routing.
- **References**:
  - Bundled: `skills/ovphysx-session-write/references/{python,c_api}.md`
  - Public docs: <https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/index.html>
  - Worked example (source checkout, not shipped): `tests/python_samples_internal/rigid_body_falling_tensors.py`; C write loop in `tests/c_unittests/test_joint_datamovement.cpp`

### `tensor-bindings-cpu` (deprecated)
- **Goal**: Exchange CPU simulation state (poses, velocities, joint targets) as caller-owned NumPy arrays through tensor bindings: create tensor bindings, write control inputs, step, read back state on CPU. The tensor-binding CODE API is deprecated (ovphysx 0.6) in favor of the session read/write API; use the ovphysx-session-write skill for the write path and ovphysx-output-read for reads. Kept only for maintaining existing caller-owned bulk-NumPy binding code.
- **Skill version**: 0.1.4
- **APIs**: `create_tensor_binding()`, `.native_device`, `.read()`, `.write()` / C: `ovphysx_create_tensor_binding()`, `ovphysx_get_tensor_binding_native_device()`, `ovphysx_read_tensor_binding()`, `ovphysx_write_tensor_binding()`
- **Doc**: [skills/tensor-bindings-cpu/SKILL.md](skills/tensor-bindings-cpu/SKILL.md)
- **Evaluation**: `skills/tensor-bindings-cpu/evals/evals.json` covers maintaining/extending existing binding code, the target-vs-state read pitfall, and routing new code (and GPU / ovstage-identity needs) to the session skills -- `ovphysx-session-write` / `ovphysx-output-read`.
- **References**:
  - Docs: `docs/tutorials/tensor_bindings.md`
  - Python sample: `samples/python_samples/tensor_bindings.py` (wheel; source: `tests/python_samples/tensor_bindings.py`)
  - C sample: `samples/c_samples/tensor_bindings_c/main.c` (SDK; source: `tests/c_samples/tensor_bindings_c/main.c`)

### `clone-environments`
- **Goal**: Clone one USD environment subtree into many runtime-only PhysX environments before `warmup()` or the first simulation step.
- **Skill version**: 0.1.4
- **APIs**: Python: `PhysX.clone()` / C: `ovphysx_clone()`
- **Doc**: [skills/clone-environments/SKILL.md](skills/clone-environments/SKILL.md)
- **Evaluation**: `skills/clone-environments/evals/evals.json` covers clone-before-step ordering, `anchor_transforms`, CPU collision-isolation guidance, and routing away from USD authoring.
- **References**:
  - Docs: `docs/tutorials/cloning.md`
  - Python sample: `samples/python_samples/clone.py` (wheel; source: `tests/python_samples/clone.py`)
  - C sample: `samples/c_samples/clone_c/main.c` (SDK; source: `tests/c_samples/clone_c/main.c`)

### `tensor-bindings-gpu` (deprecated)
- **Goal**: Exchange GPU simulation state as caller-owned CUDA tensors (DLPack, GPU-to-GPU with no CPU staging) through tensor bindings: read and write simulation data on GPU using CUDA device pointers and DLPack. The tensor-binding CODE API is deprecated (ovphysx 0.6) in favor of the session read/write API; use the ovphysx-session-write skill for the write path and ovphysx-output-read for reads. Kept only for maintaining existing caller-owned bulk-CUDA binding code.
- **Skill version**: 0.1.4
- **APIs**: Same as CPU bindings; query `.native_device` (or the C getter)
  before allocating because native CUDA state bindings require DirectGPU and
  CPU-only binding types remain on CPU.
- **Doc**: [skills/tensor-bindings-gpu/SKILL.md](skills/tensor-bindings-gpu/SKILL.md)
- **Evaluation**: `skills/tensor-bindings-gpu/evals/evals.json` covers maintaining existing DLPack CUDA binding code, the DirectGPU vs contact-modification tradeoff, and routing new code (and CPU-only needs) to the session skills or clone.
- **References**:
  - Docs: `docs/tutorials/tensor_bindings.md`; `docs/developer_guide.md` for GPU/DirectGPU specifics
  - C sample: `samples/c_samples/tensor_bindings_gpu_c/main.c` (SDK; source: `tests/c_samples/tensor_bindings_gpu_c/main.c`)
