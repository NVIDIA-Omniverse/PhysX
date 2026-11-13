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
- These skills cover public ovphysx authoring and runtime workflows, including
  ovstage integration and the session read/write API.
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

## Populating physics

The USD population examples pass `descs=physics_population_descs(rendering=True)`:
ovphysx's physics population description, with RENDERING beside it for colliders
under native scene-graph instances. Sealing the population ordinal composes world
transforms with or without it; see
[Populating physics](docs/ovstage_integration.md#populating-physics-the-physics-population-description).

## Quick start

- Author a USD scene for ovstage to populate and ovphysx to simulate (rigid
  bodies, colliders): use `ovphysx-usd-authoring`.
- Smallest possible workflow: use `ovphysx-basic-workflow`.
- Read simulation output or write it back through ovstage: use `ovphysx-output-read`.
- Record selected objects to FILE or TCP, use receiver discovery/control, and investigate finalized captures: use `ovphysx-ovd-investigation` with a compatible receiver or separately installed SDK inspector.
- Replicated environments for RL: use `ovphysx-clone-environments`.
- Bulk simulation I/O: use the session read/write API — `ovphysx-output-read` to read output back
  (ovstage-native, identity-preserving), and `ovphysx-session-write` to push control inputs and state
  in (`ovphysx_write` / `PhysX.write`). The deprecated tensor-binding APIs remain
  available for compatibility, but their product skills have been removed; use
  the session skills for supported NumPy/CUDA read and write guidance.

## Skills

### `ovphysx-ovd-investigation`
- **Goal**: Bound an OvPhysX 0.8 capture by scene/object and primary/secondary data, discover objects before TCP sampling, and corroborate finalized recorded state with live CPU/GPU reads.
- **APIs**: `set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)` before the first `PhysX()`, collected attach/detach, `OmniPvdDestination.file` / `.tcp`, `OmniPvdFilter`, `PhysX.start_recording(..., filter=..., wait_for_selection=...)`, `PhysX.stop_recording`, `PhysX.read_sync`; native receiver `requestCapture` / `requestCaptureCatalog` and separately installed SDK `ovd-inspect`.
- **Doc**: [skills/ovphysx-ovd-investigation/SKILL.md](skills/ovphysx-ovd-investigation/SKILL.md)
- **Evaluation**: `skills/ovphysx-ovd-investigation/evals/evals.json` covers installed sample discovery and required `physx-usd-schemas` installation, global recording capability, schema-support preservation, external schema registration and collected attach/detach, physics-description population with RENDERING and synchronous live snapshots, CPU/GPU runtime evidence, carried-snapshot provenance, structured load/output errors, machine-readable payload budgets, opt-in stopped-tail recovery, typed orientation and independent frame completion, optional annotations, scene-level contact impulse limitations, and routing live-only reads to `ovphysx-output-read`.
  Selection cases cover timestamped directory paths, exact object/scene unions, primary/secondary payloads, particle/deformable/DirectGPU scope, contact counterparts, discovery bounds, owner-step control pumping, applied acknowledgments, legacy TCP peers and missing-data interpretation.
- **References**:
  - [Recording tutorial](docs/tutorials/omnipvd_recording.md)
  - Wheel sample: `samples/python_samples/omnipvd_inspection.py` (source: `tests/python_samples/omnipvd_inspection.py`)
  - Inspector build/install guide: `physx/pvddom/README.md` in a PhysX SDK source distribution (`PVDDOM_BUILD_INSPECTION=ON` and `PVDDOM_BUILD_TOOLS=ON`).
  - SDK inspection/comparison playbooks: `share/physx/SKILLS.md` after installing that inspector.

### `ovphysx-usd-authoring`
- **Goal**: Author USD physics content (rigid bodies, colliders, mass, physics
  scene) for ovstage to populate and ovphysx to simulate.
- **Skill version**: 0.1.2
- **APIs**: USD authoring via `.usda` text or Python
  (`UsdPhysics.RigidBodyAPI`); PhysX-specific schemas via
  `physx_schema_paths()` + generic
  `prim.ApplyAPI("PhysxRigidBodyAPI")`.
- **Doc**: [skills/ovphysx-usd-authoring/SKILL.md](skills/ovphysx-usd-authoring/SKILL.md)
- **References**:
  - Physics scene and ground: `skills/ovphysx-usd-authoring/references/scene_setup.md`
  - Rigid body: `skills/ovphysx-usd-authoring/references/rigid_body.md`
- **Evaluation**: `skills/ovphysx-usd-authoring/evals/evals.json` covers `.usda` and Python authoring of scene/collider/rigid-body/mass, the codeless PhysX schema path for PhysX-only attributes, and routing runtime stepping/reads to `ovphysx-basic-workflow` / `ovphysx-output-read` while routing cloning and rendering away.

### `ovphysx-basic-workflow`
- **Goal**: Create an instance, attach an ovstage scene, step simulation, clean up.
- **Skill version**: 0.1.11
- **APIs**: `PhysX()`, `set_global_config` / `PhysX.set_config` (GPU ordinal after create), `attach_ovstage()`, `step_sync()` (default) / `step()` + `wait_task()`, `destroy()` / C: `ovphysx_create_instance()`, `ovphysx_set_global_config` / `ovphysx_set_config`, `ovphysx_attach_ovstage()`, `ovphysx_step_sync()` / `ovphysx_step()` + `ovphysx_wait_task()`, `ovphysx_destroy_instance()`
- **Doc**: [skills/ovphysx-basic-workflow/SKILL.md](skills/ovphysx-basic-workflow/SKILL.md)
- **Evaluation**: `skills/ovphysx-basic-workflow/evals/evals.json` covers the Python 3.10-3.13 prerequisite, installation and registration of both PhysX and Newton schemas before population, populating through `open_usd_with_desc` and `physics_population_descs(rendering=True)` for native-instance compatibility, with no RENDERING needed for a translated parent because sealing composes world transforms, the minimal Python/C lifecycle including config-before-create and config-after-create ordering, async Task/wait_task/TaskError handling, the packaged population-update helper, following the SDK C sample's source and build/launch recipe, Windows runtime helpers and schema setup, and routing clone/control work to `ovphysx-clone-environments` / `ovphysx-session-write`.
- **References**:
  - Docs: `docs/tutorials/hello_world.md`
  - Python sample: `samples/python_samples/hello_world.py` (wheel; source: `tests/python_samples/hello_world.py`)
  - C sample and build/launch recipe: `samples/c_samples/basic_workflow_c/main.c`, `samples/c_samples/basic_workflow_c/CMakeLists.txt`, and `samples/c_samples/basic_workflow_c/README.md` (SDK; source: `tests/c_samples/basic_workflow_c/`)
  - Later population edits: `samples/python_samples/population_update.py` (wheel; source: `tests/python_samples/population_update.py`)

### `ovphysx-output-read`
- **Goal**: Read simulation output by type and attribute, or write ovstage-native output groups back into the attached Stage with separate control/output ordinals.
- **Metadata**: Canonical skill version, author, tags, and ovphysx compatibility are declared in the skill frontmatter.
- **APIs**: Python: `PhysX.query()` + `PhysX.read()` operations, `PhysX.read_sync()` / `PhysX.read_tokens_sync()`; C: `ovphysx_query()`, `ovphysx_read()` (with optional query/read results), `ovphysx_fetch_read_next()`, and matching release calls
- **Doc**: [skills/ovphysx-output-read/SKILL.md](skills/ovphysx-output-read/SKILL.md)
- **Evaluation**: `skills/ovphysx-output-read/evals/evals.json` covers Python reads and framework interoperability, C task completion/null exhaustion and no-repack write-back through the attached Stage's dictionary, read-result omission flags that separate empty, unsupported, and not-ready answers, deprecation-aware new-code routing, `ACTIVE`/layout guardrails, host sleep-state snapshots with body/device semantics and wake-command handoff, articulation link/DOF/tendon identity joins with count-based padding, and the read-only `DEFORMABLE_SKIN_MESH` skinned-mesh read (skin-local device `points`, one tensor per skin prim).
- **References**:
  - Bundled: `skills/ovphysx-output-read/references/{python,c_api,scope_and_layout,closed_loop}.md`
  - Public docs: <https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/index.html>
  - Python sample: `samples/python_samples/output_read.py` (wheel; source: `tests/python_samples/output_read.py`)
  - C sample: `samples/c_samples/output_read_c/main.c` (SDK; source: `tests/c_samples/output_read_c/main.c`)

### `ovphysx-session-write`
- **Goal**: Push caller-owned control inputs and state into the simulation by type and attribute -- fill borrowed write groups and commit them, then step. The successor to the deprecated tensor-binding write; read output back with `ovphysx-output-read`.
- **Metadata**: Canonical skill version, author, tags, and ovphysx compatibility are declared in the skill frontmatter.
- **APIs**: Python: `PhysX.write()` operations and the waiting `PhysX.write_sync()` convenience, `WriteSession`, `WriteGroup`; C: `ovphysx_query()`, `ovphysx_write()` (attribute array), `ovphysx_fetch_write_next()` (`ovphysx_fetched_write_group_t`), `ovphysx_commit_group()`, and matching release calls
- **Doc**: [skills/ovphysx-session-write/SKILL.md](skills/ovphysx-session-write/SKILL.md)
- **Evaluation**: `skills/ovphysx-session-write/evals/evals.json` covers writing control targets then reading state, CPU/GPU pre-step write applying (DirectGPU still refuses), force-vs-wrench placement, native task/error/cleanup handling with the fetched-group pair, multi-attribute sessions identified by `attribute_index` with non-transactional commits, wake/sleep command restrictions and state-read handoff, and read/deprecation routing.
- **References**:
  - Bundled: `skills/ovphysx-session-write/references/{python,c_api}.md`
  - Public docs: <https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/index.html>
  - Worked example: `samples/python_samples/session_write.py` (wheel; source: `tests/python_samples/session_write.py`); C write loop in `tests/c_unittests/test_joint_datamovement.cpp`

### `ovphysx-clone-environments`
- **Goal**: Clone one USD environment subtree into many runtime-only PhysX environments before `warmup()` or the first simulation step.
- **Skill version**: 0.1.9
- **APIs**: Python: `PhysX.clone()`, `PhysX.wait_task()` / `PhysX.wait_all()` / C: `ovphysx_clone()`, `ovphysx_wait_task()` / `ovphysx_wait_all()`
- **Doc**: [skills/ovphysx-clone-environments/SKILL.md](skills/ovphysx-clone-environments/SKILL.md)
- **Evaluation**: `skills/ovphysx-clone-environments/evals/evals.json` covers the Python 3.10-3.13 prerequisite, installation and registration of both PhysX and Newton schemas before population, populating through `open_usd_with_desc` and `physics_population_descs(rendering=True)` for native-instance compatibility, with no RENDERING needed for a translated parent because sealing composes world transforms, clone-before-step ordering, `anchor_transforms`, CPU collision-isolation guidance, C++ wait-result cleanup, and routing away from USD authoring.
- **References**:
  - Docs: `docs/tutorials/cloning.md`
  - Python sample: `samples/python_samples/clone.py` (wheel; source: `tests/python_samples/clone.py`)
  - C sample: `samples/c_samples/clone_c/main.c` (SDK; source: `tests/c_samples/clone_c/main.c`)
