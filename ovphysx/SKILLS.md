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

- ovphysx bundles its own OpenUSD libraries. It is not needed to install
  `usd-core` as a separate dependency -- doing so may cause version conflicts.
- ovphysx exchanges tensor data via DLPack, so any framework that understands
  DLPack can consume simulation outputs. Common optional companions are
  `numpy` (CPU tensors) and `torch` (GPU tensors), but they are not
  dependencies of this package. Add whichever you need to your own project
  requirements, for example in your `pyproject.toml`:
  ```toml
  dependencies = ["ovphysx", "numpy"]
  ```

## Population domains

Every skill below populates with `domains=ovstage.PopulationDomain.PHYSICS`
because their sample USD is known not to put physics under native scene-graph
instances. For arbitrary content -- including headless apps -- prefer `ALL`
(equivalently `PHYSICS | RENDERING`); `PHYSICS` alone can silently omit
instanced colliders on the currently pinned ovstage. Refer to
`docs/ovstage_integration.md` ("Population domains") before copying a snippet
into production code.

## Quick start

- Author a USD scene to simulate (rigid bodies, colliders): use `ovphysx-usd-authoring`.
- Smallest possible workflow: use `basic-workflow`.
- Read simulation output or write it back through ovstage: use `ovphysx-output-read`.
- Replicated environments for RL: use `clone-environments`.
- Bulk simulation I/O on CPU: use `tensor-bindings-cpu`.
- Bulk simulation I/O on GPU (CUDA): use `tensor-bindings-gpu`.

## Skills

### `ovphysx-usd-authoring`
- **Goal**: Author USD physics content (rigid bodies, colliders, mass, physics scene) that ovphysx can load and simulate.
- **Skill version**: 0.1.0
- **APIs**: USD authoring via `.usda` text or Python (`UsdPhysics.*`); PhysX-specific schemas via `ovphysx.codeless_schema_paths()` + generic `prim.ApplyAPI(...)`.
- **Doc**: [skills/ovphysx-usd-authoring/SKILL.md](skills/ovphysx-usd-authoring/SKILL.md)
- **References**:
  - Physics scene and ground: `skills/ovphysx-usd-authoring/references/scene_setup.md`
  - Rigid body: `skills/ovphysx-usd-authoring/references/rigid_body.md`

### `basic-workflow`
- **Goal**: Create an instance, attach an ovstage scene, step simulation, clean up.
- **Skill version**: 0.1.1
- **APIs**: `PhysX()`, `attach_ovstage()`, `step_sync()` (default) / `step()`, `release()` / C: `ovphysx_create_instance()`, `ovphysx_attach_ovstage()`, `ovphysx_step_sync()` / `ovphysx_step()`, `ovphysx_destroy_instance()`
- **Doc**: [skills/basic-workflow/SKILL.md](skills/basic-workflow/SKILL.md)
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

### `tensor-bindings-cpu`
- **Goal**: Create tensor bindings, write control inputs, step, read back state on CPU.
- **Skill version**: 0.1.0
- **APIs**: `create_tensor_binding()`, `.read()`, `.write()` / C: `ovphysx_create_tensor_binding()`, `ovphysx_read_tensor_binding()`, `ovphysx_write_tensor_binding()`
- **Doc**: [skills/tensor-bindings-cpu/SKILL.md](skills/tensor-bindings-cpu/SKILL.md)
- **References**:
  - Docs: `docs/tutorials/tensor_bindings.md`
  - Python sample: `samples/python_samples/tensor_bindings.py` (wheel; source: `tests/python_samples/tensor_bindings.py`)
  - C sample: `samples/c_samples/tensor_bindings_c/main.c` (SDK; source: `tests/c_samples/tensor_bindings_c/main.c`)

### `clone-environments`
- **Goal**: Clone one USD environment subtree into many runtime-only PhysX environments before the first simulation step (or GPU warmup, if used).
- **Skill version**: 0.1.0
- **APIs**: Python: `PhysX.clone()` / C: `ovphysx_clone()`
- **Doc**: [skills/clone-environments/SKILL.md](skills/clone-environments/SKILL.md)
- **References**:
  - Docs: `docs/tutorials/cloning.md`
  - Python sample: `samples/python_samples/clone.py` (wheel; source: `tests/python_samples/clone.py`)
  - C sample: `samples/c_samples/clone_c/main.c` (SDK; source: `tests/c_samples/clone_c/main.c`)

### `tensor-bindings-gpu`
- **Goal**: Read and write simulation data on GPU using CUDA device pointers and DLPack.
- **Skill version**: 0.1.0
- **APIs**: Same as CPU bindings; set `physxScene:enableGPUDynamics=true` in the USD stage to use GPU.
- **Doc**: [skills/tensor-bindings-gpu/SKILL.md](skills/tensor-bindings-gpu/SKILL.md)
- **References**:
  - Docs: `docs/tutorials/tensor_bindings.md`; `docs/developer_guide.md` for GPU/DirectGPU specifics
  - C sample: `samples/c_samples/tensor_bindings_gpu_c/main.c` (SDK; source: `tests/c_samples/tensor_bindings_gpu_c/main.c`)
