# AGENTS.md — NVIDIA PhysX Repository

## ovphysx (primary AI-agent use case)

Self-contained Python/C library for USD-based physics simulation — the fastest
path to running PhysX from Python for reinforcement learning and robotics.

```bash
pip install ovphysx
```

- **Documentation:** https://nvidia-omniverse.github.io/PhysX/ovphysx/index.html
- **Source subfolder:** [`ovphysx/`](ovphysx/)
- **Samples:** `ovphysx/tests/python_samples/` (hello_world.py, clone.py, etc.)

The installed wheel ships `SKILLS.md` with step-by-step playbooks for common
tasks (scene loading, environment cloning, tensor bindings, resetting).

## Other projects in this repo

| Directory | What it is |
|---|---|
| [`physx/`](physx/) | PhysX SDK — C++ real-time physics engine |
| [`blast/`](blast/) | Blast SDK — destruction and fracture simulation |
| [`flow/`](flow/) | Flow SDK — fluid and fire simulation |
| [`omni/`](omni/) | Omniverse PhysX extensions for Kit-based apps |

Each subfolder has its own README with build and usage instructions.
