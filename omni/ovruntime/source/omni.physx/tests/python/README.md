# ovruntime Python Tests

Standalone (non-Kit) physics tests using the ovruntime build.

## Prerequisites

Build ovruntime from `omni/ovruntime/`:

```bash
./build.sh        # builds Debug + Release, creates venv
./build.sh -r     # Release only
./build.sh -d     # Debug only
```

This creates `_build/.venv/` with Python, pytest, numpy, and the required `.pth` files.

## Running tests

From `omni/ovruntime/source/omni.physx/tests/python/`:

```bash
# Activate the venv
source ../../../../_build/.venv/bin/activate

# Run all tests
pytest -v

# Run a specific test file
pytest testPhysxContactReportAPI.py -v

# Run a specific test method
pytest testPhysicsDeformableBodyAPI.py::PhysicsDeformableBodyAPITestMemoryStage::test_volume_deformable_setup_simmesh -v

# Or without activating the venv:
../../../../_build/.venv/bin/python -m pytest -v
```

## GPU tests

Some tests (deformable bodies, SDF collision) require a CUDA-capable GPU.
The test infrastructure preloads `libcuda.so.1` at import time so that CUDA
driver symbols are available to the physics plugins. `libPhysXGpu_64.so` is
loaded automatically by the foundation plugin at startup.

If no GPU is available, GPU-dependent tests will fail with:
`CUDA libs are present, but no suitable CUDA GPU was found!`

## Architecture

- `_carb_setup.py` — Carbonite framework init, path setup, core plugin loading
- `_physics_setup.py` — Physics plugin loading, CUDA preload, PhysX schema registration, module bridges
- `conftest.py` — Pytest conftest (imports `_carb_setup`)
- `physicsBase.py` — Base test classes (`PhysicsBaseTestCase`, `PhysicsMemoryStageBaseTestCase`)
- `pyproject.toml` — Pytest configuration
