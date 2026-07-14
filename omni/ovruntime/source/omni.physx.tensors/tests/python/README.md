# omni.physx.tensors Standalone Tests

Standalone pytest suite for the `omni.physx.tensors` tensor API. These tests
run outside Kit using the ovruntime build and exercise rigid bodies,
articulations, contacts, force sensors, deformable/soft bodies, particles,
SDF shapes, and more across CPU and GPU pipelines.

## Prerequisites

Build ovruntime from `omni/ovruntime/`:

```bash
cd omni/ovruntime
./build.sh        # Linux
build.bat         # Windows
```

This creates the Python virtual environment at `omni/ovruntime/_build/.venv/`
with all required dependencies (pytest, numpy, warp, USD, PhysX plugins).

## Running Tests

From the test directory:

```bash
cd omni/ovruntime/source/omni.physx.tensors/tests/python
../../../../_build/.venv/bin/python -m pytest -v
```

### Useful pytest options

```bash
# Run a single test file
../../../../_build/.venv/bin/python -m pytest -v testRigidBody.py

# Run a single test method
../../../../_build/.venv/bin/python -m pytest -v testRigidBody.py::PhysxTensorsRigidBodyTests::test_rigid_body_transforms_cc

# Run only GPU tests (match _gg suffix)
../../../../_build/.venv/bin/python -m pytest -v -k "_gg"

# Run only CPU tests (match _cc suffix)
../../../../_build/.venv/bin/python -m pytest -v -k "_cc"

# Stop on first failure
../../../../_build/.venv/bin/python -m pytest -v -x
```

## Running Benchmarks

The `benchmarks.py` module provides benchmark scenarios that use the same
infrastructure as the tests. To run the cartpole benchmark:

```bash
cd omni/ovruntime/source/omni.physx.tensors/tests/python

../../../../_build/.venv/bin/python -c "
from benchmarks import BenchCartpole
from scenario import DeviceParams, SyncParams, RunnerInMemory

bench = BenchCartpole(DeviceParams(True, True), num_envs=5000)
runner = RunnerInMemory(bench, 'warp', SyncParams(sync_usd=True, sync_fabric=False, transforms_only=False))
runner.start(warm_start=True)
runner.simulate()  # runs 600 steps
runner.stop()
print('Done')
"
```

Adjust `num_envs` to control the workload. Use `DeviceParams(False, False)`
for CPU-only execution.

## Test naming conventions

| Suffix | DeviceParams | Description |
|--------|-------------|-------------|
| `_cc`  | `(False, False)` | CPU broadphase, CPU pipeline |
| `_gc`  | `(True, False)`  | GPU broadphase, CPU pipeline |
| `_gg`  | `(True, True)`   | GPU broadphase, GPU pipeline |
| `_cpu` | `(False, False)` | CPU (alternate naming) |
| `_gpu` | `(True, True)`   | GPU (alternate naming) |

## File overview

| File | Tests | Description |
|------|-------|-------------|
| `testArticulationGetSet.py` | 36 | Get/set root transforms, velocities, DOF positions/velocities/forces |
| `testArticulationDynamics.py` | 24 | Jacobians, mass matrices, Coriolis, gravity, friction, joint forces |
| `testArticulationJointBodyOrder.py` | 21 | DOF limits, positions, velocities, targets, forces with joint ordering |
| `testArticulationView.py` | 9 | Articulation/humanoid view creation, DOF properties, special cases |
| `testArticulationProperties.py` | 16 | Body/shape properties, fixed/spatial tendon properties |
| `testRigidBody.py` | 45 | Rigid body transforms, velocities, forces, properties, materials |
| `testLinearAngularDof.py` | 30 | Linear and angular DOF positions, velocities, forces, targets |
| `testHeterogeneous.py` | 9 | Heterogeneous articulation scenes, centroidal momentum |
| `testContacts.py` | 14 | Rigid/articulation contacts, contact matrix, raw contact data |
| `testForceSensors.py` | 33 | Force/torque sensors, joint actuation forces |
| `testForceProjection.py` | 90 | Single/double link force projection across axes and DOF configs |
| `testMisc.py` | 5 | Prim deletion, sim view invalidation, joint limit changes |
| `testSimulationView.py` | 3 | Simulation view gravity, asset path validation |
| `testSoftBody.py` | 6 | Soft body views, element indices, positions, velocities, kinematic targets |
| `testSoftBodyMaterial.py` | 3 | Soft body material elasticity, damping, friction |
| `testDeformableBody.py` | 12 | Volume/surface deformable body views, positions, velocities, rest data |
| `testDeformableMaterial.py` | 2 | Deformable material Young's modulus, dynamic friction |
| `testParticleCloth.py` | 5 | Particle cloth positions, velocities, masses, springs |
| `testParticleSystem.py` | 4 | Particle system offsets, wind |
| `testParticleMaterial.py` | 5 | Particle material friction, damping, gravity scale, lift, drag |
| `testSdfShape.py` | 1 | SDF shape values and gradients |
| `testWarp.py` | 2 | Warp utility tests (CPU and GPU) |
| **Total** | **375** | |
