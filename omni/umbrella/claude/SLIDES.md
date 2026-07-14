# Physics Umbrella Refactor
### `feature/umbrella_refactor` Branch Summary

---

## Slide 1: What & Why

**Problem:** The Omni Physics runtime code was tightly coupled to the Kit/ovruntime build system, making it difficult to develop, test, and integrate physics engines independently.

**Solution:** A new **`umbrella/`** project that extracts the physics simulation layer into a standalone, self-contained module with:

- Clean C++ interfaces (no Kit dependency in the core)
- Optional Carbonite plugin integration
- Python bindings via pybind11
- Independent build system (CMake + Packman)

---

## Slide 2: Scope of Work

| Metric | Value |
|---|---|
| Commits | 14 (including 3 trunk merges) |
| Files changed | 119 |
| Lines added | ~12,500 |
| Lines removed | ~37 |
| New C++ headers | 12 |
| Source files moved | 10 (from ovruntime/plugins to umbrella/source) |
| New test files | 17 (11 C++ + 6 Python) |

---

## Slide 3: Architecture Overview

```
umbrella/
  include/omni/physics/simulation/   <-- Public C++ interfaces
      IPhysics.h                          (simulation registry)
      IPhysicsSimulation.h                (lifecycle & stepping)
      IPhysicsSceneQuery.h                (raycasts, sweeps, overlaps)
      IPhysicsBenchmark.h                 (profiling)
      IPhysicsInteraction.h               (user interaction)
      IPhysicsStageUpdate.h               (stage management)
      simulator/                          (data types & implementations)

  source/                             <-- Core C++ implementation
      (moved from ovruntime/plugins/)

  bindings/                           <-- Python bindings (pybind11)
      _physics module + __init__.py

  tests/
      cpp/                            <-- C++ tests (doctest)
      python/                         <-- Python tests (unittest/pytest)
```

---

## Slide 4: Six Core Interfaces

| Interface | Responsibility |
|---|---|
| **IPhysics** | Simulation registry - register, activate, deactivate engines |
| **IPhysicsSimulation** | Simulation lifecycle - attach stage, step, fetch results |
| **IPhysicsSceneQuery** | Raycasts, sweeps, overlap queries |
| **IPhysicsInteraction** | User interaction events |
| **IPhysicsBenchmark** | Performance profiling & statistics |
| **IPhysicsStageUpdate** | Stage attachment & timeline events (legacy) |

All interfaces follow the Carbonite `acquire/release` pattern and are exposed to both C++ and Python.

---

## Slide 5: Plugin Architecture

**Function-pointer based extensibility** - physics engines register via callback structs:

- `SimulationFns` - core simulation callbacks
- `SceneQueryFns` - query callbacks
- `InteractionFns` - interaction callbacks
- `BenchmarkFns` - profiling callbacks
- `StageUpdateFns` - stage lifecycle callbacks

**SimulationRegistry** manages multiple engines simultaneously, with event subscriptions for registration, activation, and deactivation.

---

## Slide 6: Build System

**Standalone CMake build** with three targets:

1. **`PhysicsUmbrella`** (shared library / DLL)
   - Core C++ implementation, Carbonite linking

2. **`_physics`** (Python extension module / PYD)
   - pybind11 bindings, depends on PhysicsUmbrella + Carbonite

3. **`PhysicsUmbrellaTests`** (C++ test executable)
   - doctest framework

**Dependencies via Packman:** pybind11, Python 3.12, Carbonite SDK

Cross-platform: Windows (MSVC/Ninja) and Linux supported.

---

## Slide 7: Python Bindings

The `_physics` pybind11 module exposes all six interfaces to Python:

```python
import omni.physics.umbrella as phys

# Acquire interfaces (Carbonite pattern)
physics = phys.acquire_physics_interface()
sim     = phys.acquire_physics_simulation_interface()
query   = phys.acquire_physics_scene_query_interface()

# Register simulations, run queries, subscribe to events...
```

**Key types exposed:** `IPhysics`, `Simulation`, `SimulationId`, `ContactEventHeader`, `ContactData`, `PhysicsStepContext`, `ForceMode`, and more.

Post-build staging copies `.pyd`, `.dll`, and `carb.dll` to `_build/bindings/` for a clean development workflow.

---

## Slide 8: Test Coverage

### C++ Tests (doctest) - 8 test cases, 63 assertions
- Simulation interface lifecycle
- SimulationRegistry (register/unregister/activate)
- Registry events & subscriptions
- Scene query operations
- Benchmark, Interaction, StageUpdate interfaces
- Error callback handling

### Python Tests (unittest/pytest) - 43 tests across 6 files
- `testSimulatorSimulation.py` - simulation lifecycle
- `testSimulationRegistry.py` - registry operations
- `testSimulatorSceneQuery.py` - query bindings
- `testSimulatorBenchmark.py` - profiling bindings
- `testSimulatorInteraction.py` - interaction bindings
- `testSimulatorStageUpdate.py` - stage update bindings

---

## Slide 9: Developer Experience

- **VS Code integration** - launch configs, tasks, extensions recommendations
- **One-command builds** - `build.bat` / `build.sh` handle dependency pull + CMake + compile
- **Packman dependency management** - reproducible builds with pinned dependencies
- **Shared CMake modules** - `CompilerDefaults.cmake`, `NvidiaBuildOptions.cmake` reusable across projects
- **`.gitignore`** configured for `_build/` artifacts

---

## Slide 10: Key Design Decisions

1. **Core has zero external dependencies** - pure C++, Carbonite is optional
2. **Function-pointer plugin model** - allows multiple physics engines to coexist
3. **Source files moved, not copied** - clean migration from `ovruntime/plugins/`
4. **Carbonite `acquire/release` pattern** preserved for API compatibility
5. **Separate public headers** (`include/`) from implementation (`source/`)
6. **Dual test harness** - C++ (doctest) + Python (unittest/pytest) for full coverage

---

## Slide 11: What's Next

- Integration testing with actual physics engines (PhysX)
- USD stage loading through the umbrella interfaces
- Wheel packaging (`pyproject.toml` + `setup.py` already scaffolded)
- CI/CD pipeline integration
- Merge to trunk
