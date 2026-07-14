# Physics Umbrella C++ Tests

This directory contains comprehensive C++ tests for the standalone physics umbrella library.

## Test Organization

Tests are organized by functional group for clarity and maintainability:

### Core Tests

- **test_umbrella_basic.cpp** - Basic functionality tests without external dependencies
  - Interface initialization
  - Basic types (Float3, Float4, SimulationId)
  - Registration and counting

### Doctest-based Test Suite

All tests below use the doctest framework and are compiled into `test_umbrella_suite.exe`:

- **doctest_main.cpp** - Test runner entry point

#### Functional Group Tests

1. **TestSimulationInterface.cpp** - Simulation interface and structure
   - Basic simulation structure
   - Function pointer attachment

2. **TestSimulationRegistry.cpp** - Simulation registry management
   - Create simulation
   - Register/unregister simulation
   - Get number of simulations
   - Activate/deactivate simulation
   - Get simulation IDs
   - Invalid simulation operations

3. **TestSimulatorSimulation.cpp** - Simulation functions
   - AttachStage and DetachStage
   - Simulate and FetchResults

4. **TestSimulatorStageUpdate.cpp** - Stage update lifecycle
   - Stage lifecycle callbacks (onAttach, onDetach, onUpdate, onResume, onPause, onReset)

5. **TestSimulatorSceneQuery.cpp** - Scene queries
   - Raycast functions (raycastClosest, raycastAny)
   - Overlap functions (overlapSphere, overlapSphereAny)

6. **TestSimulatorInteraction.cpp** - User interactions
   - Reset on stop control
   - Raycast handling

7. **TestSimulatorBenchmark.cpp** - Performance monitoring
   - Profile stats subscription/unsubscription

8. **TestRegistryEvents.cpp** - Event notification system
   - Simulation registry event notifications

## Building Tests

Tests are built as part of the main umbrella library build:

```bash
cd umbrella
./build.bat    # Windows
./build.sh     # Linux/Mac
```

Or build specific test targets:

```bash
cd umbrella/_build/windows-x86_64
cmake --build . --config Debug --target test_umbrella_basic
cmake --build . --config Debug --target test_umbrella_suite
```

## Running Tests

### Basic Test
```bash
umbrella/_build/windows-x86_64/bin/Debug/test_umbrella_basic.exe
```

### Full Test Suite
```bash
umbrella/_build/windows-x86_64/bin/Debug/test_umbrella_suite.exe
```

### Run Specific Test Cases
```bash
# Run only SceneQuery tests
test_umbrella_suite.exe --test-case="Scene Query*"

# List all test cases
test_umbrella_suite.exe --list-test-cases

# Show detailed output
test_umbrella_suite.exe --success
```

## Test Coverage

- **8 test cases**
- **80 assertions**
- **All 5 function groups** covered (Simulation, StageUpdate, SceneQuery, Interaction, Benchmark)
- **Registry events** covered
- **Error handling** covered

## Dependencies

- **doctest 2.4.5** - C++ testing framework (from `_build/target-deps/doctest/include`)
- **C++17** - Standard library
- **PhysicsUmbrella** - The library being tested

## Notes

- Expected error messages may appear during tests (e.g., "simulation not found") - these are part of testing error handling with invalid IDs
- Minor compiler warnings about unused parameters in lambda functions are cosmetic and do not affect functionality
