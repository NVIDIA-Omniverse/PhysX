# PhysX Interop: Direct PhysX SDK Access

This tutorial shows how to get a raw PhysX SDK pointer from ovphysx and call
PhysX methods directly. The sample moves a kinematic rigid body using
`PxRigidDynamic::setKinematicTarget()`, then reads back the pose through a tensor
binding to confirm the move.

## Prerequisites

- Complete the [Hello World](hello_world.md) tutorial.
- Familiarity with the ovphysx C API (`ovphysx_get_physx_ptr()`).
- A C++17 compiler.

## Setup

The ovphysx SDK ships PhysX headers under `include/physx/`. Include them in your
project through the `ovphysx_PHYSX_INCLUDE_DIR` CMake variable (set automatically by
`find_package(ovphysx)`). No PhysX library linking is needed.

## CMakeLists.txt

```{literalinclude} ../../tests/c_samples/physx_interop_cpp/CMakeLists.txt
:language: cmake
```

Note: `ovphysx_PHYSX_INCLUDE_DIR` is set automatically by `find_package(ovphysx)`.

## Source

```{literalinclude} ../../tests/c_samples/physx_interop_cpp/main.cpp
:language: cpp
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

## How It Works

1. **Create instance and load USD** — standard ovphysx workflow.
2. **Step once** — initializes the PhysX scene so rigid body actors exist.
3. **Get pointer** — `ovphysx_get_physx_ptr()` returns a `void*` for the prim path.
4. **Cast and validate** — `OVPHYSX_PHYSX_TYPE_ACTOR` can return either a
   `PxRigidDynamic*` or `PxRigidStatic*`, so cast to `PxRigidActor*` first, then
   use `is<PxRigidDynamic>()` to validate the concrete type before calling
   `setKinematicTarget()`.
5. **Step again** — PhysX moves the kinematic body to the target pose.
6. **Verify** — read pose back through tensor binding to confirm the move.

## Pointer Lifecycle

- Pointers are valid until `ovphysx_reset_stage()` or instance destruction.
- `ovphysx_step()` does **not** invalidate pointers.
- Do not call `release()` on returned pointers — ovphysx owns them.

## Thread Safety

PhysX APIs on returned pointers must only be called **between** simulation
steps — after `wait_op()` completes for the preceding step and before the next
`ovphysx_step()` call. Refer to the
[developer guide](../developer_guide.md#thread-safety) for details.

## Result

After running the sample, you should see output confirming the kinematic body
moved to the target position `(3, 2, 0)` through a direct PhysX `setKinematicTarget()` call.
