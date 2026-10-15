<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Native extension samples

## Direct runtime interfaces

Include `ovphysx_companion/custom_interfaces.h` explicitly to use the existing
`omni::physx::IPhysxCustomJoint` and `IPhysxCustomGeometry` tables and original
callback structures. Include paths are the installed SDK `include` and
`include/physx`; neither Carbonite nor USD headers are required. Link the existing
ovphysx library, with no PhysX static library.

The [direct registration consumer](direct_custom.h) requires an already acquired,
validated interface table. It shows explicit callbacks and retryable registration
retirement. The [checked acquisition and lifetime tests](../../../tests/c_unittests/test_companion_custom.cpp)
show how to acquire the table, validate its compatibility, and retain its runtime lease.

Keep the lease, callback code and context until registrations retire successfully.
The lease creates no scene and retains the existing runtime even if its bootstrap
instance closes. Registrations are shared by that runtime; live joint operations
use an exact `AttachHandle` and `ObjectKey`. The [direct registration consumer](direct_custom.h)
shows both original interfaces and retryable retirement.

## Optional compiled-module convenience

These modules use the installed companion C++ SDK and the exact matching PhysX
headers. They link no PhysX static libraries. The host imports provide non-inline
query utilities from the existing runtime owner. Native code uses no USD or Python.

```sh
cmake -S ovphysx/companion/samples/native -B /tmp/companion-plugins \
  -DOVPHYSX_SDK="$PWD/ovphysx/_install" -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/companion-plugins --config Release
PYTHONPATH="$PWD/ovphysx/companion/python" python \
  ovphysx/companion/samples/native_extensions.py /tmp/companion-plugins
```

Use the same compiler/runtime ABI and SDK as the owner. The SDK build digest
covers the headers and packaged archives; a different descriptor/build/compiler
or malformed callback table fails before startup. Module static initialization
must be inert. Every callback must remain compiled native and must not throw or
block waiting for owner operations.

- `spring.cpp` registers the genuine `CompanionSpringJoint` type. Its local frames
  account for center-of-mass and origin shifts; its solver emits one spring row.
  Explicit refresh copies parameters from the owner's current sealed input and
  marks its data dirty. ALWAYS_UPDATE can be set and cleared on the live joint.
- `ball.cpp` registers `CompanionBallAPI`. Each created shape owns a distinct state.
  It supplies mass, bounds, plane/sphere/own-ball contacts, raycast/overlap/sweep
  delegation and debug axes. Its other contact pairs return no contacts; this is
  a small teaching example, not a general replacement for a built-in sphere.
- `schema/resources` defines both external codeless schemas. Register those in
  customer USD and in population independently, before their first schema lookup.
  `population_description` retains custom fields and inherited joint body/frame
  properties. The PHYSICS domain alone is insufficient for an external joint.

v1 loading requires hard process CPU mode (`PhysX.set_cpu_mode(True)`); other
modes fail before plugin startup. Load modules before attaching their scenes. Keep them alive through object
teardown; close returns Busy while a native object or registration snapshot can
still call their code. After detaching every owner, retry close. Bootstrap core
instances may be destroyed earlier because modules retain runtime services.

The automated CPU tests simulate independent attachments, verify spring motion,
COM/origin frames, bounded aligned solver rows, ball mass/contact/query/debug
behavior, edited parameter ingestion and balanced release. They exercise live
object deletion, construction rejection, pending callback retirement, authored
ovstage clones, and unit-scale point instances with mixed ordinary/custom shapes.
Geometry refresh uses explicit owner teardown and reattachment with new sealed input.

`NativePlugin.capabilities` reports actual optional callbacks; `require(operation)`
returns `UNAVAILABLE` for absent modes. GPU/DirectGPU, articulation/tensor DOFs,
custom SDF fallback and mesh-merge processing are unavailable. Non-unit point
instance scale is rejected before create. Binary SDK callback-state replication
is not qualified; use authored clones/repopulation. The [matrix](../../docs/compatibility.md)
keeps these independent of ordinary CPU SDF cooking and other core capabilities.

`read_only.h` / `read_only.cpp` is a separate C++ consumer of `NativeAccessApi`.
Negotiate with this SDK's `Compatibility`, resolve a body using the inspection
table, and pass the API/reference to the sample. Its synchronous callback copies
a mass value; no SDK pointer escapes. The module links no PhysX runtime. This
pattern is for exceptional native inspection; Python uses copied records.
