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

### Select the matching toolchain first

Use the C SDK and companion source matching the **exact build** of the loaded
ovphysx library. A matching SDK alone is insufficient: the plugin's compiler/runtime
fingerprint must also match the owner. CMake otherwise selects the host's default
compiler and can successfully build a plugin that the owner rejects.

For the released **Linux x86_64 ovphysx 0.7.0.73411310** SDK and wheel, use
**GCC 11.4.0**, the GCC C++ ABI value **1016**, the libstdc++ C++11 ABI
(`_GLIBCXX_USE_CXX11_ABI=1`), and **Release** configuration. The required compiler
fingerprint is `gcc:11.4.0:abi:1016:stdlib:1`. This value is specific to that
build; do not assume it applies to every 0.7 SDK or another platform.

Select a compiler installation with those values before configuring. For example,
if GCC 11.4.0 is installed as `/usr/bin/g++-11`, check its macros with:

```sh
CXX=/usr/bin/g++-11
"$CXX" -std=c++17 -dM -E -x c++ -include string /dev/null \
  | grep -E '^#define (__VERSION__|__GXX_ABI_VERSION|_GLIBCXX_USE_CXX11_ABI) '
```

For that release, the values must be `"11.4.0"`, `1016`, and `1`, respectively.
The executable name `g++-11` alone does not establish the patch version. The
fingerprint is constructed in
[`native_compatibility.h`](../../include/ovphysx_companion/native_compatibility.h)
from these macros. The installed `native_build.h` records the SDK digest and debug
mode, but does **not** record the owner's compiler fingerprint. For a locally built
owner, use its recorded CMake compiler, matching compiler/runtime flags and build
configuration. For other released builds, obtain the compiler/runtime configuration
from that build's producer before building plugins.

Set `OVPHYSX_SDK` below to the extracted matching C SDK, or to `ovphysx/_install`
when using the owner built in this checkout. Run from the repository root, using a
fresh build directory when changing compilers or SDKs:

```sh
OVPHYSX_SDK="$PWD/ovphysx/_install"
cmake -S ovphysx/companion/samples/native -B /tmp/companion-plugins \
  -DOVPHYSX_SDK="$OVPHYSX_SDK" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER="$CXX"
cmake --build /tmp/companion-plugins --config Release
```

Before running the scene sample, a minimal Linux CPU loader check can verify the
compiled spring plugin against the matching installed wheel. Run it in a fresh
Python process; it needs no populated scene:

```sh
PYTHONPATH="$PWD/ovphysx/companion/python" python - <<'PY'
from ovphysx import PhysX
from ovphysx_companion.plugins import NativePlugin

PhysX.set_cpu_mode(True)
physics = PhysX()
try:
    with NativePlugin(physics, "/tmp/companion-plugins/libcompanion_spring.so"):
        print("Plugin loaded with the selected toolchain")
finally:
    physics.destroy()
PY
```

Then run the scene sample:

```sh
PYTHONPATH="$PWD/ovphysx/companion/python" python \
  ovphysx/companion/samples/native_extensions.py /tmp/companion-plugins
```

For example, a default GCC 13.3.0 build produces
`gcc:13.3.0:abi:1018:stdlib:1` and is rejected by the 0.7.0.73411310 owner with
`ABI_MISMATCH`. Rebuild with the matching toolchain. Do not redefine compiler
identity macros or edit the descriptor to pass admission: that does not establish
binary compatibility. Other descriptor mismatches can produce the same status.

### Plugin contracts

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
  properties. The physics population description alone is insufficient for an external joint.

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
