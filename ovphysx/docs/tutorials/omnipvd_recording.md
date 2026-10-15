<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# OmniPVD Recording -- Capture Physics Internals to FILE or TCP

This tutorial shows how to record OmniPVD data from ovphysx to `.ovd` files
for offline inspection or stream it live to a TCP listener. OmniPVD captures
selected PhysX object and contact state each frame so you can debug and
visualize physics behavior.

## Prerequisites

- Install ovphysx and confirm native libraries load.
- A USD scene with physics objects. This tutorial uses `links_chain_sample.usda`,
  which ships with every package under `ovphysx/samples/data/` in the wheel,
  `<sdk-root>/samples/data/` in the C/C++ SDK, and `tests/data/` in a repository
  checkout.
- For offline inspection: a compatible Kit application with the OmniPVD extension.

## Required Config

Set `GlobalConfigKey.OVD_RECORDING_CAPABLE` to `True` with module-level
`set_global_config()` before `PhysX()`. In C, initialize and set
`OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE` with
`ovphysx_config_value_bool(true)` before plain `ovphysx_create_instance()`.
This process-wide BOOL bootstrap option installs the provider and sampler when
`PxPhysics` is created. It defaults to false; without it, late start fails.
Capability alone leaves output idle but retains sampler, listener, mutex, and
scene readback costs. Only the default disabled configuration avoids those costs.
The scoped setter is rejected after the first successful create until successful
shutdown with no live instances, including after all native instances are destroyed.
Python's `PhysX.destroy()` releases its lifecycle owner; releasing the last Python
owner performs that shutdown, provided no native instances remain, and reopens
the setter. Its configured value survives shutdown.
The getter reports the configured bool. Failed creates do not freeze the setter.

Attach and initialize the scene, then call `start_recording()` with an
`OmniPvdDestination`. To capture from the first simulated frame, start before
the first step. FILE uses one exact path whose parent exists; TCP requires a
ready listener. Stop finalizes the output and allows a new destination on the
same instance. TCP is synchronous trusted plaintext.

Creation, attach, and reattach never start output automatically. Old startup
configuration is rejected; see the [API migration guide](../ovd_api_migration.md).

## Code

### Python

This sample provisions capability, attaches `links_chain_sample.usda`, records
to an exact FILE destination before the first step, then stops and tears down:

```{literalinclude} ../../tests/python_samples/omnipvd_recording.py
:language: python
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

### C++

**CMakeLists.txt**

The C++ sample uses the same `find_package(ovphysx)` and `ovphysx::ovphysx`
target as every other sample:

```{literalinclude} ../../tests/c_samples/omnipvd_recording_cpp/CMakeLists.txt
:language: cmake
```

**Source**

The C++ sample starts an exact FILE recording after attaching the scene and
before the first simulation step:

```{literalinclude} ../../tests/c_samples/omnipvd_recording_cpp/main.cpp
:language: cpp
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

### Late TCP Recording

Opt in before creating the first instance, before attaching and stepping the
scene. Start the trusted-plaintext TCP listener before calling
`start_recording()`; the connect is synchronous. This complete sequence
simulates before recording, records to TCP, stops, continues simulating, and
then records to a FILE destination. Replace `scene.usda` with a path to your own
USD scene, such as the `links_chain_sample.usda` file named in the
prerequisites:

```python
import ovstage
from ovphysx import GlobalConfigKey, OmniPvdDestination, PhysX, newton_schema_root, set_global_config
from ovphysx.samples.python_samples.schema_support import physx_schema_paths

set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)
physx = PhysX()
# Register PhysX and Newton schemas before the first USD schema read (population or export).
ovstage.population.register_usd_schemas([*[str(p) for p in physx_schema_paths()], str(newton_schema_root())])
stage = ovstage.Stage("late-recorded-scene")
ovstage.population.open_usd(
    stage, "scene.usda", ordinal=1, domains=ovstage.PopulationDomain.ALL
)
stage.advance_write_floor(ordinal=1).wait()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

physx.step_sync(1 / 60)  # work before recording is allowed
physx.start_recording(
    OmniPvdDestination.tcp("127.0.0.1", 5425, timeout_ms=3000)
)
assert physx.is_recording()
physx.step_sync(1 / 60)  # streamed to the TCP listener
physx.stop_recording()

physx.step_sync(1 / 60)  # simulation continues without recording
physx.start_recording(OmniPvdDestination.file("capture.ovd"))
physx.step_sync(1 / 60)  # written to capture.ovd
physx.stop_recording()

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
```

#### C and C++

To adapt the complete C++ sample in [Code](#code), keep its initialization,
stage attachment, and cleanup. The sample already enables
recording capability and starts recording explicitly. Keep this configuration
when replacing its FILE destination and recording loop with the TCP sequence
below. Use `create_recording_instance()` for instance creation, checking its
returned status. Remove the sample's final file-count check; a TCP session
writes no local file:

```c
#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_config.h>

static ovphysx_result_t create_recording_instance(ovphysx_instance_handle_t* handle)
{
    const ovphysx_result_t configured = ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE, ovphysx_config_value_bool(true));
    if (configured.status != OVPHYSX_API_SUCCESS)
        return configured;
    return ovphysx_create_instance(handle);
}
```

Call this helper after `ovphysx_initialize()` and before creating any instance.
After it succeeds and a physics stage is attached,
replace the sample's FILE recording sequence with `record_tcp(handle)`,
then perform the sample's cleanup and return its result.
The preliminary step demonstrates starting after simulation has already begun;
omit it to record from the first simulation frame:

```c
#include <ovphysx/ovphysx.h>

static int record_tcp(ovphysx_instance_handle_t handle)
{
    const ovphysx_omnipvd_destination_t destination = {
        OVPHYSX_OMNIPVD_TRANSPORT_TCP,
        OVPHYSX_LITERAL(""),
        OVPHYSX_LITERAL("127.0.0.1"),
        5425,
        3000,
    };

    ovphysx_result_t result = ovphysx_step_sync(handle, 1.0f / 60.0f);
    if (result.status != OVPHYSX_API_SUCCESS)
        return 1;

    result = ovphysx_ovd_start_recording(handle, &destination);
    if (result.status != OVPHYSX_API_SUCCESS)
        return 1;

    bool recording = false;
    result = ovphysx_ovd_is_recording(handle, &recording);
    if (result.status != OVPHYSX_API_SUCCESS || !recording)
        return 1;

    result = ovphysx_step_sync(handle, 1.0f / 60.0f);
    if (result.status != OVPHYSX_API_SUCCESS)
        return 1;

    result = ovphysx_ovd_stop_recording(handle);
    if (result.status != OVPHYSX_API_SUCCESS)
        return 1;
    return 0;
}
```

If the listener is not ready, `start_recording()` fails without consuming the
session; start the listener and retry. After `stop_recording()`, the same
instance can later record to another TCP listener or to an exact FILE path.

FILE uses the exact requested path, and TCP connects synchronously to an
already-ready listener. Failed opens can be retried. Only one stream can be
active in the shared runtime, and an active stream cannot be replaced. After
stop, the owning instance can start another FILE or TCP session. The instance
that explicitly starts recording owns it; peers report inactive and cannot stop
its session. Detaching the active stage
finalizes the shared session and clears its public owner, including when a peer
handle started it. Reattach remains dormant until another explicit start.

These synchronous APIs follow the ovphysx same-thread contract. The caller must
serialize recording calls on each handle and recording/attach/detach/destroy
transitions across all handles sharing the runtime; concurrent calls are not
supported. Recording is supported on Windows x86_64 and Linux x86_64/aarch64.
Initial and restarted sessions each capture the current core PhysX,
PhysXExtensions (including joints and custom geometry), and PhysXVehicle state.
Stopping releases only the session's telemetry handles; the next start takes a
fresh full-state snapshot.

## What Happens at Runtime

1. FILE opens its exact path or TCP connects before sampling begins.
2. Start snapshots the current world; each subsequent step records its state.
3. Stop finalizes output. Another recording can start on the same capable runtime.

## Inspecting .ovd Files in Kit

Before you begin, stop the FILE recording to finalize the exact path you
requested. If output is missing or incomplete, check the runtime log and verify
that the parent directory exists and is writable. You also need the compatible
Kit-based application with the OmniPVD extension named in the prerequisites.

Then inspect the recording:

1. Open a compatible Kit-based application (for example, USD Composer or Isaac Sim with a compatible OmniPVD extension generation).
2. Enable the **OmniPVD** extension (`omni.physx.pvd`) from **Window > Extensions**.
3. Use **File > Open** or the OmniPVD panel to load the `.ovd` file.
4. Use the timeline scrubber to step through recorded frames and inspect shapes, contacts, and solver state.

Inspection succeeded when the OmniPVD panel lists your `.ovd` file as imported
and the timeline scrubber advances through recorded frames, with shapes,
contacts, and solver state updating as you scrub. A file the panel refuses to
import can be unreadable, malformed, truncated, or incompatible with the reader.
Check the reader's reported error first. If it reports a format or version
mismatch, refer to [Capture Format and Compatibility](#capture-format-and-compatibility)
for the two version checks the reader applies.

For more details on the Kit-side OmniPVD workflow, refer to the PhysX Visual Debugger documentation included with the Kit application you use for inspection.

## Capture Format and Compatibility

ovphysx OmniPVD capture is optional. When enabled, the runtime writes PhysX OmniPVD
recordings in the **OVD** binary format. FILE output keeps its exact requested path.

**OVD capture format and compatibility policy**

This table records the format, version, and platform policy for ovphysx OmniPVD
capture:

| Item | Policy |
|---|---|
| Format | PhysX OmniPVD OVD command stream (not USD, not a general capture/replay API) |
| OmniPVD stream version | Each recording begins with a 12-byte OmniPVD version header. The writer in the PhysX SDK pinned by this ovphysx build emits **0.4.0** (`OMNI_PVD_VERSION_*` in `OmniPvdDefines.h`). |
| PhysX OVD integration version | The stream's `PxOmniPvdMetaData` object independently identifies the PhysX object schema as **3.1** (`PX_PHYSICS_OVD_INTEGRATION_VERSION_*` in `PxOmniPvd.h`). Integration-major changes can remove or change existing schema; integration-minor changes are additive. This version is separate from the 0.4.0 command-stream version. |
| Canonical reader | Kit OmniPVD extension `omni.physx.pvd` (import OVD → cached USDA stage for scrubbing / inspection) |
| Reader compatibility | Both versions must be compatible. The OmniPVD runtime reader rejects a command-stream version newer than its own `(major, minor, patch)`. Separately, when the integration metadata is present, the PvdDom reader paired with this build's pinned PhysX SDK accepts integration major 3 or older and rejects a stream with a newer integration major; it treats missing metadata as legacy. Integration-minor changes are additive, and this reader does not reject solely because the stream minor is newer. |
| ovphysx version coupling | ovphysx does not define a separate OVD version. Compatibility follows the PhysX / OmniPVD runtime pinned by that ovphysx build and the Kit OmniPVD extension that reads the file. Prefer matching generations: inspect with a Kit build whose OmniPVD extension supports both the recorded OmniPVD stream version and the PhysX OVD integration major. |
| Platform | OmniPVD recording is supported on Windows x86_64 and Linux x86_64/aarch64. PVDRuntime is statically linked into the shipped native library; no separate PVDRuntime `.dll`/`.so` is deployed. |

This is the public compatibility story for ovphysx OmniPVD capture. If you need a
guaranteed cross-release matrix beyond the two Reader compatibility checks in
[Capture Format and Compatibility](#capture-format-and-compatibility), confirm
with the OmniPVD owners before treating an older Kit reader as supported against
a newer writer.

## Troubleshooting

These symptoms cover the configuration and lifecycle mistakes that stop a
recording from appearing:

**OmniPVD recording symptoms and fixes**

| Symptom | Cause | Fix |
|---|---|---|
| No `.ovd` file after simulation | Capability or explicit start was omitted | Set `OVD_RECORDING_CAPABLE` before creation, attach the scene, and start a FILE recording |
| Output is incomplete | Recording was not finalized | Call `stop_recording()` before teardown |
| FILE `start_recording()` fails ("failed to open the destination or start sampling") | Parent directory is missing or not writable | Create the parent directory first (FILE does not create directories), then retry |
| TCP start fails | Listener was not ready or address/port was wrong | Start the trusted plaintext listener before calling `start_recording()`, then retry |

## Result

After this tutorial you can stream live OmniPVD data over TCP or capture `.ovd`
recordings from an ovphysx simulation for offline inspection in Kit.
