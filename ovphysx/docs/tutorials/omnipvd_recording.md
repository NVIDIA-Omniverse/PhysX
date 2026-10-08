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

Startup OmniPVD output and late-recording capability are selected at instance
creation. The default FILE startup transport also uses a recording directory:

**Startup output and late-recording capability fields**

| Config field (Python) | C builder | Description |
|---|---|---|
| `omnipvd_ovd_recording_directory` | `ovphysx_config_entry_omnipvd_ovd_recording_directory()` | Writable directory where `.ovd` files are saved |
| `omnipvd_output_enabled` | `ovphysx_config_entry_omnipvd_output_enabled()` | Enables OmniPVD data capture |
| `omnipvd_recording_capable` | `ovphysx_config_entry_omnipvd_recording_capable()` | Permits recording to start later without enabling startup output |

Set `omnipvd_output_enabled` before creating the PhysX instance. For FILE, also set
`omnipvd_ovd_recording_directory`; TCP does not use it. Pass fields through `PhysXConfig`
(Python) or `config_entries` in `ovphysx_create_args` (C/C++).

Startup output implicitly installs late-recording capability. The capability is
process-wide and fixed when the shared runtime is created. With startup output
and capability both disabled, the default instance deliberately passes a null
`PxOmniPvd` to PhysX and cannot start recording later; this avoids incremental
sampler, factory-listener, and sampling-mutex overhead. This is the only
zero-OmniPVD-overhead configuration. Explicit late-recording capability creates
the provider and sampler at `PxPhysics` creation and enables OVD and collision
readback on attached scenes even while recording is idle; DirectGPU scenes
therefore retain that readback cost.

For FILE, the runtime auto-creates the recording directory if it does not exist.
FILE is the default startup transport. TCP startup adds four fields:

**TCP startup transport fields**

| Config field (Python) | C builder | Description |
|---|---|---|
| `omnipvd_transport` | `ovphysx_config_entry_omnipvd_transport()` | Exact lowercase `"file"` or `"tcp"` |
| `omnipvd_tcp_address` | `ovphysx_config_entry_omnipvd_tcp_address()` | Ready listener address |
| `omnipvd_tcp_port` | `ovphysx_config_entry_omnipvd_tcp_port()` | Listener port, 1 through 65535 |
| `omnipvd_tcp_timeout_ms` | `ovphysx_config_entry_omnipvd_tcp_timeout_ms()` | Blocked-send timeout in milliseconds; 0 leaves it at the OS default and uses a 3000 ms connect window |

With TCP, ovphysx is the client. Start the listener before constructing the
PhysX instance: the connection is synchronous. TCP is trusted plaintext, so use
it only on a trusted network.

## Code

### Python

This complete sample enables startup FILE output with a recording directory,
attaches `links_chain_sample.usda`, steps for two seconds, destroys the instance
so the runtime finalizes the recording, and then lists the `*_rec.ovd` files it
produced:

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

### TCP Startup

To stream to a TCP listener from instance creation, pass the transport and
listener fields together:

```python
physx = PhysX(config=PhysXConfig(
    omnipvd_output_enabled=True,
    omnipvd_transport="tcp",
    omnipvd_tcp_address="127.0.0.1",
    omnipvd_tcp_port=5425,
    omnipvd_tcp_timeout_ms=3000,
))
```

The equivalent compact C config is:

```c
ovphysx_config_entry_t config[] = {
    ovphysx_config_entry_omnipvd_output_enabled(true),
    ovphysx_config_entry_omnipvd_transport(OVPHYSX_OMNIPVD_TRANSPORT_TCP_NAME),
    ovphysx_config_entry_omnipvd_tcp_address(OVPHYSX_LITERAL("127.0.0.1")),
    ovphysx_config_entry_omnipvd_tcp_port(5425),
    ovphysx_config_entry_omnipvd_tcp_timeout_ms(3000),
};
ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
args.config_entries = config;
args.config_entry_count = 5;
```

**Source**

The C++ sample records startup FILE output through the C API, but it attaches
`simple_physics_scene.usda` and runs 10 steps:

```{literalinclude} ../../tests/c_samples/omnipvd_recording_cpp/main.cpp
:language: cpp
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

### Late TCP Recording

Opt in when creating the first instance, before attaching and stepping the
scene. Start the trusted-plaintext TCP listener before calling
`start_recording()`; the connect is synchronous. This complete sequence
simulates before recording, records to TCP, stops, continues simulating, and
then records to a FILE destination. Replace `scene.usda` with a path to your own
USD scene, such as the `links_chain_sample.usda` file named in the
prerequisites:

```python
import ovstage
from ovphysx import OmniPvdDestination, PhysX, PhysXConfig, codeless_schema_root

physx = PhysX(config=PhysXConfig(omnipvd_recording_capable=True))
# Register the codeless PhysX schemas before the first population call.
ovstage.population.register_usd_schemas([str(codeless_schema_root())])
stage = ovstage.Stage("late-recorded-scene")
ovstage.population.open_usd(
    stage, "scene.usda", ordinal=1, domains=ovstage.PopulationDomain.PHYSICS
)
stage.advance_write_floor(ordinal=1).wait()
physx.attach_ovstage(stage, read_ordinal=1)

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

physx.detach_ovstage()
physx.destroy()
stage.destroy()
```

#### C and C++

To adapt the complete C++ sample in [Code](#code), reuse its initialization, instance,
stage-attachment, and cleanup scaffolding. Remove its recording-directory
setup, startup-FILE comments, and final `*_rec.ovd` file-count check. Replace
the startup-output config with late-recording capability before creating the
first instance. After the existing stage, instance, and runtime cleanup, return
`0` in place of the removed file-count block.

```c
const ovphysx_config_entry_t config[] = {
    ovphysx_config_entry_omnipvd_recording_capable(true),
};
ovphysx_create_args create_args = OVPHYSX_CREATE_ARGS_DEFAULT;
create_args.config_entries = config;
create_args.config_entry_count = 1;
```

After creating `handle` with those arguments and attaching a physics stage,
replace the sample's simulation loop with the following sequence. Its first
successful step initializes lazy physics before the late start:

```c
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

result = ovphysx_start_recording(handle, &destination);
if (result.status != OVPHYSX_API_SUCCESS)
    return 1;

bool recording = false;
result = ovphysx_is_recording(handle, &recording);
if (result.status != OVPHYSX_API_SUCCESS || !recording)
    return 1;

result = ovphysx_step_sync(handle, 1.0f / 60.0f);
if (result.status != OVPHYSX_API_SUCCESS)
    return 1;

result = ovphysx_stop_recording(handle);
if (result.status != OVPHYSX_API_SUCCESS)
    return 1;
```

If the listener is not ready, `start_recording()` fails without consuming the
session; start the listener and retry. After `stop_recording()`, the same
instance can later record to another TCP listener or to an exact FILE path.

FILE uses the exact requested path, and TCP connects synchronously to an
already-ready listener. Failed opens can be retried. Only one stream can be
active in the shared runtime, and an active stream cannot be replaced. After
stop, the owning instance can start another FILE or TCP session. Startup output
is owned by the instance whose creation started it: that instance can query and
publicly stop the startup session and restart to a late destination, while peer
instances report inactive and cannot stop it. On cold startup the creator's
ownership is reserved when creation succeeds and becomes observable when the
first stage attach starts sampling. Detaching the active stage
finalizes the shared session and clears its public owner, including when a peer
handle started it. After reattach, capability-only recording is dormant and can
start immediately. Configured startup output instead starts a new startup
session owned by the reattaching instance; stop it before selecting a late
destination.

These synchronous APIs follow the ovphysx same-thread contract. The caller must
serialize recording calls on each handle and recording/attach/detach/destroy
transitions across all handles sharing the runtime; concurrent calls are not
supported. Recording is supported on Windows x86_64 and Linux x86_64/aarch64.
Startup, late, and restarted sessions each capture the current core PhysX,
PhysXExtensions (including joints and custom geometry), and PhysXVehicle state.
Stopping releases only the session's telemetry handles; the next start takes a
fresh full-state snapshot.

## What Happens at Runtime

A recording session moves through these stages:

1. Late FILE opens its exact path, and TCP connects its socket, before the runtime binds that stream to the OmniPVD writer. A late FILE path's parent directory must already exist.
2. Startup FILE output creates `tmp.ovd`; late FILE output opens the exact requested path. TCP connects to the already-ready listener.
3. Each simulation step writes physics state to the selected stream.
4. Stop, active detach, or active-owner destruction cleanly finalizes the stream. Only startup FILE output renames `tmp.ovd` to `YYYY_MM_DD_HH_MM_SS_CC_rec.ovd` and updates the Kit import directory; late FILE keeps its exact path.

With FILE startup output, an empty recording directory disables startup
recording. `omnipvd_output_enabled=false` disables startup capture, but an
instance created with `omnipvd_recording_capable=true` can still start an exact
FILE or TCP session later.

## Inspecting .ovd Files in Kit

Before you begin, confirm two things. First, you need a finalized recording:
startup FILE output leaves a `YYYY_MM_DD_HH_MM_SS_CC_rec.ovd` file in the
recording directory after clean shutdown, and late FILE output leaves the exact
path you requested. A remaining `tmp.ovd` means startup output was not finalized.
Possible causes include an instance that was not destroyed cleanly, failure to
create the output directory, or failure to rename the temporary file. Check the
runtime log for a filesystem error and verify that the destination is writable
before continuing. Second, you need the compatible Kit-based application with
the OmniPVD extension named in the prerequisites.

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
recordings in the **OVD** binary format (startup FILE output is named
`*_rec.ovd` after clean finalization; late FILE output keeps its exact path).

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
| No `.ovd` file after simulation | FILE directory or output enablement was omitted before instance creation | Pass both FILE fields in `PhysXConfig` / `config_entries` at init |
| `tmp.ovd` exists but no `*_rec.ovd` | Instance not properly destroyed | Ensure `destroy()` / `ovphysx_destroy_instance` is called |
| Runtime error about directory | Directory path is invalid or not writable | Use an absolute path to a writable location |
| TCP start fails | Listener was not ready or address/port was wrong | Start the trusted plaintext listener before creating the instance or calling `start_recording()`, then retry |

## Result

After this tutorial you can stream live OmniPVD data over TCP or capture `.ovd`
recordings from an ovphysx simulation for offline inspection in Kit.
