<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Instance and OVD API migration

0.7 uses `ovphysx_instance_handle_t` for an instance and
`ovphysx_ovd_start_recording`, `ovphysx_ovd_stop_recording`, and
`ovphysx_ovd_is_recording` for recording. The handle remains `uint64_t`; zero
is invalid. The renamed recording functions preserve ownership, shared-runtime
semantics, destination layout, and explicit recording behavior. Python method
names stay unchanged.
Other 0.6 to 0.7 source breaks are in the [migration guide](migrating_0_6_to_0_7.md).

## C start signature

`ovphysx_ovd_start_recording` now takes an optional third argument for recording
options. Rebuild native clients against the matching SDK and pass `NULL` (C) or
`nullptr` (C++) to keep recording all objects and data:

```c
ovphysx_ovd_start_recording(instance, &destination, NULL);
```

For selection, initialize `ovphysx_omnipvd_recording_options_t` with
`OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT`, set its scene/object paths or
`record_secondary_data`, and pass its address. The previous two-argument C ABI
is not compatible with this signature. Python keeps `start_recording(destination)`
and accepts `filter=` without changing existing calls.

## Compatibility boundary

Old C recording names remain deprecated forwarding exports in 0.7, and
`ovphysx_handle_t` remains a deprecated typedef of `ovphysx_instance_handle_t`.
They were kept in this release. New code uses the names above. Existing
binaries keep their recording symbols. Retired configuration slots are reserved.
Current Python bindings require the matching native library with the canonical
exports.

The current ovphysx runtime no longer supports startup recording configuration or
`PhysXConfig.omnipvd_recording_capable`. The legacy OVD config header and Python
module from 0.7 development builds are removed. Old typed numeric keys and raw Carbonite OVD startup keys
return `OVPHYSX_API_INVALID_ARGUMENT` with guidance to use the global capability
key and explicit recording calls. Their numeric slots are never reused.
Unrelated legacy configuration and recording-function forwarding aliases remain.
The runtime also removes automatic recording on attachment and output-directory
import finalization. Timestamped captures remain available through an explicit
FILE start with a directory path. Inherited Carbonite output, directory and
transport settings are inert: they neither enable capability nor start output,
and the runtime does not clear them or issue a startup-recording warning.

Every application using the current runtime must migrate, including host and
companion attachment paths. Kit applications maintained on separate Kit release
branches build their own runtime source and retain those branches' behavior.
That source boundary does not provide a startup recording option in this runtime.

## Configuration mapping

| Old key suffix | Old kind/value | 0.6 Python field / C builder suffix | Current destination |
|---|---|---|---|
| `OMNIPVD_OUTPUT_ENABLED` | bool / 3 | `omnipvd_output_enabled` | Explicit start/stop |
| `OMNIPVD_OVD_RECORDING_DIRECTORY` | string / 0 | `omnipvd_ovd_recording_directory` | Directory path in `destination.file_path` / `OmniPvdDestination.file(directory)` |
| `OMNIPVD_TRANSPORT` | string / 2 | `omnipvd_transport` | `destination.transport` |
| `OMNIPVD_TCP_ADDRESS` | string / 3 | `omnipvd_tcp_address` | `destination.tcp_address` |
| `OMNIPVD_TCP_PORT` | int32 / 2 | `omnipvd_tcp_port` | `destination.tcp_port` |
| `OMNIPVD_TCP_TIMEOUT_MS` | int32 / 3 | `omnipvd_tcp_timeout_ms` | `destination.tcp_timeout_ms` |
| `OMNIPVD_RECORDING_CAPABLE` | bool / 5 in 0.6.x (4 in 0.7 development builds) | `omnipvd_recording_capable` | `OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE` / `GlobalConfigKey.OVD_RECORDING_CAPABLE` |

Explicitly supplied raw Carbonite entries use the same rejection and migration
policy as the typed keys. Replace these paths with the following APIs:

| Retired Carbonite path | Current replacement |
|---|---|
| `/physics/omniPvdOutputEnabled` | Explicit recording start/stop |
| `/persistent/physics/omniPvdOvdRecordingDirectory` | Directory in `destination.file_path` / `OmniPvdDestination.file(directory)` |
| `/physics/omniPvdTransport` | `destination.transport` |
| `/physics/omniPvdTcpAddress` | `destination.tcp_address` |
| `/physics/omniPvdTcpPort` | `destination.tcp_port` |
| `/physics/omniPvdTcpTimeoutMs` | `destination.tcp_timeout_ms` / Python TCP destination `timeout_ms` |

Inherited values of these six paths are inert. Supplying them, or
`/physics/omniPvdRecordingCapable`, to creation or global configuration APIs
returns `OVPHYSX_API_INVALID_ARGUMENT`. The capability path stores
`OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE`: a value already present in a
shared Carbonite settings store is honoured and reported by the getter.
Set the global key before instance creation instead of writing the path.
`/physics/omniPvdIsOVDStage` is no longer read, and
`/persistent/physics/omniPvdImportDirectory` is no longer published.

The fields above are removed from `PhysXConfig`; the corresponding
`ovphysx_config_entry_<suffix>()` builders, `OVPHYSX_CONFIG_OMNIPVD_*` C keys,
and `OMNIPVD_*` members of Python `ConfigBool`, `ConfigInt32`, and `ConfigString`
are removed too. The C string constants `OVPHYSX_OMNIPVD_TRANSPORT_FILE_NAME`
and `OVPHYSX_OMNIPVD_TRANSPORT_TCP_NAME` are removed; use the explicit destination
transport enum instead.

Removed Python constructor keywords raise `TypeError` with migration guidance;
removed enum members raise `AttributeError`. Removed C builders are undeclared
(a compile error, for example an implicit-function-declaration diagnostic).
Raw entries passed through Python raise
`RuntimeError: ovphysx_create_instance() failed: Legacy OVD configuration was removed; ...`;
retired numeric and raw C entries return `OVPHYSX_API_INVALID_ARGUMENT` with the
same migration guidance.

To preserve the old directory workflow, pass that directory to the existing
FILE destination instead of submitting the removed key. Explicit start creates
missing directories and opens a new timestamped `*_rec.ovd` directly there.
It does not use an intermediate `tmp.ovd`, rename on stop, or publish an import
directory. A path ending in the literal lowercase `.ovd` selects an exact
filename instead; see the path rules below.

0.6 numeric values are not ABI-compatible with 0.7; rebuild against 0.7 headers.
Within 0.7, retired slots remain reserved and other values and COUNT sentinels
are not renumbered. Reserved slots must not be assigned new meanings.
Destination and transport types retain their `omnipvd` spellings in this
migration; no new transport or independent runtime is introduced.

## Replace automatic startup recording

The recording-capability flag is a process-global BOOL:
`OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE` (`0x0000000c`) in C and
`GlobalConfigKey.OVD_RECORDING_CAPABLE` in Python. There is no `ovd_capable`
constructor keyword. Configure it with `ovphysx_set_global_config` or
`set_global_config` before the first successful instance creation.

The default `false` creates no OmniPVD provider. Setting `true` provisions the
provider, sampler and scene readback without starting output; those resources
and readback costs are already enabled. Capability alone never
opens a FILE or connects to a TCP receiver. It cannot be added to an existing
runtime created without the provider.

Replace the old output-enabled flag and automatic attach behavior with this
application sequence:

1. Initialize the C API, then enable global recording capability before
   `ovphysx_create_instance()`. Python initializes through `PhysX()`, so set
   the global key before that constructor.
2. Populate and seal the ovstage, then attach it and collect its task receipt.
   Creation, attachment,
   reattachment and companion preview attachment do not start a session or
   stop an existing shared recording.
3. Call explicit start with a FILE or TCP destination immediately after attach.
   Start snapshots the initialized world. Starting before the first step
   captures that step without a preliminary simulation step.
4. Step while recording is active, then explicitly stop before releasing the
   attached stage. Stop finalizes the selected stream.
5. To record again, explicitly start a new destination on the same attached
   instance. To change capability, first complete the process teardown
   described below.

For applications that previously recorded every attachment, put attach and
start in one application helper. For example, Python can use:

```python
def attach_and_record(physx, stage, read_ordinal, destination):
    physx.wait_task(physx.attach_ovstage(stage, read_ordinal=read_ordinal))
    physx.start_recording(destination)
```

Do not insert a warmup step between these calls. If start fails, the successful
attachment remains available and the application can retry recording.

## Python: FILE and TCP

This helper accepts an application-owned, populated ovstage whose physics data
is sealed at the supplied nonzero ordinal. Keep that stage alive until the
helper returns. For a complete USD-loading example, use the
[recording tutorial and samples](tutorials/omnipvd_recording.md#code).

```python
from pathlib import Path
from ovphysx import (
    GlobalConfigKey, OmniPvdDestination, PhysX, set_global_config,
)


def record_sessions(stage, read_ordinal, destinations):
    # This example owns the only PhysX instance in the process.
    set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)
    physx = PhysX()
    attached = False
    try:
        physx.wait_task(physx.attach_ovstage(stage, read_ordinal=read_ordinal))
        attached = True
        for destination in destinations:
            physx.start_recording(destination)
            try:
                for _ in range(120):
                    physx.step_sync(1.0 / 60.0)
            finally:
                physx.stop_recording()
    finally:
        try:
            if attached:
                physx.wait_task(physx.detach_ovstage())
        finally:
            physx.destroy()


# Explicit FILE start creates missing parents or the selected directory.
output_dir = Path("recordings")
file_destination = OmniPvdDestination.file(str(output_dir / "capture.ovd"))
directory_destination = OmniPvdDestination.file(str(output_dir / "factory"))
tcp_destination = OmniPvdDestination.tcp(
    "127.0.0.1", 5425, timeout_ms=3000,
)

# With your populated and sealed stage:
# record_sessions(stage, 1, [file_destination])   # FILE only
# record_sessions(stage, 1, [directory_destination])  # Timestamped FILE
# record_sessions(stage, 1, [tcp_destination])    # TCP only
# record_sessions(stage, 1, [file_destination, tcp_destination])
# record_sessions(stage, 1, [directory_destination, directory_destination])
```

The FILE/TCP call stops FILE recording and starts a fresh TCP session on the
same attachment. Reusing a directory destination creates a separate timestamped
file for each session and preserves earlier captures. Each start snapshots the
current world. Start the TCP receiver before running a TCP example.

## C: FILE and TCP

This C11 helper likewise borrows an application-populated, sealed ovstage.
The caller keeps it alive until detach or instance destruction completes.
Missing FILE parent directories are created at explicit start. Pass
`include_tcp = false` for FILE only, or `true` to stop FILE and then start a
TCP session. Supply a
nonnull `out_cleanup_instance` so incomplete destruction preserves the handle
for the caller.

```c
#include <stdbool.h>
#include <stdio.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"

static bool checked(ovphysx_result_t result, const char* operation)
{
    if (result.status == OVPHYSX_API_SUCCESS)
        return true;
    /* Read the error before another API call on this thread. */
    const ovphysx_string_t error = ovphysx_get_last_error();
    fprintf(stderr, "%s: %.*s\n", operation, (int)error.length,
            error.ptr ? error.ptr : "");
    return false;
}

static bool checked_task(ovphysx_instance_handle_t instance,
                         ovphysx_enqueue_result_t task, const char* operation)
{
    ovphysx_result_t status = { task.status };
    ovphysx_wait_task_result_t result = { 0 };
    if (task.status == OVPHYSX_API_SUCCESS)
        status = ovphysx_wait_task(instance, task, OVPHYSX_TIMEOUT_INFINITE, &result);
    for (size_t index = 0; index < result.num_errors; ++index)
    {
        const ovphysx_string_t error = ovphysx_get_last_task_error(instance, result.error_tasks[index]);
        fprintf(stderr, "%s: %.*s\n", operation, (int)error.length,
                error.ptr ? error.ptr : "");
    }
    const bool success = result.num_errors ? false : checked(status, operation);
    ovphysx_destroy_wait_task_result(&result);
    return success;
}

int record_sessions(ovstage_instance_t* stage,
                    ovstage_ordinal_t read_ordinal, bool include_tcp,
                    ovphysx_instance_handle_t* out_cleanup_instance)
{
    ovphysx_instance_handle_t instance = 0;
    bool attached = false, recording = false;
    int failed = 0;
    ovphysx_omnipvd_destination_t destinations[2] = {0};

    if (!out_cleanup_instance)
        return 1;
    *out_cleanup_instance = 0;

    destinations[0].transport = OVPHYSX_OMNIPVD_TRANSPORT_FILE;
    /* Use "recordings/" instead for a new timestamped file each start. */
    destinations[0].file_path = OVPHYSX_LITERAL("recordings/capture.ovd");
    destinations[1].transport = OVPHYSX_OMNIPVD_TRANSPORT_TCP;
    destinations[1].tcp_address = OVPHYSX_LITERAL("127.0.0.1");
    destinations[1].tcp_port = 5425;
    destinations[1].tcp_timeout_ms = 3000;

    /* This example owns process initialization and its only instance. */
    if (!checked(ovphysx_initialize(), "initialize"))
        return 1;
#define TRY(call) do { if (!checked((call), #call)) { \
    failed = 1; goto cleanup; } } while (0)
    TRY(ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE,
        ovphysx_config_value_bool(true)));
    TRY(ovphysx_create_instance(&instance));
    if (!checked_task(instance, ovphysx_attach_ovstage(instance, stage, read_ordinal), "attach"))
    {
        failed = 1;
        goto cleanup;
    }
    attached = true;

    for (unsigned session = 0; session < (include_tcp ? 2u : 1u); ++session)
    {
        TRY(ovphysx_ovd_start_recording(instance, &destinations[session], NULL));
        recording = true;
        for (unsigned frame = 0; frame < 120; ++frame)
            TRY(ovphysx_step_sync(instance, 1.0f / 60.0f));
        TRY(ovphysx_ovd_stop_recording(instance));
        recording = false;
    }

cleanup:
    if (recording)
        failed |= !checked(ovphysx_ovd_stop_recording(instance), "stop");
    if (attached)
        failed |= !checked_task(instance, ovphysx_detach_ovstage(instance), "detach");
    if (instance)
    {
        if (checked(ovphysx_destroy_instance(instance), "destroy"))
            instance = 0;
        else
        {
            failed = 1;
            *out_cleanup_instance = instance;
        }
    }
    /* Do not shut down while destruction remains unresolved. */
    if (!instance)
        failed |= !checked(ovphysx_shutdown(), "shutdown");
#undef TRY
    return failed;
}
```

For TCP only, use the TCP descriptor as the first destination and run one
session. Descriptor fields for the other transport must remain empty or zero.
Check every result. A nonzero `out_cleanup_instance` transfers unresolved
instance cleanup back to the caller; keep the stage alive. Destruction returning
`OVPHYSX_API_INVALID_STATE` leaves ownership live: finish pending transitions
or close companion leases, then retry `ovphysx_destroy_instance()` with that
handle and shut down after success. For other destruction errors, consult the
diagnostics rather than assuming teardown completed. If shutdown itself failed
after successful destruction, complete shutdown before changing capability.

## Destinations, ownership and retry

FILE uses the existing `file_path` field and Python `OmniPvdDestination.file()`.
Exact filenames and directory paths both use UTF-8, including on Windows.
There is no new directory transport or factory. A nonempty path ending in the
literal, case-sensitive `.ovd` suffix is an exact filename; every other path
selects a directory. The same rule applies whether the path already exists.

| FILE path | Behavior at explicit start |
|---|---|
| `recordings/factory/capture.ovd` | Create missing parents recursively and open this exact file |
| `recordings/factory` | Create the directory recursively and open a new timestamped file inside it |
| `recordings/capture.OVD` or `recordings/capture.ovd.backup` | Directory, because the lowercase `.ovd` suffix does not match |
| `recordings/capture.ovd/` | Directory, because the trailing separator follows the suffix |
| An existing directory named `recordings/capture.ovd` | Exact-file open fails; add a trailing separator to select it as a directory |

Directory captures use local-time names `YYYY_MM_DD_HH_MM_SS_CC_rec.ovd`.
`CC` starts at `01`, has at least two digits, and advances past existing
names. Each final filename is reserved exclusively at open, so a restart or
another recorder cannot overwrite an earlier directory capture. Recording
writes directly to that filename; stop closes it without temporary-file
renaming or import-directory publication.

Exact filenames retain replacement behavior: restarting the same exact path
can overwrite it. Use a directory destination or a new exact filename for
captures you want to preserve. Restart begins a new recording, not an append
operation. Filesystem preparation occurs during admitted explicit FILE start;
capability, creation and attachment do not create output directories. Failed
filesystem preparation leaves recording inactive and permits retry, though
directories already created by that attempt may remain.

Successful FILE start logs the actual UTF-8 filename at INFO level on channel
`omni.physx`. Call `ovphysx_set_log_level(OVPHYSX_LOG_INFO)` (Python:
`ovphysx.set_log_level(ovphysx.LogLevel.INFO)`). Observe the record through a
registered callback or `enable_python_logging()` with a minimum severity that
admits INFO to identify which timestamped capture belongs to the session.
Open failures log the attempted filename and operating-system open error. If
the initial snapshot fails after reserving a directory file, the runtime closes it
and attempts to remove that incomplete file, logging removal errors. An exact
filename is retained on failure because it may have replaced caller data.

TCP connects synchronously from the simulation process to a ready listener.
Use a nonempty address, a port in `1..65535`, and a nonnegative timeout;
`timeout_ms=0` selects the transport default. TCP recording creates no FILE
output. Stop closes the connection; each restart opens a fresh stream, so the
receiver must be ready to accept it. TCP uses trusted plaintext.

Only one recording can be active. The instance that starts it owns stop and
status; peers cannot stop it and report inactive.

- Start and stop wait for queued and running work on every instance. They do
  not collect task results.
- The status query does not wait.
- During recording, attach and detach also wait for instance work and finish
  before returning.
- Detaching the recording owner, or the last live attachment, tries to finish
  the recording. Call stop first if recording errors must be returned directly.
- Detaching a non-owner keeps the recording alive while another attachment
  remains. Reattach never starts a recording.
- Pending companion preview results reject detach. Fetch or close the preview
  and retry.

Serialize calls on each handle. Overlapping recording and lifecycle transitions
can return `OVPHYSX_API_INVALID_STATE`; retry after the other transition ends.

Per-instance ordering does not provide global exclusion for native calls.
On an idle instance, cached-query validation, recording-status queries,
completed-receipt collection and already-absent query/read/write releases may
progress during a peer's lifecycle callback. A bulk-teardown object-change
callback must not wait for another thread's ovphysx call, directly or through
an application lock or event that thread holds.

With no active recording, invalid destinations or failed opens leave recording
inactive and can be retried once the cause is fixed, for example after
correcting the path or starting the TCP listener. Starting without capability
or without a live physics stage in the shared runtime fails. See
[OmniPVD Recording](developer_guide.md#omnipvd-recording) for the invalid-state
conditions and synchronization contract.

## Change capability after teardown

Stopping output does not remove the provider or disable its costs. The global
setter rejects writes after the first successful create, even if every instance
has since been destroyed, until `ovphysx_shutdown()` succeeds with no live
instances. For C, stop, detach, destroy all instances, and successfully shut
down; then initialize again and set capability before creating a new instance.

Python releases its lifecycle owner on `PhysX.destroy()`. Releasing the last
Python owner performs that shutdown when no native instances remain and
reopens the setter. There is no separate public Python shutdown call. Release
other runtime owners and borrowed output views before teardown; if native
instances coexist, their owner must destroy them and complete native shutdown.
After successful teardown, explicitly set
`GlobalConfigKey.OVD_RECORDING_CAPABLE` to `False` before the next `PhysX()`
to restore the default provider-free runtime.

Configured capability survives shutdown, and the global getter reports that
configured bool; it does not report whether output is active. Use
`ovphysx_ovd_is_recording()` or `PhysX.is_recording()` for session status.
Failed creates do not freeze the setter. Profiler and cache bootstrap settings
likewise use the scoped global setter before create. `NUM_THREADS` and
`OVSTAGE_READ_POOL_MAX_MB` use the scoped setter too; Carbonite tasking/TBB
pool-size settings stay on the legacy create-time path.
