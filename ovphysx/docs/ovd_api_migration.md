<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Instance and OVD API migration

0.7 uses `ovphysx_instance_handle_t` for an instance and
`ovphysx_ovd_start_recording`, `ovphysx_ovd_stop_recording`, and
`ovphysx_ovd_is_recording` for recording. The handle remains `uint64_t`; zero
is invalid. Ownership, shared-runtime semantics, destination layout, and
explicit recording behavior are unchanged. Python method names stay unchanged.
Other 0.6 to 0.7 source breaks are in the [migration guide](migrating_0_6_to_0_7.md).

## Compatibility boundary

Old C recording names remain deprecated forwarding exports in 0.7, and
`ovphysx_handle_t` remains a deprecated typedef of `ovphysx_instance_handle_t`.
They were kept in this release. New code uses the names above. Existing
binaries keep their recording symbols. Retired configuration slots are reserved.
Current Python bindings require the matching native library with the canonical
exports.

Standalone ovphysx no longer supports startup recording configuration or
`PhysXConfig.omnipvd_recording_capable`. The legacy OVD config header and Python
module from 0.7 development builds are removed. Old typed numeric keys and raw Carbonite OVD startup keys
return `OVPHYSX_API_INVALID_ARGUMENT` with guidance to use the global capability
key and explicit recording calls. Their numeric slots are never reused.
Unrelated legacy configuration and recording-function forwarding aliases remain.
Kit applications that start recording through `/physics/omniPvdOutputEnabled`
are unaffected; ovphysx creation and attachment never start recording.

## Configuration mapping

| Old key suffix | Old kind/value | 0.6 Python field / C builder suffix | Current destination |
|---|---|---|---|
| `OMNIPVD_OUTPUT_ENABLED` | bool / 3 | `omnipvd_output_enabled` | Explicit start/stop |
| `OMNIPVD_OVD_RECORDING_DIRECTORY` | string / 0 | `omnipvd_ovd_recording_directory` | Exact `destination.file_path` |
| `OMNIPVD_TRANSPORT` | string / 2 | `omnipvd_transport` | `destination.transport` |
| `OMNIPVD_TCP_ADDRESS` | string / 3 | `omnipvd_tcp_address` | `destination.tcp_address` |
| `OMNIPVD_TCP_PORT` | int32 / 2 | `omnipvd_tcp_port` | `destination.tcp_port` |
| `OMNIPVD_TCP_TIMEOUT_MS` | int32 / 3 | `omnipvd_tcp_timeout_ms` | `destination.tcp_timeout_ms` |
| `OMNIPVD_RECORDING_CAPABLE` | bool / 5 in 0.6.x (4 in 0.7 development builds) | `omnipvd_recording_capable` | `OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE` / `GlobalConfigKey.OVD_RECORDING_CAPABLE` |

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

The old directory key wrote a timestamped `*_rec.ovd` inside that directory.
`file_path` names the exact output file, and its parent directory must already
exist; FILE recording does not create directories.

0.6 numeric values are not ABI-compatible with 0.7; rebuild against 0.7 headers.
Within 0.7, retired slots remain reserved and other values and COUNT sentinels
are not renumbered. Reserved slots must not be assigned new meanings.
Destination and transport types retain their `omnipvd` spellings in this
migration; no new transport or independent runtime is introduced.

## Capture from the first simulation frame

Enable capability before creating the first shared runtime, attach and
initialize the stage, start recording with the desired exact destination,
then step. Start snapshots the current world, so no preliminary simulation
step is needed. Stop before releasing the stage. The complete
[Python and C++ samples](tutorials/omnipvd_recording.md#code) implement this sequence.
Creation, attachment, and reattachment never start standalone recording.

Capability cannot be added to a runtime constructed without its provider.
It remains process-global and false by default. New code uses the BOOL key
`OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE` (`0x0000000c`), or Python
`GlobalConfigKey.OVD_RECORDING_CAPABLE`, with the scoped global setter before
plain creation. Capability alone does not start output. The scoped setter
rejects writes after the first successful create until successful shutdown with
no live instances, even when no instances remain. Python releases its lifecycle
owner on `PhysX.destroy()`; releasing the last Python owner performs that shutdown,
provided no native instances remain, and reopens the setter. Failed creates do not freeze
the setter; the configured value is retained across shutdown. The getter reports
that configured bool. Profiler, cache, and other
scoped settings likewise use `ovphysx_set_global_config` / `set_global_config`
before create.
`NUM_THREADS` and `OVSTAGE_READ_POOL_MAX_MB` are covered there; the Carbonite
tasking/TBB pool-size settings stay on the legacy create-time path.
