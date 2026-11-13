<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-COMPILED-CACHE-001
title: Compiled ovstage Cache Mode Configuration
status: implemented
owner: ovphysx
---

## Description

The runtime can reuse a compiled whole-stage ovstage attach, and later subtree
insertions, from the process's UJITSO datastore: the store cooked colliders use.
Stored results are keyed by the selected physics inputs and the runtime build.
ovphysx exposes this experimental cache as one process-global key and passes the
resolved mode to every `ovphysx_attach_ovstage()` through
`PhysxAttachOptions::compiledStageCache`.

This requirement owns the configuration SURFACE and the attach-time mode
resolution. What the cache stores, its identity, hit/miss/bypass rules,
datastore checks and failure handling are the runtime's contract, owned by
[REQ-CACHE-ATTACH-001](../../../ovruntime/plc/requirements/lifecycle/REQ-CACHE-ATTACH-001.md)
and
[REQ-CACHE-UPDATE-001](../../../ovruntime/plc/requirements/lifecycle/REQ-CACHE-UPDATE-001.md).

AUTOMATIC writes only into an application-configured cooked-collider directory,
only while UJITSO collision cooking is on (an application that turned cooking
off opted out of that datastore), only into the directory the running datastore
was built from (it keeps that directory until it stops, whatever a later
instance configures), and on POSIX only while that directory is private to the
user: a hit deserializes PhysX binary data from it in place, which a checksum
guards against corruption but nothing authenticates. Without a configured directory, ovphysx
points the datastore at a process-private directory that is deleted at shutdown,
so storing there would make every run pay the store cost and reuse results only
within that process.

Trust boundary: PhysX deserializes stored results in place. Their checksum
detects corruption only and is not authenticated, so anyone who can write the
cooked-collider directory can corrupt the attaching process, as with cooked
colliders. User documentation says to keep that directory private to the user.

## Acceptance Criteria

- AC-1: **Key and values.** `OVPHYSX_GLOBAL_CONFIG_COMPILED_STAGE_CACHE`
  (`0x0000000e`) takes an INT64 `ovphysx_compiled_stage_cache_mode_t`:
  `AUTOMATIC` (0), `DISABLED` (1), `READ_ONLY` (2), `READ_WRITE` (3). Python
  mirrors them as `GlobalConfigKey.COMPILED_STAGE_CACHE` and
  `CompiledStageCacheMode`. A value outside 0..3, including one outside int32,
  and a non-INT64 tag return `OVPHYSX_API_INVALID_ARGUMENT` (Python
  `ValueError`) and leave the configured value unchanged.
- AC-2: **Default and observation.** Unset, `ovphysx_get_global_config()`
  reports `AUTOMATIC`. The getter reports the configured value, `AUTOMATIC`
  included, never the mode an attach resolved. The value persists across
  instance destruction and shutdown like other global keys.
- AC-3: **Runtime mutable, read per attach.** Writes succeed before the first
  instance, with live instances and while a stage is attached. Each
  `ovphysx_attach_ovstage()` call reads the value current when it is made, also
  when its task runs later; an attachment keeps the
  mode it attached with, also for its later insertions. With no stored result to
  restore, every mode attaches a loadable stage.
- AC-4: **Mode resolution.** `DISABLED`, `READ_ONLY` and `READ_WRITE` pass
  `PhysxCompiledStageCacheMode::eDisabled`, `eReadOnly` and `eReadWrite`.
  `AUTOMATIC` passes `eReadWrite` only when `/UJITSO/datastore/allowLocalDataStore`
  and `/physics/cooking/ujitsoCollisionCooking` are true,
  `/UJITSO/datastore/localCachePath` is non-empty and this framework
  lifecycle's datastore does not use the process-private fallback directory;
  when that setting still names the directory the datastore was built from,
  which startup records, with its device and inode on POSIX, right after
  `carb.ujitso.default` starts from the settings ovphysx configured (nothing is
  recorded when a host started it before, and shutdown forgets the record);
  and, on POSIX, when that directory is private: `lstat` of the path without
  trailing separators finds the recorded directory, not a symlink, owned by the
  effective user and with no group or other permission bits, and each directory
  from its resolved parent up to `/` is owned by the effective user or root and
  is either writable by neither group nor others or sticky. Otherwise it
  passes `eDisabled`; when a configured directory fails the record or the
  privacy check it logs the directory and the reason at INFO. The check takes no
  loader lock. Startup creates each missing directory of a configured path with
  owner-only permissions (0700), whatever the umask, and changes no existing one. With
  collision cooking off, startup sets up no directory and leaves
  `/UJITSO/datastore/localCachePath` as configured; without a configured one it
  also turns `/UJITSO/datastore/allowLocalDataStore` off, so an explicit mode
  reports `unavailable` instead of writing under Carbonite's application
  directory. When a host application owns the Carbonite framework, its datastore
  settings serve every UJITSO client in the process and stay as the host set
  them: ovphysx instead attaches with the cache disabled and reports
  `unavailable` for an explicit mode.
- AC-5: **Outcome report.** Every attach passes a result slot. Each attach the
  runtime does not refuse as busy logs one `OVPHYSX_LOG_INFO` line
  `compiled stage cache: <result> (mode <resolved>[, automatic])`, where
  `<result>` names the `PhysxCompiledStageCacheResult` (`disabled`, `bypassed`,
  `miss`, `stored`, `hit`, `restore failed`, `store failed`, `unavailable`).
  A disabled mode reports `disabled`. A whole stage the cache bypasses for an
  unsupported prim type or schema names it in an INFO line of its own. Only the cache options change in the
  attach options ovphysx passes; every other field keeps its default.
- AC-6: **Persistent reuse.** With `AUTOMATIC` and a configured cooked-collider
  directory, the first attach of a supported stage reports `stored`, and an
  attach of the same stage in a later process with the same directory reports
  `hit`.

## Known limitations

- Companion `PreviewSession` attachments do not read this key; they attach with
  the cache disabled.

## Test References

- [TEST-CAPI-COMPILED-CACHE-001](../../tests/capi/TEST-CAPI-COMPILED-CACHE-001.md)

## Code References

- ovphysx/include/ovphysx/ovphysx_types.h (`OVPHYSX_GLOBAL_CONFIG_COMPILED_STAGE_CACHE`, `ovphysx_compiled_stage_cache_mode_t`) - AC-1, AC-2
- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_get_global_config`, `ovphysx_attach_ovstage`) - AC-2, AC-3, AC-5
- ovphysx/src/ovphysx/ovphysx.cpp (validation, default, `resolveCompiledStageCacheMode`, `ovphysx_attach_ovstage`) - AC-1 to AC-5
- ovphysx/src/CarboniteLoader/CarboniteLoader.cpp (`usesConfiguredCookedColliderCache`, `configureUjitsoLocalCache`) - AC-4
- ovphysx/src/include/internal/CarboniteLoaderState.hpp (`processCacheDirInUse`, `datastoreDirectory`) - AC-4
- ovphysx/python/ovphysx/types.py (`GlobalConfigKey.COMPILED_STAGE_CACHE`, `CompiledStageCacheMode`) - AC-1
- ovphysx/python/ovphysx/api.py (`get_global_config`) - AC-2

## Dependencies

- [REQ-CAPI-CONFIG-001](REQ-CAPI-CONFIG-001.md) - scoped configuration surface.
- [REQ-CAPI-CACHE-001](REQ-CAPI-CACHE-001.md) - process-private cooked-collider
  directory and its teardown.
- REQ-CACHE-ATTACH-001 and REQ-CACHE-UPDATE-001 (ovruntime) - cache behavior
  behind `PhysxAttachOptions::compiledStageCache`.
- [ADR-0057](../../../ovruntime/plc/adr/ADR-0057-compiled-attachments-and-physics-input-identities.md)
