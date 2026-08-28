<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

---
title: ADR-0001 — The cooked-collider cache location is application-provided
status: proposed
date: 2026-07-27
supersedes: none
---

## Status

Proposed — the *record* is new and awaits owner sign-off, though the *policy* it describes has
been in force since ovphysx 0.5 (`3746c9b20f`, !7308). This ADR writes it down so it does not
have to be recovered from commit history, and adds the process-private fallback that NVBug
6504275 forced.

## Context

Collision cooking is CPU-expensive, so ovphysx caches cooked colliders through UJITSO, whose
local datastore needs a directory. Two questions had to be settled: who picks it, and what
happens when nobody does.

The first was settled on !7308. The original implementation read `UJITSO_DATASTORE_CONFIG`
and derived a per-user default from `XDG_CACHE_HOME` / `HOME` / `LOCALAPPDATA`. Review
rejected both:

> "I think this should be read from a config passed to ovphysx, we should not write randomly
> if the user can tell use the location of the cache"
>
> "Please no env variables, this is something that the app should provide, no magic env
> variables" — Ales Borovicka, !7308

`3746c9b20f` removed `computeUjitsoCacheDir()` and the env-var handling accordingly.

The second surfaced as NVBug 6504275. Leaving `/UJITSO/datastore/localCachePath` unset does
**not** mean "no location is chosen" — it means `carb.ujitso.default` chooses one, defaulting
to `<carb app dir>/cache/DerivedDataCache`. Kitless, carb's app dir is the resolved
interpreter directory, so a standard Linux venv resolved to the non-writable
`/usr/bin/cache/DerivedDataCache` and `carb.datastore` logged two `[Error]` lines on every
scene attach. Declining to choose delegates the choice rather than avoiding it, and the
datastore has no memory-only mode to fall back on.

## Decision

The cooked-collider cache location is **application-provided**.

1. The only supported input is `PhysXConfig.cooked_collider_cache_dir` (C:
   `OVPHYSX_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY`), applied to
   `/UJITSO/datastore/localCachePath` before `loadPhysxPlugins()`.
2. ovphysx reads **no environment variable** to select a cache location, and persists to no
   location of its own choosing.
3. When the app configures nothing, ovphysx cooks to a **process-private** directory under
   the OS temp dir and removes it at process exit. Nothing survives the process, so the
   "nothing is persisted" contract holds; the directory exists only because the datastore
   requires a valid writable path. The OS temp root itself follows the platform
   `TMPDIR`/`TMP`/`TEMP` convention — that is temp-file placement, not a persistent cache
   location, and is the one exception to (2).
4. The same process-private fallback is used when a configured directory cannot be created
   or written, so an unusable configuration degrades quietly instead of being handed back to
   the datastore to fail on.
5. The configured directory is applied at **first runtime bootstrap** in a process. The
   datastore is built once, so a later `PhysXConfig` in the same process does not move it.

**Out of scope:** Hub / Nucleus / GRPC datastores (ovphysx is local and in-process only);
cache eviction policy, which is `carb.datastore`'s bounded disk GC; and any persistent
default location (see Open questions).

## Alternatives considered

| Alternative | Why it lost |
|---|---|
| Per-user persistent default (`$XDG_CACHE_HOME`, `%LOCALAPPDATA%`) when unset | Reverses the !7308 decision: ovphysx would choose a persistent location for the app and read the environment to do it. Better default UX, but an owner-level policy change, not a bug fix. |
| Disable the local datastore when unset | Does not remove the error output. `carb.ujitso.default` then logs "Failed to obtain a data store, aborting agent/service setup" and cooking silently runs uncached — the NVBug 6262606 regression. |
| Leave `localCachePath` unset (pre-6504275 behaviour) | Delegates the choice to `carb.ujitso.default`, which picks a non-writable interpreter-relative path. This is the defect. |

## Consequences

**Positive**
- One input, no hidden precedence between config and environment.
- The documented contract and the implementation agree: unset means nothing persisted.
- No ERROR-level output on a default run.

**Costs**
- An app that configures nothing re-cooks every run. Deliberate — persistence is opt-in.
- The fallback directory needs a lifecycle: it is process-wide, so it is removed only at
  process exit, never from a per-instance `CarboniteLoader::shutdown()`.
- Removal is best-effort on Windows, where the datastore may still hold files open; the OS
  reclaims the temp tree instead.

**Lifecycle**
- Revisit if UJITSO gains a memory-only datastore mode, which would remove the need for a
  fallback path entirely.

## Open questions

1. Should ovphysx ship a persistent per-user default after all? It would give new users
   cross-run caching instead of re-cooking every launch, which is what NVBug 6504275
   recommended. Leaning: no by default, and only with owner sign-off, since it reverses the
   decision above — but the UX argument is real.
2. Should a changed `cooked_collider_cache_dir` in an already-bootstrapped process raise
   instead of being silently ignored? Leaning: yes, but it is an API behaviour change.

## References

- !7308 / `3746c9b20f` — original decision, review by Ales Borovicka
- !7860 (release/ovphysx/0.5), !7867 (trunk) — NVBug 6504275 / OMPE-102840
- NVBug 6262606 — why UJITSO is loaded in the kitless loader at all
- `ovphysx/src/CarboniteLoader/CarboniteLoader.cpp` — `configureUjitsoLocalCache()`
- `ovphysx/docs/developer_guide.md` — "Cooked-Collider Cache (UJITSO)"
