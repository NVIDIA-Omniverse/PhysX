<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PACKAGING-TESTDEPS-001
title: Python Test Harness Resolves ovstage From The Locked Staged Wheel Directory
status: implemented
owner: ovphysx
---

## Description

Three entrypoints run the Python suites under `uv`: the CMake drivers
`scripts/test_python_runtime.cmake` and `scripts/test_benchmarks_python.cmake`, and
`scripts/run_uv_pytest.py`, the developer wrapper behind
`tests/python_tests/run_pytest.sh` / `.bat`. The two CMake drivers invoke `uv` with
`UV_NO_CONFIG=1`, so no `uv.toml` or `[tool.uv]` configuration *file* on the machine can
change what a CI or `validate_all` run resolves. That is narrower than hermeticity: the
drivers pass their variables through `cmake -E env`, which adds to the caller's
environment rather than replacing it, so inherited `UV_*` variables are still in effect.
What `UV_NO_CONFIG=1` suppresses is `uv`'s *configuration*
and not the project metadata sharing the same `[tool.uv]` table: measured on 0.12.8,
`find-links` and `index-url` are discarded while `required-environments` and
`[tool.uv.sources]` stay in effect, which is the line `uv`'s own settings reference
draws between its two halves. So the driver has to pass the staged wheel directory
itself, while the required platforms a test project declares still bind whatever
resolution it performs there.
The developer wrapper deliberately leaves a developer's `uv` configuration in effect,
but it passes the same `UV_FIND_LINKS` and adds `--locked`, so how that directory is
spelled matters there too.

`scripts/fetch_deps.cmake` stages exactly one ovstage wheel — the host platform's —
while `uv.lock` records wheel filenames for all three supported platforms, so
resolution succeeds only if `uv` **accepts the lock** rather than re-resolving it.
What decides that, as measured on `uv` 0.12.8, is whether the find-links directory is
given as a *relative* path that resolves to the directory the lock records in its
`source = { registry = ... }` — `../../_build/target-deps/ovstage_wheel` from
`tests/python_tests`. Every relative spelling that resolves there is accepted; an
**absolute** path to that very same directory is not, and the lock is discarded and
re-resolved. A relative path to a different existing directory is likewise rejected,
while passing no find-links at all is accepted. Those are the observed outcomes, and
this requirement rests on them rather than on any mechanism inside `uv`; passing the
relative spelling the lock itself records satisfies them and every stricter reading of
them. The lock's form is part of the same property: the CMake drivers do not pass
`--locked`, so a re-resolution that *did* find a solution would rewrite `uv.lock` in
place, trading the relative registry for a PyPI URL under the same pin.

The version in that lock is the other half. One ovstage release is pinned, in
`OVSTAGE_VERSION` in `scripts/fetch_ovstage_release.py`, and `python/pyproject.toml`
plus every checked-in lock whose resolved graph records ovstage repeat it — three of
the seven locks under `ovphysx/` today, the four under `tests/python_samples*/` not
among them. Only the pinned release's wheel is staged, so a lock naming a superseded
release has no candidate at all. That set is not a fixed list: a project depending on
ovphysx acquires an ovstage entry the moment anyone runs `uv lock` in it, so it has to
be derived from the tree rather than kept as a hand-maintained list. Repeating the
version as text is faithful only while ovstage's `[[package]]` block records nothing
but its name, version, source and wheel paths, as it does today: rewriting those
strings cannot refresh a recorded dependency graph, so an ovstage release that gained
dependencies of its own would need these locks regenerated rather than stamped.

This requirement constrains how the harness resolves its dependencies; no ovphysx
binary, header, or Python symbol changes with it.

## Acceptance Criteria

- AC-1: **The CMake drivers resolve independently of `uv` configuration files.** Every
  resolving `uv` invocation (`uv run`) `scripts/test_python_runtime.cmake` and
  `scripts/test_benchmarks_python.cmake` make runs with `UV_NO_CONFIG=1`, so no user- or
  system-level `uv` configuration *file* can change what a CI or `validate_all` run
  resolves. What that discards and the run still needs — `pyproject.toml`'s
  `find-links` — is supplied explicitly in the environment the driver builds. The
  criterion claims nothing stronger, and in particular not hermeticity: the drivers build
  that environment with `cmake -E env`, which adds to the caller's environment rather
  than replacing it, so inherited resolution-affecting variables survive — `UV_INDEX` and
  `PIP_INDEX_URL` on the CI runners, as the TEST records, and equally `UV_NO_SOURCES`,
  `UV_RESOLUTION` or `UV_EXCLUDE_NEWER` — and the drivers clear none of them. What such a
  variable would do to these drivers was not measured; neither this criterion nor AC-3 is
  written against it. `scripts/run_uv_pytest.py` is deliberately outside this criterion:
  it is the developer wrapper, and a developer's `uv` configuration stays in effect
  there. AC-2 and AC-3 still bind it, so it does not *depend* on that configuration to
  resolve.

- AC-2: **The find-links location is a relative path resolving to the directory the
  lock records as its registry.** Every entrypoint that passes `UV_FIND_LINKS` — both
  CMake drivers and `scripts/run_uv_pytest.py` — names the build-staged ovstage wheel
  directory as a path relative to the directory `uv` runs in, and sets that working
  directory to the project directory the path was computed against. Measured on `uv`
  0.12.8, a relative find-links resolving to the directory the lock records in
  `source = { registry = ... }` is accepted while an absolute path to the same
  directory is not; what each entrypoint passes is the lock's own string
  (`../../_build/target-deps/ovstage_wheel` from `tests/python_tests`), which meets
  that condition and any stricter reading of it. The path is spelled with forward
  slashes on every platform, as `file(RELATIVE_PATH)` and the lock both do.

- AC-3: **Every checked-in lock is accepted, never re-resolved.** Under the environment
  of AC-1 and AC-2, `uv lock --check` reports all three checked-in locks
  (`python/uv.lock`, `tests/python_tests/uv.lock`, `tests/python_benchmarks/uv.lock`)
  up to date. For `python/` and `tests/python_tests/` a re-resolution fails outright:
  only the host wheel is staged and `[tool.uv] required-environments` demands three
  platforms. `tests/python_benchmarks/` declares no required environments, so a
  re-resolution there succeeds and narrows the ovstage entry to the single staged wheel;
  acceptance of the lock as written is what keeps its other two platform entries alive.
  `scripts/run_uv_pytest.py` passes `--locked`, so a developer run whose lock is not
  accepted fails loudly instead of rewriting it.

- AC-4: **Resolution does not depend on the pinned `uv` version, nor on which lock
  happens to be checked in.** AC-3 holds both on the `uv` version CI pins (its
  `UV_VERSION` variable) and on later releases. A run in which the pinned version
  accepts an absolute find-links path does not satisfy it, because that acceptance
  turns on the lock in the tree rather than on the version: the pinned 0.9.7 discarded
  the lock under an absolute find-links throughout August 2026 and accepts it against
  the lock checked in today. See the TEST for what about the lock changed — it is not
  established.

- AC-5: **Every checked-in lock keeps its relative local-wheel source.** Each of
  `python/uv.lock`, `tests/python_tests/uv.lock` and `tests/python_benchmarks/uv.lock`
  records ovstage as `source = { registry = ... }` naming the staged wheel directory
  relative to its own project directory, with local `{ path = ... }` wheel entries and
  no PyPI URL. The expected string is therefore not one constant: it is
  `../../_build/target-deps/ovstage_wheel` for the two locks under `tests/` and
  `../_build/target-deps/ovstage_wheel` for `python/`, and the check derives it per
  lock from that lock's own location, as `file(RELATIVE_PATH)` does in the CMake
  drivers. `scripts/validate_python_test_uv_lock.py`, run by
  `scripts/ci_validate.cmake`, fails the validation when any present lock drifts. A
  lock the tree does not contain is skipped rather than reported missing, because the
  public source drop ships two of the three -- `python/uv.lock` and
  `tests/python_tests/uv.lock`, the locks its documented entry points run against,
  which the packaging job requires in the generated tree -- and not
  `tests/python_benchmarks/uv.lock`; a tree containing none of them fails, so skipping
  cannot turn into a vacuous pass.

- AC-6: **One bump reaches every copy of the pin, and no hand-written list is the last
  word on which copies those are.** The pin has one source of truth, `OVSTAGE_VERSION` in
  `scripts/fetch_ovstage_release.py`. `python/pyproject.toml` and **every checked-in
  `uv.lock` under `ovphysx/` whose resolved graph records ovstage** carry that same
  version — three at the time of writing, but the criterion is the property, not the
  count. The set of locks is derived from the tree and compared against the list the
  tooling declares, so a lock that starts recording ovstage fails the build instead of
  being skipped silently. That comparison is a checked-in test CI runs on every build,
  **not** something the bump command performs: a newly added lock is caught at the next
  build rather than picked up by the bump that preceded it. What the bump command itself
  guarantees is all-or-nothing at the level of the individual version occurrence — every
  declared target must exist, every version site in it must match, and no line naming
  ovstage in the rewritten text may still carry a different version, before any file is
  written. So a lock format change that leaves one occurrence stale while the others
  rewrite aborts instead of half-stamping, as long as the stale occurrence sits on a line
  naming ovstage. No file the build resolves ovstage from names a
  version other than the pinned one, and no lock silently lists fewer than the three supported
  platforms' wheels. Agreement between `python/pyproject.toml` and `python/uv.lock` is
  established by reading both as text, not by asking a resolver: `scripts/ci_validate.cmake`
  runs before `fetch_deps`, so `scripts/test_pyright.cmake` — the sole consumer of
  `python/uv.lock` — must resolve from the lock as written (`uv sync --frozen`). `--locked`
  cannot serve as the consumer-side check: its freshness check invalidates the lock at that
  stage and re-resolves, reaching the package index and failing on ovstage even when lock
  and `pyproject.toml` agree. That constraint binds wherever the lock is present. The
  public source drop carries `python/uv.lock`, so the constraint binds there too; the
  step's skip is a fallback for a tree with no lock at all, and a lock that is present
  but malformed still fails, so the skip cannot mask drift.

## Test References

- TEST-PACKAGING-TESTDEPS-001 — partly automated. AC-5 is automated by
  `scripts/validate_python_test_uv_lock.py`, run from `scripts/ci_validate.cmake`.
  AC-1 through AC-3 are exercised end to end by the `python-runtime` CTest entry, which
  cannot reach a test case when resolution fails, and which CI runs on all three
  platforms — but only against the `uv` version CI pins. The consumer-side half of AC-6
  is a constraint on `scripts/test_pyright.cmake` (`--frozen`, no re-resolution) rather
  than a check it performs. AC-4, and AC-3 for the two locks no CTest entry exercises,
  are verified by hand; see that TEST for the commands and what was not executed.

## Code References

- ovphysx/scripts/test_python_runtime.cmake (`UV_ENV`, the `file(RELATIVE_PATH)`
  spelling of `UV_FIND_LINKS`, `WORKING_DIRECTORY` — AC-1, AC-2, AC-3)
- ovphysx/scripts/test_benchmarks_python.cmake (the same construction for
  `BASE_UV_ENV` — AC-1, AC-2, AC-3)
- ovphysx/scripts/run_uv_pytest.py (`os.path.relpath` spelling of `UV_FIND_LINKS`
  against the directory `uv` runs in, `--locked` — AC-2, AC-3)
- ovphysx/scripts/validate_python_test_uv_lock.py (`LOCK_PATHS`,
  `expected_registry()`, `validate_all_locks()`'s skip-if-absent — AC-5)
- ovphysx/scripts/fetch_ovstage_release.py (`OVSTAGE_VERSION` — the pin AC-6 stamps from)
- ovphysx/scripts/test_pyright.cmake (`uv sync --frozen`, and the fallback skip when
  `python/uv.lock` is absent — AC-6's consumer side)
- ovphysx/scripts/ci_validate.cmake (runs the AC-5 validator and `test_pyright.cmake`)
- ovphysx/scripts/fetch_deps.cmake (stages the host platform's wheel only — AC-3's premise)
- ovphysx/python/pyproject.toml (AC-3, AC-6)
- ovphysx/tests/python_tests/pyproject.toml (`find-links`, `required-environments` —
  AC-1, AC-3), ovphysx/tests/python_tests/uv.lock (AC-2, AC-3, AC-5)
- ovphysx/python/uv.lock (AC-3, AC-5, AC-6) — ships in the public drop, so
  `test_pyright.cmake` can type-check the stub tree there
- ovphysx/tests/python_benchmarks/pyproject.toml (AC-2, AC-3)

## Dependencies

- None
