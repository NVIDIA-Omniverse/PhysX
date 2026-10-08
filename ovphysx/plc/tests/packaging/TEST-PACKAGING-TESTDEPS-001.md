<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PACKAGING-TESTDEPS-001
maps_to: REQ-PACKAGING-TESTDEPS-001
type: integration
---

## Coverage status

Partly automated.

- **Automated:** AC-5 — `scripts/validate_python_test_uv_lock.py`, run by
  `scripts/ci_validate.cmake`, over every one of the three locks that is present, each
  against the registry string derived from its own depth.
- **Automated indirectly:** AC-1 through AC-3 for the runtime harness — the
  `python-runtime` CTest entry cannot reach a single test case when resolution fails, so
  every green run of it is a passing run of those criteria on the runner's `uv`.
- **Automated:** AC-1 through AC-3 on all three platforms — the `python-runtime` entry
  runs in CI on Linux x86_64, Linux aarch64 and Windows, so AC-2's Windows half is
  executed rather than reasoned.
- **Manual:** AC-4; the direct form of AC-3 for the two locks no CTest entry
  exercises; and AC-2 and AC-3 for `scripts/run_uv_pytest.py`, which no CTest entry
  drives.
- The consumer side of AC-6 is a constraint on `scripts/test_pyright.cmake`
  (`--frozen`), not a check it performs. The step skips itself where `python/uv.lock`
  is absent; the public source drop carries that lock, so it binds there too.
- The bump side of AC-6 is automated by a checked-in test the public source drop does
  not ship.

## Scenario

The Python test harness resolves ovstage from the one wheel the build staged, by
accepting `uv.lock` rather than re-resolving it, and — in the CMake drivers — without
reading a developer's `uv` configuration. The regression this pins is a `UV_FIND_LINKS`
spelled as an absolute path under `UV_NO_CONFIG=1`: `uv` then discards the lock,
re-resolves against the single staged wheel, and has no solution for the `win32` /
`AMD64` entry in `[tool.uv] required-environments`. Only a *relative* find-links that
resolves to the directory the lock records as its registry is accepted; an absolute
path to that same directory is not. The failure is **not** version-gated. Against the
lock checked in today the `uv` version CI pins, 0.9.7, accepts the lock in that same
combination, which is why the harness passes now — but that same 0.9.7 discarded the
lock under the same absolute find-links through August 2026, failing the
`ovphysx-opensource*` jobs on 20 nightly runs. Dormancy is a property of the
current lock rather than of the pinned version, and what changed in the lock to end it
is not established. The developer wrapper `scripts/run_uv_pytest.py` spelled it the
same absolute way; there the project's own `[tool.uv] find-links` masked the defect,
because the wrapper does not set `UV_NO_CONFIG`. The second regression is a pin that
reached only some of the files recording it. Covers REQ-PACKAGING-TESTDEPS-001 AC-1
through AC-6.

## Given

- An `ovphysx/` checkout built far enough that `_build/target-deps/ovstage_wheel` holds
  the host platform's ovstage wheel and `_install/` exists (`./build.sh`, then
  `cmake -P scripts/install.cmake`).
- `tests/python_tests/uv.lock` as checked in: ovstage at
  `source = { registry = "../../_build/target-deps/ovstage_wheel" }` with wheel entries
  for `manylinux_2_35_aarch64`, `manylinux_2_35_x86_64` and `win_amd64` — three
  platforms against the one wheel the build staged.
- Two `uv` versions: the one CI's `UV_VERSION` pins (0.9.7 when this was measured) and a
  later release (0.12.8).
- Every dependent copy of the pin as checked in: `python/pyproject.toml` plus the three
  locks that record ovstage (`python/uv.lock`, `tests/python_tests/uv.lock`,
  `tests/python_benchmarks/uv.lock`) — a set derived from the tree, not a count fixed by
  this document.
- The consumer-side pass needs neither `_build/` nor `_install/`: it reads only the
  checked-in files, so it runs on a clean checkout. So do the find-links spelling
  cases, which use an empty staged wheel directory on purpose.

## When

- **Lock-acceptance pass, per `uv` version.** From `tests/python_tests`, with the
  environment the driver builds (`UV_CACHE_DIR`, `UV_NO_CONFIG=1`, `UV_HTTP_TIMEOUT`,
  `UV_SKIP_WHEEL_FILENAME_CHECK=1`), run `uv lock --check` twice: once with
  `UV_FIND_LINKS` absolute, once with it set to
  `../../_build/target-deps/ovstage_wheel`.
- **Find-links spelling pass.** From `tests/python_tests`, under the same environment
  and against an *empty* staged wheel directory, run `uv lock --check` once per
  spelling of `UV_FIND_LINKS`: the plain relative path; the same path reached by a
  detour (`../python_tests/../../_build/target-deps/ovstage_wheel`); a `./`-prefixed
  and a trailing-slash variant; a relative path arriving from a sibling
  (`../../../ovphysx/_build/target-deps/ovstage_wheel`); a relative path to a
  *different* existing directory (`../../scripts`); no `UV_FIND_LINKS` at all; and an
  absolute path to the same directory.
- **`--no-config` scope pass.** In scratch projects under a build directory, against
  hand-built wheels for names published on no index, run `uv lock` twice per setting —
  once with `UV_NO_CONFIG=1`, once without — for `[tool.uv] find-links`; for
  `[tool.uv] index-url` pointed at an unreachable host; for
  `[tool.uv] required-environments` naming a platform the only wheel has no tag for,
  with find-links supplied through the environment so it survives either way; and for
  `[tool.uv.sources]`. No `uv.lock` exists in any of them at the start.
- **Historical-CI pass.** Read the `ovphysx-opensource*` job logs from the daily
  scheduled trunk pipelines across August and early September 2026 for the `uv` version
  the driver found, the `UV_FIND_LINKS` value it echoed, and whether the step re-resolved
  or installed from the lock.
- **Developer-wrapper pass.** Run `tests/python_tests/run_pytest.sh` with a stub `uv`
  on `PATH` that prints its working directory and `UV_FIND_LINKS`, to read off exactly
  what the wrapper passes and from where. Then feed that value to `uv lock --check`
  from that directory, twice: with `UV_NO_CONFIG=1` and without it, so the project
  config's own `find-links` is first excluded and then included.
- **End-to-end pass.** `cmake -P scripts/validate_all.cmake` with the fix in place, and
  the same drivers under CI on each of the three supported platforms.
- **Lock-form pass.** `python3 scripts/validate_python_test_uv_lock.py`, then the same
  script in a scratch copy of `scripts/` plus the three locks outside the repository:
  once with one lock's ovstage entry rewritten to PyPI URLs at the *same* version, once
  with a lock carrying the other depth's registry spelling, once with only
  `tests/python_tests/uv.lock` present (the public drop's shape), and once with no lock
  at all.
- **Non-runtime lock-acceptance pass.** `uv lock --check` and then a plain `uv lock`,
  under the driver's environment, from `python/`
  (`UV_FIND_LINKS=../_build/target-deps/ovstage_wheel`) and from
  `tests/python_benchmarks/` (`UV_FIND_LINKS=../../_build/target-deps/ovstage_wheel`).
- **Benchmarks re-resolution pass.** In a scratch copy holding only `VERSION`,
  `python/`, `tests/python_benchmarks/` and the one staged ovstage wheel, delete
  `tests/python_benchmarks/uv.lock`, run `uv lock` under the same environment, and
  compare the regenerated ovstage block against the checked-in one.
- **Consumer-side pass.** From `python/`, with `UV_NO_CONFIG=1` and `UV_CACHE_DIR` set
  and **no** `_build/` present (the `ci_validate` ordering), run
  `uv sync --only-group dev --no-install-project --dry-run` with `--frozen` and then
  with `--locked`, first against the restamped lock and then against a lock whose
  `requires-dist` specifier has been reverted to `0.1.1.354763`. Repeat the whole matrix
  on both `uv` versions. Then run the real thing: `cmake -P scripts/ci_validate.cmake`
  in a CI job, before `fetch_deps`, on each of the three supported platforms.

## Then

- With the **absolute** `UV_FIND_LINKS`, and against the lock checked in today,
  `uv lock --check` passes on 0.9.7 and fails on 0.12.8. The 0.12.8 failure is a
  re-resolution that cannot satisfy the Windows required environment, not a
  missing-file error (REQ AC-3, AC-4):

  ```text
  × No solution found when resolving dependencies for split (markers: ...
  │ platform_machine == 'AMD64' and sys_platform == 'win32'):
  ╰─▶ Because ovstage==<the pin at measurement time> has no `platform_machine ==
      'AMD64' and sys_platform == 'win32'`-compatible wheels ...
  ```

- With the **relative** `UV_FIND_LINKS`, `uv lock --check` passes on both versions: the
  lock is accepted, nothing is re-resolved, the single staged wheel suffices, and the
  `uv` pin stops being load-bearing (REQ AC-2, AC-3, AC-4).
- The spelling pass locates the boundary on 0.12.8, and it is relativeness rather than
  string equality with the lock. Every relative spelling that resolves to the recorded
  registry directory is accepted — the plain path, the detour through
  `../python_tests/`, the `./`-prefixed and trailing-slash variants, and the sibling
  route through `../../../ovphysx/` — none of which is byte-equal to the lock's string.
  A relative path to a different existing directory (`../../scripts`) is rejected, and
  so is an absolute path to the very same directory; passing no `UV_FIND_LINKS` at all
  is accepted. The requirement is written against those outcomes and claims no mechanism
  inside `uv`; the fix passes the lock's own string, which satisfies the observed rule
  and any stricter one (REQ AC-2).
- The `--no-config` scope pass settles what `UV_NO_CONFIG=1` costs the driver, on `uv`
  0.12.8. It suppresses `[tool.uv] find-links`: without it the project resolves, with it
  the run ends `Because ovphysx-probe-pure was not found in the package registry`. It
  suppresses `[tool.uv] index-url`: without it the unreachable host is contacted and the
  run fails on `Connection refused`, with it the run resolves from the default
  `https://pypi.org/simple`. It does **not** suppress `[tool.uv] required-environments`:
  the `platform_machine == 'AMD64' and sys_platform == 'win32'` split fails identically
  either way, and with no `uv.lock` present at all, so that marker split comes from
  `pyproject.toml` and not from a lock's `required-markers`. It does not suppress
  `[tool.uv.sources]` either. That is `uv`'s own division: its settings reference splits
  `[tool.uv]` into a *Project metadata* half holding `required-environments` and
  `sources` and a *Configuration* half holding `find-links` and `index-url`, and
  `--no-config` is defined as "Avoid discovering configuration files". So AC-1's
  explicit `UV_FIND_LINKS` is necessary, and AC-3's three required platforms bind the
  driver's resolution regardless of `UV_NO_CONFIG` (REQ AC-1, AC-3).
- The historical-CI pass shows the absolute spelling failing on the pinned `uv`, so the
  defect was live rather than latent. On 3 August the
  `ovphysx-opensource-build-test-linux` job printed `Found uv: uv 0.9.7`, echoed a
  `UV_FIND_LINKS` under `/tmp/oss_repo/.../_build/target-deps/ovstage_wheel` — an
  absolute path — and failed the way the 0.12.8 developer measurement does, at
  `scripts/test_python_runtime.cmake`:

  ```text
  × No solution found when resolving dependencies for split (markers:
  │ platform_machine == 'AMD64' and sys_platform == 'win32'):
  ╰─▶ Because ovstage==0.1.1.354763 has no `platform_machine == 'AMD64' and
      sys_platform == 'win32'`-compatible wheels and ovphysx==0.6.0 depends
      on ovstage==0.1.1.354763, we can conclude that ovphysx==0.6.0 cannot
      be used.
  ```

  The four `ovphysx-opensource*` jobs failed that way on every 02:40 trunk nightly from
  3 to 26 August 2026 that ran them — 23 nightlies over those 24 days, there being no
  02:40 schedule on 5 August, of which the 7, 14 and 19 August ones had all four jobs
  canceled; the other 20 all failed with `uv 0.9.7`, an absolute `UV_FIND_LINKS` and this
  same split at `scripts/test_python_runtime.cmake`. The 26 August nightly is the last
  one failing *that* way: the four jobs stayed red on the 29 and 30 August nightlies, and
  three of the four on 2 September, but with the lock consumed — `417096135` prints
  `Installed 10 packages in 62ms` and `collected 557 items` before its test failures.
  Dormancy today is observed rather than assumed: on 2 September
  `ovphysx-opensource-devphysx-build-test-linux` (`422276761`) ran the same driver, on
  the same pinned 0.9.7, with the same absolute `UV_FIND_LINKS`, and printed
  `Installed 10 packages in 64ms` and no `Resolved` line at its pytest step — the lock
  was consumed, not re-resolved. The plain `ovphysx-opensource-build-test-linux` that day
  (`422276759`) failed in setup and never reached the harness. The pinned version is
  therefore not what makes the absolute spelling survive (REQ AC-4). That August log's
  index hint names internal Artifactory indexes rather than the `https://pypi.org/simple`
  the test project's `[tool.uv] index-url` sets, which the scope pass above explains: that
  setting is configuration and was suppressed, and the runner images export `UV_INDEX` /
  `PIP_INDEX_URL` pointing at those indexes. Environment variables are not configuration
  files, so `--no-config` leaves them alone, and `cmake -E env` adds the driver's
  variables to the caller's environment rather than replacing it. That is why AC-1 is
  scoped to configuration files and claims no hermeticity. That reading was not confirmed
  against an environment dump from the job itself.
- `validate_all.cmake` exits 0 with all 6 CTest entries passing: `cpp-unit`,
  `cpp-samples`, `source-link`, `python-runtime`, `python-wheel`, `python-samples`.
  `python-runtime` reaching its cases at all is the end-to-end statement of AC-1 through
  AC-3 (REQ AC-1, AC-2, AC-3).
- In CI the same driver reaches its cases on all three platforms. Child pipeline
  `65994557` — the one that ran against the then-current head `ff4e9ababb`, as
  merge-result commit `7f751a51` — reports `success` and holds 29 jobs,
  of which **12 ran and all 12 passed**; the other 17 never executed — 8 are `manual` and
  were not played, 9 were `skipped`. The 12 that ran are `ovphysx-linux`,
  `ovphysx-linux-debug`, `ovphysx-linux-aarch64`, `ovphysx-linux-aarch64-debug`,
  `ovphysx-windows`, `ovphysx-windows-vs`, `ovphysx-windows-debug`,
  `ovphysx-linux-x86_64-benchmarks`, `ovphysx-windows-x86_64-benchmarks`,
  `ovphysx-linux-nogpu`, `ovphysx-windows-nogpu` and `validate-oss-source-tree`. That set
  covers all three supported platforms, so the platform question is settled even though
  most of the pipeline stood down — but not every harness job ran: the four
  `ovphysx-opensource*` jobs, the ones the August evidence above shows failing at
  `scripts/test_python_runtime.cmake`, are among the `skipped`. The driver passes
  `UV_FIND_LINKS` through `cmake -E env` on an `execute_process` with
  `COMMAND_ECHO STDOUT`, so the six of those jobs that run the pytest harness — the four
  Linux build jobs and the two Windows release ones, all reporting
  `Found uv: uv 0.9.7` — carry the relative `../../_build/target-deps/ovstage_wheel`
  spelling verbatim; `ovphysx-windows-debug` runs no Python suite and echoes none. That
  settles AC-2's Windows half: `file(RELATIVE_PATH)` emits forward slashes, both inputs share a
  drive because they derive from `CMAKE_CURRENT_LIST_FILE`, and every `uv` invocation
  runs in the directory the path was computed against (REQ AC-2). The AC-5 validator ran
  in the `ci_validate` step of four of them — `ovphysx-linux`, `ovphysx-linux-aarch64`,
  `ovphysx-windows` and `ovphysx-windows-vs` — logging
  `validate_python_test_uv_lock: OK (python/uv.lock, tests/python_tests/uv.lock,
  tests/python_benchmarks/uv.lock)` on all three platforms, so all three locks are read
  in CI rather than only in a developer tree (REQ AC-5).
- The `python-runtime` driver collected the same suite sizes in all six jobs that run it:
  `collected 679 items` in the main suite and `collected 496 items` in CPU mode, none
  failing. The main suite also reports 51 passing subtests, which are outcomes rather
  than collected items, so its result line totals 730 — 679 is the collected count and
  the two must not be conflated. Skips differ by platform: `670 passed, 9 skipped` and
  `493 passed, 3 skipped` on all four Linux jobs, `647 passed, 32 skipped` and
  `492 passed, 4 skipped` on both Windows ones (REQ AC-1, AC-2, AC-3).
- The developer wrapper passes `../../_build/target-deps/ovstage_wheel` from
  `tests/python_tests`, byte-identical to what the CMake driver computes, on its single
  `uv` invocation. Fed to `uv lock --check` from that directory on 0.12.8, that value is
  accepted whether or not `UV_NO_CONFIG=1` is set. The absolute value it passed before
  is accepted only when the project config is read, and fails under `UV_NO_CONFIG=1` —
  which is why the defect was invisible through `run_pytest.sh`: the project's own
  relative `[tool.uv] find-links` was covering for it (REQ AC-2, AC-3).
- `validate_python_test_uv_lock.py` prints
  `validate_python_test_uv_lock: OK (python/uv.lock, tests/python_tests/uv.lock,
  tests/python_benchmarks/uv.lock)` in the internal tree, so all three are read rather
  than one. In the scratch copy it rejects each seeded defect with the offending lock
  named and the registry its own depth implies (REQ AC-5):

  ```text
  tests/python_benchmarks/uv.lock: ovstage must use registry
    '../../_build/target-deps/ovstage_wheel', got 'https://pypi.org/simple'.
  python/uv.lock: ovstage must use registry '../_build/target-deps/ovstage_wheel',
    got '../../_build/target-deps/ovstage_wheel'.
  ```

  The rewritten entry kept the pinned version and all three platform wheels, so it is
  the registry check rather than the version or platform checks that catches it. With
  only `tests/python_tests/uv.lock` present -- a subset of the public source drop,
  which ships that lock and `python/uv.lock` but not `tests/python_benchmarks/uv.lock`
  -- the script prints
  `validate_python_test_uv_lock: OK (tests/python_tests/uv.lock)` and exits 0, while the
  same stripped tree with that lock rewritten to PyPI still exits 1. A tree with no
  lock at all exits 1 with `no checked-in uv.lock recording ovstage found`, so absent
  and wrong stay distinguishable (REQ AC-5).
- `uv lock --check` passes from `python/` and from `tests/python_benchmarks/` with the
  relative spelling, so the stamped locks are accepted rather than re-resolved. A
  subsequent plain `uv lock` leaves both byte-identical: the version is the only thing an
  ovstage bump changes in these files (REQ AC-2, AC-3, AC-6).
- The from-scratch benchmarks resolve **succeeds** rather than failing, because
  `tests/python_benchmarks/pyproject.toml` declares no `[tool.uv] required-environments`:
  `uv` 0.12.8 resolves 47 packages and writes an ovstage block that keeps the relative
  registry but lists only `manylinux_2_35_x86_64`, dropping the
  `manylinux_2_35_aarch64` and `win_amd64` paths the checked-in lock carries. In the
  benchmark project it is therefore acceptance of the lock, not a resolution failure,
  that keeps the other two platforms' entries alive (REQ AC-3). The count is a property
  of the index on the day, not of this repository: the checked-in lock holds 46
  `[[package]]` blocks, and no record was kept of which packages the re-resolve differed
  on. It is the narrowed platform list, not the count, that AC-3 turns on.
- The consumer-side pass establishes that **`--locked` is not available at this stage**,
  and that the developer-machine measurement which suggested otherwise does not
  generalize. On a developer machine, `--dry-run` from `python/` with no `_build/`
  present, both flags exit 0 against the restamped lock and install the same eight
  dev-group packages, none of them ovstage; against a lock reverted to trunk's drift
  `--frozen` still exits 0 while `--locked` exits 1, identically on `uv` 0.9.7 and
  0.12.8. That made `--locked` look like a free early guard.

  It is not. Run for real by `ci_validate.cmake` in CI, before `fetch_deps`, `--locked`
  fails on **every** platform — Linux x86_64, Linux aarch64 and Windows, on the pinned
  `uv` 0.9.7, in child pipeline `65843418` — with the lock and `pyproject.toml` in
  agreement. Its freshness check invalidates the lock at that stage and re-resolves,
  reaching CI's Artifactory index (REQ AC-6):

  ```text
  × No solution found when resolving dependencies for split (markers:
  │ python_full_version >= '3.12'):
  ╰─▶ Because there is no version of ovstage==0.2.0.374259 and your project
      depends on ovstage==0.2.0.374259, we can conclude that your project's
      requirements are unsatisfiable.
      hint: `ovstage` was found on <CI index>, but not at the requested
      version (ovstage==0.2.0.374259).
  ```

  The pin is not stale — `fetch_ovstage_release.py` resolves `0.2.0.374259` from the
  index its own `INDEX_URL` names, and `65994557` staged that wheel — but the index the
  hint reports is a *different* Artifactory repository, reached through the runner's
  `UV_INDEX` / `PIP_INDEX_URL`, so the hint is not evidence about the pin either way.
  The failure is the re-resolution itself: `python/pyproject.toml` sets
  `requires-python = ">=3.10"`, so `uv` resolves for Python versions past whichever one
  the runner has — 3.10.12 on both Linux runners, 3.13.9 on Windows — opening the
  `python_full_version >= '3.12'` split the lock does not answer for. The run says so
  itself, in a second hint the block above elides: "While the active Python version is
  3.10, the resolution failed for other Python versions supported by your project.
  Consider limiting your project's supported Python versions using `requires-python`."
  Only `--frozen` declines to resolve. The consumer therefore stays on `--frozen`, and the
  pin-agreement check that `--locked` was adopted for lives in a checked-in test that
  compares the two files as text and needs no resolver (REQ AC-6).

## What was not executed

- **CI on the final head.** The pipeline evidence above is `65994557` on `ff4e9ababb`;
  no pipeline named here covers the commits after that head, none of which change the
  drivers or the locks.
- **Resolution-affecting `UV_*` environment variables in the drivers.** The scope pass
  covers `UV_NO_CONFIG`'s effect on configuration files, and the historical-CI reading
  shows `UV_INDEX` / `PIP_INDEX_URL` reaching `uv` through it, but no run here set
  `UV_NO_SOURCES`, `UV_RESOLUTION` or `UV_EXCLUDE_NEWER` against these drivers to see
  what it changes. AC-1 is written to exclude that case rather than to answer it, and the
  drivers clear no such variable.
- **The benchmark driver.** `scripts/test_benchmarks_python.cmake` carries the same
  construction against `tests/python_benchmarks/`, whose lock records the same relative
  registry string, and that lock's acceptance was measured directly with
  `uv lock --check`. Benchmarks are opt-in and excluded from `validate_all.cmake` (CTest
  label `benchmarks`), so the driver itself is covered by inspection only.
- **`python/uv.lock` in anger.** Its only resolving consumer is
  `scripts/test_pyright.cmake` with `--frozen --no-install-project --only-group dev`, so
  ovstage is never installed from it — which is why pyright stayed green through the whole
  period the lock was stale, and why nothing at that gate can catch the drift. Its ovstage
  entry is verified by `uv lock --check` and by the AC-6 test, but by no run that installs
  the wheel it names.
- **`required-environments` for the benchmark project.** Giving
  `tests/python_benchmarks/pyproject.toml` the three required environments `python/` and
  `tests/python_tests/` declare — so a re-resolution there fails instead of narrowing —
  was tried and **deliberately not taken here**. It is an input change to the lock, so
  `uv lock --check` immediately reports it out of date and the whole lock has to be
  regenerated — every package in the benchmark graph, including `torch` and the
  `nvidia-*` / `triton` packages the `gpu` extra pulls in, re-resolved for `win_amd64`
  and `linux-aarch64` with all three platforms' ovstage wheels staged. That
  regeneration was not re-run for this record, so no package or wheel-entry count for it
  is stated here. AC-3 keeps its carve-out until it lands as its own change.
- **Installing from the stamped non-runtime locks.** Only their acceptance was measured;
  no run here installed the aarch64 or `win_amd64` wheels those entries name.
- **A full pytest run through the developer wrapper.** What the wrapper passes and
  whether `uv` accepts it were both measured directly, but no suite was run end to end
  through `run_pytest.sh`: that needs a built `_install/` tree and a real staged wheel,
  which is the `python-runtime` CTest entry's job. No CI job invokes the wrapper, so its
  half of AC-2 rests on those two measurements plus inspection.
- **The newer `uv` in CI.** The pin is 0.9.7, and the pipeline logs confirm it by
  printing `Found uv: uv 0.9.7`, so every CI result recorded here is a 0.9.7 result.
  Against the lock checked in today 0.9.7 accepts both find-links spellings, so a green
  pipeline shows the relative spelling does not regress the pinned version — it does not
  exercise the discard the fix prevents. What makes that a statement about the lock
  rather than about the version is the historical-CI evidence above. AC-4's other half,
  the 0.12.8 behaviour, remains a developer-machine measurement.
- **Why the pre-27-August lock was discarded, and the current one is not.** This is
  deliberately left unestablished. The failures stop after the 26 August nightly, and
  the only change to `tests/python_tests/uv.lock` in that window is commit `70f40999af`
  (27 August), which regenerated it — not an ancestor of the 26 August nightly's SHA
  `5e798a88a2`, an ancestor of the 29 August one's `d98b2a2cf2`. So the window is right,
  but no run here reproduced the discard against the pre-27-August lock, and nothing
  identifies which of that regeneration's changes ends it. Two candidate explanations
  are ruled out rather than adopted: the pinned `uv` version did not move, and the
  31 August and 1 September ovstage bumps rewrote the same lock textually without
  bringing the failure back, so a stamp-only edit is not what invalidates it. The
  1 September ovstage bump also post-dates the last failure by five days, so it did not
  fix it either. The fix in this MR does not rest on the answer: it removes the absolute
  spelling that was the common factor in both the August failures and the 0.12.8
  measurement.

## Code References

- ovphysx/scripts/test_python_runtime.cmake
- ovphysx/scripts/test_benchmarks_python.cmake
- ovphysx/scripts/run_uv_pytest.py
- ovphysx/tests/python_tests/run_pytest.sh, ovphysx/tests/python_tests/run_pytest.bat
- ovphysx/scripts/validate_python_test_uv_lock.py
- ovphysx/scripts/ci_validate.cmake
- ovphysx/scripts/fetch_ovstage_release.py
- ovphysx/scripts/test_pyright.cmake
- ovphysx/tests/python_tests/uv.lock
- ovphysx/tests/python_tests/pyproject.toml
- ovphysx/python/pyproject.toml
