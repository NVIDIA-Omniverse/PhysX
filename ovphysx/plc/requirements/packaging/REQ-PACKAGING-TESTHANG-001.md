<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PACKAGING-TESTHANG-001
title: A hung Python test fails its pass within minutes and leaves every thread's stack
status: implemented
owner: ovphysx
---

## Description

The installed-SDK Python test drivers run pytest in child processes and bound
each pass with `execute_process(TIMEOUT)`. That bound is not a guard against a
hung test: CMake 3.28 to 4.1.2 (issue #27378; the Windows runners use 4.1.0)
stop waiting when the timer fires without terminating the child, and the
orphaned `uv` and `python` processes then keep the CI step's output pipes open
until the job timeout, with no diagnostic; CMake 4.4 (GitLab Linux) kills the
child, again with no diagnostic. The lifecycle phase of 2026-10-20 (GitHub job
118663756922, one lifecycle test that never wrote its junit file, a Windows GPU
runner held for five hours) is handled by the lifecycle runner's own per-job
budget and tree kill (`scripts/parallel_execute.cmake`); this requirement is the
same hazard for the passes that run inside one pytest process.

The guard therefore lives in the process that can hang. The test project's
pytest configuration arms pytest's built-in faulthandler watchdog for every
test, so a single test that overruns its budget writes the Python stack of
every thread to the process's stderr and ends the process. The driver then
reports a failed pass through its ordinary exit-code check, and the log holds
the frames a developer needs.

## Acceptance Criteria

- **AC-1: One per-test budget, declared in the test project.**
  `tests/python_tests/pyproject.toml` sets `faulthandler_timeout` and
  `faulthandler_exit_on_timeout = true` under `[tool.pytest.ini_options]`,
  so every pytest invocation of that project (whole-suite, CPU-mode and utils
  passes, developer runs through `run_pytest.*`, and each lifecycle file's own
  process) is covered by the same rule. No driver passes a second watchdog.
  Outside it: the lifecycle files are ended by their runner's 120 s per-job
  budget first, and the companion suites are another pytest project. The
  project requires `pytest>=9`, where `faulthandler_exit_on_timeout` exists.
- **AC-2: Sized from measured test durations, and below the pass bounds.** The
  budget is per test and at least five times the slowest test measured on any
  CI runner (about 40 s for `test_public_build_flags` on the Windows VS runner;
  the slowest CPU-mode test is about 11 s on linux-debug). Every pass
  `TIMEOUT` in `scripts/test_python_runtime.cmake` exceeds the budget plus the
  pass's own runtime (900 s against a budget of 300 s and passes of up to about
  150 s), so the dump is in the log before the driver stops waiting or, on
  CMake 4.4, kills the child.
- **AC-3: Thread stacks before exit.** On overrun, every thread's Python stack
  is written to the pytest process's stderr (the fd pytest duplicated at
  configure time, which the CMake driver echoes into the job log) before the
  process exits with status 1; the driver's exit-code check then fails the
  pass and the junit XML for that pass is missing. The timer covers the
  setup, call and teardown of each test. It does not cover collection, session
  start or finish, atexit handlers or a non-daemon thread at interpreter exit.
  pytest's own pdb cancels it; an IDE debugger does not, so a long breakpoint
  needs `-o faulthandler_timeout=0`.
- **AC-4: A failure does not disarm the watchdog.** pytest's faulthandler plugin
  cancels its timer in `pytest_exception_interact`, which runs for every
  failed setup, call or subtest. `tests/python_tests/_watchdog_rearm.py`
  (loaded through `pytest_plugins` in `conftest.py`) arms the same budget again
  after pytest's hook unless `--pdb` is in effect, so a teardown that hangs
  after an assertion failure is ended like any other hang.

## Test References

- [TEST-PACKAGING-TESTHANG-001](../../tests/packaging/TEST-PACKAGING-TESTHANG-001.md)

## Code References

- `ovphysx/tests/python_tests/pyproject.toml`: `faulthandler_timeout`,
  `faulthandler_exit_on_timeout`, `pytest>=9` (AC-1 through AC-3).
- `ovphysx/tests/python_tests/_watchdog_rearm.py`, registered in
  `tests/python_tests/conftest.py` (AC-4).
- `ovphysx/tests/python_tests/test_test_harness_watchdog.py`: regression test.
- `ovphysx/scripts/test_python_runtime.cmake`: the pass `TIMEOUT`s sized above
  the budget (AC-2); its comment records which CMake versions stop waiting
  without terminating the child.

## Dependencies

- REQ-PACKAGING-TESTRESULT-001 (the driver's result checks that turn the exit
  into a failed pass).
