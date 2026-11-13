<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PACKAGING-TESTHANG-001
maps_to: REQ-PACKAGING-TESTHANG-001
type: unit
---

## Scenario

Prove that the test project's pytest configuration arms a per-test watchdog
that dumps every thread's stack and ends the process, without hanging the
suite that runs the proof.

## Given

- `tests/python_tests/pyproject.toml` as shipped.
- Throwaway test files: one whose single test sleeps far longer than a short
  override of the budget, with a second thread blocked in a sleep so the dump
  has more than one thread to show; one whose test fails and whose fixture
  teardown then sleeps.

## When

- `test_test_harness_watchdog.py` reads the shipped configuration with
  `tomllib`.
- It runs each throwaway test in a child Python with `-c` pointing at the
  shipped `pyproject.toml`, `--confcutdir` at the throwaway directory (so the
  suite's native fixtures stay out of the child), `-p _watchdog_rearm`, and
  `-o faulthandler_timeout=2` as the only override.

## Then

- The shipped configuration declares `faulthandler_exit_on_timeout = true` and
  a `faulthandler_timeout` between 200 and 600 seconds: above five times the
  slowest measured test, below the 900 s pass bounds (REQ AC-1, AC-2).
- The sleeping child exits with status 1 well before the sleep would end, its
  stderr names the timeout and lists the frames of more than one thread,
  including the sleeping test function; no completed-session summary line is
  printed (REQ AC-3).
- The failing child is ended the same way during its teardown: the failure
  cancelled pytest's own timer and the re-arm hook armed it again (REQ AC-4).
- Not exercised here, by design: hangs in collection, session start or finish
  and interpreter exit, which REQ AC-3 lists as outside the watchdog.
