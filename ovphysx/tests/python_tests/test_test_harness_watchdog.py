# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""The shipped pytest configuration ends a hung test with every thread's stack."""

# @implements REQ-PACKAGING-TESTHANG-001
# @covers AC-1 AC-2 AC-3 AC-4
# @maps_to TEST-PACKAGING-TESTHANG-001

import os
from pathlib import Path
import subprocess
import sys
import textwrap
import time

try:
    import tomllib
except ModuleNotFoundError:  # Python 3.10
    import tomli as tomllib

PROJECT = Path(__file__).resolve().parent
PYPROJECT = PROJECT / "pyproject.toml"


def _ini_options():
    with PYPROJECT.open("rb") as handle:
        return tomllib.load(handle)["tool"]["pytest"]["ini_options"]


def test_watchdog_is_declared_once_in_the_test_project():
    options = _ini_options()
    assert options["faulthandler_exit_on_timeout"] is True
    # Per test: at least five times the slowest test measured on any CI runner (about 40 s), and
    # below the pass TIMEOUTs in scripts/test_python_runtime.cmake (900 s) so the dump reaches the log.
    assert 200 <= float(options["faulthandler_timeout"]) <= 600


def _run_child(tmp_path, test_source, *extra):
    (tmp_path / "test_child.py").write_text(textwrap.dedent(test_source), encoding="utf-8")
    env = os.environ.copy()
    env.pop("PYTEST_ADDOPTS", None)
    env["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
    # The re-arm plugin lives next to this file; --confcutdir keeps the suite's native conftest out.
    env["PYTHONPATH"] = str(PROJECT) + os.pathsep + env.get("PYTHONPATH", "")
    command = [
        sys.executable, "-m", "pytest", "-c", str(PYPROJECT), f"--confcutdir={tmp_path}",
        "--rootdir", str(tmp_path), "-p", "no:cacheprovider", "-p", "_watchdog_rearm",
        "-o", "faulthandler_timeout=2", *extra, str(tmp_path / "test_child.py"),
    ]
    started = time.monotonic()
    result = subprocess.run(command, cwd=tmp_path, env=env, capture_output=True, text=True, timeout=90)
    return result, time.monotonic() - started


def _assert_ended_by_watchdog(result, elapsed, test_name, min_threads):
    output = result.stdout + result.stderr
    # faulthandler's exit=True ends the process with status 1.
    assert result.returncode == 1, output
    assert elapsed < 60, f"watchdog did not end the child in time ({elapsed:.1f}s):\n{output}"
    assert "Timeout (0:00:02)!" in result.stderr, output
    assert result.stderr.count("Thread 0x") >= min_threads, output
    assert test_name in result.stderr, output
    # The process ended from the watchdog, not from a completed session.
    assert "1 failed" not in output and "1 passed" not in output, output


def test_hung_test_dumps_every_thread_and_ends_the_process(tmp_path):
    result, elapsed = _run_child(tmp_path, """\
        import threading
        import time

        def test_sleeps_past_the_budget():
            helper = threading.Thread(target=time.sleep, args=(120,), name="helper", daemon=True)
            helper.start()
            time.sleep(120)
        """)
    _assert_ended_by_watchdog(result, elapsed, "test_sleeps_past_the_budget", min_threads=2)


def test_teardown_hang_after_a_failure_is_still_ended(tmp_path):
    # pytest cancels the timer in pytest_exception_interact; _watchdog_rearm arms it again.
    result, elapsed = _run_child(tmp_path, """\
        import time

        import pytest

        @pytest.fixture
        def stuck_on_teardown():
            yield
            time.sleep(120)

        def test_fails_then_teardown_hangs(stuck_on_teardown):
            assert False, "the failure that disarms pytest's own timer"
        """)
    # One thread here; the frame of the fixture generator is what the dump must show.
    _assert_ended_by_watchdog(result, elapsed, "stuck_on_teardown", min_threads=1)
