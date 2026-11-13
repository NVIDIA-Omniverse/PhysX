# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""scripts/parallel_execute.cmake ends an overrunning job as a whole process tree.

Drives the real runner through a small cmake script with a 5 s budget and three
jobs: one that exits 0, one that exits 3, and a python that starts a python child
and sleeps (on Linux it also ignores TERM). Expected: both exit codes reported,
one TIMEOUT line, the sleeper and its child gone, the phase over soon after the
budget. On Windows this is the only place the overrun path (process tree,
minidump, taskkill /T) runs outside a real hang.
"""

import os
from pathlib import Path
import shutil
import subprocess
import sys
import textwrap
import time

import pytest

ROOT = Path(__file__).resolve().parents[2]
RUNNER = ROOT / "scripts" / "parallel_execute.cmake"
pytestmark = pytest.mark.skipif(shutil.which("cmake") is None, reason="cmake not on PATH")


def _proc_state(pid):
    """Linux: (state, ppid, pgid, sid) from /proc, None when the pid is gone. State Z is a zombie."""
    try:
        raw = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    fields = raw[raw.rindex(")") + 2:].split()
    return fields[0], fields[1], fields[2], fields[3]


def _alive(pid):
    # A killed process whose parent died before reaping it stays a zombie on a container whose
    # PID 1 does not reap (the GitHub hooks pods, where PID 1 is the runner's node agent):
    # os.kill(pid, 0) still succeeds on it, but it is dead (probe run 39775960932).
    if os.name == "nt":
        import ctypes
        kernel32 = ctypes.windll.kernel32
        handle = kernel32.OpenProcess(0x1000, False, pid)  # PROCESS_QUERY_LIMITED_INFORMATION
        if not handle:
            return False
        try:
            code = ctypes.c_ulong()
            kernel32.GetExitCodeProcess(handle, ctypes.byref(code))
            return code.value == 259  # STILL_ACTIVE
        finally:
            kernel32.CloseHandle(handle)
    state = _proc_state(pid) if sys.platform.startswith("linux") else None
    if state is not None:
        return state[0] != "Z"
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def test_overrunning_job_tree_is_ended_and_exit_codes_are_reported(tmp_path):
    jobs = tmp_path / "jobs"
    jobs.mkdir()
    pid_file = tmp_path / "sleeper_pids.txt"
    sleeper = tmp_path / "sleeper.py"
    sleeper.write_text(textwrap.dedent(f"""\
        import os, signal, subprocess, sys, time
        if os.name != "nt":
            signal.signal(signal.SIGTERM, signal.SIG_IGN)  # the runner must not depend on TERM being honoured
        child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(600)"])
        with open({str(pid_file)!r}, "w", encoding="utf-8") as handle:
            handle.write(f"{{os.getpid()}} {{child.pid}}\\n")
        time.sleep(600)
        """), encoding="utf-8")
    # Python one-liners for the trivial jobs: a quoted "/c" is not a switch to cmd.exe, so
    # `cmd /c exit 0` through the job script's argument quoting returns 1, not 0. No semicolon
    # in an argument: ovphysx_write_job_script takes a CMake list, which splits on ';'.
    python = Path(sys.executable).as_posix()
    ok_cmd = f'[==[{python}]==] -c "raise SystemExit(0)"'
    fail_cmd = f'[==[{python}]==] -c "raise SystemExit(3)"'
    ext = "bat" if os.name == "nt" else "sh"
    driver = tmp_path / "driver.cmake"
    driver.write_text(textwrap.dedent(f"""\
        cmake_minimum_required(VERSION 3.19)
        set(OVPHYSX_PARALLEL_JOB_TIMEOUT 5)
        include([==[{RUNNER.as_posix()}]==])
        set(_jobs [==[{jobs.as_posix()}]==])
        ovphysx_write_job_script("${{_jobs}}/ok.{ext}" "${{_jobs}}" {ok_cmd})
        ovphysx_write_job_script("${{_jobs}}/fail.{ext}" "${{_jobs}}" {fail_cmd})
        # The real job shape: `cmake -E env ... python`, where CMake 4.4's env exits on TERM at once.
        ovphysx_write_job_script("${{_jobs}}/sleeper.{ext}" "${{_jobs}}" "${{CMAKE_COMMAND}}" -E env OVPHYSX_PARALLEL_SELFTEST=1 [==[{Path(sys.executable).as_posix()}]==] [==[{sleeper.as_posix()}]==])
        set(_scripts "${{_jobs}}/ok.{ext}" "${{_jobs}}/fail.{ext}" "${{_jobs}}/sleeper.{ext}")
        ovphysx_run_job_scripts_parallel(3 "${{_scripts}}" _failed _output)
        message(STATUS "FAILED=${{_failed}}")
        """), encoding="utf-8")

    started = time.monotonic()
    result = subprocess.run([shutil.which("cmake"), "-P", str(driver)], cwd=tmp_path,
                            capture_output=True, text=True, timeout=240)
    elapsed = time.monotonic() - started
    output = result.stdout + result.stderr
    assert result.returncode == 0, output

    failed_line = next(line for line in result.stdout.splitlines() if "FAILED=" in line)
    failed = {Path(item).stem for item in failed_line.split("FAILED=", 1)[1].split(";") if item}
    assert failed == {"fail", "sleeper"}, output
    assert "done ok rc=0" in output, output
    assert "done fail rc=" in output and "done fail rc=0" not in output, output  # 3 on Linux, 1 via the .bat
    assert "TIMEOUT sleeper after" in output, output
    # The budget is 5 s; the Windows diagnostics (CIM query, minidump job, taskkill) may add up to a minute.
    assert elapsed < (150 if os.name == "nt" else 60), f"phase took {elapsed:.0f}s:\n{output}"

    pids = [int(token) for token in pid_file.read_text(encoding="utf-8").split()]
    assert len(pids) == 2, output
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline and any(_alive(pid) for pid in pids):
        time.sleep(0.5)
    survivors = [pid for pid in pids if _alive(pid)]
    states = {pid: _proc_state(pid) for pid in pids} if sys.platform.startswith("linux") else {}
    assert not survivors, f"sleeper processes still alive: {survivors} (state, ppid, pgid, sid: {states})\n{output}"

    if os.name == "nt":
        assert "taskkill /T /F:" in output, output
        dumps = sorted(jobs.glob("sleeper.bat.*.dmp"))
        assert dumps, f"no minidump next to the sleeper job log:\n{output}"
        assert all(dump.stat().st_size > 0 for dump in dumps), output
        assert "minidump" in output, output
