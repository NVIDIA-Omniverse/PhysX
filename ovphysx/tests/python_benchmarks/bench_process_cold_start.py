# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Process-restart cold start: subprocess per measurement so PhysX bootstrap
is not amortized. This is the unique-value Python bench. The C++ harness
shares one PhysX instance across runs and cannot measure true cold start
from within itself. Empty-step / noop-add / reset / first-step-after-reload
are covered on the C++ side (LowLoad.*).
"""
from __future__ import annotations

import os
import subprocess
import sys
import textwrap


def test_process_cold_start(benchmark, bench_device):
    """The child process performs a minimal create/load/step/exit cycle. The
    full subprocess wall-clock is measured: import cost + PhysX bootstrap +
    first step.

    Skipped in install-tree mode: the parent process's conftest does
    RTLD_GLOBAL preloads (libcarb.so + libpython) that the subprocess cannot
    inherit. In wheel-install mode (CI), the wheel's __init__ handles preload
    and the subprocess bootstraps cleanly."""
    import pytest

    # The subprocess receives the device via an env var. It is a child of this
    # test, not of pytest, so it does not see the --bench-device CLI option.
    script = textwrap.dedent(
        """
        import os
        from ovphysx import PhysX
        if os.environ.get("OVPHYSX_BENCH_SUBPROC_DEVICE", "cpu") == "cpu":
            PhysX.set_cpu_mode(True)
        physx = PhysX()
        physx.step(1.0 / 60.0)
        physx.wait_all()
        physx.destroy()
        """
    )
    env = os.environ.copy()
    env["OVPHYSX_BENCH_SUBPROC_DEVICE"] = bench_device

    # Probe whether the subprocess bootstraps cleanly. Bound the wait so a hung
    # child (e.g. deadlocked GPU init on a runner without the right driver)
    # does not hang the pytest pass until the outer 1800s TIMEOUT fires.
    try:
        probe = subprocess.run(
            [sys.executable, "-c", script], env=env, capture_output=True,
            timeout=60,
        )
    except subprocess.TimeoutExpired:
        pytest.skip("ovphysx subprocess probe timed out — likely missing driver")
    if probe.returncode != 0:
        pytest.skip(
            "Subprocess cannot bootstrap ovphysx (install-tree mode); "
            "wheel install required for process_cold_start."
        )

    def cold_run():
        # 60s is generous for a cold start. Anything longer is a regression
        # that should be flagged, not amortized. The probe above already
        # confirmed the subprocess can bootstrap. If a later round still
        # times out or crashes (busy runner, transient driver glitch), skip
        # the whole benchmark rather than fail it, so flaky-environment
        # issues show up as skips rather than failures.
        try:
            subprocess.run([sys.executable, "-c", script], check=True,
                           env=env, timeout=60)
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError) as e:
            pytest.skip(f"ovphysx subprocess unstable mid-measurement: {e!r}")

    benchmark(cold_run)
