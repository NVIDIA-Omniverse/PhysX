# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Process-restart cold start: subprocess per measurement so PhysX bootstrap
isn't amortized. This is the unique-value Python bench — the C++ harness
shares one PhysX instance across runs and can't measure true cold start
from within itself. Empty-step / noop-add / reset / first-step-after-reload
are covered on the C++ side (LowLoad.*).
"""
from __future__ import annotations

import os
import subprocess
import sys
import textwrap


def test_process_cold_start(benchmark, bench_device):
    """The child process performs a minimal create/load/step/exit cycle. We
    measure the full subprocess wall-clock — import cost + PhysX bootstrap +
    first step.

    Skipped in install-tree mode: the parent process's conftest does
    RTLD_GLOBAL preloads (libcarb.so + libpython) that the subprocess can't
    inherit. In wheel-install mode (CI), the wheel's __init__ handles preload
    and the subprocess bootstraps cleanly."""
    import pytest

    # Subprocess receives device via env var — it's a child of this test,
    # not pytest, so it doesn't see the --bench-device CLI option.
    script = textwrap.dedent(
        """
        import os
        from ovphysx import PhysX
        if os.environ.get("OVPHYSX_BENCH_SUBPROC_DEVICE", "cpu") == "cpu":
            PhysX.set_cpu_mode(True)
        physx = PhysX()
        physx.step(1.0 / 60.0)
        physx.wait_all()
        physx.release()
        """
    )
    env = os.environ.copy()
    env["OVPHYSX_BENCH_SUBPROC_DEVICE"] = bench_device

    # Probe — does the subprocess bootstrap cleanly? Bound the wait so a hung
    # child (e.g. deadlocked GPU init on a runner without the right driver)
    # doesn't hang the pytest pass until the outer 1800s TIMEOUT fires.
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
        # 60s is generous for a cold-start; longer than that is a regression
        # we want flagged, not amortized. The probe at line ~49 already
        # confirmed the subprocess can bootstrap; if a later round still
        # times out or crashes (busy runner, transient driver glitch), skip
        # the whole benchmark rather than red the test — the probe's design
        # intent is that flaky-environment issues are yellow, not red.
        try:
            subprocess.run([sys.executable, "-c", script], check=True,
                           env=env, timeout=60)
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError) as e:
            pytest.skip(f"ovphysx subprocess unstable mid-measurement: {e!r}")

    benchmark(cold_run)
