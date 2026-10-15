# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Log-callback reads and startup writes return instead of deadlocking.

@implements REQ-CAPI-CONFIG-001
@covers AC-9

Runs in its own subprocess (see test_python_runtime.cmake). Each case nests
another subprocess with a timeout so a deadlock fails the test.
"""

import subprocess
import sys
import textwrap

from ovphysx import ProfilerBackend

_TIMEOUT_S = 60


def _run_child(script: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, "-c", textwrap.dedent(script)],
        check=False,
        capture_output=True,
        text=True,
        timeout=_TIMEOUT_S,
    )


def test_profiler_read_from_log_callback_returns():
    """A handler can read the bound profiler while the already-bound warning is delivered."""
    completed = _run_child(
        """
        import logging
        import os

        from ovphysx import (
            GlobalConfigKey,
            PhysX,
            ProfilerBackend,
            enable_python_logging,
            get_global_config,
            set_global_config,
        )

        os.environ.pop("OVPHYSX_PROFILER", None)
        os.environ.pop("OVPHYSX_PROFILER_OUTPUT", None)
        set_global_config(GlobalConfigKey.PROFILER_BACKEND, int(ProfilerBackend.NVTX))
        enable_python_logging()
        logger = logging.getLogger("ovphysx")
        logger.setLevel(logging.DEBUG)

        class Handler(logging.Handler):
            def emit(self, record):
                if "already bound" not in record.getMessage():
                    return
                backend = get_global_config(GlobalConfigKey.PROFILER_BACKEND)
                print(f"BACKEND={backend}", flush=True)

        logger.addHandler(Handler())
        first = PhysX()
        try:
            os.environ["OVPHYSX_PROFILER"] = "none"
            second = PhysX()
            second.destroy()
        finally:
            first.destroy()
        """
    )
    assert completed.returncode == 0, completed.stderr
    assert f"BACKEND={int(ProfilerBackend.NVTX)}" in completed.stdout


def test_cpu_only_write_from_log_callback_is_rejected():
    """Setting CPU-only from the first create's log returns instead of taking the create lock."""
    completed = _run_child(
        """
        import logging

        from ovphysx import (
            GlobalConfigKey,
            LogLevel,
            PhysX,
            enable_python_logging,
            set_global_config,
            set_log_level,
        )

        set_log_level(LogLevel.VERBOSE)
        enable_python_logging()
        logger = logging.getLogger("ovphysx")
        logger.setLevel(logging.DEBUG)
        seen = {"rejected": False}

        class Handler(logging.Handler):
            def emit(self, record):
                if seen["rejected"]:
                    return
                try:
                    set_global_config(GlobalConfigKey.CPU_ONLY, True)
                except RuntimeError as error:
                    if "log callback" not in str(error):
                        raise
                    seen["rejected"] = True

        logger.addHandler(Handler())
        instance = PhysX()
        instance.destroy()
        if not seen["rejected"]:
            raise SystemExit("startup-only write was not rejected from the log callback")
        print("REJECTED", flush=True)
        """
    )
    assert completed.returncode == 0, completed.stderr
    assert "REJECTED" in completed.stdout
