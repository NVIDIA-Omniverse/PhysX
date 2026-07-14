# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# Pytest conftest: initializes Carbonite and verifies USD Python bindings.
#
# _carb_setup starts the Carbonite framework and loads core plugins.
# The pxr module is made importable by a .pth file installed into the venv
# by build.sh.

import _carb_setup  # noqa: F401 - initializes Carbonite framework and loads plugins

from pxr import Usd  # noqa: F401

# Carb background threads call ExitProcess() during Python finalization, overriding
# pytest's exit code. Save it in pytest_sessionfinish (after XML and summary are
# written) then force-exit in pytest_unconfigure (last hook) before finalization runs.
_session_exitstatus = 0


def pytest_sessionfinish(session, exitstatus):
    global _session_exitstatus
    _session_exitstatus = int(exitstatus)


def pytest_unconfigure(config):
    import sys
    import os

    # Skip os._exit() when running under a debugger (e.g. debugpy) — it
    # terminates the process before the debugger can close the session,
    # which prevents breakpoints from working in VSCode's test explorer.
    if sys.gettrace() is not None:
        return

    os._exit(_session_exitstatus)
