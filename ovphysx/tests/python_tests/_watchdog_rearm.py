# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Keep the hung-test watchdog armed after a failure.

pytest's faulthandler plugin arms ``faulthandler.dump_traceback_later`` per
test (``faulthandler_timeout``) and cancels it in its ``pytest_exception_interact``
hook, which the runner calls for every failed setup, call or subtest, so a
debugger can take over. Without a debugger that leaves the rest of the test,
its teardown included, unguarded: an assertion that fails and a ``destroy()`` in
teardown that then waits on a stuck step would hold the process indefinitely.
This hook runs after pytest's (``trylast``) and arms the same budget again
unless ``--pdb`` is in effect; the plugin's own ``finally`` cancels it when the
test ends. Loaded through ``pytest_plugins`` in conftest.py.
"""

# @implements REQ-PACKAGING-TESTHANG-001
# @covers AC-4

import faulthandler

import pytest
from _pytest.faulthandler import fault_handler_stderr_fd_key


@pytest.hookimpl(trylast=True)
def pytest_exception_interact(node, call, report):
    config = node.config
    if config.getoption("usepdb", default=False):
        return
    timeout = float(config.getini("faulthandler_timeout") or 0)
    stderr_fd = config.stash.get(fault_handler_stderr_fd_key, None)
    if timeout <= 0 or stderr_fd is None:
        return
    faulthandler.dump_traceback_later(
        timeout, file=stderr_fd, exit=bool(config.getini("faulthandler_exit_on_timeout")))
