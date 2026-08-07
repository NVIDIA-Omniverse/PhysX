# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
"""Verify that ERROR_MARKERS in conftest.py catch native C++ runtime warnings.

The conftest hook checks captured stdout/stderr and native log records for
strings listed in ERROR_MARKERS.  This test emits a matching sentinel via
stderr and verifies that the conftest hook causes the emitting test to fail.

Because the error-marker mechanism fails the *emitting* test (not the test that
checks for failure), we run the emitting test in a subprocess via pytest and
assert that it exits with a failure status.
"""

import os
import subprocess
import sys
import tempfile
import textwrap

import pytest


def test_error_markers_catch_native_warnings():
    """A test that emits an ERROR_MARKERS-matching string must fail."""
    # "[ErrorMarkerSentinel]" is in ERROR_MARKERS in conftest.py.
    # The inner test prints it to stderr, which the conftest makereport hook
    # picks up via report.capstderr.
    inner_test = textwrap.dedent("""\
        import sys

        def test_emit_sentinel():
            # Matches an ERROR_MARKERS entry; the conftest makereport hook scans captured stderr for these.
            print("[ErrorMarkerSentinel]", file=sys.stderr)
    """)

    test_dir = os.path.dirname(os.path.abspath(__file__))
    conftest_dir = os.path.dirname(test_dir)  # parent has conftest.py

    with tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".py",
        prefix="test_emit_marker_",
        dir=conftest_dir,
        delete=False,
    ) as f:
        f.write(inner_test)
        tmp_test = f.name

    try:
        # Use --capture=fd to capture C-level file descriptor output.
        # Do NOT pass -s (which disables capture entirely).
        result = subprocess.run(
            [sys.executable, "-m", "pytest", "--override-ini=addopts=", "--capture=fd", "-xv", tmp_test],
            capture_output=True,
            text=True,
            cwd=conftest_dir,
            timeout=120,
        )

        combined = result.stdout + result.stderr
        # The inner test should FAIL because it emits an ERROR_MARKERS match
        assert result.returncode != 0, (
            f"Expected subprocess pytest to fail (rc!=0) but got rc={result.returncode}.\n"
            f"output:\n{combined[-3000:]}"
        )
        assert "FAILED" in combined, f"Expected 'FAILED' in subprocess output.\n" f"output:\n{combined[-3000:]}"
    finally:
        os.unlink(tmp_test)
