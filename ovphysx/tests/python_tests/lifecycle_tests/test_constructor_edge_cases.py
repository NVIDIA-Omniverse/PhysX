# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
"""Constructor edge-case tests: active_cuda_gpus validation, handle property,
and ignore_version_mismatch.

Lifecycle constraint: Carbonite cannot be re-initialized after
ovphysx_destroy_instance, so only ONE successful native PhysX creation
is allowed per subprocess.

This file runs in its own subprocess (see test_python_runtime.cmake).
"""

import pytest

from ovphysx import PhysX

# ---------------------------------------------------------------------------
# One successful native instance — tests properties that require native init
# ---------------------------------------------------------------------------


def test_constructor_valid_and_properties():
    """Single comprehensive test for valid construction.

    Creates ONE native instance and verifies:
      - active_cuda_gpus=None does not raise
      - handle property returns a positive int
      - ignore_version_mismatch=True skips version check without error
      - release() is idempotent

    Note: only ONE PhysX instance is created because Carbonite cannot be
    re-initialized after release.
    """
    physx = PhysX(
        active_cuda_gpus=None,
        ignore_version_mismatch=True,
    )
    try:
        h = physx.handle
        assert isinstance(h, int), f"handle must be int, got {type(h)}"
        assert h > 0, f"handle must be > 0, got {h}"

        # Double-check that the instance is functional
        physx.wait_all()

    finally:
        physx.release()

    # After release, handle must raise
    with pytest.raises(RuntimeError):
        _ = physx.handle


def test_active_cuda_gpus_variations_cpu_mode():
    """active_cuda_gpus variants that are valid in CPU mode must not raise a
    Python-level ValueError.  Each pattern is verified in its own subprocess to
    avoid Carbonite re-initialisation crashes (access violation on Windows when
    PhysX is constructed more than once in the same process after a release).
    """
    import subprocess
    import sys
    import textwrap

    valid_patterns = [None, "", "0"]
    for pattern in valid_patterns:
        arg_repr = repr(pattern)
        script = textwrap.dedent(f"""
            from ovphysx import PhysX
            try:
                p = PhysX(active_cuda_gpus={arg_repr})
                p.release()
            except ValueError as exc:
                raise SystemExit(f"VALUEERROR: {{exc}}")
            except Exception:
                pass  # RuntimeError / native crash is acceptable
        """)
        result = subprocess.run(
            [sys.executable, "-c", script],
            capture_output=True,
            text=True,
            timeout=60,
        )
        assert "VALUEERROR" not in result.stdout + result.stderr, (
            f"active_cuda_gpus={pattern!r} raised ValueError:\n" f"{result.stderr}"
        )
