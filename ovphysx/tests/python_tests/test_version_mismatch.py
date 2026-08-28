# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

import pytest

from ovphysx import PhysX, _bindings


def test_version_mismatch_raises(monkeypatch):
    monkeypatch.setattr(_bindings, "get_native_version_string", lambda: "9.9.9")

    with pytest.raises(RuntimeError, match="version does not match"):
        PhysX()


def test_shared_fixture_available(physx_sdk):
    """Smoke check that the shared physx_sdk fixture is alive and holds a valid handle.

    The ignore_version_mismatch create/release path is tested in
    lifecycle_tests/test_version_mismatch.py (separate subprocess).
    """
    assert physx_sdk is not None
    assert physx_sdk.handle != 0
