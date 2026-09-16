# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Version mismatch lifecycle test with ONE create+destroy.

Runs in its own subprocess (see test_python.cmake).
"""

import pytest

from ovphysx import PhysX, _bindings


def test_version_mismatch_can_be_ignored(monkeypatch):
    monkeypatch.setattr(_bindings, "get_native_version_string", lambda: "9.9.9")

    physx = PhysX(ignore_version_mismatch=True)
    physx.destroy()
