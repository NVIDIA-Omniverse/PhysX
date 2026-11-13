# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-MASS-001
# @covers AC-1
# @implements REQ-COMP-VEHICLE-001
# @covers AC-1
"""Worker loader selection tests; native integration is in wheel_worker_smoke.py."""
import ctypes
import importlib.util
import os
from types import SimpleNamespace
import sys

import pytest
from ovphysx_companion import _worker


pytestmark = pytest.mark.skipif(sys.platform != "linux", reason="Linux wheel bootstrap")


@pytest.fixture
def wheel_layout(tmp_path, monkeypatch):
    core = tmp_path / "ovphysx/lib/libovphysx.so"
    stage = tmp_path / "ovstage/bin/libovstage.so"
    for library in (core, stage):
        library.parent.mkdir(parents=True)
        library.touch()
    packages = {
        name: SimpleNamespace(origin=str(tmp_path / name / "__init__.py"))
        for name in ("ovphysx", "ovstage")
    }
    monkeypatch.setattr(importlib.util, "find_spec", packages.get)
    monkeypatch.setenv("OVPHYSX_LIB", str(core))
    calls = []

    def load(path, **kwargs):
        handle = SimpleNamespace(path=path)
        calls.append((path, kwargs, handle))
        return handle

    monkeypatch.setattr(ctypes, "CDLL", load)
    return core, stage, packages, calls


def test_wheel_retains_dependency_before_loading_core(wheel_layout):
    core, stage, _, calls = wheel_layout
    library = _worker.load_library()
    assert [(path, kwargs) for path, kwargs, _ in calls] == [
        (str(stage), {"mode": os.RTLD_GLOBAL}), (str(core), {})]
    assert library._ovstage_dependency is calls[0][2]


def test_sdk_override_does_not_preload_installed_wheel(wheel_layout, tmp_path, monkeypatch):
    _, _, _, calls = wheel_layout
    sdk = tmp_path / "sdk/libovphysx.so"
    sdk.parent.mkdir()
    sdk.touch()
    monkeypatch.setenv("OVPHYSX_LIB", str(sdk))
    library = _worker.load_library()
    assert [(path, kwargs) for path, kwargs, _ in calls] == [(str(sdk), {})]
    assert library._ovstage_dependency is None


def test_sdk_without_python_packages(wheel_layout):
    core, _, packages, calls = wheel_layout
    packages.clear()
    _worker.load_library()
    assert [(path, kwargs) for path, kwargs, _ in calls] == [(str(core), {})]


def test_missing_wheel_dependency_fails_before_core_load(wheel_layout):
    _, _, packages, calls = wheel_layout
    del packages["ovstage"]
    with pytest.raises(RuntimeError, match="Install ovstage"):
        _worker.load_library()
    assert not calls
