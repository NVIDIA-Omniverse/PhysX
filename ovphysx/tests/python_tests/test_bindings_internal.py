# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Tests for internal _bindings module.

Covers low-level ctypes bindings, library loading helpers, and the two
sentinel constants that live in _bindings (not in types.py).

Enum value coverage (TensorType, ApiStatus, LogLevel, BindingPrimMode)
is handled by test_types_sync.py and test_module_exports.py.
"""

import os
import sys
from types import SimpleNamespace

import pytest


def _make_fake_ovstage_runtime(tmp_path, monkeypatch, bindings):
    package_dir = tmp_path / "ovstage"
    plugins_dir = package_dir / "bin" / "plugins"
    plugins_dir.mkdir(parents=True)
    package_file = package_dir / "__init__.py"
    package_file.write_text("", encoding="utf-8")
    usd_path = plugins_dir / "libov_25.11usd_ms.so"
    ovstage_path = package_dir / "bin" / "libovstage.so"
    usd_path.write_bytes(b"fake usd runtime")
    ovstage_path.write_bytes(b"fake ovstage runtime")

    monkeypatch.setitem(sys.modules, "ovstage", SimpleNamespace(__file__=str(package_file)))
    monkeypatch.setattr(bindings, "_prefer_ovstage_runtime_dir", lambda _path: None)
    monkeypatch.setattr(bindings, "_runtime_library_handles", [])
    return usd_path, ovstage_path


@pytest.mark.skipif(sys.platform != "linux", reason="RTLD_NOLOAD regression is Linux-only")
def test_preload_ovstage_runtime_reuses_already_loaded_dependencies(monkeypatch, tmp_path):
    from ovphysx import _bindings

    usd_path, ovstage_path = _make_fake_ovstage_runtime(tmp_path, monkeypatch, _bindings)
    usd_handle = object()
    ovstage_handle = object()
    handles_by_name = {
        usd_path.name: usd_handle,
        ovstage_path.name: ovstage_handle,
    }
    calls = []

    def fake_cdll(path, mode):
        path = os.fspath(path)
        calls.append((path, mode))
        return handles_by_name[os.path.basename(path)]

    monkeypatch.setattr(_bindings.ctypes, "CDLL", fake_cdll)

    _bindings._preload_ovstage_runtime_deps()

    probe_mode = os.RTLD_NOW | os.RTLD_NOLOAD | os.RTLD_GLOBAL
    assert calls == [
        (usd_path.name, probe_mode),
        (ovstage_path.name, probe_mode),
    ]
    assert _bindings._runtime_library_handles == [usd_handle, ovstage_handle]


@pytest.mark.skipif(sys.platform != "linux", reason="RTLD_NOLOAD regression is Linux-only")
def test_preload_ovstage_runtime_falls_back_after_soname_probe(monkeypatch, tmp_path):
    from ovphysx import _bindings

    usd_path, ovstage_path = _make_fake_ovstage_runtime(tmp_path, monkeypatch, _bindings)
    usd_handle = object()
    ovstage_handle = object()
    handles_by_path = {
        str(usd_path): usd_handle,
        str(ovstage_path): ovstage_handle,
    }
    calls = []

    def fake_cdll(path, mode):
        path = os.fspath(path)
        calls.append((path, mode))
        if not os.path.isabs(path):
            raise OSError(f"{path} is not loaded")
        return handles_by_path[path]

    monkeypatch.setattr(_bindings.ctypes, "CDLL", fake_cdll)

    _bindings._preload_ovstage_runtime_deps()

    probe_mode = os.RTLD_NOW | os.RTLD_NOLOAD | os.RTLD_GLOBAL
    assert calls == [
        (usd_path.name, probe_mode),
        (str(usd_path), os.RTLD_GLOBAL),
        (ovstage_path.name, probe_mode),
        (str(ovstage_path), os.RTLD_GLOBAL),
    ]
    assert _bindings._runtime_library_handles == [usd_handle, ovstage_handle]


def test_platform_lib_names():
    from ovphysx._bindings import _platform_lib_names

    lib_names = _platform_lib_names()
    assert isinstance(lib_names, list)
    assert len(lib_names) > 0
    if sys.platform == "win32":
        assert "ovphysx.dll" in lib_names
    else:
        assert "libovphysx.so" in lib_names


def test_invalid_handle_constant():
    from ovphysx._bindings import OVPHYSX_INVALID_HANDLE

    assert isinstance(OVPHYSX_INVALID_HANDLE, int)
    assert OVPHYSX_INVALID_HANDLE == 0


def test_op_index_all_constant():
    from ovphysx._bindings import OP_INDEX_ALL

    assert isinstance(OP_INDEX_ALL, int)
    assert OP_INDEX_ALL == 0xFFFFFFFFFFFFFFFF


def test_bundled_lib_path():
    from ovphysx._bindings import _bundled_lib_path

    path = _bundled_lib_path()
    if path is not None:
        assert isinstance(path, str)
        assert len(path) > 0


def test_load_library_preserves_wheel_dependencies_for_bundled_override(monkeypatch, tmp_path):
    from ovphysx import _bindings

    wheel_path = str(tmp_path / "ovphysx" / "lib" / _bindings._platform_lib_names()[0])
    loaded_library = object()
    preload_calls = []

    monkeypatch.setenv("OVPHYSX_LIB", wheel_path)
    monkeypatch.setattr(_bindings, "_bundled_lib_path", lambda: wheel_path)
    monkeypatch.setattr(_bindings, "_wheel_lib_path", lambda: wheel_path)
    monkeypatch.setattr(_bindings, "_preload_ovstage_runtime_deps", lambda: preload_calls.append(True))
    monkeypatch.setattr(_bindings, "_preload_windows_cpp_runtime", lambda: None)
    monkeypatch.setattr(_bindings, "_add_dll_directory", lambda _path: None)
    monkeypatch.setattr(_bindings, "_prefer_ovstage_runtime_dir", lambda _path: None)
    monkeypatch.setattr(_bindings.ctypes, "CDLL", lambda _path: loaded_library)

    assert _bindings._load_library() is loaded_library
    assert preload_calls == [True]


def test_load_library_keeps_external_override_isolated(monkeypatch, tmp_path):
    from ovphysx import _bindings

    library_name = _bindings._platform_lib_names()[0]
    wheel_path = str(tmp_path / "wheel" / "ovphysx" / "lib" / library_name)
    external_path = tmp_path / "sdk" / "lib" / library_name
    loaded_library = object()
    preload_calls = []
    preferred_runtime_dirs = []

    ovstage_bin = str(tmp_path / "ovstage" / "bin")

    monkeypatch.setenv("OVPHYSX_LIB", str(external_path))
    monkeypatch.setattr(_bindings, "_bundled_lib_path", lambda: wheel_path)
    monkeypatch.setattr(_bindings, "_wheel_lib_path", lambda: wheel_path)
    monkeypatch.setattr(_bindings, "_preload_ovstage_runtime_deps", lambda: preload_calls.append(True))
    monkeypatch.setattr(_bindings, "_preload_windows_cpp_runtime", lambda: None)
    monkeypatch.setattr(_bindings, "_add_dll_directory", lambda _path: None)
    monkeypatch.setattr(_bindings, "_prefer_ovstage_runtime_dir", preferred_runtime_dirs.append)
    monkeypatch.setattr(_bindings, "_ovstage_bin_dir", lambda _pkg_dir: ovstage_bin)
    monkeypatch.setattr(_bindings.ctypes, "CDLL", lambda _path: loaded_library)

    assert _bindings._load_library() is loaded_library
    assert preload_calls == []
    # The ovstage DLL directory is only registered on Windows.
    expected_dirs = [str(external_path.parent)]
    if sys.platform == "win32":
        expected_dirs.append(ovstage_bin)
    assert preferred_runtime_dirs == expected_dirs
