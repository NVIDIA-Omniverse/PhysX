# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Tests for internal _bindings module.

Covers low-level ctypes bindings, library loading helpers, and the two
sentinel constants that live in _bindings (not in types.py).

Enum value coverage (TensorType, ApiStatus, LogLevel, BindingPrimMode)
is handled by test_types_sync.py and test_module_exports.py.
"""

import sys


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

    monkeypatch.setenv("OVPHYSX_LIB", str(external_path))
    monkeypatch.setattr(_bindings, "_bundled_lib_path", lambda: wheel_path)
    monkeypatch.setattr(_bindings, "_wheel_lib_path", lambda: wheel_path)
    monkeypatch.setattr(_bindings, "_preload_ovstage_runtime_deps", lambda: preload_calls.append(True))
    monkeypatch.setattr(_bindings, "_preload_windows_cpp_runtime", lambda: None)
    monkeypatch.setattr(_bindings, "_add_dll_directory", lambda _path: None)
    monkeypatch.setattr(_bindings, "_prefer_ovstage_runtime_dir", preferred_runtime_dirs.append)
    monkeypatch.setattr(_bindings.ctypes, "CDLL", lambda _path: loaded_library)

    assert _bindings._load_library() is loaded_library
    assert preload_calls == []
    assert preferred_runtime_dirs == [str(external_path.parent)]
