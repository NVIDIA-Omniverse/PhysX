# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Tests for namespaced USD host isolation in ovphysx Python bootstrap."""

import os
import subprocess
import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest


def _bundled_usd_plugin_dir():
    """Return the ovphysx package's own ``plugins/usd`` if it exists, else None.

    ``_candidate_plugin_paths()`` ALWAYS appends ``<package>/plugins/usd`` (the
    schemas that ship with this ovphysx) in addition to any ``OVPHYSX_LIB``-derived
    path. That directory is present in a wheel / installed layout and absent in a
    bare source checkout, so tests that assert the exact resulting path list (or that
    registration fails when OVPHYSX_LIB has no plugins/usd) must account for it.
    """
    import ovphysx.usd_version_check as _uvc

    bundled = Path(_uvc.__file__).parent / "plugins" / "usd"
    return str(bundled.resolve(strict=False)) if bundled.is_dir() else None


def _with_bundled(expected):
    """Append the bundled plugins/usd (if present) to an expected path list."""
    bundled = _bundled_usd_plugin_dir()
    if bundled and bundled not in expected:
        return [*expected, bundled]
    return expected


def test_ensure_pxr_plugin_path_uses_namespaced_env_for_ovphysx_lib(tmp_path, monkeypatch):
    from ovphysx.usd_version_check import _ensure_pxr_plugin_path

    root_dir = tmp_path
    lib_dir = root_dir / "lib"
    lib_dir.mkdir()
    lib_file = lib_dir / "libovphysx.so"
    lib_file.touch()
    plugins_dir = root_dir / "plugins" / "usd"
    plugins_dir.mkdir(parents=True)

    monkeypatch.setenv("OVPHYSX_LIB", str(lib_file))
    monkeypatch.delenv("PXR_PLUGINPATH_NAME", raising=False)
    monkeypatch.delenv("OV_PXR_PLUGINPATH_2511", raising=False)

    env_var = _ensure_pxr_plugin_path()

    assert env_var == "OV_PXR_PLUGINPATH_2511"
    assert os.environ.get("PXR_PLUGINPATH_NAME", "") == ""
    assert str(plugins_dir) in os.environ.get("OV_PXR_PLUGINPATH_2511", "")


def test_ensure_pxr_plugin_path_accepts_ovphysx_lib_directory(tmp_path, monkeypatch):
    from ovphysx.usd_version_check import _ensure_pxr_plugin_path

    root_dir = tmp_path
    lib_dir = root_dir / "lib"
    lib_dir.mkdir()
    plugins_dir = root_dir / "plugins" / "usd"
    plugins_dir.mkdir(parents=True)

    monkeypatch.setenv("OVPHYSX_LIB", str(lib_dir))
    monkeypatch.delenv("OV_PXR_PLUGINPATH_2511", raising=False)

    env_var = _ensure_pxr_plugin_path()

    assert env_var == "OV_PXR_PLUGINPATH_2511"
    assert os.environ.get("OV_PXR_PLUGINPATH_2511", "").split(os.pathsep) == _with_bundled(
        [str(plugins_dir.resolve(strict=False))]
    )


def test_ensure_pxr_plugin_path_supports_copied_runtime_layout(tmp_path, monkeypatch):
    from ovphysx.usd_version_check import _ensure_pxr_plugin_path

    root_dir = tmp_path
    app_dir = root_dir / "app"
    app_dir.mkdir()
    lib_file = app_dir / "libovphysx.so"
    lib_file.touch()
    (root_dir / "plugins").mkdir()
    plugins_dir = app_dir / "plugins" / "usd"
    plugins_dir.mkdir(parents=True)

    monkeypatch.setenv("OVPHYSX_LIB", str(lib_file))
    monkeypatch.delenv("OV_PXR_PLUGINPATH_2511", raising=False)

    env_var = _ensure_pxr_plugin_path()

    assert env_var == "OV_PXR_PLUGINPATH_2511"
    assert os.environ.get("OV_PXR_PLUGINPATH_2511", "").split(os.pathsep) == _with_bundled(
        [str(plugins_dir.resolve(strict=False))]
    )


def test_ensure_pxr_plugin_path_appends_without_duplicate(tmp_path, monkeypatch):
    from ovphysx.usd_version_check import _ensure_pxr_plugin_path

    root_dir = tmp_path
    lib_dir = root_dir / "lib"
    lib_dir.mkdir()
    lib_file = lib_dir / "libovphysx.so"
    lib_file.touch()
    plugins_dir = root_dir / "plugins" / "usd"
    plugins_dir.mkdir(parents=True)
    existing_dir = root_dir / "other_usd_plugins"
    existing_dir.mkdir()

    monkeypatch.setenv("OVPHYSX_LIB", str(lib_file))
    monkeypatch.setenv("OV_PXR_PLUGINPATH_2511", os.pathsep.join([str(existing_dir), str(plugins_dir)]))
    monkeypatch.delenv("PXR_PLUGINPATH_NAME", raising=False)

    env_var = _ensure_pxr_plugin_path()

    paths = os.environ.get("OV_PXR_PLUGINPATH_2511", "").split(os.pathsep)
    assert env_var == "OV_PXR_PLUGINPATH_2511"
    assert paths == _with_bundled([str(existing_dir), str(plugins_dir)])
    assert os.environ.get("PXR_PLUGINPATH_NAME", "") == ""


def test_check_usd_compatibility_skips_host_pxr_validation(tmp_path, monkeypatch):
    import ovphysx.usd_version_check as usd_version_check

    root_dir = tmp_path
    lib_dir = root_dir / "lib"
    lib_dir.mkdir()
    lib_file = lib_dir / "libovphysx.so"
    lib_file.touch()
    plugins_dir = root_dir / "plugins" / "usd"
    plugins_dir.mkdir(parents=True)

    host_pxr = MagicMock()
    host_pxr.Usd.GetVersion.return_value = "21.8"

    monkeypatch.setenv("OVPHYSX_LIB", str(lib_file))
    monkeypatch.delenv("PXR_PLUGINPATH_NAME", raising=False)
    monkeypatch.delenv("OV_PXR_PLUGINPATH_2511", raising=False)
    monkeypatch.setitem(sys.modules, "pxr", host_pxr)

    usd_version_check.check_usd_compatibility()

    assert host_pxr.Usd.GetVersion.call_count == 0
    assert os.environ.get("PXR_PLUGINPATH_NAME", "") == ""
    assert str(plugins_dir) in os.environ.get("OV_PXR_PLUGINPATH_2511", "")


def test_register_schema_paths_top_level_appends_and_dedupes(tmp_path, monkeypatch):
    import ovphysx.usd_version_check as usd_version_check

    import ovphysx

    root_dir = tmp_path
    lib_dir = root_dir / "lib"
    lib_dir.mkdir()
    lib_file = lib_dir / "libovphysx.so"
    lib_file.touch()
    plugins_dir = root_dir / "plugins" / "usd"
    plugins_dir.mkdir(parents=True)
    existing_dir = root_dir / "ovrtx_usd_plugins"
    existing_dir.mkdir()

    monkeypatch.setenv("OVPHYSX_LIB", str(lib_file))
    monkeypatch.setenv("OV_PXR_PLUGINPATH_2511", str(existing_dir))
    monkeypatch.delenv("PXR_PLUGINPATH_NAME", raising=False)

    ovphysx.register_schema_paths()
    ovphysx.register_schema_paths()

    paths = os.environ.get("OV_PXR_PLUGINPATH_2511", "").split(os.pathsep)
    assert paths == _with_bundled([str(existing_dir), str(plugins_dir.resolve(strict=False))])
    assert os.environ.get("PXR_PLUGINPATH_NAME", "") == ""


def test_register_schema_paths_failure_does_not_mark_done_and_can_retry(tmp_path, monkeypatch):
    import ovphysx.usd_version_check as usd_version_check

    import ovphysx

    # This test exercises the "no plugins/usd anywhere -> RuntimeError" failure path.
    # In a wheel / installed layout the package's own plugins/usd is ALWAYS a valid
    # fallback candidate, so registration cannot fail on a missing OVPHYSX_LIB root --
    # the scenario under test is unreachable. Skip it there.
    if _bundled_usd_plugin_dir() is not None:
        pytest.skip("bundled plugins/usd present (wheel layout): missing-root failure is unreachable")

    bad_root = tmp_path / "bad"
    bad_lib_dir = bad_root / "lib"
    bad_lib_dir.mkdir(parents=True)
    bad_lib_file = bad_lib_dir / "libovphysx.so"
    bad_lib_file.touch()

    monkeypatch.setenv("OVPHYSX_LIB", str(bad_lib_file))
    monkeypatch.setenv("OV_PXR_PLUGINPATH_2511", "sentinel")

    try:
        ovphysx.register_schema_paths()
    except RuntimeError as exc:
        assert "plugins/usd" in str(exc)
    else:
        raise AssertionError("register_schema_paths() should fail for a missing plugins/usd root")

    assert os.environ.get("OV_PXR_PLUGINPATH_2511") == "sentinel"

    good_root = tmp_path / "good"
    good_lib_dir = good_root / "lib"
    good_lib_dir.mkdir(parents=True)
    good_lib_file = good_lib_dir / "libovphysx.so"
    good_lib_file.touch()
    good_plugins_dir = good_root / "plugins" / "usd"
    good_plugins_dir.mkdir(parents=True)

    monkeypatch.setenv("OVPHYSX_LIB", str(good_lib_file))
    ovphysx.register_schema_paths()

    paths = os.environ.get("OV_PXR_PLUGINPATH_2511", "").split(os.pathsep)
    assert paths == ["sentinel", str(good_plugins_dir.resolve(strict=False))]


def test_register_schema_paths_does_not_native_bootstrap(tmp_path):
    root_dir = tmp_path
    lib_dir = root_dir / "lib"
    lib_dir.mkdir()
    lib_file = lib_dir / "libovphysx.so"
    lib_file.touch()
    plugins_dir = root_dir / "plugins" / "usd"
    plugins_dir.mkdir(parents=True)

    env = os.environ.copy()
    env["OVPHYSX_LIB"] = str(lib_file)
    env.pop("OV_PXR_PLUGINPATH_2511", None)
    env.pop("PXR_PLUGINPATH_NAME", None)
    script = """
import os
import sys
import ovphysx
ovphysx.register_schema_paths()
assert "ovphysx._bindings" not in sys.modules
assert getattr(ovphysx, "_native_bootstrapped") is False
assert os.environ.get("PXR_PLUGINPATH_NAME", "") == ""
assert os.environ.get("OV_PXR_PLUGINPATH_2511")
"""
    subprocess.run([sys.executable, "-c", script], env=env, check=True)
