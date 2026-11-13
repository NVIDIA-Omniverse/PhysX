# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Managed test environments and caches stay beneath the local build directory."""

import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

# Same value as test_build_clean.SUBPROCESS_TIMEOUT, for consistency; the two children here (a
# directory junction and a resolution-only cmake -P) take well under a second on every runner.
SUBPROCESS_TIMEOUT = 120


ROOT = Path(__file__).resolve().parents[2]


def _directory_link(link, target):
    link.parent.mkdir(parents=True, exist_ok=True)
    if os.name == "nt":
        result = subprocess.run(["cmd", "/c", "mklink", "/J", str(link), str(target)],
                                capture_output=True, text=True, timeout=SUBPROCESS_TIMEOUT)
        if result.returncode:
            pytest.skip(f"Cannot create test junction: {result.stderr}")
    else:
        link.symlink_to(target, target_is_directory=True)


@pytest.fixture
def runner(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location("cache_test_runner", ROOT / "scripts/run_uv_pytest.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    project = tmp_path / "project"
    tests = project / "tests/python_tests"
    tests.mkdir(parents=True)
    (tests / "pyproject.toml").write_text("", encoding="utf-8")
    monkeypatch.setattr(module, "__file__", str(project / "scripts/run_uv_pytest.py"))
    monkeypatch.setattr(sys, "argv", ["run_uv_pytest.py", str(tests), "test_example.py"])
    monkeypatch.delenv("OVPHYSX_BUILD_DIR", raising=False)
    calls = []
    def capture(cmd, **kwargs):
        # Retain only test-owned settings; failures must not print the process environment.
        kwargs["env"] = {key: kwargs["env"].get(key) for key in
                         ("UV_PROJECT_ENVIRONMENT", "UV_CACHE_DIR", "PYTHONPYCACHEPREFIX")}
        calls.append((cmd, kwargs))
        return 0

    monkeypatch.setattr(module.subprocess, "call", capture)
    return module, project, calls


@pytest.mark.parametrize("selection", ["_build", "_build/custom build", "absolute"])
def test_pytest_environment_and_caches_are_under_selected_build(runner, monkeypatch, selection):
    module, project, calls = runner
    selected = project / ("_build/absolute build" if selection == "absolute" else selection)
    monkeypatch.setenv("OVPHYSX_BUILD_DIR", str(selected) if selection == "absolute" else selection)
    assert module.main() == 0
    assert len(calls) == 2
    sync, run = calls
    assert sync[0] == ["uv", "sync", "--locked", "--no-install-project"]
    assert run[0][:4] == ["uv", "run", "--no-sync", "pytest"]
    for _, kwargs in calls:
        env = kwargs["env"]
        for key in ("UV_PROJECT_ENVIRONMENT", "UV_CACHE_DIR", "PYTHONPYCACHEPREFIX"):
            assert Path(env[key]).is_relative_to(selected), (key, env[key])
    assert "-o" in run[0]
    cache_option = next(value for value in run[0] if value.startswith("cache_dir="))
    assert Path(cache_option.removeprefix("cache_dir=")).is_relative_to(selected)
    assert not (project / "tests/python_tests/.venv").exists()


@pytest.mark.parametrize("selection", [".", "scripts", "../outside", "_build/../../outside", "C:", "C:foo"])
def test_pytest_rejects_outside_build_before_starting_uv(runner, monkeypatch, capsys, selection):
    module, _, calls = runner
    monkeypatch.setenv("OVPHYSX_BUILD_DIR", selection)
    assert module.main() == 2
    assert not calls
    assert "_build" in capsys.readouterr().err


@pytest.mark.parametrize("link_at_root", [True, False])
def test_pytest_rejects_build_links_without_touching_target(runner, monkeypatch, tmp_path, link_at_root):
    module, project, calls = runner
    target = tmp_path / "outside"
    target.mkdir()
    marker = target / "keep.txt"
    marker.write_text("keep", encoding="utf-8")
    link = project / "_build" if link_at_root else project / "_build/linked"
    _directory_link(link, target)
    monkeypatch.setenv("OVPHYSX_BUILD_DIR", str(link))
    assert module.main() == 2
    assert not calls
    assert marker.read_text(encoding="utf-8") == "keep"


@pytest.mark.parametrize("cache_path", ["venv/python_tests", "package_temp"])
def test_shared_cmake_guard_rejects_linked_cache_without_deleting(tmp_path, cache_path):
    cmake = shutil.which("cmake")
    if not cmake:
        pytest.skip("CMake is required by the cache-path check")
    project = tmp_path / "project"
    target = tmp_path / "outside"
    target.mkdir()
    marker = target / "keep.txt"
    marker.write_text("keep", encoding="utf-8")
    cache = project / "_build" / cache_path
    _directory_link(project / "_build" / Path(cache_path).parts[0], target)
    driver = tmp_path / "validate-cache.cmake"
    driver.write_text(
        'cmake_minimum_required(VERSION 3.16)\n'
        f'set(PROJECT_ROOT "{project.as_posix()}")\n'
        'set(BUILD_DIR "_build")\n'
        f'include("{ROOT.as_posix()}/scripts/build_directory.cmake")\n'
        f'ovphysx_require_build_path("{cache.as_posix()}")\n',
        encoding="utf-8",
    )
    result = subprocess.run([cmake, "-P", str(driver)], capture_output=True, text=True, timeout=SUBPROCESS_TIMEOUT)
    assert result.returncode != 0
    assert "symbolic links or junctions" in result.stdout + result.stderr
    assert marker.read_text(encoding="utf-8") == "keep"


def test_pytest_does_not_run_when_dependency_sync_fails(runner, monkeypatch):
    module, _, calls = runner

    def fail_sync(cmd, **kwargs):
        calls.append((cmd, {}))
        return 17

    monkeypatch.setattr(module.subprocess, "call", fail_sync)
    assert module.main() == 17
    assert len(calls) == 1
    assert calls[0][0][1] == "sync"
