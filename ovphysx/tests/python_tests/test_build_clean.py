# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Build cleanup removes generated outputs without following directory links."""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile

import pytest


ROOT = Path(__file__).resolve().parents[2]
TEST_AREA = ROOT / "_build" / "tests" / "build_clean"
CLEANUP_ROOTS = ("_build", "_install", "_repo", "_dist")
pytestmark = pytest.mark.skipif(shutil.which("cmake") is None, reason="cmake not on PATH")

# Budget for one child cmake / build.bat run: a runaway guard, not an expectation. Measured
# from junit on the 2026-10-20 runs: the 14 locked-file cases take 6 to 20 s on the GitHub
# Windows VMs and 0.6 to 7.3 s on the GitLab Windows runners (CMake retries a directory
# removal five times, 500 ms apart, over file-system work that varies several-fold between
# VMs; the two packaging_lock cases go through file(REMOVE), which does not retry), and
# the first clean of a session took 5 to 14 s green on GitLab. Four jobs failed with a 30 s
# budget, each with the clean still running when it was killed (whole-test times 31 to
# 42 s): GitHub nightly job 118646237423, GitLab jobs 497956074, 498543508 and 498543510.
# Linux runs the same calls in under a second.
#
# Keep this below faulthandler_timeout in pyproject.toml (300 s): that per-test watchdog
# starts before any child does, and if it fired first it would end the whole pytest
# process (no junit, remaining tests lost) while the child kept running. With this limit
# only the one test fails with TimeoutExpired. For the two build.bat calls the watchdog is
# still the last resort: on a timeout subprocess.run kills cmd.exe and then waits, with
# no limit, for the output pipes that the cmake.exe it started still holds.
SUBPROCESS_TIMEOUT = 120


def _assert_disposable(path):
    # A broken cleaner must only ever receive a disposable fixture source tree.
    assert path.resolve().is_relative_to(TEST_AREA.resolve())
    assert path.resolve() != TEST_AREA.resolve()


@pytest.fixture
def project(monkeypatch):
    monkeypatch.delenv("OVPHYSX_BUILD_DIR", raising=False)
    monkeypatch.delenv("OVPHYSX_USE_RELEASE_RUNTIME_DEPS", raising=False)
    TEST_AREA.mkdir(parents=True, exist_ok=True)
    sandbox = Path(tempfile.mkdtemp(prefix="case-", dir=TEST_AREA))
    project = sandbox / "source tree" / "ovphysx"
    _assert_disposable(project)
    (project / "scripts").mkdir(parents=True)
    for name in (
        "build.cmake", "build_common.cmake", "build_directory.cmake",
        "crossplatform_helpers.cmake", "host_path_utils.cmake", "host_toolchain.cmake",
        "validate_build_clean.cmake",
    ):
        source = ROOT / "scripts" / name
        if source.is_file():
            shutil.copy2(source, project / "scripts" / name)
    shutil.copytree(ROOT / "cmake", project / "cmake")
    shutil.copy2(ROOT / "build.bat", project / "build.bat")
    # The public source drop ships the native launcher as build.sh.
    native = ROOT / "build_native.sh"
    shutil.copy2(native if native.is_file() else ROOT / "build.sh", project / "build_native.sh")
    (project / "ovruntime").mkdir()
    (project / "ovruntime" / "CMakeLists.txt").write_text("# fixture\n", encoding="utf-8")
    (project / "source.txt").write_text("keep source\n", encoding="utf-8")
    yield project
    _assert_disposable(sandbox)
    shutil.rmtree(sandbox)


def _artifact(directory, name="CMakeCache.txt"):
    _assert_disposable(directory)
    directory.mkdir(parents=True, exist_ok=True)
    artifact = directory / name
    artifact.write_text("preserve these bytes\n", encoding="utf-8")
    return artifact


def _outside_artifacts(project):
    directories = (
        "scripts", "ovruntime/ordinary", "nested/ordinary", "_build-other",
        ".pytest_cache", "nested/.pytest_cache", "../physx/_build",
    )
    paths = [_artifact(project / directory, "keep.txt") for directory in directories]
    paths.append(_artifact(project / "packaging_lock", "linux.json"))
    paths.append(_artifact(project / "packaging_lock/nested", "linux_new.json"))
    paths.extend((project / "source.txt", project / "ovruntime/CMakeLists.txt"))
    return {path: path.read_bytes() for path in paths}


def _legacy_outputs(project):
    directories = (
        ".venv", "__pycache__", "nested/_build", "nested/_install",
        "nested/_repo", "nested/_dist", "nested/.venv", "nested/__pycache__",
        "ovruntime/_build", "ovruntime/_install", "ovruntime/_repo",
        "ovruntime/_dist", "ovruntime/_downloads",
    )
    outputs = [project / directory for directory in directories]
    for directory in outputs:
        _artifact(directory, "stale.txt")
    outputs.append(_artifact(project / "packaging_lock", "linux_new.json"))
    return outputs


def _assert_preserved(artifacts):
    for path, contents in artifacts.items():
        assert path.is_file(), f"Cleanup removed preserved data: {path}"
        assert path.read_bytes() == contents, f"Cleanup modified preserved data: {path}"


def _clean(project, *args):
    _assert_disposable(project)
    return subprocess.run(
        ["cmake", "-DJOBS=1", "-DCLEAN_BUILD=ON", "-DCLEAN_ONLY=ON", *args,
         "-P", str(project / "scripts" / "build.cmake")],
        cwd=project.parent, capture_output=True, text=True, timeout=SUBPROCESS_TIMEOUT,
    )


def _resolve(project, *args):
    # Resolution-only: no deletion, dependency fetch, configure, or target mkdir.
    # Real roots, drive-relative paths and source directories are tested here only.
    probe = project.parent / "resolve.cmake"
    probe.write_text(
        'cmake_minimum_required(VERSION 3.16)\n'
        f'set(PROJECT_ROOT [==[{project.as_posix()}]==])\n'
        f'include([==[{project.as_posix()}/scripts/build_directory.cmake]==])\n'
        'message(STATUS "RESOLVED_BUILD_PATH=${BUILD_PATH}")\n',
        encoding="utf-8",
    )
    return subprocess.run(
        ["cmake", *args, "-P", str(probe)], cwd=project.parent,
        capture_output=True, text=True, timeout=SUBPROCESS_TIMEOUT,
    )


@pytest.mark.parametrize("source,absolute", [
    ("default", False), ("environment", False), ("environment", True),
    ("argument", False), ("argument", True),
    ("argument_over_environment", False), ("argument_over_environment", True),
])
def test_clean_removes_generated_outputs(project, monkeypatch, source, absolute):
    selected = project / "_build" if source == "default" else project / "_build" / "custom cache"
    value = str(selected) if absolute else "_build/custom cache"
    args = []
    if source == "environment":
        monkeypatch.setenv("OVPHYSX_BUILD_DIR", value)
    elif source.startswith("argument"):
        args.append(f"-DBUILD_DIR={value}")
        if source == "argument_over_environment":
            # Explicit -D takes priority even when the environment is invalid.
            monkeypatch.setenv("OVPHYSX_BUILD_DIR", "outside cache")
    resolved = _resolve(project, *args)
    assert resolved.returncode == 0, resolved.stdout + resolved.stderr
    assert f"RESOLVED_BUILD_PATH={selected.as_posix()}" in resolved.stdout

    _artifact(selected)
    _artifact(project / "_build" / "other variant", "stale.o")
    for name in CLEANUP_ROOTS[1:]:
        _artifact(project / name, "stale.txt")
    preserved = _outside_artifacts(project)
    legacy_outputs = _legacy_outputs(project)
    result = _clean(project, *args)
    assert result.returncode == 0, result.stdout + result.stderr
    assert all(not (project / name).exists() for name in CLEANUP_ROOTS)
    assert all(not path.exists() for path in legacy_outputs)
    _assert_preserved(preserved)


def test_clean_accepts_missing_build_directory(project):
    for name in CLEANUP_ROOTS[1:]:
        _artifact(project / name, "stale.txt")
    preserved = _outside_artifacts(project)
    legacy_outputs = _legacy_outputs(project)
    result = _clean(project, "-DBUILD_DIR=_build/not yet built")
    assert result.returncode == 0, result.stdout + result.stderr
    assert all(not (project / name).exists() for name in CLEANUP_ROOTS)
    assert all(not path.exists() for path in legacy_outputs)
    _assert_preserved(preserved)


@pytest.mark.parametrize("value", [
    "", ".", "..", "../..", "nested/..", "scripts", "ovruntime", "../physx",
    "custom cache", "_build-other", "_build/../scripts", "_build/../../other",
    "/", "C:/", "D:", "C:foo", "Z:/", "//server/share",
])
def test_build_directory_rejects_outside_paths_without_cleaning(project, value):
    result = _resolve(project, f"-DBUILD_DIR={value}")
    assert result.returncode != 0, f"Accepted unsupported build directory {value!r}:\n{result.stdout}"
    assert "_build" in result.stdout + result.stderr
    assert "RESOLVED_BUILD_PATH=" not in result.stdout


def test_build_directory_rejects_outside_environment_path(project, monkeypatch):
    monkeypatch.setenv("OVPHYSX_BUILD_DIR", str(project.parent / "external cache"))
    result = _resolve(project)
    assert result.returncode != 0, result.stdout + result.stderr
    assert "RESOLVED_BUILD_PATH=" not in result.stdout


def _directory_link(link, target):
    _assert_disposable(link)
    _assert_disposable(target)
    link.parent.mkdir(parents=True, exist_ok=True)
    try:
        link.symlink_to(target, target_is_directory=True)
    except OSError:
        if os.name != "nt":
            pytest.skip("Directory symlinks are unavailable")
        # Directory junctions exercise the same Windows reparse-point boundary.
        result = subprocess.run(
            ["cmd", "/c", "mklink", "/J", str(link), str(target)],
            capture_output=True, text=True, timeout=SUBPROCESS_TIMEOUT,
        )
        if result.returncode != 0:
            pytest.skip(f"Directory links are unavailable: {result.stdout}{result.stderr}")


@pytest.mark.parametrize("location,target_location", [
    ("_build", "outside"), ("_build/linked", "outside"),
    ("_build/linked", "_build/real"),
])
def test_build_directory_rejects_link_components(project, location, target_location):
    target = project / target_location
    sentinel = _artifact(target, "keep.txt")
    _directory_link(project / location, target)
    result = _resolve(project, f"-DBUILD_DIR={location}/not yet built")
    assert result.returncode != 0, result.stdout + result.stderr
    assert "RESOLVED_BUILD_PATH=" not in result.stdout
    assert sentinel.read_text(encoding="utf-8") == "preserve these bytes\n"


@pytest.mark.parametrize("cleanup_root", CLEANUP_ROOTS)
def test_clean_does_not_follow_links_inside_output_directories(project, cleanup_root):
    target = project.parent / "external cache"
    sentinel = _artifact(target, "keep.txt")
    _directory_link(project / cleanup_root / "linked", target)
    result = _clean(project)
    assert result.returncode == 0, result.stdout + result.stderr
    assert all(not (project / name).exists() for name in CLEANUP_ROOTS)
    assert sentinel.read_text(encoding="utf-8") == "preserve these bytes\n"


@pytest.mark.parametrize("mode", ["--clean", "--rebuild"])
def test_native_wrapper_cleans_generated_outputs_before_fetching(project, monkeypatch, mode):
    monkeypatch.setenv("OVPHYSX_BUILD_DIR", "_build/custom cache")
    caches = [_artifact(project / "_build" / name)
              for name in ("custom cache", "other variant")]
    caches.extend(_artifact(project / name, "stale.txt") for name in CLEANUP_ROOTS[1:])
    preserved = _outside_artifacts(project)
    legacy_outputs = _legacy_outputs(project)
    if os.name == "nt":
        wrapper = [str(project / "build.bat")]
        dependency_script = project / "ovruntime" / "pull_dependencies.bat"
        dependency_script.write_text("@echo off\necho CLEAN_TEST_FETCH_BOUNDARY\nexit /b 42\n")
    else:
        wrapper = ["bash", str(project / "build_native.sh")]
        dependency_script = project / "ovruntime" / "pull_dependencies.sh"
        dependency_script.write_text("#!/bin/sh\necho CLEAN_TEST_FETCH_BOUNDARY\nexit 42\n")
        dependency_script.chmod(0o755)

    _assert_disposable(project)
    result = subprocess.run(
        [*wrapper, mode], cwd=project.parent,
        capture_output=True, text=True, timeout=SUBPROCESS_TIMEOUT,
    )
    output = result.stdout + result.stderr
    if mode == "--clean":
        assert result.returncode == 0, output
        assert "CLEAN_TEST_FETCH_BOUNDARY" not in output
    else:
        # Stop at fetching to verify rebuild ordering without compiling/network.
        assert result.returncode != 0, output
        assert "CLEAN_TEST_FETCH_BOUNDARY" in output, output
    assert all(not cache.exists() for cache in caches), output
    assert all(not path.exists() for path in legacy_outputs), output
    assert all(not (project / name).exists() for name in CLEANUP_ROOTS[1:]), output
    _assert_preserved(preserved)

@pytest.mark.parametrize("location", [".", "../direct cache", "_build-other"])
def test_top_level_configure_rejects_outside_build_directory(project, location):
    for name in ("CMakeLists.txt", "VERSION"):
        shutil.copy2(ROOT / name, project / name)
    destination = (project / location).resolve()
    _assert_disposable(destination)
    preserved = _outside_artifacts(project)
    missing_compiler = project / "missing-compiler"
    result = subprocess.run(
        ["cmake", "-S", str(project), "-B", str(destination), "-G", "Ninja",
         f"-DCMAKE_MAKE_PROGRAM={shutil.which('cmake')}",
         f"-DCMAKE_C_COMPILER={missing_compiler}",
         f"-DCMAKE_CXX_COMPILER={missing_compiler}"],
        capture_output=True, text=True, timeout=SUBPROCESS_TIMEOUT,
    )
    output = result.stdout + result.stderr
    assert result.returncode != 0, output
    # An unsupported output directory must fail before compiler detection/fetch.
    assert "_build" in output, output
    assert "CMAKE_C_COMPILER:" not in output, output
    assert "CMAKE_CXX_COMPILER:" not in output, output
    _assert_preserved(preserved)

@pytest.mark.skipif(os.name != "nt", reason="Windows open files deny deletion")
@pytest.mark.parametrize("mode", ["--clean", "--rebuild"])
@pytest.mark.parametrize("directory,filename", [
    *((root, "CMakeCache.txt") for root in CLEANUP_ROOTS),
    ("nested/.venv", "keep.txt"), ("ovruntime/_downloads", "keep.txt"),
    ("packaging_lock", "linux_new.json"),
])
def test_native_wrapper_stops_when_output_is_locked(project, mode, directory, filename):
    cache = _artifact(project / directory, filename)
    preserved = _outside_artifacts(project)
    dependency_script = project / "ovruntime" / "pull_dependencies.bat"
    dependency_script.write_text("@echo off\necho CLEAN_TEST_FETCH_BOUNDARY\nexit /b 42\n")

    _assert_disposable(project)
    # Python's ordinary Windows file handle does not share delete access.
    # Keep it open while the child attempts cleanup, then release it for teardown.
    with cache.open("r", encoding="utf-8"):
        result = subprocess.run(
            [str(project / "build.bat"), mode], cwd=project.parent,
            capture_output=True, text=True, timeout=SUBPROCESS_TIMEOUT,
        )
        assert cache.is_file(), "Fixture file did not block deletion on this host"
    output = result.stdout + result.stderr
    assert result.returncode != 0, output
    assert "CLEAN_TEST_FETCH_BOUNDARY" not in output, output
    _assert_preserved(preserved)


@pytest.mark.parametrize("cleanup_root", (*CLEANUP_ROOTS, "ovruntime", "packaging_lock"))
def test_clean_rejects_root_link_before_removing_other_roots(project, cleanup_root):
    markers = [_artifact(project / name, "keep.txt")
               for name in CLEANUP_ROOTS if name != cleanup_root]
    target = project.parent / "external output"
    markers.append(_artifact(target, "keep.txt"))
    markers.append(_artifact(target / "_build", "keep.txt"))
    if cleanup_root == "ovruntime":
        source = project / "ovruntime" / "CMakeLists.txt"
        (target / "CMakeLists.txt").write_bytes(source.read_bytes())
        source.unlink()
        (project / "ovruntime").rmdir()
    _directory_link(project / cleanup_root, target)
    preserved = {path: path.read_bytes() for path in markers}

    result = _clean(project)
    assert result.returncode != 0, result.stdout + result.stderr
    _assert_preserved(preserved)


@pytest.mark.parametrize("location", ["nested/.venv", "ovruntime/_downloads"])
def test_clean_unlinks_recognized_output_without_following_target(project, location):
    target = project.parent / "external output"
    marker = _artifact(target / "_build", "keep.txt")
    link = project / location
    _directory_link(link, target)
    result = _clean(project)
    assert result.returncode == 0, result.stdout + result.stderr
    assert not os.path.lexists(link)
    assert marker.read_text(encoding="utf-8") == "preserve these bytes\n"


def test_clean_skips_unrecognized_linked_subtree(project):
    target = project.parent / "external subtree"
    markers = [_artifact(target / name, "keep.txt")
               for name in ("_build", ".venv", "nested/__pycache__")]
    preserved = {path: path.read_bytes() for path in markers}
    link = project / "nested/ordinary"
    _directory_link(link, target)
    build_marker = _artifact(project / "_build")
    result = _clean(project)
    assert result.returncode == 0, result.stdout + result.stderr
    assert not build_marker.exists()
    assert link.is_dir()
    _assert_preserved(preserved)
