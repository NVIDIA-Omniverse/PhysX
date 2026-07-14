# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
"""
Cross-platform build script for the umbrella project.

Usage:
    python tools/build.py [flags]

Flags:
    -c, --clean        Clean artifacts only (no build)
    -x, --rebuild      Clean then build
    -d, --debug        Build debug configuration
    -r, --release      Build release configuration
    -t, --target NAME  Build a specific CMake target
    -g, --generate     Configure only (no build)
    -n, --no-docker    Skip Docker, build on host (Linux)
    --test-venv        Set up Python test environment only, then quit

If neither -d nor -r is given, builds release only.
"""

import argparse
import os
import platform
import shutil
import stat
import subprocess
import sys
import time
from pathlib import Path


def _get_cpu_count() -> int:
    """Return CPU count respecting cgroup limits (Kubernetes / Docker).

    On containerised runners ``os.cpu_count()`` returns the *host* CPU count,
    which can vastly exceed the container's actual allocation.  Read the cgroup
    quota instead, falling back to ``os.cpu_count()`` when no limit is set.

    NOTE: keep in sync with omni/ovexts/tools/build.py::_get_cpu_count().
    """
    try:  # cgroup v2
        text = Path("/sys/fs/cgroup/cpu.max").read_text()
        quota, period = text.strip().split()
        if quota != "max":
            # Ceiling division so fractional allocations (e.g. 0.5 CPU) get at least 1.
            cpus = -(-int(quota) // int(period))
            if cpus > 0:
                return cpus
    except (FileNotFoundError, ValueError, OSError):
        pass
    try:  # cgroup v1
        quota = int(Path("/sys/fs/cgroup/cpu/cpu.cfs_quota_us").read_text().strip())
        period = int(Path("/sys/fs/cgroup/cpu/cpu.cfs_period_us").read_text().strip())
        if quota > 0 and period > 0:
            cpus = -(-quota // period)
            if cpus > 0:
                return cpus
    except (FileNotFoundError, ValueError, OSError):
        pass
    return os.cpu_count() or 1


IS_WINDOWS = sys.platform == "win32"
IS_AARCH64 = not IS_WINDOWS and platform.machine() == "aarch64"


def detect_linux_compiler_family() -> str:
    compiler = os.environ.get("CXX") or os.environ.get("CC") or "c++"
    exe_name = Path(compiler).name.lower()
    if "clang" in exe_name:
        return "clang"
    try:
        out = subprocess.run([compiler, "--version"], capture_output=True, text=True, check=False).stdout.lower()
    except OSError:
        out = ""
    return "clang" if "clang" in out else "gcc"

if IS_WINDOWS:
    BUILD_PLATFORM   = "windows-x86_64"
    PACKMAN_PLATFORM = "windows-x86_64"
    PACKMAN          = Path("tools/packman/packman.cmd")
    VENV_PYTHON      = Path("_build/.venv/Scripts/python.exe")
    PACKMAN_PYTHON   = Path("_build/target-deps/python/python.exe")
    COMPILER_DIR     = Path("_compiler/vc17win64")
elif IS_AARCH64:
    BUILD_PLATFORM   = "linux-aarch64"
    PACKMAN_PLATFORM = "manylinux_2_35_aarch64"
    PACKMAN          = Path("tools/packman/packman")
    VENV_PYTHON      = Path("_build/.venv/bin/python")
    PACKMAN_PYTHON   = Path("_build/target-deps/python/python")
    COMPILER_DIR     = Path(f"_compiler/{detect_linux_compiler_family()}-linux-aarch64")
else:
    BUILD_PLATFORM   = "linux-x86_64"
    PACKMAN_PLATFORM = "manylinux_2_35_x86_64"
    PACKMAN          = Path("tools/packman/packman")
    VENV_PYTHON      = Path("_build/.venv/bin/python")
    PACKMAN_PYTHON   = Path("_build/target-deps/python/python")
    COMPILER_DIR     = Path(f"_compiler/{detect_linux_compiler_family()}-linux-x86_64")

BUILD_DIR   = Path(f"_build/{BUILD_PLATFORM}")
INSTALL_DIR = Path("_install/umbrella")
VENV_DIR    = Path("_build/.venv")


def fmt_elapsed(seconds: float) -> str:
    s = int(seconds)
    h, s = divmod(s, 3600)
    m, s = divmod(s, 60)
    if h:
        return f"{h}h {m:02d}m {s:02d}s"
    if m:
        return f"{m}m {s:02d}s"
    return f"{s}s"


def run(cmd, *, cwd, docker_prefix=None, env=None):
    """Run a command (optionally wrapped in Docker), raising SystemExit on failure."""
    full_cmd = (docker_prefix or []) + [str(c) for c in cmd]
    print(f"+ {' '.join(full_cmd)}")
    merged_env = os.environ.copy()
    if env:
        merged_env.update(env)
    result = subprocess.run(full_cmd, cwd=cwd, env=merged_env)
    if result.returncode != 0:
        sys.exit(result.returncode)


def make_docker_prefix(root: Path) -> list:
    omni_dir      = str(root.parent)
    packman_cache = os.environ.get("PM_PACKAGES_ROOT", str(Path.home() / ".cache/packman"))
    uid_gid       = f"{os.getuid()}:{os.getgid()}"
    return [
        "docker", "run", "--rm",
        "-v", f"{omni_dir}:{omni_dir}",
        "-v", f"{packman_cache}:{packman_cache}:ro",
        "-w", str(root),
        "-u", uid_gid,
        DOCKER_IMAGE,
    ]


def find_cmake(root: Path) -> str:
    """Prefer the packman-provided cmake; fall back to system cmake if not found."""
    suffix = ".exe" if IS_WINDOWS else ""
    packman_cmake = root / f"_build/target-deps/cmake/bin/cmake{suffix}"
    if packman_cmake.is_file():
        return str(packman_cmake)
    print(f"Warning: packman cmake not found at {packman_cmake}, falling back to system cmake")
    return "cmake"


def get_local_python_env(root: Path) -> dict:
    """Use a repo-local temp/cache area so Python bootstrap does not depend on user TEMP permissions."""
    tmp_dir = root / "_build" / "tmp"
    pip_cache_dir = root / "_build" / "pip-cache"
    tmp_dir.mkdir(parents=True, exist_ok=True)
    pip_cache_dir.mkdir(parents=True, exist_ok=True)
    return {
        "TMP": str(tmp_dir),
        "TEMP": str(tmp_dir),
        "TMPDIR": str(tmp_dir),
        "PIP_CACHE_DIR": str(pip_cache_dir),
    }


def write_pyvenv_cfg(root: Path, venv_dir: Path) -> None:
    """Recreate pyvenv.cfg for a partially-created venv."""
    base_python = root / PACKMAN_PYTHON
    python_env = get_local_python_env(root)
    version = subprocess.run(
        [base_python, "-c", "import sys; print(sys.version.split()[0])"],
        capture_output=True,
        text=True,
        check=True,
        env={**os.environ, **python_env},
    ).stdout.strip()
    executable = subprocess.run(
        [base_python, "-c", "import sys; print(sys.executable)"],
        capture_output=True,
        text=True,
        check=True,
        env={**os.environ, **python_env},
    ).stdout.strip()
    (venv_dir / "pyvenv.cfg").write_text(
        f"home = {base_python.parent}\n"
        "include-system-site-packages = false\n"
        f"version = {version}\n"
        f"executable = {executable}\n"
        f"command = {base_python} -m venv {venv_dir}\n",
        encoding="utf-8",
    )


def setup_python_venv(root: Path) -> None:
    """Create the Python test venv if it does not already exist."""
    venv_dir = root / VENV_DIR
    pyvenv_cfg = venv_dir / "pyvenv.cfg"
    venv_python = root / VENV_PYTHON
    python_env = get_local_python_env(root)

    def _has_pytest() -> bool:
        if not venv_python.exists():
            return False
        result = subprocess.run(
            [venv_python, "-c", "import pytest"],
            cwd=root,
            env={**os.environ, **python_env},
            capture_output=True,
            text=True,
        )
        return result.returncode == 0

    if pyvenv_cfg.exists() and _has_pytest():
        print("Python test environment already exists.")
        return

    if venv_python.exists():
        print("Repairing incomplete Python test environment...")
        write_pyvenv_cfg(root, venv_dir)
        run([venv_python, "-m", "ensurepip", "--upgrade"], cwd=root, env=python_env)
        run([venv_python, "-m", "pip", "install", "--quiet", "pytest", "numpy", "warp-lang", "debugpy"], cwd=root, env=python_env)
        return

    if venv_dir.exists():
        print("Recreating incomplete Python test environment...")
        shutil.rmtree(venv_dir, ignore_errors=True)
        print("Creating Python test environment...")
        run([root / PACKMAN_PYTHON, "-m", "venv", venv_dir], cwd=root, env=python_env)
        run([root / VENV_PYTHON, "-m", "pip", "install", "--quiet", "pytest", "numpy", "warp-lang", "debugpy"], cwd=root, env=python_env)
        return

    print("Creating Python test environment...")
    run([root / PACKMAN_PYTHON, "-m", "venv", venv_dir], cwd=root, env=python_env)
    run([root / VENV_PYTHON, "-m", "pip", "install", "--quiet", "pytest", "numpy", "warp-lang", "debugpy"], cwd=root, env=python_env)


def generate_test_scripts(root: Path) -> None:
    """Generate test_unit and test_python wrapper scripts into _build/ for the current platform."""
    out = root / "_build"
    out.mkdir(parents=True, exist_ok=True)
    if IS_WINDOWS:
        for name, content in [
            ("test_unit.bat",
             "@echo off\r\nsetlocal enableextensions\r\n"
             '"%~dp0target-deps\\cmake\\bin\\ctest.exe"'
             f' --test-dir "%~dp0..\\{COMPILER_DIR}" -C Release -V\r\n'),
            ("test_python.bat",
             "@echo off\r\nsetlocal enableextensions\r\n"
             '"%~dp0.venv\\Scripts\\python.exe" -m pytest "%~dp0..\\tests\\python" --junitxml="%~dp0..\\pytest_results.xml"\r\n'),
        ]:
            p = out / name
            if p.exists():
                p.unlink()
            p.write_text(content, encoding="utf-8")
    else:
        unit = (
            "#!/bin/bash\nset -e\n"
            "SCRIPT_DIR=\"$(cd \"$(dirname \"${BASH_SOURCE[0]}\")\" && pwd)\"\n"
            'CTEST="$SCRIPT_DIR/target-deps/cmake/bin/ctest"\n'
            '[ -x "$CTEST" ] || CTEST=ctest\n'
            f'"$CTEST" --test-dir "$SCRIPT_DIR/../{COMPILER_DIR}/build-release" -C Release -V\n'
        )
        python = (
            "#!/bin/bash\nset -e\n"
            "SCRIPT_DIR=\"$(cd \"$(dirname \"${BASH_SOURCE[0]}\")\" && pwd)\"\n"
            '"$SCRIPT_DIR/.venv/bin/python" -m pytest "$SCRIPT_DIR/../tests/python" --junitxml="$SCRIPT_DIR/../pytest_results.xml"\n'
        )
        for name, content in [("test_unit.sh", unit), ("test_python.sh", python)]:
            p = out / name
            if p.exists():
                p.unlink()
            p.write_text(content, encoding="utf-8")
            try:
                p.chmod(0o755)
            except OSError:
                pass


def pull_dependencies(root: Path) -> None:
    """Pull all packman dependencies"""
    print("Pulling Physics Umbrella dependencies with packman...")
    packman = root / PACKMAN
    p = PACKMAN_PLATFORM

    deps = [
        # core target deps (cmake, USD, PhysX, etc.)
        ["deps/target-deps.packman.xml", "-p", p],
        # kit-sdk (kit-kernel) for release config
        ["deps/kit-sdk-target-deps.packman.xml", "-p", p, "-t", f"platform_target_abi={p}", "-t", "config=release"],
        # kit-sdk imports (carb_sdk_plugins, pybind11, python, doctest)
        ["deps/kit-sdk-target-deps-import.packman.xml", "-p", p, "-t", f"platform_target_abi={p}", "-t", "config=release"],
    ]

    for args in deps:
        run([packman, "pull", *args], cwd=root)

    print("\nDependencies pulled successfully!\n")


def setup_msvc(root: Path, compiler_dir: Path) -> list:
    """Set up packman MSVC/WinSDK and return cmake -D args. Mirrors ovruntime/build.bat."""
    msvc   = root / "_build/host-deps/msvc"
    winsdk = root / "_build/host-deps/winsdk"

    # Host-toolchain mode: when the packman MSVC package isn't present (e.g. removed
    # from host-deps for the open-source release), skip the packman setup and let
    # CMake's VS generator auto-detect the locally-installed MSVC/WinSDK.
    if not msvc.exists():
        print("  packman MSVC absent — using host-installed MSVC/WinSDK toolchain")
        return []

    tools_dir = sorted((msvc / "VC/Tools/MSVC").glob("*"))[-1]
    ver = tools_dir.name
    print(f"  MSVC tools: {ver}")

    if (winsdk / "Include/ucrt").exists():
        sdk_ver, inc, lib, sdk_bin = "", winsdk / "Include", winsdk / "Lib", winsdk / "bin/x64"
    else:
        sdk_ver = sorted((winsdk / "Include").glob("*"))[-1].name
        inc, lib, sdk_bin = winsdk / f"Include/{sdk_ver}", winsdk / f"Lib/{sdk_ver}", winsdk / f"bin/{sdk_ver}/x64"

    bin_dir  = tools_dir / "bin/Hostx64/x64"
    includes = f"{tools_dir}\\include;{inc}\\ucrt;{inc}\\um;{inc}\\shared"
    libs     = f"{tools_dir}\\lib\\x64;{lib}\\ucrt\\x64;{lib}\\um\\x64"
    os.environ["PATH"] = f"{bin_dir};{sdk_bin};{os.environ['PATH']}"

    # Patch NVIDIA.ImportBefore.props (ships with stale version/paths)
    p = msvc / "MSBuild/Current/Imports/Microsoft.Common.Props/ImportBefore/NVIDIA.ImportBefore.props"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(f'<Project xmlns="http://schemas.microsoft.com/developer/msbuild/2003"><PropertyGroup>'
                 f'<DisableRegistryUse>true</DisableRegistryUse><VCToolsVersion>{ver}</VCToolsVersion>'
                 f'<VCInstallDir_170>{msvc}\\VC\\</VCInstallDir_170>'
                 f'<VCToolsInstallDir_170>{tools_dir}\\</VCToolsInstallDir_170>'
                 f'</PropertyGroup></Project>')

    # Also write ZZ_PackmanFix.ImportBefore.props so it overrides any stale version written
    # by a previous ovruntime build into the same shared packman MSVC cache directory.
    # (ovruntime writes this file with its own absolute paths; since both projects symlink
    # to the same packman MSVC package, whichever ran last wins alphabetically.)
    fix = msvc / "MSBuild/Current/Imports/Microsoft.Common.Props/ImportBefore/ZZ_PackmanFix.ImportBefore.props"
    fix.write_text(f'<Project xmlns="http://schemas.microsoft.com/developer/msbuild/2003"><PropertyGroup>'
                   f'<VCToolsVersion>{ver}</VCToolsVersion>'
                   f'<VCInstallDir_170>{msvc}\\VC\\</VCInstallDir_170>'
                   f'<VCToolsInstallDir_170>{tools_dir}\\</VCToolsInstallDir_170>'
                   f'</PropertyGroup></Project>')

    # VCToolsVersion props (cmake needs this, not shipped with packman MSVC)
    vtv = msvc / f"VC/Auxiliary/Build/{ver}"
    vtv.mkdir(parents=True, exist_ok=True)
    (vtv / f"Microsoft.VCToolsVersion.{ver}.props").write_text(
        f'<?xml version="1.0"?><Project ToolsVersion="4.0" xmlns="http://schemas.microsoft.com/developer/msbuild/2003">'
        f'<PropertyGroup><VCToolsVersion>{ver}</VCToolsVersion></PropertyGroup></Project>')

    # Directory.Build.props: include/lib/exe paths for cmake's VCTargetsPath detection
    sdk_ver_tag = f"<WindowsTargetPlatformVersion>{sdk_ver}</WindowsTargetPlatformVersion>" if sdk_ver else ""
    compiler_dir.mkdir(parents=True, exist_ok=True)
    (compiler_dir / "Directory.Build.props").write_text(
        f'<Project><PropertyGroup><VCToolsVersion>{ver}</VCToolsVersion>'
        f'<WindowsSDKDir>{winsdk}\\</WindowsSDKDir>{sdk_ver_tag}'
        f'<IncludePath>{includes};$(IncludePath)</IncludePath>'
        f'<LibraryPath>{libs};$(LibraryPath)</LibraryPath>'
        f'<ExecutablePath>{bin_dir};{sdk_bin};$(ExecutablePath)</ExecutablePath>'
        f'</PropertyGroup></Project>')

    # cmake 3.25.1 embeds CMAKE_GENERATOR_INSTANCE with ,version= suffix — create junction so path resolves
    vs_ver = subprocess.run(
        ["powershell", "-NoProfile", "-Command",
         f"(Get-Item '{msvc}\\MSBuild\\Current\\Bin\\MSBuild.exe').VersionInfo.FileVersion"],
        capture_output=True, text=True).stdout.strip()
    link = root / f"_build/host-deps/msvc,version={vs_ver}"
    if not link.exists():
        subprocess.run(f'mklink /J "{link}" "{msvc}"', shell=True, check=True)

    return [
        f"-DCMAKE_GENERATOR_INSTANCE={msvc},version={vs_ver}",
        f"-DCMAKE_VS_PLATFORM_TOOLSET_VERSION={ver}",
        f"-DCMAKE_VS_SDK_INCLUDE_DIRECTORIES={includes.replace(';', chr(92) + ';')}",
        f"-DCMAKE_VS_SDK_LIBRARY_DIRECTORIES={libs.replace(';', chr(92) + ';')}",
    ]


def main():
    parser = argparse.ArgumentParser(description="Build umbrella (Windows/Linux)")
    parser.add_argument("-c", "--clean",      dest="clean",         action="store_true", help="Clean artifacts only (no build)")
    parser.add_argument("-x", "--rebuild",    dest="rebuild",       action="store_true", help="Clean then build")
    parser.add_argument("-d", "--debug",      dest="debug",         action="store_true", help="Build debug configuration")
    parser.add_argument("-r", "--release",    dest="release",       action="store_true", help="Build release configuration")
    parser.add_argument("-t", "--target",     dest="target",        default="",          help="Build a specific CMake target")
    parser.add_argument("-g", "--generate",   dest="generate_only", action="store_true", help="Configure only (no build)")
    parser.add_argument("-n", "--no-docker",  dest="no_docker",     action="store_true", help="Skip Docker, build on host (Linux)")
    parser.add_argument("--test-venv",        dest="setup_venv",    action="store_true", help="Set up Python test environment only, then quit")
    args = parser.parse_args()

    root        = Path(__file__).resolve().parent.parent  # umbrella root
    total_start = time.monotonic()

    # --- Clean ---
    if args.clean or args.rebuild:
        print("Cleaning build artifacts...")
        for d in sorted(root.glob("_*")):
            if d.is_dir():
                print(f"  Removing {d.name}")
                shutil.rmtree(d, ignore_errors=True)
                if d.exists():
                    def _force_remove(_func, _path, _exc_info):
                        try:
                            os.chmod(_path, stat.S_IWRITE)
                            os.unlink(_path)
                        except OSError:
                            pass
                    shutil.rmtree(d, onerror=_force_remove)

    # Clean-only: stop here
    if args.clean and not args.rebuild:
        print("Clean complete.")
        return

    # --- Default: release only ---
    do_debug   = args.debug
    do_release = args.release or (not args.debug)

    # --- Docker prefix (Linux only, auto-disabled if docker CLI not found) ---
    use_docker = False
    docker_prefix = make_docker_prefix(root) if use_docker else None
    if use_docker:
        print(f"Building inside Docker ({DOCKER_IMAGE})...")

    # --- Pull dependencies (always on host) ---
    pull_dependencies(root)
    if args.setup_venv:
        setup_python_venv(root)
        return

    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    COMPILER_DIR.mkdir(parents=True, exist_ok=True)
    cmake = find_cmake(root)
    output_base = str(root / BUILD_DIR)

    if IS_WINDOWS:
        if (root / "deps/host-deps.packman.xml").exists():
            run([root / PACKMAN, "pull", "deps/host-deps.packman.xml", "-p", PACKMAN_PLATFORM], cwd=root)
        msvc_cmake_args = setup_msvc(root, COMPILER_DIR)
        # Delete stale cmake cache — source path baked into it may differ across runners
        (COMPILER_DIR / "CMakeCache.txt").unlink(missing_ok=True)
        # Visual Studio multi-config generator: one configure, then --config per build
        run([cmake, "-S", ".", "-B", COMPILER_DIR,
             "-G", "Visual Studio 17 2022", "-A", "x64",
             "-DTARGET_BUILD_PLATFORM=windows",
             f"-DPX_OUTPUT_LIB_DIR={output_base}",
             f"-DPX_OUTPUT_BIN_DIR={output_base}",
             *msvc_cmake_args],
            cwd=root)
    else:
        output_base = str(root / BUILD_DIR)
        for cfg, enabled in [("Debug", do_debug), ("Release", do_release)]:
            if enabled:
                cmake_dir = COMPILER_DIR / f"build-{cfg.lower()}"
                cmake_dir.mkdir(parents=True, exist_ok=True)
                (cmake_dir / "CMakeCache.txt").unlink(missing_ok=True)
                run([cmake, "-S", ".", "-B", cmake_dir, "-G", "Unix Makefiles",
                     f"-DCMAKE_BUILD_TYPE={cfg}", "-DTARGET_BUILD_PLATFORM=linux",
                     f"-DPX_OUTPUT_LIB_DIR={output_base}", f"-DPX_OUTPUT_BIN_DIR={output_base}"],
                    cwd=root, docker_prefix=docker_prefix)

    if args.generate_only:
        print("Generate-only: configuration complete.")
        return

    generate_test_scripts(root)

    if IS_WINDOWS:
        for cfg, do_build in [("Debug", do_debug), ("Release", do_release)]:
            if do_build:
                t0 = time.monotonic()
                build_cmd = [cmake, "--build", COMPILER_DIR, "--config", cfg]
                if args.target:
                    build_cmd += ["--target", args.target]
                build_cmd += ["--", "/m"]
                run(build_cmd, cwd=root)
                print(f"{cfg} build: {fmt_elapsed(time.monotonic() - t0)}")
                if not args.target:
                    run([cmake, "--install", COMPILER_DIR, "--config", cfg,
                         "--prefix", INSTALL_DIR / cfg.lower()], cwd=root)

    else:
        jobs = str(min(_get_cpu_count(), 12))

        for cfg, enabled in [("Debug", do_debug), ("Release", do_release)]:
            if enabled:
                cmake_dir = COMPILER_DIR / f"build-{cfg.lower()}"
                t0 = time.monotonic()
                build_cmd = [cmake, "--build", cmake_dir, f"-j{jobs}"]
                if args.target:
                    build_cmd += ["--target", args.target]
                run(build_cmd, cwd=root, docker_prefix=docker_prefix)
                print(f"{cfg} build: {fmt_elapsed(time.monotonic() - t0)}")
                if not args.target:
                    run([cmake, "--install", cmake_dir, "--prefix", INSTALL_DIR / cfg.lower()],
                        cwd=root, docker_prefix=docker_prefix)

    # --- Python venv ---
    setup_python_venv(root)

    print(f"\nBuild completed successfully in {fmt_elapsed(time.monotonic() - total_start)}")


if __name__ == "__main__":
    main()
