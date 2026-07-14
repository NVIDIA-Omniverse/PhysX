# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""
Cross-platform build script for the ovexts project.

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
    --devphysx         Build PhysX SDK from source
    --devschema        Use locally-built physics schema

If neither -d nor -r is given, builds release only.
Run from the ovexts project root.
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

    NOTE: keep in sync with omni/umbrella/tools/build.py::_get_cpu_count().
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
        out = subprocess.run(
            [compiler, "--version"], capture_output=True, text=True, check=False
        ).stdout.lower()
    except OSError:
        out = ""
    return "clang" if "clang" in out else "gcc"


if IS_WINDOWS:
    BUILD_PLATFORM = "windows-x86_64"
    PACKMAN_PLATFORM = "windows-x86_64"
    PACKMAN = Path("tools/packman/packman.cmd")
    PACKMAN_PYTHON = Path("_build/target-deps/python/python.exe")
    COMPILER_DIR = Path("_compiler/vc17win64")
    REPO_CMD = "repo.bat"
elif IS_AARCH64:
    BUILD_PLATFORM = "linux-aarch64"
    PACKMAN_PLATFORM = "manylinux_2_35_aarch64"
    PACKMAN = Path("tools/packman/packman")
    PACKMAN_PYTHON = Path("_build/target-deps/python/python")
    COMPILER_DIR = Path(f"_compiler/{detect_linux_compiler_family()}-linux-aarch64")
    REPO_CMD = "./repo.sh"
else:
    BUILD_PLATFORM = "linux-x86_64"
    PACKMAN_PLATFORM = "manylinux_2_35_x86_64"
    PACKMAN = Path("tools/packman/packman")
    PACKMAN_PYTHON = Path("_build/target-deps/python/python")
    COMPILER_DIR = Path(f"_compiler/{detect_linux_compiler_family()}-linux-x86_64")
    REPO_CMD = "./repo.sh"

BUILD_DIR = Path(f"_build/{BUILD_PLATFORM}")


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
    # Mount the repo root, not just omni/, because physx/ source is needed in two cases:
    #   - --devphysx: the PhysX SDK is built from physx/ source
    #   - default:    omni.physx.pvd's CMakeLists.txt add_subdirectory's physx/pvddom
    repo_root = str(root.parent.parent)  # omni/ovexts -> omni -> physics
    primary_mount = f"{repo_root}:{repo_root}"
    packman_cache = os.environ.get(
        "PM_PACKAGES_ROOT", str(Path.home() / ".cache/packman")
    )
    # Kit extension cache — precache_exts downloads extensions here and creates
    # symlinks from _build/.../exts/ into this directory. Docker must be able to
    # follow these symlinks, so we mount this path read-only.
    ov_data_dir = os.environ.get(
        "XDG_DATA_HOME", str(Path.home() / ".local/share")
    ) + "/ov/data"
    uid_gid = f"{os.getuid()}:{os.getgid()}"
    mounts = [
        "-v", primary_mount,
        "-v", f"{packman_cache}:{packman_cache}:ro",
    ]
    if os.path.isdir(ov_data_dir):
        mounts += ["-v", f"{ov_data_dir}:{ov_data_dir}:ro"]
    return [
        "docker", "run", "--rm",
        *mounts,
        "-w", str(root),
        "-u", uid_gid,
        DOCKER_IMAGE,
    ]


def make_dir_link(link: Path, target: Path) -> None:
    """Create a directory link: junction on Windows (no privilege needed), symlink elsewhere."""
    if IS_WINDOWS:
        subprocess.run(f'mklink /J "{link}" "{target}"', shell=True, check=True)
    else:
        link.symlink_to(target)


def find_cmake(root: Path) -> str:
    """Prefer the packman-provided cmake; fall back to system cmake if not found."""
    suffix = ".exe" if IS_WINDOWS else ""
    packman_cmake = root / f"_build/target-deps/cmake/bin/cmake{suffix}"
    if packman_cmake.is_file():
        return str(packman_cmake)
    print(
        f"Warning: packman cmake not found at {packman_cmake}, falling back to system cmake"
    )
    return "cmake"


def run_stubgen(root: Path, configs: list) -> None:
    """Generate Python type stubs (.pyi) for pybind11 bindings via repo stubgen."""
    if IS_AARCH64:
        return
    for cfg in configs:
        print(f"Generating Python stubs ({cfg})...")
        run([REPO_CMD, "stubgen", "-c", cfg], cwd=root)


def pull_dependencies(root: Path, *, configs: list) -> None:
    """Pull all packman dependencies."""
    print("Pulling Physics ovexts dependencies with packman...")
    packman = root / PACKMAN
    p = PACKMAN_PLATFORM

    deps = [
        # core target deps (PhysX, onnx-mlir, etc.)
        ["deps/target-deps.packman.xml", "-p", p],
    ]

    # kit-sdk and imports are config-dependent (debug/release have separate packages)
    for cfg in configs:
        deps.append([
            "deps/kit-sdk-target-deps.packman.xml",
            "-p", p,
            "-t", f"platform_target_abi={p}",
            "-t", f"config={cfg}",
        ])
        deps.append([
            "deps/kit-sdk-target-deps-import.packman.xml",
            "-p", p,
            "-t", f"platform_target_abi={p}",
            "-t", f"config={cfg}",
        ])
        deps.append([
            "deps/schema-deps.packman.xml",
            "-p", p,
            "-t", f"platform_target_abi={p}",
            "-t", f"config={cfg}",
        ])

    # ovruntime_deps package (RTX/fabric plugins + ujitso/fabric/blobkey
    # headers). Not config-dependent — the release package carries headers and
    # release-mode plugin binaries consumed by ovruntime's cooking tests.
    deps.append([
        "deps/ovruntime-deps.packman.xml",
        "-p", p,
        "-t", f"platform_target_abi={p}",
    ])

    if IS_WINDOWS and (root / "deps/host-deps.packman.xml").exists():
        deps.append(["deps/host-deps.packman.xml", "-p", p])

    for args in deps:
        run([packman, "pull", *args], cwd=root)

    print("\nDependencies pulled successfully!\n")


def setup_msvc(root: Path, compiler_dir: Path) -> list:
    """Set up packman MSVC/WinSDK and return cmake -D args. Mirrors umbrella/tools/build.py."""
    msvc = root / "_build/host-deps/msvc"
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
        sdk_ver, inc, lib, sdk_bin = (
            "",
            winsdk / "Include",
            winsdk / "Lib",
            winsdk / "bin/x64",
        )
    else:
        sdk_ver = sorted((winsdk / "Include").glob("*"))[-1].name
        inc, lib, sdk_bin = (
            winsdk / f"Include/{sdk_ver}",
            winsdk / f"Lib/{sdk_ver}",
            winsdk / f"bin/{sdk_ver}/x64",
        )

    bin_dir = tools_dir / "bin/Hostx64/x64"
    includes = f"{tools_dir}\\include;{inc}\\ucrt;{inc}\\um;{inc}\\shared"
    libs = f"{tools_dir}\\lib\\x64;{lib}\\ucrt\\x64;{lib}\\um\\x64"
    os.environ["PATH"] = f"{bin_dir};{sdk_bin};{os.environ['PATH']}"

    # Patch NVIDIA.ImportBefore.props
    p = (
        msvc
        / "MSBuild/Current/Imports/Microsoft.Common.Props/ImportBefore/NVIDIA.ImportBefore.props"
    )
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(
        f'<Project xmlns="http://schemas.microsoft.com/developer/msbuild/2003"><PropertyGroup>'
        f"<DisableRegistryUse>true</DisableRegistryUse><VCToolsVersion>{ver}</VCToolsVersion>"
        f"<VCInstallDir_160>{msvc}\\VC\\</VCInstallDir_160>"
        f"<VCToolsInstallDir_160>{tools_dir}\\</VCToolsInstallDir_160>"
        f"</PropertyGroup></Project>"
    )

    # VCToolsVersion props
    vtv = msvc / f"VC/Auxiliary/Build/{ver}"
    vtv.mkdir(parents=True, exist_ok=True)
    (vtv / f"Microsoft.VCToolsVersion.{ver}.props").write_text(
        f'<?xml version="1.0"?><Project ToolsVersion="4.0" xmlns="http://schemas.microsoft.com/developer/msbuild/2003">'
        f"<PropertyGroup><VCToolsVersion>{ver}</VCToolsVersion></PropertyGroup></Project>"
    )

    # Directory.Build.props
    sdk_ver_tag = (
        f"<WindowsTargetPlatformVersion>{sdk_ver}</WindowsTargetPlatformVersion>"
        if sdk_ver
        else ""
    )
    compiler_dir.mkdir(parents=True, exist_ok=True)
    (compiler_dir / "Directory.Build.props").write_text(
        f"<Project><PropertyGroup><VCToolsVersion>{ver}</VCToolsVersion>"
        f"<WindowsSDKDir>{winsdk}\\</WindowsSDKDir>{sdk_ver_tag}"
        f"<IncludePath>{includes};$(IncludePath)</IncludePath>"
        f"<LibraryPath>{libs};$(LibraryPath)</LibraryPath>"
        f"<ExecutablePath>{bin_dir};{sdk_bin};$(ExecutablePath)</ExecutablePath>"
        # CUDA MSBuild targets use VC_ExecutablePath_x64_x64 for -ccbin.
        # Override it so nvcc uses the same cl.exe as PATH (packman MSVC),
        # avoiding "cl.exe in PATH is different than -ccbin" errors.
        f"<VC_ExecutablePath_x64_x64>{bin_dir}</VC_ExecutablePath_x64_x64>"
        f"</PropertyGroup></Project>"
    )

    # cmake 3.25.1 junction workaround
    vs_ver = subprocess.run(
        [
            "powershell",
            "-NoProfile",
            "-Command",
            f"(Get-Item '{msvc}\\MSBuild\\Current\\Bin\\MSBuild.exe').VersionInfo.FileVersion",
        ],
        capture_output=True,
        text=True,
    ).stdout.strip()
    link = root / f"_build/host-deps/msvc,version={vs_ver}"
    if not link.exists():
        make_dir_link(link, Path(msvc))

    return [
        f"-DCMAKE_GENERATOR_INSTANCE={msvc},version={vs_ver}",
        f"-DCMAKE_VS_PLATFORM_TOOLSET_VERSION={ver}",
        f"-DCMAKE_VS_SDK_INCLUDE_DIRECTORIES={includes.replace(';', chr(92) + ';')}",
        f"-DCMAKE_VS_SDK_LIBRARY_DIRECTORIES={libs.replace(';', chr(92) + ';')}",
    ]


def main():
    parser = argparse.ArgumentParser(description="Build ovexts (Windows/Linux)")
    parser.add_argument("-c", "--clean", dest="clean", action="store_true", help="Clean artifacts only (no build)")
    parser.add_argument("-x", "--rebuild", dest="rebuild", action="store_true", help="Clean then build")
    parser.add_argument("-d", "--debug", dest="debug", action="store_true", help="Build debug configuration")
    parser.add_argument("-r", "--release", dest="release", action="store_true", help="Build release configuration")
    parser.add_argument("-t", "--target", dest="target", default="", help="Build a specific CMake target")
    parser.add_argument("-g", "--generate", dest="generate_only", action="store_true", help="Configure only (no build)")
    parser.add_argument("-n", "--no-docker", dest="no_docker", action="store_true", help="Skip Docker, build on host (Linux)")
    parser.add_argument("--devphysx", dest="devphysx", action="store_true", help="Build PhysX SDK from source")
    parser.add_argument("--devschema", dest="devschema", action="store_true", help="Use locally-built physics schema")
    args = parser.parse_args()

    root = Path(__file__).resolve().parent.parent  # ovexts root
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
    do_debug = args.debug
    do_release = args.release or (not args.debug)

    # --- Docker prefix (Linux only, auto-disabled if docker CLI not found) ---
    use_docker = False
    docker_prefix = make_docker_prefix(root) if use_docker else None
    if use_docker:
        print(f"Building inside Docker ({DOCKER_IMAGE})...")

    # --- Pull dependencies (always on host) ---
    # Always pull release — headers and include files (USD, physics schema, etc.)
    # are needed from the release packages even when building debug only.
    pull_configs = ["release"]
    if do_debug:
        pull_configs.append("debug")
    pull_dependencies(root, configs=pull_configs)

    # --- Fetch ovruntime pip dependencies (Newton schemas) ---
    # The Newton USD schema pip package must be installed before CMake configure
    # so that FetchPipDependencies.cmake can generate NewtonSchemaTokens.h.
    ovruntime_dir = root / "../ovruntime"
    pip_toml = ovruntime_dir / "deps/pip_newton.toml"
    pip_fetch_script = ovruntime_dir / "tools/pip_fetch.py"
    packman_python = root / PACKMAN_PYTHON
    if pip_toml.is_file() and pip_fetch_script.is_file() and packman_python.is_file():
        print("Fetching ovruntime pip dependencies (Newton schemas)...")
        run(
            [packman_python, pip_fetch_script, pip_toml,
             "--python", str(packman_python.parent),
             "--target-deps", str(root / "_build/target-deps")],
            cwd=root,
        )

    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    COMPILER_DIR.mkdir(parents=True, exist_ok=True)
    cmake = find_cmake(root)
    output_base = str(root / BUILD_DIR)

    # --- Create kit/ and apps/ symlinks, then resolve Kit extensions ---
    # Kit extensions (omni.ui, omni.usd.core, etc.) are resolved from the Kit extension
    # registry by `repo.sh precache_exts`. This must run before cmake configure so that
    # the extension libraries are available for linking.
    # Both kit/ (Kit SDK binary) and apps/ (.kit app files) must exist for precache_exts.
    for cfg, enabled in [("debug", do_debug), ("release", do_release)]:
        if enabled:
            cfg_dir = root / BUILD_DIR.parent / BUILD_PLATFORM / cfg
            cfg_dir.mkdir(parents=True, exist_ok=True)
            # kit/ -> Kit SDK
            kit_link = cfg_dir / "kit"
            kit_sdk = root / f"_build/target-deps/kit_sdk_{cfg}"
            if kit_sdk.is_dir() and not kit_link.exists():
                make_dir_link(kit_link, kit_sdk)
                print(f"  Linked {kit_link} -> {kit_sdk}")
            # apps/ -> source apps directory (contains .kit files needed by precache_exts)
            apps_link = cfg_dir / "apps"
            apps_src = root / "apps"
            if apps_src.is_dir() and not apps_link.exists():
                make_dir_link(apps_link, apps_src)
                print(f"  Linked {apps_link} -> {apps_src}")

    # --- Resolve build-time Kit extensions (pre-configure) ---
    # Build-time extensions (omni.graph.tools, omni.ui, etc.) must be available
    # before cmake configure so that OVEXTS_OGN_AVAILABLE is set correctly and
    # extension libraries are available for linking.
    for cfg, enabled in [("debug", do_debug), ("release", do_release)]:
        if enabled:
            cfg_dir = root / BUILD_DIR.parent / BUILD_PLATFORM / cfg
            precache_kit = cfg_dir / "apps/precacheforbuild.kit"
            kit_exe = cfg_dir / ("kit/kit.exe" if IS_WINDOWS else "kit/kit")
            if not precache_kit.is_file() or not kit_exe.is_file():
                print(f"Warning: Kit SDK or precacheforbuild.kit not found, skipping build-time extension resolution ({cfg})")
                continue
            print(f"Resolving build-time Kit extensions ({cfg})...")
            exts_cache = str(cfg_dir / "exts")
            run([
                kit_exe, str(precache_kit),
                "--allow-root", "--portable", "--ext-precache-mode",
                "--/crashreporter/gatherUserStory=0",
                "--/app/settings/persistent=0",
                "--/app/settings/loadUserConfig=0",
                "--/app/extensions/parallelPullEnabled=1",
                "--/exts/omni.kit.registry.nucleus/omitExtVersion=1",
                "--/app/enableStdoutOutput=1",
                "--/app/extensions/registryEnabled=1",
                "--/app/extensions/mkdirExtFolders=0",
                f"--/app/extensions/registryCacheFull={exts_cache}",
                "--/log/flushStandardStreamOutput=1",
                f"--/app/extensions/target/config={cfg}",
                f"--portable-root", str(cfg_dir),
            ], cwd=root)

    # --- Optional: build PhysX from source (for --devphysx) ---
    cmake_extra_args = []
    if args.devphysx:
        cmake_extra_args.append("-DOVEXTS_DEV_PHYSX=ON")
        # PhysX GPU projects need SecureLoadLibrary (Windows) and other deps
        # from the PhysX dependencies manifest.  Pull using PhysX's own packman
        # (same approach as ovruntime/pull_dependencies.bat) and capture the
        # PM_SECURELOADLIBRARY_PATH env var that packman sets.
        physx_root = (root / "../../physx").resolve()
        physx_deps = physx_root / "dependencies.xml"
        physx_packman = physx_root / "buildtools/packman/packman.cmd" if IS_WINDOWS \
            else physx_root / "buildtools/packman/packman"
        if physx_deps.is_file() and physx_packman.is_file():
            physx_platform = "vc17win64" if IS_WINDOWS else "linux"
            print("DevPhysX mode: pulling PhysX source-build dependencies...")
            if IS_WINDOWS:
                # Run packman via a batch wrapper so env vars propagate, then
                # echo PM_SECURELOADLIBRARY_PATH so we can capture it.
                cmd = (
                    f'call {physx_packman} pull {physx_deps} --platform {physx_platform} && '
                    f'echo PM_SECURELOADLIBRARY_PATH=%PM_SECURELOADLIBRARY_PATH%'
                )
                result = subprocess.run(
                    cmd, shell=True,
                    cwd=physx_root, capture_output=True, text=True,
                )
                print(result.stdout)
                if result.returncode != 0:
                    print(result.stderr)
                    sys.exit(result.returncode)
                # Parse the echoed env var
                sl_path = ""
                for line in result.stdout.splitlines():
                    if line.startswith("PM_SECURELOADLIBRARY_PATH="):
                        sl_path = line.split("=", 1)[1].strip()
                        if sl_path == "%PM_SECURELOADLIBRARY_PATH%":
                            sl_path = ""
                        break
                # Fallback: find the package in the packman cache
                if not sl_path:
                    pm_root = os.environ.get("PM_PACKAGES_ROOT", "")
                    if pm_root:
                        import glob as _glob
                        for candidate in _glob.glob(os.path.join(pm_root, "chk", "SecureLoadLibrary", "*")):
                            if os.path.isfile(os.path.join(candidate, "src", "nvSecureLoadLibrary.c")):
                                sl_path = candidate
                                break
                if sl_path:
                    sl_path = sl_path.replace("\\", "/")
                    print(f"  SecureLoadLibrary: {sl_path}")
                    os.environ["PM_SECURELOADLIBRARY_PATH"] = sl_path
                else:
                    print("Warning: SecureLoadLibrary not found — PhysX GPU build may fail")
                    print(f"  (packman output: {result.stdout[-200:] if result.stdout else 'empty'})")
                    print(f"  PM_PACKAGES_ROOT={os.environ.get('PM_PACKAGES_ROOT', 'unset')}")
            else:
                run([physx_packman, "pull", str(physx_deps), "--platform", physx_platform],
                    cwd=physx_root)

    # --- Optional: build local schema first (for --devschema) ---
    if args.devschema:
        cmake_extra_args.append("-DOVEXTS_DEV_SCHEMA=ON")
        schema_dir = root / "../schema"
        if schema_dir.is_dir():
            schema_dir = schema_dir.resolve()
            schema_build_args = []
            if do_debug and not do_release:
                schema_build_args.append("-d")
            if do_release and not do_debug:
                schema_build_args.append("-r")
            fetch_args = list(schema_build_args)
            if args.clean or args.rebuild:
                fetch_args.append("-x")
            print("DevSchema mode: building local physics schema...")
            run([REPO_CMD, "build", "--fetch-only", *fetch_args], cwd=schema_dir)
            run([REPO_CMD, "usd"], cwd=schema_dir)
            run([REPO_CMD, "build", *schema_build_args], cwd=schema_dir)
        else:
            print(f"Warning: schema directory not found at {schema_dir}")

    # --- Configure ---
    if IS_WINDOWS:
        if (root / "deps/host-deps.packman.xml").exists():
            run(
                [root / PACKMAN, "pull", "deps/host-deps.packman.xml", "-p", PACKMAN_PLATFORM],
                cwd=root,
            )
        # Packman MSVC pins VS 2019 (msvc 2019-16.11.17-2); the host-toolchain
        # fallback (packman MSVC absent) targets the modern VS 2022 (v143).
        host_toolchain = not (root / "_build/host-deps/msvc").exists()
        vs_generator = "Visual Studio 17 2022" if host_toolchain else "Visual Studio 16 2019"
        msvc_cmake_args = setup_msvc(root, COMPILER_DIR)
        (COMPILER_DIR / "CMakeCache.txt").unlink(missing_ok=True)
        # When building PhysX from source (--devphysx), PhysX calls
        # ENABLE_LANGUAGE(CUDA) which requires the CUDA VS integration.
        # Point the VS generator at the packman CUDA toolkit via -T cuda=<path>.
        toolset_args = []
        if args.devphysx:
            cuda_dir = root / "_build/target-deps/cuda"
            if cuda_dir.is_dir():
                toolset_args = ["-T", f"cuda={cuda_dir}"]
        run(
            [
                cmake, "-S", ".", "-B", COMPILER_DIR,
                "-G", vs_generator, "-A", "x64",
                *toolset_args,
                "-DTARGET_BUILD_PLATFORM=windows",
                f"-DPX_OUTPUT_LIB_DIR={output_base}",
                f"-DPX_OUTPUT_BIN_DIR={output_base}",
                *msvc_cmake_args,
                *cmake_extra_args,
            ],
            cwd=root,
        )
    else:
        for cfg, enabled in [("Debug", do_debug), ("Release", do_release)]:
            if enabled:
                cmake_dir = COMPILER_DIR / f"build-{cfg.lower()}"
                cmake_dir.mkdir(parents=True, exist_ok=True)
                (cmake_dir / "CMakeCache.txt").unlink(missing_ok=True)
                run(
                    [
                        cmake, "-S", ".", "-B", cmake_dir,
                        "-G", "Unix Makefiles",
                        f"-DCMAKE_BUILD_TYPE={cfg}",
                        "-DTARGET_BUILD_PLATFORM=linux",
                        f"-DPX_OUTPUT_LIB_DIR={output_base}",
                        f"-DPX_OUTPUT_BIN_DIR={output_base}",
                        *cmake_extra_args,
                    ],
                    cwd=root,
                    docker_prefix=docker_prefix,
                )

    if args.generate_only:
        print("Generate-only: configuration complete.")
        return

    # --- Resolve app Kit extensions (post-configure) ---
    # CMake configure (above) created the extension directory layout via
    # ovexts_prebuild_link — symlinks for config/extension.toml etc. now exist
    # in extsPhysics/ and extsPhysicsRepo/.  We pass those as --ext-folder so
    # Kit recognises the local physics extensions and skips fetching them from
    # the registry, while still resolving all their transitive dependencies.
    # Build-time deps were already resolved above; here we process the remaining
    # app .kit files that depend on the locally-built physics extensions.
    precache_kits = [
        "apps/precache.kit",
        "apps/omni.bloky.kit",
        "apps/omni.bloky.test_ext.kit",
    ]
    for cfg, enabled in [("debug", do_debug), ("release", do_release)]:
        if enabled:
            cfg_dir = root / BUILD_DIR.parent / BUILD_PLATFORM / cfg
            kit_exe = cfg_dir / ("kit/kit.exe" if IS_WINDOWS else "kit/kit")
            if not kit_exe.is_file():
                print(f"Warning: Kit SDK not found, skipping extension precache ({cfg})")
                continue
            # Local physics extension folders — Kit uses these to identify
            # extensions that are provided locally and skip registry fetch.
            ext_folders = []
            for d in ["extsPhysics", "extsPhysicsRepo"]:
                ext_folders.extend(["--ext-folder", str(cfg_dir / d)])
            exts_cache = str(cfg_dir / "exts")
            for kit_file_rel in precache_kits:
                kit_file = cfg_dir / kit_file_rel
                if not kit_file.is_file():
                    continue
                print(f"Precaching extensions: {kit_file_rel} ({cfg})...")
                run([
                    kit_exe, str(kit_file),
                    "--allow-root", "--portable", "--ext-precache-mode",
                    "--/crashreporter/gatherUserStory=0",
                    "--/app/settings/persistent=0",
                    "--/app/settings/loadUserConfig=0",
                    "--/app/extensions/parallelPullEnabled=1",
                    "--/exts/omni.kit.registry.nucleus/omitExtVersion=1",
                    "--/app/enableStdoutOutput=1",
                    "--/app/extensions/registryEnabled=1",
                    "--/app/extensions/mkdirExtFolders=0",
                    f"--/app/extensions/registryCacheFull={exts_cache}",
                    "--/log/flushStandardStreamOutput=1",
                    f"--/app/extensions/target/config={cfg}",
                    f"--portable-root", str(cfg_dir),
                    *ext_folders,
                ], cwd=root)

    # --- Build ---
    # CPU count is cgroup-aware on Linux so it respects Kubernetes / Docker limits.
    # Even with correct cgroup detection, cap at a maximum to avoid OOM from
    # concurrent linker instances.  Windows cap is lower (8) than Linux (16).
    # Override with BUILD_JOB_COUNT env var in any environment.
    max_jobs = 8 if IS_WINDOWS else 16
    if "BUILD_JOB_COUNT" in os.environ:
        raw = os.environ["BUILD_JOB_COUNT"]
        try:
            jobs = int(raw)
            if jobs <= 0:
                raise ValueError("must be positive")
        except ValueError as e:
            raise ValueError(f"Invalid BUILD_JOB_COUNT='{raw}': must be a positive integer") from e
    else:
        jobs = min(_get_cpu_count(), max_jobs)

    if IS_WINDOWS:
        for cfg, do_build in [("Debug", do_debug), ("Release", do_release)]:
            if do_build:
                t0 = time.monotonic()
                build_cmd = [cmake, "--build", COMPILER_DIR, "--config", cfg]
                if args.target:
                    build_cmd += ["--target", args.target]
                build_cmd += ["--", f"/m:{jobs}"]
                run(build_cmd, cwd=root)
                print(f"{cfg} build: {fmt_elapsed(time.monotonic() - t0)}")
    else:
        for cfg, enabled in [("Debug", do_debug), ("Release", do_release)]:
            if enabled:
                cmake_dir = COMPILER_DIR / f"build-{cfg.lower()}"
                t0 = time.monotonic()
                build_cmd = [cmake, "--build", cmake_dir, f"-j{jobs}"]
                if args.target:
                    build_cmd += ["--target", args.target]
                run(build_cmd, cwd=root, docker_prefix=docker_prefix)
                print(f"{cfg} build: {fmt_elapsed(time.monotonic() - t0)}")

    # --- Install (populate _install/ for packaging) ---
    install_dir = root / "_install"
    if install_dir.exists():
        shutil.rmtree(install_dir)
    if IS_WINDOWS:
        # Multi-config: install rules are config-independent (headers/deps only),
        # so one invocation with any config is sufficient.
        run([cmake, "--install", COMPILER_DIR, "--config", "Release"], cwd=root)
    else:
        # Single-config: pick the first available build directory.
        for cfg in ["Release", "Debug"]:
            cmake_dir = COMPILER_DIR / f"build-{cfg.lower()}"
            if cmake_dir.is_dir():
                run(
                    [cmake, "--install", cmake_dir],
                    cwd=root,
                    docker_prefix=docker_prefix,
                )
                break

    # --- Install (populate _install/ for packaging) ---
    install_dir = root / "_install"
    if install_dir.exists():
        shutil.rmtree(install_dir)
    if IS_WINDOWS:
        # Multi-config: install rules are config-independent (headers/deps only),
        # so one invocation with any config is sufficient.
        run([cmake, "--install", COMPILER_DIR, "--config", "Release"], cwd=root)
    else:
        # Single-config: pick the first available build directory.
        for cfg in ["Release", "Debug"]:
            cmake_dir = COMPILER_DIR / f"build-{cfg.lower()}"
            if cmake_dir.is_dir():
                run(
                    [cmake, "--install", cmake_dir],
                    cwd=root,
                    docker_prefix=docker_prefix,
                )
                break


    print(
        f"\nBuild completed successfully in {fmt_elapsed(time.monotonic() - total_start)}"
    )


if __name__ == "__main__":
    main()
