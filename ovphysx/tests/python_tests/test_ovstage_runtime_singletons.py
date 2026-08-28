# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Process-isolated startup-order coverage for the OVStage runtime provider."""

import importlib.util
import json
import os
import shutil
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

_RESULT_PREFIX = "OVPHYSX_RUNTIME_MAPS="
_CHILD_SCRIPT = textwrap.dedent(f"""
    import json
    import os
    import sys
    from pathlib import Path

    def collect_loaded_runtime_paths():
        if sys.platform == "linux":
            paths = []
            for line in Path("/proc/self/maps").read_text().splitlines():
                fields = line.split()
                if len(fields) >= 6 and fields[-1].startswith("/"):
                    paths.append(fields[-1])
            return paths

        if sys.platform == "win32":
            import ctypes
            from ctypes import wintypes

            psapi = ctypes.WinDLL("psapi.dll", use_last_error=True)
            kernel32 = ctypes.WinDLL("kernel32.dll", use_last_error=True)
            kernel32.GetCurrentProcess.argtypes = []
            kernel32.GetCurrentProcess.restype = wintypes.HANDLE
            psapi.EnumProcessModules.argtypes = [
                wintypes.HANDLE,
                ctypes.POINTER(wintypes.HMODULE),
                wintypes.DWORD,
                ctypes.POINTER(wintypes.DWORD),
            ]
            psapi.EnumProcessModules.restype = wintypes.BOOL
            psapi.GetModuleFileNameExW.argtypes = [
                wintypes.HANDLE,
                wintypes.HMODULE,
                wintypes.LPWSTR,
                wintypes.DWORD,
            ]
            psapi.GetModuleFileNameExW.restype = wintypes.DWORD
            process = kernel32.GetCurrentProcess()
            needed = wintypes.DWORD()
            if not psapi.EnumProcessModules(process, None, 0, ctypes.byref(needed)):
                raise ctypes.WinError(ctypes.get_last_error())

            module_count = max(1, needed.value // ctypes.sizeof(wintypes.HMODULE))
            modules = (wintypes.HMODULE * module_count)()
            if not psapi.EnumProcessModules(
                process, modules, ctypes.sizeof(modules), ctypes.byref(needed)
            ):
                raise ctypes.WinError(ctypes.get_last_error())

            paths = []
            path_buffer = ctypes.create_unicode_buffer(32768)
            returned_count = needed.value // ctypes.sizeof(wintypes.HMODULE)
            for module in modules[:returned_count]:
                length = psapi.GetModuleFileNameExW(
                    process, module, path_buffer, len(path_buffer)
                )
                if length:
                    paths.append(path_buffer.value)
            return paths

        raise RuntimeError(f"unsupported singleton mapping platform: {{sys.platform}}")

    order = sys.argv[1]
    usd_path = sys.argv[2]
    result_path = sys.argv[3]
    alternate_usd_path = sys.argv[4] if len(sys.argv) > 4 else None
    stage = None
    physx = None
    preloaded_usd = None
    try:
        if order == "ovstage-first":
            import ovstage

            stage = ovstage.Stage("ovphysx-runtime-provider-ovstage-first")
            ovstage.population.open_usd(
                stage,
                usd_path,
                ordinal=1,
                domains=ovstage.PopulationDomain.PHYSICS,
            )
            import ovphysx

            ovphysx.PhysX.set_cpu_mode(True)
            physx = ovphysx.PhysX()
        elif order == "physx-first":
            import ovphysx

            ovphysx.PhysX.set_cpu_mode(True)
            physx = ovphysx.PhysX()
            import ovstage

            stage = ovstage.Stage("ovphysx-runtime-provider-physx-first")
            ovstage.population.open_usd(
                stage,
                usd_path,
                ordinal=1,
                domains=ovstage.PopulationDomain.PHYSICS,
            )
        elif order == "alternate-usd-first":
            import ctypes

            if alternate_usd_path is None:
                raise RuntimeError("alternate-usd-first requires a copied USD runtime path")
            preloaded_usd = ctypes.CDLL(
                alternate_usd_path,
                mode=os.RTLD_NOW | os.RTLD_GLOBAL,
            )
            import ovphysx
            from ovphysx import _bindings

            # Runtime tests use OVPHYSX_LIB and therefore do not exercise the
            # bundled-wheel preload automatically. Invoke the same loader path
            # explicitly before initializing either consumer.
            _bindings._preload_ovstage_runtime_deps()
            ovphysx.PhysX.set_cpu_mode(True)
            physx = ovphysx.PhysX()
            import ovstage

            stage = ovstage.Stage("ovphysx-runtime-provider-alternate-usd-first")
            ovstage.population.open_usd(
                stage,
                usd_path,
                ordinal=1,
                domains=ovstage.PopulationDomain.PHYSICS,
            )
        else:
            raise RuntimeError(f"unknown startup order: {{order}}")

        # Population does not seal: the caller owns ordinal lifecycle, and
        # attach_ovstage() reads at a sealed ordinal.
        stage.advance_write_floor(ordinal=1).wait()
        physx.attach_ovstage(stage, read_ordinal=1)
        physx.step(1.0 / 60.0)
        physx.wait_all()

        mapped = {{}}
        for path in collect_loaded_runtime_paths():
            name = os.path.basename(path).lower()
            if name in (
                "libovstage.so",
                "libomni_usd_resolver.so",
                "libomniclient.so",
                "libomniverse_connection.so",
                "omni_usd_resolver.dll",
                "omniclient.dll",
                "omniverse_connection.dll",
            ) or (
                (name.startswith("libov_") and name.endswith("usd_ms.so"))
                or (name.startswith("ov_") and name.endswith("usd_ms.dll"))
            ):
                mapped.setdefault(name, set()).add(path)

        serializable = {{name: sorted(paths) for name, paths in mapped.items()}}
        # Hand the result over via a file: carb logs to stdout from other
        # threads (e.g. the WinSDK version-check spam when the module was
        # built with a newer SDK than the host runs), and that interleaving
        # can splice into a stdout-printed result line.
        Path(result_path).write_text(json.dumps(serializable, sort_keys=True), encoding="utf-8")
        print("{_RESULT_PREFIX}" + json.dumps(serializable, sort_keys=True), flush=True)
    finally:
        if physx is not None and stage is not None:
            physx.detach_ovstage()
        if stage is not None:
            stage.destroy()
        if physx is not None:
            physx.release()
    """)


@pytest.mark.skipif(
    sys.platform not in ("linux", "win32"),
    reason="singleton mapping assertion supports Linux and Windows",
)
@pytest.mark.parametrize("order", ["physx-first", "ovstage-first"])
def test_ovstage_runtime_is_singleton_in_both_startup_orders(order, tmp_path):
    usd_path = Path(__file__).resolve().parents[1] / "data" / "basic_simulation.usda"
    result_path = tmp_path / f"runtime_maps_{order}.json"
    result = subprocess.run(
        [sys.executable, "-c", _CHILD_SCRIPT, order, str(usd_path), str(result_path)],
        check=False,
        capture_output=True,
        text=True,
        env=os.environ.copy(),
        timeout=120,
    )

    assert result.returncode == 0, f"child failed\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    assert result_path.is_file(), f"child produced no result file\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    mapped = json.loads(result_path.read_text(encoding="utf-8"))
    print(f"{order} OVStage runtime mappings: {json.dumps(mapped, sort_keys=True)}")

    if sys.platform == "win32":
        expected_names = {
            "omni_usd_resolver.dll",
            "omniclient.dll",
            "omniverse_connection.dll",
        }
        usd_monoliths = [name for name in mapped if name.startswith("ov_") and name.endswith("usd_ms.dll")]
    else:
        expected_names = {
            "libovstage.so",
            "libomni_usd_resolver.so",
            "libomniclient.so",
            "libomniverse_connection.so",
        }
        usd_monoliths = [name for name in mapped if name.startswith("libov_") and name.endswith("usd_ms.so")]
    assert expected_names <= mapped.keys()
    assert len(usd_monoliths) == 1

    for name, paths in mapped.items():
        assert len(paths) == 1, f"{name} mapped from more than one runtime file: {paths}"
        normalized_path = paths[0].replace("\\", "/").lower()
        assert "/target-deps/client-library/" not in normalized_path
        assert "/target-deps/omni_usd_resolver/" not in normalized_path


@pytest.mark.skipif(sys.platform != "linux", reason="alternate-path USD regression is Linux-only")
def test_preloaded_usd_monolith_at_alternate_path_is_reused(tmp_path):
    ovstage_spec = importlib.util.find_spec("ovstage")
    assert ovstage_spec is not None and ovstage_spec.origin is not None, "installed ovstage package not found"
    ovstage_bin = Path(ovstage_spec.origin).resolve().parent / "bin"
    usd_candidates = sorted((ovstage_bin / "plugins").glob("libov_*usd_ms.so"))
    assert len(usd_candidates) == 1, f"expected one installed OVStage USD monolith, found {usd_candidates}"
    installed_usd = usd_candidates[0]

    alternate_dir = tmp_path / "alternate-runtime"
    alternate_dir.mkdir()
    alternate_usd = Path(shutil.copy2(installed_usd, alternate_dir / installed_usd.name)).resolve()

    child_env = os.environ.copy()
    runtime_dirs = [str(installed_usd.parent), str(ovstage_bin)]
    if child_env.get("LD_LIBRARY_PATH"):
        runtime_dirs.append(child_env["LD_LIBRARY_PATH"])
    child_env["LD_LIBRARY_PATH"] = os.pathsep.join(runtime_dirs)

    usd_path = Path(__file__).resolve().parents[1] / "data" / "basic_simulation.usda"
    result_path = tmp_path / "runtime_maps_alternate_usd.json"
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            _CHILD_SCRIPT,
            "alternate-usd-first",
            str(usd_path),
            str(result_path),
            str(alternate_usd),
        ],
        check=False,
        capture_output=True,
        text=True,
        env=child_env,
        timeout=120,
    )

    assert result.returncode == 0, f"child failed\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    assert result_path.is_file(), f"child produced no result file\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    mapped = json.loads(result_path.read_text(encoding="utf-8"))
    usd_name = installed_usd.name.lower()
    assert usd_name in mapped, f"{usd_name} was not mapped: {mapped}"
    assert mapped[usd_name] == [str(alternate_usd)], (
        f"{usd_name} mapped from unexpected runtime file: {mapped[usd_name]}"
        if len(mapped[usd_name]) == 1
        else f"{usd_name} mapped from {len(mapped[usd_name])} runtime files: {mapped[usd_name]}"
    )
