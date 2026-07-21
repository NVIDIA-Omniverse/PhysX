# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Session-scoped PhysX instance for the benchmark suite.

Each benchmark process exercises one device (cpu or gpu). The harness driver
(``scripts/test_benchmarks_python.cmake``) passes ``--bench-device=cpu|gpu``
to pytest, which we read via the standard ``pytest_addoption`` hook.

Tests that target a specific device (e.g. ``bench_tensor_io_gpu``) use
``pytest.skip`` when the active device does not match.
"""

from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path

import pytest


def pytest_addoption(parser):
    # --bench-device replaces the OVPHYSX_BENCH_DEVICE env var (review on
    # MR !7247: env-var config is hard to discover; CLI option is idiomatic
    # pytest). Default = cpu so local `pytest` invocations work without args.
    parser.addoption(
        "--bench-device",
        action="store",
        default="cpu",
        choices=["cpu", "gpu"],
        help="Device to bring up the ovphysx runtime in (cpu or gpu).",
    )


def _pkg_dir() -> Path | None:
    spec = importlib.util.find_spec("ovphysx")
    if spec and spec.origin:
        return Path(spec.origin).parent
    return None


def _is_wheel_mode() -> bool:
    pkg_dir = _pkg_dir()
    if pkg_dir is None:
        return False
    lib = "ovphysx.dll" if sys.platform == "win32" else "libovphysx.so"
    return (pkg_dir / "lib" / lib).exists()


def _install_dir() -> Path | None:
    here = Path(__file__).resolve().parent
    candidate = here.parent.parent / "_install"
    return candidate if candidate.is_dir() else None


def pytest_configure(config):
    """Wire OVPHYSX_LIB and preload native libs when running against `_install/`.

    Mirrors the bootstrap performed by tests/python_tests/conftest.py so the
    benchmark suite works in the same install-tree configuration without
    requiring a wheel build.
    """
    if _is_wheel_mode():
        os.environ.setdefault("PYTHONHOME", sys.prefix)
        return

    install = _install_dir()
    if install is None:
        return

    lib_dir = install / ("bin" if sys.platform == "win32" else "lib")
    lib_name = "ovphysx.dll" if sys.platform == "win32" else "libovphysx.so"
    lib_path = lib_dir / lib_name
    if lib_path.exists():
        os.environ.setdefault("OVPHYSX_LIB", str(lib_path))

    if sys.platform != "win32":
        import ctypes
        RTLD_NODELETE = 0x01000

        if not hasattr(sys, "_ovphysx_preloaded_libs"):
            sys._ovphysx_preloaded_libs = []  # type: ignore[attr-defined]

        import sysconfig
        cfg = sysconfig.get_config_vars()
        py_lib_name = cfg.get("LDLIBRARY") or cfg.get("DLLLIBRARY")
        if py_lib_name:
            for d in filter(None, [cfg.get("LIBDIR"), cfg.get("LIBPL")]):
                full = Path(d) / py_lib_name
                if full.exists():
                    sys._ovphysx_preloaded_libs.append(  # type: ignore[attr-defined]
                        ctypes.CDLL(str(full), mode=ctypes.RTLD_GLOBAL | RTLD_NODELETE))
                    break

    os.environ.setdefault("PYTHONHOME", sys.prefix)


@pytest.fixture(scope="session")
def bench_device(request) -> str:
    return request.config.getoption("--bench-device").lower()


@pytest.fixture(scope="session")
def physx(bench_device):
    """Single PhysX instance for the whole session."""
    from ovphysx import PhysX
    sdk = PhysX()
    yield sdk
    try:
        sdk.release()
    except Exception:
        pass


def attach_usd_with_ovstage(physx, usd_path: Path, stage_name: str):
    import ovstage

    if not ovstage.population.available():
        raise RuntimeError("ovstage population bridge is unavailable")

    stage = ovstage.Stage(stage_name)
    ordinal = 1
    try:
        ovstage.population.open_usd(stage, str(usd_path), ordinal=ordinal, domains=ovstage.PopulationDomain.PHYSICS)
        physx.attach_ovstage(stage, read_ordinal=ordinal)
    except Exception:
        stage.destroy()
        raise

    stages = getattr(physx, "_ovphysx_benchmark_ovstages", None)
    if stages is None:
        stages = []
        setattr(physx, "_ovphysx_benchmark_ovstages", stages)
    stages.append(stage)
    return stage


def destroy_ovstage_benchmark_attachments(physx):
    stages = getattr(physx, "_ovphysx_benchmark_ovstages", None)
    if not stages:
        return
    try:
        physx.detach_ovstage()
    except Exception:
        pass
    while stages:
        stage = stages.pop()
        try:
            stage.destroy()
        except Exception:
            pass


@pytest.fixture(autouse=True)
def _reset_stage(physx):
    """Reset the stage between tests so each benchmark starts clean."""
    physx.reset_stage()
    physx.wait_all()
    destroy_ovstage_benchmark_attachments(physx)
    yield
    physx.reset_stage()
    physx.wait_all()
    destroy_ovstage_benchmark_attachments(physx)


@pytest.fixture(scope="session")
def data_dir() -> Path:
    return Path(__file__).resolve().parent.parent / "data"


def gpu_only(device: str) -> None:
    if device != "gpu":
        pytest.skip(f"Benchmark requires GPU device (active={device})")


def cpu_only(device: str) -> None:
    if device != "cpu":
        pytest.skip(f"Benchmark requires CPU device (active={device})")
