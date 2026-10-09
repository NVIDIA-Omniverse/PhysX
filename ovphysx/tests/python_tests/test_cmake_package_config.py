# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Configure real CMake consumers without compiling or downloading an SDK."""

# @implements REQ-PACKAGING-OVSTAGEVERSION-001
# @covers AC-1 AC-2 AC-3
# @maps_to TEST-PACKAGING-OVSTAGEVERSION-001

import shutil
import subprocess
from pathlib import Path

import pytest

OVPHYSX_DIR = Path(__file__).resolve().parents[2]
PIN = "0.2.1.385922"


def _run_cmake(*args: str) -> subprocess.CompletedProcess:
    cmake = shutil.which("cmake")
    if not cmake:
        pytest.fail("cmake is required to test the installed SDK package configuration")
    return subprocess.run([cmake, *args], text=True, capture_output=True, check=False)


def _output(result: subprocess.CompletedProcess) -> str:
    return result.stdout + result.stderr


def _read_pin_commands(pin_source: Path) -> str:
    return (
        f'include("{(OVPHYSX_DIR / "cmake" / "ReadOvstageVersion.cmake").as_posix()}")\n'
        f'ovphysx_read_ovstage_version("{pin_source.as_posix()}"\n'
        '    OVPHYSX_OVSTAGE_VERSION OVPHYSX_OVSTAGE_CMAKE_VERSION)\n'
    )


def _generate_packages(tmp_path: Path, pin: str, supplied_version: str, *, schema_api: bool = True) -> Path:
    """Use the shipped template and CMake's normal package-version selection."""
    sdk = tmp_path / "sdk"
    config_dir = sdk / "lib" / "cmake" / "ovphysx"
    config_dir.mkdir(parents=True, exist_ok=True)
    (sdk / "include").mkdir(exist_ok=True)
    (config_dir / "ovphysxTargets.cmake").write_text(
        'add_library(ovphysx::ovphysx INTERFACE IMPORTED)\n'
        'set_target_properties(ovphysx::ovphysx PROPERTIES\n'
        '    INTERFACE_INCLUDE_DIRECTORIES "${PACKAGE_PREFIX_DIR}/include")\n',
        encoding="utf-8",
    )

    ovstage = tmp_path / "ovstage"
    ovstage.mkdir(exist_ok=True)
    include_dir = ovstage / "include"
    (include_dir / "ovstage").mkdir(parents=True, exist_ok=True)
    (include_dir / "ovstage" / "ovstage_population.h").write_text(
        "void ovstage_population_register_usd_schemas();\n" if schema_api else "/* No schema API. */\n",
        encoding="utf-8",
    )
    (ovstage / "ovstageConfig.cmake").write_text(
        'add_library(ovstage::ovstage INTERFACE IMPORTED)\n'
        f'set(ovstage_INCLUDE_DIRS "{include_dir.as_posix()}")\n'
        'set_target_properties(ovstage::ovstage PROPERTIES\n'
        '    INTERFACE_INCLUDE_DIRECTORIES "${ovstage_INCLUDE_DIRS}")\n',
        encoding="utf-8",
    )

    pin_source = tmp_path / "fetch_ovstage_release.py"
    pin_source.write_text(f'OVSTAGE_VERSION = "{pin}"\n', encoding="utf-8")
    generate = tmp_path / "generate.cmake"
    generate.write_text(
        'cmake_minimum_required(VERSION 3.22)\n'
        + _read_pin_commands(pin_source)
        + 'include(CMakePackageConfigHelpers)\n'
        'set(PROJECT_VERSION "0.6.0")\n'
        f'set(CMAKE_INSTALL_PREFIX "{sdk.as_posix()}")\n'
        'set(CMAKE_INSTALL_BINDIR bin)\n'
        'set(CMAKE_INSTALL_LIBDIR lib)\n'
        'set(OVPHYSX_PLUGINSDIR plugins)\n'
        'configure_package_config_file(\n'
        f'    "{(OVPHYSX_DIR / "cmake" / "ovphysxConfig.cmake.in").as_posix()}"\n'
        f'    "{(config_dir / "ovphysxConfig.cmake").as_posix()}"\n'
        '    INSTALL_DESTINATION lib/cmake/ovphysx\n'
        '    PATH_VARS CMAKE_INSTALL_BINDIR CMAKE_INSTALL_LIBDIR OVPHYSX_PLUGINSDIR)\n'
        'write_basic_package_version_file(\n'
        f'    "{(ovstage / "ovstageConfigVersion.cmake").as_posix()}"\n'
        # OVStage's published config accepts newer patches within the same 0.x minor.
        f'    VERSION "{supplied_version}" COMPATIBILITY SameMinorVersion ARCH_INDEPENDENT)\n',
        encoding="utf-8",
    )
    result = _run_cmake("-P", str(generate))
    assert result.returncode == 0, _output(result)
    return config_dir


def _configure_consumer(tmp_path: Path, config_dir: Path) -> subprocess.CompletedProcess:
    source = tmp_path / "consumer"
    source.mkdir(exist_ok=True)
    (source / "CMakeLists.txt").write_text(
        'cmake_minimum_required(VERSION 3.22)\n'
        'project(ovphysx_consumer NONE)\n'
        'set(CMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH OFF)\n'
        'set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH OFF)\n'
        'set(CMAKE_FIND_USE_CMAKE_SYSTEM_PATH OFF)\n'
        'find_package(ovphysx REQUIRED CONFIG)\n'
        'get_target_property(dependencies ovphysx::ovphysx INTERFACE_LINK_LIBRARIES)\n'
        'if(NOT "ovstage::ovstage" IN_LIST dependencies)\n'
        '    message(FATAL_ERROR "Consumer did not inherit the OVStage target")\n'
        'endif()\n'
        'file(WRITE "${CMAKE_BINARY_DIR}/resolved.txt"\n'
        '    "${ovphysx_OVSTAGE_VERSION}\\n${ovphysx_OVSTAGE_CMAKE_VERSION}\\n")\n',
        encoding="utf-8",
    )
    return _run_cmake(
        "-S", str(source), "-B", str(tmp_path / "consumer-build"),
        f"-Dovphysx_DIR={config_dir.as_posix()}",
        f"-Dovstage_DIR={(tmp_path / 'ovstage').as_posix()}",
        "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF",
        "-DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF",
    )


@pytest.mark.parametrize("supplied_version", ["0.2.0", "0.2.2", "0.3.0"])
def test_consumer_rejects_ovstage_from_another_release(tmp_path, supplied_version):
    config_dir = _generate_packages(tmp_path, PIN, supplied_version)
    result = _configure_consumer(tmp_path, config_dir)
    assert result.returncode != 0, f"OVStage {supplied_version} was accepted for pin {PIN}:\n{_output(result)}"
    assert "ovstage" in _output(result)
    assert "0.2.1" in _output(result)


@pytest.mark.parametrize("pin", [PIN, "0.7.2.999999"])
def test_consumer_accepts_matching_release_and_exposes_full_pin(tmp_path, pin):
    semver = ".".join(pin.split(".")[:3])
    config_dir = _generate_packages(tmp_path, pin, semver)
    result = _configure_consumer(tmp_path, config_dir)
    assert result.returncode == 0, _output(result)
    assert (tmp_path / "consumer-build" / "resolved.txt").read_text(encoding="utf-8").splitlines() == [pin, semver]


def test_consumer_rejects_missing_schema_registration_api(tmp_path):
    config_dir = _generate_packages(tmp_path, PIN, "0.2.1", schema_api=False)
    result = _configure_consumer(tmp_path, config_dir)
    assert result.returncode != 0, _output(result)
    assert "ovstage_population_register_usd_schemas" in _output(result)


@pytest.mark.parametrize(
    "source",
    [
        None,
        "# No pin\n",
        'OVSTAGE_VERSION = "0.2"\n',
        'OVSTAGE_VERSION = "0.2.1.bad"\n',
        'OVSTAGE_VERSION = "0.2.1.385922"\nOVSTAGE_VERSION = "0.2.2.999999"\n',
    ],
    ids=["missing-file", "missing-pin", "missing-patch", "invalid-build", "duplicate-pin"],
)
def test_package_generation_rejects_missing_or_malformed_pin(tmp_path, source):
    pin_source = tmp_path / "fetch_ovstage_release.py"
    if source is not None:
        pin_source.write_text(source, encoding="utf-8")
    script = tmp_path / "read_pin.cmake"
    script.write_text('cmake_minimum_required(VERSION 3.22)\n' + _read_pin_commands(pin_source), encoding="utf-8")
    result = _run_cmake("-P", str(script))
    assert result.returncode != 0, _output(result)
    assert "OVSTAGE_VERSION" in _output(result) or "version source is missing" in _output(result)
