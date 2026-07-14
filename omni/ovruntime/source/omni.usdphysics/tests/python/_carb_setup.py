# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# Carbonite framework initialization for standalone Python tests.
#
# Import this module before using any carb API to ensure the Carbonite
# framework is started and core plugins are loaded.
#
# Usage:
#     import _carb_setup  # noqa: F401
#     import carb.settings

import sys
import os
import ctypes
from pathlib import Path

# --- Path setup ---

# _carb_setup.py is in source/omni.usdphysics/tests/python/ -> ovruntime root is four levels up
OVRUNTIME_DIR = Path(__file__).resolve().parent.parent.parent.parent.parent
BUILD_DIR = OVRUNTIME_DIR / "_build"
TARGET_DEPS = BUILD_DIR / "target-deps"

# Platform-specific build layout
if sys.platform == "win32":
    _PLATFORM = "windows-x86_64"
else:
    _PLATFORM = "linux-x86_64"

# Carbonite SDK (contains carb shared libs, plugin DLLs, and carb Python bindings)
CARB_SDK = TARGET_DEPS / "carb_sdk_plugins"
CARB_BIN_DIR = CARB_SDK / "_build" / _PLATFORM / "release"
CARB_PY_DIR = CARB_BIN_DIR / "bindings-python"

# ovruntime_deps runtime plugins (carb.stats, omni.fabric, omni.tbb.globalcontrol, omni.usd)
OVRUNTIME_DEPS = TARGET_DEPS / "ovruntime_deps_release"
OVRUNTIME_DEPS_PLUGINS_DIR = OVRUNTIME_DEPS / "_build" / _PLATFORM / "release" / "plugins"

# ovruntime build output (physics plugins: omni.usdphysics.plugin.so, _usdphysics.so bindings, etc.)
OVRUNTIME_BIN_DIR = BUILD_DIR / _PLATFORM / "release"

# USD libraries (needed at runtime by _usdphysics.so and other physics bindings)
USD_LIB_DIR = TARGET_DEPS / "usd" / "release" / "lib"
USD_BIN_DIR = TARGET_DEPS / "usd" / "release" / "bin"
USD_PY_DIR = USD_LIB_DIR / "python"

# Add USD Python bindings (pxr) to sys.path so tests work outside the venv
if USD_PY_DIR.exists():
    if str(USD_PY_DIR) not in sys.path:
        sys.path.insert(0, str(USD_PY_DIR))

# Add carb Python module to sys.path
if CARB_PY_DIR.exists():
    if str(CARB_PY_DIR) not in sys.path:
        sys.path.insert(0, str(CARB_PY_DIR))
else:
    raise RuntimeError(
        f"Carbonite Python bindings not found at {CARB_PY_DIR}\n"
        f"Run: pull_dependencies.bat (or .sh) to fetch dependencies via packman"
    )

# Add physics Python bindings (_usdphysics.so etc.) to sys.path
if OVRUNTIME_BIN_DIR.exists():
    if str(OVRUNTIME_BIN_DIR) not in sys.path:
        sys.path.insert(0, str(OVRUNTIME_BIN_DIR))

# Add DLL directories (Windows - needed for carb.dll and plugin DLLs)
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    for d in [CARB_BIN_DIR, OVRUNTIME_BIN_DIR, USD_LIB_DIR, USD_BIN_DIR]:
        if d.exists():
            os.add_dll_directory(str(d))

# --- Carbonite framework initialization ---

import carb  # noqa: E402
import carb.events  # noqa: E402 - registers IEventStream pybind11 type

framework = carb.get_framework()
framework.startup([], '{ "log": { "level": "warning" } }')

# --- Phase 1: Load Carbonite core plugins ---
# Use ["*"] from CARB_BIN_DIR only.  Suppress stdout/stderr during loading to
# hide harmless warnings from non-plugin DLLs (carb.dll, mimalloc.dll, etc.).
_plugin_search_paths = []
if CARB_BIN_DIR.exists():
    _plugin_search_paths.append(str(CARB_BIN_DIR))

_saved_stdout_fd = os.dup(1)
_saved_stderr_fd = os.dup(2)
_devnull = os.open(os.devnull, os.O_WRONLY)
os.dup2(_devnull, 1)
os.dup2(_devnull, 2)
os.close(_devnull)

framework.load_plugins(["*"], search_paths=_plugin_search_paths)

os.dup2(_saved_stdout_fd, 1)
os.dup2(_saved_stderr_fd, 2)
os.close(_saved_stdout_fd)
os.close(_saved_stderr_fd)
