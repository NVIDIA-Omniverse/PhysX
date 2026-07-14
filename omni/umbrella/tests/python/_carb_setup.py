# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# Carbonite framework initialization for standalone Python tests.
#
# Import this module before importing _physics to ensure the Carbonite
# framework is started and the PhysicsUmbrella plugin is loaded.
#
# Usage:
#     import _carb_setup  # noqa: F401
#     from _physics import acquire_physics_interface, ...

import sys
import os
from pathlib import Path

# --- Path setup ---

# _carb_setup.py is in tests/python/ -> umbrella is two levels up
UMBRELLA_DIR = Path(__file__).resolve().parent.parent.parent
BUILD_DIR = UMBRELLA_DIR / "_build"
TARGET_DEPS = BUILD_DIR / "target-deps"

# Build artifacts are staged into _build/<platform>/bindings/ by POST_BUILD.
if sys.platform == "win32":
    BINDINGS_DIR = BUILD_DIR / "windows-x86_64" / "bindings"
else:
    BINDINGS_DIR = BUILD_DIR / "linux-x86_64" / "bindings"

# Carbonite SDK (contains carb.dll, plugin DLLs, and carb Python bindings)
CARB_SDK = TARGET_DEPS / "carb_sdk_plugins"
if sys.platform == "win32":
    CARB_BIN_DIR = CARB_SDK / "_build" / "windows-x86_64" / "release"
else:
    CARB_BIN_DIR = CARB_SDK / "_build" / "linux-x86_64" / "release"

CARB_PY_DIR = CARB_BIN_DIR / "bindings-python"

# Add carb Python module to sys.path
if CARB_PY_DIR.exists():
    if str(CARB_PY_DIR) not in sys.path:
        sys.path.insert(0, str(CARB_PY_DIR))
else:
    raise RuntimeError(
        f"Carbonite Python bindings not found at {CARB_PY_DIR}\n"
    )

# Add bindings directory to sys.path (for _physics module)
if str(BINDINGS_DIR) not in sys.path:
    sys.path.insert(0, str(BINDINGS_DIR))

# Add DLL directories (Windows - needed for carb.dll, PhysicsUmbrella.dll)
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    for dll_dir in [BINDINGS_DIR, CARB_BIN_DIR]:
        if dll_dir.exists():
            os.add_dll_directory(str(dll_dir))

# --- Carbonite framework initialization ---

import carb  # noqa: E402
import carb.events  # noqa: E402 - registers IEventStream pybind11 type

framework = carb.get_framework()
framework.startup([], '{ "log": { "level": "warning" } }')

# Load the PhysicsUmbrella plugin. load_plugins(["*"]) is required because the
# Carbonite Python API only supports "*" as a wildcard (specific names/patterns
# don't work). Suppress stdout/stderr during loading to hide harmless warnings
# from non-plugin DLLs (carb.dll, mimalloc.dll, etc.) that fail the plugin check.
_plugin_search_paths = []
if CARB_BIN_DIR.exists():
    _plugin_search_paths.append(str(CARB_BIN_DIR))
if BINDINGS_DIR.exists():
    _plugin_search_paths.append(str(BINDINGS_DIR))

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

# Import _physics so it's available to test modules
import _physics  # noqa: E402, F401
