# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# UsdPhysics plugin initialization for standalone Python tests.
#
# Builds on _carb_setup (Carbonite core) by loading the RTX infrastructure
# plugins (TBB, Fabric) and the omni.usdphysics plugin.
#
# Usage:
#     import _usdphysics_setup  # noqa: F401 - loads usdphysics plugin
#     import _usdphysics

import sys
import os
import ctypes
import _carb_setup  # noqa: F401 - Carbonite core must be initialized first

# --- Preload shared libraries required by plugins ---
# RTX and physics plugins depend on TBB and USD shared libraries that are not
# on the default library search path.  Preloading them with RTLD_GLOBAL makes
# their symbols available to all subsequent dlopen calls.

_RTX = _carb_setup.OVRUNTIME_DEPS_PLUGINS_DIR
_USD = _carb_setup.USD_LIB_DIR
_RTLD_GLOBAL = getattr(ctypes, "RTLD_GLOBAL", 0)  # RTLD_GLOBAL is Linux-only


def _preload(path):
    """Load a shared library into the global symbol space, silently skip if missing."""
    if path.exists():
        try:
            ctypes.CDLL(str(path), mode=_RTLD_GLOBAL)
        except OSError:
            pass


# TBB (required by omni.tbb.globalcontrol and omni.fabric)
_preload(_RTX / "libtbb.so.12")

# USD libraries (required by omni.fabric, omni.usdphysics, and physics plugins).
# Order matters: low-level libs first so higher-level ones find their dependencies.
for _name in [
    "libusd_arch.so", "libusd_tf.so", "libusd_js.so", "libusd_trace.so",
    "libusd_work.so", "libusd_plug.so", "libusd_vt.so", "libusd_gf.so",
    "libusd_pegtl.so", "libusd_ar.so", "libusd_kind.so", "libusd_sdf.so",
    "libusd_pcp.so", "libusd_usd.so", "libusd_usdGeom.so", "libusd_usdPhysics.so",
    "libusd_usdUtils.so", "libusd_usdShade.so", "libusd_sdr.so", "libusd_hf.so",
    "libusd_usdRender.so", "libusd_boost.so", "libusd_python.so",
]:
    _preload(_USD / _name)

# --- Load RTX infrastructure plugins ---
# These provide TBB global control, Fabric, and USD ABI support used by the
# usdphysics plugin.
_rtx_search_paths = []
for _subdir in ["carb.stats", "omni.fabric", "omni.usd"]:
    _p = _RTX / _subdir
    if _p.exists():
        _rtx_search_paths.append(str(_p))

_carb_setup.framework.load_plugins(["*"], search_paths=_rtx_search_paths)

# --- Load omni.usdphysics plugin by name ---
# We load by explicit name (not "*") because the build output directory also
# contains Python binding modules (_usdphysics.so, etc.) that must NOT be
# loaded as Carbonite plugins — they are pybind11 modules imported via
# Python's import mechanism instead.
_USDPHYSICS_PLUGINS = [
    "omni.usdphysics.plugin",
]

# Windows: register ext physics lib dir so physxSchema.dll is findable when loading plugins
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    _ext_lib = _carb_setup.TARGET_DEPS / "usd_ext_physics" / "release" / "lib"
    if _ext_lib.exists():
        os.add_dll_directory(str(_ext_lib))

_bin = _carb_setup.OVRUNTIME_BIN_DIR
if _bin.exists():
    _carb_setup.framework.load_plugins(_USDPHYSICS_PLUGINS, search_paths=[str(_bin)])

# --- Register PhysX USD schemas ---
# Without this, PhysX schema types (PhysxDeformableBodyAPI, etc.) are unknown
# to USD and schema-based APIs fail silently.
_USD_EXT_PHYSICS = _carb_setup.TARGET_DEPS / "usd_ext_physics" / "release"
_SCHEMA_PLUGINS = _USD_EXT_PHYSICS / "share" / "usd" / "plugins"

if _SCHEMA_PLUGINS.exists():
    from pxr import Plug
    _plug_registry = Plug.Registry()
    _plug_registry.RegisterPlugins(str(_SCHEMA_PLUGINS / "PhysxSchema" / "resources"))
    _plug_registry.RegisterPlugins(str(_SCHEMA_PLUGINS / "PhysxSchemaAddition" / "resources"))
    _plug_registry.RegisterPlugins(str(_SCHEMA_PLUGINS / "OmniUsdPhysicsDeformableSchema" / "resources"))

# Preload PhysX schema native libraries (needed by the Python bindings)
_EXT_PHYSICS_LIB = _USD_EXT_PHYSICS / "lib"
_preload(_EXT_PHYSICS_LIB / "libphysxSchema.so")
_preload(_EXT_PHYSICS_LIB / "libphysicsSchemaTools.so")
