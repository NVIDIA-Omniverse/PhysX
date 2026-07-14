# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# Physics plugin initialization for standalone Python tests.
#
# Builds on _carb_setup (Carbonite core) by loading the RTX infrastructure
# plugins (TBB, Fabric) and the ovruntime physics plugins (omni.physx, etc.).
#
# Usage:
#     import _physics_setup  # noqa: F401 - loads physics plugins
#     import _physx
#     sim = _physx.acquire_physx_simulation_interface()

import sys
import os
import ctypes
import _carb_setup  # noqa: F401 - Carbonite core must be initialized first

# --- Preload shared libraries required by physics plugins ---
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
# physics simulation engine.
_rtx_search_paths = []
for _subdir in ["carb.stats", "omni.fabric", "omni.usd"]:
    _p = _RTX / _subdir
    if _p.exists():
        _rtx_search_paths.append(str(_p))

_carb_setup.framework.load_plugins(["*"], search_paths=_rtx_search_paths)

# --- Preload CUDA driver for GPU physics support ---
# The physics plugins link with --allow-shlib-undefined, so CUDA driver symbols
# (cuInit, cuDeviceGetCount, etc.) must be globally available before plugin load.
# PhysXGpu is NOT preloaded here — the foundation plugin loads it from its bin
# directory during carbOnPluginStartup via carb::extras::loadLibrary.
if sys.platform != "win32":
    try:
        ctypes.CDLL("libcuda.so.1", mode=_RTLD_GLOBAL)
    except OSError:
        pass

# --- Load ovruntime physics plugins by name ---
# We load by explicit name (not "*") because the build output directory also
# contains Python binding modules (_physx.so, _physicsTensors.so, etc.) that
# must NOT be loaded as Carbonite plugins — they are pybind11 modules imported
# via Python's import mechanism instead.
_PHYSICS_PLUGINS = [
    "omni.convexdecomposition.plugin",
    "omni.usdphysics.plugin",
    "omni.physx.gpu.plugin",
    "omni.physx.foundation.plugin",
    "omni.physx.cooking.plugin",
    "omni.physx.plugin",
]

# Windows: register ext physics lib dir so physxSchema.dll is findable when loading plugins
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    _ext_lib = _carb_setup.TARGET_DEPS / "usd_ext_physics" / "release" / "lib"
    if _ext_lib.exists():
        os.add_dll_directory(str(_ext_lib))

_bin = _carb_setup.OVRUNTIME_BIN_DIR
if _bin.exists():
    _carb_setup.framework.load_plugins(_PHYSICS_PLUGINS, search_paths=[str(_bin)])

# GPU initialization notes:
# - The omni.physx.foundation.plugin loads libPhysXGpu_64.so from its bin directory
#   during carbOnPluginStartup (via carb::extras::loadLibrary).
# - cudaDeviceCheck() uses a direct CUDA query fallback when the Graphics plugin
#   (renderer path) is not available, which is the normal case for standalone tests.
# - _physxFoundation.create_gpu_foundation() is available via the Python bindings
#   for environments that have the Graphics/gpu_foundation plugins loaded.

# --- Register PhysX USD schemas ---
# Without this, PhysX schema types (PhysxRigidBodyAPI, PhysxSceneAPI, etc.) are
# unknown to USD and schema-based APIs fail silently.  Mirrors the C++ unit test
# setup in PhysicsTools.cpp which registers via USD_EXT_PHYSICS_PLUGIN_PATH.
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

# Add PhysxSchema / PhysicsSchemaTools Python modules to sys.path and pxr namespace
_EXT_PHYSICS_PYTHON = _EXT_PHYSICS_LIB / "python"
if _EXT_PHYSICS_PYTHON.exists():
    if str(_EXT_PHYSICS_PYTHON) not in sys.path:
        sys.path.insert(0, str(_EXT_PHYSICS_PYTHON))

    # Import and inject into pxr namespace so "from pxr import PhysxSchema" works
    import pxr  # noqa: E402
    import PhysxSchema  # noqa: E402
    pxr.PhysxSchema = PhysxSchema

    import PhysicsSchemaTools  # noqa: E402
    pxr.PhysicsSchemaTools = PhysicsSchemaTools

# --- Set up omni.physx Python package for physicsUtils / utils imports ---
# The ovruntime source tree contains the omni.physx Python package with utility
# modules (physicsUtils, utils) that tests need.  These modules use Kit-style
# imports like ``from omni.physx.bindings._physx import ...`` internally.
# We bridge those imports by registering synthetic modules in sys.modules that
# delegate to the standalone _physx binding module.
import types as _types
import _physx as _physx_mod  # noqa: E402

# omni.physx.bindings._physx  ->  _physx (the standalone pybind11 module)
_bindings = _types.ModuleType("omni.physx.bindings")
_bindings._physx = _physx_mod
sys.modules["omni.physx.bindings"] = _bindings
sys.modules["omni.physx.bindings._physx"] = _physx_mod

# omni.physx  (top-level package with convenience accessors)
import omni as _omni  # noqa: E402 - already a namespace package from carb
_physx_pkg = _types.ModuleType("omni.physx")
_physx_pkg.bindings = _bindings

# Forward get_physx_*_interface helpers (mirrors ifaces.py)
def _lazy_iface(acq_fn):
    """Create a lazy-caching interface getter (same pattern as ifaces.py)."""
    cache = {}
    def getter():
        if "iface" not in cache:
            cache["iface"] = acq_fn()
        return cache["iface"]
    return getter

for _name in [
    "physx", "physxunittests", "physx_visualization", "physx_scene_query",
    "physx_cooking", "physx_cooking_private", "physx_simulation",
    "physx_benchmarks", "physx_attachment_private", "physx_property_query",
    "physx_replicator", "physx_stage_update", "physx_statistics",
]:
    _acq_name = f"acquire_{_name}_interface"
    if hasattr(_physx_mod, _acq_name):
        setattr(_physx_pkg, f"get_{_name}_interface", _lazy_iface(getattr(_physx_mod, _acq_name)))

sys.modules["omni.physx"] = _physx_pkg
_omni.physx = _physx_pkg

# omni.physx.scripts.ifaces  (used by utils.py internally)
_scripts_pkg = _types.ModuleType("omni.physx.scripts")
_PHYSX_SCRIPTS_DIR = str(_carb_setup.OVRUNTIME_DIR / "source" / "omni.physx" / "python" / "scripts")
_scripts_pkg.__path__ = [_PHYSX_SCRIPTS_DIR]
_ifaces_mod = _types.ModuleType("omni.physx.scripts.ifaces")
for _attr in dir(_physx_pkg):
    if _attr.startswith("get_physx"):
        setattr(_ifaces_mod, _attr, getattr(_physx_pkg, _attr))
sys.modules["omni.physx.scripts"] = _scripts_pkg
sys.modules["omni.physx.scripts.ifaces"] = _ifaces_mod
_physx_pkg.scripts = _scripts_pkg
_scripts_pkg.ifaces = _ifaces_mod

# Provide a stub for usdrt (not available in ovruntime, only used by one
# function in physicsUtils that standalone tests do not call).
if "usdrt" not in sys.modules:
    try:
        import usdrt  # noqa: F401
    except ImportError:
        sys.modules["usdrt"] = _types.ModuleType("usdrt")

# --- Add physicsUtils / utils scripts to sys.path for direct import ---
# Tests can now ``import physicsUtils`` and ``import utils as physxUtils``.
_OVRUNTIME_DIR = _carb_setup.OVRUNTIME_DIR
_PHYSX_SCRIPTS_DIR = _OVRUNTIME_DIR / "source" / "omni.physx" / "python" / "scripts"
if _PHYSX_SCRIPTS_DIR.exists() and str(_PHYSX_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_PHYSX_SCRIPTS_DIR))
