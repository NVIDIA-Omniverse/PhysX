# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# Tensor plugin initialization for standalone Python tests.
#
# Builds on _physics_setup (Carbonite + PhysX plugins) by loading the tensor
# API plugins and bridging Kit-style "omni.physics.tensors" imports to the
# standalone _physicsTensors pybind11 module.
#
# Usage:
#     import _tensors_setup  # noqa: F401 - loads tensor plugins
#     import omni.physics.tensors

import sys
import types as _types
import importlib
import _physics_setup  # noqa: F401 - loads Carbonite + PhysX plugins
import _carb_setup

# --- Load tensor plugins ---
# These implement the tensor API backend (PhysX-backed tensor views).
_TENSOR_PLUGINS = [
    "omni.physics.tensors.plugin",
    "omni.physx.tensors.plugin",
]

_bin = _carb_setup.OVRUNTIME_BIN_DIR
if _bin.exists():
    _carb_setup.framework.load_plugins(_TENSOR_PLUGINS, search_paths=[str(_bin)])

# --- Load fabric plugin (needed by fabric-sync tests) ---
_FABRIC_PLUGINS = [
    "omni.physx.fabric.plugin",
]
_carb_setup.framework.load_plugins(_FABRIC_PLUGINS, search_paths=[str(_bin)])

# --- Initialize Warp BEFORE starting the tensor plugin ---
# The omni.physx.tensors.plugin startup loads shared libraries that shadow the
# CUDA driver symbols (cuInit, cuDeviceGetCount, etc.) with stubs.  If Warp
# hasn't already resolved the real CUDA driver at this point, it will see the
# stubs and conclude that no GPU is available.  Calling wp.init() first forces
# Warp to cache the real driver entry points so they survive the plugin load.
import warp as _wp
_wp.init()

# --- Explicitly start omni.physx.tensors.plugin ---
# Carbonite lazily starts plugins when their interfaces are first acquired.
# omni.physx.tensors.plugin only exports IExt (used by Kit's extension manager),
# so in standalone mode nobody acquires it and carbOnPluginStartup() — which
# registers the "physx" simulation backend — is never called.  Explicitly
# starting the plugin forces the backend registration.
_carb_setup.framework.start_plugin("omni.physx.tensors.plugin")

# --- Bridge omni.physics.tensors module hierarchy ---
# The tensor tests import "omni.physics.tensors" which in Kit resolves to a
# Python package.  For standalone tests we construct the same module hierarchy
# backed by the standalone _physicsTensors pybind11 module and the Python
# source in the omni.physics.tensors extension directory.

import _physicsTensors as _pt_mod  # noqa: E402

# The tensors __init__.py does:
#   from omni.physics.tensors.bindings._physicsTensors import acquire_tensor_api
# We pre-register these synthetic modules so the import resolves to _physicsTensors.
_bindings_pkg = _types.ModuleType("omni.physics.tensors.bindings")
_bindings_pkg._physicsTensors = _pt_mod
sys.modules["omni.physics.tensors.bindings"] = _bindings_pkg
sys.modules["omni.physics.tensors.bindings._physicsTensors"] = _pt_mod

# Point to the actual Python source for omni.physics.tensors
_TENSORS_PY_DIR = _carb_setup.OVRUNTIME_DIR / "source" / "omni.physics.tensors" / "python"

# Build the omni -> omni.physics -> omni.physics.tensors package hierarchy.
# omni is already a namespace package from carb/physx setup.
import omni  # noqa: E402

# Create omni.physics as a proper package (with __path__) so sub-packages resolve
if "omni.physics" not in sys.modules:
    _physics_pkg = _types.ModuleType("omni.physics")
    _physics_pkg.__path__ = []
    _physics_pkg.__package__ = "omni.physics"
    sys.modules["omni.physics"] = _physics_pkg
    omni.physics = _physics_pkg
else:
    _physics_pkg = sys.modules["omni.physics"]
    if not hasattr(_physics_pkg, "__path__"):
        _physics_pkg.__path__ = []

# Create omni.physics.tensors as a package backed by the source directory.
# We import __init__.py content manually to avoid import system confusion.
_tensors_pkg = _types.ModuleType("omni.physics.tensors")
_tensors_pkg.__path__ = [str(_TENSORS_PY_DIR)]
_tensors_pkg.__package__ = "omni.physics.tensors"
_tensors_pkg.__file__ = str(_TENSORS_PY_DIR / "__init__.py")
sys.modules["omni.physics.tensors"] = _tensors_pkg
_physics_pkg.tensors = _tensors_pkg

# Pre-populate with _physicsTensors exports (types like float32, uint8, etc.).
# Kit's extension system does this automatically; we replicate it so api.py
# can resolve `from omni.physics.tensors import float32, ...` during __init__
# execution (which triggers a circular import back to this package).
for _attr in dir(_pt_mod):
    if not _attr.startswith("_"):
        setattr(_tensors_pkg, _attr, getattr(_pt_mod, _attr))

# Now exec the __init__.py content within the package context.
# This also loads .api via relative import (from .api import ...).
_init_file = _TENSORS_PY_DIR / "__init__.py"
if _init_file.exists():
    _code = compile(_init_file.read_text(), str(_init_file), "exec")
    exec(_code, _tensors_pkg.__dict__)

# Also set up omni.physics.tensors.impl.api alias used by test isinstance checks.
# In Kit the Python files live under an "impl" subpackage.
_impl_pkg = _types.ModuleType("omni.physics.tensors.impl")
_impl_pkg.__path__ = []
_impl_pkg.__package__ = "omni.physics.tensors.impl"
_impl_pkg.api = sys.modules.get("omni.physics.tensors.api", _tensors_pkg)
sys.modules["omni.physics.tensors.impl"] = _impl_pkg
sys.modules["omni.physics.tensors.impl.api"] = _impl_pkg.api
_tensors_pkg.impl = _impl_pkg

# --- Bridge omni.physxfabric for fabric-sync tests ---
try:
    import _physxFabric as _pxf_mod  # noqa: E402
    _physxfabric_pkg = _types.ModuleType("omni.physxfabric")

    # Forward get_physx_fabric_interface helper
    def _get_physx_fabric_interface():
        if not hasattr(_get_physx_fabric_interface, "_cache"):
            _get_physx_fabric_interface._cache = _pxf_mod.acquire_physx_fabric_interface()
        return _get_physx_fabric_interface._cache
    _physxfabric_pkg.get_physx_fabric_interface = _get_physx_fabric_interface

    sys.modules["omni.physxfabric"] = _physxfabric_pkg
    omni.physxfabric = _physxfabric_pkg
except ImportError:
    pass
