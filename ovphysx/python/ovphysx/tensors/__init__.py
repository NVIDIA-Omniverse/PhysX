# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""TensorAPI for batch GPU operations on PhysX simulations.

This module provides high-performance tensor views over PhysX simulation data,
enabling efficient batch operations for robotics and RL applications.

Usage:
    from ovphysx import PhysX, tensors

    physx = PhysX()
    physx.add_usd("scene.usda")
    physx.wait_all()

    stage_id = physx.get_stage_id()
    sim_view = tensors.create_simulation_view("numpy", stage_id=stage_id)
    art_view = sim_view.create_articulation_view("/World/robot*")

    # Batch operations on all robots
    positions = art_view.get_dof_positions()
    art_view.set_dof_position_targets(positions, indices)

NOTE: This module imports without side effects. Tensor plugins are loaded lazily
on first use of any tensor function, which requires PhysX() to have been created.
"""
import sys
import types
from pathlib import Path

# Cache for the real tensor implementation module
_impl_module = None


def _get_impl():
    """Load tensor plugins and return the real implementation module."""
    global _impl_module
    if _impl_module is not None:
        return _impl_module

    # Legacy TensorAPI is only shipped for a limited set of CPython minors.
    # The wheel includes pybind11 binaries for each supported minor (e.g. 3.11 and 3.12),
    # and at runtime the interpreter can only load the one matching its CPython minor.
    # We gate early (before loading plugins) so unsupported versions fail with a clear
    # error instead of a low-level binary import failure.
    if sys.version_info.major != 3 or sys.version_info.minor not in (11, 12):
        raise ImportError(
            "ovphysx.tensors (legacy TensorAPI, pybind11) is only supported on CPython 3.11 and 3.12.\n"
            f"Current interpreter: {sys.version_info.major}.{sys.version_info.minor}\n"
            "\n"
            "For Python-minor-agnostic tensor access, prefer TensorBindingsAPI via:\n"
            "  - ovphysx.PhysX.create_tensor_binding(...)\n"
            "  - see tests/python_samples/tensor_bindings_views_sample.py\n"
        )

    # Ensure PhysX is initialized and plugins are loaded
    from .. import _ensure_tensor_plugins_loaded

    _ensure_tensor_plugins_loaded()

    # Now set up the namespace and import the real tensor module
    _deps_dir = Path(__file__).parent.parent / "deps"
    _tensor_exts = _deps_dir / "v2"
    _kit_py = _deps_dir / "_kit_py"

    # Add _kit_py to path for carb/omni.ext if available
    # `omni.physics.tensors` (legacy TensorAPI) imports `omni.ext` (Kit Python) via its
    # extension entrypoint. In a plain Python environment there is no Kit install, so we
    # ship a minimal subset in `deps/_kit_py` and put it on sys.path before importing.
    if _kit_py.exists() and str(_kit_py) not in sys.path:
        sys.path.insert(0, str(_kit_py))

    # Find the omni.physics.tensors extension directory
    _tensor_ext_dir = None
    if _tensor_exts.exists():
        # Deterministic selection (important if multiple matching entries exist)
        for ext_dir in sorted(_tensor_exts.iterdir(), key=lambda p: p.name):
            if ext_dir.name.startswith("omni.physics.tensors") and "tests" not in ext_dir.name:
                _tensor_ext_dir = ext_dir
                break

    if not _tensor_ext_dir:
        raise ImportError(
            "Tensor extensions not found. Ensure the wheel was built with tensor support. "
            f"Expected extensions in: {_tensor_exts}"
        )

    _omni_dir = _tensor_ext_dir / "omni"
    _physics_dir = _omni_dir / "physics"

    # Set up omni namespace package
    if "omni" not in sys.modules:
        omni_ns = types.ModuleType("omni")
        omni_ns.__path__ = [str(_omni_dir)]
        sys.modules["omni"] = omni_ns
    else:
        omni_ns = sys.modules["omni"]
        if hasattr(omni_ns, "__path__"):
            if str(_omni_dir) not in omni_ns.__path__:
                omni_ns.__path__.insert(0, str(_omni_dir))

    # Set up omni.physics namespace package
    if "omni.physics" not in sys.modules:
        omni_physics = types.ModuleType("omni.physics")
        omni_physics.__path__ = [str(_physics_dir)]
        sys.modules["omni.physics"] = omni_physics
    else:
        omni_physics = sys.modules["omni.physics"]
        if hasattr(omni_physics, "__path__"):
            if str(_physics_dir) not in omni_physics.__path__:
                omni_physics.__path__.insert(0, str(_physics_dir))

    # Import the real tensor module
    try:
        import omni.physics.tensors as real_tensors
    except Exception as e:
        bindings_dir = _physics_dir / "tensors" / "bindings"
        available_bindings = []
        try:
            if bindings_dir.exists():
                available_bindings = sorted(p.name for p in bindings_dir.glob("_physicsTensors.*"))
        except Exception:
            # Best-effort debug info only
            available_bindings = []

        raise ImportError(
            "Failed to import legacy TensorAPI module 'omni.physics.tensors'.\n"
            "This usually means the ovphysx wheel does not include compatible pybind11 "
            "TensorAPI bindings for your current Python interpreter.\n"
            f"Python: {sys.version_info.major}.{sys.version_info.minor}\n"
            f"Expected bindings dir: {bindings_dir}\n"
            f"Available bindings: {available_bindings or '<none>'}\n"
            f"Original error: {e!r}\n"
            "\n"
            "For Python-minor-agnostic tensor access, prefer TensorBindingsAPI via:\n"
            "  - ovphysx.PhysX.create_tensor_binding(...)\n"
            "  - see tests/python_samples/tensor_bindings_views_sample.py\n"
        ) from e

    _impl_module = real_tensors
    return _impl_module


# Public API - all functions lazily load plugins on first call
def create_simulation_view(*args, **kwargs):
    """Create a simulation view for tensor operations."""
    return _get_impl().create_simulation_view(*args, **kwargs)


def acquire_tensor_api(*args, **kwargs):
    """Acquire the low-level tensor API interface."""
    return _get_impl().acquire_tensor_api(*args, **kwargs)


# Lazy attribute access for any other tensor module attributes
def __getattr__(name: str):
    """Proxy attribute access to the real tensor module."""
    impl = _get_impl()
    try:
        return getattr(impl, name)
    except AttributeError:
        raise AttributeError(f"module 'ovphysx.tensors' has no attribute '{name}'") from None


# Export commonly used types/constants directly for convenience
__all__ = [
    "create_simulation_view",
    "acquire_tensor_api",
]
