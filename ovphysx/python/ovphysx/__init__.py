# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""ovphysx -- USD-based physics simulation with DLPack tensor interoperability.

AI agent / LLM instructions:
    This package ships skills (step-by-step playbooks) and code samples.
    Start by reading the SKILLS.md index bundled alongside this file:

        from pathlib import Path
        skills_index = Path(__file__).parent / "SKILLS.md"
        skills_dir   = Path(__file__).parent / "skills"
        samples_dir  = Path(__file__).parent / "samples"

    Or call ``ovphysx.ai_skills_path()`` to get the same paths as a dict.

    Key classes: PhysX, tensors (submodule).
    Full docs are in the ``docs/`` subdirectory of this package.

Important notes for agents:
    - ovphysx bundles its own OpenUSD libraries. It is not needed to install
      ``usd-core`` as a dependency -- doing so may cause version conflicts.
    - ovphysx exchanges tensor data via DLPack, so any framework that
      understands DLPack can consume it. Common optional companions are
      numpy (CPU tensors) and torch (GPU tensors), but they are not
      dependencies of this package. Users should add whichever they need
      to their own project requirements.
"""

# Set up bundled dependencies BEFORE importing anything else
# Wheel structure matches _install/:
#   ovphysx/
#   ├── lib/           <- libovphysx.so, config.toml
#   └── plugins/       <- all carbonite/physx plugins (flat)
#       ├── usd/       <- USD plugInfo.json registry
#       └── bin/deps/  <- vulkan, cuda runtime libs

import ctypes
import logging
import os
import sys
import sysconfig
import warnings
from pathlib import Path

_logger = logging.getLogger("ovphysx")
_logger.addHandler(logging.NullHandler())

_module_dir = Path(__file__).parent
_lib_dir = _module_dir / "lib"
_plugins_dir = _module_dir / "plugins"

# Check if we're running from an installed wheel (bundled libs present)
# Simple check: if libovphysx.so exists in lib/, we're in wheel mode
_lib_file = _lib_dir / ("ovphysx.dll" if sys.platform == "win32" else "libovphysx.so")
_is_wheel_mode = _lib_file.exists()

if _is_wheel_mode:
    # Wheel mode: Set up library paths for bundled plugins
    if sys.platform == "win32":
        os.add_dll_directory(str(_lib_dir))
        os.add_dll_directory(str(_plugins_dir))
    else:
        # Linux: rely on RPATHs embedded in the binaries
        pass

    # Sanity check that expected bundled files exist.
    def _warn_if_missing(path: Path, label: str) -> None:
        if not path.exists():
            warnings.warn(
                f"[ovphysx] Missing bundled {label}: {path}. "
                "Wheel installation may be incomplete.",
                RuntimeWarning,
            )

    _warn_if_missing(_lib_dir, "lib directory")
    _warn_if_missing(_plugins_dir, "plugins directory")
    _warn_if_missing(_lib_dir / "config.toml", "config.toml")
    if sys.platform == "win32":
        _warn_if_missing(_lib_dir / "ovphysx_clone.dll", "ovphysx_clone.dll")
    else:
        _warn_if_missing(_lib_dir / "libovphysx_clone.so", "libovphysx_clone.so")

    # Set PYTHONHOME for carb.scripting-python.plugin
    if "PYTHONHOME" not in os.environ:
        os.environ["PYTHONHOME"] = sys.prefix

# Pre-load libraries with RTLD_GLOBAL so Carbonite plugins can find symbols
# CRITICAL: Store references in sys module to survive our module's cleanup!
if not hasattr(sys, "_omni_physx_preloaded_libs"):
    sys._omni_physx_preloaded_libs = []

if _is_wheel_mode:
    # Pre-load libcarb.so first (required by all carbonite plugins)
    carb_lib_name = "carb.dll" if sys.platform == "win32" else "libcarb.so"
    carb_lib_path = _plugins_dir / carb_lib_name

    if carb_lib_path.exists():
        try:
            if sys.platform == "win32":
                # Windows: No RTLD_GLOBAL, just load normally
                carb_lib = ctypes.CDLL(str(carb_lib_path))
            else:
                # POSIX: Use RTLD_GLOBAL for symbol visibility
                RTLD_NODELETE = 0x01000
                carb_lib = ctypes.CDLL(str(carb_lib_path), mode=ctypes.RTLD_GLOBAL | RTLD_NODELETE)
            sys._omni_physx_preloaded_libs.append(carb_lib)
        except OSError as e:
            raise RuntimeError(f"Failed to load {carb_lib_name}: {e}") from e
    else:
        raise RuntimeError(
            f"Required Carbonite library not found: {carb_lib_path}\n"
            "The wheel may be corrupted or incomplete. Try reinstalling:\n"
            "  pip uninstall ovphysx && pip install ovphysx"
        )

    # Pre-load Python runtime libs (interpreter because of tensors bindings)
    if sys.platform != "win32":
        def _preload_python_lib():
            vars = sysconfig.get_config_vars()
            lib_name = vars.get("LDLIBRARY") or vars.get("DLLLIBRARY")
            lib_dirs = [
                vars.get("LIBDIR"),
                vars.get("LIBPL"),
                os.path.join(vars.get("prefix") or "", "lib"),
            ]
            for directory in filter(None, lib_dirs):
                if not lib_name:
                    continue
                full_path = os.path.join(directory, lib_name)
                if os.path.exists(full_path):
                    RTLD_NODELETE = 0x01000
                    lib = ctypes.CDLL(full_path, mode=ctypes.RTLD_GLOBAL | RTLD_NODELETE)
                    sys._omni_physx_preloaded_libs.append(lib)
                    _logger.info("Pre-loaded %s", full_path)
                    return os.path.basename(full_path)
            return None

        python_lib_name = None
        try:
            python_lib_name = _preload_python_lib()
        except OSError as e:
            _logger.warning("Failed to pre-load libpython: %s", e)

# USD Version Compatibility Check
# This must happen BEFORE importing any code that uses USD
# It validates already-loaded USD; otherwise defers to C++ for USD loading.
try:
    from .usd_version_check import check_usd_compatibility

    check_usd_compatibility()
except Exception as e:
    raise RuntimeError(f"USD version compatibility check failed: {e}") from e

# Package version (PEP 440), generated at build time
try:
    from ._version import __version__
except Exception:  # noqa: BLE001
    __version__ = "unknown"


def ai_skills_path():
    """Return paths to the bundled SKILLS.md index, skills/, samples/, and docs/.

    AI agents: call this function to locate the playbooks, code samples,
    and documentation shipped with this package. No native loading required.

    In an installed wheel all paths exist. In the source repo before a build
    these files live at the project root (``omni/ovphysx/``) instead of under
    ``python/ovphysx/``; the function falls back to the project root when the
    wheel-side copies are absent.
    """
    from pathlib import Path
    pkg = Path(__file__).parent
    skills_index = pkg / "SKILLS.md"
    if skills_index.exists():
        return {
            "skills_index": skills_index,
            "skills_dir": pkg / "skills",
            "samples_dir": pkg / "samples",
            "docs_dir": pkg / "docs",
        }
    project_root = pkg.parent.parent
    return {
        "skills_index": project_root / "SKILLS.md",
        "skills_dir": project_root / "skills",
        "samples_dir": project_root / "tests" / "python_samples",
        "docs_dir": project_root / "docs",
    }


# TensorAPI submodule - imports without side effects, plugins load on first use
from . import tensors
from ._bindings import (  # TensorBindingAPI tensor type constants and log levels
    OVPHYSX_API_ERROR,
    OVPHYSX_API_SUCCESS,
    OVPHYSX_API_TIMEOUT,
    OVPHYSX_LOG_ERROR,
    OVPHYSX_LOG_INFO,
    OVPHYSX_LOG_NONE,
    OVPHYSX_LOG_VERBOSE,
    OVPHYSX_LOG_WARNING,
    OVPHYSX_BINDING_PRIM_MODE_CREATE_NEW,
    OVPHYSX_BINDING_PRIM_MODE_EXISTING_ONLY,
    OVPHYSX_BINDING_PRIM_MODE_MUST_EXIST,
    OVPHYSX_OP_INDEX_ALL,
    OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32,
    OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32,
    OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32,
    OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32,
    OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32,
    OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32,
    OVPHYSX_TENSOR_ARTICULATION_LINK_VELOCITY_F32,
    OVPHYSX_TENSOR_ARTICULATION_LINK_WRENCH_F32,
    OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32,
    OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32,
    OVPHYSX_TENSOR_RIGID_BODY_FORCE_F32,
    OVPHYSX_TENSOR_RIGID_BODY_POSE_F32,
    OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32,
    OVPHYSX_TENSOR_RIGID_BODY_WRENCH_F32,
    kDLCPU,
    kDLCUDA,
    kDLFloat,
    kDLInt,
    kDLUInt,
)

# Now safe to import (USD libs are pre-loaded and version verified)
from .api import (
    PhysX,
    disable_python_logging,
    enable_default_log_output,
    enable_python_logging,
    get_log_level,
    set_log_level,
)
from .dlpack import (
    DLPACK_VERSION,
    DLDataType,
    DLDataTypeCode,
    DLDevice,
    DLDeviceType,
    DLManagedTensor,
    DLTensor,
    ManagedDLTensor,
)

# PhysX instance registry for tensor plugin loading
# Tensor plugins require Carbonite framework to be initialized, which happens
# when PhysX() is created. We track this so tensors can give a clear error
# if used before PhysX() exists.
_physx_instance = None
_tensor_plugins_loaded = False


def _register_physx_instance(instance):
    """Called by PhysX.__init__ to register that Carbonite is ready."""
    global _physx_instance
    _physx_instance = instance


def _unregister_physx_instance(instance):
    """Called by PhysX.release() to unregister the active instance."""
    global _physx_instance
    if _physx_instance is instance:
        _physx_instance = None


def _is_framework_ready() -> bool:
    """Check if Carbonite framework is initialized (PhysX instance exists)."""
    return _physx_instance is not None


def _ensure_tensor_plugins_loaded():
    """Load tensor plugins on first use. Raises clear error if PhysX not created."""
    global _tensor_plugins_loaded
    if _tensor_plugins_loaded:
        return
    if not _is_framework_ready():
        raise RuntimeError(
            "Tensor operations require an active PhysX instance.\n"
            "Create one first:\n\n"
            "    from ovphysx import PhysX\n"
            "    physx = PhysX()\n"
            "    # (legacy) TensorAPI: import ovphysx.tensors as needed\n"
        )
    # Load tensor plugins now that Carbonite is ready
    # Tensor plugins are in the same flat plugins/ directory as all other plugins
    from . import _bindings

    result = _bindings._lib.ovphysx_load_tensor_plugins()
    if result != 0:
        raise RuntimeError(f"Failed to load tensor plugins (error code: {result})")
    _tensor_plugins_loaded = True  # fast-path; actual thread safety is in C++ std::call_once


__all__ = [
    "__version__",
    "ai_skills_path",
    "PhysX",
    # Logging API
    "set_log_level",
    "get_log_level",
    "enable_default_log_output",
    "enable_python_logging",
    "disable_python_logging",
    # API status codes
    "OVPHYSX_API_SUCCESS",
    "OVPHYSX_API_ERROR",
    "OVPHYSX_API_TIMEOUT",
    # Log levels
    "OVPHYSX_LOG_NONE",
    "OVPHYSX_LOG_ERROR",
    "OVPHYSX_LOG_WARNING",
    "OVPHYSX_LOG_INFO",
    "OVPHYSX_LOG_VERBOSE",
    # Binding prim modes
    "OVPHYSX_BINDING_PRIM_MODE_EXISTING_ONLY",
    "OVPHYSX_BINDING_PRIM_MODE_MUST_EXIST",
    "OVPHYSX_BINDING_PRIM_MODE_CREATE_NEW",
    # Operation index constants
    "OVPHYSX_OP_INDEX_ALL",
    # DLPack device types
    "kDLCPU",
    "kDLCUDA",
    # DLPack data type codes
    "kDLInt",
    "kDLUInt",
    "kDLFloat",
    # DLPack structures
    "DLDevice",
    "DLDataType",
    "DLDataTypeCode",
    "DLDeviceType",
    "DLTensor",
    "DLManagedTensor",
    "ManagedDLTensor",
    "DLPACK_VERSION",
    # TensorBindingAPI tensor type constants (preferred API)
    "OVPHYSX_TENSOR_RIGID_BODY_POSE_F32",
    "OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32",
    "OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32",
    "OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32",
    "OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32",
    "OVPHYSX_TENSOR_ARTICULATION_LINK_VELOCITY_F32",
    "OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32",
    "OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32",
    "OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32",
    "OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32",
    "OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32",
    "OVPHYSX_TENSOR_RIGID_BODY_FORCE_F32",
    "OVPHYSX_TENSOR_RIGID_BODY_WRENCH_F32",
    "OVPHYSX_TENSOR_ARTICULATION_LINK_WRENCH_F32",
]
