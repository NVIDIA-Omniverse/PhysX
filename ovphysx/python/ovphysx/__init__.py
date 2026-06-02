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

    Key classes: PhysX and ContactBinding.
    Full docs are in the ``docs/`` subdirectory of this package.

Important notes for agents:
    - The default ovphysx wheel bundles a namespaced, py-less OpenUSD runtime
      so it can coexist with a host that already loaded classic USD. Do not add
      ``usd-core`` as an ovphysx dependency; install it only if your application
      explicitly needs its own classic USD Python API.
    - ovphysx exchanges tensor data via DLPack, so any framework that
      understands DLPack can consume it. Common optional companions are
      numpy (CPU tensors) and torch (GPU tensors), but they are not
      dependencies of this package. Users should add whichever they need
      to their own project requirements.
    - ``import ovphysx`` appends to ``OV_PXR_PLUGINPATH_2511`` so ovphysx's
      schema plugins are visible before the first USD stage open. No native
      loading happens at import; that is still deferred until a native
      attribute (``PhysX``, ``ContactBinding``, etc.) is first accessed.
"""

# =============================================================================
# Pure-Python exports (no native loading, no USD; one OS env var is appended)
# =============================================================================
#
# Importing ovphysx makes the IntEnum types, ``__version__``, and the
# ``register_schema_paths`` / ``ai_skills_path`` / ``bootstrap`` helpers
# available. Native library loading (libcarb, USD version checks, _bindings,
# api classes) is deferred to first access of a native-dependent attribute
# via ``__getattr__``.
#
# Side effect on import: ``register_schema_paths()`` is called. It appends
# ovphysx's namespaced USD plugin path to ``OV_PXR_PLUGINPATH_2511`` (no-op
# if already present). This avoids a class of silent-schema-drop bugs in
# mixed-process apps (e.g. ovphysx + ovrtx) where USD's ``PlugRegistry`` is
# locked in by another subsystem before the user remembers to call
# ``register_schema_paths()`` explicitly. The call is pure-Python and does
# not trigger native loading. It is always-idempotent: re-reads the live
# env (Python and native views), merges, dedupes, and writes back, so it is
# safe to call repeatedly. Callers that need an env-var-pure import can pop
# ``OV_PXR_PLUGINPATH_2511`` from ``os.environ`` after the import; a later
# ``register_schema_paths()`` call will re-add ovphysx's path.

import logging as _logging

_logger = _logging.getLogger("ovphysx")
_logger.addHandler(_logging.NullHandler())

# Silence USD's TfScriptModuleLoader auto-import warnings for pxr.<Name>
# Python peers that the ovphysx wheel does not ship (PhysxSchema,
# PhysicsSchemaTools, CameraUtil, PxOsd). Must run BEFORE any code that
# triggers USD's native plugin scan -- i.e. before any native ovphysx
# attribute access via __getattr__ -- and ideally before any other USD
# library loads. See _pxr_stubs.py for the full rationale.
#
# This is a temporary runtime workaround. It is removed once the schema C++
# plugins that register pxr.PhysxSchema / pxr.PhysicsSchemaTools stop being
# built (which eliminates two of the four warnings at the source) and the
# bundled USD runtime stops registering pxr.CameraUtil / pxr.PxOsd without
# shipping their Python wrappers. When both are addressed, drop this block
# and _pxr_stubs.py entirely.
from . import _pxr_stubs as _pxr_stubs
_pxr_stubs.install_pxr_stubs()

from .types import (
    ApiStatus,
    BindingPrimMode,
    ConfigBool,
    ConfigFloat,
    ConfigInt32,
    ConfigString,
    DeviceType,
    LogLevel,
    PhysXDeviceError,
    PhysXType,
    SceneQueryGeometryType,
    SceneQueryMode,
    TensorType,
)
from .config import PhysXConfig
from .usd_version_check import register_schema_paths

# Auto-register ovphysx's USD schema/plugin paths at import time so that
# applications sharing a process with another USD-aware subsystem (e.g.
# ovrtx) do not have to remember to call register_schema_paths() manually
# before the first stage open. USD's PlugRegistry is populated lazily on
# first stage open and never re-scans; missing this window means ovphysx's
# applied schemas are silently dropped from the registry and prim.HasAPI()
# returns false thereafter.
#
# Pure-Python (env var append only) -- no native loading is triggered.
# Failures (e.g. no plugins/usd directory found in a partially-installed
# checkout) are surfaced as a logged warning rather than raised so import
# does not hard-fail; apps that need stronger guarantees can call
# register_schema_paths() explicitly in a try/except (always-idempotent,
# so the explicit call is a cheap no-op once the env var is in place).
try:
    register_schema_paths()
except Exception as _exc:  # noqa: BLE001 -- swallow any failure to a logged warning
    _logger.warning(
        "ovphysx schema-path auto-registration failed: %s. Applications that "
        "mix ovphysx with another USD-aware subsystem in the same process "
        "must call ovphysx.register_schema_paths() explicitly before the "
        "first USD stage open or schema-gated features will silently no-op.",
        _exc,
    )

# Package version resolution (PEP 440), tried in order:
#   1. _version.py -- generated by build_wheel.cmake during wheel staging.
#      Present only in installed wheels, never checked into the source tree.
#   2. Repo VERSION file -- three directories up from this file
#      (python/ovphysx/__init__.py -> omni/ovphysx/VERSION).
#      Works for editable installs and `uv run pytest` from the source tree.
#   3. "unknown" -- fallback if neither is available.
try:
    from ._version import __version__
except Exception:  # noqa: BLE001
    try:
        from pathlib import Path as _Path
        __version__ = (_Path(__file__).parent.parent.parent / "VERSION").read_text().strip()
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
    # Source-repo fallback: python/ovphysx/../../ == omni/ovphysx project root
    project_root = pkg.parent.parent
    return {
        "skills_index": project_root / "SKILLS.md",
        "skills_dir": project_root / "skills",
        "samples_dir": project_root / "tests" / "python_samples",
        "docs_dir": project_root / "docs",
    }


# =============================================================================
# Lazy native loading -- triggered on first access to PhysX, etc.
# =============================================================================

_native_bootstrapped = False
_native_bootstrap_error: BaseException | None = None

def _bootstrap_native():
    """One-time native library loading. Deferred until first access to a native attr.

    On failure the original exception is stored and re-raised on every subsequent
    call, so callers always see the real error (not a confusing AttributeError).
    """
    global _native_bootstrapped, _native_bootstrap_error
    if _native_bootstrapped:
        if _native_bootstrap_error is not None:
            raise _native_bootstrap_error
        return
    _native_bootstrapped = True

    try:
        _bootstrap_native_impl()
    except BaseException as exc:
        _native_bootstrap_error = exc
        raise


def _bootstrap_native_impl():
    """One-time native bootstrap. Loads libraries and populates module globals.

    Steps (in order):
      - Pre-load libcarb.so with RTLD_GLOBAL so Carbonite plugins can resolve symbols.
      - Run USD version compatibility check.
      - Import and inject all native-dependent names into module globals.
      - Drift guard: assert every __all__ entry is now resolvable.

    Raises RuntimeError on any failure (missing library, USD version mismatch, etc.).
    """
    import ctypes
    import os
    import sys
    import warnings
    from pathlib import Path

    module_dir = Path(__file__).parent
    lib_dir = module_dir / "lib"
    plugins_dir = module_dir / "plugins"

    # Wheel mode: libovphysx.so bundled in lib/ means we're running from an installed wheel.
    # Wheel structure: lib/ (libovphysx.so, config.toml), plugins/ (carbonite/physx plugins, usd/).
    lib_file = lib_dir / ("ovphysx.dll" if sys.platform == "win32" else "libovphysx.so")
    is_wheel_mode = lib_file.exists()

    if is_wheel_mode:
        if sys.platform == "win32":
            os.add_dll_directory(str(lib_dir))
            os.add_dll_directory(str(plugins_dir))

        def _warn_if_missing(path: Path, label: str) -> None:
            if not path.exists():
                warnings.warn(
                    f"[ovphysx] Missing bundled {label}: {path}. "
                    "Wheel installation may be incomplete.",
                    RuntimeWarning,
                )

        _warn_if_missing(lib_dir, "lib directory")
        _warn_if_missing(plugins_dir, "plugins directory")
        _warn_if_missing(lib_dir / "config.toml", "config.toml")
        if sys.platform == "win32":
            _warn_if_missing(lib_dir / "ovphysx_clone.dll", "ovphysx_clone.dll")
        else:
            _warn_if_missing(lib_dir / "libovphysx_clone.so", "libovphysx_clone.so")

    # Pre-load libraries with RTLD_GLOBAL so Carbonite plugins can find symbols.
    # Store references in sys to survive module cleanup.
    if not hasattr(sys, "_omni_physx_preloaded_libs"):
        sys._omni_physx_preloaded_libs = []

    if is_wheel_mode:
        carb_lib_name = "carb.dll" if sys.platform == "win32" else "libcarb.so"
        carb_lib_path = plugins_dir / carb_lib_name

        if carb_lib_path.exists():
            try:
                if sys.platform == "win32":
                    carb_lib = ctypes.CDLL(str(carb_lib_path))
                else:
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

    # USD version check must happen BEFORE importing any code that uses USD.
    # Validates already-loaded USD; otherwise defers to C++ for USD loading.
    try:
        from .usd_version_check import check_usd_compatibility
        check_usd_compatibility()
    except Exception as e:
        raise RuntimeError(f"USD version compatibility check failed: {e}") from e

    # Import native-dependent modules and inject into our module globals
    g = globals()

    from ._bindings import OP_INDEX_ALL
    from .contact_types import ContactEventHeader, ContactPoint, FrictionAnchor
    g["OP_INDEX_ALL"] = OP_INDEX_ALL
    g["ContactEventHeader"] = ContactEventHeader
    g["ContactPoint"] = ContactPoint
    g["FrictionAnchor"] = FrictionAnchor

    from .api import (
        PhysX,
        ContactBinding,
        configure_azure_sas,
        configure_s3,
        disable_python_logging,
        enable_default_log_output,
        enable_python_logging,
        get_log_level,
        set_log_level,
    )
    g["PhysX"] = PhysX
    g["ContactBinding"] = ContactBinding
    g["configure_s3"] = configure_s3
    g["configure_azure_sas"] = configure_azure_sas
    g["set_log_level"] = set_log_level
    g["get_log_level"] = get_log_level
    g["enable_default_log_output"] = enable_default_log_output
    g["enable_python_logging"] = enable_python_logging
    g["disable_python_logging"] = disable_python_logging

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
    g["DLPACK_VERSION"] = DLPACK_VERSION
    g["DLDataType"] = DLDataType
    g["DLDataTypeCode"] = DLDataTypeCode
    g["DLDevice"] = DLDevice
    g["DLDeviceType"] = DLDeviceType
    g["DLManagedTensor"] = DLManagedTensor
    g["DLTensor"] = DLTensor
    g["ManagedDLTensor"] = ManagedDLTensor

    # Drift guard: every public name in __all__ must now be resolvable.
    missing = [name for name in __all__ if name not in g]
    if missing:
        raise RuntimeError(
            f"_bootstrap_native() did not populate these __all__ entries: {missing}"
        )


def bootstrap() -> None:
    """Explicitly trigger native library loading.

    Normally this happens automatically on first access to any native
    attribute (``PhysX``, ``ContactBinding``, etc.).  Call this
    when you need to control the exact moment loading occurs -- for example,
    to ensure it happens before other USD-aware code runs or before
    environment variables that affect USD plugin discovery are modified.

    Idempotent: safe to call multiple times; loading happens at most once.

    Raises:
        RuntimeError: If native library loading fails (e.g. missing shared
            library, incompatible USD version already loaded in the process).
    """
    _bootstrap_native()


def __getattr__(name):
    if name in __all__:
        _bootstrap_native()
        try:
            return globals()[name]
        except KeyError:
            pass
    raise AttributeError(f"module 'ovphysx' has no attribute {name!r}")


# Kept explicit (not assembled from submodule __all__s) so the full public API
# surface is readable in one place. The drift guard in _bootstrap_native_impl()
# ensures this list stays in sync with what actually gets injected into globals.
__all__ = [
    "__version__",
    # IntEnum types and exceptions (pure Python, always available)
    "TensorType",
    "LogLevel",
    "ApiStatus",
    "DeviceType",
    "BindingPrimMode",
    "PhysXType",
    "PhysXDeviceError",
    "SceneQueryMode",
    "SceneQueryGeometryType",
    # Config types (pure Python, always available)
    "PhysXConfig",
    "ConfigBool",
    "ConfigInt32",
    "ConfigFloat",
    "ConfigString",
    # Eager initialization (module-level, always available)
    "bootstrap",
    "register_schema_paths",
    "ai_skills_path",
    # Core API (native, lazy-loaded)
    "PhysX",
    "ContactBinding",
    # Remote storage credentials
    "configure_s3",
    "configure_azure_sas",
    # Logging API
    "set_log_level",
    "get_log_level",
    "enable_default_log_output",
    "enable_python_logging",
    "disable_python_logging",
    # Contact report structs (pure-Python ctypes mirrors of C ABI structs)
    "ContactEventHeader",
    "ContactPoint",
    "FrictionAnchor",
    # Operation index sentinel
    "OP_INDEX_ALL",
    # DLPack structures
    "DLDevice",
    "DLDataType",
    "DLDataTypeCode",
    "DLDeviceType",
    "DLTensor",
    "DLManagedTensor",
    "ManagedDLTensor",
    "DLPACK_VERSION",
]
