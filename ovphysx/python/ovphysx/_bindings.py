# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Low-level ctypes bindings for the ovphysx library.

This module handles library loading and defines C structures and function prototypes.
"""

import ctypes
import glob
import importlib
import importlib.util
import logging
import os
import sys
from ctypes import (
    POINTER,
    c_char_p,
    c_double,
    c_float,
    c_int,
    c_int32,
    c_int64,
    c_size_t,
    c_uint8,
    c_uint16,
    c_uint32,
    c_uint64,
    c_void_p,
)
from importlib import resources as importlib_resources
from typing import Callable

_logger = logging.getLogger(__name__)
_dll_directory_handles: list[object] = []
_runtime_library_handles: list[object] = []


def _preload_windows_cpp_runtime() -> None:
    """Load the system MSVC C++ runtime before dependency search paths can shadow it."""
    if sys.platform != "win32":
        return
    system_root = os.environ.get("SystemRoot", r"C:\Windows")
    runtime_path = os.path.join(system_root, "System32", "msvcp140.dll")
    if not os.path.isfile(runtime_path):
        return
    try:
        _runtime_library_handles.append(ctypes.WinDLL(runtime_path))
        _logger.debug("Preloaded system MSVC C++ runtime: %s", runtime_path)
    except OSError as exc:
        _logger.debug("Could not preload system MSVC C++ runtime %s: %s", runtime_path, exc)


def _add_dll_directory(path: str) -> None:
    if os.path.isdir(path):
        _dll_directory_handles.append(os.add_dll_directory(path))
        _logger.debug("Added DLL directory: %s", path)


def _ovstage_bin_dir(pkg_dir: str) -> str:
    """ovstage runtime dir; find_spec locates the package without importing it."""
    hint = os.environ.get("OVSTAGE_LIBRARY_PATH_HINT")
    if hint and os.path.isdir(hint):
        return hint
    spec = importlib.util.find_spec("ovstage")
    if spec is not None and spec.origin:
        return os.path.join(os.path.dirname(spec.origin), "bin")
    return os.path.join(os.path.dirname(pkg_dir), "ovstage", "bin")


def _prefer_ovstage_runtime_dir(path: str) -> None:
    """Make ovstage's Python loader reuse the ovstage runtime paired with ovphysx."""
    if not path:
        return
    lib_name = "ovstage.dll" if sys.platform == "win32" else "libovstage.so"
    if not os.path.exists(os.path.join(path, lib_name)):
        return
    os.environ.setdefault("OVSTAGE_LIBRARY_PATH_HINT", path)
    try:
        ovstage_bindings = importlib.import_module("ovstage._src.bindings")
        if not getattr(ovstage_bindings, "OVSTAGE_LIBRARY_PATH_HINT", None):
            ovstage_bindings.OVSTAGE_LIBRARY_PATH_HINT = path
    except Exception as exc:  # noqa: BLE001 -- best-effort hint; ovstage import is optional
        _logger.debug("Could not hint ovstage runtime dir to %s: %s", path, exc)


def _reuse_loaded_runtime_dependency(path: str) -> bool:
    """Promote and retain an already-loaded Linux dependency by SONAME."""
    if sys.platform != "linux" or not hasattr(os, "RTLD_NOLOAD"):
        return False
    mode = os.RTLD_NOW | os.RTLD_NOLOAD | os.RTLD_GLOBAL
    try:
        handle = ctypes.CDLL(os.path.basename(path), mode=mode)
    except OSError:
        return False
    _runtime_library_handles.append(handle)
    _logger.debug("Reusing already-loaded runtime dependency: %s", os.path.basename(path))
    return True


def _preload_ovstage_runtime_deps() -> None:
    """Preload the ovstage-provided USD runtime so libovphysx's cross-wheel deps resolve.

    In the two-wheel split the slim ovphysx wheel does not bundle the USD monolith
    or libovstage; libovphysx.so links them (NEEDED libovstage.so + libov_*usd_ms.so)
    and they ship in the separately-installed ovstage wheel under
    ``site-packages/ovstage/bin[/plugins]``. On Linux the ELF loader resolves NEEDED
    entries via libovphysx's RPATH ($ORIGIN), which cannot reach a sibling Python
    package, so ``import ovphysx`` would fail with "libovstage.so / libov_*usd_ms.so
    not found". Preload them here (RTLD_GLOBAL) so the loader binds libovphysx's
    NEEDED entries to these already-mapped objects by soname.

    Best-effort and Linux-only: Windows uses os.add_dll_directory in _load_library,
    and external SDK/dev overrides resolve these via the install tree's own RPATH.
    """
    if sys.platform == "win32":
        return
    try:
        import ovstage  # noqa: PLC0415 -- optional runtime dependency
    except Exception as exc:  # noqa: BLE001
        _logger.debug("ovstage not importable; skipping ovstage runtime preload: %s", exc)
        return
    ov_bin = os.path.join(os.path.dirname(ovstage.__file__), "bin")
    if not os.path.isdir(ov_bin):
        return
    _prefer_ovstage_runtime_dir(ov_bin)
    # Monolith first (libovstage depends on it), then libovstage.
    candidates = sorted(glob.glob(os.path.join(ov_bin, "plugins", "libov_*usd_ms.so")))
    candidates.append(os.path.join(ov_bin, "libovstage.so"))
    for dep in candidates:
        if os.path.exists(dep):
            if _reuse_loaded_runtime_dependency(dep):
                continue
            try:
                _runtime_library_handles.append(ctypes.CDLL(dep, mode=os.RTLD_GLOBAL))
                _logger.debug("Preloaded ovstage runtime dependency: %s", dep)
            except OSError as exc:
                _logger.debug("Could not preload ovstage runtime dependency %s: %s", dep, exc)


from .dlpack import (
    DLDataType,
    DLDataTypeCode,
    DLDevice,
    DLDeviceType,
    DLManagedTensor,
    DLTensor,
)

# Invalid handle sentinel (matches OVPHYSX_INVALID_HANDLE in ovphysx_types.h)
OVPHYSX_INVALID_HANDLE = 0

# Operation index sentinel for wait_op (wait for all pending operations)
OP_INDEX_ALL = 0xFFFFFFFFFFFFFFFF


def _platform_lib_names() -> list[str]:
    """Return candidate shared library names for the current platform (Windows or Linux)."""
    if sys.platform == "win32":
        return ["ovphysx.dll"]
    return ["libovphysx.so"]


def _bundled_lib_path() -> str | None:
    candidates = _platform_lib_names()

    pkg_files = importlib_resources.files(__package__)

    # Check lib/ subdirectory first (wheel structure matches _install/)
    for name in candidates:
        p = pkg_files / "lib" / name
        try:
            with importlib_resources.as_file(p) as fp:
                if fp.exists():
                    return str(fp)
        except FileNotFoundError:
            continue
        except Exception:
            continue

    # Fallbacks for development/editable installs
    base_dir = os.path.dirname(__file__)

    # Try lib/ subdirectory (wheel structure)
    for name in candidates:
        p = os.path.join(base_dir, "lib", name)
        if os.path.exists(p):
            return p

    # Try deps/ subdirectory (legacy)
    for name in candidates:
        p = os.path.join(base_dir, "deps", name)
        if os.path.exists(p):
            return p

    # Try root directory (development mode)
    for name in candidates:
        p = os.path.join(base_dir, name)
        if os.path.exists(p):
            return p

    return None


def _same_library_path(first: str, second: str) -> bool:
    """Return whether two paths resolve to the same shared library."""
    normalized_first = os.path.normcase(os.path.realpath(os.path.abspath(first)))
    normalized_second = os.path.normcase(os.path.realpath(os.path.abspath(second)))
    return normalized_first == normalized_second


def _wheel_lib_path() -> str | None:
    """Return the native library path only for the installed wheel layout."""
    package_lib_dir = os.path.join(os.path.dirname(__file__), "lib")
    for name in _platform_lib_names():
        path = os.path.join(package_lib_dir, name)
        if os.path.isfile(path):
            return path
    return None


def _load_library() -> ctypes.CDLL:
    """Load libovphysx shared library.

    Order:
    - OVPHYSX_LIB (absolute path) - takes precedence for development mode
    - Packaged/bundled binary inside this wheel (platform-specific name)
    - Default linker paths: platform name
    """
    # On Windows, add dependency directories to the DLL search path so that
    # ovphysx.dll's transitive runtime dependencies in lib/ and plugins/ are
    # discoverable.  This makes _bindings.py self-sufficient - it doesn't
    # rely on __init__.py's bootstrap having run first.
    # On Linux, rpath=$ORIGIN covers this wheel's own plugins; the cross-wheel
    # ovstage deps (libovstage + USD monolith) are preloaded by
    # _preload_ovstage_runtime_deps() below.
    lib_env = os.environ.get("OVPHYSX_LIB")
    bundled = _bundled_lib_path()
    wheel_lib = _wheel_lib_path()
    uses_bundled_lib = not lib_env or (wheel_lib is not None and _same_library_path(lib_env, wheel_lib))
    if lib_env and not uses_bundled_lib:
        # OVPHYSX_LIB selects a complete SDK runtime, not just libovphysx.
        # Keep later ovstage Python loads on that same module directory.
        _prefer_ovstage_runtime_dir(os.path.dirname(os.path.abspath(lib_env)))

    if sys.platform == "win32":
        _preload_windows_cpp_runtime()
        pkg_dir = os.path.dirname(__file__)
        ovstage_bin_dir = _ovstage_bin_dir(pkg_dir)
        try:
            if lib_env and not uses_bundled_lib:
                # Derive SDK root from OVPHYSX_LIB (<root>/bin/ovphysx.dll on Windows,
                # <root>/lib/libovphysx.so on Linux). In SDK install mode the DLL
                # imports dependencies from sibling plugin directories before
                # Python has loaded ovphysx.
                sdk_root = os.path.dirname(os.path.dirname(lib_env))
                _add_dll_directory(os.path.join(sdk_root, "bin"))
                _add_dll_directory(os.path.join(sdk_root, "plugins"))
                _add_dll_directory(os.path.join(sdk_root, "plugins", "gpu"))
                _add_dll_directory(os.path.join(sdk_root, "plugins", "bin", "deps"))

                for _cfg in ("debug", "release", "checked"):
                    kit_sdk_dir = os.path.join(sdk_root, "target-deps", f"kit_sdk_{_cfg}")
                    if os.path.exists(kit_sdk_dir):
                        _add_dll_directory(kit_sdk_dir)
                        _logger.info("Adding kit SDK DLL directory from OVPHYSX_LIB: %s", kit_sdk_dir)
            else:
                _add_dll_directory(os.path.join(pkg_dir, "lib"))
                _add_dll_directory(os.path.join(pkg_dir, "plugins"))

            # Neither the SDK nor this wheel ships ovstage, and ovphysx.dll imports
            # it. os.add_dll_directory() does not search subdirectories, so the
            # USD/TBB closure under plugins/ needs its own entry.
            _add_dll_directory(ovstage_bin_dir)
            _add_dll_directory(os.path.join(ovstage_bin_dir, "plugins"))
            _prefer_ovstage_runtime_dir(ovstage_bin_dir)
        except Exception as e:
            _logger.debug("Failed to add DLL directories to search path: %s", e)

    if uses_bundled_lib:
        # An override that resolves to this wheel's own library still needs the
        # exact sibling ovstage wheel dependency closure.
        _preload_ovstage_runtime_deps()

    # Check OVPHYSX_LIB first (development mode takes precedence over bundled deps)
    env_override = lib_env
    if env_override:
        _logger.info("Loading library from OVPHYSX_LIB=%s (overriding bundled/default discovery)", env_override)
        try:
            return ctypes.CDLL(env_override)
        except OSError as e:
            raise OSError(f"Failed to load OVPHYSX_LIB='{env_override}': {e}") from e

    # Fall back to the bundled library when OVPHYSX_LIB is not set.
    if bundled:
        try:
            # Keep ctypes' secure Windows defaults. They include the absolute
            # DLL's directory and the os.add_dll_directory() paths above;
            # winmode=0 would bypass those registered dependency directories.
            return ctypes.CDLL(bundled)
        except OSError as e:
            _logger.warning("Failed to load bundled library %s: %s; will try fallback paths", bundled, e)

    errors: list[str] = []

    # Default linker paths
    for name in _platform_lib_names():
        try:
            return ctypes.CDLL(name)
        except OSError as e:
            errors.append(f"{name}: {e}")
            continue

    detail = ""
    if errors:
        detail = "\nLoad attempts:\n  - " + "\n  - ".join(errors)
    raise OSError(
        "Could not load libovphysx (bundled or system). Consider installing a wheel that includes the "
        "native library for your platform, or set OVPHYSX_LIB to the shared library path." + detail
    )


_lib = _load_library()
_logger.debug("Loaded library: %s", _lib)


class ovphysx_string_t(ctypes.Structure):
    """String structure with pointer and length (matches ovphysx_string_t in C API)."""

    _fields_ = [
        ("ptr", c_char_p),
        ("length", c_size_t),
    ]

    def __init__(self, value=None):
        super().__init__()
        encoded = str(value or "").encode("utf-8")
        self._bytes = ctypes.create_string_buffer(encoded)
        self.ptr = ctypes.cast(self._bytes, c_char_p)
        self.length = len(encoded)

    def __str__(self):
        # Use explicit length; C side does not guarantee null termination.
        if not self.ptr or not self.length:
            return ""
        return ctypes.string_at(self.ptr, self.length).decode("utf-8")

    def __bool__(self):
        return self.ptr is not None and self.length > 0

    def __len__(self) -> int:
        """Return byte length (excluding null terminator)."""
        return int(self.length)


# Config key type discriminator (matches ovphysx_config_key_type_t)
OVPHYSX_CONFIG_KEY_TYPE_BOOL = 0
OVPHYSX_CONFIG_KEY_TYPE_INT32 = 1
OVPHYSX_CONFIG_KEY_TYPE_FLOAT = 2
OVPHYSX_CONFIG_KEY_TYPE_STRING = 3
OVPHYSX_CONFIG_KEY_TYPE_CARBONITE = 4


# Key union for ovphysx_config_entry_t
class _config_key_union(ctypes.Union):
    _fields_ = [
        ("bool_key", c_int),         # ovphysx_config_bool_t enum
        ("int32_key", c_int),        # ovphysx_config_int32_t enum
        ("float_key", c_int),        # ovphysx_config_float_t enum
        ("string_key", c_int),       # ovphysx_config_string_t enum
        ("carbonite_key", ovphysx_string_t),  # for KEY_TYPE_CARBONITE
    ]


# Value union for ovphysx_config_entry_t
class _config_value_union(ctypes.Union):
    _fields_ = [
        ("bool_value", ctypes.c_bool),
        ("int32_value", c_int32),
        ("float_value", c_float),
        ("string_value", ovphysx_string_t),  # for KEY_TYPE_STRING and KEY_TYPE_CARBONITE
    ]


class ovphysx_config_entry_t(ctypes.Structure):
    """Typed config entry (matches ovphysx_config_entry_t in C API)."""

    _fields_ = [
        ("key_type", c_int),         # ovphysx_config_key_type_t enum
        ("key", _config_key_union),
        ("value", _config_value_union),
    ]


# ovphysx_create_args - matches ovphysx_create_args in C API
class ovphysx_create_args(ctypes.Structure):
    """Configuration for creating an ovphysx instance."""

    _fields_ = [
        ("bundled_deps_path", ovphysx_string_t),
        ("config_entries", POINTER(ovphysx_config_entry_t)),
        ("config_entry_count", c_uint32),
        ("active_cuda_gpus", ovphysx_string_t),
    ]


# ovphysx result structures
class ovphysx_result_t(ctypes.Structure):
    """Result returned by synchronous API functions."""

    _fields_ = [
        ("status", c_int),
    ]


class ovphysx_enqueue_result_t(ctypes.Structure):
    """Result returned by asynchronous API functions."""

    _fields_ = [
        ("status", c_int),
        ("op_index", c_uint64),
    ]


class ovphysx_op_wait_result_t(ctypes.Structure):
    """Result from ovphysx_wait_op() containing failed op indices and pending operation status."""

    _fields_ = [
        ("error_op_indices", POINTER(c_uint64)),
        ("num_errors", c_size_t),
        ("lowest_pending_op_index", c_uint64),
    ]


# CUDA sync structure
class ovphysx_cuda_sync_t(ctypes.Structure):
    """CUDA synchronization for GPU operations."""

    _fields_ = [
        ("stream", c_uint64),
        ("wait_event", c_uint64),
        ("signal_event", c_uint64),
    ]


# Tensor binding descriptor
class ovphysx_tensor_binding_desc_t(ctypes.Structure):
    """Descriptor for creating a tensor binding.

    A tensor binding connects physics-object paths to a tensor type, enabling
    bulk read/write for authored USD objects and runtime-only clones.

    Prim selection (mutually exclusive - use ONE of these):
      - pattern: Glob pattern like "/World/robot*"
      - prim_paths: Explicit list of exact physics-object paths

    If prim_paths is set, pattern is ignored.
    """

    _fields_ = [
        ("pattern", ovphysx_string_t),  # Physics-object path glob
        ("prim_paths", POINTER(ovphysx_string_t)),  # Exact object paths (NULL = use pattern)
        ("prim_paths_count", c_uint32),  # Number of object paths (0 = use pattern)
        ("tensor_type", c_int),  # ovphysx_tensor_type_t enum
    ]


# Tensor specification returned by ovphysx_get_tensor_binding_spec
class ovphysx_tensor_spec_t(ctypes.Structure):
    """Complete tensor specification for DLTensor construction.

    Use ovphysx_get_tensor_binding_spec() to get the exact dtype, rank, and shape
    needed to allocate a compatible tensor.

    Tensor specifications by type:
      - Rigid body pose:     ndim=2, shape=[N, 7]
      - Rigid body velocity: ndim=2, shape=[N, 6]
      - Articulation root:   ndim=2, shape=[N, 7] or [N, 6]
      - Articulation links:  ndim=3, shape=[N, L, 7] or [N, L, 6]
      - Articulation DOF:    ndim=2, shape=[N, D]
      - Deformable body:     ndim=3, shape=[N, V, C] or [N, E, K]
      - Deformable material: ndim=1, shape=[M]
    """

    _fields_ = [
        ("dtype", DLDataType),  # DLPack data type for this tensor type
        ("ndim", c_int32),  # Number of dimensions (2 or 3)
        ("shape", c_int64 * 4),  # Shape dimensions [dim0, dim1, dim2, 0]
    ]


class ovphysx_articulation_metadata_t(ctypes.Structure):
    """Articulation topology metadata (matches ovphysx_articulation_metadata_t in C API)."""

    _fields_ = [
        ("dof_count", c_int32),
        ("body_count", c_int32),
        ("joint_count", c_int32),
        ("fixed_tendon_count", c_int32),
        ("spatial_tendon_count", c_int32),
        ("is_fixed_base", ctypes.c_bool),
    ]


# --- ovstage / ovx native types (the read surface uses these directly) ---------
# These ctypes structs mirror the C ABI layout of ovstage's own types from
# <ovstage/ovstage_api/ovstage_api_types.h> and <ovx/string_types.h>; field
# order/types/padding must match those headers exactly.


class ovx_string_t(ctypes.Structure):
    """ovx_string_t: pointer + length (not necessarily null-terminated)."""

    _fields_ = [
        ("ptr", c_char_p),
        ("length", c_size_t),
    ]


class ovx_string_or_token_t(ctypes.Structure):
    """ovx_string_or_token_t: an interned ``token`` OR a ``string`` name.

    Pass ``token != 0`` for an interned attribute token, else a ``string`` name.
    """

    _fields_ = [
        ("token", c_uint64),   # ovx_token_t
        ("string", ovx_string_t),
    ]


class ovstage_ordinal_range_t(ctypes.Structure):
    """ovstage_ordinal_range_t — range for update_from_ovstage.

    ``has_start_ordinal`` True ⇒ closed range [start_ordinal, end_ordinal];
    False ⇒ the single end_ordinal.
    """

    _fields_ = [
        ("start_ordinal", c_uint64),       # ovstage_ordinal_t
        ("end_ordinal", c_uint64),
        ("has_start_ordinal", ctypes.c_bool),
    ]


class ovstage_prim_group_t(ctypes.Structure):
    """ovstage_prim_group_t — which prims a group covers."""

    _fields_ = [
        ("list", c_uint64),                # ovx_primpath_list_t
        ("offset", c_uint32),
        ("count", c_uint32),
        ("index_map", POINTER(c_uint32)),
    ]


class ovstage_cuda_sync_t(ctypes.Structure):
    """ovstage_cuda_sync_t - producer stream and wait event."""

    _fields_ = [
        ("stream", c_size_t),
        ("wait_event", c_size_t),
    ]


class ovstage_data_t(ctypes.Structure):
    """ovstage_data_t — borrowed tensor(s) plus sparsity / GPU-sync metadata."""

    _fields_ = [
        ("tensors", POINTER(DLTensor)),
        ("tensor_count", c_uint32),
        ("count", c_uint32),
        ("index_map", POINTER(c_uint32)),
        ("mask", c_void_p),                # ovstage_mask_t (const uint64_t*)
        ("cuda_sync", ovstage_cuda_sync_t),
    ]


class ovstage_attribute_meta_t(ctypes.Structure):
    """ovstage_attribute_meta_t — write floor + layout generation."""

    _fields_ = [
        ("attribute_write_floor_ordinal", c_uint64),
        ("layout_generation", c_uint64),
    ]


class ovstage_read_group_t(ctypes.Structure):
    """ovstage_read_group_t — one physics-output column group (ovstage's own type).

    The ovphysx output read returns these verbatim (producer-owned borrowed pointer
    from :func:`ovphysx_fetch_read_next`). ``attribute`` and ``prims.list`` are
    interned handles (resolve via :meth:`PhysX.query_shared_dictionary`); ``data.tensors``
    is a borrowed array of ``data.tensor_count`` DLTensors with tuple width in each
    tensor's ``dtype.lanes``. Borrowed fields stay valid until the group's
    ``read_group_id`` is released via :meth:`PhysX.release_group`. Field
    order/types/padding must match ovstage_api_types.h exactly.
    """

    _fields_ = [
        ("read_group_id", c_uint64),       # ovstage_read_group_id_t
        ("attribute", c_uint64),           # ovx_token_t
        ("ordinal", c_uint64),             # ovstage_ordinal_t
        ("is_delete", ctypes.c_bool),
        ("is_array", ctypes.c_bool),
        ("semantic", c_int32),             # ovstage_attribute_semantic_t
        ("prims", ovstage_prim_group_t),
        ("data", ovstage_data_t),
        ("meta", ovstage_attribute_meta_t),
    ]


class ovstage_query_result_t(ctypes.Structure):
    """ovstage_query_result_t — query discovery summary (ovstage's own type).

    ``attributes`` is a borrowed array of ``attribute_count`` interned attribute
    tokens, valid until the query is released. ``total_prim_count == 0`` is the
    empty-match case.
    """

    _fields_ = [
        ("query_result_id", c_uint64),     # ovstage_query_result_id_t
        ("attributes", POINTER(c_uint64)), # const ovx_token_t*
        ("attribute_count", c_size_t),
        ("all_handle", c_uint64),          # ovstage_query_handle_t
        ("total_prim_count", c_size_t),
    ]


# Core API function prototypes
_lib.ovphysx_initialize.restype = ovphysx_result_t
_lib.ovphysx_initialize.argtypes = []

_lib.ovphysx_shutdown.restype = ovphysx_result_t
_lib.ovphysx_shutdown.argtypes = []

_lib.ovphysx_create_instance.restype = ovphysx_result_t
_lib.ovphysx_create_instance.argtypes = [POINTER(ovphysx_create_args), POINTER(c_uint64)]

_lib.ovphysx_set_cpu_mode.restype = ovphysx_result_t
_lib.ovphysx_set_cpu_mode.argtypes = [ctypes.c_bool]

_lib.ovphysx_destroy_instance.restype = ovphysx_result_t
_lib.ovphysx_destroy_instance.argtypes = [c_uint64]

_lib.ovphysx_reset_stage.restype = ovphysx_enqueue_result_t
_lib.ovphysx_reset_stage.argtypes = [c_uint64]

_lib.ovphysx_step.restype = ovphysx_enqueue_result_t
_lib.ovphysx_step.argtypes = [c_uint64, c_float]

_lib.ovphysx_step_sync.restype = ovphysx_result_t
_lib.ovphysx_step_sync.argtypes = [c_uint64, c_float]

_lib.ovphysx_step_n_sync.restype = ovphysx_result_t
_lib.ovphysx_step_n_sync.argtypes = [c_uint64, c_int32, c_float]

_lib.ovphysx_update_articulations_kinematic.restype = ovphysx_result_t
_lib.ovphysx_update_articulations_kinematic.argtypes = [c_uint64]

_lib.ovphysx_attach_ovstage.restype = ovphysx_result_t
# stage is an ovstage_instance_t* (opaque pointer); pass as c_void_p.
_lib.ovphysx_attach_ovstage.argtypes = [c_uint64, ctypes.c_void_p, c_uint64]

_lib.ovphysx_update_from_ovstage.restype = ovphysx_result_t
_lib.ovphysx_update_from_ovstage.argtypes = [c_uint64, ovstage_ordinal_range_t]

# Physics output read (ADR-0007) — ovstage-native types.
_lib.ovphysx_query.restype = ovphysx_result_t
_lib.ovphysx_query.argtypes = [c_uint64, c_int32, c_int32, POINTER(c_uint64)]

_lib.ovphysx_fetch_query_result.restype = ovphysx_result_t
_lib.ovphysx_fetch_query_result.argtypes = [c_uint64, c_uint64, POINTER(ovstage_query_result_t)]

_lib.ovphysx_query_shared_dictionary.restype = ovphysx_result_t
_lib.ovphysx_query_shared_dictionary.argtypes = [c_uint64, c_uint64, POINTER(c_void_p)]

# attributes are ovx_string_or_token_t (string name or interned token per entry).
_lib.ovphysx_read.restype = ovphysx_result_t
_lib.ovphysx_read.argtypes = [c_uint64, c_uint64, POINTER(ovx_string_or_token_t), c_size_t, POINTER(c_uint64)]

# Producer-owned group: fetch returns a borrowed ovstage_read_group_t* (POINTER(POINTER(...))).
_lib.ovphysx_fetch_read_next.restype = ovphysx_result_t
_lib.ovphysx_fetch_read_next.argtypes = [c_uint64, c_uint64, POINTER(POINTER(ovstage_read_group_t))]

_lib.ovphysx_release_group.restype = ovphysx_result_t
_lib.ovphysx_release_group.argtypes = [c_uint64, c_uint64, c_uint64]

_lib.ovphysx_release_read.restype = ovphysx_result_t
_lib.ovphysx_release_read.argtypes = [c_uint64, c_uint64]

_lib.ovphysx_release_query.restype = ovphysx_result_t
_lib.ovphysx_release_query.argtypes = [c_uint64, c_uint64]

_lib.ovphysx_detach_ovstage.restype = ovphysx_result_t
_lib.ovphysx_detach_ovstage.argtypes = [c_uint64]

_lib.ovphysx_clone.restype = ovphysx_enqueue_result_t
_lib.ovphysx_clone.argtypes = [
    c_uint64, ovphysx_string_t, POINTER(ovphysx_string_t), c_uint32, POINTER(c_float), POINTER(c_uint32)
]

# Typed config API
_lib.ovphysx_set_global_config.restype = ovphysx_result_t
_lib.ovphysx_set_global_config.argtypes = [ovphysx_config_entry_t]

_lib.ovphysx_get_global_config_bool.restype = ovphysx_result_t
_lib.ovphysx_get_global_config_bool.argtypes = [c_int, POINTER(ctypes.c_bool)]

_lib.ovphysx_get_global_config_int32.restype = ovphysx_result_t
_lib.ovphysx_get_global_config_int32.argtypes = [c_int, POINTER(c_int32)]

_lib.ovphysx_get_global_config_float.restype = ovphysx_result_t
_lib.ovphysx_get_global_config_float.argtypes = [c_int, POINTER(c_float)]

_lib.ovphysx_get_global_config_string.restype = ovphysx_result_t
_lib.ovphysx_get_global_config_string.argtypes = [c_int, POINTER(ovphysx_string_t), POINTER(c_size_t)]

# Async operations
_lib.ovphysx_wait_op.restype = ovphysx_result_t
_lib.ovphysx_wait_op.argtypes = [c_uint64, c_uint64, c_uint64, POINTER(ovphysx_op_wait_result_t)]

_lib.ovphysx_get_last_error.restype = ovphysx_string_t
_lib.ovphysx_get_last_error.argtypes = []

_lib.ovphysx_get_last_op_error.restype = ovphysx_string_t
_lib.ovphysx_get_last_op_error.argtypes = [c_uint64]

_lib.ovphysx_destroy_wait_result.restype = None
_lib.ovphysx_destroy_wait_result.argtypes = [POINTER(ovphysx_op_wait_result_t)]

# Version query
_lib.ovphysx_get_version_string.restype = c_char_p
_lib.ovphysx_get_version_string.argtypes = []

# Logging configuration API
# ovphysx_log_fn: void (*)(uint32_t level, const char* message, void* user_data)
ovphysx_log_fn = ctypes.CFUNCTYPE(None, c_uint32, c_char_p, c_void_p)

_lib.ovphysx_set_log_level.restype = ovphysx_result_t
_lib.ovphysx_set_log_level.argtypes = [c_uint32]

_lib.ovphysx_get_log_level.restype = c_uint32
_lib.ovphysx_get_log_level.argtypes = []

_lib.ovphysx_enable_default_log_output.restype = ovphysx_result_t
_lib.ovphysx_enable_default_log_output.argtypes = [ctypes.c_bool]

_lib.ovphysx_register_log_callback.restype = ovphysx_result_t
_lib.ovphysx_register_log_callback.argtypes = [ovphysx_log_fn, c_void_p]

_lib.ovphysx_unregister_log_callback.restype = ovphysx_result_t
_lib.ovphysx_unregister_log_callback.argtypes = [ovphysx_log_fn, c_void_p]

# Log diagnostics (for testing)
_lib.ovphysx_log_emit_test_messages.restype = None
_lib.ovphysx_log_emit_test_messages.argtypes = []

# Tensor Binding API - bulk data access for physics simulation
_lib.ovphysx_create_tensor_binding.restype = ovphysx_result_t
_lib.ovphysx_create_tensor_binding.argtypes = [
    c_uint64,  # handle
    POINTER(ovphysx_tensor_binding_desc_t),  # desc
    POINTER(c_uint64),  # out_binding_handle
]

_lib.ovphysx_destroy_tensor_binding.restype = ovphysx_result_t
_lib.ovphysx_destroy_tensor_binding.argtypes = [c_uint64, c_uint64]  # handle, binding_handle

_lib.ovphysx_get_tensor_binding_spec.restype = ovphysx_result_t
_lib.ovphysx_get_tensor_binding_spec.argtypes = [
    c_uint64,  # handle
    c_uint64,  # binding_handle
    POINTER(ovphysx_tensor_spec_t),  # out_spec
]

_lib.ovphysx_read_tensor_binding.restype = ovphysx_result_t
_lib.ovphysx_read_tensor_binding.argtypes = [
    c_uint64,  # handle
    c_uint64,  # binding_handle
    POINTER(DLTensor),  # dst_tensor
]

_lib.ovphysx_write_tensor_binding.restype = ovphysx_result_t
_lib.ovphysx_write_tensor_binding.argtypes = [
    c_uint64,  # handle
    c_uint64,  # binding_handle
    POINTER(DLTensor),  # src_tensor
    POINTER(DLTensor),  # index_tensor (optional, can be NULL)
]

_lib.ovphysx_write_tensor_binding_masked.restype = ovphysx_result_t
_lib.ovphysx_write_tensor_binding_masked.argtypes = [
    c_uint64,  # handle
    c_uint64,  # binding_handle
    POINTER(DLTensor),  # src_tensor
    POINTER(DLTensor),  # mask_tensor
]

_lib.ovphysx_warmup_gpu.restype = ovphysx_result_t
_lib.ovphysx_warmup_gpu.argtypes = [c_uint64]  # handle

_lib.ovphysx_rigid_body_view_wake_up.restype = ovphysx_result_t
_lib.ovphysx_rigid_body_view_wake_up.argtypes = [
    c_uint64,  # handle
    c_uint64,  # binding_handle
    POINTER(DLTensor),  # indices (optional, can be NULL)
]

_lib.ovphysx_rigid_body_view_sleep.restype = ovphysx_result_t
_lib.ovphysx_rigid_body_view_sleep.argtypes = [
    c_uint64,  # handle
    c_uint64,  # binding_handle
    POINTER(DLTensor),  # indices (optional, can be NULL)
]

_lib.ovphysx_articulation_update_kinematic.restype = ovphysx_result_t
_lib.ovphysx_articulation_update_kinematic.argtypes = [
    c_uint64,  # handle
    c_uint64,  # binding_handle
    c_uint32,  # flags (ovphysx_articulation_kinematic_flag_t bitmask)
]

_lib.ovphysx_shutdown.restype = ovphysx_result_t
_lib.ovphysx_shutdown.argtypes = []

# Articulation metadata (single consolidated call)
_lib.ovphysx_get_articulation_metadata.restype = ovphysx_result_t
_lib.ovphysx_get_articulation_metadata.argtypes = [c_uint64, c_uint64, POINTER(ovphysx_articulation_metadata_t)]

_lib.ovphysx_articulation_get_dof_names.restype = ovphysx_result_t
_lib.ovphysx_articulation_get_dof_names.argtypes = [
    c_uint64,
    c_uint64,
    POINTER(ovphysx_string_t),
    c_uint32,
    POINTER(c_uint32),
]

_lib.ovphysx_articulation_get_body_names.restype = ovphysx_result_t
_lib.ovphysx_articulation_get_body_names.argtypes = [
    c_uint64,
    c_uint64,
    POINTER(ovphysx_string_t),
    c_uint32,
    POINTER(c_uint32),
]

_lib.ovphysx_articulation_get_joint_names.restype = ovphysx_result_t
_lib.ovphysx_articulation_get_joint_names.argtypes = [
    c_uint64,
    c_uint64,
    POINTER(ovphysx_string_t),
    c_uint32,
    POINTER(c_uint32),
]

_lib.ovphysx_tensor_binding_get_prim_paths.restype = ovphysx_result_t
_lib.ovphysx_tensor_binding_get_prim_paths.argtypes = [
    c_uint64,
    c_uint64,
    POINTER(ovphysx_string_t),
    c_uint32,
    POINTER(c_uint32),
]

# Contact binding API
_lib.ovphysx_create_contact_binding.restype = ovphysx_result_t
_lib.ovphysx_create_contact_binding.argtypes = [
    c_uint64,
    POINTER(ovphysx_string_t),
    c_uint32,
    POINTER(ovphysx_string_t),
    c_uint32,
    c_uint32,
    POINTER(c_uint64),
]

_lib.ovphysx_destroy_contact_binding.restype = ovphysx_result_t
_lib.ovphysx_destroy_contact_binding.argtypes = [c_uint64, c_uint64]

_lib.ovphysx_get_contact_binding_spec.restype = ovphysx_result_t
_lib.ovphysx_get_contact_binding_spec.argtypes = [c_uint64, c_uint64, POINTER(c_int32), POINTER(c_int32)]

_lib.ovphysx_contact_binding_get_sensor_paths.restype = ovphysx_result_t
_lib.ovphysx_contact_binding_get_sensor_paths.argtypes = [
    c_uint64,
    c_uint64,
    POINTER(ovphysx_string_t),
    c_uint32,
    POINTER(c_uint32),
]

_lib.ovphysx_get_object_type.restype = ovphysx_result_t
_lib.ovphysx_get_object_type.argtypes = [
    c_uint64,
    ovphysx_string_t,
    POINTER(c_uint32),
]

_lib.ovphysx_contact_binding_get_filter_paths.restype = ovphysx_result_t
_lib.ovphysx_contact_binding_get_filter_paths.argtypes = [
    c_uint64,
    c_uint64,
    POINTER(ovphysx_string_t),
    c_uint32,
    POINTER(c_uint32),
]

_lib.ovphysx_get_contact_binding_capacity.restype = ovphysx_result_t
_lib.ovphysx_get_contact_binding_capacity.argtypes = [c_uint64, c_uint64, POINTER(c_uint32)]

_lib.ovphysx_read_contact_net_forces.restype = ovphysx_result_t
_lib.ovphysx_read_contact_net_forces.argtypes = [c_uint64, c_uint64, POINTER(DLTensor)]

_lib.ovphysx_read_contact_force_matrix.restype = ovphysx_result_t
_lib.ovphysx_read_contact_force_matrix.argtypes = [c_uint64, c_uint64, POINTER(DLTensor)]

_lib.ovphysx_read_contact_data.restype = ovphysx_result_t
_lib.ovphysx_read_contact_data.argtypes = [
    c_uint64,
    c_uint64,
    POINTER(DLTensor),
    POINTER(DLTensor),
    POINTER(DLTensor),
    POINTER(DLTensor),
    POINTER(DLTensor),
    POINTER(DLTensor),
]

_lib.ovphysx_read_friction_data.restype = ovphysx_result_t
_lib.ovphysx_read_friction_data.argtypes = [
    c_uint64,
    c_uint64,
    POINTER(DLTensor),
    POINTER(DLTensor),
    POINTER(DLTensor),
    POINTER(DLTensor),
]

_lib.ovphysx_read_raw_contact_data.restype = ovphysx_result_t
_lib.ovphysx_read_raw_contact_data.argtypes = [
    c_uint64,  # handle
    c_uint64,  # contact_handle
    POINTER(DLTensor),  # contact_force_tensor       [C, 1]
    POINTER(DLTensor),  # contact_point_tensor       [C, 3]
    POINTER(DLTensor),  # contact_normal_tensor      [C, 3]
    POINTER(DLTensor),  # contact_separation_tensor  [C, 1]
    POINTER(DLTensor),  # contact_count_tensor       [S]
    POINTER(DLTensor),  # contact_start_indices      [S]
    POINTER(DLTensor),  # other_actor_ids_tensor     [C] uint64
]

_lib.ovphysx_contact_binding_get_other_actor_paths_from_ids.restype = ovphysx_result_t
_lib.ovphysx_contact_binding_get_other_actor_paths_from_ids.argtypes = [
    c_uint64,  # handle
    c_uint64,  # contact_handle
    POINTER(DLTensor),  # ids_tensor [N] uint64
    POINTER(ovphysx_string_t),  # out_paths
    c_uint32,  # max_paths
    POINTER(c_uint32),  # out_count
]


# Contact report struct mirrors live in contact_types so they can be imported
# without triggering native-library load (and rendered via Sphinx automodule).
from .contact_types import ContactEventHeader, ContactPoint, FrictionAnchor


_lib.ovphysx_get_contact_report.restype = ovphysx_result_t
_lib.ovphysx_get_contact_report.argtypes = [
    c_uint64,
    POINTER(POINTER(ContactEventHeader)),
    POINTER(c_uint32),
    POINTER(POINTER(ContactPoint)),
    POINTER(c_uint32),
    POINTER(POINTER(FrictionAnchor)),  # out_friction_anchors (optional, can be None)
    POINTER(c_uint32),                 # out_num_friction_anchors (optional, can be None)
]


# Scene query types (match ovphysx_types.h)

class _SphereGeom(ctypes.Structure):
    _fields_ = [("radius", ctypes.c_float), ("position", ctypes.c_float * 3)]


class _BoxGeom(ctypes.Structure):
    _fields_ = [
        ("half_extent", ctypes.c_float * 3),
        ("position", ctypes.c_float * 3),
        ("rotation", ctypes.c_float * 4),
    ]


class _ShapeGeom(ctypes.Structure):
    _fields_ = [("prim_path", ovphysx_string_t)]


class _GeomUnion(ctypes.Union):
    _fields_ = [("sphere", _SphereGeom), ("box", _BoxGeom), ("shape", _ShapeGeom)]


class ovphysx_scene_query_geometry_desc_t(ctypes.Structure):
    """Geometry descriptor for sweep/overlap queries (matches C API)."""
    _fields_ = [("type", c_int), ("_geom", _GeomUnion)]


class ovphysx_scene_query_hit_t(ctypes.Structure):
    """Scene query hit result (matches C API)."""
    _fields_ = [
        ("collision", c_uint64),
        ("rigid_body", c_uint64),
        ("proto_index", c_uint32),
        ("normal", ctypes.c_float * 3),
        ("position", ctypes.c_float * 3),
        ("distance", ctypes.c_float),
        ("face_index", c_uint32),
        ("material", c_uint64),
    ]


# Scene query function prototypes
_lib.ovphysx_raycast.restype = ovphysx_result_t
_lib.ovphysx_raycast.argtypes = [
    c_uint64,                                              # handle
    ctypes.c_float * 3,                                    # origin
    ctypes.c_float * 3,                                    # direction
    ctypes.c_float,                                        # distance
    ctypes.c_bool,                                         # both_sides
    c_int,                                                 # mode
    POINTER(POINTER(ovphysx_scene_query_hit_t)),           # out_hits
    POINTER(c_uint32),                                     # out_count
]

_lib.ovphysx_sweep.restype = ovphysx_result_t
_lib.ovphysx_sweep.argtypes = [
    c_uint64,                                              # handle
    POINTER(ovphysx_scene_query_geometry_desc_t),          # geometry
    ctypes.c_float * 3,                                    # direction
    ctypes.c_float,                                        # distance
    ctypes.c_bool,                                         # both_sides
    c_int,                                                 # mode
    POINTER(POINTER(ovphysx_scene_query_hit_t)),           # out_hits
    POINTER(c_uint32),                                     # out_count
]

_lib.ovphysx_overlap.restype = ovphysx_result_t
_lib.ovphysx_overlap.argtypes = [
    c_uint64,                                              # handle
    POINTER(ovphysx_scene_query_geometry_desc_t),          # geometry
    c_int,                                                 # mode
    POINTER(POINTER(ovphysx_scene_query_hit_t)),           # out_hits
    POINTER(c_uint32),                                     # out_count
]


# ---------------------------------------------------------------------------
# SDF view
# ---------------------------------------------------------------------------

_lib.ovphysx_create_sdf_view.restype = ovphysx_result_t
_lib.ovphysx_create_sdf_view.argtypes = [
    c_uint64,           # handle
    ovphysx_string_t,   # pattern
    c_uint32,           # max_query_points
    POINTER(c_uint64),  # out_handle
]

_lib.ovphysx_sdf_view_get_count.restype = ovphysx_result_t
_lib.ovphysx_sdf_view_get_count.argtypes = [c_uint64, c_uint64, POINTER(c_uint32)]

_lib.ovphysx_sdf_view_get_max_query_points.restype = ovphysx_result_t
_lib.ovphysx_sdf_view_get_max_query_points.argtypes = [c_uint64, c_uint64, POINTER(c_uint32)]

_lib.ovphysx_evaluate_sdf.restype = ovphysx_result_t
_lib.ovphysx_evaluate_sdf.argtypes = [
    c_uint64,                  # handle
    c_uint64,                  # sdf_handle
    ctypes.c_void_p,           # query_points (DLTensor*)
    ctypes.c_void_p,           # out_distances_and_gradients (DLTensor*)
]

_lib.ovphysx_destroy_sdf_view.restype = ovphysx_result_t
_lib.ovphysx_destroy_sdf_view.argtypes = [c_uint64, c_uint64]


def get_native_version_string() -> str:
    """Return the native library version string, or empty string if unavailable."""
    try:
        value = _lib.ovphysx_get_version_string()
        if not value:
            return ""
        if isinstance(value, bytes):
            return value.decode("utf-8", errors="replace")
        return str(value)
    except Exception:
        return ""
