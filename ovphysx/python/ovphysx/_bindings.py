# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Low-level ctypes bindings for the ovphysx library.

This module handles library loading and defines C structures and function prototypes.
"""

import ctypes
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

# Import DLPack structures from dlpack module
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


def _load_library() -> ctypes.CDLL:
    """Load libovphysx shared library.

    Order:
    - OVPHYSX_LIB (absolute path) - takes precedence for development mode
    - Packaged/bundled binary inside this wheel (platform-specific name)
    - Default linker paths: platform name
    - OVPHYSX_BIN_DIR/libovphysx.so if provided
    """
    # On Windows, add dependency directories to the DLL search path so that
    # ovphysx.dll's transitive dependencies (carb.dll in plugins/, etc.) are
    # discoverable.  This makes _bindings.py self-sufficient — it doesn't
    # rely on __init__.py's bootstrap having run first.
    # On Linux, rpath=$ORIGIN handles this at the ELF level.
    if sys.platform == "win32":
        pkg_dir = os.path.dirname(__file__)
        try:
            for subdir in ("lib", "plugins"):
                d = os.path.join(pkg_dir, subdir)
                if os.path.isdir(d):
                    os.add_dll_directory(d)
                    _logger.debug("Added DLL directory: %s", d)

            if "OVPHYSX_ROOT" in os.environ:
                sdk_root = os.environ["OVPHYSX_ROOT"]
                # Try config-specific kit SDK dirs (debug, release, checked)
                for _cfg in ("debug", "release", "checked"):
                    kit_sdk_dir = os.path.join(sdk_root, "target-deps", f"kit_sdk_{_cfg}")
                    if os.path.exists(kit_sdk_dir):
                        os.add_dll_directory(kit_sdk_dir)
                        _logger.debug("Added DLL directory: %s", kit_sdk_dir)
        except Exception as e:
            _logger.debug("Failed to add DLL directories to search path: %s", e)

    # Check OVPHYSX_LIB first (development mode takes precedence over bundled deps)
    env_override = os.environ.get("OVPHYSX_LIB")
    if env_override:
        try:
            return ctypes.CDLL(env_override)
        except OSError as e:
            raise OSError(f"Failed to load OVPHYSX_LIB='{env_override}': {e}") from e

    # Fall back to bundled library if OVPHYSX_LIB not set (wheel mode)
    bundled = _bundled_lib_path()
    if bundled:
        try:
            # On Windows, use winmode=0 to allow DLL dependencies to be loaded from the current directory
            # This is needed for the MSVC runtime and other dependencies
            if sys.platform == "win32" and sys.version_info >= (3, 8):
                return ctypes.CDLL(bundled, winmode=0)
            else:
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

    omni_bin = os.environ.get("OVPHYSX_BIN_DIR")
    if omni_bin:
        for name in _platform_lib_names():
            candidate = os.path.join(omni_bin, name)
            if os.path.exists(candidate):
                try:
                    return ctypes.CDLL(candidate)
                except OSError as e:
                    errors.append(f"{candidate}: {e}")

    detail = ""
    if errors:
        detail = "\nLoad attempts:\n  - " + "\n  - ".join(errors)
    raise OSError(
        "Could not load libovphysx (bundled or system). Consider installing a wheel that includes the "
        "native library for your platform, or set OVPHYSX_LIB to the shared library path." + detail
    )


_lib = _load_library()
_logger.debug("Loaded library: %s", _lib)


# ovphysx_string_t - matches C API definition
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
        # ovphysx_device_t (enum, 4 bytes)
        ("device", c_int),
        # int32_t
        ("gpu_index", c_int32),
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

    A tensor binding connects USD prims to a tensor type, enabling bulk
    read/write of physics data for all matching prims.

    Prim selection (mutually exclusive - use ONE of these):
      - pattern: Glob pattern like "/World/robot*"
      - prim_paths: Explicit list of exact prim paths

    If prim_paths is set, pattern is ignored.
    """

    _fields_ = [
        ("pattern", ovphysx_string_t),  # USD path glob pattern
        ("prim_paths", POINTER(ovphysx_string_t)),  # Explicit list of exact prim paths (NULL = use pattern)
        ("prim_paths_count", c_uint32),  # Number of prim paths (0 = use pattern)
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
    """

    _fields_ = [
        ("dtype", DLDataType),  # DLPack data type (always float32 currently)
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


# Core API function prototypes
_lib.ovphysx_create_instance.restype = ovphysx_result_t
_lib.ovphysx_create_instance.argtypes = [POINTER(ovphysx_create_args), POINTER(c_uint64)]

_lib.ovphysx_destroy_instance.restype = ovphysx_result_t
_lib.ovphysx_destroy_instance.argtypes = [c_uint64]

_lib.ovphysx_add_usd.restype = ovphysx_enqueue_result_t
_lib.ovphysx_add_usd.argtypes = [c_uint64, ovphysx_string_t, ovphysx_string_t, POINTER(c_uint64)]

_lib.ovphysx_remove_usd.restype = ovphysx_enqueue_result_t
_lib.ovphysx_remove_usd.argtypes = [c_uint64, c_uint64]

_lib.ovphysx_reset.restype = ovphysx_enqueue_result_t
_lib.ovphysx_reset.argtypes = [c_uint64]

_lib.ovphysx_step.restype = ovphysx_enqueue_result_t
_lib.ovphysx_step.argtypes = [c_uint64, c_float, c_float]

_lib.ovphysx_step_sync.restype = ovphysx_result_t
_lib.ovphysx_step_sync.argtypes = [c_uint64, c_float, c_float]

_lib.ovphysx_step_n_sync.restype = ovphysx_result_t
_lib.ovphysx_step_n_sync.argtypes = [c_uint64, c_int32, c_float, c_float]

_lib.ovphysx_get_stage_id.restype = ovphysx_result_t
_lib.ovphysx_get_stage_id.argtypes = [c_uint64, POINTER(c_int64)]

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

_lib.ovphysx_clone.restype = ovphysx_enqueue_result_t
_lib.ovphysx_clone.argtypes = [c_uint64, ovphysx_string_t, POINTER(ovphysx_string_t), c_uint32, POINTER(c_float)]

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

# Remote storage credential configuration
_lib.ovphysx_configure_s3.restype = ovphysx_result_t
_lib.ovphysx_configure_s3.argtypes = [c_char_p, c_char_p, c_char_p, c_char_p, c_char_p, c_char_p]

_lib.ovphysx_configure_azure_sas.restype = ovphysx_result_t
_lib.ovphysx_configure_azure_sas.argtypes = [c_char_p, c_char_p, c_char_p]

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

_lib.ovphysx_read_contact_net_forces.restype = ovphysx_result_t
_lib.ovphysx_read_contact_net_forces.argtypes = [c_uint64, c_uint64, POINTER(DLTensor)]

_lib.ovphysx_read_contact_force_matrix.restype = ovphysx_result_t
_lib.ovphysx_read_contact_force_matrix.argtypes = [c_uint64, c_uint64, POINTER(DLTensor)]


# PhysX object interop (unified)
_lib.ovphysx_get_physx_ptr.restype = ovphysx_result_t
_lib.ovphysx_get_physx_ptr.argtypes = [c_uint64, c_char_p, c_int, POINTER(c_void_p)]

# Contact report -- ctypes struct mirrors of the C ABI structs.


class ContactEventHeader(ctypes.Structure):
    _fields_ = [
        ("type", c_int32),
        ("stageId", c_int64),
        ("actor0", c_uint64),
        ("actor1", c_uint64),
        ("collider0", c_uint64),
        ("collider1", c_uint64),
        ("contactDataOffset", c_uint32),
        ("numContactData", c_uint32),
        ("frictionAnchorsDataOffset", c_uint32),
        ("numfrictionAnchorsData", c_uint32),
        ("protoIndex0", c_uint32),
        ("protoIndex1", c_uint32),
    ]


class ContactPoint(ctypes.Structure):
    _fields_ = [
        ("position", c_float * 3),
        ("normal", c_float * 3),
        ("impulse", c_float * 3),
        ("separation", c_float),
        ("faceIndex0", c_uint32),
        ("faceIndex1", c_uint32),
        ("material0", c_uint64),
        ("material1", c_uint64),
    ]


class FrictionAnchor(ctypes.Structure):
    _fields_ = [
        ("position", c_float * 3),
        ("impulse", c_float * 3),
    ]


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
    _fields_ = [("prim_path", c_char_p)]


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


# Tensor plugin loader (private API; needed by tensorAPI module only)
_lib.ovphysx_load_tensor_plugins.restype = c_int
_lib.ovphysx_load_tensor_plugins.argtypes = []
