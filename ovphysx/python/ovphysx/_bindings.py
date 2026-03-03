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

# ovphysx API status enum
OVPHYSX_API_SUCCESS = 0
OVPHYSX_API_ERROR = 1
OVPHYSX_API_TIMEOUT = 2
OVPHYSX_API_NOT_IMPLEMENTED = 3
OVPHYSX_API_INVALID_ARGUMENT = 4
OVPHYSX_API_NOT_FOUND = 5

# Invalid handle sentinel (matches OVPHYSX_INVALID_HANDLE in ovphysx_types.h)
OVPHYSX_INVALID_HANDLE = 0

# Binding prim mode enum
OVPHYSX_BINDING_PRIM_MODE_EXISTING_ONLY = 0
OVPHYSX_BINDING_PRIM_MODE_MUST_EXIST = 1
OVPHYSX_BINDING_PRIM_MODE_CREATE_NEW = 2

# DLPack device types (convenience constants)
kDLCPU = DLDeviceType.kDLCPU
kDLCUDA = DLDeviceType.kDLCUDA

# DLPack data type codes (convenience constants)
kDLInt = DLDataTypeCode.kDLInt
kDLUInt = DLDataTypeCode.kDLUInt
kDLFloat = DLDataTypeCode.kDLFloat

# Operation index sentinel for wait_op (wait for all operations)
OVPHYSX_OP_INDEX_ALL = 0xFFFFFFFFFFFFFFFF

# Tensor type enum (ovphysx_tensor_type_t from ovphysx_types.h)
OVPHYSX_TENSOR_INVALID = 0
# Rigid body tensors
OVPHYSX_TENSOR_RIGID_BODY_POSE_F32 = 1  # [N, 7] poses in world frame
OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32 = 2  # [N, 6] velocities in world frame
# Articulation root tensors
OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32 = 10  # [N, 7] root poses
OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32 = 11  # [N, 6] root velocities
# Articulation link tensors (3D)
OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32 = 20  # [N, L, 7] link poses
OVPHYSX_TENSOR_ARTICULATION_LINK_VELOCITY_F32 = 21  # [N, L, 6] link velocities
# Articulation DOF tensors
OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32 = 30  # [N, D] joint positions
OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32 = 31  # [N, D] joint velocities
OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32 = 32  # [N, D] position targets
OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32 = 33  # [N, D] velocity targets
OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32 = 34  # [N, D] actuation forces
# External forces/wrenches - WRITE-ONLY (control inputs applied each step)
OVPHYSX_TENSOR_RIGID_BODY_FORCE_F32 = 50  # [N, 3] forces at center of mass
OVPHYSX_TENSOR_RIGID_BODY_WRENCH_F32 = 51  # [N, 9] row-major: [fx,fy,fz,tx,ty,tz,px,py,pz]
OVPHYSX_TENSOR_ARTICULATION_LINK_WRENCH_F32 = 52  # [N, L, 9] row-major: same layout per link


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
    # On Windows, ensure the package directory and dependency directories are on the DLL search path
    # On Linux, the rpath is set to $ORIGIN so dependent libraries are discoverable
    if sys.platform == "win32":
        pkg_dir = os.path.dirname(__file__)
        try:
            os.add_dll_directory(pkg_dir)  # Python 3.8+
            _logger.info("Added DLL directory: %s", pkg_dir)

            # Also add kit_sdk_release for carb.dll and other Carbonite dependencies
            # Check OVPHYSX_ROOT environment variable first (set by test scripts)
            if "OVPHYSX_ROOT" in os.environ:
                sdk_root = os.environ["OVPHYSX_ROOT"]
                kit_sdk_dir = os.path.join(sdk_root, "target-deps", "kit_sdk_release")
                if os.path.exists(kit_sdk_dir):
                    os.add_dll_directory(kit_sdk_dir)
                    _logger.info("Added DLL directory: %s", kit_sdk_dir)
                else:
                    _logger.warning("Kit SDK directory not found: %s", kit_sdk_dir)
            else:
                _logger.warning("OVPHYSX_ROOT not set in environment")
        except Exception as e:
            _logger.warning("Failed to add DLL directories to search path: %s", e)

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
_logger.info("Loaded library: %s", _lib)


# Log level constants - mirrors ovphysx_log_level_t in ovphysx_types.h
OVPHYSX_LOG_NONE = 0       # No logging
OVPHYSX_LOG_ERROR = 1      # Error messages only
OVPHYSX_LOG_WARNING = 2    # Warnings and errors (default)
OVPHYSX_LOG_INFO = 3       # Info, warnings, and errors
OVPHYSX_LOG_VERBOSE = 4    # All messages including verbose/debug


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


# ovphysx_create_args - matches ovphysx_create_args in C API
class ovphysx_create_args(ctypes.Structure):
    """Configuration for creating an ovphysx instance."""

    _fields_ = [
        ("bundled_deps_path", ovphysx_string_t),
        ("settings_keys", POINTER(ovphysx_string_t)),
        ("settings_values", POINTER(ovphysx_string_t)),
        ("settings_count", c_uint32),
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
        ("error", ovphysx_string_t),
    ]


class ovphysx_enqueue_result_t(ctypes.Structure):
    """Result returned by asynchronous API functions."""

    _fields_ = [
        ("status", c_int),
        ("error", ovphysx_string_t),
        ("op_index", c_uint64),
    ]


class ovphysx_op_error_t(ctypes.Structure):
    """Error associated with a specific operation."""

    _fields_ = [
        ("op_index", c_uint64),
        ("error", ovphysx_string_t),
    ]


class ovphysx_op_wait_result_t(ctypes.Structure):
    """Result from ovphysx_wait_op() containing errors and pending operation status."""

    _fields_ = [
        ("errors", POINTER(ovphysx_op_error_t)),
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

_lib.ovphysx_get_stage_id.restype = ovphysx_result_t
_lib.ovphysx_get_stage_id.argtypes = [c_uint64, POINTER(c_int64)]

_lib.ovphysx_set_setting.restype = ovphysx_result_t
_lib.ovphysx_set_setting.argtypes = [c_uint64, ovphysx_string_t, ovphysx_string_t]

_lib.ovphysx_get_setting.restype = ovphysx_result_t
_lib.ovphysx_get_setting.argtypes = [c_uint64, ovphysx_string_t, POINTER(ovphysx_string_t), POINTER(c_size_t)]

# Async operations
_lib.ovphysx_wait_op.restype = ovphysx_result_t
_lib.ovphysx_wait_op.argtypes = [c_uint64, c_uint64, c_uint64, POINTER(ovphysx_op_wait_result_t)]

_lib.ovphysx_clone.restype = ovphysx_enqueue_result_t
_lib.ovphysx_clone.argtypes = [c_uint64, ovphysx_string_t, POINTER(ovphysx_string_t), c_uint32]

_lib.ovphysx_destroy_error.restype = None
_lib.ovphysx_destroy_error.argtypes = [ovphysx_string_t]

_lib.ovphysx_destroy_errors.restype = None
_lib.ovphysx_destroy_errors.argtypes = [POINTER(ovphysx_op_error_t), c_size_t]

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
