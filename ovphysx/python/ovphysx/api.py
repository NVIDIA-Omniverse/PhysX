# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""High-level Python API for the ovphysx library.

Stream-Ordered Execution Model
------------------------------
All operations in this API are stream-ordered, meaning they execute in submission
order as if on a single queue. This provides sequential consistency:

- Operations appear to complete in submission order
- Writes from operation N are visible to operation N+1
- You don't need explicit synchronization between dependent operations
- Independent operations may execute concurrently internally for performance

Example (no explicit waits needed between dependent operations):

.. code-block:: python
   :caption: Stream-ordered operation sequence

    usd_handle, _ = physx.add_usd("scene.usda")  # Returns immediately
    physx.step(dt, time)                         # Sees the USD load (stream-ordered)
    binding = physx.create_tensor_binding(...)   # Sees step results
    binding.read(output)                       # Reads current state

Use wait_op() when:

- Before accessing results outside the stream (e.g., reading data on CPU/GPU)
- To ensure operations complete before program exit
- For explicit synchronization points in your application

Thread Safety
-------------
- Multiple PhysX instances: fully thread-safe
- Single instance: NOT thread-safe. Use external synchronization if calling
  from multiple threads

"""

import ctypes
import os
import threading
from ctypes import (
    POINTER,
    byref,
    c_char_p,
    c_float,
    c_int64,
    c_uint8,
    c_uint32,
    c_uint64,
    c_void_p,
    cast,
)
from pathlib import Path

from packaging.version import Version

from ._bindings import (  # Tensor type enum constants, log levels, log callback type
    OVPHYSX_API_NOT_FOUND,
    OVPHYSX_API_SUCCESS,
    OVPHYSX_API_TIMEOUT,
    OVPHYSX_INVALID_HANDLE,
    OVPHYSX_LOG_ERROR,
    OVPHYSX_LOG_INFO,
    OVPHYSX_LOG_VERBOSE,
    OVPHYSX_LOG_WARNING,
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
    _lib,
    kDLCPU,
    kDLFloat,
    kDLInt,
    ovphysx_create_args,
    ovphysx_cuda_sync_t,
    ovphysx_enqueue_result_t,
    ovphysx_op_wait_result_t,
    ovphysx_result_t,
    ovphysx_log_fn,
    ovphysx_string_t,
    ovphysx_tensor_binding_desc_t,
    ovphysx_tensor_spec_t,
)
from ._version import __version__ as _python_version
from .dlpack import (
    DLDataType,
    DLDevice,
    DLTensor,
)


def _check_version_match() -> None:
    import logging

    from . import _bindings

    logger = logging.getLogger(__name__)
    native_version = _bindings.get_native_version_string()
    if not native_version:
        # Native library didn't report a version - warn but don't fail
        logger.warning(
            "ovphysx native library did not report a version string. "
            "Version compatibility check skipped. This may indicate an old or "
            "development build of the native library."
        )
        return

    try:
        python_base = Version(_python_version).base_version
        native_base = Version(native_version).base_version
    except Exception as exc:
        raise RuntimeError(
            "Failed to parse ovphysx version for compatibility check. "
            f"Python: '{_python_version}', native: '{native_version}'."
        ) from exc

    if python_base != native_base:
        raise RuntimeError(
            "ovphysx Python package version does not match the native library. "
            f"Python: '{_python_version}' (base {python_base}), "
            f"native: '{native_version}' (base {native_base}). "
            "Reinstall the wheel or set OVPHYSX_LIB to the matching library. "
            "To bypass this check, pass ignore_version_mismatch=True."
        )


class TensorBinding:
    """Tensor binding for bulk physics data access via DLPack.

    A tensor binding connects a USD prim pattern to a tensor type, enabling efficient
    bulk read/write of physics data (poses, velocities, joint positions, etc.).

    This is a synchronous API - operations complete before returning.

    Usage patterns:

    - Context manager (auto-cleanup)::

        with physx.create_tensor_binding("/World/robot*", OVPHYSX_TENSOR_RIGID_BODY_POSE_F32) as binding:
            poses = np.zeros(binding.shape, dtype=np.float32)
            binding.read(poses)
            # ... modify poses ...
            binding.write(poses)
        # Auto-destroyed here

    - Manual (explicit cleanup)::

        binding = physx.create_tensor_binding("/World/robot*", OVPHYSX_TENSOR_RIGID_BODY_POSE_F32)
        poses = np.zeros(binding.shape, dtype=np.float32)
        binding.read(poses)
        binding.destroy()
    """

    def __init__(self, sdk, handle: int, tensor_type: int, ndim: int, shape: tuple):
        """Initialize tensor binding (created by PhysX.create_tensor_binding)."""
        self._sdk = sdk
        self._handle = handle
        self._tensor_type = tensor_type
        self._ndim = ndim
        self._shape = shape
        self._destroyed = False
        self._lock = threading.Lock()

    def __repr__(self) -> str:
        state = "destroyed" if self._destroyed else "alive"
        return (
            f"TensorBinding(handle={self._handle}, tensor_type={self._tensor_type}, "
            f"shape={self._shape}, state={state})"
        )

    @property
    def handle(self) -> int:
        """Get the binding handle."""
        return self._handle

    @property
    def tensor_type(self) -> int:
        """Get the tensor type enum value."""
        return self._tensor_type

    @property
    def ndim(self) -> int:
        """Get the number of dimensions (2 or 3)."""
        return self._ndim

    @property
    def shape(self) -> tuple:
        """Get tensor shape as tuple.

        Returns:
            - 2D tensors: (N, C) where N=count, C=components (e.g., 7 for pose, 6 for velocity)
            - 3D tensors: (N, L, C) where L=links for articulation link data
        """
        return self._shape

    @property
    def count(self) -> int:
        """Get number of entities (first dimension of shape)."""
        return self._shape[0] if self._shape else 0

    def _check_sdk_valid(self) -> None:
        """Ensure the parent PhysX instance is still alive."""
        if self._sdk._omni_physx_sdk_handle is None:
            raise RuntimeError("Cannot use TensorBinding: parent PhysX instance has been released.")

    def read(self, tensor) -> None:
        """Read simulation data into a user-provided tensor (synchronous).

        The tensor must have matching shape and dtype (float32). Can be a NumPy array,
        PyTorch tensor, or any object with __dlpack__ protocol.

        Args:
            tensor: DLPack-compatible tensor with pre-allocated storage matching self.shape.
                   Must be float32 on matching device (CPU or GPU).

        Preconditions:
            - This binding is not destroyed.
            - tensor has matching shape, dtype (float32), and device.
        Side effects:
            - Blocks until data is available and writes into the provided tensor.
        Ownership/Lifetime:
            - Caller owns tensor storage and must keep it alive for the duration of the call.
        Threading:
            - Serialized per binding via an internal lock.
        Errors:
            - RuntimeError if read fails (shape mismatch, device mismatch, etc.).
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("TensorBinding has been destroyed")
            self._check_sdk_valid()

            # Check for read-only NumPy arrays to prevent C-layer SIGSEGV
            if hasattr(tensor, "flags"):
                try:
                    writeable = tensor.flags["WRITEABLE"]
                except (KeyError, TypeError, AttributeError):
                    writeable = True
                if not writeable:
                    raise ValueError(
                        "Array passed to binding.read() must be writeable. "
                        "Use np.array(buf) to create a writeable copy."
                    )

            dl_tensor, keepalive = self._acquire_dltensor(tensor)

            result = self._sdk._lib.ovphysx_read_tensor_binding(
                self._sdk._omni_physx_sdk_handle.value, self._handle, ctypes.byref(dl_tensor)
            )

            # Keep capsule alive through the native call.
            _ = keepalive

            if result.status != OVPHYSX_API_SUCCESS:
                error_msg = self._sdk._err_and_destroy(result)
                raise RuntimeError(f"Failed to read tensor binding: {error_msg}")

    def write(self, tensor, indices=None, mask=None) -> None:
        """Write data from a user-provided tensor into the simulation (synchronous).

        The tensor must have matching shape and dtype (float32). Can be a NumPy array,
        PyTorch tensor, or any object with __dlpack__ protocol.

        Args:
            tensor: DLPack-compatible tensor with data to write, shape matching self.shape.
                   Must be float32 on matching device (CPU or GPU).
            indices: Optional int32 tensor of indices for partial update. If provided,
                    only the rows at the given indices are written. The tensor argument
                    must still be full shape [N, ...] matching the binding spec; only the
                    selected rows are applied. Shape of indices: [K] where K <= N.
            mask: Optional bool/uint8 tensor for masked update. If provided, only
                 elements where mask[i] != 0 are written. Shape: [N] matching the
                 binding's first dimension. When mask is provided, tensor must be
                 full shape [N, ...]. If both mask and indices are provided, mask
                 takes precedence and indices are ignored (with a warning).

                 Note: there is no corresponding ``read(..., mask=...)``; reads always
                 return the full [N,...] tensor and callers can index the result themselves.
                 This write-only mask design matches other RL physics APIs such as Newton's
                 selectionAPI, where masks selectively apply actions but observations are
                 always returned in full.

        Preconditions:
            - This binding is not destroyed.
            - tensor matches shape, dtype (float32), and device.
            - indices (if provided) is int32 and within bounds.
            - mask (if provided) is bool/uint8 with shape [N] on matching device.
        Side effects:
            - Updates simulation state for the bound entities.
        Ownership/Lifetime:
            - Caller owns tensor/indices/mask storage and must keep it alive for the call.
        Threading:
            - Serialized per binding via an internal lock.
        Errors:
            - RuntimeError if write fails (shape mismatch, device mismatch, etc.).
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("TensorBinding has been destroyed")
            self._check_sdk_valid()

            if mask is not None and indices is not None:
                import warnings
                warnings.warn(
                    "Both mask and indices provided to TensorBinding.write(); mask takes precedence, indices ignored.",
                    UserWarning,
                    stacklevel=2,
                )
                indices = None

            if mask is not None:
                dl_tensor, keepalive = self._acquire_dltensor(tensor)
                dl_mask, mask_keepalive = self._acquire_dltensor(mask)

                result = self._sdk._lib.ovphysx_write_tensor_binding_masked(
                    self._sdk._omni_physx_sdk_handle.value, self._handle, ctypes.byref(dl_tensor), ctypes.byref(dl_mask)
                )

                # prevent GC of DLPack capsules while the C call borrows their pointers
                _ = (keepalive, mask_keepalive)

                if result.status != OVPHYSX_API_SUCCESS:
                    error_msg = self._sdk._err_and_destroy(result)
                    raise RuntimeError(f"Failed to write tensor binding (masked): {error_msg}")
                return

            dl_tensor, keepalive = self._acquire_dltensor(tensor)

            # Handle optional indices
            if indices is not None:
                idx_dl_tensor, idx_keepalive = self._acquire_dltensor(indices)
                idx_ptr = ctypes.byref(idx_dl_tensor)
            else:
                idx_dl_tensor = None
                idx_keepalive = None
                idx_ptr = None

            result = self._sdk._lib.ovphysx_write_tensor_binding(
                self._sdk._omni_physx_sdk_handle.value, self._handle, ctypes.byref(dl_tensor), idx_ptr
            )

            # Keep capsules alive through the native call.
            _ = (keepalive, idx_keepalive)

            if result.status != OVPHYSX_API_SUCCESS:
                error_msg = self._sdk._err_and_destroy(result)
                raise RuntimeError(f"Failed to write tensor binding: {error_msg}")

    def _acquire_dltensor(self, obj) -> tuple[DLTensor, object | None]:
        """Extract DLTensor and a keepalive reference (if needed)."""
        from ._dlpack_utils import acquire_dltensor

        return acquire_dltensor(obj)

    def destroy(self) -> None:
        """Release binding resources.

        Safe to call multiple times. Called automatically on garbage collection
        or when exiting a context manager.

        Preconditions:
            - Binding must not be in use by other threads.
        Side effects:
            - Releases native resources and invalidates the binding.
        Ownership/Lifetime:
            - After destruction, the binding cannot be used.
        Threading:
            - Serialized per binding via an internal lock.
        Errors:
            - RuntimeError if destruction fails.
        """
        with self._lock:
            if self._destroyed:
                return
            self._check_sdk_valid()

            result = self._sdk._lib.ovphysx_destroy_tensor_binding(self._sdk._omni_physx_sdk_handle.value, self._handle)
            if result.status != OVPHYSX_API_SUCCESS:
                error_msg = self._sdk._err_and_destroy(result)
                raise RuntimeError(f"Failed to destroy tensor binding: {error_msg}")
            self._destroyed = True

    def __enter__(self):
        """Enter context manager.

        Preconditions:
            - Binding is valid and not destroyed.
        Side effects:
            - None.
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Exit context manager - destroys binding.

        Side effects:
            - Releases binding resources.
        """
        self.destroy()
        return False

    def __del__(self):
        """Destructor - ensures cleanup on garbage collection.

        Note: During interpreter shutdown, calling C functions may fail.
        We check sys.is_finalizing() to avoid spurious errors.
        """
        try:
            import sys
            if sys.is_finalizing():
                return
        except Exception:
            return
        if not self._destroyed:
            try:
                self.destroy()
            except Exception:
                pass


# =============================================================================
# Module-level logging configuration
# =============================================================================

def set_log_level(level: int) -> None:
    """Set the global log level threshold.

    Messages below this level are suppressed for all outputs (console and
    registered callbacks). Callable at any time, including before instance
    creation.

    Args:
        level: Log level threshold (OVPHYSX_LOG_NONE through OVPHYSX_LOG_VERBOSE).
               Default: OVPHYSX_LOG_WARNING.

    Raises:
        ValueError: If level is out of range. No state change is applied.
    """
    result = _lib.ovphysx_set_log_level(level)
    if result.status != OVPHYSX_API_SUCCESS:
        err_msg = ""
        if result.error.length > 0:
            err_msg = ctypes.string_at(result.error.ptr, result.error.length).decode("utf-8", errors="replace")
            _lib.ovphysx_destroy_error(result.error)
        raise ValueError(err_msg or f"Invalid log level: {level}")


def get_log_level() -> int:
    """Get the current global log level threshold.

    Returns:
        The current log level (int matching ovphysx_log_level_t constants).
    """
    return int(_lib.ovphysx_get_log_level())


def enable_default_log_output(enable: bool = True) -> None:
    """Enable or disable Carbonite's built-in console log output.

    By default, Carbonite logs to the console. When custom callbacks are
    registered (or :func:`enable_python_logging` is active), both the
    built-in console output and the callbacks receive messages, which may
    cause duplicate output.

    Call with ``False`` to suppress the built-in console output while
    keeping callbacks active. Call with ``True`` to re-enable it.

    This is independent of callback registration and the global log level.

    Args:
        enable: ``True`` to enable (default), ``False`` to disable.
    """
    result = _lib.ovphysx_enable_default_log_output(enable)
    if result.status != OVPHYSX_API_SUCCESS:
        err_msg = ""
        if result.error.length > 0:
            err_msg = ctypes.string_at(result.error.ptr, result.error.length).decode("utf-8", errors="replace")
            _lib.ovphysx_destroy_error(result.error)
        raise RuntimeError(err_msg or "Failed to set default log output")


# Internal state for Python logging bridge
_python_log_callback = None  # prevent GC of the ctypes callback
_python_log_logger_name = None


def enable_python_logging(logger_name: str = "ovphysx") -> None:
    """Route native log messages to Python's logging module.

    Registers a C-level callback that forwards every message (at or above
    the global log level) to ``logging.getLogger(logger_name)`` at the
    corresponding Python log level.

    Call :func:`disable_python_logging` to stop forwarding.

    Args:
        logger_name: Name of the Python logger to route to (default: "ovphysx").
    """
    import logging as _logging

    global _python_log_callback, _python_log_logger_name

    if _python_log_callback is not None:
        disable_python_logging()

    py_logger = _logging.getLogger(logger_name)

    # Map ovphysx levels to Python logging levels
    level_map = {
        OVPHYSX_LOG_ERROR: _logging.ERROR,
        OVPHYSX_LOG_WARNING: _logging.WARNING,
        OVPHYSX_LOG_INFO: _logging.INFO,
        OVPHYSX_LOG_VERBOSE: _logging.DEBUG,
    }

    @ovphysx_log_fn
    def _callback(level, message, user_data):
        py_level = level_map.get(level, _logging.DEBUG)
        try:
            text = message.decode("utf-8") if isinstance(message, bytes) else str(message)
        except Exception:
            text = str(message)
        py_logger.log(py_level, "%s", text)

    _python_log_callback = _callback
    _python_log_logger_name = logger_name

    result = _lib.ovphysx_register_log_callback(_python_log_callback, None)
    if result.status != OVPHYSX_API_SUCCESS:
        _python_log_callback = None
        _python_log_logger_name = None
        err_msg = ""
        if result.error.length > 0:
            err_msg = ctypes.string_at(result.error.ptr, result.error.length).decode("utf-8", errors="replace")
            _lib.ovphysx_destroy_error(result.error)
        raise RuntimeError(f"Failed to enable Python logging: {err_msg}")


def disable_python_logging() -> None:
    """Stop routing native log messages to Python's logging module.

    If :func:`enable_python_logging` was not called, this is a no-op.
    """
    global _python_log_callback, _python_log_logger_name

    if _python_log_callback is None:
        return

    result = _lib.ovphysx_unregister_log_callback(_python_log_callback, None)
    if result.error.length > 0:
        _lib.ovphysx_destroy_error(result.error)

    _python_log_callback = None
    _python_log_logger_name = None


class PhysX:
    """High-level wrapper around the C API using ctypes."""

    def __init__(
        self,
        settings: dict[str, bool | int | float | str] | None = None,
        ignore_version_mismatch: bool = False,
        device: str | int | None = None,
        gpu_index: int = 0,
    ) -> None:
        """Initialize PhysX SDK.

        Args:
            settings: Dictionary of Carbonite settings to apply at initialization.
                Keys are setting paths (e.g., "/physics/physxDispatcher"),
                values can be bool, int, float, or str.
                See SDK documentation for supported settings.

                Common settings:
                - "/physics/cudaDevice": int (-1 = auto) - CUDA device ordinal
                - "/physics/numThreads": int (0 = auto) - Number of worker threads
            ignore_version_mismatch: Skip Python/native version match check
            device: Select simulation device for this instance.
                - "gpu" or 0: GPU simulation (default)
                - "cpu" or 1: CPU simulation (required for the TensorAPI "numpy" frontend)

                .. warning::

                    All instances within the same process must use the same device mode.
                    The device setting is applied to process-global Carbonite/PhysX state
                    during the first ``PhysX()`` call and cannot be changed afterwards.
                    Creating a CPU instance after a GPU instance (or vice versa) in the
                    same process will raise an error.

            gpu_index: CUDA device ordinal to use when device="gpu" (default: 0).

        Preconditions:
            - The ovphysx native library is available in the environment or bundled deps.
        Side effects:
            - Loads runtime components and initializes process-level resources.
        Ownership/Lifetime:
            - The instance owns native resources until release() or context exit.
        Threading:
            - The instance is not thread-safe for concurrent calls.
        Errors:
            - Raises RuntimeError for initialization failures or version mismatch.

        Note:
            Log level is configured globally via ovphysx.set_log_level() before
            creating an instance. Default: OVPHYSX_LOG_WARNING.
        """
        self._lib = _lib

        if not ignore_version_mismatch:
            _check_version_match()

        # Create args structure
        args = ovphysx_create_args()

        # Device selection
        # Note: This is NOT a Carbonite setting; it is an ovphysx create-arg.
        if device is None:
            # Keep default (OVPHYSX_DEVICE_GPU = 0)
            args.device = 0
        elif isinstance(device, str):
            d = device.strip().lower()
            if d == "cpu":
                args.device = 1  # OVPHYSX_DEVICE_CPU
            elif d == "gpu":
                args.device = 0  # OVPHYSX_DEVICE_GPU
            else:
                raise ValueError(f"Invalid device '{device}'. Expected 'cpu' or 'gpu'.")
        else:
            # int-like
            args.device = int(device)

        args.gpu_index = int(gpu_index)

        # Set up bundled dependencies path (for self-contained wheel)
        bundled_deps_path = self._get_bundled_deps_path()
        if bundled_deps_path:
            args.bundled_deps_path = ovphysx_string_t(bundled_deps_path)
        else:
            args.bundled_deps_path = ovphysx_string_t()

        # Process settings dictionary
        if settings:
            settings_count = len(settings)
            keys_list = list(settings.keys())
            values_list = [self._setting_value_to_str(v) for v in settings.values()]

            # Allocate ctypes arrays for keys and values
            settings_keys_c = (ovphysx_string_t * settings_count)()
            settings_values_c = (ovphysx_string_t * settings_count)()
            for i, (k, v) in enumerate(zip(keys_list, values_list)):
                settings_keys_c[i] = ovphysx_string_t(k)
                settings_values_c[i] = ovphysx_string_t(v)

            args.settings_keys = cast(settings_keys_c, POINTER(ovphysx_string_t))
            args.settings_values = cast(settings_values_c, POINTER(ovphysx_string_t))
            args.settings_count = settings_count
        else:
            args.settings_keys = None
            args.settings_values = None
            args.settings_count = 0

        self._omni_physx_sdk_handle = c_uint64(OVPHYSX_INVALID_HANDLE)
        result = self._lib.ovphysx_create_instance(byref(args), byref(self._omni_physx_sdk_handle))
        if result.status != OVPHYSX_API_SUCCESS:
            error_msg = self._err_and_destroy(result)
            raise RuntimeError(f"ovphysx_create_instance() failed: {error_msg}")
        if self._omni_physx_sdk_handle.value == OVPHYSX_INVALID_HANDLE:
            self._omni_physx_sdk_handle = None
            raise RuntimeError("ovphysx_create_instance() returned invalid handle")

        # Register this instance so tensor plugins know Carbonite is ready
        from . import _register_physx_instance

        _register_physx_instance(self)

    @staticmethod
    def _setting_value_to_str(value: bool | int | float | str) -> str:
        """Convert a Python value to a string for the C settings API."""
        if isinstance(value, bool):
            return "true" if value else "false"
        return str(value)

    def _get_bundled_deps_path(self) -> str:
        """Get the path to bundled dependencies.

        Returns:
        - Path to complete bundle (wheel mode): Use pre-packaged deps
        - None (no bundle): Use default runtime discovery (RPATH/OVPHYSX_PLUGINS_DIR)

        The bundle must be complete (carbonite_plugins + v2/extensions).
        Returns None if bundle doesn't exist - runtime discovery is used.

        Development mode: If OVPHYSX_ROOT points to _build directory,
        skip bundled deps and use live build output.
        """
        # Check if we're in development mode (tests use live build from _build)
        # If OVPHYSX_ROOT ends with "_build", prefer live build output over bundled deps
        sdk_root = os.environ.get("OVPHYSX_ROOT", "")
        if sdk_root.endswith("_build") or sdk_root.endswith("_build/"):
            # Development mode - use _build/lib/deps
            return None

        module_dir = Path(__file__).parent
        bundled_deps_dir = module_dir / "deps"

        # Only use bundled deps if COMPLETE bundle exists
        # (both carbonite_plugins AND v2 with extensions)
        carbonite_plugins = bundled_deps_dir / "carbonite_plugins"
        v2_dir = bundled_deps_dir / "v2"

        if carbonite_plugins.exists() and v2_dir.exists():
            # Complete bundle - use wheel mode
            return str(bundled_deps_dir)

        # No complete bundle - use runtime discovery
        return None

    @staticmethod
    def _ovx_to_str(s: ovphysx_string_t) -> str:
        if not s.ptr or s.length == 0:
            return ""
        try:
            return ctypes.string_at(s.ptr, s.length).decode("utf-8", errors="replace")
        except Exception:
            return ""

    def _err_and_destroy(self, result) -> str:
        """Convert result.error to string and destroy it to avoid leaks."""
        msg = "Unknown error"
        err = getattr(result, "error", None)
        if err and err.ptr:
            try:
                msg = self._ovx_to_str(err) or msg
            finally:
                try:
                    self._lib.ovphysx_destroy_error(err)
                except Exception:
                    pass
        return msg

    def add_usd(self, usd_path: str, path_prefix: str = "") -> tuple[int, int]:
        """Add USD file to stage (async, returns immediately).

        Args:
            usd_path: Path to USD file
            path_prefix: Optional prefix for all paths in USD

        Returns:
            Tuple of (usd_handle, op_index). The usd_handle is used for remove_usd().
            op_index can be used with wait_op() if you need explicit synchronization.

        Examples:
            # Simple usage (stream-ordered)
            usd_handle, _ = physx.add_usd("scene.usda")
            physx.step(dt, time)  # Automatically waits for USD to load

            # Explicit synchronization (if needed before non-stream operations)
            usd_handle, op = physx.add_usd("scene.usda")
            physx.wait_op(op)  # Ensure load completes before external access

        Preconditions:
            - usd_path must exist and be readable.
        Side effects:
            - Enqueues stage load and allocates runtime resources.
        Ownership/Lifetime:
            - The returned usd_handle is owned by this instance until remove_usd/reset.
        Threading:
            - Do not call concurrently on the same instance without external sync.
        Errors:
            - Raises RuntimeError on load or enqueue failure.
        """
        usd_handle = c_uint64(0)
        result = self._lib.ovphysx_add_usd(
            self._omni_physx_sdk_handle.value,
            ovphysx_string_t(usd_path),
            ovphysx_string_t(path_prefix),
            byref(usd_handle),
        )

        if result.status != OVPHYSX_API_SUCCESS:
            error_msg = self._err_and_destroy(result)
            raise RuntimeError(f"Failed to add USD: {error_msg}")

        return (usd_handle.value, result.op_index)

    def remove_usd(self, usd_handle: int) -> int:
        """Remove USD file from stage (async, returns immediately).

        Args:
            usd_handle: Handle from add_usd()

        Returns:
            op_index (can be used with wait_op() for explicit synchronization)

        Example:
            # Simple usage (stream-ordered)
            physx.remove_usd(usd_handle)
            physx.step(dt, time)  # Automatically waits for removal

        Preconditions:
            - usd_handle must be valid for this instance.
        Side effects:
            - Removes USD data from the runtime stage when complete.
        Ownership/Lifetime:
            - usd_handle becomes invalid after completion.
        Threading:
            - Do not call concurrently on the same instance without external sync.
        Errors:
            - Raises RuntimeError on failure.
        """
        result = self._lib.ovphysx_remove_usd(self._omni_physx_sdk_handle.value, usd_handle)

        if result.status == OVPHYSX_API_NOT_FOUND:
            error_msg = self._err_and_destroy(result)
            raise ValueError(f"usd_handle {usd_handle} is not active: {error_msg}")

        if result.status != OVPHYSX_API_SUCCESS:
            error_msg = self._err_and_destroy(result)
            raise RuntimeError(f"Failed to remove USD: {error_msg}")

        return result.op_index

    def reset(self) -> int:
        """Reset stage to empty (async, invalidates all usd_handles).

        Returns:
            op_index (can be used with wait_op() for explicit synchronization)

        Example:
            # Simple usage (stream-ordered)
            physx.reset()
            usd_handle, _ = physx.add_usd("new_scene.usda")  # Automatically waits for reset

        Preconditions:
            - Instance must be valid.
        Side effects:
            - Clears the runtime stage and invalidates all USD handles.
        Ownership/Lifetime:
            - All previously returned usd_handles become invalid after completion.
        Threading:
            - Do not call concurrently on the same instance without external sync.
        Errors:
            - Raises RuntimeError on failure.
        """
        result = self._lib.ovphysx_reset(self._omni_physx_sdk_handle.value)

        if result.status != OVPHYSX_API_SUCCESS:
            error_msg = self._err_and_destroy(result)
            raise RuntimeError(f"Failed to reset: {error_msg}")

        return result.op_index

    def clone(self, source_path: str, target_paths: list[str]) -> int:
        """Clone a USD prim hierarchy to create multiple Fabric-based copies (asynchronous, returns immediately).

        Creates physics-optimized clones in Fabric for high-performance simulation.
        The source prim must exist in the loaded USD stage and have physics properties.
        Due to stream-ordered execution, subsequent operations automatically see the result of clone.

        Args:
            source_path: USD path of the source prim hierarchy to clone (e.g., "/World/env0")
            target_paths: List of USD paths for the cloned hierarchies (e.g., ["/World/env1", "/World/env2"])

        Returns:
            op_index (can be used with wait_op() for explicit synchronization)

        Raises:
            ValueError: If source_path is empty, target_paths is empty, or any target path matches source path
            RuntimeError: If clone fails to queue or if no USD scene is loaded

        Preconditions:
            - A USD stage is loaded and source_path exists.
            - target_paths are unique and do not already exist.
        Side effects:
            - Creates cloned runtime prims that immediately participate in simulation.
        Ownership/Lifetime:
            - Clones remain valid until reset/remove_usd.
        Threading:
            - Do not call concurrently on the same instance without external sync.
        Errors:
            - Raises ValueError for invalid inputs.
            - Raises RuntimeError on enqueue or internal failure.
        """
        if not source_path:
            raise ValueError("source_path must be a non-empty string")
        if not target_paths:
            raise ValueError("target_paths must be a non-empty list")
        if source_path in target_paths:
            raise ValueError(f"Target path cannot be the same as source path '{source_path}'")

        num_targets = len(target_paths)
        target_array = (ovphysx_string_t * num_targets)()
        for i, path in enumerate(target_paths):
            target_array[i] = ovphysx_string_t(path)

        result = self._lib.ovphysx_clone(
            self._omni_physx_sdk_handle.value, ovphysx_string_t(source_path), target_array, c_uint32(num_targets)
        )
        if result.status != OVPHYSX_API_SUCCESS:
            error_msg = self._err_and_destroy(result)
            raise RuntimeError(f"Failed to queue clone: {error_msg}")

        return result.op_index

    def step(self, dt: float, sim_time: float) -> int:
        """Initiate physics step (async, returns op_index).

        Args:
            dt: Delta time for this step
            sim_time: Current simulation time

        Returns:
            op_index (can be used with wait_op() for explicit synchronization)

        Examples:
            # Simple usage (stream-ordered)
            physx.step(0.016, 0.0)
            binding.read(output)  # Automatically waits for step

            # Explicit wait (if accessing results outside stream)
            op = physx.step(0.016, 0.0)
            physx.wait_op(op)  # Ensure step completes before external GPU work

        Preconditions:
            - A USD stage is loaded if physics content is expected.
        Side effects:
            - Advances simulation time and mutates physics state.
        Ownership/Lifetime:
            - Returned op_index is single-use and must be waited once if needed.
        Threading:
            - Do not call concurrently on the same instance without external sync.
        Errors:
            - Raises RuntimeError on failure to enqueue.
        """
        result = self._lib.ovphysx_step(self._omni_physx_sdk_handle.value, c_float(dt), c_float(sim_time))

        if result.status != OVPHYSX_API_SUCCESS:
            error_msg = self._err_and_destroy(result)
            raise RuntimeError(f"Failed to step: {error_msg}")

        return result.op_index

    def wait_op(self, op_index: int, timeout_ns: int = None) -> None:
        """Wait for operation(s) to complete.

        Args:
            op_index: Operation index to wait for, or OVPHYSX_OP_INDEX_ALL for all ops
            timeout_ns: Timeout in nanoseconds (None = infinite, 0 = poll)

        Raises:
            RuntimeError: If operation failed
            TimeoutError: If timeout expired (e.g., when polling with timeout_ns=0 and the operation is not ready)

        Preconditions:
            - op_index must be valid and not previously consumed.
        Side effects:
            - Consumes op_index on successful wait.
        Ownership/Lifetime:
            - Error strings returned by the API are destroyed internally.
        Threading:
            - Do not wait on the same op_index from multiple threads.
        """
        if timeout_ns is None:
            timeout_ns = 0xFFFFFFFFFFFFFFFF  # Infinite wait

        wait_result = ovphysx_op_wait_result_t()
        result = self._lib.ovphysx_wait_op(self._omni_physx_sdk_handle.value, op_index, timeout_ns, byref(wait_result))

        # Check for errors
        if wait_result.num_errors > 0:
            errors = []
            for i in range(wait_result.num_errors):
                op_error = wait_result.errors[i]
                error_msg = str(op_error.error) if op_error.error.ptr else "Unknown error"
                errors.append(f"op {op_error.op_index}: {error_msg}")

            # Clean up error strings
            self._lib.ovphysx_destroy_errors(wait_result.errors, wait_result.num_errors)

            raise RuntimeError(f"Operation(s) failed:\n  " + "\n  ".join(errors))

        # Check for timeout
        if result.status == OVPHYSX_API_TIMEOUT:
            if result.error.ptr:
                try:
                    self._lib.ovphysx_destroy_error(result.error)
                except Exception:
                    pass
            raise TimeoutError(f"Operation {op_index} timed out")

        if result.status != OVPHYSX_API_SUCCESS:
            error_msg = self._err_and_destroy(result)
            raise RuntimeError(f"wait_op failed: {error_msg}")

        if result.error.ptr:
            # Clean up any stray error string even on success
            try:
                self._lib.ovphysx_destroy_error(result.error)
            except Exception:
                pass

    def wait_all(self, timeout_ns: int | None = None) -> None:
        """Wait for all pending operations (convenience wrapper for wait_op(ALL)).

        Args:
            timeout_ns: Timeout in nanoseconds (None = infinite, 0 = poll)

        Preconditions:
            - Instance must be valid.
        Side effects:
            - Blocks until all pending ops complete or timeout.
        Threading:
            - Safe to call if no other thread is waiting on specific op_indices.
        Errors:
            - Raises RuntimeError on failure.
            - Raises TimeoutError if timeout expired (e.g., when polling with timeout_ns=0 and operations are not ready).
        """
        self.wait_op(OVPHYSX_OP_INDEX_ALL, timeout_ns=timeout_ns)

    def get_stage_id(self) -> int:
        """Return the attached USD stage id.

        Preconditions:
            - Instance must be valid.
        Side effects:
            - None.
        Ownership/Lifetime:
            - Returned ID is owned by the runtime and may change after stage reload.
        Threading:
            - Do not call concurrently with stage mutation without external sync.
        Errors:
            - Returns -1 if the query fails.
        """
        sid = c_int64(0)
        result = self._lib.ovphysx_get_stage_id(self._omni_physx_sdk_handle.value, byref(sid))

        if result.status != OVPHYSX_API_SUCCESS:
            self._err_and_destroy(result)  # Free C error buffer
            return -1

        return int(sid.value)

    def set_setting(self, key: str, value: bool | int | float | str) -> None:
        """Set a Carbonite setting at runtime.

        Args:
            key: Setting path (e.g., "/physics/physxDispatcher")
            value: Setting value (bool, int, float, or str)

        Preconditions:
            - key must be a valid setting path.
        Side effects:
            - Updates a process-global setting affecting all instances.
        Ownership/Lifetime:
            - Values are copied to native strings during the call.
        Threading:
            - Safe to call from any thread; avoid concurrent configuration changes.
        Errors:
            - Raises RuntimeError on failure.
        """
        value_str = self._setting_value_to_str(value)
        result = self._lib.ovphysx_set_setting(
            self._omni_physx_sdk_handle.value, ovphysx_string_t(key), ovphysx_string_t(value_str)
        )

        if result.status != OVPHYSX_API_SUCCESS:
            error_msg = self._err_and_destroy(result)
            raise RuntimeError(f"Failed to set setting '{key}': {error_msg}")

    def get_setting(self, key: str) -> str | None:
        """Get a Carbonite setting value.

        Args:
            key: Setting path (e.g., "/physics/physxDispatcher")

        Preconditions:
            - key must be a valid setting path.
        Side effects:
            - None.
        Ownership/Lifetime:
            - Returns a new Python string on success.
        Threading:
            - Safe to call from any thread.
        Errors:
            - Returns None if the lookup fails.
        """
        buffer_size = 256
        buffer = ctypes.create_string_buffer(buffer_size)
        value_out = ovphysx_string_t()
        value_out.ptr = ctypes.cast(buffer, c_char_p)
        value_out.length = buffer_size

        required_size = ctypes.c_size_t(0)
        result = self._lib.ovphysx_get_setting(
            self._omni_physx_sdk_handle.value, ovphysx_string_t(key), byref(value_out), byref(required_size)
        )

        if result.status != OVPHYSX_API_SUCCESS:
            self._err_and_destroy(result)  # Free C error buffer
            return None

        return str(value_out)

    def release(self) -> None:
        """Release PhysX instance.

        Preconditions:
            - Instance is valid and not in use by other threads.
        Side effects:
            - Releases native resources and unregisters the instance.
        Ownership/Lifetime:
            - The instance becomes unusable after release.
        Threading:
            - Do not call concurrently with other operations on this instance.
        Errors:
            - Errors during cleanup are suppressed for robustness.
        """
        if hasattr(self, "_omni_physx_sdk_handle") and self._omni_physx_sdk_handle is not None:
            try:
                result = self._lib.ovphysx_destroy_instance(self._omni_physx_sdk_handle.value)
            except Exception:
                pass
            self._omni_physx_sdk_handle = None

        # Unregister this instance so tensor modules can give accurate errors.
        try:
            from . import _unregister_physx_instance

            _unregister_physx_instance(self)
        except Exception:
            pass

    # -------------------------------------------------------------------------
    # Tensor Binding API - efficient bulk access to physics simulation data
    # -------------------------------------------------------------------------

    def create_tensor_binding(
        self,
        pattern: str = None,
        prim_paths: list[str] = None,
        tensor_type: int = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32,
    ) -> TensorBinding:
        """Create tensor binding for bulk physics data access (synchronous).

        A tensor binding connects USD prims (by pattern or explicit paths) to a
        tensor type, enabling efficient bulk read/write of physics data.

        :param pattern: USD path glob pattern (e.g., "/World/robot*", "/World/env[N]/robot").
            Mutually exclusive with ``prim_paths``.
        :param prim_paths: Explicit list of prim paths. Mutually exclusive with ``pattern``.
        :param tensor_type: Tensor type enum value (``OVPHYSX_TENSOR_*``).
        :returns: TensorBinding object for reading/writing tensor data.
        :raises ValueError: If neither ``pattern`` nor ``prim_paths`` is provided, or both are.
        :raises RuntimeError: If binding creation fails.

        Examples::

            # Read all robot poses by pattern
            with physx.create_tensor_binding(
                "/World/robot*", tensor_type=OVPHYSX_TENSOR_RIGID_BODY_POSE_F32
            ) as binding:
                poses = np.zeros(binding.shape, dtype=np.float32)
                binding.read(poses)

            # Set joint position targets for specific articulations
            binding = physx.create_tensor_binding(
                prim_paths=["/World/env1/robot", "/World/env2/robot"],
                tensor_type=OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32,
            )
            targets = np.zeros(binding.shape, dtype=np.float32)
            binding.write(targets)
            binding.destroy()

        Preconditions:
            - Exactly one of ``pattern`` or ``prim_paths`` must be provided.
            - A USD stage is loaded.
        Side effects:
            - Allocates native binding resources.
        Ownership/Lifetime:
            - Returned TensorBinding owns native resources until ``destroy()``.
        Threading:
            - Do not call concurrently on the same instance without external sync.
        Errors:
            - Raises ``ValueError`` for invalid arguments.
            - Raises ``RuntimeError`` on creation failure.
        """
        # Validate arguments
        if pattern is None and prim_paths is None:
            raise ValueError("Either 'pattern' or 'prim_paths' must be provided")
        if pattern is not None and prim_paths is not None:
            raise ValueError("Cannot specify both 'pattern' and 'prim_paths'")

        # Build descriptor
        desc = ovphysx_tensor_binding_desc_t()
        desc.tensor_type = tensor_type

        # Keep references to prevent garbage collection during C call
        c_paths_array = None
        c_paths_refs = []

        if prim_paths is not None:
            # Use explicit prim paths
            desc.pattern = ovphysx_string_t()  # Empty pattern
            desc.prim_paths_count = len(prim_paths)
            c_paths_array = (ovphysx_string_t * len(prim_paths))()
            for i, path in enumerate(prim_paths):
                c_paths_array[i] = ovphysx_string_t(path)
                c_paths_refs.append(c_paths_array[i])
            desc.prim_paths = cast(c_paths_array, POINTER(ovphysx_string_t))
        else:
            # Use pattern
            desc.pattern = ovphysx_string_t(pattern)
            desc.prim_paths = None
            desc.prim_paths_count = 0

        # Create binding
        handle = c_uint64(0)
        result = self._lib.ovphysx_create_tensor_binding(self._omni_physx_sdk_handle.value, byref(desc), byref(handle))

        if result.status != OVPHYSX_API_SUCCESS:
            error_msg = self._err_and_destroy(result)
            raise RuntimeError(f"Failed to create tensor binding: {error_msg}")

        # Get tensor spec to know the shape
        spec = ovphysx_tensor_spec_t()
        spec_result = self._lib.ovphysx_get_tensor_binding_spec(
            self._omni_physx_sdk_handle.value, handle.value, byref(spec)
        )

        if spec_result.status != OVPHYSX_API_SUCCESS:
            # Clean up the binding we just created
            self._lib.ovphysx_destroy_tensor_binding(self._omni_physx_sdk_handle.value, handle.value)
            error_msg = self._err_and_destroy(spec_result)
            raise RuntimeError(f"Failed to get tensor binding spec: {error_msg}")

        # Extract shape from spec
        ndim = spec.ndim
        shape = tuple(spec.shape[i] for i in range(ndim))

        return TensorBinding(self, handle.value, tensor_type, ndim, shape)

    def warmup_gpu(self) -> None:
        """Explicitly initialize GPU buffers (synchronous).

        In GPU mode, PhysX DirectGPU buffers need one simulation step to initialize.
        This is normally done automatically on the first tensor read (auto-warmup).

        Call this function explicitly if you want to:
        - Control exactly when the warmup latency occurs
        - Avoid a latency spike on the first tensor read
        - Verify GPU initialization succeeded before starting your main loop

        This function is idempotent - calling it multiple times has no effect after
        the first successful call. In CPU mode, this is a no-op.

        Raises:
            RuntimeError: If GPU warmup fails.

        Preconditions:
            - Instance is configured for GPU mode.
        Side effects:
            - Advances simulation by a minimal timestep on first call.
        Ownership/Lifetime:
            - No ownership changes; affects current stage state.
        Threading:
            - Do not call concurrently with other operations on this instance.
        """
        result = self._lib.ovphysx_warmup_gpu(self._omni_physx_sdk_handle.value)

        if result.status != OVPHYSX_API_SUCCESS:
            error_msg = self._err_and_destroy(result)
            raise RuntimeError(f"Failed to warmup GPU: {error_msg}")

    def __enter__(self) -> "PhysX":
        """Enter context manager.

        Preconditions:
            - Instance is valid.
        """
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        """Exit context manager - releases the instance."""
        self.release()

    def __del__(self) -> None:
        """Destructor - ensures cleanup on garbage collection.

        Note: During interpreter shutdown, calling C functions may fail.
        We check sys.is_finalizing() to avoid spurious errors.
        """
        try:
            import sys
            if sys.is_finalizing():
                return
        except Exception:
            return
        try:
            self.release()
        except Exception:
            pass
