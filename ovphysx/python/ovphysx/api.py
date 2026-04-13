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
    binding.read(output)                         # Reads current state

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
    c_int32,
    c_int64,
    c_uint8,
    c_uint32,
    c_uint64,
    c_void_p,
    cast,
)
from pathlib import Path
from typing import TYPE_CHECKING

from packaging.version import Version

if TYPE_CHECKING:
    from .config import PhysXConfig

from ._bindings import (
    OP_INDEX_ALL,
)
from ._bindings import OVPHYSX_INVALID_HANDLE as _INVALID_HANDLE
from ._bindings import (
    ContactEventHeader,
    ContactPoint,
    FrictionAnchor,
    _lib,
    ovphysx_articulation_metadata_t,
    ovphysx_create_args,
    ovphysx_log_fn,
    ovphysx_op_wait_result_t,
    ovphysx_string_t,
    ovphysx_tensor_binding_desc_t,
    ovphysx_tensor_spec_t,
    ovphysx_config_entry_t,
    ovphysx_scene_query_geometry_desc_t,
    ovphysx_scene_query_hit_t,
)
from . import __version__ as _python_version
from .types import ApiStatus, DeviceType, LogLevel, SceneQueryGeometryType, SceneQueryMode, TensorType

# Set of tensor types for which articulation metadata (dof_count, body_names, etc.) is valid.
# Derived from the enum so it stays in sync automatically when new ARTICULATION_ types are added.
_ARTICULATION_TENSOR_TYPES: frozenset[int] = frozenset(t for t in TensorType if t.name.startswith("ARTICULATION_"))
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


from collections import namedtuple

_CacheEntry = namedtuple("_CacheEntry", [
    "tensor",        # the tensor object (identity check)
    "c_func",        # C function pointer
    "sdk_handle",    # SDK handle integer
    "bind_handle",   # binding handle
    "dl_ptr",        # ctypes pointer to DLTensor
    "dl_tensor",     # DLTensor struct (prevents GC)
    "keepalive",     # DLPack capsule keepalive (prevents GC)
    "data_ptr",      # data pointer at cache time (int or None)
    "ptr_getter",    # callable to re-extract data pointer (or None)
])


def _detect_data_ptr(tensor):
    """The cache checks ``tensor is cached_tensor`` (Python object identity),
    but a numpy array can be resized in place -- ``buf.resize((bigger,),
    refcheck=False)`` -- which reallocates the underlying memory while
    ``id(buf)`` stays the same.  Without this guard the cache would pass the
    old DLTensor (pointing to freed memory) to the C layer.

    Returns ``(current_ptr, getter_fn)`` where *getter_fn* re-extracts the
    pointer on the fast path, or ``(None, None)`` for types we cannot cheaply
    check (raw DLTensor structs, etc.).
    """
    if hasattr(tensor, "ctypes"):
        return tensor.ctypes.data, lambda t: t.ctypes.data
    if hasattr(tensor, "data_ptr") and callable(tensor.data_ptr):
        return tensor.data_ptr(), lambda t: t.data_ptr()
    return None, None


class TensorBinding:
    """Tensor binding for bulk physics data access via DLPack.

    A tensor binding connects a USD prim pattern to a tensor type, enabling efficient
    bulk read/write of physics data (poses, velocities, joint positions, etc.).

    This is a synchronous API - operations complete before returning.

    Usage patterns:

    - Context manager (auto-cleanup)::

        with sdk.create_tensor_binding("/World/robot*", TensorType.RIGID_BODY_POSE) as binding:
            poses = np.zeros(binding.shape, dtype=np.float32)
            binding.read(poses)
            # ... modify poses ...
            binding.write(poses)
        # Auto-destroyed here

    - Manual (explicit cleanup)::

        binding = sdk.create_tensor_binding("/World/robot*", TensorType.RIGID_BODY_POSE)
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
        self._artic_metadata = None
        self._read_cache = None
        self._write_cache = None

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

    def _get_artic_metadata(self) -> ovphysx_articulation_metadata_t:
        """Return cached articulation metadata struct (one C call total per binding)."""
        if self._tensor_type not in _ARTICULATION_TENSOR_TYPES:
            try:
                name = TensorType(self._tensor_type).name
            except ValueError:
                name = str(self._tensor_type)
            raise TypeError(
                f"Articulation metadata (dof_count, body_names, etc.) is not available "
                f"for tensor type {name!r}. Only ARTICULATION_* tensor types carry this "
                f"metadata. Use an articulation tensor type such as "
                f"TensorType.ARTICULATION_DOF_POSITION."
            )
        if self._artic_metadata is None:
            self._check_sdk_valid()
            meta = ovphysx_articulation_metadata_t()
            result = _lib.ovphysx_get_articulation_metadata(
                self._sdk._omni_physx_sdk_handle.value, self._handle, ctypes.byref(meta)
            )
            if result.status != ApiStatus.SUCCESS:
                error_msg = self._sdk._get_last_error()
                raise RuntimeError(f"Failed to get articulation metadata: {error_msg}")
            self._artic_metadata = meta
        return self._artic_metadata

    @property
    def dof_count(self) -> int:
        """Number of degrees of freedom (DOFs). 0 if not an articulation binding."""
        return self._get_artic_metadata().dof_count

    @property
    def body_count(self) -> int:
        """Number of links."""
        return self._get_artic_metadata().body_count

    @property
    def is_fixed_base(self) -> bool:
        """Whether the articulation has a fixed base."""
        return bool(self._get_artic_metadata().is_fixed_base)

    def _get_names(self, c_func, count_prop: str) -> list[str]:
        """Helper to fetch name lists from metadata queries."""
        self._check_sdk_valid()
        count = getattr(self, count_prop)
        if count == 0:
            return []
        names_arr = (ovphysx_string_t * count)()
        out_count = c_uint32(0)
        result = c_func(self._sdk._omni_physx_sdk_handle.value, self._handle, names_arr, count, ctypes.byref(out_count))
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._sdk._get_last_error()
            raise RuntimeError(f"Failed to get names: {error_msg}")
        return [str(names_arr[i]) for i in range(out_count.value)]

    @property
    def dof_names(self) -> list[str]:
        """List of DOF names (one per DOF)."""
        return self._get_names(_lib.ovphysx_articulation_get_dof_names, "dof_count")

    @property
    def body_names(self) -> list[str]:
        """List of body/link names."""
        return self._get_names(_lib.ovphysx_articulation_get_body_names, "body_count")

    @property
    def joint_count(self) -> int:
        """Number of joints per articulation."""
        return self._get_artic_metadata().joint_count

    @property
    def joint_names(self) -> list[str]:
        """List of joint names."""
        return self._get_names(_lib.ovphysx_articulation_get_joint_names, "joint_count")

    @property
    def fixed_tendon_count(self) -> int:
        """Number of fixed tendons per articulation (0 if none).

        Use to decide whether to allocate buffers for fixed tendon property
        tensors (types 80-85) and to skip tendon code paths when T=0.
        """
        return self._get_artic_metadata().fixed_tendon_count

    @property
    def spatial_tendon_count(self) -> int:
        """Number of spatial tendons per articulation (0 if none).

        Use to decide whether to allocate buffers for spatial tendon property
        tensors (types 90-93) and to skip tendon code paths when T=0.
        """
        return self._get_artic_metadata().spatial_tendon_count

    def read(self, tensor) -> None:
        """Read simulation data into a user-provided tensor (synchronous).

        The tensor must have matching shape and dtype (float32). Can be a NumPy array,
        PyTorch tensor, or any object with __dlpack__ protocol.

        When called repeatedly with the same buffer object, an internal cache
        skips DLPack acquisition and attribute chain lookups, giving
        near-raw-C-call overhead.  The numpy writeable guard is preserved
        on the fast path.  Callers that want this fast path should reuse
        the same tensor object with unchanged backing storage across calls.
        Calling ``numpy.ndarray.resize()`` or ``torch.Tensor.resize_()``
        between calls is safe (a staleness guard detects the pointer change
        and rebuilds the cache) but defeats the purpose of caching.

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
            - Do not mutate the tensor's backing storage (``resize()``, ``set_()``,
              etc.) between cached calls.  For numpy and torch tensors, a staleness
              guard detects common mutations and falls back to the slow path; other
              types rely on the caller honouring this contract.
        Threading:
            - Serialized per binding via an internal lock.
        Errors:
            - RuntimeError if read fails (shape mismatch, device mismatch, etc.).
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("TensorBinding has been destroyed")

            # Fast path: same tensor object as last successful read.
            cached = self._read_cache
            if cached is not None and cached.tensor is tensor:
                if cached.ptr_getter is not None and cached.ptr_getter(tensor) != cached.data_ptr:
                    self._read_cache = None
                    # Backing storage changed (e.g. numpy resize) -- fall through to slow path.
                else:
                    if self._sdk._omni_physx_sdk_handle is None:
                        raise RuntimeError("Cannot use TensorBinding: parent PhysX instance has been released.")
                    if hasattr(tensor, "flags") and not getattr(tensor.flags, "writeable", True):
                        raise ValueError(
                            "Array passed to binding.read() must be writeable. "
                            "Use np.array(buf) to create a writeable copy."
                        )
                    result = cached.c_func(cached.sdk_handle, cached.bind_handle, cached.dl_ptr)
                    if result.status != ApiStatus.SUCCESS:
                        self._read_cache = None
                        error_msg = self._sdk._get_last_error()
                        raise RuntimeError(f"Failed to read tensor binding: {error_msg}")
                    return

            # Slow path: first call or different tensor.
            self._check_sdk_valid()

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
            dl_ptr = ctypes.byref(dl_tensor)
            c_func = self._sdk._lib.ovphysx_read_tensor_binding
            sdk_handle_val = self._sdk._omni_physx_sdk_handle.value

            result = c_func(sdk_handle_val, self._handle, dl_ptr)

            # Keep capsule alive through the native call.
            _ = keepalive

            if result.status != ApiStatus.SUCCESS:
                error_msg = self._sdk._get_last_error()
                raise RuntimeError(f"Failed to read tensor binding: {error_msg}")

            _cached_ptr, _ptr_getter = _detect_data_ptr(tensor)
            self._read_cache = _CacheEntry(tensor, c_func, sdk_handle_val, self._handle, dl_ptr, dl_tensor, keepalive, _cached_ptr, _ptr_getter)

    def write(self, tensor, indices=None, mask=None) -> None:
        """Write data from a user-provided tensor into the simulation (synchronous).

        The tensor must have matching shape and dtype (float32). Can be a NumPy array,
        PyTorch tensor, or any object with __dlpack__ protocol.

        When called repeatedly with the same buffer object and no indices/mask,
        an internal cache skips DLPack acquisition and attribute chain lookups,
        giving near-raw-C-call overhead.  Callers that want this fast path
        should reuse the same tensor object with unchanged backing storage
        across calls; see :meth:`read` for the full contract.

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

            # Fast path: same tensor, no indices, no mask.
            if indices is None and mask is None:
                cached = self._write_cache
                if cached is not None and cached.tensor is tensor:
                    if cached.ptr_getter is not None and cached.ptr_getter(tensor) != cached.data_ptr:
                        self._write_cache = None
                        # Backing storage changed -- fall through to slow path.
                    else:
                        if self._sdk._omni_physx_sdk_handle is None:
                            raise RuntimeError("Cannot use TensorBinding: parent PhysX instance has been released.")
                        result = cached.c_func(cached.sdk_handle, cached.bind_handle, cached.dl_ptr, None)
                        if result.status != ApiStatus.SUCCESS:
                            self._write_cache = None
                            error_msg = self._sdk._get_last_error()
                            raise RuntimeError(f"Failed to write tensor binding: {error_msg}")
                        return

            # Slow path: first call, different tensor, or indices/mask provided.
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

                if result.status != ApiStatus.SUCCESS:
                    error_msg = self._sdk._get_last_error()
                    raise RuntimeError(f"Failed to write tensor binding (masked): {error_msg}")
                return

            dl_tensor, keepalive = self._acquire_dltensor(tensor)
            dl_ptr = ctypes.byref(dl_tensor)
            c_func = self._sdk._lib.ovphysx_write_tensor_binding
            sdk_handle_val = self._sdk._omni_physx_sdk_handle.value

            if indices is not None:
                idx_dl_tensor, idx_keepalive = self._acquire_dltensor(indices)
                idx_ptr = ctypes.byref(idx_dl_tensor)
            else:
                idx_dl_tensor = None
                idx_keepalive = None
                idx_ptr = None

            result = c_func(sdk_handle_val, self._handle, dl_ptr, idx_ptr)

            # Keep capsules alive through the native call.
            _ = (keepalive, idx_keepalive)

            if result.status != ApiStatus.SUCCESS:
                error_msg = self._sdk._get_last_error()
                raise RuntimeError(f"Failed to write tensor binding: {error_msg}")

            # Only cache simple writes (no indices, no mask).
            if indices is None and mask is None:
                _cached_ptr, _ptr_getter = _detect_data_ptr(tensor)
                self._write_cache = _CacheEntry(tensor, c_func, sdk_handle_val, self._handle, dl_ptr, dl_tensor, keepalive, _cached_ptr, _ptr_getter)

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
            # Mark destroyed after the native destroy attempt so __del__ does not retry after SDK release.
            self._destroyed = True
            self._read_cache = None
            self._write_cache = None
            if result.status != ApiStatus.SUCCESS:
                error_msg = self._sdk._get_last_error()
                raise RuntimeError(f"Failed to destroy tensor binding: {error_msg}")

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


class ContactBinding:
    """Contact force binding backed by IRigidContactView.

    Do not instantiate directly. Use :meth:`PhysxSDK.create_contact_binding` to
    obtain an instance.
    """

    def __init__(self, sdk, handle: int, sensor_count: int, filter_count: int):
        self._sdk = sdk
        self._handle = handle
        self._sensor_count = sensor_count
        self._filter_count = filter_count
        self._destroyed = False
        self._lock = threading.Lock()

    def _check_sdk_valid(self) -> None:
        if self._sdk._omni_physx_sdk_handle is None:
            raise RuntimeError("Cannot use ContactBinding: parent PhysX instance has been released.")

    @property
    def sensor_count(self) -> int:
        """Number of sensor bodies matched."""
        return self._sensor_count

    @property
    def filter_count(self) -> int:
        """Number of filter bodies per sensor (0 when no filters specified)."""
        return self._filter_count

    def read_net_forces(self, output) -> None:
        """Read net contact forces into output. Expected shape: [sensor_count, 3].

        The dt for impulse-to-force conversion is taken automatically from the
        last :meth:`PhysxSDK.step` call.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("ContactBinding has been destroyed")
            self._check_sdk_valid()
            from ._dlpack_utils import acquire_dltensor

            dl_tensor, keepalive = acquire_dltensor(output)
            result = _lib.ovphysx_read_contact_net_forces(
                self._sdk._omni_physx_sdk_handle.value, self._handle, ctypes.byref(dl_tensor)
            )
            _ = keepalive
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"Failed to read net contact forces: {self._sdk._get_last_error()}")

    def read_force_matrix(self, output) -> None:
        """Read contact force matrix into output. Expected shape: [sensor_count, filter_count, 3].

        The dt for impulse-to-force conversion is taken automatically from the
        last :meth:`PhysxSDK.step` call.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("ContactBinding has been destroyed")
            self._check_sdk_valid()
            from ._dlpack_utils import acquire_dltensor

            dl_tensor, keepalive = acquire_dltensor(output)
            result = _lib.ovphysx_read_contact_force_matrix(
                self._sdk._omni_physx_sdk_handle.value, self._handle, ctypes.byref(dl_tensor)
            )
            _ = keepalive
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"Failed to read contact force matrix: {self._sdk._get_last_error()}")

    def destroy(self) -> None:
        """Release contact binding resources.

        Safe to call multiple times. Captures strong references to the SDK and
        library before the C call to guard against GC ordering issues (Python
        may collect self._sdk before self if both go out of scope together).
        """
        with self._lock:
            if not self._destroyed:
                sdk = self._sdk
                sdk_handle = sdk._omni_physx_sdk_handle
                if sdk_handle is None:
                    self._destroyed = True
                    return
                result = _lib.ovphysx_destroy_contact_binding(sdk_handle.value, self._handle)
                self._destroyed = True
                if result.status != ApiStatus.SUCCESS:
                    raise RuntimeError(f"Failed to destroy contact binding: {sdk._get_last_error()}")

    def __del__(self):
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

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.destroy()


# =============================================================================
# Remote storage credential configuration (process-wide)
# =============================================================================


def configure_s3(
    host: str,
    bucket: str,
    region: str,
    access_key_id: str,
    secret_access_key: str,
    session_token: str | None = None,
) -> None:
    """Configure S3 credentials for remote USD loading via HTTPS S3 URLs.

    Credentials are process-global and take effect immediately for all
    :class:`PhysX` instances. Call before :meth:`PhysX.add_usd` with an
    S3 HTTPS URL.

    Args:
        host: S3 endpoint (e.g., ``"my-bucket.s3.us-east-1.amazonaws.com"``).
        bucket: S3 bucket name.
        region: AWS region (e.g., ``"us-east-1"``).
        access_key_id: AWS access key ID.
        secret_access_key: AWS secret access key.
        session_token: STS session token (``None`` if not using temporary
            credentials).

    Raises:
        ValueError: If a required parameter is empty.
        RuntimeError: If the OmniClient library is unavailable.
    """
    result = _lib.ovphysx_configure_s3(
        host.encode("utf-8"),
        bucket.encode("utf-8"),
        region.encode("utf-8"),
        access_key_id.encode("utf-8"),
        secret_access_key.encode("utf-8"),
        session_token.encode("utf-8") if session_token else None,
    )
    if result.status != ApiStatus.SUCCESS:
        err = _lib.ovphysx_get_last_error()
        err_msg = ""
        if err and err.ptr:
            err_msg = ctypes.string_at(err.ptr, err.length).decode(
                "utf-8", errors="replace"
            )
        exc_cls = ValueError if result.status == ApiStatus.INVALID_ARGUMENT else RuntimeError
        raise exc_cls(err_msg or "Failed to configure S3 credentials")


def configure_azure_sas(
    host: str,
    container: str,
    sas_token: str,
) -> None:
    """Configure an Azure SAS token for remote USD loading via Azure Blob Storage.

    Credentials are process-global and take effect immediately for all
    :class:`PhysX` instances. Call before :meth:`PhysX.add_usd` with an
    Azure Blob URL.

    Args:
        host: Azure Blob host (e.g.,
            ``"myaccount.blob.core.windows.net"``).
        container: Azure container name.
        sas_token: SAS token string (without leading ``'?'``).

    Raises:
        ValueError: If a required parameter is empty.
        RuntimeError: If the OmniClient library is unavailable.
    """
    result = _lib.ovphysx_configure_azure_sas(
        host.encode("utf-8"),
        container.encode("utf-8"),
        sas_token.encode("utf-8"),
    )
    if result.status != ApiStatus.SUCCESS:
        err = _lib.ovphysx_get_last_error()
        err_msg = ""
        if err and err.ptr:
            err_msg = ctypes.string_at(err.ptr, err.length).decode(
                "utf-8", errors="replace"
            )
        exc_cls = ValueError if result.status == ApiStatus.INVALID_ARGUMENT else RuntimeError
        raise exc_cls(err_msg or "Failed to configure Azure SAS token")


# =============================================================================
# Module-level logging configuration
# =============================================================================


def set_log_level(level: int) -> None:
    """Set the global log level threshold.

    Messages below this level are suppressed for all outputs (console and
    registered callbacks). Callable at any time, including before instance
    creation.

    Args:
        level: Log level threshold (LogLevel.VERBOSE through LogLevel.NONE).
               Default: LogLevel.WARNING.

    Raises:
        ValueError: If level is out of range. No state change is applied.
    """
    result = _lib.ovphysx_set_log_level(level)
    if result.status != ApiStatus.SUCCESS:
        err = _lib.ovphysx_get_last_error()
        err_msg = str(err) if err and err.ptr else ""
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
    if result.status != ApiStatus.SUCCESS:
        err = _lib.ovphysx_get_last_error()
        err_msg = str(err) if err and err.ptr else ""
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
        LogLevel.ERROR: _logging.ERROR,
        LogLevel.WARNING: _logging.WARNING,
        LogLevel.INFO: _logging.INFO,
        LogLevel.VERBOSE: _logging.DEBUG,
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
    if result.status != ApiStatus.SUCCESS:
        _python_log_callback = None
        _python_log_logger_name = None
        err = _lib.ovphysx_get_last_error()
        err_msg = str(err) if err and err.ptr else ""
        raise RuntimeError(f"Failed to enable Python logging: {err_msg}")


def disable_python_logging() -> None:
    """Stop routing native log messages to Python's logging module.

    If :func:`enable_python_logging` was not called, this is a no-op.
    """
    global _python_log_callback, _python_log_logger_name

    if _python_log_callback is None:
        return

    result = _lib.ovphysx_unregister_log_callback(_python_log_callback, None)
    if result.status != 0:
        import logging as _logging
        err = _lib.ovphysx_get_last_error()
        err_msg = str(err) if err and err.ptr else "unknown error"
        _logging.getLogger(__name__).warning("Failed to unregister Python log callback: %s", err_msg)

    _python_log_callback = None
    _python_log_logger_name = None


class PhysX:
    """High-level wrapper around the C API using ctypes."""

    def __init__(
        self,
        config: "PhysXConfig | None" = None,
        ignore_version_mismatch: bool = False,
        device: str | int | None = None,
        gpu_index: int = 0,
    ) -> None:
        """Initialize PhysX SDK.

        Args:
            config: Typed config dataclass. Only non-None fields are applied.

                Example::

                    from ovphysx import PhysX, PhysXConfig
                    physx = PhysX(config=PhysXConfig(
                        disable_contact_processing=True,
                        num_threads=4,
                    ))
            ignore_version_mismatch: Skip Python/native version match check
            device: Select simulation device for this instance.
                - "auto" or 0: AUTO (GPU preferred). If CUDA is available, use GPU.
                  Otherwise, fall back to CPU. This is the recommended default.
                - "gpu" or 1: GPU REQUIRED. If CUDA is unavailable, initialization fails.
                - "cpu" or 2: CPU ONLY (required for the TensorAPI "numpy" frontend)

                .. warning::

                    All instances within the same process must use the same device mode.
                    The device setting is applied to process-global Carbonite/PhysX state
                    during the first ``PhysX()`` call and cannot be changed afterwards.
                    Creating a CPU instance after a GPU instance (or vice versa) in the
                    same process will raise an error.

            gpu_index: CUDA device ordinal to use when device="gpu" (default: 0).
                Special value: -1 requests PhysX's default CUDA selection
                (equivalent to /physics/cudaDevice=-1).

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
            creating an instance. Default: LogLevel.WARNING.
        """
        self._lib = _lib

        if not ignore_version_mismatch:
            _check_version_match()

        # Create args structure
        args = ovphysx_create_args()

        # Device selection
        # Note: This is NOT a Carbonite setting; it is an ovphysx create-arg.
        if device is None:
            args.device = DeviceType.AUTO
        elif isinstance(device, str):
            d = device.strip().lower()
            if d == "cpu":
                args.device = DeviceType.CPU
            elif d == "gpu":
                args.device = DeviceType.GPU
            elif d == "auto":
                args.device = DeviceType.AUTO
            else:
                raise ValueError(f"Invalid device '{device}'. Expected 'auto', 'cpu', or 'gpu'.")
        else:
            d = int(device)
            if d not in (DeviceType.AUTO, DeviceType.GPU, DeviceType.CPU):
                raise ValueError(
                    f"Invalid device value {d}. Expected {DeviceType.AUTO} (AUTO), "
                    f"{DeviceType.GPU} (GPU), or {DeviceType.CPU} (CPU), "
                    f"or use string: 'gpu', 'cpu', 'auto'."
                )
            args.device = d

        args.gpu_index = int(gpu_index)

        # Set up bundled dependencies path (for self-contained wheel)
        bundled_deps_path = self._get_bundled_deps_path()
        if bundled_deps_path:
            args.bundled_deps_path = ovphysx_string_t(bundled_deps_path)
        else:
            args.bundled_deps_path = ovphysx_string_t()

        # Build config entries from PhysXConfig dataclass
        all_entries = []
        if config is not None:
            from .config import _to_c_config
            all_entries.extend(_to_c_config(config))

        if all_entries:
            config_arr = (ovphysx_config_entry_t * len(all_entries))(*all_entries)
            args.config_entries = cast(config_arr, POINTER(ovphysx_config_entry_t))
            args.config_entry_count = len(all_entries)
        else:
            args.config_entries = None
            args.config_entry_count = 0

        self._omni_physx_sdk_handle = c_uint64(_INVALID_HANDLE)
        result = self._lib.ovphysx_create_instance(byref(args), byref(self._omni_physx_sdk_handle))
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            if result.status == ApiStatus.GPU_NOT_AVAILABLE:
                raise RuntimeError(
                    f"GPU requested but not available: {error_msg}. "
                    f"Use device='cpu' or device='auto' to run without a GPU."
                )
            if "device mode" in error_msg.lower() and "locked" in error_msg.lower():
                from .types import PhysXDeviceError
                raise PhysXDeviceError(
                    f"{error_msg} To use a different device mode, "
                    f"run in a separate subprocess "
                    f"(e.g. subprocess.run([sys.executable, 'script.py']))."
                )
            raise RuntimeError(f"ovphysx_create_instance() failed: {error_msg}")
        if self._omni_physx_sdk_handle.value == _INVALID_HANDLE:
            self._omni_physx_sdk_handle = None
            raise RuntimeError("ovphysx_create_instance() returned invalid handle")

        # Register this instance so tensor plugins know Carbonite is ready
        from . import _register_physx_instance

        _register_physx_instance(self)

    def _check_valid(self) -> None:
        if self._omni_physx_sdk_handle is None:
            raise RuntimeError(
                "PhysX instance has been released. Create a new PhysX() instance."
            )

    @property
    def handle(self) -> int:
        """The raw ``ovphysx_handle_t`` for this instance (read-only).

        Use this when passing the handle to C/C++ code that calls the
        ovphysx C API directly (e.g. ``ovphysx_get_physx_ptr``).

        Raises:
            RuntimeError: If the instance has been released.
        """
        self._check_valid()
        return self._omni_physx_sdk_handle.value

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

    def _get_last_error(self) -> str:
        """Get the last error message from TLS."""
        err = self._lib.ovphysx_get_last_error()
        if err and err.ptr:
            try:
                return self._ovx_to_str(err) or "Unknown error"
            except Exception:
                return "Unknown error"
        return "Unknown error"

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
        self._check_valid()
        usd_handle = c_uint64(0)
        result = self._lib.ovphysx_add_usd(
            self._omni_physx_sdk_handle.value,
            ovphysx_string_t(usd_path),
            ovphysx_string_t(path_prefix),
            byref(usd_handle),
        )

        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
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
        self._check_valid()
        result = self._lib.ovphysx_remove_usd(self._omni_physx_sdk_handle.value, usd_handle)

        if result.status == ApiStatus.NOT_FOUND:
            error_msg = self._get_last_error()
            raise ValueError(f"Failed to remove USD: {error_msg}")

        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
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
        self._check_valid()
        result = self._lib.ovphysx_reset(self._omni_physx_sdk_handle.value)

        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to reset: {error_msg}")

        return result.op_index

    def clone(self, source_path: str, target_paths: list[str],
              parent_transforms: list[tuple[float, float, float, float, float, float, float]] | None = None) -> int:
        """Clone a USD prim hierarchy to create multiple Fabric-based copies (asynchronous, returns immediately).

        Creates physics-optimized clones in Fabric for high-performance simulation.
        The source prim must exist in the loaded USD stage and have physics properties.
        Due to stream-ordered execution, subsequent operations automatically see the result of clone.

        Args:
            source_path: USD path of the source prim hierarchy to clone (e.g., "/World/env0")
            target_paths: List of USD paths for the cloned hierarchies (e.g., ["/World/env1", "/World/env2"])
            parent_transforms: Optional list of (px, py, pz, qx, qy, qz, qw) transforms
                for each target's parent Xform prim.  Position followed by quaternion
                rotation (imaginary-first, matching tensor API convention).
                Identity rotation = (0, 0, 0, 1).  Must have the same length as
                target_paths.  When provided, newly-created parent prims are placed at
                the given transform.  Pass None to use identity for all parents.

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
        self._check_valid()
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

        # Pack optional parent transforms into a flat float array [N*7].
        if parent_transforms is not None:
            if len(parent_transforms) != num_targets:
                raise ValueError(
                    f"parent_transforms length ({len(parent_transforms)}) "
                    f"must match target_paths length ({num_targets})"
                )
            xform_flat = [v for t7 in parent_transforms for v in t7]
            xform_array = (c_float * len(xform_flat))(*xform_flat)
            xform_ptr = ctypes.cast(xform_array, POINTER(c_float))
        else:
            xform_ptr = None

        result = self._lib.ovphysx_clone(
            self._omni_physx_sdk_handle.value, ovphysx_string_t(source_path),
            target_array, c_uint32(num_targets), xform_ptr
        )
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
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
        self._check_valid()
        result = self._lib.ovphysx_step(self._omni_physx_sdk_handle.value, c_float(dt), c_float(sim_time))

        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to step: {error_msg}")

        return result.op_index

    def step_sync(self, dt: float, sim_time: float) -> None:
        """Step simulation and wait for completion in a single call.

        Faster than ``step()`` + ``wait_op()`` for performance-critical
        applications like RL training that always wait immediately.

        Args:
            dt: Delta time [s] for this step.
            sim_time: Current simulation time [s].

        Raises:
            RuntimeError: If the step or wait fails.
        """
        self._check_valid()
        result = self._lib.ovphysx_step_sync(self._omni_physx_sdk_handle.value, c_float(dt), c_float(sim_time))
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"step_sync failed: {error_msg}")

    def step_n_sync(self, n: int, dt: float, current_time: float) -> None:
        """Run N steps in a single C call, saving (N-1) ctypes round-trips.

        Equivalent to calling ``step_sync(dt, current_time + i*dt)`` for
        i in ``[0, n)``, but with only one Python-to-C transition.

        Args:
            n: Number of steps to run (must be >= 1).
            dt: Duration of each step [s].
            current_time: Simulation time at the start of the first step [s].

        Raises:
            RuntimeError: If any step fails.
        """
        self._check_valid()
        result = self._lib.ovphysx_step_n_sync(
            self._omni_physx_sdk_handle.value, c_int32(n), c_float(dt), c_float(current_time)
        )
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"step_n_sync failed: {error_msg}")

    def wait_op(self, op_index: int, timeout_ns: int = None) -> None:
        """Wait for operation(s) to complete.

        Args:
            op_index: Operation index to wait for, or OP_INDEX_ALL for all ops
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

        Examples::

            # Blocking wait (default)
            physx.wait_op(op_index)

            # Non-blocking poll
            try:
                physx.wait_op(op_index, timeout_ns=0)
            except TimeoutError:
                pass  # operation not yet complete
        """
        self._check_valid()
        if timeout_ns is None:
            timeout_ns = 0xFFFFFFFFFFFFFFFF  # Infinite wait

        wait_result = ovphysx_op_wait_result_t()
        result = self._lib.ovphysx_wait_op(self._omni_physx_sdk_handle.value, op_index, timeout_ns, byref(wait_result))

        try:
            # Check for errors
            if wait_result.num_errors > 0:
                errors = []
                for i in range(wait_result.num_errors):
                    failed_op = wait_result.error_op_indices[i]
                    err_str = self._lib.ovphysx_get_last_op_error(failed_op)
                    error_msg = str(err_str) if err_str.ptr else "Unknown error"
                    errors.append(f"op {failed_op}: {error_msg}")

                raise RuntimeError("Operation(s) failed:\n  " + "\n  ".join(errors))

            # Check for timeout
            if result.status == ApiStatus.TIMEOUT:
                raise TimeoutError(f"Operation {op_index} timed out")

            if result.status != ApiStatus.SUCCESS:
                error_msg = self._get_last_error()
                raise RuntimeError(f"wait_op failed: {error_msg}")
        finally:
            self._lib.ovphysx_destroy_wait_result(byref(wait_result))

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
        # _check_valid() is called inside wait_op()
        self.wait_op(OP_INDEX_ALL, timeout_ns=timeout_ns)

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
        self._check_valid()
        sid = c_int64(0)
        result = self._lib.ovphysx_get_stage_id(self._omni_physx_sdk_handle.value, byref(sid))

        if result.status != ApiStatus.SUCCESS:
            self._get_last_error()  # discard; caller sees -1
            return -1

        return int(sid.value)

    def set_config(self, entry: ovphysx_config_entry_t) -> None:
        """Set a typed global config entry at runtime (process-global).

        Prefer the typed setters (:meth:`set_config_bool`, :meth:`set_config_int32`,
        :meth:`set_config_float`) for a cleaner API.

        Args:
            entry: Typed config entry (``ovphysx_config_entry_t``).
        """
        result = self._lib.ovphysx_set_global_config(entry)
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to set config: {error_msg}")

    def set_config_bool(self, key: int, value: bool) -> None:
        """Set a boolean config value at runtime (process-global).

        Args:
            key: Boolean config key (e.g., ``ConfigBool.DISABLE_CONTACT_PROCESSING``).
            value: Boolean value.
        """
        self._check_valid()
        from .config import _make_bool_entry
        self.set_config(_make_bool_entry(key, value))

    def set_config_int32(self, key: int, value: int) -> None:
        """Set an int32 config value at runtime (process-global).

        Args:
            key: Int32 config key (e.g., ``ConfigInt32.NUM_THREADS``).
            value: Int32 value.
        """
        self._check_valid()
        from .config import _make_int32_entry
        self.set_config(_make_int32_entry(key, value))

    def set_config_float(self, key: int, value: float) -> None:
        """Set a float config value at runtime (process-global).

        Args:
            key: Float config key.
            value: Float value.
        """
        self._check_valid()
        from .config import _make_float_entry
        self.set_config(_make_float_entry(key, value))

    def get_config_bool(self, key: int) -> bool:
        """Get a boolean config value.

        Args:
            key: Boolean config key (e.g., ``ovphysx.ConfigBool.DISABLE_CONTACT_PROCESSING``).

        Returns:
            Current boolean value.
        """
        self._check_valid()
        out = ctypes.c_bool(False)
        result = self._lib.ovphysx_get_global_config_bool(key, byref(out))
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to get config bool: {error_msg}")
        return bool(out.value)

    def get_config_int32(self, key: int) -> int:
        """Get an int32 config value.

        Args:
            key: Int32 config key (e.g., ``ovphysx.ConfigInt32.NUM_THREADS``).

        Returns:
            Current int32 value.
        """
        self._check_valid()
        out = c_int32(0)
        result = self._lib.ovphysx_get_global_config_int32(key, byref(out))
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to get config int32: {error_msg}")
        return int(out.value)

    def get_config_float(self, key: int) -> float:
        """Get a float config value.

        Args:
            key: Float config key.

        Returns:
            Current float value.
        """
        self._check_valid()
        out = c_float(0.0)
        result = self._lib.ovphysx_get_global_config_float(key, byref(out))
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to get config float: {error_msg}")
        return float(out.value)

    def get_config_string(self, key: int) -> str | None:
        """Get a string config value.

        Args:
            key: String config key from ``ovphysx.ConfigString``.

        Returns:
            Current string value, or None if not found.
        """
        self._check_valid()
        buffer_size = 256
        buffer = ctypes.create_string_buffer(buffer_size)
        value_out = ovphysx_string_t()
        value_out.ptr = ctypes.cast(buffer, c_char_p)
        value_out.length = buffer_size

        required_size = ctypes.c_size_t(0)
        result = self._lib.ovphysx_get_global_config_string(key, byref(value_out), byref(required_size))

        if result.status == ApiStatus.BUFFER_TOO_SMALL:
            buffer_size = required_size.value
            buffer = ctypes.create_string_buffer(buffer_size)
            value_out = ovphysx_string_t()
            value_out.ptr = ctypes.cast(buffer, c_char_p)
            value_out.length = buffer_size
            required_size = ctypes.c_size_t(0)
            result = self._lib.ovphysx_get_global_config_string(key, byref(value_out), byref(required_size))
        if result.status != ApiStatus.SUCCESS:
            self._get_last_error()
            return None
        return str(value_out)

    def release(self) -> None:
        """Release PhysX instance.

        Preconditions:
            - Instance is valid and not in use by other threads.
        Side effects:
            - Releases native resources and unregisters the instance.
        Ownership/Lifetime:
            - All tensor bindings and contact bindings created by this instance are
              automatically released.
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
        tensor_type: int = TensorType.RIGID_BODY_POSE,
    ) -> TensorBinding:
        """Create tensor binding for bulk physics data access (synchronous).

        A tensor binding connects USD prims (by pattern or explicit paths) to a
        tensor type, enabling efficient bulk read/write of physics data.

        :param pattern: USD path glob pattern (e.g., "/World/robot*", "/World/env[N]/robot").
            Mutually exclusive with ``prim_paths``.
        :param prim_paths: Explicit list of prim paths. Mutually exclusive with ``pattern``.
        :param tensor_type: Tensor type enum value (``TensorType.*``).
        :returns: TensorBinding object for reading/writing tensor data.
        :raises ValueError: If neither ``pattern`` nor ``prim_paths`` is provided, or both are.
        :raises RuntimeError: If binding creation fails.

        Examples::

            # Read all robot poses by pattern
            with sdk.create_tensor_binding(
                "/World/robot*", tensor_type=TensorType.RIGID_BODY_POSE
            ) as binding:
                poses = np.zeros(binding.shape, dtype=np.float32)
                binding.read(poses)

            # Set joint position targets for specific articulations
            binding = physx.create_tensor_binding(
                prim_paths=["/World/env1/robot", "/World/env2/robot"],
                tensor_type=TensorType.ARTICULATION_DOF_POSITION_TARGET,
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
        self._check_valid()
        # Validate arguments
        if pattern is None and prim_paths is None:
            raise ValueError("Either 'pattern' or 'prim_paths' must be provided")
        if pattern is not None and prim_paths is not None:
            raise ValueError("Cannot specify both 'pattern' and 'prim_paths'")
        if prim_paths is not None and len(prim_paths) == 0:
            raise ValueError("prim_paths must not be empty; pass at least one prim path.")

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

        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to create tensor binding: {error_msg}")

        # Get tensor spec to know the shape
        spec = ovphysx_tensor_spec_t()
        spec_result = self._lib.ovphysx_get_tensor_binding_spec(
            self._omni_physx_sdk_handle.value, handle.value, byref(spec)
        )

        if spec_result.status != ApiStatus.SUCCESS:
            # Clean up the binding we just created
            self._lib.ovphysx_destroy_tensor_binding(self._omni_physx_sdk_handle.value, handle.value)
            error_msg = self._get_last_error()
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
        self._check_valid()
        result = self._lib.ovphysx_warmup_gpu(self._omni_physx_sdk_handle.value)

        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to warmup GPU: {error_msg}")

    # ------------------------------------------------------------------
    # PhysX object interop
    # ------------------------------------------------------------------

    def get_physx_ptr(self, prim_path: str, physx_type: int) -> int:
        """Return a raw PhysX SDK pointer by USD prim path and type.

        Args:
            prim_path: Absolute USD prim path (e.g. ``"/World/physicsScene"``).
            physx_type: One of the ``PhysXType`` enum members (or matching int)
                indicating which PhysX object type to look up.

        Returns:
            Integer address of the PhysX pointer, or ``0`` if no object of
            the requested type exists at the given path.

        Raises:
            RuntimeError: On infrastructure failures (no stage loaded,
                invalid handle, etc.).  A missing object does **not** raise;
                check for a ``0`` return value instead.

        The returned address can be passed to C/C++ code that casts it to
        the appropriate PhysX type (see ``ovphysx_physx_type_t``).

        Pointer lifetime: valid until ``remove_usd()``, ``reset()``, or
        instance destruction. ``step()`` and ``clone()`` do NOT invalidate
        pointers. Do not call ``release()`` on returned pointers.

        Thread safety: PhysX APIs on returned pointers must only be called
        between simulation steps.
        """
        out = ctypes.c_void_p(0)
        result = _lib.ovphysx_get_physx_ptr(
            self._omni_physx_sdk_handle.value, prim_path.encode("utf-8"), ctypes.c_int(physx_type), ctypes.byref(out)
        )
        if result.status != ApiStatus.SUCCESS:
            if result.status == ApiStatus.NOT_FOUND:
                return 0
            raise RuntimeError(f"Failed to get PhysX ptr: {self._get_last_error()}")
        return out.value or 0

    # ------------------------------------------------------------------
    # Contact report
    # ------------------------------------------------------------------

    def get_contact_report(self, include_friction_anchors: bool = False) -> dict:
        """Get per-contact-point event data for the current simulation step.

        Use this for custom contact sensors, collision debugging, or per-point
        force analysis. For **aggregate force tensors** (net forces or force
        matrices between sensor/filter body sets), use
        :meth:`create_contact_binding` instead.

        Args:
            include_friction_anchors: If True, also return friction anchor data
                (position and impulse at each friction anchor point).

        Returns a dict with:
            - ``headers``: ctypes array of :class:`ContactEventHeader` structs
              describing each contact pair (actors, colliders, event type).
              Length is ``num_headers``.
            - ``num_headers`` (int): Number of contact event headers.
            - ``points``: ctypes array of :class:`ContactPoint` structs with
              per-contact-point data (position, normal, impulse, separation).
              Length is ``num_points``.
            - ``num_points`` (int): Number of contact point entries.
            - ``anchors`` (only if ``include_friction_anchors=True``): ctypes
              array of :class:`FrictionAnchor` structs. Length is ``num_anchors``.
            - ``num_anchors`` (int, only if ``include_friction_anchors=True``):
              Number of friction anchors.

        The arrays are zero-copy views into internal buffers, valid until the
        next simulation step. Individual fields are accessible by index::

            report = physx.get_contact_report()
            for i in range(report["num_headers"]):
                h = report["headers"][i]
                print(h.actor0, h.numContactData)
            for j in range(report["num_points"]):
                p = report["points"][j]
                print(p.position[0], p.normal[1], p.impulse[2])

        Prims must have ``PhysxContactReportAPI`` applied in the USD stage
        for contacts to be reported.

        Raises:
            RuntimeError: If the call fails.
        """
        headers_ptr = ctypes.POINTER(ContactEventHeader)()
        num_headers = ctypes.c_uint32(0)
        data_ptr = ctypes.POINTER(ContactPoint)()
        num_data = ctypes.c_uint32(0)
        anchors_ptr = ctypes.POINTER(FrictionAnchor)() if include_friction_anchors else None
        num_anchors = ctypes.c_uint32(0) if include_friction_anchors else None
        result = _lib.ovphysx_get_contact_report(
            self._omni_physx_sdk_handle.value,
            ctypes.byref(headers_ptr),
            ctypes.byref(num_headers),
            ctypes.byref(data_ptr),
            ctypes.byref(num_data),
            ctypes.byref(anchors_ptr) if anchors_ptr is not None else None,
            ctypes.byref(num_anchors) if num_anchors is not None else None,
        )
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"Failed to get contact report: {self._get_last_error()}")

        nh = num_headers.value
        np_ = num_data.value
        headers = (
            ctypes.cast(headers_ptr, ctypes.POINTER(ContactEventHeader * nh)).contents
            if nh
            else (ContactEventHeader * 0)()
        )
        points = ctypes.cast(data_ptr, ctypes.POINTER(ContactPoint * np_)).contents if np_ else (ContactPoint * 0)()
        out = {
            "headers": headers,
            "num_headers": nh,
            "points": points,
            "num_points": np_,
        }
        if include_friction_anchors:
            na = num_anchors.value
            anchors = (
                ctypes.cast(anchors_ptr, ctypes.POINTER(FrictionAnchor * na)).contents
                if na
                else (FrictionAnchor * 0)()
            )
            out["anchors"] = anchors
            out["num_anchors"] = na
        return out

    # ------------------------------------------------------------------
    # Scene queries
    # ------------------------------------------------------------------

    @staticmethod
    def _make_float3(values: list | tuple) -> "ctypes.Array[ctypes.c_float]":
        """Convert a 3-element sequence to a ctypes float[3]."""
        arr = (ctypes.c_float * 3)()
        arr[0], arr[1], arr[2] = float(values[0]), float(values[1]), float(values[2])
        return arr

    def _parse_scene_query_hits(self, hits_ptr, count: int) -> list[dict]:
        """Convert the internal hit buffer to a list of dicts."""
        results = []
        for i in range(count):
            h = hits_ptr[i]
            results.append({
                "collision": h.collision,
                "rigid_body": h.rigid_body,
                "proto_index": h.proto_index,
                "normal": (h.normal[0], h.normal[1], h.normal[2]),
                "position": (h.position[0], h.position[1], h.position[2]),
                "distance": h.distance,
                "face_index": h.face_index,
                "material": h.material,
            })
        return results

    def raycast(
        self,
        origin: tuple | list,
        direction: tuple | list,
        distance: float,
        mode: SceneQueryMode = SceneQueryMode.CLOSEST,
        both_sides: bool = False,
    ) -> list[dict]:
        """Cast a ray and return hits.

        Args:
            origin: Ray origin [x, y, z].
            direction: Normalized ray direction [x, y, z].
            distance: Maximum ray length (>= 0).
            mode: :class:`~ovphysx.SceneQueryMode` (CLOSEST, ANY, or ALL).
            both_sides: If True, test both sides of mesh triangles.

        Returns:
            List of hit dicts. Each dict contains ``collision``, ``rigid_body``,
            ``proto_index``, ``normal``, ``position``, ``distance``,
            ``face_index``, ``material``. For ANY mode, hit fields are zeroed.
        """
        o = self._make_float3(origin)
        d = self._make_float3(direction)
        hits_ptr = ctypes.POINTER(ovphysx_scene_query_hit_t)()
        count = ctypes.c_uint32(0)
        result = _lib.ovphysx_raycast(
            self._omni_physx_sdk_handle.value,
            o, d,
            ctypes.c_float(distance),
            ctypes.c_bool(both_sides),
            ctypes.c_int(int(mode)),
            ctypes.byref(hits_ptr),
            ctypes.byref(count))
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(
                f"Raycast failed: {self._get_last_error()}")
        return self._parse_scene_query_hits(hits_ptr, count.value)

    _GEOMETRY_PARAMS = {
        SceneQueryGeometryType.SPHERE: {"radius", "position"},
        SceneQueryGeometryType.BOX: {"half_extent", "position", "rotation"},
        SceneQueryGeometryType.SHAPE: {"prim_path"},
    }

    def _make_geometry_desc(self, geometry_type: SceneQueryGeometryType, **kwargs) -> "ovphysx_scene_query_geometry_desc_t":
        """Build an ovphysx_scene_query_geometry_desc_t from keyword args."""
        expected = self._GEOMETRY_PARAMS.get(geometry_type)
        if expected is not None:
            unexpected = set(kwargs) - expected
            if unexpected:
                raise TypeError(
                    f"Unexpected kwargs for {SceneQueryGeometryType(geometry_type).name}: {unexpected}")
        desc = ovphysx_scene_query_geometry_desc_t()
        desc.type = int(geometry_type)
        if geometry_type == SceneQueryGeometryType.SPHERE:
            desc._geom.sphere.radius = float(kwargs["radius"])
            pos = kwargs.get("position", (0, 0, 0))
            desc._geom.sphere.position[0] = float(pos[0])
            desc._geom.sphere.position[1] = float(pos[1])
            desc._geom.sphere.position[2] = float(pos[2])
        elif geometry_type == SceneQueryGeometryType.BOX:
            he = kwargs["half_extent"]
            desc._geom.box.half_extent[0] = float(he[0])
            desc._geom.box.half_extent[1] = float(he[1])
            desc._geom.box.half_extent[2] = float(he[2])
            pos = kwargs.get("position", (0, 0, 0))
            desc._geom.box.position[0] = float(pos[0])
            desc._geom.box.position[1] = float(pos[1])
            desc._geom.box.position[2] = float(pos[2])
            rot = kwargs.get("rotation", (0, 0, 0, 1))
            desc._geom.box.rotation[0] = float(rot[0])
            desc._geom.box.rotation[1] = float(rot[1])
            desc._geom.box.rotation[2] = float(rot[2])
            desc._geom.box.rotation[3] = float(rot[3])
        elif geometry_type == SceneQueryGeometryType.SHAPE:
            prim_path = kwargs["prim_path"]
            encoded = prim_path.encode("utf-8") if isinstance(prim_path, str) else prim_path
            desc._geom.shape.prim_path = encoded
            desc._keepalive = encoded  # prevent GC before the C call uses the pointer
        else:
            raise ValueError(f"Unknown geometry type: {geometry_type}")
        return desc

    def sweep(
        self,
        geometry_type: SceneQueryGeometryType,
        direction: tuple | list,
        distance: float,
        mode: SceneQueryMode = SceneQueryMode.CLOSEST,
        both_sides: bool = False,
        **kwargs,
    ) -> list[dict]:
        """Sweep a geometry shape along a direction and return hits.

        Args:
            geometry_type: :class:`~ovphysx.SceneQueryGeometryType`.
            direction: Normalized sweep direction [x, y, z].
            distance: Maximum sweep distance (>= 0).
            mode: :class:`~ovphysx.SceneQueryMode`.
            both_sides: If True, test both sides of mesh triangles.
            **kwargs: Geometry parameters:

                - SPHERE: ``radius``, ``position``
                - BOX: ``half_extent``, ``position``, ``rotation`` (xyzw quaternion)
                - SHAPE: ``prim_path`` (USD prim path string)

        Returns:
            List of hit dicts (same format as :meth:`raycast`).
        """
        desc = self._make_geometry_desc(geometry_type, **kwargs)
        d = self._make_float3(direction)
        hits_ptr = ctypes.POINTER(ovphysx_scene_query_hit_t)()
        count = ctypes.c_uint32(0)
        result = _lib.ovphysx_sweep(
            self._omni_physx_sdk_handle.value,
            ctypes.byref(desc),
            d,
            ctypes.c_float(distance),
            ctypes.c_bool(both_sides),
            ctypes.c_int(int(mode)),
            ctypes.byref(hits_ptr),
            ctypes.byref(count))
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(
                f"Sweep failed: {self._get_last_error()}")
        return self._parse_scene_query_hits(hits_ptr, count.value)

    def overlap(
        self,
        geometry_type: SceneQueryGeometryType,
        mode: SceneQueryMode = SceneQueryMode.ALL,
        **kwargs,
    ) -> list[dict]:
        """Test geometry overlap against objects in the scene.

        For overlap queries, location fields (normal, position, distance,
        face_index, material) are zeroed -- only object identity is populated.

        Args:
            geometry_type: :class:`~ovphysx.SceneQueryGeometryType`.
            mode: :class:`~ovphysx.SceneQueryMode` (ANY or ALL; CLOSEST treated as ALL).
            **kwargs: Geometry parameters (same as :meth:`sweep`).

        Returns:
            List of hit dicts (same format as :meth:`raycast`).
        """
        desc = self._make_geometry_desc(geometry_type, **kwargs)
        hits_ptr = ctypes.POINTER(ovphysx_scene_query_hit_t)()
        count = ctypes.c_uint32(0)
        result = _lib.ovphysx_overlap(
            self._omni_physx_sdk_handle.value,
            ctypes.byref(desc),
            ctypes.c_int(int(mode)),
            ctypes.byref(hits_ptr),
            ctypes.byref(count))
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(
                f"Overlap failed: {self._get_last_error()}")
        return self._parse_scene_query_hits(hits_ptr, count.value)

    # ------------------------------------------------------------------
    # Contact bindings
    # ------------------------------------------------------------------

    def create_contact_binding(
        self,
        sensor_patterns: list[str],
        filter_patterns: list[str] | None = None,
        filters_per_sensor: int = 0,
        max_contact_data_count: int = 0,
    ) -> ContactBinding:
        """Create a contact binding for reading aggregate force tensors.

        Returns DLPack-compatible tensors of net forces ``[S, 3]`` or force
        matrices ``[S, F, 3]`` — suitable for RL rewards, safety limits, or
        force monitoring. GPU-compatible. For **per-contact-point geometry**
        (position, normal, impulse), use :meth:`get_contact_report` instead.

        A **sensor** is a set of rigid body prims matched by a USD prim path
        pattern. A **filter** is a second set of bodies whose contacts with each
        sensor you want to measure. No extra USD schema is needed beyond the
        rigid bodies themselves.

        The binding must be created *before* the first simulation step whose
        contacts you want to observe. Call :meth:`read_net_forces` or
        :meth:`read_force_matrix` after :meth:`PhysxSDK.step`. Before the first
        step, both return all-zeros tensors.

        Result tensor shapes after step:
          - net forces:    ``[S, 3]``   where S = matched sensor count
          - force matrix:  ``[S, F, 3]`` where F = matched filter count per sensor

        Example::

            cb = sdk.create_contact_binding(
                sensor_patterns=["/World/robot_0/ee"],
                filter_patterns=["/World/obstacles/box"],
                filters_per_sensor=1,
                max_contact_data_count=256,
            )
            # After sdk.step():
            forces = torch.zeros(cb.sensor_count, 3, device="cuda")
            cb.read_net_forces(forces)

        Args:
            sensor_patterns: USD prim path patterns for sensor bodies.
            filter_patterns: Flat list of USD prim path patterns for filters.
                Total length must equal ``len(sensor_patterns) * filters_per_sensor``.
                Pass ``None`` with ``filters_per_sensor=0`` to get contacts with all bodies.
            filters_per_sensor: Number of filter patterns per sensor (same for all sensors).
            max_contact_data_count: Max raw contact pairs to track (passed to TensorAPI).
        """
        self._check_valid()
        n_sensors = len(sensor_patterns)
        if n_sensors == 0:
            raise ValueError("sensor_patterns must be non-empty")
        if filters_per_sensor < 0:
            raise ValueError("filters_per_sensor must be >= 0")
        c_sensors = (ovphysx_string_t * n_sensors)(*[ovphysx_string_t(p) for p in sensor_patterns])

        if filter_patterns:
            expected = n_sensors * filters_per_sensor
            if filters_per_sensor == 0:
                raise ValueError("filters_per_sensor must be > 0 when filter_patterns is provided")
            if len(filter_patterns) != expected:
                raise ValueError(
                    f"filter_patterns length {len(filter_patterns)} != "
                    f"n_sensors ({n_sensors}) * filters_per_sensor ({filters_per_sensor}) = {expected}"
                )
            n_filters = len(filter_patterns)
            c_filters = (ovphysx_string_t * n_filters)(*[ovphysx_string_t(p) for p in filter_patterns])
            c_filters_ptr = cast(c_filters, POINTER(ovphysx_string_t))
        else:
            c_filters = None
            c_filters_ptr = None
            filters_per_sensor = 0

        out_handle = c_uint64(0)
        result = self._lib.ovphysx_create_contact_binding(
            self._omni_physx_sdk_handle.value,
            cast(c_sensors, POINTER(ovphysx_string_t)),
            c_uint32(n_sensors),
            c_filters_ptr,
            c_uint32(filters_per_sensor),
            c_uint32(max_contact_data_count),
            byref(out_handle),
        )

        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"Failed to create contact binding: {self._get_last_error()}")

        from ctypes import c_int32 as _c_int32

        sensor_count = _c_int32(0)
        filter_count = _c_int32(0)
        spec_result = self._lib.ovphysx_get_contact_binding_spec(
            self._omni_physx_sdk_handle.value, out_handle.value, byref(sensor_count), byref(filter_count)
        )

        if spec_result.status != ApiStatus.SUCCESS:
            self._lib.ovphysx_destroy_contact_binding(self._omni_physx_sdk_handle.value, out_handle.value)
            raise RuntimeError(f"Failed to get contact spec: {self._get_last_error()}")

        return ContactBinding(self, out_handle.value, sensor_count.value, filter_count.value)

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
                # Don't attempt cleanup during interpreter shutdown - the native
                # library may already be unloaded or in an inconsistent state.
                return
        except Exception:
            # During interpreter shutdown, importing sys itself can fail with
            # "import of sys halted; None in sys.modules". In that case, do not
            # attempt cleanup.
            return
        try:
            self.release()
        except Exception:
            pass
