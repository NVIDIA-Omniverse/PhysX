# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""High-level Python API for the ovphysx library.

Stream-Ordered Execution Model
------------------------------
All operations in this API are stream-ordered, meaning they execute in submission
order as if on a single queue. This provides sequential consistency:

- Operations appear to complete in submission order
- Writes from operation N are visible to operation N+1
- You don't need explicit synchronization between dependent operations
- Independent operations may execute concurrently internally for performance

Example (no explicit ovphysx waits needed between dependent operations):

.. code-block:: python
   :caption: Stream-ordered operation sequence

    from ovphysx import TensorType

    def step_and_read(
        physx, stage, output, initial_ordinal, from_ordinal, to_ordinal, dt
    ):
        stage.advance_write_floor(ordinal=initial_ordinal).wait()
        physx.attach_ovstage(stage, read_ordinal=initial_ordinal)
        # After the application authors later ovstage edits:
        stage.advance_write_floor(ordinal=to_ordinal).wait()
        physx.update_from_ovstage(from_ordinal, to_ordinal)
        physx.step(dt)  # Sees the drained stage edits
        with physx.create_tensor_binding(
            "/World/Cube", tensor_type=TensorType.RIGID_BODY_POSE
        ) as binding:
            binding.read(output)  # Reads current state

Use wait_op() when:

- Before accessing results outside the stream (e.g., reading data on CPU/GPU)
- To ensure operations complete before program exit
- For explicit synchronization points in your application

Thread Safety
-------------
- PhysX instances share the underlying omni.physx runtime. Serialize simulation,
  stage mutation, and binding creation across instances.
- A single instance is NOT thread-safe. Use external synchronization if calling
  from multiple threads.
- ctypes releases the GIL during native calls, so concurrent ``step()`` and
  ``TensorBinding.read()`` / ``write()`` from different threads is a data race.
  See the developer guide threading section for the recommended pattern.

"""

import ctypes
import math
import operator
import os
import sys
import threading
import warnings
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
from typing import TYPE_CHECKING, NamedTuple

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
    ovstage_read_group_t,
    ovstage_query_result_t,
    ovstage_ordinal_range_t,
    ovx_string_or_token_t,
    ovx_string_t,
    ovphysx_scene_query_geometry_desc_t,
    ovphysx_scene_query_hit_t,
)
from . import __version__ as _python_version
from .types import (
    ApiStatus,
    LogLevel,
    SimObjectType,
    ObjectScope,
    SceneQueryGeometryType,
    SceneQueryMode,
    TensorType,
)

# Set of tensor types for which articulation metadata (dof_count, body_names, etc.) is valid.
# Derived from the enum so it stays in sync automatically when new ARTICULATION_ types are added.
_ARTICULATION_TENSOR_TYPES: frozenset[int] = frozenset(t for t in TensorType if t.name.startswith("ARTICULATION_"))


class ReadGroup(NamedTuple):
    """One physics-output column group returned by :meth:`PhysX.read`.

    A flattened view of the native ``ovstage_read_group_t`` the read returns (its
    ``prims`` / ``data`` / ``meta`` sub-structs unpacked into these fields). The
    interned identifiers (``attribute`` token and ``prim_list`` handle) resolve
    through the *same* process-shared ovstage path dictionary the attached Stage uses
    — so an ``ovstage.PathDictionary(stage)`` resolves them, and ``prim_list`` feeds
    straight into ``stage.query_from_path_list`` for a no-repack write-back.
    ``object_type`` is NOT part of the native group (the read is opened over one
    type); it is stamped here from the ``object_type`` passed to :meth:`PhysX.read`.

    Constant on the physics-output path (so callers can rely on them): ``is_delete``
    is always ``False`` (this read never emits tombstones); ``prim_offset`` is ``0``
    and ``prim_index_map`` is ``None`` (each group carries its own full ``prim_list``);
    ``ordinal`` is ``0`` (groups are not ordinal-stamped here).

    Attributes:
        attribute: Interned EMITTED attribute token (resolve via the path
            dictionary). May differ from the requested name — a "position" request on
            a point-instancer is emitted as "positions" (instancer-local); write back
            using this token, not the requested string.
        object_type: The queried :class:`SimObjectType` (stamped from the read call,
            not carried by the native ovstage group).
        ordinal: The data ordinal of this group (``0`` on this path).
        is_array: Follows the SOURCE attribute kind — ``True`` for a ragged / USD-array
            / byte-string column (e.g. a point-instancer's positions, a deformable
            mesh's points), ``False`` for a fixed scalar column. It is NOT decided by
            whether the per-element dims happen to be uniform: a fixed-width array
            attribute is still ``is_array=True``. It is the only signal that tells e.g.
            a 1-byte string from a scalar uint8 apart.
        is_delete: Always ``False`` for physics output (tombstone flag).
        semantic: Authored USD interpretation (``ovstage_attribute_semantic_t`` value).
        prim_list: Interned prim-path-list handle covering this group's prims.
        prim_offset: Start index within ``prim_list`` (``0`` on this path).
        prim_count: Number of prims in this group.
        prim_index_map: ``uint32`` NumPy array of sparse indices into ``prim_list``,
            or ``None`` for a contiguous range (always ``None`` on this path).
        index_map: ``uint32`` NumPy array — gather/scatter over the OUTER element axis
            — or ``None``. ``None`` for point-instancer rigid-body output, which always
            emits the instancer's full instance array (by-index) so it forwards into
            the ovstage write path verbatim.
        layout_generation: Reserved metadata field; the current output producer
            always returns ``0``. Do not use it for structural invalidation.
        write_floor_ordinal: Reserved metadata field; the current output producer
            always returns ``0``. The application owns ovstage write-floor advancement.
        tensors: ``list`` of NumPy copies, one per native tensor (1 for a fixed
            column; per-prim for an array group). Tuple width is the trailing dim
            (e.g. a vec3 column is shape ``[N, 3]``). Copied within the borrow
            window so the value is safe to keep.
    """

    attribute: int
    object_type: SimObjectType
    ordinal: int
    is_array: bool
    is_delete: bool
    semantic: int
    prim_list: int
    prim_offset: int
    prim_count: int
    prim_index_map: "object"  # numpy.ndarray | None
    index_map: "object"  # numpy.ndarray | None
    layout_generation: int
    write_floor_ordinal: int
    tensors: "list"  # list[numpy.ndarray]


class ReadResult:
    """Context-managed result of :meth:`PhysX.read` (ADR-0007).

    Holds the query + read session open so each group's interned ``prim_list`` /
    ``attribute`` handles stay valid for the lifetime of the ``with`` block — feed
    them straight back into the ovstage write path (``stage.query_from_path_list(
    group.prim_list)``) for a no-repack write-back. Each group's ``tensors`` are
    NumPy copies, safe to keep past the block. Exiting releases every group, the
    read session, and the query.

    Usage::

        with physx.read(SimObjectType.RIGID_BODY, ["position"]) as result:
            for g in result.groups:
                print(g.prim_list, g.attribute)
    """

    def __init__(self, sdk, query: int, read: int, groups: "list[ReadGroup]", group_ids: "list[int]"):
        self._sdk = sdk
        self._query = int(query)
        self._read = int(read)
        self._group_ids = list(group_ids)
        self.groups = groups
        self._closed = False

    @property
    def dictionary(self) -> int:
        """Opaque process-shared path-dictionary pointer (resolves tokens / prim lists)."""
        return self._sdk.query_shared_dictionary(self._query) if self._query else 0

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        lib = getattr(getattr(self._sdk, "_omni_physx_sdk_handle", None), "value", None)
        if lib is None:
            return  # parent released; native session already gone
        if self._read:
            for gid in self._group_ids:
                self._sdk._lib.ovphysx_release_group(lib, c_uint64(self._read), c_uint64(gid))
            self._sdk._lib.ovphysx_release_read(lib, c_uint64(self._read))
        if self._query:
            self._sdk._lib.ovphysx_release_query(lib, c_uint64(self._query))

    def __enter__(self) -> "ReadResult":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
from .dlpack import (
    DLDataType,
    DLDataTypeCode,
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

_CacheEntry = namedtuple(
    "_CacheEntry",
    [
        "tensor",  # the tensor object (identity check)
        "c_func",  # C function pointer
        "sdk_handle",  # SDK handle integer
        "bind_handle",  # binding handle
        "dl_ptr",  # ctypes pointer to DLTensor
        "dl_tensor",  # DLTensor struct (prevents GC)
        "data_ptr",  # data pointer at cache time (int or None)
        "ptr_getter",  # callable to re-extract data pointer (or None)
    ],
)


def _copy_dl_data_type(dtype: DLDataType | None = None) -> DLDataType:
    """Return a Python-owned copy of a DLPack data type descriptor."""
    if dtype is None:
        return DLDataType(DLDataTypeCode.kDLFloat, 32, 1)
    return DLDataType.from_buffer_copy(dtype)


class TensorBindingSpec(NamedTuple):
    """Python-owned tensor binding metadata returned by :attr:`TensorBinding.spec`."""

    dtype: DLDataType
    ndim: int
    shape: tuple


def _warn_unclosed_resource(resource_name: str, source: object) -> None:
    warnings.warn(
        f"{resource_name} was garbage-collected without explicit destroy(); "
        "use destroy() or a context manager to release native resources promptly.",
        ResourceWarning,
        source=source,
    )


def _detect_data_ptr(tensor):
    """The cache checks ``tensor is cached_tensor`` (Python object identity),
    but a numpy array can be resized in place -- ``buf.resize((bigger,),
    refcheck=False)`` -- which reallocates the underlying memory while
    ``id(buf)`` stays the same.  Without this guard the cache would pass the
    old DLTensor (pointing to freed memory) to the C layer.

    Returns ``(current_ptr, getter_fn)`` where *getter_fn* re-extracts the
    pointer on the fast path, or ``(None, None)`` for providers whose storage
    ownership is not known.
    """
    module_root = type(tensor).__module__.partition(".")[0]
    if module_root == "numpy" and hasattr(tensor, "ctypes"):
        return tensor.ctypes.data, lambda t: t.ctypes.data
    if module_root == "torch" and hasattr(tensor, "data_ptr") and callable(tensor.data_ptr):
        return tensor.data_ptr(), lambda t: t.data_ptr()
    if module_root == "warp" and hasattr(tensor, "ptr"):
        return tensor.ptr, lambda t: t.ptr
    return None, None


def _dltensor_data_ptr(dl_tensor: DLTensor) -> int:
    """Return the effective first-element address of a DLTensor."""
    return int(dl_tensor.data or 0) + int(dl_tensor.byte_offset)


def _make_cache_entry(tensor, c_func, sdk_handle, bind_handle, dl_tensor):
    """Build a cache entry when the tensor has stable, known storage."""
    if isinstance(tensor, DLTensor):
        return _CacheEntry(
            tensor,
            c_func,
            sdk_handle,
            bind_handle,
            ctypes.byref(tensor),
            tensor,
            None,
            None,
        )

    cached_ptr, ptr_getter = _detect_data_ptr(tensor)
    if ptr_getter is None or cached_ptr != _dltensor_data_ptr(dl_tensor):
        return None

    from ._dlpack_utils import copy_dltensor

    cached_dl_tensor = copy_dltensor(dl_tensor)
    return _CacheEntry(
        tensor,
        c_func,
        sdk_handle,
        bind_handle,
        ctypes.byref(cached_dl_tensor),
        cached_dl_tensor,
        cached_ptr,
        ptr_getter,
    )


def _contact_header_to_dict(h: "ContactEventHeader") -> dict:
    """Materialize a ContactEventHeader ctypes struct into a Python-owned dict.

    Used by ``get_contact_report(copy=True)`` so callers can retain contact
    data across simulation steps without zero-copy lifetime hazards.
    """
    return {
        "type": int(h.type),
        "stageId": int(h.stageId),
        "actor0": int(h.actor0),
        "actor1": int(h.actor1),
        "collider0": int(h.collider0),
        "collider1": int(h.collider1),
        "contactDataOffset": int(h.contactDataOffset),
        "numContactData": int(h.numContactData),
        "frictionAnchorsDataOffset": int(h.frictionAnchorsDataOffset),
        "numfrictionAnchorsData": int(h.numfrictionAnchorsData),
        "protoIndex0": int(h.protoIndex0),
        "protoIndex1": int(h.protoIndex1),
    }


def _contact_point_to_dict(p: "ContactPoint") -> dict:
    """Materialize a ContactPoint ctypes struct into a Python-owned dict."""
    return {
        "position": (float(p.position[0]), float(p.position[1]), float(p.position[2])),
        "normal": (float(p.normal[0]), float(p.normal[1]), float(p.normal[2])),
        "impulse": (float(p.impulse[0]), float(p.impulse[1]), float(p.impulse[2])),
        "separation": float(p.separation),
        "faceIndex0": int(p.faceIndex0),
        "faceIndex1": int(p.faceIndex1),
        "material0": int(p.material0),
        "material1": int(p.material1),
    }


def _friction_anchor_to_dict(a: "FrictionAnchor") -> dict:
    """Materialize a FrictionAnchor ctypes struct into a Python-owned dict."""
    return {
        "position": (float(a.position[0]), float(a.position[1]), float(a.position[2])),
        "impulse": (float(a.impulse[0]), float(a.impulse[1]), float(a.impulse[2])),
    }


class TensorBinding:
    """Tensor binding for bulk physics data access via DLPack.

    A tensor binding connects a physics-object path pattern to a tensor type,
    enabling efficient bulk read/write for authored USD objects and runtime-only
    clones (poses, velocities, joint positions, etc.).
    The :attr:`shape`, :attr:`ndim`, and :attr:`dtype` metadata come from
    ``ovphysx_get_tensor_binding_spec()``. Use them to allocate compatible
    buffers instead of assuming every tensor type is ``float32``.

    This is a synchronous API - operations complete before returning.
    Bindings are tied to the currently realized physics objects. Reuse them
    across simulation steps, but do not keep them across reset_stage(), removing USD
    data that contains bound objects, or replacing/reparsing the stage so bound
    objects are destroyed and recreated. Destroy cached bindings before those
    lifecycle operations when practical; if a stale binding survives, only
    destroy it. Create replacement bindings after the operation completes.

    Usage patterns:

    - Context manager (auto-cleanup)::

        import numpy as np
        from ovphysx import TensorType

        def raise_robot_poses(physx):
            with physx.create_tensor_binding(
                "/World/robot*", tensor_type=TensorType.RIGID_BODY_POSE
            ) as binding:
                poses = np.zeros(binding.shape, dtype=np.dtype(str(binding.dtype)))
                binding.read(poses)
                poses[:, 2] += 0.1  # raise z position
                binding.write(poses)
            # Auto-destroyed here

    - Manual (explicit cleanup)::

        import numpy as np
        from ovphysx import TensorType

        def read_robot_poses(physx):
            binding = physx.create_tensor_binding(
                "/World/robot*", tensor_type=TensorType.RIGID_BODY_POSE
            )
            poses = np.zeros(binding.shape, dtype=np.dtype(str(binding.dtype)))
            binding.read(poses)
            binding.destroy()
            return poses
    """

    def __init__(self, sdk, handle: int, tensor_type: int, ndim: int, shape: tuple, dtype: DLDataType | None = None):
        """Initialize tensor binding (created by PhysX.create_tensor_binding)."""
        self._sdk = sdk
        self._handle = handle
        self._tensor_type = tensor_type
        self._ndim = ndim
        self._shape = shape
        self._dtype = _copy_dl_data_type(dtype)
        self._destroyed = False
        self._lock = threading.Lock()
        self._artic_metadata = None
        self._read_cache = None
        self._write_cache = None

    def __repr__(self) -> str:
        state = "destroyed" if self._destroyed else "alive"
        return (
            f"TensorBinding(handle={self._handle}, tensor_type={self._tensor_type}, "
            f"shape={self._shape}, dtype={self.dtype_name}, state={state})"
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
        """Get the number of dimensions reported by ``ovphysx_get_tensor_binding_spec()``."""
        return self._ndim

    @property
    def shape(self) -> tuple:
        """Get tensor shape as tuple reported by ``ovphysx_get_tensor_binding_spec()``.

        Returns:
            Tensor dimensions for this binding. Scalar-property bindings use
            ``(N,)``. Flat state tensors use ``(N, C)``. Articulation and
            deformable mesh tensors use ``(N, L, C)``.
        """
        return self._shape

    @property
    def dtype(self) -> DLDataType:
        """Get the required DLPack dtype for tensors passed to this binding.

        Most bindings use ``float32``. Index bindings (deformable element indices)
        use ``int32``. The bool bindings (DISABLE_SIMULATION, DISABLE_GRAVITY) use
        ``uint8``, as does ARTICULATION_DOF_DRIVE_TYPE, which is a per-DOF enum
        byte rather than a flag.
        See :attr:`dtype_name` for a compact string form.
        """
        return _copy_dl_data_type(self._dtype)

    @property
    def dtype_name(self) -> str:
        """Get the required tensor dtype as a short string such as ``float32``, ``int32``, or ``uint8``."""
        return str(self._dtype)

    @property
    def spec(self) -> TensorBindingSpec:
        """Get a Python-owned tensor spec snapshot for this binding."""
        return TensorBindingSpec(dtype=self.dtype, ndim=self._ndim, shape=self._shape)

    @property
    def count(self) -> int:
        """Get number of entities (first dimension of shape)."""
        return self._shape[0] if self._shape else 0

    def _check_sdk_valid(self) -> None:
        """Ensure the parent PhysX instance is still alive."""
        if self._sdk._omni_physx_sdk_handle is None:
            raise RuntimeError("Cannot use TensorBinding: parent PhysX instance has been released.")

    @property
    def prim_paths(self) -> list[str]:
        """Resolved physics-object paths in tensor row order.

        Rigid-body bindings return one path per rigid-body tensor row.
        Articulation bindings return one root object path per articulation row.
        For per-articulation link names, use :attr:`body_names`.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("TensorBinding has been destroyed")
            self._check_sdk_valid()
            count = self.count
            if count == 0:
                return []
            paths_arr = (ovphysx_string_t * count)()
            out_count = c_uint32(0)
            result = _lib.ovphysx_tensor_binding_get_prim_paths(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                paths_arr,
                count,
                ctypes.byref(out_count),
            )
            if result.status != ApiStatus.SUCCESS:
                error_msg = self._sdk._get_last_error()
                raise RuntimeError(f"Failed to get tensor binding prim paths: {error_msg}")
            if out_count.value > count:
                # Native layer wrote past the buffer we sized to `count` -- this
                # is a real bug in the C library, not an expected truncation.
                # The min() below keeps us from indexing past the buffer.
                import logging as _logging
                _logging.getLogger(__name__).warning(
                    "ovphysx_tensor_binding_get_prim_paths: native out_count=%d > buffer size=%d; truncating",
                    out_count.value, count,
                )
            return [str(paths_arr[i]) for i in range(min(out_count.value, count))]

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

        The tensor must have matching shape and dtype (:attr:`shape` and
        :attr:`dtype`). Can be a NumPy array, PyTorch tensor, or any object
        with __dlpack__ protocol.

        When called repeatedly with the same NumPy, PyTorch, Warp, or direct
        ``DLTensor`` buffer object, an internal cache skips DLPack acquisition
        and attribute chain lookups, giving near-raw-C-call overhead. The numpy
        writeable guard is preserved on the fast path. Other DLPack providers
        are reacquired on every call. Callers that want the fast path should
        reuse the same tensor object with unchanged backing storage across calls.
        Do not resize or rebind storage between cached calls. The staleness
        guard rebuilds the cache when it detects a pointer change, but storage
        mutations that reuse the same pointer violate the cache contract.
        Direct ``DLTensor`` inputs retain their caller-owned descriptor.

        Args:
            tensor: DLPack-compatible tensor with pre-allocated storage matching self.shape.
                   Must use ``self.dtype``. CPU/CUDA device mismatches are staged when
                   CUDA is available; cross-GPU mismatches and CUDA tensors in
                   process-wide CPU-only mode are rejected.

        Preconditions:
            - This binding is not destroyed.
            - tensor has matching shape and dtype and uses a supported device.
        Side effects:
            - Blocks until data is available and writes into the provided tensor.
        Ownership/Lifetime:
            - Caller owns tensor storage and must keep it alive for the duration of the call.
            - Do not mutate the tensor's backing storage (``resize()``, ``set_()``,
              etc.) between cached calls. A staleness guard detects pointer changes
              for NumPy, PyTorch, and Warp inputs and falls back to the slow path.
              Direct ``DLTensor`` inputs retain their caller-owned descriptor.
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

            keepalive = None
            cache_entry = None
            try:
                dl_tensor, keepalive = self._acquire_dltensor(tensor)
                dl_ptr = ctypes.byref(dl_tensor)
                c_func = self._sdk._lib.ovphysx_read_tensor_binding
                sdk_handle_val = self._sdk._omni_physx_sdk_handle.value

                result = c_func(sdk_handle_val, self._handle, dl_ptr)
                if result.status == ApiStatus.SUCCESS:
                    cache_entry = _make_cache_entry(tensor, c_func, sdk_handle_val, self._handle, dl_tensor)
            finally:
                del keepalive

            if result.status != ApiStatus.SUCCESS:
                error_msg = self._sdk._get_last_error()
                raise RuntimeError(f"Failed to read tensor binding: {error_msg}")

            self._read_cache = cache_entry

    def write(self, tensor, indices=None, mask=None) -> None:
        """Write data from a user-provided tensor into the simulation (synchronous).

        The tensor must have matching shape and dtype (:attr:`shape` and
        :attr:`dtype`). Can be a NumPy array, PyTorch tensor, or any object
        with __dlpack__ protocol.

        When called repeatedly with the same supported buffer object and no
        indices/mask, an internal cache skips DLPack acquisition and attribute
        chain lookups, giving near-raw-C-call overhead. Callers that want this
        fast path should reuse the same tensor object with unchanged backing
        storage across calls; see :meth:`read` for the supported providers and
        full contract.

        Args:
            tensor: DLPack-compatible tensor with data to write, shape matching self.shape.
                   Must use ``self.dtype``. CPU/CUDA device mismatches are staged when
                   CUDA is available; cross-GPU mismatches and CUDA tensors in
                   process-wide CPU-only mode are rejected.
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
            - tensor matches shape and dtype and uses a supported device.
            - indices (if provided) is int32 and within bounds.
            - mask (if provided) is bool/uint8 with shape [N] on a supported device.
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
                keepalive = None
                mask_keepalive = None
                try:
                    dl_tensor, keepalive = self._acquire_dltensor(tensor)
                    dl_mask, mask_keepalive = self._acquire_dltensor(mask)

                    result = self._sdk._lib.ovphysx_write_tensor_binding_masked(
                        self._sdk._omni_physx_sdk_handle.value,
                        self._handle,
                        ctypes.byref(dl_tensor),
                        ctypes.byref(dl_mask),
                    )

                    if result.status != ApiStatus.SUCCESS:
                        error_msg = self._sdk._get_last_error()
                        raise RuntimeError(f"Failed to write tensor binding (masked): {error_msg}")
                    return
                finally:
                    del keepalive
                    del mask_keepalive

            keepalive = None
            idx_keepalive = None
            cache_entry = None
            try:
                dl_tensor, keepalive = self._acquire_dltensor(tensor)
                dl_ptr = ctypes.byref(dl_tensor)
                c_func = self._sdk._lib.ovphysx_write_tensor_binding
                sdk_handle_val = self._sdk._omni_physx_sdk_handle.value

                if indices is not None:
                    idx_dl_tensor, idx_keepalive = self._acquire_dltensor(indices)
                    idx_ptr = ctypes.byref(idx_dl_tensor)
                else:
                    idx_dl_tensor = None
                    idx_ptr = None

                result = c_func(sdk_handle_val, self._handle, dl_ptr, idx_ptr)
                if result.status == ApiStatus.SUCCESS and indices is None:
                    cache_entry = _make_cache_entry(tensor, c_func, sdk_handle_val, self._handle, dl_tensor)
            finally:
                del keepalive
                del idx_keepalive

            if result.status != ApiStatus.SUCCESS:
                error_msg = self._sdk._get_last_error()
                raise RuntimeError(f"Failed to write tensor binding: {error_msg}")

            if indices is None:
                self._write_cache = cache_entry

    def _acquire_dltensor(self, obj) -> tuple[DLTensor, object | None]:
        """Extract DLTensor and a keepalive reference (if needed)."""
        from ._dlpack_utils import acquire_dltensor

        return acquire_dltensor(obj)

    def wake_up(self, indices=None) -> None:
        """Wake rigid bodies in this binding.

        Mirrors PhysX SDK ``PxRigidDynamic::wakeUp``. Bodies that still have
        ``RIGID_BODY_DISABLE_SIMULATION`` set are silently skipped (the
        engine refuses to wake disabled actors).

        Typical pair: clear the disable flag on an actor (re-add it to
        simulation in a sleep state) and then call this so the actor is
        active for the next ``step()``.

        Only valid on a rigid-body binding. Articulation bindings raise.

        Args:
            indices: Optional int32 DLPack-compatible tensor of indices into
                this binding. If None, every body in the binding is woken.

        Errors:
            RuntimeError if the binding is destroyed, is not a rigid-body
            binding, has been invalidated by a stage change, or the engine
            wake call fails.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("TensorBinding has been destroyed")
            self._check_sdk_valid()

            idx_ptr = None
            idx_keepalive = None
            if indices is not None:
                idx_dl, idx_keepalive = self._acquire_dltensor(indices)
                idx_ptr = ctypes.byref(idx_dl)

            result = self._sdk._lib.ovphysx_rigid_body_view_wake_up(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                idx_ptr,
            )
            # Keep the indices buffer alive across the native call.
            _ = idx_keepalive
            if result.status != ApiStatus.SUCCESS:
                error_msg = self._sdk._get_last_error()
                raise RuntimeError(f"wake_up failed: {error_msg}")

    def sleep(self, indices=None) -> None:
        """Force rigid bodies in this binding to sleep.

        Mirrors PhysX SDK ``PxRigidDynamic::putToSleep``. Symmetric
        counterpart to :meth:`wake_up`. Bodies that have
        ``RIGID_BODY_DISABLE_SIMULATION`` set are silently skipped.

        Only valid on a rigid-body binding. Articulation bindings raise.

        Args:
            indices: Optional int32 DLPack-compatible tensor of indices into
                this binding. If None, every body in the binding is put to
                sleep.

        Errors:
            RuntimeError if the binding is destroyed, is not a rigid-body
            binding, has been invalidated by a stage change, or the engine
            call fails.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("TensorBinding has been destroyed")
            self._check_sdk_valid()

            idx_ptr = None
            idx_keepalive = None
            if indices is not None:
                idx_dl, idx_keepalive = self._acquire_dltensor(indices)
                idx_ptr = ctypes.byref(idx_dl)

            result = self._sdk._lib.ovphysx_rigid_body_view_sleep(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                idx_ptr,
            )
            _ = idx_keepalive
            if result.status != ApiStatus.SUCCESS:
                error_msg = self._sdk._get_last_error()
                raise RuntimeError(f"sleep failed: {error_msg}")

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
            if getattr(getattr(self, "_sdk", None), "_omni_physx_sdk_handle", None) is not None:
                try:
                    _warn_unclosed_resource("TensorBinding", self)
                except Exception:
                    pass
            try:
                self.destroy()
            except Exception:
                pass


class SdfView:
    """SDF shape view for evaluating signed distance fields at query points.

    Created by PhysX.create_sdf_view(). Use as a context manager or call
    destroy() explicitly when done.

    Evaluation is GPU-only: the query and output tensors must live on the
    same CUDA device as the instance (e.g. torch/cupy tensors).

    SdfView handles are tied to the attached USD stage. After
    :meth:`PhysX.reset_stage` or :meth:`PhysX.detach_ovstage`, existing views
    are invalid; destroy them and create replacements after re-attaching a stage.

    Example::

        with physx.create_sdf_view(pattern="/World/Mesh*", max_query_points=100) as sdf:
            pts = torch.zeros((sdf.count, 100, 3), dtype=torch.float32, device="cuda")
            out = torch.zeros((sdf.count, 100, 4), dtype=torch.float32, device="cuda")
            sdf.evaluate(pts.__dlpack__(), out.__dlpack__())
    """

    def __init__(self, sdk, handle: int, count: int, max_query_points: int):
        """Initialize SDF view (created by PhysX.create_sdf_view)."""
        self._sdk = sdk
        self._handle = handle
        self._count = count
        self._max_query_points = max_query_points
        self._destroyed = False
        self._lock = threading.Lock()

    def __repr__(self) -> str:
        state = "destroyed" if self._destroyed else "alive"
        return f"SdfView(handle={self._handle}, count={self._count}, max_query_points={self._max_query_points}, state={state})"

    @property
    def count(self) -> int:
        """Number of SDF shapes in this view (first dimension of query tensors)."""
        return self._count

    @property
    def max_query_points(self) -> int:
        """Maximum number of query points per shape this view was created with."""
        return self._max_query_points

    def evaluate(self, query_points, out_distances_and_gradients) -> None:
        """Evaluate SDF at query points and write distances + gradients.

        Args:
            query_points: DLPack-compatible tensor with shape [N, Q, 3], float32.
                N must equal count; Q must equal max_query_points.
            out_distances_and_gradients: Pre-allocated DLPack-compatible tensor
                with shape [N, Q, 4], float32. Component layout per point:
                (grad.x, grad.y, grad.z, distance).

        Preconditions:
            - This view is not destroyed.
            - Tensors are float32 on the GPU (kDLCUDA); requires a GPU instance.
        """
        from ._dlpack_utils import acquire_dltensor

        with self._lock:
            if self._destroyed:
                raise RuntimeError("SdfView has been destroyed")

            qp_dl, qp_keepalive = acquire_dltensor(query_points)
            out_dl, out_keepalive = acquire_dltensor(out_distances_and_gradients)
            result = _lib.ovphysx_evaluate_sdf(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                ctypes.byref(qp_dl),
                ctypes.byref(out_dl),
            )
            del qp_keepalive, out_keepalive
            if result.status != 0:
                raise RuntimeError(f"ovphysx_evaluate_sdf failed: {self._sdk._get_last_error()}")

    def destroy(self) -> None:
        """Release the SDF view and its resources.

        Safe to call multiple times, including on stale handles after
        reset_stage() or detach_ovstage() cleanup removed the native view.
        """
        with self._lock:
            if self._destroyed:
                return
            # Guard against the SDK being torn down before an explicit destroy()
            # (GC may collect self._sdk's handle first); mirrors ContactBinding.destroy.
            sdk_handle = self._sdk._omni_physx_sdk_handle
            if sdk_handle is None:
                self._destroyed = True
                return
            result = _lib.ovphysx_destroy_sdf_view(sdk_handle.value, self._handle)
            self._destroyed = True
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"ovphysx_destroy_sdf_view failed: {self._sdk._get_last_error()}")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.destroy()

    def __del__(self):
        try:
            import sys

            if sys.is_finalizing():
                return
        except Exception:
            return
        if not self._destroyed:
            if getattr(getattr(self, "_sdk", None), "_omni_physx_sdk_handle", None) is not None:
                try:
                    _warn_unclosed_resource("SdfView", self)
                except Exception:
                    pass
            try:
                self.destroy()
            except Exception:
                pass


class ContactBinding:
    """Contact tensor binding backed by IRigidContactView.

    Do not instantiate directly. Use :meth:`PhysX.create_contact_binding` to
    obtain an instance. The :attr:`sensor_paths` and :attr:`filter_paths`
    properties expose the row/column metadata for the returned tensors.
    """

    def __init__(self, sdk, handle: int, sensor_count: int, filter_count: int, max_contact_data_count: int):
        self._sdk = sdk
        self._handle = handle
        self._sensor_count = sensor_count
        self._filter_count = filter_count
        self._max_contact_data_count = max_contact_data_count
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

    @property
    def max_contact_data_count(self) -> int:
        """Flat-buffer capacity for raw and detailed contact/friction reads."""
        return self._max_contact_data_count

    @property
    def sensor_paths(self) -> list[str]:
        """Resolved sensor physics-object paths in contact tensor row order.

        Returns:
            list[str]: One path per sensor row, in the same order as the
            contact data tensors.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("ContactBinding has been destroyed")
            self._check_sdk_valid()
            if self._sensor_count <= 0:
                return []
            paths_arr = (ovphysx_string_t * self._sensor_count)()
            out_count = c_uint32(0)
            result = _lib.ovphysx_contact_binding_get_sensor_paths(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                paths_arr,
                self._sensor_count,
                ctypes.byref(out_count),
            )
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"Failed to get contact sensor paths: {self._sdk._get_last_error()}")
            if out_count.value > self._sensor_count:
                # Same defensive log as TensorBinding.prim_paths -- a native
                # out_count exceeding the buffer size is a C-library bug.
                import logging as _logging
                _logging.getLogger(__name__).warning(
                    "ovphysx_contact_binding_get_sensor_paths: native out_count=%d > buffer size=%d; truncating",
                    out_count.value, self._sensor_count,
                )
            return [str(paths_arr[i]) for i in range(min(out_count.value, self._sensor_count))]

    @property
    def filter_paths(self) -> list[list[str]]:
        """Resolved filter physics-object paths in contact tensor column order.

        The outer list is indexed by sensor row and the inner list by filter
        column. Each inner list is empty for unfiltered contact bindings
        (i.e. when ``filter_count == 0``).

        Returns:
            list[list[str]]: Nested list of shape [sensor_count][filter_count].
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("ContactBinding has been destroyed")
            self._check_sdk_valid()
            total = self._sensor_count * self._filter_count
            if total <= 0:
                return [[] for _ in range(max(self._sensor_count, 0))]
            paths_arr = (ovphysx_string_t * total)()
            out_count = c_uint32(0)
            result = _lib.ovphysx_contact_binding_get_filter_paths(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                paths_arr,
                total,
                ctypes.byref(out_count),
            )
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"Failed to get contact filter paths: {self._sdk._get_last_error()}")
            if out_count.value != total:
                raise RuntimeError(f"Expected {total} contact filter paths, got {out_count.value}")
            flat = [str(paths_arr[i]) for i in range(out_count.value)]
            return [
                flat[sensor_idx * self._filter_count:(sensor_idx + 1) * self._filter_count]
                for sensor_idx in range(self._sensor_count)
            ]

    def read_net_forces(self, output) -> None:
        """Read net contact forces into output. Expected shape: [sensor_count, 3].

        The dt for impulse-to-force conversion is taken automatically from the
        last successful :meth:`PhysX.step`, :meth:`PhysX.step_sync`, or
        :meth:`PhysX.step_n_sync` call.
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
        last successful :meth:`PhysX.step`, :meth:`PhysX.step_sync`, or
        :meth:`PhysX.step_n_sync` call.
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

    def read_contact_data(self, contact_forces, positions, normals, separations, counts, start_indices) -> None:
        """Read detailed contact data into flat buffers.

        Expected shapes are ``[C, 1]`` for ``contact_forces`` and ``separations``,
        ``[C, 3]`` for ``positions`` and ``normals``, and ``[sensor_count,
        filter_count]`` for ``counts`` and ``start_indices``. ``C`` is
        :attr:`max_contact_data_count`; both ``C`` and ``filter_count`` must
        be positive. Count and start-index tensors may be int32 or uint32.
        Contact force magnitudes use the timestep from the last successful
        :meth:`PhysX.step`, :meth:`PhysX.step_sync`, or
        :meth:`PhysX.step_n_sync` call.

        Raises:
            RuntimeError: If ``C`` is too small. The payload arrays must not
                be used; ``counts`` and ``start_indices`` contain the full
                required layout. Set ``max_contact_data_count`` to the maximum
                element of ``start_indices + counts`` when recreating the
                binding for subsequent simulation steps. Recreating a binding
                does not recover the overflowing step's payload.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("ContactBinding has been destroyed")
            self._check_sdk_valid()
            if self._max_contact_data_count <= 0:
                raise RuntimeError(
                    "Detailed contact reads require max_contact_data_count > 0 "
                    "when creating the contact binding."
                )
            if self._filter_count <= 0:
                raise RuntimeError(
                    "Detailed contact reads require filters_per_sensor > 0 "
                    "when creating the contact binding."
                )
            from ._dlpack_utils import acquire_dltensor

            force_dl, force_keepalive = acquire_dltensor(contact_forces)
            pos_dl, pos_keepalive = acquire_dltensor(positions)
            normal_dl, normal_keepalive = acquire_dltensor(normals)
            sep_dl, sep_keepalive = acquire_dltensor(separations)
            count_dl, count_keepalive = acquire_dltensor(counts)
            start_dl, start_keepalive = acquire_dltensor(start_indices)

            result = _lib.ovphysx_read_contact_data(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                ctypes.byref(force_dl),
                ctypes.byref(pos_dl),
                ctypes.byref(normal_dl),
                ctypes.byref(sep_dl),
                ctypes.byref(count_dl),
                ctypes.byref(start_dl),
            )
            _ = (force_keepalive, pos_keepalive, normal_keepalive, sep_keepalive, count_keepalive, start_keepalive)
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"Failed to read detailed contact data: {self._sdk._get_last_error()}")

    def read_raw_contact_data(
        self,
        contact_forces,
        positions,
        normals,
        separations,
        counts,
        start_indices,
        other_actor_ids,
    ) -> None:
        """Read raw (unfiltered) contact data into flat buffers.

        Filter-less variant of :meth:`read_contact_data` — returns every
        contact involving each sensor regardless of which other actor it
        collided with, plus a per-contact ``other_actor_ids`` lookup for
        identifying the contacting body via
        :meth:`get_other_actor_paths_from_ids`.

        Expected shapes are ``[C, 1]`` for ``contact_forces`` and
        ``separations``, ``[C, 3]`` for ``positions`` and ``normals``,
        ``[sensor_count]`` for ``counts`` and ``start_indices``, and
        ``[C]`` for ``other_actor_ids``. ``C`` is
        :attr:`max_contact_data_count` and must be positive; no filter
        dimension is required. Count and start-index tensors may be
        int32 or uint32; ``other_actor_ids`` must be int64 or uint64.
        Contact force magnitudes use the timestep from the last successful
        :meth:`PhysX.step`, :meth:`PhysX.step_sync`, or
        :meth:`PhysX.step_n_sync` call.

        Raises:
            RuntimeError: If ``C`` is too small. The payload arrays, including
                ``other_actor_ids``, must not be used; ``counts`` and
                ``start_indices`` contain the full required layout. Set
                ``max_contact_data_count`` to the maximum element of
                ``start_indices + counts`` when recreating the binding for
                subsequent simulation steps. Recreating a binding does not
                recover the overflowing step's payload.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("ContactBinding has been destroyed")
            self._check_sdk_valid()
            if self._max_contact_data_count <= 0:
                raise RuntimeError(
                    "Raw contact reads require max_contact_data_count > 0 "
                    "when creating the contact binding."
                )
            from ._dlpack_utils import acquire_dltensor

            force_dl, force_keepalive = acquire_dltensor(contact_forces)
            pos_dl, pos_keepalive = acquire_dltensor(positions)
            normal_dl, normal_keepalive = acquire_dltensor(normals)
            sep_dl, sep_keepalive = acquire_dltensor(separations)
            count_dl, count_keepalive = acquire_dltensor(counts)
            start_dl, start_keepalive = acquire_dltensor(start_indices)
            ids_dl, ids_keepalive = acquire_dltensor(other_actor_ids)

            result = _lib.ovphysx_read_raw_contact_data(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                ctypes.byref(force_dl),
                ctypes.byref(pos_dl),
                ctypes.byref(normal_dl),
                ctypes.byref(sep_dl),
                ctypes.byref(count_dl),
                ctypes.byref(start_dl),
                ctypes.byref(ids_dl),
            )
            _ = (force_keepalive, pos_keepalive, normal_keepalive, sep_keepalive,
                 count_keepalive, start_keepalive, ids_keepalive)
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"Failed to read raw contact data: {self._sdk._get_last_error()}")

    def get_other_actor_paths_from_ids(self, ids_array) -> list[str]:
        """Resolve actor IDs from :meth:`read_raw_contact_data` to physics-object paths.

        ``ids_array`` is a 1D int64/uint64 array (numpy / warp / torch with
        DLPack support) holding actor IDs. IDs that cannot be resolved
        yield empty strings. Path strings are copied into Python -- the
        caller can keep them across subsequent ovphysx calls.

        Returns:
            list[str]: Physics-object paths in the same order as the input IDs.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("ContactBinding has been destroyed")
            self._check_sdk_valid()
            from ._dlpack_utils import acquire_dltensor

            ids_dl, ids_keepalive = acquire_dltensor(ids_array)
            # The native call writes exactly N paths (N = ids_dl.shape[0]).
            if ids_dl.ndim != 1:
                del ids_keepalive
                raise ValueError(
                    f"ids_array must be a 1D int64/uint64 tensor; got ndim={ids_dl.ndim}"
                )
            n = int(ids_dl.shape[0])
            if n == 0:
                _ = ids_keepalive
                return []

            buf = (ovphysx_string_t * n)()
            count = ctypes.c_uint32(0)
            result = _lib.ovphysx_contact_binding_get_other_actor_paths_from_ids(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                ctypes.byref(ids_dl),
                buf,
                ctypes.c_uint32(n),
                ctypes.byref(count),
            )
            _ = ids_keepalive
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(
                    f"Failed to resolve actor IDs to paths: {self._sdk._get_last_error()}"
                )
            # Clamp to the buffer we provided; the engine should never write
            # more than `n`, but never index past `buf` if it does.
            written = min(int(count.value), n)
            # ovphysx_string_t is not guaranteed NUL-terminated (see header
            # docstring); rely on the struct's __str__, which uses
            # ctypes.string_at(ptr, length) to read exactly `length` bytes.
            # Direct c_char_p slicing would auto-truncate at the first NUL.
            return [str(buf[i]) for i in range(written)]

    def read_friction_data(self, friction_forces, friction_points, counts, start_indices) -> None:
        """Read detailed friction data into flat buffers.

        Expected shapes are ``[C, 3]`` for ``friction_forces`` and
        ``friction_points``, and ``[sensor_count, filter_count]`` for ``counts``
        and ``start_indices``. ``C`` is :attr:`max_contact_data_count` and must
        be positive; ``filter_count`` must also be positive. Count and
        start-index tensors may be int32 or uint32.
        Friction entries are per-anchor; sum each flat slice to build a
        pair-level ``[sensor_count, filter_count, 3]`` force tensor.
        Friction forces use the timestep from the last successful
        :meth:`PhysX.step`, :meth:`PhysX.step_sync`, or
        :meth:`PhysX.step_n_sync` call.

        Raises:
            RuntimeError: If ``C`` is too small. The payload arrays must not
                be used; ``counts`` and ``start_indices`` contain the full
                required layout. Set ``max_contact_data_count`` to the maximum
                element of ``start_indices + counts`` when recreating the
                binding for subsequent simulation steps. Recreating a binding
                does not recover the overflowing step's payload.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("ContactBinding has been destroyed")
            self._check_sdk_valid()
            if self._max_contact_data_count <= 0:
                raise RuntimeError(
                    "Detailed friction reads require max_contact_data_count > 0 "
                    "when creating the contact binding."
                )
            if self._filter_count <= 0:
                raise RuntimeError(
                    "Detailed friction reads require filters_per_sensor > 0 "
                    "when creating the contact binding."
                )
            from ._dlpack_utils import acquire_dltensor

            force_dl, force_keepalive = acquire_dltensor(friction_forces)
            point_dl, point_keepalive = acquire_dltensor(friction_points)
            count_dl, count_keepalive = acquire_dltensor(counts)
            start_dl, start_keepalive = acquire_dltensor(start_indices)

            result = _lib.ovphysx_read_friction_data(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                ctypes.byref(force_dl),
                ctypes.byref(point_dl),
                ctypes.byref(count_dl),
                ctypes.byref(start_dl),
            )
            _ = (force_keepalive, point_keepalive, count_keepalive, start_keepalive)
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"Failed to read detailed friction data: {self._sdk._get_last_error()}")

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
            if getattr(getattr(self, "_sdk", None), "_omni_physx_sdk_handle", None) is not None:
                try:
                    _warn_unclosed_resource("ContactBinding", self)
                except Exception:
                    pass
            try:
                self.destroy()
            except Exception:
                pass

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.destroy()


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


_PROCESS_LIFECYCLE_LOCK = threading.Lock()
_PROCESS_LIFECYCLE_REFCOUNT = 0


def _get_last_error_from_lib() -> str:
    err = _lib.ovphysx_get_last_error()
    if err and err.ptr:
        try:
            return ctypes.string_at(err.ptr, err.length).decode("utf-8", errors="replace") or "Unknown error"
        except Exception:
            return "Unknown error"
    return "Unknown error"


def _acquire_process_lifecycle() -> None:
    global _PROCESS_LIFECYCLE_REFCOUNT
    with _PROCESS_LIFECYCLE_LOCK:
        if _PROCESS_LIFECYCLE_REFCOUNT == 0:
            result = _lib.ovphysx_initialize()
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"ovphysx_initialize() failed: {_get_last_error_from_lib()}")
        _PROCESS_LIFECYCLE_REFCOUNT += 1


def _release_process_lifecycle() -> None:
    global _PROCESS_LIFECYCLE_REFCOUNT
    with _PROCESS_LIFECYCLE_LOCK:
        if _PROCESS_LIFECYCLE_REFCOUNT == 0:
            return
        _PROCESS_LIFECYCLE_REFCOUNT -= 1
        if _PROCESS_LIFECYCLE_REFCOUNT == 0:
            result = _lib.ovphysx_shutdown()
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"ovphysx_shutdown() failed: {_get_last_error_from_lib()}")


class PhysX:
    """High-level wrapper around the C API using ctypes."""

    def __init__(
        self,
        config: "PhysXConfig | None" = None,
        ignore_version_mismatch: bool = False,
        active_cuda_gpus: str | None = None,
    ) -> None:
        """Initialize PhysX SDK.

        Args:
            config: Typed config dataclass. Only non-None fields are applied.
            ignore_version_mismatch: Skip Python/native version match check.
            active_cuda_gpus: Comma-separated CUDA device ordinals
                (default: None = PhysX automatic CUDA selection).
                Restricts which GPU ordinal(s) are used.
                Supported: None/"" (automatic), "0" (GPU 0), "N" (GPU N),
                "-1" (PhysX automatic selection),
                "0,1,...,N-1" (all GPUs round-robin), "1,2,...,N-1" (skip first).
                Any non-empty value overrides config.scene_multi_gpu_mode; a single
                ordinal disables multi-GPU scene distribution.

        To force process-wide CPU-only mode, call PhysX.set_cpu_mode(True) before
        creating any PhysX instances.
        """
        self._lib = _lib
        self._lifecycle_acquired = False
        # Keepalive for an attached ovstage Stage. ovphysx captures the raw
        # ovstage_instance_t* and dereferences it until detach, so the Python
        # wrapper must hold a reference to keep the Stage (and its native
        # instance) alive for the duration of the attachment. Set in
        # attach_ovstage(), cleared in detach_ovstage()/release().
        self._attached_ovstage = None

        if not ignore_version_mismatch:
            _check_version_match()

        args = ovphysx_create_args()

        args.active_cuda_gpus = ovphysx_string_t(active_cuda_gpus or "")

        # Current wheel/source layouts use runtime discovery from lib/ and
        # plugins/, so the Python wrapper leaves bundled_deps_path empty.
        args.bundled_deps_path = ovphysx_string_t()

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
        try:
            _acquire_process_lifecycle()
            self._lifecycle_acquired = True

            result = self._lib.ovphysx_create_instance(byref(args), byref(self._omni_physx_sdk_handle))
            if result.status != ApiStatus.SUCCESS:
                error_msg = self._get_last_error()
                raise RuntimeError(f"ovphysx_create_instance() failed: {error_msg}")
            if self._omni_physx_sdk_handle.value == _INVALID_HANDLE:
                self._omni_physx_sdk_handle = None
                raise RuntimeError("ovphysx_create_instance() returned invalid handle")
        except Exception:
            if self._lifecycle_acquired:
                self._lifecycle_acquired = False
                _release_process_lifecycle()
            raise

        # Track explicit-release state for the ResourceWarning finalizer.
        self._released = False

    def _check_valid(self) -> None:
        if self._omni_physx_sdk_handle is None:
            raise RuntimeError(
                "PhysX instance has been released. Create a new PhysX() instance."
            )

    @property
    def handle(self) -> int:
        """The raw ``ovphysx_handle_t`` for this instance (read-only).

        Use this when passing the handle to C/C++ code that calls the
        ovphysx C API directly.

        Raises:
            RuntimeError: If the instance has been released.
        """
        self._check_valid()
        return self._omni_physx_sdk_handle.value

    @staticmethod
    def set_cpu_mode(cpu_only: bool) -> None:
        """Force process-wide CPU-only mode.

        Call before the first PhysX instance is ever created to guarantee that
        CUDA is never touched. The call requires no active instances. Once set
        to True successfully, the mode cannot be reversed for this process.

        When True: no CUDA driver is touched; all PhysX scenes use CPU dynamics
        regardless of their USD physxScene:enableGPUDynamics settings.

        Raises:
            RuntimeError: If any PhysX instances are currently active, or if
                attempting to set False after True has been applied (CPU-only
                mode is sticky as soon as enabling it succeeds).
        """
        result = _lib.ovphysx_set_cpu_mode(cpu_only)
        if result.status != 0:
            raise RuntimeError(
                f"set_cpu_mode failed ({ApiStatus(result.status).name}): "
                f"{_get_last_error_from_lib()}"
            )

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


    def reset_stage(self) -> int:
        """Reset stage to empty (async).

        Returns:
            op_index (can be used with wait_op() for explicit synchronization)

        Example:
            # Simple usage (stream-ordered)
            physx.reset_stage()
            physx.wait_all()

        Preconditions:
            - Instance must be valid.
        Side effects:
            - Clears the runtime stage.
            - Detaches any attached ovstage Stage (the C runtime calls
              detach_ovstage internally). Callers must re-attach with
              attach_ovstage() before any further update_from_ovstage().
        Ownership/Lifetime:
            - All TensorBinding, ContactBinding, and SdfView objects for the previous
              stage become invalid. Destroy cached bindings and SDF views before reset
              when practical; if a stale handle survives, only destroy it. Create
              replacement bindings and SDF views after the reset completes.
        Threading:
            - Do not call concurrently on the same instance without external sync.
        Errors:
            - Raises RuntimeError on failure.
        """
        self._check_valid()
        result = self._lib.ovphysx_reset_stage(self._omni_physx_sdk_handle.value)

        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to reset_stage: {error_msg}")

        # reset_stage detaches any attached ovstage in the C runtime; drop the
        # Python keepalive to match (see attach_ovstage / detach_ovstage).
        self._attached_ovstage = None
        return result.op_index

    def clone(self, source_path: str, target_paths: list[str],
              parent_transforms: list[tuple[float, float, float, float, float, float, float]] | None = None,
              env_ids: list[int] | None = None) -> int:
        """Clone a prim hierarchy to create multiple runtime physics copies.

        Creates physics-optimized clones in the runtime representation for high-performance
        simulation, backed by the PhysX SDK replicator so cloned articulations are real articulations.
        The source prim must exist in the loaded stage and have physics properties.
        Replication executes inline. The returned operation index is already complete, so
        subsequent operations see the clone immediately and ``wait_op()`` returns immediately.

        This is the clone entrypoint for both standalone callers and callers that
        populate the scene through an ovstage Stage attached via
        :meth:`attach_ovstage`. Replication runs in the internal representation
        only (USD untouched).

        Cross-environment collision filtering can optionally use PhysX environment ids, controlled
        by the ``/ovphysx/clone/useEnvIds`` setting (default: on). When enabled and the scene runs
        GPU dynamics + GPU broadphase, each cloned environment gets a distinct environment id so
        copies in different environments do not collide. The source environment is included: its
        bodies are created holding environment id 0 (assigned as the attach parses them; clones
        get 1..N), so co-located clones (``parent_transforms=None``) are collision-isolated from
        the source as well. Like all carbonite settings it is **per-process** (shared by every
        ovphysx instance in the process), so set it consistently before attaching.

        When one logical environment is assembled from SEVERAL clone calls (e.g. an IsaacLab
        ClonePlan cloning one source row at a time: first ``/env0/Robot`` to every environment,
        then ``/env0/Object``), pass ``env_ids`` so objects that share an environment share an
        environment id -- with ``env_ids=None`` each call numbers its copies afresh, so
        ``/env1/Robot`` and ``/env1/Object`` cloned by different calls would land on different
        ids and never collide with each other::

            env_ids = [0, 1]  # same ids in every call -> same logical environments
            physx.clone("/env0/Robot",  ["/env1/Robot",  "/env2/Robot"],  transforms_r, env_ids)
            physx.clone("/env0/Object", ["/env1/Object", "/env2/Object"], transforms_o, env_ids)

        Args:
            source_path: USD path of the source prim hierarchy to clone (e.g., "/World/env0")
            target_paths: Runtime physics-object paths for the cloned hierarchies
                (e.g., ["/World/env1", "/World/env2"])
            parent_transforms: Optional list of (px, py, pz, qx, qy, qz, qw) transforms
                giving the world pose of each copy's parent.  Position followed by
                quaternion rotation (imaginary-first, matching tensor API convention).
                Identity rotation = (0, 0, 0, 1).  Must have the same length as
                target_paths.  Each cloned body keeps its pose relative to the source's
                parent, so a copy's world pose is transform * inverse(source_parent) *
                source_body -- for a source authored at the origin this places each body
                exactly at the transform.  Pass None to co-locate every copy on the source.
            env_ids: Optional logical environment id per target (list of int, same length
                as target_paths, each 0 <= id < 0x00FFFFFF -- PhysX supports at most
                1<<24 environments and the runtime id is env_ids[i] + 1).  Stable across calls: the
                same id always maps to the same runtime environment, so clones from
                different calls that share an id collide with each other and stay
                isolated from every other environment (engages under GPU dynamics + GPU
                broadphase, like all env-id filtering).  Pass None for automatic
                per-call numbering (each call's copies get fresh ids past every
                previous call's).

        Returns:
            op_index (can be used with wait_op() for explicit synchronization)

        Raises:
            ValueError: If source_path is empty, target_paths is empty, or any target path matches source path
            RuntimeError: If cloning fails, if no ovstage is attached,
                if target_paths contains duplicate or overlapping entries, or if
                clone() is called after the first :meth:`step`, :meth:`step_sync`,
                or :meth:`step_n_sync`. The after-step rejection applies in CPU
                and GPU mode. GPU warmup is also rejected; hard CPU mode treats
                :meth:`warmup_gpu` as a no-op.

        Preconditions:
            - An ovstage source is attached and source_path exists.
            - target_paths are unique, disjoint, and do not already contain physics.
            - No :meth:`step` / :meth:`step_sync` / :meth:`step_n_sync` has run
              since the last :meth:`reset_stage` or :meth:`attach_ovstage` call,
              in either CPU or GPU mode -- violating this raises
              ``RuntimeError``. (Note: :meth:`update_from_ovstage` does *not*
              reset this -- only a fresh :meth:`attach_ovstage` or
              :meth:`reset_stage` does.)
            - :meth:`warmup_gpu` has not been called since the same reset
              points, but this precondition is currently GPU-only -- see above.
        Side effects:
            - Creates live PhysX objects keyed by the target paths. No USD or runtime-stage
              prims are authored.
        Ownership/Lifetime:
            - Clones remain valid until reset_stage().
        Threading:
            - Do not call concurrently on the same instance without external sync.
        Errors:
            - Raises ValueError for invalid inputs.
            - Raises RuntimeError on internal failure, including
              duplicate-target and after-step/after-warmup ordering violations.
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
            # Each entry must be exactly 7 finite numeric values (px,py,pz,qx,qy,qz,qw): the native
            # path reads 7 per target, so a short entry would read past this buffer and a long one
            # would shift every later target's pose. Validate before allocating the ctypes array.
            xform_flat = []
            for i, entry in enumerate(parent_transforms):
                vals = list(entry)
                if len(vals) != 7:
                    raise ValueError(
                        f"parent_transforms[{i}] must have exactly 7 values "
                        f"(px,py,pz,qx,qy,qz,qw), got {len(vals)}"
                    )
                try:
                    fvals = [float(v) for v in vals]
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"parent_transforms[{i}] must contain numeric values") from exc
                if not all(math.isfinite(fv) for fv in fvals):
                    raise ValueError(f"parent_transforms[{i}] values must be finite")
                xform_flat.extend(fvals)
            xform_array = (c_float * len(xform_flat))(*xform_flat)
            xform_ptr = ctypes.cast(xform_array, POINTER(c_float))
        else:
            xform_ptr = None

        # Pack optional logical env ids into a uint32 array [N]. The native path reads N entries
        # and maps each to a runtime environment id (env_ids[i] + 1), so validate length and range
        # before allocating the ctypes array.
        if env_ids is not None:
            if len(env_ids) != num_targets:
                raise ValueError(
                    f"env_ids length ({len(env_ids)}) must match target_paths length ({num_targets})"
                )
            ids = []
            for i, entry in enumerate(env_ids):
                try:
                    iv = operator.index(entry)
                except TypeError as exc:
                    raise ValueError(f"env_ids[{i}] must be an integer") from exc
                if iv < 0 or iv >= 0x00FFFFFF:
                    raise ValueError(f"env_ids[{i}] must be in [0, 0x00FFFFFF), got {iv}")
                ids.append(iv)
            env_id_array = (c_uint32 * num_targets)(*ids)
            env_id_ptr = ctypes.cast(env_id_array, POINTER(c_uint32))
        else:
            env_id_ptr = None

        result = self._lib.ovphysx_clone(
            self._omni_physx_sdk_handle.value, ovphysx_string_t(source_path),
            target_array, c_uint32(num_targets), xform_ptr, env_id_ptr
        )
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to clone: {error_msg}")

        return result.op_index

    def get_object_type(self, prim_path: str) -> int:
        """Classify an authored USD prim by TensorAPI object type.

        Unresolved paths return INVALID. Raises RuntimeError for invalid input
        (empty path, embedded NUL byte) or if no stage is attached.

        Returns:
            ObjectType: One of RIGID_BODY, ARTICULATION, ARTICULATION_LINK,
            ARTICULATION_ROOT_LINK, ARTICULATION_JOINT, or INVALID.
        """
        self._check_valid()
        from .types import ObjectType
        out = c_uint32(0)
        result = _lib.ovphysx_get_object_type(
            self._omni_physx_sdk_handle.value,
            ovphysx_string_t(prim_path),
            ctypes.byref(out),
        )
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"get_object_type failed: {self._get_last_error()}")
        return ObjectType(out.value)

    def step(self, dt: float) -> int:
        """Initiate physics step (async, returns op_index).

        Simulation time is tracked internally; each step advances it by ``dt``.

        Args:
            dt: Delta time for this step [s].

        Returns:
            op_index (can be used with wait_op() for explicit synchronization)

        Examples:
            # Simple usage (stream-ordered)
            physx.step(0.016)
            binding.read(output)  # Automatically waits for step

            # Explicit wait (if accessing results outside stream)
            op = physx.step(0.016)
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
        result = self._lib.ovphysx_step(self._omni_physx_sdk_handle.value, c_float(dt))

        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to step: {error_msg}")

        return result.op_index

    def step_sync(self, dt: float) -> None:
        """Step simulation and wait for completion in a single call.

        Faster than ``step()`` + ``wait_op()`` for performance-critical
        applications like RL training that always wait immediately.
        Simulation time is tracked internally and advanced by ``dt``.

        Args:
            dt: Delta time [s] for this step.

        Raises:
            RuntimeError: If the step or wait fails.
        """
        self._check_valid()
        result = self._lib.ovphysx_step_sync(self._omni_physx_sdk_handle.value, c_float(dt))
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"step_sync failed: {error_msg}")

    def step_n_sync(self, n: int, dt: float) -> None:
        """Run N steps in a single C call, saving (N-1) ctypes round-trips.

        Equivalent to calling ``step_sync(dt)`` n times, but with only one
        Python-to-C transition. Simulation time is tracked internally and
        advanced by ``n * dt``.

        Args:
            n: Number of steps to run (must be >= 1).
            dt: Duration of each step [s].

        Raises:
            RuntimeError: If any step fails.
        """
        self._check_valid()
        result = self._lib.ovphysx_step_n_sync(
            self._omni_physx_sdk_handle.value, c_int32(n), c_float(dt)
        )
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"step_n_sync failed: {error_msg}")

    def update_articulations_kinematic(self) -> None:
        """Update articulation link poses from current joint positions.

        This performs a synchronous articulation forward-kinematics update
        without running a normal simulation step, collision detection, or
        contact generation. Call it after writing articulation DOF positions
        and before reading articulation link pose tensors when fresh link poses
        are needed in the same frame.

        In GPU mode, the first kinematic update after loading USD may perform
        the same automatic DirectGPU warmup step used by tensor reads/writes.

        Raises:
            RuntimeError: If the update fails.
        """
        self._check_valid()
        result = self._lib.ovphysx_update_articulations_kinematic(
            self._omni_physx_sdk_handle.value
        )
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"update_articulations_kinematic failed: {error_msg}")

    def wait_op(self, op_index: int, timeout_ns: int | None = None) -> None:
        """Wait for operation(s) to complete.

        Args:
            op_index: Operation index to wait for, or OP_INDEX_ALL for all ops
            timeout_ns: Timeout in nanoseconds (None = infinite, 0 = poll)

        Raises:
            RuntimeError: If an operation failed or op_index is invalid or already consumed.
            TimeoutError: If timeout expired (e.g., when polling with timeout_ns=0 and the operation is not ready)

        Preconditions:
            - op_index must be valid and not previously consumed.
        Side effects:
            - Consumes each completed or failed operation reached up to op_index.
            - An operation still pending when the wait times out is not consumed.
            - An index completed by internal stream synchronization may be
              acknowledged once; that acknowledgement consumes it.
        Ownership/Lifetime:
            - Wait-result storage is released internally.
            - Error strings are borrowed and remain valid until the next API
              call on the same thread.
        Threading:
            - Serialize calls on one PhysX instance externally.
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
            - Consumes each completed or failed operation reached before
              success or timeout.
        Threading:
            - Serialize all calls on the same instance externally.
        Errors:
            - Raises RuntimeError on failure.
            - Raises TimeoutError if timeout expired (e.g., when polling with timeout_ns=0 and operations are not ready).
        """
        # _check_valid() is called inside wait_op()
        self.wait_op(OP_INDEX_ALL, timeout_ns=timeout_ns)

    def attach_ovstage(self, stage, read_ordinal: int = 1) -> None:
        """Attach an ovstage Stage as the orchestration data surface.

        Attach performs the initial scene parse at ``read_ordinal``. After the
        producer authors later ovstage edits, call :meth:`update_from_ovstage`
        with only those subsequent ordinals. Tensor bindings remain available
        as a perf escape hatch.

        Args:
            stage: An ``ovstage.Stage`` or a raw ``ovstage_instance_t*`` handle.
            read_ordinal: Caller-owned sealed ovstage ordinal the initial scene
                parse reads at. The application owns ordinal advancement; defaults
                to 1 (read from the first sealed ordinal).

        Preconditions:
            - Instance must be valid.
            - Not already attached to a Stage.
            - ``read_ordinal`` is sealed by a completed write-floor advance
              covering it. The population API never opens or commits an ordinal
              of its own, so waiting on ``ovstage.population.open_usd()`` only
              completes population; call
              ``stage.advance_write_floor(ordinal=read_ordinal).wait()`` first.
              Reads target sealed data, so attaching at an unsealed ordinal
              yields a partial parse: rigid bodies may still load while
              articulations and joints can be dropped.

        Lifetime:
            - ``stage`` must outlive the attachment because ovphysx captures and
              dereferences its native pointer until detach. This wrapper holds a
              reference to ``stage`` for the duration of the attachment, so a
              Stage created inline (``attach_ovstage(ovstage.Stage(...))``) stays
              alive; the reference is dropped by :meth:`detach_ovstage` and
              :meth:`release`.

        Errors:
            - Raises ``RuntimeError`` if already attached, ``stage`` is null,
              or the runtime attach fails. Instance remains unattached on
              failure.
        """
        self._check_valid()
        if stage is None:
            raise RuntimeError("attach_ovstage: stage is None")
        if hasattr(stage, "handle"):
            ptr = stage.handle()
        elif hasattr(stage, "_inst"):
            ptr = ctypes.cast(stage._inst, ctypes.c_void_p).value
        else:
            ptr = int(stage)
        if not ptr:
            raise RuntimeError("attach_ovstage: stage handle is null")
        result = self._lib.ovphysx_attach_ovstage(
            self._omni_physx_sdk_handle.value, ctypes.c_void_p(ptr), c_uint64(read_ordinal))
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"Failed to attach ovstage: {self._get_last_error()}")
        # Keep the Stage alive for the duration of the attachment: ovphysx retains
        # its raw native pointer until detach. Without this, a caller that does not
        # retain their own reference would have the Stage GC'd (its __del__ destroys
        # the native instance) → use-after-free on later stage-dependent calls.
        self._attached_ovstage = stage

    def update_from_ovstage(self, from_ordinal: int, to_ordinal: int) -> None:
        """Apply committed ovstage edits over the closed range ``[from_ordinal, to_ordinal]``.

        The application that writes to ovstage owns the ordinal range and calls
        this after sealing the writes. ``population.apply_usd_changes()`` waits
        for population work but does not seal its ordinal; complete
        ``stage.advance_write_floor(ordinal).wait()`` before this call. ovphysx
        forwards the range (as ovstage's own ``ovstage_ordinal_range_t``) to the
        runtime ovstage change feed and applies the resulting deltas to the
        simulation.

        The initial ``read_ordinal`` was already parsed by :meth:`attach_ovstage`.
        Normal incremental updates pass only later ordinals; including the initial
        ordinal replays the initial scene changes rather than only the new delta.
        """
        self._check_valid()
        # The closed range must be ordered. Validate Python-side and raise ValueError
        # (the conventional "bad argument" error for this wrapper surface) before the
        # native call, which would otherwise surface the same precondition as a less
        # specific RuntimeError.
        if int(from_ordinal) > int(to_ordinal):
            raise ValueError(
                f"update_from_ovstage requires from_ordinal <= to_ordinal, got "
                f"from_ordinal={from_ordinal}, to_ordinal={to_ordinal}"
            )
        # Build ovstage's range type. has_start_ordinal=True ⇒ closed [from, to].
        rng = ovstage_ordinal_range_t(
            start_ordinal=int(from_ordinal),
            end_ordinal=int(to_ordinal),
            has_start_ordinal=True,
        )
        result = self._lib.ovphysx_update_from_ovstage(
            self._omni_physx_sdk_handle.value, rng)
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"update_from_ovstage failed: {self._get_last_error()}")

    def detach_ovstage(self) -> None:
        """Detach the currently-attached ovstage Stage.

        Idempotent — calling on an unattached instance is a no-op success.
        Clears registered interests and output-buffer registrations, so a
        subsequent :meth:`attach_ovstage` to a different Stage starts clean.
        After detach, stage-dependent calls such as :meth:`update_from_ovstage`
        and :meth:`step` fail until a Stage is attached again. Detach invalidates
        the stage's tensor, contact, and SDF views. Do not read, write, or
        evaluate existing bindings or SDF views; destroy them and create
        replacements after calling :meth:`attach_ovstage` and realizing a stage
        again.

        Errors:
            - Raises ``RuntimeError`` on internal failures.
        """
        self._check_valid()
        result = self._lib.ovphysx_detach_ovstage(self._omni_physx_sdk_handle.value)
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"Failed to detach ovstage: {self._get_last_error()}")
        # Native no longer references the Stage — drop the keepalive.
        self._attached_ovstage = None

    def read(
        self,
        object_type: SimObjectType,
        attribute_names: "list[str]",
        scope: ObjectScope = ObjectScope.ALL,
    ) -> "ReadResult":
        """Read physics output (ADR-0007) for one simulated type as column groups.

        Mirrors the ovstage read idiom: open a query over ``object_type`` in
        ``scope``, read the named ``attribute_names`` (e.g. ``["position",
        "orientation"]``), and return a context-managed :class:`ReadResult` whose
        ``groups`` is one :class:`ReadGroup` per typed column. The read is
        ovstage-native — attach an ovstage Stage first.

        Use it as a context manager: the query + read session stay open for the
        ``with`` block so each group's interned ``prim_list`` / ``attribute``
        handles are valid — feed them straight into the ovstage write path
        (``stage.query_from_path_list(group.prim_list)``) for a no-repack
        write-back. Group ``tensors`` are NumPy copies, safe to keep past the block.

        This is the *physics → app* direction. To avoid physics consuming its own
        output, write the data back into ovstage at ordinals that are never passed
        to :meth:`update_from_ovstage`. See the ovstage Integration guide for the
        ordinal-coupling principle.

        Args:
            object_type: Simulated type to read (:class:`SimObjectType`).
            attribute_names: Semantic attribute names to read.
            scope: ``ALL`` or ``ACTIVE`` (active is single-frame).

        Returns:
            A :class:`ReadResult` context manager. ``result.groups`` is empty if no
            objects matched.

        Raises:
            RuntimeError: on a native error (e.g. no ovstage attached).
            TypeError: if a returned column carries a DLPack dtype this reader does
                not support, instead of silently decoding it as float32.
        """
        import numpy as np

        self._check_valid()
        handle = self._omni_physx_sdk_handle.value

        query = c_uint64(0)
        result = self._lib.ovphysx_query(
            handle, int(object_type), int(scope), ctypes.byref(query))
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"read query failed: {self._get_last_error()}")
        if query.value == 0:
            # Defensive: ovphysx_query reports a nonzero handle on success (an empty
            # match included) and surfaces 0 as an error above; treat a stray 0 as
            # an empty result rather than dereferencing it.
            return ReadResult(self, 0, 0, [], [])

        # Build the ovx_string_or_token_t array from string names (token = 0).
        names = list(attribute_names)
        name_bytes = [n.encode("utf-8") for n in names]  # keepalive across the call
        arr = (ovx_string_or_token_t * len(names))()
        for i, b in enumerate(name_bytes):
            arr[i].token = 0
            arr[i].string = ovx_string_t(b, len(b))
        read = self._open_read_session(handle, query, arr, len(names))
        return self._iterate_read_groups(handle, query, read, object_type)

    def read_tokens(
        self,
        object_type: SimObjectType,
        attribute_tokens: "list[int]",
        scope: ObjectScope = ObjectScope.ALL,
    ) -> "ReadResult":
        """Token form of :meth:`read`.

        Identical to :meth:`read` but the attributes are given as interned attribute
        tokens (e.g. those from :meth:`fetch_query_result`) instead of strings, so a
        token can be fed straight back in with no token→string→name round-trip.
        Both forms build the same ``ovx_string_or_token_t`` array under the hood.

        Args:
            object_type: Simulated type to read (:class:`SimObjectType`).
            attribute_tokens: Interned attribute tokens to read.
            scope: ``ALL`` or ``ACTIVE`` (active is single-frame).

        Returns:
            A :class:`ReadResult` context manager (see :meth:`read`).

        Raises:
            RuntimeError: on a native error (e.g. no ovstage attached).
            TypeError: if a returned column carries a DLPack dtype this reader does
                not support, instead of silently decoding it as float32.
        """
        self._check_valid()
        handle = self._omni_physx_sdk_handle.value

        query = c_uint64(0)
        result = self._lib.ovphysx_query(
            handle, int(object_type), int(scope), ctypes.byref(query))
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"read query failed: {self._get_last_error()}")
        if query.value == 0:
            return ReadResult(self, 0, 0, [], [])

        toks = [int(t) for t in attribute_tokens]
        arr = (ovx_string_or_token_t * len(toks))()
        for i, t in enumerate(toks):
            arr[i].token = t
            arr[i].string = ovx_string_t(None, 0)
        read = self._open_read_session(handle, query, arr, len(toks))
        return self._iterate_read_groups(handle, query, read, object_type)

    def _open_read_session(self, handle, query, attrs_array, count) -> "object":
        """Open a read session over an ovx_string_or_token_t array (shared by read / read_tokens)."""
        read = c_uint64(0)
        result = self._lib.ovphysx_read(handle, query, attrs_array, count, ctypes.byref(read))
        if result.status != ApiStatus.SUCCESS:
            self._lib.ovphysx_release_query(handle, query)
            raise RuntimeError(f"read session open failed: {self._get_last_error()}")
        return read

    def _iterate_read_groups(self, handle, query, read, object_type) -> "ReadResult":
        """Drain an open read session into a :class:`ReadResult` (shared by read / read_tokens).

        The native group is ovstage's ``ovstage_read_group_t`` (no ovphysx mirror); the
        queried ``object_type`` is stamped on each :class:`ReadGroup` from the caller's
        request, since the native group does not carry it.
        """
        import numpy as np

        groups: list[ReadGroup] = []
        group_ids: list[int] = []
        try:
            while True:
                # Producer-owned group: fetch hands back a borrowed ovstage_read_group_t*
                # (valid until the group/read is released), not a caller-filled struct.
                gp = ctypes.POINTER(ovstage_read_group_t)()
                result = self._lib.ovphysx_fetch_read_next(handle, read, ctypes.byref(gp))
                if result.status == ApiStatus.END_OF_ITERATION:
                    break
                if result.status != ApiStatus.SUCCESS:
                    raise RuntimeError(f"read fetch failed: {self._get_last_error()}")
                if not gp:
                    break
                g = gp.contents

                tensors = [
                    self._dltensor_to_numpy(g.data.tensors[i], np)
                    for i in range(int(g.data.tensor_count))
                ] if g.data.tensors else []
                index_map = self._u32_array(g.data.index_map, int(g.data.count), np)
                prim_index_map = self._u32_array(g.prims.index_map, int(g.prims.count), np)
                groups.append(
                    ReadGroup(
                        attribute=int(g.attribute),
                        object_type=SimObjectType(int(object_type)),
                        ordinal=int(g.ordinal),
                        is_array=bool(g.is_array),
                        is_delete=bool(g.is_delete),
                        semantic=int(g.semantic),
                        prim_list=int(g.prims.list),
                        prim_offset=int(g.prims.offset),
                        prim_count=int(g.prims.count),
                        prim_index_map=prim_index_map,
                        index_map=index_map,
                        layout_generation=int(g.meta.layout_generation),
                        write_floor_ordinal=int(g.meta.attribute_write_floor_ordinal),
                        tensors=tensors,
                    )
                )
                # Keep each group alive (its prim_list / attribute handles stay
                # valid) until the ReadResult is closed.
                group_ids.append(int(g.read_group_id))
        except Exception:
            for gid in group_ids:
                self._lib.ovphysx_release_group(handle, read, c_uint64(gid))
            self._lib.ovphysx_release_read(handle, read)
            self._lib.ovphysx_release_query(handle, query)
            raise

        return ReadResult(self, int(query.value), int(read.value), groups, group_ids)

    def query_shared_dictionary(self, query: int) -> int:
        """Return the opaque pointer to the shared ovstage path dictionary backing a query.

        This is NOT an ovphysx-private dictionary: it is the process-shared ovstage
        dictionary the attached Stage uses, the same one that interned the query's
        tokens / prim lists, so a group's ``attribute`` token / ``prim_list`` handle
        resolve through an ``ovstage.PathDictionary(stage)`` as well. Returns 0 if
        unavailable.
        """
        self._check_valid()
        out = c_void_p(0)
        result = self._lib.ovphysx_query_shared_dictionary(
            self._omni_physx_sdk_handle.value, c_uint64(query), ctypes.byref(out))
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"query_shared_dictionary failed: {self._get_last_error()}")
        return int(out.value or 0)

    # DLPack dtype.code/bits -> (ctypes element type, numpy dtype). The output read
    # produces float32 columns today; ints/uints are mapped for forward safety.
    # (6, 8) is kDLBool (bits=8): a supported encoding the native decoder emits (e.g.
    # OvstagePopulator.setBool writes {kDLBool, 8, 1}); it must decode, not raise.
    _DL_READ_CTYPE = {
        (2, 32): (ctypes.c_float, "float32"),
        (2, 64): (ctypes.c_double, "float64"),
        (0, 64): (ctypes.c_int64, "int64"),
        (0, 32): (ctypes.c_int32, "int32"),
        (0, 8): (ctypes.c_int8, "int8"),
        (1, 64): (ctypes.c_uint64, "uint64"),
        (1, 32): (ctypes.c_uint32, "uint32"),
        (1, 8): (ctypes.c_uint8, "uint8"),
        (6, 8): (ctypes.c_uint8, "bool"),
    }

    @staticmethod
    def _u32_array(ptr, n, np) -> "object":
        """Copy a borrowed uint32 array (index_map / prim_index_map) into NumPy, or None."""
        if not ptr or n <= 0:
            return None
        buf = (ctypes.c_uint32 * n).from_address(ctypes.cast(ptr, ctypes.c_void_p).value)
        return np.ctypeslib.as_array(buf).copy()

    @classmethod
    def _dltensor_to_numpy(cls, t, np) -> "object":
        """Copy a borrowed read-column DLTensor into a NumPy array (lanes preserved).

        ovstage columns carry tuple width in ``dtype.lanes`` of a flat
        ``shape=[N]`` column; a lanes>1 column becomes a trailing NumPy dim
        (e.g. a vec3 column -> ``[N, 3]``). The copy is safe past the borrow window.
        """
        # dtype.code is typed as DLDataTypeCode (a ctypes.c_uint8 *subclass*), so
        # reading the field yields a ctypes instance rather than a plain int; int()
        # on it would parse its raw byte through the buffer protocol (ValueError on
        # b'\x02'). Read .value to get the integer. bits/lanes are plain c_uint8/
        # c_uint16 and already come back as ints, but normalize them the same way.
        def _ival(v):
            return int(getattr(v, "value", v))

        code, bits, lanes = _ival(t.dtype.code), _ival(t.dtype.bits), _ival(t.dtype.lanes) or 1
        mapped = cls._DL_READ_CTYPE.get((code, bits))
        if mapped is None:
            # Never silently reinterpret an unmapped dtype as float32: that returns wrong
            # values (and only half the buffer for 64-bit elements) with no error. Fail
            # loudly so a native dtype-encoding change is caught instead of corrupting data.
            raise TypeError(
                f"ovphysx read column has unsupported DLPack dtype "
                f"(code={code}, bits={bits}, lanes={lanes}); cannot decode to NumPy"
            )
        ctype, npdt = mapped
        ndim = int(t.ndim)
        shape = [int(t.shape[i]) for i in range(ndim)]
        elems = 1
        for d in shape:
            elems *= d
        total = elems * lanes
        out_shape = shape + ([lanes] if lanes > 1 else [])
        if total == 0 or not t.data:
            return np.empty(out_shape, dtype=npdt)
        addr = ctypes.cast(t.data, ctypes.c_void_p).value + int(t.byte_offset)
        buf = (ctype * total).from_address(addr)
        # astype(npdt) rather than copy() so the returned dtype matches the empty path:
        # a kDLBool column (ctype c_uint8) becomes numpy bool, not uint8. copy=True keeps
        # the result safe past the borrowed-buffer window. For non-bool types npdt already
        # equals the ctype's natural dtype, so this is a plain copy.
        return np.ctypeslib.as_array(buf).reshape(out_shape).astype(npdt, copy=True)

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
        # The native instance (which detaches any attached ovstage on destroy) is
        # gone, so drop the Stage keepalive.
        self._attached_ovstage = None
        if getattr(self, "_lifecycle_acquired", False):
            try:
                _release_process_lifecycle()
            except Exception:
                pass
            self._lifecycle_acquired = False
        self._released = True

    # -------------------------------------------------------------------------
    # Tensor Binding API - efficient bulk access to physics simulation data
    # -------------------------------------------------------------------------

    def create_tensor_binding(
        self,
        pattern: str = None,
        prim_paths: list[str] = None,
        tensor_type: int = TensorType.RIGID_BODY_POSE,
        *,
        raise_if_empty: bool = False,
    ) -> TensorBinding:
        """Create tensor binding for bulk physics data access (synchronous).

        A tensor binding connects physics objects (by path pattern or explicit
        paths) to a tensor type, including authored USD objects and runtime-only
        clones.

        :param pattern: Physics-object path glob pattern
            (e.g., "/World/robot*", "/World/env[N]/robot").
            Mutually exclusive with ``prim_paths``.
        :param prim_paths: Explicit list of physics-object paths. Mutually
            exclusive with ``pattern``.
        :param tensor_type: Tensor type enum value (``TensorType.*``).
        :param raise_if_empty: If ``True``, raise ``ValueError`` when the
            binding matches zero physics objects. The default keeps empty bindings valid;
            prefer it for optional or broad queries and check ``binding.count``.
        :returns: TensorBinding object for reading/writing tensor data.
        :raises ValueError: If neither ``pattern`` nor ``prim_paths`` is provided, both are,
            or ``raise_if_empty`` is true and no physics objects match.
        :raises RuntimeError: If binding creation fails.

        Examples::

            import numpy as np
            from ovphysx import TensorType

            def use_tensor_bindings(physx):
                # Optional broad queries can be empty.
                with physx.create_tensor_binding(
                    "/World/robot*", tensor_type=TensorType.RIGID_BODY_POSE
                ) as binding:
                    if binding.count:
                        poses = np.zeros(
                            binding.shape, dtype=np.dtype(str(binding.dtype))
                        )
                        binding.read(poses)

                binding = physx.create_tensor_binding(
                    prim_paths=["/World/env1/robot", "/World/env2/robot"],
                    tensor_type=TensorType.ARTICULATION_DOF_POSITION_TARGET,
                )
                targets = np.zeros(
                    binding.shape, dtype=np.dtype(str(binding.dtype))
                )
                binding.write(targets)
                binding.destroy()

        Preconditions:
            - Exactly one of ``pattern`` or ``prim_paths`` must be provided.
            - A USD stage is loaded.
        Side effects:
            - Allocates native binding resources.
        Ownership/Lifetime:
            - Returned TensorBinding owns native resources until ``destroy()``.
            - Use ``binding.shape`` and ``binding.dtype`` (or ``binding.spec``)
              to allocate compatible buffers; most tensor types are float32,
              but some types such as ``RIGID_BODY_DISABLE_SIMULATION`` are not.
            - The binding is tied to the current stage topology. Reuse it across
              steps, but do not keep it across ``reset_stage()``, removing USD data
              that contains bound objects, or replacing/reparsing the stage so
              bound objects are destroyed and recreated. Destroy cached bindings
              before those lifecycle operations when practical; if a stale
              binding survives, only destroy it. Create replacements after the
              operation completes.
        Diagnostics:
            - Pattern bindings can intentionally match zero physics objects, so expected
              TensorAPI no-match diagnostics are quieted on the simulation view
              used to create that binding.
            - Explicit ``prim_paths`` keep the default error-level no-match
              diagnostics for typo detection. To detect partial misses
              programmatically, compare the requested ``prim_paths`` with the
              resolved ``binding.prim_paths`` returned after creation.
        Threading:
            - Do not create bindings concurrently with stage mutation.
        Errors:
            - Raises ``ValueError`` for invalid arguments.
            - Raises ``RuntimeError`` on creation failure.
        """
        self._check_valid()
        if pattern is None and prim_paths is None:
            raise ValueError("Either 'pattern' or 'prim_paths' must be provided")
        if pattern is not None and prim_paths is not None:
            raise ValueError("Cannot specify both 'pattern' and 'prim_paths'")
        if prim_paths is not None and len(prim_paths) == 0:
            raise ValueError("prim_paths must not be empty; pass at least one prim path.")

        desc = ovphysx_tensor_binding_desc_t()
        desc.tensor_type = tensor_type

        # Keep references to prevent garbage collection during C call
        c_paths_array = None
        c_paths_refs = []

        if prim_paths is not None:
            desc.pattern = ovphysx_string_t()  # Empty pattern
            desc.prim_paths_count = len(prim_paths)
            c_paths_array = (ovphysx_string_t * len(prim_paths))()
            for i, path in enumerate(prim_paths):
                c_paths_array[i] = ovphysx_string_t(path)
                c_paths_refs.append(c_paths_array[i])
            desc.prim_paths = cast(c_paths_array, POINTER(ovphysx_string_t))
        else:
            desc.pattern = ovphysx_string_t(pattern)
            desc.prim_paths = None
            desc.prim_paths_count = 0

        handle = c_uint64(0)
        result = self._lib.ovphysx_create_tensor_binding(self._omni_physx_sdk_handle.value, byref(desc), byref(handle))

        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to create tensor binding: {error_msg}")

        spec = ovphysx_tensor_spec_t()
        spec_result = self._lib.ovphysx_get_tensor_binding_spec(
            self._omni_physx_sdk_handle.value, handle.value, byref(spec)
        )

        if spec_result.status != ApiStatus.SUCCESS:
            self._lib.ovphysx_destroy_tensor_binding(self._omni_physx_sdk_handle.value, handle.value)
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to get tensor binding spec: {error_msg}")

        ndim = spec.ndim
        shape = tuple(spec.shape[i] for i in range(ndim))

        if raise_if_empty and (not shape or shape[0] == 0):
            cleanup_result = self._lib.ovphysx_destroy_tensor_binding(self._omni_physx_sdk_handle.value, handle.value)
            cleanup_error = None
            if cleanup_result.status != ApiStatus.SUCCESS:
                cleanup_error = self._get_last_error() or "unknown error"
            if prim_paths is not None:
                source = f"prim_paths ({len(prim_paths)} entries)"
            else:
                source = f"pattern {pattern!r}"
            if cleanup_error is not None:
                warnings.warn(
                    f"Failed to destroy empty tensor binding after {source} matched 0 prims: {cleanup_error}",
                    RuntimeWarning,
                )
            raise ValueError(
                f"create_tensor_binding matched 0 prims for {source}. "
                "Check the path input and ensure USDs are loaded before creating the binding. "
                "For optional readback or broad scene queries, leave raise_if_empty=False "
                "and check binding.count before reading."
            )

        return TensorBinding(self, handle.value, tensor_type, ndim, shape, spec.dtype)

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
    # Contact report
    # ------------------------------------------------------------------

    def get_contact_report(self, include_friction_anchors: bool = False, copy: bool = False) -> dict:
        """Get per-contact-point event data for the current simulation step.

        Use this for custom contact sensors, collision debugging, or per-point
        force analysis. For **aggregate force tensors** (net forces or force
        matrices between sensor/filter body sets), use
        :meth:`create_contact_binding` instead.

        .. warning::
            With the default ``copy=False``, the returned ``headers``,
            ``points``, and ``anchors`` are zero-copy ctypes views into
            internal C buffers that are valid **only until the next**
            :meth:`step` or :meth:`step_sync` call. After the next step the
            buffers may be reallocated or reused; accessing the views is
            undefined behavior (silent data corruption or segfault). Python
            cannot detect this dangling state.

            Pass ``copy=True`` to get Python-owned lists of dicts that are
            safe to retain across simulation steps. This is the recommended
            mode for RL training loops or any code that holds contact data
            beyond a single step.

        Args:
            include_friction_anchors: If True, also return friction anchor data
                (position and impulse at each friction anchor point).
            copy: If True, return Python-owned ``list[dict]`` for each section
                (safe to hold across steps). If False (default), return
                zero-copy ctypes array views (faster but valid only until the
                next ``step()``/``step_sync()``).

        Returns a dict with:
            - ``headers``: contact event headers describing each contact pair
              (actors, colliders, event type). When ``copy=False``, a ctypes
              array of :class:`ContactEventHeader`; when ``copy=True``, a
              ``list[dict]`` with the same field names. Length is
              ``num_headers``.
            - ``num_headers`` (int): Number of contact event headers.
            - ``points``: per-contact-point data (position, normal, impulse,
              separation). When ``copy=False``, a ctypes array of
              :class:`ContactPoint`; when ``copy=True``, a ``list[dict]``.
              Length is ``num_points``.
            - ``num_points`` (int): Number of contact point entries.
            - ``anchors`` (only if ``include_friction_anchors=True``): friction
              anchor data. When ``copy=False``, a ctypes array of
              :class:`FrictionAnchor`; when ``copy=True``, a ``list[dict]``.
              Length is ``num_anchors``.
            - ``num_anchors`` (int, only if ``include_friction_anchors=True``):
              Number of friction anchors.

        Example (safe across steps, ``copy=True``)::

            report = physx.get_contact_report(copy=True)
            physx.step_sync(dt)  # next step — report still valid
            for h in report["headers"]:
                print(h["actor0"], h["numContactData"])
            for p in report["points"]:
                print(p["position"], p["normal"], p["impulse"])

        Example (zero-copy, ``copy=False``)::

            report = physx.get_contact_report()
            for i in range(report["num_headers"]):
                h = report["headers"][i]
                print(h.actor0, h.numContactData)
            # Do NOT call step() before finishing access to report.

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
        if copy:
            headers = [_contact_header_to_dict(headers_ptr[i]) for i in range(nh)]
            points = [_contact_point_to_dict(data_ptr[i]) for i in range(np_)]
        else:
            headers = (
                ctypes.cast(headers_ptr, ctypes.POINTER(ContactEventHeader * nh)).contents
                if nh
                else (ContactEventHeader * 0)()
            )
            points = (
                ctypes.cast(data_ptr, ctypes.POINTER(ContactPoint * np_)).contents
                if np_
                else (ContactPoint * 0)()
            )
        out = {
            "headers": headers,
            "num_headers": nh,
            "points": points,
            "num_points": np_,
        }
        if include_friction_anchors:
            na = num_anchors.value
            if copy:
                anchors = [_friction_anchor_to_dict(anchors_ptr[i]) for i in range(na)]
            else:
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
            if isinstance(prim_path, (bytes, bytearray)):
                prim_path = bytes(prim_path).decode("utf-8")
            # ovphysx_string_t is length-prefixed so embedded NULs reach the C
            # API, which rejects them instead of silently truncating the path.
            path_str = ovphysx_string_t(prim_path)
            desc._geom.shape.prim_path = path_str
            desc._keepalive = path_str  # keeps path_str._bytes buffer alive
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
            mode: :class:`~ovphysx.SceneQueryMode` (ANY or ALL).
            **kwargs: Geometry parameters (same as :meth:`sweep`).

        Returns:
            List of hit dicts (same format as :meth:`raycast`).

        Raises:
            ValueError: If ``mode`` is :attr:`~ovphysx.SceneQueryMode.CLOSEST`.
        """
        if mode == SceneQueryMode.CLOSEST:
            raise ValueError(
                "overlap() does not support SceneQueryMode.CLOSEST -- "
                "CLOSEST has no meaning for overlap queries (no direction/ray). "
                "Use SceneQueryMode.ALL or SceneQueryMode.ANY.")
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
        """Create a contact binding for reading aggregate and detailed contact tensors.

        Returns DLPack-compatible tensors of net forces ``[S, 3]`` or force
        matrices ``[S, F, 3]``. Detailed contact and friction data are exposed
        as flat ``[C, ...]`` buffers plus ``[S, F]`` count/start-index tensors
        via :meth:`ContactBinding.read_contact_data` and
        :meth:`ContactBinding.read_friction_data`.

        A **sensor** is a set of rigid bodies matched by a physics-object path
        pattern. A **filter** is a second set of bodies whose contacts with each
        sensor you want to measure. Patterns include authored USD objects and
        runtime-only clones.

        Contact reporting is opt-in: every authored USD prim matched by
        ``sensor_patterns`` must have ``PhysxContactReportAPI`` applied, on the
        prim named as the sensor itself (not a parent body or child collider). A
        matched prim without the schema is dropped from the binding, and if that
        leaves no sensors this call raises ``RuntimeError``. Filter prims need no
        extra schema, and runtime-only clones inherit contact reporting from the
        source actor.

        The binding must be created *before* the first simulation step whose
        contacts you want to observe. Call
        :meth:`ContactBinding.read_net_forces` or
        :meth:`ContactBinding.read_force_matrix` after a successful
        :meth:`PhysX.step`, :meth:`PhysX.step_sync`, or
        :meth:`PhysX.step_n_sync` call. Before the first step, both return
        all-zeros tensors.

        Result tensor shapes after step:
          - net forces:    ``[S, 3]``   where S = matched sensor count
          - force matrix:  ``[S, F, 3]`` where F = matched filter count per sensor
          - detailed data: flat ``[C, 1]`` or ``[C, 3]`` buffers indexed by
            ``counts`` and ``start_indices`` with shape ``[S, F]``

        Use :attr:`ContactBinding.sensor_paths` and
        :attr:`ContactBinding.filter_paths` to map rows and columns back to
        resolved physics-object paths.

        Example::

            import torch

            def read_contact_forces(physx):
                with physx.create_contact_binding(
                    sensor_patterns=["/World/robot_0/ee"],
                    filter_patterns=["/World/obstacles/box"],
                    filters_per_sensor=1,
                    max_contact_data_count=256,
                ) as binding:
                    # Call this after a successful simulation step.
                    forces = torch.zeros(
                        (binding.sensor_count, 3), device="cuda"
                    )
                    binding.read_net_forces(forces)
                    return forces

        Args:
            sensor_patterns: Physics-object path patterns for sensor bodies.
            filter_patterns: Flat list of physics-object path patterns for filters.
                Total length must equal ``len(sensor_patterns) * filters_per_sensor``.
                Pass ``None`` with ``filters_per_sensor=0`` to get contacts with all bodies.
            filters_per_sensor: Number of filter patterns per sensor (same for all sensors).
            max_contact_data_count: Maximum contact-data entries that the raw
                and detailed contact/friction flat-buffer reads can hold.
                Detailed reads require this value and ``filters_per_sensor`` to
                be positive. If it is too small, raw and detailed reads raise
                ``RuntimeError`` and report the required layout through their
                count and start-index arrays.
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

        capacity = c_uint32(0)
        capacity_result = self._lib.ovphysx_get_contact_binding_capacity(
            self._omni_physx_sdk_handle.value, out_handle.value, byref(capacity)
        )
        if capacity_result.status != ApiStatus.SUCCESS:
            self._lib.ovphysx_destroy_contact_binding(self._omni_physx_sdk_handle.value, out_handle.value)
            raise RuntimeError(f"Failed to get contact capacity: {self._get_last_error()}")

        return ContactBinding(self, out_handle.value, sensor_count.value, filter_count.value, capacity.value)

    def create_sdf_view(self, pattern: str, max_query_points: int) -> SdfView:
        """Create an SDF shape view for evaluating signed distance fields.

        Requires a GPU instance; CPU SDF evaluation is not yet implemented.

        Args:
            pattern: USD-style object-path glob matching SDF collision shapes,
                including runtime-only clones.
            max_query_points: Number of query points per shape per call. Query
                tensors passed to ``SdfView.evaluate`` must have Q equal to this.

        Returns:
            SdfView with count == number of matched shapes.

        Example::

            sdf = physx.create_sdf_view("/World/Mesh*", max_query_points=64)
            # Query/output tensors must be on the CUDA device (SDF eval is GPU-only).
            pts = torch.zeros((sdf.count, 64, 3), dtype=torch.float32, device="cuda")
            out = torch.zeros((sdf.count, 64, 4), dtype=torch.float32, device="cuda")
            sdf.evaluate(pts, out)
            sdf.destroy()
        """
        handle = self._omni_physx_sdk_handle.value
        pat = ovphysx_string_t(pattern)
        out_handle = c_uint64(0)
        result = _lib.ovphysx_create_sdf_view(handle, pat, c_uint32(max_query_points), byref(out_handle))
        if result.status != 0:
            raise RuntimeError(f"create_sdf_view failed: {self._get_last_error()}")
        count_out = c_uint32(0)
        count_result = _lib.ovphysx_sdf_view_get_count(handle, out_handle.value, byref(count_out))
        if count_result.status != 0:
            _lib.ovphysx_destroy_sdf_view(handle, out_handle.value)
            raise RuntimeError(f"create_sdf_view failed querying shape count: {self._get_last_error()}")
        return SdfView(self, out_handle.value, count_out.value, max_query_points)

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

        Emits a :class:`ResourceWarning` when the instance is garbage-collected
        without an explicit :meth:`release` or context-manager exit. Mirrors
        Python file-object semantics. The warning is silent by default
        (filtered out unless ``python -W default::ResourceWarning`` or a test
        suite captures it), so existing code is not surprised — but a missing
        release is surfaced to anyone looking for resource hygiene issues.

        Note: During interpreter shutdown, calling C functions may fail.
        We check sys.is_finalizing() to avoid spurious errors.
        """
        try:
            import sys as _sys  # local re-bind: module-level sys may be None at shutdown

            if _sys.is_finalizing():
                # Don't attempt cleanup during interpreter shutdown - the native
                # library may already be unloaded or in an inconsistent state.
                return
        except Exception:
            # During interpreter shutdown, importing sys itself can fail with
            # "import of sys halted; None in sys.modules". In that case, do not
            # attempt cleanup.
            return
        if not getattr(self, "_released", True):
            try:
                import warnings as _warnings  # local re-bind for shutdown safety

                _warnings.warn(
                    "PhysX instance garbage-collected without explicit "
                    "release(). Use `with PhysX() as physx:` or call "
                    "physx.release() to ensure deterministic cleanup. "
                    "Implicit cleanup at interpreter shutdown is "
                    "non-deterministic and may crash or hang if the "
                    "Carbonite plugin runtime is torn down first.",
                    ResourceWarning,
                    stacklevel=2,
                )
            except Exception:
                pass
        try:
            self.release()
        except Exception:
            pass
