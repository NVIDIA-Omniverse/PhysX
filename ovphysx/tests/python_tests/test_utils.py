# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-OVSTAGE-SCHEMA-001
# @covers AC-4

"""Shared test utilities for PhysX Python tests.

This module provides common helper functions used across multiple test files
to avoid duplication and maintain consistency.

Tests use the TensorBinding API with NumPy arrays directly, e.g.:
    data = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
    binding.write(data)
    binding.read(data)
"""

import ctypes
import math
import os
import sys

import numpy as np
from ovphysx.dlpack import DLDataType, DLDataTypeCode, DLDevice, DLDeviceType, DLTensor
from ovphysx._bindings import OP_INDEX_ALL

_cuda = None


def get_cuda_driver():
    """Lazily load the CUDA driver shared library."""
    global _cuda
    if _cuda is not None:
        return _cuda

    load_error = None
    if sys.platform == "win32":
        for lib_name in ("nvcuda.dll",):
            try:
                loader = getattr(ctypes, "WinDLL", ctypes.CDLL)
                _cuda = loader(lib_name)
                return _cuda
            except OSError as exc:
                load_error = exc
    else:
        for lib_name in ("libcuda.so.1", "libcuda.so"):
            try:
                _cuda = ctypes.CDLL(lib_name)
                return _cuda
            except OSError as exc:
                load_error = exc

    if load_error is not None:
        raise load_error
    raise OSError("No supported CUDA driver library name found for this platform")


NP_TO_DL_DTYPE = {
    np.dtype(np.float32): (DLDataTypeCode.kDLFloat, 32),
    np.dtype(np.float16): (DLDataTypeCode.kDLFloat, 16),
    np.dtype(np.int32): (DLDataTypeCode.kDLInt, 32),
    np.dtype(np.uint8): (DLDataTypeCode.kDLUInt, 8),
}


class CudaArray:
    """GPU array allocated via the CUDA driver API with configurable dtype.

    Exposes a DLTensor (kDLCUDA device=0) accepted by ovphysx read/write.
    Use .numpy() to copy contents back to host for assertions.
    """

    def __init__(self, shape: tuple, dtype=np.float32, device_id: int = 0):
        self._shape = tuple(shape)
        self._device_id = device_id
        self._np_dtype = np.dtype(dtype)
        self._numel = math.prod(self._shape) if self._shape else 0
        self._nbytes = self._numel * self._np_dtype.itemsize

        drv = get_cuda_driver()
        self._ptr = ctypes.c_uint64(0)
        if self._nbytes > 0:
            rc = drv.cuMemAlloc_v2(ctypes.byref(self._ptr), ctypes.c_size_t(self._nbytes))
            if rc != 0:
                raise RuntimeError(f"cuMemAlloc_v2 failed with CUDA error {rc}")
            rc = drv.cuMemsetD8_v2(self._ptr, ctypes.c_ubyte(0), ctypes.c_size_t(self._nbytes))
            if rc != 0:
                raise RuntimeError(f"cuMemsetD8_v2 failed with CUDA error {rc}")

        self._shape_arr = (ctypes.c_int64 * len(self._shape))(*self._shape)

        dl_code, dl_bits = NP_TO_DL_DTYPE[self._np_dtype]
        self._dl = DLTensor()
        self._dl.data = ctypes.c_void_p(self._ptr.value)
        self._dl.device = DLDevice()
        self._dl.device.device_type = DLDeviceType(DLDeviceType.kDLCUDA)
        self._dl.device.device_id = device_id
        self._dl.ndim = len(self._shape)
        self._dl.dtype = DLDataType()
        self._dl.dtype.code = DLDataTypeCode(dl_code)
        self._dl.dtype.bits = dl_bits
        self._dl.dtype.lanes = 1
        self._dl.shape = ctypes.cast(self._shape_arr, ctypes.POINTER(ctypes.c_int64))
        self._dl.strides = None
        self._dl.byte_offset = 0

    @property
    def shape(self):
        return self._shape

    @property
    def dltensor(self):
        return self._dl

    def numpy(self) -> np.ndarray:
        """Copy GPU data to a numpy host array."""
        host = np.empty(self._shape, dtype=self._np_dtype)
        if self._nbytes > 0:
            drv = get_cuda_driver()
            rc = drv.cuMemcpyDtoH_v2(host.ctypes.data_as(ctypes.c_void_p), self._ptr, ctypes.c_size_t(self._nbytes))
            if rc != 0:
                raise RuntimeError(f"cuMemcpyDtoH_v2 failed with CUDA error {rc}")
        return host

    def upload(self, host_array: np.ndarray):
        """Copy numpy host array to GPU."""
        src = np.ascontiguousarray(host_array, dtype=self._np_dtype)
        if self._nbytes > 0:
            drv = get_cuda_driver()
            rc = drv.cuMemcpyHtoD_v2(self._ptr, src.ctypes.data_as(ctypes.c_void_p), ctypes.c_size_t(self._nbytes))
            if rc != 0:
                raise RuntimeError(f"cuMemcpyHtoD_v2 failed with CUDA error {rc}")

    def __del__(self):
        if self._ptr.value and self._nbytes > 0:
            try:
                drv = get_cuda_driver()
                drv.cuMemFree_v2(self._ptr)
            except Exception:
                pass
            self._ptr = ctypes.c_uint64(0)


def data_path(filename):
    """Resolve a test data file path inside the tests/data directory.

    Args:
        filename (str): File name to locate in the test data directory.

    Returns:
        str: Absolute path to the requested data file.

    Example:
        usd_path = data_path("basic_simulation.usda")
        # Returns: /path/to/tests/data/basic_simulation.usda
    """
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(test_dir, "data", filename)


_physx_schemas_registered = False


def register_physx_schemas_with_ovstage():
    """Register ovphysx's codeless PhysX schemas with ovstage, once per process.

    ovphysx ships the schemas as data and never registers them itself: the
    application owns the USD runtime. USD assembles its schema registry once, so
    this must run before the first population call in the process. The tests
    therefore route every population through this helper.
    """
    global _physx_schemas_registered
    if _physx_schemas_registered:
        return

    import ovphysx
    import ovstage

    ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])
    _physx_schemas_registered = True


def attach_usd_with_ovstage(physx, usd_path: str, ordinal: int = 1):
    """Populate an ovstage Stage and attach it at the populated ordinal.

    The returned Stage is owned by the caller, but this helper also keeps it
    alive on the PhysX instance so fixture teardown can detach and destroy it.
    """
    check_valid = getattr(physx, "_check_valid", None)
    if check_valid is not None:
        check_valid()

    if getattr(physx, "_ovphysx_test_ovstages", None):
        destroy_ovstage_test_attachments(physx)

    import ovstage

    if not ovstage.population.available():
        raise RuntimeError("ovstage population bridge is unavailable")

    register_physx_schemas_with_ovstage()

    # The population bridge owns the backing USD stage.
    stage = ovstage.Stage("ovphysx-test-stage")
    attached = False
    try:
        ovstage.population.open_usd(
            stage,
            str(usd_path),
            ordinal=ordinal,
            domains=ovstage.PopulationDomain.PHYSICS,
        )
        stage.advance_write_floor(ordinal=ordinal).wait()
        physx.attach_ovstage(stage, read_ordinal=ordinal)
        attached = True
    except Exception as exc:
        if attached:
            try:
                physx.detach_ovstage()
            except Exception as detach_exc:
                # PhysX still owns a keepalive for stage. Do not destroy the
                # native Stage while a failed detach may still reference it.
                raise RuntimeError(
                    f"Failed to detach ovstage after USD setup failed for '{usd_path}': {exc}"
                ) from detach_exc
        stage.destroy()
        raise RuntimeError(f"Failed to populate ovstage from USD '{usd_path}': {exc}") from exc

    stages = getattr(physx, "_ovphysx_test_ovstages", None)
    if stages is None:
        stages = []
        setattr(physx, "_ovphysx_test_ovstages", stages)
    stages.append(stage)
    return stage


def load_usd_with_ovstage(physx, usd_path: str, path_prefix: str = "", ordinal: int = 1):
    """Test helper that populates ovstage and attaches it to ovphysx.

    Returns an `op_index` for callers that want to wait on the load. The ovstage
    attach performs the initial parse synchronously, so this is `OP_INDEX_ALL`. Waiting on it
    drains all currently pending work without advancing simulation state.
    """
    if path_prefix:
        raise RuntimeError("ovstage population helper does not support path_prefix")
    attach_usd_with_ovstage(physx, usd_path, ordinal=ordinal)
    return OP_INDEX_ALL


def destroy_ovstage_test_attachments(physx) -> None:
    """Detach and destroy ovstage instances created by attach_usd_with_ovstage."""
    if physx is None:
        return
    try:
        physx.detach_ovstage()
    except Exception as exc:
        # Keep both the explicit list and PhysX's Stage keepalive intact. A
        # failed detach may still leave native code referencing the Stage.
        raise RuntimeError("Failed to detach ovstage test attachment") from exc

    stages = getattr(physx, "_ovphysx_test_ovstages", None)
    if not stages:
        return
    while stages:
        stage = stages.pop()
        try:
            stage.destroy()
        except Exception:
            pass
    try:
        setattr(physx, "_ovphysx_test_ovstages", [])
    except Exception:
        pass
    try:
        import gc

        gc.collect()
    except Exception:
        pass


# DEPRECATED (tensor-binding-deprecation): a TensorBinding-only helper used solely by
# test_tensor_bindings.py, removed with the binding. The rest of this module stays.
def verify_tensor_shape(binding, expected_count, expected_components):
    """Verify tensor binding shape matches expectations.

    Args:
        binding: TensorBinding object
        expected_count: Expected number of entities (N)
        expected_components: Expected component count (C) or tuple (L, C) for 3D

    Returns:
        None: Raises AssertionError if shape doesn't match

    Examples:
        # For rigid body poses [N, 7]
        verify_tensor_shape(binding, expected_count=5, expected_components=7)

        # For articulation link poses [N, L, 7]
        verify_tensor_shape(binding, expected_count=2, expected_components=(3, 7))
    """
    if isinstance(expected_components, tuple):
        # 3D tensor [N, L, C]
        expected_shape = (expected_count, *expected_components)
        assert binding.ndim == 3, f"Expected 3D tensor, got ndim={binding.ndim}"
    else:
        # 2D tensor [N, C]
        expected_shape = (expected_count, expected_components)
        assert binding.ndim == 2, f"Expected 2D tensor, got ndim={binding.ndim}"

    assert binding.shape == expected_shape, f"Expected shape {expected_shape}, got {binding.shape}"
    assert binding.count == expected_count, f"Expected count {expected_count}, got {binding.count}"


def read_rigid_body_poses(sdk):
    """Read every rigid body's pose as an ``[N, 7]`` array (xyz + quaternion) via the session API.

    PhysX.read emits one group per scene partition per attribute, so a position column arrives as
    ``[N, 3]`` and an orientation column as ``[N, 4]``, and a multi-scene stage yields several of
    each. PhysX.read does NOT guarantee the position and orientation groups arrive in the same
    partition order, so pair the two columns per partition by the interned ``group.prim_list`` handle
    before joining. Concatenating each attribute independently and ``hstack``-ing would glue a
    body's position to another partition's quaternion on a repartitioned read (and keeping only the
    last group would drop every partition but one). ``tensor.numpy()`` copies a CUDA column back to
    the host, so this works on CPU or GPU. Returns an empty ``[0, 7]`` array when the scene has no
    rigid bodies.
    """
    from ovphysx.types import ObjectScope, SimObjectType

    # prim_list is the interned handle for a partition's prim set. The position and orientation groups
    # of one partition share it, so it pairs them. dict insertion order keeps partitions first-seen.
    by_partition = {}
    with sdk.read(SimObjectType.RIGID_BODY, ["position", "orientation"], scope=ObjectScope.ALL) as result:
        for group in result.groups:
            # Rows are assumed to be in prim order. An index_map would permute them and break the
            # element-wise comparisons callers run on the returned array, so fail loudly instead.
            assert group.index_map is None and group.prim_index_map is None, (
                "read_rigid_body_poses does not handle index_map permutation"
            )
            slot = by_partition.setdefault(group.prim_list, {})
            for tensor in group.tensors:
                column = tensor if isinstance(tensor, np.ndarray) else tensor.numpy()
                if column.shape[1] == 3:
                    slot["position"] = column
                elif column.shape[1] == 4:
                    slot["orientation"] = column
    # hstack within a partition pairs row i of position with row i of orientation (same body, same
    # prim_list). Every partition must yield BOTH columns. A missing one is a read fault, so fail
    # loudly rather than drop the partition. A silent drop would pass a before/after comparison
    # while hiding lost bodies.
    rows = []
    for slot in by_partition.values():
        assert "position" in slot and "orientation" in slot, (
            "read partition is missing its position or orientation column; cannot build a pose"
        )
        rows.append(np.hstack([slot["position"], slot["orientation"]]))
    if not rows:
        return np.zeros((0, 7), dtype=np.float32)
    return np.concatenate(rows).astype(np.float32, copy=False)
