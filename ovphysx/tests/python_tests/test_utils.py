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

# @implements REQ-CAPI-OVSTAGE-XFORM-001
# @covers AC-1

import ctypes
import math
import os
import sys
from contextlib import contextmanager

import numpy as np
import pytest
from ovphysx.dlpack import DLDataType, DLDataTypeCode, DLDevice, DLDeviceType, DLTensor

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
            # Device memset can return before completion. Physics may use a
            # different CUDA context, whose default stream does not order this
            # initialization before its writes to the same allocation.
            rc = drv.cuCtxSynchronize()
            if rc != 0:
                raise RuntimeError(f"CUDA buffer initialization failed with CUDA error {rc}")

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
            # Pageable host copies can also return before the device copy ends.
            rc = drv.cuCtxSynchronize()
            if rc != 0:
                raise RuntimeError(f"CUDA buffer upload failed with CUDA error {rc}")

    def __del__(self):
        if self._ptr.value and self._nbytes > 0:
            try:
                drv = get_cuda_driver()
                drv.cuMemFree_v2(self._ptr)
            except Exception:
                pass
            self._ptr = ctypes.c_uint64(0)


@contextmanager
def contact_test_case(**parameters):
    """Include the contact method or capacity in failures from a combined test."""
    try:
        yield
    except (Exception, pytest.fail.Exception) as exc:
        raise AssertionError(f"Contact test case {parameters}: {exc}") from exc


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
    """Register external schemas before the first population in the process.

    All test population uses this helper because USD builds its registry once.
    """
    global _physx_schemas_registered
    if _physx_schemas_registered:
        return

    import ovphysx
    from schema_support import physx_schema_paths
    import ovstage

    # Register Newton too, so population retains authored newton:* attributes.
    ovstage.population.register_usd_schemas(
        [*[str(p) for p in physx_schema_paths()], str(ovphysx.newton_schema_root())]
    )
    _physx_schemas_registered = True


def attach_usd_with_ovstage(physx, usd_path: str, ordinal: int = 1, domains=None):
    """Populate an ovstage Stage and attach it at the populated ordinal.

    ALL is the default so population resolves world transforms. A test may
    explicitly select another domain to exercise its population boundary.

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
    try:
        ovstage.population.open_usd(
            stage,
            str(usd_path),
            ordinal=ordinal,
            domains=ovstage.PopulationDomain.ALL if domains is None else domains,
        )
        stage.advance_write_floor(ordinal=ordinal).wait()
        physx.wait_task(physx.attach_ovstage(stage, read_ordinal=ordinal))
    except Exception as exc:
        if physx.is_stage_attached(stage):
            try:
                physx.wait_task(physx.detach_ovstage())
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


def load_usd_with_ovstage(physx, usd_path: str, path_prefix: str = "", ordinal: int = 1, domains=None):
    """Test helper that populates ovstage and attaches it to ovphysx.

    This helper collects the attach task and returns None. Call
    ``physx.wait_all()`` when a test also needs to collect outstanding work.
    """
    if path_prefix:
        raise RuntimeError("ovstage population helper does not support path_prefix")
    attach_usd_with_ovstage(physx, usd_path, ordinal=ordinal, domains=domains)


# The contact sensor's two net contact-force components (OVPHYSX_ATTR_NET_*_CONTACT_FORCE).
NET_NORMAL_CONTACT_FORCE = "netNormalContactForce"
NET_FRICTION_CONTACT_FORCE = "netFrictionContactForce"
NORMAL_CONTACT_FORCE_MATRIX = "normalContactForceMatrix"
FRICTION_CONTACT_FORCE_MATRIX = "frictionContactForceMatrix"
FILTERED_CONTACT_COUNT = "filteredContactCount"
FILTERED_CONTACT_FORCE = "filteredContactForce"
FILTERED_CONTACT_NORMAL = "filteredContactNormal"
FILTERED_CONTACT_POINT = "filteredContactPoint"
RAW_CONTACT_FORCE = "rawContactForce"
RAW_CONTACT_NORMAL = "rawContactNormal"
RAW_CONTACT_OTHER_BODY = "rawContactOtherBody"
CONTACT_EVENT_TYPE = "contactEventType"
CONTACT_EVENT_OTHER_BODY = "contactEventOtherBody"
CONTACT_EVENT_POINT_COUNT = "contactEventPointCount"


def write_contact_sensor_usda(tmp_path) -> str:
    """Write a scene of contact-reporting boxes, with known weights, resting on a static ground.

    Z-up, meters, gravity 9.81 down; the ground's top face is at z = 0. Three bodies carry
    PhysxContactReportAPI: /World/Light (1 kg) and /World/Heavy (2 kg) rest on the ground, and
    /World/Float is kinematic and hovers, touching nothing. /World/Quiet rests on the ground
    without the API. The resting sensors have sleep disabled: a settled body sleeps and stops
    generating contact reports, which would read as a zero force.
    """
    def box(name, x, z, mass, report=True, kinematic=False):
        schemas = ["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI"]
        if report:
            schemas.append("PhysxContactReportAPI")
        quoted = ", ".join(f'"{s}"' for s in schemas)
        lines = [
            f'    def Cube "{name}" ( prepend apiSchemas = [{quoted}] )',
            "    {",
            "        double size = 1",
            f"        float physics:mass = {mass}",
            f"        double3 xformOp:translate = ({x}, 0, {z})",
            '        uniform token[] xformOpOrder = ["xformOp:translate"]',
        ]
        if report:
            lines += ["        float physxContactReport:threshold = 0"]
        if kinematic:
            lines += ["        bool physics:kinematicEnabled = 1"]
        else:
            lines += ["        float physxRigidBody:sleepThreshold = 0"]
        return lines + ["    }"]

    p = tmp_path / "contact_sensors.usda"
    p.write_text("\n".join(
        ['#usda 1.0', '(', '    defaultPrim = "World"', '    metersPerUnit = 1', '    upAxis = "Z"', ')', '',
         'def Xform "World"', '{',
         '    def PhysicsScene "physicsScene"', '    {',
         '        vector3f physics:gravityDirection = (0, 0, -1)', '        float physics:gravityMagnitude = 9.81',
         '    }',
         '    def Cube "Ground" ( prepend apiSchemas = ["PhysicsCollisionAPI"] )', '    {',
         '        double size = 1',
         '        double3 xformOp:translate = (0, 0, -0.5)', '        float3 xformOp:scale = (40, 40, 1)',
         '        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]',
         '    }']
        + box("Light", 0, 0.5, 1)
        + box("Heavy", 3, 0.5, 2)
        + box("Float", 6, 5, 1, kinematic=True)
        + box("Quiet", 9, 0.5, 1, report=False)
        + ['}', '']))
    return str(p)


def assert_contact_sensor_forces(forces):
    """Check a ``read_net_contact_forces`` result against the write_contact_sensor_usda scene.

    Discovery selects on PhysxContactReportAPI (the hovering sensor is a row, the schemaless box
    is not), each resting sensor carries its own weight as its normal force on its own row, and
    the hovering one reads zero. The two resting sensors weigh differently, so a row assigned to
    the wrong prim fails. Nothing pushes a resting sensor sideways, so its friction is near zero.
    """
    normal, friction = forces[NET_NORMAL_CONTACT_FORCE], forces[NET_FRICTION_CONTACT_FORCE]
    sensors = {"/World/Light", "/World/Heavy", "/World/Float"}
    assert set(normal) == sensors and set(friction) == sensors
    for path, mass in (("/World/Light", 1.0), ("/World/Heavy", 2.0)):
        fx, fy, fz = normal[path]
        weight = mass * 9.81
        assert abs(fz - weight) < 0.05 * weight, f"{path}: normal force z {fz}, expected its weight {weight}"
        assert abs(fx) < 0.05 * weight and abs(fy) < 0.05 * weight, f"{path}: lateral normal force ({fx}, {fy})"
        assert all(abs(v) < 0.05 * weight for v in friction[path]), f"{path}: friction {friction[path]}"
    for component in (normal, friction):
        assert all(abs(v) < 1e-6 for v in component["/World/Float"]), f"/World/Float: {component['/World/Float']}"


def read_net_contact_forces(physx, stage):
    """Read both net contact-force components for every contact sensor, in one read.

    Returns ``(forces, columns)``: ``forces`` maps each component's attribute name to a dict from
    sensor prim path to its ``(x, y, z)`` force, and ``columns`` holds one ``(attribute, is_cuda,
    shape)`` per returned group, in return order, taken while the read is still open. Rows are
    mapped to prims through each group's interned prim list, and groups to components through
    their interned attribute token, both resolved with the stage's path dictionary.
    """
    import ovstage
    from ovphysx.types import ObjectScope, SimObjectType

    paths = ovstage.PathDictionary(stage)
    forces, columns = {}, []
    attrs = [NET_NORMAL_CONTACT_FORCE, NET_FRICTION_CONTACT_FORCE]
    with physx.read_sync(SimObjectType.CONTACT_SENSOR, attrs, scope=ObjectScope.ALL) as result:
        for group in result.groups:
            attr = paths.token_to_string(group.attribute)
            column = group.tensors[0]
            columns.append((attr, column.device.is_cuda, tuple(column.shape)))
            rows = forces.setdefault(attr, {})
            for path, row in zip(paths.get_path_strings(group.prim_list), column.numpy()):
                rows[path] = tuple(float(v) for v in row)
    return forces, columns


def read_contact_rows(physx, stage, **read_kwargs):
    """Read netNormalContactForce for the contact sensors a query selects, as ``[(path, (x, y, z)), ...]``.

    Rows keep their order, group by group, so a caller can check the order a contact query or a selection
    produces. ``read_kwargs`` go to ``PhysX.read_sync`` (``pattern=``, ``prim_paths=``, ``contacts=``).
    """
    import ovstage
    from ovphysx.types import SimObjectType

    paths = ovstage.PathDictionary(stage)
    rows = []
    with physx.read_sync(SimObjectType.CONTACT_SENSOR, [NET_NORMAL_CONTACT_FORCE], **read_kwargs) as result:
        for group in result.groups:
            for path, row in zip(paths.get_path_strings(group.prim_list), group.tensors[0].numpy()):
                rows.append((path, tuple(float(v) for v in row)))
    return rows


def slide(physx, pattern, vx=0.5):
    """Write a tangential linear velocity so the next step reports friction (the at-rest scenes leave it ~0)."""
    from ovphysx.types import SimObjectType

    with physx.write_sync(SimObjectType.RIGID_BODY, "linearVelocity", pattern=pattern) as session:
        for group in session.groups:
            tensor = group.tensors[0]
            tensor.assign(np.tile(np.asarray((vx, 0.0, 0.0), dtype=np.float32), (group.prim_count, 1)))
            if tensor.device.is_cuda:
                import warp as wp
                session.commit_sync(group, cuda_stream=int(wp.get_stream(tensor.device).cuda_stream or 1))
            else:
                session.commit_sync(group)
    physx.wait_all()
    physx.step_sync(1.0 / 60.0)


def read_contact_columns(physx, stage, attributes, **read_kwargs):
    """Read contact-sensor columns as ``{attribute: [(path, row_values), ...]}``, rows in order.

    Also returns ``{attribute: (is_cuda, shape)}`` for each group: a fixed group's tensor shape, or for an
    array group (one tensor per sensor) the tuple of its tensors' shapes, a row's values then being its
    elements one after the other -- prim-path handles resolved to path strings ("" for 0).
    ``read_kwargs`` go to ``PhysX.read_sync``.
    """
    import numpy
    import ovstage
    from ovphysx.types import SimObjectType

    paths = ovstage.PathDictionary(stage)
    columns, meta = {}, {}
    with physx.read_sync(SimObjectType.CONTACT_SENSOR, list(attributes), **read_kwargs) as result:
        for group in result.groups:
            attr = paths.token_to_string(group.attribute)
            prims = paths.get_path_strings(group.prim_list)
            rows = columns.setdefault(attr, [])
            if group.is_array:
                meta[attr] = (group.tensors[0].device.is_cuda, tuple(tuple(t.shape) for t in group.tensors))
                for path, tensor in zip(prims, group.tensors):
                    values = tensor.numpy().reshape(-1)
                    if values.dtype == numpy.uint64:  # prim-path handles
                        rows.append((path, [paths.path_to_string(int(h)) if h else "" for h in values]))
                    else:
                        rows.append((path, [float(v) for v in values]))
                continue
            tensor = group.tensors[0]
            meta[attr] = (tensor.device.is_cuda, tuple(tensor.shape))
            for path, row in zip(prims, tensor.numpy()):
                rows.append((path, [float(v) for v in row.reshape(-1)]))
    return columns, meta


def check_filtered_contact_detail(columns, meta, is_cuda, weights):
    """Check a filtered-detail read of sensors resting on /World/Ground, each filtering the ground then the other.

    ``weights`` maps each sensor path, in row order, to its weight. The count companion is an int32 ``(S, 2)``
    array with a positive ground lane and a zero other-sensor lane; each detail group holds one array per
    sensor, as many elements as its ground count, whose force x normal z sums to its weight and whose points
    lie on the ground's top face.
    """
    paths = list(weights)
    assert meta[FILTERED_CONTACT_COUNT] == (is_cuda, (len(paths), 2))
    counts = dict(columns[FILTERED_CONTACT_COUNT])
    assert [path for path, _ in columns[FILTERED_CONTACT_COUNT]] == paths
    for path in paths:
        assert counts[path][0] > 0 and counts[path][1] == 0, f"{path}: {counts[path]}"
    for attr, lanes in ((FILTERED_CONTACT_FORCE, 1), (FILTERED_CONTACT_NORMAL, 3), (FILTERED_CONTACT_POINT, 3)):
        is_cuda_group, shapes = meta[attr]
        assert is_cuda_group == is_cuda
        assert [path for path, _ in columns[attr]] == paths
        expected = tuple((int(counts[path][0]),) + ((lanes,) if lanes > 1 else ()) for path in paths)
        assert shapes == expected, f"{attr}: {shapes}"
    force, normal, point = (dict(columns[a]) for a in (FILTERED_CONTACT_FORCE, FILTERED_CONTACT_NORMAL, FILTERED_CONTACT_POINT))
    for path, weight in weights.items():
        z = sum(f * normal[path][3 * i + 2] for i, f in enumerate(force[path]))
        assert abs(z - weight) < 0.05 * weight, f"{path}: {z}"
        assert all(abs(point[path][3 * i + 2]) < 0.05 for i in range(len(force[path]))), f"{path}: {point[path]}"


def check_raw_contacts(columns, meta, is_cuda):
    """Check a raw-contact read of the write_contact_sensor_usda scene.

    The two resting sensors each hold an array of points, all against /World/Ground, whose force x normal z sums
    to its weight; the hovering sensor touches nothing, so it is absent from every raw group.
    """
    weights = {"/World/Light": 9.81, "/World/Heavy": 19.62}
    for attr, lanes in ((RAW_CONTACT_FORCE, 1), (RAW_CONTACT_NORMAL, 3), (RAW_CONTACT_OTHER_BODY, 1)):
        assert sorted(path for path, _ in columns[attr]) == sorted(weights), attr
        is_cuda_group, shapes = meta[attr]
        assert is_cuda_group == is_cuda
        assert all(shape[1:] == ((lanes,) if lanes > 1 else ()) for shape in shapes), f"{attr}: {shapes}"
    force, normal, other = (dict(columns[a]) for a in (RAW_CONTACT_FORCE, RAW_CONTACT_NORMAL, RAW_CONTACT_OTHER_BODY))
    for path, weight in weights.items():
        assert len(force[path]) > 0 and len(other[path]) == len(force[path])
        assert set(other[path]) == {"/World/Ground"}, f"{path}: {other[path]}"
        z = sum(f * normal[path][3 * i + 2] for i, f in enumerate(force[path]))
        assert abs(z - weight) < 0.05 * weight, f"{path}: {z}"


def destroy_ovstage_test_attachments(physx) -> None:
    """Detach and destroy ovstage instances created by attach_usd_with_ovstage."""
    if physx is None:
        return
    try:
        physx.wait_task(physx.detach_ovstage())
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

    PhysX.read_sync emits one group per scene partition per attribute, so a position column arrives as
    ``[N, 3]`` and an orientation column as ``[N, 4]``, and a multi-scene stage yields several of
    each. PhysX.read_sync does NOT guarantee the position and orientation groups arrive in the same
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
    with sdk.read_sync(SimObjectType.RIGID_BODY, ["position", "orientation"], scope=ObjectScope.ALL) as result:
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


def clone_bare_leaf_body_and_step(sdk, *, step_api: str = "sync", num_clones: int = 4, steps: int = 30):
    """NVBugs 6401436 regression body: clone a bare leaf rigid body, then simulate and read it.

    ``/World/Cube1`` in ``boxes_falling_on_groundplane.usda`` (Z-up) is a leaf prim that itself
    carries the rigid-body and collision APIs. There is no Xform wrapper, so the clone root IS the
    body. The USD-backed load accepted that source, returned success from clone(), and then
    SIGSEGV'd on the first step: a per-step USD mirror walk dereferenced the parent of a cloned
    path with no backing prim. Clones are runtime-only physics on the ovstage attach, so the
    clones must simulate (both step entry points), fall under gravity from their anchors, and
    come back through the session read.

    Shared by the GPU (DirectGPU) and CPU suites, since the crash reproduced on every device.
    Returns the ``[num_clones, 7]`` clone poses, one row per anchor in anchor order.
    """
    load_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
    sdk.wait_all()

    # Anchors far from the 11 authored boxes (x in 6..20, y = 10) so the clones are identifiable
    # by position alone when reading the complete rigid-body set.
    anchors = [(40.0 + 3.0 * i, 40.0, 5.0, 0.0, 0.0, 0.0, 1.0) for i in range(num_clones)]
    sdk.clone("/World/Cube1", [f"/World/Cube1_clone{i}" for i in range(num_clones)], anchor_transforms=anchors)
    sdk.wait_all()

    dt = 1.0 / 60.0
    for _ in range(steps):
        if step_api == "sync":
            sdk.step_sync(dt)
        else:
            sdk.step(dt)
            sdk.wait_all()

    poses = read_rigid_body_poses(sdk)
    assert poses.shape[0] == 11 + num_clones, f"expected 11 source bodies + {num_clones} clones, got {poses.shape[0]}"
    assert np.all(np.isfinite(poses)), "read returned non-finite poses"

    clones = poses[poses[:, 0] > 35.0]
    assert clones.shape[0] == num_clones, f"expected {num_clones} clone rows by position, got {clones.shape[0]}"
    ordered = np.zeros((num_clones, 7), dtype=np.float32)
    for i, (ax, ay, az, *_unused) in enumerate(anchors):
        match = clones[(np.abs(clones[:, 0] - ax) < 0.5) & (np.abs(clones[:, 1] - ay) < 0.5)]
        assert match.shape[0] == 1, f"clone {i} not found at its anchor ({ax}, {ay}); rows: {clones[:, :3]}"
        z = float(match[0, 2])
        # Free fall from z=5 over `steps` frames; must have dropped but not tunneled the ground (z=0).
        assert -2.0 < z < az - 0.5, f"clone {i} did not fall under gravity as expected: z={z}"
        ordered[i] = match[0]
    return ordered
