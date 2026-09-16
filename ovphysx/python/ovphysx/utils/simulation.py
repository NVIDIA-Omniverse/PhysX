# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-FRAME-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5
# @implements REQ-PYTHON-UTILS-001
# @covers AC-13

"""Stepping a running simulation and writing its output back to ovstage.

Every other submodule here authors a stage; this one drives a running
simulation. :func:`step_and_write_to_ovstage` performs the whole
step-read-write-back loop an application would otherwise spell out by hand,
without that workflow becoming part of the core ``ovphysx.api.PhysX`` surface.

That makes this the one submodule whose helpers are not pure ``pxr``: they need
``ovstage`` when called, and an attached ``PhysX`` instance to call them on.
``ovstage`` is therefore imported inside the function bodies, so
``import ovphysx.utils`` still needs nothing beyond ``pxr`` and the standard
library and still loads no native library.
"""

from collections.abc import Mapping, Sequence
from contextlib import ExitStack, contextmanager
from typing import TYPE_CHECKING

from ..types import ObjectScope, SimObjectType

if TYPE_CHECKING:
    from ..api import PhysX


_DEFAULT_OVSTAGE_OUTPUTS = {
    SimObjectType.RIGID_BODY: ("position", "orientation", "linearVelocity", "angularVelocity"),
    SimObjectType.ARTICULATION_LINK: ("position", "orientation", "linearVelocity", "angularVelocity"),
    SimObjectType.ARTICULATION: ("rootPosition", "rootOrientation", "rootLinearVelocity", "rootAngularVelocity"),
    SimObjectType.ARTICULATION_JOINT: ("jointPosition", "jointVelocity"),
    SimObjectType.VEHICLE_WHEEL: ("position", "orientation"),
    SimObjectType.DEFORMABLE_VOLUME: ("points", "velocities"),
    SimObjectType.DEFORMABLE_SURFACE: ("points", "velocities"),
    SimObjectType.PARTICLE_SET: ("points", "velocities"),
}

_FIXED_POSE_TYPES = {
    SimObjectType.RIGID_BODY,
    SimObjectType.ARTICULATION_LINK,
    SimObjectType.VEHICLE_WHEEL,
}


def _lane_fold_ovstage_array_tensors(tensors):
    import ovstage

    columns = []
    for tensor in tensors:
        source = ovstage.make_dltensor(tensor)
        source_shape = source.shape_tuple
        lanes = source_shape[-1] if len(source_shape) >= 2 else 1
        leading_shape = source_shape[:-1] if len(source_shape) >= 2 else source_shape
        leading = 1
        for dimension in leading_shape:
            leading *= dimension
        columns.append(
            ovstage.make_dltensor(
                tensor,
                dtype=ovstage.DLDataType(source.dtype.code, source.dtype.bits, lanes),
                shape=[leading],
                ndim=1,
            )
        )
    return columns


class _FixedPoseBuffers:
    __slots__ = ("scales", "matrices")

    def __init__(self, scales, matrices):
        self.scales = scales
        self.matrices = matrices


class _InstancerBuffers:
    __slots__ = (
        "baseline_positions",
        "baseline_orientations",
        "output_positions",
        "output_orientations",
    )

    def __init__(self, baseline_positions, baseline_orientations, output_positions, output_orientations):
        self.baseline_positions = baseline_positions
        self.baseline_orientations = baseline_orientations
        self.output_positions = output_positions
        self.output_orientations = output_orientations


class _OvStageOutputContext:
    """Short-lived OVStage wrappers used by one output call."""

    def __init__(self, physx: "PhysX"):
        import ovstage

        stage = physx._attached_ovstage
        if stage is None:
            raise RuntimeError("OvStageOutputCache requires an attached OVStage")
        required = (
            "get_attribute_write_floor",
            "read_attributes",
            "query_from_path_list",
            "release_query",
            "write_attribute",
            "advance_write_floor",
        )
        if any(not hasattr(stage, method) for method in required):
            raise RuntimeError("OvStageOutputCache requires attachment with an ovstage.Stage object")

        self._physx = physx
        self._stage = stage
        self._attach_handle = physx.get_attach_handle()
        if not self._attach_handle:
            raise RuntimeError("OvStageOutputCache requires a live ovphysx attachment")
        self._paths = ovstage.PathDictionary(stage)
        self._closed = False
        self._token_names = {}
        try:
            self._tokens = {
                name: self._paths.intern_token(name)
                for name in (
                    "positions",
                    "orientations",
                    "omni:fabric:worldMatrix",
                )
            }
        except Exception:
            self.close()
            raise

    def _check_bound(self):
        if self._closed:
            raise RuntimeError("OvStageOutputCache is closed")
        if self._physx._attached_ovstage is not self._stage:
            raise RuntimeError("OvStageOutputCache cannot be reused with another OVStage")
        if self._physx.get_attach_handle() != self._attach_handle:
            raise RuntimeError("OvStageOutputCache cannot be reused after detach or reattach")

    def _current_write_floor(self):
        with self._stage.get_attribute_write_floor(None) as floor_query:
            return floor_query.fetch()

    @contextmanager
    def _read_current_attributes(self, prim_list, attributes):
        import ovstage

        query = self._stage.query_from_path_list(prim_list)
        try:
            with self._stage.read_attributes(
                query,
                attributes,
                ovstage.OrdinalRange.latest(self._current_write_floor()),
            ) as read:
                read.wait()
                yield read.groups()
        finally:
            self._stage.release_query(query).wait()

    def _group_array(self, group, tensor_index, *, lanes, bits, name):
        import warp as wp

        tensor = group.tensor(tensor_index)
        if int(tensor.dtype.bits) != bits or int(tensor.dtype.lanes or 1) != lanes:
            raise RuntimeError(f"point-instancer {name} must use float{bits} elements with {lanes} lanes")
        if int(group.raw.data.cuda_sync.wait_event):
            self._physx._order_warp_stream_after_group(group.raw, wp)
        borrowed = wp.from_dlpack(group.dlpack(tensor_index))
        if len(borrowed.shape) != 2 or borrowed.shape[1] != lanes:
            raise RuntimeError(f"point-instancer {name} must have shape [N, {lanes}]")
        return borrowed

    def _capture_world_scales(self, prim_list, path_tuple, device):
        import warp as wp

        from .._utils_kernels import (
            extract_contiguous_world_scales,
            extract_world_scales,
        )

        scales = wp.empty((len(path_tuple), 3), dtype=wp.float64, device=device)
        destination_by_path = None
        captured = set()
        with self._read_current_attributes(prim_list, [self._tokens["omni:fabric:worldMatrix"]]) as groups:
            for group in groups:
                with group:
                    if group.is_delete:
                        continue
                    if group.is_array:
                        raise RuntimeError("omni:fabric:worldMatrix must be a fixed-size attribute")
                    tensor = group.tensor(0)
                    if int(tensor.dtype.bits) != 64 or int(tensor.dtype.lanes or 1) != 16:
                        raise RuntimeError("omni:fabric:worldMatrix must use float64 elements with 16 lanes")
                    if int(group.raw.data.cuda_sync.wait_event):
                        self._physx._order_warp_stream_after_group(group.raw, wp)
                    borrowed = wp.from_dlpack(group.dlpack(0))
                    if borrowed.dtype != wp.float64 or len(borrowed.shape) != 2 or borrowed.shape[1] != 16:
                        raise RuntimeError("omni:fabric:worldMatrix must have shape [N, 16]")

                    full_group = (
                        group.prim_count == len(path_tuple)
                        and group.prim_offset == 0
                        and not group.has_prim_index_map
                        and not group.has_data_index_map
                        and borrowed.shape[0] == len(path_tuple)
                    )
                    if full_group:
                        source_scales = scales
                        if borrowed.device != scales.device:
                            source_scales = wp.empty(
                                (len(path_tuple), 3),
                                dtype=wp.float64,
                                device=borrowed.device,
                            )
                        wp.launch(
                            extract_contiguous_world_scales,
                            dim=len(path_tuple),
                            inputs=[borrowed],
                            outputs=[source_scales],
                            device=borrowed.device,
                        )
                        if source_scales is not scales:
                            if borrowed.device.is_cuda:
                                wp.synchronize_stream(borrowed.device.stream)
                            wp.copy(scales, source_scales)
                        copy_device = scales.device if scales.device.is_cuda else borrowed.device
                        if copy_device.is_cuda:
                            wp.synchronize_stream(copy_device.stream)
                        return scales

                    if destination_by_path is None:
                        destination_by_path = {path: row for row, path in enumerate(path_tuple)}
                    group_paths = self._paths.get_paths(group.prim_list)
                    source_rows = []
                    destination_rows = []
                    for local in range(group.prim_count):
                        path = group_paths[group.prim_index(local)]
                        destination_row = destination_by_path.get(path)
                        if destination_row is None:
                            continue
                        source_rows.append(group.data_row_index(local))
                        destination_rows.append(destination_row)
                        captured.add(path)
                    if not source_rows:
                        continue

                    source_rows_array = wp.array(source_rows, dtype=wp.int32, device=device)
                    destination_rows_array = wp.array(destination_rows, dtype=wp.int32, device=device)
                    if borrowed.device == scales.device:
                        matrices = borrowed
                    else:
                        if borrowed.device.is_cuda:
                            wp.synchronize_stream(borrowed.device.stream)
                        matrices = wp.clone(borrowed, device=device)
                    wp.launch(
                        extract_world_scales,
                        dim=len(source_rows),
                        inputs=[matrices, source_rows_array, destination_rows_array],
                        outputs=[scales],
                        device=device,
                    )
                    if scales.device.is_cuda:
                        wp.synchronize_stream(scales.device.stream)
        missing = [path for path in path_tuple if path not in captured]
        if missing:
            missing_path = self._paths.path_to_string(missing[0])
            raise RuntimeError(
                f"omni:fabric:worldMatrix is missing for {missing_path!r}; "
                "populate or compute world matrices before calling this helper"
            )
        return scales

    def _capture_instancer_baseline(self, prim_list, path, device):
        import warp as wp

        with self._borrow_instancer_baseline(prim_list, path, device) as baseline:
            positions, orientations = baseline
            for source in baseline:
                if source.device != device and source.device.is_cuda:
                    wp.synchronize_stream(source.device.stream)
            owned_positions = wp.clone(positions, device=device)
            owned_orientations = wp.clone(orientations, device=device)
            if device.is_cuda:
                wp.synchronize_stream(device.stream)
            else:
                for source in baseline:
                    if source.device.is_cuda:
                        wp.synchronize_stream(source.device.stream)
            return owned_positions, owned_orientations

    @contextmanager
    def _borrow_instancer_baseline(self, prim_list, path, device):
        import warp as wp

        captured = {}
        attributes = [self._tokens["positions"], self._tokens["orientations"]]
        with self._read_current_attributes(prim_list, attributes) as groups:
            with ExitStack() as stack:
                for group in groups:
                    stack.enter_context(group)
                    if group.is_delete:
                        continue
                    group_paths = self._paths.get_paths(group.prim_list)
                    for local in range(group.prim_count):
                        if group_paths[group.prim_index(local)] != path:
                            continue
                        if not group.is_array:
                            raise RuntimeError("point-instancer pose columns must be array attributes")
                        if group.attribute == self._tokens["positions"]:
                            captured["positions"] = self._group_array(
                                group,
                                group.data_row_index(local),
                                lanes=3,
                                bits=32,
                                name="positions",
                            )
                        elif group.attribute == self._tokens["orientations"]:
                            captured["orientations"] = self._group_array(
                                group,
                                group.data_row_index(local),
                                lanes=4,
                                bits=16,
                                name="orientations",
                            )
                positions = captured.get("positions")
                if positions is None:
                    positions = wp.empty((0, 3), dtype=wp.float32, device=device)
                orientations = captured.get("orientations")
                if orientations is None:
                    orientations = wp.empty((0, 4), dtype=wp.float16, device=device)
                if len(positions) != len(orientations):
                    raise RuntimeError("point-instancer positions and orientations must have the same length")
                yield positions, orientations

    def close(self) -> None:
        """Release OVStage wrappers. Safe to call repeatedly."""
        if self._closed:
            return
        self._closed = True
        paths = self._paths
        self._paths = None
        if paths is not None:
            paths.destroy()

    def __enter__(self):
        self._check_bound()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _path_tuple(self, group):
        paths = tuple(self._paths.get_paths(group.prim_list))
        if len(paths) != group.prim_count:
            raise RuntimeError(f"pose prim list has {len(paths)} paths for {group.prim_count} prims")
        return paths

    def _token_name(self, token):
        name = self._token_names.get(token)
        if name is None:
            name = self._paths.token_to_string(token)
            if not name:
                raise RuntimeError(f"could not resolve emitted OVStage attribute token {token}")
            self._token_names[token] = name
        return name

    def _fixed_pose_buffers(self, prim_list, path_tuple, device):
        import warp as wp

        key = (path_tuple, str(device))
        buffers = self._fixed_buffers.get(key)
        if buffers is not None:
            return buffers
        buffers = _FixedPoseBuffers(
            self._capture_world_scales(prim_list, path_tuple, device),
            wp.empty(len(path_tuple), dtype=wp.mat44d, device=device),
        )
        self._fixed_buffers[key] = buffers
        return buffers

    def _instancer_pose_buffers(self, prim_list, path, device, count):
        import warp as wp

        key = (path, str(device))
        buffers = self._instancer_buffers.get(key)
        if buffers is None:
            baseline_positions, baseline_orientations = self._capture_instancer_baseline(prim_list, path, device)
            required = max(count, len(baseline_positions), len(baseline_orientations))
            buffers = _InstancerBuffers(
                baseline_positions,
                baseline_orientations,
                wp.empty((required, 3), dtype=wp.float32, device=device),
                wp.empty((required, 4), dtype=wp.float16, device=device),
            )
            self._instancer_buffers[key] = buffers
        required = max(count, len(buffers.baseline_positions), len(buffers.baseline_orientations))
        if required != buffers.output_positions.shape[0]:
            buffers.output_positions = wp.empty((required, 3), dtype=wp.float32, device=device)
            buffers.output_orientations = wp.empty((required, 4), dtype=wp.float16, device=device)
        return buffers, required, len(buffers.baseline_positions), len(buffers.baseline_orientations)

    def _transient_fixed_output(self, prim_list, path_tuple, positions, orientations):
        import warp as wp

        from .._utils_kernels import (
            compose_world_xforms,
            compose_world_xforms_from_matrices,
            extract_contiguous_world_scales,
        )

        count = len(positions)
        matrices = wp.empty(count, dtype=wp.mat44d, device=positions.device)
        with self._read_current_attributes(prim_list, [self._tokens["omni:fabric:worldMatrix"]]) as groups:
            for group in groups:
                with group:
                    if group.is_delete:
                        continue
                    tensor = group.tensor(0)
                    if group.is_array or int(tensor.dtype.bits) != 64 or int(tensor.dtype.lanes or 1) != 16:
                        raise RuntimeError("omni:fabric:worldMatrix must use float64 elements with 16 lanes")
                    if int(group.raw.data.cuda_sync.wait_event):
                        self._physx._order_warp_stream_after_group(group.raw, wp)
                    borrowed = wp.from_dlpack(group.dlpack(0))
                    full_group = (
                        group.prim_count == count
                        and group.prim_offset == 0
                        and not group.has_prim_index_map
                        and not group.has_data_index_map
                        and borrowed.shape == (count, 16)
                    )
                    if full_group:
                        if borrowed.device == positions.device:
                            if count:
                                wp.launch(
                                    compose_world_xforms_from_matrices,
                                    dim=count,
                                    inputs=[positions, orientations, borrowed],
                                    outputs=[matrices],
                                    device=positions.device,
                                )
                        else:
                            source_scales = wp.empty(
                                (count, 3),
                                dtype=wp.float64,
                                device=borrowed.device,
                            )
                            scales = wp.empty(
                                (count, 3),
                                dtype=wp.float64,
                                device=positions.device,
                            )
                            wp.launch(
                                extract_contiguous_world_scales,
                                dim=count,
                                inputs=[borrowed],
                                outputs=[source_scales],
                                device=borrowed.device,
                            )
                            if borrowed.device.is_cuda:
                                wp.synchronize_stream(borrowed.device.stream)
                            wp.copy(scales, source_scales)
                            copy_device = scales.device if scales.device.is_cuda else borrowed.device
                            if copy_device.is_cuda:
                                wp.synchronize_stream(copy_device.stream)
                            if count:
                                wp.launch(
                                    compose_world_xforms,
                                    dim=count,
                                    inputs=[positions, orientations, scales],
                                    outputs=[matrices],
                                    device=positions.device,
                                )
                        if matrices.device.is_cuda:
                            wp.synchronize_stream(matrices.device.stream)
                        return matrices

        if path_tuple is None:
            path_tuple = tuple(self._paths.get_paths(prim_list))
        scales = self._capture_world_scales(prim_list, path_tuple, positions.device)
        if count:
            wp.launch(
                compose_world_xforms,
                dim=count,
                inputs=[positions, orientations, scales],
                outputs=[matrices],
                device=positions.device,
            )
        if matrices.device.is_cuda:
            wp.synchronize_stream(matrices.device.stream)
        return matrices

    def _transient_instancer_output(self, prim_list, path, positions, orientations):
        import warp as wp

        from .._utils_kernels import merge_instancer_poses

        with self._borrow_instancer_baseline(prim_list, path, positions.device) as baseline:
            baseline_positions, baseline_orientations = baseline
            count = max(len(positions), len(baseline_positions), len(baseline_orientations))
            output_positions = wp.empty((count, 3), dtype=wp.float32, device=positions.device)
            output_orientations = wp.empty((count, 4), dtype=wp.float16, device=positions.device)
            kernel_baseline_positions = baseline_positions
            kernel_baseline_orientations = baseline_orientations
            if baseline_positions.device != positions.device:
                if baseline_positions.device.is_cuda:
                    wp.synchronize_stream(baseline_positions.device.stream)
                if len(baseline_positions):
                    wp.copy(output_positions, baseline_positions)
                kernel_baseline_positions = output_positions
            if baseline_orientations.device != positions.device:
                if baseline_orientations.device.is_cuda:
                    wp.synchronize_stream(baseline_orientations.device.stream)
                if len(baseline_orientations):
                    wp.copy(output_orientations, baseline_orientations)
                kernel_baseline_orientations = output_orientations

            if count:
                wp.launch(
                    merge_instancer_poses,
                    dim=count,
                    inputs=[
                        positions,
                        orientations,
                        len(positions),
                        kernel_baseline_positions,
                        kernel_baseline_orientations,
                        len(baseline_positions),
                        len(baseline_orientations),
                    ],
                    outputs=[output_positions, output_orientations],
                    device=positions.device,
                )
            if output_positions.device.is_cuda:
                wp.synchronize_stream(output_positions.device.stream)
            return output_positions, output_orientations

    def _cuda_event(self, device):
        import warp as wp

        if not device.is_cuda:
            return None
        key = str(device)
        event = self._events.get(key)
        if event is None:
            event = wp.Event(device=device)
            self._events[key] = event
        return event


class OvStageOutputCache(_OvStageOutputContext):
    """Reusable application-owned state for :func:`step_and_write_to_ovstage`.

    The cache binds to the exact OVStage attachment and lazily owns copies of
    scales derived from ``omni:fabric:worldMatrix`` and point-instancer pose
    arrays. It also reuses Warp buffers on each producing device. Call
    :meth:`refresh` after authored transforms, point-instancer pose arrays, or
    topology change. Omit the cache from :func:`step_and_write_to_ovstage` to
    read current OVStage values on every call instead.
    """

    def __init__(self, physx: "PhysX"):
        self._fixed_buffers = {}
        self._instancer_buffers = {}
        self._events = {}
        super().__init__(physx)

    def refresh(self) -> None:
        """Discard owned snapshots and device-specific buffers."""
        self._check_bound()
        self._fixed_buffers.clear()
        self._instancer_buffers.clear()

    def close(self) -> None:
        """Release cache-owned wrappers and buffers. Safe to call repeatedly."""
        if self._closed:
            return
        self._fixed_buffers.clear()
        self._instancer_buffers.clear()
        self._events.clear()
        super().close()


def _validate_fixed_pose_pair(position_group, orientation_group):
    import warp as wp

    if position_group.is_array or orientation_group.is_array:
        raise RuntimeError("fixed pose columns must not be array attributes")
    if position_group.prim_count != orientation_group.prim_count:
        raise RuntimeError("position and orientation groups have different prim counts")
    if position_group.index_map is not None or orientation_group.index_map is not None:
        raise RuntimeError("fixed pose columns unexpectedly carry a data index map")
    if position_group.prim_index_map is not None or orientation_group.prim_index_map is not None:
        raise RuntimeError("fixed pose columns unexpectedly carry a prim index map")
    if position_group.prim_offset != 0 or orientation_group.prim_offset != 0:
        raise RuntimeError("fixed pose columns unexpectedly carry a prim offset")

    positions = position_group.tensors[0]
    orientations = orientation_group.tensors[0]
    if positions.shape != (position_group.prim_count, 3):
        raise RuntimeError(f"position tensor must have shape [N, 3], got {positions.shape}")
    if orientations.shape != (orientation_group.prim_count, 4):
        raise RuntimeError(f"orientation tensor must have shape [N, 4], got {orientations.shape}")
    if positions.dtype != wp.float32 or orientations.dtype != wp.float32:
        raise RuntimeError("position and orientation tensors must use float32 elements")
    if positions.device != orientations.device:
        raise RuntimeError("position and orientation tensors must be on the same device")
    return positions, orientations


def _validate_instancer_pose_pair(position_group, orientation_group, path_tuple):
    import warp as wp

    if not position_group.is_array or not orientation_group.is_array:
        raise RuntimeError("point-instancer pose columns must be array attributes")
    if position_group.prim_count != 1 or orientation_group.prim_count != 1 or len(path_tuple) != 1:
        raise RuntimeError("each point-instancer pose group must describe exactly one instancer")
    if position_group.index_map is not None or orientation_group.index_map is not None:
        raise RuntimeError("point-instancer pose columns unexpectedly carry a data index map")
    if position_group.prim_index_map is not None or orientation_group.prim_index_map is not None:
        raise RuntimeError("point-instancer pose columns unexpectedly carry a prim index map")
    if position_group.prim_offset != 0 or orientation_group.prim_offset != 0:
        raise RuntimeError("point-instancer pose columns unexpectedly carry a prim offset")
    if len(position_group.tensors) != 1 or len(orientation_group.tensors) != 1:
        raise RuntimeError("point-instancer pose groups must carry one tensor per instancer")

    positions = position_group.tensors[0]
    orientations = orientation_group.tensors[0]
    if len(positions.shape) != 2 or positions.shape[1] != 3:
        raise RuntimeError(f"point-instancer positions must have shape [N, 3], got {positions.shape}")
    if len(orientations.shape) != 2 or orientations.shape[1] != 4:
        raise RuntimeError(f"point-instancer orientations must have shape [N, 4], got {orientations.shape}")
    if positions.shape[0] != orientations.shape[0]:
        raise RuntimeError("point-instancer positions and orientations have different lengths")
    if positions.dtype != wp.float32 or orientations.dtype != wp.float32:
        raise RuntimeError("point-instancer pose tensors must use float32 elements")
    if positions.device != orientations.device:
        raise RuntimeError("point-instancer pose tensors must be on the same device")
    return positions, orientations


def _record_completion_event(cache, device):
    event = cache._cuda_event(device)
    if event is not None:
        import warp as wp

        wp.get_stream(device).record_event(event, external=False)
    return event


def step_and_write_to_ovstage(
    physx: "PhysX",
    *,
    dt: float,
    output_ordinal: int,
    cache: OvStageOutputCache | None = None,
    outputs: Mapping[SimObjectType, Sequence[str]] | None = None,
) -> int:
    """Step once and write selected physics output to the attached OVStage.

    Fixed rigid-body, articulation-link, and vehicle-wheel poses are combined
    with scale derived from the current ``omni:fabric:worldMatrix`` and written
    directly to that attribute. The helper never writes ``omni:xform`` or
    ``omni:resetXformStack``. Point-instancer rigid-body poses are written to
    their native instancer-local ``positions`` and ``orientations`` arrays;
    unsimulated slots retain the current OVStage values.

    By default the helper reads current OVStage values on every call and keeps
    no snapshots after returning. Passing an application-owned ``cache`` opts
    into retaining derived scale, point-instancer baselines, Warp buffers, and
    CUDA events across calls. Persistent borrowed OVStage views are never kept.

    Other selected output is written to its shadow ``sim:<name>`` attribute.
    ``output_ordinal`` is caller-owned and must never be drained back into
    physics with :meth:`ovphysx.api.PhysX.update_from_ovstage`.

    Args:
        physx: Attached simulation instance to step and read.
        dt: Simulation time step in seconds.
        output_ordinal: OVStage ordinal used only for physics output.
        cache: Optional application-owned snapshots and reusable buffers.
        outputs: Optional object-type-to-attribute selection. ``None`` uses the
            documented default dynamic output set.

    Returns:
        Number of OVStage attributes written.
    """
    if cache is None:
        with _OvStageOutputContext(physx) as context:
            return _step_and_write_to_ovstage(physx, dt, output_ordinal, context, outputs, retain=False)
    if not isinstance(cache, OvStageOutputCache):
        raise TypeError("cache must be an OvStageOutputCache or None")
    cache._check_bound()
    if cache._physx is not physx:
        raise RuntimeError("cache belongs to another PhysX instance")
    return _step_and_write_to_ovstage(physx, dt, output_ordinal, cache, outputs, retain=True)


def _step_and_write_to_ovstage(physx, dt, output_ordinal, cache, outputs, *, retain):
    stage = cache._stage

    selected_outputs = _DEFAULT_OVSTAGE_OUTPUTS if outputs is None else outputs
    if not isinstance(selected_outputs, Mapping):
        raise TypeError("outputs must be a mapping")

    validated_outputs: list[tuple[SimObjectType, list[str]]] = []
    for object_type, attribute_names in selected_outputs.items():
        if not isinstance(object_type, SimObjectType):
            raise TypeError("outputs keys must be SimObjectType values")
        if isinstance(attribute_names, (str, bytes)) or not isinstance(attribute_names, Sequence):
            raise TypeError("outputs values must be sequences of attribute strings")
        names = list(attribute_names)
        if any(not isinstance(name, str) for name in names):
            raise TypeError("outputs values must contain only attribute strings")
        if object_type in _FIXED_POSE_TYPES:
            has_position = "position" in names
            has_orientation = "orientation" in names
            if has_position != has_orientation:
                raise ValueError(f"{object_type.name} position and orientation must be selected together")
        validated_outputs.append((object_type, names))

    import ovstage
    import warp as wp

    from .._utils_kernels import compose_world_xforms, merge_instancer_poses

    physx.step_sync(dt)
    written = 0
    for object_type, attribute_names in validated_outputs:
        with physx.read(object_type, attribute_names, scope=ObjectScope.ALL) as result:
            regular_groups = []
            pose_groups = {}
            for group in result.groups:
                if group.is_delete or not group.tensors:
                    continue
                emitted_name = cache._token_name(group.attribute)
                is_pose = object_type in _FIXED_POSE_TYPES and emitted_name in (
                    "position",
                    "orientation",
                    "positions",
                    "orientations",
                )
                if not is_pose:
                    regular_groups.append((group, emitted_name))
                    continue
                key = (group.is_array, group.prim_list)
                slot = pose_groups.setdefault(key, {})
                if emitted_name in slot:
                    raise RuntimeError(f"duplicate {emitted_name} group for prim list {group.prim_list}")
                slot[emitted_name] = group

            prepared = []
            for (is_array, prim_list_key), pair in pose_groups.items():
                expected = {"positions", "orientations"} if is_array else {"position", "orientation"}
                if set(pair) != expected:
                    missing = next(iter(expected - set(pair)))
                    raise RuntimeError(f"pose group for prim list {prim_list_key} is missing {missing}")
                if is_array:
                    path_tuple = cache._path_tuple(pair["positions"])
                    positions, orientations = _validate_instancer_pose_pair(
                        pair["positions"], pair["orientations"], path_tuple
                    )
                    if retain:
                        (
                            buffers,
                            count,
                            baseline_position_count,
                            baseline_orientation_count,
                        ) = cache._instancer_pose_buffers(
                            pair["positions"].prim_list,
                            path_tuple[0],
                            positions.device,
                            positions.shape[0],
                        )
                        if count:
                            wp.launch(
                                merge_instancer_poses,
                                dim=count,
                                inputs=[
                                    positions,
                                    orientations,
                                    positions.shape[0],
                                    buffers.baseline_positions,
                                    buffers.baseline_orientations,
                                    baseline_position_count,
                                    baseline_orientation_count,
                                ],
                                outputs=[
                                    buffers.output_positions,
                                    buffers.output_orientations,
                                ],
                                device=positions.device,
                            )
                        event = _record_completion_event(cache, positions.device)
                        output_positions = buffers.output_positions
                        output_orientations = buffers.output_orientations
                    else:
                        output_positions, output_orientations = cache._transient_instancer_output(
                            pair["positions"].prim_list,
                            path_tuple[0],
                            positions,
                            orientations,
                        )
                        event = None
                    prepared.append((True, pair, output_positions, output_orientations, event))
                else:
                    positions, orientations = _validate_fixed_pose_pair(pair["position"], pair["orientation"])
                    if retain:
                        path_tuple = cache._path_tuple(pair["position"])
                        buffers = cache._fixed_pose_buffers(pair["position"].prim_list, path_tuple, positions.device)
                        if positions.shape[0]:
                            wp.launch(
                                compose_world_xforms,
                                dim=positions.shape[0],
                                inputs=[positions, orientations, buffers.scales],
                                outputs=[buffers.matrices],
                                device=positions.device,
                            )
                        event = _record_completion_event(cache, positions.device)
                        matrices = buffers.matrices
                    else:
                        matrices = cache._transient_fixed_output(
                            pair["position"].prim_list,
                            None,
                            positions,
                            orientations,
                        )
                        event = None
                    prepared.append((False, pair, matrices, None, event))

            for group, emitted_name in regular_groups:
                query = stage.query_from_path_list(group.prim_list)
                try:
                    tensors = _lane_fold_ovstage_array_tensors(group.tensors) if group.is_array else group.tensors[0]
                    index_map = group.index_map.numpy().tolist() if group.index_map is not None else None
                    stage.write_attribute(
                        query,
                        f"sim:{emitted_name}",
                        output_ordinal,
                        tensors,
                        is_array=group.is_array,
                        semantic=group.semantic,
                        index_map=index_map,
                        count=group.prim_count if index_map is not None else None,
                        cuda_event=group.cuda_wait_event,
                        cuda_stream=group.cuda_stream,
                    ).wait()
                    written += 1
                finally:
                    stage.release_query(query).wait()

            for is_array, pair, first_output, second_output, event in prepared:
                if is_array:
                    query = stage.query_from_path_list(pair["positions"].prim_list)
                    try:
                        stage.write_attribute(
                            query,
                            cache._tokens["positions"],
                            output_ordinal,
                            _lane_fold_ovstage_array_tensors([first_output]),
                            is_array=True,
                            semantic=ovstage.AttributeSemantic.POINT,
                            index_map=None,
                            count=None,
                            cuda_event=event.cuda_event if event is not None else None,
                            cuda_stream=None,
                        ).wait()
                        written += 1
                        stage.write_attribute(
                            query,
                            cache._tokens["orientations"],
                            output_ordinal,
                            _lane_fold_ovstage_array_tensors([second_output]),
                            is_array=True,
                            semantic=ovstage.AttributeSemantic.QUATERNION,
                            index_map=None,
                            count=None,
                            cuda_event=event.cuda_event if event is not None else None,
                            cuda_stream=None,
                        ).wait()
                        written += 1
                    finally:
                        stage.release_query(query).wait()
                else:
                    query = stage.query_from_path_list(pair["position"].prim_list)
                    try:
                        stage.write_attribute(
                            query,
                            cache._tokens["omni:fabric:worldMatrix"],
                            output_ordinal,
                            first_output,
                            is_array=False,
                            semantic=ovstage.AttributeSemantic.MATRIX,
                            index_map=None,
                            count=None,
                            cuda_event=event.cuda_event if event is not None else None,
                            cuda_stream=None,
                        ).wait()
                        written += 1
                    finally:
                        stage.release_query(query).wait()

    stage.advance_write_floor(ordinal=output_ordinal).wait()
    return written


__all__ = ["OvStageOutputCache", "step_and_write_to_ovstage"]
