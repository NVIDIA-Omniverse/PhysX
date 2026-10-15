# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-DEFORM-WRITE-001
# @covers AC-1 AC-2 AC-3
# @implements REQ-COMP-CAMERA-001
# @covers AC-1 AC-4
# @implements REQ-COMP-QUERY-001
# @covers AC-3 AC-4
# @implements REQ-COMP-BUFFERS-001
# @covers AC-1 AC-2 AC-3
"""Explicit, copied simulation output through the existing core read API."""
from dataclasses import dataclass
import math
from .native import ToolsSession


@dataclass(frozen=True)
class DeformableOutput:
    owner: object
    path: str
    positions: object
    velocities: object
    step: int
    simulation_time: float
    position_frame: str = 'simulation-local'
    velocity_frame: str = 'world'
    units: str = 'stage'


@dataclass(frozen=True)
class BodyTarget:
    """Host-identified object incarnation; renew source_revision after input edits.

    Attachment identity and path alone do not distinguish deletion/recreation at
    the same path. The host supplies the immutable source revision associated
    with the sealed native input; output publication must not advance it.
    """
    owner: object
    path: str
    source_revision: object


@dataclass(frozen=True)
class BodyOutput:
    target: BodyTarget
    position: tuple
    orientation: tuple  # xyzw
    linear_velocity: tuple
    step: int
    simulation_time: float
    meters_per_unit: float
    frame: str = 'world'


def capture_body_output(physics, *, source_revision, step, simulation_time, meters_per_unit,
                        pattern=None, copy_gpu=False):
    """Copy completed world pose/velocity columns using the core read API.

    Host serialization and step completion precede this call. Units are the
    attached stage's length units and seconds. A GPU copy is explicit and blocking.
    Source revision is supplied by the host's sealed-input association, never
    inferred from a recyclable native key. No USD imports or simulation steps.
    """
    import numpy as np
    import ovstage
    from ovphysx.types import SimObjectType, ObjectScope
    if source_revision is None:
        raise ValueError('A source revision is required')
    hash(source_revision)
    if (isinstance(step, bool) or not isinstance(step, int) or step < 0 or
            not math.isfinite(simulation_time) or simulation_time < 0 or
            not math.isfinite(meters_per_unit) or meters_per_unit <= 0):
        raise ValueError('Declare a completed step, time and positive length scale')
    stage = getattr(physics, '_attached_ovstage', None)
    if stage is None:
        raise ValueError('A live ovstage attachment is required')
    attributes = {'position': 3, 'orientation': 4, 'linearVelocity': 3}
    data = {}
    with ToolsSession(physics) as lease, ovstage.PathDictionary(stage) as dictionary:
        identity = lease.identity
        with physics.read_sync(SimObjectType.RIGID_BODY, list(attributes), pattern=pattern, scope=ObjectScope.ALL) as result:
            for group in result.groups:
                attribute = dictionary.token_to_string(group.attribute)
                if group.is_array:
                    raise ValueError('Rigid output capture supports scalar bodies only; select a pattern excluding PointInstancer bodies and use instance-aware reads for those arrays')
                if (attribute not in attributes or group.is_array or group.is_delete or len(group.tensors) != 1 or
                        group.index_map is not None or group.prim_index_map is not None):
                    raise ValueError('Expected complete fixed body columns')
                paths = dictionary.get_path_strings(group.prim_list)
                paths = paths[group.prim_offset:group.prim_offset + group.prim_count]
                tensor = group.tensors[0]
                if tensor.device.is_cuda and not copy_gpu:
                    raise ValueError('GPU output requires explicit copy_gpu=True')
                values = np.asarray(tensor.numpy()).reshape(-1, attributes[attribute])
                if len(paths) != group.prim_count or len(values) != len(paths) or not np.isfinite(values).all():
                    raise ValueError('Invalid completed body output')
                for path, row in zip(paths, values):
                    columns = data.setdefault(path, {})
                    if attribute in columns:
                        raise ValueError(f'Duplicate body column for {path}')
                    columns[attribute] = tuple(float(v) for v in row)
        if lease.identity != identity:
            raise ValueError('Attachment changed during body capture')
    records = []
    for path, columns in sorted(data.items()):
        if set(columns) != set(attributes):
            raise ValueError(f'Incomplete body output for {path}')
        records.append(BodyOutput(BodyTarget(identity, path, source_revision), columns['position'],
                                  columns['orientation'], columns['linearVelocity'], step,
                                  simulation_time, meters_per_unit))
    return tuple(records)


def capture_deformable_output(physics, *, kind, step, simulation_time, pattern=None, copy_gpu=False):
    """Read every returned primitive, including ragged groups, into owned CPU arrays.

    Records have no source-revision or topology generation. The host must retain
    and validate the matching sealed input revision and mapping before writeback,
    including deletion/recreation at the same path.
    Call after the host has completed the selected simulation step, on its owner
    thread. `copy_gpu=True` explicitly authorizes blocking CUDA-to-host copies;
    the core read orders its producer event on Warp's current stream, and the
    array's synchronous numpy() copy completes before publication. No USD is used,
    and this function neither steps physics nor collects unrelated task receipts.
    """
    import numpy as np
    import ovstage
    from ovphysx.types import SimObjectType
    types = {'volume': SimObjectType.DEFORMABLE_VOLUME, 'surface': SimObjectType.DEFORMABLE_SURFACE}
    if kind not in types:
        raise ValueError('kind must be volume or surface')
    if isinstance(step, bool) or not isinstance(step, int) or step < 0 or not math.isfinite(simulation_time) or simulation_time < 0:
        raise ValueError('Supply a nonnegative completed step and finite simulation time')
    stage = getattr(physics, '_attached_ovstage', None)
    if stage is None:
        raise ValueError('A live ovstage attachment is required')
    data = {}
    with ToolsSession(physics) as lease, ovstage.PathDictionary(stage) as dictionary:
        identity = lease.identity
        with physics.read_sync(types[kind], ['points', 'velocities'], pattern=pattern) as result:
            for group in result.groups:
                attribute = dictionary.token_to_string(group.attribute)
                if attribute not in ('points', 'velocities') or group.is_delete:
                    raise ValueError('Unexpected deformable output column')
                paths = dictionary.get_path_strings(group.prim_list)
                if not group.is_array or len(paths) != group.prim_count or len(group.tensors) != len(paths):
                    raise ValueError('Deformable output must have one array per primitive')
                if group.index_map is not None or group.prim_index_map is not None:
                    raise ValueError('Sparse output requires an explicit host mapping')
                for path, tensor in zip(paths, group.tensors):
                    if tensor.device.is_cuda and not copy_gpu:
                        raise ValueError('GPU output requires explicit copy_gpu=True')
                    array = np.array(tensor.numpy(), dtype=np.float32, order='C', copy=True)
                    if array.ndim != 2 or array.shape[1] != 3 or not np.isfinite(array).all():
                        raise ValueError('Invalid completed deformable XYZ output')
                    array.flags.writeable = False
                    columns = data.setdefault(path, {})
                    if attribute in columns:
                        raise ValueError(f'Duplicate deformable column for {path}')
                    columns[attribute] = array
        if lease.identity != identity:
            raise ValueError('Attachment changed during output capture')
    output = []
    for path in sorted(data):
        columns = data[path]
        if set(columns) != {'points', 'velocities'} or columns['points'].shape != columns['velocities'].shape:
            raise ValueError(f'Incomplete deformable output for {path}')
        output.append(DeformableOutput(identity, path, columns['points'], columns['velocities'], step, simulation_time))
    return tuple(output)
