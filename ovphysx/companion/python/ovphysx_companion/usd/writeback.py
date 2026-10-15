# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-DEFORM-WRITE-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-USD-EDIT-001
# @covers AC-1 AC-2 AC-3 AC-4

"""Map completed CPU output and publish it to customer USD meshes.

map_deformable_output performs native embedding interpolation and explicit frame
conversion. write_simulation_output accepts destination-local stage-unit arrays
and applies reversible edits. Device transfer occurs explicitly before either.
"""

from dataclasses import dataclass
import math

from .meshes import MeshSnapshot, TetMeshSnapshot, cpu_vectors, validate_geometry
from .session import _usd


@dataclass(frozen=True)
class MeshFrame:
    destination: MeshSnapshot | TetMeshSnapshot
    positions: object
    step: int
    simulation_time: float
    velocities: object = None
    frame: str = "mesh-local"
    units: str = "stage"


def write_simulation_output(session, frames, *, layer, time_code):
    """Publish a complete batch after preflight; return a reversible EditPatch.

    time_code=None authors default values (e.g. a temporary preview layer).
    A finite numeric time_code authors that USD time sample, explicitly independent
    of the simulation's seconds. All entries must belong to one completed step.
    Callers retain exclusive access to input arrays until the call returns.
    """
    _, _, Usd, _, Vt = _usd()
    session._validate_layer(layer)
    if time_code is not None and (isinstance(time_code, bool) or not math.isfinite(time_code)):
        raise ValueError("time_code must be None or a finite numeric USD time sample")
    time = Usd.TimeCode.Default() if time_code is None else Usd.TimeCode(time_code)
    frames = tuple(frames)
    if not frames:
        raise ValueError("An output batch must contain at least one mesh")
    prepared, paths, step_time = [], set(), None
    for frame in frames:
        if (not isinstance(frame.step, int) or isinstance(frame.step, bool) or frame.step < 0
                or not math.isfinite(frame.simulation_time) or frame.simulation_time < 0):
            raise ValueError("Output requires a nonnegative step and finite simulation time")
        if step_time is None:
            step_time = (frame.step, frame.simulation_time)
        if step_time != (frame.step, frame.simulation_time):
            raise ValueError("All meshes in one batch must refer to the same step/time")
        if frame.frame != "mesh-local" or frame.units != "stage":
            raise ValueError("Convert output to destination-local stage units before USD publication")
        mesh = validate_geometry(session, frame.destination)
        path = frame.destination.path
        if path in paths:
            raise ValueError(f"Duplicate output destination: {path}")
        paths.add(path)
        size = len(frame.destination.points)
        positions = Vt.Vec3fArray.FromNumpy(cpu_vectors(frame.positions, size, "positions"))
        velocities = None
        if frame.velocities is not None:
            velocities = Vt.Vec3fArray.FromNumpy(cpu_vectors(frame.velocities, size, "velocities"))
        prepared.append((mesh, positions, velocities))

    def edit():
        for mesh, positions, velocities in prepared:
            if not mesh.GetPointsAttr().Set(positions, time):
                raise RuntimeError(f"USD rejected points for {mesh.GetPath()}")
            if velocities is not None and not mesh.GetVelocitiesAttr().Set(velocities, time):
                raise RuntimeError(f"USD rejected velocities for {mesh.GetPath()}")

    return session._apply(layer, edit, output=True)


def map_deformable_output(session, output, simulation, destination, mapping, *, library, topology_version, expected_owner):
    """Map owned solver output to a visual mesh using the native element kernel.

    Mapping points/offsets use simulation-local stage units. Simulation positions
    are local; velocities are world vectors. Both destination and simulation are
    captured from this edit session. expected_owner is the current runtime/source
    association; output from another attachment is rejected before interpolation.
    """
    import numpy as np
    from ..mesh import interpolate_elements, interpolate_velocities
    validate_geometry(session, simulation)
    validate_geometry(session, destination)
    if output.owner != expected_owner:
        raise ValueError('Output belongs to a stale or foreign runtime attachment')
    if (output.path != simulation.path or output.position_frame != 'simulation-local' or
            output.velocity_frame != 'world' or output.units != 'stage' or
            mapping.frame != 'simulation-local' or mapping.units != 'stage'):
        raise ValueError('Output and mapping require explicit simulation-local/world stage conventions')
    if simulation.meters_per_unit != destination.meters_per_unit:
        raise ValueError('Source and destination must use the same stage unit scale')
    if len(mapping.elements) != len(destination.points):
        raise ValueError('Visual mapping does not cover the destination topology')
    positions = cpu_vectors(output.positions, len(simulation.points), 'simulation positions')
    mapped = np.asarray(interpolate_elements(library, positions, mapping, topology_version=topology_version), dtype=np.float64)
    source_matrix = np.asarray(simulation.local_to_world, dtype=np.float64)
    destination_matrix = np.asarray(destination.local_to_world, dtype=np.float64)
    if source_matrix.shape != (4,4) or destination_matrix.shape != (4,4) or not np.isfinite(source_matrix).all() or not np.isfinite(destination_matrix).all():
        raise ValueError('Expected finite affine source and destination transforms')
    if not np.array_equal(source_matrix[:, 3], [0, 0, 0, 1]) or not np.array_equal(destination_matrix[:, 3], [0, 0, 0, 1]):
        raise ValueError('Expected affine source and destination transforms')
    inverse = np.linalg.inv(destination_matrix)
    homogeneous = np.column_stack((mapped, np.ones(len(mapped))))
    local = (homogeneous @ source_matrix @ inverse)[:, :3]
    velocities = None
    if output.velocities is not None:
        source_velocities = cpu_vectors(output.velocities, len(simulation.points), 'simulation velocities')
        # Convert world vectors before differentiating the simulation-local
        # normal, then apply the same source-to-destination linear transform.
        local_velocities = source_velocities @ np.linalg.inv(source_matrix[:3, :3])
        mapped_velocities = np.asarray(interpolate_velocities(library, positions, local_velocities, mapping,
                                        topology_version=topology_version), dtype=np.float64)
        velocities = mapped_velocities @ source_matrix[:3, :3] @ inverse[:3, :3]
    return MeshFrame(destination, local, output.step, output.simulation_time, velocities)
