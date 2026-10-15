# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-ATTACH-COMPUTE-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-DEFORM-COOK-001
# @covers AC-2 AC-3 AC-4
"""Copied CPU triangle/tetrahedron mapping, backed by USD-free runtime kernels."""
import ctypes as C
from dataclasses import dataclass
from operator import index
from .native import CompanionError, Status, _check, _host_sequence, _host_scalar

_F = C.POINTER(C.c_float)
_U = C.POINTER(C.c_uint32)
_I = C.POINTER(C.c_int32)


class _MeshApi(C.Structure):
    _fields_ = [('size', C.c_uint32), ('version', C.c_uint32),
                ('embed', C.CFUNCTYPE(C.c_uint32, _F, C.c_uint32, _U, C.c_uint32, C.c_uint32,
                                     _F, C.c_uint32, C.c_float, _I, _F, _F, _F, _U)),
                ('interpolate', C.CFUNCTYPE(C.c_uint32, _F, C.c_uint32, _U, C.c_uint32, C.c_uint32,
                                           _I, _F, _F, C.c_uint32, _F))]


def _acquire(library):
    try:
        acquire = library.ovphysx_companion_mesh_acquire
    except AttributeError as error:
        raise CompanionError(Status.UNAVAILABLE, 'mesh services') from error
    acquire.argtypes = [C.c_uint32, C.c_uint32, C.POINTER(_MeshApi)]
    acquire.restype = C.c_uint32
    table = _MeshApi()
    _check(acquire(1, C.sizeof(table), C.byref(table)), 'acquire_mesh')
    if table.version != 1 or table.size != C.sizeof(table) or not table.embed or not table.interpolate:
        raise CompanionError(Status.ABI_MISMATCH, 'mesh table')
    return table


def _points(values):
    points = tuple(tuple(_host_scalar(v) for v in _host_sequence(p)) for p in _host_sequence(values))
    if len(points) >= 2**32 or any(len(p) != 3 for p in points):
        raise ValueError('Expected fewer than 2**32 XYZ points')
    return points


def _topology(indices, width):
    if width not in (3, 4):
        raise ValueError('element_width must be 3 or 4')
    result = tuple(tuple(index(i) for i in _host_sequence(e)) for e in _host_sequence(indices))
    if len(result) >= 2**31 or any(len(e) != width or any(i < 0 or i >= 2**32 for i in e) for e in result):
        raise ValueError('Expected uint32 triangle/tetrahedron topology')
    return result


def _flat(values, ctype):
    return (ctype * sum(map(len, values)))(*(v for row in values for v in row))


@dataclass(frozen=True)
class ElementMap:
    """Owned mapping in the producer's explicit common frame and units.

    A matched point outside an element is projected: `distances` distinguishes
    projection from containment. Missing points have element=-1 and distance=inf.
    Triangles retain signed normal offsets; tangent displacement beyond a triangle
    edge cannot be reconstructed from barycentric coordinates and normal alone.
    """
    topology_version: str
    vertex_count: int
    topology: tuple
    element_width: int
    elements: tuple
    weights: tuple
    distances: tuple
    normal_offsets: tuple
    frame: str
    units: str

    @property
    def complete(self):
        return all(i >= 0 for i in self.elements)

    def require_complete(self):
        missing = tuple(i for i, element in enumerate(self.elements) if element < 0)
        if missing:
            raise ValueError(f'Unmapped point indices: {missing}')
        return self


def embed_points(library, vertices, topology, points, *, element_width, topology_version,
                 max_distance=0, frame='mesh-local', units='stage'):
    """Match copied points to triangles/tets without initializing a runtime.

    Arrays must already be ready CPU data in one frame/unit. The caller supplies
    the topology revision; interpolation requires that exact version again.
    Invalid topology reports its element index. Cancellation uses AuthoringService.
    """
    if not all(isinstance(value, str) and value for value in (topology_version, frame, units)):
        raise ValueError('Declare topology_version, frame and units')
    vertices, points = _points(vertices), _points(points)
    topology = _topology(topology, element_width)
    count = len(points)
    verts, queries = _flat(vertices, C.c_float), _flat(points, C.c_float)
    indices = _flat(topology, C.c_uint32)
    elements, weights = (C.c_int32*count)(), (C.c_float*(4*count))()
    distances, offsets = (C.c_float*count)(), (C.c_float*count)()
    invalid = C.c_uint32(2**32-1)
    status = _acquire(library).embed(verts, len(vertices), indices, len(topology), element_width,
        queries, count, _host_scalar(max_distance), elements, weights, distances, offsets, C.byref(invalid))
    if status != Status.OK:
        error = CompanionError(status, f'embed_points (invalid element {invalid.value})')
        error.invalid_element = None if invalid.value == 2**32-1 else invalid.value
        raise error
    return ElementMap(topology_version, len(vertices), topology, element_width, tuple(elements),
                      tuple(tuple(weights[i*4:i*4+4]) for i in range(count)), tuple(distances), tuple(offsets), frame, units)


def _interpolation_inputs(vertices, mapping, topology_version):
    if topology_version != mapping.topology_version:
        raise ValueError('Stale simulation topology version')
    mapping.require_complete()
    vertices = _points(vertices)
    if len(vertices) != mapping.vertex_count:
        raise ValueError('Simulation vertex count changed')
    topology = _topology(mapping.topology, mapping.element_width)
    count = len(mapping.elements)
    if len(mapping.weights) != count or any(len(w) != 4 for w in mapping.weights) or len(mapping.normal_offsets) != count:
        raise ValueError('Invalid mapping array lengths')
    elements = tuple(index(i) for i in mapping.elements)
    if any(i < 0 or i >= 2**31 for i in elements):
        raise ValueError('Invalid element index')
    weights = tuple(tuple(_host_scalar(v) for v in w) for w in mapping.weights)
    offsets = tuple(_host_scalar(v) for v in mapping.normal_offsets)
    return (_flat(vertices, C.c_float), len(vertices), _flat(topology, C.c_uint32),
            len(topology), mapping.element_width, (C.c_int32*count)(*elements), _flat(weights, C.c_float),
            (C.c_float*count)(*offsets), count)


def interpolate_elements(library, vertices, mapping, *, topology_version):
    """Map completed simulation points to owned visual/attachment positions.

    The vertices use the mapping's frame and units. Caller synchronization
    precedes this call; no GPU wait or implicit transfer is performed here.
    """
    args = _interpolation_inputs(vertices, mapping, topology_version)
    count = args[-1]
    output = (C.c_float*(count*3))()
    _check(_acquire(library).interpolate(*args, output), 'interpolate_elements')
    return tuple(tuple(output[i*3:i*3+3]) for i in range(count))


class _MotionApi(C.Structure):
    _fields_ = [('size', C.c_uint32), ('version', C.c_uint32),
                ('velocities', C.CFUNCTYPE(C.c_uint32, _F, _F, C.c_uint32, _U, C.c_uint32,
                                          C.c_uint32, _I, _F, _F, C.c_uint32, _F))]


def interpolate_velocities(library, vertices, velocities, mapping, *, topology_version):
    """Interpolate velocity, including a triangle normal offset's derivative.

    Positions and velocities share one Cartesian frame and length unit; velocity
    is per second. Weights and signed offsets are fixed material coordinates.
    Collapsed current triangles with nonzero offsets are rejected before output.
    """
    args = _interpolation_inputs(vertices, mapping, topology_version)
    values = _points(velocities)
    if len(values) != args[1]:
        raise ValueError('Velocity count must match simulation vertex count')
    try:
        acquire = library.ovphysx_companion_mesh_motion_acquire
    except AttributeError as error:
        raise CompanionError(Status.UNAVAILABLE, 'mesh motion') from error
    acquire.argtypes = [C.c_uint32, C.c_uint32, C.POINTER(_MotionApi)]
    acquire.restype = C.c_uint32
    table = _MotionApi()
    _check(acquire(1, C.sizeof(table), C.byref(table)), 'acquire_mesh_motion')
    if table.version != 1 or table.size != C.sizeof(table) or not table.velocities:
        raise CompanionError(Status.ABI_MISMATCH, 'mesh motion table')
    count = args[-1]
    output = (C.c_float*(count*3))()
    _check(table.velocities(args[0], _flat(values, C.c_float), *args[1:], output), 'interpolate_velocities')
    return tuple(tuple(output[i*3:i*3+3]) for i in range(count))

class _FilterApi(C.Structure):
    _fields_ = [('size',C.c_uint32),('version',C.c_uint32),
                ('pairs',C.CFUNCTYPE(C.c_uint32,_F,C.c_uint32,_U,C.c_uint32,
                                    _F,C.c_uint32,_U,C.c_uint32,C.c_float,C.c_uint32,_U,_U))]


def triangle_filter_pairs(library, vertices0, triangles0, vertices1, triangles1, *, max_distance, max_pairs=100000):
    """Copied CPU triangle pairs at inclusive distance in one common frame/unit.

    Degenerate/nonfinite topology, more than ten million candidate pairs, or
    result capacity exhaustion rejects the entire operation (INVALID_ARGUMENT).
    No partial pairs are published. Pair order is deterministic, source0 first.
    No USD, runtime attachment or SDK initialization is required.
    """
    a,b = _points(vertices0),_points(vertices1)
    ta,tb = _topology(triangles0,3),_topology(triangles1,3)
    max_pairs = index(max_pairs)
    if not 0 <= max_pairs <= 1000000 or len(ta)*len(tb)>10000000:
        raise ValueError('Filter capacity/work limit exceeded')
    try:
        acquire = library.ovphysx_companion_mesh_filter_acquire
    except AttributeError as error:
        raise CompanionError(Status.UNAVAILABLE,'mesh filters') from error
    acquire.argtypes = [C.c_uint32,C.c_uint32,C.POINTER(_FilterApi)]
    acquire.restype = C.c_uint32
    table = _FilterApi()
    _check(acquire(1,C.sizeof(table),C.byref(table)),'acquire_mesh_filter')
    if table.size != C.sizeof(table) or table.version != 1 or not table.pairs:
        raise CompanionError(Status.ABI_MISMATCH,'mesh filter table')
    output = (C.c_uint32*(2*max_pairs))()
    count = C.c_uint32()
    _check(table.pairs(_flat(a,C.c_float),len(a),_flat(ta,C.c_uint32),len(ta),
                       _flat(b,C.c_float),len(b),_flat(tb,C.c_uint32),len(tb),
                       _host_scalar(max_distance),max_pairs,output,C.byref(count)),'triangle_filter_pairs')
    return tuple(tuple(output[2*i:2*i+2]) for i in range(count.value))
