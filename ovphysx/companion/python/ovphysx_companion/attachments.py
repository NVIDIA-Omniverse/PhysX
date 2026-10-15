# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-ATTACH-COMPUTE-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Immutable attachment proposals; computation accepts no USD objects or pointers."""
from dataclasses import dataclass
import math
from .mesh import _points, _topology, embed_points, triangle_filter_pairs

_IDENTITY = ((1.,0.,0.,0.),(0.,1.,0.,0.),(0.,0.,1.,0.),(0.,0.,0.,1.))


def _matrix(value):
    result = tuple(tuple(float(v) for v in row) for row in value)
    if (len(result) != 4 or any(len(row) != 4 for row in result) or
            not all(math.isfinite(v) for row in result for v in row) or
            any(result[i][3] != 0 for i in range(3)) or result[3][3] != 1):
        raise ValueError('Expected a finite affine row-vector transform')
    _inverse(result)
    return result


def _inverse(matrix):
    # Gauss-Jordan with pivoting; no optional array/graphics dependency.
    rows = [list(matrix[i])+[float(i==j) for j in range(4)] for i in range(4)]
    for col in range(4):
        pivot = max(range(col,4), key=lambda row: abs(rows[row][col]))
        if rows[pivot][col] == 0:
            raise ValueError('Singular endpoint transform')
        rows[col], rows[pivot] = rows[pivot], rows[col]
        scale = rows[col][col]
        rows[col] = [v/scale for v in rows[col]]
        for row in range(4):
            if row != col:
                scale = rows[row][col]
                rows[row] = [a-scale*b for a,b in zip(rows[row],rows[col])]
    result = tuple(tuple(row[4:]) for row in rows)
    if not all(math.isfinite(v) for row in result for v in row):
        raise ValueError('Unrepresentable endpoint inverse transform')
    return result


def _transform(points, matrix):
    return tuple(tuple(sum(p[j]*matrix[j][i] for j in range(3))+matrix[3][i]
                       for i in range(3)) for p in points)


@dataclass(frozen=True)
class MeshEndpoint:
    """Copied identity/topology and local geometry with an affine world transform.

    source is a hashable host source/revision token; path identifies its mesh,
    topology_version identifies its exact index domain. Geometry uses local stage
    units; local_to_world follows USD/Gf's row-vector convention and includes
    scale/shear. No instance/prototype expansion is inferred.
    """
    source: object
    path: str
    topology_version: str
    points: tuple
    topology: tuple
    element_width: int
    local_to_world: tuple = _IDENTITY
    meters_per_unit: float = 1.

    def __post_init__(self):
        hash(self.source)
        if not isinstance(self.path,str) or not self.path.startswith('/') or self.path == '/':
            raise ValueError('Expected an absolute mesh path')
        if not isinstance(self.topology_version,str) or not self.topology_version:
            raise ValueError('Declare topology_version')
        points = _points(self.points)
        topology = _topology(self.topology,self.element_width)
        units = float(self.meters_per_unit)
        if (not math.isfinite(units) or units <= 0 or
                any(not math.isfinite(v) for p in points for v in p) or
                any(i >= len(points) for row in topology for i in row)):
            raise ValueError('Expected finite geometry, valid topology and positive units')
        object.__setattr__(self,'points',points)
        object.__setattr__(self,'topology',topology)
        object.__setattr__(self,'local_to_world',_matrix(self.local_to_world))
        object.__setattr__(self,'meters_per_unit',units)

    def world_meters(self):
        return tuple(tuple(v*self.meters_per_unit for v in p)
                     for p in _transform(self.points,self.local_to_world))


def _endpoints(a,b):
    if not isinstance(a,MeshEndpoint) or not isinstance(b,MeshEndpoint):
        raise TypeError('Pass immutable MeshEndpoint records')
    if a.source != b.source or a.path == b.path:
        raise ValueError('Expected distinct endpoints from the same source revision')


@dataclass(frozen=True)
class AttachmentProposal:
    source0: MeshEndpoint
    source1: MeshEndpoint
    kind: str
    vertex_indices: tuple
    element_indices: tuple
    coordinates: tuple
    mapping: object
    unmatched_vertices: tuple
    projected_vertices: tuple


def propose_attachment(library, source0, source1, *, max_distance=0, allow_projection=False):
    """Map source0 vertices into source1's local triangle/tet domain.

    max_distance is explicitly in target-local stage units, also for nonuniform
    scale/shear. Coordinates are the FIRST two triangle weights plus signed local
    normal offset, or FIRST three tet weights (last weight is the remainder).
    Unmatched vertices are reported and omitted. Tet projections and triangle
    edge projections that lose tangential displacement are omitted unless
    allow_projection=True. Empty proposals are valid diagnostics; USD application
    rejects empty constraints. The native mapping retains all diagnostics.
    """
    _endpoints(source0,source1)
    world = tuple(tuple(v/source1.meters_per_unit for v in p) for p in source0.world_meters())
    local = _transform(world,_inverse(source1.local_to_world))
    mapping = embed_points(library,source1.points,source1.topology,local,
                          element_width=source1.element_width,topology_version=source1.topology_version,
                          max_distance=max_distance,frame=source1.path,units='target-local-stage-units')
    selected, unmatched, projected = [], [], []
    for i,e in enumerate(mapping.elements):
        if e < 0:
            unmatched.append(i)
            continue
        # A triangle's signed normal displacement is representable. Displacement
        # past an edge is not; a projected tet point is also not an exact binding.
        error = mapping.distances[i] if source1.element_width == 4 else max(
            0.,mapping.distances[i]**2-mapping.normal_offsets[i]**2)**.5
        if error > 1e-6*max(1.,mapping.distances[i]):
            projected.append(i)
            if not allow_projection:
                continue
        selected.append(i)
    kind = 'triangle' if source1.element_width == 3 else 'tetrahedron'
    coords = tuple((mapping.weights[i][:2]+(mapping.normal_offsets[i],)) if kind == 'triangle'
                   else mapping.weights[i][:3] for i in selected)
    return AttachmentProposal(source0,source1,kind,tuple(selected),tuple(mapping.elements[i] for i in selected),
                              coords,mapping,tuple(unmatched),tuple(projected))


@dataclass(frozen=True)
class FilterProposal:
    source0: MeshEndpoint
    source1: MeshEndpoint
    pairs: tuple
    distance_meters: float

    @property
    def groups0(self):
        return tuple((i,) for i,j in self.pairs)

    @property
    def groups1(self):
        return tuple((j,) for i,j in self.pairs)


def propose_filter(library, source0, source1, *, distance_meters, max_pairs=100000):
    """Compute triangle pairs within inclusive world-meter distance.

    Endpoints must explicitly supply collision surface triangles, including for
    TetMesh. Empty results mean no pairs (never a wildcard/all-elements group).
    Results are lexicographically ordered and own all endpoint/result data.
    """
    _endpoints(source0,source1)
    if source0.element_width != 3 or source1.element_width != 3:
        raise ValueError('Collision filters require explicit surface triangle index domains')
    pairs = triangle_filter_pairs(library,source0.world_meters(),source0.topology,
                                  source1.world_meters(),source1.topology,
                                  max_distance=distance_meters,max_pairs=max_pairs)
    return FilterProposal(source0,source1,pairs,float(distance_meters))
