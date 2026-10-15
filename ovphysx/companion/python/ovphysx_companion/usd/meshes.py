# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-ATTACH-USD-001
# @covers AC-1 AC-4
# @implements REQ-COMP-IDENTITY-001
# @covers AC-3 AC-4

"""USD extraction into immutable, source-scoped records (no USD objects inside)."""

from dataclasses import dataclass

from .session import SourceRevision, StaleResultError, DependencyError, _usd


@dataclass(frozen=True)
class MeshSnapshot:
    source: SourceRevision
    path: str
    points: tuple
    face_counts: tuple
    face_indices: tuple
    local_to_world: tuple
    meters_per_unit: float


def _mesh(session, path):
    Sdf, _, _, UsdGeom, _ = _usd()
    session._check()
    path = Sdf.Path(path)
    if not path.IsAbsolutePath() or not path.IsPrimPath():
        raise ValueError("Expected an absolute prim path")
    prim = session.stage.GetPrimAtPath(path)
    if not prim or not prim.IsActive() or not prim.IsLoaded():
        raise StaleResultError(f"Missing/inactive mesh: {path}")
    if prim.IsInstanceProxy() or prim.IsInstance() or prim.IsInPrototype():
        raise ValueError(f"Instance authoring is not supported: {path}")
    mesh = UsdGeom.Mesh(prim)
    if not mesh:
        raise ValueError(f"Expected a polygon mesh: {path}")
    return mesh


def capture_mesh(session, path):
    """Capture a static polygon mesh at default time, including frame and units.

    Instanced and time-varying topology are rejected. Use capture_tet_mesh for
    a volume mesh; its element and surface index domains are kept separate.
    """
    _, _, Usd, UsdGeom, _ = _usd()
    mesh = _mesh(session, path)
    for attr in (mesh.GetFaceVertexCountsAttr(), mesh.GetFaceVertexIndicesAttr()):
        if attr.ValueMightBeTimeVarying():
            raise ValueError("Time-varying topology is not supported")
    points = mesh.GetPointsAttr().Get()
    counts, indices = mesh.GetFaceVertexCountsAttr().Get(), mesh.GetFaceVertexIndicesAttr().Get()
    if points is None or counts is None or indices is None or not len(points):
        raise ValueError(f"Mesh has no default points/topology: {path}")
    if any(n < 3 for n in counts) or sum(counts) != len(indices) or any(i < 0 or i >= len(points) for i in indices):
        raise ValueError(f"Invalid polygon topology: {path}")
    matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(mesh.GetPrim())
    return MeshSnapshot(session.source, str(mesh.GetPath()), tuple(map(tuple, points)),
                        tuple(counts), tuple(indices), tuple(map(tuple, matrix)),
                        UsdGeom.GetStageMetersPerUnit(session.stage))


def validate_mesh(session, snapshot):
    session.validate(snapshot.source)
    mesh = _mesh(session, snapshot.path)
    if (tuple(mesh.GetFaceVertexCountsAttr().Get() or ()) != snapshot.face_counts
            or tuple(mesh.GetFaceVertexIndicesAttr().Get() or ()) != snapshot.face_indices
            or len(mesh.GetPointsAttr().Get() or ()) != len(snapshot.points)):
        raise StaleResultError(f"Topology changed: {snapshot.path}")
    return mesh


def cpu_vectors(value, count, label):
    """Require explicit ready host data; never trigger an implicit GPU transfer."""
    try:
        import numpy as np
    except ImportError as exc:
        raise DependencyError("USD array helpers require numpy from the companion usd extra") from exc
    if not isinstance(value, np.ndarray):
        raise TypeError(f"{label} must be a ready CPU numpy.ndarray; complete/copy GPU data explicitly")
    if value.shape != (count, 3) or value.dtype.kind != "f":
        raise ValueError(f"{label} must contain {count} floating-point vectors of length 3")
    with np.errstate(over="ignore", invalid="ignore"):
        result = np.array(value, dtype=np.float32, order="C", copy=True)
    if not np.isfinite(result).all():
        raise ValueError(f"{label} contains non-finite or unrepresentable values")
    return result


@dataclass(frozen=True)
class TetMeshSnapshot:
    """Copied volume topology; tetrahedra and surface triangles have distinct domains."""
    source: SourceRevision
    path: str
    points: tuple
    tetrahedra: tuple
    surface_triangles: tuple
    local_to_world: tuple
    meters_per_unit: float


def _tet_mesh(session, path):
    Sdf, _, _, UsdGeom, _ = _usd()
    session._check()
    path = Sdf.Path(path)
    if not path.IsAbsolutePath() or not path.IsPrimPath():
        raise ValueError('Expected an absolute prim path')
    prim = session.stage.GetPrimAtPath(path)
    if not prim or not prim.IsActive() or not prim.IsLoaded():
        raise StaleResultError(f'Missing/inactive tetrahedral mesh: {path}')
    if prim.IsInstanceProxy() or prim.IsInstance() or prim.IsInPrototype():
        raise ValueError(f'Instance authoring is not supported: {path}')
    if not hasattr(UsdGeom, 'TetMesh'):
        raise DependencyError('Customer USD must provide UsdGeom.TetMesh for volume authoring')
    mesh = UsdGeom.TetMesh(prim)
    if not mesh:
        raise ValueError(f'Expected a tetrahedral mesh: {path}')
    return mesh


def capture_tet_mesh(session, path):
    """Capture explicit default-time tetrahedra and optional authored surface faces.

    Surface indices are never inferred: filters need the exact collision index
    domain. Missing surface faces prevent surface filtering, not tet attachments.
    """
    import math
    _, _, Usd, UsdGeom, _ = _usd()
    mesh = _tet_mesh(session, path)
    attrs = (mesh.GetTetVertexIndicesAttr(), mesh.GetSurfaceFaceVertexIndicesAttr())
    if any(attr.ValueMightBeTimeVarying() for attr in attrs):
        raise ValueError('Time-varying topology is not supported')
    points = tuple(map(tuple, mesh.GetPointsAttr().Get() or ()))
    tetrahedra = tuple(map(tuple, attrs[0].Get() or ()))
    surface = tuple(map(tuple, attrs[1].Get() or ()))
    if not points or not tetrahedra or not all(math.isfinite(v) for point in points for v in point):
        raise ValueError('Expected finite default points and tetrahedral topology')
    for values, width in ((tetrahedra, 4), (surface, 3)):
        if any(len(element) != width or len(set(element)) != width or
               any(i < 0 or i >= len(points) for i in element) for element in values):
            raise ValueError('Invalid tetrahedral/surface topology')
    matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(mesh.GetPrim())
    return TetMeshSnapshot(session.source, str(mesh.GetPath()), points, tetrahedra, surface,
                           tuple(map(tuple, matrix)), UsdGeom.GetStageMetersPerUnit(session.stage))


def validate_geometry(session, snapshot):
    if isinstance(snapshot, MeshSnapshot):
        return validate_mesh(session, snapshot)
    if not isinstance(snapshot, TetMeshSnapshot):
        raise TypeError('Expected a captured polygon or tetrahedral mesh')
    session.validate(snapshot.source)
    mesh = _tet_mesh(session, snapshot.path)
    if (tuple(map(tuple, mesh.GetTetVertexIndicesAttr().Get() or ())) != snapshot.tetrahedra or
            tuple(map(tuple, mesh.GetSurfaceFaceVertexIndicesAttr().Get() or ())) != snapshot.surface_triangles or
            len(mesh.GetPointsAttr().Get() or ()) != len(snapshot.points)):
        raise StaleResultError(f'Topology changed: {snapshot.path}')
    return mesh
