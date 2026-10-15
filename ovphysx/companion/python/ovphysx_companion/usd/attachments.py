# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-ATTACH-COMPUTE-001
# @covers AC-1 AC-2 AC-3 AC-4

# @implements REQ-COMP-ATTACH-USD-001
# @covers AC-2 AC-3 AC-4
# @implements REQ-COMP-PYTHON-001
# @covers AC-2 AC-3 AC-4

"""Apply copied vertex, element, transform and filter proposals through customer USD."""

from dataclasses import dataclass
from operator import index

from .meshes import MeshSnapshot, TetMeshSnapshot, validate_geometry
from .session import DependencyError, _usd

_TYPE = "OmniPhysicsVtxVtxAttachment"
_OWNER_KEY = "ovphysxCompanion:attachmentOwner"


@dataclass(frozen=True)
class VertexAttachment:
    """Native-result boundary: paired indices into two captured simulation meshes."""

    source0: MeshSnapshot | TetMeshSnapshot
    source1: MeshSnapshot | TetMeshSnapshot
    indices0: tuple[int, ...]
    indices1: tuple[int, ...]


def apply_vertex_attachment(session, result, *, path, layer, owner):
    """Apply/update a vertex-to-vertex attachment at default time.

    Endpoints must be captured surface or volume simulation meshes. The
    caller supplies a stable owner string; existing prims without that ownership
    are never replaced. Only endpoint relationships and index arrays are changed.
    Stiffness, damping, other properties and children are preserved on update.
    This helper uses vertex indices. Element-coordinate attachments and collision
    filters use the separately typed helpers in this module.
    """
    Sdf, _, Usd, _, Vt = _usd()
    session._validate_layer(layer)
    if not isinstance(owner, str) or not owner:
        raise ValueError("owner must be a nonempty stable host/tool identifier")
    path = Sdf.Path(path)
    if not path.IsAbsolutePath() or not path.IsPrimPath() or path == Sdf.Path.absoluteRootPath:
        raise ValueError("attachment path must be an absolute prim path")
    definition = Usd.SchemaRegistry().FindConcretePrimDefinition(_TYPE)
    required = ("omniphysics:src0", "omniphysics:src1",
                "omniphysics:vtxIndicesSrc0", "omniphysics:vtxIndicesSrc1")
    if not definition or not all(p in definition.GetPropertyNames() for p in required):
        raise DependencyError(f"Register compatible codeless {_TYPE} schemas before opening any stage")
    endpoints = [validate_geometry(session, s) for s in (result.source0, result.source1)]
    if result.source0.path == result.source1.path:
        raise ValueError("attachment endpoints must be distinct meshes")
    for mesh in endpoints:
        if not (mesh.GetPrim().HasAPI("OmniPhysicsSurfaceDeformableSimAPI") or
                mesh.GetPrim().HasAPI("OmniPhysicsVolumeDeformableSimAPI")):
            raise ValueError(f"Endpoint is not a deformable simulation mesh: {mesh.GetPath()}")
        if path.HasPrefix(mesh.GetPath()):
            raise ValueError("attachment must not replace an endpoint or its subtree")
    pairs = []
    for values, snapshot in ((result.indices0, result.source0), (result.indices1, result.source1)):
        values = tuple(index(i) for i in values)
        if not values or any(i < 0 or i >= len(snapshot.points) for i in values):
            raise ValueError(f"Invalid attachment vertex indices: {snapshot.path}")
        pairs.append(values)
    if len(pairs[0]) != len(pairs[1]):
        raise ValueError("Attachment index arrays must have equal lengths")
    parent = session.stage.GetPrimAtPath(path.GetParentPath())
    if not parent or not parent.IsActive() or parent.IsInstanceProxy() or parent.IsInPrototype() or parent.IsInstance():
        raise ValueError("Attachment parent must exist and be editable, outside instances")
    existing = session.stage.GetPrimAtPath(path)
    if existing and (existing.GetCustomDataByKey(_OWNER_KEY) != owner or existing.GetTypeName() != _TYPE):
        raise ValueError("Destination is not an attachment owned by this tool")
    if existing and (not existing.IsActive() or existing.IsInstanceProxy()):
        raise ValueError("Existing attachment is not editable")

    def edit():
        prim = session.stage.DefinePrim(path, _TYPE)
        prim.SetCustomDataByKey(_OWNER_KEY, owner)
        if not prim.CreateAttribute("omniphysics:attachmentEnabled", Sdf.ValueTypeNames.Bool, custom=False).Set(True):
            raise RuntimeError("USD rejected attachmentEnabled")
        for n, snapshot in enumerate((result.source0, result.source1)):
            if not prim.CreateRelationship(f"omniphysics:src{n}", custom=False).SetTargets([snapshot.path]):
                raise RuntimeError("USD rejected attachment endpoint")
            if not prim.CreateAttribute(f"omniphysics:vtxIndicesSrc{n}", Sdf.ValueTypeNames.IntArray,
                                        custom=False).Set(Vt.IntArray(pairs[n])):
                raise RuntimeError("USD rejected attachment indices")

    return session._apply(layer, edit)


def match_vertex_attachment(source0, source1, *, distance, matcher):
    """Propose pairs from captured meshes using one bulk native matcher call.

    matcher(source_points, target_points, distance) accepts copied world-meter
    XYZ arrays and returns one int32 target index per source (-1 for unmatched).
    It can call companion.native.match_points in a native process or transport
    those arrays to a worker. No USD object crosses that computation boundary.
    Apply with apply_vertex_attachment, which checks both source revisions again.
    """
    import math

    if source0.source != source1.source or source0.path == source1.path:
        raise ValueError("Expected distinct endpoints from the same source revision")
    if not math.isfinite(distance) or distance < 0:
        raise ValueError("distance must be finite and nonnegative meters")

    def world_points(snapshot):
        matrix = snapshot.local_to_world
        units = snapshot.meters_per_unit
        if (len(matrix) != 4 or any(len(row) != 4 for row in matrix)
                or not all(math.isfinite(v) for row in matrix for v in row)
                or not math.isfinite(units) or units <= 0
                or any(matrix[i][3] != 0 for i in range(3)) or matrix[3][3] != 1):
            raise ValueError("Expected a finite affine transform and positive meter scale")
        points = []
        for point in snapshot.points:
            if len(point) != 3 or not all(math.isfinite(v) for v in point):
                raise ValueError("Expected finite XYZ source points")
            points.append(tuple((sum(point[j]*matrix[j][i] for j in range(3))+matrix[3][i])*units
                                for i in range(3)))
        return tuple(points)

    matches = tuple(index(i) for i in matcher(world_points(source0), world_points(source1), distance))
    if len(matches) != len(source0.points) or any(i < -1 or i >= len(source1.points) for i in matches):
        raise ValueError("Matcher returned invalid correspondence")
    pairs = tuple((i, j) for i, j in enumerate(matches) if j >= 0)
    if not pairs:
        raise ValueError("No attachment points matched within the supplied distance")
    return VertexAttachment(source0, source1, tuple(i for i, _ in pairs), tuple(j for _, j in pairs))


@dataclass(frozen=True)
class ElementAttachment:
    """Vertex source0 to triangle/tetrahedron source1 in captured index domains.

    Triangle coordinates are (u,v,normal offset) in target local stage units;
    tetrahedral coordinates are (u,v,w), fourth weight 1-u-v-w. Coordinates are
    computed by native authoring kernels, not by this USD application helper.
    """
    source0: object
    source1: object
    vertex_indices: tuple
    element_indices: tuple
    coordinates: tuple
    kind: str


@dataclass(frozen=True)
class ElementCollisionFilter:
    """Paired groups in collision triangle index domains (TetMesh surface faces).

    A zero-length group filters its entire source against the paired group.
    Both source meshes must have CollisionAPI; at least one must be deformable.
    """
    source0: object
    source1: object
    groups0: tuple
    groups1: tuple


def _owned_target(session, path, layer, owner, schema, required):
    Sdf, _, Usd, _, _ = _usd()
    session._validate_layer(layer)
    if not isinstance(owner, str) or not owner:
        raise ValueError('owner must be a nonempty stable host/tool identifier')
    path = Sdf.Path(path)
    if not path.IsAbsolutePath() or not path.IsPrimPath() or path == Sdf.Path.absoluteRootPath:
        raise ValueError('Expected an absolute prim path')
    definition = Usd.SchemaRegistry().FindConcretePrimDefinition(schema)
    if not definition or not all(name in definition.GetPropertyNames() for name in required):
        raise DependencyError(f'Register compatible codeless {schema} schemas before opening any stage')
    parent = session.stage.GetPrimAtPath(path.GetParentPath())
    if not parent or not parent.IsActive() or parent.IsInstanceProxy() or parent.IsInstance() or parent.IsInPrototype():
        raise ValueError('Destination parent must exist outside instances')
    prim = session.stage.GetPrimAtPath(path)
    if prim and (prim.GetCustomDataByKey(_OWNER_KEY) != owner or prim.GetTypeName() != schema):
        raise ValueError('Destination is not owned by this tool')
    if prim and (not prim.IsActive() or prim.IsInstanceProxy()):
        raise ValueError('Existing destination is not editable')
    return path


def _element_count(snapshot, kind):
    if kind == 'tetrahedron':
        if not isinstance(snapshot, TetMeshSnapshot):
            raise ValueError('Tetrahedron indices require a TetMesh source')
        return len(snapshot.tetrahedra)
    if isinstance(snapshot, TetMeshSnapshot):
        if not snapshot.surface_triangles:
            raise ValueError('Triangle indices require explicit TetMesh surface faces')
        return len(snapshot.surface_triangles)
    if any(count != 3 for count in snapshot.face_counts):
        raise ValueError('Triangle index domains require triangulated polygon meshes')
    return len(snapshot.face_counts)


def _write_owned(session, path, layer, owner, schema, sources, fields, enabled):
    Sdf, _, _, _, _ = _usd()
    def edit():
        prim = session.stage.DefinePrim(path, schema)
        prim.SetCustomDataByKey(_OWNER_KEY, owner)
        if not prim.CreateAttribute(enabled, Sdf.ValueTypeNames.Bool, custom=False).Set(True):
            raise RuntimeError('USD rejected enabled state')
        for n, snapshot in enumerate(sources):
            if not prim.CreateRelationship(f'omniphysics:src{n}', custom=False).SetTargets([snapshot.path]):
                raise RuntimeError('USD rejected endpoint')
        for name, value_type, value in fields:
            if not prim.CreateAttribute(name, value_type, custom=False).Set(value):
                raise RuntimeError(f'USD rejected {name}')
    return session._apply(layer, edit)


def apply_element_attachment(session, result, *, path, layer, owner):
    import math
    Sdf, _, _, _, Vt = _usd()
    if result.kind not in ('triangle', 'tetrahedron'):
        raise ValueError('Element attachment kind must be triangle or tetrahedron')
    suffix = 'Tri' if result.kind == 'triangle' else 'Tet'
    element_name = 'omniphysics:' + suffix.lower() + 'IndicesSrc1'
    coordinate_name = 'omniphysics:' + suffix.lower() + 'CoordsSrc1'
    schema = 'OmniPhysicsVtx' + suffix + 'Attachment'
    required = ('omniphysics:src0', 'omniphysics:src1', 'omniphysics:vtxIndicesSrc0', element_name, coordinate_name)
    path = _owned_target(session, path, layer, owner, schema, required)
    snapshots = (result.source0, result.source1)
    endpoints = [validate_geometry(session, source) for source in snapshots]
    if snapshots[0].path == snapshots[1].path:
        raise ValueError('Attachment sources must be distinct')
    for mesh in endpoints:
        if path.HasPrefix(mesh.GetPath()):
            raise ValueError('Attachment must not replace a source subtree')
        if not (mesh.GetPrim().HasAPI('OmniPhysicsSurfaceDeformableSimAPI') or
                mesh.GetPrim().HasAPI('OmniPhysicsVolumeDeformableSimAPI')):
            raise ValueError('Expected deformable simulation endpoints')
    indices = tuple(index(i) for i in result.vertex_indices)
    elements = tuple(index(i) for i in result.element_indices)
    coordinates = tuple(tuple(float(v) for v in value) for value in result.coordinates)
    limit = _element_count(result.source1, result.kind)
    if (not indices or len(indices) != len(elements) or len(indices) != len(coordinates) or
            any(i < 0 or i >= len(result.source0.points) for i in indices) or
            any(i < 0 or i >= limit for i in elements)):
        raise ValueError('Invalid attachment indices or coordinate counts')
    for value in coordinates:
        if len(value) != 3 or not all(math.isfinite(v) and abs(v) <= 3.402823466e38 for v in value):
            raise ValueError('Expected finite element coordinates')
        weights = value if result.kind == 'tetrahedron' else value[:2]
        if min(weights) < -1e-5 or sum(weights) > 1 + 1e-5:
            raise ValueError('Element coordinates lie outside the selected element')
    fields = [('omniphysics:vtxIndicesSrc0', Sdf.ValueTypeNames.IntArray, Vt.IntArray(indices)),
              (element_name, Sdf.ValueTypeNames.IntArray, Vt.IntArray(elements)),
              (coordinate_name, Sdf.ValueTypeNames.Vector3fArray, Vt.Vec3fArray(coordinates))]
    return _write_owned(session, path, layer, owner, schema, snapshots, fields, 'omniphysics:attachmentEnabled')


def apply_collision_filter(session, result, *, path, layer, owner):
    Sdf, _, _, _, Vt = _usd()
    from pxr import UsdPhysics
    schema = 'OmniPhysicsElementCollisionFilter'
    required = tuple(f'omniphysics:groupElem{field}{n}' for n in (0, 1) for field in ('Counts', 'Indices'))
    path = _owned_target(session, path, layer, owner, schema, required)
    snapshots = (result.source0, result.source1)
    endpoints = [validate_geometry(session, source) for source in snapshots]
    deformable = False
    for mesh in endpoints:
        prim = mesh.GetPrim()
        if not prim.HasAPI(UsdPhysics.CollisionAPI) or path.HasPrefix(prim.GetPath()):
            raise ValueError('Expected collision endpoints outside the filter destination')
        cursor = prim
        while cursor and not cursor.IsPseudoRoot():
            deformable |= cursor.HasAPI('OmniPhysicsDeformableBodyAPI')
            cursor = cursor.GetParent()
    if not deformable:
        raise ValueError('At least one filter endpoint must belong to a deformable body')
    groups = [tuple(tuple(index(i) for i in group) for group in values)
              for values in (result.groups0, result.groups1)]
    if not groups[0] or len(groups[0]) != len(groups[1]):
        raise ValueError('Filter group arrays must be nonempty and paired')
    fields = []
    for n, (snapshot, values) in enumerate(zip(snapshots, groups)):
        limit = _element_count(snapshot, 'triangle')
        if any(i < 0 or i >= limit for group in values for i in group):
            raise ValueError('Invalid filter element indices')
        fields += [(f'omniphysics:groupElemCounts{n}', Sdf.ValueTypeNames.UIntArray, Vt.UIntArray([len(g) for g in values])),
                   (f'omniphysics:groupElemIndices{n}', Sdf.ValueTypeNames.UIntArray, Vt.UIntArray([i for g in values for i in g]))]
    return _write_owned(session, path, layer, owner, schema, snapshots, fields, 'omniphysics:filterEnabled')


def remove_owned_attachment(session, *, path, layer, owner):
    """Disable a helper-owned attachment/filter without deleting user content.

    Retain the prim, authored tuning values, unrelated properties and children.
    Reapplication re-enables the managed object; undo restores its prior state.
    Removing a generated container with arbitrary user content is never implicit.
    """
    Sdf, _, _, _, _ = _usd()
    session._validate_layer(layer)
    prim = session.stage.GetPrimAtPath(path)
    schema = prim.GetTypeName() if prim else ''
    supported = {'OmniPhysicsVtxVtxAttachment', 'OmniPhysicsVtxTriAttachment',
                 'OmniPhysicsVtxTetAttachment', 'OmniPhysicsVtxXformAttachment', 'OmniPhysicsElementCollisionFilter'}
    if not prim or schema not in supported or not owner or prim.GetCustomDataByKey(_OWNER_KEY) != owner:
        raise ValueError('Destination is not an attachment/filter owned by this tool')
    _owned_target(session, path, layer, owner, schema, ())
    name = 'omniphysics:filterEnabled' if schema == 'OmniPhysicsElementCollisionFilter' else 'omniphysics:attachmentEnabled'
    def edit():
        if not prim.CreateAttribute(name, Sdf.ValueTypeNames.Bool, custom=False).Set(False):
            raise RuntimeError('USD rejected managed removal')
    return session._apply(layer, edit)


def attachment_input(snapshot, *, domain):
    """Convert a captured USD mesh to a native-data endpoint with a topology digest.

    domain is 'triangle' (explicit collision/surface faces) or 'tetrahedron'.
    The returned record contains only immutable Python values and can be sent to
    a worker; it retains the authoring source revision independently of topology.
    """
    import hashlib
    from ..attachments import MeshEndpoint
    _element_count(snapshot, domain)
    if domain == 'tetrahedron':
        topology, width = snapshot.tetrahedra, 4
    elif domain == 'triangle':
        width = 3
        topology = snapshot.surface_triangles if isinstance(snapshot,TetMeshSnapshot) else tuple(
            tuple(snapshot.face_indices[i:i+3]) for i in range(0,len(snapshot.face_indices),3))
    else:
        raise ValueError('Domain must be triangle or tetrahedron')
    version = hashlib.sha256(repr((len(snapshot.points),width,topology)).encode('utf-8')).hexdigest()
    return MeshEndpoint(snapshot.source,snapshot.path,version,snapshot.points,topology,width,
                        snapshot.local_to_world,snapshot.meters_per_unit)


def _proposal_sources(proposal, source0, source1):
    from .session import StaleResultError
    for endpoint,snapshot in ((proposal.source0,source0),(proposal.source1,source1)):
        domain = 'triangle' if endpoint.element_width == 3 else 'tetrahedron'
        if endpoint != attachment_input(snapshot,domain=domain):
            raise StaleResultError('Proposal endpoint/source/topology does not match the captured USD input')


def apply_attachment_proposal(session, proposal, source0, source1, **destination):
    """Validate the worker's exact endpoints, then apply one reversible constraint."""
    from ..attachments import AttachmentProposal
    if not isinstance(proposal,AttachmentProposal):
        raise TypeError('Expected an AttachmentProposal')
    _proposal_sources(proposal,source0,source1)
    result = ElementAttachment(source0,source1,proposal.vertex_indices,proposal.element_indices,
                               proposal.coordinates,proposal.kind)
    return apply_element_attachment(session,result,**destination)


def apply_filter_proposal(session, proposal, source0, source1, **destination):
    """An empty proposal authors nothing; it is never expanded to an all-body filter."""
    from ..attachments import FilterProposal
    if not isinstance(proposal,FilterProposal):
        raise TypeError('Expected a FilterProposal')
    _proposal_sources(proposal,source0,source1)
    return apply_collision_filter(session,ElementCollisionFilter(source0,source1,proposal.groups0,proposal.groups1),
                                  **destination)


@dataclass(frozen=True)
class TransformSnapshot:
    source: object
    path: str
    local_to_world: tuple
    meters_per_unit: float


def capture_transform(session,path):
    """Capture an editable default-time transform endpoint, including rigid subtrees."""
    from ..attachments import _matrix
    Sdf,_,Usd,UsdGeom,_ = _usd()
    session._check()
    path = Sdf.Path(path)
    prim = session.stage.GetPrimAtPath(path) if path.IsAbsolutePath() and path.IsPrimPath() else None
    if not prim or not prim.IsActive() or not prim.IsLoaded():
        raise ValueError('Expected an active transform endpoint')
    if prim.IsInstance() or prim.IsInstanceProxy() or prim.IsInPrototype() or not UsdGeom.Xformable(prim):
        raise ValueError('Expected an editable transform outside instances')
    matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    return TransformSnapshot(session.source,str(path),_matrix(matrix),UsdGeom.GetStageMetersPerUnit(session.stage))


@dataclass(frozen=True)
class TransformAttachment:
    source0: object
    source1: TransformSnapshot
    vertex_indices: tuple
    local_positions: tuple


def match_transform_attachment(source0, source1, *, vertex_indices=None):
    """Bind selected simulation vertices to a copied transform frame.

    This is a coordinate conversion only: no USD objects are read and no native
    spatial search is needed. Local anchors preserve nonuniform scale and shear.
    """
    from ..attachments import _inverse,_matrix,_transform
    if not isinstance(source1,TransformSnapshot) or source0.source != source1.source or source0.path == source1.path:
        raise ValueError('Expected distinct endpoints from the same source revision')
    indices = tuple(range(len(source0.points))) if vertex_indices is None else tuple(index(i) for i in vertex_indices)
    if not indices or any(i < 0 or i >= len(source0.points) for i in indices):
        raise ValueError('Expected valid nonempty vertex indices')
    world = _transform(tuple(source0.points[i] for i in indices),_matrix(source0.local_to_world))
    ratio = source0.meters_per_unit/source1.meters_per_unit
    local = _transform(tuple(tuple(v*ratio for v in p) for p in world),_inverse(_matrix(source1.local_to_world)))
    return TransformAttachment(source0,source1,indices,local)


def apply_transform_attachment(session,result,*,path,layer,owner):
    import math
    Sdf,_,_,_,Vt = _usd()
    schema = 'OmniPhysicsVtxXformAttachment'
    fields = ('omniphysics:src0','omniphysics:src1','omniphysics:vtxIndicesSrc0','omniphysics:localPositionsSrc1')
    path = _owned_target(session,path,layer,owner,schema,fields)
    mesh = validate_geometry(session,result.source0)
    session.validate(result.source1.source)
    if capture_transform(session,result.source1.path) != result.source1:
        raise ValueError('Transform endpoint changed')
    if not (mesh.GetPrim().HasAPI('OmniPhysicsSurfaceDeformableSimAPI') or
            mesh.GetPrim().HasAPI('OmniPhysicsVolumeDeformableSimAPI')):
        raise ValueError('Expected a deformable simulation source')
    if result.source0.path == result.source1.path or any(path.HasPrefix(Sdf.Path(s.path)) for s in (result.source0,result.source1)):
        raise ValueError('Expected distinct endpoints outside the destination subtree')
    indices = tuple(index(i) for i in result.vertex_indices)
    positions = tuple(tuple(float(v) for v in p) for p in result.local_positions)
    if (not indices or len(indices) != len(positions) or any(i < 0 or i >= len(result.source0.points) for i in indices) or
            any(len(p) != 3 or not all(math.isfinite(v) and abs(v) <= 3.402823466e38 for v in p) for p in positions)):
        raise ValueError('Invalid transform attachment indices/anchors')
    values = [(fields[2],Sdf.ValueTypeNames.IntArray,Vt.IntArray(indices)),
              (fields[3],Sdf.ValueTypeNames.Point3fArray,Vt.Vec3fArray(positions))]
    return _write_owned(session,path,layer,owner,schema,(result.source0,result.source1),values,'omniphysics:attachmentEnabled')
