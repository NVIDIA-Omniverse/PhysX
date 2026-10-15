# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-DEFORM-USD-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Apply copied deformable cook records using customer-installed USD only.

Records use one explicit body-local frame and stage units. Topology versions
come from the producer; visual embedding is metadata for the companion output
mapper. USD's native deformable parser still consumes the standard bind poses.
"""
from dataclasses import dataclass
from operator import index
import math
from .session import _usd, SourceRevision, DependencyError
from ..native import _host_sequence, _host_scalar

_OWNER = 'ovphysxCompanion:deformableOwner'
_VERSION = 'ovphysxCompanion:topologyVersion'


@dataclass(frozen=True)
class Embedding:
    """One simulation element and full barycentric weights per visual vertex."""
    simulation_topology: str
    elements: tuple
    weights: tuple


@dataclass(frozen=True)
class DeformableMesh:
    name: str
    role: str  # simulation, collision, visual
    kind: str  # triangle, tetrahedron
    points: tuple
    elements: tuple
    topology_version: str
    rest_points: object = None
    rest_elements: object = None
    bind_points: object = None
    embedding: object = None
    collision: bool = False  # simulation mesh may also be its own collider


@dataclass(frozen=True)
class DeformableMaterial:
    """Material coefficients in the destination stage's length/mass units."""
    density: float = 1000
    youngs_modulus: float = 500000
    poissons_ratio: float = .45
    static_friction: float = .5
    dynamic_friction: float = .25


@dataclass(frozen=True)
class DeformableBody:
    source: SourceRevision
    meshes: tuple
    material: DeformableMaterial = DeformableMaterial()
    frame: str = 'body-local'
    units: str = 'stage'


def _points(values):
    points = tuple(tuple(_host_scalar(v) for v in _host_sequence(point)) for point in _host_sequence(values))
    if not points or any(len(p) != 3 or not all(math.isfinite(v) and abs(v) <= 3.402823466e38 for v in p) for p in points):
        raise ValueError('Expected finite float32 XYZ points')
    return points


def _elements(values, point_count, width):
    elements = tuple(tuple(index(v) for v in _host_sequence(element)) for element in _host_sequence(values))
    if not elements or any(len(e) != width or len(set(e)) != width or
                           any(i < 0 or i >= point_count for i in e) for e in elements):
        raise ValueError('Invalid mesh element topology')
    return elements


def _schema(prim, name, instance=None):
    if not (prim.ApplyAPI(name, instance) if instance is not None else prim.ApplyAPI(name)):
        raise DependencyError(f'Customer USD rejected {name}; register compatible codeless schemas')


def _put(attribute, value):
    if not attribute.Set(value):
        raise RuntimeError(f'USD rejected {attribute.GetPath()}')


def _set(prim, name, value_type, value, *, custom=False):
    _put(prim.CreateAttribute(name, value_type, custom=custom), value)


def apply_deformable_body(session, result, *, path, layer, owner):
    """Construct/update managed mesh roles and material in one reversible batch.

    Topology edits invalidate captured source revisions. Role names and types are
    stable on reconstruction: changing them requires an explicit host migration.
    Existing non-owned prims and properties outside the schema fields below are
    preserved. Input data contains no USD or native runtime objects.
    """
    Sdf, _, Usd, UsdGeom, Vt = _usd()
    from pxr import UsdPhysics, UsdShade
    session.validate(result.source)
    session._validate_layer(layer)
    if result.frame != 'body-local' or result.units != 'stage':
        raise ValueError('Convert every mesh to body-local stage units before application')
    if not isinstance(owner, str) or not owner:
        raise ValueError('owner must be a nonempty stable tool identifier')
    root = Sdf.Path(path)
    if not root.IsAbsolutePath() or not root.IsPrimPath() or root == Sdf.Path.absoluteRootPath:
        raise ValueError('Expected an absolute body prim path')
    parent = session.stage.GetPrimAtPath(root.GetParentPath())
    if not parent or not parent.IsActive() or parent.IsInstance() or parent.IsInstanceProxy() or parent.IsInPrototype():
        raise ValueError('Body parent must exist outside instances')
    meshes = tuple(result.meshes)
    sims = [m for m in meshes if m.role == 'simulation']
    if len(sims) != 1 or sum(m.role == 'collision' or m.collision for m in meshes) != 1:
        raise ValueError('A body requires exactly one simulation mesh and one collider')
    simulation = sims[0]
    if simulation.kind not in ('triangle', 'tetrahedron'):
        raise ValueError('Unsupported simulation topology')
    if simulation.kind == 'tetrahedron' and not hasattr(UsdGeom, 'TetMesh'):
        raise DependencyError('Volume construction requires customer UsdGeom.TetMesh')
    names, prepared = set(), []
    for mesh in meshes:
        if (not isinstance(mesh.name, str) or not Sdf.Path.IsValidIdentifier(mesh.name) or
                mesh.name == 'Material' or mesh.name in names):
            raise ValueError('Use unique mesh names distinct from Material')
        names.add(mesh.name)
        if mesh.role not in ('simulation', 'collision', 'visual') or mesh.kind not in ('triangle', 'tetrahedron'):
            raise ValueError('Unsupported mesh role or topology')
        if mesh.role == 'visual' and mesh.kind != 'triangle':
            raise ValueError('Visual topology must be triangles')
        if mesh.collision and mesh.role != 'simulation':
            raise ValueError('collision flag is only for a simulation mesh used as its own collider')
        if mesh.role == 'collision' and mesh.kind != simulation.kind:
            raise ValueError('Collision and simulation topology kinds must match')
        if not isinstance(mesh.topology_version, str) or not mesh.topology_version:
            raise ValueError('Every mesh requires an explicit topology version')
        points = _points(mesh.points)
        width = 4 if mesh.kind == 'tetrahedron' else 3
        elements = _elements(mesh.elements, len(points), width)
        rest = points if mesh.rest_points is None else _points(mesh.rest_points)
        rest_elements = elements if mesh.rest_elements is None else _elements(mesh.rest_elements, len(rest), width)
        if len(rest_elements) != len(elements):
            raise ValueError('Rest and simulation elements require explicit one-to-one correspondence')
        if mesh.rest_elements is None and any(i >= len(rest) for e in elements for i in e):
            raise ValueError('Rest points do not match the declared rest topology')
        bind = points if mesh.bind_points is None else _points(mesh.bind_points)
        if len(bind) != len(points):
            raise ValueError('Bind point count must match its own mesh')
        embedding = mesh.embedding
        if embedding is not None:
            if mesh.role != 'visual' or embedding.simulation_topology != simulation.topology_version:
                raise ValueError('Embedding belongs to a different simulation topology')
            ids = tuple(index(i) for i in _host_sequence(embedding.elements))
            weights = tuple(tuple(_host_scalar(v) for v in _host_sequence(w)) for w in _host_sequence(embedding.weights))
            sim_width = 4 if simulation.kind == 'tetrahedron' else 3
            if (len(ids) != len(points) or len(weights) != len(points) or
                    any(i < 0 or i >= len(simulation.elements) for i in ids) or
                    any(len(w) != sim_width or not all(math.isfinite(v) and 0 <= v <= 1 for v in w) or
                        abs(sum(w)-1) > 1e-5 for w in weights)):
                raise ValueError('Incomplete or invalid visual embedding')
            embedding = (ids, weights)
        prepared.append((mesh, points, elements, rest, rest_elements, bind, embedding))
    material = result.material
    values = {name: _host_scalar(getattr(material, field)) for name, field in (
        ('density', 'density'), ('youngsModulus', 'youngs_modulus'), ('poissonsRatio', 'poissons_ratio'),
        ('staticFriction', 'static_friction'), ('dynamicFriction', 'dynamic_friction'))}
    if (not all(math.isfinite(v) and abs(v) <= 3.402823466e38 for v in values.values()) or
            values['density'] <= 0 or values['youngsModulus'] <= 0 or not -1 < values['poissonsRatio'] < .5 or
            values['staticFriction'] < 0 or values['dynamicFriction'] < 0):
        raise ValueError('Invalid deformable material coefficients')
    required = ['OmniPhysicsDeformableBodyAPI', 'OmniPhysicsDeformableMaterialAPI',
                'OmniPhysicsVolumeDeformableSimAPI' if simulation.kind == 'tetrahedron' else 'OmniPhysicsSurfaceDeformableSimAPI']
    for name in required:
        if not Usd.SchemaRegistry().FindAppliedAPIPrimDefinition(name):
            raise DependencyError(f'Register compatible codeless {name} before construction')
    targets = [(root, 'Xform'), (root.AppendChild('Material'), 'Material')]
    targets += [(root.AppendChild(m.name), 'TetMesh' if m.kind == 'tetrahedron' else 'Mesh') for m in meshes]
    for target, type_name in targets:
        prim = session.stage.GetPrimAtPath(target)
        if prim and (prim.GetCustomDataByKey(_OWNER) != owner or prim.GetTypeName() != type_name or
                     not prim.IsActive() or prim.IsInstance() or prim.IsInstanceProxy()):
            raise ValueError(f'Destination {target} is not a compatible helper-owned prim')
    existing = session.stage.GetPrimAtPath(root)
    if existing:
        previous = tuple(existing.GetCustomDataByKey('ovphysxCompanion:meshNames') or ())
        if set(previous) != names:
            raise ValueError('Changing managed mesh roles requires an explicit host migration')
        for mesh in meshes:
            prim = session.stage.GetPrimAtPath(root.AppendChild(mesh.name))
            if prim and prim.GetCustomDataByKey('ovphysxCompanion:meshRole') != mesh.role:
                raise ValueError('Changing managed mesh roles requires an explicit host migration')
        for mesh, points, elements, *_ in prepared:
            prim = session.stage.GetPrimAtPath(root.AppendChild(mesh.name))
            if prim and prim.GetCustomDataByKey(_VERSION) == mesh.topology_version:
                if mesh.kind == 'tetrahedron':
                    previous_elements = tuple(map(tuple, prim.GetAttribute('tetVertexIndices').Get() or ()))
                else:
                    indices = tuple(prim.GetAttribute('faceVertexIndices').Get() or ())
                    counts = tuple(prim.GetAttribute('faceVertexCounts').Get() or ())
                    previous_elements = tuple(zip(*[iter(indices)]*3)) if counts == (3,)*len(elements) else ()
                if len(prim.GetAttribute('points').Get() or ()) != len(points) or previous_elements != elements:
                    raise ValueError('Changed topology requires a new topology version')
    def edit():
        body = UsdGeom.Xform.Define(session.stage, root).GetPrim()
        body.SetCustomDataByKey(_OWNER, owner)
        body.SetCustomDataByKey('ovphysxCompanion:meshNames', Vt.StringArray(sorted(names)))
        mat = UsdShade.Material.Define(session.stage, root.AppendChild('Material'))
        mat.GetPrim().SetCustomDataByKey(_OWNER, owner)
        _schema(mat.GetPrim(), 'OmniPhysicsDeformableMaterialAPI')
        for name, value in values.items():
            _set(mat.GetPrim(), 'omniphysics:' + name, Sdf.ValueTypeNames.Float, value)
        for mesh, points, elements, rest, rest_elements, bind, embedding in prepared:
            target = root.AppendChild(mesh.name)
            geometry = (UsdGeom.TetMesh.Define(session.stage, target) if mesh.kind == 'tetrahedron'
                        else UsdGeom.Mesh.Define(session.stage, target))
            prim = geometry.GetPrim()
            prim.SetCustomDataByKey(_OWNER, owner)
            prim.SetCustomDataByKey(_VERSION, mesh.topology_version)
            prim.SetCustomDataByKey('ovphysxCompanion:meshRole', mesh.role)
            if not geometry.GetPointsAttr().Set(Vt.Vec3fArray(points)):
                raise RuntimeError('USD rejected deformable points')
            if mesh.kind == 'tetrahedron':
                _put(geometry.GetTetVertexIndicesAttr(), Vt.Vec4iArray(elements))
                _put(geometry.GetSurfaceFaceVertexIndicesAttr(), UsdGeom.TetMesh.ComputeSurfaceFaces(geometry, Usd.TimeCode.Default()))
            else:
                _put(geometry.GetFaceVertexCountsAttr(), Vt.IntArray([3]*len(elements)))
                _put(geometry.GetFaceVertexIndicesAttr(), Vt.IntArray([i for e in elements for i in e]))
                _put(geometry.GetSubdivisionSchemeAttr(), 'none')
            _schema(prim, 'OmniPhysicsDeformablePoseAPI', 'default')
            _set(prim, 'deformablePose:default:omniphysics:purposes', Sdf.ValueTypeNames.TokenArray, Vt.TokenArray(['bindPose']))
            _set(prim, 'deformablePose:default:omniphysics:points', Sdf.ValueTypeNames.Point3fArray, Vt.Vec3fArray(bind))
            if mesh.role == 'simulation':
                _schema(prim, required[-1])
                _set(prim, 'omniphysics:restShapePoints', Sdf.ValueTypeNames.Point3fArray, Vt.Vec3fArray(rest))
                name, value_type, values_type = (('restTetVtxIndices', Sdf.ValueTypeNames.Int4Array, Vt.Vec4iArray)
                    if mesh.kind == 'tetrahedron' else ('restTriVtxIndices', Sdf.ValueTypeNames.Int3Array, Vt.Vec3iArray))
                _set(prim, 'omniphysics:' + name, value_type, values_type(rest_elements))
            if mesh.role == 'collision' or mesh.collision:
                if not UsdPhysics.CollisionAPI.Apply(prim):
                    raise RuntimeError('USD rejected CollisionAPI')
            if mesh.role in ('simulation','collision') or mesh.collision:
                binding = UsdShade.MaterialBindingAPI.Apply(prim)
                if not binding.Bind(mat, materialPurpose='physics'):
                    raise RuntimeError('USD rejected physics material binding')
            if embedding is not None:
                ids, weights = embedding
                prefix = 'ovphysxCompanion:embedding:'
                if not prim.CreateRelationship(prefix + 'simulation', custom=True).SetTargets([root.AppendChild(simulation.name)]):
                    raise RuntimeError('USD rejected embedding source')
                _set(prim, prefix + 'topologyVersion', Sdf.ValueTypeNames.String, simulation.topology_version, custom=True)
                _set(prim, prefix + 'elements', Sdf.ValueTypeNames.IntArray, Vt.IntArray(ids), custom=True)
                width = 4 if simulation.kind == 'tetrahedron' else 3
                value_type = Sdf.ValueTypeNames.Float4Array if width == 4 else Sdf.ValueTypeNames.Float3Array
                array = Vt.Vec4fArray(weights) if width == 4 else Vt.Vec3fArray(weights)
                _set(prim, prefix + 'weights', value_type, array, custom=True)
            elif mesh.role == 'visual':
                prefix = 'ovphysxCompanion:embedding:'
                for name in ('topologyVersion', 'elements', 'weights'):
                    attr = prim.GetAttribute(prefix + name)
                    if attr:
                        attr.Block()
                relation = prim.GetRelationship(prefix + 'simulation')
                if relation and not relation.SetTargets([]):
                    raise RuntimeError('USD rejected embedding invalidation')
        _schema(body, 'OmniPhysicsDeformableBodyAPI')
    return session._apply(layer, edit)


def embedding_from_mapping(mapping):
    """Convert a complete native volume map into USD hierarchy embedding metadata.

    Volume points must be contained: projected outside points are not silently
    accepted as a complete visual binding. Triangle normal-offset bindings need
    their explicit offset-aware representation rather than this volume helper.
    """
    mapping.require_complete()
    if mapping.element_width != 4 or any(distance != 0 for distance in mapping.distances):
        raise ValueError('Expected fully contained tetrahedral embedding')
    return Embedding(mapping.topology_version, mapping.elements, mapping.weights)
