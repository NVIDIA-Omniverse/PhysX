# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

__all__ = ["DeformableSchemaChecker"]

from omni.asset_validator.core import BaseRuleChecker, registerRule, Suggestion
from omni.asset_validator.core.complianceChecker import is_omni_path
from pxr import Usd, UsdPhysics, UsdUtils, UsdGeom, PhysxSchema, Sdf, Gf, UsdShade, Vt, Tf
from typing import Any, List, Dict, Set, Optional, Tuple, Union, Type
import carb
import usdrt
import omni
import omni.physx.bindings._physx as physx_bindings
from omni.physx import acquire_physx_attachment_private_interface
import numpy as np
from .utils import check_timeline_playing


########################################################################################################################
# General schema helpers
########################################################################################################################

def _spec_authors_api(spec: Sdf.PrimSpec, schema: str) -> bool:
    if hasattr(Sdf.PrimSpec, "GetAppliedAPISchemas"): # USD ≥ 24.07
        if schema in spec.GetAppliedAPISchemas():
            return True
    else: # Older builds (≤ 24.06)
        list_op = spec.GetInfo("apiSchemas")
        if not list_op:
            return False
        if isinstance(list_op, Sdf.TokenListOp):
            positive_items = (
                (list_op.explicitItems  or [])
                + (list_op.prependedItems or [])
                + (list_op.appendedItems or [])
                + (list_op.addedItems    or [])
            )
        else: # raw list / tuple
            positive_items = list_op

        if schema in positive_items:
            return True
    return False

def _strip_api_from_spec(spec: Sdf.PrimSpec, schema: str):

    if hasattr(spec, "RemoveAppliedAPISchema"):          # USD ≥ 24.07
        spec.RemoveAppliedAPISchema(schema)
        return

    src_list_op = spec.GetInfo("apiSchemas")
    if not isinstance(src_list_op, Sdf.TokenListOp):
        return

    def _keep(tokens):
        return [t for t in (tokens or []) if t != schema]

    explicit   = _keep(src_list_op.explicitItems)
    prepended  = _keep(src_list_op.prependedItems)
    appended   = _keep(src_list_op.appendedItems)
    added      = _keep(src_list_op.addedItems)      # legacy, still copy
    deleted    = _keep(src_list_op.deletedItems)

    # If *everything* is empty after filtering, wipe the field.
    if not (explicit or prepended or appended or added or deleted):
        spec.ClearInfo("apiSchemas")
        return

    if src_list_op.isExplicit:
        # This call sets the isExplicit flag and clears the other buckets
        dst_list_op = Sdf.TokenListOp.CreateExplicit(explicit)
        # Nothing else to copy – other buckets are ignored anyway
    else:
        dst_list_op = Sdf.TokenListOp()
        if explicit:   dst_list_op.explicitItems  = explicit   # base list
        if prepended:  dst_list_op.prependedItems = prepended
        if appended:   dst_list_op.appendedItems  = appended
        if added:      dst_list_op.addedItems     = added
        if deleted:    dst_list_op.deletedItems   = deleted

    spec.SetInfo("apiSchemas", dst_list_op)

def _get_schema_prop_names(schema_or_type: Union[Tf.Type, Type[Usd.SchemaBase]]) -> Set[str]:
    names: Set[str] = set()

    if isinstance(schema_or_type, Tf.Type):
        reg = Usd.SchemaRegistry()
        is_api = Usd.SchemaRegistry().IsAppliedAPISchema(schema_or_type)
        if is_api:
            prim_def = reg.FindAppliedAPIPrimDefinition(reg.GetAPISchemaTypeName(schema_or_type))
        else:
            prim_def = reg.FindConcretePrimDefinition(reg.GetConcreteSchemaTypeName(schema_or_type))
        if prim_def:
            names = set(prim_def.GetPropertyNames())
    else:
        names = set(schema_or_type.GetSchemaAttributeNames(True))
        if hasattr(schema_or_type, "GetSchemaRelationshipNames"):
            names.update(schema_or_type.GetSchemaRelationshipNames(True))

    return names


########################################################################################################################
# Conversion layer helpers
########################################################################################################################

def _get_layers_to_convert(stage: Usd.Stage) -> Set[Sdf.Layer]:
    """Return a set of SdfLayer handles that belong to the *root* layer stack and session layer"""
    root    = stage.GetRootLayer()
    session = stage.GetSessionLayer()
    out: Set[Sdf.Layer] = set()

    def _recurse(layer):
        if not layer or layer in out:
            return
        for path in layer.subLayerPaths:
            _recurse(Sdf.Layer.FindRelativeToLayer(layer, path))
        if layer.permissionToEdit:
            out.add(layer)

    _recurse(root)
    _recurse(session)
    return out

def _has_schema_api_to_convert(prim: Usd.Prim, schema: str) -> bool:

    if schema not in prim.GetAppliedSchemas():
        return False

    local_layers = _get_layers_to_convert(prim.GetStage())
    for spec in prim.GetPrimStack():                         # strongest → weakest
        if spec.layer not in local_layers:
            continue
        if _spec_authors_api(spec, schema):
            return True
    return False

def _has_schema_type_to_convert(prim: Usd.Prim, schema: str) -> bool:
    if prim.GetTypeName() != schema:
        return False

    stage = prim.GetStage()

    # Climb up the hierarchy so a child prim inside the payload still tests False
    while prim and prim.IsValid():
        filt = Usd.PrimCompositionQuery.Filter()
        filt.arcTypeFilter = Usd.PrimCompositionQuery.ArcTypeFilter.ReferenceOrPayload
        filt.dependencyTypeFilter = Usd.PrimCompositionQuery.DependencyTypeFilter.Direct

        if Usd.PrimCompositionQuery(prim, filt).GetCompositionArcs():
            return False          # found the declaring arc

        prim = prim.GetParent()   # walk upward to the declaring prim

    return True

def _remove_schemas(prim: Usd.Prim, schemas: List[str]):
    if not prim or not prim.IsValid():
        return
    stage = prim.GetStage()
    local_layers = _get_layers_to_convert(stage)
    processed_layers = set()

    # walk weakest → strongest to avoid list-op conflicts
    for spec in reversed(prim.GetPrimStack()):
        layer = spec.layer
        if layer not in local_layers or layer in processed_layers:
            continue

        # edit *every* spec that lives in this layer
        for spec2 in [s for s in prim.GetPrimStack() if s.layer is layer]:
            for schema in schemas:
                # not using prim.RemoveAPI as this seems to result in putting in negative 
                # APIs even if only one layer is involved.
                _strip_api_from_spec(spec2, schema)
            processed_layers.add(spec.layer)

def _remove_properties_by_prefixes(prim: Usd.Prim, prefixes: List[str]) -> None:
    prop_names = [name for name in prim.GetPropertyNames() if any(name.startswith(pfx) for pfx in prefixes)]
    for layer in _get_layers_to_convert(prim.GetStage()):
        with Usd.EditContext(prim.GetStage(), layer):
            for name in prop_names:
                prim.RemoveProperty(name)


########################################################################################################################
# Propery getter utils
########################################################################################################################

def _get_attr_val(old_token: str, old_prim: Usd.Prim):
    if not old_prim:
        return None
    attr = old_prim.GetAttribute(old_token)
    if not attr:
        return None
    if attr.HasAuthoredValue():
        return attr.Get()
    return None

def _get_attr_array(old_token: str, old_prim: Usd.Prim) -> list:
    if not old_prim:
        return []
    attr = old_prim.GetAttribute(old_token)
    if not attr:
        return []
    if attr.HasAuthoredValue():
        return attr.Get()
    return []

def _get_rel_single(old_token: str, old_prim: Usd.Prim) -> Sdf.Path:
    old_rel = old_prim.GetRelationship(old_token)
    if not old_rel:
        return Sdf.Path()

    targets = old_rel.GetTargets()
    if len(targets) == 0:
        return Sdf.Path()

    return targets[0]

def _get_particle_system(particle_prim: Usd.Prim) -> PhysxSchema.PhysxParticleSystem:
    ps_path = _get_rel_single("physxParticle:particleSystem", particle_prim)
    ps_prim = particle_prim.GetStage().GetPrimAtPath(ps_path)
    return PhysxSchema.PhysxParticleSystem(ps_prim)

def _get_bindpose_points(prim: Usd.Prim) -> Vt.Vec3fArray:
    if not prim.HasAPI("OmniPhysicsDeformablePoseAPI", "default"):
        return Vt.Vec3fArray()
    pointsAttr = prim.GetAttribute("deformablePose:default:omniphysics:points")
    if not pointsAttr:
        return Vt.Vec3fArray()
    return pointsAttr.Get()


########################################################################################################################
# Propery conversion utils
########################################################################################################################

def _convert_attr(old_token: str, new_token: str, old_prim: Usd.Prim, new_prim: Usd.Prim, old_default = None):
    value = _get_attr_val(old_token, old_prim)
    if value is not None:
        new_prim.GetAttribute(new_token).Set(value)
    elif old_default is not None:
        new_prim.GetAttribute(new_token).Set(old_default)

def _convert_rel(old_token: str, new_token: str, old_prim: Usd.Prim, new_prim: Usd.Prim, target_map: Optional[Dict] = None):
    old_rel = old_prim.GetRelationship(old_token)
    if not old_rel:
        return

    targets = old_rel.GetTargets()
    if len(targets) == 0:
        return

    new_targets = []
    for target in targets:
        if target_map is not None and target in target_map:
            new_targets.append(target_map[target])
        else:
            new_targets.append(target)
    
    new_rel = new_prim.CreateRelationship(new_token)
    new_rel.SetTargets(new_targets)

def _convert_mat_binding(old_token: str, new_token: str, old_prim: Usd.Prim, new_prim: Usd.Prim):
    new_rel = _convert_rel(old_token, new_token, old_prim, new_prim)
    if not new_rel:
        return

    old_rel = old_prim.GetRelationship(old_token)
    bind_material_as = old_rel.GetMetadata('bindMaterialAs')
    if bind_material_as:
        new_rel.SetMetadata('bindMaterialAs', bind_material_as)

def _add_bindpose(point_based: UsdGeom.PointBased, rest_points: Vt.Vec3fArray) -> None:      
    point_based.GetPrim().ApplyAPI("OmniPhysicsDeformablePoseAPI", "default")
    purposes = ["bindPose"]
    point_based.GetPrim().CreateAttribute("deformablePose:default:omniphysics:points", Sdf.ValueTypeNames.Point3fArray).Set(rest_points)
    point_based.GetPrim().CreateAttribute("deformablePose:default:omniphysics:purposes", Sdf.ValueTypeNames.TokenArray).Set(purposes)

########################################################################################################################
# Geometry utils
########################################################################################################################

def _triangulate_mesh(faceVertexCounts: Vt.IntArray, faceVertexIndices: Vt.IntArray):
    old_face_vertex_counts = list(faceVertexCounts)
    old_face_vertex_indices = list(faceVertexIndices)
    new_face_vertex_counts = []
    new_face_vertex_indices = []
    is_tri_face = all(count == 3 for count in old_face_vertex_counts)
    if is_tri_face:
        new_face_vertex_counts = old_face_vertex_counts
        new_face_vertex_indices = old_face_vertex_indices
    else:
        # Fan triangulation for any polygon with ≥3 vertices
        offset = 0
        for count in old_face_vertex_counts:
            if count < 3:
                offset += count
                continue

            v0 = old_face_vertex_indices[offset]
            for i in range(1, count - 1):
                v1 = old_face_vertex_indices[offset + i]
                v2 = old_face_vertex_indices[offset + i + 1]

                new_face_vertex_counts.append(3)
                new_face_vertex_indices.extend([v0, v1, v2])

            offset += count

    return new_face_vertex_counts, new_face_vertex_indices

def _weld_tri_mesh(
    points: Vt.Vec3fArray,
    rest_points: Vt.Vec3fArray,
    sim_indices: Vt.IntArray
) -> Tuple[Vt.Vec3fArray, Vt.Vec3fArray, Vt.IntArray]:
    """
    In-place “weld” of a triangle mesh.
    """
    if len(points) != len(rest_points) or len(sim_indices) % 3:
        return points, rest_points, sim_indices

    key_to_new = {}
    old_to_new = [None] * len(points)
    new_points, new_rest = [], []

    for old_i, p in enumerate(points):
        key = (float(p[0]), float(p[1]), float(p[2]))  # hashable
        idx = key_to_new.setdefault(key, len(new_points))
        if idx == len(new_points):          # first time we see this vertex
            new_points.append(p)
            new_rest.append(rest_points[old_i])
        old_to_new[old_i] = idx

    # remap triangle indices
    new_sim = [old_to_new[i] for i in sim_indices]

    return (
        type(points)(new_points),
        type(rest_points)(new_rest),
        type(sim_indices)(new_sim)
    )

def _find_surface_triangle_indices_for_tets(
    tetVertexIndices: list[Gf.Vec4i],
    surfaceVertexIndices: list[Gf.Vec3i],
    tetIndices: list[int]
) -> list[int]:
    """
    Given a tet mesh and a surface triangle list, return a sorted list
    of unique surface triangle indices that belong to the specified tets.
    """
    # Map triangle face sets to their index
    tri_set_to_index = {
        frozenset((tri[0], tri[1], tri[2])): i
        for i, tri in enumerate(surfaceVertexIndices)
    }

    def get_faces_of_tet(tet):
        a, b, c, d = tet
        return [
            frozenset((a, b, c)),
            frozenset((a, b, d)),
            frozenset((a, c, d)),
            frozenset((b, c, d)),
        ]

    matched_indices = set()
    for ti in tetIndices:
        tet = tetVertexIndices[ti]
        for face in get_faces_of_tet(tet):
            if face in tri_set_to_index:
                matched_indices.add(tri_set_to_index[face])

    return sorted(matched_indices)

def _find_triangle_indices_for_vertices(
    triVertexIndices: list[Gf.Vec3i],
    vertexIndices: list[int]
) -> list[int]:
    """
    Given a triangle mesh and a set of vertex indices, return a sorted list
    of unique triangle indices that include any of those vertices.
    """
    vertex_set = set(vertexIndices)
    matching_tri_indices = []

    for i, tri in enumerate(triVertexIndices):
        if vertex_set.intersection((tri[0], tri[1], tri[2])):
            matching_tri_indices.append(i)

    return sorted(matching_tri_indices)


########################################################################################################################
# USDRT helpers
########################################################################################################################

def _get_or_insert_stage_id(stage: Usd.Stage) -> int:
    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    if stage_id == -1:
        stage_id = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()
    return stage_id
    if stage_id == -1:
        stage_id = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()
    return stage_id
    if stage_id == -1:
        stage_id = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()
    return stage_id

def _get_prim_paths_with_specified_api(stage: Usd.Stage, schema: str) -> list:
    stage_id = _get_or_insert_stage_id(stage) 
    usdrt_stage = usdrt.Usd.Stage.Attach(stage_id)

    if usdrt_stage:
        return usdrt_stage.GetPrimsWithAppliedAPIName(schema)
    else:
        return []

def _get_prim_paths_with_specified_type_name(stage: Usd.Stage, type_name: str) -> list:
    stage_id = _get_or_insert_stage_id(stage)     
    usdrt_stage = usdrt.Usd.Stage.Attach(stage_id)

    if usdrt_stage:
        return usdrt_stage.GetPrimsWithTypeName(type_name)
    else:
        return []   


########################################################################################################################
########################################################################################################################
# BaseRuleChecker implementation
########################################################################################################################
########################################################################################################################

@registerRule("Omni:Physx")
class DeformableSchemaChecker(BaseRuleChecker):
    
    physx_attachment = acquire_physx_attachment_private_interface()

    # discovered paths to process
    physxParticleCloth_paths: Set = set()
    physxPBDMaterial_cloth_paths: Set = set()
    physxDeformableBodyMaterial_paths: Set = set()
    physxDeformableSurfaceMaterial_paths: Set = set()
    physxDeformableBody_paths: Set = set()
    physxDeformableSurface_paths: Set = set()
    physxPhysicsAttachment_paths: Set = set()
    tetrahedalMesh_paths: Set = set()

    # maps
    root_to_sim_path_map: Dict = {}                          # map from old paths to new sim mesh paths
    root_to_coll_path_map: Dict = {}                         # map from old paths to new collision mesh paths


    @staticmethod
    def clear():
        DeformableSchemaChecker.physxParticleCloth_paths = set()
        DeformableSchemaChecker.physxPBDMaterial_cloth_paths = set()
        DeformableSchemaChecker.physxDeformableBodyMaterial_paths = set()
        DeformableSchemaChecker.physxDeformableSurfaceMaterial_paths = set()
        DeformableSchemaChecker.physxDeformableBody_paths = set()
        DeformableSchemaChecker.physxDeformableSurface_paths = set()
        DeformableSchemaChecker.physxPhysicsAttachment_paths = set()
        DeformableSchemaChecker.tetrahedalMesh_paths = set()
        DeformableSchemaChecker.root_to_sim_path_map = {}
        DeformableSchemaChecker.root_to_coll_path_map = {}


    @staticmethod
    def clone_and_clean_skin_mesh(root_prim: Usd.Prim) -> UsdGeom.Mesh:
        stage = root_prim.GetStage()
        skin_mesh_path = root_prim.GetPath().AppendChild("skinMesh")
        
        with Usd.EditContext(stage, stage.GetRootLayer()):
            omni.usd.duplicate_prim(stage, root_prim.GetPath(), skin_mesh_path)

        skin_mesh = UsdGeom.Mesh.Get(stage, skin_mesh_path)
        skin_prim = skin_mesh.GetPrim()

        DeformableSchemaChecker.clean_old_deformable_apis_and_attrs(skin_prim)

        remove_prefixes = ["xformOp", "xformOpOrder", "velocities", "material:binding:physics"]
        _remove_properties_by_prefixes(skin_prim, remove_prefixes)

        # Remove cloned PhysxPhysicsAttachment from within the new copied render mesh
        traversal = iter(Usd.PrimRange(skin_prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)))
        attachments_to_remove = []
        for prim in traversal:
            if prim.GetTypeName() == "PhysxPhysicsAttachment":
                attachments_to_remove.append(prim.GetPath())

        for attachment_path in attachments_to_remove:
            stage.RemovePrim(attachment_path)

        return skin_mesh

    @staticmethod
    def retype_mesh_to_xform(
            stage: Usd.Stage,
            prim_path: Sdf.Path
    ) -> Usd.Prim:
        prim = stage.GetPrimAtPath(prim_path)
        mesh = UsdGeom.Mesh(prim)
        if not mesh:
            return Usd.Xform()

        xform = UsdGeom.Xform.Define(stage, prim_path)
        if not xform:
            return Usd.Xform()

        old_props = _get_schema_prop_names(UsdGeom.Mesh)
        new_props = _get_schema_prop_names(UsdGeom.Xform)
        remove_props = old_props - new_props
        if not remove_props:
            return Usd.Xform()

        _remove_properties_by_prefixes(prim, list(remove_props) + ["primvars"])

        prim.GetPayloads().ClearPayloads()
        prim.GetReferences().ClearReferences()

        return xform

  
    @staticmethod
    def get_or_create_world_transform(stage: Usd.Stage) -> UsdGeom.Xformable:
        # Check default prim first
        default_prim = stage.GetDefaultPrim()
        default_xf = UsdGeom.Xformable(default_prim)
        if default_xf and not default_xf.GetOrderedXformOps():
            return default_xf

        # Check other top-level Xformables with identity local ops
        for child in stage.GetPseudoRoot().GetChildren():
            child_xf = UsdGeom.Xformable(child)
            if child_xf and not child_xf.GetOrderedXformOps():
                return child_xf

        # Create new
        world_path = Sdf.Path(omni.usd.get_stage_next_free_path(stage, "/World", False))
        world_xform = UsdGeom.Xform.Define(stage, world_path)
        return world_xform
       

    @staticmethod
    def clean_old_material_apis_and_attrs(material_prim: Usd.Prim):

        prefixes = ["physxDeformableBodyMaterial:", "physxDeformableSurfaceMaterial:",
                    "physxPBDMaterial:lift", "physxPBDMaterial:drag"]
        _remove_properties_by_prefixes(material_prim, prefixes)

        schemas = ["PhysxDeformableBodyMaterialAPI", "PhysxDeformableSurfaceMaterialAPI"]
        _remove_schemas(material_prim, schemas)


    @staticmethod
    def convert_PhysxPBDMaterial(stage: Usd.Stage, path: Sdf.Path):
        prim = stage.GetPrimAtPath(path)
        if not _has_schema_api_to_convert(prim, "PhysxPBDMaterialAPI"):
            return

        prim.ApplyAPI("OmniPhysicsDeformableMaterialAPI")
        prim.ApplyAPI("OmniPhysicsSurfaceDeformableMaterialAPI")
        prim.ApplyAPI("PhysxDeformableMaterialAPI")
        prim.ApplyAPI("PhysxSurfaceDeformableMaterialAPI")

        mpu = UsdGeom.GetStageMetersPerUnit(stage)
        kpu = UsdPhysics.GetStageKilogramsPerUnit(stage)

        _convert_attr("physxPBDMaterial:density", "omniphysics:density", prim, prim)
        _convert_attr("physxPBDMaterial:friction", "omniphysics:dynamicFriction", prim, prim, 0.25)
        _convert_attr("physxPBDMaterial:friction", "omniphysics:staticFriction", prim, prim, 0.25)
        _convert_attr("nonexistent", "omniphysics:poissonsRatio", prim, prim, 0.0)
        _convert_attr("nonexistent", "omniphysics:youngsModulus", prim, prim, 10000000.0*mpu/kpu)
        _convert_attr("nonexistent", "omniphysics:surfaceThickness", prim, prim, 0.005/mpu)

        DeformableSchemaChecker.clean_old_material_apis_and_attrs(prim)


    @staticmethod
    def convert_PhysxDeformableSurfaceMaterial(stage: Usd.Stage, path: Sdf.Path):
        prim = stage.GetPrimAtPath(path)
        if not _has_schema_api_to_convert(prim, "PhysxDeformableSurfaceMaterialAPI"):
            return

        prim.ApplyAPI("OmniPhysicsDeformableMaterialAPI")
        prim.ApplyAPI("OmniPhysicsSurfaceDeformableMaterialAPI")
        prim.ApplyAPI("PhysxDeformableMaterialAPI")
        prim.ApplyAPI("PhysxSurfaceDeformableMaterialAPI")

        mpu = UsdGeom.GetStageMetersPerUnit(stage)
        kpu = UsdPhysics.GetStageKilogramsPerUnit(stage)

        _convert_attr("physxDeformableSurfaceMaterial:density", "omniphysics:density", prim, prim)
        _convert_attr("physxDeformableSurfaceMaterial:dynamicFriction", "omniphysics:dynamicFriction", prim, prim, 0.25)
        _convert_attr("physxDeformableSurfaceMaterial:dynamicFriction", "omniphysics:staticFriction", prim, prim, 0.25)
        _convert_attr("physxDeformableSurfaceMaterial:poissonsRatio", "omniphysics:poissonsRatio", prim, prim, 0.0)
        _convert_attr("physxDeformableSurfaceMaterial:youngsModulus", "omniphysics:youngsModulus", prim, prim, 10000000.0*mpu/kpu)
        _convert_attr("physxDeformableSurfaceMaterial:thickness", "omniphysics:surfaceThickness", prim, prim, 0.005/mpu)

        DeformableSchemaChecker.clean_old_material_apis_and_attrs(prim)


    @staticmethod
    def convert_PhysxDeformableBodyMaterial(stage: Usd.Stage, path: Sdf.Path):
        prim = stage.GetPrimAtPath(path)
        if not _has_schema_api_to_convert(prim, "PhysxDeformableBodyMaterialAPI"):
            return

        prim.ApplyAPI("OmniPhysicsDeformableMaterialAPI")
        prim.ApplyAPI("PhysxDeformableMaterialAPI")

        mpu = UsdGeom.GetStageMetersPerUnit(stage)
        kpu = UsdPhysics.GetStageKilogramsPerUnit(stage)

        _convert_attr("physxDeformableBodyMaterial:density", "omniphysics:density", prim, prim)
        _convert_attr("physxDeformableBodyMaterial:dynamicFriction", "omniphysics:dynamicFriction", prim, prim, 0.25)
        _convert_attr("physxDeformableBodyMaterial:dynamicFriction", "omniphysics:staticFriction", prim, prim, 0.25)
        _convert_attr("physxDeformableBodyMaterial:poissonsRatio", "omniphysics:poissonsRatio", prim, prim, 0.45)
        _convert_attr("physxDeformableBodyMaterial:youngsModulus","omniphysics:youngsModulus", prim, prim, 50000000.0*mpu/kpu)
        _convert_attr("physxDeformableBodyMaterial:elasticityDamping", "physxDeformableMaterial:elasticityDamping", prim, prim, 0.005)

        DeformableSchemaChecker.clean_old_material_apis_and_attrs(prim)


    @staticmethod
    def clean_old_deformable_apis_and_attrs(deformable_prim: Usd.Prim):

        prefixes = ["physxAutoParticleCloth:", 
                    "physxDeformable:",
                    "physxParticle:",
                    "physxCollision:",
                    "physics:mass"]
        _remove_properties_by_prefixes(deformable_prim, prefixes)

        schemas = ["PhysxParticleClothAPI", "PhysxAutoParticleClothAPI", "PhysxParticleAPI",
                   "PhysxDeformableSurfaceAPI", "PhysxDeformableBodyAPI", "PhysxDeformableAPI",
                   "PhysxCollisionAPI", "PhysicsMassAPI"]
        _remove_schemas(deformable_prim, schemas)


    @staticmethod
    def convert_PhysxParticleCloth(stage: Usd.Stage, path: Sdf.Path):
        prim = stage.GetPrimAtPath(path)
        if not _has_schema_api_to_convert(prim, "PhysxParticleClothAPI"):
            return

        old_mesh = UsdGeom.Mesh(prim)
        if not old_mesh:
            return

        points = old_mesh.GetPointsAttr().Get()
        if len(points) == 0:
            return     

        rest_points = _get_attr_array("physxParticle:restPoints", prim)
        if len(rest_points) != len(points):
            rest_points = points

        face_vertex_counts = old_mesh.GetFaceVertexCountsAttr().Get()
        face_vertex_indices = old_mesh.GetFaceVertexIndicesAttr().Get()

        # Copy render mesh and apply bind pose
        skin_mesh = DeformableSchemaChecker.clone_and_clean_skin_mesh(prim)
        _add_bindpose(skin_mesh, rest_points)

        # Retype UsdGeom.Mesh to UsdGeom.Xform
        xform = DeformableSchemaChecker.retype_mesh_to_xform(stage, path)
        if not xform:
            return

        # Setup deformable root
        prim.ApplyAPI("OmniPhysicsDeformableBodyAPI")
        prim.ApplyAPI("PhysxBaseDeformableBodyAPI")
        prim.ApplyAPI("PhysxSurfaceDeformableBodyAPI")
        prim.ApplyAPI("MaterialBindingAPI")

        # Convert mass property, ignoring density, because we would need to store it into the material which might 
        # be problematic due to material sharing.
        _convert_attr("physics:mass", "omniphysics:mass", prim, prim)

        # Convert attributes of old PhysxParticleClothAPI
        # Ignoring "physxParticle:selfCollisionFilter" and just using defoult self collision filtering
        _convert_attr("physxParticle:selfCollision", "physxDeformableBody:selfCollision", prim, prim)

        # Create simulation mesh under root
        sim_mesh_path = path.AppendChild("simMesh")
        sim_mesh = UsdGeom.Mesh.Define(stage, sim_mesh_path)
        if not sim_mesh:
            return

        DeformableSchemaChecker.root_to_sim_path_map[path] = sim_mesh_path
        DeformableSchemaChecker.root_to_coll_path_map[path] = sim_mesh_path

        sim_mesh_tri_vtx_counts = []
        sim_mesh_tri_vtx_indices = []
        sim_mesh_points = []
        sim_mesh_rest_points = []

        welded_triangle_indices = _get_attr_array("physxParticle:weldedTriangleIndices", prim)
        welded_vertices_remap_to_orig = _get_attr_array("physxParticle:weldedVerticesRemapToOrig", prim)

        if len(welded_triangle_indices) > 0:
            if (len(welded_triangle_indices) % 3 == 0 and
                len(welded_vertices_remap_to_orig) > 0 and
                len(welded_vertices_remap_to_orig) < len(points)):
                sim_mesh_tri_vtx_counts = [3] * (len(welded_triangle_indices) // 3)
                sim_mesh_tri_vtx_indices = Vt.IntArray([int(i) for i in welded_triangle_indices])
                sim_mesh_points = [points[i] for i in welded_vertices_remap_to_orig]
                sim_mesh_rest_points = [rest_points[i] for i in welded_vertices_remap_to_orig]
            else:
                return
        else:
            sim_mesh_tri_vtx_counts, sim_mesh_tri_vtx_indices = _triangulate_mesh(face_vertex_counts, face_vertex_indices)
            # Duplicate points are removed, according to the requirements of current surface deformable instantiation.
            # This might be relaxed later on.
            sim_mesh_points, sim_mesh_rest_points, sim_mesh_tri_vtx_indices = _weld_tri_mesh(points, rest_points, sim_mesh_tri_vtx_indices)

        sim_mesh.CreatePointsAttr(sim_mesh_points)
        sim_mesh.CreateFaceVertexCountsAttr(sim_mesh_tri_vtx_counts)
        sim_mesh.CreateFaceVertexIndicesAttr(sim_mesh_tri_vtx_indices)
        sim_mesh_tri_vtx_indices3 = [tup for tup in zip(*[iter(sim_mesh_tri_vtx_indices)] * 3)]

        sim_mesh.GetPrim().ApplyAPI("OmniPhysicsSurfaceDeformableSimAPI")
        sim_mesh.GetPrim().ApplyAPI(UsdPhysics.CollisionAPI)
        sim_mesh.GetPrim().ApplyAPI(PhysxSchema.PhysxCollisionAPI)
        sim_mesh.GetPrim().GetAttribute("omniphysics:restShapePoints").Set(sim_mesh_rest_points)
        sim_mesh.GetPrim().GetAttribute("omniphysics:restTriVtxIndices").Set(sim_mesh_tri_vtx_indices3)

        sim_mesh.CreatePurposeAttr(UsdGeom.Tokens.guide)

        _add_bindpose(sim_mesh.GetPrim(), sim_mesh_rest_points)

        # Convert PhysxParticleSystem attributes
        particle_system = _get_particle_system(prim)
        
        if particle_system:
            particle_system_prim = particle_system.GetPrim()
            _convert_rel("simulationOwner", "omniphysics:simulationOwner", particle_system_prim, prim) 
            _convert_attr("contactOffset", "physxCollision:contactOffset", particle_system_prim, sim_mesh.GetPrim())
            _convert_attr("restOffset", "physxCollision:restOffset", particle_system_prim, sim_mesh.GetPrim())
            _convert_attr("enableCCD", "physxDeformableBody:enableSpeculativeCCD", particle_system_prim, prim)
            _convert_attr("solverPositionIterationCount", "physxDeformableBody:solverPositionIterationCount", particle_system_prim, prim)
            _convert_attr("maxDepenetrationVelocity", "physxDeformableBody:maxDepenetrationVelocity", particle_system_prim, prim)
            _convert_attr("maxVelocity", "physxDeformableBody:maxLinearVelocity", particle_system_prim, prim)
            # Could maybe use physxParticle:springStiffnesses, physxParticle:springDampings, physxParticle:springRestLengths
            # to estimate material parameters better.
            _convert_mat_binding("material:binding:physics", "material:binding:physics", particle_system_prim, prim)

        DeformableSchemaChecker.clean_old_deformable_apis_and_attrs(prim)


    @staticmethod
    def convert_PhysxDeformableSurface(stage: Usd.Stage, path: Sdf.Path):
        prim = stage.GetPrimAtPath(path)
        if not _has_schema_api_to_convert(prim, "PhysxDeformableSurfaceAPI"):
            return

        old_mesh = UsdGeom.Mesh(prim)
        if not old_mesh:
            return

        points = old_mesh.GetPointsAttr().Get()
        if len(points) == 0:
            return

        rest_points = _get_attr_array("physxDeformable:restPoints", prim)
        if len(rest_points) != len(points):
            rest_points = points

        sim_indices = _get_attr_array("physxDeformable:simulationIndices", prim)
        if len(sim_indices) == 0 or len(sim_indices) % 3 != 0:
            return

        # Copy render mesh and apply bind pose
        skin_mesh = DeformableSchemaChecker.clone_and_clean_skin_mesh(prim)
        _add_bindpose(skin_mesh, rest_points)

        # Retype UsdGeom.Mesh to UsdGeom.Xform
        xform = DeformableSchemaChecker.retype_mesh_to_xform(stage, path)
        if not xform:
            return

        # Setup deformable root
        prim.ApplyAPI("OmniPhysicsDeformableBodyAPI")
        prim.ApplyAPI("PhysxBaseDeformableBodyAPI")
        prim.ApplyAPI("PhysxSurfaceDeformableBodyAPI")
        prim.ApplyAPI("MaterialBindingAPI")

        # Convert attributes of old PhysxDeformableAPI
        _convert_attr("physxDeformable:deformableEnabled", "omniphysics:deformableBodyEnabled", prim, prim)
        _convert_attr("physxDeformable:solverPositionIterationCount", "physxDeformableBody:solverPositionIterationCount", prim, prim)
        _convert_attr("physxDeformable:vertexVelocityDamping", "physxDeformableBody:linearDamping", prim, prim, 0.005)
        _convert_attr("physxDeformable:sleepDamping", "physxDeformableBody:settlingDamping", prim, prim)
        _convert_attr("physxDeformable:sleepThreshold", "physxDeformableBody:sleepThreshold", prim, prim)
        _convert_attr("physxDeformable:settlingThreshold", "physxDeformableBody:settlingThreshold", prim, prim)
        _convert_attr("physxDeformable:maxDepenetrationVelocity", "physxDeformableBody:maxDepenetrationVelocity", prim, prim)
        _convert_attr("physxDeformable:selfCollision", "physxDeformableBody:selfCollision", prim, prim)
        _convert_attr("physxDeformable:selfCollisionFilterDistance", "physxDeformableBody:selfCollisionFilterDistance", prim, prim)
        _convert_attr("physxDeformable:enableCCD", "physxDeformableBody:enableSpeculativeCCD", prim, prim)
        _convert_rel("physxDeformable:simulationOwner", "omniphysics:simulationOwner", prim, prim) 

        # Deal with attributes of old PhysxDeformableSurfaceAPI
        _convert_attr("physxDeformableSurface:collisionPairUpdateFrequency", "physxDeformableBody:collisionPairUpdateFrequency", prim, prim)
        _convert_attr("physxDeformableSurface:collisionIterationMultiplier", "physxDeformableBody:collisionIterationMultiplier", prim, prim)
        _convert_attr("physxDeformableSurface:maxVelocity", "physxDeformableBody:maxLinearVelocity", prim, prim)

        # Convert mass property, ignoring density, because we would need to store it into the material which might 
        # be problematic due to material sharing.
        _convert_attr("physics:mass", "omniphysics:mass", prim, prim)

        # Create simulation mesh under root    
        sim_mesh_path = path.AppendChild("simMesh")
        sim_mesh = UsdGeom.Mesh.Define(stage, sim_mesh_path)
        if not sim_mesh:
            return

        DeformableSchemaChecker.root_to_sim_path_map[path] = sim_mesh_path
        DeformableSchemaChecker.root_to_coll_path_map[path] = sim_mesh_path

        # Duplicate points are removed, according to the requirements of current surface deformable instantiation.
        # This might be relaxed later on.
        points, rest_points, sim_indices = _weld_tri_mesh(points, rest_points, sim_indices)

        # Set simulation mesh attributes
        sim_mesh.CreatePointsAttr(points)
        sim_mesh.CreateFaceVertexCountsAttr([3] * (len(sim_indices) // 3))
        sim_mesh.CreateFaceVertexIndicesAttr(sim_indices)
        
        flatteningEnabled = _get_attr_val("physxDeformableSurface:flatteningEnabled", prim)
        rest_bend_angles_default = "restShapeDefault" if flatteningEnabled is None or not flatteningEnabled else "flatDefault"
        sim_mesh.GetPrim().CreateAttribute("omniphysics:restBendAnglesDefault", Sdf.ValueTypeNames.Token).Set(rest_bend_angles_default)

        sim_velocities = _get_attr_array("physxDeformable:simulationVelocities", prim)
        if len(sim_velocities) == len(points):
            sim_mesh.CreateVelocitiesAttr(sim_velocities)

        sim_mesh_tri_vtx_indices = [tup for tup in zip(*[iter(sim_indices)] * 3)]

        sim_mesh.GetPrim().ApplyAPI("OmniPhysicsSurfaceDeformableSimAPI")
        sim_mesh.GetPrim().CreateAttribute("omniphysics:restShapePoints", Sdf.ValueTypeNames.Point3fArray).Set(rest_points)
        sim_mesh.GetPrim().CreateAttribute("omniphysics:restTriVtxIndices", Sdf.ValueTypeNames.Int3Array).Set(sim_mesh_tri_vtx_indices)

        _add_bindpose(sim_mesh, rest_points)
        
        sim_mesh.CreatePurposeAttr(UsdGeom.Tokens.guide)

        # Collision properties
        sim_mesh.GetPrim().ApplyAPI(UsdPhysics.CollisionAPI)
        sim_mesh.GetPrim().ApplyAPI(PhysxSchema.PhysxCollisionAPI)
        _convert_attr("physxCollision:contactOffset", "physxCollision:contactOffset", prim, sim_mesh.GetPrim())
        _convert_attr("physxCollision:restOffset", "physxCollision:restOffset", prim, sim_mesh.GetPrim())

        DeformableSchemaChecker.clean_old_deformable_apis_and_attrs(prim)


    @staticmethod
    def convert_PhysxDeformableBody(stage: Usd.Stage, path: Sdf.Path):
        prim = stage.GetPrimAtPath(path)
        if not _has_schema_api_to_convert(prim, "PhysxDeformableBodyAPI"):
            return

        old_mesh = UsdGeom.Mesh(prim)
        if not old_mesh:
            return

        points = old_mesh.GetPointsAttr().Get()
        if len(points) == 0:
            return

        rest_points = _get_attr_array("physxDeformable:restPoints", prim)
        if len(rest_points) != len(points):
            rest_points = points

        sim_points = _get_attr_array("physxDeformable:simulationPoints", prim)
        sim_rest_points = _get_attr_array("physxDeformable:simulationRestPoints", prim)
        if len(sim_rest_points) != len(sim_points):
            sim_rest_points = sim_points

        if len(sim_points) == 0:
            return

        sim_indices = _get_attr_array("physxDeformable:simulationIndices", prim)
        if len(sim_indices) == 0 or len(sim_indices) % 4 != 0:
            return

        coll_points = _get_attr_array("physxDeformable:collisionPoints", prim)
        coll_rest_points = _get_attr_array("physxDeformable:collisionRestPoints", prim)
        if len(coll_rest_points) != len(coll_points):
            coll_rest_points = coll_points

        if len(coll_points) == 0:
            return

        coll_indices = _get_attr_array("physxDeformable:collisionIndices", prim)
        if len(coll_indices) == 0 or len(coll_indices) % 4 != 0:
            return

        # Copy render mesh and apply bind pose
        skin_mesh = DeformableSchemaChecker.clone_and_clean_skin_mesh(prim)
        _add_bindpose(skin_mesh, rest_points)

        # Retype UsdGeom.Mesh to UsdGeom.Xform
        xform = DeformableSchemaChecker.retype_mesh_to_xform(stage, path)
        if not xform:
            return

        # Setup deformable root
        prim.ApplyAPI("OmniPhysicsDeformableBodyAPI")
        prim.ApplyAPI("PhysxBaseDeformableBodyAPI")
        prim.ApplyAPI("MaterialBindingAPI")

        # Convert attributes of old PhysxDeformableAPI
        _convert_attr("physxDeformable:deformableEnabled", "omniphysics:deformableBodyEnabled", prim, prim)
        _convert_attr("physxDeformable:solverPositionIterationCount", "physxDeformableBody:solverPositionIterationCount", prim, prim)
        _convert_attr("physxDeformable:vertexVelocityDamping", "physxDeformableBody:linearDamping", prim, prim, 0.005)
        _convert_attr("physxDeformable:sleepDamping", "physxDeformableBody:settlingDamping", prim, prim)
        _convert_attr("physxDeformable:sleepThreshold", "physxDeformableBody:sleepThreshold", prim, prim)
        _convert_attr("physxDeformable:settlingThreshold", "physxDeformableBody:settlingThreshold", prim, prim)
        _convert_attr("physxDeformable:maxDepenetrationVelocity", "physxDeformableBody:maxDepenetrationVelocity", prim, prim)
        _convert_attr("physxDeformable:selfCollision", "physxDeformableBody:selfCollision", prim, prim)
        _convert_attr("physxDeformable:selfCollisionFilterDistance", "physxDeformableBody:selfCollisionFilterDistance", prim, prim)
        _convert_attr("physxDeformable:enableCCD", "physxDeformableBody:enableSpeculativeCCD", prim, prim)
        _convert_rel("physxDeformable:simulationOwner", "omniphysics:simulationOwner", prim, prim) 

        # Convert attributes of old PhysxDeformableBodyAPI
        _convert_attr("physxDeformable:disableGravity", "physxDeformableBody:disableGravity", prim, prim)

        # Convert mass property, ignoring density, because we would need to store it into the material which might 
        # be problematic due to material sharing.
        _convert_attr("physics:mass", "omniphysics:mass", prim, prim)

        # Create simulation mesh under root
        sim_mesh_path = path.AppendChild("simMesh")
        sim_mesh = UsdGeom.TetMesh.Define(stage, sim_mesh_path)
        if not sim_mesh:
            return

        DeformableSchemaChecker.root_to_sim_path_map[path] = sim_mesh_path

        # Set simulation mesh attributes
        sim_mesh.CreatePointsAttr(sim_points)
        sim_mesh_tet_vtx_indices = [tup for tup in zip(*[iter(sim_indices)] * 4)]
        sim_mesh.CreateTetVertexIndicesAttr(sim_mesh_tet_vtx_indices)
        sim_mesh.CreatePurposeAttr(UsdGeom.Tokens.guide)
        sim_velocities = _get_attr_val("physxDeformable:simulationVelocities", prim)
        if sim_velocities is not None and len(sim_velocities) == len(sim_points):
            sim_mesh.CreateVelocitiesAttr(sim_velocities)

        sim_mesh.GetPrim().ApplyAPI("OmniPhysicsVolumeDeformableSimAPI")
        sim_mesh.GetPrim().GetAttribute("omniphysics:restShapePoints").Set(sim_rest_points)
        sim_mesh.GetPrim().GetAttribute("omniphysics:restTetVtxIndices").Set(sim_mesh_tet_vtx_indices)

        _add_bindpose(sim_mesh, sim_rest_points)

        # Determine whether sim mesh should be equal to coll mesh
        coll_mesh_tet_vtx_indices = [tup for tup in zip(*[iter(coll_indices)] * 4)]
        sim_mesh_is_coll_mesh = False
        if sim_rest_points == coll_rest_points and sim_mesh_tet_vtx_indices == coll_mesh_tet_vtx_indices:
            sim_mesh_is_coll_mesh = True

        if sim_mesh_is_coll_mesh:
            coll_mesh_path = sim_mesh_path
            coll_mesh = sim_mesh
        else:
            # Create collision mesh under root
            coll_mesh_path = path.AppendChild("collMesh")
            coll_mesh = UsdGeom.TetMesh.Define(stage, coll_mesh_path)
            if not coll_mesh:
                return

            # Set collision mesh attributes
            coll_mesh.CreatePointsAttr(coll_points)
            coll_mesh.CreateTetVertexIndicesAttr(coll_mesh_tet_vtx_indices)
            coll_mesh.CreatePurposeAttr(UsdGeom.Tokens.guide)

            _add_bindpose(coll_mesh, coll_rest_points)

        DeformableSchemaChecker.root_to_coll_path_map[path] = coll_mesh_path
        surfaceFaceVertexIndices = UsdGeom.TetMesh.ComputeSurfaceFaces(coll_mesh, Usd.TimeCode.Default())
        coll_mesh.CreateSurfaceFaceVertexIndicesAttr(surfaceFaceVertexIndices)

        coll_mesh.GetPrim().ApplyAPI(UsdPhysics.CollisionAPI)
        coll_mesh.GetPrim().ApplyAPI(PhysxSchema.PhysxCollisionAPI)

        _convert_attr("physxCollision:contactOffset", "physxCollision:contactOffset", prim, coll_mesh.GetPrim())
        _convert_attr("physxCollision:restOffset", "physxCollision:restOffset", prim, coll_mesh.GetPrim())

        # Convert auto functionality, currently disabled
        # tet_mesh_crc = _get_attr_array("physxDeformable:tetMeshCrc",prim)
        # if len(tet_mesh_crc) > 0:
        #    prim.ApplyAPI("PhysxAutoDeformableBodyAPI")
        #    prim.GetAttribute("physxDeformableBody:autoDeformableBodyEnabled").Set(True)
        #    prim.GetRelationship("physxDeformableBody:cookingSourceMesh").SetTargets([skin_mesh.GetPrim().GetPath()])
        #    sim_res = _get_attr_val("physxDeformable:simulationHexahedralResolution", prim)
        #    if sim_res is not None and sim_res > 0:
        #        prim.ApplyAPI("PhysxAutoDeformableHexahedralMeshAPI")
        #        prim.GetAttribute("physxDeformableBody:resolution").Set(sim_res)
        #    simp_enabled = _get_attr_val("physxDeformable:collisionSimplification", prim)
        #    if simp_enabled is not None and simp_enabled == True:
        #        prim.ApplyAPI("PhysxAutoDeformableMeshSimplificationAPI")
        #        prim.GetAttribute("physxDeformableBody:autoDeformableMeshSimplificationEnabled").Set(True)
        #        _convert_attr("physxDeformable:collisionSimplificationRemeshing", "physxDeformableBody:remeshingEnabled", prim, prim)
        #        _convert_attr("physxDeformable:collisionSimplificationRemeshingResolution", "physxDeformableBody:remeshingResolution", prim, prim)
        #        _convert_attr("physxDeformable:collisionSimplificationTargetTriangleCount", "physxDeformableBody:targetTriangleCount", prim, prim)
        #        _convert_attr("physxDeformable:collisionSimplificationForceConforming", "physxDeformableBody:forceConforming", prim, prim)

        DeformableSchemaChecker.clean_old_deformable_apis_and_attrs(prim)


    @staticmethod
    def convert_TetrahedralMesh(stage: Usd.Stage, path: Sdf.Path):
        prim = stage.GetPrimAtPath(path)
        if not prim or prim.GetTypeName() != "TetrahedralMesh":
            return

        indices = _get_attr_array("indices", prim)
        if len(indices) == 0 or len(indices) % 4 != 0:
            return

        tetmesh = UsdGeom.TetMesh.Define(stage, path)
        if not tetmesh:
            return

        tetmesh.GetTetVertexIndicesAttr().Set([tup for tup in zip(*[iter(indices)] * 4)])

        _remove_properties_by_prefixes(prim, ["indices"])


    @staticmethod
    def convert_DeformableTriMeshRigidAttachment(
        prim: Usd.Prim,
        deformable_prim: Usd.Prim, deformableSlot: int,
        xformable: UsdGeom.Xformable
    ) -> None:
        """
        Converts a PhysxPhysicsAttachment for PhysxParticleClothAPI/PhysxDeformableSurfaceAPI - Rigid actors
        """

        stage = prim.GetStage()
        sim_mesh_path = DeformableSchemaChecker.root_to_sim_path_map[deformable_prim.GetPath()]
        sim_mesh_prim = stage.GetPrimAtPath(sim_mesh_path)
        sim_mesh = UsdGeom.Mesh(sim_mesh_prim)
        if not sim_mesh_prim or not sim_mesh:
            return

        # Create new Vtx Xform Attachment
        new_vtx_xform_attachment_path = prim.GetPath().AppendChild("vtx_xform_attachment")
        new_vtx_xform_attachment = stage.DefinePrim(new_vtx_xform_attachment_path, "OmniPhysicsVtxXformAttachment")
        if not new_vtx_xform_attachment:
            return

        new_prim = new_vtx_xform_attachment.GetPrim()
        new_prim.GetAttribute("omniphysics:attachmentEnabled").Set(True)

        _convert_rel(f"actor{deformableSlot}", "omniphysics:src0", prim, new_prim, DeformableSchemaChecker.root_to_sim_path_map)
        rigid_rel = _get_rel_single(f"actor{1-deformableSlot}", prim)
        if rigid_rel == Sdf.Path():
            new_prim.GetRelationship("omniphysics:src1").SetTargets([xformable.GetPrim().GetPath()])
        else:
            _convert_rel(f"actor{1-deformableSlot}", "omniphysics:src1", prim, new_prim)

        rigid_attachment_points = _get_attr_array(f"points{1-deformableSlot}", prim)
        deformable_attachment_points = _get_attr_array(f"points{deformableSlot}", prim)
        if len(rigid_attachment_points) != len(deformable_attachment_points):
            return

        # vtx points correspond to new sim mesh bind points, which happen to correspond to the rest points,
        # which have been resolved regarding welding (for PhysxParticleClothAPI).
        bind_points = _get_bindpose_points(sim_mesh_prim)
        if (len(bind_points) == 0):
            return

        point_finder = DeformableSchemaChecker.physx_attachment.create_point_finder(bind_points)
        output = DeformableSchemaChecker.physx_attachment.points_to_indices(point_finder, deformable_attachment_points)
        deformable_vtx_indices = output["indices"]
        DeformableSchemaChecker.physx_attachment.release_point_finder(point_finder)
        if len(deformable_vtx_indices) != len(deformable_attachment_points):
            return

        new_prim.GetAttribute("omniphysics:vtxIndicesSrc0").Set(deformable_vtx_indices)
        new_prim.GetAttribute("omniphysics:localPositionsSrc1").Set(rigid_attachment_points)

        # Create new Element Collision Filter
        if rigid_rel != Sdf.Path():
            new_element_collision_filter_path = prim.GetPath().AppendChild("element_collision_filter")
            new_element_collision_filter = stage.DefinePrim(new_element_collision_filter_path, "OmniPhysicsElementCollisionFilter")
            if not new_element_collision_filter:
                return

            new_prim = new_element_collision_filter.GetPrim()
            new_prim.GetAttribute("omniphysics:filterEnabled").Set(True)

            _convert_rel(f"actor{deformableSlot}", "omniphysics:src0", prim, new_prim, DeformableSchemaChecker.root_to_coll_path_map)
            _convert_rel(f"actor{1-deformableSlot}", "omniphysics:src1", prim, new_prim)

            deformable_filter_type = _get_attr_val(f"filterType{deformableSlot}", prim)
            if not deformable_filter_type or not deformable_filter_type == "Vertices":
                return

            deformable_filter_indices = _get_attr_array(f"collisionFilterIndices{deformableSlot}", prim)

            face_vertex_indices = sim_mesh.GetFaceVertexIndicesAttr().Get()
            face_vertex_indices3 = [tup for tup in zip(*[iter(face_vertex_indices)] * 3)]

            # This will fail for welded vertices, but apparently attachments with welded meshes causes a crash.
            # Given that PhysxDeformableSurface was never officially released, we'll let that one slide.
            deformable_tri_indices = _find_triangle_indices_for_vertices(face_vertex_indices3, deformable_filter_indices)

            new_element_collision_filter.GetPrim().GetAttribute("omniphysics:groupElemCounts0").Set([len(deformable_tri_indices)])
            new_element_collision_filter.GetPrim().GetAttribute("omniphysics:groupElemIndices0").Set(deformable_tri_indices)


    @staticmethod
    def convert_DeformableBodyRigidAttachment(
        prim: Usd.Prim,
        deformable_prim: Usd.Prim, deformableSlot: int,
        xformable: UsdGeom.Xformable
    ) -> None:

        stage = prim.GetStage()

        sim_mesh_path = DeformableSchemaChecker.root_to_sim_path_map[deformable_prim.GetPath()]
        sim_mesh_prim = stage.GetPrimAtPath(sim_mesh_path)
        sim_mesh = UsdGeom.TetMesh(sim_mesh_prim)
        if not sim_mesh_prim or not sim_mesh:
            return

        coll_mesh_path = DeformableSchemaChecker.root_to_coll_path_map[deformable_prim.GetPath()]
        coll_mesh_prim = stage.GetPrimAtPath(coll_mesh_path)
        coll_mesh = UsdGeom.TetMesh(coll_mesh_prim)
        if not coll_mesh_prim or not coll_mesh:
            return

        new_tet_xform_attachment_path = prim.GetPath().AppendChild("tet_xform_attachment")
        new_tet_xform_attachment = stage.DefinePrim(new_tet_xform_attachment_path, "OmniPhysicsTetXformAttachment")
        if not new_tet_xform_attachment:
            return

        new_prim = new_tet_xform_attachment.GetPrim()
        new_prim.GetAttribute("omniphysics:attachmentEnabled").Set(True)

        _convert_rel(f"actor{deformableSlot}", "omniphysics:src0", prim, new_prim, DeformableSchemaChecker.root_to_sim_path_map)
        rigid_rel = _get_rel_single(f"actor{1-deformableSlot}", prim)
        if rigid_rel == Sdf.Path():
            new_prim.GetRelationship("omniphysics:src1").SetTargets([xformable.GetPrim().GetPath()])
        else:
            _convert_rel(f"actor{1-deformableSlot}", "omniphysics:src1", prim, new_prim)

        rigid_attachment_points = _get_attr_array(f"points{1-deformableSlot}", prim)
        deformable_coll_attachment_points = _get_attr_array(f"points{deformableSlot}", prim)
        if len(rigid_attachment_points) != len(deformable_coll_attachment_points):
            return

        # Convert points from collision mesh space to simulation mesh space
        coll_to_world = coll_mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        sim_to_world = sim_mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        coll_to_sim = coll_to_world * sim_to_world.GetInverse()
        deformable_sim_attachment_points = [coll_to_sim.Transform(p) for p in deformable_coll_attachment_points]        

        # Map points in sim space to tets and barycentric coordinates
        sim_mesh_bind_points = _get_bindpose_points(sim_mesh_prim)
        sim_mesh_indices = [i for tet in sim_mesh.GetTetVertexIndicesAttr().Get() for i in tet]
        if (len(sim_mesh_bind_points) == 0 or len(sim_mesh_indices) == 0):
            return

        tet_finder_sim = DeformableSchemaChecker.physx_attachment.create_tet_finder(sim_mesh_bind_points, sim_mesh_indices)
        output = DeformableSchemaChecker.physx_attachment.points_to_tetmesh_local(tet_finder_sim, deformable_sim_attachment_points)
        sim_mesh_tet_ids = output["tet_ids"]
        sim_mesh_barycentric_coords = output["barycentric_coords"]
        DeformableSchemaChecker.physx_attachment.release_tet_finder(tet_finder_sim)
        
        if (len(sim_mesh_tet_ids) != len(deformable_sim_attachment_points) or 
            len(sim_mesh_barycentric_coords) != len(deformable_sim_attachment_points)):
            return

        #filter out attachment points outside of collision mesh
        triples = [(i, b[:3], r) for i, b, r in zip(sim_mesh_tet_ids, sim_mesh_barycentric_coords, rigid_attachment_points) if i != -1]
        if len(triples) > 0:
            sim_mesh_tet_ids, sim_mesh_barycentric_coords, rigid_attachment_points = map(list, zip(*triples))
            new_prim.GetAttribute("omniphysics:tetIndicesSrc0").Set(sim_mesh_tet_ids)
            new_prim.GetAttribute("omniphysics:tetCoordsSrc0").Set(sim_mesh_barycentric_coords)
            new_prim.GetAttribute("omniphysics:localPositionsSrc1").Set(rigid_attachment_points)

        # Create new Element Collision Filter
        if rigid_rel != Sdf.Path():
            new_element_collision_filter_path = prim.GetPath().AppendChild("element_collision_filter")
            new_element_collision_filter = stage.DefinePrim(new_element_collision_filter_path, "OmniPhysicsElementCollisionFilter")
            if not new_element_collision_filter:
                return

            new_prim = new_element_collision_filter.GetPrim()
            new_prim.GetAttribute("omniphysics:filterEnabled").Set(True)

            _convert_rel(f"actor{deformableSlot}", "omniphysics:src0", prim, new_prim, DeformableSchemaChecker.root_to_coll_path_map)
            _convert_rel(f"actor{1-deformableSlot}", "omniphysics:src1", prim, new_prim)

            deformable_filter_type = _get_attr_val(f"filterType{deformableSlot}", prim)
            if not deformable_filter_type or not deformable_filter_type == "Geometry":
                return

            deformable_filter_indices = _get_attr_array(f"collisionFilterIndices{deformableSlot}", prim)
            coll_mesh_tet_vertex_indices = coll_mesh.GetTetVertexIndicesAttr().Get()
            coll_mesh_surface_face_vertex_indices = coll_mesh.GetSurfaceFaceVertexIndicesAttr().Get()

            deformable_tri_indices = _find_surface_triangle_indices_for_tets(
                coll_mesh_tet_vertex_indices, coll_mesh_surface_face_vertex_indices, deformable_filter_indices)
            
            new_element_collision_filter.GetPrim().GetAttribute("omniphysics:groupElemCounts0").Set([len(deformable_tri_indices)])
            new_element_collision_filter.GetPrim().GetAttribute("omniphysics:groupElemIndices0").Set(deformable_tri_indices)


    @staticmethod
    def convert_PhysxPhysicsAttachment(stage: Usd.Stage, path: Sdf.Path):
        prim = stage.GetPrimAtPath(path)
        if not prim or prim.GetTypeName() != "PhysxPhysicsAttachment":
            return

        actor_paths = []
        actor_paths.append(_get_rel_single("actor0", prim))
        actor_paths.append(_get_rel_single("actor1", prim))
        
        slots = [0, 1]
        if not actor_paths[slots[0]]:
            slots.reverse()

        if not actor_paths[slots[0]]:
            return

        deformable_targets = (DeformableSchemaChecker.physxParticleCloth_paths |
                              DeformableSchemaChecker.physxDeformableSurface_paths |
                              DeformableSchemaChecker.physxDeformableBody_paths)

        if actor_paths[slots[0]] in deformable_targets:
            if (actor_paths[slots[0]] in DeformableSchemaChecker.physxParticleCloth_paths or
                actor_paths[slots[0]] in DeformableSchemaChecker.physxDeformableSurface_paths):
                if actor_paths[slots[1]] in DeformableSchemaChecker.physxDeformableBody_paths:
                    slots.reverse()
        elif actor_paths[slots[1]] in deformable_targets:
            slots.reverse()
        else:
            return

        # Retype PhysxPhysicsAttachment to UsdGeom.Scope
        scope = UsdGeom.Scope.Define(stage, path)
        if not scope:
            return

        deformable_prim = stage.GetPrimAtPath(actor_paths[slots[0]])
        if not deformable_prim:
            return

        if actor_paths[slots[1]] in deformable_targets:
            # Deformable-Deformable attachment
            # We can't really implement deformable body - deformable body attachments
            # because we can't exactly represent the attachment points without TetTet attachments.
            # Particle-Cloth/DeformableSurface vs Deformable Body attachments we could convert
            # but will be omited for this version of the conversion.
            return
        else:
            xformable = UsdGeom.Xformable()
            if not actor_paths[slots[1]]:
                # World attachment
                xformable = DeformableSchemaChecker.get_or_create_world_transform(stage)
            else:
                # Rigid or collider attachment
                xformable = UsdGeom.Xformable(stage.GetPrimAtPath(actor_paths[slots[1]]))
                if not xformable:
                    return

            # Deformable-Xformable attachment
            if actor_paths[slots[0]] in DeformableSchemaChecker.physxParticleCloth_paths:
                DeformableSchemaChecker.convert_DeformableTriMeshRigidAttachment(prim, deformable_prim, slots[0], xformable)
            elif actor_paths[slots[0]] in DeformableSchemaChecker.physxDeformableSurface_paths:
                DeformableSchemaChecker.convert_DeformableTriMeshRigidAttachment(prim, deformable_prim, slots[0], xformable)
            elif actor_paths[slots[0]] in DeformableSchemaChecker.physxDeformableBody_paths:
                DeformableSchemaChecker.convert_DeformableBodyRigidAttachment(prim, deformable_prim, slots[0], xformable)
            else:
                return
        
        prefixes = ["physxAttachment:", "physxAutoAttachment:",
                    "actor0", "actor1",
                    "collisionFilterIndices0", "collisionFilterIndices1",
                    "filterType0", "filterType1",
                    "points0", "points1"]
        _remove_properties_by_prefixes(prim, prefixes)

        schemas = ["PhysxAutoAttachmentAPI"]
        _remove_schemas(prim, schemas)


    @staticmethod
    def convert_stage(stage: Usd.Stage, location: Usd.Prim):
        """
        Particle Systems are left in stage - if they are redundant they need to be deleted manually.
        Clean materials, PhysxPBDMaterialAPI is left for particle set usage and need to be cleaned manually.
        """
        # clear out stale global data
        DeformableSchemaChecker.clear()

        # TODO: Use usdrt
        # One issue with usdrt is that get_prim_paths_with_specified_api() sometimes returns empty
        # within fix_deformables_and_tetmeshes() call

        traversal = iter(
            Usd.PrimRange(
                location,
                Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate),
            )
        )

        for prim in traversal:
            if is_omni_path(prim.GetPath()):
                traversal.PruneChildren()
                continue
            if _has_schema_api_to_convert(prim, "PhysxParticleClothAPI"):
                DeformableSchemaChecker.physxParticleCloth_paths.add(prim.GetPath())
                particle_system = _get_particle_system(prim)
                if particle_system:
                    pbd_material = _get_rel_single("material:binding:physics", particle_system.GetPrim())
                    if pbd_material:
                        DeformableSchemaChecker.physxPBDMaterial_cloth_paths.add(pbd_material)
            elif _has_schema_api_to_convert(prim, "PhysxDeformableBodyMaterialAPI"):
                DeformableSchemaChecker.physxDeformableBodyMaterial_paths.add(prim.GetPath())
            elif _has_schema_api_to_convert(prim, "PhysxDeformableSurfaceMaterialAPI"):
                DeformableSchemaChecker.physxDeformableSurfaceMaterial_paths.add(prim.GetPath())
            elif _has_schema_api_to_convert(prim, "PhysxDeformableBodyAPI"):
                DeformableSchemaChecker.physxDeformableBody_paths.add(prim.GetPath())
            elif _has_schema_api_to_convert(prim, "PhysxDeformableSurfaceAPI"):
                DeformableSchemaChecker.physxDeformableSurface_paths.add(prim.GetPath())
            elif _has_schema_type_to_convert(prim, "PhysxPhysicsAttachment"):
                DeformableSchemaChecker.physxPhysicsAttachment_paths.add(prim.GetPath())
            elif _has_schema_type_to_convert(prim, "TetrahedralMesh"):
                DeformableSchemaChecker.tetrahedalMesh_paths.add(prim.GetPath())


        for path in DeformableSchemaChecker.physxPBDMaterial_cloth_paths:
            DeformableSchemaChecker.convert_PhysxPBDMaterial(stage, path)

        for path in DeformableSchemaChecker.physxDeformableBodyMaterial_paths:
            DeformableSchemaChecker.convert_PhysxDeformableBodyMaterial(stage, path)

        for path in DeformableSchemaChecker.physxDeformableSurfaceMaterial_paths:
            DeformableSchemaChecker.convert_PhysxDeformableSurfaceMaterial(stage, path)

        for path in DeformableSchemaChecker.physxParticleCloth_paths:
            DeformableSchemaChecker.convert_PhysxParticleCloth(stage, path)

        for path in DeformableSchemaChecker.physxDeformableBody_paths:
            DeformableSchemaChecker.convert_PhysxDeformableBody(stage, path)

        for path in DeformableSchemaChecker.physxDeformableSurface_paths:
            DeformableSchemaChecker.convert_PhysxDeformableSurface(stage, path)

        for path in DeformableSchemaChecker.tetrahedalMesh_paths:
            DeformableSchemaChecker.convert_TetrahedralMesh(stage, path)

        for path in DeformableSchemaChecker.physxPhysicsAttachment_paths:
            DeformableSchemaChecker.convert_PhysxPhysicsAttachment(stage, path)


    @staticmethod
    def dump_stage(stage: Usd.Stage, out_path: str):
        root_layer = stage.GetRootLayer()
        root_layer.Export(out_path)
        print(f"Stage exported to {out_path}")


    @staticmethod
    def fixer(stage: Usd.Stage, location: Usd.Prim) -> None:
        #DeformableSchemaChecker.dump_stage(stage, r"D:/tmp_orig.usda")
        DeformableSchemaChecker.convert_stage(stage, location)
        #DeformableSchemaChecker.dump_stage(stage, r"D:/tmp_new.usda")


    def CheckStage(self, stage: Usd.Stage):

        if check_timeline_playing(self, stage):
            return

        defaultPrim = stage.GetDefaultPrim()
        if not defaultPrim:
            DeformableSchemaChecker._AddError(
                message="No default prim on stage",
                at=stage,
            )
            return

        deprecated_paths = []
        deprecated_paths.extend(_get_prim_paths_with_specified_api(stage, "PhysxParticleClothAPI"))
        deprecated_paths.extend(_get_prim_paths_with_specified_api(stage, "PhysxDeformableBodyAPI"))
        deprecated_paths.extend(_get_prim_paths_with_specified_api(stage, "PhysxDeformableBodyMaterialAPI"))
        deprecated_paths.extend(_get_prim_paths_with_specified_api(stage, "PhysxDeformableSurfaceAPI"))
        deprecated_paths.extend(_get_prim_paths_with_specified_api(stage, "PhysxDeformableSurfaceMaterialAPI"))
        deprecated_paths.extend(_get_prim_paths_with_specified_type_name(stage, "PhysxPhysicsAttachment"))
        deprecated_paths.extend(_get_prim_paths_with_specified_type_name(stage, "TetrahedralMesh"))

        defaultPath = defaultPrim.GetPath()
        deprecated_paths[:] = [p for p in deprecated_paths if Sdf.Path(p.GetText()).HasPrefix(defaultPath)]

        need_fix = len(deprecated_paths) > 0
        if need_fix: 
            self._AddFailedCheck(
                message="There are old particle cloth/deformable body/deformable surface/attachment schema, you can update it to be using new schemas!",
                at=defaultPrim,
                suggestion=Suggestion(
                    message="Update schemas of old particle cloth/deformable body/deformable surface/attachment",
                    callable=DeformableSchemaChecker.fixer,
                    at=[stage.GetRootLayer()],                                                                     # Preferred location
                ),
            )
