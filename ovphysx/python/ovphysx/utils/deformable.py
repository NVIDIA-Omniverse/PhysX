# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-UTILS-001
# @covers AC-1 AC-2 AC-8 AC-10 AC-12

"""Volume and surface deformable body authoring.

A deformable body is spread across several prims and API schemas, and getting
the combination wrong fails quietly at parse time rather than loudly at
authoring time. These helpers validate the prim type up front, apply the schema
set in the order the parser expects, and author the rest-shape attributes.

Two shapes are supported. The ``set_physics_*_deformable_body`` pair configures
a single prim that is both the simulation and the collision geometry. The
``create_auto_*_deformable_hierarchy`` pair builds the multi-prim layout instead,
where a root prim carries the body API, a cooking source mesh drives generation
of the simulation and collision meshes, and any remaining point-based geometry
in the subtree becomes skinned visual geometry with a bind pose.

The deformable schemas are multiple-apply and partly codeless, so they are
reached by type name through ``Usd.Prim.ApplyAPI`` and by property path rather
than through generated Python classes.

The bind-pose writes in the two ``create_auto_*`` helpers are the one exception
to routing property access through :mod:`~ovphysx.utils.codeless`. They use
``Usd.Prim.CreateAttribute`` directly, so that a prim whose
``OmniPhysicsDeformablePoseAPI`` application USD refuses does not abort the pass
over the rest of the subtree; ``codeless.set_attr`` would raise there.
``REQ-PYTHON-UTILS-001``, "Where the deformable pose attributes stay raw",
records the reasoning.
"""

import functools
import logging
import typing

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from . import codeless
from .materials import ensure_material_on_path
from .schema import (
    get_schema_instances,
    get_schema_property_names,
    remove_api_schema_properties,
    remove_multiple_api_schema_properties,
)

logger = logging.getLogger(__name__)

__all__ = [
    "set_physics_volume_deformable_body",
    "create_auto_volume_deformable_hierarchy",
    "set_physics_surface_deformable_body",
    "create_auto_surface_deformable_hierarchy",
    "remove_auto_deformable_body",
    "remove_auto_deformable_hexahedral_mesh",
    "add_auto_deformable_mesh_simplification",
    "remove_auto_deformable_mesh_simplification",
    "remove_deformable_body",
    "add_deformable_material",
    "add_surface_deformable_material",
]


def _missing_required_default_values(caller, prim, values):
    """Return and report required geometry values missing at default time."""
    missing = [name for name, value in values.items() if value is None]
    if missing:
        logger.warning(
            f"{caller}: required geometry has no value at default time: "
            f"{', '.join(missing)}, {prim.GetPath()}"
        )
    return missing


def set_physics_volume_deformable_body(
    stage,
    prim_path: Sdf.Path,
) -> bool:
    """Setup a volume deformable body based on a UsdGeom.TetMesh at prim_path on stage and add necessary prims and APIs.
    For hierarchical setups use create_auto_volume_deformable_hierarchy.

    Args:
        stage:                          The stage
        prim_path:                      Path to UsdGeom.TetMesh 'sim mesh'
                                        to which the UsdPhysics.DeformableBodyAPI is applied to.

    Returns:
        True / False that indicates success of schema application

        Missing default-time points or tetrahedron indices return False before
        the stage is modified.
    """

    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        logger.warning(
            "set_physics_volume_deformable_body: " + f"No valid primitive prim_path provided: ('{prim_path}')"
        )
        return False

    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        logger.warning(
            "set_physics_volume_deformable_body: "
            + "UsdPhysics.DeformableBodyAPI cannot be applied to a "
            + f"primitive with UsdPhysics.RigidBodyAPI, {prim.GetPath()}"
        )
        return False

    if not prim.IsA(UsdGeom.TetMesh):
        logger.warning(
            "set_physics_volume_deformable_body: "
            + "Supports only adding UsdPhysics.DeformableBodyAPI to UsdGeom.TetMesh"
            + f"Use create_auto_volume_deformable_hierarchy for hierarchical setups, {prim.GetPath()}"
        )
        return False

    tet_mesh = UsdGeom.TetMesh(prim)
    points = tet_mesh.GetPointsAttr().Get()
    indices = tet_mesh.GetTetVertexIndicesAttr().Get()
    if _missing_required_default_values(
        "set_physics_volume_deformable_body",
        prim,
        {"points": points, "tetVertexIndices": indices},
    ):
        return False

    if not codeless.try_apply_api(prim, "OmniPhysicsDeformableBodyAPI"):
        logger.warning(
            "set_physics_volume_deformable_body: "
            + f"UsdPhysics.DeformableBodyAPI application unsuccessful, {prim.GetPath()}"
        )
        return False

    if not codeless.try_apply_api(prim, "OmniPhysicsVolumeDeformableSimAPI"):
        logger.warning(
            "set_physics_volume_deformable_body: "
            + f"Application of UsdPhysics.VolumeDeformableSimAPI unsuccessfull, {prim.GetPath()}"
        )
        return False

    codeless.set_attr(prim, "omniphysics:restShapePoints", points)
    codeless.set_attr(prim, "omniphysics:restTetVtxIndices", indices)

    if not UsdPhysics.CollisionAPI.Apply(prim):
        logger.warning(
            "set_physics_volume_deformable_body: "
            + f"Application of UsdPhysics.CollisionAPI unsuccessfull, {prim.GetPath()}"
        )
        return False

    # Element collision filters need the tet mesh's surface faces authored.
    surfaceFaceIndices = UsdGeom.TetMesh.ComputeSurfaceFaces(UsdGeom.TetMesh(prim), Usd.TimeCode.Default())
    UsdGeom.TetMesh(prim).GetSurfaceFaceVertexIndicesAttr().Set(surfaceFaceIndices)

    return True


def _collect_visual_geometry(root_prim, excluded_paths, caller):
    """Collect visual prims and reject missing default-time point data."""
    excluded_paths = {Sdf.Path(path) for path in excluded_paths}
    visual_geometry = []
    prim_range = Usd.PrimRange(root_prim, Usd.PrimAllPrimsPredicate)
    for prim in prim_range:
        if prim != root_prim:
            xformable = UsdGeom.Xformable(prim)
            if xformable and xformable.GetResetXformStack():
                prim_range.PruneChildren()
                continue

        if not prim.IsA(UsdGeom.PointBased) or prim.GetPath() in excluded_paths:
            continue

        points = UsdGeom.PointBased(prim).GetPointsAttr().Get()
        if points is None:
            logger.warning(
                f"{caller}: PointBased visual prim has no points at default time, "
                f"{prim.GetPath()}"
            )
            return None
        visual_geometry.append((prim, points))
    return visual_geometry


def create_auto_volume_deformable_hierarchy(
    stage: Usd.Stage,
    root_prim_path: typing.Union[str, Sdf.Path],
    simulation_tetmesh_path: typing.Union[str, Sdf.Path],
    collision_tetmesh_path: typing.Union[str, Sdf.Path],
    cooking_src_mesh_path: typing.Union[str, Sdf.Path],
    simulation_hex_mesh_enabled: bool,
    cooking_src_simplification_enabled: bool,
    set_visibility_with_guide_purpose: bool = False,
) -> bool:
    """Creates a volume deformable body from a stage hierachy and adds necessary prims and APIs.
    For single prim deformable bodies, use set_physics_volume_deformable_body on a UsdGeom.TetMesh.

    Args:
        stage:                              The stage
        root_prim_path:                     Path to valid a UsdGeom.Imageable which cannot be a UsdGeom.Gprim.
                                            The UsdPhysics.DeformableBodyAPI is applied to this prim.
        simulation_tetmesh_path:            Path to where simulation mesh should be created or a valid UsdGeom.TetMesh.
        collision_tetmesh_path:             Path to where collision mesh should be created or a valid UsdGeom.TetMesh.
                                            CollisionAPI is applied to the collision mesh.
                                            May be identical to simulation_tetmesh_path.
        cooking_src_mesh_path:              Path to valid UsdGeom.Mesh that is used in cooking to generate the simulation
                                            and collision mesh.
                                            May be outside of root_prim_path sub-hierarchy.
        simulation_hex_mesh_enabled:        If True, simulation mesh is generated as a hexahedral mesh.
        cooking_src_simplification_enabled: If True, PhysxAutoDeformableMeshSimplificationAPI is applied.
        set_visibility_with_guide_purpose:  If True, the simulation and collision meshes are assigned the guide purpose
                                            to hide them from rendering - but only if other GPrims are present under
                                            root_prim_path to provide visible geometry. If the simulation and collision
                                            meshes are the only geometry present and are distinct, then only the
                                            simulation mesh is assigned the guide purpose, leaving the collision mesh
                                            for visual representation.

    Returns:
        True / False that indicates success of creation.

        A hexahedral mesh API that fails to apply returns False, on the same
        terms as a failed mesh simplification request. A caller that asked for a
        hexahedral simulation mesh is never told it succeeded with a tetrahedral
        one.

        A PointBased visual prim without points at the default time returns
        False before the stage is modified.
    """

    root_prim = stage.GetPrimAtPath(root_prim_path)
    if not root_prim:
        logger.warning(
            "create_auto_volume_deformable_hierarchy: "
            + f"No valid primitive root_prim_path provided: ('{root_prim_path}')"
        )
        return False

    if root_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        logger.warning(
            "create_auto_volume_deformable_hierarchy: "
            + "UsdPhysics.DeformableBodyAPI cannot be applied to a "
            + f"primitive with UsdPhysics.RigidBodyAPI, {root_prim.GetPath()}"
        )
        return False

    if root_prim.IsA(UsdGeom.TetMesh):
        logger.warning(
            "create_auto_volume_deformable_hierarchy: "
            + "Use set_physics_volume_deformable_body for applying UsdPhysics.DeformableBodyAPI to UsdGeom.TetMesh"
            + f", {root_prim.GetPath()}"
        )
        return False

    is_valid_root = root_prim.IsA(UsdGeom.Imageable) and not root_prim.IsA(UsdGeom.Gprim)
    if not is_valid_root:
        logger.warning(
            "create_auto_volume_deformable_hierarchy: "
            + "UsdPhysics.DeformableBodyAPI for a volume deformable hierarchy can only be applied to a "
            + f"UsdGeom.Imageable which is not UsdGeom.Gprim, {root_prim.GetPath()}"
        )
        return False

    if Sdf.Path(simulation_tetmesh_path).GetParentPath() != root_prim.GetPath():
        logger.warning(
            "create_auto_volume_deformable_hierarchy: "
            + "simulation_tetmesh_path needs to be immediate child path of root_prim_path: "
            + f"simulation_tetmesh_path: {simulation_tetmesh_path}, root_prim_path: {root_prim_path}"
        )
        return False

    if Sdf.Path(collision_tetmesh_path).GetParentPath() != root_prim.GetPath():
        logger.warning(
            "create_auto_volume_deformable_hierarchy: "
            + "collision_tetmesh_path needs to be immediate child path of root_prim_path: "
            + f"collision_tetmesh_path: {collision_tetmesh_path}, root_prim_path: {root_prim_path}"
        )
        return False

    cooking_src_prim = stage.GetPrimAtPath(cooking_src_mesh_path)
    if not cooking_src_prim or not cooking_src_prim.IsA(UsdGeom.Mesh):
        logger.warning(
            "create_auto_volume_deformable_hierarchy: "
            + f"cooking_src_mesh_path needs to point to valid UsdGeom.Mesh, root_prim_path: {root_prim_path}"
        )
        return False

    visual_geometry = _collect_visual_geometry(
        root_prim,
        (simulation_tetmesh_path, collision_tetmesh_path),
        "create_auto_volume_deformable_hierarchy",
    )
    if visual_geometry is None:
        return False

    _remove_deformable_body(
        stage,
        root_prim.GetPath(),
        keep_collision_api_paths=frozenset(
            (Sdf.Path(simulation_tetmesh_path), Sdf.Path(collision_tetmesh_path))
        ),
    )

    if not codeless.try_apply_api(root_prim, "PhysxAutoDeformableBodyAPI"):
        logger.warning(
            "create_auto_volume_deformable_hierarchy: "
            + f"PhysxAutoDeformableBodyAPI application unsuccessful, {root_prim.GetPath()}"
        )
        return False

    codeless.set_rel(root_prim, "physxDeformableBody:cookingSourceMesh", [cooking_src_mesh_path])

    if cooking_src_simplification_enabled:
        if not add_auto_deformable_mesh_simplification(stage, root_prim.GetPath()):
            return False

    if simulation_hex_mesh_enabled:
        if not codeless.try_apply_api(root_prim, "PhysxAutoDeformableHexahedralMeshAPI"):
            logger.warning(
                "create_auto_volume_deformable_hierarchy: failed to apply PhysxAutoDeformableHexahedralMeshAPI, "
                + f"{root_prim_path}"
            )
            return False

    sim_tet_mesh = UsdGeom.TetMesh.Define(stage, simulation_tetmesh_path)
    if not sim_tet_mesh:
        logger.warning(
            "create_auto_volume_deformable_hierarchy: failed to define UsdGeom.TetMesh at: "
            + f"{simulation_tetmesh_path}"
        )
        return False

    if not codeless.try_apply_api(sim_tet_mesh.GetPrim(), "OmniPhysicsVolumeDeformableSimAPI"):
        logger.warning(
            "create_auto_volume_deformable_hierarchy: "
            + f"UsdPhysics.VolumeDeformableSimAPI application unsuccessful, {sim_tet_mesh.GetPath()}"
        )
        return False

    coll_tet_mesh = sim_tet_mesh
    if collision_tetmesh_path != simulation_tetmesh_path:
        coll_tet_mesh = UsdGeom.TetMesh.Define(stage, collision_tetmesh_path)
        if not coll_tet_mesh:
            logger.warning(
                "create_auto_volume_deformable_hierarchy: failed to define UsdGeom.TetMesh at: "
                + f"{collision_tetmesh_path}"
            )
            return False

    if not coll_tet_mesh.GetPrim().ApplyAPI(UsdPhysics.CollisionAPI):
        logger.warning(
            "create_auto_volume_deformable_hierarchy: "
            + f"UsdPhysics.CollisionAPI application unsuccessful, {coll_tet_mesh.GetPath()}"
        )
        return False

    purposes = ["bindPose"]
    for prim, points in visual_geometry:
        codeless.try_apply_api(prim, "OmniPhysicsDeformablePoseAPI", "default")
        prim.CreateAttribute("deformablePose:default:omniphysics:purposes", Sdf.ValueTypeNames.TokenArray).Set(purposes)
        prim.CreateAttribute("deformablePose:default:omniphysics:points", Sdf.ValueTypeNames.Point3fArray).Set(points)

    codeless.try_apply_api(sim_tet_mesh.GetPrim(), "OmniPhysicsDeformablePoseAPI", "default")
    sim_tet_mesh.GetPrim().CreateAttribute(
        "deformablePose:default:omniphysics:purposes", Sdf.ValueTypeNames.TokenArray
    ).Set(purposes)
    codeless.try_apply_api(coll_tet_mesh.GetPrim(), "OmniPhysicsDeformablePoseAPI", "default")
    coll_tet_mesh.GetPrim().CreateAttribute(
        "deformablePose:default:omniphysics:purposes", Sdf.ValueTypeNames.TokenArray
    ).Set(purposes)

    if set_visibility_with_guide_purpose:
        if len(visual_geometry) > 0:
            UsdGeom.Imageable(sim_tet_mesh).GetPurposeAttr().Set(UsdGeom.Tokens.guide)
            UsdGeom.Imageable(coll_tet_mesh).GetPurposeAttr().Set(UsdGeom.Tokens.guide)
        elif sim_tet_mesh != coll_tet_mesh:
            UsdGeom.Imageable(sim_tet_mesh).GetPurposeAttr().Set(UsdGeom.Tokens.guide)

    # applying the body API last is what makes the runtime pick the completed
    # hierarchy up for cooking
    if not codeless.try_apply_api(root_prim, "OmniPhysicsDeformableBodyAPI"):
        logger.warning(
            "create_auto_volume_deformable_hierarchy: "
            + f"UsdPhysics.DeformableBodyAPI application unsuccessful, {root_prim.GetPath()}"
        )
        return False

    return True


def set_physics_surface_deformable_body(
    stage,
    prim_path: Sdf.Path,
) -> bool:
    """Setup a surface deformable body based on a UsdGeom.Mesh at prim_path on stage and add necessary prims and APIs.
    For hierarchical setups use create_auto_surface_deformable_hierarchy.

    Args:
        stage:                          The stage
        prim_path:                      Path to UsdGeom.Mesh 'sim mesh' to which the UsdPhysics.DeformableBodyAPI is applied to.

    Returns:
        True / False that indicates success of schema application

        Missing default-time points or face topology return False before the
        stage is modified.
    """

    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        logger.warning(
            "set_physics_surface_deformable_body: " + f"No valid primitive prim_path provided: ('{prim_path}')"
        )
        return False

    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        logger.warning(
            "set_physics_surface_deformable_body: "
            + "UsdPhysics.DeformableBodyAPI cannot be applied to a "
            + f"primitive with UsdPhysics.RigidBodyAPI, {prim.GetPath()}"
        )
        return False

    if not prim.IsA(UsdGeom.Mesh):
        logger.warning(
            "set_physics_surface_deformable_body: "
            + "Supports only adding UsdPhysics.DeformableBodyAPI to UsdGeom.Mesh"
            + f"Use create_auto_surface_deformable_hierarchy for hierarchical setups, {prim.GetPath()}"
        )
        return False

    mesh = UsdGeom.Mesh(prim)

    points = mesh.GetPointsAttr().Get()
    face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get()
    face_vertex_indices = mesh.GetFaceVertexIndicesAttr().Get()
    if _missing_required_default_values(
        "set_physics_surface_deformable_body",
        prim,
        {
            "points": points,
            "faceVertexCounts": face_vertex_counts,
            "faceVertexIndices": face_vertex_indices,
        },
    ):
        return False

    tri_indices = [Gf.Vec3i(0, 0, 0)] * len(face_vertex_counts)
    for i, face_count in enumerate(face_vertex_counts):
        if face_count != 3:
            logger.warning(
                "set_physics_surface_deformable_body: "
                + "UsdPhysics.SurfaceDeformableSimAPI can only be applied to meshes limited to "
                + f"triangular faces, {prim.GetPath()}"
            )
            return False
        tri_indices[i] = Gf.Vec3i(
            face_vertex_indices[3 * i + 0], face_vertex_indices[3 * i + 1], face_vertex_indices[3 * i + 2]
        )

    if not codeless.try_apply_api(prim, "OmniPhysicsDeformableBodyAPI"):
        logger.warning(
            "set_physics_surface_deformable_body: "
            + f"UsdPhysics.DeformableBodyAPI application unsuccessful, {prim.GetPath()}"
        )
        return False

    if not prim.HasAPI("OmniPhysicsSurfaceDeformableSimAPI"):
        if not codeless.try_apply_api(prim, "OmniPhysicsSurfaceDeformableSimAPI"):
            logger.warning(
                "set_physics_surface_deformable_body: "
                + f"Application of UsdPhysics.SurfaceDeformableSimAPI unsuccessfull, {prim.GetPath()}"
            )
            return False

    codeless.set_attr(prim, "omniphysics:restShapePoints", points)
    codeless.set_attr(prim, "omniphysics:restTriVtxIndices", tri_indices)

    if not UsdPhysics.CollisionAPI.Apply(prim):
        logger.warning(
            "set_physics_surface_deformable_body: "
            + f"Application of UsdPhysics.CollisionAPI unsuccessfull, {prim.GetPath()}"
        )
        return False

    return True


def create_auto_surface_deformable_hierarchy(
    stage: Usd.Stage,
    root_prim_path: typing.Union[str, Sdf.Path],
    simulation_mesh_path: typing.Union[str, Sdf.Path],
    cooking_src_mesh_path: typing.Union[str, Sdf.Path],
    cooking_src_simplification_enabled: bool,
    set_visibility_with_guide_purpose: bool = False,
) -> bool:
    """Creates a surface deformable body from a stage hierachy and adds necessary prims and APIs.
    For single prim deformable bodies, use set_physics_surface_deformable_body on a UsdGeom.Mesh.

    Args:
        stage:                              The stage
        root_prim_path:                     Path to valid a UsdGeom.Imageable which cannot be a UsdGeom.Gprim.
                                            The UsdPhysics.DeformableBodyAPI is applied to this prim.
        simulation_mesh_path:               Path to where simulation mesh should be created or a valid UsdGeom.Mesh.
        cooking_src_mesh_path:              Path to valid UsdGeom.Mesh that is used in cooking to generate the
                                            simulation mesh.
                                            May be outside of root_prim_path sub-hierarchy.
        cooking_src_simplification_enabled: If True, PhysxAutoDeformableMeshSimplificationAPI is applied.
        set_visibility_with_guide_purpose:  If True, the simulation mesh is assigned the guide purpose to hide it from
                                            rendering - but only if other GPrims are present under root_prim_path to
                                            provide visible geometry.

    Returns:
        True / False that indicates success of creation.

        A PointBased visual prim without points at the default time returns
        False before the stage is modified.
    """

    root_prim = stage.GetPrimAtPath(root_prim_path)
    if not root_prim:
        logger.warning(
            "create_auto_surface_deformable_hierarchy: "
            + f"No valid primitive root_prim_path provided: ('{root_prim_path}')"
        )
        return False

    if root_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        logger.warning(
            "create_auto_surface_deformable_hierarchy: "
            + "UsdPhysics.DeformableBodyAPI cannot be applied to a "
            + f"primitive with UsdPhysics.RigidBodyAPI, {root_prim.GetPath()}"
        )
        return False

    if root_prim.IsA(UsdGeom.Mesh):
        logger.warning(
            "create_auto_surface_deformable_hierarchy: "
            + "Use set_physics_surface_deformable_body for applying UsdPhysics.DeformableBodyAPI to UsdGeom.Mesh"
            + f", {root_prim.GetPath()}"
        )
        return False

    is_valid_root = root_prim.IsA(UsdGeom.Imageable) and not root_prim.IsA(UsdGeom.Gprim)
    if not is_valid_root:
        logger.warning(
            "create_auto_surface_deformable_hierarchy: "
            + "UsdPhysics.DeformableBodyAPI for a volume deformable hierarchy can only be applied to a "
            + f"UsdGeom.Imageable which is not UsdGeom.Gprim, {root_prim.GetPath()}"
        )
        return False

    if Sdf.Path(simulation_mesh_path).GetParentPath() != root_prim.GetPath():
        logger.warning(
            "create_auto_surface_deformable_hierarchy: "
            + "simulation_mesh_path needs to be immediate child path of root_prim_path: "
            + f"simulation_mesh_path: {simulation_mesh_path}, root_prim_path: {root_prim_path}"
        )
        return False

    cooking_src_prim = stage.GetPrimAtPath(cooking_src_mesh_path)
    if not cooking_src_prim or not cooking_src_prim.IsA(UsdGeom.Mesh):
        logger.warning(
            "create_auto_surface_deformable_hierarchy: "
            + f"cooking_src_mesh_path needs to point to valid UsdGeom.Mesh, root_prim_path: {root_prim_path}"
        )
        return False

    visual_geometry = _collect_visual_geometry(
        root_prim,
        (simulation_mesh_path,),
        "create_auto_surface_deformable_hierarchy",
    )
    if visual_geometry is None:
        return False

    _remove_deformable_body(
        stage,
        root_prim.GetPath(),
        keep_collision_api_paths=frozenset((Sdf.Path(simulation_mesh_path),)),
    )

    if not codeless.try_apply_api(root_prim, "PhysxAutoDeformableBodyAPI"):
        logger.warning(
            "create_auto_surface_deformable_hierarchy: "
            + f"PhysxAutoDeformableBodyAPI application unsuccessful, {root_prim.GetPath()}"
        )
        return False

    codeless.set_rel(root_prim, "physxDeformableBody:cookingSourceMesh", [cooking_src_mesh_path])

    if cooking_src_simplification_enabled:
        if not add_auto_deformable_mesh_simplification(stage, root_prim.GetPath()):
            return False

    sim_mesh = UsdGeom.Mesh.Define(stage, simulation_mesh_path)
    if not sim_mesh:
        logger.warning(
            "create_auto_surface_deformable_hierarchy: failed to define UsdGeom.Mesh at: " + f"{simulation_mesh_path}"
        )
        return False

    if not codeless.try_apply_api(sim_mesh.GetPrim(), "OmniPhysicsSurfaceDeformableSimAPI"):
        logger.warning(
            "create_auto_surface_deformable_hierarchy: "
            + f"UsdPhysics.SurfaceDeformableSimAPI application unsuccessful, {sim_mesh.GetPath()}"
        )
        return False

    if not sim_mesh.GetPrim().ApplyAPI(UsdPhysics.CollisionAPI):
        logger.warning(
            "create_auto_surface_deformable_hierarchy: "
            + f"UsdPhysics.CollisionAPI application unsuccessful, {sim_mesh.GetPath()}"
        )
        return False

    purposes = ["bindPose"]
    for prim, points in visual_geometry:
        codeless.try_apply_api(prim, "OmniPhysicsDeformablePoseAPI", "default")
        prim.CreateAttribute("deformablePose:default:omniphysics:purposes", Sdf.ValueTypeNames.TokenArray).Set(purposes)
        prim.CreateAttribute("deformablePose:default:omniphysics:points", Sdf.ValueTypeNames.Point3fArray).Set(points)

    codeless.try_apply_api(sim_mesh.GetPrim(), "OmniPhysicsDeformablePoseAPI", "default")
    sim_mesh.GetPrim().CreateAttribute(
        "deformablePose:default:omniphysics:purposes", Sdf.ValueTypeNames.TokenArray
    ).Set(purposes)

    if set_visibility_with_guide_purpose:
        if len(visual_geometry) > 0:
            UsdGeom.Imageable(sim_mesh).GetPurposeAttr().Set(UsdGeom.Tokens.guide)

    # applying the body API last is what makes the runtime pick the completed
    # hierarchy up for cooking
    if not codeless.try_apply_api(root_prim, "OmniPhysicsDeformableBodyAPI"):
        logger.warning(
            "create_auto_surface_deformable_hierarchy: "
            + f"UsdPhysics.DeformableBodyAPI application unsuccessful, {root_prim.GetPath()}"
        )
        return False

    return True


def remove_auto_deformable_body(
    stage,
    prim_path: Sdf.Path,
):
    """Removes the auto deformable body API set and its generated sub-component APIs.

    Args:
        stage:      The stage.
        prim_path:  Path to the prim carrying PhysxAutoDeformableBodyAPI.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if prim:
        codeless.remove_api(prim, "PhysxAutoDeformableBodyAPI")
        remove_api_schema_properties("PhysxAutoDeformableBodyAPI", prim)
        attributes_to_remove = [x.GetName() for x in prim.GetAuthoredPropertiesInNamespace(["physxDeformableBody"])]
        for attribute in attributes_to_remove:
            prim.RemoveProperty(attribute)
        # remove sub components after removing PhysxAutoDeformableBodyAPI, so
        # previously setup meshes don't get erased.
        remove_auto_deformable_hexahedral_mesh(stage, prim_path)
        remove_auto_deformable_mesh_simplification(stage, prim_path)


def remove_auto_deformable_hexahedral_mesh(
    stage,
    prim_path: Sdf.Path,
):
    """Removes the hexahedral simulation mesh request from an auto deformable body.

    Args:
        stage:      The stage.
        prim_path:  Path to the prim carrying PhysxAutoDeformableHexahedralMeshAPI.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if prim:
        codeless.remove_api(prim, "PhysxAutoDeformableHexahedralMeshAPI")
        remove_api_schema_properties("PhysxAutoDeformableHexahedralMeshAPI", prim)


def add_auto_deformable_mesh_simplification(
    stage,
    prim_path: Sdf.Path,
) -> bool:
    """Add a simplification collision mesh on a prim with PhysxAutoDeformableBodyAPI, and setup the deformable
       body correspondingly.

    Args:
        stage:                          The stage
        prim_path:                      Path to UsdGeom.Scope/UsdGeom.Xform
                                        to which the PhysxAutoDeformableBodyAPI is applied to.

    Returns:
        True / False that indicates success of schema application
    """

    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        logger.warning("No valid primitive prim_path provided")
        return False

    if not (prim.IsA(UsdGeom.Scope) or prim.IsA(UsdGeom.Xform)):
        logger.warning(
            "PhysxAutoDeformableMeshSimplificationAPI can only be applied to a "
            "UsdGeom.Scope or UsdGeom.Xform"
        )
        return False

    if not prim.HasAPI("PhysxAutoDeformableBodyAPI"):
        logger.warning(
            "PhysxAutoDeformableMeshSimplificationAPI can only be applied to a prim with "
            "PhysxAutoDeformableBodyAPI"
        )
        return False

    if not codeless.try_apply_api(prim, "PhysxAutoDeformableMeshSimplificationAPI"):
        logger.warning("PhysxAutoDeformableMeshSimplificationAPI application unsuccessful")
        return False

    return True


def remove_auto_deformable_mesh_simplification(
    stage,
    prim_path: Sdf.Path,
):
    """Removes the cooking-source simplification request from an auto deformable body.

    Args:
        stage:      The stage.
        prim_path:  Path to the prim carrying PhysxAutoDeformableMeshSimplificationAPI.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if prim:
        codeless.remove_api(prim, "PhysxAutoDeformableMeshSimplificationAPI")
        remove_api_schema_properties("PhysxAutoDeformableMeshSimplificationAPI", prim)


# Marks a collider as the body's own. Every collision API these helpers apply
# lands on a prim they also give one of these. A volume hierarchy's separate
# collision mesh is the one collider they author without one.
_COLLISION_OWNER_APIS = (
    "OmniPhysicsVolumeDeformableSimAPI",
    "OmniPhysicsSurfaceDeformableSimAPI",
)


def _collision_api_owners(prim, keep_paths):
    """The prims a removal takes ``UsdPhysics.CollisionAPI`` from.

    Call before anything is stripped: the evidence is a simulation API the strip
    removes.

    Args:
        prim:       The prim the removal was asked for.
        keep_paths: Paths whose collision API stays, whoever authored it.
    """
    return [
        candidate
        for candidate in Usd.PrimRange(prim, Usd.PrimAllPrimsPredicate)
        if candidate.GetPath() not in keep_paths
        and candidate.HasAPI(UsdPhysics.CollisionAPI)
        and any(candidate.HasAPI(api) for api in _COLLISION_OWNER_APIS)
    ]


# The single-apply deformable APIs a removal takes. Grouped by schema family,
# not ordered: each entry is an independent `apiSchemas` edit and the properties
# go in one pass afterwards.
_REMOVED_DEFORMABLE_APIS = (
    "PhysxAutoDeformableBodyAPI",
    "PhysxAutoDeformableHexahedralMeshAPI",
    "PhysxAutoDeformableMeshSimplificationAPI",
    # Legacy spellings. Teardown takes them; nothing here applies them.
    "PhysxBaseDeformableBodyAPI",
    "PhysxSurfaceDeformableBodyAPI",
    "OmniPhysicsDeformableBodyAPI",
    "OmniPhysicsVolumeDeformableSimAPI",
    "OmniPhysicsSurfaceDeformableSimAPI",
    "OmniPhysicsCurvesDeformableSimAPI",
)


@functools.lru_cache(maxsize=None)
def _schema_property_names(schema):
    """The property names a schema declares, cached.

    Returns a ``frozenset``: a cached value must not be mutable. An unregistered
    schema raises rather than caching an empty set.
    """
    return frozenset(get_schema_property_names(schema))


@functools.lru_cache(maxsize=None)
def _removed_deformable_property_names():
    """Every property name ``_REMOVED_DEFORMABLE_APIS`` declares, as one set."""
    return frozenset().union(*(_schema_property_names(api) for api in _REMOVED_DEFORMABLE_APIS))


def _remove_deformable_apis(prim):
    """Strip every deformable API a removal takes, and its properties, from one prim.

    ``codeless.remove_api`` and ``Usd.Prim.RemoveAPI`` drop the prim's
    ``apiSchemas`` entry and leave every property authored under it in place, so
    the properties are removed here by name.

    The property strip does not depend on the API being applied, so a property
    orphaned by an earlier partial teardown is still taken. What protects a
    property is an API that survives this call and declares it: several of these
    APIs share a property, and ``OmniPhysicsBodyAPI`` is the case that matters
    most. It is a built-in of ``OmniPhysicsDeformableBodyAPI`` and shares three
    properties with it, so it stays applied after that removal only when the
    caller applied it in its own right -- which is the evidence that the shared
    properties are the caller's and not this removal's to take.

    Args:
        prim: The Usd.Prim to strip.
    """
    # Resolve the property names before removing anything: a partly registered
    # schema set raises here, and it must do so before the prim is edited.
    removed_names = _removed_deformable_property_names()

    # One applied-schemas read decides the whole family. `HasAPI` answers from
    # this same list, and the list is re-read only when a removal changed it.
    applied = prim.GetAppliedSchemas()
    carried = [api for api in _REMOVED_DEFORMABLE_APIS if api in applied]
    for api in carried:
        codeless.remove_api(prim, api)
    if carried:
        applied = prim.GetAppliedSchemas()

    # `RemoveProperty` on a name no layer authored changes nothing, and most
    # prims of a subtree author none of these.
    authored = prim.GetAuthoredPropertyNames()
    doomed = set(removed_names.intersection(authored))
    if doomed:
        for name in applied:
            doomed.difference_update(_schema_property_names(name.split(":", 1)[0]))

    doomed.update(name for name in authored if name.startswith("physxDeformableBody:"))

    for name in sorted(doomed):
        prim.RemoveProperty(name)

    # a multiple-apply schema's instance names are not reachable from its
    # Python class, so they are recovered from the applied schema list
    for pose_instance_name in get_schema_instances(prim, "OmniPhysicsDeformablePoseAPI"):
        codeless.remove_api(prim, "OmniPhysicsDeformablePoseAPI", pose_instance_name)
        remove_multiple_api_schema_properties(
            "OmniPhysicsDeformablePoseAPI", prim, "deformablePose", pose_instance_name
        )


def remove_deformable_body(
    stage,
    prim_path: Sdf.Path,
):
    """Removes every deformable body API, and its local properties, from a prim and its subtree.

    Both the single-prim and the hierarchical layouts are handled, and the
    per-instance deformable pose APIs are removed too. A simulation mesh prim
    that was generated automatically is left in place, since a caller may have
    taken ownership of it.

    The sweep covers the supplied prim and every descendant, at any depth. It
    does not stop at a prim that resets its xform stack, so it also reaches
    pose APIs a caller applied outside the range the
    ``create_auto_*_deformable_hierarchy`` helpers author over.

    ``UsdPhysics.CollisionAPI`` is the exception, and is taken only from a prim
    carrying a deformable simulation API, at any depth of that same sweep. Every
    other prim keeps it, a volume hierarchy's separate collision mesh included.
    A collision API the caller applied to the simulation geometry goes with the
    body.

    A property is removed from the current edit target, so one whose opinion
    arrives over a reference or an inherit arc, or from a stronger layer, keeps
    its resolved value after its API is gone. The API itself does go: the
    removal composes over such an arc.

    Args:
        stage:      The stage.
        prim_path:  Path to the deformable body root prim.
    """
    _remove_deformable_body(stage, prim_path, keep_collision_api_paths=frozenset())


def _remove_deformable_body(stage, prim_path, keep_collision_api_paths):
    """:func:`remove_deformable_body`, with paths whose collision API it keeps.

    The ``create_auto_*`` helpers exempt the mesh paths they were given: taking
    the collision API there destroys the caller's properties under it, and no
    return after the teardown reapplies it. A path a rebuild abandons is not
    exempt, so an attributable collider is taken there. A volume layout's
    separate collision mesh carries no simulation API, so an abandoned one keeps
    its collider and stays a second one in the subtree.

    Args:
        stage:                    The stage.
        prim_path:                Path to the deformable body root prim.
        keep_collision_api_paths: Paths whose ``UsdPhysics.CollisionAPI`` stays.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        return

    collision_owners = _collision_api_owners(prim, keep_collision_api_paths)

    for descendant in Usd.PrimRange(prim, Usd.PrimAllPrimsPredicate):
        _remove_deformable_apis(descendant)

    for owner in collision_owners:
        owner.RemoveAPI(UsdPhysics.CollisionAPI)
        remove_api_schema_properties(UsdPhysics.CollisionAPI, owner)


def add_deformable_material(
    stage: Usd.Stage,
    path,
    density=None,
    static_friction=None,
    dynamic_friction=None,
    youngs_modulus=None,
    poissons_ratio=None,
):
    """Applies the UsdPhysics.DeformableMaterialAPI to the prim at path on stage.

    Args:
        stage:                          The stage
        path:                           Path to UsdShade.Material to which the material API should be applied to.
        ... schema attributes:          See USD schema for documentation

    Returns:
        True if the API apply succeeded.
    """

    if not ensure_material_on_path(stage, path):
        return False

    prim = stage.GetPrimAtPath(path)
    if not codeless.try_apply_api(prim, "OmniPhysicsDeformableMaterialAPI"):
        logger.warning("OmniPhysicsDeformableMaterialAPI application unsuccessful")
        return False

    # same defaults as in usdLoad/Material.cpp
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    kpu = UsdPhysics.GetStageKilogramsPerUnit(stage)
    density = -1.0 if density is None else density
    static_friction = 0.5 if static_friction is None else static_friction
    dynamic_friction = 0.25 if dynamic_friction is None else dynamic_friction
    youngs_modulus = (5.0e5 * mpu / kpu) if youngs_modulus is None else youngs_modulus
    poissons_ratio = 0.45 if poissons_ratio is None else poissons_ratio

    codeless.set_attr(prim, "omniphysics:density", density)
    codeless.set_attr(prim, "omniphysics:staticFriction", static_friction)
    codeless.set_attr(prim, "omniphysics:dynamicFriction", dynamic_friction)
    codeless.set_attr(prim, "omniphysics:youngsModulus", youngs_modulus)
    codeless.set_attr(prim, "omniphysics:poissonsRatio", poissons_ratio)

    return True


def add_surface_deformable_material(
    stage: Usd.Stage,
    path,
    density=None,
    static_friction=None,
    dynamic_friction=None,
    youngs_modulus=None,
    poissons_ratio=None,
    surface_thickness=None,
    surface_stretch_stiffness=None,
    surface_shear_stiffness=None,
    surface_bend_stiffness=None,
):
    """Applies the UsdPhysics.SurfaceDeformableMaterialAPI to the prim at path on stage.

    Args:
        stage:                          The stage
        path:                           Path to UsdShade.Material to which the material API should be applied to.
        ... schema attributes:          See USD schema for documentation

    Returns:
        True if the API apply succeeded.
    """
    if not add_deformable_material(
        stage,
        path,
        density=density,
        static_friction=static_friction,
        dynamic_friction=dynamic_friction,
        youngs_modulus=youngs_modulus,
        poissons_ratio=poissons_ratio,
    ):
        return False

    prim = stage.GetPrimAtPath(path)
    if not codeless.try_apply_api(prim, "OmniPhysicsSurfaceDeformableMaterialAPI"):
        logger.warning("OmniPhysicsSurfaceDeformableMaterialAPI application unsuccessful")
        return False

    # same defaults as in usdLoad/Material.cpp
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    surface_thickness = (0.001 / mpu) if surface_thickness is None else surface_thickness
    surface_stretch_stiffness = 0.0 if surface_stretch_stiffness is None else surface_stretch_stiffness
    surface_shear_stiffness = 0.0 if surface_shear_stiffness is None else surface_shear_stiffness
    surface_bend_stiffness = 0.0 if surface_bend_stiffness is None else surface_bend_stiffness

    codeless.set_attr(prim, "omniphysics:surfaceThickness", surface_thickness)
    codeless.set_attr(prim, "omniphysics:surfaceStretchStiffness", surface_stretch_stiffness)
    codeless.set_attr(prim, "omniphysics:surfaceShearStiffness", surface_shear_stiffness)
    codeless.set_attr(prim, "omniphysics:surfaceBendStiffness", surface_bend_stiffness)

    return True
