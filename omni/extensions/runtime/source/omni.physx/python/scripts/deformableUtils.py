# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import typing
from typing import List
import carb
from . import physicsUtils, utils
from pxr import UsdGeom, Usd, Sdf, Tf, PhysxSchema, UsdPhysics, Gf, UsdShade, Vt
from omni.physx import get_physx_cooking_interface
from omni.physx import get_physx_attachment_private_interface
import omni.physx.bindings._physx as pxb

# DEPRECATED
class TetMeshData:
    def __init__(self):
        self.points = []
        self.indices = []
        self.embedding = []

    def from_dict(self, data: typing.Dict[str, typing.List]):
        self.points = [Gf.Vec3f(v.x, v.y, v.z) for v in data["points"]]
        self.indices = [index for index in data["indices"]]
        if "embedding" in data:
            self.embedding = [index for index in data["embedding"]]

    def from_mesh(self, mesh: PhysxSchema.TetrahedralMesh):
        self.points = mesh.GetPointsAttr().Get()
        self.indices = mesh.GetIndicesAttr().Get()

    def is_valid(self) -> bool:
        return self.points and self.indices


# DEPRECATED
def add_physx_deformable_body(  # TODO PREIST: Get defaults from schema metadata instead of hardcoding here
    stage,
    prim_path: Sdf.Path,
    collision_rest_points: typing.List[Gf.Vec3f] = None,
    collision_indices: typing.List[int] = None,
    kinematic_enabled : bool = False,
    collision_simplification : bool = True,
    collision_simplification_remeshing: bool = True,
    collision_simplification_remeshing_resolution: int = 0,
    collision_simplification_target_triangle_count: int = 0,
    collision_simplification_force_conforming: bool = False,
    simulation_rest_points: typing.List[Gf.Vec3f] = None,
    simulation_indices: typing.List[int] = None,
    embedding: typing.List[int] = None,
    simulation_hexahedral_resolution: int = 10,
    solver_position_iteration_count: int = None,
    vertex_velocity_damping: float = None,
    sleep_damping: float = None,
    sleep_threshold: float = None,
    settling_threshold: float = None,
    self_collision: bool = None,
    self_collision_filter_distance: float = None,
) -> bool:

    """DEPRECATED: Will be replaced by new deformable implementation in future release.
    Applies the deformable body API to an Xform at prim_path on stage.

    Args:
        stage:                                          The stage
        prim_path:                                      Path to UsdGeom.Mesh 'skin mesh' to which the PhysxSchema.PhysXDeformableBodyAPI is applied to.
        collision_rest_points:                          List of vertices of the collision tetrahedral mesh at rest.
                                                        If a collision mesh is provided, the simulation mesh needs to be provided too.
                                                        If no collision mesh is provided, it will be computed implicitly based on the simplification parameter.
        collision_indices:                              List of indices of the collision tetrahedral mesh.
        kinematic_enabled:                              Enables kinematic body.
        collision_simplification:                       Boolean flag indicating if simplification should be applied to the mesh before creating a
                                                        softbody out of it. Is ignored if simulation mesh has been provided.
        collision_simplification_remeshing:             Boolean flag indicating if the simplification should be based on remeshing.
                                                        Ignored if collision_simplification equals False.
        collision_simplification_remeshing_resolution:  The resolution used for remeshing. A value of 0 indicates that a heuristic is used to determine
                                                        the resolution. Ignored if collision_simplification_remeshing is False.
        collision_simplification_target_triangle_count: The target triangle count used for the simplification. A value of 0 indicates
                                                        that a heuristic based on the simulation_hexahedral_resolution is to determine the target count.
                                                        Ignored if collision_simplification equals False.
        collision_simplification_force_conforming:      Boolean flag indicating that the tretrahedralizer used to generate the collision mesh should produce
                                                        tetrahedra that conform to the triangle mesh. If False the implementation chooses the tretrahedralizer
                                                        used.
        simulation_rest_points:                         List of vertices of the simulation tetrahedral mesh at rest.
                                                        If a simulation mesh is provided, the collision mesh needs to be provided too.
                                                        If no simulation mesh is provided it will be computed implicitly based on simulation_hexahedral_resolution.
        simulation_indices:                             List of indices of the simulation tetrahedral mesh.
        embedding:                                      Optional embedding information mapping collision points to containing simulation tetrahedra.
        simulation_hexahedral_resolution:               Target resolution of voxel simulation mesh. Is ignored if simulation mesh has been provided.
        ...:                                            See USD schema for documentation

    Returns:
        True / False that indicates success of schema application
    """

    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        carb.log_warn("No valid primitive prim_path provided")
        return False

    # check if it is a rigid body:
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        carb.log_warn(
            "PhysxSchema.PhysxDeformableBodyAPI cannot be applied to a primitive with UsdPhysics.RigidBodyAPI"
        )
        return False

    # check if it is a UsdGeom.Mesh
    if not prim.IsA(UsdGeom.Mesh):
        carb.log_warn("PhysxSchema.PhysxDeformableBodyAPI can only be applied to a UsdGeom.Mesh")
        return False

    # check collision mesh
    if collision_rest_points:
        if len(collision_rest_points) < 4:
            carb.log_warn("collision_rest_points is invalid")
            return False
        if not collision_indices:
            carb.log_warn("collision mesh invalid")
            return False
        if not simulation_rest_points:
            carb.log_warn("collision mesh is invalid without simulation mesh")
            return False

    if collision_indices:
        if len(collision_indices) < 4 or len(collision_indices) % 4 != 0:
            carb.log_warn("collision_indices is invalid")
            return False
        if not collision_rest_points:
            carb.log_warn("collision mesh invalid")
            return False

    # check simulation mesh
    if simulation_rest_points:
        if len(simulation_rest_points) < 4:
            carb.log_warn("simulation_rest_points is invalid")
            return False
        if not simulation_indices:
            carb.log_warn("simulation mesh invalid")
            return False
        if not collision_rest_points:
            carb.log_warn("simulation mesh is invalid without collision mesh")
            return False

    if simulation_indices:
        if len(simulation_indices) < 4 or len(simulation_indices) % 4 != 0:
            carb.log_warn("simulation_indices is invalid")
            return False
        if not simulation_rest_points:
            carb.log_warn("simulation mesh invalid")
            return False

    if embedding:
        if len(embedding) != len(collision_rest_points):
            carb.log_warn("embedding is invalid")
            return False
        if not simulation_rest_points:
            carb.log_warn("embedding is invalid without simulation mesh")
            return False

    # warnings
    if kinematic_enabled and (collision_rest_points or simulation_rest_points):
        carb.log_warn("provided custom collision or simulation mesh: unable to enable kinematic on time varying skin mesh")
        kinematic_enabled = False

    if kinematic_enabled and collision_simplification_remeshing is not None and collision_simplification_remeshing == True:
        carb.log_warn("enable kinematic: remeshing disabled")
        collision_simplification_remeshing = False

    if kinematic_enabled and collision_simplification_force_conforming is not None and collision_simplification_force_conforming == False:
        carb.log_warn("enable kinematic: force conforming enabled")
        collision_simplification_force_conforming = True

    # apply APIs and create attributes
    deformable_body_api = PhysxSchema.PhysxDeformableBodyAPI.Apply(prim)
    deformable_api = PhysxSchema.PhysxDeformableAPI(deformable_body_api)

    if solver_position_iteration_count is not None:
        deformable_api.CreateSolverPositionIterationCountAttr().Set(solver_position_iteration_count)
    if vertex_velocity_damping is not None:
        deformable_api.CreateVertexVelocityDampingAttr().Set(vertex_velocity_damping)
    if sleep_damping is not None:
        deformable_api.CreateSleepDampingAttr().Set(sleep_damping)
    if sleep_threshold is not None:
        deformable_api.CreateSleepThresholdAttr().Set(sleep_threshold)
    if settling_threshold is not None:
        deformable_api.CreateSettlingThresholdAttr().Set(settling_threshold)
    if self_collision is not None:
        deformable_api.CreateSelfCollisionAttr().Set(self_collision)
    if self_collision_filter_distance is not None:
        deformable_api.CreateSelfCollisionFilterDistanceAttr().Set(self_collision_filter_distance)

    if collision_indices:
        deformable_body_api.CreateCollisionIndicesAttr().Set(collision_indices)
    if collision_rest_points:
        deformable_body_api.CreateCollisionRestPointsAttr().Set(collision_rest_points)
    if simulation_indices:
        deformable_api.CreateSimulationIndicesAttr().Set(simulation_indices)
    if simulation_rest_points:
        deformable_body_api.CreateSimulationRestPointsAttr().Set(simulation_rest_points)

    # Custom attributes
    if not simulation_rest_points and not collision_rest_points:
        prim.CreateAttribute("physxDeformable:kinematicEnabled", Sdf.ValueTypeNames.Bool).Set(kinematic_enabled)

    if not simulation_rest_points:
        prim.CreateAttribute("physxDeformable:simulationHexahedralResolution", Sdf.ValueTypeNames.UInt).Set(
            simulation_hexahedral_resolution
        )
        prim.CreateAttribute("physxDeformable:numberOfTetsPerHex", Sdf.ValueTypeNames.UInt).Set(
            5
        )

    if not collision_rest_points:
        prim.CreateAttribute("physxDeformable:collisionSimplification", Sdf.ValueTypeNames.Bool).Set(
            collision_simplification
        )
        prim.CreateAttribute("physxDeformable:collisionSimplificationRemeshing", Sdf.ValueTypeNames.Bool).Set(
            collision_simplification_remeshing
        )
        prim.CreateAttribute("physxDeformable:collisionSimplificationRemeshingResolution", Sdf.ValueTypeNames.UInt).Set(
            collision_simplification_remeshing_resolution
        )
        prim.CreateAttribute("physxDeformable:collisionSimplificationTargetTriangleCount", Sdf.ValueTypeNames.UInt).Set(
            collision_simplification_target_triangle_count
        )
        prim.CreateAttribute("physxDeformable:collisionSimplificationForceConforming", Sdf.ValueTypeNames.Bool).Set(
            collision_simplification_force_conforming
        )

    if embedding:
        prim.CreateAttribute("physxDeformable:collisionVertexToSimulationTetIndices", Sdf.ValueTypeNames.IntArray).Set(
            embedding
        )

    # turn on ccd (In the schema, it is off by default)
    deformable_api.CreateEnableCCDAttr().Set(True)

    PhysxSchema.PhysxCollisionAPI.Apply(prim)

    return True


# DEPRECATED
def add_physx_deformable_surface(  # TODO PREIST: Get defaults from schema metadata instead of hardcoding here
    stage,
    prim_path: Sdf.Path,
    simulation_rest_points: typing.List[Gf.Vec3f] = None,
    simulation_indices: typing.List[int] = None,
    bending_stiffness_scale: float = None,
    solver_position_iteration_count: int = None,
    vertex_velocity_damping: float = None,
    sleep_damping: float = None,
    sleep_threshold: float = None,
    settling_threshold: float = None,
    self_collision: bool = None,
    self_collision_filter_distance: float = None
) -> bool:

    """DEPRECATED: Will be replaced by new deformable implementation in future release.
    Applies the deformable surface API to an Xform at prim_path on stage.

    Args:
        stage:                                          The stage
        prim_path:                                      Path to UsdGeom.Mesh 'skin mesh' to which the PhysxSchema.PhysXDeformableBodyAPI is applied to.
        simulation_rest_points:                         List of vertices of the simulation mesh at rest.
        simulation_indices:                             List of indices of the simulation mesh.
        ...:                                            See USD schema for documentation

    Returns:
        True / False that indicates success of schema application
    """

    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        carb.log_warn("No valid primitive prim_path provided")
        return False

    # check if it is a rigid body:
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        carb.log_warn(
            "PhysxSchema.PhysxDeformableSurfaceAPI cannot be applied to a primitive with UsdPhysics.RigidBodyAPI"
        )
        return False

    # check if it is a UsdGeom.Mesh
    if not prim.IsA(UsdGeom.Mesh):
        carb.log_warn("PhysxSchema.PhysxDeformableSurfaceAPI can only be applied to a UsdGeom.Mesh")

    # check simulation mesh
    if simulation_rest_points:
        if len(simulation_rest_points) < 3:
            carb.log_warn("simulation_rest_points is invalid")
            return False
        if not simulation_indices:
            carb.log_warn("simulation mesh invalid")
            return False

    if simulation_indices:
        if len(simulation_indices) < 3 or len(simulation_indices) % 3 != 0:
            carb.log_warn("simulation_indices is invalid")
            return False

    # apply APIs and create attributes
    deformable_surface_api = PhysxSchema.PhysxDeformableSurfaceAPI.Apply(prim)
    deformable_api = PhysxSchema.PhysxDeformableAPI(deformable_surface_api)
    if bending_stiffness_scale is not None:
        carb.log_warn("add_physx_deformable_surface bending_stiffness_scale is ignored, use physxDeformableSurfaceMaterial:bendStiffness instead.")

    if solver_position_iteration_count is not None:
        deformable_api.CreateSolverPositionIterationCountAttr().Set(solver_position_iteration_count)
    if vertex_velocity_damping is not None:
        deformable_api.CreateVertexVelocityDampingAttr().Set(vertex_velocity_damping)
    if sleep_damping is not None:
        deformable_api.CreateSleepDampingAttr().Set(sleep_damping)
    if sleep_threshold is not None:
        deformable_api.CreateSleepThresholdAttr().Set(sleep_threshold)
    if settling_threshold is not None:
        deformable_api.CreateSettlingThresholdAttr().Set(settling_threshold)
    if self_collision is not None:
        deformable_api.CreateSelfCollisionAttr().Set(self_collision)
    if self_collision_filter_distance is not None:
        deformable_api.CreateSelfCollisionFilterDistanceAttr().Set(self_collision_filter_distance)

    if simulation_indices:
        deformable_api.CreateSimulationIndicesAttr().Set(simulation_indices)
    if simulation_rest_points:
        deformable_api.CreateRestPointsAttr().Set(simulation_rest_points)

    PhysxSchema.PhysxCollisionAPI.Apply(prim)

    return True


# DEPRECATED
def add_deformable_body_material(
    stage,
    path,
    damping_scale=None,
    density=None,
    dynamic_friction=None,
    elasticity_damping=None,
    poissons_ratio=None,
    youngs_modulus=None,
):
    """DEPRECATED: Will be replaced by new deformable implementation in future release.
    Applies the PhysxSchema.PhysxDeformableSurfaceMaterialAPI to the prim at path on stage.

    Args:
        stage:                          The stage
        path:                           Path to UsdShade.Material to which the material API should be applied to.
        ... schema attributes:          See USD schema for documentation

    Returns:
        True if the API apply succeeded.
    """
    if not utils.ensureMaterialOnPath(stage, path):
        return False

    material = PhysxSchema.PhysxDeformableBodyMaterialAPI.Apply(stage.GetPrimAtPath(path))

    if damping_scale is not None:
        material.CreateDampingScaleAttr().Set(damping_scale)
    if density is not None:
        material.CreateDensityAttr().Set(density)
    if dynamic_friction is not None:
        material.CreateDynamicFrictionAttr().Set(dynamic_friction)
    if elasticity_damping is not None:
        material.CreateElasticityDampingAttr().Set(elasticity_damping)
    if poissons_ratio is not None:
        material.CreatePoissonsRatioAttr().Set(poissons_ratio)
    if youngs_modulus is not None:
        material.CreateYoungsModulusAttr().Set(youngs_modulus)

    return True


# DEPRECATED
def add_deformable_surface_material(
    stage,
    path,
    density=None,
    dynamic_friction=None,
    poissons_ratio=None,
    thickness=None,
    youngs_modulus=None,
):
    """DEPRECATED: Will be replaced by new deformable implementation in future release.
    Applies the PhysxSchema.PhysxDeformableSurfaceMaterialAPI to the prim at path on stage.

    Args:
        stage:                          The stage
        path:                           Path to UsdShade.Material to which the material API should be applied to.
        ... schema attributes:          See USD schema for documentation

    Returns:
        True if the API apply succeeded.
    """
    if not utils.ensureMaterialOnPath(stage, path):
        return False

    material = PhysxSchema.PhysxDeformableSurfaceMaterialAPI.Apply(stage.GetPrimAtPath(path))

    if density is not None:
        material.CreateDensityAttr().Set(density)
    if dynamic_friction is not None:
        material.CreateDynamicFrictionAttr().Set(dynamic_friction)
    if poissons_ratio is not None:
        material.CreatePoissonsRatioAttr().Set(poissons_ratio)
    if thickness is not None:
        material.CreateThicknessAttr().Set(thickness)
    if youngs_modulus is not None:
        material.CreateYoungsModulusAttr().Set(youngs_modulus)

    # create custom attributes for ux
    material.GetPrim().CreateAttribute("physxDeformableSurfaceMaterial:bendDamping", Sdf.ValueTypeNames.Float).Set(0.0)
    material.GetPrim().CreateAttribute("physxDeformableSurfaceMaterial:elasticityDamping", Sdf.ValueTypeNames.Float).Set(0.0)
    material.GetPrim().CreateAttribute("physxDeformableSurfaceMaterial:bendStiffness", Sdf.ValueTypeNames.Float).Set(0.0)

    return True


def extractTriangleSurfaceFromTetra(tetra_points, tetra_indices):

    # extract all triangles
    triangles = [(-1, -1, -1)] * len(tetra_indices)  # tetra has as many triangles as vertices
    for t in range(0, len(tetra_indices) // 4):
        (v0, v1, v2, v3) = (
            tetra_indices[t * 4],
            tetra_indices[t * 4 + 1],
            tetra_indices[t * 4 + 2],
            tetra_indices[t * 4 + 3],
        )
        triangles[t * 4 + 0] = (v0, v1, v2)
        triangles[t * 4 + 1] = (v1, v3, v2)
        triangles[t * 4 + 2] = (v0, v3, v1)
        triangles[t * 4 + 3] = (v0, v2, v3)

    # extract surface triangles
    surface_triangles_dict = {}
    for i, t in enumerate(triangles):
        vs = sorted([t[0], t[1], t[2]])
        key = (vs[0], vs[1], vs[2])
        if key in surface_triangles_dict:
            del surface_triangles_dict[key]
        else:
            surface_triangles_dict[key] = t

    surface_triangles = list(surface_triangles_dict.values())

    points = []
    indices = []
    tetra_points_to_points = [-1] * len(tetra_points)
    for t in surface_triangles:
        (v0, v1, v2) = t
        if tetra_points_to_points[v0] < 0:
            tetra_points_to_points[v0] = len(points)
            points.append(tetra_points[v0])
        if tetra_points_to_points[v1] < 0:
            tetra_points_to_points[v1] = len(points)
            points.append(tetra_points[v1])
        if tetra_points_to_points[v2] < 0:
            tetra_points_to_points[v2] = len(points)
            points.append(tetra_points[v2])

        indices.extend([tetra_points_to_points[v0], tetra_points_to_points[v1], tetra_points_to_points[v2]])

    return points, indices


# DEPRECATED
def create_skin_mesh_from_tetrahedral_mesh(
    target_path: Sdf.Path, stage: Usd.Stage, source_tetrahedal_mesh_path: Sdf.Path
) -> UsdGeom.Mesh:
    """DEPRECATED: Will be replaced by new deformable implementation in future release."""

    skin_mesh = UsdGeom.Mesh.Define(stage, target_path)
    if skin_mesh:
        collision_mesh = PhysxSchema.TetrahedralMesh(stage.GetPrimAtPath(source_tetrahedal_mesh_path))
        tet_points = collision_mesh.GetPointsAttr().Get()
        tet_indices = collision_mesh.GetIndicesAttr().Get()
        tri_points, tri_indices = extractTriangleSurfaceFromTetra(tet_points, tet_indices)
        skin_mesh.GetPointsAttr().Set(tri_points)
        skin_mesh.GetFaceVertexIndicesAttr().Set(tri_indices)
        skin_mesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))
        skin_mesh.GetSubdivisionSchemeAttr().Set("none")
        physicsUtils.copy_transform_as_scale_orient_translate(collision_mesh, skin_mesh)
    return skin_mesh


def triangulate_mesh(mesh: UsdGeom.Mesh) -> typing.List[int]:
    # indices and faces converted to triangles
    indices = mesh.GetFaceVertexIndicesAttr().Get()
    faces = mesh.GetFaceVertexCountsAttr().Get()

    triangles = []
    if not indices or not faces:
        return triangles

    indices_offset = 0

    for face_count in faces:
        start_index = indices[indices_offset]
        for face_index in range(face_count - 2):
            index1 = indices_offset + face_index + 1
            index2 = indices_offset + face_index + 2
            triangles.append(start_index)
            triangles.append(indices[index1])
            triangles.append(indices[index2])
        indices_offset += face_count

    return triangles


def compute_conforming_tetrahedral_mesh(
    triangle_mesh_points: typing.List[Gf.Vec3f], triangle_mesh_indices: typing.List[int]
) -> typing.Tuple[typing.List[Gf.Vec3f], typing.List[int]]:

    data = get_physx_cooking_interface().compute_conforming_tetrahedral_mesh(
        triangle_mesh_points, triangle_mesh_indices
    )
    conforming_tet_points = [Gf.Vec3f(v.x, v.y, v.z) for v in data["points"]]
    conforming_tet_indices = [index for index in data["indices"]]
    return conforming_tet_points, conforming_tet_indices


def compute_voxel_tetrahedral_mesh(
    tet_mesh_points: typing.List[Gf.Vec3f], tet_mesh_indices: typing.List[int], scale: Gf.Vec3f, resolution: int
) -> typing.Tuple[typing.List[Gf.Vec3f], typing.List[int]]:

    carbScale = carb.Float3(scale[0], scale[1], scale[2])
    data = get_physx_cooking_interface().compute_voxel_tetrahedral_mesh(
        tet_mesh_points, tet_mesh_indices, carbScale, resolution
    )
    voxel_tet_points = [Gf.Vec3f(v.x, v.y, v.z) for v in data["points"]]
    voxel_tet_indices = [index for index in data["indices"]]
    return voxel_tet_points, voxel_tet_indices


def create_triangle_mesh_square(dimx: int, dimy: int, scale: float = 1.0):
    """Creates points and vertex data for a regular-grid flat triangle mesh square.

    Args:
        dimx:                       Mesh-vertex resolution in X
        dimy:                       Mesh-vertex resolution in Y
        scale:                      Uniform scale applied to vertices

    Returns:
        points, indices:            The vertex and index data
    """

    points = [Gf.Vec3f(0.0)] * (dimx + 1) * (dimy + 1)
    indices = [-1] * (dimx * dimy) * 2 * 3

    for y in range(dimy + 1):
        for x in range(dimx + 1):
            points[y * (dimx + 1) + x] = Gf.Vec3f(x, y, 0.0)

    offset = 0
    for y in range(dimy):
        for x in range(dimx):
            v0 = y * (dimx + 1) + x
            v1 = y * (dimx + 1) + x + 1
            v2 = (y + 1) * (dimx + 1) + x
            v3 = (y + 1) * (dimx + 1) + x + 1
            if (x % 2 == 0) != (y % 2 == 0):
                indices[offset] = v0
                indices[offset + 1] = v1
                indices[offset + 2] = v2
                indices[offset + 3] = v1
                indices[offset + 4] = v3
                indices[offset + 5] = v2
            else:
                indices[offset] = v0
                indices[offset + 1] = v1
                indices[offset + 2] = v3
                indices[offset + 3] = v0
                indices[offset + 4] = v3
                indices[offset + 5] = v2
            offset = offset + 6

    for i in range(len(points)):
        p = points[i]
        points[i] = Gf.Vec3f(p[0] / dimx, p[1] / dimy, p[2]) - Gf.Vec3f(0.5, 0.5, 0.0)
        points[i] = Gf.Vec3f(scale * points[i][0], scale * points[i][1], scale * points[i][2])

    return points, indices


def _get_schema_instances(prim: Usd.Prim, schema_type_name: str):
    return {s[len(schema_type_name) + 1:] for s in prim.GetAppliedSchemas() if s.startswith(schema_type_name)}


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
    """

    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        carb.log_warn("set_physics_volume_deformable_body: "+
                      f"No valid primitive prim_path provided: ('{prim_path}')")
        return False

    # check if it is a rigid body:
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        carb.log_warn("set_physics_volume_deformable_body: "+
                      "UsdPhysics.DeformableBodyAPI cannot be applied to a "+
                      f"primitive with UsdPhysics.RigidBodyAPI, {prim.GetPath()}")
        return False

    # check if type is compatible with API
    if not prim.IsA(UsdGeom.TetMesh):
        carb.log_warn("set_physics_volume_deformable_body: "+
                      "Supports only adding UsdPhysics.DeformableBodyAPI to UsdGeom.TetMesh"+
                      f"Use create_auto_volume_deformable_hierarchy for hierarchical setups, {prim.GetPath()}")
        return False

    # apply deformable body api
    if not prim.ApplyAPI("OmniPhysicsDeformableBodyAPI"):
        carb.log_warn("set_physics_volume_deformable_body: "+
                      f"UsdPhysics.DeformableBodyAPI application unsuccessful, {prim.GetPath()}")
        return False

    points = UsdGeom.PointBased(prim).GetPointsAttr().Get()
    indices = UsdGeom.TetMesh(prim).GetTetVertexIndicesAttr().Get()

    if not prim.ApplyAPI("OmniPhysicsVolumeDeformableSimAPI"):
        carb.log_warn("set_physics_volume_deformable_body: " +
                      f"Application of UsdPhysics.VolumeDeformableSimAPI unsuccessfull, {prim.GetPath()}")
        return False

    prim.GetAttribute("omniphysics:restShapePoints").Set(points);
    prim.GetAttribute("omniphysics:restTetVtxIndices").Set(indices);

    if not UsdPhysics.CollisionAPI.Apply(prim):
        carb.log_warn("set_physics_volume_deformable_body: " +
                      f"Application of UsdPhysics.CollisionAPI unsuccessfull, {prim.GetPath()}")
        return False

    # add surface to tetmesh, as it's mandatory to support element collision filters
    surfaceFaceIndices = UsdGeom.TetMesh.ComputeSurfaceFaces(UsdGeom.TetMesh(prim), Usd.TimeCode.Default())
    UsdGeom.TetMesh(prim).GetSurfaceFaceVertexIndicesAttr().Set(surfaceFaceIndices)

    return True


def create_auto_volume_deformable_hierarchy(
    stage: Usd.Stage,
    root_prim_path: typing.Union[str, Sdf.Path],
    simulation_tetmesh_path: typing.Union[str, Sdf.Path],
    collision_tetmesh_path: typing.Union[str, Sdf.Path],
    cooking_src_mesh_path : typing.Union[str, Sdf.Path],
    simulation_hex_mesh_enabled: bool,
    cooking_src_simplification_enabled: bool,
    set_visibility_with_guide_purpose: bool = False
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
    """

    root_prim = stage.GetPrimAtPath(root_prim_path)
    if not root_prim:
        carb.log_warn("create_auto_volume_deformable_hierarchy: "+
                      f"No valid primitive root_prim_path provided: ('{root_prim_path}')")
        return False

    # check if it is a rigid body:
    if root_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        carb.log_warn("create_auto_volume_deformable_hierarchy: "+
                      "UsdPhysics.DeformableBodyAPI cannot be applied to a "+
                      f"primitive with UsdPhysics.RigidBodyAPI, {root_prim.GetPath()}")
        return False

    # check if type is compatible with API
    if root_prim.IsA(UsdGeom.TetMesh):
        carb.log_warn("create_auto_volume_deformable_hierarchy: "+
                      "Use set_physics_volume_deformable_body for applying UsdPhysics.DeformableBodyAPI to UsdGeom.TetMesh"+
                      f", {root_prim.GetPath()}")
        return False

    is_valid_root =  root_prim.IsA(UsdGeom.Imageable) and not root_prim.IsA(UsdGeom.Gprim)
    if not is_valid_root:
        carb.log_warn("create_auto_volume_deformable_hierarchy: "+
                      "UsdPhysics.DeformableBodyAPI for a volume deformable hierarchy can only be applied to a "+
                      f"UsdGeom.Imageable which is not UsdGeom.Gprim, {root_prim.GetPath()}")
        return False

    if Sdf.Path(simulation_tetmesh_path).GetParentPath() != root_prim.GetPath():
        carb.log_warn("create_auto_volume_deformable_hierarchy: "+
                      "simulation_tetmesh_path needs to be immediate child path of root_prim_path: "+
                      f"simulation_tetmesh_path: {simulation_tetmesh_path}, root_prim_path: {root_prim_path}")
        return False

    if Sdf.Path(collision_tetmesh_path).GetParentPath() != root_prim.GetPath():
        carb.log_warn("create_auto_volume_deformable_hierarchy: "+
                      "collision_tetmesh_path needs to be immediate child path of root_prim_path: "+
                      f"collision_tetmesh_path: {collision_tetmesh_path}, root_prim_path: {root_prim_path}")
        return False

    cooking_src_prim = stage.GetPrimAtPath(cooking_src_mesh_path)
    if not cooking_src_prim or not cooking_src_prim.IsA(UsdGeom.Mesh):
        carb.log_warn("create_auto_volume_deformable_hierarchy: "+
                      f"cooking_src_mesh_path needs to point to valid UsdGeom.Mesh, root_prim_path: {root_prim_path}")
        return False

    # remove potential previous configuration
    remove_deformable_body(stage, root_prim.GetPath())

    # apply auto deformable body api and set source meshes
    if not root_prim.ApplyAPI("PhysxAutoDeformableBodyAPI"):
        carb.log_warn("create_auto_volume_deformable_hierarchy: "+
                      f"PhysxAutoDeformableBodyAPI application unsuccessful, {root_prim.GetPath()}")
        return False

    root_prim.GetRelationship("physxDeformableBody:cookingSourceMesh").SetTargets([cooking_src_mesh_path])

    # apply simplification api
    if cooking_src_simplification_enabled:
        if not add_auto_deformable_mesh_simplification(stage, root_prim.GetPath()):
            return False

    # apply hex api
    if simulation_hex_mesh_enabled:
        if not root_prim.ApplyAPI("PhysxAutoDeformableHexahedralMeshAPI"):
            carb.log_warn("create_auto_volume_deformable_hierarchy: failed to apply PhysxAutoDeformableHexahedralMeshAPI, "+
                          f"{root_prim_path}")

    sim_tet_mesh = UsdGeom.TetMesh.Define(stage, simulation_tetmesh_path)
    if not sim_tet_mesh:
        carb.log_warn("create_auto_volume_deformable_hierarchy: failed to define UsdGeom.TetMesh at: "+
                      f"{simulation_tetmesh_path}")
        return False

    # apply sim API
    if not sim_tet_mesh.GetPrim().ApplyAPI("OmniPhysicsVolumeDeformableSimAPI"):
        carb.log_warn("create_auto_volume_deformable_hierarchy: "+
                      f"UsdPhysics.VolumeDeformableSimAPI application unsuccessful, {tet_mesh.GetPath()}")
        return False

    coll_tet_mesh = sim_tet_mesh
    if collision_tetmesh_path != simulation_tetmesh_path:
        coll_tet_mesh = UsdGeom.TetMesh.Define(stage, collision_tetmesh_path)
        if not coll_tet_mesh:
            carb.log_warn("create_auto_volume_deformable_hierarchy: failed to define UsdGeom.TetMesh at: "+
                          f"{collision_tetmesh_path}")
            return False

    # apply collision API
    if not coll_tet_mesh.GetPrim().ApplyAPI(UsdPhysics.CollisionAPI):
        carb.log_warn("create_auto_volume_deformable_hierarchy: "+
                      f"UsdPhysics.CollisionAPI application unsuccessful, {coll_tet_mesh.GetPath()}")
        return False

    # visit hierarchy and collect purely visual geometries
    visual_geometry_prims = []
    prim_range = Usd.PrimRange(root_prim, Usd.PrimAllPrimsPredicate)
    for prim in prim_range:
        # skip the root itself for the special transform check
        if prim != root_prim:
            xformable = UsdGeom.Xformable(prim)
            if xformable and xformable.GetResetXformStack():
                # prune the subtree under this prim and move on
                prim_range.PruneChildren()
                continue

        # check if it's a UsdGeomPointBased, skip if not
        if not prim.IsA(UsdGeom.PointBased):
            continue

        # skip if sim or coll mesh
        if prim == sim_tet_mesh.GetPrim() or prim == coll_tet_mesh.GetPrim():
            continue

        visual_geometry_prims.append(prim)

    # apply bind pose deformable pose API
    purposes = ["bindPose"]
    for prim in visual_geometry_prims:
        prim.ApplyAPI("OmniPhysicsDeformablePoseAPI", "default")
        prim.CreateAttribute("deformablePose:default:omniphysics:purposes", Sdf.ValueTypeNames.TokenArray).Set(purposes)
        point_based = UsdGeom.PointBased(prim)
        points = point_based.GetPointsAttr().Get()
        prim.CreateAttribute("deformablePose:default:omniphysics:points", Sdf.ValueTypeNames.Point3fArray).Set(points)

    sim_tet_mesh.GetPrim().ApplyAPI("OmniPhysicsDeformablePoseAPI", "default")
    sim_tet_mesh.GetPrim().CreateAttribute("deformablePose:default:omniphysics:purposes", Sdf.ValueTypeNames.TokenArray).Set(purposes)
    coll_tet_mesh.GetPrim().ApplyAPI("OmniPhysicsDeformablePoseAPI", "default")
    coll_tet_mesh.GetPrim().CreateAttribute("deformablePose:default:omniphysics:purposes", Sdf.ValueTypeNames.TokenArray).Set(purposes)

    # apply guide purpose for visibility, if requested
    if set_visibility_with_guide_purpose:
        if sim_tet_mesh != coll_tet_mesh:
            UsdGeom.Imageable(sim_tet_mesh).GetPurposeAttr().Set(UsdGeom.Tokens.guide)
        elif len(visual_geometry_prims) > 0:
            UsdGeom.Imageable(coll_tet_mesh).GetPurposeAttr().Set(UsdGeom.Tokens.guide)

    # apply deformable body api
    # HACK to trigger async cooking later - WHY is the stage update triggered asynchronously with this code?
    if not root_prim.ApplyAPI("OmniPhysicsDeformableBodyAPI"):
        carb.log_warn("create_auto_volume_deformable_hierarchy: "+
                      f"UsdPhysics.DeformableBodyAPI application unsuccessful, {root_prim.GetPath()}")
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
    """

    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        carb.log_warn("set_physics_surface_deformable_body: "+
                      f"No valid primitive prim_path provided: ('{prim_path}')")
        return False

    # check if it is a rigid body:
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        carb.log_warn("set_physics_surface_deformable_body: "+
                      "UsdPhysics.DeformableBodyAPI cannot be applied to a "+
                      f"primitive with UsdPhysics.RigidBodyAPI, {prim.GetPath()}")
        return False

    # check if type is compatible with API
    if not prim.IsA(UsdGeom.Mesh):
        carb.log_warn("set_physics_surface_deformable_body: "+
                      "Supports only adding UsdPhysics.DeformableBodyAPI to UsdGeom.Mesh"+
                      f"Use create_auto_surface_deformable_hierarchy for hierarchical setups, {prim.GetPath()}")
        return False

    mesh = UsdGeom.Mesh(prim)
    if not mesh:
        return False

    points = mesh.GetPointsAttr().Get()
    face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get()
    face_vertex_indices = mesh.GetFaceVertexIndicesAttr().Get()

    tri_indices = [Gf.Vec3i(0,0,0)]*len(face_vertex_counts)
    for i, face_count in enumerate(face_vertex_counts):
        if face_count != 3:
            carb.log_warn("set_physics_surface_deformable_body: " +
                          f"UsdPhysics.SurfaceDeformableSimAPI can only be applied to meshes limited to " +
                          f"triangular faces, {prim.GetPath()}")
            return False
        tri_indices[i] = Gf.Vec3i(face_vertex_indices[3*i + 0],
                                  face_vertex_indices[3*i + 1],
                                  face_vertex_indices[3*i + 2])

    # apply deformable body api
    if not prim.ApplyAPI("OmniPhysicsDeformableBodyAPI"):
        carb.log_warn("set_physics_surface_deformable_body: "+
                      f"UsdPhysics.DeformableBodyAPI application unsuccessful, {prim.GetPath()}")
        return False

    if not prim.HasAPI("OmniPhysicsSurfaceDeformableSimAPI"):
        if not prim.ApplyAPI("OmniPhysicsSurfaceDeformableSimAPI"):
            carb.log_warn("set_physics_surface_deformable_body: " +
                          f"Application of UsdPhysics.SurfaceDeformableSimAPI unsuccessfull, {prim.GetPath()}")
            return False

    prim.GetAttribute("omniphysics:restShapePoints").Set(points);
    prim.GetAttribute("omniphysics:restTriVtxIndices").Set(tri_indices);

    if not UsdPhysics.CollisionAPI.Apply(prim):
        carb.log_warn("set_physics_surface_deformable_body: " +
                      f"Application of UsdPhysics.CollisionAPI unsuccessfull, {prim.GetPath()}")
        return False

    return True


def create_auto_surface_deformable_hierarchy(
    stage: Usd.Stage,
    root_prim_path: typing.Union[str, Sdf.Path],
    simulation_mesh_path: typing.Union[str, Sdf.Path],
    cooking_src_mesh_path : typing.Union[str, Sdf.Path],
    cooking_src_simplification_enabled: bool,
    set_visibility_with_guide_purpose: bool = False
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
    """

    root_prim = stage.GetPrimAtPath(root_prim_path)
    if not root_prim:
        carb.log_warn("create_auto_surface_deformable_hierarchy: "+
                      f"No valid primitive root_prim_path provided: ('{root_prim_path}')")
        return False

    # check if it is a rigid body:
    if root_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        carb.log_warn("create_auto_surface_deformable_hierarchy: "+
                      "UsdPhysics.DeformableBodyAPI cannot be applied to a "+
                      f"primitive with UsdPhysics.RigidBodyAPI, {root_prim.GetPath()}")
        return False

    # check if type is compatible with API
    if root_prim.IsA(UsdGeom.Mesh):
        carb.log_warn("create_auto_surface_deformable_hierarchy: "+
                      "Use set_physics_surface_deformable_body for applying UsdPhysics.DeformableBodyAPI to UsdGeom.Mesh"+
                      f", {root_prim.GetPath()}")
        return False

    is_valid_root =  root_prim.IsA(UsdGeom.Imageable) and not root_prim.IsA(UsdGeom.Gprim)
    if not is_valid_root:
        carb.log_warn("create_auto_surface_deformable_hierarchy: "+
                      "UsdPhysics.DeformableBodyAPI for a volume deformable hierarchy can only be applied to a "+
                      f"UsdGeom.Imageable which is not UsdGeom.Gprim, {root_prim.GetPath()}")
        return False

    if Sdf.Path(simulation_mesh_path).GetParentPath() != root_prim.GetPath():
        carb.log_warn("create_auto_surface_deformable_hierarchy: "+
                      "simulation_mesh_path needs to be immediate child path of root_prim_path: "+
                      f"simulation_mesh_path: {simulation_mesh_path}, root_prim_path: {root_prim_path}")
        return False

    cooking_src_prim = stage.GetPrimAtPath(cooking_src_mesh_path)
    if not cooking_src_prim or not cooking_src_prim.IsA(UsdGeom.Mesh):
        carb.log_warn("create_auto_surface_deformable_hierarchy: "+
                      f"cooking_src_mesh_path needs to point to valid UsdGeom.Mesh, root_prim_path: {root_prim_path}")
        return False

    # remove potential previous configuration
    remove_deformable_body(stage, root_prim.GetPath())

    # apply auto deformable body api and set source meshes
    if not root_prim.ApplyAPI("PhysxAutoDeformableBodyAPI"):
        carb.log_warn("create_auto_surface_deformable_hierarchy: "+
                      f"PhysxAutoDeformableBodyAPI application unsuccessful, {root_prim.GetPath()}")
        return False

    root_prim.GetRelationship("physxDeformableBody:cookingSourceMesh").SetTargets([cooking_src_mesh_path])

    # apply simplification api
    if cooking_src_simplification_enabled:
        if not add_auto_deformable_mesh_simplification(stage, root_prim.GetPath()):
            return False

    sim_mesh = UsdGeom.Mesh.Define(stage, simulation_mesh_path)
    if not sim_mesh:
        carb.log_warn("create_auto_surface_deformable_hierarchy: failed to define UsdGeom.Mesh at: "+
                      f"{simulation_mesh_path}")
        return False

    # apply sim API
    if not sim_mesh.GetPrim().ApplyAPI("OmniPhysicsSurfaceDeformableSimAPI"):
        carb.log_warn("create_auto_surface_deformable_hierarchy: "+
                      f"UsdPhysics.SurfaceDeformableSimAPI application unsuccessful, {sim_mesh.GetPath()}")
        return False

    # apply collision API
    if not sim_mesh.GetPrim().ApplyAPI(UsdPhysics.CollisionAPI):
        carb.log_warn("create_auto_surface_deformable_hierarchy: "+
                      f"UsdPhysics.CollisionAPI application unsuccessful, {sim_mesh.GetPath()}")
        return False

    # visit hierarchy and collect purely visual geometries
    visual_geometry_prims = []
    prim_range = Usd.PrimRange(root_prim, Usd.PrimAllPrimsPredicate)
    for prim in prim_range:
        # skip the root itself for the special transform check
        if prim != root_prim:
            xformable = UsdGeom.Xformable(prim)
            if xformable and xformable.GetResetXformStack():
                # prune the subtree under this prim and move on
                prim_range.PruneChildren()
                continue

        # check if it's a UsdGeomPointBased, skip if not
        if not prim.IsA(UsdGeom.PointBased):
            continue

        # skip if sim mesh
        if prim == sim_mesh.GetPrim():
            continue

        visual_geometry_prims.append(prim)

    # apply bind pose deformable pose API
    purposes = ["bindPose"]
    for prim in visual_geometry_prims:
        prim.ApplyAPI("OmniPhysicsDeformablePoseAPI", "default")
        prim.CreateAttribute("deformablePose:default:omniphysics:purposes", Sdf.ValueTypeNames.TokenArray).Set(purposes)
        point_based = UsdGeom.PointBased(prim)
        points = point_based.GetPointsAttr().Get()
        prim.CreateAttribute("deformablePose:default:omniphysics:points", Sdf.ValueTypeNames.Point3fArray).Set(points)

    sim_mesh.GetPrim().ApplyAPI("OmniPhysicsDeformablePoseAPI", "default")
    sim_mesh.GetPrim().CreateAttribute("deformablePose:default:omniphysics:purposes", Sdf.ValueTypeNames.TokenArray).Set(purposes)

    # apply guide purpose for visibility, if requested
    if set_visibility_with_guide_purpose:
        if len(visual_geometry_prims) > 0:
            UsdGeom.Imageable(sim_mesh).GetPurposeAttr().Set(UsdGeom.Tokens.guide)

    # apply deformable body api
    # HACK to trigger async cooking later - WHY is the stage update triggered asynchronously with this code?
    if not root_prim.ApplyAPI("OmniPhysicsDeformableBodyAPI"):
        carb.log_warn("create_auto_surface_deformable_hierarchy: "+
                      f"UsdPhysics.DeformableBodyAPI application unsuccessful, {root_prim.GetPath()}")
        return False

    return True


def remove_auto_deformable_body(
    stage,
    prim_path: Sdf.Path,
):
    prim = stage.GetPrimAtPath(prim_path)
    if prim:
        prim.RemoveAPI("PhysxAutoDeformableBodyAPI")
        utils.removeAPISchemaProperties("PhysxAutoDeformableBodyAPI", prim)
        # custom properties
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
    prim = stage.GetPrimAtPath(prim_path)
    if prim:
        prim.RemoveAPI("PhysxAutoDeformableHexahedralMeshAPI")
        utils.removeAPISchemaProperties("PhysxAutoDeformableHexahedralMeshAPI", prim)


def add_auto_deformable_mesh_simplification(
    stage,
    prim_path: Sdf.Path,
) -> bool:

    """Add a simplification collision mesh on a prim with PhysxSchema.PhysxAutoDeformableBodyAPI, and setup the deformable
       body correspondingly.

    Args:
        stage:                          The stage
        prim_path:                      Path to UsdGeom.Scope/UsdGeom.Xform
                                        to which the PhysxSchema.PhysxAutoDeformableBodyAPI is applied to.

    Returns:
        True / False that indicates success of schema application
    """

    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        carb.log_warn("No valid primitive prim_path provided")
        return False

    # check if type is compatible with API
    if not (prim.IsA(UsdGeom.Scope) or prim.IsA(UsdGeom.Xform)):
        carb.log_warn("PhysxSchema.PhysxAutoDeformableMeshSimplificationAPI can only be applied to a UsdGeom.Scope or UsdGeom.Xform")
        return False

    # check if prim has PhysxSchema.PhysxAutoDeformableBodyAPI
    if not prim.HasAPI("PhysxAutoDeformableBodyAPI"):
        carb.log_warn("PhysxSchema.PhysxAutoDeformableMeshSimplificationAPI can only be applied to a prim with PhysxSchema.PhysxAutoDeformableBodyAPI")
        return False

    # apply deformable body api
    if not prim.ApplyAPI("PhysxAutoDeformableMeshSimplificationAPI"):
        carb.log_warn("PhysxSchema.PhysxAutoDeformableMeshSimplificationAPI application unsuccessful")
        return False

    return True


def remove_auto_deformable_mesh_simplification(
    stage,
    prim_path: Sdf.Path,
):
    prim = stage.GetPrimAtPath(prim_path)
    if prim:
        prim.RemoveAPI("PhysxAutoDeformableMeshSimplificationAPI")
        utils.removeAPISchemaProperties("PhysxAutoDeformableMeshSimplificationAPI", prim)


def remove_deformable_body(
    stage,
    prim_path: Sdf.Path,
):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        return

    remove_auto_deformable_body(stage, prim_path)

    prim.RemoveAPI("PhysxBaseDeformableBodyAPI")
    utils.removeAPISchemaProperties("PhysxBaseDeformableBodyAPI", prim)

    prim.RemoveAPI("PhysxSurfaceDeformableBodyAPI")
    utils.removeAPISchemaProperties("PhysxSurfaceDeformableBodyAPI", prim)

    prim.RemoveAPI("OmniPhysicsDeformableBodyAPI")
    utils.removeAPISchemaProperties("OmniPhysicsDeformableBodyAPI", prim)

    #for non-hierarchical
    prim.RemoveAPI("OmniPhysicsVolumeDeformableSimAPI")
    utils.removeAPISchemaProperties("OmniPhysicsVolumeDeformableSimAPI", prim)
    prim.RemoveAPI("OmniPhysicsSurfaceDeformableSimAPI")
    utils.removeAPISchemaProperties("OmniPhysicsSurfaceDeformableSimAPI", prim)
    prim.RemoveAPI(UsdPhysics.CollisionAPI)
    utils.removeAPISchemaProperties(UsdPhysics.CollisionAPI, prim)

    # custom properties
    attributes_to_remove = [x.GetName() for x in prim.GetAuthoredPropertiesInNamespace(["physxDeformableBody"])]
    for attribute in attributes_to_remove:
        prim.RemoveProperty(attribute)

    # iterate children to find sim mesh and skin meshes to remove deformable APIs
    # Note that we don't remove the potentially automatically
    # created UsdGeom.TetMesh simulation mesh, as it might be user owned.
    children = prim.GetChildren()
    for child in children:
        if child.HasAPI("OmniPhysicsVolumeDeformableSimAPI"):
            child.RemoveAPI("OmniPhysicsVolumeDeformableSimAPI")
            utils.removeAPISchemaProperties("OmniPhysicsVolumeDeformableSimAPI", child)
        if child.HasAPI("OmniPhysicsSurfaceDeformableSimAPI"):
            child.RemoveAPI("OmniPhysicsSurfaceDeformableSimAPI")
            utils.removeAPISchemaProperties("OmniPhysicsSurfaceDeformableSimAPI", child)
        if child.HasAPI(UsdPhysics.CollisionAPI):
            child.RemoveAPI(UsdPhysics.CollisionAPI)
            utils.removeAPISchemaProperties(UsdPhysics.CollisionAPI, child)

        #pose_apis = UsdPhysics.DeformablePoseAPI.GetAll(child)
        #unfortunately we can't access the instance name from a multi applied schema
        #in python...
        pose_instance_names = _get_schema_instances(child, "OmniPhysicsDeformablePoseAPI")
        for pose_instance_name in pose_instance_names:
            child.RemoveAPI("OmniPhysicsDeformablePoseAPI", pose_instance_name)
            utils.removeMultipleAPISchemaProperties("OmniPhysicsDeformablePoseAPI", child, "deformablePose", pose_instance_name)


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

    if not utils.ensureMaterialOnPath(stage, path):
        return False

    prim = stage.GetPrimAtPath(path)
    if not prim.ApplyAPI("OmniPhysicsDeformableMaterialAPI"):
        carb.log_warn("OmniPhysicsDeformableMaterialAPI application unsuccessful")
        return False

    #same defaults as in usdLoad/Materials.cpp
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    kpu = UsdPhysics.GetStageKilogramsPerUnit(stage)
    density = -1.0 if density is None else density
    static_friction = 0.5 if static_friction is None else static_friction
    dynamic_friction = 0.25 if dynamic_friction is None else dynamic_friction
    youngs_modulus = (5.0e5*mpu/kpu) if youngs_modulus is None else youngs_modulus
    poissons_ratio = 0.45 if poissons_ratio is None else poissons_ratio

    prim.GetAttribute("omniphysics:density").Set(density)
    prim.GetAttribute("omniphysics:staticFriction").Set(static_friction)
    prim.GetAttribute("omniphysics:dynamicFriction").Set(dynamic_friction)
    prim.GetAttribute("omniphysics:youngsModulus").Set(youngs_modulus)
    prim.GetAttribute("omniphysics:poissonsRatio").Set(poissons_ratio)

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
    surface_bend_stiffness=None
):
    """Applies the UsdPhysics.SurfaceDeformableMaterialAPI to the prim at path on stage.

    Args:
        stage:                          The stage
        path:                           Path to UsdShade.Material to which the material API should be applied to.
        ... schema attributes:          See USD schema for documentation

    Returns:
        True if the API apply succeeded.
    """
    if not add_deformable_material(stage, path,
        density=density,
        static_friction=static_friction,
        dynamic_friction=dynamic_friction,
        youngs_modulus=youngs_modulus,
        poissons_ratio=poissons_ratio
    ):
        return False

    prim = stage.GetPrimAtPath(path)
    if not prim.ApplyAPI("OmniPhysicsSurfaceDeformableMaterialAPI"):
        carb.log_warn("OmniPhysicsSurfaceDeformableMaterialAPI application unsuccessful")
        return False

    #same defaults as in usdLoad/Materials.cpp
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    surface_thickness = (0.001/mpu) if surface_thickness is None else surface_thickness
    surface_stretch_stiffness = 0.0 if surface_stretch_stiffness is None else surface_stretch_stiffness
    surface_shear_stiffness = 0.0 if surface_shear_stiffness is None else surface_shear_stiffness
    surface_bend_stiffness = 0.0 if surface_bend_stiffness is None else surface_bend_stiffness

    prim.GetAttribute("omniphysics:surfaceThickness").Set(surface_thickness)
    prim.GetAttribute("omniphysics:surfaceStretchStiffness").Set(surface_stretch_stiffness)
    prim.GetAttribute("omniphysics:surfaceShearStiffness").Set(surface_shear_stiffness)
    prim.GetAttribute("omniphysics:surfaceBendStiffness").Set(surface_bend_stiffness)

    return True


def create_auto_deformable_attachment(
    stage: Usd.Stage,
    target_attachment_path: Sdf.Path,
    attachable0_path: Sdf.Path,
    attachable1_path: Sdf.Path) -> bool:

    if stage.GetPrimAtPath(target_attachment_path):
        carb.log_warn(f"create_auto_deformable_attachment: pre-existing prim found at {target_attachment_path}")
        return False

    attachable0 = stage.GetPrimAtPath(attachable0_path)
    attachable1 = stage.GetPrimAtPath(attachable1_path)

    if not attachable0:
        carb.log_warn(f"create_auto_deformable_attachment: invalid attachable found at {attachable0_path}")
        return False

    if not attachable1:
        carb.log_warn(f"create_auto_deformable_attachment: invalid attachable found at {attachable1_path}")
        return False

    attachable0_is_deformable = attachable0.HasAPI("OmniPhysicsDeformableBodyAPI")
    attachable1_is_deformable = attachable1.HasAPI("OmniPhysicsDeformableBodyAPI")

    attachable0_is_volume = False
    attachable1_is_volume = False
    attachable0_is_surface = False
    attachable1_is_surface = False

    vsimType = Usd.SchemaRegistry().GetTypeFromSchemaTypeName("OmniPhysicsVolumeDeformableSimAPI")
    ssimType = Usd.SchemaRegistry().GetTypeFromSchemaTypeName("OmniPhysicsSurfaceDeformableSimAPI")
    dbType = Usd.SchemaRegistry().GetTypeFromSchemaTypeName("OmniPhysicsDeformableBodyAPI")

    if attachable0_is_deformable:
        attachable0_is_volume = pxb.descendantHasAPI(vsimType, attachable0)
        attachable0_is_surface = pxb.descendantHasAPI(ssimType, attachable0)

    if attachable1_is_deformable:
        attachable1_is_volume = pxb.descendantHasAPI(vsimType, attachable1)
        attachable1_is_surface = pxb.descendantHasAPI(ssimType, attachable1)

    attachable0_is_xformable = not pxb.ancestorHasAPI(dbType, attachable0) and attachable0.IsA(UsdGeom.Xformable)
    attachable1_is_xformable = not pxb.ancestorHasAPI(dbType, attachable1) and attachable1.IsA(UsdGeom.Xformable)

    if not attachable0_is_deformable and not attachable1_is_deformable:
        carb.log_warn(f"create_auto_deformable_attachment: neither attachables have UsdPhysicsDeformableBodyAPI: {attachable0_path}, {attachable1_path}")
        return False

    if attachable0_is_deformable and not attachable1_is_deformable and not attachable1_is_xformable:
        carb.log_warn(f"create_auto_deformable_attachment: non-deformable attachable needs to be UsdGeomXformable and cannot be child of deformable body: {attachable1_path}")
        return False

    if attachable1_is_deformable and not attachable0_is_deformable and not attachable0_is_xformable:
        carb.log_warn(f"create_auto_deformable_attachment: non-deformable attachable needs to be UsdGeomXformable and cannot be child of deformable body: {attachable0_path}")
        return False

    if attachable0_is_deformable and (not attachable0_is_volume and not attachable0_is_surface):
        carb.log_warn(f"create_auto_deformable_attachment: found unknown or incomplete deformable body: {attachable0_path}")
        return False

    if attachable1_is_deformable and (not attachable1_is_volume and not attachable1_is_surface):
        carb.log_warn(f"create_auto_deformable_attachment: found unknown or incomplete deformable body: {attachable1_path}")
        return False

    #create scope primitive, apply auto attachment API

    scope = UsdGeom.Scope.Define(stage, target_attachment_path)
    if not scope:
        carb.log_warn("create_auto_deformable_attachment: failed to create UsdGeomScope")
        return False

    if not scope.GetPrim().ApplyAPI("PhysxAutoDeformableAttachmentAPI"):
        carb.log_warn("create_auto_deformable_attachment: failed to apply PhysxAutoDeformableAttachmentAPI")
        return False

    scope.GetPrim().GetRelationship("physxAutoDeformableAttachment:attachable0").SetTargets([attachable0_path])
    scope.GetPrim().GetRelationship("physxAutoDeformableAttachment:attachable1").SetTargets([attachable1_path])

    #setup all needed primitives, all the actual attachment data is filled out by the attachment authoring
    physx_attachment_private_interface = get_physx_attachment_private_interface()
    return physx_attachment_private_interface.setup_auto_deformable_attachment(str(target_attachment_path))
