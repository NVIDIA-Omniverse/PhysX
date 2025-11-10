# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
import typing
import carb
import re
from omni.physx.bindings._physx import SimulationEvent, ContactEventType, SETTING_UPDATE_TO_USD
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdUtils, UsdGeom, UsdPhysics, UsdShade
import omni.usd
import usdrt
from omni.physx import get_physx_simulation_interface


HALF_PI = 1.57079632679489662
MAX_FLOAT = 3.40282347e38


def get_stage_next_free_path(
    stage: Usd.Stage, path: typing.Union[str, Sdf.Path], prepend_default_prim: bool
) -> str:
    """
    Gets valid path in stage, if the path already exists it will append number.

    Args:
        stage:      The Usd.Stage to add path.
        path:       The desired path to create.
        prepend_default_prim: Whether prepend default prim path name.        
    """    
    if isinstance(path, str) and not Sdf.Path.IsValidPathString(path):
        raise ValueError(f"{path} is not a valid path")

    path = Sdf.Path(path)
    # If path is missing leading slash, it's still ValidPathString but may crash in other USD api. Correct it here and issue a warning.
    corrected_path = path.MakeAbsolutePath(Sdf.Path.absoluteRootPath)
    if path != corrected_path:
        carb.log_warn(f"Path {path} is auto-corrected to {corrected_path}. Please verify your path format.")
        path = corrected_path

    if prepend_default_prim and stage.HasDefaultPrim():
        defaultPrim = stage.GetDefaultPrim()
        if defaultPrim and not (path.HasPrefix(defaultPrim.GetPath()) and path != defaultPrim.GetPath()):
            path = path.ReplacePrefix(Sdf.Path.absoluteRootPath, defaultPrim.GetPath())

    def increment_path(path):
        match = re.search("_(\d+)$", path)
        if match:
            new_num = int(match.group(1)) + 1
            ret = re.sub("_(\d+)$", str.format("_{:02d}", new_num), path)
        else:
            ret = path + "_01"
        return ret

    path_string = path.pathString
    while stage.GetPrimAtPath(path_string):
        path_string = increment_path(path_string)

    return path_string

def compute_bounding_box_diagonal(points: typing.List[carb.Float3]) -> float:
    """
    Gets diagonal length of given point bounds.

    Args:
        points:      The input points.
    """     
    v_min_x = MAX_FLOAT
    v_min_y = MAX_FLOAT
    v_min_z = MAX_FLOAT
    v_max_x = -MAX_FLOAT
    v_max_y = -MAX_FLOAT
    v_max_z = -MAX_FLOAT
    for v in points:
        if v.x < v_min_x:
            v_min_x = v.x
        if v.x > v_max_x:
            v_max_x = v.x
        if v.y < v_min_y:
            v_min_y = v.y
        if v.y > v_max_y:
            v_max_y = v.y
        if v.z < v_min_z:
            v_min_z = v.z
        if v.z > v_max_z:
            v_max_z = v.z
    return math.sqrt((v_max_x - v_min_x) ** 2 + (v_max_y - v_min_y) ** 2 + (v_max_z - v_min_z) ** 2)


def create_mesh(
    stage: Usd.Stage, 
    path: typing.Union[str, Sdf.Path], 
    points : typing.Union[typing.List[Gf.Vec3f], typing.List[Gf.Vec3d]], 
    normals : typing.Union[typing.List[Gf.Vec3f], typing.List[Gf.Vec3d]], 
    indices : typing.List[int], 
    vertexCounts : typing.List[Gf.Vec3f]
) -> UsdGeom.Mesh:
    """
    Create UsdGeom.Mesh from given points, normals, indices and face counts.

    Args:
        stage:      The Usd.Stage to add path.
        path:       The desired path to create.
        points:     The input points.
        normals:    The input normals.
        indices:    The indices for faces.
        vertexCounts:     Face counts.
    """     
    mesh = UsdGeom.Mesh.Define(stage, path)

    # Fill in VtArrays
    mesh.CreateFaceVertexCountsAttr().Set(vertexCounts)
    mesh.CreateFaceVertexIndicesAttr().Set(indices)
    mesh.CreatePointsAttr().Set(points)
    mesh.CreateDoubleSidedAttr().Set(False)
    mesh.CreateNormalsAttr().Set(normals)

    return mesh


def create_mesh_square_axis(
    stage: Usd.Stage, 
    path: typing.Union[str, Sdf.Path], 
    axis: str, 
    halfSize: float
) -> UsdGeom.Mesh:
    """
    Create UsdGeom.Mesh that represents a square.

    Args:
        stage:      The Usd.Stage to add path.
        path:       The desired path to create.
        axis:       The up axis "Y", "Z".
        halfSize:   The half size of the square.
    """     
    if axis == "X":
        points = [
            Gf.Vec3f(0.0, -halfSize, -halfSize),
            Gf.Vec3f(0.0, halfSize, -halfSize),
            Gf.Vec3f(0.0, halfSize, halfSize),
            Gf.Vec3f(0.0, -halfSize, halfSize),
        ]
        normals = [Gf.Vec3f(1, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(1, 0, 0)]
        indices = [0, 1, 2, 3]
        vertexCounts = [4]

        # Create the mesh
        return create_mesh(stage, path, points, normals, indices, vertexCounts)
    elif axis == "Y":
        points = [
            Gf.Vec3f(-halfSize, 0.0, -halfSize),
            Gf.Vec3f(halfSize, 0.0, -halfSize),
            Gf.Vec3f(halfSize, 0.0, halfSize),
            Gf.Vec3f(-halfSize, 0.0, halfSize),
        ]
        normals = [Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 1, 0)]
        indices = [3, 2, 1, 0]
        vertexCounts = [4]

        # Create the mesh
        return create_mesh(stage, path, points, normals, indices, vertexCounts)

    points = [
        Gf.Vec3f(-halfSize, -halfSize, 0.0),
        Gf.Vec3f(halfSize, -halfSize, 0.0),
        Gf.Vec3f(halfSize, halfSize, 0.0),
        Gf.Vec3f(-halfSize, halfSize, 0.0),
    ]
    normals = [Gf.Vec3f(0, 0, 1), Gf.Vec3f(0, 0, 1), Gf.Vec3f(0, 0, 1), Gf.Vec3f(0, 0, 1)]
    indices = [0, 1, 2, 3]
    vertexCounts = [4]

    # Create the mesh
    mesh = create_mesh(stage, path, points, normals, indices, vertexCounts)

    # text coord
    texCoords = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.varying)
    texCoords.Set([(0, 0), (1, 0), (1, 1), (0, 1)])

    return mesh

def create_mesh_concave(
    stage: Usd.Stage, 
    path: typing.Union[str, Sdf.Path], 
    halfSize: float
) -> UsdGeom.Mesh:
    """
    Create UsdGeom.Mesh that represents a concave mesh.

    Args:
        stage:      The Usd.Stage to add path.
        path:       The desired path to create.
        halfSize:   The half size of the mesh.
    """
    points = [
        Gf.Vec3f(halfSize, -halfSize, -halfSize),
        Gf.Vec3f(halfSize, halfSize, -halfSize),
        Gf.Vec3f(halfSize, halfSize, halfSize),
        Gf.Vec3f(halfSize, -halfSize, halfSize),
        Gf.Vec3f(0.0, -halfSize, halfSize * 0.2),
        Gf.Vec3f(0.0, halfSize, halfSize * 0.2),
        Gf.Vec3f(-halfSize, -halfSize, -halfSize),
        Gf.Vec3f(-halfSize, halfSize, -halfSize),
        Gf.Vec3f(-halfSize, halfSize, halfSize),
        Gf.Vec3f(-halfSize, -halfSize, halfSize),
    ]
    normals = [Gf.Vec3f(1, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(1, 0, 0),
        Gf.Vec3f(0, 0, 1), Gf.Vec3f(0, 0, 1),
        Gf.Vec3f(-1, 0, 0), Gf.Vec3f(-1, 0, 0), Gf.Vec3f(-1, 0, 0), Gf.Vec3f(-1, 0, 0)]
    indices = [
        0, 1, 2, 3,
        1, 7, 8, 5, 2,
        3, 2, 5, 4,
        4, 5, 8, 9,
        9, 8, 7, 6,
        0, 6, 7, 1,
        0, 3, 4, 9, 6]
    vertexCounts = [4, 5, 4, 4, 4, 4, 5]

    # Create the mesh
    return create_mesh(stage, path, points, normals, indices, vertexCounts)


def create_mesh_cube(
    stage: Usd.Stage, 
    path: typing.Union[str, Sdf.Path], 
    halfSize: float
) -> UsdGeom.Mesh:
    """
    Create UsdGeom.Mesh that represents a cube mesh.

    Args:
        stage:      The Usd.Stage to add path.
        path:       The desired path to create.
        halfSize:   The half size of the cube.
    """
    points = [
        Gf.Vec3f(halfSize, -halfSize, -halfSize),
        Gf.Vec3f(halfSize, halfSize, -halfSize),
        Gf.Vec3f(halfSize, halfSize, halfSize),
        Gf.Vec3f(halfSize, -halfSize, halfSize),
        Gf.Vec3f(-halfSize, -halfSize, -halfSize),
        Gf.Vec3f(-halfSize, halfSize, -halfSize),
        Gf.Vec3f(-halfSize, halfSize, halfSize),
        Gf.Vec3f(-halfSize, -halfSize, halfSize),
    ]
    normals = [
        Gf.Vec3f(1, 0, 0),
        Gf.Vec3f(1, 0, 0),
        Gf.Vec3f(1, 0, 0),
        Gf.Vec3f(1, 0, 0),
        Gf.Vec3f(-1, 0, 0),
        Gf.Vec3f(-1, 0, 0),
        Gf.Vec3f(-1, 0, 0),
        Gf.Vec3f(-1, 0, 0),
    ]
    indices = [0, 1, 2, 3, 1, 5, 6, 2, 3, 2, 6, 7, 0, 3, 7, 4, 1, 0, 4, 5, 5, 4, 7, 6]
    vertexCounts = [4, 4, 4, 4, 4, 4]

    # Create the mesh
    return create_mesh(stage, path, points, normals, indices, vertexCounts)

def create_mesh_cylinder(
    stage: Usd.Stage, 
    path: typing.Union[str, Sdf.Path], 
    height: float, 
    radius: float, 
    tesselation: int = 32
) -> UsdGeom.Mesh:
    """
    Create UsdGeom.Mesh that represents a cylinder mesh.

    Args:
        stage:      The Usd.Stage to add path.
        path:       The desired path to create.
        height:     The height of the cylinder.
        radius:     The radius of the cylinder.
        tesselation:     The tesselation of the cylinder mesh.
    """
    points = []
    normals = []
    indices = []
    angle = 0.0

    for i in range(tesselation):
        angle = 2.0 * math.pi * i / tesselation
        angle_cos = math.cos(angle)
        angle_sin = math.sin(angle)
        side_normal = Gf.Vec3f(angle_cos, angle_sin, 0)
        # Top facing upwards.
        points.append(
            Gf.Vec3f(
                angle_cos * radius,
                angle_sin * radius,
                + height / 2
            ))
        normals.append(Gf.Vec3f(0, 0, 1))
        # Top facing sideways.
        points.append(
            Gf.Vec3f(
                angle_cos * radius,
                angle_sin * radius,
                + height / 2
            ))
        normals.append(side_normal)
        # Bottom facing downwards
        points.append(
            Gf.Vec3f(
                angle_cos * radius,
                angle_sin * radius,
                - height / 2
            ))
        normals.append(Gf.Vec3f(0, 0, -1))
        # Bottom facing sideways
        points.append(
            Gf.Vec3f(
                angle_cos * radius,
                angle_sin * radius,
                - height / 2
            ))
        normals.append(side_normal)

        # Top
        indices.append(i * 4) # Edge
        indices.append((tesselation) * 4) # Top center
        # Previous edge
        if i > 0:
            indices.append((i - 1) * 4)
        else:
            indices.append((tesselation - 1) * 4)

        # Upper sideways
        indices.append(i * 4 + 1)
        indices.append(i * 4 + 3) # Edge
        # Previous edge
        if i > 0:
            indices.append((i - 1) * 4 + 1)
        else:
            indices.append((tesselation - 1) * 4 + 1)

        # Bottom
        indices.append((tesselation) * 4 + 1) # Bottom center
        indices.append(i * 4 + 2) # Edge
        # Previous edge
        if i > 0:
            indices.append((i - 1) * 4 + 2)
        else:
            indices.append((tesselation - 1) * 4 + 2)

        # Lower sideways
        indices.append(i * 4 + 3) # Edge
        # Previous edge
        if i > 0:
            indices.append((i - 1) * 4 + 3)
            indices.append((i - 1) * 4 + 1)
        else:
            indices.append((tesselation - 1) * 4 + 3)
            indices.append((tesselation - 1) * 4 + 1)

    # Top center vertex.
    points.append(Gf.Vec3f(0, 0, height / 2))
    normals.append(Gf.Vec3f(0, 0, 1))
    # Bottom center vertex.
    points.append(Gf.Vec3f(0, 0, -height / 2))
    normals.append(Gf.Vec3f(0, 0, -1))

    vertexCounts = [3] * 4 * tesselation

    # Create the mesh
    return create_mesh(stage, path, points, normals, indices, vertexCounts)

def create_mesh_cone(
    stage: Usd.Stage, path: typing.Union[str, Sdf.Path], 
    height: float, 
    radius: float, 
    tesselation: int = 32
) -> UsdGeom.Mesh:
    """
    Create UsdGeom.Mesh that represents a cone mesh.

    Args:
        stage:      The Usd.Stage to add path.
        path:       The desired path to create.
        height:     The height of the cone.
        radius:     The radius of the cone.
        tesselation:     The tesselation of the cone mesh.
    """
    points = []
    normals = []
    indices = []

    intersection_offset = 1 / 3
    normal_z = math.sin(math.atan(radius / height))
    normal_xy = math.sqrt(1.0 - normal_z * normal_z)

    for i in range(tesselation):
        angle = 2.0 * math.pi * i / tesselation
        angle_cos = math.cos(angle)
        angle_sin = math.sin(angle)

        # Tip vertex.
        points.append(Gf.Vec3f(0, 0, height / 2))
        normal = Gf.Vec3f(angle_cos * normal_xy, angle_sin * normal_xy, normal_z)
        normals.append(normal)

        # Intersection point vertex
        points.append(
            Gf.Vec3f(
                angle_cos * radius * intersection_offset,
                angle_sin * radius * intersection_offset,
                height / 2 - height * intersection_offset
            ))

        normals.append(normal)

        # Base vertex sideways
        points.append(
            Gf.Vec3f(
                angle_cos * radius,
                angle_sin * radius,
                - height / 2
            ))

        normals.append(normal)

        # Base vertex downwards.
        points.append(
            Gf.Vec3f(
                angle_cos * radius,
                angle_sin * radius,
                - height / 2
            ))

        normal = Gf.Vec3f(0, 0, -1)
        normals.append(normal)

        # Tip section
        indices.append(i * 4 + 1)
        indices.append(i * 4) # Tip vertex
        # Previous vertex
        if i > 0:
            indices.append((i - 1) * 4 + 1)
        else:
            indices.append((tesselation - 1) * 4 + 1)

        # Upper sideways
        indices.append(i * 4 + 2) # Base vertex
        indices.append(i * 4 + 1) # Intersection vertex
        # Previous vertex
        if i > 0:
            indices.append((i - 1) * 4 + 1)
        else:
            indices.append((tesselation - 1) * 4 + 1)

        # Base sideways
        indices.append(i * 4 + 2) # Base vertex
        # Previous vertex
        if i > 0:
            indices.append((i - 1) * 4 + 1)
            indices.append((i - 1) * 4 + 2)
        else:
            indices.append((tesselation - 1) * 4 + 1)
            indices.append((tesselation - 1) * 4 + 2)

        # Base downwards
        indices.append(tesselation * 4) # Base center
        indices.append(i * 4 + 3) # Base vertex
        # Previous
        if i > 0:
            indices.append((i - 1) * 4 + 3)
        else:
            indices.append((tesselation - 1) * 4 + 3)

    # Base center vertex.
    points.append(Gf.Vec3f(0, 0, -height / 2))
    normals.append(Gf.Vec3f(0, 0, -1))

    vertexCounts = [3] * 4 * tesselation

    # Create the mesh
    return create_mesh(stage, path, points, normals, indices, vertexCounts)

def add_density(
    stage: Usd.Stage, 
    path: typing.Union[str, Sdf.Path], 
    value: float
) -> UsdPhysics.MassAPI:
    """
    Add density to given prim. Note that his will apply MassAPI on the prim.

    Args:
        stage:      The Usd.Stage to add path.
        path:       The desired path to create.
        value:      The desired density.
    """    
    rbPrim = stage.GetPrimAtPath(Sdf.Path(path))
    densityAPI = UsdPhysics.MassAPI.Apply(rbPrim)
    densityAPI.CreateDensityAttr().Set(value)
    return densityAPI

def add_mass(
    stage: Usd.Stage, 
    path: typing.Union["str", Sdf.Path], 
    mass: float = 1.0
) -> UsdPhysics.MassAPI:
    """
    Add mass to given prim. Note that his will apply MassAPI on the prim.

    Args:
        stage:      The Usd.Stage to add path.
        path:       The desired path to create.
        value:      The desired mass.
    """
    rbPrim = stage.GetPrimAtPath(Sdf.Path(path))
    massAPI = UsdPhysics.MassAPI.Apply(rbPrim)
    massAPI.CreateMassAttr().Set(mass)
    return massAPI

def add_force_torque(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    force: Gf.Vec3f = Gf.Vec3f(0.0),
    torque: Gf.Vec3f = Gf.Vec3f(0.0),
    mode: str = "acceleration",
    isEnabled: bool = True,
    isWorldSpace: bool = False
) -> PhysxSchema.PhysxForceAPI:
    """
    Add force/torque to given prim. Note that his will apply PhysxForceAPI on the prim.

    Args:
        stage:      The Usd.Stage to add path.
        path:       The desired path to create.
        force:      The desired force.
        torque:     The desired torque.
        mode:       The force/torque mode.
        isEnabled:  Bool defining whether force is enabled or not.
        isWorldSpace:   Bool defining whether force is applied in world space or body local space.
    """
    rbPrim = stage.GetPrimAtPath(Sdf.Path(path))
    physx_force_api = PhysxSchema.PhysxForceAPI.Apply(rbPrim)
    physx_force_api.CreateForceAttr().Set(force)
    physx_force_api.CreateTorqueAttr().Set(torque)
    physx_force_api.CreateModeAttr().Set(mode)
    physx_force_api.CreateForceEnabledAttr().Set(isEnabled)
    physx_force_api.CreateWorldFrameEnabledAttr().Set(isWorldSpace)
    return physx_force_api

def add_physics_material_to_prim(
    stage: Usd.Stage, 
    prim: Usd.Prim, 
    materialPath: typing.Union["str", Sdf.Path],
):
    """
    Bind physics material to a given prim.

    Args:
        stage:      The Usd.Stage to add path.
        prim:       The Usd.Prim where material should have the binding to.
        materialPath:   The path of the material.
    """    
    bindingAPI = UsdShade.MaterialBindingAPI.Apply(prim)
    materialPrim = UsdShade.Material(stage.GetPrimAtPath(materialPath))
    bindingAPI.Bind(materialPrim, UsdShade.Tokens.weakerThanDescendants, "physics")

def _get_collision_group_includes(stage: Usd.Stage, collisionGroupPath: typing.Union["str", Sdf.Path]):
    collisionGroup = stage.GetPrimAtPath(collisionGroupPath)
    if collisionGroup:
        collisionToken = "colliders"
        collectionAPI = Usd.CollectionAPI.Get(collisionGroup, collisionToken)
        if collectionAPI:
            return collectionAPI.GetIncludesRel()

    return None

def add_collision_to_collision_group(
    stage: Usd.Stage, 
    collisionPath: typing.Union["str", Sdf.Path], 
    collisionGroupPath: typing.Union["str", Sdf.Path]
):
    """
    Add collision path to a collision group include rel.

    Args:
        stage:      The Usd.Stage to add path.
        collisionPath:  Collision path to add.
        collisionGroupPath: Collision group prim path.
    """
    includesRel = _get_collision_group_includes(stage, collisionGroupPath)
    if includesRel:
        includesRel.AddTarget(collisionPath)


def remove_collision_from_collision_group(
    stage: Usd.Stage, 
    collisionPath: typing.Union["str", Sdf.Path], 
    collisionGroupPath: typing.Union["str", Sdf.Path]
):
    """
    Remove collision path to a collision group include rel.

    Args:
        stage:      The Usd.Stage to add path.
        collisionPath:  Collision path to add.
        collisionGroupPath: Collision group prim path.
    """    
    includesRel = _get_collision_group_includes(stage, collisionGroupPath)
    if includesRel:
       includesRel.RemoveTarget(collisionPath)

def is_in_collision_group(
    stage: Usd.Stage, 
    collisionPath: typing.Union["str", Sdf.Path], 
    collisionGroupPath: typing.Union["str", Sdf.Path]
) -> bool:
    """
    Checks if a collision path belongs to a collision group include rel.

    Args:
        stage:      The Usd.Stage to add path.
        collisionPath:  Collision path to add.
        collisionGroupPath: Collision group prim path.
    """    
    includesRel = _get_collision_group_includes(stage, collisionGroupPath)
    if includesRel:
        targets = includesRel.GetTargets()
        for t in targets:
            if (t == collisionPath):
                return True

    return False

def add_ground_plane(
    stage: Usd.Stage, 
    planePath: typing.Union["str", Sdf.Path], 
    axis: str, 
    size: float, 
    position: typing.Union[Gf.Vec3f, Gf.Vec3d], 
    color: Gf.Vec3f
) -> str:
    """
    Add ground plane to the stage. Note that it will add
    a mesh for rendering purpose and UsdPhysics.Plane for collision
    purpose.

    Args:
        stage:      The Usd.Stage to add path.
        planePath:  The desired ground plane path.
        axis:       The up axis - "Y", "Z"
        size:       The half size of the mesh.
        position:   The position where the mesh should be placed in stage.
        color:      The color of the mesh.
    """    
    # plane xform, so that we dont nest geom prims
    planePath = get_stage_next_free_path(stage, planePath, True)
    planeXform = UsdGeom.Xform.Define(stage, planePath)
    planeXform.AddTranslateOp().Set(position)
    planeXform.AddOrientOp().Set(Gf.Quatf(1.0))
    planeXform.AddScaleOp().Set(Gf.Vec3f(1.0))

    # (Graphics) Plane mesh
    geomPlanePath = planePath + "/CollisionMesh"
    entityPlane = create_mesh_square_axis(stage, geomPlanePath, axis, size)
    entityPlane.CreateDisplayColorAttr().Set([color])

    # (Collision) Plane
    colPlanePath = planePath + "/CollisionPlane"
    planeGeom = UsdGeom.Plane.Define(stage, colPlanePath)
    planeGeom.CreatePurposeAttr().Set("guide")
    planeGeom.CreateAxisAttr().Set(axis)

    prim = stage.GetPrimAtPath(colPlanePath)
    UsdPhysics.CollisionAPI.Apply(prim)

    return planePath


def add_quad_plane(
    stage: Usd.Stage, 
    quadPath: typing.Union["str", Sdf.Path], 
    axis: str, 
    size: float, 
    position: typing.Union[Gf.Vec3f, Gf.Vec3d], 
    color: Gf.Vec3f
):
    """
    Add quad mesh to the stage to act as a sized plane.

    Args:
        stage:      The Usd.Stage to add path.
        quadPath:   The desired ground plane path.
        axis:       The up axis - "Y", "Z"
        size:       The half size of the mesh.
        position:   The position where the mesh should be placed in stage.
        color:      The color of the mesh.
    """        
    # Plane quad mesh
    planePath = get_stage_next_free_path(stage, quadPath, True)
    entityPlane = create_mesh_square_axis(stage, planePath, axis, size)
    entityPlane.CreateDisplayColorAttr().Set([color])
    entityPlane.AddTranslateOp().Set(position)
    entityPlane.AddOrientOp().Set(Gf.Quatf(1.0))
    entityPlane.AddScaleOp().Set(Gf.Vec3f(1.0))

    UsdPhysics.CollisionAPI.Apply(entityPlane.GetPrim())


def add_cube_ground_plane(
    stage: Usd.Stage, 
    cubePath: typing.Union["str", Sdf.Path], 
    axis: str, 
    size: float, 
    position: typing.Union[Gf.Vec3f, Gf.Vec3d],
    color: Gf.Vec3f
):
    """
    Add UsdGeom.Cube to the stage to act as a sized plane with thickness.
    The cube is scaled by a vector Gf.Vec3f(0.01, 1.0, 1.0) depending on the up Axis

    Args:
        stage:      The Usd.Stage to add path.
        cubePath:   The desired ground plane path.
        axis:       The up axis - "Y", "Z"
        size:       The half size of the mesh.
        position:   The position where the mesh should be placed in stage.
        color:      The color of the mesh.
    """    
    cubePath = get_stage_next_free_path(stage, cubePath, True)
    cubeGeom = UsdGeom.Cube.Define(stage, cubePath)
    cubeGeom.AddTranslateOp().Set(position)
    cubeGeom.AddOrientOp().Set(Gf.Quatf(1.0))
    cubeGeom.CreateDisplayColorAttr().Set([color])
    cubeGeom.CreateSizeAttr(size)
    half_extent = size / 2
    cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])

    if axis == "X":
        cubeGeom.AddScaleOp().Set(Gf.Vec3f(0.01, 1.0, 1.0))
    elif axis == "Y":
        cubeGeom.AddScaleOp().Set(Gf.Vec3f(1.0, 0.01, 1.0))
    elif axis == "Z":
        cubeGeom.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 0.01))
    UsdPhysics.CollisionAPI.Apply(cubeGeom.GetPrim())

def _add_transformation(
    xform_geom: UsdGeom.Xformable, 
    position: typing.Union[Gf.Vec3f, Gf.Vec3d],
    orientation: typing.Union[Gf.Quatf, Gf.Quatd],
    scale: Gf.Vec3f = Gf.Vec3f(1.0)
):
    xform_geom.AddTranslateOp().Set(position)
    xform_geom.AddOrientOp().Set(orientation)
    xform_geom.AddScaleOp().Set(scale)


def _add_collider(
    xform_prim: Usd.Prim
):
    UsdPhysics.CollisionAPI.Apply(xform_prim)


def _add_rigid(
    xform_prim: Usd.Prim, 
    density: float, 
    lin_velocity: Gf.Vec3f, 
    ang_velocity: Gf.Vec3f
):
    _add_collider(xform_prim)
    if density != 0.0:
        rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(xform_prim)
        rigid_body_api.CreateVelocityAttr().Set(lin_velocity)
        rigid_body_api.CreateAngularVelocityAttr().Set(ang_velocity)
        mass_api = UsdPhysics.MassAPI.Apply(xform_prim)
        mass_api.CreateDensityAttr(density)


def add_box(
    stage: Usd.Stage, 
    path: typing.Union["str", Sdf.Path], 
    size: Gf.Vec3f = Gf.Vec3f(1.0), 
    position: Gf.Vec3f = Gf.Vec3f(0.0), 
    orientation: Gf.Quatf = Gf.Quatf(1.0), 
    color: Gf.Vec3f = Gf.Vec3f(1.0)
) -> Usd.Prim:
    """
    Add UsdGeom.Cube to the stage.

    Args:
        stage:      The Usd.Stage to add cube.
        path:       The desired cube path.
        size:       The size of the cube.
        position:   The position where the cube should be placed in stage.
        orientation:   The cube orientation.
        color:      The color of the mesh.
    """
    path = get_stage_next_free_path(stage, path, True)
    cube_geom = UsdGeom.Cube.Define(stage, path)
    cube_size = 1.0
    half_extent = cube_size / 2
    cube_geom.CreateSizeAttr(cube_size)
    cube_geom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
    cube_geom.CreateDisplayColorAttr().Set([color])

    # make it possible to define size as float/int
    if not isinstance(size, Gf.Vec3f):
        size = Gf.Vec3f(size)

    _add_transformation(cube_geom, position, orientation, size)
    return stage.GetPrimAtPath(path)


def add_collider_box(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    size: Gf.Vec3f = Gf.Vec3f(1.0),
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    orientation: Gf.Quatf = Gf.Quatf(1.0),
    color: Gf.Vec3f = Gf.Vec3f(1.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Cube to the stage and add physics collider API to it.

    Args:
        stage:      The Usd.Stage to add cube.
        path:       The desired cube path.
        size:       The size of the cube.
        position:   The position where the cube should be placed in stage.
        orientation:   The cube orientation.
        color:      The color of the mesh.
    """    
    cube_prim = add_box(stage, path, size, position, orientation, color)
    _add_collider(cube_prim)
    return cube_prim


def add_rigid_box(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    size: Gf.Vec3f = Gf.Vec3f(1.0),
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    orientation: Gf.Quatf = Gf.Quatf(1.0),
    color: Gf.Vec3f = Gf.Vec3f(1.0),
    density: float =1.0,
    lin_velocity: Gf.Vec3f = Gf.Vec3f(0.0),
    ang_velocity: Gf.Vec3f = Gf.Vec3f(0.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Cube to the stage and add physics rigid body and collider API to it.

    Args:
        stage:      The Usd.Stage to add cube.
        path:       The desired cube path.
        size:       The size of the cube.
        position:   The position where the cube should be placed in stage.
        orientation:   The cube orientation.
        color:      The color of the mesh.
        lin_velocity: The initial linear velocity of the rigid body.
        ang_velocity: The initial angular velocity of the rigid body.
    """    
    cube_prim = add_box(stage, path, size, position, orientation, color)
    _add_rigid(cube_prim, density, lin_velocity, ang_velocity)
    return cube_prim


def add_cube(
    stage: Usd.Stage, 
    path: typing.Union["str", Sdf.Path],
    size: Gf.Vec3f=Gf.Vec3f(1.0), 
    position: Gf.Vec3f=Gf.Vec3f(0.0), 
    orientation: Gf.Quatf=Gf.Quatf(1.0), 
    color: Gf.Vec3f=Gf.Vec3f(1.0)
) -> Usd.Prim:
    """
    Add UsdGeom.Cube to the stage.

    Args:
        stage:      The Usd.Stage to add cube.
        path:       The desired cube path.
        size:       The size of the cube.
        position:   The position where the cube should be placed in stage.
        orientation:   The cube orientation.
        color:      The color of the mesh.
    """    
    return add_box(stage, path, size, position, orientation, color)


def add_collider_cube(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    size: Gf.Vec3f=Gf.Vec3f(1.0),
    position: Gf.Vec3f=Gf.Vec3f(0.0),
    orientation: Gf.Quatf=Gf.Quatf(1.0),
    color: Gf.Vec3f=Gf.Vec3f(1.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Cube to the stage and add physics collider API to it.

    Args:
        stage:      The Usd.Stage to add cube.
        path:       The desired cube path.
        size:       The size of the cube.
        position:   The position where the cube should be placed in stage.
        orientation:   The cube orientation.
        color:      The color of the mesh.
    """    
    return add_collider_box(stage, path, size, position, orientation, color)


def add_rigid_cube(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    size: Gf.Vec3f=Gf.Vec3f(1.0),
    position: Gf.Vec3f=Gf.Vec3f(0.0),
    orientation: Gf.Quatf=Gf.Quatf(1.0),
    color: Gf.Vec3f=Gf.Vec3f(1.0),
    density: float=1.0,
    lin_velocity: Gf.Vec3f=Gf.Vec3f(0.0),
    ang_velocity: Gf.Vec3f=Gf.Vec3f(0.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Cube to the stage and add physics rigid body and collider API to it.

    Args:
        stage:      The Usd.Stage to add cube.
        path:       The desired cube path.
        size:       The size of the cube.
        position:   The position where the cube should be placed in stage.
        orientation:   The cube orientation.
        color:      The color of the mesh.
        lin_velocity: The initial linear velocity of the rigid body.
        ang_velocity: The initial angular velocity of the rigid body.
    """    
    return add_rigid_box(stage, path, size, position, orientation, color, density, lin_velocity, ang_velocity)


def add_sphere(
    stage: Usd.Stage, 
    path: typing.Union["str", Sdf.Path], 
    radius: float=1.0, 
    position: Gf.Vec3f=Gf.Vec3f(0.0), 
    orientation: Gf.Quatf=Gf.Quatf(1.0), 
    color: Gf.Vec3f=Gf.Vec3f(1.0)
) -> Usd.Prim:
    """
    Add UsdGeom.Sphere to the stage.

    Args:
        stage:      The Usd.Stage to add sphere.
        path:       The desired sphere path.
        radius:     The radius of the sphere.
        position:   The position where the sphere should be placed in stage.
        orientation:   The sphere orientation.
        color:      The color of the mesh.
    """        
    path = get_stage_next_free_path(stage, path, True)
    sphere_geom = UsdGeom.Sphere.Define(stage, path)
    sphere_geom.CreateRadiusAttr(radius)
    sphere_geom.CreateExtentAttr([(-radius, -radius, -radius), (radius, radius, radius)])
    sphere_geom.CreateDisplayColorAttr().Set([color])
    _add_transformation(sphere_geom, position, orientation)
    return stage.GetPrimAtPath(path)


def add_collider_sphere(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    radius: float=1.0,
    position: Gf.Vec3f=Gf.Vec3f(0.0),
    orientation: Gf.Quatf=Gf.Quatf(1.0),
    color: Gf.Vec3f=Gf.Vec3f(1.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Sphere to the stage and add physics collider API to it.

    Args:
        stage:      The Usd.Stage to add sphere.
        path:       The desired sphere path.
        radius:     The radius of the sphere.
        position:   The position where the sphere should be placed in stage.
        orientation:   The sphere orientation.
        color:      The color of the mesh.
    """        
    sphere_prim = add_sphere(stage, path, radius, position, orientation, color)
    _add_collider(sphere_prim)
    return sphere_prim


def add_rigid_sphere(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    radius: float=1.0,
    position: Gf.Vec3f=Gf.Vec3f(0.0),
    orientation: Gf.Quatf=Gf.Quatf(1.0),
    color: Gf.Vec3f=Gf.Vec3f(1.0),
    density: float=1.0,
    lin_velocity: Gf.Vec3f=Gf.Vec3f(0.0),
    ang_velocity: Gf.Vec3f=Gf.Vec3f(0.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Sphere to the stage and add physics rigid body and collider API to it.

    Args:
        stage:      The Usd.Stage to add sphere.
        path:       The desired sphere path.
        radius:     The radius of the sphere.
        position:   The position where the sphere should be placed in stage.
        orientation:   The sphere orientation.
        color:      The color of the mesh.
        lin_velocity: The initial linear velocity of the rigid body.
        ang_velocity: The initial angular velocity of the rigid body.
    """        
    sphere_prim = add_sphere(stage, path, radius, position, orientation, color)
    UsdPhysics.CollisionAPI.Apply(sphere_prim)
    _add_rigid(sphere_prim, density, lin_velocity, ang_velocity)
    return sphere_prim

def add_capsule(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    radius: float=1.0,
    height: float=1.0,
    axis: str="Y",
    position: Gf.Vec3f=Gf.Vec3f(0.0),
    orientation: Gf.Quatf=Gf.Quatf(1.0),
    color: Gf.Vec3f=Gf.Vec3f(1.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Capsule to the stage.

    Args:
        stage:      The Usd.Stage to add capsule.
        path:       The desired capsule path.
        radius:     The radius of the capsule.
        height:     The height of the capsule.
        axis:       The axis of the capsule.
        position:   The position where the capsule should be placed in stage.
        orientation:   The capsule orientation.
        color:      The color of the mesh.
    """            
    path = get_stage_next_free_path(stage, path, True)
    capsule_geom = UsdGeom.Capsule.Define(stage, path)
    capsule_geom.CreateRadiusAttr(radius)
    capsule_geom.CreateHeightAttr(height)
    capsule_geom.CreateExtentAttr([(-radius, -radius, -height), (radius, radius, height)])
    capsule_geom.CreateAxisAttr(axis)
    capsule_geom.CreateDisplayColorAttr().Set([color])
    _add_transformation(capsule_geom, position, orientation)
    return stage.GetPrimAtPath(path)


def add_collider_capsule(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    radius: float=1.0,
    height: float=1.0,
    axis: str="Y",
    position: Gf.Vec3f=Gf.Vec3f(0.0),
    orientation: Gf.Quatf=Gf.Quatf(1.0),
    color: Gf.Vec3f=Gf.Vec3f(1.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Capsule to the stage and add physics collider API to it.

    Args:
        stage:      The Usd.Stage to add capsule.
        path:       The desired capsule path.
        radius:     The radius of the capsule.
        height:     The height of the capsule.
        axis:       The axis of the capsule.
        position:   The position where the capsule should be placed in stage.
        orientation:   The capsule orientation.
        color:      The color of the mesh.
    """     
    capsule_prim = add_capsule(stage, path, radius, height, axis, position, orientation, color)
    _add_collider(capsule_prim)
    return capsule_prim


def add_rigid_capsule(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    radius: float=1.0,
    height: float=1.0,
    axis: str="Y",
    position: Gf.Vec3f=Gf.Vec3f(0.0),
    orientation: Gf.Quatf=Gf.Quatf(1.0),
    color: Gf.Vec3f=Gf.Vec3f(1.0),
    density: float=1.0,
    lin_velocity: Gf.Vec3f=Gf.Vec3f(0.0),
    ang_velocity: Gf.Vec3f=Gf.Vec3f(0.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Capsule to the stage and add physics rigid body and collider API to it.

    Args:
        stage:      The Usd.Stage to add capsule.
        path:       The desired capsule path.
        radius:     The radius of the capsule.
        height:     The height of the capsule.
        axis:       The axis of the capsule.
        position:   The position where the capsule should be placed in stage.
        orientation:   The capsule orientation.
        color:      The color of the mesh.
        lin_velocity: The initial linear velocity of the rigid body.
        ang_velocity: The initial angular velocity of the rigid body.
    """     
    capsule_prim = add_capsule(stage, path, radius, height, axis, position, orientation, color)
    _add_rigid(capsule_prim, density, lin_velocity, ang_velocity)
    return capsule_prim

def add_cylinder(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    radius: float=1.0,
    height: float=1.0,
    axis: str="Y",
    position: Gf.Vec3f=Gf.Vec3f(0.0),
    orientation: Gf.Quatf=Gf.Quatf(1.0),
    color: Gf.Vec3f=Gf.Vec3f(1.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Cylinder to the stage.

    Args:
        stage:      The Usd.Stage to add cylinder.
        path:       The desired cylinder path.
        radius:     The radius of the cylinder.
        height:     The height of the cylinder.
        axis:       The axis of the cylinder.
        position:   The position where the cylinder should be placed in stage.
        orientation:   The cylinder orientation.
        color:      The color of the mesh.
    """    
    path = get_stage_next_free_path(stage, path, True)
    cylinder_geom = UsdGeom.Cylinder.Define(stage, path)
    cylinder_geom.CreateRadiusAttr(radius)
    cylinder_geom.CreateHeightAttr(height)
    cylinder_geom.CreateExtentAttr([(-radius, -radius, -height), (radius, radius, height)])
    cylinder_geom.CreateAxisAttr(axis)
    cylinder_geom.CreateDisplayColorAttr().Set([color])
    _add_transformation(cylinder_geom, position, orientation)
    return stage.GetPrimAtPath(path)


def add_collider_cylinder(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    radius: float=1.0,
    height: float=1.0,
    axis: str="Y",
    position: Gf.Vec3f=Gf.Vec3f(0.0),
    orientation: Gf.Quatf=Gf.Quatf(1.0),
    color: Gf.Vec3f=Gf.Vec3f(1.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Cylinder to the stage and add physics collider API to it.

    Args:
        stage:      The Usd.Stage to add cylinder.
        path:       The desired cylinder path.
        radius:     The radius of the cylinder.
        height:     The height of the cylinder.
        axis:       The axis of the cylinder.
        position:   The position where the cylinder should be placed in stage.
        orientation:   The cylinder orientation.
        color:      The color of the mesh.
    """    
    cylinder_prim = add_cylinder(stage, path, radius, height, axis, position, orientation, color)
    _add_collider(cylinder_prim)
    return cylinder_prim


def add_rigid_cylinder(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    radius: float=1.0,
    height: float=1.0,
    axis: str="Y",
    position: Gf.Vec3f=Gf.Vec3f(0.0),
    orientation: Gf.Quatf=Gf.Quatf(1.0),
    color: Gf.Vec3f=Gf.Vec3f(1.0),
    density: float=1.0,
    lin_velocity: Gf.Vec3f=Gf.Vec3f(0.0),
    ang_velocity: Gf.Vec3f=Gf.Vec3f(0.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Cylinder to the stage and add physics rigid body and collider API to it.

    Args:
        stage:      The Usd.Stage to add cylinder.
        path:       The desired cylinder path.
        radius:     The radius of the cylinder.
        height:     The height of the cylinder.
        axis:       The axis of the cylinder.
        position:   The position where the cylinder should be placed in stage.
        orientation:   The cylinder orientation.
        color:      The color of the mesh.
        lin_velocity: The initial linear velocity of the rigid body.
        ang_velocity: The initial angular velocity of the rigid body.
    """    
    cylinder_prim = add_cylinder(stage, path, radius, height, axis, position, orientation, color)
    _add_rigid(cylinder_prim, density, lin_velocity, ang_velocity)
    return cylinder_prim


def add_cone(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    radius: float=1.0,
    height: float=1.0,
    axis: str="Y",
    position: Gf.Vec3f=Gf.Vec3f(0.0),
    orientation: Gf.Quatf=Gf.Quatf(1.0),
    color: Gf.Vec3f=Gf.Vec3f(1.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Cone to the stage.

    Args:
        stage:      The Usd.Stage to add cone.
        path:       The desired cone path.
        radius:     The radius of the cone.
        height:     The height of the cone.
        axis:       The axis of the cone.
        position:   The position where the cone should be placed in stage.
        orientation:   The cone orientation.
        color:      The color of the mesh.
    """    
    path = get_stage_next_free_path(stage, path, True)
    cone_geom = UsdGeom.Cone.Define(stage, path)
    cone_geom.CreateRadiusAttr(radius)
    cone_geom.CreateHeightAttr(height)
    cone_geom.CreateExtentAttr([(-radius, -radius, -height), (radius, radius, height)])
    cone_geom.CreateAxisAttr(axis)
    cone_geom.CreateDisplayColorAttr().Set([color])
    _add_transformation(cone_geom, position, orientation)
    return stage.GetPrimAtPath(path)


def add_collider_cone(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    radius: float=1.0,
    height: float=1.0,
    axis: str="Y",
    position: Gf.Vec3f=Gf.Vec3f(0.0),
    orientation: Gf.Quatf=Gf.Quatf(1.0),
    color: Gf.Vec3f=Gf.Vec3f(1.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Cone to the stage and add physics collider API to it.

    Args:
        stage:      The Usd.Stage to add cone.
        path:       The desired cone path.
        radius:     The radius of the cone.
        height:     The height of the cone.
        axis:       The axis of the cone.
        position:   The position where the cone should be placed in stage.
        orientation:   The cone orientation.
        color:      The color of the mesh.
    """     
    cone_prim = add_cone(stage, path, radius, height, axis, position, orientation, color)
    _add_collider(cone_prim)
    return cone_prim


def add_rigid_cone(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    radius: float=1.0,
    height: float=1.0,
    axis: str="Y",
    position: Gf.Vec3f=Gf.Vec3f(0.0),
    orientation: Gf.Quatf=Gf.Quatf(1.0),
    color: Gf.Vec3f=Gf.Vec3f(1.0),
    density: float=1.0,
    lin_velocity: Gf.Vec3f=Gf.Vec3f(0.0),
    ang_velocity: Gf.Vec3f=Gf.Vec3f(0.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Cone to the stage and add physics rigid body and collider API to it.

    Args:
        stage:      The Usd.Stage to add cone.
        path:       The desired cone path.
        radius:     The radius of the cone.
        height:     The height of the cone.
        axis:       The axis of the cone.
        position:   The position where the cone should be placed in stage.
        orientation:   The cone orientation.
        color:      The color of the mesh.
        lin_velocity: The initial linear velocity of the rigid body.
        ang_velocity: The initial angular velocity of the rigid body.
    """     
    cone_prim = add_cone(stage, path, radius, height, axis, position, orientation, color)
    _add_rigid(cone_prim, density, lin_velocity, ang_velocity)
    return cone_prim

def add_xform(
    stage: Usd.Stage, 
    path: typing.Union["str", Sdf.Path], 
    position: Gf.Vec3f=Gf.Vec3f(0.0), 
    orientation: Gf.Quatf=Gf.Quatf(1.0), 
    scale: Gf.Vec3f=Gf.Vec3f(1.0)
) -> Usd.Prim:
    """
    Add xform to the stage with given transformation.

    Args:
        stage:      The Usd.Stage to add cone.
        path:       The desired xform path.
        position:   The position where the xform should be placed in stage.
        orientation:   The xform orientation.
        scale:      The xform scale.
    """    
    path = get_stage_next_free_path(stage, path, True)
    xform_geom = UsdGeom.Xform.Define(stage, path)
    _add_transformation(xform_geom, position, orientation, scale)
    return stage.GetPrimAtPath(path)


def add_rigid_xform(
    stage: Usd.Stage, 
    path: typing.Union["str", Sdf.Path], 
    position: Gf.Vec3f=Gf.Vec3f(0.0), 
    orientation: Gf.Quatf=Gf.Quatf(1.0), 
    scale: Gf.Vec3f=Gf.Vec3f(1.0)
) -> Usd.Prim:
    """
    Add xform to the stage with given transformation and add rigid body API to it.

    Args:
        stage:      The Usd.Stage to add cone.
        path:       The desired xform path.
        position:   The position where the xform should be placed in stage.
        orientation:   The xform orientation.
        scale:      The xform scale.
    """    
    xform_prim = add_xform(stage, path, position, orientation, scale)
    physicsAPI = UsdPhysics.RigidBodyAPI.Apply(xform_prim)
    physicsAPI.CreateRigidBodyEnabledAttr(True)
    return xform_prim

def get_translation(
    prim: Usd.Prim
) -> typing.Union[Gf.Vec3f, Gf.Vec3d]:
    """
    Return translate attribute value from the given prim.

    Args:
        prim:      The Usd.Prim to check.
    """    
    return prim.GetAttribute("xformOp:translate").Get()

def add_joint_fixed(
    stage: Usd.Stage, 
    jointPath: typing.Union["str", Sdf.Path], 
    actor0: typing.Union["str", Sdf.Path], 
    actor1: typing.Union["str", Sdf.Path], 
    localPos0: Gf.Vec3f, 
    localRot0: Gf.Quatf, 
    localPos1: Gf.Vec3f, 
    localRot1: Gf.Quatf, 
    breakForce: float, 
    breakTorque: float
) -> UsdPhysics.FixedJoint:
    """
    Add fixed joint to the stage.

    Args:
        stage:      The Usd.Stage to add the joint.
        jointPath:  The desired joint path.
        actor0:     The actor0 for the joint.
        actor1:     The actor1 for the joint.
        localPos0:  The joint local position offset from the actor0
        localRot0:  The joint local rotation offset from the actor0
        localPos1:  The joint local position offset from the actor1
        localRot1:  The joint local rotation offset from the actor1
        breakForce: The joint break force.
        breakTorque: The joint break torque.
    """    
    # D6 fixed joint
    jointPath = get_stage_next_free_path(stage, jointPath, True)
    d6FixedJoint = UsdPhysics.FixedJoint.Define(stage, jointPath)

    if actor0:
        d6FixedJoint.CreateBody0Rel().SetTargets([actor0])
    if actor1:
        d6FixedJoint.CreateBody1Rel().SetTargets([actor1])

    d6FixedJoint.CreateLocalPos0Attr().Set(localPos0)
    d6FixedJoint.CreateLocalRot0Attr().Set(localRot0)

    d6FixedJoint.CreateLocalPos1Attr().Set(localPos1)
    d6FixedJoint.CreateLocalRot1Attr().Set(localRot1)

    d6FixedJoint.CreateBreakForceAttr().Set(breakForce)
    d6FixedJoint.CreateBreakTorqueAttr().Set(breakTorque)
    return d6FixedJoint

def set_or_add_scale_op(
    xformable: UsdGeom.Xformable, scale: typing.Union[Gf.Vec3f, Gf.Vec3d, Gf.Vec3h]
) -> UsdGeom.XformOp:
    """
    Sets or adds the scale XformOp on the input Xformable to provided scale value.

    Note that:
        - The precision of an added attribute is UsdGeom.XformOp.PrecisionFloat.

    Args:
        xformable:  The Xformable to modify.
        scale:      The scale vector
    Returns:
        The set or added XformOp
    """
    prim = xformable.GetPrim()
    if not (prim.IsA(UsdGeom.Xformable)):
        carb.log_warn(f"{__name__}.set_or_add_scale_op: Provided prim is not an Xformable")
        return False
    xformOp = _get_or_create_xform_op(xformable, "xformOp:scale", UsdGeom.XformOp.TypeScale)
    if xformOp.Get() is None:
        xformOp.Set(Gf.Vec3f(scale))
    else:
        typeName = type(xformOp.Get())
        xformOp.Set(typeName(scale))
    return xformOp


def set_or_add_translate_op(
    xformable: UsdGeom.Xformable, translate: typing.Union[Gf.Vec3f, Gf.Vec3d, Gf.Vec3h]
) -> UsdGeom.XformOp:
    """
    Sets or adds the translate XformOp on the input Xformable to provided translate value.

    Note that:
        - The precision of an added attribute is UsdGeom.XformOp.PrecisionFloat.

    Args:
        xformable:  The Xformable to modify.
        translate:      The translate vector
    Returns:
        The set or added XformOp
    """
    prim = xformable.GetPrim()
    if not (prim.IsA(UsdGeom.Xformable)):
        carb.log_warn(f"{__name__}.set_or_add_translate_op: Provided prim is not an Xformable")
        return False
    xformOp = _get_or_create_xform_op(xformable, "xformOp:translate", UsdGeom.XformOp.TypeTranslate)
    if xformOp.Get() is None:
        xformOp.Set(Gf.Vec3f(translate))
    else:
        typeName = type(xformOp.Get())
        xformOp.Set(typeName(translate))
    return xformOp


def set_or_add_orient_op(
    xformable: UsdGeom.Xformable, orient: typing.Union[Gf.Quatf, Gf.Quatd, Gf.Quath]
) -> UsdGeom.XformOp:
    """
    Sets or adds the orient XformOp on the input Xformable to provided orient value.

    Note that:
        - The precision of an added attribute is UsdGeom.XformOp.PrecisionFloat.

    Args:
        xformable:  The Xformable to modify.
        orient:      The orient quaternion
    Returns:
        The set or added XformOp
    """
    prim = xformable.GetPrim()
    if not (prim.IsA(UsdGeom.Xformable)):
        carb.log_warn(f"{__name__}.set_or_add_orient_op: Provided prim is not an Xformable")
        return None
    xformOp = _get_or_create_xform_op(xformable, "xformOp:orient", UsdGeom.XformOp.TypeOrient)
    if xformOp.Get() is None:
        xformOp.Set(Gf.Quatf(orient))
    else:
        typeName = type(xformOp.Get())
        xformOp.Set(typeName(orient))
    return xformOp


def set_or_add_scale_orient_translate(
    xformable: UsdGeom.Xformable,
    scale: typing.Union[Gf.Vec3f, Gf.Vec3d, Gf.Vec3h],
    orient: typing.Union[Gf.Quatf, Gf.Quatd, Gf.Quath],
    translate: typing.Union[Gf.Vec3f, Gf.Vec3d, Gf.Vec3h],
) -> typing.List[UsdGeom.XformOp]:
    """
    Sets or adds scale, orient, and translate XformOps of xformable.

    Note that:
        - The precision of created attributes is UsdGeom.XformOp.PrecisionFloat.

    Args:
        xformable:  The Xformable to modify.
        scale:      The scale vector
        orient:     The orientation quaternion
        translate:  The translation vector
    Returns:
        List of set and created xform ops that will be [translate, orient, scale]
    """
    prim = xformable.GetPrim()
    if not (prim.IsA(UsdGeom.Xformable)):
        carb.log_warn(f"{__name__}.set_or_add_scale_orient_translate: Provided prim is not an Xformable")
        return False
    tosOps = []
    tosOps.append(set_or_add_translate_op(xformable, translate))
    tosOps.append(set_or_add_orient_op(xformable, orient))
    tosOps.append(set_or_add_scale_op(xformable, scale))
    return tosOps


def setup_transform_as_scale_orient_translate(xformable: UsdGeom.Xformable):
    """
    Changes the local transform (ops) to the physics default scale->orient->translate stack.

    Note that:
        - Any skew in the transform will be lost.
        - A resetXformStack is preserved, but not the XformOps that are ignored due to the reset.
        - The transform attribute precision is set to UsdGeom.XformOp.PrecisionFloat.
        - Obsolete xformOp: namespace attributes are not removed (and cannot be for layers)

    Args:
        xformable: The Xformable to modify.
    """
    prim = xformable.GetPrim()
    if not (prim.IsA(UsdGeom.Xformable)):
        carb.log_warn(f"{__name__}.setup_transform_as_scale_orient_translate: Provided prim is not an Xformable")
        return
    # make sure we are working with an Xformable (e.g. if just a prim was passed in)
    xformable = UsdGeom.Xformable(xformable)
    hasReset = xformable.GetResetXformStack()
    with Sdf.ChangeBlock():  # batch the transform changes:
        tf = Gf.Transform(xformable.GetLocalTransformation())
        scale = Gf.Vec3d(tf.GetScale())
        translation = Gf.Vec3d(tf.GetTranslation())
        quat = Gf.Quatd(tf.GetRotation().GetQuat())
        newOps = set_or_add_scale_orient_translate(xformable, scale, quat, translation)
        xformable.SetXformOpOrder(newOps, hasReset)


def copy_transform_as_scale_orient_translate(src: UsdGeom.Xformable, dst: UsdGeom.Xformable):
    """
    Copies the local transforms from one Xformable to another as a default scale->orient->translate stack.

    Note that:
        - Any skew in the src transform will be lost.
        - A resetXformStack is preserved, but not the XformOps that are ignored due to the reset.
        - The transform attribute precision of added XformOps is set to UsdGeom.XformOp.PrecisionFloat.
        - Obsolete xformOp: namespace attributes in dst are not removed (and cannot be for layers)

    Args:
        src: The source Xformable.
        dst: The destination Xformable.
    """
    srcPrim = src.GetPrim()
    dstPrim = dst.GetPrim()
    if not (srcPrim.IsA(UsdGeom.Xformable) and dstPrim.IsA(UsdGeom.Xformable)):
        carb.log_warn(
            f"{__name__}.copy_transform_as_scale_orient_translate: Either the src or dst Xformable parameter is not an Xformable"
        )
        return
    srcXformable = UsdGeom.Xformable(src)
    dstXformable = UsdGeom.Xformable(dst)
    hasReset = srcXformable.GetResetXformStack()
    with Sdf.ChangeBlock():  # batch the transform changes:
        tf = Gf.Transform(srcXformable.GetLocalTransformation())
        scale = Gf.Vec3d(tf.GetScale())
        translation = Gf.Vec3d(tf.GetTranslation())
        quat = Gf.Quatd(tf.GetRotation().GetQuat())
        newOps = set_or_add_scale_orient_translate(dstXformable, scale, quat, translation)
        dstXformable.SetXformOpOrder(newOps, hasReset)


def get_initial_collider_pairs(stage: Usd.Stage) -> typing.Set[typing.Tuple[str, str]]:
    """
    Get all collider pairs that are in contact in the physics simulation.

    This function performs a single physics simulation step and collects all collider pairs
    that are in contact. It temporarily modifies physics settings to ensure accurate contact
    detection and restores them after completion.

    The function:
    1. Creates a temporary session layer for contact reporting
    2. Enables contact reporting for all rigid bodies
    3. Runs a single physics simulation step
    4. Collects all collider pairs that are in contact
    5. Restores original physics settings

    Args:
        stage (Usd.Stage): The USD stage containing the physics scene to analyze.

    Returns:
        typing.Set[typing.Tuple[str, str]]: A set of tuples, where each tuple contains
            the paths of two colliders that are in contact. The paths in each tuple are
            sorted alphabetically to ensure consistent ordering regardless of which collider
            initiated the contact.

    Note:
        This function temporarily modifies physics settings and runs a simulation step.
        The original settings are restored after the function completes.
    """
    unique_collider_pairs = set()  # Use a set to store unique collider pairs
    session_sub_layer = Sdf.Layer.CreateAnonymous()
    stage.GetSessionLayer().subLayerPaths.append(session_sub_layer.identifier)
    old_layer = stage.GetEditTarget().GetLayer()
    stage.SetEditTarget(Usd.EditTarget(session_sub_layer))

    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    usdrtStage = usdrt.Usd.Stage.Attach(stage_id)
    prim_paths = usdrtStage.GetPrimsWithAppliedAPIName("PhysicsRigidBodyAPI")

    for prim_path in prim_paths:
        prim = stage.GetPrimAtPath(str(prim_path))
        if prim:
            contact_report_api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
            contact_report_api.CreateThresholdAttr().Set(0)

    settings = carb.settings.get_settings()
    write_usd = settings.get_as_bool(SETTING_UPDATE_TO_USD)
    write_fabric = settings.get_as_bool("/physics/fabricEnabled")
    
    settings.set(SETTING_UPDATE_TO_USD, False)
    settings.set("/physics/fabricEnabled", False)

    initial_attach = False
    if get_physx_simulation_interface().get_attached_stage() != stage_id:
        get_physx_simulation_interface().attach_stage(stage_id)
        initial_attach = True

    get_physx_simulation_interface().simulate(1.0/60.0, 0.0)
    get_physx_simulation_interface().fetch_results()

    contact_headers, contact_data = get_physx_simulation_interface().get_contact_report()        
    if len(contact_headers) > 0:
        for contact_header in contact_headers:
            if contact_header.type == ContactEventType.CONTACT_FOUND:
                collider0 = str(PhysicsSchemaTools.intToSdfPath(contact_header.collider0))
                collider1 = str(PhysicsSchemaTools.intToSdfPath(contact_header.collider1))
                # Store as a tuple, ensuring consistent ordering
                pair = tuple(sorted([collider0, collider1]))
                unique_collider_pairs.add(pair)

    if initial_attach:
        get_physx_simulation_interface().detach_stage()

    settings.set(SETTING_UPDATE_TO_USD, write_usd)
    settings.set("/physics/fabricEnabled", write_fabric)

    stage.SetEditTarget(old_layer)

    stage.GetSessionLayer().subLayerPaths.remove(session_sub_layer.identifier)
    session_sub_layer = None

    return unique_collider_pairs


def _get_or_create_xform_op(
    xformable: UsdGeom.Xformable, opName: str, opType: str, opPrecisionIfCreate=UsdGeom.XformOp.PrecisionFloat
) -> UsdGeom.XformOp:
    """
    Gets or creates an XformOp of an Xformable.

    Note that:
        - Any skew in the transform will be lost.
        - A resetXformStack is preserved, but not the XformOps that are ignored due to the reset.
        - The transform attribute precision is set to UsdGeom.XformOp.PrecisionFloat.

    Args:
        xformable:  The Xformable to modify.
        opName:     The XformOp attribute name, e.g. "xformOp:translate"
        opType:     The XformOp type, e.g. UsdGeom.XformOp.TypeScale
    """
    dstOp = UsdGeom.XformOp(xformable.GetPrim().GetAttribute(opName))
    if not dstOp:
        # create op
        dstOp = xformable.AddXformOp(opType, opPrecisionIfCreate)
    return dstOp
