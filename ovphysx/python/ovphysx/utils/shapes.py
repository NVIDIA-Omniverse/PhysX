# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-UTILS-001
# @covers AC-1 AC-2 AC-5 AC-12

"""Shape constructors: geometry, colliders and rigid bodies in one call.

Every shape comes in three flavors. The plain ``add_*`` helper defines the
``UsdGeom`` prim with a display color and a translate-orient-scale xform op
stack; ``add_collider_*`` additionally applies ``UsdPhysics.CollisionAPI``; and
``add_rigid_*`` applies ``UsdPhysics.RigidBodyAPI`` and ``UsdPhysics.MassAPI``
with a density and initial velocities on top of that. Each is a one-line way to
put a falling box or a static ramp on a stage.

The physics APIs are applied here directly rather than through
:func:`~ovphysx.utils.authoring.set_collider`, so a shape carries a bare
collision API and no mesh approximation: these are all analytic ``UsdGeom``
shapes, for which an approximation has nothing to describe. Reach for
``set_collider`` instead when the prim is a mesh or already exists.
"""

import typing

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from .paths import get_stage_next_free_path

__all__ = [
    "add_box",
    "add_collider_box",
    "add_rigid_box",
    "add_cube",
    "add_collider_cube",
    "add_rigid_cube",
    "add_sphere",
    "add_collider_sphere",
    "add_rigid_sphere",
    "add_capsule",
    "add_collider_capsule",
    "add_rigid_capsule",
    "add_cylinder",
    "add_collider_cylinder",
    "add_rigid_cylinder",
    "add_cone",
    "add_collider_cone",
    "add_rigid_cone",
    "add_xform",
    "add_rigid_xform",
]


def _add_transformation(
    xform_geom: UsdGeom.Xformable,
    position: typing.Union[Gf.Vec3f, Gf.Vec3d],
    orientation: typing.Union[Gf.Quatf, Gf.Quatd],
    scale: Gf.Vec3f = Gf.Vec3f(1.0),
):
    xform_geom.AddTranslateOp().Set(position)
    xform_geom.AddOrientOp().Set(orientation)
    xform_geom.AddScaleOp().Set(scale)


def _author_computed_extent(gprim: UsdGeom.Boundable):
    """Author the extent the gprim's own schema computes for it.

    An ``extent`` opinion is what every consumer reads before it looks at the
    geometry, so a wrong one mis-frames a camera, mis-sizes a selection
    highlight and drops the shape out of a cull or a picking test. The extent
    has to follow ``axis``, use half of ``height`` either side of the centre for
    a cylinder and a cone, and add ``radius`` for a capsule's two hemispherical
    caps.

    ``UsdGeom.Boundable.ComputeExtentFromPlugins`` is the computation
    ``UsdGeomCapsule``, ``UsdGeomCylinder`` and ``UsdGeomCone`` each register
    for their own geometry, and is the one USD itself falls back on when no
    extent is authored. Authoring its result therefore cannot disagree with a
    bound computed later, which hand-rolled arithmetic here could drift from.
    The per-schema static ``ComputeExtent(radius, height, axis, &extent)``
    overloads are C++-only -- Python sees just the inherited ``Boundable``
    instance method -- and that one prefers an already-authored ``extent`` over
    recomputing, so ``ComputeExtentFromPlugins`` is the call that asks the
    schema rather than the prim.

    It answers ``None`` for an ``axis`` token USD does not recognise, since
    there is then no axis to compute along. No extent is authored in that case,
    which leaves the prim in the state it would be in with the attribute never
    written -- USD cannot compute a bound for such a prim either way, and an
    invented extent would be the only thing claiming otherwise.

    Args:
        gprim: The freshly defined ``UsdGeom`` shape, with its size and axis
               attributes already authored.
    """
    extent = UsdGeom.Boundable.ComputeExtentFromPlugins(gprim, Usd.TimeCode.Default())
    if extent:
        gprim.CreateExtentAttr(extent)


def _add_collider(xform_prim: Usd.Prim):
    UsdPhysics.CollisionAPI.Apply(xform_prim)


def _add_rigid(xform_prim: Usd.Prim, density: float, lin_velocity: Gf.Vec3f, ang_velocity: Gf.Vec3f):
    _add_collider(xform_prim)
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
    color: Gf.Vec3f = Gf.Vec3f(1.0),
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
    density: float = 1.0,
    lin_velocity: Gf.Vec3f = Gf.Vec3f(0.0),
    ang_velocity: Gf.Vec3f = Gf.Vec3f(0.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Cube to the stage and add physics rigid body and collider API to it.

    A ``density`` of 0.0 still applies ``UsdPhysics.RigidBodyAPI`` and
    ``UsdPhysics.MassAPI``; it is not a request for a static collider. Use
    add_collider_box for that.

    Args:
        stage:      The Usd.Stage to add cube.
        path:       The desired cube path.
        size:       The size of the cube.
        position:   The position where the cube should be placed in stage.
        orientation:   The cube orientation.
        color:      The color of the mesh.
        density:    The density of the rigid body.
        lin_velocity: The initial linear velocity of the rigid body.
        ang_velocity: The initial angular velocity of the rigid body.
    """
    cube_prim = add_box(stage, path, size, position, orientation, color)
    _add_rigid(cube_prim, density, lin_velocity, ang_velocity)
    return cube_prim


def add_cube(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    size: Gf.Vec3f = Gf.Vec3f(1.0),
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    orientation: Gf.Quatf = Gf.Quatf(1.0),
    color: Gf.Vec3f = Gf.Vec3f(1.0),
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
    return add_collider_box(stage, path, size, position, orientation, color)


def add_rigid_cube(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    size: Gf.Vec3f = Gf.Vec3f(1.0),
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    orientation: Gf.Quatf = Gf.Quatf(1.0),
    color: Gf.Vec3f = Gf.Vec3f(1.0),
    density: float = 1.0,
    lin_velocity: Gf.Vec3f = Gf.Vec3f(0.0),
    ang_velocity: Gf.Vec3f = Gf.Vec3f(0.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Cube to the stage and add physics rigid body and collider API to it.

    A ``density`` of 0.0 still applies ``UsdPhysics.RigidBodyAPI`` and
    ``UsdPhysics.MassAPI``; it is not a request for a static collider. Use
    add_collider_cube for that.

    Args:
        stage:      The Usd.Stage to add cube.
        path:       The desired cube path.
        size:       The size of the cube.
        position:   The position where the cube should be placed in stage.
        orientation:   The cube orientation.
        color:      The color of the mesh.
        density:    The density of the rigid body.
        lin_velocity: The initial linear velocity of the rigid body.
        ang_velocity: The initial angular velocity of the rigid body.
    """
    return add_rigid_box(stage, path, size, position, orientation, color, density, lin_velocity, ang_velocity)


def add_sphere(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    radius: float = 1.0,
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    orientation: Gf.Quatf = Gf.Quatf(1.0),
    color: Gf.Vec3f = Gf.Vec3f(1.0),
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
    radius: float = 1.0,
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    orientation: Gf.Quatf = Gf.Quatf(1.0),
    color: Gf.Vec3f = Gf.Vec3f(1.0),
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
    radius: float = 1.0,
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    orientation: Gf.Quatf = Gf.Quatf(1.0),
    color: Gf.Vec3f = Gf.Vec3f(1.0),
    density: float = 1.0,
    lin_velocity: Gf.Vec3f = Gf.Vec3f(0.0),
    ang_velocity: Gf.Vec3f = Gf.Vec3f(0.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Sphere to the stage and add physics rigid body and collider API to it.

    A ``density`` of 0.0 still applies ``UsdPhysics.RigidBodyAPI`` and
    ``UsdPhysics.MassAPI``; it is not a request for a static collider. Use
    add_collider_sphere for that.

    Args:
        stage:      The Usd.Stage to add sphere.
        path:       The desired sphere path.
        radius:     The radius of the sphere.
        position:   The position where the sphere should be placed in stage.
        orientation:   The sphere orientation.
        color:      The color of the mesh.
        density:    The density of the rigid body.
        lin_velocity: The initial linear velocity of the rigid body.
        ang_velocity: The initial angular velocity of the rigid body.
    """
    sphere_prim = add_sphere(stage, path, radius, position, orientation, color)
    _add_rigid(sphere_prim, density, lin_velocity, ang_velocity)
    return sphere_prim


def add_capsule(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    radius: float = 1.0,
    height: float = 1.0,
    axis: str = "Y",
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    orientation: Gf.Quatf = Gf.Quatf(1.0),
    color: Gf.Vec3f = Gf.Vec3f(1.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Capsule to the stage.

    The extent authored here is the one UsdGeom.Capsule's own schema
    computation produces, so it follows ``axis`` and reaches
    ``height / 2 + radius`` along it -- the cylindrical section plus a
    hemispherical cap at each end -- and ``radius`` across it.

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
    capsule_geom.CreateAxisAttr(axis)
    _author_computed_extent(capsule_geom)
    capsule_geom.CreateDisplayColorAttr().Set([color])
    _add_transformation(capsule_geom, position, orientation)
    return stage.GetPrimAtPath(path)


def add_collider_capsule(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    radius: float = 1.0,
    height: float = 1.0,
    axis: str = "Y",
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    orientation: Gf.Quatf = Gf.Quatf(1.0),
    color: Gf.Vec3f = Gf.Vec3f(1.0),
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
    radius: float = 1.0,
    height: float = 1.0,
    axis: str = "Y",
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    orientation: Gf.Quatf = Gf.Quatf(1.0),
    color: Gf.Vec3f = Gf.Vec3f(1.0),
    density: float = 1.0,
    lin_velocity: Gf.Vec3f = Gf.Vec3f(0.0),
    ang_velocity: Gf.Vec3f = Gf.Vec3f(0.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Capsule to the stage and add physics rigid body and collider API to it.

    A ``density`` of 0.0 still applies ``UsdPhysics.RigidBodyAPI`` and
    ``UsdPhysics.MassAPI``; it is not a request for a static collider. Use
    add_collider_capsule for that.

    Args:
        stage:      The Usd.Stage to add capsule.
        path:       The desired capsule path.
        radius:     The radius of the capsule.
        height:     The height of the capsule.
        axis:       The axis of the capsule.
        position:   The position where the capsule should be placed in stage.
        orientation:   The capsule orientation.
        color:      The color of the mesh.
        density:    The density of the rigid body.
        lin_velocity: The initial linear velocity of the rigid body.
        ang_velocity: The initial angular velocity of the rigid body.
    """
    capsule_prim = add_capsule(stage, path, radius, height, axis, position, orientation, color)
    _add_rigid(capsule_prim, density, lin_velocity, ang_velocity)
    return capsule_prim


def add_cylinder(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    radius: float = 1.0,
    height: float = 1.0,
    axis: str = "Y",
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    orientation: Gf.Quatf = Gf.Quatf(1.0),
    color: Gf.Vec3f = Gf.Vec3f(1.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Cylinder to the stage.

    The extent authored here is the one UsdGeom.Cylinder's own schema
    computation produces, so it follows ``axis`` and reaches ``height / 2``
    along it and ``radius`` across it.

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
    cylinder_geom.CreateAxisAttr(axis)
    _author_computed_extent(cylinder_geom)
    cylinder_geom.CreateDisplayColorAttr().Set([color])
    _add_transformation(cylinder_geom, position, orientation)
    return stage.GetPrimAtPath(path)


def add_collider_cylinder(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    radius: float = 1.0,
    height: float = 1.0,
    axis: str = "Y",
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    orientation: Gf.Quatf = Gf.Quatf(1.0),
    color: Gf.Vec3f = Gf.Vec3f(1.0),
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
    radius: float = 1.0,
    height: float = 1.0,
    axis: str = "Y",
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    orientation: Gf.Quatf = Gf.Quatf(1.0),
    color: Gf.Vec3f = Gf.Vec3f(1.0),
    density: float = 1.0,
    lin_velocity: Gf.Vec3f = Gf.Vec3f(0.0),
    ang_velocity: Gf.Vec3f = Gf.Vec3f(0.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Cylinder to the stage and add physics rigid body and collider API to it.

    A ``density`` of 0.0 still applies ``UsdPhysics.RigidBodyAPI`` and
    ``UsdPhysics.MassAPI``; it is not a request for a static collider. Use
    add_collider_cylinder for that.

    Args:
        stage:      The Usd.Stage to add cylinder.
        path:       The desired cylinder path.
        radius:     The radius of the cylinder.
        height:     The height of the cylinder.
        axis:       The axis of the cylinder.
        position:   The position where the cylinder should be placed in stage.
        orientation:   The cylinder orientation.
        color:      The color of the mesh.
        density:    The density of the rigid body.
        lin_velocity: The initial linear velocity of the rigid body.
        ang_velocity: The initial angular velocity of the rigid body.
    """
    cylinder_prim = add_cylinder(stage, path, radius, height, axis, position, orientation, color)
    _add_rigid(cylinder_prim, density, lin_velocity, ang_velocity)
    return cylinder_prim


def add_cone(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    radius: float = 1.0,
    height: float = 1.0,
    axis: str = "Y",
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    orientation: Gf.Quatf = Gf.Quatf(1.0),
    color: Gf.Vec3f = Gf.Vec3f(1.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Cone to the stage.

    The extent authored here is the one UsdGeom.Cone's own schema computation
    produces, so it follows ``axis`` and reaches ``height / 2`` along it and
    ``radius`` across it.

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
    cone_geom.CreateAxisAttr(axis)
    _author_computed_extent(cone_geom)
    cone_geom.CreateDisplayColorAttr().Set([color])
    _add_transformation(cone_geom, position, orientation)
    return stage.GetPrimAtPath(path)


def add_collider_cone(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    radius: float = 1.0,
    height: float = 1.0,
    axis: str = "Y",
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    orientation: Gf.Quatf = Gf.Quatf(1.0),
    color: Gf.Vec3f = Gf.Vec3f(1.0),
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
    radius: float = 1.0,
    height: float = 1.0,
    axis: str = "Y",
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    orientation: Gf.Quatf = Gf.Quatf(1.0),
    color: Gf.Vec3f = Gf.Vec3f(1.0),
    density: float = 1.0,
    lin_velocity: Gf.Vec3f = Gf.Vec3f(0.0),
    ang_velocity: Gf.Vec3f = Gf.Vec3f(0.0),
) -> Usd.Prim:
    """
    Add UsdGeom.Cone to the stage and add physics rigid body and collider API to it.

    A ``density`` of 0.0 still applies ``UsdPhysics.RigidBodyAPI`` and
    ``UsdPhysics.MassAPI``; it is not a request for a static collider. Use
    add_collider_cone for that.

    Args:
        stage:      The Usd.Stage to add cone.
        path:       The desired cone path.
        radius:     The radius of the cone.
        height:     The height of the cone.
        axis:       The axis of the cone.
        position:   The position where the cone should be placed in stage.
        orientation:   The cone orientation.
        color:      The color of the mesh.
        density:    The density of the rigid body.
        lin_velocity: The initial linear velocity of the rigid body.
        ang_velocity: The initial angular velocity of the rigid body.
    """
    cone_prim = add_cone(stage, path, radius, height, axis, position, orientation, color)
    _add_rigid(cone_prim, density, lin_velocity, ang_velocity)
    return cone_prim


def add_xform(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    orientation: Gf.Quatf = Gf.Quatf(1.0),
    scale: Gf.Vec3f = Gf.Vec3f(1.0),
) -> Usd.Prim:
    """
    Add xform to the stage with given transformation.

    Args:
        stage:      The Usd.Stage to add the xform.
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
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    orientation: Gf.Quatf = Gf.Quatf(1.0),
    scale: Gf.Vec3f = Gf.Vec3f(1.0),
) -> Usd.Prim:
    """
    Add xform to the stage with given transformation and add rigid body API to it.

    Args:
        stage:      The Usd.Stage to add the xform.
        path:       The desired xform path.
        position:   The position where the xform should be placed in stage.
        orientation:   The xform orientation.
        scale:      The xform scale.
    """
    xform_prim = add_xform(stage, path, position, orientation, scale)
    physicsAPI = UsdPhysics.RigidBodyAPI.Apply(xform_prim)
    physicsAPI.CreateRigidBodyEnabledAttr(True)
    return xform_prim
