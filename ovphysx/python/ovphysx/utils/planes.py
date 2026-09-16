# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-UTILS-001
# @covers AC-1 AC-2 AC-5 AC-12 AC-15

"""Ground planes and sized quad planes.

A ``UsdPhysics.Plane`` is infinite and renders as nothing, so a usable ground
plane is two prims: a mesh for the viewport and a collision prim beside it.
:func:`add_ground_plane` authors that pair under one Xform;
:func:`add_quad_plane` and :func:`add_cube_ground_plane` instead give the
collider the finite extent of the geometry itself, which is what a scene needs
when a body must be able to fall off the edge.
"""

import typing

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from .authoring import set_collider
from .mesh import create_mesh_square_axis
from .paths import get_stage_next_free_path

__all__ = [
    "add_ground_plane",
    "add_quad_plane",
    "add_cube_ground_plane",
    "add_plane_collider",
]


def add_ground_plane(
    stage: Usd.Stage,
    plane_path: typing.Union["str", Sdf.Path],
    axis: str,
    size: float,
    position: typing.Union[Gf.Vec3f, Gf.Vec3d],
    color: Gf.Vec3f,
) -> str:
    """
    Add ground plane to the stage. Note that it will add
    a mesh for rendering purpose and UsdPhysics.Plane for collision
    purpose.

    Args:
        stage:      The Usd.Stage to add path.
        plane_path: The desired ground plane path.
        axis:       The up axis - "Y", "Z"
        size:       The half size of the mesh.
        position:   The position where the mesh should be placed in stage.
        color:      The color of the mesh.
    """
    # An Xform parent, so the two geom prims below are siblings rather than nested.
    plane_path = get_stage_next_free_path(stage, plane_path, True)
    planeXform = UsdGeom.Xform.Define(stage, plane_path)
    planeXform.AddTranslateOp().Set(position)
    planeXform.AddOrientOp().Set(Gf.Quatf(1.0))
    planeXform.AddScaleOp().Set(Gf.Vec3f(1.0))

    # (Graphics) Plane mesh
    geomPlanePath = plane_path + "/CollisionMesh"
    entityPlane = create_mesh_square_axis(stage, geomPlanePath, axis, size)
    entityPlane.CreateDisplayColorAttr().Set([color])

    # (Collision) Plane
    colPlanePath = plane_path + "/CollisionPlane"
    planeGeom = UsdGeom.Plane.Define(stage, colPlanePath)
    planeGeom.CreatePurposeAttr().Set("guide")
    planeGeom.CreateAxisAttr().Set(axis)

    prim = stage.GetPrimAtPath(colPlanePath)
    UsdPhysics.CollisionAPI.Apply(prim)

    return plane_path


def add_quad_plane(
    stage: Usd.Stage,
    quad_path: typing.Union["str", Sdf.Path],
    axis: str,
    size: float,
    position: typing.Union[Gf.Vec3f, Gf.Vec3d],
    color: Gf.Vec3f,
):
    """
    Add quad mesh to the stage to act as a sized plane.

    Args:
        stage:      The Usd.Stage to add path.
        quad_path:  The desired ground plane path.
        axis:       The up axis - "Y", "Z"
        size:       The half size of the mesh.
        position:   The position where the mesh should be placed in stage.
        color:      The color of the mesh.
    """
    planePath = get_stage_next_free_path(stage, quad_path, True)
    entityPlane = create_mesh_square_axis(stage, planePath, axis, size)
    entityPlane.CreateDisplayColorAttr().Set([color])
    entityPlane.AddTranslateOp().Set(position)
    entityPlane.AddOrientOp().Set(Gf.Quatf(1.0))
    entityPlane.AddScaleOp().Set(Gf.Vec3f(1.0))

    UsdPhysics.CollisionAPI.Apply(entityPlane.GetPrim())


def add_cube_ground_plane(
    stage: Usd.Stage,
    cube_path: typing.Union["str", Sdf.Path],
    axis: str,
    size: float,
    position: typing.Union[Gf.Vec3f, Gf.Vec3d],
    color: Gf.Vec3f,
):
    """
    Add UsdGeom.Cube to the stage to act as a sized plane with thickness.
    The cube is scaled by a vector Gf.Vec3f(0.01, 1.0, 1.0) depending on the up Axis

    ``size`` is the cube's full edge length, which is what ``UsdGeom.Cube``'s own
    ``size`` attribute means, so ``size=2`` reaches ``[-1, 1]`` across the two
    lateral axes. On :func:`add_ground_plane` and :func:`add_quad_plane` ``size``
    is a half size instead.

    Args:
        stage:      The Usd.Stage to add path.
        cube_path:  The desired ground plane path.
        axis:       The up axis - "Y", "Z"
        size:       The edge length of the cube, before the flattening scale.
        position:   The position where the mesh should be placed in stage.
        color:      The color of the mesh.
    """
    cube_path = get_stage_next_free_path(stage, cube_path, True)
    cubeGeom = UsdGeom.Cube.Define(stage, cube_path)
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


def add_plane_collider(stage: Usd.Stage, prim_path: typing.Union[str, Sdf.Path], up_axis: str):
    """
    Define a guide-purpose UsdGeom.Plane and make it a collider.

    An occupied path raises ``ValueError`` and is not modified.

    Args:
        stage:      The Usd.Stage to add the plane.
        prim_path:  The desired plane path.
        up_axis:    The plane's up axis.
    """
    if stage.GetPrimAtPath(prim_path):
        raise ValueError(f"{prim_path} is already held by a prim")

    plane = UsdGeom.Plane.Define(stage, prim_path)
    plane.CreateAxisAttr().Set(up_axis)
    plane.CreatePurposeAttr().Set("guide")
    planePrim = stage.GetPrimAtPath(prim_path)
    set_collider(planePrim)
