# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-UTILS-001
# @covers AC-1 AC-2 AC-6 AC-12 AC-15

"""Procedural mesh construction and triangle / tetrahedron mesh math.

Two groups of helper live here: constructors that define a ``UsdGeom.Mesh`` for
a primitive shape, and pure geometry routines over point and index lists that
carry no USD state.

``create_tetra_voxels`` yields an empty mesh for a non-positive integer
``voxel_dim`` rather than raising.
"""

import collections
import itertools
import logging
import math
import typing

from pxr import Gf, Sdf, Usd, UsdGeom

from ._deprecation import deprecated_alias

logger = logging.getLogger(__name__)

__all__ = [
    "create_mesh",
    "create_mesh_square_axis",
    "create_mesh_concave",
    "create_mesh_cube",
    "create_mesh_cylinder",
    "create_mesh_cone",
    "compute_bounding_box_diagonal",
    "triangulate_mesh",
    "extract_triangle_surface_from_tetra",
    "create_triangle_mesh_square",
    "calculate_tetra_volume",
    "fixup_tetra_mesh_volumes",
    "verify_tetra_mesh",
    "cube_tetrahedra",
    "create_tetra_voxels",
    "create_tetra_voxel_box",
    "create_tetra_voxel_sphere",
    "create_triangle_mesh_cube",
    "convert_tetra_to_triangle_soup",
]


def create_mesh(
    stage: Usd.Stage,
    path: typing.Union[str, Sdf.Path],
    points: typing.Union[typing.List[Gf.Vec3f], typing.List[Gf.Vec3d]],
    normals: typing.Union[typing.List[Gf.Vec3f], typing.List[Gf.Vec3d]],
    indices: typing.List[int],
    vertex_counts: typing.List[int],
) -> UsdGeom.Mesh:
    """
    Create UsdGeom.Mesh from given points, normals, indices and face counts.

    Args:
        stage:      The Usd.Stage to add path.
        path:       The desired path to create.
        points:     The input points.
        normals:    The input normals.
        indices:    The indices for faces.
        vertex_counts:    Face counts.
    """
    mesh = UsdGeom.Mesh.Define(stage, path)

    mesh.CreateFaceVertexCountsAttr().Set(vertex_counts)
    mesh.CreateFaceVertexIndicesAttr().Set(indices)
    mesh.CreatePointsAttr().Set(points)
    mesh.CreateDoubleSidedAttr().Set(False)
    mesh.CreateNormalsAttr().Set(normals)

    return mesh


def create_mesh_square_axis(
    stage: Usd.Stage, path: typing.Union[str, Sdf.Path], axis: str, half_size: float
) -> UsdGeom.Mesh:
    """
    Create UsdGeom.Mesh that represents a square.

    Args:
        stage:      The Usd.Stage to add path.
        path:       The desired path to create.
        axis:       The up axis "Y", "Z".
        half_size:  The half size of the square.
    """
    if axis == "X":
        points = [
            Gf.Vec3f(0.0, -half_size, -half_size),
            Gf.Vec3f(0.0, half_size, -half_size),
            Gf.Vec3f(0.0, half_size, half_size),
            Gf.Vec3f(0.0, -half_size, half_size),
        ]
        normals = [Gf.Vec3f(1, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(1, 0, 0)]
        indices = [0, 1, 2, 3]
        vertexCounts = [4]

        return create_mesh(stage, path, points, normals, indices, vertexCounts)
    elif axis == "Y":
        points = [
            Gf.Vec3f(-half_size, 0.0, -half_size),
            Gf.Vec3f(half_size, 0.0, -half_size),
            Gf.Vec3f(half_size, 0.0, half_size),
            Gf.Vec3f(-half_size, 0.0, half_size),
        ]
        normals = [Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 1, 0)]
        indices = [3, 2, 1, 0]
        vertexCounts = [4]

        return create_mesh(stage, path, points, normals, indices, vertexCounts)

    points = [
        Gf.Vec3f(-half_size, -half_size, 0.0),
        Gf.Vec3f(half_size, -half_size, 0.0),
        Gf.Vec3f(half_size, half_size, 0.0),
        Gf.Vec3f(-half_size, half_size, 0.0),
    ]
    normals = [Gf.Vec3f(0, 0, 1), Gf.Vec3f(0, 0, 1), Gf.Vec3f(0, 0, 1), Gf.Vec3f(0, 0, 1)]
    indices = [0, 1, 2, 3]
    vertexCounts = [4]

    mesh = create_mesh(stage, path, points, normals, indices, vertexCounts)

    texCoords = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.varying
    )
    texCoords.Set([(0, 0), (1, 0), (1, 1), (0, 1)])

    return mesh


def create_mesh_concave(stage: Usd.Stage, path: typing.Union[str, Sdf.Path], half_size: float) -> UsdGeom.Mesh:
    """
    Create UsdGeom.Mesh that represents a concave mesh.

    Args:
        stage:      The Usd.Stage to add path.
        path:       The desired path to create.
        half_size:  The half size of the mesh.
    """
    points = [
        Gf.Vec3f(half_size, -half_size, -half_size),
        Gf.Vec3f(half_size, half_size, -half_size),
        Gf.Vec3f(half_size, half_size, half_size),
        Gf.Vec3f(half_size, -half_size, half_size),
        Gf.Vec3f(0.0, -half_size, half_size * 0.2),
        Gf.Vec3f(0.0, half_size, half_size * 0.2),
        Gf.Vec3f(-half_size, -half_size, -half_size),
        Gf.Vec3f(-half_size, half_size, -half_size),
        Gf.Vec3f(-half_size, half_size, half_size),
        Gf.Vec3f(-half_size, -half_size, half_size),
    ]
    normals = [
        Gf.Vec3f(1, 0, 0),
        Gf.Vec3f(1, 0, 0),
        Gf.Vec3f(1, 0, 0),
        Gf.Vec3f(1, 0, 0),
        Gf.Vec3f(0, 0, 1),
        Gf.Vec3f(0, 0, 1),
        Gf.Vec3f(-1, 0, 0),
        Gf.Vec3f(-1, 0, 0),
        Gf.Vec3f(-1, 0, 0),
        Gf.Vec3f(-1, 0, 0),
    ]
    indices = [0, 1, 2, 3, 1, 7, 8, 5, 2, 3, 2, 5, 4, 4, 5, 8, 9, 9, 8, 7, 6, 0, 6, 7, 1, 0, 3, 4, 9, 6]
    vertexCounts = [4, 5, 4, 4, 4, 4, 5]

    return create_mesh(stage, path, points, normals, indices, vertexCounts)


def create_mesh_cube(stage: Usd.Stage, path: typing.Union[str, Sdf.Path], half_size: float) -> UsdGeom.Mesh:
    """
    Create UsdGeom.Mesh that represents a cube mesh.

    Args:
        stage:      The Usd.Stage to add path.
        path:       The desired path to create.
        half_size:  The half size of the cube.
    """
    points = [
        Gf.Vec3f(half_size, -half_size, -half_size),
        Gf.Vec3f(half_size, half_size, -half_size),
        Gf.Vec3f(half_size, half_size, half_size),
        Gf.Vec3f(half_size, -half_size, half_size),
        Gf.Vec3f(-half_size, -half_size, -half_size),
        Gf.Vec3f(-half_size, half_size, -half_size),
        Gf.Vec3f(-half_size, half_size, half_size),
        Gf.Vec3f(-half_size, -half_size, half_size),
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

    return create_mesh(stage, path, points, normals, indices, vertexCounts)


def create_mesh_cylinder(
    stage: Usd.Stage, path: typing.Union[str, Sdf.Path], height: float, radius: float, tesselation: int = 32
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
        points.append(Gf.Vec3f(angle_cos * radius, angle_sin * radius, +height / 2))
        normals.append(Gf.Vec3f(0, 0, 1))
        # Top facing sideways.
        points.append(Gf.Vec3f(angle_cos * radius, angle_sin * radius, +height / 2))
        normals.append(side_normal)
        # Bottom facing downwards
        points.append(Gf.Vec3f(angle_cos * radius, angle_sin * radius, -height / 2))
        normals.append(Gf.Vec3f(0, 0, -1))
        # Bottom facing sideways
        points.append(Gf.Vec3f(angle_cos * radius, angle_sin * radius, -height / 2))
        normals.append(side_normal)

        # Top
        indices.append(i * 4)  # Edge
        indices.append((tesselation) * 4)  # Top center
        # Previous edge
        if i > 0:
            indices.append((i - 1) * 4)
        else:
            indices.append((tesselation - 1) * 4)

        # Upper sideways
        indices.append(i * 4 + 1)
        indices.append(i * 4 + 3)  # Edge
        # Previous edge
        if i > 0:
            indices.append((i - 1) * 4 + 1)
        else:
            indices.append((tesselation - 1) * 4 + 1)

        # Bottom
        indices.append((tesselation) * 4 + 1)  # Bottom center
        indices.append(i * 4 + 2)  # Edge
        # Previous edge
        if i > 0:
            indices.append((i - 1) * 4 + 2)
        else:
            indices.append((tesselation - 1) * 4 + 2)

        # Lower sideways
        indices.append(i * 4 + 3)  # Edge
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

    return create_mesh(stage, path, points, normals, indices, vertexCounts)


def create_mesh_cone(
    stage: Usd.Stage, path: typing.Union[str, Sdf.Path], height: float, radius: float, tesselation: int = 32
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
                height / 2 - height * intersection_offset,
            )
        )

        normals.append(normal)

        # Base vertex sideways
        points.append(Gf.Vec3f(angle_cos * radius, angle_sin * radius, -height / 2))

        normals.append(normal)

        # Base vertex downwards.
        points.append(Gf.Vec3f(angle_cos * radius, angle_sin * radius, -height / 2))

        normal = Gf.Vec3f(0, 0, -1)
        normals.append(normal)

        # Tip section
        indices.append(i * 4 + 1)
        indices.append(i * 4)  # Tip vertex
        # Previous vertex
        if i > 0:
            indices.append((i - 1) * 4 + 1)
        else:
            indices.append((tesselation - 1) * 4 + 1)

        # Upper sideways
        indices.append(i * 4 + 2)  # Base vertex
        indices.append(i * 4 + 1)  # Intersection vertex
        # Previous vertex
        if i > 0:
            indices.append((i - 1) * 4 + 1)
        else:
            indices.append((tesselation - 1) * 4 + 1)

        # Base sideways
        indices.append(i * 4 + 2)  # Base vertex
        # Previous vertex
        if i > 0:
            indices.append((i - 1) * 4 + 1)
            indices.append((i - 1) * 4 + 2)
        else:
            indices.append((tesselation - 1) * 4 + 1)
            indices.append((tesselation - 1) * 4 + 2)

        # Base downwards
        indices.append(tesselation * 4)  # Base center
        indices.append(i * 4 + 3)  # Base vertex
        # Previous
        if i > 0:
            indices.append((i - 1) * 4 + 3)
        else:
            indices.append((tesselation - 1) * 4 + 3)

    # Base center vertex.
    points.append(Gf.Vec3f(0, 0, -height / 2))
    normals.append(Gf.Vec3f(0, 0, -1))

    vertexCounts = [3] * 4 * tesselation

    return create_mesh(stage, path, points, normals, indices, vertexCounts)


def compute_bounding_box_diagonal(points) -> float:
    """
    Gets diagonal length of given point bounds.

    Args:
        points:      The input points. Any iterable of indexable three-component
                     points, e.g. Gf.Vec3f, tuples or lists.

    Raises:
        ValueError: If ``points`` is empty.
    """
    points_iter = iter(points)
    try:
        first = next(points_iter)
    except StopIteration:
        raise ValueError("points must contain at least one point") from None

    v_min_x = v_max_x = first[0]
    v_min_y = v_max_y = first[1]
    v_min_z = v_max_z = first[2]
    for v in points_iter:
        if v[0] < v_min_x:
            v_min_x = v[0]
        if v[0] > v_max_x:
            v_max_x = v[0]
        if v[1] < v_min_y:
            v_min_y = v[1]
        if v[1] > v_max_y:
            v_max_y = v[1]
        if v[2] < v_min_z:
            v_min_z = v[2]
        if v[2] > v_max_z:
            v_max_z = v[2]
    return math.sqrt((v_max_x - v_min_x) ** 2 + (v_max_y - v_min_y) ** 2 + (v_max_z - v_min_z) ** 2)


def triangulate_mesh(mesh: UsdGeom.Mesh) -> typing.List[int]:
    """Fan-triangulate a mesh's faces into a flat triangle index list.

    Args:
        mesh: The UsdGeom.Mesh to read.
    """
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


def extract_triangle_surface_from_tetra(tetra_points, tetra_indices):
    """Extract the outer surface triangles of a tetrahedral mesh.

    Occurrences of each face are counted: a face that appears more than once is
    interior and is discarded, and the faces appearing exactly once form the
    surface.

    Args:
        tetra_points:  The tet mesh points.
        tetra_indices: The tet mesh vertex indices, four per tetrahedron.
    """
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

    face_counts = collections.Counter(tuple(sorted(t)) for t in triangles)
    surface_triangles = [t for t in triangles if face_counts[tuple(sorted(t))] == 1]

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


def create_triangle_mesh_square(dimx: int, dimy: int, scale: float = 1.0):
    """Creates points and vertex data for a regular-grid flat triangle mesh square.

    A non-positive integer grid dimension yields an empty mesh.

    Args:
        dimx:                       Mesh-vertex resolution in X
        dimy:                       Mesh-vertex resolution in Y
        scale:                      Uniform scale applied to vertices

    Returns:
        points, indices:            The vertex and index data
    """
    if dimx <= 0 or dimy <= 0:
        return [], []

    points = [Gf.Vec3f(x, y, 0.0) for y in range(dimy + 1) for x in range(dimx + 1)]
    indices = [-1] * (dimx * dimy) * 2 * 3

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

    center = Gf.Vec3f(0.5, 0.5, 0.0)
    centered = [Gf.Vec3f(p[0] / dimx, p[1] / dimy, p[2]) - center for p in points]
    points = [Gf.Vec3f(scale * p[0], scale * p[1], scale * p[2]) for p in centered]

    return points, indices


def calculate_tetra_volume(a, b, c, d):
    """Compute the signed volume of a tetrahedron.

    Args:
        a, b, c, d: The four tetrahedron corner points.
    """
    a, b, c = a - d, b - d, c - d
    volume = (-1.0 / 6.0) * Gf.Dot(Gf.Cross(a, b), c)
    return volume


def fixup_tetra_mesh_volumes(points, indices):
    """Return tet indices with inverted tetrahedra re-wound to positive volume.

    Args:
        points:  The tet mesh points.
        indices: The tet mesh vertex indices, four per tetrahedron.
    """
    fixed_indices = []

    for t in range(0, len(indices) // 4):
        t0, t1, t2, t3 = indices[t * 4 + 0], indices[t * 4 + 1], indices[t * 4 + 2], indices[t * 4 + 3]
        volume = calculate_tetra_volume(points[t0], points[t1], points[t2], points[t3])
        if volume <= 0.0:
            fixed_indices.extend([t1, t0, t2, t3])
        else:
            fixed_indices.extend([t0, t1, t2, t3])

    return fixed_indices


def verify_tetra_mesh(points, indices):
    """Check a tet mesh for index-count, out-of-range and inverted-volume errors.

    Problems are logged as warnings; the first one found stops the check. An
    index below zero is out of range on the same terms as one past the end.

    Args:
        points:  The tet mesh points.
        indices: The tet mesh vertex indices, four per tetrahedron.
    """
    if len(indices) % 4 != 0:
        logger.warning("verify_tetra_mesh: len(indices) not multiple of 4")
        return

    for i in indices:
        if i < 0 or i >= len(points):
            logger.warning("verify_tetra_mesh: invalid index " + str(i) + ". only " + str(len(points)) + " vertices")
            return

    for t in range(0, len(indices) // 4):
        t0, t1, t2, t3 = indices[t * 4 + 0], indices[t * 4 + 1], indices[t * 4 + 2], indices[t * 4 + 3]
        volume = calculate_tetra_volume(points[t0], points[t1], points[t2], points[t3])
        if volume <= 0.0:
            logger.warning("verify_tetra_mesh: tetra " + str(t) + " has no or negative volume " + str(volume))
            return


def cube_tetrahedra():
    """Return the five-tetrahedron decomposition of a unit cube."""
    tetra = []
    tetra.append([(0, 0, 0), (1, 0, 0), (1, 1, 0), (1, 0, 1)])
    tetra.append([(0, 0, 0), (1, 0, 1), (1, 1, 0), (0, 1, 1)])
    tetra.append([(0, 0, 0), (0, 0, 1), (1, 0, 1), (0, 1, 1)])
    tetra.append([(1, 0, 1), (1, 1, 1), (1, 1, 0), (0, 1, 1)])
    tetra.append([(0, 0, 0), (1, 1, 0), (0, 1, 0), (0, 1, 1)])
    return tetra


def create_tetra_voxels(voxel_dim, occupancy_filter_func):
    """Build a tetrahedral mesh from an occupancy-filtered voxel grid.

    Every occupied voxel is split into the five tetrahedra of
    ``cube_tetrahedra``. Alternate cubes are mirrored per axis, and a cube
    mirrored an odd number of times has its tetrahedra re-wound, so neighbouring
    cubes share faces.

    Args:
        voxel_dim:             The grid resolution, used for all three axes. A
                               non-positive integer yields an empty mesh.
        occupancy_filter_func: Called as (x, y, z, dimx, dimy, dimz); returns
                               True for an occupied voxel.
    """
    dimx, dimy, dimz = voxel_dim, voxel_dim, voxel_dim

    grid = [[[False] * dimz for _ in range(dimy)] for _ in range(dimx)]
    num_voxels = 0
    for x in range(dimx):
        for y in range(dimy):
            for z in range(dimz):
                if occupancy_filter_func(x, y, z, dimx, dimy, dimz):
                    grid[x][y][z] = True
                    num_voxels = num_voxels + 1

    # create vertex grid to compact list map
    grid_to_indices = [[[-1] * (dimz + 1) for _ in range(dimy + 1)] for _ in range(dimx + 1)]

    index = 0
    for x in range(dimx + 1):
        for y in range(dimy + 1):
            for z in range(dimz + 1):
                (x_b, x_e) = (max(x - 1, 0), min(x + 1, dimx))
                (y_b, y_e) = (max(y - 1, 0), min(y + 1, dimy))
                (z_b, z_e) = (max(z - 1, 0), min(z + 1, dimz))
                neighbors = itertools.product(range(x_b, x_e), range(y_b, y_e), range(z_b, z_e))
                if any(grid[nx][ny][nz] for nx, ny, nz in neighbors):
                    grid_to_indices[x][y][z] = index
                    index = index + 1

    points = [0] * index
    for x in range(dimx + 1):
        for y in range(dimy + 1):
            for z in range(dimz + 1):
                point_index = grid_to_indices[x][y][z]
                if point_index > -1:
                    points[point_index] = Gf.Vec3f(x, y, z)

    cube_tetra = cube_tetrahedra()
    indices = [0] * num_voxels * len(cube_tetra) * 4
    index = 0
    for x in range(dimx):
        for y in range(dimy):
            for z in range(dimz):
                if not grid[x][y][z]:
                    continue
                mx, my, mz = x % 2, y % 2, z % 2
                flip = (mx + my + mz) % 2
                for src_tet in cube_tetra:
                    # Placeholder is ints, the four slots then take corner
                    # indices from src_tet, so the element type stays open.
                    tet: list = [-1] * 4
                    if flip:
                        tet[0], tet[1], tet[2], tet[3] = src_tet[1], src_tet[0], src_tet[2], src_tet[3]
                    else:
                        tet = src_tet

                    for cx, cy, cz in tet:
                        wx = mx + (1 - 2 * mx) * cx
                        wy = my + (1 - 2 * my) * cy
                        wz = mz + (1 - 2 * mz) * cz
                        indices[index] = int(grid_to_indices[x + wx][y + wy][z + wz])
                        index = index + 1

    return points, indices


def create_tetra_voxel_box(voxel_dim):
    """Build a unit-cube tetrahedral mesh centered on the origin.

    A non-positive integer grid resolution yields an empty mesh.

    Args:
        voxel_dim: The voxel grid resolution.
    """

    def pass_all_test(x, y, z, dimx, dimy, dimz):
        return True

    points, indices = create_tetra_voxels(voxel_dim, pass_all_test)
    if not points:
        return points, indices
    voxel_dim_inv = 1.0 / voxel_dim
    center = Gf.Vec3f(0.5, 0.5, 0.5)
    return [(p * voxel_dim_inv) - center for p in points], indices


def create_tetra_voxel_sphere(voxel_dim):
    """Build a unit-diameter sphere tetrahedral mesh centered on the origin.

    A non-positive integer grid resolution yields an empty mesh.

    Args:
        voxel_dim: The voxel grid resolution.
    """

    def sphere_test(x, y, z, dimx, dimy, dimz):
        c = Gf.Vec3f(dimx / 2.0, dimy / 2.0, dimz / 2.0)
        r = dimx / 2.0
        v = Gf.Vec3f(x + 0.5, y + 0.5, z + 0.5)
        return (v - c).GetLength() < r

    points, indices = create_tetra_voxels(voxel_dim, sphere_test)
    if not points:
        return points, indices
    voxel_dim_inv = 1.0 / voxel_dim
    center = Gf.Vec3f(0.5, 0.5, 0.5)
    return [(p * voxel_dim_inv) - center for p in points], indices


def create_triangle_mesh_cube(dim: int):
    """Build the surface triangle mesh of a voxelized unit cube.

    A non-positive integer grid resolution yields an empty mesh.

    Args:
        dim: The voxel grid resolution.
    """
    points, indices = create_tetra_voxel_box(dim)
    tri_points, tri_indices = extract_triangle_surface_from_tetra(points, indices)
    return tri_points, tri_indices


def _add_triangle(points, indices, p0, p1, p2):
    """Append one triangle, with its own three vertices, to point and index lists.

    Args:
        points:     The point list to extend.
        indices:    The index list to extend.
        p0, p1, p2: The triangle corner points.
    """
    o = len(points)
    indices.extend([o + 0, o + 1, o + 2])
    points.extend([p0, p1, p2])


def convert_tetra_to_triangle_soup(points_in, indices_in):
    """Expand every tetrahedron into its four unshared triangles.

    Args:
        points_in:  The tet mesh points.
        indices_in: The tet mesh vertex indices, four per tetrahedron.
    """
    points = []
    indices = []
    for t in range(0, len(indices_in) // 4):
        v0, v1, v2, v3 = indices_in[t * 4 + 0], indices_in[t * 4 + 1], indices_in[t * 4 + 2], indices_in[t * 4 + 3]
        p0, p1, p2, p3 = points_in[v0], points_in[v1], points_in[v2], points_in[v3]
        _add_triangle(points, indices, p0, p1, p2)
        _add_triangle(points, indices, p1, p3, p2)
        _add_triangle(points, indices, p0, p3, p1)
        _add_triangle(points, indices, p0, p2, p3)

    return (points, indices)


# Deprecated alias; see the note beside authoring.py's for why it is not in
# __all__ (AC-15).
extractTriangleSurfaceFromTetra = deprecated_alias(
    extract_triangle_surface_from_tetra, "extractTriangleSurfaceFromTetra"
)
