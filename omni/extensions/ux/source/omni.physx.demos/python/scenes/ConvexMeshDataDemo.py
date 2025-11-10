# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
from pxr import UsdGeom, Gf, Usd, UsdPhysics, UsdUtils, PhysicsSchemaTools
import omni.physxdemos as demo
from typing import Any
from omni.physx import get_physx_cooking_interface
from omni.physx.bindings._physx import PhysxCollisionRepresentationResult, PhysxConvexMeshData
from omni.debugdraw import get_debug_draw_interface
from omni.physx.scripts import utils, physicsUtils
import omni


def to_world(point, world_matrix):
    gf_point = Gf.Vec3f(point.x, point.y, point.z)
    world_point = world_matrix.Transform(gf_point)
    return carb.Float3(world_point[0], world_point[1], world_point[2])


# Copy paste from DebugColors.cpp from omni.physx.ui
GLOBAL_HEX_COLORS = [
    0x00188f,
    0x009e49,
    0x00b294,
    0x00bcf2,
    0x17cea3,
    0x34926e,
    0x376495,
    0x4398f2,
    0x44ad57,
    0x4ffccb,
    0x68217a,
    0x7db87d,
    0x7fd51a,
    0x88c674,
    0x991b82,
    0xa079cb,
    0xb7a171,
    0xbad80a,
    0xc6e072,
    0xc8745f,
    0xd11c12,
    0xd1f74d,
    0xd2be4c,
    0xe4b375,
    0xe81123,
    0xec008c,
    0xf55f0d,
    0xf7f219,
    0xff8c00,
    0xfff100
]


class ConvexMeshDatademo(demo.Base):
    title = "Convex Mesh Data"
    category = demo.Categories.SCENE_QUERY
    short_description = "Demo showing convex mesh data usage"
    description = """
The convex mesh data from cooking can be obtained in or out of simulation.
This example specifically obtains before simulation:

- A single convex hull overlapped with yellow outlines on a simple convex mesh using debug draw.
- Multiple convex hulls created as Usd meshes to display the convex decomposition of the torus object.

Entering simulation mode will not cause anything else to happen."""

    def create(self, stage):
        self._stage = stage
        self._debugDraw = get_debug_draw_interface()

        self.defaultPrimPath, scene = demo.setup_physics_scene(
            self, stage, upAxis=UsdGeom.Tokens.y)
        room = demo.get_demo_room(
            self, stage, zoom=0.5, floorOffset=-100.0, hasTable=False, camYaw=0.06)
        self._convexes = None
        self.create_torus_decomposition()
        self.create_convex()

    def create_torus_decomposition(self):
        path_tuple = omni.kit.commands.execute(
            "CreateMeshPrimWithDefaultXform", prim_type="Torus")
        torus_path = path_tuple[1]
        torus_prim = self._stage.GetPrimAtPath(torus_path)
        torus_xform = UsdGeom.Xformable(torus_prim)
        physicsUtils.set_or_add_translate_op(torus_xform, Gf.Vec3f(100, 100, 0))
        UsdPhysics.CollisionAPI.Apply(torus_prim)
        mesh_api = UsdPhysics.MeshCollisionAPI.Apply(torus_prim)
        mesh_api.CreateApproximationAttr(UsdPhysics.Tokens.convexDecomposition)
        physx_cooking = get_physx_cooking_interface()
        stage_id = UsdUtils.StageCache.Get().GetId(self._stage).ToLongInt()
        prim_id = PhysicsSchemaTools.sdfPathToInt(torus_path)
        # Do an asynchronous request for a convex representation
        physx_cooking.request_convex_collision_representation(stage_id=stage_id,
                                                              collision_prim_id=prim_id,
                                                              run_asynchronously=True,  # <-- Asynchronous
                                                              on_result=self.on_torus_representation_ready)

    def on_torus_representation_ready(self, result: PhysxCollisionRepresentationResult, convexes: list[PhysxConvexMeshData]):
        if result == PhysxCollisionRepresentationResult.RESULT_VALID:
            root_path = "/World/torus_decomposition"
            convex_xform: UsdGeom.Xform = UsdGeom.Xform.Define(
                self._stage, root_path)
            convex_xform.AddTranslateOp().Set(Gf.Vec3f(100.0, 0.0, 0.0))
            # We need to sort the convexes because convex decomposition order of reported convexes is not reproducible
            sorted_convex = sorted(convexes, key=lambda convex: (len(convex.indices),
                                                                 len(convex.vertices),
                                                                 len(convex.polygons),
                                                                 convex.vertices[0].x,
                                                                 convex.vertices[0].y,
                                                                 convex.vertices[0].z))
            for idx in range(len(sorted_convex)):
                self._build_usd_mesh(
                    self._stage, f"{root_path}/Convex_{idx}", sorted_convex[idx], idx / len(sorted_convex))

    def create_convex(self):
        root_path = self.defaultPrimPath + "convex_xform"
        convex_xform: UsdGeom.Xform = UsdGeom.Xform.Define(
            self._stage, root_path)
        convex_xform.AddTranslateOp().Set(Gf.Vec3f(-100.0, 0.0, 0.0))
        convex_path = root_path + "/convexActor"
        convex_geom = UsdGeom.Mesh.Define(self._stage, convex_path)
        convex_prim = self._stage.GetPrimAtPath(convex_path)
        self._convexes = None
        faceVertexCounts = [4, 4, 4, 4, 4, 4]
        faceVertexIndices = [0, 1, 3, 2, 4, 5, 7, 6, 10, 11,
                             13, 12, 14, 15, 9, 8, 17, 23, 21, 19, 22, 16, 18, 20]
        convexSize = 50
        convexSize2 = 25
        points = [
            Gf.Vec3f(-convexSize, convexSize, -convexSize), Gf.Vec3f(convexSize, convexSize,
                                                                     - convexSize), Gf.Vec3f(-convexSize2, convexSize2, convexSize), Gf.Vec3f(convexSize2, convexSize2, convexSize),
            Gf.Vec3f(-convexSize2, -convexSize2, convexSize), Gf.Vec3f(convexSize2, -convexSize2,
                                                                       convexSize), Gf.Vec3f(-convexSize, -convexSize, -convexSize), Gf.Vec3f(convexSize, -convexSize, -convexSize),
            Gf.Vec3f(-convexSize, convexSize, -convexSize), Gf.Vec3f(convexSize, convexSize,
                                                                     - convexSize), Gf.Vec3f(-convexSize2, convexSize2, convexSize), Gf.Vec3f(convexSize2, convexSize2, convexSize),
            Gf.Vec3f(-convexSize2, -convexSize2, convexSize), Gf.Vec3f(convexSize2, -convexSize2,
                                                                       convexSize), Gf.Vec3f(-convexSize, -convexSize, -convexSize), Gf.Vec3f(convexSize, -convexSize, -convexSize),
            Gf.Vec3f(-convexSize, convexSize, -convexSize), Gf.Vec3f(convexSize, convexSize,
                                                                     - convexSize), Gf.Vec3f(-convexSize2, convexSize2, convexSize), Gf.Vec3f(convexSize2, convexSize2, convexSize),
            Gf.Vec3f(-convexSize2, -convexSize2, convexSize), Gf.Vec3f(convexSize2, -convexSize2,
                                                                       convexSize), Gf.Vec3f(-convexSize, -convexSize, -convexSize), Gf.Vec3f(convexSize, -convexSize, -convexSize),
        ]

        convex_geom.CreateFaceVertexCountsAttr(faceVertexCounts)
        convex_geom.CreateFaceVertexIndicesAttr(faceVertexIndices)
        convex_geom.CreatePointsAttr(points)
        convex_geom.CreateDisplayColorAttr().Set([demo.get_primary_color()])
        UsdPhysics.CollisionAPI.Apply(convex_prim)
        mesh_api = UsdPhysics.MeshCollisionAPI.Apply(convex_prim)
        mesh_api.CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
        self._mesh_prim = convex_prim
        physx_cooking = get_physx_cooking_interface()
        stage_id = UsdUtils.StageCache.Get().GetId(self._stage).ToLongInt()
        prim_id = PhysicsSchemaTools.sdfPathToInt(convex_path)
        # Do a synchronous request for a convex representation
        physx_cooking.request_convex_collision_representation(stage_id=stage_id,
                                                              collision_prim_id=prim_id,
                                                              run_asynchronously=False,  # <-- Synchronous
                                                              on_result=self.on_convex_representation_ready)

    def on_convex_representation_ready(self, result: PhysxCollisionRepresentationResult, convexes: list[Any]):
        if result == PhysxCollisionRepresentationResult.RESULT_VALID:
            self._convexes = convexes

    @staticmethod
    def _build_usd_mesh(stage: Usd.Stage, new_path: str, convex: PhysxConvexMeshData, param):
        mesh = UsdGeom.Mesh.Define(stage, new_path)

        indices = convex.indices
        vertices = convex.vertices
        polygons = convex.polygons

        points = []
        face_vertex_counts = []
        face_vertex_indices = []

        for vertex in vertices:
            points.append(Gf.Vec3f(vertex.x, vertex.y, vertex.z))

        for polygon in polygons:
            index_base = polygon.index_base
            num_vertices = polygon.num_vertices
            face_vertex_counts.append(num_vertices)
            face_vertex_indices.extend(
                indices[index_base:index_base + num_vertices])

        mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
        mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)
        mesh.CreatePointsAttr(points)
        color_index = int(param * len(GLOBAL_HEX_COLORS))

        def convert_hex_to_color(hex_value):
            r = ((hex_value >> 16) & 0xFF) / 255.0
            g = ((hex_value >> 8) & 0xFF) / 255.0
            b = (hex_value & 0xFF) / 255.0
            return Gf.Vec3f(r, g, b)

        mesh.CreateDisplayColorAttr().Set(
            [convert_hex_to_color(GLOBAL_HEX_COLORS[color_index])])
        mesh.GetSubdivisionSchemeAttr().Set("none")

    def update(self, stage, dt, viewport, physxIFace):
        # Only draw after we've received the convexes
        if self._convexes:
            self._draw_convex_hulls(self._convexes)

    def _draw_convex_hulls(self, convex_hulls: list[Any]):
        source_to_world = UsdGeom.Xformable(
            self._mesh_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        for convex_hull_data in convex_hulls:
            if len(convex_hull_data.polygons) > 0:
                indices = convex_hull_data.indices
                vertices = convex_hull_data.vertices
                polygons = convex_hull_data.polygons
                for poly_index in range(len(convex_hull_data.polygons)):
                    index_base = polygons[poly_index].index_base
                    previous_index = -1
                    for vertex_index in range(polygons[poly_index].num_vertices):
                        current_index = indices[index_base + vertex_index]
                        if previous_index != -1:
                            color = 0xffffff00
                            point0 = to_world(
                                vertices[previous_index], source_to_world)
                            point1 = to_world(
                                vertices[current_index], source_to_world)
                            self._debugDraw.draw_line(
                                point0, color, point1, color)
                        previous_index = current_index
                    point0 = to_world(
                        vertices[previous_index], source_to_world)
                    point1 = to_world(
                        vertices[indices[index_base]], source_to_world)
                    self._debugDraw.draw_line(
                        point0, color, 1.0, point1, color, 1.0)
