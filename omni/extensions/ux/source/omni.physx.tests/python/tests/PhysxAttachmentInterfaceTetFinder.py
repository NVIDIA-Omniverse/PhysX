# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import Gf, Sdf, Vt, UsdGeom, Usd, UsdPhysics, UsdLux, PhysxSchema
from omni.physxtests.utils.physicsBase import PhysicsBaseAsyncTestCase, TestCategory
from omni.physxtests import utils
from omni.physxcommands import AddGroundPlaneCommand, SetRigidBodyCommand
import omni.kit.test
from omni.physx import get_physx_cooking_interface
import carb
from pxr.UsdGeom import Cylinder
from omni.physx.scripts import physicsUtils
import numpy as np
from omni.physxtests.utils.physicsBase import TestCategory

class PhysxAttachmentInterfaceTetFinderTestAsync(PhysicsBaseAsyncTestCase):
    category = TestCategory.Kit
    @classmethod
    def setUpClass(self):
        # init for attributes that are not stage-dependent:
        self._prim_type_list = ['Cone', 'Cube', 'Cylinder', 'Sphere', 'Torus']

    # runs before each test case
    async def setUp(self):
        await super().setUp()
        self._baseWasSetup = False

    # runs after each test case and runs base_terminate if setup was called:
    async def tearDown(self):
        if self._baseWasSetup:
            await self.base_terminate()
        await super().tearDown()  

    async def base_setup(self):
        self._baseWasSetup = True
        self._stage = await utils.new_stage_setup()
        sphereLight = UsdLux.SphereLight.Define(self._stage, Sdf.Path("/SphereLight"))
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

        self._upAxis = UsdGeom.GetStageUpAxis(self._stage)
        self._defaultPrimPath = self._stage.GetDefaultPrim().GetPath()

        # add physics scene
        omni.kit.commands.execute("AddPhysicsScene", stage=self._stage, path='/World/PhysicsScene')

        # create a material (make a bit squishier for clearer deformation results)
        self._deformable_body_material_path = '/World/DeformableBodyMaterial'
        omni.kit.commands.execute("AddDeformableBodyMaterial",
                                  stage=self._stage, path=self._deformable_body_material_path,
                                  youngsModulus=5000.0)

        self._physxattachment = omni.physx.acquire_physx_attachment_private_interface()
        self._rest_coll_points = []
        self._coll_indices = []

    async def base_terminate(self):
        pass

    def add_groundplane(self):
        AddGroundPlaneCommand.execute(self._stage, '/CollisionPlane',
                                      self._upAxis, 1000.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    async def _runAddDeformableBodyComponentCommand(self, skin_mesh_path: Sdf.Path = Sdf.Path(), collision_mesh_path: Sdf.Path = Sdf.Path(), simulation_mesh_path: Sdf.Path=Sdf.Path()) -> bool:
        # make path for softbody
        self.assertTrue(bool(skin_mesh_path))

        # create softbody:
        success = omni.kit.commands.execute(
            "AddDeformableBodyComponent",
            skin_mesh_path=skin_mesh_path,
            collision_mesh_path=collision_mesh_path,
            simulation_mesh_path=simulation_mesh_path)

        # set deformable body material
        physicsUtils.add_physics_material_to_prim(self._stage, self._stage.GetPrimAtPath(skin_mesh_path), self._deformable_body_material_path)

        # this is a workaround for hang in logger while async cooking, would be nice to get at the bottom of this
        # tests that check for the cooked meshes should still call cook_deformable_body_mesh separately
        get_physx_cooking_interface().cook_deformable_body_mesh(str(skin_mesh_path))
        return success

    @staticmethod
    def set_prim_translation(prim: Usd.Prim, translateVec: Gf.Vec3d):
        translate_mtx = Gf.Matrix4d().SetTranslate(translateVec)
        omni.kit.commands.execute("TransformPrim", path=prim.GetPath(), new_transform_matrix=translate_mtx)

    def _create_mesh_prims(self, prim_type_list: list) -> list:
        mesh_list = []
        for prim_type in prim_type_list:
            path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/" + prim_type, True))
            omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type=prim_type)
            mesh = UsdGeom.Mesh.Get(self._stage, path)
            self.assertTrue(mesh)
            mesh_list.append(mesh)
        return mesh_list

    def set_cloth(self, mesh):
        omni.kit.commands.execute("AddParticleClothComponent", prim_path=mesh.GetPrim().GetPath())
        # check cloth API got applied
        cloth = PhysxSchema.PhysxParticleClothAPI.Get(self._stage, mesh.GetPath())
        self.assertTrue(cloth)
        return cloth

    def point_indices_check(self, point_handler, points):
        points = [self._rest_coll_points[0], self._rest_coll_points[10], self._rest_coll_points[50], Gf.Vec3d(0.0, 0.0, 0.0)]
        output = self._physxattachment.points_to_indices(point_handler, points)
        indices = output["indices"]
        self.assertTrue(indices[0] == 0)
        self.assertTrue(indices[1] == 10)
        self.assertTrue(indices[2] == 50)
        self.assertTrue(indices[3] == -1)

    async def _create_mesh_primitives(self, prim_type_list, starting_height=60.0):
        mesh_list = self._create_mesh_prims(prim_type_list)

        height = starting_height
        offset = 150
        origin = Gf.Vec3d(-offset * 3 / 2, height, -offset * 3 / 2)
        for i in range(3):
            for j in range(3):
                index = i * 3 + j
                if index < len(mesh_list):
                    self.set_prim_translation(mesh_list[index].GetPrim(), origin + Gf.Vec3d(i * offset, 0, j * offset))

        return mesh_list

    async def create_deformable_body(self, mesh, starting_height):
        translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, starting_height, 0))
        omni.kit.commands.execute("TransformPrim", path=mesh.GetPath(), new_transform_matrix=translate_mtx)
        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh.GetPath())
        soft_body_translation = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertTrue(soft_body_translation[1] == starting_height)

        prim = self._stage.GetPrimAtPath(mesh.GetPath())
        pb = UsdGeom.PointBased(prim)
        self.assertTrue(pb)

        deformable_api = PhysxSchema.PhysxDeformableBodyAPI(prim)
        self.assertTrue(deformable_api)

        self._rest_coll_points = list(deformable_api.GetCollisionRestPointsAttr().Get())
        self._coll_indices = list(deformable_api.GetCollisionIndicesAttr().Get())

    async def test_tet_finder(self):
        await self.base_setup()
        self.add_groundplane()
        starting_height = 170.0
        prim_type_list = ['Cone']
        cone_mesh, = await self._create_mesh_primitives(prim_type_list, starting_height=starting_height)

        await self.create_deformable_body(cone_mesh, starting_height)

        tet_finder_handler = self._physxattachment.create_tet_finder(self._rest_coll_points, self._coll_indices)
        self.assertTrue(tet_finder_handler >= 0)

        points_outside = [carb.Float3(0.0, 100, 0.0), carb.Float3(0.0, 120, 100.0), carb.Float3(200.0, 120, 200.0)]  
        output = self._physxattachment.points_to_tetmesh_local(tet_finder_handler, points_outside)
        tet_ids = output["tet_ids"]
        for i in range(len(tet_ids)):
            # Points in points_outside should all get return value -1
            self.assertTrue(tet_ids[i] == -1)

        points_inside = [self._rest_coll_points[10], self._rest_coll_points[50], self._rest_coll_points[100]]
        output = self._physxattachment.points_to_tetmesh_local(tet_finder_handler, points_inside)
        tet_ids = output["tet_ids"]
        for i in range(len(tet_ids)):
            # Points in points_inside should all get return value >= 0
            self.assertTrue(tet_ids[i] >= 0)

        input_tet_ids = [0, 10, 50]
        barycentric = [[0.1, 0.1, 0.1, 0.7], [0.1, 0.2, 0.2, 0.5], [0.2, 0.2, 0.2, 0.4]]
        output = self._physxattachment.tetmesh_local_to_points(tet_finder_handler, input_tet_ids, barycentric)
        points = output["points"]
        output = self._physxattachment.points_to_tetmesh_local(tet_finder_handler, points)
        output_tet_ids = output["tet_ids"]
        for i in range(len(points)):
            # point = points[i]
            # print("(%f, %f, %f) -> tetId: %d" % (point[0], point[1], point[2], output_tet_ids[i]))
            self.assertTrue(output_tet_ids[i] == input_tet_ids[i])

        self._physxattachment.release_tet_finder(tet_finder_handler)

    async def test_overlap_tetmesh(self):
        await self.base_setup()
        self.add_groundplane()
        starting_height = 170.0
        meshes = await self._create_mesh_primitives(self._prim_type_list, starting_height=starting_height)

        for mesh in meshes:
            translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, starting_height, 0))
            omni.kit.commands.execute("TransformPrim", path=mesh.GetPath(), new_transform_matrix=translate_mtx)
            await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh.GetPath())
            soft_body_translation = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
            self.assertTrue(soft_body_translation[1] == starting_height)

            await self.create_deformable_body(mesh, starting_height)

            tet_finder_handler = self._physxattachment.create_tet_finder(self._rest_coll_points, self._coll_indices)
            self.assertTrue(tet_finder_handler >= 0)

            # Does NOT overlap
            center = Gf.Vec3d(0.0, -100.0, 0.0)
            radius = 10.0
            output = self._physxattachment.overlap_tetmesh_sphere(tet_finder_handler, center, radius)
            tet_ids = output["tet_ids"]
            for id in tet_ids:
                self.assertTrue(id == -1)

            # Overlap
            center = Gf.Vec3d(0.0, 0.0, 0.0)
            radius = 50.0
            output = self._physxattachment.overlap_tetmesh_sphere(tet_finder_handler, center, radius)
            tet_ids = output["tet_ids"]
            for id in tet_ids:
                self.assertTrue(id >= 0)

            self._physxattachment.release_tet_finder(tet_finder_handler)

    async def test_point_finder(self):
        await self.base_setup()
        self.add_groundplane()
        starting_height = 170.0
        prim_type_list = ['Cylinder']
        cylinder_mesh, = await self._create_mesh_primitives(prim_type_list, starting_height=starting_height)

        translate_mtx_cylinder = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, starting_height, 50))
        omni.kit.commands.execute("TransformPrim", path=cylinder_mesh.GetPath(), new_transform_matrix=translate_mtx_cylinder)

        # Test deformables
        await self.create_deformable_body(cylinder_mesh, starting_height)

        point_handler_deformable = self._physxattachment.create_point_finder(self._rest_coll_points)
        self.assertTrue(point_handler_deformable >= 0)

        self.point_indices_check(point_handler_deformable, self._rest_coll_points)

        self._physxattachment.release_point_finder(point_handler_deformable)

        # Test cloth
        prim_type_list = ['Sphere']
        sphere_mesh, = await self._create_mesh_primitives(prim_type_list, starting_height=starting_height)

        translate_mtx_sphere = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, starting_height, -50))
        omni.kit.commands.execute("TransformPrim", path=cylinder_mesh.GetPath(), new_transform_matrix=translate_mtx_sphere)

        point_handler_cloth = self._physxattachment.create_point_finder(self._rest_coll_points)
        self.assertTrue(point_handler_cloth >= 0)

        self.set_cloth(mesh=sphere_mesh)
        particle_cloth_translation = sphere_mesh.ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        ).ExtractTranslation()
        self.assertTrue(particle_cloth_translation[1] == starting_height)

        self.point_indices_check(point_handler_cloth, self._rest_coll_points)

        self._physxattachment.release_point_finder(point_handler_cloth)

    async def test_get_closest_points(self):
        await self.base_setup()
        self.add_groundplane()
        starting_height = 200.0
        mesh_list = await self._create_mesh_primitives(self._prim_type_list, starting_height=starting_height)
        points = [carb.Float3(-225, 200.0, -225)]
        for mesh in mesh_list:
            mesh_path = mesh.GetPath()
            SetRigidBodyCommand.execute(mesh_path, "convexHull", False)
            output = self._physxattachment.get_closest_points(points, str(mesh_path))
            dists = output["dists"]
            closest_points = output["closest_points"]
            if dists[0] != 0.0:   
                closest_point = closest_points[0]  
                print("Closest point (%f, %f, %f) found." % (closest_point[0], closest_point[1], closest_point[2])) 
            else:
                print("Point is inside prim %s, closest point will not be calculated." % mesh_path)
