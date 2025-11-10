# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import Gf, Sdf, Vt, UsdGeom, Usd, UsdPhysics, UsdLux, PhysxSchema
from omni.physxtests.utils.physicsBase import PhysicsBaseAsyncTestCase, TestCategory
from omni.physxtests import utils
from omni.physxcommands import AddGroundPlaneCommand, SetRigidBodyCommand
from omni.physx import get_physx_interface
import omni.kit.test
from omni.physx import get_physx_cooking_interface
import carb
from pxr.UsdGeom import Cylinder
from omni.physx.scripts import physicsUtils
import numpy as np
from omni.physxtests.utils.physicsBase import TestCategory

class PhysxAttachmentInterfacePoissonSamplingTestAsync(PhysicsBaseAsyncTestCase):
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

    async def _create_mesh_primitives(self, prim_type_list, starting_height=60.0):
        mesh_list = self._create_mesh_prims(prim_type_list)

        height = starting_height
        offset = 0
        origin = Gf.Vec3d(-offset * 3 / 2, height, -offset * 3 / 2)
        for i in range(3):
            for j in range(3):
                index = i * 3 + j
                if index < len(mesh_list):
                    self.set_prim_translation(mesh_list[index].GetPrim(), origin + Gf.Vec3d(i * offset, 0, j * offset))

        return mesh_list

    def add_groundplane(self):
        AddGroundPlaneCommand.execute(self._stage, '/CollisionPlane',
                                      self._upAxis, 1000.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    @staticmethod
    def _get_time_step():
        return 1.0 / 60.0

    def _start(self):
        physx_interface = get_physx_interface()
        physx_interface.start_simulation()

    def _step(self, numSteps):
        physx_interface = get_physx_interface()
        time = 0.0
        dtime = self._get_time_step()
        for i in range(numSteps):
            physx_interface.update_simulation(dtime, time)
            physx_interface.update_transformations(True, True, True, False)
            time = time + dtime

    def _reset(self):
        physx_interface = get_physx_interface()
        physx_interface.reset_simulation()

    async def test_poisson_sampling(self):
        await self.base_setup()
        starting_height = 0.0
        meshes = await self._create_mesh_primitives(self._prim_type_list, starting_height=starting_height)

        sampling_dist = 1.0
        for mesh in meshes:
            mesh_path = mesh.GetPath()
            sampling_dist = sampling_dist + 0.5
            SetRigidBodyCommand.execute(mesh_path, "", False)
            handler_0 = self._physxattachment.create_surface_sampler(str(mesh_path), sampling_dist)
            self.assertTrue(handler_0 >= 0)

            output = self._physxattachment.get_surface_sampler_points(handler_0)
            existing_points = output["points"]
            self.assertTrue(len(existing_points) == 0)

            center = Gf.Vec3d(0.0, 0.0, 0.0)
            radius = 100
            output = self._physxattachment.sample_surface(handler_0, center, radius, sampling_dist)
            existing_points = output["points"]
            num_existing_points = len(existing_points)
            print("# of samples is : %d" % num_existing_points)
            #self.assertTrue(num_existing_points > 0)         

            self._physxattachment.release_surface_sampler(handler_0)

            handler_1 = self._physxattachment.create_surface_sampler(str(mesh_path), sampling_dist)
            self.assertTrue(handler_1 >= 0)
            self._physxattachment.add_surface_sampler_points(handler_1, existing_points)
            output = self._physxattachment.get_surface_sampler_points(handler_1)
            current_points = output["points"]            
            self.assertTrue(num_existing_points == len(current_points))

            self._physxattachment.release_surface_sampler(handler_1)

    # OM-47990
    async def test_poisson_sampling_simulate_stop(self):
        await self.base_setup()
        self.add_groundplane()
        starting_height = 150.0
        prim_type_list = ['Cylinder']
        cylinder_mesh, = await self._create_mesh_primitives(prim_type_list, starting_height=starting_height)
        sampling_dist = 1.0
        mesh_path = cylinder_mesh.GetPath()
        SetRigidBodyCommand.execute(mesh_path, "", False)
        handler = self._physxattachment.create_surface_sampler(str(mesh_path), sampling_dist)
        self.assertTrue(handler >= 0)

        center = Gf.Vec3d(0.0, 150.0, 0.0)
        radius = 50
        output = self._physxattachment.sample_surface(handler, center, radius, sampling_dist)
        existing_points = output["points"]
        num_existing_points = len(existing_points)
        print("# of samples before simulation is : %d" % num_existing_points)
        #self.assertTrue(num_existing_points > 0)

        self._start()
        self._step(30)
        self._reset()

        self.assertTrue(handler >= 0)
        output = self._physxattachment.get_surface_sampler_points(handler)
        existing_points = output["points"]
        print("# of samples after reset is : %d" % len(existing_points))
        self.assertTrue(len(existing_points) == num_existing_points)

        if len(existing_points) > 0:
            target_points = [existing_points[0], existing_points[10], existing_points[20]]
            self._physxattachment.remove_surface_sampler_points(handler, target_points)
            output = self._physxattachment.get_surface_sampler_points(handler)
            points = output["points"]
            print("# of samples after removing is : %d" % len(points))
            self.assertTrue(len(points) == len(existing_points) - 3)

        self._physxattachment.release_surface_sampler(handler)

