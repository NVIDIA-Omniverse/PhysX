# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.test
import omni.kit.commands
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema, Sdf
import omni.physx
from omni.physxcommands import AddGroundPlaneCommand
from omni.physx.scripts import physicsUtils, particleUtils, deformableUtils
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
from omni.physxtests import utils
import omni.usd
import unittest
import carb
import os
import omni.physx.bindings._physx as pb


class PhysxParticleAPITestKitStage(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Core
    
    # runs before each test case
    async def setUp(self):
        await super().setUp()
        await self.base_setup()

    # runs after each test case and runs base_terminate if setup was called:
    async def tearDown(self):
        for setting, value in self._settings_restore_data.items():
            self._settings.set(setting, value)
        await super().tearDown()

    async def base_setup(self):
        # save all settings for later restore:
        # no need to save/restore non persistent settings as they are reset with a new stage
        settings_to_restore = [
            pb.SETTING_DISPLAY_PARTICLES,
        ]
        self._settings = carb.settings.get_settings()
        self._settings_restore_data = {}
        for setting in settings_to_restore:
            self._settings_restore_data[setting] = self._settings.get(setting)

        # hide all as default setting for tests
        self._settings.set(pb.SETTING_DISPLAY_PARTICLES, pb.VisualizerMode.NONE)
        # set to 1 for this test
        self._settings.set(pb.SETTING_UPDATE_TO_USD, True)

        self.fail_on_log_error = True
        await self.new_stage()  # adds self._stage member
        default_prim_xform = UsdGeom.Xform.Define(self._stage, "/World")
        self._stage.SetDefaultPrim(default_prim_xform.GetPrim())

        self._up_axis = UsdGeom.GetStageUpAxis(self._stage)
        self._default_prim_path = self._stage.GetDefaultPrim().GetPath()
        physicsScenePath = self._default_prim_path.AppendChild("physicsScene")
        scene = UsdPhysics.Scene.Define(self._stage, physicsScenePath)
        physxAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxAPI.CreateSolverTypeAttr("TGS")

        self._pbd_material_path = self._default_prim_path.AppendChild("PBDMaterial")
        particleUtils.add_pbd_particle_material(self._stage, self._pbd_material_path)
        self._pbd_material_api = PhysxSchema.PhysxPBDMaterialAPI.Get(self._stage, self._pbd_material_path)

        self._particle_system_path = self._default_prim_path.AppendChild("particleSystem")
        particleUtils.add_physx_particle_system(
            stage=self._stage,
            particle_system_path=self._particle_system_path,
            simulation_owner=scene.GetPath(),
        )
        self._particle_system = PhysxSchema.PhysxParticleSystem.Get(self._stage, self._particle_system_path)

        physicsUtils.add_physics_material_to_prim(self._stage, self._particle_system.GetPrim(), self._pbd_material_path)

    def add_groundplane(self):
        AddGroundPlaneCommand.execute(
            self._stage, "/CollisionPlane", self._up_axis, 1000.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5)
        )

    async def _create_mesh_primitives(self, prim_type_list, starting_height=60.0):
        mesh_list = []
        for prim_type in prim_type_list:
            path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/" + prim_type, True))
            omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type=prim_type)
            mesh = UsdGeom.Mesh.Get(self._stage, path)
            self.assertTrue(mesh)
            mesh_list.append(mesh)

        height = starting_height
        offset = 150
        origin = Gf.Vec3d(-offset * 3 / 2, height, -offset * 3 / 2)
        for i in range(3):
            for j in range(3):
                index = i * 3 + j
                if index < len(mesh_list):
                    translate_mtx = Gf.Matrix4d().SetTranslate(origin + Gf.Vec3d(i * offset, 0, j * offset))
                    omni.kit.commands.execute(
                        "TransformPrim", path=mesh_list[index].GetPrim().GetPath(), new_transform_matrix=translate_mtx
                    )

        return mesh_list



    # particle set tests

    def create_particle_set(
        self,
        path=Sdf.Path(),
        positions_list=None,
        velocities_list=None,
        widths_list=None,
        particle_group=0,
        fluid=True,
        self_collision=True,
    ):

        geom_points = particleUtils.add_physx_particleset_points(
            self._stage,
            path,
            positions_list,
            velocities_list,
            widths_list,
            self._particle_system_path,
            self_collision,
            fluid,
            particle_group,
            1.0,
            0.02,
        )

        pointsPrim = self._stage.GetPrimAtPath(path)
        points = UsdGeom.Points(pointsPrim)
        self.assertTrue(points)

        pointPositions = points.GetPointsAttr().Get()
        self.assertTrue(pointPositions)
        self.assertEqual(len(pointPositions), len(positions_list))

        return geom_points

    def create_particle_set_pointInstancer(
        self,
        path=Sdf.Path,
        positions=None,
        velocities=None,
        particle_group=0,
        fluid=True,
        self_collision=True,
        prototype=Sdf.Path(),
    ):

        # create pointInstancer
        instancer = UsdGeom.PointInstancer.Define(self._stage, path)

        # create prototypes
        prototype_base_path = path.pathString + "/particlePrototype"
        mesh_list = instancer.GetPrototypesRel()
        if prototype:
            mesh_list.AddTarget(prototype)
        else:
            UsdGeom.Sphere.Define(self._stage, Sdf.Path(prototype_base_path))
            # add mesh references to point instancer
            mesh_list.AddTarget(Sdf.Path(prototype_base_path))

        # set particle instance data
        prototype_indices = [0] * len(positions)
        proto_indices = [x for x in prototype_indices]

        instancer.GetProtoIndicesAttr().Set(proto_indices)
        instancer.GetPositionsAttr().Set(positions)
        instancer.GetVelocitiesAttr().Set(velocities)

        # configure the particle set
        particleApi = PhysxSchema.PhysxParticleSetAPI.Apply(instancer.GetPrim())
        PhysxSchema.PhysxParticleAPI(particleApi).CreateSelfCollisionAttr().Set(self_collision)
        particleApi.CreateFluidAttr().Set(fluid)
        PhysxSchema.PhysxParticleAPI(particleApi).CreateParticleGroupAttr().Set(particle_group)
        PhysxSchema.PhysxParticleAPI(particleApi).CreateParticleSystemRel().SetTargets([self._particle_system_path])

        pointsPrim = self._stage.GetPrimAtPath(path)
        points = UsdGeom.PointInstancer(pointsPrim)
        self.assertTrue(points)

        pointPositions = points.GetPositionsAttr().Get()
        self.assertTrue(pointPositions)
        self.assertEqual(len(pointPositions), len(positions))

        return instancer

    async def test_particleSet_enable_disable(self):
        particlePointsPath = Sdf.Path("/particles")

        # create 1 particle
        positions_list = []
        velocities_list = []
        widths_list = []

        positions_list.append(Gf.Vec3f(1.0, 1.0, 1.0))
        velocities_list.append(Gf.Vec3f(0.0, 0.0, 0.0))
        widths_list.append(0.5)

        particleSet = self.create_particle_set(particlePointsPath, positions_list, velocities_list, widths_list)

        pointsPrim = particleSet.GetPrim()
        points = UsdGeom.Points(pointsPrim)
        pointPositions = points.GetPointsAttr().Get()
        init_point0 = pointPositions[0]

        particleAPI = PhysxSchema.PhysxParticleAPI(pointsPrim)

        # disable particle set
        particleAPI.CreateParticleEnabledAttr().Set(False)

        # step and check that it does not move
        await self.step(num_steps=5)
        pointPositions = points.GetPointsAttr().Get()
        current_point0 = pointPositions[0]
        self.assertEqual(current_point0[1], init_point0[1])

        # enable particle set
        particleAPI.CreateParticleEnabledAttr().Set(True)

        # should fall with gravity.
        await self.step(num_steps=5)
        pointPositions = points.GetPointsAttr().Get()
        current_point0 = pointPositions[0]
        self.assertLess(current_point0[1], init_point0[1])

    # add two solid particles - should have solidRestDistance
    # toggle to fluid, distance should be less
    # toggle to solids again, should separate again.
    async def test_particleSet_toggle_fluid(self):
        # set the solidRestOffset to a fixed value to trick the autocomputation
        solidRestOffset = 7.0
        self._particle_system.CreateSolidRestOffsetAttr().Set(solidRestOffset)

        self.add_groundplane()

        positions_list = []
        velocities_list = []
        widths_list = []

        positions_list.append(Gf.Vec3f(1.0, 10.0, 1.0))
        positions_list.append(Gf.Vec3f(1.0, solidRestOffset * 4.0, 1.0))
        velocities_list = [Gf.Vec3f(0.0, 0.0, 0.0)] * len(positions_list)
        widths_list = [0.5] * len(positions_list)

        particleSet = self.create_particle_set(
            Sdf.Path("/particleSet1"),
            positions_list,
            velocities_list,
            widths_list,
            particle_group=0,
            fluid=False,
            self_collision=True,
        )

        pointsPrim = particleSet.GetPrim()
        points = UsdGeom.Points(pointsPrim)

        await self.step(10)
        pointPositions = points.GetPointsAttr().Get()

        # check the distance, should be larger than solidRestOffset
        diffy = abs(pointPositions[0][1] - pointPositions[1][1])
        self.assertGreater(diffy, solidRestOffset)

        # toggle to fluid - should get closer
        particleAPI = PhysxSchema.PhysxParticleSetAPI(pointsPrim)
        particleAPI.CreateFluidAttr().Set(True)

        await self.step(10)
        pointPositions = points.GetPointsAttr().Get()

        # check the distance, should be less than solidRestOffset
        diffy = abs(pointPositions[0][1] - pointPositions[1][1])
        self.assertLess(diffy, solidRestOffset)

        # toggle to solid again
        particleAPI.CreateFluidAttr().Set(False)
        await self.step(10)
        pointPositions = points.GetPointsAttr().Get()

        # check the distance, should be greater than solidRestOffset
        diffy = abs(pointPositions[0][1] - pointPositions[1][1])
        self.assertGreater(diffy, solidRestOffset)

    async def test_particleSet_add_remove_api(self):
        particlePointsPath = Sdf.Path("/particles")

        # create 1 particle
        positions_list = []
        velocities_list = []
        widths_list = []

        positions_list.append(Gf.Vec3f(1.0, 1.0, 1.0))
        velocities_list.append(Gf.Vec3f(0.0, 0.0, 0.0))
        widths_list.append(0.5)

        particleSet = self.create_particle_set(particlePointsPath, positions_list, velocities_list, widths_list)

        pointsPrim = particleSet.GetPrim()
        points = UsdGeom.Points(pointsPrim)
        pointPositions = points.GetPointsAttr().Get()
        init_point0 = pointPositions[0]

        PhysxSchema.PhysxParticleAPI(pointsPrim)
        await self.step(num_steps=1)

        # should fall with gravity
        pointPositions = points.GetPointsAttr().Get()
        current_point0 = pointPositions[0]
        self.assertLess(current_point0[1], init_point0[1])

        # remove particle api
        pointsPrim.RemoveAPI(PhysxSchema.PhysxParticleSetAPI)

        await self.step(5)
        pointPositions = points.GetPointsAttr().Get()
        current_point1 = pointPositions[0]
        self.assertEqual(current_point1[1], current_point0[1])

        # add particle api
        PhysxSchema.PhysxParticleSetAPI.Apply(pointsPrim)

        await self.step(5)
        pointPositions = points.GetPointsAttr().Get()
        current_point2 = pointPositions[0]
        self.assertLess(current_point2[1], current_point1[1])

    # particle system tests

    # checks the functionality of the global enable/disable flag
    async def test_particleSystem_enable_disable(self):
        particlePointsPath = Sdf.Path("/particles")

        # create 1 particle
        positions_list = []
        velocities_list = []
        widths_list = []

        positions_list.append(Gf.Vec3f(1.0, 1.0, 1.0))
        velocities_list.append(Gf.Vec3f(0.0, 0.0, 0.0))
        widths_list.append(0.5)

        particleSet = self.create_particle_set(particlePointsPath, positions_list, velocities_list, widths_list)

        pointsPrim = particleSet.GetPrim()
        points = UsdGeom.Points(pointsPrim)
        pointPositions = points.GetPointsAttr().Get()
        init_point0 = pointPositions[0]

        self._particle_system.CreateParticleSystemEnabledAttr().Set(False)

        # step and check that it does not move
        await self.step(num_steps=5)
        pointPositions = points.GetPointsAttr().Get()
        current_point0 = pointPositions[0]
        self.assertEqual(current_point0[1], init_point0[1])

        self._particle_system.CreateParticleSystemEnabledAttr().Set(True)

        # should fall with gravity.
        await self.step(num_steps=5)
        pointPositions = points.GetPointsAttr().Get()
        current_point0 = pointPositions[0]
        self.assertLess(current_point0[1], init_point0[1])

    # adds a ground plane and tests if the particle is staying on top of it
    async def test_particleSystem_collision_with_collider(self):
        self.add_groundplane()

        particlePointsPath = Sdf.Path("/particles")

        # create 1 particle
        positions_list = []
        velocities_list = []
        widths_list = []

        positions_list.append(Gf.Vec3f(1.0, 10.0, 1.0))
        velocities_list.append(Gf.Vec3f(0.0, 0.0, 0.0))
        widths_list.append(0.5)

        particleSet = self.create_particle_set(particlePointsPath, positions_list, velocities_list, widths_list)

        pointsPrim = particleSet.GetPrim()
        points = UsdGeom.Points(pointsPrim)

        for i in range(0, 20):
            await self.step(1)
            pointPositions = points.GetPointsAttr().Get()
            self.assertGreater(pointPositions[0][1], 0.0)

    # adds two particle sets with different collision groups with 1 particle above each other
    # let them fall with gravity - should be more than solidRestOffset apart.
    async def test_particleSystem_particle_collisions(self):
        # set the solidRestOffset to a fixed value to trick the autocomputation
        solidRestOffset = 7.0
        self._particle_system.CreateSolidRestOffsetAttr().Set(solidRestOffset)

        self.add_groundplane()

        positions_list1 = []
        positions_list2 = []
        velocities_list = []
        widths_list = []

        # because we only have gravity and they are exactly on top of each other,
        # the two particles should end up in a "stacked" state
        positions_list1.append(Gf.Vec3f(1.0, 10.0, 1.0))
        positions_list2.append(Gf.Vec3f(1.0, solidRestOffset * 4.0, 1.0))
        velocities_list.append(Gf.Vec3f(0.0, 0.0, 0.0))
        widths_list.append(0.5)

        particleSet1 = self.create_particle_set(
            Sdf.Path("/particleSet1"), positions_list1, velocities_list, widths_list, particle_group=0, fluid=False
        )
        particleSet2 = self.create_particle_set(
            Sdf.Path("/particleSet2"), positions_list2, velocities_list, widths_list, particle_group=1, fluid=False
        )

        pointsPrim1 = particleSet1.GetPrim()
        points1 = UsdGeom.Points(pointsPrim1)

        pointsPrim2 = particleSet2.GetPrim()
        points2 = UsdGeom.Points(pointsPrim2)

        await self.step(10)
        pointPositions1 = points1.GetPointsAttr().Get()
        pointPositions2 = points2.GetPointsAttr().Get()

        # check the distance, should be larger than solidRestOffset
        diff = abs(pointPositions1[0][1] - pointPositions2[0][1])
        # dist = Gf.Sqrt(diff[0] ** 2 + diff[1] ** 2 + diff[2] ** 2)

        self.assertGreater(diff, solidRestOffset)

    # adds 2 particles on top of each other with the same phase, initially enable self-collision.
    # they should stack on top of each other. Then disable self collision, the stack should collapse
    async def test_particleSystem_self_collisions(self):
        # set the solidRestOffset to a fixed value to trick the autocomputation
        solidRestOffset = 7.0
        self._particle_system.CreateSolidRestOffsetAttr().Set(solidRestOffset)

        self.add_groundplane()

        positions_list = []
        velocities_list = []
        widths_list = []

        positions_list.append(Gf.Vec3f(1.0, 10.0, 1.0))
        positions_list.append(Gf.Vec3f(1.0, solidRestOffset * 4.0, 1.0))
        velocities_list = [Gf.Vec3f(0.0, 0.0, 0.0)] * len(positions_list)
        widths_list = [0.5] * len(positions_list)

        particleSet = self.create_particle_set(
            Sdf.Path("/particleSet1"),
            positions_list,
            velocities_list,
            widths_list,
            particle_group=0,
            fluid=False,
            self_collision=True,
        )

        pointsPrim = particleSet.GetPrim()
        points = UsdGeom.Points(pointsPrim)

        await self.step(20)
        pointPositions = points.GetPointsAttr().Get()

        # check the distance, should be larger than solidRestOffset
        diffy = abs(pointPositions[0][1] - pointPositions[1][1])
        self.assertGreater(diffy, solidRestOffset)

        # disable self collision - they should collapse.
        particleAPI = PhysxSchema.PhysxParticleAPI(pointsPrim)
        particleAPI.CreateSelfCollisionAttr().Set(False)

        await self.step(20)
        pointPositions = points.GetPointsAttr().Get()

        # check the distance, should be almost 0
        diffy = abs(pointPositions[0][1] - pointPositions[1][1])
        self.assertLess(diffy, solidRestOffset)

    # adds two particle sets with the same collision groups with 1 particle above each other
    # let them fall with gravity - should be more than solidRestOffset apart.
    # then disable the self collision flags on the sets and check if they move closer
    async def test_particleSystem_particle_self_collisions_two_sets(self):
        # set the solidRestOffset to a fixed value to trick the autocomputation
        solidRestOffset = 7.0
        self._particle_system.CreateSolidRestOffsetAttr().Set(solidRestOffset)

        self.add_groundplane()

        positions_list1 = []
        positions_list2 = []
        velocities_list = []
        widths_list = []

        # because we only have gravity and they are exactly on top of each other,
        # the two particles should end up in a "stacked" state
        positions_list1.append(Gf.Vec3f(1.0, 10.0, 1.0))
        positions_list2.append(Gf.Vec3f(1.0, solidRestOffset * 4.0, 1.0))
        velocities_list.append(Gf.Vec3f(0.0, 0.0, 0.0))
        widths_list.append(0.5)

        particleSet1 = self.create_particle_set(
            Sdf.Path("/particleSet1"), positions_list1, velocities_list, widths_list, particle_group=0, fluid=False
        )
        particleSet2 = self.create_particle_set(
            Sdf.Path("/particleSet2"), positions_list2, velocities_list, widths_list, particle_group=0, fluid=False
        )

        pointsPrim1 = particleSet1.GetPrim()
        points1 = UsdGeom.Points(pointsPrim1)

        pointsPrim2 = particleSet2.GetPrim()
        points2 = UsdGeom.Points(pointsPrim2)

        await self.step(10)
        pointPositions1 = points1.GetPointsAttr().Get()
        pointPositions2 = points2.GetPointsAttr().Get()

        # check the distance, should be larger than solidRestOffset
        diffy = abs(pointPositions1[0][1] - pointPositions2[0][1])
        self.assertGreater(diffy, solidRestOffset)

        # disable self-collsisions on top particle, should still collide
        particleSetApi1 = PhysxSchema.PhysxParticleSetAPI(particleSet1)
        PhysxSchema.PhysxParticleAPI(particleSetApi1).CreateSelfCollisionAttr().Set(False)
        await self.step(3)

        pointPositions1 = points1.GetPointsAttr().Get()
        pointPositions2 = points2.GetPointsAttr().Get()

        # check the distance, should still be larger than solidRestOffset
        diffy = abs(pointPositions1[0][1] - pointPositions2[0][1])
        self.assertGreater(diffy, solidRestOffset)

        # disable self-collsisions on lower particle, should not collide anymore
        particleSetApi2 = PhysxSchema.PhysxParticleSetAPI(particleSet2)
        PhysxSchema.PhysxParticleAPI(particleSetApi2).CreateSelfCollisionAttr().Set(False)
        await self.step(10)

        pointPositions1 = points1.GetPointsAttr().Get()
        pointPositions2 = points2.GetPointsAttr().Get()

        # check the distance, should be smaller than solidRestOffset
        diffy = abs(pointPositions1[0][1] - pointPositions2[0][1])
        self.assertLess(diffy, solidRestOffset)

    # setup a particle system with two particles in the same group, and disabled self collision
    # the global self collision flag should force them apart, then disable and see them collapse
    # no matter what self collision flag is set on the particle set itself.
    @unittest.skip("OM-46536")
    async def test_particleSystem_global_self_collision_flag(self):
        # setup PS, disable self collision on sets, but global self collision is on.
        # set the solidRestOffset to a fixed value to trick the autocomputation
        solidRestOffset = 7.0
        self._particle_system.CreateSolidRestOffsetAttr().Set(solidRestOffset)

        self.add_groundplane()

        positions_list = []
        velocities_list = []
        widths_list = []

        positions_list.append(Gf.Vec3f(1.0, 10.0, 1.0))
        positions_list.append(Gf.Vec3f(1.0, solidRestOffset * 3.0, 1.0))
        velocities_list = [Gf.Vec3f(0.0, 0.0, 0.0)] * len(positions_list)
        widths_list = [0.5] * len(positions_list)

        particleSet = self.create_particle_set(
            Sdf.Path("/particleSet1"),
            positions_list,
            velocities_list,
            widths_list,
            particle_group=0,
            fluid=False,
            self_collision=True,
        )

        pointsPrim = particleSet.GetPrim()
        points = UsdGeom.Points(pointsPrim)

        await self.step(20)
        pointPositions = points.GetPointsAttr().Get()

        # check the distance, should be larger than solidRestOffset
        diffy = abs(pointPositions[0][1] - pointPositions[1][1])
        self.assertGreater(diffy, solidRestOffset)

        # globally disable self collision - they should collapse.
        self._particle_system.CreateGlobalSelfCollisionEnabledAttr().Set(False)

        await self.step(20)
        pointPositions = points.GetPointsAttr().Get()

        # check the distance, should be almost 0
        diffy = abs(pointPositions[0][1] - pointPositions[1][1])
        self.assertLess(diffy, solidRestOffset)

    # setup a particle system with 1 particle, let it fall to the ground.
    # should stay there, then disable particle-rigid collisions
    # it should fall below the ground plane.
    @unittest.skip("OM-46536")
    async def test_particleSystem_non_particle_collision_flag(self):
        self.add_groundplane()

        particlePointsPath = Sdf.Path("/particles")

        # create 1 particle
        positions_list = []
        velocities_list = []
        widths_list = []

        positions_list.append(Gf.Vec3f(1.0, 10.0, 1.0))
        velocities_list.append(Gf.Vec3f(0.0, 0.0, 0.0))
        widths_list.append(0.5)

        particleSet = self.create_particle_set(particlePointsPath, positions_list, velocities_list, widths_list)

        pointsPrim = particleSet.GetPrim()
        points = UsdGeom.Points(pointsPrim)

        for i in range(0, 20):
            await self.step(1)
            pointPositions = points.GetPointsAttr().Get()
            self.assertGreater(pointPositions[0][1], 0.0)

        self._particle_system.CreateNonParticleCollisionEnabledAttr().Set(False)

        await self.step(5)
        pointPositions = points.GetPointsAttr().Get()
        self.AssertLess(pointPositions[0][1], 0.0)

    @unittest.skip("OM-46541")
    async def test_particleSystem_ccd(self):
        pass

    # add a set of particles - run sim - add another set - run sim - remove first set
    # triggers resizing - test if the newly added particle interacts with first set
    # and test if it falls to the ground plane after removing the first set.
    async def test_particleSystem_add_remove_sets_runtime(self):
        self.add_groundplane()

        # set the solidRestOffset to a fixed value to trick the autocomputation
        solidRestOffset = 7.0
        self._particle_system.CreateSolidRestOffsetAttr().Set(solidRestOffset)

        particlePointsPath1 = Sdf.Path("/particles1")

        # create 1 particle
        positions_list = []
        velocities_list = []
        widths_list = []

        positions_list.append(Gf.Vec3f(1.0, 10.0, 1.0))
        velocities_list.append(Gf.Vec3f(0.0, 0.0, 0.0))
        widths_list.append(0.5)

        particleSet = self.create_particle_set(
            particlePointsPath1, positions_list, velocities_list, widths_list, fluid=False
        )

        pointsPrim = particleSet.GetPrim()
        points = UsdGeom.Points(pointsPrim)

        await self.step(20)
        pointPositions = points.GetPointsAttr().Get()

        # initial check - particle should be above ground plane
        self.assertGreater(pointPositions[0][1], 0.0)

        # add another particle
        particlePointsPath2 = Sdf.Path("/particles2")
        positions_list = []
        velocities_list = []
        widths_list = []

        positions_list.append(Gf.Vec3f(1.0, 25.0, 1.0))
        velocities_list.append(Gf.Vec3f(0.0, 0.0, 0.0))
        widths_list.append(0.5)

        particleSet2 = self.create_particle_set(
            particlePointsPath2, positions_list, velocities_list, widths_list, fluid=False
        )

        pointsPrim2 = particleSet2.GetPrim()
        points2 = UsdGeom.Points(pointsPrim2)

        await self.step(20)
        pointPositions2 = points2.GetPointsAttr().Get()
        pointPositions = points.GetPointsAttr().Get()

        # new particle should be on top of first particle
        self.assertGreater(pointPositions2[0][1], pointPositions[0][1])
        # check the distance, should be larger than solidRestOffset
        diffy = abs(pointPositions2[0][1] - pointPositions[0][1])
        self.assertGreater(diffy, solidRestOffset)

        # save for later
        oldPosy = pointPositions2[0][1]

        # remove the first set.
        self._stage.RemovePrim(particlePointsPath1)

        await self.step(20)
        pointPositions2 = points2.GetPointsAttr().Get()

        # should fall down, but stay above ground plane.
        self.assertGreater(oldPosy, pointPositions2[0][1])
        self.assertGreater(pointPositions2[0][1], 0)

    # OM-36091
    async def test_particles_save_restore(self):
        filename = "particles.usda"

        particlePointsPath = Sdf.Path("/particles")

        # create 1 particle
        positions_list = []
        velocities_list = []
        widths_list = []

        positions_list.append(Gf.Vec3f(1.0, 1.0, 1.0))
        velocities_list.append(Gf.Vec3f(0.0, 0.0, 0.0))
        widths_list.append(0.5)

        particleSet = self.create_particle_set(particlePointsPath, positions_list, velocities_list, widths_list)

        pointsPrim = particleSet.GetPrim()
        points = UsdGeom.Points(pointsPrim)
        pointPositions = points.GetPointsAttr().Get()
        init_point0 = pointPositions[0]

        # run for a few moments
        await self.step(num_steps=5)

        # get positions
        pointPositions = points.GetPointsAttr().Get()
        current_point0 = pointPositions[0]
        self.assertNotEqual(current_point0[1], init_point0[1])

        # save
        self._stage.Export(filename)

        # run some more
        await self.step(num_steps=5)

        # check that it ran
        pointPositions = points.GetPointsAttr().Get()
        current_point1 = pointPositions[0]
        self.assertNotEqual(current_point1[1], current_point0[1])

        # load the saved file
        await omni.usd.get_context().open_stage_async(filename)
        new_stage = omni.usd.get_context().get_stage()

        points = UsdGeom.Points.Get(new_stage, particlePointsPath)
        pointPositions = points.GetPointsAttr().Get()
        restored_point0 = pointPositions[0]
        self.assertNotEqual(restored_point0[1], current_point1[1])

        # check if the points are in same state as saved
        self.assertEqual(restored_point0, current_point0)

        # run
        await self.step(num_steps=5)

        # check if they fall some more
        pointPositions = points.GetPointsAttr().Get()
        restored_point1 = pointPositions[0]
        self.assertNotEqual(restored_point0[1], restored_point1[1])

        # run + stop
        await self.step(num_steps=5, stop_timeline_after=True)

        # check if stop resets into saved state
        pointPositions = points.GetPointsAttr().Get()
        stopped_point0 = pointPositions[0]
        self.assertEqual(restored_point0, stopped_point0)

        # cleanup the file save
        os.remove(filename)

    async def _run_position_velocity_update_usd_test(self, updateToUsdSetting, updateVelToUsdSetting):
        # setup settings (backup restore is in setup/teardown)
        self._settings = carb.settings.get_settings()
        self._settings.set(pb.SETTING_UPDATE_TO_USD, updateToUsdSetting)
        self._settings.set(pb.SETTING_UPDATE_PARTICLES_TO_USD, updateToUsdSetting)
        self._settings.set(pb.SETTING_UPDATE_VELOCITIES_TO_USD, updateVelToUsdSetting)

        # setup a single particle to sim and check its pos/vel change:
        particlePointsPath = Sdf.Path("/particles")

        # create 1 particle
        positions_list = []
        velocities_list = []
        widths_list = []

        startPos = Gf.Vec3f(1.0, 1.0, 1.0)
        startVel = Gf.Vec3f(0.0, 0.0, 0.0)
        positions_list.append(startPos)
        velocities_list.append(startVel)
        widths_list.append(0.5)

        particleSet = self.create_particle_set(particlePointsPath, positions_list, velocities_list, widths_list)

        # step
        await self.step(num_steps=1)

        pointsPrim = particleSet.GetPrim()
        points = UsdGeom.Points(pointsPrim)
        positions = points.GetPointsAttr().Get()
        vels = points.GetVelocitiesAttr().Get()

        if not updateToUsdSetting:
            self.assertEqual(positions[0], startPos)
            self.assertEqual(vels[0], startVel)

        if updateToUsdSetting:
            self.assertLess(positions[0][1], startPos[1])
            if not updateVelToUsdSetting:
                self.assertEqual(vels[0], startVel)
            else:
                self.assertLess(vels[0][1], 0)

    # OM-47210 - test update to USD settings
    async def test_particles_update_to_usd_settings(self):
        for updateToUsd in [False, True]:
            for updateVelToUsd in [False, True]:
                # use subtest context to get better failure info and run all permutations even if any one fails
                with self.subTest(updateToUsd=updateToUsd, updateVelToUsd=updateVelToUsd):
                    await self._run_position_velocity_update_usd_test(
                        updateToUsdSetting=updateToUsd, updateVelToUsdSetting=updateVelToUsd
                    )
                # need to cycle teardown and setup to start fresh, regardless of subtest success/failure
                await self.tearDown()
                await self.setUp()


    # OM-47658
    async def test_particles_non_sphere(self):
        # create a particle set with cubes
        cubePath = Sdf.Path("/cube")
        cubePrim = UsdGeom.Cube.Define(self._stage, cubePath)

        # 1 particle
        positions_list = []
        velocities_list = []

        positions_list.append(Gf.Vec3f(1.0, 1.0, 1.0))
        velocities_list.append(Gf.Vec3f(0.0, 0.0, 0.0))

        instancerPath = Sdf.Path("/particles")

        instancer = self.create_particle_set_pointInstancer(
            path = instancerPath,
            positions = positions_list,
            velocities = velocities_list,
            prototype = cubePath
        )

        # save init point
        pointPositions = instancer.GetPositionsAttr().Get()
        init_point0 = pointPositions[0]

        # run
        await self.step(num_steps=5)

        # check that the particle moved
        pointPositions = instancer.GetPositionsAttr().Get()
        current_point0 = pointPositions[0]
        self.assertLess(current_point0[1], init_point0[1])

    # test pair filter by letting particle fall through ground plane
    async def test_particleSystem_pair_filter(self):
        self.add_groundplane()

        particlePointsPath = Sdf.Path("/particles")

        # create 1 particle
        positions_list = [Gf.Vec3f(1.0, 10.0, 1.0)]
        velocities_list = [Gf.Vec3f(0.0, 0.0, 0.0)]
        widths_list = [0.5]

        particleSet = self.create_particle_set(particlePointsPath, positions_list, velocities_list, widths_list)

        pointsPrim = particleSet.GetPrim()
        points = UsdGeom.Points(pointsPrim)

        for i in range(0, 20):
            await self.step(1)
            pointPositions = points.GetPointsAttr().Get()
            self.assertGreater(pointPositions[0][1], 0.0)

        await self.step(num_steps=1, stop_timeline_after=True)

        #create filter
        filter_api = UsdPhysics.FilteredPairsAPI.Apply(self._particle_system.GetPrim())
        plane_path = self._default_prim_path.AppendChild("CollisionPlane").AppendChild("CollisionPlane")
        print(plane_path)
        filter_api.GetFilteredPairsRel().AddTarget(Sdf.Path(plane_path))

        for i in range(0, 20):
            await self.step(1)

        pointPositions = points.GetPointsAttr().Get()
        self.assertGreater(0.0, pointPositions[0][1])

    # OM-90086 - crash because of double free of particle set when deleting in paused state
    async def test_particles_delete_set_and_system(self):
        self.add_groundplane()

        # create 1 particle
        positions_list = [Gf.Vec3f(1.0, 10.0, 1.0)]
        velocities_list = [Gf.Vec3f(0.0, 0.0, 0.0)]
        widths_list = [0.5]

        particlePointsPath = Sdf.Path("/particles")
        particleSet = self.create_particle_set(particlePointsPath, positions_list, velocities_list, widths_list)

        await self.step(num_steps=1, stop_timeline_after=False)

        #remove particle system and particle set
        self._stage.RemovePrim(self._particle_system_path)
        self._stage.RemovePrim(particlePointsPath)

    async def test_physics_particle_set_inconsistent_update(self):
        await self.base_setup()

        positions_list = [Gf.Vec3f(1.0, 10.0, 1.0)]
        velocities_list = [Gf.Vec3f(0.0, 0.0, 0.0)]
        widths_list = [0.5]

        particlePointsPath = Sdf.Path("/particles")
        geom_points = self.create_particle_set(particlePointsPath, positions_list, velocities_list, widths_list)

        await self.step(num_steps=1)

        #just add to points, should also add to velocities and widths
        positions_list.extend([Gf.Vec3f(1.0, 11.0, 1.0)])
        geom_points.GetPointsAttr().Set(positions_list)

        await self.step(num_steps=1)
        velocities = geom_points.GetVelocitiesAttr().Get()
        widths = geom_points.GetWidthsAttr().Get()

        #velocties are expected to be written
        self.assertEqual(2, len(velocities))
        self.assertEqual(1, len(widths))

        #just add to velocities, should also add to points and widths
        velocities_list.extend([Gf.Vec3f(0.0, 0.0, 0.0), Gf.Vec3f(0.0, 0.0, 0.0)])
        geom_points.GetVelocitiesAttr().Set(velocities_list)

        await self.step(num_steps=1)
        points = geom_points.GetPointsAttr().Get()
        widths = geom_points.GetWidthsAttr().Get()

        #points are expected to be written
        self.assertEqual(3, len(points))
        self.assertEqual(1, len(widths))

        await self.step(num_steps=1, stop_timeline_after=True)

    #regression test for catching more invalid conditions during parsing for OM-110476
    async def helper_particle_set_inconsistent_setup(self, use_instancer):
        await self.base_setup()
        positions_list = [Gf.Vec3f(1.0, 10.0, 1.0), Gf.Vec3f(1.0, 10.0, 1.0)]
        velocities_list = [Gf.Vec3f(0.0, 0.0, 0.0), Gf.Vec3f(0.0, 0.0, 0.0)]
        widths_list = [0.5, 0.5]
        particle_path = Sdf.Path("/particles")
        prim = None
        particle_set = None
        if use_instancer:
            prim = self.create_particle_set_pointInstancer(path = particle_path, positions = positions_list, velocities = velocities_list).GetPrim()
        else:
            prim = self.create_particle_set(particle_path, positions_list, velocities_list, widths_list).GetPrim()
        particle_set = PhysxSchema.PhysxParticleSetAPI(prim)
        return prim, particle_set

    async def test_particle_set_inconsistent_setup(self):

        message = f"UsdGeomPointInstancer:velocities and UsdGeomPointInstancer:positions have inconsistent sizes."
        prim, particle_set = await self.helper_particle_set_inconsistent_setup(True)
        with utils.ExpectMessage(self, message, expected_result=True):
            UsdGeom.PointInstancer(prim).GetVelocitiesAttr().Set([Gf.Vec3f(0,0,0)])
            await self.step(num_steps=1, stop_timeline_after=True)

        message = f"UsdGeomPointBased:velocities and UsdGeomPointBased:points have inconsistent sizes."
        prim, particle_set = await self.helper_particle_set_inconsistent_setup(False)
        with utils.ExpectMessage(self, message, expected_result=True):
            UsdGeom.PointBased(prim).GetVelocitiesAttr().Set([Gf.Vec3f(0,0,0)])
            await self.step(num_steps=1, stop_timeline_after=True)

        message = f"PhysxSchemaPhysxParticleSetAPI:simulationPoints and UsdGeomPointInstancer:positions have inconsistent sizes."
        prim, particle_set = await self.helper_particle_set_inconsistent_setup(True)
        with utils.ExpectMessage(self, message, expected_result=True):
            particle_set.GetSimulationPointsAttr().Set([Gf.Vec3f(0,0,0)])
            await self.step(num_steps=1, stop_timeline_after=True)

        message = f"PhysxSchemaPhysxParticleSetAPI:simulationPoints and UsdGeomPointBased:points have inconsistent sizes."
        prim, particle_set = await self.helper_particle_set_inconsistent_setup(False)
        with utils.ExpectMessage(self, message, expected_result=True):
            particle_set.GetSimulationPointsAttr().Set([Gf.Vec3f(0,0,0)])
            await self.step(num_steps=1, stop_timeline_after=True)

        message = f"PhysxSchemaPhysxParticleSetAPI:maxParticles is lower than UsdGeomPointInstancer:positions size."
        prim, particle_set = await self.helper_particle_set_inconsistent_setup(True)
        with utils.ExpectMessage(self, message, expected_result=True):
            prim.CreateAttribute("physxParticle:maxParticles", Sdf.ValueTypeNames.Int).Set(1)
            await self.step(num_steps=1, stop_timeline_after=True)

        message = f"PhysxSchemaPhysxParticleSetAPI:maxParticles is lower than UsdGeomPointBased:points size."
        prim, particle_set = await self.helper_particle_set_inconsistent_setup(False)
        with utils.ExpectMessage(self, message, expected_result=True):
            prim.CreateAttribute("physxParticle:maxParticles", Sdf.ValueTypeNames.Int).Set(1)
            await self.step(num_steps=1, stop_timeline_after=True)
