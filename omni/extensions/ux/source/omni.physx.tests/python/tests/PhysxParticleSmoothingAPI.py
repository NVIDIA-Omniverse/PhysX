# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.test
import omni.kit.commands
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Usd
from omni.physxtests import utils
from omni.physxcommands import AddGroundPlaneCommand
from omni.physx.scripts import physicsUtils, particleUtils
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
import omni.usd
import carb
import os
import unittest
import omni.physx.bindings._physx as pb


class PhysxParticleSmoothingAPITestKitStage(PhysicsKitStageAsyncTestCase):
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

        self.fail_on_log_error = True
        # setup stage before changing settings
        self._stage = await utils.new_stage_setup()

        # save all settings for later restore
        # no need to save/restore non persistent settings as they are reset with a new stage
        settings_to_restore = [
            pb.SETTING_DISPLAY_PARTICLES,
        ]
        self._settings = carb.settings.get_settings()
        self._settings_restore_data = {}
        for setting in settings_to_restore:
            self._settings_restore_data[setting] = self._settings.get(setting)

        # hide all debug viz as default setting for tests
        self._settings.set(pb.SETTING_DISPLAY_PARTICLES, pb.VisualizerMode.NONE)
        # make sure updates to usd is enabled
        self._settings.set(pb.SETTING_UPDATE_TO_USD, True)

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

        self._scene = scene

        self.add_groundplane()

    def add_groundplane(self):
        AddGroundPlaneCommand.execute(
            self._stage, "/CollisionPlane", self._up_axis, 1000.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5)
        )

    def create_particle_set(
        self,
        path=Sdf.Path(),
        initial_point=Gf.Vec3f(0.0, 1.0, 0.0),
        num_x=20,
        num_y=20,
        num_z=20,
        particle_spacing=None,
    ):

        positions, velocities = particleUtils.create_particles_grid(
            initial_point, particle_spacing + 0.01, num_x, num_y, num_z
        )

        widths = [particle_spacing] * len(positions)
        particleSet = particleUtils.add_physx_particleset_points(
            self._stage, path, positions, velocities, widths, self._particle_system_path, True, True, 0, 1.0, 0.02
        )

        pointsPrim = self._stage.GetPrimAtPath(path)
        points = UsdGeom.Points(pointsPrim)
        self.assertTrue(points)

        pointPositions = points.GetPointsAttr().Get()
        self.assertTrue(pointPositions)
        self.assertEqual(len(pointPositions), len(positions))

        return particleSet

    def create_particle_set_pointinstancer(
        self,
        path=Sdf.Path(),
        initial_point=Gf.Vec3f(0.0, 1.0, 0.0),
        num_x=20,
        num_y=20,
        num_z=20,
        particle_spacing=None,
    ):

        positions, velocities = particleUtils.create_particles_grid(
            initial_point, particle_spacing + 0.01, num_x, num_y, num_z
        )

        particleSet = particleUtils.add_physx_particleset_pointinstancer(
            self._stage,
            path,
            positions,
            velocities,
            self._particle_system_path,
            True,
            True,
            0,
            1.0,
            0.02,
            1,
        )

        prototypeStr = str(path) + "/particlePrototype0"
        gprim = UsdGeom.Sphere.Define(self._stage, Sdf.Path(prototypeStr))
        gprim.CreateRadiusAttr().Set(particle_spacing * 0.5)

        instancerPrim = self._stage.GetPrimAtPath(path)
        points = UsdGeom.PointInstancer(instancerPrim)
        self.assertTrue(points)

        pointPositions = points.GetPositionsAttr().Get()
        self.assertTrue(pointPositions)
        self.assertEqual(len(pointPositions), len(positions))

        return particleSet

    def get_point_distance(self, point0=None, point1=None):
        diff = point0 - point1
        dist = Gf.Sqrt(diff[0] ** 2 + diff[1] ** 2 + diff[2] ** 2)
        return dist

    async def test_particlePostProcessing_smoothing(self):
        # create set with two particles not intersecting
        particlePath = Sdf.Path("/particles")
        particleSpacing = 5.0
        particlePointsPrim = self.create_particle_set_pointinstancer(
            path=particlePath,
            initial_point=Gf.Vec3f(0.0, 50.0, 0.0),
            particle_spacing=particleSpacing,
            num_x=2,
            num_y=1,
            num_z=1,
        )

        # get the initial distance
        instancer = UsdGeom.PointInstancer(particlePointsPrim)
        points = instancer.GetPositionsAttr().Get()
        dist = self.get_point_distance(points[0], points[1])

        # add smoothing
        PhysxSchema.PhysxParticleSmoothingAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check if they move closer together.
        points = instancer.GetPositionsAttr().Get()
        dist2 = self.get_point_distance(points[0], points[1])
        self.assertLess(dist2, dist)

    async def run_particlePostProcessing_smoothing_enable_disable(self, use_api=False):
        # create set with two particles not intersecting
        particlePath = Sdf.Path("/particles")
        particleSpacing = 5.0
        particlePointsPrim = self.create_particle_set_pointinstancer(
            path=particlePath,
            initial_point=Gf.Vec3f(0.0, 50.0, 0.0),
            particle_spacing=particleSpacing,
            num_x=2,
            num_y=1,
            num_z=1,
        )

        # get the initial distance
        instancer = UsdGeom.PointInstancer(particlePointsPrim)
        points = instancer.GetPositionsAttr().Get()
        dist = self.get_point_distance(points[0], points[1])

        # add smoothing
        smoothingAPI = PhysxSchema.PhysxParticleSmoothingAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check if they move closer together.
        points = instancer.GetPositionsAttr().Get()
        dist2 = self.get_point_distance(points[0], points[1])
        self.assertLess(dist2, dist)

        # disable
        if use_api:
            self._particle_system.GetPrim().RemoveAPI(PhysxSchema.PhysxParticleSmoothingAPI)
        else:
            smoothingAPI.CreateParticleSmoothingEnabledAttr().Set(False)
        await omni.kit.app.get_app().next_update_async()

        # check if distance is same as original
        points = instancer.GetPositionsAttr().Get()
        dist3 = self.get_point_distance(points[0], points[1])
        self.assertAlmostEqual(dist, dist3)
        self.assertGreater(dist3, dist2)

        # enable again
        if use_api:
            smoothingAPI = PhysxSchema.PhysxParticleSmoothingAPI.Apply(self._particle_system.GetPrim())
        else:
            smoothingAPI.CreateParticleSmoothingEnabledAttr().Set(True)
        await omni.kit.app.get_app().next_update_async()

        # check if they move together again
        points = instancer.GetPositionsAttr().Get()
        dist4 = self.get_point_distance(points[0], points[1])
        self.assertAlmostEqual(dist2, dist4)
        self.assertLess(dist4, dist3)
        p1 = points[0]

        # play
        await self.step(2)

        # check if the particles moved
        points = instancer.GetPositionsAttr().Get()
        self.assertNotEqual(p1, points[0])
        dist5 = self.get_point_distance(points[0], points[1])

        # disable
        if use_api:
            self._particle_system.GetPrim().RemoveAPI(PhysxSchema.PhysxParticleSmoothingAPI)
        else:
            smoothingAPI.CreateParticleSmoothingEnabledAttr().Set(False)
        await omni.kit.app.get_app().next_update_async()

        # check if distance is greater than before
        points = instancer.GetPositionsAttr().Get()
        dist6 = self.get_point_distance(points[0], points[1])
        self.assertGreater(dist6, dist5)

        # play
        await self.step(2)

        # enable
        if use_api:
            smoothingAPI = PhysxSchema.PhysxParticleSmoothingAPI.Apply(self._particle_system.GetPrim())
        else:
            smoothingAPI.CreateParticleSmoothingEnabledAttr().Set(True)
        await omni.kit.app.get_app().next_update_async()

        # check if distance is smaller than before
        points = instancer.GetPositionsAttr().Get()
        dist7 = self.get_point_distance(points[0], points[1])
        self.assertGreater(dist6, dist7)

    async def test_particlePostProcessing_smoothing_enable_disable(self):
        await self.run_particlePostProcessing_smoothing_enable_disable(use_api=False)

    async def test_particlePostProcessing_smoothing_add_remove_api(self):
        await self.run_particlePostProcessing_smoothing_enable_disable(use_api=True)

    async def test_particlePostProcessing_smoothing_fluid_toggle(self):
        # two particle sets, both initially fluid
        particlePath1 = Sdf.Path("/particles1")
        particleSpacing = 5.0
        particlePointsPrim1 = self.create_particle_set_pointinstancer(
            path=particlePath1,
            initial_point=Gf.Vec3f(0.0, 50.0, 0.0),
            num_x=1,
            num_y=1,
            num_z=1,
            particle_spacing=particleSpacing,
        )

        particlePath2 = Sdf.Path("/particles2")
        particleSpacing = 5.0
        particlePointsPrim2 = self.create_particle_set_pointinstancer(
            path=particlePath2,
            initial_point=Gf.Vec3f(5.0, 50.0, 0.0),
            num_x=1,
            num_y=1,
            num_z=1,
            particle_spacing=particleSpacing,
        )

        # get the initial distance
        instancer1 = UsdGeom.PointInstancer(particlePointsPrim1)
        points1 = instancer1.GetPositionsAttr().Get()
        instancer2 = UsdGeom.PointInstancer(particlePointsPrim2)
        points2 = instancer2.GetPositionsAttr().Get()
        dist = self.get_point_distance(points1[0], points2[0])

        # add smoothing
        PhysxSchema.PhysxParticleSmoothingAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check if they move closer together.
        points1 = instancer1.GetPositionsAttr().Get()
        points2 = instancer2.GetPositionsAttr().Get()
        dist2 = self.get_point_distance(points1[0], points2[0])
        self.assertLess(dist2, dist)

        # toggle fluid on set 2 - smoothing should not be done on either set.
        particleSetAPI = PhysxSchema.PhysxParticleSetAPI(particlePointsPrim2.GetPrim())
        particleSetAPI.CreateFluidAttr().Set(False)
        await omni.kit.app.get_app().next_update_async()

        # check if they are in the initial positions
        points1 = instancer1.GetPositionsAttr().Get()
        points2 = instancer2.GetPositionsAttr().Get()
        dist3 = self.get_point_distance(points1[0], points2[0])
        self.assertLess(dist2, dist3)

        self.assertAlmostEqual(dist3, dist)

        # toggle fluid on set 1 - nothing happens
        particleSetAPI1 = PhysxSchema.PhysxParticleSetAPI(particlePointsPrim1.GetPrim())
        particleSetAPI1.CreateFluidAttr().Set(False)
        await omni.kit.app.get_app().next_update_async()

        # check if distance is still the same.
        points1 = instancer1.GetPositionsAttr().Get()
        points2 = instancer2.GetPositionsAttr().Get()
        dist4 = self.get_point_distance(points1[0], points2[0])
        self.assertAlmostEqual(dist4, dist3)

        # toggle fluid on both sets again
        particleSetAPI.CreateFluidAttr().Set(True)
        particleSetAPI1.CreateFluidAttr().Set(True)
        await omni.kit.app.get_app().next_update_async()

        # check if they move closer together
        points1 = instancer1.GetPositionsAttr().Get()
        points2 = instancer2.GetPositionsAttr().Get()
        dist5 = self.get_point_distance(points1[0], points2[0])
        self.assertLess(dist5, dist4)
        p1 = points1[0]

        # simulate some steps
        await self.step(1)

        # check if the particles moved
        points1 = instancer1.GetPositionsAttr().Get()
        points2 = instancer2.GetPositionsAttr().Get()
        self.assertNotEqual(p1, points1[0])
        dist6 = self.get_point_distance(points1[0], points2[0])

        # toggle set 1 to solid again as well
        particleSetAPI1.CreateFluidAttr().Set(False)
        await omni.kit.app.get_app().next_update_async()

        # check if they move apart
        points1 = instancer1.GetPositionsAttr().Get()
        points2 = instancer2.GetPositionsAttr().Get()
        dist7 = self.get_point_distance(points1[0], points2[0])
        self.assertLess(dist6, dist7)

    async def test_particlePostProcessing_smoothing_strength(self):
        # create set with two particles not intersecting
        particlePath = Sdf.Path("/particles")
        particleSpacing = 5.0
        particlePointsPrim = self.create_particle_set_pointinstancer(
            path=particlePath,
            initial_point=Gf.Vec3f(0.0, 50.0, 0.0),
            particle_spacing=particleSpacing,
            num_x=2,
            num_y=1,
            num_z=1,
        )

        # get the initial distance
        instancer = UsdGeom.PointInstancer(particlePointsPrim)
        points = instancer.GetPositionsAttr().Get()
        dist = self.get_point_distance(points[0], points[1])

        # add smoothing
        smoothingAPI = PhysxSchema.PhysxParticleSmoothingAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check if they move closer together.
        points = instancer.GetPositionsAttr().Get()
        dist2 = self.get_point_distance(points[0], points[1])
        self.assertLess(dist2, dist)

        # reduce strength
        smoothingAPI.CreateStrengthAttr().Set(0.5)
        await omni.kit.app.get_app().next_update_async()

        # particles should move apart
        points = instancer.GetPositionsAttr().Get()
        dist3 = self.get_point_distance(points[0], points[1])
        self.assertGreater(dist3, dist2)

        # increase strength
        smoothingAPI.CreateStrengthAttr().Set(1.0)
        await omni.kit.app.get_app().next_update_async()

        # check if they move together again
        points = instancer.GetPositionsAttr().Get()
        dist4 = self.get_point_distance(points[0], points[1])
        self.assertLess(dist4, dist3)

        # reset to 0.5 - should be same as dist3
        smoothingAPI.CreateStrengthAttr().Set(0.5)
        await omni.kit.app.get_app().next_update_async()

        # check if distance is almost equal to dist3
        points = instancer.GetPositionsAttr().Get()
        dist5 = self.get_point_distance(points[0], points[1])
        self.assertAlmostEqual(dist5, dist3)

        # set to 0 - should be same as without smoothing
        smoothingAPI.CreateStrengthAttr().Set(0.0)
        await omni.kit.app.get_app().next_update_async()

        # check if distance is almost equal to dist3
        points = instancer.GetPositionsAttr().Get()
        dist6 = self.get_point_distance(points[0], points[1])
        self.assertAlmostEqual(dist6, dist)

    # OM-36091
    async def test_particlePostProcessing_smoothing_save_restore(self):
        filename = "particles.usda"

        # create set with two particles not intersecting
        particlePath = Sdf.Path("/particles")
        particleSpacing = 5.0
        particlePointsPrim = self.create_particle_set_pointinstancer(
            path=particlePath,
            initial_point=Gf.Vec3f(0.0, 50.0, 0.0),
            particle_spacing=particleSpacing,
            num_x=2,
            num_y=1,
            num_z=1,
        )

        # get the initial distance
        instancer = UsdGeom.PointInstancer(particlePointsPrim)
        points = instancer.GetPositionsAttr().Get()
        dist = self.get_point_distance(points[0], points[1])

        # add smoothing
        PhysxSchema.PhysxParticleSmoothingAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check if they move closer together.
        points = instancer.GetPositionsAttr().Get()
        dist2 = self.get_point_distance(points[0], points[1])
        self.assertLess(dist2, dist)

        # run for a few moments
        await self.step(1)

        # check if particles moved
        points1 = instancer.GetPositionsAttr().Get()
        self.assertLess(points1[0][1], points[0][1])
        self.assertLess(points1[1][1], points[1][1])

        # save
        self._stage.Export(filename)

        # check that save does not affect points
        points2 = instancer.GetPositionsAttr().Get()
        self.assertEqual(points1, points2)

        # run some more
        await self.step(1)

        # check if they nove
        points3 = instancer.GetPositionsAttr().Get()
        self.assertLess(points3[0][1], points2[0][1])
        self.assertLess(points3[1][1], points2[1][1])

        # load the saved file
        await omni.usd.get_context().open_stage_async(filename)
        self._stage = omni.usd.get_context().get_stage()

        instancer = UsdGeom.PointInstancer(self._stage.GetPrimAtPath(particlePath))
        points4 = instancer.GetPositionsAttr().Get()
        self.assertEqual(points4, points1)

        # run
        await self.step(1)

        # check if they moves
        points5 = instancer.GetPositionsAttr().Get()
        self.assertLess(points5[0][1], points4[0][1])
        self.assertLess(points5[1][1], points4[1][1])

        # run + stop
        await self.step(num_steps=1, stop_timeline_after=True)

        points6 = instancer.GetPositionsAttr().Get()
        self.assertEqual(points6, points4)

        # cleanup the file save
        os.remove(filename)

    # OM-47210 - test update to USD settings
    async def _run_smoothing_update_usd_test(self, enableSmoothing, updateToUsdSetting, updateParticlesToUsdSetting):
        # setup settings (backup restore is in setup/teardown)
        self._settings.set(pb.SETTING_UPDATE_TO_USD, updateToUsdSetting)
        # always enable, if updateToUSD is off, it overrides this and velocities are not updated
        self._settings.set(pb.SETTING_UPDATE_VELOCITIES_TO_USD, True)
        self._settings.set(pb.SETTING_UPDATE_PARTICLES_TO_USD, updateParticlesToUsdSetting)

        particlePath = Sdf.Path("/particles")
        particleSpacing = 5.0
        initial_pos = Gf.Vec3f(0.0, 50.0, 0.0)
        particle_prim = self.create_particle_set_pointinstancer(
            path=particlePath, initial_point=initial_pos, particle_spacing=particleSpacing, num_x=4, num_y=1, num_z=4
        )
        particle_instancer = UsdGeom.PointInstancer(particle_prim)
        position_attr = particle_instancer.GetPositionsAttr()
        vels_attr = particle_instancer.GetVelocitiesAttr()
        set_api = PhysxSchema.PhysxParticleSetAPI(particle_prim)
        sim_position_attr = set_api.GetSimulationPointsAttr()

        if enableSmoothing:
            PhysxSchema.PhysxParticleSmoothingAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        preSimPoint = position_attr.Get()[0]
        if enableSmoothing:
            # check that the first particle moved without sim, which means smoothing is on
            self.assertNotEqual(initial_pos, preSimPoint)
            preSimSimPoint = sim_position_attr.Get()[0]
            self.assertEqual(preSimSimPoint, initial_pos)
        await self.step(1)

        # if update to usd is off, nothing must move or be updated:
        if not updateToUsdSetting:
            # check that not moved and velocity not updated
            # not moved also tests that smoothing is not applied
            self.assertEqual(preSimPoint, position_attr.Get()[0])
            if enableSmoothing:
                self.assertEqual(preSimSimPoint, initial_pos)
            self.assertEqual(Gf.Vec3f(0.0), vels_attr.Get()[0])
            return

        # otherwise:
        # - if smoothing is on
        #   - points update with smoothed positions
        #   - sim points update with real sim positions iff updateParticlesToUsd is on
        if enableSmoothing:
            self.assertNotEqual(preSimPoint, position_attr.Get()[0])
            if updateParticlesToUsdSetting:
                self.assertNotEqual(sim_position_attr.Get()[0], preSimSimPoint)
            else:
                self.assertEqual(sim_position_attr.Get()[0], preSimSimPoint)

    # OM-47210 - test update to USD settings
    async def test_particlePostProcessing_smoothing_update_to_usd_settings(self):
        for updateToUsd in [False, True]:
            for enableSmoothing in [False, True]:
                # use subtest context to get better failure info and run all permutations even if any one fails
                with self.subTest(
                    updateToUsd=updateToUsd, updateParticlesToUsd=updateToUsd, enableSmoothing=enableSmoothing
                ):
                    await self._run_smoothing_update_usd_test(
                        enableSmoothing=enableSmoothing,
                        updateToUsdSetting=updateToUsd,
                        updateParticlesToUsdSetting=updateToUsd,
                    )
                # need to cycle teardown and setup to start fresh, regardless of subtest success/failure
                await self.tearDown()
                await self.setUp()
