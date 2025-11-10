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


class PhysxParticleAnisotropyAPITestKitStage(PhysicsKitStageAsyncTestCase):
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

    async def test_particlePostProcessing_anisotropy(self):
        # create set with two particles not intersecting
        particlePath = Sdf.Path("/particles")
        particleSpacing = 7.0
        particlePointsPrim = self.create_particle_set_pointinstancer(
            path=particlePath,
            initial_point=Gf.Vec3f(0.0, 50.0, 0.0),
            particle_spacing=particleSpacing,
            num_x=3,
            num_y=1,
            num_z=1,
        )

        # add anisotropy
        PhysxSchema.PhysxParticleAnisotropyAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check the scales, x should be different from default for middle particle
        # without aniso, it looks like this
        # o o o
        # with aniso the particle in the middle is stretched like this
        # o<=>o
        instancer = UsdGeom.PointInstancer(particlePointsPrim.GetPrim())
        scales = instancer.GetScalesAttr().Get()

        changedScale = scales[1][0]
        self.assertNotEqual(scales[0][0], changedScale)
        self.assertNotEqual(scales[2][0], changedScale)
        self.assertAlmostEqual(scales[0][0], scales[2][0])

        # play
        await self.step(2)

        scales = instancer.GetScalesAttr().Get()
        self.assertLess(scales[1][0], changedScale)

    async def run_particlePostProcessing_anisotropy_enable_disable(self, use_api=False):
        # create set with three particles not intersecting
        particlePath = Sdf.Path("/particles")
        particleSpacing = 7.0
        particlePointsPrim = self.create_particle_set_pointinstancer(
            path=particlePath,
            initial_point=Gf.Vec3f(0.0, 50.0, 0.0),
            particle_spacing=particleSpacing,
            num_x=3,
            num_y=1,
            num_z=1,
        )

        # add anisotropy
        anisotropyAPI = PhysxSchema.PhysxParticleAnisotropyAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check the scales, x should be different from default for middle particle
        # without aniso, it looks like this
        # o o o
        # with aniso the particle in the middle is stretched like this
        # o<=>o
        instancer = UsdGeom.PointInstancer(particlePointsPrim.GetPrim())
        scales1 = instancer.GetScalesAttr().Get()

        changedScale = scales1[1][0]
        self.assertNotEqual(scales1[0][0], changedScale)
        self.assertNotEqual(scales1[2][0], changedScale)
        self.assertAlmostEqual(scales1[0][0], scales1[2][0])

        # disable
        if use_api:
            self._particle_system.GetPrim().RemoveAPI(PhysxSchema.PhysxParticleAnisotropyAPI)
        else:
            anisotropyAPI.CreateParticleAnisotropyEnabledAttr().Set(False)
        await omni.kit.app.get_app().next_update_async()

        # make sure scales are reset.
        scales = instancer.GetScalesAttr().Get()
        self.assertEqual(scales[0], Gf.Vec3f(1))
        self.assertEqual(scales[1], Gf.Vec3f(1))
        self.assertEqual(scales[2], Gf.Vec3f(1))

        # enable
        if use_api:
            anisotropyAPI = PhysxSchema.PhysxParticleAnisotropyAPI.Apply(self._particle_system.GetPrim())
        else:
            anisotropyAPI.CreateParticleAnisotropyEnabledAttr().Set(True)
        await omni.kit.app.get_app().next_update_async()

        scales = instancer.GetScalesAttr().Get()
        # test if scales are same as with last enable
        self.assertAlmostEqual(scales1, scales)

        # play
        await self.step(2)

        # disable
        if use_api:
            self._particle_system.GetPrim().RemoveAPI(PhysxSchema.PhysxParticleAnisotropyAPI)
        else:
            anisotropyAPI.CreateParticleAnisotropyEnabledAttr().Set(False)
        await omni.kit.app.get_app().next_update_async()

        # make sure scales are reset.
        scales = instancer.GetScalesAttr().Get()
        self.assertEqual(scales[0], Gf.Vec3f(1))
        self.assertEqual(scales[1], Gf.Vec3f(1))
        self.assertEqual(scales[2], Gf.Vec3f(1))

        # play
        await self.step(2)

        # enable
        if use_api:
            anisotropyAPI = PhysxSchema.PhysxParticleAnisotropyAPI.Apply(self._particle_system.GetPrim())
            self.assertTrue(anisotropyAPI)
        else:
            anisotropyAPI.CreateParticleAnisotropyEnabledAttr().Set(True)
        await omni.kit.app.get_app().next_update_async()

        # play
        await self.step(2)

        # test middle scale should still be different
        scales = instancer.GetScalesAttr().Get()
        changedScale = scales[1][0]
        self.assertNotEqual(scales[0][0], changedScale)
        self.assertNotEqual(scales[2][0], changedScale)
        self.assertAlmostEqual(scales[0][0], scales[2][0])

    async def test_particlePostProcessing_anisotropy_enable_disable(self):
        await self.run_particlePostProcessing_anisotropy_enable_disable(use_api=False)

    async def test_particlePostProcessing_anisotropy_add_remove_api(self):
        await self.run_particlePostProcessing_anisotropy_enable_disable(use_api=True)

    async def test_particlePostProcessing_anisotropy_fluid_toggle(self):

        # setup
        #
        #   o    1 set with 1 particles
        # o o o  1 set with 3 particles
        #   o    1 set with 1 particle
        #
        # with anisotropy, looks like
        #   o
        # o O o
        #   o
        #
        # toggle the upper particle to solid, looks like
        #
        #   o
        # o<=>o   # middle particle has larger scale for x axis.
        #   o

        # create set with three particles not intersecting
        particlePath = Sdf.Path("/particles1")
        particleSpacing = 7.0
        particlePointsPrimMiddle = self.create_particle_set_pointinstancer(
            path=particlePath,
            initial_point=Gf.Vec3f(0.0, 50.0, 0.0),
            particle_spacing=particleSpacing,
            num_x=3,
            num_y=1,
            num_z=1,
        )

        # create another set with a single particle
        particlePath = Sdf.Path("/particles2")
        particleSpacing = 7.0
        particlePointsPrimTop = self.create_particle_set_pointinstancer(
            path=particlePath,
            initial_point=Gf.Vec3f(7.0, 57.0, 0.0),
            particle_spacing=particleSpacing,
            num_x=1,
            num_y=1,
            num_z=1,
        )

        # create another set with a single particle
        particlePath = Sdf.Path("/particles3")
        particleSpacing = 7.0
        particlePointsPrimBottom = self.create_particle_set_pointinstancer(
            path=particlePath,
            initial_point=Gf.Vec3f(7.0, 43.0, 0.0),
            particle_spacing=particleSpacing,
            num_x=1,
            num_y=1,
            num_z=1,
        )

        # add anisotropy
        PhysxSchema.PhysxParticleAnisotropyAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        instancerMiddle = UsdGeom.PointInstancer(particlePointsPrimMiddle)
        instancerTop = UsdGeom.PointInstancer(particlePointsPrimTop)
        instancerBottom = UsdGeom.PointInstancer(particlePointsPrimBottom)
        initialScalesMiddle = instancerMiddle.GetScalesAttr().Get()
        initialScalesTop = instancerTop.GetScalesAttr().Get()
        initialScalesBottom = instancerBottom.GetScalesAttr().Get()

        # middle particle should be scaled larger than 1 in x + y directions
        self.assertGreater(initialScalesMiddle[1][0], 1.0)
        self.assertGreater(initialScalesMiddle[1][1], 1.0)

        middleY1 = initialScalesMiddle[1][1]

        # toggle top particle to solid
        particleApiTop = PhysxSchema.PhysxParticleSetAPI(particlePointsPrimTop)
        particleApiTop.CreateFluidAttr().Set(False)
        await omni.kit.app.get_app().next_update_async()

        # top instancer should have scales reset
        scalesTop = instancerTop.GetScalesAttr().Get()
        self.assertEqual(scalesTop[0], Gf.Vec3f(1.0))

        # middle instancer should have smaller scale in y dim
        scalesMiddle = instancerMiddle.GetScalesAttr().Get()
        middleY2 = scalesMiddle[1][1]
        self.assertLess(middleY2, middleY1)

        # toggle bottom particle to solid
        particleApiBottom = PhysxSchema.PhysxParticleSetAPI(particlePointsPrimBottom)
        particleApiBottom.CreateFluidAttr().Set(False)
        await omni.kit.app.get_app().next_update_async()

        # bottom instancer should have scales reset
        scalesBottom = instancerBottom.GetScalesAttr().Get()
        self.assertEqual(scalesBottom[0], Gf.Vec3f(1.0))

        # middle instancer should have smaller scale in y dim
        scalesMiddle = instancerMiddle.GetScalesAttr().Get()
        middleY3 = scalesMiddle[1][1]
        self.assertLess(middleY3, middleY2)
        # y scaling should not be affected at all anymore, so will be same as neighboring particles
        self.assertEqual(scalesMiddle[1][1], scalesMiddle[0][1])

        # toggle middles particles to solid
        particleApiMiddle = PhysxSchema.PhysxParticleSetAPI(particlePointsPrimMiddle)
        particleApiMiddle.CreateFluidAttr().Set(False)
        await omni.kit.app.get_app().next_update_async()

        # all should be reset
        scalesMiddle = instancerMiddle.GetScalesAttr().Get()
        self.assertEqual(scalesMiddle[0], Gf.Vec3f(1))
        self.assertEqual(scalesMiddle[1], Gf.Vec3f(1))
        self.assertEqual(scalesMiddle[2], Gf.Vec3f(1))

        # toggle all of them back to fluid
        particleApiMiddle.CreateFluidAttr().Set(True)
        particleApiTop.CreateFluidAttr().Set(True)
        particleApiBottom.CreateFluidAttr().Set(True)
        await omni.kit.app.get_app().next_update_async()

        scalesMiddle = instancerMiddle.GetScalesAttr().Get()
        scalesBottom = instancerBottom.GetScalesAttr().Get()
        scalesTop = instancerTop.GetScalesAttr().Get()

        # scales should be same as initial setup
        self.assertAlmostEqual(scalesMiddle, initialScalesMiddle)
        self.assertAlmostEqual(scalesTop, initialScalesTop)
        self.assertAlmostEqual(scalesBottom, initialScalesBottom)

    async def test_particlePostProcessing_anisotropy_scale(self):
        # create set with three particles not intersecting
        particlePath = Sdf.Path("/particles")
        particleSpacing = 7.0
        particlePointsPrim = self.create_particle_set_pointinstancer(
            path=particlePath,
            initial_point=Gf.Vec3f(0.0, 50.0, 0.0),
            particle_spacing=particleSpacing,
            num_x=3,
            num_y=1,
            num_z=1,
        )

        # add anisotropy
        anisotropyAPI = PhysxSchema.PhysxParticleAnisotropyAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check the scales, x should be different from default for middle particle
        # without aniso, it looks like this
        # o o o
        # with aniso the particle in the middle is stretched like this
        # o<=>o
        instancer = UsdGeom.PointInstancer(particlePointsPrim)
        scales1 = instancer.GetScalesAttr().Get()

        changedScale = scales1[1][0]
        self.assertNotEqual(scales1[0][0], changedScale)
        self.assertNotEqual(scales1[2][0], changedScale)
        self.assertAlmostEqual(scales1[0][0], scales1[2][0])

        # increase anisotropy scale value
        initialScale = anisotropyAPI.GetScaleAttr().Get()
        anisotropyAPI.CreateScaleAttr().Set(initialScale + 1.0)
        await omni.kit.app.get_app().next_update_async()

        # scale for middle particle should be larger now
        scales2 = instancer.GetScalesAttr().Get()
        self.assertGreater(scales2[1][0], scales1[1][0])

        # decrease anisotropy scale value
        anisotropyAPI.CreateScaleAttr().Set(initialScale - 1.0)
        await omni.kit.app.get_app().next_update_async()

        # scale for middle particle should be smaller than both others
        scales3 = instancer.GetScalesAttr().Get()
        self.assertGreater(scales2[1][0], scales3[1][0])
        self.assertGreater(scales1[1][0], scales3[1][0])

        # set back to initial
        anisotropyAPI.CreateScaleAttr().Set(initialScale)
        await omni.kit.app.get_app().next_update_async()

        # should be same as initial
        scales4 = instancer.GetScalesAttr().Get()
        self.assertAlmostEqual(scales4[1][0], scales1[1][0])

        # increase min scale
        initialMin = anisotropyAPI.GetMinAttr().Get()
        anisotropyAPI.CreateMinAttr().Set(initialMin + 0.2)
        await omni.kit.app.get_app().next_update_async()

        # the values for left & right particle should now be larger
        scales5 = instancer.GetScalesAttr().Get()
        self.assertGreater(scales5[0][0], scales4[0][0])
        self.assertGreater(scales5[0][1], scales4[0][1])
        self.assertGreater(scales5[0][2], scales4[0][2])
        self.assertGreater(scales5[2][0], scales4[2][0])
        self.assertGreater(scales5[2][1], scales4[2][1])
        self.assertGreater(scales5[2][2], scales4[2][2])

        # reset and decrease max
        anisotropyAPI.CreateMinAttr().Set(initialMin)
        anisotropyAPI.CreateMaxAttr().Set(0.4)
        await omni.kit.app.get_app().next_update_async()

        # x of middle particle should now be smaller
        scales6 = instancer.GetScalesAttr().Get()
        self.assertLess(scales6[1][0], scales4[1][0])

    # OM-36091
    async def test_particlePostProcessing_anisotropy_save_restore(self):
        filename = "particles.usda"

        # create set with three particles not intersecting
        particlePath = Sdf.Path("/particles")
        particleSpacing = 7.0
        particlePointsPrim = self.create_particle_set_pointinstancer(
            path=particlePath,
            initial_point=Gf.Vec3f(0.0, 50.0, 0.0),
            particle_spacing=particleSpacing,
            num_x=3,
            num_y=1,
            num_z=1,
        )

        # add anisotropy
        PhysxSchema.PhysxParticleAnisotropyAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check the scales, x should be different from default for middle particle
        # without aniso, it looks like this
        # o o o
        # with aniso the particle in the middle is stretched like this
        # o<=>o
        instancer = UsdGeom.PointInstancer(particlePointsPrim)
        scales = instancer.GetScalesAttr().Get()
        points = instancer.GetPositionsAttr().Get()

        changedScale = scales[1][0]
        self.assertNotEqual(scales[0][0], changedScale)
        self.assertNotEqual(scales[2][0], changedScale)
        self.assertAlmostEqual(scales[0][0], scales[2][0])

        # run for a few moments
        await self.step(1)

        # check if particles moved
        points1 = instancer.GetPositionsAttr().Get()
        self.assertLess(points1[0][1], points[0][1])
        self.assertLess(points1[1][1], points[1][1])
        self.assertLess(points1[2][1], points[2][1])

        scales1 = instancer.GetScalesAttr().Get()

        # save
        self._stage.Export(filename)

        # check that save does not affect points
        points2 = instancer.GetPositionsAttr().Get()
        self.assertEqual(points1, points2)
        scales2 = instancer.GetScalesAttr().Get()
        self.assertEqual(scales1, scales2)

        # run some more
        await self.step(1)

        # check if they nove
        points3 = instancer.GetPositionsAttr().Get()
        self.assertLess(points3[0][1], points2[0][1])
        self.assertLess(points3[1][1], points2[1][1])
        self.assertLess(points3[2][1], points2[2][1])

        # load the saved file
        await omni.usd.get_context().open_stage_async(filename)
        self._stage = omni.usd.get_context().get_stage()

        instancer = UsdGeom.PointInstancer(self._stage.GetPrimAtPath(particlePath))
        points4 = instancer.GetPositionsAttr().Get()
        self.assertEqual(points4, points1)
        scales4 = instancer.GetScalesAttr().Get()
        self.assertEqual(scales4, scales1)

        # run
        await self.step(1)

        # check if they moves
        points5 = instancer.GetPositionsAttr().Get()
        self.assertLess(points5[0][1], points4[0][1])
        self.assertLess(points5[1][1], points4[1][1])
        self.assertLess(points5[2][1], points4[2][1])

        # run + stop
        await self.step(num_steps=1, stop_timeline_after=True)

        points6 = instancer.GetPositionsAttr().Get()
        self.assertEqual(points6, points4)
        scales6 = instancer.GetScalesAttr().Get()
        self.assertEqual(scales6, scales4)

        # cleanup the file save
        os.remove(filename)
