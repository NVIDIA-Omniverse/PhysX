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


class PhysxParticleIsosurfaceAPITestKitStage(PhysicsKitStageAsyncTestCase):
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

    def get_extents(self, pointbased=None):
        pts = pointbased.GetPointsAttr().Get()
        extent = UsdGeom.PointBased.ComputeExtent(pts)
        transform = pointbased.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        for i in range(len(extent)):
            extent[i] = transform.Transform(extent[i])

        return extent

    def get_extents_instancer(self, instancer=None):
        pts = instancer.GetPositionsAttr().Get()
        extent = UsdGeom.PointBased.ComputeExtent(pts)
        transform = instancer.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        for i in range(len(extent)):
            extent[i] = transform.Transform(extent[i])

        return extent

    def compare_extents(self, extents0=None, extents1=None):
        diff = False
        diff = diff or abs(extents0[0][0] - extents1[0][0]) > 0.001
        diff = diff or abs(extents0[0][1] - extents1[0][1]) > 0.001
        diff = diff or abs(extents0[0][2] - extents1[0][2]) > 0.001

        diff = diff or abs(extents0[1][0] - extents1[1][0]) > 0.001
        diff = diff or abs(extents0[1][1] - extents1[1][1]) > 0.001
        diff = diff or abs(extents0[1][2] - extents1[1][2]) > 0.001

        return diff

    def check_extents_differ(self, extents0=None, extents1=None):
        different = self.compare_extents(extents0, extents1)
        self.assertTrue(different)

    def check_extents_same(self, extents0=None, extents1=None):
        different = self.compare_extents(extents0, extents1)
        self.assertFalse(different)

    # check if extents0 is contained within extents1 +/- surfaceDistance
    def check_extents_contained(self, extents0=None, extents1=None, surface_distance=None):
        self.assertGreaterEqual(extents0[0][0], extents1[0][0] - surface_distance)
        self.assertGreaterEqual(extents0[0][1], extents1[0][1] - surface_distance)
        self.assertGreaterEqual(extents0[0][2], extents1[0][2] - surface_distance)

        self.assertLessEqual(extents0[1][0], extents1[1][0] + surface_distance)
        self.assertLessEqual(extents0[1][1], extents1[1][1] + surface_distance)
        self.assertLessEqual(extents0[1][2], extents1[1][2] + surface_distance)

    def get_isosurface_path(self, particle_system_path=Sdf.Path()):
        if particle_system_path != Sdf.Path():
            return particle_system_path.AppendElementString("Isosurface")
        else:
            return self._particle_system_path.AppendElementString("Isosurface")

    def get_isosurface_extents(self, particle_system_path=Sdf.Path()):
        # get the isosurface mesh
        isoPath = self.get_isosurface_path(particle_system_path)
        isoPrim = self._stage.GetPrimAtPath(isoPath)
        self.assertTrue(isoPrim)
        isoMesh = UsdGeom.Mesh(isoPrim)
        self.assertTrue(isoMesh)

        # get the extents of the mesh
        return self.get_extents(isoMesh)

    def check_isosurface_exists(self, particle_paths=[], particle_system_path=Sdf.Path()):
        isoExtent = self.get_isosurface_extents(particle_system_path)

        # AABB max should be larger than min.
        self.assertGreater(isoExtent[1][0], isoExtent[0][0])
        self.assertGreater(isoExtent[1][1], isoExtent[0][1])
        self.assertGreater(isoExtent[1][2], isoExtent[0][2])

        # make sure it's visible
        isoImg = UsdGeom.Imageable(self._stage.GetPrimAtPath(self.get_isosurface_path()))
        self.assertTrue(self.is_visible(isoImg))

        if particle_paths != []:

            surfaceDistance = 5.0
            min = Gf.Vec3f(float("inf"), float("inf"), float("inf"))
            max = Gf.Vec3f(float("-inf"), float("-inf"), float("-inf"))

            for particleSetPath in particle_paths:
                # check if invisible
                particleImg = UsdGeom.Imageable(self._stage.GetPrimAtPath(particleSetPath))
                self.assertFalse(self.is_visible(particleImg))

                particleSetPrim = self._stage.GetPrimAtPath(particleSetPath)
                particleInstancer = UsdGeom.PointInstancer(particleSetPrim)
                particleExtent = self.get_extents_instancer(particleInstancer)

                min = Gf.Vec3f(
                    Gf.Min(min[0], particleExtent[0][0]),
                    Gf.Min(min[1], particleExtent[0][1]),
                    Gf.Min(min[2], particleExtent[0][2]),
                )
                max = Gf.Vec3f(
                    Gf.Max(max[0], particleExtent[1][0]),
                    Gf.Max(max[1], particleExtent[1][1]),
                    Gf.Max(max[2], particleExtent[1][2]),
                )

            # check if extents are contained
            self.check_extents_contained(extents0=isoExtent, extents1=(min, max), surface_distance=surfaceDistance)

        return isoExtent

    def is_visible(self, imageable: UsdGeom.Imageable):
        visible = imageable.GetVisibilityAttr().Get()
        purpose = imageable.GetPurposeAttr().Get()
        return visible == "inherited" and purpose == "default"

    async def test_particlePostProcessing_isosurface(self):
        particlePath = Sdf.Path("/particles")
        particleSpacing = 5.0
        self.create_particle_set_pointinstancer(
            path=particlePath, initial_point=Gf.Vec3f(0.0, 50.0, 0.0), particle_spacing=particleSpacing
        )

        # add an isosurface
        PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check if an isosurface is generated and it has similar extents to the points prim
        isoExtents0 = self.check_isosurface_exists(particle_paths=[particlePath])

        # start simulation
        await self.step(1)
        isoExtents1 = self.get_isosurface_extents()

        # check if it moved
        self.check_extents_differ(extents0=isoExtents0, extents1=isoExtents1)

        # sanity check if it falls through ground plane.
        self.assertGreaterEqual(isoExtents1[0][1], 0.0)

    # adds a particle set with isosurface and toggles enable/disable with attribute or API
    async def run_particlePostProcessing_isosurface_enable_disable(self, use_api=False):
        particlePath = Sdf.Path("/particles")
        particleSpacing = 5.0
        particlePointsPrim = self.create_particle_set_pointinstancer(
            path=particlePath,
            initial_point=Gf.Vec3f(0.0, 50.0, 0.0),
            particle_spacing=particleSpacing,
        )
        particlePointsInstancer = UsdGeom.PointInstancer(particlePointsPrim)

        # add an isosurface
        isosurfaceAPI = PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check if an isosurface is generated and it has similar extents to the points prim
        isoExtents0 = self.check_isosurface_exists(particle_paths=[particlePath])

        # disable
        if use_api:
            self._particle_system.GetPrim().RemoveAPI(PhysxSchema.PhysxParticleIsosurfaceAPI)
        else:
            isosurfaceAPI.CreateIsosurfaceEnabledAttr().Set(False)
        await omni.kit.app.get_app().next_update_async()

        # check if it is gone
        isoPrim = self._stage.GetPrimAtPath(self.get_isosurface_path())
        self.assertFalse(isoPrim)

        # check if particles are visible
        self.assertTrue(self.is_visible(particlePointsInstancer))

        # enable - check if there and particles invisible
        if use_api:
            isosurfaceAPI = PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(self._particle_system.GetPrim())
        else:
            isosurfaceAPI.CreateIsosurfaceEnabledAttr().Set(True)
        await omni.kit.app.get_app().next_update_async()

        # extents should be same as before
        isoExtents1 = self.check_isosurface_exists(particle_paths=[particlePath])
        self.check_extents_same(extents0=isoExtents0, extents1=isoExtents1)

        # play
        await self.step(1)

        # check if still there and moving
        isoExtents2 = self.check_isosurface_exists(particle_paths=[particlePath])
        self.check_extents_differ(extents0=isoExtents2, extents1=isoExtents1)

        # disable
        if use_api:
            self._particle_system.GetPrim().RemoveAPI(PhysxSchema.PhysxParticleIsosurfaceAPI)
        else:
            isosurfaceAPI.CreateIsosurfaceEnabledAttr().Set(False)
        await omni.kit.app.get_app().next_update_async()

        # check if gone and particle moving
        isoPrim = self._stage.GetPrimAtPath(self.get_isosurface_path())
        self.assertFalse(isoPrim)
        self.assertTrue(self.is_visible(particlePointsInstancer))

        particleExtent0 = self.get_extents_instancer(particlePointsInstancer)

        # play
        await self.step(1)

        particleExtent1 = self.get_extents_instancer(particlePointsInstancer)
        self.check_extents_differ(extents0=particleExtent0, extents1=particleExtent1)

        # enable
        if use_api:
            isosurfaceAPI = PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(self._particle_system.GetPrim())
        else:
            isosurfaceAPI.CreateIsosurfaceEnabledAttr().Set(True)
        await omni.kit.app.get_app().next_update_async()

        # check if there and particles invisible
        isoExtents3 = self.check_isosurface_exists(particle_paths=[particlePath])

        await self.step(1)

        # check if it moves
        isoExtents4 = self.get_isosurface_extents()
        self.check_extents_differ(extents0=isoExtents3, extents1=isoExtents4)

    async def test_particlePostProcessing_isosurface_enable_disable(self):
        await self.run_particlePostProcessing_isosurface_enable_disable(use_api=False)

    async def test_particlePostProcessing_isosurface_add_remove_api(self):
        await self.run_particlePostProcessing_isosurface_enable_disable(use_api=True)

    # add two particle sets and toggle them between fluid and not, testing
    # if they are added into the surface.
    async def test_particlePostProcessing_isosurface_fluid_toggle(self):
        # two particle sets, both initially fluid
        particlePath1 = Sdf.Path("/particles1")
        particleSpacing = 5.0
        particlePointsPrim1 = self.create_particle_set_pointinstancer(
            path=particlePath1, initial_point=Gf.Vec3f(0.0, 50.0, 0.0), particle_spacing=particleSpacing
        )
        particlePointsInstancer1 = UsdGeom.PointInstancer(particlePointsPrim1)

        particlePath2 = Sdf.Path("/particles2")
        particleSpacing = 5.0
        particlePointsPrim2 = self.create_particle_set_pointinstancer(
            path=particlePath2, initial_point=Gf.Vec3f(200.0, 50.0, 0.0), particle_spacing=particleSpacing
        )
        particlePointsInstancer2 = UsdGeom.PointInstancer(particlePointsPrim2)

        # add isosurface - both should be in the surface
        PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check if an isosurface is generated
        self.check_isosurface_exists(particle_paths=[particlePath1, particlePath2])

        # toggle fluid on set 2 - should get ejected from isosurface
        particleSetAPI = PhysxSchema.PhysxParticleSetAPI(particlePointsPrim2.GetPrim())
        particleSetAPI.CreateFluidAttr().Set(False)
        await omni.kit.app.get_app().next_update_async()

        # check if iso still exists & compare extents
        self.check_isosurface_exists(particle_paths=[particlePath1])
        self.assertTrue(self.is_visible(particlePointsInstancer2))

        # toggle fluid on set 1 - should get ejected from isosurface
        particleSetAPI1 = PhysxSchema.PhysxParticleSetAPI(particlePointsPrim1.GetPrim())
        particleSetAPI1.CreateFluidAttr().Set(False)
        await omni.kit.app.get_app().next_update_async()

        # iso still exists but doesn't have in it anything anymore
        meshPrim = UsdGeom.Mesh(self._stage.GetPrimAtPath(self.get_isosurface_path()))
        isoExtents2 = self.get_extents(meshPrim)
        self.assertEqual(isoExtents2[0], isoExtents2[1])
        self.assertTrue(self.is_visible(particlePointsInstancer1))
        self.assertTrue(self.is_visible(particlePointsInstancer2))

        # toggle fluid on set 2 again
        particleSetAPI.CreateFluidAttr().Set(True)
        await omni.kit.app.get_app().next_update_async()

        # check if iso exists & compare extents
        isoExtents3 = self.check_isosurface_exists(particle_paths=[particlePath2])
        self.assertTrue(self.is_visible(particlePointsInstancer1))

        particleExtents3 = self.get_extents_instancer(particlePointsInstancer1)

        # simulate some steps
        await self.step(1)

        # check if the points moved
        particleExtents4 = self.get_extents_instancer(particlePointsInstancer1)
        self.check_extents_differ(extents0=particleExtents3, extents1=particleExtents4)

        # check if the isosurface moved
        isoExtents4 = self.check_isosurface_exists(particle_paths=[particlePath2])
        self.check_extents_differ(extents0=isoExtents4, extents1=isoExtents3)

        # toggle set 1 to fluid again as well
        particleSetAPI1.CreateFluidAttr().Set(True)
        await omni.kit.app.get_app().next_update_async()

        # check if an isosurface is generated
        self.check_isosurface_exists(particle_paths=[particlePath1, particlePath2])

    # test if the isosurface & visibility attributes are correct across a sequence
    # of play / pause / stop / play interactions driven by the Kit timeline.
    async def test_particlePostProcessing_isosurface_play_pause_stop(self):
        particlePath = Sdf.Path("/particles")
        particleSpacing = 5.0
        self.create_particle_set_pointinstancer(
            path=particlePath, initial_point=Gf.Vec3f(0.0, 50.0, 0.0), particle_spacing=particleSpacing
        )

        # add an isosurface
        PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check if an isosurface is generated and it has similar extents to the points prim
        isoExtents0 = self.check_isosurface_exists(particle_paths=[particlePath])

        await self.step(1)

        # check if an isosurface is generated and it has similar extents to the points prim
        isoExtents1 = self.check_isosurface_exists(particle_paths=[particlePath])
        # check if the isosurface moved
        self.check_extents_differ(extents0=isoExtents0, extents1=isoExtents1)

        await self.step(1, stop_timeline_after=True)

        # check if an isosurface is generated and it has similar extents to the points prim
        isoExtents2 = self.check_isosurface_exists(particle_paths=[particlePath])
        # check if the isosurface moved
        self.check_extents_differ(extents0=isoExtents1, extents1=isoExtents2)
        # check if at initial position again
        self.check_extents_same(extents0=isoExtents0, extents1=isoExtents2)

        await self.step(1)

        # check if an isosurface is generated and it has similar extents to the points prim
        isoExtents3 = self.check_isosurface_exists(particle_paths=[particlePath])
        # check if the isosurface moved
        self.check_extents_differ(extents0=isoExtents2, extents1=isoExtents3)

    async def test_particlePostProcessing_isosurface_surface_distance(self):
        particlePath = Sdf.Path("/particles")
        particleSpacing = 5.0
        self.create_particle_set_pointinstancer(
            path=particlePath, initial_point=Gf.Vec3f(0.0, 50.0, 0.0), particle_spacing=particleSpacing
        )

        # add an isosurface
        isosurfaceAPI = PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check if an isosurface is generated and it has similar extents to the points prim
        isoExtents0 = self.check_isosurface_exists(particle_paths=[particlePath])

        # change the surface distance attribute
        # we know it's 5.0 right now, so check 6.0
        isosurfaceAPI.CreateSurfaceDistanceAttr().Set(6.0)
        await omni.kit.app.get_app().next_update_async()

        # check if extents are larger now
        isoExtents1 = self.get_isosurface_extents()

        self.assertLess(isoExtents1[0][0], isoExtents0[0][0])
        self.assertLess(isoExtents1[0][1], isoExtents0[0][1])
        self.assertLess(isoExtents1[0][2], isoExtents0[0][2])
        self.assertGreater(isoExtents1[1][0], isoExtents0[1][0])
        self.assertGreater(isoExtents1[1][1], isoExtents0[1][1])
        self.assertGreater(isoExtents1[1][2], isoExtents0[1][2])

    # OM-36091
    @unittest.skip("OM-47472")
    async def test_particlePostProcessing_isosurface_save_restore(self):
        filename = "particles.usda"

        particlePath = Sdf.Path("/particles")
        particleSpacing = 5.0
        self.create_particle_set(
            path=particlePath, initial_point=Gf.Vec3f(0.0, 50.0, 0.0), particle_spacing=particleSpacing
        )

        # add an isosurface
        PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check if an isosurface is generated and it has similar extents to the points prim
        isoExtents0 = self.check_isosurface_exists(particle_paths=[particlePath])

        # run for a few moments
        await self.step(1)

        # check if it moved
        isoExtents1 = self.get_isosurface_extents()
        self.check_extents_differ(extents0=isoExtents0, extents1=isoExtents1)

        # save
        self._stage.Export(filename)

        # check that save does not affect isosurface
        isoExtentsSaved = self.get_isosurface_extents()
        self.check_extents_same(extents0=isoExtentsSaved, extents1=isoExtents1)

        # run some more
        await self.step(1)

        # check that it ran
        isoExtents2 = self.get_isosurface_extents()
        self.check_extents_differ(extents0=isoExtents1, extents1=isoExtents2)

        # load the saved file
        await omni.usd.get_context().open_stage_async(filename)
        self._stage = omni.usd.get_context().get_stage()

        # get the new isosurface extents - check that iso is same as in saved state
        restoredExtents = self.check_isosurface_exists(particle_paths=[particlePath])
        self.check_extents_same(restoredExtents, isoExtents1)

        # run
        await self.step(1)

        # check if it moves
        restoredExtents1 = self.get_isosurface_extents()
        self.check_extents_differ(extents0=restoredExtents1, extents1=restoredExtents)

        # run + stop
        await self.step(num_steps=1, stop_timeline_after=True)

        stoppedExtents = self.get_isosurface_extents()
        self.check_extents_same(extents0=stoppedExtents, extents1=restoredExtents)

        # cleanup the file save
        os.remove(filename)

    # OM-47212
    async def test_particlePostProcessing_remove_readd_particle_system(self):
        particlePath = Sdf.Path("/particles")
        particleSpacing = 5.0
        self.create_particle_set_pointinstancer(
            path=particlePath, initial_point=Gf.Vec3f(0.0, 50.0, 0.0), particle_spacing=particleSpacing
        )

        # remove particle system
        self._stage.RemovePrim(self._particle_system_path)
        await omni.kit.app.get_app().next_update_async()

        # readd particle system
        particleUtils.add_physx_particle_system(
            stage=self._stage,
            particle_system_path=self._particle_system_path,
            simulation_owner=self._scene.GetPath(),
        )
        self._particle_system = PhysxSchema.PhysxParticleSystem.Get(self._stage, self._particle_system_path)

        # add isosurface
        PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # make sure it exists
        self.check_isosurface_exists(particle_paths=[particlePath])

        # run + stop - make sure it doesn't crash
        await self.step(num_steps=1, stop_timeline_after=True)

    # OM-47200
    async def test_particlePostProcessing_duplicate_particle_system(self):
        particlePath = Sdf.Path("/particles")
        particleSpacing = 5.0
        particlePointsPrim = self.create_particle_set_pointinstancer(
            path=particlePath, initial_point=Gf.Vec3f(0.0, 50.0, 0.0), particle_spacing=particleSpacing
        )

        # add an isosurface
        PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check if an isosurface is generated and it has similar extents to the points prim
        isoExtents0 = self.check_isosurface_exists(particle_paths=[particlePath])

        # duplicate particle system
        duplicate_path = Sdf.Path("/World/Particlesystem_01")
        omni.usd.duplicate_prim(stage=self._stage, prim_path=self._particle_system_path, path_to=duplicate_path)

        # remove the particle system relationship.
        particlesAPI = PhysxSchema.PhysxParticleSetAPI(particlePointsPrim.GetPrim())
        PhysxSchema.PhysxParticleAPI(particlesAPI).GetParticleSystemRel().ClearTargets(True)

        await omni.kit.app.get_app().next_update_async()

        # add the set to the second particle system
        particleSystemTargets = [duplicate_path]
        PhysxSchema.PhysxParticleAPI(particlesAPI).CreateParticleSystemRel().SetTargets(particleSystemTargets)
        await omni.kit.app.get_app().next_update_async()

        # test that there is an isosurface on the second system
        isoExtents1 = self.check_isosurface_exists(particle_paths=[particlePath], particle_system_path=duplicate_path)

        # iso extents should be the same
        self.check_extents_same(isoExtents0, isoExtents1)

        # test that is no isosurface on the first system
        isoExtents2 = self.get_isosurface_extents()

        # min == max means no isosurface
        self.assertEqual(isoExtents2[0], isoExtents2[1])

        # run + stop - make sure it doesn't crash
        await self.step(num_steps=1, stop_timeline_after=True)

    async def _run_isosurface_update_usd_test(self, enableIsosurface, updateToUsdSetting, updateParticlesToUsdSetting):
        # setup settings (backup restore is in setup/teardown)
        self._settings.set(pb.SETTING_UPDATE_TO_USD, updateToUsdSetting)
        # always enable, if updateToUSD is off, it overrides this and velocities are not updated
        self._settings.set(pb.SETTING_UPDATE_VELOCITIES_TO_USD, True)
        self._settings.set(pb.SETTING_UPDATE_PARTICLES_TO_USD, updateParticlesToUsdSetting)

        particlePath = Sdf.Path("/particles")
        particleSpacing = 5.0
        initial_pos = Gf.Vec3f(0.0, 50.0, 0.0)
        particle_prim = self.create_particle_set_pointinstancer(
            path=particlePath, initial_point=initial_pos, particle_spacing=particleSpacing
        )
        particle_instancer = UsdGeom.PointInstancer(particle_prim)
        position_attr = particle_instancer.GetPositionsAttr()
        vels_attr = particle_instancer.GetVelocitiesAttr()

        # add isosurface if requested
        if enableIsosurface:
            PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check if an isosurface is generated and it has similar extents to the points prim
        if enableIsosurface:
            isoExtents0 = self.check_isosurface_exists(particle_paths=[particlePath])

        await self.step(1)

        if enableIsosurface:
            isoExtents1 = self.get_isosurface_extents()

        # if update to usd is off, nothing must move or be updated:
        if not updateToUsdSetting:
            # check that not moved and velocity not updated:
            self.assertEqual(initial_pos, position_attr.Get()[0])
            self.assertEqual(Gf.Vec3f(0.0), vels_attr.Get()[0])
            if enableIsosurface:
                self.check_extents_same(extents0=isoExtents0, extents1=isoExtents1)
            return

        # otherwise:
        # - isosurface mesh updates if isosurface api is applied
        # - if isosurface is off, updateParticlesToUsd does not matter, and the positions should be updated in any case
        # - if isosurface is on, updateParticlesToUsd True/False determines particle pos and vel updated/not updated
        #   (specifically, velocity updates according to vel update setting, but we set it to true here)
        pointsAndVelShouldUpdate = not enableIsosurface or updateParticlesToUsdSetting
        if enableIsosurface:
            # check if the isosurface moved
            self.check_extents_differ(extents0=isoExtents0, extents1=isoExtents1)

        if pointsAndVelShouldUpdate:
            self.assertLess(position_attr.Get()[0][1], initial_pos[1])
            self.assertLess(vels_attr.Get()[0][1], 0)

    # OM-47210 - test update to USD settings
    async def test_particlePostProcessing_isosurface_update_to_usd_settings(self):
        for updateToUsd in [False, True]:
            for enableIsosurface in [False, True]:
                # use subtest context to get better failure info and run all permutations even if any one fails
                with self.subTest(
                    updateToUsd=updateToUsd, updateParticlesToUsd=updateToUsd, enableIsosurface=enableIsosurface
                ):
                    # OM-
                    await self._run_isosurface_update_usd_test(
                        enableIsosurface=enableIsosurface,
                        updateToUsdSetting=updateToUsd,
                        updateParticlesToUsdSetting=updateToUsd,
                    )
                # need to cycle teardown and setup to start fresh, regardless of subtest success/failure
                await self.tearDown()
                await self.setUp()

    # OM-47101
    async def test_particlePostProcessing_multiple_particlesystems(self):

        # create two particle sets
        particlePath1 = Sdf.Path("/particles1")
        particleSpacing = 5.0
        self.create_particle_set_pointinstancer(
            path=particlePath1, initial_point=Gf.Vec3f(0.0, 50.0, 0.0), particle_spacing=particleSpacing
        )

        particlePath2 = Sdf.Path("/particles2")
        particleSpacing = 5.0
        particlePointsPrim2 = self.create_particle_set_pointinstancer(
            path=particlePath2, initial_point=Gf.Vec3f(200.0, 50.0, 0.0), particle_spacing=particleSpacing
        )
        instancer2 = UsdGeom.PointInstancer(particlePointsPrim2)

        # add isosurface - both should be in the surface
        PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(self._particle_system.GetPrim())

        # wait two update cycles because we defer initial creation due to a rendering bug
        # see particleAuthoring.cpp:update()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check if an isosurface is generated
        self.check_isosurface_exists(particle_paths=[particlePath1, particlePath2])

        # remove the particle system relationship for the second particle set.
        particlesAPI2 = PhysxSchema.PhysxParticleSetAPI(particlePointsPrim2.GetPrim())
        PhysxSchema.PhysxParticleAPI(particlesAPI2).GetParticleSystemRel().ClearTargets(True)

        await omni.kit.app.get_app().next_update_async()

        # isosurface now only on first set
        self.check_isosurface_exists(particle_paths=[particlePath1])
        self.assertTrue(self.is_visible(instancer2))

        # create a new particle system
        particleSystemPath2 = self._default_prim_path.AppendChild("particleSystem_01")
        particleUtils.add_physx_particle_system(
            stage=self._stage,
            particle_system_path=particleSystemPath2,
            simulation_owner=self._scene.GetPath(),
        )
        particleSystem2 = PhysxSchema.PhysxParticleSystem.Get(self._stage, particleSystemPath2)

        # save p1 for later
        points = instancer2.GetPositionsAttr().Get()
        p1 = points[0]

        # add smoothing to this particle systen
        PhysxSchema.PhysxParticleSmoothingAPI.Apply(particleSystem2.GetPrim())
        await omni.kit.app.get_app().next_update_async()

        # assign the new particle system to set 2
        particleSystemTargets = [particleSystemPath2]
        PhysxSchema.PhysxParticleAPI(particlesAPI2).CreateParticleSystemRel().SetTargets(particleSystemTargets)
        await omni.kit.app.get_app().next_update_async()

        # check if smoothing happened
        points = instancer2.GetPositionsAttr().Get()
        self.assertNotEqual(points[0], p1)
        self.assertTrue(particlesAPI2.GetSimulationPointsAttr().HasAuthoredValue())
