# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.test
import omni.kit.commands
from pxr import Gf, Sdf, Vt, UsdGeom, Usd, UsdLux, PhysxSchema
import omni.physx
from omni.physxtests import utils
from omni.physx import get_physx_cooking_interface, get_physx_cooking_private_interface, get_physx_simulation_interface
from omni.physx.scripts import physicsUtils, particleUtils
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
import omni.usd
import unittest
import carb
import os

class PhysxParticleSamplingAPITestKitStage(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Core
    
     # runs before each test case
    async def setUp(self):
        await super().setUp()
        await self.base_setup()

    # runs after each test case and runs base_terminate if setup was called:
    async def tearDown(self):
        self._settings.set(self._DEBUG_VIZ_SETTING, self._debug_viz_setting_restore)
        self._settings.set_int(self._UPDATE_TO_USD_SETTING, self._update_to_usd_setting_restore)
        await super().tearDown()

    async def _wait_cooking_finished(self):
        while True:
            await omni.kit.app.get_app().next_update_async()
            cooking_statistics = get_physx_cooking_private_interface().get_cooking_statistics()
            running_tasks = cooking_statistics.total_scheduled_tasks - cooking_statistics.total_finished_tasks
            self.assertGreaterEqual(running_tasks, 0)
            if running_tasks <= 0:
                break

    async def base_setup(self):
        self._DEBUG_VIZ_SETTING = omni.physx.bindings._physx.SETTING_DISPLAY_PARTICLES
        self._settings = carb.settings.get_settings()
        self._debug_viz_setting_restore = self._settings.get(self._DEBUG_VIZ_SETTING)
        # hide all as default setting for tests
        self._settings.set_int(self._DEBUG_VIZ_SETTING, 0)

        self._UPDATE_TO_USD_SETTING = omni.physx.bindings._physx.SETTING_UPDATE_TO_USD
        self._settings = carb.settings.get_settings()
        self._update_to_usd_setting_restore = self._settings.get(self._UPDATE_TO_USD_SETTING)
        # set to 1 for this test
        self._settings.set_int(self._UPDATE_TO_USD_SETTING, 1)

        self.fail_on_log_error = True
        self._stage = await utils.new_stage_setup()
        sphereLight = UsdLux.SphereLight.Define(self._stage, Sdf.Path("/SphereLight"))
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

        self._upAxis = UsdGeom.GetStageUpAxis(self._stage)
        self._defaultPrimPath = self._stage.GetDefaultPrim().GetPath()

        # add physics scene
        scenePath = "/World/PhysicsScene"
        omni.kit.commands.execute("AddPhysicsScene", stage=self._stage, path=scenePath)

        # create a particle system
        particleSystemPath = Sdf.Path("/particleSystem0")
        self._particle_system_path = particleSystemPath

        particleSpacing = 2.0
        restOffset = particleSpacing * 0.9
        solidRestOffset = restOffset
        fluidRestOffset = restOffset * 0.6
        particleContactOffset = max(solidRestOffset + 0.001, fluidRestOffset / 0.6)
        contactOffset = restOffset + 0.001
        particleUtils.add_physx_particle_system(
            stage=self._stage,
            particle_system_path=particleSystemPath,
            contact_offset=contactOffset,
            rest_offset=restOffset,
            particle_contact_offset=particleContactOffset,
            solid_rest_offset=solidRestOffset,
            fluid_rest_offset=fluidRestOffset,
            solver_position_iterations=4,
            simulation_owner=scenePath,
        )

        particleSystem = self._stage.GetPrimAtPath(particleSystemPath)
        # Create a pbd particle material and set it on the particle system
        pbd_particle_material_path = omni.usd.get_stage_next_free_path(self._stage, "/pbdParticleMaterial", True)
        particleUtils.add_pbd_particle_material(self._stage, pbd_particle_material_path, cohesion=0.002)
        physicsUtils.add_physics_material_to_prim(self._stage, particleSystem, pbd_particle_material_path)

        self._particleContactOffset = particleContactOffset

        (self._mesh,) = await self._create_mesh_primitives(["Cube"])
        self._mesh_prim = self._mesh.GetPrim()
        self._mesh_path = self._mesh.GetPath()

        # scale down to avoid long runtime during sampling
        self._scale_vec = Gf.Vec3d(0.2, 0.2, 0.2)
        self.set_prim_scale(self._mesh_prim, self._scale_vec)

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

    @staticmethod
    def set_prim_scale(prim: Usd.Prim, scaleVec: Gf.Vec3d):
        scale_mtx = Gf.Matrix4d().SetScale(scaleVec)
        omni.kit.commands.execute("TransformPrim", path=prim.GetPath(), new_transform_matrix=scale_mtx)

    def check_inside_AABB(self, p: Gf.Vec3f, minimum: Gf.Vec3f, maximum: Gf.Vec3f):
        self.assertGreaterEqual(p[0], minimum[0])
        self.assertGreaterEqual(p[1], minimum[1])
        self.assertGreaterEqual(p[2], minimum[2])
        self.assertLessEqual(p[0], maximum[0])
        self.assertLessEqual(p[1], maximum[1])
        self.assertLessEqual(p[2], maximum[2])

    def step(self, num_steps=1, dt=1.0 / 60.0):
        for i in range(num_steps):
            get_physx_simulation_interface().simulate(dt, i * dt)
            get_physx_simulation_interface().fetch_results()

    def get_position_array(self, mesh_path):
        mesh_prim = self._stage.GetPrimAtPath(mesh_path)
        samplingApi = PhysxSchema.PhysxParticleSamplingAPI(mesh_prim)
        pointTargets = samplingApi.GetParticlesRel().GetTargets()

        particlePath = pointTargets[0]
        pointsPrim = self._stage.GetPrimAtPath(particlePath)
        points = UsdGeom.Points(pointsPrim)

        positions = points.GetPointsAttr().Get()
        return positions

    def get_number_of_points(self, mesh_path):
        positions = self.get_position_array(mesh_path)
        return len(positions)

    def get_first_point(self, mesh_path):
        positions = self.get_position_array(mesh_path)
        return positions[0]

    def get_particleSet_Api(self, mesh_path):
        mesh_prim = self._stage.GetPrimAtPath(mesh_path)
        samplingApi = PhysxSchema.PhysxParticleSamplingAPI(mesh_prim)
        pointTargets = samplingApi.GetParticlesRel().GetTargets()
        particlePath = pointTargets[0]
        pointsPrim = self._stage.GetPrimAtPath(particlePath)
        particleApi = PhysxSchema.PhysxParticleSetAPI(pointsPrim)
        return particleApi

    def get_particle_Api(self, mesh_path):
        mesh_prim = self._stage.GetPrimAtPath(mesh_path)
        samplingApi = PhysxSchema.PhysxParticleSamplingAPI(mesh_prim)
        pointTargets = samplingApi.GetParticlesRel().GetTargets()
        particlePath = pointTargets[0]
        pointsPrim = self._stage.GetPrimAtPath(particlePath)
        particleApi = PhysxSchema.PhysxParticleAPI(pointsPrim)
        return particleApi

    async def check_API_and_particles(self, mesh_path, pointInstancer = False):
        mesh_prim = self._stage.GetPrimAtPath(mesh_path)
        # check API
        self.assertTrue(PhysxSchema.PhysxParticleSamplingAPI.Get(self._stage, mesh_path))

        # check if points have been created
        samplingApi = PhysxSchema.PhysxParticleSamplingAPI(mesh_prim)
        pointTargets = samplingApi.GetParticlesRel().GetTargets()

        self.assertTrue(pointTargets)
        self.assertTrue(len(pointTargets) == 1)

        particlePath = pointTargets[0]
        pointsPrim = self._stage.GetPrimAtPath(particlePath)

        if pointInstancer:
            instancer = UsdGeom.PointInstancer(pointsPrim)
            self.assertTrue(instancer)

            prototypes = instancer.GetPrototypesRel().GetTargets()
            self.assertGreaterEqual(len(prototypes), 1)
        else:
            points = UsdGeom.Points(pointsPrim)
            self.assertTrue(points)

    #@unittest.skip("OM-49894")
    async def test_particleSampling_sample_particles(self):
        omni.kit.commands.execute("AddParticleSamplingCommand", prim=self._mesh_prim)
        await self._wait_cooking_finished()
        await self.check_API_and_particles(self._mesh_path)

        # check if particle radius is within limits
        samplingApi = PhysxSchema.PhysxParticleSamplingAPI(self._mesh_prim)

        # check if number of particles has been limited to 50000.
        pointTargets = samplingApi.GetParticlesRel().GetTargets()
        if pointTargets:
            pointsPrim = self._stage.GetPrimAtPath(pointTargets[0])
            points = UsdGeom.Points(pointsPrim)
            pointPositions = points.GetPointsAttr().Get()
            self.assertLessEqual(len(pointPositions), 50000)

        # get AABB + slack for mesh cube
        mesh_pts = self._mesh.GetPointsAttr().Get()
        extent = UsdGeom.PointBased.ComputeExtent(mesh_pts)
        transform = self._mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        for i in range(len(extent)):
            extent[i] = transform.Transform(extent[i])

        # get fluid/solidRestOffset to get a good slack value for the extent test.
        particleSetApi = self.get_particleSet_Api(self._mesh_path)
        fluid = particleSetApi.GetFluidAttr().Get()

        particleApi = self.get_particle_Api(self._mesh_path)
        particleSystemPaths = particleApi.GetParticleSystemRel().GetTargets()
        particleSystem = PhysxSchema.PhysxParticleSystem(self._stage.GetPrimAtPath(particleSystemPaths[0]))
        radius = 0.0
        if (fluid):
            radius = particleSystem.GetFluidRestOffsetAttr().Get()
        else:
            radius = particleSystem.GetSolidRestOffsetAttr().Get()

        extent[0] = extent[0] - Gf.Vec3f(radius)
        extent[1] = extent[1] + Gf.Vec3f(radius)

        # check if points are inside bounding box of the mesh
        for p in pointPositions:
            self.check_inside_AABB(p, extent[0], extent[1])

    async def run_particleSampling_remove(self, wait_and_check = True):
        omni.kit.commands.execute("AddParticleSamplingCommand", prim=self._mesh_prim)
        if (wait_and_check):
            await self._wait_cooking_finished()

            await self.check_API_and_particles(self._mesh_path)

            # get pointTargets for later
            samplingApi = PhysxSchema.PhysxParticleSamplingAPI(self._mesh_prim)
            pointTargets = samplingApi.GetParticlesRel().GetTargets()
            # get first point for later
            p1 = self.get_first_point(self._mesh_path)
            numPointsFirstRun = self.get_number_of_points(self._mesh_path)

        # remove particlesCommand
        omni.kit.commands.execute("RemoveParticleSamplingCommand", stage=self._stage, prim=self._mesh_prim)

        # check if API has been removed
        self.assertFalse(PhysxSchema.PhysxParticleSamplingAPI.Get(self._stage, self._mesh_path))

        # check if particle prim is still there
        if (wait_and_check):
            self.assertTrue(self._stage.GetPrimAtPath(pointTargets[0]))

        if (wait_and_check):
            # OM-46517: check if toggling fluid attribute does not lead to a crash/does not lead to any sampling activity.
            particleSetAPI = PhysxSchema.PhysxParticleSetAPI(self._stage.GetPrimAtPath(pointTargets[0]))
            fluid = particleSetAPI.GetFluidAttr().Get()
            particleSetAPI.CreateFluidAttr().Set(not fluid)
            await self._wait_cooking_finished()

            points = UsdGeom.Points(self._stage.GetPrimAtPath(pointTargets[0]))
            pointPositions = points.GetPointsAttr().Get()
            numPointsSecondRun = len(pointPositions)
            self.assertEqual(0, numPointsSecondRun)
        else:
            particlePrim = self._stage.GetPrimAtPath(Sdf.Path("/World/particleSet"))
            self.assertFalse(particlePrim)

        # check if custom attributes have been removed
        self.assertFalse(self._mesh_prim.HasAttribute("physxParticleSampling:crc"))

    #@unittest.skip("OM-49894")
    async def test_particleSampling_remove(self):
        await self.run_particleSampling_remove(wait_and_check=True)

    #@unittest.skip("OM-49894")
    async def test_particleSampling_remove_immediately(self):
        await self.run_particleSampling_remove(wait_and_check=False)

    # adds a longer sampling and calls remove before sampling is finished - 
    # the latest sampling task should be canceled.
    #@unittest.skip("OM-49894")
    async def test_particleSampling_remove_while_sampling(self):
        omni.kit.commands.execute("AddParticleSamplingCommand", prim=self._mesh_prim)
        await self._wait_cooking_finished()

        await self.check_API_and_particles(self._mesh_path)

        # get pointTargets for later
        samplingApi = PhysxSchema.PhysxParticleSamplingAPI(self._mesh_prim)
        pointTargets = samplingApi.GetParticlesRel().GetTargets()
        # get first point for later
        p1 = self.get_first_point(self._mesh_path)
        numPointsFirstRun = self.get_number_of_points(self._mesh_path)

        # set number of points to unbounded and kick of a longer sampling task
        samplingApi.CreateMaxSamplesAttr().Set(0)
        scale_mtx = Gf.Matrix4d().SetScale(self._scale_vec + Gf.Vec3d(0.7, 0.7, 0.7))
        omni.kit.commands.execute("TransformPrim", path=self._mesh_path, new_transform_matrix=scale_mtx)

        # wait for scale change to be picked up
        await omni.kit.app.get_app().next_update_async() 

        # immediately call the remove command.
        omni.kit.commands.execute("RemoveParticleSamplingCommand", stage=self._stage, prim=self._mesh_prim)

        # still need to wait for the sampling to finish to check if something happened.
        await self._wait_cooking_finished()

        # check if API has been removed
        self.assertFalse(PhysxSchema.PhysxParticleSamplingAPI.Get(self._stage, self._mesh_path))

        # check if particle prim is still there
        self.assertTrue(self._stage.GetPrimAtPath(pointTargets[0]))

        # check if custom attributes have been removed
        self.assertFalse(self._mesh_prim.HasAttribute("physxParticleSampling:crc"))


    async def run_particleSampling_undo_redo(self, wait_and_check=True):
        omni.kit.commands.execute("AddParticleSamplingCommand", prim=self._mesh_prim)
        if (wait_and_check):
            await self._wait_cooking_finished()

            # check API
            await self.check_API_and_particles(self._mesh_path)
            samplingApi = PhysxSchema.PhysxParticleSamplingAPI(self._mesh_prim)
            pointTargets = samplingApi.GetParticlesRel().GetTargets()

        # undo
        await omni.kit.app.get_app().next_update_async() # need to wait for one update cycle because otherwise the undo results in the add command never being dispatched.
        omni.kit.undo.undo()
        await omni.kit.app.get_app().next_update_async()

        # check if API and particle prim is deleted
        self.assertFalse(PhysxSchema.PhysxParticleSamplingAPI.Get(self._stage, self._mesh_path))

         # check if custom attributes have been removed
        self.assertFalse(self._mesh_prim.HasAttribute("physxParticleSampling:crc"))

        # redo
        omni.kit.undo.redo()
        await self._wait_cooking_finished()

        await self.check_API_and_particles(self._mesh_path)

    #@unittest.skip("OM-49894")
    async def test_particleSampling_undo_redo(self):
        await self.run_particleSampling_undo_redo(wait_and_check = True)

    #@unittest.skip("OM-49894")
    async def test_particleSampling_undo_redo_immediately(self):
        await self.run_particleSampling_undo_redo(wait_and_check = False)

    #@unittest.skip("OM-49894")
    async def test_particleSampling_resample_change_sampling_distance(self):
        # change sampling distance:
        # should always trigger a resampling, unless sampling distance is too small
        # and is clamped by the USD parsing.

        #base setup
        omni.kit.commands.execute("AddParticleSamplingCommand", prim=self._mesh_prim)
        await self._wait_cooking_finished()
        await self.check_API_and_particles(self._mesh_path)

        # get the current number uf points
        numPointsFirstRun = self.get_number_of_points(self._mesh_path)
        p1 = self.get_first_point(self._mesh_path)

        # set the sampling distance to very small (fluidRestOffset was 1.08)
        samplingApi = PhysxSchema.PhysxParticleSamplingAPI(self._mesh_prim)

        # this should trigger resampling, but will clamp at 0.75 * fluidRestOffset
        samplingApi.GetSamplingDistanceAttr().Set(0.05)
        await self._wait_cooking_finished()

        # should have more points now because of smaller distance
        numPointsSecondRun = self.get_number_of_points(self._mesh_path)
        p2 = self.get_first_point(self._mesh_path)
        self.assertGreater(numPointsSecondRun, numPointsFirstRun)
        self.assertNotEqual(p1, p2)

        # another small value - should not change number of points because of clamping + same seed in sampling
        # but will resample.
        samplingApi.GetSamplingDistanceAttr().Set(0.025)
        await self._wait_cooking_finished()

        numPointsThirdRun = self.get_number_of_points(self._mesh_path)
        p3 = self.get_first_point(self._mesh_path)
        self.assertEqual(numPointsThirdRun, numPointsSecondRun)
        self.assertAlmostEqual(p2, p3)

        # this should trigger a resampling again
        samplingApi.GetSamplingDistanceAttr().Set(3.0)
        await self._wait_cooking_finished()

        numPointsFourthRun = self.get_number_of_points(self._mesh_path)
        p4 = self.get_first_point(self._mesh_path)
        self.assertGreater(numPointsFirstRun, numPointsFourthRun)
        self.assertGreater(numPointsSecondRun, numPointsFourthRun)
        self.assertNotEqual(p3, p4)

    #@unittest.skip("OM-49894")
    async def test_particleSampling_toggle_fluid_solid(self):
        # toggling fluid/solid on the particle set will change
        # the clamping basis and potentially also the sampling distance,
        # if set to autocopmute.

        omni.kit.commands.execute("AddParticleSamplingCommand", prim=self._mesh_prim)
        await self._wait_cooking_finished()
        await self.check_API_and_particles(self._mesh_path)

        # get the current number uf points
        numPointsFirstRun = self.get_number_of_points(self._mesh_path)
        p1 = self.get_first_point(self._mesh_path)

        # toggling to solid will trigger a resampling and change sampling distance
        # because of the clamping
        particleSetApi = self.get_particleSet_Api(self._mesh_path)
        particleSetApi.GetFluidAttr().Set(False)
        await self._wait_cooking_finished()

        numPointsSecondRun = self.get_number_of_points(self._mesh_path)
        p2 = self.get_first_point(self._mesh_path)
        self.assertGreater(numPointsFirstRun, numPointsSecondRun)
        self.assertNotEqual(p1, p2)

        # back to fluid, should sample small particles again
        # because distance is still autocompute
        particleSetApi.GetFluidAttr().Set(True)
        await self._wait_cooking_finished()

        numPointsThirdRun = self.get_number_of_points(self._mesh_path)
        p3 = self.get_first_point(self._mesh_path)
        self.assertGreater(numPointsThirdRun, numPointsSecondRun)
        self.assertAlmostEqual(p1, p3)

    #@unittest.skip("OM-49894")
    async def test_particleSampling_autocompute_sampling_distance(self):
        omni.kit.commands.execute("AddParticleSamplingCommand", prim=self._mesh_prim)
        await self._wait_cooking_finished()
        await self.check_API_and_particles(self._mesh_path)

        # get the current number uf points - autocomputation is on.
        numPointsFirstRun = self.get_number_of_points(self._mesh_path)
        p1 = self.get_first_point(self._mesh_path)

        samplingApi = PhysxSchema.PhysxParticleSamplingAPI(self._mesh_prim)

        # set some random sampling distance - number of points should differ.
        samplingApi.GetSamplingDistanceAttr().Set(7.5)
        await self._wait_cooking_finished()

        numPointsSecondRun = self.get_number_of_points(self._mesh_path)
        p2 = self.get_first_point(self._mesh_path)
        self.assertNotEqual(numPointsFirstRun, numPointsSecondRun)
        self.assertNotEqual(p1, p2)

        # toggle fluid/solid - should not make a difference since sampling distance is set by user
        particleSetApi = self.get_particleSet_Api(self._mesh_path)
        particleSetApi.GetFluidAttr().Set(False)
        await self._wait_cooking_finished()

        numPointsThirdRun = self.get_number_of_points(self._mesh_path)
        p3 = self.get_first_point(self._mesh_path)
        self.assertEqual(numPointsSecondRun, numPointsThirdRun)
        self.assertAlmostEqual(p2, p3)

        # toggle back to setup for final check
        particleSetApi.GetFluidAttr().Set(True)
        await self._wait_cooking_finished()

        numPointsFourthRun = self.get_number_of_points(self._mesh_path)
        self.assertEqual(numPointsFourthRun, numPointsThirdRun)

        # set sampling distance to autocompute again - should get same number of points than in the beginning.
        samplingApi.CreateSamplingDistanceAttr().Set(0.0)
        await self._wait_cooking_finished()

        numPointsFifthRun = self.get_number_of_points(self._mesh_path)
        p4 = self.get_first_point(self._mesh_path)
        self.assertEqual(numPointsFifthRun, numPointsFirstRun)
        self.assertAlmostEqual(p1, p4)

    #@unittest.skip("OM-49894")
    async def test_particleSampling_change_max_samples(self):
        omni.kit.commands.execute("AddParticleSamplingCommand", prim=self._mesh_prim)
        await self._wait_cooking_finished()
        await self.check_API_and_particles(self._mesh_path)

        # get the current number uf points - autocomputation is on.
        numPointsFirstRun = self.get_number_of_points(self._mesh_path)

        samplingApi = PhysxSchema.PhysxParticleSamplingAPI(self._mesh_prim)

        # small maxSamples - number of points should be small.
        numSamplesSecondRun = 200
        samplingApi.GetMaxSamplesAttr().Set(numSamplesSecondRun)
        await self._wait_cooking_finished()

        numPointsSecondRun = self.get_number_of_points(self._mesh_path)
        self.assertGreater(numPointsFirstRun, numPointsSecondRun)
        self.assertEqual(numPointsSecondRun, numSamplesSecondRun)

        # change to larger value
        numSamplesThirdRun = 400
        samplingApi.GetMaxSamplesAttr().Set(numSamplesThirdRun)
        await self._wait_cooking_finished()

        numPointsThirdRun = self.get_number_of_points(self._mesh_path)
        self.assertGreater(numPointsThirdRun, numPointsSecondRun)
        self.assertEqual(numPointsThirdRun, numSamplesThirdRun)

        # change to unlimited - same number of points as in the beginning
        samplingApi.GetMaxSamplesAttr().Set(0)
        await self._wait_cooking_finished()

        numPointsFourthRun = self.get_number_of_points(self._mesh_path)
        self.assertGreater(numPointsFourthRun, numPointsThirdRun)
        self.assertEqual(numPointsFourthRun, numPointsFirstRun)

    #@unittest.skip("OM-49894")
    async def test_particleSampling_translate_mesh(self):
        # a translation of the source mesh should not trigger a resampling

        omni.kit.commands.execute("AddParticleSamplingCommand", prim=self._mesh_prim)
        await self._wait_cooking_finished()
        await self.check_API_and_particles(self._mesh_path)

        numPointsFirstRun = self.get_number_of_points(self._mesh_path)

        # get the UsdGeomPoints
        samplingApi = PhysxSchema.PhysxParticleSamplingAPI(self._mesh_prim)
        pointTargets = samplingApi.GetParticlesRel().GetTargets()
        pointsPrim = self._stage.GetPrimAtPath(pointTargets[0])
        points = UsdGeom.Points(pointsPrim)

        # get the extents of the particle prim
        particle_pts = points.GetPointsAttr().Get()
        firstExtent = UsdGeom.PointBased.ComputeExtent(particle_pts)
        transform = points.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        for i in range(len(firstExtent)):
            firstExtent[i] = transform.Transform(firstExtent[i])

        # translate it
        scale_mtx = Gf.Matrix4d().SetScale(self._scale_vec)
        translation = Gf.Vec3d(0.0, 10.0, 0.0)
        translate_mtx = Gf.Matrix4d().SetTranslate(translation)
        final_mtx = scale_mtx * translate_mtx
        omni.kit.commands.execute("TransformPrim", path=self._mesh_path, new_transform_matrix=final_mtx)

        # should not trigger a resampling, but still wait for this
        await self._wait_cooking_finished()

        numPointsSecondRun = self.get_number_of_points(self._mesh_path)
        self.assertEqual(numPointsFirstRun, numPointsSecondRun)

        particle_pts1 = points.GetPointsAttr().Get()
        secondExtent = UsdGeom.PointBased.ComputeExtent(particle_pts1)
        transform = points.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        for i in range(len(secondExtent)):
            secondExtent[i] = transform.Transform(secondExtent[i])

        # particles are always in world space, so check if each of the particles made the translation
        # need to use eps due to float/double conversion
        translationf = Gf.Vec3f(translation)
        for i in range(len(particle_pts)):
            translated = particle_pts[i] + translationf
            self.assertLess(abs(translated[0] - particle_pts1[i][0]), 0.001)
            self.assertLess(abs(translated[1] - particle_pts1[i][1]), 0.001)
            self.assertLess(abs(translated[2] - particle_pts1[i][2]), 0.001)

        # check extent differences as well
        diffMin = secondExtent[0] - firstExtent[0]
        diffMax = secondExtent[1] - firstExtent[1]

        self.assertLess(abs(diffMin[0] - translationf[0]), 0.001)
        self.assertLess(abs(diffMin[1] - translationf[1]), 0.001)
        self.assertLess(abs(diffMin[2] - translationf[2]), 0.001)

        self.assertLess(abs(diffMax[0] - translationf[0]), 0.001)
        self.assertLess(abs(diffMax[1] - translationf[1]), 0.001)
        self.assertLess(abs(diffMax[2] - translationf[2]), 0.001)

    #@unittest.skip("OM-49894")
    async def test_particleSampling_scale_mesh(self):
        # scaling mesh should trigger resampling

        omni.kit.commands.execute("AddParticleSamplingCommand", prim=self._mesh_prim)
        await self._wait_cooking_finished()
        await self.check_API_and_particles(self._mesh_path)

        numPointsFirstRun = self.get_number_of_points(self._mesh_path)
        p1 = self.get_first_point(self._mesh_path)

        # set scale to 0.3
        scale_mtx = Gf.Matrix4d().SetScale(self._scale_vec + Gf.Vec3d(0.1, 0.1, 0.1))
        omni.kit.commands.execute("TransformPrim", path=self._mesh_path, new_transform_matrix=scale_mtx)
        await self._wait_cooking_finished()

        numPointsSecondRun = self.get_number_of_points(self._mesh_path)
        p2 = self.get_first_point(self._mesh_path)
        self.assertGreater(numPointsSecondRun, numPointsFirstRun)
        self.assertNotEqual(p1, p2)

    #@unittest.skip("OM-49894")
    async def test_particleSampling_during_sim(self):
        # resampling during sim is not allowed - we change the scale of the source
        # mesh but it shouldn't trigger a resampling

        omni.kit.commands.execute("AddParticleSamplingCommand", prim=self._mesh_prim)
        await self._wait_cooking_finished()
        await self.check_API_and_particles(self._mesh_path)

        numPointsFirstRun = self.get_number_of_points(self._mesh_path)

        # play for a few steps
        omni.timeline.get_timeline_interface().play()
        for i in range(20):
            await omni.kit.app.get_app().next_update_async()

        omni.timeline.get_timeline_interface().pause()
        await omni.kit.app.get_app().next_update_async()

        # set scale to 0.3 - shouldn't do anything because simulation already started
        scale_mtx = Gf.Matrix4d().SetScale(self._scale_vec + Gf.Vec3d(0.1, 0.1, 0.1))
        omni.kit.commands.execute("TransformPrim", path=self._mesh_path, new_transform_matrix=scale_mtx)
        await self._wait_cooking_finished()

        numPointsSecondRun = self.get_number_of_points(self._mesh_path)
        self.assertEqual(numPointsSecondRun, numPointsFirstRun)

    # common setup function for OM-46517
    async def run_particleSampling_delete_relationship(self, delete_mesh = False):

        # mesh 1
        omni.kit.commands.execute("AddParticleSamplingCommand", prim=self._mesh_prim)
        await self._wait_cooking_finished()
        await self.check_API_and_particles(self._mesh_path)

        samplingAPI = PhysxSchema.PhysxParticleSamplingAPI(self._mesh_prim)
        particlesTargets = samplingAPI.GetParticlesRel().GetTargets()
        particlePrimPath = particlesTargets[0]
        particlePrim = self._stage.GetPrimAtPath(particlePrimPath)
        self.assertTrue(particlePrim)

        # create second mesh
        (self._mesh_1,) = await self._create_mesh_primitives(["Cube"])
        self._mesh_prim_1 = self._mesh_1.GetPrim()
        self._mesh_path_1 = self._mesh_1.GetPath()

        # scale down to avoid long runtime during sampling
        self.set_prim_scale(self._mesh_prim_1, self._scale_vec)

        # add sampling
        omni.kit.commands.execute("AddParticleSamplingCommand", prim=self._mesh_prim_1)
        await self._wait_cooking_finished()
        await self.check_API_and_particles(self._mesh_path_1)

        samplingAPI1 = PhysxSchema.PhysxParticleSamplingAPI(self._mesh_prim_1)
        particlesTargets1 = samplingAPI1.GetParticlesRel().GetTargets()
        particlePrimPath1 = particlesTargets1[0]
        particlePrim1 = self._stage.GetPrimAtPath(particlePrimPath1)
        self.assertTrue(particlePrim1)

        # check that the paths match.
        self.assertEqual(particlePrimPath1, particlePrimPath)

        # save the particle path
        self._particle_path = particlePrimPath

        pointsPrim = UsdGeom.Points(particlePrim)
        pointPositions = pointsPrim.GetPointsAttr().Get()
        numPoints1 = len(pointPositions)

        # remove the relationship/mesh
        if (delete_mesh):
            self._stage.RemovePrim(self._mesh_path)
        else:
            samplingAPI.GetParticlesRel().ClearTargets(True)

        await omni.kit.app.get_app().next_update_async()

        await self._wait_cooking_finished()

        # check mesh gone/relationship removed
        if (delete_mesh):
            self.assertFalse(self._stage.GetPrimAtPath(self._mesh_path))
        else:
            targets = samplingAPI.GetParticlesRel().GetTargets()
            self.assertTrue(len(targets) == 0)

        # check if particle prim is still there but with reduced number of points.
        self.assertTrue(self._stage.GetPrimAtPath(particlePrimPath))
        
        pointPositions = pointsPrim.GetPointsAttr().Get()
        numPoints2 = len(pointPositions)
        self._num_points = numPoints2

        self.assertGreater(numPoints1, numPoints2)

    # delete the mesh with the sampling API and check if the prim is not removed.
    # (formerly OM-46517, behaviour changed)
    #@unittest.skip("OM-49894")
    async def test_particleSampling_delete_mesh(self):
        await self.run_particleSampling_delete_relationship(delete_mesh = True)

        # add a new cube
        (newMesh, ) = await self._create_mesh_primitives(["Cube"])
        meshPrim = newMesh.GetPrim()
        meshPath = newMesh.GetPath()

        # scale down to avoid long runtime during sampling
        scaleVec = Gf.Vec3d(0.2, 0.2, 0.2)
        self.set_prim_scale(meshPrim, scaleVec)
        
        # make sure adding a new mesh with a sampler on the same path will NOT add a new particle set
        omni.kit.commands.execute("AddParticleSamplingCommand", prim=meshPrim)
        await self._wait_cooking_finished()
        await self.check_API_and_particles(meshPath)

        # get the new particles prim
        samplingAPI = PhysxSchema.PhysxParticleSamplingAPI(meshPrim)
        particlesTargets = samplingAPI.GetParticlesRel().GetTargets()
        particlePrimPath = particlesTargets[0]

        # should sample into the same prim
        self.assertEqual(particlePrimPath, self._particle_path)

        # compare sizes again - should be more again.
        pointsOld = UsdGeom.Points.Get(self._stage, self._particle_path)
        pointPositionsOld = pointsOld.GetPointsAttr().Get()
        numPointsOld1 = len(pointPositionsOld)
        self.assertGreater(numPointsOld1, self._num_points)

        return

        # AD: the following crashes hydra from some reason, not sure why

        # delete both meshes - the prim will not be removed
        self._stage.RemovePrim(self._mesh_path_1)
        self._stage.RemovePrim(meshPath)

        # check that it is removed - need to wait for changes to be applied.
        await self._wait_cooking_finished()
        particlePrim = self._stage.GetPrimAtPath(particlePrimPath)
        self.assertTrue(particlePrim)

        # make sure point count is 0.
        pointsEmpty = UsdGeom.Points.Get(self._stage, particlePrimPath)
        pointPositionsEmpty = pointsEmpty.GetPointsAttr().Get()
        numPointsEmpty = len(pointPositionsEmpty)
        self.assertEqual(0, numPointsEmpty)
        

    # OM-46517 - delete the relationship and add a relationship to the same 
    # particle prim again
    #@unittest.skip("OM-49894")
    async def test_particleSampling_delete_particlesRel(self):
        await self.run_particleSampling_delete_relationship()

        points = UsdGeom.Points.Get(self._stage, self._particle_path)
        pointPositions = points.GetPointsAttr().Get()
        numPoints1 = len(pointPositions)

        # change something in the sampler - nothing should happen
        samplingAPI = PhysxSchema.PhysxParticleSamplingAPI(self._mesh_prim)
        samplingAPI.CreateSamplingDistanceAttr().Set(7.5)
        await self._wait_cooking_finished()

        pointPositions = points.GetPointsAttr().Get()
        numPoints2 = len(pointPositions)

        self.assertEqual(numPoints1, numPoints2)

        # make sure readding the rel will update things again
        samplingAPI.CreateParticlesRel().SetTargets([self._particle_path])

        await self._wait_cooking_finished()
        await self.check_API_and_particles(self._mesh_path)

        numPoints3 = self.get_number_of_points(self._mesh_path)
        self.assertGreater(numPoints3, numPoints2)

    # OM-46517 - delete the relationship and add a relationship to an invalid
    # prim (not a particle prim) and then a new valid prim and check things.
    #@unittest.skip("OM-49894")
    async def test_particleSampling_delete_particlesRel_new_particles(self):
        await self.run_particleSampling_delete_relationship()

        points = UsdGeom.Points.Get(self._stage, self._particle_path)
        pointPositions = points.GetPointsAttr().Get()
        p1 = pointPositions[0]
        numPoints1 = len(pointPositions)

        # change something in the sampler - nothing should happen
        samplingAPI = PhysxSchema.PhysxParticleSamplingAPI(self._mesh_prim)
        samplingAPI.CreateSamplingDistanceAttr().Set(7.5)

        pointPositions = points.GetPointsAttr().Get()
        numPoints2 = len(pointPositions)

        self.assertEqual(numPoints1, numPoints2)

        # add a relationship to an non-particle prim
        newPath = Sdf.Path("/invalidParticles")
        newPrim = UsdGeom.Points.Define(self._stage, newPath)
        samplingAPI.CreateParticlesRel().SetTargets([newPath])
        await self._wait_cooking_finished()

        # shouldn't sample anything
        newPoints = newPrim.GetPointsAttr().Get()
        self.assertFalse(newPoints)

        # add a relationship to a real particle prim
        newPath2 = Sdf.Path("/validParticles")
        newPrim2 = UsdGeom.Points.Define(self._stage, newPath2)
        particleSetAPI = PhysxSchema.PhysxParticleSetAPI.Apply(newPrim2.GetPrim())
        particleAPI = PhysxSchema.PhysxParticleAPI(newPrim2.GetPrim())
        particleAPI.CreateParticleSystemRel().SetTargets([Sdf.Path("/particleSystem0")])
        self.assertTrue(particleAPI)

        # triggers a resampling
        samplingAPI.CreateParticlesRel().SetTargets([newPath2])
        await self._wait_cooking_finished()

        pointPositions = newPrim2.GetPointsAttr().Get()
        numPoints3 = len(pointPositions)

        # check that we resampled and that things changed on the new valid set
        self.assertGreater(numPoints3, 0)
        self.assertNotEqual(numPoints3, numPoints2)

        pointPositions = points.GetPointsAttr().Get()
        p4 = pointPositions[0]
        numPoints4 = len(pointPositions)

        # check that nothing happened on the old set
        self.assertEqual(numPoints1, numPoints4)

    # OM-46517
    #@unittest.skip("OM-49894")
    async def test_particleSampling_delete_particles(self):
        omni.kit.commands.execute("AddParticleSamplingCommand", prim=self._mesh_prim)
        await self._wait_cooking_finished()
        await self.check_API_and_particles(self._mesh_path)

        # delete the particles
        samplingAPI = PhysxSchema.PhysxParticleSamplingAPI(self._mesh_prim)
        particlesTargets = samplingAPI.GetParticlesRel().GetTargets()
        particlePrimPath = particlesTargets[0]

        self._stage.RemovePrim(particlePrimPath)
        await omni.kit.app.get_app().next_update_async() 

        # check that the particles are gone
        self.assertFalse(self._stage.GetPrimAtPath(particlePrimPath))

        # change the sampling distance
        samplingAPI.CreateSamplingDistanceAttr().Set(7.5)
        await self._wait_cooking_finished()

        # check that particles are recreated.
        self.assertTrue(self._stage.GetPrimAtPath(particlePrimPath))
        points = UsdGeom.Points(self._stage.GetPrimAtPath(particlePrimPath))
        positions = points.GetPointsAttr().Get()
        self.assertTrue(len(positions) > 0)

    # OM-36091
    #@unittest.skip("OM-49894")
    async def test_particleSampling_save_restore(self):
        filename = "SamplingResults.usda"

        omni.kit.commands.execute("AddParticleSamplingCommand", prim=self._mesh_prim)
        await self._wait_cooking_finished()
        await self.check_API_and_particles(self._mesh_path)

        # run for a few moments
        self.step(num_steps=20)

        # get point positions
        samplingAPI = PhysxSchema.PhysxParticleSamplingAPI(self._mesh_prim)
        particlesTargets = samplingAPI.GetParticlesRel().GetTargets()
        particlePrimPath = particlesTargets[0]
        particlePrim = self._stage.GetPrimAtPath(particlePrimPath)
        self.assertTrue(particlePrim)

        points1 = UsdGeom.Points.Get(self._stage, particlePrimPath)
        pointPositions1 = points1.GetPointsAttr().Get()
        numPoints1 = len(pointPositions1)

        # set point 0 to some fixed value - if resampling would happen, this would change things back.
        pointPositions1[0] = Gf.Vec3f(50.0, 50.0, 50.0)
        points1.CreatePointsAttr().Set(pointPositions1)

        # save
        self._stage.Export(filename)

        # make sure the saving does not lead to a resampling
        await self._wait_cooking_finished()

        points2 = UsdGeom.Points.Get(self._stage, particlePrimPath)
        pointPositions2 = points2.GetPointsAttr().Get()
        numPoints2 = len(pointPositions2)

        self.assertEqual(numPoints1, numPoints2)
        self.assertEqual(pointPositions1, pointPositions2)
        
        # run some more
        self.step(num_steps=20)

        # check that it ran
        points3 = UsdGeom.Points.Get(self._stage, particlePrimPath)
        pointPositions3 = points3.GetPointsAttr().Get()
        numPoints3 = len(pointPositions3)

        self.assertNotEqual(pointPositions3, pointPositions2)
        
        # load the saved file
        await omni.usd.get_context().open_stage_async(filename)
        new_stage = omni.usd.get_context().get_stage()
        
        # wait to check if opening does not lead to a resampling
        await self._wait_cooking_finished()

        # check if points are still in saved state
        points4 = UsdGeom.Points.Get(new_stage, particlePrimPath)
        pointPositions4 = points4.GetPointsAttr().Get()
        numPoints4 = len(pointPositions4)

        self.assertEqual(pointPositions4, pointPositions1)
        self.assertEqual(pointPositions4, pointPositions2)
        self.assertNotEqual(pointPositions4, pointPositions3)

        # make sure triggering a resampling works and samples something
        samplingAPI = PhysxSchema.PhysxParticleSamplingAPI(new_stage.GetPrimAtPath(self._mesh_path))
        samplingAPI.CreateSamplingDistanceAttr().Set(9.0)
        await self._wait_cooking_finished()

        points5 = UsdGeom.Points.Get(new_stage, particlePrimPath)
        pointPositions5 = points5.GetPointsAttr().Get()
        numPoints5 = len(pointPositions5)

        self.assertNotEqual(numPoints5, numPoints4)
        self.assertNotEqual(pointPositions5, pointPositions4)

        # cleanup the file save
        os.remove(filename)

    # OM-47658
    #@unittest.skip("OM-49894")
    async def test_particleSampling_sample_cubes(self): 

        # create point instancer with cubes
        particleInstancerPath = Sdf.Path("/particles")
        cubePath = particleInstancerPath.AppendElementString("proto")
        protoCube = UsdGeom.Cube.Define(self._stage, cubePath)

        instancer = UsdGeom.PointInstancer.Define(self._stage, particleInstancerPath)
        protoList = instancer.GetPrototypesRel()
        protoList.AddTarget(Sdf.Path(cubePath))
        instancer.GetProtoIndicesAttr().Set([0])
        instancer.GetPositionsAttr().Set([Gf.Vec3f(0.0, 0.0, 1.0)])

        particle_api = PhysxSchema.PhysxParticleAPI.Apply(instancer.GetPrim())
        particle_api.CreateParticleSystemRel().SetTargets([self._particle_system_path])

        # add the particle sampler
        fluid_rest_offset = 2.0 * 0.9 * 0.6
        particle_sampler_distance = 2.0 * fluid_rest_offset

        # reference the particle set in the sampling api
        sampling_api = PhysxSchema.PhysxParticleSamplingAPI.Apply(self._mesh_prim)
        sampling_api.CreateParticlesRel().AddTarget(particleInstancerPath)
        sampling_api.CreateSamplingDistanceAttr().Set(particle_sampler_distance)
        sampling_api.CreateMaxSamplesAttr().Set(5e5)
        sampling_api.CreateVolumeAttr().Set(True)

        # check if things are correct
        await self._wait_cooking_finished()
        await self.check_API_and_particles(self._mesh_path, pointInstancer = True)

        newInstancer = UsdGeom.PointInstancer.Get(self._stage, particleInstancerPath)
        self.assertTrue(newInstancer)
        instancerPrototypes = newInstancer.GetPrototypesRel().GetTargets()
        newCube = UsdGeom.Cube.Get(self._stage, instancerPrototypes[0])
        self.assertTrue(newCube)

    async def run_particleSampling_wait_at_sim_start(self, wait_one_cycle=False, explicit_setup=False):

        # tests awaiting particle sampling at sim start:
        #
        # because of how the implementation generates the particle prims, there is a possible case where
        # the particle sampling API is set on a prim, but the "particles" relationship target does not exist
        # yet. This happens when the simulation is started less than one Kit update cycle after the sampling
        # API is applied to the prim. In this case, we issue a warning and the sampling fails.
        #
        # We cover 3 cases:
        #
        # wait_one_cycle = True: the simulation starts more than 1 kit update cycle after the sampling is defined, everything
        #                        should be setup correctly and the sim start awaits the sampling
        # wait_one_cycle = False: the simulation starts immediately, don't sample and issue a warning if the particle prim
        #                        does not exist yet
        # explicit_setup:        the user has set up the particle prim and the relationships, it will work even if the sim
        #                        starts immediately.

        if (explicit_setup):
            # construct the particle prim
            particlePath = Sdf.Path("/particles")
            particlePrim = UsdGeom.Points.Define(self._stage, particlePath)
            particleSetAPI = PhysxSchema.PhysxParticleSetAPI.Apply(particlePrim.GetPrim())
            particleAPI = PhysxSchema.PhysxParticleAPI(particlePrim.GetPrim())
            particleAPI.CreateParticleSystemRel().SetTargets([Sdf.Path("/particleSystem0")])

            # set the API & relationship on the target
            particleSamplingAPI = PhysxSchema.PhysxParticleSamplingAPI.Apply(self._mesh_prim)
            particleSamplingAPI.CreateParticlesRel().SetTargets([particlePath])
        else:
            omni.kit.commands.execute("AddParticleSamplingCommand", prim=self._mesh_prim)

        # need one update cycle for authoring to pick up the task
        if wait_one_cycle:
            await omni.kit.app.get_app().next_update_async()

        # play immediately - should wait for sampling.
        omni.timeline.get_timeline_interface().play()
        for i in range(1):
            await omni.kit.app.get_app().next_update_async()

        if wait_one_cycle or explicit_setup: # wait one cycle - stuff should show up
            await self.check_API_and_particles(self._mesh_path)
            numPoints = self.get_number_of_points(self._mesh_path)
            self.assertGreater(numPoints, 0)
        else: # nothing should show up
            mesh_prim = self._stage.GetPrimAtPath(self._mesh_path)
            samplingApi = PhysxSchema.PhysxParticleSamplingAPI(mesh_prim)
            pointTargets = samplingApi.GetParticlesRel().GetTargets()
            self.assertEqual(len(pointTargets), 0)            

    # OM-49368 - make sure sim start waits for particle sampler
    #@unittest.skip("OM-49894")
    async def test_particleSampling_wait_at_sim_start(self):
        await self.run_particleSampling_wait_at_sim_start(wait_one_cycle=True)

    #@unittest.skip("OM-49894")
    async def test_particleSampling_wait_at_sim_start_immediately(self):
        message = str(self._mesh_path) + ": particle sampler does not point to a valid particle prim, needs to be set before stage parsing starts."
        with utils.ExpectMessage(self, message):
            await self.run_particleSampling_wait_at_sim_start(wait_one_cycle=False)

    #@unittest.skip("OM-49894")
    async def test_particleSampling_wait_at_sim_start_user_setup(self):
        await self.run_particleSampling_wait_at_sim_start(wait_one_cycle=False, explicit_setup=True)

    async def run_inconsistent_particle_prim_test(self, change_velocities):
        # setup first cube & sample
        omni.kit.commands.execute("AddParticleSamplingCommand", prim=self._mesh_prim)
        await self._wait_cooking_finished()
        await self.check_API_and_particles(self._mesh_path)

        first_point_count = self.get_number_of_points(self._mesh_path)

        # create second mesh
        (self._mesh_1,) = await self._create_mesh_primitives(["Cube"])
        self._mesh_prim_1 = self._mesh_1.GetPrim()
        self._mesh_path_1 = self._mesh_1.GetPath()

        # scale down to avoid long runtime during sampling
        self.set_prim_scale(self._mesh_prim_1, self._scale_vec)

        # add sampling
        omni.kit.commands.execute("AddParticleSamplingCommand", prim=self._mesh_prim_1)
        await self._wait_cooking_finished()
        await self.check_API_and_particles(self._mesh_path_1)

        second_point_count_with_first = self.get_number_of_points(self._mesh_path_1)
        second_point_count = second_point_count_with_first - first_point_count


        #create third mesh
        (self._mesh_2,) = await self._create_mesh_primitives(["Cube"])
        self._mesh_prim_2 = self._mesh_2.GetPrim()
        self._mesh_path_2 = self._mesh_2.GetPath()

        # scale down to avoid long runtime during sampling
        self.set_prim_scale(self._mesh_prim_2, self._scale_vec)

        # add sampling
        omni.kit.commands.execute("AddParticleSamplingCommand", prim=self._mesh_prim_2)
        await self._wait_cooking_finished()
        await self.check_API_and_particles(self._mesh_path_2)

        # get the particle prim
        mesh_prim = self._stage.GetPrimAtPath(self._mesh_path_1)
        samplingApi = PhysxSchema.PhysxParticleSamplingAPI(mesh_prim)
        pointTargets = samplingApi.GetParticlesRel().GetTargets()

        particlePath = pointTargets[0]
        pointsPrim = self._stage.GetPrimAtPath(particlePath)
        points = UsdGeom.Points(pointsPrim)
        positions = points.GetPointsAttr().Get()
        initial_point_count = len(positions)

       

        # mess with the velocities/points count of particle prim
        if change_velocities:
            velocities = Vt.Vec3fArray([])
            points.CreateVelocitiesAttr().Set(velocities)
        else:
            positions = Vt.Vec3fArray([])
            points.CreatePointsAttr().Set(positions)

        await self._wait_cooking_finished()

        message = str(particlePath) + ": Physx particle sampling - target particle prim has a different point count than expected! Trying to recover.."
        with utils.ExpectMessage(self, message):

            # resample the middle mesh
            new_scale_vec = self._scale_vec - Gf.Vec3d(0.1, 0.1, 0.1)
            self.set_prim_scale(self._mesh_prim_1, new_scale_vec)

            await self._wait_cooking_finished()

        # check that we now only have the points of the middle mesh
        new_point_count = self.get_number_of_points(self._mesh_path_1)
        self.assertLess(new_point_count, initial_point_count)

        # we scaled down the second mesh, so we expect fewer points
        self.assertLess(new_point_count, second_point_count)

        # resample the first mesh.
        self.set_prim_scale(self._mesh_prim, new_scale_vec)

        await self._wait_cooking_finished()

        latest_point_count = self.get_number_of_points(self._mesh_path_1)
        self.assertGreater(latest_point_count, new_point_count)

    # OM-108064, OM-98647
    #
    # mostly checks that we do not crash if something other than the particle authoring system messes
    # with the data in the particle prim we sample into. We mess with the velocities/points arrays
    # when multiple samplers are sampling into the same prim. Any inconsistencies should result in
    # the particle prim being cleared and only the mesh being currently resampled being part of the 
    # points list. After that, forcing resamples on the other meshes should make things append again.
    async def test_particleSampling_inconsistent_point_count_velocities(self):
        await self.run_inconsistent_particle_prim_test(True)

    async def test_particleSampling_inconsistent_point_count_points(self):
        await self.run_inconsistent_particle_prim_test(True)
