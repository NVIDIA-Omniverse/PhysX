# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.app
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema, Sdf
from omni.physxtests import utils
from omni.physxcommands import AddGroundPlaneCommand
from omni.physx.scripts import physicsUtils, particleUtils, deformableUtils
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
import carb
import omni.physx.bindings._physx as pb
import os
import platform
import unittest


class ParticleVisualizationTestKitStage(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Core

    num_particles_per_dim = 4
    cloth_resolution = 5

    # runs before each test case
    async def setUp(self):
        await super().setUp()
        await self.base_setup()

    # runs after each test case and runs base_terminate if setup was called:
    async def tearDown(self):
        for setting, value in self._settings_restore_data.items():
            self._settings.set(setting, value)
        if self._save_file is not None:
            os.remove(self._save_file)
            self._save_file = None
        await super().tearDown()

    async def base_setup(self):
        # save all settings for later restore
        # no need to save/restore non persistent settings as they are reset with a new stage
        settings_to_restore = [
            pb.SETTING_DISPLAY_PARTICLES,
            pb.SETTING_DISPLAY_PARTICLES_SHOW_DIFFUSE,
            pb.SETTING_DISPLAY_PARTICLES_POSITION_TYPE,
            pb.SETTING_DISPLAY_PARTICLES_RADIUS_TYPE,
            pb.SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_MESH_LINES,
            pb.SETTING_DISPLAY_PARTICLES_SHOW_PARTICLE_SET_PARTICLES,
            pb.SETTING_DISPLAY_PARTICLES_SHOW_FLUID_SURFACE,
            pb.SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_PARTICLES,
            pb.SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_MESH,
        ]
        self._settings = carb.settings.get_settings()
        self._settings_restore_data = {}
        for setting in settings_to_restore:
            self._settings_restore_data[setting] = self._settings.get(setting)

        # show all debug viz as default setting for tests
        self._settings.set(pb.SETTING_DISPLAY_PARTICLES, pb.VisualizerMode.NONE)
        # otherwise set defaults
        self._settings.set(pb.SETTING_DISPLAY_PARTICLES_SHOW_DIFFUSE, False)
        self._settings.set(
            pb.SETTING_DISPLAY_PARTICLES_POSITION_TYPE, pb.ParticleVisualizationPositionType.SIM_POSITIONS
        )
        self._settings.set(
            pb.SETTING_DISPLAY_PARTICLES_RADIUS_TYPE, pb.ParticleVisualizationRadiusType.PARTICLE_CONTACT_OFFSET
        )  # particle contact offset
        self._settings.set(pb.SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_MESH_LINES, False)
        self._settings.set(pb.SETTING_DISPLAY_PARTICLES_SHOW_PARTICLE_SET_PARTICLES, True)
        self._settings.set(pb.SETTING_DISPLAY_PARTICLES_SHOW_FLUID_SURFACE, False)
        self._settings.set(pb.SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_PARTICLES, True)
        self._settings.set(pb.SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_MESH, False)
        # make sure updates to usd are enabled
        self.fail_on_log_error = True
        self._stage = await utils.new_stage_setup()
        self._settings.set(pb.SETTING_UPDATE_TO_USD, True)
        self._settings.set(pb.SETTING_UPDATE_PARTICLES_TO_USD, True)

        default_prim_xform = UsdGeom.Xform.Define(self._stage, "/World")
        self._stage.SetDefaultPrim(default_prim_xform.GetPrim())

        UsdGeom.SetStageUpAxis(self._stage, "Y")
        self._up_axis = UsdGeom.GetStageUpAxis(self._stage)
        self._default_prim_path = self._stage.GetDefaultPrim().GetPath()
        physicsScenePath = self._default_prim_path.AppendChild("physicsScene")
        self._scene = UsdPhysics.Scene.Define(self._stage, physicsScenePath)
        physxAPI = PhysxSchema.PhysxSceneAPI.Apply(self._scene.GetPrim())
        physxAPI.CreateSolverTypeAttr("TGS")

        self._pbd_material_path = self._default_prim_path.AppendChild("PBDMaterial")
        particleUtils.add_pbd_particle_material(self._stage, self._pbd_material_path)
        self._pbd_material_api = PhysxSchema.PhysxPBDMaterialAPI.Get(self._stage, self._pbd_material_path)

        self._particle_system_path = self._default_prim_path.AppendChild("particleSystem")

        self._offsets = {
            "contact": 5.1,
            "rest": 2.5,
            "particleContact": 5.0,
            "fluidRest": 2.3,
            "solidRest": 2.4,
        }
        self._render_radius = 2.2

        # save load file:
        self._save_file = None

        self._particle_system = particleUtils.add_physx_particle_system(
            stage=self._stage,
            particle_system_path=self._particle_system_path,
            simulation_owner=self._scene.GetPath(),
            contact_offset=self._offsets["contact"],
            rest_offset=self._offsets["rest"],
            particle_contact_offset=self._offsets["particleContact"],
            fluid_rest_offset=self._offsets["fluidRest"],
            solid_rest_offset=self._offsets["solidRest"],
        )

        physicsUtils.add_physics_material_to_prim(self._stage, self._particle_system.GetPrim(), self._pbd_material_path)

        self.add_groundplane()

    def add_groundplane(self):
        AddGroundPlaneCommand.execute(
            self._stage, "/CollisionPlane", self._up_axis, 1000.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5)
        )

    def _save_stage_to_file(self):
        # save root layer only (avoid flattening session layer into save file)
        self._save_file = "particle_debug_viz_test_temp_file.usda"
        self._stage.GetRootLayer().Export(self._save_file)

    async def _load_stage_from_file(self):
        self.assertIsNotNone(self._save_file)
        await omni.usd.get_context().open_stage_async(self._save_file)
        self._stage = omni.usd.get_context().get_stage()

    def create_particle_set_points(
        self,
        path,
        initial_point=Gf.Vec3f(0.0, 1.0, 0.0),
        num_particles_per_dim=None,
        particle_spacing=5.0,
        fluid=True,
    ) -> UsdGeom.Points:

        if num_particles_per_dim is None:
            num_particles_per_dim = self.num_particles_per_dim

        positions, velocities = particleUtils.create_particles_grid(
            initial_point, particle_spacing, num_particles_per_dim, num_particles_per_dim, num_particles_per_dim
        )

        widths = [self._render_radius * 2.0] * len(positions)
        particleSet = particleUtils.add_physx_particleset_points(
            stage=self._stage,
            path=path,
            positions_list=positions,
            velocities_list=velocities,
            widths_list=widths,
            particle_system_path=self._particle_system_path,
            self_collision=True,
            fluid=fluid,
            particle_group=0,
            particle_mass=1.0,
            density=0.02,
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
        path,
        initial_point=Gf.Vec3f(0.0, 1.0, 0.0),
        num_particles_per_dim=None,
        particle_spacing=5.0,
        fluid=True,
    ) -> UsdGeom.PointInstancer:

        if num_particles_per_dim is None:
            num_particles_per_dim = self.num_particles_per_dim

        positions, velocities = particleUtils.create_particles_grid(
            initial_point, particle_spacing, num_particles_per_dim, num_particles_per_dim, num_particles_per_dim
        )

        particleSet = particleUtils.add_physx_particleset_pointinstancer(
            stage=self._stage,
            path=path,
            positions=positions,
            velocities=velocities,
            particle_system_path=self._particle_system_path,
            self_collision=True,
            fluid=fluid,
            particle_group=0,
            particle_mass=1.0,
            density=0.02,
        )

        prototypeStr = str(path) + "/particlePrototype0"
        gprim = UsdGeom.Sphere.Define(self._stage, Sdf.Path(prototypeStr))
        gprim.CreateRadiusAttr().Set(self._render_radius)

        instancerPrim = self._stage.GetPrimAtPath(path)
        points = UsdGeom.PointInstancer(instancerPrim)
        self.assertTrue(points)

        pointPositions = points.GetPositionsAttr().Get()
        self.assertTrue(pointPositions)
        self.assertEqual(len(pointPositions), len(positions))

        return particleSet

    def is_shown(self, path: Sdf.Path):
        imageable = UsdGeom.Imageable.Get(self._stage, path)
        if imageable:
            purpose = imageable.GetPurposeAttr().Get()
            return purpose == "default"
        else:
            return False

    def add_cloth(self, resolution=None):
        if resolution is None:
            resolution = self.cloth_resolution
        # Create grid mesh
        self._cloth_mesh_path = self._default_prim_path.AppendChild("cloth")
        self._cloth_mesh = UsdGeom.Mesh.Define(self._stage, self._cloth_mesh_path)
        tri_points, tri_indices = deformableUtils.create_triangle_mesh_square(
            dimx=resolution, dimy=resolution, scale=self._offsets["solidRest"] * resolution * 2.0
        )
        self._cloth_mesh.GetPointsAttr().Set(tri_points)
        self._cloth_mesh.GetFaceVertexIndicesAttr().Set(tri_indices)
        self._cloth_mesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))

        particleUtils.add_physx_particle_cloth(
            stage=self._stage,
            path=self._cloth_mesh_path,
            dynamic_mesh_path=None,
            particle_system_path=self._particle_system_path,
        )

        particleMass = 0.02
        massApi = UsdPhysics.MassAPI.Apply(self._cloth_mesh.GetPrim())
        massApi.GetMassAttr().Set(particleMass * len(tri_points))

    def _get_session_layer_xform_path(self, path: Sdf.Path) -> Sdf.Path:
        session_base_path = Sdf.Path("/PhysxProxiesVisualization")
        path_mangled = path.pathString.replace("/", "_")
        return session_base_path.AppendChild(path_mangled)

    def _get_solid_viz_path(self):
        return self._get_session_layer_xform_path(self._solid_particles_path).AppendChild("particle")

    def _get_fluid_viz_path(self):
        return self._get_session_layer_xform_path(self._fluid_particles_path).AppendChild("particle")

    def _get_cloth_particles_viz_path(self):
        return self._get_session_layer_xform_path(self._cloth_mesh_path).AppendChild("particle")

    def _get_cloth_mesh_viz_path(self):
        return self._get_session_layer_xform_path(self._cloth_mesh_path).AppendChild("mesh")

    def _get_isosurface_viz_path(self):
        return self._get_session_layer_xform_path(self._particle_system_path).AppendChild("surface")

    def _get_isosurface_path(self):
        return self._particle_system_path.AppendElementString("Isosurface")

    def _get_positions_from_pointbased_or_pointinstancer(self, points_or_point_instancer_path: Sdf.Path):
        pointBased = UsdGeom.PointBased.Get(self._stage, points_or_point_instancer_path)
        pointInstancer = UsdGeom.PointInstancer.Get(self._stage, points_or_point_instancer_path)
        if pointInstancer:
            self.assertTrue(pointInstancer.GetPositionsAttr())
            return pointInstancer.GetPositionsAttr().Get()

        if pointBased:
            return pointBased.GetPointsAttr().Get()

        # no valid points passed in
        self.assertTrue(False)

    def _are_positions_equal(self, viz_path: Sdf.Path, particles_path: Sdf.Path) -> bool:
        viz_pos = self._get_positions_from_pointbased_or_pointinstancer(viz_path)
        particle_pos = self._get_positions_from_pointbased_or_pointinstancer(particles_path)
        are_equal = True
        for v, p in zip(viz_pos, particle_pos):
            are_equal = are_equal and (v == p)
        return are_equal

    def _are_vecs_almost_equal(self, veca, vecb, eps):
        almost_equal = True
        for a, b in zip(veca, vecb):
            are_close = abs(a - b) <= eps
            almost_equal = almost_equal and are_close
            # if not are_close:
            #     print(f"NOT CLOSE WITH A = {a}, B = {b} and abs(delta) = {abs(a - b)}")
        return almost_equal

    def _are_positions_almost_equal(self, viz_path: Sdf.Path, particles_path: Sdf.Path, eps=0.0001) -> bool:
        viz_pos = self._get_positions_from_pointbased_or_pointinstancer(viz_path)
        particle_pos = self._get_positions_from_pointbased_or_pointinstancer(particles_path)
        are_equal = True
        for v, p in zip(viz_pos, particle_pos):
            are_equal = are_equal and self._are_vecs_almost_equal(v, p, eps)
        return are_equal

    def _are_uniform_scale(self, points_or_point_instancer_path: Sdf.Path) -> bool:
        points = UsdGeom.Points.Get(self._stage, points_or_point_instancer_path)
        are_uniform_scale = True
        if points:
            if points.GetWidthsAttr():
                widths = points.GetWidthsAttr().Get()
                if len(widths) > 0:
                    w0 = widths[0]
                    for w in widths:
                        are_uniform_scale = are_uniform_scale and (w0 == w)
            return are_uniform_scale

        pointInstancer = UsdGeom.PointInstancer.Get(self._stage, points_or_point_instancer_path)

        self.assertTrue(pointInstancer)
        scalesAttr = pointInstancer.GetScalesAttr()
        if not scalesAttr:
            return True  # scales are uniform if the attributes is not set

        points = pointInstancer.GetPositionsAttr().Get()
        self.assertGreater(len(points), 0)
        scales = scalesAttr.Get()
        if not scales:
            return True  # scales are uniform if the attributes is empty
        self.assertEqual(len(scales), len(points))
        firstScale = scales[0]
        are_uniform_scale = are_uniform_scale and (firstScale[0] == firstScale[1])
        are_uniform_scale = are_uniform_scale and (firstScale[0] == firstScale[2])

        for s in scales:
            are_uniform_scale = are_uniform_scale and (s == firstScale)
        return are_uniform_scale

    def _get_radius(self, points_or_point_instancer_path: Sdf.Path) -> float:
        points = UsdGeom.Points.Get(self._stage, points_or_point_instancer_path)
        pointInstancer = UsdGeom.PointInstancer.Get(self._stage, points_or_point_instancer_path)

        haveDebugParticles = points or pointInstancer
        self.assertTrue(haveDebugParticles)

        radius = -1
        if points:
            radius = points.GetWidthsAttr().Get()[0] * 0.5
        else:
            sphere_path = points_or_point_instancer_path.AppendChild("prototype")
            sphereGeom = UsdGeom.Sphere.Get(self._stage, sphere_path)
            radius = sphereGeom.GetRadiusAttr().Get()
        return radius

    def _check_mesh_extents_equal(self, meshA: UsdGeom.Mesh, meshB: UsdGeom.Mesh):
        self.assertTrue(meshA)
        self.assertTrue(meshB)
        extentsA = self.get_extents(meshA)
        extentsB = self.get_extents(meshB)
        self.check_extents_same(extentsA, extentsB)

    def _setup_particle_objects(self, fluidPointInstancer=False, solidPointInstancer=False):
        self._fluid_particles_path = self._default_prim_path.AppendChild("fluidParticles")
        self._solid_particles_path = self._default_prim_path.AppendChild("solidParticles")

        fluid_init = Gf.Vec3f(-150.0, 100.0, 0.0)
        solid_init = Gf.Vec3f(0.0, 100.0, 0.0)
        cloth_location = Gf.Vec3f(200.0, 100.0, 0.0)

        if fluidPointInstancer:
            self._fluid_particles = self.create_particle_set_pointinstancer(
                path=self._fluid_particles_path,
                initial_point=fluid_init,
                fluid=True,
            )
        else:
            self._fluid_particles = self.create_particle_set_points(
                path=self._fluid_particles_path,
                initial_point=fluid_init,
                fluid=True,
            )

        if solidPointInstancer:
            self._solid_particles = self.create_particle_set_pointinstancer(
                path=self._solid_particles_path,
                initial_point=solid_init,
                fluid=False,
            )
        else:
            self._solid_particles = self.create_particle_set_points(
                path=self._solid_particles_path,
                initial_point=solid_init,
                fluid=False,
            )

        self.add_cloth()
        self._cloth_mesh.AddTranslateOp().Set(cloth_location)

    def _get_expected_fluid_radius(self, radius_type: pb.ParticleVisualizationRadiusType) -> float:
        if radius_type == pb.ParticleVisualizationRadiusType.CONTACT_OFFSET:
            return self._offsets["contact"]
        elif radius_type == pb.ParticleVisualizationRadiusType.REST_OFFSET:
            return self._offsets["rest"]
        elif radius_type == pb.ParticleVisualizationRadiusType.PARTICLE_CONTACT_OFFSET:
            return self._offsets["particleContact"]
        elif radius_type == pb.ParticleVisualizationRadiusType.PARTICLE_REST_OFFSET:
            return self._offsets["fluidRest"]
        elif radius_type == 4:
            self.assertTrue(False, "Get expected radius not supported for anisotropy radius type")
        elif radius_type == 5:
            self.assertTrue(False, "Get expected radius not supported for render geom radius type")
        return None

    def _check_particle_positions(self, fluidEqual=True, solidEqual=True, clothEqual=True):
        if fluidEqual:
            self.assertTrue(self._are_positions_equal(self._get_fluid_viz_path(), self._fluid_particles_path))
        else:
            self.assertFalse(self._are_positions_equal(self._get_fluid_viz_path(), self._fluid_particles_path))

        if solidEqual:
            self.assertTrue(self._are_positions_equal(self._get_solid_viz_path(), self._solid_particles_path))
        else:
            self.assertFalse(self._are_positions_equal(self._get_solid_viz_path(), self._solid_particles_path))

        if clothEqual:
            self.assertTrue(self._are_positions_equal(self._get_cloth_particles_viz_path(), self._cloth_mesh_path))
        else:
            self.assertFalse(self._are_positions_equal(self._get_cloth_particles_viz_path(), self._cloth_mesh_path))

    def _check_particle_offsets_and_visibility(self, offset_id):
        if offset_id == pb.ParticleVisualizationRadiusType.RENDER_GEOMETRY:
            # render geom must be visible, and debug invisible:
            self.assertFalse(self.is_shown(self._get_fluid_viz_path()))
            self.assertTrue(self.is_shown(self._fluid_particles_path))

            self.assertFalse(self.is_shown(self._get_solid_viz_path()))
            self.assertTrue(self.is_shown(self._fluid_particles_path))

            self.assertFalse(self.is_shown(self._get_cloth_particles_viz_path()))
            self.assertTrue(self.is_shown(self._cloth_mesh_path))
            return

        # otherwise check radii and debug viz visibility
        fluid_radius = self._get_expected_fluid_radius(offset_id)
        solid_radius = fluid_radius
        if offset_id == pb.ParticleVisualizationRadiusType.PARTICLE_REST_OFFSET:
            solid_radius = self._offsets["solidRest"]

        places = 5
        self.assertAlmostEqual(fluid_radius, self._get_radius(self._get_fluid_viz_path()), places)
        self.assertTrue(self.is_shown(self._get_fluid_viz_path()))
        self.assertFalse(self.is_shown(self._fluid_particles_path))
        self.assertTrue(self._are_uniform_scale(self._get_fluid_viz_path()))

        self.assertAlmostEqual(solid_radius, self._get_radius(self._get_solid_viz_path()), places)
        self.assertTrue(self.is_shown(self._get_solid_viz_path()))
        self.assertFalse(self.is_shown(self._solid_particles_path))
        self.assertTrue(self._are_uniform_scale(self._get_solid_viz_path()))

        self.assertAlmostEqual(solid_radius, self._get_radius(self._get_cloth_particles_viz_path()), places)
        self.assertTrue(self.is_shown(self._get_cloth_particles_viz_path()))
        self.assertFalse(self.is_shown(self._cloth_mesh_path))
        self.assertTrue(self._are_uniform_scale(self._get_cloth_particles_viz_path()))

        # and ensure positions the same:
        self._check_particle_positions(fluidEqual=True, solidEqual=True, clothEqual=True)

    #(OM-109132)
    async def test_particle_viz_set_offsets_sim(self):
        for offset_id in [
            pb.ParticleVisualizationRadiusType.CONTACT_OFFSET,
            pb.ParticleVisualizationRadiusType.REST_OFFSET,
            pb.ParticleVisualizationRadiusType.PARTICLE_CONTACT_OFFSET,
            pb.ParticleVisualizationRadiusType.PARTICLE_REST_OFFSET,
            pb.ParticleVisualizationRadiusType.RENDER_GEOMETRY,
        ]:
            with self.subTest(offset_id=offset_id):
                self._setup_particle_objects()
                self._settings.set(pb.SETTING_DISPLAY_PARTICLES_RADIUS_TYPE, offset_id)
                self._settings.set(pb.SETTING_DISPLAY_PARTICLES, pb.VisualizerMode.ALL)

                await self.wait(5)
                self._check_particle_offsets_and_visibility(offset_id=offset_id)

                # run a sim step, check that equal, and visibility is still correct:
                await self.step(1)
                await self.wait(5)
                self._check_particle_offsets_and_visibility(offset_id=offset_id)

                # step again and reset to test reset correctness
                await self.step(1, stop_timeline_after=True)
                await self.wait(5)
                self._check_particle_offsets_and_visibility(offset_id=offset_id)

            await self.tearDown()
            await self.setUp()

    async def test_particle_viz_set_offsets_toggle(self):
        self._setup_particle_objects()
        self._settings.set(pb.SETTING_DISPLAY_PARTICLES_RADIUS_TYPE, pb.ParticleVisualizationRadiusType.CONTACT_OFFSET)
        self._settings.set(pb.SETTING_DISPLAY_PARTICLES, pb.VisualizerMode.ALL)

        await self.wait(5)
        self._check_particle_offsets_and_visibility(offset_id=pb.ParticleVisualizationRadiusType.CONTACT_OFFSET)

        self._settings.set(
            pb.SETTING_DISPLAY_PARTICLES_RADIUS_TYPE, pb.ParticleVisualizationRadiusType.PARTICLE_CONTACT_OFFSET
        )
        await self.wait(5)
        self._check_particle_offsets_and_visibility(
            offset_id=pb.ParticleVisualizationRadiusType.PARTICLE_CONTACT_OFFSET
        )

        self._settings.set(pb.SETTING_DISPLAY_PARTICLES_RADIUS_TYPE, pb.ParticleVisualizationRadiusType.CONTACT_OFFSET)
        await self.wait(5)
        self._check_particle_offsets_and_visibility(offset_id=pb.ParticleVisualizationRadiusType.CONTACT_OFFSET)

    #(OM-54279, OM-60666)
    async def test_particle_viz_set_offsets_save_load(self):
        for offset_id in [
            pb.ParticleVisualizationRadiusType.CONTACT_OFFSET,
            pb.ParticleVisualizationRadiusType.REST_OFFSET,
            pb.ParticleVisualizationRadiusType.PARTICLE_CONTACT_OFFSET,
            pb.ParticleVisualizationRadiusType.PARTICLE_REST_OFFSET,
            pb.ParticleVisualizationRadiusType.RENDER_GEOMETRY,
        ]:
            with self.subTest(offset_id=offset_id):
                self._setup_particle_objects()
                self._settings.set(pb.SETTING_DISPLAY_PARTICLES_RADIUS_TYPE, offset_id)
                self._settings.set(pb.SETTING_DISPLAY_PARTICLES, pb.VisualizerMode.ALL)

                await self.wait(5)
                self._check_particle_offsets_and_visibility(offset_id=offset_id)

                # save_load, check that equal, and visibility is still correct:
                self._save_stage_to_file()
                await self.wait(5)
                self._check_particle_offsets_and_visibility(offset_id=offset_id)

                await self._load_stage_from_file()
                await self.wait(5)
                self._check_particle_offsets_and_visibility(offset_id=offset_id)

            await self.tearDown()
            await self.setUp()

    async def test_particle_viz_offsets_mixed_points_pointInstancer(self):
        offset_id = pb.ParticleVisualizationRadiusType.REST_OFFSET
        parameterSets = [(True, False), (False, True)]
        for set in parameterSets:
            with self.subTest(fluidPointInstancer=set[0], solidPointInstancer=set[1]):
                self._setup_particle_objects(fluidPointInstancer=set[0], solidPointInstancer=set[1])
                self._settings.set(pb.SETTING_DISPLAY_PARTICLES_RADIUS_TYPE, offset_id)
                self._settings.set(pb.SETTING_DISPLAY_PARTICLES, pb.VisualizerMode.ALL)

                await self.wait(5)
                self._check_particle_offsets_and_visibility(offset_id=offset_id)

                # run a sim step, check that equal, and visibility is still correct:
                await self.step(1)
                await self.wait(5)
                self._check_particle_offsets_and_visibility(offset_id=offset_id)

                # step again and reset to test reset correctness
                await self.step(1, stop_timeline_after=True)
                await self.wait(5)
                self._check_particle_offsets_and_visibility(offset_id=offset_id)

            # always do td / setup
            await self.tearDown()
            await self.setUp()

    def _check_particles_smoothing(self, position_type: pb.ParticleVisualizationPositionType):
        self.assertTrue(self.is_shown(self._get_fluid_viz_path()))
        self.assertFalse(self.is_shown(self._fluid_particles_path))

        self.assertTrue(self.is_shown(self._get_solid_viz_path()))
        self.assertFalse(self.is_shown(self._solid_particles_path))

        self.assertFalse(self.is_shown(self._cloth_mesh_path))
        self.assertTrue(self.is_shown(self._get_cloth_particles_viz_path()))

        # if smoothed positions are shown, they will match the render geom positions
        # if sim pos are shown, they will not match
        fluid_must_be_equal = position_type == pb.ParticleVisualizationPositionType.SMOOTHED_POSITIONS
        self._check_particle_positions(fluidEqual=fluid_must_be_equal, solidEqual=True, clothEqual=True)

    async def test_particle_viz_with_smoothing_sim(self):
        for smooth_position_mode in [
            pb.ParticleVisualizationPositionType.SIM_POSITIONS,
            pb.ParticleVisualizationPositionType.SMOOTHED_POSITIONS,
        ]:
            with self.subTest(smooth_position_mode=smooth_position_mode):
                self._setup_particle_objects()
                self._settings.set(pb.SETTING_DISPLAY_PARTICLES, pb.VisualizerMode.ALL)
                self._settings.set(pb.SETTING_DISPLAY_PARTICLES_POSITION_TYPE, smooth_position_mode)
                PhysxSchema.PhysxParticleSmoothingAPI.Apply(self._particle_system.GetPrim())

                await self.wait(5)
                self._check_particles_smoothing(position_type=smooth_position_mode)

                # run a sim step, check that equal, and visibility is still correct:
                await self.step(1)
                await self.wait(5)
                self._check_particles_smoothing(position_type=smooth_position_mode)

                # step again and reset to test reset correctness
                await self.step(1, stop_timeline_after=True)
                await self.wait(5)
                self._check_particles_smoothing(position_type=smooth_position_mode)

            # always do td / setup
            await self.tearDown()
            await self.setUp()

    async def test_particle_viz_with_smoothing_save_load(self):
        for smooth_position_mode in [
            pb.ParticleVisualizationPositionType.SIM_POSITIONS,
            pb.ParticleVisualizationPositionType.SMOOTHED_POSITIONS,
        ]:
            with self.subTest(smooth_position_mode=smooth_position_mode):
                self._setup_particle_objects()
                self._settings.set(pb.SETTING_DISPLAY_PARTICLES, pb.VisualizerMode.ALL)
                self._settings.set(pb.SETTING_DISPLAY_PARTICLES_POSITION_TYPE, smooth_position_mode)
                PhysxSchema.PhysxParticleSmoothingAPI.Apply(self._particle_system.GetPrim())

                await self.wait(5)
                self._check_particles_smoothing(position_type=smooth_position_mode)

                # save_load, check that equal, and visibility is still correct:
                self._save_stage_to_file()
                await self.wait(5)
                self._check_particles_smoothing(position_type=smooth_position_mode)

                await self._load_stage_from_file()
                await self.wait(5)
                self._check_particles_smoothing(position_type=smooth_position_mode)

            # always do td / setup
            await self.tearDown()
            await self.setUp()

    def _check_particles_anisotropy(self):
        self.assertTrue(self.is_shown(self._get_fluid_viz_path()))
        self.assertFalse(self._are_uniform_scale(self._get_fluid_viz_path()))
        self.assertFalse(self.is_shown(self._fluid_particles_path))

        self.assertTrue(self.is_shown(self._get_solid_viz_path()))
        self.assertTrue(self._are_uniform_scale(self._get_solid_viz_path()))
        self.assertFalse(self.is_shown(self._solid_particles_path))

        self.assertTrue(self.is_shown(self._get_cloth_particles_viz_path()))
        self.assertTrue(self._are_uniform_scale(self._get_cloth_particles_viz_path()))
        self.assertFalse(self.is_shown(self._cloth_mesh_path))

    async def test_particle_viz_with_anisotropy_save_load(self):
        self._setup_particle_objects(fluidPointInstancer=True, solidPointInstancer=True)
        self._settings.set(pb.SETTING_DISPLAY_PARTICLES, pb.VisualizerMode.ALL)
        self._settings.set(pb.SETTING_DISPLAY_PARTICLES_RADIUS_TYPE, pb.ParticleVisualizationRadiusType.ANISOTROPY)
        PhysxSchema.PhysxParticleAnisotropyAPI.Apply(self._particle_system.GetPrim())

        await self.wait(5)
        self._check_particles_anisotropy()

        # save_load, check that equal, and visibility is still correct:
        self._save_stage_to_file()
        await self.wait(5)
        self._check_particles_anisotropy()

        await self._load_stage_from_file()
        await self.wait(5)
        self._check_particles_anisotropy()

    async def test_particle_viz_with_anisotropy_sim(self):
        self._setup_particle_objects(fluidPointInstancer=True, solidPointInstancer=True)
        self._settings.set(pb.SETTING_DISPLAY_PARTICLES, pb.VisualizerMode.ALL)
        self._settings.set(pb.SETTING_DISPLAY_PARTICLES_RADIUS_TYPE, pb.ParticleVisualizationRadiusType.ANISOTROPY)
        PhysxSchema.PhysxParticleAnisotropyAPI.Apply(self._particle_system.GetPrim())

        await self.wait(5)
        self._check_particles_anisotropy()

        await self.step(1)
        await self.wait(5)
        self._check_particles_anisotropy()

        # step again and reset to test reset correctness
        await self.step(1, stop_timeline_after=True)
        await self.wait(5)
        self._check_particles_anisotropy()

    def _check_particles_isosurface(self, show_particles, show_surface):
        self.assertEqual(show_particles, self.is_shown(self._get_fluid_viz_path()))
        self.assertEqual(show_surface, self.is_shown(self._get_isosurface_viz_path()))
        self.assertEqual(not (show_surface or show_particles), self.is_shown(self._get_isosurface_path()))
        if show_surface:
            self.assertTrue(
                self._are_positions_almost_equal(self._get_isosurface_viz_path(), self._get_isosurface_path(), eps=1e-4)
            )
        self.assertFalse(self.is_shown(self._fluid_particles_path))

        self.assertEqual(show_particles, self.is_shown(self._get_solid_viz_path()))
        self.assertEqual(not show_particles, self.is_shown(self._solid_particles_path))

        self.assertFalse(self.is_shown(self._cloth_mesh_path))
        self.assertTrue(self.is_shown(self._get_cloth_particles_viz_path()))

    #(OM-54279)
    async def test_particle_viz_with_isosurface_sim(self):
        self._setup_particle_objects(fluidPointInstancer=True, solidPointInstancer=True)
        self._settings.set(pb.SETTING_DISPLAY_PARTICLES, pb.VisualizerMode.ALL)
        PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(self._particle_system.GetPrim())

        for show_surface in [True, False]:
            for show_particles in [True, False]:
                with self.subTest(show_surface=show_surface, show_particles=show_particles):
                    self._settings.set(pb.SETTING_DISPLAY_PARTICLES_SHOW_FLUID_SURFACE, show_surface)
                    self._settings.set(pb.SETTING_DISPLAY_PARTICLES_SHOW_PARTICLE_SET_PARTICLES, show_particles)
                    await self.wait(5)
                    self._check_particles_isosurface(show_particles=show_particles, show_surface=show_surface)

                    await self.step(1)
                    await self.wait(5)
                    self._check_particles_isosurface(show_particles=show_particles, show_surface=show_surface)

                    # step again and reset to test reset correctness
                    await self.step(1, stop_timeline_after=True)
                    await self.wait(5)
                    self._check_particles_isosurface(show_particles=show_particles, show_surface=show_surface)

    async def test_particle_viz_with_isosurface_save_load(self):
        self._setup_particle_objects(fluidPointInstancer=True, solidPointInstancer=True)
        self._settings.set(pb.SETTING_DISPLAY_PARTICLES, pb.VisualizerMode.ALL)
        PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(self._particle_system.GetPrim())

        for show_surface in [True, False]:
            for show_particles in [True, False]:
                with self.subTest(show_surface=show_surface, show_particles=show_particles):
                    self._settings.set(pb.SETTING_DISPLAY_PARTICLES_SHOW_FLUID_SURFACE, show_surface)
                    self._settings.set(pb.SETTING_DISPLAY_PARTICLES_SHOW_PARTICLE_SET_PARTICLES, show_particles)
                    await self.wait(5)
                    self._check_particles_isosurface(show_particles=show_particles, show_surface=show_surface)

                    # save_load, check that equal, and visibility is still correct:
                    self._save_stage_to_file()
                    await self.wait(5)
                    self._check_particles_isosurface(show_particles=show_particles, show_surface=show_surface)

                    await self._load_stage_from_file()
                    await self.wait(5)
                    self._check_particles_isosurface(show_particles=show_particles, show_surface=show_surface)

    def _check_cloth_viz(self, show_cloth_particles, show_cloth_mesh):
        self.assertEqual(show_cloth_particles, self.is_shown(self._get_cloth_particles_viz_path()))
        self.assertEqual(show_cloth_mesh, self.is_shown(self._get_cloth_mesh_viz_path()))
        self.assertEqual(not (show_cloth_mesh or show_cloth_particles), self.is_shown(self._cloth_mesh_path))
        if show_cloth_mesh:
            self.assertTrue(self._are_positions_almost_equal(self._get_cloth_mesh_viz_path(), self._cloth_mesh_path))

        self.assertTrue(self.is_shown(self._get_solid_viz_path()))
        self.assertFalse(self.is_shown(self._solid_particles_path))

        self.assertTrue(self.is_shown(self._get_fluid_viz_path()))
        self.assertFalse(self.is_shown(self._fluid_particles_path))

    async def test_particle_viz_cloth_sim(self):
        self._setup_particle_objects(fluidPointInstancer=True, solidPointInstancer=True)
        self._settings.set(pb.SETTING_DISPLAY_PARTICLES, pb.VisualizerMode.ALL)

        for show_cloth_mesh in [True, False]:
            for show_cloth_particles in [True, False]:
                with self.subTest(show_cloth_mesh=show_cloth_mesh, show_cloth_particles=show_cloth_particles):
                    self._settings.set(pb.SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_MESH, show_cloth_mesh)
                    self._settings.set(pb.SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_PARTICLES, show_cloth_particles)

                    await self.wait(5)
                    self._check_cloth_viz(show_cloth_particles=show_cloth_particles, show_cloth_mesh=show_cloth_mesh)

                    await self.step(1)
                    await self.wait(5)
                    self._check_cloth_viz(show_cloth_particles=show_cloth_particles, show_cloth_mesh=show_cloth_mesh)

                    # step again and reset to test reset correctness
                    await self.step(1, stop_timeline_after=True)
                    await self.wait(5)
                    self._check_cloth_viz(show_cloth_particles=show_cloth_particles, show_cloth_mesh=show_cloth_mesh)

    async def test_particle_viz_cloth_save_load(self):
        self._setup_particle_objects(fluidPointInstancer=True, solidPointInstancer=True)
        self._settings.set(pb.SETTING_DISPLAY_PARTICLES, pb.VisualizerMode.ALL)

        for show_cloth_mesh in [True, False]:
            for show_cloth_particles in [True, False]:
                with self.subTest(show_cloth_mesh=show_cloth_mesh, show_cloth_particles=show_cloth_particles):
                    self._settings.set(pb.SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_MESH, show_cloth_mesh)
                    self._settings.set(pb.SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_PARTICLES, show_cloth_particles)

                    await self.wait(5)
                    self._check_cloth_viz(show_cloth_particles=show_cloth_particles, show_cloth_mesh=show_cloth_mesh)

                    # save_load, check that equal, and visibility is still correct:
                    self._save_stage_to_file()
                    await self.wait(5)
                    self._check_cloth_viz(show_cloth_particles=show_cloth_particles, show_cloth_mesh=show_cloth_mesh)

                    await self._load_stage_from_file()
                    await self.wait(5)
                    self._check_cloth_viz(show_cloth_particles=show_cloth_particles, show_cloth_mesh=show_cloth_mesh)
