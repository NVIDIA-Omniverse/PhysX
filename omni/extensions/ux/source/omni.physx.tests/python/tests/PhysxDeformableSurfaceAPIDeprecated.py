# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import omni.kit.test
import omni.kit.commands
from pxr import Gf, Sdf, UsdGeom, UsdUtils, Usd, UsdPhysics, UsdLux, PhysxSchema
import omni.physx
from omni.physxcommands import AddGroundPlaneCommand, SetRigidBodyCommand
from omni.physxtests import utils
from omni.physx import get_physx_interface, get_physx_cooking_interface, get_physx_simulation_interface
from omni.physx.scripts import physicsUtils, deformableUtils
import unittest
from omni.physxtests.utils.physicsBase import PhysicsBaseAsyncTestCase, TestCategory
from omni.physxtests.utils.embeddedData import EmbeddedData
import omni.usd
import os
import math

# loosely following PhysxDeformableBodyAPITestAsync
class PhysxDeformableSurfaceAPITestAsync(PhysicsBaseAsyncTestCase):
    category = TestCategory.Kit

    @classmethod
    def setUpClass(self):
        # init for attributes that are not stage-dependent:
        # default number of places to check float equality:
        self._places = 3
        # carb settings and bloky dev mode:
        self._settings = carb.settings.get_settings()

    # runs before each test case
    async def setUp(self):
        await super().setUp()
        self._baseWasSetup = False

    # runs after each test case and runs base_terminate if setup was called:
    async def tearDown(self):
        if self._baseWasSetup:
            await self.base_terminate()
        omni.kit.commands.execute("SelectPrims", old_selected_paths=[], new_selected_paths=[], expand_in_stage=False)
        await super().tearDown()

    # physics scene and light setting
    async def base_setup(self, usd_file: str = None):
        self._baseWasSetup = True
        self.fail_on_log_error = True
        if usd_file is not None:
            await self._open_usd(usd_file)
            self._stage = omni.usd.get_context().get_stage()
        else:
            self._stage = await utils.new_stage_setup()
        sphereLight = UsdLux.SphereLight.Define(self._stage, Sdf.Path("/SphereLight"))
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

        self._upAxis = UsdGeom.GetStageUpAxis(self._stage)
        self._defaultPrimPath = self._stage.GetDefaultPrim().GetPath()

        # add physics scene
        omni.kit.commands.execute("AddPhysicsScene", stage=self._stage, path='/World/PhysicsScene')

        # material path: currently using only a single material
        self._deformable_surface_material_path = '/World/DeformableSurfaceMaterial'

    async def base_terminate(self):
        pass 

    def add_groundplane(self):
        AddGroundPlaneCommand.execute(self._stage, '/CollisionPlane',
                                      self._upAxis, 1000.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def assert_float_equal(self, floatA: float, floatB: float, places: int = None):
        if places is None:
            places = self._places
        self.assertAlmostEqual(floatA, floatB, places=places)

    def assert_almost_equal_vector3(self, vectorA, vectorB):
        for a, b in zip(vectorA, vectorB):
            self.assert_float_equal(a, b)

    def assert_transform_close(self, transform: Gf.Matrix4d, reference_transform: Gf.Matrix4d):
        for va, vb in zip(transform, reference_transform):
            for a, b in zip(va, vb):
                self.assert_float_equal(a, b)

    def assert_extents_close(self, extentA: Gf.Range3d, extentB: Gf.Range3d):
        self.assert_almost_equal_vector3(extentA.GetMin(), extentB.GetMin())
        self.assert_almost_equal_vector3(extentA.GetMax(), extentB.GetMax())

    def set_deformable_surface_iterations(self, deformableSurface_xform: UsdGeom.Xform, iterations):
        # check if body is deformable surface:
        ds_api = PhysxSchema.PhysxDeformableSurfaceAPI.Get(self._stage, deformableSurface_xform.GetPath())
        self.assertTrue(ds_api)
        deformable_api = PhysxSchema.PhysxDeformableAPI.Get(self._stage, deformableSurface_xform.GetPath())
        self.assertTrue(deformable_api.GetSolverPositionIterationCountAttr().Set(iterations))

    def check_deformable_surface(self, deformable_surface_path: Sdf.Path) -> bool:
        prim = self._stage.GetPrimAtPath(deformable_surface_path)
        self.assertTrue(prim and prim.IsA(UsdGeom.Mesh))
        self.assertTrue(prim and prim.HasAPI(PhysxSchema.PhysxDeformableAPI))
        self.assertTrue(prim and prim.HasAPI(PhysxSchema.PhysxDeformableSurfaceAPI))

        deformable = PhysxSchema.PhysxDeformableAPI(prim)

        indicesAttr = deformable.GetSimulationIndicesAttr()
        self.assertTrue(indicesAttr.HasAuthoredValue())
        indices = indicesAttr.Get()
        self.assertTrue(bool(indices) and len(indices) > 0 and len(indices) % 3 == 0)

        mesh = UsdGeom.Mesh(prim)
        pointsAttr = mesh.GetPointsAttr()
        self.assertTrue(pointsAttr.HasAuthoredValue())
        points = pointsAttr.Get()
        self.assertTrue(bool(points) and len(points) >= 3)

        restPointsAttr = deformable.GetRestPointsAttr()
        if restPointsAttr.HasAuthoredValue():
            restPoints = restPointsAttr.Get()
            self.assertTrue(bool(restPoints) and len(restPoints) == len(points))

    def start(self):
        physx_interface = get_physx_interface()
        physx_interface.start_simulation()

    def step(self, numSteps):
        physx_interface = get_physx_interface()
        time = 0.0
        dtime = self.get_time_step()
        for i in range(numSteps):
            physx_interface.update_simulation(dtime, time)
            physx_interface.update_transformations(True, True, True, False)
            time = time + dtime

    def reset(self):
        physx_interface = get_physx_interface()
        physx_interface.reset_simulation()

    def create_plane_prims(self, dimx, dimy, scale, num_meshes=1) -> list:
        prim_type="Plane" # currently, testing plane primitive only.
        mesh_list = []

        for i in range(num_meshes):
            # create grid mesh
            mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/" + prim_type + str(i), True))
            mesh = UsdGeom.Mesh.Define(self._stage, mesh_path)

            tri_points, tri_indices = deformableUtils.create_triangle_mesh_square(dimx=dimx, dimy=dimy, scale=scale)
            mesh.GetPointsAttr().Set(tri_points)
            mesh.GetFaceVertexIndicesAttr().Set(tri_indices)
            mesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))

            self.assertTrue(mesh)
            mesh_list.append(mesh)
        return mesh_list

    @staticmethod
    def apply_prim_translation(prim: Usd.Prim, translateVec: Gf.Vec3d, local_trafo=Gf.Matrix4d()):        
        new_translateVec = local_trafo.ExtractTranslation() + translateVec
        rot = local_trafo.ExtractRotation()
        transform_mtx = Gf.Matrix4d(rot, new_translateVec)
        omni.kit.commands.execute("TransformPrim", path=prim.GetPath(), new_transform_matrix=transform_mtx)

    @staticmethod
    def apply_prim_rotation(prim: Usd.Prim, rotationAxis: Gf.Vec3d, rotationAngle, local_trafo=Gf.Matrix4d()):
        translateVec = local_trafo.ExtractTranslation()
        rot = Gf.Rotation(rotationAxis, rotationAngle) # previous rotation is ignored for simplicity
        transform_mtx = Gf.Matrix4d(rot, translateVec)
        omni.kit.commands.execute("TransformPrim", path=prim.GetPath(), new_transform_matrix=transform_mtx)

    @staticmethod
    def apply_prim_transformation(prim: Usd.Prim, translateVec: Gf.Vec3d, rotationAxis: Gf.Vec3d, rotationAngle):
        new_translateVec = local_trafo.ExtractTranslation() + translateVec
        rot = local_trafo.ExtractRotation() # previous rotation is ignored for simplicity
        transform_mtx = Gf.Matrix4d(rot, new_translateVec)
        omni.kit.commands.execute("TransformPrim", path=prim.GetPath(), new_transform_matrix=transform_mtx)

    @staticmethod
    def getWorldBounds(imageable: UsdGeom.Imageable) -> Gf.Range3d:
        obb = imageable.ComputeWorldBound(Usd.TimeCode.Default(), purpose1=imageable.GetPurposeAttr().Get())
        return obb.ComputeAlignedBox()

    @staticmethod
    def get_time_step():
        return 1.0 / 60.0

    async def create_mesh_primitives(self, prim_type_list, starting_height=60.0):
        mesh_list = []
        for prim_type in prim_type_list:
            path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/" + prim_type, True))
            omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type=prim_type)
            mesh = UsdGeom.Mesh.Get(self._stage, path)
            self.assertTrue(mesh)
            mesh_list.append(mesh)
        
        height = starting_height
        offset = 150
        origin = Gf.Vec3d(-offset*3/2, height, -offset*3/2)
        for i in range(3):
            for j in range(3):
                index = i*3+j
                if index < len(mesh_list):
                    physicsUtils.set_or_add_translate_op(mesh_list[index], origin + Gf.Vec3d(i*offset, 0, j*offset))
                    
        return mesh_list

    async def create_plane_primitives(self, dimx=32, dimy=32, scale=50.0, starting_height=0.0, num_meshes=1, gap=20.0, vertical_mesh=True):
        mesh_list = self.create_plane_prims(dimx=dimx, dimy=dimy, scale=scale, num_meshes=num_meshes)

        height = starting_height
        origin = Gf.Vec3d(0.0, height, 0.0)
        for i in range(num_meshes):
            localTrafo = mesh_list[i].GetLocalTransformation()

            if vertical_mesh:
                self.apply_prim_translation(mesh_list[i].GetPrim(), origin + Gf.Vec3d(0.0, 0.0, i*gap), local_trafo=localTrafo)
            else:
                self.apply_prim_rotation(mesh_list[i].GetPrim(), rotationAxis=Gf.Vec3d(1.0, 0.0, 0.0), rotationAngle=90.0, local_trafo=localTrafo)
                localTrafo = mesh_list[i].GetLocalTransformation()
                self.apply_prim_translation(mesh_list[i].GetPrim(), origin + Gf.Vec3d(0.0, 100.0 + i*gap, 0.0), local_trafo=localTrafo)

        return mesh_list

    async def run_add_deformable_surface_component_command(self, skin_mesh_path: Sdf.Path=Sdf.Path(), material_path: Sdf.Path=Sdf.Path(), young=500.0) -> bool:
        # make path for deformable surface
        self.assertTrue(bool(skin_mesh_path))

        # make path for deformable surface material
        self.assertTrue(bool(material_path))

        # create deformable surface:
        success = omni.kit.commands.execute(
            "AddDeformableSurfaceComponent", mesh_path=skin_mesh_path);

        # create a material with small Young's modulus
        omni.kit.commands.execute("AddDeformableSurfaceMaterial",
                                  stage=self._stage, path=material_path,
                                  youngsModulus=young)

        # set deformable surface material
        physicsUtils.add_physics_material_to_prim(self._stage, self._stage.GetPrimAtPath(skin_mesh_path), material_path)
        return success

    async def run_reset_test(self):
        await self.base_setup()
        self.add_groundplane()
        starting_height = 80.0

        mesh, = await self.create_plane_primitives(starting_height=starting_height)
        mesh_path = mesh.GetPath()
        material_path = self._deformable_surface_material_path

        await self.run_add_deformable_surface_component_command(skin_mesh_path=mesh_path, material_path=material_path, young=50.0)

        initial_transform = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        initial_extents = self.getWorldBounds(mesh)

        self.start()
        self.step(20)

        # ensure that it moved:
        pause_height = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[1]
        self.assertLess(pause_height, starting_height)

        self.reset()

        # sync, get trafo, check:
        reset_transform = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assert_transform_close(reset_transform, reference_transform=initial_transform)
        reset_extent = self.getWorldBounds(mesh)
        self.assert_extents_close(initial_extents, reset_extent)

    async def run_deformable_surface_duplication(self):
        await self.base_setup()
        self.add_groundplane()

        num_meshes = 3
        starting_height = 70.0

        mesh_list = await self.create_plane_primitives(num_meshes=num_meshes)
        material_path = self._deformable_surface_material_path # currently, using only a single material

        mesh_path_list = [mesh.GetPath() for mesh in mesh_list]

        for mesh_path in mesh_path_list:
            await self.run_add_deformable_surface_component_command(skin_mesh_path=mesh_path, material_path=material_path, young=50.0)

        for mesh in mesh_list:
            initial_bounds = self.getWorldBounds(mesh)
            localTrafo = mesh.GetLocalTransformation()
            self.apply_prim_translation(mesh.GetPrim(), Gf.Vec3d(0.0, initial_bounds.GetSize()[1] * 0.5, 0.0), local_trafo=localTrafo)

        # get initial trafos:
        initial_transforms = [mesh.GetLocalTransformation() for mesh in mesh_list]

        # set iterations higher for this test to get less penetration:
        for deformable_surface_path in mesh_path_list:
            self.set_deformable_surface_iterations(UsdGeom.Mesh.Get(self._stage, deformable_surface_path), 50)

        duplicate_paths = []
        for path in mesh_path_list:
            duplicate_paths.append(Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(path), False)))

        paths_from = [str(path) for path in mesh_path_list]
        paths_to = [str(path) for path in duplicate_paths]
        omni.kit.commands.execute("CopyPrims", paths_from=paths_from, paths_to=paths_to)

        x_offset = 100
        z_offset = 100
        duplicate_deformable_surfaces = [UsdGeom.Mesh.Get(self._stage, duplicate_path) for duplicate_path in duplicate_paths]
        for deformable_surface, parentTrafo in zip(duplicate_deformable_surfaces, initial_transforms):
            self.assertTrue(PhysxSchema.PhysxDeformableSurfaceAPI.Get(self._stage, deformable_surface.GetPath()))
            self.apply_prim_translation(prim=deformable_surface.GetPrim(), translateVec=Gf.Vec3d(x_offset, 0, z_offset), local_trafo=parentTrafo)

        self.start()
        self.step(100)

        tol = 1.0  
        for deformable_surface in duplicate_deformable_surfaces:
            endTrafo = deformable_surface.GetLocalTransformation()
            self.assertLess(endTrafo.ExtractTranslation()[1], starting_height)
            extents = self.getWorldBounds(deformable_surface)
            self.assertGreater(extents.GetMin()[1], -tol)

    async def run_deformable_surface_reset_transform_test(self, delete_attribute_during_sim=False):
        await self.base_setup()

        starting_height = 100.0

        mesh, = await self.create_plane_primitives(starting_height=starting_height)
        mesh_path = mesh.GetPath()
        material_path = self._deformable_surface_material_path

        await self.run_add_deformable_surface_component_command(skin_mesh_path=mesh_path, material_path=material_path)

        # clear and recreate transforms
        mesh.ClearXformOpOrder()

        # setup a transform:
        init_translation = Gf.Vec3f(1.5, 100.0, 10.5)
        init_rotx = 5.0
        mesh.SetResetXformStack(True)
        mesh.AddTranslateOp().Set(init_translation)
        mesh.AddRotateXOp().Set(init_rotx)

        # step and check that it moved:
        self.start()
        self.step(5)
        translation = mesh.GetPrim().GetAttribute("xformOp:translate").Get()
        self.assertLess(translation[1], init_translation[1])

        sanitized_xform_ops = mesh.GetOrderedXformOps()
        self.assertEqual(len(sanitized_xform_ops), 3)
        opNames = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
        for op, opName in zip(sanitized_xform_ops, opNames):
            self.assertEqual(op.GetName(), opName)
        self.assertTrue(mesh.GetResetXformStack())

        if delete_attribute_during_sim:
            mesh.GetPrim().RemoveProperty('xformOp:rotateX')

        # reset and check that original ops order is restored and ops attributes as well
        self.reset()

        reset_xform_ops = mesh.GetOrderedXformOps()
        self.assertEqual(len(reset_xform_ops), 2)
        self.assertTrue(mesh.GetResetXformStack())

        self.assertEqual(reset_xform_ops[0].GetOpType(), UsdGeom.XformOp.TypeTranslate)
        self.assertEqual(reset_xform_ops[0].Get(), init_translation)

        self.assertEqual(reset_xform_ops[1].GetOpType(), UsdGeom.XformOp.TypeRotateX)
        self.assertEqual(reset_xform_ops[1].Get(), init_rotx)

        # check that orphaned standard ops are removed:
        self.assertFalse(mesh.GetPrim().GetAttribute("xformOp:scale"))
        self.assertFalse(mesh.GetPrim().GetAttribute("xformOp:orient"))


    async def test_deformable_surface_assign(self):        
        await self.base_setup()

        mesh, = await self.create_mesh_primitives(['Cube'])
        mesh_path = mesh.GetPath()
        material_path = self._deformable_surface_material_path

        await self.run_add_deformable_surface_component_command(skin_mesh_path=mesh_path, material_path=material_path)
        self.check_deformable_surface(mesh_path)

    # a vertical plane settles on a ground plane. 
    async def test_deformable_surface_basic(self):
        young = 50.0
        num_meshes = 3

        await self.base_setup()
        self.add_groundplane()

        mesh_list = await self.create_plane_primitives(num_meshes=num_meshes)
        material_path = self._deformable_surface_material_path # currently, using only a single material
        initial_bounds_list = []
        for mesh in mesh_list:
            initial_bounds = self.getWorldBounds(mesh)
            initial_bounds_list.append(initial_bounds)
            localTrafo = mesh.GetLocalTransformation()
            self.apply_prim_translation(mesh.GetPrim(), Gf.Vec3d(0.0, initial_bounds.GetSize()[1] * 0.5, 0.0), local_trafo=localTrafo)

        for mesh in mesh_list:
            await self.run_add_deformable_surface_component_command(skin_mesh_path=mesh.GetPath(), material_path=material_path, young=50.0)

        self.start()
        self.step(50)
        tol = 1.0

        for mesh, initial_bounds in zip(mesh_list, initial_bounds_list):
            self.check_deformable_surface(mesh.GetPath())
            deformed_bounds = self.getWorldBounds(mesh)
            self.assertGreater(deformed_bounds.GetMin()[1], -tol)
            self.assertLess(deformed_bounds.GetSize()[1] + tol, initial_bounds.GetSize()[1])

            self.assertGreater(deformed_bounds.GetMin()[1], -tol)
            self.assertLess(deformed_bounds.GetSize()[1] + tol, initial_bounds.GetSize()[1])

    async def test_deformable_surface_create_during_play(self):
        await self.base_setup()
        stepper = utils.PhysicsStepper()

        mesh, = await self.create_plane_primitives()
        mesh_path = mesh.GetPath()
        material_path = self._deformable_surface_material_path

        mesh_transform_0 = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(await stepper.play_and_step_and_pause(1, 5))

        await self.run_add_deformable_surface_component_command(skin_mesh_path=mesh_path, material_path=material_path)
        self.assertTrue(PhysxSchema.PhysxDeformableSurfaceAPI.Get(self._stage, mesh.GetPath()))

        self.assertTrue(await stepper.play_and_step_and_pause(1, 5))
        self.check_deformable_surface(mesh.GetPath())

        self.assertTrue(await stepper.play_and_step_and_pause(10, 5))
        mesh_transform_1 = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(Gf.Transform(mesh_transform_1).GetTranslation()[1] < Gf.Transform(mesh_transform_0).GetTranslation()[1] - 10.0)

        self.assertTrue(await stepper.play_and_step_and_stop(1, 5))

        self.assertTrue(PhysxSchema.PhysxDeformableSurfaceAPI.Get(self._stage, mesh_path))
        self.check_deformable_surface(mesh_path)

    async def test_deformable_surface_duplication(self):
        await self.run_deformable_surface_duplication()

    # test enable/disable deformable body
    async def test_deformable_surface_enable_disable(self):
        await self.base_setup()

        mesh, = await self.create_plane_primitives()
        mesh_path = mesh.GetPath()
        material_path = self._deformable_surface_material_path

        # setup deformable surface and record initial position
        await self.run_add_deformable_surface_component_command(skin_mesh_path=mesh_path, material_path=material_path)
        deformable_surface = PhysxSchema.PhysxDeformableSurfaceAPI.Apply(mesh.GetPrim())

        initial_translation = physicsUtils.get_translation(mesh.GetPrim())
        moved_delta = 10.0

        # simulate, check new position is lower than the initial one
        self.start()
        self.step(10)
        new_translation = physicsUtils.get_translation(mesh.GetPrim())
        self.assertLess(new_translation[1], initial_translation[1] - moved_delta)
        self.reset()

        # disable, simulate and check it didn't move
        PhysxSchema.PhysxDeformableAPI(deformable_surface).GetDeformableEnabledAttr().Set(False)
        self.start()
        self.step(10)
        new_translation = physicsUtils.get_translation(mesh.GetPrim())
        self.assertEqual(new_translation, initial_translation)
        self.reset()

        # re-enable, simulate and check new position is lower than the initial one
        PhysxSchema.PhysxDeformableAPI(deformable_surface).GetDeformableEnabledAttr().Set(True)
        self.start()
        self.step(10)
        new_translation = physicsUtils.get_translation(mesh.GetPrim())
        self.assertLess(new_translation[1], initial_translation[1] - moved_delta)

    # multi scenes
    async def test_deformable_surface_multi_scenes(self):
        num_meshes = 2
        starting_height = 100.0

        await self.base_setup()

        mesh_list = await self.create_plane_primitives(num_meshes=num_meshes, starting_height=starting_height)
        material_path = self._deformable_surface_material_path # currently, using only a single material

        for m in mesh_list:
            await self.run_add_deformable_surface_component_command(skin_mesh_path=m.GetPath(), material_path=material_path)

        #scene 1 has 1/4 the default gravity so objects fall much slower
        scene1Path = Sdf.Path('/World/PhysicsScene1')
        metersPerUnit = UsdGeom.GetStageMetersPerUnit(self._stage)
        scene = UsdPhysics.Scene.Define(self._stage, scene1Path)
        scene.CreateGravityMagnitudeAttr().Set(9.8 / metersPerUnit / 4)

        defaultScenePath = Sdf.Path('/World/PhysicsScene')
        targets = [defaultScenePath]
        prim = self._stage.GetPrimAtPath(mesh_list[0].GetPath())
        deformableAPI = PhysxSchema.PhysxDeformableSurfaceAPI(prim)
        PhysxSchema.PhysxDeformableAPI(deformableAPI).GetSimulationOwnerRel().SetTargets(targets)

        targets = [scene1Path]
        prim = self._stage.GetPrimAtPath(mesh_list[1].GetPath())
        deformableAPI = PhysxSchema.PhysxDeformableSurfaceAPI(prim)
        PhysxSchema.PhysxDeformableAPI(deformableAPI).GetSimulationOwnerRel().SetTargets(targets)

        self.start()
        self.step(10)

        height0 = mesh_list[0].ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[1]
        height1 = mesh_list[1].ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[1]
        self.assertLess(height0, height1)

    async def test_deformable_surface_pair_filter(self):
        await self.base_setup()
        self.add_groundplane()
        starting_height = 0.0

        mesh, = await self.create_plane_primitives()
        mesh_path = mesh.GetPath()
        material_path = self._deformable_surface_material_path

        # place mesh just above ground
        initial_bounds = self.getWorldBounds(mesh)
        localTrafo = mesh.GetLocalTransformation()
        self.apply_prim_translation(mesh.GetPrim(), Gf.Vec3d(0.0, initial_bounds.GetSize()[1] * 0.5, 0.0), local_trafo=localTrafo)

        await self.run_add_deformable_surface_component_command(skin_mesh_path=mesh_path, material_path=material_path)

        penetration_tol = 1.0

        # simulate and check that bound Ys have not moved below the surface:
        self.start()
        self.step(50)
        current_bounds = self.getWorldBounds(mesh)
        self.assertGreater(current_bounds.GetMin()[1], -penetration_tol)

        self.reset()

        # create filter
        filter_api = UsdPhysics.FilteredPairsAPI.Apply(mesh.GetPrim())
        plane_path = self._defaultPrimPath.AppendChild("CollisionPlane").AppendChild("CollisionPlane")
        print(plane_path)
        filter_api.GetFilteredPairsRel().AddTarget(Sdf.Path(plane_path))

        self.start()
        self.step(50)
        current_bounds = self.getWorldBounds(mesh)
        self.assertGreater(-penetration_tol, current_bounds.GetMin()[1])

    async def test_deformable_surface_reset_behavior(self):
        for x in range(3):
            await self.run_reset_test()

    async def test_deformable_surface_reset_transform(self):
        await self.run_deformable_surface_reset_transform_test()

    async def test_deformable_surface_reset_transform_delete_xformop(self):
        await self.run_deformable_surface_reset_transform_test(delete_attribute_during_sim=True)

    async def test_deformable_surface_rest_points(self):
        await self.base_setup()

        mesh, = await self.create_plane_primitives()
        mesh_path = mesh.GetPath()
        material_path = self._deformable_surface_material_path

        await self.run_add_deformable_surface_component_command(skin_mesh_path=mesh_path, material_path=material_path)

        #make rest points count inconsistent with mesh points count deliberatly
        rest_points = mesh.GetPointsAttr().Get()[:-1]
        deformable_surface = PhysxSchema.PhysxDeformableSurfaceAPI.Apply(mesh.GetPrim())
        PhysxSchema.PhysxDeformableAPI(deformable_surface).CreateRestPointsAttr().Set(rest_points)

        # check warning report for resetting rest points
        message = 'PhysxSchemaPhysxDeformableAPI:restPoints and UsdGeomMesh:points have inconsistent size. Resetting the restPoints to UsdGeomMesh:points.'
        with utils.ExpectMessage(self, message):
            self.start()

    async def test_deformable_surface_transform_sanitation(self):
        await self.base_setup()

        starting_height = 100.0

        mesh, = await self.create_plane_primitives(starting_height=starting_height)
        mesh_path = mesh.GetPath()
        material_path = self._deformable_surface_material_path

        # clear and recreate transforms
        mesh.ClearXformOpOrder()

        extraZrotAngle = 45.0
        mesh.AddRotateZOp().Set(extraZrotAngle)

        rotXangle = 10.0
        rotYangle = 20.0
        rotZangle = 30.0
        rotZYXangles = Gf.Vec3d(rotXangle, rotYangle, rotZangle)
        mesh.AddRotateZYXOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(rotZYXangles)

        scaleXYZ = Gf.Vec3d(2.0, 1.5, 0.8)
        mesh.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(scaleXYZ) # double not supported in reate_triangle_mesh_square

        translationXYZ = Gf.Vec3d(11, 12, 13)
        mesh.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(translationXYZ)

        initial_transform = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

        await self.run_add_deformable_surface_component_command(skin_mesh_path=mesh_path, material_path=material_path)

        # make sure adding deformable surface component does not sanitize transform
        self.assertTrue(len(mesh.GetOrderedXformOps()) == 4)

        # sim to stop
        self.start()
        self.step(10)

        # make sure transform got sanitized
        self.assertTrue(len(mesh.GetOrderedXformOps()) == 3)

        # ensure that it moved:
        pause_height = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[1]
        self.assertLess(pause_height, starting_height)

        self.reset()

        # sync, get trafo, check:
        reset_transform = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assert_transform_close(reset_transform, reference_transform=initial_transform)

        # check that original op stack got restored
        self.assertTrue(len(mesh.GetOrderedXformOps()) == 4)

    async def test_deformable_surface_undo_redo_add_deformable_surface_component_command(self):
        await self.base_setup()

        mesh, = await self.create_plane_primitives()
        mesh_path = mesh.GetPath()
        material_path = self._deformable_surface_material_path

        await self.run_add_deformable_surface_component_command(skin_mesh_path=mesh_path, material_path=material_path, young=50.0)

        self.assertTrue(PhysxSchema.PhysxDeformableSurfaceAPI.Get(self._stage, mesh_path))
        self.check_deformable_surface(mesh_path)

        #undo
        omni.kit.undo.undo() # undo AddDeformableSurfaceMaterial
        omni.kit.undo.undo() # undo AddDeformableSurfaceComponent
        self.assertFalse(PhysxSchema.PhysxDeformableSurfaceAPI.Get(self._stage, mesh_path))

        #redo
        omni.kit.undo.redo() # redo AddDeformableSurfaceComponent
        omni.kit.undo.redo() # redo AddDeformableSurfaceMaterial
        self.assertTrue(PhysxSchema.PhysxDeformableSurfaceAPI.Get(self._stage, mesh_path))
        self.check_deformable_surface(mesh_path)

    async def test_deformable_surface_undo_redo_remove_deformable_surface_component_command(self):
        await self.base_setup()

        mesh, = await self.create_plane_primitives()
        mesh_path = mesh.GetPath()
        mesh_prim = self._stage.GetPrimAtPath(mesh_path)
        material_path = self._deformable_surface_material_path

        await self.run_add_deformable_surface_component_command(skin_mesh_path=mesh_path, material_path=material_path, young=50.0)

        self.assertTrue(PhysxSchema.PhysxDeformableSurfaceAPI.Get(self._stage, mesh_path))
        self.check_deformable_surface(mesh_path)

        customProperties = [x.GetName() for x in mesh_prim.GetAuthoredPropertiesInNamespace(["physxDeformable"])]
        self.assertTrue(customProperties)

        success = omni.kit.commands.execute(
            "RemoveDeformableSurfaceComponent",
            prim_path=mesh_path)

        self.assertFalse(PhysxSchema.PhysxDeformableSurfaceAPI.Get(self._stage, mesh_path))
        customProperties = [x.GetName() for x in mesh_prim.GetAuthoredPropertiesInNamespace(["physxDeformable"])]
        self.assertFalse(customProperties)

        omni.kit.undo.undo()

        self.assertTrue(PhysxSchema.PhysxDeformableSurfaceAPI.Get(self._stage, mesh_path))
        self.check_deformable_surface(mesh_path)

        customProperties = [x.GetName() for x in mesh_prim.GetAuthoredPropertiesInNamespace(["physxDeformable"])]
        self.assertTrue(customProperties)

        omni.kit.undo.redo()

        self.assertFalse(PhysxSchema.PhysxDeformableSurfaceAPI.Get(self._stage, mesh_path))
        customProperties = [x.GetName() for x in mesh_prim.GetAuthoredPropertiesInNamespace(["physxDeformable"])]
        self.assertFalse(customProperties)

    async def _open_usd(self, filename):
        schema_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, "../../../../data/")))
        schema_folder = schema_folder.replace("\\", "/") + "/"

        #print(schema_folder + filename + ".usda")
        await omni.usd.get_context().open_stage_async(schema_folder + filename + ".usda")

        usd_context = omni.usd.get_context()
        self.assertIn(filename, usd_context.get_stage_url())

    async def run_deformable_surface_reset_weld(self, usd_file):
        await self.base_setup(usd_file)
        self.add_groundplane()

        mesh = UsdGeom.Mesh.Get(self._stage, '/World/' + 'Disk')
        xformable = UsdGeom.Xformable(mesh)
        initial_scale = Gf.Vec3f(1.0)
        initial_orient = Gf.Quatf(0.965925826, Gf.Vec3f(0.0, 0.0, 0.2588190451))
        initial_translate = Gf.Vec3f(60.0, 100.0, 50.0)
        physicsUtils.set_or_add_scale_orient_translate(xformable, initial_scale, initial_orient, initial_translate)        
        initial_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

        mesh_path = mesh.GetPath() 
        material_path = self._deformable_surface_material_path
        await self.run_add_deformable_surface_component_command(skin_mesh_path=mesh_path, material_path=material_path, young=50.0)
        self.assertTrue(PhysxSchema.PhysxDeformableSurfaceAPI.Get(self._stage, mesh_path))
        initial_transform = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        initial_extents = self.getWorldBounds(mesh)

        self.start()
        self.step(20)

        # ensure that it moved:
        pause_height = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[1]
        self.assertFalse(math.isnan(pause_height))
        self.assertLess(pause_height, initial_transform.ExtractTranslation()[1])

        self.reset()

        # sync, get trafo, check:
        reset_transform = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assert_transform_close(reset_transform, reference_transform=initial_transform)
        reset_extent = self.getWorldBounds(mesh)
        self.assert_extents_close(initial_extents, reset_extent)

    async def test_deformable_surface_reset_weld(self, usd_file="generatedLegacyMeshes"):
        await self.run_deformable_surface_reset_weld(usd_file)

    @unittest.skip("OM-96809")
    async def test_deformable_surface_reset_extra_vertices_weld(self, usd_file="weldedMeshWithExtraVertices"):
        await self.run_deformable_surface_reset_weld(usd_file)


    async def test_physics_deformable_surface_inconsistent_update(self):
        await self.base_setup()

        material_path = self._deformable_surface_material_path

        mesh, = await self.create_plane_primitives()
        await self.run_add_deformable_surface_component_command(skin_mesh_path=mesh.GetPath(), material_path=material_path, young=50.0)

        self.start()
        self.step(1)

        #clear points
        points = list(mesh.GetPointsAttr().Get())
        points.clear()

        message = f"Size of points of {mesh.GetPath()} has changed - skipping update."
        with utils.ExpectMessage(self, message):
            mesh.GetPointsAttr().Set(points)
            self.step(1)

        #clear sim velocities
        deformable = PhysxSchema.PhysxDeformableAPI(mesh)
        sim_velocities = list(deformable.GetSimulationVelocitiesAttr().Get())
        sim_velocities.clear()

        message = f"Size of physxDeformable:simulationVelocities of {mesh.GetPath()} has changed - skipping update."
        with utils.ExpectMessage(self, message):
            deformable.GetSimulationVelocitiesAttr().Set(sim_velocities)
            self.step(1)
