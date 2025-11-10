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
from omni.physx import get_physx_interface, get_physx_cooking_interface, get_physx_cooking_private_interface, get_physx_simulation_interface
from omni.physx.scripts import physicsUtils, deformableUtils
import unittest
from omni.physxtests.utils.physicsBase import PhysicsBaseAsyncTestCase, TestCategory
from omni.physxtests.utils.embeddedData import EmbeddedData
import omni.usd


class PhysxDeformableBodyAPITestAsync(PhysicsBaseAsyncTestCase):
    category = TestCategory.Kit
    @classmethod
    def setUpClass(self):
        # init for attributes that are not stage-dependent:
        # default number of places to check float equality:
        self._places = 3
        # carb settings and bloky dev mode:
        self._settings = carb.settings.get_settings()
        self._prim_type_list = ['Cone', 'Cube', 'Cylinder', 'Sphere', 'Torus']

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

    async def base_setup(self):
        self._baseWasSetup = True
        self.fail_on_log_error = True
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

    async def base_terminate(self):
        pass

    def add_groundplane(self):
        AddGroundPlaneCommand.execute(self._stage, '/CollisionPlane',
                                      self._upAxis, 1000.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    async def _runAddDeformableBodyComponentCommand(self, skin_mesh_path: Sdf.Path=Sdf.Path(), collision_mesh_path: Sdf.Path=Sdf.Path(), simulation_mesh_path: Sdf.Path=Sdf.Path()) -> bool:
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

    def _test_float_equal(self, floatA: float, floatB: float, places: int = None):
        if places is None:
            places = self._places
        self.assertAlmostEqual(floatA, floatB, places=places)

    def _assertAlmostEqualVector3(self, vectorA, vectorB):
        for a, b in zip(vectorA, vectorB):
            self._test_float_equal(a, b)

    def _assert_transform_close(self, transform: Gf.Matrix4d, reference_transform: Gf.Matrix4d):
        for va, vb in zip(transform, reference_transform):
            for a, b in zip(va, vb):
                self._test_float_equal(a, b)

    def _assert_extents_close(self, extentA: Gf.Range3d, extentB: Gf.Range3d):
        self._assertAlmostEqualVector3(extentA.GetMin(), extentB.GetMin())
        self._assertAlmostEqualVector3(extentA.GetMax(), extentB.GetMax())

    def _set_softbody_iterations(self, softbody_xform: UsdGeom.Xform, iterations):
        # check if body is soft:
        sb_api = PhysxSchema.PhysxDeformableBodyAPI.Get(self._stage, softbody_xform.GetPath())
        self.assertTrue(sb_api)
        deformable_api = PhysxSchema.PhysxDeformableAPI.Get(self._stage, softbody_xform.GetPath())
        self.assertTrue(deformable_api.GetSolverPositionIterationCountAttr().Set(iterations))

    @staticmethod
    def set_prim_translation(prim: Usd.Prim, translateVec: Gf.Vec3d):
        translate_mtx = Gf.Matrix4d().SetTranslate(translateVec)
        omni.kit.commands.execute("TransformPrim", path=prim.GetPath(), new_transform_matrix=translate_mtx)

    @staticmethod
    def getWorldBounds(imageable: UsdGeom.Imageable) -> Gf.Range3d:
        obb = imageable.ComputeWorldBound(Usd.TimeCode.Default(), purpose1=imageable.GetPurposeAttr().Get())
        return obb.ComputeAlignedBox()

    def _verify_coll_and_viz_mesh_conincide(self, coll_mesh_path: Sdf.Path, viz_mesh_path: Sdf.Path):
        viz_mesh = UsdGeom.Mesh.Get(self._stage, viz_mesh_path)
        coll_mesh = PhysxSchema.TetrahedralMesh.Get(self._stage, coll_mesh_path)

        # get bounding boxes:
        coll_bb = PhysXSoftbodyTest.getWorldBounds(coll_mesh)
        viz_bb = PhysXSoftbodyTest.getWorldBounds(viz_mesh)

        # make sure the sizes are close:
        tol = 0.05  # 5%
        for i in range(3):
            diff = viz_bb.GetMidpoint()[i] - coll_bb.GetMidpoint()[i]
            minSize = 1.0
            size = max(minSize, viz_bb.GetSize()[i])
            diff = abs(diff / size)
            self.assertGreater(tol, diff)

    def _check_collision_mesh(self, soft_body_path: Sdf.Path):
        prim = self._stage.GetPrimAtPath(soft_body_path)
        deformable_body = PhysxSchema.PhysxDeformableBodyAPI(prim)
        indicesAttr = deformable_body.GetCollisionIndicesAttr()
        self.assertTrue(indicesAttr.HasAuthoredValue())
        indices = indicesAttr.Get()
        self.assertTrue(bool(indices) and len(indices) > 0 and len(indices) % 4 == 0)
        restPointsAttr = deformable_body.GetCollisionRestPointsAttr();
        self.assertTrue(restPointsAttr.HasAuthoredValue())
        restPoints = restPointsAttr.Get()
        self.assertTrue(bool(restPoints) and len(restPoints) >= 4)
        pointsAttr = deformable_body.GetCollisionPointsAttr()
        if pointsAttr.HasAuthoredValue():
            points = pointsAttr.Get()
            self.assertTrue(bool(points) and len(points) == len(restPoints))

    def _check_simulation_mesh(self, soft_body_path: Sdf.Path):
        prim = self._stage.GetPrimAtPath(soft_body_path)
        deformable = PhysxSchema.PhysxDeformableAPI(prim)
        deformable_body = PhysxSchema.PhysxDeformableBodyAPI(prim)
        indicesAttr = deformable.GetSimulationIndicesAttr()
        if indicesAttr.HasAuthoredValue():
            indices = indicesAttr.Get()
            self.assertTrue(bool(indices) and len(indices) > 0 and len(indices) % 4 == 0)
        restPointsAttr = deformable_body.GetSimulationRestPointsAttr()
        if restPointsAttr.HasAuthoredValue():
            restPoints = restPointsAttr.Get()
            self.assertTrue(bool(restPoints) and len(restPoints) >= 4)
        pointsAttr = deformable_body.GetSimulationPointsAttr()
        if pointsAttr.HasAuthoredValue():
            points = pointsAttr.Get()
            self.assertTrue(bool(points) and len(points) == len(restPoints))

    def _check_soft_body(self, soft_body_path: Sdf.Path) -> bool:
        prim = self._stage.GetPrimAtPath(soft_body_path)
        self.assertTrue(prim and prim.IsA(UsdGeom.Mesh))
        self.assertTrue(prim and prim.HasAPI(PhysxSchema.PhysxDeformableAPI))
        self.assertTrue(prim and prim.HasAPI(PhysxSchema.PhysxDeformableBodyAPI))
        self._check_collision_mesh(soft_body_path)
        self._check_simulation_mesh(soft_body_path)

        #check skin mesh
        mesh = UsdGeom.Mesh(prim)
        deformable = PhysxSchema.PhysxDeformableAPI(prim)
        points = mesh.GetPointsAttr().Get()
        restPointsAttr = deformable.GetRestPointsAttr()
        if restPointsAttr.HasAuthoredValue():
            restPoints = restPointsAttr.Get()
            self.assertTrue(bool(restPoints) and len(restPoints) == len(points))
        
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
        offset = 150
        origin = Gf.Vec3d(-offset * 3 / 2, height, -offset * 3 / 2)
        for i in range(3):
            for j in range(3):
                index = i * 3 + j
                if index < len(mesh_list):
                    self.set_prim_translation(mesh_list[index].GetPrim(), origin + Gf.Vec3d(i * offset, 0, j * offset))

        return mesh_list

    async def test_undo_redo_add_deformable_body_component_command(self):
        await self.base_setup()

        mesh, = await self._create_mesh_primitives(['Cube'])
        mesh_path = mesh.GetPath()

        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh_path)

        self.assertTrue(PhysxSchema.PhysxDeformableBodyAPI.Get(self._stage, mesh_path))
        #need to cook softbodies synchronous so meshes can be checked
        get_physx_cooking_interface().cook_deformable_body_mesh(str(mesh_path))
        self._check_soft_body(mesh_path)
        omni.kit.undo.undo()

        self.assertFalse(PhysxSchema.PhysxDeformableBodyAPI.Get(self._stage, mesh_path))

        omni.kit.undo.redo()

        self.assertTrue(PhysxSchema.PhysxDeformableBodyAPI.Get(self._stage, mesh_path))
        get_physx_cooking_interface().cook_deformable_body_mesh(str(mesh_path))
        self._check_soft_body(mesh_path)

    async def test_undo_redo_remove_deformable_body_component_command(self):
        await self.base_setup()

        mesh, = await self._create_mesh_primitives(['Cube'])
        mesh_path = mesh.GetPath()
        mesh_prim = self._stage.GetPrimAtPath(mesh_path)

        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh_path)

        self.assertTrue(PhysxSchema.PhysxDeformableBodyAPI.Get(self._stage, mesh_path))
        get_physx_cooking_interface().cook_deformable_body_mesh(str(mesh_path))
        self._check_soft_body(mesh_path)
        customProperties = [x.GetName() for x in mesh_prim.GetAuthoredPropertiesInNamespace(["physxDeformable"])]
        self.assertTrue(customProperties)

        success = omni.kit.commands.execute(
            "RemoveDeformableBodyComponent",
            prim_path=mesh_path)

        self.assertFalse(PhysxSchema.PhysxDeformableBodyAPI.Get(self._stage, mesh_path))
        customProperties = [x.GetName() for x in mesh_prim.GetAuthoredPropertiesInNamespace(["physxDeformable"])]
        self.assertFalse(customProperties)

        omni.kit.undo.undo()

        self.assertTrue(PhysxSchema.PhysxDeformableBodyAPI.Get(self._stage, mesh_path))
        get_physx_cooking_interface().cook_deformable_body_mesh(str(mesh_path))
        self._check_soft_body(mesh_path)
        customProperties = [x.GetName() for x in mesh_prim.GetAuthoredPropertiesInNamespace(["physxDeformable"])]
        self.assertTrue(customProperties)

        omni.kit.undo.redo()

        self.assertFalse(PhysxSchema.PhysxDeformableBodyAPI.Get(self._stage, mesh_path))
        customProperties = [x.GetName() for x in mesh_prim.GetAuthoredPropertiesInNamespace(["physxDeformable"])]
        self.assertFalse(customProperties)


    @staticmethod
    def _get_time_step():
        return 1.0 / 60.0

    def _start(self):
        physx_interface = get_physx_interface()
        physx_interface.start_simulation()
        self._time = 0.0

    def _step(self, numSteps):
        physx_interface = get_physx_interface()
        
        dtime = self._get_time_step()
        for i in range(numSteps):
            physx_interface.update_simulation(dtime, self._time)
            physx_interface.update_transformations(True, True, True, False)
            self._time = self._time + dtime

    def _reset(self):
        physx_interface = get_physx_interface()
        physx_interface.reset_simulation()

    async def _run_mesh_test(self):
        await self.base_setup()
        self.add_groundplane()
        starting_height = 0.0

        omni.kit.commands.execute("AddDeformableBodyMaterial",
                                  stage=self._stage, path=self._deformable_body_material_path,
                                  youngsModulus=800.0)

        primList = ['Cone', 'Cube', 'Cylinder', 'Sphere', 'Torus']
        mesh_list = await self._create_mesh_primitives(primList, starting_height=starting_height)

        initial_bounds_list = []
        for mesh in mesh_list:
            initial_bounds = self.getWorldBounds(mesh)
            initial_bounds_list.append(initial_bounds)
            localTrafo = mesh.GetLocalTransformation()
            self.set_prim_translation(mesh.GetPrim(), localTrafo.ExtractTranslation() + Gf.Vec3d(0.0, initial_bounds.GetSize()[1] * 0.5, 0.0))

        for m in mesh_list:
            await self._runAddDeformableBodyComponentCommand(skin_mesh_path=m.GetPath())

        self._start()
        self._step(50)

        # check that bound Ys have not moved below the surface and that body deformed:
        tol = 10.0
        deformTol = 1.0
        for mesh, initial_bounds in zip(mesh_list, initial_bounds_list):
            self._check_collision_mesh(mesh.GetPath())
            deformed_bounds = self.getWorldBounds(mesh)
            self.assertGreater(deformed_bounds.GetMin()[1], -tol)
            self.assertLess(deformed_bounds.GetSize()[1] + deformTol, initial_bounds.GetSize()[1])

    # this test creates a softbody from each of the mesh (non-flat) primitives exposed in Create->Mesh
    # let's all of them settle on a ground plane, and for them to be on the ground plane and
    # if they are deformed
    # used to fail (OM-45168)
    async def test_mesh_softbody(self):
        await self._run_mesh_test()

    # Test reset behavior:
    async def _run_reset_test(self, set_scale: bool = False):
        await self.base_setup()
        self.add_groundplane()
        starting_height = 80.0 + set_scale * 20.0
        prim_type_list = ['Cone']
        mesh, = await self._create_mesh_primitives(prim_type_list, starting_height=starting_height)

        for op in mesh.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeRotateZYX:
                op.Set(Gf.Vec3d(10.0, 20.0, 30.0))
            if op.GetOpType() == UsdGeom.XformOp.TypeScale and set_scale:
                op.Set(Gf.Vec3d(2.0, 1.0, 1.5))

        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh.GetPath())

        initial_transform = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        initial_extents = self.getWorldBounds(mesh)

        self._start()
        self._step(20)

        # ensure that it moved:
        pause_height = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[1]
        self.assertLess(pause_height, starting_height)

        self._reset()

        # sync, get trafo, check:
        reset_transform = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self._assert_transform_close(reset_transform, reference_transform=initial_transform)
        reset_extent = self.getWorldBounds(mesh)
        self._assert_extents_close(initial_extents, reset_extent)

    #used to fail (OM-21616)
    async def test_mesh_softbody_reset_behavior(self):
        for x in range(5):
            await self._run_reset_test()

    async def _run_softbody_duplication(self):
        await self.base_setup()
        self.add_groundplane()
        starting_height = 70.0
        meshes = await self._create_mesh_primitives(self._prim_type_list, starting_height=starting_height)
        # get initial trafos:
        initial_transforms = [mesh.GetLocalTransformation() for mesh in meshes]

        # set softbody (which will apply the SoftbodyAPI)
        mesh_path_list = [mesh.GetPath() for mesh in meshes]

        for m in mesh_path_list:
            await self._runAddDeformableBodyComponentCommand(skin_mesh_path=m)

        # set iterations higher for this test to get less penetration:
        for soft_body_path in mesh_path_list:
            self._set_softbody_iterations(UsdGeom.Mesh.Get(self._stage, soft_body_path), 50)

        duplicate_paths = []
        for path in mesh_path_list:
            #can fail with Sdf.Path
            duplicate_paths.append(Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(path), False)))

        #doesn't take Sdf.Path
        paths_from = [str(path) for path in mesh_path_list]
        paths_to = [str(path) for path in duplicate_paths]
        omni.kit.commands.execute("CopyPrims", paths_from=paths_from, paths_to=paths_to)

        x_offset = 250
        z_offset = 250
        duplicate_soft_bodies = [UsdGeom.Mesh.Get(self._stage, duplicate_path) for duplicate_path in duplicate_paths]
        for soft_body, parentTrafo in zip(duplicate_soft_bodies, initial_transforms):
            self.assertTrue(PhysxSchema.PhysxDeformableBodyAPI.Get(self._stage, soft_body.GetPath()))
            self.set_prim_translation(prim=soft_body.GetPrim(), translateVec=parentTrafo.ExtractTranslation() + Gf.Vec3d(x_offset, 0, z_offset))

        self._start()
        self._step(100)

        tol = 25.0  # get quite a bit of penetration so be tolerant here
        for soft_body in duplicate_soft_bodies:
            endTrafo = soft_body.GetLocalTransformation()
            self.assertLess(endTrafo.ExtractTranslation()[1], starting_height)
            extents = self.getWorldBounds(soft_body)
            self.assertGreater(extents.GetMin()[1], -tol)

    async def test_mesh_softbody_duplication(self):
        await self._run_softbody_duplication()

    async def _run_softbody_reset_transform_test(self, deleteAttributeDuringSim=False):
        await self.base_setup()
        prim_type_list = ['Cube']
        starting_height = 100.0
        mesh, = await self._create_mesh_primitives(prim_type_list, starting_height=starting_height)

        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh.GetPath())

        # clear and recreate transforms
        mesh.ClearXformOpOrder()
        # setup a transform:
        init_translation = Gf.Vec3f(1.5, 100.0, 10.5)
        init_rotx = 5.0
        mesh.SetResetXformStack(True)
        mesh.AddTranslateOp().Set(init_translation)
        mesh.AddRotateXOp().Set(init_rotx)

        # step and check that it moved:
        self._start()
        self._step(5)
        pos = mesh.GetPrim().GetAttribute("xformOp:translate").Get()
        self.assertLess(pos[1], init_translation[1])

        sanitized_xform_ops = mesh.GetOrderedXformOps()
        self.assertEqual(len(sanitized_xform_ops), 3)
        opNames = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
        for op, opName in zip(sanitized_xform_ops, opNames):
            self.assertEqual(op.GetName(), opName)
        self.assertTrue(mesh.GetResetXformStack())

        if deleteAttributeDuringSim:
            mesh.GetPrim().RemoveProperty('xformOp:rotateX')

        # reset and check that original ops order is restored and ops attributes as well
        self._reset()

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

    async def test_physics_softbody_reset_transform(self):
        await self._run_softbody_reset_transform_test()

    async def test_physics_softbody_reset_transform_delete_xformop(self):
        await self._run_softbody_reset_transform_test(deleteAttributeDuringSim=True)

    #used to fail (OM-22997)
    async def test_softbody_transform_sanitation(self):
        await self.base_setup()
        prim_type_list = ['Cube']
        starting_height = 100.0
        mesh, = await self._create_mesh_primitives(prim_type_list, starting_height=starting_height)

        # clear and recreate transforms
        mesh.ClearXformOpOrder()
        extraZrotAngle = 45.0

        mesh.AddRotateZOp().Set(extraZrotAngle)
        rotXangle = 10.0
        rotYangle = 20.0
        rotZangle = 30.0
        rotZYXangles = Gf.Vec3f(rotXangle, rotYangle, rotZangle)
        mesh.AddRotateZYXOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(rotZYXangles)
        scaleXYZ = Gf.Vec3d(2.0, 1.5, 0.8)
        mesh.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(scaleXYZ)
        translationXYZ = Gf.Vec3d(11, 12, 13)
        mesh.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(translationXYZ)

        initial_transform = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh.GetPath())
        # make sure adding deformable body component does not sanitize transform
        self.assertTrue(len(mesh.GetOrderedXformOps()) == 4)

        # sim to stop
        self._start()
        self._step(10)

        # make sure transform got sanitized
        self.assertTrue(len(mesh.GetOrderedXformOps()) == 3)

        # ensure that it moved:
        pause_height = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[1]
        self.assertLess(pause_height, starting_height)

        self._reset()

        # sync, get trafo, check:
        reset_transform = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self._assert_transform_close(reset_transform, reference_transform=initial_transform)

        # check that original op stack got restored
        self.assertTrue(len(mesh.GetOrderedXformOps()) == 4)

    #used to fail (OM-22996)
    async def test_mesh_softbody_oblong(self):
        await self._run_reset_test(set_scale=True)

    async def _wait_cooking_finished(self):
        while True:
            await omni.kit.app.get_app().next_update_async()
            cooking_statistics = get_physx_cooking_private_interface().get_cooking_statistics()
            running_tasks = cooking_statistics.total_scheduled_tasks - cooking_statistics.total_finished_tasks
            self.assertGreaterEqual(running_tasks, 0)
            if running_tasks <= 0:
                break

    async def _wait_for_deformable_body_cooking(self, mesh_list, asynchronous: bool = False, max_updates: int = 100):
        cooking_iface = get_physx_cooking_interface()
        if asynchronous:
            await omni.kit.app.get_app().next_update_async()
            #wait for tet mesh and softbody cooking tasks to finish (note, we assume that they get scheduled back to back)
            await self._wait_cooking_finished()
        else:
            for mesh in mesh_list:
                cooking_iface.cook_deformable_body_mesh(str(mesh.GetPath()))

    #test re-cook sensitivity on transform changes:
    async def _run_recooking_test(self, change_type: str, sync_type: str):
        await self.base_setup()

        mesh_type_list = ['Torus', 'Cone', 'Cube']
        mesh_list = await self._create_mesh_primitives(mesh_type_list)

        #initialize meshes with non-uniform scale and rotation
        initial_orient = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1,1,1).GetNormalized(), 35.0).GetQuat())
        initial_scale = Gf.Vec3f(1.0, 0.5, 0.75)
        initial_translate = Gf.Vec3f(0.0, 0.0, 0.0)

        for mesh in mesh_list:
            xformOps = mesh.GetXformOpOrderAttr().Get()
            for op in xformOps:
                mesh.GetPrim().RemoveProperty(op)
            mesh.GetXformOpOrderAttr().Clear()
            physicsUtils.set_or_add_scale_orient_translate(mesh, initial_scale, initial_orient, initial_translate)

        initial_tetmesh_crc_list = []
        for mesh in mesh_list:
            #create softbody
            await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh.GetPath())

            #cook and check both crc are set
            get_physx_cooking_interface().cook_deformable_body_mesh(str(mesh.GetPath()))
            tetmesh_crc_attr = mesh.GetPrim().GetAttribute("physxDeformable:tetMeshCrc")
            self.assertTrue(tetmesh_crc_attr)

            #store initial_crc
            initial_tetmesh_crc_list.append(tetmesh_crc_attr.Get())

        # trigger differnt recooking scenarios:
        for mesh in mesh_list:
            if change_type == "change_translation":
                physicsUtils.set_or_add_translate_op(mesh, initial_translate + Gf.Vec3f(120.0, 10.0, -10.0))
            elif change_type == "change_rotation":
                new_orient = Gf.Quatf(Gf.Rotation(Gf.Vec3d(-1,2,1).GetNormalized(), 15.0).GetQuat())
                physicsUtils.set_or_add_orient_op(mesh, new_orient)
            elif change_type == "change_scale_uniform":
                physicsUtils.set_or_add_scale_op(mesh, initial_scale*2.0)
            elif change_type == "change_scale_non_uniform":
                physicsUtils.set_or_add_scale_op(mesh, Gf.Vec3f(2.1, 1.0, 1.6))
            elif change_type == "change_stack_scale_non_uniform":
                #change scale through op stack removal
                old_xform_op_order = mesh.GetXformOpOrderAttr().Get()
                new_xform_op_order = []
                for xform_op in old_xform_op_order:
                    if xform_op != 'xformOp:scale':
                        new_xform_op_order.append(xform_op)
                mesh.GetXformOpOrderAttr().Set(new_xform_op_order)

        # trigger cooking in different ways
        if sync_type == "sync_sim":
            self._start()
            self._step(1)
        elif sync_type == "sync_explicit":
            await self._wait_for_deformable_body_cooking(mesh_list, asynchronous=False)
        elif sync_type == "sync_wait":
            await self._wait_for_deformable_body_cooking(mesh_list, asynchronous=True, max_updates=len(mesh_type_list)*100)

        # check expected CRCs
        for mesh, initial_tetmesh_crc in zip(mesh_list, initial_tetmesh_crc_list):
            tetmesh_crc_attr = mesh.GetPrim().GetAttribute("physxDeformable:tetMeshCrc")
            self.assertTrue(tetmesh_crc_attr)

            if change_type in ["change_translation", "change_rotation", "change_scale_uniform"]:
                self.assertTrue(initial_tetmesh_crc == tetmesh_crc_attr.Get())
            elif change_type in ["change_scale_non_uniform", "change_stack_scale_non_uniform"]:
                self.assertTrue(initial_tetmesh_crc != tetmesh_crc_attr.Get())


    #test re-cook sensitivity on transform changes:
    async def test_softbody_recooking_mesh_on_transform_changes(self):
        #also for OM-46697, making sure async cooking triggered correctly
        for sync_type in ["sync_sim", "sync_explicit", "sync_wait"]:
            for change_type in ["change_translation", "change_rotation", "change_scale_uniform", "change_scale_non_uniform", "change_stack_scale_non_uniform"]:
                await self._run_recooking_test(change_type=change_type, sync_type=sync_type)

    #test that cooking triggering has no effect during simulation (warning for cooking parameters, no warning for scale) 
    async def test_softbody_trigger_recooking_mesh_during_play(self):
        await self.base_setup()
        stepper = utils.PhysicsStepper()

        prim_type_list = ['Torus']
        torus_mesh, = await self._create_mesh_primitives(prim_type_list)

        self.assertTrue(await self._runAddDeformableBodyComponentCommand(skin_mesh_path=torus_mesh.GetPath()))

        simres_attr = torus_mesh.GetPrim().GetAttribute("physxDeformable:simulationHexahedralResolution")
        collsimp_attr = torus_mesh.GetPrim().GetAttribute("physxDeformable:collisionSimplification")
        scale_attr = torus_mesh.GetPrim().GetAttribute("xformOp:scale")
        self.assertTrue(simres_attr)
        self.assertTrue(collsimp_attr)
        self.assertTrue(scale_attr)
        initial_simres = simres_attr.Get()
        initial_collsimp = collsimp_attr.Get()
        initial_scale = scale_attr.Get()

        tetmesh_crc_attr = torus_mesh.GetPrim().GetAttribute("physxDeformable:tetMeshCrc")
        self.assertTrue(tetmesh_crc_attr)
        initial_tetmesh_crc = tetmesh_crc_attr.Get()

        self.assertTrue(await stepper.play_and_step_and_pause(1, 5))

        error_message = f"Changing deformable body mesh or meshing related PhysxSchemaPhysxDeformableBodyAPI parameter during simulation is not supported. Prim: {torus_mesh.GetPrim().GetPath()}"
        #set simulation resolution
        with utils.ExpectMessage(self, error_message):
            simres_attr.Set(initial_simres+1)
            await omni.kit.app.get_app().next_update_async()

        #set collision simplification
        with utils.ExpectMessage(self, error_message):
            collsimp_attr.Set(not initial_simres)
            await omni.kit.app.get_app().next_update_async()

        #set scale
        new_scale = initial_scale
        new_scale[2] = new_scale[2]*0.5
        scale_attr.Set(new_scale)
        await omni.kit.app.get_app().next_update_async()

        #check crc
        await self._wait_for_deformable_body_cooking([torus_mesh], asynchronous=True, max_updates=100)
        new_tetmesh_crc = tetmesh_crc_attr.Get()
        self.assertTrue(initial_tetmesh_crc == new_tetmesh_crc)


    def _check_failed_cooking_state_deformable_body(self, mesh):
        deformableBody = PhysxSchema.PhysxDeformableBodyAPI(mesh.GetPrim())
        self.assertTrue(deformableBody)
        collisionIndices = deformableBody.GetCollisionIndicesAttr().Get()
        collisionRestPoints = deformableBody.GetCollisionRestPointsAttr().Get()
        simulationIndices = PhysxSchema.PhysxDeformableAPI(deformableBody).GetSimulationIndicesAttr().Get()
        simulationRestPoints = deformableBody.GetSimulationRestPointsAttr().Get()
        self.assertTrue(len(collisionIndices) == 0)
        self.assertTrue(len(collisionRestPoints) == 0)
        self.assertTrue(len(simulationIndices) == 0)
        self.assertTrue(len(simulationRestPoints) == 0)

    async def _run_flat_shapes(self, mesh_type):
        await self.base_setup()

        mesh, = await self._create_mesh_primitives([mesh_type])

        error_message = 'Mesh is flat, creating tetrahedral meshes for deformable simulation failed: ' + str(mesh.GetPath())

        #flatten sphere through scale
        if mesh_type == 'Sphere':
            physicsUtils.set_or_add_scale_op(mesh, Gf.Vec3f(1, 0, 1))

        with utils.ExpectMessage(self, error_message):
            self.assertTrue(await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh.GetPath()))
            get_physx_cooking_interface().cook_deformable_body_mesh(str(mesh.GetPath()))
            self._check_failed_cooking_state_deformable_body(mesh)

        #expect another message when start the simulation
        with utils.ExpectMessage(self, error_message):
            self._start()
            self._step(1)


        if mesh_type == 'Sphere':
            #flatten sphere after initial cooking
            await self.base_setup()
            mesh, = await self._create_mesh_primitives([mesh_type])
            await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh.GetPath())
            get_physx_cooking_interface().cook_deformable_body_mesh(str(mesh.GetPath()))

            physicsUtils.set_or_add_scale_op(mesh, Gf.Vec3f(1, 0, 1))

            #here we expect that no error message is issues, as typing in scales in the transform property
            #rollout can trigger with 0 values while typing
            await self._wait_for_deformable_body_cooking([mesh], asynchronous=True, max_updates=100)
            self._check_failed_cooking_state_deformable_body(mesh)

            #we still expect a message, when we finaly start the simulation
            with utils.ExpectMessage(self, error_message):
                self._start()
                self._step(1)


    #also for OM-46697, in case scaling one dim to 0
    async def test_mesh_softbody_fail_flat(self):
        mesh_type_list = ['Plane', 'Disk', 'Sphere']
        for mesh_type in mesh_type_list:
            await self._run_flat_shapes(mesh_type)

    async def test_softbody_change_hexahedral_resolution(self):
        await self.base_setup()

        mesh, = await self._create_mesh_primitives(['Sphere'])

        start_resolution = 5
        omni.kit.commands.execute("AddDeformableBodyComponent", skin_mesh_path=mesh.GetPath(), voxel_resolution=start_resolution)
        deformable_body_api = PhysxSchema.PhysxDeformableBodyAPI(mesh.GetPrim())
        hexResolutionAttr = mesh.GetPrim().GetAttribute("physxDeformable:simulationHexahedralResolution")
        self.assertTrue(deformable_body_api)
        simRestPointsAttr = deformable_body_api.GetSimulationRestPointsAttr()
        simIndicesAttr = PhysxSchema.PhysxDeformableAPI(deformable_body_api).GetSimulationIndicesAttr()

        #check we didn't already set the sim mesh but have the correct resolution set
        self.assertTrue(hexResolutionAttr and hexResolutionAttr.Get() == start_resolution)
        self.assertFalse(simRestPointsAttr.HasAuthoredValue())
        self.assertFalse(simIndicesAttr.HasAuthoredValue())

        #cook the sim mesh
        get_physx_cooking_interface().cook_deformable_body_mesh(str(mesh.GetPath()))
        self.assertTrue(simRestPointsAttr.HasAuthoredValue())
        self.assertTrue(simIndicesAttr.HasAuthoredValue())
        numRestPoints = len(simRestPointsAttr.Get())
        numIndices = len(simIndicesAttr.Get())
        self.assertTrue(numRestPoints > 0)
        self.assertTrue(numIndices > 0)

        #set new resolution and cook again
        new_resolution = start_resolution + 1
        hexResolutionAttr.Set(new_resolution)
        get_physx_cooking_interface().cook_deformable_body_mesh(str(mesh.GetPath()))
        #check sim mesh is available and is bigger
        self.assertTrue(simRestPointsAttr.HasAuthoredValue())
        self.assertTrue(simIndicesAttr.HasAuthoredValue())
        new_numRestPoints = len(simRestPointsAttr.Get())
        new_numIndices = len(simIndicesAttr.Get())
        self.assertTrue(new_numRestPoints > numRestPoints)
        self.assertTrue(new_numIndices > numIndices)


    async def test_create_softbody_during_play(self):
        await self.base_setup()
        stepper = utils.PhysicsStepper()

        mesh, = await self._create_mesh_primitives(['Torus'])
        mesh_transform_0 = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(await stepper.play_and_step_and_pause(1, 5))
        
        await self._runAddDeformableBodyComponentCommand(mesh.GetPath())

        self.assertTrue(PhysxSchema.PhysxDeformableBodyAPI.Get(self._stage, mesh.GetPath()))
        #play will cause softbody mesh to be cooked synchronously
        self.assertTrue(await stepper.play_and_step_and_pause(1, 5))
        self._check_soft_body(mesh.GetPath())

        self.assertTrue(await stepper.play_and_step_and_pause(10, 5))
        mesh_transform_1 = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(Gf.Transform(mesh_transform_1).GetTranslation()[1] < Gf.Transform(mesh_transform_0).GetTranslation()[1] - 10.0)

        self.assertTrue(await stepper.play_and_step_and_stop(1, 5))

        self.assertTrue(PhysxSchema.PhysxDeformableBodyAPI.Get(self._stage, mesh.GetPath()))
        self._check_soft_body(mesh.GetPath())


    # test for potential crash (OM-40997)
    async def test_deformable_body_restpoints(self):
        await self.base_setup()

        mesh, = await self._create_mesh_primitives(['Torus'])

        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh.GetPath())

        #make rest points count inconsistent with mesh points count deliberatly
        rest_points = mesh.GetPointsAttr().Get()[:-1]
        deformable_body = PhysxSchema.PhysxDeformableBodyAPI.Apply(mesh.GetPrim())
        PhysxSchema.PhysxDeformableAPI(deformable_body).CreateRestPointsAttr().Set(rest_points)

        # check warning report for resetting rest points
        message = 'PhysxSchemaPhysxDeformableAPI:restPoints and UsdGeomMesh:points have inconsistent size. Resetting the restPoints to UsdGeomMesh:points.'
        with utils.ExpectMessage(self, message):
            self._start()


    # test enable/disable deformable body
    async def test_deformable_body_enable_disable(self):
        await self.base_setup()

        mesh, = await self._create_mesh_primitives(['Torus'])

        # setup deformable body and record initial position
        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh.GetPath())
        deformable_body = PhysxSchema.PhysxDeformableBodyAPI.Apply(mesh.GetPrim())

        initial_translation = physicsUtils.get_translation(mesh.GetPrim())
        moved_delta = 10.0

        # simulate, check new position is lower than the initial one
        self._start()
        self._step(10)
        new_translation = physicsUtils.get_translation(mesh.GetPrim())
        self.assertLess(new_translation[1], initial_translation[1] - moved_delta)
        self._reset()

        # disable, simulate and check it didn't move
        PhysxSchema.PhysxDeformableAPI(deformable_body).GetDeformableEnabledAttr().Set(False)
        self._start()
        self._step(10)
        new_translation = physicsUtils.get_translation(mesh.GetPrim())
        self.assertEqual(new_translation, initial_translation)
        self._reset()

        # re-enable, simulate and check new position is lower than the initial one
        PhysxSchema.PhysxDeformableAPI(deformable_body).GetDeformableEnabledAttr().Set(True)
        self._start()
        self._step(10)
        new_translation = physicsUtils.get_translation(mesh.GetPrim())
        self.assertLess(new_translation[1], initial_translation[1] - moved_delta)


    async def test_deformable_body_pair_filter(self):
        await self.base_setup()
        self.add_groundplane()
        starting_height = 0.0

        sphere_mesh, = await self._create_mesh_primitives(['Sphere'], starting_height=starting_height)

        #place mesh just above ground
        initial_bounds = self.getWorldBounds(sphere_mesh)
        localTrafo = sphere_mesh.GetLocalTransformation()
        self.set_prim_translation(sphere_mesh.GetPrim(), localTrafo.ExtractTranslation() + Gf.Vec3d(0.0, initial_bounds.GetSize()[1] * 0.5, 0.0))

        #create deformable body
        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=sphere_mesh.GetPath())

        penetration_tol = 10.0

        #simulate and check that bound Ys have not moved below the surface:
        self._start()
        self._step(50)
        current_bounds = self.getWorldBounds(sphere_mesh)
        self.assertGreater(current_bounds.GetMin()[1], -penetration_tol)

        self._reset()

        #create filter
        filter_api = UsdPhysics.FilteredPairsAPI.Apply(sphere_mesh.GetPrim())
        plane_path = self._defaultPrimPath.AppendChild("CollisionPlane").AppendChild("CollisionPlane")
        print(plane_path)
        filter_api.GetFilteredPairsRel().AddTarget(Sdf.Path(plane_path))

        self._start()
        self._step(50)
        current_bounds = self.getWorldBounds(sphere_mesh)
        self.assertGreater(-penetration_tol, current_bounds.GetMin()[1])


    def _import_tetrahedral_mesh_base(self, with_surface_mesh: bool):
        self._target_mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/tetrahedral_mesh_conforming", False))
        if with_surface_mesh:
            self._target_surface_mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(
                self._stage, "/tetrahedral_mesh", False
            ))
        else:
            self._target_surface_mesh_path = None

        face_data = EmbeddedData.get_tet_mesh_face_data() if with_surface_mesh else ""
        success = omni.kit.commands.execute("ImportTetrahedralMeshCommand",
                                            target_tet_mesh_path=self._target_mesh_path,
                                            target_surface_mesh_path=self._target_surface_mesh_path,
                                            path_without_extension="",
                                            node_str=EmbeddedData.get_tet_mesh_node_data(),
                                            tet_str=EmbeddedData.get_tet_mesh_tetrahedron_data(),
                                            face_str=face_data
                                            )

        self.assertTrue(success[1])
        mesh_list = self._get_tet_mesh_list_and_assert(True, with_surface_mesh)

        return mesh_list

    def _get_tet_mesh_list_and_assert(self, assert_true:bool = True, with_surface_mesh:bool = True):
        mesh_list = [
            PhysxSchema.TetrahedralMesh.Get(self._stage, self._target_mesh_path)
        ]
        if with_surface_mesh:
            mesh_list.append(UsdGeom.Mesh.Get(self._stage, self._target_surface_mesh_path))

        for mesh in mesh_list:
            if assert_true:
                self.assertTrue(mesh)
            else:
                self.assertFalse(mesh)
        return mesh_list

    def _tet_mesh_undo_redo_base(self, with_surface_mesh: bool):
        omni.kit.undo.undo()
        self._get_tet_mesh_list_and_assert(False, with_surface_mesh)

        omni.kit.undo.redo()
        self._get_tet_mesh_list_and_assert(True, with_surface_mesh)

    async def test_import_tetrahedral_mesh(self):
        await self.base_setup()
        self._import_tetrahedral_mesh_base(False)
        self._tet_mesh_undo_redo_base(False)

    async def test_import_tetrahedral_mesh_with_surface(self):
        await self.base_setup()
        self._import_tetrahedral_mesh_base(True)
        self._tet_mesh_undo_redo_base(True)

    async def test_import_tetrahedral_mesh_fail(self):
        await self.base_setup()
        self._target_mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/tetrahedral_mesh", False))
        success = omni.kit.commands.execute(
            "ImportTetrahedralMeshCommand",
            target_tet_mesh_path=self._target_mesh_path,
            suppress_errors=True
        )
        self.assertFalse(success[1])

    async def test_import_tetrahedral_mesh_as_deformable(self):
        await self.base_setup()
        self.add_groundplane()
        mesh_list = self._import_tetrahedral_mesh_base(True)

        starting_height = 200.0
        translation_xyz = Gf.Vec3d(0.0, starting_height, 0.0)

        # move meshes up so they do not intersect the ground plane
        for mesh in mesh_list:
            mesh.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(translation_xyz)

        await self._runAddDeformableBodyComponentCommand(
            skin_mesh_path=self._target_surface_mesh_path,
            collision_mesh_path=self._target_mesh_path
        )
        self.assertTrue(PhysxSchema.PhysxDeformableBodyAPI.Get(self._stage, self._target_surface_mesh_path))

        # test that the resulting mesh actually does simulate
        deformable_body = mesh_list[1]

        self._start()
        self._step(20)

        # ensure that it moved:
        pause_height = deformable_body.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[1]
        self.assertLess(pause_height, starting_height)

        self._reset()

    async def test_regression_usd_api(self):
        await self.base_setup()

        # GetAttribute(...).Get(...) is not well documented and there might be no guarantee that the output variable is not modified when Get() fails because the attribute is not present
        # Check that Get() performs as expected
        # The same regression test is implemented as C++ unit test SUBCASE("USD API Regression Test")

        defaultPrim = self._stage.GetDefaultPrim()

        checkBool = True;
        defaultPrim.CreateAttribute("regressionTestBool", Sdf.ValueTypeNames.Bool, True).Set(checkBool)

        checkBool = False;
        checkBool = defaultPrim.GetAttribute("regressionTestBool").Get()
        self.assertTrue(checkBool)

        checkBool = defaultPrim.GetAttribute("regressionTestBoolBad").Get()
        self.assertTrue(checkBool == None)

        FLT_MAX = 3.4028230607370965e+38
        defaultPrim.CreateAttribute("regressionTestFloat", Sdf.ValueTypeNames.Float, True).Set(FLT_MAX)

        value = 0.0
        value = defaultPrim.GetAttribute("regressionTestFloat").Get()
        self.assertTrue(value == FLT_MAX)

        value = 0.0
        value = defaultPrim.GetAttribute("regressionTestFloatBad").Get()
        self.assertTrue(value == None)

    # Multi scenes
    async def test_physics_deformable_body_multi_scenes(self):
        await self.base_setup()
        prim_type_list = ['Cube', 'Cube']
        starting_height = 100.0
        mesh = await self._create_mesh_primitives(prim_type_list, starting_height=starting_height)

        for m in mesh:
            await self._runAddDeformableBodyComponentCommand(skin_mesh_path=m.GetPath())

        #scene 1 has 1/4 the default gravity so objects fall much slower
        scene1Path = Sdf.Path('/World/PhysicsScene1')
        metersPerUnit = UsdGeom.GetStageMetersPerUnit(self._stage)
        scene = UsdPhysics.Scene.Define(self._stage, scene1Path)
        scene.CreateGravityMagnitudeAttr().Set(9.8 / metersPerUnit / 4)

        defaultScenePath = Sdf.Path('/World/PhysicsScene')
        targets = [defaultScenePath]
        prim = self._stage.GetPrimAtPath(mesh[0].GetPath())
        deformableAPI = PhysxSchema.PhysxDeformableBodyAPI(prim)
        PhysxSchema.PhysxDeformableAPI(deformableAPI).GetSimulationOwnerRel().SetTargets(targets)

        targets = [scene1Path]
        prim = self._stage.GetPrimAtPath(mesh[1].GetPath())
        deformableAPI = PhysxSchema.PhysxDeformableBodyAPI(prim)
        PhysxSchema.PhysxDeformableAPI(deformableAPI).GetSimulationOwnerRel().SetTargets(targets)

        self._start()
        self._step(10)

        height0 = mesh[0].ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[1]
        height1 = mesh[1].ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[1]
        self.assertLess(height0, height1)

    async def test_physics_deformable_body_kinematic_setup(self):
        await self.base_setup()
        prim_type_list = ['Sphere']
        mesh, = await self._create_mesh_primitives(prim_type_list, starting_height=0.0)
        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh.GetPath())
        mesh.GetPrim().CreateAttribute("physxDeformable:kinematicEnabled", Sdf.ValueTypeNames.Bool, True).Set(True)

        position = Gf.Vec3d(0.0, 0.0, 0.0)
        positionEnd = Gf.Vec3d(0.0, 500.0, 0.0)
        xformable = UsdGeom.Xformable(mesh.GetPrim())
        ordered_xform_ops = xformable.GetOrderedXformOps()
        translateOp = None
        for op in ordered_xform_ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translateOp = op
        self.assertTrue(translateOp)
        translateOp.Set(time=0, value=position)
        translateOp.Set(time=50, value=positionEnd)

        self._start()
        self._step(1)

        xformMatrix = xformable.GetLocalTransformation(Usd.TimeCode(50))

        translate = xformMatrix.ExtractTranslation()
        
        epsilon = 0.001
        self.assertTrue(Gf.IsClose(translate, positionEnd, epsilon))

    async def _run_physics_deformable_body_kinematic_fallback(self, kinematic, transform_varying, mesh_varying):
        await self.base_setup()
        prim_type_list = ['Sphere']
        mesh, = await self._create_mesh_primitives(prim_type_list, starting_height=0.0)

        if transform_varying:
            position = Gf.Vec3d(0.0, 0.0, 0.0)
            positionEnd = Gf.Vec3d(0.0, 500.0, 0.0)
            xformable = UsdGeom.Xformable(mesh.GetPrim())
            ordered_xform_ops = xformable.GetOrderedXformOps()
            translateOp = None
            for op in ordered_xform_ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translateOp = op
            self.assertTrue(translateOp)
            translateOp.Set(time=0, value=position)
            translateOp.Set(time=50, value=positionEnd)

        if mesh_varying:
            default_points = mesh.GetPointsAttr().Get()
            moved_points = [x + Gf.Vec3f(0.0, 500.0, 0.0) for x in default_points]
            mesh.GetPointsAttr().Set(default_points, time=0)
            mesh.GetPointsAttr().Set(moved_points, time=50)

        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh.GetPath())
        mesh.GetPrim().CreateAttribute("physxDeformable:kinematicEnabled", Sdf.ValueTypeNames.Bool, True).Set(kinematic)

        self._start()
        self._step(1)

        #no sim stats for deformables available! check for sim mesh equals coll mesh
        deformableAPI = PhysxSchema.PhysxDeformableBodyAPI(mesh.GetPrim())
        collisionIndices = deformableAPI.GetCollisionIndicesAttr().Get()
        simulationIndices = PhysxSchema.PhysxDeformableAPI(deformableAPI).GetSimulationIndicesAttr().Get()

        equal_sim_coll = len(collisionIndices) == len(simulationIndices)
        self.assertTrue((kinematic or transform_varying or mesh_varying)  == equal_sim_coll)

    async def test_physics_deformable_body_kinematic_fallback(self):
        await self._run_physics_deformable_body_kinematic_fallback(False, False, False)
        await self._run_physics_deformable_body_kinematic_fallback(False, True, False)
        await self._run_physics_deformable_body_kinematic_fallback(False, False, True)
        await self._run_physics_deformable_body_kinematic_fallback(True, False, False)
        await self._run_physics_deformable_body_kinematic_fallback(True, True, False)
        await self._run_physics_deformable_body_kinematic_fallback(True, False, True)


    #regression test for OM-103699
    async def test_physics_deformable_body_kinematic_inconsistent_update(self):
        await self.base_setup()
        prim_type_list = ['Cube']
        mesh, = await self._create_mesh_primitives(prim_type_list, starting_height=0.0)

        timestep = self._get_time_step()
        points = list(mesh.GetPointsAttr().Get())
        mesh.GetPointsAttr().Set(points, time=0.0) #get points and set them at time = 0
        points.clear()
        mesh.GetPointsAttr().Set(points, time=timestep) #remove all points at time = timestep

        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh.GetPath())
        mesh.GetPrim().CreateAttribute("physxDeformable:kinematicEnabled", Sdf.ValueTypeNames.Bool, True).Set(True)

        self._start()
        self._step(1)

        message = f"Size of points of {mesh.GetPath()} has changed - skipping kinematic update."
        with utils.ExpectMessage(self, message):
            self._step(1)


    #regression test for OM-103698
    async def test_physics_deformable_body_inconsistent_update(self):
        await self.base_setup()
        prim_type_list = ['Cube']
        mesh, = await self._create_mesh_primitives(prim_type_list, starting_height=0.0)

        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh.GetPath())

        self._start()
        self._step(1)

        deformable_body = PhysxSchema.PhysxDeformableBodyAPI(mesh)

        #clear sim points
        sim_points = list(deformable_body.GetSimulationPointsAttr().Get())
        sim_points.clear()

        message = f"Size of physxDeformable:simulationPoints of {mesh.GetPath()} has changed - skipping update."
        with utils.ExpectMessage(self, message, expected_result=True):
            deformable_body.GetSimulationPointsAttr().Set(sim_points)
            self._step(1)

        #clear sim velocities
        deformable = PhysxSchema.PhysxDeformableAPI(mesh)
        sim_velocities = list(deformable.GetSimulationVelocitiesAttr().Get())
        sim_velocities.clear()

        message = f"Size of physxDeformable:simulationVelocities of {mesh.GetPath()} has changed - skipping update."
        with utils.ExpectMessage(self, message, expected_result=True):
            deformable.GetSimulationVelocitiesAttr().Set(sim_velocities)
            self._step(1)

    #regression test for catching more invalid conditions during parsing for OM-110476
    async def helper_physics_deformable_body_inconsistent_setup(self):
        await self.base_setup()
        prim_type_list = ['Cone']
        mesh, = await self._create_mesh_primitives(prim_type_list, starting_height=0.0)
        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh.GetPath())
        deformable_body = PhysxSchema.PhysxDeformableBodyAPI(mesh)
        deformable = PhysxSchema.PhysxDeformableAPI(mesh)
        return deformable, deformable_body

    async def test_physics_deformable_body_inconsistent_setup(self):

        message = f"PhysxSchemaPhysxDeformableAPI:simulationVelocities and simulation points have inconsistent sizes."
        deformable, deformable_body = await self.helper_physics_deformable_body_inconsistent_setup()
        with utils.ExpectMessage(self, message, expected_result=True):
            deformable.GetSimulationVelocitiesAttr().Set([Gf.Vec3f(0,0,0)])
            self._start()
            self._reset()

        message = f"PhysxSchemaPhysxDeformableBodyAPI:simulationRestPoints and :simulationPoints have inconsistent sizes."
        deformable, deformable_body = await self.helper_physics_deformable_body_inconsistent_setup()
        with utils.ExpectMessage(self, message, expected_result=True):
            deformable_body.GetSimulationRestPointsAttr().Set([Gf.Vec3f(0,0,0)])
            self._start()
            self._reset()

        message = f"PhysxSchemaPhysxDeformableAPI:simulationindices has invalid size or contains out of range indices."
        deformable, deformable_body = await self.helper_physics_deformable_body_inconsistent_setup()
        with utils.ExpectMessage(self, message, expected_result=True):
            indices_list = list(deformable.GetSimulationIndicesAttr().Get())
            #remove last element
            deformable.GetSimulationIndicesAttr().Set(indices_list[:-1])
            self._start()
            self._reset()

        message = f"PhysxSchemaPhysxDeformableAPI:simulationindices has invalid size or contains out of range indices."
        deformable, deformable_body = await self.helper_physics_deformable_body_inconsistent_setup()
        with utils.ExpectMessage(self, message, expected_result=True):
            indices_list = list(deformable.GetSimulationIndicesAttr().Get())
            #make index 10 invalid
            deformable.GetSimulationIndicesAttr().Set(indices_list[:10] + [9999999] + indices_list[11:]) 
            self._start()
            self._reset()

        message = f"PhysxSchemaPhysxDeformableBodyAPI:collisionRestPoints and collisionPoints have inconsistent sizes."
        deformable, deformable_body = await self.helper_physics_deformable_body_inconsistent_setup()
        with utils.ExpectMessage(self, message, expected_result=True):
            deformable_body.GetCollisionRestPointsAttr().Set([Gf.Vec3f(0,0,0)])
            self._start()
            self._reset()

        message = f"PhysxSchemaPhysxDeformableBodyAPI:collisionindices has invalid size or contains out of range indices."
        deformable, deformable_body = await self.helper_physics_deformable_body_inconsistent_setup()
        with utils.ExpectMessage(self, message, expected_result=True):
            indices_list = list(deformable_body.GetCollisionIndicesAttr().Get())
            #remove last element
            deformable_body.GetCollisionIndicesAttr().Set(indices_list[:-1])
            self._start()
            self._reset()

        message = f"PhysxSchemaPhysxDeformableBodyAPI:collisionindices has invalid size or contains out of range indices."
        deformable, deformable_body = await self.helper_physics_deformable_body_inconsistent_setup()
        with utils.ExpectMessage(self, message, expected_result=True):
            indices_list = list(deformable_body.GetCollisionIndicesAttr().Get())
            #make index 10 invalid
            deformable_body.GetCollisionIndicesAttr().Set(indices_list[:10] + [9999999] + indices_list[11:]) 
            self._start()
            self._reset()

        #not testing custom attributes now, they seem much less likely to fail in practice

    #regression test for OM-116228
    @unittest.skip("missing functionality to expect error by substring")
    async def test_physics_deformable_body_highres_crashing(self):
        await self.base_setup()
        self.add_groundplane()
        prim_type_list = ['Cube']
        mesh, = await self._create_mesh_primitives(prim_type_list, starting_height=50.0)

        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh.GetPath())

        message0 = f"PhysX error: NpSoftBody::attachSimulationMesh: simulation mesh contains too many tetrahedons, see PX_MAX_NB_SOFTBODY_TET"
        message1 = f"PhysX Deformable Body creation failed."
        with utils.ExpectMessage(self, [message0, message1], expected_result=True):
            mesh.GetPrim().GetAttribute("physxDeformable:simulationHexahedralResolution").Set(60)
            self._start()
            self._step(1)

    # PX-4406 - crash because input indices are invalid
    async def test_deformable_body_invalid_mesh(self):
        await self.base_setup()
        self.add_groundplane()
        prim_type_list = ['Torus']
        mesh, = await self._create_mesh_primitives(prim_type_list, starting_height=50.0)

        error_message = 'Mesh is invalid, creating tetrahedral meshes for deformable simulation failed: /World/Torus'
        with utils.ExpectMessage(self, error_message, expected_result=True):
            indices_list = list(mesh.GetFaceVertexIndicesAttr().Get())
            #make index 10 invalid
            mesh.GetFaceVertexIndicesAttr().Set(indices_list[:10] + [9999999] + indices_list[11:]) 
            await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh.GetPath())

        with utils.ExpectMessage(self, error_message, expected_result=True):
            self._start()
            self._step(1)
            self._reset()

    async def test_deformable_body_zero_volume(self):
        await self.base_setup()
        self.add_groundplane()

        # Create grid mesh with zero volume
        mesh_path = self._defaultPrimPath.AppendChild("Grid")
        mesh = UsdGeom.Mesh.Define(self._stage, mesh_path)
        dimx=32
        dimy=32
        scale=50.0
        tri_points, tri_indices = deformableUtils.create_triangle_mesh_square(dimx, dimy, scale)
        mesh.GetPointsAttr().Set(tri_points)
        mesh.GetFaceVertexIndicesAttr().Set(tri_indices)
        mesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))

        error_message = 'Mesh is flat, creating tetrahedral meshes for deformable simulation failed: /World/Grid'
        with utils.ExpectMessage(self, error_message, expected_result=True):
            await self._runAddDeformableBodyComponentCommand(skin_mesh_path=mesh.GetPath())
        
        with utils.ExpectMessage(self, error_message, expected_result=True):
            self._start()
            self._step(1)
            self._reset()   
