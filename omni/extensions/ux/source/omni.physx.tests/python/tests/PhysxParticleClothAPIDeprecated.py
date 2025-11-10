# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.test
from omni.kit.commands import execute
from pxr import UsdLux, Gf, Sdf, UsdGeom, PhysxSchema, UsdPhysics, Usd
from omni.physx import get_physx_interface, get_physx_cooking_interface, get_physx_cooking_private_interface, get_physx_simulation_interface
from omni.physxcommands import AddPhysicsSceneCommand, AddGroundPlaneCommand
from omni.physxtests import utils
from omni.physx.scripts import particleUtils, deformableUtils, physicsUtils
import unittest
from omni.physxtests.utils.physicsBase import PhysicsBaseAsyncTestCase, TestCategory
import carb.settings
import os


class PhysxParticleClothAPITestAsync(PhysicsBaseAsyncTestCase):
    category = TestCategory.Core
    
    async def base_setup(self, usd_file: str = None):
        self.fail_on_log_error = True

        if usd_file is not None:
            await self._open_usd(usd_file)
            self._stage = omni.usd.get_context().get_stage()
        else:
            self._stage = await utils.new_stage_setup()
            default_prim_xform = UsdGeom.Xform.Define(self._stage, "/World")
            self._stage.SetDefaultPrim(default_prim_xform.GetPrim())

        self._up_axis = UsdGeom.GetStageUpAxis(self._stage)
        self._default_prim_path = self._stage.GetDefaultPrim().GetPath()
        self._stepper = utils.PhysicsStepper()

        #add physics scene
        self._scene_path = self._default_prim_path.AppendChild("PhysicsScene")
        execute("AddPhysicsScene", stage=self._stage, path=str(self._scene_path))

        sphereLight = UsdLux.SphereLight.Define(self._stage, Sdf.Path("/SphereLight"))
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

    @staticmethod
    def _get_time_step():
        return 1.0 / 60.0

    def _start(self):
        physx_interface = get_physx_interface()
        physx_interface.start_simulation()

    def _step(self, num_steps):
        physx_interface = get_physx_interface()
        time = 0.0
        dtime = self._get_time_step()
        for i in range(num_steps):
            physx_interface.update_simulation(dtime, time)
            physx_interface.update_transformations(True, True, True, False)
            time = time + dtime

    def _reset(self):
        physx_interface = get_physx_interface()
        physx_interface.reset_simulation()

    def add_groundplane(self):
        AddGroundPlaneCommand.execute(self._stage, '/CollisionPlane', 
            self._up_axis, 1000.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def _create_tesselated_plane_mesh(self, u_resolution=10, v_resolution=10, scale=100.0):

        success, mesh_path = omni.kit.commands.execute(
            "CreateMeshPrimWithDefaultXform",
            prim_type="Plane",
            u_patches=u_resolution,
            v_patches=v_resolution,
            u_verts_scale=1,
            v_verts_scale=1,
            half_scale=scale * 0.5,
        )
        self.assertTrue(success)

        return UsdGeom.Mesh.Get(self._stage, mesh_path)

    def _create_mesh_primitives(self, prim_type_list):
        mesh_list = []
        for prim_type in prim_type_list:
            success, mesh_path = execute("CreateMeshPrimWithDefaultXform", prim_type=prim_type)
            mesh = UsdGeom.Mesh.Get(self._stage, mesh_path)
            self.assertTrue(mesh)
            mesh_list.append(mesh)
        
        height = 60
        offset = 150
        origin = Gf.Vec3d(-offset*3/2, height, -offset*3/2)
        for i in range(3):
            for j in range(3):
                index = i*3+j
                if index < len(mesh_list):
                    physicsUtils.set_or_add_translate_op(mesh_list[index], origin + Gf.Vec3d(i*offset, 0, j*offset))
                    
        return mesh_list

    def select_prim(self, prim):
        old_selected_paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        execute("SelectPrims", old_selected_paths=old_selected_paths, new_selected_paths=[str(prim.GetPath())], expand_in_stage=False)

    def unselect_all(self):
        old_selected_paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        execute("SelectPrims", old_selected_paths=old_selected_paths, new_selected_paths=[], expand_in_stage=False)

    def add_particle_cloth_to_mesh_command(self, mesh):
        self.select_prim(mesh)
        execute("AddParticleClothComponent", prim_path=mesh.GetPrim().GetPath())
        self.unselect_all()

        #check PhysxParticleClothAPI and PhysxAutoParticleClothAPI got applied
        particle_cloth_api = PhysxSchema.PhysxParticleClothAPI(mesh)
        self.assertTrue(particle_cloth_api)
        auto_particle_cloth_api = PhysxSchema.PhysxAutoParticleClothAPI(mesh)
        self.assertTrue(auto_particle_cloth_api)
        
        return particle_cloth_api

    def add_particle_cloth(self, particle_system, dimx=32, dimy=32, scale=50.0):
        # Create grid mesh
        cloth_mesh_path = self._default_prim_path.AppendChild("cloth")
        cloth_mesh = UsdGeom.Mesh.Define(self._stage, cloth_mesh_path)
        tri_points, tri_indices = deformableUtils.create_triangle_mesh_square(dimx=dimx, dimy=dimy, scale=scale)
        cloth_mesh.GetPointsAttr().Set(tri_points)
        cloth_mesh.GetFaceVertexIndicesAttr().Set(tri_indices)
        cloth_mesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))

        particleUtils.add_physx_particle_cloth(
            stage=self._stage,
            path=cloth_mesh_path,
            dynamic_mesh_path=None,
            particle_system_path=particle_system.GetPath()
        )

        particleMass = 0.02
        massApi = UsdPhysics.MassAPI.Apply(cloth_mesh.GetPrim())
        massApi.GetMassAttr().Set(particleMass * len(tri_points))
        return cloth_mesh

    def _create_particle_system(self, path="particleSystem"):
        particle_system_path = self._default_prim_path.AppendChild(path)
        particleUtils.add_physx_particle_system(
            stage=self._stage,
            particle_system_path=particle_system_path,
            simulation_owner=self._scene_path,
        )
        particle_system = PhysxSchema.PhysxParticleSystem.Get(self._stage, particle_system_path)
        return particle_system

    def _get_particle_system_path(self, mesh):
        particles = PhysxSchema.PhysxParticleAPI(mesh)
        self.assertTrue(particles)
        targets = particles.GetParticleSystemRel().GetTargets()
        self.assertTrue(len(targets) == 1)
        particle_system_path = targets[0]
        return particle_system_path

    def _assertAlmostEqualVector3(self, vectorA: Gf.Vec3f, vectorB: Gf.Vec3f, places):
        for a, b in zip(vectorA, vectorB):
            self.assertAlmostEqual(a, b, places=places)

    def _assert_transform_close(self, transform: Gf.Matrix4d, reference_transform: Gf.Matrix4d, places: int):
        for va, vb in zip(transform, reference_transform):
            for a, b in zip(va, vb):
                self.assertAlmostEqual(a, b, places=places)

    async def _run_particle_cloth_reset_transform_test(self, deleteAttributeDuringSim=False):
        await self.base_setup()

        particle_system = self._create_particle_system()
        cloth_mesh = self.add_particle_cloth(particle_system)
        # setup a transform:
        init_translation = Gf.Vec3f(1.5, 100.0, 10.5)
        init_rotx = 5.0
        cloth_mesh.SetResetXformStack(True)
        cloth_mesh.AddTranslateOp().Set(init_translation)
        cloth_mesh.AddRotateXOp().Set(init_rotx)

        # step and check that it moved:
        self._step(num_steps=5)
        pos = cloth_mesh.GetPrim().GetAttribute("xformOp:translate").Get()
        self.assertLess(pos[1], init_translation[1])

        sanitized_xform_ops = cloth_mesh.GetOrderedXformOps()
        self.assertEqual(len(sanitized_xform_ops), 3)
        opNames = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
        for op, opName in zip(sanitized_xform_ops, opNames):
            self.assertEqual(op.GetName(), opName)
        self.assertTrue(cloth_mesh.GetResetXformStack())

        if deleteAttributeDuringSim:
            cloth_mesh.GetPrim().RemoveProperty("xformOp:rotateX")

        # reset and check that original ops order is restored and ops attributes as well
        omni.physx.get_physx_interface().reset_simulation()

        reset_xform_ops = cloth_mesh.GetOrderedXformOps()
        self.assertEqual(len(reset_xform_ops), 2)
        self.assertTrue(cloth_mesh.GetResetXformStack())

        self.assertEqual(reset_xform_ops[0].GetOpType(), UsdGeom.XformOp.TypeTranslate)
        self.assertEqual(reset_xform_ops[0].Get(), init_translation)

        self.assertEqual(reset_xform_ops[1].GetOpType(), UsdGeom.XformOp.TypeRotateX)
        self.assertEqual(reset_xform_ops[1].Get(), init_rotx)

        # check that orphaned standard ops are removed:
        self.assertFalse(cloth_mesh.GetPrim().GetAttribute("xformOp:scale"))
        self.assertFalse(cloth_mesh.GetPrim().GetAttribute("xformOp:orient"))

    async def check_particle_cloth_moved(self, cloth_mesh):
        init_translation = Gf.Vec3f(1.5, 100.0, 10.5)
        cloth_mesh.AddTranslateOp().Set(init_translation)
        particle_system_path = self._get_particle_system_path(cloth_mesh)
        particle_system_prim = self._stage.GetPrimAtPath(particle_system_path)
        particle_system = PhysxSchema.PhysxParticleSystem(particle_system_prim)

        cloth_prim = cloth_mesh.GetPrim()

        # disable particle system, step and check that it does NOT move
        particle_system.CreateParticleSystemEnabledAttr().Set(False)
        self._step(num_steps=5)
        pos = cloth_prim.GetAttribute("xformOp:translate").Get()
        self.assertEqual(pos[1], init_translation[1])

        # enable particle system, step and check that it does move
        particle_system.CreateParticleSystemEnabledAttr().Set(True)
        self._step(num_steps=5)
        pos = cloth_prim.GetAttribute("xformOp:translate").Get()
        self.assertLess(pos[1], init_translation[1])

        particle_cloth_api = PhysxSchema.PhysxParticleClothAPI.Get(self._stage, cloth_prim.GetPath())
        self.assertTrue(particle_cloth_api)

        current_translation = cloth_prim.GetAttribute("xformOp:translate").Get()

        # disable particle cloth, step and check that it does NOT move
        PhysxSchema.PhysxParticleAPI(particle_cloth_api).CreateParticleEnabledAttr().Set(False)
        self._step(num_steps=5)
        pos = cloth_prim.GetAttribute("xformOp:translate").Get()
        self.assertEqual(pos[1], current_translation[1])

        # enable particle cloth, step and check that it does move
        PhysxSchema.PhysxParticleAPI(particle_cloth_api).CreateParticleEnabledAttr().Set(True)
        self._step(num_steps=5)
        pos = cloth_prim.GetAttribute("xformOp:translate").Get()
        self.assertLess(pos[1], current_translation[1])

    ## particle cloth tests

    async def test_particle_cloth_reset_transform(self):
        await self._run_particle_cloth_reset_transform_test()

    async def test_particle_cloth_reset_transform_delete_xformop(self):
        await self._run_particle_cloth_reset_transform_test(deleteAttributeDuringSim=True)

    async def test_particle_cloth_enable_disable(self):
        await self.base_setup()
        particle_system = self._create_particle_system()
        cloth_mesh = self.add_particle_cloth(particle_system)

        await self.check_particle_cloth_moved(cloth_mesh)

    # OM-36091
    async def test_particle_cloth_save_restore(self):
        await self.base_setup()

        filename = "particles.usda"

        # add some cloth
        particle_system = self._create_particle_system()
        cloth_mesh = self.add_particle_cloth(particle_system)

        # add some translation
        init_translation = Gf.Vec3f(1.5, 100.0, 10.5)
        cloth_mesh.AddTranslateOp().Set(init_translation)

        # run
        self._step(num_steps=5)

        # check that it moved
        cloth_prim = cloth_mesh.GetPrim()
        pos = cloth_prim.GetAttribute("xformOp:translate").Get()
        self.assertLess(pos[1], init_translation[1])

        # get the initial rest points & rest spring lengths
        clothAPI = PhysxSchema.PhysxParticleClothAPI(cloth_mesh.GetPrim())
        restPoints = clothAPI.GetRestPointsAttr().Get()
        springLengths = clothAPI.GetSpringRestLengthsAttr().Get()

        # save the points & indices
        meshPoints = cloth_mesh.GetPointsAttr().Get()
        meshIndices = cloth_mesh.GetFaceVertexIndicesAttr().Get()

        # save
        self._stage.Export(filename)

        # rest points & rest spring lengths should still be the same
        restPoints2 = clothAPI.GetRestPointsAttr().Get()
        springLengths2 = clothAPI.GetSpringRestLengthsAttr().Get()
        self.assertEqual(restPoints, restPoints2)
        self.assertEqual(springLengths, springLengths2)

        # run some more
        self._step(num_steps=5)

        # check that it moved
        pos1 = cloth_prim.GetAttribute("xformOp:translate").Get()
        self.assertLess(pos1[1], pos[1])

        # load the saved file
        await omni.usd.get_context().open_stage_async(filename)
        new_stage = omni.usd.get_context().get_stage()

        # check position again - should be same as before save
        cloth_prim = new_stage.GetPrimAtPath(cloth_mesh.GetPath())
        pos2 = cloth_prim.GetAttribute("xformOp:translate").Get()
        self.assertEqual(pos2, pos)

        # check points & indices as well
        cloth_mesh = UsdGeom.Mesh(cloth_prim)
        meshPoints1 = cloth_mesh.GetPointsAttr().Get()
        meshIndices1 = cloth_mesh.GetFaceVertexIndicesAttr().Get()

        self.assertEqual(meshPoints, meshPoints1)
        self.assertEqual(meshIndices, meshIndices1)

        # get rest points from file, should still be the same
        clothAPI = PhysxSchema.PhysxParticleClothAPI(cloth_prim)
        restPoints3 = clothAPI.GetRestPointsAttr().Get()
        springLengths3 = clothAPI.GetSpringRestLengthsAttr().Get()

        self.assertEqual(restPoints, restPoints3)
        self.assertEqual(springLengths, springLengths3)

        # run
        self._step(num_steps=5)

        # check if it moves
        pos3 = cloth_prim.GetAttribute("xformOp:translate").Get()
        self.assertLess(pos3[1], pos2[1])

        # run + stop
        self._step(num_steps=5)
        self._reset()

        # check if in same place as restore
        pos4 = cloth_prim.GetAttribute("xformOp:translate").Get()
        self.assertEqual(pos4, pos2)

        os.remove(filename)

    async def test_add_particle_cloth(self):
        await self.base_setup()
        
        mesh, = self._create_mesh_primitives(['Cube'])
        self.add_particle_cloth_to_mesh_command(mesh)

    async def test_undo_redo_add_particle_cloth(self):
        await self.base_setup()
        
        mesh, = self._create_mesh_primitives(['Cube'])
        self.add_particle_cloth_to_mesh_command(mesh)
        
        #need to undo select, unselect as well
        omni.kit.undo.undo()
        omni.kit.undo.undo()
        omni.kit.undo.undo()
        
        #check application of API was undone
        cloth = PhysxSchema.PhysxParticleClothAPI.Get(self._stage, mesh.GetPath())
        self.assertTrue(not cloth)  
        
        #redo
        omni.kit.undo.redo()
        omni.kit.undo.redo()
        omni.kit.undo.redo()
        
        #check application of API was redone
        cloth = PhysxSchema.PhysxParticleClothAPI.Get(self._stage, mesh.GetPath())
        self.assertTrue(cloth)  

    async def test_remove_particle_cloth(self):
        await self.base_setup()
        
        mesh, = self._create_mesh_primitives(['Cube'])
        self.add_particle_cloth_to_mesh_command(mesh)  
        
        self.select_prim(mesh)
        execute("RemoveParticleClothComponent", prim_path=mesh.GetPrim().GetPath())
        self.unselect_all()
        
        cloth = PhysxSchema.PhysxParticleClothAPI.Get(self._stage, mesh.GetPath())
        self.assertTrue(not cloth)

    async def test_undo_redo_remove_particle_cloth(self):
        await self.base_setup()
        
        mesh, = self._create_mesh_primitives(['Cube'])
        self.add_particle_cloth_to_mesh_command(mesh)
        
        self.select_prim(mesh)
        execute("RemoveParticleClothComponent", prim_path=mesh.GetPrim().GetPath())
        self.unselect_all()
        
        #need to undo select, unselect as well
        omni.kit.undo.undo()
        omni.kit.undo.undo()
        omni.kit.undo.undo()
        
        #check application of API was undone
        cloth = PhysxSchema.PhysxParticleClothAPI.Get(self._stage, mesh.GetPath())
        self.assertTrue(cloth)  
        
        #redo
        omni.kit.undo.redo()
        omni.kit.undo.redo()
        omni.kit.undo.redo()
        
        #check application of API was redone
        cloth = PhysxSchema.PhysxParticleClothAPI.Get(self._stage, mesh.GetPath())
        self.assertTrue(not cloth)  

    #used to not work ("OM-22238")
    async def test_particle_cloth_make_inflatable(self):
        await self.base_setup()
        mesh, = self._create_mesh_primitives(['Cube'])
        particle_cloth = self.add_particle_cloth_to_mesh_command(mesh)
        particle_cloth.GetPressureAttr().Set(1.2)

    # this test creates a cloth from a mesh primitive exposed in Create->Mesh, default uv settings
    # lets it settle on the ground plane, and tests for the bounds to be at the 
    # expected locations
    async def _run_particle_cloth_on_mesh_primitive(self, mesh_type: str):
        await self.base_setup()
        self.add_groundplane()

        mesh, = self._create_mesh_primitives([mesh_type])

        # set cloth (which will create a copy of the mesh and apply the ClothAPI)
        cloth = self.add_particle_cloth_to_mesh_command(mesh)
        # await self.wait(10)

        self.assertTrue(await self._stepper.play_and_step_and_pause(25, 10))

        # await self.wait(10)
        # check cloth collided with plane
        # check that cloth frame moved along
        xformable = UsdGeom.Xformable(cloth)
        globalPose = xformable.ComputeLocalToWorldTransform(0)
        trans = globalPose.ExtractTranslation()
        self.assertTrue(trans[1] > 0.0)

    @staticmethod
    def getWorldBounds(imageable: UsdGeom.Imageable) -> Gf.Range3d:
        obb = imageable.ComputeWorldBound(Usd.TimeCode.Default(), purpose1=imageable.GetPurposeAttr().Get())
        return obb.ComputeAlignedBox()

    #used to crash ("OM-47483")
    async def test_particle_cloth_on_standard_mesh_prims(self):
        mesh_type_list = ['Cube', 'Plane', 'Torus', 'Sphere', 'Cone', 'Cylinder', 'Disk']
        for mesh_type in mesh_type_list:
            await self._run_particle_cloth_on_mesh_primitive(mesh_type)

    async def test_particle_cloth_stale_particle_system_rel(self):
        await self.base_setup()

        #create a mesh
        mesh = self._create_tesselated_plane_mesh()

        #add particle cloth component, this will create a ParticleSystem
        self.add_particle_cloth_to_mesh_command(mesh)

        #check ParticleSystem reference and get path
        particle_system_path = self._get_particle_system_path(mesh)

        #make sure there is a particle system
        particle_system = PhysxSchema.PhysxParticleSystem.Get(self._stage, particle_system_path)
        self.assertTrue(particle_system)

        #remove ParticleSystem
        self._stage.RemovePrim(particle_system_path)

        #simulation shouldn't crash
        self._start()
        self._step(1)

    async def _open_usd(self, filename):
        schema_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, "../../../../data/")))
        schema_folder = schema_folder.replace("\\", "/") + "/"

        #print(schema_folder + filename + ".usda")
        await omni.usd.get_context().open_stage_async(schema_folder + filename + ".usda")

        usd_context = omni.usd.get_context()
        self.assertIn(filename, usd_context.get_stage_url())

    #automation test for OM-48582
    async def test_particle_cloth_pull_to_origin(self):
        await self.base_setup(usd_file = "generatedLegacyMeshes")

        mesh = UsdGeom.Mesh.Get(self._stage, '/World/' + 'Disk')
        physicsUtils.set_or_add_translate_op(mesh, Gf.Vec3d(0, 100, 0))
        cloth = self.add_particle_cloth_to_mesh_command(mesh)

        cloth_auto_api = PhysxSchema.PhysxAutoParticleClothAPI.Apply(cloth.GetPrim())
        cloth_auto_api.CreateDisableMeshWeldingAttr().Set(True)

        self._start()
        self._step(1)

        # bounds should not contain origin
        bounds = self.getWorldBounds(mesh)
        self.assertFalse(bounds.Contains(Gf.Vec3d(0, 0, 0)))

    async def _wait_cooking_finished(self):
        while True:
            await omni.kit.app.get_app().next_update_async()
            cooking_statistics = get_physx_cooking_private_interface().get_cooking_statistics()
            running_tasks = cooking_statistics.total_scheduled_tasks - cooking_statistics.total_finished_tasks
            self.assertGreaterEqual(running_tasks, 0)
            if running_tasks <= 0:
                break

    #test re-cook sensitivity on transform changes:
    async def _run_recooking_test(self, change_type: str, sync_type: str):
        await self.base_setup()

        #create mesh
        mesh_list = self._create_mesh_primitives(['Torus', 'Sphere'])
        mesh_list.append(self._create_tesselated_plane_mesh())

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

        for mesh in mesh_list:
            #create particle cloth
            self.add_particle_cloth_to_mesh_command(mesh)

        #cook
        await self._wait_cooking_finished()

        #check crc is set
        initial_cloth_crc_list = []
        for mesh in mesh_list:
            cloth_crc_attr = mesh.GetPrim().GetAttribute("physxParticle:clothDataInputCrc")
            self.assertTrue(cloth_crc_attr)

            #store initial_crc
            initial_cloth_crc_list.append(cloth_crc_attr.Get())

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
        elif sync_type == "sync_wait":
            await self._wait_cooking_finished()

        # check expected CRCs
        for mesh, initial_cloth_crc in zip(mesh_list, initial_cloth_crc_list):
            cloth_crc_attr = mesh.GetPrim().GetAttribute("physxParticle:clothDataInputCrc")
            self.assertTrue(cloth_crc_attr)

            #non-uniform scaled meshes can still share all the cooked data for particle cloth
            if change_type in ["change_translation", "change_rotation", "change_scale_uniform", "change_scale_non_uniform", "change_stack_scale_non_uniform"]:
                self.assertTrue(initial_cloth_crc == cloth_crc_attr.Get())
            else: #leaving this code path in case cloth cooking becomes sensitive to non-isometric transforms
                self.assertFalse(initial_cloth_crc == cloth_crc_attr.Get())

    #test re-cook sensitivity on transform changes:
    async def test_particle_cloth_recooking_mesh_on_transform_changes(self):
        for sync_type in ["sync_sim", "sync_wait"]:
            for change_type in ["change_translation", "change_rotation", "change_scale_uniform", "change_scale_non_uniform", "change_stack_scale_non_uniform"]:
                await self._run_recooking_test(change_type=change_type, sync_type=sync_type)

    #test that cooking triggering has no effect during simulation (warning for cooking parameters, no warning for scale) 
    async def test_particle_cloth_trigger_recooking_mesh_during_play(self):
        await self.base_setup()

        mesh, = self._create_mesh_primitives(['Torus'])

        #create particle cloth
        self.add_particle_cloth_to_mesh_command(mesh)

        auto_cloth_api = PhysxSchema.PhysxAutoParticleClothAPI(mesh.GetPrim())
        self.assertTrue(auto_cloth_api)

        stretch_attr = auto_cloth_api.GetSpringStretchStiffnessAttr()
        welding_attr = auto_cloth_api.GetDisableMeshWeldingAttr()
        scale_attr = mesh.GetPrim().GetAttribute("xformOp:scale")

        self.assertTrue(stretch_attr)
        self.assertTrue(welding_attr)
        self.assertTrue(scale_attr)

        initial_stretch = stretch_attr.Get()
        initial_welding = welding_attr.Get()
        initial_scale = scale_attr.Get()

        #wait for cooking to write crc
        await self._wait_cooking_finished()

        cloth_crc_attr = mesh.GetPrim().GetAttribute("physxParticle:clothDataInputCrc")
        self.assertTrue(cloth_crc_attr)
        initial_cloth_crc = cloth_crc_attr.Get()

        self.assertTrue(await self._stepper.play_and_step_and_pause(1, 5))

        error_message = f"Changing particle cloth mesh or PhysxSchemaPhysxAutoParticleClothAPI parameter during simulation is not supported. Prim: {mesh.GetPrim().GetPath()}"
        #set stretch parameter
        with utils.AssertErrorMessage(self, error_message):
            stretch_attr.Set(initial_stretch+0.01)
            await omni.kit.app.get_app().next_update_async()

        #set welding parameter
        with utils.AssertErrorMessage(self, error_message):
            welding_attr.Set(not initial_welding)
            await omni.kit.app.get_app().next_update_async()

        #set scale
        new_scale = initial_scale
        new_scale[2] = new_scale[2]*0.5
        scale_attr.Set(new_scale)
        await omni.kit.app.get_app().next_update_async()

        #check crc
        await self._wait_cooking_finished()
        new_cloth_crc = cloth_crc_attr.Get()
        self.assertTrue(initial_cloth_crc == new_cloth_crc)

    @staticmethod
    def _scale_mesh_points(mesh, scale_vec):
        points = mesh.GetPointsAttr().Get()
        for i, p in enumerate(points):
            points[i] = Gf.CompMult(p, Gf.Vec3f(scale_vec))
        mesh.GetPointsAttr().Set(points)

    def _create_weight_measure(self, cloth_mesh):
        #create rigid body sphere and attachment
        cloth_mesh_transform = cloth_mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        cloth_mesh_translation = cloth_mesh_transform.ExtractTranslation()

        rigid_prim_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/rigidbody", True))
        rigid_prim = physicsUtils.add_rigid_sphere(self._stage, rigid_prim_path, position=cloth_mesh_translation, radius=5.0, density=0.001)
        attachment_path = cloth_mesh.GetPath().AppendElementString("attachment")
        attachment_actor0_path = cloth_mesh.GetPath()
        attachment_actor1_path = rigid_prim.GetPath()
        omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=attachment_path, actor0_path=attachment_actor0_path, actor1_path=attachment_actor1_path)
        auto_attachment_api = PhysxSchema.PhysxAutoAttachmentAPI.Get(self._stage, attachment_path)
        auto_attachment_api.GetCollisionFilteringOffsetAttr().Set(3.0)

        #create prismatic joint and attach rigid body
        joint_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/joint", True))
        joint = UsdPhysics.PrismaticJoint.Define(self._stage, joint_path)
        joint.CreateAxisAttr("Y")
        joint.CreateLowerLimitAttr(0)
        joint.CreateUpperLimitAttr(0)
        joint.CreateBody0Rel().SetTargets(["/World"])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(cloth_mesh_translation))
        joint.CreateBody1Rel().SetTargets([rigid_prim.GetPath()])
        physx_limit_api = PhysxSchema.PhysxLimitAPI.Apply(joint.GetPrim(), "linear");
        physx_limit_api.GetStiffnessAttr().Set(1000.0)
        physx_limit_api.GetDampingAttr().Set(100.0)

    #test scaling to equal geometry results in equal particle masses
    #testing with attachent/prismatic joint since we don't have direct access to particle masses right now
    async def _run_particle_cloth_scale_masses(self, mass_type, scale):
        await self.base_setup()

        meters_per_unit = UsdGeom.GetStageMetersPerUnit(self._stage)
        kilograms_per_unit = UsdPhysics.GetStageKilogramsPerUnit(self._stage)

        #parameterization
        u_res = 20
        v_res = 20
        width = 100.0
        ps_rest_offset = 0.02/meters_per_unit

        default_density = 1000.0 #kg/m3
        default_density = default_density*(meters_per_unit*meters_per_unit*meters_per_unit)/kilograms_per_unit
        cloth_thickness = 2.0 * ps_rest_offset;
        cloth_area = width*width
        cloth_num_particles = (u_res+1)*(v_res+1)

        translation_ref = Gf.Vec3d(-100.0, 0.0, 0.0)
        translation_cand = Gf.Vec3d(100.0, 0.0, 0.0)

        #create reference mesh
        mesh_ref = self._create_tesselated_plane_mesh(u_resolution=u_res, v_resolution=v_res, scale=width)
        self.assertTrue(len(mesh_ref.GetPointsAttr().Get()) == cloth_num_particles)
        physicsUtils.set_or_add_translate_op(mesh_ref.GetPrim(), translation_ref)

        #assume default density is used for the ref particle_cloth
        particle_cloth_api_ref = self.add_particle_cloth_to_mesh_command(mesh_ref)

        #query particle system and set rest offset
        ps_path = PhysxSchema.PhysxParticleAPI(particle_cloth_api_ref).GetParticleSystemRel().GetTargets()[0]
        ps = PhysxSchema.PhysxParticleSystem.Get(self._stage, ps_path)
        self.assertTrue(ps)
        ps.GetRestOffsetAttr().Set(ps_rest_offset)

        #create contraption to compare masses medival style using a spring (masses not directly accessible right now)
        self._create_weight_measure(mesh_ref)

        #create candidate Mesh
        mesh_cand = self._create_tesselated_plane_mesh(u_resolution=u_res, v_resolution=v_res, scale=width)

        #scale points, and compensate via transform
        inv_scale = Gf.Vec3d(1.0/scale[0], 1.0/scale[1], 1.0/scale[2])
        self._scale_mesh_points(mesh_cand, scale)
        physicsUtils.set_or_add_scale_op(mesh_cand.GetPrim(), inv_scale)
        physicsUtils.set_or_add_translate_op(mesh_cand.GetPrim(), translation_cand)

        #create particle cloth with auto API
        particle_cloth_api_cand = self.add_particle_cloth_to_mesh_command(mesh_cand)
        if mass_type == 'default':
            pass
        elif mass_type == 'material_density':
            #setting up a material density, will be used by both ref & cand because of shared particle system
            pbd_material_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/pbdmaterial", True))
            particleUtils.add_pbd_particle_material(self._stage, pbd_material_path)
            pbd_material_api = PhysxSchema.PhysxPBDMaterialAPI.Get(self._stage, pbd_material_path)
            pbd_material_api.GetDensityAttr().Set(default_density*10)
            physicsUtils.add_physics_material_to_prim(self._stage, ps.GetPrim(), pbd_material_path)
        elif mass_type == 'massapi_mass':
            #setting up mass of cand to match default density of ref
            mass_api = UsdPhysics.MassAPI.Apply(mesh_cand.GetPrim())
            mass_api.GetMassAttr().Set(cloth_area*cloth_thickness*default_density)
        elif mass_type == 'massapi_density':
            #setting up density of cand to match default density of ref
            mass_api = UsdPhysics.MassAPI.Apply(mesh_cand.GetPrim())
            mass_api.GetDensityAttr().Set(default_density)

        #create contraption to compare masses (not directly accessible right now)
        self._create_weight_measure(mesh_cand)

        self._start()
        self._step(100)

        #make sure candidate cloth dropped the same height within a margin of 5%
        transform_ref = mesh_ref.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        height_ref = transform_ref.ExtractTranslation()[1]
        transform_cand = mesh_cand.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        height_cand = transform_cand.ExtractTranslation()[1]

        self.assertTrue(height_cand > height_ref*1.05 and height_cand < height_ref*0.95)

    async def test_particle_cloth_scale_masses(self):
        for mass_type in ['default', 'material_density', 'massapi_mass', 'massapi_density']:
            for scale in [Gf.Vec3d(1), Gf.Vec3d(0.01), Gf.Vec3d(0.01, 1, 1)]:
                await self._run_particle_cloth_scale_masses(mass_type, scale)



    # Multi scenes
    async def test_particle_cloth_multi_scenes(self):
        await self.base_setup()
        particle_system = self._create_particle_system()

        #scene 1 has 1/4 the default gravity so objects fall much slower
        scene1Path = Sdf.Path('/World/PhysicsScene1')
        metersPerUnit = UsdGeom.GetStageMetersPerUnit(self._stage)
        scene = UsdPhysics.Scene.Define(self._stage, scene1Path)
        scene.CreateGravityMagnitudeAttr().Set(9.8 / metersPerUnit / 4)

        particle_system1 = self._create_particle_system("particleSystem1")
        particle_system1.CreateSimulationOwnerRel().SetTargets([scene1Path])

        prim_type_list = ['Plane', 'Plane']
        mesh = self._create_mesh_primitives(prim_type_list)

        defaultParticleSystemPath = Sdf.Path('/World/particleSystem')
        psPaths = [defaultParticleSystemPath, particle_system1.GetPath()]

        for idx, m in enumerate(mesh):
            cloth = self.add_particle_cloth_to_mesh_command(m)
            particle_cloth_api = PhysxSchema.PhysxParticleClothAPI(cloth)
            PhysxSchema.PhysxParticleAPI(particle_cloth_api).CreateParticleSystemRel().SetTargets([psPaths[idx]])

        self.assertTrue(await self._stepper.play_and_step_and_pause(10, 10))

        height0 = mesh[0].ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[1]
        height1 = mesh[1].ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[1]
        self.assertLess(height0, height1)

    # Reset positions and velocities at runtime
    async def _run_particle_cloth_reset_at_runtime(self, mesh: UsdGeom.Mesh):
        particle_system = self._create_particle_system()

        particle_cloth_api = self.add_particle_cloth_to_mesh_command(mesh)
        xformable = UsdGeom.Xformable(mesh)

        initial_scale = Gf.Vec3f(1.0)
        initial_orient = Gf.Quatf(0.965925826, Gf.Vec3f(0.0, 0.0, 0.2588190451))
        initial_translate = Gf.Vec3f(60.0, 100.0, 50.0)
        physicsUtils.set_or_add_scale_orient_translate(xformable, initial_scale, initial_orient, initial_translate)

        initial_transform = xformable.ComputeLocalToWorldTransform(0)
        initial_points = mesh.GetPointsAttr().Get()
        initial_velocities = [Gf.Vec3f(0.0, 200.0, 0.0)]*len(initial_points)
        mesh.GetVelocitiesAttr().Set(initial_velocities)

        # create material and assign it to the system:
        particle_material_path = self._default_prim_path.AppendChild("particleMaterial")
        particleUtils.add_pbd_particle_material(self._stage, particle_material_path)
        # add some drag and lift to get aerodynamic effects
        particleUtils.add_pbd_particle_material(self._stage, particle_material_path, drag=0.5, lift=0.8, friction=0.6)
        physicsUtils.add_physics_material_to_prim(
            self._stage, particle_system.GetPrim(), particle_material_path
        )

        #start, step and check that cloth is falling
        self._start()
        self._step(1)

        first_step_points = mesh.GetPointsAttr().Get()
        first_step_velocities = mesh.GetVelocitiesAttr().Get()
        first_step_transform = xformable.ComputeLocalToWorldTransform(0)

        #check that cloth moved
        for i in range(len(initial_points)):
            initial_point_world = initial_transform.Transform(initial_points[i])
            first_step_point_world = first_step_transform.Transform(first_step_points[i])
            self.assertTrue(first_step_point_world[1] != initial_point_world[1])

        #step some more
        self._step(100)

        #reset transform, positions and velocities
        physicsUtils.set_or_add_scale_orient_translate(xformable, initial_scale, initial_orient, initial_translate)
        mesh.GetPointsAttr().Set(initial_points)
        mesh.GetVelocitiesAttr().Set(initial_velocities)

        #step and check all was reset, comparing to first_step_transform, first_step_points
        self._step(1)

        reset_step_points = mesh.GetPointsAttr().Get()
        reset_step_velocities = mesh.GetVelocitiesAttr().Get()
        reset_step_transform = xformable.ComputeLocalToWorldTransform(0)

        self._assert_transform_close(reset_step_transform, first_step_transform, 5)
        for i in range(len(first_step_points)):
            self._assertAlmostEqualVector3(first_step_points[i], reset_step_points[i], 5)
            #only testing for one decimal place, as there is some non-determinism in the velocities
            #doesn't seem to happen in 104
            self._assertAlmostEqualVector3(first_step_velocities[i], reset_step_velocities[i], 1)

    async def test_particle_cloth_reset_at_runtime_noweld(self):
        await self.base_setup()
        u_res = 20
        v_res = 20
        mesh = self._create_tesselated_plane_mesh(u_resolution=u_res, v_resolution=v_res, scale=100.0)
        await self._run_particle_cloth_reset_at_runtime(mesh)

    async def test_particle_cloth_reset_at_runtime_weld(self):
        await self.base_setup(usd_file="generatedLegacyMeshes")
        mesh = UsdGeom.Mesh.Get(self._stage, '/World/' + 'Disk')
        await self._run_particle_cloth_reset_at_runtime(mesh)

    async def test_particle_cloth_reset_at_runtime_extra_vertices_weld(self):
        await self.base_setup(usd_file="weldedMeshWithExtraVertices")
        self.add_groundplane()
        mesh = UsdGeom.Mesh.Get(self._stage, '/World/' + 'Disk')     
        await self._run_particle_cloth_reset_at_runtime(mesh)

    # OM-90086 - crash because of double free of particle cloth when deleting in paused state
    async def test_particle_cloth_delete_cloth_and_system(self):
        await self.base_setup()

        #create a mesh
        mesh = self._create_tesselated_plane_mesh()

        #add particle cloth component, this will create a ParticleSystem
        self.add_particle_cloth_to_mesh_command(mesh)
        particle_system_path = self._get_particle_system_path(mesh)

        await utils.play_and_step_and_pause(self, 1)

        #remove particle system and particle cloth
        self._stage.RemovePrim(particle_system_path)
        self._stage.RemovePrim(mesh.GetPath())

    # OM-103703 - crash because of particle cloth triangle index out of range 
    async def test_particle_cloth_triangle_index_out_of_range(self):
        await self.base_setup()

        particle_system = self._create_particle_system()

        # Create grid mesh
        cloth_mesh_path = self._default_prim_path.AppendChild("cloth")
        cloth_mesh = UsdGeom.Mesh.Define(self._stage, cloth_mesh_path)
        tri_points, tri_indices = deformableUtils.create_triangle_mesh_square(32, 32, 50)
        cloth_mesh.GetPointsAttr().Set(tri_points)
        tri_indices[0] = len(tri_points)
        cloth_mesh.GetFaceVertexIndicesAttr().Set(tri_indices)
        cloth_mesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))

        particleUtils.add_physx_particle_cloth(
            stage=self._stage,
            path=cloth_mesh_path,
            dynamic_mesh_path=None,
            particle_system_path=particle_system.GetPath()
        )

        particleMass = 0.02
        massApi = UsdPhysics.MassAPI.Apply(cloth_mesh.GetPrim())
        massApi.GetMassAttr().Set(particleMass * len(tri_points))

        # setup a transform:
        init_translation = Gf.Vec3f(1.5, 100.0, 10.5)
        init_rotx = 5.0
        cloth_mesh.SetResetXformStack(True)
        cloth_mesh.AddTranslateOp().Set(init_translation)
        cloth_mesh.AddRotateXOp().Set(init_rotx)

        self._start()
        self._reset()
        
        attribute_name = "physxParticle:weldedTriangleIndices"
        if cloth_mesh.GetPrim().HasAttribute(attribute_name):
            welded_tri_indices = list(cloth_mesh.GetPrim().GetAttribute(attribute_name).Get())
            welded_tri_indices[0] = len(tri_points)

            error_message = 'PhysxSchemaPhysxParticleClothAPI:weldedTriangleIndices has invalid size or contains out of range indices.'
            # expect message when restart the simulation
            with utils.ExpectMessage(self, error_message, expected_result=True):
                cloth_mesh.GetPrim().GetAttribute(attribute_name).Set(welded_tri_indices)
                self._step(num_steps=5)
                pos = cloth_mesh.GetPrim().GetAttribute("xformOp:translate").Get()
                self.assertEqual(pos[1], init_translation[1])

    # OM-103960 - crash because previously we didn't early out when non watertight mesh is simulated as inflatable
    async def test_particle_cloth_invalid_mesh(self):
        data_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, "../../../../data/")))
        data_folder = data_folder.replace("\\", "/") + "/"

        filename = "NonWatertightMesh.usd"
        filepath = os.path.normpath(data_folder + filename)

        # load particle cloth assets that already have non-manifold springs
        await omni.usd.get_context().open_stage_async(filepath)
        usd_context = omni.usd.get_context()
        self.assertIn(filename, usd_context.get_stage_url())

        # simulation shouldn't crash, but only give errors
        error_message = 'Non watertight mesh on inflatable is not supported.'
        with utils.ExpectMessage(self, error_message, expected_result=True):
            await omni.kit.app.get_app().next_update_async()
            self._start()
            self._step(1)
            self._reset()

        await omni.usd.get_context().close_stage_async()

        filename = "InvalidParticleCloth.usd"
        filepath = os.path.normpath(data_folder + filename)

        mesh_path0 = "/World/RootNode/Katherine_bag/Katherine_bag"
        mesh_path1 = "/World/RootNode/Katherine_bag_01/Katherine_bag"
        mesh_path2 = "/World/RootNode/Katherine_bag_05/Katherine_bag"

        # load particle cloth assets that have duplicate indices in one spring
        await omni.usd.get_context().open_stage_async(filepath)
        usd_context = omni.usd.get_context()
        self.assertIn(filename, usd_context.get_stage_url())

        # simulation shouldn't crash, but only give errors
        error_msg_a0 = f"Cooking mesh for particle cloth simulation failed: {mesh_path0}. The input mesh becomes non-manifold after welding. Consider disabling welding."
        error_msg_a1 = f"Cooking mesh for particle cloth simulation failed: {mesh_path1}. The input mesh becomes non-manifold after welding. Consider disabling welding."
        error_msg_a2 = f"Cooking mesh for particle cloth simulation failed: {mesh_path2}. The input mesh becomes non-manifold after welding. Consider disabling welding."
        error_msg_b = f"PhysxSchemaPhysxParticleClothAPI:springIndices has no elements."
        error_msg_c0 = f"PhysxSchemaPhysxParticleClothAPI parsing failed: {mesh_path0}"
        error_msg_c1 = f"PhysxSchemaPhysxParticleClothAPI parsing failed: {mesh_path1}"
        error_msg_c2 = f"PhysxSchemaPhysxParticleClothAPI parsing failed: {mesh_path2}"

        with utils.ExpectMessage(self, [error_msg_a0, error_msg_a1, error_msg_a2, error_msg_b, error_msg_c0, error_msg_c1, error_msg_c2], expected_result=True):
            await omni.kit.app.get_app().next_update_async()
            self._start()
            self._step(1)
            self._reset()

        # reload stage
        await omni.usd.get_context().open_stage_async(filepath)
        usd_context = omni.usd.get_context()
        self.assertIn(filename, usd_context.get_stage_url())

        # simulation shouldn't crash in case of pressure 0
        cloth_prim_1 = usd_context.get_stage().GetPrimAtPath(Sdf.Path(mesh_path0))
        particle_cloth_api_1 = PhysxSchema.PhysxParticleClothAPI(cloth_prim_1)
        particle_cloth_api_1.GetPressureAttr().Set(0.0)

        cloth_prim_2 = usd_context.get_stage().GetPrimAtPath(Sdf.Path(mesh_path1))
        particle_cloth_api_2 = PhysxSchema.PhysxParticleClothAPI(cloth_prim_2)
        particle_cloth_api_2.GetPressureAttr().Set(0.0)

        with utils.ExpectMessage(self, [error_msg_a0, error_msg_a1, error_msg_a2, error_msg_b, error_msg_c0, error_msg_c1, error_msg_c2], expected_result=True):
            await omni.kit.app.get_app().next_update_async()
            self._start()
            self._step(1)


    #regression test for catching more invalid conditions during parsing for OM-110476
    async def helper_particle_cloth_inconsistent_setup(self):
        await self.base_setup()
        particle_system = self._create_particle_system()
        # Create grid mesh
        cloth_mesh_path = self._default_prim_path.AppendChild("cloth")
        cloth_mesh = UsdGeom.Mesh.Define(self._stage, cloth_mesh_path)
        tri_points, tri_indices = deformableUtils.create_triangle_mesh_square(5, 5, 50)
        #add a copy of the last triangle to the end to be welded
        tri_indices = list(tri_indices)
        tri_indices = tri_indices + [len(tri_points), len(tri_points)+1, len(tri_points)+2]
        tri_points = list(tri_points)
        tri_points = tri_points + tri_points[-3:]
        cloth_mesh.GetPointsAttr().Set(tri_points)
        cloth_mesh.GetFaceVertexIndicesAttr().Set(tri_indices)
        cloth_mesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))
        particleUtils.add_physx_particle_cloth(
            stage=self._stage,
            path=cloth_mesh_path,
            dynamic_mesh_path=None,
            particle_system_path=particle_system.GetPath()
        )
        cloth_auto_api = PhysxSchema.PhysxAutoParticleClothAPI(cloth_mesh)
        cloth_auto_api.CreateDisableMeshWeldingAttr().Set(False)
        await self._wait_cooking_finished()

        return cloth_mesh, PhysxSchema.PhysxParticleClothAPI(cloth_mesh)

    async def test_particle_cloth_inconsistent_setup(self):

        message = f"UsdGeomMesh:velocities and points have inconsistent sizes."
        mesh, particle_cloth = await self.helper_particle_cloth_inconsistent_setup()
        with utils.ExpectMessage(self, message, expected_result=True):
            mesh.GetVelocitiesAttr().Set([Gf.Vec3f(0,0,0)])
            self._start()
            self._reset()

        message1 = f"PhysxSchemaPhysxParticleClothAPI:restPoints and UsdGeomMesh:points have inconsistent sizes."
        message2 = f"Particle cloth mesh, failed to setup cooking params, prim: /World/cloth"
        mesh, particle_cloth = await self.helper_particle_cloth_inconsistent_setup()
        with utils.ExpectMessage(self, [message1, message2], expected_result=True):
            particle_cloth.GetRestPointsAttr().Set([Gf.Vec3f(0,0,0)])
            self._start()
            self._reset()

        message = f"PhysxSchemaPhysxParticleClothAPI:weldedTriangleIndices has invalid size or contains out of range indices."
        mesh, particle_cloth = await self.helper_particle_cloth_inconsistent_setup()
        with utils.ExpectMessage(self, message, expected_result=True):
            attr = mesh.GetPrim().GetAttribute("physxParticle:weldedTriangleIndices")
            indices_list = list(attr.Get())
            #remove last element
            attr.Set(indices_list[:-1])
            self._start()
            self._reset()

        message = f"PhysxSchemaPhysxParticleClothAPI:weldedTriangleIndices has invalid size or contains out of range indices."
        mesh, particle_cloth = await self.helper_particle_cloth_inconsistent_setup()
        with utils.ExpectMessage(self, message, expected_result=True):
            attr = mesh.GetPrim().GetAttribute("physxParticle:weldedTriangleIndices")
            indices_list = list(attr.Get())
            #make index 10 invalid
            attr.Set(indices_list[:10] + [9999999] + indices_list[11:]) 
            self._start()
            self._reset()

        message = f"PhysxSchemaPhysxParticleClothAPI:weldedVerticesRemapToWeld has invalid size or contains out of range indices."
        mesh, particle_cloth = await self.helper_particle_cloth_inconsistent_setup()
        with utils.ExpectMessage(self, message, expected_result=True):
            attr = mesh.GetPrim().GetAttribute("physxParticle:weldedVerticesRemapToWeld")
            indices_list = list(attr.Get())
            #remove last element
            attr.Set(indices_list[:-1])
            self._start()
            self._reset()

        message = f"PhysxSchemaPhysxParticleClothAPI:weldedVerticesRemapToWeld has invalid size or contains out of range indices."
        mesh, particle_cloth = await self.helper_particle_cloth_inconsistent_setup()
        with utils.ExpectMessage(self, message, expected_result=True):
            attr = mesh.GetPrim().GetAttribute("physxParticle:weldedVerticesRemapToWeld")
            indices_list = list(attr.Get())
            #make index 10 invalid
            attr.Set(indices_list[:10] + [9999999] + indices_list[11:]) 
            self._start()
            self._reset()

        message = f"PhysxSchemaPhysxParticleClothAPI:weldedVerticesRemapToOrig has invalid size or contains out of range indices."
        mesh, particle_cloth = await self.helper_particle_cloth_inconsistent_setup()
        with utils.ExpectMessage(self, message, expected_result=True):
            attr = mesh.GetPrim().GetAttribute("physxParticle:weldedVerticesRemapToOrig")
            indices_list = list(attr.Get())
            #make index 10 invalid
            attr.Set(indices_list[:10] + [9999999] + indices_list[11:]) 
            self._start()
            self._reset()

        message = f"PhysxSchemaPhysxParticleClothAPI:springStiffnesses and springIndices have inconsistent sizes."
        mesh, particle_cloth = await self.helper_particle_cloth_inconsistent_setup()
        with utils.ExpectMessage(self, message, expected_result=True):
            particle_cloth.GetSpringStiffnessesAttr().Set([0.5])
            self._start()
            self._reset()

        message = f"PhysxSchemaPhysxParticleClothAPI:springDampings and springIndices have inconsistent sizes."
        mesh, particle_cloth = await self.helper_particle_cloth_inconsistent_setup()
        with utils.ExpectMessage(self, message, expected_result=True):
            particle_cloth.GetSpringDampingsAttr().Set([0.5])
            self._start()
            self._reset()

        message = f"PhysxSchemaPhysxParticleClothAPI:springRestLengths and springIndices have inconsistent sizes."
        mesh, particle_cloth = await self.helper_particle_cloth_inconsistent_setup()
        with utils.ExpectMessage(self, message, expected_result=True):
            particle_cloth.GetSpringRestLengthsAttr().Set([0.5])
            self._start()
            self._reset()

        message = f"PhysxSchemaPhysxParticleClothAPI:springIndices has invalid size or contains out of range indices."
        mesh, particle_cloth = await self.helper_particle_cloth_inconsistent_setup()
        with utils.ExpectMessage(self, message, expected_result=True):
            indices_list = list(particle_cloth.GetSpringIndicesAttr().Get())
            #make index 10 invalid
            particle_cloth.GetSpringIndicesAttr().Set(indices_list[:10] + [Gf.Vec2i(9999999,9999999)] + indices_list[11:]) 
            self._start()
            self._reset()
