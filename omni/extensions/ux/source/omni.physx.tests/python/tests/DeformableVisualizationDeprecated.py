# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.app
from pxr import Gf, UsdGeom, PhysxSchema, Sdf, UsdLux, Usd, UsdPhysics, Vt
from omni.physxtests import utils
from omni.physxcommands import AddGroundPlaneCommand, SetRigidBodyCommand
from omni.physx.scripts import physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
from omni.physx import get_physx_interface, get_physx_cooking_interface
import carb
import omni.physx.bindings._physx as pb
import math
from enum import Enum

COLOR_COLLISION = Gf.Vec3f(0.0, 0.3, 0.05)  # green
COLOR_FILTER = Gf.Vec3f(0.0, 0.7, 0.5)   # cyan


class DeformableVisualizationTestKitStageDeprecated(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Kit
    @classmethod
    def setUpClass(self):
        # init for attributes that are not stage-dependent:
        # carb settings and bloky dev mode:
        self._settings = carb.settings.get_settings()

    # runs before each test case
    async def setUp(self):
        await super().setUp()

    # runs after each test case and runs base_terminate if setup was called:
    async def tearDown(self):
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

    @staticmethod
    def get_time_step():
        return 1.0 / 60.0

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

    def add_groundplane(self):
        AddGroundPlaneCommand.execute(self._stage, '/CollisionPlane',
                                      self._upAxis, 1000.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    @staticmethod
    def set_prim_translation(prim: Usd.Prim, translateVec: Gf.Vec3d):
        translate_mtx = Gf.Matrix4d().SetTranslate(translateVec)
        omni.kit.commands.execute("TransformPrim", path=prim.GetPath(), new_transform_matrix=translate_mtx)

    @staticmethod
    def get_world_bounds(imageable: UsdGeom.Imageable) -> Gf.Range3d:
        obb = imageable.ComputeWorldBound(Usd.TimeCode.Default(), purpose1=imageable.GetPurposeAttr().Get())
        return obb.ComputeAlignedBox()

    async def run_add_deformable_body_component_command(self, skin_mesh_path: Sdf.Path=Sdf.Path(), collision_mesh_path: Sdf.Path=Sdf.Path(), simulation_mesh_path: Sdf.Path=Sdf.Path()) -> bool:
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

    def get_session_layer_xform_path(self, path: Sdf.Path) -> Sdf.Path:
        session_base_path = Sdf.Path("/PhysxDeformablesVisualizationMeshes")
        path_mangled = path.pathString.replace("/", "_")
        return session_base_path.AppendChild(path_mangled)

    def get_collision_mesh_path(self, path: Sdf.Path) -> Sdf.Path:
        path_mangled = self.get_session_layer_xform_path(path)   
        return Sdf.Path(path_mangled.pathString + "_collision")

    def get_simulation_mesh_path(self, path: Sdf.Path) -> Sdf.Path:
        path_mangled = self.get_session_layer_xform_path(path)
        return Sdf.Path(path_mangled.pathString + "_simulation")

    def is_skin_mesh_shown(self, skin_mesh_path: Sdf.Path):
        imageable = UsdGeom.Imageable.Get(self._stage, skin_mesh_path)
        if imageable:
            purpose = imageable.GetPurposeAttr().Get()
            return purpose == "default"
        else:
            return False

    def is_tet_mesh_shown(self, tet_mesh_path: Sdf.Path):
        imageable = UsdGeom.Imageable.Get(self._stage, tet_mesh_path)
        if imageable:
            visible = imageable.GetVisibilityAttr().Get()
            return visible == "inherited"
        else:
            return False

    def check_deformable_bodies_viz(self, skin_mesh_path, selected_mesh_path, display_deformable_bodies, display_deformable_body_type):
        is_skin_mesh_shown = True
        is_collision_mesh_shown = False
        is_simulation_mesh_shown = False
        if display_deformable_bodies == pb.VisualizerMode.ALL or (display_deformable_bodies == pb.VisualizerMode.SELECTED and str(skin_mesh_path) == selected_mesh_path):
            is_skin_mesh_shown = False
            if display_deformable_body_type == 0:   #Simulation
                is_simulation_mesh_shown = True
            elif display_deformable_body_type == 1: #Collision
                is_collision_mesh_shown = True
        
        self.assertEqual(is_skin_mesh_shown, self.is_skin_mesh_shown(skin_mesh_path))

        if display_deformable_body_type == 0:   #Simulation
            simulation_mesh_path = self.get_simulation_mesh_path(skin_mesh_path)
            self.assertEqual(is_simulation_mesh_shown, self.is_tet_mesh_shown(simulation_mesh_path))      
        elif display_deformable_body_type == 1: #Collision
            collision_mesh_path = self.get_collision_mesh_path(skin_mesh_path)
            self.assertEqual(is_collision_mesh_shown, self.is_tet_mesh_shown(collision_mesh_path))         

    def check_filtered_colors(self, collision_mesh_path):
        # filtered tets have different color
        collision_mesh = UsdGeom.Mesh.Get(self._stage, collision_mesh_path)
        collision_mesh_colors = collision_mesh.GetDisplayColorPrimvar().Get()
        self.assertIsNotNone(collision_mesh_colors)
        self.assertTrue(len(collision_mesh_colors) > 0)

        result = False
        for color in collision_mesh_colors:
            if color == COLOR_COLLISION:
                continue
            elif color == COLOR_FILTER:
                result = True
            else:
                return False

        return result

    def check_attachment_one_deformable_body_viz(self, attachment_path, skin_mesh_path, selected_mesh_path, display_attachments, hide_deformable_body, hide_the_other_prim):
        is_collision_mesh_shown = False
        is_simulation_mesh_shown = False
        if (display_attachments == pb.VisualizerMode.ALL or (display_attachments == pb.VisualizerMode.SELECTED and str(skin_mesh_path) == selected_mesh_path)) and hide_deformable_body is False:
            is_collision_mesh_shown = True
        
        simulation_mesh_path = self.get_simulation_mesh_path(skin_mesh_path)
        self.assertEqual(is_simulation_mesh_shown, self.is_tet_mesh_shown(simulation_mesh_path))
        collision_mesh_path = self.get_collision_mesh_path(skin_mesh_path)
        self.assertEqual(is_collision_mesh_shown, self.is_tet_mesh_shown(collision_mesh_path))

        if is_collision_mesh_shown:
            self.assertTrue(self.check_filtered_colors(collision_mesh_path))

    def check_attachment_two_deformable_body_viz(self, attachment_path, skin_mesh_path_actor0, skin_mesh_path_actor1, selected_mesh_path, display_attachments, hide_actor0, hide_actor1):
        is_actor0_collision_mesh_shown = False
        is_actor1_collision_mesh_shown = False
        is_actor0_simulation_mesh_shown = False
        is_actor1_simulation_mesh_shown = False
        if display_attachments == pb.VisualizerMode.ALL or (display_attachments == pb.VisualizerMode.SELECTED and str(skin_mesh_path_actor0) == selected_mesh_path):
            if hide_actor0 is False:
                is_actor0_collision_mesh_shown = True
            if hide_actor1 is False:
                is_actor1_collision_mesh_shown = True                
        
        actor0_simulation_mesh_path = self.get_simulation_mesh_path(skin_mesh_path_actor0)
        self.assertEqual(is_actor0_simulation_mesh_shown, self.is_tet_mesh_shown(actor0_simulation_mesh_path))
        actor1_simulation_mesh_path = self.get_simulation_mesh_path(skin_mesh_path_actor1)
        self.assertEqual(is_actor1_simulation_mesh_shown, self.is_tet_mesh_shown(actor1_simulation_mesh_path))

        actor0_collision_mesh_path = self.get_collision_mesh_path(skin_mesh_path_actor0)
        self.assertEqual(is_actor0_collision_mesh_shown, self.is_tet_mesh_shown(actor0_collision_mesh_path))
        if is_actor0_collision_mesh_shown:
            self.assertTrue(self.check_filtered_colors(actor0_collision_mesh_path))


        actor1_collision_mesh_path = self.get_collision_mesh_path(skin_mesh_path_actor1)
        self.assertEqual(is_actor1_collision_mesh_shown, self.is_tet_mesh_shown(actor1_collision_mesh_path))
        if is_actor1_collision_mesh_shown:
            self.assertTrue(self.check_filtered_colors(actor1_collision_mesh_path))

    def create_mesh_prims(self, prim_type_list: list) -> list:
        mesh_list = []
        for prim_type in prim_type_list:
            path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/" + prim_type, True))
            omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type=prim_type)
            mesh = UsdGeom.Mesh.Get(self._stage, path)
            self.assertTrue(mesh)
            mesh_list.append(mesh)
        return mesh_list

    async def create_mesh_primitives(self, prim_type_list, starting_height=60.0):
        mesh_list = self.create_mesh_prims(prim_type_list)

        height = starting_height
        offset = 150
        origin = Gf.Vec3d(-offset * 3 / 2, height, -offset * 3 / 2)
        for i in range(3):
            for j in range(3):
                index = i * 3 + j
                if index < len(mesh_list):
                    self.set_prim_translation(mesh_list[index].GetPrim(), origin + Gf.Vec3d(i * offset, 0, j * offset))

        return mesh_list

    async def test_deformable_body_viz(self):
        await self.base_setup()
        self.add_groundplane()
        starting_height = 0.0

        omni.kit.commands.execute("AddDeformableBodyMaterial",
                                  stage=self._stage, path=self._deformable_body_material_path,
                                  youngsModulus=800.0)

        prim_list = ['Cone', 'Cube', 'Cylinder', 'Sphere', 'Torus']
        mesh_list = await self.create_mesh_primitives(prim_list, starting_height=starting_height)

        for mesh in mesh_list:
            initial_bounds = self.get_world_bounds(mesh)
            localTrafo = mesh.GetLocalTransformation()
            self.set_prim_translation(mesh.GetPrim(), localTrafo.ExtractTranslation() + Gf.Vec3d(0.0, initial_bounds.GetSize()[1] * 0.5, 0.0))

            skin_mesh_path = mesh.GetPath()
            for m in mesh_list:
                await self.run_add_deformable_body_component_command(skin_mesh_path)

        self.start()
        self.step(50)

        selected_mesh_path = '/World/' + prim_list[-1]
        for display_deformable_bodies in [pb.VisualizerMode.NONE, pb.VisualizerMode.SELECTED, pb.VisualizerMode.ALL]:
            for display_deformable_body_type in [0, 1]:     # Simulation: 0  Collision : 1
                self._settings.set(pb.SETTING_DISPLAY_DEFORMABLE_BODIES, display_deformable_bodies)
                self._settings.set(pb.SETTING_DISPLAY_DEFORMABLE_BODY_TYPE, display_deformable_body_type)
                for mesh in mesh_list:
                    skin_mesh_path = mesh.GetPath()
                    with self.subTest(display_deformable_bodies = display_deformable_bodies):
                        await self.wait(2)
                        self.check_deformable_bodies_viz(skin_mesh_path, selected_mesh_path, display_deformable_bodies=display_deformable_bodies, display_deformable_body_type=display_deformable_body_type)

    async def generate_procedural_torus_mesh(self, r, R, nb_sides, nb_rings):
        points = []
        face_vertex_indices = []
        # Generate points
        for i in range(nb_rings + 1):
            theta = 2.0 * math.pi * i / nb_rings
            cos_theta, sin_theta = math.cos(theta), math.sin(theta)
            for j in range(nb_sides + 1):
                phi = 2.0 * math.pi * j / nb_sides
                cos_phi, sin_phi = math.cos(phi), math.sin(phi)
                x = (R + r * cos_phi) * cos_theta
                y = (R + r * cos_phi) * sin_theta
                z = r * sin_phi
                points.append((x, y, z))
        # Generate face vertex indices for quad faces
        for i in range(nb_rings):
            for j in range(nb_sides):
                i0 = i * (nb_sides + 1) + j
                i1 = i0 + nb_sides + 1
                i2 = i1 + 1
                i3 = i0 + 1
                # Faces are defined as quads (four vertices per face)
                face_vertex_indices.extend([i0, i1, i2, i3])
        return points, face_vertex_indices
    
    async def create_torus_mesh_prim(self, minor_rdius, major_radius, nb_cross_section_sides, nb_radial_divisions):
        path = omni.usd.get_stage_next_free_path(self._stage, "/Softbody", True)
        mesh = UsdGeom.Mesh.Define(self._stage, path)
        # Set points and vertex indices
        # We create procedural mesh here since create_mesh_primitives() can NOT repro the bug
        points, faceVertexIndices = await self.generate_procedural_torus_mesh(minor_rdius, major_radius, nb_cross_section_sides, nb_radial_divisions)
        mesh.CreatePointsAttr(Vt.Vec3fArray(points))
        # Each face is a quad, hence the vertex count per face is 4
        faceVertexCounts = Vt.IntArray(len(faceVertexIndices) // 4 * [4])
        mesh.CreateFaceVertexCountsAttr(faceVertexCounts)
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(faceVertexIndices))
        return mesh

    async def check_tet_mesh_viz(self, tet_path, num_iterations):
        usd_context = omni.usd.get_context()
        tet_mesh_prim = usd_context.get_stage().GetPrimAtPath(tet_path)
        self.assertTrue(tet_mesh_prim)
        xformable = UsdGeom.Xformable(tet_mesh_prim)
        transform_original = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

        self.start()
        self.step(num_iterations)
        await self.wait(2)
        self.reset()

        await self.wait(2)
        transform_on_stop = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(transform_original == transform_on_stop)   

    # OM-110826 - deformable Body visualization doesn't reset properly on Stop
    async def test_deformable_body_viz_reset_on_stop(self):
        await self.base_setup()

        # Create the Torus mesh prim
        render_mesh = await self.create_torus_mesh_prim(1.0, 2.0, 20, 20)
        render_mesh_path = render_mesh.GetPath()
        await self.run_add_deformable_body_component_command(render_mesh_path)

        self._settings.set(pb.SETTING_DISPLAY_DEFORMABLE_BODIES, pb.VisualizerMode.ALL)

        # Test collision mesh visualization   
        self._settings.set(pb.SETTING_DISPLAY_DEFORMABLE_BODY_TYPE, 1)  # Simulation: 0  Collision : 1
        await self.wait(2)

        collision_mesh_path = self.get_collision_mesh_path(render_mesh_path)
        await self.check_tet_mesh_viz(collision_mesh_path, 5)

        # Test simulation mesh visualization
        self._settings.set(pb.SETTING_DISPLAY_DEFORMABLE_BODY_TYPE, 0)  # Simulation: 0  Collision : 1
        await self.wait(2)

        simulation_mesh_path = self.get_simulation_mesh_path(render_mesh_path)
        await self.check_tet_mesh_viz(simulation_mesh_path, 10)

    async def test_attachment_viz_deformableBody_rigidBody(self):
        await self.base_setup()

        starting_height = 170.0
        prim_list = ['Cone', 'Cube', 'Cylinder', 'Sphere', 'Torus']    
        mesh_list = await self.create_mesh_primitives(prim_list, starting_height=starting_height) 

        z = 150
        for deformable_mesh in mesh_list:
            translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, starting_height, z))
            omni.kit.commands.execute("TransformPrim", path=deformable_mesh.GetPath(), new_transform_matrix=translate_mtx)
            await self.run_add_deformable_body_component_command(skin_mesh_path=deformable_mesh.GetPath())

            cubeGeometry = omni.physx.scripts.physicsUtils.add_cube(self._stage, "/cubeShape", 100.0, Gf.Vec3f(0.0, starting_height + 30.0, z))
            SetRigidBodyCommand.execute(cubeGeometry.GetPath(), "", True)

            target_attachment_path = deformable_mesh.GetPath().AppendElementString("rigidAttachment")
            target_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(target_attachment_path), False))
            omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=target_attachment_path, actor0_path=deformable_mesh.GetPath(), actor1_path=cubeGeometry.GetPath())

            self.start()
            self.step(30)

            z += 150

        self._settings.set(pb.SETTING_DISPLAY_DEFORMABLE_BODIES, pb.VisualizerMode.NONE)
        selected_mesh_path = '/World/' + prim_list[-1]
        for display_attachments in [pb.VisualizerMode.NONE, pb.VisualizerMode.SELECTED, pb.VisualizerMode.ALL]:
            for hide_actor0 in [True, False]:   # Hide deformables
                for hide_actor1 in [True, False]:   # Hide rigid bodies
                    self._settings.set(pb.SETTING_DISPLAY_ATTACHMENTS, display_attachments)
                    self._settings.set(pb.SETTING_DISPLAY_ATTACHMENTS_HIDE_ACTOR_0, hide_actor0)
                    for deformable_mesh in mesh_list:
                        deformable_body_path = deformable_mesh.GetPath()
                        target_attachment_path = deformable_body_path.AppendElementString("rigidAttachment")
                        with self.subTest(display_attachments = display_attachments):
                            await self.wait(2)
                            self.check_attachment_one_deformable_body_viz(target_attachment_path, deformable_body_path, selected_mesh_path, display_attachments, hide_actor0, hide_actor1)

    async def test_attachment_viz_deformableBody_deformableBody(self):
        await self.base_setup()
        self.add_groundplane()
        cone_height = 170.0
        cube_height = cone_height + 30.0
        sphere_height = cone_height - 90.0
        prim_type_list = ['Cone', 'Sphere']
        coneMesh, sphereMesh = await self.create_mesh_primitives(prim_type_list, starting_height=cone_height)

        translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, cone_height, 0))
        omni.kit.commands.execute("TransformPrim", path=coneMesh.GetPath(), new_transform_matrix=translate_mtx)

        translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, sphere_height, 0))
        omni.kit.commands.execute("TransformPrim", path=sphereMesh.GetPath(), new_transform_matrix=translate_mtx)

        await self.run_add_deformable_body_component_command(skin_mesh_path=coneMesh.GetPath())
        soft_body_0_translation = coneMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertTrue(soft_body_0_translation[1] == cone_height)

        await self.run_add_deformable_body_component_command(skin_mesh_path=sphereMesh.GetPath())
        soft_body_1_translation = sphereMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertTrue(soft_body_1_translation[1] == sphere_height)

        cubePrim = omni.physx.scripts.physicsUtils.add_cube(self._stage, "/cubeActor", 100.0, Gf.Vec3f(0.0, cube_height, 0.0))
        SetRigidBodyCommand.execute(cubePrim.GetPath(), "", False)

        physicsBodyAPI = UsdPhysics.RigidBodyAPI.Get(self._stage, cubePrim.GetPath())
        physicsBodyAPI.CreateKinematicEnabledAttr(True)

        target_attachment_path = coneMesh.GetPath().AppendElementString("rigidAttachment")
        target_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(target_attachment_path), False))
        omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=target_attachment_path, actor0_path=coneMesh.GetPath(), actor1_path=cubePrim.GetPath())

        target_attachment_path = coneMesh.GetPath().AppendElementString("deformableAttachment")
        target_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(target_attachment_path), False))
        omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=target_attachment_path, actor0_path=coneMesh.GetPath(), actor1_path=sphereMesh.GetPath())

        self.start()
        self.step(30)

        # if the softbody is attached, it will not fall to the ground
        soft_body_1_translation = sphereMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        pause_height = soft_body_1_translation[1]
        self.assertTrue(pause_height > (sphere_height / 2.0))

        self._settings.set(pb.SETTING_DISPLAY_DEFORMABLE_BODIES, pb.VisualizerMode.NONE)
        selected_mesh_path = '/World/' + prim_type_list[-1]
        for display_attachments in [pb.VisualizerMode.NONE, pb.VisualizerMode.SELECTED, pb.VisualizerMode.ALL]:
            for hide_actor0 in [True, False]:
                for hide_actor1 in [True, False]:
                    self._settings.set(pb.SETTING_DISPLAY_ATTACHMENTS, display_attachments)
                    self._settings.set(pb.SETTING_DISPLAY_ATTACHMENTS_HIDE_ACTOR_0, hide_actor0)
                    self._settings.set(pb.SETTING_DISPLAY_ATTACHMENTS_HIDE_ACTOR_1, hide_actor1)
                    target_attachment_path = coneMesh.GetPath().AppendElementString("rigidAttachment")
                    with self.subTest(display_attachments = display_attachments):
                        await self.wait(2)
                        self.check_attachment_two_deformable_body_viz(target_attachment_path, coneMesh.GetPath(), sphereMesh.GetPath(), selected_mesh_path, display_attachments, hide_actor0, hide_actor1)
