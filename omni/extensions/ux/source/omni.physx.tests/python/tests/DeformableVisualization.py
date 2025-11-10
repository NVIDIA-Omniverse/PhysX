# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.app
from pxr import Gf, UsdGeom, PhysxSchema, Sdf, UsdLux, Usd, UsdPhysics, Vt
from omni.physxtests import utils
from omni.physxcommands import AddGroundPlaneCommand, SetRigidBodyCommand
from omni.physx.scripts import deformableMeshUtils, deformableUtils
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
from omni.physx import get_physx_interface, get_physx_cooking_interface, get_physx_cooking_private_interface
import carb
import omni.physx.bindings._physx as pb
import math
import unittest

def create_transform(translate = Gf.Vec3d(0.0),
                        rotate = Gf.Rotation(Gf.Quatd(1.0)),
                        scale = Gf.Vec3d(1.0),
                        pivot_pos = Gf.Vec3d(0.0),
                        pivot_orient = Gf.Rotation(Gf.Quatd(1.0))):
    return Gf.Transform(translate, rotate, scale, pivot_pos, pivot_orient)

def set_tetmesh_data(tetmesh):
    points, indices = deformableMeshUtils.createTetraVoxelBox(3)
    tet_indices = [Gf.Vec4i(*indices[i:i+4]) for i in range(0, len(indices), 4)]
    tetmesh.GetPointsAttr().Set(points)
    tetmesh.GetTetVertexIndicesAttr().Set(tet_indices)

def setup_xform_body(stage, path, transform: Gf.Transform):
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTransformOp().Set(transform.GetMatrix())
    deformable_body = xform.GetPrim().ApplyAPI("OmniPhysicsDeformableBodyAPI")
    return xform

def add_surface_faces(tetmesh):
    surfaceFaceIndices = UsdGeom.TetMesh.ComputeSurfaceFaces(tetmesh, Usd.TimeCode.Default())
    tetmesh.GetSurfaceFaceVertexIndicesAttr().Set(surfaceFaceIndices)

def setup_simmesh(stage, path, transform: Gf.Transform, add_collision: bool = False):
    simmesh = UsdGeom.TetMesh.Define(stage, path)
    simmesh.AddTransformOp().Set(transform.GetMatrix())
    set_tetmesh_data(simmesh)
    # add sim api
    simmesh.GetPrim().ApplyAPI("OmniPhysicsVolumeDeformableSimAPI")
    simmesh.GetPrim().GetAttribute("omniphysics:restShapePoints").Set(simmesh.GetPointsAttr().Get())
    simmesh.GetPrim().GetAttribute("omniphysics:restTetVtxIndices").Set(simmesh.GetTetVertexIndicesAttr().Get())

    # add collision
    if add_collision:
        UsdPhysics.CollisionAPI.Apply(simmesh.GetPrim())
        add_surface_faces(simmesh)

    return simmesh

def setup_collmesh(stage, path, transform: Gf.Transform):
    collmesh = UsdGeom.TetMesh.Define(stage, path)
    collmesh.AddTransformOp().Set(transform.GetMatrix())
    set_tetmesh_data(collmesh)

    # add coll api and surface faces
    UsdPhysics.CollisionAPI.Apply(collmesh.GetPrim())
    add_surface_faces(collmesh)

    return collmesh

def set_trimesh_data_from_tetmesh_surface(trimesh, tetmesh):
    tet_indices_flat = [e for vec in tetmesh.GetTetVertexIndicesAttr().Get() for e in vec]
    tri_points, tri_indices = deformableUtils.extractTriangleSurfaceFromTetra(tetmesh.GetPointsAttr().Get(), tet_indices_flat)
    trimesh.GetPointsAttr().Set(tri_points)
    trimesh.GetFaceVertexCountsAttr().Set([3]*(len(tri_indices)//3))
    trimesh.GetFaceVertexIndicesAttr().Set(tri_indices)

def setup_skinmesh_from_tetmesh(stage, path, tetmesh, scalefactor):
    skinmesh = UsdGeom.Mesh.Define(stage, path)
    tetmesh_transform = Gf.Transform(tetmesh.GetLocalTransformation())
    tetmesh_transform.SetScale(tetmesh_transform.GetScale()*scalefactor)
    skinmesh.AddTransformOp().Set(tetmesh_transform.GetMatrix())
    set_trimesh_data_from_tetmesh_surface(skinmesh, tetmesh)
    return skinmesh

def set_trimesh_data(trimesh):
    tri_points, tri_indices = deformableMeshUtils.createTriangleMeshCube(3)
    trimesh.GetPointsAttr().Set(tri_points)
    trimesh.GetFaceVertexCountsAttr().Set([3]*(len(tri_indices)//3))
    trimesh.GetFaceVertexIndicesAttr().Set(tri_indices)

def setup_xform_auto_body(stage, path, transform: Gf.Transform):
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTransformOp().Set(transform.GetMatrix())
    xform.GetPrim().ApplyAPI("OmniPhysicsDeformableBodyAPI")
    xform.GetPrim().ApplyAPI("PhysxAutoDeformableBodyAPI")
    return xform

def setup_cube_trimesh(stage, path, transform: Gf.Transform, dim: int):
    tri_points, tri_indices = deformableMeshUtils.createTriangleMeshCube(dim)
    skinmesh = UsdGeom.Mesh.Define(stage, path)
    skinmesh.AddTransformOp().Set(transform.GetMatrix())
    skinmesh.GetPointsAttr().Set(tri_points)
    skinmesh.GetFaceVertexCountsAttr().Set([3]*(len(tri_indices)//3))
    skinmesh.GetFaceVertexIndicesAttr().Set(tri_indices)
    return skinmesh

def get_meshes(root_prim):
    meshes = []
    if root_prim.GetPrim().IsA(UsdGeom.Mesh):
        meshes.append(UsdGeom.Mesh(root_prim))
    for child in root_prim.GetPrim().GetChildren():
        if child.IsA(UsdGeom.Mesh):
            meshes.append(UsdGeom.Mesh(child))
    return meshes

def get_sim_mesh(root_prim):
    sim_mesh = UsdGeom.Mesh()
    for mesh in get_meshes(root_prim):
        if mesh.GetPrim().HasAPI("OmniPhysicsSurfaceDeformableSimAPI"):
            sim_mesh = mesh
    return sim_mesh

class MeshType:
    SIM_DEFAULT = 0
    SIM_BIND = 1
    SIM_REST = 2
    COLL_DEFAULT = 3
    COLL_BIND = 4

class DeformableVisualizationTestKitStage(PhysicsKitStageAsyncTestCase):
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
        self._default_path = "/World"
        sphereLight = UsdLux.SphereLight.Define(self._stage, Sdf.Path("/SphereLight"))
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

        self._upAxis = UsdGeom.GetStageUpAxis(self._stage)
        self._defaultPrimPath = self._stage.GetDefaultPrim().GetPath()

        # add physics scene
        omni.kit.commands.execute("AddPhysicsScene", stage=self._stage, path='/World/PhysicsScene')

    def add_groundplane(self):
        AddGroundPlaneCommand.execute(self._stage, '/CollisionPlane',
                                      self._upAxis, 1000.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def get_session_layer_xform_path(self, path: Sdf.Path) -> Sdf.Path:
        session_base_path = Sdf.Path("/PhysxProxiesVisualization")
        path_mangled = path.pathString.replace("/", "_")
        return session_base_path.AppendChild(path_mangled)

    def get_proxy_sim_mesh_path(self, path: Sdf.Path) -> Sdf.Path:
        path_mangled = self.get_session_layer_xform_path(path)
        return path_mangled.AppendChild("_sim_mesh")

    def get_proxy_coll_mesh_path(self, path: Sdf.Path) -> Sdf.Path:
        path_mangled = self.get_session_layer_xform_path(path)   
        return path_mangled.AppendChild("_coll_mesh")

    def get_proxy_rest_shape_path(self, path: Sdf.Path) -> Sdf.Path:
        path_mangled = self.get_session_layer_xform_path(path)
        return path_mangled.AppendChild("_rest_shape")

    def get_proxy_attachment_path(self, path: Sdf.Path) -> Sdf.Path:
        path_mangled = self.get_session_layer_xform_path(path)
        return path_mangled.AppendChild("_attachment")

    def is_skin_mesh_shown(self, skin_mesh_path: Sdf.Path):
        imageable = UsdGeom.Imageable.Get(self._stage, skin_mesh_path)
        if imageable:
            purpose = imageable.GetPurposeAttr().Get()
            return purpose == "default"
        else:
            return False

    def is_proxy_tet_mesh_shown(self, tet_mesh_path: Sdf.Path):
        imageable = UsdGeom.Imageable.Get(self._stage, tet_mesh_path)
        if imageable:
            visible = imageable.GetVisibilityAttr().Get()
            return visible == "inherited"
        else:
            return False

    def is_actual_tet_mesh_shown(self, tet_mesh_path: Sdf.Path):
        imageable = UsdGeom.Imageable.Get(self._stage, tet_mesh_path)
        if imageable:
            purpose = imageable.GetPurposeAttr().Get()
            return purpose == "default"
        else:
            return False

    def check_deformable_bodies_viz(self, sim_mesh_path, coll_mesh_path, selected_mesh_path, visualizer_mode, display_type):
        is_actual_sim_mesh_shown = True
        is_actual_coll_mesh_shown = True
        is_proxy_sim_mesh_shown = False
        is_proxy_coll_mesh_shown = False
        is_proxy_rest_shape_shown = False

        if visualizer_mode == pb.VisualizerMode.ALL or (visualizer_mode == pb.VisualizerMode.SELECTED and sim_mesh_path == selected_mesh_path):
            is_actual_sim_mesh_shown = False
            is_actual_coll_mesh_shown = False

            if display_type == MeshType.SIM_DEFAULT:
                is_proxy_sim_mesh_shown = True
            elif display_type == MeshType.SIM_BIND:
                # don't have bind pose in these tests, so the visualizer enables the user purpose
                is_proxy_sim_mesh_shown = False
                is_actual_sim_mesh_shown = True
                is_actual_coll_mesh_shown = is_actual_sim_mesh_shown if sim_mesh_path == coll_mesh_path else False
            elif display_type == MeshType.SIM_REST:
                is_proxy_rest_shape_shown = True
            elif display_type == MeshType.COLL_DEFAULT:
                is_proxy_coll_mesh_shown = True
            elif display_type == MeshType.COLL_BIND:
                # don't have bind pose in these tests, so the visualizer enables the user purpose
                is_proxy_coll_mesh_shown = False
                is_actual_coll_mesh_shown = True
                is_actual_sim_mesh_shown = is_actual_coll_mesh_shown if sim_mesh_path == coll_mesh_path else False

            #omni.usd.get_context().get_selection().set_selected_prim_paths([selected_mesh_path.pathString], True)

        self.assertEqual(is_actual_coll_mesh_shown, self.is_actual_tet_mesh_shown(coll_mesh_path))
        self.assertEqual(is_actual_sim_mesh_shown, self.is_actual_tet_mesh_shown(sim_mesh_path))

        if display_type == MeshType.SIM_DEFAULT or display_type == MeshType.SIM_BIND:
            proxy_sim_mesh_path = self.get_proxy_sim_mesh_path(sim_mesh_path)
            self.assertEqual(is_proxy_sim_mesh_shown, self.is_proxy_tet_mesh_shown(proxy_sim_mesh_path))
        elif display_type == MeshType.SIM_REST:
            proxy_rest_shape_path = self.get_proxy_rest_shape_path(sim_mesh_path)
            self.assertEqual(is_proxy_rest_shape_shown, self.is_proxy_tet_mesh_shown(proxy_rest_shape_path))
        elif display_type == MeshType.COLL_DEFAULT or display_type == MeshType.COLL_BIND:
            proxy_coll_mesh_path = self.get_proxy_coll_mesh_path(sim_mesh_path)
            self.assertEqual(is_proxy_coll_mesh_shown, self.is_proxy_tet_mesh_shown(proxy_coll_mesh_path))

    async def _wait_cooking_finished(self):
        while True:
            await omni.kit.app.get_app().next_update_async()
            cooking_statistics = get_physx_cooking_private_interface().get_cooking_statistics()
            running_tasks = cooking_statistics.total_scheduled_tasks - cooking_statistics.total_finished_tasks
            self.assertGreaterEqual(running_tasks, 0)
            if running_tasks <= 0:
                break

    async def test_deformable_viz_multiple_volume_deformables(self):
        await self.base_setup()
        self.add_groundplane()

        meshscale = 50.0

        # deformable body prim 1
        xform_transform1 = create_transform(translate = Gf.Vec3d(0.0,100.0,0.0), scale = Gf.Vec3d(1.0, 2.0, 3.0))
        xform1 = setup_xform_body(self._stage, "/xform1", xform_transform1)

        # sim tetmesh
        tetmesh_transform1 = create_transform(translate = Gf.Vec3d(1.0), scale = Gf.Vec3d(meshscale))
        tetmesh1 = setup_simmesh(self._stage, "/xform1/simMesh", tetmesh_transform1, add_collision = True)

        # deformable body prim 2
        xform_transform2 = create_transform(translate = Gf.Vec3d(100.0,100.0,0.0), scale = Gf.Vec3d(1.0, 2.0, 3.0))
        xform2 = setup_xform_body(self._stage, "/xform2", xform_transform2)

        # sim tetmesh
        tetmesh_transform2 = create_transform(translate = Gf.Vec3d(1.0), scale = Gf.Vec3d(meshscale))
        tetmesh2 = setup_simmesh(self._stage, "/xform2/simMesh", tetmesh_transform2, add_collision = True)

        # skin meshes
        num_skinmeshes = 2
        scalefactor = 1.0
        for i in range(num_skinmeshes):
            #make scale a little smaller than the previous for matrjoschka skin meshes
            scalefactor = scalefactor*0.95
            setup_skinmesh_from_tetmesh(self._stage, "/xform1/triMesh_" + str(i), tetmesh1, scalefactor)
            setup_skinmesh_from_tetmesh(self._stage, "/xform2/triMesh_" + str(i), tetmesh2, scalefactor)

        sim_mesh_paths = [tetmesh1.GetPath(), tetmesh2.GetPath()]
        coll_mesh_paths = sim_mesh_paths  # Single node case
        selected_mesh_path = sim_mesh_paths[0]
        for visualizer_mode in [pb.VisualizerMode.NONE, pb.VisualizerMode.ALL]: # pb.VisualizerMode.SELECTED
            for display_type in [MeshType.SIM_DEFAULT, MeshType.SIM_BIND, MeshType.SIM_REST, MeshType.COLL_DEFAULT, MeshType.COLL_BIND]:
                self._settings.set(pb.SETTING_DISPLAY_DEFORMABLES, visualizer_mode)
                self._settings.set(pb.SETTING_DISPLAY_DEFORMABLE_MESH_TYPE, display_type)
                await self.wait(2)
                for mesh_index in [0, 1]:
                    self.check_deformable_bodies_viz(sim_mesh_paths[mesh_index], coll_mesh_paths[mesh_index], selected_mesh_path, visualizer_mode=visualizer_mode, display_type=display_type)

    async def test_deformable_viz_volume_deformable_single_node(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        # sim tetmesh
        transform = create_transform(translate = Gf.Vec3d(1.0), scale = Gf.Vec3d(5.0))
        simmesh_path = "/tetMesh"
        collmesh_path = simmesh_path
        tetmesh = UsdGeom.TetMesh.Define(stage, simmesh_path)
        tetmesh.AddTransformOp().Set(transform.GetMatrix())
        set_tetmesh_data(tetmesh)

        # deformable body and sim api
        tetmesh.GetPrim().ApplyAPI("OmniPhysicsDeformableBodyAPI")
        tetmesh.GetPrim().ApplyAPI("OmniPhysicsVolumeDeformableSimAPI")
        tetmesh.GetPrim().GetAttribute("omniphysics:restShapePoints").Set(tetmesh.GetPointsAttr().Get())
        tetmesh.GetPrim().GetAttribute("omniphysics:restTetVtxIndices").Set(tetmesh.GetTetVertexIndicesAttr().Get())

        # need to add collision, currently no support without and add surface faces
        collision = UsdPhysics.CollisionAPI.Apply(tetmesh.GetPrim())
        add_surface_faces(tetmesh)

        await self.step(1)

        #for visualizer_mode in [pb.VisualizerMode.NONE, pb.VisualizerMode.ALL]: # pb.VisualizerMode.SELECTED
        for visualizer_mode in [pb.VisualizerMode.ALL]: # pb.VisualizerMode.SELECTED
            for display_type in [MeshType.SIM_DEFAULT, MeshType.SIM_BIND, MeshType.SIM_REST, MeshType.COLL_DEFAULT, MeshType.COLL_BIND]:
                self._settings.set(pb.SETTING_DISPLAY_DEFORMABLES, visualizer_mode)
                self._settings.set(pb.SETTING_DISPLAY_DEFORMABLE_MESH_TYPE, display_type)
                await self.wait(2)
                self.check_deformable_bodies_viz(Sdf.Path(simmesh_path), Sdf.Path(collmesh_path), None, visualizer_mode=visualizer_mode, display_type=display_type)

    async def volume_deformable_viz_hierarch(self, use_scope):
        await self.base_setup()
        self.add_groundplane()

        # deformable body prim
        if use_scope:
            body_transform = create_transform(translate = Gf.Vec3d(10.0,6.0,3.5))
            body = UsdGeom.Scope.Define(self._stage, "/body")
            body.GetPrim().ApplyAPI("OmniPhysicsDeformableBodyAPI")
        else:
            body_transform = create_transform(translate = Gf.Vec3d(10.0,6.0,3.5), scale = Gf.Vec3d(1.0, 2.0, 3.0))
            body = setup_xform_body(self._stage, "/body", body_transform)

        # sim tetmesh
        simmesh_path = "/body/simMesh"
        collmesh_path = simmesh_path
        tetmesh_transform = create_transform(translate = Gf.Vec3d(1.0), scale = Gf.Vec3d(5.0))
        tetmesh = setup_simmesh(self._stage, simmesh_path, tetmesh_transform, add_collision = True)

        await self.step(1)

        for visualizer_mode in [pb.VisualizerMode.NONE, pb.VisualizerMode.ALL]: # pb.VisualizerMode.SELECTED
            for display_type in [MeshType.SIM_DEFAULT, MeshType.SIM_BIND, MeshType.SIM_REST, MeshType.COLL_DEFAULT, MeshType.COLL_BIND]:
                self._settings.set(pb.SETTING_DISPLAY_DEFORMABLES, visualizer_mode)
                self._settings.set(pb.SETTING_DISPLAY_DEFORMABLE_MESH_TYPE, display_type)
                await self.wait(2)
                self.check_deformable_bodies_viz(Sdf.Path(simmesh_path), Sdf.Path(collmesh_path), None, visualizer_mode=visualizer_mode, display_type=display_type)

    async def test_deformable_viz_volume_deformable_hierarch(self):
        await self.volume_deformable_viz_hierarch(use_scope=True)
        await self.volume_deformable_viz_hierarch(use_scope=False)

    async def test_deformable_viz_surface_deformable_single_node(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        transform = create_transform(translate = Gf.Vec3d(0.0, 50.0, 0.0))
        simmesh_path = "/mesh"
        collmesh_path = simmesh_path
        mesh = UsdGeom.Mesh.Define(stage, simmesh_path)
        mesh.AddTransformOp().Set(transform.GetMatrix())
        set_trimesh_data(mesh)
        prim = mesh.GetPrim()
        skin_mesh = None

        success = omni.kit.commands.execute("SetSurfaceDeformableBody",
            prim_path=prim.GetPath())
        self.assertTrue(success)

        #cooking should be triggered in the next update
        await self._wait_cooking_finished()

        for visualizer_mode in [pb.VisualizerMode.NONE, pb.VisualizerMode.ALL]: # pb.VisualizerMode.SELECTED
            for display_type in [MeshType.SIM_DEFAULT, MeshType.SIM_BIND, MeshType.SIM_REST, MeshType.COLL_DEFAULT, MeshType.COLL_BIND]:
                self._settings.set(pb.SETTING_DISPLAY_DEFORMABLES, visualizer_mode)
                self._settings.set(pb.SETTING_DISPLAY_DEFORMABLE_MESH_TYPE, display_type)
                await self.wait(2)
                self.check_deformable_bodies_viz(Sdf.Path(simmesh_path), Sdf.Path(collmesh_path), None, visualizer_mode=visualizer_mode, display_type=display_type)

    async def test_deformable_viz_surface_deformable_hierarch(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        xform = setup_xform_auto_body(stage, "/xform", create_transform(translate = Gf.Vec3d(0.0, 50.0, 0.0)))
        skin_mesh = setup_cube_trimesh(stage, "/xform/skin_mesh", create_transform(scale = Gf.Vec3d(20.0)), 10)
        xform.GetPrim().GetRelationship("physxDeformableBody:cookingSourceMesh").SetTargets([skin_mesh.GetPath()])
        sim_mesh = UsdGeom.Mesh.Define(stage, "/xform/sim_mesh")
        sim_mesh.GetPrim().ApplyAPI("OmniPhysicsSurfaceDeformableSimAPI")
        sim_mesh.GetPrim().ApplyAPI(UsdPhysics.CollisionAPI)

        get_physx_cooking_interface().cook_auto_deformable_body(str(xform.GetPath()))

        # test instancing
        sim_mesh = get_sim_mesh(xform)
        simmesh_path = sim_mesh.GetPath()
        collmesh_path = simmesh_path

        await self.step(1)

        for visualizer_mode in [pb.VisualizerMode.NONE, pb.VisualizerMode.ALL]: # pb.VisualizerMode.SELECTED
            for display_type in [MeshType.SIM_DEFAULT, MeshType.SIM_BIND, MeshType.SIM_REST, MeshType.COLL_DEFAULT, MeshType.COLL_BIND]:
                self._settings.set(pb.SETTING_DISPLAY_DEFORMABLES, visualizer_mode)
                self._settings.set(pb.SETTING_DISPLAY_DEFORMABLE_MESH_TYPE, display_type)
                await self.wait(2)
                self.check_deformable_bodies_viz(simmesh_path, collmesh_path, None, visualizer_mode=visualizer_mode, display_type=display_type)

    def is_prim_valid(self, stage, prim_path):
        prim = stage.GetPrimAtPath(prim_path)
        return prim.IsValid()

    async def test_attachment_viz_volume_deformable_rigidBody(self):
        await self.base_setup()
        self.add_groundplane()

        body_transform = create_transform(translate = Gf.Vec3d(10.0,6.0,3.5), scale = Gf.Vec3d(1.0, 2.0, 3.0))
        body_path = self._default_path + "/body"
        body = setup_xform_body(self._stage, body_path, body_transform)

        # Set up volume deformable with separate collision mesh
        simmesh_path = body_path + "/simMesh"
        collmesh_path = body_path + "/collMesh"
        tetmesh_transform = create_transform(translate = Gf.Vec3d(1.0), scale = Gf.Vec3d(5.0))
        simmesh = setup_simmesh(self._stage, simmesh_path, tetmesh_transform, add_collision = False)
        collmesh = setup_collmesh(self._stage, collmesh_path, tetmesh_transform)

        # Set up 
        cubeGeometry = omni.physx.scripts.physicsUtils.add_cube(self._stage, self._default_path + "/cubeShape", 10.0, Gf.Vec3f(10.0, 6.0, 15))
        SetRigidBodyCommand.execute(cubeGeometry.GetPath(), "", True)

        target_attachment_path = body.GetPath().AppendElementString("attachment")
        target_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(target_attachment_path), False))
        omni.kit.commands.execute("CreateAutoDeformableAttachment", target_attachment_path=target_attachment_path, attachable0_path=body_path, attachable1_path=cubeGeometry.GetPath())

        self._settings.set(pb.SETTING_DISPLAY_DEFORMABLES, pb.VisualizerMode.ALL)
        for display_type in [MeshType.SIM_DEFAULT, MeshType.SIM_BIND, MeshType.SIM_REST, MeshType.COLL_DEFAULT, MeshType.COLL_BIND]:
            self._settings.set(pb.SETTING_DISPLAY_DEFORMABLE_MESH_TYPE, display_type)
            self._settings.set(pb.SETTING_DISPLAY_DEFORMABLE_ATTACHMENTS, False)

            proxy_attachment_path = self.get_proxy_attachment_path(target_attachment_path.AppendChild("vtx_xform_attachment"))
            await self.wait(2)
            self.assertFalse(self.is_prim_valid(self._stage, proxy_attachment_path))            

            self._settings.set(pb.SETTING_DISPLAY_DEFORMABLE_ATTACHMENTS, True)
            await self.wait(2)
            self.assertTrue(self.is_prim_valid(self._stage, proxy_attachment_path))
