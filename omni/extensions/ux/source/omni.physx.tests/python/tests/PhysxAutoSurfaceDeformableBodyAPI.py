# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx import get_physx_interface, get_physx_cooking_interface, get_physx_cooking_private_interface
from omni.physxtests import utils
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory, PhysicsBaseAsyncTestCase
from omni.physx.scripts import deformableMeshUtils, deformableUtils, physicsUtils
import omni.kit.commands
import omni.kit.undo
from pxr import Sdf, Usd, Gf, UsdShade, UsdGeom, UsdPhysics, UsdUtils, Vt, PhysicsSchemaTools, PhysxSchema


def create_transform(translate = Gf.Vec3d(0.0),
                        rotate = Gf.Rotation(Gf.Quatd(1.0)),
                        scale = Gf.Vec3d(1.0),
                        pivot_pos = Gf.Vec3d(0.0),
                        pivot_orient = Gf.Rotation(Gf.Quatd(1.0))):
    return Gf.Transform(translate, rotate, scale, pivot_pos, pivot_orient)

def create_transform_copy(transform):
    return Gf.Transform(transform.GetTranslation(), transform.GetRotation(), transform.GetScale(),
        transform.GetPivotPosition(), transform.GetPivotOrientation())

def set_trimesh_data(trimesh):
    tri_points, tri_indices = deformableMeshUtils.createTriangleMeshCube(3)
    trimesh.GetPointsAttr().Set(tri_points)
    trimesh.GetFaceVertexCountsAttr().Set([3]*(len(tri_indices)//3))
    trimesh.GetFaceVertexIndicesAttr().Set(tri_indices)

def compute_world_bounds(imageable: UsdGeom.Imageable) -> Gf.Range3d:
    obb = imageable.ComputeWorldBound(Usd.TimeCode.Default(), purpose1=imageable.GetPurposeAttr().Get())
    return obb.ComputeAlignedBox()

def setup_xform(stage, path, transform: Gf.Transform):
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTransformOp().Set(transform.GetMatrix())
    return xform

def setup_cube_trimesh(stage, path, transform: Gf.Transform, dim: int):
    tri_points, tri_indices = deformableMeshUtils.createTriangleMeshCube(dim)
    skinmesh = UsdGeom.Mesh.Define(stage, path)
    skinmesh.AddTransformOp().Set(transform.GetMatrix())
    skinmesh.GetPointsAttr().Set(tri_points)
    skinmesh.GetFaceVertexCountsAttr().Set([3]*(len(tri_indices)//3))
    skinmesh.GetFaceVertexIndicesAttr().Set(tri_indices)
    return skinmesh

def setup_sphere_trimesh_with_command(stage, path, transform: Gf.Transform):
    omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Sphere", prim_path=path)
    tmp_path = str(stage.GetDefaultPrim().GetPath()) + path
    omni.kit.commands.execute("MovePrim", path_from=tmp_path, path_to=path)
    skinmesh = UsdGeom.Mesh.Get(stage, path)
    skinmesh.ClearXformOpOrder()
    skinmesh.AddTransformOp().Set(transform.GetMatrix())
    return skinmesh

def setup_voxel_sphere_trimesh(stage, path, transform: Gf.Transform, dim: int):
    points, indices = deformableMeshUtils.createTetraVoxelSphere(dim)
    tri_points, tri_indices = deformableUtils.extractTriangleSurfaceFromTetra(points, indices)
    skinmesh = UsdGeom.Mesh.Define(stage, path)
    skinmesh.AddTransformOp().Set(transform.GetMatrix())
    skinmesh.GetPointsAttr().Set(tri_points)
    skinmesh.GetFaceVertexCountsAttr().Set([3]*(len(tri_indices)//3))
    skinmesh.GetFaceVertexIndicesAttr().Set(tri_indices)
    return skinmesh

def get_schema_instances(prim: Usd.Prim, schema_type_name: str):
    return {s[len(schema_type_name) + 1:] for s in prim.GetAppliedSchemas() if s.startswith(schema_type_name)}

def has_pose(prim: Usd.Prim):
    return len(get_schema_instances(prim, "OmniPhysicsDeformablePoseAPI")) > 0

def get_pose_name_with_purpose(prim: Usd.Prim, purpose_name: str):
    pose_names = get_schema_instances(prim, "OmniPhysicsDeformablePoseAPI")
    for pose_name in pose_names:
        if purpose_name in prim.GetAttribute("deformablePose:" + pose_name + ":omniphysics:purposes").Get():
            return pose_name
    return None

def has_pose_with_purpose(prim: Usd.Prim, purpose_name: str):
    return get_pose_name_with_purpose(prim, purpose_name) is not None

def check_body_apis(test, stage, body_path, expect_body=True, expect_auto=True, cooking_src_path=None):
    prim = stage.GetPrimAtPath(body_path)
    test.assertTrue(prim.HasAPI("OmniPhysicsDeformableBodyAPI") == expect_body)
    test.assertTrue(prim.HasAPI("PhysxAutoDeformableBodyAPI") == expect_auto)
    test.assertTrue(prim.HasRelationship("physxDeformableBody:cookingSourceMesh") == (cooking_src_path is not None))
    if cooking_src_path is not None:
        cookingSrcRel = prim.GetRelationship("physxDeformableBody:cookingSourceMesh")
        test.assertTrue(len(cookingSrcRel.GetTargets()) == 1)
        test.assertTrue(cookingSrcRel.GetTargets()[0] == Sdf.Path(cooking_src_path))

def check_pose(test, prim, purpose_name, expect_purpose=True):
    test.assertTrue(has_pose(prim) == expect_purpose)
    test.assertTrue(has_pose_with_purpose(prim, purpose_name) == expect_purpose)

def check_sim_apis(test, stage, sim_path, expect_sim=True, expect_bindpose=True):
    prim = stage.GetPrimAtPath(sim_path)
    test.assertTrue(prim.IsA(UsdGeom.Mesh))
    test.assertTrue(prim.HasAPI("OmniPhysicsSurfaceDeformableSimAPI") == expect_sim)
    check_pose(test, prim, "bindPose", expect_bindpose)

def check_coll_apis(test, stage, coll_path, expect_coll=True, expect_bindpose=True):
    prim = stage.GetPrimAtPath(coll_path)
    test.assertTrue(prim.IsA(UsdGeom.Mesh))
    test.assertTrue(prim.HasAPI(UsdPhysics.CollisionAPI) == expect_coll)
    check_pose(test, prim, "bindPose", expect_bindpose)

def check_skin_apis(test, stage, skin_paths, expect_bindpose=True):
    for skin_path in skin_paths:
        prim = stage.GetPrimAtPath(skin_path)
        check_pose(test, prim, "bindPose", expect_bindpose)

def check_pose_points(test, prim, purpose_name, expect_purpose=True):
    pose_name = get_pose_name_with_purpose(prim, purpose_name)
    test.assertTrue(expect_purpose == (pose_name is not None))
    if pose_name is not None:
        pose_point_attr = prim.GetAttribute("deformablePose:" + pose_name + ":omniphysics:points")
        pose_points = pose_point_attr.Get()
        test.assertTrue(len(pose_points) > 0)

def check_sim_attrs(test, stage, sim_path, expect_bindpose=True):
    prim = stage.GetPrimAtPath(sim_path)
    mesh = UsdGeom.Mesh(prim)
    mesh_points = mesh.GetPointsAttr().Get()
    test.assertTrue(len(mesh_points) > 0)
    mesh_face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get()
    test.assertTrue(len(mesh_face_vertex_counts) > 0)
    mesh_face_vertex_indices = mesh.GetFaceVertexIndicesAttr().Get()
    test.assertTrue(len(mesh_face_vertex_indices) > 0 and len(mesh_face_vertex_indices) % 3 == 0)
    rest_shape_points = prim.GetAttribute("omniphysics:restShapePoints").Get()
    test.assertTrue(len(rest_shape_points) > 0)
    rest_shape_indices = prim.GetAttribute("omniphysics:restTriVtxIndices").Get()
    test.assertTrue(len(rest_shape_indices) > 0)
    check_pose_points(test, prim, "bindPose", expect_bindpose)

def check_coll_attrs(test, stage, coll_path, expect_bindpose=True):
    prim = stage.GetPrimAtPath(coll_path)
    mesh = UsdGeom.Mesh(prim)
    mesh_points = mesh.GetPointsAttr().Get()
    test.assertTrue(len(mesh_points) > 0)
    mesh_face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get()
    test.assertTrue(len(mesh_face_vertex_counts) > 0)
    mesh_face_vertex_indices = mesh.GetFaceVertexIndicesAttr().Get()
    test.assertTrue(len(mesh_face_vertex_indices) > 0 and len(mesh_face_vertex_indices) % 3 == 0)
    check_pose_points(test, prim, "bindPose", expect_bindpose)

def check_skin_attrs(test, stage, skin_paths, expect_bindpose=True):
    for skin_path in skin_paths:
        prim = stage.GetPrimAtPath(skin_path)
        check_pose_points(test, prim, "bindPose", expect_bindpose)

#debug class:
#class PhysxAutoSurfaceDeformableBodyAPITestDebugStage(PhysicsBaseAsyncTestCase):
class PhysxAutoSurfaceDeformableBodyAPITestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core

    async def new_stage(self):
        class_name = self.__class__.__name__
        if class_name == 'PhysxAutoSurfaceDeformableBodyAPITestDebugStage':
            return await utils.new_stage_setup(def_up_and_mpu=True, up=UsdGeom.Tokens.z, mpu=1.0)
        else:
            return await super().new_stage(def_up_and_mpu=True, up=UsdGeom.Tokens.z, mpu=1.0)

    def step(self, num_steps):
        dtime = 1.0/60.0
        class_name = self.__class__.__name__
        if class_name == 'PhysxAutoSurfaceDeformableBodyAPITestDebugStage':
            physx_interface = get_physx_interface()
            time = 0.0
            for i in range(num_steps):
                physx_interface.update_simulation(dtime, time)
                physx_interface.update_transformations(True, True, True, False)
                time = time + dtime
        else:
            super().step(num_steps=num_steps, dt=dtime)

    async def _wait_cooking_finished(self):
        while True:
            await omni.kit.app.get_app().next_update_async()
            cooking_statistics = get_physx_cooking_private_interface().get_cooking_statistics()
            running_tasks = cooking_statistics.total_scheduled_tasks - cooking_statistics.total_finished_tasks
            self.assertGreaterEqual(running_tasks, 0)
            if running_tasks <= 0:
                break

    async def test_physx_surface_deformable_from_skin(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        xform = setup_xform(stage, "/xform", create_transform(translate = Gf.Vec3d(0.0, 0.0, 5.0)))
        skin_mesh = setup_cube_trimesh(stage, "/xform/skin_mesh", create_transform(scale = Gf.Vec3d(2.0)), 10)

        success = deformableUtils.create_auto_surface_deformable_hierarchy(stage,
            root_prim_path = "/xform",
            simulation_mesh_path = "/xform/sim_mesh",
            cooking_src_mesh_path = "/xform/skin_mesh",
            cooking_src_simplification_enabled = True
        )
        self.assertTrue(success)
        check_body_apis(self, stage, "/xform", cooking_src_path="/xform/skin_mesh")
        check_sim_apis(self, stage, "/xform/sim_mesh")
        check_coll_apis(self, stage, "/xform/sim_mesh")
        check_skin_apis(self, stage, ["/xform/skin_mesh"])

        success = get_physx_cooking_interface().cook_auto_deformable_body("/xform")
        self.assertTrue(success)
        check_sim_attrs(self, stage, "/xform/sim_mesh")
        check_coll_attrs(self, stage, "/xform/sim_mesh")
        check_skin_attrs(self, stage, ["/xform/skin_mesh"])

        # test instantiation
        sim_mesh = UsdGeom.Mesh(stage.GetPrimAtPath("/xform/sim_mesh"))
        initial_point = sim_mesh.GetPointsAttr().Get()[0]

        # step and check that it's fallen a bit under gravity
        self.step(1)

        post_point = sim_mesh.GetPointsAttr().Get()[0]
        epsilon = 0.001
        self.assertTrue(initial_point[2] - post_point[2] > epsilon)


    async def test_physx_surface_deformable_runtime_remove_body(self):
        stage = await self.new_stage()
        UsdPhysics.Scene.Define(stage, "/physicsScene")
        xform = setup_xform(stage, "/xform", create_transform(translate = Gf.Vec3d(0.0, 0.0, 5.0)))
        skin_mesh = setup_cube_trimesh(stage, "/xform/skin_mesh", create_transform(scale = Gf.Vec3d(2.0)), 10)

        success = deformableUtils.create_auto_surface_deformable_hierarchy(stage,
            root_prim_path = "/xform",
            simulation_mesh_path = "/xform/sim_mesh",
            cooking_src_mesh_path = "/xform/skin_mesh",
            cooking_src_simplification_enabled = True
        )
        self.assertTrue(success)
        check_body_apis(self, stage, "/xform", cooking_src_path="/xform/skin_mesh")
        check_sim_apis(self, stage, "/xform/sim_mesh")
        check_coll_apis(self, stage, "/xform/sim_mesh")
        check_skin_apis(self, stage, ["/xform/skin_mesh"])

        success = get_physx_cooking_interface().cook_auto_deformable_body("/xform")
        self.assertTrue(success)
        check_sim_attrs(self, stage, "/xform/sim_mesh")
        check_coll_attrs(self, stage, "/xform/sim_mesh")
        check_skin_attrs(self, stage, ["/xform/skin_mesh"])

        sim_mesh = UsdGeom.Mesh(stage.GetPrimAtPath("/xform/sim_mesh"))

        initial_point = sim_mesh.GetPointsAttr().Get()[0]

        #start simulation
        self.step(2)

        #test falling
        post_sim_point = sim_mesh.GetPointsAttr().Get()[0]
        epsilon = 0.001
        self.assertTrue(initial_point[2] - post_sim_point[2] > epsilon)

        #remove deformable body API
        xform.GetPrim().RemoveAPI("PhysxAutoDeformableBodyAPI")
        xform.GetPrim().RemoveAPI("OmniPhysicsDeformableBodyAPI")

        #resume simulation
        self.step(2)

        #test not falling
        post_remove_point = sim_mesh.GetPointsAttr().Get()[0]
        self.assertTrue(post_sim_point[2] == post_remove_point[2])


    async def test_physx_surface_deformable_welding(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        xform = setup_xform(stage, "/xform", create_transform(translate = Gf.Vec3d(0.0, 0.0, 5.0)))

        tri_points = [
            #2 triangles, disjoint
            Gf.Vec3f(0.0, 0.0, 0.0), Gf.Vec3f(0.0, 0.0, 1.0), Gf.Vec3f(1.0, 0.0, 1.0), #[0, 1, 2]
            Gf.Vec3f(0.0, 0.0, 0.0), Gf.Vec3f(1.0, 0.0, 1.0), Gf.Vec3f(1.0, 0.0, 0.0), #[3, 4, 5]
            #1 isolated degenerate triangle with 1 vertex
            Gf.Vec3f(0.0, 1.0, 0.0),                                                   #[6]
            #1 isolated degenerate triangle with 2 vertices
            Gf.Vec3f(0.0, 2.0, 0.0), Gf.Vec3f(0.0, 2.0, 1.0),                          #[7,8]
            #1 isolated degenerate triangle with 3 vertices
            Gf.Vec3f(0.0, 3.0, 0.0), Gf.Vec3f(0.0, 3.0, 1.0), Gf.Vec3f(0.0, 3.0, 1.0), #[9, 10, 11]
            #1 attached degenerate triangle with 1 extra vertex
            Gf.Vec3f(0.0, 0.0, 2.0),                                                   #[12]
            #1 attached degenerate triangle with 2 extra vertices
            Gf.Vec3f(1.0, 0.0, 0.0), Gf.Vec3f(1.0, 0.0, 1.0)                           #[13, 14]
        ]

        tri_indices = [
            #2 triangles, disjoint
            0, 1, 2, 3, 4, 5,
            #1 isolated degenerate triangle with 1 vertex
            6, 6, 6,
            #1 isolated degenerate triangle with 2 vertices
            7, 8, 7,
            #1 isolated degenerate triangle with 3 vertices
            9, 10, 11,
            #1 attached degenerate triangle with 1 extra vertex
            12, 1, 12,
            #1 attached degenerate triangle with 2 extra vertices
            13, 14, 4
        ]

        skinmesh = UsdGeom.Mesh.Define(stage, "/xform/skin_mesh")
        skinmesh.AddTransformOp().Set(create_transform(scale = Gf.Vec3d(2.0)).GetMatrix())
        skinmesh.GetPointsAttr().Set(tri_points)
        skinmesh.GetFaceVertexCountsAttr().Set([3]*(len(tri_indices)//3))
        skinmesh.GetFaceVertexIndicesAttr().Set(tri_indices)

        success = deformableUtils.create_auto_surface_deformable_hierarchy(stage,
            "/xform",
            "/xform/sim_mesh",
            "/xform/skin_mesh",
            False
        )

        get_physx_cooking_interface().cook_auto_deformable_body(str(xform.GetPath()))

        sim_mesh = UsdGeom.Mesh(stage.GetPrimAtPath("/xform/sim_mesh"))
        initial_point = sim_mesh.GetPointsAttr().Get()[0]

        # step and check that it's fallen a bit under gravity
        self.step(1)

        post_point = sim_mesh.GetPointsAttr().Get()[0]
        epsilon = 0.001
        self.assertTrue(initial_point[2] - post_point[2] > epsilon)


    ####################################################################################
    # stopped working with memory state (physx update not triggered with _wait_cooking_finished)
    ####################################################################################

class PhysxAutoSurfaceDeformableBodyAPITestStage(PhysicsBaseAsyncTestCase):
    category = TestCategory.Core

    async def new_stage(self):
        return await utils.new_stage_setup(def_up_and_mpu=True, up=UsdGeom.Tokens.z, mpu=1.0)

    def step(self, num_steps):
        dtime = 1.0/60.0
        physx_interface = get_physx_interface()
        time = 0.0
        for i in range(num_steps):
            physx_interface.update_simulation(dtime, time)
            physx_interface.update_transformations(True, True, True, False)
            time = time + dtime

    async def _wait_cooking_finished(self):
        while True:
            await omni.kit.app.get_app().next_update_async()
            cooking_statistics = get_physx_cooking_private_interface().get_cooking_statistics()
            running_tasks = cooking_statistics.total_scheduled_tasks - cooking_statistics.total_finished_tasks
            self.assertGreaterEqual(running_tasks, 0)
            if running_tasks <= 0:
                break


    cooking_inputs = [
        "xform:transform",
        "skinmesh:transform",
        "skinmesh:posepurpose",
        "skinmesh:posepoints",
        "skinmesh:vertexCounts",
        "skinmesh:vertexIndices"
    ]

    async def physx_surface_deformable_update_cooking_input(self, cooking_input, is_runtime):
        stage = await self.new_stage()
        print(cooking_input)

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        xform = setup_xform(stage, "/xform", create_transform(translate = Gf.Vec3d(0.0, 0.0, 5.0)))
        skin_mesh = setup_cube_trimesh(stage, "/xform/skin_mesh", create_transform(scale = Gf.Vec3d(2.0)), 10)

        success = deformableUtils.create_auto_surface_deformable_hierarchy(stage,
            root_prim_path = "/xform",
            simulation_mesh_path = "/xform/sim_mesh",
            cooking_src_mesh_path = "/xform/skin_mesh",
            cooking_src_simplification_enabled = True
        )
        self.assertTrue(success)
        check_body_apis(self, stage, "/xform", cooking_src_path="/xform/skin_mesh")
        check_sim_apis(self, stage, "/xform/sim_mesh")
        check_coll_apis(self, stage, "/xform/sim_mesh")
        check_skin_apis(self, stage, ["/xform/skin_mesh"])

        success = get_physx_cooking_interface().cook_auto_deformable_body("/xform")
        self.assertTrue(success)
        check_sim_attrs(self, stage, "/xform/sim_mesh")
        check_coll_attrs(self, stage, "/xform/sim_mesh")
        check_skin_attrs(self, stage, ["/xform/skin_mesh"])

        sim_mesh = UsdGeom.Mesh(stage.GetPrimAtPath("/xform/sim_mesh"))

        if is_runtime:
            self.step(5)

        deformableBodyDataCrcOrig = xform.GetPrim().GetAttribute("physxDeformableBody:deformableBodyDataCrc").Get()
        expect_cooking_change = False

        if cooking_input == "xform:transform":
            scaleOp = xform.AddScaleOp()
            scaleOp.Set(Gf.Vec3f(1.0, 1.0, 2.0))
            expect_cooking_change = not is_runtime # TODO should be true, but resync currently not supported for scaling changes

        elif cooking_input == "skinmesh:transform":
            scaleOp = skin_mesh.AddScaleOp()
            scaleOp.Set(Gf.Vec3f(1.0, 1.0, 2.0))
            expect_cooking_change = not is_runtime # TODO should be true, but resync currently not supported for scaling changes

        elif cooking_input == "skinmesh:posepurpose":
            skin_bind_purposes_attr = skin_mesh.GetPrim().GetAttribute("deformablePose:default:omniphysics:purposes")
            skin_bind_purposes_attr.Set(["bindPose", "bindPose"])
            expect_cooking_change = False

        elif cooking_input == "skinmesh:posepoints":
            skin_bind_points_attr = skin_mesh.GetPrim().GetAttribute("deformablePose:default:omniphysics:points")
            points = skin_bind_points_attr.Get()
            points[0] += Gf.Vec3f(0.001, 0.0, 0.0)
            skin_bind_points_attr.Set(points)
            expect_cooking_change = True

        elif cooking_input == "skinmesh:vertexCounts":
            counts = skin_mesh.GetFaceVertexCountsAttr().Get()
            counts_new = [c for c in counts]
            counts_new.pop()
            skin_mesh.GetFaceVertexCountsAttr().Set(counts_new)
            expect_cooking_change = True
            
        elif cooking_input == "skinmesh:vertexIndices":
            indices = skin_mesh.GetFaceVertexIndicesAttr().Get()
            indices_new = [i for i in indices]
            indices_new.extend([0, 1, 2])
            skin_mesh.GetFaceVertexIndicesAttr().Set(indices_new)
            expect_cooking_change = True

        if is_runtime:
            # the resync of the deformable body is only processed with a simulation step
            self.step(1)
        else:
            await self._wait_cooking_finished()

        if expect_cooking_change:
            deformableBodyDataCrcNew = xform.GetPrim().GetAttribute("physxDeformableBody:deformableBodyDataCrc").Get()
            self.assertTrue(deformableBodyDataCrcOrig != deformableBodyDataCrcNew)


    async def test_physx_surface_deformable_update_cooking_input_non_runtime(self):
        for cooking_input in self.cooking_inputs:
            await self.physx_surface_deformable_update_cooking_input(cooking_input, False)


    async def test_physx_surface_deformable_update_cooking_input_runtime(self):
        for cooking_input in self.cooking_inputs:
            await self.physx_surface_deformable_update_cooking_input(cooking_input, True)


    ####################################################################################
    # utils
    ####################################################################################

    async def test_physx_create_auto_surface_deformable_hierarchy(self):
        stage = await self.new_stage()

        xform_transform = create_transform(translate = Gf.Vec3d(0.0, 0.0, 1.0))
        mesh_transform = create_transform(scale = Gf.Vec3d(2.0))

        xform = UsdGeom.Xform.Define(stage, "/xform_cooking_src")
        xform.AddTransformOp().Set(xform_transform.GetMatrix())
        cooking_src_mesh = setup_sphere_trimesh_with_command(stage, "/xform_cooking_src/mesh", mesh_transform)
        cooking_src_mesh.MakeInvisible()

        xform = UsdGeom.Xform.Define(stage, "/xform")
        xform.AddTransformOp().Set(xform_transform.GetMatrix())

        skin_mesh_0 = setup_voxel_sphere_trimesh(stage, "/xform/skin_mesh_0", mesh_transform, 10)
        skin_mesh_1 = setup_voxel_sphere_trimesh(stage, "/xform/skin_mesh_1", mesh_transform, 20)

        success = deformableUtils.create_auto_surface_deformable_hierarchy(stage,
            "/xform",
            "/xform/sim_mesh",
            "/xform_cooking_src/mesh",
            True
        )
        self.assertTrue(success)
        check_body_apis(self, stage, "/xform", cooking_src_path="/xform_cooking_src/mesh")
        check_sim_apis(self, stage, "/xform/sim_mesh")
        check_coll_apis(self, stage, "/xform/sim_mesh")
        check_skin_apis(self, stage, ["/xform/skin_mesh_0", "/xform/skin_mesh_1"])

        success = get_physx_cooking_interface().cook_auto_deformable_body("/xform")
        self.assertTrue(success)
        check_sim_attrs(self, stage, "/xform/sim_mesh")
        check_coll_attrs(self, stage, "/xform/sim_mesh")
        check_skin_attrs(self, stage, ["/xform/skin_mesh_0", "/xform/skin_mesh_1"])

        sim_mesh = UsdGeom.Mesh.Get(stage, "/xform/sim_mesh")
        self.assertTrue(sim_mesh.GetPrim().IsValid())
        self.assertTrue(len(sim_mesh.GetPointsAttr().Get()) > 0)

        sim_mesh.MakeInvisible()

    ####################################################################################
    #commands
    ####################################################################################

    async def test_physx_surface_deformable_commands(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        transform = create_transform(translate = Gf.Vec3d(0.0, 0.0, 5.0))

        mesh = UsdGeom.Mesh.Define(stage, "/mesh")
        mesh.AddTransformOp().Set(transform.GetMatrix())
        set_trimesh_data(mesh)
        prim = mesh.GetPrim()

        success = omni.kit.commands.execute("SetSurfaceDeformableBody", prim_path="/mesh")
        self.assertTrue(success)
        check_body_apis(self, stage, "/mesh", expect_auto=False)
        check_sim_apis(self, stage, "/mesh", expect_bindpose=False)
        check_coll_apis(self, stage, "/mesh", expect_bindpose=False)
        check_sim_attrs(self, stage, "/mesh", expect_bindpose=False)
        check_coll_attrs(self, stage, "/mesh", expect_bindpose=False)

        omni.kit.undo.undo()
        check_body_apis(self, stage, "/mesh", expect_body=False, expect_auto=False)
        check_sim_apis(self, stage, "/mesh", expect_sim=False, expect_bindpose=False)
        check_coll_apis(self, stage, "/mesh", expect_coll=False, expect_bindpose=False)

        omni.kit.undo.redo()
        check_body_apis(self, stage, "/mesh", expect_auto=False)
        check_sim_apis(self, stage, "/mesh", expect_bindpose=False)
        check_coll_apis(self, stage, "/mesh", expect_bindpose=False)
        check_sim_attrs(self, stage, "/mesh", expect_bindpose=False)
        check_coll_attrs(self, stage, "/mesh", expect_bindpose=False)

        success = omni.kit.commands.execute("RemoveBaseDeformableBodyComponent", prim_path="/mesh")
        self.assertTrue(success)
        check_body_apis(self, stage, "/mesh", expect_body=False, expect_auto=False)
        check_sim_apis(self, stage, "/mesh", expect_sim=False, expect_bindpose=False)
        check_coll_apis(self, stage, "/mesh", expect_coll=False, expect_bindpose=False)

        omni.kit.undo.undo()
        check_body_apis(self, stage, "/mesh", expect_auto=False)
        check_sim_apis(self, stage, "/mesh", expect_bindpose=False)
        check_coll_apis(self, stage, "/mesh", expect_bindpose=False)
        check_sim_attrs(self, stage, "/mesh", expect_bindpose=False)
        check_coll_attrs(self, stage, "/mesh", expect_bindpose=False)

        success = omni.kit.commands.execute("RemoveSurfaceDeformableSimComponent", prim_path="/mesh")
        self.assertTrue(success)
        check_body_apis(self, stage, "/mesh", expect_auto=False)
        check_sim_apis(self, stage, "/mesh", expect_sim=False, expect_bindpose=False)
        check_coll_apis(self, stage, "/mesh", expect_bindpose=False)

        omni.kit.undo.undo()
        check_body_apis(self, stage, "/mesh", expect_auto=False)
        check_sim_apis(self, stage, "/mesh", expect_bindpose=False)
        check_coll_apis(self, stage, "/mesh", expect_bindpose=False)
        check_sim_attrs(self, stage, "/mesh", expect_bindpose=False)
        check_coll_attrs(self, stage, "/mesh", expect_bindpose=False)

        # test instantiation
        initial_point = mesh.GetPointsAttr().Get()[0]

        self.step(1)

        post_point = mesh.GetPointsAttr().Get()[0]
        epsilon = 0.001
        self.assertTrue(initial_point[2] - post_point[2] > epsilon)


    async def test_physx_surface_deformable_hier_commands(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        transform = create_transform(translate = Gf.Vec3d(0.0, 0.0, 5.0))
        xform = UsdGeom.Xform.Define(stage, "/xform")
        xform.AddTransformOp().Set(transform.GetMatrix())
        skin_mesh = setup_cube_trimesh(stage, "/xform/skin_mesh", create_transform(scale = Gf.Vec3d(2.0)), 10)

        success = omni.kit.commands.execute("CreateAutoSurfaceDeformableHierarchy",
            root_prim_path="/xform",
            simulation_mesh_path="/xform/sim_mesh",
            cooking_src_mesh_path="/xform/skin_mesh",
            cooking_src_simplification_enabled=True
        )
        self.assertTrue(success)
        check_body_apis(self, stage, "/xform", cooking_src_path="/xform/skin_mesh")
        check_sim_apis(self, stage, "/xform/sim_mesh")
        check_coll_apis(self, stage, "/xform/sim_mesh")
        check_skin_apis(self, stage, ["/xform/skin_mesh"])

        #cooking should be triggered in the next update
        await self._wait_cooking_finished()
        check_sim_attrs(self, stage, "/xform/sim_mesh")
        check_coll_attrs(self, stage, "/xform/sim_mesh")
        check_skin_attrs(self, stage, ["/xform/skin_mesh"])

        omni.kit.undo.undo()
        check_body_apis(self, stage, "/xform", expect_body=False, expect_auto=False)
        #TODO, why doesn't this work, even though sim_mesh is removed from stage (in stage tree view)?
        #self.assertTrue(not stage.GetPrimAtPath("/xform/sim_mesh").IsValid())
        check_skin_apis(self, stage, ["/xform/skin_mesh"], expect_bindpose=False)

        omni.kit.undo.redo()
        check_body_apis(self, stage, "/xform", cooking_src_path="/xform/skin_mesh")
        check_sim_apis(self, stage, "/xform/sim_mesh")
        check_coll_apis(self, stage, "/xform/sim_mesh")
        check_skin_apis(self, stage, ["/xform/skin_mesh"])

        #cooking should be triggered in the next update
        await self._wait_cooking_finished()
        check_sim_attrs(self, stage, "/xform/sim_mesh")
        check_coll_attrs(self, stage, "/xform/sim_mesh")
        check_skin_attrs(self, stage, ["/xform/skin_mesh"])

        success = omni.kit.commands.execute("RemoveBaseDeformableBodyComponent", prim_path="/xform")
        self.assertTrue(success)
        check_body_apis(self, stage, "/xform", expect_body=False, expect_auto=False)
        check_sim_apis(self, stage, "/xform/sim_mesh", expect_sim=False, expect_bindpose=False)
        check_coll_apis(self, stage, "/xform/sim_mesh", expect_coll=False, expect_bindpose=False)
        check_skin_apis(self, stage, ["/xform/skin_mesh"], expect_bindpose=False)

        omni.kit.undo.undo()
        check_body_apis(self, stage, "/xform", cooking_src_path="/xform/skin_mesh")
        check_sim_apis(self, stage, "/xform/sim_mesh")
        check_coll_apis(self, stage, "/xform/sim_mesh")
        check_skin_apis(self, stage, ["/xform/skin_mesh"])

        check_sim_attrs(self, stage, "/xform/sim_mesh")
        check_coll_attrs(self, stage, "/xform/sim_mesh")
        check_skin_attrs(self, stage, ["/xform/skin_mesh"])

        success = omni.kit.commands.execute("RemoveSurfaceDeformableSimComponent", prim_path="/xform/sim_mesh")
        self.assertTrue(success)
        check_sim_apis(self, stage, "/xform/sim_mesh", expect_sim=False)

        omni.kit.undo.undo()
        check_body_apis(self, stage, "/xform", cooking_src_path="/xform/skin_mesh")
        check_sim_apis(self, stage, "/xform/sim_mesh")
        check_coll_apis(self, stage, "/xform/sim_mesh")
        check_skin_apis(self, stage, ["/xform/skin_mesh"])

        check_sim_attrs(self, stage, "/xform/sim_mesh")
        check_coll_attrs(self, stage, "/xform/sim_mesh")
        check_skin_attrs(self, stage, ["/xform/skin_mesh"])

        success = omni.kit.commands.execute("RemoveDeformablePoseComponent", prim_path="/xform/sim_mesh", instance_name="")
        self.assertTrue(success)
        check_sim_apis(self, stage, "/xform/sim_mesh", expect_bindpose=False)

        omni.kit.undo.undo()
        check_body_apis(self, stage, "/xform", cooking_src_path="/xform/skin_mesh")
        check_sim_apis(self, stage, "/xform/sim_mesh")
        check_coll_apis(self, stage, "/xform/sim_mesh")
        check_skin_apis(self, stage, ["/xform/skin_mesh"])

        check_sim_attrs(self, stage, "/xform/sim_mesh")
        check_coll_attrs(self, stage, "/xform/sim_mesh")
        check_skin_attrs(self, stage, ["/xform/skin_mesh"])

        success = omni.kit.commands.execute("RemoveDeformablePoseComponent", prim_path="/xform/skin_mesh", instance_name="")
        self.assertTrue(success)
        check_skin_apis(self, stage, ["/xform/skin_mesh"], expect_bindpose=False)

        omni.kit.undo.undo()
        check_skin_apis(self, stage, ["/xform/skin_mesh"])

        check_sim_attrs(self, stage, "/xform/sim_mesh")
        check_coll_attrs(self, stage, "/xform/sim_mesh")
        check_skin_attrs(self, stage, ["/xform/skin_mesh"])

        # test instantiation
        sim_mesh = UsdGeom.Mesh(stage.GetPrimAtPath("/xform/sim_mesh"))
        initial_point = sim_mesh.GetPointsAttr().Get()[0]

        self.step(1)

        post_point = sim_mesh.GetPointsAttr().Get()[0]
        epsilon = 0.001
        self.assertTrue(initial_point[2] - post_point[2] > epsilon)

