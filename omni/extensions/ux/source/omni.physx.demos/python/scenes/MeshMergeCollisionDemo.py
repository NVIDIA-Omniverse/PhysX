# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.commands
from omni.physx.scripts.physicsUtils import set_or_add_translate_op
from pxr import UsdGeom, Sdf, Gf, Vt, PhysicsSchemaTools, UsdPhysics, PhysxSchema
import omni.physxdemos as demo


class MeshMergeCollisionDemo(demo.Base):
    title = "Mesh Merge Collision"
    category = demo.Categories.RIGID_BODIES
    short_description = "Demo showing mesh merge collision usage"
    description = "Demo showing mesh merge collision feature. In this demo notice that collision is one convex hull encapsulating all meshes except for the cube mesh which is excluded. The mesh merge collision collection defines what meshes will be merged and then approximated. Please enable collision debug visualization (eye icon) to see the shape of the produced collision representation. Press play (space) to run the simulation."

    def create(self, stage):
        self._stage = stage
        self.defaultPrimPath, scene = demo.setup_physics_scene(self, stage)

        self._room = demo.get_demo_room(self, stage, hasTable=False, floorOffset=-30.0)

        # prepare top prim for rigid body
        rigid_body_path = self.defaultPrimPath + "/rigidBody"
        rigid_body_prim = UsdGeom.Xform.Define(stage, rigid_body_path).GetPrim()

        UsdPhysics.RigidBodyAPI.Apply(rigid_body_prim)

        result, tmp_path = omni.kit.commands.execute("CreateMeshPrim", prim_type="Torus", select_new_prim=False)
        torus0_path = rigid_body_path + "/torus0"
        omni.kit.commands.execute("MovePrim", path_from=tmp_path, path_to=torus0_path)

        prim_mesh = UsdGeom.Mesh.Get(stage, torus0_path)
        set_or_add_translate_op(prim_mesh, translate=Gf.Vec3f(50.0, 0.0, 0.0))
        prim_mesh.CreateDisplayColorAttr().Set([demo.get_primary_color(1.0)])

        result, tmp_path = omni.kit.commands.execute("CreateMeshPrim", prim_type="Torus", select_new_prim=False)
        torus1_path = rigid_body_path + "/torus1"
        omni.kit.commands.execute("MovePrim", path_from=tmp_path, path_to=torus1_path)

        prim_mesh = UsdGeom.Mesh.Get(stage, torus1_path)
        set_or_add_translate_op(prim_mesh, translate=Gf.Vec3f(-50.0, 0.0, 0.0))
        prim_mesh.CreateDisplayColorAttr().Set([demo.get_primary_color(1.0)])

        result, tmp_path = omni.kit.commands.execute("CreateMeshPrim", prim_type="Cube", select_new_prim=False)
        cube_path = rigid_body_path + "/cube"
        omni.kit.commands.execute("MovePrim", path_from=tmp_path, path_to=cube_path)

        prim_mesh = UsdGeom.Mesh.Get(stage, cube_path)
        set_or_add_translate_op(prim_mesh, translate=Gf.Vec3f(0.0, 0.0, 0.0))
        prim_mesh.CreateDisplayColorAttr().Set([demo.get_primary_color(0.5)])

        # collision API needs to be applied to read the collision parameters
        UsdPhysics.CollisionAPI.Apply(rigid_body_prim)
        col_mesh_api = UsdPhysics.MeshCollisionAPI.Apply(rigid_body_prim)
        col_mesh_api.GetApproximationAttr().Set("convexHull")

        # setup the mesh merge collision
        mesh_merge_api = PhysxSchema.PhysxMeshMergeCollisionAPI.Apply(rigid_body_prim)

        # setup the collection that defines what meshes should be merged
        collection_api = mesh_merge_api.GetCollisionMeshesCollectionAPI()
        collection_api.GetIncludesRel().AddTarget(rigid_body_path)
        collection_api.GetExcludesRel().AddTarget(cube_path)


    
