# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import omni.kit.test
from pxr import UsdGeom, Gf, Sdf, Usd
import omni.usd
import omni.kit.commands
from omni.physxtests import utils
from omni.physxui.scripts import physxAttachmentsDialog
from omni.kit.commands import execute
from omni.physxcommands import SetRigidBodyCommand
from omni.physx.scripts.ifaces import get_physx_cooking_private_interface

class AttachmentsDialogTests(omni.kit.test.AsyncTestCase):
    async def base_setup(self):
        self._stage = await utils.new_stage_setup()

    async def _wait_cooking_finished(self):
        while True:
            await omni.kit.app.get_app().next_update_async()
            cooking_statistics = get_physx_cooking_private_interface().get_cooking_statistics()
            running_tasks = cooking_statistics.total_scheduled_tasks - cooking_statistics.total_finished_tasks
            if running_tasks <= 0:
                break

    @staticmethod
    def set_prim_translation(prim: Usd.Prim, translateVec: Gf.Vec3d):
        translate_mtx = Gf.Matrix4d().SetTranslate(translateVec)
        omni.kit.commands.execute("TransformPrim", path=prim.GetPath(), new_transform_matrix=translate_mtx)

    def _create_mesh_prims(self, prim_type_list: list) -> list:
        mesh_list = []
        for prim_type in prim_type_list:
            path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/" + prim_type, True))
            omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type=prim_type)
            mesh = UsdGeom.Mesh.Get(self._stage, path)
            self.assertTrue(mesh)
            mesh_list.append(mesh)
        return mesh_list

    def _create_deformable(self, name, mesh_type="Sphere"):
        xform_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/" + name, True))
        xform = UsdGeom.Xform.Define(self._stage, xform_path)
        _, tmp_path = omni.kit.commands.execute("CreateMeshPrim", prim_type=mesh_type, select_new_prim=False)
        mesh_path = xform_path.AppendChild("mesh")
        omni.kit.commands.execute("MovePrim", path_from=tmp_path, path_to=str(mesh_path))

        sim_mesh_path = xform_path.AppendChild("simMesh")
        coll_mesh_path = xform_path.AppendChild("collMesh")
        execute("CreateAutoVolumeDeformableHierarchy",
            root_prim_path=xform_path,
            simulation_tetmesh_path=sim_mesh_path,
            collision_tetmesh_path=coll_mesh_path,
            cooking_src_mesh_path=mesh_path,
            simulation_hex_mesh_enabled=True,
            cooking_src_simplification_enabled=False)
        return xform

    async def test_is_overlap(self):
        await self.base_setup()

        mesh_type_list = ['Cone', 'Cube']
        cone, cube = self._create_mesh_prims(mesh_type_list)
        self.set_prim_translation(cone.GetPrim(), Gf.Vec3d((9.0, 127.0, 97.0)))
        self.set_prim_translation(cube.GetPrim(), Gf.Vec3d(-105.0, 127.0, 117.0))

        cone_path = cone.GetPath()
        cube_path = cube.GetPath()
        self.assertFalse(physxAttachmentsDialog.is_overlap(self._stage, cone_path, cube_path, 0.1))
        self.assertTrue(physxAttachmentsDialog.is_overlap(self._stage, cone_path, cube_path, 0.3))

    async def test_compute_overlapped_paths_pairs(self):
        await self.base_setup()

        omni.kit.commands.execute("AddPhysicsScene", stage=self._stage, path='/World/PhysicsScene')

        sphere_xform = self._create_deformable("sphere", "Sphere")
        torus_xform = self._create_deformable("torus", "Torus")

        await self._wait_cooking_finished()

        self.set_prim_translation(sphere_xform.GetPrim(), Gf.Vec3d(85.7, 89.1, 133.0))
        self.set_prim_translation(torus_xform.GetPrim(), Gf.Vec3d(144.6, 92.7, -152.2))

        mesh_type_list = ['Cylinder', 'Cube']
        cylinder, cube = self._create_mesh_prims(mesh_type_list)
        self.set_prim_translation(cylinder.GetPrim(), Gf.Vec3d(-145.5, 95.6, 126.5))
        self.set_prim_translation(cube.GetPrim(), Gf.Vec3d(-137.8, 100.4, -166.2))
        SetRigidBodyCommand.execute(cylinder.GetPath(), "", False)
        SetRigidBodyCommand.execute(cube.GetPath(), "", False)

        deformables = [sphere_xform.GetPath(), torus_xform.GetPath()]
        xformables = [cylinder.GetPath(), cube.GetPath()]
        overlap_tolerance = 0.01
        result = physxAttachmentsDialog.compute_overlapped_paths_pairs(self._stage, deformables, xformables, overlap_tolerance)
        self.assertIsInstance(result, dict)
