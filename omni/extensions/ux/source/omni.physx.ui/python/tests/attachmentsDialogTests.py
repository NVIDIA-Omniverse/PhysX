# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

class AttachmentsDialogTests(omni.kit.test.AsyncTestCase):
    async def base_setup(self):
        self._stage = await utils.new_stage_setup()

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

    async def test_is_overlap(self):
        await self.base_setup()

        mesh_type_list = ['Cone', 'Cube']
        cone, cube = self._create_mesh_prims(mesh_type_list)
        self.set_prim_translation(cone.GetPrim(), Gf.Vec3d((9.0, 127.0, 97.0)))
        self.set_prim_translation(cube.GetPrim(), Gf.Vec3d(-105.0, 127.0, 117.0))

        cone_path = cone.GetPath()
        cube_path = cube.GetPath()
        self.assertFalse(physxAttachmentsDialog.is_overlap_deprecated(self._stage, cone_path, cube_path, 0.1))
        self.assertTrue(physxAttachmentsDialog.is_overlap_deprecated(self._stage, cone_path, cube_path, 0.3))

    async def test_compute_overlapped_paths_pairs(self):
        await self.base_setup()

        mesh_type_list = ['Sphere', 'Torus', 'Cylinder', 'Cube']
        sphere, torus, cylinder, cube = self._create_mesh_prims(mesh_type_list)
        self.set_prim_translation(sphere.GetPrim(), Gf.Vec3d(85.7, 89.1, 133.0))
        self.set_prim_translation(torus.GetPrim(), Gf.Vec3d(144.6, 92.7, -152.2))
        self.set_prim_translation(cylinder.GetPrim(), Gf.Vec3d(-145.5, 95.6, 126.5))
        self.set_prim_translation(cube.GetPrim(), Gf.Vec3d(-137.8, 100.4, -166.2))

        plane = self._create_tesselated_plane_mesh(u_resolution=20, v_resolution=20, scale=300.0)
        self.set_prim_translation(plane.GetPrim(), Gf.Vec3d(0, 88.1, 0))

        execute("AddDeformableBodyComponent", skin_mesh_path=sphere.GetPath())
        execute("AddDeformableBodyComponent", skin_mesh_path=torus.GetPath())
        execute("AddParticleClothComponent", prim_path=plane.GetPath())
        SetRigidBodyCommand.execute(cylinder.GetPath(), "", False)
        SetRigidBodyCommand.execute(cube.GetPath(), "", False)

        particlecloths = [plane.GetPath()]
        deformablebodies = [sphere.GetPath(), torus.GetPath()]
        deformablesurfaces = []
        rigids = [cylinder.GetPath(), cube.GetPath()]
        overlap_tolerance = 0.01
        result = physxAttachmentsDialog.compute_overlapped_paths_pairs_deprecated(self._stage, particlecloths, deformablebodies, deformablesurfaces, rigids, overlap_tolerance)
        target_pairs = {plane.GetPath(): [sphere.GetPath(), torus.GetPath(), cylinder.GetPath(), cube.GetPath()]}
        self.assertTrue(result == target_pairs)
