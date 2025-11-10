# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.commands
import omni.physxdemos as demo
import omni.physx.bindings._physx as physx_settings_bindings
from omni.physx.scripts import physicsUtils, deformableUtils
from pxr import UsdGeom, Sdf, Gf, PhysxSchema
import carb

deformable_beta_on = carb.settings.get_settings().get_as_bool(physx_settings_bindings.SETTING_ENABLE_DEFORMABLE_BETA)

class DeformableAttachmentsDemo(demo.Base):
    title = "Deformable Attachments"
    category = demo.Categories.DEFORMABLES if deformable_beta_on else demo.Categories.NONE
    short_description = "Deformable attachments"
    description = "This snippet shows different types of deformable attachments."

    @staticmethod
    def create_sphere_mesh(stage, target_path):
        _, tmp_path = omni.kit.commands.execute("CreateMeshPrim", prim_type="Sphere", select_new_prim=False)
        omni.kit.commands.execute("MovePrim", path_from=tmp_path, path_to=target_path)
        omni.usd.get_context().get_selection().set_selected_prim_paths([], False)
        return UsdGeom.Mesh.Get(stage, target_path)

    def create(self, stage):
        default_prim_path, scene = demo.setup_physics_scene(self, stage, metersPerUnit=1.0)
        _ = demo.get_demo_room(self, stage, zoom=0.8, hasTable=False)

        scale = 0.5

        # Create a material (make a bit squishier for clearer deformation results)
        deformable_body_material_path = Sdf.Path(default_prim_path + "/deformableMaterial")
        deformableUtils.add_deformable_material(stage, deformable_body_material_path, youngs_modulus=5000000.0)

        box_color = demo.get_static_color()

        # Left and right collider
        cube_left_prim = physicsUtils.add_rigid_box(stage, default_prim_path + "/cubeLeft", size=Gf.Vec3f(1.0) * scale, position=Gf.Vec3f(2.0, 0.0, 5) * scale, density=0.0, color=box_color)
        cube_right_prim = physicsUtils.add_rigid_box(stage, default_prim_path + "/cubeRight", size=Gf.Vec3f(1.0) * scale, position=Gf.Vec3f(-2.0, 0.0, 5) * scale, density=0.0, color=box_color)

        blob_color = demo.get_primary_color()

        # Create dangling meshes and add deformables
        blob_left_xform = UsdGeom.Xform.Define(stage, default_prim_path + "/blobLeft")
        blob_left_mesh = self.create_sphere_mesh(stage, default_prim_path + "/blobLeft/mesh")
        blob_left_mesh.ClearXformOpOrder()
        blob_left_mesh.AddTranslateOp().Set(Gf.Vec3f(2.0, 0.0, 3.2) * scale)
        blob_left_mesh.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1.0, 1.0, 3.0) * scale)
        blob_left_mesh.CreateDisplayColorAttr().Set([blob_color])

        deformableUtils.create_auto_volume_deformable_hierarchy(stage,
            root_prim_path = blob_left_xform.GetPath(),
            simulation_tetmesh_path = default_prim_path + "/blobLeft/simMesh",
            collision_tetmesh_path = default_prim_path + "/blobLeft/collMesh",
            cooking_src_mesh_path = blob_left_mesh.GetPath(),
            simulation_hex_mesh_enabled = True,
            cooking_src_simplification_enabled = False,
            set_visibility_with_guide_purpose = True
        )
        # Set resolution attribute of PhysxAutoDeformableHexahedralMeshAPI
        blob_left_xform.GetPrim().GetAttribute("physxDeformableBody:resolution").Set(15)
        physicsUtils.add_physics_material_to_prim(stage, blob_left_xform.GetPrim(), deformable_body_material_path)

        blob_right_xform = UsdGeom.Xform.Define(stage, default_prim_path + "/blobRight")
        blob_right_mesh = self.create_sphere_mesh(stage, default_prim_path + "/blobRight/mesh")
        blob_right_mesh.ClearXformOpOrder()
        blob_right_mesh.AddTranslateOp().Set(Gf.Vec3f(-2.0, 0.0, 3.2) * scale)
        blob_right_mesh.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1.0, 1.0, 3.0) * scale)
        blob_right_mesh.CreateDisplayColorAttr().Set([blob_color])

        deformableUtils.create_auto_volume_deformable_hierarchy(stage,
            root_prim_path = blob_right_xform.GetPath(),
            simulation_tetmesh_path = default_prim_path + "/blobRight/simMesh",
            collision_tetmesh_path = default_prim_path + "/blobRight/collMesh",
            cooking_src_mesh_path = blob_right_mesh.GetPath(),
            simulation_hex_mesh_enabled = True,
            cooking_src_simplification_enabled = False,
            set_visibility_with_guide_purpose = True
        )
        # Set resolution attribute of PhysxAutoDeformableHexahedralMeshAPI
        blob_right_xform.GetPrim().GetAttribute("physxDeformableBody:resolution").Set(15)
        physicsUtils.add_physics_material_to_prim(stage, blob_right_xform.GetPrim(), deformable_body_material_path)

        # Create connecting mesh and add deformable
        blob_horizontal_xform = UsdGeom.Xform.Define(stage, default_prim_path + "/blobHorizontal")
        blob_horizontal_mesh = self.create_sphere_mesh(stage, default_prim_path + "/blobHorizontal/mesh")
        blob_horizontal_mesh.ClearXformOpOrder()
        blob_horizontal_mesh.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 1.8) * scale)
        blob_horizontal_mesh.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(4.3, 1.0, 1.0) * scale)
        blob_horizontal_mesh.CreateDisplayColorAttr().Set([blob_color])

        deformableUtils.create_auto_volume_deformable_hierarchy(stage,
            root_prim_path = blob_horizontal_xform.GetPath(),
            simulation_tetmesh_path = default_prim_path + "/blobHorizontal/simMesh",
            collision_tetmesh_path = default_prim_path + "/blobHorizontal/collMesh",
            cooking_src_mesh_path = blob_horizontal_mesh.GetPath(),
            simulation_hex_mesh_enabled = True,
            cooking_src_simplification_enabled = False,
            set_visibility_with_guide_purpose = True
        )
        # Set resolution attribute of PhysxAutoDeformableHexahedralMeshAPI
        blob_horizontal_xform.GetPrim().GetAttribute("physxDeformableBody:resolution").Set(15)
        physicsUtils.add_physics_material_to_prim(stage, blob_horizontal_xform.GetPrim(), deformable_body_material_path)

        # Create attachments

        attachment_cd_left_path = blob_left_xform.GetPath().AppendElementString("attachment")
        deformableUtils.create_auto_deformable_attachment(stage,
            target_attachment_path = attachment_cd_left_path,
            attachable0_path = blob_left_xform.GetPath(),
            attachable1_path = cube_left_prim.GetPath()
        )

        attachment_cd_right_path = blob_right_xform.GetPath().AppendElementString("attachment")
        deformableUtils.create_auto_deformable_attachment(stage,
            target_attachment_path = attachment_cd_right_path,
            attachable0_path = blob_right_xform.GetPath(),
            attachable1_path = cube_right_prim.GetPath()
        )

        attachment_dd_left_path = blob_horizontal_xform.GetPath().AppendElementString("attachment_left")
        deformableUtils.create_auto_deformable_attachment(stage,
            target_attachment_path = attachment_dd_left_path,
            attachable0_path = blob_horizontal_xform.GetPath(),
            attachable1_path = blob_left_xform.GetPath()
        )

        attachment_dd_right_path = blob_horizontal_xform.GetPath().AppendElementString("attachment_right")
        deformableUtils.create_auto_deformable_attachment(stage,
            target_attachment_path = attachment_dd_right_path,
            attachable0_path = blob_horizontal_xform.GetPath(),
            attachable1_path = blob_right_xform.GetPath()
        )
