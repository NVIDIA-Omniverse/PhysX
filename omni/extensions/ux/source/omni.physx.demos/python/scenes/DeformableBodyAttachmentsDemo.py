# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.commands
import omni.physxdemos as demo

from omni.physx.scripts import physicsUtils, deformableUtils
import omni.physx.bindings._physx as physx_settings_bindings
from pxr import UsdGeom, Sdf, Gf, PhysxSchema
import carb

deformable_beta_on = carb.settings.get_settings().get_as_bool(physx_settings_bindings.SETTING_ENABLE_DEFORMABLE_BETA)

class DeformableBodyAttachmentsDemo(demo.Base):
    title = "Deformable Body Attachments"
    category = demo.Categories.DEFORMABLES if not deformable_beta_on else demo.Categories.NONE
    short_description = "Deformable body attachments"
    description = "This snippet shows the different types of deformable body attachments."

    @staticmethod
    def create_sphere_mesh(stage, target_path):
        _, tmp_path = omni.kit.commands.execute("CreateMeshPrim", prim_type="Sphere", select_new_prim=False)
        omni.kit.commands.execute("MovePrim", path_from=tmp_path, path_to=target_path)
        omni.usd.get_context().get_selection().set_selected_prim_paths([], False)
        return UsdGeom.Mesh.Get(stage, target_path)

    def create(self, stage):
        default_prim_path, scene = demo.setup_physics_scene(self, stage)
        _ = demo.get_demo_room(self, stage, zoom=0.8, hasTable=False)

        scale = 0.5

        # Create a material (make a bit squishier for clearer deformation results)
        deformable_body_material_path = Sdf.Path(default_prim_path + "/deformableBodyMaterial")
        deformableUtils.add_deformable_body_material(stage, deformable_body_material_path, youngs_modulus=50000.0)

        box_color = demo.get_static_color()

        # Left and right collider
        cube_left_prim = physicsUtils.add_rigid_box(stage, default_prim_path + "/cubeLeft", size=Gf.Vec3f(100.0) * scale, position=Gf.Vec3f(200.0, 0.0, 500) * scale, density=0.0, color=box_color)
        cube_right_prim = physicsUtils.add_rigid_box(stage, default_prim_path + "/cubeRight", size=Gf.Vec3f(100.0) * scale, position=Gf.Vec3f(-200.0, 0.0, 500) * scale, density=0.0, color=box_color)

        blob_color = demo.get_primary_color()

        # Create dangling meshes and add deformables
        blob_left_mesh = self.create_sphere_mesh(stage, default_prim_path + "/blobLeft")
        blob_left_mesh.ClearXformOpOrder()
        blob_left_mesh.AddTranslateOp().Set(Gf.Vec3f(200.0, 0.0, 320.0) * scale)
        blob_left_mesh.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1.0, 1.0, 3.0) * scale)
        blob_left_mesh.CreateDisplayColorAttr().Set([blob_color])

        deformableUtils.add_physx_deformable_body(
            stage, blob_left_mesh.GetPath(), collision_simplification=False, simulation_hexahedral_resolution=15
        )

        physicsUtils.add_physics_material_to_prim(stage, blob_left_mesh.GetPrim(), deformable_body_material_path)

        blob_right_mesh = self.create_sphere_mesh(stage, default_prim_path + "/blobRight")
        blob_right_mesh.ClearXformOpOrder()
        blob_right_mesh.AddTranslateOp().Set(Gf.Vec3f(-200.0, 0.0, 320.0) * scale)
        blob_right_mesh.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1.0, 1.0, 3.0) * scale)
        blob_right_mesh.CreateDisplayColorAttr().Set([blob_color])

        deformableUtils.add_physx_deformable_body(
            stage, blob_right_mesh.GetPath(), collision_simplification=False, simulation_hexahedral_resolution=15
        )

        physicsUtils.add_physics_material_to_prim(stage, blob_right_mesh.GetPrim(), deformable_body_material_path)

        # Create connecting mesh and add deformable
        blob_horizontal_mesh = self.create_sphere_mesh(stage, default_prim_path + "/blobHorizontal")
        blob_horizontal_mesh.ClearXformOpOrder()
        blob_horizontal_mesh.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 180.0) * scale)
        blob_horizontal_mesh.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(4.3, 1.0, 1.0) * scale)
        blob_horizontal_mesh.CreateDisplayColorAttr().Set([blob_color])

        deformableUtils.add_physx_deformable_body(
            stage, blob_horizontal_mesh.GetPath(), collision_simplification=False, simulation_hexahedral_resolution=15
        )

        physicsUtils.add_physics_material_to_prim(stage, blob_horizontal_mesh.GetPrim(), deformable_body_material_path)

        # Create attachments
        attachment_cd_left_path = blob_left_mesh.GetPath().AppendElementString("attachment")
        attachment_cd_left = PhysxSchema.PhysxPhysicsAttachment.Define(stage, attachment_cd_left_path)
        attachment_cd_left.GetActor0Rel().SetTargets([blob_left_mesh.GetPath()])
        attachment_cd_left.GetActor1Rel().SetTargets([cube_left_prim.GetPath()])
        PhysxSchema.PhysxAutoAttachmentAPI.Apply(attachment_cd_left.GetPrim())

        attachment_cd_right_path = blob_right_mesh.GetPath().AppendElementString("attachment")
        attachment_cd_right = PhysxSchema.PhysxPhysicsAttachment.Define(stage, attachment_cd_right_path)
        attachment_cd_right.GetActor0Rel().SetTargets([blob_right_mesh.GetPath()])
        attachment_cd_right.GetActor1Rel().SetTargets([cube_right_prim.GetPath()])
        PhysxSchema.PhysxAutoAttachmentAPI.Apply(attachment_cd_right.GetPrim())

        attachment_dd_left_path = blob_horizontal_mesh.GetPath().AppendElementString("attachment_left")
        attachment_dd_left = PhysxSchema.PhysxPhysicsAttachment.Define(stage, attachment_dd_left_path)
        attachment_dd_left.GetActor0Rel().SetTargets([blob_horizontal_mesh.GetPath()])
        attachment_dd_left.GetActor1Rel().SetTargets([blob_left_mesh.GetPath()])
        PhysxSchema.PhysxAutoAttachmentAPI.Apply(attachment_dd_left.GetPrim())

        attachment_dd_right_path = blob_horizontal_mesh.GetPath().AppendElementString("attachment_right")
        attachment_dd_right = PhysxSchema.PhysxPhysicsAttachment.Define(stage, attachment_dd_right_path)
        attachment_dd_right.GetActor0Rel().SetTargets([blob_horizontal_mesh.GetPath()])
        attachment_dd_right.GetActor1Rel().SetTargets([blob_right_mesh.GetPath()])
        PhysxSchema.PhysxAutoAttachmentAPI.Apply(attachment_dd_right.GetPrim())
