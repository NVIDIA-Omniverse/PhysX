# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni
from omni.physx.scripts import deformableUtils, physicsUtils
import omni.physx.bindings._physx as physx_settings_bindings
import omni.physxdemos as demo
import omni.usd
from pxr import UsdGeom, Gf
import carb

deformable_beta_on = carb.settings.get_settings().get_as_bool(physx_settings_bindings.SETTING_ENABLE_DEFORMABLE_BETA)

class VolumeDeformableDemo(demo.Base):
    title = "Volume Deformable"
    category = demo.Categories.DEFORMABLES if deformable_beta_on else demo.Categories.NONE
    short_description = "Volume deformable body setup"
    description = "This snippet sets up a volume deformable body scene"

    @staticmethod
    def create_sphere_mesh(stage, target_path):
        _, tmp_path = omni.kit.commands.execute("CreateMeshPrim", prim_type="Sphere", select_new_prim=False)
        omni.kit.commands.execute("MovePrim", path_from=tmp_path, path_to=target_path)
        omni.usd.get_context().get_selection().set_selected_prim_paths([], False)
        return UsdGeom.Mesh.Get(stage, target_path)

    def create(self, stage):
        default_prim_path, scene = demo.setup_physics_scene(self, stage, metersPerUnit=1.0)
        _ = demo.get_demo_room(self, stage, zoom=0.5)

        # Create xform that serves as the root of the deformable body structure
        deformable_body = UsdGeom.Xform.Define(stage, default_prim_path + "/deformableBody")

        # Create sphere mesh used as the 'skin mesh' for the deformable body.
        skin_mesh = self.create_sphere_mesh(stage, default_prim_path + "/deformableBody/mesh")
        skin_mesh.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, 0.0, 3.0))
        skin_mesh.GetPrim().GetAttribute("xformOp:scale").Set(Gf.Vec3f(2.0, 2.0, 0.5))
        color = demo.get_primary_color()
        skin_mesh.CreateDisplayColorAttr().Set([color])

        # Create tet meshes for simulation and collision based on the skin mesh
        simulation_resolution = 10

        # Apply OmniPhysicsDeformableBodyAPI to root, and setup deformable body structure
        deformableUtils.create_auto_volume_deformable_hierarchy(stage,
            root_prim_path = deformable_body.GetPath(),
            simulation_tetmesh_path = default_prim_path + "/deformableBody/simMesh",
            collision_tetmesh_path = default_prim_path + "/deformableBody/collMesh",
            cooking_src_mesh_path = skin_mesh.GetPath(),
            simulation_hex_mesh_enabled = True,
            cooking_src_simplification_enabled = True,
            set_visibility_with_guide_purpose = True
        )
        # Set resolution attribute of PhysxAutoDeformableHexahedralMeshAPI
        deformable_body.GetPrim().GetAttribute("physxDeformableBody:resolution").Set(simulation_resolution)
        # Apply PhysxBaseDeformableBodyAPI for PhysX specific attributes.
        deformable_body.GetPrim().ApplyAPI("PhysxBaseDeformableBodyAPI")
        deformable_body.GetPrim().GetAttribute("physxDeformableBody:selfCollision").Set(False)

        # Create a deformable material and set it on the deformable body
        deformable_material_path = omni.usd.get_stage_next_free_path(stage, default_prim_path + "/deformableMaterial", True)
        deformableUtils.add_deformable_material(stage, deformable_material_path,
            youngs_modulus=1000000.0,
            poissons_ratio=0.49,
            dynamic_friction=0.5
        )
        physicsUtils.add_physics_material_to_prim(stage, deformable_body.GetPrim(), deformable_material_path)
