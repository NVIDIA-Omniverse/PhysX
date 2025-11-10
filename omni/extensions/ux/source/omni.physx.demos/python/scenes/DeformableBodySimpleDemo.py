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

class DeformableBodyDemo(demo.Base):
    title = "Deformable Body"
    category = demo.Categories.DEFORMABLES if not deformable_beta_on else demo.Categories.NONE
    short_description = "Deformable body scene setup"
    description = "This snippet sets up a deformable body scene"

    @staticmethod
    def create_sphere_mesh(stage, target_path):
        _, tmp_path = omni.kit.commands.execute("CreateMeshPrim", prim_type="Sphere", select_new_prim=False)
        omni.kit.commands.execute("MovePrim", path_from=tmp_path, path_to=target_path)
        omni.usd.get_context().get_selection().set_selected_prim_paths([], False)
        return UsdGeom.Mesh.Get(stage, target_path)

    def create(self, stage):
        default_prim_path, scene = demo.setup_physics_scene(self, stage)
        _ = demo.get_demo_room(self, stage, zoom=0.5)

        # Create sphere mesh used as the 'skin mesh' for the deformable body
        skin_mesh = self.create_sphere_mesh(stage, default_prim_path + "/deformableBody")
        skin_mesh.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, 0.0, 300.0))
        skin_mesh.GetPrim().GetAttribute("xformOp:scale").Set(Gf.Vec3f(2.0, 2.0, 0.5))

        color = demo.get_primary_color()
        skin_mesh.CreateDisplayColorAttr().Set([color])

        # Create tet meshes for simulation and collision based on the skin mesh
        simulation_resolution = 10

        # Apply PhysxDeformableBodyAPI and PhysxCollisionAPI to skin mesh and set parameter to default values
        _ = deformableUtils.add_physx_deformable_body(
            stage,
            skin_mesh.GetPath(),
            collision_simplification=True,
            simulation_hexahedral_resolution=simulation_resolution,
            self_collision=False,
        )

        # Create a deformable body material and set it on the deformable body
        deformable_material_path = omni.usd.get_stage_next_free_path(stage, default_prim_path + "/deformableBodyMaterial", True)
        deformableUtils.add_deformable_body_material(
            stage,
            deformable_material_path,
            youngs_modulus=10000.0,
            poissons_ratio=0.49,
            damping_scale=0.0,
            dynamic_friction=0.5,
        )
        physicsUtils.add_physics_material_to_prim(stage, skin_mesh.GetPrim(), deformable_material_path)
