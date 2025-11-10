# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.scripts import deformableUtils, physicsUtils
import omni.physx.bindings._physx as physx_settings_bindings
from pxr import UsdGeom, Gf, Sdf, UsdPhysics, PhysxSchema
import omni.usd
import omni.kit.commands
import omni.physxdemos as demo
import carb

deformable_beta_on = carb.settings.get_settings().get_as_bool(physx_settings_bindings.SETTING_ENABLE_DEFORMABLE_BETA)

class SurfaceDeformableDemo(demo.Base):
    title = "Surface Deformable"
    category = demo.Categories.DEFORMABLES if deformable_beta_on else demo.Categories.NONE
    short_description = "Surface deformable body setup"
    description = "This snippet sets up a surface deformable body scene"

    @staticmethod
    def create_plane_mesh(stage, target_path):
        plane_resolution = 80
        plane_width = 350.0
        _, tmp_path = omni.kit.commands.execute(
            "CreateMeshPrimWithDefaultXform",
            prim_type="Plane",
            u_patches=plane_resolution,
            v_patches=plane_resolution,
            u_verts_scale=1,
            v_verts_scale=1,
            half_scale=0.5 * plane_width,
            select_new_prim=False,
        )
        omni.kit.commands.execute("MovePrim", path_from=tmp_path, path_to=Sdf.Path(target_path))
        omni.usd.get_context().get_selection().set_selected_prim_paths([], False)
        return UsdGeom.Mesh.Get(stage, target_path)

    def create(self, stage):
        default_prim_path, scene = demo.setup_physics_scene(self, stage, metersPerUnit=1.0)
        _ = demo.get_demo_room(self, stage, zoom=0.5)

        timeStepsPerSecond = 120
        restOffset = 0.01
        contactOffset = 3*restOffset

        scene.GetPrim().ApplyAPI(PhysxSchema.PhysxSceneAPI)
        PhysxSchema.PhysxSceneAPI(scene).GetTimeStepsPerSecondAttr().Set(timeStepsPerSecond)
        PhysxSchema.PhysxSceneAPI(scene).GetGpuMaxDeformableSurfaceContactsAttr().Set(4*1048576)

        # Create xform that serves as the root of the deformable body structure
        deformable_body = UsdGeom.Xform.Define(stage, default_prim_path + "/deformableBody")

        # Create tesselated plane mesh used as the 'skin mesh' for the deformable body.
        skin_mesh = self.create_plane_mesh(stage, default_prim_path + "/deformableBody/mesh")
        physicsUtils.setup_transform_as_scale_orient_translate(skin_mesh)
        color = demo.get_primary_color()
        skin_mesh.CreateDisplayColorAttr().Set([color])

        # Pose deformable body
        physicsUtils.set_or_add_translate_op(deformable_body, Gf.Vec3f(0.0, 0.0, 1.5))
        physicsUtils.set_or_add_orient_op(deformable_body, Gf.Quatf(0.965925826, Gf.Vec3f(0.0, 0.0, 0.2588190451)))
        physicsUtils.set_or_add_scale_op(deformable_body, Gf.Vec3f(1.0))

        # Apply OmniPhysicsDeformableBodyAPI to root, and setup deformable body structure
        deformableUtils.create_auto_surface_deformable_hierarchy(stage,
            root_prim_path = deformable_body.GetPath(),
            simulation_mesh_path = default_prim_path + "/deformableBody/simMesh",
            cooking_src_mesh_path = skin_mesh.GetPath(),
            cooking_src_simplification_enabled = False,
            set_visibility_with_guide_purpose = True
        )

        # Apply PhysxSurfaceDeformableBodyAPI for PhysX specific attributes.
        deformable_body.GetPrim().ApplyAPI("PhysxSurfaceDeformableBodyAPI")
        deformable_body.GetPrim().GetAttribute("physxDeformableBody:selfCollision").Set(True)
        deformable_body.GetPrim().GetAttribute("physxDeformableBody:enableSpeculativeCCD").Set(True)
        deformable_body.GetPrim().GetAttribute("physxDeformableBody:solverPositionIterationCount").Set(16)
        deformable_body.GetPrim().GetAttribute("physxDeformableBody:collisionPairUpdateFrequency").Set(4)
        deformable_body.GetPrim().GetAttribute("physxDeformableBody:collisionIterationMultiplier").Set(4)
        deformable_body.GetPrim().GetAttribute("physxDeformableBody:maxLinearVelocity").Set(contactOffset*timeStepsPerSecond)

        #deformable_body.GetPrim().GetAttribute("physxDeformableBody:maxLinearVelocity").Set(-1.0)

        # Access generated sim mesh prim to turn it invisible and adjust collision properties
        sim_mesh_prim = stage.GetPrimAtPath(default_prim_path + "/deformableBody/simMesh")
        sim_mesh_prim.ApplyAPI(PhysxSchema.PhysxCollisionAPI)
        PhysxSchema.PhysxCollisionAPI(sim_mesh_prim).GetRestOffsetAttr().Set(restOffset)
        PhysxSchema.PhysxCollisionAPI(sim_mesh_prim).GetContactOffsetAttr().Set(contactOffset)

        # Create a deformable material and set it on the deformable body
        deformable_material_path = omni.usd.get_stage_next_free_path(stage, default_prim_path + "/deformableMaterial", True)
        deformableUtils.add_surface_deformable_material(stage, deformable_material_path,
            dynamic_friction=0.2,
            surface_thickness=0.2,
            surface_stretch_stiffness=1000.0,
            surface_shear_stiffness=0.1,
            surface_bend_stiffness=0.2
        )
        physicsUtils.add_physics_material_to_prim(stage, deformable_body.GetPrim(), deformable_material_path)
        # Apply PhysxSurfaceDeformableMaterialAPI for PhysX specific attributes.
        deformable_material_prim = stage.GetPrimAtPath(deformable_material_path)
        deformable_material_prim.ApplyAPI("PhysxSurfaceDeformableMaterialAPI")
        deformable_material_prim.GetAttribute("physxDeformableMaterial:elasticityDamping").Set(0.0)
        deformable_material_prim.GetAttribute("physxDeformableMaterial:bendDamping").Set(0.0)
