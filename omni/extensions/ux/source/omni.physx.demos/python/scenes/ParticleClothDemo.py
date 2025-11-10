# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.scripts import physicsUtils, particleUtils
import omni.physx.bindings._physx as physx_settings_bindings
from pxr import UsdGeom, Gf, Sdf, UsdPhysics
import omni.usd
import omni.kit.commands
import omni.physxdemos as demo
from .ParticleDemoBaseDemo import ParticleDemoBase
import carb

deformable_beta_on = carb.settings.get_settings().get_as_bool(physx_settings_bindings.SETTING_ENABLE_DEFORMABLE_BETA)

class ParticleClothDemo(demo.Base, ParticleDemoBase):
    title = "Particle Cloth"
    category = demo.Categories.PARTICLES if not deformable_beta_on else demo.Categories.NONE
    short_description = "Basic demo showing setup of a plane mesh as a particle cloth"
    description = "A basic plane mesh is setup as a particle simulation cloth mesh."

    def create(self, stage):
        default_prim_path = self.setup_base_scene(stage, zoomAmount=1.0)

        # create a mesh that is turned into a cloth
        plane_resolution = 100
        plane_width = 400.0

        success, tmp_path = omni.kit.commands.execute(
            "CreateMeshPrimWithDefaultXform",
            prim_type="Plane",
            u_patches=plane_resolution,
            v_patches=plane_resolution,
            u_verts_scale=1,
            v_verts_scale=1,
            half_scale=0.5 * plane_width,
            select_new_prim=False,
        )
        if not success:
            return

        cloth_mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(stage, "/Cloth", True))
        omni.kit.commands.execute("MovePrim", path_from=tmp_path, path_to=cloth_mesh_path)

        cloth_mesh = UsdGeom.Mesh.Define(stage, cloth_mesh_path)
        physicsUtils.setup_transform_as_scale_orient_translate(cloth_mesh)
        physicsUtils.set_or_add_translate_op(cloth_mesh, Gf.Vec3f(400.0, 500.0, 100.0))
        physicsUtils.set_or_add_orient_op(cloth_mesh, Gf.Quatf(0.965925826, Gf.Vec3f(0.0, 0.0, 0.2588190451)))
        physicsUtils.set_or_add_scale_op(cloth_mesh, Gf.Vec3f(1.0))

        # configure and create particle system
        # we use all defaults, so the particle contact offset will be 5cm / 0.05m
        # so the simulation determines the other offsets from the particle contact offset
        particle_system_path = default_prim_path.AppendChild("particleSystem")

        # size rest offset according to plane resolution and width so that particles are just touching at rest
        radius = 0.5 * (plane_width / plane_resolution)
        restOffset = radius
        contactOffset = restOffset * 1.5

        particleUtils.add_physx_particle_system(
            stage=stage,
            particle_system_path=particle_system_path,
            contact_offset=contactOffset,
            rest_offset=restOffset,
            particle_contact_offset=contactOffset,
            solid_rest_offset=restOffset,
            fluid_rest_offset=0.0,
            solver_position_iterations=16,
            simulation_owner=self._scene.GetPath(),
        )

        # create material and assign it to the system:
        particle_material_path = default_prim_path.AppendChild("particleMaterial")
        particleUtils.add_pbd_particle_material(stage, particle_material_path)
        # add some drag and lift to get aerodynamic effects
        particleUtils.add_pbd_particle_material(stage, particle_material_path, drag=0.1, lift=0.3, friction=0.6)
        physicsUtils.add_physics_material_to_prim(
            stage, stage.GetPrimAtPath(particle_system_path), particle_material_path
        )

        # configure as cloth
        stretchStiffness = 10000.0
        bendStiffness = 200.0
        shearStiffness = 100.0
        damping = 0.2
        particleUtils.add_physx_particle_cloth(
            stage=stage,
            path=cloth_mesh_path,
            dynamic_mesh_path=None,
            particle_system_path=particle_system_path,
            spring_stretch_stiffness=stretchStiffness,
            spring_bend_stiffness=bendStiffness,
            spring_shear_stiffness=shearStiffness,
            spring_damping=damping,
            self_collision=True,
            self_collision_filter=True,
        )

        # configure mass:
        particle_mass = 0.02
        num_verts = len(cloth_mesh.GetPointsAttr().Get())
        mass = particle_mass * num_verts
        massApi = UsdPhysics.MassAPI.Apply(cloth_mesh.GetPrim())
        massApi.GetMassAttr().Set(mass)

        # add render material:
        material_path = self.create_pbd_material("OmniPBR")
        omni.kit.commands.execute(
            "BindMaterialCommand", prim_path=cloth_mesh_path, material_path=material_path, strength=None
        )
