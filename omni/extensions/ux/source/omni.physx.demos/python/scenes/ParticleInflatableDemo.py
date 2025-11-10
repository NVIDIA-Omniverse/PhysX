# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.scripts import physicsUtils, particleUtils
import omni.physx.bindings._physx as physx_settings_bindings
from pxr import UsdGeom, Sdf, Gf, PhysxSchema, UsdPhysics
import omni.usd
import omni.physxdemos as demo
from .ParticleDemoBaseDemo import ParticleDemoBase
import carb

deformable_beta_on = carb.settings.get_settings().get_as_bool(physx_settings_bindings.SETTING_ENABLE_DEFORMABLE_BETA)

class ParticleInflatableDemo(demo.Base, ParticleDemoBase):
    title = "Particle Inflatable"
    category = demo.Categories.PARTICLES if not deformable_beta_on else demo.Categories.NONE
    short_description = "Basic demo showing setup of a sphere as a particle inflatable"
    description = "A basic sphere mesh is setup as a particle simulation inflatable cloth mesh."

    def create(self, stage):
        default_prim_path = self.setup_base_scene(stage, zoomAmount=0.8)

        # configure and create particle system
        # we use all defaults, so the particle contact offset will be 5cm / 0.05m
        # so the simulation determines the other offsets from the particle contact offset
        particle_system_path = default_prim_path.AppendChild("particleSystem")
        particle_system = PhysxSchema.PhysxParticleSystem.Define(stage, particle_system_path)
        particle_system.CreateSimulationOwnerRel().SetTargets([self._scene.GetPath()])

        # create material and assign it to the system:
        particle_material_path = default_prim_path.AppendChild("particleMaterial")
        particleUtils.add_pbd_particle_material(stage, particle_material_path)
        physicsUtils.add_physics_material_to_prim(
            stage, stage.GetPrimAtPath(particle_system_path), particle_material_path
        )

        # create a mesh that is turned into an inflatable
        cube_u_resolution = 20
        cube_v_resolution = 20
        cube_w_resolution = 4

        success, tmp_path = omni.kit.commands.execute(
            "CreateMeshPrimWithDefaultXform",
            prim_type="Cube",
            u_patches=cube_u_resolution,
            v_patches=cube_v_resolution,
            w_patches=cube_w_resolution,
            u_verts_scale=1,
            v_verts_scale=1,
            w_verts_scale=1,
            half_scale=60.0,
            select_new_prim=False,
        )

        cloth_mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(stage, "/Inflatable", True))
        omni.kit.commands.execute("MovePrim", path_from=tmp_path, path_to=cloth_mesh_path)

        cloth_mesh = UsdGeom.Mesh.Get(stage, cloth_mesh_path)
        physicsUtils.setup_transform_as_scale_orient_translate(cloth_mesh)
        physicsUtils.set_or_add_translate_op(cloth_mesh, Gf.Vec3f(0.0, 200.0, -20.0))
        physicsUtils.set_or_add_orient_op(cloth_mesh, Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), 10.0).GetQuat()))
        physicsUtils.set_or_add_scale_op(cloth_mesh, Gf.Vec3f(1.0, 1.0, 0.04))

        # configure as cloth
        pressure = 8.0
        stretchStiffness = 20000.0
        bendStiffness = 100.0
        shearStiffness = 100.0
        damping = 0.5
        particleUtils.add_physx_particle_cloth(
            stage=stage,
            path=cloth_mesh_path,
            dynamic_mesh_path=None,
            particle_system_path=particle_system_path,
            spring_stretch_stiffness=stretchStiffness,
            spring_bend_stiffness=bendStiffness,
            spring_shear_stiffness=shearStiffness,
            spring_damping=damping,
            pressure=pressure,
        )

        # configure mass:
        particle_mass = 0.1
        num_verts = len(cloth_mesh.GetPointsAttr().Get())
        mass = particle_mass * num_verts
        massApi = UsdPhysics.MassAPI.Apply(cloth_mesh.GetPrim())
        massApi.GetMassAttr().Set(mass)

        # add render material:
        material_path = self.create_pbd_material("OmniPBR")
        omni.kit.commands.execute(
            "BindMaterialCommand", prim_path=cloth_mesh_path, material_path=material_path, strength=None
        )
