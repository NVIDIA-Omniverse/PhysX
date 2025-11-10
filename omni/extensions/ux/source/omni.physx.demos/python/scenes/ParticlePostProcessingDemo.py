# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.scripts import physicsUtils, particleUtils
from pxr import Sdf, UsdGeom, Gf, PhysxSchema, Vt
import omni.usd
import omni.physxdemos as demo
from .ParticleDemoBaseDemo import ParticleDemoBase
import math


class ParticlePostProcessingDemo(demo.Base, ParticleDemoBase):
    title = "Particle Postprocessing"
    category = demo.Categories.PARTICLES
    short_description = "Basic demo showing particle postprocessing"
    description = (
        "A particle is configured as fluid. Use the checkboxes below to apply postprocessing options."
    )

    params = {
        "Anisotropy": demo.CheckboxParam(False),
        "Smoothing": demo.CheckboxParam(False),
        "Isosurface": demo.CheckboxParam(False),
    }

    kit_settings = {
        "rtx/translucency/maxRefractionBounces": 12,
    }

    def on_shutdown(self):
        ParticleDemoBase.on_shutdown(self)

    def __init__(self):
        ParticleDemoBase.__init__(self, enable_fabric=False, fabric_compatible=False)

    def set_contact_offset_and_shadow(self, stage, path, offsetAmount, restAmount):
        prim = stage.GetPrimAtPath(path)
        physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        physxCollisionAPI.CreateContactOffsetAttr(offsetAmount)
        physxCollisionAPI.CreateRestOffsetAttr(restAmount)
        prim.CreateAttribute("primvars:doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)

    def create(self, stage, Anisotropy, Smoothing, Isosurface):
        default_prim_path = self.setup_base_scene(stage)

        # Create render material
        material_path = self.create_pbd_material("OmniPBR")

        # Create particle system
        particle_system_path = default_prim_path.AppendChild("particleSystem")
        particle_system = PhysxSchema.PhysxParticleSystem.Define(stage, particle_system_path)
        particle_system.CreateSimulationOwnerRel().SetTargets([self._scene.GetPath()])

        # Use a smaller particle size for nicer fluid, and let the sim figure out the other offsets
        particle_contact_offset = 1.0  # cm because stage is default cm scale with metersPerUnit = 0.01
        particle_system.CreateParticleContactOffsetAttr().Set(particle_contact_offset)

        # Limit particle velocity for better collision detection
        particle_system.CreateMaxVelocityAttr().Set(250.0)

        if Anisotropy:
            # apply api and use all defaults
            PhysxSchema.PhysxParticleAnisotropyAPI.Apply(particle_system.GetPrim())

        if Smoothing:
            # apply api and use all defaults
            PhysxSchema.PhysxParticleSmoothingAPI.Apply(particle_system.GetPrim())

        if Isosurface:
            # apply api and use all defaults
            PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(particle_system.GetPrim())
            # tweak anisotropy min, max, and scale to work better with isosurface:
            if Anisotropy:
                ani_api = PhysxSchema.PhysxParticleAnisotropyAPI.Apply(particle_system.GetPrim())
                ani_api.CreateScaleAttr().Set(5.0)
                ani_api.CreateMinAttr().Set(1.0)  # avoids gaps in surface
                ani_api.CreateMaxAttr().Set(2.0)

        # create material and assign it to the system:
        # the isosurface uses the render material assigned to the particle system, so add
        # particle material to the render material created earlier
        particleUtils.add_pbd_particle_material(stage, material_path)
        omni.kit.commands.execute(
            "BindMaterialCommand",
            prim_path=stage.GetPrimAtPath(particle_system_path).GetPath(),
            material_path=material_path,
            strength=None,
        )

        size = 40.0
        lower = Gf.Vec3f(-size * 0.5)
        # use particle fluid restoffset to create a particle grid, using same formula as simulation, see
        # https://docs.omniverse.nvidia.com/prod_extensions/prod_extensions/ext_physics.html#offset-autocomputation
        particle_fluid_restoffset = 0.99 * 0.6 * particle_contact_offset
        particleSpacing = 2.0 * particle_fluid_restoffset
        num_samples = round(size / particleSpacing) + 1
        positions, velocities = particleUtils.create_particles_grid(
            lower, particleSpacing, num_samples, num_samples, num_samples
        )
        particle_point_instancer_path = default_prim_path.AppendChild("particles")

        particleUtils.add_physx_particleset_pointinstancer(
            stage,
            particle_point_instancer_path,
            Vt.Vec3fArray(positions),
            Vt.Vec3fArray(velocities),
            particle_system_path,
            self_collision=True,
            fluid=True,
            particle_group=0,
            particle_mass=0.001,
            density=0.0,
        )

        point_instancer = UsdGeom.PointInstancer.Get(stage, particle_point_instancer_path)
        physicsUtils.set_or_add_translate_op(point_instancer, translate=Gf.Vec3f(0, 50, 0))

        # get and config sphere prototype:
        particle_prototype_sphere = UsdGeom.Sphere.Get(
            stage, particle_point_instancer_path.AppendChild("particlePrototype0")
        )
        particle_prototype_sphere.CreateRadiusAttr().Set(particle_fluid_restoffset)

        omni.kit.commands.execute(
            "BindMaterialCommand",
            prim_path=particle_prototype_sphere.GetPath(),
            material_path=material_path,
            strength=None,
        )

        # create catch box:
        self.create_particle_box_collider(
            default_prim_path.AppendChild("box"),
            side_length=70.0,
            height=60.0,
            thickness=4.0,
            translate=Gf.Vec3f(0, -5.0, 0.0),
            add_cylinder_top=False
        )

        basePath = str(default_prim_path)
        paths = [
            basePath + "/box/front",
            basePath + "/box/back",
            basePath + "/box/left",
            basePath + "/box/right"
        ]

        for path in paths:
            self.set_contact_offset_and_shadow(stage, path, 2.0, 1.0)
