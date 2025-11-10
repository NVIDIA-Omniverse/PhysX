# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.scripts import physicsUtils, particleUtils
from pxr import UsdGeom, Sdf, Gf, PhysxSchema
import omni.usd
import omni.physxdemos as demo
from .ParticleDemoBaseDemo import ParticleDemoBase


class ParticleSamplerDemo(demo.Base, ParticleDemoBase):
    title = "Particle Sampler"
    category = demo.Categories.PARTICLES
    short_description = "Basic demo showing particle sampler use"
    description = "A mesh has the particle sampler API applied which results in a particle set from sampling points in/on the mesh geometry."

    params = {
        "Sample_Volume": demo.CheckboxParam(True),
        "Particle_Contact_Offset": demo.FloatParam(1.0, 1.0, 7.0, 1.0),
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

    def create(self, stage, Sample_Volume, Particle_Contact_Offset):
        default_prim_path = self.setup_base_scene(stage)

        # configure and create particle system
        particle_system_path = default_prim_path.AppendChild("particleSystem")
        particle_system = PhysxSchema.PhysxParticleSystem.Define(stage, particle_system_path)
        particle_system.CreateSimulationOwnerRel().SetTargets([self._scene.GetPath()])
        # The simulation determines the other offsets from the particle contact offset
        particle_system.CreateParticleContactOffsetAttr().Set(Particle_Contact_Offset)
        # Limit particle velocity for better collision detection
        particle_system.CreateMaxVelocityAttr().Set(250.0)

        # create particle material and assign it to the system:
        particle_material_path = default_prim_path.AppendChild("particleMaterial")
        particleUtils.add_pbd_particle_material(stage, particle_material_path)
        physicsUtils.add_physics_material_to_prim(
            stage, stage.GetPrimAtPath(particle_system_path), particle_material_path
        )

        # create a cube mesh that shall be sampled:
        cube_mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(stage, "/Cube", True))
        cube_resolution = (
            2  # resolution can be low because we'll sample the surface / volume only irrespective of the vertex count
        )
        omni.kit.commands.execute(
            "CreateMeshPrimWithDefaultXform", prim_type="Cube", u_patches=cube_resolution, v_patches=cube_resolution, select_new_prim=False
        )
        cube_mesh = UsdGeom.Mesh.Get(stage, cube_mesh_path)
        physicsUtils.set_or_add_translate_op(cube_mesh, Gf.Vec3f(0.0, 55.0, 0.0))
        physicsUtils.set_or_add_scale_op(cube_mesh, Gf.Vec3f(0.4))

        # configure target particle set:
        particle_points_path = default_prim_path.AppendChild("sampledParticles")
        points = UsdGeom.Points.Define(stage, particle_points_path)
        # add render material:
        material_path = self.create_pbd_material("OmniPBR")
        omni.kit.commands.execute(
            "BindMaterialCommand", prim_path=particle_points_path, material_path=material_path, strength=None
        )

        particle_set_api = PhysxSchema.PhysxParticleSetAPI.Apply(points.GetPrim())
        PhysxSchema.PhysxParticleAPI(particle_set_api).CreateParticleSystemRel().SetTargets([particle_system_path])

        # compute particle sampler sampling distance
        # use particle fluid restoffset to determine sampler distance, using same formula as simulation, see
        # https://docs.omniverse.nvidia.com/prod_extensions/prod_extensions/ext_physics.html#offset-autocomputation
        fluid_rest_offset = 0.99 * 0.6 * Particle_Contact_Offset
        particle_sampler_distance = 2.0 * fluid_rest_offset

        # reference the particle set in the sampling api
        sampling_api = PhysxSchema.PhysxParticleSamplingAPI.Apply(cube_mesh.GetPrim())
        sampling_api.CreateParticlesRel().AddTarget(particle_points_path)
        sampling_api.CreateSamplingDistanceAttr().Set(particle_sampler_distance)
        sampling_api.CreateMaxSamplesAttr().Set(5e5)
        sampling_api.CreateVolumeAttr().Set(Sample_Volume)

        # create catch box:
        self.create_particle_box_collider(
            default_prim_path.AppendChild("box"),
            side_length=70.0,
            height=50.0,
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
