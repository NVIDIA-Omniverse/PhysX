# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
import typing
import os

from abc import ABC, abstractmethod
from typing import List

from pxr import Usd, Gf, Sdf, UsdGeom, UsdUtils, UsdLux, Vt
from pxr import UsdPhysics, PhysxSchema

import carb
import carb.settings

import omni.physx.bindings._physx as physx_bindings
from omni.physx.scripts import physicsUtils, particleUtils, deformableUtils, deformableMeshUtils
from omni.physx import get_physx_cooking_interface, get_physx_simulation_interface

import omni.physics.tensors

#import omni.physxfabric

import warp as wp
import numpy as np


def get_asset_root():
    dir = os.path.dirname(__file__)
    return os.path.abspath(os.path.join(dir, "..", "..", "..", "data", "usd"))


def set_physx_device_ordinal_hint(device_ordinal):
    if os.name == "nt":
        os.environ["PHYSXDEVICE_GPU_CUDA_ORDINAL"] = str(device_ordinal)
    else:
        os.environ["PHYSX_GPU_DEVICE"] = str(device_ordinal)


class Transform:
    def __init__(self, p=Gf.Vec3f(0.0, 0.0, 0.0), q=Gf.Quatf(1.0)):
        self.p = p
        self.q = q


class DeviceParams:
    def __init__(self, use_gpu_sim=False, use_gpu_pipeline=False, num_workers=None):
        self.use_gpu_sim = use_gpu_sim
        self.use_gpu_pipeline = use_gpu_pipeline
        self.num_workers = num_workers


class SimParams:
    def __init__(self):
        self.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        self.gravity_mag = 9.81
        self.add_default_light = True
        self.add_default_ground = True

        # omni.physx determines the number of substeps per frame as timeStepsPerSecond / minFrameRate.
        self.time_steps_per_second = 60
        self.min_frame_rate = 60


class GridParams:
    def __init__(self, num_envs=16, env_spacing=2.0):
        self.num_envs = num_envs
        self.env_spacing = env_spacing
        self.num_rows = None
        self.row_spacing = None
        self.col_spacing = None


class SyncParams:
    def __init__(self, sync_usd=True, sync_fabric=False, transforms_only=False):
        self.sync_usd = sync_usd
        self.sync_fabric = sync_fabric
        self.transforms_only = transforms_only


class ScenarioBase(ABC):
    def __init__(self, sim_params, device_params):
        stage = Usd.Stage.CreateInMemory()
        self.stage = stage

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        self.scene_path = Sdf.Path("/physicsScene")
        physics_scene = UsdPhysics.Scene.Define(stage, self.scene_path)
        physics_scene.CreateGravityDirectionAttr().Set(sim_params.gravity_dir)
        physics_scene.CreateGravityMagnitudeAttr().Set(sim_params.gravity_mag)

        physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
        physx_scene.CreateEnableGPUDynamicsAttr().Set(device_params.use_gpu_sim)
        if device_params.use_gpu_sim:
            physx_scene.CreateBroadphaseTypeAttr().Set("GPU")
        else:
            physx_scene.CreateBroadphaseTypeAttr().Set("MBP")

        # disable scene query support
        physx_scene.CreateEnableSceneQuerySupportAttr().Set(False)

        physx_scene.CreateTimeStepsPerSecondAttr().Set(sim_params.time_steps_per_second)
        self.physx_scene = physx_scene

        if sim_params.add_default_ground:
            physicsUtils.add_quad_plane(stage, "/groundPlane", "Z", 20.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        if sim_params.add_default_light:
            self.create_distant_light()

        self.sim_params = sim_params
        self.device_params = device_params
        self.wp_device = "cuda:0" if device_params.use_gpu_pipeline else "cpu"
        self.finished = False
        self.failed = False

        # run at least this many steps; useful to visually inspect quick tests before they disappear
        self.minsteps = None
        # run at most this many steps; terminate tests that fail to finish on their own
        self.maxsteps = None
    
    def set_camera_properties(self, position, target=Gf.Vec3f(0.0, 0.0, 0.0)):
        custom_layer_data = {
            "cameraSettings": {
                "Perspective": {
                    "position": Gf.Vec3d(position[0], position[1], position[2]),
                    "target": Gf.Vec3d(0, 0, 0)
                }
            }
        }
        root_layer = self.stage.GetRootLayer()
        root_layer.customLayerData = custom_layer_data

    def create_rigid_ball(self, actor_path, transform, radius=0.5):
        sphere = UsdGeom.Sphere.Define(self.stage, actor_path)
        sphere.CreateRadiusAttr(radius)
        sphere.AddTranslateOp().Set(transform.p)
        sphere.AddOrientOp().Set(transform.q)
        sphere.CreateDisplayColorAttr().Set([Gf.Vec3f(71.0 / 255.0, 105.0 / 255.0, 1.0)])

        prim = sphere.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim)

        return prim

    def create_multi_shape_rigid_body(self, actor_path, transform, radius=0.5):

        xform = UsdGeom.Xform.Define(self.stage, actor_path)
        physicsUtils.set_or_add_scale_orient_translate(xform, scale=Gf.Vec3f(1.0), orient=transform.q, translate=transform.p)

        sphere = UsdGeom.Sphere.Define(self.stage, actor_path.AppendChild("ball"))
        sphere.CreateRadiusAttr(radius)
        sphere.AddTranslateOp().Set(transform.p)
        sphere.AddOrientOp().Set(transform.q)
        sphere.CreateDisplayColorAttr().Set([Gf.Vec3f(71.0 / 255.0, 105.0 / 255.0, 1.0)])

        box = UsdGeom.Cube.Define(self.stage, actor_path.AppendChild("box"))
        box.CreateSizeAttr(1.0)
        
        box.AddTranslateOp().Set(Gf.Vec3f(transform.p[0], transform.p[1], transform.p[2]-radius))
        box.AddOrientOp().Set(transform.q)
        box.AddScaleOp().Set(Gf.Vec3f(radius, radius, radius))
        box.CreateDisplayColorAttr().Set([Gf.Vec3f(71.0 / 255.0, 105.0 / 255.0, 1.0)])

        prim = xform.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim)

        return prim

    def create_pendulum_articulation(self,
                                     pendulum_path: typing.Union[Sdf.Path, str],
                                     transform: Transform,
                                     link_half_length: float = 0.5,
                                     link_mass: float = 1.0,
                                     add_fixed_joint_link: bool = False,
                                     revolute_joint_axis: str="Z",
                                     revolute_joint_frame_quat: Gf.Quatf = Gf.Quatf(1),
                                     fixed_joint_frame_quat: Gf.Quatf = Gf.Quatf(1),
                                    ) -> dict:

        pendulum_path = Sdf.Path(pendulum_path)
        xform = UsdGeom.Xform.Define(self.stage, pendulum_path)
        physicsUtils.set_or_add_scale_orient_translate(xform, scale=Gf.Vec3f(1.0), orient=transform.q, translate=transform.p)
        xform_prim = xform.GetPrim()

        root_link_path = pendulum_path.AppendChild("RootLink")
        root_link_color = Gf.Vec3f(165.0 / 255.0, 21.0 / 255.0, 21.0 / 255.0)

        child_link_path = pendulum_path.AppendChild("ChildLink")
        child_link_color = Gf.Vec3f(21.0 / 255.0, 165.0 / 255.0, 21.0 / 255.0)

        fixed_joint_link_path = pendulum_path.AppendChild("FixedJointLink")
        fixed_joint_link_color = Gf.Vec3f(21.0 / 255.0, 21.0 / 255.0, 165.0 / 255.0)

        aspect_ratio = 0.1  # ratio half_width / half_length
        link_length = 2.0 * link_half_length
        link_width = link_length * aspect_ratio
        root_radius = link_width * 0.7
        link_size = Gf.Vec3f(link_length, link_width, link_width)

        # make it an articulation
        UsdPhysics.ArticulationRootAPI.Apply(xform_prim)
        articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(xform_prim)
        articulation_api.CreateSleepThresholdAttr(0.0)

        # add fixed root link:
        link_position = Gf.Vec3f(0)
        physicsUtils.add_rigid_sphere(self.stage, root_link_path, root_radius, link_position, color=root_link_color)
        joint = UsdPhysics.FixedJoint.Define(self.stage, root_link_path.AppendChild("FixedJoint"))
        joint.CreateBody1Rel().SetTargets([root_link_path])

        # child link
        link_position += Gf.Vec3f(link_length, 0, 0)
        physicsUtils.add_rigid_box(self.stage, child_link_path, link_size, link_position, color=child_link_color)
        physicsUtils.add_mass(self.stage, child_link_path, link_mass)
        joint = UsdPhysics.RevoluteJoint.Define(self.stage, child_link_path.AppendChild("RevoluteJoint"))
        joint.CreateAxisAttr(revolute_joint_axis)
        joint.CreateBody0Rel().SetTargets([root_link_path])
        joint.CreateBody1Rel().SetTargets([child_link_path])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0))
        joint.CreateLocalRot0Attr().Set(revolute_joint_frame_quat)
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(-0.5, 0, 0))  # link size is achieved via scale
        joint.CreateLocalRot1Attr().Set(revolute_joint_frame_quat)

        if add_fixed_joint_link:
            link_position += Gf.Vec3f(link_length, 0, 0)
            physicsUtils.add_rigid_box(self.stage, fixed_joint_link_path, link_size, link_position, color=fixed_joint_link_color)
            physicsUtils.add_mass(self.stage, fixed_joint_link_path, link_mass)
            joint = UsdPhysics.FixedJoint.Define(self.stage, fixed_joint_link_path.AppendChild("FixedJoint"))
            joint.CreateBody0Rel().SetTargets([child_link_path])
            joint.CreateBody1Rel().SetTargets([fixed_joint_link_path])
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.5, 0, 0))
            joint.CreateLocalRot0Attr().Set(fixed_joint_frame_quat)
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(-0.5, 0, 0))  # link size is achieved via scale
            joint.CreateLocalRot1Attr().Set(fixed_joint_frame_quat)
        else:
            fixed_joint_link_path = Sdf.Path()

        return dict({"root": root_link_path, "child": child_link_path, "fixed": fixed_joint_link_path})

    def create_custom_pendulum_articulation(self,
                                            pendulum_path: typing.Union[Sdf.Path, str],
                                            transform: Transform,
                                            link_half_length: float = 0.5,
                                            link_height = 0.025,
                                            link_mass: float = 1.0,
                                            add_fixed_joint_link: bool = False,
                                            joint_type: str = "D6", # "Spherical", "Revolute"
                                            rotational_joint_frame_quat: Gf.Quatf = Gf.Quatf(1),
                                            usd_d6_free_rot_dof: dict[str, bool] = {'rotX': False, 'rotY': False, 'rotZ': True},
                                            fixed_joint_frame_quat: Gf.Quatf = Gf.Quatf(1),
                                            rotation_axis="X",
                                        ) -> dict:

        pendulum_path = Sdf.Path(pendulum_path)
        xform = UsdGeom.Xform.Define(self.stage, pendulum_path)
        physicsUtils.set_or_add_scale_orient_translate(xform, scale=Gf.Vec3f(1.0), orient=transform.q, translate=transform.p)
        xform_prim = xform.GetPrim()

        root_link_path = pendulum_path.AppendChild("RootLink")
        root_link_color = Gf.Vec3f(165.0 / 255.0, 21.0 / 255.0, 21.0 / 255.0)

        child_link_path = pendulum_path.AppendChild("ChildLink")
        child_link_color = Gf.Vec3f(21.0 / 255.0, 165.0 / 255.0, 21.0 / 255.0)

        fixed_joint_link_path = pendulum_path.AppendChild("FixedJointLink")
        fixed_joint_link_color = Gf.Vec3f(21.0 / 255.0, 21.0 / 255.0, 165.0 / 255.0)

        aspect_ratio = 0.1  # ratio half_width / half_length
        link_length = 2.0 * link_half_length
        link_width = link_length * aspect_ratio
        root_radius = link_height * 2
        link_size = Gf.Vec3f(link_length, link_width, link_width)

        # make it an articulation
        UsdPhysics.ArticulationRootAPI.Apply(xform_prim)
        articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(xform_prim)
        articulation_api.CreateSleepThresholdAttr(0.0)
        articulation_api.CreateEnabledSelfCollisionsAttr().Set(False)

        # add fixed root link:
        link_position = Gf.Vec3f(0)
        physicsUtils.add_rigid_sphere(self.stage, root_link_path, root_radius, link_position, color=root_link_color)
        joint = UsdPhysics.FixedJoint.Define(self.stage, root_link_path.AppendChild("FixedJoint"))
        joint.CreateBody1Rel().SetTargets([root_link_path])

        # child link
        link_position += Gf.Vec3f(link_length/2, 0, 0)
        # physicsUtils.add_rigid_box(self.stage, child_link_path, link_size, link_position, color=child_link_color)
        physicsUtils.add_rigid_capsule(self.stage, child_link_path, link_height, link_length, "X", link_position, color=child_link_color)
        physicsUtils.add_mass(self.stage, child_link_path, link_mass)
        if joint_type == "D6":
            rotational_joint = UsdPhysics.Joint.Define(self.stage, child_link_path.AppendChild("D6Joint"))
        else:
            for k,val in usd_d6_free_rot_dof.items():
                joint_axis = k[-1] if val else None

            if joint_type == "Spherical":
                rotational_joint = UsdPhysics.SphericalJoint.Define(self.stage, child_link_path.AppendChild("SphericalJoint"))
                # sphericalJoint.CreateAxisAttr(joint_axis)
            elif joint_type == "Revolute":
                rotational_joint = UsdPhysics.RevoluteJoint.Define(self.stage, child_link_path.AppendChild("RevoluteJoint"))
                rotational_joint.CreateAxisAttr(joint_axis)

        rotational_joint.CreateBody0Rel().SetTargets([root_link_path])
        rotational_joint.CreateBody1Rel().SetTargets([child_link_path])

        rotational_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0))
        rotational_joint.CreateLocalRot0Attr().Set(rotational_joint_frame_quat)
        rotational_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(-0.5, 0, 0))  # link size is achieved via scale
        rotational_joint.CreateLocalRot1Attr().Set(rotational_joint_frame_quat)
    
        joint_Prim = rotational_joint.GetPrim()
        
        if joint_type == "D6":
            locked_dofs = ["transX", "transY", "transZ"]
            for axis in usd_d6_free_rot_dof:
                if not usd_d6_free_rot_dof[axis]:
                    locked_dofs.append(axis)
        
            for limit_name in locked_dofs:
                limit_api = UsdPhysics.LimitAPI.Apply(joint_Prim, limit_name)
                limit_api.CreateLowAttr(1.0)
                limit_api.CreateHighAttr(-1.0)

        if add_fixed_joint_link:
            link_position += Gf.Vec3f(link_length, 0, 0)
            # physicsUtils.add_rigid_box(self.stage, fixed_joint_link_path, link_size, link_position, color=fixed_joint_link_color)
            physicsUtils.add_rigid_capsule(self.stage, fixed_joint_link_path, link_height, link_length, "X", link_position, color=fixed_joint_link_color)
            physicsUtils.add_mass(self.stage, fixed_joint_link_path, link_mass)
            joint = UsdPhysics.FixedJoint.Define(self.stage, fixed_joint_link_path.AppendChild("FixedJoint"))
            joint.CreateBody0Rel().SetTargets([child_link_path])
            joint.CreateBody1Rel().SetTargets([fixed_joint_link_path])
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.5, 0, 0))
            joint.CreateLocalRot0Attr().Set(fixed_joint_frame_quat)
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(-0.5, 0, 0))  # link size is achieved via scale
            joint.CreateLocalRot1Attr().Set(fixed_joint_frame_quat)
        else:
            fixed_joint_link_path = Sdf.Path()

        return dict({"root": root_link_path, "child": child_link_path, "fixed": fixed_joint_link_path})


    def create_rigid_box(self, actor_path, transform, scale=Gf.Vec3f(1.0, 1.0, 1.0)):
        box = UsdGeom.Cube.Define(self.stage, actor_path)
        box.CreateSizeAttr(1.0)
        box.AddTranslateOp().Set(transform.p)
        box.AddOrientOp().Set(transform.q)
        box.AddScaleOp().Set(scale)
        box.CreateDisplayColorAttr().Set([Gf.Vec3f(71.0 / 255.0, 105.0 / 255.0, 1.0)])

        prim = box.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim)

        return prim

    def create_sdf_object(self, actor_path, transform, length):
        # sphere = UsdGeom.Sphere.Define(self.stage, actor_path)
        # sphere.CreateRadiusAttr(radius)
        sphere = physicsUtils.create_mesh_cube(self.stage, actor_path, length)
        sphere.AddTranslateOp().Set(transform.p)
        sphere.AddOrientOp().Set(transform.q)
        # sphere.CreateDisplayColorAttr().Set([Gf.Vec3f(71.0 / 255.0, 105.0 / 255.0, 1.0)])
        prim = sphere.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        meshcollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(prim)
        sdf_api = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
        meshcollisionAPI.CreateApproximationAttr().Set("sdf")
        return prim

    def create_particle_system(self, particle_system_path, rest_offset, contact_offset, solid_rest_offset, fluid_rest_offset, particle_contact_offset):

        particleUtils.add_physx_particle_system(
            stage=self.stage,
            particle_system_path=particle_system_path,
            contact_offset=contact_offset,
            rest_offset=rest_offset,
            particle_contact_offset=particle_contact_offset,
            solid_rest_offset=solid_rest_offset,
            fluid_rest_offset=fluid_rest_offset,
            solver_position_iterations=16,
            simulation_owner=self.scene_path,
        )

    def create_particle_cloth(self, cloth_path="/cloth", create_particle_system=True, stiffness=None, damping=0.2, dimX=5, dimY=5, rotate=False):
        plane_mesh = UsdGeom.Mesh.Define(self.stage, cloth_path)
        tri_points, tri_indices = deformableUtils.create_triangle_mesh_square(dimx=dimX, dimy=dimY, scale=0.6)
        plane_mesh.GetPointsAttr().Set(tri_points)
        plane_mesh.GetFaceVertexIndicesAttr().Set(tri_indices)
        plane_mesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))

        physicsUtils.setup_transform_as_scale_orient_translate(plane_mesh)
        physicsUtils.set_or_add_translate_op(plane_mesh, Gf.Vec3f(0.0, 0.0, 0.5))

        #if rotate is true, rotate 90 degrees.
        if rotate:
            physicsUtils.set_or_add_orient_op(plane_mesh, Gf.Quatf(0.707, 0.707, 0, 0))

        particle_system_path = plane_mesh.GetPrim().GetParent().GetPath().AppendChild("particleSystem")

        if create_particle_system:
            # size rest offset according to plane resolution and width so that particles are just touching at rest
            radius = 0.5 * (0.6 / 5.0)
            restOffset = radius
            contactOffset = restOffset * 1.5

            self.create_particle_system(
                particle_system_path=particle_system_path,
                rest_offset=restOffset,
                contact_offset=contactOffset,
                solid_rest_offset=restOffset,
                fluid_rest_offset=restOffset,
                particle_contact_offset=contactOffset)

        # create material and assign it to the system:
        particle_material_path = plane_mesh.GetPrim().GetParent().GetPath().AppendChild("particleMaterial")
        particleUtils.add_pbd_particle_material(self.stage, particle_material_path)
        # add some drag and lift to get aerodynamic effects
        particleUtils.add_pbd_particle_material(self.stage, particle_material_path, drag=0.0, lift=0.0, friction=0.6)
        physicsUtils.add_physics_material_to_prim(
            self.stage, self.stage.GetPrimAtPath(particle_system_path), particle_material_path
        )

        # configure as cloth
        if stiffness == None:
            stretchStiffness = 10000.0
            bendStiffness = 200.0
            shearStiffness = 100.0
        else:
            stretchStiffness = stiffness
            bendStiffness = stiffness
            shearStiffness = stiffness

        particleUtils.add_physx_particle_cloth(
            stage=self.stage,
            path=cloth_path,
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
        num_verts = len(plane_mesh.GetPointsAttr().Get())
        mass = particle_mass * num_verts
        massApi = UsdPhysics.MassAPI.Apply(plane_mesh.GetPrim())
        massApi.GetMassAttr().Set(mass)

        return tri_points


    def create_soft_body(self, soft_body_path, resolution = 1):
        stage=self.stage
        mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(stage, soft_body_path , True))
        skin_mesh = UsdGeom.Mesh.Define(stage, mesh_path)
        tri_points, tri_indices = deformableMeshUtils.createTriangleMeshCube(8)
        skin_mesh.GetPointsAttr().Set(tri_points)
        skin_mesh.GetFaceVertexIndicesAttr().Set(tri_indices)
        skin_mesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))
        skin_mesh.GetSubdivisionSchemeAttr().Set("none")

        skin_mesh.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 1.0))
        # skin_mesh.AddOrientOp().Set(Gf.Quatf(0.707, 0.707, 0, 0))
        # skin_mesh.AddScaleOp().Set(Gf.Vec3f(.5, .5, 0.5))
        points=skin_mesh.GetPointsAttr().Get()
        # Create tet meshes for simulation and collision based on the skin mesh
        simulation_resolution = resolution
        # Apply PhysxDeformableBodyAPI and PhysxCollisionAPI to skin mesh and set parameter to default values
        success = deformableUtils.add_physx_deformable_body(
            stage,
            skin_mesh.GetPath(),
            collision_simplification=True,
            simulation_hexahedral_resolution=simulation_resolution,
            self_collision=True,
        )
        # Create a deformable body material and set it on the deformable body
        deformable_material_path = skin_mesh.GetPrim().GetParent().GetPath().AppendChild("deformableBodyMaterial")
        # deformable_material_path = omni.usd.get_stage_next_free_path(stage, "/deformableBodyMaterial", True)
        deformableUtils.add_deformable_body_material(
            stage,
            deformable_material_path,
            youngs_modulus=20000.0,
            poissons_ratio=0.49,
            damping_scale=0.5,
            dynamic_friction=0.5,
        )
        physicsUtils.add_physics_material_to_prim(stage, skin_mesh.GetPrim(), deformable_material_path)
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))


    @staticmethod
    def create_transform(translate = Gf.Vec3d(0.0),
                         rotate = Gf.Rotation(Gf.Quatd(1.0)),
                         scale = Gf.Vec3d(1.0),
                         pivot_pos = Gf.Vec3d(0.0),
                         pivot_orient = Gf.Rotation(Gf.Quatd(1.0))):
        return Gf.Transform(translate, rotate, scale, pivot_pos, pivot_orient)

    @staticmethod
    def check_tet_element_indices(
        stage,
        tet_mesh_paths: List[str],
        tet_elem_indices: np.ndarray
    ) -> bool:

        if len(tet_mesh_paths) != tet_elem_indices.shape[0]:
            return False

        for env, path in enumerate(tet_mesh_paths):
            # reference tets from USD
            prim = stage.GetPrimAtPath(path)
            tet_mesh = UsdGeom.TetMesh(prim)
            usd_tets = np.array(tet_mesh.GetTetVertexIndicesAttr().Get(), dtype=np.int64)
            usd_num_tets = usd_tets.shape[0]
            # tensor tets from tensor view (drop padding)
            # TODO: this is currently done with usd_num_tets, is there some fixed sentinel convention?
            tensor_tets = tet_elem_indices[env, :usd_num_tets]
            tensor_num_tets = tensor_tets.shape[0]
            # assuming same order
            if tensor_num_tets != usd_num_tets or not np.array_equal(tensor_tets, usd_tets):
                return False

        return True

    @staticmethod
    def check_tri_element_indices(
        stage,
        tri_mesh_paths: List[str],
        tri_elem_indices: np.ndarray
    ) -> bool:

        if len(tri_mesh_paths) != tri_elem_indices.shape[0]:
            return False

        for env, path in enumerate(tri_mesh_paths):
            # reference tets from USD
            prim = stage.GetPrimAtPath(path)
            tri_mesh = UsdGeom.Mesh(prim)
            usd_tris = np.array(tri_mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int64).reshape(-1, 3)          
            usd_num_tris = usd_tris.shape[0]
            # tensor tris from tensor view (drop padding)
            # TODO: this is currently done with usd_num_tris, is there some fixed sentinel convention?
            tensor_tris = tri_elem_indices[env, :usd_num_tris]
            tensor_num_tris = tensor_tris.shape[0]
            # assuming same order
            if tensor_num_tris != usd_num_tris or not np.array_equal(tensor_tris, usd_tris):
                return False

        return True

    def create_volume_deformable_body(self,
        volume_deformable_body_path,
        translate=Gf.Vec3d(0.0)
    ):
        stage=self.stage
        deformable_path = Sdf.Path(omni.usd.get_stage_next_free_path(stage, volume_deformable_body_path , True))
        xform = UsdGeom.Xform.Define(stage, deformable_path)
        xform.AddTransformOp().Set(self.create_transform(translate=translate).GetMatrix())
        mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(stage, str(xform.GetPath()) + "/mesh", True))
        sim_mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(stage, str(xform.GetPath()) + "/sim_mesh", True))
        coll_mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(stage, str(xform.GetPath()) + "/coll_mesh", True))

        skin_mesh = UsdGeom.Mesh.Define(stage, mesh_path)
        tri_points, tri_indices = deformableMeshUtils.createTriangleMeshCube(8)
        skin_mesh.GetPointsAttr().Set(tri_points)
        skin_mesh.GetFaceVertexIndicesAttr().Set(tri_indices)
        skin_mesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))
        skin_mesh.GetSubdivisionSchemeAttr().Set("none")
        skin_mesh.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 1.0))

        success = deformableUtils.create_auto_volume_deformable_hierarchy(stage,
            root_prim_path = deformable_path,
            simulation_tetmesh_path = sim_mesh_path,
            collision_tetmesh_path = coll_mesh_path,
            cooking_src_mesh_path = mesh_path,
            simulation_hex_mesh_enabled = True,
            cooking_src_simplification_enabled = True,
            set_visibility_with_guide_purpose = True
        )
        get_physx_cooking_interface().cook_auto_deformable_body(str(deformable_path))

        # Create a deformable body material and set it on the deformable body
        deformable_material_path = xform.GetPrim().GetParent().GetPath().AppendChild("volumeDeformableMaterial")
        success = deformableUtils.add_deformable_material(stage, deformable_material_path, dynamic_friction=0.5, youngs_modulus=500000.0, poissons_ratio=0.49)
        physicsUtils.add_physics_material_to_prim(stage, stage.GetPrimAtPath(deformable_path), deformable_material_path)

        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    @staticmethod
    def createTriangleMeshQuad(n: int):
        step = 1.0 / n
        tri_points = [
            Gf.Vec3f(i * step, j * step, 0.0) - Gf.Vec3f(0.5, 0.5, 0.0)
            for j in range(n + 1)
            for i in range(n + 1)
        ]
        row_stride = n + 1
        tri_indices = []
        for j in range(n):
            for i in range(n):
                v0 =  j * row_stride + i
                v1 =  v0 + 1
                v2 = (j+1) * row_stride + i
                v3 =  v2 + 1
                tri_indices.extend([v0, v1, v3])
                tri_indices.extend([v0, v3, v2])
        return tri_points, tri_indices

    def create_surface_deformable_body(self, 
        surface_deformable_body_path,
        resolution=10,
        translate=Gf.Vec3d(0.0, 0.0, 1.0),
        rotate=Gf.Rotation(Gf.Quatd(1.0)),
        attach=False
    ):
        stage=self.stage
        deformable_path = Sdf.Path(omni.usd.get_stage_next_free_path(stage, surface_deformable_body_path , True))

        xform = UsdGeom.Xform.Define(stage, deformable_path)
        xform.AddTransformOp().Set(self.create_transform(translate=translate, rotate=rotate).GetMatrix())
        mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(stage, str(xform.GetPath()) + "/mesh", True))
        sim_mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(stage, str(xform.GetPath()) + "/sim_mesh", True))

        skin_mesh = UsdGeom.Mesh.Define(stage, mesh_path)
        tri_points, tri_indices = self.createTriangleMeshQuad(resolution)
        skin_mesh.GetPointsAttr().Set(tri_points)
        skin_mesh.GetFaceVertexIndicesAttr().Set(tri_indices)
        skin_mesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))
        skin_mesh.GetSubdivisionSchemeAttr().Set("none")

        success = deformableUtils.create_auto_surface_deformable_hierarchy(stage,
            root_prim_path = deformable_path,
            simulation_mesh_path = sim_mesh_path,
            cooking_src_mesh_path = mesh_path,
            cooking_src_simplification_enabled = False,
            set_visibility_with_guide_purpose = True
        )
        get_physx_cooking_interface().cook_auto_deformable_body(str(deformable_path))

        if attach:
            attachment_path = xform.GetPrim().GetPath().AppendChild("attachment")
            attachment_prim = stage.DefinePrim(attachment_path, "OmniPhysicsVtxXformAttachment")
            attachment_prim.GetRelationship("omniphysics:src0").SetTargets([sim_mesh_path])
            attachment_prim.GetRelationship("omniphysics:src1").SetTargets([xform.GetPrim().GetParent().GetPath()])
            vtx_indices = list(range(resolution+1))
            skin_to_world = skin_mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            env_to_world = UsdGeom.Xformable(xform.GetPrim().GetParent()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            skin_to_env = skin_to_world * env_to_world.GetInverse()
            vtx_positions = [Gf.Vec3f(skin_to_env.Transform(x)) for x in tri_points[:resolution+1]]
            attachment_prim.GetAttribute("omniphysics:vtxIndicesSrc0").Set(vtx_indices)
            attachment_prim.GetAttribute("omniphysics:localPositionsSrc1").Set(vtx_positions)

        # Create a deformable body material and set it on the deformable body
        deformable_material_path = xform.GetPrim().GetParent().GetPath().AppendChild("surfaceDeformableMaterial")
        success = deformableUtils.add_surface_deformable_material(stage, deformable_material_path, dynamic_friction=0.5, youngs_modulus=500000.0, poissons_ratio=0.49)
        physicsUtils.add_physics_material_to_prim(stage, stage.GetPrimAtPath(deformable_path), deformable_material_path)

    def create_actor_from_asset(self, actor_path, transform, asset_path, prim_path=Sdf.Path()):
        prim = self.stage.OverridePrim(actor_path)
        prim.GetReferences().AddReference(asset_path, prim_path)

        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp().Set(transform.p)
        xf.AddOrientOp().Set(transform.q)

        return prim

    def create_distant_light(self, dir=Gf.Vec3f(-2.0, -1.0, -4.0), angle=10.0, intensity=500.0):
        light = UsdLux.DistantLight.Define(self.stage, Sdf.Path("/dirLight"))

        # compute quaternion from given direction vector
        down_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        dir = dir.GetNormalized()
        dot = Gf.Dot(down_dir, dir)
        if dot > 0.9999:
            q = Gf.Quatf.GetIdentity()
        elif dot < -0.9999:
            q = Gf.Quatf(0.0, 0.0, 1.0, 0.0)
        else:
            v = Gf.Cross(down_dir, dir)
            w = 1.0 + Gf.Dot(down_dir, dir)
            q = Gf.Quatf(w, v[0], v[1], v[2])
            q.Normalize()

        light.CreateAngleAttr(angle)
        light.CreateIntensityAttr(intensity)
        light.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 20.0))
        light.AddOrientOp().Set(q)

        return light.GetPrim()

    def finish(self):
        self.finished = True

    def fail(self):
        self.finished = True
        self.failed = True

    # subclasses can override the conditions
    def should_quit(self, stepno):
        if self.failed:
            return True

        if self.maxsteps is not None and stepno >= self.maxsteps:
            return True

        if self.finished:
            if self.minsteps is None or stepno >= self.minsteps:
                return True

        return False

    @abstractmethod
    def start(self, frontend="warp", stage_id=-1):
        """Called when simulation starts"""

    @abstractmethod
    def physics_step(self, stepno, dt):
        """Called after every physics simulation step"""


class GridScenarioBase(ScenarioBase):
    def __init__(self, grid_params: GridParams, sim_params: SimParams, device_params: DeviceParams):

        super().__init__(sim_params, device_params)
        stage = self.stage

        self.env_spacing = grid_params.env_spacing
        self.num_envs = max(grid_params.num_envs, 1)
        if type(grid_params.num_rows) is int:
            self.num_rows = max(min(grid_params.num_rows, self.num_envs), 1)
        else:
            self.num_rows = int(math.ceil(math.sqrt(self.num_envs)))
        self.num_cols = int(math.ceil(float(self.num_envs) / float(self.num_rows)))

        # create blank env template
        env_template_path = Sdf.Path("/envTemplate")
        env_template_xform = UsdGeom.Xform.Define(stage, env_template_path)
        env_template_xform.GetPrim().SetSpecifier(Sdf.SpecifierClass)

        env_scope_path = Sdf.Path("/envs")
        env_scope = UsdGeom.Scope.Define(stage, env_scope_path)

        if grid_params.row_spacing is not None:
            self.row_spacing = grid_params.row_spacing
        else:
            self.row_spacing = grid_params.env_spacing
        if grid_params.col_spacing is not None:
            self.col_spacing = grid_params.col_spacing
        else:
            self.col_spacing = grid_params.env_spacing

        env_xforms = []

        # create envs
        row_offset = 0.5 * self.row_spacing * (self.num_rows - 1)
        col_offset = 0.5 * self.col_spacing * (self.num_cols - 1)
        for i in range(self.num_envs):
            row = i // self.num_cols
            col = i % self.num_cols
            x = row_offset - row * self.row_spacing
            y = col * self.col_spacing - col_offset

            env_origin = Gf.Vec3f(x, y, 0.0)

            env_path = env_scope_path.AppendChild("env" + str(i))
            env_xform = UsdGeom.Xform.Define(stage, env_path)
            env_xform.GetPrim().GetInherits().AddInherit(env_template_path)
            env_xform.AddTranslateOp().Set(env_origin)

            env_xforms.append(env_xform)

        # specify initial camera transform
        avg_spacing = 0.5 * (self.row_spacing + self.col_spacing)
        cam_x = row_offset + 3 * avg_spacing
        cam_y = 0.0
        cam_z = 2.0 * avg_spacing

        custom_layer_data = {
            "cameraSettings": {
                "Perspective": {
                    "position": Gf.Vec3d(cam_x, cam_y, cam_z),
                    "target": Gf.Vec3d(0, 0, 0)
                }
            }
        }
        root_layer = stage.GetRootLayer()
        root_layer.customLayerData = custom_layer_data

        self.grid_params = grid_params

        self.env_template_path = env_template_path
        self.env_xforms = env_xforms


def set_drive(joint_prim, drive_type, target_type, target_value, stiffness, damping, max_force):
    if not joint_prim:
        raise Exception("Invalid joint prim")

    # set drive type ("angular" or "linear")
    drive = UsdPhysics.DriveAPI.Apply(joint_prim, drive_type)

    # set target type ("position" or "velocity")
    if target_type == "position":
        if not drive.GetTargetPositionAttr():
            drive.CreateTargetPositionAttr(target_value)
        else:
            drive.GetTargetPositionAttr().Set(target_value)
    elif target_type == "velocity":
        if not drive.GetTargetVelocityAttr():
            drive.CreateTargetVelocityAttr(target_value)
        else:
            drive.GetTargetVelocityAttr().Set(target_value)
    else:
        raise Exception("Invalid joint drive target type ({})".format(target_type))

    if not drive.GetStiffnessAttr():
        drive.CreateStiffnessAttr(stiffness)
    else:
        drive.GetStiffnessAttr().Set(stiffness)

    if not drive.GetDampingAttr():
        drive.CreateDampingAttr(damping)
    else:
        drive.GetDampingAttr().Set(damping)

    if not drive.GetMaxForceAttr():
        drive.CreateMaxForceAttr(max_force)
    else:
        drive.GetMaxForceAttr().Set(max_force)


class Runner(ABC):

    def __init__(self, scenario, frontend, sync_params, fast_step=False):
        self.scenario = scenario
        self.frontend = frontend
        self.sync_params = sync_params
        self.fast_step = fast_step

        stage = self.scenario.stage
        self.stage_id = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()

        import omni.physx
        self.iphysx = omni.physx.get_physx_interface()
        self.iphysx_sim = omni.physx.get_physx_simulation_interface()
        if self.sync_params.sync_fabric:
            # NOTE: omni.physx.fabric has to be enabled, otherwise this import will fail
            import omni.physxfabric
            self.iphysx_fc = omni.physxfabric.get_physx_fabric_interface()


    def push_preference_settings(self):
        #print("APPLING PREFERENCES")
        settings = carb.settings.acquire_settings_interface()
        self.saved_use_active_cuda_context = settings.get_as_bool(physx_bindings.SETTING_USE_ACTIVE_CUDA_CONTEXT)
        settings.set(physx_bindings.SETTING_USE_ACTIVE_CUDA_CONTEXT, False)
        
        self.saved_cuda_device = settings.get_as_int(physx_bindings.SETTING_CUDA_DEVICE)
        settings.set(physx_bindings.SETTING_CUDA_DEVICE, 0)
        self.saved_reset_on_stop = settings.get_as_bool(physx_bindings.SETTING_RESET_ON_STOP)
        settings.set(physx_bindings.SETTING_RESET_ON_STOP, False)
        self.saved_display_joints = settings.get_as_bool(physx_bindings.SETTING_DISPLAY_JOINTS)
        settings.set(physx_bindings.SETTING_DISPLAY_JOINTS, False)

    def pop_preference_settings(self):
        #print("RESTORING PREFERENCES")
        settings = carb.settings.acquire_settings_interface()
        if hasattr(self, "saved_use_active_cuda_context"):
            settings.set(physx_bindings.SETTING_USE_ACTIVE_CUDA_CONTEXT, self.saved_use_active_cuda_context)
        if hasattr(self, "saved_reset_on_stop"):
            settings.set(physx_bindings.SETTING_RESET_ON_STOP, self.saved_reset_on_stop)
        if hasattr(self, "saved_display_joints"):
            settings.set(physx_bindings.SETTING_DISPLAY_JOINTS, self.saved_display_joints)

    def push_stage_setup_settings(self, device_params):
        #print("APPLYING STAGE SETUP SETTINGS")
        settings = carb.settings.acquire_settings_interface()
        #print("++++++ Setting SUPPRESS_READBACK", use_gpu_pipeline)
        self.saved_suppress_readback = settings.get_as_bool(physx_bindings.SETTING_SUPPRESS_READBACK)
        settings.set(physx_bindings.SETTING_SUPPRESS_READBACK, device_params.use_gpu_pipeline)
        self.saved_num_threads = settings.get_as_int(physx_bindings.SETTING_NUM_THREADS)
        if isinstance(device_params.num_workers, int):
            settings.set(physx_bindings.SETTING_NUM_THREADS, device_params.num_workers)

    def pop_stage_setup_settings(self):
        #print("RESTORING STAGE SETUP SETTINGS")
        settings = carb.settings.acquire_settings_interface()
        if hasattr(self, "saved_suppress_readback"):
            settings.set(physx_bindings.SETTING_SUPPRESS_READBACK, self.saved_suppress_readback)
        if hasattr(self, "saved_num_threads"):
            settings.set(physx_bindings.SETTING_NUM_THREADS, self.saved_num_threads)

    def push_stage_runtime_settings(self, sim_params, sync_params):
        #print("APPLYING STAGE RUNTIME SETTINGS")
        settings = carb.settings.acquire_settings_interface()

        self.saved_min_frame_rate = settings.get_as_int(physx_bindings.SETTING_MIN_FRAME_RATE)
        settings.set(physx_bindings.SETTING_MIN_FRAME_RATE, sim_params.min_frame_rate)

        # USD sync settings
        self.saved_update_to_usd = settings.get_as_bool(physx_bindings.SETTING_UPDATE_TO_USD)
        self.saved_update_velocities_to_usd = settings.get_as_bool(physx_bindings.SETTING_UPDATE_VELOCITIES_TO_USD)
        if sync_params.sync_usd:
            settings.set(physx_bindings.SETTING_UPDATE_TO_USD, True)
            settings.set(physx_bindings.SETTING_UPDATE_VELOCITIES_TO_USD, not sync_params.transforms_only)
        else:
            settings.set(physx_bindings.SETTING_UPDATE_TO_USD, False)
            settings.set(physx_bindings.SETTING_UPDATE_VELOCITIES_TO_USD, False)

        # Fabric sync settings
        self.saved_fabric_enabled = settings.get_as_bool("/physics/fabricEnabled")
        self.saved_fabric_update_transformations = settings.get_as_bool("/physics/fabricUpdateTransformations")
        self.saved_fabric_update_velocities = settings.get_as_bool("/physics/fabricUpdateVelocities")
        self.saved_fabric_update_force_sensors = settings.get_as_bool("/physics/fabricUpdateForceSensors")
        if sync_params.sync_fabric:
            settings.set("/physics/fabricEnabled", True)
            settings.set("/physics/fabricUpdateTransformations", True)
            settings.set("/physics/fabricUpdateVelocities", not sync_params.transforms_only)
            settings.set("/physics/fabricUpdateForceSensors", not sync_params.transforms_only)
        else:
            settings.set("/physics/fabricEnabled", False)
            settings.set("/physics/fabricUpdateTransformations", False)
            settings.set("/physics/fabricUpdateVelocities", False)
            settings.set("/physics/fabricUpdateForceSensors", False)

    def pop_stage_runtime_settings(self):
        #print("RESTORING STAGE RUNTIME SETTINGS")
        settings = carb.settings.acquire_settings_interface()

        if hasattr(self, "saved_min_frame_rate"):
            settings.set(physx_bindings.SETTING_MIN_FRAME_RATE, self.saved_min_frame_rate)

        # USD sync settings
        if hasattr(self, "saved_update_to_usd"):
            settings.set(physx_bindings.SETTING_UPDATE_TO_USD, self.saved_update_to_usd)
        if hasattr(self, "saved_update_velocities_to_usd"):
            settings.set(physx_bindings.SETTING_UPDATE_VELOCITIES_TO_USD, self.saved_update_velocities_to_usd)

        # Fabric sync settings
        if hasattr(self, "saved_fabric_enabled"):
            settings.set("/physics/fabricEnabled", self.saved_fabric_enabled)
        if hasattr(self, "saved_fabric_update_transformations"):
            settings.set("/physics/fabricUpdateTransformations", self.saved_fabric_update_transformations)
        if hasattr(self, "saved_num_threads"):
            settings.set("/physics/saved_fabric_update_velocities", self.saved_fabric_update_velocities)
        if hasattr(self, "saved_num_threads"):
            settings.set("/physics/saved_fabric_update_force_sensors", self.saved_fabric_update_force_sensors)


    @abstractmethod
    def start(self, warm_start):
        """Start scenario"""

    @abstractmethod
    def simulate(self):
        """Simulate scenario for a number of steps"""

    @abstractmethod
    def stop(self):
        """Start scenario"""

    # shortcut to start, simulate, and stop
    def run(self, warm_start):
        self.start(warm_start)
        self.simulate()
        self.stop()


class RunnerInContext(Runner):

    def __init__(self, scenario, frontend, sync_params, **kwargs):
        super().__init__(scenario, frontend, sync_params, **kwargs)

    def start(self, warm_start):
        #print("STAAAAAAAAAART RunnerInContext")

        self.push_preference_settings()

        # attach to USD context
        omni.usd.get_context().attach_stage_with_callback(self.stage_id)

        # apply stage settings
        omni.kit.app.get_app().update()
        self.push_stage_setup_settings(self.scenario.device_params)
        omni.kit.app.get_app().update()

        # Note: make PhysX use device 0 for compatibility with Warp
        #set_physx_device_ordinal_hint(0)

        # attach physics directly, disconnect from stage update
        # Note: we do this with the Warp context active and tell PhysX to use it ('/persistent/physics/useActiveCudaContext')
        with wp.ScopedDevice(self.scenario.wp_device):
            self.iphysx_sim.attach_stage(self.stage_id)

        if self.sync_params.sync_fabric:
            self.iphysx_fc.attach_stage(self.stage_id)

        omni.kit.app.get_app().update()

        # stage-specific runtime settings
        self.push_stage_runtime_settings(self.scenario.sim_params, self.sync_params)

        omni.kit.app.get_app().update()

        self.dt = 1.0 / self.scenario.sim_params.time_steps_per_second

        if warm_start:
            # run a simulation step
            self.iphysx_sim.simulate(self.dt, 0.0)
            self.iphysx_sim.fetch_results()

            if self.sync_params.sync_fabric:
                self.iphysx_fc.update(0.0, self.dt)

        # start the scenario
        self.scenario.start(self.frontend, self.stage_id)

    def simulate(self):
        #print("RUUUUUUUUUUUUN RunnerInContext")

        stepno = 0

        while not self.scenario.should_quit(stepno):
            #print("STEPPING", stepno)

            omni.kit.app.get_app().update()

            if self.fast_step:
                self.scenario.sim.step(self.dt)
            else:
                self.iphysx_sim.simulate(self.dt, stepno * self.dt)
                self.iphysx_sim.fetch_results()
            stepno += 1

            if self.sync_params.sync_fabric:
                self.iphysx_fc.update(stepno * self.dt, self.dt)

            self.scenario.physics_step(stepno, self.dt)

    def stop(self):
        omni.physics.tensors.reset()

        self.iphysx_sim.detach_stage()

        if self.sync_params.sync_fabric:
            self.iphysx_fc.detach_stage()

        # restore stage settings
        omni.kit.app.get_app().update()
        self.pop_stage_runtime_settings()
        self.pop_stage_setup_settings()
        self.pop_preference_settings()
        omni.kit.app.get_app().update()


class RunnerInContextAsync(Runner):

    def __init__(self, scenario, frontend, sync_params, **kwargs):
        super().__init__(scenario, frontend, sync_params, **kwargs)

    async def start(self, warm_start):
        #print("STAAAAAAAAAART RunnerInContextAsync")

        self.push_preference_settings()

        # attach to USD context
        await omni.usd.get_context().attach_stage_async(self.scenario.stage)

        # apply stage settings
        await omni.kit.app.get_app().next_update_async()
        self.push_stage_setup_settings(self.scenario.device_params)
        await omni.kit.app.get_app().next_update_async()

        # Note: make PhysX use device 0 for compatibility with Warp
        #set_physx_device_ordinal_hint(0)

        # attach physics directly, disconnect from stage update
        # Note: we do this with the Warp context active and tell PhysX to use it ('/persistent/physics/useActiveCudaContext')
        with wp.ScopedDevice(self.scenario.wp_device):
            self.iphysx_sim.attach_stage(self.stage_id)

        if self.sync_params.sync_fabric:
            self.iphysx_fc.attach_stage(self.stage_id)

        await omni.kit.app.get_app().next_update_async()

        # stage-specific runtime settings
        self.push_stage_runtime_settings(self.scenario.sim_params, self.sync_params)

        await omni.kit.app.get_app().next_update_async()

        self.dt = 1.0 / self.scenario.sim_params.time_steps_per_second

        if warm_start:
            # run a simulation step
            self.iphysx_sim.simulate(self.dt, 0.0)
            self.iphysx_sim.fetch_results()

            if self.sync_params.sync_fabric:
                self.iphysx_fc.update(0.0, self.dt)

        # start the scenario
        self.scenario.start(self.frontend, self.stage_id)

    async def simulate(self):
        #print("RUUUUUUUUUUUUN RunnerInContextAsync")

        stepno = 0

        # run the minimum amount of steps
        while not self.scenario.should_quit(stepno):
            #print("STEPPING", stepno)

            await omni.kit.app.get_app().next_update_async()

            if self.fast_step:
                self.scenario.sim.step(self.dt)
            else:
                self.iphysx_sim.simulate(self.dt, stepno * self.dt)
                self.iphysx_sim.fetch_results()
            stepno += 1

            if self.sync_params.sync_fabric:
                self.iphysx_fc.update(stepno * self.dt, self.dt)

            self.scenario.physics_step(stepno, self.dt)

    async def stop(self):
        omni.physics.tensors.reset()

        self.iphysx_sim.detach_stage()

        if self.sync_params.sync_fabric:
            self.iphysx_fc.detach_stage()

        # restore stage settings
        await omni.kit.app.get_app().next_update_async()
        self.pop_stage_runtime_settings()
        self.pop_stage_setup_settings()
        self.pop_preference_settings()
        await omni.kit.app.get_app().next_update_async()

    async def run(self, warm_start):
        await self.start(warm_start)
        await self.simulate()
        await self.stop()


class RunnerInMemory(Runner):
    
    def __init__(self, scenario, frontend, sync_params, **kwargs):
        super().__init__(scenario, frontend, sync_params, **kwargs)

    def start(self, warm_start):
        #print("STAAAAAAAAAART RunnerInMemory")

        self.push_preference_settings()

        # settings needed before physics scene creation
        self.push_stage_setup_settings(self.scenario.device_params)

        # Note: make PhysX use device 0 for compatibility with Warp
        #set_physx_device_ordinal_hint(0)

        # attach physics to stage in memory
        # Note: we do this with the Warp context active and tell PhysX to use it ('/persistent/physics/useActiveCudaContext')
        with wp.ScopedDevice(self.scenario.wp_device):
            self.iphysx_sim.attach_stage(self.stage_id)

        if self.sync_params.sync_fabric:
            self.iphysx_fc.attach_stage(self.stage_id)

        # stage-specific runtime settings
        self.push_stage_runtime_settings(self.scenario.sim_params, self.sync_params)

        self.dt = 1.0 / self.scenario.sim_params.time_steps_per_second

        if warm_start:
            self.iphysx_sim.simulate(self.dt, 0.0)
            self.iphysx_sim.fetch_results()

            if self.sync_params.sync_fabric:
                self.iphysx_fc.update(0.0, self.dt)

        self.scenario.start(self.frontend, self.stage_id)

    def simulate(self):
        #print("RUUUUUUUUUUUUN RunnerInMemory")

        stepno = 0

        while not self.scenario.should_quit(stepno):
            if self.fast_step:
                self.scenario.sim.step(self.dt)
            else:
                self.iphysx_sim.simulate(self.dt, stepno * self.dt)
                self.iphysx_sim.fetch_results()
            stepno += 1

            if self.sync_params.sync_fabric:
                self.iphysx_fc.update(stepno * self.dt, self.dt)

            self.scenario.physics_step(stepno, self.dt)

    def stop(self):
        omni.physics.tensors.reset()

        self.iphysx_sim.detach_stage()

        if self.sync_params.sync_fabric:
            self.iphysx_fc.detach_stage()

        # restore settings
        self.pop_stage_runtime_settings()
        self.pop_stage_setup_settings()
        self.pop_preference_settings()
