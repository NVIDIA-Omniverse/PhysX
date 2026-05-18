# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import omni.physx.bindings._physx as pxb
from pxr import UsdPhysics, PhysxSchema, UsdGeom, UsdShade
from collections import namedtuple
import carb.settings
import omni.timeline
from ..utils import get_schema_name
from .schemas import attachment_apis

deformable_deprecated_enabled = carb.settings.get_settings().get(pxb.SETTING_ENABLE_DEFORMABLE_DEPRECATED)

Component = namedtuple('Component', "is_present, can_add, name, title, main_schema, add_component_fn, remove_component_fn, can_show")
Component.__new__.__defaults__ = (
    None,   # add_component_fn
    None,   # remove_component_fn
    None,   # can_show
)

components = {
    UsdPhysics.RigidBodyAPI.__name__:
    Component(
        lambda p: p._refresh_cache.has_rigidbody_api,
        lambda p: not p._refresh_cache.has_rigidbody_api,
        "PhysicsRigidBodyAPI", "Rigid Body", UsdPhysics.RigidBodyAPI,
        can_show=lambda p: p.IsA(UsdGeom.Xformable) and not p.IsA(PhysxSchema.PhysxParticleSystem) and not p._refresh_cache.hasconflictingapis_RigidBodyAPI,
    ),
    UsdPhysics.CollisionAPI.__name__:
    Component(
        lambda p: p._refresh_cache.has_collision_api,
        lambda p: not p._refresh_cache.has_collision_api,
        "PhysicsCollisionAPI", "Collider", UsdPhysics.CollisionAPI,
        can_show=lambda p: (p.IsA(UsdGeom.Gprim) or p.IsInstanceable()) and not p.IsA(PhysxSchema.PhysxParticleSystem) and not p._refresh_cache.hasconflictingapis_CollisionAPI,
    ),
    UsdPhysics.ArticulationRootAPI.__name__:
    Component(
        lambda p: p.HasAPI(UsdPhysics.ArticulationRootAPI),
        lambda p: p.IsA(UsdGeom.Imageable) and not p.IsA(PhysxSchema.PhysxParticleSystem) and not p._refresh_cache.hasconflictingapis_ArticulationRoot,
        "PhysicsArticulationRootAPI", "Articulation Root", UsdPhysics.ArticulationRootAPI
    ),
    "OmniPhysicsDeformableBodyAPI":
    Component(
        lambda p: p.HasAPI("OmniPhysicsDeformableBodyAPI"),
        lambda p: False,
        "OmniPhysicsDeformableBodyAPI", "Deformable Body", "OmniPhysicsDeformableBodyAPI"
    ),
    "OmniPhysicsVolumeDeformableSimAPI":
    Component(
        lambda p: p.HasAPI("OmniPhysicsVolumeDeformableSimAPI"),
        lambda p: False,
        "OmniPhysicsVolumeDeformableSimAPI", "Volume Deformable Simulation Mesh", "OmniPhysicsVolumeDeformableSimAPI"
    ),
    "OmniPhysicsSurfaceDeformableSimAPI":
    Component(
        lambda p: p.HasAPI("OmniPhysicsSurfaceDeformableSimAPI"),
        lambda p: False,
        "OmniPhysicsSurfaceDeformableSimAPI", "Surface Deformable Simulation Mesh", "OmniPhysicsSurfaceDeformableSimAPI"
    ),
    "OmniPhysicsDeformablePoseAPI":
    Component(
        lambda p: p.HasAPI("OmniPhysicsDeformablePoseAPI"),
        lambda p: False,
        "OmniPhysicsDeformablePoseAPI", "Deformable Pose", "OmniPhysicsDeformablePoseAPI"
    ),
    "PhysxAutoDeformableBodyAPI":
    Component(
        lambda p: p.HasAPI("PhysxAutoDeformableBodyAPI"),
        lambda p: False,
        "PhysxAutoDeformableBodyAPI", "Auto Deformable Body", "PhysxAutoDeformableBodyAPI"
    ),
    "PhysxAutoDeformableMeshSimplificationAPI":
    Component(
        lambda p: p.HasAPI("PhysxAutoDeformableMeshSimplificationAPI"),
        lambda p: p.HasAPI("PhysxAutoDeformableBodyAPI"),
        "PhysxAutoDeformableMeshSimplificationAPI", "Deformable Mesh Simplification", "PhysxAutoDeformableMeshSimplificationAPI"
    ),
    "PhysxAutoDeformableHexahedralMeshAPI":
    Component(
        lambda p: p.HasAPI("PhysxAutoDeformableHexahedralMeshAPI"),
        lambda p: False,
        "PhysxAutoDeformableHexahedralMeshAPI", "Deformable Hexahedral Mesh", "PhysxAutoDeformableHexahedralMeshAPI"
    ),
    "PhysxAutoDeformableAttachmentAPI":
    Component(
        lambda p: p.HasAPI("PhysxAutoDeformableAttachmentAPI"),
        lambda p: p.HasAPI("PhysxAutoDeformableAttachmentAPI"),
        "PhysxAutoDeformableAttachmentAPI", "Deformable Attachment", "PhysxAutoDeformableAttachmentAPI"
    ),
    PhysxSchema.PhysxCharacterControllerAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxCharacterControllerAPI),
        lambda p: p.IsA(UsdGeom.Capsule) and not p._refresh_cache.has_rigidbody_api,
        "PhysxCharacterControllerAPI", "Character Controller", PhysxSchema.PhysxCharacterControllerAPI
    ),
    UsdPhysics.MassAPI.__name__:
    Component(
        lambda p: p.HasAPI(UsdPhysics.MassAPI),
        lambda p: p._refresh_cache.canadd_massapi,
        "PhysicsMassAPI", "Mass", UsdPhysics.MassAPI
    ),
    PhysxSchema.PhysxContactReportAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxContactReportAPI),
        lambda p: p._refresh_cache.has_rigidbody_api,
        "ContactReportAPI", "Contact Reporter", PhysxSchema.PhysxContactReportAPI
    ),
    PhysxSchema.PhysxSurfaceVelocityAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxSurfaceVelocityAPI),
        lambda p: p._refresh_cache.has_rigidbody_api,
        "SurfaceVelocityAPI", "Surface Velocity", PhysxSchema.PhysxSurfaceVelocityAPI
    ),    
    "PhysxSplinesSurfaceVelocityAPI":
    Component(
        lambda p: p.HasAPI("PhysxSplinesSurfaceVelocityAPI"),
        lambda p: p._refresh_cache.has_rigidbody_api,
        "PhysxSplinesSurfaceVelocityAPI", "Splines Surface Velocity", "PhysxSplinesSurfaceVelocityAPI"
    ),    
    UsdPhysics.FilteredPairsAPI.__name__:
    Component(
        lambda p: p.HasAPI(UsdPhysics.FilteredPairsAPI),
        lambda p: p._refresh_cache.canadd_filteredpairs,
        "PhysicsFilteredPairsAPI", "Filtered Pairs", UsdPhysics.FilteredPairsAPI
    ),
    PhysxSchema.PhysxTriggerAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxTriggerAPI),
        lambda p: p._refresh_cache.has_collision_api,
        "TriggerAPI", "Trigger", PhysxSchema.PhysxTriggerAPI
    ),
    PhysxSchema.PhysxMeshMergeCollisionAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxMeshMergeCollisionAPI),
        lambda p: p.IsA(UsdGeom.Xformable),
        "MeshMergeCollisionAPI", "MeshMergeCollision", PhysxSchema.PhysxMeshMergeCollisionAPI,
    ),
    PhysxSchema.PhysxSceneQuasistaticAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxSceneQuasistaticAPI),
        lambda p: p.IsA(UsdPhysics.Scene),
        "QuasistaticAPI", "Quasistatic Mode", PhysxSchema.PhysxSceneQuasistaticAPI,
    ),
    PhysxSchema.PhysxTriggerStateAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxTriggerStateAPI),
        lambda p: p.HasAPI(PhysxSchema.PhysxTriggerAPI),
        "TriggerStateAPI", "TriggerState", PhysxSchema.PhysxTriggerStateAPI
    ),
    PhysxSchema.PhysxForceAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxForceAPI),
        lambda p: not p.HasAPI(PhysxSchema.PhysxForceAPI),
        "ForceAPI", "Force", PhysxSchema.PhysxForceAPI,
        can_show=lambda p: p.IsA(UsdGeom.Xformable) and pxb.ancestorHasAPI(UsdPhysics.RigidBodyAPI, p),
    ),
    UsdPhysics.DistanceJoint.__name__:
    Component(
        lambda p: p.IsA(UsdPhysics.DistanceJoint),
        lambda p: p._refresh_cache.is_a_base_joint,
        "PhysicsDistanceJoint", "Distance Joint", UsdPhysics.DistanceJoint,
        can_show=lambda p: p._refresh_cache.is_a_base_joint,
    ),
    UsdPhysics.PrismaticJoint.__name__:
    Component(
        lambda p: p.IsA(UsdPhysics.PrismaticJoint),
        lambda p: p._refresh_cache.is_a_base_joint,
        "PhysicsPrismaticJoint", "Prismatic Joint", UsdPhysics.PrismaticJoint,
        can_show=lambda p: p._refresh_cache.is_a_base_joint,
    ),
    UsdPhysics.RevoluteJoint.__name__:
    Component(
        lambda p: p.IsA(UsdPhysics.RevoluteJoint),
        lambda p: p._refresh_cache.is_a_base_joint,
        "PhysicsRevoluteJoint", "Revolute Joint", UsdPhysics.RevoluteJoint,
        can_show=lambda p: p._refresh_cache.is_a_base_joint,
    ),
    UsdPhysics.SphericalJoint.__name__:
    Component(
        lambda p: p.IsA(UsdPhysics.SphericalJoint),
        lambda p: p._refresh_cache.is_a_base_joint,
        "PhysicsSphericalJoint", "Spherical Joint", UsdPhysics.SphericalJoint,
        can_show=lambda p: p._refresh_cache.is_a_base_joint,
    ),
    UsdPhysics.FixedJoint.__name__:
    Component(
        lambda p: p.IsA(UsdPhysics.FixedJoint),
        lambda p: p._refresh_cache.is_a_base_joint,
        "PhysicsFixedJoint", "Fixed Joint", UsdPhysics.FixedJoint,
        can_show=lambda p: p._refresh_cache.is_a_base_joint,
    ),
    UsdPhysics.MaterialAPI.__name__:
    Component(
        lambda p: p.HasAPI(UsdPhysics.MaterialAPI),
        lambda p: p.IsA(UsdShade.Material),
        "PhysicsMaterialAPI", "Rigid Body Material", UsdPhysics.MaterialAPI,
    ),
    # DEPRECATED
    PhysxSchema.PhysxDeformableBodyMaterialAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxDeformableBodyMaterialAPI),
        lambda p: p.IsA(UsdShade.Material),
        "PhysxDeformableBodyMaterialAPI", "Deformable Body Material (deprecated)", PhysxSchema.PhysxDeformableBodyMaterialAPI,
        can_show=lambda p: deformable_deprecated_enabled
    ),
    "OmniPhysicsDeformableMaterialAPI":
    Component(
        lambda p: p.HasAPI("OmniPhysicsDeformableMaterialAPI"),
        lambda p: p.IsA(UsdShade.Material),
        "OmniPhysicsDeformableMaterialAPI", "Deformable Material", "OmniPhysicsDeformableMaterialAPI",
        can_show=lambda p: not deformable_deprecated_enabled
    ),
    "OmniPhysicsSurfaceDeformableMaterialAPI":
    Component(
        lambda p: p.HasAPI("OmniPhysicsSurfaceDeformableMaterialAPI"),
        lambda p: p.IsA(UsdShade.Material),
        "OmniPhysicsSurfaceDeformableMaterialAPI", "Surface Deformable Material", "OmniPhysicsSurfaceDeformableMaterialAPI",
        can_show=lambda p: not deformable_deprecated_enabled
    ),
    PhysxSchema.PhysxVehicleContextAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxVehicleContextAPI),
        lambda p: p.IsA(UsdPhysics.Scene),
        "PhysxVehicleContextAPI", "Vehicle Context", PhysxSchema.PhysxVehicleContextAPI
    ),
    PhysxSchema.PhysxVehicleSuspensionComplianceAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxVehicleSuspensionComplianceAPI) and omni.timeline.get_timeline_interface().is_stopped(),
        lambda p: p.HasAPI(PhysxSchema.PhysxVehicleWheelAttachmentAPI) and omni.timeline.get_timeline_interface().is_stopped(),
        "PhysxVehicleSuspensionComplianceAPI", "Vehicle Suspension Compliance", PhysxSchema.PhysxVehicleSuspensionComplianceAPI
    ),
    PhysxSchema.PhysxVehicleSteeringAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxVehicleSteeringAPI) and omni.timeline.get_timeline_interface().is_stopped(),
        lambda p: p.HasAPI(PhysxSchema.PhysxVehicleAPI) and (not p.HasAPI(PhysxSchema.PhysxVehicleAckermannSteeringAPI)) and omni.timeline.get_timeline_interface().is_stopped(),
        "PhysxVehicleSteeringAPI", "Vehicle Steering Basic", PhysxSchema.PhysxVehicleSteeringAPI
    ),
    PhysxSchema.PhysxVehicleAckermannSteeringAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxVehicleAckermannSteeringAPI) and omni.timeline.get_timeline_interface().is_stopped(),
        lambda p: p.HasAPI(PhysxSchema.PhysxVehicleAPI) and (not p.HasAPI(PhysxSchema.PhysxVehicleSteeringAPI)) and omni.timeline.get_timeline_interface().is_stopped(),
        "PhysxVehicleAckermannSteeringAPI", "Vehicle Steering Ackermann", PhysxSchema.PhysxVehicleAckermannSteeringAPI
    ),
    PhysxSchema.PhysxTendonAttachmentAPI.__name__:
    Component(
        lambda _: False,  # one can always add another attachment
        lambda p: p.IsA(UsdGeom.Xformable) and p._refresh_cache.has_rigidbody_api,
        "PhysxTendonAttachmentAPI", "Spatial Tendon Attachment", PhysxSchema.PhysxTendonAttachmentAPI
    ),
    PhysxSchema.PhysxTendonAttachmentRootAPI.__name__:
    Component(
        lambda _: False,  # one can always add another attachment
        lambda p: p.IsA(UsdGeom.Xformable) and p._refresh_cache.has_rigidbody_api,
        "PhysxTendonAttachmentRootAPI", "Spatial Tendon Root Attachment", PhysxSchema.PhysxTendonAttachmentRootAPI
    ),
    PhysxSchema.PhysxTendonAttachmentLeafAPI.__name__:
    Component(
        lambda _: False,  # one can always add another attachment
        lambda p: p.IsA(UsdGeom.Xformable) and p._refresh_cache.has_rigidbody_api,
        "PhysxTendonAttachmentLeafAPI", "Spatial Tendon Leaf Attachment", PhysxSchema.PhysxTendonAttachmentLeafAPI
    ),
    PhysxSchema.PhysxTendonAxisAPI.__name__:
    Component(
        lambda _: False,  # one can always add another attachment
        lambda p: p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint),
        "PhysxTendonAxisAPI", "Fixed Tendon Axis", PhysxSchema.PhysxTendonAxisAPI
    ),
    PhysxSchema.PhysxTendonAxisRootAPI.__name__:
    Component(
        lambda _: False,  # one can always add another attachment
        lambda p: p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint),
        "PhysxTendonAxisRootAPI", "Fixed Tendon Root Axis", PhysxSchema.PhysxTendonAxisRootAPI
    ),
    # DEPRECATED
    PhysxSchema.PhysxDeformableBodyAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxDeformableBodyAPI),
        lambda p: p.IsA(UsdGeom.Mesh) and not pxb.hasconflictingapis_PhysxDeformableBodyAPI_deprecated(p, True),
        "PhysxDeformableBodyAPI", "Deformable Body (deprecated)", PhysxSchema.PhysxDeformableBodyAPI,
        can_show=lambda p: deformable_deprecated_enabled
    ),
    # DEPRECATED
    PhysxSchema.PhysxDeformableSurfaceAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxDeformableSurfaceAPI),
        lambda p: p.IsA(UsdGeom.Mesh) and not pxb.hasconflictingapis_PhysxDeformableSurfaceAPI_deprecated(p, True),
        "PhysxDeformableSurfaceAPI", "Deformable Surface (deprecated)", PhysxSchema.PhysxDeformableSurfaceAPI,
        can_show=lambda p: False
    ),
    # DEPRECATED
    PhysxSchema.PhysxAutoAttachmentAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxAutoAttachmentAPI),
        lambda p: p.IsA(PhysxSchema.PhysxPhysicsAttachment),
        "PhysxAutoAttachmentAPI", "Compute Auto Attachment (deprecated)", PhysxSchema.PhysxAutoAttachmentAPI
    ),
    PhysxSchema.PhysxParticleSamplingAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxParticleSamplingAPI),
        lambda p: p.IsA(UsdGeom.Mesh) and not pxb.hasconflictingapis_PhysxParticleSamplingAPI(p, False),
        "PhysxParticleSamplingAPI", "Particle Sampler", PhysxSchema.PhysxParticleSamplingAPI
    ),
    PhysxSchema.PhysxParticleSetAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxParticleSetAPI),
        lambda p: (p.IsA(UsdGeom.Points) or p.IsA(UsdGeom.PointInstancer)),
        "PhysxParticleSetAPI", "Particle Set", PhysxSchema.PhysxParticleSetAPI
    ),
    # DEPRECATED
    PhysxSchema.PhysxParticleClothAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxParticleClothAPI),
        lambda p: p.IsA(UsdGeom.Mesh) and not pxb.hasconflictingapis_PhysxParticleClothAPI_deprecated(p, False),
        "PhysxParticleClothAPI", "Particle Cloth (deprecated)", PhysxSchema.PhysxParticleClothAPI,
        can_show=lambda p: deformable_deprecated_enabled
    ),
    # DEPRECATED
    PhysxSchema.PhysxAutoParticleClothAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxAutoParticleClothAPI),
        lambda p: p.HasAPI(PhysxSchema.PhysxParticleClothAPI),
        "PhysxAutoParticleClothAPI", "Compute Springs for Cloth", PhysxSchema.PhysxAutoParticleClothAPI
    ),
    PhysxSchema.PhysxParticleAnisotropyAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxParticleAnisotropyAPI),
        lambda p: p.IsA(PhysxSchema.PhysxParticleSystem),
        "PhysxParticleAnisotropyAPI", "Particle Anisotropy", PhysxSchema.PhysxParticleAnisotropyAPI
    ),
    PhysxSchema.PhysxParticleSmoothingAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxParticleSmoothingAPI),
        lambda p: p.IsA(PhysxSchema.PhysxParticleSystem),
        "PhysxParticleSmoothingAPI", "Particle Smoothing", PhysxSchema.PhysxParticleSmoothingAPI
    ),
    PhysxSchema.PhysxParticleIsosurfaceAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxParticleIsosurfaceAPI),
        lambda p: p.IsA(PhysxSchema.PhysxParticleSystem),
        "PhysxParticleIsosurfaceAPI", "Particle Isosurface", PhysxSchema.PhysxParticleIsosurfaceAPI
    ),
    PhysxSchema.PhysxDiffuseParticlesAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxDiffuseParticlesAPI),
        lambda p: p.HasAPI(PhysxSchema.PhysxParticleSetAPI),
        "PhysxDiffuseParticlesAPI", "Diffuse Particles", PhysxSchema.PhysxDiffuseParticlesAPI
    ),
    PhysxSchema.PhysxPBDMaterialAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxPBDMaterialAPI),
        lambda p: p.IsA(UsdShade.Material),
        "PhysxPBDMaterialAPI", "PBD Material", PhysxSchema.PhysxPBDMaterialAPI,
    ),
}

MAData = namedtuple('MAData', "gen_title, instances, apply_name, can_add, gen_inst_name, gen_component_name, schema")

def get_translation_str(axis):
    return f"{axis} Axis Translation"


def get_rotation_str(axis):
    return f"{axis} Axis Rotation"

axes = ["X", "Y", "Z"]

multi_apply_data = [
    MAData(
        lambda axis: f"{get_translation_str(axis)} Limit", axes, "PhysicsLimitAPI",
        lambda p: p._refresh_cache.is_a_base_joint, lambda axis: f"trans{axis}", None, UsdPhysics.LimitAPI
    ),
    MAData(
        lambda axis: f"{get_rotation_str(axis)} Limit", axes, "PhysicsLimitAPI",
        lambda p: p._refresh_cache.is_a_base_joint,
        lambda axis: f"rot{axis}", None, UsdPhysics.LimitAPI
    ),
    MAData(
        lambda _: "Distance Limit", [""], "PhysxLimitAPI",
        lambda p: p._refresh_cache.is_a_base_joint,
        lambda _: "distance", lambda _: "PhysicsLimit:distance", UsdPhysics.LimitAPI
    ),
    MAData(
        lambda axis: f"{get_translation_str(axis)} Drive", axes, "PhysicsDriveAPI",
        lambda p: p._refresh_cache.is_a_base_joint,
        lambda axis: f"trans{axis}", None, UsdPhysics.DriveAPI
    ),
    MAData(
        lambda axis: f"{get_rotation_str(axis)} Drive", axes, "PhysicsDriveAPI",
        lambda p: p._refresh_cache.is_a_base_joint,
        lambda axis: f"rot{axis}", None, UsdPhysics.DriveAPI
    ),
    MAData(
        lambda _: "Linear Drive", [""], "PhysicsDriveAPI",
        lambda p: p.IsA(UsdPhysics.PrismaticJoint),
        lambda _: "linear", lambda _: "PhysicsDrive:linear", UsdPhysics.DriveAPI
    ),
    MAData(
        lambda _: "Angular Drive", [""], "PhysicsDriveAPI",
        lambda p: p.IsA(UsdPhysics.RevoluteJoint),
        lambda _: "angular", lambda _: "PhysicsDrive:angular", UsdPhysics.DriveAPI
    ),
    MAData(
        lambda axis: f"{get_rotation_str(axis)} Joint Properties", 
        axes, 
        "PhysxJointAxisAPI",
        lambda p: p._refresh_cache.is_a_base_joint or p.IsA(UsdPhysics.SphericalJoint),
        lambda axis: f"rot{axis}", 
        lambda axis: f"PhysxJointAxis:rot{axis}", 
        "PhysxJointAxisAPI"
    ),
    MAData(
        lambda _: "Linear Joint Properties", 
        [""], 
        "PhysxJointAxisAPI",
        lambda p: p.IsA(UsdPhysics.PrismaticJoint),
        lambda _: "linear", 
        lambda _: "PhysxJointAxis:linear", 
        "PhysxJointAxisAPI"
    ),
    MAData(
        lambda _: "Angular Joint Properties", 
        [""], 
        "PhysxJointAxisAPI",
        lambda p:  p.IsA(UsdPhysics.RevoluteJoint),
        lambda _: "angular", 
        lambda _: "PhysxJointAxis:angular", 
        "PhysxJointAxisAPI"
    ),
    MAData(
        lambda axis: f"{get_translation_str(axis)} Joint State", axes, "PhysicsJointStateAPI",
        lambda p: p._refresh_cache.is_a_base_joint,
        lambda axis: f"trans{axis}", None, PhysxSchema.JointStateAPI
    ),
    MAData(
        lambda axis: f"{get_rotation_str(axis)} Joint State", axes, "PhysicsJointStateAPI",
        lambda p: p._refresh_cache.is_a_base_joint,
        lambda axis: f"rot{axis}", None, PhysxSchema.JointStateAPI
    ),
    MAData(
        lambda _: "Joint State Linear", [""], "PhysicsJointStateAPI",
        lambda p: p.IsA(UsdPhysics.PrismaticJoint),
        lambda _: "linear", lambda _: "PhysicsJointState:linear", PhysxSchema.JointStateAPI
    ),
    MAData(
        lambda _: "Joint State Angular", [""], "PhysicsJointStateAPI",
        lambda p: p.IsA(UsdPhysics.RevoluteJoint),
        lambda _: "angular", lambda _: "PhysicsJointState:angular", PhysxSchema.JointStateAPI
    ),
    MAData(
        lambda brake: f"Vehicle Brakes {brake}", ["0", "1"], "PhysxVehicleBrakesAPI",
        lambda p: p.HasAPI(PhysxSchema.PhysxVehicleAPI) and omni.timeline.get_timeline_interface().is_stopped(),
        lambda brake: f"brakes{brake}", None, PhysxSchema.PhysxVehicleBrakesAPI
    ),
    MAData(
        lambda _: "Vehicle Nonlinear Command Response (drive)", [""], "PhysxVehicleNonlinearCommandResponseAPI",
        lambda p: p.HasAPI(PhysxSchema.PhysxVehicleDriveBasicAPI) and omni.timeline.get_timeline_interface().is_stopped(),
        lambda _: PhysxSchema.Tokens.drive, None, PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI
    ),
    MAData(
        lambda _: "Vehicle Nonlinear Command Response (steer)", [""], "PhysxVehicleNonlinearCommandResponseAPI",
        lambda p: (p.HasAPI(PhysxSchema.PhysxVehicleSteeringAPI) or p.HasAPI(PhysxSchema.PhysxVehicleAckermannSteeringAPI))
            and omni.timeline.get_timeline_interface().is_stopped(),
        lambda _: PhysxSchema.Tokens.steer, None, PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI
    ),
    MAData(
        lambda _: "Vehicle Nonlinear Command Response (brakes0)", [""], "PhysxVehicleNonlinearCommandResponseAPI",
        lambda p: p.HasAPI(PhysxSchema.PhysxVehicleBrakesAPI, PhysxSchema.Tokens.brakes0) and omni.timeline.get_timeline_interface().is_stopped(),
        lambda _: PhysxSchema.Tokens.brakes0, None, PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI
    ),
    MAData(
        lambda _: "Vehicle Nonlinear Command Response (brakes1)", [""], "PhysxVehicleNonlinearCommandResponseAPI",
        lambda p: p.HasAPI(PhysxSchema.PhysxVehicleBrakesAPI, PhysxSchema.Tokens.brakes1) and omni.timeline.get_timeline_interface().is_stopped(),
        lambda _: PhysxSchema.Tokens.brakes1, None, PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI
    ),
    # mimic joint UI will focus on single degree of freedom joints. The "exotic" case of mimic joints on spherical joints and their respective
    # axes will only be covered in the sense that those will show up as components in the property window but nothing more.
    MAData(
        lambda _: "Mimic Joint", [""], "PhysxMimicJointAPI",
        lambda p: (p.IsA(UsdPhysics.PrismaticJoint) or p.IsA(UsdPhysics.RevoluteJoint)) and not p.HasAPI(PhysxSchema.PhysxMimicJointAPI),
        lambda _: UsdPhysics.Tokens.rotX, None, PhysxSchema.PhysxMimicJointAPI
    ),
    MAData(
        lambda _: "Mimic Joint (rotY)", [""], "PhysxMimicJointAPI",
        lambda _: False,
        lambda _: UsdPhysics.Tokens.rotY, None, PhysxSchema.PhysxMimicJointAPI
    ),
    MAData(
        lambda _: "Mimic Joint (rotZ)", [""], "PhysxMimicJointAPI",
        lambda _: False,
        lambda _: UsdPhysics.Tokens.rotZ, None, PhysxSchema.PhysxMimicJointAPI
    ),
]


for mad in multi_apply_data:
    for inst in mad.instances:
        inst_name = mad.gen_inst_name(inst)
        apply_name = f"{mad.apply_name}:{inst_name}"
        comp_name = mad.gen_component_name(inst) if mad.gen_component_name else apply_name
        components[f"{get_schema_name(mad.schema)}:{inst_name}"] = Component(
            lambda p, apply_name=apply_name: apply_name in p.GetAppliedSchemas(),
            mad.can_add, comp_name, mad.gen_title(inst), mad.schema)

instance_display_names = {
    **{f"trans{axis}": get_translation_str(axis) for axis in axes},
    **{f"rot{axis}": get_rotation_str(axis) for axis in axes},
    "linear": "Linear",
    "angular": "Angular",
    "distance": "Distance",
    "colliders": "Colliders",
}

display_raw_instance_name_apis = set([
    PhysxSchema.PhysxTendonAxisAPI,
    PhysxSchema.PhysxTendonAxisRootAPI,
    *attachment_apis, PhysxSchema.PhysxMimicJointAPI
])
