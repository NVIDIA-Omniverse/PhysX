# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from collections import namedtuple, defaultdict
from pxr import UsdPhysics, PhysxSchema, UsdGeom, Usd, UsdShade, Gf
from .utils import Limits, generate_schema_aliases, generate_codeless_schema_aliases, get_schema_name
from omni.kit.commands import execute
import omni.kit.notification_manager as nm
from omni.physx.scripts import utils as physxUtils
from omni.physx.scripts import particleUtils
import omni.timeline
import carb.settings
import math
import copy
import omni.physx.bindings._physx as pxb

material_apis = [
    UsdPhysics.MaterialAPI,
    PhysxSchema.PhysxDeformableBodyMaterialAPI, # DEPRECATED
    PhysxSchema.PhysxDeformableSurfaceMaterialAPI, # DEPRECATED
    "OmniPhysicsDeformableMaterialAPI", #codeless schema type
    "OmniPhysicsSurfaceDeformableMaterialAPI", #codeless schema type
    PhysxSchema.PhysxPBDMaterialAPI,
]

base_prims = [
    UsdPhysics.Scene,
    UsdPhysics.CollisionGroup,
    PhysxSchema.TetrahedralMesh, # DEPRECATED
    "OmniPhysicsVtxVtxAttachment", #codeless schema type
    "OmniPhysicsVtxTriAttachment", #codeless schema type
    "OmniPhysicsVtxTetAttachment", #codeless schema type
    "OmniPhysicsTriTriAttachment", #codeless schema type
    "OmniPhysicsVtxXformAttachment", #codeless schema type
    "OmniPhysicsElementCollisionFilter", #codeless schema type
    UsdGeom.Plane,
    PhysxSchema.PhysxParticleSystem,
    PhysxSchema.PhysxPhysicsAttachment, # DEPRECATED
]

base_apis = [
    UsdPhysics.RigidBodyAPI,
    UsdPhysics.CollisionAPI,
    UsdPhysics.MassAPI,
    UsdPhysics.FilteredPairsAPI,
    UsdPhysics.ArticulationRootAPI,
    "OmniPhysicsDeformableBodyAPI", #codeless schema type
    "OmniPhysicsVolumeDeformableSimAPI", #codeless schema type
    "OmniPhysicsSurfaceDeformableSimAPI", #codeless schema type
    "OmniPhysicsDeformablePoseAPI", #codeless schema type
    "PhysxAutoDeformableBodyAPI", #codeless schema type
    "PhysxAutoDeformableMeshSimplificationAPI", #codeless schema type
    "PhysxAutoDeformableHexahedralMeshAPI", #codeless schema type
    "PhysxAutoDeformableAttachmentAPI", #codeless schema type
    PhysxSchema.PhysxCharacterControllerAPI,
    PhysxSchema.PhysxDeformableBodyAPI, # DEPRECATED
    PhysxSchema.PhysxDeformableSurfaceAPI, # DEPRECATED
    PhysxSchema.PhysxContactReportAPI,
    PhysxSchema.PhysxSurfaceVelocityAPI,
    "PhysxSplinesSurfaceVelocityAPI",  # codeless schema
    PhysxSchema.PhysxTriggerAPI,
    PhysxSchema.PhysxMeshMergeCollisionAPI,
    PhysxSchema.PhysxTriggerStateAPI,
    PhysxSchema.PhysxAutoAttachmentAPI, # DEPRECATED
    PhysxSchema.PhysxParticleSamplingAPI,
    PhysxSchema.PhysxParticleSetAPI,
    PhysxSchema.PhysxParticleClothAPI, # DEPRECATED
    PhysxSchema.PhysxAutoParticleClothAPI, # DEPRECATED
    PhysxSchema.PhysxParticleAnisotropyAPI,
    PhysxSchema.PhysxParticleSmoothingAPI,
    PhysxSchema.PhysxParticleIsosurfaceAPI,
    PhysxSchema.PhysxDiffuseParticlesAPI,
    PhysxSchema.PhysxForceAPI,
    PhysxSchema.PhysxSceneQuasistaticAPI,
    PhysxSchema.PhysxResidualReportingAPI,
    *material_apis,
]

derived_attachment_apis = [
    PhysxSchema.PhysxTendonAttachmentLeafAPI,
    PhysxSchema.PhysxTendonAttachmentRootAPI
]

attachment_apis = [
    PhysxSchema.PhysxTendonAttachmentAPI,
    *derived_attachment_apis
]

derived_axis_apis = [
    PhysxSchema.PhysxTendonAxisRootAPI,
]

axis_apis = [
    PhysxSchema.PhysxTendonAxisAPI,
    *derived_axis_apis
]

derived_joint_prims = [
    UsdPhysics.RevoluteJoint,
    UsdPhysics.PrismaticJoint,
    UsdPhysics.SphericalJoint,
    UsdPhysics.DistanceJoint,
    UsdPhysics.FixedJoint,
    PhysxSchema.PhysxPhysicsGearJoint,
    PhysxSchema.PhysxPhysicsRackAndPinionJoint,
]

joint_prims = [
    UsdPhysics.Joint,
    *derived_joint_prims
]

joint_apis = [
    UsdPhysics.LimitAPI,
    UsdPhysics.DriveAPI,
    PhysxSchema.JointStateAPI,
    "PhysxDrivePerformanceEnvelopeAPI", 
    "PhysxJointAxisAPI"
]

vehicle_apis = [
    PhysxSchema.PhysxVehicleContextAPI,
    PhysxSchema.PhysxVehicleWheelAPI,
    PhysxSchema.PhysxVehicleTireAPI,
    PhysxSchema.PhysxVehicleSuspensionAPI,
    PhysxSchema.PhysxVehicleEngineAPI,
    PhysxSchema.PhysxVehicleGearsAPI,
    PhysxSchema.PhysxVehicleAutoGearBoxAPI,
    PhysxSchema.PhysxVehicleClutchAPI,
    PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI,
    PhysxSchema.PhysxVehicleDriveBasicAPI,
    PhysxSchema.PhysxVehicleDriveStandardAPI,
    PhysxSchema.PhysxVehicleWheelAttachmentAPI,
    PhysxSchema.PhysxVehicleSuspensionComplianceAPI,
    PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI,
    PhysxSchema.PhysxVehicleTankDifferentialAPI,
    PhysxSchema.PhysxVehicleBrakesAPI,
    PhysxSchema.PhysxVehicleSteeringAPI,
    PhysxSchema.PhysxVehicleAckermannSteeringAPI,
    PhysxSchema.PhysxVehicleAPI,
    PhysxSchema.PhysxVehicleControllerAPI,
    PhysxSchema.PhysxVehicleTankControllerAPI,
    PhysxSchema.PhysxVehicleWheelControllerAPI,
]

camera_apis = [
    PhysxSchema.PhysxCameraFollowLookAPI,
    PhysxSchema.PhysxCameraFollowVelocityAPI,
    PhysxSchema.PhysxCameraDroneAPI,
]

mimic_joint_apis = [
    PhysxSchema.PhysxMimicJointAPI,
]

# api extensions to base physics apis/prims
extension_api_map = {
    UsdPhysics.Scene: [PhysxSchema.PhysxSceneAPI],
    UsdPhysics.RigidBodyAPI: [PhysxSchema.PhysxRigidBodyAPI],
    "OmniPhysicsDeformableBodyAPI": ["PhysxBaseDeformableBodyAPI", "PhysxSurfaceDeformableBodyAPI"],
    UsdPhysics.CollisionAPI: [
        PhysxSchema.PhysxCollisionAPI,
        UsdPhysics.MeshCollisionAPI,
        PhysxSchema.PhysxConvexHullCollisionAPI,
        PhysxSchema.PhysxConvexDecompositionCollisionAPI,
        PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI,
        PhysxSchema.PhysxTriangleMeshCollisionAPI,
        PhysxSchema.PhysxSDFMeshCollisionAPI,
        PhysxSchema.PhysxSphereFillCollisionAPI,
    ],
    UsdPhysics.MaterialAPI: [PhysxSchema.PhysxMaterialAPI],
    "OmniPhysicsDeformableMaterialAPI": ["PhysxDeformableMaterialAPI"],
    "OmniPhysicsSurfaceDeformableMaterialAPI": ["PhysxSurfaceDeformableMaterialAPI"],
    UsdPhysics.Joint: [PhysxSchema.PhysxJointAPI],
    UsdPhysics.DistanceJoint: [PhysxSchema.PhysxPhysicsDistanceJointAPI],
    UsdPhysics.LimitAPI: [PhysxSchema.PhysxLimitAPI],
    UsdPhysics.DriveAPI: ["PhysxDrivePerformanceEnvelopeAPI"],
    UsdPhysics.ArticulationRootAPI: [PhysxSchema.PhysxArticulationAPI],
    # DEPRECATED
    PhysxSchema.PhysxDeformableBodyAPI: [PhysxSchema.PhysxCollisionAPI], 
    PhysxSchema.PhysxDeformableSurfaceAPI: [PhysxSchema.PhysxCollisionAPI],
}

derived_joint_schemas = physxUtils.getDerivedSchemas(UsdPhysics.Joint)
for djs in physxUtils.getDerivedSchemas(UsdPhysics.Joint):
    extension_api_map[djs] = extension_api_map.get(djs, []) + [PhysxSchema.PhysxJointAPI]

# api extensions + other apis
other_api_map = {
    UsdPhysics.Scene: [Usd.CollectionAPI],
    UsdPhysics.CollisionGroup: [Usd.CollectionAPI],
    PhysxSchema.PhysxMeshMergeCollisionAPI: [Usd.CollectionAPI],
    PhysxSchema.PhysxSceneQuasistaticAPI: [Usd.CollectionAPI],
}

# special case api extension placeholders
extension_api_conditions = {
    UsdPhysics.MeshCollisionAPI: lambda prim: prim.IsA(UsdGeom.Mesh),
}

for name, api in physxUtils.MESH_APPROXIMATIONS.items():
    if api is not None:
        extension_api_conditions[api] = lambda prim: prim.IsA(UsdGeom.Mesh) and UsdPhysics.MeshCollisionAPI(prim) and UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() == name


widget_extension_map = defaultdict(list, copy.deepcopy(extension_api_map))
for k, v in other_api_map.items():
    widget_extension_map[k] += v

custom_order_map = {
    Usd.CollectionAPI: ["includes", "excludes", "expansionRule", "includeRoot"]
}

limit_drive_order = ["transX", "transY", "transZ", "rotX", "rotY", "rotZ"]
properties_order= limit_drive_order = ["rotX", "rotY", "rotZ"]

custom_instance_order_map = {
    UsdPhysics.LimitAPI: limit_drive_order,
    PhysxSchema.PhysxLimitAPI: limit_drive_order,
    UsdPhysics.DriveAPI: limit_drive_order,
    Usd.SchemaRegistry.GetTypeFromSchemaTypeName("PhysxJointAxisAPI"): properties_order
}

ext_multi_api_prefixes = {
    PhysxSchema.PhysxLimitAPI: "physxLimit",
    PhysxSchema.PhysxTendonAttachmentAPI: "physxTendon",
    PhysxSchema.PhysxTendonAttachmentLeafAPI: "physxTendon",
    PhysxSchema.PhysxTendonAttachmentRootAPI: "physxTendon",
    PhysxSchema.PhysxTendonAxisRootAPI: "physxTendon",
    PhysxSchema.PhysxTendonAxisAPI: "physxTendon",
    PhysxSchema.PhysxVehicleBrakesAPI: PhysxSchema.Tokens.physxVehicleBrakes,
    PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI: PhysxSchema.Tokens.physxVehicleNCR,
    PhysxSchema.PhysxMimicJointAPI: PhysxSchema.Tokens.physxMimicJoint,
    Usd.SchemaRegistry.GetTypeFromSchemaTypeName("PhysxDrivePerformanceEnvelopeAPI"): "physxDrivePerformanceEnvelope",
    Usd.SchemaRegistry.GetTypeFromSchemaTypeName("OmniPhysicsDeformablePoseAPI"): "deformablePose",
}

multi_api_add_button_text = {
    UsdPhysics.LimitAPI: "Limit",
    PhysxSchema.PhysxLimitAPI: "Limit",
    UsdPhysics.DriveAPI: "Drive",
    Usd.SchemaRegistry.GetTypeFromSchemaTypeName("PhysxDrivePerformanceEnvelopeAPI"): "Envelope",
    Usd.SchemaRegistry.GetTypeFromSchemaTypeName("PhysxJointAxisAPI"): "Properties",
    PhysxSchema.JointStateAPI: "Joint State",
    PhysxSchema.PhysxTendonAttachmentAPI: "Spatial Tendon Attachment",
    PhysxSchema.PhysxTendonAxisAPI: "Fixed Tendon Axis",
    PhysxSchema.PhysxTendonAttachmentRootAPI: "Spatial Tendon Root Attachment",
    PhysxSchema.PhysxTendonAttachmentLeafAPI: "Spatial Tendon Leaf Attachment",
    PhysxSchema.PhysxTendonAxisRootAPI: "Fixed Tendon Root Axis",
    PhysxSchema.PhysxVehicleBrakesAPI: "Brakes",
    PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI: "Nonlinear Command Response",
    PhysxSchema.PhysxMimicJointAPI: "Mimic Joint",
    Usd.SchemaRegistry.GetTypeFromSchemaTypeName("OmniPhysicsDeformablePoseAPI"): "Deformable Pose",
}

MultiApiNamingInfo = namedtuple('MultiApiNamingInfo', "title, text")

named_apis = {
    PhysxSchema.PhysxTendonAttachmentAPI.__name__:
    MultiApiNamingInfo("Add Spatial Tendon Attachment", "Please enter an instance name for the attachment:"),
    PhysxSchema.PhysxTendonAttachmentRootAPI.__name__:
    MultiApiNamingInfo("Add Spatial Tendon Root", "Please enter an instance name for the root attachment:"),
    PhysxSchema.PhysxTendonAttachmentLeafAPI.__name__:
    MultiApiNamingInfo("Add Spatial Tendon Leaf", "Please enter an instance name for the leaf attachment:"),
    PhysxSchema.PhysxTendonAxisRootAPI.__name__:
    MultiApiNamingInfo("Add Fixed Tendon Root Axis", "Please enter a fixed tendon name:"),
    PhysxSchema.PhysxTendonAxisAPI.__name__:
    MultiApiNamingInfo("Add Fixed Tendon Axis", "Please enter the name of the tendon that this axis belongs to:"),
}

Component = namedtuple('Component', "is_present, can_add, name, title, main_schema, add_component_fn, remove_component_fn, can_show")
Component.__new__.__defaults__ = (
    None,   # add_component_fn
    None,   # remove_component_fn
    None,   # can_show
)

# OM-96962: Commenting this type name computation since it can happen before necessary libs are 
#   loaded and result in start up error like this:
#   pxr.Tf.ErrorException: 
#       Error in 'pxrInternal_v0_22__pxrReserved__::`anonymous-namespace'::_GetPluginForType' at line 147 in file 
#       W:\ac88d7d902b57417\USD\pxr\usd\ar\resolver.cpp : 'Failed to find plugin for OmniUsdResolver'
# derived_axis_type_names = {
#     *[Usd.SchemaRegistry().GetSchemaTypeName(api) for api in derived_axis_apis]
# }

# OM-96962: Delay the computation of `derived_joint_prims` and `derived_attachments_type_names` 
#   to avoid error on startup
_derived_joints_type_names = None
_derived_attachments_type_names = None

def is_a_base_joint(p):
    global _derived_joints_type_names
    if _derived_joints_type_names is None:
        _derived_joints_type_names = {
            *[Usd.SchemaRegistry().GetSchemaTypeName(prim) for prim in derived_joint_prims]
        }
    return p.IsA(UsdPhysics.Joint) and not p.GetTypeName() in _derived_joints_type_names


def is_a_base_attachment(api):
    global _derived_attachments_type_names
    if _derived_attachments_type_names is None:
        _derived_attachments_type_names = {
            *[Usd.SchemaRegistry().GetSchemaTypeName(api) for api in derived_attachment_apis]
        }
    return api.__name__ not in _derived_attachments_type_names


def is_a_base_axis(api):
    return api.__name__ not in derived_axis_apis


def has_any_material_api(p):
    return any([p.HasAPI(api) for api in material_apis])


schema_aliases = dict()
generate_schema_aliases(schema_aliases, UsdPhysics, "Usd")
generate_schema_aliases(schema_aliases, PhysxSchema, "PhysxSchema")
generate_codeless_schema_aliases(schema_aliases, "physxSchemaAddition")
generate_codeless_schema_aliases(schema_aliases, "omniUsdPhysicsDeformableSchema")

deformable_beta_enabled = carb.settings.get_settings().get(pxb.SETTING_ENABLE_DEFORMABLE_BETA)

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
        "OmniPhysicsDeformableBodyAPI", "Deformable Body (beta)", "OmniPhysicsDeformableBodyAPI"
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
        "PhysxAutoDeformableAttachmentAPI", "Deformable Attachment (beta)", "PhysxAutoDeformableAttachmentAPI"
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
    PhysxSchema.PhysxResidualReportingAPI.__name__:
    Component(
        lambda p: p.HasAPI(PhysxSchema.PhysxResidualReportingAPI),
        lambda p: p.IsA(UsdPhysics.Scene) or p.IsA(UsdPhysics.Joint) or p.HasAPI(UsdPhysics.ArticulationRootAPI),
        "PhysxResidualReportingAPI", "Residual Reporting", PhysxSchema.PhysxResidualReportingAPI,
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
        can_show=lambda p: not deformable_beta_enabled
    ),
    "OmniPhysicsDeformableMaterialAPI":
    Component(
        lambda p: p.HasAPI("OmniPhysicsDeformableMaterialAPI"),
        lambda p: p.IsA(UsdShade.Material),
        "OmniPhysicsDeformableMaterialAPI", "Deformable Material (beta)", "OmniPhysicsDeformableMaterialAPI",
        can_show=lambda p: deformable_beta_enabled
    ),
    "OmniPhysicsSurfaceDeformableMaterialAPI":
    Component(
        lambda p: p.HasAPI("OmniPhysicsSurfaceDeformableMaterialAPI"),
        lambda p: p.IsA(UsdShade.Material),
        "OmniPhysicsSurfaceDeformableMaterialAPI", "Surface Deformable Material (beta)", "OmniPhysicsSurfaceDeformableMaterialAPI",
        can_show=lambda p: deformable_beta_enabled
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
        can_show=lambda p: not deformable_beta_enabled
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
        can_show=lambda p: not deformable_beta_enabled
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
    *axis_apis, *attachment_apis, PhysxSchema.PhysxMimicJointAPI
])

def check_surface_deformable_application(prim: Usd.Prim) -> bool:
    error_msg = None
    mesh = UsdGeom.Mesh(prim)
    if mesh:
        face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get()
        if not face_vertex_counts or len(face_vertex_counts) == 0:
            error_msg = "Surface Deformable Body: Mesh is empty or has non-triangular faces."
        elif not all(c == 3 for c in face_vertex_counts):
            error_msg = (
                "Surface Deformable Body: Mesh has non-triangular faces. "
                "Nest mesh under Xform and create surface deformable on Xform."
            )
    if error_msg is not None:
        nm.post_notification(error_msg, duration=8, status=nm.NotificationStatus.WARNING)
        return False
    return True

ExtraAddItem = namedtuple("ExtraItem", "title, can_add, on_click, can_show")
ExtraAddItem.__new__.__defaults__ = (
    None,   # can_show
)

extra_add_items = [
    ExtraAddItem(
        "Rigid Body with Colliders Preset",
        lambda p: not p._refresh_cache.has_rigidbody_api,
        lambda p: execute("SetRigidBody", path=p.GetPath(), approximationShape="convexHull", kinematic=False),
        lambda p: p.IsA(UsdGeom.Xformable) and not p.IsA(PhysxSchema.PhysxParticleSystem) and not p._refresh_cache.hasconflictingapis_RigidBodyAPI,
    ),
    ExtraAddItem(
        "Colliders Preset",
        lambda p: not p._refresh_cache.has_collision_api,
        lambda p: execute("SetStaticCollider", path=p.GetPath(), approximationShape="none"),
        lambda p: p.IsA(UsdGeom.Xformable) and not p.IsA(PhysxSchema.PhysxParticleSystem) and not p._refresh_cache.hasconflictingapis_CollisionAPI,
    ),
    ExtraAddItem(
        "Volume Deformable Body (beta)",
        lambda p: not p.HasAPI("OmniPhysicsDeformableBodyAPI"),
        lambda p: execute("SetVolumeDeformableBody", prim_path=p.GetPath()),
        lambda p: deformable_beta_enabled and p.IsA(UsdGeom.TetMesh) and not pxb.hasconflictingapis_DeformableBodyAPI(p, True),
    ),
    ExtraAddItem(
        "Surface Deformable Body (beta)",
        lambda p: not p.HasAPI("OmniPhysicsDeformableBodyAPI"),
        lambda p: check_surface_deformable_application(p) and execute("SetSurfaceDeformableBody", prim_path=p.GetPath()),
        lambda p: deformable_beta_enabled and p.IsA(UsdGeom.Mesh) and not pxb.hasconflictingapis_DeformableBodyAPI(p, True),
    ),
    ExtraAddItem(
        "Water Preset (PBD Material)",
        lambda p: p.IsA(UsdShade.Material),
        lambda p: particleUtils.AddPBDMaterialWater(p),
    ),
    ExtraAddItem(
        "Viscous Fluid Preset (PBD Material)",
        lambda p: p.IsA(UsdShade.Material),
        lambda p: particleUtils.AddPBDMaterialViscous(p),
    )
]


class InfoData:
    def __init__(self, min=None, max=None, step=0.1, multiply_meter_fn=None, multiply_kg_fn=None):
        self.min = min
        self.max = max
        self.step = step
        self.multiply_meter_fn = multiply_meter_fn
        self.multiply_kg_fn = multiply_kg_fn


INFO_FLT_NONNEG = InfoData(*Limits.RANGE_FLT_NONNEG)
INFO_DEG = InfoData(*Limits.RANGE_DEG, 1)
INFO_RAD = InfoData(*Limits.RANGE_RAD)
INFO_FLT_NONNEG_VEC = InfoData(Gf.Vec3f(0), Gf.Vec3f(Limits.FLT_INF))
INFO_DEG_VEC = InfoData(Gf.Vec3f(-Limits.DEG_MAX), Gf.Vec3f(Limits.DEG_MAX))
INFO_BYTE_POS = InfoData(1, 255, 1)
INFO_BYTE_VEL = InfoData(0, 255, 1)
INFO_FLT_FINITE = InfoData(Limits.FLT_NMAX, Limits.FLT_MAX)

property_info = {
    "physics:gravityMagnitude": INFO_FLT_NONNEG,
    "physics:mass": INFO_FLT_NONNEG,
    "physics:density": InfoData(*Limits.RANGE_FLT_NONNEG, 0.1, multiply_meter_fn=lambda mpu: mpu * mpu * mpu, multiply_kg_fn=lambda kgpu: 1.0 / kgpu),
    "physics:diagonalInertia": INFO_FLT_NONNEG_VEC,
    "physics:dynamicFriction": INFO_FLT_NONNEG,
    "physics:staticFriction": INFO_FLT_NONNEG,
    "physics:restitution": InfoData(0, 1),
    "physics:breakForce": InfoData(*Limits.RANGE_FLT_NONNEG, 0.1, multiply_meter_fn=lambda mpu: 1.0 / mpu, multiply_kg_fn=lambda kgpu: 1.0 / kgpu),
    "physics:breakTorque": InfoData(*Limits.RANGE_FLT_NONNEG, 0.1, multiply_meter_fn=lambda mpu: 1.0 / (mpu * mpu), multiply_kg_fn=lambda kgpu: 1.0 / kgpu),
    "physics:coneAngle0Limit": InfoData(0, 180, 1),
    "physics:coneAngle1Limit": InfoData(0, 180, 1),
    "physics:minDistance": INFO_FLT_NONNEG,
    "physics:maxDistance": INFO_FLT_NONNEG,
    "physics:low": {"rot": INFO_DEG, "trans": None, "distance": None},
    "physics:high": {"rot": INFO_DEG, "trans": None, "distance": None},
    "physics:maxForce": INFO_FLT_NONNEG,  # for now we are not going to scale this with a multiply function, since it scales differently based on whether this is a linear or angular drive
    "physics:damping": INFO_FLT_NONNEG,
    "physics:stiffness": INFO_FLT_NONNEG,
    "maxActuatorVelocity": INFO_FLT_NONNEG,
    "velocityDependentResistance": INFO_FLT_NONNEG,
    "speedEffortGradient": INFO_FLT_NONNEG,
    "armature": INFO_FLT_NONNEG,
    "maxJointVelocity": INFO_FLT_NONNEG,
    "staticFrictionEffort": INFO_FLT_NONNEG,
    "dynamicFrictionEffort": INFO_FLT_NONNEG,
    "viscousFrictionCoefficient": INFO_FLT_NONNEG,
    "slopeLimit": InfoData(0, 0.5 * math.pi),
    "physxScene:bounceThreshold": InfoData(*Limits.RANGE_FLT_NONNEG, 0.1, lambda mpu: 10 / mpu),
    "physxScene:frictionOffsetThreshold": InfoData(*Limits.RANGE_FLT_NONNEG, 0.1, lambda mpu: 1 / mpu),
    "physxScene:minPositionIterationCount": INFO_BYTE_POS,
    "physxScene:maxPositionIterationCount": INFO_BYTE_POS,
    "physxScene:minVelocityIterationCount": INFO_BYTE_VEL,
    "physxScene:maxVelocityIterationCount": INFO_BYTE_VEL,
    "physxScene:envIdInBoundsBitCount": InfoData(-1, 16, step=1),
    "physxRigidBody:linearDamping": INFO_FLT_NONNEG,
    "physxRigidBody:angularDamping": INFO_FLT_NONNEG,
    "physxRigidBody:maxLinearVelocity": INFO_FLT_NONNEG,
    "physxRigidBody:maxAngularVelocity": INFO_FLT_NONNEG,
    "physxRigidBody:sleepThreshold": InfoData(*Limits.RANGE_FLT_NONNEG, 0.1, lambda mpu: (10 / mpu) * (10 / mpu)),
    "physxRigidBody:stabilizationThreshold": InfoData(*Limits.RANGE_FLT_NONNEG, 0.1, lambda mpu: (10 / mpu) * (10 / mpu)),
    "physxRigidBody:maxDepenetrationVelocity": InfoData(*Limits.RANGE_FLT_NONNEG, multiply_meter_fn=lambda mpu: 1.0 / mpu),
    "physxRigidBody:maxContactImpulse": INFO_FLT_NONNEG,
    "physxRigidBody:solverPositionIterationCount": INFO_BYTE_POS,
    "physxRigidBody:solverVelocityIterationCount": INFO_BYTE_VEL,
    "physxContactReport:threshold": InfoData(multiply_meter_fn=lambda mpu: 1 / mpu),
    "physxCollision:contactOffset": InfoData(*Limits.RANGE_FLT_NONNEG, 0.1, lambda mpu: 1 / mpu),
    "physxCollision:torsionalPatchRadius": INFO_FLT_NONNEG,
    "physxCollision:minTorsionalPatchRadius": INFO_FLT_NONNEG,
    "physxConvexHullCollision:hullVertexLimit": InfoData(8, 64, 1),
    "physxConvexHullCollision:minThickness": InfoData(0.001, 10.0, 0.1, lambda mpu: 1 / mpu),
    "physxConvexDecompositionCollision:hullVertexLimit": InfoData(8, 64, 1),
    "physxConvexDecompositionCollision:maxConvexHulls": InfoData(1, 2048, 1),
    "physxConvexDecompositionCollision:minThickness": InfoData(0.001, 10.0, 0.1, lambda mpu: 1 / mpu),
    "physxConvexDecompositionCollision:voxelResolution": InfoData(50000, 5000000, 10000),
    "physxConvexDecompositionCollision:errorPercentage": InfoData(0, 20, 0.25),
    "physxTriangleMeshSimplificationCollision:metric": InfoData(0, 1, 0.02),
    "physxJoint:jointFriction": INFO_FLT_NONNEG,
    "physxPhysicsDistanceJoint:springStiffness": INFO_FLT_NONNEG,
    "physxPhysicsDistanceJoint:springDamping": INFO_FLT_NONNEG,
    "restitution": INFO_FLT_NONNEG,
    "bounceThreshold": INFO_FLT_NONNEG,
    "stiffness": INFO_FLT_NONNEG,
    "damping": INFO_FLT_NONNEG,
    "physxArticulation:solverPositionIterationCount": INFO_BYTE_POS,
    "physxArticulation:solverVelocityIterationCount": INFO_BYTE_VEL,
    "physxArticulation:sleepThreshold": InfoData(*Limits.RANGE_FLT_NONNEG, 0.1, lambda mpu: (10 / mpu) * (10 / mpu)),
    "physxArticulation:stabilizationThreshold": InfoData(*Limits.RANGE_FLT_NONNEG, 0.1, lambda mpu: (10 / mpu) * (10 / mpu)),
    "physxArticulationJoint:maxJointVelocity": INFO_FLT_NONNEG,
    "physxArticulationJoint:frictionCoefficient": INFO_FLT_NONNEG,
    "physxCharacterController:invisibleWallHeight": INFO_FLT_NONNEG,
    "physxCharacterController:maxJumpHeight": INFO_FLT_NONNEG,
    "physxCharacterController:contactOffset": INFO_FLT_NONNEG,
    "physxCharacterController:stepOffset": INFO_FLT_NONNEG,
    "physxCharacterController:scaleCoeff": INFO_FLT_NONNEG,
    "physxCharacterController:volumeGrowth": INFO_FLT_NONNEG,
    "limitStiffness": INFO_FLT_NONNEG,
    "restLength": INFO_FLT_NONNEG,
    # DEPRECATED
    "physxDeformable:simulationHexahedralResolution": InfoData(4, 64, 1),
    "physxDeformable:collisionSimplificationRemeshingResolution": InfoData(10, 1000, 1),
    "physxDeformable:collisionSimplificationTargetTriangleCount": InfoData(0, 100000, 1),
    "physxDeformable:sleepThreshold": InfoData(*Limits.RANGE_FLT_NONNEG, 0.01, lambda mpu: 1.0 / mpu),
    "physxDeformable:settlingThreshold": InfoData(*Limits.RANGE_FLT_NONNEG, 0.01, lambda mpu: 1.0 / mpu),
    "physxDeformable:selfCollisionFilterDistance": InfoData(*Limits.RANGE_FLT_NONNEG, 0.01, lambda mpu: 1.0 / mpu),
    "physxDeformable:solverPositionIterationCount": INFO_BYTE_POS,
    "physxDeformableBodyMaterial:dampingScale": InfoData(0, 1),
    "physxDeformableBodyMaterial:density": InfoData(*Limits.RANGE_FLT_NONNEG, 0.1, multiply_meter_fn=lambda mpu: mpu * mpu * mpu, multiply_kg_fn=lambda kgpu: 1.0 / kgpu),
    "physxDeformableBodyMaterial:dynamicFriction": INFO_FLT_NONNEG,
    "physxDeformableBodyMaterial:elasticityDamping": INFO_FLT_NONNEG,
    "physxDeformableBodyMaterial:poissonsRatio": InfoData(0, 0.499),
    "physxDeformableBodyMaterial:youngsModulus": InfoData(*Limits.RANGE_FLT_NONNEG, 1, lambda mpu: mpu),
    "physxDeformableSurfaceMaterial:bendingDamping": InfoData(0, 1, multiply_meter_fn=lambda mpu: 1.0 / mpu / mpu),
    "physxDeformableSurfaceMaterial:density": InfoData(*Limits.RANGE_FLT_NONNEG, 0.1, multiply_meter_fn=lambda mpu: mpu * mpu * mpu, multiply_kg_fn=lambda kgpu: 1.0 / kgpu),
    "physxDeformableSurfaceMaterial:dynamicFriction": INFO_FLT_NONNEG,
    "physxDeformableSurfaceMaterial:elasticityDamping": INFO_FLT_NONNEG,
    "physxDeformableSurfaceMaterial:poissonsRatio": InfoData(0, 0.499),
    "physxDeformableSurfaceMaterial:thickness": InfoData(*Limits.RANGE_FLT_NONNEG, 0.1, lambda mpu: 1.0 / mpu),
    "physxDeformableSurfaceMaterial:youngsModulus": InfoData(*Limits.RANGE_FLT_NONNEG, 1, lambda mpu: mpu),
    #~DEPRECATED
    "contactOffset": InfoData(*Limits.RANGE_FLT_NONNEG, 0.01, multiply_meter_fn=lambda mpu: 1 / mpu), ## particle system is not an API, does not have a prefix
    "restOffset": InfoData(*Limits.RANGE_FLT_NONNEG, 0.01, multiply_meter_fn=lambda mpu: 1 / mpu),
    "particleContactOffset": InfoData(*Limits.RANGE_FLT_NONNEG, 0.01, multiply_meter_fn=lambda mpu: 1 / mpu),
    "solidRestOffset": InfoData(*Limits.RANGE_FLT_NONNEG, 0.01, multiply_meter_fn=lambda mpu: 1 / mpu),
    "fluidRestOffset": InfoData(*Limits.RANGE_FLT_NONNEG, 0.01, multiply_meter_fn=lambda mpu: 1 / mpu),
    "solverPositionIterationCount": INFO_BYTE_POS,
    "maxDepenetrationVelocity": InfoData(*Limits.RANGE_FLT_NONNEG, multiply_meter_fn=lambda mpu: 1.0 / mpu),
    "maxVelocity": InfoData(*Limits.RANGE_FLT_NONNEG, multiply_meter_fn=lambda mpu: 1.0 / mpu), ## end particle system attributes
    "physxParticleIsosurface:maxVertices": InfoData(3, Limits.INT_MAX),
    "physxParticleIsosurface:maxTriangles": InfoData(1, Limits.INT_MAX),
    "physxParticleIsosurface:maxSubgrids": InfoData(1, Limits.INT_MAX),
    "physxParticleIsosurface:gridSpacing": InfoData(*Limits.RANGE_FLT_NONNEG, multiply_meter_fn=lambda mpu: 1 / mpu),
    "physxParticleIsosurface:surfaceDistance": InfoData(*Limits.RANGE_FLT_NONNEG, multiply_meter_fn=lambda mpu: 1 / mpu),
    "physxParticleIsosurface:gridSmoothingRadius": INFO_FLT_NONNEG,
    "physxParticleIsosurface:numMeshSmoothingPasses": InfoData(0, Limits.INT_MAX),
    "physxParticleIsosurface:numMeshNormalSmoothingPasses": InfoData(0, Limits.INT_MAX),
    "physxParticleSmoothing:strength": InfoData(0, 1),
    "physxParticleAnisotropy:scale": INFO_FLT_NONNEG,
    "physxParticleAnisotropy:min": INFO_FLT_NONNEG,
    "physxParticleAnisotropy:max": INFO_FLT_NONNEG,
    "physxDiffuseParticles:maxDiffuseParticleMultiplier": InfoData(0, 4),
    "physxDiffuseParticles:threshold": InfoData(0, Limits.FLT_MAX, multiply_meter_fn=lambda mpu: mpu * mpu, multiply_kg_fn=lambda kpu: 1 / kpu),
    "physxDiffuseParticles:lifetime": INFO_FLT_NONNEG,
    "physxDiffuseParticles:airDrag": INFO_FLT_NONNEG,
    "physxDiffuseParticles:bubbleDrag": INFO_FLT_NONNEG,
    "physxDiffuseParticles:buoyancy": INFO_FLT_NONNEG,
    "physxDiffuseParticles:kineticEnergyWeight": INFO_FLT_NONNEG,
    "physxDiffuseParticles:pressureWeight": INFO_FLT_NONNEG,
    "physxDiffuseParticles:divergenceWeight": INFO_FLT_NONNEG,
    "physxDiffuseParticles:collisionDecay": InfoData(0, 1),
    "physxPBDMaterial:friction": INFO_FLT_NONNEG,
    "physxPBDMaterial:particleFrictionScale": INFO_FLT_NONNEG,
    "physxPBDMaterial:damping": INFO_FLT_NONNEG,
    "physxPBDMaterial:viscosity": INFO_FLT_NONNEG,
    "physxPBDMaterial:vorticityConfinement": INFO_FLT_NONNEG,
    "physxPBDMaterial:surfaceTension": InfoData(*Limits.RANGE_FLT_NONNEG, multiply_meter_fn=lambda mpu: mpu * mpu * mpu),
    "physxPBDMaterial:cohesion": INFO_FLT_NONNEG,
    "physxPBDMaterial:adhesion": INFO_FLT_NONNEG,
    "physxPBDMaterial:particleAdhesionScale": INFO_FLT_NONNEG,
    "physxPBDMaterial:adhesionOffsetScale": INFO_FLT_NONNEG,
    "physxPBDMaterial:lift": INFO_FLT_NONNEG,
    "physxPBDMaterial:drag": INFO_FLT_NONNEG,
    "physxPBDMaterial:density": InfoData(*Limits.RANGE_FLT_NONNEG, 0.1, multiply_meter_fn=lambda mpu: mpu * mpu * mpu, multiply_kg_fn=lambda kgpu: 1.0 / kgpu),
    "physxPBDMaterial:cflCoefficient": InfoData(0, Limits.FLT_MAX),
    "physxParticle:particleGroup": InfoData(1, 2 ** 20, 1),
    "physxParticleSampling:samplingDistance": INFO_FLT_NONNEG,
    "physxParticleSampling:maxSamples": InfoData(0, Limits.INT_MAX, 1000),
    # DEPRECATED
    "physxParticle:pressure": INFO_FLT_NONNEG,
    "physxAutoParticleCloth:springStretchStiffness": InfoData(*Limits.RANGE_FLT_NONNEG, multiply_kg_fn=lambda kpu: 1 / kpu),
    "physxAutoParticleCloth:springBendStiffness": InfoData(*Limits.RANGE_FLT_NONNEG, multiply_kg_fn=lambda kpu: 1 / kpu),
    "physxAutoParticleCloth:springShearStiffness": InfoData(*Limits.RANGE_FLT_NONNEG, multiply_kg_fn=lambda kpu: 1 / kpu),
    "physxAutoParticleCloth:springDamping": InfoData(*Limits.RANGE_FLT_NONNEG, multiply_kg_fn=lambda kpu: 1 / kpu),
    #~DEPRECATED
    "physxMaterial:compliantContactStiffness": INFO_FLT_NONNEG,
    "physxMaterial:compliantContactDamping": INFO_FLT_NONNEG,
    "physxSDFMeshCollision:sdfResolution": InfoData(1, 1250),
    "physxSDFMeshCollision:sdfSubgridResolution": InfoData(0, 30),
    "physxSDFMeshCollision:sdfNarrowBandThickness": InfoData(0, 1),
    "physxSDFMeshCollision:sdfMargin": InfoData(0, 1),
    "physxScene:gpuTempBufferCapacity": InfoData(4096),
    "physxScene:gpuMaxRigidContactCount": InfoData(1),
    "physxScene:gpuMaxRigidPatchCount": InfoData(1),
    "physxScene:gpuHeapCapacity": InfoData(128),
    "physxScene:gpuFoundLostPairsCapacity": InfoData(1),
    "physxScene:gpuFoundLostAggregatePairsCapacity": InfoData(1),
    "physxScene:gpuTotalAggregatePairsCapacity": InfoData(1),
    "physxScene:gpuMaxSoftBodyContacts": InfoData(1),
    "physxScene:gpuMaxParticleContacts": InfoData(1),
    "physxScene:gpuMaxNumPartitions": InfoData(1, 32),
}

not_authored_token = "N/A"

property_sentinels = {
    "physics:gravityMagnitude": {"Earth Gravity": Limits.FLT_NINF},
    "physics:diagonalInertia": {"Ignore": Gf.Vec3f(0)},
    "physics:mass": {"Autocomputed": 0},
    "physics:density": {"Autocomputed": 0},
    "physxDeformableBodyMaterial:density": {"Autocomputed": 0},
    "physics:lowerLimit": {"Not Limited": Limits.FLT_NINF},
    "physics:upperLimit": {"Not Limited": Limits.FLT_INF},
    "physics:coneAngle0Limit": {"Not Limited": -1.0},
    "physics:coneAngle1Limit": {"Not Limited": -1.0},
    "physics:minDistance": {"Not Limited": -1.0},
    "physics:maxDistance": {"Not Limited": -1.0},
    "physics:low": {"Not Limited": Limits.FLT_NINF},
    "physics:high": {"Not Limited": Limits.FLT_INF},
    "physics:maxForce": {"Not Limited": Limits.FLT_INF},
    "restLength": {"Set to initial length": Limits.FLT_NINF},
    "physxCollision:contactOffset": {"Autocomputed": Limits.FLT_NINF},
    "physxCollision:restOffset": {"Autocomputed": Limits.FLT_NINF},
    "physxTriangleMeshCollision:weldTolerance": {"Autocomputed": Limits.FLT_NINF},
    # DEPRECATED
    "physxDeformable:collisionSimplificationRemeshingResolution": {"Autocomputed": 0},
    "physxDeformable:collisionSimplificationTargetTriangleCount": {"Autocomputed": 0},
    "physxAutoAttachment:rigidSurfaceSamplingDistance": {"Autocomputed": Limits.FLT_NINF},
    "physxAutoAttachment:collisionFilteringOffset": {"Autocomputed": Limits.FLT_NINF},
    #~DEPRECATED
    "omniphysics:mass": {"Autocomputed": 0},
    "physxDeformableBody:selfCollisionFilterDistance": {"Autocomputed": Limits.FLT_NINF},
    "physxDeformableBody:resolution": {"Autocomputed": 0},
    "physxDeformableBody:remeshingResolution": {"Autocomputed": 0},
    "physxDeformableBody:targetTriangleCount": {"Autocomputed": 0},
    "physxAutoDeformableAttachment:rigidSurfaceSamplingDistance": {"Autocomputed": Limits.FLT_NINF},
    "physxAutoDeformableAttachment:collisionFilteringOffset": {"Autocomputed": Limits.FLT_NINF},
    "physxParticleIsosurface:gridSpacing": {"Autocomputed": Limits.FLT_NINF},
    "physxParticleIsosurface:surfaceDistance": {"Autocomputed": Limits.FLT_NINF},
    "physxParticleIsosurface:gridSmoothingRadius": {"Autocomputed": Limits.FLT_NINF},
    "physxParticleSampling:samplingDistance": {"Autocomputed": 0},
    "physxDiffuseParticles:maxDiffuseParticleMultiplier": {"Autocomputed": Limits.FLT_NINF},
    "contactOffset": {"Autocomputed": Limits.FLT_NINF},  # particle system attribute do not have a prefix
    "restOffset": {"Autocomputed": Limits.FLT_NINF},
    "solidRestOffset": {"Autocomputed": Limits.FLT_NINF},
    "fluidRestOffset": {"Autocomputed": Limits.FLT_NINF},  # end particle system attributes
    "physics:centerOfMass": {"Autocomputed": not_authored_token},
    "physics:principalAxes": {"Autocomputed": not_authored_token},
    "physxMaterial:compliantContactStiffness": {"Compliance Disabled": 0.0},
    "physxVehicle:subStepThresholdLongitudinalSpeed": {"Autocomputed": not_authored_token},
    "physxVehicle:minLongitudinalSlipDenominator": {"Autocomputed": not_authored_token},
    "physxVehicle:minPassiveLongitudinalSlipDenominator": {"Use Deprecated Attribute": 0},
    "physxVehicle:minActiveLongitudinalSlipDenominator": {"Autocomputed": 0},
    "physxVehicle:minLateralSlipDenominator": {"Autocomputed": 0},
    "physxVehicle:longitudinalStickyTireThresholdSpeed": {"Autocomputed": -1.0},
    "physxVehicle:lateralStickyTireThresholdSpeed": {"Autocomputed": -1.0},
    "physxVehicleSuspension:sprungMass": {"Autocomputed": 0.0},
    "physxVehicleTire:restLoad": {"Autocomputed": 0.0},
    "physxVehicleTire:lateralStiffnessGraph": {"Use Deprecated Attributes": Gf.Vec2f(0)},
    "physxVehicleTire:longitudinalStiffness": {"Use Deprecated Attribute": 0},
    "physxVehicleTire:camberStiffness": {"Use Deprecated Attribute": -1.0},
    "physxVehicleEngine:peakTorque": {"Autocomputed": -1.0},
    "physxVehicleEngine:dampingRateFullThrottle": {"Autocomputed": -1.0},
    "physxVehicleEngine:dampingRateZeroThrottleClutchEngaged": {"Autocomputed": -1.0},
    "physxVehicleEngine:dampingRateZeroThrottleClutchDisengaged": {"Autocomputed": -1.0},
    "physxVehicleDriveBasic:peakTorque": {"Autocomputed": -1.0},
}

property_change_on_edit_end_override = {
    "physics:lowerLimit": False,
    "physics:upperLimit": False,
    "physics:localRot0": False,
    "physics:localRot1": False,
}

property_instance_type = {
    **{f"trans{axis}": "trans" for axis in axes},
    **{f"rot{axis}": "rot" for axis in axes},
    "linear": "linear",
    "angular": "angular",
    "distance": "distance",
}


# call this from e.g. Manager.__init__ to get string to update default_order_map below in a case of a schema change
# relative path of the schemas to ${kit} is valid only for the physics repo!
def generate_order():
    import carb
    from pxr import Sdf
    from collections import defaultdict

    names = ["UsdPhysics", "PhysxSchema"]
    order_dict = defaultdict(list)
    for name in names:
        path = f"{name}/resources/schema.usda"
        path = carb.tokens.get_tokens_interface().resolve("${kit}/../schema/" + path)
        sdfLayer = Sdf.Layer.FindOrOpen(path)

        for sdfPrim in sdfLayer.rootPrims:
            if sdfPrim.name == "Typed" or sdfPrim.specifier != Sdf.SpecifierClass:
                continue

            sdf_name = sdfPrim.customData.get("className", sdfPrim.name)
            sdf_name = f"{name}.{sdf_name}"

            for sdfProp in sdfPrim.properties:
                order_dict[sdf_name].append(sdfProp.name)

    for name, order in order_dict.items():
        print(f"{name}: {order},")


vehicleControllerAPIPropOrder = ['physxVehicleController:accelerator', 'physxVehicleController:brake0', 'physxVehicleController:brake1', 'physxVehicleController:steer', 'physxVehicleController:targetGear']
vehicleMultiWheelDifferentialAPIPropOrder = ['physxVehicleMultiWheelDifferential:wheels', 'physxVehicleMultiWheelDifferential:torqueRatios', 'physxVehicleMultiWheelDifferential:averageWheelSpeedRatios']

default_order_map = {
    UsdPhysics.Scene: ['physics:gravityDirection', 'physics:gravityMagnitude'],
    UsdPhysics.RigidBodyAPI: ['physics:rigidBodyEnabled', 'physics:kinematicEnabled', 'physics:simulationOwner', 'physics:startsAsleep', pxb.METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES, 'physics:velocity', 'physics:angularVelocity'],
    "OmniPhysicsDeformableBodyAPI": ['omniphysics:deformableBodyEnabled', 'omniphysics:kinematicEnabled', 'omniphysics:simulationOwner', 'omniphysics:startsAsleep', 'omniphysics:mass'],
    "OmniPhysicsDeformableMaterialAPI": ['omniphysics:density', 'omniphysics:dynamicFriction', 'omniphysics:staticFriction', 'omniphysics:youngsModulus', 'omniphysics:poissonsRatio'],
    "OmniPhysicsSurfaceDeformableMaterialAPI": ['omniphysics:surfaceThickness', 'omniphysics:surfaceStretchStiffness', 'omniphysics:surfaceShearStiffness', 'omniphysics:surfaceBendStiffness'],
    UsdPhysics.MassAPI: ['physics:mass', 'physics:density', 'physics:centerOfMass', 'physics:diagonalInertia', 'physics:principalAxes'],
    UsdPhysics.CollisionAPI: ['physics:collisionEnabled', 'physics:simulationOwner'],
    UsdPhysics.MeshCollisionAPI: ['physics:approximation'],
    UsdPhysics.MaterialAPI: ['physics:dynamicFriction', 'physics:staticFriction', 'physics:restitution', 'physics:density'],
    UsdPhysics.CollisionGroup: ['physics:filteredGroups'],
    UsdPhysics.FilteredPairsAPI: ['physics:filteredPairs'],
    UsdPhysics.Joint: ['physics:jointEnabled', 'physics:body0', 'physics:body1', 'physics:localPos0', 'physics:localRot0', 'physics:localPos1', 'physics:localRot1', 'physics:collisionEnabled', 'physics:excludeFromArticulation', 'physics:breakForce', 'physics:breakTorque'],
    UsdPhysics.RevoluteJoint: ['physics:jointEnabled', 'physics:axis', 'physics:lowerLimit', 'physics:upperLimit'],
    UsdPhysics.PrismaticJoint: ['physics:jointEnabled', 'physics:axis', 'physics:lowerLimit', 'physics:upperLimit'],
    UsdPhysics.SphericalJoint: ['physics:jointEnabled', 'physics:axis', 'physics:coneAngle0Limit', 'physics:coneAngle1Limit'],
    UsdPhysics.DistanceJoint: ['physics:jointEnabled', 'physics:minDistance', 'physics:maxDistance'],
    UsdPhysics.FixedJoint: ['physics:jointEnabled'],
    UsdPhysics.LimitAPI: ['physics:low', 'physics:high'],
    UsdPhysics.DriveAPI: ['physics:type', 'physics:maxForce', 'physics:targetPosition', 'physics:targetVelocity', 'physics:damping', 'physics:stiffness'],
    PhysxSchema.TetrahedralMesh: ['indices'], # DEPRECATED
    UsdGeom.Plane: ['axis'],
    PhysxSchema.PhysxSceneAPI: ['physxScene:bounceThreshold', 'physxScene:frictionOffsetThreshold', 'physxScene:collisionSystem', 'physxScene:solverType', 'physxScene:broadphaseType', 'physxScene:frictionType', 'physxScene:enableCCD', 'physxScene:enableStabilization', 'physxScene:enableGPUDynamics', 'physxScene:enableEnhancedDeterminism', 'physxScene:enableExternalForcesEveryIteration', 'physxScene:enableResidualReporting', 'physxScene:solveArticulationContactLast','physxScene:gpuTempBufferCapacity', 'physxScene:gpuMaxRigidContactCount', 'physxScene:gpuMaxRigidPatchCount', 'physxScene:gpuHeapCapacity', 'physxScene:gpuFoundLostPairsCapacity', 'physxScene:gpuFoundLostAggregatePairsCapacity', 'physxScene:gpuTotalAggregatePairsCapacity', 'physxScene:gpuMaxSoftBodyContacts', 'physxScene:gpuMaxFEMClothContacts', 'physxScene:gpuMaxParticleContacts', 'physxScene:gpuMaxNumPartitions', 'physxScene:invertCollisionGroupFilter'],
    PhysxSchema.PhysxRigidBodyAPI: ['physxRigidBody:linearDamping', 'physxRigidBody:angularDamping', 'physxRigidBody:maxLinearVelocity', 'physxRigidBody:maxAngularVelocity', 'physxRigidBody:sleepThreshold', 'physxRigidBody:stabilizationThreshold', 'physxRigidBody:maxDepenetrationVelocity', 'physxRigidBody:maxContactImpulse', 'physxRigidBody:solverPositionIterationCount', 'physxRigidBody:solverVelocityIterationCount', 'physxRigidBody:enableCCD', 'physxRigidBody:enableSpeculativeCCD', 'physxRigidBody:disableGravity', 'physxRigidBody:lockedPosAxis', 'physxRigidBody:lockedRotAxis'],
    PhysxSchema.PhysxContactReportAPI: ['physxContactReport:threshold', 'physxContactReport:reportPairs'],
    PhysxSchema.PhysxCollisionAPI: ['physxCollision:contactOffset', 'physxCollision:restOffset', 'physxCollision:torsionalPatchRadius', 'physxCollision:minTorsionalPatchRadius'],
    PhysxSchema.PhysxMaterialAPI: ['physxMaterial:frictionCombineMode', 'physxMaterial:restitutionCombineMode', 'physxMaterial:dampingCombineMode', 'physxMaterial:compliantContactAccelerationSpring', 'physxMaterial:compliantContactStiffness', 'physxMaterial:compliantContactDamping'],
    PhysxSchema.PhysxPhysicsGearJoint: ['physics:jointEnabled', 'physics:hinge0', 'physics:hinge1', 'physics:gearRatio'],
    PhysxSchema.PhysxPhysicsRackAndPinionJoint: ['physics:jointEnabled', 'physics:hinge', 'physics:prismatic', 'physics:ratio'],
    PhysxSchema.PhysxJointAPI: ['physxJoint:jointFriction', 'physxJoint:maxJointVelocity'],
    PhysxSchema.PhysxPhysicsDistanceJointAPI: ['physxPhysicsDistanceJoint:springEnabled', 'physxPhysicsDistanceJoint:springStiffness', 'physxPhysicsDistanceJoint:springDamping'],
    PhysxSchema.PhysxLimitAPI: ['restitution', 'bounceThreshold', 'stiffness', 'damping'],
    PhysxSchema.PhysxArticulationAPI: ['physxArticulation:articulationEnabled', 'physxArticulation:solverPositionIterationCount', 'physxArticulation:solverVelocityIterationCount', 'physxArticulation:sleepThreshold', 'physxArticulation:stabilizationThreshold', 'physxArticulation:enabledSelfCollisions'],
    PhysxSchema.PhysxCharacterControllerAPI: ['physxCharacterController:moveTarget', 'physxCharacterController:slopeLimit', 'physxCharacterController:upAxis', 'physxCharacterController:nonWalkableMode', 'physxCharacterController:climbingMode', 'physxCharacterController:invisibleWallHeight', 'physxCharacterController:maxJumpHeight', 'physxCharacterController:contactOffset', 'physxCharacterController:stepOffset', 'physxCharacterController:scaleCoeff', 'physxCharacterController:volumeGrowth', 'physxCharacterController:simulationOwner'],
    PhysxSchema.PhysxTriggerAPI: ['physxTrigger:enterScriptType', 'physxTrigger:leaveScriptType', 'physxTrigger:onEnterScript', 'physxTrigger:onLeaveScript'],
    PhysxSchema.PhysxCookedDataAPI: ['physxCookedData:type', 'physxCookedData:buffer'],
    PhysxSchema.PhysxVehicleContextAPI: ['physxVehicleContext:updateMode', 'physxVehicleContext:verticalAxis', 'physxVehicleContext:longitudinalAxis'],
    PhysxSchema.PhysxVehicleTireFrictionTable: ['groundMaterials', 'frictionValues', 'defaultFrictionValue'],
    PhysxSchema.PhysxVehicleWheelAPI: ['physxVehicleWheel:radius', 'physxVehicleWheel:width', 'physxVehicleWheel:mass', 'physxVehicleWheel:moi', 'physxVehicleWheel:dampingRate'],
    PhysxSchema.PhysxVehicleTireAPI: ['physxVehicleTire:lateralStiffnessGraph', 'physxVehicleTire:longitudinalStiffness', 'physxVehicleTire:camberStiffness', 'physxVehicleTire:frictionVsSlipGraph', 'physxVehicleTire:frictionTable', 'physxVehicleTire:restLoad'],
    PhysxSchema.PhysxVehicleSuspensionAPI: ['physxVehicleSuspension:springStrength', 'physxVehicleSuspension:springDamperRate', 'physxVehicleSuspension:travelDistance', 'physxVehicleSuspension:sprungMass'],
    PhysxSchema.PhysxVehicleWheelAttachmentAPI: ['physxVehicleWheelAttachment:index', 'physxVehicleWheelAttachment:wheel', 'physxVehicleWheelAttachment:tire', 'physxVehicleWheelAttachment:suspension', 'physxVehicleWheelAttachment:suspensionTravelDirection', 'physxVehicleWheelAttachment:suspensionFramePosition', 'physxVehicleWheelAttachment:suspensionFrameOrientation', 'physxVehicleWheelAttachment:wheelFramePosition', 'physxVehicleWheelAttachment:wheelFrameOrientation', 'physxVehicleWheelAttachment:collisionGroup'],
    PhysxSchema.PhysxVehicleEngineAPI: ['physxVehicleEngine:moi', 'physxVehicleEngine:peakTorque', 'physxVehicleEngine:maxRotationSpeed', 'physxVehicleEngine:idleRotationSpeed', 'physxVehicleEngine:torqueCurve', 'physxVehicleEngine:dampingRateFullThrottle', 'physxVehicleEngine:dampingRateZeroThrottleClutchEngaged', 'physxVehicleEngine:dampingRateZeroThrottleClutchDisengaged'],
    PhysxSchema.PhysxVehicleGearsAPI: ['physxVehicleGears:ratios', 'physxVehicleGears:ratioScale', 'physxVehicleGears:switchTime'],
    PhysxSchema.PhysxVehicleAutoGearBoxAPI: ['physxVehicleAutoGearBox:upRatios', 'physxVehicleAutoGearBox:downRatios', 'physxVehicleAutoGearBox:latency'],
    PhysxSchema.PhysxVehicleClutchAPI: ['physxVehicleClutch:strength'],
    PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI: ['commandValues', 'speedResponsesPerCommandValue', 'speedResponses'],
    PhysxSchema.PhysxVehicleDriveBasicAPI: ['physxVehicleDriveBasic:peakTorque'],
    PhysxSchema.PhysxVehicleDriveStandardAPI: ['physxVehicleDriveStandard:engine', 'physxVehicleDriveStandard:gears', 'physxVehicleDriveStandard:autoGearBox', 'physxVehicleDriveStandard:clutch'],
    PhysxSchema.PhysxVehicleAPI: ['physxVehicle:vehicleEnabled', PhysxSchema.Tokens.referenceFrameIsCenterOfMass, 'physxVehicle:limitSuspensionExpansionVelocity', 'physxVehicle:drive', 'physxVehicle:suspensionLineQueryType', 'physxVehicle:subStepThresholdLongitudinalSpeed', 'physxVehicle:lowForwardSpeedSubStepCount', 'physxVehicle:highForwardSpeedSubStepCount', 'physxVehicle:minPassiveLongitudinalSlipDenominator', 'physxVehicle:minActiveLongitudinalSlipDenominator', 'physxVehicle:minLateralSlipDenominator', 'physxVehicle:longitudinalStickyTireThresholdSpeed', 'physxVehicle:longitudinalStickyTireThresholdTime', 'physxVehicle:longitudinalStickyTireDamping', 'physxVehicle:lateralStickyTireThresholdSpeed', 'physxVehicle:lateralStickyTireThresholdTime', 'physxVehicle:lateralStickyTireDamping'],
    PhysxSchema.PhysxVehicleControllerAPI: vehicleControllerAPIPropOrder,
    PhysxSchema.PhysxVehicleTankControllerAPI: ['physxVehicleTankController:thrust0', 'physxVehicleTankController:thrust1'] + vehicleControllerAPIPropOrder,
    PhysxSchema.PhysxVehicleWheelControllerAPI: ['physxVehicleWheelController:driveTorque', 'physxVehicleWheelController:brakeTorque', 'physxVehicleWheelController:steerAngle'],
    PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI: vehicleMultiWheelDifferentialAPIPropOrder,
    PhysxSchema.PhysxVehicleTankDifferentialAPI: vehicleMultiWheelDifferentialAPIPropOrder + ['physxVehicleTankDifferential:numberOfWheelsPerTrack', 'physxVehicleTankDifferential:thrustIndexPerTrack', 'physxVehicleTankDifferential:trackToWheelIndices', 'physxVehicleTankDifferential:wheelIndicesInTrackOrder'],
    PhysxSchema.PhysxVehicleBrakesAPI: ['maxBrakeTorque', 'wheels', 'torqueMultipliers'],
    PhysxSchema.PhysxVehicleSteeringAPI: ['physxVehicleSteering:wheels', 'physxVehicleSteering:maxSteerAngle', 'physxVehicleSteering:angleMultipliers'],
    PhysxSchema.PhysxVehicleAckermannSteeringAPI: ['physxVehicleAckermannSteering:wheel0', 'physxVehicleAckermannSteering:wheel1', 'physxVehicleAckermannSteering:maxSteerAngle', 'physxVehicleAckermannSteering:wheelBase', 'physxVehicleAckermannSteering:trackWidth', 'physxVehicleAckermannSteering:strength'],
    PhysxSchema.PhysxVehicleSuspensionComplianceAPI: ['physxVehicleSuspensionCompliance:wheelToeAngle', 'physxVehicleSuspensionCompliance:wheelCamberAngle', 'physxVehicleSuspensionCompliance:suspensionForceAppPoint', 'physxVehicleSuspensionCompliance:tireForceAppPoint'],
    PhysxSchema.PhysxCameraAPI: ['physxCamera:alwaysUpdateEnabled', 'physxCamera:subject'],
    PhysxSchema.PhysxCameraFollowLookAPI: ['physxFollowCamera:yawAngle', 'physxFollowCamera:pitchAngle', 'physxFollowCamera:pitchAngleTimeConstant', 'physxFollowCamera:slowSpeedPitchAngleScale', 'physxFollowCamera:slowPitchAngleSpeed', 'physxFollowLookCamera:downHillGroundAngle', 'physxFollowLookCamera:downHillGroundPitch', 'physxFollowLookCamera:upHillGroundAngle', 'physxFollowLookCamera:upHillGroundPitch', 'physxFollowCamera:velocityNormalMinSpeed', 'physxFollowLookCamera:velocityBlendTimeConstant', 'physxFollowCamera:followMinSpeed', 'physxFollowCamera:followMinDistance', 'physxFollowCamera:followMaxSpeed', 'physxFollowCamera:followMaxDistance', 'physxFollowLookCamera:followReverseSpeed', 'physxFollowLookCamera:followReverseDistance', 'physxFollowCamera:yawRateTimeConstant', 'physxFollowCamera:followTurnRateGain', 'physxFollowCamera:cameraPositionTimeConstant', 'physxFollowCamera:vehiclePositionOffset', 'physxFollowCamera:lookAheadMinSpeed', 'physxFollowCamera:lookAheadMinDistance', 'physxFollowCamera:lookAheadMaxSpeed', 'physxFollowCamera:lookAheadMaxDistance', 'physxFollowCamera:lookAheadTurnRateGain', 'physxFollowCamera:lookPositionHeight', 'physxFollowCamera:lookPositionTimeConstant'],
    PhysxSchema.PhysxCameraFollowVelocityAPI: ['physxVelocityCamera:yawAngle', 'physxVelocityCamera:pitchAngle', 'physxVelocityCamera:pitchAngleTimeConstant', 'physxVelocityCamera:slowSpeedPitchAngleScale', 'physxVelocityCamera:slowPitchAngleSpeed', 'physxVelocityCamera:velocityNormalMinSpeed', 'physxVelocityCamera:followMinSpeed', 'physxVelocityCamera:followMinDistance', 'physxVelocityCamera:followMaxSpeed', 'physxVelocityCamera:followMaxDistance', 'physxVelocityCamera:cameraPositionTimeConstant', 'physxVelocityCamera:PositionOffset', 'physxVelocityCamera:lookAheadMinSpeed', 'physxVelocityCamera:lookAheadMinDistance', 'physxVelocityCamera:lookAheadMaxSpeed', 'physxFollowVelocityCamera:lookAheadMaxDistance', 'physxVelocityCamera:lookPositionHeight', 'physxVelocityCamera:lookPositionTimeConstant'],
    PhysxSchema.PhysxCameraDroneAPI: ['physxVehicleDroneCamera:followHeight', 'physxVehicleDroneCamera:followDistance', 'physxVehicleDroneCamera:maxDistance', 'physxVehicleDroneCamera:maxSpeed', 'physxVehicleDroneCamera:horizontalVelocityGain', 'physxVehicleDroneCamera:verticalVelocityGain', 'physxVehicleDroneCamera:feedForwardVelocityGain', 'physxVehicleDroneCamera:velocityFilterTimeConstant', 'physxVehicleDroneCamera:rotationFilterTimeConstant', 'physxVehicleDroneCamera:vehiclePositionOffset'],
    PhysxSchema.PhysxParticleSystem: ['particleSystemEnabled', 'enableCCD', 'contactOffset', 'restOffset', 'particleContactOffset', 'solidRestOffset', 'fluidRestOffset', 'solverPositionIterationCount', 'solverVelocityIterationCount', 'maxVelocity', 'maxDepenetrationVelocity', 'enableCCD', 'wind', 'maxNeighborhood', 'neighborhoodScale'],
	# DEPRECATED
    PhysxSchema.PhysxDeformableBodyMaterialAPI: ['physxDeformableBodyMaterial:density', 'physxDeformableBodyMaterial:dynamicFriction', 'physxDeformableBodyMaterial:youngsModulus', 'physxDeformableBodyMaterial:poissonsRatio', 'physxDeformableBodyMaterial:elasticityDamping', 'physxDeformableBodyMaterial:dampingScale'],
    PhysxSchema.PhysxDeformableSurfaceMaterialAPI: ['physxDeformableSurfaceMaterial:density', 'physxDeformableSurfaceMaterial:thickness', 'physxDeformableSurfaceMaterial:dynamicFriction', 'physxDeformableSurfaceMaterial:youngsModulus', 'physxDeformableSurfaceMaterial:poissonsRatio', 'physxDeformableSurfaceMaterial:elasticityDamping', 'physxDeformableSurfaceMaterial:bendStiffness', 'physxDeformableSurfaceMaterial:bendDamping'],
    PhysxSchema.PhysxDeformableBodyAPI: ['physxDeformable:deformableEnabled', 'physxDeformable:kinematicEnabled', 'physxDeformable:collisionSimplification', 'physxDeformable:selfCollision', 'physxDeformable:enableCCD', 'physxDeformable:simulationHexahedralResolution', 'physxDeformable:vertexVelocityDamping', 'physxDeformable:solverPositionIterationCount', 'physxDeformable:sleepThreshold', 'physxDeformable:settlingThreshold', 'physxDeformable:sleepDamping', 'physxDeformable:selfCollisionFilterDistance'],
    PhysxSchema.PhysxDeformableSurfaceAPI: ['physxDeformable:deformableEnabled', 'physxDeformable:selfCollision', 'physxDeformableSurface:flatteningEnabled', 'physxDeformable:solverPositionIterationCount', 'physxDeformableSurface:maxVelocity', 'physxDeformable:vertexVelocityDamping', 'physxDeformable:sleepThreshold', 'physxDeformable:settlingThreshold', 'physxDeformable:sleepDamping', 'physxCollision:contactOffset', 'physxCollision:restOffset', 'physxDeformableSurface:collisionPairUpdateFrequency', 'physxDeformableSurface:collisionIterationMultiplier', 'physxDeformable:maxDepenetrationVelocity', 'physxDeformable:selfCollisionFilterDistance'],
    PhysxSchema.PhysxPhysicsAttachment: ['attachmentEnabled'],
    PhysxSchema.PhysxAutoAttachmentAPI: ['physxAutoAttachment:enableDeformableVertexAttachments', 'physxAutoAttachment:deformableVertexOverlapOffset', 'physxAutoAttachment:enableRigidSurfaceAttachments', 'physxAutoAttachment:rigidSurfaceSamplingDistance', 'physxAutoAttachment:enableCollisionFiltering', 'physxAutoAttachment:collisionFilteringOffset', 'physxAutoAttachment:maskShapes'],
    #~DEPRECATED
    "PhysxBaseDeformableBodyAPI": ['physxDeformableBody:linearDamping', 'physxDeformableBody:maxLinearVelocity', 'physxDeformableBody:sleepThreshold', 'physxDeformableBody:selfCollision', 'physxDeformableBody:enableSpeculativeCCD', 'physxDeformableBody:disableGravity', 'physxDeformableBody:settlingThreshold', 'physxDeformableBody:settlingDamping', 'physxDeformableBody:maxDepenetrationVelocity', 'physxDeformableBody:solverPositionIterationCount', 'physxDeformableBody:selfCollisionFilterDistance'],
    "PhysxAutoDeformableAttachmentAPI": ['physxAutoDeformableAttachment:attachable0','physxAutoDeformableAttachment:attachable1','physxAutoDeformableAttachment:enableDeformableVertexAttachments', 'physxAutoDeformableAttachment:deformableVertexOverlapOffset', 'physxAutoDeformableAttachment:enableRigidSurfaceAttachments', 'physxAutoDeformableAttachment:rigidSurfaceSamplingDistance', 'physxAutoDeformableAttachment:enableCollisionFiltering', 'physxAutoDeformableAttachment:collisionFilteringOffset', 'physxAutoDeformableAttachment:maskShapes'],
    "PhysxDeformableMaterialAPI" : ['physxDeformableMaterial:elasticityDamping'],
    "PhysxSurfaceDeformableMaterialAPI" : ['physxDeformableMaterial:bendDamping'],
    PhysxSchema.PhysxTendonAxisAPI: ['gearing', 'jointAxis'],
    PhysxSchema.PhysxTendonAxisRootAPI: ['tendonEnabled', 'stiffness', 'limitStiffness', 'damping', 'offset', 'restLength', 'lowerLimit', 'upperLimit'],
    PhysxSchema.PhysxTendonAttachmentRootAPI: ['tendonEnabled', 'stiffness', 'limitStiffness', 'damping', 'offset'],
    PhysxSchema.PhysxTendonAttachmentLeafAPI: ['restLength', 'upperLimit', 'lowerLimit'],
    PhysxSchema.PhysxParticleSamplingAPI: ['physxParticleSampling:samplingDistance', 'physxParticleSampling:volume', 'physxParticleSampling:particles', 'physxParticleSampling:maxSamples'],
    PhysxSchema.PhysxParticleSetAPI: ['physxParticle:particleEnabled', 'physxParticle:selfCollision', 'physxParticle:fluid', 'physxParticle:particleGroup'],
    # DEPRECATED
    PhysxSchema.PhysxParticleClothAPI: ['physxParticle:particleEnabled', 'physxParticleCloth:selfCollisionFilter', 'physxParticle:pressure'],
    PhysxSchema.PhysxAutoParticleClothAPI: ['physxAutoParticleCloth:springStretchStiffness', 'physxAutoParticleCloth:springBendStiffness', 'physxAutoParticleCloth:springShearStiffness', 'physxAutoParticleCloth:springDamping'],
    #~DEPRECATED
    PhysxSchema.PhysxParticleAnisotropyAPI: ['physxParticleAnisotropy:particleAnisotropyEnabled', 'physxParticleAnisotropy:scale', 'physxParticleAnisotropy:min', 'physxParticleAnisotropy:max'],
    PhysxSchema.PhysxParticleSmoothingAPI: ['physxParticleSmoothing:particleSmoothingEnabled', 'physxParticleSmoothing:smoothing'],
    PhysxSchema.PhysxParticleIsosurfaceAPI: ['physxParticleIsosurface:isosurfaceEnabled', 'physxParticleIsosurface:maxVertices', 'physxParticleIsosurface:maxTriangles', 'physxParticleIsosurface:maxSubgrids', 'physxParticleIsosurface:gridSpacing', 'physxParticleIsosurface:surfaceDistance', 'physxParticleIsosurface:gridFilteringFlags', 'physxParticleIsosurface:gridSmoothingRadiusRelativeToCellSize', 'physxParticleIsosurface:enableAnisotropy', 'physxParticleIsosurface:anisotropyMin', 'physxParticleIsosurface:anisotropyMax', 'physxParticleIsosurface:anisotropyRadius', 'physxParticleIsosurface:numMeshSmoothingPasses', 'physxParticleIsosurface:numMeshNormalSmoothingPasses'],
    PhysxSchema.PhysxDiffuseParticlesAPI: ['physxDiffuseParticles:diffuseParticlesEnabled', 'physxDiffuseParticles:maxDiffuseParticles', 'physxDiffuseParticles:threshold', 'physxDiffuseParticles:lifetime', 'physxDiffuseParticles:airDrag', 'physxDiffuseParticles:bubbleDrag', 'physxDiffuseParticles:buoyancy', 'physxDiffuseParticles:kineticEnergyWeight', 'physxDiffuseParticles:pressureWeight', 'physxDiffuseParticles:divergenceWeight', 'physxDiffuseParticles:collisionDecay'],
    PhysxSchema.PhysxPBDMaterialAPI: ['physxPBDMaterial:density', 'physxPBDMaterial:friction', 'physxPBDMaterial:damping', 'physxPBDMaterial:adhesion', 'physxPBDMaterial:viscosity', 'physxPBDMaterial:cohesion', 'physxPBDMaterial:surfaceTension', 'physxPBDMaterial:particleAdhesionScale', 'physxPBDMaterial:adhesionOffsetScale', 'physxPBDMaterial:drag', 'physxPBDMaterial:lift', 'physxPBDMaterial:particleFrictionScale', 'physxPBDMaterial:vorticityConfinement'],
    PhysxSchema.PhysxMimicJointAPI: ['physxMimicJoint:referenceJoint', 'physxMimicJoint:referenceJointAxis', 'physxMimicJoint:gearing', 'physxMimicJoint:offset', 'physxMimicJoint:naturalFrequency', 'physxMimicJoint:dampingRatio'],
    'PhysxDrivePerformanceEnvelopeAPI': ["maxActuatorVelocity", "speedEffortGradient", "velocityDependentResistance"], 
    'PhysxJointAxisAPI': ["maxJointVelocity", "armature", "staticFrictionEffort", "dynamicFrictionEffort", "viscousFrictionCoefficient"]
}
