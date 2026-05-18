# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from pxr import UsdPhysics, PhysxSchema, UsdGeom, Usd
from collections import defaultdict, namedtuple
from omni.physx.scripts import utils as physxUtils

_extension_api_maps = defaultdict(dict)

def add_extension_apis(parent_schema, api, extension_apis):
    _extension_api_maps[parent_schema][api] = extension_apis

def remove_extension_apis(parent_schema):
    _extension_api_maps.pop(parent_schema, None)

def get_extension_apis(root_api):
    ret = []
    for api_map in _extension_api_maps.values():
        if root_api in api_map:
            ret += api_map[root_api]
    return ret

_internal_extension_api_maps = defaultdict(dict)

def add_internal_extension_apis(parent_schema, api, extension_apis):
    _internal_extension_api_maps[parent_schema][api] = extension_apis

def remove_internal_extension_apis(parent_schema):
    _internal_extension_api_maps.pop(parent_schema, None)

def get_internal_extension_apis(root_api):
    ret = []
    for api_map in _internal_extension_api_maps.values():
        if root_api in api_map:
            ret += api_map[root_api]
    return ret

_extras_api_maps = defaultdict(dict)

def add_extras_apis(parent_schema, api, extras_apis):
    _extras_api_maps[parent_schema][api] = extras_apis

def remove_extras_apis(parent_schema):
    _extras_api_maps.pop(parent_schema, None)

def get_extras_apis(root_api):
    ret = []
    for api_map in _extras_api_maps.values():
        if root_api in api_map:
            ret += api_map[root_api]
    return ret

"""
FIXME: do we need this?
base_prims = [
    UsdGeom.Plane,
]
"""

material_apis = [
    UsdPhysics.MaterialAPI,
    PhysxSchema.PhysxDeformableBodyMaterialAPI, # DEPRECATED
    PhysxSchema.PhysxDeformableSurfaceMaterialAPI, # DEPRECATED
    "OmniPhysicsDeformableMaterialAPI",
    "OmniPhysicsSurfaceDeformableMaterialAPI",
    PhysxSchema.PhysxPBDMaterialAPI,
]

attachment_apis = [
    PhysxSchema.PhysxTendonAttachmentAPI,
    PhysxSchema.PhysxTendonAttachmentLeafAPI,
    PhysxSchema.PhysxTendonAttachmentRootAPI
]

# FIXME: hardoced for visual tests to pass now
priority_dict = {
    "PhysicsJoint": -1,
    "PhysicsRigidBodyAPI": -1,
    "PhysxParticleSetAPI": -1,
    "PhysxParticleClothAPI": -1,
    # everything else is 0
    "OmniPhysicsDeformableBodyAPI": 1,
    "PhysxDiffuseParticlesAPI": 1,
    "PhysxDeformableBodyAPI": 1,
    "OmniPhysicsDeformableBodyAPI": 1,
    "PhysxAutoDeformableBodyAPI": 3,
    "PhysxAutoDeformableMeshSimplificationAPI": 4,
    "PhysxAutoDeformableHexahedralMeshAPI": 5,
}

# special case api extension placeholders
extension_api_conditions = {
    UsdPhysics.MeshCollisionAPI: lambda prim: prim.IsA(UsdGeom.Mesh),
}

for name, api in physxUtils.MESH_APPROXIMATIONS.items():
    def check(prim, name=name):
        if not (prim.IsA(UsdGeom.Mesh) or (prim.IsA(UsdGeom.Xformable) and prim.HasAPI(PhysxSchema.PhysxMeshMergeCollisionAPI))):
            return False
        if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            return False
        if UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() != name:
            return False
        return True

    if api is not None:
        extension_api_conditions[api] = check

# mutlti-api hardcoded prefixes
# FIXME: can this be parsed out too? doesn't seem to be the case ...
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
