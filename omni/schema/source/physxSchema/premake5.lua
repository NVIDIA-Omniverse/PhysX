-- Copyright 2023 NVIDIA CORPORATION
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--    http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- require base files
local usd_plugin = require(root.."/_repo/deps/repo_usd/templates/premake/premake5-usdplugin")

-- setup options for included methods
local options = {
    usd_root = root.."/_build/target-deps/usd/%{cfg.buildcfg}",
    boost_include_path = root.."/_build/target-deps/usd/%{cfg.buildcfg}/include/boost",
    usd_lib_prefix = "",
    usd_suppress_warnings = true,
    python_root = root.."/_build/target-deps/python",
    plugin_include_dir = root.."/_build/%{cfg.system}-%{cfg.platform}/%{cfg.buildcfg}/schema/include/physxSchema",
    plugin_lib_dir = root.."/_build/%{cfg.system}-%{cfg.platform}/%{cfg.buildcfg}/schema/lib",
    plugin_module_dir = root.."/_build/%{cfg.system}-%{cfg.platform}/%{cfg.buildcfg}/schema/lib/python/PhysxSchema",
    plugin_resources_dir = root.."/_build/%{cfg.system}-%{cfg.platform}/%{cfg.buildcfg}/schema/share/usd/plugins/PhysxSchema/resources"
}

print("Using plugin_include_dir: "..options.plugin_include_dir)

local usd_libs = usd_prefix({
    "arch","tf","vt","sdf","usd","usdGeom","usdPhysics"
})

local public_headers = {
    "api.h","jointStateAPI.h","physxSurfaceVelocityAPI.h","physxSceneQuasistaticAPI.h","physxMeshMergeCollisionAPI.h","physxArticulationAPI.h", "physxAutoAttachmentAPI.h","physxAutoParticleClothAPI.h","physxCameraAPI.h","physxCameraDroneAPI.h","physxCameraFollowAPI.h","physxCameraFollowLookAPI.h","physxCameraFollowVelocityAPI.h","physxCharacterControllerAPI.h","physxCollisionAPI.h","physxContactReportAPI.h","physxConvexDecompositionCollisionAPI.h","physxConvexHullCollisionAPI.h","physxCookedDataAPI.h","physxDeformableAPI.h","physxDeformableBodyAPI.h","physxDeformableBodyMaterialAPI.h","physxDeformableSurfaceAPI.h","physxDeformableSurfaceMaterialAPI.h","physxDiffuseParticlesAPI.h","physxForceAPI.h","physxIsosurfaceAPI.h","physxJointAPI.h","physxLimitAPI.h","physxMaterialAPI.h","physxMimicJointAPI.h","physxParticleAnisotropyAPI.h","physxParticleAPI.h","physxParticleClothAPI.h","physxParticleIsosurfaceAPI.h","physxParticleSamplingAPI.h","physxParticleSetAPI.h","physxParticleSmoothingAPI.h","physxParticleSystem.h","physxPBDMaterialAPI.h","physxPhysicsAttachment.h","physxPhysicsDistanceJointAPI.h","physxPhysicsGearJoint.h","physxPhysicsInstancer.h","physxPhysicsJointInstancer.h","physxPhysicsRackAndPinionJoint.h","physxRigidBodyAPI.h","physxSceneAPI.h","physxSDFMeshCollisionAPI.h","physxSphereFillCollisionAPI.h","physxTendonAttachmentAPI.h","physxTendonAttachmentLeafAPI.h","physxTendonAttachmentRootAPI.h","physxTendonAxisAPI.h","physxTendonAxisRootAPI.h","physxTriangleMeshCollisionAPI.h","physxTriangleMeshSimplificationCollisionAPI.h","physxTriggerAPI.h","physxTriggerStateAPI.h","physxVehicleAckermannSteeringAPI.h","physxVehicleAPI.h","physxVehicleAutoGearBoxAPI.h","physxVehicleBrakesAPI.h","physxVehicleClutchAPI.h","physxVehicleContextAPI.h","physxVehicleControllerAPI.h","physxVehicleDriveBasicAPI.h","physxVehicleDriveStandardAPI.h","physxVehicleEngineAPI.h","physxVehicleGearsAPI.h","physxVehicleMultiWheelDifferentialAPI.h","physxVehicleNonlinearCommandResponseAPI.h","physxVehicleSteeringAPI.h","physxVehicleSuspensionAPI.h","physxVehicleSuspensionComplianceAPI.h","physxVehicleTankControllerAPI.h","physxVehicleTankDifferentialAPI.h","physxVehicleTireAPI.h","physxVehicleTireFrictionTable.h","physxVehicleWheelAPI.h","physxVehicleWheelAttachmentAPI.h","physxVehicleWheelControllerAPI.h","tetrahedralMesh.h","tokens.h","physxResidualReportingAPI.h"
}

local private_headers = {
    
}

local cpp_files = {
    "jointStateAPI.cpp","moduleDeps.cpp","physxSurfaceVelocityAPI.cpp","physxSceneQuasistaticAPI.cpp","physxMeshMergeCollisionAPI.cpp","physxArticulationAPI.cpp","physxAutoAttachmentAPI.cpp","physxAutoParticleClothAPI.cpp","physxCameraAPI.cpp","physxCameraDroneAPI.cpp","physxCameraFollowAPI.cpp","physxCameraFollowLookAPI.cpp","physxCameraFollowVelocityAPI.cpp","physxCharacterControllerAPI.cpp","physxCollisionAPI.cpp","physxContactReportAPI.cpp","physxConvexDecompositionCollisionAPI.cpp","physxConvexHullCollisionAPI.cpp","physxCookedDataAPI.cpp","physxDeformableAPI.cpp","physxDeformableBodyAPI.cpp","physxDeformableBodyMaterialAPI.cpp","physxDeformableSurfaceAPI.cpp","physxDeformableSurfaceMaterialAPI.cpp","physxDiffuseParticlesAPI.cpp","physxForceAPI.cpp","physxJointAPI.cpp","physxLimitAPI.cpp","physxMaterialAPI.cpp","physxMimicJointAPI.cpp","physxParticleAnisotropyAPI.cpp","physxParticleAPI.cpp","physxParticleClothAPI.cpp","physxParticleIsosurfaceAPI.cpp","physxParticleSamplingAPI.cpp","physxParticleSetAPI.cpp","physxParticleSmoothingAPI.cpp","physxParticleSystem.cpp","physxPBDMaterialAPI.cpp","physxPhysicsAttachment.cpp","physxPhysicsDistanceJointAPI.cpp","physxPhysicsGearJoint.cpp","physxPhysicsInstancer.cpp","physxPhysicsJointInstancer.cpp","physxPhysicsRackAndPinionJoint.cpp","physxRigidBodyAPI.cpp","physxSceneAPI.cpp","physxSDFMeshCollisionAPI.cpp","physxSphereFillCollisionAPI.cpp","physxTendonAttachmentAPI.cpp","physxTendonAttachmentLeafAPI.cpp","physxTendonAttachmentRootAPI.cpp","physxTendonAxisAPI.cpp","physxTendonAxisRootAPI.cpp","physxTriangleMeshCollisionAPI.cpp","physxTriangleMeshSimplificationCollisionAPI.cpp","physxTriggerAPI.cpp","physxTriggerStateAPI.cpp","physxVehicleAckermannSteeringAPI.cpp","physxVehicleAPI.cpp","physxVehicleAutoGearBoxAPI.cpp","physxVehicleBrakesAPI.cpp","physxVehicleClutchAPI.cpp","physxVehicleContextAPI.cpp","physxVehicleControllerAPI.cpp","physxVehicleDriveBasicAPI.cpp","physxVehicleDriveStandardAPI.cpp","physxVehicleEngineAPI.cpp","physxVehicleGearsAPI.cpp","physxVehicleMultiWheelDifferentialAPI.cpp","physxVehicleNonlinearCommandResponseAPI.cpp","physxVehicleSteeringAPI.cpp","physxVehicleSuspensionAPI.cpp","physxVehicleSuspensionComplianceAPI.cpp","physxVehicleTankControllerAPI.cpp","physxVehicleTankDifferentialAPI.cpp","physxVehicleTireAPI.cpp","physxVehicleTireFrictionTable.cpp","physxVehicleWheelAPI.cpp","physxVehicleWheelAttachmentAPI.cpp","physxVehicleWheelControllerAPI.cpp","tetrahedralMesh.cpp","tokens.cpp","physxResidualReportingAPI.cpp"
}

local python_module_cpp_files = {
    "module.cpp","wrapPhysxSurfaceVelocityAPI.cpp","wrapPhysxSceneQuasistaticAPI.cpp","wrapPhysxMeshMergeCollisionAPI.cpp","wrapJointStateAPI.cpp","wrapPhysxArticulationAPI.cpp","wrapPhysxAutoAttachmentAPI.cpp","wrapPhysxAutoParticleClothAPI.cpp","wrapPhysxCameraAPI.cpp","wrapPhysxCameraDroneAPI.cpp","wrapPhysxCameraFollowAPI.cpp","wrapPhysxCameraFollowLookAPI.cpp","wrapPhysxCameraFollowVelocityAPI.cpp","wrapPhysxCharacterControllerAPI.cpp","wrapPhysxCollisionAPI.cpp","wrapPhysxContactReportAPI.cpp","wrapPhysxConvexDecompositionCollisionAPI.cpp","wrapPhysxConvexHullCollisionAPI.cpp","wrapPhysxCookedDataAPI.cpp","wrapPhysxDeformableAPI.cpp","wrapPhysxDeformableBodyAPI.cpp","wrapPhysxDeformableBodyMaterialAPI.cpp","wrapPhysxDeformableSurfaceAPI.cpp","wrapPhysxDeformableSurfaceMaterialAPI.cpp","wrapPhysxDiffuseParticlesAPI.cpp","wrapPhysxForceAPI.cpp","wrapPhysxJointAPI.cpp","wrapPhysxLimitAPI.cpp","wrapPhysxMaterialAPI.cpp","wrapPhysxMimicJointAPI.cpp","wrapPhysxParticleAnisotropyAPI.cpp","wrapPhysxParticleAPI.cpp","wrapPhysxParticleClothAPI.cpp","wrapPhysxParticleIsosurfaceAPI.cpp","wrapPhysxParticleSamplingAPI.cpp","wrapPhysxParticleSetAPI.cpp","wrapPhysxParticleSmoothingAPI.cpp","wrapPhysxParticleSystem.cpp","wrapPhysxPBDMaterialAPI.cpp","wrapPhysxPhysicsAttachment.cpp","wrapPhysxPhysicsDistanceJointAPI.cpp","wrapPhysxPhysicsGearJoint.cpp","wrapPhysxPhysicsInstancer.cpp","wrapPhysxPhysicsJointInstancer.cpp","wrapPhysxPhysicsRackAndPinionJoint.cpp","wrapPhysxRigidBodyAPI.cpp","wrapPhysxSceneAPI.cpp","wrapPhysxSDFMeshCollisionAPI.cpp","wrapPhysxSphereFillCollisionAPI.cpp","wrapPhysxTendonAttachmentAPI.cpp","wrapPhysxTendonAttachmentLeafAPI.cpp","wrapPhysxTendonAttachmentRootAPI.cpp","wrapPhysxTendonAxisAPI.cpp","wrapPhysxTendonAxisRootAPI.cpp","wrapPhysxTriangleMeshCollisionAPI.cpp","wrapPhysxTriangleMeshSimplificationCollisionAPI.cpp","wrapPhysxTriggerAPI.cpp","wrapPhysxTriggerStateAPI.cpp","wrapPhysxVehicleAckermannSteeringAPI.cpp","wrapPhysxVehicleAPI.cpp","wrapPhysxVehicleAutoGearBoxAPI.cpp","wrapPhysxVehicleBrakesAPI.cpp","wrapPhysxVehicleClutchAPI.cpp","wrapPhysxVehicleContextAPI.cpp","wrapPhysxVehicleControllerAPI.cpp","wrapPhysxVehicleDriveBasicAPI.cpp","wrapPhysxVehicleDriveStandardAPI.cpp","wrapPhysxVehicleEngineAPI.cpp","wrapPhysxVehicleGearsAPI.cpp","wrapPhysxVehicleMultiWheelDifferentialAPI.cpp","wrapPhysxVehicleNonlinearCommandResponseAPI.cpp","wrapPhysxVehicleSteeringAPI.cpp","wrapPhysxVehicleSuspensionAPI.cpp","wrapPhysxVehicleSuspensionComplianceAPI.cpp","wrapPhysxVehicleTankControllerAPI.cpp","wrapPhysxVehicleTankDifferentialAPI.cpp","wrapPhysxVehicleTireAPI.cpp","wrapPhysxVehicleTireFrictionTable.cpp","wrapPhysxVehicleWheelAPI.cpp","wrapPhysxVehicleWheelAttachmentAPI.cpp","wrapPhysxVehicleWheelControllerAPI.cpp","wrapTetrahedralMesh.cpp","wrapTokens.cpp","wrapPhysxResidualReportingAPI.cpp"
}

local python_module_files = {
    "__init__.py"
}

local resource_files = {
    "generatedSchema.usda","plugInfo.json","schema.usda"
}


-- USD plugin C++ project
project("physxSchema")
    -- standard USD plugin settings
    usd_plugin.usd_plugin("physxSchema", options, public_headers, private_headers, cpp_files, resource_files, usd_libs)
    usd_plugin.use_standard_usd_options()
    do_usd_zcinline_fix()
    link_boost_for_windows_wdefault()
    filter { "system:linux"}
        buildoptions { "-fvisibility=default" }
    
local count = 0
if python_module_cpp_files ~= nil then
    for _ in pairs(python_module_cpp_files) do
        count = count + 1
    end
end

if count > 0 then
    
project("_physxSchema")
    -- standard USD python plugin settings
    usd_plugin.usd_python_plugin("physxSchema", options, python_module_cpp_files, python_module_files, usd_libs)
    usd_plugin.use_standard_usd_options()
    do_usd_zcinline_fix()
    link_boost_for_windows_wdefault()

end