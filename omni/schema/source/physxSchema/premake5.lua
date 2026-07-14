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
local usd_base = require(root.."/_repo/deps/repo_usd/templates/premake/premake5-usd")

-- setup options for included methods
local options = {
    usd_root = root.."/_build/target-deps/usd/%{cfg.buildcfg}",
    usd_suppress_warnings = true,
    -- python_root is omitted here so use_usd does not try to link the non-existent
    -- {prefix}python.lib (Python support is included in the monolithic USD lib).
    -- Python headers and the Python runtime lib are added manually below.
    plugin_include_dir = root.."/_build/%{cfg.system}-%{cfg.platform}/%{cfg.buildcfg}/schema/include/physxSchema",
    plugin_lib_dir = root.."/_build/%{cfg.system}-%{cfg.platform}/%{cfg.buildcfg}/schema/lib",
    plugin_module_dir = root.."/_build/%{cfg.system}-%{cfg.platform}/%{cfg.buildcfg}/schema/lib/python/PhysxSchema",
    plugin_resources_dir = root.."/_build/%{cfg.system}-%{cfg.platform}/%{cfg.buildcfg}/schema/share/usd/plugins/PhysxSchema/resources"
}

print("Using plugin_include_dir: "..options.plugin_include_dir)

local usd_libs = {
    "arch","tf","vt","sdf","usd","usdGeom","usdPhysics", "boost"
}

local public_headers = {
    "api.h","jointStateAPI.h","physxSurfaceVelocityAPI.h","physxSceneQuasistaticAPI.h","physxMeshMergeCollisionAPI.h","physxArticulationAPI.h","physxCameraAPI.h","physxCameraDroneAPI.h","physxCameraFollowAPI.h","physxCameraFollowLookAPI.h","physxCameraFollowVelocityAPI.h","physxCharacterControllerAPI.h","physxCollisionAPI.h","physxContactReportAPI.h","physxConvexDecompositionCollisionAPI.h","physxConvexHullCollisionAPI.h","physxCookedDataAPI.h","physxDiffuseParticlesAPI.h","physxForceAPI.h","physxIsosurfaceAPI.h","physxJointAPI.h","physxLimitAPI.h","physxMaterialAPI.h","physxMimicJointAPI.h","physxParticleAnisotropyAPI.h","physxParticleAPI.h","physxParticleIsosurfaceAPI.h","physxParticleSamplingAPI.h","physxParticleSetAPI.h","physxParticleSmoothingAPI.h","physxParticleSystem.h","physxPBDMaterialAPI.h","physxPhysicsDistanceJointAPI.h","physxPhysicsGearJoint.h","physxPhysicsInstancer.h","physxPhysicsJointInstancer.h","physxPhysicsRackAndPinionJoint.h","physxRigidBodyAPI.h","physxSceneAPI.h","physxSDFMeshCollisionAPI.h","physxSphereFillCollisionAPI.h","physxTendonAttachmentAPI.h","physxTendonAttachmentLeafAPI.h","physxTendonAttachmentRootAPI.h","physxTendonAxisAPI.h","physxTendonAxisRootAPI.h","physxTriangleMeshCollisionAPI.h","physxTriangleMeshSimplificationCollisionAPI.h","physxTriggerAPI.h","physxTriggerStateAPI.h","physxVehicleAckermannSteeringAPI.h","physxVehicleAPI.h","physxVehicleAutoGearBoxAPI.h","physxVehicleBrakesAPI.h","physxVehicleClutchAPI.h","physxVehicleContextAPI.h","physxVehicleControllerAPI.h","physxVehicleDriveBasicAPI.h","physxVehicleDriveStandardAPI.h","physxVehicleEngineAPI.h","physxVehicleGearsAPI.h","physxVehicleMultiWheelDifferentialAPI.h","physxVehicleNonlinearCommandResponseAPI.h","physxVehicleSteeringAPI.h","physxVehicleSuspensionAPI.h","physxVehicleSuspensionComplianceAPI.h","physxVehicleTankControllerAPI.h","physxVehicleTankDifferentialAPI.h","physxVehicleTireAPI.h","physxVehicleTireFrictionTable.h","physxVehicleWheelAPI.h","physxVehicleWheelAttachmentAPI.h","physxVehicleWheelControllerAPI.h","tokens.h"
}

local private_headers = {
    
}

local cpp_files = {
    "jointStateAPI.cpp","physxSurfaceVelocityAPI.cpp","physxSceneQuasistaticAPI.cpp","physxMeshMergeCollisionAPI.cpp","physxArticulationAPI.cpp","physxCameraAPI.cpp","physxCameraDroneAPI.cpp","physxCameraFollowAPI.cpp","physxCameraFollowLookAPI.cpp","physxCameraFollowVelocityAPI.cpp","physxCharacterControllerAPI.cpp","physxCollisionAPI.cpp","physxContactReportAPI.cpp","physxConvexDecompositionCollisionAPI.cpp","physxConvexHullCollisionAPI.cpp","physxCookedDataAPI.cpp","physxDiffuseParticlesAPI.cpp","physxForceAPI.cpp","physxJointAPI.cpp","physxLimitAPI.cpp","physxMaterialAPI.cpp","physxMimicJointAPI.cpp","physxParticleAnisotropyAPI.cpp","physxParticleAPI.cpp","physxParticleIsosurfaceAPI.cpp","physxParticleSamplingAPI.cpp","physxParticleSetAPI.cpp","physxParticleSmoothingAPI.cpp","physxParticleSystem.cpp","physxPBDMaterialAPI.cpp","physxPhysicsDistanceJointAPI.cpp","physxPhysicsGearJoint.cpp","physxPhysicsInstancer.cpp","physxPhysicsJointInstancer.cpp","physxPhysicsRackAndPinionJoint.cpp","physxRigidBodyAPI.cpp","physxSceneAPI.cpp","physxSDFMeshCollisionAPI.cpp","physxSphereFillCollisionAPI.cpp","physxTendonAttachmentAPI.cpp","physxTendonAttachmentLeafAPI.cpp","physxTendonAttachmentRootAPI.cpp","physxTendonAxisAPI.cpp","physxTendonAxisRootAPI.cpp","physxTriangleMeshCollisionAPI.cpp","physxTriangleMeshSimplificationCollisionAPI.cpp","physxTriggerAPI.cpp","physxTriggerStateAPI.cpp","physxVehicleAckermannSteeringAPI.cpp","physxVehicleAPI.cpp","physxVehicleAutoGearBoxAPI.cpp","physxVehicleBrakesAPI.cpp","physxVehicleClutchAPI.cpp","physxVehicleContextAPI.cpp","physxVehicleControllerAPI.cpp","physxVehicleDriveBasicAPI.cpp","physxVehicleDriveStandardAPI.cpp","physxVehicleEngineAPI.cpp","physxVehicleGearsAPI.cpp","physxVehicleMultiWheelDifferentialAPI.cpp","physxVehicleNonlinearCommandResponseAPI.cpp","physxVehicleSteeringAPI.cpp","physxVehicleSuspensionAPI.cpp","physxVehicleSuspensionComplianceAPI.cpp","physxVehicleTankControllerAPI.cpp","physxVehicleTankDifferentialAPI.cpp","physxVehicleTireAPI.cpp","physxVehicleTireFrictionTable.cpp","physxVehicleWheelAPI.cpp","physxVehicleWheelAttachmentAPI.cpp","physxVehicleWheelControllerAPI.cpp","tokens.cpp"
}

local python_module_cpp_files = {
    "module.cpp","moduleDeps.cpp","wrapPhysxSurfaceVelocityAPI.cpp","wrapPhysxSceneQuasistaticAPI.cpp","wrapPhysxMeshMergeCollisionAPI.cpp","wrapJointStateAPI.cpp","wrapPhysxArticulationAPI.cpp","wrapPhysxCameraAPI.cpp","wrapPhysxCameraDroneAPI.cpp","wrapPhysxCameraFollowAPI.cpp","wrapPhysxCameraFollowLookAPI.cpp","wrapPhysxCameraFollowVelocityAPI.cpp","wrapPhysxCharacterControllerAPI.cpp","wrapPhysxCollisionAPI.cpp","wrapPhysxContactReportAPI.cpp","wrapPhysxConvexDecompositionCollisionAPI.cpp","wrapPhysxConvexHullCollisionAPI.cpp","wrapPhysxCookedDataAPI.cpp","wrapPhysxDiffuseParticlesAPI.cpp","wrapPhysxForceAPI.cpp","wrapPhysxJointAPI.cpp","wrapPhysxLimitAPI.cpp","wrapPhysxMaterialAPI.cpp","wrapPhysxMimicJointAPI.cpp","wrapPhysxParticleAnisotropyAPI.cpp","wrapPhysxParticleAPI.cpp","wrapPhysxParticleIsosurfaceAPI.cpp","wrapPhysxParticleSamplingAPI.cpp","wrapPhysxParticleSetAPI.cpp","wrapPhysxParticleSmoothingAPI.cpp","wrapPhysxParticleSystem.cpp","wrapPhysxPBDMaterialAPI.cpp","wrapPhysxPhysicsDistanceJointAPI.cpp","wrapPhysxPhysicsGearJoint.cpp","wrapPhysxPhysicsInstancer.cpp","wrapPhysxPhysicsJointInstancer.cpp","wrapPhysxPhysicsRackAndPinionJoint.cpp","wrapPhysxRigidBodyAPI.cpp","wrapPhysxSceneAPI.cpp","wrapPhysxSDFMeshCollisionAPI.cpp","wrapPhysxSphereFillCollisionAPI.cpp","wrapPhysxTendonAttachmentAPI.cpp","wrapPhysxTendonAttachmentLeafAPI.cpp","wrapPhysxTendonAttachmentRootAPI.cpp","wrapPhysxTendonAxisAPI.cpp","wrapPhysxTendonAxisRootAPI.cpp","wrapPhysxTriangleMeshCollisionAPI.cpp","wrapPhysxTriangleMeshSimplificationCollisionAPI.cpp","wrapPhysxTriggerAPI.cpp","wrapPhysxTriggerStateAPI.cpp","wrapPhysxVehicleAckermannSteeringAPI.cpp","wrapPhysxVehicleAPI.cpp","wrapPhysxVehicleAutoGearBoxAPI.cpp","wrapPhysxVehicleBrakesAPI.cpp","wrapPhysxVehicleClutchAPI.cpp","wrapPhysxVehicleContextAPI.cpp","wrapPhysxVehicleControllerAPI.cpp","wrapPhysxVehicleDriveBasicAPI.cpp","wrapPhysxVehicleDriveStandardAPI.cpp","wrapPhysxVehicleEngineAPI.cpp","wrapPhysxVehicleGearsAPI.cpp","wrapPhysxVehicleMultiWheelDifferentialAPI.cpp","wrapPhysxVehicleNonlinearCommandResponseAPI.cpp","wrapPhysxVehicleSteeringAPI.cpp","wrapPhysxVehicleSuspensionAPI.cpp","wrapPhysxVehicleSuspensionComplianceAPI.cpp","wrapPhysxVehicleTankControllerAPI.cpp","wrapPhysxVehicleTankDifferentialAPI.cpp","wrapPhysxVehicleTireAPI.cpp","wrapPhysxVehicleTireFrictionTable.cpp","wrapPhysxVehicleWheelAPI.cpp","wrapPhysxVehicleWheelAttachmentAPI.cpp","wrapPhysxVehicleWheelControllerAPI.cpp","wrapTokens.cpp"
}

local python_module_files = {
    "__init__.py"
}

local resource_files = {
    "generatedSchema.usda","plugInfo.json","schema.usda"
}

local python_options = { python_root = root.."/_build/target-deps/python" }

-- USD plugin C++ project
project("physxSchema")
    -- standard USD plugin settings
    usd_plugin.usd_plugin("physxSchema", options, public_headers, private_headers, cpp_files, resource_files, usd_libs)
    usd_plugin.use_standard_usd_options()
    do_usd_zcinline_fix()
    link_boost_for_windows_wdefault()
    -- Manually add Python support (headers + runtime lib) without the USD-specific python lib
    usd_base.use_python(python_options)
    -- Force-include nopy_compat.h: saturates the pxr.h include guard and un-defines
    -- PXR_PYTHON_SUPPORT_ENABLED so that no pxr_boost::python symbols appear in physxSchema.dll's
    -- import table.  This makes the DLL binary-compatible with both usd.py312 and usd.nopy.
    -- The _physxSchema Python extension is compiled separately and is unaffected.
    -- premake maps forceincludes to /FI on MSVC and -include on GCC/Clang.
    forceincludes { root.."/source/nopy_compat.h" }
    filter { "system:windows" }
        -- C4251: USD template members (TfSingleton::_instance, TsSplineSamples::sources) lack
        -- dll-interface. These are harmless — the classes are correctly exported in the USD DLL.
        -- /external:W0 suppresses them for angle-bracket USD includes, but force-included files
        -- are exempt from /experimental:external treatment in VS2019, so we disable explicitly.
        disablewarnings { "4251" }
    filter {}
    filter { "system:linux"}
        buildoptions { "-fvisibility=default" }
    filter { "system:linux", "configurations:debug" }
        libdirs { options.usd_root .. "/lib"}
        links { "tbb_debug" }

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
    -- Manually add Python support (headers + runtime lib) without the USD-specific python lib
    usd_base.use_python(python_options)
    -- For non-monolithic USD (namespaced USD 25.x), pxr_boost::python lives in usd_python.
    -- For monolithic USD (*usd_ms), those symbols are already inside the monolithic lib.
    link_usd_python_if_needed()
    filter { "system:linux", "configurations:debug" }
        libdirs { options.usd_root .. "/lib"}
        links { "tbb_debug" }

end