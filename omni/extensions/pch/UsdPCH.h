// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

// !!! DO NOT INCLUDE THIS FILE IN A HEADER !!!
// When you include this file in a cpp file, add the file name to premake5.lua's pchFiles list!

// The usd headers drag in heavy dependencies and are very slow to build.
// Make it PCH to speed up building time.

#ifdef _MSC_VER
#    pragma warning(push)
#    pragma warning(disable : 4003) // not enough arguments for function-like macro invocation
#    pragma warning(disable : 4005) // NOMINMAX macro redefinition
#    pragma warning(disable : 4244) // = Conversion from double to float / int to float
#    pragma warning(disable : 4267) // conversion from size_t to int
#    pragma warning(disable : 4305) // argument truncation from double to float
#    pragma warning(disable : 4800) // int to bool
#    pragma warning(disable : 4996) // call to std::copy with parameters that may be unsafe
#    define NOMINMAX // Make sure nobody #defines min or max
#    include <Windows.h> // Include this here so we can curate
#    undef small // defined in rpcndr.h
#elif defined(__GNUC__)
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#    pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#    pragma GCC diagnostic ignored "-Wunused-function"
// This suppresses deprecated header warnings, which is impossible with pragmas.
// Alternative is to specify -Wno-deprecated build option, but that disables other useful warnings too.
#    ifdef __DEPRECATED
#        define OMNI_USD_SUPPRESS_DEPRECATION_WARNINGS
#        undef __DEPRECATED
#    endif
#endif

// Include cstdio here so that vsnprintf is properly declared. This is necessary because pyerrors.h has
// #define vsnprintf _vsnprintf which later causes <cstdio> to declare std::_vsnprintf instead of the correct and proper
// std::vsnprintf. By doing it here before everything else, we avoid this nonsense.
#include <cstdio>

// Python must be included first because it monkeys with macros that cause
// TBB to fail to compile in debug mode if TBB is included before Python
#include <boost/python/object.hpp>
#include <pxr/base/arch/stackTrace.h>
#include <pxr/base/arch/threads.h>
#include <pxr/base/gf/api.h>
#include <pxr/base/gf/camera.h>
#include <pxr/base/gf/frustum.h>
#include <pxr/base/gf/matrix3f.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/matrix4f.h>
#include <pxr/base/gf/quaternion.h>
#include <pxr/base/gf/rotation.h>
#include <pxr/base/gf/transform.h>
#include <pxr/base/gf/vec2f.h>
#include <pxr/base/tf/staticTokens.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/trace/reporter.h>
#include <pxr/base/trace/trace.h>
#include <pxr/base/vt/value.h>
#include <pxr/base/work/loops.h>
#include <pxr/base/work/threadLimits.h>
#include <pxr/imaging/hd/camera.h>
#include <pxr/imaging/hd/engine.h>
#include <pxr/imaging/hd/extComputation.h>
#include <pxr/imaging/hd/flatNormals.h>
#include <pxr/imaging/hd/instancer.h>
#include <pxr/imaging/hd/light.h>
#include <pxr/imaging/hd/material.h>
#include <pxr/imaging/hd/mesh.h>
#include <pxr/imaging/hd/meshUtil.h>
#include <pxr/imaging/hd/renderBuffer.h>
#include <pxr/imaging/hd/renderIndex.h>
#include <pxr/imaging/hd/renderPass.h>
#include <pxr/imaging/hd/renderPassState.h>
#include <pxr/imaging/hd/rendererPluginRegistry.h>
#include <pxr/imaging/hd/resourceRegistry.h>
#include <pxr/imaging/hd/rprim.h>
#include <pxr/imaging/hd/smoothNormals.h>
#include <pxr/imaging/hd/sprim.h>
#include <pxr/imaging/hd/vertexAdjacency.h>
#include <pxr/imaging/hdx/tokens.h>
#include <pxr/imaging/pxOsd/tokens.h>
#include <pxr/usd/ar/resolver.h>
#include <pxr/usd/ar/resolverContext.h>
#include <pxr/usd/ar/resolverContextBinder.h>
#include <pxr/usd/kind/registry.h>
#include <pxr/usd/pcp/layerStack.h>
#include <pxr/usd/pcp/site.h>
#include <pxr/usd/sdf/attributeSpec.h>
#include <pxr/usd/sdf/copyUtils.h>
#include <pxr/usd/sdf/fileFormat.h>
#include <pxr/usd/sdf/layerStateDelegate.h>
#include <pxr/usd/sdf/layerUtils.h>
#include <pxr/usd/sdf/relationshipSpec.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/editContext.h>
#include <pxr/usd/usd/modelAPI.h>
#include <pxr/usd/usd/notice.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/relationship.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/stageCache.h>
#include <pxr/usd/usd/usdFileFormat.h>
#include <pxr/usd/usdGeom/camera.h>
#include <pxr/usd/usdGeom/capsule.h>
#include <pxr/usd/usdGeom/cone.h>
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/basisCurves.h>
#include <pxr/usd/usdGeom/cylinder.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/plane.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdGeom/scope.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/subset.h>
#include <pxr/usd/usdGeom/tetMesh.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/xformCommonAPI.h>
#include <pxr/usd/usdGeom/points.h>
#include <pxr/usd/usdLux/cylinderLight.h>
#include <pxr/usd/usdLux/diskLight.h>
#include <pxr/usd/usdLux/distantLight.h>
#include <pxr/usd/usdLux/domeLight.h>
#include <pxr/usd/usdLux/rectLight.h>
#include <pxr/usd/usdLux/sphereLight.h>
#include <pxr/usd/usdLux/tokens.h>
#include <pxr/usd/usdShade/tokens.h>
#include <pxr/usd/usdSkel/animation.h>
#include <pxr/usd/usdSkel/tokens.h>
#include <pxr/usd/usdUtils/stageCache.h>
#include <pxr/usdImaging/usdImaging/delegate.h>


// -- Hydra

#include <pxr/imaging/hd/renderDelegate.h>
#include <pxr/imaging/hd/renderIndex.h>
#include <pxr/imaging/hd/rendererPlugin.h>
#include <pxr/imaging/hd/sceneDelegate.h>
#include <pxr/imaging/hd/tokens.h>
#include <pxr/imaging/hdx/taskController.h>
#include <pxr/usdImaging/usdImaging/gprimAdapter.h>
#include <pxr/usdImaging/usdImaging/indexProxy.h>
#include <pxr/usdImaging/usdImaging/tokens.h>

// physics
#include <pxr/usd/usdPhysics/articulationRootAPI.h>
#include <pxr/usd/usdPhysics/collisionAPI.h>
#include <pxr/usd/usdPhysics/meshCollisionAPI.h>
#include <pxr/usd/usdPhysics/collisionGroup.h>
#include <pxr/usd/usdPhysics/distanceJoint.h>
#include <pxr/usd/usdPhysics/driveAPI.h>
#include <pxr/usd/usdPhysics/filteredPairsAPI.h>
#include <pxr/usd/usdPhysics/joint.h>
#include <pxr/usd/usdPhysics/limitAPI.h>
#include <pxr/usd/usdPhysics/massAPI.h>
#include <pxr/usd/usdPhysics/metrics.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#include <pxr/usd/usdPhysics/materialAPI.h>
#include <pxr/usd/usdPhysics/scene.h>
#include <pxr/usd/usdPhysics/prismaticJoint.h>
#include <pxr/usd/usdPhysics/revoluteJoint.h>
#include <pxr/usd/usdPhysics/sphericalJoint.h>
#include <pxr/usd/usdPhysics/fixedJoint.h>
#include <physxSchema/physxConvexHullCollisionAPI.h>
#include <physxSchema/physxConvexDecompositionCollisionAPI.h>
#include <physxSchema/physxTriangleMeshSimplificationCollisionAPI.h>
#include <physxSchema/physxTriangleMeshCollisionAPI.h>
#include <physxSchema/physxSDFMeshCollisionAPI.h>
#include <physxSchema/physxSphereFillCollisionAPI.h>
#include <physxSchema/physxParticleClothAPI.h>
#include <physxSchema/physxAutoParticleClothAPI.h>
#include <physxSchema/physxParticleSamplingAPI.h>
#include <physxSchema/physxDiffuseParticlesAPI.h>
#include <physxSchema/physxParticleSmoothingAPI.h>
#include <physxSchema/physxParticleAnisotropyAPI.h>
#include <physxSchema/physxParticleIsosurfaceAPI.h>
#include <physxSchema/physxSceneAPI.h>
#include <physxSchema/physxSceneQuasistaticAPI.h>
#include "physxSchema/physxResidualReportingAPI.h"
#include <physxSchema/physxMeshMergeCollisionAPI.h>
#include <physxSchema/physxTriggerAPI.h>
#include <physxSchema/physxTriggerStateAPI.h>
#include <physxSchema/physxLimitAPI.h>
#include <physxSchema/physxCookedDataAPI.h>
#include <physxSchema/physxParticleAPI.h>
#include <physxSchema/physxParticleSetAPI.h>
#include <physxSchema/physxPhysicsAttachment.h>
#include <physxSchema/physxAutoAttachmentAPI.h>
#include <physxSchema/physxParticleSystem.h>
#include <physxSchema/physxRigidBodyAPI.h>
#include <physxSchema/physxSurfaceVelocityAPI.h>
#include <physxSchema/physxCollisionAPI.h>
#include <physxSchema/physxMaterialAPI.h>
#include <physxSchema/physxArticulationAPI.h>
#include <physxSchema/physxJointAPI.h>
#include <physxSchema/physxForceAPI.h>
#include <physxSchema/jointStateAPI.h>
#include <physxSchema/physxCharacterControllerAPI.h>
#include <physxSchema/physxContactReportAPI.h>
#include <physxSchema/physxDeformableBodyMaterialAPI.h>
#include <physxSchema/physxDeformableSurfaceMaterialAPI.h>
#include <physxSchema/physxPBDMaterialAPI.h>
#include <physxSchema/physxDeformableAPI.h>
#include <physxSchema/physxDeformableBodyAPI.h>
#include <physxSchema/physxDeformableSurfaceAPI.h>
#include <physxSchema/physxVehicleAPI.h>
#include <physxSchema/physxVehicleAutoGearBoxAPI.h>
#include <physxSchema/physxVehicleBrakesAPI.h>
#include <physxSchema/physxVehicleClutchAPI.h>
#include <physxSchema/physxVehicleControllerAPI.h>
#include <physxSchema/physxVehicleTankControllerAPI.h>
#include <physxSchema/physxVehicleDriveBasicAPI.h>
#include <physxSchema/physxVehicleDriveStandardAPI.h>
#include <physxSchema/physxVehicleEngineAPI.h>
#include <physxSchema/physxVehicleGearsAPI.h>
#include <physxSchema/physxVehicleContextAPI.h>
#include <physxSchema/physxVehicleMultiWheelDifferentialAPI.h>
#include <physxSchema/physxVehicleTankDifferentialAPI.h>
#include <physxSchema/physxVehicleNonlinearCommandResponseAPI.h>
#include <physxSchema/physxVehicleSteeringAPI.h>
#include <physxSchema/physxVehicleAckermannSteeringAPI.h>
#include <physxSchema/physxVehicleSuspensionAPI.h>
#include <physxSchema/physxVehicleSuspensionComplianceAPI.h>
#include <physxSchema/physxVehicleTireAPI.h>
#include <physxSchema/physxVehicleTireFrictionTable.h>
#include <physxSchema/physxVehicleWheelAPI.h>
#include <physxSchema/physxVehicleWheelAttachmentAPI.h>
#include <physxSchema/physxVehicleWheelControllerAPI.h>
#include <physxSchema/physxCameraAPI.h>
#include <physxSchema/physxCameraFollowAPI.h>
#include <physxSchema/physxCameraFollowLookAPI.h>
#include <physxSchema/physxCameraFollowVelocityAPI.h>
#include <physxSchema/physxCameraDroneAPI.h>
#include <physxSchema/physxPhysicsGearJoint.h>
#include <physxSchema/physxPhysicsRackAndPinionJoint.h>
#include <physxSchema/physxPhysicsJointInstancer.h>
#include <physxSchema/physxPhysicsDistanceJointAPI.h>
#include <physxSchema/physxTendonAxisAPI.h>
#include <physxSchema/physxTendonAxisRootAPI.h>
#include <physxSchema/physxTendonAttachmentAPI.h>
#include <physxSchema/physxTendonAttachmentRootAPI.h>
#include <physxSchema/physxTendonAttachmentLeafAPI.h>
#include <physxSchema/physxMimicJointAPI.h>
#include <physicsSchemaTools/UsdTools.h>
#include <physicsSchemaTools/physicsSchemaTokens.h>


#ifdef _MSC_VER
#    pragma warning(pop)
#elif defined(__GNUC__)
#    pragma GCC diagnostic pop
#    ifdef OMNI_USD_SUPPRESS_DEPRECATION_WARNINGS
#        define __DEPRECATED
#        undef OMNI_USD_SUPPRESS_DEPRECATION_WARNINGS
#    endif
#endif
