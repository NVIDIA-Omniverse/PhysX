// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-SCAN-001
 * @covers AC-1 AC-14 AC-15 AC-16 AC-17 AC-18
 *
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-24
 *
 * @implements REQ-PARSE-CCT-001
 * @covers AC-5
 *
 * @implements REQ-PARSE-MAT-001
 * @covers AC-6
 *
 * @implements REQ-PUBLICAPI-003
 * @covers AC-4
 *
 * @implements REQ-PARSE-SHAPE-004
 * @covers AC-1
 *
 * @implements REQ-PARSE-JOINT-005
 * @covers AC-3 AC-4
 *
 * @implements REQ-PARSE-FEED-003
 * @covers AC-10 AC-12 AC-13 AC-14
 *
 * @implements REQ-PARSE-COL-005
 * @covers AC-1
 */

#include "OvstageWalker.h"

#include "OvstageSource.h"



#include <carb/extras/ScopeExit.h>

#include <omni/physics/parse/ArticulationGraph.h>
#include <omni/physics/parse/CustomTokens.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>
#include <private/omni/physics/CollisionShapeTransform.h>
#include <private/omni/physics/JointFrameTransform.h>

#include <foundation/PxMat44.h>

#include <algorithm>
#include <atomic>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace omni::physics::ovstage
{
namespace
{
ovx_string_t ovxStr(const char* s)
{
    return { s, std::string_view(s).size() };
}

// Test-only count of enumerate()'s data-column reads: a family with no matching prim must not
// pay one to materialise zero prims.
std::atomic_size_t& enumerateReadCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

// Test-only count of enumerate()'s whole-stage filter queries: a prim-type family the source's
// type index knows to be empty must not pay one.
std::atomic_size_t& enumerateQueryCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

// Test-only: the next N batched probe reads in enumerate() fail, so the per-column fallback runs.
// Zero in production; set only by setOvstageWalkerEnumerateReadFaultForTest().
std::atomic_int& enumerateReadFaultCounter()
{
    static std::atomic_int counter{ 0 };
    return counter;
}

bool consumeEnumerateReadFault()
{
    int n = enumerateReadFaultCounter().load(std::memory_order_relaxed);
    while (n > 0)
    {
        if (enumerateReadFaultCounter().compare_exchange_weak(n, n - 1, std::memory_order_relaxed))
            return true;
    }
    return false;
}

// parse::Matrix4d is sixteen doubles in the layout PxMat44d/GfMatrix4d both use;
// this is an element copy, not a transpose. Only the *product* order differs,
// see common/foundation/MatrixTools.h.
::physx::PxMat44d toPxMat44d(const Matrix4d& matrix)
{
    double values[16];
    for (int i = 0; i < 16; ++i)
        values[i] = matrix.data[i];
    return ::physx::PxMat44d(values);
}

// This local helper reports completion for strict schema reads; OvstageChangeFeed keeps its lenient copy.
bool waitAndRelease(ovstage_instance_t* inst, ovstage_enqueue_result_t result) noexcept
{
    if (result.status != OVSTAGE_OK || result.op_index == OVSTAGE_INVALID_OP_ID)
        return false;

    const ovstage_api_status_t waitStatus =
        ovstage_wait_op(inst, result.op_index, OVSTAGE_TIMEOUT_INFINITE, nullptr);
    if (waitStatus == OVSTAGE_OK || waitStatus == OVSTAGE_ERROR_OP_FAILED)
        (void)ovstage_release_op(inst, result.op_index);
    return waitStatus == OVSTAGE_OK;
}

[[noreturn]] void throwDataError(ovstage_api_status_t status,
                                 const char* operation,
                                 ovstage_ordinal_t readOrdinal)
{
    throw std::runtime_error("OVStage schema scan failed during " + std::string(operation) +
                             " at read ordinal " + std::to_string(readOrdinal) +
                             " (status " + std::to_string(static_cast<int>(status)) + ")");
}

// A concept's matched prims plus the attribute columns discovered on them — the
// latter feeds the columnar bulk read (prefetchBucket) so the per-concept parse
// loop reads from a cache rather than a per-prim round trip.
struct Bucket
{
    std::vector<ObjectKey> keys;
    std::vector<std::string> attrs; // discoverable data columns (no metadata / computed cols)
};

void appendBucketAttr(Bucket& bucket, const char* attr)
{
    if (!attr || attr[0] == '\0')
        return;
    if (std::find(bucket.attrs.begin(), bucket.attrs.end(), attr) == bucket.attrs.end())
        bucket.attrs.emplace_back(attr);
}

void appendBucketAttr(Bucket& bucket, const std::string& attr)
{
    if (attr.empty())
        return;
    if (std::find(bucket.attrs.begin(), bucket.attrs.end(), attr) == bucket.attrs.end())
        bucket.attrs.push_back(attr);
}

void appendBucketAttrs(Bucket& bucket, std::initializer_list<const char*> attrs)
{
    for (const char* attr : attrs)
        appendBucketAttr(bucket, attr);
}

std::vector<std::string> appendedAttrs(const Bucket& bucket, size_t previousSize)
{
    if (previousSize >= bucket.attrs.size())
        return {};
    return std::vector<std::string>(bucket.attrs.begin() + previousSize, bucket.attrs.end());
}

// Drop appended columns [firstAppended, end) of hasSchema-gated joint sub-schema families
// that no matched prim authors (absent from `discovered`): parseJoint reads them only behind
// a hasSchema gate that is false when the schema is unapplied. Unconditionally read columns
// carry none of these prefixes. Applied-but-all-default falls back to a live per-prim read.
void trimUnauthoredGatedJointAttrs(Bucket& bucket,
                                   size_t firstAppended,
                                   const std::vector<std::string>& discovered)
{
    if (firstAppended >= bucket.attrs.size())
        return;
    static const std::string_view kGatedPrefixes[] = {
        "physxLimit:", "physxJointAxis:", "drive:", "limit:", "state:",
        "physxDrivePerformanceEnvelope:", "physxPhysicsDistanceJoint:",
    };
    const std::unordered_set<std::string_view> authored(discovered.begin(), discovered.end());
    auto isGated = [](std::string_view name) {
        for (const std::string_view pfx : kGatedPrefixes)
            if (name.size() > pfx.size() && name.compare(0, pfx.size(), pfx) == 0)
                return true;
        return false;
    };
    std::vector<std::string>& attrs = bucket.attrs;
    attrs.erase(std::remove_if(attrs.begin() + firstAppended, attrs.end(),
                               [&](const std::string& name) {
                                   return isGated(name) && authored.count(name) == 0;
                               }),
                attrs.end());
}

void appendTransformAttrs(Bucket& bucket)
{
    appendBucketAttrs(bucket, {
        conv::kFabricWorldMatrix,
        conv::kFabricLocalMatrix,
        conv::kLocalTransform,
        conv::kResetXformStack,
    });
}

void prefetchTransformAncestors(OvstageSource& src, const std::vector<ObjectKey>& keys)
{
    Bucket bucket;
    bucket.keys = src.collectAncestors(keys);
    if (bucket.keys.empty())
        return;
    appendTransformAttrs(bucket);
    src.prefetchBucket(bucket.keys, bucket.attrs);
}

void prefetchTransformsForKeys(OvstageSource& src, const std::vector<ObjectKey>& keys)
{
    Bucket bucket;
    bucket.keys = keys;
    if (bucket.keys.empty())
        return;
    appendTransformAttrs(bucket);
    src.prefetchBucket(bucket.keys, bucket.attrs);
}

void appendSceneAttrs(Bucket& bucket)
{
    appendBucketAttrs(bucket, {
        "physics:gravityDirection",
        "physics:gravityMagnitude",
        "physxScene:updateType",
        "physxScene:bounceThreshold",
        "physxScene:frictionOffsetThreshold",
        "physxScene:frictionCorrelationDistance",
        "physxScene:maxBiasCoefficient",
        "physxScene:timeStepsPerSecond",
        "physxScene:minPositionIterationCount",
        "physxScene:maxPositionIterationCount",
        "physxScene:minVelocityIterationCount",
        "physxScene:maxVelocityIterationCount",
        "physxScene:enableCCD",
        "physxScene:enableStabilization",
        "physxScene:enableGPUDynamics",
        "physxScene:enableEnhancedDeterminism",
        "physxScene:enableExternalForcesEveryIteration",
        "physxScene:invertCollisionGroupFilter",
        "physxScene:reportKinematicKinematicPairs",
        "physxScene:reportKinematicStaticPairs",
        "physxScene:enableSceneQuerySupport",
        "physxScene:solveArticulationContactLast",
        "physxScene:disableSleeping",
        "physxScene:collisionSystem",
        "physxScene:solverType",
        "physxScene:broadphaseType",
        "physxScene:frictionType",
        "physxScene:gpuTempBufferCapacity",
        "physxScene:gpuMaxRigidContactCount",
        "physxScene:gpuMaxRigidPatchCount",
        "physxScene:gpuHeapCapacity",
        "physxScene:gpuFoundLostPairsCapacity",
        "physxScene:gpuFoundLostAggregatePairsCapacity",
        "physxScene:gpuTotalAggregatePairsCapacity",
        "physxScene:gpuMaxDeformableVolumeContacts",
        "physxScene:gpuMaxDeformableSurfaceContacts",
        "physxScene:gpuMaxParticleContacts",
        "physxScene:gpuCollisionStackSize",
        "physxScene:gpuMaxNumPartitions",
        "physxScene:envIdInBoundsBitCount",
        "physxSceneQuasistatic:enableQuasistatic",
        "newton:timeStepsPerSecond",
        "newton:gravityEnabled",
    });
}

void appendMaterialAttrs(Bucket& bucket)
{
    appendBucketAttrs(bucket, {
        "physics:staticFriction",
        "physics:dynamicFriction",
        "physics:restitution",
        "physics:density",
        "physxMaterial:frictionCombineMode",
        "physxMaterial:restitutionCombineMode",
        "physxMaterial:dampingCombineMode",
        "physxMaterial:compliantContactAccelerationSpring",
        "physxMaterial:compliantContactStiffness",
        "physxMaterial:compliantContactDamping",
    });
}

void appendRigidBodyAttrs(Bucket& bucket)
{
    appendTransformAttrs(bucket);
    appendBucketAttrs(bucket, {
        "physics:rigidBodyEnabled",
        "physics:kinematicEnabled",
        "physics:startsAsleep",
        "physics:velocity",
        "physics:angularVelocity",
        "physics:localSpaceVelocities",
        "physics:simulationOwner",
        "physics:filteredPairs",
        "physxRigidBody:linearDamping",
        "physxRigidBody:angularDamping",
        "physxRigidBody:maxLinearVelocity",
        "physxRigidBody:maxAngularVelocity",
        "physxRigidBody:sleepThreshold",
        "physxRigidBody:stabilizationThreshold",
        "physxRigidBody:maxDepenetrationVelocity",
        "physxRigidBody:contactSlopCoefficient",
        "physxRigidBody:maxContactImpulse",
        "physxRigidBody:cfmScale",
        "physxRigidBody:solverPositionIterationCount",
        "physxRigidBody:solverVelocityIterationCount",
        "physxRigidBody:enableCCD",
        "physxRigidBody:enableSpeculativeCCD",
        "physxRigidBody:disableGravity",
        "physxRigidBody:retainAccelerations",
        "physxRigidBody:enableGyroscopicForces",
        "physxRigidBody:solveContact",
        "physxRigidBody:lockedPosAxis",
        "physxRigidBody:lockedRotAxis",
        "physxSurfaceVelocity:surfaceVelocityEnabled",
        "physxSurfaceVelocity:surfaceVelocityLocalSpace",
        "physxSurfaceVelocity:surfaceVelocity",
        "physxSurfaceVelocity:surfaceAngularVelocity",
        "physxSplinesSurfaceVelocity:surfaceVelocityEnabled",
        "physxSplinesSurfaceVelocity:surfaceVelocityMagnitude",
        "physxSplinesSurfaceVelocity:surfaceVelocityCurve",
    });
}

void appendMassAttrs(Bucket& bucket)
{
    appendTransformAttrs(bucket);
    appendBucketAttrs(bucket, {
        "physics:mass",
        "physics:density",
        "physics:centerOfMass",
        "physics:diagonalInertia",
        "physics:principalAxes",
    });
}

void appendCollisionShapeAttrs(Bucket& bucket)
{
    appendTransformAttrs(bucket);
    appendBucketAttrs(bucket, {
        "physics:collisionEnabled",
        "physics:simulationOwner",
        "physics:filteredPairs",
        "physics:approximation",
        "material:binding:physics",
        "material:binding",
        "size",
        "radius",
        "height",
        "axis",
        "doubleSided",
        "physxCollision:torsionalPatchRadius",
        "physxCollision:minTorsionalPatchRadius",
        "physxCollision:contactOffset",
        "physxCollision:restOffset",
        "physxConvexGeometry:margin",
        "physxConvexHullCollision:hullVertexLimit",
        "physxConvexHullCollision:minThickness",
        "physxConvexDecompositionCollision:minThickness",
        "physxConvexDecompositionCollision:maxConvexHulls",
        "physxConvexDecompositionCollision:hullVertexLimit",
        "physxConvexDecompositionCollision:voxelResolution",
        "physxConvexDecompositionCollision:errorPercentage",
        "physxConvexDecompositionCollision:shrinkWrap",
        "physxSphereFillCollision:maxSpheres",
        "physxSphereFillCollision:seedCount",
        "physxSphereFillCollision:voxelResolution",
        "physxSphereFillCollision:fillMode",
        "physxTriangleMeshCollision:weldTolerance",
        "physxTriangleMeshSimplificationCollision:metric",
        "physxTriangleMeshSimplificationCollision:weldTolerance",
        "physxSDFMeshCollision:sdfResolution",
        "physxSDFMeshCollision:sdfSubgridResolution",
        "physxSDFMeshCollision:sdfBitsPerSubgridPixel",
        "physxSDFMeshCollision:sdfNarrowBandThickness",
        "physxSDFMeshCollision:sdfMargin",
        "physxSDFMeshCollision:sdfEnableRemeshing",
        "physxSDFMeshCollision:sdfTriangleCountReductionFactor",
        "newton:maxHullVertices",
        "newton:contactMargin",
        "newton:contactGap",
    });
}

void appendDeformableBodyAttrs(Bucket& bucket)
{
    appendTransformAttrs(bucket);
    appendBucketAttrs(bucket, {
        "omniphysics:deformableBodyEnabled",
        "omniphysics:mass",
        "omniphysics:kinematicEnabled",
        "omniphysics:startsAsleep",
        "omniphysics:simulationOwner",
        "physics:filteredPairs",
        "physxDeformableBody:linearDamping",
        "physxDeformableBody:maxLinearVelocity",
        "physxDeformableBody:sleepThreshold",
        "physxDeformableBody:settlingThreshold",
        "physxDeformableBody:settlingDamping",
        "physxDeformableBody:maxDepenetrationVelocity",
        "physxDeformableBody:selfCollisionFilterDistance",
        "physxDeformableBody:solverPositionIterationCount",
        "physxDeformableBody:enableSpeculativeCCD",
        "physxDeformableBody:selfCollision",
        "physxDeformableBody:disableGravity",
        "physxDeformableBody:collisionPairUpdateFrequency",
        "physxDeformableBody:collisionIterationMultiplier",
        "physxDeformableBody:autoDeformableBodyEnabled",
        "physxDeformableBody:cookingSourceMesh",
        "physxDeformableBody:autoDeformableMeshSimplificationEnabled",
        "physxDeformableBody:remeshingEnabled",
        "physxDeformableBody:remeshingResolution",
        "physxDeformableBody:targetTriangleCount",
        "physxDeformableBody:forceConforming",
        "physxDeformableBody:resolution",
    });
}

void appendCollisionGroupAttrs(Bucket& bucket)
{
    appendBucketAttrs(bucket, {
        "physics:filteredGroups",
        "physics:invertFilteredGroups",
        "collection:colliders:includes",
        "collection:colliders:excludes",
    });
}

void appendArticulationAttrs(Bucket& bucket)
{
    appendTransformAttrs(bucket);
    appendBucketAttrs(bucket, {
        "physics:filteredPairs",
        "physxArticulation:articulationEnabled",
        "physxArticulation:sleepThreshold",
        "physxArticulation:stabilizationThreshold",
        "physxArticulation:solverPositionIterationCount",
        "physxArticulation:solverVelocityIterationCount",
        "physxArticulation:enabledSelfCollisions",
        "newton:selfCollisionEnabled",
    });
}

void appendJointCommonAttrs(Bucket& bucket)
{
    appendBucketAttrs(bucket, {
        "physics:jointEnabled",
        "physics:collisionEnabled",
        "physics:excludeFromArticulation",
        "physics:breakForce",
        "physics:breakTorque",
        "physics:body0",
        "physics:body1",
        "physics:localPos0",
        "physics:localRot0",
        "physics:localPos1",
        "physics:localRot1",
        "physxJoint:jointFriction",
    });
}

void appendPhysxJointAxisAttrs(Bucket& bucket, const char* axis)
{
    const std::string axisBase = std::string("physxJointAxis:") + axis + ":";
    appendBucketAttr(bucket, axisBase + "armature");
    appendBucketAttr(bucket, axisBase + "maxJointVelocity");
    appendBucketAttr(bucket, axisBase + "staticFrictionEffort");
    appendBucketAttr(bucket, axisBase + "dynamicFrictionEffort");
    appendBucketAttr(bucket, axisBase + "viscousFrictionCoefficient");

    appendBucketAttrs(bucket, {
        "physxJoint:armature",
        "physxJoint:maxJointVelocity",
        // Seeds maxJointVelocity before the PhysX reads (ParseJoint.cpp's
        // readPhysxJointAxisApi), so it is read once per joint per axis.
        "newton:velocityLimit",
    });
}

void appendPhysxLimitAttrs(Bucket& bucket, const char* axis)
{
    const std::string limitBase = std::string("physxLimit:") + axis + ":";
    appendBucketAttr(bucket, limitBase + "restitution");
    appendBucketAttr(bucket, limitBase + "bounceThreshold");
    appendBucketAttr(bucket, limitBase + "stiffness");
    appendBucketAttr(bucket, limitBase + "damping");
}

void appendPhysicsLimitAttrs(Bucket& bucket, const char* axis)
{
    const std::string limitBase = std::string("limit:") + axis + ":physics:";
    appendBucketAttr(bucket, limitBase + "low");
    appendBucketAttr(bucket, limitBase + "high");
    appendPhysxLimitAttrs(bucket, axis);
}

void appendPhysicsDriveAttrs(Bucket& bucket, const char* axis)
{
    const std::string driveBase = std::string("drive:") + axis + ":physics:";
    appendBucketAttr(bucket, driveBase + "stiffness");
    appendBucketAttr(bucket, driveBase + "damping");
    appendBucketAttr(bucket, driveBase + "targetPosition");
    appendBucketAttr(bucket, driveBase + "targetVelocity");
    appendBucketAttr(bucket, driveBase + "maxForce");
    appendBucketAttr(bucket, driveBase + "type");
}

void appendPhysxDrivePerformanceEnvelopeAttrs(Bucket& bucket, const char* axis)
{
    const std::string envelopeBase = std::string("physxDrivePerformanceEnvelope:") + axis + ":";
    appendBucketAttr(bucket, envelopeBase + "maxActuatorVelocity");
    appendBucketAttr(bucket, envelopeBase + "velocityDependentResistance");
    appendBucketAttr(bucket, envelopeBase + "speedEffortGradient");
}

void appendPhysicsJointStateAttrs(Bucket& bucket, const char* axis)
{
    const std::string stateBase = std::string("state:") + axis + ":physics:";
    appendBucketAttr(bucket, stateBase + "position");
    appendBucketAttr(bucket, stateBase + "velocity");
}

void appendNewtonMimicAttrs(Bucket& bucket)
{
    appendBucketAttrs(bucket, {
        "newton:mimicJoint",
    });
}

void appendJointAttrs(Bucket& bucket, ObjectType type)
{
    appendJointCommonAttrs(bucket);

    switch (type)
    {
    case eJointFixed:
        appendNewtonMimicAttrs(bucket);
        break;

    case eJointSpherical:
        appendBucketAttrs(bucket, {
            "physics:axis",
            "physics:coneAngle0Limit",
            "physics:coneAngle1Limit",
        });
        appendPhysxLimitAttrs(bucket, "cone");
        appendPhysxJointAxisAttrs(bucket, "rotX");
        appendPhysxJointAxisAttrs(bucket, "rotY");
        appendPhysxJointAxisAttrs(bucket, "rotZ");
        appendNewtonMimicAttrs(bucket);
        break;

    case eJointRevolute:
        appendBucketAttrs(bucket, {
            "physics:axis",
            "physics:lowerLimit",
            "physics:upperLimit",
        });
        appendPhysxLimitAttrs(bucket, "angular");
        appendPhysxJointAxisAttrs(bucket, "angular");
        appendPhysicsDriveAttrs(bucket, "angular");
        appendPhysxDrivePerformanceEnvelopeAttrs(bucket, "angular");
        appendPhysicsJointStateAttrs(bucket, "angular");
        appendNewtonMimicAttrs(bucket);
        break;

    case eJointPrismatic:
        appendBucketAttrs(bucket, {
            "physics:axis",
            "physics:lowerLimit",
            "physics:upperLimit",
        });
        appendPhysxLimitAttrs(bucket, "linear");
        appendPhysxJointAxisAttrs(bucket, "linear");
        appendPhysicsDriveAttrs(bucket, "linear");
        appendPhysxDrivePerformanceEnvelopeAttrs(bucket, "linear");
        appendPhysicsJointStateAttrs(bucket, "linear");
        appendNewtonMimicAttrs(bucket);
        break;

    case eJointDistance:
        appendBucketAttrs(bucket, {
            "physics:minDistance",
            "physics:maxDistance",
            "physxPhysicsDistanceJoint:springEnabled",
            "physxPhysicsDistanceJoint:springStiffness",
            "physxPhysicsDistanceJoint:springDamping",
        });
        appendPhysxLimitAttrs(bucket, "distance");
        appendNewtonMimicAttrs(bucket);
        break;

    case eJointD6:
    {
        static const char* const kD6Axes[] = { "transX", "transY", "transZ", "rotX", "rotY", "rotZ", "distance" };
        for (const char* axis : kD6Axes)
        {
            appendPhysicsLimitAttrs(bucket, axis);
            appendPhysicsDriveAttrs(bucket, axis);
        }

        static const char* const kD6RotAxes[] = { "rotX", "rotY", "rotZ" };
        for (const char* axis : kD6RotAxes)
        {
            appendPhysxJointAxisAttrs(bucket, axis);
            appendPhysxDrivePerformanceEnvelopeAttrs(bucket, axis);
        }

        static const char* const kD6StateAxes[] = { "transX", "transY", "transZ", "rotX", "rotY", "rotZ" };
        for (const char* axis : kD6StateAxes)
            appendPhysicsJointStateAttrs(bucket, axis);

        appendNewtonMimicAttrs(bucket);
        break;
    }

    case eJointGear:
        appendBucketAttrs(bucket, {
            "physics:gearRatio",
            "physics:hinge0",
            "physics:hinge1",
        });
        break;

    case eJointRackAndPinion:
        appendBucketAttrs(bucket, {
            "physics:ratio",
            "physics:hinge",
            "physics:prismatic",
        });
        break;

    default:
        break;
    }
}

// Enumerate the ObjectKeys of prims matching a single-predicate filter, plus the
// data attributes discovered on the matched set. `probeAttr` must be an attribute
// every matched prim carries: the read group's prims.list materialises the
// matched set (ovstage gives no path list on the query result itself).
Bucket enumerate(const OvstageSource& src,
                 ovstage_instance_t* inst,
                 ovx_path_dictionary_t* dict,
                 const char* predAttr,
                 ovstage_filter_op_t op,
                 const char* predValue,
                 const char* probeAttr,
                 ovstage_ordinal_t readOrdinal,
                 const char* metadataProbeAttr)
{
    Bucket out;
    std::vector<ObjectKey>& keys = out.keys;
    const bool requiresSchemaProbe = std::string_view(predAttr) == conv::kUsdSchemas;

    ovx_string_t value = ovxStr(predValue);
    ovstage_predicate_t pred{};
    pred.attribute.token = 0;
    pred.attribute.string = ovxStr(predAttr);
    pred.op = op;
    pred.values = &value;
    pred.value_count = 1;

    ovstage_filter_t filter{};
    filter.predicates = &pred;
    filter.count = 1;

    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    CARB_SCOPE_EXIT
    {
        if (q != OVSTAGE_INVALID_QUERY_HANDLE)
            waitAndRelease(inst, ovstage_release_query(inst, q));
    };
    enumerateQueryCounter().fetch_add(1, std::memory_order_relaxed);
    const ovstage_enqueue_result_t qe = ovstage_query(inst, &filter, nullptr, 0, &q);
    if (!waitAndRelease(inst, qe) || q == OVSTAGE_INVALID_QUERY_HANDLE)
    {
        if (requiresSchemaProbe)
            throwDataError(qe.status == OVSTAGE_OK ? OVSTAGE_ERROR_OP_FAILED : qe.status,
                           "ovstage_query", readOrdinal);
        return out;
    }

    // ovstage gives no prim path list on the query result itself — the matched
    // prims materialise only as a read group's prims.list. Rather than assume a
    // specific probe attribute exists (different data models author different
    // attributes — the hand populator vs ovpopulation), read the query result's
    // own *discovered* attributes: those are exactly the attributes present on the
    // matched set, so reading them back yields every matched prim. `probeAttr` is
    // a fallback when the result reports no attributes. Schema membership queries
    // are not ordinal-scoped, so `usd-schemas` itself is the required probe for a
    // schema predicate; reading an ordinary column would not prove that the schema
    // data is readable at `readOrdinal`.
    ovstage_query_handle_t use = q;
    std::vector<ovx_token_t> probes; // all usable data columns; their union covers the matched set
    ovstage_query_result_t qr{};
    const ovstage_api_status_t queryResultStatus =
        ovstage_fetch_query_result(inst, q, OVSTAGE_TIMEOUT_INFINITE, &qr);
    if (queryResultStatus != OVSTAGE_OK && requiresSchemaProbe)
        throwDataError(queryResultStatus, "ovstage_fetch_query_result", readOrdinal);
    bool noMatch = false;
    if (queryResultStatus == OVSTAGE_OK)
    {
        noMatch = qr.total_prim_count == 0;
        if (qr.all_handle != OVSTAGE_INVALID_QUERY_HANDLE)
            use = qr.all_handle;
        // Probe with attributes the matched prims actually carry: discovered
        // attributes (data models differ — hand populator vs ovpopulation — so a
        // hardcoded probe name is wrong). Skip fabric-internal computed columns
        // (leading '_', e.g. "_worldExtent"/"_worldVisibility"): they aren't
        // readable through the data-plane read, so probing them yields no prims.
        for (size_t i = 0; i < qr.attribute_count; ++i)
        {
            ovx_string_t s{};
            if (ovx_path_dictionary_token_to_string(dict, qr.attributes[i], &s) != OVX_OK || !s.ptr || s.length == 0)
                continue;
            if (s.ptr[0] == '_')
                continue;

            // Built-in usd-* metadata is not scalar parse data. Some metadata
            // columns are readable, though, so use the explicit metadata probe
            // only for enumeration fallback, not as a bulk-prefetch attribute.
            if (s.length >= 4 && std::strncmp(s.ptr, "usd-", 4) == 0)
                continue;
            if (!requiresSchemaProbe)
                probes.push_back(qr.attributes[i]); // each usable column probes + bulk-reads
            out.attrs.emplace_back(s.ptr, s.length);
        }
        (void)ovstage_release_query_result(inst, &qr);
    }
    // The query's own count decides an empty family: no fallback read to materialise zero
    // prims. Schema predicates keep their read, which doubles as the readability proof.
    if (noMatch && !requiresSchemaProbe)
        return out;
    if (requiresSchemaProbe || probes.empty())
    {
        // Schema predicates must read their metadata column. Other predicates
        // use the explicit metadata/probe column only when no ordinary data
        // column can materialize the matched prims.
        ovx_token_t fb = OVX_INVALID_TOKEN;
        const char* fallbackProbe = metadataProbeAttr ? metadataProbeAttr : probeAttr;
        if (ovx_path_dictionary_intern_token(dict, ovxStr(fallbackProbe), &fb) == OVX_OK &&
            fb != OVX_INVALID_TOKEN)
            probes.push_back(fb);
        else if (requiresSchemaProbe)
            throw std::runtime_error("OVStage schema scan could not intern required usd-schemas token");
    }

    ovstage_ordinal_range_t range{};
    range.end_ordinal = readOrdinal;
    range.has_start_ordinal = false;

    // Read EVERY usable column, in ONE multi-attribute read, and union the matched prim
    // lists. A single column is only authored on the prims that carry it, so one probe
    // under-counts the matched set: a read group's prims.list holds only the prims with that
    // column, dropping matched prims whose authored column was not the first one
    // picked (e.g. Sphere/Mesh/Cube colliders, particle systems). The discovered
    // columns are the union over the matched set, so every matched prim carries at
    // least one — reading them all and unioning (dedup by handle) yields the
    // complete set, which is what let us drop the USD-Traverse supplement in
    // OvstageSource::collectPrimTypeKeys. Handles come from the data-column read
    // (the populator's canonical primpaths) so downstream key lookups still match. The
    // groups come back tagged per column; only their prim lists matter here, so one round
    // trip serves every column. Should that batched read fail, the columns are read one by
    // one instead, so a single failing column drops only its own rows rather than emptying the
    // family (a schema predicate's failing column is still an error, as before).
    std::unordered_set<uint64_t> seen;
    // One read over `toks`, its groups' prims unioned into `keys`. False when the read itself
    // could not be issued (the batched caller then falls back per column).
    auto readProbeUnion = [&](const ovx_token_t* toks, size_t tokCount, bool batched) -> bool
    {
        if (batched && consumeEnumerateReadFault())
            return false;
        ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
        CARB_SCOPE_EXIT
        {
            if (rh != OVSTAGE_INVALID_READ_HANDLE)
                waitAndRelease(inst, ovstage_release_read(inst, rh));
        };
        enumerateReadCounter().fetch_add(1, std::memory_order_relaxed);
        const ovstage_enqueue_result_t re = ovstage_read_attributes(inst, use, toks, tokCount, range, &rh);
        if (!waitAndRelease(inst, re) || rh == OVSTAGE_INVALID_READ_HANDLE)
        {
            if (!batched && requiresSchemaProbe)
                throwDataError(re.status == OVSTAGE_OK ? OVSTAGE_ERROR_OP_FAILED : re.status,
                               "ovstage_read_attributes", readOrdinal);
            return false;
        }
        ovstage_read_group_t g{};
        // Union prims from EVERY read group, not just the first. A scalar column
        // comes back as a single group whose prims.list covers all owning prims,
        // but an array/relationship column (e.g. `collection:colliders:includes`,
        // `physics:filteredGroups`) is emitted one group PER owning prim — so a
        // first-group-only scan captured just one owner and dropped the rest (a
        // second collision group with only a ragged membership column went missing).
        // Dedup via `seen` keeps scalar reads idempotent.
        ovstage_api_status_t fetchStatus = OVSTAGE_OK;
        // A list is fetched and unioned once, however many groups (one per owning prim on an array
        // column) address it.
        ReadListMemo listMemo(dict);
        std::unordered_set<ovx_primpath_list_t> unioned;
        while ((fetchStatus = ovstage_fetch_read_next(inst, rh, OVSTAGE_TIMEOUT_INFINITE, &g)) == OVSTAGE_OK)
        {
            const ovx_primpath_t* paths = nullptr;
            size_t count = 0;
            if (listMemo.paths(g.prims.list, &paths, &count) && unioned.insert(g.prims.list).second)
                for (size_t i = 0; i < count; ++i)
                    if (seen.insert(paths[i]).second)
                        keys.push_back(src.internKey(paths[i]));
            ovstage_release_group(inst, &g);
        }
        if (fetchStatus != OVSTAGE_ERROR_END_OF_ITERATION && requiresSchemaProbe)
            throwDataError(fetchStatus, "ovstage_fetch_read_next", readOrdinal);
        return true;
    };
    if (!probes.empty() && !readProbeUnion(probes.data(), probes.size(), /*batched=*/true))
    {
        for (const ovx_token_t probe : probes)
            readProbeUnion(&probe, 1, /*batched=*/false);
    }
    return out;
}

// Quaternion (x,y,z,w) from the row-major, row-vector rotation matrix that
// IPhysicsSource::getLocalToWorldRotationAndScale returns. The row-vector form
// (v' = v R) is the transpose of the column-vector matrix M = R^T the standard
// trace-based extraction expects, so the transposed indices are fed in. Lets a
// bodyless collider's authored orientation survive into the shape's local pose.
carb::Float4 quatFromRotation(const Matrix3d& r)
{
    const double m00 = r.data[0], m01 = r.data[3], m02 = r.data[6];
    const double m10 = r.data[1], m11 = r.data[4], m12 = r.data[7];
    const double m20 = r.data[2], m21 = r.data[5], m22 = r.data[8];
    const double trace = m00 + m11 + m22;
    carb::Float4 q{ 0.0f, 0.0f, 0.0f, 1.0f };
    if (trace > 0.0)
    {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q.w = static_cast<float>(0.25 * s);
        q.x = static_cast<float>((m21 - m12) / s);
        q.y = static_cast<float>((m02 - m20) / s);
        q.z = static_cast<float>((m10 - m01) / s);
    }
    else if (m00 > m11 && m00 > m22)
    {
        const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
        q.w = static_cast<float>((m21 - m12) / s);
        q.x = static_cast<float>(0.25 * s);
        q.y = static_cast<float>((m01 + m10) / s);
        q.z = static_cast<float>((m02 + m20) / s);
    }
    else if (m11 > m22)
    {
        const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
        q.w = static_cast<float>((m02 - m20) / s);
        q.x = static_cast<float>((m01 + m10) / s);
        q.y = static_cast<float>(0.25 * s);
        q.z = static_cast<float>((m12 + m21) / s);
    }
    else
    {
        const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
        q.w = static_cast<float>((m10 - m01) / s);
        q.x = static_cast<float>((m02 + m20) / s);
        q.y = static_cast<float>((m12 + m21) / s);
        q.z = static_cast<float>(0.25 * s);
    }
    const double lenSq = static_cast<double>(q.x) * static_cast<double>(q.x) +
                         static_cast<double>(q.y) * static_cast<double>(q.y) +
                         static_cast<double>(q.z) * static_cast<double>(q.z) +
                         static_cast<double>(q.w) * static_cast<double>(q.w);
    if (!std::isfinite(lenSq) || lenSq <= 1.0e-24)
        return { 0.0f, 0.0f, 0.0f, 1.0f };
    const double invLen = 1.0 / std::sqrt(lenSq);
    q.x = static_cast<float>(static_cast<double>(q.x) * invLen);
    q.y = static_cast<float>(static_cast<double>(q.y) * invLen);
    q.z = static_cast<float>(static_cast<double>(q.z) * invLen);
    q.w = static_cast<float>(static_cast<double>(q.w) * invLen);
    return q;
}

float maxAbs2(float a, float b)
{
    return std::max(std::fabs(a), std::fabs(b));
}

float maxAbs3(float a, float b, float c)
{
    return std::max(maxAbs2(a, b), std::fabs(c));
}

void scaleRoundShape(Axis axis, const carb::Float3& scale, float& radius, float& halfHeight)
{
    if (axis == eX)
    {
        halfHeight *= scale.x;
        radius *= maxAbs2(scale.y, scale.z);
    }
    else if (axis == eY)
    {
        halfHeight *= scale.y;
        radius *= maxAbs2(scale.x, scale.z);
    }
    else
    {
        halfHeight *= scale.z;
        radius *= maxAbs2(scale.x, scale.y);
    }
}


// Does prim `path` have prim-type == `typeName`? (usd-prim-type is filter-only,
// like usd-schemas — so a scoped query, not a column read.)
bool isType(ovstage_instance_t* inst, const std::string& path, const char* typeName)
{
    ovx_string_t pathVal = ovxStr(path.c_str());
    ovx_string_t typeVal = ovxStr(typeName);

    ovstage_predicate_t preds[2]{};
    preds[0].attribute.string = ovxStr("usd-path");
    preds[0].op = OVSTAGE_FILTER_OP_IN;
    preds[0].values = &pathVal;
    preds[0].value_count = 1;
    preds[1].attribute.string = ovxStr("usd-prim-type");
    preds[1].op = OVSTAGE_FILTER_OP_IN;
    preds[1].values = &typeVal;
    preds[1].value_count = 1;

    ovstage_filter_t filter{};
    filter.predicates = preds;
    filter.count = 2;

    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    const ovstage_enqueue_result_t e = ovstage_query(inst, &filter, nullptr, 0, &q);
    if (e.status != OVSTAGE_OK)
        return false;
    waitAndRelease(inst, e);

    size_t count = 0;
    ovstage_query_result_t qr{};
    if (ovstage_fetch_query_result(inst, q, OVSTAGE_TIMEOUT_INFINITE, &qr) == OVSTAGE_OK)
    {
        count = qr.total_prim_count;
        ovstage_release_query_result(inst, &qr);
    }
    waitAndRelease(inst, ovstage_release_query(inst, q));
    return count >= 1;
}

bool isType(const OvstageSource& src, ovstage_instance_t* inst, ObjectKey key, const char* typeName)
{
    const std::string path(src.sourceKeyToString(key));
    return !path.empty() && isType(inst, path, typeName);
}

bool tokenColumnContains(ovstage_instance_t* inst,
                         ovx_path_dictionary_t* dict,
                         const OvstageSource& src,
                         ObjectKey key,
                         const char* attrName,
                         ovx_token_t wantedToken,
                         ovstage_ordinal_t readOrdinal)
{
    if (!inst || !dict || !key.valid() || !attrName || wantedToken == OVX_INVALID_TOKEN)
        return false;

    const uint64_t canonical = src.canonicalPath(key);
    // key.handle is the packed, generation-tagged ObjectKey (ADR-0021), never
    // a valid ovx_primpath_t for the C API below -- rawHandle() is the true
    // raw-handle fallback when canonicalization fails.
    ovx_primpath_t path = canonical ? canonical : src.rawHandle(key);
    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
    if (ovx_path_dictionary_create_path_list(dict, &path, 1, &list) != OVX_OK)
        return false;

    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(inst, list, &q) != OVSTAGE_OK || q == OVSTAGE_INVALID_QUERY_HANDLE)
    {
        ovx_path_dictionary_destroy_path_list(dict, list);
        return false;
    }

    ovx_token_t attr = OVX_INVALID_TOKEN;
    if (ovx_path_dictionary_intern_token(dict, ovxStr(attrName), &attr) != OVX_OK || attr == OVX_INVALID_TOKEN)
    {
        waitAndRelease(inst, ovstage_release_query(inst, q));
        ovx_path_dictionary_destroy_path_list(dict, list);
        return false;
    }

    ovstage_ordinal_range_t range{};
    range.end_ordinal = readOrdinal;
    range.has_start_ordinal = false;

    bool found = false;
    ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
    const ovstage_enqueue_result_t re = ovstage_read_attributes(inst, q, &attr, 1, range, &rh);
    if (re.status == OVSTAGE_OK)
    {
        waitAndRelease(inst, re);
        ovstage_read_group_t g{};
        while (!found && ovstage_fetch_read_next(inst, rh, OVSTAGE_TIMEOUT_INFINITE, &g) == OVSTAGE_OK)
        {
            if (g.data.tensor_count > 0 && g.data.tensors && g.data.tensors[0].data)
            {
                const DLTensor& t = g.data.tensors[0];
                size_t valueCount = 1;
                for (int i = 0; i < t.ndim; ++i)
                    valueCount *= static_cast<size_t>(t.shape[i]);

                const uint8_t* bytes = static_cast<const uint8_t*>(t.data) + t.byte_offset;
                if (t.dtype.code == kDLUInt && t.dtype.bits == 64)
                {
                    const uint64_t* values = reinterpret_cast<const uint64_t*>(bytes);
                    for (size_t i = 0; i < valueCount; ++i)
                    {
                        if (values[i] == static_cast<uint64_t>(wantedToken))
                        {
                            found = true;
                            break;
                        }
                    }
                }
                else if (t.dtype.code == kDLUInt && t.dtype.bits == 32)
                {
                    const uint32_t* values = reinterpret_cast<const uint32_t*>(bytes);
                    for (size_t i = 0; i < valueCount; ++i)
                    {
                        if (values[i] == static_cast<uint32_t>(wantedToken))
                        {
                            found = true;
                            break;
                        }
                    }
                }
            }
            ovstage_release_group(inst, &g);
        }
        waitAndRelease(inst, ovstage_release_read(inst, rh));
    }

    waitAndRelease(inst, ovstage_release_query(inst, q));
    ovx_path_dictionary_destroy_path_list(dict, list);
    return found;
}

bool isXformable(const OvstageSource& src, ObjectKey key)
{
    return src.isA(key, src.internToken("Xformable"));
}

// `src` provides the canonical identity used to dedup: `dst`/`srcKeys` may mix
// keys minted through different routes for the same logical prim (schema-cache
// keys are always canonical, but a raw enumerate()-sourced key is not
// guaranteed to pack to the same value as its canonical alias -- ADR-0021), so
// a raw ObjectKey.handle comparison alone is not a reliable identity test.
void appendUnique(const OvstageSource& src, std::vector<ObjectKey>& dst, const std::vector<ObjectKey>& srcKeys)
{
    for (const ObjectKey key : srcKeys)
    {
        const uint64_t canonical = src.canonicalPath(key);
        const uint64_t identity = canonical ? canonical : key.handle;
        const auto it = std::find_if(dst.begin(), dst.end(), [&](ObjectKey existing)
        {
            const uint64_t existingCanonical = src.canonicalPath(existing);
            return (existingCanonical ? existingCanonical : existing.handle) == identity;
        });
        if (it == dst.end())
            dst.push_back(key);
    }
}

Bucket enumerateSchema(const OvstageSource& src,
                       ovstage_instance_t* inst,
                       ovx_path_dictionary_t* dict,
                       const char* schemaName,
                       const char* probeAttr,
                       ovstage_ordinal_t readOrdinal)
{
    return enumerate(src, inst, dict, "usd-schemas", OVSTAGE_FILTER_OP_CONTAINS,
                     schemaName, probeAttr, readOrdinal, conv::kUsdSchemas);
}

Bucket enumerateSchemas(const OvstageSource& src,
                        ovstage_instance_t* inst,
                        ovx_path_dictionary_t* dict,
                        const char* const* schemaNames,
                        size_t schemaNameCount,
                        const char* probeAttr,
                        ovstage_ordinal_t readOrdinal)
{
    Bucket out;
    for (size_t i = 0; i < schemaNameCount; ++i)
    {
        Bucket one = enumerateSchema(src, inst, dict, schemaNames[i], probeAttr, readOrdinal);
        appendUnique(src, out.keys, one.keys);
        for (const std::string& attr : one.attrs)
        {
            if (std::find(out.attrs.begin(), out.attrs.end(), attr) == out.attrs.end())
                out.attrs.push_back(attr);
        }
    }
    return out;
}

std::vector<ObjectKey> relationshipTargets(const OvstageSource& src, ObjectKey key, const char* relName)
{
    std::vector<ObjectKey> targets;
    src.getRelationshipTargets(key, src.internToken(relName), targets);
    return targets;
}

std::vector<ObjectKey> relationshipTargetsEither(const OvstageSource& src, ObjectKey key,
                                                 const char* primaryRel, const char* fallbackRel)
{
    std::vector<ObjectKey> targets = relationshipTargets(src, key, primaryRel);
    if (targets.empty() && fallbackRel && fallbackRel[0] != '\0')
        targets = relationshipTargets(src, key, fallbackRel);
    return targets;
}

ObjectKey resolveRelOrApi(const OvstageSource& src, ObjectKey key, const char* relName, const char* apiName)
{
    std::vector<ObjectKey> targets = relationshipTargets(src, key, relName);
    if (targets.size() == 1)
        return targets[0];
    if (targets.size() > 1)
        return {};
    if (src.hasSchema(key, src.internToken(apiName)))
        return key;
    return {};
}

template <typename T>
bool readTypedArray(const OvstageSource& src, ObjectKey key, const char* attrName,
                    BufferElemType type, int comps, std::vector<T>& out)
{
    out.clear();
    BufferHandle h = src.readArrayAttribute(key, attrName, type, comps);
    if (!h.valid())
        return false;

    size_t byteCount = 0;
    const void* data = src.resolveBuffer(h, byteCount);
    if (!data)
    {
        src.releaseBuffer(h);
        return false;
    }

    const size_t count = std::min<size_t>(h.elemCount, byteCount / sizeof(T));
    const T* typed = static_cast<const T*>(data);
    out.assign(typed, typed + count);
    src.releaseBuffer(h);
    return true;
}

bool readFloatArray(const OvstageSource& src, ObjectKey key, const char* attrName, std::vector<float>& out)
{
    return readTypedArray(src, key, attrName, BufferElemType::eFloat, 1, out);
}

bool readFloatArrayEither(const OvstageSource& src, ObjectKey key,
                          const char* primaryAttr, const char* fallbackAttr, std::vector<float>& out)
{
    if (readFloatArray(src, key, primaryAttr, out))
        return true;
    return fallbackAttr && fallbackAttr[0] != '\0' && readFloatArray(src, key, fallbackAttr, out);
}

bool readIntArray(const OvstageSource& src, ObjectKey key, const char* attrName, std::vector<int>& out)
{
    out.clear();
    std::vector<int32_t> tmp;
    if (!readTypedArray(src, key, attrName, BufferElemType::eInt32, 1, tmp))
        return false;
    out.reserve(tmp.size());
    for (int32_t v : tmp)
        out.push_back(static_cast<int>(v));
    return true;
}

bool readFloat2Array(const OvstageSource& src, ObjectKey key, const char* attrName, std::vector<carb::Float2>& out)
{
    return readTypedArray(src, key, attrName, BufferElemType::eVec2, 2, out);
}

bool readFloat4Array(const OvstageSource& src, ObjectKey key, const char* attrName, std::vector<carb::Float4>& out)
{
    return readTypedArray(src, key, attrName, BufferElemType::eVec4, 4, out);
}

bool hasCollisionInSubtree(const OvstageSource& src, ObjectKey key, TokenId collisionApi)
{
    if (src.hasSchema(key, collisionApi))
        return true;
    bool found = false;
    src.forEachChild(key, [&](ObjectKey child)
    {
        if (!found && hasCollisionInSubtree(src, child, collisionApi))
            found = true;
    });
    return found;
}

bool fillCommonShape(ParseContext& ctx, ObjectKey key, const ShapeInfo& info, PhysxShapeDesc& desc)
{
    setToDefault(desc, ctx.units());

    desc.collisionEnabled = info.collisionEnabled;
    desc.collisionGroup = info.collisionGroup;
    desc.rigidBody = info.rigidBody;
    desc.localPos = info.localPos;
    desc.localRot = info.localRot;
    desc.localScale = info.localScale;
    desc.sourceGprim = info.sourceGprim;

    CollisionExtFields fields;
    fields.contactOffset = desc.contactOffset;
    fields.restOffset = desc.restOffset;
    fields.torsionalPatchRadius = desc.torsionalPatchRadius;
    fields.minTorsionalPatchRadius = desc.minTorsionalPatchRadius;
    fields.isTrigger = desc.isTrigger;
    fields.isTriggerUsdOutput = desc.isTriggerUsdOutput;
    fields.attributeFallback = info.collisionAttributeFallback;
    parseCollisionExt(ctx, key, fields);
    desc.contactOffset = fields.contactOffset;
    desc.restOffset = fields.restOffset;
    desc.torsionalPatchRadius = fields.torsionalPatchRadius;
    desc.minTorsionalPatchRadius = fields.minTorsionalPatchRadius;
    desc.isTrigger = fields.isTrigger;
    desc.isTriggerUsdOutput = fields.isTriggerUsdOutput;

    for (ObjectKey owner : info.simulationOwners)
    {
        ObjectId id = ctx.objects().findEntry(owner, eScene);
        if (id != kInvalidObjectId)
            desc.sceneIds.push_back(id);
    }
    return info.simulationOwners.empty() || !desc.sceneIds.empty();
}

// Mesh-shape descriptor builder — the ovstage peer of NativeWalker::readMeshShapeDesc.
// Reads the mesh approximation + cooking knobs via the shared parse functions, applies
// the TriangleMesh / MeshSimplification -> ConvexHull fallback for dynamic non-kinematic
// bodies (PhysX rejects non-SDF triangle meshes as a simulation shape on a dynamic body),
// and allocates the typed descriptor. meshPrimKey = gprimKey: the cooking service reads
// the geometry from there via IPhysicsSource::getMeshAttributes (FROM_PRIM_MESH_VIEW path).
// Bounding-sphere/cube are not handled (they need an eagerly-merged point buffer the scan
// owns) -> fall through to a triangle mesh.
DescPtr<PhysxShapeDesc> buildMeshShapeDesc(OvstageScanResult& out, ParseContext& ctx, const IPhysicsSource& src,
                                           ObjectKey shapeKey, ObjectKey gprimKey, ObjectKey rigidBodyKey,
                                           const carb::Float3& meshScale, const carb::Float3& signScale,
                                           bool doubleSided)
{
    MeshApproximation approx = parseMeshApproximation(ctx, shapeKey);

    if ((approx == MeshApproximation::eNone || approx == MeshApproximation::eMeshSimplification) &&
        rigidBodyKey.valid())
    {
        bool kinematic = false, enabled = true;
        src.getAttribute(rigidBodyKey, src.internToken("physics:kinematicEnabled"), kinematic);
        src.getAttribute(rigidBodyKey, src.internToken("physics:rigidBodyEnabled"), enabled);
        if (!kinematic && enabled)
            approx = MeshApproximation::eConvexHull;
    }

    switch (approx)
    {
    case MeshApproximation::eConvexHull:
    {
        DescPtr<ConvexMeshPhysxShapeDesc> cmd = allocateDesc<ConvexMeshPhysxShapeDesc>(ctx.descriptorAllocator());
        cmd->meshScale = meshScale;
        cmd->meshPrimKey = gprimKey;
        cmd->convexCookingParams.signScale = signScale;
        parseConvexHullCookingExt(ctx, shapeKey, cmd->convexCookingParams);
        return descPtrCast<PhysxShapeDesc>(std::move(cmd));
    }
    case MeshApproximation::eMeshSimplification:
    {
        DescPtr<TriangleMeshPhysxShapeDesc> tmd = allocateDesc<TriangleMeshPhysxShapeDesc>(ctx.descriptorAllocator());
        tmd->meshScale = meshScale;
        tmd->meshPrimKey = gprimKey;
        tmd->doubleSided = doubleSided;
        tmd->triangleMeshCookingParams.mode = omni::physx::TriangleMeshMode::eQUADRIC_SIMPLIFICATION;
        tmd->sdfMeshCookingParams.sdfResolution = 0;
        parseTriangleMeshSimplificationCookingExt(ctx, shapeKey, tmd->triangleMeshCookingParams);
        return descPtrCast<PhysxShapeDesc>(std::move(tmd));
    }
    case MeshApproximation::eConvexDecomposition:
    {
        DescPtr<ConvexMeshDecompositionPhysxShapeDesc> cdd =
            allocateDesc<ConvexMeshDecompositionPhysxShapeDesc>(ctx.descriptorAllocator());
        cdd->meshScale = meshScale;
        cdd->meshPrimKey = gprimKey;
        cdd->doubleSided = doubleSided;
        cdd->convexDecompositionCookingParams.signScale = signScale;
        cdd->sdfMeshCookingParams.sdfResolution = 0;
        parseConvexDecompositionCookingExt(ctx, shapeKey, cdd->convexDecompositionCookingParams);
        return descPtrCast<PhysxShapeDesc>(std::move(cdd));
    }
    case MeshApproximation::eSphereFill:
    {
        DescPtr<SpherePointsPhysxShapeDesc> sfd = allocateDesc<SpherePointsPhysxShapeDesc>(ctx.descriptorAllocator());
        sfd->meshScale = meshScale;
        sfd->meshPrimKey = gprimKey;
        sfd->doubleSided = doubleSided;
        sfd->sphereFillCookingParams.signScale = signScale;
        sfd->sdfMeshCookingParams.sdfResolution = 0;
        parseSphereFillCookingExt(ctx, shapeKey, sfd->sphereFillCookingParams);
        return descPtrCast<PhysxShapeDesc>(std::move(sfd));
    }
    case MeshApproximation::eSdf:
    {
        DescPtr<TriangleMeshPhysxShapeDesc> tmd = allocateDesc<TriangleMeshPhysxShapeDesc>(ctx.descriptorAllocator());
        tmd->meshScale = meshScale;
        tmd->meshPrimKey = gprimKey;
        tmd->doubleSided = doubleSided;
        parseTriangleMeshCookingExt(ctx, shapeKey, tmd->triangleMeshCookingParams);
        parseSdfMeshCookingExt(ctx, shapeKey, tmd->sdfMeshCookingParams);
        return descPtrCast<PhysxShapeDesc>(std::move(tmd));
    }
    case MeshApproximation::eBoundingSphere:
    case MeshApproximation::eBoundingCube:
    {
        // Read the gprim's points into a mergedMesh buffer; the consumer computes
        // the bounding-sphere radius / bounding-box halfExtents from those points.
        // (Mirrors NativeWalker's USD path; needed so a dynamic body gets an analytic
        // bounding shape instead of an illegal triangle-mesh simulation shape.)
        // The points must carry the gprim's world scale: unlike the cooked-mesh
        // descs there is no meshScale field for the consumer to apply later, so an
        // unscaled buffer yields a bounding shape in mesh-local units.
        DescPtr<MergeMeshDesc> mm = allocateDesc<MergeMeshDesc>(ctx.descriptorAllocator());
        const MeshGeometry geom = parseMeshGeometry(ctx, gprimKey);
        scaleMeshPoints(ctx, geom.points, meshScale, mm->points);

        DescPtr<MergeMeshPhysxShapeDesc> typed;
        if (approx == MeshApproximation::eBoundingSphere)
        {
            DescPtr<BoundingSpherePhysxShapeDesc> bs =
                allocateDesc<BoundingSpherePhysxShapeDesc>(ctx.descriptorAllocator());
            typed = descPtrCast<MergeMeshPhysxShapeDesc>(std::move(bs));
        }
        else
        {
            DescPtr<BoundingBoxPhysxShapeDesc> bb =
                allocateDesc<BoundingBoxPhysxShapeDesc>(ctx.descriptorAllocator());
            typed = descPtrCast<MergeMeshPhysxShapeDesc>(std::move(bb));
        }
        typed->mergedMesh = mm.get();
        out.ownedMeshes.push_back(std::move(mm));
        return descPtrCast<PhysxShapeDesc>(std::move(typed));
    }
    case MeshApproximation::eNone:
    default:
    {
        DescPtr<TriangleMeshPhysxShapeDesc> tmd = allocateDesc<TriangleMeshPhysxShapeDesc>(ctx.descriptorAllocator());
        tmd->meshScale = meshScale;
        tmd->meshPrimKey = gprimKey;
        tmd->doubleSided = doubleSided;
        tmd->sdfMeshCookingParams.sdfResolution = 0;
        parseTriangleMeshCookingExt(ctx, shapeKey, tmd->triangleMeshCookingParams);
        return descPtrCast<PhysxShapeDesc>(std::move(tmd));
    }
    }
}

} // namespace

// Not declared in the public header; test translation units forward-declare these.
void resetOvstageWalkerEnumerateReadCountForTest()
{
    enumerateReadCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageWalkerEnumerateReadCountForTest()
{
    return enumerateReadCounter().load(std::memory_order_relaxed);
}

void resetOvstageWalkerEnumerateQueryCountForTest()
{
    enumerateQueryCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageWalkerEnumerateQueryCountForTest()
{
    return enumerateQueryCounter().load(std::memory_order_relaxed);
}

void setOvstageWalkerEnumerateReadFaultForTest(int count)
{
    enumerateReadFaultCounter().store(count, std::memory_order_relaxed);
}

int getOvstageWalkerEnumerateReadFaultForTest()
{
    return enumerateReadFaultCounter().load(std::memory_order_relaxed);
}

void emitTireFrictionTable(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    TireFrictionTableInfo info;
    info.defaultFrictionValue = 1.0f;
    if (!src.getAttribute(key, src.internToken("defaultFrictionValue"), info.defaultFrictionValue))
        src.getAttribute(key, src.internToken("physxVehicleTireFrictionTable:defaultFrictionValue"),
                         info.defaultFrictionValue);

    info.materialPaths = relationshipTargetsEither(src, key, "groundMaterials",
                                                   "physxVehicleTireFrictionTable:groundMaterials");
    readFloatArrayEither(src, key, "frictionValues", "physxVehicleTireFrictionTable:frictionValues",
                         info.frictionValues);

    if (DescPtr<TireFrictionTableDesc> desc = parseTireFrictionTable(ctx, key, info))
        out.tireFrictionTables.push_back(std::move(desc));
}

void emitVehicleContext(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    if (!src.isA(key, src.internToken("PhysicsScene")))
        return;
    if (DescPtr<VehicleContextDesc> desc = parseVehicleContext(ctx, key))
        out.vehicleContexts.push_back(std::move(desc));
}

void emitVehicleWheel(OvstageScanResult& out, ParseContext& ctx, ObjectKey key)
{
    if (DescPtr<WheelDesc> desc = parseWheel(ctx, key))
        out.vehicleWheels.push_back(std::move(desc));
}

void emitVehicleTire(OvstageScanResult& out, ovstage_instance_t* inst, OvstageSource& src,
                     ParseContext& ctx, ObjectKey key, float massScale)
{
    TireInfo info;
    info.massScale = massScale;

    std::vector<carb::Float2> graph;
    if (readFloat2Array(src, key, "physxVehicleTire:frictionVsSlipGraph", graph))
    {
        if (graph.size() != 3)
            return;
        info.hasFrictionVsSlipGraph = true;
        for (uint32_t i = 0; i < 3; ++i)
            info.frictionVsSlipGraph[i] = graph[i];
    }

    const std::vector<ObjectKey> ftTargets = relationshipTargets(src, key, "physxVehicleTire:frictionTable");
    if (ftTargets.size() == 1)
    {
        if (isType(src, inst, ftTargets[0], "PhysxVehicleTireFrictionTable"))
            info.frictionTableKey = ftTargets[0];
        else
            info.frictionTableTooMany = true;
    }
    else if (ftTargets.size() > 1)
    {
        info.frictionTableTooMany = true;
    }

    if (DescPtr<TireDesc> desc = parseTire(ctx, key, info))
        out.vehicleTires.push_back(std::move(desc));
}

void emitVehicleSuspension(OvstageScanResult& out, ParseContext& ctx, ObjectKey key)
{
    if (DescPtr<SuspensionDesc> desc = parseSuspension(ctx, key))
        out.vehicleSuspensions.push_back(std::move(desc));
}

void emitVehicleEngine(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx,
                       ObjectKey key, float kgmsScale)
{
    EngineInfo info;
    info.kgmsScale = kgmsScale;

    std::vector<carb::Float2> curve;
    if (readFloat2Array(src, key, "physxVehicleEngine:torqueCurve", curve))
    {
        info.hasTorqueCurve = true;
        info.torqueCurvePointCount = static_cast<uint32_t>(curve.size());
        const uint32_t copyCount = std::min(info.torqueCurvePointCount,
            static_cast<uint32_t>(EngineDesc::maxNumberOfTorqueCurvePoints));
        for (uint32_t i = 0; i < copyCount; ++i)
            info.torqueCurve[i] = curve[i];
    }

    if (DescPtr<EngineDesc> desc = parseEngine(ctx, key, info))
        out.vehicleEngines.push_back(std::move(desc));
}

void emitVehicleGears(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    GearsInfo info;
    info.hasRatios = readFloatArray(src, key, "physxVehicleGears:ratios", info.ratios);

    if (DescPtr<GearsDesc> desc = parseGears(ctx, key, info))
    {
        out.vehicleGears.push_back(std::move(desc));
        out.vehicleGearsPaths.push_back(key);
    }
}

void emitVehicleClutch(OvstageScanResult& out, ParseContext& ctx, ObjectKey key, float kgmsScale)
{
    if (DescPtr<ClutchDesc> desc = parseClutch(ctx, key, kgmsScale))
    {
        out.vehicleClutches.push_back(std::move(desc));
        out.vehicleClutchPaths.push_back(key);
    }
}

void emitVehicleDriveBasic(OvstageScanResult& out, ParseContext& ctx, ObjectKey key, float kgmsScale)
{
    if (DescPtr<DriveBasicDesc> desc = parseDriveBasic(ctx, key, kgmsScale))
        out.vehicleDrivesBasic.push_back(std::move(desc));
}

void emitVehicleDriveStandard(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    DriveStandardInfo info;
    info.engineKey = resolveRelOrApi(src, key, "physxVehicleDriveStandard:engine", "PhysxVehicleEngineAPI");
    info.gearsKey = resolveRelOrApi(src, key, "physxVehicleDriveStandard:gears", "PhysxVehicleGearsAPI");
    info.autoGearBoxKey = resolveRelOrApi(src, key, "physxVehicleDriveStandard:autoGearBox",
                                          "PhysxVehicleAutoGearBoxAPI");
    info.clutchKey = resolveRelOrApi(src, key, "physxVehicleDriveStandard:clutch", "PhysxVehicleClutchAPI");

    if (DescPtr<DriveStandardDesc> desc = parseDriveStandard(ctx, key, info))
    {
        out.vehicleDrivesStandard.push_back(std::move(desc));
        out.vehicleDrivesStandardPaths.push_back(key);
        out.vehicleDrivesStandardCrossRefs.push_back(info);
    }
}

void readDifferentialInfo(OvstageSource& src, ObjectKey key, DifferentialInfo& info)
{
    readIntArray(src, key, "physxVehicleMultiWheelDifferential:wheels", info.wheels);
    info.hasTorqueRatios = readFloatArray(src, key, "physxVehicleMultiWheelDifferential:torqueRatios",
                                          info.torqueRatios);
    info.hasAverageWheelSpeedRatios = readFloatArray(
        src, key, "physxVehicleMultiWheelDifferential:averageWheelSpeedRatios", info.averageWheelSpeedRatios);
}

void emitVehicleMultiWheelDifferential(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    DifferentialInfo info;
    readDifferentialInfo(src, key, info);
    if (DescPtr<MultiWheelDifferentialDesc> desc = parseMultiWheelDifferential(ctx, key, info))
    {
        out.vehicleMultiWheelDifferentials.push_back(std::move(desc));
        out.vehicleMultiWheelDifferentialPaths.push_back(key);
    }
}

void emitVehicleTankDifferential(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    DifferentialInfo info;
    readDifferentialInfo(src, key, info);

    TankDifferentialInfo tankInfo;
    tankInfo.hasNumberOfWheelsPerTrack = readIntArray(
        src, key, "physxVehicleTankDifferential:numberOfWheelsPerTrack", tankInfo.numberOfWheelsPerTrack);
    tankInfo.hasThrustIndexPerTrack = readIntArray(
        src, key, "physxVehicleTankDifferential:thrustIndexPerTrack", tankInfo.thrustIndexPerTrack);
    tankInfo.hasWheelIndicesInTrackOrder = readIntArray(
        src, key, "physxVehicleTankDifferential:wheelIndicesInTrackOrder", tankInfo.wheelIndicesInTrackOrder);
    tankInfo.hasTrackToWheelIndices = readIntArray(
        src, key, "physxVehicleTankDifferential:trackToWheelIndices", tankInfo.trackToWheelIndices);

    if (DescPtr<TankDifferentialDesc> desc = parseTankDifferential(ctx, key, info, tankInfo))
    {
        out.vehicleTankDifferentials.push_back(std::move(desc));
        out.vehicleTankDifferentialPaths.push_back(key);
    }
}

void emitVehicleAutoGearBox(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    AutoGearBoxInfo info;
    info.hasUpRatios = readFloatArray(src, key, "physxVehicleAutoGearBox:upRatios", info.upRatios);
    info.hasDownRatios = readFloatArray(src, key, "physxVehicleAutoGearBox:downRatios", info.downRatios);

    if (DescPtr<AutoGearBoxDesc> desc = parseAutoGearBox(ctx, key, info))
    {
        out.vehicleAutoGearBoxes.push_back(std::move(desc));
        out.vehicleAutoGearBoxPaths.push_back(key);
    }
}

void emitVehicleBrakes(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    src.forEachMultiApplyInstance(key, "PhysxVehicleBrakesAPI", [&](std::string_view instance)
    {
        uint8_t brakesIndex = 0;
        if (instance == "brakes0")
            brakesIndex = 0;
        else if (instance == "brakes1")
            brakesIndex = 1;
        else
            return;

        const std::string attrBase = std::string("physxVehicleBrakes:") + std::string(instance) + ":";
        BrakesInfo info;
        readIntArray(src, key, (attrBase + "wheels").c_str(), info.wheels);
        info.hasTorqueMultipliers = readFloatArray(src, key, (attrBase + "torqueMultipliers").c_str(),
                                                   info.torqueMultipliers);

        if (DescPtr<BrakesDesc> desc = parseBrakes(ctx, key, instance, brakesIndex, info))
        {
            out.vehicleBrakes.push_back(std::move(desc));
            out.vehicleBrakesPaths.push_back(key);
            out.vehicleBrakesInstanceTokens.push_back(src.internToken(instance));
        }
    });
}

void emitVehicleSteeringBasic(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    SteeringBasicInfo info;
    readIntArray(src, key, "physxVehicleSteering:wheels", info.wheels);
    info.hasAngleMultipliers = readFloatArray(src, key, "physxVehicleSteering:angleMultipliers",
                                              info.angleMultipliers);

    if (DescPtr<SteeringBasicDesc> desc = parseSteeringBasic(ctx, key, info))
    {
        out.vehicleSteeringBasic.push_back(std::move(desc));
        out.vehicleSteeringBasicPaths.push_back(key);
    }
}

void emitVehicleSteeringAckermann(OvstageScanResult& out, ParseContext& ctx, ObjectKey key)
{
    if (DescPtr<SteeringAckermannDesc> desc = parseSteeringAckermann(ctx, key))
    {
        out.vehicleSteeringAckermann.push_back(std::move(desc));
        out.vehicleSteeringAckermannPaths.push_back(key);
    }
}

void emitVehicleNonlinearCmdResponse(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    src.forEachMultiApplyInstance(key, "PhysxVehicleNonlinearCommandResponseAPI", [&](std::string_view instance)
    {
        const std::string attrBase = std::string("physxVehicleNonlinearCommandResponse:") +
                                     std::string(instance) + ":";
        NonlinearCmdResponseInfo info;
        info.hasCommandValues = readFloatArray(src, key, (attrBase + "commandValues").c_str(),
                                               info.commandValues);
        info.hasSpeedResponsesPerCommandValue = readIntArray(
            src, key, (attrBase + "speedResponsesPerCommandValue").c_str(), info.speedResponsesPerCommandValue);
        info.hasSpeedResponses = readFloat2Array(src, key, (attrBase + "speedResponses").c_str(),
                                                 info.speedResponses);

        if (DescPtr<NonlinearCmdResponseDesc> desc = parseNonlinearCmdResponse(ctx, key, instance, info))
        {
            out.vehicleNonlinearCmdResponses.push_back(std::move(desc));
            out.vehicleNonlinearCmdResponsePaths.push_back(key);
            out.vehicleNonlinearCmdResponseInstanceTokens.push_back(src.internToken(instance));
        }
    });
}

void emitVehicleSuspensionCompliance(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    SuspensionComplianceInfo info;
    readFloat2Array(src, key, "physxVehicleSuspensionCompliance:wheelToeAngle", info.wheelToeAngles);
    readFloat2Array(src, key, "physxVehicleSuspensionCompliance:wheelCamberAngle", info.wheelCamberAngles);
    readFloat4Array(src, key, "physxVehicleSuspensionCompliance:suspensionForceAppPoint",
                    info.suspensionForceAppPoints);
    readFloat4Array(src, key, "physxVehicleSuspensionCompliance:tireForceAppPoint", info.tireForceAppPoints);

    if (DescPtr<SuspensionComplianceDesc> desc = parseSuspensionCompliance(ctx, key, info))
    {
        out.vehicleSuspensionCompliances.push_back(std::move(desc));
        out.vehicleSuspensionCompliancePaths.push_back(key);
    }
}

void emitVehicleWheelAttachment(OvstageScanResult& out, ovstage_instance_t* inst, OvstageSource& src,
                                ParseContext& ctx, ObjectKey key)
{
    WheelAttachmentInfo info;
    info.wheelKey = resolveRelOrApi(src, key, "physxVehicleWheelAttachment:wheel", "PhysxVehicleWheelAPI");
    info.tireKey = resolveRelOrApi(src, key, "physxVehicleWheelAttachment:tire", "PhysxVehicleTireAPI");
    info.suspensionKey = resolveRelOrApi(src, key, "physxVehicleWheelAttachment:suspension",
                                         "PhysxVehicleSuspensionAPI");

    const TokenId vehicleApi = src.internToken("PhysxVehicleAPI");
    for (ObjectKey parent = src.getParent(key); parent.valid(); parent = src.getParent(parent))
    {
        if (src.hasSchema(parent, vehicleApi))
        {
            info.vehicleKey = parent;
            break;
        }
    }
    out.vehicleWheelAttachmentOwners.push_back({ info.vehicleKey, key });

    src.getAttribute(key, src.internToken("physxVehicleWheelAttachment:suspensionTravelDirection"),
                     info.suspensionTravelDirection);

    const TokenId suspForceTok = src.internToken("physxVehicleWheelAttachment:suspensionForceAppPointOffset");
    if (src.hasAuthoredAttribute(key, suspForceTok) && src.getAttribute(key, suspForceTok,
                                                                        info.suspensionForceAppPointOffset))
        info.state |= WheelAttachmentDesc::eHAS_SUSP_FORCE_APP_POINT;

    const TokenId suspFramePosTok = src.internToken("physxVehicleWheelAttachment:suspensionFramePosition");
    if (src.hasAuthoredAttribute(key, suspFramePosTok) && src.getAttribute(key, suspFramePosTok,
                                                                           info.suspensionFramePosition))
        info.state |= WheelAttachmentDesc::eHAS_SUSPENSION_FRAME;

    if (!(info.state & WheelAttachmentDesc::eHAS_SUSPENSION_FRAME))
    {
        const TokenId wheelComTok = src.internToken("physxVehicleWheelAttachment:wheelCenterOfMassOffset");
        if (src.hasAuthoredAttribute(key, wheelComTok) && src.getAttribute(key, wheelComTok,
                                                                           info.wheelCenterOfMassOffset))
            info.state |= WheelAttachmentDesc::eHAS_WHEEL_COM_OFFSET;
    }

    src.getAttribute(key, src.internToken("physxVehicleWheelAttachment:suspensionFrameOrientation"),
                     info.suspensionFrameOrientation);

    const TokenId tireForceTok = src.internToken("physxVehicleWheelAttachment:tireForceAppPointOffset");
    if (src.hasAuthoredAttribute(key, tireForceTok) && src.getAttribute(key, tireForceTok,
                                                                        info.tireForceAppPointOffset))
        info.state |= WheelAttachmentDesc::eHAS_TIRE_FORCE_APP_POINT;

    src.getAttribute(key, src.internToken("physxVehicleWheelAttachment:wheelFramePosition"),
                     info.wheelFramePosition);
    src.getAttribute(key, src.internToken("physxVehicleWheelAttachment:wheelFrameOrientation"),
                     info.wheelFrameOrientation);
    src.getAttribute(key, src.internToken("physxVehicleWheelAttachment:driven"), info.driven);

    int64_t index = 0;
    if (src.getAttribute(key, src.internToken("physxVehicleWheelAttachment:index"), index))
        info.index = static_cast<int>(index);

    const std::vector<ObjectKey> cgTargets = relationshipTargets(src, key,
                                                                 "physxVehicleWheelAttachment:collisionGroup");
    if (cgTargets.size() == 1 && isType(src, inst, cgTargets[0], "PhysicsCollisionGroup"))
        info.collisionGroupKey = cgTargets[0];

    if (isXformable(src, key))
    {
        info.state |= WheelAttachmentDesc::eMANAGE_TRANSFORMS;
        const TokenId collisionApi = src.internToken("PhysicsCollisionAPI");
        bool foundShape = false;
        if (src.hasSchema(key, collisionApi))
        {
            info.shapeKey = key;
            info.state |= WheelAttachmentDesc::eHAS_SHAPE;
            foundShape = true;
        }

        bool sawNestedCollider = false;
        src.forEachChild(key, [&](ObjectKey child)
        {
            if (src.hasSchema(child, collisionApi))
            {
                if (foundShape)
                {
                    sawNestedCollider = true;
                    return;
                }
                info.shapeKey = child;
                info.state |= WheelAttachmentDesc::eHAS_SHAPE;
                foundShape = true;
            }
            src.forEachChild(child, [&](ObjectKey grandChild)
            {
                if (hasCollisionInSubtree(src, grandChild, collisionApi))
                    sawNestedCollider = true;
            });
        });
        if (sawNestedCollider)
            return;
    }

    if (DescPtr<WheelAttachmentDesc> desc = parseWheelAttachment(ctx, key, info))
    {
        out.vehicleWheelAttachments.push_back(std::move(desc));
        out.vehicleWheelAttachmentInfos.push_back(info);
    }
}

void emitVehicle(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx,
                 ObjectKey key, float lengthScale)
{
    VehicleInfo info;
    info.lengthScale = lengthScale;
    Matrix3d rotation;
    src.getLocalToWorldRotationAndScale(key, rotation, info.scale);
    if (info.scale.x == 0.0f || info.scale.y == 0.0f || info.scale.z == 0.0f)
        return;

    TokenId queryType;
    const TokenId sweepType = src.internToken("sweep");
    if (src.getAttribute(key, src.internToken("physxVehicle:suspensionLineQueryType"), queryType) &&
        queryType == sweepType)
        info.queryType = 1;

    if (DescPtr<VehicleDesc> desc = parseVehicle(ctx, key, info))
    {
        out.vehicles.push_back(std::move(desc));
        out.vehiclePaths.push_back(key);
    }
}


void emitDeformableMaterial(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    DescPtr<PhysxDeformableMaterialDesc> desc;
    if (src.hasSchema(key, src.internToken("OmniPhysicsSurfaceDeformableMaterialAPI")))
    {
        DescPtr<PhysxSurfaceDeformableMaterialDesc> surf = allocateDesc<PhysxSurfaceDeformableMaterialDesc>(ctx.descriptorAllocator());
        parseDeformableMaterial(ctx, key, *surf);
        parseSurfaceDeformableMaterial(ctx, key, *surf);
        desc = descPtrCast<PhysxDeformableMaterialDesc>(std::move(surf));
    }
    else
    {
        desc = allocateDesc<PhysxDeformableMaterialDesc>(ctx.descriptorAllocator());
        parseDeformableMaterial(ctx, key, *desc);
    }
    if (!desc)
        return;
    desc->poissonsRatio = std::min(desc->poissonsRatio, 0.4999f);
    desc->materialKey = key;
    out.deformableMaterials.push_back(std::move(desc));
}

void emitParticleSystem(OvstageScanResult& out, ParseContext& ctx, ObjectKey key)
{
    DescPtr<ParticleSystemDesc> sys = parseParticleSystem(ctx, key);
    if (!sys)
        return;
    const float fluidRestOffset = sys->fluidRestOffset;
    out.particleSystems.push_back(std::move(sys));

    if (DescPtr<ParticleAnisotropyDesc> a = parseParticleAnisotropy(ctx, key))
        out.particleAnisotropies.push_back(std::move(a));
    if (DescPtr<ParticleSmoothingDesc> sm = parseParticleSmoothing(ctx, key))
        out.particleSmoothings.push_back(std::move(sm));
    if (DescPtr<ParticleIsosurfaceDesc> iso = parseParticleIsosurface(ctx, key, fluidRestOffset))
        out.particleIsosurfaces.push_back(std::move(iso));
}

void emitParticleSet(OvstageScanResult& out, ParseContext& ctx, ObjectKey key)
{
    if (DescPtr<ParticleSetDesc> desc = parseParticleSet(ctx, key))
    {
        if (desc->particleSystemKey.valid() && ctx.source().exists(desc->particleSystemKey))
        {
            const ObjectKey canonicalSystemKey = ctx.source().canonicalKey(desc->particleSystemKey);
            const bool alreadyEmitted =
                std::any_of(out.particleSystems.begin(), out.particleSystems.end(),
                            [&](const DescPtr<ParticleSystemDesc>& existing)
                            {
                                return ctx.source().canonicalKey(existing->systemKey).handle == canonicalSystemKey.handle;
                            });
            if (!alreadyEmitted)
            {
                emitParticleSystem(out, ctx, desc->particleSystemKey);
            }
        }
        out.particleSets.push_back(std::move(desc));
    }
}

void emitParticleSampler(OvstageScanResult& out, ParseContext& ctx, ObjectKey key)
{
    if (DescPtr<ParticleSamplingDesc> desc = parseParticleSampling(ctx, key))
    {
        out.particleSamplers.push_back(std::move(desc));
        out.particleSamplerKeys.push_back(key);
    }
}

ObjectType attachmentSubtypeForPrimType(std::string_view typeName)
{
    if (typeName == "OmniPhysicsVtxVtxAttachment") return eAttachmentVtxVtx;
    if (typeName == "OmniPhysicsVtxTriAttachment") return eAttachmentVtxTri;
    if (typeName == "OmniPhysicsVtxTetAttachment") return eAttachmentVtxTet;
    if (typeName == "OmniPhysicsVtxCrvAttachment") return eAttachmentVtxCrv;
    if (typeName == "OmniPhysicsVtxXformAttachment") return eAttachmentVtxXform;
    if (typeName == "OmniPhysicsTetXformAttachment") return eAttachmentTetXform;
    if (typeName == "OmniPhysicsTriTriAttachment") return eAttachmentTriTri;
    return eUndefined;
}

void emitAttachment(OvstageScanResult& out, ParseContext& ctx, ObjectKey key, ObjectType subtype)
{
    if (subtype == eUndefined)
        return;
    if (DescPtr<PhysxDeformableAttachmentDesc> desc = parseAttachment(ctx, key, subtype))
    {
        desc->primKey = key;
        out.attachments.push_back(std::move(desc));
    }
}

void emitElementCollisionFilter(OvstageScanResult& out, ParseContext& ctx, ObjectKey key)
{
    if (DescPtr<PhysxDeformableCollisionFilterDesc> desc = parseElementCollisionFilter(ctx, key))
    {
        desc->primKey = key;
        out.deformableCollisionFilters.push_back(std::move(desc));
    }
}

// Character-controller emit — the ovstage peer of NativeWalker.cpp::emitCct.
// Requires the prim to be a Capsule gprim, reads its radius/height and world
// transform (translation + scale), and the simulationOwner relationship from
// PhysxCharacterControllerAPI. Per-axis scale baking matches the native walker
// exactly: radius scaled by scale.y, halfHeight by scale.z. parse-lib's
// source-agnostic parseCct resolves slopeLimit and the owner key.
void emitCct(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    // A CCT on non-capsule geometry is dropped, as in the native walker. That one
    // logs "CCT prim must be a capsule geom"; this module links no logging backend
    // (there is no carb/logging dependency in omni.physics.ovstage), so the drop is
    // silent here.
    if (!src.isA(key, src.internToken("Capsule")))
        return;

    // UsdGeomCapsule schema fallbacks: radius 0.5, height 1.0 -- the same pair the
    // capsule SHAPE path below uses. `getAttribute` leaves `out` untouched on a miss,
    // so these initializers ARE the fallback whenever the source has no resident USD
    // stage to fall back through. The native walker gets away with initializing
    // radius to 1.0 because `GetRadiusAttr().Get()` on a typed capsule always writes
    // the schema fallback; here 1.0 would silently double an unauthored CCT's width.
    double radiusAttr = 0.5, heightAttr = 1.0;
    src.getAttribute(key, src.internToken("radius"), radiusAttr);
    src.getAttribute(key, src.internToken("height"), heightAttr);

    Matrix4d world;
    src.getLocalToWorldTransform(key, world);
    Matrix3d rotation;
    carb::Float3 scale{ 1.0f, 1.0f, 1.0f };
    src.getLocalToWorldRotationAndScale(key, rotation, scale);

    CctInfo info;
    info.radius = scale.y * static_cast<float>(radiusAttr);
    info.halfHeight = scale.z * static_cast<float>(heightAttr) * 0.5f;
    info.scale = scale;
    info.pos = { static_cast<float>(world.data[12]), static_cast<float>(world.data[13]),
                 static_cast<float>(world.data[14]) };
    src.getRelationshipTargets(key, src.internToken("physxCharacterController:simulationOwner"),
                               info.simulationOwners);

    if (DescPtr<CapsuleCctDesc> desc = parseCct(ctx, key, info))
        out.ccts.push_back(std::move(desc));
}

bool isPointBased(const OvstageSource& src, ObjectKey key)
{
    return src.isA(key, src.internToken("PointBased"));
}

// A tet sim mesh is identified by its TET DATA, not by its concrete prim type.
//
// ovstage's `usd-prim-type` column reports a UsdGeomTetMesh as plain "Mesh" — the
// populator has no TetMesh mapping — and isType() is an exact string match with no
// inheritance, so requiring "TetMesh" here rejected every volume deformable whose
// sim mesh came from ovstage. The surface branch only worked by luck: its required
// type IS the fallback the populator emits.
//
// Gating on the presence of readable tetVertexIndices is both weaker in the right
// way and stronger in practice: OmniPhysicsVolumeDeformableSimAPI already declares
// intent, and tet connectivity is what the consumer actually needs. USD still
// satisfies the fast path, since there the prim really is a TetMesh.
// Presence check only — resolve the buffer to prove it is readable, but never copy it. A tet
// mesh's connectivity is one of the largest arrays on the prim and this runs per candidate prim.
bool hasTetConnectivity(const OvstageSource& src, ObjectKey key)
{
    const BufferHandle h = src.readArrayAttribute(key, "tetVertexIndices", BufferElemType::eInt4, 4);
    if (!h.valid())
        return false;
    size_t byteCount = 0;
    const bool readable = src.resolveBuffer(h, byteCount) != nullptr;
    const bool nonEmpty = readable && h.elemCount > 0 && byteCount > 0;
    src.releaseBuffer(h);
    return nonEmpty;
}

bool isDeformableSimMesh(const OvstageSource& src, ovstage_instance_t* inst, ObjectKey key, ObjectType& outType)
{
    outType = eUndefined;
    if (src.hasSchema(key, src.internToken("OmniPhysicsVolumeDeformableSimAPI")))
    {
        if (!isType(src, inst, key, "TetMesh") && !hasTetConnectivity(src, key))
            return false;
        outType = eVolumeDeformableBody;
        return true;
    }
    if (src.hasSchema(key, src.internToken("OmniPhysicsSurfaceDeformableSimAPI")))
    {
        if (!isType(src, inst, key, "Mesh"))
            return false;
        outType = eSurfaceDeformableBody;
        return true;
    }
    return false;
}

bool isEnabledCollisionGeom(const OvstageSource& src, ObjectKey key)
{
    if (!src.hasSchema(key, src.internToken("PhysicsCollisionAPI")))
        return false;
    if (!isPointBased(src, key))
        return false;
    bool enabled = true;
    const TokenId collisionEnabled = src.internToken("physics:collisionEnabled");
    if (!src.getAttribute(key, collisionEnabled, enabled))
    {
        const ObjectKey backing = src.collisionAttributeBackingKey(key);
        if (backing.valid() && backing != key)
            src.getAttribute(backing, collisionEnabled, enabled);
    }
    return enabled;
}

TokenId findPoseNameFromPurpose(OvstageSource& src, ObjectKey key, std::string_view purpose)
{
    TokenId out;
    const TokenId purposeToken = src.internToken(purpose);
    src.forEachMultiApplyInstance(key, "OmniPhysicsDeformablePoseAPI", [&](std::string_view inst)
    {
        if (out.valid())
            return;
        // ovstage currently has no token-array source contract. Support the common
        // scalar-token case if a producer publishes a single purpose value.
        const std::string attr = std::string("omniphysics:") + std::string(inst) + ":purposes";
        TokenId tok;
        if (src.getAttribute(key, src.internToken(attr), tok) && tok == purposeToken)
            out = src.internToken(inst);
    });
    return out;
}

void collectPointBasedSubtree(OvstageSource& src, ObjectKey root, std::vector<ObjectKey>& out)
{
    if (isPointBased(src, root))
        out.push_back(root);
    src.forEachChild(root, [&](ObjectKey child)
    {
        collectPointBasedSubtree(src, child, out);
    });
}

void emitDeformableBody(OvstageScanResult& out, ovstage_instance_t* inst, OvstageSource& src,
                        ParseContext& ctx, ObjectKey key, const SourceUnits& units,
                        std::unordered_map<uint64_t, bool>& deformableBodyEnabledByCanonical)
{
    const bool validGeomType = isType(src, inst, key, "Mesh") || isType(src, inst, key, "TetMesh") ||
                               isType(src, inst, key, "BasisCurves");
    const bool validRootType = isXformable(src, key) && !src.isA(key, src.internToken("Gprim"));
    if (!validGeomType && !validRootType)
        return;

    const DeformableBodyParse parsed = parseDeformableBody(ctx, key);

    ObjectKey simMeshKey;
    ObjectType simMeshType = eUndefined;
    std::vector<ObjectKey> collisionGeomKeys;
    std::vector<TokenId> collisionGeomBindPoseTokens;
    std::vector<bool> collisionGeomLeftHandedOrientations;
    std::vector<ObjectKey> skinGeomKeys;
    std::vector<TokenId> skinGeomBindPoseTokens;
    std::vector<ObjectKey> hierarchyFilteredPairs;

    std::vector<ObjectKey> pointBased;
    collectPointBasedSubtree(src, key, pointBased);
    for (ObjectKey pKey : pointBased)
    {
        ObjectType candidateType = eUndefined;
        const bool isSim = isDeformableSimMesh(src, inst, pKey, candidateType);
        const bool isColl = isEnabledCollisionGeom(src, pKey);
        if (isSim)
        {
            if (simMeshKey.valid())
                return;
            const bool isBodyPrim = src.canonicalPath(pKey) == src.canonicalPath(key);
            const bool isImmediateChild = src.canonicalPath(src.getParent(pKey)) == src.canonicalPath(key);
            if (!isBodyPrim && !isImmediateChild)
                return;
            simMeshKey = pKey;
            simMeshType = candidateType;
        }
        if (isColl)
        {
            collisionGeomKeys.push_back(pKey);
            collisionGeomBindPoseTokens.push_back(findPoseNameFromPurpose(src, pKey, "bindPose"));
            TokenId orientation;
            const TokenId leftHandedToken = src.internToken("leftHanded");
            const bool leftHandedOrientation = src.getAttribute(pKey, src.internToken("orientation"), orientation) &&
                                               orientation == leftHandedToken;
            collisionGeomLeftHandedOrientations.push_back(leftHandedOrientation);

            std::vector<ObjectKey> childPairs = parseFilteredPairs(ctx, pKey);
            hierarchyFilteredPairs.insert(hierarchyFilteredPairs.end(), childPairs.begin(), childPairs.end());
        }
        if (!isSim && !isColl)
        {
            skinGeomKeys.push_back(pKey);
            skinGeomBindPoseTokens.push_back(findPoseNameFromPurpose(src, pKey, "bindPose"));
        }
    }

    if (!simMeshKey.valid())
        return;

    DescPtr<PhysxDeformableBodyDesc> desc;
    if (simMeshType == eSurfaceDeformableBody)
    {
        DescPtr<PhysxSurfaceDeformableBodyDesc> surf = allocateDesc<PhysxSurfaceDeformableBodyDesc>(ctx.descriptorAllocator());
        setToDefault(*surf, units);
        desc = descPtrCast<PhysxDeformableBodyDesc>(std::move(surf));
    }
    else
    {
        DescPtr<PhysxVolumeDeformableBodyDesc> vol = allocateDesc<PhysxVolumeDeformableBodyDesc>(ctx.descriptorAllocator());
        setToDefault(*vol, units);
        desc = descPtrCast<PhysxDeformableBodyDesc>(std::move(vol));
    }

    desc->primKey = key;
    desc->bodyEnabled = parsed.bodyEnabled;
    desc->kinematicBody = parsed.kinematicBody;
    desc->startsAsleep = parsed.startsAsleep;
    desc->mass = (parsed.mass <= 0.0f) ? -1.0f : parsed.mass;
    desc->sourceSimulationOwners = parsed.simulationOwners;
    desc->sourceFilteredCollisions = parsed.filteredCollisions;
    desc->sourceFilteredCollisions.insert(desc->sourceFilteredCollisions.end(),
                                          hierarchyFilteredPairs.begin(), hierarchyFilteredPairs.end());

    Matrix4d transform;
    src.getLocalToWorldTransform(key, transform);
    std::memcpy(desc->transform.data, transform.data, sizeof(double) * 16);

    desc->simMeshKey = simMeshKey;
    desc->simMeshBindPoseToken = findPoseNameFromPurpose(src, simMeshKey, "bindPose");
    {
        TokenId orientation;
        const TokenId leftHandedToken = src.internToken("leftHanded");
        desc->simMeshLeftHandedOrientation =
            src.getAttribute(simMeshKey, src.internToken("orientation"), orientation) &&
            orientation == leftHandedToken;
    }

    if (!collisionGeomKeys.empty())
    {
        desc->collisionMeshKey = collisionGeomKeys.front();
        if (!collisionGeomBindPoseTokens.empty())
            desc->collisionMeshBindPoseToken = collisionGeomBindPoseTokens.front();
        if (!collisionGeomLeftHandedOrientations.empty())
            desc->collisionMeshLeftHandedOrientation = collisionGeomLeftHandedOrientations.front();
    }

    desc->skinGeomPaths = std::move(skinGeomKeys);
    desc->skinGeomBindPoseTokens = std::move(skinGeomBindPoseTokens);

    const TokenId baseDBTok = src.internToken("PhysxBaseDeformableBodyAPI");
    if (src.hasSchema(key, baseDBTok))
    {
        auto rdF = [&](const char* name, float def) {
            float v = def;
            src.getAttribute(key, src.internToken(name), v);
            return v;
        };
        auto rdB = [&](const char* name, bool def) {
            bool v = def;
            src.getAttribute(key, src.internToken(name), v);
            return v;
        };
        auto rdU = [&](const char* name, uint32_t def) {
            int64_t iv = 0;
            if (src.getAttribute(key, src.internToken(name), iv))
                return uint32_t(iv);
            return def;
        };
        desc->linearDamping = rdF("physxDeformableBody:linearDamping", desc->linearDamping);
        desc->maxLinearVelocity = rdF("physxDeformableBody:maxLinearVelocity", desc->maxLinearVelocity);
        desc->sleepThreshold = rdF("physxDeformableBody:sleepThreshold", desc->sleepThreshold);
        desc->settlingThreshold = rdF("physxDeformableBody:settlingThreshold", desc->settlingThreshold);
        desc->settlingDamping = rdF("physxDeformableBody:settlingDamping", desc->settlingDamping);
        desc->maxDepenetrationVelocity = rdF("physxDeformableBody:maxDepenetrationVelocity", desc->maxDepenetrationVelocity);
        desc->selfCollisionFilterDistance = rdF("physxDeformableBody:selfCollisionFilterDistance", desc->selfCollisionFilterDistance);
        desc->solverPositionIterationCount = rdU("physxDeformableBody:solverPositionIterationCount", desc->solverPositionIterationCount);
        desc->enableSpeculativeCCD = rdB("physxDeformableBody:enableSpeculativeCCD", desc->enableSpeculativeCCD);
        desc->selfCollision = rdB("physxDeformableBody:selfCollision", desc->selfCollision);
        desc->disableGravity = rdB("physxDeformableBody:disableGravity", desc->disableGravity);
    }

    if (simMeshType == eSurfaceDeformableBody && src.hasSchema(key, src.internToken("PhysxSurfaceDeformableBodyAPI")))
    {
        PhysxSurfaceDeformableBodyDesc* surf = static_cast<PhysxSurfaceDeformableBodyDesc*>(desc.get());
        int64_t freq = 0;
        if (src.getAttribute(key, src.internToken("physxDeformableBody:collisionPairUpdateFrequency"), freq))
            surf->collisionPairUpdateFrequency = uint32_t(freq);
        int64_t mult = 0;
        if (src.getAttribute(key, src.internToken("physxDeformableBody:collisionIterationMultiplier"), mult))
            surf->collisionIterationMultiplier = uint32_t(mult);
    }

    if (src.hasSchema(key, src.internToken("PhysxAutoDeformableBodyAPI")))
    {
        src.getAttribute(key, src.internToken("physxDeformableBody:autoDeformableBodyEnabled"), desc->hasAutoAPI);
        std::vector<ObjectKey> cookingTargets;
        src.getRelationshipTargets(key, src.internToken("physxDeformableBody:cookingSourceMesh"), cookingTargets);
        if (!cookingTargets.empty())
        {
            desc->cookingSrcMeshKey = cookingTargets.front();
            desc->cookingSrcMeshBindPoseToken = findPoseNameFromPurpose(src, cookingTargets.front(), "bindPose");
        }

        if (src.hasSchema(key, src.internToken("PhysxAutoDeformableMeshSimplificationAPI")))
        {
            auto rdU = [&](const char* name, uint32_t def) {
                int64_t iv = 0;
                if (src.getAttribute(key, src.internToken(name), iv))
                    return uint32_t(iv);
                return def;
            };
            auto rdB = [&](const char* name, bool def) {
                bool v = def;
                src.getAttribute(key, src.internToken(name), v);
                return v;
            };
            desc->isAutoMeshSimplificationEnabled = rdB("physxDeformableBody:autoDeformableMeshSimplificationEnabled",
                                                        desc->isAutoMeshSimplificationEnabled);
            desc->isAutoRemeshingEnabled = rdB("physxDeformableBody:remeshingEnabled", desc->isAutoRemeshingEnabled);
            desc->autoRemeshingResolution = rdU("physxDeformableBody:remeshingResolution", desc->autoRemeshingResolution);
            desc->autoTriangleTargetCount = rdU("physxDeformableBody:targetTriangleCount", desc->autoTriangleTargetCount);
            desc->hasAutoForceConforming = rdB("physxDeformableBody:forceConforming", desc->hasAutoForceConforming);
        }
        if (simMeshType != eSurfaceDeformableBody && src.hasSchema(key, src.internToken("PhysxAutoDeformableHexahedralMeshAPI")))
        {
            PhysxVolumeDeformableBodyDesc* vol = static_cast<PhysxVolumeDeformableBodyDesc*>(desc.get());
            vol->isAutoHexahedralMeshEnabled = true;
            int64_t resolution = 0;
            if (src.getAttribute(key, src.internToken("physxDeformableBody:resolution"), resolution))
                vol->autoHexahedralResolution = uint32_t(resolution);
        }
    }

    deformableBodyEnabledByCanonical[src.canonicalPath(key)] = parsed.bodyEnabled;
    out.deformables.push_back(std::move(desc));
}

OvstageScanResult scanOvstage(ovstage_instance_t* instance,
                              ovx_path_dictionary_t* dict,
                              IDescriptorAllocator& allocator,
                              ovstage_ordinal_t readOrdinal,
                              const OvstageScanFilter* filter,
                              uint64_t usdStageId,
                              OvstageSource* attached)
{
    OvstageScanResult out;
    if (!instance || !dict)
        return out;

    if (attached && attached->instance() != instance)
        attached = nullptr; // a foreign attach's source cannot mint keys for this instance
    std::unique_ptr<OvstageSource> owned;
    if (!attached)
        owned = std::make_unique<OvstageSource>(instance, dict, readOrdinal, usdStageId);
    OvstageSource& src = attached ? *attached : *owned;
    out.source = std::move(owned);
    out.borrowedSource = attached;
    // A consumer that opened a load-cache window on the attached source around its whole
    // incremental load (LoadStage::loadFromRange) shares it: the scan's prefetches then also
    // serve the consumer's follow-up reads, and the window is the consumer's to end.
    const bool ownWindow = !(attached && attached->loadCacheActive());
    if (ownWindow)
        src.beginLoadCache();
    // A borrowed source outlives the scan: drop the concept bucket with the load cache, or its
    // last prefetch would keep serving later reads. A caller that re-enters the scan with a
    // bucket of its own open (a drain's seeded read group, the mass update's prefetch, through
    // PointInstancer::parsePrototype or the collision-group scan) gets it back instead: the
    // scan's prefetches run on a suspended copy, mirroring the load-cache window join above.
    struct ScopedLoadCache
    {
        OvstageSource& source;
        bool endWindow;
        OvstageSource::BucketSnapshotPtr callerBucket;
        ~ScopedLoadCache()
        {
            if (endWindow)
                source.clearLoadCache();
            if (callerBucket)
                source.resumeBucket(std::move(callerBucket));
            else
                source.clearBucket();
        }
    } scopedLoadCache{ src, ownWindow, attached ? src.suspendBucket() : nullptr };

    ParseContext ctx(src, allocator);
    const SourceUnits units = src.getSourceUnits();
    const float massScale = (units.kilogramsPerUnit != 0.0f) ? (1.0f / units.kilogramsPerUnit) : 1.0f;
    const float lengthScale = (units.metersPerUnit != 0.0f) ? (1.0f / units.metersPerUnit) : 1.0f;
    const float kgmsScale = (lengthScale * lengthScale) * massScale;

    auto isRootPrimPath = [](std::string_view path) -> bool
    {
        return path.size() > 1 && path[0] == '/' && path.find('/', 1) == std::string_view::npos;
    };
    auto pathIsAtOrBelow = [](std::string_view path, std::string_view root) -> bool
    {
        if (root.empty() || root == "/")
            return true;
        return path == root || (path.size() > root.size() &&
                                path.compare(0, root.size(), root) == 0 &&
                                path[root.size()] == '/');
    };
    std::vector<std::string> pointInstancerSubtreeRoots;
    auto keyPassesFilter = [&](ObjectKey key) -> bool
    {
        if (src.isPrototypeBackingKey(key))
            return false;
        if (!filter || (filter->scanRoots.empty() && filter->excludePaths.empty()))
            return true;

        const std::string path(src.sourceKeyToString(key));
        bool inRoots = filter->scanRoots.empty();
        for (const std::string& root : filter->scanRoots)
        {
            if (pathIsAtOrBelow(path, root))
            {
                inRoots = true;
                break;
            }
        }
        if (!inRoots)
            return false;

        for (const std::string& exclude : filter->excludePaths)
        {
            if (isRootPrimPath(exclude))
                continue;
            if (pathIsAtOrBelow(path, exclude))
                return false;
        }
        return true;
    };
    auto isBelowPointInstancer = [&](ObjectKey key) -> bool
    {
        if (pointInstancerSubtreeRoots.empty())
            return false;
        const std::string path(src.sourceKeyToString(key));
        for (const std::string& root : pointInstancerSubtreeRoots)
            if (path != root && pathIsAtOrBelow(path, root))
                return true;
        return false;
    };
    // --- Deterministic enumeration order ---------------------------------------
    //
    // ovstage does NOT give a stage-derived enumeration order, from either of the
    // two routes a bucket's keys arrive by:
    //   - enumerate() unions the prims.list of every read group returned for a
    //     column, and ovstage emits those groups in an order that varies with
    //     process history. The same column with byte-identical group contents was
    //     observed coming back in three different orders depending on what had
    //     been attached earlier in the process.
    //   - collectSchemaKeys() serves from an unordered_set of prim handles, so its
    //     iteration order is hash order over handles that themselves migrate
    //     between id arenas as more ovstage instances are created.
    //
    // Scan order is load-bearing: Articulation.cpp::createArticulationLinks sorts
    // the joint vector by scan index ("required to maintain parsing order and
    // determinism") and createLinkHierarchy DFS-walks it, so scan order becomes
    // PhysX link and DOF indices. An unstable order therefore silently permutes
    // every tensor-view column for an identical asset — no error, matching counts,
    // only the meaning of the indices moves.
    //
    // Re-key every enumerated bucket to the source hierarchy's depth-first
    // traversal order. For valid absolute prim paths this is exactly one
    // decorate-sort by canonical path string: USD prim names are [A-Za-z0-9_]+ and
    // the '/' separator (0x2F) sorts below every legal name character, so
    // lexicographic full-path order IS preorder DFS with path-sorted siblings.
    // That guarantee needs no per-ancestor sibling query and no whole-stage walk,
    // and — unlike an ancestor-chain rank that routes each key to its immediate
    // parent's observed child set — it stays correct when an intermediate
    // hierarchy level has no queryable row of its own: a child's full path sorts
    // into position whether or not its typeless "/World"-style ancestors are
    // separately enumerable (e.g. "/World/a/body" before "/World/z" even when
    // "/World/a" is never authored as its own row).
    //
    // This makes ovstage agree with ITSELF across process histories. It does
    // NOT make it agree with the USD backend: ovstage exposes no authoring
    // order, and USD namespace order is authoring order, not path order.
    // Articulation link/DOF indices therefore still differ between the two
    // backends — closing that gap needs authoring order from ovstage.
    //
    // The sort key is the resolved canonical path string (never ObjectKey.handle
    // -- ADR-0021): ObjectKey packs a per-instance local index over whichever raw
    // handle happened to be interned first for a path, so two ObjectKeys naming
    // the same prim are not guaranteed to carry the same packed handle. The path
    // string is the one stable, alias-independent identity this ordering relies
    // on, and OvstageSource resolves it from an interned per-key cache (no ovstage
    // query, no stage walk). Keys whose path cannot be resolved (unparented /
    // private rows) have no namespace position and are parked after the ranked
    // keys, ordered among themselves by that same string, so the result is fully
    // determined either way (REQ-PARSE-SCAN-001 AC-14).
    auto orderBucketKeys = [&](std::vector<ObjectKey>& keys)
    {
        if (keys.size() < 2)
            return;
        std::vector<std::pair<std::string, ObjectKey>> decorated;
        decorated.reserve(keys.size());
        for (const ObjectKey key : keys)
            decorated.emplace_back(std::string(src.sourceKeyToString(key)), key);
        std::stable_sort(decorated.begin(), decorated.end(),
                         [](const std::pair<std::string, ObjectKey>& a,
                            const std::pair<std::string, ObjectKey>& b) -> bool
                         {
                             // Ranked (path-resolved) keys sort before parked
                             // (unresolved-path) keys; within each group, by path.
                             if (a.first.empty() != b.first.empty())
                                 return !a.first.empty();
                             return a.first < b.first;
                         });
        for (size_t i = 0; i < keys.size(); ++i)
            keys[i] = decorated[i].second;
    };

    auto applyScanFilter = [&](Bucket bucket) -> Bucket
    {
        if (bucket.keys.empty())
            return bucket;
        bucket.keys.erase(std::remove_if(bucket.keys.begin(), bucket.keys.end(),
                                         [&](ObjectKey key)
                                         {
                                             return !keyPassesFilter(key) || isBelowPointInstancer(key);
                                         }),
                          bucket.keys.end());
        orderBucketKeys(bucket.keys);
        return bucket;
    };
    auto enumerateFiltered = [&](const char* predAttr, ovstage_filter_op_t op, const char* predValue,
                                 const char* probeAttr, ovstage_ordinal_t ordinal) -> Bucket
    {
        return applyScanFilter(enumerate(src, instance, dict, predAttr, op, predValue, probeAttr, ordinal, nullptr));
    };
    auto enumeratePrimTypeFiltered = [&](const char* typeName, const char* probeAttr,
                                         ovstage_ordinal_t ordinal) -> Bucket
    {
        // A family the source's type index knows to be empty costs no whole-stage query; a
        // populated one still enumerates live, which also discovers the columns its bucket
        // prefetches.
        std::vector<ObjectKey> indexed;
        if (src.collectPrimTypeKeys(typeName, indexed))
        {
            // Nor does a populated family none of whose prims the scan's filter would keep (a
            // scoped rescan below a spawned prim, with the scene elsewhere): the live enumerate
            // could only return keys applyScanFilter drops.
            const bool anyInScope = std::any_of(indexed.begin(), indexed.end(), [&](ObjectKey key)
                                                { return keyPassesFilter(key) && !isBelowPointInstancer(key); });
            if (!anyInScope)
                return Bucket{};
        }
        Bucket bucket = enumerate(src, instance, dict, "usd-prim-type", OVSTAGE_FILTER_OP_IN,
                                  typeName, probeAttr, ordinal, conv::kUsdPath);
        return applyScanFilter(std::move(bucket));
    };
    auto enumerateSchemaFiltered = [&](const char* schemaName, const char* probeAttr,
                                       ovstage_ordinal_t ordinal) -> Bucket
    {
        const TokenId schemaToken = src.internToken(schemaName);
        std::vector<ObjectKey> cachedKeys;
        if (src.collectSchemaKeys(schemaToken, cachedKeys))
        {
            if (cachedKeys.empty())
                return applyScanFilter(Bucket{});

            (void)probeAttr;
            (void)ordinal;
            Bucket bucket;
            bucket.keys = std::move(cachedKeys);
            return applyScanFilter(std::move(bucket));
        }

        Bucket bucket = enumerateSchema(src, instance, dict, schemaName, probeAttr, ordinal);
        return applyScanFilter(bucket);
    };
    auto enumerateSchemasFiltered = [&](const char* const* schemaNames, size_t schemaCount, const char* probeAttr,
                                        ovstage_ordinal_t ordinal) -> Bucket
    {
        Bucket outBucket;
        for (size_t i = 0; i < schemaCount; ++i)
        {
            Bucket one = enumerateSchemaFiltered(schemaNames[i], probeAttr, ordinal);
            appendUnique(src, outBucket.keys, one.keys);
            for (const std::string& attr : one.attrs)
            {
                if (std::find(outBucket.attrs.begin(), outBucket.attrs.end(), attr) == outBucket.attrs.end())
                    outBucket.attrs.push_back(attr);
            }
        }
        // Each per-schema bucket is namespace-ordered on its own, but appending
        // them yields [schema0 keys][schema1 keys]. Re-order the union so a prim
        // carrying several of the schemas lands at its namespace position rather
        // than at whichever schema happened to claim it first.
        orderBucketKeys(outBucket.keys);
        return outBucket;
    };

    const TokenId tokRigidBodyEnabled = src.internToken("physics:rigidBodyEnabled");
    const TokenId tokGravityDir = src.internToken("physics:gravityDirection");
    const TokenId tokGravityMag = src.internToken("physics:gravityMagnitude");
    const TokenId tokSize = src.internToken("size");
    const TokenId tokRadius = src.internToken("radius");
    const TokenId tokHeight = src.internToken("height");
    const TokenId tokAxis = src.internToken("axis");
    const TokenId tokAxisX = src.internToken("X");
    const TokenId tokAxisY = src.internToken("Y");
    const TokenId tokAxisZ = src.internToken("Z");
    const TokenId tokDoubleSided = src.internToken("doubleSided");
    const TokenId tokRigidBodyAPI = src.internToken("PhysicsRigidBodyAPI");
    const TokenId tokCollisionAPI = src.internToken("PhysicsCollisionAPI");
    const TokenId tokCollisionEnabled = src.internToken("physics:collisionEnabled");
    const TokenId tokSimulationOwner = src.internToken("physics:simulationOwner");
    const TokenId tokGeomSubset = src.internToken("GeomSubset");
    const TokenId tokElementType = src.internToken("elementType");
    const TokenId tokFace = src.internToken("face");
    const TokenId tokPhysicsMaterialAPI = src.internToken("PhysicsMaterialAPI");

    const bool prunePointInstancerDescendants = !filter || filter->prunePointInstancerDescendants;
    auto collectPointInstancerPrimType = [&](const char* typeName)
    {
        const Bucket pointInstancerBucket = enumeratePrimTypeFiltered(typeName, conv::kLocalTransform, readOrdinal);
        if (!pointInstancerBucket.keys.empty())
            out.hasPointInstancerPrims = true;
        if (!prunePointInstancerDescendants)
            return;
        pointInstancerSubtreeRoots.reserve(pointInstancerSubtreeRoots.size() + pointInstancerBucket.keys.size());
        for (const ObjectKey key : pointInstancerBucket.keys)
            pointInstancerSubtreeRoots.emplace_back(src.sourceKeyToString(key));
    };
    collectPointInstancerPrimType("PointInstancer");
    collectPointInstancerPrimType("PhysxPhysicsJointInstancer");
    // Consumer-registered instancer-shaped prim types (REQ-PARSE-CORE-005). The
    // native walker tests every prim's type against the registry; ovstage
    // enumerates instead, so this is one extra query per registered token —
    // ADR-0002 open question 1. "PhysxPhysicsJointInstancer" is pre-registered
    // and already collected above; skip it rather than query it twice.
    for (const std::string& instancerToken : parse::customTokens(parse::CustomTokenKind::ePhysicsInstancer))
    {
        if (instancerToken != "PointInstancer" && instancerToken != "PhysxPhysicsJointInstancer")
            collectPointInstancerPrimType(instancerToken.c_str());
    }

    // --- Scenes (PRIM_TYPE == PhysicsScene) ---
    Bucket sceneBucket = enumeratePrimTypeFiltered("PhysicsScene", "physics:gravityMagnitude", readOrdinal);
    const std::vector<std::string> sceneDiscoveredAttrs = sceneBucket.attrs;
    const size_t sceneAttrCount = sceneBucket.attrs.size();
    appendSceneAttrs(sceneBucket);
    src.prefetchBucket(sceneBucket.keys, sceneDiscoveredAttrs);
    src.prefetchBucket(sceneBucket.keys, appendedAttrs(sceneBucket, sceneAttrCount));
    for (const ObjectKey key : sceneBucket.keys)
    {
        DescPtr<PhysxSceneDesc> desc = allocateDesc<PhysxSceneDesc>(allocator);
        if (!desc)
            continue;
        setToDefault(*desc, units);

        SceneInfo info;
        carb::Float3 dir{};
        float mag = 0.0f;
        if (src.getAttribute(key, tokGravityDir, dir))
            info.gravityDirection = dir;
        if (src.getAttribute(key, tokGravityMag, mag))
            info.gravityMagnitude = mag;

        parseScene(ctx, key, info, *desc);
        desc->primKey = key;
        out.scenes.push_back(std::move(desc));
    }
    src.clearBucket();

    // No authored PhysicsScene → synthesize a default scene descriptor so a lone
    // body still simulates (parity with the USD path, which authors a temp scene
    // prim). ovstage authors NO prim: this is an in-memory descriptor flagged
    // `synthetic`, with a stable synthetic path so pathFor/keyFor round-trip in the
    // consumer (which skips the source existence/ownership gates for it). See
    // makeDefaultSceneDesc + LoadStage processScannedDescs.
    //
    // "This stage authors no PhysicsScene" is a WHOLE-STAGE fact and cannot be
    // concluded from a SCOPED scan. An incremental re-scan rooted at a newly added
    // prim (PrimUpdateMap::addPrim) normally contains no scene, so synthesizing one
    // here published a spurious extra `/__defaultPhysicsScene__` on every runtime
    // prim add — measured as a third object-created notification where USD reports
    // two. The USD path has no walker-side synthesis at all: LoadStage creates its
    // default scene only when `initialStageLoad && !mSceneFound`. Gating on an
    // unscoped scan is the ovstage mirror of that, and a scoped initial load still
    // gets LoadStage's own fallback.
    //
    // An exclude-only filter scopes the scan just as much as a `scanRoots` one does --
    // the excluded subtree is exactly where the authored scene may live -- so it must
    // not synthesize either. "Scoped" here has to mean the same thing it means to
    // `keyPassesFilter` above, which *ignores* a root-prim-level exclude (see
    // `LoadStage.cpp::forEachLoadObject` for why: the legacy `Traverse()` walk yielded
    // the root prim unchecked, so excluding e.g. "/World" was always a no-op). An
    // exclude list holding only such paths therefore prunes nothing and leaves the
    // scan genuinely whole-stage.
    //
    // Scope note: no in-repo runtime caller constructs an exclude-only filter today.
    // `loadFromStage` passes `scanRoots{SdfPath::AbsoluteRootPath()}` on every load,
    // selective or not (`LoadStage.cpp`), so the production selective-load path was
    // already scoped by its roots. This gate is the direct `scanOvstage` /
    // `IScanBackend` contract, pinned by `TestOvstageWalker.cpp`.
    const bool excludesScopeTheScan =
        filter && std::any_of(filter->excludePaths.begin(), filter->excludePaths.end(),
                              [&](const std::string& exclude) { return !isRootPrimPath(exclude); });
    const bool wholeStageScan = !filter || (filter->scanRoots.empty() && !excludesScopeTheScan);
    if (out.scenes.empty() && wholeStageScan)
    {
        if (DescPtr<PhysxSceneDesc> def = makeDefaultSceneDesc(allocator, units))
        {
            def->primKey = src.findByPath(conv::kDefaultScenePath);
            out.scenes.push_back(std::move(def));
        }
    }

    // --- Materials (HAS_APPLIED_SCHEMA PhysicsMaterialAPI) ---
    // Emit the material descriptors (friction / restitution / density). The
    // material→shape binding is resolved by ovpopulation as a target-path
    // relationship and read back via OvstageSource::getMaterialBinding, wired
    // into each shape's sourceMaterials in the shape loop below.
    Bucket materialBucket = enumerateSchemaFiltered("PhysicsMaterialAPI",
                                                    "physics:dynamicFriction",
                                                    readOrdinal);
    const std::vector<std::string> materialDiscoveredAttrs = materialBucket.attrs;
    const size_t materialAttrCount = materialBucket.attrs.size();
    appendMaterialAttrs(materialBucket);
    src.prefetchBucket(materialBucket.keys, materialDiscoveredAttrs);
    src.prefetchBucket(materialBucket.keys, appendedAttrs(materialBucket, materialAttrCount));
    for (const ObjectKey key : materialBucket.keys)
    {
        DescPtr<PhysxMaterialDesc> mat = parseMaterial(ctx, key);
        if (!mat)
            continue;
        mat->materialKey = key; // parseMaterial sets this; keep explicit alongside the other concepts
        out.materials.push_back(std::move(mat));
    }
    src.clearBucket();

    // --- PBD materials (PhysxPBDMaterialAPI) ---
    // ovpopulation currently exposes material prims through PhysicsMaterialAPI;
    // probe those visible material keys for PBD attributes because usd-schemas may
    // not include PhysxPBDMaterialAPI under ovstage yet.
    const std::vector<std::string> pbdMaterialAttrs = {
        "physxPBDMaterial:friction",
        "physxPBDMaterial:particleFrictionScale",
        "physxPBDMaterial:damping",
        "physxPBDMaterial:viscosity",
        "physxPBDMaterial:vorticityConfinement",
        "physxPBDMaterial:surfaceTension",
        "physxPBDMaterial:cohesion",
        "physxPBDMaterial:adhesion",
        "physxPBDMaterial:particleAdhesionScale",
        "physxPBDMaterial:adhesionOffsetScale",
        "physxPBDMaterial:gravityScale",
        "physxPBDMaterial:cflCoefficient",
        "physxPBDMaterial:density",
    };
    src.prefetchBucket(materialBucket.keys, pbdMaterialAttrs);
    for (const ObjectKey key : materialBucket.keys)
    {
        DescPtr<PBDMaterialDesc> mat = parsePBDMaterial(ctx, key);
        if (!mat)
            continue;
        mat->materialKey = key;
        out.pbdMaterials.push_back(std::move(mat));
    }
    src.clearBucket();

    // --- Deformable materials (OmniPhysicsDeformableMaterialAPI family) ---
    const char* const deformableMaterialSchemas[] = {
        "OmniPhysicsDeformableMaterialAPI",
        "OmniPhysicsSurfaceDeformableMaterialAPI",
        "OmniPhysicsCurveDeformableMaterialAPI",
    };
    const Bucket deformableMaterialBucket = enumerateSchemasFiltered(deformableMaterialSchemas, 3,
                                                             "omniphysics:youngsModulus", readOrdinal);
    src.prefetchBucket(deformableMaterialBucket.keys, deformableMaterialBucket.attrs);
    for (const ObjectKey key : deformableMaterialBucket.keys)
        emitDeformableMaterial(out, src, ctx, key);
    src.clearBucket();

    // --- Rigid bodies (HAS_APPLIED_SCHEMA PhysicsRigidBodyAPI) ---
    std::unordered_set<ObjectKey, ObjectKey::Hash> bodySet;
    std::unordered_map<ObjectKey, PhysxRigidBodyDesc*, ObjectKey::Hash> bodyByKey;
    // canonical intern_path id → body's enumerate key. Populated relationship
    // targets (physics:body0/1) are canonical intern_path ids, which the ovstage
    // lib hands back DISTINCT from the enumerate/get_paths handles in `bodySet`
    // for the same path. This maps a canonical handle back to the enumerate body
    // key so a resolved body matches the body descriptor's primKey.
    std::unordered_map<uint64_t, ObjectKey> bodyByCanonical;

    Bucket bodyBucket = enumerateSchemaFiltered("PhysicsRigidBodyAPI",
                                                conv::kLocalTransform,
                                                readOrdinal);
    const std::vector<std::string> bodyDiscoveredAttrs = bodyBucket.attrs;
    const size_t bodyAttrCount = bodyBucket.attrs.size();
    appendRigidBodyAttrs(bodyBucket);
    appendMassAttrs(bodyBucket);

    // The collision-shape family is enumerated here (the shape section below reuses it) so ONE
    // columnar read can cover both families: bodies, shapes, their geometry backings and every
    // ancestor, times every column the two sections and the consumer's follow-ups (mass update,
    // actor setup) ask. Under the load cache the per-section prefetches below then find their
    // columns covered and read nothing; usd-path in the set seeds the existence memo for the
    // ancestor chains the actor setup walks.
    Bucket shapeBucket = enumerateSchemaFiltered("PhysicsCollisionAPI", conv::kLocalTransform, readOrdinal);
    const std::vector<std::string> shapeDiscoveredAttrs = shapeBucket.attrs;
    const size_t shapeAttrCount = shapeBucket.attrs.size();
    appendCollisionShapeAttrs(shapeBucket);
    appendMassAttrs(shapeBucket);
    std::vector<ObjectKey> shapeReadKeys = shapeBucket.keys;
    {
        std::unordered_set<uint64_t> shapeReadCanonical;
        shapeReadCanonical.reserve(shapeBucket.keys.size() * 3);
        for (const ObjectKey key : shapeBucket.keys)
        {
            const uint64_t canonical = src.canonicalPath(key);
            if (canonical)
                shapeReadCanonical.insert(canonical);
        }
        for (const ObjectKey key : shapeBucket.keys)
        {
            const ObjectKey backingKey = src.geometryBackingKey(key);
            const uint64_t canonical = src.canonicalPath(backingKey);
            if (canonical && shapeReadCanonical.insert(canonical).second)
                shapeReadKeys.push_back(backingKey);
            const ObjectKey collisionBackingKey = src.collisionAttributeBackingKey(key);
            const uint64_t collisionCanonical = src.canonicalPath(collisionBackingKey);
            if (collisionCanonical && shapeReadCanonical.insert(collisionCanonical).second)
                shapeReadKeys.push_back(collisionBackingKey);
        }
    }
    {
        Bucket merged;
        std::unordered_set<uint64_t> mergedIdentity; // canonical prim id: one row per prim
        auto addKeys = [&](const std::vector<ObjectKey>& keys)
        {
            for (const ObjectKey key : keys)
            {
                const uint64_t canonical = src.canonicalPath(key);
                if (mergedIdentity.insert(canonical ? canonical : key.handle).second)
                    merged.keys.push_back(key);
            }
        };
        addKeys(bodyBucket.keys);
        addKeys(shapeReadKeys);
        addKeys(src.collectAncestors(merged.keys));
        appendTransformAttrs(merged);
        for (const std::string& attr : OvstageSource::relationshipPrefetchAttrs())
            appendBucketAttr(merged, attr.c_str());
        for (const Bucket* family : { &bodyBucket, &shapeBucket })
            for (const std::string& attr : family->attrs)
                appendBucketAttr(merged, attr.c_str());
        appendBucketAttr(merged, conv::kUsdPath);
        src.prefetchBucket(merged.keys, merged.attrs);
    }

    prefetchTransformAncestors(src, bodyBucket.keys);
    src.prefetchRelationshipAncestors(bodyBucket.keys);
    prefetchTransformsForKeys(src, bodyBucket.keys);
    src.prefetchBucket(bodyBucket.keys, bodyDiscoveredAttrs);
    src.prefetchBucket(bodyBucket.keys, appendedAttrs(bodyBucket, bodyAttrCount));
    for (const ObjectKey key : bodyBucket.keys)
    {
        bool enabled = true;
        bool enabledVal = false;
        if (src.getAttribute(key, tokRigidBodyEnabled, enabledVal))
            enabled = enabledVal;

        DescPtr<PhysxRigidBodyDesc> base;
        if (enabled)
        {
            DescPtr<DynamicPhysxRigidBodyDesc> dyn = parseDynamicBody(ctx, key);
            if (!dyn)
                continue;
            base = descPtrCast<PhysxRigidBodyDesc>(std::move(dyn));
        }
        else
        {
            DescPtr<StaticPhysxRigidBodyDesc> stat = parseStaticBody(ctx, key);
            if (!stat)
                continue;
            stat->sourceGPrimKey = key;
            base = descPtrCast<PhysxRigidBodyDesc>(std::move(stat));
        }

        base->primKey = key;
        // World-space body pose from the resolved transform. The engine creates
        // actors from position + rotation, so preserve both here.
        Matrix4d m;
        src.getLocalToWorldTransform(key, m);
        base->position = { static_cast<float>(m.data[12]), static_cast<float>(m.data[13]),
                           static_cast<float>(m.data[14]) };
        Matrix3d rotation;
        carb::Float3 scale{ 1.0f, 1.0f, 1.0f };
        src.getLocalToWorldRotationAndScale(key, rotation, scale);
        base->rotation = quatFromRotation(rotation);
        base->scale = scale;

        // Per-body collision filtering (PhysxFilteredPairsAPI → physics:filteredPairs);
        // no-op when the API is absent.
        base->sourceFilteredCollisions = parseFilteredPairs(ctx, key);
        src.getRelationshipTargets(key, tokSimulationOwner, base->sourceSimulationOwners);

        bodySet.insert(key);
        bodyByKey.emplace(key, base.get());
        bodyByCanonical.emplace(src.canonicalPath(key), key);
        out.bodies.push_back(std::move(base));
    }
    src.clearBucket();

    // Resolve the nearest enclosing rigid body for a collision shape, walking
    // the path-parent chain (mirrors the native walker's body-of-shape walk).
    // Ovstage can hand back distinct ObjectKey handles for the same path across
    // schema queries, so compare canonical path ids and return the body query key.
    auto resolveBody = [&](ObjectKey shapeKey) -> ObjectKey
    {
        ObjectKey k = shapeKey;
        for (int guard = 0; k.valid() && guard < 64; ++guard)
        {
            const auto it = bodyByCanonical.find(src.canonicalPath(k));
            if (it != bodyByCanonical.end())
                return it->second;
            k = src.getParent(k);
        }
        return {};
    };

    // --- Deformable bodies (HAS_APPLIED_SCHEMA OmniPhysicsDeformableBodyAPI) ---
    // Parse before rigid collision shapes so deformable collision children can be
    // skipped by the rigid-shape scan, matching the native walker.
    std::unordered_map<uint64_t, bool> deformableBodyEnabledByCanonical;
    Bucket deformableBodyBucket = enumerateSchemaFiltered("OmniPhysicsDeformableBodyAPI",
                                                          conv::kLocalTransform, readOrdinal);
    const std::vector<std::string> deformableBodyDiscoveredAttrs = deformableBodyBucket.attrs;
    const size_t deformableBodyAttrCount = deformableBodyBucket.attrs.size();
    appendDeformableBodyAttrs(deformableBodyBucket);
    prefetchTransformAncestors(src, deformableBodyBucket.keys);
    prefetchTransformsForKeys(src, deformableBodyBucket.keys);
    src.prefetchBucket(deformableBodyBucket.keys, deformableBodyDiscoveredAttrs);
    src.prefetchBucket(deformableBodyBucket.keys, appendedAttrs(deformableBodyBucket, deformableBodyAttrCount));
    for (const ObjectKey key : deformableBodyBucket.keys)
        emitDeformableBody(out, instance, src, ctx, key, units, deformableBodyEnabledByCanonical);
    src.clearBucket();

    auto isDeformableCollider = [&](ObjectKey shapeKey) -> bool
    {
        ObjectKey k = shapeKey;
        for (int guard = 0; k.valid() && guard < 64; ++guard)
        {
            auto it = deformableBodyEnabledByCanonical.find(src.canonicalPath(k));
            if (it != deformableBodyEnabledByCanonical.end() && it->second)
                return true;
            k = src.getParent(k);
        }
        return false;
    };

    // Resolve a USD round-shape `axis` token (X/Y/Z) to a parse::Axis; USD's
    // default for capsule/cylinder/cone is Z. ovstage stores the token as an id
    // column, so keep it in source token space.
    auto readAxis = [&](ObjectKey key) -> Axis
    {
        TokenId at{};
        if (src.getAttribute(key, tokAxis, at) && at.valid())
        {
            if (at == tokAxisX)
                return eX;
            if (at == tokAxisY)
                return eY;
        }
        return eZ;
    };

    // --- Collision shapes (HAS_APPLIED_SCHEMA PhysicsCollisionAPI) ---
    // Analytic and mesh geometry keep logical identity and transforms on an
    // instance-proxy path while reading authored geometry from its prototype. The family
    // (shapeBucket / shapeReadKeys) was enumerated with the rigid bodies above; the merged read
    // there already covers these prefetches under the load cache.
    prefetchTransformAncestors(src, shapeBucket.keys);
    src.prefetchRelationshipAncestors(shapeBucket.keys);
    prefetchTransformsForKeys(src, shapeBucket.keys);
    src.prefetchBucket(shapeReadKeys, shapeDiscoveredAttrs);
    src.prefetchBucket(shapeReadKeys, appendedAttrs(shapeBucket, shapeAttrCount));
    enum class PrimType : uint8_t
    {
        eCube,
        eSphere,
        eCapsule,
        eCylinder,
        eCone,
        ePlane,
        eMesh,
        eNone, // resolved, but not one of the analytic types above (e.g. Xform)
    };
    // No eager stage-wide prefetch here: primTypeIs() below is a per-key point
    // query (usd-path IN [key] AND usd-prim-type IN [type], never a subtree or
    // prefix scan), so its cost is bounded by the number of collision shapes
    // actually classified, not by stage size. A prior version of this code
    // prefetched every prim of each analytic type stage-wide into
    // primTypeByCanonical up front -- including render-only geometry that has
    // nothing to do with physics -- which made attach cost scale with total
    // scene size instead of physics-relevant shape count (NVBug 6532970).
    // primTypeByCanonical is instead populated lazily by primTypeIs() the first
    // time a key's type resolves, so repeat probes for the same shape are served
    // from the map.
    std::unordered_map<uint64_t, PrimType> primTypeByCanonical;
    auto primTypeName = [](PrimType primType) -> const char*
    {
        switch (primType)
        {
        case PrimType::eCube: return "Cube";
        case PrimType::eSphere: return "Sphere";
        case PrimType::eCapsule: return "Capsule";
        case PrimType::eCylinder: return "Cylinder";
        case PrimType::eCone: return "Cone";
        case PrimType::ePlane: return "Plane";
        case PrimType::eMesh: return "Mesh";
        case PrimType::eNone: return nullptr;
        }
        return nullptr;
    };
    // Map an authored USD type name to one of the analytic PrimType buckets. Returns
    // eNone for any prim whose type is not analytic geometry (Xform colliders, etc.).
    auto primTypeFromName = [](std::string_view s) -> PrimType
    {
        if (s == "Cube") return PrimType::eCube;
        if (s == "Sphere") return PrimType::eSphere;
        if (s == "Capsule") return PrimType::eCapsule;
        if (s == "Cylinder") return PrimType::eCylinder;
        if (s == "Cone") return PrimType::eCone;
        if (s == "Plane") return PrimType::ePlane;
        if (s == "Mesh") return PrimType::eMesh;
        return PrimType::eNone;
    };
    auto primTypeIs = [&](ObjectKey key, PrimType primType) -> bool
    {
        const uint64_t canonical = src.canonicalPath(key);
        if (!canonical)
            return false;
        std::unordered_map<uint64_t, PrimType>::const_iterator it = primTypeByCanonical.find(canonical);
        if (it != primTypeByCanonical.end())
            return it->second == primType;

        // Resolve the prim type once (getTypeName reads the geometry backing; tetMesh
        // reports as "Mesh") so the analytic-type probes below hit the cache.
        const TokenId actualTypeToken = src.getTypeName(key);
        if (actualTypeToken.valid())
        {
            const std::string_view actualTypeName = src.tokenToString(actualTypeToken);
            if (!actualTypeName.empty())
            {
                const PrimType resolved = primTypeFromName(actualTypeName);
                primTypeByCanonical.emplace(canonical, resolved);
                return resolved == primType;
            }
        }

        // usd-prim-type unreadable: keep the uncached per-type probe.
        const char* typeName = primTypeName(primType);
        if (!typeName)
            return false;
        ovx_token_t typeToken = OVX_INVALID_TOKEN;
        if (ovx_path_dictionary_intern_token(dict, ovxStr(typeName), &typeToken) == OVX_OK &&
            tokenColumnContains(instance, dict, src, key, conv::kUsdPrimType, typeToken, readOrdinal))
        {
            // Remember the resolved type: the classifier calls primTypeIs() for
            // several candidate types per shape (and Mesh twice, for subset
            // materials), so caching the positive result answers the repeat and
            // every subsequent candidate from the map instead of paying another
            // synchronous usd-prim-type / isType() probe per shape.
            primTypeByCanonical.emplace(canonical, primType);
            return true;
        }
        if (isType(src, instance, key, typeName))
        {
            primTypeByCanonical.emplace(canonical, primType);
            return true;
        }
        return false;
    };

    // Consumer-registered custom-geometry tokens (REQ-PARSE-CORE-005). A token
    // matches either as an applied API on the collider or as its prim type, so
    // both are resolved here: applied APIs through the source's schema map, prim
    // types through one query per token (ADR-0002 open question 1) folded into a
    // canonical-path -> token index the way `rememberPrimType` does for the
    // analytic types.
    //
    // `PhysxMeshMergeCollisionAPI` is deliberately excluded. It is pre-registered
    // as a shape token but the native walker routes it to a *typed* mesh-merge
    // descriptor (`buildMeshMergeCustomShape`), not to a generic
    // CustomPhysxShapeDesc, and that builder has no ovstage counterpart. Letting
    // it through here would replace today's behaviour (children picked up as
    // ordinary colliders) with a custom shape whose hash no consumer has
    // registered — strictly worse. Mesh merge stays exactly as measured in
    // PLAN-ovstage-test-coverage-completion.md §6 gap #16.
    static const std::string kMeshMergeToken = "PhysxMeshMergeCollisionAPI";
    const std::vector<std::string> customShapeTokens = parse::customTokens(parse::CustomTokenKind::eShape);
    std::vector<TokenId> customShapeSchemaTokens;
    customShapeSchemaTokens.reserve(customShapeTokens.size());
    std::unordered_map<uint64_t, size_t> customShapeTokenByCanonical;
    for (size_t i = 0; i < customShapeTokens.size(); ++i)
    {
        const std::string& shapeToken = customShapeTokens[i];
        customShapeSchemaTokens.push_back(shapeToken == kMeshMergeToken ? TokenId{} :
                                                                          src.internToken(shapeToken));
        if (shapeToken == kMeshMergeToken)
            continue;
        const Bucket tokenBucket = enumeratePrimTypeFiltered(shapeToken.c_str(), conv::kLocalTransform, readOrdinal);
        for (const ObjectKey tokenKey : tokenBucket.keys)
        {
            const uint64_t canonical = src.canonicalPath(tokenKey);
            if (canonical)
                customShapeTokenByCanonical.emplace(canonical, i);
        }
    }
    // Index of the registered token that makes `colliderKey` a custom shape, or
    // npos. Applied APIs are tested first, matching the native walker's
    // `isCustomShapeApplied`, which scans the applied-API list before falling
    // back to the prim type.
    auto findCustomShapeToken = [&](ObjectKey colliderKey) -> size_t
    {
        if (customShapeTokens.empty())
            return std::string::npos;
        for (size_t i = 0; i < customShapeSchemaTokens.size(); ++i)
        {
            if (customShapeSchemaTokens[i].valid() && src.hasSchema(colliderKey, customShapeSchemaTokens[i]))
                return i;
        }
        const uint64_t canonical = src.canonicalPath(colliderKey);
        if (canonical)
        {
            const std::unordered_map<uint64_t, size_t>::const_iterator it =
                customShapeTokenByCanonical.find(canonical);
            if (it != customShapeTokenByCanonical.end())
                return it->second;
        }
        return std::string::npos;
    };

    // Build and register one shape for a (collider, logical gprim) pair.
    // colliderKey owns collision state and gprimKey owns runtime identity,
    // transforms, and material. geometryKey is private ovstage backing for
    // prototype-authored type and geometry values. Selected collision values
    // may use colliderBackingKey only after their logical read misses.
    auto emitShape = [&](ObjectKey colliderKey, ObjectKey gprimKey) -> bool
    {
        if (isDeformableCollider(colliderKey))
            return true;
        const ObjectKey geometryKey = src.geometryBackingKey(gprimKey);
        const ObjectKey colliderBackingKey = src.collisionAttributeBackingKey(colliderKey);

        ShapeInfo info;
        info.rigidBody = resolveBody(colliderKey);
        info.sourceGprim = gprimKey;
        info.collisionAttributeFallback = colliderBackingKey;
        if (!src.getAttribute(colliderKey, tokCollisionEnabled, info.collisionEnabled) &&
            colliderBackingKey.valid() && colliderBackingKey != colliderKey)
            src.getAttribute(colliderBackingKey, tokCollisionEnabled, info.collisionEnabled);
        // info.simulationOwners stays empty so the parsers do not drop the shape on
        // unresolved scenes; the collider's owners are read onto the descriptor's
        // sourceSimulationOwners below, which is where the native walker puts them
        // and where the consumer resolves them.

        Matrix3d gprimRot;
        carb::Float3 gprimScale{ 1.0f, 1.0f, 1.0f };
        src.getLocalToWorldRotationAndScale(gprimKey, gprimRot, gprimScale);

        // Shape local pose. Mirror UsdGeomXformCache::ComputeRelativeTransform by
        // accumulating local transforms from the gprim up to (but excluding) its
        // body, stopping at a reset-xform-stack. The shared helper decomposes
        // that matrix and bakes the body scale; transform sourcing remains
        // ovstage-specific. A bodyless collider uses the gprim's world pose.
        if (info.rigidBody.valid())
        {
            const uint64_t bodyCanonical = src.canonicalPath(info.rigidBody);
            if (src.canonicalPath(gprimKey) != bodyCanonical)
            {
                ::physx::PxMat44d relativeTransform(::physx::PxIdentity);
                ObjectKey current = gprimKey;
                for (int guard = 0; current.valid() && guard < 64; ++guard)
                {
                    if (src.canonicalPath(current) == bodyCanonical)
                        break;

                    Matrix4d localTransform;
                    bool resetsXformStack = false;
                    src.getLocalTransform(current, ReadTime::defaultTime(), localTransform, resetsXformStack);
                    // Gf order was `relativeTransform *= local`; operands swap in
                    // PhysX order (see common/foundation/MatrixTools.h).
                    relativeTransform = toPxMat44d(localTransform) * relativeTransform;
                    if (resetsXformStack)
                        break;
                    current = src.getParent(current);
                }

                Matrix4d bodyWorldTransform;
                src.getLocalToWorldTransform(info.rigidBody, bodyWorldTransform);

                omni::physics::decomposeCollisionShapeLocalTransform(relativeTransform,
                                                                    toPxMat44d(bodyWorldTransform), info.localPos,
                                                                    info.localRot, info.localScale);
            }
        }
        else
        {
            Matrix4d wm;
            src.getLocalToWorldTransform(gprimKey, wm);
            info.localPos = { static_cast<float>(wm.data[12]), static_cast<float>(wm.data[13]),
                              static_cast<float>(wm.data[14]) };
            info.localRot = quatFromRotation(gprimRot);
            info.localScale = gprimScale;
        }

        DescPtr<PhysxShapeDesc> shape;
        bool commonFilled = true;
        // Precedence mirrors NativeWalker::emitShape: a Plane gprim is decided
        // before the custom-token check (which matters because "Plane" is itself a
        // pre-registered shape token, a legacy alias for a non-UsdGeomPlane prim
        // of that type), and the custom branch then outranks every analytic and
        // mesh branch below -- a custom-geometry API on a Sphere is a custom
        // shape, not a sphere.
        //
        // The two checks deliberately interrogate different keys, matching which
        // prim NativeWalker::emitShape tests for each: for a collider applied
        // directly to a gprim, colliderKey/gprimKey/geometryKey all collapse to
        // that one prim, same as native's single `shapePrim`. For a collider on a
        // non-gprim ancestor (the Xform-descent branch), native decides
        // `customShape` exactly once against the collider prim before
        // descending -- it is never re-tested per descendant child -- while the
        // Plane check runs per child against that child's own prim. So
        // `findCustomShapeToken` below is meant to see `colliderKey` (the
        // collider, fixed across children) and `planeGprim` is meant to see
        // `geometryKey` (the per-child geometry). Keep it this way rather than
        // unifying them onto one key; that would change which prim decides
        // custom-shape-ness for the Xform-descent case and diverge from native.
        const bool planeGprim = primTypeIs(geometryKey, PrimType::ePlane);
        const size_t customShapeToken = planeGprim ? std::string::npos : findCustomShapeToken(colliderKey);
        if (planeGprim)
        {
            shape = descPtrCast<PhysxShapeDesc>(parsePlaneShape(ctx, colliderKey, info, readAxis(geometryKey)));
        }
        else if (customShapeToken != std::string::npos)
        {
            shape = descPtrCast<PhysxShapeDesc>(parseCustomShape(
                ctx, colliderKey, info, customGeometryTokenHash(customShapeTokens[customShapeToken])));
        }
        else if (primTypeIs(geometryKey, PrimType::eCube))
        {
            double sizeAttr = 1.0;
            src.getAttribute(geometryKey, tokSize, sizeAttr);
            const float half = static_cast<float>(std::abs(sizeAttr)) * 0.5f;
            shape = descPtrCast<PhysxShapeDesc>(parseBoxShape(
                ctx, colliderKey, info,
                { half * std::fabs(gprimScale.x), half * std::fabs(gprimScale.y), half * std::fabs(gprimScale.z) }));
        }
        else if (primTypeIs(geometryKey, PrimType::eSphere))
        {
            double radius = 1.0;
            src.getAttribute(geometryKey, tokRadius, radius);
            shape = descPtrCast<PhysxShapeDesc>(
                parseSphereShape(ctx, colliderKey, info,
                                 static_cast<float>(std::abs(radius)) *
                                     maxAbs3(gprimScale.x, gprimScale.y, gprimScale.z)));
        }
        else if (primTypeIs(geometryKey, PrimType::eCapsule))
        {
            double radius = 0.5, height = 1.0;
            src.getAttribute(geometryKey, tokRadius, radius);
            src.getAttribute(geometryKey, tokHeight, height);
            float scaledRadius = static_cast<float>(radius);
            float scaledHalfHeight = static_cast<float>(height) * 0.5f;
            const Axis axis = readAxis(geometryKey);
            scaleRoundShape(axis, gprimScale, scaledRadius, scaledHalfHeight);
            shape = descPtrCast<PhysxShapeDesc>(parseCapsuleShape(
                ctx, colliderKey, info, std::fabs(scaledRadius), std::fabs(scaledHalfHeight), axis));
        }
        else if (primTypeIs(geometryKey, PrimType::eCylinder))
        {
            double radius = 1.0, height = 2.0;
            src.getAttribute(geometryKey, tokRadius, radius);
            src.getAttribute(geometryKey, tokHeight, height);
            float scaledRadius = static_cast<float>(radius);
            float scaledHalfHeight = static_cast<float>(height) * 0.5f;
            const Axis axis = readAxis(geometryKey);
            scaleRoundShape(axis, gprimScale, scaledRadius, scaledHalfHeight);
            shape = descPtrCast<PhysxShapeDesc>(parseCylinderShape(
                ctx, colliderKey, info, std::fabs(scaledRadius), std::fabs(scaledHalfHeight), axis));
        }
        else if (primTypeIs(geometryKey, PrimType::eCone))
        {
            double radius = 1.0, height = 2.0;
            src.getAttribute(geometryKey, tokRadius, radius);
            src.getAttribute(geometryKey, tokHeight, height);
            float scaledRadius = static_cast<float>(radius);
            float scaledHalfHeight = static_cast<float>(height) * 0.5f;
            const Axis axis = readAxis(geometryKey);
            scaleRoundShape(axis, gprimScale, scaledRadius, scaledHalfHeight);
            shape = descPtrCast<PhysxShapeDesc>(parseConeShape(
                ctx, colliderKey, info, std::fabs(scaledRadius), std::fabs(scaledHalfHeight), axis));
        }
        else if (primTypeIs(geometryKey, PrimType::eMesh))
        {
            // Mesh collider: build the cooking-input descriptor (triangle / convex /
            // decomposition / sphere-fill / SDF per UsdPhysicsMeshCollisionAPI). The
            // mesh scale is the gprim's world scale; the actual geometry is read later
            // by the cooking service via IPhysicsSource::getMeshAttributes(gprimKey).
            Matrix3d meshRot;
            carb::Float3 meshScale{ 1.0f, 1.0f, 1.0f };
            src.getLocalToWorldRotationAndScale(gprimKey, meshRot, meshScale);
            const carb::Float3 signScale = scaleToSignScale(meshScale);
            bool doubleSided = false;
            src.getAttribute(geometryKey, tokDoubleSided, doubleSided);
            shape = buildMeshShapeDesc(out, ctx, src, colliderKey, gprimKey, info.rigidBody, meshScale, signScale,
                                       doubleSided);
            commonFilled = false;
        }
        else
        {
            return false; // convex / unhandled geometry — later slice
        }

        if (!shape)
            return false;
        if (!commonFilled && !fillCommonShape(ctx, colliderKey, info, *shape))
            return false;
        shape->primKey = colliderKey;
        shape->sourceGprim = gprimKey;

        // Per-face physics materials: a collider mesh expresses them as GeomSubset
        // children that each bind their own material, and that is the only way to
        // author more than one physics material on a mesh. They occupy
        // sourceMaterials [0..N) with the collider's own binding as the trailing
        // default slot, which is the order the consumer hands to
        // PxShape::setMaterials. Mirrors the native walker's mesh-subset branch.
        // Resolved on the geometry key so prototype-authored subsets are found for
        // an instance proxy too.
        if (primTypeIs(geometryKey, PrimType::eMesh))
        {
            src.forEachChild(geometryKey, [&](ObjectKey child)
            {
                if (!src.isA(child, tokGeomSubset))
                    return;
                TokenId elementType{};
                if (!src.getAttribute(child, tokElementType, elementType) || elementType != tokFace)
                    return;
                // A subset that binds no material of its own resolves the mesh's, so
                // gate on the material actually being a physics material: a subset
                // bound only to a render material must not take a slot.
                const ObjectKey subsetMaterial = src.getMaterialBinding(child);
                if (subsetMaterial.valid() && src.hasSchema(subsetMaterial, tokPhysicsMaterialAPI))
                    shape->sourceMaterials.push_back(subsetMaterial);
            });
        }

        // Read direct material relationships or the resolved instance-material
        // column through the source. Keep the lookup on the logical gprim so
        // per-instance overrides survive. Mirrors the native walker, including its
        // unconditional trailing slot: with per-face subsets present the collider's
        // own binding is the default-fallback entry at index N, and the consumer
        // indexes it positionally, so the slot has to exist even when nothing is
        // bound there.
        const ObjectKey material = src.getMaterialBinding(gprimKey);
        if (material.valid() || !shape->sourceMaterials.empty())
            shape->sourceMaterials.push_back(material);

        // Per-shape collision filtering (PhysxFilteredPairsAPI on the collider).
        shape->sourceFilteredCollisions = parseFilteredPairs(ctx, colliderKey);

        // Collider-level simulationOwner (physics:simulationOwner on CollisionAPI),
        // which is what decides ownership for a bodyless static collider. Mirrors
        // the native walker: the owners ride on the descriptor and the consumer
        // resolves them, rather than going through ShapeInfo — whose
        // unresolved-scene path drops the shape outright.
        src.getRelationshipTargets(colliderKey, tokSimulationOwner, shape->sourceSimulationOwners);

        if (info.rigidBody.valid())
        {
            auto it = bodyByKey.find(info.rigidBody);
            if (it != bodyByKey.end())
                it->second->sourceCollisions.insert(colliderKey);
        }
        out.shapes.push_back(std::move(shape));
        return true;
    };

    for (const ObjectKey key : shapeBucket.keys)
    {
        // A collider applied directly to an analytic gprim emits one shape. A
        // collider applied to a non-geometry prim (e.g. an Xform) parents its
        // geometry on descendant gprims — emit a shape per analytic child, with the
        // static body kept at the collider's path (mirrors the native walker's
        // Xform-collider descent). Child gprims carry no physics schema, so they are
        // only present when the rendering domain was populated too.
        if (emitShape(key, key))
            continue;

        bool emitted = false;
        src.forEachChild(key, [&](ObjectKey child)
        {
            if (emitShape(key, child))
                emitted = true;
        });
        if (!emitted)
            ++out.skippedShapes; // unsupported collider or no supported child geometry
    }
    src.clearBucket();

    // --- Joints (typed PhysicsJoint prims) ---
    // body0/body1 are resolved relationships (read back from ovpopulation).
    // Authored local frames are normalized, transformed from each relationship
    // target into its resolved body frame, and have the body's scale baked into
    // their translations. Joint drives and D6 per-axis data are populated below.
    struct JointType
    {
        const char* typeName;
        ObjectType type;
    };
    static const JointType kJointTypes[] = {
        { "PhysicsFixedJoint", eJointFixed },
        { "PhysicsRevoluteJoint", eJointRevolute },
        { "PhysicsPrismaticJoint", eJointPrismatic },
        { "PhysicsSphericalJoint", eJointSpherical },
        { "PhysicsDistanceJoint", eJointDistance },
        { "PhysicsJoint", eJointD6 },
        { "PhysxPhysicsGearJoint", eJointGear },
        { "PhysxPhysicsRackAndPinionJoint", eJointRackAndPinion },
    };

    // Consumer-registered custom joint prim types (REQ-PARSE-CORE-005). Mirrors
    // NativeWalker::jointPrimTypeToParse exactly: the typed schema variants win
    // on their schema bit, gear / rack win on their name, and the registry's only
    // effect there is to turn the eJointD6 fallback into eJointCustom. What the
    // native walker gets for free and this one does not is *reaching* a type the
    // list above does not name -- it visits every prim, while ovstage queries by
    // exact prim type -- so a registered type absent from the list needs its own
    // query (ADR-0002 open question 1).
    const std::vector<std::string> customJointTokens = parse::customTokens(parse::CustomTokenKind::eJoint);
    std::vector<JointType> jointTypes(std::begin(kJointTypes), std::end(kJointTypes));
    for (JointType& jt : jointTypes)
    {
        if (jt.type == eJointD6 &&
            parse::isCustomToken(parse::CustomTokenKind::eJoint, jt.typeName))
        {
            jt.type = eJointCustom;
        }
    }
    for (const std::string& jointToken : customJointTokens)
    {
        const bool alreadyQueried =
            std::any_of(std::begin(kJointTypes), std::end(kJointTypes),
                        [&](const JointType& jt) { return jointToken == jt.typeName; });
        if (!alreadyQueried)
            jointTypes.push_back(JointType{ jointToken.c_str(), eJointCustom });
    }

    const TokenId tokJointEnabled = src.internToken("physics:jointEnabled");
    const TokenId tokBreakForce = src.internToken("physics:breakForce");
    const TokenId tokBreakTorque = src.internToken("physics:breakTorque");
    const TokenId tokExcludeArt = src.internToken("physics:excludeFromArticulation");
    const TokenId tokBody0 = src.internToken("physics:body0");
    const TokenId tokBody1 = src.internToken("physics:body1");
    const TokenId tokLocalPos0 = src.internToken("physics:localPos0");
    const TokenId tokLocalRot0 = src.internToken("physics:localRot0");
    const TokenId tokLocalPos1 = src.internToken("physics:localPos1");
    const TokenId tokLocalRot1 = src.internToken("physics:localRot1");
    const TokenId tokJointAxis = src.internToken("physics:axis");
    const TokenId tokLowerLimit = src.internToken("physics:lowerLimit");
    const TokenId tokUpperLimit = src.internToken("physics:upperLimit");
    const TokenId tokCone0 = src.internToken("physics:coneAngle0Limit");
    const TokenId tokCone1 = src.internToken("physics:coneAngle1Limit");
    const TokenId tokMinDist = src.internToken("physics:minDistance");
    const TokenId tokMaxDist = src.internToken("physics:maxDistance");
    const TokenId tokDriveAcceleration = src.internToken("acceleration");

    auto firstTarget = [&](ObjectKey jointKey, TokenId rel) -> ObjectKey
    {
        std::vector<ObjectKey> t;
        src.getRelationshipTargets(jointKey, rel, t);
        return t.empty() ? ObjectKey{} : t.front();
    };
    // Resolve a relationship target to the nearest rigid-body ancestor. If none
    // exists, preserve the native walker's legacy fallback to the outermost
    // CollisionAPI ancestor. At each level the scan's body map is the fast path;
    // RigidBodyAPI membership is checked only after that level misses so a body
    // created in an earlier incremental ordinal still outranks a farther mapped
    // body. CollisionAPI membership is deferred to a second pass outside the
    // common rigid-body path.
    auto resolveBodyFromTarget = [&](ObjectKey relTarget) -> ObjectKey
    {
        ObjectKey k = relTarget;
        for (int guard = 0; k.valid() && guard < 64; ++guard)
        {
            const std::unordered_map<uint64_t, ObjectKey>::const_iterator it =
                bodyByCanonical.find(src.canonicalPath(k));
            if (it != bodyByCanonical.end())
                return it->second;
            if (src.hasSchema(k, tokRigidBodyAPI))
                return src.canonicalKey(k);
            k = src.getParent(k);
        }

        ObjectKey collisionBody;
        k = relTarget;
        for (int guard = 0; k.valid() && guard < 64; ++guard)
        {
            if (src.hasSchema(k, tokCollisionAPI))
                collisionBody = src.canonicalKey(k);
            k = src.getParent(k);
        }
        return collisionBody;
    };
    auto readJointAxis = [&](ObjectKey key) -> Axis
    {
        TokenId at{};
        if (src.getAttribute(key, tokJointAxis, at) && at.valid())
        {
            if (at == tokAxisY)
                return eY;
            if (at == tokAxisZ)
                return eZ;
        }
        return eX; // USD joint default axis is X
    };
    auto transformJointFrame = [&](ObjectKey relationshipTarget,
                                   ObjectKey body,
                                   carb::Float3& localPosition,
                                   carb::Float4& localOrientation) {
        ::physx::PxQuat orientation(localOrientation.x, localOrientation.y, localOrientation.z, localOrientation.w);
        orientation.normalize();
        localOrientation = { orientation.x, orientation.y, orientation.z, orientation.w };
        if (!relationshipTarget.valid())
        {
            return;
        }

        Matrix4d relationshipWorld;
        src.getLocalToWorldTransform(relationshipTarget, relationshipWorld);

        const bool relationshipTargetsBody =
            body.valid() && src.canonicalPath(body) == src.canonicalPath(relationshipTarget);
        Matrix4d bodyWorld = relationshipWorld;
        if (!body.valid())
            bodyWorld = Matrix4d{}; // identity (Math.h): no body, the pose stays world-space
        else if (!relationshipTargetsBody)
            src.getLocalToWorldTransform(body, bodyWorld);

        omni::physics::transformJointFrameToBody(toPxMat44d(relationshipWorld), toPxMat44d(bodyWorld),
                                                 relationshipTargetsBody, localPosition, localOrientation);
    };

    struct ScannedJointEntry
    {
        PhysxJointDesc* desc = nullptr;
        bool jointEnabled = true;
        bool excludeFromArticulation = false;
        std::vector<std::pair<JointAxis, JointLimitInfo>> jointLimits;
    };
    using ScannedJointMap = std::unordered_map<ObjectKey, ScannedJointEntry, ObjectKey::Hash>;
    ScannedJointMap jointMap;
    std::vector<ObjectKey> jointOrder;

    // Stage-wide joint sub-schema presence: one count-only membership query per family,
    // OR'd over every instance parseJoint could probe. Any applied instance anywhere
    // trips the flag, so skipping the per-joint probes is behaviour-preserving. Probed once,
    // on the first non-empty joint bucket: the flags are read only inside the joint loop, so a
    // stage without joints keeps the (conservative) defaults and pays nothing.
    bool jointSchemaPresenceProbed = false;
    auto probeJointSchemaPresence = [&]()
    {
        if (jointSchemaPresenceProbed)
            return;
        jointSchemaPresenceProbed = true;
        auto countOf = [](auto& arr) { return sizeof(arr) / sizeof(arr[0]); };
        static const char* const kLimit[] = {
            "PhysxLimitAPI:angular", "PhysxLimitAPI:linear", "PhysxLimitAPI:cone", "PhysxLimitAPI:distance",
            "PhysxLimitAPI:transX", "PhysxLimitAPI:transY", "PhysxLimitAPI:transZ",
            "PhysxLimitAPI:rotX", "PhysxLimitAPI:rotY", "PhysxLimitAPI:rotZ" };
        static const char* const kAxis[] = {
            "PhysxJointAxisAPI:angular", "PhysxJointAxisAPI:linear",
            "PhysxJointAxisAPI:rotX", "PhysxJointAxisAPI:rotY", "PhysxJointAxisAPI:rotZ" };
        static const char* const kEnv[] = {
            "PhysxDrivePerformanceEnvelopeAPI:angular", "PhysxDrivePerformanceEnvelopeAPI:linear",
            "PhysxDrivePerformanceEnvelopeAPI:rotX", "PhysxDrivePerformanceEnvelopeAPI:rotY",
            "PhysxDrivePerformanceEnvelopeAPI:rotZ" };
        static const char* const kState[] = {
            "PhysicsJointStateAPI:angular", "PhysicsJointStateAPI:linear",
            "PhysicsJointStateAPI:transX", "PhysicsJointStateAPI:transY", "PhysicsJointStateAPI:transZ",
            "PhysicsJointStateAPI:rotX", "PhysicsJointStateAPI:rotY", "PhysicsJointStateAPI:rotZ" };
        static const char* const kDrive[] = {
            "PhysicsDriveAPI:angular", "PhysicsDriveAPI:linear",
            "PhysicsDriveAPI:transX", "PhysicsDriveAPI:transY", "PhysicsDriveAPI:transZ",
            "PhysicsDriveAPI:rotX", "PhysicsDriveAPI:rotY", "PhysicsDriveAPI:rotZ", "PhysicsDriveAPI:distance" };
        static const char* const kJointApi[] = { "PhysxJointAPI" };
        static const char* const kDistApi[] = { "PhysxPhysicsDistanceJointAPI" };
        // An undeterminable answer keeps the fail-open default (probe per joint).
        JointSchemaPresence& present = ctx.jointSchemaPresence();
        auto known = [&](const char* const* names, size_t count, bool& flag)
        {
            if (const std::optional<bool> v = src.stageHasAnySchema(names, count))
                flag = *v;
        };
        known(kLimit, countOf(kLimit), present.physxLimit);
        known(kAxis, countOf(kAxis), present.physxJointAxis);
        known(kEnv, countOf(kEnv), present.drivePerfEnvelope);
        known(kState, countOf(kState), present.jointState);
        known(kDrive, countOf(kDrive), present.physicsDrive);
        known(kJointApi, countOf(kJointApi), present.physxJointApi);
        known(kDistApi, countOf(kDistApi), present.physxDistanceJoint);
    };

    for (const JointType& jt : jointTypes)
    {
        Bucket jointBucket = enumeratePrimTypeFiltered(jt.typeName, "physics:localPos0", readOrdinal);
        if (!jointBucket.keys.empty())
            probeJointSchemaPresence();
        const bool stageHasPhysicsDrive = ctx.jointSchemaPresence().physicsDrive;
        const std::vector<std::string> jointDiscoveredAttrs = jointBucket.attrs;
        const size_t jointAttrCount = jointBucket.attrs.size();
        appendJointAttrs(jointBucket, jt.type);
        // Prefetch only gated-family columns some prim authors (trimUnauthoredGatedJointAttrs).
        trimUnauthoredGatedJointAttrs(jointBucket, jointAttrCount, jointDiscoveredAttrs);
        // One columnar read of discovered + appended attrs.
        src.prefetchBucket(jointBucket.keys, jointBucket.attrs);

        // Drive tokens are constant per joint type: intern once here, not per joint.
        const char* const driveInst = (jt.type == eJointRevolute)  ? "angular"
                                       : (jt.type == eJointPrismatic) ? "linear"
                                                                      : nullptr;
        TokenId driveApiToken, driveStiffnessTok, driveDampingTok, driveTargetPosTok,
            driveTargetVelTok, driveMaxForceTok, driveTypeTok;
        if (driveInst)
        {
            driveApiToken = src.internToken(std::string("PhysicsDriveAPI:") + driveInst);
            const std::string pfx = std::string("drive:") + driveInst + ":physics:";
            driveStiffnessTok = src.internToken(pfx + "stiffness");
            driveDampingTok = src.internToken(pfx + "damping");
            driveTargetPosTok = src.internToken(pfx + "targetPosition");
            driveTargetVelTok = src.internToken(pfx + "targetVelocity");
            driveMaxForceTok = src.internToken(pfx + "maxForce");
            driveTypeTok = src.internToken(pfx + "type");
        }

        for (const ObjectKey key : jointBucket.keys)
        {
            JointInfo info;
            info.type = jt.type;

            bool flag = true;
            if (src.getAttribute(key, tokJointEnabled, flag))
                info.jointEnabled = flag;
            flag = false;
            if (src.getAttribute(key, tokCollisionEnabled, flag))
                info.collisionEnabled = flag;
            // Default to "no break" (FLT_MAX), overwritten only if authored.
            info.breakForce = FLT_MAX;
            info.breakTorque = FLT_MAX;
            src.getAttribute(key, tokBreakForce, info.breakForce);
            src.getAttribute(key, tokBreakTorque, info.breakTorque);
            flag = false;
            if (src.getAttribute(key, tokExcludeArt, flag))
                info.excludeFromArticulation = flag;

            // Resolved body relationships → nearest enabling rigid body.
            const ObjectKey rel0 = firstTarget(key, tokBody0);
            const ObjectKey rel1 = firstTarget(key, tokBody1);
            const ObjectKey body0 = rel0.valid() ? resolveBodyFromTarget(rel0) : ObjectKey{};
            const ObjectKey body1 = rel1.valid() ? resolveBodyFromTarget(rel1) : ObjectKey{};
            info.rel0 = rel0;
            info.rel1 = rel1;
            info.body0 = body0;
            info.body1 = body1;

            // Authored local frames are relative to the relationship targets;
            // convert them to the resolved body frames after reading.
            {             src.getAttribute(key, tokLocalPos0, info.localPose0Position);
            src.getAttribute(key, tokLocalRot0, info.localPose0Orientation);
            src.getAttribute(key, tokLocalPos1, info.localPose1Position);
            src.getAttribute(key, tokLocalRot1, info.localPose1Orientation);
            transformJointFrame(rel0, body0, info.localPose0Position, info.localPose0Orientation);
            transformJointFrame(rel1, body1, info.localPose1Position, info.localPose1Orientation);
            }

            // Per-type axis and single limit. D6 per-axis data and drives are
            // populated separately below.
            {             if (jt.type == eJointRevolute || jt.type == eJointPrismatic || jt.type == eJointSpherical)
                info.axis = readJointAxis(key);
            // A limit is active only when a bound is finite and inside the sentinel
            // range; USD's unlimited default (-inf/+inf, or the +-0.5e38 sentinel)
            // leaves it disabled. Mirrors NativeWalker — without the finite check an
            // unlimited revolute/prismatic joint feeds inf bounds to the articulation
            // joint's setLimitParams and PhysX rejects them.
            constexpr float kJointSentinelLimit = 0.5e38f;
            if (jt.type == eJointRevolute || jt.type == eJointPrismatic)
            {
                src.getAttribute(key, tokLowerLimit, info.limit.lower);
                src.getAttribute(key, tokUpperLimit, info.limit.upper);
                info.limit.enabled =
                    (std::isfinite(info.limit.lower) && info.limit.lower > -kJointSentinelLimit) ||
                    (std::isfinite(info.limit.upper) && info.limit.upper < kJointSentinelLimit);
            }
            else if (jt.type == eJointSpherical)
            {
                src.getAttribute(key, tokCone0, info.limit.lower);
                src.getAttribute(key, tokCone1, info.limit.upper);
                info.limit.enabled = std::isfinite(info.limit.lower) && std::isfinite(info.limit.upper) &&
                                     info.limit.lower >= 0.0f && info.limit.upper >= 0.0f;
            }
            else if (jt.type == eJointDistance)
            {
                float mn = -1.0f, mx = -1.0f;
                info.minEnabled = src.getAttribute(key, tokMinDist, mn) && mn >= 0.0f;
                info.maxEnabled = src.getAttribute(key, tokMaxDist, mx) && mx >= 0.0f;
                info.limit.lower = mn;
                info.limit.upper = mx;
            }
            else if (jt.type == eJointD6)
            {
                // D6 carries per-axis limits/drives as multi-apply UsdPhysicsLimitAPI:
                // <axis> / UsdPhysicsDriveAPI:<axis> instances (axis = transX/Y/Z,
                // rotX/Y/Z, distance). These axis names are fixed by the D6 schema,
                // so check the exact schema tokens directly.
                struct D6AxisSchema
                {
                    const char* instance;
                    JointAxis axis;
                };
                static const D6AxisSchema kD6AxisSchemas[] = {
                    { "transX", eTransX }, { "transY", eTransY }, { "transZ", eTransZ },
                    { "rotX", eRotX },     { "rotY", eRotY },     { "rotZ", eRotZ },
                    { "distance", eDistance },
                };
                for (const D6AxisSchema& axisSchema : kD6AxisSchemas)
                {
                    const std::string limitApi = std::string("PhysicsLimitAPI:") + axisSchema.instance;
                    if (!src.hasSchema(key, src.internToken(limitApi)))
                        continue;
                    const std::string pfx = std::string("limit:") + axisSchema.instance + ":physics:";
                    JointLimitInfo lim;
                    src.getAttribute(key, src.internToken(pfx + "low"), lim.lower);
                    src.getAttribute(key, src.internToken(pfx + "high"), lim.upper);
                    // A limit is active only for finite bounds inside the sentinel
                    // range; USD's unlimited (-inf/+inf or beyond +-0.5e38) leaves the
                    // axis FREE (enabled=false), matching the revolute/prismatic rule.
                    // Feeding inf to PxD6Joint's pyramid/limit setters is rejected.
                    lim.enabled =
                        (std::isfinite(lim.lower) && lim.lower > -kJointSentinelLimit) ||
                        (std::isfinite(lim.upper) && lim.upper < kJointSentinelLimit);
                    info.jointLimits.push_back({ axisSchema.axis, lim });
                }
                for (const D6AxisSchema& axisSchema : kD6AxisSchemas)
                {
                    const std::string driveApi = std::string("PhysicsDriveAPI:") + axisSchema.instance;
                    if (!src.hasSchema(key, src.internToken(driveApi)))
                        continue;
                    const std::string pfx = std::string("drive:") + axisSchema.instance + ":physics:";
                    JointDriveInfo drv;
                    drv.enabled = true;
                    drv.forceLimit = FLT_MAX;
                    src.getAttribute(key, src.internToken(pfx + "stiffness"), drv.stiffness);
                    src.getAttribute(key, src.internToken(pfx + "damping"), drv.damping);
                    src.getAttribute(key, src.internToken(pfx + "targetPosition"), drv.targetPosition);
                    src.getAttribute(key, src.internToken(pfx + "targetVelocity"), drv.targetVelocity);
                    src.getAttribute(key, src.internToken(pfx + "maxForce"), drv.forceLimit);
                    TokenId driveType{};
                    if (src.getAttribute(key, src.internToken(pfx + "type"), driveType) && driveType.valid())
                        drv.acceleration = (driveType == tokDriveAcceleration);
                    info.jointDrives.push_back({ axisSchema.axis, drv });
                }
            }

            // Revolute/prismatic single-axis drive (UsdPhysicsDriveAPI:<instance>).
            // The instance is fixed by joint type ("angular" for revolute,
            // "linear" for prismatic), so it is read directly without multi-apply
            // enumeration. The drive is active iff that DriveAPI instance is applied.
            if (driveInst && stageHasPhysicsDrive && src.hasSchema(key, driveApiToken))
            {
                info.drive.enabled = true;
                info.drive.forceLimit = FLT_MAX;
                src.getAttribute(key, driveStiffnessTok, info.drive.stiffness);
                src.getAttribute(key, driveDampingTok, info.drive.damping);
                src.getAttribute(key, driveTargetPosTok, info.drive.targetPosition);
                src.getAttribute(key, driveTargetVelTok, info.drive.targetVelocity);
                src.getAttribute(key, driveMaxForceTok, info.drive.forceLimit);
                TokenId driveType{};
                if (src.getAttribute(key, driveTypeTok, driveType) && driveType.valid())
                    info.drive.acceleration = (driveType == tokDriveAcceleration);
            }
            }

            DescPtr<PhysxJointDesc> joint = parseJoint(ctx, key, info);
            if (!joint)
                continue;

            auto bodyFromReferencedJoint = [&](ObjectKey referencedJoint) -> ObjectKey
            {
                ScannedJointMap::const_iterator refIt = jointMap.find(src.canonicalKey(referencedJoint));
                if (refIt == jointMap.end() || !refIt->second.desc)
                    return {};

                const PhysxJointDesc* refDesc = refIt->second.desc;
                return refDesc->body1.valid() ? refDesc->body1 : refDesc->body0;
            };

            // ovpopulation currently surfaces gear/rack custom-joint prims and their
            // cross-joint refs, but can omit their inherited PhysicsJoint body rels.
            // Derive the missing bodies from the referenced child joints so the
            // runtime second-pass gear/rack creation has valid actors.
            if (jt.type == eJointGear)
            {
                GearPhysxJointDesc* gear = static_cast<GearPhysxJointDesc*>(joint.get());
                if (!gear->body0.valid())
                    gear->body0 = bodyFromReferencedJoint(gear->hingePrimPath0);
                if (!gear->body1.valid())
                    gear->body1 = bodyFromReferencedJoint(gear->hingePrimPath1);
            }
            else if (jt.type == eJointRackAndPinion)
            {
                RackPhysxJointDesc* rack = static_cast<RackPhysxJointDesc*>(joint.get());
                if (!rack->body0.valid())
                    rack->body0 = bodyFromReferencedJoint(rack->hingePrimKey);
                if (!rack->body1.valid())
                    rack->body1 = bodyFromReferencedJoint(rack->prismaticPrimKey);
            }

            joint->jointPrimKey = key;

            ScannedJointEntry entry;
            entry.desc = joint.get();
            entry.jointEnabled = info.jointEnabled;
            entry.excludeFromArticulation = info.excludeFromArticulation;
            entry.jointLimits = info.jointLimits;
            const ObjectKey canonicalJointKey = src.canonicalKey(key);
            jointMap[canonicalJointKey] = std::move(entry);
            jointOrder.push_back(canonicalJointKey);

            out.joints.push_back(std::move(joint));
        }
        src.clearBucket();
    }

    if (!jointMap.empty())
    {
        JointTypeLookup jointTypeOf = [&src, &jointMap](ObjectKey key) -> ObjectType
        {
            const ObjectKey canonicalKey = src.canonicalKey(key);
            ScannedJointMap::const_iterator it = jointMap.find(canonicalKey);
            if (it == jointMap.end() || !it->second.desc)
                return eUndefined;
            return it->second.desc->type;
        };

        JointLimitLookup jointLimitOf = [&src, &jointMap](ObjectKey key, JointAxis axis) -> const JointLimitInfo*
        {
            const ObjectKey canonicalKey = src.canonicalKey(key);
            ScannedJointMap::const_iterator it = jointMap.find(canonicalKey);
            if (it == jointMap.end())
                return nullptr;
            for (const std::pair<JointAxis, JointLimitInfo>& limit : it->second.jointLimits)
                if (limit.first == axis)
                    return &limit.second;
            return nullptr;
        };

        for (const ObjectKey& jointKey : jointOrder)
        {
            ScannedJointMap::const_iterator it = jointMap.find(jointKey);
            if (it == jointMap.end() || !it->second.desc)
                continue;
            const ScannedJointEntry& jointEntry = it->second;

            MimicJointParseInfo info;
            info.jointType = jointEntry.desc->type;
            info.jointEnabled = jointEntry.jointEnabled;
            info.excludeFromArticulation = jointEntry.excludeFromArticulation;

            const bool physxMimicAllowed = info.jointType == eJointRevolute ||
                                           info.jointType == eJointPrismatic ||
                                           info.jointType == eJointD6;
            const bool newtonMimicAllowed = physxMimicAllowed ||
                                            info.jointType == eJointFixed ||
                                            info.jointType == eJointSpherical ||
                                            info.jointType == eJointDistance;
            if (physxMimicAllowed)
                parseMimicJoints(ctx, jointKey, info, jointTypeOf, jointLimitOf, out.mimicJoints);
            if (newtonMimicAllowed)
                parseNewtonMimicJoints(ctx, jointKey, info, jointTypeOf, out.mimicJoints);
        }

        // parseFixedTendons probes every joint (one schema read each); skip the pass when no
        // prim applies a fixed-tendon axis schema. Membership, not authored attributes: an
        // applied instance with schema-default values is still a tendon.
        static const char* const kFixedTendonBases[] = { "PhysxTendonAxisRootAPI", "PhysxTendonAxisAPI" };
        if (src.stageHasAnySchema(kFixedTendonBases, sizeof(kFixedTendonBases) / sizeof(kFixedTendonBases[0]))
                .value_or(true))
        for (const ObjectKey& jointKey : jointOrder)
        {
            ScannedJointMap::const_iterator it = jointMap.find(jointKey);
            if (it == jointMap.end() || !it->second.desc)
                continue;
            FixedTendonParseInfo info;
            info.jointType = it->second.desc->type;
            info.body0 = it->second.desc->body0;
            info.body1 = it->second.desc->body1;
            parseFixedTendons(ctx, jointKey, info, out.fixedTendonAxes, out.fixedTendons);
        }
    }

    // Spatial tendon attachments are arbitrary multi-apply schemas on articulation
    // links / rigid bodies. Use the cached usd-schemas index to check the base
    // schema families without probing possible instance names through ovstage.
    static const char* const kSpatialTendonBases[] = {
        "PhysxTendonAttachmentRootAPI",
        "PhysxTendonAttachmentLeafAPI",
        "PhysxTendonAttachmentAPI",
    };
    constexpr size_t kSpatialTendonBaseCount =
        sizeof(kSpatialTendonBases) / sizeof(kSpatialTendonBases[0]);
    std::vector<ObjectKey> spatialTendonKeys;
    // Spatial-tendon instance names are user-arbitrary, so collectMultiApplySchemaKeys needs
    // a whole-stage schema parse; skip it unless some prim applies one of the bases.
    if (src.stageHasAnySchema(kSpatialTendonBases, kSpatialTendonBaseCount).value_or(true))
        for (const char* base : kSpatialTendonBases)
            src.collectMultiApplySchemaKeys(src.internToken(base), spatialTendonKeys);
    if (!spatialTendonKeys.empty())
    {
        for (const auto& bodyEntry : bodyByKey)
        {
            if (!bodyEntry.second)
                continue;
            SpatialTendonParseInfo info;
            info.linkWorldScale = bodyEntry.second->scale;
            parseSpatialTendons(ctx, bodyEntry.first, info, out.spatialTendonAttachments);
        }
    }

    // --- Collision groups (typed UsdPhysicsCollisionGroup prims) ---
    // filteredGroups (group↔group filtering) + the `colliders` collection
    // membership, both resolved through the source from ovpopulation-persisted
    // relationships (parseCollisionGroup → getRelationshipTargets / resolveCollection).
    Bucket cgBucket = enumeratePrimTypeFiltered("PhysicsCollisionGroup",
                                                "physics:invertFilteredGroups",
                                                readOrdinal);
    const std::vector<std::string> cgDiscoveredAttrs = cgBucket.attrs;
    const size_t cgAttrCount = cgBucket.attrs.size();
    appendCollisionGroupAttrs(cgBucket);
    src.prefetchBucket(cgBucket.keys, cgDiscoveredAttrs);
    src.prefetchBucket(cgBucket.keys, appendedAttrs(cgBucket, cgAttrCount));
    for (const ObjectKey key : cgBucket.keys)
    {
        CollisionGroupInfo info = parseCollisionGroup(ctx, key);
        DescPtr<CollisionGroupDesc> desc = allocateDesc<CollisionGroupDesc>(allocator);
        if (!desc)
            continue;
        desc->primKey = key;
        desc->sourceFilteredGroups = std::move(info.filteredGroups);
        desc->sourceMembers = std::move(info.members);
        out.collisionGroups.push_back(std::move(desc));
    }
    src.clearBucket();

    // --- Articulations (HAS_APPLIED_SCHEMA PhysicsArticulationRootAPI) ---
    // Collect the roots (config via parseArticulation) and hand them — with the
    // already-scanned bodies + joints — to the SHARED root-election / graph
    // algorithm (parse::buildArticulations); the native walker runs the same code.
    std::vector<ArticulationRootInput> artRoots;
    std::unordered_set<ObjectKey, ObjectKey::Hash> articulationRootKeys;
    Bucket artBucket = enumerateSchemaFiltered("PhysicsArticulationRootAPI",
                                               conv::kLocalTransform,
                                               readOrdinal);
    const std::vector<std::string> artDiscoveredAttrs = artBucket.attrs;
    const size_t artAttrCount = artBucket.attrs.size();
    appendArticulationAttrs(artBucket);
    prefetchTransformAncestors(src, artBucket.keys);
    prefetchTransformsForKeys(src, artBucket.keys);
    src.prefetchBucket(artBucket.keys, artDiscoveredAttrs);
    src.prefetchBucket(artBucket.keys, appendedAttrs(artBucket, artAttrCount));
    auto addArticulationRoot = [&](ObjectKey key)
    {
        if (!key.valid() || !keyPassesFilter(key) || isBelowPointInstancer(key))
            return;
        const ObjectKey canonical = src.canonicalKey(key);
        const ObjectKey dedupeKey = canonical.valid() ? canonical : key;
        if (!articulationRootKeys.insert(dedupeKey).second)
            return;

        ArticulationRootInput in;
        in.key = key;
        setToDefault(in.fields, units);
        parseArticulation(ctx, key, in.fields);
        in.sourceFilteredCollisions = parseFilteredPairs(ctx, key);
        artRoots.push_back(std::move(in));
    };
    for (const ObjectKey key : artBucket.keys)
        addArticulationRoot(key);

    src.clearBucket();

    buildArticulations(src, allocator, artRoots, out.bodies, out.joints, out.articulations);

    auto scanBucket = [&](const Bucket& bucket, auto emit)
    {
        src.prefetchBucket(bucket.keys, bucket.attrs);
        for (const ObjectKey key : bucket.keys)
            emit(key);
        src.clearBucket();
    };
    auto scanBucketWithTransform = [&](const Bucket& bucket, auto emit)
    {
        Bucket withTransforms = bucket;
        appendTransformAttrs(withTransforms);
        prefetchTransformAncestors(src, withTransforms.keys);
        prefetchTransformsForKeys(src, withTransforms.keys);
        src.prefetchBucket(bucket.keys, bucket.attrs);
        for (const ObjectKey key : withTransforms.keys)
            emit(key);
        src.clearBucket();
    };

    // --- Particles ---
    Bucket particleSystemBucket = enumeratePrimTypeFiltered("PhysxParticleSystem",
                                                           "particleSystemEnabled",
                                                           readOrdinal);
    appendBucketAttr(particleSystemBucket, "physics:simulationOwner");
    appendBucketAttr(particleSystemBucket, "physics:filteredPairs");
    scanBucket(particleSystemBucket, [&](ObjectKey key) { emitParticleSystem(out, ctx, key); });

    Bucket particleSetBucket = enumerateSchemaFiltered("PhysxParticleSetAPI",
                                                     "physxParticle:particleSystem", readOrdinal);
    appendBucketAttr(particleSetBucket, "physxParticle:particleSystem");
    scanBucket(particleSetBucket, [&](ObjectKey key) { emitParticleSet(out, ctx, key); });

    Bucket particleSamplerBucket = enumerateSchemaFiltered("PhysxParticleSamplingAPI",
                                                          "physxParticleSampling:samplingDistance", readOrdinal);
    appendBucketAttr(particleSamplerBucket, "physxParticleSampling:particles");
    scanBucket(particleSamplerBucket, [&](ObjectKey key) { emitParticleSampler(out, ctx, key); });

    // --- Deformable attachments and element collision filters ---
    static const char* const attachmentTypeNames[] = {
        "OmniPhysicsVtxVtxAttachment", "OmniPhysicsVtxTriAttachment",
        "OmniPhysicsVtxTetAttachment", "OmniPhysicsVtxCrvAttachment",
        "OmniPhysicsVtxXformAttachment", "OmniPhysicsTetXformAttachment",
        "OmniPhysicsTriTriAttachment",
    };
    for (const char* typeName : attachmentTypeNames)
    {
        const Bucket attachmentBucket = enumeratePrimTypeFiltered(typeName,
                                                                  "omniphysics:attachmentEnabled",
                                                                  readOrdinal);
        const ObjectType subtype = attachmentSubtypeForPrimType(typeName);
        scanBucket(attachmentBucket, [&](ObjectKey key) { emitAttachment(out, ctx, key, subtype); });
    }

    const Bucket filterBucket = enumeratePrimTypeFiltered("OmniPhysicsElementCollisionFilter",
                                                          "omniphysics:filterEnabled",
                                                          readOrdinal);
    scanBucket(filterBucket, [&](ObjectKey key) { emitElementCollisionFilter(out, ctx, key); });

    // --- Character controllers ---
    // Peer of NativeWalker's ApiFlag::eCharacterControllerAPI dispatch. Needs the
    // resolved world transform (position + scale), so it rides scanBucketWithTransform.
    Bucket cctBucket = enumerateSchemaFiltered("PhysxCharacterControllerAPI",
                                               "physxCharacterController:slopeLimit", readOrdinal);
    appendBucketAttrs(cctBucket, {
        "physxCharacterController:slopeLimit",
        "physxCharacterController:simulationOwner",
        "radius",
        "height",
    });
    scanBucketWithTransform(cctBucket, [&](ObjectKey key) { emitCct(out, src, ctx, key); });

    // --- Vehicles: scene context + tire friction tables ---
    // Descriptors in their own right (a vehicle may reference them later): always scanned.
    const Bucket vehCtxBucket = enumerateSchemaFiltered("PhysxVehicleContextAPI",
                                                "physxVehicleContext:updateMode", readOrdinal);
    scanBucket(vehCtxBucket, [&](ObjectKey key) { emitVehicleContext(out, src, ctx, key); });

    const Bucket tftBucket = enumeratePrimTypeFiltered("PhysxVehicleTireFrictionTable",
                                                       "frictionValues",
                                                       readOrdinal);
    scanBucket(tftBucket, [&](ObjectKey key) { emitTireFrictionTable(out, src, ctx, key); });

    // The ~16 component / vehicle enumerates below are each an ovstage query even when
    // empty; one count query over the whole vehicle schema family gates them. Unknown → scan.
    static const char* const kVehicleFamilySchemas[] = {
        "PhysxVehicleAPI", "PhysxVehicleWheelAPI", "PhysxVehicleTireAPI", "PhysxVehicleSuspensionAPI",
        "PhysxVehicleEngineAPI", "PhysxVehicleGearsAPI", "PhysxVehicleClutchAPI", "PhysxVehicleDriveBasicAPI",
        "PhysxVehicleDriveStandardAPI", "PhysxVehicleTankDifferentialAPI", "PhysxVehicleMultiWheelDifferentialAPI",
        "PhysxVehicleAutoGearBoxAPI", "PhysxVehicleBrakesAPI:brakes0", "PhysxVehicleBrakesAPI:brakes1",
        "PhysxVehicleSteeringAPI", "PhysxVehicleAckermannSteeringAPI",
        "PhysxVehicleNonlinearCommandResponseAPI:drive", "PhysxVehicleNonlinearCommandResponseAPI:steer",
        "PhysxVehicleNonlinearCommandResponseAPI:brakes0", "PhysxVehicleNonlinearCommandResponseAPI:brakes1",
        "PhysxVehicleWheelAttachmentAPI", "PhysxVehicleSuspensionComplianceAPI" };
    if (src.stageHasAnySchema(kVehicleFamilySchemas, sizeof(kVehicleFamilySchemas) / sizeof(kVehicleFamilySchemas[0]))
            .value_or(true))
    {
    // --- Vehicles: shareable wheel/tire/suspension components ---
    const Bucket wheelBucket = enumerateSchemaFiltered("PhysxVehicleWheelAPI",
                                               "physxVehicleWheel:radius", readOrdinal);
    scanBucket(wheelBucket, [&](ObjectKey key) { emitVehicleWheel(out, ctx, key); });

    const Bucket tireBucket = enumerateSchemaFiltered("PhysxVehicleTireAPI",
                                              "physxVehicleTire:lateralStiffnessGraph", readOrdinal);
    scanBucket(tireBucket, [&](ObjectKey key) { emitVehicleTire(out, instance, src, ctx, key, massScale); });

    const Bucket suspensionBucket = enumerateSchemaFiltered("PhysxVehicleSuspensionAPI",
                                                    "physxVehicleSuspension:springStrength", readOrdinal);
    scanBucket(suspensionBucket, [&](ObjectKey key) { emitVehicleSuspension(out, ctx, key); });

    // --- Vehicles: drivetrain components ---
    const Bucket engineBucket = enumerateSchemaFiltered("PhysxVehicleEngineAPI",
                                                "physxVehicleEngine:peakTorque", readOrdinal);
    scanBucket(engineBucket, [&](ObjectKey key) { emitVehicleEngine(out, src, ctx, key, kgmsScale); });

    const Bucket gearsBucket = enumerateSchemaFiltered("PhysxVehicleGearsAPI",
                                               "physxVehicleGears:ratios", readOrdinal);
    scanBucket(gearsBucket, [&](ObjectKey key) { emitVehicleGears(out, src, ctx, key); });

    const Bucket clutchBucket = enumerateSchemaFiltered("PhysxVehicleClutchAPI",
                                                "physxVehicleClutch:strength", readOrdinal);
    scanBucket(clutchBucket, [&](ObjectKey key) { emitVehicleClutch(out, ctx, key, kgmsScale); });

    const Bucket driveBasicBucket = enumerateSchemaFiltered("PhysxVehicleDriveBasicAPI",
                                                    "physxVehicleDriveBasic:peakTorque", readOrdinal);
    scanBucket(driveBasicBucket, [&](ObjectKey key) { emitVehicleDriveBasic(out, ctx, key, kgmsScale); });

    const Bucket driveStandardBucket = enumerateSchemaFiltered("PhysxVehicleDriveStandardAPI",
                                                       "physxVehicleDriveStandard:engine", readOrdinal);
    scanBucket(driveStandardBucket, [&](ObjectKey key) { emitVehicleDriveStandard(out, src, ctx, key); });

    const Bucket tankDiffBucket = enumerateSchemaFiltered("PhysxVehicleTankDifferentialAPI",
                                                  "physxVehicleTankDifferential:numberOfWheelsPerTrack", readOrdinal);
    scanBucket(tankDiffBucket, [&](ObjectKey key) { emitVehicleTankDifferential(out, src, ctx, key); });

    const Bucket multiDiffBucket = enumerateSchemaFiltered("PhysxVehicleMultiWheelDifferentialAPI",
                                                   "physxVehicleMultiWheelDifferential:wheels", readOrdinal);
    const TokenId tokTankDiffApi = src.internToken("PhysxVehicleTankDifferentialAPI");
    scanBucket(multiDiffBucket, [&](ObjectKey key)
    {
        if (!src.hasSchema(key, tokTankDiffApi))
            emitVehicleMultiWheelDifferential(out, src, ctx, key);
    });

    const Bucket autoGearBucket = enumerateSchemaFiltered("PhysxVehicleAutoGearBoxAPI",
                                                  "physxVehicleAutoGearBox:upRatios", readOrdinal);
    scanBucket(autoGearBucket, [&](ObjectKey key) { emitVehicleAutoGearBox(out, src, ctx, key); });

    // --- Vehicles: brakes, steering, nonlinear response ---
    const char* const brakeSchemas[] = { "PhysxVehicleBrakesAPI:brakes0", "PhysxVehicleBrakesAPI:brakes1" };
    const Bucket brakesBucket = enumerateSchemasFiltered(brakeSchemas, 2,
                                                 "physxVehicleBrakes:brakes0:maxBrakeTorque", readOrdinal);
    scanBucket(brakesBucket, [&](ObjectKey key) { emitVehicleBrakes(out, src, ctx, key); });

    const Bucket steeringBucket = enumerateSchemaFiltered("PhysxVehicleSteeringAPI",
                                                  "physxVehicleSteering:wheels", readOrdinal);
    scanBucket(steeringBucket, [&](ObjectKey key) { emitVehicleSteeringBasic(out, src, ctx, key); });

    const Bucket ackermannBucket = enumerateSchemaFiltered("PhysxVehicleAckermannSteeringAPI",
                                                   "physxVehicleAckermannSteering:wheel0", readOrdinal);
    const TokenId tokSteeringApi = src.internToken("PhysxVehicleSteeringAPI");
    scanBucket(ackermannBucket, [&](ObjectKey key)
    {
        if (!src.hasSchema(key, tokSteeringApi))
            emitVehicleSteeringAckermann(out, ctx, key);
    });

    const char* const ncrSchemas[] = {
        "PhysxVehicleNonlinearCommandResponseAPI:drive",
        "PhysxVehicleNonlinearCommandResponseAPI:steer",
        "PhysxVehicleNonlinearCommandResponseAPI:brakes0",
        "PhysxVehicleNonlinearCommandResponseAPI:brakes1",
    };
    const Bucket ncrBucket = enumerateSchemasFiltered(ncrSchemas, 4,
                                              "physxVehicleNonlinearCommandResponse:drive:commandValues", readOrdinal);
    scanBucket(ncrBucket, [&](ObjectKey key) { emitVehicleNonlinearCmdResponse(out, src, ctx, key); });

    // --- Vehicles: wheel attachments, suspension compliance, chassis roots ---
    const Bucket wheelAttachmentBucket = enumerateSchemaFiltered("PhysxVehicleWheelAttachmentAPI",
                                                         "physxVehicleWheelAttachment:index", readOrdinal);
    scanBucket(wheelAttachmentBucket, [&](ObjectKey key) { emitVehicleWheelAttachment(out, instance, src, ctx, key); });

    const Bucket suspensionComplianceBucket = enumerateSchemaFiltered("PhysxVehicleSuspensionComplianceAPI",
                                                              "physxVehicleSuspensionCompliance:suspensionForceAppPoint",
                                                              readOrdinal);
    scanBucket(suspensionComplianceBucket, [&](ObjectKey key) { emitVehicleSuspensionCompliance(out, src, ctx, key); });

    const Bucket vehicleBucket = enumerateSchemaFiltered("PhysxVehicleAPI",
                                                 "physxVehicle:vehicleEnabled", readOrdinal);
    scanBucketWithTransform(vehicleBucket, [&](ObjectKey key) { emitVehicle(out, src, ctx, key, lengthScale); });
    }

    return out;
}

} // namespace omni::physics::ovstage
