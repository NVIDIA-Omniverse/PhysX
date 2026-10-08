// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27 AC-28
 *
 * @implements REQ-PARSE-JOINT-004
 * @covers AC-1 AC-2
 *
 * @implements REQ-PARSE-CORE-006
 * @covers AC-8
 */

#include <limits>

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <omni/physics/parse/KnownTokens.h>
#include <omni/physics/parse/ScanBackend.h>
#include <omni/physics/parse/ScannedStage.h>
#include <common/foundation/Allocator.h>

#include "LoadTools.h"
#include "LoadUsd.h"
#include "PointInstancer.h"
#include "Joint.h"

#include <internal/InternalActor.h>

#include <PhysXTools.h>
// pxr-free half of TypeCast.h: this file only ever names the carb <-> PhysX
// overloads (toPhysX/toPhysXQuat/toFloat3/toFloat4), never a Gf type. Must
// follow a PhysX header: (Carb)PhysXCast.h assumes ::physx is declared.
#include <common/foundation/CarbPhysXCast.h>
#include <OmniPhysX.h>
#include <PhysXScene.h>

#include "IceDescriptorAllocator.h"

using namespace carb;
using namespace ::physx;

namespace omni
{
namespace physx
{
namespace usdparser
{

using omni::physics::parse::ObjectKey;

// Parse the first joint found under `targetKey`'s subtree.  Used by the
// joint-instancer to fetch a prototype joint descriptor it can stamp
// per instance.  Runs scanStage on the subtree and returns the first
// joint emitted. (No UsdPrim -- the scan is driven off the AttachedStage's
// attach target via the backend-dispatched scan path, ScanBackend.h.)
//
// Lifetime: `out.scannedStage` owns the ScannedStage, so `out.desc` stays a
// non-owning pointer into it and is freed with it -- same contract as
// PointInstancer.cpp's parsePrototype.
void parseJointPrototype(AttachedStage& attachedStage,
    ObjectKey targetKey,
    const ObjectInstance& objectInstance,
    TargetDesc& out)
{
    (void)objectInstance;

    const std::vector<std::string> scanRoots{ std::string(attachedStage.textViewFor(targetKey)) };
    static const std::vector<std::string> kNoExclude;
    omni::physics::parse::ScanOptions scanOptions;
    auto scanned = std::make_unique<omni::physics::parse::ScannedStage>();
    *scanned = omni::physics::parse::scanStage(attachedStage.attachTarget(), scanRoots, kNoExclude, scanOptions,
                                               omni::physx::usdparser::iceDescriptorAllocator());

    if (scanned->joints.empty())
    {
        out.scannedStage = std::move(scanned);
        return;
    }

    // Re-key the joint's scan-space key into attachedStage-space via the scanned
    // source's own string identity (the same opaque-identity round trip
    // ScannedShapeCookingDispatch.cpp's ObjectKey-native functions use).
    out.descKey = attachedStage.keyFor(scanned->source().sourceKeyToString(scanned->joints[0]->jointPrimKey));
    out.desc = scanned->joints[0].get();
    out.scannedStage = std::move(scanned);
}

ObjectId resolveBodyPath(AttachedStage& attachedStage, ObjectKey bodyKey, int bodyIndex)
{
    ObjectDb* objectDb = attachedStage.getObjectDatabase();
    internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (src && bodyKey.valid())
    {
        const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
        if (src->exists(bodyKey))
        {
            // A.B. this is super slow and extremely inefficient
            if (bodyIndex >= 0 && src->isA(bodyKey, tok.pointInstancerType))
            {
                std::vector<int32_t> indices;
                internal::getArrayValue(attachedStage, bodyKey, tok.protoIndices,
                                         omni::physics::parse::ReadTime::defaultTime(), indices);
                if (bodyIndex < indices.size())
                {
                    const int protoIndex = indices[bodyIndex];
                    std::vector<omni::physics::parse::ObjectKey> protoKeys;
                    src->getRelationshipTargets(bodyKey, tok.prototypes, protoKeys);
                    if (protoIndex < protoKeys.size())
                    {
                        const ObjectIdMap* entries = objectDb->getEntries(protoKeys[protoIndex]);
                        if (entries)
                        {
                            ObjectIdMap::const_iterator it = entries->begin();
                            while (it != entries->end())
                            {
                                if (it->first == eBody)
                                {
                                    const ObjectId body = it->second;
                                    if (body < db.getRecords().size())
                                    {
                                        const internal::InternalDatabase::Record& rec = db.getRecords()[body];
                                        internal::InternalActor* internalActor = (internal::InternalActor*)rec.mInternalPtr;
                                        if (internalActor->mInstanceIndex == bodyIndex)
                                        {
                                            return body;
                                        }
                                    }
                                }
                                it++;
                            }
                        }
                    }
                }
            }
            else
            {
                ObjectId body = objectDb->findEntry(bodyKey, eBody);
                if (body == kInvalidObjectId)
                {
                    body = objectDb->findEntry(bodyKey, eArticulationLink);
                }
                return body;
            }
        }
    }

    return kInvalidObjectId;
}

void parseJointInstancer(AttachedStage& attachedStage,
    omni::physics::parse::ObjectKey instancerKey)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    // Diagnostics-only path text for the CARB_LOG_WARN calls below; textFor()
    // avoids materializing a path just to format a log string.
    const char* instancerText = attachedStage.textFor(instancerKey);

    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();

    // Resolve a relationship's targets into ObjectKeys (source key-space, which is
    // attachedStage's own -- no re-key round trip needed, unlike parseJointPrototype's
    // scanned-subtree descs above).
    const auto readTargets = [&](omni::physics::parse::TokenId rel) -> std::vector<ObjectKey>
    {
        std::vector<ObjectKey> out;
        if (!src)
            return out;
        src->getRelationshipTargets(instancerKey, rel, out);
        return out;
    };
    // Prefer the default value, fall back to the earliest time sample
    // (mirrors getAttributeArrayTimedFallback) — all via the source.
    const auto readArray = [&](omni::physics::parse::TokenId attr, auto& out) -> bool
    {
        return internal::getArrayValue(attachedStage, instancerKey, attr,
                                        omni::physics::parse::ReadTime::defaultTime(), out) ||
               // std::numeric_limits<double>::lowest() is UsdTimeCode::EarliestTime()'s
               // exact underlying value (pxr/usd/usd/timeCode.h); ReadTime::at() round-trips
               // it straight back into UsdTimeCode(t) on the USD backend, so this is the
               // same earliest-time read without needing the pxr timeCode header here.
               internal::getArrayValue(attachedStage, instancerKey, attr,
                                        omni::physics::parse::ReadTime::at(std::numeric_limits<double>::lowest()),
                                        out);
    };

    std::vector<ObjectKey> targets = readTargets(tok.physicsPrototypes);
    TargetDescVector targetObjects;

    // these attributes are required and must match in length
    std::vector<int32_t> indices;
    if (!readArray(tok.physicsProtoIndices, indices))
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) indices array not valid\n", instancerText);
    }

    if (indices.size() == 0)
    {
        return;
    }

    // gather the joint instancer data
    std::vector<ObjectKey> body0s = readTargets(tok.physicsBody0s);
    std::vector<ObjectKey> body1s = readTargets(tok.physicsBody1s);
    std::vector<int32_t> body0indices;
    if (!readArray(tok.physicsBody0Indices, body0indices))
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) body0 indices array not valid\n", instancerText);
    }
    std::vector<int32_t> body1indices;
    if (!readArray(tok.physicsBody1Indices, body1indices))
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) body1 indices array not valid\n", instancerText);
    }
    std::vector<carb::Float3> localPos0s;
    if (!readArray(tok.physicsLocalPos0s, localPos0s) || localPos0s.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) localPos0 array not valid\n", instancerText);
    }
    // carb::Float4 also serves the on-disk GfQuath storage -- PhysXTools.h's fillArray
    // dispatches on the source BufferElemType (eQuath) at runtime and widens half->float,
    // it is not a memcpy.
    std::vector<carb::Float4> localRot0s;
    if (readArray(tok.physicsLocalRot0s, localRot0s) && localRot0s.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) localRot0 array size does not match instance count\n",
            instancerText);
    }
    std::vector<carb::Float3> localPos1s;
    if (!readArray(tok.physicsLocalPos1s, localPos1s) || localPos1s.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) localPos1 array not valid\n", instancerText);
    }
    std::vector<carb::Float4> localRot1s;
    if (readArray(tok.physicsLocalRot1s, localRot1s) && localRot1s.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) localRot1 array size does not match instance count\n",
            instancerText);
    }

    for (size_t i = 0; i < targets.size(); i++)
    {
        ObjectInstance objectInstance = { instancerKey, (uint32_t)i, omni::physics::parse::ObjectKey{}, false };
        const ObjectKey targetKey = targets[i];
        if (src && src->exists(targetKey))
        {
            TargetDesc targetDesc;
            parseJointPrototype(attachedStage, targetKey, objectInstance, targetDesc);
            targetDesc.outsideInstancer = isOutsideInstancer(src, targetKey, instancerKey);
            targetObjects.push_back(std::move(targetDesc));
        }
        else
        {
            targetObjects.emplace_back();
        }
    }

    for (size_t i = 0; i < indices.size(); i++)
    {
        if (size_t(indices[i]) >= targets.size())
            continue;

        ObjectInstance objectInstance = { instancerKey, (uint32_t)i, targets[indices[i]], false };
        const TargetDesc& targetDesc = targetObjects[indices[i]];
        PhysxObjectDesc* objectDesc = targetDesc.desc;

        if (!objectDesc)
            continue;

        if (objectDesc->type >= eJointFixed || objectDesc->type <= eJointCustom)
        {
            PhysxJointDesc* jointDesc = static_cast<PhysxJointDesc*>(objectDesc);

            // USD read boundary: convert the authored instance arrays to PhysX math once,
            // then compose entirely with PxTransform (Gf's row-vector matrix product
            // A * B corresponds to the PxTransform product B * A).
            // toPhysXQuat(GfQuath) yields the same four floats the old
            // toPhysX(GfQuatd(GfQuath)) did: both go through GetImaginary()/GetReal(),
            // and half->double->float is the same value as half->float.
            const PxTransform instancePose0(
                i < localPos0s.size() ? toPhysX(localPos0s[i]) : PxVec3(0.0f),
                i < localRot0s.size() ? toPhysXQuat(localRot0s[i]) : PxQuat(PxIdentity));
            const PxTransform instancePose1(
                i < localPos1s.size() ? toPhysX(localPos1s[i]) : PxVec3(0.0f),
                i < localRot1s.size() ? toPhysXQuat(localRot1s[i]) : PxQuat(PxIdentity));

            const PxTransform localPose0 = toPhysX(jointDesc->localPose0Position, jointDesc->localPose0Orientation);
            const PxTransform localPose1 = toPhysX(jointDesc->localPose1Position, jointDesc->localPose1Orientation);

            // The authored orientations are half-precision, so composing them can drift
            // outside PxQuat::isUnit(). The Gf path normalized implicitly (GfRotation
            // stores axis + angle); do it explicitly here.
            const PxTransform pose0 = (instancePose0 * localPose0).getNormalized();
            const PxTransform pose1 = (instancePose1 * localPose1).getNormalized();

            jointDesc->localPose0Position = toFloat3(pose0.p);
            jointDesc->localPose0Orientation = toFloat4(pose0.q);

            jointDesc->localPose1Position = toFloat3(pose1.p);
            jointDesc->localPose1Orientation = toFloat4(pose1.q);

            const ObjectKey body0Key = i < body0s.size() ? body0s[i] : (body0s.empty() ? ObjectKey{} : body0s[0]);
            const ObjectKey body1Key = i < body1s.size() ? body1s[i] : (body1s.empty() ? ObjectKey{} : body1s[0]);

            const int body0Index = i < body0indices.size() ? body0indices[i] : -1;
            const int body1Index = i < body1indices.size() ? body1indices[i] : -1;

            const ObjectId resolvedBody0 = resolveBodyPath(attachedStage, body0Key, body0Index);
            const ObjectId resolvedBody1 = resolveBodyPath(attachedStage, body1Key, body1Index);

            const bool body0Dynamic = resolvedBody0 == kInvalidObjectId ? false : isRigidBodyDynamic(resolvedBody0);
            const bool body1Dynamic = resolvedBody1 == kInvalidObjectId ? false : isRigidBodyDynamic(resolvedBody1);

            createJoint(attachedStage, targetDesc.descKey, jointDesc, resolvedBody0, body0Dynamic, resolvedBody1, body1Dynamic);

            // change back the desc localPose
            jointDesc->localPose0Position = toFloat3(localPose0.p);
            jointDesc->localPose0Orientation = toFloat4(localPose0.q);

            jointDesc->localPose1Position = toFloat3(localPose1.p);
            jointDesc->localPose1Orientation = toFloat4(localPose1.q);

        }
    }
}
}
}
}
