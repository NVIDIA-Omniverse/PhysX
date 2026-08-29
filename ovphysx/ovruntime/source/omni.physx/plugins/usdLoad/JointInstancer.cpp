// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <omni/physics/usd/PrimIterator.h>
#include <common/foundation/Allocator.h>

#include "LoadTools.h"
#include "LoadUsd.h"
#include "PointInstancer.h"
#include "Joint.h"

#include <internal/InternalActor.h>

#include <PhysXTools.h>
#include <OmniPhysX.h>
#include <PhysXScene.h>

#include <omni/physics/usd/StageScan.h>

#include "IceDescriptorAllocator.h"


using namespace PXR_NS;
using namespace carb;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{
namespace usdparser
{

namespace
{
// schemaTypeToken lives in PhysXTools.h (single boundary translation).
using omni::physx::internal::schemaTypeToken;
} // namespace

// Parse the first joint found under `targetPath`'s subtree.  Used by the
// joint-instancer to fetch a prototype joint descriptor it can stamp
// per instance.  Runs scanStage on the subtree and returns the first
// joint emitted. (No UsdPrim — the stage comes from the AttachedStage.)
std::pair<SdfPath, PhysxObjectDesc*> parseJointPrototype(AttachedStage& attachedStage,
    const SdfPath& targetPath,
    const ObjectInstance& objectInstance)
{
    (void)objectInstance;

    const std::vector<SdfPath> scanRoots{ targetPath };
    static const std::unordered_set<SdfPath, SdfPath::Hash> kNoExclude;
    omni::physics::usd::ScannedStage scanned = omni::physics::usd::scanStage(
        attachedStage.attachTarget(), scanRoots, kNoExclude,
        omni::physx::usdparser::iceDescriptorAllocator());

    if (scanned.joints.empty())
        return std::make_pair(SdfPath(), nullptr);

    // Take ownership of the first joint descriptor. `release()` hands the
    // raw pointer to the caller, whose lifecycle today does not free it.
    PXR_NS::SdfPath protoPath = scanned.pathFor(scanned.joints[0]->jointPrimKey);
    PhysxObjectDesc* protoDesc = scanned.joints[0].release();
    return std::make_pair(protoPath, protoDesc);
}

ObjectId resolveBodyPath(AttachedStage& attachedStage, const SdfPath& bodyPath, int bodyIndex)
{    
    ObjectDb* objectDb = attachedStage.getObjectDatabase();
    internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (src && bodyPath != SdfPath())
    {
        const omni::physics::parse::ObjectKey bodyKey = attachedStage.keyFor(bodyPath);
        if (src->exists(bodyKey))
        {
            // A.B. this is super slow and extremely inefficient
            if (bodyIndex >= 0 && src->isA(bodyKey, schemaTypeToken<UsdGeomPointInstancer>(*src)))
            {
                VtArray<int> indices;
                internal::getArrayValue(attachedStage, bodyKey, UsdGeomTokens->protoIndices, UsdTimeCode::Default(), indices);
                if (bodyIndex < indices.size())
                {
                    const int protoIndex = indices[bodyIndex];
                    SdfPathVector protos;
                    {
                        std::vector<omni::physics::parse::ObjectKey> protoKeys;
                        src->getRelationshipTargets(bodyKey, src->internToken(UsdGeomTokens->prototypes.GetString()), protoKeys);
                        protos.reserve(protoKeys.size());
                        for (const auto& k : protoKeys)
                            protos.push_back(attachedStage.pathFor(k));
                    }
                    if (protoIndex < protos.size())
                    {
                        const SdfPath protoBodyPath = protos[protoIndex];
                        const ObjectIdMap* entries = objectDb->getEntries(protoBodyPath);
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
                ObjectId body = objectDb->findEntry(bodyPath, eBody);
                if (body == kInvalidObjectId)
                {
                    body = objectDb->findEntry(bodyPath, eArticulationLink);
                }
                return body;
            }
        }
    }

    return kInvalidObjectId;
}

void parseJointInstancer(AttachedStage& attachedStage,
    const SdfPath& instancerPath)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const omni::physics::parse::ObjectKey instancerKey = attachedStage.keyFor(instancerPath);

    // Resolve a relationship's targets into prim paths (source key-space).
    const auto readTargets = [&](const PXR_NS::TfToken& rel) -> SdfPathVector
    {
        SdfPathVector out;
        if (!src)
            return out;
        std::vector<omni::physics::parse::ObjectKey> keys;
        src->getRelationshipTargets(instancerKey, src->internToken(rel.GetString()), keys);
        out.reserve(keys.size());
        for (const auto& k : keys)
            out.push_back(attachedStage.pathFor(k));
        return out;
    };
    // Prefer the default value, fall back to the earliest time sample
    // (mirrors getAttributeArrayTimedFallback) — all via the source.
    const auto readArray = [&](const PXR_NS::TfToken& attr, auto& out) -> bool
    {
        return internal::getArrayValue(attachedStage, instancerKey, attr, UsdTimeCode::Default(), out) ||
               internal::getArrayValue(attachedStage, instancerKey, attr, UsdTimeCode::EarliestTime(), out);
    };

    SdfPathVector targets = readTargets(PhysxSchemaTokens->physicsPrototypes);
    TargetDescVector targetObjects;

    // these attributes are required and must match in length
    VtArray<int> indices;
    if (!readArray(PhysxSchemaTokens->physicsProtoIndices, indices))
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) indices array not valid\n", instancerPath.GetText());
    }

    if (indices.size() == 0)
    {
        return;
    }

    // gather the joint instancer data
    SdfPathVector body0s = readTargets(PhysxSchemaTokens->physicsBody0s);
    SdfPathVector body1s = readTargets(PhysxSchemaTokens->physicsBody1s);
    VtArray<int> body0indices;
    if (!readArray(PhysxSchemaTokens->physicsBody0Indices, body0indices))
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) body0 indices array not valid\n", instancerPath.GetText());
    }
    VtArray<int> body1indices;
    if (!readArray(PhysxSchemaTokens->physicsBody1Indices, body1indices))
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) body1 indices array not valid\n", instancerPath.GetText());
    }
    VtArray<GfVec3f> localPos0s;
    if (!readArray(PhysxSchemaTokens->physicsLocalPos0s, localPos0s) || localPos0s.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) localPos0 array not valid\n", instancerPath.GetText());
    }
    VtArray<GfQuath> localRot0s;
    if (readArray(PhysxSchemaTokens->physicsLocalRot0s, localRot0s) && localRot0s.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) localRot0 array size does not match instance count\n",
            instancerPath.GetText());
    }
    VtArray<GfVec3f> localPos1s;
    if (!readArray(PhysxSchemaTokens->physicsLocalPos1s, localPos1s) || localPos1s.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) localPos1 array not valid\n", instancerPath.GetText());
    }
    VtArray<GfQuath> localRot1s;
    if (readArray(PhysxSchemaTokens->physicsLocalRot1s, localRot1s) && localRot1s.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) localRot1 array size does not match instance count\n",
            instancerPath.GetText());
    }

    for (size_t i = 0; i < targets.size(); i++)
    {
        ObjectInstance objectInstance = { instancerPath, (uint32_t)i, SdfPath(), false };
        const SdfPath& targetPath = targets[i];
        if (src && src->exists(attachedStage.keyFor(targetPath)))
        {
            TargetDesc targetDesc;
            std::pair<SdfPath, PhysxObjectDesc*> proto = parseJointPrototype(attachedStage, targetPath, objectInstance);
            targetDesc.desc = proto.second;
            targetDesc.descPath = proto.first;
            targetDesc.outsideInstancer = isOutsideInstancer(src, attachedStage.keyFor(targetPath), instancerKey);
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

        ObjectInstance objectInstance = { instancerPath, (uint32_t)i, targets[indices[i]], false };
        const TargetDesc& targetDesc = targetObjects[indices[i]];
        PhysxObjectDesc* objectDesc = targetDesc.desc;

        if (!objectDesc)
            continue;

        if (objectDesc->type >= eJointFixed || objectDesc->type <= eJointCustom)
        {
            PhysxJointDesc* jointDesc = static_cast<PhysxJointDesc*>(objectDesc);

            const GfVec3f instanceLocalPos0 = i < localPos0s.size() ? localPos0s[i] : GfVec3f(0.0f);
            const GfQuatd instanceLocalRot0 = i < localRot0s.size() ? GfQuatd(localRot0s[i]) : GfQuatd(1.0);
            const GfVec3f instanceLocalPos1 = i < localPos1s.size() ? localPos1s[i] : GfVec3f(0.0f);
            const GfQuatd instanceLocalRot1 = i < localRot1s.size() ? GfQuatd(localRot1s[i]) : GfQuatd(1.0);

            const GfVec3f localJointPos0 = (GfVec3f&)jointDesc->localPose0Position;
            GfQuatd localJointRot0;
            Float4ToGfQuat(jointDesc->localPose0Orientation, localJointRot0);
            const GfVec3f localJointPos1 = (GfVec3f&)jointDesc->localPose1Position;
            GfQuatd localJointRot1;
            Float4ToGfQuat(jointDesc->localPose1Orientation, localJointRot1);

            GfMatrix4d localPose0Matrix;
            localPose0Matrix.SetTranslate(GfVec3d(localJointPos0));
            localPose0Matrix.SetRotateOnly(localJointRot0);

            GfMatrix4d localPose1Matrix;
            localPose1Matrix.SetTranslate(GfVec3d(localJointPos1));
            localPose1Matrix.SetRotateOnly(localJointRot1);

            GfMatrix4d instancePose0Matrix;
            instancePose0Matrix.SetTranslate(GfVec3d(instanceLocalPos0));
            instancePose0Matrix.SetRotateOnly(instanceLocalRot0);

            GfMatrix4d instancePose1Matrix;
            instancePose1Matrix.SetTranslate(GfVec3d(instanceLocalPos1));
            instancePose1Matrix.SetRotateOnly(instanceLocalRot1);

            const GfMatrix4d pose0Matrix = localPose0Matrix * instancePose0Matrix;
            const GfMatrix4d pose1Matrix = localPose1Matrix * instancePose1Matrix;

            GfVec3ToFloat3(pose0Matrix.ExtractTranslation(), jointDesc->localPose0Position);            
            GfQuatToFloat4(GfQuatf(pose0Matrix.ExtractRotation().GetQuat()), jointDesc->localPose0Orientation);

            GfVec3ToFloat3(pose1Matrix.ExtractTranslation(), jointDesc->localPose1Position);
            GfQuatToFloat4(GfQuatf(pose1Matrix.ExtractRotation().GetQuat()), jointDesc->localPose1Orientation);

            const SdfPath body0Path = i < body0s.size() ? body0s[i] : (body0s.empty() ? SdfPath() : body0s[0]);
            const SdfPath body1Path = i < body1s.size() ? body1s[i] : (body1s.empty() ? SdfPath() : body1s[0]);

            const int body0Index = i < body0indices.size() ? body0indices[i] : -1;
            const int body1Index = i < body1indices.size() ? body1indices[i] : -1;

            const ObjectId resolvedBody0 = resolveBodyPath(attachedStage, body0Path, body0Index);
            const ObjectId resolvedBody1 = resolveBodyPath(attachedStage, body1Path, body1Index);

            const bool body0Dynamic = resolvedBody0 == kInvalidObjectId ? false : isRigidBodyDynamic(resolvedBody0);
            const bool body1Dynamic = resolvedBody1 == kInvalidObjectId ? false : isRigidBodyDynamic(resolvedBody1);

            createJoint(attachedStage, targetDesc.descPath, jointDesc, resolvedBody0, body0Dynamic, resolvedBody1, body1Dynamic);

            // change back the desc localPose
            GfVec3ToFloat3(localJointPos0, jointDesc->localPose0Position);
            GfQuatToFloat4(GfQuatf(localJointRot0), jointDesc->localPose0Orientation);

            GfVec3ToFloat3(localJointPos1, jointDesc->localPose1Position);
            GfQuatToFloat4(GfQuatf(localJointRot1), jointDesc->localPose1Orientation);

        }  
    }
}
}
}
}
