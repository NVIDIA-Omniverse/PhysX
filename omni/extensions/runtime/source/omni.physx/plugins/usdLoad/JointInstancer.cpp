// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <private/omni/physics/schema/IUsdPhysics.h>
#include <common/foundation/Allocator.h>

#include "LoadTools.h"
#include "LoadUsd.h"
#include "PointInstancer.h"
#include "Joint.h"

#include <internal/InternalActor.h>

#include <PhysXTools.h>
#include <OmniPhysX.h>
#include <PhysXScene.h>


using namespace pxr;
using namespace carb;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{
namespace usdparser
{
            
class PhysxJointPrototypeListener : public omni::physics::schema::IUsdPhysicsListener
{
public:

    PhysxJointPrototypeListener(AttachedStage& inAttachedStage, UsdGeomXformCache& inXfCache,
        const UsdPrim& inUsdPrim,        
        const ObjectInstance& inObjectInstance)
        : protoDesc(nullptr), usdPrim(inUsdPrim),
        objectInstance(inObjectInstance),
        xfCache(inXfCache),        
        attachedStage(inAttachedStage)
    {
        stage = attachedStage.getStage();
    }

    void parsePrim(const pxr::UsdPrim& prim, omni::physics::schema::ObjectDesc* objectDesc, uint64_t, const pxr::TfTokenVector& appliedApis) override
    {
    }

    void reportObjectDesc(const pxr::SdfPath& path, const omni::physics::schema::ObjectDesc* objectDesc) override
    {
        switch (objectDesc->type)
        {
        case omni::physics::schema::ObjectType::eJointFixed:
        case omni::physics::schema::ObjectType::eJointRevolute:
        case omni::physics::schema::ObjectType::eJointPrismatic:
        case omni::physics::schema::ObjectType::eJointSpherical:
        case omni::physics::schema::ObjectType::eJointDistance:
        case omni::physics::schema::ObjectType::eJointD6:
        case omni::physics::schema::ObjectType::eJointCustom:        
        {
            const JointDesc* inDesc = (const JointDesc*)objectDesc;
            protoDesc = parseJoint(stage, *inDesc, xfCache);
            protoPath = path;
        }
        break;
        default:
            break;
        }
    }

    PhysxObjectDesc* getProtoDesc() const
    {
        return protoDesc;
    }

    const SdfPath& getProtoPath() const
    {
        return protoPath;
    }

    void parsePrim()
    {
        IUsdPhysics* usdPhysics = carb::getCachedInterface<IUsdPhysics>();
        usdPhysics->registerPhysicsListener(this);
        pxr::UsdPrimRange range(usdPrim, UsdTraverseInstanceProxies(pxr::UsdPrimIsActive && pxr::UsdPrimIsLoaded && !pxr::UsdPrimIsAbstract));
        UsdGeomXformCache xfCache;
        omni::physics::schema::PrimIteratorRange primIteratorRange(range);
        usdPhysics->loadFromRange(stage, xfCache, primIteratorRange);
        usdPhysics->unregisterPhysicsListener(this);
    }

    void clear()
    {
    }

private:
    PhysxObjectDesc* protoDesc;
    SdfPath         protoPath;
    ObjectIdVector   shapeIds;
    UsdStageWeakPtr stage;
    const UsdPrim& usdPrim;
    const ObjectInstance& objectInstance;
    UsdGeomXformCache& xfCache;
    AttachedStage& attachedStage;
};

std::pair<SdfPath, PhysxObjectDesc*> parseJointPrototype(AttachedStage& attachedStage, UsdGeomXformCache& xfCache,
    const UsdPrim& usdPrim,    
    const ObjectInstance& objectInstance)
{
    PhysxJointPrototypeListener prototypeListener(attachedStage, xfCache, usdPrim, objectInstance);
    prototypeListener.parsePrim();
    prototypeListener.clear();

    return std::make_pair(prototypeListener.getProtoPath(), prototypeListener.getProtoDesc());
}

ObjectId resolveBodyPath(AttachedStage& attachedStage, const SdfPath& bodyPath, int bodyIndex)
{    
    ObjectDb* objectDb = attachedStage.getObjectDatabase();
    internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    if (attachedStage.getStage() && bodyPath != SdfPath())
    {
        const UsdPrim bodyPrim = attachedStage.getStage()->GetPrimAtPath(bodyPath);
        if (bodyPrim)
        {
            // A.B. this is super slow and extremely inefficient 
            if (bodyPrim.IsA<UsdGeomPointInstancer>() && bodyIndex >= 0)
            {
                UsdGeomPointInstancer instancer(bodyPrim);
                VtArray<int> indices;
                instancer.GetProtoIndicesAttr().Get(&indices);
                if (bodyIndex < indices.size())
                {
                    const int protoIndex = indices[bodyIndex];
                    SdfPathVector protos;
                    instancer.GetPrototypesRel().GetTargets(&protos);
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

void parseJointInstancer(AttachedStage& attachedStage, UsdGeomXformCache& xfCache,
    const UsdPrim& usdPrim)
{
    const PhysxSchemaPhysxPhysicsJointInstancer instancer(usdPrim);
    UsdRelationship prototypes = instancer.GetPhysicsPrototypesRel();

    SdfPathVector targets;
    TargetDescVector targetObjects;
    prototypes.GetTargets(&targets);

    UsdTimeCode earliestTime = UsdTimeCode::EarliestTime();

    // these attributes are required and must match in length
    VtArray<int> indices;
    if (!getAttributeArrayTimedFallback(indices, instancer.GetPhysicsProtoIndicesAttr(), earliestTime))
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) indices array not valid\n", instancer.GetPath().GetText());
    }

    if (indices.size() == 0)
    {
        return;
    }

    // gather the joint instancer data
    SdfPathVector body0s;
    {
        UsdRelationship rel = instancer.GetPhysicsBody0sRel();
        rel.GetTargets(&body0s);
    }
    SdfPathVector body1s;
    {
        UsdRelationship rel = instancer.GetPhysicsBody1sRel();
        rel.GetTargets(&body1s);
    }
    VtArray<int> body0indices;
    if (!getAttributeArrayTimedFallback(body0indices, instancer.GetPhysicsBody0IndicesAttr(), earliestTime))
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) body0 indices array not valid\n", instancer.GetPath().GetText());
    }
    VtArray<int> body1indices;
    if (!getAttributeArrayTimedFallback(body1indices, instancer.GetPhysicsBody1IndicesAttr(), earliestTime))
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) body1 indices array not valid\n", instancer.GetPath().GetText());
    }
    VtArray<GfVec3f> localPos0s;
    if (!getAttributeArrayTimedFallback(localPos0s, instancer.GetPhysicsLocalPos0sAttr(), earliestTime) || localPos0s.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) localPos0 array not valid\n", instancer.GetPath().GetText());
    }
    VtArray<GfQuath> localRot0s;
    if (getAttributeArrayTimedFallback(localRot0s, instancer.GetPhysicsLocalRot0sAttr(), earliestTime) && localRot0s.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) localRot0 array size does not match instance count\n",
            instancer.GetPath().GetText());
    }
    VtArray<GfVec3f> localPos1s;
    if (!getAttributeArrayTimedFallback(localPos1s, instancer.GetPhysicsLocalPos1sAttr(), earliestTime) || localPos1s.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) localPos1 array not valid\n", instancer.GetPath().GetText());
    }
    VtArray<GfQuath> localRot1s;
    if (getAttributeArrayTimedFallback(localRot1s, instancer.GetPhysicsLocalRot1sAttr(), earliestTime) && localRot1s.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:JointInstancer: (%s) localRot1 array size does not match instance count\n",
            instancer.GetPath().GetText());
    }

    for (size_t i = 0; i < targets.size(); i++)
    {
        ObjectInstance objectInstance = { usdPrim.GetPrimPath(), (uint32_t)i, SdfPath(), false };
        const UsdPrim targetPrim = attachedStage.getStage()->GetPrimAtPath(targets[i]);
        if (targetPrim)
        {
            TargetDesc targetDesc;
            targetDesc.targetPrim = targetPrim;
            std::pair<SdfPath, PhysxObjectDesc*> proto = parseJointPrototype(attachedStage, xfCache, targetPrim, objectInstance);
            targetDesc.desc = proto.second;
            targetDesc.descPath = proto.first;
            targetDesc.outsideInstancer = isOutsideInstancer(targetPrim, usdPrim);
            targetObjects.push_back(targetDesc);
        }
        else
        {
            targetObjects.push_back(TargetDesc());
        }
    }

    for (size_t i = 0; i < indices.size(); i++)
    {
        if (size_t(indices[i]) >= targets.size())
            continue;

        ObjectInstance objectInstance = { usdPrim.GetPrimPath(), (uint32_t)i, targets[indices[i]], false };
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

            // Change the desc
            // local poses
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
