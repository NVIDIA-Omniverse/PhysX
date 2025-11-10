// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXPropertiesUpdate.h"

#include <PhysXTools.h>
#include <Setup.h>
#include <OmniPhysX.h>

#include <carb/logging/Log.h>

#include <usdLoad/LoadUsd.h>

#include <PxPhysicsAPI.h>


using namespace ::physx;
using namespace carb;
using namespace pxr;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;


// point instancer
bool updateBodyInstancedTransform(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode, bool positionUpdate)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTPointInstancer)
    {
        const UsdPrim prim = attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath);
        const UsdGeomPointInstancer pointInst(prim);
        const GfMatrix4d instancerMatrix = pointInst.ComputeLocalToWorldTransform(UsdTimeCode::Default());

        if (pointInst)
        {
            VtArray<GfVec3f> positions;
            pointInst.GetPositionsAttr().Get(&positions);
            VtArray<GfQuath> orientations;
            pointInst.GetOrientationsAttr().Get(&orientations);
            SdfPathVector targets;
            pointInst.GetPrototypesRel().GetTargets(&targets);

            UsdPrim topBodyPrim = UsdPrim();
            GfMatrix4d topBodyMatrix(1.0);
            GfTransform topBodyTransform;

            const uint64_t instancerApis = attachedStage.getObjectDatabase()->getSchemaAPIs(prim.GetPrimPath());

            for (size_t i = 0; i < targets.size(); i++)
            {
                const usdparser::ObjectIdMap* entries = attachedStage.getObjectIds(targets[i]);
                if (entries && !entries->empty())
                {
                    const UsdGeomXformable protoPrim = UsdGeomXformable(attachedStage.getStage()->GetPrimAtPath(targets[i]));
                    GfMatrix4d localProtoPrimMatrix;
                    bool resetStack = false;
                    protoPrim.GetLocalTransformation(&localProtoPrimMatrix, &resetStack);

                    auto it = entries->begin();
                    while (it != entries->end())
                    {                        
                        if (it->first == eBody)
                        {
                            const usdparser::ObjectId protoObjectId = it->second;
                            objectRecord = db.getFullRecord(internalType, protoObjectId);
                            if (objectRecord && internalType == ePTActor)
                            {
                                const InternalActor* internalActor = (const InternalActor*)objectRecord->mInternalPtr;
                                if (internalActor->mInstanceIndex != kInvalidUint32_t && internalActor->mInstanceIndex < positions.size())
                                {
                                    PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;

                                    const GfVec3f instancePos = internalActor->mInstanceIndex < positions.size() ?
                                                                    positions[internalActor->mInstanceIndex] :
                                                                    GfVec3f(0.0f);
                                    const GfQuatf instanceOrient = internalActor->mInstanceIndex < orientations.size() ?
                                                                    GfQuatf(orientations[internalActor->mInstanceIndex]) :
                                                                    GfQuatf(1.0f);

                                    GfMatrix4d instanceMatrix;
                                    instanceMatrix.SetTranslate(GfVec3d(instancePos));
                                    instanceMatrix.SetRotateOnly(instanceOrient);

                                    const GfMatrix4d bodyMatrix = localProtoPrimMatrix * instanceMatrix * instancerMatrix;
                                    
                                    PxTransform globalPose = actor->getGlobalPose();

                                    const float tolerance = 1e-3f;
                                    bool updateTr = false;
                                    if (positionUpdate)
                                    {
                                        const GfVec3f newPos(bodyMatrix.ExtractTranslation());
                                        if ((fabsf(newPos[0] - globalPose.p.x) > tolerance) ||
                                            (fabsf(newPos[1] - globalPose.p.y) > tolerance) || (fabsf(newPos[2] - globalPose.p.z) > tolerance))
                                        {
                                            globalPose.p = toPhysX(newPos);
                                            updateTr = true;
                                        }
                                    }
                                    else
                                    {
                                        const GfQuatf newRot(bodyMatrix.RemoveScaleShear().ExtractRotationQuat());

                                        const float dot = (newRot.GetImaginary()[0] * globalPose.q.x) +
                                            (newRot.GetImaginary()[1] * globalPose.q.y) +
                                            (newRot.GetImaginary()[2] * globalPose.q.z) + (newRot.GetReal() * globalPose.q.w);
                                        if (abs(dot) < (1.0f - tolerance))
                                        {
                                            globalPose.q = toPhysX(newRot);
                                            updateTr = true;
                                        }
                                    }

                                    if (updateTr)
                                    {
                                        PxRigidDynamic* dynamicActor = actor->is<PxRigidDynamic>();
                                        

                                        if (dynamicActor && (dynamicActor->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
                                        {
                                            dynamicActor->setKinematicTarget(globalPose);
                                        }
                                        else
                                        {
                                            actor->setGlobalPose(globalPose);
                                            if (dynamicActor)
                                            {
                                                dynamicActor->setLinearVelocity(PxVec3(0.0f));
                                                dynamicActor->setAngularVelocity(PxVec3(0.0f));
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        else if ((instancerApis & SchemaAPIFlag::eRigidBodyAPI) && it->first == eShape)
                        {
                            const usdparser::ObjectId protoObjectId = it->second;
                            objectRecord = db.getFullRecord(internalType, protoObjectId);
                            if (objectRecord && internalType == ePTShape)
                            {
                                const InternalShape* internalShape = (const InternalShape*)objectRecord->mInternalPtr;
                                if (internalShape->mInstanceIndex != kInvalidUint32_t && internalShape->mInstanceIndex < positions.size())
                                {
                                    PxShape* shape = (PxShape*)objectRecord->mPtr;

                                    if (topBodyPrim == UsdPrim())
                                    {
                                        PxRigidActor* actor = shape->getActor();
                                        if (actor)
                                        {
                                            objectRecord = db.getFullRecord(internalType, size_t(actor->userData));
                                            if (objectRecord && internalType == ePTActor)
                                            {
                                                topBodyPrim = attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath);
                                                CARB_ASSERT(topBodyPrim);

                                                const GfMatrix4d topBodyPrimMatrix = pointInst.ComputeLocalToWorldTransform(UsdTimeCode::Default());
                                                const GfMatrix4d topBodyPrimMatrixInv = topBodyPrimMatrix.GetInverse();
                                                
                                                topBodyMatrix = instancerMatrix * topBodyPrimMatrixInv;

                                                topBodyTransform = GfTransform(topBodyMatrix);
                                            }
                                        }
                                    }

                                    const GfVec3f instancePos = internalShape->mInstanceIndex < positions.size() ? positions[internalShape->mInstanceIndex] : GfVec3f(0.0f);
                                    const GfQuatf instanceOrient = internalShape->mInstanceIndex < orientations.size() ? GfQuatf(orientations[internalShape->mInstanceIndex]) : GfQuatf(1.0f);

                                    GfMatrix4d instanceMatrix;
                                    instanceMatrix.SetTranslate(GfVec3d(instancePos));
                                    instanceMatrix.SetRotateOnly(instanceOrient);

                                    const GfMatrix4d shapeMatrix = localProtoPrimMatrix * instanceMatrix * topBodyMatrix;

                                    pxr::GfVec3f localPos = pxr::GfVec3f(shapeMatrix.ExtractTranslation());
                                    pxr::GfQuatf localRotOut = pxr::GfQuatf(shapeMatrix.ExtractRotationQuat());

                                    const pxr::GfVec3d sc = topBodyTransform.GetScale();
                                    for (int iA = 0; iA < 3; iA++)
                                    {
                                        localPos[iA] *= (float)sc[iA];
                                    }                                    
                                    PxTransform localPose = shape->getLocalPose();

                                    const float tolerance = 1e-3f;
                                    bool updateTr = false;
                                    if (positionUpdate)
                                    {
                                        const GfVec3f newPos(localPos);
                                        if ((fabsf(newPos[0] - localPose.p.x) > tolerance) ||
                                            (fabsf(newPos[1] - localPose.p.y) > tolerance) || (fabsf(newPos[2] - localPose.p.z) > tolerance))
                                        {
                                            localPose.p = toPhysX(newPos);
                                            updateTr = true;
                                        }
                                    }
                                    else
                                    {
                                        const GfQuatf newRot(shapeMatrix.ExtractRotationQuat());

                                        const float dot = (newRot.GetImaginary()[0] * localPose.q.x) +
                                            (newRot.GetImaginary()[1] * localPose.q.y) +
                                            (newRot.GetImaginary()[2] * localPose.q.z) + (newRot.GetReal() * localPose.q.w);
                                        if (abs(dot) < (1.0f - tolerance))
                                        {
                                            localPose.q = toPhysX(newRot);
                                            updateTr = true;
                                        }
                                    }

                                    if (updateTr)
                                    {
                                        shape->setLocalPose(localPose);
                                    }
                                }
                            }
                        }
                        it++;
                    }
                }
            }
        }
    }
    return true;
}

bool omni::physx::updateBodyInstancedPositions(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateBodyInstancedTransform(attachedStage, objectId, property, timeCode, true);    
}

bool omni::physx::updateBodyInstancedOrientations(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateBodyInstancedTransform(attachedStage, objectId, property, timeCode, false);
}

bool updateBodyInstancedVelocitiesInternal(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode, bool linearVelocity)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTPointInstancer)
    {
        const UsdPrim prim = attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath);
        const UsdGeomPointInstancer pointInst(prim);

        if (pointInst)
        {
            VtArray<GfVec3f> velocities;
            if (linearVelocity)
                pointInst.GetVelocitiesAttr().Get(&velocities);
            else
                pointInst.GetAngularVelocitiesAttr().Get(&velocities);

            SdfPathVector targets;
            pointInst.GetPrototypesRel().GetTargets(&targets);

            for (size_t i = 0; i < targets.size(); i++)
            {
                const usdparser::ObjectIdMap* entries = attachedStage.getObjectIds(targets[i]);
                if (entries && !entries->empty())
                {
                    auto it = entries->begin();
                    while (it != entries->end())
                    {
                        if (it->first == eBody)
                        {
                            const usdparser::ObjectId protoObjectId = it->second;
                            objectRecord = db.getFullRecord(internalType, protoObjectId);
                            if (objectRecord && internalType == ePTActor)
                            {
                                const InternalActor* internalActor = (const InternalActor*)objectRecord->mInternalPtr;
                                if (internalActor->mInstanceIndex != 0xffffffff && internalActor->mInstanceIndex < velocities.size())
                                {
                                    PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
                                    PxRigidDynamic* dynamicActor = actor->is<PxRigidDynamic>();

                                    if (dynamicActor && !((dynamicActor->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)))
                                    {
                                        if (linearVelocity)
                                            dynamicActor->setLinearVelocity(toPhysX(velocities[internalActor->mInstanceIndex]));
                                        else
                                            dynamicActor->setAngularVelocity(toPhysX(degToRad(velocities[internalActor->mInstanceIndex])));
                                    }
                                }
                            }
                        }
                        it++;
                    }
                }
            }
        }
    }
    return true;
}

bool omni::physx::updateBodyInstancedVelocities(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateBodyInstancedVelocitiesInternal(attachedStage, objectId, property, timeCode, true);
}

bool omni::physx::updateBodyInstancedAngularVelocities(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateBodyInstancedVelocitiesInternal(attachedStage, objectId, property, timeCode, false);
}
