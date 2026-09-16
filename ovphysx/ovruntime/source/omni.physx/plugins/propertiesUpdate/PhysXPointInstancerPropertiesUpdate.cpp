// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-INSTANCER-002
 * @covers AC-2
 *
 * @implements REQ-PARSE-INSTANCER-003
 * @covers AC-2 AC-3
 */

#include "PhysXPropertiesUpdate.h"

#include <omni/physics/parse/KnownTokens.h>

#include <PhysXTools.h>
#include <Setup.h>
#include <OmniPhysX.h>

#include <carb/logging/Log.h>

#include <usdLoad/LoadUsd.h>

#include <PxPhysicsAPI.h>


using namespace ::physx;
using namespace carb;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;


// point instancer
bool updateBodyInstancedTransform(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode, bool positionUpdate)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTPointInstancer)
    {
        const omni::physics::parse::ObjectKey instancerKey = objectRecord->mKey;
        const PxMat44d instancerMatrix = getWorldTransform(attachedStage, instancerKey, omni::physics::parse::ReadTime::defaultTime());

        const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
        omni::physics::parse::KnownTokens tok;
        if (source)
            tok.intern(*source);

        std::vector<carb::Float3> positions;
        getArrayValue(attachedStage, instancerKey, tok.positions, omni::physics::parse::ReadTime::defaultTime(), positions);
        // Orientations are authored as half-precision quaternions; the array read
        // widens them to carb::Float4 lanes x,y,z,w with w == the real part
        // (PhysXTools.h fillArray ladder's eQuath -> carb::Float4 overload).
        std::vector<carb::Float4> orientations;
        getArrayValue(attachedStage, instancerKey, tok.orientations, omni::physics::parse::ReadTime::defaultTime(), orientations);
        std::vector<omni::physics::parse::ObjectKey> targets;
        getRelationshipValue(attachedStage, instancerKey, tok.prototypes, targets);

        bool topBodyResolved = false;
        PxMat44d topBodyMatrix(PxIdentity);
        PxVec3 topBodyScale(1.0f);

        const uint64_t instancerApis = attachedStage.getObjectDatabase()->getSchemaAPIs(instancerKey);

        for (size_t i = 0; i < targets.size(); i++)
        {
                const usdparser::ObjectIdMap* entries = attachedStage.getObjectIds(targets[i]);
                if (entries && !entries->empty())
                {
                    bool resetStack = false;
                    const PxMat44d localProtoPrimMatrix = getLocalTransform(
                        attachedStage, targets[i], omni::physics::parse::ReadTime::defaultTime(), resetStack);

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

                                    // toPhysXQuat(carb::Float4) is the same four floats the old
                                    // toPhysX(GfQuatf(GfQuath)) produced (both via
                                    // GetImaginary()/GetReal(); GfHalf->float is exact), and
                                    // GfQuatf(1.0f) was the identity.
                                    const PxVec3 instancePos = internalActor->mInstanceIndex < positions.size() ?
                                                                    toPhysX(positions[internalActor->mInstanceIndex]) :
                                                                    PxVec3(0.0f);
                                    const PxQuat instanceOrient = internalActor->mInstanceIndex < orientations.size() ?
                                                                    toPhysXQuat(orientations[internalActor->mInstanceIndex]) :
                                                                    PxQuat(PxIdentity);

                                    const PxMat44d instanceMatrix =
                                        makeMatrix(PxTransform(instancePos, instanceOrient));

                                    // Gf order was localProtoPrimMatrix * instanceMatrix * instancerMatrix;
                                    // Gf A * B is PhysX B * A, so the chain is written in reverse here.
                                    const PxMat44d bodyMatrix = instancerMatrix * instanceMatrix * localProtoPrimMatrix;


                                    PxTransform globalPose = actor->getGlobalPose();

                                    const float tolerance = 1e-3f;
                                    bool updateTr = false;
                                    if (positionUpdate)
                                    {
                                        const PxVec3d p = bodyMatrix.getPosition();
                                        const PxVec3 newPos(float(p.x), float(p.y), float(p.z));
                                        if ((fabsf(newPos.x - globalPose.p.x) > tolerance) ||
                                            (fabsf(newPos.y - globalPose.p.y) > tolerance) || (fabsf(newPos.z - globalPose.p.z) > tolerance))
                                        {
                                            globalPose.p = newPos;
                                            updateTr = true;
                                        }
                                    }
                                    else
                                    {
                                        // Bit-exact GfTransform::GetRotation() (gfmath::decomposeWithPivot), not
                                        // the PhysX-native polar toTransform: bodyMatrix composes an instancer
                                        // world transform with a prototype's local transform, so ancestor
                                        // non-uniform scale plus rotation can shear it, and the two
                                        // decompositions disagree on that input (see MatrixTools.h).
                                        const gfmath::PivotTransform bodyXf = gfmath::decomposeWithPivot(bodyMatrix);
                                        const PxQuatd bodyRotD = gfmath::getQuat(bodyXf.rotation);
                                        const PxQuat newRot(
                                            float(bodyRotD.x), float(bodyRotD.y), float(bodyRotD.z), float(bodyRotD.w));

                                        const float dot = newRot.dot(globalPose.q);
                                        if (abs(dot) < (1.0f - tolerance))
                                        {
                                            globalPose.q = newRot;
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

                                    if (!topBodyResolved)
                                    {
                                        PxRigidActor* actor = shape->getActor();
                                        if (actor)
                                        {
                                            objectRecord = db.getFullRecord(internalType, size_t(actor->userData));
                                            if (objectRecord && internalType == ePTActor)
                                            {
                                                const PxMat44d topBodyPrimMatrix =
                                                    getWorldTransform(attachedStage, instancerKey, omni::physics::parse::ReadTime::defaultTime());
                                                const PxMat44d topBodyPrimMatrixInv = affineInverse(topBodyPrimMatrix);

                                                // Gf order was instancerMatrix * topBodyPrimMatrixInv.
                                                topBodyMatrix = topBodyPrimMatrixInv * instancerMatrix;
                                                // Bit-exact GfTransform::GetScale() (gfmath::decomposeWithPivot),
                                                // not the PhysX-native polar getScale: topBodyMatrix composes an
                                                // instancer world transform with a prototype's local transform,
                                                // so ancestor non-uniform scale plus rotation can shear it, and
                                                // the two decompositions disagree on that input (see
                                                // MatrixTools.h). Feeds the shape local position below.
                                                const PxVec3d topBodyScaleD = gfmath::decomposeWithPivot(topBodyMatrix).scale;
                                                topBodyScale = PxVec3(
                                                    float(topBodyScaleD.x), float(topBodyScaleD.y), float(topBodyScaleD.z));
                                                topBodyResolved = true;
                                            }
                                        }
                                    }

                                    const PxVec3 instancePos = internalShape->mInstanceIndex < positions.size() ? toPhysX(positions[internalShape->mInstanceIndex]) : PxVec3(0.0f);
                                    const PxQuat instanceOrient = internalShape->mInstanceIndex < orientations.size() ? toPhysXQuat(orientations[internalShape->mInstanceIndex]) : PxQuat(PxIdentity);

                                    const PxMat44d instanceMatrix =
                                        makeMatrix(PxTransform(instancePos, instanceOrient));

                                    // Gf order was localProtoPrimMatrix * instanceMatrix * topBodyMatrix.
                                    const PxMat44d shapeMatrix = topBodyMatrix * instanceMatrix * localProtoPrimMatrix;

                                    const PxVec3d sp = shapeMatrix.getPosition();
                                    const PxVec3 localPos =
                                        PxVec3(float(sp.x), float(sp.y), float(sp.z)).multiply(topBodyScale);

                                    PxTransform localPose = shape->getLocalPose();

                                    const float tolerance = 1e-3f;
                                    bool updateTr = false;
                                    if (positionUpdate)
                                    {
                                        if ((fabsf(localPos.x - localPose.p.x) > tolerance) ||
                                            (fabsf(localPos.y - localPose.p.y) > tolerance) || (fabsf(localPos.z - localPose.p.z) > tolerance))
                                        {
                                            localPose.p = localPos;
                                            updateTr = true;
                                        }
                                    }
                                    else
                                    {
                                        // Bit-exact GfTransform::GetRotation() (gfmath::decomposeWithPivot), not
                                        // the PhysX-native polar toTransform: shapeMatrix composes an instancer
                                        // world transform with a prototype's local transform, so ancestor
                                        // non-uniform scale plus rotation can shear it, and the two
                                        // decompositions disagree on that input (see MatrixTools.h).
                                        const gfmath::PivotTransform shapeXf = gfmath::decomposeWithPivot(shapeMatrix);
                                        const PxQuatd shapeRotD = gfmath::getQuat(shapeXf.rotation);
                                        const PxQuat newRot(
                                            float(shapeRotD.x), float(shapeRotD.y), float(shapeRotD.z), float(shapeRotD.w));

                                        const float dot = newRot.dot(localPose.q);
                                        if (abs(dot) < (1.0f - tolerance))
                                        {
                                            localPose.q = newRot;
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
    return true;
}

bool omni::physx::updateBodyInstancedPositions(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    return updateBodyInstancedTransform(attachedStage, objectId, property, timeCode, true);
}

bool omni::physx::updateBodyInstancedOrientations(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    return updateBodyInstancedTransform(attachedStage, objectId, property, timeCode, false);
}

bool updateBodyInstancedVelocitiesInternal(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode, bool linearVelocity)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTPointInstancer)
    {
        const omni::physics::parse::ObjectKey instancerKey = objectRecord->mKey;
        const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
        omni::physics::parse::KnownTokens tok;
        if (src)
            tok.intern(*src);

        if (src && src->isA(instancerKey, tok.pointInstancerType))
        {
            std::vector<carb::Float3> velocities;
            if (linearVelocity)
                getArrayValue(attachedStage, instancerKey, tok.velocities, omni::physics::parse::ReadTime::defaultTime(), velocities);
            else
                getArrayValue(attachedStage, instancerKey, tok.angularVelocities, omni::physics::parse::ReadTime::defaultTime(), velocities);

            std::vector<omni::physics::parse::ObjectKey> targets;
            getRelationshipValue(attachedStage, instancerKey, tok.prototypes, targets);

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
                                            // degToRad(PxVec3) applies the same float scalar
                                            // as the GfVec3f overload it replaces.
                                            dynamicActor->setAngularVelocity(degToRad(toPhysX(velocities[internalActor->mInstanceIndex])));
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

bool omni::physx::updateBodyInstancedVelocities(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    return updateBodyInstancedVelocitiesInternal(attachedStage, objectId, property, timeCode, true);
}

bool omni::physx::updateBodyInstancedAngularVelocities(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    return updateBodyInstancedVelocitiesInternal(attachedStage, objectId, property, timeCode, false);
}
