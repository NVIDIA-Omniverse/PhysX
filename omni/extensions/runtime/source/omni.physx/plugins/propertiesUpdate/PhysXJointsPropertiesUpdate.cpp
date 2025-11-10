// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXPropertiesUpdate.h"
#include <physicsSchemaTools/physicsSchemaTokens.h>
#include <PhysXTools.h>
#include <Setup.h>
#include <OmniPhysX.h>

#include <PhysXCustomJoint.h>
#include <PhysXUSDProperties.h>

#include <usdLoad/AttachedStage.h>
#include <usdInterface/UsdInterface.h>

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>

OMNI_LOG_DECLARE_CHANNEL(kRoboticsLogChannel)

using namespace ::physx;
using namespace carb;
using namespace pxr;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;


////////////////////////////////////////////////////////////////////////////////////////////////////////
// joint

void wakeJointActors(PxJoint& joint)
{
    PxRigidActor* actor0 = nullptr;
    PxRigidActor* actor1 = nullptr;
    joint.getActors(actor0, actor1);
    if (actor0 && actor0->getType() == PxConcreteType::eRIGID_DYNAMIC)
    {
        ((PxRigidDynamic*)actor0)->wakeUp();
    }
    if (actor1 && actor1->getType() == PxConcreteType::eRIGID_DYNAMIC)
    {
        ((PxRigidDynamic*)actor1)->wakeUp();
    }        
}

void updateJointDriveLinearTarget(PxD6Joint* d6Joint, const PhysxJointDrive& jointDrive, uint32_t axisIndex, bool position)
{
    if (!position)
    {
        const PxVec3 angVel(0.0f, 0.0f, 0.0f);
        PxVec3 driveVel(0.0f, 0.0f, 0.0f);
        driveVel[axisIndex] = jointDrive.targetVelocity;
        d6Joint->setDriveVelocity(driveVel, angVel);
    }
    else
    {
        PxVec3 drivePos(0.0f, 0.0f, 0.0f);
        drivePos[axisIndex] = jointDrive.targetPosition;
        d6Joint->setDrivePosition(PxTransform(drivePos));
    }
}

void updateJointDriveAngularTarget(PxD6Joint* d6Joint, const PhysxJointDrive& jointDrive, uint32_t axisIndex, bool position)
{
    if (!position)
    {
        PxVec3 angVel(0.0f, 0.0f, 0.0f);
        const PxVec3 driveVel(0.0f, 0.0f, 0.0f);
        angVel[axisIndex] = jointDrive.targetVelocity;
        d6Joint->setDriveVelocity(driveVel, angVel);
    }
    else
    {
        const PxVec3 drivePos(0.0f, 0.0f, 0.0f);
        PxVec3 rotAxis(0.0f, 0.0f, 0.0f);
        rotAxis[axisIndex] = 1.0f;
        PxQuat rot(jointDrive.targetPosition, rotAxis);
        d6Joint->setDrivePosition(PxTransform(drivePos, rot));
    }
}

void updateArticulationJointDrive(const omni::physx::usdparser::PhysxJointDrive& drive, PxArticulationJointReducedCoordinate* joint, ::physx::PxArticulationAxis::Enum axis)
{
    PxArticulationReducedCoordinate& articulation = joint->getChildArticulationLink().getArticulation();
    if (articulation.getScene())
        articulation.wakeUp();
    joint->setDriveParams(axis, toPhysX(drive));
}

void updateJointDriveTarget(bool updatePos, PxD6Joint* d6Joint, const PhysxJointDrive* jointDrives)
{
    if (updatePos)
    {
        const PxVec3 posDrive(jointDrives[0].targetPosition, jointDrives[1].targetPosition, jointDrives[2].targetPosition);
        const PxQuat qx(jointDrives[3].targetPosition, PxVec3(1.0f, 0.0f, 0.0f));
        const PxQuat qy(jointDrives[4].targetPosition, PxVec3(0.0f, 1.0f, 0.0f));
        const PxQuat qz(jointDrives[5].targetPosition, PxVec3(0.0f, 0.0f, 1.0f));
        const PxQuat rot = qz * qy * qx;

        d6Joint->setDrivePosition(PxTransform(posDrive, rot));
    }
    else
    {
        const PxVec3 linVelDrive(jointDrives[0].targetVelocity, jointDrives[1].targetVelocity, jointDrives[2].targetVelocity);
        const PxVec3 angVelDrive(jointDrives[3].targetVelocity, jointDrives[4].targetVelocity, jointDrives[5].targetVelocity);
        d6Joint->setDriveVelocity(linVelDrive, angVelDrive);
    }
}

bool getD6ArticulationAxisFromLimit(const std::string& nameString, PxArticulationAxis::Enum& axis)
{
    // limit properties will have names like limit:rotY:physics:high
    if (nameString.find("limit:rotX") != std::string::npos)
    {
        axis = PxArticulationAxis::eTWIST;
        return true;
    }
    else if (nameString.find("limit:rotY") != std::string::npos)
    {
        axis = PxArticulationAxis::eSWING1;
        return true;
    }
    else if (nameString.find("limit:rotZ") != std::string::npos)
    {
        axis = PxArticulationAxis::eSWING2;
        return true;
    }
    return false;
}

bool getD6ArticulationAxisFromDrive(const std::string& nameString, PxArticulationAxis::Enum& axis)
{
    // drive properties will have names like drive:rotY:physics:targetPosition
    if (nameString.find("drive:rotX") != std::string::npos)
    {
        axis = PxArticulationAxis::eTWIST;
        return true;
    }
    else if (nameString.find("drive:rotY") != std::string::npos)
    {
        axis = PxArticulationAxis::eSWING1;
        return true;
    }
    else if (nameString.find("drive:rotZ") != std::string::npos)
    {
        axis = PxArticulationAxis::eSWING2;
        return true;
    }
    return false;
}

bool mapAxisToToken(PxArticulationAxis::Enum& axis, TfToken& token) 
{
    if (axis == PxArticulationAxis::eTWIST) {
        token = UsdPhysicsTokens->rotX;
        return true;
    } else if (axis == PxArticulationAxis::eSWING1) {
        token = UsdPhysicsTokens->rotY;
        return true;
    } else if (axis == PxArticulationAxis::eSWING2) {
        token = UsdPhysicsTokens->rotZ;
        return true;
    }
        return false;
}

bool getD6ArticulationAxisFromEnvelope(const std::string& nameString, PxArticulationAxis::Enum& axis)
{
    if (nameString.find("physxDrivePerformanceEnvelope:rotX") != std::string::npos)
    {
        axis = PxArticulationAxis::eTWIST;
        return true;
    }
    else if (nameString.find("physxDrivePerformanceEnvelope:rotY") != std::string::npos)
    {
        axis = PxArticulationAxis::eSWING1;
        return true;
    }
    else if (nameString.find("physxDrivePerformanceEnvelope:rotZ") != std::string::npos)
    {
        axis = PxArticulationAxis::eSWING2;
        return true;
    }
    return false;
}

bool getD6ArticulationAxisFromProperties(const std::string& nameString, PxArticulationAxis::Enum& axis)
{
    if (nameString.find("physxJointAxis:rotX") != std::string::npos)
    {
        axis = PxArticulationAxis::eTWIST;
        return true;
    }
    else if (nameString.find("physxJointAxis:rotY") != std::string::npos)
    {
        axis = PxArticulationAxis::eSWING1;
        return true;
    }
    else if (nameString.find("physxJointAxis:rotZ") != std::string::npos)
    {
        axis = PxArticulationAxis::eSWING2;
        return true;
    }
    return false;
}
bool getD6ArticulationAxisFromJointState(const std::string& nameString, PxArticulationAxis::Enum& axis)
{
    // state properties will have names like state:rotY:physics:position
    if (nameString.find("state:rotX") != std::string::npos)
    {
        axis = PxArticulationAxis::eTWIST;
        return true;
    }
    else if (nameString.find("state:rotY") != std::string::npos)
    {
        axis = PxArticulationAxis::eSWING1;
        return true;
    }
    else if (nameString.find("state:rotZ") != std::string::npos)
    {
        axis = PxArticulationAxis::eSWING2;
        return true;
    }
    return false;
}


void getAxisAndDrive(const std::string& nameString, int& index, PxD6Drive::Enum& drive)
{
    if (nameString.find("drive:transY") != std::string::npos)
    {
        index = 1;
        drive = PxD6Drive::eY;
    }
    else if (nameString.find("drive:transZ") != std::string::npos)
    {
        index = 2;
        drive = PxD6Drive::eZ;
    }
    else if (nameString.find("drive:rotX") != std::string::npos)
    {
        index = 3;
        drive = PxD6Drive::eTWIST;
    }
    else if (nameString.find("drive:rotY") != std::string::npos)
    {
        index = 4;
        drive = PxD6Drive::eSWING1;
    }
    else if (nameString.find("drive:rotZ") != std::string::npos)
    {
        index = 5;
        drive = PxD6Drive::eSWING2;
    }
}

bool updateDriveTargetInternal(const AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode, bool position)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTLinkJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        if (joint && intJoint)
        {
            PxArticulationReducedCoordinate& articulation = joint->getChildArticulationLink().getArticulation();
            if (articulation.getScene())
                articulation.wakeUp();

            switch (joint->getJointType())
            {
            case PxArticulationJointType::eREVOLUTE:
            case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
            {
                data = degToRad(data);
                if (position)
                {
                    intJoint->setArticulationDrivePositionTarget(joint, ::physx::PxArticulationAxis::eTWIST, data, objectRecord->mPath);
                }
                else
                {
                    intJoint->setArticulationDriveVelocityTarget(joint, ::physx::PxArticulationAxis::eTWIST, data);
                }
            }
            break;
            case PxArticulationJointType::ePRISMATIC:
            {
                if (position)
                {
                    intJoint->setArticulationDrivePositionTarget(joint, ::physx::PxArticulationAxis::eX, data);
                }
                else
                {
                    intJoint->setArticulationDriveVelocityTarget(joint, ::physx::PxArticulationAxis::eX, data);
                }
            }
            break;
            case PxArticulationJointType::eFIX:
            case PxArticulationJointType::eSPHERICAL:
            {
                if (intJoint->mJointType == usdparser::eJointD6)
                {
                    PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
                    const bool validAxis = getD6ArticulationAxisFromDrive(property.GetString(), axis);
                    if (validAxis)
                    {
                        data = degToRad(data);
                        if (position)
                        {
                            intJoint->setArticulationDrivePositionTarget(joint, axis, data);
                        }
                        else
                        {
                            intJoint->setArticulationDriveVelocityTarget(joint, axis, data);
                        }
                    }
                }
            }
            break;
            case PxArticulationJointType::eUNDEFINED:
                break;
            }
        }
    }
    else if (internalType == ePTJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxJoint* joint = (PxJoint*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        if (joint)
        {
            switch (intJoint->mJointType)
            {
            case eJointRevolute:
            {
                if (position)
                {
                    intJoint->mJointDrive.targetPosition = degToRad(data);
                }
                else
                {
                    // preist: Switch the sign to be consistent with articulations and position drive behavior. Remove when changed in the SDK
                    // OM-42441
                    intJoint->mJointDrive.targetVelocity = -degToRad(data);
                }

                PxD6Joint* d6Joint = (PxD6Joint*)joint;
                updateJointDriveAngularTarget(d6Joint, intJoint->mJointDrive, intJoint->mAxisIndex, position);
            }
            break;
            case eJointPrismatic:
            {
                if (position)
                    intJoint->mJointDrive.targetPosition = data;
                else
                    intJoint->mJointDrive.targetVelocity = data;

                PxD6Joint* d6Joint = (PxD6Joint*)joint;
                updateJointDriveLinearTarget(d6Joint, intJoint->mJointDrive, intJoint->mAxisIndex, position);
            }
            break;
            case eJointD6:
            {
                PxD6Joint* d6Joint = (PxD6Joint*)joint;
                const std::string& nameString = property.GetString();
                bool updatePos = false;
                int index = 0;
                PxD6Drive::Enum drive = PxD6Drive::eX;
                getAxisAndDrive(nameString, index, drive);
                if (index > 2)
                    data = degToRad(data);
                if (position)
                {
                    updatePos = true;
                    intJoint->mJointDrives[index].targetPosition = data;
                }
                else
                {
                    // preist: Switch the sign to be consistent with articulations and position drive behavior. Remove when changed in the SDK
                    // OM-42441
                    intJoint->mJointDrives[index].targetVelocity = -data;
                }
                updateJointDriveTarget(updatePos, d6Joint, &intJoint->mJointDrives[0]);
            }
            break;
            default:
                break;
            }
        }
    }

    return true;
}

bool omni::physx::updateDriveTargetPosition(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attachedStage.getPhysXPhysicsInterface()->finishSetup(attachedStage);
    return updateDriveTargetInternal(attachedStage, objectId, property, timeCode, true);
}

bool omni::physx::updateDriveTargetVelocity(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attachedStage.getPhysXPhysicsInterface()->finishSetup(attachedStage);
    return updateDriveTargetInternal(attachedStage, objectId, property, timeCode, false);
}

bool omni::physx::updateDriveMaxForce(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    attachedStage.getPhysXPhysicsInterface()->finishSetup(attachedStage);

    if (internalType == ePTLinkJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        const UsdPrim prim =  attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath);
        if (joint && intJoint)
        {
            switch (joint->getJointType())
            {
            case PxArticulationJointType::eREVOLUTE:
            case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
            {
                if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, UsdPhysicsTokens->angular))
                    intJoint->mJointDrive.isEnvelopeUsed = true;
                intJoint->mJointDrive.forceLimit = isfinite(data) ? data : FLT_MAX;
                updateArticulationJointDrive(intJoint->mJointDrive, joint, ::physx::PxArticulationAxis::eTWIST);
            }
            break;
            case PxArticulationJointType::ePRISMATIC:
            {
                if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, UsdPhysicsTokens->linear))
                    intJoint->mJointDrive.isEnvelopeUsed = true;
                intJoint->mJointDrive.forceLimit = isfinite(data) ? data : FLT_MAX;
                updateArticulationJointDrive(intJoint->mJointDrive, joint, ::physx::PxArticulationAxis::eX);
            }
            break;
            case PxArticulationJointType::eSPHERICAL:
            {
                if (intJoint->mJointType == usdparser::eJointD6)
                {
                    PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
                    const bool validAxis = getD6ArticulationAxisFromDrive(property.GetString(), axis);
                    TfToken token = UsdPhysicsTokens->rotX;
                    if (validAxis)
                    {
                        mapAxisToToken(axis, token);
                        if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, token))
                            intJoint->mJointDrive.isEnvelopeUsed = true;
                        intJoint->mJointDrives[axis].forceLimit = isfinite(data) ? data : FLT_MAX;
                        updateArticulationJointDrive(intJoint->mJointDrives[axis], joint, axis);
                    }
                }
            }
            }
        }
    }
    else if (internalType == ePTJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxJoint* joint = (PxJoint*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        intJoint->mJointDrive.forceLimit = isfinite(data) ? data : FLT_MAX;
        if (joint)
        {
            switch (intJoint->mJointType)
            {
            case eJointRevolute:
            {
                PxD6Joint* d6Joint = (PxD6Joint*)joint;

                PxD6JointDrive drive(intJoint->mJointDrive.stiffness, intJoint->mJointDrive.damping,
                    intJoint->mJointDrive.forceLimit, intJoint->mJointDrive.acceleration);
                d6Joint->setDrive(intJoint->mD6JointDrive, drive);
                updateJointDriveAngularTarget(d6Joint, intJoint->mJointDrive, intJoint->mAxisIndex, true);
                updateJointDriveAngularTarget(d6Joint, intJoint->mJointDrive, intJoint->mAxisIndex, false);
            }
            break;
            case eJointPrismatic:
            {
                PxD6Joint* d6Joint = (PxD6Joint*)joint;

                PxD6JointDrive drive(intJoint->mJointDrive.stiffness, intJoint->mJointDrive.damping,
                    intJoint->mJointDrive.forceLimit, intJoint->mJointDrive.acceleration);
                d6Joint->setDrive(intJoint->mD6JointDrive, drive);
                updateJointDriveLinearTarget(d6Joint, intJoint->mJointDrive, intJoint->mAxisIndex, true);
                updateJointDriveLinearTarget(d6Joint, intJoint->mJointDrive, intJoint->mAxisIndex, false);
            }
            break;
            case eJointD6:
            {
                PxD6Joint* d6Joint = (PxD6Joint*)joint;
                const std::string& nameString = property.GetString();
                int index = 0;
                PxD6Drive::Enum drive = PxD6Drive::eX;
                getAxisAndDrive(nameString, index, drive);
                intJoint->mJointDrives[index].forceLimit = data;
                PxD6JointDrive jointDrive(intJoint->mJointDrives[index].stiffness, intJoint->mJointDrives[index].damping,
                    intJoint->mJointDrives[index].forceLimit,
                    intJoint->mJointDrives[index].acceleration);
                d6Joint->setDrive(drive, jointDrive);
                updateJointDriveTarget(true, d6Joint, &intJoint->mJointDrives[0]);
                updateJointDriveTarget(false, d6Joint, &intJoint->mJointDrives[0]);
            }
            break;
            default:
                break;
            }
        }
    }

    return true;
}

bool omni::physx::updateDriveMaxActuatorVelocity(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    attachedStage.getPhysXPhysicsInterface()->finishSetup(attachedStage);

    if (internalType == ePTLinkJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;
        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        const UsdPrim prim =  attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath);
        if (joint && intJoint)
        {
            switch (joint->getJointType())
            {
            case PxArticulationJointType::eREVOLUTE:
            case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
            {
                if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, UsdPhysicsTokens->angular))
                {       
                    intJoint->mJointDrive.isEnvelopeUsed = true;
                    intJoint->mJointDrive.maxActuatorVelocity = isfinite(data) ? degToRad(data) : FLT_MAX;
                    updateArticulationJointDrive(intJoint->mJointDrive, joint, ::physx::PxArticulationAxis::eTWIST);
                }
            }
            break;
            case PxArticulationJointType::ePRISMATIC:
            {
                if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, UsdPhysicsTokens->linear))
                {       
                    intJoint->mJointDrive.isEnvelopeUsed = true;
                    intJoint->mJointDrive.maxActuatorVelocity = isfinite(data) ? data : FLT_MAX;
                    updateArticulationJointDrive(intJoint->mJointDrive, joint, ::physx::PxArticulationAxis::eX);
                }
            }
            break;
            case PxArticulationJointType::eSPHERICAL:
            {
                if (intJoint->mJointType == usdparser::eJointD6)
                {
                    PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
                    const bool validAxis = getD6ArticulationAxisFromEnvelope(property.GetString(), axis);
                    TfToken token = UsdPhysicsTokens->rotX;
                    if (validAxis)
                    {
                        mapAxisToToken(axis, token);
                        if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, token))
                        {
                            intJoint->mJointDrives[axis].isEnvelopeUsed = true;
                            intJoint->mJointDrives[axis].maxActuatorVelocity = isfinite(data) ? degToRad(data) : FLT_MAX;
                            updateArticulationJointDrive(intJoint->mJointDrives[axis], joint, axis);
                        }
                    }
                }
            }
            }
        }
    }
    return true;
}

bool omni::physx::updateDriveVelocityDependentResistance(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    attachedStage.getPhysXPhysicsInterface()->finishSetup(attachedStage);

    if (internalType == ePTLinkJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        const UsdPrim prim =  attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath);
        if (joint && intJoint)
        {
            switch (joint->getJointType())
            {
            case PxArticulationJointType::eREVOLUTE:
            case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
            {
                if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, UsdPhysicsTokens->angular))
                {       
                    intJoint->mJointDrive.isEnvelopeUsed = true;
                    intJoint->mJointDrive.velocityDependentResistance = isfinite(data) ? radToDeg(data) : FLT_MAX; //torque * second / degrees                       
                    updateArticulationJointDrive(intJoint->mJointDrive, joint, ::physx::PxArticulationAxis::eTWIST);
                }
            }
            break;
            case PxArticulationJointType::ePRISMATIC:
            {
                if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, UsdPhysicsTokens->linear))
                {       
                    intJoint->mJointDrive.isEnvelopeUsed = true;
                    intJoint->mJointDrive.velocityDependentResistance = isfinite(data) ? data : FLT_MAX;
                    updateArticulationJointDrive(intJoint->mJointDrive, joint, ::physx::PxArticulationAxis::eX);
                }
            }
            break;
            case PxArticulationJointType::eSPHERICAL:
            {
                if (intJoint->mJointType == usdparser::eJointD6)
                {
                    PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
                    const bool validAxis = getD6ArticulationAxisFromEnvelope(property.GetString(), axis);
                    TfToken token = UsdPhysicsTokens->rotX;
                    if (validAxis)
                    {
                        mapAxisToToken(axis, token);
                        if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, token))
                        {
                            intJoint->mJointDrives[axis].isEnvelopeUsed = true;
                            intJoint->mJointDrives[axis].velocityDependentResistance = isfinite(data) ? radToDeg(data) : FLT_MAX; //torque * second / degrees
                            updateArticulationJointDrive(intJoint->mJointDrives[axis], joint, axis);
                        }
                    }
                }
            }
            }
        }
    }
    return true;
}

bool omni::physx::updateDriveSpeedEffortGradient(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    attachedStage.getPhysXPhysicsInterface()->finishSetup(attachedStage);

    if (internalType == ePTLinkJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        const UsdPrim prim =  attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath);
        if (joint && intJoint)
        {
            switch (joint->getJointType())
            {
            case PxArticulationJointType::eREVOLUTE:
            case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
            {
                if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, UsdPhysicsTokens->angular))
                {       
                    intJoint->mJointDrive.isEnvelopeUsed = true;
                    intJoint->mJointDrive.speedEffortGradient = isfinite(data) ? degToRad(data) : FLT_MAX;
                    updateArticulationJointDrive(intJoint->mJointDrive, joint, ::physx::PxArticulationAxis::eTWIST);
                }
            }
            break;
            case PxArticulationJointType::ePRISMATIC:
            {
                if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, UsdPhysicsTokens->linear))
                {       
                    intJoint->mJointDrive.isEnvelopeUsed = true;
                    intJoint->mJointDrive.speedEffortGradient = isfinite(data) ? data : FLT_MAX;
                    updateArticulationJointDrive(intJoint->mJointDrive, joint, ::physx::PxArticulationAxis::eX);
                }
            }
            break;
            case PxArticulationJointType::eSPHERICAL:
            {
                if (intJoint->mJointType == usdparser::eJointD6)
                {
                    PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
                    const bool validAxis = getD6ArticulationAxisFromEnvelope(property.GetString(), axis);
                    TfToken token = UsdPhysicsTokens->rotX;
                    if (validAxis)
                    {
                        mapAxisToToken(axis, token);
                        if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, token))
                        {
                            intJoint->mJointDrives[axis].isEnvelopeUsed = true;
                            intJoint->mJointDrives[axis].speedEffortGradient = isfinite(data) ? degToRad(data) : FLT_MAX;
                            updateArticulationJointDrive(intJoint->mJointDrives[axis], joint, axis);
                        }
                    }
                }
            }
            }
        }
    }
    return true;
}

bool omni::physx::updateDriveDamping(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    attachedStage.getPhysXPhysicsInterface()->finishSetup(attachedStage);

    if (internalType == ePTLinkJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        if (joint && intJoint)
        {
            switch (joint->getJointType())
            {
            case PxArticulationJointType::eREVOLUTE:
            case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
            {
                // for rot dof stiffness and damping are in 1/deg in usd,
                //  so converting via radToDeg will convert correctly, i.e. 1 / (pi / 180) = 180 / pi
                intJoint->mJointDrive.damping = isfinite(data) ? radToDeg(data) : FLT_MAX;
                updateArticulationJointDrive(intJoint->mJointDrive, joint, ::physx::PxArticulationAxis::eTWIST);
            }
            break;
            case PxArticulationJointType::ePRISMATIC:
            {
                intJoint->mJointDrive.damping = isfinite(data) ? data : FLT_MAX;
                updateArticulationJointDrive(intJoint->mJointDrive, joint, ::physx::PxArticulationAxis::eX);
            }
            break;
            case PxArticulationJointType::eSPHERICAL:
            {
                if (intJoint->mJointType == usdparser::eJointD6)
                {
                    PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
                    const bool validAxis = getD6ArticulationAxisFromDrive(property.GetString(), axis);
                    if (validAxis)
                    {
                        // for rot dof stiffness and damping are in 1/deg in usd,
                        //  so converting via radToDeg will convert correctly, i.e. 1 / (pi / 180) = 180 / pi
                        intJoint->mJointDrives[axis].damping = isfinite(data) ? radToDeg(data) : FLT_MAX;
                        updateArticulationJointDrive(intJoint->mJointDrives[axis], joint, axis);
                    }
                }
            }
            }
        }
    }
    else if (internalType == ePTJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxJoint* joint = (PxJoint*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        intJoint->mJointDrive.damping = data;
        if (joint)
        {
            switch (intJoint->mJointType)
            {
            case eJointRevolute:
            {
                PxD6Joint* d6Joint = (PxD6Joint*)joint;
                // for rot dof stiffness and damping are in 1/deg in usd,
                //  so converting via radToDeg will convert correctly, i.e. 1 / (pi / 180) = 180 / pi
                intJoint->mJointDrive.damping = radToDeg(intJoint->mJointDrive.damping);
                PxD6JointDrive drive(intJoint->mJointDrive.stiffness, intJoint->mJointDrive.damping,
                    intJoint->mJointDrive.forceLimit, intJoint->mJointDrive.acceleration);
                d6Joint->setDrive(intJoint->mD6JointDrive, drive);
                updateJointDriveAngularTarget(d6Joint, intJoint->mJointDrive, intJoint->mAxisIndex, true);
                updateJointDriveAngularTarget(d6Joint, intJoint->mJointDrive, intJoint->mAxisIndex, false);
            }
            break;
            case eJointPrismatic:
            {
                PxD6Joint* d6Joint = (PxD6Joint*)joint;

                PxD6JointDrive drive(intJoint->mJointDrive.stiffness, intJoint->mJointDrive.damping,
                    intJoint->mJointDrive.forceLimit, intJoint->mJointDrive.acceleration);
                d6Joint->setDrive(intJoint->mD6JointDrive, drive);
                updateJointDriveLinearTarget(d6Joint, intJoint->mJointDrive, intJoint->mAxisIndex, true);
                updateJointDriveLinearTarget(d6Joint, intJoint->mJointDrive, intJoint->mAxisIndex, false);
            }
            break;
            case eJointD6:
            {
                PxD6Joint* d6Joint = (PxD6Joint*)joint;
                const std::string& nameString = property.GetString();
                int index = 0;
                PxD6Drive::Enum drive = PxD6Drive::eX;
                getAxisAndDrive(nameString, index, drive);
                // for rot dof (index >= 3) stiffness and damping are in 1/deg in usd,
                //  so converting via radToDeg will convert correctly, i.e. 1 / (pi / 180) = 180 / pi
                intJoint->mJointDrives[index].damping = index < 3 ? data : radToDeg(data);

                PxD6JointDrive jointDrive(intJoint->mJointDrives[index].stiffness, intJoint->mJointDrives[index].damping,
                    intJoint->mJointDrives[index].forceLimit,
                    intJoint->mJointDrives[index].acceleration);
                d6Joint->setDrive(drive, jointDrive);
                updateJointDriveTarget(true, d6Joint, &intJoint->mJointDrives[0]);
                updateJointDriveTarget(false, d6Joint, &intJoint->mJointDrives[0]);
            }
            break;
            default:
                break;
            }
        }
    }
    return true;
}

bool omni::physx::updateDriveStiffness(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    attachedStage.getPhysXPhysicsInterface()->finishSetup(attachedStage);

    if (internalType == ePTLinkJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        if (joint && intJoint)
        {
            switch (joint->getJointType())
            {
            case PxArticulationJointType::eREVOLUTE:
            case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
            {
                intJoint->mJointDrive.stiffness = isfinite(data) ? radToDeg(data) : FLT_MAX;
                updateArticulationJointDrive(intJoint->mJointDrive, joint, ::physx::PxArticulationAxis::eTWIST);
            }
            break;
            case PxArticulationJointType::ePRISMATIC:
            {
                intJoint->mJointDrive.stiffness = isfinite(data) ? data : FLT_MAX;
                updateArticulationJointDrive(intJoint->mJointDrive, joint, ::physx::PxArticulationAxis::eX);
            }
            break;
            case PxArticulationJointType::eSPHERICAL:
            {
                if (intJoint->mJointType == usdparser::eJointD6)
                {
                    PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
                    const bool validAxis = getD6ArticulationAxisFromDrive(property.GetString(), axis);
                    if (validAxis)
                    {
                        intJoint->mJointDrives[axis].stiffness = isfinite(data) ? radToDeg(data) : FLT_MAX;
                        updateArticulationJointDrive(intJoint->mJointDrives[axis], joint, axis);
                    }
                }
            }
            }
        }
    }
    else if (internalType == ePTJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxJoint* joint = (PxJoint*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        intJoint->mJointDrive.stiffness = data;
        if (joint)
        {
            switch (intJoint->mJointType)
            {
            case eJointRevolute:
            {
                PxD6Joint* d6Joint = (PxD6Joint*)joint;
                intJoint->mJointDrive.stiffness = radToDeg(intJoint->mJointDrive.stiffness);
                PxD6JointDrive drive(intJoint->mJointDrive.stiffness, intJoint->mJointDrive.damping,
                    intJoint->mJointDrive.forceLimit, intJoint->mJointDrive.acceleration);
                d6Joint->setDrive(intJoint->mD6JointDrive, drive);
                updateJointDriveAngularTarget(d6Joint, intJoint->mJointDrive, intJoint->mAxisIndex, true);
                updateJointDriveAngularTarget(d6Joint, intJoint->mJointDrive, intJoint->mAxisIndex, false);
            }
            break;
            case eJointPrismatic:
            {
                PxD6Joint* d6Joint = (PxD6Joint*)joint;

                PxD6JointDrive drive(intJoint->mJointDrive.stiffness, intJoint->mJointDrive.damping,
                    intJoint->mJointDrive.forceLimit, intJoint->mJointDrive.acceleration);
                d6Joint->setDrive(intJoint->mD6JointDrive, drive);
                updateJointDriveLinearTarget(d6Joint, intJoint->mJointDrive, intJoint->mAxisIndex, true);
                updateJointDriveLinearTarget(d6Joint, intJoint->mJointDrive, intJoint->mAxisIndex, false);
            }
            break;
            case eJointD6:
            {
                PxD6Joint* d6Joint = (PxD6Joint*)joint;
                const std::string& nameString = property.GetString();
                int index = 0;
                PxD6Drive::Enum drive = PxD6Drive::eX;
                getAxisAndDrive(nameString, index, drive);
                // for rot dof (index >= 3) stiffness and damping are in 1/deg in usd,
                //  so converting via radToDeg will convert correctly, i.e. 1 / (pi / 180) = 180 / pi
                intJoint->mJointDrives[index].stiffness = index < 3 ? data : radToDeg(data);
                PxD6JointDrive jointDrive(intJoint->mJointDrives[index].stiffness, intJoint->mJointDrives[index].damping,
                    intJoint->mJointDrives[index].forceLimit,
                    intJoint->mJointDrives[index].acceleration);
                d6Joint->setDrive(drive, jointDrive);
                updateJointDriveTarget(true, d6Joint, &intJoint->mJointDrives[0]);
                updateJointDriveTarget(false, d6Joint, &intJoint->mJointDrives[0]);
            }
            break;
            default:
                break;
            }
        }
    }

    return true;
}

bool omni::physx::updateDriveType(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    attachedStage.getPhysXPhysicsInterface()->finishSetup(attachedStage);

    if (internalType == ePTLinkJoint)
    {
        TfToken data;
        if (!getValue<TfToken>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        if (joint && intJoint)
        {
            switch (joint->getJointType())
            {
            case PxArticulationJointType::eREVOLUTE:
            case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
            {
                intJoint->mJointDrive.acceleration = (data == UsdPhysicsTokens.Get()->acceleration) ? true : false;
                updateArticulationJointDrive(intJoint->mJointDrive, joint, ::physx::PxArticulationAxis::eTWIST);
            }
            break;
            case PxArticulationJointType::ePRISMATIC:
            {
                intJoint->mJointDrive.acceleration = (data == UsdPhysicsTokens.Get()->acceleration) ? true : false;
                updateArticulationJointDrive(intJoint->mJointDrive, joint, ::physx::PxArticulationAxis::eX);
            }
            break;
            case PxArticulationJointType::eSPHERICAL:
            {
                if (intJoint->mJointType == usdparser::eJointD6)
                {
                    PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
                    const bool validAxis = getD6ArticulationAxisFromDrive(property.GetString(), axis);
                    if (validAxis)
                    {
                        intJoint->mJointDrives[axis].acceleration = (data == UsdPhysicsTokens.Get()->acceleration) ? true : false;
                        updateArticulationJointDrive(intJoint->mJointDrives[axis], joint, axis);
                    }
                }
            }
            }
        }
    }
    else if (internalType == ePTJoint)
    {
        TfToken data;
        if (!getValue<TfToken>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxJoint* joint = (PxJoint*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        intJoint->mJointDrive.acceleration = (data == UsdPhysicsTokens.Get()->acceleration) ? true : false;
        if (joint)
        {
            switch (intJoint->mJointType)
            {
            case eJointRevolute:
            {
                PxD6Joint* d6Joint = (PxD6Joint*)joint;

                PxD6JointDrive drive(intJoint->mJointDrive.stiffness, intJoint->mJointDrive.damping,
                    intJoint->mJointDrive.forceLimit, intJoint->mJointDrive.acceleration);
                d6Joint->setDrive(intJoint->mD6JointDrive, drive);
                updateJointDriveAngularTarget(d6Joint, intJoint->mJointDrive, intJoint->mAxisIndex, true);
                updateJointDriveAngularTarget(d6Joint, intJoint->mJointDrive, intJoint->mAxisIndex, false);
            }
            break;
            case eJointPrismatic:
            {
                PxD6Joint* d6Joint = (PxD6Joint*)joint;

                PxD6JointDrive drive(intJoint->mJointDrive.stiffness, intJoint->mJointDrive.damping,
                    intJoint->mJointDrive.forceLimit, intJoint->mJointDrive.acceleration);
                d6Joint->setDrive(intJoint->mD6JointDrive, drive);
                updateJointDriveLinearTarget(d6Joint, intJoint->mJointDrive, intJoint->mAxisIndex, true);
                updateJointDriveLinearTarget(d6Joint, intJoint->mJointDrive, intJoint->mAxisIndex, false);
            }
            break;
            case eJointD6:
            {
                PxD6Joint* d6Joint = (PxD6Joint*)joint;
                const std::string& nameString = property.GetString();
                int index = 0;
                PxD6Drive::Enum drive = PxD6Drive::eX;
                getAxisAndDrive(nameString, index, drive);
                intJoint->mJointDrives[index].acceleration =
                    (data == UsdPhysicsTokens.Get()->acceleration) ? true : false;
                PxD6JointDrive jointDrive(intJoint->mJointDrives[index].stiffness, intJoint->mJointDrives[index].damping,
                    intJoint->mJointDrives[index].forceLimit,
                    intJoint->mJointDrives[index].acceleration);
                d6Joint->setDrive(drive, jointDrive);
                updateJointDriveTarget(true, d6Joint, &intJoint->mJointDrives[0]);
                updateJointDriveTarget(false, d6Joint, &intJoint->mJointDrives[0]);
            }
            break;
            default:
                break;
            }
        }
    }

    return true;
}


bool updateJointStateInternal(const AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode, bool position)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTLinkJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        if (joint && intJoint)
        {
            PxArticulationReducedCoordinate& articulation = joint->getChildArticulationLink().getArticulation();
            if (articulation.getScene())
                articulation.wakeUp();

            switch (joint->getJointType())
            {
            case PxArticulationJointType::eREVOLUTE:
            case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
            {
                data = degToRad(data);
                if (position)
                {
                    intJoint->setArticulationJointPosition(joint, ::physx::PxArticulationAxis::eTWIST, data);
                    if (articulation.getScene())
                        articulation.updateKinematic(PxArticulationKinematicFlag::ePOSITION);
                }
                else
                {
                    intJoint->setArticulationJointVelocity(joint, ::physx::PxArticulationAxis::eTWIST, data);
                    if (articulation.getScene())
                        articulation.updateKinematic(PxArticulationKinematicFlag::eVELOCITY);
                }
            }
            break;
            case PxArticulationJointType::ePRISMATIC:
            {
                if (position)
                {
                    intJoint->setArticulationJointPosition(joint, ::physx::PxArticulationAxis::eX, data);
                    if (articulation.getScene())
                        articulation.updateKinematic(PxArticulationKinematicFlag::ePOSITION);
                }
                else
                {
                    intJoint->setArticulationJointVelocity(joint, ::physx::PxArticulationAxis::eX, data);
                    if (articulation.getScene())
                        articulation.updateKinematic(PxArticulationKinematicFlag::eVELOCITY);
                }
            }
            break;
            case PxArticulationJointType::eFIX:
            case PxArticulationJointType::eSPHERICAL:
            {
                if (intJoint->mJointType == usdparser::eJointD6)
                {
                    PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
                    const bool validAxis = getD6ArticulationAxisFromJointState(property.GetString(), axis);
                    if (validAxis)
                    {
                        data = degToRad(data);
                        if (position)
                        {
                            intJoint->setArticulationJointPosition(joint, axis, data);
                            if (articulation.getScene())
                                articulation.updateKinematic(PxArticulationKinematicFlag::ePOSITION);
                        }
                        else
                        {
                            intJoint->setArticulationJointVelocity(joint, axis, data);
                            if (articulation.getScene())
                                articulation.updateKinematic(PxArticulationKinematicFlag::eVELOCITY);
                        }
                    }
                }
            }
            break;
            case PxArticulationJointType::eUNDEFINED:
                break;
            }
        }
    }

    return true;
}

bool omni::physx::updateJointStatePosition(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attachedStage.getPhysXPhysicsInterface()->finishSetup(attachedStage);
    return updateJointStateInternal(attachedStage, objectId, property, timeCode, true);
}

bool omni::physx::updateJointStateVelocity(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attachedStage.getPhysXPhysicsInterface()->finishSetup(attachedStage);
    return updateJointStateInternal(attachedStage, objectId, property, timeCode, false);
}

bool omni::physx::updateLimitHigh(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    attachedStage.getPhysXPhysicsInterface()->finishSetup(attachedStage);

    if (internalType == ePTLinkJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        if (!isfinite(data))
        {
            OMNI_LOG_WARN(
                kRoboticsLogChannel,
                "Cannot change high limit to %2.f on %s during simulation. This change requires a simulation restart.",
                data, objectRecord->mPath.GetText());
            return true;
        }            

        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        if (joint && intJoint)
        {
            PxArticulationReducedCoordinate& articulation = joint->getChildArticulationLink().getArticulation();
            if (articulation.getScene())
                articulation.wakeUp();

            switch (joint->getJointType())
            {
            case PxArticulationJointType::eREVOLUTE:
            case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
            {
                const float angleInDeg = data;
                data = degToRad(angleInDeg);
                if(joint->getMotion(PxArticulationAxis::eTWIST) == PxArticulationMotion::eLIMITED && isfinite(angleInDeg))
                {
                    intJoint->updateArticulationJointLimitHigh(joint, PxArticulationAxis::eTWIST, data);
                }
                else
                {
                    OMNI_LOG_WARN(
                        kRoboticsLogChannel,
                        "Cannot change high limit to %2.f on %s during simulation. This change requires a simulation restart.",
                        angleInDeg, objectRecord->mPath.GetText());
                }
            }
            break;
            case PxArticulationJointType::ePRISMATIC:
            {
                intJoint->updateArticulationJointLimitHigh(joint, PxArticulationAxis::eX, data);
            }
            break;
            case PxArticulationJointType::eSPHERICAL:
            {
                data = degToRad(data);
                if (intJoint->mJointType == usdparser::eJointD6)
                {
                    PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
                    const bool validAxis = getD6ArticulationAxisFromLimit(property.GetString(), axis);
                    if (validAxis)
                    {
                        intJoint->updateArticulationJointLimitHigh(joint, axis, data);
                    }
                }
                else  // case USD spherical joint - limit high is called on ConeAngle1Limit change on PxArticulationAxis::eSWING2
                {
                    if (data >= 0.0f)
                    {
                        joint->setLimitParams(::physx::PxArticulationAxis::eSWING2, PxArticulationLimit(-data, data));
                    }
                    else
                    {
                        joint->setLimitParams(::physx::PxArticulationAxis::eSWING2, PxArticulationLimit(-FLT_MAX, FLT_MAX));
                    }
                }
            }
            case PxArticulationJointType::eFIX:
            case PxArticulationJointType::eUNDEFINED:
                break;
            }
        }
    }
    else if (internalType == ePTJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        if (!isfinite(data))
            return true;

        PxJoint* joint = (PxJoint*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        if (joint)
        {
            switch (intJoint->mJointType)
            {
            case eJointRevolute:
            {
                PxD6Joint* revJoint = (PxD6Joint*)joint;
                PxJointAngularLimitPair limitPair = revJoint->getTwistLimit();
                data = degToRad(data);
                limitPair.upper = data;
                checkRevoluteJointLimits(limitPair, objectRecord->mPath.GetText());
                revJoint->setTwistLimit(limitPair);
            }
            break;
            case eJointPrismatic:
            {
                PxD6Joint* prismJoint = (PxD6Joint*)joint;
                PxJointLinearLimitPair limitPair = prismJoint->getLinearLimit((PxD6Axis::Enum)intJoint->mAxisIndex);
                limitPair.upper = data;
                prismJoint->setLinearLimit((PxD6Axis::Enum)intJoint->mAxisIndex, limitPair);
            }
            break;
            case eJointSpherical:
            {
                PxD6Joint* d6j = (PxD6Joint*)joint;
                PxJointLimitCone coneLimit = d6j->getSwingLimit();
                data = degToRad(data);
                coneLimit.zAngle = data;
                d6j->setSwingLimit(coneLimit);
            }
            break;
            case eJointDistance:
            {
                PxDistanceJoint* distanceJoint = (PxDistanceJoint*)joint;
                if (data >= 0.0f)
                {
                    distanceJoint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
                    distanceJoint->setMaxDistance(data);
                }
                else
                {
                    distanceJoint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, false);
                }
            }
            break;
            case eJointD6:
            {
                PxD6Joint* d6Joint = (PxD6Joint*)joint;
                const std::string& nameString = property.GetString();
                if (nameString == "limit:transX:physics:high")
                {
                    PxJointLinearLimitPair limitPair = d6Joint->getLinearLimit(PxD6Axis::eX);
                    limitPair.upper = data;
                    d6Joint->setLinearLimit(PxD6Axis::eX, limitPair);
                }
                else if (nameString == "limit:transY:physics:high")
                {
                    PxJointLinearLimitPair limitPair = d6Joint->getLinearLimit(PxD6Axis::eY);
                    limitPair.upper = data;
                    d6Joint->setLinearLimit(PxD6Axis::eY, limitPair);
                }
                else if (nameString == "limit:transZ:physics:high")
                {
                    PxJointLinearLimitPair limitPair = d6Joint->getLinearLimit(PxD6Axis::eZ);
                    limitPair.upper = data;
                    d6Joint->setLinearLimit(PxD6Axis::eZ, limitPair);
                }
                else if (nameString == "limit:distance:physics:high")
                {
                    PxJointLinearLimit limit = d6Joint->getDistanceLimit();
                    limit.value = data;
                    d6Joint->setDistanceLimit(limit);
                }
                else if (nameString == "limit:rotX:physics:high")
                {
                    PxJointAngularLimitPair limitPair = d6Joint->getTwistLimit();
                    data = degToRad(data);
                    limitPair.upper = data;
                    d6Joint->setTwistLimit(limitPair);
                }
                else if (nameString == "limit:rotY:physics:high")
                {
                    PxJointLimitPyramid limitPair = d6Joint->getPyramidSwingLimit();
                    data = degToRad(data);
                    limitPair.yAngleMax = data;
                    d6Joint->setPyramidSwingLimit(limitPair);
                }
                else if (nameString == "limit:rotZ:physics:high")
                {
                    PxJointLimitPyramid limitPair = d6Joint->getPyramidSwingLimit();
                    data = degToRad(data);
                    limitPair.zAngleMax = data;
                    d6Joint->setPyramidSwingLimit(limitPair);
                }
            }
            break;
            default:
                break;
            }
        }
    }
    return true;
}

bool omni::physx::updateLimitLow(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    attachedStage.getPhysXPhysicsInterface()->finishSetup(attachedStage);

    if (internalType == ePTLinkJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        if (!isfinite(data))
        {
            OMNI_LOG_WARN(
                kRoboticsLogChannel,
                "Cannot change low limit to %2.f on %s during simulation. This change requires a simulation restart.",
                data, objectRecord->mPath.GetText());
            return true;
        }

        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        if (joint && intJoint)
        {
            PxArticulationReducedCoordinate& articulation = joint->getChildArticulationLink().getArticulation();
            if (articulation.getScene())
                articulation.wakeUp();

            switch (joint->getJointType())
            {
            case PxArticulationJointType::eREVOLUTE:
            case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
            {
                const float angleInDeg = data;
                data = degToRad(angleInDeg);
                if(joint->getMotion(PxArticulationAxis::eTWIST) == PxArticulationMotion::eLIMITED && isfinite(angleInDeg))
                {
                    intJoint->updateArticulationJointLimitLow(joint, PxArticulationAxis::eTWIST, data);
                }
                else
                {
                    OMNI_LOG_WARN(
                        kRoboticsLogChannel,
                        "Cannot change low limit to %2.f on %s during simulation. This change requires a simulation restart.",
                        angleInDeg, objectRecord->mPath.GetText());
                }
            }
            break;
            case PxArticulationJointType::ePRISMATIC:
            {
                intJoint->updateArticulationJointLimitLow(joint, PxArticulationAxis::eX, data);
            }
            break;
            case PxArticulationJointType::eSPHERICAL:
                data = degToRad(data);
                if (intJoint->mJointType == usdparser::eJointD6)
                {
                    PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
                    const bool validAxis = getD6ArticulationAxisFromLimit(property.GetString(), axis);
                    if (validAxis)
                    {
                        intJoint->updateArticulationJointLimitLow(joint, axis, data);
                    }
                }
                else  // case USD spherical joint - limit low is called on ConeAngle0Limit change on PxArticulationAxis::eSWING1
                {
                    if (data >= 0.0f)
                    {
                        joint->setLimitParams(::physx::PxArticulationAxis::eSWING1, PxArticulationLimit(-data, data));
                    }
                    else
                    {
                        joint->setLimitParams(::physx::PxArticulationAxis::eSWING1, PxArticulationLimit(-FLT_MAX, FLT_MAX));
                    }
                }
            case PxArticulationJointType::eFIX:
            case PxArticulationJointType::eUNDEFINED:
                break;
            }
        }
    }
    else if (internalType == ePTJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        // not right strictly speaking we should be able to switch between free/limited
        if (!isfinite(data))
            return true;

        PxJoint* joint = (PxJoint*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        if (joint)
        {
            switch (intJoint->mJointType)
            {
            case eJointRevolute:
            {
                PxD6Joint* revJoint = (PxD6Joint*)joint;
                PxJointAngularLimitPair limitPair = revJoint->getTwistLimit();
                data = degToRad(data);
                limitPair.lower = data;
                checkRevoluteJointLimits(limitPair, objectRecord->mPath.GetText());
                revJoint->setTwistLimit(limitPair);
            }
            break;
            case eJointPrismatic:
            {
                PxD6Joint* prismJoint = (PxD6Joint*)joint;
                PxJointLinearLimitPair limitPair = prismJoint->getLinearLimit((PxD6Axis::Enum)intJoint->mAxisIndex);
                limitPair.lower = data;
                prismJoint->setLinearLimit((PxD6Axis::Enum)intJoint->mAxisIndex, limitPair);
            }
            break;
            case eJointSpherical:
            {
                PxD6Joint* d6j = (PxD6Joint*)joint;
                PxJointLimitCone coneLimit = d6j->getSwingLimit();
                data = degToRad(data);
                coneLimit.yAngle = data;
                d6j->setSwingLimit(coneLimit);
            }
            break;
            case eJointDistance:
            {
                PxDistanceJoint* distanceJoint = (PxDistanceJoint*)joint;
                if (data >= 0.0f)
                {
                    distanceJoint->setDistanceJointFlag(PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, true);
                    distanceJoint->setMinDistance(data);
                }
                else
                {
                    distanceJoint->setDistanceJointFlag(PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, false);
                }
            }
            break;
            case eJointD6:
            {
                PxD6Joint* d6Joint = (PxD6Joint*)joint;
                const std::string& nameString = property.GetString();
                if (nameString == "limit:transX:physics:low")
                {
                    PxJointLinearLimitPair limitPair = d6Joint->getLinearLimit(PxD6Axis::eX);
                    limitPair.lower = data;                    
                    d6Joint->setLinearLimit(PxD6Axis::eX, limitPair);
                }
                else if (nameString == "limit:transY:physics:low")
                {
                    PxJointLinearLimitPair limitPair = d6Joint->getLinearLimit(PxD6Axis::eY);
                    limitPair.lower = data;
                    d6Joint->setLinearLimit(PxD6Axis::eY, limitPair);
                }
                else if (nameString == "limit:transZ:physics:low")
                {
                    PxJointLinearLimitPair limitPair = d6Joint->getLinearLimit(PxD6Axis::eZ);
                    limitPair.lower = data;
                    d6Joint->setLinearLimit(PxD6Axis::eZ, limitPair);
                }
                else if (nameString == "limit:rotX:physics:low")
                {
                    PxJointAngularLimitPair limitPair = d6Joint->getTwistLimit();
                    data = degToRad(data);
                    limitPair.lower = data;
                    d6Joint->setTwistLimit(limitPair);
                }
                else if (nameString == "limit:rotY:physics:low")
                {
                    PxJointLimitPyramid limitPair = d6Joint->getPyramidSwingLimit();
                    data = degToRad(data);
                    limitPair.yAngleMin = data;
                    d6Joint->setPyramidSwingLimit(limitPair);
                }
                else if (nameString == "limit:rotZ:physics:low")
                {
                    PxJointLimitPyramid limitPair = d6Joint->getPyramidSwingLimit();
                    data = degToRad(data);
                    limitPair.zAngleMin = data;
                    d6Joint->setPyramidSwingLimit(limitPair);
                }
            }
            break;
            default:
                break;
            }
        }
    }

    return true;
}

bool omni::physx::updateEnableCollision(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTJoint)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxJoint* joint = (PxJoint*)objectRecord->mPtr;
        if (joint)
            joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, data);
    }
    else if (internalType == ePTCustomJoint)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        CustomPhysXJoint* joint = (CustomPhysXJoint*)objectRecord->mPtr;
        if (joint)
            joint->getConstraint()->setFlag(PxConstraintFlag::eCOLLISION_ENABLED, data);
    }

    return true;
}

bool omni::physx::updateBreakForce(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxJoint* joint = (PxJoint*)objectRecord->mPtr;
        if (joint)
        {
            float breakForce;
            float breakTorque;
            joint->getBreakForce(breakForce, breakTorque);
            breakForce = isfinite(data) ? data : FLT_MAX;
            joint->setBreakForce(breakForce, breakTorque);
        }
    }
    else if (internalType == ePTCustomJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        CustomPhysXJoint* joint = (CustomPhysXJoint*)objectRecord->mPtr;
        if (joint)
        {
            float breakForce;
            float breakTorque;
            joint->getConstraint()->getBreakForce(breakForce, breakTorque);
            breakForce = isfinite(data) ? data : FLT_MAX;
            joint->getConstraint()->setBreakForce(breakForce, breakTorque);
        }
    }

    return true;
}

bool omni::physx::updateBreakTorque(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxJoint* joint = (PxJoint*)objectRecord->mPtr;
        if (joint)
        {
            float breakForce;
            float breakTorque;
            joint->getBreakForce(breakForce, breakTorque);
            breakTorque = isfinite(data) ? data : FLT_MAX;
            joint->setBreakForce(breakForce, breakTorque);
        }
    }
    else if (internalType == ePTCustomJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        CustomPhysXJoint* joint = (CustomPhysXJoint*)objectRecord->mPtr;
        if (joint)
        {
            float breakForce;
            float breakTorque;
            joint->getConstraint()->getBreakForce(breakForce, breakTorque);
            breakTorque = isfinite(data) ? data : FLT_MAX;
            joint->getConstraint()->setBreakForce(breakForce, breakTorque);
        }
    }

    return true;
}

void wakeUpJointRigidsOrArticulations(PxRigidActor* actor0, PxRigidActor* actor1)
{
    if (actor0)
    {
        if(PxRigidDynamic* rigidLink = actor0->is<PxRigidDynamic>())
        {
            rigidLink->wakeUp();
        }
        else if(PxArticulationLink* articulationLink = actor0->is<PxArticulationLink>())
        {
            PxArticulationReducedCoordinate& articulation = articulationLink->getArticulation();
            if(articulation.getScene())
            {
                articulation.wakeUp();
            }
        }
    }
    if (actor1)
    {
        if(PxRigidDynamic* rigidLink = actor1->is<PxRigidDynamic>())
        {
            rigidLink->wakeUp();
        }
        else if(PxArticulationLink* articulationLink = actor1->is<PxArticulationLink>())
        {
            PxArticulationReducedCoordinate& articulation = articulationLink->getArticulation();
            if(articulation.getScene())
            {
                articulation.wakeUp();
            }
        }
    }
}

bool updateLocalPosInternal(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode, bool pose0)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTJoint)
    {
        GfVec3f data;
        if (!getValue<GfVec3f>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxJoint* joint = (PxJoint*)objectRecord->mPtr;
        if (joint)
        {
            const PxJointActorIndex::Enum actorIndex = pose0 ? PxJointActorIndex::eACTOR0 : PxJointActorIndex::eACTOR1;
            PxTransform localPose = joint->getLocalPose(actorIndex);
            PxRigidActor* actor0 = nullptr;
            PxRigidActor* actor1 = nullptr;
            joint->getActors(actor0, actor1);
            ObjectId actorObjectID = kInvalidObjectId;
            if(pose0)
            {
                if(actor0)
                {
                    actorObjectID = (ObjectId)actor0->userData;
                }
            }
            else
            {
                if(actor1)
                {
                    actorObjectID = (ObjectId)actor1->userData;
                }
            }
            localPose.p = toPhysX(data);
            if(actorObjectID != kInvalidObjectId)
            {
                const InternalDatabase::Record& actorRecord = db.getRecords()[actorObjectID];
                UsdPrim actorPrim = attachedStage.getStage()->GetPrimAtPath(actorRecord.mPath);
                if(actorPrim)
                {
                    UsdGeomXform actorXForm = UsdGeomXform(actorPrim);
                    GfMatrix4d worldRel = actorXForm.ComputeLocalToWorldTransform(timeCode);
                    const GfTransform tr(worldRel);
                    GfVec3f scale = GfVec3f(tr.GetScale());
                    localPose.p.x *= scale[0];
                    localPose.p.y *= scale[1];
                    localPose.p.z *= scale[2];
                }
            }
            joint->setLocalPose(actorIndex, localPose);
            wakeUpJointRigidsOrArticulations(actor0, actor1);
        }
    }
    else if(internalType == ePTLinkJoint)
    {
        // OM-42711
        CARB_LOG_WARN_ONCE("Updating joint local poses in articulations is not supported after simulation start (Joint %s)", objectRecord->mPath.GetText());
    }

    return true;
}

bool omni::physx::updateLocalPos0(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateLocalPosInternal(attachedStage, objectId, property, timeCode, true);
}

bool omni::physx::updateLocalPos1(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateLocalPosInternal(attachedStage, objectId, property, timeCode, false);
}

bool updateLocalRotInternal(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode, bool pose0)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTJoint)
    {
        GfQuatf data;
        if (!getValue<GfQuatf>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxJoint* joint = (PxJoint*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        if (joint && intJoint)
        {
            const PxJointActorIndex::Enum actorIndex = pose0 ? PxJointActorIndex::eACTOR0 : PxJointActorIndex::eACTOR1;
            PxTransform localPose = joint->getLocalPose(actorIndex);
            localPose.q = toPhysX(data);
            intJoint->fixupLocalPose(localPose);
            joint->setLocalPose(actorIndex, localPose);
            PxRigidActor* actor0 = nullptr;
            PxRigidActor* actor1 = nullptr;
            joint->getActors(actor0, actor1);
            wakeUpJointRigidsOrArticulations(actor0, actor1);
        }
    }
    else if (internalType == ePTLinkJoint)
    {
        CARB_LOG_WARN_ONCE("Updating joint local poses in articulations is not supported after simulation start (Joint %s)", objectRecord->mPath.GetText());
        /* TODO PREIST: OM-42711
        GfQuatf data;
        if (!getValue<GfQuatf>(objectRecord->mPrim, property, timeCode, data))
            return true;

        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        if (joint && intJoint)
        {
            // pose0 && mLocalPose0IsParentPose -> update Parent
            // !pose0 && mLocalPose0IsParentPose -> update Child
            // pose0 && !mLocalPose0IsParentPose -> update Child
            // !pose0 && !mLocalPose0IsParentPose -> update Parent
            // -> XOR
            const bool updateChild = pose0 ^ intJoint->mLocalPose0IsParentPose;
            PxTransform localPose = updateChild ? joint->getChildPose() : joint->getParentPose();
            localPose.q = toPhysX(data);
            intJoint->fixupLocalPose(localPose);
            if (updateChild)
            {
                joint->setChildPose(localPose);
            }
            else
            {
                joint->setParentPose(localPose);
            }
        }
        */
    }

    return true;
}

bool omni::physx::updateLocalRot0(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateLocalRotInternal(attachedStage, objectId, property, timeCode, true);
}

bool omni::physx::updateLocalRot1(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateLocalRotInternal(attachedStage, objectId, property, timeCode, false);
}

bool omni::physx::updateGearRatio(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxJoint* joint = (PxJoint*)objectRecord->mPtr;
        const InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;

        if (intJoint->mJointType == eJointGear)
        {
            PxGearJoint* gearJoint = (PxGearJoint*)joint;
            if (gearJoint)
                gearJoint->setGearRatio(data);
        }
    }

    return true;
}

bool updateGearHinge(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode, bool hinge0)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTJoint)
    {
        PxJoint* joint = (PxJoint*)objectRecord->mPtr;
        const InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;

        if (intJoint->mJointType == eJointGear)
        {
            PxGearJoint* gearJoint = (PxGearJoint*)joint;
            if (gearJoint)
            {
                PxTransform localPose(PxIdentity);
                PxRigidActor* actor0 = nullptr;
                PxRigidActor* actor1 = nullptr;
                gearJoint->getActors(actor0, actor1);
                PxBase* hinge = nullptr;
                UsdRelationship rel = attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath).GetRelationship(property);
                if (rel)
                {
                    const PxBase* h0 = nullptr;
                    const PxBase* h1 = nullptr;
                    gearJoint->getHinges(h0, h1);

                    SdfPathVector hinges;
                    rel.GetTargets(&hinges);

                    if (hinge0)
                    {
                        if (hinges.empty())
                        {
                            gearJoint->setHinges(nullptr, h1);
                        }
                        else
                        {
                            if (!getJointAndLocalPose(attachedStage, hinges[0], actor0, hinge, localPose))
                            {
                                CARB_LOG_ERROR(
                                    "Invalid configuration for gear joint(% s) - neither parent nor child link of hinge 0 (% s) refer to body 0.",
                                    objectRecord->mPath.GetText(), hinges[0].GetText());
                                return true;
                            }
                            gearJoint->setHinges(hinge, h1);
                            gearJoint->setLocalPose(::physx::PxJointActorIndex::eACTOR0, localPose);
                        }
                    }
                    else
                    {
                        if (hinges.empty())
                        {
                            gearJoint->setHinges(h0, nullptr);
                        }
                        else
                        {
                            if (!getJointAndLocalPose(attachedStage, hinges[0], actor1, hinge, localPose))
                            {
                                CARB_LOG_ERROR(
                                    "Invalid configuration for gear joint(% s) - neither parent nor child link of hinge 0 (% s) refer to body 0.",
                                    objectRecord->mPath.GetText(), hinges[0].GetText());
                                return true;
                            }
                            gearJoint->setHinges(h0, hinge);
                            gearJoint->setLocalPose(::physx::PxJointActorIndex::eACTOR1, localPose);
                        }
                    }
                }
            }                
        }
    }

    return true;
}

bool omni::physx::updateGearHinge0(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateGearHinge(attachedStage, objectId, property, timeCode, true);
}

bool omni::physx::updateGearHinge1(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
return updateGearHinge(attachedStage, objectId, property, timeCode, false);
}

bool updateRackHingePrismatic(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode, bool hingeJointUpdate)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTJoint)
    {
        PxJoint* joint = (PxJoint*)objectRecord->mPtr;
        const InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;

        if (intJoint->mJointType == eJointRackAndPinion)
        {
            PxRackAndPinionJoint* rackJoint = (PxRackAndPinionJoint*)joint;
            if (rackJoint)
            {
                PxTransform localPose(PxIdentity);
                PxRigidActor* actor0 = nullptr;
                PxRigidActor* actor1 = nullptr;
                rackJoint->getActors(actor0, actor1);
                PxBase* hinge = nullptr;
                UsdRelationship rel = attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath).GetRelationship(property);
                if (rel)
                {
                    const PxBase* j0 = nullptr;
                    const PxBase* j1 = nullptr;
                    rackJoint->getJoints(j0, j1);

                    SdfPathVector hinges;
                    rel.GetTargets(&hinges);
                    if (hingeJointUpdate)
                    {
                        if (hinges.empty())
                        {
                            rackJoint->setJoints(nullptr, j1);
                        }
                        else
                        {
                            if (!getJointAndLocalPose(attachedStage, hinges[0], actor0, hinge, localPose))
                            {
                                CARB_LOG_ERROR(
                                    "Invalid configuration for gear joint(% s) - neither parent nor child link of hinge 0 (% s) refer to body 0.",
                                    objectRecord->mPath.GetText(), hinges[0].GetText());
                                return true;
                            }
                            rackJoint->setJoints(hinge, j1);
                            rackJoint->setLocalPose(::physx::PxJointActorIndex::eACTOR0, localPose);
                        }
                    }
                    else
                    {
                        if (hinges.empty())
                        {
                            rackJoint->setJoints(j0, nullptr);
                        }
                        else
                        {
                            if (!getJointAndLocalPose(attachedStage, hinges[0], actor1, hinge, localPose))
                            {
                                CARB_LOG_ERROR(
                                    "Invalid configuration for gear joint(% s) - neither parent nor child link of hinge 0 (% s) refer to body 0.",
                                    objectRecord->mPath.GetText(), hinges[0].GetText());
                                return true;
                            }
                            rackJoint->setJoints(j0, hinge);
                            rackJoint->setLocalPose(::physx::PxJointActorIndex::eACTOR1, localPose);
                        }
                    }
                }
            }                
        }
    }

    return true;
}

bool omni::physx::updateRackHinge(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateRackHingePrismatic(attachedStage, objectId, property, timeCode, true);
}

bool omni::physx::updateRackPrismatic(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
return updateRackHingePrismatic(attachedStage, objectId, property, timeCode, false);
}


bool omni::physx::updateRackPinionRatio(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
                                  const pxr::TfToken& property,
                                  const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxJoint* joint = (PxJoint*)objectRecord->mPtr;        
        const InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;

        if (intJoint->mJointType == eJointRackAndPinion)
        {
            PxRackAndPinionJoint* rpJoint = (PxRackAndPinionJoint*)joint;
            // USD ratio is deg / distance - PhysX is rad / distance
            rpJoint->setRatio(omni::physx::degToRad(data));
        }
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// articulation
bool omni::physx::updateArticulationFixBase(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTArticulation)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationReducedCoordinate* articulation = (PxArticulationReducedCoordinate*)objectRecord->mPtr;
        if (articulation)
        {
            articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, data);
        }
    }

    return true;
}

bool omni::physx::updateArticulationSolverPositionIterationCount(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTArticulation)
    {
        int data;
        if (!getValue<int>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationReducedCoordinate* articulation = (PxArticulationReducedCoordinate*)objectRecord->mPtr;
        InternalArticulation* intArt = (InternalArticulation*)objectRecord->mInternalPtr;
        if (articulation && intArt)
        {
            PxU32 posIt, velIt;
            articulation->getSolverIterationCounts(posIt, velIt);
            const InternalScene* intScene = intArt->mPhysxScene->getInternalScene();
            data = intScene->clampPosIterationCount(data);
            articulation->setSolverIterationCounts(data, velIt);
        }
    }

    return true;
}

bool omni::physx::updateArticulationSolverVelocityIterationCount(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTArticulation)
    {
        int data;
        if (!getValue<int>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationReducedCoordinate* articulation = (PxArticulationReducedCoordinate*)objectRecord->mPtr;
        InternalArticulation* intArt = (InternalArticulation*)objectRecord->mInternalPtr;
        if (articulation && intArt)
        {
            PxU32 posIt, velIt;
            articulation->getSolverIterationCounts(posIt, velIt);
            const InternalScene* intScene = intArt->mPhysxScene->getInternalScene();
            data = intScene->clampVelIterationCount(data);
            articulation->setSolverIterationCounts(posIt, data);
        }
    }

    return true;
}

bool omni::physx::updateArticulationSleepThreshold(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTArticulation)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationReducedCoordinate* articulation = (PxArticulationReducedCoordinate*)objectRecord->mPtr;
        if (articulation)
        {
            articulation->setSleepThreshold(data);
        }
    }

    return true;
}

bool omni::physx::updateArticulationStabilizationThreshold(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTArticulation)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationReducedCoordinate* articulation = (PxArticulationReducedCoordinate*)objectRecord->mPtr;
        if (articulation)
        {
            articulation->setStabilizationThreshold(data);
        }
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// articulation link
bool omni::physx::updateArticulationMaxJointVelocity(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTLinkJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        if (joint)
        {
            switch (joint->getJointType())
            {
                case PxArticulationJointType::eREVOLUTE:
                case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
                case PxArticulationJointType::eSPHERICAL:
                    data =  isfinite(data) ? degToRad(data) : FLT_MAX;
                    break;
                default:
                    data =  isfinite(data) ? data : FLT_MAX;
                    break;
            }

            joint->setMaxJointVelocity(data);
        }
    }
    return true;
}

bool omni::physx::updateArticulationMaxJointVelocityPerAxis(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTLinkJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        const UsdPrim prim =  attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath);
        if (joint)
        {
            switch (joint->getJointType())
            {
            case PxArticulationJointType::eREVOLUTE:
            case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
            {
                if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, UsdPhysicsTokens->angular))
                {
                    data =  isfinite(data) ? degToRad(data) : FLT_MAX;
                    joint->setMaxJointVelocity(PxArticulationAxis::eTWIST, data);
                }
            }
            break;
            case PxArticulationJointType::ePRISMATIC:
            {
                if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, UsdPhysicsTokens->linear))
                {
                    data = isfinite(data) ? data : FLT_MAX;
                    joint->setMaxJointVelocity(PxArticulationAxis::eX, data);
                }
            }
            break;
            case PxArticulationJointType::eSPHERICAL:
            {
                PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
                const bool validAxis = getD6ArticulationAxisFromProperties(property.GetString(), axis);
                TfToken token = UsdPhysicsTokens->rotX;
                if (validAxis)
                {
                    mapAxisToToken(axis, token);
                    if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, token))
                    {
                        data = isfinite(data) ? degToRad(data) : FLT_MAX;
                        joint->setMaxJointVelocity(axis, data);
                    }
                }
            }
            case PxArticulationJointType::eFIX:
            case PxArticulationJointType::eUNDEFINED:
                break;
            }
        }
    }
    return true;
}

bool omni::physx::updateArticulationFrictionCoefficient(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTLinkJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        if (joint)
        {
            joint->setFrictionCoefficient(data);
        }
    }
    return true;
}

bool omni::physx::updateArmature(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTLinkJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        if (joint && intJoint)
        {
            PxArticulationReducedCoordinate& articulation = joint->getChildArticulationLink().getArticulation();
            if (articulation.getScene())
                articulation.wakeUp();

            switch (joint->getJointType())
            {
            case PxArticulationJointType::eREVOLUTE:
            case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
            {
                joint->setArmature(PxArticulationAxis::eTWIST, data);
            }
            break;
            case PxArticulationJointType::ePRISMATIC:
            {
                joint->setArmature(PxArticulationAxis::eX, data);
            }
            break;
            case PxArticulationJointType::eSPHERICAL:
            {
                joint->setArmature(PxArticulationAxis::eTWIST, data);
                joint->setArmature(PxArticulationAxis::eSWING1, data);
                joint->setArmature(PxArticulationAxis::eSWING2, data);
            }
            case PxArticulationJointType::eFIX:
            case PxArticulationJointType::eUNDEFINED:
                break;
            }
        }
    }

    return true;
}

bool omni::physx::updateArmaturePerAxis(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTLinkJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        const UsdPrim prim =  attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath);
        if (joint && intJoint)
        {
            PxArticulationReducedCoordinate& articulation = joint->getChildArticulationLink().getArticulation();
            if (articulation.getScene())
                articulation.wakeUp();

            switch (joint->getJointType())
            {
            case PxArticulationJointType::eREVOLUTE:
            case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
            {
                if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, UsdPhysicsTokens->angular))
                {
                    data = isfinite(data) ? data : FLT_MAX;
                    joint->setArmature(PxArticulationAxis::eTWIST, data);
                }
            }
            break;
            case PxArticulationJointType::ePRISMATIC:
            {
                if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, UsdPhysicsTokens->linear))
                {
                    data = isfinite(data) ? data : FLT_MAX;
                    joint->setArmature(PxArticulationAxis::eX, data);
                }
            }
            break;
            case PxArticulationJointType::eSPHERICAL:
            {
                PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
                const bool validAxis = getD6ArticulationAxisFromProperties(property.GetString(), axis);
                TfToken token = UsdPhysicsTokens->rotX;
                if (validAxis)
                {
                    mapAxisToToken(axis, token);
                    if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, token))
                    {
                        data = isfinite(data) ? data : FLT_MAX;
                        joint->setArmature(axis, data);
                    }
                }
            }
            case PxArticulationJointType::eFIX:
            case PxArticulationJointType::eUNDEFINED:
                break;
            }
        }
    }

    return true;
}

bool omni::physx::updateArticulationStaticFrictionEffort(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTLinkJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        const UsdPrim prim =  attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath);
        if (joint && intJoint)
        {
            PxArticulationReducedCoordinate& articulation = joint->getChildArticulationLink().getArticulation();
            if (articulation.getScene())
                articulation.wakeUp();

            switch (joint->getJointType())
            {
            case PxArticulationJointType::eREVOLUTE:
            case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
            {
                if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, UsdPhysicsTokens->angular))
                {
                    PxJointFrictionParams params = joint->getFrictionParams(PxArticulationAxis::eTWIST);
                    params.staticFrictionEffort = isfinite(data) ? data : FLT_MAX;
                    joint->setFrictionParams(PxArticulationAxis::eTWIST, params);
                }
            }
            break;
            case PxArticulationJointType::ePRISMATIC:
            {
                if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, UsdPhysicsTokens->linear))
                {
                    PxJointFrictionParams params = joint->getFrictionParams(PxArticulationAxis::eX);
                    params.staticFrictionEffort = isfinite(data) ? data : FLT_MAX;
                    joint->setFrictionParams(PxArticulationAxis::eX, params);
                }
            }
            break;
            case PxArticulationJointType::eSPHERICAL:
            {
                PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
                const bool validAxis = getD6ArticulationAxisFromProperties(property.GetString(), axis);
                TfToken token = UsdPhysicsTokens->rotX;
                if (validAxis)
                    {
                        mapAxisToToken(axis, token);
                        if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, token))
                        {
                            PxJointFrictionParams params = joint->getFrictionParams(axis);
                            params.staticFrictionEffort = isfinite(data) ? data : FLT_MAX;
                            joint->setFrictionParams(axis, params);
                        }
                    }
                    
            }
            case PxArticulationJointType::eFIX:
            case PxArticulationJointType::eUNDEFINED:
                break;
            }
        }
    }

    return true;
}

bool omni::physx::updateArticulationDynamicFrictionEffort(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTLinkJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        const UsdPrim prim =  attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath);
        if (joint && intJoint)
        {
            PxArticulationReducedCoordinate& articulation = joint->getChildArticulationLink().getArticulation();
            if (articulation.getScene())
                articulation.wakeUp();

            switch (joint->getJointType())
            {
            case PxArticulationJointType::eREVOLUTE:
            case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
            {
                if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, UsdPhysicsTokens->angular))
                {
                    PxJointFrictionParams params = joint->getFrictionParams(PxArticulationAxis::eTWIST);
                    params.dynamicFrictionEffort = isfinite(data) ? data : FLT_MAX;
                    joint->setFrictionParams(PxArticulationAxis::eTWIST, params);
                }
            }
            break;
            case PxArticulationJointType::ePRISMATIC:
            {
                if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, UsdPhysicsTokens->linear))
                {
                    PxJointFrictionParams params = joint->getFrictionParams(PxArticulationAxis::eX);
                    params.dynamicFrictionEffort = isfinite(data) ? data : FLT_MAX;
                    joint->setFrictionParams(PxArticulationAxis::eX, params);
                }
            }
            break;
            case PxArticulationJointType::eSPHERICAL:
            {
                PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
                const bool validAxis = getD6ArticulationAxisFromProperties(property.GetString(), axis);
                TfToken token = UsdPhysicsTokens->rotX;
                if (validAxis)
                    {
                        mapAxisToToken(axis, token);
                        if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, token))
                        {
                            PxJointFrictionParams params = joint->getFrictionParams(axis);
                            params.dynamicFrictionEffort = isfinite(data) ? data : FLT_MAX;
                            joint->setFrictionParams(axis, params);
                        }
                    }
                    
            }
            case PxArticulationJointType::eFIX:
            case PxArticulationJointType::eUNDEFINED:
                break;
            }
        }
    }

    return true;
}

bool omni::physx::updateArticulationViscousFrictionCoefficient(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTLinkJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationJointReducedCoordinate* joint = (PxArticulationJointReducedCoordinate*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        const UsdPrim prim =  attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath);
        if (joint && intJoint)
        {
            PxArticulationReducedCoordinate& articulation = joint->getChildArticulationLink().getArticulation();
            if (articulation.getScene())
                articulation.wakeUp();

            switch (joint->getJointType())
            {
            case PxArticulationJointType::eREVOLUTE:
            case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
            {
                if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, UsdPhysicsTokens->angular))
                {
                    PxJointFrictionParams params = joint->getFrictionParams(PxArticulationAxis::eTWIST);
                    params.viscousFrictionCoefficient = isfinite(data) ? radToDeg(data) : FLT_MAX; // torque * second / degrees
                    joint->setFrictionParams(PxArticulationAxis::eTWIST, params);
                }
            }
            break;
            case PxArticulationJointType::ePRISMATIC:
            {
                if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, UsdPhysicsTokens->linear))
                {
                    PxJointFrictionParams params = joint->getFrictionParams(PxArticulationAxis::eX);
                    params.viscousFrictionCoefficient = isfinite(data) ? data : FLT_MAX;;
                    joint->setFrictionParams(PxArticulationAxis::eX, params);
                }
            }
            break;
            case PxArticulationJointType::eSPHERICAL:
            {
                PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
                const bool validAxis = getD6ArticulationAxisFromProperties(property.GetString(), axis);
                TfToken token = UsdPhysicsTokens->rotX;
                if (validAxis)
                    {
                        mapAxisToToken(axis, token);
                        if (prim.IsA<UsdPhysicsJoint>() && prim.HasAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, token))
                        {
                            PxJointFrictionParams params = joint->getFrictionParams(axis);
                            params.viscousFrictionCoefficient = isfinite(data) ? radToDeg(data) : FLT_MAX; // torque * second / degrees
                            joint->setFrictionParams(axis, params);
                        }
                    }
                    
            }
            case PxArticulationJointType::eFIX:
            case PxArticulationJointType::eUNDEFINED:
                break;
            }
        }
    }

    return true;
}


bool updateLimitField(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
                      const pxr::TfToken& property,
                      const pxr::UsdTimeCode& timeCode,
                      PxReal PxJointAngularLimitPair::*angularField,
                      PxReal PxJointLinearLimitPair::*linearField,
                      PxReal PxJointLimitCone::*coneField,
                      PxReal PxJointLimitPyramid::*pyramidField,
                      void (PxDistanceJoint::*distanceFunc)(PxReal),
                      const pxr::TfToken d6Tokens[6])
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxJoint* joint = (PxJoint*)objectRecord->mPtr;
        InternalJoint* intJoint = (InternalJoint*)objectRecord->mInternalPtr;
        if (joint)
        {
            switch (intJoint->mJointType)
            {
            case eJointRevolute: {
                PxD6Joint* revJoint = (PxD6Joint*)joint;
                PxJointAngularLimitPair limitPair = revJoint->getTwistLimit();
                limitPair.*angularField = data;
                checkRevoluteJointLimits(limitPair, objectRecord->mPath.GetText());
                revJoint->setTwistLimit(limitPair);
            }
            break;
            case eJointPrismatic: {
                PxD6Joint* prismJoint = (PxD6Joint*)joint;
                PxJointLinearLimitPair limitPair = prismJoint->getLinearLimit((PxD6Axis::Enum)intJoint->mAxisIndex);
                limitPair.*linearField = data;
                prismJoint->setLinearLimit((PxD6Axis::Enum)intJoint->mAxisIndex, limitPair);
            }
            break;
            case eJointSpherical: {
                PxD6Joint* d6j = (PxD6Joint*)joint;
                PxJointLimitCone coneLimit = d6j->getSwingLimit();
                coneLimit.*coneField = data;
                d6j->setSwingLimit(coneLimit);
            }
            break;
            case eJointDistance: {
                PxDistanceJoint* distanceJoint = (PxDistanceJoint*)joint;
                if(distanceFunc)
                {
                    (distanceJoint->*distanceFunc)(data);
                }
            }
            break;
            case eJointD6: {
                PxD6Joint* d6Joint = (PxD6Joint*)joint;
                if (property == d6Tokens[0])
                {
                    PxJointLinearLimitPair limitPair = d6Joint->getLinearLimit(PxD6Axis::eX);
                    limitPair.*linearField = data;
                    d6Joint->setLinearLimit(PxD6Axis::eX, limitPair);
                }
                else if (property == d6Tokens[1])
                {
                    PxJointLinearLimitPair limitPair = d6Joint->getLinearLimit(PxD6Axis::eY);
                    limitPair.*linearField = data;
                    d6Joint->setLinearLimit(PxD6Axis::eY, limitPair);
                }
                else if (property == d6Tokens[2])
                {
                    PxJointLinearLimitPair limitPair = d6Joint->getLinearLimit(PxD6Axis::eZ);
                    limitPair.*linearField = data;
                    d6Joint->setLinearLimit(PxD6Axis::eZ, limitPair);
                }
                else if (property == d6Tokens[3])
                {
                    PxJointAngularLimitPair limitPair = d6Joint->getTwistLimit();
                    limitPair.*angularField = data;
                    d6Joint->setTwistLimit(limitPair);
                }
                else if (property == d6Tokens[4])
                {
                    PxJointLimitPyramid limitPair = d6Joint->getPyramidSwingLimit();
                    limitPair.*pyramidField = data;
                    d6Joint->setPyramidSwingLimit(limitPair);
                }
                else if (property == d6Tokens[5])
                {
                    PxJointLimitPyramid limitPair = d6Joint->getPyramidSwingLimit();
                    limitPair.*pyramidField = data;
                    d6Joint->setPyramidSwingLimit(limitPair);
                }
            }
            break;
            default:
                break;
            }
        }
    }
    return true;
}

bool omni::physx::updateLimitBounceThreshold(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    static const TfToken d6Tokens[] = 
    {
        TfToken("physxLimit:transX:bounceThreshold"),
        TfToken("physxLimit:transY:bounceThreshold"),
        TfToken("physxLimit:transZ:bounceThreshold"),
        TfToken("physxLimit:rotX:bounceThreshold"),
        TfToken("physxLimit:rotY:bounceThreshold"),
        TfToken("physxLimit:rotZ:bounceThreshold")
    };
    return updateLimitField(attachedStage, objectId, property, timeCode,
                            &PxJointAngularLimitPair::bounceThreshold,
                            &PxJointLinearLimitPair::bounceThreshold,
                            &PxJointLimitCone::bounceThreshold,
                            &PxJointLimitPyramid::bounceThreshold,
                            nullptr,
                            d6Tokens);
}

bool omni::physx::updateLimitDamping(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    static const TfToken d6Tokens[] = 
    {
        TfToken("physxLimit:transX:damping"),
        TfToken("physxLimit:transY:damping"),
        TfToken("physxLimit:transZ:damping"),
        TfToken("physxLimit:rotX:damping"),
        TfToken("physxLimit:rotY:damping"),
        TfToken("physxLimit:rotZ:damping")
    };
    return updateLimitField(attachedStage, objectId, property, timeCode,
                            &PxJointAngularLimitPair::damping,
                            &PxJointLinearLimitPair::damping,
                            &PxJointLimitCone::damping,
                            &PxJointLimitPyramid::damping,
                            &PxDistanceJoint::setDamping,
                            d6Tokens);
}

bool omni::physx::updateLimitRestitution(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    static const TfToken d6Tokens[] = 
    {
        TfToken("physxLimit:transX:restitution"),
        TfToken("physxLimit:transY:restitution"),
        TfToken("physxLimit:transZ:restitution"),
        TfToken("physxLimit:rotX:restitution"),
        TfToken("physxLimit:rotY:restitution"),
        TfToken("physxLimit:rotZ:restitution")
    };
    return updateLimitField(attachedStage, objectId, property, timeCode,
                            &PxJointAngularLimitPair::restitution,
                            &PxJointLinearLimitPair::restitution,
                            &PxJointLimitCone::restitution,
                            &PxJointLimitPyramid::restitution,
                            nullptr,
                            d6Tokens);
}

bool omni::physx::updateLimitStiffness(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    static const TfToken d6Tokens[] = 
    {
        TfToken("physxLimit:transX:stiffness"),
        TfToken("physxLimit:transY:stiffness"),
        TfToken("physxLimit:transZ:stiffness"),
        TfToken("physxLimit:rotX:stiffness"),
        TfToken("physxLimit:rotY:stiffness"),
        TfToken("physxLimit:rotZ:stiffness")
    };
    return updateLimitField(attachedStage, objectId, property, timeCode,
                            &PxJointAngularLimitPair::stiffness,
                            &PxJointLinearLimitPair::stiffness,
                            &PxJointLimitCone::stiffness,
                            &PxJointLimitPyramid::stiffness,
                            &PxDistanceJoint::setStiffness,
                            d6Tokens);
}

bool omni::physx::updateDistanceJointSpringDamping(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxJoint* joint = (PxJoint*)objectRecord->mPtr;
        PxDistanceJoint* dj = joint->is<PxDistanceJoint>();
        if (dj)
        {
            dj->setDamping(data);
            wakeJointActors(*dj);
        }
    }

    return true;
}

bool omni::physx::updateDistanceJointSpringStiffness(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTJoint)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxJoint* joint = (PxJoint*)objectRecord->mPtr;
        PxDistanceJoint* dj = joint->is<PxDistanceJoint>();
        if (dj)
        {
            dj->setStiffness(data);
            wakeJointActors(*dj);
        }
    }

    return true;
}

bool omni::physx::updateDistanceJointSpringEnabled(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTJoint)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxJoint* joint = (PxJoint*)objectRecord->mPtr;
        PxDistanceJoint* dj = joint->is<PxDistanceJoint>();
        if (dj)
        {
            dj->setDistanceJointFlag(PxDistanceJointFlag::eSPRING_ENABLED, data);
            wakeJointActors(*dj);
        }
    }

    return true;
}
