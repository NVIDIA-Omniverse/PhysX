// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXPropertiesUpdate.h"

#include <PhysXTools.h>
#include <Setup.h>
#include <OmniPhysX.h>

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>


using namespace ::physx;
using namespace carb;
using namespace pxr;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

//           123456789012
// length of physxTendon: attribute namespace plus colon:
static size_t g_PhysxTendonLength = 12u;

static bool isCorrectInstance(const std::string& instanceName, const std::string& propertyName, const size_t first, const size_t last)
{
    return instanceName == propertyName.substr(first, propertyName.length() - last - first - 1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// update spatial tendons
bool omni::physx::updateSpatialTendonStiffness(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && intAttachment && isCorrectInstance(intAttachment->instanceName.GetString(), property, g_PhysxTendonLength, 9u))
    {
        const pxr::SdfPath basePath = objectRecord->mPath;
        const std::string enabledString = "physxTendon:" + intAttachment->instanceName.GetString() + ":tendonEnabled";
        bool isEnabled;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, TfToken(enabledString), timeCode, isEnabled))
            return true;

        if (isEnabled)
        {
            float data;
            if (getFloatBounded(attachedStage, objectRecord->mPath, property, timeCode, data, 0.f, FLT_MAX))
            {
                PxArticulationSpatialTendon* const tendon = pxAttachment->getTendon();
                tendon->setStiffness(data);

                // wake up articulation
                tendon->getArticulation()->wakeUp();
            }
        }
    }

    return true;
}

bool omni::physx::updateSpatialTendonDamping(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && intAttachment && isCorrectInstance(intAttachment->instanceName.GetString(), property, g_PhysxTendonLength, 7u))
    {
        const pxr::SdfPath basePath = objectRecord->mPath;
        const std::string enabledString = "physxTendon:" + intAttachment->instanceName.GetString() + ":tendonEnabled";
        bool isEnabled;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, TfToken(enabledString), timeCode, isEnabled))
            return true;

        if (isEnabled)
        {
            float data;
            if (getFloatBounded(attachedStage, objectRecord->mPath, property, timeCode, data, 0.f, FLT_MAX))
            {
                PxArticulationSpatialTendon* const tendon = pxAttachment->getTendon();
                tendon->setDamping(data);

                // wake up articulation
                tendon->getArticulation()->wakeUp();
            }
        }
    }

    return true;
}

bool omni::physx::updateSpatialTendonLimitStiffness(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && intAttachment && isCorrectInstance(intAttachment->instanceName.GetString(), property, g_PhysxTendonLength, 14u))
    {
        const pxr::SdfPath basePath = objectRecord->mPath;
        const std::string enabledString = "physxTendon:" + intAttachment->instanceName.GetString() + ":tendonEnabled";
        bool isEnabled;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, TfToken(enabledString), timeCode, isEnabled))
            return true;

        if (isEnabled)
        {
            float data;
            if (getFloatBounded(attachedStage, objectRecord->mPath, property, timeCode, data, 0.f, FLT_MAX))
            {
                PxArticulationSpatialTendon* const tendon = pxAttachment->getTendon();
                tendon->setLimitStiffness(data);

                // wake up articulation
                tendon->getArticulation()->wakeUp();
            }
        }
    }

    return true;
}

bool omni::physx::updateSpatialTendonOffset(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && intAttachment && isCorrectInstance(intAttachment->instanceName.GetString(), property, g_PhysxTendonLength, 6u))
    {
        float data;
        if (getFloatBounded(attachedStage, objectRecord->mPath, property, timeCode, data, -FLT_MAX, FLT_MAX))
        {
            PxArticulationSpatialTendon* const tendon = pxAttachment->getTendon();
            tendon->setOffset(data);

            // wake up articulation
            tendon->getArticulation()->wakeUp();
        }
    }

    return true;
}

bool omni::physx::updateSpatialTendonEnabled(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && intAttachment && isCorrectInstance(intAttachment->instanceName.GetString(), property, g_PhysxTendonLength, 13u))
    {
        bool isEnabled;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, isEnabled))
            return true;

        PxArticulationSpatialTendon* const tendon = pxAttachment->getTendon();
        if (isEnabled)
        {
            const pxr::SdfPath basePath = objectRecord->mPath;
            const std::string stiffnessString = "physxTendon:" + intAttachment->instanceName.GetString() + ":stiffness";
            const std::string limitStiffnessString = "physxTendon:" + intAttachment->instanceName.GetString() + ":limitStiffness";
            const std::string dampingString = "physxTendon:" + intAttachment->instanceName.GetString() + ":damping";

            float stiffness, limitStiffness, damping;
            if (getFloatBounded(attachedStage, objectRecord->mPath, TfToken(stiffnessString), timeCode, stiffness, 0.f, FLT_MAX))
                tendon->setStiffness(stiffness);

            if (getFloatBounded(attachedStage, objectRecord->mPath, TfToken(limitStiffnessString), timeCode, limitStiffness, 0.f, FLT_MAX))
                tendon->setLimitStiffness(limitStiffness);

            if (getFloatBounded(attachedStage, objectRecord->mPath, TfToken(dampingString), timeCode, damping, 0.f, FLT_MAX))
                tendon->setDamping(damping);
        }
        else
        {
            tendon->setStiffness(0.f);
            tendon->setLimitStiffness(0.f);
            tendon->setDamping(0.f);
        }

        // wake up articulation
        tendon->getArticulation()->wakeUp();
    }

    return true;
}

bool omni::physx::updateTendonAttachmentGearing(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && intAttachment && isCorrectInstance(intAttachment->instanceName.GetString(), property, g_PhysxTendonLength, 7u))
    {
        float data;
        if (getFloatBounded(attachedStage, objectRecord->mPath, property, timeCode, data, -FLT_MAX, FLT_MAX))
        {
            pxAttachment->setCoefficient(data);

            // wake up articulation
            pxAttachment->getTendon()->getArticulation()->wakeUp();
        }
    }

    return true;
}

bool omni::physx::updateTendonAttachmentLocalPos(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && intAttachment && isCorrectInstance(intAttachment->instanceName.GetString(), property, g_PhysxTendonLength, 8u))
    {
        pxr::GfVec3f vec;
        if (getValue(attachedStage, objectRecord->mPath, property, timeCode, vec))
        {
            // get link scale:
            UsdGeomXformable xform = UsdGeomXformable(attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath));
            if (xform)
            {
                // attachment local pos is in scaled link-local coordinates, so unscale for PhysX:
                const pxr::GfTransform attachmentTrans = xform.ComputeLocalToWorldTransform(UsdTimeCode::Default());
                const pxr::GfVec3f scale(attachmentTrans.GetScale());
                pxAttachment->setRelativeOffset(PxVec3(vec[0] * scale[0], vec[1] * scale[1], vec[2] * scale[2]));

                // wake up articulation. Todo preist: Remove after SDK update where wake up is fixed.
                pxAttachment->getTendon()->getArticulation()->wakeUp();
            }
        }
    }

    return true;
}

bool omni::physx::updateTendonAttachmentLeafRestLength(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && pxAttachment->isLeaf() && intAttachment && isCorrectInstance(intAttachment->instanceName.GetString(), property, g_PhysxTendonLength, 10u))
    {
        float data;
        if (getFloatBounded(attachedStage, objectRecord->mPath, property, timeCode, data, -FLT_MAX, FLT_MAX))
        {
            if (data < 0.f)
            {
                pxAttachment->setRestLength(intAttachment->initLength);
            }
            else
            {
                pxAttachment->setRestLength(data);
            }

            // wake up articulation. Todo preist: Remove after SDK update where wake up is fixed.
            pxAttachment->getTendon()->getArticulation()->wakeUp();
        }
    }

    return true;
}

bool omni::physx::updateTendonAttachmentLeafLowLimit(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && pxAttachment->isLeaf() && intAttachment && isCorrectInstance(intAttachment->instanceName.GetString(), property, g_PhysxTendonLength, 10u))
    {
        float data;
        if (getFloatBounded(attachedStage, objectRecord->mPath, property, timeCode, data, -FLT_MAX, FLT_MAX))
        {
            PxArticulationTendonLimit limits = pxAttachment->getLimitParameters();

            if (data > limits.highLimit)
            {
                limits.highLimit = data;
            }

            limits.lowLimit = data;

            pxAttachment->setLimitParameters(limits);

            // wake up articulation
            pxAttachment->getTendon()->getArticulation()->wakeUp();
        }
    }

    return true;
}

bool omni::physx::updateTendonAttachmentLeafHighLimit(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && pxAttachment->isLeaf() && intAttachment && isCorrectInstance(intAttachment->instanceName.GetString(), property, g_PhysxTendonLength, 10u))
    {
        float data;
        if (getFloatBounded(attachedStage, objectRecord->mPath, property, timeCode, data, -FLT_MAX, FLT_MAX))
        {
            PxArticulationTendonLimit limits = pxAttachment->getLimitParameters();

            if (data < limits.lowLimit)
            {
                limits.lowLimit = data;
            }

            limits.highLimit = data;

            pxAttachment->setLimitParameters(limits);

            // wake up articulation
            pxAttachment->getTendon()->getArticulation()->wakeUp();
        }
    }

    return true;
}

// update fixed tendons
bool omni::physx::updateFixedTendonStiffness(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    if (intAxis && pxAxis && isCorrectInstance(intAxis->instanceName.GetString(), property, g_PhysxTendonLength, 9u))
    {
        const pxr::SdfPath basePath = objectRecord->mPath;
        const std::string enabledString = "physxTendon:" + intAxis->instanceName.GetString() + ":tendonEnabled";
        bool isEnabled;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, TfToken(enabledString), timeCode, isEnabled))
            return true;

        if (isEnabled)
        {
            float data;
            if (getFloatBounded(attachedStage, objectRecord->mPath, property, timeCode, data, 0.f, FLT_MAX))
            {
                PxArticulationFixedTendon* const tendon = pxAxis->getTendon();
                tendon->setStiffness(data);

                tendon->getArticulation()->wakeUp();
            }
        }
    }

    return true;
}

bool omni::physx::updateFixedTendonLimitStiffness(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    if (intAxis && pxAxis && isCorrectInstance(intAxis->instanceName.GetString(), property, g_PhysxTendonLength, 14u))
    {
        const pxr::SdfPath basePath = objectRecord->mPath;
        const std::string enabledString = "physxTendon:" + intAxis->instanceName.GetString() + ":tendonEnabled";
        bool isEnabled;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, TfToken(enabledString), timeCode, isEnabled))
            return true;

        if (isEnabled)
        {
            float data;
            if (getFloatBounded(attachedStage, objectRecord->mPath, property, timeCode, data, 0.f, FLT_MAX))
            {
                PxArticulationFixedTendon* const tendon = pxAxis->getTendon();
                tendon->setLimitStiffness(data);

                tendon->getArticulation()->wakeUp();
            }
        }
    }

    return true;
}

bool omni::physx::updateFixedTendonDamping(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    if (intAxis && pxAxis && isCorrectInstance(intAxis->instanceName.GetString(), property, g_PhysxTendonLength, 7u))
    {
        const pxr::SdfPath basePath = objectRecord->mPath;
        const std::string enabledString = "physxTendon:" + intAxis->instanceName.GetString() + ":tendonEnabled";
        bool isEnabled;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, TfToken(enabledString), timeCode, isEnabled))
            return true;

        if (isEnabled)
        {
            float data;
            if (getFloatBounded(attachedStage, objectRecord->mPath, property, timeCode, data, 0.f, FLT_MAX))
            {
                PxArticulationFixedTendon* const tendon = pxAxis->getTendon();
                tendon->setDamping(data);

                tendon->getArticulation()->wakeUp();
            }
        }
    }

    return true;
}

bool omni::physx::updateFixedTendonOffset(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    if (intAxis && pxAxis && isCorrectInstance(intAxis->instanceName.GetString(), property, g_PhysxTendonLength, 6u))
    {
        float data;
        if (getFloatBounded(attachedStage, objectRecord->mPath, property, timeCode, data, -FLT_MAX, FLT_MAX))
        {
            PxArticulationFixedTendon* const tendon = pxAxis->getTendon();
            tendon->setOffset(data);

            tendon->getArticulation()->wakeUp();
        }
    }

    return true;
}

bool omni::physx::updateFixedTendonRestLength(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    if (intAxis && pxAxis && isCorrectInstance(intAxis->instanceName.GetString(), property, g_PhysxTendonLength, 10u))
    {
        float data;
        if (getFloatBounded(attachedStage, objectRecord->mPath, property, timeCode, data, -FLT_MAX, FLT_MAX))
        {
            PxArticulationFixedTendon* const tendon = pxAxis->getTendon();
            tendon->setRestLength(data);

            tendon->getArticulation()->wakeUp();
        }
    }

    return true;
}

bool omni::physx::updateFixedTendonLowLimit(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    if (intAxis && pxAxis && isCorrectInstance(intAxis->instanceName.GetString(), property, g_PhysxTendonLength, 10u))
    {
        float data;
        if (getFloatBounded(attachedStage, objectRecord->mPath, property, timeCode, data, -FLT_MAX, FLT_MAX))
        {
            PxArticulationFixedTendon* const tendon = pxAxis->getTendon();
            PxArticulationTendonLimit limits = tendon->getLimitParameters();

            if (data > limits.highLimit)
            {
                limits.highLimit = data;
            }

            limits.lowLimit = data;

            tendon->setLimitParameters(limits);

            tendon->getArticulation()->wakeUp();
        }
    }

    return true;
}

bool omni::physx::updateFixedTendonHighLimit(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    if (intAxis && pxAxis && isCorrectInstance(intAxis->instanceName.GetString(), property, g_PhysxTendonLength, 10u))
    {
        float data;
        if (getFloatBounded(attachedStage, objectRecord->mPath, property, timeCode, data, -FLT_MAX, FLT_MAX))
        {
            PxArticulationFixedTendon* const tendon = pxAxis->getTendon();
            PxArticulationTendonLimit limits = tendon->getLimitParameters();

            if (data < limits.lowLimit)
            {
                limits.lowLimit = data;
            }

            limits.highLimit = data;

            tendon->setLimitParameters(limits);

            tendon->getArticulation()->wakeUp();
        }
    }

    return true;
}

bool omni::physx::updateFixedTendonEnabled(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    if (intAxis && pxAxis && isCorrectInstance(intAxis->instanceName.GetString(), property, g_PhysxTendonLength, 13u))
    {
        bool isEnabled;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, isEnabled))
            return true;

        PxArticulationFixedTendon* const tendon = pxAxis->getTendon();
        if (isEnabled)
        {
            const pxr::SdfPath basePath = objectRecord->mPath;
            const std::string stiffnessString = "physxTendon:" + intAxis->instanceName.GetString() + ":stiffness";
            const std::string limitStiffnessString = "physxTendon:" + intAxis->instanceName.GetString() + ":limitStiffness";
            const std::string dampingString = "physxTendon:" + intAxis->instanceName.GetString() + ":damping";

            float stiffness, limitStiffness, damping;
            if (getFloatBounded(attachedStage, objectRecord->mPath, TfToken(stiffnessString), timeCode, stiffness, 0.f, FLT_MAX))
                tendon->setStiffness(stiffness);

            if (getFloatBounded(attachedStage, objectRecord->mPath, TfToken(limitStiffnessString), timeCode, limitStiffness, 0.f, FLT_MAX))
                tendon->setLimitStiffness(limitStiffness);

            if (getFloatBounded(attachedStage, objectRecord->mPath, TfToken(dampingString), timeCode, damping, 0.f, FLT_MAX))
                tendon->setDamping(damping);
        }
        else
        {
            tendon->setStiffness(0.f);
            tendon->setLimitStiffness(0.f);
            tendon->setDamping(0.f);
        }

        // wake up articulation
        tendon->getArticulation()->wakeUp();
    }

    return true;
}

// WARNING: only works for joints with a single DOF (a.k.a revolute and prismatic joints)
bool omni::physx::updateTendonAxisSingleGearing(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    if (pxAxis && intAxis && isCorrectInstance(intAxis->instanceName.GetString(), property, g_PhysxTendonLength, 7u))
    {
        VtFloatArray temp;
        if (!getArrayValue(attachedStage, objectRecord->mPath, property, timeCode, temp) || temp.empty())
            return true;

        float setValue = temp.cdata()[0];

        PxArticulationJointReducedCoordinate* joint = static_cast<PxArticulationJointReducedCoordinate*>
            (pxAxis->getLink()->getInboundJoint());

        // convert data from deg based on joint / dof type:
        if (joint)
        {
            PxArticulationJointType::Enum jointType = joint->getJointType();

            if (jointType == PxArticulationJointType::eREVOLUTE || jointType == PxArticulationJointType::eREVOLUTE_UNWRAPPED)
            {
                if (setValue > static_cast<float>(GfDegreesToRadians(FLT_MAX)))
                {
                    setValue = FLT_MAX;
                }
                else if (setValue < static_cast<float>(GfDegreesToRadians(-FLT_MAX)))
                {
                    setValue = -FLT_MAX;
                }
                else
                {
                    // user sets coefficent to map from deg to tendon length.
                    // Therefore, in order to get the same tendon length when the joint angle is in radians, multiply by
                    // rad2deg
                    setValue = static_cast<float>(GfRadiansToDegrees(setValue));
                }
            }
        }

        float gearing = 0.0f;
        float forceCoefficient = 0.0f;
        PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
        pxAxis->getCoefficient(axis, gearing, forceCoefficient);
        pxAxis->setCoefficient(axis, setValue, forceCoefficient);
        pxAxis->getTendon()->getArticulation()->wakeUp();
    }

    return true;
}

// WARNING: only works for joints with a single DOF (a.k.a revolute and prismatic joints)
bool omni::physx::updateTendonAxisSingleForceCoefficient(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
                                                         const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    if (pxAxis && intAxis && isCorrectInstance(intAxis->instanceName.GetString(), property, g_PhysxTendonLength, 16u))
    {
        VtFloatArray temp;
        if (!getArrayValue(attachedStage, objectRecord->mPath, property, timeCode, temp) || temp.empty())
            return true;

        float setValue = temp.cdata()[0];
        float gearing = 0.0f;
        float forceCoefficient = 0.0f;
        PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
        pxAxis->getCoefficient(axis, gearing, forceCoefficient);
        pxAxis->setCoefficient(axis, gearing, setValue);
        pxAxis->getTendon()->getArticulation()->wakeUp();
    }

    return true;
}
