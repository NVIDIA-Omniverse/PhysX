// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PhysXPropertiesUpdate.h"

#include <PhysXTools.h>
#include <Setup.h>
#include <OmniPhysX.h>

#include <carb/logging/Log.h>

#include <common/foundation/MatrixTools.h>

#include <PxPhysicsAPI.h>


using namespace ::physx;
using namespace carb;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

//           123456789012
// length of physxTendon: attribute namespace plus colon:
static size_t g_PhysxTendonLength = 12u;

// InternalTendonAxis/InternalTendonAttachment::instanceName is unconditionally std::string
// now (TendonInstanceNameHandle, InternalScene.h); this pass-through is kept only to avoid
// touching every one of its call sites below.
static const std::string& tendonInstanceName(const std::string& name)
{
    return name;
}

// pxr-free: resolves `property`'s name through the source's interned token table
// rather than a materialized TfToken, since the caller only ever had a TokenId
// to begin with once the dispatch typedef retyped away from TfToken.
static bool isCorrectInstance(const std::string& instanceName, const usdparser::AttachedStage& attachedStage,
                              omni::physics::parse::TokenId property, const size_t first, const size_t last)
{
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return false;
    const std::string_view propertyName = source->tokenToString(property);
    return instanceName == propertyName.substr(first, propertyName.length() - last - first - 1);
}

// PhysXTools.h has no TokenId+ReadTime sibling of getFloatBounded; this is that sibling,
// built on the pxr-free getValue<float> overload, mirroring the header's logic.
static bool getFloatBounded(const usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey key,
                            omni::physics::parse::TokenId attributeName, omni::physics::parse::ReadTime timeCode,
                            float& outFloat, const float lowBound, const float upBound)
{
    float data = 0.0f;
    const bool result = getValue<float>(attachedStage, key, attributeName, timeCode, data);
    if (data > upBound)
        data = upBound;
    else if (data < lowBound)
        data = lowBound;
    outFloat = data;
    return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// update spatial tendons
bool omni::physx::updateSpatialTendonStiffness(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && intAttachment && isCorrectInstance(tendonInstanceName(intAttachment->instanceName), attachedStage, property, g_PhysxTendonLength, 9u))
    {
        const std::string enabledString = "physxTendon:" + tendonInstanceName(intAttachment->instanceName) + ":tendonEnabled";
        const omni::physics::parse::IPhysicsSource* enabledSource = attachedStage.getSource();
        bool isEnabled;
        if (!enabledSource ||
            !getValue<bool>(attachedStage, objectRecord->mKey, enabledSource->internToken(enabledString), timeCode, isEnabled))
            return true;

        if (isEnabled)
        {
            float data;
            if (getFloatBounded(attachedStage, objectRecord->mKey, property, timeCode, data, 0.f, FLT_MAX))
            {
                PxArticulationSpatialTendon* const tendon = pxAttachment->getTendon();
                tendon->setStiffness(data);

                tendon->getArticulation()->wakeUp();
            }
        }
    }

    return true;
}

bool omni::physx::updateSpatialTendonDamping(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && intAttachment && isCorrectInstance(tendonInstanceName(intAttachment->instanceName), attachedStage, property, g_PhysxTendonLength, 7u))
    {
        const std::string enabledString = "physxTendon:" + tendonInstanceName(intAttachment->instanceName) + ":tendonEnabled";
        const omni::physics::parse::IPhysicsSource* enabledSource = attachedStage.getSource();
        bool isEnabled;
        if (!enabledSource ||
            !getValue<bool>(attachedStage, objectRecord->mKey, enabledSource->internToken(enabledString), timeCode, isEnabled))
            return true;

        if (isEnabled)
        {
            float data;
            if (getFloatBounded(attachedStage, objectRecord->mKey, property, timeCode, data, 0.f, FLT_MAX))
            {
                PxArticulationSpatialTendon* const tendon = pxAttachment->getTendon();
                tendon->setDamping(data);

                tendon->getArticulation()->wakeUp();
            }
        }
    }

    return true;
}

bool omni::physx::updateSpatialTendonLimitStiffness(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && intAttachment && isCorrectInstance(tendonInstanceName(intAttachment->instanceName), attachedStage, property, g_PhysxTendonLength, 14u))
    {
        const std::string enabledString = "physxTendon:" + tendonInstanceName(intAttachment->instanceName) + ":tendonEnabled";
        const omni::physics::parse::IPhysicsSource* enabledSource = attachedStage.getSource();
        bool isEnabled;
        if (!enabledSource ||
            !getValue<bool>(attachedStage, objectRecord->mKey, enabledSource->internToken(enabledString), timeCode, isEnabled))
            return true;

        if (isEnabled)
        {
            float data;
            if (getFloatBounded(attachedStage, objectRecord->mKey, property, timeCode, data, 0.f, FLT_MAX))
            {
                PxArticulationSpatialTendon* const tendon = pxAttachment->getTendon();
                tendon->setLimitStiffness(data);

                tendon->getArticulation()->wakeUp();
            }
        }
    }

    return true;
}

bool omni::physx::updateSpatialTendonOffset(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && intAttachment && isCorrectInstance(tendonInstanceName(intAttachment->instanceName), attachedStage, property, g_PhysxTendonLength, 6u))
    {
        float data;
        if (getFloatBounded(attachedStage, objectRecord->mKey, property, timeCode, data, -FLT_MAX, FLT_MAX))
        {
            PxArticulationSpatialTendon* const tendon = pxAttachment->getTendon();
            tendon->setOffset(data);

            tendon->getArticulation()->wakeUp();
        }
    }

    return true;
}

bool omni::physx::updateSpatialTendonEnabled(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && intAttachment && isCorrectInstance(tendonInstanceName(intAttachment->instanceName), attachedStage, property, g_PhysxTendonLength, 13u))
    {
        bool isEnabled;
        if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, isEnabled))
            return true;

        PxArticulationSpatialTendon* const tendon = pxAttachment->getTendon();
        if (isEnabled)
        {
            const std::string stiffnessString = "physxTendon:" + tendonInstanceName(intAttachment->instanceName) + ":stiffness";
            const std::string limitStiffnessString = "physxTendon:" + tendonInstanceName(intAttachment->instanceName) + ":limitStiffness";
            const std::string dampingString = "physxTendon:" + tendonInstanceName(intAttachment->instanceName) + ":damping";

            const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
            float stiffness, limitStiffness, damping;
            if (source && getFloatBounded(attachedStage, objectRecord->mKey, source->internToken(stiffnessString), timeCode, stiffness, 0.f, FLT_MAX))
                tendon->setStiffness(stiffness);

            if (source && getFloatBounded(attachedStage, objectRecord->mKey, source->internToken(limitStiffnessString), timeCode, limitStiffness, 0.f, FLT_MAX))
                tendon->setLimitStiffness(limitStiffness);

            if (source && getFloatBounded(attachedStage, objectRecord->mKey, source->internToken(dampingString), timeCode, damping, 0.f, FLT_MAX))
                tendon->setDamping(damping);
        }
        else
        {
            tendon->setStiffness(0.f);
            tendon->setLimitStiffness(0.f);
            tendon->setDamping(0.f);
        }

        tendon->getArticulation()->wakeUp();
    }

    return true;
}

bool omni::physx::updateTendonAttachmentGearing(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && intAttachment && isCorrectInstance(tendonInstanceName(intAttachment->instanceName), attachedStage, property, g_PhysxTendonLength, 7u))
    {
        float data;
        if (getFloatBounded(attachedStage, objectRecord->mKey, property, timeCode, data, -FLT_MAX, FLT_MAX))
        {
            pxAttachment->setCoefficient(data);

            pxAttachment->getTendon()->getArticulation()->wakeUp();
        }
    }

    return true;
}

bool omni::physx::updateTendonAttachmentLocalPos(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && intAttachment && isCorrectInstance(tendonInstanceName(intAttachment->instanceName), attachedStage, property, g_PhysxTendonLength, 8u))
    {
        carb::Float3 vec;
        if (getValue(attachedStage, objectRecord->mKey, property, timeCode, vec))
        {
            // get link scale: attachment local pos is in scaled link-local
            // coordinates, so unscale for PhysX.
            const PxVec3 scale =
                omni::physx::getScale(getWorldTransform(attachedStage, objectRecord->mKey, omni::physics::parse::ReadTime::defaultTime()));
            pxAttachment->setRelativeOffset(toPhysX(vec).multiply(scale));

            // wake up articulation. Todo preist: Remove after SDK update where wake up is fixed.
            pxAttachment->getTendon()->getArticulation()->wakeUp();
        }
    }

    return true;
}

bool omni::physx::updateTendonAttachmentLeafRestLength(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && pxAttachment->isLeaf() && intAttachment && isCorrectInstance(tendonInstanceName(intAttachment->instanceName), attachedStage, property, g_PhysxTendonLength, 10u))
    {
        float data;
        if (getFloatBounded(attachedStage, objectRecord->mKey, property, timeCode, data, -FLT_MAX, FLT_MAX))
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
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && pxAttachment->isLeaf() && intAttachment && isCorrectInstance(tendonInstanceName(intAttachment->instanceName), attachedStage, property, g_PhysxTendonLength, 10u))
    {
        float data;
        if (getFloatBounded(attachedStage, objectRecord->mKey, property, timeCode, data, -FLT_MAX, FLT_MAX))
        {
            PxArticulationTendonLimit limits = pxAttachment->getLimitParameters();

            if (data > limits.highLimit)
            {
                limits.highLimit = data;
            }

            limits.lowLimit = data;

            pxAttachment->setLimitParameters(limits);

            pxAttachment->getTendon()->getArticulation()->wakeUp();
        }
    }

    return true;
}

bool omni::physx::updateTendonAttachmentLeafHighLimit(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTTendonAttachment, objectId);
    if (!objectRecord)
        return true;

    PxArticulationAttachment* pxAttachment = static_cast<PxArticulationAttachment*>(objectRecord->mPtr);
    InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord->mInternalPtr);
    if (pxAttachment && pxAttachment->isLeaf() && intAttachment && isCorrectInstance(tendonInstanceName(intAttachment->instanceName), attachedStage, property, g_PhysxTendonLength, 10u))
    {
        float data;
        if (getFloatBounded(attachedStage, objectRecord->mKey, property, timeCode, data, -FLT_MAX, FLT_MAX))
        {
            PxArticulationTendonLimit limits = pxAttachment->getLimitParameters();

            if (data < limits.lowLimit)
            {
                limits.lowLimit = data;
            }

            limits.highLimit = data;

            pxAttachment->setLimitParameters(limits);

            pxAttachment->getTendon()->getArticulation()->wakeUp();
        }
    }

    return true;
}

// update fixed tendons
bool omni::physx::updateFixedTendonStiffness(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    if (intAxis && pxAxis && isCorrectInstance(tendonInstanceName(intAxis->instanceName), attachedStage, property, g_PhysxTendonLength, 9u))
    {
        const std::string enabledString = "physxTendon:" + tendonInstanceName(intAxis->instanceName) + ":tendonEnabled";
        const omni::physics::parse::IPhysicsSource* enabledSource = attachedStage.getSource();
        bool isEnabled;
        if (!enabledSource ||
            !getValue<bool>(attachedStage, objectRecord->mKey, enabledSource->internToken(enabledString), timeCode, isEnabled))
            return true;

        if (isEnabled)
        {
            float data;
            if (getFloatBounded(attachedStage, objectRecord->mKey, property, timeCode, data, 0.f, FLT_MAX))
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
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    if (intAxis && pxAxis && isCorrectInstance(tendonInstanceName(intAxis->instanceName), attachedStage, property, g_PhysxTendonLength, 14u))
    {
        const std::string enabledString = "physxTendon:" + tendonInstanceName(intAxis->instanceName) + ":tendonEnabled";
        const omni::physics::parse::IPhysicsSource* enabledSource = attachedStage.getSource();
        bool isEnabled;
        if (!enabledSource ||
            !getValue<bool>(attachedStage, objectRecord->mKey, enabledSource->internToken(enabledString), timeCode, isEnabled))
            return true;

        if (isEnabled)
        {
            float data;
            if (getFloatBounded(attachedStage, objectRecord->mKey, property, timeCode, data, 0.f, FLT_MAX))
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
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    if (intAxis && pxAxis && isCorrectInstance(tendonInstanceName(intAxis->instanceName), attachedStage, property, g_PhysxTendonLength, 7u))
    {
        const std::string enabledString = "physxTendon:" + tendonInstanceName(intAxis->instanceName) + ":tendonEnabled";
        const omni::physics::parse::IPhysicsSource* enabledSource = attachedStage.getSource();
        bool isEnabled;
        if (!enabledSource ||
            !getValue<bool>(attachedStage, objectRecord->mKey, enabledSource->internToken(enabledString), timeCode, isEnabled))
            return true;

        if (isEnabled)
        {
            float data;
            if (getFloatBounded(attachedStage, objectRecord->mKey, property, timeCode, data, 0.f, FLT_MAX))
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
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    if (intAxis && pxAxis && isCorrectInstance(tendonInstanceName(intAxis->instanceName), attachedStage, property, g_PhysxTendonLength, 6u))
    {
        float data;
        if (getFloatBounded(attachedStage, objectRecord->mKey, property, timeCode, data, -FLT_MAX, FLT_MAX))
        {
            PxArticulationFixedTendon* const tendon = pxAxis->getTendon();
            tendon->setOffset(data);

            tendon->getArticulation()->wakeUp();
        }
    }

    return true;
}

bool omni::physx::updateFixedTendonRestLength(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    if (intAxis && pxAxis && isCorrectInstance(tendonInstanceName(intAxis->instanceName), attachedStage, property, g_PhysxTendonLength, 10u))
    {
        float data;
        if (getFloatBounded(attachedStage, objectRecord->mKey, property, timeCode, data, -FLT_MAX, FLT_MAX))
        {
            PxArticulationFixedTendon* const tendon = pxAxis->getTendon();
            tendon->setRestLength(data);

            tendon->getArticulation()->wakeUp();
        }
    }

    return true;
}

bool omni::physx::updateFixedTendonLowLimit(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    if (intAxis && pxAxis && isCorrectInstance(tendonInstanceName(intAxis->instanceName), attachedStage, property, g_PhysxTendonLength, 10u))
    {
        float data;
        if (getFloatBounded(attachedStage, objectRecord->mKey, property, timeCode, data, -FLT_MAX, FLT_MAX))
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
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    if (intAxis && pxAxis && isCorrectInstance(tendonInstanceName(intAxis->instanceName), attachedStage, property, g_PhysxTendonLength, 10u))
    {
        float data;
        if (getFloatBounded(attachedStage, objectRecord->mKey, property, timeCode, data, -FLT_MAX, FLT_MAX))
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
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    if (intAxis && pxAxis && isCorrectInstance(tendonInstanceName(intAxis->instanceName), attachedStage, property, g_PhysxTendonLength, 13u))
    {
        bool isEnabled;
        if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, isEnabled))
            return true;

        PxArticulationFixedTendon* const tendon = pxAxis->getTendon();
        if (isEnabled)
        {
            const std::string stiffnessString = "physxTendon:" + tendonInstanceName(intAxis->instanceName) + ":stiffness";
            const std::string limitStiffnessString = "physxTendon:" + tendonInstanceName(intAxis->instanceName) + ":limitStiffness";
            const std::string dampingString = "physxTendon:" + tendonInstanceName(intAxis->instanceName) + ":damping";

            const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
            float stiffness, limitStiffness, damping;
            if (source && getFloatBounded(attachedStage, objectRecord->mKey, source->internToken(stiffnessString), timeCode, stiffness, 0.f, FLT_MAX))
                tendon->setStiffness(stiffness);

            if (source && getFloatBounded(attachedStage, objectRecord->mKey, source->internToken(limitStiffnessString), timeCode, limitStiffness, 0.f, FLT_MAX))
                tendon->setLimitStiffness(limitStiffness);

            if (source && getFloatBounded(attachedStage, objectRecord->mKey, source->internToken(dampingString), timeCode, damping, 0.f, FLT_MAX))
                tendon->setDamping(damping);
        }
        else
        {
            tendon->setStiffness(0.f);
            tendon->setLimitStiffness(0.f);
            tendon->setDamping(0.f);
        }

        tendon->getArticulation()->wakeUp();
    }

    return true;
}

// WARNING: only works for joints with a single DOF (a.k.a revolute and prismatic joints)
bool omni::physx::updateTendonAxisSingleGearing(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    if (pxAxis && intAxis && isCorrectInstance(tendonInstanceName(intAxis->instanceName), attachedStage, property, g_PhysxTendonLength, 7u))
    {
        std::vector<float> temp;
        if (!getArrayValue(attachedStage, objectRecord->mKey, property, timeCode, temp) || temp.empty())
            return true;

        float setValue = temp[0];

        PxArticulationJointReducedCoordinate* joint = static_cast<PxArticulationJointReducedCoordinate*>
            (pxAxis->getLink()->getInboundJoint());

        // convert data from deg based on joint / dof type:
        if (joint)
        {
            PxArticulationJointType::Enum jointType = joint->getJointType();

            if (jointType == PxArticulationJointType::eREVOLUTE || jointType == PxArticulationJointType::eREVOLUTE_UNWRAPPED)
            {
                if (setValue > degToRad(FLT_MAX))
                {
                    setValue = FLT_MAX;
                }
                else if (setValue < degToRad(-FLT_MAX))
                {
                    setValue = -FLT_MAX;
                }
                else
                {
                    // user sets coefficent to map from deg to tendon length.
                    // Therefore, in order to get the same tendon length when the joint angle is in radians, multiply by
                    // rad2deg
                    setValue = radToDeg(setValue);
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
                                                         omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTFixedTendonAxis, objectId);
    if (!objectRecord)
        return true;

    PxArticulationTendonJoint* pxAxis = static_cast<PxArticulationTendonJoint*>(objectRecord->mPtr);
    InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord->mInternalPtr);
    if (pxAxis && intAxis && isCorrectInstance(tendonInstanceName(intAxis->instanceName), attachedStage, property, g_PhysxTendonLength, 16u))
    {
        std::vector<float> temp;
        if (!getArrayValue(attachedStage, objectRecord->mKey, property, timeCode, temp) || temp.empty())
            return true;

        float setValue = temp[0];
        float gearing = 0.0f;
        float forceCoefficient = 0.0f;
        PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
        pxAxis->getCoefficient(axis, gearing, forceCoefficient);
        pxAxis->setCoefficient(axis, gearing, setValue);
        pxAxis->getTendon()->getArticulation()->wakeUp();
    }

    return true;
}
