// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/logging/Log.h>

#include "LoadTools.h"

#include "MimicJoint.h"

#include <omni/physx/IPhysxSettings.h>
#include <OmniPhysX.h>

//See comment for gMimicJointNaturalFrequencyAttributeName and remove accordingly.
#include <omni/physx/PhysxTokens.h>

using namespace pxr;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{
namespace usdparser
{


static bool revoluteHasLimitSet(const pxr::UsdPrim& usdPrim)
{
    constexpr float sentinelLimit = 0.5e38f;

    UsdPhysicsRevoluteJoint revoluteJoint(usdPrim);

    float lower, upper;

    revoluteJoint.GetLowerLimitAttr().Get(&lower);
    revoluteJoint.GetUpperLimitAttr().Get(&upper);
    if (isfinite(lower) && isfinite(upper) && (lower > -sentinelLimit) && (upper < sentinelLimit))
        return true;
    else
        return false;
}

///
/// Verify that the joint type is supported and that the target degree of freedom is valid.
///
/// \param[in] usdPrim A prim that is of type UsdPhysicsJoint
/// \param[in] targetDegreeOfFreedom Token of the degree of freedom to work with (UsdPhysicsTokens->transX etc.)
/// \param[out] degreeOfFreedom JointAxis enum (::eRotX, ::eRotY or ::eRotZ) that matches targetDegreeOfFreedom
///             if the joint type is generic (D6 style). MimicJointDesc::eDEFAULT_AXIS for revolute or prismatic
///             joint. Undefined if the method does not succeed.
/// \return true on success, false if an error occured.
///
static bool checkJointDegreeOfFreedom(const pxr::UsdPrim& usdPrim, const TfToken& targetDegreeOfFreedom,
    int32_t& degreeOfFreedom)
{
    if (usdPrim.IsA<UsdPhysicsRevoluteJoint>())
    {
        degreeOfFreedom = MimicJointDesc::eDEFAULT_AXIS;

        if (revoluteHasLimitSet(usdPrim))
            return true;
        else
        {
            CARB_LOG_ERROR(
                "Usd Physics: the revolute joint at prim %s needs a finite limit set to be used by the mimic joint feature.\n",
                usdPrim.GetPath().GetText());

            return false;
        }
    }
    else if (usdPrim.IsA<UsdPhysicsPrismaticJoint>())
    {
        degreeOfFreedom = MimicJointDesc::eDEFAULT_AXIS;
        return true;
    }
    else if ((!usdPrim.IsA<UsdPhysicsSphericalJoint>()) &&
        (!usdPrim.IsA<UsdPhysicsDistanceJoint>()) &&
        (!usdPrim.IsA<UsdPhysicsFixedJoint>()) &&
        (!usdPrim.IsA<PhysxSchemaPhysxPhysicsGearJoint>()) &&
        (!usdPrim.IsA<PhysxSchemaPhysxPhysicsRackAndPinionJoint>()))
    {
        static const TfToken transAxisTokenList[] = { UsdPhysicsTokens->transX, UsdPhysicsTokens->transY, UsdPhysicsTokens->transZ };
        const uint32_t axisCount = sizeof(transAxisTokenList) / sizeof(transAxisTokenList[0]);

        uint32_t lockedTransAxisMap = 0;
        const uint32_t allTransAxisLocked = (1 << 0) | (1 << 1) | (1 << 2);

        for (uint32_t i = 0; i < axisCount; i++)
        {
            const UsdPhysicsLimitAPI limitAPI = UsdPhysicsLimitAPI::Get(usdPrim, transAxisTokenList[i]);
            if (limitAPI)
            {
                float lower, upper;
                limitAPI.GetLowAttr().Get(&lower);
                limitAPI.GetHighAttr().Get(&upper);

                if (lower > upper)
                    lockedTransAxisMap |= (1 << i);
            }
        }

        if (lockedTransAxisMap == allTransAxisLocked)
        {
            if ((targetDegreeOfFreedom == UsdPhysicsTokens->rotX) ||
                (targetDegreeOfFreedom == UsdPhysicsTokens->rotY) ||
                (targetDegreeOfFreedom == UsdPhysicsTokens->rotZ))
            {
                bool targetDegreeOfFreedomNotLocked = false;

                const UsdPhysicsLimitAPI limitAPI = UsdPhysicsLimitAPI::Get(usdPrim, targetDegreeOfFreedom);
                if (limitAPI)
                {
                    float lower, upper;
                    limitAPI.GetLowAttr().Get(&lower);
                    limitAPI.GetHighAttr().Get(&upper);

                    targetDegreeOfFreedomNotLocked = (lower < upper);
                }
                else
                {
                    // if the limit does not exist, then the degree of freedom is free
                    targetDegreeOfFreedomNotLocked = true;
                }

                if (targetDegreeOfFreedomNotLocked)
                {
                    if (targetDegreeOfFreedom == UsdPhysicsTokens->rotX)
                        degreeOfFreedom = eRotX;
                    else if (targetDegreeOfFreedom == UsdPhysicsTokens->rotY)
                        degreeOfFreedom = eRotY;
                    else
                    {
                        CARB_ASSERT(targetDegreeOfFreedom == UsdPhysicsTokens->rotZ);
                        degreeOfFreedom = eRotZ;
                    }

                    return true;
                }
                else
                {
                    CARB_LOG_ERROR(
                        "Usd Physics: the mimic joint feature needs a non locked degree of freedom. The joint "
                        "at prim %s has the axis \"%s\" locked\n.",
                        usdPrim.GetPath().GetText(), targetDegreeOfFreedom.GetText());

                    return false;
                }
            }
            else
            {
                CARB_LOG_ERROR(
                    "Usd Physics: mimic joint axis tokens have to be one of {rotX, rotY, rotZ}. Provided token: \"%s\"\n.",
                    targetDegreeOfFreedom.GetText());

                return false;
            }
        }
        else
        {
            CARB_LOG_ERROR(
                "Usd Physics: a generic joint needs all linear degrees of freedom locked to support the mimic joint feature. "
                "The joint at prim %s does not meet the requirement.\n",
                usdPrim.GetPath().GetText());

            return false;
        }
    }
    else
    {
        CARB_LOG_ERROR(
            "Usd Physics: the joint at prim %s has a type that is not supported by the mimic joint feature. "
            "Please refer to the USD documentation for a list of supported joint types.\n",
            usdPrim.GetPath().GetText());

        return false;
    }
}

void parseMimicJoints(const pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& usdPrim,
    std::vector<MimicJointDesc>& mimicJointDescList)
{
    CARB_ASSERT(usdPrim.IsA<UsdPhysicsJoint>());  // in the current code, this should only get called if the prim is a joint

    if (usdPrim.HasAPI<PhysxSchemaPhysxMimicJointAPI>())
    {
        UsdPhysicsJoint usdTargetJoint = UsdPhysicsJoint(usdPrim);
        bool excludeFromArticulation;
        usdTargetJoint.GetExcludeFromArticulationAttr().Get(&excludeFromArticulation);

        if (!excludeFromArticulation)
        {
            bool jointEnabled;
            usdTargetJoint.GetJointEnabledAttr().Get(&jointEnabled);

            if (jointEnabled)  // ignore disabled joints (without sending error message)
            {
                static const TfToken rotAxisTokenList[] = { UsdPhysicsTokens->rotX, UsdPhysicsTokens->rotY, UsdPhysicsTokens->rotZ };
                static const uint32_t rotAxisTokenCount = sizeof(rotAxisTokenList) / sizeof(rotAxisTokenList[0]);

                static const ObjectType objectTypeList[rotAxisTokenCount] = { eMimicJointRotX, eMimicJointRotY, eMimicJointRotZ };

                uint32_t appliedMimicJointAPICount = 0;

                for (uint32_t i = 0; i < rotAxisTokenCount; i++)
                {
                    const TfToken& axisToken = rotAxisTokenList[i];

                    const PhysxSchemaPhysxMimicJointAPI mimicJointAPI = PhysxSchemaPhysxMimicJointAPI::Get(usdPrim, axisToken);
                    if (mimicJointAPI)
                    {
                        MimicJointDesc desc;
                        desc.type = objectTypeList[i];

                        desc.mimicJointPath = usdPrim.GetPath();

                        mimicJointAPI.GetGearingAttr().Get(&desc.gearing);
                        mimicJointAPI.GetOffsetAttr().Get(&desc.offset);

                        float naturalFrequency = 0.0f;
                        if(usdPrim.HasAttribute(gMimicJointNaturalFrequencyAttributeNameToken[i]))
                        {
                            usdPrim.GetAttribute(gMimicJointNaturalFrequencyAttributeNameToken[i]).Get(&naturalFrequency);
                        }
                        desc.naturalFrequency = naturalFrequency;

                        float dampingRatio = 0.0f;
                        if(usdPrim.HasAttribute(gMimicJointDampingRatioAttributeNameToken[i]))
                        {
                            usdPrim.GetAttribute(gMimicJointDampingRatioAttributeNameToken[i]).Get(&dampingRatio);
                        }
                        desc.dampingRatio = dampingRatio;           

                        TfToken refJointAxis;
                        mimicJointAPI.GetReferenceJointAxisAttr().Get(&refJointAxis);

                        if ((refJointAxis == UsdPhysicsTokens->rotX) ||
                            (refJointAxis == UsdPhysicsTokens->rotY) ||
                            (refJointAxis == UsdPhysicsTokens->rotZ))
                        {
                            if (checkJointDegreeOfFreedom(usdPrim, axisToken, desc.mimicJointAxis))
                            {
                                if ((desc.mimicJointAxis != MimicJointDesc::eDEFAULT_AXIS) || (appliedMimicJointAPICount == 0))
                                {
                                    UsdRelationship rel = mimicJointAPI.GetReferenceJointRel();
                                    if (rel.HasAuthoredTargets())
                                    {
                                        SdfPathVector paths;
                                        rel.GetTargets(&paths);
                                        if (paths.size() == 1)
                                        {
                                            const SdfPath& path = paths[0];

                                            desc.referenceJointPath = path;

                                            const UsdPrim refJointPrim = stage->GetPrimAtPath(path);

                                            if (refJointPrim)
                                            {
                                                if (refJointPrim.IsA<UsdPhysicsJoint>())
                                                {
                                                    UsdPhysicsJoint usdReferenceJoint = UsdPhysicsJoint(refJointPrim);
                                                    usdReferenceJoint.GetExcludeFromArticulationAttr().Get(&excludeFromArticulation);

                                                    if (!excludeFromArticulation)
                                                    {
                                                        usdReferenceJoint.GetJointEnabledAttr().Get(&jointEnabled);

                                                        if (jointEnabled)  // ignore disabled joints (without sending error message)
                                                        {
                                                            if (checkJointDegreeOfFreedom(refJointPrim, refJointAxis, desc.referenceJointAxis))
                                                            {
                                                                if ((path != usdPrim.GetPath()) || (desc.referenceJointAxis != desc.mimicJointAxis))
                                                                {
                                                                    mimicJointDescList.emplace_back(desc);
                                                                    appliedMimicJointAPICount++;
                                                                }
                                                                else
                                                                {
                                                                    CARB_LOG_ERROR("Usd Physics: PhysxMimicJointAPI at %s has same joint for mimic and reference. The axes to operate on "
                                                                        "must not be the same in this case.\n",
                                                                        usdPrim.GetPath().GetText());
                                                                }
                                                            }
                                                        }
                                                    }
                                                    else
                                                    {
                                                        CARB_LOG_ERROR("Usd Physics: PhysxMimicJointAPI is only supported on articulation joints. The mimic joint at %s points to reference "
                                                            "joint %s which has \"excludeFromArticulation\" set to true.\n",
                                                            usdPrim.GetPath().GetText(), path.GetText());
                                                    }
                                                }
                                                else
                                                {
                                                    CARB_LOG_ERROR("Usd Physics: PhysxMimicJointAPI at %s points to prim %s that is not a UsdPhysicsJoint (see attribute \"referenceJoint\").\n",
                                                        usdPrim.GetPath().GetText(), path.GetText());
                                                }
                                            }
                                            else
                                            {
                                                CARB_LOG_ERROR("Usd Physics: PhysxMimicJointAPI at %s points to a non existing prim at %s in attribute \"referenceJoint\".\n",
                                                    usdPrim.GetPath().GetText(), path.GetText());
                                            }
                                        }
                                        else
                                        {
                                            CARB_LOG_ERROR("Usd Physics: PhysxMimicJointAPI at %s must have exactly 1 \"referenceJoint\" relationship defined.\n",
                                                usdPrim.GetPath().GetText());
                                        }
                                    }
                                    else
                                    {
                                        CARB_LOG_ERROR("Usd Physics: PhysxMimicJointAPI at %s must have exactly 1 \"referenceJoint\" relationship defined.\n",
                                            usdPrim.GetPath().GetText());
                                    }
                                }
                                else
                                {
                                    CARB_LOG_ERROR("Usd Physics: PhysxMimicJointAPI at %s: prismatic and revolute joints can only have one PhysxMimicJointAPI "
                                        "instance applied.\n",
                                        usdPrim.GetPath().GetText());
                                }
                            }
                        }
                        else
                        {
                            CARB_LOG_ERROR("Usd Physics: PhysxMimicJointAPI at %s has unsupported token \"%s\" in attribute \"referenceJointAxis\". "
                                "Supported tokens are: {rotX, rotY, rotZ}.\n",
                                usdPrim.GetPath().GetText(), refJointAxis.GetText());
                        }
                    }
                }
            }
        }
        else
        {
            CARB_LOG_ERROR("Usd Physics: PhysxMimicJointAPI is only supported on articulation joints. The joint at %s has \"excludeFromArticulation\" "
                "set to true.\n",
                usdPrim.GetPath().GetText());
        }
    }
}

static ObjectType getObjectType(SchemaAPIFlag::Enum schemaAPIFlag)
{
    if (schemaAPIFlag == SchemaAPIFlag::eMimicJointRotXAPI)
        return eMimicJointRotX;
    else if (schemaAPIFlag == SchemaAPIFlag::eMimicJointRotYAPI)
        return eMimicJointRotY;
    else
    {
        CARB_ASSERT(schemaAPIFlag == SchemaAPIFlag::eMimicJointRotZAPI);
        return eMimicJointRotZ;
    }
}

static SchemaAPIFlag::Enum getSchemaAPIFlag(ObjectType objectType)
{
    if (objectType == eMimicJointRotX)
        return SchemaAPIFlag::eMimicJointRotXAPI;
    else if (objectType == eMimicJointRotY)
        return SchemaAPIFlag::eMimicJointRotYAPI;
    else
    {
        CARB_ASSERT(objectType == eMimicJointRotZ);
        return SchemaAPIFlag::eMimicJointRotZAPI;
    }
}

ObjectId createMimicJoint(AttachedStage& attachedStage, MimicJointDesc& desc)
{
    ObjectDb* objectDb = attachedStage.getObjectDatabase();

    ObjectId mimicJointId = objectDb->findEntry(desc.mimicJointPath, eArticulationJoint);
    if (mimicJointId != kInvalidObjectId)
    {
        desc.mimicJointId = mimicJointId;

        ObjectId referenceJointId = objectDb->findEntry(desc.referenceJointPath, eArticulationJoint);
        if (referenceJointId != kInvalidObjectId)
        {
            desc.referenceJointId = referenceJointId;

            PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();
            const ObjectId id = physInt->createObject(attachedStage, desc.mimicJointPath, desc);

            if (id != kInvalidObjectId)
            {
                objectDb->findOrCreateEntry(desc.mimicJointPath, desc.type, id);

                SchemaAPIFlag::Enum schemaAPIFlag = getSchemaAPIFlag(desc.type);
                objectDb->addSchemaAPI(desc.mimicJointPath, schemaAPIFlag);
            }

            return id;
        }
        else if (OmniPhysX::getInstance().getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) == nullptr)
        {
            // scristiano: if in forced parsing single scene mode, the joints may have not been created
            CARB_LOG_ERROR("Usd Physics: failed to find internal joint object for reference joint at prim "
                "%s for PhysxMimicJointAPI at %s. Please ensure that the prim is a supported joint type and "
                "is part of an articulation.\n",
                desc.referenceJointPath.GetText(), desc.mimicJointPath.GetText());
        }
    }
    else if (OmniPhysX::getInstance().getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) == nullptr)
    {
        // scristiano: if in forced parsing single scene mode, the joints may have not been created
        CARB_LOG_ERROR("Usd Physics: failed to find internal joint object for PhysxMimicJointAPI at %s. "
            "Please ensure that the prim is a supported joint type and is part of an articulation.\n",
            desc.mimicJointPath.GetText());
    }

    return kInvalidObjectId;
}

void releaseMimicJoint(AttachedStage& attachedStage, const pxr::SdfPath& path,
    SchemaAPIFlag::Enum schemaAPIFlag)
{
    ObjectType type = getObjectType(schemaAPIFlag);

    ObjectDb* objectDb = attachedStage.getObjectDatabase();

    ObjectId mimicJointId = objectDb->findEntry(path, type);
    if (mimicJointId != kInvalidObjectId)
    {
        PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();

        physInt->releaseObject(attachedStage, path, mimicJointId);

        objectDb->removeSchemaAPI(path, schemaAPIFlag);

        objectDb->removeEntry(path, type, mimicJointId);
    }
}


} // namespace usdparser
} // namespace physx
} // namespace omni
