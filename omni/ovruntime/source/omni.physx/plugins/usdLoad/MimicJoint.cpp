// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/logging/Log.h>

#include "LoadTools.h"

#include "MimicJoint.h"
#include "NewtonCompat.h"

#include <omni/physx/IPhysxSettings.h>
#include <OmniPhysX.h>

using namespace PXR_NS;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{
namespace usdparser
{


static bool revoluteHasLimitSet(const PXR_NS::UsdPrim& usdPrim)
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
static bool checkJointDegreeOfFreedom(const PXR_NS::UsdPrim& usdPrim, const TfToken& targetDegreeOfFreedom,
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

void parseMimicJoints(const PXR_NS::UsdStageWeakPtr stage, const PXR_NS::UsdPrim& usdPrim,
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
                        mimicJointAPI.GetNaturalFrequencyAttr().Get(&desc.naturalFrequency);
                        mimicJointAPI.GetDampingRatioAttr().Get(&desc.dampingRatio);

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

// Verify that the joint at usdPrim is a single-DOF joint supported by NewtonMimicAPI.
// Multi-DOF joints are rejected with an error message.
static bool checkNewtonMimicSingleDofJoint(const PXR_NS::UsdPrim& usdPrim, const char* role)
{
    if (usdPrim.IsA<UsdPhysicsRevoluteJoint>())
    {
        if (revoluteHasLimitSet(usdPrim))
            return true;

        CARB_LOG_ERROR(
            "Usd Physics: NewtonMimicAPI %s joint at %s is a revolute joint without a finite limit set. "
            "A finite limit is required.\n",
            role, usdPrim.GetPath().GetText());
        return false;
    }
    else if (usdPrim.IsA<UsdPhysicsPrismaticJoint>())
    {
        return true;
    }
    else
    {
        CARB_LOG_ERROR(
            "Usd Physics: NewtonMimicAPI is only supported on single-DOF joints (PhysicsRevoluteJoint with a "
            "finite limit or PhysicsPrismaticJoint). The %s joint at %s is not supported.\n",
            role, usdPrim.GetPath().GetText());
        return false;
    }
}

void parseNewtonMimicJoints(const PXR_NS::UsdStageWeakPtr stage, const PXR_NS::UsdPrim& usdPrim,
    std::vector<MimicJointDesc>& mimicJointDescList)
{
    CARB_ASSERT(usdPrim.IsA<UsdPhysicsJoint>());

    if (!usdPrim.HasAPI(NewtonSchemaTokens->NewtonMimicAPI))
        return;

    UsdPhysicsJoint usdTargetJoint(usdPrim);

    bool excludeFromArticulation;
    usdTargetJoint.GetExcludeFromArticulationAttr().Get(&excludeFromArticulation);
    if (excludeFromArticulation)
    {
        CARB_LOG_ERROR(
            "Usd Physics: NewtonMimicAPI is only supported on articulation joints. The joint at %s has "
            "\"excludeFromArticulation\" set to true.\n",
            usdPrim.GetPath().GetText());
        return;
    }

    bool jointEnabled;
    usdTargetJoint.GetJointEnabledAttr().Get(&jointEnabled);
    if (!jointEnabled)
        return;  // ignore disabled joints (without sending error message)

    bool mimicEnabled = true;
    if (UsdAttribute enabledAttr = usdPrim.GetAttribute(NewtonSchemaTokens->newtonMimicEnabled))
        enabledAttr.Get(&mimicEnabled);
    if (!mimicEnabled)
        return;

    if (!checkNewtonMimicSingleDofJoint(usdPrim, "follower"))
        return;

    UsdRelationship rel = usdPrim.GetRelationship(NewtonSchemaTokens->newtonMimicJoint);
    if (!rel || !rel.HasAuthoredTargets())
    {
        CARB_LOG_ERROR(
            "Usd Physics: NewtonMimicAPI at %s must have exactly 1 \"newton:mimicJoint\" relationship defined.\n",
            usdPrim.GetPath().GetText());
        return;
    }

    SdfPathVector paths;
    rel.GetTargets(&paths);
    if (paths.size() != 1)
    {
        CARB_LOG_ERROR(
            "Usd Physics: NewtonMimicAPI at %s must have exactly 1 \"newton:mimicJoint\" relationship defined.\n",
            usdPrim.GetPath().GetText());
        return;
    }
    const SdfPath& refPath = paths[0];

    if (refPath == usdPrim.GetPath())
    {
        CARB_LOG_ERROR(
            "Usd Physics: NewtonMimicAPI at %s has the same joint for follower and leader, which is not allowed.\n",
            usdPrim.GetPath().GetText());
        return;
    }

    const UsdPrim refJointPrim = stage->GetPrimAtPath(refPath);
    if (!refJointPrim)
    {
        CARB_LOG_ERROR(
            "Usd Physics: NewtonMimicAPI at %s points to a non existing prim at %s in \"newton:mimicJoint\".\n",
            usdPrim.GetPath().GetText(), refPath.GetText());
        return;
    }
    if (!refJointPrim.IsA<UsdPhysicsJoint>())
    {
        CARB_LOG_ERROR(
            "Usd Physics: NewtonMimicAPI at %s points to prim %s that is not a UsdPhysicsJoint "
            "(see \"newton:mimicJoint\").\n",
            usdPrim.GetPath().GetText(), refPath.GetText());
        return;
    }

    UsdPhysicsJoint usdReferenceJoint(refJointPrim);
    usdReferenceJoint.GetExcludeFromArticulationAttr().Get(&excludeFromArticulation);
    if (excludeFromArticulation)
    {
        CARB_LOG_ERROR(
            "Usd Physics: NewtonMimicAPI is only supported on articulation joints. The mimic joint at %s points "
            "to reference joint %s which has \"excludeFromArticulation\" set to true.\n",
            usdPrim.GetPath().GetText(), refPath.GetText());
        return;
    }
    usdReferenceJoint.GetJointEnabledAttr().Get(&jointEnabled);
    if (!jointEnabled)
        return;  // ignore disabled joints (without sending error message)

    if (!checkNewtonMimicSingleDofJoint(refJointPrim, "leader"))
        return;

    float coef0 = 0.0f;
    float coef1 = 1.0f;
    if (UsdAttribute attr = usdPrim.GetAttribute(NewtonSchemaTokens->newtonMimicCoef0))
        attr.Get(&coef0);
    if (UsdAttribute attr = usdPrim.GetAttribute(NewtonSchemaTokens->newtonMimicCoef1))
        attr.Get(&coef1);

    MimicJointDesc desc;
    desc.type = eNewtonMimicJoint;
    desc.mimicJointPath = usdPrim.GetPath();
    desc.mimicJointAxis = MimicJointDesc::eDEFAULT_AXIS;
    desc.referenceJointPath = refPath;
    desc.referenceJointAxis = MimicJointDesc::eDEFAULT_AXIS;
    // Newton:  joint0 = coef0 + coef1 * joint1
    // PhysX:   joint0 + gearing * joint1 + offset = 0
    desc.gearing = -coef1;
    desc.offset = -coef0;
    desc.naturalFrequency = 0.0f;
    desc.dampingRatio = 0.0f;

    mimicJointDescList.emplace_back(desc);
}

static ObjectType getObjectType(SchemaAPIFlag::Enum schemaAPIFlag)
{
    if (schemaAPIFlag == SchemaAPIFlag::eMimicJointRotXAPI)
        return eMimicJointRotX;
    else if (schemaAPIFlag == SchemaAPIFlag::eMimicJointRotYAPI)
        return eMimicJointRotY;
    else if (schemaAPIFlag == SchemaAPIFlag::eMimicJointRotZAPI)
        return eMimicJointRotZ;
    else if (schemaAPIFlag == SchemaAPIFlag::eNewtonMimicAPI)
        return eNewtonMimicJoint;
    else
    {
        CARB_LOG_WARN("Usd Physics: unexpected SchemaAPIFlag %d passed to mimic joint getObjectType; "
            "defaulting to eNewtonMimicJoint.", static_cast<int>(schemaAPIFlag));
        return eNewtonMimicJoint;
    }
}

static SchemaAPIFlag::Enum getSchemaAPIFlag(ObjectType objectType)
{
    if (objectType == eMimicJointRotX)
        return SchemaAPIFlag::eMimicJointRotXAPI;
    else if (objectType == eMimicJointRotY)
        return SchemaAPIFlag::eMimicJointRotYAPI;
    else if (objectType == eMimicJointRotZ)
        return SchemaAPIFlag::eMimicJointRotZAPI;
    else if (objectType == eNewtonMimicJoint)
        return SchemaAPIFlag::eNewtonMimicAPI;
    else
    {
        CARB_LOG_WARN("Usd Physics: unexpected ObjectType %d passed to mimic joint getSchemaAPIFlag; "
            "defaulting to eNewtonMimicAPI.", static_cast<int>(objectType));
        return SchemaAPIFlag::eNewtonMimicAPI;
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

void releaseMimicJoint(AttachedStage& attachedStage, const PXR_NS::SdfPath& path,
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
