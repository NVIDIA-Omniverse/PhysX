// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/logging/Log.h>
#include <common/foundation/Allocator.h>

#include <ChangeRegister.h>
#include <propertiesUpdate/PhysXPropertiesUpdate.h>

#include "LoadTools.h"
#include "LoadUsd.h"
#include "FixedTendon.h"
#include "AttributeHelpers.h"


using namespace pxr;

namespace omni
{
namespace physx
{
namespace usdparser
{
    static const TfToken g_tendonAxis("PhysxTendonAxisAPI:");
    static const TfToken g_tendonRootAxis("PhysxTendonAxisRootAPI:");

    void parseFixedTendon(AttachedStage& attachedStage,
        const PhysxSchemaPhysxTendonAxisRootAPI& rootApi, PhysxTendonFixedDesc* desc)
    {
        // setup change listeners
        registerFixedTendonChangeParams(attachedStage, rootApi.GetName().GetString());

        // basic root attributes
        // use helper to enforce value ranges & enable time sampling
        getAttribute(desc->stiffness, rootApi.GetStiffnessAttr(), 0.0f, FLT_MAX, updateFixedTendonStiffness);
        getAttribute(desc->limitStiffness, rootApi.GetLimitStiffnessAttr(), 0.0f, FLT_MAX, updateFixedTendonLimitStiffness);
        getAttribute(desc->damping, rootApi.GetDampingAttr(), 0.0f, FLT_MAX, updateFixedTendonDamping);
        getAttribute(desc->offset, rootApi.GetOffsetAttr(), -FLT_MAX, FLT_MAX, updateFixedTendonOffset);
        getBoolAttribute(desc->isEnabled, rootApi.GetTendonEnabledAttr(), updateFixedTendonEnabled);

        // fixed tendon root attributes
        getAttribute(desc->restLength, rootApi.GetRestLengthAttr(), -FLT_MAX, FLT_MAX, updateFixedTendonRestLength); // allow negative value
        getAttribute(desc->lowLimit, rootApi.GetLowerLimitAttr(), -FLT_MAX, FLT_MAX, updateFixedTendonLowLimit);
        getAttribute(desc->highLimit, rootApi.GetUpperLimitAttr(), desc->lowLimit, FLT_MAX, updateFixedTendonHighLimit);

        if (desc->limitStiffness != 0.f && desc->lowLimit == -FLT_MAX && desc->highLimit == FLT_MAX)
        {
            CARB_LOG_WARN("The fixed tendon at %s has a positive limit stiffness but no limits set!", rootApi.GetPath().GetText());
        }

        desc->instanceToken = rootApi.GetName();
        desc->jointPath = rootApi.GetPath();
        desc->rootAxis = nullptr;
    }

    // WARNING: currently only support revolute and prismatic joints
    void parseAxes(AttachedStage& attachedStage, const PhysxSchemaPhysxTendonAxisAPI& axisApi,
        const omni::physics::schema::JointDesc* jointDesc, PhysxTendonAxisDesc* desc)
    {
        const pxr::UsdAttribute& gearingAttr = axisApi.GetGearingAttr();
        {
            const pxr::UsdAttribute& forceCoefficientAttr = axisApi.GetForceCoefficientAttr();
            VtFloatArray temp;
            forceCoefficientAttr.Get(&temp);
            if(!temp.empty())
                desc->forceCoefficients[0] = temp.cdata()[0];
        }
        if (axisApi.GetPrim().IsA<UsdPhysicsRevoluteJoint>())
        {
            VtFloatArray temp;
            gearingAttr.Get(&temp);
            float setValue = 1.0f;
            if(!temp.empty())
                setValue = temp.cdata()[0];
            if(setValue > static_cast<float>(GfDegreesToRadians(FLT_MAX)))
            {
                setValue = FLT_MAX;
            }
            else if(setValue < static_cast<float>(GfDegreesToRadians(-FLT_MAX)))
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

            desc->gearings[0] = setValue;
            desc->axes[0] = JointAxis::eRotX; // will always be RotX due to articulation parser

            if (gearingAttr.ValueMightBeTimeVarying())
            {
                attachedStage.registerTimeSampledAttribute(gearingAttr.GetPath(), updateTendonAxisSingleGearing);
            }
        }
        else if (axisApi.GetPrim().IsA<UsdPhysicsPrismaticJoint>())
        {
            VtFloatArray temp;
            gearingAttr.Get(&temp);
            if (!temp.empty())
            {
                desc->gearings[0] = temp.cdata()[0];
            }
            else
            {
                desc->gearings[0] = 1.0f;
            }

            desc->axes[0] = JointAxis::eTransX; // will always be TransX due to articulation parser

            if (gearingAttr.GetNumTimeSamples() > 1)
            {
                attachedStage.registerTimeSampledAttribute(gearingAttr.GetPath(), updateTendonAxisSingleGearing);
            }
        }
        /*else if (axisApi.GetPrim().IsA<UsdPhysicsFixedJoint>())
        {
            desc->gearings[0] = 0.0f;
            desc->axes[0] = JointAxis::eRotX;  // does not matter
        }
        else if (usdPrim.GetParent().IsA<UsdPhysicsSphericalJoint>())
        {
            TfToken axis = pxr::UsdPhysicsTokens.Get()->none;
            tendonJointPrim.GetAxisAttr().Get(&axis);

            if (axis == PhysxSchemaTokens.Get()->rotX)
                desc->axis = JointAxis::eRotX;
            else if (axis == PhysxSchemaTokens.Get()->rotY)
                desc->axis = JointAxis::eRotY;
            else if (axis == PhysxSchemaTokens.Get()->rotZ)
                desc->axis = JointAxis::eRotZ;
        }*/
        else
        {
            CARB_LOG_WARN("Tendon axis at %s applied to unsupported joint type. Only revolute and prismatic are currently supported.", axisApi.GetPath().GetText());
        }

        // setup change listener and allow time-sampled values
        registerTendonAxisChangeParam(attachedStage, axisApi.GetName().GetString());

        // get topology information (hierarchy not inferred at this point)
        desc->link0 = jointDesc->rel0;
        desc->link1 = jointDesc->rel1;

        desc->instanceToken = axisApi.GetName();
        desc->jointPath = axisApi.GetPath();
    }

    bool isNoDuplicate(const PhysxSchemaPhysxTendonAxisAPI& api, TfToken::Set& instanceNames)
    {
        if (instanceNames.insert(api.GetName()).second)
        {
            return true;
        }

        CARB_LOG_ERROR("More than one tendon axis instance with name %s was applied at joint %s.",
                       api.GetName().GetText(), api.GetPath().GetText());
        return false;
    }

    void parseTendonAxes(AttachedStage& attachedStage, UsdPrim prim, const omni::physics::schema::JointDesc* jointDesc,
        TendonAxisMap& tendonAxes, FixedTendonVector& fixedTendons)
    {
        // assert preconditions
        CARB_ASSERT(prim.HasAPI<PhysxSchemaPhysxTendonAxisAPI>());
        CARB_ASSERT(prim.IsA<UsdPhysicsJoint>());

        const UsdStageWeakPtr& stage = prim.GetStage();

        // loop over applied API schemas
        TfTokenVector appliedSchemas = prim.GetPrimTypeInfo().GetAppliedAPISchemas();
        const size_t& axisLen = g_tendonAxis.GetString().length();
        const size_t& rootLen = g_tendonRootAxis.GetString().length();
        TfToken::Set axisInstanceNames;
        for (auto token : appliedSchemas)
        {
            if (token.GetString().length() > rootLen &&
                token.GetString().substr(0, rootLen) == g_tendonRootAxis.GetString())
            {
                // fixed tendon root axis found
                PhysxSchemaPhysxTendonAxisRootAPI rootApi =
                    PhysxSchemaPhysxTendonAxisRootAPI::Get(prim, TfToken(token.GetString().substr(rootLen)));
                if (rootApi && isNoDuplicate(PhysxSchemaPhysxTendonAxisAPI(rootApi, rootApi.GetName()), axisInstanceNames))
                {
                    // parse both the axis and the general fixed tendon info
                    std::shared_ptr<PhysxTendonAxisDesc> axisDesc(ICE_PLACEMENT_NEW(PhysxTendonAxisDesc),
                        [](PhysxTendonAxisDesc* p) { ICE_FREE(p); }); // pass correct destructor into shared ptr
                    parseAxes(attachedStage, PhysxSchemaPhysxTendonAxisAPI(rootApi, rootApi.GetName()), jointDesc, axisDesc.get());
                    tendonAxes[axisDesc->link0].push_back(axisDesc);
                    tendonAxes[axisDesc->link1].push_back(axisDesc);
                    tendonAxes[axisDesc->jointPath].push_back(axisDesc);

                    // use shared ptr (not unique) to benefit from easier deleter initialization
                    // shared ptr okay because we do not depend on a single owner anywhere
                    std::shared_ptr<PhysxTendonFixedDesc> tendonDesc(ICE_PLACEMENT_NEW(PhysxTendonFixedDesc),
                        [](PhysxTendonFixedDesc* p) { ICE_FREE(p) }); // pass correct destructor into unique ptr
                    parseFixedTendon(attachedStage, rootApi, tendonDesc.get());
                    tendonDesc->rootAxis = axisDesc.get();
                    fixedTendons.push_back(tendonDesc);
                }
            }
            else if (token.GetString().length() > axisLen &&
                     token.GetString().substr(0, axisLen) == g_tendonAxis.GetString())
            {
                // standard tendon axis found
                PhysxSchemaPhysxTendonAxisAPI axisApi =
                    PhysxSchemaPhysxTendonAxisAPI::Get(prim, TfToken(token.GetString().substr(axisLen)));
                if (axisApi && isNoDuplicate(axisApi, axisInstanceNames))
                {
                    std::shared_ptr<PhysxTendonAxisDesc> axisDesc(ICE_PLACEMENT_NEW(PhysxTendonAxisDesc),
                        [](PhysxTendonAxisDesc* p) { ICE_FREE(p); }); // pass correct destructor into shared ptr
                    parseAxes(attachedStage, axisApi, jointDesc, axisDesc.get());
                    tendonAxes[axisDesc->link0].push_back(axisDesc);
                    tendonAxes[axisDesc->link1].push_back(axisDesc);
                    tendonAxes[axisDesc->jointPath].push_back(axisDesc);
                }
            }
        }
    }

    void createFixedTendonAxesRecursive(AttachedStage& attachedStage, const ObjectId& parentAxisId,
        const pxr::TfToken& instanceToken, const pxr::SdfPath& currLinkPath, TendonAxisMap& tendonAxes)
    {
        for (TendonAxisMap::mapped_type::const_reference AxisRef : tendonAxes[currLinkPath])
        {
            if (AxisRef->wasVisited)
            {
                continue;
            }

            if (AxisRef->instanceToken == instanceToken)
            {
                // add parent Axis object Id to description to save runtime when retrieving it during creation
                AxisRef->parentAxisId = parentAxisId;
                // create tendon axis
                const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, AxisRef->jointPath, *AxisRef);

                if (id == kInvalidObjectId)
                {
                    // User has already been warned in createObject();
                    continue;
                }

                attachedStage.getObjectDatabase()->findOrCreateEntry(AxisRef->jointPath, eTendonAxis, id);
                AxisRef->wasVisited = true;

                // infer directionality and continue traversal in right direction
                if (currLinkPath == AxisRef->link0)
                {
                    createFixedTendonAxesRecursive(attachedStage, id, instanceToken, AxisRef->link1, tendonAxes);
                }
                else
                {
                    createFixedTendonAxesRecursive(attachedStage, id, instanceToken, AxisRef->link0, tendonAxes);
                }
            }
        }
    }

    void createFixedTendons(AttachedStage& attachedStage, TendonAxisMap& tendonAxes, FixedTendonVector& fixedTendons)
    {        
        ObjectDb* objDb = attachedStage.getObjectDatabase();

        // PRECONDITION: Root axis must be the common ancestor to all other tendon axes with respect to articulation hierarchy
        for (FixedTendonVector::reference tendonRef : fixedTendons)
        {
            const SdfPath rootJointPath = tendonRef->rootAxis->jointPath;

            // create tendon (also creates dummy axis)
            const ObjectId tendonRootId = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, rootJointPath, *tendonRef);
            objDb->findOrCreateEntry(rootJointPath, eTendonFixed, tendonRootId);

            if (tendonRootId == kInvalidObjectId)
            {
                continue;
            }

            // create root tendon axis
            tendonRef->rootAxis->parentAxisId = tendonRootId;
            const ObjectId tendonAxisId = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, rootJointPath, *(tendonRef->rootAxis));
            tendonRef->rootAxis->wasVisited = true;

            if (tendonRootId == kInvalidObjectId)
            {
                // User has already been warned in createObject();
                continue;
            }

            objDb->findOrCreateEntry(rootJointPath, eTendonAxis, tendonAxisId);

            // articulation hierarchy not known, try both directions of joint link refs:
            createFixedTendonAxesRecursive(attachedStage, tendonAxisId, tendonRef->instanceToken, tendonRef->rootAxis->link0, tendonAxes);
            createFixedTendonAxesRecursive(attachedStage, tendonAxisId, tendonRef->instanceToken, tendonRef->rootAxis->link1, tendonAxes);
        }

        // give out warnings for unparsed tendon axes
        for (TendonAxisMap::reference AxesVec : tendonAxes)
        {
            for (TendonAxisMap::mapped_type::reference Axis : AxesVec.second)
            {
                if (!Axis->wasVisited)
                {
                    CARB_LOG_WARN("The fixed tendon axis at %s was not parsed due to a topology issue: Refer to the topology constraints in the USD schema doc for PhysxTendonAxisAPI.",
                        Axis->jointPath.GetText());
                    Axis->wasVisited = true;
                }
            }
        }
    }

    std::vector<PhysxTendonAxisHierarchyDesc*> getAxisUiInfoRecursive(
        const pxr::TfToken& instanceToken, const pxr::SdfPath& currLinkPath, TendonAxisMap& tendonAxes)
    {
        std::vector<PhysxTendonAxisHierarchyDesc*> result;

        for (TendonAxisMap::mapped_type::const_reference AxisRef : tendonAxes[currLinkPath])
        {
            if (AxisRef->wasVisited)
            {
                continue;
            }

            if (AxisRef->instanceToken == instanceToken)
            {
                AxisRef->wasVisited = true;

                PhysxTendonAxisHierarchyDesc* axisDesc = ICE_PLACEMENT_NEW(PhysxTendonAxisHierarchyDesc);

                // get information
                axisDesc->jointPath = AxisRef->jointPath;
                axisDesc->link0 = AxisRef->link0;
                axisDesc->link1 = AxisRef->link1;
                axisDesc->axes = AxisRef->axes;
                axisDesc->type = AxisRef->type;

                // infer directionality and continue traversal in right direction
                if (currLinkPath == AxisRef->link0)
                {
                    axisDesc->children = getAxisUiInfoRecursive(instanceToken, AxisRef->link1, tendonAxes);
                }
                else
                {
                    axisDesc->children = getAxisUiInfoRecursive(instanceToken, AxisRef->link0, tendonAxes);
                }
                result.push_back(axisDesc);
            }
        }

        return result;
    }

    std::vector<PhysxTendonAxisHierarchyDesc*> getAxesUiInfo(TendonAxisMap& tendonAxes, FixedTendonVector& fixedTendons)
    {
        std::vector<PhysxTendonAxisHierarchyDesc*> result;

        // PRECONDITION: Assumes root axis to be a parent to all belonging tendon axes in articulation hierarchy
        for (FixedTendonVector::reference tendonRef : fixedTendons)
        {
            PhysxTendonAxisHierarchyDesc* axisDesc = ICE_PLACEMENT_NEW(PhysxTendonAxisHierarchyDesc);

            // get information
            axisDesc->jointPath = tendonRef->rootAxis->jointPath;
            axisDesc->instanceToken = tendonRef->instanceToken;
            axisDesc->link0 = tendonRef->rootAxis->link0;
            axisDesc->link1 = tendonRef->rootAxis->link1;
            axisDesc->axes = tendonRef->rootAxis->axes;
            axisDesc->type = tendonRef->type;
            tendonRef->rootAxis->wasVisited = true;

            // try both 'directions' as root API can be anywhere in topology
            const std::vector<PhysxTendonAxisHierarchyDesc*>& children1 = getAxisUiInfoRecursive(tendonRef->instanceToken, tendonRef->rootAxis->link0, tendonAxes);
            const std::vector<PhysxTendonAxisHierarchyDesc*>& children2 = getAxisUiInfoRecursive(tendonRef->instanceToken, tendonRef->rootAxis->link1, tendonAxes);

            axisDesc->children = children1;
            axisDesc->children.insert(axisDesc->children.begin(), children2.begin(), children2.end());

            result.push_back(axisDesc);
        }

        // give out warnings for unparsed tendon axes
        for (TendonAxisMap::reference AxesVec : tendonAxes)
        {
            for (TendonAxisMap::mapped_type::reference Axis : AxesVec.second)
            {
                if (!Axis->wasVisited)
                {
                    PhysxTendonAxisHierarchyDesc* axisDesc = ICE_PLACEMENT_NEW(PhysxTendonAxisHierarchyDesc);

                    // get information
                    axisDesc->jointPath = Axis->jointPath;
                    axisDesc->link0 = Axis->link0;
                    axisDesc->link1 = Axis->link1;
                    axisDesc->axes = Axis->axes;

                    result.push_back(axisDesc);
                    Axis->wasVisited = true;
                }
            }
        }

        return result;
    }

} // namespace usdparser
} // namespace physx
} // namespace omni
