// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on
#include "Vehicle.h"
#include "LoadUsd.h"

#include <common/foundation/TypeCast.h>
#include <common/utilities/Utilities.h>

#include <carb/Types.h>
#include <carb/logging/Log.h>

using namespace pxr;
using namespace carb;

namespace omni
{
namespace physx
{
namespace usdparser
{

template <typename T>
static bool checkParamInRange(const T value, const T min, const T max, const char* paramName, const UsdPrim& usdPrim)
{
    if ((value >= min) && (value < max))
        return true;
    else
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\" needs to be in [%d, %d).\n", usdPrim.GetName().GetText(),
                       paramName, min, max);
        return false;
    }
}

template <>
bool checkParamInRange<float>(
    const float value, const float min, const float max, const char* paramName, const UsdPrim& usdPrim)
{
    if ((value >= min) && (value < max))
        return true;
    else
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\" needs to be in [%f, %f).\n", usdPrim.GetName().GetText(),
                       paramName, min, max);
        return false;
    }
}

template <typename T>
static bool checkParamIsPositive(const T value, const char* paramName, const UsdPrim& usdPrim)
{
    if (value > static_cast<const T>(0))
        return true;
    else
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\" needs to be positive.\n", usdPrim.GetName().GetText(),
                       paramName);
        return false;
    }
}

template <typename T>
static bool checkParamIsNonNegative(const T value, const char* paramName, const UsdPrim& usdPrim)
{
    if (value >= static_cast<const T>(0))
        return true;
    else
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\" needs to be greater or equal zero.\n", usdPrim.GetName().GetText(),
                       paramName);
        return false;
    }
}

template <typename T>
bool SafeGetAuthoredAttribute(T* out, const UsdAttribute& attribute)
{
    if (attribute.HasAuthoredValue())
    {
        attribute.Get(out);
        return true;
    }
    else
        return false;
}

template <typename T>
static bool getExpectedAttribute(T& value, const UsdAttribute& attr, const UsdPrim& usdPrim, const char* paramName)
{
    if (SafeGetAttribute(&value, attr))
    {
        return true;
    }
    else
    {
        CARB_LOG_ERROR(
            "Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.\n", usdPrim.GetName().GetText(), paramName);
        return false;
    }
}

static uint32_t getSingleRelationshipPath(const UsdRelationship& relationship,
                                          SdfPath& path,
                                          const char* paramName,
                                          const UsdPrim& usdPrim)
{
    if (relationship.HasAuthoredTargets())
    {
        SdfPathVector paths;
        relationship.GetTargets(&paths);
        if (paths.size() == 1)
        {
            path = paths[0];
            return 0;
        }
        else if (paths.size() == 0)
        {
            return 1;
        }
        else
        {
            CARB_LOG_ERROR("Usd Physics: \"%s\" must not have more than 1 \"%s\" relationship defined.\n",
                            usdPrim.GetName().GetText(), paramName);
            return 2;
        }
    }
    else
    {
        return 1;
    }
}


VehicleComponentTracker::VehicleComponentTracker()
{
}

VehicleComponentTracker::~VehicleComponentTracker()
{
    deleteComponents(mWheels);
    deleteComponents(mTires);
    deleteComponents(mSuspensions);
    deleteComponents(mEngines);
    deleteComponents(mGears);
    deleteComponents(mAutoGearBoxes);
    deleteComponents(mClutches);
    deleteComponents(mDrivesBasic);
    deleteComponents(mDrivesStandard);

    deleteComponents(mMultiWheelDifferentials);
    deleteComponents(mTankDifferentials);
    deleteComponents(mBrakes);
    deleteComponents(mSteeringBasic);
    deleteComponents(mSteeringAckermann);
    deleteComponents(mSuspensionCompliances);
    deleteComponents(mNonlinearCmdResponses);
}


struct ParseContext
{
    ParseContext(const pxr::UsdStageWeakPtr stage_, VehicleComponentTracker& vehicleComponentTracker_)
        : stage(stage_),
          vehicleComponentTracker(vehicleComponentTracker_),
          lengthScale(1.0f / static_cast<float>(UsdGeomGetStageMetersPerUnit(stage_))),
          lengthScaleSqr(lengthScale * lengthScale),
          massScale(1.0f / static_cast<float>(UsdPhysicsGetStageKilogramsPerUnit(stage_))),
          kgmsScale(lengthScaleSqr * massScale)
    {
    }

    const pxr::UsdStageWeakPtr stage;
    VehicleComponentTracker& vehicleComponentTracker;
    float lengthScale;
    float lengthScaleSqr;
    float massScale;
    float kgmsScale;
};

TireFrictionTableDesc* parseTireFrictionTable(const pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& usdPrim)
{
    CARB_ASSERT(usdPrim.IsA<PhysxSchemaPhysxVehicleTireFrictionTable>());

    PhysxSchemaPhysxVehicleTireFrictionTable tireFrictionTable(usdPrim);

    float defaultFrictionValue;
    if (getExpectedAttribute(defaultFrictionValue, tireFrictionTable.GetDefaultFrictionValueAttr(), usdPrim, "defaultFrictionValue"))
    {
        if (!checkParamInRange(defaultFrictionValue, 0.0f, FLT_MAX, "defaultFrictionValue", usdPrim))
            return nullptr;
    }
    else
        return nullptr;

    SdfPath tireFrictionTablePath = usdPrim.GetPath();
    TireFrictionTableDesc* tireFrictionTableDesc = ICE_PLACEMENT_NEW(TireFrictionTableDesc)();
    if (tireFrictionTableDesc)
    {
        bool isValid = true;

        UsdRelationship materialRelationships = tireFrictionTable.GetGroundMaterialsRel();
        if (materialRelationships.HasAuthoredTargets())
        {
            SdfPathVector paths;
            materialRelationships.GetTargets(&paths);
            const uint32_t pathCount = static_cast<uint32_t>(paths.size());
            if (pathCount > 0)
            {
                tireFrictionTableDesc->materialPaths.reserve(pathCount);
                for (uint32_t i = 0; i < pathCount; i++)
                {
                    const UsdPrim materialPrim = stage->GetPrimAtPath(paths[i]);
                    if (materialPrim)
                    {
                        if (materialPrim.HasAPI<UsdPhysicsMaterialAPI>())
                            tireFrictionTableDesc->materialPaths.push_back(paths[i]);
                        else
                        {
                            CARB_LOG_ERROR(
                                "Usd Physics: tire friction table \"%s\": \"groundMaterials\" relationship \"%s\" must have MaterialAPI applied.\n",
                                usdPrim.GetName().GetText(), paths[i].GetText());
                            isValid = false;
                        }
                    }
                    else
                    {
                        CARB_LOG_ERROR(
                            "Usd Physics: tire friction table \"%s\": \"groundMaterials\" relationship has nonexistent path \"%s\".\n",
                            usdPrim.GetName().GetText(), paths[i].GetText());
                        isValid = false;
                    }
                }
            }
            else
            {
                CARB_LOG_ERROR("Usd Physics: tire friction table \"%s\": \"groundMaterials\" needs at least one entry.\n",
                               usdPrim.GetName().GetText());
                isValid = false;
            }
        }

        UsdAttribute frictionValuesAttr = tireFrictionTable.GetFrictionValuesAttr();
        if (frictionValuesAttr.HasAuthoredValue())
        {
            pxr::VtValue arrayDataValue;
            frictionValuesAttr.Get(&arrayDataValue);
            const size_t size = arrayDataValue.GetArraySize();

            const pxr::VtArray<float>& arrayData = arrayDataValue.Get<pxr::VtArray<float>>();
            tireFrictionTableDesc->frictionValues.reserve(size);
            for (uint32_t i = 0; i < static_cast<uint32_t>(size); i++)
            {
                tireFrictionTableDesc->frictionValues.push_back(arrayData[i]);
            }
        }

        if (tireFrictionTableDesc->materialPaths.size() != tireFrictionTableDesc->frictionValues.size())
        {
            CARB_LOG_ERROR(
                "Usd Physics: tire friction table \"%s\": attribute \"frictionValues\" needs to have the same number of entries as relationships in \"groundMaterials\".\n",
                usdPrim.GetName().GetText());
            isValid = false;
        }

        if (isValid)
        {
            tireFrictionTableDesc->defaultFrictionValue = defaultFrictionValue;
            tireFrictionTableDesc->path = tireFrictionTablePath;
            return tireFrictionTableDesc;
        }
        else
        {
            tireFrictionTableDesc->~TireFrictionTableDesc();
            ICE_FREE(tireFrictionTableDesc);
            return nullptr;
        }
    }
    else
    {
        CARB_LOG_ERROR("Usd Physics: allocation of tire friction table descriptor failed.\n");
        return nullptr;
    }
}

bool parseWheel(WheelDesc& out, const UsdPrim& usdPrim, const void*)
{
    PhysxSchemaPhysxVehicleWheelAPI wheel(usdPrim);

    out.path = usdPrim.GetPath();

    if (getExpectedAttribute(out.radius, wheel.GetRadiusAttr(), usdPrim, "radius"))
    {
        if (!checkParamInRange(out.radius, 0.0f, FLT_MAX, "radius", usdPrim))
            return false;
    }
    else
        return false;

    if (getExpectedAttribute(out.width, wheel.GetWidthAttr(), usdPrim, "width"))
    {
        if (!checkParamInRange(out.width, 0.0f, FLT_MAX, "width", usdPrim))
            return false;
    }
    else
        return false;

    if (getExpectedAttribute(out.mass, wheel.GetMassAttr(), usdPrim, "mass"))
    {
        if (!checkParamInRange(out.mass, 0.0f, FLT_MAX, "mass", usdPrim))
            return false;
    }
    else
        return false;

    if (getExpectedAttribute(out.moi, wheel.GetMoiAttr(), usdPrim, "moi"))
    {
        if (!checkParamInRange(out.moi, 0.0f, FLT_MAX, "moi", usdPrim))
            return false;
    }
    else
        return false;

    if (getExpectedAttribute(out.dampingRate, wheel.GetDampingRateAttr(), usdPrim, "dampingRate"))
    {
        if (!checkParamInRange(out.dampingRate, 0.0f, FLT_MAX, "dampingRate", usdPrim))
            return false;
    }
    else
        return false;

    if (SafeGetAttribute(&out.maxBrakeTorque, wheel.GetMaxBrakeTorqueAttr()))
    {
        CARB_LOG_WARN("Usd Physics: wheel \"%s\": attribute \"maxBrakeTorque\" is deprecated. "
            "Please use PhysxVehicleBrakesAPI instead.\n",
            usdPrim.GetName().GetText());

        if (!checkParamInRange(out.maxBrakeTorque, 0.0f, FLT_MAX, "maxBrakeTorque", usdPrim))
            return false;
    }
    else
        out.maxBrakeTorque = 0.0f;

    if (SafeGetAttribute(&out.maxHandBrakeTorque, wheel.GetMaxHandBrakeTorqueAttr()))
    {
        CARB_LOG_WARN("Usd Physics: wheel \"%s\": attribute \"maxHandBrakeTorque\" is deprecated. "
            "Please use PhysxVehicleBrakesAPI instead.\n",
            usdPrim.GetName().GetText());

        if (!checkParamInRange(out.maxHandBrakeTorque, 0.0f, FLT_MAX, "maxHandBrakeTorque", usdPrim))
            return false;
    }
    else
        out.maxHandBrakeTorque = 0.0f;

    if (SafeGetAttribute(&out.maxSteerAngle, wheel.GetMaxSteerAngleAttr()))
    {
        CARB_LOG_WARN("Usd Physics: wheel \"%s\": attribute \"maxSteerAngle\" is deprecated. "
            "Please use PhysxVehicleSteeringAPI instead.\n",
            usdPrim.GetName().GetText());

        if (!checkParamInRange(
            out.maxSteerAngle, static_cast<float>(-M_PI_2) + FLT_MIN, static_cast<float>(M_PI_2), "maxSteerAngle", usdPrim))
            return false;
    }
    else
        out.maxSteerAngle = 0.0f;

    if (SafeGetAttribute(&out.toeAngle, wheel.GetToeAngleAttr()))
    {
        CARB_LOG_WARN("Usd Physics: wheel \"%s\": attribute \"toeAngle\" is deprecated. "
            "Please use PhysxVehicleSuspensionComplianceAPI instead.\n",
            usdPrim.GetName().GetText());

        if (!checkParamInRange(
                out.toeAngle, static_cast<float>(-M_PI_2) + FLT_MIN, static_cast<float>(M_PI_2), "toeAngle", usdPrim))
            return false;
    }
    else
        out.toeAngle = 0.0f;

    return true;
}

bool parseTire(TireDesc& out, const UsdPrim& usdPrim, const void* userData)
{
    const ParseContext& parseContext = *reinterpret_cast<const ParseContext*>(userData);

    PhysxSchemaPhysxVehicleTireAPI tire(usdPrim);

    out.path = usdPrim.GetPath();

    GfVec2f vec2;
    if (getExpectedAttribute(vec2, tire.GetLateralStiffnessGraphAttr(), usdPrim, "lateralStiffnessGraph"))
    {
        out.lateralStiffnessGraph.x = vec2[0];
        out.lateralStiffnessGraph.y = vec2[1];

        if ((vec2[0] != 0.0f) || (vec2[1] != 0.0f))
        {
            if (!checkParamIsNonNegative(vec2[0], "lateralStiffnessGraph.x", usdPrim))
                return false;

            if (!checkParamIsPositive(vec2[1], "lateralStiffnessGraph.y", usdPrim))
                return false;
        }
        else
        {
            CARB_LOG_WARN("Usd Physics: tire \"%s\": attributes \"latStiffX\" and \"latStiffY\" "
                "are deprecated. Please use lateralStiffnessGraph instead.\n",
                usdPrim.GetName().GetText());

            if (getExpectedAttribute(out.latStiffX, tire.GetLatStiffXAttr(), usdPrim, "latStiffX"))
            {
                if (!checkParamInRange(out.latStiffX, 0.0f, FLT_MAX, "latStiffX", usdPrim))
                    return false;
            }
            else
                return false;

            if (SafeGetAttribute(&out.latStiffY, tire.GetLatStiffYAttr()))
            {
                if (!checkParamInRange(out.latStiffY, 0.0f, FLT_MAX, "latStiffY", usdPrim))
                    return false;
            }
            else
                out.latStiffY = 17.095f;
        }
    }
    else
        return false;

    if (getExpectedAttribute(out.longitudinalStiffness, tire.GetLongitudinalStiffnessAttr(), usdPrim, "longitudinalStiffness"))
    {
        if (out.longitudinalStiffness != 0.0f)
        {
            if (!checkParamIsPositive(out.longitudinalStiffness, "longitudinalStiffness", usdPrim))
                return false;
        }
        else
        {
            CARB_LOG_WARN("Usd Physics: tire \"%s\": attribute \"longitudinalStiffnessPerUnitGravity\" "
                "is deprecated. Please use lateralStiffnessGraph instead.\n",
                usdPrim.GetName().GetText());

            if (SafeGetAttribute(&out.longitudinalStiffnessPerUnitGravity, tire.GetLongitudinalStiffnessPerUnitGravityAttr()))
            {
                if (!checkParamInRange(
                        out.longitudinalStiffnessPerUnitGravity, 0.0f, FLT_MAX, "longitudinalStiffnessPerUnitGravity", usdPrim))
                    return false;
            }
            else
                out.longitudinalStiffnessPerUnitGravity = 500.0f * parseContext.massScale;
        }
    }
    else
        return false;

    if (getExpectedAttribute(out.camberStiffness, tire.GetCamberStiffnessAttr(), usdPrim, "camberStiffness"))
    {
        if (out.camberStiffness != -1.0f)
        {
            if (!checkParamIsNonNegative(out.camberStiffness, "camberStiffness", usdPrim))
                return false;
        }
        else
        {
            CARB_LOG_WARN("Usd Physics: tire \"%s\": attribute \"camberStiffnessPerUnitGravity\" "
                "is deprecated. Please use camberStiffness instead.\n",
                usdPrim.GetName().GetText());

            if (getExpectedAttribute(out.camberStiffnessPerUnitGravity, tire.GetCamberStiffnessPerUnitGravityAttr(),
                    usdPrim, "camberStiffnessPerUnitGravity"))
            {
                if (!checkParamInRange(out.camberStiffnessPerUnitGravity, 0.0f, FLT_MAX, "camberStiffnessPerUnitGravity", usdPrim))
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;

    VtArray<GfVec2f> tireFrictionVsSlipGraph;
    if (SafeGetAttribute(&tireFrictionVsSlipGraph, tire.GetFrictionVsSlipGraphAttr()))
    {
        if ((tireFrictionVsSlipGraph.size() == 3))
        {
            for (uint32_t i = 0; i < 3; i++)
            {
                out.frictionVsSlipGraph[i].x = tireFrictionVsSlipGraph[i][0];
                out.frictionVsSlipGraph[i].y = tireFrictionVsSlipGraph[i][1];
            }

            if ((out.frictionVsSlipGraph[0].x >= out.frictionVsSlipGraph[1].x) ||
                (out.frictionVsSlipGraph[1].x >= out.frictionVsSlipGraph[2].x))
            {
                CARB_LOG_ERROR(
                    "Usd Physics: attribute \"frictionVsSlipGraph\" of tire \"%s\" has invalid values: "
                    "frictionVsSlipGraph[2].x > frictionVsSlipGraph[1].x > frictionVsSlipGraph[0].x has to hold.\n",
                    usdPrim.GetName().GetText());
                return false;
            }
        }
        else
        {
            CARB_LOG_ERROR(
                "Usd Physics: attribute \"frictionVsSlipGraph\" of tire \"%s\" needs to have exactly 3 entries.\n",
                usdPrim.GetName().GetText());

            return false;
        }
    }
    else
    {
        out.frictionVsSlipGraph[0].x = 0.0f;
        out.frictionVsSlipGraph[0].y = 1.0f;
        out.frictionVsSlipGraph[1].x = 0.1f;
        out.frictionVsSlipGraph[1].y = 1.0f;
        out.frictionVsSlipGraph[2].x = 1.0f;
        out.frictionVsSlipGraph[2].y = 1.0f;
    }

    SdfPath frictionTablePath;
    uint32_t errCode = getSingleRelationshipPath(tire.GetFrictionTableRel(), frictionTablePath, "frictionTable", usdPrim);
    if (errCode == 0)
    {
        const UsdPrim frictionTablePrim = parseContext.stage->GetPrimAtPath(frictionTablePath);
        if (frictionTablePrim)
        {
            if (frictionTablePrim.IsA<PhysxSchemaPhysxVehicleTireFrictionTable>())
                out.frictionTablePath = frictionTablePath;
            else
            {
                CARB_LOG_ERROR("Usd Physics: tire \"%s\": \"frictionTable\" relationship does not point to a "
                    "PhysxVehicleTireFrictionTable prim (\"%s\").\n",
                    usdPrim.GetName().GetText(), frictionTablePath.GetText());
                return false;
            }
        }
        else
        {
            CARB_LOG_ERROR("Usd Physics: tire \"%s\": \"frictionTable\" relationship has nonexistent path \"%s\".\n",
                           usdPrim.GetName().GetText(), frictionTablePath.GetText());
            return false;
        }
    }
    else if (errCode == 2)  // it is valid to not have a friction table defined
        return false;

    if (getExpectedAttribute(out.restLoad, tire.GetRestLoadAttr(), usdPrim, "restLoad"))
    {
        if (!checkParamInRange(out.restLoad, 0.0f, FLT_MAX, "restLoad", usdPrim))
            return false;
    }
    else
        return false;

    return true;
}

bool parseSuspension(SuspensionDesc& out, const UsdPrim& usdPrim, const void*)
{
    PhysxSchemaPhysxVehicleSuspensionAPI suspension(usdPrim);

    out.path = usdPrim.GetPath();

    if (getExpectedAttribute(out.springStrength, suspension.GetSpringStrengthAttr(), usdPrim, "springStrength"))
    {
        if (!checkParamInRange(out.springStrength, 0.0f, FLT_MAX, "springStrength", usdPrim))
            return false;
    }
    else
        return false;

    if (getExpectedAttribute(out.springDamperRate, suspension.GetSpringDamperRateAttr(), usdPrim, "springDamperRate"))
    {
        if (!checkParamInRange(out.springDamperRate, 0.0f, FLT_MAX, "springDamperRate", usdPrim))
            return false;
    }
    else
        return false;

    if (SafeGetAttribute(&out.travelDistance, suspension.GetTravelDistanceAttr()))
    {
        if (!checkParamIsPositive(out.travelDistance, "travelDistance", usdPrim))
            return false;

        // still important to set maxDroop=0 to make sure the system treats it as user-defined
        // max droop because that avoids triggering code that tries to update max droop when
        // sprung mass related properties change. It also avoids obsolete error messages being
        // sent.
        out.maxDroop = 0.0f;
        out.maxCompression = 0.0f;
    }
    else
    {
        out.travelDistance = -1.0f;  // marker for not defined

        if (getExpectedAttribute(out.maxCompression, suspension.GetMaxCompressionAttr(), usdPrim, "maxCompression"))
        {
            CARB_LOG_WARN("Usd Physics: suspension \"%s\": attribute \"maxCompression\" is deprecated. "
                "Please use travelDistance instead.\n",
                usdPrim.GetName().GetText());

            if (!checkParamInRange(out.maxCompression, 0.0f, FLT_MAX, "maxCompression", usdPrim))
                return false;
        }
        else
            return false;

        if (getExpectedAttribute(out.maxDroop, suspension.GetMaxDroopAttr(), usdPrim, "maxDroop"))
        {
            CARB_LOG_WARN("Usd Physics: suspension \"%s\": attribute \"maxDroop\" is deprecated. "
                "Please use travelDistance instead.\n",
                usdPrim.GetName().GetText());

            // note: no range check as negative value is legal and means to compute automatically
        }
        else
            return false;
    }

    if (getExpectedAttribute(out.sprungMass, suspension.GetSprungMassAttr(), usdPrim, "sprungMass"))
    {
        if (!checkParamInRange(out.sprungMass, 0.0f, FLT_MAX, "sprungMass", usdPrim))
            return false;
    }
    else
        return false;

    if (SafeGetAttribute(&out.camberAtRest, suspension.GetCamberAtRestAttr()))
    {
        CARB_LOG_WARN("Usd Physics: suspension \"%s\": attribute \"camberAtRest\" is deprecated. "
            "Please use PhysxVehicleSuspensionComplianceAPI instead.\n",
            usdPrim.GetName().GetText());

        if (!checkParamInRange(out.camberAtRest, -static_cast<float>(M_PI_2), static_cast<float>(M_PI_2) + FLT_MIN,
                               "camberAtRest", usdPrim))
            return false;
    }
    else
        out.camberAtRest = 0.0f;

    if (SafeGetAttribute(&out.camberAtMaxCompression, suspension.GetCamberAtMaxCompressionAttr()))
    {
        CARB_LOG_WARN("Usd Physics: suspension \"%s\": attribute \"camberAtMaxCompression\" is deprecated. "
            "Please use PhysxVehicleSuspensionComplianceAPI instead.\n",
            usdPrim.GetName().GetText());

        if (!checkParamInRange(out.camberAtMaxCompression, -static_cast<float>(M_PI_2),
                               static_cast<float>(M_PI_2) + FLT_MIN, "camberAtMaxCompression", usdPrim))
            return false;
    }
    else
        out.camberAtMaxCompression = 0.0f;

    if (SafeGetAttribute(&out.camberAtMaxDroop, suspension.GetCamberAtMaxDroopAttr()))
    {
        CARB_LOG_WARN("Usd Physics: suspension \"%s\": attribute \"camberAtMaxDroop\" is deprecated. "
            "Please use PhysxVehicleSuspensionComplianceAPI instead.\n",
            usdPrim.GetName().GetText());

        if (!checkParamInRange(out.camberAtMaxDroop, -static_cast<float>(M_PI_2), static_cast<float>(M_PI_2) + FLT_MIN,
                               "camberAtMaxDroop", usdPrim))
            return false;
    }
    else
        out.camberAtMaxDroop = 0.0f;

    return true;
}

template <typename TDesc, typename TDescOut>
bool parseComponent(const pxr::UsdPrim& prim, const pxr::SdfPath& path,
                    VehicleComponentTracker& vehicleComponentTracker,
                    std::map<pxr::SdfPath, TDesc*>& componentMap,
                    bool (*parseFunction)(TDesc&, const pxr::UsdPrim&, const void*),
                    const void* userData,
                    const char* errMsgDescriptorName,
                    TDescOut*& out)
{
    CARB_ASSERT(!vehicleComponentTracker.findComponent(path, componentMap));

    TDesc* desc = vehicleComponentTracker.addComponent(path, componentMap);
    if (desc)
    {
        if (parseFunction(*desc, prim, userData))
        {
            out = desc;
            return true;
        }
        else
        {
            vehicleComponentTracker.removeComponent(path, componentMap);
            return false;
        }
    }
    else
    {
        CARB_LOG_ERROR("Usd Physics: %s: failed to allocate memory for %s descriptor.\n",
                        prim.GetPath().GetText(), errMsgDescriptorName);
        return false;
    }
}

template <typename TDesc, typename TDescOut>
bool parseComponentWithMapCheck(const pxr::UsdPrim& prim, const pxr::SdfPath& path,
                    VehicleComponentTracker& vehicleComponentTracker,
                    std::map<pxr::SdfPath, TDesc*>& componentMap,
                    bool (*parseFunction)(TDesc&, const pxr::UsdPrim&, const void*),
                    const void* userData,
                    const char* errMsgDescriptorName,
                    TDescOut*& out)
{
    TDesc* desc = vehicleComponentTracker.findComponent(path, componentMap);

    if (desc)
    {
        out = desc;
        return true;
    }
    else
    {
        return parseComponent<TDesc, TDescOut>(prim, path, vehicleComponentTracker, componentMap,
            parseFunction, userData, errMsgDescriptorName, out);
    }
}

template <typename TUsdObjectType, typename TDesc, typename TDescOut>
bool parseComponent(const TUsdObjectType& primOrAPI,
                    VehicleComponentTracker& vehicleComponentTracker,
                    std::vector<TDesc*>& componentList,
                    bool (*parseFunction)(TDesc&, const TUsdObjectType&, const void*),
                    const void* userData,
                    const char* errMsgDescriptorName,
                    TDescOut*& out)
{
    TDesc* desc = vehicleComponentTracker.addComponent(componentList);
    if (desc)
    {
        if (parseFunction(*desc, primOrAPI, userData))
        {
            out = desc;
            return true;
        }
        else
        {
            vehicleComponentTracker.removeLastComponent(componentList);
            return false;
        }
    }
    else
    {
        CARB_LOG_ERROR("Usd Physics: %s: failed to allocate memory for %s descriptor.\n",
                        primOrAPI.GetPath().GetText(), errMsgDescriptorName);
        return false;
    }
}

template <typename TDesc, typename TDescOut, typename TSchema>
bool parseRelationship(const pxr::UsdStageWeakPtr stage,
                       const pxr::UsdPrim& primWithRelationship,
                       const pxr::SdfPath& relationshipPath,
                       VehicleComponentTracker& vehicleComponentTracker,
                       std::map<pxr::SdfPath, TDesc*>& componentMap,
                       bool (*parseFunction)(TDesc&, const pxr::UsdPrim&, const void*),
                       const void* userData,
                       const char* errMsgPrimWithRelDesc,
                       const char* errMsgAttributeName,
                       const char* errMsgTypeName,
                       TDescOut*& out)
{
    TDesc* desc = vehicleComponentTracker.findComponent(relationshipPath, componentMap);

    if (desc)
    {
        out = desc;
        return true;
    }
    else
    {
        CARB_ASSERT(relationshipPath.IsPrimPath());
        const UsdPrim relationshipPrim = stage->GetPrimAtPath(relationshipPath);
        if (relationshipPrim)
        {
            if (relationshipPrim.HasAPI<TSchema>())
            {
                return parseComponent(relationshipPrim, relationshipPath, vehicleComponentTracker, componentMap,
                    parseFunction, userData, errMsgAttributeName, out);
            }
            else
            {
                CARB_LOG_ERROR("Usd Physics: %s \"%s\": \"%s\" relationship must point to prim with %s applied.\n",
                                errMsgPrimWithRelDesc, primWithRelationship.GetName().GetText(), errMsgAttributeName,
                                errMsgTypeName);
                return false;
            }
        }
        else
        {
            CARB_LOG_ERROR("Usd Physics: %s \"%s\": \"%s\" relationship has nonexistent path \"%s\".\n",
                            errMsgPrimWithRelDesc, primWithRelationship.GetName().GetText(), errMsgAttributeName,
                            relationshipPath.GetText());
            return false;
        }
    }
}

//
// many of the vehicle components can be defined either by applying an API directly or by linking
// to a prim with that API through relationships
//
template <typename TDesc, typename TDescOut, typename TSchema, const bool tIsOptional>
bool parseRelationshipOrAPI(const pxr::UsdStageWeakPtr stage,
                       const pxr::UsdPrim& primWithRelationship,
                       const pxr::UsdRelationship& relationship,
                       VehicleComponentTracker& vehicleComponentTracker,
                       std::map<pxr::SdfPath, TDesc*>& componentMap,
                       bool (*parseFunction)(TDesc&, const pxr::UsdPrim&, const void*),
                       const void* userData,
                       const char* errMsgPrimWithRelDesc,
                       const char* errMsgAttributeName,
                       const char* errMsgTypeName,
                       TDescOut*& out)
{
    SdfPath relationshipPath;
    const uint32_t errCode = getSingleRelationshipPath(relationship, relationshipPath, errMsgAttributeName, primWithRelationship);

    if (errCode == 0)
    {
        if (primWithRelationship.HasAPI<TSchema>())
        {
            CARB_LOG_WARN("Usd Physics: %s \"%s\": has \"%s\" relationship defined and at the same time the API %s "
                "applied. Only one or the other is allowed. Applied API will be ignored.\n",
                errMsgPrimWithRelDesc, primWithRelationship.GetName().GetText(), errMsgAttributeName,
                errMsgTypeName);
        }

        return parseRelationship<TDesc, TDescOut, TSchema>(
            stage, primWithRelationship, relationshipPath, vehicleComponentTracker, componentMap,
            parseFunction, userData, errMsgPrimWithRelDesc, errMsgAttributeName, errMsgTypeName,
            out);
    }
    else if (errCode == 1)
    {
        if (primWithRelationship.HasAPI<TSchema>())
        {
            // even though the API is directly applied to the prim, it is technically possible,
            // that another vehicle has a relationship to this prim and thus it might have been
            // parsed already. That's why the component map will be checked first.

            const pxr::SdfPath& path = primWithRelationship.GetPath();
            return parseComponentWithMapCheck<TDesc, TDescOut>(primWithRelationship, path,
                vehicleComponentTracker, componentMap,
                parseFunction, userData, errMsgAttributeName, out);
        }
        else
        {
            if (tIsOptional)
                return true;
            else
            {
                CARB_LOG_WARN("Usd Physics: %s \"%s\": has no \"%s\" relationship defined nor is the API %s "
                    "applied. One of those two requirements needs to be met.\n",
                    errMsgPrimWithRelDesc, primWithRelationship.GetName().GetText(), errMsgAttributeName,
                    errMsgTypeName);
            }
        }
    }

    return false;
}

static bool setSuspensionComplianceAngles(const VtArray<GfVec2f>& values,
    std::vector<carb::Float2>& out, const UsdPrim& usdPrim, const char* attrName)
{
    const size_t valueCount = values.size();
    float previousJounce = -1.0f;
    if (valueCount)
    {
        if (valueCount > 3)
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\": max number of supported "
                "entries is 3.\n", usdPrim.GetName().GetText(), attrName);

            return false;
        }

        for (GfVec2f value : values)
        {
            if ((value[0] < 0.0f) || (value[0] > 1.0f))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s  (first axis)\" needs to be in range [0, 1].\n",
                    usdPrim.GetName().GetText(), attrName);

                return false;
            }

            if (fabsf(value[1]) > static_cast<float>(M_PI_2))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s  (second axis)\" needs to be in range [-pi, pi].\n",
                    usdPrim.GetName().GetText(), attrName);

                return false;
            }

            if (value[0] <= previousJounce)
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\": normalized jounce values "
                    "have to be monotonically increasing.\n", usdPrim.GetName().GetText(), attrName);

                return false;
            }

            out.push_back(carb::Float2{ value[0], value[1] });

            previousJounce = value[0];
        }
    }

    return true;
}

static bool setSuspensionCompliancePoints(const VtArray<GfVec4f>& values,
    std::vector<carb::Float4>& out, const UsdPrim& usdPrim, const char* attrName)
{
    const size_t valueCount = values.size();
    float previousJounce = -1.0f;
    if (valueCount)
    {
        if (valueCount > 3)
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\": max number of supported "
                "entries is 3.\n", usdPrim.GetName().GetText(), attrName);

            return false;
        }

        for (GfVec4f value : values)
        {
            if ((value[0] < 0.0f) || (value[0] > 1.0f))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s  (first axis)\" needs to be in range [0, 1].\n",
                    usdPrim.GetName().GetText(), attrName);

                return false;
            }

            if (value[0] <= previousJounce)
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\": normalized jounce values "
                    "have to be monotonically increasing.\n", usdPrim.GetName().GetText(), attrName);

                return false;
            }

            out.push_back(carb::Float4{ value[0], value[1], value[2], value[3] });

            previousJounce = value[0];
        }
    }

    return true;
}

bool parseSuspensionCompliance(SuspensionComplianceDesc& out, const UsdPrim& usdPrim, const void*)
{
    CARB_ASSERT(usdPrim.HasAPI<PhysxSchemaPhysxVehicleSuspensionComplianceAPI>());

    PhysxSchemaPhysxVehicleSuspensionComplianceAPI suspensionCompliance(usdPrim);

    VtArray<GfVec2f> valuesVec2f;
    if (SafeGetAttribute(&valuesVec2f, suspensionCompliance.GetWheelToeAngleAttr()))
    {
        if (!setSuspensionComplianceAngles(valuesVec2f, out.wheelToeAngleList,
                usdPrim, "wheelToeAngle"))
        {
            return false;
        }
    }

    valuesVec2f.clear();
    if (SafeGetAttribute(&valuesVec2f, suspensionCompliance.GetWheelCamberAngleAttr()))
    {
        if (!setSuspensionComplianceAngles(valuesVec2f, out.wheelCamberAngleList,
                usdPrim, "wheelCamberAngle"))
        {
            return false;
        }
    }

    VtArray<GfVec4f> valuesVec4f;
    if (SafeGetAttribute(&valuesVec4f, suspensionCompliance.GetSuspensionForceAppPointAttr()))
    {
        if (!setSuspensionCompliancePoints(valuesVec4f, out.suspensionForceAppPointList,
                usdPrim, "suspensionForceAppPoint"))
        {
            return false;
        }
    }

    valuesVec4f.clear();
    if (SafeGetAttribute(&valuesVec4f, suspensionCompliance.GetTireForceAppPointAttr()))
    {
        if (!setSuspensionCompliancePoints(valuesVec4f, out.tireForceAppPointList,
                usdPrim, "tireForceAppPoint"))
        {
            return false;
        }
    }

    return true;
}

bool parseWheelAttachment(WheelAttachmentDesc& out,
                          const UsdPrim& usdPrim,
                          const ParseContext& parseContext,
                          VehicleComponentTracker& vehicleComponentTracker)
{
    out.path = usdPrim.GetPath();
    out.state = 0;

    PhysxSchemaPhysxVehicleWheelAttachmentAPI wheelAttachment(usdPrim);

    if (!parseRelationshipOrAPI<WheelDesc, WheelDesc, PhysxSchemaPhysxVehicleWheelAPI, false>(
            parseContext.stage, usdPrim, wheelAttachment.GetWheelRel(), vehicleComponentTracker,
            vehicleComponentTracker.mWheels, parseWheel, nullptr, "wheel attachment", "wheel", "PhysxVehicleWheelAPI",
            out.wheel))
    {
        return false;
    }

    if (!parseRelationshipOrAPI<TireDesc, TireDesc, PhysxSchemaPhysxVehicleTireAPI, false>(
            parseContext.stage, usdPrim, wheelAttachment.GetTireRel(), vehicleComponentTracker,
            vehicleComponentTracker.mTires, parseTire, &parseContext, "wheel attachment", "tire", "PhysxVehicleTireAPI",
            out.tire))
    {
        return false;
    }

    if (!parseRelationshipOrAPI<SuspensionDesc, SuspensionDesc, PhysxSchemaPhysxVehicleSuspensionAPI, false>(
            parseContext.stage, usdPrim, wheelAttachment.GetSuspensionRel(), vehicleComponentTracker,
            vehicleComponentTracker.mSuspensions, parseSuspension, nullptr, "wheel attachment", "suspension",
            "PhysxVehicleSuspensionAPI", out.suspension))
    {
        return false;
    }

    if (!getExpectedAttribute(out.suspensionTravelDirection, wheelAttachment.GetSuspensionTravelDirectionAttr(),
                              usdPrim, "suspensionTravelDirection"))
    {
        return false;
    }

    if (SafeGetAttribute(&out.suspensionForceAppPointOffset, wheelAttachment.GetSuspensionForceAppPointOffsetAttr()))
    {
        CARB_LOG_WARN("Usd Physics: wheel attachment \"%s\": attribute \"suspensionForceAppPointOffset\" is deprecated. "
            "Please use PhysxVehicleSuspensionComplianceAPI instead.\n",
            usdPrim.GetName().GetText());

        out.state |= WheelAttachmentDesc::eHAS_SUSP_FORCE_APP_POINT;
    }

    if (SafeGetAttribute(&out.suspensionFramePosition, wheelAttachment.GetSuspensionFramePositionAttr()))
    {
        out.state |= WheelAttachmentDesc::eHAS_SUSPENSION_FRAME;
    }
    else if (getExpectedAttribute(out.wheelCenterOfMassOffset, wheelAttachment.GetWheelCenterOfMassOffsetAttr(),
                 usdPrim, "wheelCenterOfMassOffset"))
    {
        CARB_LOG_WARN("Usd Physics: wheel attachment \"%s\": attribute \"wheelCenterOfMassOffset\" is deprecated. "
            "Please use suspensionFramePosition instead.\n",
            usdPrim.GetName().GetText());

        out.state |= WheelAttachmentDesc::eHAS_WHEEL_COM_OFFSET;
    }
    else
    {
        CARB_LOG_ERROR("Usd Physics: wheel attachment \"%s\": either \"suspensionFramePosition\" or the deprecated "
            "\"wheelCenterOfMassOffset\" have to be defined.\n",
            usdPrim.GetName().GetText());

        return false;
    }

    pxr::GfQuatf suspFrameOrient;
    if (getExpectedAttribute(suspFrameOrient, wheelAttachment.GetSuspensionFrameOrientationAttr(), usdPrim,
        "suspensionFrameOrientation"))
    {
        GfQuatToFloat4(suspFrameOrient, out.suspensionFrameOrientation);
    }
    else
        return false;

    if (SafeGetAttribute(&out.tireForceAppPointOffset, wheelAttachment.GetTireForceAppPointOffsetAttr()))
    {
        CARB_LOG_WARN("Usd Physics: wheel attachment \"%s\": attribute \"tireForceAppPointOffset\" is deprecated. "
            "Please use PhysxVehicleSuspensionComplianceAPI instead.\n",
            usdPrim.GetName().GetText());

        out.state |= WheelAttachmentDesc::eHAS_TIRE_FORCE_APP_POINT;
    }

    if (!getExpectedAttribute(out.wheelFramePosition, wheelAttachment.GetWheelFramePositionAttr(), usdPrim,
                              "wheelFramePosition"))
    {
        return false;
    }

    pxr::GfQuatf wheelFrameOrient;
    if (getExpectedAttribute(wheelFrameOrient, wheelAttachment.GetWheelFrameOrientationAttr(), usdPrim,
        "wheelFrameOrientation"))
    {
        GfQuatToFloat4(wheelFrameOrient, out.wheelFrameOrientation);
    }
    else
        return false;

    if (SafeGetAttribute(&out.driven, wheelAttachment.GetDrivenAttr()))
    {
        CARB_LOG_WARN("Usd Physics: wheel attachment \"%s\": attribute \"driven\" is deprecated. "
            "Please use PhysxVehicleMultiWheelDifferentialAPI to specify the driven wheels.\n",
            usdPrim.GetName().GetText());
    }
    else
        out.driven = false;

    if (getExpectedAttribute(out.index, wheelAttachment.GetIndexAttr(), usdPrim, "index"))
    {
        if (!checkParamInRange(out.index, -1, static_cast<int>(VehicleDesc::maxNumberOfWheels), "index", usdPrim))
            return false;
    }
    else
        return false;

    SdfPath collisionGroupPath;
    uint32_t errCode =
        getSingleRelationshipPath(wheelAttachment.GetCollisionGroupRel(), collisionGroupPath, "collisionGroup", usdPrim);
    if (errCode == 0)
    {
        const UsdPrim collisionGroupPrim = parseContext.stage->GetPrimAtPath(collisionGroupPath);
        if (collisionGroupPrim)
        {
            if (collisionGroupPrim.IsA<UsdPhysicsCollisionGroup>())
                out.collisionGroupPath = collisionGroupPath;
            else
            {
                CARB_LOG_ERROR("Usd Physics: wheel attachment \"%s\": \"collisionGroup\" relationship does not point to a "
                    "UsdPhysicsCollisionGroup prim (\"%s\").\n",
                    usdPrim.GetName().GetText(), collisionGroupPath.GetText());
                return false;
            }
        }
        else
        {
            CARB_LOG_ERROR(
                "Usd Physics: wheel attachment \"%s\": \"collisionGroup\" relationship has nonexistent path \"%s\".\n",
                usdPrim.GetName().GetText(), collisionGroupPath.GetText());
            return false;
        }
    }
    else if (errCode == 2)
        return false;

    if (usdPrim.IsA<UsdGeomXformable>())
    {
        out.state |= WheelAttachmentDesc::eMANAGE_TRANSFORMS;

        // the wheel root prim might be a collision shape
        bool foundCollisionShape = false;
        if (usdPrim.HasAPI<UsdPhysicsCollisionAPI>())
        {
            out.shapePath = usdPrim.GetPath();
            out.state |= WheelAttachmentDesc::eHAS_SHAPE;
            foundCollisionShape = true;
        }
        // note: traversal of descendants is done anyway to find illegal configurations

        // there might be a collision shape among the children in which case its transform should be managed by the
        // vehicle simulation. Traversing through all descendants to find misusage (which might happen easily for
        // people not reading the schema docu since there are quite a few restrictions)
        UsdPrimSubtreeRange subPrims = usdPrim.GetDescendants();
        for (UsdPrim subPrim : subPrims)
        {
            if (subPrim.HasAPI<UsdPhysicsCollisionAPI>())
            {
                if (!foundCollisionShape)
                {
                    if (subPrim.GetParent() == usdPrim)
                    {
                        out.shapePath = subPrim.GetPath();
                        out.state |= WheelAttachmentDesc::eHAS_SHAPE;
                        foundCollisionShape = true;
                    }
                    else
                    {
                        CARB_LOG_ERROR(
                            "Usd Physics: wheel attachment \"%s\": the sub prim with CollisionAPI applied has to be a direct child "
                            "of the wheel attachment prim (triggered at path: \"%s\").\n",
                            usdPrim.GetName().GetText(), subPrim.GetPath().GetText());
                        return false;
                    }
                }
                else
                {
                    CARB_LOG_ERROR(
                        "Usd Physics: wheel attachment \"%s\": only the wheel attachment or one child can have CollisionAPI applied (triggered at path: \"%s\").\n",
                        usdPrim.GetName().GetText(), subPrim.GetPath().GetText());
                    return false;
                }
            }
        }
    }

    if (usdPrim.HasAPI<PhysxSchemaPhysxVehicleSuspensionComplianceAPI>())
    {
        if (!parseComponent<UsdPrim, SuspensionComplianceDesc, SuspensionComplianceDesc>(usdPrim,
                parseContext.vehicleComponentTracker, parseContext.vehicleComponentTracker.mSuspensionCompliances,
                parseSuspensionCompliance, nullptr, "suspensionCompliance", out.suspensionCompliance))
        {
            return false;
        }
    }
    else
        out.suspensionCompliance = nullptr;

    return true;
}

bool parseWheelController(WheelControllerDesc& out, const UsdPrim& usdPrim)
{
    out.path = usdPrim.GetPath();

    PhysxSchemaPhysxVehicleWheelControllerAPI wheelController(usdPrim);

    if (!SafeGetAttribute(&out.driveTorque, wheelController.GetDriveTorqueAttr()))
        out.driveTorque = 0.0f;

    if (SafeGetAttribute(&out.brakeTorque, wheelController.GetBrakeTorqueAttr()))
    {
        if (!checkParamInRange(out.brakeTorque, 0.0f, FLT_MAX, "brakeTorque", usdPrim))
            return false;
    }
    else
        out.brakeTorque = 0.0f;

    if (!SafeGetAttribute(&out.steerAngle, wheelController.GetSteerAngleAttr()))
        out.steerAngle = 0.0f;

    return true;
}

bool parseEngine(EngineDesc& out, const UsdPrim& usdPrim, const void* userData)
{
    const ParseContext& parseContext = *reinterpret_cast<const ParseContext*>(userData);

    out.path = usdPrim.GetPath();

    PhysxSchemaPhysxVehicleEngineAPI engine(usdPrim);

    if (SafeGetAttribute(&out.moi, engine.GetMoiAttr()))
    {
        if (!checkParamInRange(out.moi, FLT_MIN, FLT_MAX, "moi", usdPrim))
            return false;
    }
    else
        out.moi = 1.0f * parseContext.kgmsScale;

    if (getExpectedAttribute(out.peakTorque, engine.GetPeakTorqueAttr(), usdPrim, "peakTorque"))
    {
        if (out.peakTorque != -1.0f)
        {
            if (!checkParamInRange(out.peakTorque, 0.0f, FLT_MAX, "peakTorque", usdPrim))
                return false;
        }
        else
            out.peakTorque = 500.0f * parseContext.kgmsScale;
    }
    else
        return false;

    if (getExpectedAttribute(out.maxRotationSpeed, engine.GetMaxRotationSpeedAttr(), usdPrim, "maxRotationSpeed"))
    {
        if (!checkParamInRange(out.maxRotationSpeed, 0.0f, FLT_MAX, "maxRotationSpeed", usdPrim))
            return false;
    }
    else
        return false;

    if (getExpectedAttribute(out.idleRotationSpeed, engine.GetIdleRotationSpeedAttr(), usdPrim, "idleRotationSpeed"))
    {
        if (!checkParamInRange(out.idleRotationSpeed, 0.0f, FLT_MAX, "idleRotationSpeed", usdPrim))
            return false;
    }
    else
        return false;

    VtArray<GfVec2f> torqueCurve;
    if (SafeGetAttribute(&torqueCurve, engine.GetTorqueCurveAttr()))
    {
        uint32_t torqueCurvePointCount = static_cast<uint32_t>(torqueCurve.size());

        if (torqueCurvePointCount > 0)
        {
            if (torqueCurvePointCount > EngineDesc::maxNumberOfTorqueCurvePoints)
            {
                CARB_LOG_ERROR(
                    "Usd Physics: attribute \"torqueCurve\" of engine \"%s\" has more than %d entries. Entries will "
                    "be culled.\n",
                    usdPrim.GetName().GetText(), EngineDesc::maxNumberOfTorqueCurvePoints);

                torqueCurvePointCount = EngineDesc::maxNumberOfTorqueCurvePoints;
            }

            float xVal = 0.0f;
            for (uint32_t i = 0; i < torqueCurvePointCount; i++)
            {
                if (xVal > torqueCurve[i][0])
                {
                    CARB_LOG_ERROR(
                        "Usd Physics: attribute \"torqueCurve\" of engine \"%s\" has invalid values: first dimension "
                        "expects increasing values in [0, 1].\n",
                        usdPrim.GetName().GetText());
                    return false;
                }

                out.torqueCurve[i].x = torqueCurve[i][0];
                out.torqueCurve[i].y = torqueCurve[i][1];

                xVal = torqueCurve[i][0];
            }

            if (xVal > 1.0f)
            {
                CARB_LOG_ERROR(
                    "Usd Physics: attribute \"torqueCurve\" of engine \"%s\" has invalid values: first dimension "
                    "expects increasing values in [0, 1].\n",
                    usdPrim.GetName().GetText());
                return false;
            }

            out.torqueCurvePointCount = torqueCurvePointCount;
        }
        else
        {
            CARB_LOG_ERROR("Usd Physics: attribute \"torqueCurve\" of engine \"%s\" needs to have at least 1 entry.\n",
                           usdPrim.GetName().GetText());

            return false;
        }
    }
    else
    {
        out.torqueCurve[0].x = 0.0f;
        out.torqueCurve[0].y = 0.8f;
        out.torqueCurve[1].x = 0.33f;
        out.torqueCurve[1].y = 1.0f;
        out.torqueCurve[2].x = 1.0f;
        out.torqueCurve[2].y = 0.8f;

        out.torqueCurvePointCount = 3;
    }

    if (getExpectedAttribute(out.dampingRateFullThrottle, engine.GetDampingRateFullThrottleAttr(),
        usdPrim, "dampingRateFullThrottle"))
    {
        if (out.dampingRateFullThrottle != -1.0f)
        {
            if (!checkParamInRange(out.dampingRateFullThrottle, 0.0f, FLT_MAX, "dampingRateFullThrottle", usdPrim))
                return false;
        }
        else
            out.dampingRateFullThrottle = 0.15f * parseContext.kgmsScale;
    }
    else
        return false;

    if (getExpectedAttribute(out.dampingRateZeroThrottleClutchEngaged, engine.GetDampingRateZeroThrottleClutchEngagedAttr(),
        usdPrim, "dampingRateZeroThrottleClutchEngaged"))
    {
        if (out.dampingRateZeroThrottleClutchEngaged != -1.0f)
        {
            if (!checkParamInRange(out.dampingRateZeroThrottleClutchEngaged, 0.0f, FLT_MAX,
                "dampingRateZeroThrottleClutchEngaged", usdPrim))
                return false;
        }
        else
            out.dampingRateZeroThrottleClutchEngaged = 2.0f * parseContext.kgmsScale;
    }
    else
        return false;

    if (getExpectedAttribute(out.dampingRateZeroThrottleClutchDisengaged, engine.GetDampingRateZeroThrottleClutchDisengagedAttr(),
        usdPrim, "dampingRateZeroThrottleClutchDisengaged"))
    {
        if (out.dampingRateZeroThrottleClutchDisengaged != -1.0f)
        {
            if (!checkParamInRange(out.dampingRateZeroThrottleClutchDisengaged, 0.0f, FLT_MAX,
                "dampingRateZeroThrottleClutchDisengaged", usdPrim))
                return false;
        }
        else
            out.dampingRateZeroThrottleClutchDisengaged = 0.35f * parseContext.kgmsScale;
    }
    else
        return false;

    return true;
}

static constexpr uint32_t maxForwardGearCount = GearsDesc::maxNumberOfGears - 2;  // without reverse and neutral

bool parseGears(GearsDesc& out, const UsdPrim& usdPrim, const void*)
{
    PhysxSchemaPhysxVehicleGearsAPI gears(usdPrim);

    VtArray<float> ratios;
    if (SafeGetAttribute(&ratios, gears.GetRatiosAttr()))
    {
        const uint32_t ratioCount = static_cast<uint32_t>(ratios.size());
        if (ratioCount > 0)
        {
            if (ratioCount > (maxForwardGearCount + 1))
            {
                CARB_LOG_ERROR(
                    "Usd Physics: attribute \"ratios\" of gears \"%s\" only supports one reverse and %d forward gears.\n",
                    usdPrim.GetName().GetText(), maxForwardGearCount);
                return false;
            }

            out.ratios.reserve(ratioCount);

            if (ratios[0] < 0.0f)
                out.ratios.push_back(ratios[0]);
            else
            {
                CARB_LOG_ERROR(
                    "Usd Physics: attribute \"ratios\" of gears \"%s\" has invalid values: first entry for "
                    "reverse gear ratio needs to be negative.\n",
                    usdPrim.GetName().GetText());
                return false;
            }

            float previousRatio = FLT_MAX;
            for (uint32_t i = 1; i < ratioCount; i++)
            {
                if ((ratios[i] > 0.0f) && (ratios[i] < previousRatio))
                {
                    out.ratios.push_back(ratios[i]);
                    previousRatio = ratios[i];
                }
                else
                {
                    CARB_LOG_ERROR(
                        "Usd Physics: attribute \"ratios\" of gears \"%s\" has invalid values: entries for "
                        "forward gear ratios need to be positive and form a descending sequence.\n",
                        usdPrim.GetName().GetText());
                    return false;
                }
            }
        }
        else
        {
            CARB_LOG_ERROR("Usd Physics: attribute \"ratios\" of gears \"%s\" needs to have at least 1 entry.\n",
                           usdPrim.GetName().GetText());

            return false;
        }
    }
    else
    {
        out.ratios.reserve(6);
        out.ratios.push_back(-4.0f);
        out.ratios.push_back(4.0f);
        out.ratios.push_back(2.0f);
        out.ratios.push_back(1.5f);
        out.ratios.push_back(1.1f);
        out.ratios.push_back(1.0f);
    }

    if (SafeGetAttribute(&out.ratioScale, gears.GetRatioScaleAttr()))
    {
        if (!checkParamInRange(out.ratioScale, FLT_MIN, FLT_MAX, "ratioScale", usdPrim))
            return false;
    }
    else
        out.ratioScale = 4.0f;

    if (SafeGetAttribute(&out.switchTime, gears.GetSwitchTimeAttr()))
    {
        if (!checkParamInRange(out.switchTime, 0.0f, FLT_MAX, "switchTime", usdPrim))
            return false;
    }
    else
        out.switchTime = 0.5f;

    return true;
}

bool parseAutoGearBox(AutoGearBoxDesc& out, const UsdPrim& usdPrim, const void* userData)
{
    const uint32_t forwardGearCount = *reinterpret_cast<const uint32_t*>(userData);
    const uint32_t expectedRatioCount = forwardGearCount ? (forwardGearCount - 1) : 0;
    // up ratios: first to second highest gear
    // down ratios: second to highest gear

    PhysxSchemaPhysxVehicleAutoGearBoxAPI autoGearBox(usdPrim);

    VtArray<float> upRatios;
    if (SafeGetAttribute(&upRatios, autoGearBox.GetUpRatiosAttr()))
    {
        const uint32_t ratioCount = static_cast<uint32_t>(upRatios.size());
        if (ratioCount >= expectedRatioCount)
        {
            out.upRatios.reserve(expectedRatioCount);

            for (uint32_t i = 0; i < expectedRatioCount; i++)
            {
                if (upRatios[i] >= 0.0f)
                    out.upRatios.push_back(upRatios[i]);
                else
                {
                    CARB_LOG_ERROR(
                        "Usd Physics: attribute \"upRatios\" of autoGearBox \"%s\" has invalid values: entries must "
                        "not be negative.\n",
                        usdPrim.GetName().GetText());
                    return false;
                }
            }

            if (ratioCount > expectedRatioCount)
            {
                CARB_LOG_WARN(
                    "Usd Physics: attribute \"upRatios\" of autoGearBox \"%s\": entry count is larger than the number "
                    "of relevant gears (first to second highest gear). Extra entries will be ignored.\n",
                    usdPrim.GetName().GetText());
            }
        }
        else
        {
            CARB_LOG_ERROR(
                "Usd Physics: attribute \"upRatios\" of autoGearBox \"%s\": entry count needs to match with the number "
                "of relevant gears (first to second highest gear).\n",
                usdPrim.GetName().GetText());

            return false;
        }
    }
    else
    {
        out.upRatios.reserve(expectedRatioCount);

        for (uint32_t i = 0; i < expectedRatioCount; i++)
        {
            out.upRatios.push_back(0.65f);
        }
    }

    VtArray<float> downRatios;
    if (SafeGetAttribute(&downRatios, autoGearBox.GetDownRatiosAttr()))
    {
        const uint32_t ratioCount = static_cast<uint32_t>(downRatios.size());

        if (ratioCount >= expectedRatioCount)
        {
            out.downRatios.reserve(expectedRatioCount);

            for (uint32_t i = 0; i < expectedRatioCount; i++)
            {
                if (downRatios[i] >= 0.0f)
                    out.downRatios.push_back(downRatios[i]);
                else
                {
                    CARB_LOG_ERROR(
                        "Usd Physics: attribute \"downRatios\" of autoGearBox \"%s\" has invalid values: entries must "
                        "not be negative.\n",
                        usdPrim.GetName().GetText());
                    return false;
                }
            }

            if (ratioCount > expectedRatioCount)
            {
                CARB_LOG_WARN(
                    "Usd Physics: attribute \"downRatios\" of autoGearBox \"%s\": entry count is larger than the number "
                    "of relevant gears (second gear to highest gear). Extra entries will be ignored.\n",
                    usdPrim.GetName().GetText());
            }
        }
        else
        {
            CARB_LOG_ERROR(
                "Usd Physics: attribute \"downRatios\" of autoGearBox \"%s\": entry count needs to match with the number "
                "of specified gears (second gear to highest gear).\n",
                usdPrim.GetName().GetText());
            return false;
        }
    }
    else
    {
        out.downRatios.reserve(expectedRatioCount);

        for (uint32_t i = 0; i < expectedRatioCount; i++)
        {
            out.downRatios.push_back(0.5f);
        }
    }

    if (SafeGetAttribute(&out.latency, autoGearBox.GetLatencyAttr()))
    {
        if (!checkParamInRange(out.latency, 0.0f, FLT_MAX, "latency", usdPrim))
            return false;
    }
    else
        out.latency = 2.0f;

    return true;
}

bool parseClutch(ClutchDesc& out, const UsdPrim& usdPrim, const void* userData)
{
    const ParseContext& parseContext = *reinterpret_cast<const ParseContext*>(userData);

    PhysxSchemaPhysxVehicleClutchAPI clutch(usdPrim);

    if (SafeGetAttribute(&out.strength, clutch.GetStrengthAttr()))
    {
        if (!checkParamInRange(out.strength, FLT_MIN, FLT_MAX, "strength", usdPrim))
            return false;
    }
    else
        out.strength = 10.0f * parseContext.kgmsScale;

    return true;
}

static bool parseNonlinearCommandResponseInstance(NonlinearCmdResponseDesc& out,
    const PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI& nonlinearCmdResponseAPI,
    const void* userData)
{
    UsdPrim usdPrim = nonlinearCmdResponseAPI.GetPrim();

#define CMD_VAL_ERR_PREFIX "Usd Physics: attribute \"commandValues\" of nonlinear command response \"%s\" "

    VtArray<float> commandValues;
    if (SafeGetAttribute(&commandValues, nonlinearCmdResponseAPI.GetCommandValuesAttr()))
    {
        if (commandValues.size() > NonlinearCmdResponseDesc::maxNumberOfCommandValues)
        {
            CARB_LOG_ERROR(
                CMD_VAL_ERR_PREFIX
                "has more than the maximum allowed number of %d entries.\n",
                usdPrim.GetName().GetText(), NonlinearCmdResponseDesc::maxNumberOfCommandValues);

            return false;
        }

        float lastCommandValue = -1.0f;
        out.commandValues.reserve(commandValues.size());
        for (float cmdValue : commandValues)
        {
            if ((cmdValue < 0.0f) || (cmdValue > 1.0f))
            {
                CARB_LOG_ERROR(
                    CMD_VAL_ERR_PREFIX
                    "expects values in range [0, 1].\n",
                    usdPrim.GetName().GetText());

                return false;
            }

            if (cmdValue <= lastCommandValue)
            {
                CARB_LOG_ERROR(
                    CMD_VAL_ERR_PREFIX
                    "expects strictly increasing entries.\n",
                    usdPrim.GetName().GetText());

                return false;
            }

            out.commandValues.push_back(cmdValue);

            lastCommandValue = cmdValue;
        }
    }

#define SP_RESP_PER_CV_ERR_PREFIX "Usd Physics: attribute \"speedResponsesPerCommandValue\" of nonlinear command response \"%s\" "

    VtArray<int> speedResponsesPerCommandValue;
    if (SafeGetAttribute(&speedResponsesPerCommandValue, nonlinearCmdResponseAPI.GetSpeedResponsesPerCommandValueAttr()))
    {
        if (speedResponsesPerCommandValue.size() != commandValues.size())
        {
            CARB_LOG_ERROR(
                SP_RESP_PER_CV_ERR_PREFIX
                "needs to have the same number of entries as the \"commandValues\" attribute.\n",
                usdPrim.GetName().GetText());

            return false;
        }
    }

#define SP_RESP_ERR_PREFIX "Usd Physics: attribute \"speedResponses\" of nonlinear command response \"%s\" "

    VtArray<GfVec2f> speedResponses;
    if (SafeGetAttribute(&speedResponses, nonlinearCmdResponseAPI.GetSpeedResponsesAttr()))
    {
        if (speedResponses.size() > NonlinearCmdResponseDesc::maxNumberOfSpeedResponses)
        {
            CARB_LOG_ERROR(
                SP_RESP_ERR_PREFIX
                "has more than the maximum allowed number of %d entries.\n",
                usdPrim.GetName().GetText(), NonlinearCmdResponseDesc::maxNumberOfSpeedResponses);

            return false;
        }
    }

    {
        int lastIndex = -1;
        out.speedResponsesPerCommandValue.reserve(speedResponsesPerCommandValue.size());
        for (int index : speedResponsesPerCommandValue)
        {
            if ((index < 0) || ((index + 1) > speedResponses.size()))
            {
                CARB_LOG_ERROR(
                    SP_RESP_PER_CV_ERR_PREFIX
                    "expects values in range [0, %d).\n",
                    usdPrim.GetName().GetText(), static_cast<int>(speedResponses.size()));

                return false;
            }

            if (index <= lastIndex)  // at least one entry per command value is expected
            {
                CARB_LOG_ERROR(
                    SP_RESP_PER_CV_ERR_PREFIX
                    "expects strictly increasing entries.\n",
                    usdPrim.GetName().GetText());

                return false;
            }

            out.speedResponsesPerCommandValue.push_back(index);

            lastIndex = index;
        }
    }

    {
        float lastSpeedValue = -FLT_MAX;
        out.speedResponses.reserve(speedResponses.size());
        int speedGraphIndex = 1;
        int entryIndex = 0;
        for (GfVec2f sp : speedResponses)
        {
            if ((sp[1] < 0.0f) || (sp[1] > 1.0f))
            {
                CARB_LOG_ERROR(
                    SP_RESP_ERR_PREFIX
                    "expects response values in range [0, 1].\n",
                    usdPrim.GetName().GetText());

                return false;
            }

            if (sp[0] <= lastSpeedValue)
            {
                CARB_LOG_ERROR(
                    SP_RESP_ERR_PREFIX
                    "expects strictly increasing speed entries.\n",
                    usdPrim.GetName().GetText());

                return false;
            }

            out.speedResponses.push_back(carb::Float2{ sp[0], sp[1] });

            entryIndex++;

            if ((speedGraphIndex >= speedResponsesPerCommandValue.size()) || (entryIndex < speedResponsesPerCommandValue[speedGraphIndex]))
            {
                lastSpeedValue = sp[0];
            }
            else
            {
                lastSpeedValue = -FLT_MAX;
            }
        }
    }

    return true;
}

static bool parseNonlinearCommandResponse(NonlinearCmdResponseDesc*& out,
    const UsdPrim& usdPrim,
    const TfToken& instanceName,
    const ParseContext& parseContext)
{
    if (usdPrim.HasAPI<PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI>())
    {
        const PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI nonlinCmdResponseAPI = PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI::Get(
            usdPrim, instanceName);

        if (nonlinCmdResponseAPI)
        {
            if (!parseComponent<PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI, NonlinearCmdResponseDesc, NonlinearCmdResponseDesc>(
                nonlinCmdResponseAPI,
                parseContext.vehicleComponentTracker, parseContext.vehicleComponentTracker.mNonlinearCmdResponses,
                parseNonlinearCommandResponseInstance, &parseContext, "nonlinearCommandResponse", out))
            {
                return false;
            }
        }
        else
        {
            out = nullptr;

            // note: using an unexpected instance name is not treated as error but the API will be ignored
            //       since the same prim might have different instances of this API applied.
        }
    }
    else
        out = nullptr;

    return true;
}

bool parseDriveBasic(DriveBasicDesc& out, const UsdPrim& usdPrim, const void* userData)
{
    CARB_ASSERT(usdPrim.HasAPI<PhysxSchemaPhysxVehicleDriveBasicAPI>());

    const ParseContext& parseContext = *reinterpret_cast<const ParseContext*>(userData);

    const PhysxSchemaPhysxVehicleDriveBasicAPI drive(usdPrim);

    out.path = usdPrim.GetPath();

    if (getExpectedAttribute(out.peakTorque, drive.GetPeakTorqueAttr(), usdPrim, "peakTorque"))
    {
        if (out.peakTorque != -1.0f)
        {
            if (!checkParamInRange(out.peakTorque, 0.0f, FLT_MAX, "peakTorque", usdPrim))
                return false;
        }
        else
            out.peakTorque = 1000.0f * parseContext.kgmsScale;
    }
    else
        return false;

    return parseNonlinearCommandResponse(out.nonlinearCmdResponse, usdPrim, PhysxSchemaTokens->drive,
        parseContext);
}

bool parseDriveStandard(DriveStandardDesc& out, const UsdPrim& usdPrim, const void* userData)
{
    CARB_ASSERT(usdPrim.HasAPI<PhysxSchemaPhysxVehicleDriveStandardAPI>());

    const ParseContext& parseContext = *reinterpret_cast<const ParseContext*>(userData);

    const PhysxSchemaPhysxVehicleDriveStandardAPI drive(usdPrim);

    if (!parseRelationshipOrAPI<EngineDesc, EngineDesc, PhysxSchemaPhysxVehicleEngineAPI, false>(
            parseContext.stage, usdPrim, drive.GetEngineRel(), parseContext.vehicleComponentTracker,
            parseContext.vehicleComponentTracker.mEngines, parseEngine, &parseContext,
            "vehicle drive", "engine", "PhysxVehicleEngineAPI", out.engine))
    {
        return false;
    }

    if (!parseRelationshipOrAPI<GearsDesc, const GearsDesc, PhysxSchemaPhysxVehicleGearsAPI, false>(
            parseContext.stage, usdPrim, drive.GetGearsRel(), parseContext.vehicleComponentTracker,
            parseContext.vehicleComponentTracker.mGears, parseGears, nullptr,
            "vehicle drive", "gears", "PhysxVehicleGearsAPI", out.gears))
    {
        return false;
    }

    const UsdRelationship autoGearBoxRel = drive.GetAutoGearBoxRel();
    out.autoGearBox = nullptr;
    {
        const uint32_t forwardGearCount = static_cast<uint32_t>(out.gears->ratios.size()) - 1;

        if (!parseRelationshipOrAPI<AutoGearBoxDesc, const AutoGearBoxDesc, PhysxSchemaPhysxVehicleAutoGearBoxAPI, true>(
                parseContext.stage, usdPrim, autoGearBoxRel, parseContext.vehicleComponentTracker,
                parseContext.vehicleComponentTracker.mAutoGearBoxes, parseAutoGearBox, &forwardGearCount, "vehicle drive",
                "autoGearBox", "PhysxVehicleAutoGearBoxAPI", out.autoGearBox))
        {
            return false;
        }
    }

    if (!parseRelationshipOrAPI<ClutchDesc, const ClutchDesc, PhysxSchemaPhysxVehicleClutchAPI, false>(
            parseContext.stage, usdPrim, drive.GetClutchRel(), parseContext.vehicleComponentTracker,
            parseContext.vehicleComponentTracker.mClutches, parseClutch, &parseContext,
            "vehicle drive", "clutch", "PhysxVehicleClutchAPI", out.clutch))
    {
        return false;
    }

    return true;
}

static bool parseDrive(const pxr::UsdPrim& usdPrim, const PhysxSchemaPhysxVehicleAPI& usdVehicleAPI,
    const ParseContext& parseContext,
    DriveDesc*& out)
{
    SdfPath drivePath;
    uint32_t errCode = getSingleRelationshipPath(usdVehicleAPI.GetDriveRel(), drivePath, "drive", usdPrim);
    if (errCode == 0)
    {
        const UsdPrim drivePrim = parseContext.stage->GetPrimAtPath(drivePath);
        if (drivePrim)
        {
            if (drivePrim.HasAPI<PhysxSchemaPhysxVehicleDriveStandardAPI>())
            {
                return parseComponentWithMapCheck<DriveStandardDesc, DriveDesc>(drivePrim, drivePath,
                    parseContext.vehicleComponentTracker, parseContext.vehicleComponentTracker.mDrivesStandard,
                    parseDriveStandard, &parseContext, "drive", out);
            }
            else if (drivePrim.HasAPI<PhysxSchemaPhysxVehicleDriveBasicAPI>())
            {
                return parseComponentWithMapCheck<DriveBasicDesc, DriveDesc>(drivePrim, drivePath,
                    parseContext.vehicleComponentTracker, parseContext.vehicleComponentTracker.mDrivesBasic,
                    parseDriveBasic, &parseContext, "drive", out);
            }
            else
            {
                CARB_LOG_ERROR("Usd Physics: vehicle \"%s\": \"drive\" relationship must point to prim with "
                    "PhysxVehicleDriveBasicAPI or PhysxVehicleDriveStandardAPI applied.\n",
                    usdPrim.GetName().GetText());
                return false;
            }
        }
        else
        {
            CARB_LOG_ERROR("Usd Physics: vehicle \"%s\": \"drive\" relationship has nonexistent path \"%s\".\n",
                            usdPrim.GetName().GetText(), drivePath.GetText());
            return false;
        }
    }
    else if (errCode == 1)
    {
        if (usdPrim.HasAPI<PhysxSchemaPhysxVehicleDriveStandardAPI>())
        {
            if (!usdPrim.HasAPI<PhysxSchemaPhysxVehicleDriveBasicAPI>())
            {
                return parseComponentWithMapCheck<DriveStandardDesc, DriveDesc>(usdPrim, usdPrim.GetPath(),
                    parseContext.vehicleComponentTracker, parseContext.vehicleComponentTracker.mDrivesStandard,
                    parseDriveStandard, &parseContext, "drive", out);
            }
            else
            {
                CARB_LOG_ERROR("Usd Physics: vehicle \"%s\" has both PhysxSchemaPhysxVehicleDriveStandardAPI and "
                    "PhysxSchemaPhysxVehicleDriveBasicAPI applied. Only one is allowed.\n",
                    usdPrim.GetName().GetText());
                return false;
            }
        }
        else if (usdPrim.HasAPI<PhysxSchemaPhysxVehicleDriveBasicAPI>())
        {
            return parseComponentWithMapCheck<DriveBasicDesc, DriveDesc>(usdPrim, usdPrim.GetPath(),
                parseContext.vehicleComponentTracker, parseContext.vehicleComponentTracker.mDrivesBasic,
                parseDriveBasic, &parseContext, "drive", out);
        }
        else
        {
            out = nullptr;
            return true;
        }
    }
    else
        return false;
}

bool parseMultiWheelDifferential(MultiWheelDifferentialDesc& out, const UsdPrim& usdPrim, const void*)
{
    PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI mwDifferential(usdPrim);

    VtArray<int> wheels;
    if (SafeGetAttribute(&wheels, mwDifferential.GetWheelsAttr()))
    {
        out.wheels.reserve(wheels.size());
        for (int wheelIndex : wheels)
        {
            if (wheelIndex < 0)
            {
                CARB_LOG_ERROR(
                    "Usd Physics: attribute \"wheels\" of differential \"%s\" can not hold negative values.\n",
                    usdPrim.GetName().GetText());

                return false;
            }

            out.wheels.push_back(wheelIndex);
        }
    }

    VtArray<float> values;
    if (SafeGetAttribute(&values, mwDifferential.GetTorqueRatiosAttr()))
    {
        const size_t ratioCount = values.size();
        if (ratioCount)
        {
            if (ratioCount != out.wheels.size())
            {
                CARB_LOG_ERROR(
                    "Usd Physics: attribute \"torqueRatios\" of differential \"%s\" needs to have the same number of entries as the "
                    "\"wheels\" attribute or else should not be defined at all.\n",
                    usdPrim.GetName().GetText());

                return false;
            }

            out.torqueRatios.reserve(ratioCount);

            for (float ratio : values)
            {
                if (!checkParamInRange(ratio, -1.0f, 1.0f + FLT_EPSILON, "torqueRatios", usdPrim))
                    return false;

                out.torqueRatios.push_back(ratio);
            }
        }
    }

    values.clear();
    if (SafeGetAttribute(&values, mwDifferential.GetAverageWheelSpeedRatiosAttr()))
    {
        const size_t ratioCount = values.size();
        if (ratioCount)
        {
            if (ratioCount != out.wheels.size())
            {
                CARB_LOG_ERROR(
                    "Usd Physics: attribute \"averageWheelSpeedRatios\" of differential \"%s\" needs to have the same number of entries as the "
                    "\"wheels\" attribute or else should not be defined at all.\n",
                    usdPrim.GetName().GetText());

                return false;
            }

            out.averageWheelSpeedRatios.reserve(ratioCount);

            for (float ratio : values)
            {
                if (!checkParamInRange(ratio, 0.0f, 1.0f + FLT_EPSILON, "averageWheelSpeedRatios", usdPrim))
                    return false;

                out.averageWheelSpeedRatios.push_back(ratio);
            }
        }
    }

    return true;
}

bool parseTankDifferential(TankDifferentialDesc& out, const UsdPrim& usdPrim, const void* userData)
{
    CARB_ASSERT(userData == nullptr);

    if (parseMultiWheelDifferential(out, usdPrim, userData))
    {
        PhysxSchemaPhysxVehicleTankDifferentialAPI tDifferential(usdPrim);

        VtArray<int> values;

        size_t trackCount;
        if (SafeGetAttribute(&values, tDifferential.GetNumberOfWheelsPerTrackAttr()))
        {
            trackCount = values.size();
            out.numberOfWheelsPerTrack.reserve(trackCount);

            if (trackCount > VehicleDesc::maxNumberOfWheels)
            {
                CARB_LOG_ERROR(
                    "Usd Physics: attribute \"numberOfWheelsPerTrack\" of differential \"%s\" can not have more than %d entries.\n",
                    usdPrim.GetName().GetText(), VehicleDesc::maxNumberOfWheels);

                return false;
            }

            for (int numberOfWheels : values)
            {
                if (numberOfWheels < 0)
                {
                    CARB_LOG_ERROR(
                        "Usd Physics: attribute \"numberOfWheelsPerTrack\" of differential \"%s\": entries must not be negative.\n",
                        usdPrim.GetName().GetText());

                    return false;
                }

                out.numberOfWheelsPerTrack.push_back(numberOfWheels);
            }
        }
        else
        {
            trackCount = 0;
        }

        values.clear();
        if (SafeGetAttribute(&values, tDifferential.GetThrustIndexPerTrackAttr()))
        {
            out.thrustIndexPerTrack.reserve(values.size());

            for (int thrustIndex : values)
            {
                if ((thrustIndex < 0) || (thrustIndex > 1))
                {
                    CARB_LOG_ERROR(
                        "Usd Physics: attribute \"thrustIndexPerTrack\" of differential \"%s\" expects values that are 0 or 1.\n",
                        usdPrim.GetName().GetText());

                    return false;
                }

                out.thrustIndexPerTrack.push_back(thrustIndex);
            }
        }
        if (values.size() != trackCount)
        {
            CARB_LOG_ERROR(
                "Usd Physics: attribute \"thrustIndexPerTrack\" of differential \"%s\" needs to have the same number of entries "
                "as the \"numberOfWheelsPerTrack\" attribute.\n",
                usdPrim.GetName().GetText());

            return false;
        }

        size_t numberOfEntriesInWheelIndexList;
        values.clear();
        if (SafeGetAttribute(&values, tDifferential.GetWheelIndicesInTrackOrderAttr()))
        {
            numberOfEntriesInWheelIndexList = values.size();

            if (numberOfEntriesInWheelIndexList > VehicleDesc::maxNumberOfWheels)
            {
                CARB_LOG_ERROR(
                    "Usd Physics: attribute \"wheelIndicesInTrackOrder\" of differential \"%s\" can not have more than %d entries.\n",
                    usdPrim.GetName().GetText(), VehicleDesc::maxNumberOfWheels);

                return false;
            }

            out.wheelIndicesInTrackOrder.reserve(numberOfEntriesInWheelIndexList);

            for (int wheelIndex : values)
            {
                out.wheelIndicesInTrackOrder.push_back(wheelIndex);
            }
        }
        else
        {
            numberOfEntriesInWheelIndexList = 0;
        }

        values.clear();
        if (SafeGetAttribute(&values, tDifferential.GetTrackToWheelIndicesAttr()))
        {
            const int trackToWheelIndicesCount = static_cast<int>(values.size());
            out.trackToWheelIndices.reserve(trackToWheelIndicesCount);

            static_assert(VehicleDesc::maxNumberOfWheels <= 32, "");  // using bit logic to track the indices
            uint32_t encounteredIndices = 0;
            int nextMinStartIndex = 0;
            for (int i = 0; i < trackToWheelIndicesCount; i++)
            {
                int startIndex = values[i];

                if (startIndex >= nextMinStartIndex)
                {
                    if (i < trackCount)
                    {
                        const int numberOfWheelsPerTrack = out.numberOfWheelsPerTrack[i];
                        nextMinStartIndex = startIndex + numberOfWheelsPerTrack;

                        if (nextMinStartIndex <= numberOfEntriesInWheelIndexList)
                        {
                            for (int j = startIndex; j < nextMinStartIndex; j++)
                            {
                                int wheelIndex = out.wheelIndicesInTrackOrder[j];

                                if (wheelIndex < 0)
                                {
                                    CARB_LOG_ERROR(
                                        "Usd Physics: attribute \"wheelIndicesInTrackOrder\" of differential \"%s\" can not have negative "
                                        "entries.\n",
                                        usdPrim.GetName().GetText());

                                    return false;
                                }

                                uint32_t indexBit = 1 << wheelIndex;
                                if (!(encounteredIndices & indexBit))
                                {
                                    encounteredIndices |= indexBit;
                                }
                                else
                                {
                                    CARB_LOG_ERROR(
                                        "Usd Physics: attribute \"wheelIndicesInTrackOrder\" of differential \"%s\" can not contain the "
                                        "same index multiple times as a wheel can only be assigned to one track.\n",
                                        usdPrim.GetName().GetText());

                                    return false;
                                }
                            }
                        }
                        else
                        {
                            CARB_LOG_ERROR(
                                "Usd Physics: entries in attributes \"trackToWheelIndices\" and \"numberOfWheelsPerTrack\" of differential \"%s\" "
                                "form a range that points outside the range of entries in the \"wheelIndicesInTrackOrder\".\n",
                                usdPrim.GetName().GetText());

                            return false;
                        }
                    }
                    // note: matching track count is tested outside the root if-block

                    out.trackToWheelIndices.push_back(startIndex);
                }
                else
                {
                    // wheelIndicesInTrackOrder is in track order, thus the indices must not decrease.

                    CARB_LOG_ERROR(
                        "Usd Physics: attribute \"trackToWheelIndices\" of differential \"%s\" expects positive values and "
                        "a sequence of non decreasing index values.\n",
                        usdPrim.GetName().GetText());

                    return false;
                }
            }
        }
        if (values.size() != trackCount)
        {
            CARB_LOG_ERROR(
                "Usd Physics: attribute \"trackToWheelIndices\" of differential \"%s\" needs to have the same number of entries "
                "as the \"numberOfWheelsPerTrack\" attribute.\n",
                usdPrim.GetName().GetText());

            return false;
        }
    }
    else
        return false;

    return true;
}

static bool parseDifferential(MultiWheelDifferentialDesc*& out,
    const UsdPrim& usdPrim,
    const ParseContext& parseContext)
{
    if (usdPrim.HasAPI<PhysxSchemaPhysxVehicleTankDifferentialAPI>())
    {
        return parseComponent<UsdPrim, TankDifferentialDesc, MultiWheelDifferentialDesc>(usdPrim,
            parseContext.vehicleComponentTracker, parseContext.vehicleComponentTracker.mTankDifferentials,
            parseTankDifferential, nullptr, "differential", out);
    }
    else if (usdPrim.HasAPI<PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI>())
    {
        return parseComponent<UsdPrim, MultiWheelDifferentialDesc, MultiWheelDifferentialDesc>(usdPrim,
            parseContext.vehicleComponentTracker, parseContext.vehicleComponentTracker.mMultiWheelDifferentials,
            parseMultiWheelDifferential, nullptr, "differential", out);
    }
    else
    {
        out = nullptr;
        return true;
    }
}

static bool parseBrakesInstance(BrakesDesc& out,
    const PhysxSchemaPhysxVehicleBrakesAPI& brakesAPI,
    const void* userData)
{
    UsdPrim usdPrim = brakesAPI.GetPrim();

    if (getExpectedAttribute(out.maxBrakeTorque, brakesAPI.GetMaxBrakeTorqueAttr(),
            usdPrim, "maxBrakeTorque"))
    {
        if (!checkParamInRange(out.maxBrakeTorque, 0.0f, FLT_MAX, "maxBrakeTorque", usdPrim))
            return false;
    }
    else
        return false;

    VtArray<int> wheels;
    if (SafeGetAttribute(&wheels, brakesAPI.GetWheelsAttr()))
    {
        out.wheels.reserve(wheels.size());
        for (int wheelIndex : wheels)
        {
            if (wheelIndex < 0)
            {
                CARB_LOG_ERROR(
                    "Usd Physics: attribute \"wheels\" of braking system \"%s\" can not hold negative values.\n",
                    brakesAPI.GetName().GetText());

                return false;
            }

            out.wheels.push_back(wheelIndex);
        }
    }

    VtArray<float> values;
    if (SafeGetAttribute(&values, brakesAPI.GetTorqueMultipliersAttr()))
    {
        const size_t multiplierCount = values.size();
        if (multiplierCount)
        {
            if (multiplierCount != out.wheels.size())
            {
                CARB_LOG_ERROR(
                    "Usd Physics: attribute \"torqueMultipliers\" of braking system \"%s\" needs to have the same "
                    "number of entries as the \"wheels\" attribute or else should not be defined at all.\n",
                    brakesAPI.GetName().GetText());

                return false;
            }

            out.torqueMultipliers.reserve(multiplierCount);

            for (float multiplier : values)
            {
                if (multiplier < 0.0f)
                {
                    CARB_LOG_ERROR(
                        "Usd Physics: attribute \"torqueMultipliers\" of braking system \"%s\" can not hold "
                        "negative values.\n", brakesAPI.GetName().GetText());

                    return false;
                }

                out.torqueMultipliers.push_back(multiplier);
            }
        }
    }

    const ParseContext& parseContext = *reinterpret_cast<const ParseContext*>(userData);

    return parseNonlinearCommandResponse(out.nonlinearCmdResponse, usdPrim, brakesAPI.GetName(),  // this is the instance name and the same is used for nonlinearCmdResponse
        parseContext);
}

static bool parseBrakes(std::vector<const BrakesDesc*>& out,
    const UsdPrim& usdPrim,
    const ParseContext& parseContext)
{
    if (usdPrim.HasAPI<PhysxSchemaPhysxVehicleBrakesAPI>())
    {
        TfToken brakesTokens[] = { PhysxSchemaTokens->brakes0, PhysxSchemaTokens->brakes1 };
        const uint32_t brakesTokensCount = sizeof(brakesTokens) / sizeof(brakesTokens[0]);

        for (uint32_t i = 0; i < brakesTokensCount; i++)
        {
            BrakesDesc* brakesDesc;
            const PhysxSchemaPhysxVehicleBrakesAPI brakesAPI = PhysxSchemaPhysxVehicleBrakesAPI::Get(usdPrim, brakesTokens[i]);
            if (brakesAPI)
            {
                if (parseComponent<PhysxSchemaPhysxVehicleBrakesAPI, BrakesDesc, BrakesDesc>(brakesAPI,
                        parseContext.vehicleComponentTracker, parseContext.vehicleComponentTracker.mBrakes,
                        parseBrakesInstance, &parseContext, "brakes", brakesDesc))
                {
                    brakesDesc->brakesIndex = i;
                    out.push_back(brakesDesc);
                }
                else
                    return false;
            }
        }

        if (!out.size())
        {
            CARB_LOG_ERROR("Usd Physics: \"%s\": PhysxVehicleBrakesAPI is applied but no valid instance token could "
                "be found.\n", usdPrim.GetName().GetText());

            return false;
        }
    }
    else
    {
        // it's valid to not have brakes API applied
        return true;
    }

    return true;
}

static bool checkMaxSteerAngleWithMultiplier(const float maxSteerAngle, const float multiplier,
    const char* paramName, const UsdPrim& usdPrim)
{
    const float maxAngle = maxSteerAngle * multiplier;
    if (fabsf(maxAngle) <= static_cast<float>(M_PI))
    {
        return true;
    }
    else
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\": maxSteerAngle * angleMultiplier "
            "has to in range [-pi, pi] for all steered wheels.\n",
            usdPrim.GetName().GetText(), paramName);

        return false;
    }
}

static bool parseSteeringBasic(SteeringBasicDesc& out, const UsdPrim& usdPrim, const void* userData)
{
    CARB_ASSERT(usdPrim.HasAPI<PhysxSchemaPhysxVehicleSteeringAPI>());

    const PhysxSchemaPhysxVehicleSteeringAPI steeringAPI(usdPrim);

    if (!getExpectedAttribute(out.maxSteerAngle, steeringAPI.GetMaxSteerAngleAttr(),
            usdPrim, "maxSteerAngle"))
    {
        return false;
    }

    VtArray<int> wheels;
    if (SafeGetAttribute(&wheels, steeringAPI.GetWheelsAttr()))
    {
        out.wheels.reserve(wheels.size());
        for (int wheelIndex : wheels)
        {
            if (wheelIndex < 0)
            {
                CARB_LOG_ERROR(
                    "Usd Physics: attribute \"wheels\" of steering system \"%s\" can not hold negative values.\n",
                    usdPrim.GetName().GetText());

                return false;
            }

            out.wheels.push_back(wheelIndex);
        }
    }

    VtArray<float> values;
    if (SafeGetAttribute(&values, steeringAPI.GetAngleMultipliersAttr()))
    {
        const size_t multiplierCount = values.size();
        if (multiplierCount)
        {
            if (multiplierCount != out.wheels.size())
            {
                CARB_LOG_ERROR(
                    "Usd Physics: attribute \"angleMultipliers\" of steering system \"%s\" needs to have the same "
                    "number of entries as the \"wheels\" attribute or else should not be defined at all.\n",
                    usdPrim.GetName().GetText());

                return false;
            }

            out.angleMultipliers.reserve(multiplierCount);

            for (float multiplier : values)
            {
                if (!checkMaxSteerAngleWithMultiplier(out.maxSteerAngle, multiplier,
                    "angleMultipliers", usdPrim))
                    return false;

                out.angleMultipliers.push_back(multiplier);
            }
        }
    }

    const ParseContext& parseContext = *reinterpret_cast<const ParseContext*>(userData);

    return parseNonlinearCommandResponse(out.nonlinearCmdResponse, usdPrim, PhysxSchemaTokens->steer,
        parseContext);
}

static bool parseSteeringAckermann(SteeringAckermannDesc& out, const UsdPrim& usdPrim, const void* userData)
{
    CARB_ASSERT(usdPrim.HasAPI<PhysxSchemaPhysxVehicleAckermannSteeringAPI>());

    const PhysxSchemaPhysxVehicleAckermannSteeringAPI steeringAPI(usdPrim);

    if (!getExpectedAttribute(out.maxSteerAngle, steeringAPI.GetMaxSteerAngleAttr(),
            usdPrim, "maxSteerAngle"))
    {
        return false;
    }

    if (getExpectedAttribute(out.wheel0, steeringAPI.GetWheel0Attr(), usdPrim,
        "wheel0"))
    {
        if (!checkParamIsNonNegative(out.wheel0, "wheel0", usdPrim))
            return false;
    }
    else
        return false;

    if (getExpectedAttribute(out.wheel1, steeringAPI.GetWheel1Attr(), usdPrim,
        "wheel1"))
    {
        if (!checkParamIsNonNegative(out.wheel1, "wheel1", usdPrim))
            return false;
    }
    else
        return false;

    if (out.wheel0 == out.wheel1)
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attributes \"wheel0\" and \"wheel1\" can not hold the "
            "same value.\n",
            usdPrim.GetName().GetText());

        return false;
    }

    if (getExpectedAttribute(out.wheelBase, steeringAPI.GetWheelBaseAttr(), usdPrim,
        "wheelBase"))
    {
        if (!checkParamIsPositive(out.wheelBase, "wheelBase", usdPrim))
            return false;
    }
    else
        return false;

    if (getExpectedAttribute(out.trackWidth, steeringAPI.GetTrackWidthAttr(), usdPrim,
        "trackWidth"))
    {
        if (!checkParamIsPositive(out.trackWidth, "trackWidth", usdPrim))
            return false;
    }
    else
        return false;

    if (SafeGetAttribute(&out.strength, steeringAPI.GetStrengthAttr()))
    {
        if (!checkParamInRange(out.strength, 0.0f, 1.0f + FLT_EPSILON, "strength", usdPrim))
            return false;
    }
    else
        out.strength = 1.0f;

    const ParseContext& parseContext = *reinterpret_cast<const ParseContext*>(userData);

    return parseNonlinearCommandResponse(out.nonlinearCmdResponse, usdPrim, PhysxSchemaTokens->steer,
        parseContext);
}

static bool parseSteering(SteeringDesc*& out,
    const UsdPrim& usdPrim,
    const ParseContext& parseContext)
{
    if (usdPrim.HasAPI<PhysxSchemaPhysxVehicleSteeringAPI>())
    {
        return parseComponent<UsdPrim, SteeringBasicDesc, SteeringDesc>(usdPrim,
            parseContext.vehicleComponentTracker, parseContext.vehicleComponentTracker.mSteeringBasic,
            parseSteeringBasic, &parseContext, "steering", out);
    }
    else if (usdPrim.HasAPI<PhysxSchemaPhysxVehicleAckermannSteeringAPI>())
    {
        return parseComponent<UsdPrim, SteeringAckermannDesc, SteeringDesc>(usdPrim,
            parseContext.vehicleComponentTracker, parseContext.vehicleComponentTracker.mSteeringAckermann,
            parseSteeringAckermann, &parseContext, "steering", out);
    }
    else
    {
        // it's valid to not have steering API applied
        out = nullptr;
        return true;
    }
}

static bool getAxisDir(const TfToken& axisToken, VehicleContextDesc::AxisDir& axisEnum)
{
    if (axisToken == PhysxSchemaTokens.Get()->posX)
        axisEnum = VehicleContextDesc::ePosX;
    else if (axisToken == PhysxSchemaTokens.Get()->negX)
        axisEnum = VehicleContextDesc::eNegX;
    else if (axisToken == PhysxSchemaTokens.Get()->posY)
        axisEnum = VehicleContextDesc::ePosY;
    else if (axisToken == PhysxSchemaTokens.Get()->negY)
        axisEnum = VehicleContextDesc::eNegY;
    else if (axisToken == PhysxSchemaTokens.Get()->posZ)
        axisEnum = VehicleContextDesc::ePosZ;
    else if (axisToken == PhysxSchemaTokens.Get()->negZ)
        axisEnum = VehicleContextDesc::eNegZ;
    else
    {
        CARB_LOG_ERROR("Usd Physics: PhysxVehicleContextAPI encountered invalid axis token \"%s\".",
            axisToken.GetText());
        return false;
    }

    return true;
}

bool parseVehicleContext(const pxr::UsdPrim& usdPrim, VehicleContextDesc& vehicleContextDesc)
{
    CARB_ASSERT(usdPrim.HasAPI<PhysxSchemaPhysxVehicleContextAPI>());

    const PhysxSchemaPhysxVehicleContextAPI usdVehicleContextAPI(usdPrim);

    if (!usdPrim.IsA<UsdPhysicsScene>())
    {
        CARB_LOG_ERROR("Usd Physics: \"%s\": PhysxVehicleContextAPI requires to be applied to a "
            "PhysicsScene prim.\n", usdPrim.GetName().GetText());
        return false;
    }

    vehicleContextDesc.scenePath = usdPrim.GetPath(); // the vehicle context has to be applied to the scene prim

    TfToken updateMode;
    vehicleContextDesc.vehicleUpdateMode = eAcceleration;

    if (SafeGetAttribute(&updateMode, usdVehicleContextAPI.GetUpdateModeAttr()))
    {
        if (updateMode == PhysxSchemaTokens.Get()->velocityChange)
        {
            vehicleContextDesc.vehicleUpdateMode = eVelocityChange;
        }
    }
    else
    {
        CARB_LOG_ERROR("Usd Physics: \"%s\" needs to have token \"updateMode\" defined.\n", usdPrim.GetName().GetText());
        return false;
    }

    TfToken axis;
    if (getExpectedAttribute(
            axis, usdVehicleContextAPI.GetVerticalAxisAttr(), usdPrim, "verticalAxis"))
    {
        if (axis != pxr::PhysxSchemaTokens.Get()->undefined)
        {
            if (!getAxisDir(axis, vehicleContextDesc.verticalAxis))
                return false;
        }
        else
        {
            vehicleContextDesc.verticalAxis = VehicleContextDesc::eUndefined;  // marker to refer to deprecated value instead

            if (getExpectedAttribute(
                    vehicleContextDesc.upAxis, usdVehicleContextAPI.GetUpAxisAttr(), usdPrim, "upAxis"))
            {
                CARB_LOG_WARN("Usd Physics: vehicle context \"%s\": attribute \"upAxis\" is deprecated. "
                    "Please use verticalAxis instead.\n",
                    usdPrim.GetName().GetText());
            }
            else
                return false;
        }
    }
    else
        return false;

    if (getExpectedAttribute(
            axis, usdVehicleContextAPI.GetLongitudinalAxisAttr(), usdPrim, "longitudinalAxis"))
    {
        if (axis != pxr::PhysxSchemaTokens.Get()->undefined)
        {
            if (!getAxisDir(axis, vehicleContextDesc.longitudinalAxis))
                return false;
        }
        else
        {
            vehicleContextDesc.longitudinalAxis = VehicleContextDesc::eUndefined;  // marker to refer to deprecated value instead

            if (getExpectedAttribute(
                vehicleContextDesc.forwardAxis, usdVehicleContextAPI.GetForwardAxisAttr(), usdPrim, "forwardAxis"))
            {
                CARB_LOG_WARN("Usd Physics: vehicle context \"%s\": attribute \"forwardAxis\" is deprecated. "
                    "Please use longitudinalAxis instead.\n",
                    usdPrim.GetName().GetText());
            }
            else
                return false;
        }
    }
    else
        return false;

    // to ensure the code further below stays valid
    static_assert(VehicleContextDesc::ePosX == 0, "");
    static_assert(VehicleContextDesc::eNegX == 1, "");
    static_assert(VehicleContextDesc::ePosY == 2, "");
    static_assert(VehicleContextDesc::eNegY == 3, "");
    static_assert(VehicleContextDesc::ePosZ == 4, "");
    static_assert(VehicleContextDesc::eNegZ == 5, "");
    if ((vehicleContextDesc.verticalAxis != VehicleContextDesc::eUndefined) &&
        (vehicleContextDesc.longitudinalAxis != VehicleContextDesc::eUndefined) &&
        ((vehicleContextDesc.verticalAxis >> 1) == (vehicleContextDesc.longitudinalAxis >> 1)))
    {
        CARB_LOG_WARN("Usd Physics: vehicle context \"%s\": tokens \"verticalAxis\" and \"longitudinalAxis\" "
            "can not use the same axis.\n",
            usdPrim.GetName().GetText());
    }

    return true;
}

static bool parseVehicleController(VehicleControllerDesc& out, const UsdPrim& usdPrim)
{
    CARB_ASSERT(usdPrim.HasAPI<PhysxSchemaPhysxVehicleControllerAPI>());

    PhysxSchemaPhysxVehicleControllerAPI controller(usdPrim);

    if (SafeGetAttribute(&out.accelerator, controller.GetAcceleratorAttr()))
    {
        if (!checkParamInRange(out.accelerator, 0.0f, 1.0f + FLT_EPSILON, "accelerator", usdPrim))
            return false;
    }
    else
        out.accelerator = 0.0f;

    if (SafeGetAttribute(&out.brake0, controller.GetBrake0Attr()))
    {
        if (!checkParamInRange(out.brake0, 0.0f, 1.0f + FLT_EPSILON, "brake0", usdPrim))
            return false;
    }
    else
        out.brake0 = 0.0f;

    if (SafeGetAttribute(&out.brake1, controller.GetBrake1Attr()))
    {
        if (!checkParamInRange(out.brake1, 0.0f, 1.0f + FLT_EPSILON, "brake1", usdPrim))
            return false;
    }
    else
        out.brake1 = 0.0f;

    if (SafeGetAuthoredAttribute(&out.brake, controller.GetBrakeAttr()))
    {
        CARB_LOG_WARN("Usd Physics: vehicle controller \"%s\": attribute \"brake\" is deprecated. "
            "Please use brake0 or brake1 instead.\n",
            usdPrim.GetName().GetText());

        if (!checkParamInRange(out.brake, 0.0f, 1.0f + FLT_EPSILON, "brake", usdPrim))
            return false;
    }
    else
        out.brake = 0.0f;

    if (SafeGetAuthoredAttribute(&out.handbrake, controller.GetHandbrakeAttr()))
    {
        CARB_LOG_WARN("Usd Physics: vehicle controller \"%s\": attribute \"handbrake\" is deprecated. "
            "Please use brake0 or brake1 instead.\n",
            usdPrim.GetName().GetText());

        if (!checkParamInRange(out.handbrake, 0.0f, 1.0f + FLT_EPSILON, "handbrake", usdPrim))
            return false;
    }
    else
        out.handbrake = 0.0f;

    if (SafeGetAttribute(&out.steer, controller.GetSteerAttr()))
    {
        if (!checkParamInRange(out.steer, -1.0f, 1.0f + FLT_EPSILON, "steer", usdPrim))
            return false;
    }
    else
        out.steer = 0.0f;

    if (SafeGetAuthoredAttribute(&out.steerLeft, controller.GetSteerLeftAttr()))
    {
        CARB_LOG_WARN("Usd Physics: vehicle controller \"%s\": attribute \"steerLeft\" is deprecated. "
            "Please use steer instead.\n",
            usdPrim.GetName().GetText());

        if (!checkParamInRange(out.steerLeft, 0.0f, 1.0f + FLT_EPSILON, "steerLeft", usdPrim))
            return false;
    }
    else
        out.steerLeft = 0.0f;

    if (SafeGetAuthoredAttribute(&out.steerRight, controller.GetSteerRightAttr()))
    {
        CARB_LOG_WARN("Usd Physics: vehicle controller \"%s\": attribute \"steerRight\" is deprecated. "
            "Please use steer instead.\n",
            usdPrim.GetName().GetText());

        if (!checkParamInRange(out.steerRight, 0.0f, 1.0f + FLT_EPSILON, "steerRight", usdPrim))
            return false;
    }
    else
        out.steerRight = 0.0f;

    if (SafeGetAttribute(&out.targetGear, controller.GetTargetGearAttr()))
    {
        if ((out.targetGear != VehicleControllerDesc::automaticGearValue) &&
            !checkParamInRange(out.targetGear, -1, static_cast<int>(maxForwardGearCount + 1),
            "targetGear", usdPrim))
            return false;
    }
    else
        out.targetGear = VehicleControllerDesc::automaticGearValue;

    return true;
}

static bool parseVehicleTankController(VehicleTankControllerDesc& out, const UsdPrim& usdPrim)
{
    CARB_ASSERT(usdPrim.HasAPI<PhysxSchemaPhysxVehicleTankControllerAPI>());

    if (parseVehicleController(out, usdPrim))
    {
        PhysxSchemaPhysxVehicleTankControllerAPI controller(usdPrim);

        if (SafeGetAttribute(&out.thrust0, controller.GetThrust0Attr()))
        {
            if (!checkParamInRange(out.thrust0, -1.0f, 1.0f + FLT_EPSILON, "thrust0", usdPrim))
                return false;
        }
        else
            out.thrust0 = 0.0f;

        if (SafeGetAttribute(&out.thrust1, controller.GetThrust1Attr()))
        {
            if (!checkParamInRange(out.thrust1, -1.0f, 1.0f + FLT_EPSILON, "thrust1", usdPrim))
                return false;
        }
        else
            out.thrust1 = 0.0f;
    }
    else
        return false;

    return true;
}

static void testDriveAndWheelIndexList(const DriveDesc* drive,
    const std::vector<int>& wheelIndices, const uint32_t nbWheels,
    const pxr::UsdPrim& usdPrim, const char* apiName,
    bool& isValid)
{
    if (!drive)
    {
        CARB_LOG_ERROR(
            "Usd Physics: vehicle \"%s\" has %s applied which requires a drive "
            "being defined but there is none.\n",
            usdPrim.GetName().GetText(), apiName);

        isValid = false;
    }

    if (wheelIndices.size() > nbWheels)
    {
        CARB_LOG_ERROR(
            "Usd Physics: %s of vehicle \"%s\" has more wheel indices defined "
            "than the number of wheels.\n",
            apiName, usdPrim.GetName().GetText());

        isValid = false;
    }

    for (int wheelIndex : wheelIndices)
    {
        if (wheelIndex >= static_cast<int>(nbWheels))
        {
            CARB_LOG_ERROR(
                "Usd Physics: %s of vehicle \"%s\" has illegal wheel index %d "
                "specified (must not be larger than numberOfWheels-1).\n",
                apiName, usdPrim.GetName().GetText(), wheelIndex);

            isValid = false;
        }
    }
}

static Float3 getTotalScale(const UsdPrim& usdPrim, UsdGeomXformCache& xfCache,
    const char* warnMsgTypeName)
{
    GfMatrix4d mat = xfCache.GetLocalToWorldTransform(usdPrim);
    const GfTransform transform(mat);
    const GfVec3d sc = transform.GetScale();

    if (!scaleIsUniform(sc[0], sc[1], sc[2]) && transform.GetScaleOrientation().GetQuaternion() != GfQuaternion::GetIdentity())
    {
        CARB_LOG_WARN("Usd Physics: %s prim \"%s\": ScaleOrientation is not supported. "
            "You may ignore this if the scale is close to uniform.\n",
            warnMsgTypeName, usdPrim.GetPath().GetText());
    }

    return toFloat3(sc);
}

static constexpr int maxSubstepCount = 255;

bool parseVehicle(const pxr::UsdStageWeakPtr stage,
                  const pxr::UsdPrim& usdPrim,
                  VehicleDesc& vehicleDesc,
                  VehicleControllerDesc& vehicleControllerDesc,
                  VehicleTankControllerDesc& vehicleTankControllerDesc,
                  ObjectType& vehicleControllerType,
                  VehicleComponentTracker& vehicleComponentTracker,
                  pxr::UsdGeomXformCache& xfCache)
{
    vehicleControllerType = eUndefined;

    if (usdPrim.HasAPI<PhysxSchemaPhysxVehicleAPI>())
    {
        if (usdPrim.HasAPI<UsdPhysicsRigidBodyAPI>())
        {
            if (usdPrim.IsA<UsdGeomXformable>())
            {
                vehicleDesc.scale = getTotalScale(usdPrim, xfCache, "vehicle");
                if ((vehicleDesc.scale.x != 0.0f) && (vehicleDesc.scale.y != 0.0f) && (vehicleDesc.scale.z != 0.0f))
                {
                    const bool isScaleUniform = scaleIsUniform(vehicleDesc.scale.x, vehicleDesc.scale.y, vehicleDesc.scale.z);

                    ParseContext parseContext(stage, vehicleComponentTracker);

                    bool isValid = true;
                    const PhysxSchemaPhysxVehicleAPI usdVehicleAPI(usdPrim);

                    isValid = parseDrive(usdPrim, usdVehicleAPI, parseContext,
                        vehicleDesc.drive);

                    {
                        const bool res = SafeGetAttribute(&vehicleDesc.enabled, usdVehicleAPI.GetVehicleEnabledAttr());
                        CARB_ASSERT(res);
                        CARB_UNUSED(res);
                    }

                    {
                        const bool res = SafeGetAttribute(&vehicleDesc.limitSuspensionExpansionVelocity, usdVehicleAPI.GetLimitSuspensionExpansionVelocityAttr());
                        CARB_ASSERT(res);
                        CARB_UNUSED(res);
                    }

                    VtValue referenceFrameIsCenterOfMassVal;
                    bool referenceFrameIsCenterOfMass;
                    if (usdPrim.GetMetadataByDictKey(pxr::SdfFieldKeys->CustomData, PhysxSchemaTokens->referenceFrameIsCenterOfMass, &referenceFrameIsCenterOfMassVal))
                    {
                        referenceFrameIsCenterOfMass = referenceFrameIsCenterOfMassVal.Get<bool>();

                        if (referenceFrameIsCenterOfMass)
                        {
                            CARB_LOG_WARN("Usd Physics: vehicle \"%s\": the custom metadata attribute physxVehicle:referenceFrameIsCenterOfMass "
                                "is set to \"True\". Using the center of mass frame as reference has been deprecated and in the future there will "
                                "be no such option anymore.\n",
                                usdPrim.GetName().GetText());
                        }
                    }
                    else
                    {
                        CARB_LOG_WARN("Usd Physics: vehicle \"%s\": the custom metadata attribute physxVehicle:referenceFrameIsCenterOfMass "
                            "is missing on this prim. A value of \"True\" is assumed for backwards compatibility reasons. Note that using "
                            "the center of mass frame as reference has been deprecated and in the future there will be no such option anymore "
                            "(vehicles will be treated as if this custom metadata has been set to \"False\").\n",
                            usdPrim.GetName().GetText());

                        referenceFrameIsCenterOfMass = true;
                    }
                    vehicleDesc.referenceFrameIsCenterOfMass = referenceFrameIsCenterOfMass;

                    UsdPhysicsRigidBodyAPI usdBodyAPI(usdPrim);

                    bool rigidBodyEnabled;
                    {
                        const bool res = SafeGetAttribute(&rigidBodyEnabled, usdBodyAPI.GetRigidBodyEnabledAttr());
                        CARB_ASSERT(res);
                        CARB_UNUSED(res);
                    }
                    if (!rigidBodyEnabled)
                    {
                        CARB_LOG_ERROR("Usd Physics: vehicle \"%s\": the attribute \"rigidBodyEnabled\" of RigidBodyAPI is false "
                            "which is incompatible with vehicles.\n",
                            usdPrim.GetName().GetText());

                        isValid = false;
                    }

                    bool kinematicEnabled;
                    {
                        const bool res = SafeGetAttribute(&kinematicEnabled, usdBodyAPI.GetKinematicEnabledAttr());
                        CARB_ASSERT(res);
                        CARB_UNUSED(res);
                    }
                    if (kinematicEnabled && vehicleDesc.enabled)
                    {
                        CARB_LOG_ERROR("Usd Physics: vehicle \"%s\": the attribute \"kinematicEnabled\" of RigidBodyAPI is true. "
                            "This is only supported if the vehicle simulation is disabled (see attribute vehicleEnabled).\n",
                            usdPrim.GetName().GetText());

                        isValid = false;
                    }

                    TfToken queryTypeToken;
                    uint8_t queryType = VehicleDesc::eRAYCAST;

                    if (usdVehicleAPI.GetSuspensionLineQueryTypeAttr().Get(&queryTypeToken))
                    {
                        if (queryTypeToken == PhysxSchemaTokens.Get()->sweep)
                        {
                            queryType = VehicleDesc::eSWEEP;
                        }
                    }
                    vehicleDesc.queryType = queryType;

                    if (SafeGetAttribute(&vehicleDesc.subStepThresholdLongitudinalSpeed,
                        usdVehicleAPI.GetSubStepThresholdLongitudinalSpeedAttr()))
                    {
                        isValid &= checkParamInRange(vehicleDesc.subStepThresholdLongitudinalSpeed, 0.0f, FLT_MAX,
                            "subStepThresholdLongitudinalSpeed", usdPrim);
                    }
                    else
                        vehicleDesc.subStepThresholdLongitudinalSpeed = 5.0f * parseContext.lengthScale;

                    if (SafeGetAttribute(
                        &vehicleDesc.lowForwardSpeedSubStepCount, usdVehicleAPI.GetLowForwardSpeedSubStepCountAttr()))
                    {
                        isValid &= checkParamIsPositive(vehicleDesc.lowForwardSpeedSubStepCount, "lowForwardSpeedSubStepCount", usdPrim);

                        if (vehicleDesc.lowForwardSpeedSubStepCount > maxSubstepCount)
                        {
                            vehicleDesc.lowForwardSpeedSubStepCount = maxSubstepCount;

                            CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"lowForwardSpeedSubStepCount\" needs to be smaller or equal %d. "
                                "Value will be clamped.\n",
                                usdPrim.GetName().GetText(), maxSubstepCount);
                        }
                    }
                    else
                        vehicleDesc.lowForwardSpeedSubStepCount = 3;

                    if (SafeGetAttribute(
                        &vehicleDesc.highForwardSpeedSubStepCount, usdVehicleAPI.GetHighForwardSpeedSubStepCountAttr()))
                    {
                        isValid &= checkParamIsPositive(vehicleDesc.highForwardSpeedSubStepCount, "highForwardSpeedSubStepCount", usdPrim);

                        if (vehicleDesc.highForwardSpeedSubStepCount > maxSubstepCount)
                        {
                            vehicleDesc.highForwardSpeedSubStepCount = maxSubstepCount;

                            CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"highForwardSpeedSubStepCount\" needs to be smaller or equal %d. "
                                "Value will be clamped.\n",
                                usdPrim.GetName().GetText(), maxSubstepCount);
                        }
                    }
                    else
                        vehicleDesc.highForwardSpeedSubStepCount = 1;

                    if (getExpectedAttribute(vehicleDesc.minPassiveLongitudinalSlipDenominator,
                        usdVehicleAPI.GetMinPassiveLongitudinalSlipDenominatorAttr(), usdPrim, "minPassiveLongitudinalSlipDenominator"))
                    {
                        if (vehicleDesc.minPassiveLongitudinalSlipDenominator != 0.0f)
                        {
                            isValid &= checkParamIsPositive(vehicleDesc.minPassiveLongitudinalSlipDenominator, "minPassiveLongitudinalSlipDenominator", usdPrim);
                        }
                        else
                        {
                            CARB_LOG_WARN("Usd Physics: prim \"%s\": attribute \"minLongitudinalSlipDenominator\" "
                                "is deprecated. Please use minPassiveLongitudinalSlipDenominator instead.\n",
                                usdPrim.GetName().GetText());

                            if (SafeGetAttribute(
                                &vehicleDesc.minLongitudinalSlipDenominator, usdVehicleAPI.GetMinLongitudinalSlipDenominatorAttr()))
                            {
                                isValid &= checkParamInRange(vehicleDesc.minLongitudinalSlipDenominator, 0.0f, FLT_MAX,
                                    "minLongitudinalSlipDenominator", usdPrim);
                            }
                            else
                                vehicleDesc.minLongitudinalSlipDenominator = 4.0f * parseContext.lengthScale;
                        }
                    }
                    else
                        isValid = false;

                    if (getExpectedAttribute(vehicleDesc.minActiveLongitudinalSlipDenominator,
                        usdVehicleAPI.GetMinActiveLongitudinalSlipDenominatorAttr(), usdPrim, "minActiveLongitudinalSlipDenominator"))
                    {
                        if (vehicleDesc.minActiveLongitudinalSlipDenominator != 0.0f)
                        {
                            isValid &= checkParamIsPositive(vehicleDesc.minActiveLongitudinalSlipDenominator, "minActiveLongitudinalSlipDenominator", usdPrim);
                        }
                        else
                        {
                            vehicleDesc.minActiveLongitudinalSlipDenominator = 0.1f * parseContext.lengthScale;
                        }
                    }
                    else
                        isValid = false;

                    if (getExpectedAttribute(vehicleDesc.minLateralSlipDenominator,
                        usdVehicleAPI.GetMinLateralSlipDenominatorAttr(), usdPrim, "minLateralSlipDenominator"))
                    {
                        if (vehicleDesc.minLateralSlipDenominator != 0.0f)
                        {
                            isValid &= checkParamIsPositive(vehicleDesc.minLateralSlipDenominator, "minLateralSlipDenominator", usdPrim);
                        }
                        else
                        {
                            vehicleDesc.minLateralSlipDenominator = 1.0f * parseContext.lengthScale;
                        }
                    }
                    else
                        isValid = false;

                    if (getExpectedAttribute(vehicleDesc.longitudinalStickyTireThresholdSpeed,
                        usdVehicleAPI.GetLongitudinalStickyTireThresholdSpeedAttr(), usdPrim, "longitudinalStickyTireThresholdSpeed"))
                    {
                        if (vehicleDesc.longitudinalStickyTireThresholdSpeed != -1.0f)
                        {
                            isValid &= checkParamIsNonNegative(vehicleDesc.longitudinalStickyTireThresholdSpeed, "longitudinalStickyTireThresholdSpeed", usdPrim);
                        }
                        else
                        {
                            vehicleDesc.longitudinalStickyTireThresholdSpeed = 0.2f * parseContext.lengthScale;
                        }
                    }
                    else
                        isValid = false;

                    if (getExpectedAttribute(vehicleDesc.longitudinalStickyTireThresholdTime, usdVehicleAPI.GetLongitudinalStickyTireThresholdTimeAttr(),
                        usdPrim, "longitudinalStickyTireThresholdTime"))
                    {
                        isValid &= checkParamInRange(vehicleDesc.longitudinalStickyTireThresholdTime, 0.0f, FLT_MAX,
                            "longitudinalStickyTireThresholdTime", usdPrim);
                    }
                    else
                        isValid = false;

                    if (getExpectedAttribute(vehicleDesc.longitudinalStickyTireDamping, usdVehicleAPI.GetLongitudinalStickyTireDampingAttr(),
                        usdPrim, "longitudinalStickyTireDamping"))
                    {
                        isValid &= checkParamInRange(vehicleDesc.longitudinalStickyTireDamping, 0.0f, FLT_MAX,
                            "longitudinalStickyTireDamping", usdPrim);
                    }
                    else
                        isValid = false;

                    if (getExpectedAttribute(vehicleDesc.lateralStickyTireThresholdSpeed,
                        usdVehicleAPI.GetLateralStickyTireThresholdSpeedAttr(), usdPrim, "lateralStickyTireThresholdSpeed"))
                    {
                        if (vehicleDesc.lateralStickyTireThresholdSpeed != -1.0f)
                        {
                            isValid &= checkParamIsNonNegative(vehicleDesc.lateralStickyTireThresholdSpeed, "lateralStickyTireThresholdSpeed", usdPrim);
                        }
                        else
                        {
                            vehicleDesc.lateralStickyTireThresholdSpeed = 0.2f * parseContext.lengthScale;
                        }
                    }
                    else
                        isValid = false;

                    if (getExpectedAttribute(vehicleDesc.lateralStickyTireThresholdTime, usdVehicleAPI.GetLongitudinalStickyTireThresholdTimeAttr(),
                        usdPrim, "lateralStickyTireThresholdTime"))
                    {
                        isValid &= checkParamInRange(vehicleDesc.lateralStickyTireThresholdTime, 0.0f, FLT_MAX,
                            "lateralStickyTireThresholdTime", usdPrim);
                    }
                    else
                        isValid = false;

                    if (getExpectedAttribute(vehicleDesc.lateralStickyTireDamping, usdVehicleAPI.GetLateralStickyTireDampingAttr(),
                        usdPrim, "lateralStickyTireDamping"))
                    {
                        isValid &= checkParamInRange(vehicleDesc.lateralStickyTireDamping, 0.0f, FLT_MAX,
                            "lateralStickyTireDamping", usdPrim);
                    }
                    else
                        isValid = false;

                    // note: to be consistent with collision shapes, a wheel does not have to be a direct child of the vehicle
                    // prim
                    uint32_t nbWheels = 0;
                    uint32_t nbUserDefinedSprungMass = 0;
                    uint32_t nbUserDefinedMaxDroop = 0;
                    uint32_t nbUserDefinedRestLoad = 0;
                    uint32_t nbDeprecatedLatStiffYUsed = 0;
                    static_assert(VehicleDesc::maxNumberOfWheels <= 32, "");  // using bit logic to track the indices that were assigned to wheel attachments 
                    uint32_t encounteredAttachmentIndices = 0;
                    UsdPrimSubtreeRange subPrims = usdPrim.GetDescendants();
                    for (UsdPrim subPrim : subPrims)
                    {
                        if (subPrim.HasAPI<PhysxSchemaPhysxVehicleWheelAttachmentAPI>())
                        {
                            if (nbWheels < VehicleDesc::maxNumberOfWheels)
                            {
                                WheelAttachmentDesc wheelAttachment;
                                if (parseWheelAttachment(wheelAttachment, subPrim, parseContext, vehicleComponentTracker))
                                {
                                    CARB_ASSERT(wheelAttachment.index < static_cast<int>(VehicleDesc::maxNumberOfWheels));
                                    if (wheelAttachment.index >= 0)
                                    {
                                        // track recorded indices as bits
                                        const uint32_t bitIndex = (1 << wheelAttachment.index);
                                        if (!(encounteredAttachmentIndices & bitIndex))
                                            encounteredAttachmentIndices |= bitIndex;
                                        else
                                        {
                                            CARB_LOG_ERROR(
                                                "Usd Physics: vehicle \"%s\": multiple wheel attachments use the same index %d.\n",
                                                usdPrim.GetName().GetText(), wheelAttachment.index);
                                            isValid = false;
                                        }
                                    }
                                    else
                                    {
                                        // if wheel attachment index is -1, then choose index based on order of parsing
                                        wheelAttachment.index = nbWheels;
                                    }

                                    if ((!isScaleUniform) && (!toPhysXQuat(wheelAttachment.suspensionFrameOrientation).isIdentity()))
                                    {
                                        // some attributes are defined in the suspension frame. If that one is rotated with respect to
                                        // the vehicle frame, we end up with additional complexity and shear scenarios etc. which
                                        // we want to ignore. Thus, only uniform scale is supported in such a case.

                                        CARB_LOG_WARN("Usd Physics: vehicle \"%s\", wheel attachment \"%s\": ScaleOrientation in suspension frame is not supported. "
                                            "You may ignore this if the vehicle frame scale is close to uniform.\n",
                                            usdPrim.GetName().GetText(), subPrim.GetName().GetText());
                                        isValid = false;
                                    }

                                    vehicleDesc.wheelAttachments.push_back(wheelAttachment);

                                    if (subPrim.HasAPI<PhysxSchemaPhysxVehicleWheelControllerAPI>())
                                    {
                                        if (!vehicleDesc.drive)
                                        {
                                            WheelControllerDesc wheelController;
                                            if (parseWheelController(wheelController, subPrim))
                                            {
                                                vehicleDesc.wheelControllers.push_back(wheelController);
                                            }
                                            else
                                                isValid = false;
                                        }
                                        else
                                        {
                                            CARB_LOG_ERROR(
                                                "Usd Physics: vehicle \"%s\" has a drive specified but wheel attachment \"%s\" has "
                                                "PhysxVehicleWheelControllerAPI applied. This is an illegal configuration.\n",
                                                usdPrim.GetName().GetText(), subPrim.GetName().GetText());
                                            isValid = false;
                                        }
                                    }

                                    if (wheelAttachment.suspension->sprungMass > 0.0f)
                                        nbUserDefinedSprungMass++;

                                    if (wheelAttachment.suspension->maxDroop >= 0.0f)
                                        nbUserDefinedMaxDroop++;

                                    if (wheelAttachment.tire->restLoad > 0.0f)
                                        nbUserDefinedRestLoad++;

                                    if (wheelAttachment.tire->lateralStiffnessGraph.y == 0)
                                        nbDeprecatedLatStiffYUsed++;
                                }
                                else
                                    isValid = false;

                                nbWheels++;
                            }
                            else
                            {
                                CARB_LOG_ERROR(
                                    "Usd Physics: vehicle \"%s\" has more than the maximum allowed number of %d wheels.\n",
                                    usdPrim.GetName().GetText(), VehicleDesc::maxNumberOfWheels);
                                isValid = false;
                            }
                        }
                    }

                    vehicleDesc.hasUserDefinedSprungMassValues = nbUserDefinedSprungMass > 0;
                    vehicleDesc.hasUserDefinedMaxDroopValues = nbUserDefinedMaxDroop > 0;
                    vehicleDesc.hasUserDefinedRestLoadValues = nbUserDefinedRestLoad > 0;
                    vehicleDesc.isUsingDeprecatedLatStiffY = nbDeprecatedLatStiffYUsed > 0;

                    if (nbWheels == 0)
                    {
                        CARB_LOG_ERROR(
                            "Usd Physics: vehicle \"%s\" needs to have valid wheel attachments among descendants (see PhysxVehicleWheelAttachmentAPI).\n",
                            usdPrim.GetName().GetText());
                        isValid = false;
                    }

                    if (nbUserDefinedSprungMass && (nbUserDefinedSprungMass != nbWheels))
                    {
                        CARB_LOG_ERROR(
                            "Usd Physics: vehicle \"%s\": the sprung mass values of the suspensions need to be either all zero or have user defined "
                            "positive values.\n",
                            usdPrim.GetName().GetText());
                        isValid = false;
                    }

                    if (nbUserDefinedMaxDroop && (nbUserDefinedMaxDroop != nbWheels))
                    {
                        CARB_LOG_ERROR(
                            "Usd Physics: vehicle \"%s\": the max droop values of the suspensions need to be either all negative or have user defined "
                            "non-negative values.\n",
                            usdPrim.GetName().GetText());
                        isValid = false;
                    }

                    if (nbUserDefinedRestLoad && (nbUserDefinedRestLoad != nbWheels))
                    {
                        CARB_LOG_ERROR(
                            "Usd Physics: vehicle \"%s\": the rest load values of the tires need to be either all zero or have user defined "
                            "positive values.\n",
                            usdPrim.GetName().GetText());
                        isValid = false;
                    }

                    if (nbDeprecatedLatStiffYUsed && (nbDeprecatedLatStiffYUsed != nbWheels))
                    {
                        CARB_LOG_ERROR(
                            "Usd Physics: vehicle \"%s\": either all tires need to use lateralStiffnessGraph or all tires need to use the "
                            "deprecated latStiffX/latStiffY attributes.\n",
                            usdPrim.GetName().GetText());
                        isValid = false;
                    }

                    if ((encounteredAttachmentIndices > 0) && (encounteredAttachmentIndices != ((1 << nbWheels) - 1)))
                    {
                        CARB_LOG_ERROR(
                            "Usd Physics: vehicle \"%s\": either all wheel attachment indices need to be -1 or they need to cover all entries in the group "
                            "{0, ..., (numberOfWheels-1)}.\n",
                            usdPrim.GetName().GetText());
                        isValid = false;
                    }

                    if (isValid)
                    {
                        if (parseDifferential(vehicleDesc.differential, usdPrim, parseContext))
                        {
                            if (vehicleDesc.differential)  // no differential might be specified
                            {
                                testDriveAndWheelIndexList(vehicleDesc.drive, vehicleDesc.differential->wheels, nbWheels,
                                    usdPrim, "PhysxVehicleMultiWheelDifferentialAPI", isValid);

                                if (vehicleDesc.differential->type == DifferentialDesc::eTank)
                                {
                                    const TankDifferentialDesc* tankDifferential = static_cast<const TankDifferentialDesc*>(vehicleDesc.differential);

                                    const uint32_t trackCount = static_cast<uint32_t>(tankDifferential->trackToWheelIndices.size());

                                    CARB_ASSERT(tankDifferential->trackToWheelIndices.size() == tankDifferential->numberOfWheelsPerTrack.size());
                                    // validation should have aborted if that is not the case

                                    for (uint32_t k = 0; k < trackCount; k++)
                                    {
                                        const int startIndex = tankDifferential->trackToWheelIndices[k];
                                        const int endIndexPlusOne = startIndex + tankDifferential->numberOfWheelsPerTrack[k];

                                        CARB_ASSERT(startIndex >= 0);
                                        CARB_ASSERT((startIndex == endIndexPlusOne) || (endIndexPlusOne <= tankDifferential->wheelIndicesInTrackOrder.size()));
                                        // validation should have aborted if that is not the case

                                        for (int l = startIndex; l < endIndexPlusOne; l++)
                                        {
                                            const int wheelIndex = tankDifferential->wheelIndicesInTrackOrder[l];

                                            CARB_ASSERT(wheelIndex >= 0);
                                            // validation should have aborted if that is not the case

                                            if (wheelIndex >= static_cast<int>(nbWheels))
                                            {
                                                CARB_LOG_ERROR(
                                                    "Usd Physics: PhysxVehicleTankDifferentialAPI of vehicle \"%s\" has illegal wheel index %d "
                                                    "specified in the \"wheelIndicesInTrackOrder\" attribute (index must not be larger than "
                                                    "numberOfWheels-1).\n",
                                                    usdPrim.GetName().GetText(), wheelIndex);

                                                isValid = false;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        else
                            isValid = false;
                    }

                    if (isValid)
                    {
                        if (parseBrakes(vehicleDesc.brakes, usdPrim, parseContext))
                        {
                            for (const BrakesDesc* brakesDesc : vehicleDesc.brakes) // no brakes might be specified
                            {
                                testDriveAndWheelIndexList(vehicleDesc.drive, brakesDesc->wheels, nbWheels,
                                    usdPrim, "PhysxVehicleBrakesAPI", isValid);
                            }
                        }
                        else
                            isValid = false;
                    }

                    if (isValid)
                    {
                        if (parseSteering(vehicleDesc.steering, usdPrim, parseContext))
                        {
                            if (vehicleDesc.steering)  // no steering might be specified
                            {
                                if (vehicleDesc.steering->type == SteeringDesc::eBasic)
                                {
                                    const SteeringBasicDesc* steeringBasic = static_cast<const SteeringBasicDesc*>(vehicleDesc.steering);

                                    testDriveAndWheelIndexList(vehicleDesc.drive, steeringBasic->wheels, nbWheels,
                                        usdPrim, "PhysxVehicleSteeringAPI", isValid);
                                }
                                else
                                {
                                    CARB_ASSERT(vehicleDesc.steering->type == SteeringDesc::eAckermann);
                                    const SteeringAckermannDesc* steeringAckermann = static_cast<const SteeringAckermannDesc*>(vehicleDesc.steering);

                                    std::vector<int> indexList{ steeringAckermann->wheel0, steeringAckermann->wheel1 };
                                    testDriveAndWheelIndexList(vehicleDesc.drive, indexList, nbWheels,
                                        usdPrim, "PhysxVehicleAckermannSteeringAPI", isValid);
                                }
                            }
                        }
                        else
                            isValid = false;
                    }

                    if (isValid && usdPrim.HasAPI<PhysxSchemaPhysxVehicleControllerAPI>())
                    {
                        if (!vehicleDesc.wheelControllers.size())
                        {
                            bool controllerParseSuccess;
                            if (!usdPrim.HasAPI<PhysxSchemaPhysxVehicleTankControllerAPI>())
                            {
                                vehicleControllerType = eVehicleControllerStandard;
                                controllerParseSuccess = parseVehicleController(vehicleControllerDesc, usdPrim);
                            }
                            else
                            {
                                vehicleControllerType = eVehicleControllerTank;
                                controllerParseSuccess = parseVehicleTankController(vehicleTankControllerDesc, usdPrim);

                                if (vehicleDesc.drive && (vehicleDesc.drive->type != eVehicleDriveStandard))
                                {
                                    CARB_LOG_ERROR(
                                        "Usd Physics: vehicle \"%s\" has PhysxVehicleTankControllerAPI applied which requires "
                                        "a standard drive (see PhysxVehicleDriveStandardAPI).\n",
                                        usdPrim.GetName().GetText());

                                    isValid = false;
                                }

                                if (!vehicleDesc.differential || (vehicleDesc.differential->type != DifferentialDesc::eTank))
                                {
                                    CARB_LOG_ERROR(
                                        "Usd Physics: vehicle \"%s\" has PhysxVehicleTankControllerAPI applied which requires "
                                        "a tank differential (see PhysxVehicleTankDifferentialAPI).\n",
                                        usdPrim.GetName().GetText());

                                    isValid = false;
                                }
                            }

                            if (controllerParseSuccess)
                            {
                                if (!vehicleDesc.drive)
                                {
                                    CARB_LOG_ERROR(
                                        "Usd Physics: vehicle \"%s\" has PhysxVehicleControllerAPI applied which requires a drive "
                                        "being defined but there is none.\n",
                                        usdPrim.GetName().GetText());

                                    isValid = false;
                                }
                            }
                            else
                                isValid = false;
                        }
                        else
                        {
                            CARB_LOG_ERROR(
                                "Usd Physics: vehicle \"%s\" has PhysxVehicleControllerAPI applied and descendants with "
                                "PhysxVehicleWheelControllerAPI applied. Only one or the other is allowed.\n",
                                usdPrim.GetName().GetText());

                            isValid = false;
                        }
                    }

                    return isValid;
                }
                else
                {
                    CARB_LOG_ERROR(
                        "Usd Physics: vehicle \"%s\" has a scale component that is zero.\n", usdPrim.GetName().GetText());
                    return false;
                }
            }
            else
            {
                CARB_LOG_ERROR(
                    "Usd Physics: vehicle \"%s\" needs to be a UsdGeomXformable.\n", usdPrim.GetName().GetText());
                return false;
            }
        }
        else
        {
            CARB_LOG_ERROR(
                "Usd Physics: vehicle \"%s\" needs to have RigidBodyAPI applied.\n", usdPrim.GetName().GetText());
            return false;
        }
    }
    else
    {
        return false;
    }
}

} // namespace usdparser
} // namespace physx
} // namespace omni
