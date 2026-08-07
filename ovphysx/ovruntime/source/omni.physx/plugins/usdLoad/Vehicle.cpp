// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on
#include "Vehicle.h"
#include "LoadUsd.h"
#include "PhysXTools.h"

#include <common/foundation/TypeCast.h>
#include <common/utilities/Utilities.h>

#include <carb/Types.h>
#include <carb/logging/Log.h>

using namespace PXR_NS;
using namespace carb;

namespace omni
{
namespace physx
{
namespace usdparser
{

// Source-read mirror of SafeGetAttribute (resolved value incl. schema fallback);
// getValue's bool matches UsdAttribute::HasValue().
template <typename T>
static bool srcGet(const AttachedStage& as, omni::physics::parse::ObjectKey key, const TfToken& attr, T& out)
{
    return omni::physx::internal::getValue(as, key, attr, UsdTimeCode::Default(), out);
}

// Source-read mirror of SafeGetAuthoredAttribute (authored-only).
template <typename T>
static bool srcGetAuthored(const AttachedStage& as, omni::physics::parse::ObjectKey key, const TfToken& attr, T& out)
{
    const omni::physics::parse::IPhysicsSource* src = as.getSource();
    if (!src || !src->hasAuthoredAttribute(key, src->internToken(attr.GetString())))
        return false;
    omni::physx::internal::getValue(as, key, attr, UsdTimeCode::Default(), out);
    return true;
}

template <typename T>
static bool checkParamInRange(const T value, const T min, const T max, const char* paramName, const char* primName)
{
    if ((value >= min) && (value < max))
        return true;
    else
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\" needs to be in [%d, %d).\n", primName,
                       paramName, min, max);
        return false;
    }
}

template <>
bool checkParamInRange<float>(
    const float value, const float min, const float max, const char* paramName, const char* primName)
{
    if ((value >= min) && (value < max))
        return true;
    else
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\" needs to be in [%f, %f).\n", primName,
                       paramName, min, max);
        return false;
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
    deleteComponents(mWheelAttachmentsOwned);
    deleteComponents(mVehiclesOwned);
}

// Vehicle / wheel controllers carry runtime-mutable input state (not scanned);
// read them through the source by ObjectKey.
bool parseWheelController(AttachedStage& as, omni::physics::parse::ObjectKey key, WheelControllerDesc& out)
{
    const SdfPath path = as.pathFor(key);
    out.path = path;
    const std::string nameStr = path.GetName();
    const char* primName = nameStr.c_str();

    if (!srcGet(as, key, PhysxSchemaTokens->physxVehicleWheelControllerDriveTorque, out.driveTorque))
        out.driveTorque = 0.0f;

    if (srcGet(as, key, PhysxSchemaTokens->physxVehicleWheelControllerBrakeTorque, out.brakeTorque))
    {
        if (!checkParamInRange(out.brakeTorque, 0.0f, FLT_MAX, "brakeTorque", primName))
            return false;
    }
    else
        out.brakeTorque = 0.0f;

    if (!srcGet(as, key, PhysxSchemaTokens->physxVehicleWheelControllerSteerAngle, out.steerAngle))
        out.steerAngle = 0.0f;

    return true;
}

static constexpr uint32_t maxForwardGearCount = GearsDesc::maxNumberOfGears - 2;  // without reverse and neutral

static bool parseVehicleController(AttachedStage& as, omni::physics::parse::ObjectKey key, VehicleControllerDesc& out)
{
    const std::string nameStr = as.pathFor(key).GetName();
    const char* primName = nameStr.c_str();

    if (srcGet(as, key, PhysxSchemaTokens->physxVehicleControllerAccelerator, out.accelerator))
    {
        if (!checkParamInRange(out.accelerator, 0.0f, 1.0f + FLT_EPSILON, "accelerator", primName))
            return false;
    }
    else
        out.accelerator = 0.0f;

    if (srcGet(as, key, PhysxSchemaTokens->physxVehicleControllerBrake0, out.brake0))
    {
        if (!checkParamInRange(out.brake0, 0.0f, 1.0f + FLT_EPSILON, "brake0", primName))
            return false;
    }
    else
        out.brake0 = 0.0f;

    if (srcGet(as, key, PhysxSchemaTokens->physxVehicleControllerBrake1, out.brake1))
    {
        if (!checkParamInRange(out.brake1, 0.0f, 1.0f + FLT_EPSILON, "brake1", primName))
            return false;
    }
    else
        out.brake1 = 0.0f;

    if (srcGetAuthored(as, key, PhysxSchemaTokens->physxVehicleControllerBrake, out.brake))
    {
        CARB_LOG_WARN("Usd Physics: vehicle controller \"%s\": attribute \"brake\" is deprecated. "
            "Please use brake0 or brake1 instead.\n",
            primName);

        if (!checkParamInRange(out.brake, 0.0f, 1.0f + FLT_EPSILON, "brake", primName))
            return false;
    }
    else
        out.brake = 0.0f;

    if (srcGetAuthored(as, key, PhysxSchemaTokens->physxVehicleControllerHandbrake, out.handbrake))
    {
        CARB_LOG_WARN("Usd Physics: vehicle controller \"%s\": attribute \"handbrake\" is deprecated. "
            "Please use brake0 or brake1 instead.\n",
            primName);

        if (!checkParamInRange(out.handbrake, 0.0f, 1.0f + FLT_EPSILON, "handbrake", primName))
            return false;
    }
    else
        out.handbrake = 0.0f;

    if (srcGet(as, key, PhysxSchemaTokens->physxVehicleControllerSteer, out.steer))
    {
        if (!checkParamInRange(out.steer, -1.0f, 1.0f + FLT_EPSILON, "steer", primName))
            return false;
    }
    else
        out.steer = 0.0f;

    if (srcGetAuthored(as, key, PhysxSchemaTokens->physxVehicleControllerSteerLeft, out.steerLeft))
    {
        CARB_LOG_WARN("Usd Physics: vehicle controller \"%s\": attribute \"steerLeft\" is deprecated. "
            "Please use steer instead.\n",
            primName);

        if (!checkParamInRange(out.steerLeft, 0.0f, 1.0f + FLT_EPSILON, "steerLeft", primName))
            return false;
    }
    else
        out.steerLeft = 0.0f;

    if (srcGetAuthored(as, key, PhysxSchemaTokens->physxVehicleControllerSteerRight, out.steerRight))
    {
        CARB_LOG_WARN("Usd Physics: vehicle controller \"%s\": attribute \"steerRight\" is deprecated. "
            "Please use steer instead.\n",
            primName);

        if (!checkParamInRange(out.steerRight, 0.0f, 1.0f + FLT_EPSILON, "steerRight", primName))
            return false;
    }
    else
        out.steerRight = 0.0f;

    if (srcGet(as, key, PhysxSchemaTokens->physxVehicleControllerTargetGear, out.targetGear))
    {
        if ((out.targetGear != VehicleControllerDesc::automaticGearValue) &&
            !checkParamInRange(out.targetGear, -1, static_cast<int>(maxForwardGearCount + 1),
            "targetGear", primName))
            return false;
    }
    else
        out.targetGear = VehicleControllerDesc::automaticGearValue;

    return true;
}

static bool parseVehicleTankController(AttachedStage& as, omni::physics::parse::ObjectKey key, VehicleTankControllerDesc& out)
{
    if (parseVehicleController(as, key, out))
    {
        const std::string nameStr = as.pathFor(key).GetName();
        const char* primName = nameStr.c_str();

        if (srcGet(as, key, PhysxSchemaTokens->physxVehicleTankControllerThrust0, out.thrust0))
        {
            if (!checkParamInRange(out.thrust0, -1.0f, 1.0f + FLT_EPSILON, "thrust0", primName))
                return false;
        }
        else
            out.thrust0 = 0.0f;

        if (srcGet(as, key, PhysxSchemaTokens->physxVehicleTankControllerThrust1, out.thrust1))
        {
            if (!checkParamInRange(out.thrust1, -1.0f, 1.0f + FLT_EPSILON, "thrust1", primName))
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
    const char* primName, const char* apiName,
    bool& isValid)
{
    if (!drive)
    {
        CARB_LOG_ERROR(
            "Usd Physics: vehicle \"%s\" has %s applied which requires a drive "
            "being defined but there is none.\n",
            primName, apiName);

        isValid = false;
    }

    if (wheelIndices.size() > nbWheels)
    {
        CARB_LOG_ERROR(
            "Usd Physics: %s of vehicle \"%s\" has more wheel indices defined "
            "than the number of wheels.\n",
            apiName, primName);

        isValid = false;
    }

    for (int wheelIndex : wheelIndices)
    {
        if (wheelIndex >= static_cast<int>(nbWheels))
        {
            CARB_LOG_ERROR(
                "Usd Physics: %s of vehicle \"%s\" has illegal wheel index %d "
                "specified (must not be larger than numberOfWheels-1).\n",
                apiName, primName, wheelIndex);

            isValid = false;
        }
    }
}

// Fully source-driven: the vehicle's components and per-attachment data are
// consumed from the scanned VehicleComponentTracker (built in
// processScannedDescs); the wheel-attachment set comes from the walker-resolved
// vehicle->attachment association (mVehicleWheelAttachments) instead of a USD
// GetDescendants walk. Only runtime-mutable state (rigid-body gates, controller
// inputs) is read live through the source. No UsdPrim.
bool parseVehicle(AttachedStage& attachedStage,
                  omni::physics::parse::ObjectKey vehicleKey,
                  VehicleDesc& vehicleDesc,
                  VehicleControllerDesc& vehicleControllerDesc,
                  VehicleTankControllerDesc& vehicleTankControllerDesc,
                  ObjectType& vehicleControllerType,
                  VehicleComponentTracker& vehicleComponentTracker)
{
    vehicleControllerType = eUndefined;

    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src || !omni::physx::internal::hasAppliedSchema<PhysxSchemaPhysxVehicleAPI>(*src, vehicleKey))
        return false;

    const SdfPath vehiclePath = attachedStage.pathFor(vehicleKey);
    const std::string vehNameStr = vehiclePath.GetName();
    const char* vehName = vehNameStr.c_str();

    if (!omni::physx::internal::hasAppliedSchema<UsdPhysicsRigidBodyAPI>(*src, vehicleKey))
    {
        CARB_LOG_ERROR("Usd Physics: vehicle \"%s\" needs to have RigidBodyAPI applied.\n", vehName);
        return false;
    }
    if (!src->isA(vehicleKey, omni::physx::internal::schemaTypeToken<UsdGeomXformable>(*src)))
    {
        CARB_LOG_ERROR("Usd Physics: vehicle \"%s\" needs to be a UsdGeomXformable.\n", vehName);
        return false;
    }

    // Walker-side pre-pop is the only path that fills VehicleDesc. A side-table
    // miss means processScannedDescs:vehicle dropped this prim (already logged).
    auto vehicleByPathIt = vehicleComponentTracker.mVehicleByPath.find(vehiclePath);
    if (vehicleByPathIt == vehicleComponentTracker.mVehicleByPath.end())
        return false;

    {
        const VehicleDesc& s = *vehicleByPathIt->second;
        vehicleDesc.scale                                   = s.scale;
        vehicleDesc.drive                                   = s.drive;
        vehicleDesc.differential                            = s.differential;
        vehicleDesc.steering                                = s.steering;
        vehicleDesc.brakes                                  = s.brakes;
        vehicleDesc.enabled                                 = s.enabled;
        vehicleDesc.limitSuspensionExpansionVelocity        = s.limitSuspensionExpansionVelocity;
        vehicleDesc.referenceFrameIsCenterOfMass            = s.referenceFrameIsCenterOfMass;
        vehicleDesc.queryType                               = s.queryType;
        vehicleDesc.subStepThresholdLongitudinalSpeed       = s.subStepThresholdLongitudinalSpeed;
        vehicleDesc.lowForwardSpeedSubStepCount             = s.lowForwardSpeedSubStepCount;
        vehicleDesc.highForwardSpeedSubStepCount            = s.highForwardSpeedSubStepCount;
        vehicleDesc.minLongitudinalSlipDenominator          = s.minLongitudinalSlipDenominator;
        vehicleDesc.minPassiveLongitudinalSlipDenominator   = s.minPassiveLongitudinalSlipDenominator;
        vehicleDesc.minActiveLongitudinalSlipDenominator    = s.minActiveLongitudinalSlipDenominator;
        vehicleDesc.minLateralSlipDenominator               = s.minLateralSlipDenominator;
        vehicleDesc.longitudinalStickyTireThresholdSpeed    = s.longitudinalStickyTireThresholdSpeed;
        vehicleDesc.longitudinalStickyTireThresholdTime     = s.longitudinalStickyTireThresholdTime;
        vehicleDesc.longitudinalStickyTireDamping           = s.longitudinalStickyTireDamping;
        vehicleDesc.lateralStickyTireThresholdSpeed         = s.lateralStickyTireThresholdSpeed;
        vehicleDesc.lateralStickyTireThresholdTime          = s.lateralStickyTireThresholdTime;
        vehicleDesc.lateralStickyTireDamping                = s.lateralStickyTireDamping;
    }

    const bool isScaleUniform = scaleIsUniform(vehicleDesc.scale.x, vehicleDesc.scale.y, vehicleDesc.scale.z);
    bool isValid = true;

    bool rigidBodyEnabled = true;
    srcGet(attachedStage, vehicleKey, UsdPhysicsTokens->physicsRigidBodyEnabled, rigidBodyEnabled);
    if (!rigidBodyEnabled)
    {
        CARB_LOG_ERROR("Usd Physics: vehicle \"%s\": the attribute \"rigidBodyEnabled\" of RigidBodyAPI is false "
            "which is incompatible with vehicles.\n", vehName);
        isValid = false;
    }

    bool kinematicEnabled = false;
    srcGet(attachedStage, vehicleKey, UsdPhysicsTokens->physicsKinematicEnabled, kinematicEnabled);
    if (kinematicEnabled && vehicleDesc.enabled)
    {
        CARB_LOG_ERROR("Usd Physics: vehicle \"%s\": the attribute \"kinematicEnabled\" of RigidBodyAPI is true. "
            "This is only supported if the vehicle simulation is disabled (see attribute vehicleEnabled).\n", vehName);
        isValid = false;
    }

    uint32_t nbWheels = 0;
    uint32_t nbUserDefinedSprungMass = 0;
    uint32_t nbUserDefinedMaxDroop = 0;
    uint32_t nbUserDefinedRestLoad = 0;
    uint32_t nbDeprecatedLatStiffYUsed = 0;
    static_assert(VehicleDesc::maxNumberOfWheels <= 32, "");  // bit logic for assigned indices
    uint32_t encounteredAttachmentIndices = 0;

    // Wheel attachments via the scanned per-vehicle association (no GetDescendants).
    auto attIt = vehicleComponentTracker.mVehicleWheelAttachments.find(vehiclePath);
    if (attIt != vehicleComponentTracker.mVehicleWheelAttachments.end())
    {
        for (const SdfPath& attachPath : attIt->second)
        {
            if (nbWheels < VehicleDesc::maxNumberOfWheels)
            {
                auto waIt = vehicleComponentTracker.mWheelAttachmentByPath.find(attachPath);
                if (waIt != vehicleComponentTracker.mWheelAttachmentByPath.end())
                {
                    WheelAttachmentDesc wheelAttachment = *waIt->second;  // pre-popped from scan
                    CARB_ASSERT(wheelAttachment.index < static_cast<int>(VehicleDesc::maxNumberOfWheels));
                    if (wheelAttachment.index >= 0)
                    {
                        const uint32_t bitIndex = (1 << wheelAttachment.index);
                        if (!(encounteredAttachmentIndices & bitIndex))
                            encounteredAttachmentIndices |= bitIndex;
                        else
                        {
                            CARB_LOG_ERROR(
                                "Usd Physics: vehicle \"%s\": multiple wheel attachments use the same index %d.\n",
                                vehName, wheelAttachment.index);
                            isValid = false;
                        }
                    }
                    else
                    {
                        // index -1 → choose based on parse order
                        wheelAttachment.index = nbWheels;
                    }

                    if ((!isScaleUniform) && (!toPhysXQuat(wheelAttachment.suspensionFrameOrientation).isIdentity()))
                    {
                        CARB_LOG_WARN("Usd Physics: vehicle \"%s\", wheel attachment \"%s\": ScaleOrientation in suspension frame is not supported. "
                            "You may ignore this if the vehicle frame scale is close to uniform.\n",
                            vehName, attachPath.GetName().c_str());
                        isValid = false;
                    }

                    vehicleDesc.wheelAttachments.push_back(wheelAttachment);

                    const omni::physics::parse::ObjectKey attachKey = attachedStage.keyFor(attachPath);
                    if (omni::physx::internal::hasAppliedSchema<PhysxSchemaPhysxVehicleWheelControllerAPI>(*src, attachKey))
                    {
                        if (!vehicleDesc.drive)
                        {
                            WheelControllerDesc wheelController;
                            if (parseWheelController(attachedStage, attachKey, wheelController))
                                vehicleDesc.wheelControllers.push_back(wheelController);
                            else
                                isValid = false;
                        }
                        else
                        {
                            CARB_LOG_ERROR(
                                "Usd Physics: vehicle \"%s\" has a drive specified but wheel attachment \"%s\" has "
                                "PhysxVehicleWheelControllerAPI applied. This is an illegal configuration.\n",
                                vehName, attachPath.GetName().c_str());
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
                {
                    // Malformed attachment (bad index / nested collider / missing ref):
                    // dropped from the scan + diagnostic already logged there.
                    isValid = false;
                }

                nbWheels++;
            }
            else
            {
                CARB_LOG_ERROR(
                    "Usd Physics: vehicle \"%s\" has more than the maximum allowed number of %d wheels.\n",
                    vehName, VehicleDesc::maxNumberOfWheels);
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
            vehName);
        isValid = false;
    }
    if (nbUserDefinedSprungMass && (nbUserDefinedSprungMass != nbWheels))
    {
        CARB_LOG_ERROR(
            "Usd Physics: vehicle \"%s\": the sprung mass values of the suspensions need to be either all zero or have user defined "
            "positive values.\n", vehName);
        isValid = false;
    }
    if (nbUserDefinedMaxDroop && (nbUserDefinedMaxDroop != nbWheels))
    {
        CARB_LOG_ERROR(
            "Usd Physics: vehicle \"%s\": the max droop values of the suspensions need to be either all negative or have user defined "
            "non-negative values.\n", vehName);
        isValid = false;
    }
    if (nbUserDefinedRestLoad && (nbUserDefinedRestLoad != nbWheels))
    {
        CARB_LOG_ERROR(
            "Usd Physics: vehicle \"%s\": the rest load values of the tires need to be either all zero or have user defined "
            "positive values.\n", vehName);
        isValid = false;
    }
    if (nbDeprecatedLatStiffYUsed && (nbDeprecatedLatStiffYUsed != nbWheels))
    {
        CARB_LOG_ERROR(
            "Usd Physics: vehicle \"%s\": either all tires need to use lateralStiffnessGraph or all tires need to use the "
            "deprecated latStiffX/latStiffY attributes.\n", vehName);
        isValid = false;
    }
    if ((encounteredAttachmentIndices > 0) && (encounteredAttachmentIndices != ((1 << nbWheels) - 1)))
    {
        CARB_LOG_ERROR(
            "Usd Physics: vehicle \"%s\": either all wheel attachment indices need to be -1 or they need to cover all entries in the group "
            "{0, ..., (numberOfWheels-1)}.\n", vehName);
        isValid = false;
    }

    if (isValid && vehicleDesc.differential)  // no differential might be specified
    {
        testDriveAndWheelIndexList(vehicleDesc.drive, vehicleDesc.differential->wheels, nbWheels,
            vehName, "PhysxVehicleMultiWheelDifferentialAPI", isValid);

        if (vehicleDesc.differential->type == DifferentialDesc::eTank)
        {
            const TankDifferentialDesc* tankDifferential = static_cast<const TankDifferentialDesc*>(vehicleDesc.differential);
            const uint32_t trackCount = static_cast<uint32_t>(tankDifferential->trackToWheelIndices.size());
            CARB_ASSERT(tankDifferential->trackToWheelIndices.size() == tankDifferential->numberOfWheelsPerTrack.size());
            for (uint32_t k = 0; k < trackCount; k++)
            {
                const int startIndex = tankDifferential->trackToWheelIndices[k];
                const int endIndexPlusOne = startIndex + tankDifferential->numberOfWheelsPerTrack[k];
                CARB_ASSERT(startIndex >= 0);
                CARB_ASSERT((startIndex == endIndexPlusOne) || (endIndexPlusOne <= tankDifferential->wheelIndicesInTrackOrder.size()));
                for (int l = startIndex; l < endIndexPlusOne; l++)
                {
                    const int wheelIndex = tankDifferential->wheelIndicesInTrackOrder[l];
                    CARB_ASSERT(wheelIndex >= 0);
                    if (wheelIndex >= static_cast<int>(nbWheels))
                    {
                        CARB_LOG_ERROR(
                            "Usd Physics: PhysxVehicleTankDifferentialAPI of vehicle \"%s\" has illegal wheel index %d "
                            "specified in the \"wheelIndicesInTrackOrder\" attribute (index must not be larger than "
                            "numberOfWheels-1).\n", vehName, wheelIndex);
                        isValid = false;
                    }
                }
            }
        }
    }

    if (isValid)
    {
        for (const BrakesDesc* brakesDesc : vehicleDesc.brakes)  // no brakes might be specified
        {
            testDriveAndWheelIndexList(vehicleDesc.drive, brakesDesc->wheels, nbWheels,
                vehName, "PhysxVehicleBrakesAPI", isValid);
        }
    }

    if (isValid && vehicleDesc.steering)  // no steering might be specified
    {
        if (vehicleDesc.steering->type == SteeringDesc::eBasic)
        {
            const SteeringBasicDesc* steeringBasic = static_cast<const SteeringBasicDesc*>(vehicleDesc.steering);
            testDriveAndWheelIndexList(vehicleDesc.drive, steeringBasic->wheels, nbWheels,
                vehName, "PhysxVehicleSteeringAPI", isValid);
        }
        else
        {
            CARB_ASSERT(vehicleDesc.steering->type == SteeringDesc::eAckermann);
            const SteeringAckermannDesc* steeringAckermann = static_cast<const SteeringAckermannDesc*>(vehicleDesc.steering);
            std::vector<int> indexList{ steeringAckermann->wheel0, steeringAckermann->wheel1 };
            testDriveAndWheelIndexList(vehicleDesc.drive, indexList, nbWheels,
                vehName, "PhysxVehicleAckermannSteeringAPI", isValid);
        }
    }

    if (isValid && omni::physx::internal::hasAppliedSchema<PhysxSchemaPhysxVehicleControllerAPI>(*src, vehicleKey))
    {
        if (!vehicleDesc.wheelControllers.size())
        {
            bool controllerParseSuccess;
            if (!omni::physx::internal::hasAppliedSchema<PhysxSchemaPhysxVehicleTankControllerAPI>(*src, vehicleKey))
            {
                vehicleControllerType = eVehicleControllerStandard;
                controllerParseSuccess = parseVehicleController(attachedStage, vehicleKey, vehicleControllerDesc);
            }
            else
            {
                vehicleControllerType = eVehicleControllerTank;
                controllerParseSuccess = parseVehicleTankController(attachedStage, vehicleKey, vehicleTankControllerDesc);

                if (vehicleDesc.drive && (vehicleDesc.drive->type != eVehicleDriveStandard))
                {
                    CARB_LOG_ERROR(
                        "Usd Physics: vehicle \"%s\" has PhysxVehicleTankControllerAPI applied which requires "
                        "a standard drive (see PhysxVehicleDriveStandardAPI).\n", vehName);
                    isValid = false;
                }
                if (!vehicleDesc.differential || (vehicleDesc.differential->type != DifferentialDesc::eTank))
                {
                    CARB_LOG_ERROR(
                        "Usd Physics: vehicle \"%s\" has PhysxVehicleTankControllerAPI applied which requires "
                        "a tank differential (see PhysxVehicleTankDifferentialAPI).\n", vehName);
                    isValid = false;
                }
            }

            if (controllerParseSuccess)
            {
                if (!vehicleDesc.drive)
                {
                    CARB_LOG_ERROR(
                        "Usd Physics: vehicle \"%s\" has PhysxVehicleControllerAPI applied which requires a drive "
                        "being defined but there is none.\n", vehName);
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
                "PhysxVehicleWheelControllerAPI applied. Only one or the other is allowed.\n", vehName);
            isValid = false;
        }
    }

    return isValid;
}

} // namespace usdparser
} // namespace physx
} // namespace omni
