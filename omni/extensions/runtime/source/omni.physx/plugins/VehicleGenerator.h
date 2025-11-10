// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "OmniPvdWriter.h"
#include "PxPhysicsAPI.h"

#include <carb/logging/Log.h>

#include "internal/Internal.h"

#include <private/omni/physx/PhysxUsd.h>

namespace omni
{
namespace physx
{

class VehicleQueryFilterCallback : public ::physx::PxQueryFilterCallback
{
public:
    VehicleQueryFilterCallback()
    {
    }
    virtual ~VehicleQueryFilterCallback()
    {
    }

    // Filter callback used to filter drivable and non-drivable surfaces
    virtual ::physx::PxQueryHitType::Enum preFilter(const ::physx::PxFilterData&,
                                                    const ::physx::PxShape*,
                                                    const ::physx::PxRigidActor*,
                                                    ::physx::PxHitFlags&) override;

    virtual ::physx::PxQueryHitType::Enum postFilter(const ::physx::PxFilterData&,
                                                     const ::physx::PxQueryHit&,
                                                     const ::physx::PxShape*,
                                                     const ::physx::PxRigidActor*) override
    {
        return ::physx::PxQueryHitType::eBLOCK;
    }
};


struct PhysXVehicleType
{
    enum Enum
    {
        eRAW_WHEEL_CONTROL, // vehicle where steer angle, brake torque and drive torque are set per wheel directly

        eDIRECT_DRIVE, // vehicle where steer, brake, throttle, ... commands are set per vehicle to control
                       // the dynamics. There is no engine and the drive torque is defined directly.

        eENGINE_DRIVE // vehicle where steer, brake, throttle, ... commands are set per vehicle to control
                      // the dynamics. There is an engine, gears, clutch etc. to deliver drive torque to the
                      // wheels.
    };
};

struct PhysXVehicleBaseDataMemoryOffsets
{
    size_t brakeCommandResponseStatesOffset;
    size_t steerCommandResponseStatesOffset;
    size_t suspensionParamsOffset;
    size_t suspensionStatesOffset;
    size_t suspensionComplianceParamsOffset;
    size_t suspensionComplianceStatesOffset;
    size_t suspensionForceParamsOffset;
    size_t suspensionForcesOffset;
    size_t roadGeomStatesOffset;
    size_t tireSlipStatesOffset;
    size_t tireGripStatesOffset;
    size_t tireDirectionStatesOffset;
    size_t tireSpeedStatesOffset;
    size_t tireCamberAngleStatesOffset;
    size_t tireStickyStatesOffset;
    size_t tireForceParamsOffset;
    size_t tireForcesOffset;
    size_t wheelParamsOffset;
    size_t wheelRigidBody1dStatesOffset;
    size_t wheelLocalPosesOffset;
    size_t wheelActuationStatesOffset;
    size_t suspensionLegacyParamsOffset;

    static const size_t kInvalidOffset = static_cast<size_t>(0xffffffffFFFFFFFF);
};

class PhysXVehicleBase : public ::physx::vehicle2::PxVehicleRigidBodyComponent,
                         public ::physx::vehicle2::PxVehicleSuspensionComponent,
                         public ::physx::vehicle2::PxVehicleTireComponent,
                         public ::physx::vehicle2::PxVehicleWheelComponent,
                         public ::physx::vehicle2::PxVehiclePVDComponent
{
private:
    struct SuspensionLegacyParams
    {
        float maxDroop;
    };

protected:
    PhysXVehicleBase()
        : mPvdObjectHandles(nullptr),
          mSubstepGroupId(::physx::vehicle2::PxVehicleComponentSequence::eINVALID_SUBSTEP_GROUP)
    {
    }

    virtual ~PhysXVehicleBase()
    {
    }

    virtual void getDataForRigidBodyComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        const ::physx::vehicle2::PxVehicleRigidBodyParams*& rigidBodyParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionForce>& suspensionForces,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireForce>& tireForces,
        const ::physx::vehicle2::PxVehicleAntiRollTorque*& antiRollTorque,
        ::physx::vehicle2::PxVehicleRigidBodyState*& rigidBodyState) override
    {
        axleDescription = &mAxleDescription;
        rigidBodyParams = &mRigidBodyParams;
        suspensionForces.setData(mSuspensionForces);
        tireForces.setData(mTireForces);
        antiRollTorque = nullptr;
        rigidBodyState = &mRigidBodyState;
    }

    virtual void getDataForSuspensionComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        const ::physx::vehicle2::PxVehicleRigidBodyParams*& rigidBodyParams,
        const ::physx::vehicle2::PxVehicleSuspensionStateCalculationParams*& suspensionStateCalculationParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal>& steerResponseStates,
        const ::physx::vehicle2::PxVehicleRigidBodyState*& rigidBodyState,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelParams>& wheelParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionParams>& suspensionParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionComplianceParams>&
            suspensionComplianceParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionForceParams>& suspensionForceParams,
        ::physx::vehicle2::PxVehicleSizedArrayData<const ::physx::vehicle2::PxVehicleAntiRollForceParams>& antiRollForceParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleRoadGeometryState>& wheelRoadGeomStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleSuspensionState>& suspensionStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleSuspensionComplianceState>& suspensionComplianceStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleSuspensionForce>& suspensionForces,
        ::physx::vehicle2::PxVehicleAntiRollTorque*& antiRollTorque) override
    {
        axleDescription = &mAxleDescription;
        rigidBodyParams = &mRigidBodyParams;
        suspensionStateCalculationParams = &mSuspensionStateCalculationParams;
        steerResponseStates.setData(mSteerCommandResponseStates);
        rigidBodyState = &mRigidBodyState;
        wheelParams.setData(mWheelParams);
        suspensionParams.setData(mSuspensionParams);
        suspensionComplianceParams.setData(mSuspensionComplianceParams);
        suspensionForceParams.setData(mSuspensionForceParams);
        antiRollForceParams.setEmpty();
        wheelRoadGeomStates.setData(mRoadGeomStates);
        suspensionStates.setData(mSuspensionStates);
        suspensionComplianceStates.setData(mSuspensionComplianceStates);
        suspensionForces.setData(mSuspensionForces);
        antiRollTorque = nullptr;
    }

    virtual void getDataForTireComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal>& steerResponseStates,
        const ::physx::vehicle2::PxVehicleRigidBodyState*& rigidBodyState,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelActuationState>& actuationStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelParams>& wheelParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionParams>& suspensionParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireForceParams>& tireForceParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleRoadGeometryState>& roadGeomStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionState>& suspensionStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionComplianceState>&
            suspensionComplianceStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionForce>& suspensionForces,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelRigidBody1dState>& wheelRigidBody1DStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleTireGripState>& tireGripStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleTireDirectionState>& tireDirectionStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleTireSpeedState>& tireSpeedStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleTireSlipState>& tireSlipStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleTireCamberAngleState>& tireCamberAngleStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleTireStickyState>& tireStickyStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleTireForce>& tireForces) override
    {
        axleDescription = &mAxleDescription;
        steerResponseStates.setData(mSteerCommandResponseStates);
        rigidBodyState = &mRigidBodyState;
        actuationStates.setData(mWheelActuationStates);
        wheelParams.setData(mWheelParams);
        suspensionParams.setData(mSuspensionParams);
        tireForceParams.setData(mTireForceParams);
        roadGeomStates.setData(mRoadGeomStates);
        suspensionStates.setData(mSuspensionStates);
        suspensionComplianceStates.setData(mSuspensionComplianceStates);
        suspensionForces.setData(mSuspensionForces);
        wheelRigidBody1DStates.setData(mWheelRigidBody1dStates);
        tireGripStates.setData(mTireGripStates);
        tireDirectionStates.setData(mTireDirectionStates);
        tireSpeedStates.setData(mTireSpeedStates);
        tireSlipStates.setData(mTireSlipStates);
        tireCamberAngleStates.setData(mTireCamberAngleStates);
        tireStickyStates.setData(mTireStickyStates);
        tireForces.setData(mTireForces);
    }

    virtual void getDataForWheelComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal>& steerResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelParams>& wheelParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionParams>& suspensionParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelActuationState>& actuationStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionState>& suspensionStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionComplianceState>&
            suspensionComplianceStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireSpeedState>& tireSpeedStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleWheelLocalPose>& wheelLocalPoses) override
    {
        axleDescription = &mAxleDescription;
        steerResponseStates.setData(mSteerCommandResponseStates);
        wheelParams.setData(mWheelParams);
        suspensionParams.setData(mSuspensionParams);
        actuationStates.setData(mWheelActuationStates);
        suspensionStates.setData(mSuspensionStates);
        suspensionComplianceStates.setData(mSuspensionComplianceStates);
        tireSpeedStates.setData(mTireSpeedStates);
        wheelRigidBody1dStates.setData(mWheelRigidBody1dStates);
        wheelLocalPoses.setData(mWheelLocalPoses);
    }

    virtual void getDataForPVDComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        const ::physx::vehicle2::PxVehicleRigidBodyParams*& rbodyParams,
        const ::physx::vehicle2::PxVehicleRigidBodyState*& rbodyState,
        const ::physx::vehicle2::PxVehicleSuspensionStateCalculationParams*& suspStateCalcParams,
        ::physx::vehicle2::PxVehicleSizedArrayData<const ::physx::vehicle2::PxVehicleBrakeCommandResponseParams>&
            brakeResponseParams,
        const ::physx::vehicle2::PxVehicleSteerCommandResponseParams*& steerResponseParams,
        const ::physx::vehicle2::PxVehicleAckermannParams*& ackermannParams,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& brakeResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& steerResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelParams>& wheelParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelActuationState>& wheelActuationStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelLocalPose>& wheelLocalPoses,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleRoadGeometryState>& roadGeomStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionParams>& suspParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionComplianceParams>& suspCompParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionForceParams>& suspForceParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionState>& suspStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionComplianceState>& suspCompStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionForce>& suspForces,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireForceParams>& tireForceParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireDirectionState>& tireDirectionStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireSpeedState>& tireSpeedStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireSlipState>& tireSlipStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireStickyState>& tireStickyStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireGripState>& tireGripStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireCamberAngleState>& tireCamberStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireForce>& tireForces,
        ::physx::vehicle2::PxVehicleSizedArrayData<const ::physx::vehicle2::PxVehicleAntiRollForceParams>& antiRollForceParams,
        const ::physx::vehicle2::PxVehicleAntiRollTorque*& antiRollTorque,
        const ::physx::vehicle2::PxVehicleCommandState*& commandState,
        const ::physx::vehicle2::PxVehicleDirectDriveThrottleCommandResponseParams*& directDriveThrottleResponseParams,
        const ::physx::vehicle2::PxVehicleDirectDriveTransmissionCommandState*& directDriveTransmissionState,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& directDrivethrottleResponseState,
        const ::physx::vehicle2::PxVehicleClutchCommandResponseParams*& clutchResponseParams,
        const ::physx::vehicle2::PxVehicleClutchParams*& clutchParams,
        const ::physx::vehicle2::PxVehicleEngineParams*& engineParams,
        const ::physx::vehicle2::PxVehicleGearboxParams*& gearboxParams,
        const ::physx::vehicle2::PxVehicleAutoboxParams*& autoboxParams,
        const ::physx::vehicle2::PxVehicleMultiWheelDriveDifferentialParams*& multiWheelDiffParams,
        const ::physx::vehicle2::PxVehicleFourWheelDriveDifferentialParams*& fourWheelDiffParams,
        const ::physx::vehicle2::PxVehicleTankDriveDifferentialParams*& tankDiffParams,
        const ::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState*& engineDriveTransmissionState,
        const ::physx::vehicle2::PxVehicleTankDriveTransmissionCommandState*& tankDriveTransmissionState,
        const ::physx::vehicle2::PxVehicleClutchCommandResponseState*& clutchResponseState,
        const ::physx::vehicle2::PxVehicleEngineDriveThrottleCommandResponseState*& engineDriveThrottleResponseState,
        const ::physx::vehicle2::PxVehicleEngineState*& engineState,
        const ::physx::vehicle2::PxVehicleGearboxState*& gearboxState,
        const ::physx::vehicle2::PxVehicleAutoboxState*& autoboxState,
        const ::physx::vehicle2::PxVehicleDifferentialState*& diffState,
        const ::physx::vehicle2::PxVehicleClutchSlipState*& clutchSlipState,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXSuspensionLimitConstraintParams>&
            physxConstraintParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams>&
            physxMaterialFrictionParams,
        const ::physx::vehicle2::PxVehiclePhysXActor*& physxActor,
        const ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryParams*& physxRoadGeomQryParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryState>& physxRoadGeomStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXConstraintState>& physxConstraintStates,
        const ::physx::vehicle2::PxVehiclePhysXSteerState*& physxSteerState,
        ::physx::vehicle2::PxVehiclePvdObjectHandles*& objectHandles) override
    {
        axleDescription = &mAxleDescription;
        rbodyParams = &mRigidBodyParams;
        rbodyState = &mRigidBodyState;
        suspStateCalcParams = &mSuspensionStateCalculationParams;
        brakeResponseParams.setEmpty();
        steerResponseParams = nullptr;
        ackermannParams = nullptr;
        brakeResponseStates.setData(mBrakeCommandResponseStates);
        steerResponseStates.setData(mSteerCommandResponseStates);
        wheelParams.setData(mWheelParams);
        wheelActuationStates.setData(mWheelActuationStates);
        wheelRigidBody1dStates.setData(mWheelRigidBody1dStates);
        wheelLocalPoses.setData(mWheelLocalPoses);
        roadGeomStates.setData(mRoadGeomStates);
        suspParams.setData(mSuspensionParams);
        suspCompParams.setData(mSuspensionComplianceParams);
        suspForceParams.setData(mSuspensionForceParams);
        suspStates.setData(mSuspensionStates);
        suspCompStates.setData(mSuspensionComplianceStates);
        suspForces.setData(mSuspensionForces);
        tireForceParams.setData(mTireForceParams);
        tireDirectionStates.setData(mTireDirectionStates);
        tireSpeedStates.setData(mTireSpeedStates);
        tireSlipStates.setData(mTireSlipStates);
        tireStickyStates.setData(mTireStickyStates);
        tireGripStates.setData(mTireGripStates);
        tireCamberStates.setData(mTireCamberAngleStates);
        tireForces.setData(mTireForces);
        antiRollForceParams.setEmpty();
        antiRollTorque = nullptr;
        commandState = nullptr;
        directDriveThrottleResponseParams = nullptr;
        directDriveTransmissionState = nullptr;
        directDrivethrottleResponseState.setEmpty();
        clutchResponseParams = nullptr;
        clutchParams = nullptr;
        engineParams = nullptr;
        gearboxParams = nullptr;
        autoboxParams = nullptr;
        multiWheelDiffParams = nullptr;
        fourWheelDiffParams = nullptr;
        tankDiffParams = nullptr;
        engineDriveTransmissionState = nullptr;
        tankDriveTransmissionState = nullptr;
        clutchResponseState = nullptr;
        engineDriveThrottleResponseState = nullptr;
        engineState = nullptr;
        gearboxState = nullptr;
        autoboxState = nullptr;
        diffState = nullptr;
        clutchSlipState = nullptr;
        physxConstraintParams.setEmpty();
        physxMaterialFrictionParams.setEmpty();
        physxActor = nullptr;
        physxRoadGeomQryParams = nullptr;
        physxRoadGeomStates.setEmpty();
        physxConstraintStates.setEmpty();
        physxSteerState = nullptr;
        objectHandles = mPvdObjectHandles;
    }

protected:
    static size_t computeDataSizeExcludingClass(const usdparser::VehicleDesc&, PhysXVehicleBaseDataMemoryOffsets&);
    void setDataPointers(uint8_t* memory, const PhysXVehicleBaseDataMemoryOffsets&);
    void setDataValues(const usdparser::VehicleDesc&,
                       const std::vector<uint32_t>& wheelIndexToDataMap,
                       float vehicleMass,
                       const ::physx::PxVec3& vehicleMassSpaceInertiaTensor,
                       const ::physx::PxTransform& vehicleMassFrame,
                       const ::physx::vehicle2::PxVehicleFrame&,
                       const float lengthScale,
                       const float gravityMagnitude);

    static void adjustFixedSizeLookupTable(::physx::vehicle2::PxVehicleFixedSizeLookupTable<::physx::PxReal, 3>& table,
                                           const uint32_t index,
                                           const float value)
    {
        const ::physx::PxU32 pairCount = table.nbDataPairs;
        if (pairCount == 1)
        {
            const float x = table.xVals[0];
            const float y = table.yVals[0];
            table.clear();
            table.addPair(0.0f, (index == 0) ? value : y);
            table.addPair(CARB_CLAMP(x, FLT_EPSILON, 1.0f - FLT_EPSILON), (index == 1) ? value : y);
            table.addPair(1.0f, (index == 2) ? value : y);
        }
        else if (pairCount == 3)
        {
            table.yVals[index] = value;
        }
        else
        {
            CARB_LOG_ERROR(
                "PhysX Vehicle: unexpected number of reference points for camber angle graph detected "
                "while processing change to camberAtRest, camberAtMaxCompression or camberAtMaxDroop. This indicates "
                "that these deprecated attributes and their replacements are used on the same vehicle.\n");
        }
    }

    static void setSuspensionComplianceAngle(const pxr::GfVec2f* angleEntries,
                                             const uint32_t entryCount,
                                             ::physx::vehicle2::PxVehicleFixedSizeLookupTable<::physx::PxReal, 3>& table)
    {
        CARB_ASSERT(entryCount <= 3);

        table.clear();

        for (uint32_t i = 0; i < entryCount; i++)
        {
            const pxr::GfVec2f& entry = angleEntries[i];
            table.addPair(entry[0], entry[1]);
        }
    }

    static void setSuspensionCompliancePoints(const pxr::GfVec4f* pointEntries,
                                              const uint32_t entryCount,
                                              const ::physx::PxVec3& scale,
                                              ::physx::vehicle2::PxVehicleFixedSizeLookupTable<::physx::PxVec3, 3>& table)
    {
        CARB_ASSERT(entryCount <= 3);

        table.clear();

        for (uint32_t i = 0; i < entryCount; i++)
        {
            const pxr::GfVec4f& entry = pointEntries[i];
            table.addPair(entry[0], ::physx::PxVec3(entry[1] * scale.x, entry[2] * scale.y, entry[3] * scale.z));
        }
    }

    inline void setBrakeAndSteerCommandResponseStatesToDefault()
    {
        ::physx::PxMemZero(mBrakeCommandResponseStates, mWheelCapacity * sizeof(mBrakeCommandResponseStates[0]));
        ::physx::PxMemZero(mSteerCommandResponseStates, mWheelCapacity * sizeof(mSteerCommandResponseStates[0]));
    }

    static ::physx::PxVec3 getScaledCoMFramePosition(const ::physx::PxVec3& position,
                                                     const ::physx::PxVec3& scale,
                                                     const ::physx::PxTransform* vehicleMassFrame)
    {
        ::physx::PxVec3 positionAdjusted = position.multiply(scale);
        if (vehicleMassFrame)
            positionAdjusted = vehicleMassFrame->transformInv(positionAdjusted);

        return positionAdjusted;
    }

    void updateSprungMassProperties(const uint32_t wheelIndex,
                                    const float sprungMassValue,
                                    const float massCorrection,
                                    const bool updateMaxDroopValues,
                                    const bool updateRestLoad,
                                    const bool updateLatStiffY);

    void releasePvdObjectHandles(OmniPvdWriter&, ::physx::PxAllocatorCallback&);

public:
    virtual void release(OmniPvdWriter*, ::physx::PxAllocatorCallback*);

    void createPvdObjectHandles(::physx::PxAllocatorCallback&, const uint32_t maxNbMaterialFrictionEntries);

    const ::physx::vehicle2::PxVehiclePvdObjectHandles* getPvdObjectHandles() const
    {
        return mPvdObjectHandles;
    }

    virtual PhysXVehicleType::Enum getType() const = 0;

    virtual void simulateBegin(const float dt, const ::physx::vehicle2::PxVehicleSimulationContext& context) = 0;
    virtual void simulate(const float dt, const ::physx::vehicle2::PxVehicleSimulationContext& context);
    virtual void simulateEnd(const float dt, const ::physx::vehicle2::PxVehicleSimulationContext& context) = 0;

    inline const ::physx::PxTransform& getWheelLocalPose(const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return mWheelLocalPoses[wheelIndex].localPose;
    }

    inline void updateMaxDroop(const uint32_t wheelIndex, const float massCorrection)
    {
        const float oldMaxDroop = getSuspensionMaxDroop(wheelIndex);
        const float newMaxDroop = massCorrection * oldMaxDroop;
        setSuspensionMaxDroop(wheelIndex, newMaxDroop);
    }

    void updateMassProperties(const float mass,
                              const ::physx::PxVec3& massSpaceInertiaTensor,
                              const ::physx::PxTransform* vehicleMassFrameChange,
                              const ::physx::vehicle2::PxVehicleAxes::Enum upAxis,
                              const bool updateSprungMassRelatedValues,
                              const bool updateMaxDroopValues,
                              const bool updateRestLoad,
                              const bool updateLatStiffY);

    bool updateSprungMassProperties(const ::physx::vehicle2::PxVehicleAxes::Enum upAxis,
                                    const bool updateMaxDroopValues,
                                    const bool updateRestLoad,
                                    const bool updateLatStiffY,
                                    const bool deprecatedWheelCenterOfMassChange);

    inline void setWheelRadius(const uint32_t wheelIndex, const float radius)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mWheelParams[wheelIndex].radius = radius;
    }

    inline void setWheelHalfWidth(const uint32_t wheelIndex, const float halfWidth)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mWheelParams[wheelIndex].halfWidth = halfWidth;
    }

    inline void setWheelMass(const uint32_t wheelIndex, const float mass)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mWheelParams[wheelIndex].mass = mass;
    }

    inline void setWheelMoi(const uint32_t wheelIndex, const float moi)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mWheelParams[wheelIndex].moi = moi;
    }

    inline void setWheelDampingRate(const uint32_t wheelIndex, const float dampingRate)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mWheelParams[wheelIndex].dampingRate = dampingRate;
    }

    // deprecated
    inline void setWheelCenterOffset(const uint32_t wheelIndex,
                                     const ::physx::PxVec3& offset,
                                     const ::physx::PxVec3& scale,
                                     const ::physx::PxTransform* vehicleMassFrame)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);

        const ::physx::PxVec3 offsetMod = PhysXVehicleBase::getScaledCoMFramePosition(offset, scale, vehicleMassFrame);

        const float maxCompression = getSuspensionMaxCompression(wheelIndex);
        mSuspensionParams[wheelIndex].suspensionAttachment.p =
            offsetMod - (mSuspensionParams[wheelIndex].suspensionTravelDir * maxCompression);
    }

    inline float getWheelRotationSpeed(const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return mWheelRigidBody1dStates[wheelIndex].correctedRotationSpeed;
    }

    inline void setWheelRotationSpeed(const uint32_t wheelIndex, const float rotationSpeed)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        ::physx::vehicle2::PxVehicleWheelRigidBody1dState& wrbs = mWheelRigidBody1dStates[wheelIndex];
        wrbs.rotationSpeed = rotationSpeed;
        wrbs.correctedRotationSpeed = rotationSpeed;
    }

    inline float getWheelRotationAngle(const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return mWheelRigidBody1dStates[wheelIndex].rotationAngle;
    }

    inline void setWheelRotationAngle(const uint32_t wheelIndex, const float rotationAngle)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mWheelRigidBody1dStates[wheelIndex].rotationAngle = rotationAngle;
    }

    inline float getWheelSteerAngle(const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return mSteerCommandResponseStates[wheelIndex] + mSuspensionComplianceStates[wheelIndex].toe;
    }

    inline bool isWheelOnGround(const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return ::physx::vehicle2::PxVehicleIsWheelOnGround(mSuspensionStates[wheelIndex]);
    }

    inline const ::physx::vehicle2::PxVehicleRoadGeometryState& getRoadGeometryState(const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return mRoadGeomStates[wheelIndex];
    }

    inline void setSuspensionStiffness(const uint32_t wheelIndex, const float stiffness)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mSuspensionForceParams[wheelIndex].stiffness = stiffness;
    }

    inline void setSuspensionDamping(const uint32_t wheelIndex, const float damping)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mSuspensionForceParams[wheelIndex].damping = damping;
    }

    inline float getSuspensionSprungMass(const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return mSuspensionForceParams[wheelIndex].sprungMass;
    }

    inline void setSuspensionSprungMass(const uint32_t wheelIndex, const float sprungMass)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mSuspensionForceParams[wheelIndex].sprungMass = sprungMass;
    }

    inline float getSuspensionMaxDroop(const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return mSuspensionLegacyParams[wheelIndex].maxDroop;
    }

    inline void setSuspensionMaxDroop(const uint32_t wheelIndex, const float maxDroop)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        const float maxCompression = getSuspensionMaxCompression(wheelIndex);
        mSuspensionParams[wheelIndex].suspensionTravelDist = maxCompression + maxDroop;
        mSuspensionLegacyParams[wheelIndex].maxDroop = maxDroop;
    }

    inline float getSuspensionMaxCompression(const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return (mSuspensionParams[wheelIndex].suspensionTravelDist - mSuspensionLegacyParams[wheelIndex].maxDroop);
    }

    inline void setSuspensionMaxCompression(const uint32_t wheelIndex, const float maxCompression)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        const float oldMaxCompression = getSuspensionMaxCompression(wheelIndex);
        mSuspensionParams[wheelIndex].suspensionTravelDist =
            maxCompression + mSuspensionLegacyParams[wheelIndex].maxDroop;

        // to match legacy behavior
        const float delta = oldMaxCompression - maxCompression;
        mSuspensionParams[wheelIndex].suspensionAttachment.p += mSuspensionParams[wheelIndex].suspensionTravelDir * delta;
    }

    inline void setSuspensionTravelDistance(const uint32_t wheelIndex, const float travelDistance)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        const float oldMaxCompression = getSuspensionMaxCompression(wheelIndex);
        mSuspensionParams[wheelIndex].suspensionTravelDist = travelDistance;

        mSuspensionLegacyParams[wheelIndex].maxDroop = CARB_MAX(travelDistance - oldMaxCompression, 0.0f);
    }

    // deprecated
    inline void setSuspensionCamberAtRest(const uint32_t wheelIndex, const float camberAtRest)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        ::physx::vehicle2::PxVehicleFixedSizeLookupTable<::physx::PxReal, 3>& camberTable =
            mSuspensionComplianceParams[wheelIndex].wheelCamberAngle;
        adjustFixedSizeLookupTable(camberTable, 1, camberAtRest);
    }

    // deprecated
    inline void setSuspensionCamberAtMaxCompression(const uint32_t wheelIndex, const float camberAtMaxCompression)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        ::physx::vehicle2::PxVehicleFixedSizeLookupTable<::physx::PxReal, 3>& camberTable =
            mSuspensionComplianceParams[wheelIndex].wheelCamberAngle;
        adjustFixedSizeLookupTable(camberTable, 2, camberAtMaxCompression);
    }

    // deprecated
    inline void setSuspensionCamberAtMaxDroop(const uint32_t wheelIndex, const float camberAtMaxDroop)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        ::physx::vehicle2::PxVehicleFixedSizeLookupTable<::physx::PxReal, 3>& camberTable =
            mSuspensionComplianceParams[wheelIndex].wheelCamberAngle;
        adjustFixedSizeLookupTable(camberTable, 0, camberAtMaxDroop);
    }

    inline void setSuspensionComplianceWheelCamberAngle(const uint32_t wheelIndex,
                                                        const pxr::GfVec2f* wheelCamberAngleEntries,
                                                        const uint32_t entryCount)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);

        setSuspensionComplianceAngle(
            wheelCamberAngleEntries, entryCount, mSuspensionComplianceParams[wheelIndex].wheelCamberAngle);
    }

    // deprecated
    inline void setSuspensionToeAngle(const uint32_t wheelIndex, const float toeAngle)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        ::physx::vehicle2::PxVehicleFixedSizeLookupTable<::physx::PxReal, 3>& toeTable =
            mSuspensionComplianceParams[wheelIndex].wheelToeAngle;
        if (toeTable.nbDataPairs == 1)
        {
            toeTable.yVals[0] = toeAngle;
        }
        else if (toeTable.nbDataPairs == 0)
        {
            toeTable.addPair(0.0f, toeAngle);
        }
        else
        {
            CARB_LOG_ERROR(
                "PhysX Vehicle: setting a single toe angle when toe angle was initially defined as a "
                "graph with multiple reference points is not supported. This indicates that deprecated attributes "
                "and their replacements are used on the same vehicle.\n");
        }
    }

    inline void setSuspensionComplianceWheelToeAngle(const uint32_t wheelIndex,
                                                     const pxr::GfVec2f* wheelToeAngleEntries,
                                                     const uint32_t entryCount)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);

        setSuspensionComplianceAngle(
            wheelToeAngleEntries, entryCount, mSuspensionComplianceParams[wheelIndex].wheelToeAngle);
    }

    inline void setSuspensionTravelDirection(const uint32_t wheelIndex,
                                             const ::physx::PxVec3& direction,
                                             const ::physx::PxTransform* vehicleMassFrame)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        if (vehicleMassFrame)
            mSuspensionParams[wheelIndex].suspensionTravelDir = vehicleMassFrame->q.rotateInv(direction);
        else
            mSuspensionParams[wheelIndex].suspensionTravelDir = direction; // deprecated
    }

    // deprecated
    void setSuspensionForceAppPointOffset(const uint32_t wheelIndex,
                                          const ::physx::PxVec3& offset,
                                          const ::physx::PxVec3& scale,
                                          const ::physx::PxTransform* vehicleMassFrame);

    inline float getSuspensionJounce(const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return (mSuspensionStates[wheelIndex].jounce != PX_VEHICLE_UNSPECIFIED_JOUNCE) ?
                   mSuspensionStates[wheelIndex].jounce :
                   0.0f;
    }

    inline void setSuspensionComplianceSuspensionForceAppPoint(const uint32_t wheelIndex,
                                                               const pxr::GfVec4f* suspForceAppPointEntries,
                                                               const uint32_t entryCount,
                                                               const ::physx::PxVec3& scale)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);

        setSuspensionCompliancePoints(
            suspForceAppPointEntries, entryCount, scale, mSuspensionComplianceParams[wheelIndex].suspForceAppPoint);
    }

    inline const ::physx::PxVec3& getSuspensionForce(const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return mSuspensionForces[wheelIndex].force;
    }

    inline float getTireRestLoad(const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return mTireForceParams[wheelIndex].restLoad;
    }

    inline void setTireRestLoad(const uint32_t wheelIndex, const float restLoad)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mTireForceParams[wheelIndex].restLoad = restLoad;
    }

    inline void setTireLateralStiffnessX(const uint32_t wheelIndex, const float value)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mTireForceParams[wheelIndex].latStiffX = value;
    }

    inline float getTireLateralStiffnessY(const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return mTireForceParams[wheelIndex].latStiffY;
    }

    inline void setTireLateralStiffnessY(const uint32_t wheelIndex, const float value)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mTireForceParams[wheelIndex].latStiffY = value;
    }

    inline void setTireLongitudinalStiffness(const uint32_t wheelIndex, const float value)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mTireForceParams[wheelIndex].longStiff = value;
    }

    inline void setTireCamberStiffness(const uint32_t wheelIndex, const float value)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mTireForceParams[wheelIndex].camberStiff = value;
    }

    // deprecated
    void setTireForceAppPointOffset(const uint32_t wheelIndex,
                                    const ::physx::PxVec3& offset,
                                    const ::physx::PxVec3& scale,
                                    const ::physx::PxTransform* vehicleMassFrame);

    inline const float getTireFriction(const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return mTireGripStates[wheelIndex].friction;
    }

    inline void setSuspensionComplianceTireForceAppPoint(const uint32_t wheelIndex,
                                                         const pxr::GfVec4f* tireForceAppPointEntries,
                                                         const uint32_t entryCount,
                                                         const ::physx::PxVec3& scale)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);

        setSuspensionCompliancePoints(
            tireForceAppPointEntries, entryCount, scale, mSuspensionComplianceParams[wheelIndex].tireForceAppPoint);
    }

    inline const ::physx::PxVec3& getTireLongitudinalDirection(const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return mTireDirectionStates[wheelIndex].directions[::physx::vehicle2::PxVehicleTireDirectionModes::eLONGITUDINAL];
    }

    inline const ::physx::PxVec3& getTireLateralDirection(const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return mTireDirectionStates[wheelIndex].directions[::physx::vehicle2::PxVehicleTireDirectionModes::eLATERAL];
    }

    inline const float getTireLongitudinalSlip(const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return mTireSlipStates[wheelIndex].slips[::physx::vehicle2::PxVehicleTireDirectionModes::eLONGITUDINAL];
    }

    inline const float getTireLateralSlip(const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return mTireSlipStates[wheelIndex].slips[::physx::vehicle2::PxVehicleTireDirectionModes::eLATERAL];
    }

    inline ::physx::PxVec3 getTireForce(const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return mTireForces[wheelIndex].forces[::physx::vehicle2::PxVehicleTireDirectionModes::eLONGITUDINAL] +
               mTireForces[wheelIndex].forces[::physx::vehicle2::PxVehicleTireDirectionModes::eLATERAL];
    }

    inline const ::physx::vehicle2::PxVehicleTireSlipParams& getTireSlipParams() const
    {
        return mTireSlipParams;
    }

    inline const ::physx::vehicle2::PxVehicleTireStickyParams& getTireStickyParams() const
    {
        return mTireStickyParams;
    }

    inline void setTireMinPassiveLongitudinalSlipDenominator(const float value)
    {
        mTireSlipParams.minPassiveLongSlipDenominator = value;
    }

    inline void setTireMinActiveLongitudinalSlipDenominator(const float value)
    {
        mTireSlipParams.minActiveLongSlipDenominator = value;
    }

    inline void setTireMinLateralSlipDenominator(const float value)
    {
        mTireSlipParams.minLatSlipDenominator = value;
    }

    inline void setTireLongitudinalStickyThresholdSpeed(const float value)
    {
        mTireStickyParams.stickyParams[::physx::vehicle2::PxVehicleTireDirectionModes::eLONGITUDINAL].thresholdSpeed =
            value;
    }

    inline void setTireLongitudinalStickyThresholdTime(const float value)
    {
        mTireStickyParams.stickyParams[::physx::vehicle2::PxVehicleTireDirectionModes::eLONGITUDINAL].thresholdTime =
            value;
    }

    inline void setTireLongitudinalStickyDamping(const float value)
    {
        mTireStickyParams.stickyParams[::physx::vehicle2::PxVehicleTireDirectionModes::eLONGITUDINAL].damping = value;
    }

    inline void setTireLateralStickyThresholdSpeed(const float value)
    {
        mTireStickyParams.stickyParams[::physx::vehicle2::PxVehicleTireDirectionModes::eLATERAL].thresholdSpeed = value;
    }

    inline void setTireLateralStickyThresholdTime(const float value)
    {
        mTireStickyParams.stickyParams[::physx::vehicle2::PxVehicleTireDirectionModes::eLATERAL].thresholdTime = value;
    }

    inline void setTireLateralStickyDamping(const float value)
    {
        mTireStickyParams.stickyParams[::physx::vehicle2::PxVehicleTireDirectionModes::eLATERAL].damping = value;
    }

    inline void setSuspensionFramePosition(const uint32_t wheelIndex,
                                           const ::physx::PxVec3& position,
                                           const ::physx::PxVec3& scale,
                                           const ::physx::PxTransform* vehicleMassFrame)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        const ::physx::PxVec3 positionAdjusted =
            PhysXVehicleBase::getScaledCoMFramePosition(position, scale, vehicleMassFrame);
        mSuspensionParams[wheelIndex].suspensionAttachment.p = positionAdjusted;
    }

    inline void setSuspensionFrameOrientation(const uint32_t wheelIndex,
                                              const ::physx::PxQuat& orientation,
                                              const ::physx::PxTransform* vehicleMassFrame)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        if (vehicleMassFrame)
            mSuspensionParams[wheelIndex].suspensionAttachment.q = vehicleMassFrame->q.getConjugate() * orientation;
        else
            mSuspensionParams[wheelIndex].suspensionAttachment.q = orientation; // deprecated
    }

    inline void setWheelFramePosition(const uint32_t wheelIndex,
                                      const ::physx::PxVec3& position,
                                      const ::physx::PxVec3& scale)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mSuspensionParams[wheelIndex].wheelAttachment.p = position.multiply(scale);
    }

    inline void setWheelFrameOrientation(const uint32_t wheelIndex, const ::physx::PxQuat& orientation)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mSuspensionParams[wheelIndex].wheelAttachment.q = orientation;
    }

    inline void setLimitSuspensionExpansionVelocity(const bool limit)
    {
        mSuspensionStateCalculationParams.limitSuspensionExpansionVelocity = limit;
    }

    virtual float getForwardVelocity(const ::physx::vehicle2::PxVehicleFrame&) const;

    virtual void disableWheel(const uint32_t wheelIndex);

    virtual void setWheelsToRestState();
    virtual void setToRestState();


protected:
    ::physx::vehicle2::PxVehicleComponentSequence mComponentSequence;

    ::physx::vehicle2::PxVehicleAxleDescription mAxleDescription;

    ::physx::PxReal* mBrakeCommandResponseStates;
    ::physx::PxReal* mSteerCommandResponseStates;

    ::physx::vehicle2::PxVehicleSuspensionParams* mSuspensionParams;
    ::physx::vehicle2::PxVehicleSuspensionState* mSuspensionStates;
    ::physx::vehicle2::PxVehicleSuspensionStateCalculationParams mSuspensionStateCalculationParams;
    ::physx::vehicle2::PxVehicleSuspensionComplianceParams* mSuspensionComplianceParams;
    ::physx::vehicle2::PxVehicleSuspensionComplianceState* mSuspensionComplianceStates;
    ::physx::vehicle2::PxVehicleSuspensionForceParams* mSuspensionForceParams;
    ::physx::vehicle2::PxVehicleSuspensionForce* mSuspensionForces;

    ::physx::vehicle2::PxVehicleRoadGeometryState* mRoadGeomStates;

    ::physx::vehicle2::PxVehicleTireSlipParams mTireSlipParams;
    ::physx::vehicle2::PxVehicleTireSlipState* mTireSlipStates;
    ::physx::vehicle2::PxVehicleTireGripState* mTireGripStates;
    ::physx::vehicle2::PxVehicleTireDirectionState* mTireDirectionStates;
    ::physx::vehicle2::PxVehicleTireSpeedState* mTireSpeedStates;
    ::physx::vehicle2::PxVehicleTireCamberAngleState* mTireCamberAngleStates;
    ::physx::vehicle2::PxVehicleTireStickyState* mTireStickyStates;
    ::physx::vehicle2::PxVehicleTireForceParams* mTireForceParams;
    ::physx::vehicle2::PxVehicleTireForce* mTireForces;

    ::physx::vehicle2::PxVehicleWheelParams* mWheelParams;
    ::physx::vehicle2::PxVehicleWheelRigidBody1dState* mWheelRigidBody1dStates;
    ::physx::vehicle2::PxVehicleWheelLocalPose* mWheelLocalPoses;
    ::physx::vehicle2::PxVehicleWheelActuationState* mWheelActuationStates;

    ::physx::vehicle2::PxVehicleRigidBodyParams mRigidBodyParams;
    ::physx::vehicle2::PxVehicleRigidBodyState mRigidBodyState;

    SuspensionLegacyParams* mSuspensionLegacyParams; // deprecated (remove once maxDroop/maxCompression is gone)

    ::physx::vehicle2::PxVehiclePvdObjectHandles* mPvdObjectHandles;

    float mSubstepThresholdLongitudinalSpeed;
    uint8_t mLowForwardSpeedSubstepCount;
    uint8_t mHighForwardSpeedSubstepCount;
    uint8_t mSubstepGroupId;
    uint32_t mWheelCapacity; // the number of wheels including disabled ones

    ::physx::vehicle2::PxVehicleTireStickyParams mTireStickyParams;
};


struct PhysXActorVehicleBaseDataMemoryOffsets : PhysXVehicleBaseDataMemoryOffsets
{
    size_t physxRoadGeomQueryFilterDatasOffset;
    size_t physxMaterialFrictionParamsOffset;
    size_t physxRoadGeomQueryStatesOffset;
    size_t physxSuspensionLimitConstraintParamsOffset;
    size_t physxConstraintConnectorsOffset;
    size_t physxWheelShapeLocalPoseOffset;
};

class PhysXActorVehicleBase : public PhysXVehicleBase,
                              public ::physx::vehicle2::PxVehiclePhysXActorEndComponent,
                              public ::physx::vehicle2::PxVehiclePhysXConstraintComponent,
                              public ::physx::vehicle2::PxVehiclePhysXRoadGeometrySceneQueryComponent
{
protected:
    PhysXActorVehicleBase() : PhysXVehicleBase()
    {
    }
    virtual ~PhysXActorVehicleBase();

    virtual void getDataForPhysXActorEndComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        const ::physx::vehicle2::PxVehicleRigidBodyState*& rigidBodyState,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelParams>& wheelParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxTransform>& wheelShapeLocalPoses,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelLocalPose>& wheelLocalPoses,
        const ::physx::vehicle2::PxVehicleGearboxState*& gearState,
        const ::physx::PxReal*& throttle,
        ::physx::vehicle2::PxVehiclePhysXActor*& physxActor) override
    {
        axleDescription = &mAxleDescription;
        rigidBodyState = &mRigidBodyState;
        wheelParams.setData(mWheelParams);
        wheelShapeLocalPoses.setData(mPhysxWheelShapeLocalPoses);
        wheelRigidBody1dStates.setData(mWheelRigidBody1dStates);
        wheelLocalPoses.setData(mWheelLocalPoses);
        physxActor = &mPhysxActor;

        gearState = nullptr;
        throttle = nullptr;
    }

    virtual void getDataForPhysXConstraintComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        const ::physx::vehicle2::PxVehicleRigidBodyState*& rigidBodyState,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionParams>& suspensionParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXSuspensionLimitConstraintParams>&
            suspensionLimitParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionState>& suspensionStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionComplianceState>&
            suspensionComplianceStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleRoadGeometryState>& wheelRoadGeomStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireDirectionState>& tireDirectionStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireStickyState>& tireStickyStates,
        ::physx::vehicle2::PxVehiclePhysXConstraints*& constraints) override
    {
        axleDescription = &mAxleDescription;
        rigidBodyState = &mRigidBodyState;
        suspensionParams.setData(mSuspensionParams);
        suspensionLimitParams.setData(mPhysxSuspensionLimitConstraintParams);
        suspensionStates.setData(mSuspensionStates);
        suspensionComplianceStates.setData(mSuspensionComplianceStates);
        wheelRoadGeomStates.setData(mRoadGeomStates);
        tireDirectionStates.setData(mTireDirectionStates);
        tireStickyStates.setData(mTireStickyStates);
        constraints = &mPhysxConstraints;
    }

    virtual void getDataForPhysXRoadGeometrySceneQueryComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        const ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryParams*& roadGeomParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal>& steerResponseStates,
        const ::physx::vehicle2::PxVehicleRigidBodyState*& rigidBodyState,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelParams>& wheelParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionParams>& suspensionParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams>&
            materialFrictionParams,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleRoadGeometryState>& roadGeometryStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryState>&
            physxRoadGeometryStates) override
    {
        axleDescription = &mAxleDescription;
        roadGeomParams = &mPhysxRoadGeometryQueryParams;
        steerResponseStates.setData(mSteerCommandResponseStates);
        rigidBodyState = &mRigidBodyState;
        wheelParams.setData(mWheelParams);
        suspensionParams.setData(mSuspensionParams);
        materialFrictionParams.setData(mPhysxMaterialFrictionParams);
        roadGeometryStates.setData(mRoadGeomStates);
        physxRoadGeometryStates.setData(mPhysxRoadGeometryQueryStates);
    }

    virtual void getDataForPVDComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        const ::physx::vehicle2::PxVehicleRigidBodyParams*& rbodyParams,
        const ::physx::vehicle2::PxVehicleRigidBodyState*& rbodyState,
        const ::physx::vehicle2::PxVehicleSuspensionStateCalculationParams*& suspStateCalcParams,
        ::physx::vehicle2::PxVehicleSizedArrayData<const ::physx::vehicle2::PxVehicleBrakeCommandResponseParams>&
            brakeResponseParams,
        const ::physx::vehicle2::PxVehicleSteerCommandResponseParams*& steerResponseParams,
        const ::physx::vehicle2::PxVehicleAckermannParams*& ackermannParams,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& brakeResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& steerResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelParams>& wheelParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelActuationState>& wheelActuationStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelLocalPose>& wheelLocalPoses,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleRoadGeometryState>& roadGeomStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionParams>& suspParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionComplianceParams>& suspCompParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionForceParams>& suspForceParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionState>& suspStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionComplianceState>& suspCompStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionForce>& suspForces,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireForceParams>& tireForceParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireDirectionState>& tireDirectionStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireSpeedState>& tireSpeedStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireSlipState>& tireSlipStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireStickyState>& tireStickyStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireGripState>& tireGripStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireCamberAngleState>& tireCamberStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireForce>& tireForces,
        ::physx::vehicle2::PxVehicleSizedArrayData<const ::physx::vehicle2::PxVehicleAntiRollForceParams>& antiRollForceParams,
        const ::physx::vehicle2::PxVehicleAntiRollTorque*& antiRollTorque,
        const ::physx::vehicle2::PxVehicleCommandState*& commandState,
        const ::physx::vehicle2::PxVehicleDirectDriveThrottleCommandResponseParams*& directDriveThrottleResponseParams,
        const ::physx::vehicle2::PxVehicleDirectDriveTransmissionCommandState*& directDriveTransmissionState,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& directDrivethrottleResponseState,
        const ::physx::vehicle2::PxVehicleClutchCommandResponseParams*& clutchResponseParams,
        const ::physx::vehicle2::PxVehicleClutchParams*& clutchParams,
        const ::physx::vehicle2::PxVehicleEngineParams*& engineParams,
        const ::physx::vehicle2::PxVehicleGearboxParams*& gearboxParams,
        const ::physx::vehicle2::PxVehicleAutoboxParams*& autoboxParams,
        const ::physx::vehicle2::PxVehicleMultiWheelDriveDifferentialParams*& multiWheelDiffParams,
        const ::physx::vehicle2::PxVehicleFourWheelDriveDifferentialParams*& fourWheelDiffParams,
        const ::physx::vehicle2::PxVehicleTankDriveDifferentialParams*& tankDiffParams,
        const ::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState*& engineDriveTransmissionState,
        const ::physx::vehicle2::PxVehicleTankDriveTransmissionCommandState*& tankDriveTransmissionState,
        const ::physx::vehicle2::PxVehicleClutchCommandResponseState*& clutchResponseState,
        const ::physx::vehicle2::PxVehicleEngineDriveThrottleCommandResponseState*& engineDriveThrottleResponseState,
        const ::physx::vehicle2::PxVehicleEngineState*& engineState,
        const ::physx::vehicle2::PxVehicleGearboxState*& gearboxState,
        const ::physx::vehicle2::PxVehicleAutoboxState*& autoboxState,
        const ::physx::vehicle2::PxVehicleDifferentialState*& diffState,
        const ::physx::vehicle2::PxVehicleClutchSlipState*& clutchSlipState,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXSuspensionLimitConstraintParams>&
            physxConstraintParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams>&
            physxMaterialFrictionParams,
        const ::physx::vehicle2::PxVehiclePhysXActor*& physxActor,
        const ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryParams*& physxRoadGeomQryParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryState>& physxRoadGeomStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXConstraintState>& physxConstraintStates,
        const ::physx::vehicle2::PxVehiclePhysXSteerState*& physxSteerState,
        ::physx::vehicle2::PxVehiclePvdObjectHandles*& objectHandles) override
    {
        PhysXVehicleBase::getDataForPVDComponent(
            axleDescription, rbodyParams, rbodyState, suspStateCalcParams, brakeResponseParams, steerResponseParams,
            ackermannParams, brakeResponseStates, steerResponseStates, wheelParams, wheelActuationStates,
            wheelRigidBody1dStates, wheelLocalPoses, roadGeomStates, suspParams, suspCompParams, suspForceParams,
            suspStates, suspCompStates, suspForces, tireForceParams, tireDirectionStates, tireSpeedStates,
            tireSlipStates, tireStickyStates, tireGripStates, tireCamberStates, tireForces, antiRollForceParams,
            antiRollTorque, commandState, directDriveThrottleResponseParams, directDriveTransmissionState,
            directDrivethrottleResponseState, clutchResponseParams, clutchParams, engineParams, gearboxParams,
            autoboxParams, multiWheelDiffParams, fourWheelDiffParams, tankDiffParams, engineDriveTransmissionState,
            tankDriveTransmissionState, clutchResponseState, engineDriveThrottleResponseState, engineState, gearboxState,
            autoboxState, diffState, clutchSlipState, physxConstraintParams, physxMaterialFrictionParams, physxActor,
            physxRoadGeomQryParams, physxRoadGeomStates, physxConstraintStates, physxSteerState, objectHandles);

        physxConstraintParams.setData(mPhysxSuspensionLimitConstraintParams);
        physxMaterialFrictionParams.setData(mPhysxMaterialFrictionParams);
        physxActor = &mPhysxActor;
        physxRoadGeomQryParams = &mPhysxRoadGeometryQueryParams;
        physxRoadGeomStates.setData(mPhysxRoadGeometryQueryStates);
        physxConstraintStates.setData(mPhysxConstraints.constraintStates);
    }

protected:
    static size_t computeDataSizeExcludingClass(const usdparser::VehicleDesc&, PhysXActorVehicleBaseDataMemoryOffsets&);
    void setDataPointers(uint8_t* memory, const PhysXActorVehicleBaseDataMemoryOffsets&);
    void setDataValues(
        const usdparser::VehicleDesc&,
        ::physx::PxRigidDynamic& vehicleActor,
        ::physx::PxPhysics& pxPhysics,
        const std::vector<::physx::PxTransform>& wheelShapeLocalPoses,
        const std::vector<::physx::PxShape*>& wheelShapeMapping,
        const std::vector<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>& tireMaterialFrictionTables,
        const std::vector<uint32_t>& wheelIndexToDataMap,
        const ::physx::vehicle2::PxVehicleFrame&,
        const float gravityMagnitude);

public:
    void simulateEnd(const float dt, const ::physx::vehicle2::PxVehicleSimulationContext&) override;

    inline ::physx::PxRigidDynamic* getRigidDynamicActor()
    {
        CARB_ASSERT(mPhysxActor.rigidBody->getType() == ::physx::PxActorType::eRIGID_DYNAMIC);
        return mPhysxActor.rigidBody->is<::physx::PxRigidDynamic>();
    }

    inline const ::physx::PxRigidDynamic* getRigidDynamicActor() const
    {
        CARB_ASSERT(mPhysxActor.rigidBody->getType() == ::physx::PxActorType::eRIGID_DYNAMIC);
        return mPhysxActor.rigidBody->is<::physx::PxRigidDynamic>();
    }

    inline ::physx::PxRigidDynamic* getRigidDynamicActorNoCheck() // for example if the actor might have been released
                                                                  // already
    {
        return static_cast<::physx::PxRigidDynamic*>(mPhysxActor.rigidBody);
    }

    inline bool isSleeping() const
    {
        const ::physx::PxRigidDynamic* rd = getRigidDynamicActor();
        return rd->isSleeping();
    }

    inline void setTireMaterialFrictionTable(
        const uint32_t wheelIndex, const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams* materialFrictionTable)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mPhysxMaterialFrictionParams[wheelIndex] = materialFrictionTable;
    }

    inline void setWheelFilterData(const uint32_t wheelIndex, const ::physx::PxFilterData& fd)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mPhysxRoadGeometryQueryParams.filterDataEntries[wheelIndex].data = fd;
    }

    inline void setWheelShape(const uint32_t wheelIndex, ::physx::PxShape* shape)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mPhysxActor.wheelShapes[wheelIndex] = shape;
    }
    void removeWheelShape(const ::physx::PxShape*);

    inline ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryType::Enum getRoadGeometryQueryType() const
    {
        return mPhysxRoadGeometryQueryParams.roadGeometryQueryType;
    }

    inline const ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryState& getPhysXRoadGeometryQueryState(
        const uint32_t wheelIndex) const
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        return mPhysxRoadGeometryQueryStates[wheelIndex];
    }

    virtual float getForwardVelocity(const ::physx::vehicle2::PxVehicleFrame&) const override;

    virtual void disableWheel(const uint32_t wheelIndex) override;

    virtual void setWheelsToRestState() override;
    virtual void setToRestState() override;


protected:
    ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryParams mPhysxRoadGeometryQueryParams;
    const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams** mPhysxMaterialFrictionParams;
    ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryState* mPhysxRoadGeometryQueryStates;

    ::physx::vehicle2::PxVehiclePhysXSuspensionLimitConstraintParams* mPhysxSuspensionLimitConstraintParams;
    ::physx::vehicle2::PxVehicleConstraintConnector* mPhysxConstraintConnectors;
    ::physx::vehicle2::PxVehiclePhysXConstraints mPhysxConstraints;

    ::physx::vehicle2::PxVehiclePhysXActor mPhysxActor;

    ::physx::PxTransform* mPhysxWheelShapeLocalPoses;
};


class PhysXVehicleRawWheelControlBeginComponent : public ::physx::vehicle2::PxVehicleComponent
{
public:
    PhysXVehicleRawWheelControlBeginComponent() : ::physx::vehicle2::PxVehicleComponent()
    {
    }

    ~PhysXVehicleRawWheelControlBeginComponent()
    {
    }

    virtual void getDataForPhysXVehicleRawWheelControlBeginComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal>& steerCommandResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal>& throttleCommandResponseStates,
        ::physx::vehicle2::PxVehiclePhysXActor*& physxActor,
        ::physx::vehicle2::PxVehiclePhysXSteerState*& physxSteerStates,
        ::physx::vehicle2::PxVehiclePhysXConstraints*& physxConstraints,
        ::physx::vehicle2::PxVehicleRigidBodyState*& rigidBodyState,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates) = 0;

    virtual bool update(const ::physx::PxReal dt, const ::physx::vehicle2::PxVehicleSimulationContext& context) override;
};

struct PhysXVehicleRawWheelControlDataMemoryOffsets : PhysXActorVehicleBaseDataMemoryOffsets
{
    size_t physxSteerStatesOffset;
    size_t throttleCommandResponseStatesOffset;
};

class PhysXVehicleRawWheelControl : public PhysXActorVehicleBase,
                                    public PhysXVehicleRawWheelControlBeginComponent,
                                    public ::physx::vehicle2::PxVehicleDirectDriveActuationStateComponent,
                                    public ::physx::vehicle2::PxVehicleDirectDrivetrainComponent
{
public:
    PhysXVehicleRawWheelControl() : PhysXActorVehicleBase()
    {
    }

protected:
    virtual ~PhysXVehicleRawWheelControl()
    {
    }

    virtual void getDataForPhysXVehicleRawWheelControlBeginComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal>& steerCommandResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal>& throttleCommandResponseStates,
        ::physx::vehicle2::PxVehiclePhysXActor*& physxActor,
        ::physx::vehicle2::PxVehiclePhysXSteerState*& physxSteerStates,
        ::physx::vehicle2::PxVehiclePhysXConstraints*& physxConstraints,
        ::physx::vehicle2::PxVehicleRigidBodyState*& rigidBodyState,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates) override
    {
        axleDescription = &mAxleDescription;
        steerCommandResponseStates.setData(mSteerCommandResponseStates);
        throttleCommandResponseStates.setData(mThrottleCommandResponseStates);
        physxActor = &mPhysxActor;
        physxSteerStates = mPhysxSteerStates;
        physxConstraints = &mPhysxConstraints;
        rigidBodyState = &mRigidBodyState;
        wheelRigidBody1dStates.setData(mWheelRigidBody1dStates);
    }

    virtual void getDataForDirectDriveActuationStateComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal>& brakeResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal>& throttleResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleWheelActuationState>& actuationStates) override
    {
        axleDescription = &mAxleDescription;
        brakeResponseStates.setData(mBrakeCommandResponseStates);
        throttleResponseStates.setData(mThrottleCommandResponseStates);
        actuationStates.setData(mWheelActuationStates);
    }

    virtual void getDataForDirectDrivetrainComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal>& brakeResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal>& throttleResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelParams>& wheelParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelActuationState>& actuationStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireForce>& tireForces,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates) override
    {
        axleDescription = &mAxleDescription;
        brakeResponseStates.setData(mBrakeCommandResponseStates);
        throttleResponseStates.setData(mThrottleCommandResponseStates);
        wheelParams.setData(mWheelParams);
        actuationStates.setData(mWheelActuationStates);
        tireForces.setData(mTireForces);
        wheelRigidBody1dStates.setData(mWheelRigidBody1dStates);
    }

    virtual void getDataForPVDComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        const ::physx::vehicle2::PxVehicleRigidBodyParams*& rbodyParams,
        const ::physx::vehicle2::PxVehicleRigidBodyState*& rbodyState,
        const ::physx::vehicle2::PxVehicleSuspensionStateCalculationParams*& suspStateCalcParams,
        ::physx::vehicle2::PxVehicleSizedArrayData<const ::physx::vehicle2::PxVehicleBrakeCommandResponseParams>&
            brakeResponseParams,
        const ::physx::vehicle2::PxVehicleSteerCommandResponseParams*& steerResponseParams,
        const ::physx::vehicle2::PxVehicleAckermannParams*& ackermannParams,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& brakeResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& steerResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelParams>& wheelParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelActuationState>& wheelActuationStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelLocalPose>& wheelLocalPoses,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleRoadGeometryState>& roadGeomStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionParams>& suspParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionComplianceParams>& suspCompParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionForceParams>& suspForceParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionState>& suspStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionComplianceState>& suspCompStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionForce>& suspForces,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireForceParams>& tireForceParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireDirectionState>& tireDirectionStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireSpeedState>& tireSpeedStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireSlipState>& tireSlipStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireStickyState>& tireStickyStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireGripState>& tireGripStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireCamberAngleState>& tireCamberStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireForce>& tireForces,
        ::physx::vehicle2::PxVehicleSizedArrayData<const ::physx::vehicle2::PxVehicleAntiRollForceParams>& antiRollForceParams,
        const ::physx::vehicle2::PxVehicleAntiRollTorque*& antiRollTorque,
        const ::physx::vehicle2::PxVehicleCommandState*& commandState,
        const ::physx::vehicle2::PxVehicleDirectDriveThrottleCommandResponseParams*& directDriveThrottleResponseParams,
        const ::physx::vehicle2::PxVehicleDirectDriveTransmissionCommandState*& directDriveTransmissionState,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& directDrivethrottleResponseState,
        const ::physx::vehicle2::PxVehicleClutchCommandResponseParams*& clutchResponseParams,
        const ::physx::vehicle2::PxVehicleClutchParams*& clutchParams,
        const ::physx::vehicle2::PxVehicleEngineParams*& engineParams,
        const ::physx::vehicle2::PxVehicleGearboxParams*& gearboxParams,
        const ::physx::vehicle2::PxVehicleAutoboxParams*& autoboxParams,
        const ::physx::vehicle2::PxVehicleMultiWheelDriveDifferentialParams*& multiWheelDiffParams,
        const ::physx::vehicle2::PxVehicleFourWheelDriveDifferentialParams*& fourWheelDiffParams,
        const ::physx::vehicle2::PxVehicleTankDriveDifferentialParams*& tankDiffParams,
        const ::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState*& engineDriveTransmissionState,
        const ::physx::vehicle2::PxVehicleTankDriveTransmissionCommandState*& tankDriveTransmissionState,
        const ::physx::vehicle2::PxVehicleClutchCommandResponseState*& clutchResponseState,
        const ::physx::vehicle2::PxVehicleEngineDriveThrottleCommandResponseState*& engineDriveThrottleResponseState,
        const ::physx::vehicle2::PxVehicleEngineState*& engineState,
        const ::physx::vehicle2::PxVehicleGearboxState*& gearboxState,
        const ::physx::vehicle2::PxVehicleAutoboxState*& autoboxState,
        const ::physx::vehicle2::PxVehicleDifferentialState*& diffState,
        const ::physx::vehicle2::PxVehicleClutchSlipState*& clutchSlipState,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXSuspensionLimitConstraintParams>&
            physxConstraintParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams>&
            physxMaterialFrictionParams,
        const ::physx::vehicle2::PxVehiclePhysXActor*& physxActor,
        const ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryParams*& physxRoadGeomQryParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryState>& physxRoadGeomStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXConstraintState>& physxConstraintStates,
        const ::physx::vehicle2::PxVehiclePhysXSteerState*& physxSteerState,
        ::physx::vehicle2::PxVehiclePvdObjectHandles*& objectHandles) override
    {
        PhysXActorVehicleBase::getDataForPVDComponent(
            axleDescription, rbodyParams, rbodyState, suspStateCalcParams, brakeResponseParams, steerResponseParams,
            ackermannParams, brakeResponseStates, steerResponseStates, wheelParams, wheelActuationStates,
            wheelRigidBody1dStates, wheelLocalPoses, roadGeomStates, suspParams, suspCompParams, suspForceParams,
            suspStates, suspCompStates, suspForces, tireForceParams, tireDirectionStates, tireSpeedStates,
            tireSlipStates, tireStickyStates, tireGripStates, tireCamberStates, tireForces, antiRollForceParams,
            antiRollTorque, commandState, directDriveThrottleResponseParams, directDriveTransmissionState,
            directDrivethrottleResponseState, clutchResponseParams, clutchParams, engineParams, gearboxParams,
            autoboxParams, multiWheelDiffParams, fourWheelDiffParams, tankDiffParams, engineDriveTransmissionState,
            tankDriveTransmissionState, clutchResponseState, engineDriveThrottleResponseState, engineState, gearboxState,
            autoboxState, diffState, clutchSlipState, physxConstraintParams, physxMaterialFrictionParams, physxActor,
            physxRoadGeomQryParams, physxRoadGeomStates, physxConstraintStates, physxSteerState, objectHandles);

        directDrivethrottleResponseState.setData(mThrottleCommandResponseStates);
    }

public:
    static size_t computeDataSizeExcludingClass(const usdparser::VehicleDesc&,
                                                PhysXVehicleRawWheelControlDataMemoryOffsets&);
    void setDataPointers(uint8_t* memory, const PhysXVehicleRawWheelControlDataMemoryOffsets&);
    void setDataValues(
        const usdparser::VehicleDesc&,
        ::physx::PxRigidDynamic& vehicleActor,
        ::physx::PxPhysics& pxPhysics,
        const std::vector<::physx::PxTransform>& wheelShapeLocalPoses,
        const std::vector<::physx::PxShape*>& wheelShapeMapping,
        const std::vector<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>& tireMaterialFrictionTables,
        const std::vector<uint32_t>& wheelIndexToDataMap,
        const ::physx::vehicle2::PxVehicleFrame&,
        const float gravityMagnitude);
    void setComponentSequence();

    static PhysXVehicleRawWheelControl* create(
        ::physx::PxRigidDynamic& vehicleActor,
        ::physx::PxPhysics&,
        const usdparser::VehicleDesc&,
        const std::vector<::physx::PxTransform>& wheelShapeLocalPoses,
        const std::vector<::physx::PxShape*>& wheelShapeMapping,
        const std::vector<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>& tireMaterialFrictionTables,
        const std::vector<uint32_t>& wheelIndexToDataMap,
        const ::physx::vehicle2::PxVehicleFrame&,
        const float gravityMagnitude,
        ::physx::PxAllocatorCallback*);

    virtual void release(OmniPvdWriter*, ::physx::PxAllocatorCallback*) override;

    void simulateBegin(const float dt, const ::physx::vehicle2::PxVehicleSimulationContext&) override;

    virtual PhysXVehicleType::Enum getType() const override
    {
        return PhysXVehicleType::eRAW_WHEEL_CONTROL;
    }

    void setDriveTorque(const uint32_t wheelIndex, const float driveTorque);
    void setBrakeTorque(const uint32_t wheelIndex, const float brakeTorque);
    void setSteerAngle(const uint32_t wheelIndex, const float steerAngle);

    virtual void disableWheel(const uint32_t wheelIndex) override;


protected:
    ::physx::vehicle2::PxVehiclePhysXSteerState* mPhysxSteerStates;

    ::physx::PxReal* mThrottleCommandResponseStates;
};


struct PhysXVehicleManagedWheelControlMemoryOffsets : PhysXActorVehicleBaseDataMemoryOffsets
{
    size_t brakeCommandResponseParamsOffset;
    size_t ackermannParamsOffset;
    size_t wheelIndexListsOffset;
};

class PhysXVehicleManagedWheelControl : public PhysXActorVehicleBase,
                                        public ::physx::vehicle2::PxVehiclePhysXActorBeginComponent
{
protected:
    static constexpr uint32_t sBrakingSystemMaxCount = 2;
    static constexpr uint8_t sInvalidInternalBrakesIndex = 0xff;

    struct WheelIndexListType
    {
        enum Enum
        {
            eDRIVE, // list of wheels that are driven
            eBRAKES, // list of wheels that are assigned to braking systems. Since there can
                     // be multiple braking systems eBRAKES+index should be used.
            eSTEER = eBRAKES + sBrakingSystemMaxCount, // list of wheels that are steered
            eCOUNT
        };
    };

    PhysXVehicleManagedWheelControl() : PhysXActorVehicleBase()
    {
    }
    virtual ~PhysXVehicleManagedWheelControl()
    {
    }

    virtual void getDataForPhysXActorBeginComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        const ::physx::vehicle2::PxVehicleCommandState*& commands,
        const ::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState*& transmissionCommands,
        const ::physx::vehicle2::PxVehicleGearboxParams*& gearParams,
        const ::physx::vehicle2::PxVehicleGearboxState*& gearState,
        const ::physx::vehicle2::PxVehicleEngineParams*& engineParams,
        ::physx::vehicle2::PxVehiclePhysXActor*& physxActor,
        ::physx::vehicle2::PxVehiclePhysXSteerState*& physxSteerState,
        ::physx::vehicle2::PxVehiclePhysXConstraints*& physxConstraints,
        ::physx::vehicle2::PxVehicleRigidBodyState*& rigidBodyState,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
        ::physx::vehicle2::PxVehicleEngineState*& engineState) override
    {
        axleDescription = &mAxleDescription;
        commands = &mCommandState;
        physxActor = &mPhysxActor;
        physxSteerState = &mPhysxSteerState;
        physxConstraints = &mPhysxConstraints;
        rigidBodyState = &mRigidBodyState;
        wheelRigidBody1dStates.setData(mWheelRigidBody1dStates);

        transmissionCommands = nullptr;
        gearParams = nullptr;
        gearState = nullptr;
        engineParams = nullptr;
        engineState = nullptr;
    }

    virtual void getDataForPVDComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        const ::physx::vehicle2::PxVehicleRigidBodyParams*& rbodyParams,
        const ::physx::vehicle2::PxVehicleRigidBodyState*& rbodyState,
        const ::physx::vehicle2::PxVehicleSuspensionStateCalculationParams*& suspStateCalcParams,
        ::physx::vehicle2::PxVehicleSizedArrayData<const ::physx::vehicle2::PxVehicleBrakeCommandResponseParams>&
            brakeResponseParams,
        const ::physx::vehicle2::PxVehicleSteerCommandResponseParams*& steerResponseParams,
        const ::physx::vehicle2::PxVehicleAckermannParams*& ackermannParams,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& brakeResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& steerResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelParams>& wheelParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelActuationState>& wheelActuationStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelLocalPose>& wheelLocalPoses,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleRoadGeometryState>& roadGeomStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionParams>& suspParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionComplianceParams>& suspCompParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionForceParams>& suspForceParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionState>& suspStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionComplianceState>& suspCompStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionForce>& suspForces,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireForceParams>& tireForceParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireDirectionState>& tireDirectionStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireSpeedState>& tireSpeedStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireSlipState>& tireSlipStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireStickyState>& tireStickyStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireGripState>& tireGripStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireCamberAngleState>& tireCamberStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireForce>& tireForces,
        ::physx::vehicle2::PxVehicleSizedArrayData<const ::physx::vehicle2::PxVehicleAntiRollForceParams>& antiRollForceParams,
        const ::physx::vehicle2::PxVehicleAntiRollTorque*& antiRollTorque,
        const ::physx::vehicle2::PxVehicleCommandState*& commandState,
        const ::physx::vehicle2::PxVehicleDirectDriveThrottleCommandResponseParams*& directDriveThrottleResponseParams,
        const ::physx::vehicle2::PxVehicleDirectDriveTransmissionCommandState*& directDriveTransmissionState,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& directDrivethrottleResponseState,
        const ::physx::vehicle2::PxVehicleClutchCommandResponseParams*& clutchResponseParams,
        const ::physx::vehicle2::PxVehicleClutchParams*& clutchParams,
        const ::physx::vehicle2::PxVehicleEngineParams*& engineParams,
        const ::physx::vehicle2::PxVehicleGearboxParams*& gearboxParams,
        const ::physx::vehicle2::PxVehicleAutoboxParams*& autoboxParams,
        const ::physx::vehicle2::PxVehicleMultiWheelDriveDifferentialParams*& multiWheelDiffParams,
        const ::physx::vehicle2::PxVehicleFourWheelDriveDifferentialParams*& fourWheelDiffParams,
        const ::physx::vehicle2::PxVehicleTankDriveDifferentialParams*& tankDiffParams,
        const ::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState*& engineDriveTransmissionState,
        const ::physx::vehicle2::PxVehicleTankDriveTransmissionCommandState*& tankDriveTransmissionState,
        const ::physx::vehicle2::PxVehicleClutchCommandResponseState*& clutchResponseState,
        const ::physx::vehicle2::PxVehicleEngineDriveThrottleCommandResponseState*& engineDriveThrottleResponseState,
        const ::physx::vehicle2::PxVehicleEngineState*& engineState,
        const ::physx::vehicle2::PxVehicleGearboxState*& gearboxState,
        const ::physx::vehicle2::PxVehicleAutoboxState*& autoboxState,
        const ::physx::vehicle2::PxVehicleDifferentialState*& diffState,
        const ::physx::vehicle2::PxVehicleClutchSlipState*& clutchSlipState,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXSuspensionLimitConstraintParams>&
            physxConstraintParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams>&
            physxMaterialFrictionParams,
        const ::physx::vehicle2::PxVehiclePhysXActor*& physxActor,
        const ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryParams*& physxRoadGeomQryParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryState>& physxRoadGeomStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXConstraintState>& physxConstraintStates,
        const ::physx::vehicle2::PxVehiclePhysXSteerState*& physxSteerState,
        ::physx::vehicle2::PxVehiclePvdObjectHandles*& objectHandles) override
    {
        PhysXActorVehicleBase::getDataForPVDComponent(
            axleDescription, rbodyParams, rbodyState, suspStateCalcParams, brakeResponseParams, steerResponseParams,
            ackermannParams, brakeResponseStates, steerResponseStates, wheelParams, wheelActuationStates,
            wheelRigidBody1dStates, wheelLocalPoses, roadGeomStates, suspParams, suspCompParams, suspForceParams,
            suspStates, suspCompStates, suspForces, tireForceParams, tireDirectionStates, tireSpeedStates,
            tireSlipStates, tireStickyStates, tireGripStates, tireCamberStates, tireForces, antiRollForceParams,
            antiRollTorque, commandState, directDriveThrottleResponseParams, directDriveTransmissionState,
            directDrivethrottleResponseState, clutchResponseParams, clutchParams, engineParams, gearboxParams,
            autoboxParams, multiWheelDiffParams, fourWheelDiffParams, tankDiffParams, engineDriveTransmissionState,
            tankDriveTransmissionState, clutchResponseState, engineDriveThrottleResponseState, engineState, gearboxState,
            autoboxState, diffState, clutchSlipState, physxConstraintParams, physxMaterialFrictionParams, physxActor,
            physxRoadGeomQryParams, physxRoadGeomStates, physxConstraintStates, physxSteerState, objectHandles);

        brakeResponseParams.setDataAndCount(mBrakeCommandResponseParams, mCommandState.nbBrakes);
        steerResponseParams = &mSteerCommandResponseParams;
        ackermannParams = mAckermannParams;
        commandState = &mCommandState;
        physxSteerState = &mPhysxSteerState;
    }

protected:
    static size_t computeDataSizeExcludingClass(const usdparser::VehicleDesc&,
                                                PhysXVehicleManagedWheelControlMemoryOffsets&);
    void setDataPointers(uint8_t* memory, const PhysXVehicleManagedWheelControlMemoryOffsets&);
    void setDataValues(
        const usdparser::VehicleDesc&,
        ::physx::PxRigidDynamic& vehicleActor,
        ::physx::PxPhysics& pxPhysics,
        const std::vector<::physx::PxTransform>& wheelShapeLocalPoses,
        const std::vector<::physx::PxShape*>& wheelShapeMapping,
        const std::vector<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>& tireMaterialFrictionTables,
        const std::vector<uint32_t>& wheelIndexToDataMap,
        const ::physx::vehicle2::PxVehicleFrame&,
        const float gravityMagnitude);

    inline uint8_t* getWheelIndexListIncludingCountEntry(const uint32_t type) const
    {
        const uint32_t offset = type * (mWheelCapacity + 1);
        uint8_t* indices = mWheelIndexLists + offset;
        return indices;
    }

    inline uint8_t* getWheelIndexList(const uint32_t type, uint32_t& count) const
    {
        uint8_t* indices = getWheelIndexListIncludingCountEntry(type);
        count = *indices;
        return (indices + 1);
    }

public:
    void simulateBegin(const float dt, const ::physx::vehicle2::PxVehicleSimulationContext&) override;

    inline float getBrake(const uint32_t brakesIndex) const
    {
        CARB_ASSERT(brakesIndex < sBrakingSystemMaxCount);

        uint32_t internalBrakesIndex = mExternalToInternalBrakesIndex[brakesIndex];
        if (internalBrakesIndex != sInvalidInternalBrakesIndex)
            return mCommandState.brakes[internalBrakesIndex];
        else
            return 0.0f;
    }

    inline void setBrake(const uint32_t brakesIndex, const float value)
    {
        CARB_ASSERT(brakesIndex < sBrakingSystemMaxCount);

        uint32_t internalBrakesIndex = mExternalToInternalBrakesIndex[brakesIndex];
        if (internalBrakesIndex != sInvalidInternalBrakesIndex)
            mCommandState.brakes[internalBrakesIndex] = value;
    }

    inline float getThrottle() const
    {
        return mCommandState.throttle;
    }
    inline void setThrottle(const float value)
    {
        mCommandState.throttle = value;
    }

    inline float getSteer() const
    {
        return mCommandState.steer;
    }
    inline void setSteer(const float value)
    {
        mCommandState.steer = value;
    }

    inline void setMaxBrakeTorque(const uint32_t brakesIndex, const float maxBrakeTorque)
    {
        CARB_ASSERT(brakesIndex < sBrakingSystemMaxCount);

        uint32_t internalBrakesIndex = mExternalToInternalBrakesIndex[brakesIndex];
        if (internalBrakesIndex != sInvalidInternalBrakesIndex)
            mBrakeCommandResponseParams[internalBrakesIndex].maxResponse = maxBrakeTorque;
    }

    // deprecated
    inline void setWheelMaxBrakeTorque(const uint32_t wheelIndex, const uint32_t brakesIndex, const float maxBrakeTorque)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        CARB_ASSERT(brakesIndex < sBrakingSystemMaxCount);

        uint32_t internalBrakesIndex = mExternalToInternalBrakesIndex[brakesIndex];
        if (internalBrakesIndex != sInvalidInternalBrakesIndex)
            mBrakeCommandResponseParams[internalBrakesIndex].wheelResponseMultipliers[wheelIndex] = maxBrakeTorque;
    }

    inline bool setBrakeTorqueMultipliers(const uint32_t brakesIndex, const float* torqueMultipliers, const uint32_t count)
    {
        CARB_ASSERT(brakesIndex < sBrakingSystemMaxCount);

        uint32_t internalBrakesIndex = mExternalToInternalBrakesIndex[brakesIndex];
        if (internalBrakesIndex != sInvalidInternalBrakesIndex)
        {
            ::physx::vehicle2::PxVehicleBrakeCommandResponseParams& brakeCommandResponseParams =
                mBrakeCommandResponseParams[internalBrakesIndex];

            uint32_t currentCount;
            const uint8_t* indices = getWheelIndexList(WheelIndexListType::eBRAKES + internalBrakesIndex, currentCount);
            if (count == currentCount)
            {
                for (uint32_t i = 0; i < count; i++)
                {
                    const uint32_t index = indices[i];
                    brakeCommandResponseParams.wheelResponseMultipliers[index] = torqueMultipliers[i];
                }

                return true;
            }
            else
                return false;
        }
        else
            return false;
    }

    inline float getMaxSteerAngle() const
    {
        return mSteerCommandResponseParams.maxResponse;
    }

    inline void setMaxSteerAngle(const float maxSteerAngle)
    {
        mSteerCommandResponseParams.maxResponse = maxSteerAngle;
    }

    // deprecated
    inline void setWheelMaxSteerAngle(const uint32_t wheelIndex, const float maxSteerAngle)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mSteerCommandResponseParams.wheelResponseMultipliers[wheelIndex] = maxSteerAngle;
    }

    inline uint32_t getSteerAngleMultipliers(const uint8_t*& indices, const float*& angleMultipliers) const
    {
        uint32_t currentCount;
        indices = getWheelIndexList(WheelIndexListType::eSTEER, currentCount);
        angleMultipliers = mSteerCommandResponseParams.wheelResponseMultipliers;
        return currentCount;
    }

    inline bool setSteerAngleMultipliers(const float* angleMultipliers, const uint32_t count)
    {
        uint32_t currentCount;
        const uint8_t* indices = getWheelIndexList(WheelIndexListType::eSTEER, currentCount);
        if (count == currentCount)
        {
            for (uint32_t i = 0; i < count; i++)
            {
                const uint32_t index = indices[i];
                mSteerCommandResponseParams.wheelResponseMultipliers[index] = angleMultipliers[i];
            }

            return true;
        }
        else
            return false;
    }

    inline bool isUsingAckermannSteering() const
    {
        return (mAckermannParams != nullptr);
    }

    inline void setAckermannWheelBase(const float wheelBase)
    {
        CARB_ASSERT(mAckermannParams); // only call if the vehicle is using Ackermann correction
        mAckermannParams->wheelBase = wheelBase;
    }

    inline void setAckermannTrackWidth(const float trackWidth)
    {
        CARB_ASSERT(mAckermannParams); // only call if the vehicle is using Ackermann correction
        mAckermannParams->trackWidth = trackWidth;
    }

    inline void setAckermannStrength(const float strength)
    {
        CARB_ASSERT(mAckermannParams); // only call if the vehicle is using Ackermann correction
        mAckermannParams->strength = strength;
    }

    inline bool isUsingDeprecatedBrakesSetup() const
    {
        return mIsUsingDeprecatedBrakesSetup;
    }

    inline bool isUsingDeprecatedSteerSetup() const
    {
        return mIsUsingDeprecatedSteerSetup;
    }

    virtual void setWheelsToRestState() override;


protected:
    ::physx::vehicle2::PxVehiclePhysXSteerState mPhysxSteerState;

    ::physx::vehicle2::PxVehicleBrakeCommandResponseParams* mBrakeCommandResponseParams;
    ::physx::vehicle2::PxVehicleSteerCommandResponseParams mSteerCommandResponseParams;
    ::physx::vehicle2::PxVehicleAckermannParams* mAckermannParams; // if Ackermann correction is requested, else nullptr
    ::physx::vehicle2::PxVehicleCommandState mCommandState;

    // The following array will contain multiple lists of wheel index sets and data will be arranged as follows:
    // [validEntryCount, index{0} index{1}, index{validEntryCount-1}, ..., index{wheelCount-1}, validEntryCount,
    // index{0}, ...]
    //
    // example: 2 of 4 wheels are driven. The driven wheels have indices 0 and 1.
    // => [2, 0, 1, garbage, garbage, {start of next wheel index set}]
    //
    uint8_t* mWheelIndexLists;

    uint8_t mExternalToInternalBrakesIndex[sBrakingSystemMaxCount];

    bool mIsUsingDeprecatedBrakesSetup;
    bool mIsUsingDeprecatedSteerSetup;
};


struct PhysXVehicleDirectDriveDataMemoryOffsets : PhysXVehicleManagedWheelControlMemoryOffsets
{
    size_t throttleCommandResponseStatesOffset;
};

class PhysXVehicleDirectDrive : public PhysXVehicleManagedWheelControl,
                                public ::physx::vehicle2::PxVehicleDirectDriveCommandResponseComponent,
                                public ::physx::vehicle2::PxVehicleDirectDriveActuationStateComponent,
                                public ::physx::vehicle2::PxVehicleDirectDrivetrainComponent
{
public:
    PhysXVehicleDirectDrive() : PhysXVehicleManagedWheelControl()
    {
    }

protected:
    virtual ~PhysXVehicleDirectDrive()
    {
    }

    virtual void getDataForPhysXActorEndComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        const ::physx::vehicle2::PxVehicleRigidBodyState*& rigidBodyState,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelParams>& wheelParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxTransform>& wheelShapeLocalPoses,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelLocalPose>& wheelLocalPoses,
        const ::physx::vehicle2::PxVehicleGearboxState*& gearState,
        const ::physx::PxReal*& throttle,
        ::physx::vehicle2::PxVehiclePhysXActor*& physxActor) override
    {
        axleDescription = &mAxleDescription;
        rigidBodyState = &mRigidBodyState;
        wheelParams.setData(mWheelParams);
        wheelShapeLocalPoses.setData(mPhysxWheelShapeLocalPoses);
        wheelRigidBody1dStates.setData(mWheelRigidBody1dStates);
        wheelLocalPoses.setData(mWheelLocalPoses);
        physxActor = &mPhysxActor;

        gearState = nullptr;
        throttle = &mCommandState.throttle;
    }

    virtual void getDataForDirectDriveActuationStateComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal>& brakeResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal>& throttleResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleWheelActuationState>& actuationStates) override
    {
        axleDescription = &mAxleDescription;
        brakeResponseStates.setData(mBrakeCommandResponseStates);
        throttleResponseStates.setData(mThrottleCommandResponseStates);
        actuationStates.setData(mWheelActuationStates);
    }

    void getDataForDirectDriveCommandResponseComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        ::physx::vehicle2::PxVehicleSizedArrayData<const ::physx::vehicle2::PxVehicleBrakeCommandResponseParams>&
            brakeResponseParams,
        const ::physx::vehicle2::PxVehicleDirectDriveThrottleCommandResponseParams*& throttleResponseParams,
        const ::physx::vehicle2::PxVehicleSteerCommandResponseParams*& steerResponseParams,
        ::physx::vehicle2::PxVehicleSizedArrayData<const ::physx::vehicle2::PxVehicleAckermannParams>& ackermannParams,
        const ::physx::vehicle2::PxVehicleCommandState*& commands,
        const ::physx::vehicle2::PxVehicleDirectDriveTransmissionCommandState*& transmissionCommands,
        const ::physx::vehicle2::PxVehicleRigidBodyState*& rigidBodyState,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& brakeResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& throttleResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& steerResponseStates) override
    {
        axleDescription = &mAxleDescription;
        brakeResponseParams.setDataAndCount(mBrakeCommandResponseParams, mCommandState.nbBrakes);
        throttleResponseParams = &mThrottleCommandResponseParams;
        steerResponseParams = &mSteerCommandResponseParams;
        if (mAckermannParams)
            ackermannParams.setDataAndCount(mAckermannParams, 1);
        else
            ackermannParams.setEmpty();
        commands = &mCommandState;
        transmissionCommands = &mTransmissionCommandState;
        rigidBodyState = &mRigidBodyState;
        brakeResponseStates.setData(mBrakeCommandResponseStates);
        throttleResponseStates.setData(mThrottleCommandResponseStates);
        steerResponseStates.setData(mSteerCommandResponseStates);
    }

    virtual void getDataForDirectDrivetrainComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal>& brakeResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal>& throttleResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelParams>& wheelParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelActuationState>& actuationStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireForce>& tireForces,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates) override
    {
        axleDescription = &mAxleDescription;
        brakeResponseStates.setData(mBrakeCommandResponseStates);
        throttleResponseStates.setData(mThrottleCommandResponseStates);
        wheelParams.setData(mWheelParams);
        actuationStates.setData(mWheelActuationStates);
        tireForces.setData(mTireForces);
        wheelRigidBody1dStates.setData(mWheelRigidBody1dStates);
    }

    virtual void getDataForPVDComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        const ::physx::vehicle2::PxVehicleRigidBodyParams*& rbodyParams,
        const ::physx::vehicle2::PxVehicleRigidBodyState*& rbodyState,
        const ::physx::vehicle2::PxVehicleSuspensionStateCalculationParams*& suspStateCalcParams,
        ::physx::vehicle2::PxVehicleSizedArrayData<const ::physx::vehicle2::PxVehicleBrakeCommandResponseParams>&
            brakeResponseParams,
        const ::physx::vehicle2::PxVehicleSteerCommandResponseParams*& steerResponseParams,
        const ::physx::vehicle2::PxVehicleAckermannParams*& ackermannParams,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& brakeResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& steerResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelParams>& wheelParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelActuationState>& wheelActuationStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelLocalPose>& wheelLocalPoses,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleRoadGeometryState>& roadGeomStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionParams>& suspParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionComplianceParams>& suspCompParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionForceParams>& suspForceParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionState>& suspStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionComplianceState>& suspCompStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionForce>& suspForces,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireForceParams>& tireForceParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireDirectionState>& tireDirectionStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireSpeedState>& tireSpeedStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireSlipState>& tireSlipStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireStickyState>& tireStickyStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireGripState>& tireGripStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireCamberAngleState>& tireCamberStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireForce>& tireForces,
        ::physx::vehicle2::PxVehicleSizedArrayData<const ::physx::vehicle2::PxVehicleAntiRollForceParams>& antiRollForceParams,
        const ::physx::vehicle2::PxVehicleAntiRollTorque*& antiRollTorque,
        const ::physx::vehicle2::PxVehicleCommandState*& commandState,
        const ::physx::vehicle2::PxVehicleDirectDriveThrottleCommandResponseParams*& directDriveThrottleResponseParams,
        const ::physx::vehicle2::PxVehicleDirectDriveTransmissionCommandState*& directDriveTransmissionState,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& directDrivethrottleResponseState,
        const ::physx::vehicle2::PxVehicleClutchCommandResponseParams*& clutchResponseParams,
        const ::physx::vehicle2::PxVehicleClutchParams*& clutchParams,
        const ::physx::vehicle2::PxVehicleEngineParams*& engineParams,
        const ::physx::vehicle2::PxVehicleGearboxParams*& gearboxParams,
        const ::physx::vehicle2::PxVehicleAutoboxParams*& autoboxParams,
        const ::physx::vehicle2::PxVehicleMultiWheelDriveDifferentialParams*& multiWheelDiffParams,
        const ::physx::vehicle2::PxVehicleFourWheelDriveDifferentialParams*& fourWheelDiffParams,
        const ::physx::vehicle2::PxVehicleTankDriveDifferentialParams*& tankDiffParams,
        const ::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState*& engineDriveTransmissionState,
        const ::physx::vehicle2::PxVehicleTankDriveTransmissionCommandState*& tankDriveTransmissionState,
        const ::physx::vehicle2::PxVehicleClutchCommandResponseState*& clutchResponseState,
        const ::physx::vehicle2::PxVehicleEngineDriveThrottleCommandResponseState*& engineDriveThrottleResponseState,
        const ::physx::vehicle2::PxVehicleEngineState*& engineState,
        const ::physx::vehicle2::PxVehicleGearboxState*& gearboxState,
        const ::physx::vehicle2::PxVehicleAutoboxState*& autoboxState,
        const ::physx::vehicle2::PxVehicleDifferentialState*& diffState,
        const ::physx::vehicle2::PxVehicleClutchSlipState*& clutchSlipState,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXSuspensionLimitConstraintParams>&
            physxConstraintParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams>&
            physxMaterialFrictionParams,
        const ::physx::vehicle2::PxVehiclePhysXActor*& physxActor,
        const ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryParams*& physxRoadGeomQryParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryState>& physxRoadGeomStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXConstraintState>& physxConstraintStates,
        const ::physx::vehicle2::PxVehiclePhysXSteerState*& physxSteerState,
        ::physx::vehicle2::PxVehiclePvdObjectHandles*& objectHandles) override
    {
        PhysXVehicleManagedWheelControl::getDataForPVDComponent(
            axleDescription, rbodyParams, rbodyState, suspStateCalcParams, brakeResponseParams, steerResponseParams,
            ackermannParams, brakeResponseStates, steerResponseStates, wheelParams, wheelActuationStates,
            wheelRigidBody1dStates, wheelLocalPoses, roadGeomStates, suspParams, suspCompParams, suspForceParams,
            suspStates, suspCompStates, suspForces, tireForceParams, tireDirectionStates, tireSpeedStates,
            tireSlipStates, tireStickyStates, tireGripStates, tireCamberStates, tireForces, antiRollForceParams,
            antiRollTorque, commandState, directDriveThrottleResponseParams, directDriveTransmissionState,
            directDrivethrottleResponseState, clutchResponseParams, clutchParams, engineParams, gearboxParams,
            autoboxParams, multiWheelDiffParams, fourWheelDiffParams, tankDiffParams, engineDriveTransmissionState,
            tankDriveTransmissionState, clutchResponseState, engineDriveThrottleResponseState, engineState, gearboxState,
            autoboxState, diffState, clutchSlipState, physxConstraintParams, physxMaterialFrictionParams, physxActor,
            physxRoadGeomQryParams, physxRoadGeomStates, physxConstraintStates, physxSteerState, objectHandles);

        directDriveThrottleResponseParams = &mThrottleCommandResponseParams;
        directDriveTransmissionState = &mTransmissionCommandState;
        directDrivethrottleResponseState.setData(mThrottleCommandResponseStates);
    }

    inline void setThrottleCommandResponseStatesToDefault()
    {
        ::physx::PxMemZero(mThrottleCommandResponseStates, mWheelCapacity * sizeof(mThrottleCommandResponseStates[0]));
    }

public:
    static size_t computeDataSizeExcludingClass(const usdparser::VehicleDesc&, PhysXVehicleDirectDriveDataMemoryOffsets&);
    void setDataPointers(uint8_t* memory, const PhysXVehicleDirectDriveDataMemoryOffsets&);
    void setDataValues(
        const usdparser::VehicleDesc&,
        ::physx::PxRigidDynamic& vehicleActor,
        ::physx::PxPhysics& pxPhysics,
        const std::vector<::physx::PxTransform>& wheelShapeLocalPoses,
        const std::vector<::physx::PxShape*>& wheelShapeMapping,
        const std::vector<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>& tireMaterialFrictionTables,
        const std::vector<uint32_t>& wheelIndexToDataMap,
        const ::physx::vehicle2::PxVehicleFrame&,
        const float gravityMagnitude);
    void setComponentSequence();

    static PhysXVehicleDirectDrive* create(
        ::physx::PxRigidDynamic& vehicleActor,
        ::physx::PxPhysics&,
        const usdparser::VehicleDesc&,
        const std::vector<::physx::PxTransform>& wheelShapeLocalPoses,
        const std::vector<::physx::PxShape*>& wheelShapeMapping,
        const std::vector<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>& tireMaterialFrictionTables,
        const std::vector<uint32_t>& wheelIndexToDataMap,
        const ::physx::vehicle2::PxVehicleFrame&,
        const float gravityMagnitude,
        ::physx::PxAllocatorCallback*);

    virtual void release(OmniPvdWriter*, ::physx::PxAllocatorCallback*) override;

    virtual PhysXVehicleType::Enum getType() const override
    {
        return PhysXVehicleType::eDIRECT_DRIVE;
    }

    // 1: forward
    // 0: neutral
    // -1: reverse
    inline void setDriveMode(int mode)
    {
        if (mode > 0)
            mTransmissionCommandState.gear = ::physx::vehicle2::PxVehicleDirectDriveTransmissionCommandState::eFORWARD;
        else if (mode < 0)
            mTransmissionCommandState.gear = ::physx::vehicle2::PxVehicleDirectDriveTransmissionCommandState::eREVERSE;
        else
            mTransmissionCommandState.gear = ::physx::vehicle2::PxVehicleDirectDriveTransmissionCommandState::eNEUTRAL;
    }

    inline void setPeakDriveTorque(const float peakTorque)
    {
        mThrottleCommandResponseParams.maxResponse = peakTorque;
    }

    inline bool setDifferentialTorqueRatios(const float* torqueRatios, const uint32_t count)
    {
        uint32_t currentCount;
        const uint8_t* indices = getWheelIndexList(WheelIndexListType::eDRIVE, currentCount);
        if (count == currentCount)
        {
            for (uint32_t i = 0; i < count; i++)
            {
                const uint32_t index = indices[i];
                mThrottleCommandResponseParams.wheelResponseMultipliers[index] = torqueRatios[i];
            }

            return true;
        }
        else
            return false;
    }

    // deprecated (should only get called in combination with the deprecated "driven" attribute anymore)
    inline void setWheelToDriven(const uint32_t wheelIndex, const bool driven)
    {
        CARB_ASSERT(wheelIndex < mWheelCapacity);
        mThrottleCommandResponseParams.wheelResponseMultipliers[wheelIndex] = driven ? 1.0f : 0.0f;
    }

    virtual void disableWheel(const uint32_t wheelIndex) override;

    virtual void setWheelsToRestState() override;


protected:
    ::physx::vehicle2::PxVehicleDirectDriveTransmissionCommandState mTransmissionCommandState;

    ::physx::vehicle2::PxVehicleDirectDriveThrottleCommandResponseParams mThrottleCommandResponseParams;
    ::physx::PxReal* mThrottleCommandResponseStates;
};


struct PhysXVehicleEngineDriveDataMemoryOffsets : PhysXVehicleManagedWheelControlMemoryOffsets
{
    size_t transmissionCommandStateOffset;
    size_t differentialParamsOffset;
    size_t constraintGroupStateOffset;
    size_t autoboxOffset;
};

class PhysXVehicleEngineDrive : public PhysXVehicleManagedWheelControl,
                                public ::physx::vehicle2::PxVehicleEngineDriveCommandResponseComponent,
                                public ::physx::vehicle2::PxVehicleMultiWheelDriveDifferentialStateComponent,
                                public ::physx::vehicle2::PxVehicleTankDriveDifferentialStateComponent,
                                public ::physx::vehicle2::PxVehicleEngineDriveActuationStateComponent,
                                public ::physx::vehicle2::PxVehicleEngineDrivetrainComponent
{
private:
    enum
    {
        eNEUTRAL_GEAR = 1
    };

    struct DifferentialType
    {
        enum Enum
        {
            eMULTI_WHEEL,
            eTANK
        };
    };

    struct Autobox
    {
        ::physx::vehicle2::PxVehicleAutoboxParams mParams;
        ::physx::vehicle2::PxVehicleAutoboxState mState;
    };

public:
    PhysXVehicleEngineDrive() : PhysXVehicleManagedWheelControl()
    {
    }

protected:
    virtual ~PhysXVehicleEngineDrive()
    {
    }

    virtual void getDataForPhysXActorBeginComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        const ::physx::vehicle2::PxVehicleCommandState*& commands,
        const ::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState*& transmissionCommands,
        const ::physx::vehicle2::PxVehicleGearboxParams*& gearParams,
        const ::physx::vehicle2::PxVehicleGearboxState*& gearState,
        const ::physx::vehicle2::PxVehicleEngineParams*& engineParams,
        ::physx::vehicle2::PxVehiclePhysXActor*& physxActor,
        ::physx::vehicle2::PxVehiclePhysXSteerState*& physxSteerState,
        ::physx::vehicle2::PxVehiclePhysXConstraints*& physxConstraints,
        ::physx::vehicle2::PxVehicleRigidBodyState*& rigidBodyState,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
        ::physx::vehicle2::PxVehicleEngineState*& engineState) override
    {
        axleDescription = &mAxleDescription;
        commands = &mCommandState;
        physxActor = &mPhysxActor;
        physxSteerState = &mPhysxSteerState;
        physxConstraints = &mPhysxConstraints;
        rigidBodyState = &mRigidBodyState;
        wheelRigidBody1dStates.setData(mWheelRigidBody1dStates);

        transmissionCommands = mTransmissionCommandState;
        gearParams = &mGearboxParams;
        gearState = &mGearboxState;
        engineParams = &mEngineParams;
        engineState = &mEngineState;
    }

    virtual void getDataForPhysXActorEndComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        const ::physx::vehicle2::PxVehicleRigidBodyState*& rigidBodyState,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelParams>& wheelParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxTransform>& wheelShapeLocalPoses,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelLocalPose>& wheelLocalPoses,
        const ::physx::vehicle2::PxVehicleGearboxState*& gearState,
        const ::physx::PxReal*& throttle,
        ::physx::vehicle2::PxVehiclePhysXActor*& physxActor) override
    {
        axleDescription = &mAxleDescription;
        rigidBodyState = &mRigidBodyState;
        wheelParams.setData(mWheelParams);
        wheelShapeLocalPoses.setData(mPhysxWheelShapeLocalPoses);
        wheelRigidBody1dStates.setData(mWheelRigidBody1dStates);
        wheelLocalPoses.setData(mWheelLocalPoses);
        physxActor = &mPhysxActor;

        gearState = &mGearboxState;
        throttle = &mCommandState.throttle;
    }

    virtual void getDataForEngineDriveCommandResponseComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        ::physx::vehicle2::PxVehicleSizedArrayData<const ::physx::vehicle2::PxVehicleBrakeCommandResponseParams>&
            brakeResponseParams,
        const ::physx::vehicle2::PxVehicleSteerCommandResponseParams*& steerResponseParams,
        ::physx::vehicle2::PxVehicleSizedArrayData<const ::physx::vehicle2::PxVehicleAckermannParams>& ackermannParams,
        const ::physx::vehicle2::PxVehicleGearboxParams*& gearboxParams,
        const ::physx::vehicle2::PxVehicleClutchCommandResponseParams*& clutchResponseParams,
        const ::physx::vehicle2::PxVehicleEngineParams*& engineParams,
        const ::physx::vehicle2::PxVehicleRigidBodyState*& rigidBodyState,
        const ::physx::vehicle2::PxVehicleEngineState*& engineState,
        const ::physx::vehicle2::PxVehicleAutoboxParams*& autoboxParams,
        const ::physx::vehicle2::PxVehicleCommandState*& commands,
        const ::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState*& transmissionCommands,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& brakeResponseStates,
        ::physx::vehicle2::PxVehicleEngineDriveThrottleCommandResponseState*& throttleResponseState,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& steerResponseStates,
        ::physx::vehicle2::PxVehicleGearboxState*& gearboxResponseState,
        ::physx::vehicle2::PxVehicleClutchCommandResponseState*& clutchResponseState,
        ::physx::vehicle2::PxVehicleAutoboxState*& autoboxState) override
    {
        axleDescription = &mAxleDescription;
        brakeResponseParams.setDataAndCount(mBrakeCommandResponseParams, mCommandState.nbBrakes);
        steerResponseParams = &mSteerCommandResponseParams;
        if (mAckermannParams)
            ackermannParams.setDataAndCount(mAckermannParams, 1);
        else
            ackermannParams.setEmpty();
        gearboxParams = &mGearboxParams;
        clutchResponseParams = &mClutchCommandResponseParams;
        engineParams = mAutobox ? &mEngineParams : nullptr;
        rigidBodyState = &mRigidBodyState;
        engineState = mAutobox ? &mEngineState : nullptr;
        autoboxParams = mAutobox ? &mAutobox->mParams : nullptr;
        commands = &mCommandState;
        transmissionCommands = mTransmissionCommandState;
        brakeResponseStates.setData(mBrakeCommandResponseStates);
        throttleResponseState = &mThrottleCommandResponseState;
        steerResponseStates.setData(mSteerCommandResponseStates);
        gearboxResponseState = &mGearboxState;
        clutchResponseState = &mClutchCommandResponseState;
        autoboxState = mAutobox ? &mAutobox->mState : nullptr;
    }

    virtual void getDataForMultiWheelDriveDifferentialStateComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        const ::physx::vehicle2::PxVehicleMultiWheelDriveDifferentialParams*& differentialParams,
        ::physx::vehicle2::PxVehicleDifferentialState*& differentialState) override
    {
        CARB_ASSERT(mDifferentialType == DifferentialType::eMULTI_WHEEL);

        axleDescription = &mAxleDescription;
        differentialParams = mDifferentialParams;
        differentialState = &mDifferentialState;
    }

    virtual void getDataForTankDriveDifferentialStateComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        const ::physx::vehicle2::PxVehicleTankDriveTransmissionCommandState*& transmissionCommands,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelParams>& wheelParams,
        const ::physx::vehicle2::PxVehicleTankDriveDifferentialParams*& differentialParams,
        ::physx::vehicle2::PxVehicleDifferentialState*& differentialState,
        ::physx::vehicle2::PxVehicleWheelConstraintGroupState*& constraintGroupState) override
    {
        CARB_ASSERT(mDifferentialType == DifferentialType::eTANK);

        axleDescription = &mAxleDescription;
        transmissionCommands =
            static_cast<::physx::vehicle2::PxVehicleTankDriveTransmissionCommandState*>(mTransmissionCommandState);
        wheelParams.setData(mWheelParams);
        differentialParams = static_cast<::physx::vehicle2::PxVehicleTankDriveDifferentialParams*>(mDifferentialParams);
        differentialState = &mDifferentialState;
        constraintGroupState = mConstraintGroupState;
    }

    virtual void getDataForEngineDriveActuationStateComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        const ::physx::vehicle2::PxVehicleGearboxParams*& gearboxParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal>& brakeResponseStates,
        const ::physx::vehicle2::PxVehicleEngineDriveThrottleCommandResponseState*& throttleResponseState,
        const ::physx::vehicle2::PxVehicleGearboxState*& gearboxState,
        const ::physx::vehicle2::PxVehicleDifferentialState*& differentialState,
        const ::physx::vehicle2::PxVehicleClutchCommandResponseState*& clutchResponseState,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleWheelActuationState>& actuationStates) override
    {
        axleDescription = &mAxleDescription;
        gearboxParams = &mGearboxParams;
        brakeResponseStates.setData(mBrakeCommandResponseStates);
        throttleResponseState = &mThrottleCommandResponseState;
        gearboxState = &mGearboxState;
        differentialState = &mDifferentialState;
        clutchResponseState = &mClutchCommandResponseState;
        actuationStates.setData(mWheelActuationStates);
    }

    virtual void getDataForEngineDrivetrainComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelParams>& wheelParams,
        const ::physx::vehicle2::PxVehicleEngineParams*& engineParams,
        const ::physx::vehicle2::PxVehicleClutchParams*& clutchParams,
        const ::physx::vehicle2::PxVehicleGearboxParams*& gearboxParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal>& brakeResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelActuationState>& actuationStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireForce>& tireForces,
        const ::physx::vehicle2::PxVehicleEngineDriveThrottleCommandResponseState*& throttleResponseState,
        const ::physx::vehicle2::PxVehicleClutchCommandResponseState*& clutchResponseState,
        const ::physx::vehicle2::PxVehicleDifferentialState*& differentialState,
        const ::physx::vehicle2::PxVehicleWheelConstraintGroupState*& constraintGroupState,
        ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
        ::physx::vehicle2::PxVehicleEngineState*& engineState,
        ::physx::vehicle2::PxVehicleGearboxState*& gearboxState,
        ::physx::vehicle2::PxVehicleClutchSlipState*& clutchSlipState) override
    {
        axleDescription = &mAxleDescription;
        wheelParams.setData(mWheelParams);
        engineParams = &mEngineParams;
        clutchParams = &mClutchParams;
        gearboxParams = &mGearboxParams;
        brakeResponseStates.setData(mBrakeCommandResponseStates);
        actuationStates.setData(mWheelActuationStates);
        tireForces.setData(mTireForces);
        throttleResponseState = &mThrottleCommandResponseState;
        clutchResponseState = &mClutchCommandResponseState;
        differentialState = &mDifferentialState;
        constraintGroupState = mConstraintGroupState;
        wheelRigidBody1dStates.setData(mWheelRigidBody1dStates);
        engineState = &mEngineState;
        gearboxState = &mGearboxState;
        clutchSlipState = &mClutchSlipState;
    }

    virtual void getDataForPVDComponent(
        const ::physx::vehicle2::PxVehicleAxleDescription*& axleDescription,
        const ::physx::vehicle2::PxVehicleRigidBodyParams*& rbodyParams,
        const ::physx::vehicle2::PxVehicleRigidBodyState*& rbodyState,
        const ::physx::vehicle2::PxVehicleSuspensionStateCalculationParams*& suspStateCalcParams,
        ::physx::vehicle2::PxVehicleSizedArrayData<const ::physx::vehicle2::PxVehicleBrakeCommandResponseParams>&
            brakeResponseParams,
        const ::physx::vehicle2::PxVehicleSteerCommandResponseParams*& steerResponseParams,
        const ::physx::vehicle2::PxVehicleAckermannParams*& ackermannParams,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& brakeResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& steerResponseStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelParams>& wheelParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelActuationState>& wheelActuationStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleWheelLocalPose>& wheelLocalPoses,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleRoadGeometryState>& roadGeomStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionParams>& suspParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionComplianceParams>& suspCompParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionForceParams>& suspForceParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionState>& suspStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionComplianceState>& suspCompStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleSuspensionForce>& suspForces,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireForceParams>& tireForceParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireDirectionState>& tireDirectionStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireSpeedState>& tireSpeedStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireSlipState>& tireSlipStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireStickyState>& tireStickyStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireGripState>& tireGripStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireCamberAngleState>& tireCamberStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehicleTireForce>& tireForces,
        ::physx::vehicle2::PxVehicleSizedArrayData<const ::physx::vehicle2::PxVehicleAntiRollForceParams>& antiRollForceParams,
        const ::physx::vehicle2::PxVehicleAntiRollTorque*& antiRollTorque,
        const ::physx::vehicle2::PxVehicleCommandState*& commandState,
        const ::physx::vehicle2::PxVehicleDirectDriveThrottleCommandResponseParams*& directDriveThrottleResponseParams,
        const ::physx::vehicle2::PxVehicleDirectDriveTransmissionCommandState*& directDriveTransmissionState,
        ::physx::vehicle2::PxVehicleArrayData<::physx::PxReal>& directDrivethrottleResponseState,
        const ::physx::vehicle2::PxVehicleClutchCommandResponseParams*& clutchResponseParams,
        const ::physx::vehicle2::PxVehicleClutchParams*& clutchParams,
        const ::physx::vehicle2::PxVehicleEngineParams*& engineParams,
        const ::physx::vehicle2::PxVehicleGearboxParams*& gearboxParams,
        const ::physx::vehicle2::PxVehicleAutoboxParams*& autoboxParams,
        const ::physx::vehicle2::PxVehicleMultiWheelDriveDifferentialParams*& multiWheelDiffParams,
        const ::physx::vehicle2::PxVehicleFourWheelDriveDifferentialParams*& fourWheelDiffParams,
        const ::physx::vehicle2::PxVehicleTankDriveDifferentialParams*& tankDiffParams,
        const ::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState*& engineDriveTransmissionState,
        const ::physx::vehicle2::PxVehicleTankDriveTransmissionCommandState*& tankDriveTransmissionState,
        const ::physx::vehicle2::PxVehicleClutchCommandResponseState*& clutchResponseState,
        const ::physx::vehicle2::PxVehicleEngineDriveThrottleCommandResponseState*& engineDriveThrottleResponseState,
        const ::physx::vehicle2::PxVehicleEngineState*& engineState,
        const ::physx::vehicle2::PxVehicleGearboxState*& gearboxState,
        const ::physx::vehicle2::PxVehicleAutoboxState*& autoboxState,
        const ::physx::vehicle2::PxVehicleDifferentialState*& diffState,
        const ::physx::vehicle2::PxVehicleClutchSlipState*& clutchSlipState,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXSuspensionLimitConstraintParams>&
            physxConstraintParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams>&
            physxMaterialFrictionParams,
        const ::physx::vehicle2::PxVehiclePhysXActor*& physxActor,
        const ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryParams*& physxRoadGeomQryParams,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryState>& physxRoadGeomStates,
        ::physx::vehicle2::PxVehicleArrayData<const ::physx::vehicle2::PxVehiclePhysXConstraintState>& physxConstraintStates,
        const ::physx::vehicle2::PxVehiclePhysXSteerState*& physxSteerState,
        ::physx::vehicle2::PxVehiclePvdObjectHandles*& objectHandles) override
    {
        PhysXVehicleManagedWheelControl::getDataForPVDComponent(
            axleDescription, rbodyParams, rbodyState, suspStateCalcParams, brakeResponseParams, steerResponseParams,
            ackermannParams, brakeResponseStates, steerResponseStates, wheelParams, wheelActuationStates,
            wheelRigidBody1dStates, wheelLocalPoses, roadGeomStates, suspParams, suspCompParams, suspForceParams,
            suspStates, suspCompStates, suspForces, tireForceParams, tireDirectionStates, tireSpeedStates,
            tireSlipStates, tireStickyStates, tireGripStates, tireCamberStates, tireForces, antiRollForceParams,
            antiRollTorque, commandState, directDriveThrottleResponseParams, directDriveTransmissionState,
            directDrivethrottleResponseState, clutchResponseParams, clutchParams, engineParams, gearboxParams,
            autoboxParams, multiWheelDiffParams, fourWheelDiffParams, tankDiffParams, engineDriveTransmissionState,
            tankDriveTransmissionState, clutchResponseState, engineDriveThrottleResponseState, engineState, gearboxState,
            autoboxState, diffState, clutchSlipState, physxConstraintParams, physxMaterialFrictionParams, physxActor,
            physxRoadGeomQryParams, physxRoadGeomStates, physxConstraintStates, physxSteerState, objectHandles);

        clutchResponseParams = &mClutchCommandResponseParams;
        clutchParams = &mClutchParams;
        engineParams = &mEngineParams;
        gearboxParams = &mGearboxParams;
        autoboxParams = mAutobox ? &mAutobox->mParams : nullptr;
        if (mDifferentialType == DifferentialType::eMULTI_WHEEL)
        {
            multiWheelDiffParams = mDifferentialParams;
            engineDriveTransmissionState = mTransmissionCommandState;
        }
        else
        {
            CARB_ASSERT(mDifferentialType == DifferentialType::eTANK);
            tankDiffParams =
                static_cast<const ::physx::vehicle2::PxVehicleTankDriveDifferentialParams*>(mDifferentialParams);
            tankDriveTransmissionState =
                static_cast<const ::physx::vehicle2::PxVehicleTankDriveTransmissionCommandState*>(
                    mTransmissionCommandState);
        }
        clutchResponseState = &mClutchCommandResponseState;
        engineDriveThrottleResponseState = &mThrottleCommandResponseState;
        engineState = &mEngineState;
        gearboxState = &mGearboxState;
        autoboxState = mAutobox ? &mAutobox->mState : nullptr;
        diffState = &mDifferentialState;
        clutchSlipState = &mClutchSlipState;
    }

public:
    static size_t computeDataSizeExcludingClass(const usdparser::VehicleDesc&, PhysXVehicleEngineDriveDataMemoryOffsets&);
    void setDataPointers(uint8_t* memory, const PhysXVehicleEngineDriveDataMemoryOffsets&);
    void setDataValues(
        const usdparser::VehicleDesc&,
        ::physx::PxRigidDynamic& vehicleActor,
        ::physx::PxPhysics& pxPhysics,
        const std::vector<::physx::PxTransform>& wheelShapeLocalPoses,
        const std::vector<::physx::PxShape*>& wheelShapeMapping,
        const std::vector<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>& tireMaterialFrictionTables,
        const std::vector<uint32_t>& wheelIndexToDataMap,
        const ::physx::vehicle2::PxVehicleFrame&,
        const float gravityMagnitude);
    void setComponentSequence();

    static PhysXVehicleEngineDrive* create(
        ::physx::PxRigidDynamic& vehicleActor,
        ::physx::PxPhysics&,
        const usdparser::VehicleDesc&,
        const std::vector<::physx::PxTransform>& wheelShapeLocalPoses,
        const std::vector<::physx::PxShape*>& wheelShapeMapping,
        const std::vector<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>& tireMaterialFrictionTables,
        const std::vector<uint32_t>& wheelIndexToDataMap,
        const ::physx::vehicle2::PxVehicleFrame&,
        const float gravityMagnitude,
        ::physx::PxAllocatorCallback*);

    virtual void release(OmniPvdWriter*, ::physx::PxAllocatorCallback*) override;

    virtual PhysXVehicleType::Enum getType() const override
    {
        return PhysXVehicleType::eENGINE_DRIVE;
    }

    inline float getClutch() const
    {
        return mTransmissionCommandState->clutch;
    }

    inline bool getAutomaticTransmissionEnabled() const
    {
        if (mAutobox)
            return (mTransmissionCommandState->targetGear ==
                    ::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR);
        else
            return false;
    }

    inline void setTargetGearCommand(const uint32_t targetGear, const bool shiftImmediately)
    {
        mTransmissionCommandState->targetGear = targetGear;

        if ((targetGear != ::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR) &&
            shiftImmediately)
        {
            mGearboxState.currentGear = targetGear;
            mGearboxState.targetGear = targetGear;
            mGearboxState.gearSwitchTime = PX_VEHICLE_NO_GEAR_SWITCH_PENDING;
        }
    }

    inline uint32_t getTargetGear() const
    {
        return mGearboxState.targetGear;
    }
    inline uint32_t getCurrentGear() const
    {
        return mGearboxState.currentGear;
    }

    inline float getGearSwitchTime() const
    {
        return mGearboxState.gearSwitchTime;
    }

    inline float getAutoboxTimeSinceLastShift() const
    {
        return mAutobox ? mAutobox->mState.timeSinceLastShift : 0.0f;
    }

    inline void setEngineMoi(const float moi)
    {
        mEngineParams.moi = moi;
    }

    inline void setEnginePeakTorque(const float peakTorque)
    {
        mEngineParams.peakTorque = peakTorque;
    }

    inline void setEngineMaxRotationSpeed(const float maxRotationSpeed)
    {
        mEngineParams.maxOmega = maxRotationSpeed;
    }

    inline void setEngineIdleRotationSpeed(const float idleRotationSpeed)
    {
        mEngineParams.idleOmega = idleRotationSpeed;
    }

    inline float getEngineRotationSpeed() const
    {
        return mEngineState.rotationSpeed;
    }

    inline void setEngineDampingRateFullThrottle(const float dampingRateFullThrottle)
    {
        mEngineParams.dampingRateFullThrottle = dampingRateFullThrottle;
    }

    inline void setEngineDampingRateZeroThrottleClutchEngaged(const float dampingRateZeroThrottleClutchEngaged)
    {
        mEngineParams.dampingRateZeroThrottleClutchEngaged = dampingRateZeroThrottleClutchEngaged;
    }

    inline void setEngineDampingRateZeroThrottleClutchDisengaged(const float dampingRateZeroThrottleClutchDisengaged)
    {
        mEngineParams.dampingRateZeroThrottleClutchDisengaged = dampingRateZeroThrottleClutchDisengaged;
    }

    inline bool setDifferentialArrayValues(const float* values, const uint32_t count, float* target)
    {
        uint32_t currentCount;
        const uint8_t* indices = getWheelIndexList(WheelIndexListType::eDRIVE, currentCount);
        if (count == currentCount)
        {
            for (uint32_t i = 0; i < count; i++)
            {
                const uint32_t index = indices[i];
                target[index] = values[i];
            }

            return true;
        }
        else
            return false;
    }

    inline bool setDifferentialTorqueRatios(const float* torqueRatios, const uint32_t count)
    {
        return setDifferentialArrayValues(torqueRatios, count, mDifferentialParams->torqueRatios);
    }

    inline bool setDifferentialAverageWheelSpeedRatios(const float* avgWheelSpeedRatios, const uint32_t count)
    {
        return setDifferentialArrayValues(avgWheelSpeedRatios, count, mDifferentialParams->aveWheelSpeedRatios);
    }

    inline void setThrust(const unsigned int thrustIndex, const float thrust)
    {
        CARB_ASSERT(mDifferentialType == DifferentialType::eTANK);

        ::physx::vehicle2::PxVehicleTankDriveTransmissionCommandState* tankDriveTransmissionCommandState =
            static_cast<::physx::vehicle2::PxVehicleTankDriveTransmissionCommandState*>(mTransmissionCommandState);

        tankDriveTransmissionCommandState->thrusts[thrustIndex] = thrust;
    }

    // deprecated (should only get called in combination with the deprecated "driven" attribute anymore)
    void setWheelToDriven(const uint32_t wheelIndex, const bool driven);

    virtual void disableWheel(const uint32_t wheelIndex) override;

    virtual void setToRestState() override;


protected:
    ::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState* mTransmissionCommandState;

    ::physx::vehicle2::PxVehicleGearboxParams mGearboxParams;
    ::physx::vehicle2::PxVehicleGearboxState mGearboxState;
    Autobox* mAutobox;

    ::physx::vehicle2::PxVehicleClutchParams mClutchParams;
    ::physx::vehicle2::PxVehicleClutchCommandResponseParams mClutchCommandResponseParams;
    ::physx::vehicle2::PxVehicleClutchCommandResponseState mClutchCommandResponseState;
    ::physx::vehicle2::PxVehicleClutchSlipState mClutchSlipState;
    ::physx::vehicle2::PxVehicleEngineDriveThrottleCommandResponseState mThrottleCommandResponseState;

    ::physx::vehicle2::PxVehicleMultiWheelDriveDifferentialParams* mDifferentialParams;
    ::physx::vehicle2::PxVehicleDifferentialState mDifferentialState;
    ::physx::vehicle2::PxVehicleWheelConstraintGroupState* mConstraintGroupState; // for tanks only

    ::physx::vehicle2::PxVehicleEngineParams mEngineParams;
    ::physx::vehicle2::PxVehicleEngineState mEngineState;

    uint8_t mDifferentialType;
};


class VehicleGenerator
{
public:
    static PhysXActorVehicleBase* createVehicle(
        ::physx::PxRigidDynamic& vehicleActor,
        ::physx::PxPhysics& physics,
        const usdparser::VehicleDesc& vehicleDesc,
        const std::vector<::physx::PxTransform>& wheelShapeLocalPoses,
        const std::vector<::physx::PxShape*>& wheelShapeMapping,
        const std::vector<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>& tireMaterialFrictionTables,
        const std::vector<uint32_t>& wheelIndexToDataMap, // defines for each wheel which entries to pick
                                                          // in the parsed data and arrays like wheelShapeLocalPoses
                                                          // etc.
        const ::physx::vehicle2::PxVehicleFrame& frame,
        const float gravityMagnitude,
        ::physx::PxAllocatorCallback*);
};

} // namespace physx
} // namespace omni
