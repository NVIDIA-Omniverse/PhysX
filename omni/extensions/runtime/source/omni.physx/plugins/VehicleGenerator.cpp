// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"
#include "PhysXTools.h"
#include "VehicleGenerator.h"


using namespace physx;
using namespace omni::physx::usdparser;

namespace omni
{
namespace physx
{

static_assert(::physx::vehicle2::PxVehicleLimits::eMAX_NB_AXLES == ::physx::vehicle2::PxVehicleLimits::eMAX_NB_WHEELS, "");
// It should be possible to support cases where each wheel is controlled individually and thus
// having a wheel per axle for up to the max wheel count.

static_assert(VehicleDesc::maxNumberOfWheels == ::physx::vehicle2::PxVehicleLimits::eMAX_NB_WHEELS, "");
static_assert(EngineDesc::maxNumberOfTorqueCurvePoints == ::physx::vehicle2::PxVehicleEngineParams::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES, "");
static_assert(GearsDesc::maxNumberOfGears == ::physx::vehicle2::PxVehicleGearboxParams::eMAX_NB_GEARS, "");
static_assert(VehicleControllerDesc::automaticGearValue == ::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR, "");
static_assert(NonlinearCmdResponseDesc::maxNumberOfCommandValues == ::physx::vehicle2::PxVehicleCommandNonLinearResponseParams::eMAX_NB_COMMAND_VALUES, "");
static_assert(NonlinearCmdResponseDesc::maxNumberOfSpeedResponses == ::physx::vehicle2::PxVehicleCommandValueResponseTable::eMAX_NB_SPEED_RESPONSES, "");


static uint32_t getIndex(const ::physx::vehicle2::PxVehicleAxes::Enum axisEnum)
{
    // to ensure the index can be retrieved by doing a bit shift to the right (equivalent to division by 2)
    static_assert(::physx::vehicle2::PxVehicleAxes::ePosX == 0, "");
    static_assert(::physx::vehicle2::PxVehicleAxes::eNegX == 1, "");
    static_assert(::physx::vehicle2::PxVehicleAxes::ePosY == 2, "");
    static_assert(::physx::vehicle2::PxVehicleAxes::eNegY == 3, "");
    static_assert(::physx::vehicle2::PxVehicleAxes::ePosZ == 4, "");
    static_assert(::physx::vehicle2::PxVehicleAxes::eNegZ == 5, "");

    return (axisEnum >> 1);
}


PxQueryHitType::Enum VehicleQueryFilterCallback::preFilter(
    const PxFilterData& filterData0, const PxShape* shape, const PxRigidActor* actor, PxHitFlags&)
{
    // filterData0 is the vehicle suspension raycast.

    PxFilterData filterData1 = shape->getQueryFilterData();

    if (filterData0.word2 && filterData1.word2)
    {
        const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
        const CollisionGroupsPairsSet& colGroupsPairsSet = physxSetup.getCollisionGroupFilteredPairs();

        const PhysXScene* scene = physxSetup.getPhysXScene((size_t)actor->getScene()->userData);        

        const Pair<uint32_t> groupPair(filterData0.word2, filterData1.word2);
        CollisionGroupsPairsSet::const_iterator it = colGroupsPairsSet.find(groupPair);
        if (!scene->isInvertedCollisionGroupFilter())
        {
            if (it != colGroupsPairsSet.end())
                return PxQueryHitType::eNONE;
        }
        else
        {
            if (it == colGroupsPairsSet.end())
                return PxQueryHitType::eNONE;
        }
    }

    return PxQueryHitType::eBLOCK;
}

VehicleQueryFilterCallback gVehicleQueryFilterCallback;


#define IS_POWER_OF_TWO(value) ((value != 0) && ((value & (value - 1)) == 0))
#define ALIGNED_SIZE(size, alignment) (((size) + (alignment-1)) & (~(alignment-1)))  // note: alignment needs to be power of 2
#define IS_ALIGNED(size, alignment) ((size & (alignment-1)) == 0) // note: alignment needs to be power of 2

#define VEHICLE_MEM_ALIGNMENT 8  // not really needed but to avoid worrying about alignment of sub parts
static_assert(IS_POWER_OF_TWO(VEHICLE_MEM_ALIGNMENT), "");

#define ALIGNED_SIZE_VEHICLE(size) ALIGNED_SIZE(size, VEHICLE_MEM_ALIGNMENT)


size_t PhysXVehicleBase::computeDataSizeExcludingClass(const usdparser::VehicleDesc& vehicleDesc,
    PhysXVehicleBaseDataMemoryOffsets& memOffsets)
{
    const uint32_t wheelCount = static_cast<uint32_t>(vehicleDesc.wheelAttachments.size());

    size_t memSize = 0;

    memOffsets.brakeCommandResponseStatesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(PhysXVehicleBase::mBrakeCommandResponseStates[0]));

    memOffsets.steerCommandResponseStatesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(PhysXVehicleBase::mSteerCommandResponseStates[0]));

    memOffsets.suspensionParamsOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehicleSuspensionParams));

    memOffsets.suspensionStatesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehicleSuspensionState));

    memOffsets.suspensionComplianceParamsOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehicleSuspensionComplianceParams));

    memOffsets.suspensionComplianceStatesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehicleSuspensionComplianceState));

    memOffsets.suspensionForceParamsOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehicleSuspensionForceParams));

    memOffsets.suspensionForcesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehicleSuspensionForce));

    memOffsets.roadGeomStatesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehicleRoadGeometryState));

    memOffsets.tireSlipStatesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehicleTireSlipState));

    memOffsets.tireGripStatesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehicleTireGripState));

    memOffsets.tireDirectionStatesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehicleTireDirectionState));

    memOffsets.tireSpeedStatesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehicleTireSpeedState));

    memOffsets.tireCamberAngleStatesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehicleTireCamberAngleState));

    memOffsets.tireStickyStatesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehicleTireStickyState));

    memOffsets.tireForceParamsOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehicleTireForceParams));

    memOffsets.tireForcesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehicleTireForce));

    memOffsets.wheelParamsOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehicleWheelParams));

    memOffsets.wheelRigidBody1dStatesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehicleWheelRigidBody1dState));

    memOffsets.wheelLocalPosesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehicleWheelLocalPose));

    memOffsets.wheelActuationStatesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehicleWheelActuationState));

    memOffsets.suspensionLegacyParamsOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(SuspensionLegacyParams));

    return memSize;
}

void PhysXVehicleBase::setDataPointers(uint8_t* memory, const PhysXVehicleBaseDataMemoryOffsets& memOffsets)
{
    static_assert(sizeof(PhysXVehicleBase::mBrakeCommandResponseStates[0]) == sizeof(::physx::PxReal), "");
    mBrakeCommandResponseStates = reinterpret_cast<::physx::PxReal*>(memory + memOffsets.brakeCommandResponseStatesOffset);

    static_assert(sizeof(PhysXVehicleBase::mSteerCommandResponseStates[0]) == sizeof(::physx::PxReal), "");
    mSteerCommandResponseStates = reinterpret_cast<::physx::PxReal*>(memory + memOffsets.steerCommandResponseStatesOffset);

    mSuspensionParams = reinterpret_cast<::physx::vehicle2::PxVehicleSuspensionParams*>(memory + memOffsets.suspensionParamsOffset);
    mSuspensionStates = reinterpret_cast<::physx::vehicle2::PxVehicleSuspensionState*>(memory + memOffsets.suspensionStatesOffset);
    mSuspensionComplianceParams = reinterpret_cast<::physx::vehicle2::PxVehicleSuspensionComplianceParams*>(memory + memOffsets.suspensionComplianceParamsOffset);
    mSuspensionComplianceStates = reinterpret_cast<::physx::vehicle2::PxVehicleSuspensionComplianceState*>(memory + memOffsets.suspensionComplianceStatesOffset);
    mSuspensionForceParams = reinterpret_cast<::physx::vehicle2::PxVehicleSuspensionForceParams*>(memory + memOffsets.suspensionForceParamsOffset);
    mSuspensionForces = reinterpret_cast<::physx::vehicle2::PxVehicleSuspensionForce*>(memory + memOffsets.suspensionForcesOffset);
    mRoadGeomStates = reinterpret_cast<::physx::vehicle2::PxVehicleRoadGeometryState*>(memory + memOffsets.roadGeomStatesOffset);
    mTireSlipStates = reinterpret_cast<::physx::vehicle2::PxVehicleTireSlipState*>(memory + memOffsets.tireSlipStatesOffset);
    mTireGripStates = reinterpret_cast<::physx::vehicle2::PxVehicleTireGripState*>(memory + memOffsets.tireGripStatesOffset);
    mTireDirectionStates = reinterpret_cast<::physx::vehicle2::PxVehicleTireDirectionState*>(memory + memOffsets.tireDirectionStatesOffset);
    mTireSpeedStates = reinterpret_cast<::physx::vehicle2::PxVehicleTireSpeedState*>(memory + memOffsets.tireSpeedStatesOffset);
    mTireCamberAngleStates = reinterpret_cast<::physx::vehicle2::PxVehicleTireCamberAngleState*>(memory + memOffsets.tireCamberAngleStatesOffset);
    mTireStickyStates = reinterpret_cast<::physx::vehicle2::PxVehicleTireStickyState*>(memory + memOffsets.tireStickyStatesOffset);
    mTireForceParams = reinterpret_cast<::physx::vehicle2::PxVehicleTireForceParams*>(memory + memOffsets.tireForceParamsOffset);
    mTireForces = reinterpret_cast<::physx::vehicle2::PxVehicleTireForce*>(memory + memOffsets.tireForcesOffset);
    mWheelParams = reinterpret_cast<::physx::vehicle2::PxVehicleWheelParams*>(memory + memOffsets.wheelParamsOffset);
    mWheelRigidBody1dStates = reinterpret_cast<::physx::vehicle2::PxVehicleWheelRigidBody1dState*>(memory + memOffsets.wheelRigidBody1dStatesOffset);
	mWheelLocalPoses = reinterpret_cast<::physx::vehicle2::PxVehicleWheelLocalPose*>(memory + memOffsets.wheelLocalPosesOffset);
    mWheelActuationStates = reinterpret_cast<::physx::vehicle2::PxVehicleWheelActuationState*>(memory + memOffsets.wheelActuationStatesOffset);
    mSuspensionLegacyParams = reinterpret_cast<SuspensionLegacyParams*>(memory + memOffsets.suspensionLegacyParamsOffset);
}

static void setForceOffset(const ::physx::PxVec3& offset, const ::physx::PxVec3& refFrame,
    ::physx::vehicle2::PxVehicleFixedSizeLookupTable<::physx::PxVec3, 3>& table)
{
    const ::physx::PxVec3 localOffset = offset - refFrame;
    table.addPair(0.0f, localOffset);
}

static void takeScaleIntoAccount(const carb::Float3& scale, ::physx::PxVec3& vec3)
{
    // take scale into account as PhysX does not support scale
    vec3.x *= scale.x;
    vec3.y *= scale.y;
    vec3.z *= scale.z;
}

void PhysXVehicleBase::setDataValues(const usdparser::VehicleDesc& vehicleDesc,
    const std::vector<uint32_t>& wheelIndexToDataMap,
    const float vehicleMass, const ::physx::PxVec3& vehicleMassSpaceInertiaTensor, const ::physx::PxTransform& vehicleMassFrame,
    const ::physx::vehicle2::PxVehicleFrame& frame, const float lengthScale, const float gravityMagnitude)
{
    const uint32_t wheelCount = static_cast<uint32_t>(vehicleDesc.wheelAttachments.size());

    // this method should not get called if the number of wheels exceeds the limit
    CARB_ASSERT(wheelCount <= ::physx::vehicle2::PxVehicleLimits::eMAX_NB_WHEELS);
    CARB_ASSERT(wheelCount <= ::physx::vehicle2::PxVehicleLimits::eMAX_NB_AXLES);

    mWheelCapacity = wheelCount;

    const PxTransform vehicleMassFrameInv = vehicleMassFrame.getInverse();
    PxVec3 wheelAttPositionsAdjusted[vehicle2::PxVehicleLimits::eMAX_NB_WHEELS];
    // suspensionFramePosition or wheelCenterOfMassOffset transformed to body space and scale
    // taken into account

    for (PxU32 i = 0; i < wheelCount; i++)
    {
        const usdparser::WheelAttachmentDesc& wheelAttDesc = vehicleDesc.wheelAttachments[i];

        // note: vehicleMassFrame has scale taken into account already

        if (wheelAttDesc.state & usdparser::WheelAttachmentDesc::eHAS_SUSPENSION_FRAME)
        {
            PxVec3 scaledPos = toPhysX(wheelAttDesc.suspensionFramePosition);
            takeScaleIntoAccount(vehicleDesc.scale, scaledPos);

            if (vehicleDesc.referenceFrameIsCenterOfMass)
                wheelAttPositionsAdjusted[i] = scaledPos;  // deprecated code path
            else
                wheelAttPositionsAdjusted[i] = vehicleMassFrameInv.transform(scaledPos);
        }
        else  // deprecated code path
        {
            CARB_ASSERT(wheelAttDesc.state & usdparser::WheelAttachmentDesc::eHAS_WHEEL_COM_OFFSET);

            PxVec3 scaledPos = toPhysX(wheelAttDesc.wheelCenterOfMassOffset);
            takeScaleIntoAccount(vehicleDesc.scale, scaledPos);

            if (vehicleDesc.referenceFrameIsCenterOfMass)
                wheelAttPositionsAdjusted[i] = scaledPos;  // deprecated code path
            else
                wheelAttPositionsAdjusted[i] = vehicleMassFrameInv.transform(scaledPos);
        }
    }

    float sprungMassValues[vehicle2::PxVehicleLimits::eMAX_NB_WHEELS];
    const bool computeSprungMassValues = !vehicleDesc.hasUserDefinedSprungMassValues;
    if (computeSprungMassValues)
    {
        if (!::physx::vehicle2::PxVehicleComputeSprungMasses(wheelCount, wheelAttPositionsAdjusted, vehicleMass, frame.vrtAxis, sprungMassValues))
        {
            CARB_LOG_ERROR("PhysX Vehicle: sprung mass computation failed. Setup might be ill-conditioned. "
                "Make sure the center of mass is enclosed by wheel positions. Mass will be divided by number "
                "of wheels.\n");

            const PxReal dummyValue = vehicleMass / wheelCount;
            for (PxU32 i = 0; i < wheelCount; i++)
            {
                sprungMassValues[i] = dummyValue;
            }
        }
    }

    //---

    mAxleDescription.nbAxles = wheelCount;
    mAxleDescription.nbWheels = wheelCount;

    //---

    setBrakeAndSteerCommandResponseStatesToDefault();

    //---

    if (vehicleDesc.queryType == VehicleDesc::eRAYCAST)
        mSuspensionStateCalculationParams.suspensionJounceCalculationType = ::physx::vehicle2::PxVehicleSuspensionJounceCalculationType::eRAYCAST;
    else
        mSuspensionStateCalculationParams.suspensionJounceCalculationType = ::physx::vehicle2::PxVehicleSuspensionJounceCalculationType::eSWEEP;
    mSuspensionStateCalculationParams.limitSuspensionExpansionVelocity = vehicleDesc.limitSuspensionExpansionVelocity;

    //---

    for (uint32_t i = 0; i < wheelCount; i++)
    {
        const uint32_t dataIndex = wheelIndexToDataMap[i];
        CARB_ASSERT(dataIndex < wheelCount);

        const usdparser::WheelAttachmentDesc& wheelAttDesc = vehicleDesc.wheelAttachments[dataIndex];
        CARB_ASSERT(wheelAttDesc.suspension);
        const usdparser::SuspensionDesc& suspDesc = *wheelAttDesc.suspension;
        CARB_ASSERT(wheelAttDesc.tire);
        const usdparser::TireDesc& tireDesc = *wheelAttDesc.tire;
        CARB_ASSERT(wheelAttDesc.wheel);
        const usdparser::WheelDesc& wheelDesc = *wheelAttDesc.wheel;
        const usdparser::SuspensionComplianceDesc* suspComplianceDesc = wheelAttDesc.suspensionCompliance;

        //---

        mAxleDescription.nbWheelsPerAxle[i] = 1;
        mAxleDescription.axleToWheelIds[i] = i;
        mAxleDescription.wheelIdsInAxleOrder[i] = i;

        //---

        PxReal sprungMass;
        if (computeSprungMassValues)
            sprungMass = sprungMassValues[dataIndex];
        else
        {
            CARB_ASSERT(suspDesc.sprungMass > 0.0f);
            sprungMass = suspDesc.sprungMass;
        }

        ::physx::vehicle2::PxVehicleSuspensionParams& suspParam = mSuspensionParams[i];
        new(&suspParam)(::physx::vehicle2::PxVehicleSuspensionParams)();

        SuspensionLegacyParams& suspLegacyParams = mSuspensionLegacyParams[i];
        new(&suspLegacyParams)(SuspensionLegacyParams)();

        suspParam.suspensionTravelDir.x = wheelAttDesc.suspensionTravelDirection.x;
        suspParam.suspensionTravelDir.y = wheelAttDesc.suspensionTravelDirection.y;
        suspParam.suspensionTravelDir.z = wheelAttDesc.suspensionTravelDirection.z;
        if (!vehicleDesc.referenceFrameIsCenterOfMass)
        {
            suspParam.suspensionTravelDir = vehicleMassFrameInv.rotate(suspParam.suspensionTravelDir);
        }

        const float restLoadEstimate = sprungMass * gravityMagnitude;

        float maxDroop;
        if (suspDesc.travelDistance > 0.0f)
        {
            maxDroop = CARB_MIN((suspDesc.springStrength > 0.0f) ? (restLoadEstimate / suspDesc.springStrength) : suspDesc.travelDistance,
                suspDesc.travelDistance);

            suspLegacyParams.maxDroop = maxDroop;
            suspParam.suspensionTravelDist = suspDesc.travelDistance;
        }
        else  // deprecated code path
        {
            if (suspDesc.maxDroop >= 0.0f)
                maxDroop = suspDesc.maxDroop;
            else
            {
                maxDroop = (suspDesc.springStrength > 0.0f) ? (restLoadEstimate / suspDesc.springStrength) : 0.0f;
                // for zero spring strength, maxDroop would rather have to be infinity but this setup is
                // questionable and it's a deprecated code path anyway
            }

            suspLegacyParams.maxDroop = maxDroop;
            suspParam.suspensionTravelDist = suspDesc.maxCompression + maxDroop;
        }

        const PxVec3& wheelAttPosition = wheelAttPositionsAdjusted[dataIndex];
        PxVec3 suspAttPosition;
        if (wheelAttDesc.state & usdparser::WheelAttachmentDesc::eHAS_SUSPENSION_FRAME)
        {
            suspAttPosition = wheelAttPosition;
        }
        else  // deprecated code path
        {
            CARB_ASSERT(wheelAttDesc.state & usdparser::WheelAttachmentDesc::eHAS_WHEEL_COM_OFFSET);

            if (suspDesc.travelDistance > 0.0f)
                suspAttPosition = wheelAttPosition - (suspParam.suspensionTravelDir * (suspDesc.travelDistance - maxDroop));
            else  // deprecated code path
                suspAttPosition = wheelAttPosition - (suspParam.suspensionTravelDir * suspDesc.maxCompression);
        }
        PxQuat suspFrameOrient = toPhysXQuat(wheelAttDesc.suspensionFrameOrientation);
        if (!vehicleDesc.referenceFrameIsCenterOfMass)
        {
            suspFrameOrient = vehicleMassFrameInv.q * suspFrameOrient;
        }
        suspParam.suspensionAttachment = ::physx::PxTransform(suspAttPosition, suspFrameOrient);

        // note: the suspension frame does not define scale, thus the same scale as in the
        //       center-of-mass scale is used.
        suspParam.wheelAttachment = toPhysX(wheelAttDesc.wheelFramePosition, wheelAttDesc.wheelFrameOrientation);
        takeScaleIntoAccount(vehicleDesc.scale, suspParam.wheelAttachment.p);

        //---

        ::physx::vehicle2::PxVehicleSuspensionState& suspState = mSuspensionStates[i];
        new(&suspState)(::physx::vehicle2::PxVehicleSuspensionState)();
        suspState.setToDefault();

        //---

        ::physx::vehicle2::PxVehicleSuspensionComplianceParams& suspComplianceParams = mSuspensionComplianceParams[i];
        new(&suspComplianceParams)(::physx::vehicle2::PxVehicleSuspensionComplianceParams)();

        if (suspComplianceDesc)
        {
            // note: if no entry is defined, 0 will be used which is what we want
            //       for all our compliance terms

            if (suspComplianceDesc->wheelToeAngleList.size() > 0)
            {
                for (const carb::Float2& toeAngle : suspComplianceDesc->wheelToeAngleList)
                {
                    suspComplianceParams.wheelToeAngle.addPair(toeAngle.x, toeAngle.y);
                }
            }

            if (suspComplianceDesc->wheelCamberAngleList.size() > 0)
            {
                for (const carb::Float2& camberAngle : suspComplianceDesc->wheelCamberAngleList)
                {
                    suspComplianceParams.wheelCamberAngle.addPair(camberAngle.x, camberAngle.y);
                }
            }

            // note: the suspension frame does not define scale, thus the same scale as in the
            //       center-of-mass scale is used.

            if (suspComplianceDesc->suspensionForceAppPointList.size() > 0)
            {
                for (const carb::Float4& suspForceAppPoint : suspComplianceDesc->suspensionForceAppPointList)
                {
                    ::physx::PxVec3 appPoint(suspForceAppPoint.y, suspForceAppPoint.z, suspForceAppPoint.w);
                    takeScaleIntoAccount(vehicleDesc.scale, appPoint);

                    suspComplianceParams.suspForceAppPoint.addPair(suspForceAppPoint.x, appPoint);
                }
            }

            if (suspComplianceDesc->tireForceAppPointList.size() > 0)
            {
                for (const carb::Float4& tireForceAppPoint : suspComplianceDesc->tireForceAppPointList)
                {
                    ::physx::PxVec3 appPoint(tireForceAppPoint.y, tireForceAppPoint.z, tireForceAppPoint.w);
                    takeScaleIntoAccount(vehicleDesc.scale, appPoint);

                    suspComplianceParams.tireForceAppPoint.addPair(tireForceAppPoint.x, appPoint);
                }
            }
        }
        else
        {
            // deprecated toeAngle definition in wheel descriptor
            suspComplianceParams.wheelToeAngle.addPair(0, wheelDesc.toeAngle);

            const float restJounce = CARB_CLAMP(maxDroop / suspParam.suspensionTravelDist, FLT_EPSILON, 1.0f - FLT_EPSILON);

            // deprecated camberAtMaxDroop/camberAtRest/camberAtMaxCompression
            if ((suspDesc.camberAtRest != suspDesc.camberAtMaxCompression) || (suspDesc.camberAtRest != suspDesc.camberAtMaxDroop))
            {
                suspComplianceParams.wheelCamberAngle.addPair(0.0f, suspDesc.camberAtMaxDroop);
                suspComplianceParams.wheelCamberAngle.addPair(restJounce, suspDesc.camberAtRest);
                suspComplianceParams.wheelCamberAngle.addPair(1.0f, suspDesc.camberAtMaxCompression);
            }
            else
            {
                suspComplianceParams.wheelCamberAngle.addPair(restJounce, suspDesc.camberAtRest);
            }

            if (wheelAttDesc.state & usdparser::WheelAttachmentDesc::eHAS_SUSP_FORCE_APP_POINT)  // deprecated
            {
                ::physx::PxVec3 suspForceAppOffset = toPhysX(wheelAttDesc.suspensionForceAppPointOffset);
                takeScaleIntoAccount(vehicleDesc.scale, suspForceAppOffset);
                if (!vehicleDesc.referenceFrameIsCenterOfMass)
                    suspForceAppOffset = vehicleMassFrameInv.transform(suspForceAppOffset);

                setForceOffset(suspForceAppOffset, suspParam.suspensionAttachment.p,
                    suspComplianceParams.suspForceAppPoint);
            }
            else
            {
                // if the suspension force application point has not been defined, it will be set
                // to the estimated rest position of the wheel

                const ::physx::PxVec3 suspTravelDirLocal = suspParam.suspensionAttachment.q.rotateInv(suspParam.suspensionTravelDir);
                const ::physx::PxVec3 suspForceAppPoint = suspTravelDirLocal * (suspParam.suspensionTravelDist - maxDroop);
                
                suspComplianceParams.suspForceAppPoint.addPair(0, suspForceAppPoint);
            }

            if (wheelAttDesc.state & usdparser::WheelAttachmentDesc::eHAS_TIRE_FORCE_APP_POINT)  // deprecated
            {
                ::physx::PxVec3 tireForceAppOffset = toPhysX(wheelAttDesc.tireForceAppPointOffset);
                takeScaleIntoAccount(vehicleDesc.scale, tireForceAppOffset);
                if (!vehicleDesc.referenceFrameIsCenterOfMass)
                    tireForceAppOffset = vehicleMassFrameInv.transform(tireForceAppOffset);

                setForceOffset(tireForceAppOffset, suspParam.suspensionAttachment.p,
                    suspComplianceParams.tireForceAppPoint);
            }
            else
            {
                // if the tire force application point has not been defined, it will be set
                // to the estimated rest position of the wheel

                const ::physx::PxVec3 suspTravelDirLocal = suspParam.suspensionAttachment.q.rotateInv(suspParam.suspensionTravelDir);
                const ::physx::PxVec3 tireForceAppPoint = suspTravelDirLocal * (suspParam.suspensionTravelDist - maxDroop);
                
                suspComplianceParams.tireForceAppPoint.addPair(0, tireForceAppPoint);
            }
        }

        //---

        ::physx::vehicle2::PxVehicleSuspensionComplianceState& suspComplianceState = mSuspensionComplianceStates[i];
        new(&suspComplianceState)(::physx::vehicle2::PxVehicleSuspensionComplianceState)();
        suspComplianceState.setToDefault();

        //---

        ::physx::vehicle2::PxVehicleSuspensionForceParams& suspForceParams = mSuspensionForceParams[i];
        new(&suspForceParams)(::physx::vehicle2::PxVehicleSuspensionForceParams)();
        suspForceParams.stiffness = suspDesc.springStrength;
        suspForceParams.damping = suspDesc.springDamperRate;
        suspForceParams.sprungMass = sprungMass;

        //---

        ::physx::vehicle2::PxVehicleSuspensionForce& suspForces = mSuspensionForces[i];
        new(&suspForces)(::physx::vehicle2::PxVehicleSuspensionForce)();
        suspForces.setToDefault();

        //---

        ::physx::vehicle2::PxVehicleRoadGeometryState& roadGeomState = mRoadGeomStates[i];
        new(&roadGeomState)(::physx::vehicle2::PxVehicleRoadGeometryState)();
        roadGeomState.setToDefault();

        //---

        mTireSlipParams.minLatSlipDenominator = vehicleDesc.minLateralSlipDenominator;
        if (vehicleDesc.minPassiveLongitudinalSlipDenominator != 0.0f)
            mTireSlipParams.minPassiveLongSlipDenominator = vehicleDesc.minPassiveLongitudinalSlipDenominator;
        else
            mTireSlipParams.minPassiveLongSlipDenominator = vehicleDesc.minLongitudinalSlipDenominator;
        mTireSlipParams.minActiveLongSlipDenominator = vehicleDesc.minActiveLongitudinalSlipDenominator;

        //---

        ::physx::vehicle2::PxVehicleTireSlipState& tireSlipState = mTireSlipStates[i];
        new(&tireSlipState)(::physx::vehicle2::PxVehicleTireSlipState)();
        tireSlipState.setToDefault();
            
        //---

        ::physx::vehicle2::PxVehicleTireGripState& tireGripState = mTireGripStates[i];
        new(&tireGripState)(::physx::vehicle2::PxVehicleTireGripState)();
        tireGripState.setToDefault();
            
        //---

        ::physx::vehicle2::PxVehicleTireDirectionState& tireDirectionState = mTireDirectionStates[i];
        new(&tireDirectionState)(::physx::vehicle2::PxVehicleTireDirectionState)();
        tireDirectionState.setToDefault();
            
        //---

        ::physx::vehicle2::PxVehicleTireSpeedState& tireSpeedState = mTireSpeedStates[i];
        new(&tireSpeedState)(::physx::vehicle2::PxVehicleTireSpeedState)();
        tireSpeedState.setToDefault();
            
        //---

        ::physx::vehicle2::PxVehicleTireCamberAngleState& tireCamberAngleState = mTireCamberAngleStates[i];
        new(&tireCamberAngleState)(::physx::vehicle2::PxVehicleTireCamberAngleState)();
        tireCamberAngleState.setToDefault();
            
        //---

        ::physx::vehicle2::PxVehicleTireStickyState& tireStickyState = mTireStickyStates[i];
        new(&tireStickyState)(::physx::vehicle2::PxVehicleTireStickyState)();
        tireStickyState.setToDefault();
            
        //---

        ::physx::vehicle2::PxVehicleTireForceParams& tireForceParams = mTireForceParams[i];
        new(&tireForceParams)(::physx::vehicle2::PxVehicleTireForceParams)();
        if (tireDesc.restLoad > 0)
            tireForceParams.restLoad = tireDesc.restLoad;
        else
            tireForceParams.restLoad = restLoadEstimate;

        if (tireDesc.lateralStiffnessGraph.y != 0.0f)
        {
            tireForceParams.latStiffX = tireDesc.lateralStiffnessGraph.x;
            tireForceParams.latStiffY = tireDesc.lateralStiffnessGraph.y;
        }
        else  // deprecated attributes used
        {
            tireForceParams.latStiffX = tireDesc.latStiffX;
            tireForceParams.latStiffY = tireDesc.latStiffY * tireForceParams.restLoad;
        }

        if (tireDesc.longitudinalStiffness != 0.0f)
            tireForceParams.longStiff = tireDesc.longitudinalStiffness;
        else  // deprecated attribute used
            tireForceParams.longStiff = tireDesc.longitudinalStiffnessPerUnitGravity * gravityMagnitude;

        if (tireDesc.camberStiffness != -1.0f)
            tireForceParams.camberStiff = tireDesc.camberStiffness;
        else  // deprecated attribute used
            tireForceParams.camberStiff = tireDesc.camberStiffnessPerUnitGravity * gravityMagnitude;

        static_assert(sizeof(tireDesc.frictionVsSlipGraph) == 6 * sizeof(float), "");
        static_assert(sizeof(tireDesc.frictionVsSlipGraph) == sizeof(tireForceParams.frictionVsSlip), "");
        memcpy(tireForceParams.frictionVsSlip, tireDesc.frictionVsSlipGraph, 6 * sizeof(float));
        CARB_ASSERT(tireForceParams.frictionVsSlip[2][0] == tireDesc.frictionVsSlipGraph[2].x);
        CARB_ASSERT(tireForceParams.frictionVsSlip[2][1] == tireDesc.frictionVsSlipGraph[2].y);
        // using the previous default values in PhysX
        tireForceParams.loadFilter[0][0] = 0.0f;
        tireForceParams.loadFilter[0][1] = 0.2308f;
        tireForceParams.loadFilter[1][0] = 3.0f;
        tireForceParams.loadFilter[1][1] = 3.0f;

        //---

        ::physx::vehicle2::PxVehicleTireForce& tireForce = mTireForces[i];
        new(&tireForce)(::physx::vehicle2::PxVehicleTireForce)();
        tireForce.setToDefault();
            
        //---

        ::physx::vehicle2::PxVehicleWheelParams& wheelParams = mWheelParams[i];
        new(&wheelParams)(::physx::vehicle2::PxVehicleWheelParams)();
        wheelParams.radius = wheelDesc.radius;
        wheelParams.halfWidth = wheelDesc.width * 0.5f;
        wheelParams.mass = wheelDesc.mass;
        wheelParams.moi = wheelDesc.moi;
        wheelParams.dampingRate = wheelDesc.dampingRate;
            
        //---

        ::physx::vehicle2::PxVehicleWheelRigidBody1dState& wheelRigidBody1dState = mWheelRigidBody1dStates[i];
        new(&wheelRigidBody1dState)(::physx::vehicle2::PxVehicleWheelRigidBody1dState)();
        wheelRigidBody1dState.setToDefault();
            
        //---

        ::physx::vehicle2::PxVehicleWheelLocalPose& wheelLocalPose = mWheelLocalPoses[i];
        new(&wheelLocalPose)(::physx::vehicle2::PxVehicleWheelLocalPose)();
        wheelLocalPose.setToDefault();
            
        //---

        ::physx::vehicle2::PxVehicleWheelActuationState& wheelActuationState = mWheelActuationStates[i];
        new(&wheelActuationState)(::physx::vehicle2::PxVehicleWheelActuationState)();
        wheelActuationState.setToDefault();
    }

    //---

    mRigidBodyParams.mass = vehicleMass;
    mRigidBodyParams.moi = vehicleMassSpaceInertiaTensor;
            
    //---

    mRigidBodyState.setToDefault();

    //---

    mSubstepThresholdLongitudinalSpeed = vehicleDesc.subStepThresholdLongitudinalSpeed;
    mLowForwardSpeedSubstepCount = vehicleDesc.lowForwardSpeedSubStepCount;
    mHighForwardSpeedSubstepCount = vehicleDesc.highForwardSpeedSubStepCount;
    
    //---

    // Note that damping for sticky tire mode has changed compared to the old vehicle SDK. The USD
    // defaults are set to high damping for backwards compatibility and since it can help stability
    // for scenarios like jointed vehicles.

    mTireStickyParams.stickyParams[::physx::vehicle2::PxVehicleTireDirectionModes::eLONGITUDINAL].thresholdSpeed =
        vehicleDesc.longitudinalStickyTireThresholdSpeed;
    mTireStickyParams.stickyParams[::physx::vehicle2::PxVehicleTireDirectionModes::eLONGITUDINAL].thresholdTime =
        vehicleDesc.longitudinalStickyTireThresholdTime;
    mTireStickyParams.stickyParams[::physx::vehicle2::PxVehicleTireDirectionModes::eLONGITUDINAL].damping =
        vehicleDesc.longitudinalStickyTireDamping;

    mTireStickyParams.stickyParams[::physx::vehicle2::PxVehicleTireDirectionModes::eLATERAL].thresholdSpeed =
        vehicleDesc.lateralStickyTireThresholdSpeed;
    mTireStickyParams.stickyParams[::physx::vehicle2::PxVehicleTireDirectionModes::eLATERAL].thresholdTime =
        vehicleDesc.lateralStickyTireThresholdTime;
    mTireStickyParams.stickyParams[::physx::vehicle2::PxVehicleTireDirectionModes::eLATERAL].damping =
        vehicleDesc.lateralStickyTireDamping;
}

void PhysXVehicleBase::release(OmniPvdWriter* pvdWriter, ::physx::PxAllocatorCallback* allocator)
{
    if (mPvdObjectHandles)
    {
        CARB_ASSERT(pvdWriter);
        CARB_ASSERT(allocator);

        releasePvdObjectHandles(*pvdWriter, *allocator);
    }
}

void PhysXVehicleBase::simulate(const float dt, const ::physx::vehicle2::PxVehicleSimulationContext& context)
{
    if (mSubstepGroupId != ::physx::vehicle2::PxVehicleComponentSequence::eINVALID_SUBSTEP_GROUP)
    {
        float forwardVelMagn = ::physx::PxAbs(getForwardVelocity(context.frame));
        const uint32_t substepCount = (forwardVelMagn < mSubstepThresholdLongitudinalSpeed) ? mLowForwardSpeedSubstepCount :
            mHighForwardSpeedSubstepCount;
        mComponentSequence.setSubsteps(mSubstepGroupId, substepCount);
    }

    mComponentSequence.update(dt, context);
}

void PhysXVehicleBase::createPvdObjectHandles(::physx::PxAllocatorCallback& allocator,
    const uint32_t maxNbMaterialFrictionEntries)
{
    const uint32_t nbAntiRolls = 0;
    const OmniPvdContextHandle contextHandle = 1;  // hardcoded at the moment like in PhysX. Needs to be fetched from the PhysX scene
                                                   // as soon as PhysX makes proper use of contexts (see OM-83903)
    mPvdObjectHandles = ::physx::vehicle2::PxVehiclePvdObjectCreate(
        mWheelCapacity, nbAntiRolls, maxNbMaterialFrictionEntries, contextHandle, 
        allocator);
}

void PhysXVehicleBase::releasePvdObjectHandles(OmniPvdWriter& pvdWriter, ::physx::PxAllocatorCallback& allocator)
{
    ::physx::vehicle2::PxVehiclePvdObjectRelease(pvdWriter, allocator,
        *mPvdObjectHandles);
}

void PhysXVehicleBase::updateSprungMassProperties(const uint32_t wheelIndex,
    const float sprungMassValue, const float massCorrection,
    const bool updateMaxDroopValues, const bool updateRestLoad, const bool updateLatStiffY)
{
    mSuspensionForceParams[wheelIndex].sprungMass = sprungMassValue;

    if (updateRestLoad)
    {
        mTireForceParams[wheelIndex].restLoad *= massCorrection;
        if (updateLatStiffY)  // deprecated attribute latStiffY used
            mTireForceParams[wheelIndex].latStiffY *= massCorrection;
    }

    if (updateMaxDroopValues)
    {
        updateMaxDroop(wheelIndex, massCorrection);
    }
}

void PhysXVehicleBase::updateMassProperties(const float mass, const ::physx::PxVec3& massSpaceInertiaTensor,
    const ::physx::PxTransform* vehicleMassFrameChange, const ::physx::vehicle2::PxVehicleAxes::Enum upAxis,
    const bool updateSprungMassRelatedValues, const bool updateMaxDroopValues, const bool updateRestLoad,
    const bool updateLatStiffY)
{
    const float oldMass = mRigidBodyParams.mass;
    mRigidBodyParams.mass = mass;
    mRigidBodyParams.moi = massSpaceInertiaTensor;

    if (vehicleMassFrameChange && (!((*vehicleMassFrameChange) == PxTransform(PxIdentity))))
    {
        for (uint32_t i = 0; i < mWheelCapacity; i++)
        {
            mSuspensionParams[i].suspensionAttachment = (*vehicleMassFrameChange) * mSuspensionParams[i].suspensionAttachment;
            mSuspensionParams[i].suspensionTravelDir = vehicleMassFrameChange->rotate(mSuspensionParams[i].suspensionTravelDir);

            // note: deprecated suspensionForceAppPointOffset or tireForceAppPointOffset are stored
            //       in suspension frame local space, thus they do not need to get adjusted
        }

        if (updateSprungMassRelatedValues)
        {
            // note: since mRigidBodyParams.mass was adjusted above, the sprung mass computation in the method
            //       below will be based on the new mass

            updateSprungMassProperties(upAxis, updateMaxDroopValues, updateRestLoad, updateLatStiffY,
                false); // note: not really correct to just assume that the deprecated wheelCenterOfMassOffset is not used
                        //       but it really is not worth the trouble to track this for a deprecated feature and the rare
                        //       case of a center of mass frame change
        }
    }
    else if (updateSprungMassRelatedValues)
    {
        // note: only checking for updateMaxDroopValues/updateRestLoad if updateSprungMassRelatedValues
        //       is true since only a change in sprungMass should trigger a change in maxDroop/restLoad
        //       with the current auto-compute logic

        const float massCorrection = mass / oldMass;
        for (uint32_t i = 0; i < mWheelCapacity; i++)
        {
            updateSprungMassProperties(i, mSuspensionForceParams[i].sprungMass * massCorrection, massCorrection,
                updateMaxDroopValues, updateRestLoad, updateLatStiffY);
        }
    }
}

bool PhysXVehicleBase::updateSprungMassProperties(const ::physx::vehicle2::PxVehicleAxes::Enum upAxis,
    const bool updateMaxDroopValues, const bool updateRestLoad, const bool updateLatStiffY,
    const bool deprecatedWheelCenterOfMassChange)
{
    float sprungMassValues[vehicle2::PxVehicleLimits::eMAX_NB_WHEELS];
    PxVec3 sprungMassPositions[vehicle2::PxVehicleLimits::eMAX_NB_WHEELS];
    for (PxU32 i = 0; i < mWheelCapacity; i++)
    {
        if (deprecatedWheelCenterOfMassChange)
        {
            sprungMassPositions[i] = mSuspensionParams[i].suspensionAttachment.p +
                (mSuspensionParams[i].suspensionTravelDir * getSuspensionMaxCompression(i));
        }
        else
        {
            sprungMassPositions[i] = mSuspensionParams[i].suspensionAttachment.p;
        }
    }

    if (::physx::vehicle2::PxVehicleComputeSprungMasses(mWheelCapacity, sprungMassPositions,
        mRigidBodyParams.mass, upAxis, sprungMassValues))
    {
        for (PxU32 i = 0; i < mWheelCapacity; i++)
        {
            const float massCorrection = sprungMassValues[i] / mSuspensionForceParams[i].sprungMass;
            updateSprungMassProperties(i, sprungMassValues[i], massCorrection,
                updateMaxDroopValues, updateRestLoad, updateLatStiffY);
        }

        return true;
    }
    else
    {
        CARB_LOG_ERROR("PhysX Vehicle: sprung mass computation failed. Setup might be ill-conditioned. "
            "Make sure the center of mass is enclosed by wheel positions. Sprung mass properties will not "
            "get updated.\n");
    }

    return false;
}

void PhysXVehicleBase::setSuspensionForceAppPointOffset(const uint32_t wheelIndex, const ::physx::PxVec3& offset,
    const ::physx::PxVec3& scale, const ::physx::PxTransform* vehicleMassFrame)
{
    CARB_ASSERT(wheelIndex < mWheelCapacity);

    ::physx::vehicle2::PxVehicleSuspensionComplianceParams& suspComplianceParams = mSuspensionComplianceParams[wheelIndex];

    suspComplianceParams.suspForceAppPoint.clear();

    const PxVec3 offsetMod = PhysXVehicleBase::getScaledCoMFramePosition(offset, scale, vehicleMassFrame);

    setForceOffset(offsetMod, mSuspensionParams[wheelIndex].suspensionAttachment.p,
        suspComplianceParams.suspForceAppPoint);
}

void PhysXVehicleBase::setTireForceAppPointOffset(const uint32_t wheelIndex, const ::physx::PxVec3& offset,
    const ::physx::PxVec3& scale, const ::physx::PxTransform* vehicleMassFrame)
{
    CARB_ASSERT(wheelIndex < mWheelCapacity);

    ::physx::vehicle2::PxVehicleSuspensionComplianceParams& suspComplianceParams = mSuspensionComplianceParams[wheelIndex];

    suspComplianceParams.tireForceAppPoint.clear();

    const PxVec3 offsetMod = PhysXVehicleBase::getScaledCoMFramePosition(offset, scale, vehicleMassFrame);

    setForceOffset(offsetMod, mSuspensionParams[wheelIndex].suspensionAttachment.p,
        suspComplianceParams.tireForceAppPoint);
}

float PhysXVehicleBase::getForwardVelocity(const ::physx::vehicle2::PxVehicleFrame& frame) const
{
    const ::physx::PxVec3 forwardDir = mRigidBodyState.pose.q.rotate(frame.getLngAxis());
    const ::physx::PxReal forwardVelSignedMagn = forwardDir.dot(mRigidBodyState.linearVelocity);
    return forwardVelSignedMagn;
}

void PhysXVehicleBase::disableWheel(const uint32_t wheelIndex)
{
    setWheelRotationSpeed(wheelIndex, 0.0f);

    for (uint32_t i = 0; i < mAxleDescription.nbWheels; i++)
    {
        if (mAxleDescription.wheelIdsInAxleOrder[i] == wheelIndex)
        {
            const uint32_t newWheelCount = mAxleDescription.nbWheels - 1;
            for (uint32_t j = i; j < newWheelCount; j++)
            {
                // move all entries after the disabled wheel to keep a contiguous list of
                // enabled wheels
                mAxleDescription.wheelIdsInAxleOrder[j] = mAxleDescription.wheelIdsInAxleOrder[j + 1];
            }

            for (uint32_t j = 0; j < mAxleDescription.nbAxles; j++)
            {
                const uint32_t axleToWheelStartIndex = mAxleDescription.axleToWheelIds[j];
                const uint32_t wheelPerAxleCount = mAxleDescription.nbWheelsPerAxle[j];
                if (wheelPerAxleCount)
                {
                    if (axleToWheelStartIndex > i)
                    {
                        // all entries after the disabled wheel have been moved
                        mAxleDescription.axleToWheelIds[j] = axleToWheelStartIndex - 1;
                    }
                    else if ((axleToWheelStartIndex <= i) && ((axleToWheelStartIndex + wheelPerAxleCount) > i))
                    {
                        mAxleDescription.nbWheelsPerAxle[j] = wheelPerAxleCount - 1;
                    }
                }
            }

            mAxleDescription.nbWheels = newWheelCount;

            return;
        }
    }
}

void PhysXVehicleBase::setWheelsToRestState()
{
    // note that most of these are technically not needed as they get cleared
    // every step anyway. But this seems cleaner in case someone reads states
    // directly.

    // depending on the actual type of vehicle, these should not be touched
    //mBrakeCommandResponseStates
    //mSteerCommandResponseStates

    for (uint32_t i = 0; i < mWheelCapacity; i++)
    {
        mSuspensionStates[i].setToDefault();
        mSuspensionComplianceStates[i].setToDefault();
        mSuspensionForces[i].setToDefault();

        mRoadGeomStates[i].setToDefault();

        mTireSlipStates[i].setToDefault();
        mTireGripStates[i].setToDefault();
        mTireDirectionStates[i].setToDefault();
        mTireSpeedStates[i].setToDefault();
        mTireCamberAngleStates[i].setToDefault();
        mTireStickyStates[i].setToDefault();
        mTireForces[i].setToDefault();

        mWheelRigidBody1dStates[i].setToDefault();
        mWheelActuationStates[i].setToDefault();
    }
}

void PhysXVehicleBase::setToRestState()
{
    setWheelsToRestState();

    const ::physx::PxTransform pose = mRigidBodyState.pose;
    mRigidBodyState.setToDefault();
    mRigidBodyState.pose = pose;
}


template<typename TVehicle, typename TDataMemoryOffsets>
static TVehicle* createInternal(::physx::PxRigidDynamic& vehicleActor,
    ::physx::PxPhysics& pxPhysics,
    const usdparser::VehicleDesc& vehicleDesc,
    const std::vector<::physx::PxTransform>& wheelShapeLocalPoses,
    const std::vector<::physx::PxShape*>& wheelShapeMapping,
    const std::vector<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>& tireMaterialFrictionTables,
    const std::vector<uint32_t>& wheelIndexToDataMap,
    const ::physx::vehicle2::PxVehicleFrame& frame,
    const float gravityMagnitude,
    ::physx::PxAllocatorCallback* allocatorForPvd)
{
    const size_t classSize = ALIGNED_SIZE_VEHICLE(sizeof(TVehicle));
    size_t memSize = classSize;

    TDataMemoryOffsets memOffsets;
    memSize += TVehicle::computeDataSizeExcludingClass(vehicleDesc, memOffsets);

    uint8_t* mem = reinterpret_cast<uint8_t*>(ICE_ALLOC(memSize));

    // the allocator is expected to provide the desired memory alignment
    CARB_ASSERT(IS_ALIGNED(reinterpret_cast<size_t>(mem), VEHICLE_MEM_ALIGNMENT));

    if (mem)
    {
        TVehicle* vehicle = reinterpret_cast<TVehicle*>(mem);
        new(vehicle)(TVehicle)();

        mem += classSize;
        vehicle->setDataPointers(mem, memOffsets);
        vehicle->setDataValues(vehicleDesc, vehicleActor, pxPhysics,
            wheelShapeLocalPoses, wheelShapeMapping, tireMaterialFrictionTables,
            wheelIndexToDataMap, frame, gravityMagnitude);
        vehicle->setComponentSequence();

        if (allocatorForPvd)
        {
            vehicle->createPvdObjectHandles(*allocatorForPvd,
                static_cast<uint32_t>(tireMaterialFrictionTables.size()));
        }

        return vehicle;
    }
    else
        return nullptr;
}

static ::physx::PxConstraintShaderTable gVehicleConstraintTable =
{
	::physx::vehicle2::vehicleConstraintSolverPrep,
	::physx::vehicle2::visualiseVehicleConstraint,
	::physx::PxConstraintFlag::Enum(0)
};

void vehicleConstraintsCreate(const ::physx::vehicle2::PxVehicleAxleDescription& axleDescription,
    ::physx::PxPhysics& physics, ::physx::PxRigidBody& physxActor,
    ::physx::vehicle2::PxVehicleConstraintConnector* vehicleConstraintConnectors,
    ::physx::vehicle2::PxVehiclePhysXConstraints& vehicleConstraints)
{
	vehicleConstraints.setToDefault();

	//Each PxConstraint has a limit of 12 1d constraints.
	//Each wheel has longitudinal, lateral and suspension limit degrees of freedom.
	//This sums up to 3 dofs per wheel and 12 dofs per 4 wheels.
	//4 wheels therefore equals 1 PxConstraint 
	//Iterate over each block of 4 wheels and create a PxConstraints for each block of 4.
	uint32_t constraintIndex = 0;
	for(uint32_t i = 0; i < axleDescription.getNbWheels(); i+= ::physx::vehicle2::PxVehiclePhysXConstraintLimits::eNB_WHEELS_PER_PXCONSTRAINT)
	{
        // placement new as the constraint connectors are already part of the vehicle mem block
        ::physx::vehicle2::PxVehicleConstraintConnector* pxConnector = vehicleConstraintConnectors + constraintIndex;
        new(pxConnector)(::physx::vehicle2::PxVehicleConstraintConnector)(vehicleConstraints.constraintStates + i);

		::physx::PxConstraint* pxConstraint = physics.createConstraint(&physxActor, nullptr, *pxConnector, gVehicleConstraintTable,
            sizeof(::physx::vehicle2::PxVehiclePhysXConstraintState)*::physx::vehicle2::PxVehiclePhysXConstraintLimits::eNB_WHEELS_PER_PXCONSTRAINT);
		vehicleConstraints.constraints[constraintIndex] = pxConstraint;
		vehicleConstraints.constraintConnectors[constraintIndex] = pxConnector;
		constraintIndex++;
	}
}

void vehicleConstraintsDestroy(::physx::vehicle2::PxVehiclePhysXConstraints& vehicleConstraints)
{
    for (uint32_t i = 0; i < ::physx::vehicle2::PxVehiclePhysXConstraintLimits::eNB_CONSTRAINTS_PER_VEHICLE; i++)
    {
        if (vehicleConstraints.constraints[i])
        {
            vehicleConstraints.constraints[i]->release(); 
            vehicleConstraints.constraints[i] = nullptr;
            // no deletion of the constraint connectors as those are part of the vehicle mem block
            vehicleConstraints.constraintConnectors[i] = nullptr;
        }
    }
}

size_t PhysXActorVehicleBase::computeDataSizeExcludingClass(const usdparser::VehicleDesc& vehicleDesc,
    PhysXActorVehicleBaseDataMemoryOffsets& memOffsets)
{
    const uint32_t wheelCount = static_cast<uint32_t>(vehicleDesc.wheelAttachments.size());

    size_t memSize = PhysXVehicleBase::computeDataSizeExcludingClass(vehicleDesc, memOffsets);

    memOffsets.physxRoadGeomQueryFilterDatasOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::PxQueryFilterData));

    memOffsets.physxMaterialFrictionParamsOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*));

    memOffsets.physxRoadGeomQueryStatesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryState));

    memOffsets.physxSuspensionLimitConstraintParamsOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehiclePhysXSuspensionLimitConstraintParams));

    uint32_t connectorCount = wheelCount / ::physx::vehicle2::PxVehiclePhysXConstraintLimits::eNB_WHEELS_PER_PXCONSTRAINT;
    if ((connectorCount * ::physx::vehicle2::PxVehiclePhysXConstraintLimits::eNB_WHEELS_PER_PXCONSTRAINT) < wheelCount)
        connectorCount++;
    memOffsets.physxConstraintConnectorsOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(connectorCount * sizeof(::physx::vehicle2::PxVehicleConstraintConnector));

    memOffsets.physxWheelShapeLocalPoseOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::PxTransform));

    return memSize;
}

void PhysXActorVehicleBase::setDataPointers(uint8_t* memory, const PhysXActorVehicleBaseDataMemoryOffsets& memOffsets)
{
    PhysXVehicleBase::setDataPointers(memory, memOffsets);
    mPhysxRoadGeometryQueryParams.filterDataEntries = reinterpret_cast<::physx::PxQueryFilterData*>(memory + memOffsets.physxRoadGeomQueryFilterDatasOffset);
    mPhysxMaterialFrictionParams = reinterpret_cast<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams**>(memory + memOffsets.physxMaterialFrictionParamsOffset);
    mPhysxRoadGeometryQueryStates = reinterpret_cast<::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryState*>(memory + memOffsets.physxRoadGeomQueryStatesOffset);
    mPhysxSuspensionLimitConstraintParams = reinterpret_cast<::physx::vehicle2::PxVehiclePhysXSuspensionLimitConstraintParams*>(memory + memOffsets.physxSuspensionLimitConstraintParamsOffset);
    mPhysxConstraintConnectors = reinterpret_cast<::physx::vehicle2::PxVehicleConstraintConnector*>(memory + memOffsets.physxConstraintConnectorsOffset);
    mPhysxWheelShapeLocalPoses = reinterpret_cast<::physx::PxTransform*>(memory + memOffsets.physxWheelShapeLocalPoseOffset);
}

PhysXActorVehicleBase::~PhysXActorVehicleBase()
{
    vehicleConstraintsDestroy(mPhysxConstraints);
}

void PhysXActorVehicleBase::setDataValues(const usdparser::VehicleDesc& vehicleDesc,
    ::physx::PxRigidDynamic& vehicleActor, ::physx::PxPhysics& pxPhysics,
    const std::vector<::physx::PxTransform>& wheelShapeLocalPoses, const std::vector<::physx::PxShape*>& wheelShapeMapping,
    const std::vector<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>& tireMaterialFrictionTables,
    const std::vector<uint32_t>& wheelIndexToDataMap,
    const ::physx::vehicle2::PxVehicleFrame& frame, const float gravityMagnitude)
{
    CARB_ASSERT(vehicleActor.getActorFlags() & ::physx::PxActorFlag::eDISABLE_GRAVITY);

    const float vehicleMass = vehicleActor.getMass();
    const ::physx::PxVec3 vehicleMassSpaceInertiaTensor = vehicleActor.getMassSpaceInertiaTensor();
    const ::physx::PxTransform bodyToActor = vehicleActor.getCMassLocalPose();

    PhysXVehicleBase::setDataValues(vehicleDesc, wheelIndexToDataMap,
        vehicleMass, vehicleMassSpaceInertiaTensor, bodyToActor,
        frame, pxPhysics.getTolerancesScale().length, gravityMagnitude);

    //---

    if (vehicleDesc.queryType == VehicleDesc::eRAYCAST)
        mPhysxRoadGeometryQueryParams.roadGeometryQueryType = ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryType::eRAYCAST;
    else
        mPhysxRoadGeometryQueryParams.roadGeometryQueryType = ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryType::eSWEEP;

    mPhysxRoadGeometryQueryParams.filterCallback = &gVehicleQueryFilterCallback;

    //---

    mPhysxActor.setToDefault();
    mPhysxActor.rigidBody = &vehicleActor;
    // wheel shapes get set further below if available

    //--

    const uint32_t wheelCount = static_cast<uint32_t>(vehicleDesc.wheelAttachments.size());

    for (uint32_t i = 0; i < wheelCount; i++)
    {
        const uint32_t dataIndex = wheelIndexToDataMap[i];
        CARB_ASSERT(dataIndex < wheelCount);

        const usdparser::WheelAttachmentDesc& wheelAttDesc = vehicleDesc.wheelAttachments[dataIndex];

        const ObjectId collisionGroupId = wheelAttDesc.collisionGroupId;
        const uint32_t collisionGroup = internal::convertToCollisionGroup(collisionGroupId);
        PxFilterData fd;
        internal::convertCollisionGroupToPxFilterData(collisionGroup, fd);
        ::physx::PxQueryFilterData& queryFilterData = mPhysxRoadGeometryQueryParams.filterDataEntries[i];
        new(&queryFilterData)(::physx::PxQueryFilterData)();
        queryFilterData.data = fd;
        queryFilterData.flags = ::physx::PxQueryFlag::eSTATIC | ::physx::PxQueryFlag::eDYNAMIC |
            ::physx::PxQueryFlag::ePREFILTER | ::physx::PxQueryFlag::eDISABLE_HARDCODED_FILTER;

        //---

        CARB_ASSERT(dataIndex < tireMaterialFrictionTables.size());
        mPhysxMaterialFrictionParams[i] = tireMaterialFrictionTables[dataIndex];

        //---

        ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryState& physxRoadGeomQueryState = mPhysxRoadGeometryQueryStates[i];
        new(&physxRoadGeomQueryState)(::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryState)();
        physxRoadGeomQueryState.setToDefault();

        //---

        ::physx::vehicle2::PxVehiclePhysXSuspensionLimitConstraintParams& physxSuspensionLimitConstraintParams = mPhysxSuspensionLimitConstraintParams[i];
        new(&physxSuspensionLimitConstraintParams)(::physx::vehicle2::PxVehiclePhysXSuspensionLimitConstraintParams)();
        physxSuspensionLimitConstraintParams.restitution = 0.0f;
        physxSuspensionLimitConstraintParams.directionForSuspensionLimitConstraint =
            ::physx::vehicle2::PxVehiclePhysXSuspensionLimitConstraintParams::eROAD_GEOMETRY_NORMAL;

        //---

        new(mPhysxWheelShapeLocalPoses + i)(::physx::PxTransform)();
        mPhysxWheelShapeLocalPoses[i] = wheelShapeLocalPoses[dataIndex];

        //---

        mPhysxActor.wheelShapes[i] = wheelShapeMapping[dataIndex];
    }

    //---

    //mPhysxConstraints.setToDefault();  // is done as part of vehicleConstraintsCreate() below
    // note: the constructors of the mPhysxConstraintConnectors entries will be called in
    //       vehicleConstraintsCreate() below
    vehicleConstraintsCreate(mAxleDescription, pxPhysics, vehicleActor,
        mPhysxConstraintConnectors, mPhysxConstraints);
}

void PhysXActorVehicleBase::simulateEnd(const float dt, const ::physx::vehicle2::PxVehicleSimulationContext& context)
{
    ::physx::vehicle2::PxVehiclePhysXActorEndComponent* actorEndComp = static_cast<::physx::vehicle2::PxVehiclePhysXActorEndComponent*>(this);
    actorEndComp->update(dt, context);

    if (mPvdObjectHandles)
    {
        CARB_ASSERT(static_cast<const ::physx::vehicle2::PxVehiclePhysXSimulationContext&>(context).pvdContext.attributeHandles);
        CARB_ASSERT(static_cast<const ::physx::vehicle2::PxVehiclePhysXSimulationContext&>(context).pvdContext.writer);

        ::physx::vehicle2::PxVehiclePVDComponent* pvdComp = static_cast<::physx::vehicle2::PxVehiclePVDComponent*>(this);
        pvdComp->update(dt, context);
    }
}

void PhysXActorVehicleBase::removeWheelShape(const ::physx::PxShape* removedShape)
{
    for (uint32_t i = 0; i < mWheelCapacity; i++)
    {
        if (mPhysxActor.wheelShapes[i] == removedShape)
        {
            mPhysxActor.wheelShapes[i] = nullptr;
            return;
            // note: early abort as shared shapes do not make sense for vehicle wheels
            //       (local pose would be fixed)
        }
    }
}

float PhysXActorVehicleBase::getForwardVelocity(const ::physx::vehicle2::PxVehicleFrame& frame) const
{
    const ::physx::PxVec3 linVel = mPhysxActor.rigidBody->getLinearVelocity();
    const PxTransform body2World = mPhysxActor.rigidBody->getGlobalPose() * mPhysxActor.rigidBody->getCMassLocalPose();
    const ::physx::PxVec3 forwardDir = body2World.q.rotate(frame.getLngAxis());
    const ::physx::PxReal forwardVelSignedMagn = forwardDir.dot(linVel);
    return forwardVelSignedMagn;
}

void PhysXActorVehicleBase::disableWheel(const uint32_t wheelIndex)
{
    PhysXVehicleBase::disableWheel(wheelIndex);

    setWheelShape(wheelIndex, nullptr);
    mPhysxConstraints.constraintStates[wheelIndex].setToDefault();
}

void PhysXActorVehicleBase::setWheelsToRestState()
{
    PhysXVehicleBase::setWheelsToRestState();

    for (uint32_t i = 0; i < mWheelCapacity; i++)
    {
        mPhysxConstraints.constraintStates[i].setToDefault();
    }
}

void PhysXActorVehicleBase::setToRestState()
{
    PhysXVehicleBase::setToRestState();

    if (!(mPhysxActor.rigidBody->getRigidBodyFlags() & ::physx::PxRigidBodyFlag::eKINEMATIC))
    {
        ::physx::PxRigidDynamic* rigidDynamic = mPhysxActor.rigidBody->is<::physx::PxRigidDynamic>();
        CARB_ASSERT(rigidDynamic);  // no support for articulation links yet

        rigidDynamic->setLinearVelocity(::physx::PxVec3(0.0f));
        rigidDynamic->setAngularVelocity(::physx::PxVec3(0.0f));

        // Theoretical case where actor is not in a scene and constraints get solved via immediate mode
        if (mPhysxActor.rigidBody->getScene())
        {
            mPhysxActor.rigidBody->clearForce(PxForceMode::eACCELERATION);
            mPhysxActor.rigidBody->clearForce(PxForceMode::eVELOCITY_CHANGE);
            mPhysxActor.rigidBody->clearTorque(PxForceMode::eACCELERATION);
            mPhysxActor.rigidBody->clearTorque(PxForceMode::eVELOCITY_CHANGE);
        }
    }
}


bool PhysXVehicleRawWheelControlBeginComponent::update(const ::physx::PxReal dt,
    const ::physx::vehicle2::PxVehicleSimulationContext& context)
{
    CARB_UNUSED(dt);
    CARB_UNUSED(context);

    const ::physx::vehicle2::PxVehicleAxleDescription* axleDescription;
    ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal> steerCommandResponseStates;
    ::physx::vehicle2::PxVehicleArrayData<const ::physx::PxReal> throttleCommandResponseStates;
    ::physx::vehicle2::PxVehiclePhysXActor* physxActor;
    ::physx::vehicle2::PxVehiclePhysXSteerState* physxSteerStates;
    ::physx::vehicle2::PxVehiclePhysXConstraints* physxConstraints;
    ::physx::vehicle2::PxVehicleRigidBodyState* rigidBodyState;
    ::physx::vehicle2::PxVehicleArrayData<::physx::vehicle2::PxVehicleWheelRigidBody1dState> wheelRigidBody1dStates;

    getDataForPhysXVehicleRawWheelControlBeginComponent(axleDescription,
        steerCommandResponseStates, throttleCommandResponseStates,
        physxActor, physxSteerStates, physxConstraints,
        rigidBodyState, wheelRigidBody1dStates);

    CARB_ASSERT(physxActor->rigidBody);
    if (physxActor->rigidBody->getScene())
    {
        ::physx::PxRigidDynamic* rigidDynamic = physxActor->rigidBody->is<::physx::PxRigidDynamic>();
        CARB_ASSERT(rigidDynamic);  // no support for articulation links yet
        bool isSleeping = rigidDynamic->isSleeping();
        if (isSleeping)
        {
            for (uint32_t i = 0; i < axleDescription->nbWheels; i++)
            {
                const uint32_t wheelIndex = axleDescription->wheelIdsInAxleOrder[i];

                ::physx::vehicle2::PxVehiclePhysXSteerState& steerState = physxSteerStates[wheelIndex];
                const PxReal steerAngle = steerCommandResponseStates[wheelIndex];

                if (isSleeping)
                {
                    if ((throttleCommandResponseStates[wheelIndex] != 0.0f) ||
                        ((steerState.previousSteerCommand != PX_VEHICLE_UNSPECIFIED_STEER_STATE) && (steerAngle != steerState.previousSteerCommand)))
                    {
                        rigidDynamic->wakeUp();
                        isSleeping = false;
                    }
                }

                steerState.previousSteerCommand = steerAngle;
            }
        }

        if (isSleeping)
        {
            ::physx::vehicle2::PxVehiclePhysxActorSleepCheck(*axleDescription, *physxActor->rigidBody, NULL,
                *rigidBodyState, *physxConstraints, wheelRigidBody1dStates, NULL);
            return false;
        }
    }

    ::physx::vehicle2::PxVehicleReadRigidBodyStateFromPhysXActor(*physxActor->rigidBody, *rigidBodyState);

    return true;
}

size_t PhysXVehicleRawWheelControl::computeDataSizeExcludingClass(const usdparser::VehicleDesc& vehicleDesc,
    PhysXVehicleRawWheelControlDataMemoryOffsets& memOffsets)
{
    const uint32_t wheelCount = static_cast<uint32_t>(vehicleDesc.wheelAttachments.size());

    size_t memSize = PhysXActorVehicleBase::computeDataSizeExcludingClass(vehicleDesc, memOffsets);

    memOffsets.physxSteerStatesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(::physx::vehicle2::PxVehiclePhysXSteerState));

    memOffsets.throttleCommandResponseStatesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(PhysXVehicleRawWheelControl::mThrottleCommandResponseStates[0]));

    return memSize;
}

void PhysXVehicleRawWheelControl::setDataPointers(uint8_t* memory,
    const PhysXVehicleRawWheelControlDataMemoryOffsets& memOffsets)
{
    PhysXActorVehicleBase::setDataPointers(memory, memOffsets);

    mPhysxSteerStates = reinterpret_cast<::physx::vehicle2::PxVehiclePhysXSteerState*>(memory + memOffsets.physxSteerStatesOffset);

    static_assert(sizeof(PhysXVehicleRawWheelControl::mThrottleCommandResponseStates[0]) == sizeof(::physx::PxReal), "");
    mThrottleCommandResponseStates = reinterpret_cast<::physx::PxReal*>(memory + memOffsets.throttleCommandResponseStatesOffset);
}

void PhysXVehicleRawWheelControl::setDataValues(const usdparser::VehicleDesc& vehicleDesc,
    ::physx::PxRigidDynamic& vehicleActor, ::physx::PxPhysics& pxPhysics,
    const std::vector<::physx::PxTransform>& wheelShapeLocalPoses, const std::vector<::physx::PxShape*>& wheelShapeMapping,
    const std::vector<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>& tireMaterialFrictionTables,
    const std::vector<uint32_t>& wheelIndexToDataMap,
    const ::physx::vehicle2::PxVehicleFrame& frame, const float gravityMagnitude)
{
    PhysXActorVehicleBase::setDataValues(vehicleDesc, vehicleActor, pxPhysics,
        wheelShapeLocalPoses, wheelShapeMapping, tireMaterialFrictionTables,
        wheelIndexToDataMap, frame, gravityMagnitude);

    //---

    const uint32_t wheelCount = static_cast<uint32_t>(vehicleDesc.wheelAttachments.size());
    for (uint32_t i = 0; i < wheelCount; i++)
    {
        const uint32_t dataIndex = wheelIndexToDataMap[i];
        CARB_ASSERT(dataIndex < wheelCount);

        const usdparser::WheelAttachmentDesc& wheelAttDesc = vehicleDesc.wheelAttachments[dataIndex];

        ::physx::vehicle2::PxVehiclePhysXSteerState& physxSteerState = mPhysxSteerStates[i];
        new(&physxSteerState)(::physx::vehicle2::PxVehiclePhysXSteerState)();
        physxSteerState.setToDefault();

        mThrottleCommandResponseStates[i] = 0.0f;
    }
}

void PhysXVehicleRawWheelControl::setComponentSequence()
{
    bool success;
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleDirectDriveActuationStateComponent*>(this));
    CARB_ASSERT(success);
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehiclePhysXRoadGeometrySceneQueryComponent*>(this));
    CARB_ASSERT(success);

    mSubstepGroupId = mComponentSequence.beginSubstepGroup();
    CARB_ASSERT(mSubstepGroupId != ::physx::vehicle2::PxVehicleComponentSequence::eINVALID_SUBSTEP_GROUP);
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleSuspensionComponent*>(this));
    CARB_ASSERT(success);
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleTireComponent*>(this));
    CARB_ASSERT(success);
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehiclePhysXConstraintComponent*>(this));
    CARB_ASSERT(success);
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleDirectDrivetrainComponent*>(this));
    CARB_ASSERT(success);
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleRigidBodyComponent*>(this));
    CARB_ASSERT(success);
    mComponentSequence.endSubstepGroup();

    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleWheelComponent*>(this));
    CARB_ASSERT(success);
}

PhysXVehicleRawWheelControl* PhysXVehicleRawWheelControl::create(::physx::PxRigidDynamic& vehicleActor,
                                                                 ::physx::PxPhysics& pxPhysics,
                                                                 const usdparser::VehicleDesc& vehicleDesc,
                                                                 const std::vector<::physx::PxTransform>& wheelShapeLocalPoses,
                                                                 const std::vector<::physx::PxShape*>& wheelShapeMapping,
                                                                 const std::vector<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>& tireMaterialFrictionTables,
                                                                 const std::vector<uint32_t>& wheelIndexToDataMap,
                                                                 const ::physx::vehicle2::PxVehicleFrame& frame,
                                                                 const float gravityMagnitude,
                                                                 ::physx::PxAllocatorCallback* allocatorForPvd)
{
    return createInternal<PhysXVehicleRawWheelControl, PhysXVehicleRawWheelControlDataMemoryOffsets>(
        vehicleActor, pxPhysics, vehicleDesc, wheelShapeLocalPoses, wheelShapeMapping,
        tireMaterialFrictionTables, wheelIndexToDataMap, frame, gravityMagnitude,
        allocatorForPvd);
}

void PhysXVehicleRawWheelControl::release(OmniPvdWriter* pvdWriter, ::physx::PxAllocatorCallback* allocator)
{
    PhysXActorVehicleBase::release(pvdWriter, allocator);

    ICE_PLACEMENT_DELETE(this, PhysXVehicleRawWheelControl);
}

void PhysXVehicleRawWheelControl::simulateBegin(const float dt, const ::physx::vehicle2::PxVehicleSimulationContext& context)
{
    PhysXVehicleRawWheelControlBeginComponent* actorBeginComp = static_cast<PhysXVehicleRawWheelControlBeginComponent*>(this);
    actorBeginComp->update(dt, context);
}

void PhysXVehicleRawWheelControl::setDriveTorque(const uint32_t wheelIndex, const float driveTorque)
{
    CARB_ASSERT(wheelIndex < mWheelCapacity);
    mThrottleCommandResponseStates[wheelIndex] = driveTorque;
}

void PhysXVehicleRawWheelControl::setBrakeTorque(const uint32_t wheelIndex, const float brakeTorque)
{
    CARB_ASSERT(wheelIndex < mWheelCapacity);
    CARB_ASSERT(brakeTorque >= 0.0f);
    mBrakeCommandResponseStates[wheelIndex] = brakeTorque;
}

void PhysXVehicleRawWheelControl::setSteerAngle(const uint32_t wheelIndex, const float steerAngle)
{
    CARB_ASSERT(wheelIndex < mWheelCapacity);
    mSteerCommandResponseStates[wheelIndex] = steerAngle;
}

void PhysXVehicleRawWheelControl::disableWheel(const uint32_t wheelIndex)
{
    PhysXActorVehicleBase::disableWheel(wheelIndex);

    setDriveTorque(wheelIndex, 0.0f);
}


size_t PhysXVehicleManagedWheelControl::computeDataSizeExcludingClass(const usdparser::VehicleDesc& vehicleDesc,
    PhysXVehicleManagedWheelControlMemoryOffsets& memOffsets)
{
    const uint32_t wheelCount = static_cast<uint32_t>(vehicleDesc.wheelAttachments.size());

    size_t memSize = PhysXActorVehicleBase::computeDataSizeExcludingClass(vehicleDesc, memOffsets);

    const size_t brakeCommandRespParamsCount = vehicleDesc.brakes.size() ?
        vehicleDesc.brakes.size() : sBrakingSystemMaxCount;

    memOffsets.brakeCommandResponseParamsOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(brakeCommandRespParamsCount * sizeof(::physx::vehicle2::PxVehicleBrakeCommandResponseParams));

    if ((vehicleDesc.steering) && (vehicleDesc.steering->type == SteeringDesc::eAckermann))
    {
        memOffsets.ackermannParamsOffset = memSize;
        memSize += ALIGNED_SIZE_VEHICLE(sizeof(::physx::vehicle2::PxVehicleAckermannParams));
    }
    else
        memOffsets.ackermannParamsOffset = PhysXVehicleBaseDataMemoryOffsets::kInvalidOffset;

    static_assert(::physx::vehicle2::PxVehicleLimits::eMAX_NB_WHEELS < 256, "");  // will use 1 byte per wheel index and index count
    memOffsets.wheelIndexListsOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(WheelIndexListType::eCOUNT * (wheelCount + 1) * sizeof(uint8_t));

    return memSize;
}

void PhysXVehicleManagedWheelControl::setDataPointers(uint8_t* memory, const PhysXVehicleManagedWheelControlMemoryOffsets& memOffsets)
{
    PhysXActorVehicleBase::setDataPointers(memory, memOffsets);

    mBrakeCommandResponseParams = reinterpret_cast<::physx::vehicle2::PxVehicleBrakeCommandResponseParams*>(
        memory + memOffsets.brakeCommandResponseParamsOffset);

    if (memOffsets.ackermannParamsOffset != PhysXVehicleBaseDataMemoryOffsets::kInvalidOffset)
    {
        mAckermannParams = reinterpret_cast<::physx::vehicle2::PxVehicleAckermannParams*>(
            memory + memOffsets.ackermannParamsOffset);
    }
    else
        mAckermannParams = nullptr;

    mWheelIndexLists = reinterpret_cast<uint8_t*>(memory + memOffsets.wheelIndexListsOffset);
}

static void setNonlinearCommandResponseParams(const NonlinearCmdResponseDesc& nonlinearCmdResponseDesc,
    ::physx::vehicle2::PxVehicleCommandNonLinearResponseParams& nonlinearResponseParams)
{
    CARB_ASSERT(nonlinearCmdResponseDesc.commandValues.size() == nonlinearCmdResponseDesc.speedResponsesPerCommandValue.size());
    CARB_ASSERT(nonlinearCmdResponseDesc.commandValues.size() <= ::physx::vehicle2::PxVehicleCommandNonLinearResponseParams::eMAX_NB_COMMAND_VALUES);
    CARB_ASSERT(nonlinearCmdResponseDesc.speedResponses.size() <= ::physx::vehicle2::PxVehicleCommandValueResponseTable::eMAX_NB_SPEED_RESPONSES);

    nonlinearResponseParams.nbCommandValues = static_cast<PxU16>(nonlinearCmdResponseDesc.commandValues.size());
    nonlinearResponseParams.nbSpeedResponses = static_cast<PxU16>(nonlinearCmdResponseDesc.speedResponses.size());

    PxU16 currentGraphStartIdx = (nonlinearResponseParams.nbCommandValues > 0) ? static_cast<PxU16>(nonlinearCmdResponseDesc.speedResponsesPerCommandValue[0]) : 0;

    for (PxU16 i = 0; i < nonlinearResponseParams.nbCommandValues; i++)
    {
        nonlinearResponseParams.commandValues[i] = nonlinearCmdResponseDesc.commandValues[i];
        nonlinearResponseParams.speedResponsesPerCommandValue[i] = nonlinearCmdResponseDesc.speedResponsesPerCommandValue[i];

        const PxU16 nextIdx = i + 1;
        PxU16 nextGraphStartIdx;
        PxU16 nbSpeedResponsesForCurrentCmdVal;
        if (nextIdx < nonlinearResponseParams.nbCommandValues)
        {
            nextGraphStartIdx = nonlinearCmdResponseDesc.speedResponsesPerCommandValue[nextIdx];
            CARB_ASSERT(nextGraphStartIdx > currentGraphStartIdx);

            nbSpeedResponsesForCurrentCmdVal = nextGraphStartIdx - currentGraphStartIdx;
        }
        else
        {
            nextGraphStartIdx = currentGraphStartIdx;
            nbSpeedResponsesForCurrentCmdVal = nonlinearResponseParams.nbSpeedResponses - currentGraphStartIdx;
        }

        nonlinearResponseParams.nbSpeedResponsesPerCommandValue[i] = nbSpeedResponsesForCurrentCmdVal;
        CARB_ASSERT((currentGraphStartIdx + nbSpeedResponsesForCurrentCmdVal) <= nonlinearResponseParams.nbSpeedResponses);

        PxU16 startIdxDesc = currentGraphStartIdx;
        const PxU16 startIdxPhysX = 2 * startIdxDesc;
        const PxU16 endIdxPhysX = startIdxPhysX + nbSpeedResponsesForCurrentCmdVal;
        for (PxU16 j = startIdxPhysX; j < endIdxPhysX; j++)
        {
            const carb::Float2& sr = nonlinearCmdResponseDesc.speedResponses[startIdxDesc];

            nonlinearResponseParams.speedResponses[j] = sr.x;
            nonlinearResponseParams.speedResponses[j + nbSpeedResponsesForCurrentCmdVal] = sr.y;

            startIdxDesc++;
        }

        currentGraphStartIdx = nextGraphStartIdx;
    }
}

void PhysXVehicleManagedWheelControl::setDataValues(const usdparser::VehicleDesc& vehicleDesc,
    ::physx::PxRigidDynamic& vehicleActor, ::physx::PxPhysics& pxPhysics,
    const std::vector<::physx::PxTransform>& wheelShapeLocalPoses, const std::vector<::physx::PxShape*>& wheelShapeMapping,
    const std::vector<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>& tireMaterialFrictionTables,
    const std::vector<uint32_t>& wheelIndexToDataMap,
    const ::physx::vehicle2::PxVehicleFrame& frame, const float gravityMagnitude)
{
    PhysXActorVehicleBase::setDataValues(vehicleDesc, vehicleActor, pxPhysics, wheelShapeLocalPoses, wheelShapeMapping,
        tireMaterialFrictionTables, wheelIndexToDataMap, frame, gravityMagnitude);

    const uint32_t wheelCount = static_cast<uint32_t>(vehicleDesc.wheelAttachments.size());

    //---

    mPhysxSteerState.setToDefault();

    //---

    ::physx::PxMemZero(mWheelIndexLists, WheelIndexListType::eCOUNT * (wheelCount + 1) * sizeof(mWheelIndexLists[0]));

    //---

    mCommandState.setToDefault();

    uint32_t brakingSystemCount = static_cast<uint32_t>(vehicleDesc.brakes.size());
    if (brakingSystemCount)
    {
        mCommandState.nbBrakes = brakingSystemCount;

        for (uint32_t i = 0; i < sBrakingSystemMaxCount; i++)
        {
            mExternalToInternalBrakesIndex[i] = sInvalidInternalBrakesIndex;
        }

        mIsUsingDeprecatedBrakesSetup = false;
    }
    else  // deprecated system
    {
        mCommandState.nbBrakes = sBrakingSystemMaxCount;

        for (uint32_t i = 0; i < sBrakingSystemMaxCount; i++)
        {
            mExternalToInternalBrakesIndex[i] = i;
        }

        mIsUsingDeprecatedBrakesSetup = true;
    }

    for (uint32_t i = 0; i < mCommandState.nbBrakes; i++)
    {
        ::physx::vehicle2::PxVehicleBrakeCommandResponseParams& brakeRespParams = mBrakeCommandResponseParams[i];
        new(&brakeRespParams)(::physx::vehicle2::PxVehicleBrakeCommandResponseParams)();

        if (brakingSystemCount)
            brakeRespParams.maxResponse = 0.0f;
        else  // deprecated system
            brakeRespParams.maxResponse = 1.0f;

        ::physx::PxMemZero(brakeRespParams.wheelResponseMultipliers,
            wheelCount * sizeof(brakeRespParams.wheelResponseMultipliers[0]));
    }

    for (uint32_t i = 0; i < brakingSystemCount; i++)
    {
        const usdparser::BrakesDesc* brakeDesc = vehicleDesc.brakes[i];
        const uint32_t brakesIndex = brakeDesc->brakesIndex;
        CARB_ASSERT(brakesIndex < sBrakingSystemMaxCount);

        mExternalToInternalBrakesIndex[brakesIndex] = i;

        ::physx::vehicle2::PxVehicleBrakeCommandResponseParams& brakeRespParams = mBrakeCommandResponseParams[i];

        brakeRespParams.maxResponse = brakeDesc->maxBrakeTorque;

        uint8_t* wheelIndexList = getWheelIndexListIncludingCountEntry(WheelIndexListType::eBRAKES + i);

        const uint32_t wheelIndexCount = static_cast<uint32_t>(brakeDesc->wheels.size());
        if (wheelIndexCount)
        {
            (*wheelIndexList) = static_cast<uint8_t>(wheelIndexCount);

            const uint32_t multiplierCount = static_cast<uint32_t>(brakeDesc->torqueMultipliers.size());
            CARB_ASSERT((!multiplierCount) || (multiplierCount == wheelIndexCount));

            for (uint32_t j = 0; j < wheelIndexCount; j++)
            {
                const int wheelIndex = brakeDesc->wheels[j];
                wheelIndexList++;
                (*wheelIndexList) = static_cast<uint8_t>(wheelIndex);

                brakeRespParams.wheelResponseMultipliers[wheelIndex] = multiplierCount ?
                    brakeDesc->torqueMultipliers[j] : 1.0f;
            }
        }
        else
        {
            (*wheelIndexList) = static_cast<uint8_t>(wheelCount);

            for (uint32_t j = 0; j < wheelCount; j++)
            {
                wheelIndexList++;
                (*wheelIndexList) = static_cast<uint8_t>(j);

                brakeRespParams.wheelResponseMultipliers[j] = 1.0f;
            }
        }

        if (brakeDesc->nonlinearCmdResponse)
        {
            setNonlinearCommandResponseParams(*brakeDesc->nonlinearCmdResponse, brakeRespParams.nonlinearResponse);
        }
    }

    mSteerCommandResponseParams.maxResponse = 0.0f;
    ::physx::PxMemZero(mSteerCommandResponseParams.wheelResponseMultipliers,
        wheelCount * sizeof(mSteerCommandResponseParams.wheelResponseMultipliers[0]));

    if (vehicleDesc.steering)
    {
        uint8_t* wheelIndexList = getWheelIndexListIncludingCountEntry(WheelIndexListType::eSTEER);

        if (vehicleDesc.steering->type == SteeringDesc::eBasic)
        {
            const SteeringBasicDesc* steerDesc = static_cast<const SteeringBasicDesc*>(vehicleDesc.steering);

            mSteerCommandResponseParams.maxResponse = steerDesc->maxSteerAngle;

            const uint32_t wheelIndexCount = static_cast<uint32_t>(steerDesc->wheels.size());
            if (wheelIndexCount)
            {
                (*wheelIndexList) = static_cast<uint8_t>(wheelIndexCount);

                const uint32_t multiplierCount = static_cast<uint32_t>(steerDesc->angleMultipliers.size());
                CARB_ASSERT((!multiplierCount) || (multiplierCount == wheelIndexCount));

                for (uint32_t j = 0; j < wheelIndexCount; j++)
                {
                    const int wheelIndex = steerDesc->wheels[j];
                    wheelIndexList++;
                    (*wheelIndexList) = static_cast<uint8_t>(wheelIndex);

                    mSteerCommandResponseParams.wheelResponseMultipliers[wheelIndex] = multiplierCount ?
                        steerDesc->angleMultipliers[j] : 1.0f;
                }
            }
            else
            {
                (*wheelIndexList) = static_cast<uint8_t>(wheelCount);

                for (uint32_t j = 0; j < wheelCount; j++)
                {
                    wheelIndexList++;
                    (*wheelIndexList) = static_cast<uint8_t>(j);

                    mSteerCommandResponseParams.wheelResponseMultipliers[j] = 1.0f;
                }
            }
        }
        else
        {
            CARB_ASSERT(vehicleDesc.steering->type == SteeringDesc::eAckermann);
            const SteeringAckermannDesc* steerDesc = static_cast<const SteeringAckermannDesc*>(vehicleDesc.steering);

            CARB_ASSERT(mAckermannParams != nullptr);
            ::physx::vehicle2::PxVehicleAckermannParams* ackermannParams = mAckermannParams;
            new(ackermannParams)(::physx::vehicle2::PxVehicleAckermannParams)();

            mSteerCommandResponseParams.maxResponse = steerDesc->maxSteerAngle;

            (*wheelIndexList) = 2;  // first entry is count. Fixed to two wheels for Ackermann.

            const int wheel0Index = steerDesc->wheel0;
            wheelIndexList++;
            (*wheelIndexList) = static_cast<uint8_t>(wheel0Index);
            mSteerCommandResponseParams.wheelResponseMultipliers[wheel0Index] = 1.0f;

            const int wheel1Index = steerDesc->wheel1;
            wheelIndexList++;
            (*wheelIndexList) = static_cast<uint8_t>(wheel1Index);
            mSteerCommandResponseParams.wheelResponseMultipliers[wheel1Index] = 1.0f;

            ackermannParams->wheelIds[0] = wheel0Index;
            ackermannParams->wheelIds[1] = wheel1Index;
            ackermannParams->wheelBase = steerDesc->wheelBase;
            ackermannParams->trackWidth = steerDesc->trackWidth;
            ackermannParams->strength = steerDesc->strength;
        }

        if (vehicleDesc.steering->nonlinearCmdResponse)
        {
            setNonlinearCommandResponseParams(*vehicleDesc.steering->nonlinearCmdResponse, mSteerCommandResponseParams.nonlinearResponse);
        }

        mIsUsingDeprecatedSteerSetup = false;
    }
    else  // deprecated system
    {
        mSteerCommandResponseParams.maxResponse = 1.0f;

        mIsUsingDeprecatedSteerSetup = true;
    }

    // deprecated system
    if (mIsUsingDeprecatedBrakesSetup || mIsUsingDeprecatedSteerSetup)
    {
        for (uint32_t i = 0; i < wheelCount; i++)
        {
            const uint32_t dataIndex = wheelIndexToDataMap[i];
            CARB_ASSERT(dataIndex < wheelCount);

            const usdparser::WheelAttachmentDesc& wheelAttDesc = vehicleDesc.wheelAttachments[dataIndex];

            CARB_ASSERT(wheelAttDesc.wheel);
            const usdparser::WheelDesc& wheelDesc = *wheelAttDesc.wheel;

            if (!brakingSystemCount)  // deprecated system
            {
                CARB_ASSERT(mCommandState.nbBrakes == sBrakingSystemMaxCount);
                mBrakeCommandResponseParams[0].wheelResponseMultipliers[i] = wheelDesc.maxBrakeTorque;
                mBrakeCommandResponseParams[1].wheelResponseMultipliers[i] = wheelDesc.maxHandBrakeTorque;
            }

            if (!vehicleDesc.steering)
                mSteerCommandResponseParams.wheelResponseMultipliers[i] = wheelDesc.maxSteerAngle;
        }
    }
}

void PhysXVehicleManagedWheelControl::simulateBegin(const float dt, const ::physx::vehicle2::PxVehicleSimulationContext& context)
{
    ::physx::vehicle2::PxVehiclePhysXActorBeginComponent* actorBeginComp = static_cast<::physx::vehicle2::PxVehiclePhysXActorBeginComponent*>(this);
    actorBeginComp->update(dt, context);
}

void PhysXVehicleManagedWheelControl::setWheelsToRestState()
{
    PhysXActorVehicleBase::setWheelsToRestState();

    setBrakeAndSteerCommandResponseStatesToDefault();
}


size_t PhysXVehicleDirectDrive::computeDataSizeExcludingClass(const usdparser::VehicleDesc& vehicleDesc,
    PhysXVehicleDirectDriveDataMemoryOffsets& memOffsets)
{
    const uint32_t wheelCount = static_cast<uint32_t>(vehicleDesc.wheelAttachments.size());

    size_t memSize = PhysXVehicleManagedWheelControl::computeDataSizeExcludingClass(vehicleDesc, memOffsets);

    memOffsets.throttleCommandResponseStatesOffset = memSize;
    memSize += ALIGNED_SIZE_VEHICLE(wheelCount * sizeof(PhysXVehicleDirectDrive::mThrottleCommandResponseStates[0]));

    return memSize;
}

void PhysXVehicleDirectDrive::setDataPointers(uint8_t* memory,
    const PhysXVehicleDirectDriveDataMemoryOffsets& memOffsets)
{
    PhysXVehicleManagedWheelControl::setDataPointers(memory, memOffsets);

    static_assert(sizeof(PhysXVehicleDirectDrive::mThrottleCommandResponseStates[0]) == sizeof(::physx::PxReal), "");
    mThrottleCommandResponseStates = reinterpret_cast<::physx::PxReal*>(memory + memOffsets.throttleCommandResponseStatesOffset);
}

void PhysXVehicleDirectDrive::setDataValues(const usdparser::VehicleDesc& vehicleDesc,
    ::physx::PxRigidDynamic& vehicleActor, ::physx::PxPhysics& pxPhysics,
    const std::vector<::physx::PxTransform>& wheelShapeLocalPoses, const std::vector<::physx::PxShape*>& wheelShapeMapping,
    const std::vector<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>& tireMaterialFrictionTables,
    const std::vector<uint32_t>& wheelIndexToDataMap,
    const ::physx::vehicle2::PxVehicleFrame& frame, const float gravityMagnitude)
{
    PhysXVehicleManagedWheelControl::setDataValues(vehicleDesc, vehicleActor, pxPhysics,
        wheelShapeLocalPoses, wheelShapeMapping, tireMaterialFrictionTables, wheelIndexToDataMap,
        frame, gravityMagnitude);

    //---

    mTransmissionCommandState.setToDefault();
    mTransmissionCommandState.gear = ::physx::vehicle2::PxVehicleDirectDriveTransmissionCommandState::eFORWARD;

    //---

    CARB_ASSERT(vehicleDesc.drive);
    CARB_ASSERT(vehicleDesc.drive->type == usdparser::eVehicleDriveBasic);
    const usdparser::DriveBasicDesc& driveBasicDesc = *static_cast<const usdparser::DriveBasicDesc*>(vehicleDesc.drive);
    mThrottleCommandResponseParams.maxResponse = driveBasicDesc.peakTorque;

    const uint32_t wheelCount = static_cast<uint32_t>(vehicleDesc.wheelAttachments.size());
    if (vehicleDesc.differential)
    {
        ::physx::PxMemZero(mThrottleCommandResponseParams.wheelResponseMultipliers, wheelCount * sizeof(mThrottleCommandResponseParams.wheelResponseMultipliers[0]));

        const uint32_t drivenWheelCount = static_cast<uint32_t>(vehicleDesc.differential->wheels.size());
        const uint32_t torqueRatiosCount = static_cast<uint32_t>(vehicleDesc.differential->torqueRatios.size());

        uint8_t* wheelIndexList = getWheelIndexListIncludingCountEntry(WheelIndexListType::eDRIVE);
        (*wheelIndexList) = static_cast<uint8_t>(drivenWheelCount);

        const float autoRatio = drivenWheelCount ? 1.0f / drivenWheelCount : 0.0f;

        CARB_ASSERT((!torqueRatiosCount) || (drivenWheelCount == torqueRatiosCount));

        for (uint32_t i = 0; i < drivenWheelCount; i++)
        {
            const int wheelIndex = vehicleDesc.differential->wheels[i];
            wheelIndexList++;
            (*wheelIndexList) = static_cast<uint8_t>(wheelIndex);

            mThrottleCommandResponseParams.wheelResponseMultipliers[wheelIndex] =
                torqueRatiosCount ? vehicleDesc.differential->torqueRatios[i] : autoRatio;
        }
    }
    else
    {
        for (uint32_t i = 0; i < wheelCount; i++)
        {
            const uint32_t dataIndex = wheelIndexToDataMap[i];
            CARB_ASSERT(dataIndex < wheelCount);

            const usdparser::WheelAttachmentDesc& wheelAttDesc = vehicleDesc.wheelAttachments[dataIndex];
            mThrottleCommandResponseParams.wheelResponseMultipliers[i] = wheelAttDesc.driven ? 1.0f : 0.0f;
        }
    }

    if (driveBasicDesc.nonlinearCmdResponse)
    {
        setNonlinearCommandResponseParams(*driveBasicDesc.nonlinearCmdResponse, mThrottleCommandResponseParams.nonlinearResponse);
    }

    setThrottleCommandResponseStatesToDefault();
}

void PhysXVehicleDirectDrive::setComponentSequence()
{
    bool success;
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleDirectDriveCommandResponseComponent*>(this));
    CARB_ASSERT(success);
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleDirectDriveActuationStateComponent*>(this));
    CARB_ASSERT(success);
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehiclePhysXRoadGeometrySceneQueryComponent*>(this));
    CARB_ASSERT(success);

    mSubstepGroupId = mComponentSequence.beginSubstepGroup();
    CARB_ASSERT(mSubstepGroupId != ::physx::vehicle2::PxVehicleComponentSequence::eINVALID_SUBSTEP_GROUP);
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleSuspensionComponent*>(this));
    CARB_ASSERT(success);
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleTireComponent*>(this));
    CARB_ASSERT(success);
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehiclePhysXConstraintComponent*>(this));
    CARB_ASSERT(success);
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleDirectDrivetrainComponent*>(this));
    CARB_ASSERT(success);
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleRigidBodyComponent*>(this));
    CARB_ASSERT(success);
    mComponentSequence.endSubstepGroup();

    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleWheelComponent*>(this));
    CARB_ASSERT(success);
}

PhysXVehicleDirectDrive* PhysXVehicleDirectDrive::create(::physx::PxRigidDynamic& vehicleActor,
                                                         ::physx::PxPhysics& pxPhysics,
                                                         const usdparser::VehicleDesc& vehicleDesc,
                                                         const std::vector<::physx::PxTransform>& wheelShapeLocalPoses,
                                                         const std::vector<::physx::PxShape*>& wheelShapeMapping,
                                                         const std::vector<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>& tireMaterialFrictionTables,
                                                         const std::vector<uint32_t>& wheelIndexToDataMap,
                                                         const ::physx::vehicle2::PxVehicleFrame& frame,
                                                         const float gravityMagnitude,
                                                         ::physx::PxAllocatorCallback* allocatorForPvd)
{
    return createInternal<PhysXVehicleDirectDrive, PhysXVehicleDirectDriveDataMemoryOffsets>(
        vehicleActor, pxPhysics, vehicleDesc, wheelShapeLocalPoses, wheelShapeMapping,
        tireMaterialFrictionTables, wheelIndexToDataMap, frame, gravityMagnitude,
        allocatorForPvd);
}

void PhysXVehicleDirectDrive::release(OmniPvdWriter* pvdWriter, ::physx::PxAllocatorCallback* allocator)
{
    PhysXVehicleManagedWheelControl::release(pvdWriter, allocator);

    ICE_PLACEMENT_DELETE(this, PhysXVehicleDirectDrive);
}

void PhysXVehicleDirectDrive::disableWheel(const uint32_t wheelIndex)
{
    PhysXVehicleManagedWheelControl::disableWheel(wheelIndex);

    // Current logic is that disabled wheels do not trigger torque being redistributed
    // among remaining driven wheels. User has to explicitly take care of that if desired.
    CARB_ASSERT(wheelIndex < mWheelCapacity);
    mThrottleCommandResponseParams.wheelResponseMultipliers[wheelIndex] = 0.0f;
}

void PhysXVehicleDirectDrive::setWheelsToRestState()
{
    PhysXVehicleManagedWheelControl::setWheelsToRestState();

    setThrottleCommandResponseStatesToDefault();
}


size_t PhysXVehicleEngineDrive::computeDataSizeExcludingClass(const usdparser::VehicleDesc& vehicleDesc,
    PhysXVehicleEngineDriveDataMemoryOffsets& memOffsets)
{
    size_t memSize = PhysXVehicleManagedWheelControl::computeDataSizeExcludingClass(vehicleDesc, memOffsets);

    if (vehicleDesc.differential && (vehicleDesc.differential->type == DifferentialDesc::eTank))
    {
        memOffsets.transmissionCommandStateOffset = memSize;
        memSize += ALIGNED_SIZE_VEHICLE(sizeof(::physx::vehicle2::PxVehicleTankDriveTransmissionCommandState));

        memOffsets.differentialParamsOffset = memSize;
        memSize += ALIGNED_SIZE_VEHICLE(sizeof(::physx::vehicle2::PxVehicleTankDriveDifferentialParams));

        memOffsets.constraintGroupStateOffset = memSize;
        memSize += ALIGNED_SIZE_VEHICLE(sizeof(::physx::vehicle2::PxVehicleWheelConstraintGroupState));
    }
    else
    {
        memOffsets.transmissionCommandStateOffset = memSize;
        memSize += ALIGNED_SIZE_VEHICLE(sizeof(::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState));

        memOffsets.differentialParamsOffset = memSize;
        memSize += ALIGNED_SIZE_VEHICLE(sizeof(::physx::vehicle2::PxVehicleMultiWheelDriveDifferentialParams));

        memOffsets.constraintGroupStateOffset = PhysXVehicleBaseDataMemoryOffsets::kInvalidOffset;
    }

    CARB_ASSERT(vehicleDesc.drive);
    CARB_ASSERT(vehicleDesc.drive->type == usdparser::eVehicleDriveStandard);
    const usdparser::DriveStandardDesc& driveStandardDesc = *static_cast<const usdparser::DriveStandardDesc*>(vehicleDesc.drive);
    if (driveStandardDesc.autoGearBox)
    {
        memOffsets.autoboxOffset = memSize;
        memSize += ALIGNED_SIZE_VEHICLE(sizeof(PhysXVehicleEngineDrive::Autobox));
    }
    else
    {
        memOffsets.autoboxOffset = PhysXVehicleBaseDataMemoryOffsets::kInvalidOffset;
    }

    return memSize;
}

void PhysXVehicleEngineDrive::setDataPointers(uint8_t* memory,
    const PhysXVehicleEngineDriveDataMemoryOffsets& memOffsets)
{
    PhysXVehicleManagedWheelControl::setDataPointers(memory, memOffsets);

    mTransmissionCommandState = reinterpret_cast<::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState*>(memory + memOffsets.transmissionCommandStateOffset);
    mDifferentialParams = reinterpret_cast<::physx::vehicle2::PxVehicleMultiWheelDriveDifferentialParams*>(memory + memOffsets.differentialParamsOffset);
    if (memOffsets.constraintGroupStateOffset != PhysXVehicleBaseDataMemoryOffsets::kInvalidOffset)
        mConstraintGroupState = reinterpret_cast<::physx::vehicle2::PxVehicleWheelConstraintGroupState*>(memory + memOffsets.constraintGroupStateOffset);
    else
        mConstraintGroupState = nullptr;

    if (memOffsets.autoboxOffset != PhysXVehicleBaseDataMemoryOffsets::kInvalidOffset)
        mAutobox = reinterpret_cast<Autobox*>(memory + memOffsets.autoboxOffset);
    else
        mAutobox = nullptr;
}

void PhysXVehicleEngineDrive::setDataValues(const usdparser::VehicleDesc& vehicleDesc,
    ::physx::PxRigidDynamic& vehicleActor, ::physx::PxPhysics& pxPhysics,
    const std::vector<::physx::PxTransform>& wheelShapeLocalPoses, const std::vector<::physx::PxShape*>& wheelShapeMapping,
    const std::vector<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>& tireMaterialFrictionTables,
    const std::vector<uint32_t>& wheelIndexToDataMap,
    const ::physx::vehicle2::PxVehicleFrame& frame, const float gravityMagnitude)
{
    PhysXVehicleManagedWheelControl::setDataValues(vehicleDesc, vehicleActor, pxPhysics,
        wheelShapeLocalPoses, wheelShapeMapping, tireMaterialFrictionTables, wheelIndexToDataMap,
        frame, gravityMagnitude);

    CARB_ASSERT(vehicleDesc.drive);
    CARB_ASSERT(vehicleDesc.drive->type == usdparser::eVehicleDriveStandard);
    const usdparser::DriveStandardDesc& driveStandardDesc = *static_cast<const usdparser::DriveStandardDesc*>(vehicleDesc.drive);

    //---

    if (vehicleDesc.differential && (vehicleDesc.differential->type == DifferentialDesc::eTank))
    {
        mDifferentialType = DifferentialType::eTANK;

        ::physx::vehicle2::PxVehicleTankDriveTransmissionCommandState* tankDriveTransmissionCommandState =
            static_cast<::physx::vehicle2::PxVehicleTankDriveTransmissionCommandState*>(mTransmissionCommandState);
        tankDriveTransmissionCommandState->setToDefault();

        CARB_ASSERT(mConstraintGroupState);
        mConstraintGroupState->setToDefault();
    }
    else
    {
        mDifferentialType = DifferentialType::eMULTI_WHEEL;

        mTransmissionCommandState->setToDefault();
    }

    //---

    CARB_ASSERT(driveStandardDesc.gears);
    const usdparser::GearsDesc& gearsDesc = *driveStandardDesc.gears;

    constexpr uint32_t neutralGear = eNEUTRAL_GEAR;
    mGearboxParams.neutralGear = neutralGear;
    mGearboxParams.ratios[0] = gearsDesc.ratios[0];  // reverse
    mGearboxParams.ratios[neutralGear] = 0.0f;

    const uint32_t gearRatioCountWithoutNeutral = static_cast<uint32_t>(gearsDesc.ratios.size());
    CARB_ASSERT(gearRatioCountWithoutNeutral < ::physx::vehicle2::PxVehicleGearboxParams::eMAX_NB_GEARS);  // "<" since neutral is not represented in USD
    for (uint32_t i = 1; i < gearRatioCountWithoutNeutral; i++)
    {
        mGearboxParams.ratios[neutralGear + i] = gearsDesc.ratios[i];
    }

    mGearboxParams.finalRatio = gearsDesc.ratioScale;
    mGearboxParams.nbRatios = gearRatioCountWithoutNeutral + 1; // neutral is not represented in USD as it is fixed
    mGearboxParams.switchTime = gearsDesc.switchTime;

    const uint32_t initialGear = (gearRatioCountWithoutNeutral > 1) ? neutralGear + 1 : 0;
    mGearboxState.setToDefault();
    mGearboxState.currentGear = initialGear;
    mGearboxState.targetGear = initialGear;

    mTransmissionCommandState->targetGear = mAutobox ? ::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR :
        initialGear;

    //---

    if (mAutobox)
    {
        CARB_ASSERT(driveStandardDesc.autoGearBox);
        const usdparser::AutoGearBoxDesc& autoGearboxDesc = *driveStandardDesc.autoGearBox;

        // these should never be used but rather have some well defined value in there
        mAutobox->mParams.upRatios[0] = 0.0f;
        mAutobox->mParams.upRatios[neutralGear] = 0.0f;
        mAutobox->mParams.upRatios[gearRatioCountWithoutNeutral] = 0.0f;

        const uint32_t upRatioCount = static_cast<uint32_t>(autoGearboxDesc.upRatios.size());
        CARB_ASSERT(upRatioCount < (::physx::vehicle2::PxVehicleGearboxParams::eMAX_NB_GEARS - 2));  // reverse, neutral and highest gear is not represented
                                                                                                     // in USD as the values would not be used
        for (uint32_t i = 0; i < upRatioCount; i++)
        {
            mAutobox->mParams.upRatios[neutralGear + 1 + i] = autoGearboxDesc.upRatios[i];
        }

        // these should never be used but rather have some well defined value in there
        mAutobox->mParams.downRatios[0] = 0.0f;
        mAutobox->mParams.downRatios[neutralGear] = 0.0f;
        mAutobox->mParams.downRatios[neutralGear + 1] = 0.0f;

        const uint32_t downRatioCount = static_cast<uint32_t>(autoGearboxDesc.downRatios.size());
        CARB_ASSERT(downRatioCount < (::physx::vehicle2::PxVehicleGearboxParams::eMAX_NB_GEARS - 2));  // reverse, neutral and first is not represented
                                                                                                       // in USD as the values would not be used
        for (uint32_t i = 0; i < downRatioCount; i++)
        {
            mAutobox->mParams.downRatios[neutralGear + 2 + i] = autoGearboxDesc.downRatios[i];
        }

        mAutobox->mParams.latency = autoGearboxDesc.latency;

        mAutobox->mState.setToDefault();
    }

    //---

    CARB_ASSERT(driveStandardDesc.clutch);
    const usdparser::ClutchDesc& clutchDesc = *driveStandardDesc.clutch;

    mClutchParams.accuracyMode = ::physx::vehicle2::PxVehicleClutchAccuracyMode::eBEST_POSSIBLE;
    mClutchParams.estimateIterations = 5;

    mClutchCommandResponseParams.maxResponse = clutchDesc.strength;

    mClutchCommandResponseState.setToDefault();

    //---

    mThrottleCommandResponseState.setToDefault();

    //---

    const uint32_t wheelCount = static_cast<uint32_t>(vehicleDesc.wheelAttachments.size());
    if (vehicleDesc.differential)
    {
        ::physx::PxMemZero(mDifferentialParams->torqueRatios, wheelCount * sizeof(mDifferentialParams->torqueRatios[0]));
        ::physx::PxMemZero(mDifferentialParams->aveWheelSpeedRatios, wheelCount * sizeof(mDifferentialParams->aveWheelSpeedRatios[0]));

        const uint32_t drivenWheelCount = static_cast<uint32_t>(vehicleDesc.differential->wheels.size());
        const uint32_t torqueRatiosCount = static_cast<uint32_t>(vehicleDesc.differential->torqueRatios.size());
        const uint32_t avgWheelSpeedRatiosCount = static_cast<uint32_t>(vehicleDesc.differential->averageWheelSpeedRatios.size());

        const float autoRatio = drivenWheelCount ? 1.0f / drivenWheelCount : 0.0f;

        CARB_ASSERT((!torqueRatiosCount) || (drivenWheelCount == torqueRatiosCount));
        CARB_ASSERT((!avgWheelSpeedRatiosCount) || (drivenWheelCount == avgWheelSpeedRatiosCount));

        uint8_t* wheelIndexList = getWheelIndexListIncludingCountEntry(WheelIndexListType::eDRIVE);
        (*wheelIndexList) = static_cast<uint8_t>(drivenWheelCount);

        for (uint32_t i = 0; i < drivenWheelCount; i++)
        {
            const int wheelIndex = vehicleDesc.differential->wheels[i];
            wheelIndexList++;
            (*wheelIndexList) = static_cast<uint8_t>(wheelIndex);

            mDifferentialParams->torqueRatios[wheelIndex] =
                torqueRatiosCount ? vehicleDesc.differential->torqueRatios[i] : autoRatio;

            mDifferentialParams->aveWheelSpeedRatios[wheelIndex] =
                avgWheelSpeedRatiosCount ? vehicleDesc.differential->averageWheelSpeedRatios[i] : autoRatio;
        }

        if (vehicleDesc.differential->type == DifferentialDesc::eTank)
        {
            mDifferentialType = DifferentialType::eTANK;

            ::physx::vehicle2::PxVehicleTankDriveDifferentialParams* tankDifferentialParams =
                static_cast<::physx::vehicle2::PxVehicleTankDriveDifferentialParams*>(mDifferentialParams);

            TankDifferentialDesc* tankDiffDesc = static_cast<TankDifferentialDesc*>(vehicleDesc.differential);
            const uint32_t trackCount = static_cast<uint32_t>(tankDiffDesc->numberOfWheelsPerTrack.size());
            tankDifferentialParams->nbTracks = trackCount;
            if (trackCount)
            {
                CARB_ASSERT(trackCount == static_cast<uint32_t>(tankDiffDesc->thrustIndexPerTrack.size()));
                CARB_ASSERT(trackCount == static_cast<uint32_t>(tankDiffDesc->trackToWheelIndices.size()));

                tankDifferentialParams->nbTracks = trackCount;

                for (uint32_t i = 0; i < trackCount; i++)
                {
                    tankDifferentialParams->thrustIdPerTrack[i] = tankDiffDesc->thrustIndexPerTrack[i];
                    tankDifferentialParams->nbWheelsPerTrack[i] = tankDiffDesc->numberOfWheelsPerTrack[i];
                    tankDifferentialParams->trackToWheelIds[i] = tankDiffDesc->trackToWheelIndices[i];
                }

                const uint32_t wheelIndexEntryCount = static_cast<uint32_t>(tankDiffDesc->wheelIndicesInTrackOrder.size());
                CARB_ASSERT(wheelIndexEntryCount <= ::physx::vehicle2::PxVehicleLimits::eMAX_NB_WHEELS);

                static_assert(sizeof(int) == sizeof(tankDifferentialParams->wheelIdsInTrackOrder[0]), "");
                memcpy(tankDifferentialParams->wheelIdsInTrackOrder, tankDiffDesc->wheelIndicesInTrackOrder.data(),
                    wheelIndexEntryCount * sizeof(int));
            }
        }
    }
    else
    {
        uint32_t drivenWheelCount = 0;
        for (uint32_t i = 0; i < wheelCount; i++)
        {
            // note: no need to pick matching wheel attachment descriptor here as this is just to
            // get the total count of driven wheels
            const usdparser::WheelAttachmentDesc& wheelAttDesc = vehicleDesc.wheelAttachments[i];
            if (wheelAttDesc.driven)
            {
                drivenWheelCount++;
            }
        }

        const float driveDistribution = drivenWheelCount ? 1.0f / drivenWheelCount : 0.0f;
        for (uint32_t i = 0; i < wheelCount; i++)
        {
            const uint32_t dataIndex = wheelIndexToDataMap[i];
            CARB_ASSERT(dataIndex < wheelCount);

            const usdparser::WheelAttachmentDesc& wheelAttDesc = vehicleDesc.wheelAttachments[dataIndex];
            if (wheelAttDesc.driven)
            {
                mDifferentialParams->torqueRatios[i] = driveDistribution;
                mDifferentialParams->aveWheelSpeedRatios[i] = driveDistribution;
            }
            else
            {
                mDifferentialParams->torqueRatios[i] = 0.0f;
                mDifferentialParams->aveWheelSpeedRatios[i] = 0.0f;
            }
        }
    }

    mDifferentialState.setToDefault();

    //---

    CARB_ASSERT(driveStandardDesc.engine);
    const usdparser::EngineDesc& engineDesc = *driveStandardDesc.engine;

    CARB_ASSERT(engineDesc.torqueCurvePointCount <= vehicle2::PxVehicleEngineParams::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES);
    for (uint32_t i = 0; i < engineDesc.torqueCurvePointCount; i++)
    {
        mEngineParams.torqueCurve.addPair(engineDesc.torqueCurve[i].x, engineDesc.torqueCurve[i].y);
    }
    mEngineParams.moi = engineDesc.moi;
    mEngineParams.peakTorque = engineDesc.peakTorque;
    mEngineParams.idleOmega = engineDesc.idleRotationSpeed;
    mEngineParams.maxOmega = engineDesc.maxRotationSpeed;
    mEngineParams.dampingRateFullThrottle = engineDesc.dampingRateFullThrottle;
    mEngineParams.dampingRateZeroThrottleClutchEngaged = engineDesc.dampingRateZeroThrottleClutchEngaged;
    mEngineParams.dampingRateZeroThrottleClutchDisengaged = engineDesc.dampingRateZeroThrottleClutchDisengaged;

    mEngineState.setToDefault();
    mEngineState.rotationSpeed = mEngineParams.idleOmega;
}

void PhysXVehicleEngineDrive::setComponentSequence()
{
    bool success;
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleEngineDriveCommandResponseComponent*>(this));
    CARB_ASSERT(success);

    if (mDifferentialType == DifferentialType::eMULTI_WHEEL)
    {
        success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleMultiWheelDriveDifferentialStateComponent*>(this));
        CARB_ASSERT(success);
    }
    else
    {
        CARB_ASSERT(mDifferentialType == DifferentialType::eTANK);

        success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleTankDriveDifferentialStateComponent*>(this));
        CARB_ASSERT(success);
    }

    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleEngineDriveActuationStateComponent*>(this));
    CARB_ASSERT(success);
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehiclePhysXRoadGeometrySceneQueryComponent*>(this));
    CARB_ASSERT(success);

    mSubstepGroupId = mComponentSequence.beginSubstepGroup();
    CARB_ASSERT(mSubstepGroupId != ::physx::vehicle2::PxVehicleComponentSequence::eINVALID_SUBSTEP_GROUP);
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleSuspensionComponent*>(this));
    CARB_ASSERT(success);
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleTireComponent*>(this));
    CARB_ASSERT(success);
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehiclePhysXConstraintComponent*>(this));
    CARB_ASSERT(success);
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleEngineDrivetrainComponent*>(this));
    CARB_ASSERT(success);
    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleRigidBodyComponent*>(this));
    CARB_ASSERT(success);
    mComponentSequence.endSubstepGroup();

    success = mComponentSequence.add(static_cast<::physx::vehicle2::PxVehicleWheelComponent*>(this));
    CARB_ASSERT(success);
}

PhysXVehicleEngineDrive* PhysXVehicleEngineDrive::create(::physx::PxRigidDynamic& vehicleActor,
                                                         ::physx::PxPhysics& pxPhysics,
                                                         const usdparser::VehicleDesc& vehicleDesc,
                                                         const std::vector<::physx::PxTransform>& wheelShapeLocalPoses,
                                                         const std::vector<::physx::PxShape*>& wheelShapeMapping,
                                                         const std::vector<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>& tireMaterialFrictionTables,
                                                         const std::vector<uint32_t>& wheelIndexToDataMap,
                                                         const ::physx::vehicle2::PxVehicleFrame& frame,
                                                         const float gravityMagnitude,
                                                         ::physx::PxAllocatorCallback* allocatorForPvd)
{
    return createInternal<PhysXVehicleEngineDrive, PhysXVehicleEngineDriveDataMemoryOffsets>(
        vehicleActor, pxPhysics, vehicleDesc, wheelShapeLocalPoses, wheelShapeMapping,
        tireMaterialFrictionTables, wheelIndexToDataMap, frame, gravityMagnitude,
        allocatorForPvd);
}

void PhysXVehicleEngineDrive::release(OmniPvdWriter* pvdWriter, ::physx::PxAllocatorCallback* allocator)
{
    PhysXVehicleManagedWheelControl::release(pvdWriter, allocator);

    ICE_PLACEMENT_DELETE(this, PhysXVehicleEngineDrive);
}

void PhysXVehicleEngineDrive::setWheelToDriven(const uint32_t wheelIndex, const bool driven)
{
    CARB_ASSERT(wheelIndex < mWheelCapacity);

    const bool isCurrentlyDriven = mDifferentialParams->torqueRatios[wheelIndex] > 0.0f;
    if (isCurrentlyDriven != driven)
    {
        if (isCurrentlyDriven)
        {
            mDifferentialParams->torqueRatios[wheelIndex] = 0.0f;
            mDifferentialParams->aveWheelSpeedRatios[wheelIndex] = 0.0f;
        }
        else
        {
            // set to dummy positive value
            mDifferentialParams->torqueRatios[wheelIndex] = FLT_MIN;
            mDifferentialParams->aveWheelSpeedRatios[wheelIndex] = FLT_MIN;
        }

        uint32_t drivenWheelCount = 0;
        for (uint32_t i = 0; i < mAxleDescription.nbWheels; i++)
        {
            const uint32_t enabledWheelIndex = mAxleDescription.wheelIdsInAxleOrder[i];
            if (mDifferentialParams->torqueRatios[enabledWheelIndex] > 0.0f)
            {
                drivenWheelCount++;
            }
        }

        if (drivenWheelCount > 0)
        {
            const float driveDistribution = 1.0f / drivenWheelCount;
            for (uint32_t i = 0; i < mAxleDescription.nbWheels; i++)
            {
                const uint32_t enabledWheelIndex = mAxleDescription.wheelIdsInAxleOrder[i];
                if (mDifferentialParams->torqueRatios[enabledWheelIndex] > 0.0f)
                {
                    mDifferentialParams->torqueRatios[enabledWheelIndex] = driveDistribution;
                    mDifferentialParams->aveWheelSpeedRatios[enabledWheelIndex] = driveDistribution;
                }
            }
        }
    }
}

static void disableWheelInTankDifferential(const uint32_t wheelIndex,
    ::physx::vehicle2::PxVehicleTankDriveDifferentialParams& tankDriveDiffParams)
{
    for (uint32_t i = 0; i < tankDriveDiffParams.nbTracks; i++)
    {
        const uint32_t nbWheelsInTrack = tankDriveDiffParams.nbWheelsPerTrack[i];
        const uint32_t wheelIdStartIdx = tankDriveDiffParams.trackToWheelIds[i];
        const uint32_t wheelIdEndIdx = wheelIdStartIdx + nbWheelsInTrack;

        for (uint32_t j = wheelIdStartIdx; j < wheelIdEndIdx; j++)
        {
            if (tankDriveDiffParams.wheelIdsInTrackOrder[j] == wheelIndex)
            {
                const uint32_t swapIndex = wheelIdEndIdx - 1;  // -1 is safe, since wheelIdStartIdx < wheelIdEndIdx, else
                                                                // we would not get into the for-loop
                if (j < swapIndex)
                {
                    const uint32_t swapValue = tankDriveDiffParams.wheelIdsInTrackOrder[j];
                    tankDriveDiffParams.wheelIdsInTrackOrder[j] = tankDriveDiffParams.wheelIdsInTrackOrder[swapIndex];
                    tankDriveDiffParams.wheelIdsInTrackOrder[swapIndex] = swapValue;
                }

                tankDriveDiffParams.nbWheelsPerTrack[i]--;

                return;
            }
        }
    }
}

void PhysXVehicleEngineDrive::disableWheel(const uint32_t wheelIndex)
{
    PhysXVehicleManagedWheelControl::disableWheel(wheelIndex);

    // Current logic is that disabled wheels do not trigger torque etc. being redistributed
    // among remaining driven wheels. User has to explicitly take care of that if desired.
    CARB_ASSERT(wheelIndex < mWheelCapacity);
    mDifferentialParams->torqueRatios[wheelIndex] = 0.0f;
    mDifferentialParams->aveWheelSpeedRatios[wheelIndex] = 0.0f;

    if (mDifferentialType == DifferentialType::eTANK)
    {
        ::physx::vehicle2::PxVehicleTankDriveDifferentialParams* tankDriveDiffParams =
            static_cast<::physx::vehicle2::PxVehicleTankDriveDifferentialParams*>(mDifferentialParams);

        disableWheelInTankDifferential(wheelIndex, *tankDriveDiffParams);
    }
}

void PhysXVehicleEngineDrive::setToRestState()
{
    PhysXVehicleManagedWheelControl::setToRestState();

    mGearboxState.currentGear = mGearboxState.targetGear;
    mGearboxState.gearSwitchTime = PX_VEHICLE_NO_GEAR_SWITCH_PENDING;

    if (mAutobox)
    {
        mAutobox->mState.timeSinceLastShift = PX_VEHICLE_UNSPECIFIED_TIME_SINCE_LAST_SHIFT;
        mAutobox->mState.activeAutoboxGearShift = false;
    }

    mClutchCommandResponseState.setToDefault();
    mClutchSlipState.setToDefault();

    mThrottleCommandResponseState.setToDefault();

    mDifferentialState.setToDefault();

    mEngineState.rotationSpeed = mEngineParams.idleOmega;
}


PhysXActorVehicleBase* VehicleGenerator::createVehicle(::physx::PxRigidDynamic& vehicleActor,
                                                       ::physx::PxPhysics& physics,
                                                       const VehicleDesc& vehicleDesc,
                                                       const std::vector<::physx::PxTransform>& wheelShapeLocalPoses,
                                                       const std::vector<::physx::PxShape*>& wheelShapeMapping,
                                                       const std::vector<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>& tireMaterialFrictionTables,
                                                       const std::vector<uint32_t>& wheelIndexToDataMap,
                                                       const ::physx::vehicle2::PxVehicleFrame& frame,
                                                       const float gravityMagnitude,
                                                       ::physx::PxAllocatorCallback* allocatorForPvd)
{
    if (vehicleDesc.drive)
    {
        if (vehicleDesc.drive->type == usdparser::eVehicleDriveStandard)
        {
            return PhysXVehicleEngineDrive::create(vehicleActor, physics,
                vehicleDesc, wheelShapeLocalPoses, wheelShapeMapping, tireMaterialFrictionTables,
                wheelIndexToDataMap, frame, gravityMagnitude,
                allocatorForPvd);
        }
        else
        {
            CARB_ASSERT(vehicleDesc.drive->type == usdparser::eVehicleDriveBasic);
            return PhysXVehicleDirectDrive::create(vehicleActor, physics,
                vehicleDesc, wheelShapeLocalPoses, wheelShapeMapping, tireMaterialFrictionTables,
                wheelIndexToDataMap, frame, gravityMagnitude,
                allocatorForPvd);
        }
    }
    else
    {
        return PhysXVehicleRawWheelControl::create(vehicleActor, physics,
            vehicleDesc, wheelShapeLocalPoses, wheelShapeMapping, tireMaterialFrictionTables,
            wheelIndexToDataMap, frame, gravityMagnitude,
            allocatorForPvd);
    }
}

} // namespace physx
} // namespace omni
