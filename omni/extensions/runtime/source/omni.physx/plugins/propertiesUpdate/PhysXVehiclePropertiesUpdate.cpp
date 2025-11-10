// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXPropertiesUpdate.h"

#include <PhysXTools.h>
#include <VehicleGenerator.h>
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

extern void* getInternalPtr(const pxr::SdfPath& path, omni::physx::PhysXType type);
extern ObjectId getObjectId(const pxr::SdfPath& path, PhysXType type);


static const pxr::UsdAttribute& getAttribute(const pxr::UsdProperty& property)
{
    CARB_ASSERT(property.Is<pxr::UsdAttribute>());
    return static_cast<const pxr::UsdAttribute&>(property);
}

static const pxr::UsdRelationship& getRelationship(const pxr::UsdProperty& property)
{
    CARB_ASSERT(property.Is<pxr::UsdRelationship>());
    return static_cast<const pxr::UsdRelationship&>(property);
}

static bool getSingleRelationshipPath(const pxr::UsdRelationship& relationship, pxr::SdfPath& path)
{
    SdfPathVector paths;
    relationship.GetTargets(&paths);
    if (paths.size() == 1)
    {
        path = paths[0];
        return true;
    }
    else
    {
        CARB_LOG_ERROR("Relationship \"%s\" needs to have exactly 1 entry.\n",
            relationship.GetPath().GetText());
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// vehicle

struct EngineParam
{
    enum Enum
    {
        eMOI,
        ePEAK_TORQUE,
        eMAX_ROT_SPEED,
        eIDLE_ROT_SPEED,
        eDAMP_FULL_THROTTLE,
        eDAMP_ZERO_THROTTLE_CLUTCH_ENGAGED,
        eDAMP_ZERO_THROTTLE_CLUTCH_DISENGAGED
    };
};

struct SuspensionParam
{
    enum Enum
    {
        eSPRING_STRENGTH,
        eDAMPING_RATE,
        eMAX_COMPRESSION,  // deprecated
        eMAX_DROOP,  // deprecated
        eTRAVEL_DISTANCE,
        eSPRUNG_MASS,
        eCAMBER_AT_REST,  // deprecated
        eCAMBER_AT_MAX_COMPRESSION,  // deprecated
        eCAMBER_AT_MAX_DROOP  // deprecated
    };
};

struct TireParam
{
    enum Enum
    {
        eLAT_STIFF_X,  // deprecated
        eLAT_STIFF_Y,  // deprecated
        eLAT_STIFF_GRAPH,
        eLONG_STIFF_PER_GRAV,  // deprecated
        eLONG_STIFF,
        eCAMBER_STIFF_PER_GRAV,  // deprecated
        eCAMBER_STIFF,
        eTIRE_FRICTION_TABLE,
        eREST_LOAD
    };
};

struct WheelParam
{
    enum Enum
    {
        eRADIUS,
        eHALF_WIDTH,
        eMASS,
        eMOMENT_OF_INERTIA,
        eDAMPING_RATE,
        eMAX_BRAKE_TORQUE,  // deprecated
        eMAX_HANDBRAKE_TORQUE,  // deprecated
        eMAX_STEER_ANGLE,  // deprecated
        eTOE_ANGLE  // deprecated
    };
};

struct WheelAttachmentParam
{
    enum Enum
    {
        eWHEEL,
        eTIRE,
        eSUSPENSION,
        eSUSPENSION_TRAVEL_DIRECTION,
        eSUSPENSION_FORCE_APP_POINT_OFFSET,  // deprecated
        eWHEEL_CENTER_OF_MASS_OFFSET,  // deprecated
        eTIRE_FORCE_APP_POINT_OFFSET,  // deprecated
        eSUSPENSION_FRAME_POSITION,
        eSUSPENSION_FRAME_ORIENTATION,
        eWHEEL_FRAME_POSITION,
        eWHEEL_FRAME_ORIENTATION,
        eDRIVEN,
        eCOLLISON_GROUP
    };
};

struct SuspensionComplianceParam
{
    enum Enum
    {
        eWHEEL_TOE_ANGLE,
        eWHEEL_CAMBER_ANGLE,
        eSUSPENSION_FORCE_APP_POINT,
        eTIRE_FORCE_APP_POINT
    };
};

struct VehicleParam
{
    enum Enum
    {
        eTIRE_MIN_PASSIVE_LONG_SLIP_DENOM,
        eTIRE_MIN_ACTIVE_LONG_SLIP_DENOM,
        eTIRE_MIN_LAT_SLIP_DENOM,
        eTIRE_LONG_STICKY_THRESHOLD_SPEED,
        eTIRE_LONG_STICKY_THRESHOLD_TIME,
        eTIRE_LONG_STICKY_DAMPING,
        eTIRE_LAT_STICKY_THRESHOLD_SPEED,
        eTIRE_LAT_STICKY_THRESHOLD_TIME,
        eTIRE_LAT_STICKY_DAMPING,
    };
};

struct VehicleControllerParam
{
    enum Enum
    {
        eACCELERATOR,
        eBRAKE0,
        eBRAKE1,
        eBRAKE,  // deprecated
        eHANDBRAKE,  // deprecated
        eSTEER,
        eSTEER_LEFT,  // deprecated
        eSTEER_RIGHT,  // deprecated
        eTARGET_GEAR
    };
};

struct VehicleTankControllerParam
{
    enum Enum
    {
        eTHRUST0,
        eTHRUST1
    };
};

struct VehicleWheelControllerParam
{
    enum Enum
    {
        eDRIVE_TORQUE,
        eBRAKE_TORQUE,
        eSTEER_ANGLE
    };
};

struct VehicleAckermannParam
{
    enum Enum
    {
        eMAX_STEER_ANGLE,
        eWHEEL_BASE,
        eTRACK_WIDTH,
        eSTRENGTH
    };
};

static void attrPositiveErrLog(const pxr::TfToken& attributeName)
{
    CARB_LOG_ERROR("Attribute \"%s\" requires positive value.\n", attributeName.GetText());
}

static void attrNonNegativeErrLog(const pxr::TfToken& attributeName)
{
    CARB_LOG_ERROR("Attribute \"%s\" requires non-negative value.\n", attributeName.GetText());
}

static void attrNoModDuringSimErrLog(const pxr::TfToken& attributeName)
{
    CARB_LOG_ERROR("Attribute \"%s\": modification not allowed once the simulation has been started. Changes will be ignored.\n",
        attributeName.GetText());
}

static void relNoModDuringSimErrLog(const pxr::TfToken& relationshipName)
{
    CARB_LOG_ERROR("Relationship \"%s\": modification not allowed once the simulation has been started. Changes will be ignored.\n",
        relationshipName.GetText());
}

static bool attrRangeCheck(const float value, const float min, const float max,
    const pxr::TfToken& attributeName)
{
    if ((value >= min) && (value <= max))
        return true;
    else
    {
        CARB_LOG_ERROR("Attribute \"%s\" requires values in range [%f, %f].\n",
            attributeName.GetText(), min, max);

        return false;
    }
}

static const InternalDatabase::Record* getObjectRecord(omni::physx::PhysXType type,
    omni::physx::usdparser::ObjectId objectId)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    omni::physx::internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    return db.getFullTypedRecord(type, objectId);
}

bool omni::physx::updateVehicleContextUpdateMode(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleContextVerticalAxis(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleContextLongitudinalAxis(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

static bool updateVehicleEngineData(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode, const EngineParam::Enum engineParam)
{
    const InternalDatabase::Record* objectRecord = getObjectRecord(ePTVehicleEngine, objectId);
    if (!objectRecord)
        return true;

    InternalVehicleReferenceList* vehicleRefList = static_cast<InternalVehicleReferenceList*>(objectRecord->mInternalPtr);

    float value;
    if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, value))
        return true;

    for (InternalVehicle* vehicle : vehicleRefList->mVehicles)
    {
        CARB_ASSERT(vehicle->mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE);
        PhysXVehicleEngineDrive* physxVehicle = static_cast<PhysXVehicleEngineDrive*>(vehicle->mPhysXVehicle);

        switch (engineParam)
        {
        case EngineParam::eMOI:
        {
            if (value > 0.0f)
                physxVehicle->setEngineMoi(value);
            else
                attrPositiveErrLog(property);
        }
        break;

        case EngineParam::ePEAK_TORQUE:
        {
            if (value >= 0.0f)
                physxVehicle->setEnginePeakTorque(value);
            else
                attrNonNegativeErrLog(property);
        }
        break;

        case EngineParam::eMAX_ROT_SPEED:
        {
            if (value >= 0.0f)
                physxVehicle->setEngineMaxRotationSpeed(value);
            else
                attrNonNegativeErrLog(property);
        }
        break;

        case EngineParam::eIDLE_ROT_SPEED:
        {
            if (value >= 0.0f)
                physxVehicle->setEngineIdleRotationSpeed(value);
            else
                attrNonNegativeErrLog(property);
        }
        break;

        case EngineParam::eDAMP_FULL_THROTTLE:
        {
            if (value >= 0.0f)
                physxVehicle->setEngineDampingRateFullThrottle(value);
            else
                attrNonNegativeErrLog(property);
        }
        break;

        case EngineParam::eDAMP_ZERO_THROTTLE_CLUTCH_ENGAGED:
        {
            if (value >= 0.0f)
                physxVehicle->setEngineDampingRateZeroThrottleClutchEngaged(value);
            else
                attrNonNegativeErrLog(property);
        }
        break;

        case EngineParam::eDAMP_ZERO_THROTTLE_CLUTCH_DISENGAGED:
        {
            if (value >= 0.0f)
                physxVehicle->setEngineDampingRateZeroThrottleClutchDisengaged(value);
            else
                attrNonNegativeErrLog(property);
        }
        break;
        }
    }

    return true;
}

bool omni::physx::updateVehicleEngineMomentOfInertia(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleEngineData(attachedStage, objectId, property, timeCode, EngineParam::eMOI);
}

bool omni::physx::updateVehicleEnginePeakTorque(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleEngineData(attachedStage, objectId, property, timeCode, EngineParam::ePEAK_TORQUE);
}

bool omni::physx::updateVehicleEngineMaxRotationSpeed(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleEngineData(attachedStage, objectId, property, timeCode, EngineParam::eMAX_ROT_SPEED);
}

bool omni::physx::updateVehicleEngineIdleRotationSpeed(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleEngineData(attachedStage, objectId, property, timeCode, EngineParam::eIDLE_ROT_SPEED);
}

bool omni::physx::updateVehicleEngineTorqueCurve(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleEngineDampingRateFullThrottle(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleEngineData(attachedStage, objectId, property, timeCode, EngineParam::eDAMP_FULL_THROTTLE);
}

bool omni::physx::updateVehicleEngineDampingRateZeroThrottleClutchEngaged(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleEngineData(attachedStage, objectId, property, timeCode, EngineParam::eDAMP_ZERO_THROTTLE_CLUTCH_ENGAGED);
}

bool omni::physx::updateVehicleEngineDampingRateZeroThrottleClutchDisengaged(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleEngineData(attachedStage, objectId, property, timeCode, EngineParam::eDAMP_ZERO_THROTTLE_CLUTCH_DISENGAGED);
}

bool omni::physx::updateVehicleTireFrictionTableFrictionValues(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const InternalDatabase::Record* objectRecord = getObjectRecord(ePTVehicleTireFrictionTable, objectId);
    if (!objectRecord)
        return true;

    const pxr::UsdAttribute& attribute = getAttribute(attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath).GetProperty(property));
    VtArray<float> frictionValues;
    attribute.Get(&frictionValues);

    InternalTireFrictionTable* tireFrictionTable = reinterpret_cast<InternalTireFrictionTable*>(objectRecord->mInternalPtr);
    tireFrictionTable->update(frictionValues);

    return true;
}

bool omni::physx::updateVehicleTireFrictionTableGroundMaterials(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    relNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleTireFrictionTableDefaultFrictionValue(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const InternalDatabase::Record* objectRecord = getObjectRecord(ePTVehicleTireFrictionTable, objectId);
    if (!objectRecord)
        return true;

    float value;
    if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, value))
        return false;

    InternalTireFrictionTable* tireFrictionTable = reinterpret_cast<InternalTireFrictionTable*>(objectRecord->mInternalPtr);
    tireFrictionTable->setDefaultFrictionValue(value);

    return true;
}

static void attrMaxDroopCompressionErrLog(const pxr::TfToken& attributeName)
{
    CARB_LOG_ERROR("Attribute \"%s\": either one of max droop or max compression must be greater than zero.\n",
        attributeName.GetText());
}

static bool updateVehicleSuspensionData(InternalVehicleWheelReferenceList* wheelRefList,
    const float value, const SuspensionParam::Enum suspParam, const pxr::TfToken& property)
{
    for (InternalVehicleWheelReferenceList::VehicleAndWheelsList::iterator iter = wheelRefList->mVehicleWheels.begin();
        iter != wheelRefList->mVehicleWheels.end(); iter++)
    {
        InternalVehicle* vehicle = iter->first;
        const uint32_t wheelBitmap = iter->second;
        PhysXActorVehicleBase* physxVehicle = vehicle->mPhysXVehicle;

        uint32_t currentIndex = 0;
        uint64_t currentBit = 1;
        while (wheelBitmap & (~(currentBit - 1)))
        {
            if (wheelBitmap & currentBit)
            {
                switch (suspParam)
                {
                case SuspensionParam::eSPRING_STRENGTH:
                {
                    physxVehicle->setSuspensionStiffness(currentIndex, value);
                }
                break;

                case SuspensionParam::eDAMPING_RATE:
                {
                    physxVehicle->setSuspensionDamping(currentIndex, value);
                }
                break;

                case SuspensionParam::eMAX_COMPRESSION:
                {
                    if ((value > 0) || (physxVehicle->getSuspensionMaxDroop(currentIndex) > 0))
                        physxVehicle->setSuspensionMaxCompression(currentIndex, value);
                    else
                        attrMaxDroopCompressionErrLog(property);
                }
                break;

                case SuspensionParam::eMAX_DROOP:
                {
                    if (vehicle->mFlags & InternalVehicleFlag::eUSER_DEFINED_MAX_DROOP)
                    {
                        if ((value > 0) || (physxVehicle->getSuspensionMaxCompression(currentIndex) > 0))
                            physxVehicle->setSuspensionMaxDroop(currentIndex, value);
                        else
                            attrMaxDroopCompressionErrLog(property);
                    }
                    else
                    {
                        CARB_LOG_ERROR("Attribute \"%s\": max droop values are auto-computed. Switching to user defined mode "
                            "once the simulation has been started is not supported.\n",
                            property.GetText());
                    }
                }
                break;

                case SuspensionParam::eTRAVEL_DISTANCE:
                {
                    physxVehicle->setSuspensionTravelDistance(currentIndex, value);
                }
                break;

                case SuspensionParam::eSPRUNG_MASS:
                {
                    if (vehicle->mFlags & InternalVehicleFlag::eUSER_DEFINED_SPRUNG_MASS)
                    {
                        const float oldSprungMass = physxVehicle->getSuspensionSprungMass(currentIndex);
                        physxVehicle->setSuspensionSprungMass(currentIndex, value);

                        const float massCorrection = value / oldSprungMass;

                        if (!(vehicle->mFlags & InternalVehicleFlag::eUSER_DEFINED_REST_LOAD))
                        {
                            physxVehicle->setTireRestLoad(currentIndex, massCorrection * physxVehicle->getTireRestLoad(currentIndex));

                            if (vehicle->mFlags & InternalVehicleFlag::eIS_USING_LAT_STIFF_Y)
                            {
                                // internally, lateral stiffness is defined in Newton per slip unit while in USD
                                // it is still defined as a multiplier on rest load. Thus, scaling the old value
                                // by the amount of change.
                                const float latStiff = massCorrection * physxVehicle->getTireLateralStiffnessY(currentIndex);
                                physxVehicle->setTireLateralStiffnessY(currentIndex, latStiff);
                            }
                        }

                        if (!(vehicle->mFlags & InternalVehicleFlag::eUSER_DEFINED_MAX_DROOP))
                        {
                            physxVehicle->updateMaxDroop(currentIndex, massCorrection);
                        }
                    }
                    else
                    {
                        CARB_LOG_ERROR("Attribute \"%s\": sprung mass values are auto-computed. Switching to user defined mode "
                            "once the simulation has been started is not supported.\n",
                            property.GetText());
                    }
                }
                break;

                case SuspensionParam::eCAMBER_AT_REST:
                {
                    physxVehicle->setSuspensionCamberAtRest(currentIndex, value);
                }
                break;

                case SuspensionParam::eCAMBER_AT_MAX_COMPRESSION:
                {
                    physxVehicle->setSuspensionCamberAtMaxCompression(currentIndex, value);
                }
                break;

                case SuspensionParam::eCAMBER_AT_MAX_DROOP:
                {
                    physxVehicle->setSuspensionCamberAtMaxDroop(currentIndex, value);
                }
                break;
                }
            }

            currentBit = currentBit << 1;
            currentIndex++;
        }
    }

    return true;
}

template<typename T>
static bool getWheelReferenceProperty(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode, omni::physx::PhysXType type,
    T& value, InternalVehicleWheelReferenceList*& vehicleWheelRefList)
{
    const InternalDatabase::Record* objectRecord = getObjectRecord(type, objectId);
    if (!objectRecord)
        return false;

    if (!getValue<T>(attachedStage, objectRecord->mPath, property, timeCode, value))
        return false;

    vehicleWheelRefList = static_cast<InternalVehicleWheelReferenceList*>(objectRecord->mInternalPtr);

    return true;
}

bool omni::physx::updateVehicleSuspensionSpringStrength(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleWheelRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleSuspension, value, vehicleWheelRefList))
    {
        if (value <= 0.0f)
        {
            attrPositiveErrLog(property);
            return true;
        }

        return updateVehicleSuspensionData(vehicleWheelRefList, value, SuspensionParam::eSPRING_STRENGTH, property);
    }

    return true;
}

bool omni::physx::updateVehicleSuspensionSpringDamperRate(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleWheelRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleSuspension, value, vehicleWheelRefList))
    {
        if (value < 0.0f)
        {
            attrNonNegativeErrLog(property);
            return true;
        }

        return updateVehicleSuspensionData(vehicleWheelRefList, value, SuspensionParam::eDAMPING_RATE, property);
    }

    return true;
}

bool omni::physx::updateVehicleSuspensionMaxCompression(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleWheelRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleSuspension, value, vehicleWheelRefList))
    {
        if (value < 0.0f)
        {
            attrNonNegativeErrLog(property);
            return true;
        }

        return updateVehicleSuspensionData(vehicleWheelRefList, value, SuspensionParam::eMAX_COMPRESSION, property);
    }

    return true;
}

bool omni::physx::updateVehicleSuspensionMaxDroop(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleWheelRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleSuspension, value, vehicleWheelRefList))
    {
        if (value < 0.0f)
        {
            CARB_LOG_ERROR("Attribute \"%s\": max droop values have to be non-negative. Note that it is not supported "
                "to switch max droop values to be auto-computed once the simulation has been started.\n", property.GetText());
            return true;
        }

        return updateVehicleSuspensionData(vehicleWheelRefList, value, SuspensionParam::eMAX_DROOP, property);
    }

    return true;
}

bool omni::physx::updateVehicleSuspensionTravelDistance(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleWheelRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleSuspension, value, vehicleWheelRefList))
    {
        if (value <= 0.0f)
        {
            attrPositiveErrLog(property);
            return true;
        }

        return updateVehicleSuspensionData(vehicleWheelRefList, value, SuspensionParam::eTRAVEL_DISTANCE, property);
    }

    return true;
}

bool omni::physx::updateVehicleSuspensionSprungMass(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleWheelRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleSuspension, value, vehicleWheelRefList))
    {
        if (value <= 0.0f)
        {
            CARB_LOG_ERROR("Attribute \"%s\": sprung mass values have to be greater than zero. Note that it is not supported "
                "to switch sprung mass values to be auto-computed once the simulation has been started.\n", property.GetText());
            return true;
        }

        return updateVehicleSuspensionData(vehicleWheelRefList, value, SuspensionParam::eSPRUNG_MASS, property);
    }

    return true;
}

bool omni::physx::updateVehicleSuspensionCamberAtRest(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleWheelRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleSuspension, value, vehicleWheelRefList))
    {
        return updateVehicleSuspensionData(vehicleWheelRefList, value, SuspensionParam::eCAMBER_AT_REST, property);
    }

    return true;
}

bool omni::physx::updateVehicleSuspensionCamberAtMaxCompression(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleWheelRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleSuspension, value, vehicleWheelRefList))
    {
        return updateVehicleSuspensionData(vehicleWheelRefList, value, SuspensionParam::eCAMBER_AT_MAX_COMPRESSION, property);
    }

    return true;
}

bool omni::physx::updateVehicleSuspensionCamberAtMaxDroop(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleWheelRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleSuspension, value, vehicleWheelRefList))
    {
        return updateVehicleSuspensionData(vehicleWheelRefList, value, SuspensionParam::eCAMBER_AT_MAX_DROOP, property);
    }

    return true;
}

static bool updateVehicleTireData(InternalVehicleWheelReferenceList* wheelRefList,
    const float* valueFloat, const void* valuePtr, const pxr::GfVec2f* valueFloat2,
    const TireParam::Enum tireParam,
    const pxr::TfToken& property)
{
    for (InternalVehicleWheelReferenceList::VehicleAndWheelsList::iterator iter = wheelRefList->mVehicleWheels.begin();
        iter != wheelRefList->mVehicleWheels.end(); iter++)
    {
        InternalVehicle* vehicle = iter->first;
        const uint32_t wheelBitmap = iter->second;
        PhysXActorVehicleBase* physxVehicle = vehicle->mPhysXVehicle;

        float gravityMagnitude;
        if ((tireParam == TireParam::eLONG_STIFF_PER_GRAV) || (tireParam == TireParam::eCAMBER_STIFF_PER_GRAV))
        {
            // need to do this per vehicle since different scenes might have different gravity
            gravityMagnitude = vehicle->mInternalScene.getScene()->getGravity().magnitude();
        }

        uint32_t currentIndex = 0;
        uint64_t currentBit = 1;
        while (wheelBitmap & (~(currentBit - 1)))
        {
            if (wheelBitmap & currentBit)
            {
                switch (tireParam)
                {
                case TireParam::eLAT_STIFF_X:
                {
                    CARB_ASSERT(valueFloat);
                    physxVehicle->setTireLateralStiffnessX(currentIndex, *valueFloat);
                }
                break;

                case TireParam::eLAT_STIFF_Y:
                {
                    CARB_ASSERT(valueFloat);
                    const float latStiff = (*valueFloat) * physxVehicle->getTireRestLoad(currentIndex);
                    physxVehicle->setTireLateralStiffnessY(currentIndex, latStiff);
                }
                break;

                case TireParam::eLAT_STIFF_GRAPH:
                {
                    CARB_ASSERT(valueFloat2);
                    physxVehicle->setTireLateralStiffnessX(currentIndex, (*valueFloat2)[0]);
                    physxVehicle->setTireLateralStiffnessY(currentIndex, (*valueFloat2)[1]);
                }
                break;

                case TireParam::eLONG_STIFF_PER_GRAV:
                {
                    CARB_ASSERT(valueFloat);
                    physxVehicle->setTireLongitudinalStiffness(currentIndex, (*valueFloat) * gravityMagnitude);
                }
                break;

                case TireParam::eLONG_STIFF:
                {
                    CARB_ASSERT(valueFloat);
                    physxVehicle->setTireLongitudinalStiffness(currentIndex, (*valueFloat));
                }
                break;

                case TireParam::eCAMBER_STIFF_PER_GRAV:
                {
                    CARB_ASSERT(valueFloat);
                    physxVehicle->setTireCamberStiffness(currentIndex, (*valueFloat) * gravityMagnitude);
                }
                break;

                case TireParam::eCAMBER_STIFF:
                {
                    CARB_ASSERT(valueFloat);
                    physxVehicle->setTireCamberStiffness(currentIndex, (*valueFloat));
                }
                break;

                case TireParam::eTIRE_FRICTION_TABLE:
                {
                    CARB_ASSERT(valuePtr);
                    const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams* materialFrictionTable =
                        static_cast<const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams*>(valuePtr);
                    physxVehicle->setTireMaterialFrictionTable(currentIndex, materialFrictionTable);
                }
                break;

                case TireParam::eREST_LOAD:
                {
                    CARB_ASSERT(valueFloat);
                    if (vehicle->mFlags & InternalVehicleFlag::eUSER_DEFINED_REST_LOAD)
                    {
                        const float oldRestLoad = physxVehicle->getTireRestLoad(currentIndex);

                        physxVehicle->setTireRestLoad(currentIndex, (*valueFloat));

                        if (vehicle->mFlags & InternalVehicleFlag::eIS_USING_LAT_STIFF_Y)
                        {
                            // internally, lateral stiffness is defined in Newton per slip unit while in USD
                            // it is still defined as a multiplier on rest load. Thus, scaling the old value
                            // by the amount of change.
                            const float forceCorrection = (*valueFloat) / oldRestLoad;
                            const float latStiff = forceCorrection * physxVehicle->getTireLateralStiffnessY(currentIndex);
                            physxVehicle->setTireLateralStiffnessY(currentIndex, latStiff);
                        }
                    }
                    else
                    {
                        CARB_LOG_ERROR("Attribute \"%s\": rest load values are auto-computed. Switching to user defined mode "
                            "once the simulation has been started is not supported.\n",
                            property.GetText());
                    }
                }
                }
            }

            currentBit = currentBit << 1;
            currentIndex++;
        }
    }

    return true;
}

bool omni::physx::updateVehicleTireLatStiffX(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleTireRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleTire, value, vehicleTireRefList))
    {
        if (value <= 0.0f)
        {
            attrPositiveErrLog(property);
            return true;
        }

        return updateVehicleTireData(vehicleTireRefList, &value, nullptr, nullptr,
            TireParam::eLAT_STIFF_X, property);
    }

    return true;
}

bool omni::physx::updateVehicleTireLatStiffY(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleTireRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleTire, value, vehicleTireRefList))
    {
        if (value <= 0.0f)
        {
            attrPositiveErrLog(property);
            return true;
        }

        return updateVehicleTireData(vehicleTireRefList, &value, nullptr, nullptr,
            TireParam::eLAT_STIFF_Y, property);
    }

    return true;
}

bool omni::physx::updateVehicleTireLateralStiffnessGraph(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    GfVec2f valueFloat2;
    InternalVehicleWheelReferenceList* vehicleTireRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleTire, valueFloat2, vehicleTireRefList))
    {
        if (valueFloat2[0] < 0.0f)
        {
            CARB_LOG_ERROR("Attribute \"%s\" requires non-negative value for first entry.\n", property.GetText());
            return true;
        }

        if (valueFloat2[1] <= 0.0f)
        {
            CARB_LOG_ERROR("Attribute \"%s\" requires positive value for second entry.\n", property.GetText());
            return true;
        }

        return updateVehicleTireData(vehicleTireRefList, nullptr, nullptr, &valueFloat2,
            TireParam::eLAT_STIFF_GRAPH, property);
    }

    return true;
}

bool omni::physx::updateVehicleTireLongStiffPerGrav(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleTireRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleTire, value, vehicleTireRefList))
    {
        if (value <= 0.0f)
        {
            attrPositiveErrLog(property);
            return true;
        }

        return updateVehicleTireData(vehicleTireRefList, &value, nullptr, nullptr,
            TireParam::eLONG_STIFF_PER_GRAV, property);
    }

    return true;
}

bool omni::physx::updateVehicleTireLongitudinalStiffness(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleTireRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleTire, value, vehicleTireRefList))
    {
        if (value <= 0.0f)
        {
            attrPositiveErrLog(property);
            return true;
        }

        return updateVehicleTireData(vehicleTireRefList, &value, nullptr, nullptr,
            TireParam::eLONG_STIFF, property);
    }

    return true;
}

bool omni::physx::updateVehicleTireCamberStiffPerGrav(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleTireRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleTire, value, vehicleTireRefList))
    {
        if (value < 0.0f)
        {
            attrNonNegativeErrLog(property);
            return true;
        }

        return updateVehicleTireData(vehicleTireRefList, &value, nullptr, nullptr,
            TireParam::eCAMBER_STIFF_PER_GRAV, property);
    }

    return true;
}

bool omni::physx::updateVehicleTireCamberStiffness(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleTireRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleTire, value, vehicleTireRefList))
    {
        if (value < 0.0f)
        {
            attrNonNegativeErrLog(property);
            return true;
        }

        return updateVehicleTireData(vehicleTireRefList, &value, nullptr, nullptr,
            TireParam::eCAMBER_STIFF, property);
    }

    return true;
}

bool omni::physx::updateVehicleTireFrictionVsSlip(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleTireFrictionTableRel(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const InternalDatabase::Record* objectRecord = getObjectRecord(ePTVehicleTire, objectId);
    if (!objectRecord)
        return true;

    InternalVehicleWheelReferenceList* vehicleTireRefList = static_cast<InternalVehicleWheelReferenceList*>(objectRecord->mInternalPtr);

    const pxr::UsdRelationship& relationship = getRelationship(attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath).GetProperty(property));
    SdfPath path;
    if (getSingleRelationshipPath(relationship, path))
    {
        void* obj = ::getInternalPtr(path, ePTVehicleTireFrictionTable);
        if (obj)
        {
            InternalTireFrictionTable* tireFrictionTable = static_cast<InternalTireFrictionTable*>(obj);
            return updateVehicleTireData(vehicleTireRefList, nullptr, tireFrictionTable->getMaterialFrictionTable(),
                nullptr, TireParam::eTIRE_FRICTION_TABLE, property);
        }
        else
        {
            CARB_LOG_ERROR("Relationship \"%s\": no internal tire friction table object could be found for "
                "the target path \"%s\".\n",
                relationship.GetPath().GetText(), path.GetText());
        }
    }

    return true;
}

bool omni::physx::updateVehicleTireRestLoad(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleTireRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleTire, value, vehicleTireRefList))
    {
        if (value <= 0.0f)
        {
            CARB_LOG_ERROR("Attribute \"%s\": rest load values have to be greater than zero. Note that it is not supported "
                "to switch rest load values to be auto-computed once the simulation has been started.\n", property.GetText());
            return true;
        }

        return updateVehicleTireData(vehicleTireRefList, &value, nullptr, nullptr,
            TireParam::eREST_LOAD, property);
    }

    return true;
}

static bool updateVehicleWheelData(InternalVehicleWheelReferenceList* wheelRefList,
    const float value, const WheelParam::Enum wheelParam)
{
    for (InternalVehicleWheelReferenceList::VehicleAndWheelsList::iterator iter = wheelRefList->mVehicleWheels.begin();
        iter != wheelRefList->mVehicleWheels.end(); iter++)
    {
        InternalVehicle* vehicle = iter->first;
        const uint32_t wheelBitmap = iter->second;
        PhysXActorVehicleBase* physxVehicle = vehicle->mPhysXVehicle;

        uint32_t currentIndex = 0;
        uint64_t currentBit = 1;
        while (wheelBitmap & (~(currentBit - 1)))
        {
            if (wheelBitmap & currentBit)
            {
                switch (wheelParam)
                {
                case WheelParam::eRADIUS:
                {
                    physxVehicle->setWheelRadius(currentIndex, value);
                }
                break;

                case WheelParam::eHALF_WIDTH:
                {
                    physxVehicle->setWheelHalfWidth(currentIndex, value);
                }
                break;

                case WheelParam::eMASS:
                {
                    physxVehicle->setWheelMass(currentIndex, value);
                }
                break;

                case WheelParam::eMOMENT_OF_INERTIA:
                {
                    physxVehicle->setWheelMoi(currentIndex, value);
                }
                break;

                case WheelParam::eDAMPING_RATE:
                {
                    physxVehicle->setWheelDampingRate(currentIndex, value);
                }
                break;

                case WheelParam::eMAX_BRAKE_TORQUE:
                {
                    if ((physxVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE) ||
                        (physxVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE))
                    {
                        PhysXVehicleManagedWheelControl* physxVehicleMWC = static_cast<PhysXVehicleManagedWheelControl*>(physxVehicle);
                        physxVehicleMWC->setWheelMaxBrakeTorque(currentIndex, 0, value);
                    }
                }
                break;

                case WheelParam::eMAX_HANDBRAKE_TORQUE:
                {
                    if ((physxVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE) ||
                        (physxVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE))
                    {
                        PhysXVehicleManagedWheelControl* physxVehicleMWC = static_cast<PhysXVehicleManagedWheelControl*>(physxVehicle);
                        physxVehicleMWC->setWheelMaxBrakeTorque(currentIndex, 1, value);
                    }
                }
                break;

                case WheelParam::eMAX_STEER_ANGLE:
                {
                    if ((physxVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE) ||
                        (physxVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE))
                    {
                        PhysXVehicleManagedWheelControl* physxVehicleMWC = static_cast<PhysXVehicleManagedWheelControl*>(physxVehicle);
                        physxVehicleMWC->setWheelMaxSteerAngle(currentIndex, value);
                    }
                }
                break;

                case WheelParam::eTOE_ANGLE:
                {
                    physxVehicle->setSuspensionToeAngle(currentIndex, value);
                }
                break;
                }
            }

            currentBit = currentBit << 1;
            currentIndex++;
        }
    }

    return true;
}

bool omni::physx::updateVehicleWheelRadius(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleWheelRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleWheel, value, vehicleWheelRefList))
    {
        if (value <= 0.0f)
        {
            attrPositiveErrLog(property);
            return true;
        }

        return updateVehicleWheelData(vehicleWheelRefList, value, WheelParam::eRADIUS);
    }

    return true;
}

bool omni::physx::updateVehicleWheelWidth(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleWheelRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleWheel, value, vehicleWheelRefList))
    {
        if (value <= 0.0f)
        {
            attrPositiveErrLog(property);
            return true;
        }

        return updateVehicleWheelData(vehicleWheelRefList, value * 0.5f, WheelParam::eHALF_WIDTH);
    }

    return true;
}

bool omni::physx::updateVehicleWheelMass(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleWheelRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleWheel, value, vehicleWheelRefList))
    {
        if (value <= 0.0f)
        {
            attrPositiveErrLog(property);
            return true;
        }

        return updateVehicleWheelData(vehicleWheelRefList, value, WheelParam::eMASS);
    }

    return true;
}

bool omni::physx::updateVehicleWheelMomentOfInertia(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleWheelRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleWheel, value, vehicleWheelRefList))
    {
        if (value <= 0.0f)
        {
            attrPositiveErrLog(property);
            return true;
        }

        return updateVehicleWheelData(vehicleWheelRefList, value, WheelParam::eMOMENT_OF_INERTIA);
    }

    return true;
}

bool omni::physx::updateVehicleWheelDampingRate(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleWheelRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleWheel, value, vehicleWheelRefList))
    {
        if (value < 0.0f)
        {
            attrNonNegativeErrLog(property);
            return true;
        }

        return updateVehicleWheelData(vehicleWheelRefList, value, WheelParam::eDAMPING_RATE);
    }

    return true;
}

bool omni::physx::updateVehicleWheelMaxBrakeTorque(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleWheelRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleWheel, value, vehicleWheelRefList))
    {
        if (value < 0.0f)
        {
            attrNonNegativeErrLog(property);
            return true;
        }

        return updateVehicleWheelData(vehicleWheelRefList, value, WheelParam::eMAX_BRAKE_TORQUE);
    }

    return true;
}

bool omni::physx::updateVehicleWheelMaxHandBrakeTorque(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleWheelRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleWheel, value, vehicleWheelRefList))
    {
        if (value < 0.0f)
        {
            attrNonNegativeErrLog(property);
            return true;
        }

        return updateVehicleWheelData(vehicleWheelRefList, value, WheelParam::eMAX_HANDBRAKE_TORQUE);
    }

    return true;
}

bool omni::physx::updateVehicleWheelMaxSteerAngle(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleWheelRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleWheel, value, vehicleWheelRefList))
    {
        if (PxAbs(value) >= PxHalfPi)
        {
            CARB_LOG_ERROR("Attribute \"%s\" has to be in (-Pi/2, Pi/2).\n", property.GetText());
            return true;
        }

        return updateVehicleWheelData(vehicleWheelRefList, value, WheelParam::eMAX_STEER_ANGLE);
    }

    return true;
}

bool omni::physx::updateVehicleWheelToeAngle(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelReferenceList* vehicleWheelRefList;
    if (getWheelReferenceProperty(attachedStage, objectId, property, timeCode, ePTVehicleWheel, value, vehicleWheelRefList))
    {
        if (PxAbs(value) >= PxHalfPi)
        {
            CARB_LOG_ERROR("Attribute \"%s\" has to be in (-Pi/2, Pi/2).\n", property.GetText());
            return true;
        }

        return updateVehicleWheelData(vehicleWheelRefList, value, WheelParam::eTOE_ANGLE);
    }

    return true;
}

static bool updateVehicleWheelAttachmentData(InternalVehicleWheelAttachment* wheelAttachment,
    const PxVec3* valueFloat3, const bool* valueBool, const pxr::UsdPrim* valuePrim, const PxQuat* valueQuat,
    const WheelAttachmentParam::Enum wheelAttachmentParam)
{
    InternalVehicle* vehicle = wheelAttachment->mVehicle;
    if (vehicle)
    {
        PhysXActorVehicleBase* physxVehicle = vehicle->mPhysXVehicle;
        const uint32_t wheelIndex = wheelAttachment->mWheelIndex;

        switch (wheelAttachmentParam)
        {
        case WheelAttachmentParam::eSUSPENSION_TRAVEL_DIRECTION:
        {
            CARB_ASSERT(valueFloat3);
            if (vehicle->mFlags & InternalVehicleFlag::eREFERENCE_FRAME_IS_COM)
                physxVehicle->setSuspensionTravelDirection(wheelIndex, *valueFloat3, nullptr);
            else
            {
                const PxTransform comPose = physxVehicle->getRigidDynamicActor()->getCMassLocalPose();
                physxVehicle->setSuspensionTravelDirection(wheelIndex, *valueFloat3, &comPose);
            }
        }
        break;

        case WheelAttachmentParam::eSUSPENSION_FORCE_APP_POINT_OFFSET:
        {
            CARB_ASSERT(valueFloat3);
            if (vehicle->mFlags & InternalVehicleFlag::eREFERENCE_FRAME_IS_COM)
                physxVehicle->setSuspensionForceAppPointOffset(wheelIndex, *valueFloat3, asPhysX(vehicle->mScale), nullptr);
            else
            {
                const PxTransform comPose = physxVehicle->getRigidDynamicActor()->getCMassLocalPose();
                physxVehicle->setSuspensionForceAppPointOffset(wheelIndex, *valueFloat3, asPhysX(vehicle->mScale),
                    &comPose);
            }
        }
        break;

        case WheelAttachmentParam::eWHEEL_CENTER_OF_MASS_OFFSET:
        {
            CARB_ASSERT(valueFloat3);
            if (vehicle->mFlags & InternalVehicleFlag::eREFERENCE_FRAME_IS_COM)
                physxVehicle->setWheelCenterOffset(wheelIndex, *valueFloat3, asPhysX(vehicle->mScale), nullptr);
            else
            {
                const PxTransform comPose = physxVehicle->getRigidDynamicActor()->getCMassLocalPose();
                physxVehicle->setWheelCenterOffset(wheelIndex, *valueFloat3, asPhysX(vehicle->mScale),
                    &comPose);
            }

            if (!(vehicle->mFlags & InternalVehicleFlag::eUSER_DEFINED_SPRUNG_MASS))
            {
                const InternalVehicleContext& vehicleContext = vehicle->mInternalScene.getVehicleContext();

                physxVehicle->updateSprungMassProperties(vehicleContext.getFrame().vrtAxis,
                    !(vehicle->mFlags & InternalVehicleFlag::eUSER_DEFINED_MAX_DROOP),
                    !(vehicle->mFlags & InternalVehicleFlag::eUSER_DEFINED_REST_LOAD),
                    (vehicle->mFlags & InternalVehicleFlag::eIS_USING_LAT_STIFF_Y),
                    true);
            }
        }
        break;

        case WheelAttachmentParam::eTIRE_FORCE_APP_POINT_OFFSET:
        {
            CARB_ASSERT(valueFloat3);
            if (vehicle->mFlags & InternalVehicleFlag::eREFERENCE_FRAME_IS_COM)
                physxVehicle->setTireForceAppPointOffset(wheelIndex, *valueFloat3, asPhysX(vehicle->mScale), nullptr);
            else
            {
                const PxTransform comPose = physxVehicle->getRigidDynamicActor()->getCMassLocalPose();
                physxVehicle->setTireForceAppPointOffset(wheelIndex, *valueFloat3, asPhysX(vehicle->mScale),
                    &comPose);
            }
        }
        break;

        case WheelAttachmentParam::eSUSPENSION_FRAME_POSITION:
        {
            CARB_ASSERT(valueFloat3);
            if (vehicle->mFlags & InternalVehicleFlag::eREFERENCE_FRAME_IS_COM)
                physxVehicle->setSuspensionFramePosition(wheelIndex, *valueFloat3, asPhysX(vehicle->mScale), nullptr);
            else
            {
                const PxTransform comPose = physxVehicle->getRigidDynamicActor()->getCMassLocalPose();
                physxVehicle->setSuspensionFramePosition(wheelIndex, *valueFloat3, asPhysX(vehicle->mScale),
                    &comPose);
            }

            if (!(vehicle->mFlags & InternalVehicleFlag::eUSER_DEFINED_SPRUNG_MASS))
            {
                const InternalVehicleContext& vehicleContext = vehicle->mInternalScene.getVehicleContext();

                physxVehicle->updateSprungMassProperties(vehicleContext.getFrame().vrtAxis,
                    !(vehicle->mFlags & InternalVehicleFlag::eUSER_DEFINED_MAX_DROOP),
                    !(vehicle->mFlags & InternalVehicleFlag::eUSER_DEFINED_REST_LOAD),
                    (vehicle->mFlags & InternalVehicleFlag::eIS_USING_LAT_STIFF_Y),
                    false);
            }
        }
        break;

        case WheelAttachmentParam::eSUSPENSION_FRAME_ORIENTATION:
        {
            CARB_ASSERT(valueQuat);
            if (vehicle->mFlags & InternalVehicleFlag::eREFERENCE_FRAME_IS_COM)
                physxVehicle->setSuspensionFrameOrientation(wheelIndex, *valueQuat, nullptr);
            else
            {
                const PxTransform comPose = physxVehicle->getRigidDynamicActor()->getCMassLocalPose();
                physxVehicle->setSuspensionFrameOrientation(wheelIndex, *valueQuat,
                    &comPose);
            }
        }
        break;

        case WheelAttachmentParam::eWHEEL_FRAME_POSITION:
        {
            CARB_ASSERT(valueFloat3);
            physxVehicle->setWheelFramePosition(wheelIndex, *valueFloat3, asPhysX(vehicle->mScale));
        }
        break;

        case WheelAttachmentParam::eWHEEL_FRAME_ORIENTATION:
        {
            CARB_ASSERT(valueQuat);
            physxVehicle->setWheelFrameOrientation(wheelIndex, *valueQuat);
        }
        break;

        case WheelAttachmentParam::eDRIVEN:
        {
            CARB_ASSERT(valueBool);
            vehicle->setDrivenWheel(wheelIndex, *valueBool);
        }
        break;

        case WheelAttachmentParam::eCOLLISON_GROUP:
        {
            PxFilterData fd;
            if (valuePrim)
            {
                CARB_ASSERT(valuePrim->IsA<UsdPhysicsCollisionGroup>());

                const ObjectId objectId = getObjectId(valuePrim->GetPath(), ePTCollisionGroup);
                if (objectId != kInvalidObjectId)
                {
                    const uint32_t collisionGroup = convertToCollisionGroup(objectId);
                    convertCollisionGroupToPxFilterData(collisionGroup, fd);
                }
                else
                {
                    CARB_LOG_ERROR("Collision group \"%s\": no internal record can be found. Wheel attachment change will be ignored.\n",
                        valuePrim->GetPath().GetText());
                    return true;
                }
            }
            // valuePrim being nullptr indicates that the collision group has been removed and thus
            // the filter data will get set to the default values

            physxVehicle->setWheelFilterData(wheelIndex, fd);
        }
        break;

        default:
            break;
        }
    }

    return true;
}

static InternalVehicleWheelAttachment* getWheelAttachment(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    pxr::SdfPath& path)
{
    const InternalDatabase::Record* objectRecord = getObjectRecord(ePTVehicleWheelAttachment, objectId);
    if (objectRecord)
    {
        path = objectRecord->mPath;
        return static_cast<InternalVehicleWheelAttachment*>(objectRecord->mInternalPtr);
    }
    else
        return nullptr;
}

template<typename T>
static bool getWheelAttachmentProperty(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode,
    T& value, InternalVehicleWheelAttachment*& wheelAttachment)
{
    pxr::SdfPath path;
    InternalVehicleWheelAttachment* wAtt = getWheelAttachment(attachedStage, objectId, path);
    if (wAtt)
    {
        if (!getValue<T>(attachedStage, path, property, timeCode, value))
            return false;

        wheelAttachment = wAtt;
    }
    else
        return false;

    return true;
}

bool omni::physx::updateVehicleWheelAttachmentIndex(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleWheelAttachmentWheel(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    relNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleWheelAttachmentTire(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    relNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleWheelAttachmentSuspension(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    relNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleWheelAttachmentSuspensionTravelDirection(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    GfVec3f value;
    InternalVehicleWheelAttachment* wheelAttachment;
    if (getWheelAttachmentProperty(attachedStage, objectId, property, timeCode, value, wheelAttachment))
    {
        PxVec3 pxValue = toPhysX(value);
        return updateVehicleWheelAttachmentData(wheelAttachment, &pxValue, nullptr, nullptr, nullptr,
            WheelAttachmentParam::eSUSPENSION_TRAVEL_DIRECTION);
    }

    return true;
}

bool omni::physx::updateVehicleWheelAttachmentSuspensionForceAppPointOffset(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    GfVec3f value;
    InternalVehicleWheelAttachment* wheelAttachment;
    if (getWheelAttachmentProperty(attachedStage, objectId, property, timeCode, value, wheelAttachment))
    {
        PxVec3 pxValue = toPhysX(value);
        return updateVehicleWheelAttachmentData(wheelAttachment, &pxValue, nullptr, nullptr, nullptr,
            WheelAttachmentParam::eSUSPENSION_FORCE_APP_POINT_OFFSET);
    }

    return true;
}

bool omni::physx::updateVehicleWheelAttachmentWheelCenterOfMassOffset(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    GfVec3f value;
    InternalVehicleWheelAttachment* wheelAttachment;
    if (getWheelAttachmentProperty(attachedStage, objectId, property, timeCode, value, wheelAttachment))
    {
        PxVec3 pxValue = toPhysX(value);
        return updateVehicleWheelAttachmentData(wheelAttachment, &pxValue, nullptr, nullptr, nullptr,
            WheelAttachmentParam::eWHEEL_CENTER_OF_MASS_OFFSET);
    }

    return true;
}

bool omni::physx::updateVehicleWheelAttachmentTireForceAppPointOffset(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    GfVec3f value;
    InternalVehicleWheelAttachment* wheelAttachment;
    if (getWheelAttachmentProperty(attachedStage, objectId, property, timeCode, value, wheelAttachment))
    {
        PxVec3 pxValue = toPhysX(value);
        return updateVehicleWheelAttachmentData(wheelAttachment, &pxValue, nullptr, nullptr, nullptr,
            WheelAttachmentParam::eTIRE_FORCE_APP_POINT_OFFSET);
    }

    return true;
}

bool omni::physx::updateVehicleWheelAttachmentSuspensionFramePosition(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    GfVec3f value;
    InternalVehicleWheelAttachment* wheelAttachment;
    if (getWheelAttachmentProperty(attachedStage, objectId, property, timeCode, value, wheelAttachment))
    {
        PxVec3 pxValue = toPhysX(value);
        return updateVehicleWheelAttachmentData(wheelAttachment, &pxValue, nullptr, nullptr, nullptr,
            WheelAttachmentParam::eSUSPENSION_FRAME_POSITION);
    }

    return true;
}

bool omni::physx::updateVehicleWheelAttachmentSuspensionFrameOrientation(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    GfQuatf value;
    InternalVehicleWheelAttachment* wheelAttachment;
    if (getWheelAttachmentProperty(attachedStage, objectId, property, timeCode, value, wheelAttachment))
    {
        PxQuat valueQuat = toPhysX(value);
        return updateVehicleWheelAttachmentData(wheelAttachment, nullptr, nullptr, nullptr, &valueQuat,
            WheelAttachmentParam::eSUSPENSION_FRAME_ORIENTATION);
    }

    return true;
}

bool omni::physx::updateVehicleWheelAttachmentWheelFramePosition(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    GfVec3f value;
    InternalVehicleWheelAttachment* wheelAttachment;
    if (getWheelAttachmentProperty(attachedStage, objectId, property, timeCode, value, wheelAttachment))
    {
        PxVec3 pxValue = toPhysX(value);
        return updateVehicleWheelAttachmentData(wheelAttachment, &pxValue, nullptr, nullptr, nullptr,
            WheelAttachmentParam::eWHEEL_FRAME_POSITION);
    }

    return true;
}

bool omni::physx::updateVehicleWheelAttachmentWheelFrameOrientation(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    GfQuatf value;
    InternalVehicleWheelAttachment* wheelAttachment;
    if (getWheelAttachmentProperty(attachedStage, objectId, property, timeCode, value, wheelAttachment))
    {
        PxQuat valueQuat = toPhysX(value);
        return updateVehicleWheelAttachmentData(wheelAttachment, nullptr, nullptr, nullptr, &valueQuat,
            WheelAttachmentParam::eWHEEL_FRAME_ORIENTATION);
    }

    return true;
}

bool omni::physx::updateVehicleWheelAttachmentDriven(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    bool value;
    InternalVehicleWheelAttachment* wheelAttachment;
    if (getWheelAttachmentProperty(attachedStage, objectId, property, timeCode, value, wheelAttachment))
    {
        return updateVehicleWheelAttachmentData(wheelAttachment, nullptr, &value, nullptr, nullptr,
            WheelAttachmentParam::eDRIVEN);
    }

    return true;
}

bool omni::physx::updateVehicleWheelAttachmentCollisionGroup(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    pxr::SdfPath path;
    InternalVehicleWheelAttachment* wheelAttachment = getWheelAttachment(attachedStage, objectId, path);
    if (wheelAttachment)
    {
        UsdPrim wAttPrim = attachedStage.getStage()->GetPrimAtPath(path);
        const pxr::UsdRelationship& relationship = getRelationship(wAttPrim.GetProperty(property));

        SdfPathVector paths;
        relationship.GetTargets(&paths);
        const size_t targetCount = paths.size();
        if (targetCount == 1)
        {
            UsdPrim prim = wAttPrim.GetStage()->GetPrimAtPath(paths[0]);
            if (prim)
            {
                return updateVehicleWheelAttachmentData(wheelAttachment, nullptr, nullptr, &prim, nullptr,
                    WheelAttachmentParam::eCOLLISON_GROUP);
            }
            else
            {
                CARB_LOG_ERROR("Relationship \"%s\": no prim could be found at the target path \"%s\".\n",
                    relationship.GetPath().GetText(), paths[0].GetText());
            }
        }
        else if (targetCount == 0)
        {
            // the collision group is optional, thus it might get removed

            return updateVehicleWheelAttachmentData(wheelAttachment, nullptr, nullptr, nullptr, nullptr,
                WheelAttachmentParam::eCOLLISON_GROUP);
        }
        else
        {
            CARB_LOG_ERROR("Relationship \"%s\" can have at most 1 entry.\n",
                relationship.GetPath().GetText());
        }
    }

    return true;
}

template<typename TVecType>
static bool checkSuspensionComplianceValues(const TVecType* values, const uint32_t valueCount,
    const pxr::TfToken& property)
{
    if (valueCount > 3)
    {
        CARB_LOG_ERROR("Attribute \"%s\": max number of supported entries is 3.\n",
            property.GetText());

        return false;
    }

    float previousJounce = -1.0f;
    for (uint32_t i = 0; i < valueCount; i++)
    {
        const TVecType& value = values[i];

        if ((value[0] < 0.0f) || (value[0] > 1.0f))
        {
            CARB_LOG_ERROR("Attribute \"%s\": the first axis value of each entry needs to be in range [0, 1].\n",
                property.GetText());

            return false;
        }

        if (std::is_same<TVecType, GfVec2f>::value)
        {
            if (fabsf(value[1]) > static_cast<float>(M_PI_2))
            {
                CARB_LOG_ERROR("Attribute \"%s\": the second axis value of each entry needs to be in range [-pi, pi].\n",
                    property.GetText());

                return false;
            }
        }

        if (value[0] <= previousJounce)
        {
            CARB_LOG_ERROR("Attribute \"%s\": normalized jounce values "
                "have to be monotonically increasing.\n", property.GetText());

            return false;
        }

        previousJounce = value[0];
    }

    return true;
}

template<const uint32_t tAttribute>
static bool updateVehicleSuspensionComplianceValues(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    pxr::SdfPath path;
    InternalVehicleWheelAttachment* wheelAttachment = getWheelAttachment(attachedStage, objectId, path);
    if (wheelAttachment)
    {
        const pxr::UsdAttribute& attribute = getAttribute(attachedStage.getStage()->GetPrimAtPath(path).GetProperty(property));
        PhysXActorVehicleBase* vehicle = wheelAttachment->mVehicle->mPhysXVehicle;
        
        if ((tAttribute == SuspensionComplianceParam::eWHEEL_TOE_ANGLE) || (tAttribute == SuspensionComplianceParam::eWHEEL_CAMBER_ANGLE))
        {
            VtArray<GfVec2f> vecList;
            attribute.Get(&vecList);
            const GfVec2f* values = vecList.data();
            const uint32_t valueCount = static_cast<uint32_t>(vecList.size());

            if (checkSuspensionComplianceValues(values, valueCount, property))
            {
                if (tAttribute == SuspensionComplianceParam::eWHEEL_TOE_ANGLE)
                {
                    vehicle->setSuspensionComplianceWheelToeAngle(wheelAttachment->mWheelIndex,
                        values, valueCount);
                }
                else
                {
                    CARB_ASSERT(tAttribute == SuspensionComplianceParam::eWHEEL_CAMBER_ANGLE);

                    vehicle->setSuspensionComplianceWheelCamberAngle(wheelAttachment->mWheelIndex,
                        values, valueCount);
                }
            }
            else
                return false;
        }
        else
        {
            CARB_ASSERT((tAttribute == SuspensionComplianceParam::eSUSPENSION_FORCE_APP_POINT) || (tAttribute == SuspensionComplianceParam::eTIRE_FORCE_APP_POINT));
            VtArray<GfVec4f> vecList;
            attribute.Get(&vecList);
            const GfVec4f* values = vecList.data();
            const uint32_t valueCount = static_cast<uint32_t>(vecList.size());

            if (checkSuspensionComplianceValues(values, valueCount, property))
            {
                const PxVec3 scale = asPhysX(wheelAttachment->mVehicle->mScale);

                if (tAttribute == SuspensionComplianceParam::eSUSPENSION_FORCE_APP_POINT)
                {
                    vehicle->setSuspensionComplianceSuspensionForceAppPoint(wheelAttachment->mWheelIndex,
                        values, valueCount, scale);
                }
                else
                {
                    CARB_ASSERT(tAttribute == SuspensionComplianceParam::eTIRE_FORCE_APP_POINT);

                    vehicle->setSuspensionComplianceTireForceAppPoint(wheelAttachment->mWheelIndex,
                        values, valueCount, scale);
                }
            }
            else
                return false;
        }
    }

    return true;
}

bool omni::physx::updateVehicleSuspensionComplWheelToeAngle(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    updateVehicleSuspensionComplianceValues<SuspensionComplianceParam::eWHEEL_TOE_ANGLE>(attachedStage, objectId,
        property, timeCode);

    return true;
}

bool omni::physx::updateVehicleSuspensionComplWheelCamberAngle(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    updateVehicleSuspensionComplianceValues<SuspensionComplianceParam::eWHEEL_CAMBER_ANGLE>(attachedStage, objectId,
        property, timeCode);

    return true;
}

bool omni::physx::updateVehicleSuspensionComplSuspForceAppPoint(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    updateVehicleSuspensionComplianceValues<SuspensionComplianceParam::eSUSPENSION_FORCE_APP_POINT>(attachedStage, objectId,
        property, timeCode);

    return true;
}

bool omni::physx::updateVehicleSuspensionComplTireForceAppPoint(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    updateVehicleSuspensionComplianceValues<SuspensionComplianceParam::eTIRE_FORCE_APP_POINT>(attachedStage, objectId,
        property, timeCode);

    return true;
}

static InternalVehicle* getInternalVehicle(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    pxr::SdfPath& path)
{
    const InternalDatabase::Record* objectRecord = getObjectRecord(ePTVehicle, objectId);
    if (objectRecord)
    {
        path = objectRecord->mPath;
        return static_cast<InternalVehicle*>(objectRecord->mInternalPtr);
    }
    else
        return nullptr;
}

bool omni::physx::updateVehicleEnabled(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    pxr::SdfPath path;
    InternalVehicle* vehicle = getInternalVehicle(attachedStage, objectId, path);

    if (vehicle)
    {
        bool value;
        if (!getValue<bool>(attachedStage, path, property, timeCode, value))
            return true;

        vehicle->mInternalScene.setVehicleEnabledState(*vehicle, value);
    }

    return true;
}

bool omni::physx::updateVehicleLimitSuspensionExpansionVelocity(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    pxr::SdfPath path;
    InternalVehicle* vehicle = getInternalVehicle(attachedStage, objectId, path);

    if (vehicle)
    {
        bool value;
        if (!getValue<bool>(attachedStage, path, property, timeCode, value))
            return true;

        vehicle->mPhysXVehicle->setLimitSuspensionExpansionVelocity(value);
    }

    return true;
}

static bool updateVehicleFloatVal(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode,
    const VehicleParam::Enum paramId)
{
    pxr::SdfPath path;
    InternalVehicle* vehicle = getInternalVehicle(attachedStage, objectId, path);

    if (vehicle)
    {
        float value;
        if (!getValue<float>(attachedStage, path, property, timeCode, value))
            return true;

        switch (paramId)
        {
            case VehicleParam::eTIRE_MIN_PASSIVE_LONG_SLIP_DENOM:
            {
                if (value > 0.0f)
                    vehicle->mPhysXVehicle->setTireMinPassiveLongitudinalSlipDenominator(value);
                else
                    attrPositiveErrLog(property);
            }
            break;

            case VehicleParam::eTIRE_MIN_ACTIVE_LONG_SLIP_DENOM:
            {
                if (value > 0.0f)
                    vehicle->mPhysXVehicle->setTireMinActiveLongitudinalSlipDenominator(value);
                else
                    attrPositiveErrLog(property);
            }
            break;

            case VehicleParam::eTIRE_MIN_LAT_SLIP_DENOM:
            {
                if (value > 0.0f)
                    vehicle->mPhysXVehicle->setTireMinLateralSlipDenominator(value);
                else
                    attrPositiveErrLog(property);
            }
            break;

            case VehicleParam::eTIRE_LONG_STICKY_THRESHOLD_SPEED:
            {
                if (value >= 0.0f)
                    vehicle->mPhysXVehicle->setTireLongitudinalStickyThresholdSpeed(value);
                else
                    attrNonNegativeErrLog(property);
            }
            break;

            case VehicleParam::eTIRE_LONG_STICKY_THRESHOLD_TIME:
            {
                if (value >= 0.0f)
                    vehicle->mPhysXVehicle->setTireLongitudinalStickyThresholdTime(value);
                else
                    attrNonNegativeErrLog(property);
            }
            break;

            case VehicleParam::eTIRE_LONG_STICKY_DAMPING:
            {
                if (value >= 0.0f)
                    vehicle->mPhysXVehicle->setTireLongitudinalStickyDamping(value);
                else
                    attrNonNegativeErrLog(property);
            }
            break;

            case VehicleParam::eTIRE_LAT_STICKY_THRESHOLD_SPEED:
            {
                if (value >= 0.0f)
                    vehicle->mPhysXVehicle->setTireLateralStickyThresholdSpeed(value);
                else
                    attrNonNegativeErrLog(property);
            }
            break;

            case VehicleParam::eTIRE_LAT_STICKY_THRESHOLD_TIME:
            {
                if (value >= 0.0f)
                    vehicle->mPhysXVehicle->setTireLateralStickyThresholdTime(value);
                else
                    attrNonNegativeErrLog(property);
            }
            break;

            case VehicleParam::eTIRE_LAT_STICKY_DAMPING:
            {
                if (value >= 0.0f)
                    vehicle->mPhysXVehicle->setTireLateralStickyDamping(value);
                else
                    attrNonNegativeErrLog(property);
            }
            break;
        }
    }

    return true;
}

bool omni::physx::updateVehicleMinPassiveLongslipDenom(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleFloatVal(attachedStage, objectId, property, timeCode, VehicleParam::eTIRE_MIN_PASSIVE_LONG_SLIP_DENOM);
}

bool omni::physx::updateVehicleMinActiveLongslipDenom(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleFloatVal(attachedStage, objectId, property, timeCode, VehicleParam::eTIRE_MIN_ACTIVE_LONG_SLIP_DENOM);
}

bool omni::physx::updateVehicleMinLateralSlipDenom(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleFloatVal(attachedStage, objectId, property, timeCode, VehicleParam::eTIRE_MIN_LAT_SLIP_DENOM);
}

bool omni::physx::updateVehicleLongitudinalStickyTireThresholdSpeed(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleFloatVal(attachedStage, objectId, property, timeCode, VehicleParam::eTIRE_LONG_STICKY_THRESHOLD_SPEED);
}

bool omni::physx::updateVehicleLongitudinalStickyTireThresholdTime(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleFloatVal(attachedStage, objectId, property, timeCode, VehicleParam::eTIRE_LONG_STICKY_THRESHOLD_TIME);
}

bool omni::physx::updateVehicleLongitudinalStickyTireDamping(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleFloatVal(attachedStage, objectId, property, timeCode, VehicleParam::eTIRE_LONG_STICKY_DAMPING);
}

bool omni::physx::updateVehicleLateralStickyTireThresholdSpeed(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleFloatVal(attachedStage, objectId, property, timeCode, VehicleParam::eTIRE_LAT_STICKY_THRESHOLD_SPEED);
}

bool omni::physx::updateVehicleLateralStickyTireThresholdTime(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleFloatVal(attachedStage, objectId, property, timeCode, VehicleParam::eTIRE_LAT_STICKY_THRESHOLD_TIME);
}

bool omni::physx::updateVehicleLateralStickyTireDamping(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleFloatVal(attachedStage, objectId, property, timeCode, VehicleParam::eTIRE_LAT_STICKY_DAMPING);
}

static InternalVehicle* getInternalVehicleFromController(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    pxr::SdfPath& path)
{
    const InternalDatabase::Record* objectRecord = getObjectRecord(ePTVehicleController, objectId);
    if (objectRecord)
    {
        path = objectRecord->mPath;
        return static_cast<InternalVehicle*>(objectRecord->mInternalPtr);
    }
    else
        return nullptr;
}

static bool updateVehicleControllerData(InternalVehicle* vehicle,
    const float* valueFloat, const int* valueInt,
    const VehicleControllerParam::Enum vehicleControllerParam)
{
    switch (vehicleControllerParam)
    {
    case VehicleControllerParam::eACCELERATOR:
    {
        vehicle->setAccelerator(*valueFloat);
    }
    break;

    case VehicleControllerParam::eBRAKE0:
    {
        vehicle->setBrake(0, *valueFloat);
    }
    break;

    case VehicleControllerParam::eBRAKE1:
    {
        vehicle->setBrake(1, *valueFloat);
    }
    break;

    case VehicleControllerParam::eBRAKE:
    {
        vehicle->setBrake(*valueFloat);
    }
    break;

    case VehicleControllerParam::eHANDBRAKE:
    {
        vehicle->setHandbrake(*valueFloat);
    }
    break;

    case VehicleControllerParam::eSTEER:
    {
        vehicle->setSteer(*valueFloat);
    }
    break;

    case VehicleControllerParam::eSTEER_LEFT:
    {
        vehicle->setSteerLeft(*valueFloat);
    }
    break;

    case VehicleControllerParam::eSTEER_RIGHT:
    {
        vehicle->setSteerRight(*valueFloat);
    }
    break;

    case VehicleControllerParam::eTARGET_GEAR:
    {
        vehicle->setTargetGear(*valueInt);
    }
    break;
    }

    return true;
}

static bool updateVehicleControllerFloatVal(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode,
    const VehicleControllerParam::Enum paramId,
    const float min = 0.0f, const float max = 1.0f)
{
    pxr::SdfPath path;
    InternalVehicle* vehicle = getInternalVehicleFromController(attachedStage, objectId, path);

    if (vehicle)
    {
        float value;
        if (!getValue<float>(attachedStage, path, property, timeCode, value))
            return true;

        if (!attrRangeCheck(value, min, max, property))
            return true;

        return updateVehicleControllerData(vehicle, &value, nullptr, paramId);
    }

    return true;
}

bool omni::physx::updateVehicleControllerAccelerator(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleControllerFloatVal(attachedStage, objectId, property, timeCode, VehicleControllerParam::eACCELERATOR);
}

bool omni::physx::updateVehicleControllerBrake0(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleControllerFloatVal(attachedStage, objectId, property, timeCode, VehicleControllerParam::eBRAKE0);
}

bool omni::physx::updateVehicleControllerBrake1(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleControllerFloatVal(attachedStage, objectId, property, timeCode, VehicleControllerParam::eBRAKE1);
}

bool omni::physx::updateVehicleControllerBrake(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleControllerFloatVal(attachedStage, objectId, property, timeCode, VehicleControllerParam::eBRAKE);
}

bool omni::physx::updateVehicleControllerHandbrake(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleControllerFloatVal(attachedStage, objectId, property, timeCode, VehicleControllerParam::eHANDBRAKE);
}

bool omni::physx::updateVehicleControllerSteer(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleControllerFloatVal(attachedStage, objectId, property, timeCode, VehicleControllerParam::eSTEER,
        -1.0f, 1.0f);
}

bool omni::physx::updateVehicleControllerSteerLeft(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleControllerFloatVal(attachedStage, objectId, property, timeCode, VehicleControllerParam::eSTEER_LEFT);
}

bool omni::physx::updateVehicleControllerSteerRight(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleControllerFloatVal(attachedStage, objectId, property, timeCode, VehicleControllerParam::eSTEER_RIGHT);
}

bool omni::physx::updateVehicleControllerTargetGear(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    pxr::SdfPath path;
    InternalVehicle* vehicle = getInternalVehicleFromController(attachedStage, objectId, path);

    if (vehicle)
    {
        int value;
        if (!getValue<int>(attachedStage, path, property, timeCode, value))
            return true;

        constexpr int highestGear = ::physx::vehicle2::PxVehicleGearboxParams::eMAX_NB_GEARS - 2;
        constexpr int automaticGear = ::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR;
        if ((value != automaticGear) &&
            ((value < -1) || (value > (highestGear))))
        {
            CARB_LOG_ERROR("Attribute \"%s\" has to be in [-1, %d] or the special value %d\n", property.GetText(),
                highestGear, automaticGear);
            return true;
        }

        return updateVehicleControllerData(vehicle, nullptr, &value, VehicleControllerParam::eTARGET_GEAR);
    }

    return true;
}

static bool updateVehicleTankControllerFloatVal(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode,
    const VehicleTankControllerParam::Enum paramId)
{
    pxr::SdfPath path;
    InternalVehicle* vehicle = getInternalVehicleFromController(attachedStage, objectId, path);

    if (vehicle)
    {
        float value;
        if (!getValue<float>(attachedStage, path, property, timeCode, value))
            return true;

        if ((value < -1.0f) || (value > 1.0f))
        {
            CARB_LOG_ERROR("Attribute \"%s\" has to be in [-1, 1]\n", property.GetText());
            return true;
        }

        if (paramId == VehicleTankControllerParam::eTHRUST0)
        {
            vehicle->setThrust(0, value);
        }
        else if (paramId == VehicleTankControllerParam::eTHRUST1)
        {
            vehicle->setThrust(1, value);
        }
    }

    return true;
}

bool omni::physx::updateVehicleTankControllerThrust0(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleTankControllerFloatVal(attachedStage, objectId, property, timeCode,
        VehicleTankControllerParam::eTHRUST0);
}

bool omni::physx::updateVehicleTankControllerThrust1(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleTankControllerFloatVal(attachedStage, objectId, property, timeCode,
        VehicleTankControllerParam::eTHRUST1);
}

bool omni::physx::updateVehicleDriveBasicPeakTorque(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const InternalDatabase::Record* objectRecord = getObjectRecord(ePTVehicleDriveBasic, objectId);
    if (!objectRecord)
        return true;

    float value;
    if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, value))
        return true;

    if (value < 0.0f)
    {
        attrNonNegativeErrLog(property);
        return true;
    }

    InternalVehicleReferenceList* vehicleRefList = static_cast<InternalVehicleReferenceList*>(objectRecord->mInternalPtr);

    for (InternalVehicle* vehicle : vehicleRefList->mVehicles)
    {
        CARB_ASSERT(vehicle->mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE);
        PhysXVehicleDirectDrive* physxVehicle = static_cast<PhysXVehicleDirectDrive*>(vehicle->mPhysXVehicle);
        physxVehicle->setPeakDriveTorque(value);
    }

    return true;
}

static bool updateVehicleWheelControllerData(InternalVehicleWheelAttachment* wheelAttachment,
    const float value, const VehicleWheelControllerParam::Enum vehicleWheelControllerParam)
{
    switch (vehicleWheelControllerParam)
    {
    case VehicleWheelControllerParam::eDRIVE_TORQUE:
    {
        wheelAttachment->setDriveTorque(value);
    }
    break;

    case VehicleWheelControllerParam::eBRAKE_TORQUE:
    {
        wheelAttachment->setBrakeTorque(value);
    }
    break;

    case VehicleWheelControllerParam::eSTEER_ANGLE:
    {
        wheelAttachment->setSteerAngle(value);
    }
    break;
    }

    return true;
}

template<typename T>
static bool getWheelControllerProperty(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode,
    T& value, InternalVehicleWheelAttachment*& wheelAttachment)
{
    const InternalDatabase::Record* objectRecord = getObjectRecord(ePTVehicleWheelController, objectId);
    if (!objectRecord)
        return false;

    if (!getValue<T>(attachedStage, objectRecord->mPath, property, timeCode, value))
        return false;

    wheelAttachment = static_cast<InternalVehicleWheelAttachment*>(objectRecord->mInternalPtr);

    return true;
}

bool omni::physx::updateVehicleWheelControllerDriveTorque(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelAttachment* wheelAttachment;
    if (getWheelControllerProperty<float>(attachedStage, objectId, property, timeCode, value, wheelAttachment))
    {
        return updateVehicleWheelControllerData(wheelAttachment, value, VehicleWheelControllerParam::eDRIVE_TORQUE);
    }

    return true;
}

bool omni::physx::updateVehicleWheelControllerBrakeTorque(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelAttachment* wheelAttachment;
    if (getWheelControllerProperty<float>(attachedStage, objectId, property, timeCode, value, wheelAttachment))
    {
        if (value < 0.0f)
        {
            CARB_LOG_ERROR("Attribute \"%s\" must not be negative\n", property.GetText());
            return true;
        }

        return updateVehicleWheelControllerData(wheelAttachment, value, VehicleWheelControllerParam::eBRAKE_TORQUE);
    }

    return true;
}

bool omni::physx::updateVehicleWheelControllerSteerAngle(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    float value;
    InternalVehicleWheelAttachment* wheelAttachment;
    if (getWheelControllerProperty<float>(attachedStage, objectId, property, timeCode, value, wheelAttachment))
    {
        return updateVehicleWheelControllerData(wheelAttachment, value, VehicleWheelControllerParam::eSTEER_ANGLE);
    }

    return true;
}

bool omni::physx::updateVehicleMultiWheelDifferentialWheels(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

static void sendArraySizeMismatchError(const pxr::TfToken& property)
{
    CARB_LOG_ERROR("Attribute \"%s\": modifying the size of the array is not allowed once the simulation has been started. Changes will be ignored.\n",
        property.GetText());
}

bool omni::physx::updateVehicleMultiWheelDifferentialTorqueRatios(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    pxr::SdfPath path;
    InternalVehicle* vehicle = getInternalVehicle(attachedStage, objectId, path);

    if (vehicle)
    {
        const pxr::UsdAttribute& attribute = getAttribute(attachedStage.getStage()->GetPrimAtPath(path).GetProperty(property));
        VtArray<float> torqueRatios;
        attribute.Get(&torqueRatios);
        const float* values = torqueRatios.data();
        const uint32_t valueCount = static_cast<uint32_t>(torqueRatios.size());

        for (uint32_t i = 0; i < valueCount; i++)
        {
            const float v = values[i];
            if (!attrRangeCheck(v, -1.0f, 1.0f, property))
                return true;
        }

        if (vehicle->mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE)
        {
            PhysXVehicleEngineDrive* physxVehicle = static_cast<PhysXVehicleEngineDrive*>(vehicle->mPhysXVehicle);
            if (!physxVehicle->setDifferentialTorqueRatios(values, valueCount))
            {
                sendArraySizeMismatchError(property);
            }
        }
        else
        {
            CARB_ASSERT(vehicle->mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE);
            // else the vehicle creation should have failed already

            PhysXVehicleDirectDrive* physxVehicle = static_cast<PhysXVehicleDirectDrive*>(vehicle->mPhysXVehicle);
            if (!physxVehicle->setDifferentialTorqueRatios(values, valueCount))
            {
                sendArraySizeMismatchError(property);
            }
        }
    }

    return true;
}

bool omni::physx::updateVehicleMultiWheelDifferentialAverageWheelSpeedRatios(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    pxr::SdfPath path;
    InternalVehicle* vehicle = getInternalVehicle(attachedStage, objectId, path);

    if (vehicle)
    {
        const pxr::UsdAttribute& attribute = getAttribute(attachedStage.getStage()->GetPrimAtPath(path).GetProperty(property));
        VtArray<float> avgWheelSpeedRatios;
        attribute.Get(&avgWheelSpeedRatios);
        const float* values = avgWheelSpeedRatios.data();
        const uint32_t valueCount = static_cast<uint32_t>(avgWheelSpeedRatios.size());

        for (uint32_t i = 0; i < valueCount; i++)
        {
            const float v = values[i];
            if (!attrRangeCheck(v, 0.0f, 1.0f, property))
                return true;
        }

        if (vehicle->mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE)  // direct drive does not support this attribute
        {
            PhysXVehicleEngineDrive* physxVehicle = static_cast<PhysXVehicleEngineDrive*>(vehicle->mPhysXVehicle);
            if (!physxVehicle->setDifferentialAverageWheelSpeedRatios(values, valueCount))
            {
                sendArraySizeMismatchError(property);
            }
        }
    }

    return true;
}

bool omni::physx::updateVehicleTankDifferentialNumberOfWheelsPerTrack(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleTankDifferentialThrustIndexPerTrack(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleTankDifferentialTrackToWheelIndices(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleTankDifferentialWheelIndicesInTrackOrder(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

static bool updateVehicleBrakesWheels(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode&)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleBrakes0Wheels(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleBrakesWheels(attachedStage, objectId, property, timeCode);
}

bool omni::physx::updateVehicleBrakes1Wheels(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleBrakesWheels(attachedStage, objectId, property, timeCode);
}

static bool updateVehicleBrakesMaxBrakeTorque(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode, uint32_t brakesIndex)
{
    pxr::SdfPath path;
    InternalVehicle* vehicle = getInternalVehicle(attachedStage, objectId, path);

    if (vehicle)
    {
        float value;
        if (!getValue<float>(attachedStage, path, property, timeCode, value))
            return true;

        if (value < 0.0f)
        {
            attrNonNegativeErrLog(property);
            return true;
        }

        CARB_ASSERT((vehicle->mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE) ||
            (vehicle->mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE));

        PhysXVehicleManagedWheelControl* physxVehicleMWC = static_cast<PhysXVehicleManagedWheelControl*>(vehicle->mPhysXVehicle);
        physxVehicleMWC->setMaxBrakeTorque(brakesIndex, value);
    }

    return true;
}

bool omni::physx::updateVehicleBrakes0MaxBrakeTorque(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleBrakesMaxBrakeTorque(attachedStage, objectId, property, timeCode, 0);
}

bool omni::physx::updateVehicleBrakes1MaxBrakeTorque(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleBrakesMaxBrakeTorque(attachedStage, objectId, property, timeCode, 1);
}

static bool updateVehicleBrakesTorqueMultipliers(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode, uint32_t brakesIndex)
{
    pxr::SdfPath path;
    InternalVehicle* vehicle = getInternalVehicle(attachedStage, objectId, path);

    if (vehicle)
    {
        const pxr::UsdAttribute& attribute = getAttribute(attachedStage.getStage()->GetPrimAtPath(path).GetProperty(property));
        VtArray<float> brakeTorqueMultipliers;
        attribute.Get(&brakeTorqueMultipliers);
        const float* values = brakeTorqueMultipliers.data();
        const uint32_t valueCount = static_cast<uint32_t>(brakeTorqueMultipliers.size());

        for (uint32_t i = 0; i < valueCount; i++)
        {
            const float v = values[i];
            if (v < 0.0f)
            {
                attrNonNegativeErrLog(property);
                return true;
            }
        }

        CARB_ASSERT((vehicle->mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE) ||
            (vehicle->mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE));

        PhysXVehicleManagedWheelControl* physxVehicleMWC = static_cast<PhysXVehicleManagedWheelControl*>(vehicle->mPhysXVehicle);
        if (!physxVehicleMWC->setBrakeTorqueMultipliers(brakesIndex, values, valueCount))
        {
            sendArraySizeMismatchError(property);
        }
    }

    return true;
}

bool omni::physx::updateVehicleBrakes0TorqueMultipliers(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleBrakesTorqueMultipliers(attachedStage, objectId, property, timeCode, 0); 
}

bool omni::physx::updateVehicleBrakes1TorqueMultipliers(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleBrakesTorqueMultipliers(attachedStage, objectId, property, timeCode, 1); 
}

bool omni::physx::updateVehicleSteeringWheels(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

static bool maxSteerAngleRangeCheck(const float value, const pxr::TfToken& property)
{
    if (PxAbs(value) <= PxPi)
    {
        return true;
    }
    else
    {
        CARB_LOG_ERROR("Attribute \"%s\": maxSteerAngle * angleMultiplier has to in range [-pi, pi] for all steered wheels. "
            "Changes will be ignored.\n",
            property.GetText());

        return false;
    }
}

bool omni::physx::updateVehicleSteeringMaxSteerAngle(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    pxr::SdfPath path;
    InternalVehicle* vehicle = getInternalVehicle(attachedStage, objectId, path);

    if (vehicle)
    {
        float value;
        if (!getValue<float>(attachedStage, path, property, timeCode, value))
            return true;

        CARB_ASSERT((vehicle->mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE) ||
            (vehicle->mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE));

        PhysXVehicleManagedWheelControl* physxVehicleMWC = static_cast<PhysXVehicleManagedWheelControl*>(vehicle->mPhysXVehicle);

        const uint8_t* indices;
        const float* angleMultipliers;
        uint32_t count = physxVehicleMWC->getSteerAngleMultipliers(indices, angleMultipliers);

        for (uint32_t i = 0; i < count; i++)
        {
            const float angleMult = angleMultipliers[indices[i]];
            if (!maxSteerAngleRangeCheck(angleMult * value, property))
                return true;
        }

        physxVehicleMWC->setMaxSteerAngle(value);
    }

    return true;
}

bool omni::physx::updateVehicleSteeringAngleMultipliers(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    pxr::SdfPath path;
    InternalVehicle* vehicle = getInternalVehicle(attachedStage, objectId, path);

    if (vehicle)
    {
        const pxr::UsdAttribute& attribute = getAttribute(attachedStage.getStage()->GetPrimAtPath(path).GetProperty(property));
        VtArray<float> steerAngleMultipliers;
        attribute.Get(&steerAngleMultipliers);
        const float* values = steerAngleMultipliers.data();
        const uint32_t valueCount = static_cast<uint32_t>(steerAngleMultipliers.size());

        CARB_ASSERT((vehicle->mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE) ||
            (vehicle->mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE));

        PhysXVehicleManagedWheelControl* physxVehicleMWC = static_cast<PhysXVehicleManagedWheelControl*>(vehicle->mPhysXVehicle);

        const float maxSteerAngle = physxVehicleMWC->getMaxSteerAngle();
        for (uint32_t i = 0; i < valueCount; i++)
        {
            const float v = values[i];
            if (!maxSteerAngleRangeCheck(v * maxSteerAngle, property))
                return true;
        }

        if (!physxVehicleMWC->setSteerAngleMultipliers(values, valueCount))
        {
            sendArraySizeMismatchError(property);
        }
    }

    return true;
}

static bool updateVehicleAckermannSteeringFloatVal(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode,
    const VehicleAckermannParam::Enum paramId)
{
    pxr::SdfPath path;
    InternalVehicle* vehicle = getInternalVehicle(attachedStage, objectId, path);

    if (vehicle)
    {
        float value;
        if (!getValue<float>(attachedStage, path, property, timeCode, value))
            return true;

        CARB_ASSERT((vehicle->mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE) ||
            (vehicle->mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE));

        PhysXVehicleManagedWheelControl* physxVehicleMWC = static_cast<PhysXVehicleManagedWheelControl*>(vehicle->mPhysXVehicle);

        if (!physxVehicleMWC->isUsingAckermannSteering())
        {
            // users might add the PhysxVehicleAckermannSteeringAPI after the simulation has started and then
            // set corresponding attributes. We do not support this, thus have to check whether Ackermann is
            // enabled before processing changes.

            CARB_LOG_ERROR("Attribute \"%s\" at prim \"%s\": the vehicle has not been configured for Ackermann steering. "
                "Note that adding the PhysxVehicleAckermannSteeringAPI after the simulation has started is not supported.\n",
                property.GetText(), path.GetText());

            return true;
        }

        switch (paramId)
        {
            case VehicleAckermannParam::eMAX_STEER_ANGLE:
            {
                if (!maxSteerAngleRangeCheck(value, property))
                    return true;

                physxVehicleMWC->setMaxSteerAngle(value);
            }
            break;

            case VehicleAckermannParam::eWHEEL_BASE:
            {
                if (value <= 0.0f)
                {
                    attrPositiveErrLog(property);
                    return true;
                }

                physxVehicleMWC->setAckermannWheelBase(value);
            }
            break;

            case VehicleAckermannParam::eTRACK_WIDTH:
            {
                if (value <= 0.0f)
                {
                    attrPositiveErrLog(property);
                    return true;
                }

                physxVehicleMWC->setAckermannTrackWidth(value);
            }
            break;

            case VehicleAckermannParam::eSTRENGTH:
            {
                if (!attrRangeCheck(value, 0.0f, 1.0f, property))
                    return true;

                physxVehicleMWC->setAckermannStrength(value);
            }
            break;
        }
    }

    return true;
}

bool omni::physx::updateVehicleAckermannSteeringWheel0(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleAckermannSteeringWheel1(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleAckermannSteeringMaxSteerAngle(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleAckermannSteeringFloatVal(attachedStage, objectId, property, timeCode,
        VehicleAckermannParam::eMAX_STEER_ANGLE);
}

bool omni::physx::updateVehicleAckermannSteeringWheelBase(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleAckermannSteeringFloatVal(attachedStage, objectId, property, timeCode,
        VehicleAckermannParam::eWHEEL_BASE);
}

bool omni::physx::updateVehicleAckermannSteeringTrackWidth(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleAckermannSteeringFloatVal(attachedStage, objectId, property, timeCode,
        VehicleAckermannParam::eTRACK_WIDTH);
}

bool omni::physx::updateVehicleAckermannSteeringStrength(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    return updateVehicleAckermannSteeringFloatVal(attachedStage, objectId, property, timeCode,
        VehicleAckermannParam::eSTRENGTH);
}

bool omni::physx::updateVehicleNCRDriveCommandValues(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleNCRSteerCommandValues(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleNCRBrakes0CommandValues(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleNCRBrakes1CommandValues(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleNCRDriveSpeedResponsesPerCommandValue(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleNCRSteerSpeedResponsesPerCommandValue(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleNCRBrakes0SpeedResponsesPerCommandValue(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleNCRBrakes1SpeedResponsesPerCommandValue(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleNCRDriveSpeedResponses(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleNCRSteerSpeedResponses(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleNCRBrakes0SpeedResponses(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}

bool omni::physx::updateVehicleNCRBrakes1SpeedResponses(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    attrNoModDuringSimErrLog(property);

    return true;
}
