// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "InternalTools.h"

#include <carb/logging/Log.h>
#include <common/utilities/MemoryMacros.h>
#include <PhysXDefines.h>
#include <PhysXTools.h>

using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace pxr;
using namespace carb;
using namespace ::physx;


static ::physx::vehicle2::PxVehicleAxes::Enum getAxisEnum(const PxVec3& axis)
{
    ::physx::vehicle2::PxVehicleAxes::Enum axisEnum;
    const float absX = PxAbs(axis.x);
    const float absY = PxAbs(axis.y);
    const float absZ = PxAbs(axis.z);
    float currentMax;
    if (absY > absZ)
    {
        axisEnum = (axis.y >= 0.0f) ? ::physx::vehicle2::PxVehicleAxes::ePosY : ::physx::vehicle2::PxVehicleAxes::eNegY;
        currentMax = absY;
    }
    else
    {
        axisEnum = (axis.z >= 0.0f) ? ::physx::vehicle2::PxVehicleAxes::ePosZ : ::physx::vehicle2::PxVehicleAxes::eNegZ;
        currentMax = absZ;
    }

    if (currentMax > absX)
        return axisEnum;
    else
    {
        axisEnum = (axis.x >= 0.0f) ? ::physx::vehicle2::PxVehicleAxes::ePosX : ::physx::vehicle2::PxVehicleAxes::eNegX;
        return axisEnum;
    }
}

void InternalVehicleContext::init(const VehicleContextDesc& contextDesc, ::physx::PxScene& pxScene)
{
    // note: after this call the context is not yet ready for being used in the simulation
    //       but the frame can be queried already, for example.

    if (contextDesc.vehicleUpdateMode == VehicleUpdateMode::eVelocityChange)
        mPhysXContext.physxActorUpdateMode = ::physx::vehicle2::PxVehiclePhysXActorUpdateMode::eAPPLY_VELOCITY;
    else
        mPhysXContext.physxActorUpdateMode = ::physx::vehicle2::PxVehiclePhysXActorUpdateMode::eAPPLY_ACCELERATION;

    // to ensure the code further below stays valid
    static_assert(VehicleContextDesc::ePosX == 0, "");
    static_assert(VehicleContextDesc::eNegX == 1, "");
    static_assert(VehicleContextDesc::ePosY == 2, "");
    static_assert(VehicleContextDesc::eNegY == 3, "");
    static_assert(VehicleContextDesc::ePosZ == 4, "");
    static_assert(VehicleContextDesc::eNegZ == 5, "");
    static_assert(VehicleContextDesc::ePosX == ::physx::vehicle2::PxVehicleAxes::ePosX, "");
    static_assert(VehicleContextDesc::eNegX == ::physx::vehicle2::PxVehicleAxes::eNegX, "");
    static_assert(VehicleContextDesc::ePosY == ::physx::vehicle2::PxVehicleAxes::ePosY, "");
    static_assert(VehicleContextDesc::eNegY == ::physx::vehicle2::PxVehicleAxes::eNegY, "");
    static_assert(VehicleContextDesc::ePosZ == ::physx::vehicle2::PxVehicleAxes::ePosZ, "");
    static_assert(VehicleContextDesc::eNegZ == ::physx::vehicle2::PxVehicleAxes::eNegZ, "");

    static const ::physx::PxVec3 axisToVec3Map[] = {
        ::physx::PxVec3(1.0, 0.0, 0.0),
        ::physx::PxVec3(-1.0, 0.0, 0.0),
        ::physx::PxVec3(0.0, 1.0, 0.0),
        ::physx::PxVec3(0.0, -1.0, 0.0),
        ::physx::PxVec3(0.0, 0.0, 1.0),
        ::physx::PxVec3(0.0, 0.0, -1.0)
    };

    ::physx::PxVec3 upAxis;
    if (contextDesc.verticalAxis != VehicleContextDesc::eUndefined)
    {
        mPhysXContext.frame.vrtAxis = static_cast<::physx::vehicle2::PxVehicleAxes::Enum>(contextDesc.verticalAxis);
        upAxis = axisToVec3Map[contextDesc.verticalAxis];
    }
    else
    {
        upAxis = toPhysX(contextDesc.upAxis);
        mPhysXContext.frame.vrtAxis = getAxisEnum(upAxis);
    }

    ::physx::PxVec3 forwardAxis;
    if (contextDesc.longitudinalAxis != VehicleContextDesc::eUndefined)
    {
        mPhysXContext.frame.lngAxis = static_cast<::physx::vehicle2::PxVehicleAxes::Enum>(contextDesc.longitudinalAxis);
        forwardAxis = axisToVec3Map[contextDesc.longitudinalAxis];
    }
    else
    {
        forwardAxis = toPhysX(contextDesc.forwardAxis);
        mPhysXContext.frame.lngAxis = getAxisEnum(forwardAxis);
    }

    const ::physx::PxVec3 sideAxis = upAxis.cross(forwardAxis);
    mPhysXContext.frame.latAxis = getAxisEnum(sideAxis);

    const float lengthScale = pxScene.getPhysics().getTolerancesScale().length;
    mPhysXContext.scale.scale = lengthScale;
    mPhysXContext.physxScene = &pxScene;

    mPhysXContext.thresholdForwardSpeedForWheelAngleIntegration = 5.0f * lengthScale;

    // for these the default values are used
    //mPhysXContext.physxActorWakeCounterResetValue
    //mPhysXContext.physxActorWakeCounterThreshold

    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();
    mPhysXContext.pvdContext.attributeHandles = physxSetup.getVehiclePvdRegistrationHandles();
    mPhysXContext.pvdContext.writer = physxSetup.getOmniPvd() ? physxSetup.getOmniPvd()->getWriter() : nullptr;
}


InternalVehicle::~InternalVehicle()
{
    if (mPhysXVehicle)
    {
        OmniPvdWriter* pvdWriter;
        ::physx::PxAllocatorCallback* allocatorForPvd;
        
        if (mPhysXVehicle->getPvdObjectHandles())
        {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();

            // the following should hold when getPvdObjectHandles() is available
            CARB_ASSERT(physxSetup.getVehiclePvdRegistrationHandles());
            CARB_ASSERT(physxSetup.getOmniPvd());
            CARB_ASSERT(physxSetup.getOmniPvd()->getWriter());

            pvdWriter = physxSetup.getOmniPvd()->getWriter();
            allocatorForPvd = &physxSetup.getAllocator();
        }
        else
        {
            pvdWriter = nullptr;
            allocatorForPvd = nullptr;
        }

        mPhysXVehicle->release(pvdWriter, allocatorForPvd);
    }

    if (mEngineOrDriveBasic)
        mEngineOrDriveBasic->removeVehicle(*this);

    if (mInitialControllerValues)
    {
        ICE_FREE_BASIC(mInitialControllerValues);
    }

    removeFromWheelComponents();

    for (InternalVehicleWheelAttachment* wheelAttachment : mWheelAttachments)
    {
        if (wheelAttachment)  // a removed wheel attachment might set the entry to nullptr
            wheelAttachment->mVehicle = nullptr;
    }
}

void InternalVehicle::WheelTransformManagementEntry::init(pxr::UsdPrim& wheelRootPrim_,
                                                          pxr::UsdPrim& shapePrim_,
                                                          const PxShape* shape_)
{
    wheelRootPrim = wheelRootPrim_;
    shapePrim = shapePrim_;
    shape = shape_;

    // sanitize xform ops to scale orient translate
    setupTransformOpsAsScaleOrientTranslate(wheelRootPrim_);
    pxr::GfVec3f sc(0.f);
    const bool success = getScaleFromXformOp<pxr::GfVec3f>(sc, wheelRootPrim_);
    if (success)
    {
        scale.x = sc[0];
        scale.y = sc[1];
        scale.z = sc[2];
    }
    else
    {
        scale.x = 1.0f;
        scale.y = 1.0f;
        scale.z = 1.0f;

        CARB_LOG_ERROR("PhysX Vehicle: wheel attachment prim transform has no scale (\"%s\"). Make sure the transform is writable.\n",
            wheelRootPrim_.GetPath().GetText());
    }

    getParentXform(wheelRootPrim_, wheelRootParentXformPrim);

    if (shape_ && (wheelRootPrim_ != shapePrim_))
    {
        // sanitize xform ops to scale orient translate
        setupTransformOpsAsScaleOrientTranslate(shapePrim_);
        pxr::GfVec3f shapeSc(0.f);
        const bool success = getScaleFromXformOp<pxr::GfVec3f>(shapeSc, shapePrim_);
        if (success)
        {
            shapeScale.x = shapeSc[0];
            shapeScale.y = shapeSc[1];
            shapeScale.z = shapeSc[2];
        }
        else
        {
            shapeScale.x = 1.0f;
            shapeScale.y = 1.0f;
            shapeScale.z = 1.0f;

            CARB_LOG_ERROR("PhysX Vehicle: wheel attachment collision prim transform has no scale (\"%s\"). Make sure the transform is writable.\n",
                shapePrim_.GetPath().GetText());
        }
    }
}

void InternalVehicle::restoreInitialProperties()
{
    if (mInitialControllerValues)
    {
        CARB_ASSERT((mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE) ||
        (mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE));

        PhysXVehicleManagedWheelControl* mwcVehicle = static_cast<PhysXVehicleManagedWheelControl*>(mPhysXVehicle);

        CARB_ASSERT(mInitialControllerValues->prim.HasAPI<pxr::PhysxSchemaPhysxVehicleControllerAPI>());
        pxr::PhysxSchemaPhysxVehicleControllerAPI controllerAPI(mInitialControllerValues->prim);
        const bool hasTankController = mInitialControllerValues->prim.HasAPI<pxr::PhysxSchemaPhysxVehicleTankControllerAPI>();
        controllerAPI.GetAcceleratorAttr().Set(mInitialControllerValues->accelerator);
        if (!mwcVehicle->isUsingDeprecatedBrakesSetup())
        {
            controllerAPI.GetBrake0Attr().Set(mInitialControllerValues->brake0);
            controllerAPI.GetBrake1Attr().Set(mInitialControllerValues->brake1);
        }
        else
        {
            controllerAPI.GetBrakeAttr().Set(mInitialControllerValues->brake);
            controllerAPI.GetHandbrakeAttr().Set(mInitialControllerValues->handbrake);
        }
        if ((!mwcVehicle->isUsingDeprecatedSteerSetup()) || hasTankController)
        {
            // note: a tank might not have the steering API applied but still should not
            //       restore the deprecated steer values

            controllerAPI.GetSteerAttr().Set(mInitialControllerValues->steer);
        }
        else
        {
            controllerAPI.GetSteerLeftAttr().Set(mInitialControllerValues->steerLeft);
            controllerAPI.GetSteerRightAttr().Set(mInitialControllerValues->steerRight);
        }
        controllerAPI.GetTargetGearAttr().Set(mInitialControllerValues->targetGear);

        if (hasTankController)
        {
            pxr::PhysxSchemaPhysxVehicleTankControllerAPI tankControllerAPI(mInitialControllerValues->prim);
            tankControllerAPI.GetThrust0Attr().Set(mInitialControllerValues->thrust0);
            tankControllerAPI.GetThrust1Attr().Set(mInitialControllerValues->thrust1);
        }
    }
    else
    {
        for (InternalVehicleWheelAttachment* wheelAtt : mWheelAttachments)
        {
            if (wheelAtt)  // a removed wheel attachment might set the entry to nullptr
                wheelAtt->restoreInitialProperties();
        }
    }
}

void InternalVehicle::addWheelComponent(InternalVehicleWheelReferenceList& component)
{
    WheelComponentList::iterator iter = mWheelComponents.find(&component);
    if (iter == mWheelComponents.end())
        mWheelComponents.insert(&component);
}

void InternalVehicle::removeWheelComponent(InternalVehicleWheelReferenceList& component)
{
    CARB_ASSERT(mWheelComponents.find(&component) != mWheelComponents.end());
    mWheelComponents.erase(&component);
}

void InternalVehicle::removeFromWheelComponents()
{
    for (InternalVehicleWheelReferenceList* wheelRefList : mWheelComponents)
    {
        wheelRefList->removeVehicleWheels(*this);
    }
}

void InternalVehicle::removeWheelAttachment(const uint32_t wheelIndex, const bool disableWheel)
{
    CARB_ASSERT(wheelIndex < mWheelAttachments.size());
    mWheelAttachments[wheelIndex] = nullptr;

    if (disableWheel)
    {
        // if a wheel attachment is removed while in play mode, the wheel is disabled
        // to avoid that mapped PhysX shapes are accessed (these shapes get deleted) etc.
        mPhysXVehicle->disableWheel(wheelIndex);
    }
}

void InternalVehicle::updateMassProperties(const float mass, const ::physx::PxVec3& massSpaceInertiaTensor,
    const ::physx::PxTransform& vehicleMassFrame)
{
    const uint32_t userDefinedSprungMass = mFlags & InternalVehicleFlag::eUSER_DEFINED_SPRUNG_MASS;
    const uint32_t userDefinedMaxDroop = mFlags & InternalVehicleFlag::eUSER_DEFINED_MAX_DROOP;
    const uint32_t userDefinedRestLoad = mFlags & InternalVehicleFlag::eUSER_DEFINED_REST_LOAD;
    const uint32_t isUsingLatStiffY = mFlags & InternalVehicleFlag::eIS_USING_LAT_STIFF_Y;

    const ::physx::PxTransform* centerOfMassChangeTransformPtr;
    ::physx::PxTransform centerOfMassChangeTransform;
    if (mFlags & InternalVehicleFlag::eREFERENCE_FRAME_IS_COM)
        centerOfMassChangeTransformPtr = nullptr;
    else
    {
        // in PhysX, the wheel attachment related positions/orientations are stored relative the center of mass frame,
        // so we want to apply the transform for old center of mass frame to the new center of mass frame.
        centerOfMassChangeTransform = vehicleMassFrame.getInverse() * mPhysXVehicle->getRigidDynamicActor()->getCMassLocalPose();
        centerOfMassChangeTransformPtr = &centerOfMassChangeTransform;
    }

    mPhysXVehicle->updateMassProperties(mass, massSpaceInertiaTensor, centerOfMassChangeTransformPtr,
        mInternalScene.getVehicleContext().getFrame().vrtAxis,
        !userDefinedSprungMass, !userDefinedMaxDroop, !userDefinedRestLoad,
        isUsingLatStiffY);
}

void InternalVehicle::updateShapeMappings(const ::physx::PxShape* removedShape)
{
    // vehicle wheels can map to PxShape objects. If a shape is removed, the mapping has to
    // be disabled.

    mPhysXVehicle->removeWheelShape(removedShape);

    const uint32_t wheelTMEntryCount = static_cast<uint32_t>(mWheelTransformManagementEntries.size());
    if (wheelTMEntryCount)
    {
        for (uint32_t i = 0; i < wheelTMEntryCount; i++)
        {
            InternalVehicle::WheelTransformManagementEntry& wheelTMEntry = mWheelTransformManagementEntries[i];

            if ((wheelTMEntry.shape) && (wheelTMEntry.shape == removedShape))
            {
                wheelTMEntry.shape = nullptr;
            }
        }
    }
}

InternalVehicle::InitialControllerValues* InternalVehicle::allocateInitialControllerValues()
{
    void* mem = ICE_ALLOC(sizeof(InitialControllerValues));
    mInitialControllerValues = new(mem)(InitialControllerValues)();
    return mInitialControllerValues;
}

void InternalVehicle::setControllerParams(const VehicleControllerDesc& controllerDesc, InitialControllerValues* initialValues)
{
    CARB_ASSERT((mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE) ||
        (mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE));

    PhysXVehicleManagedWheelControl* mwcVehicle = static_cast<PhysXVehicleManagedWheelControl*>(mPhysXVehicle);

    setAccelerator(controllerDesc.accelerator);
    if (!mwcVehicle->isUsingDeprecatedBrakesSetup())
    {
        setBrake(0, controllerDesc.brake0);
        setBrake(1, controllerDesc.brake1);
    }
    else
    {
        setBrake(controllerDesc.brake);
        setHandbrake(controllerDesc.handbrake);
    }
    if (!mwcVehicle->isUsingDeprecatedSteerSetup())
    {
        setSteer(controllerDesc.steer);
    }
    else
    {
        setSteerLeft(controllerDesc.steerLeft);
        setSteerRight(controllerDesc.steerRight);
    }

    if (mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE)
    {
        setTargetGearDriveStandard(controllerDesc.targetGear, true);

        if (controllerDesc.type == eVehicleControllerTank)
        {
            const VehicleTankControllerDesc& tankControllerDesc = static_cast<const VehicleTankControllerDesc&>(controllerDesc);
            setThrust(0, tankControllerDesc.thrust0);
            setThrust(1, tankControllerDesc.thrust1);
        }
    }
    else
    {
        CARB_ASSERT(mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE);
        setTargetGearDriveBasic(controllerDesc.targetGear);
    }

    if (initialValues)
    {
        initialValues->accelerator = controllerDesc.accelerator;
        initialValues->brake0 = controllerDesc.brake0;
        initialValues->brake1 = controllerDesc.brake1;
        initialValues->brake = controllerDesc.brake;
        initialValues->handbrake = controllerDesc.handbrake;
        initialValues->steer = controllerDesc.steer;
        initialValues->steerLeft = controllerDesc.steerLeft;
        initialValues->steerRight = controllerDesc.steerRight;
        initialValues->targetGear = controllerDesc.targetGear;

        if (controllerDesc.type == eVehicleControllerTank)
        {
            const VehicleTankControllerDesc& tankControllerDesc = static_cast<const VehicleTankControllerDesc&>(controllerDesc);
            initialValues->thrust0 = tankControllerDesc.thrust0;
            initialValues->thrust1 = tankControllerDesc.thrust1;
        }
    }
}

void InternalVehicle::setAccelerator(const float accelerator)
{
    CARB_ASSERT((mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE) ||
        (mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE));

    PhysXVehicleManagedWheelControl* vehicle = static_cast<PhysXVehicleManagedWheelControl*>(mPhysXVehicle);
    vehicle->setThrottle(accelerator);
}

void InternalVehicle::setBrake(const uint32_t brakesIndex, const float brake)
{
    CARB_ASSERT((mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE) ||
        (mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE));

    PhysXVehicleManagedWheelControl* vehicle = static_cast<PhysXVehicleManagedWheelControl*>(mPhysXVehicle);
    vehicle->setBrake(brakesIndex, brake);
}

void InternalVehicle::setBrake(const float brake)
{
    CARB_ASSERT((mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE) ||
        (mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE));

    PhysXVehicleManagedWheelControl* vehicle = static_cast<PhysXVehicleManagedWheelControl*>(mPhysXVehicle);
    vehicle->setBrake(0, brake);
}

void InternalVehicle::setHandbrake(const float handbrake)
{
    CARB_ASSERT((mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE) ||
        (mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE));

    PhysXVehicleManagedWheelControl* vehicle = static_cast<PhysXVehicleManagedWheelControl*>(mPhysXVehicle);
    vehicle->setBrake(1, handbrake);
}

void InternalVehicle::setSteer(const float steer)
{
    CARB_ASSERT((mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE) ||
        (mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE));

    PhysXVehicleManagedWheelControl* vehicle = static_cast<PhysXVehicleManagedWheelControl*>(mPhysXVehicle);
    vehicle->setSteer(steer);
}

void InternalVehicle::setSteerLeft(const float steerLeft)
{
    CARB_ASSERT((mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE) ||
        (mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE));

    mSteerParams.steerLeft = steerLeft;
    const float steer = steerLeft - mSteerParams.steerRight;

    PhysXVehicleManagedWheelControl* vehicle = static_cast<PhysXVehicleManagedWheelControl*>(mPhysXVehicle);
    vehicle->setSteer(steer);
}

void InternalVehicle::setSteerRight(const float steerRight)
{
    CARB_ASSERT((mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE) ||
        (mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE));

    mSteerParams.steerRight = steerRight;
    const float steer = mSteerParams.steerLeft - steerRight;

    PhysXVehicleManagedWheelControl* vehicle = static_cast<PhysXVehicleManagedWheelControl*>(mPhysXVehicle);
    vehicle->setSteer(steer);
}

void InternalVehicle::setTargetGear(const int targetGear)
{
    if (mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE)
    {
        setTargetGearDriveStandard(targetGear, false);
    }
    else
    {
        CARB_ASSERT(mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE);
        setTargetGearDriveBasic(targetGear);
    }
}

void InternalVehicle::setThrust(const unsigned int thrustIndex, const float thrust)
{
    CARB_ASSERT(mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE);

    PhysXVehicleEngineDrive* vehicle = static_cast<PhysXVehicleEngineDrive*>(mPhysXVehicle);
    vehicle->setThrust(thrustIndex, thrust);
}

void InternalVehicle::setDrivenWheel(const uint32_t wheelIndex, const bool driven)
{
    if (mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE)
    {
        PhysXVehicleEngineDrive* vehicle = static_cast<PhysXVehicleEngineDrive*>(mPhysXVehicle);
        vehicle->setWheelToDriven(wheelIndex, driven);
    }
    else if (mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE)  // check for DriveBasic as compared to other scenarios, a user can
                                                                           // accidentally set the param on a vehicle without drive
    {
        PhysXVehicleDirectDrive* vehicle = static_cast<PhysXVehicleDirectDrive*>(mPhysXVehicle);
        vehicle->setWheelToDriven(wheelIndex, driven);
    }
}

void InternalVehicle::setTargetGearDriveStandard(const int targetGear, const bool isInitialGearState)
{
    CARB_ASSERT(mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE);

    uint32_t pxTargetGear = static_cast<uint32_t>(toPhysXGear(targetGear));

    if ((pxTargetGear == ::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR) &&
        (!(mFlags & InternalVehicleFlag::eHAS_AUTO_GEAR_BOX)))
    {
        CARB_LOG_WARN("PhysX Vehicle: transmission set to automatic but no auto gear box was defined.\n");
    }

    PhysXVehicleEngineDrive* vehicle = static_cast<PhysXVehicleEngineDrive*>(mPhysXVehicle);
    vehicle->setTargetGearCommand(pxTargetGear, isInitialGearState);
}

void InternalVehicle::setTargetGearDriveBasic(const int targetGear)
{
    CARB_ASSERT(mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE);

    PhysXVehicleDirectDrive* vehicle = static_cast<PhysXVehicleDirectDrive*>(mPhysXVehicle);

    vehicle->setDriveMode(targetGear);
}

void InternalVehicle::getDriveState(omni::physx::VehicleDriveState& driveState) const
{
    CARB_ASSERT((mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE) ||
        (mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE));

    PhysXVehicleManagedWheelControl* vehicle = static_cast<PhysXVehicleManagedWheelControl*>(mPhysXVehicle);
    driveState.accelerator = vehicle->getThrottle();
    driveState.brake0 = vehicle->getBrake(0);
    driveState.brake1 = vehicle->getBrake(1);
    driveState.steer = vehicle->getSteer();

    if (mPhysXVehicle->getType() == PhysXVehicleType::eENGINE_DRIVE)
    {
        PhysXVehicleEngineDrive* vehicleEngineDrive = static_cast<PhysXVehicleEngineDrive*>(mPhysXVehicle);

        driveState.clutch = vehicleEngineDrive->getClutch();
        driveState.currentGear = fromPhysXGear(vehicleEngineDrive->getCurrentGear());
        driveState.targetGear = fromPhysXGear(vehicleEngineDrive->getTargetGear());
        driveState.gearSwitchTime = vehicleEngineDrive->getGearSwitchTime();
        driveState.autoboxTimeSinceLastShift = vehicleEngineDrive->getAutoboxTimeSinceLastShift();
        driveState.engineRotationSpeed = vehicleEngineDrive->getEngineRotationSpeed();
        driveState.automaticTransmission = vehicleEngineDrive->getAutomaticTransmissionEnabled();
    }
    else
    {
        driveState.clutch = 0.0f;
        driveState.currentGear = 0;
        driveState.targetGear = 0;
        driveState.gearSwitchTime = 0.0f;
        driveState.autoboxTimeSinceLastShift = 0.0f;
        driveState.engineRotationSpeed = 0.0f;
        driveState.automaticTransmission = false;
    }
}

void InternalVehicle::setPeakTorque(const float peakTorque)
{
    CARB_ASSERT(mPhysXVehicle->getType() == PhysXVehicleType::eDIRECT_DRIVE);

    PhysXVehicleDirectDrive* vehicle = static_cast<PhysXVehicleDirectDrive*>(mPhysXVehicle);
    vehicle->setPeakDriveTorque(peakTorque);
}

void InternalVehicle::setControllerParams(const uint32_t wheelIndex,
    const WheelControllerDesc& controllerDesc)
{
    setDriveTorque(wheelIndex, controllerDesc.driveTorque);
    setBrakeTorque(wheelIndex, controllerDesc.brakeTorque);
    setSteerAngle(wheelIndex, controllerDesc.steerAngle);
}

void InternalVehicle::setDriveTorque(const uint32_t wheelIndex, const float driveTorque)
{
    CARB_ASSERT(mPhysXVehicle->getType() == PhysXVehicleType::eRAW_WHEEL_CONTROL);

    PhysXVehicleRawWheelControl* vehicle = static_cast<PhysXVehicleRawWheelControl*>(mPhysXVehicle);
    vehicle->setDriveTorque(wheelIndex, driveTorque);
}

void InternalVehicle::setBrakeTorque(const uint32_t wheelIndex, const float brakeTorque)
{
    CARB_ASSERT(mPhysXVehicle->getType() == PhysXVehicleType::eRAW_WHEEL_CONTROL);

    PhysXVehicleRawWheelControl* vehicle = static_cast<PhysXVehicleRawWheelControl*>(mPhysXVehicle);
    vehicle->setBrakeTorque(wheelIndex, brakeTorque);
}

void InternalVehicle::setSteerAngle(const uint32_t wheelIndex, const float steerAngle)
{
    CARB_ASSERT(mPhysXVehicle->getType() == PhysXVehicleType::eRAW_WHEEL_CONTROL);

    PhysXVehicleRawWheelControl* vehicle = static_cast<PhysXVehicleRawWheelControl*>(mPhysXVehicle);
    vehicle->setSteerAngle(wheelIndex, steerAngle);
}

bool InternalVehicle::getWheelTransformations(const int* wheelIndices, const uint32_t wheelIndexCount,
    const bool addVehicleTransform,
    carb::Float3* positions, carb::Float4* orientations) const
{
    ::physx::PxRigidDynamic* vehicleActor = mPhysXVehicle->getRigidDynamicActor();
    ::physx::PxTransform bodyToActor = vehicleActor->getCMassLocalPose();
    ::physx::PxTransform body2World;
    uint32_t refFrameIsCoM;

    if (addVehicleTransform)
    {
        body2World = vehicleActor->getGlobalPose() * bodyToActor;
    }
    else
    {
        refFrameIsCoM = mFlags & InternalVehicleFlag::eREFERENCE_FRAME_IS_COM;
    }

    for (uint32_t i = 0; i < wheelIndexCount; i++)
    {
        const int wheelIndex = wheelIndices[i];
        if (wheelIndex < mWheelAttachments.size())
        {
            const ::physx::PxTransform& localPose = mPhysXVehicle->getWheelLocalPose(wheelIndex);
            if (!addVehicleTransform)
            {
                if (refFrameIsCoM)  // deprecated
                {
                    positions[i] = fromPhysX(localPose.p);
                    orientations[i] = fromPhysX(localPose.q);
                }
                else
                {
                    positions[i] = fromPhysX(bodyToActor.transform(localPose.p));
                    orientations[i] = fromPhysX(bodyToActor.q * localPose.q);
                }
            }
            else
            {
                const ::physx::PxTransform worldPose = body2World * localPose;
                positions[i] = fromPhysX(worldPose.p);
                orientations[i] = fromPhysX(worldPose.q);
            }
        }
        else
        {
            CARB_LOG_ERROR("PhysX Vehicle: getWheelTransformations: wheel index is out of bounds");
            return false;
        }
    }

    return true;
}

float InternalVehicle::computeVelocity(const carb::Float3& direction) const
{
    const ::physx::PxRigidDynamic* vehicleBody = mPhysXVehicle->getRigidDynamicActor();
    const ::physx::PxVec3 linVel = vehicleBody->getLinearVelocity();
    const ::physx::PxQuat vehicleBodyToWorld = vehicleBody->getGlobalPose().q * vehicleBody->getCMassLocalPose().q;

    const ::physx::PxVec3& dir = toPhysX(direction);
    const ::physx::PxVec3 worldDir = vehicleBodyToWorld.rotate(dir);

    const float speed = worldDir.dot(linVel);
    return speed;
}

void InternalVehicle::getWheelState(const uint32_t wheelIndex, omni::physx::VehicleWheelState& state) const
{
    const ::physx::PxTransform& localPose = mPhysXVehicle->getWheelLocalPose(wheelIndex);

    if (mFlags & InternalVehicleFlag::eREFERENCE_FRAME_IS_COM)  // deprecated
    {
        state.localPosePosition = fromPhysX(localPose.p);
        state.localPoseQuaternion = fromPhysX(localPose.q);
    }
    else
    {
        ::physx::PxRigidDynamic* vehicleActor = mPhysXVehicle->getRigidDynamicActor();
        ::physx::PxTransform bodyToActor = vehicleActor->getCMassLocalPose();

        state.localPosePosition = fromPhysX(bodyToActor.transform(localPose.p));
        state.localPoseQuaternion = fromPhysX(bodyToActor.q * localPose.q);
    }

    state.rotationSpeed = mPhysXVehicle->getWheelRotationSpeed(wheelIndex);
    state.rotationAngle = mPhysXVehicle->getWheelRotationAngle(wheelIndex);

    state.steerAngle = mPhysXVehicle->getWheelSteerAngle(wheelIndex);

    const ::physx::vehicle2::PxVehicleRoadGeometryState& roadGeomState = mPhysXVehicle->getRoadGeometryState(wheelIndex);
    const ::physx::PxPlane& groundPlane = roadGeomState.plane;
    state.groundPlane.x = groundPlane.n.x;
    state.groundPlane.y = groundPlane.n.y;
    state.groundPlane.z = groundPlane.n.z;
    state.groundPlane.w = groundPlane.d;

    state.suspensionJounce = mPhysXVehicle->getSuspensionJounce(wheelIndex);

    state.suspensionForce = fromPhysX(mPhysXVehicle->getSuspensionForce(wheelIndex));

    state.tireFriction = mPhysXVehicle->getTireFriction(wheelIndex);

    state.tireLongitudinalDirection = fromPhysX(mPhysXVehicle->getTireLongitudinalDirection(wheelIndex));
    state.tireLateralDirection = fromPhysX(mPhysXVehicle->getTireLateralDirection(wheelIndex));

    state.tireLongitudinalSlip = mPhysXVehicle->getTireLongitudinalSlip(wheelIndex);
    state.tireLateralSlip = mPhysXVehicle->getTireLateralSlip(wheelIndex);

    state.tireForce = fromPhysX(mPhysXVehicle->getTireForce(wheelIndex));

    const bool isSleeping = mPhysXVehicle->isSleeping();
    const bool isOnGround = mPhysXVehicle->isWheelOnGround(wheelIndex) && (!isSleeping);

    state.isOnGround = isOnGround;

    if (isOnGround)
    {
        CARB_ASSERT(roadGeomState.hitState);

        const ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryState& queryState =
            mPhysXVehicle->getPhysXRoadGeometryQueryState(wheelIndex);

        if (queryState.actor)
        {
            ObjectId actorId = ObjectId(queryState.actor->userData);
            state.groundActor = actorId;
        }
        else
            state.groundActor = kInvalidObjectId;

        if (queryState.shape)
        {
            ObjectId shapeId = ObjectId(queryState.shape->userData);
            state.groundShape = shapeId;
        }
        else
            state.groundShape = kInvalidObjectId;

        if (queryState.material)
        {
            ObjectId materialId = ObjectId(queryState.material->userData);
            state.groundMaterial = materialId;
        }
        else
            state.groundMaterial = kInvalidObjectId;

        state.groundHitPosition = fromPhysX(queryState.hitPosition);
    }
    else
    {
        state.groundActor = kInvalidObjectId;
        state.groundShape = kInvalidObjectId;
        state.groundMaterial = kInvalidObjectId;
        state.groundHitPosition.x = 0.0f;
        state.groundHitPosition.y = 0.0f;
        state.groundHitPosition.z = 0.0f;
    }
}


::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams InternalTireFrictionTable::sDefaultMaterialFrictionTable;
InternalTireFrictionTable::DefaultInitializer InternalTireFrictionTable::sDefaultInitializer;

InternalTireFrictionTable* InternalTireFrictionTable::create(const TireFrictionTableDesc& tireFrictionTableDesc,
    InternalPhysXDatabase& internalDatabase)
{
    CARB_ASSERT(tireFrictionTableDesc.frictionValues.size() == tireFrictionTableDesc.materialIds.size());

    const uint32_t entryCount = static_cast<uint32_t>(tireFrictionTableDesc.materialIds.size());

    InternalTireFrictionTable* tireFrictionTable = ICE_NEW(InternalTireFrictionTable);
    if (tireFrictionTable)
    {
        if (entryCount > 0)
        {
            ::physx::vehicle2::PxVehiclePhysXMaterialFriction* materialFrictionPairs = reinterpret_cast<::physx::vehicle2::PxVehiclePhysXMaterialFriction*>(
                ICE_ALLOC(sizeof(::physx::vehicle2::PxVehiclePhysXMaterialFriction) * entryCount));

            if (materialFrictionPairs)
            {
                tireFrictionTable->mMaterialFrictionTable.materialFrictions = materialFrictionPairs;
                tireFrictionTable->mMaterialFrictionTable.nbMaterialFrictions = entryCount;
                tireFrictionTable->mMaterialFrictionTable.defaultFriction = tireFrictionTableDesc.defaultFrictionValue;

                for (uint32_t i = 0; i < entryCount; i++)
                {
                    ::physx::PxMaterial* material = nullptr;
                    const ObjectId& materialId = tireFrictionTableDesc.materialIds[i];

                    if (materialId != kInvalidObjectId)
                    {
                        material = reinterpret_cast<::physx::PxMaterial*>(
                            internalDatabase.getTypedRecord(PhysXType::ePTMaterial, materialId));
                        if (!material)
                        {
                            CARB_LOG_ERROR(
                                "PhysX Vehicle: tire friction table: referenced material \"%s\" could not be found (most likely it has not been created).\n",
                                tireFrictionTableDesc.materialPaths[i].GetText());
                        }
                    }
                    else
                    {
                        CARB_LOG_ERROR(
                            "PhysX Vehicle: tire friction table: referenced material \"%s\" could not be found (most likely it has not been created).\n",
                            tireFrictionTableDesc.materialPaths[i].GetText());
                    }

                    materialFrictionPairs[i].material = material;
                    materialFrictionPairs[i].friction = tireFrictionTableDesc.frictionValues[i];
                }

                return tireFrictionTable;
            }
            else
            {
                CARB_LOG_ERROR("PhysX Vehicle: tire friction table: allocating memory failed.\n");
                SAFE_DELETE_SINGLE(tireFrictionTable);
            }
        }
        else
        {
            tireFrictionTable->mMaterialFrictionTable.materialFrictions = nullptr;
            tireFrictionTable->mMaterialFrictionTable.nbMaterialFrictions = 0;
            tireFrictionTable->mMaterialFrictionTable.defaultFriction = tireFrictionTableDesc.defaultFrictionValue;

            return tireFrictionTable;
        }
    }
    else
    {
        CARB_LOG_ERROR("PhysX Vehicle: tire friction table: allocating memory failed.\n");
    }

    return nullptr;
}

void InternalTireFrictionTable::release(InternalTireFrictionTable& tireFrictionTable)
{
    if (tireFrictionTable.mMaterialFrictionTable.materialFrictions)  // entries do not have to be defined necessarily
                                                                     // since there is the default friction value
    {
        ICE_FREE_BASIC(tireFrictionTable.mMaterialFrictionTable.materialFrictions);
    }
    delete(&tireFrictionTable);
}

void InternalTireFrictionTable::initDefault()
{
    sDefaultMaterialFrictionTable.nbMaterialFrictions = 0;
    sDefaultMaterialFrictionTable.materialFrictions = nullptr;
    sDefaultMaterialFrictionTable.defaultFriction = 1.0f;
}

void InternalTireFrictionTable::update(const pxr::VtArray<float>& frictionValues)
{
    ::physx::vehicle2::PxVehiclePhysXMaterialFriction* materialFrictionPairs = mMaterialFrictionTable.materialFrictions;

    uint32_t minFrictionValueCount = CARB_MIN(mMaterialFrictionTable.nbMaterialFrictions,
        static_cast<uint32_t>(frictionValues.size()));
    // the user might remove or add materials to the friction table while playing even though we
    // do not support it. Thus need to make sure to at least stay in valid range (but to be really
    // correct, the materials would have to be compared too, to find the matching ones and only
    // update those).

    for (uint32_t i = 0; i < minFrictionValueCount; i++)
    {
        materialFrictionPairs[i].friction = frictionValues[i];
    }
}


void InternalVehicleWheelAttachment::release(const bool disableWheel)
{
    if (mVehicle)  // the vehicle might have been deleted or vehicle creation failed
        mVehicle->removeWheelAttachment(mWheelIndex, disableWheel);

    delete this;
}

void InternalVehicleWheelAttachment::restoreInitialProperties()
{
    if (mInitialControllerValues.prim.IsValid())
    {
        CARB_ASSERT(mInitialControllerValues.prim.HasAPI<pxr::PhysxSchemaPhysxVehicleWheelControllerAPI>());
        pxr::PhysxSchemaPhysxVehicleWheelControllerAPI controllerAPI(mInitialControllerValues.prim);
        controllerAPI.GetDriveTorqueAttr().Set(mInitialControllerValues.driveTorque);
        controllerAPI.GetBrakeTorqueAttr().Set(mInitialControllerValues.brakeTorque);
        controllerAPI.GetSteerAngleAttr().Set(mInitialControllerValues.steerAngle);
    }
}
