// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"
#include "Internal.h"
#include "VehicleGenerator.h"
#include <PhysXDefines.h>

#include <private/omni/physx/PhysxUsd.h>

#include <PxPhysicsAPI.h>
#include <omnipvd/PxOmniPvd.h> // note: remove as soon as PxPhysicsAPI.h is fixed up

#include <vector>

namespace omni
{
namespace physx
{
namespace internal
{
class InternalPhysXDatabase;

class InternalScene;


class InternalVehicleContext
{
public:
    InternalVehicleContext()
    {
        mPhysXContext.setToDefault();
    }

    ~InternalVehicleContext()
    {
    }

    void init(const usdparser::VehicleContextDesc&, ::physx::PxScene&);

    ::physx::vehicle2::PxVehiclePhysXSimulationContext& getContext()
    {
        return mPhysXContext;
    }
    const ::physx::vehicle2::PxVehiclePhysXSimulationContext& getContext() const
    {
        return mPhysXContext;
    }

    const ::physx::vehicle2::PxVehicleFrame& getFrame() const
    {
        return mPhysXContext.frame;
    }


private:
    ::physx::vehicle2::PxVehiclePhysXSimulationContext mPhysXContext;
};


struct InternalTireFrictionTable : public Allocateable
{
private:
    static void initDefault();

    class DefaultInitializer
    {
    public:
        DefaultInitializer()
        {
            InternalTireFrictionTable::initDefault();
        }
    };

private:
    InternalTireFrictionTable()
    {
    }
    ~InternalTireFrictionTable()
    {
    }

public:
    static InternalTireFrictionTable* create(const usdparser::TireFrictionTableDesc& tireFrictionTableDesc,
                                             InternalPhysXDatabase&);
    static void release(InternalTireFrictionTable&);

    static const ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams* getDefault()
    {
        return &sDefaultMaterialFrictionTable;
    }

    ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams* getMaterialFrictionTable()
    {
        return &mMaterialFrictionTable;
    }

    inline void setDefaultFrictionValue(float defaultFrictionValue)
    {
        mMaterialFrictionTable.defaultFriction = defaultFrictionValue;
    }

    void update(const pxr::VtArray<float>& frictionValues);

private:
    ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams mMaterialFrictionTable;
    static ::physx::vehicle2::PxVehiclePhysXMaterialFrictionParams sDefaultMaterialFrictionTable;
    static DefaultInitializer sDefaultInitializer;
};


class InternalVehicleReferenceList;
class InternalVehicleWheelReferenceList;
class InternalVehicleWheelAttachment;

struct InternalVehicleFlag
{
    enum Enum
    {
        eSKIP_UPDATE_TRANSFORM = 1 << 0,
        eNOTIFY_TRANSFORM = 1 << 1,
        eUSER_DEFINED_SPRUNG_MASS = 1 << 2,
        eUSER_DEFINED_MAX_DROOP = 1 << 3, // deprecated
        eUSER_DEFINED_REST_LOAD = 1 << 4,
        eIS_USING_LAT_STIFF_Y = 1 << 5, // deprecated
        eHAS_AUTO_GEAR_BOX = 1 << 6,
        eREFERENCE_FRAME_IS_COM = 1 << 7 // deprecated
    };
};

class InternalVehicle : public Allocateable
{
private:
    typedef std::unordered_set<InternalVehicleWheelReferenceList*> WheelComponentList;

    // deprecated
    struct SteerParams // needed to translate from USD spec to PhysX spec
    {
        SteerParams() : steerLeft(0.0f), steerRight(0.0f)
        {
        }

        float steerLeft;
        float steerRight;
    };

    InternalVehicle();

public:
    struct WheelTransformManagementEntry
    {
        WheelTransformManagementEntry()
        {
        }

        void init(pxr::UsdPrim&, pxr::UsdPrim&, const ::physx::PxShape*);

        pxr::GfMatrix4d initialTransform;
        pxr::GfMatrix4d initialShapeTransform; // not used if there is no collision shape or if it is
                                               // the same as the wheel root

        carb::Float3 scale;
        carb::Float3 shapeScale; // not used if there is no collision shape or if it is
                                 // the same as the wheel root

        pxr::UsdPrim wheelRootPrim;

        // note: there can be a bunch of Xforms along the parent hierarchy of the wheel root Xform, so
        //       each wheel can have a different parent Xform
        pxr::UsdPrim wheelRootParentXformPrim;

        pxr::UsdPrim shapePrim; // invalid if there is no collision shape
        const ::physx::PxShape* shape; // NULL if there is no collision shape
    };

    struct InitialControllerValues
    {
        pxr::UsdPrim prim; // currently this is the only reason to store the prim
        float accelerator;
        float brake0;
        float brake1;
        float brake; // deprecated
        float handbrake; // deprecated
        float steer;
        float steerLeft; // deprecated
        float steerRight; // deprecated
        int targetGear;

        // bit of a waste as tank type vehicles seem rare but storing the initial values itself
        // feels like a bigger waste, thus keeping it simple and not using a separate struct.
        float thrust0;
        float thrust1;
    };

    InternalVehicle(InternalScene& internalScene)
        : mInternalScene(internalScene),
          mPhysXVehicle(nullptr),
          mEngineOrDriveBasic(nullptr),
          mInitialControllerValues(nullptr),
          mBufferIndex(0),
          mFlags(0)
    {
        mScale.x = 1.0f;
        mScale.y = 1.0f;
        mScale.z = 1.0f;
    }

    ~InternalVehicle();

    void restoreInitialProperties();

    void addWheelComponent(InternalVehicleWheelReferenceList&);
    void removeWheelComponent(InternalVehicleWheelReferenceList&);

    void removeWheelAttachment(const uint32_t wheelIndex, const bool disableWheel);

    void updateMassProperties(const float mass,
                              const ::physx::PxVec3& massSpaceInertiaTensor,
                              const ::physx::PxTransform& vehicleMassFrame);
    void updateShapeMappings(const ::physx::PxShape* removedShape);

    InitialControllerValues* allocateInitialControllerValues();

    ::physx::PxRigidDynamic* getRigidDynamicActor()
    {
        return mPhysXVehicle->getRigidDynamicActor();
    }

    // the following block is for vehicles with a drive
    void setControllerParams(const usdparser::VehicleControllerDesc&, InitialControllerValues* initialValues);
    void setAccelerator(const float accelerator);
    void setBrake(const uint32_t brakesIndex, const float brake);
    void setBrake(const float brake); // deprecated
    void setHandbrake(const float handbrake); // deprecated
    void setSteer(const float steer);
    void setSteerLeft(const float steerLeft); // deprecated
    void setSteerRight(const float steerRight); // deprecated
    void setTargetGear(const int targetGear);
    void setThrust(const unsigned int thrustIndex, const float thrust); // for tanks
    void setDrivenWheel(const uint32_t wheelIndex, const bool driven);
    void getDriveState(VehicleDriveState&) const;

    // the following block is for vehicles with DriveBasic
    void setPeakTorque(const float peakTorque);

    // the following block is for vehicles without drive
    void setControllerParams(const uint32_t wheelIndex, const usdparser::WheelControllerDesc&);
    void setDriveTorque(const uint32_t wheelIndex, const float driveTorque);
    void setBrakeTorque(const uint32_t wheelIndex, const float brakeTorque);
    void setSteerAngle(const uint32_t wheelIndex, const float steerAngle);

    // transformations are in vehicle space unless addVehicleTransform is set to true
    bool getWheelTransformations(const int* wheelIndices,
                                 const uint32_t wheelIndexCount,
                                 const bool addVehicleTransform,
                                 carb::Float3* positions,
                                 carb::Float4* orientations) const;

    float computeVelocity(const carb::Float3& direction) const;

    void setToRestState()
    {
        mPhysXVehicle->setToRestState();
    }

    void setWheelRotationSpeed(const uint32_t wheelIndex, const float rotationSpeed)
    {
        mPhysXVehicle->setWheelRotationSpeed(wheelIndex, rotationSpeed);
    }

    void setWheelRotationAngle(const uint32_t wheelIndex, const float rotationAngle)
    {
        mPhysXVehicle->setWheelRotationAngle(wheelIndex, rotationAngle);
    }

    void getWheelState(const uint32_t wheelIndex, VehicleWheelState& state) const;

private:
    void removeFromWheelComponents();

    void setTargetGearDriveStandard(const int targetGear, const bool isInitialGearState);
    void setTargetGearDriveBasic(const int targetGear);

    // conversion because PhysX starts with 0 for reverse, while our USD schema starts with -1
    inline int toPhysXGear(const int targetGear) const
    {
        if (targetGear != ::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR)
            return targetGear + 1;
        else
            return targetGear;
    }

    inline int fromPhysXGear(const uint32_t targetGear) const
    {
        if (targetGear != ::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR)
            return static_cast<int>(targetGear) - 1;
        else
            return static_cast<int>(targetGear);
    }


public:
    std::vector<InternalVehicleWheelAttachment*> mWheelAttachments;
    std::vector<WheelTransformManagementEntry> mWheelTransformManagementEntries; // for wheel transform management if
                                                                                 // required
    InternalScene& mInternalScene;
    PhysXActorVehicleBase* mPhysXVehicle;
    InternalVehicleReferenceList* mEngineOrDriveBasic;
    WheelComponentList mWheelComponents;
    SteerParams mSteerParams; // for a vehicle with a controller
    InitialControllerValues* mInitialControllerValues; // allocated if vehicle has controller
    carb::Float3 mScale; // the total scale of the vehicle prim
    uint32_t mBufferIndex; // position in mVehicles list of scene
    uint32_t mFlags;
};

class InternalVehicleReferenceList : public Allocateable
{
private:
    typedef std::unordered_set<InternalVehicle*> VehicleList;

public:
    InternalVehicleReferenceList()
    {
    }

    void addVehicle(InternalVehicle& vehicle)
    {
        CARB_ASSERT(mVehicles.find(&vehicle) == mVehicles.end());
        mVehicles.insert(&vehicle);
    }

    void removeVehicle(InternalVehicle& vehicle)
    {
        CARB_ASSERT(mVehicles.find(&vehicle) != mVehicles.end());
        mVehicles.erase(&vehicle);
    }

    void unlinkFromVehicles(PhysXType type)
    {
        if ((type == ePTVehicleEngine) || (type == ePTVehicleDriveBasic))
        {
            for (InternalVehicle* vehicle : mVehicles)
                vehicle->mEngineOrDriveBasic = nullptr;
        }
    }

    VehicleList mVehicles;
};

class InternalVehicleWheelReferenceList : public Allocateable
{
    static_assert(::physx::vehicle2::PxVehicleLimits::eMAX_NB_WHEELS <= 32, ""); // using bits of uint32 to mark wheel
                                                                                 // indices

public:
    typedef std::unordered_map<InternalVehicle*, uint32_t> VehicleAndWheelsList;

    InternalVehicleWheelReferenceList()
    {
    }

    ~InternalVehicleWheelReferenceList()
    {
        unlinkFromVehicles();
    }

    void addWheel(InternalVehicle& vehicle, const uint32_t wheelIndex)
    {
        CARB_ASSERT(wheelIndex < 32);
        const uint32_t wheelIndexBit = (1 << wheelIndex);
        VehicleAndWheelsList::iterator iter = mVehicleWheels.find(&vehicle);
        if (iter != mVehicleWheels.end())
        {
            CARB_ASSERT(!(iter->second & wheelIndexBit));
            iter->second |= wheelIndexBit;
        }
        else
            mVehicleWheels.insert({ &vehicle, wheelIndexBit });
    }

    void removeVehicleWheels(InternalVehicle& vehicle)
    {
        CARB_ASSERT(mVehicleWheels.find(&vehicle) != mVehicleWheels.end());
        mVehicleWheels.erase(&vehicle);
    }

private:
    void unlinkFromVehicles()
    {
        for (VehicleAndWheelsList::iterator iter = mVehicleWheels.begin(); iter != mVehicleWheels.end(); iter++)
        {
            iter->first->removeWheelComponent(*this);
        }
    }


public:
    VehicleAndWheelsList mVehicleWheels;
};

class InternalVehicleWheelAttachment : public Allocateable
{
private:
    ~InternalVehicleWheelAttachment()
    {
    }

public:
    struct InitialControllerValues
    {
        pxr::UsdPrim prim; // currently this is the only reason to store the prim
        float driveTorque;
        float brakeTorque;
        float steerAngle;
    };


    InternalVehicleWheelAttachment() : mVehicle(nullptr)
    {
    }
    // initializing mVehicle since attachments get created first and the vehicle after. If
    // the latter fails and aborts early, the attachment would have an undefined state.

    void release(const bool disableWheel);

    void restoreInitialProperties();

    // the following block is only for vehicles without drive
    void setControllerParams(const usdparser::WheelControllerDesc& desc, const bool setAsInitialValues)
    {
        mVehicle->setControllerParams(mWheelIndex, desc);

        if (setAsInitialValues)
        {
            mInitialControllerValues.driveTorque = desc.driveTorque;
            mInitialControllerValues.brakeTorque = desc.brakeTorque;
            mInitialControllerValues.steerAngle = desc.steerAngle;
        }
    }
    void setDriveTorque(const float driveTorque)
    {
        mVehicle->setDriveTorque(mWheelIndex, driveTorque);
    }
    void setBrakeTorque(const float brakeTorque)
    {
        mVehicle->setBrakeTorque(mWheelIndex, brakeTorque);
    }
    void setSteerAngle(const float steerAngle)
    {
        mVehicle->setSteerAngle(mWheelIndex, steerAngle);
    }

    void setWheelRotationSpeed(const float rotationSpeed)
    {
        mVehicle->setWheelRotationSpeed(mWheelIndex, rotationSpeed);
    }

    void setWheelRotationAngle(const float rotationAngle)
    {
        mVehicle->setWheelRotationAngle(mWheelIndex, rotationAngle);
    }

    void getWheelState(VehicleWheelState& state) const
    {
        mVehicle->getWheelState(mWheelIndex, state);
    }


public:
    InternalVehicle* mVehicle;
    uint32_t mWheelIndex;
    InitialControllerValues mInitialControllerValues; // only used if attachment has a wheel controller
};

} // namespace internal
} // namespace physx
} // namespace omni
