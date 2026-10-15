// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-VEHICLE-CONTROL-001
// @covers AC-1 AC-2 AC-3 AC-4
#include <omni/physx/NativeVehicle.h>
#include <omni/physx/PhysXRuntime.h>
#include "RuntimeExecution.h"
#include "usdLoad/LoadUsd.h"
#include "usdLoad/AttachedStage.h"
#include "internal/InternalPhysXDatabase.h"
#include "internal/InternalVehicle.h"
#include "internal/InternalScene.h"
#include <vehicle/suspension/PxVehicleSuspensionHelpers.h>
#include <algorithm>
#include <cmath>
namespace omni::physx::native
{
namespace
{
void copy(float* out, const carb::Float3& v) { out[0]=v.x; out[1]=v.y; out[2]=v.z; }
void copy(float* out, const carb::Float4& v) { out[0]=v.x; out[1]=v.y; out[2]=v.z; out[3]=v.w; }
bool range(float value, float low, float high) { return std::isfinite(value) && value>=low && value<=high; }
VehicleIdentity identity(const internal::InternalDatabase::Record* record)
{
    return record && record->mType!=ePTRemoved ? VehicleIdentity{record->mKey.handle,record->mGeneration} : VehicleIdentity{};
}
}
VehicleResult controlVehicle(uint64_t attachment, const char* path, const VehicleCommand& command, VehicleState& output) noexcept
try
{
    using namespace ::physx;
    if (!path || path[0]!='/' || command.action>2 || (command.action && !command.generation) ||
        (command.action==1 && (!range(command.throttle,0,1) || !range(command.brake0,0,1) ||
         !range(command.brake1,0,1) || !range(command.steer,-1,1) || command.direction < -2 || command.direction > 1)))
        return VehicleResult::Invalid;
    if (!runtime::tryGetPhysxInterface()) return VehicleResult::Stale;
    std::shared_ptr<omni::physx::usdparser::AttachedStage> owner=usdparser::UsdLoad::getUsdLoad()->acquireAttachment(attachment);
    if (!owner) return VehicleResult::Stale;
    RuntimeExecutionScope admission(attachment);
    if (!admission) return VehicleResult::Busy;
    const omni::physics::parse::IPhysicsSource * source=owner->getSource();
    if (!source) return VehicleResult::NotFound;
    const omni::physics::parse::ObjectKey key=source->findByPath(path);
    const omni::physx::usdparser::ObjectDb * objects=owner->getObjectDatabase();
    omni::physx::internal::InternalPhysXDatabase & db=owner->getInternalPhysXDatabase();
    const omni::physx::internal::InternalDatabase::Record * record=db.getFullTypedRecord(ePTVehicleController, objects->findEntry(key,usdparser::eVehicleControllerStandard));
    if (!record || !record->mInternalPtr || record->mKey!=key)
    {
        if (command.generation) return VehicleResult::Stale;
        if (db.getFullTypedRecord(ePTVehicleController,objects->findEntry(key,usdparser::eVehicleControllerTank)))
            return VehicleResult::Unavailable;
        const omni::physx::internal::InternalDatabase::Record * vehicleRecord=db.getFullTypedRecord(ePTVehicle,objects->findEntry(key,usdparser::eVehicle));
        if (vehicleRecord && vehicleRecord->mInternalPtr &&
            static_cast<internal::InternalVehicle*>(vehicleRecord->mInternalPtr)->isRawWheelControl())
            return VehicleResult::Unavailable;
        return VehicleResult::NotFound;
    }
    if (command.generation && command.generation!=record->mGeneration) return VehicleResult::Stale;
    omni::physx::internal::InternalVehicle * vehicle=static_cast<internal::InternalVehicle*>(record->mInternalPtr);
    if (!vehicle->mPhysXVehicle || vehicle->isRawWheelControl()) return VehicleResult::Unavailable;
    const bool engine=vehicle->mPhysXVehicle->getType()==PhysXVehicleType::eENGINE_DRIVE;
    ::physx::PxRigidDynamic * actor=vehicle->getRigidDynamicActor();
    ::physx::PxScene * scene=actor ? actor->getScene() : nullptr;
    if (!scene || scene->getFlags().isSet(PxSceneFlag::eENABLE_GPU_DYNAMICS) ||
        scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API)) return VehicleResult::Unavailable;
    static_assert(PxVehicleLimits::eMAX_NB_WHEELS==20);
    VehicleState result{};
    result.identity=identity(record); result.step=owner->simulationTimestamp(); result.engineFields=engine ? 1 : 0;
    // Resolve all wheel identities and validate bounds before mutating controls.
    for (omni::physx::internal::InternalVehicleWheelAttachment * wheel : vehicle->mWheelAttachments)
    {
        if (!wheel) continue;
        const unsigned int i=wheel->mWheelIndex;
        if (i>=20 || result.wheelCount>=20 || i>=vehicle->mWheelTransformManagementEntries.size()) return VehicleResult::Error;
        omni::physx::native::VehicleWheel & out=result.wheels[result.wheelCount++];
        out.index=i;
        const omni::physics::parse::ObjectKey wheelKey=vehicle->mWheelTransformManagementEntries[i].wheelRootKey;
        out.identity=identity(db.getFullTypedRecord(ePTVehicleWheelAttachment,objects->findEntry(wheelKey,usdparser::eVehicleWheelAttachment)));
        out.comFrame=(vehicle->mFlags & internal::InternalVehicleFlag::eREFERENCE_FRAME_IS_COM) ? 1 : 0;
        VehicleWheelState state{};
        vehicle->getWheelState(i,state);
        copy(out.position,state.localPosePosition); copy(out.orientation,state.localPoseQuaternion);
        out.rotationSpeed=state.rotationSpeed; out.rotationAngle=state.rotationAngle; out.steerAngle=state.steerAngle;
        out.onGround=state.isOnGround ? 1 : 0;
        if (state.isOnGround)
        {
            copy(out.groundPlane,state.groundPlane); copy(out.groundPosition,state.groundHitPosition);
            PhysXType type=ePTRemoved;
            out.groundActor=identity(db.getFullRecord(type,state.groundActor));
            out.groundShape=identity(db.getFullRecord(type,state.groundShape));
            out.groundMaterial=identity(db.getFullRecord(type,state.groundMaterial));
        }
        out.suspensionJounce=state.suspensionJounce; copy(out.suspensionForce,state.suspensionForce);
        out.tireFriction=state.tireFriction;
        copy(out.longitudinalDirection,state.tireLongitudinalDirection); copy(out.lateralDirection,state.tireLateralDirection);
        out.longitudinalSlip=state.tireLongitudinalSlip; out.lateralSlip=state.tireLateralSlip; copy(out.tireForce,state.tireForce);
    }
    if (command.action)
    {
        const bool clear=command.action==2;
        vehicle->setAccelerator(clear ? 0 : command.throttle);
        vehicle->setBrake(0,clear ? 0 : command.brake0); vehicle->setBrake(1,clear ? 0 : command.brake1);
        vehicle->setSteer(clear ? 0 : command.steer);
        if (!clear && command.direction!=-2) vehicle->setTargetGear(command.direction);
    }
    VehicleDriveState drive{}; vehicle->getDriveState(drive);
    result.throttle=drive.accelerator; result.brake0=drive.brake0; result.brake1=drive.brake1; result.steer=drive.steer;
    const ::physx::PxVec3T<float> forward=(actor->getGlobalPose()*actor->getCMassLocalPose()).q.rotate(vehicle->mInternalScene.getVehicleContext().getFrame().getLngAxis());
    result.forwardVelocity=actor->getLinearVelocity().dot(forward);
    if (engine)
    {
        result.clutch=drive.clutch; result.currentGear=drive.currentGear; result.targetGear=drive.targetGear;
        result.gearSwitchTime=drive.gearSwitchTime; result.autoboxTime=drive.autoboxTimeSinceLastShift;
        result.engineSpeed=drive.engineRotationSpeed; result.automaticTransmission=drive.automaticTransmission ? 1 : 0;
    }
    output=result;
    return VehicleResult::Success;
}
catch (...) { return VehicleResult::Error; }

bool vehicleSprungMasses(uint32_t count, const float* xyz, float mass, uint32_t axis, float* output) noexcept
{
    using namespace ::physx;
    if (!count || count>20 || !xyz || !output || !std::isfinite(mass) || mass<=0 || axis>2) return false;
    PxVec3 points[20]; float result[20]{};
    for (uint32_t i=0;i<count;++i)
    {
        points[i]=PxVec3(xyz[3*i],xyz[3*i+1],xyz[3*i+2]);
        if (!points[i].isFinite()) return false;
    }
    if (!PxVehicleComputeSprungMasses(count,points,mass,PxVehicleAxes::Enum(axis*2),result)) return false;
    double sum=0;
    for (uint32_t i=0;i<count;++i) { if (!std::isfinite(result[i]) || result[i]<0) return false; sum+=result[i]; }
    if (std::abs(sum-mass)>mass*1e-4) return false;
    std::copy_n(result,count,output); return true;
}
bool vehicleSuspensionFrame(const float* body, const float* wheel, float* output) noexcept
{
    using namespace ::physx;
    if (!body || !wheel || !output) return false;
    for (unsigned i=0;i<7;++i) if (!std::isfinite(body[i]) || !std::isfinite(wheel[i])) return false;
    const PxQuat bq(body[3],body[4],body[5],body[6]), wq(wheel[3],wheel[4],wheel[5],wheel[6]);
    if (!bq.isUnit() || !wq.isUnit()) return false;
    const ::physx::PxTransformT<float> value=PxTransform(PxVec3(body[0],body[1],body[2]),bq).transformInv(PxTransform(PxVec3(wheel[0],wheel[1],wheel[2]),wq));
    if (!value.isValid()) return false;
    const float result[]={value.p.x,value.p.y,value.p.z,value.q.x,value.q.y,value.q.z,value.q.w};
    std::copy_n(result,7,output); return true;
}
bool vehicleAckermann(float base, float width, float inner, float& output) noexcept
{
    if (!std::isfinite(base) || !std::isfinite(width) || !std::isfinite(inner) || base<=0 || width<=0 || std::abs(inner)>=1.57079632679) return false;
    const double angle=std::atan2(double(base)*std::sin(inner),double(base)*std::cos(inner)+double(width)*std::abs(std::sin(inner)));
    output=float(angle); return true;
}
}
