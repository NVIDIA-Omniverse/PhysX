// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-VEHICLE-001
 * @covers AC-1, AC-3, AC-8, AC-9
 * @implements REQ-INPUT-COVERAGE-001
 * @covers AC-20, AC-21
 */

// clang-format off
// clang-format on

#include "tensors/base/BaseVehicleView.h"

#include "internal/InternalScene.h"
#include "internal/InternalVehicle.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>
#include <omni/physx/RuntimeError.h>
#include <omni/physics/tensors/TensorUtils.h>

#include <cmath>

using omni::physics::tensors::checkTensorDevice;
using omni::physics::tensors::checkTensorFloat32;
using omni::physics::tensors::checkTensorSizeExact;
using omni::physics::tensors::TensorDataType;
using omni::physics::tensors::TensorDesc;
using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

void BaseVehicleView::rebuild(omni::physx::internal::InternalScene& scene)
{
    mEntries.clear();
    mScene = &scene;
    mBuiltEpoch = scene.mVehicleSetEpoch;
    // Every vehicle, not just the enabled prefix: a disabled vehicle is still a rigid body the solver
    // moves. Its wheels report whatever the simulation holds -- for a never-enabled vehicle PxVehicle
    // has computed no wheel local pose, so all wheels coincide with the chassis.
    mEntries.reserve(scene.mVehicles.size());
    for (size_t i = 0; i < scene.mVehicles.size(); ++i)
    {
        omni::physx::internal::InternalVehicle* veh = scene.mVehicles[i];
        if (!veh)
            continue;
        PxRigidDynamic* actor = veh->getRigidDynamicActor();
        if (!actor)
            continue;
        VehicleEntry e;
        e.vehicle = veh;
        e.actor = actor;
        e.numWheels = static_cast<PxU32>(veh->mWheelTransformManagementEntries.size());
        mEntries.push_back(e);
    }
}

bool BaseVehicleView::validateScene(const omni::physx::internal::InternalScene& scene) const
{
    if (mScene != &scene || mBuiltEpoch != scene.mVehicleSetEpoch || !scene.getScene() ||
        scene.getScene()->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API))
    {
        OVX_RUNTIME_ERROR("vehicle view: stale vehicle membership or unsupported DirectGPU scene");
        return false;
    }
    return true;
}

bool BaseVehicleView::validateVehicleRow(const uint32_t row) const
{
    if (row >= mEntries.size() || !mEntries[row].vehicle || !mEntries[row].vehicle->mPhysXVehicle || !mEntries[row].actor)
    {
        OVX_RUNTIME_ERROR("vehicle view: invalid vehicle row %u", row);
        return false;
    }
    return true;
}

bool BaseVehicleView::validateWheelRecord(const VehicleWheelOvStageRecord& record) const
{
    if (!validateVehicleRow(record.viewVehicleIdx))
        return false;
    const VehicleEntry& entry = mEntries[record.viewVehicleIdx];
    if (record.wheelIdx >= entry.numWheels || record.wheelIdx >= entry.vehicle->mWheelAttachments.size() ||
        !entry.vehicle->mWheelAttachments[record.wheelIdx])
    {
        OVX_RUNTIME_ERROR("vehicle view: invalid or removed wheel %u", record.wheelIdx);
        return false;
    }
    return true;
}

bool BaseVehicleView::getWheelTransformColumnsOvStage(omni::physx::internal::InternalScene& scene,
                                                      const VehicleWheelOvStageRecord* const records,
                                                      const uint32_t numOutputs,
                                                      const WheelTransformColumn* const columns,
                                                      const uint32_t numColumns) const
{
    // The entries are raw InternalVehicle / PxRigidDynamic pointers with only a range check between
    // them and a dereference, and this view has no mSimData to revalidate them against. So the epoch
    // is checked in release too -- an assert would compile out and leave the pointers unguarded.
    if (mBuiltEpoch != scene.mVehicleSetEpoch)
    {
        CARB_LOG_ERROR("vehicle wheel transforms (ovstage): view was built at vehicle-set epoch "
                       "%llu, scene is at %llu -- re-acquire through CpuSimulationView::vehicleView "
                       "before gathering.",
                       (unsigned long long)mBuiltEpoch, (unsigned long long)scene.mVehicleSetEpoch);
        return false;
    }

    if (numColumns == 0)
        return true; // nothing to emit -- not a failure
    if (!columns)
    {
        CARB_LOG_ERROR("vehicle wheel transforms (ovstage): null columns");
        return false;
    }
    if (numOutputs != 0 && !records)
    {
        CARB_LOG_ERROR("vehicle wheel transforms (ovstage): null records");
        return false;
    }

    // Validated before the zero-row early-out: a malformed descriptor is malformed whether or not
    // there are rows to write into it. Each column logs under its own name.
    for (uint32_t c = 0; c < numColumns; ++c)
    {
        const TensorDesc* const dst = columns[c].dst;
        const uint32_t comp = vehicleWheelComponents(columns[c].component);
        const char* const attribName = columns[c].component == VehicleWheelComponent::eOrientation ?
                                           "vehicle wheel orientation (ovstage)" :
                                           "vehicle wheel position (ovstage)";
        if (!dst || !dst->data)
            return false;
        // Host tensor even on a GPU-dynamics scene: see the class comment. There is no device variant.
        if (!checkTensorDevice(*dst, -1, attribName, __FUNCTION__) ||
            !checkTensorFloat32(*dst, attribName, __FUNCTION__) ||
            !checkTensorSizeExact(*dst, numOutputs * comp, attribName, __FUNCTION__))
        {
            return false;
        }
    }

    if (numOutputs == 0)
        return true; // nothing to emit -- not a failure

    // Chassis transforms are per VEHICLE, records are per WHEEL, and the reader enumerates a
    // vehicle's wheels consecutively. Memoising the last vehicle costs one getGlobalPose per vehicle
    // instead of per wheel, and does not require sorted records -- an unsorted list just recomputes.
    uint32_t memoVehicle = 0xffffffff;
    PxTransform actor2World(PxIdentity);
    PxTransform body2World(PxIdentity);

    for (uint32_t i = 0; i < numOutputs; ++i)
    {
        const VehicleWheelOvStageRecord& r = records[i];
        if (r.viewVehicleIdx >= mEntries.size())
        {
            CARB_LOG_ERROR("vehicle wheel transforms (ovstage): record %u names vehicle row %u of %zu", i,
                           r.viewVehicleIdx, mEntries.size());
            return false;
        }
        const VehicleEntry& e = mEntries[r.viewVehicleIdx];
        if (r.wheelIdx >= e.numWheels)
        {
            CARB_LOG_ERROR("vehicle wheel transforms (ovstage): record %u names wheel %u of %u", i, r.wheelIdx,
                           e.numWheels);
            return false;
        }

        if (r.viewVehicleIdx != memoVehicle)
        {
            memoVehicle = r.viewVehicleIdx;
            actor2World = e.actor->getGlobalPose();
            body2World = actor2World * e.actor->getCMassLocalPose();
        }

        // The attachment gates BOTH branches, as in InternalScene::updateVehicleTransforms:
        // removeWheelAttachment also deletes the PhysX shapes mapped to that wheel, so the shape
        // branch on a removed attachment would read freed memory.
        PxTransform wheelPose(PxIdentity);
        if (r.wheelIdx < e.vehicle->mWheelAttachments.size() && e.vehicle->mWheelAttachments[r.wheelIdx])
        {
            // Which source applies is a property of the wheel: a wheel with a collision shape carries
            // its pose on the shape, otherwise the vehicle SDK holds it, relative to the
            // centre-of-mass frame.
            const omni::physx::internal::InternalVehicle::WheelTransformManagementEntry& w =
                e.vehicle->mWheelTransformManagementEntries[r.wheelIdx];
            if (w.shape)
                wheelPose = actor2World * w.shape->getLocalPose();
            else
                wheelPose = body2World *
                            e.vehicle->mPhysXVehicle->getWheelLocalPose(
                                e.vehicle->mWheelAttachments[r.wheelIdx]->mWheelIndex);
        }

        // One composition feeds every requested column.
        for (uint32_t c = 0; c < numColumns; ++c)
        {
            const uint32_t comp = vehicleWheelComponents(columns[c].component);
            float* const out = static_cast<float*>(columns[c].dst->data) + static_cast<size_t>(i) * comp;
            if (columns[c].component == VehicleWheelComponent::eOrientation)
            {
                out[0] = wheelPose.q.x;
                out[1] = wheelPose.q.y;
                out[2] = wheelPose.q.z;
                out[3] = wheelPose.q.w;
            }
            else
            {
                out[0] = wheelPose.p.x;
                out[1] = wheelPose.p.y;
                out[2] = wheelPose.p.z;
            }
        }
    }

    return true;
}

// Single-column convenience form, taken by buildVehicleWheelState when only one wheel attribute is
// wanted. Delegates so the epoch check, range checks and composition have one implementation.
bool BaseVehicleView::getWheelTransformsOvStage(omni::physx::internal::InternalScene& scene,
                                                const VehicleWheelOvStageRecord* const records,
                                                const uint32_t numOutputs,
                                                const VehicleWheelComponent component,
                                                const TensorDesc* const dstTensor) const
{
    const WheelTransformColumn column{ component, dstTensor };
    return getWheelTransformColumnsOvStage(scene, records, numOutputs, &column, 1);
}

BaseVehicleView::StateColumnInfo BaseVehicleView::driveStateColumnInfo(const DriveState component)
{
    switch (component)
    {
    case DriveState::eCurrentGear:
    case DriveState::eTargetGear:
        return { TensorDataType::eInt32, 1, true };
    case DriveState::eAutomaticTransmission:
        return { TensorDataType::eUint8, 1, true };
    case DriveState::eAccelerator:
    case DriveState::eBrake0:
    case DriveState::eBrake1:
    case DriveState::eSteer:
        return { TensorDataType::eFloat32, 1, false };
    case DriveState::eClutch:
    case DriveState::eEngineRotationSpeed:
    case DriveState::eGearSwitchTime:
    case DriveState::eAutoboxTimeSinceLastShift:
        return { TensorDataType::eFloat32, 1, true };
    }
    return {};
}

BaseVehicleView::StateColumnInfo BaseVehicleView::wheelStateColumnInfo(const WheelState component)
{
    switch (component)
    {
    case WheelState::eGroundActorId:
    case WheelState::eGroundShapeId:
    case WheelState::eGroundMaterialId:
        return { TensorDataType::eUint64, 1 };
    case WheelState::eIsOnGround:
        return { TensorDataType::eUint8, 1 };
    case WheelState::eGroundPlane:
        return { TensorDataType::eFloat32, 4 };
    case WheelState::eSuspensionForce:
    case WheelState::eTireLongitudinalDirection:
    case WheelState::eTireLateralDirection:
    case WheelState::eTireForce:
    case WheelState::eGroundHitPosition:
        return { TensorDataType::eFloat32, 3 };
    case WheelState::eRotationSpeed:
    case WheelState::eRotationAngle:
    case WheelState::eWheelSteerAngle:
    case WheelState::eSuspensionJounce:
    case WheelState::eTireFriction:
    case WheelState::eTireLongitudinalSlip:
    case WheelState::eTireLateralSlip:
        return { TensorDataType::eFloat32, 1 };
    }
    return {};
}

namespace
{
bool checkColumn(const TensorDesc* tensor, const TensorDataType type, const size_t size)
{
    if (!tensor || (size && !tensor->data) || tensor->numDims < 1 ||
        tensor->numDims > omni::physics::tensors::kMaxDimensions || tensor->dtype != type)
    {
        OVX_RUNTIME_ERROR("vehicle column: missing storage, invalid rank, or incorrect scalar type");
        return false;
    }
    if (!checkTensorDevice(*tensor, -1, "vehicle column", __FUNCTION__) ||
        !checkTensorSizeExact(*tensor, size, "vehicle column", __FUNCTION__))
    {
        OVX_RUNTIME_ERROR("vehicle column: expected CPU storage with exactly %zu scalar values", size);
        return false;
    }
    return true;
}

void storeFloat(const TensorDesc& tensor, const uint32_t row, const float value)
{
    static_cast<float*>(tensor.data)[row] = value;
}

void storeVector(const TensorDesc& tensor, const uint32_t row, const carb::Float3& value)
{
    float* out = static_cast<float*>(tensor.data) + size_t(row) * 3;
    out[0] = value.x;
    out[1] = value.y;
    out[2] = value.z;
}

void storeVector(const TensorDesc& tensor, const uint32_t row, const carb::Float4& value)
{
    float* out = static_cast<float*>(tensor.data) + size_t(row) * 4;
    out[0] = value.x;
    out[1] = value.y;
    out[2] = value.z;
    out[3] = value.w;
}

const char* controlName(BaseVehicleView::WheelControl c)
{
    switch (c)
    {
    case BaseVehicleView::WheelControl::eDriveTorque:
        return "driveTorque";
    case BaseVehicleView::WheelControl::eBrakeTorque:
        return "brakeTorque";
    case BaseVehicleView::WheelControl::eSteerAngle:
        return "steerAngle";
    }
    return "?";
}
} // namespace

bool BaseVehicleView::getDriveStateColumnsOvStage(omni::physx::internal::InternalScene& scene,
                                                  const uint32_t* vehicleRows,
                                                  const uint32_t numOutputs,
                                                  const DriveStateColumn* columns,
                                                  const uint32_t numColumns) const
{
    if (!validateScene(scene) || (numOutputs && !vehicleRows) || (numColumns && !columns))
        return false;
    bool engineOnly = false;
    for (uint32_t c = 0; c < numColumns; ++c)
    {
        const StateColumnInfo info = driveStateColumnInfo(columns[c].component);
        if (info.dtype == TensorDataType::eNone ||
            !checkColumn(columns[c].dst, info.dtype, size_t(numOutputs) * info.width))
            return false;
        engineOnly = engineOnly || info.engineOnly;
    }
    for (uint32_t i = 0; i < numOutputs; ++i)
    {
        if (!validateVehicleRow(vehicleRows[i]))
            return false;
        const omni::physx::internal::InternalVehicle& vehicle = *mEntries[vehicleRows[i]].vehicle;
        if (!vehicle.hasDrive() || (engineOnly && !vehicle.hasEngineDrive()))
        {
            OVX_RUNTIME_ERROR("vehicle drive state: row %u does not support the requested drive state", i);
            return false;
        }
    }
    if (!numColumns)
        return true;
    for (uint32_t i = 0; i < numOutputs; ++i)
    {
        omni::physx::VehicleDriveState state{};
        mEntries[vehicleRows[i]].vehicle->getDriveState(state);
        for (uint32_t c = 0; c < numColumns; ++c)
        {
            const TensorDesc& dst = *columns[c].dst;
            switch (columns[c].component)
            {
            case DriveState::eAccelerator:
                storeFloat(dst, i, state.accelerator);
                break;
            case DriveState::eBrake0:
                storeFloat(dst, i, state.brake0);
                break;
            case DriveState::eBrake1:
                storeFloat(dst, i, state.brake1);
                break;
            case DriveState::eSteer:
                storeFloat(dst, i, state.steer);
                break;
            case DriveState::eClutch:
                storeFloat(dst, i, state.clutch);
                break;
            case DriveState::eCurrentGear:
                static_cast<int32_t*>(dst.data)[i] = state.currentGear;
                break;
            case DriveState::eTargetGear:
                static_cast<int32_t*>(dst.data)[i] = state.targetGear;
                break;
            case DriveState::eEngineRotationSpeed:
                storeFloat(dst, i, state.engineRotationSpeed);
                break;
            case DriveState::eAutomaticTransmission:
                static_cast<uint8_t*>(dst.data)[i] = state.automaticTransmission ? 1 : 0;
                break;
            case DriveState::eGearSwitchTime:
                storeFloat(dst, i, state.gearSwitchTime);
                break;
            case DriveState::eAutoboxTimeSinceLastShift:
                storeFloat(dst, i, state.autoboxTimeSinceLastShift);
                break;
            }
        }
    }
    return true;
}

bool BaseVehicleView::getWheelStateColumnsOvStage(omni::physx::internal::InternalScene& scene,
                                                  const VehicleWheelOvStageRecord* records,
                                                  const uint32_t numOutputs,
                                                  const WheelStateColumn* columns,
                                                  const uint32_t numColumns) const
{
    if (!validateScene(scene) || (numOutputs && !records) || (numColumns && !columns))
        return false;
    for (uint32_t c = 0; c < numColumns; ++c)
    {
        const StateColumnInfo info = wheelStateColumnInfo(columns[c].component);
        if (info.dtype == TensorDataType::eNone ||
            !checkColumn(columns[c].dst, info.dtype, size_t(numOutputs) * info.width))
            return false;
    }
    for (uint32_t i = 0; i < numOutputs; ++i)
        if (!validateWheelRecord(records[i]))
            return false;
    if (!numColumns)
        return true;
    for (uint32_t i = 0; i < numOutputs; ++i)
    {
        omni::physx::VehicleWheelState state{};
        mEntries[records[i].viewVehicleIdx].vehicle->getWheelState(records[i].wheelIdx, state);
        if (!state.isOnGround)
        {
            state.groundPlane = {};
            state.groundHitPosition = {};
        }
        for (uint32_t c = 0; c < numColumns; ++c)
        {
            const TensorDesc& dst = *columns[c].dst;
            switch (columns[c].component)
            {
            case WheelState::eRotationSpeed:
                storeFloat(dst, i, state.rotationSpeed);
                break;
            case WheelState::eRotationAngle:
                storeFloat(dst, i, state.rotationAngle);
                break;
            case WheelState::eWheelSteerAngle:
                storeFloat(dst, i, state.steerAngle);
                break;
            case WheelState::eSuspensionJounce:
                storeFloat(dst, i, state.suspensionJounce);
                break;
            case WheelState::eSuspensionForce:
                storeVector(dst, i, state.suspensionForce);
                break;
            case WheelState::eTireFriction:
                storeFloat(dst, i, state.tireFriction);
                break;
            case WheelState::eTireLongitudinalDirection:
                storeVector(dst, i, state.tireLongitudinalDirection);
                break;
            case WheelState::eTireLateralDirection:
                storeVector(dst, i, state.tireLateralDirection);
                break;
            case WheelState::eTireLongitudinalSlip:
                storeFloat(dst, i, state.tireLongitudinalSlip);
                break;
            case WheelState::eTireLateralSlip:
                storeFloat(dst, i, state.tireLateralSlip);
                break;
            case WheelState::eTireForce:
                storeVector(dst, i, state.tireForce);
                break;
            case WheelState::eIsOnGround:
                static_cast<uint8_t*>(dst.data)[i] = state.isOnGround ? 1 : 0;
                break;
            case WheelState::eGroundPlane:
                storeVector(dst, i, state.groundPlane);
                break;
            case WheelState::eGroundHitPosition:
                storeVector(dst, i, state.groundHitPosition);
                break;
            case WheelState::eGroundActorId:
                static_cast<uint64_t*>(dst.data)[i] = state.groundActor;
                break;
            case WheelState::eGroundShapeId:
                static_cast<uint64_t*>(dst.data)[i] = state.groundShape;
                break;
            case WheelState::eGroundMaterialId:
                static_cast<uint64_t*>(dst.data)[i] = state.groundMaterial;
                break;
            }
        }
    }
    return true;
}

bool BaseVehicleView::setVehicleControlOvStage(omni::physx::internal::InternalScene& scene,
                                               const uint32_t* vehicleRows,
                                               const uint32_t numOutputs,
                                               const VehicleControl control,
                                               const TensorDesc* srcTensor)
{
    if (!validateScene(scene) || (numOutputs && !vehicleRows))
        return false;
    TensorDataType type = TensorDataType::eFloat32;
    switch (control)
    {
    case VehicleControl::eRequestedGear:
        type = TensorDataType::eInt32;
        break;
    case VehicleControl::eResetToRestState:
        type = TensorDataType::eUint8;
        break;
    case VehicleControl::eAccelerator:
    case VehicleControl::eBrake0:
    case VehicleControl::eBrake1:
    case VehicleControl::eSteer:
    case VehicleControl::eThrust0:
    case VehicleControl::eThrust1:
        break;
    default:
        OVX_RUNTIME_ERROR("vehicle control: unknown control");
        return false;
    }
    if (!checkColumn(srcTensor, type, numOutputs))
        return false;

    // All validation precedes all mutations, including reset's internal state changes before its
    // native velocity setters. A disabled vehicle controller is allowed; a disabled chassis is not.
    for (uint32_t i = 0; i < numOutputs; ++i)
    {
        if (!validateVehicleRow(vehicleRows[i]))
            return false;
        const VehicleEntry& entry = mEntries[vehicleRows[i]];
        if (control == VehicleControl::eResetToRestState)
        {
            if (static_cast<const uint8_t*>(srcTensor->data)[i] > 1 ||
                entry.actor->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION))
            {
                OVX_RUNTIME_ERROR("vehicle reset: row %u requires 0 or 1 and a chassis with simulation enabled", i);
                return false;
            }
            continue;
        }
        if (!entry.vehicle->hasDrive() ||
            ((control == VehicleControl::eThrust0 || control == VehicleControl::eThrust1) && !entry.vehicle->isTank()))
        {
            OVX_RUNTIME_ERROR("vehicle control: row %u has an incompatible drive type", i);
            return false;
        }
        if (control == VehicleControl::eRequestedGear)
        {
            const int32_t value = static_cast<const int32_t*>(srcTensor->data)[i];
            constexpr int32_t highestGear = PxVehicleGearboxParams::eMAX_NB_GEARS - 2;
            constexpr int32_t automaticGear = PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR;
            if (value != automaticGear && (value < -1 || value > highestGear))
            {
                OVX_RUNTIME_ERROR("vehicle requestedGear: row %u requires [-1, %d] or %d", i, highestGear, automaticGear);
                return false;
            }
        }
        else
        {
            const float value = static_cast<const float*>(srcTensor->data)[i];
            const float minimum = control == VehicleControl::eSteer || control == VehicleControl::eThrust0 ||
                                          control == VehicleControl::eThrust1 ?
                                      -1.0f :
                                      0.0f;
            if (!std::isfinite(value) || value < minimum || value > 1.0f)
            {
                OVX_RUNTIME_ERROR("vehicle control: row %u requires a finite value in [%g, 1]", i, minimum);
                return false;
            }
        }
    }

    std::vector<uint8_t> resetApplied(control == VehicleControl::eResetToRestState ? mEntries.size() : 0, 0);
    for (uint32_t i = 0; i < numOutputs; ++i)
    {
        omni::physx::internal::InternalVehicle& vehicle = *mEntries[vehicleRows[i]].vehicle;
        switch (control)
        {
        case VehicleControl::eAccelerator:
            vehicle.setAccelerator(static_cast<const float*>(srcTensor->data)[i]);
            break;
        case VehicleControl::eBrake0:
            vehicle.setBrake(0, static_cast<const float*>(srcTensor->data)[i]);
            break;
        case VehicleControl::eBrake1:
            vehicle.setBrake(1, static_cast<const float*>(srcTensor->data)[i]);
            break;
        case VehicleControl::eSteer:
            vehicle.setSteer(static_cast<const float*>(srcTensor->data)[i]);
            break;
        case VehicleControl::eRequestedGear:
            vehicle.setTargetGear(static_cast<const int32_t*>(srcTensor->data)[i]);
            break;
        case VehicleControl::eThrust0:
            vehicle.setThrust(0, static_cast<const float*>(srcTensor->data)[i]);
            break;
        case VehicleControl::eThrust1:
            vehicle.setThrust(1, static_cast<const float*>(srcTensor->data)[i]);
            break;
        case VehicleControl::eResetToRestState:
            if (static_cast<const uint8_t*>(srcTensor->data)[i] && !resetApplied[vehicleRows[i]])
            {
                vehicle.setToRestState();
                resetApplied[vehicleRows[i]] = 1;
            }
            break;
        }
    }
    return true;
}

bool BaseVehicleView::setWheelControlOvStage(omni::physx::internal::InternalScene& scene,
                                             const VehicleWheelOvStageRecord* const records,
                                             const uint32_t numOutputs,
                                             const WheelControl control,
                                             const TensorDesc* const srcTensor)
{
    // Same epoch gate as the getter, and for the same reason: the entries are raw InternalVehicle
    // pointers this dereferences after nothing more than a range check, so a view built against a
    // different vehicle set must not be used. Checked in every build rather than asserted.
    if (!validateScene(scene))
        return false;
    if ((numOutputs && !records) || !checkColumn(srcTensor, TensorDataType::eFloat32, numOutputs) ||
        (control != WheelControl::eDriveTorque && control != WheelControl::eBrakeTorque &&
         control != WheelControl::eSteerAngle))
    {
        return false;
    }

    const float* src = static_cast<const float*>(srcTensor->data);
    for (uint32_t i = 0; i < numOutputs; ++i)
    {
        const VehicleWheelOvStageRecord& r = records[i];
        if (!validateWheelRecord(r))
            return false;
        const VehicleEntry& e = mEntries[r.viewVehicleIdx];
        // GUARD, not an assert. InternalVehicle::setDriveTorque and its siblings CARB_ASSERT the
        // vehicle is raw-wheel-control and then static_cast unconditionally -- so in a release build,
        // where the assert is compiled out, calling one on a DRIVE vehicle is undefined behaviour.
        // The ovstage write is handed prims and cannot know the kind, so it must ask.
        //
        // A vehicle WITH a drive is accelerated and steered as a whole; its commands live on the
        // vehicle prim, which this selector does not address.
        if (!e.vehicle->isRawWheelControl())
        {
            OVX_RUNTIME_ERROR("vehicle wheel control: '%s' applies to vehicles with NO drive, addressed per "
                           "wheel. This vehicle has a drive, so it is accelerated and steered as a whole "
                           "through commands on the vehicle prim; nothing was written.",
                           controlName(control));
            return false;
        }
        if (!std::isfinite(src[i]) || (control == WheelControl::eBrakeTorque && src[i] < 0.0f))
        {
            OVX_RUNTIME_ERROR("vehicle wheel control: '%s' row %u requires finite values and nonnegative brake torque",
                              controlName(control), i);
            return false;
        }
    }
    for (uint32_t i = 0; i < numOutputs; ++i)
    {
        const VehicleWheelOvStageRecord& r = records[i];
        const VehicleEntry& e = mEntries[r.viewVehicleIdx];
        switch (control)
        {
        case WheelControl::eDriveTorque:
            e.vehicle->setDriveTorque(r.wheelIdx, src[i]);
            break;
        case WheelControl::eBrakeTorque:
            e.vehicle->setBrakeTorque(r.wheelIdx, src[i]);
            break;
        case WheelControl::eSteerAngle:
            e.vehicle->setSteerAngle(r.wheelIdx, src[i]);
            break;
        }
    }
    return true;
}

} // namespace tensors
} // namespace physx
} // namespace omni
