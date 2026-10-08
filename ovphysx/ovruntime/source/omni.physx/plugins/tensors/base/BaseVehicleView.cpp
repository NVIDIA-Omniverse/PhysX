// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-VEHICLE-001
 * @covers AC-1, AC-3
 */

// clang-format off
// clang-format on

#include "tensors/base/BaseVehicleView.h"

#include "internal/InternalScene.h"
#include "internal/InternalVehicle.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>
#include <omni/physics/tensors/TensorUtils.h>

using omni::physics::tensors::checkTensorDevice;
using omni::physics::tensors::checkTensorFloat32;
using omni::physics::tensors::checkTensorSizeExact;
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

namespace
{
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

bool BaseVehicleView::setWheelControlOvStage(omni::physx::internal::InternalScene& scene,
                                             const VehicleWheelOvStageRecord* const records,
                                             const uint32_t numOutputs,
                                             const WheelControl control,
                                             const TensorDesc* const srcTensor)
{
    // Same epoch gate as the getter, and for the same reason: the entries are raw InternalVehicle
    // pointers this dereferences after nothing more than a range check, so a view built against a
    // different vehicle set must not be used. Checked in every build rather than asserted.
    if (mBuiltEpoch != scene.mVehicleSetEpoch)
    {
        CARB_LOG_ERROR("vehicle wheel control: view was built at vehicle-set epoch %llu, scene now "
                       "reports %llu; nothing was written",
                       (unsigned long long)mBuiltEpoch, (unsigned long long)scene.mVehicleSetEpoch);
        return false;
    }
    if (!srcTensor || !srcTensor->data || !records)
        return false;
    if (numOutputs == 0)
        return true;
    if (!omni::physics::tensors::checkTensorDevice(*srcTensor, -1, "wheel control", __FUNCTION__) ||
        !omni::physics::tensors::checkTensorFloat32(*srcTensor, "wheel control", __FUNCTION__) ||
        !omni::physics::tensors::checkTensorSizeExact(*srcTensor, numOutputs, "wheel control", __FUNCTION__))
    {
        return false;
    }

    const float* src = static_cast<const float*>(srcTensor->data);
    for (uint32_t i = 0; i < numOutputs; ++i)
    {
        const VehicleWheelOvStageRecord& r = records[i];
        // Out of range FAILS rather than being skipped, matching the getter: the only way to land
        // here is a record list that does not describe this view, and carrying on would write some
        // of the caller's values and silently drop the rest.
        if (r.viewVehicleIdx >= mEntries.size())
        {
            CARB_LOG_ERROR("vehicle wheel control: record %u names vehicle row %u of %zu", i, r.viewVehicleIdx,
                           mEntries.size());
            return false;
        }
        const VehicleEntry& e = mEntries[r.viewVehicleIdx];
        if (!e.vehicle || r.wheelIdx >= e.numWheels)
        {
            CARB_LOG_ERROR("vehicle wheel control: record %u names wheel %u of %u", i, r.wheelIdx, e.numWheels);
            return false;
        }
        // GUARD, not an assert. InternalVehicle::setDriveTorque and its siblings CARB_ASSERT the
        // vehicle is raw-wheel-control and then static_cast unconditionally -- so in a release build,
        // where the assert is compiled out, calling one on a DRIVE vehicle is undefined behaviour.
        // The ovstage write is handed prims and cannot know the kind, so it must ask.
        //
        // A vehicle WITH a drive is accelerated and steered as a whole; its commands live on the
        // vehicle prim, which this selector does not address.
        if (!e.vehicle->isRawWheelControl())
        {
            CARB_LOG_ERROR("vehicle wheel control: '%s' applies to vehicles with NO drive, addressed per "
                           "wheel. This vehicle has a drive, so it is accelerated and steered as a whole "
                           "through commands on the vehicle prim; nothing was written.",
                           controlName(control));
            return false;
        }
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
