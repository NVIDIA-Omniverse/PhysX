// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-VEHICLE-001
 * @covers AC-1, AC-3
 */

#pragma once

#include "tensors/CommonTypes.h"
#include "tensors/VehicleWheelOvStageRecord.h"

#include <omni/physics/tensors/TensorDesc.h>

#include <vector>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

// The backend's view over a scene's vehicles, so the ovstage vehicle read sources its columns from a
// view like every other type instead of walking InternalVehicle itself.
//
// A base view with no Gpu/Cpu derivatives, and permanently so: a vehicle cannot be attached to a
// DirectGPU scene, because its suspension and sticky-tire constraints are custom PxConstraints with a
// CPU solver-prep function that PhysX refuses there. That is also why only CpuSimulationView exposes
// it -- a GpuSimulationView exists only for a DirectGPU scene, so a vehicle accessor there would be
// unreachable by construction.
//
// Rows are [vehicle, wheel], matching how links index under articulations, and a read gathers the
// flat subset it wants through VehicleWheelOvStageRecord.
class BaseVehicleView
{
public:
    // Rebuild the entry list from the scene. Cheap and rare -- the caller gates it on
    // InternalScene::mVehicleSetEpoch, which is what says the rows below are renumbered.
    void rebuild(omni::physx::internal::InternalScene& scene);

    // The vehicle-set epoch the current entries were built from. A caller caching rows into this
    // view compares against InternalScene::mVehicleSetEpoch to learn they were renumbered.
    uint64_t getBuiltEpoch() const
    {
        return mBuiltEpoch;
    }

    uint32_t getCount() const
    {
        return static_cast<uint32_t>(mEntries.size());
    }

    const std::vector<VehicleEntry>& getEntries() const
    {
        return mEntries;
    }

    // One requested wheel column: which component, and where its values go.
    struct WheelTransformColumn
    {
        VehicleWheelComponent component;
        const TensorDesc* dst;
    };

    // Gather every requested component of each named wheel's WORLD transform, each into a flat host
    // column of `numOutputs * comp` float32 elements.
    //
    // ONE composition per wheel serves every requested component, straight into the destinations.
    // Composing is the expensive half of this read -- getGlobalPose, getCMassLocalPose, and either the
    // shape's local pose or the vehicle SDK's -- and it yields one PxTransform carrying both position
    // and orientation, so asking per component would run it twice and discard half of each result.
    //
    // `records` and this view's entries are valid only for the epoch getBuiltEpoch() reports: the
    // entries are raw InternalVehicle / PxRigidDynamic pointers, dereferenced after nothing more than
    // a range check. This view holds no simulation data, so `scene` stands in for the peers'
    // CHECK_VALID_DATA_SIM_RETURN: returns false, in every build, if this view was built at a
    // different epoch than the scene now reports. The contract lives HERE because the one-column form
    // below delegates -- this is where the epoch check and both record range checks run.
    bool getWheelTransformColumnsOvStage(omni::physx::internal::InternalScene& scene,
                                         const VehicleWheelOvStageRecord* records,
                                         uint32_t numOutputs,
                                         const WheelTransformColumn* columns,
                                         uint32_t numColumns) const;

    // The one-column form, taken by buildVehicleWheelState when only one wheel attribute is wanted.
    // Delegates to the multi-column gather above.
    bool getWheelTransformsOvStage(omni::physx::internal::InternalScene& scene,
                                   const VehicleWheelOvStageRecord* records,
                                   uint32_t numOutputs,
                                   VehicleWheelComponent component,
                                   const TensorDesc* dstTensor) const;

    // Which per-wheel CONTROL an ovstage column carries (ADR-0012).
    //
    // These are the controls of a vehicle WITHOUT a drive, where the control surface is per wheel.
    // A vehicle WITH a drive is steered and accelerated as a whole and its commands live on the
    // vehicle prim, which this selector does not address.
    enum class WheelControl : uint32_t
    {
        eDriveTorque = 0,
        eBrakeTorque = 1,
        eSteerAngle = 2
    };

    // Scatter one per-wheel control column into the named wheels.
    //
    // These are INPUTS the solver consumes, which is what separates them from the wheel transform
    // above: a pose is composed each step from the chassis, the suspension and the steer angle, so
    // writing one is meaningless. A drive torque is read by the next step.
    //
    // `scene` serves the same purpose it does on the getter -- this view holds no simulation data,
    // so the epoch check against it is what makes dereferencing the raw InternalVehicle pointers
    // safe. Returns false if this view was built at a different vehicle-set epoch than the scene now
    // reports.
    bool setWheelControlOvStage(omni::physx::internal::InternalScene& scene,
                                const VehicleWheelOvStageRecord* records,
                                uint32_t numOutputs,
                                WheelControl control,
                                const TensorDesc* srcTensor);

private:
    std::vector<VehicleEntry> mEntries;
    uint64_t mBuiltEpoch = 0;
};

} // namespace tensors
} // namespace physx
} // namespace omni
