// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-PARSE-VEH-DRIVE-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleMultiWheelDifferentialAPI + PhysxVehicleTankDifferentialAPI
// readers. Walker pre-reads VtArray fields (wheels VtArray<int>;
// torqueRatios / averageWheelSpeedRatios / numberOfWheelsPerTrack /
// thrustIndexPerTrack / wheelIndicesInTrackOrder / trackToWheelIndices
// VtArray<int|float>) and passes via DifferentialInfo.
//
// Validation rules: wheel indices non-negative, torque/avg-speed ratio
// ranges, track-count cross-checks, and `trackToWheelIndices` defining
// non-negative, non-overlapping in-bounds ranges (each start index
// must be >= the previous track's end).

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedVehicle.h>
#include <stdexcept>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>

namespace omni::physics::parse
{

namespace
{

// Maximum number of wheels per vehicle — mirrors VehicleDesc::maxNumberOfWheels (20).
constexpr int kMaxNumberOfWheels = 20;

bool fillMultiWheelFields(PreparedVehicleReader& /*src*/, const DifferentialInfo& info,
                          const std::string& owner, MultiWheelDifferentialDesc& out, VehicleFillStatus& status)
{
    VehicleOutputVector<typename decltype(out.wheels)::value_type> wheelsOut(out.wheels, vehicle_output_slot::wheels, status);
    VehicleOutputVector<typename decltype(out.torqueRatios)::value_type> torqueRatiosOut(out.torqueRatios, vehicle_output_slot::torqueRatios, status);
    VehicleOutputVector<typename decltype(out.averageWheelSpeedRatios)::value_type> averageWheelSpeedRatiosOut(out.averageWheelSpeedRatios, vehicle_output_slot::averageWheelSpeedRatios, status);
    wheelsOut.reserve(info.wheels.size());
    for (int w : info.wheels)
    {
        if (w < 0)
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"wheels\" of differential \"%s\" can not hold negative values.",
                           owner.c_str());
            return false;
        }
        wheelsOut.push_back(w);
    }

    if (info.hasTorqueRatios)
    {
        if (info.torqueRatios.size() != wheelsOut.size())
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"torqueRatios\" of differential \"%s\" needs to have the same number of entries as "
                           "the \"wheels\" attribute or else should not be defined at all.", owner.c_str());
            return false;
        }
        torqueRatiosOut.reserve(info.torqueRatios.size());
        for (float r : info.torqueRatios)
        {
            if (!(r >= -1.0f) || !(r < 1.0f + FLT_EPSILON))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"torqueRatios\" out of range.", owner.c_str());
                return false;
            }
            torqueRatiosOut.push_back(r);
        }
    }

    if (info.hasAverageWheelSpeedRatios)
    {
        if (info.averageWheelSpeedRatios.size() != wheelsOut.size())
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"averageWheelSpeedRatios\" of differential \"%s\" needs to have the same number of entries as "
                           "the \"wheels\" attribute or else should not be defined at all.", owner.c_str());
            return false;
        }
        averageWheelSpeedRatiosOut.reserve(info.averageWheelSpeedRatios.size());
        for (float r : info.averageWheelSpeedRatios)
        {
            if (!(r >= 0.0f) || !(r < 1.0f + FLT_EPSILON))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"averageWheelSpeedRatios\" out of range.", owner.c_str());
                return false;
            }
            averageWheelSpeedRatiosOut.push_back(r);
        }
    }

    return true;
}

} // namespace

// @implements REQ-PARSE-VEH-DRIVE-001
// @covers AC-1 AC-2 AC-3
PreparedMultiWheelDifferential prepareMultiWheelDifferential(ParseContext& ctx, ObjectKey key, const DifferentialInfo& info)
{
    PreparedMultiWheelDifferential input;
    input.key = key;
    input.info = info;
    input.reads = prepareVehicleReads(ctx.source(), key, {

    });
    return input;
}

void prepareMultiWheelDifferentialStorage(const PreparedMultiWheelDifferential& input, MultiWheelDifferentialDesc& output)
{
    output.wheels.resize(input.info.wheels.size());
    output.torqueRatios.resize(input.info.hasTorqueRatios ? input.info.torqueRatios.size() : 0);
    output.averageWheelSpeedRatios.resize(input.info.hasAverageWheelSpeedRatios ? input.info.averageWheelSpeedRatios.size() : 0);
}

bool fillMultiWheelDifferential(const PreparedMultiWheelDifferential& input, MultiWheelDifferentialDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    [[maybe_unused]] ObjectKey key = input.key;
    [[maybe_unused]] const DifferentialInfo& info = input.info;


    const std::string& ownerName = src.sourceKeyToString(key);

    MultiWheelDifferentialDesc* desc = &output;
    if (!fillMultiWheelFields(src, info, ownerName, *desc, status))
        return false;
    return true;
}

void finishMultiWheelDifferentialStorage(MultiWheelDifferentialDesc& output, const VehicleFillStatus& status)
{
    output.wheels.resize(status.written[vehicle_output_slot::wheels]);
    output.torqueRatios.resize(status.written[vehicle_output_slot::torqueRatios]);
    output.averageWheelSpeedRatios.resize(status.written[vehicle_output_slot::averageWheelSpeedRatios]);
}

DescPtr<MultiWheelDifferentialDesc> parseMultiWheelDifferential(ParseContext& ctx, ObjectKey key, const DifferentialInfo& info)
{
    const PreparedMultiWheelDifferential input = prepareMultiWheelDifferential(ctx, key, info);
    DescPtr<MultiWheelDifferentialDesc> output = allocateDesc<MultiWheelDifferentialDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareMultiWheelDifferentialStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillMultiWheelDifferential(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishMultiWheelDifferentialStorage(*output, status);
    return output;
}


// @implements REQ-PARSE-VEH-DRIVE-001
// @covers AC-1 AC-2 AC-3
PreparedTankDifferential prepareTankDifferential(ParseContext& ctx, ObjectKey key, const DifferentialInfo& info, const TankDifferentialInfo& tankInfo)
{
    PreparedTankDifferential input;
    input.key = key;
    input.info = info;
    input.tankInfo = tankInfo;
    input.reads = prepareVehicleReads(ctx.source(), key, {

    });
    return input;
}

void prepareTankDifferentialStorage(const PreparedTankDifferential& input, TankDifferentialDesc& output)
{
    output.wheels.resize(input.info.wheels.size());
    output.torqueRatios.resize(input.info.hasTorqueRatios ? input.info.torqueRatios.size() : 0);
    output.averageWheelSpeedRatios.resize(input.info.hasAverageWheelSpeedRatios ? input.info.averageWheelSpeedRatios.size() : 0);
    output.numberOfWheelsPerTrack.resize(input.tankInfo.hasNumberOfWheelsPerTrack ? input.tankInfo.numberOfWheelsPerTrack.size() : 0);
    output.thrustIndexPerTrack.resize(input.tankInfo.hasThrustIndexPerTrack ? input.tankInfo.thrustIndexPerTrack.size() : 0);
    output.wheelIndicesInTrackOrder.resize(input.tankInfo.hasWheelIndicesInTrackOrder ? input.tankInfo.wheelIndicesInTrackOrder.size() : 0);
    output.trackToWheelIndices.resize(input.tankInfo.hasTrackToWheelIndices ? input.tankInfo.trackToWheelIndices.size() : 0);
}

bool fillTankDifferential(const PreparedTankDifferential& input, TankDifferentialDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    [[maybe_unused]] ObjectKey key = input.key;
    [[maybe_unused]] const DifferentialInfo& info = input.info;
    [[maybe_unused]] const TankDifferentialInfo& tankInfo = input.tankInfo;
    VehicleOutputVector<typename decltype(output.numberOfWheelsPerTrack)::value_type> numberOfWheelsPerTrackOut(output.numberOfWheelsPerTrack, vehicle_output_slot::numberOfWheelsPerTrack, status);
    VehicleOutputVector<typename decltype(output.thrustIndexPerTrack)::value_type> thrustIndexPerTrackOut(output.thrustIndexPerTrack, vehicle_output_slot::thrustIndexPerTrack, status);
    VehicleOutputVector<typename decltype(output.wheelIndicesInTrackOrder)::value_type> wheelIndicesInTrackOrderOut(output.wheelIndicesInTrackOrder, vehicle_output_slot::wheelIndicesInTrackOrder, status);
    VehicleOutputVector<typename decltype(output.trackToWheelIndices)::value_type> trackToWheelIndicesOut(output.trackToWheelIndices, vehicle_output_slot::trackToWheelIndices, status);


    const std::string& ownerName = src.sourceKeyToString(key);

    TankDifferentialDesc* desc = &output;
    if (!fillMultiWheelFields(src, info, ownerName, *desc, status))
        return false;

    size_t trackCount = 0;
    if (tankInfo.hasNumberOfWheelsPerTrack)
    {
        trackCount = tankInfo.numberOfWheelsPerTrack.size();
        if (trackCount > kMaxNumberOfWheels)
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"numberOfWheelsPerTrack\" of differential \"%s\" can not have more than %d entries.",
                           ownerName.c_str(), kMaxNumberOfWheels);
            return false;
        }
        numberOfWheelsPerTrackOut.reserve(trackCount);
        for (int n : tankInfo.numberOfWheelsPerTrack)
        {
            if (n < 0)
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"numberOfWheelsPerTrack\" of differential \"%s\": entries must not be negative.",
                               ownerName.c_str());
                return false;
            }
            numberOfWheelsPerTrackOut.push_back(n);
        }
    }

    if (tankInfo.hasThrustIndexPerTrack)
    {
        thrustIndexPerTrackOut.reserve(tankInfo.thrustIndexPerTrack.size());
        for (int t : tankInfo.thrustIndexPerTrack)
        {
            if (t < 0 || t > 1)
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"thrustIndexPerTrack\" of differential \"%s\" expects values that are 0 or 1.",
                               ownerName.c_str());
                return false;
            }
            thrustIndexPerTrackOut.push_back(t);
        }
    }
    if (tankInfo.thrustIndexPerTrack.size() != trackCount)
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"thrustIndexPerTrack\" of differential \"%s\" needs to have the same number of entries "
                       "as the \"numberOfWheelsPerTrack\" attribute.", ownerName.c_str());
        return false;
    }

    size_t numberOfEntriesInWheelIndexList = 0;
    if (tankInfo.hasWheelIndicesInTrackOrder)
    {
        numberOfEntriesInWheelIndexList = tankInfo.wheelIndicesInTrackOrder.size();
        if (numberOfEntriesInWheelIndexList > static_cast<size_t>(kMaxNumberOfWheels))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"wheelIndicesInTrackOrder\" of differential \"%s\" can not have more than %d entries.",
                           ownerName.c_str(), kMaxNumberOfWheels);
            return false;
        }
        wheelIndicesInTrackOrderOut.assign(
            tankInfo.wheelIndicesInTrackOrder.begin(), tankInfo.wheelIndicesInTrackOrder.end());
    }

    if (tankInfo.hasTrackToWheelIndices)
    {
        const int trackToWheelIndicesCount = static_cast<int>(tankInfo.trackToWheelIndices.size());
        trackToWheelIndicesOut.reserve(trackToWheelIndicesCount);

        static_assert(kMaxNumberOfWheels <= 32, "");  // bit-mask logic
        uint32_t encounteredIndices = 0;
        int nextMinStartIndex = 0;
        for (int i = 0; i < trackToWheelIndicesCount; ++i)
        {
            const int startIndex = tankInfo.trackToWheelIndices[i];

            if (startIndex >= nextMinStartIndex)
            {
                if (i < static_cast<int>(trackCount))
                {
                    const int nWheelsThisTrack = numberOfWheelsPerTrackOut[i];
                    nextMinStartIndex = startIndex + nWheelsThisTrack;
                    if (nextMinStartIndex <= static_cast<int>(numberOfEntriesInWheelIndexList))
                    {
                        for (int j = startIndex; j < nextMinStartIndex; ++j)
                        {
                            const int wheelIndex = wheelIndicesInTrackOrderOut[j];
                            if (wheelIndex < 0 || wheelIndex >= kMaxNumberOfWheels)
                            {
                                VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"wheelIndicesInTrackOrder\" of differential \"%s\" must contain indices "
                                               "in [0, %d).", ownerName.c_str(), kMaxNumberOfWheels);
                                return false;
                            }
                            const uint32_t indexBit = 1u << wheelIndex;
                            if (encounteredIndices & indexBit)
                            {
                                VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"wheelIndicesInTrackOrder\" of differential \"%s\" can not contain the "
                                               "same index multiple times as a wheel can only be assigned to one track.", ownerName.c_str());
                                return false;
                            }
                            encounteredIndices |= indexBit;
                        }
                    }
                    else
                    {
                        VEHICLE_LOG_ERROR(status, "Usd Physics: entries in attributes \"trackToWheelIndices\" and \"numberOfWheelsPerTrack\" of differential \"%s\" "
                                       "form a range that points outside the range of entries in the \"wheelIndicesInTrackOrder\".",
                                       ownerName.c_str());
                        return false;
                    }
                }
                trackToWheelIndicesOut.push_back(startIndex);
            }
            else
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"trackToWheelIndices\" of differential \"%s\" expects positive values and "
                               "a sequence of non decreasing index values.", ownerName.c_str());
                return false;
            }
        }
    }
    if (tankInfo.trackToWheelIndices.size() != trackCount)
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"trackToWheelIndices\" of differential \"%s\" needs to have the same number of entries "
                       "as the \"numberOfWheelsPerTrack\" attribute.", ownerName.c_str());
        return false;
    }

    return true;
}

void finishTankDifferentialStorage(TankDifferentialDesc& output, const VehicleFillStatus& status)
{
    output.wheels.resize(status.written[vehicle_output_slot::wheels]);
    output.torqueRatios.resize(status.written[vehicle_output_slot::torqueRatios]);
    output.averageWheelSpeedRatios.resize(status.written[vehicle_output_slot::averageWheelSpeedRatios]);
    output.numberOfWheelsPerTrack.resize(status.written[vehicle_output_slot::numberOfWheelsPerTrack]);
    output.thrustIndexPerTrack.resize(status.written[vehicle_output_slot::thrustIndexPerTrack]);
    output.wheelIndicesInTrackOrder.resize(status.written[vehicle_output_slot::wheelIndicesInTrackOrder]);
    output.trackToWheelIndices.resize(status.written[vehicle_output_slot::trackToWheelIndices]);
}

DescPtr<TankDifferentialDesc> parseTankDifferential(ParseContext& ctx, ObjectKey key, const DifferentialInfo& info, const TankDifferentialInfo& tankInfo)
{
    const PreparedTankDifferential input = prepareTankDifferential(ctx, key, info, tankInfo);
    DescPtr<TankDifferentialDesc> output = allocateDesc<TankDifferentialDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareTankDifferentialStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillTankDifferential(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishTankDifferentialStorage(*output, status);
    return output;
}


} // namespace omni::physics::parse
