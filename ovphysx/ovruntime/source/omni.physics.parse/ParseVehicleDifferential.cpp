// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>

namespace omni::physics::parse
{

namespace
{

// Maximum number of wheels per vehicle — mirrors VehicleDesc::maxNumberOfWheels (20).
constexpr int kMaxNumberOfWheels = 20;

bool fillMultiWheelFields(IPhysicsSource& /*src*/, const DifferentialInfo& info,
                          const std::string& owner, MultiWheelDifferentialDesc& out)
{
    out.wheels.reserve(info.wheels.size());
    for (int w : info.wheels)
    {
        if (w < 0)
        {
            CARB_LOG_ERROR("Usd Physics: attribute \"wheels\" of differential \"%s\" can not hold negative values.",
                           owner.c_str());
            return false;
        }
        out.wheels.push_back(w);
    }

    if (info.hasTorqueRatios)
    {
        if (info.torqueRatios.size() != out.wheels.size())
        {
            CARB_LOG_ERROR("Usd Physics: attribute \"torqueRatios\" of differential \"%s\" needs to have the same number of entries as "
                           "the \"wheels\" attribute or else should not be defined at all.", owner.c_str());
            return false;
        }
        out.torqueRatios.reserve(info.torqueRatios.size());
        for (float r : info.torqueRatios)
        {
            if (!(r >= -1.0f) || !(r < 1.0f + FLT_EPSILON))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"torqueRatios\" out of range.", owner.c_str());
                return false;
            }
            out.torqueRatios.push_back(r);
        }
    }

    if (info.hasAverageWheelSpeedRatios)
    {
        if (info.averageWheelSpeedRatios.size() != out.wheels.size())
        {
            CARB_LOG_ERROR("Usd Physics: attribute \"averageWheelSpeedRatios\" of differential \"%s\" needs to have the same number of entries as "
                           "the \"wheels\" attribute or else should not be defined at all.", owner.c_str());
            return false;
        }
        out.averageWheelSpeedRatios.reserve(info.averageWheelSpeedRatios.size());
        for (float r : info.averageWheelSpeedRatios)
        {
            if (!(r >= 0.0f) || !(r < 1.0f + FLT_EPSILON))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"averageWheelSpeedRatios\" out of range.", owner.c_str());
                return false;
            }
            out.averageWheelSpeedRatios.push_back(r);
        }
    }

    return true;
}

} // namespace

// @implements REQ-PARSE-VEH-DRIVE-001
// @covers AC-1 AC-2 AC-3
DescPtr<MultiWheelDifferentialDesc> parseMultiWheelDifferential(
    ParseContext& ctx, ObjectKey key, const DifferentialInfo& info)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerName(src.sourceKeyToString(key));

    DescPtr<MultiWheelDifferentialDesc> desc = allocateDesc<MultiWheelDifferentialDesc>(ctx.descriptorAllocator());
    if (!fillMultiWheelFields(src, info, ownerName, *desc))
        return {};
    return desc;
}

// @implements REQ-PARSE-VEH-DRIVE-001
// @covers AC-1 AC-2 AC-3
DescPtr<TankDifferentialDesc> parseTankDifferential(
    ParseContext& ctx, ObjectKey key, const DifferentialInfo& info,
    const TankDifferentialInfo& tankInfo)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerName(src.sourceKeyToString(key));

    DescPtr<TankDifferentialDesc> desc = allocateDesc<TankDifferentialDesc>(ctx.descriptorAllocator());
    if (!fillMultiWheelFields(src, info, ownerName, *desc))
        return {};

    size_t trackCount = 0;
    if (tankInfo.hasNumberOfWheelsPerTrack)
    {
        trackCount = tankInfo.numberOfWheelsPerTrack.size();
        if (trackCount > kMaxNumberOfWheels)
        {
            CARB_LOG_ERROR("Usd Physics: attribute \"numberOfWheelsPerTrack\" of differential \"%s\" can not have more than %d entries.",
                           ownerName.c_str(), kMaxNumberOfWheels);
            return {};
        }
        desc->numberOfWheelsPerTrack.reserve(trackCount);
        for (int n : tankInfo.numberOfWheelsPerTrack)
        {
            if (n < 0)
            {
                CARB_LOG_ERROR("Usd Physics: attribute \"numberOfWheelsPerTrack\" of differential \"%s\": entries must not be negative.",
                               ownerName.c_str());
                return {};
            }
            desc->numberOfWheelsPerTrack.push_back(n);
        }
    }

    if (tankInfo.hasThrustIndexPerTrack)
    {
        desc->thrustIndexPerTrack.reserve(tankInfo.thrustIndexPerTrack.size());
        for (int t : tankInfo.thrustIndexPerTrack)
        {
            if (t < 0 || t > 1)
            {
                CARB_LOG_ERROR("Usd Physics: attribute \"thrustIndexPerTrack\" of differential \"%s\" expects values that are 0 or 1.",
                               ownerName.c_str());
                return {};
            }
            desc->thrustIndexPerTrack.push_back(t);
        }
    }
    if (tankInfo.thrustIndexPerTrack.size() != trackCount)
    {
        CARB_LOG_ERROR("Usd Physics: attribute \"thrustIndexPerTrack\" of differential \"%s\" needs to have the same number of entries "
                       "as the \"numberOfWheelsPerTrack\" attribute.", ownerName.c_str());
        return {};
    }

    size_t numberOfEntriesInWheelIndexList = 0;
    if (tankInfo.hasWheelIndicesInTrackOrder)
    {
        numberOfEntriesInWheelIndexList = tankInfo.wheelIndicesInTrackOrder.size();
        if (numberOfEntriesInWheelIndexList > static_cast<size_t>(kMaxNumberOfWheels))
        {
            CARB_LOG_ERROR("Usd Physics: attribute \"wheelIndicesInTrackOrder\" of differential \"%s\" can not have more than %d entries.",
                           ownerName.c_str(), kMaxNumberOfWheels);
            return {};
        }
        desc->wheelIndicesInTrackOrder.assign(
            tankInfo.wheelIndicesInTrackOrder.begin(), tankInfo.wheelIndicesInTrackOrder.end());
    }

    if (tankInfo.hasTrackToWheelIndices)
    {
        const int trackToWheelIndicesCount = static_cast<int>(tankInfo.trackToWheelIndices.size());
        desc->trackToWheelIndices.reserve(trackToWheelIndicesCount);

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
                    const int nWheelsThisTrack = desc->numberOfWheelsPerTrack[i];
                    nextMinStartIndex = startIndex + nWheelsThisTrack;
                    if (nextMinStartIndex <= static_cast<int>(numberOfEntriesInWheelIndexList))
                    {
                        for (int j = startIndex; j < nextMinStartIndex; ++j)
                        {
                            const int wheelIndex = desc->wheelIndicesInTrackOrder[j];
                            if (wheelIndex < 0)
                            {
                                CARB_LOG_ERROR("Usd Physics: attribute \"wheelIndicesInTrackOrder\" of differential \"%s\" can not have negative "
                                               "entries.", ownerName.c_str());
                                return {};
                            }
                            const uint32_t indexBit = 1u << wheelIndex;
                            if (encounteredIndices & indexBit)
                            {
                                CARB_LOG_ERROR("Usd Physics: attribute \"wheelIndicesInTrackOrder\" of differential \"%s\" can not contain the "
                                               "same index multiple times as a wheel can only be assigned to one track.", ownerName.c_str());
                                return {};
                            }
                            encounteredIndices |= indexBit;
                        }
                    }
                    else
                    {
                        CARB_LOG_ERROR("Usd Physics: entries in attributes \"trackToWheelIndices\" and \"numberOfWheelsPerTrack\" of differential \"%s\" "
                                       "form a range that points outside the range of entries in the \"wheelIndicesInTrackOrder\".",
                                       ownerName.c_str());
                        return {};
                    }
                }
                desc->trackToWheelIndices.push_back(startIndex);
            }
            else
            {
                CARB_LOG_ERROR("Usd Physics: attribute \"trackToWheelIndices\" of differential \"%s\" expects positive values and "
                               "a sequence of non decreasing index values.", ownerName.c_str());
                return {};
            }
        }
    }
    if (tankInfo.trackToWheelIndices.size() != trackCount)
    {
        CARB_LOG_ERROR("Usd Physics: attribute \"trackToWheelIndices\" of differential \"%s\" needs to have the same number of entries "
                       "as the \"numberOfWheelsPerTrack\" attribute.", ownerName.c_str());
        return {};
    }

    return desc;
}

} // namespace omni::physics::parse
