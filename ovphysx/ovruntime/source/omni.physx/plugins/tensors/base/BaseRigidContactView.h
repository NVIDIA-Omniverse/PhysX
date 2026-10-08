// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-TENSOR-PATH-001
 * @covers AC-7
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-40
 */

#pragma once

#include "tensors/CommonTypes.h"
#include "tensors/base/BaseSimulationData.h"

#include <omni/physics/tensors/IRigidContactView.h>

#include <string>
#include <unordered_map>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class BaseSimulationView;

class BaseRigidContactView : public omni::physics::tensors::IRigidContactView
{
public:
    BaseRigidContactView(BaseSimulationView* sim,
                         std::vector<RigidContactSensorEntry>&& entries,
                         uint32_t numFilters,
                         uint32_t maxContactDataCount);

    virtual ~BaseRigidContactView() override;

    uint32_t getSensorCount() const override;

    uint32_t getFilterCount() const override;

    uint32_t getMaxContactDataCount() const override;

    bool check() const override;

    void release() override;

    //
    // helpers
    //
    const char* getUsdPrimName(uint32_t sensorIdx) const override;
    const char* getUsdPrimPath(uint32_t sensorIdx) const override;
    const char* getFilterUsdPrimPath(uint32_t sensorIdx, uint32_t filterIdx) const override;
    const char* getFilterUsdPrimName(uint32_t sensorIdx, uint32_t filterIdx) const override;

    void _onParentRelease();

    // Batch convert other actor IDs to paths
    void getOtherActorPathsFromIds(const TensorDesc* otherActorIdsTensor, std::vector<std::string>& outPaths) const override;

protected:
    BaseSimulationDataPtr mSimData;
    BaseSimulationView* mSim = nullptr;

    std::vector<RigidContactSensorEntry> mEntries;

    uint32_t mNumFilters = 0;
    uint32_t mMaxContactDataCount = 0;

    // Cache for uint64 path ID to string path conversion
    mutable std::unordered_map<uint64_t, std::string> mPathCache;

    // Per-(sensorIdx, filterIdx) backing storage for getFilterUsdPrimName()'s
    // returned const char* -- a leaf-name substring can't be returned directly
    // since a temporary's .c_str() would dangle. Keyed rather than a single
    // shared scratch string so a second call (e.g. filling an array of hit
    // records across a sensor/filter loop) doesn't invalidate a pointer the
    // caller already returned for a prior entry; node-based unordered_map
    // gives pointer/reference stability across further insertions, matching
    // the mPathCache pattern above.
    mutable std::unordered_map<uint64_t, std::string> mFilterNameCache;
};

} // namespace tensors
} // namespace physx
} // namespace omni
