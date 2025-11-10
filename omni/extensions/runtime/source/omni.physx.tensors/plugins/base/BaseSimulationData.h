// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../CommonTypes.h"
#include <map>
#include <memory>
#include <set>

#define CHECK_VALID_DATA_SIM_RETURN(simData_, simView, return_value)                                                   \
    if (!simData_ || !simView)                                                                                         \
        return return_value;                                                                                           \
    if (!simView->getValid())                                                                                          \
    {                                                                                                                  \
        CARB_LOG_ERROR("Simulation view object is invalidated and cannot be used again to call %s", __FUNCTION__);     \
        return return_value;                                                                                           \
    }
#define CHECK_VALID_DATA_SIM_NO_RETURN(simData_, simView)                                                              \
    if (!simData_ || !simView)                                                                                         \
        return;                                                                                                        \
    if (!simView->getValid())                                                                                          \
    {                                                                                                                  \
        CARB_LOG_ERROR("Simulation view object is invalidated and cannot be used again to call %s", __FUNCTION__);     \
        return;                                                                                                        \
    }

#define PASS_EMPTY_TENSOR(tensorDesc)                                                                                  \
    if (tensorDesc && !tensorDesc->data && getTensorTotalSize(*tensorDesc) == 0)                                       \
        return (true);

namespace omni
{
namespace physx
{
namespace tensors
{

// data shared with child views that persist even if simulation view is deleted
struct BaseSimulationData
{
    std::map<pxr::SdfPath, Subspace> mSubspaces;
    std::set<ArticulationMetatype, ArticulationMetatypeLT> mUniqueTypes;

    // bi-directional map of
    std::unordered_map<std::string, uint32_t> mUniqueRCNames2Idx;
    std::unordered_map<uint32_t, std::string> mUniqueRCIdx2Names;

    /*
    ~BaseSimulationData()
    {
        printf("+++++ Deleting base simulation data\n");
    }
    */
};

using BaseSimulationDataPtr = std::shared_ptr<BaseSimulationData>;

} // namespace tensors
} // namespace physx
} // namespace omni
