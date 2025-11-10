// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once
#include "../MeshTypes.h"
namespace omni
{
namespace physx
{
namespace graph
{
struct ImmediateShared;
struct ImmediateHashMeshes
{
    ImmediateShared* shared = nullptr;

    // Inputs
    std::vector<MeshInputView> inputMeshView;

    // Outputs
    std::vector<MeshCookedData> meshCookedData;

    void init(ImmediateShared& immediateShared)
    {
        shared = &immediateShared;
    }

    void release()
    {
        inputMeshView.clear();
        meshCookedData.clear();
    }

    void computeInitialMeshData();
};
} // namespace graph
} // namespace physx
} // namespace omni
