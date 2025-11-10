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
struct ImmediateCreateGeometries
{
    ImmediateShared* shared = nullptr;

    // Outputs
    std::vector<MeshGeometryData> meshGeometryData;
    std::vector<::physx::PxBounds3> meshWorldBounds;

    const bool enableTightBounds = true;
    const float boundsEpsilon = 0.01f;

    void init(ImmediateShared& immediateShared)
    {
        shared = &immediateShared;
    }

    void release()
    {
        meshGeometryData.clear();
        meshWorldBounds.clear();
    }

    void createGeometries(const std::vector<MeshCookedData>& meshCookedData,
                          std::vector<MeshInputView>& inputMeshView,
                          bool computeBoundingBoxes);
};
} // namespace graph
} // namespace physx
} // namespace omni
