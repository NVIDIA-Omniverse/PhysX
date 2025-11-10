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
struct ImmediateCookMeshes
{
    ImmediateShared* shared = nullptr;
    void init(ImmediateShared& immediateShared)
    {
        shared = &immediateShared;
    }

    void release()
    {
    }

    void cookMeshes(const std::vector<MeshInputView>& inputMeshView,
                    const std::vector<::physx::PxU32>& uniqueMeshIndices,
                    std::vector<MeshCookedData>& meshCookedData);
};
} // namespace graph
} // namespace physx
} // namespace omni
