// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>
#include <omni/Span.h>
#include <omni/physx/IPhysxClashDetection.h>

namespace omni
{
namespace clashdetection
{

struct IClashDetectionPrivate
{
    CARB_PLUGIN_INTERFACE("omni::physx::IClashDetectionPrivate", 1, 1)

    void(CARB_ABI* test)();
    void(CARB_ABI* test2)();
    void(CARB_ABI* test3)();
    void(CARB_ABI* test4)();
    void(CARB_ABI* test5)();

    void(CARB_ABI* setSelectionWithFrustumVertices)(const carb::Float3* znear,
                                                    uint32_t numZNear,
                                                    const carb::Float3* zfar,
                                                    uint32_t numZFar);

    // Debug tools
    void(CARB_ABI* printSelection)();
    void(CARB_ABI* printSelectionHierarchy)();
    void(CARB_ABI* printSelectionChildren)();
    void(CARB_ABI* removeSelected)();
    void(CARB_ABI* removeUnselected)();
    void(CARB_ABI* removeUnselected2)();
    void(CARB_ABI* invertSelection)();
    void(CARB_ABI* printSelectionMeshInfo)();
    void(CARB_ABI* saveToBIN)();
};

} // namespace clashdetection
} // namespace omni
