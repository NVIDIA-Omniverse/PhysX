// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>

#include <string>
#include <vector>

namespace omni
{
namespace physx
{
namespace tensors
{

// Sentinel token used internally to mark a '**' recursive-descent segment.
// Chosen so it cannot collide with any legal USD prim-name glob.
constexpr const char* kRecursiveDescentToken = "**";

std::string makeAnchoredTokenPattern(const std::string& token);
std::vector<std::string> splitPatternRespectingGroups(const std::string& pattern);

void findMatchingUsdPaths(PXR_NS::UsdStageWeakPtr stage,
                          const std::string& pattern,
                          bool recursiveLeafPatternMatch,
                          std::vector<PXR_NS::SdfPath>& pathsRet);

} // namespace tensors
} // namespace physx
} // namespace omni
