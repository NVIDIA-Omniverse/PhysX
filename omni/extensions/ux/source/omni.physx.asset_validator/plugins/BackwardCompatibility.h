// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>

namespace omni
{
namespace physx
{
// piece of code to check backwards compatibility after schema changes
// This is very fragile, ideally we should use schema versions, but there is no support for that atm, so for now
// we parse the USD stage and check if there is some old schema pattern used and convert to new schema
void runBackwardCompatibilityOnStage(pxr::UsdStageWeakPtr stage);
bool checkBackwardCompatibilityOnStage(pxr::UsdStageWeakPtr stage);
const char* getBackwardsCompatibilityCheckLog();

} // namespace physx
} // namespace omni
