// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "UsdPCH.h"

namespace omni
{
namespace physx
{
namespace ui
{
struct UsdNoticeListener : public PXR_NS::TfWeakBase
{
    UsdNoticeListener() = default;

    void handle(const PXR_NS::UsdNotice::ObjectsChanged& objectsChanged);
};
} // namespace ui
} // namespace physx
} // namespace omni
