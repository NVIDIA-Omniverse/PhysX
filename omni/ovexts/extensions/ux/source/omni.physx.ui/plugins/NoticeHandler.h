// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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
