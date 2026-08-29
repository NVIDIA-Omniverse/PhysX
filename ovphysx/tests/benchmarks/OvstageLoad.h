// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "../c_samples/common/ovstage_sample.h"

#include <ovphysx/experimental/ovphysx.hpp>

#include <string>

inline bool benchmarkLoadUsdWithOvstage(
    ovphysx::PhysX* physx,
    const std::string& path,
    ovphysx_sample_stage_attachment_t& attachment)
{
    if (!physx)
    {
        return false;
    }
    if (attachment.stage)
    {
        ovphysx_sample_destroy_stage(physx->handle(), &attachment);
    }
    if (!ovphysx_sample_attach_usd_with_ovstage(physx->handle(), path.c_str(), &attachment))
    {
        return false;
    }
    physx->waitAll();
    return true;
}

inline void benchmarkClearOvstage(ovphysx::PhysX* physx, ovphysx_sample_stage_attachment_t& attachment)
{
    if (physx)
    {
        (void)ovphysx_reset_stage(physx->handle());
        physx->waitAll();
        ovphysx_sample_destroy_stage(physx->handle(), &attachment);
    }
    else if (attachment.stage)
    {
        ovphysx_sample_destroy_stage(0, &attachment);
    }
}
