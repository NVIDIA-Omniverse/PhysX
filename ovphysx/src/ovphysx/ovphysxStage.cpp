// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// SDK-side counterpart to the sidecar's close USD stage export
// (see src/ovphysxInternal/ovphysxInternalStage.cpp).
//
// This trampoline forwards to the resolved sidecar function pointer; it exists
// because only the sidecar links USD, so the SDK side can't erase stages from
// UsdUtilsStageCache directly. The instance-destroy / unload path uses it to
// release the attached USD stage from the cache. Best-effort.

#include "internal/sdk/ovphysxSDKSidecarLoader.hpp"
#include "internal/sidecar/ovphysxInternalStage.h"  // g_sidecarCloseUsdStage

#include <carb/Framework.h>
#include <carb/logging/Log.h>

// Sidecar stage-close atomic owned here; the loader writes it during
// loadInternalSidecar() via the extern in ovphysxInternalStage.h.
std::atomic<OvphysxSidecarCloseUsdStageFn>          g_sidecarCloseUsdStage{nullptr};

void ovphysx_close_usd_stage_wrapper(int64_t stage_id) {
    if (auto closeFn = g_sidecarCloseUsdStage.load(std::memory_order_acquire)) {
        closeFn(stage_id);
    }
}
