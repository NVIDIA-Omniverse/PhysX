// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//


// Separate translation unit for the UsdUtilsStageCache erase, isolated to avoid
// header-interaction issues (some sidecar headers leave PXR_NS::UsdStage
// incomplete in the TUs that include them).

#include "internal/sidecar/ovphysxInternalStage.h"
#include "internal/sidecar/ovphysxInternalUtil.hpp"

#include "UsdSchemaPaths/UsdSchemaPaths.h"

#include <carb/Framework.h>
#include <carb/logging/Log.h>
#include <pxr/usd/usdUtils/stageCache.h>

#include <cinttypes>

extern "C" {

OVPHYSX_INTERNAL_API void ovphysx_close_usd_stage(int64_t stage_id) {
    if (stage_id <= 0) {
        return;
    }

    auto id = PXR_NS::UsdStageCache::Id::FromLongInt(static_cast<long int>(stage_id));
    bool erased = PXR_NS::UsdUtilsStageCache::Get().Erase(id);
    if (erased) {
        CARB_LOG_INFO("Internal sidecar: erased stage %" PRId64 " from UsdUtilsStageCache", stage_id);
    } else {
        CARB_LOG_WARN("Internal sidecar: stage %" PRId64 " not found in UsdUtilsStageCache (already erased?)", stage_id);
    }
}

} // extern "C"
