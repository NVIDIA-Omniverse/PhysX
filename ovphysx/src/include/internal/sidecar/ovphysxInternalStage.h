// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "internal/sidecar/ovphysxInternalExport.h"

#include <atomic>
#include <cstdint>

// Sidecar USD stage close export. Wraps UsdUtilsStageCache::Erase so the main
// library can release a USD stage from the cache without linking USD itself.

extern "C" {

// Erase a stage from UsdUtilsStageCache. Called during stage teardown
// (detach / reset / destroy) to prevent stage lifetime leaks.
OVPHYSX_INTERNAL_API void ovphysx_close_usd_stage(int64_t stage_id);

// SDK-side function-pointer typedef (mirrors the above export) for dlsym use.
typedef void    (*OvphysxSidecarCloseUsdStageFn)(int64_t);

} // extern "C"

// Resolved sidecar function pointer (populated by loadInternalSidecar()).
extern std::atomic<OvphysxSidecarCloseUsdStageFn> g_sidecarCloseUsdStage;
