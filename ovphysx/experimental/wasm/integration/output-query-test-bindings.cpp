// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Qualification-only exports; the canonical package compiles bindings.cpp directly.
#include "bindings.cpp"

extern "C" EMSCRIPTEN_KEEPALIVE int ovp_test_refresh_output_cache()
{
    try
    {
        if (outputCache)
            outputCache->refresh();
        return 0;
    }
    catch (const std::exception& error)
    {
        return failure(error.what());
    }
}

extern "C" EMSCRIPTEN_KEEPALIVE int ovp_test_update_from_stage(int64_t first, int64_t last)
{
    if (!instance || !attached)
        return failure("Physics has no attached stage");
    return complete(ovphysx_update_from_ovstage(instance, { first, last }));
}

extern "C" EMSCRIPTEN_KEEPALIVE int ovp_test_publish_uncached(uint32_t stageHandle, int64_t ordinal)
{
    if (!instance || !attached)
        return failure("Physics has no attached stage");
    ovstage_instance_t* const stage = ovphysx_wasm_borrow_stage(stageHandle);
    const ovphysx::utils::OvStageOutputResult result =
        ovphysx::utils::writeWorldTransformsToOvstage(instance, stage, ordinal, nullptr);
    if (!result.ok())
    {
        errorText = result.message;
        return static_cast<int>(result.status);
    }
    return 0;
}
