// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
#include "Log.h"
#include <omni/blobkey/BlobKey.h>
#include <omni/core/BuiltIn.h>
#include <cstdlib>
#include <emscripten/emscripten.h>
namespace
{
[[noreturn]] void unsupportedService(const char* service)
{
    emscripten_log(EM_LOG_ERROR | EM_LOG_C_STACK, "%s", service);
    std::abort();
}
}
OMNI_API void* omniGetBuiltInWithoutAcquire(OmniBuiltIn builtin)
{
    switch (builtin)
    {
    case OmniBuiltIn::eILog: return ovphysxWasmLog();
    case OmniBuiltIn::eIStructuredLog: return nullptr;
    case OmniBuiltIn::eITypeFactory:
        unsupportedService("Static WASM runtime does not provide dynamic Omni type creation");
    }
    unsupportedService("Static WASM runtime requested an unknown Omni builtin service");
}
extern "C" omni::blobkey::KeyHash::Status CARB_ABI keyhash_new(omni::blobkey::KeyHash*, const uint8_t*, uint64_t, uint64_t)
{
    unsupportedService("Static WASM runtime does not support remote derived-data cache key hashing");
}
