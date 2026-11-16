// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OvStageOutputCuda.h"

#if !defined(__EMSCRIPTEN__)
#    error "The CUDA-unavailable output boundary is only for the static browser target"
#endif

namespace ovphysx::utils::detail
{
namespace
{
constexpr const char* kUnavailable = "CUDA world-transform output is unavailable in the static WASM runtime (no CUDA device)";
}

OutputCudaContextScope::OutputCudaContextScope() : OutputCudaContextScope(nullptr) {}

OutputCudaContextScope::OutputCudaContextScope(const OutputCudaBuffers* buffers)
{
    // This matches the original no-driver CPU branch. No CUDA context can exist
    // in this target; a requested GPU buffer context is never reported active.
    mReady = buffers == nullptr;
}

OutputCudaContextScope::~OutputCudaContextScope() = default;

bool OutputCudaContextScope::restoreContext() noexcept
{
    return mReady;
}

bool OutputCudaContextScope::restore(std::string& error)
{
    if (restoreContext())
        return true;
    error = kUnavailable;
    return false;
}

OutputCudaBuffers::~OutputCudaBuffers() = default;

bool OutputCudaBuffers::initialize(int, size_t, std::string& error)
{
    error = kUnavailable;
    return false;
}

bool OutputCudaBuffers::compose(const ovstage_read_column_t&,
                               const ovstage_read_column_t&,
                               const ovstage_read_column_t&,
                               std::string& error)
{
    error = kUnavailable;
    return false;
}
}
