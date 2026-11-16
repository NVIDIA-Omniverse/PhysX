// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <cuda_runtime_api.h>

#if !defined(__EMSCRIPTEN__)
#error "Browser-only unavailable CUDA adapter"
#endif

// The shared OVStage image owns the other CUDA runtime symbols. These are the
// two additional device operations referenced by OvPhysX.
extern "C"
{
cudaError_t CUDARTAPI cudaMemset(void*, int, size_t) { return cudaErrorNoDevice; }
cudaError_t CUDARTAPI cudaMemsetAsync(void*, int, size_t, cudaStream_t) { return cudaErrorNoDevice; }
}
