// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "CudaCommon.h"

#include <thrust/device_free.h>
#include <thrust/device_malloc.h>
#include <thrust/execution_policy.h>

namespace omni
{
namespace physx
{
namespace tensors
{

struct SingleAllocPolicy : thrust::device_execution_policy<SingleAllocPolicy>
{
    std::ptrdiff_t mBufferSize = 0;
    void* mBuffer = nullptr;
};

template <typename T>
thrust::pair<thrust::device_ptr<T>, std::ptrdiff_t> get_temporary_buffer(SingleAllocPolicy& p, std::ptrdiff_t n)
{
    if (n > p.mBufferSize)
    {
        // printf("Allocating %u bytes\n", unsigned(n));
        if (p.mBuffer)
        {
            CHECK_CUDA(cudaFree(p.mBuffer));
        }
        if (CHECK_CUDA(cudaMalloc(&p.mBuffer, n * sizeof(T))))
        {
            p.mBufferSize = n;
        }
    }
    else
    {
        // printf("Using pre-allocated\n");
    }

    return thrust::make_pair(thrust::device_pointer_cast((T*)p.mBuffer), n);

    // ask device_malloc for storage
    // thrust::pointer<T, SingleAllocPolicy> result(thrust::device_malloc<T>(n).get());
    // return thrust::make_pair(result,n);
}

template <typename Pointer>
void return_temporary_buffer(SingleAllocPolicy&, Pointer p)
{
    // printf("Freeing (not) the data\n");
    // thrust::device_free(thrust::device_pointer_cast(p.get()));
}

} // namespace tensors
} // namespace physx
} // namespace omni
