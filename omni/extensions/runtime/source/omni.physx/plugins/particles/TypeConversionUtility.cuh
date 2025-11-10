// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <vector_types.h>

namespace omni
{
    namespace physx
    {
        void convertVec4fToVec3fBlock(float3** out, const float4** in, const int2* offset, const size_t numBlocks, const size_t numThreadsPerBlocks);
        void convertVec3fToVec4fBlock(float4** out, const float3** in, const int2* offset, const size_t numBlocks, const size_t numThreadsPerBlocks);
    }
}
