// SPDX-FileCopyrightText: Copyright (c) 2014-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include "NvFlowExt.h"

#include "LuidTable.h"

#include "NvFlowMath.h"
#include "NvFlowUploadBuffer.h"

namespace
{
    NvFlowInt4 emitterLocationMin(NvFlowInt4 a, NvFlowInt4 b)
    {
        return NvFlowInt4{
            a.x < b.x ? a.x : b.x,
            a.y < b.y ? a.y : b.y,
            a.z < b.z ? a.z : b.z,
            a.w < b.w ? a.w : b.w
        };
    }

    NvFlowInt4 emitterLocationMax(NvFlowInt4 a, NvFlowInt4 b)
    {
        return NvFlowInt4{
            a.x > b.x ? a.x : b.x,
            a.y > b.y ? a.y : b.y,
            a.z > b.z ? a.z : b.z,
            a.w > b.w ? a.w : b.w
        };
    }

    void computeEmitterBoxBounds(
        NvFlowFloat4x4 localToWorld,
        int layer,
        int level,
        NvFlowFloat3 localPosition,
        NvFlowFloat3 localHalfSize,
        NvFlowFloat3 blockSizeWorld,
        NvFlowInt4* pLocationMin,
        NvFlowInt4* pLocationMax
    )
    {
        using namespace NvFlowMath;

        NvFlowFloat3 pos_min = {
            -localHalfSize.x + localPosition.x,
            -localHalfSize.y + localPosition.y,
            -localHalfSize.z + localPosition.z
        };
        NvFlowFloat3 pos_max = {
            +localHalfSize.x + localPosition.x,
            +localHalfSize.y + localPosition.y,
            +localHalfSize.z + localPosition.z
        };

        NvFlowFloat4 bounds[8u] = {
            vector4Transform(NvFlowFloat4{pos_min.x, pos_min.y, pos_min.z, 1.f}, localToWorld),
            vector4Transform(NvFlowFloat4{pos_max.x, pos_min.y, pos_min.z, 1.f}, localToWorld),
            vector4Transform(NvFlowFloat4{pos_max.x, pos_max.y, pos_min.z, 1.f}, localToWorld),
            vector4Transform(NvFlowFloat4{pos_min.x, pos_max.y, pos_min.z, 1.f}, localToWorld),
            vector4Transform(NvFlowFloat4{pos_min.x, pos_min.y, pos_max.z, 1.f}, localToWorld),
            vector4Transform(NvFlowFloat4{pos_max.x, pos_min.y, pos_max.z, 1.f}, localToWorld),
            vector4Transform(NvFlowFloat4{pos_max.x, pos_max.y, pos_max.z, 1.f}, localToWorld),
            vector4Transform(NvFlowFloat4{pos_min.x, pos_max.y, pos_max.z, 1.f}, localToWorld)
        };
        // normalize
        for (NvFlowUint idx = 0u; idx < 8u; idx++)
        {
            if (bounds[idx].w > 0.f)
            {
                bounds[idx].x /= bounds[idx].w;
                bounds[idx].y /= bounds[idx].w;
                bounds[idx].z /= bounds[idx].w;
            }
        }

        NvFlowFloat4 minWorldf = bounds[0u];
        NvFlowFloat4 maxWorldf = bounds[0u];
        for (NvFlowUint idx = 1u; idx < 8u; idx++)
        {
            minWorldf = vectorMin(minWorldf, bounds[idx]);
            maxWorldf = vectorMax(maxWorldf, bounds[idx]);
        }

        NvFlowFloat3 minLocationf = { minWorldf.x / blockSizeWorld.x, minWorldf.y / blockSizeWorld.y, minWorldf.z / blockSizeWorld.z };
        NvFlowFloat3 maxLocationf = { maxWorldf.x / blockSizeWorld.x, maxWorldf.y / blockSizeWorld.y, maxWorldf.z / blockSizeWorld.z };

        NvFlowInt4 minLocation = {
            int(floorf(minLocationf.x)),
            int(floorf(minLocationf.y)),
            int(floorf(minLocationf.z)),
            NvFlow_packLayerAndLevel(layer, level)
        };
        NvFlowInt4 maxLocation = {
            int(-floorf(-maxLocationf.x)),
            int(-floorf(-maxLocationf.y)),
            int(-floorf(-maxLocationf.z)),
            NvFlow_packLayerAndLevel(layer + 1, level)
        };

        *pLocationMin = minLocation;
        *pLocationMax = maxLocation;
    }

    template <typename T>
    void arrayShift(T*& elements, NvFlowUint64& elementCount, NvFlowUint64 batchIdx, NvFlowUint64 batchSize)
    {
        NvFlowUint64 shiftAmount = batchIdx * batchSize;
        if (shiftAmount < elementCount)
        {
            if (elements)
            {
                elements += shiftAmount;
            }
            elementCount -= shiftAmount;
            if (batchSize > 0llu && elementCount > batchSize)
            {
                elementCount = batchSize;
            }
        }
        else
        {
            elements = nullptr;
            elementCount = 0llu;
        }
    }
}
