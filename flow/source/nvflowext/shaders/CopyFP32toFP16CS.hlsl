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

#include "NvFlowShader.hlsli"

#include "CopyParams.h"

ConstantBuffer<CopyFP32toFP16CS_Params> paramsIn;

StructuredBuffer<float> srcIn;

RWStructuredBuffer<uint> dstOut;

[numthreads(128, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID)
{
    uint tidx = dispatchThreadID.x + (paramsIn.blockIdxOffset << 7u);
    if (tidx < paramsIn.threadCount)
    {
        uint baseDstIdx = paramsIn.dstOffset >> 1u;

        uint idx0 = 2u * (tidx + baseDstIdx);
        uint idx1 = 2u * (tidx + baseDstIdx) + 1;

        bool inBounds0 = idx0 >= paramsIn.dstOffset && idx0 < paramsIn.dstOffset + paramsIn.elementCount;
        bool inBounds1 = idx1 >= paramsIn.dstOffset && idx1 < paramsIn.dstOffset + paramsIn.elementCount;

        uint val = 0u;
        if (!inBounds0 || !inBounds1)
        {
            val = dstOut[tidx + baseDstIdx];
        }
        if (inBounds0)
        {
            val = (f32tof16(srcIn[paramsIn.srcOffset + idx0 - paramsIn.dstOffset]) & 0x0000FFFF) | (val & 0xFFFF0000);
        }
        if (inBounds1)
        {
            val = (f32tof16(srcIn[paramsIn.srcOffset + idx1 - paramsIn.dstOffset]) << 16u) | (val & 0x0000FFFF);
        }
        dstOut[tidx + baseDstIdx] = val;
    }
}
