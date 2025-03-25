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


void stat_add(inout float4 stat, inout uint count, float value)
{
    if (count == 0u)
    {
        stat.xy = value.xx;
    }
    stat.x = min(stat.x, value);
    stat.y = max(stat.y, value);
    count = count + 1u;
    float delta = value - stat.z;
    stat.z = stat.z + delta / float(count);
    stat.w = stat.w + delta * (value - stat.z);
}

void stat_addN(inout float4 stat, inout uint count, float value, uint n)
{
    if (count == 0u)
    {
        stat.xy = value.xx;
    }
    stat.x = min(stat.x, value);
    stat.y = max(stat.y, value);
    float denom = 1.f / float(count + n);
    float delta = value - stat.z;
    stat.z = stat.z + denom * delta * float(n);
    stat.w = stat.w + denom * delta * delta * float(count) * float(n);
    count = count + n;
}

void stat_merge(inout float4 stat, inout uint count, float4 other, uint otherCount)
{
    if (otherCount > 0u)
    {
        stat.x = min(stat.x, other.x);
        stat.y = max(stat.y, other.y);
        float denom = 1.f / float(count + otherCount);
        float delta = other.z - stat.z;
        stat.z = stat.z + denom * delta * float(otherCount);
        stat.w = stat.w + other.w + denom * delta * delta * float(count) * float(otherCount);
        count = count + otherCount;
    }
}

float stat_stddev(float4 stat, uint count)
{
    float var = 0.f;
    if (count > 1)
    {
        var = stat.w / float(count);
    }
    return sqrt(var);
}
