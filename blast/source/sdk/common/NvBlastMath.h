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
//
// Copyright (c) 2016-2024 NVIDIA Corporation. All rights reserved.


#ifndef NVBLASTMATH_H
#define NVBLASTMATH_H

#include <math.h>

namespace Nv
{
namespace Blast
{

namespace VecMath
{


NV_INLINE void div(float a[3], float divisor)
{
    for (int i = 0; i < 3; i++)
        a[i] /= divisor;
}

NV_INLINE void mul(float a[3], float multiplier)
{
    for (int i = 0; i < 3; i++)
        a[i] *= multiplier;
}

NV_INLINE void add(const float a[3], float b[3])
{
    for (int i = 0; i < 3; i++)
        b[i] = a[i] + b[i];
}
    
NV_INLINE void add(const float a[3], const float b[3], float r[3])
{
    for (int i = 0; i < 3; i++)
        r[i] = a[i] + b[i];
}

NV_INLINE void sub(const float a[3], const float b[3], float r[3])
{
    for (int i = 0; i < 3; i++)
        r[i] = a[i] - b[i];
}

NV_INLINE float dot(const float a[3], const float b[3])
{
    float r = 0;
    for (int i = 0; i < 3; i++)
        r += a[i] * b[i];
    return r;
}

NV_INLINE float length(const float a[3])
{
    return sqrtf(dot(a, a));
}

NV_INLINE float dist(const float a[3], const float b[3])
{
    float v[3];
    sub(a, b, v);
    return length(v);
}

NV_INLINE float normal(const float a[3], float r[3])
{
    float d = length(a);
    for (int i = 0; i < 3; i++)
        r[i] = a[i] / d;

    return d;
}


} // namespace VecMath

} // namespace Blast
} // namespace Nv


#endif // #ifndef NVBLASTMATH_H
