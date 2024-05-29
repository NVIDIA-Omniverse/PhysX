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


#ifndef NVBLASTNVSHAREDSHELPERS_H
#define NVBLASTNVSHAREDSHELPERS_H

#include "NvCTypes.h"

#include "NvVec2.h"
#include "NvVec3.h"
#include "NvVec4.h"
#include "NvTransform.h"
#include "NvPlane.h"
#include "NvMat33.h"
#include "NvMat44.h"
#include "NvBounds3.h"

using namespace nvidia;

#define WCast(type, name) reinterpret_cast<type>(name)
#define RCast(type, name) reinterpret_cast<const type>(name)

#define CONVERT(BlastType, NvSharedType)                                                                                  \
    static inline NvSharedType& toNvShared(BlastType& v)                                                                     \
    {                                                                                                                  \
        return WCast(NvSharedType&, v);                                                                                   \
    }                                                                                                                  \
    static inline const NvSharedType& toNvShared(const BlastType& v)                                                         \
    {                                                                                                                  \
        return RCast(NvSharedType&, v);                                                                                   \
    }                                                                                                                  \
    static inline const BlastType& fromNvShared(const NvSharedType& v)                                                       \
    {                                                                                                                  \
        return RCast(BlastType&, v);                                                                                   \
    }                                                                                                                  \
    static inline BlastType& fromNvShared(NvSharedType& v)                                                                   \
    {                                                                                                                  \
        return WCast(BlastType&, v);                                                                                   \
    }                                                                                                                  \
    static inline NvSharedType* toNvShared(BlastType* v)                                                                     \
    {                                                                                                                  \
        return WCast(NvSharedType*, v);                                                                                   \
    }                                                                                                                  \
    static inline const NvSharedType* toNvShared(const BlastType* v)                                                         \
    {                                                                                                                  \
        return RCast(NvSharedType*, v);                                                                                   \
    }                                                                                                                  \
    static inline const BlastType* fromNvShared(const NvSharedType* v)                                                       \
    {                                                                                                                  \
        return RCast(BlastType*, v);                                                                                   \
    }                                                                                                                  \
    static inline BlastType* fromNvShared(NvSharedType* v)                                                                   \
    {                                                                                                                  \
        return WCast(BlastType*, v);                                                                                   \
    }


CONVERT(NvcVec2, nvidia::NvVec2)
CONVERT(NvcVec3, nvidia::NvVec3)
CONVERT(NvcVec4, nvidia::NvVec4)
CONVERT(NvcQuat, nvidia::NvQuat)
CONVERT(NvcTransform, nvidia::NvTransform)
CONVERT(NvcPlane, nvidia::NvPlane)
CONVERT(NvcMat33, nvidia::NvMat33)
CONVERT(NvcMat44, nvidia::NvMat44)
CONVERT(NvcBounds3, nvidia::NvBounds3)

NV_COMPILE_TIME_ASSERT(sizeof(NvcVec2) == sizeof(nvidia::NvVec2));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcVec2, x) == NV_OFFSET_OF(nvidia::NvVec2, x));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcVec2, y) == NV_OFFSET_OF(nvidia::NvVec2, y));

NV_COMPILE_TIME_ASSERT(sizeof(NvcVec3) == sizeof(nvidia::NvVec3));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcVec3, x) == NV_OFFSET_OF(nvidia::NvVec3, x));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcVec3, y) == NV_OFFSET_OF(nvidia::NvVec3, y));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcVec3, z) == NV_OFFSET_OF(nvidia::NvVec3, z));

NV_COMPILE_TIME_ASSERT(sizeof(NvcVec4) == sizeof(nvidia::NvVec4));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcVec4, x) == NV_OFFSET_OF(nvidia::NvVec4, x));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcVec4, y) == NV_OFFSET_OF(nvidia::NvVec4, y));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcVec4, z) == NV_OFFSET_OF(nvidia::NvVec4, z));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcVec4, w) == NV_OFFSET_OF(nvidia::NvVec4, w));

NV_COMPILE_TIME_ASSERT(sizeof(NvcQuat) == sizeof(nvidia::NvQuat));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcQuat, x) == NV_OFFSET_OF(nvidia::NvQuat, x));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcQuat, y) == NV_OFFSET_OF(nvidia::NvQuat, y));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcQuat, z) == NV_OFFSET_OF(nvidia::NvQuat, z));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcQuat, w) == NV_OFFSET_OF(nvidia::NvQuat, w));

NV_COMPILE_TIME_ASSERT(sizeof(NvcTransform) == sizeof(nvidia::NvTransform));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcTransform, p) == NV_OFFSET_OF(nvidia::NvTransform, p));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcTransform, q) == NV_OFFSET_OF(nvidia::NvTransform, q));

NV_COMPILE_TIME_ASSERT(sizeof(NvcPlane) == sizeof(nvidia::NvPlane));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcPlane, n) == NV_OFFSET_OF(nvidia::NvPlane, n));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcPlane, d) == NV_OFFSET_OF(nvidia::NvPlane, d));

NV_COMPILE_TIME_ASSERT(sizeof(NvcMat33) == sizeof(nvidia::NvMat33));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcMat33, column0) == NV_OFFSET_OF(nvidia::NvMat33, column0));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcMat33, column1) == NV_OFFSET_OF(nvidia::NvMat33, column1));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcMat33, column2) == NV_OFFSET_OF(nvidia::NvMat33, column2));

NV_COMPILE_TIME_ASSERT(sizeof(NvcBounds3) == sizeof(nvidia::NvBounds3));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcBounds3, minimum) == NV_OFFSET_OF(nvidia::NvBounds3, minimum));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcBounds3, maximum) == NV_OFFSET_OF(nvidia::NvBounds3, maximum));

#endif  // #ifndef NVBLASTNVSHAREDSHELPERS_H
