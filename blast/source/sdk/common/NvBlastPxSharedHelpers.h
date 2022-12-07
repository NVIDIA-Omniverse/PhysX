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
// Copyright (c) 2016-2022 NVIDIA Corporation. All rights reserved.


#ifndef NVBLASTPXSHAREDTYPESHELPERS_H
#define NVBLASTPXSHAREDTYPESHELPERS_H

#include "NvCTypes.h"
#include <foundation/PxVec2.h>
#include <foundation/PxVec3.h>
#include <foundation/PxVec4.h>
#include <foundation/PxTransform.h>
#include <foundation/PxPlane.h>
#include <foundation/PxMat33.h>
#include <foundation/PxMat44.h>
#include <foundation/PxBounds3.h>

#define WCast(type, name) reinterpret_cast<type>(name)
#define RCast(type, name) reinterpret_cast<const type>(name)

#define CONVERT(BlastType, PxSharedType)                                                                                  \
    static inline PxSharedType& toPxShared(BlastType& v)                                                                     \
    {                                                                                                                  \
        return WCast(PxSharedType&, v);                                                                                   \
    }                                                                                                                  \
    static inline const PxSharedType& toPxShared(const BlastType& v)                                                         \
    {                                                                                                                  \
        return RCast(PxSharedType&, v);                                                                                   \
    }                                                                                                                  \
    static inline const BlastType& fromPxShared(const PxSharedType& v)                                                       \
    {                                                                                                                  \
        return RCast(BlastType&, v);                                                                                   \
    }                                                                                                                  \
    static inline BlastType& fromPxShared(PxSharedType& v)                                                                   \
    {                                                                                                                  \
        return WCast(BlastType&, v);                                                                                   \
    }                                                                                                                  \
    static inline PxSharedType* toPxShared(BlastType* v)                                                                     \
    {                                                                                                                  \
        return WCast(PxSharedType*, v);                                                                                   \
    }                                                                                                                  \
    static inline const PxSharedType* toPxShared(const BlastType* v)                                                         \
    {                                                                                                                  \
        return RCast(PxSharedType*, v);                                                                                   \
    }                                                                                                                  \
    static inline const BlastType* fromPxShared(const PxSharedType* v)                                                       \
    {                                                                                                                  \
        return RCast(BlastType*, v);                                                                                   \
    }                                                                                                                  \
    static inline BlastType* fromPxShared(PxSharedType* v)                                                                   \
    {                                                                                                                  \
        return WCast(BlastType*, v);                                                                                   \
    }


CONVERT(NvcVec2, physx::PxVec2)
CONVERT(NvcVec3, physx::PxVec3)
CONVERT(NvcVec4, physx::PxVec4)
CONVERT(NvcQuat, physx::PxQuat)
CONVERT(NvcTransform, physx::PxTransform)
CONVERT(NvcPlane, physx::PxPlane)
CONVERT(NvcMat33, physx::PxMat33)
CONVERT(NvcMat44, physx::PxMat44)
CONVERT(NvcBounds3, physx::PxBounds3)

NV_COMPILE_TIME_ASSERT(sizeof(NvcVec2) == sizeof(physx::PxVec2));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcVec2, x) == NV_OFFSET_OF(physx::PxVec2, x));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcVec2, y) == NV_OFFSET_OF(physx::PxVec2, y));

NV_COMPILE_TIME_ASSERT(sizeof(NvcVec3) == sizeof(physx::PxVec3));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcVec3, x) == NV_OFFSET_OF(physx::PxVec3, x));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcVec3, y) == NV_OFFSET_OF(physx::PxVec3, y));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcVec3, z) == NV_OFFSET_OF(physx::PxVec3, z));

NV_COMPILE_TIME_ASSERT(sizeof(NvcVec4) == sizeof(physx::PxVec4));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcVec4, x) == NV_OFFSET_OF(physx::PxVec4, x));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcVec4, y) == NV_OFFSET_OF(physx::PxVec4, y));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcVec4, z) == NV_OFFSET_OF(physx::PxVec4, z));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcVec4, w) == NV_OFFSET_OF(physx::PxVec4, w));

NV_COMPILE_TIME_ASSERT(sizeof(NvcQuat) == sizeof(physx::PxQuat));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcQuat, x) == NV_OFFSET_OF(physx::PxQuat, x));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcQuat, y) == NV_OFFSET_OF(physx::PxQuat, y));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcQuat, z) == NV_OFFSET_OF(physx::PxQuat, z));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcQuat, w) == NV_OFFSET_OF(physx::PxQuat, w));

NV_COMPILE_TIME_ASSERT(sizeof(NvcTransform) == sizeof(physx::PxTransform));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcTransform, p) == NV_OFFSET_OF(physx::PxTransform, p));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcTransform, q) == NV_OFFSET_OF(physx::PxTransform, q));

NV_COMPILE_TIME_ASSERT(sizeof(NvcPlane) == sizeof(physx::PxPlane));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcPlane, n) == NV_OFFSET_OF(physx::PxPlane, n));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcPlane, d) == NV_OFFSET_OF(physx::PxPlane, d));

NV_COMPILE_TIME_ASSERT(sizeof(NvcMat33) == sizeof(physx::PxMat33));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcMat33, column0) == NV_OFFSET_OF(physx::PxMat33, column0));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcMat33, column1) == NV_OFFSET_OF(physx::PxMat33, column1));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcMat33, column2) == NV_OFFSET_OF(physx::PxMat33, column2));

NV_COMPILE_TIME_ASSERT(sizeof(NvcBounds3) == sizeof(physx::PxBounds3));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcBounds3, minimum) == NV_OFFSET_OF(physx::PxBounds3, minimum));
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvcBounds3, maximum) == NV_OFFSET_OF(physx::PxBounds3, maximum));

#endif  // #ifndef NVBLASTPHYSXTYPESHELPERS_H
