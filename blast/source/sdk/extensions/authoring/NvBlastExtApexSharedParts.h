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


#ifndef NVBLASTEXTAPEXSHAREDPARTS_H
#define NVBLASTEXTAPEXSHAREDPARTS_H

#include "NvBlast.h"
#include <foundation/PxPlane.h>
namespace physx
{
    class PxVec3;
    class PxTransform;
    class PxBounds3;
}

namespace Nv
{
namespace Blast
{

struct Separation
{
    physx::PxPlane  plane;
    float   min0, max0, min1, max1;

    float getDistance()
    {
        return physx::PxMax(min0 - max1, min1 - max0);
    }
};

/**
    Function to compute midplane between two convex hulls. Is copied from APEX.
*/
bool importerHullsInProximityApexFree(  uint32_t hull0Count, const physx::PxVec3* hull0, physx::PxBounds3& hull0Bounds, const physx::PxTransform& localToWorldRT0In, const physx::PxVec3& scale0In,
                                        uint32_t hull1Count, const physx::PxVec3* hull1, physx::PxBounds3& hull1Bounds, const physx::PxTransform& localToWorldRT1In, const physx::PxVec3& scale1In,
                                        physx::PxF32 maxDistance, Separation* separation);

} // namespace Blast
} // namespace Nv


#endif // NVBLASTEXTAPEXSHAREDPARTS_H
