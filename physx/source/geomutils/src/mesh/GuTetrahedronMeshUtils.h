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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.

#ifndef GU_TETRAHEDRONMESHUTILS_H
#define GU_TETRAHEDRONMESHUTILS_H

#include <GuTetrahedronMesh.h>

namespace physx
{
namespace Gu
{

PX_PHYSX_COMMON_API
void convertSoftbodyCollisionToSimMeshTets(const PxTetrahedronMesh& simMesh, const SoftBodyAuxData& simState,
                                           const BVTetrahedronMesh& collisionMesh, PxU32 inTetId,
                                           const PxVec4& inTetBarycentric, PxU32& outTetId,
                                           PxVec4& outTetBarycentric);

PX_PHYSX_COMMON_API
PxVec4 addAxisToSimMeshBarycentric(const PxTetrahedronMesh& simMesh, const PxU32 simTetId,
                                   const PxVec4& simBarycentric, const PxVec3& axis);

}
}

#endif
