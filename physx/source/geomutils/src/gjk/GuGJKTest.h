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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef GU_GJK_TEST_H
#define GU_GJK_TEST_H

#include "common/PxPhysXCommonConfig.h"
#include "GuGJKUtil.h"

namespace physx
{
namespace Gu
{
	struct GjkConvex;

	PX_PHYSX_COMMON_API GjkStatus testGjk(const GjkConvex& a, const GjkConvex& b, const aos::Vec3VArg initialSearchDir, const aos::FloatVArg contactDist, aos::Vec3V& closestA, aos::Vec3V& closestB,
		aos::Vec3V& normal, aos::FloatV& dist);
	
	PX_PHYSX_COMMON_API	bool testGjkRaycast(const GjkConvex& a, const GjkConvex& b, const aos::Vec3VArg initialSearchDir, const aos::FloatVArg initialLambda, const aos::Vec3VArg s, const aos::Vec3VArg r, 
		aos::FloatV& lambda, aos::Vec3V& normal, aos::Vec3V& closestA, PxReal inflation);

	PX_PHYSX_COMMON_API GjkStatus testGjkPenetration(const GjkConvex& a, const GjkConvex& b, const aos::Vec3VArg initialSearchDir, const aos::FloatVArg contactDist,
		PxU8* aIndices, PxU8* bIndices, PxU8& size, GjkOutput& output);

	PX_PHYSX_COMMON_API GjkStatus testEpaPenetration(const GjkConvex& a, const GjkConvex& b, const PxU8* aIndices, const PxU8* bIndices, PxU8 size, GjkOutput& output);
}
}

#endif
