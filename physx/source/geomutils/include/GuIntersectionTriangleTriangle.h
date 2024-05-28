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

#ifndef GU_INTERSECTION_TRIANGLE_TRIANGLE_H
#define GU_INTERSECTION_TRIANGLE_TRIANGLE_H

#include "GuSegment.h"
#include "common/PxPhysXCommonConfig.h"

namespace physx
{
namespace Gu
{
	/**
	Tests if a two triangles intersect

	\param a1				[in] Fist point of the first triangle
	\param b1				[in] Second point of the first triangle
	\param c1				[in] Third point of the first triangle
	\param a2				[in] Fist point of the second triangle
	\param b2				[in] Second point of the second triangle
	\param c2				[in] Third point of the second triangle
	\param ignoreCoplanar	[in] True to filter out coplanar triangles
	\return	true if triangles intersect
	*/
	PX_PHYSX_COMMON_API bool intersectTriangleTriangle(	const PxVec3& a1, const PxVec3& b1, const PxVec3& c1,
														const PxVec3& a2, const PxVec3& b2, const PxVec3& c2,
														bool ignoreCoplanar = false);
} // namespace Gu
}

#endif
