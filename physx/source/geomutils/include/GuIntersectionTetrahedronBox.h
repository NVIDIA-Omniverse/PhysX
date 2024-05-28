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

#ifndef GU_INTERSECTION_TETRAHEDRON_BOX_H
#define GU_INTERSECTION_TETRAHEDRON_BOX_H

#include "foundation/PxVec3.h"
#include "foundation/PxBounds3.h"
#include "common/PxPhysXCommonConfig.h"

namespace physx
{
namespace Gu
{
	class Box;
	class BoxPadded;

	/**
	Tests if a tetrahedron overlaps a box (AABB).

	\param a	[in] tetrahedron's first point
	\param b	[in] tetrahedron's second point
	\param c	[in] tetrahedron's third point
	\param d	[in] tetrahedron's fourth point
	\param box	[in] The axis aligned box box to check for overlap
	\return	true if tetrahedron overlaps box
	*/
	PX_PHYSX_COMMON_API bool intersectTetrahedronBox(const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d, const PxBounds3& box);


} // namespace Gu
}

#endif
