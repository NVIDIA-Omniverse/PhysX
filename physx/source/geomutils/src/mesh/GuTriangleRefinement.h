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

#ifndef GU_TRIANGLE_REFINEMENT_H
#define GU_TRIANGLE_REFINEMENT_H

// Indexing scheme for sub-triangles in recursive triangle subdivision scheme
// Each subdivision step creates 4 triangles out of the previous one:
// - First triangle is the "a corner"
// - Second triangle is the "b corner"
// - Third triangle is the "c corner"
// - Fourth triangle is the center

#include "foundation/PxVec3.h"

namespace physx {
	namespace Gu {
		PX_FORCE_INLINE PX_CUDA_CALLABLE void getSubTriangle4(int id, PxVec3& baryA, PxVec3& baryB, PxVec3& baryC)
		{

			PxVec3 ab = 0.5f * (baryA + baryB);
			PxVec3 bc = 0.5f * (baryB + baryC);
			PxVec3 ca = 0.5f * (baryC + baryA);

			switch (id)
			{
				case 0:
					baryB = ab;
					baryC = ca;
					break;
				case 1:
					baryA = ab;
					baryC = bc;
					break;
				case 2:
					baryA = ca;
					baryB = bc;
					break;
				case 3:
					baryA = ab;
					baryB = bc;
					baryC = ca;
					break;
				default:
					PX_ASSERT(false);
			}
		}

		PX_FORCE_INLINE PX_CUDA_CALLABLE void getSubTriangle(PxVec3& a, PxVec3& b, PxVec3& c, PxU32 id, PxU32 numSubdivisionSteps)
		{
			for (PxU32 i = 0; i < numSubdivisionSteps; ++i)
			{
				PxU32 j = numSubdivisionSteps - i - 1;
				PxU32 local = id >> (2 * j);
				local = local & 3;
				getSubTriangle4(local, a, b, c);
			}
		}

		PX_FORCE_INLINE PX_CUDA_CALLABLE PxU32 encodeSubdivisionId(PxU32 subdivisionLevel, PxU32 subTriangleIndex)
		{
			//4 bits for subdivision level, the rest for the index
			const PxU32 shift = 30 - 4;
			return (subdivisionLevel << shift) | subTriangleIndex;
		}

		PX_FORCE_INLINE PX_CUDA_CALLABLE void decodeSubdivisionId(PxU32 encodedId, PxU32& subdivisionLevel, PxU32& subTriangleIndex)
		{
			const PxU32 shift = 30 - 4;
			subdivisionLevel = encodedId >> shift;
			subTriangleIndex = encodedId & ((1 << shift) - 1);
		}

		//Increases the subdivision level by one and sets the index to the specified sub triangle that got created out of the previous triangle due to one additional level of subdivision
		PX_FORCE_INLINE PX_CUDA_CALLABLE PxU32 elevateSubdivisionId(PxU32 encodedId, PxU32 subTriangle = 0)
		{
			PX_ASSERT(subTriangle >= 0 && subTriangle < 4);

			PxU32 subdivisionLevel;
			PxU32 subTriangleIndex;
			decodeSubdivisionId(encodedId, subdivisionLevel, subTriangleIndex);

			return encodeSubdivisionId(subdivisionLevel + 1, 4 * subTriangleIndex + subTriangle);
		}

		PX_FORCE_INLINE PX_CUDA_CALLABLE void getSubTriangleEncoded(PxVec3& a, PxVec3& b, PxVec3& c, PxU32 encodedSubIndex)
		{
			PxU32 subdivisionLevel;
			PxU32 subTriangleIndex;
			decodeSubdivisionId(encodedSubIndex, subdivisionLevel, subTriangleIndex);
			getSubTriangle(a, b, c, subTriangleIndex, subdivisionLevel);
		}

	} // namespace Gu
} // namespace physx
#endif // GU_TRIANGLE_REFINEMENT_H
