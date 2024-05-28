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

#include "foundation/PxAssert.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxMemory.h"
#include "GuSAH.h"

using namespace physx;
using namespace Gu;

static PX_FORCE_INLINE float getSurfaceArea(const PxBounds3& bounds)
{
	const PxVec3 e = bounds.maximum - bounds.minimum;
	return 2.0f * (e.x * e.y + e.x * e.z + e.y * e.z);
}

SAH_Buffers::SAH_Buffers(PxU32 nb_prims)
{
	mKeys = PX_ALLOCATE(float, nb_prims, "temp");
	mCumulativeLower = PX_ALLOCATE(float, nb_prims, "temp");
	mCumulativeUpper = PX_ALLOCATE(float, nb_prims, "temp");
	mNb = nb_prims;
}

SAH_Buffers::~SAH_Buffers()
{
	PX_FREE(mKeys);
	PX_FREE(mCumulativeLower);
	PX_FREE(mCumulativeUpper);
}

bool SAH_Buffers::split(PxU32& leftCount, PxU32 nb, const PxU32* PX_RESTRICT prims, const PxBounds3* PX_RESTRICT boxes, const PxVec3* PX_RESTRICT centers)
{
    PxU32 bestAxis = 0;
    PxU32 bestIndex = 0;
    float bestCost = PX_MAX_F32;

	PX_ASSERT(nb<=mNb);
	for(PxU32 axis=0;axis<3;axis++)
	{
		const PxU32* sorted;
		{
			float* keys = mKeys;
			for(PxU32 i=0;i<nb;i++)
			{
				const PxU32 index = prims[i];
				const float center = centers[index][axis];
				keys[i] = center;
			}

			sorted = mSorters[axis].Sort(keys, nb).GetRanks();
		}

		float* cumulativeLower = mCumulativeLower;
		float* cumulativeUpper = mCumulativeUpper;

/*		if(0)
		{
			PxBounds3 bbox = PxBounds3::empty();

			for(PxU32 i=0; i<nb; i++)
			{
				bbox.include(bboxes[references[axis][i]]);
				bbox.include(boxes[prims[nb-sortedIndex-1]]);
			}

			
			for (size_t i = end - 1; i > begin; --i) {
			bbox.extend(bboxes[references[axis][i]]);
			costs[axis][i] = bbox.half_area() * (end - i);
			}
			bbox = BoundingBox<Scalar>::empty();
			auto best_split = std::pair<Scalar, size_t>(std::numeric_limits<Scalar>::max(), end);
			for (size_t i = begin; i < end - 1; ++i) {
			bbox.extend(bboxes[references[axis][i]]);
			auto cost = bbox.half_area() * (i + 1 - begin) + costs[axis][i + 1];
			if (cost < best_split.first)
			best_split = std::make_pair(cost, i + 1);
			}
			return best_split;
		}*/

		if(1)
		{
			// two passes over data to calculate upper and lower bounds
			PxBounds3 lower = PxBounds3::empty();
			PxBounds3 upper = PxBounds3::empty();
//				lower.minimum = lower.maximum = PxVec3(0.0f);
//				upper.minimum = upper.maximum = PxVec3(0.0f);

#if PX_ENABLE_ASSERTS
			float prevLowerCenter = -PX_MAX_F32;
			float prevUpperCenter = PX_MAX_F32;
#endif
			for(PxU32 i=0; i<nb; ++i)
			{
				const PxU32 lowSortedIndex = sorted[i];
				const PxU32 highSortedIndex = sorted[nb-i-1];

				//lower.Union(m_faceBounds[faces[i]]);
					PX_ASSERT(centers[prims[lowSortedIndex]][axis]>=prevLowerCenter);
					lower.include(boxes[prims[lowSortedIndex]]);
#if PX_ENABLE_ASSERTS
					prevLowerCenter = centers[prims[lowSortedIndex]][axis];
#endif
				//upper.Union(m_faceBounds[faces[numFaces - i - 1]]);
					PX_ASSERT(centers[prims[highSortedIndex]][axis]<=prevUpperCenter);
					upper.include(boxes[prims[highSortedIndex]]);
#if PX_ENABLE_ASSERTS
					prevUpperCenter = centers[prims[highSortedIndex]][axis];
#endif

				cumulativeLower[i] = getSurfaceArea(lower);
				cumulativeUpper[nb - i - 1] = getSurfaceArea(upper);
			}

//			const float invTotalSA = 1.0f / cumulativeUpper[0];

			// test all split positions
			for (PxU32 i = 0; i < nb - 1; ++i)
			{
				const float pBelow = cumulativeLower[i];// * invTotalSA;
				const float pAbove = cumulativeUpper[i];// * invTotalSA;

//				const float cost = 0.125f + (pBelow * i + pAbove * float(nb - i));
				const float cost = (pBelow * i + pAbove * float(nb - i));
				if(cost <= bestCost)
				{
					bestCost = cost;
					bestIndex = i;
					bestAxis = axis;
				}
			}
		}
	}

	leftCount = bestIndex + 1;

	if(leftCount==1 || leftCount==nb)
	{
		// Invalid split
		return false;
	}

/*
    // re-sort by best axis
    FaceSorter predicate(&m_vertices[0], &m_indices[0], m_numFaces * 3, bestAxis);
    std::sort(faces, faces + numFaces, predicate);

    return bestIndex + 1;
*/

	{
		PxU32* tmp = reinterpret_cast<PxU32*>(mKeys);
		PxMemCopy(tmp, prims, nb*sizeof(PxU32));

		const PxU32* bestOrder = mSorters[bestAxis].GetRanks();

		PxU32* dst = const_cast<PxU32*>(prims);
		for(PxU32 i=0;i<nb;i++)
			dst[i] = tmp[bestOrder[i]];
	}

	return true;
}

