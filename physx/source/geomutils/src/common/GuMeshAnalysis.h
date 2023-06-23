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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef GU_MESH_ANALYSIS_H
#define GU_MESH_ANALYSIS_H

#include "foundation/Px.h"
#include "common/PxPhysXCommonConfig.h"
#include "GuTriangle.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxSort.h"

namespace physx
{
namespace Gu
{
	using Triangle = Gu::IndexedTriangleT<PxI32>;

	
	class MeshAnalyzer
	{
		struct Range
		{
			PxI32 start;
			PxI32 end; //Exclusive     

			Range(PxI32 start_, PxI32 end_)
			{
				start = start_;
				end = end_;
			}

			PxI32 Length() const { return end - start; }
		};

		template<typename T, typename S>
		static void splitRanges(PxArray<Range>& mergeRanges, const PxArray<PxI32>& indexer, const T* points, PxI32 dimIndex, S tol)
		{
			PxArray<Range> newMergeRanges;

			for (PxU32 i = 0; i < mergeRanges.size(); ++i)
			{
				const Range& r = mergeRanges[i];
				PxI32 start = r.start;
				for (PxI32 j = r.start + 1; j < r.end; ++j)
				{
					//PxF64 delta = PxAbs(points[start][dimIndex] - points[j - 1][dimIndex]);
					S delta = PxAbs(points[indexer[j]][dimIndex] - points[indexer[j - 1]][dimIndex]);
					if (delta > tol)
					{
						if (j - start > 1)
							newMergeRanges.pushBack(Range(start, j));
						start = j;
					}
				}
				if (r.end - start > 1)
					newMergeRanges.pushBack(Range(start, r.end));
			}

			mergeRanges.clear();
			for (PxU32 i = 0; i < newMergeRanges.size(); ++i)
				mergeRanges.pushBack(newMergeRanges[i]);
		}

		template<typename T>
		struct Comparer
		{
			const T* points;
			PxU32 dimension;

			Comparer(const T* points_, const PxU32 dimension_) : points(points_), dimension(dimension_) {}

			bool operator()(const PxI32& a, const PxI32& b) const
			{
				return points[a][dimension] > points[b][dimension];
			}

		private:
			PX_NOCOPY(Comparer)
		};

	public:
		template<typename T, typename S>
		static void mapDuplicatePoints(const T* points, const PxU32 nbPoints, PxArray<PxI32>& result, S duplicateDistanceManhattanMetric = static_cast<S>(1e-6))
		{
			result.reserve(nbPoints);
			result.forceSize_Unsafe(nbPoints);

			PxArray<PxI32> indexer;
			indexer.reserve(nbPoints);
			indexer.forceSize_Unsafe(nbPoints);
			for (PxU32 i = 0; i < nbPoints; ++i)
			{
				indexer[i] = i;
				result[i] = i;
			}

			Comparer<T> comparer(points, 0);
			PxSort(indexer.begin(), indexer.size(), comparer);

			PxArray<Range> mergeRanges;
			mergeRanges.pushBack(Range(0, nbPoints));
			splitRanges<T>(mergeRanges, indexer, points, 0, duplicateDistanceManhattanMetric);

			comparer.dimension = 1;
			for (PxU32 i = 0; i < mergeRanges.size(); ++i)
			{
				const Range& r = mergeRanges[i];
				PxSort(indexer.begin() + r.start, r.Length(), comparer);
			}
			splitRanges<T>(mergeRanges, indexer, points, 1, duplicateDistanceManhattanMetric);

			comparer.dimension = 2;
			for (PxU32 i = 0; i < mergeRanges.size(); ++i)
			{
				const Range& r = mergeRanges[i];
				PxSort(indexer.begin() + r.start, r.Length(), comparer);
			}
			splitRanges<T>(mergeRanges, indexer, points, 2, duplicateDistanceManhattanMetric);

			//Merge the ranges
			for (PxU32 i = 0; i < mergeRanges.size(); ++i)
			{
				const Range& r = mergeRanges[i];
				PxSort(indexer.begin() + r.start, r.Length());
				for (PxI32 j = r.start + 1; j < r.end; ++j)
					result[indexer[j]] = result[indexer[r.start]];
			}
		}

		PX_PHYSX_COMMON_API static bool buildTriangleAdjacency(const Triangle* tris, PxU32 numTriangles, PxArray<PxI32>& result, PxHashMap<PxU64, PxI32>& edges);
		PX_PHYSX_COMMON_API static bool checkConsistentTriangleOrientation(const Triangle* tris, PxU32 numTriangles);
		PX_PHYSX_COMMON_API static bool buildConsistentTriangleOrientationMap(const Triangle* tris, PxU32 numTriangles, PxArray<bool>& flipMap, 
			PxHashMap<PxU64, PxI32>& edges, PxArray<PxArray<PxU32>>& connectedTriangleGroups);
		PX_PHYSX_COMMON_API static bool makeTriOrientationConsistent(Triangle* tris, PxU32 numTriangles, bool invertOrientation = false);
		PX_PHYSX_COMMON_API static bool checkMeshWatertightness(const Triangle* tris, PxU32 numTriangles, bool treatInconsistentWindingAsNonWatertight = true);
	};
}
}

#endif

