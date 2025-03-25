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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef PXG_SDF_BUILDER_H
#define PXG_SDF_BUILDER_H

#include "PxSDFBuilder.h" 
#include "foundation/PxSimpleTypes.h"
#include "PxgKernelLauncher.h"

#include "PxgBVH.h"
#include "GuSDF.h"
#include "foundation/PxVec4.h"
#include "PxgAlgorithms.h"


#if !PX_DOXYGEN
namespace physx
{
#endif

	class PxgKernelLauncher;	

	// Create a linear BVH as described in Fast and Simple Agglomerative LBVH construction
	// this is a bottom-up clustering method that outputs one node per-leaf 
	// Taken from Flex
	class PxgLinearBVHBuilderGPU
	{
	public:
		PxgLinearBVHBuilderGPU(PxgKernelLauncher& kernelLauncher);		

		// takes a bvh (host ref), and pointers to the GPU lower and upper bounds for each triangle
		// the priorities array allows specifying a 5-bit [0-31] value priority such that lower priority
		// leaves will always be returned first
		void buildFromLeaveBounds(PxgBVH& bvh, const PxVec4* lowers, const PxVec4* uppers, const PxI32* priorities, PxI32 n, PxBounds3* totalBounds,
			CUstream stream, bool skipAllocate = false);

		void buildFromTriangles(PxgBVH& bvh, const PxVec3* vertices, const PxU32* triangleIndices, const PxI32* priorities,
			PxI32 n, PxBounds3* totalBounds, CUstream stream, PxReal boxMargin = 1e-5f);
		
		void buildTreeAndWindingClustersFromTriangles(PxgBVH& bvh, PxgWindingClusterApproximation* windingNumberClustersD, const PxVec3* vertices, const PxU32* triangleIndices, const PxI32* priorities,
			PxI32 n, PxBounds3* totalBounds, CUstream stream, PxReal boxMargin = 1e-5f, bool skipAllocate = false);


		void resizeBVH(PxgBVH& bvh, PxU32 numNodes);
		void releaseBVH(PxgBVH& bvh);

		//Allocates or resizes the linear bvh builder including the bvh itself
		void allocateOrResize(PxgBVH& bvh, PxU32 numItems);
		void release();

		PxI32* mMaxTreeDepth;
	private:

		void prepareHierarchConstruction(PxgBVH& bvh, const PxVec4* lowers, const PxVec4* uppers, const PxI32* priorities, PxI32 n, PxBounds3* totalBounds, CUstream stream);
		
		PxgKernelLauncher mKernelLauncher;
		PxGpuRadixSort<PxU32> mSort;

		// temporary data used during building
		PxU32* mIndices;
		PxI32* mKeys;
		PxReal* mDeltas;
		PxI32* mRangeLefts;
		PxI32* mRangeRights;
		PxI32* mNumChildren;		

		// bounds data when total item bounds built on GPU
		PxVec3* mTotalLower;
		PxVec3* mTotalUpper;
		PxVec3* mTotalInvEdges;

		PxU32 mMaxItems;
	};

	class PxgSDFBuilder : public PxSDFBuilder, public PxUserAllocated
	{
	private:
		PxgKernelLauncher mKernelLauncher;

		void computeDenseSDF(const PxgBvhTriangleMesh& mesh, const PxgWindingClusterApproximation* windingNumberClustersD,
			const Gu::GridQueryPointSampler& sampler, PxU32 sizeX, PxU32 sizeY, PxU32 sizeZ, PxReal* sdfDataD, CUstream stream, PxReal* windingNumbersD = NULL);

		// returns NULL if GPU errors occurred.
		PxReal* buildDenseSDF(const PxVec3* vertices, PxU32 numVertices, const PxU32* indicesOrig, PxU32 numTriangleIndices, PxU32 width, PxU32 height, PxU32 depth,
			const PxVec3& minExtents, const PxVec3& maxExtents, bool cellCenteredSamples, CUstream stream);

		void compressSDF(PxReal* denseSdfD, PxU32 width, PxU32 height, PxU32 depth,
			PxU32 subgridSize, PxReal narrowBandThickness, PxU32 bytesPerSubgridPixel, PxReal errorThreshold,
			PxReal& subgridGlobalMinValue, PxReal& subgridGlobalMaxValue, PxArray<PxReal>& sdfCoarse, PxArray<PxU32>& sdfSubgridsStartSlots, PxArray<PxU8>& sdfDataSubgrids,
			PxU32& sdfSubgrids3DTexBlockDimX, PxU32& sdfSubgrids3DTexBlockDimY, PxU32& sdfSubgrids3DTexBlockDimZ, CUstream stream);
		
		void fixHoles(PxU32 width, PxU32 height, PxU32 depth, PxReal* sdfDataD, const PxVec3& cellSize, const PxVec3& minExtents, const PxVec3& maxExtents,
			Gu::GridQueryPointSampler& sampler, CUstream stream);

		bool allocateBuffersForCompression(PxReal*& backgroundSdfD, PxU32 numBackgroundSdfSamples, PxU32*& subgridAddressesD, PxU8*& subgridActiveD, PxU32 numAddressEntries,
			PxReal*& subgridGlobalMinValueD, PxReal*& subgridGlobalMaxValueD, PxGpuScan& scan);

		void releaseBuffersForCompression(PxReal*& backgroundSdfD, PxU32*& subgridAddressesD, PxU8*& subgridActiveD, PxReal*& subgridGlobalMinValueD,
			PxReal*& subgridGlobalMaxValueD, PxGpuScan& scan);

	public:
		PxgSDFBuilder(PxgKernelLauncher& kernelLauncher);

		virtual bool buildSDF(const PxVec3* vertices, PxU32 numVertices, const PxU32* indicesOrig, PxU32 numTriangleIndices, PxU32 width, PxU32 height, PxU32 depth,
			const PxVec3& minExtents, const PxVec3& maxExtents, bool cellCenteredSamples, PxReal* sdf, CUstream stream) PX_OVERRIDE;
	
		virtual bool buildSparseSDF(const PxVec3* vertices, PxU32 numVertices, const PxU32* indicesOrig, PxU32 numTriangleIndices, PxU32 width, PxU32 height, PxU32 depth,
			const PxVec3& minExtents, const PxVec3& maxExtents, PxReal narrowBandThickness, PxU32 subgridSize, PxSdfBitsPerSubgridPixel::Enum bytesPerSubgridPixel,
			PxArray<PxReal>& sdfCoarse, PxArray<PxU32>& sdfSubgridsStartSlots, PxArray<PxU8>& sdfDataSubgrids,
			PxReal& subgridsMinSdfValue, PxReal& subgridsMaxSdfValue, 
			PxU32& sdfSubgrids3DTexBlockDimX, PxU32& sdfSubgrids3DTexBlockDimY, PxU32& sdfSubgrids3DTexBlockDimZ, CUstream stream) PX_OVERRIDE;
	
		void release();
	};

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
