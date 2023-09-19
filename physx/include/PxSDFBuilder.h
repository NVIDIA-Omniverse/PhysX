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

#ifndef PX_SDF_BUILDER_H
#define PX_SDF_BUILDER_H

#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"
#include "foundation/PxArray.h"
#include "cooking/PxSDFDesc.h"
#include "cudamanager/PxCudaTypes.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Utility class to compute an SDF on the GPU
*/
class PxSDFBuilder
{
public:

	/**
	\brief Constructs a dense grid SDF for a triangle mesh using the GPU
	\param[in] vertices The vertices of the triangle mesh
	\param[in] numVertices The number of vertices
	\param[in] indices The triangle indices
	\param[in] numTriangleIndices The number of triangle indices
	\param[in] width The number of samples along the x direction of the resulting SDF volume
	\param[in] height The number of samples along the y direction of the resulting SDF volume
	\param[in] depth The number of samples along the z direction of the resulting SDF volume	
	\param[in] minExtents The minimum corner location of the axis aligned box containing the SDF samples.
	\param[in] maxExtents The maximum corner location of the axis aligned box containing the SDF samples.
	\param[in] cellCenteredSamples Determines if the sample points are located at the center of a SDF cell or at the lower left (=min) corner of a cell.
	\param[out] sdf The distance values. Must provide space for width*height*depth distance samples. Negative distance means the sample point is located inside of the triangle mesh.
	\param[in] stream The cuda stream on which the conversion is processed. If the default stream (0) is used, a temporary stream will be created internally.	
	*/
	virtual void buildSDF(const PxVec3* vertices, PxU32 numVertices, const PxU32* indices, PxU32 numTriangleIndices, PxU32 width, PxU32 height, PxU32 depth,
		const PxVec3& minExtents, const PxVec3& maxExtents, bool cellCenteredSamples, PxReal* sdf, CUstream stream = 0) = 0;

	/**
	\brief Constructs a sparse grid SDF for a triangle mesh using the GPU
	\param[in] vertices The vertices of the triangle mesh
	\param[in] numVertices The number of vertices
	\param[in] indices The triangle indices
	\param[in] numTriangleIndices The number of triangle indices
	\param[in] width The number of samples along the x direction of the resulting SDF volume
	\param[in] height The number of samples along the y direction of the resulting SDF volume
	\param[in] depth The number of samples along the z direction of the resulting SDF volume
	\param[in] minExtents The minimum corner location of the axis aligned box containing the SDF samples.
	\param[in] maxExtents The maximum corner location of the axis aligned box containing the SDF samples.
	\param[in] narrowBandThickness The thickness of the narrow band.
	\param[in] cellsPerSubgrid The number of cells in a sparse subgrid block (full block has mSubgridSize^3 cells and (mSubgridSize+1)^3 samples)
	\param[in] bitsPerSubgridPixel Subgrid pixel compression
	\param[out] subgridsMinSdfValue Used to store the minimum sdf value over all subgrids
	\param[out] subgridsMaxSdfValue Used to store the maximum sdf value over all subgrids
	\param[out] sdfSubgrids3DTexBlockDimX Used to store x dimension of the texture block that stores the subgrids
	\param[out] sdfSubgrids3DTexBlockDimY Used to store y dimension of the texture block that stores the subgrids
	\param[out] sdfSubgrids3DTexBlockDimZ Used to store z dimension of the texture block that stores the subgrids
	\param[in] stream The cuda stream on which the conversion is processed. If the default stream (0) is used, a temporary stream will be created internally.		
	*/
	virtual void buildSparseSDF(const PxVec3* vertices, PxU32 numVertices, const PxU32* indices, PxU32 numTriangleIndices, PxU32 width, PxU32 height, PxU32 depth,
		const PxVec3& minExtents, const PxVec3& maxExtents, PxReal narrowBandThickness, PxU32 cellsPerSubgrid, PxSdfBitsPerSubgridPixel::Enum bitsPerSubgridPixel,
		PxArray<PxReal>& sdfCoarse, PxArray<PxU32>& sdfSubgridsStartSlots, PxArray<PxU8>& sdfDataSubgrids, 
		PxReal& subgridsMinSdfValue, PxReal& subgridsMaxSdfValue, 
		PxU32& sdfSubgrids3DTexBlockDimX, PxU32& sdfSubgrids3DTexBlockDimY, PxU32& sdfSubgrids3DTexBlockDimZ, CUstream stream = 0) = 0;
	
	/**
	\brief Releases the memory including the this pointer
	*/
	virtual void release() = 0;

	/**
	\brief Destructor
	*/
	virtual ~PxSDFBuilder()	{ }
};

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
