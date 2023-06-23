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

#ifndef PX_ARRAY_CONVERTER_H
#define PX_ARRAY_CONVERTER_H

#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"


#if !PX_DOXYGEN
namespace physx
{
#endif

#if PX_SUPPORT_GPU_PHYSX

	/**
	\brief Utility class to convert gpu arrays to a different memory layout
	*/
	class PxArrayConverter
	{
	public:
		/**
		\brief Helper function to merge two separate PxVec4 arrays into one interleaved PxVec3 array
		\param[in] verticesD The vertices device memory buffer
		\param[in] normalsD The normals device memory buffer
		\param[in] length The number of vertices and normals
		\param[out] interleavedResultBufferD The resulting interleaved buffer containing 2*length elements with the format vertex0, normal0, vertex1, normal1...
		\param[in] stream The cuda stream on which the conversion is processed
		*/		
		virtual void interleaveGpuBuffers(const PxVec4* verticesD, const PxVec4* normalsD, PxU32 length, PxVec3* interleavedResultBufferD, CUstream stream) = 0;
	
		/**
		\brief Helper function to convert the hair system's strand representation to a line list. The conversion is done on the GPU.
		\param[in] verticesD The strand vertices device memory buffer
		\param[in] numVertices The total number of vertices
		\param[in] strandPastEndIndicesD One index per strand (device memory array) to find out where the next strand starts
		\param[in] numStrands the number of strands
		\param[out] resultD A device memory buffer with 2*numVertices capacity describing line segment where line i extends from result[2*i] to result[2*i+1]
		\param[in] stream The cuda stream on which the conversion is processed
		*/
		virtual void extractLinesFromStrands(const PxVec4* verticesD, PxU32 numVertices, const PxU32* strandPastEndIndicesD,
			PxU32 numStrands, PxVec4* resultD, CUstream stream) = 0;

		/**
		\brief Helper function to convert the hair system's strand representation to a line list. The conversion is done on the GPU.
		\param[in] verticesD The strand vertices device memory buffer
		\param[in] numVertices The total number of vertices
		\param[in] strandPastEndIndicesD One index per strand (device memory array) to find out where the next strand starts
		\param[in] numStrands the number of strands
		\param[out] resultD A device memory buffer with 2*numVertices capacity describing line segment where line i extends from result[2*i] to result[2*i+1]
		\param[in] stream The cuda stream on which the conversion is processed
		*/
		virtual void extractLinesFromStrands(const PxVec3* verticesD, PxU32 numVertices, const PxU32* strandPastEndIndicesD,
			PxU32 numStrands, PxVec3* resultD, CUstream stream) = 0;

		/**
		\brief Destructor
		*/
		virtual ~PxArrayConverter() {}
	};

#endif	

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
