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

#ifndef PX_LINE_STRIP_SKINNING_H
#define PX_LINE_STRIP_SKINNING_H

#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"
#include "foundation/PxMat44.h"


#if !PX_DOXYGEN
namespace physx
{
#endif

#if PX_SUPPORT_GPU_PHYSX

	class PxHairSystemDesc;

	/**
	\brief Specifies the position and binding of a point relative to a line strip such that the point moves with the line during simulation.
	*/
	struct PxLineStripSkinnedVertex
	{
		PxVec3 mPosition; //!< The position of the vertex that should move with a line strip. Expressed in the same coordinate system as the line strip points when computing the skinning.
		PxU32 mSegmentId; //!< The id of the vertex at the beginning of the line segment to which this point is connected
		PxReal mSegmentLocation; //!< Parameter in the range 0...1 that specifies the location on the segment where the base coordinate system gets evaluated at runtime to update the skinned position.

		PxLineStripSkinnedVertex() : mPosition(0.0f), mSegmentId(0), mSegmentLocation(0.0f)
		{
		}

		PxLineStripSkinnedVertex(const PxVec3& position, PxU32 segmentId, PxReal segmentLocation) : mPosition(position), mSegmentId(segmentId), mSegmentLocation(segmentLocation)
		{
		}
	};

	/**
	\brief Utility class to embed high resolution line strips into low resolution line strips
	*/
	class PxLineStripSkinning
	{
	public:
		/**
		\brief Computes the skinning information used to update the skinned points during simulation

		\param[in] simVertices The simulated vertices in the state in which the skinning information shall be computed. These vertices will eventually drive the skinned vertices
		\param[in] simStrandPastEndIndices The index after the last strand vertex for the simulation strands (strand = line strip)
		\param[in] nbLineStrips The number of line strips
		\param[in] skinnedVertices The positions and segment locations of the skinned vertices in the initial frame. They will keep their position relative to the line strip segment during simulation.
		\param[in] nbSkinnedVertices The total number of skinned vertices
		\param[out] skinnedVertexInterpolationData Must provide space for one entry per skinned vertex. Contains the location of the skinned vertex relative to the interpolated base frame. The w component encodes the location of the base frame along the line segment.
		\param[out] skinningInfoRootStrandDirections Must provide space for one entry per line strip. Contains the direction of the first line segment per line strip during the calculation of the skinning information.
		\param[out] skinningInfoStrandStartIndices Must provide space for one entry per line strip. Contains the index of the first skinning vertex in the buffer for every line strip. The skinned vertices must be sorted such that vertices attached to the same line strip are adjacent to each other in the buffer.
		\param[in] transform Optional transform that gets applied to the simVertices before computing the skinning information
		\param[in] catmullRomAlpha Optional parameter in the range 0...1 that allows to control the curve interpolation.
		*/
		virtual void initializeInterpolatedVertices(const PxVec4* simVertices, const PxU32* simStrandPastEndIndices, PxU32 nbLineStrips, const PxLineStripSkinnedVertex* skinnedVertices, PxU32 nbSkinnedVertices,
			PxVec4* skinnedVertexInterpolationData, PxVec3* skinningInfoRootStrandDirections, PxU32* skinningInfoStrandStartIndices, const PxMat44& transform = PxMat44(PxIdentity), PxReal catmullRomAlpha = 0.5f) = 0;
		
		/**
		\brief Evaluates and updates the skinned positions based on the interpolation data on the GPU. All input arrays are GPU arrays.
		\param[in] simVertices The simulation vertices (device pointer) according to which the skinned positions shall be computed.
		\param[in] simStrandPastEndIndices  Device pointer containing the index after the last strand vertex for the simulation strands (strand = line strip)
		\param[in] nbSimStrands The number of line strips
		\param[in] skinnedVertexInterpolationData Device pointer containing the location of the skinned vertex relative to the interpolated base frame. The w component encodes the location of the base frame along the line segment.
		\param[in] skinningInfoStrandStartIndices Device pointer containing the index of the first skinning vertex in the buffer for every line strip. The skinned vertices must be sorted such that vertices attached to the same line strip are adjacent to each other in the buffer.
		\param[in] skinningInfoRootStrandDirections Device pointer containing the direction of the first line segment per line strip during the calculation of the skinning information.
		\param[in] nbSkinnedVertices The total number of skinned vertices	
		\param[out] result The skinned vertex positions where the updated positions will be written
		\param[in] stream The cuda stream on which ther kernel call gets scheduled
		\param[in] transform Optional device array holding a transform that gets applied to the simVertices before computing the skinning information
		\param[in] catmullRomAlpha Optional parameter in the range 0...1 that allows to control the curve interpolation.
		*/
		virtual void evaluateInterpolatedVertices(const PxVec4* simVertices, const PxU32* simStrandPastEndIndices, PxU32 nbSimStrands, const PxVec4* skinnedVertexInterpolationData, 
			const PxU32* skinningInfoStrandStartIndices, const PxVec3* skinningInfoRootStrandDirections, PxU32 nbSkinnedVertices, PxVec3* result, CUstream stream, const PxReal* transform = NULL, PxReal catmullRomAlpha = 0.5f) = 0;

		/**
		\brief Destructor
		*/
		virtual ~PxLineStripSkinning() {}
	};

#endif	

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
