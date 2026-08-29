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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXG_ARTICULATION_BUFFER_H
#define PXG_ARTICULATION_BUFFER_H

#include "PxgCudaBuffer.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxArray.h"

namespace physx
{
	struct PxgArticulationLink;
	struct PxgArticulationLinkSleepData;
	struct PxgArticulationLinkProp;
	class PxgArticulationTendon;
	class PxGpuSpatialTendonData;
	class PxGpuFixedTendonData;

	namespace Dy
	{
		struct ArticulationJointCore;
		class ArticulationJointCoreData;
		struct SpatialSubspaceMatrix;
		struct ArticulationMimicJointCore;
	}

	namespace Cm
	{
		struct UnAlignedSpatialVector;
	}

	class PxgArticulationBuffer : public PxUserAllocated
	{
	public:

		PxgArticulationBuffer(PxgHeapMemoryAllocator& deviceAlloc);

		~PxgArticulationBuffer();

		PxgTypedCudaBuffer<PxgArticulationLink>					links;
		PxgTypedCudaBuffer<PxReal>								linkWakeCounters;       //original set to the same as articulation wakeCounter
		PxgTypedCudaBuffer<PxgArticulationLinkSleepData>		linkSleepData;
		PxgTypedCudaBuffer<PxgArticulationLinkProp>				linkProps;
		PxgTypedCudaBuffer<Dy::ArticulationJointCore>			joints;
		PxgTypedCudaBuffer<Dy::ArticulationJointCoreData>		jointData;
		PxgTypedCudaBuffer<Cm::UnAlignedSpatialVector>			coriolisVectors;        //link coriolis vector
		PxgTypedCudaBuffer<Cm::UnAlignedSpatialVector>			zAForces;               //link spatial zero acceleration force/ spatical articulate 
		PxgTypedCudaBuffer<PxU32>								pathToRoots;            //global array store path to root for each link in continuous. Each link should have a start index and numberOfElems

		PxgTypedCudaBuffer<PxGpuSpatialTendonData>				spatialTendonParams;
		PxgTypedCudaBuffer<PxgArticulationTendon>				spatialTendons;
		PxArray<PxgCudaBuffer*>									attachmentFixedData;
		PxArray<PxgCudaBuffer*>									attachmentModData;

		PxgTypedCudaBuffer<PxGpuFixedTendonData>				fixedTendonParams;
		PxgTypedCudaBuffer<PxgArticulationTendon>				fixedTendons;
		PxArray<PxgCudaBuffer*>									tendonJointFixData;
		PxArray<PxgCudaBuffer*>									tendonJointCoefficientData;

		PxgTypedCudaBuffer<Dy::ArticulationMimicJointCore>		mimicJoints;

		PxgTypedCudaBuffer<Cm::UnAlignedSpatialVector>			externalAccelerations;

		PxgTypedCudaBuffer<PxReal>								jointForce;
		PxgTypedCudaBuffer<PxReal>								jointTargetPositions;
		PxgTypedCudaBuffer<PxReal>								jointTargetVelocities;
		PxgTypedCudaBuffer<PxU32>								jointOffsets;
		PxgTypedCudaBuffer<PxU32>								parents;
		PxgTypedCudaBuffer<Dy::SpatialSubspaceMatrix>			motionMatrix;
		PxgTypedCudaBuffer<Dy::SpatialSubspaceMatrix>			motionMatrixW;

		PxgTypedCudaBuffer<PxSpatialMatrix>						spatialArticulatedInertiaW;
		PxgTypedCudaBuffer<PxSpatialMatrix>						spatialImpulseResponseW;

		//see PxgArticulationLinkJointRootStateData
		PxgCudaBuffer											linkAndJointAndRootStates;

		PxgTypedCudaBuffer<PxTransform>							linkBody2Actors;

		PxgTypedCudaBuffer<ArticulationBitField>				children;
													 
		PxgTypedCudaBuffer<PxQuat>								relativeQuats;
		PxgTypedCudaBuffer<PxReal>								cfms;
		PxgTypedCudaBuffer<PxReal>								cfmScale;

		PxgTypedCudaBuffer<PxQuat>								tempParentToChilds;
		PxgTypedCudaBuffer<PxVec3>								tempRs; 

		PxU32													linkCount;
	};

}

#endif
