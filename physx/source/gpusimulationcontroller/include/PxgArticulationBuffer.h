// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
