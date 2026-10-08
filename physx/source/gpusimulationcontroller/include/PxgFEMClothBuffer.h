// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_FEMCLOTH_BUFFER_H
#define PXG_FEMCLOTH_BUFFER_H

#include "PxgCudaBuffer.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxBounds3.h"

namespace physx
{

struct PxgAllocatorDesc;

class PxgFEMClothBuffer : public PxUserAllocated
{
  public:
	PxgFEMClothBuffer(PxgAllocatorDesc& allocDesc);

	PxgCudaBuffer triangleMeshData;

	PxgTypedCudaBuffer<float4> deltaPos;
	PxgTypedCudaBuffer<float4> accumulatedDeltaPos;
	PxgTypedCudaBuffer<float4> accumulatedDeltaVel; // Used for damping

	PxgTypedCudaBuffer<float4> prevPositionInContactOffset; // After contact pairs are updated, cloth vertices have moved by
															// mPosition_InvMass - mPrevPositionInContactOffset.
	PxgTypedCudaBuffer<float4> prevPositionInRestOffset;	// After cloth-cloth distance is measured, cloth vertices have moved by
															// mPosition_InvMass - mPrevPositionInRestOffset.


	PxgTypedCudaBuffer<PxU16> materialIndices;
	PxgTypedCudaBuffer<float> dynamicfrictions;

	PxgTypedCudaBuffer<PxU32> trianglesWithActiveEdges;
	PxgTypedCudaBuffer<uint4> triangleVertexIndices;
	PxgTypedCudaBuffer<uint4> orderedNonSharedTriangleVertexIndices_triIndex;

	PxgTypedCudaBuffer<float2> orderedSharedTriangleLambdas;
	PxgTypedCudaBuffer<float2> orderedNonSharedTriangleLambdas;
	
	PxgTypedCudaBuffer<float4> orderedNonSharedTriangleRestPoseInv;

	PxgTypedCudaBuffer<uint4> orderedSharedTrianglePairVertexIndices;
	PxgTypedCudaBuffer<uint4> orderedNonSharedTrianglePairVertexIndices;
	
	PxgTypedCudaBuffer<float4> orderedSharedRestBendingAngle_flexuralStiffness_damping;
	PxgTypedCudaBuffer<float4> orderedNonSharedRestBendingAngle_flexuralStiffness_damping;

	PxgTypedCudaBuffer<float4> orderedSharedRestEdge0_edge1;
	PxgTypedCudaBuffer<float4> orderedSharedRestEdgeLength_material0_material1;

	PxgTypedCudaBuffer<float> sharedBendingLambdas;
	PxgTypedCudaBuffer<float> nonSharedBendingLambdas;

	PxgTypedCudaBuffer<float4> position_InvMassCP;

	PxgTypedCudaBuffer<PxU32> nonSharedTriAccumulatedPartitionsCP;

	PxgTypedCudaBuffer<PxU32> sharedTriPairRemapOutputCP;
	PxgTypedCudaBuffer<PxU32> nonSharedTriPairRemapOutputCP;

	PxgTypedCudaBuffer<PxU32> sharedTriPairAccumulatedCopiesCP;
	PxgTypedCudaBuffer<PxU32> nonSharedTriPairAccumulatedCopiesCP;

	PxgTypedCudaBuffer<PxU32> sharedTriPairAccumulatedPartitionsCP;
	PxgTypedCudaBuffer<PxU32> nonSharedTriPairAccumulatedPartitionsCP;

	PxgTypedCudaBuffer<PxBounds3> packedNodeBounds; // for refit

	PxgTypedCudaBuffer<PxU32> numPenetratedTets;
};

}

#endif
