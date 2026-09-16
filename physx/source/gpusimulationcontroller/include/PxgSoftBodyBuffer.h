// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_SOFTBODY_BUFFER_H
#define PXG_SOFTBODY_BUFFER_H

#include "PxgCudaBuffer.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxMat33.h"
#include "foundation/PxBounds3.h"

namespace physx
{

struct PxgAllocatorDesc;
struct PxgMat33Block;
struct PxgSpatialVectorBlock;
class PxgNonRigidFilterPair;

class PxgSoftBodyBuffer : public PxUserAllocated
{
public:

	PxgSoftBodyBuffer(PxgAllocatorDesc& allocDesc);

	PxgCudaBuffer						tetMeshData;
	PxgTypedCudaBuffer<PxU8>			tetMeshSurfaceHint;
	PxgTypedCudaBuffer<uint4>			tetIndices;
	PxgTypedCudaBuffer<PxU32>			tetIndicesRemapTable;
	PxgTypedCudaBuffer<PxMat33>			tetStresses;
	PxgTypedCudaBuffer<PxReal>			tetStressCoefficient;
	PxgTypedCudaBuffer<PxMat33>			tetRestPoses;
	PxgTypedCudaBuffer<float4>			tetRotations;

	PxgTypedCudaBuffer<uint4>			tetIndicesGM;
	PxgTypedCudaBuffer<float4>			pPostion_InvMassGM;
	PxgTypedCudaBuffer<bool>			vertsAreDeformed;
	PxgTypedCudaBuffer<bool>			vertsCantDeform;
	PxgTypedCudaBuffer<PxgMat33Block>	tetRestPosesGM;
	PxgTypedCudaBuffer<PxgMat33Block>	origTetRestPosesGM;
	PxgTypedCudaBuffer<float4>			tetRotationsGM;
	PxgTypedCudaBuffer<PxU32>			orderedTetGM;
	PxgTypedCudaBuffer<PxU32>			jacobiVertIndicesGM;
	PxgTypedCudaBuffer<PxgSpatialVectorBlock>	tetMultipliersGM;

	PxgTypedCudaBuffer<float4>			pDeltaVGM;

	PxgTypedCudaBuffer<float4>			pBarycentricGM;
	PxgTypedCudaBuffer<PxU32>			pRemapGM;
	PxgTypedCudaBuffer<PxU32>			tetRemapColToSim;
	PxgTypedCudaBuffer<PxU32>			tetAccumulatedRemapColToSim;
	PxgTypedCudaBuffer<PxU8>			surfaceVertsHint;
	PxgTypedCudaBuffer<PxU32>			surfaceVertToTetRemap;
	PxgTypedCudaBuffer<float4>			pDeltaPosGM;
	PxgTypedCudaBuffer<float4>			pPosition_InvMassGMCP;
	PxgTypedCudaBuffer<float4>			pVelocity_InvMassGMCP;
	PxgTypedCudaBuffer<PxU32>			remapOutputGMCP;
	PxgTypedCudaBuffer<PxU32>			accumulatedPartitionsGMCP;
	PxgTypedCudaBuffer<PxU32>			accumulatedCopiesGMCP;
	PxgTypedCudaBuffer<uint4>			pullIndices;
	PxgTypedCudaBuffer<PxU16>			orderedMaterialIndices;
	PxgTypedCudaBuffer<PxU16>			materialIndices;
	PxgTypedCudaBuffer<PxBounds3>		packedNodeBounds; //for refit
	PxgTypedCudaBuffer<PxgNonRigidFilterPair> filterPairs;
};

}

#endif
