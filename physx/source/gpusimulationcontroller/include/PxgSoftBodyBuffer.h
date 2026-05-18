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
