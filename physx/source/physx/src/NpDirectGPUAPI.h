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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef NP_DIRECT_GPU_API_H
#define NP_DIRECT_GPU_API_H

#include "PxDirectGPUAPI.h"
#include "foundation/PxUserAllocated.h"

#if PX_SUPPORT_OMNI_PVD
#include "omnipvd/NpOmniPvdSimulationControllerCallbacks.h"
#endif

namespace physx
{

class NpScene;

class NpDirectGPUAPI : public PxDirectGPUAPI, public PxUserAllocated
{
public:
	NpDirectGPUAPI(NpScene& scene);
	virtual ~NpDirectGPUAPI() { }

	// PxDirectGPUAPI
	virtual bool getRigidDynamicData(void* data, const PxRigidDynamicGPUIndex* gpuIndices, PxRigidDynamicGPUAPIReadType::Enum dataType, PxU32 nbElements, CUevent startEvent = NULL, CUevent finishEvent = NULL) const PX_OVERRIDE PX_FINAL;
	virtual bool setRigidDynamicData(const void* data, const PxRigidDynamicGPUIndex* gpuIndices, PxRigidDynamicGPUAPIWriteType::Enum dataType, PxU32 nbElements, CUevent startEvent = NULL, CUevent finishEvent = NULL) PX_OVERRIDE PX_FINAL;

	virtual bool getArticulationData(void* data, const PxArticulationGPUIndex* gpuIndices, PxArticulationGPUAPIReadType::Enum dataType, PxU32 nbElements, CUevent startEvent = NULL, CUevent finishEvent = NULL) const PX_OVERRIDE PX_FINAL;
	virtual bool setArticulationData(const void* data, const PxArticulationGPUIndex* gpuIndices, PxArticulationGPUAPIWriteType::Enum dataType, PxU32 nbElements, CUevent startEvent = NULL, CUevent finishEvent = NULL) PX_OVERRIDE PX_FINAL;
	virtual bool computeArticulationData(void* data, const PxArticulationGPUIndex* gpuIndices, PxArticulationGPUAPIComputeType::Enum operation, PxU32 nbElements, CUevent startEvent = NULL, CUevent finishEvent = NULL) PX_OVERRIDE PX_FINAL;

	virtual bool copyContactData(void* data, PxU32* numContactPairs, PxU32 maxPairs, CUevent startEvent = NULL, CUevent finishEvent = NULL) const PX_OVERRIDE PX_FINAL;
	virtual bool evaluateSDFDistances(PxVec4* localGradientAndSDFConcatenated, const PxShapeGPUIndex* shapeIndices, const PxVec4* localSamplePointsConcatenated, const PxU32* samplePointCountPerShape, PxU32 nbElements, PxU32 maxPointCount, CUevent startEvent = NULL, CUevent finishEvent = NULL) const PX_OVERRIDE PX_FINAL;

	virtual PxArticulationGPUAPIMaxCounts getArticulationGPUAPIMaxCounts()	const	PX_OVERRIDE PX_FINAL;
	//~PxDirectGPUAPI

	NpScene& mNpScene;
#if PX_SUPPORT_OMNI_PVD
	NpOmniPvdSimulationControllerCallbacks mOvdCallback;
#endif
};

}

#endif