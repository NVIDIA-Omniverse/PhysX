// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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

	virtual bool getD6JointData(void* data, const PxD6JointGPUIndex* gpuIndices, PxD6JointGPUAPIReadType::Enum dataType, PxU32 nbElements, CUevent startEvent = NULL, CUevent finishEvent = NULL) const PX_OVERRIDE PX_FINAL;
	//~PxDirectGPUAPI

	NpScene& mNpScene;
#if PX_SUPPORT_OMNI_PVD
	NpOmniPvdSimulationControllerCallbacks mOvdCallback;
#endif
};

}

#endif