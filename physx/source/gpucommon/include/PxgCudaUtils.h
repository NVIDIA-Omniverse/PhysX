// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_CUDA_UTILS_H
#define PXG_CUDA_UTILS_H

#include "cuda.h"

#include "foundation/PxErrors.h"
#include "foundation/PxAssert.h"
#include "foundation/PxFoundation.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxTime.h"

#include "cudamanager/PxCudaContext.h"

namespace physx
{

	/**
	Utility function to synchronize 2 streams. This causes dependentStream to wait for parentStream to complete its current queued workload before proceeding further.
	*/
	PX_INLINE void synchronizeStreams(PxCudaContext* cudaContext, const CUstream& parentStream, const CUstream& dependentStream)
	{
		CUevent ev = 0;
		cudaContext->eventCreate(&ev, CU_EVENT_DISABLE_TIMING);

		CUresult result = cudaContext->eventRecord(ev, parentStream);		

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "SynchronizeStreams cuEventRecord failed with error %i\n", result);
		PX_ASSERT(result == CUDA_SUCCESS);

		result = cudaContext->streamWaitEvent(dependentStream, ev);		

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "SynchronizeStreams cuStreamWaitEvent failed with error %i\n", result);
		PX_ASSERT(result == CUDA_SUCCESS);

		cudaContext->eventDestroy(ev);
	}

	/**
	Utility function to synchronize 2 streams. This causes dependentStream to wait for parentStream to complete its current queued workload before proceeding further.
	*/
	PX_INLINE void synchronizeStreams(PxCudaContext* cudaContext, CUstream& parentStream, CUstream& dependentStream, CUevent& ev)
	{
		//CUevent ev;
		//mCudaContext->eventCreate(&ev, CU_EVENT_DISABLE_TIMING);

		CUresult result = cudaContext->eventRecord(ev, parentStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "SynchronizeStreams cuEventRecord failed with error %i\n", result);

		PX_ASSERT(result == CUDA_SUCCESS);

		result = cudaContext->streamWaitEvent(dependentStream, ev);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "SynchronizeStreams cuStreamWaitEvent failed with error %i\n", result);

		PX_ASSERT(result == CUDA_SUCCESS);

		
		//mCudaContext->eventDestroy(ev);
	}

	PX_FORCE_INLINE void* getMappedDevicePtr(PxCudaContext* cudaContext, void* cpuPtr)
	{
		CUdeviceptr dPtr = 0;
		cudaContext->memHostGetDevicePointer(&dPtr, cpuPtr, 0);
		return reinterpret_cast<void*>(dPtr);
	}

	PX_FORCE_INLINE const void* getMappedDeviceConstPtr(PxCudaContext* cudaContext, const void* cpuPtr)
	{
		return getMappedDevicePtr(cudaContext, const_cast<void*>(cpuPtr));
	}

	PX_FORCE_INLINE bool spinWait(volatile PxU32& waitValue, const PxReal timeoutValue)
	{
		PxTime time;
		while (waitValue == 0)
		{
			if (PxReal(time.peekElapsedSeconds()) >= timeoutValue)
				return false;
		}
		return true;
	}
}

#endif