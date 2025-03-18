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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXG_SHAPESIM_MANAGER_H
#define	PXG_SHAPESIM_MANAGER_H

#include "foundation/PxArray.h"
#include "foundation/PxPinnedArray.h"
#include "PxgShapeSim.h"
#include "PxgCudaBuffer.h"

namespace physx
{
	namespace Cm
	{
		class FlushPool;
	}

	namespace Sc
	{
		class ShapeSimBase;
	}

	struct PxsShapeCore;
	class PxBaseTask;
	class PxgHeapMemoryAllocatorManager;
	class PxgGpuNarrowphaseCore;
	class KernelWrangler;

	struct PxgShapeSimData
	{
		PxgShapeSimData() : mShapeCore(NULL), mElementIndex_GPU(PX_INVALID_U32)
		{
		}

		const PxsShapeCore*	mShapeCore;		//	4 or 8

		// NodeIndex used to look up BodySim in island manager
		PxNodeIndex		mBodySimIndex_GPU;	//	8 or 12	unique identified for body

		// ElementID - copy of ElementSim's getElementID()
		PxU32			mElementIndex_GPU;	//	12	or	16	transform cache and bound index
	};

	class PxgShapeSimManager
	{
														PX_NOCOPY(PxgShapeSimManager)
		public:
														PxgShapeSimManager(PxgHeapMemoryAllocatorManager* heapMemoryManager);

						void							addPxgShape(Sc::ShapeSimBase* shapeSimBase, const PxsShapeCore* shapeCore, PxNodeIndex nodeIndex, PxU32 index);
						void							setPxgShapeBodyNodeIndex(PxNodeIndex nodeIndex, PxU32 index);
						void							removePxgShape(PxU32 index);

		// PT: copies new shapes from CPU memory (mShapeSims) to GPU *host* memory (mPxgShapeSimPool)
						void							copyToGpuShapeSim(PxgGpuNarrowphaseCore* npCore, PxBaseTask* continuation, Cm::FlushPool& flushPool);
		// PT: copies new shapes from GPU *host* memory (mPxgShapeSimPool) to GPU device memory (mNewShapeSimBuffer)
		// and *then* copies from device-to-device memory (mNewShapeSimBuffer => mShapeSimBuffer)
						void							gpuMemDmaUpShapeSim(PxCudaContext* cudaContext, CUstream stream, KernelWrangler* kernelWrangler);

		// PT: TODO: figure out the difference between mTotalNumShapes and mNbTotalShapeSim
		// (they both existed in different places and got logically refactored here)
		PX_FORCE_INLINE	PxU32							getTotalNbShapes()				const	{ return mTotalNumShapes;	}
		PX_FORCE_INLINE	PxU32							getNbTotalShapeSims()			const	{ return mNbTotalShapeSim;	}

		PX_FORCE_INLINE	CUdeviceptr						getShapeSimsDevicePtr()			const	{ return mShapeSimBuffer.getDevicePtr();	}
		PX_FORCE_INLINE	const PxgShapeSim*				getShapeSimsDeviceTypedPtr()	const	{ return mShapeSimBuffer.getTypedPtr();		}
		PX_FORCE_INLINE	Sc::ShapeSimBase**				getShapeSims()							{ return mShapeSimPtrs.begin();				}

		private:
						PxArray<PxgShapeSimData>		mShapeSims;
						PxArray<Sc::ShapeSimBase*>		mShapeSimPtrs;
						PxArray<PxU32>					mNewShapeSims;
						PxU32							mTotalNumShapes;
						PxU32							mNbTotalShapeSim;

						PxPinnedArray<PxgNewShapeSim>		mPxgShapeSimPool;
						PxgTypedCudaBuffer<PxgShapeSim>		mShapeSimBuffer;
						PxgTypedCudaBuffer<PxgNewShapeSim>	mNewShapeSimBuffer;

		friend class PxgCopyToShapeSimTask;
	};
}

#endif
