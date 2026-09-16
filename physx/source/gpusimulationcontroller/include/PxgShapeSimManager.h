// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_SHAPESIM_MANAGER_H
#define	PXG_SHAPESIM_MANAGER_H

#include "foundation/PxArray.h"
#include "common/PxPhysXCommonConfig.h"

#include "CmPinnableArray.h"

#include "PxgShapeSim.h"
#include "PxgCudaBuffer.h"

#define PXG_SC_DEBUG 0

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
	struct PxsCachedTransform;
	class PxBaseTask;
	struct PxgAllocatorDesc;
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
														PxgShapeSimManager(PxgAllocatorDesc& allocDesc);

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

#if PXG_SC_DEBUG
		void											validateCacheAndBounds(const PxBounds3* bounds, const PxsCachedTransform* cachedTransforms);
#endif

		private:
						PxArray<PxgShapeSimData>		mShapeSims;
						PxArray<Sc::ShapeSimBase*>		mShapeSimPtrs;
						PxArray<PxU32>					mNewShapeSims;
						PxU32							mTotalNumShapes;
						PxU32							mNbTotalShapeSim;

						Cm::PinnableArray<PxgNewShapeSim>	mPxgShapeSimPool;
						PxgTypedCudaBuffer<PxgShapeSim>		mShapeSimBuffer;
						PxgTypedCudaBuffer<PxgNewShapeSim>	mNewShapeSimBuffer;

		friend class PxgCopyToShapeSimTask;
	};
}

#endif
