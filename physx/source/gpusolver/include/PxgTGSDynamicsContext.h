// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_TGS_DYNAMICS_CONTEXT_H
#define PXG_TGS_DYNAMICS_CONTEXT_H

#include "PxgContext.h"

namespace physx
{
	namespace Cm
	{
		class FlushPool;
	}

	class PxBaseTask;

	class PxsKernelWranglerManager;
	struct PxgAllocatorDesc;

	/**
	\brief A class to represent a GPU dynamics context for the GPU rigid body solver
	*/
	class PxgTGSDynamicsContext : public PxgGpuContext
	{
		PX_NOCOPY(PxgTGSDynamicsContext)

	public:
		PxgTGSDynamicsContext(Cm::FlushPool& flushPool, PxsKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager, 
			const PxGpuDynamicsMemoryConfig& config, IG::SimpleIslandManager& islandManager, PxU32 maxNumPartitions, PxU32 maxNumStaticPartitions,
			PxReal maxBiasCoefficient, PxvSimStats& simStats, PxgAllocatorDesc& allocDesc, PxReal lengthScale, PxU64 contextID, PxSceneFlags sceneFlags);

		virtual void						destroy() PX_OVERRIDE;

		virtual PxSolverType::Enum			getSolverType()	const PX_OVERRIDE { return PxSolverType::eTGS;	}
	};
}

#endif
