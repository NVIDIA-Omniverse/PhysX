// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_DEFORMABLE_SKINNING_H
#define PXG_DEFORMABLE_SKINNING_H

#include "PxDeformableSkinning.h"

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"

#include "PxgKernelLauncher.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

#if PX_SUPPORT_GPU_PHYSX

class PxgDeformableSkinning : public PxDeformableSkinning, public PxUserAllocated
{
private:
	PxgKernelLauncher mKernelLauncher;

public:
	PxgDeformableSkinning(PxgKernelLauncher& kernelLauncher);

	virtual ~PxgDeformableSkinning() { }

	virtual void computeNormalVectors(
		PxTrimeshSkinningGpuData* skinningDataArrayD, PxU32 arrayLength,
		CUstream stream, PxU32 numGpuThreads) PX_OVERRIDE;

	virtual void evaluateVerticesEmbeddedIntoSurface(
		PxTrimeshSkinningGpuData* skinningDataArrayD, PxU32 arrayLength,
		CUstream stream, PxU32 numGpuThreads) PX_OVERRIDE;

	virtual void evaluateVerticesEmbeddedIntoVolume(
		PxTetmeshSkinningGpuData* skinningDataArrayD, PxU32 arrayLength,
		CUstream stream, PxU32 numGpuThreads) PX_OVERRIDE;
};

#endif	

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
