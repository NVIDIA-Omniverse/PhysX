// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_ARRAY_CONVERTER_H
#define PXG_ARRAY_CONVERTER_H

#include "PxArrayConverter.h"

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"

#include "PxgKernelLauncher.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

#if PX_SUPPORT_GPU_PHYSX

	class PxgArrayConverter : public PxArrayConverter, public PxUserAllocated
	{
	private:
		PxgKernelLauncher mKernelLauncher;

	public:
		PxgArrayConverter(PxgKernelLauncher& kernelLauncher);

		virtual ~PxgArrayConverter() { }

		virtual void interleaveGpuBuffers(const PxVec4* vertices, const PxVec4* normals, PxU32 length, PxVec3* interleavedResultBuffer, CUstream stream) PX_OVERRIDE;
	};

#endif	

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
