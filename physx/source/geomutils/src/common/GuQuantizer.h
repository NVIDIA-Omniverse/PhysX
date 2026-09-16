// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_QUANTIZER_H
#define GU_QUANTIZER_H

#include "foundation/PxVec3.h"
#include "common/PxPhysXCommonConfig.h"

namespace physx
{
namespace Gu
{
	//////////////////////////////////////////////////////////////////////////
	// K-means quantization class
	// see http://en.wikipedia.org/wiki/K-means_clustering
	// implementation from John Ratcliff http://codesuppository.blogspot.ch/2010/12/k-means-clustering-algorithm.html
	class Quantizer
	{
	public:
		// quantize the input vertices
		virtual const PxVec3* kmeansQuantize3D(	PxU32 vcount,
												const PxVec3* vertices,
												PxU32 stride,
												bool denormalizeResults,
												PxU32 maxVertices,
												PxU32& outVertsCount) = 0;

		// returns the denormalized scale
		virtual const PxVec3& getDenormalizeScale() const = 0;

		// returns the denormalized center
		virtual const PxVec3& getDenormalizeCenter() const = 0;

		// release internal data
		virtual void release() = 0;


	protected:
		virtual ~Quantizer()
		{
		}
	};

	// creates the quantizer class
	Quantizer * createQuantizer();
}
}

#endif
