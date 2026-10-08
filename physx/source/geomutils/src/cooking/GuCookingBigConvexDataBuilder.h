// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_COOKING_BIG_CONVEX_DATA_BUILDER_H
#define GU_COOKING_BIG_CONVEX_DATA_BUILDER_H

#include "foundation/PxMemory.h"
#include "foundation/PxVecMath.h"

namespace physx
{
	class BigConvexData;
	class ConvexHullBuilder;

	class BigConvexDataBuilder : public PxUserAllocated
	{
		public:
									BigConvexDataBuilder(const Gu::ConvexHullData* hull, BigConvexData* gm, const PxVec3* hullVerts);
									~BigConvexDataBuilder();
	// Support vertex map
				bool				precompute(PxU32 subdiv);				

				bool				initialize();				

				bool				save(PxOutputStream& stream, bool platformMismatch)	const;

				bool				computeValencies(const ConvexHullBuilder& meshBuilder);
	//~Support vertex map

	// Valencies
				bool				saveValencies(PxOutputStream& stream, bool platformMismatch)		const;
	//~Valencies
	protected:		
		PX_FORCE_INLINE void		precomputeSample(const PxVec3& dir, PxU8& startIndex, float negativeDir);

	private:
		const Gu::ConvexHullData*	mHull;
		BigConvexData*				mSVM;
		const	PxVec3*				mHullVerts;

	};

}

#endif
