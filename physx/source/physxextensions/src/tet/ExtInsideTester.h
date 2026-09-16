// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_INSIDE_TESTER_H
#define EXT_INSIDE_TESTER_H

// MM: tester whether a point is inside a triangle mesh
// all faces are projected onto the 3 canonical planes and hashed
// for fast ray mesh intersections

#include "foundation/PxVec3.h"
#include "foundation/PxArray.h"
#include "foundation/PxQuat.h"
#include "CmRandom.h"

namespace physx
{
	namespace Ext
	{

		// ----------------------------------------------------------
		class InsideTester 
		{
		public:
			void init(const PxVec3 *vertices, PxI32 numVertices, const PxI32 *triIndices, PxI32 numTris);
			bool isInside(const PxVec3& pos);

		private:
			PxArray<PxVec3> mVertices;
			PxArray<PxI32> mIndices;

			struct Grid2d
			{
				void init(PxI32 dim0, const PxArray<PxVec3> &vertices, const PxArray<PxI32> &indices);
				PxI32 numInside(const PxVec3&pos, const PxArray<PxVec3> &vertices, const PxArray<PxI32> &indices);
				PxI32 dim0;
				PxVec3 orig;
				PxI32 num1, num2;
				float spacing;
				PxArray<PxI32> first;
				PxArray<PxI32> tris;
				PxArray<int> next;

				Cm::RandomR250 rnd = Cm::RandomR250(0);
			};
			Grid2d mGrids[3];
		};
	}
}

#endif
