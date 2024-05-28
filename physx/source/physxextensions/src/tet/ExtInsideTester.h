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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

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
