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

#ifndef GU_TRIANGLE_H
#define GU_TRIANGLE_H

#include "foundation/PxVec3.h"
#include "foundation/PxUtilities.h"
#include "foundation/PxUserAllocated.h"

namespace physx
{
namespace Gu
{
	// PT: I'm taking back control of these files and re-introducing the "ICE" naming conventions:
	// - "Triangle" is for actual triangles (like the PxTriangle class)
	// - If it contains vertex indices, it's "IndexedTriangle".
	// - "v" is too ambiguous (it could be either an actual vertex or a vertex reference) so use "ref" instead.
	//   Plus we sometimes reference edges, not vertices, so "v" is too restrictive.

	template <class T>
	struct IndexedTriangleT : public PxUserAllocated
	{
		PX_INLINE	IndexedTriangleT ()										{}
		PX_INLINE	IndexedTriangleT (T a, T b, T c)						{ mRef[0] = a; mRef[1] = b; mRef[2] = c; }
		template <class TX>
		PX_INLINE	IndexedTriangleT (const IndexedTriangleT <TX>& other)	{ mRef[0] = other[0]; mRef[1] = other[1]; mRef[2] = other[2]; }

		PX_INLINE	T&				operator[](T i)				{ return mRef[i]; }
		PX_INLINE	const T&		operator[](T i)		const	{ return mRef[i]; }

		template<class TX>//any type of IndexedTriangleT <>, possibly with different T
		PX_INLINE	IndexedTriangleT <T>&	operator=(const IndexedTriangleT <TX>& i)	{ mRef[0]=i[0]; mRef[1]=i[1]; mRef[2]=i[2]; return *this; }

		void	flip()
		{
			PxSwap(mRef[1], mRef[2]);
		}

		PX_INLINE bool contains(T id) const
		{
			return mRef[0] == id || mRef[1] == id || mRef[2] == id;
		}

		PX_INLINE void center(const PxVec3* verts, PxVec3& center)	const
		{
			const PxVec3& p0 = verts[mRef[0]];
			const PxVec3& p1 = verts[mRef[1]];
			const PxVec3& p2 = verts[mRef[2]];
			center = (p0+p1+p2)*0.33333333333333333333f;
		}

		float area(const PxVec3* verts)	const
		{
			const PxVec3& p0 = verts[mRef[0]];
			const PxVec3& p1 = verts[mRef[1]];
			const PxVec3& p2 = verts[mRef[2]];
			return ((p0-p1).cross(p0-p2)).magnitude() * 0.5f;
		}

		PxU8	findEdge(T vref0, T vref1)	const
		{
					if(mRef[0]==vref0 && mRef[1]==vref1)	return 0;
			else	if(mRef[0]==vref1 && mRef[1]==vref0)	return 0;
			else	if(mRef[0]==vref0 && mRef[2]==vref1)	return 1;
			else	if(mRef[0]==vref1 && mRef[2]==vref0)	return 1;
			else	if(mRef[1]==vref0 && mRef[2]==vref1)	return 2;
			else	if(mRef[1]==vref1 && mRef[2]==vref0)	return 2;
			return 0xff;
		}

		// counter clock wise order
		PxU8	findEdgeCCW(T vref0, T vref1)	const
		{
					if(mRef[0]==vref0 && mRef[1]==vref1)	return 0;
			else	if(mRef[0]==vref1 && mRef[1]==vref0)	return 0;
			else	if(mRef[0]==vref0 && mRef[2]==vref1)	return 2;
			else	if(mRef[0]==vref1 && mRef[2]==vref0)	return 2;
			else	if(mRef[1]==vref0 && mRef[2]==vref1)	return 1;
			else	if(mRef[1]==vref1 && mRef[2]==vref0)	return 1;
			return 0xff;
		}

		bool	replaceVertex(T oldref, T newref)
		{
					if(mRef[0]==oldref)	{ mRef[0] = newref; return true; }
			else	if(mRef[1]==oldref)	{ mRef[1] = newref; return true; }
			else	if(mRef[2]==oldref)	{ mRef[2] = newref; return true; }
			return false;
		}

		bool isDegenerate()	const
		{
			if(mRef[0]==mRef[1])	return true;
			if(mRef[1]==mRef[2])	return true;
			if(mRef[2]==mRef[0])	return true;
			return false;
		}

		PX_INLINE void denormalizedNormal(const PxVec3* verts, PxVec3& normal)	const
		{
			const PxVec3& p0 = verts[mRef[0]];
			const PxVec3& p1 = verts[mRef[1]];
			const PxVec3& p2 = verts[mRef[2]];
			normal = ((p2 - p1).cross(p0 - p1));
		}

		T	mRef[3];	//vertex indices
	};

	typedef IndexedTriangleT<PxU32>	IndexedTriangle32;
	typedef IndexedTriangleT<PxU16>	IndexedTriangle16;

	PX_COMPILE_TIME_ASSERT(sizeof(IndexedTriangle32)==12);
	PX_COMPILE_TIME_ASSERT(sizeof(IndexedTriangle16)==6);
}
}

#endif
