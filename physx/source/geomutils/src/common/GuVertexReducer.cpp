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

#include "GuVertexReducer.h"
#include "foundation/PxAllocator.h"
#include "CmRadixSort.h"

using namespace physx;
using namespace Gu;
using namespace Cm;

// PT: code archeology: this initially came from ICE (IceVertexCloud.h/cpp). Consider dropping it.

ReducedVertexCloud::ReducedVertexCloud(const PxVec3* verts, PxU32 nb_verts) : mNbRVerts(0), mRVerts(NULL), mXRef(NULL)
{
	mVerts		= verts;
	mNbVerts	= nb_verts;
}

ReducedVertexCloud::~ReducedVertexCloud()
{
	clean();
}

ReducedVertexCloud& ReducedVertexCloud::clean()
{
	PX_FREE(mXRef);
	PX_FREE(mRVerts);
	return *this;
}

/**
*	Reduction method. Use this to create a minimal vertex cloud.
*	\param		rc		[out] result structure
*	\return		true if success
*	\warning	This is not about welding nearby vertices, here we look for real redundant ones.
*/
bool ReducedVertexCloud::reduce(REDUCEDCLOUD* rc)
{
	clean();

	mXRef = PX_ALLOCATE(PxU32, mNbVerts, "mXRef");

	float* f = PX_ALLOCATE(float, mNbVerts, "tmp");

	for(PxU32 i=0;i<mNbVerts;i++)
		f[i] = mVerts[i].x;

	RadixSortBuffered Radix;
	Radix.Sort(reinterpret_cast<const PxU32*>(f), mNbVerts, RADIX_UNSIGNED);

	for(PxU32 i=0;i<mNbVerts;i++)
		f[i] = mVerts[i].y;
	Radix.Sort(reinterpret_cast<const PxU32*>(f), mNbVerts, RADIX_UNSIGNED);

	for(PxU32 i=0;i<mNbVerts;i++)
		f[i] = mVerts[i].z;
	const PxU32* Sorted = Radix.Sort(reinterpret_cast<const PxU32*>(f), mNbVerts, RADIX_UNSIGNED).GetRanks();

	PX_FREE(f);

	mNbRVerts = 0;
	const PxU32 Junk[] = {PX_INVALID_U32, PX_INVALID_U32, PX_INVALID_U32};
	const PxU32* Previous = Junk;
	mRVerts = PX_ALLOCATE(PxVec3, mNbVerts, "PxVec3");
	PxU32 Nb = mNbVerts;
	while(Nb--)
	{
		const PxU32 Vertex = *Sorted++;	// Vertex number

		const PxU32* current = reinterpret_cast<const PxU32*>(&mVerts[Vertex]);
		if(current[0]!=Previous[0] || current[1]!=Previous[1] || current[2]!=Previous[2])
			mRVerts[mNbRVerts++] = mVerts[Vertex];

		Previous = current;

		mXRef[Vertex] = mNbRVerts-1;
	}

	if(rc)
	{
		rc->CrossRef	= mXRef;
		rc->NbRVerts	= mNbRVerts;
		rc->RVerts		= mRVerts;
	}
	return true;
}
