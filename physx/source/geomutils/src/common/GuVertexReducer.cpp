// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
