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

#include "ExtTetUnionFind.h"

namespace physx
{
namespace Ext
{
	//------------------------------------------------------------------------------------
	void UnionFind::init(PxI32 numSets)
	{
		mEntries.resize(numSets);
		for (PxI32 i = 0; i < numSets; i++) {
			Entry &e = mEntries[i];
			e.parent = i;
			e.rank = 0;
			e.setNr = i;
		}
	}

	//------------------------------------------------------------------------------------
	PxI32 UnionFind::find(PxI32 x)
	{
		if (mEntries[x].parent != x)
			mEntries[x].parent = find(mEntries[x].parent);
		return mEntries[x].parent;
	}

	//------------------------------------------------------------------------------------
	void UnionFind::makeSet(PxI32 x, PxI32 y)
	{
		PxI32 xroot = find(x);
		PxI32 yroot = find(y);
		if (xroot == yroot)
			return;
		if (mEntries[xroot].rank < mEntries[yroot].rank)
			mEntries[xroot].parent = yroot;
		else if (mEntries[xroot].rank > mEntries[yroot].rank)
			mEntries[yroot].parent = xroot;
		else {
			mEntries[yroot].parent = xroot;
			mEntries[xroot].rank++;
		}
	}

	//------------------------------------------------------------------------------------
	PxI32 UnionFind::computeSetNrs()
	{
		PxArray<PxI32> oldToNew(mEntries.size(), -1);
		PxI32 numSets = 0;

		for (PxU32 i = 0; i < mEntries.size(); i++) {
			PxI32 nr = find(i);
			if (oldToNew[nr] < 0)
				oldToNew[nr] = numSets++;
			mEntries[i].setNr = oldToNew[nr];
		}
		return numSets;
	}

	//------------------------------------------------------------------------------------
	PxI32 UnionFind::getSetNr(PxI32 x)
	{
		return mEntries[x].setNr;
	}
}
}
