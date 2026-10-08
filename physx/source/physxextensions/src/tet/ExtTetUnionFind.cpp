// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ExtTetUnionFind.h"

using namespace physx;
using namespace Ext;

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
