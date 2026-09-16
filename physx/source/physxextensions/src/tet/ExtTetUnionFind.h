// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef UNION_FIND_H
#define UNION_FIND_H

#include "foundation/PxArray.h"

namespace physx
{
namespace Ext
{
	class UnionFind {
	public:
		UnionFind() {}
		UnionFind(PxI32 numSets) { init(numSets); }

		void init(PxI32 numSets);
		PxI32 find(PxI32 x);
		void makeSet(PxI32 x, PxI32 y);

		PxI32 computeSetNrs();
		PxI32 getSetNr(PxI32 x);

	private:
		struct Entry {
			PxI32 parent, rank;
			PxI32 setNr;
		};

		PxArray<Entry> mEntries;
	};
}
}

#endif
