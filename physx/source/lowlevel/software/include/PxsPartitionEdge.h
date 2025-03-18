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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXS_PARTITION_EDGE_H
#define PXS_PARTITION_EDGE_H

// PT: this is a temporary place for code related to PartitionEdge. This seems to be a GPU-only class
// but for some reason some CPU bits do need it. Ideally it would be fully contained inside the GPU DLL.

#include "PxsIslandSim.h"
#include "PxcNpWorkUnit.h"

namespace physx
{
	// PT: TODO: mNextPatch is almost always null so it would make sense to store that cold data somewhere else, e.g:
	// - use one bit to mark the general case where mNextPatch is null
	// - store non-null mNextPatch in a hashmap indexed by mUniqueIndex (no need to reserve the full memory for it)
	//
	// The only annoying bit is that the mechanism needed to actually walk the linked list (i.e. the hashmap) needs to be
	// available in processPartitionEdges below, so that's more GPU stuff exposed to the CPU code. But I guess we crossed
	// that line a while ago when the heads of the LLs moved to the low-level island manager anyway. And in fact we could
	// put the two in the same structure eventually, like a PartitionEdgeManager.
	//
	// In any case the benefit of the change would be a smaller PartitionEdge. Going further, the nodes can be retrieved from
	// the edge index, so if mNextPatch also disappears then only the unique ID remains, which.... can be derived from
	// the PartitionEdge address?! So it would be just the "edge index" with some bits encoded in it, just 4 bytes.
	//
	// This is per-edge data so we could also merge this with CPUExternalData, which already contains the node indices, and
	// has an implicit unique index as the index into mEdgeNodeIndices. But maybe we cannot because there can be multiple
	// PartitionEdge for the same source edge (hence the linked list).
	//
	// Another idea would be to store the edge index instead. You would need access to the edge manager in CPU code but no
	// hashmap or new structure is needed.

	struct PartitionEdge
	{
		enum Enum
		{
			HAS_INFINITE_MASS0	= (1<<0),
			HAS_INFINITE_MASS1	= (1<<1),
			HAS_THRESHOLD		= (1<<2),
			IS_CONTACT			= (1<<3),
			SPECIAL_HANDLED		= (1<<4),

			NB_BITS	= 5
		};

		PxNodeIndex		mNode0;			//! The node index for node 0. Can be obtained from the edge index alternatively
		PxNodeIndex		mNode1;			//! The node index for node 1. Can be obtained from the edge index alternatively
		PartitionEdge*	mNextPatch;		//! for the contact manager has more than 1 patch, we have next patch's edge and previous patch's edge to connect to this edge
		private:
		IG::EdgeIndex	mEdgeIndex;		//! The edge index into the island manager. Used to identify the contact manager/constraint
		public:
		PxU32			mUniqueIndex;	//! a unique ID for this edge

		PX_FORCE_INLINE	IG::EdgeIndex	getEdgeIndex()		const	{ return mEdgeIndex >> NB_BITS;	}

		PX_FORCE_INLINE	PxU32			isArticulation0()	const	{ return mNode0.isArticulation();	}
		PX_FORCE_INLINE	PxU32			isArticulation1()	const	{ return mNode1.isArticulation();	}

		PX_FORCE_INLINE	PxU32			hasInfiniteMass0()	const	{ return mEdgeIndex & HAS_INFINITE_MASS0;	}
		PX_FORCE_INLINE	PxU32			hasInfiniteMass1()	const	{ return mEdgeIndex & HAS_INFINITE_MASS1;	}

		PX_FORCE_INLINE	void			setInfiniteMass0()			{ mEdgeIndex |= HAS_INFINITE_MASS0;	}
		PX_FORCE_INLINE	void			setInfiniteMass1()			{ mEdgeIndex |= HAS_INFINITE_MASS1;	}

		PX_FORCE_INLINE	void			setHasThreshold()			{ mEdgeIndex |= HAS_THRESHOLD;			}
		PX_FORCE_INLINE	PxU32			hasThreshold()		const	{ return mEdgeIndex & HAS_THRESHOLD;	}

		PX_FORCE_INLINE	void			setIsContact()				{ mEdgeIndex |= IS_CONTACT;			}
		PX_FORCE_INLINE	PxU32			isContact()			const	{ return mEdgeIndex & IS_CONTACT;	}

		PX_FORCE_INLINE	void			setSpecialHandled()			{ mEdgeIndex |= SPECIAL_HANDLED;		}
		PX_FORCE_INLINE	void			clearSpecialHandled()		{ mEdgeIndex &= ~SPECIAL_HANDLED;		}
		PX_FORCE_INLINE	PxU32			isSpecialHandled()	const	{ return mEdgeIndex & SPECIAL_HANDLED;	}

		//KS - This constructor explicitly does not set mUniqueIndex. It is filled in by the pool allocator and this constructor
		//is called afterwards. We do not want to stomp the uniqueIndex value
		PartitionEdge(IG::EdgeIndex index) :
			mNextPatch(NULL),
			mEdgeIndex(index << NB_BITS)
		{
			PX_ASSERT(!(index & 0xf8000000));	// PT: reserve 5 bits for internal flags
		}
	};
	PX_COMPILE_TIME_ASSERT(sizeof(PartitionEdge)<=32);	// PT: 2 of them per cache-line

	static PX_FORCE_INLINE void processPartitionEdges(const IG::GPUExternalData* gpuData, const PxcNpWorkUnit& unit)
	{
		if(gpuData && !(unit.mFlags & PxcNpWorkUnitFlag::eDISABLE_RESPONSE))
		{
			PxU32* edgeNodeIndices = gpuData->getEdgeNodeIndexPtr();
			if(edgeNodeIndices)	// PT: only non-null for GPU version
			{
				const PartitionEdge* partitionEdge = gpuData->getFirstPartitionEdge(unit.mEdgeIndex);
				while(partitionEdge)
				{
					edgeNodeIndices[partitionEdge->mUniqueIndex] = unit.mNpIndex;
					partitionEdge = partitionEdge->mNextPatch;
				}
			}
		}
	}
}

#endif
