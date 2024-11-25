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

#ifndef PXS_PARTITION_EDGE_H
#define PXS_PARTITION_EDGE_H

// PT: this is a temporary place for code related to PartitionEdge. This seems to be a GPU-only class
// but for some reason some CPU bits do need it. Ideally it would be fully contained inside the GPU DLL.

#include "PxsIslandSim.h"
#include "PxcNpWorkUnit.h"

namespace physx
{
	// PT: TODO: optimize struct size
	struct PartitionEdge
	{
		IG::EdgeIndex mEdgeIndex;	//! The edge index into the island manager. Used to identify the contact manager/constraint
		PxNodeIndex mNode0;			//! The node index for node 0. Can be obtained from the edge index alternatively
		PxNodeIndex mNode1;			//! The node idnex for node 1. Can be obtained from the edge index alternatively
		bool mInfiniteMass0;		//! Whether body 0 is kinematic
		bool mArticulation0;		//! Whether body 0 is an articulation link
		bool mInfiniteMass1;		//! Whether body 1 is kinematic
		bool mArticulation1;		//! Whether body 1 is an articulation link

		PartitionEdge* mNextPatch;	//! for the contact manager has more than 1 patch, we have next patch's edge and previous patch's edge to connect to this edge

		PxU32 mUniqueIndex;			//! a unique ID for this edge

		//KS - This constructor explicitly does not set mUniqueIndex. It is filled in by the pool allocator and this constructor
		//is called afterwards. We do not want to stomp the uniqueIndex value
		PartitionEdge() : mEdgeIndex(IG_INVALID_EDGE), mInfiniteMass0(false), mArticulation0(false),
			mInfiniteMass1(false), mArticulation1(false), mNextPatch(NULL)//, mUniqueIndex(IG_INVALID_EDGE)
		{
		}
	};

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
