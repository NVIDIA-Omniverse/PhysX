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

#ifndef PXG_PARTITION_NODE_H
#define PXG_PARTITION_NODE_H

#include "PxNodeIndex.h"

namespace physx
{
	struct PartitionIndexData
	{
		PxU16 mPartitionIndex;		//! The current partition this edge is in. Used to find the edge efficiently. PxU8 is probably too small (256 partitions max) but PxU16 should be more than enough
		PxU8 mPatchIndex;			//! The patch index for this partition edge. There may be multiple entries for a given edge if there are multiple patches.
		PxU8 mCType;				//! The type of constraint this is (PxgEdgeType)
		PxU32 mPartitionEntryIndex;	//! index of partition edges for this partition
	};

	// PT: stored in incremental partition code's mPartitionNodeArray,
	// indexed by a partition edge's unique index.
	struct PartitionNodeData
	{
		// PT: copies of PartitionEdge' node indices (the nodes connected by this edge)
		// - created in PxgIncrementalPartition::addEdge_Stage1
		PxNodeIndex mNodeIndex0;
		PxNodeIndex mNodeIndex1;

		// PT: links to next edge unique indices containing the same nodes
		// - computed in PxgIncrementalPartition::addEdge_Stage2 => PxgIncrementalPartition::addEdgeInternal
		// - used in constraintContactBlockPrePrepLaunch / constraint1DBlockPrePrepLaunch
		// - unclear what we need this for
		PxU32 mNextIndex[2];
	};
}
#endif