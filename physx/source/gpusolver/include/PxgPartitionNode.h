// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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