// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_BV4_PROCESS_STREAM_NOORDER_SEGMENT_AABB_H
#define GU_BV4_PROCESS_STREAM_NOORDER_SEGMENT_AABB_H

#ifndef GU_BV4_USE_SLABS
	template<class LeafTestT, int i, class ParamsT>
	PX_FORCE_INLINE PxIntBool BV4_ProcessNodeNoOrder(PxU32* PX_RESTRICT Stack, PxU32& Nb, const BVDataPacked* PX_RESTRICT node, ParamsT* PX_RESTRICT params)
	{
	#ifdef GU_BV4_QUANTIZED_TREE
		if(BV4_SegmentAABBOverlap(node+i, params))
	#else
		if(BV4_SegmentAABBOverlap(node[i].mAABB.mCenter, node[i].mAABB.mExtents, params))
	#endif
		{
			if(node[i].isLeaf())
			{
				if(LeafTestT::doLeafTest(params, node[i].getPrimitive()))
					return 1;
			}
			else
				Stack[Nb++] = node[i].getChildData();
		}
		return 0;
	}
#endif

#endif	// GU_BV4_PROCESS_STREAM_NOORDER_SEGMENT_AABB_H
