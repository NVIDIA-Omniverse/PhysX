// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_BV4_PROCESS_STREAM_ORDERED_SEGMENT_AABB_INFLATED_H
#define GU_BV4_PROCESS_STREAM_ORDERED_SEGMENT_AABB_INFLATED_H

#ifndef GU_BV4_USE_SLABS
	template<class LeafTestT, class ParamsT>
	PX_FORCE_INLINE void BV4_ProcessNodeOrdered(PxU32* PX_RESTRICT Stack, PxU32& Nb, const BVDataPacked* PX_RESTRICT node, ParamsT* PX_RESTRICT params, PxU32 i, PxU32 limit)
	{
	#ifdef GU_BV4_QUANTIZED_TREE
		if(i<limit && BV4_SegmentAABBOverlap(node+i, params->mOriginalExtents, params))
	#else
		if(i<limit && BV4_SegmentAABBOverlap(node[i].mAABB.mCenter, node[i].mAABB.mExtents, params->mOriginalExtents, params))
	#endif
		{
			if(node[i].isLeaf())
				LeafTestT::doLeafTest(params, node[i].getPrimitive());
			else
				Stack[Nb++] = node[i].getChildData();
		}
	}

	template<class LeafTestT, int i, class ParamsT>
	PX_FORCE_INLINE void BV4_ProcessNodeOrdered2(PxU32& code, const BVDataPacked* PX_RESTRICT node, ParamsT* PX_RESTRICT params)
	{
	#ifdef GU_BV4_QUANTIZED_TREE
		if(BV4_SegmentAABBOverlap(node+i, params->mOriginalExtents_Padded, params))
	#else
		if(BV4_SegmentAABBOverlap(node[i].mAABB.mCenter, node[i].mAABB.mExtents, params->mOriginalExtents_Padded, params))
	#endif
		{
			if(node[i].isLeaf())
				LeafTestT::doLeafTest(params, node[i].getPrimitive());
			else
				code |= 1<<i;
		}
	}
#endif

#endif	// GU_BV4_PROCESS_STREAM_ORDERED_SEGMENT_AABB_INFLATED_H
