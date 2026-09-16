// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_BV4_SLABS_SWIZZLED_NO_ORDER_H
#define GU_BV4_SLABS_SWIZZLED_NO_ORDER_H

	// Generic, no sort
/*	template<class LeafTestT, class ParamsT>
	static PxIntBool BV4_ProcessStreamSwizzledNoOrder(const BVDataPacked* PX_RESTRICT node, PxU32 initData, ParamsT* PX_RESTRICT params)
	{
		const BVDataPacked* root = node;

		PxU32 nb=1;
		PxU32 stack[GU_BV4_STACK_SIZE];
		stack[0] = initData;

		do
		{
			const PxU32 childData = stack[--nb];
			node = root + getChildOffset(childData);

			const BVDataSwizzled* tn = reinterpret_cast<const BVDataSwizzled*>(node);

			const PxU32 nodeType = getChildType(childData);

			if(nodeType>1 && BV4_ProcessNodeNoOrder_Swizzled<LeafTestT, 3>(stack, nb, tn, params))
				return 1;
			if(nodeType>0 && BV4_ProcessNodeNoOrder_Swizzled<LeafTestT, 2>(stack, nb, tn, params))
				return 1;
			if(BV4_ProcessNodeNoOrder_Swizzled<LeafTestT, 1>(stack, nb, tn, params))
				return 1;
			if(BV4_ProcessNodeNoOrder_Swizzled<LeafTestT, 0>(stack, nb, tn, params))
				return 1;

		}while(nb);

		return 0;
	}*/

	template<class LeafTestT, class ParamsT>
	static PxIntBool BV4_ProcessStreamSwizzledNoOrderQ(const BVDataPackedQ* PX_RESTRICT node, PxU32 initData, ParamsT* PX_RESTRICT params)
	{
		const BVDataPackedQ* root = node;

		PxU32 nb=1;
		PxU32 stack[GU_BV4_STACK_SIZE];
		stack[0] = initData;

		do
		{
			const PxU32 childData = stack[--nb];
			node = root + getChildOffset(childData);

			const BVDataSwizzledQ* tn = reinterpret_cast<const BVDataSwizzledQ*>(node);

			const PxU32 nodeType = getChildType(childData);

			if(nodeType>1 && BV4_ProcessNodeNoOrder_SwizzledQ<LeafTestT, 3>(stack, nb, tn, params))
				return 1;
			if(nodeType>0 && BV4_ProcessNodeNoOrder_SwizzledQ<LeafTestT, 2>(stack, nb, tn, params))
				return 1;
			if(BV4_ProcessNodeNoOrder_SwizzledQ<LeafTestT, 1>(stack, nb, tn, params))
				return 1;
			if(BV4_ProcessNodeNoOrder_SwizzledQ<LeafTestT, 0>(stack, nb, tn, params))
				return 1;

		}while(nb);

		return 0;
	}

	template<class LeafTestT, class ParamsT>
	static PxIntBool BV4_ProcessStreamSwizzledNoOrderNQ(const BVDataPackedNQ* PX_RESTRICT node, PxU32 initData, ParamsT* PX_RESTRICT params)
	{
		const BVDataPackedNQ* root = node;

		PxU32 nb=1;
		PxU32 stack[GU_BV4_STACK_SIZE];
		stack[0] = initData;

		do
		{
			const PxU32 childData = stack[--nb];
			node = root + getChildOffset(childData);

			const BVDataSwizzledNQ* tn = reinterpret_cast<const BVDataSwizzledNQ*>(node);

			const PxU32 nodeType = getChildType(childData);

			if(nodeType>1 && BV4_ProcessNodeNoOrder_SwizzledNQ<LeafTestT, 3>(stack, nb, tn, params))
				return 1;
			if(nodeType>0 && BV4_ProcessNodeNoOrder_SwizzledNQ<LeafTestT, 2>(stack, nb, tn, params))
				return 1;
			if(BV4_ProcessNodeNoOrder_SwizzledNQ<LeafTestT, 1>(stack, nb, tn, params))
				return 1;
			if(BV4_ProcessNodeNoOrder_SwizzledNQ<LeafTestT, 0>(stack, nb, tn, params))
				return 1;

		}while(nb);

		return 0;
	}

#endif // GU_BV4_SLABS_SWIZZLED_NO_ORDER_H
