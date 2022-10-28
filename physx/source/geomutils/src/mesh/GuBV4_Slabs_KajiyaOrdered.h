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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef GU_BV4_SLABS_KAJIYA_ORDERED_H
#define GU_BV4_SLABS_KAJIYA_ORDERED_H

#include "GuBVConstants.h"

#ifdef REMOVED
	// Kajiya + PNS
	template<const int inflateT, class LeafTestT, class ParamsT>
	static void BV4_ProcessStreamKajiyaOrdered(const BVDataPacked* PX_RESTRICT node, PxU32 initData, ParamsT* PX_RESTRICT params)
	{
		const BVDataPacked* root = node;

		PxU32 nb=1;
		PxU32 stack[GU_BV4_STACK_SIZE];
		stack[0] = initData;

#ifdef BV4_SLABS_SORT
		const PxU32* tmp = reinterpret_cast<const PxU32*>(&params->mLocalDir_Padded);
		const PxU32 X = tmp[0]>>31;
		const PxU32 Y = tmp[1]>>31;
		const PxU32 Z = tmp[2]>>31;
//		const PxU32 X = PX_IR(params->mLocalDir_Padded.x)>>31;
//		const PxU32 Y = PX_IR(params->mLocalDir_Padded.y)>>31;
//		const PxU32 Z = PX_IR(params->mLocalDir_Padded.z)>>31;
		const PxU32 bitIndex = 3+(Z|(Y<<1)|(X<<2));
		const PxU32 dirMask = 1u<<bitIndex;
#endif

#ifdef BV4_SLABS_FIX
		BV4_ALIGN16(float distances4[4]);
#endif
		///

		Vec4V fattenAABBsX, fattenAABBsY, fattenAABBsZ;
		if(inflateT)
		{
			Vec4V fattenAABBs4 = V4LoadU_Safe(&params->mOriginalExtents_Padded.x);
			fattenAABBs4 = V4Add(fattenAABBs4, epsInflateFloat4);	// US2385 - shapes are "closed" meaning exactly touching shapes should report overlap
			fattenAABBsX = V4SplatElement<0>(fattenAABBs4);
			fattenAABBsY = V4SplatElement<1>(fattenAABBs4);
			fattenAABBsZ = V4SplatElement<2>(fattenAABBs4);
		}

		///

		SLABS_INIT

#ifdef GU_BV4_QUANTIZED_TREE
		const Vec4V minCoeffV = V4LoadA_Safe(&params->mCenterOrMinCoeff_PaddedAligned.x);
		const Vec4V maxCoeffV = V4LoadA_Safe(&params->mExtentsOrMaxCoeff_PaddedAligned.x);
		const Vec4V minCoeffxV = V4SplatElement<0>(minCoeffV);
		const Vec4V minCoeffyV = V4SplatElement<1>(minCoeffV);
		const Vec4V minCoeffzV = V4SplatElement<2>(minCoeffV);
		const Vec4V maxCoeffxV = V4SplatElement<0>(maxCoeffV);
		const Vec4V maxCoeffyV = V4SplatElement<1>(maxCoeffV);
		const Vec4V maxCoeffzV = V4SplatElement<2>(maxCoeffV);
#endif

		do
		{
			const PxU32 childData = stack[--nb];
			node = root + getChildOffset(childData);

			const BVDataSwizzled* tn = reinterpret_cast<const BVDataSwizzled*>(node);

#ifdef GU_BV4_QUANTIZED_TREE
			Vec4V minx4a;
			Vec4V maxx4a;
			OPC_DEQ4(maxx4a, minx4a, mX, minCoeffxV, maxCoeffxV)

			Vec4V miny4a;
			Vec4V maxy4a;
			OPC_DEQ4(maxy4a, miny4a, mY, minCoeffyV, maxCoeffyV)

			Vec4V minz4a;
			Vec4V maxz4a;
			OPC_DEQ4(maxz4a, minz4a, mZ, minCoeffzV, maxCoeffzV)
#else
			Vec4V minx4a = V4LoadA(tn->mMinX);
			Vec4V miny4a = V4LoadA(tn->mMinY);
			Vec4V minz4a = V4LoadA(tn->mMinZ);

			Vec4V maxx4a = V4LoadA(tn->mMaxX);
			Vec4V maxy4a = V4LoadA(tn->mMaxY);
			Vec4V maxz4a = V4LoadA(tn->mMaxZ);
#endif
			if(inflateT)
			{
				maxx4a = V4Add(maxx4a, fattenAABBsX); maxy4a = V4Add(maxy4a, fattenAABBsY); maxz4a = V4Add(maxz4a, fattenAABBsZ);
				minx4a = V4Sub(minx4a, fattenAABBsX); miny4a = V4Sub(miny4a, fattenAABBsY); minz4a = V4Sub(minz4a, fattenAABBsZ);
			}

			SLABS_TEST

#ifdef BV4_SLABS_FIX
			if(inflateT)
				V4StoreA(maxOfNeasa, &distances4[0]);
#endif

			SLABS_TEST2

#ifdef BV4_SLABS_SORT
	#ifdef BV4_SLABS_FIX
		// PT: for some unknown reason the Linux/OSX compilers fail to understand this version
/*		#define DO_LEAF_TEST(x)														\
			{																		\
				if(!inflateT)														\
				{																	\
					if(tn->isLeaf(x))												\
					{																\
						LeafTestT::doLeafTest(params, tn->getPrimitive(x));			\
						maxT4 = V4Load(params->mStabbedFace.mDistance);				\
					}																\
					else															\
					{																\
						code2 |= 1<<x;												\
					}																\
				}																	\
				else																\
				{																	\
					if(distances4[x]<params->mStabbedFace.mDistance)				\
					{																\
						if(tn->isLeaf(x))											\
						{															\
							LeafTestT::doLeafTest(params, tn->getPrimitive(x));		\
							maxT4 = V4Load(params->mStabbedFace.mDistance);			\
						}															\
						else														\
						{															\
							code2 |= 1<<x;											\
						}															\
					}																\
				}																	\
			}*/

		// PT: TODO: check that this version compiles to the same code as above. Redo benchmarks.
		#define DO_LEAF_TEST(x)														\
			{																		\
				if(!inflateT || distances4[x]<params->mStabbedFace.mDistance + GU_EPSILON_SAME_DISTANCE)	\
				{																	\
					if(tn->isLeaf(x))												\
					{																\
						LeafTestT::doLeafTest(params, tn->getPrimitive(x));			\
						maxT4 = V4Load(params->mStabbedFace.mDistance);				\
					}																\
					else															\
					{																\
						code2 |= 1<<x;												\
					}																\
				}																	\
			}

	#else
		#define DO_LEAF_TEST(x)														\
				{																	\
					if(tn->isLeaf(x))												\
					{																\
						LeafTestT::doLeafTest(params, tn->getPrimitive(x));			\
						maxT4 = V4Load(params->mStabbedFace.mDistance);				\
					}																\
					else															\
					{																\
						code2 |= 1<<x;												\
					}																\
				}
	#endif
			PxU32 code2 = 0;
			const PxU32 nodeType = getChildType(childData);

			if(!(code&8) && nodeType>1)
				DO_LEAF_TEST(3)

			if(!(code&4) && nodeType>0)
				DO_LEAF_TEST(2)

			if(!(code&2))
				DO_LEAF_TEST(1)

			if(!(code&1))
				DO_LEAF_TEST(0)

			SLABS_PNS
#else
	#define DO_LEAF_TEST(x)														\
					{if(tn->isLeaf(x))											\
					{															\
						LeafTestT::doLeafTest(params, tn->getPrimitive(x));		\
						maxT4 = V4Load(params->mStabbedFace.mDistance);			\
					}															\
					else														\
					{															\
						stack[nb++] = tn->getChildData(x);						\
					}}


				const PxU32 nodeType = getChildType(childData);
				if(!(code&8) && nodeType>1)
					DO_LEAF_TEST(3)

				if(!(code&4) && nodeType>0)
					DO_LEAF_TEST(2)

				if(!(code&2))
					DO_LEAF_TEST(1)

				if(!(code&1))
					DO_LEAF_TEST(0)
#endif

		}while(nb);
	}
#undef DO_LEAF_TEST
#endif



#ifdef BV4_SLABS_SORT
	#ifdef BV4_SLABS_FIX
		// PT: for some unknown reason the Linux/OSX compilers fail to understand this version
/*		#define DO_LEAF_TEST(x)														\
			{																		\
				if(!inflateT)														\
				{																	\
					if(tn->isLeaf(x))												\
					{																\
						LeafTestT::doLeafTest(params, tn->getPrimitive(x));			\
						maxT4 = V4Load(params->mStabbedFace.mDistance);				\
					}																\
					else															\
					{																\
						code2 |= 1<<x;												\
					}																\
				}																	\
				else																\
				{																	\
					if(distances4[x]<params->mStabbedFace.mDistance)				\
					{																\
						if(tn->isLeaf(x))											\
						{															\
							LeafTestT::doLeafTest(params, tn->getPrimitive(x));		\
							maxT4 = V4Load(params->mStabbedFace.mDistance);			\
						}															\
						else														\
						{															\
							code2 |= 1<<x;											\
						}															\
					}																\
				}																	\
			}*/

		// PT: TODO: check that this version compiles to the same code as above. Redo benchmarks.
		#define DO_LEAF_TEST(x)														\
			{																		\
				if(!inflateT || distances4[x]<params->mStabbedFace.mDistance + GU_EPSILON_SAME_DISTANCE)	\
				{																	\
					if(tn->isLeaf(x))												\
					{																\
						LeafTestT::doLeafTest(params, tn->getPrimitive(x));			\
						maxT4 = V4Load(params->mStabbedFace.mDistance);				\
					}																\
					else															\
					{																\
						code2 |= 1<<x;	nbHits++;									\
					}																\
				}																	\
			}

	#else
		#define DO_LEAF_TEST(x)														\
				{																	\
					if(tn->isLeaf(x))												\
					{																\
						LeafTestT::doLeafTest(params, tn->getPrimitive(x));			\
						maxT4 = V4Load(params->mStabbedFace.mDistance);				\
					}																\
					else															\
					{																\
						code2 |= 1<<x;	nbHits++;									\
					}																\
				}
	#endif
#else
	#define DO_LEAF_TEST(x)														\
					{if(tn->isLeaf(x))											\
					{															\
						LeafTestT::doLeafTest(params, tn->getPrimitive(x));		\
						maxT4 = V4Load(params->mStabbedFace.mDistance);			\
					}															\
					else														\
					{															\
						stack[nb++] = tn->getChildData(x);						\
					}}
#endif

	// Kajiya + PNS
	template<const int inflateT, class LeafTestT, class ParamsT>
	static void BV4_ProcessStreamKajiyaOrderedQ(const BVDataPackedQ* PX_RESTRICT node, PxU32 initData, ParamsT* PX_RESTRICT params)
	{
		const BVDataPackedQ* root = node;

		PxU32 nb=1;
		PxU32 stack[GU_BV4_STACK_SIZE];
		stack[0] = initData;

#ifdef BV4_SLABS_SORT
		const PxU32* tmp = reinterpret_cast<const PxU32*>(&params->mLocalDir_Padded);
		const PxU32 X = tmp[0]>>31;
		const PxU32 Y = tmp[1]>>31;
		const PxU32 Z = tmp[2]>>31;
//		const PxU32 X = PX_IR(params->mLocalDir_Padded.x)>>31;
//		const PxU32 Y = PX_IR(params->mLocalDir_Padded.y)>>31;
//		const PxU32 Z = PX_IR(params->mLocalDir_Padded.z)>>31;
		const PxU32 bitIndex = 3+(Z|(Y<<1)|(X<<2));
		const PxU32 dirMask = 1u<<bitIndex;
#endif

#ifdef BV4_SLABS_FIX
		BV4_ALIGN16(float distances4[4]);
#endif
		///

		Vec4V fattenAABBsX, fattenAABBsY, fattenAABBsZ;
		if(inflateT)
		{
			Vec4V fattenAABBs4 = V4LoadU_Safe(&params->mOriginalExtents_Padded.x);
			fattenAABBs4 = V4Add(fattenAABBs4, epsInflateFloat4);	// US2385 - shapes are "closed" meaning exactly touching shapes should report overlap
			fattenAABBsX = V4SplatElement<0>(fattenAABBs4);
			fattenAABBsY = V4SplatElement<1>(fattenAABBs4);
			fattenAABBsZ = V4SplatElement<2>(fattenAABBs4);
		}

		///

		SLABS_INIT

		const Vec4V minCoeffV = V4LoadA_Safe(&params->mCenterOrMinCoeff_PaddedAligned.x);
		const Vec4V maxCoeffV = V4LoadA_Safe(&params->mExtentsOrMaxCoeff_PaddedAligned.x);
		const Vec4V minCoeffxV = V4SplatElement<0>(minCoeffV);
		const Vec4V minCoeffyV = V4SplatElement<1>(minCoeffV);
		const Vec4V minCoeffzV = V4SplatElement<2>(minCoeffV);
		const Vec4V maxCoeffxV = V4SplatElement<0>(maxCoeffV);
		const Vec4V maxCoeffyV = V4SplatElement<1>(maxCoeffV);
		const Vec4V maxCoeffzV = V4SplatElement<2>(maxCoeffV);

		do
		{
			const PxU32 childData = stack[--nb];
			node = root + getChildOffset(childData);

			const BVDataSwizzledQ* tn = reinterpret_cast<const BVDataSwizzledQ*>(node);

			Vec4V minx4a;
			Vec4V maxx4a;
			OPC_DEQ4(maxx4a, minx4a, mX, minCoeffxV, maxCoeffxV)

			Vec4V miny4a;
			Vec4V maxy4a;
			OPC_DEQ4(maxy4a, miny4a, mY, minCoeffyV, maxCoeffyV)

			Vec4V minz4a;
			Vec4V maxz4a;
			OPC_DEQ4(maxz4a, minz4a, mZ, minCoeffzV, maxCoeffzV)

			if(inflateT)
			{
				maxx4a = V4Add(maxx4a, fattenAABBsX); maxy4a = V4Add(maxy4a, fattenAABBsY); maxz4a = V4Add(maxz4a, fattenAABBsZ);
				minx4a = V4Sub(minx4a, fattenAABBsX); miny4a = V4Sub(miny4a, fattenAABBsY); minz4a = V4Sub(minz4a, fattenAABBsZ);
			}

			SLABS_TEST

#ifdef BV4_SLABS_FIX
			if(inflateT)				
				V4StoreA(maxOfNeasa, &distances4[0]);
#endif

			SLABS_TEST2

#ifdef BV4_SLABS_SORT
			PxU32 code2 = 0;
			PxU32 nbHits = 0;
			const PxU32 nodeType = getChildType(childData);

			if(!(code&8) && nodeType>1)
				DO_LEAF_TEST(3)

			if(!(code&4) && nodeType>0)
				DO_LEAF_TEST(2)

			if(!(code&2))
				DO_LEAF_TEST(1)

			if(!(code&1))
				DO_LEAF_TEST(0)

			//SLABS_PNS

			if(nbHits==1)
			{
				PNS_BLOCK3(0,1,2,3)
			}
			else
			{
				SLABS_PNS
			}
#else
			const PxU32 nodeType = getChildType(childData);
			if(!(code&8) && nodeType>1)
				DO_LEAF_TEST(3)

			if(!(code&4) && nodeType>0)
				DO_LEAF_TEST(2)

			if(!(code&2))
				DO_LEAF_TEST(1)

			if(!(code&1))
				DO_LEAF_TEST(0)
#endif

		}while(nb);
	}


	// Kajiya + PNS
	template<const int inflateT, class LeafTestT, class ParamsT>
	static void BV4_ProcessStreamKajiyaOrderedNQ(const BVDataPackedNQ* PX_RESTRICT node, PxU32 initData, ParamsT* PX_RESTRICT params)
	{
		const BVDataPackedNQ* root = node;

		PxU32 nb=1;
		PxU32 stack[GU_BV4_STACK_SIZE];
		stack[0] = initData;

#ifdef BV4_SLABS_SORT
		const PxU32* tmp = reinterpret_cast<const PxU32*>(&params->mLocalDir_Padded);
		const PxU32 X = tmp[0]>>31;
		const PxU32 Y = tmp[1]>>31;
		const PxU32 Z = tmp[2]>>31;
//		const PxU32 X = PX_IR(params->mLocalDir_Padded.x)>>31;
//		const PxU32 Y = PX_IR(params->mLocalDir_Padded.y)>>31;
//		const PxU32 Z = PX_IR(params->mLocalDir_Padded.z)>>31;
		const PxU32 bitIndex = 3+(Z|(Y<<1)|(X<<2));
		const PxU32 dirMask = 1u<<bitIndex;
#endif

#ifdef BV4_SLABS_FIX
		BV4_ALIGN16(float distances4[4]);
#endif
		///

		Vec4V fattenAABBsX, fattenAABBsY, fattenAABBsZ;
		if(inflateT)
		{
			Vec4V fattenAABBs4 = V4LoadU_Safe(&params->mOriginalExtents_Padded.x);
			fattenAABBs4 = V4Add(fattenAABBs4, epsInflateFloat4);	// US2385 - shapes are "closed" meaning exactly touching shapes should report overlap
			fattenAABBsX = V4SplatElement<0>(fattenAABBs4);
			fattenAABBsY = V4SplatElement<1>(fattenAABBs4);
			fattenAABBsZ = V4SplatElement<2>(fattenAABBs4);
		}

		///

		SLABS_INIT

		do
		{
			const PxU32 childData = stack[--nb];
			node = root + getChildOffset(childData);

			const BVDataSwizzledNQ* tn = reinterpret_cast<const BVDataSwizzledNQ*>(node);

			Vec4V minx4a = V4LoadA(tn->mMinX);
			Vec4V miny4a = V4LoadA(tn->mMinY);
			Vec4V minz4a = V4LoadA(tn->mMinZ);

			Vec4V maxx4a = V4LoadA(tn->mMaxX);
			Vec4V maxy4a = V4LoadA(tn->mMaxY);
			Vec4V maxz4a = V4LoadA(tn->mMaxZ);

			if(inflateT)
			{
				maxx4a = V4Add(maxx4a, fattenAABBsX); maxy4a = V4Add(maxy4a, fattenAABBsY); maxz4a = V4Add(maxz4a, fattenAABBsZ);
				minx4a = V4Sub(minx4a, fattenAABBsX); miny4a = V4Sub(miny4a, fattenAABBsY); minz4a = V4Sub(minz4a, fattenAABBsZ);
			}

			SLABS_TEST

#ifdef BV4_SLABS_FIX
			if(inflateT)				
				V4StoreA(maxOfNeasa, &distances4[0]);
#endif

			SLABS_TEST2

#ifdef BV4_SLABS_SORT
			PxU32 code2 = 0;
			PxU32 nbHits = 0;
			const PxU32 nodeType = getChildType(childData);

			if(!(code&8) && nodeType>1)
				DO_LEAF_TEST(3)

			if(!(code&4) && nodeType>0)
				DO_LEAF_TEST(2)

			if(!(code&2))
				DO_LEAF_TEST(1)

			if(!(code&1))
				DO_LEAF_TEST(0)

			//SLABS_PNS
			if(nbHits==1)
			{
				PNS_BLOCK3(0,1,2,3)
			}
			else
			{
				SLABS_PNS
			}
#else
			const PxU32 nodeType = getChildType(childData);
			if(!(code&8) && nodeType>1)
				DO_LEAF_TEST(3)

			if(!(code&4) && nodeType>0)
				DO_LEAF_TEST(2)

			if(!(code&2))
				DO_LEAF_TEST(1)

			if(!(code&1))
				DO_LEAF_TEST(0)
#endif

		}while(nb);
	}

#undef DO_LEAF_TEST

#endif // GU_BV4_SLABS_KAJIYA_ORDERED_H
