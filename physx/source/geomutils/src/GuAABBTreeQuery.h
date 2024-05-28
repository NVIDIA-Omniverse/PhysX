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

#ifndef GU_AABBTREEQUERY_H
#define GU_AABBTREEQUERY_H

#include "GuBVHTestsSIMD.h"
#include "GuAABBTreeBounds.h"
#include "foundation/PxInlineArray.h"
#include "GuAABBTreeNode.h"

namespace physx
{
	namespace Gu
	{
#define RAW_TRAVERSAL_STACK_SIZE 256

		//////////////////////////////////////////////////////////////////////////

		static PX_FORCE_INLINE void getBoundsTimesTwo(Vec4V& center, Vec4V& extents, const PxBounds3* bounds, PxU32 poolIndex)
		{
			const PxBounds3* objectBounds = bounds + poolIndex;

			// PT: it's safe to V4LoadU because the pointer comes from the AABBTreeBounds class
			const Vec4V minV = V4LoadU(&objectBounds->minimum.x);
			const Vec4V maxV = V4LoadU(&objectBounds->maximum.x);

			center = V4Add(maxV, minV);
			extents = V4Sub(maxV, minV);
		}

		//////////////////////////////////////////////////////////////////////////

		template<const bool tHasIndices, typename Test, typename Node, typename QueryCallback>
		static PX_FORCE_INLINE bool doOverlapLeafTest(const Test& test, const Node* node, const PxBounds3* bounds, const PxU32* indices, QueryCallback& visitor)
		{
			PxU32 nbPrims = node->getNbPrimitives();
			const bool doBoxTest = nbPrims > 1;
			const PxU32* prims = tHasIndices ? node->getPrimitives(indices) : NULL;
			while(nbPrims--)
			{
				const PxU32 primIndex = tHasIndices ? *prims++ : node->getPrimitiveIndex();
				if(doBoxTest)
				{
					Vec4V center2, extents2;
					getBoundsTimesTwo(center2, extents2, bounds, primIndex);

					const float half = 0.5f;
					const FloatV halfV = FLoad(half);

					const Vec4V extents_ = V4Scale(extents2, halfV);
					const Vec4V center_ = V4Scale(center2, halfV);

					if(!test(Vec3V_From_Vec4V(center_), Vec3V_From_Vec4V(extents_)))
						continue;
				}

				if(!visitor.invoke(primIndex))
					return false;
			}
			return true;
		}

		template<const bool tHasIndices, typename Test, typename Tree, typename Node, typename QueryCallback>
		class AABBTreeOverlap
		{
		public:
			bool operator()(const AABBTreeBounds& treeBounds, const Tree& tree, const Test& test, QueryCallback& visitor)
			{
				const PxBounds3* bounds = treeBounds.getBounds();

				PxInlineArray<const Node*, RAW_TRAVERSAL_STACK_SIZE> stack;
				stack.forceSize_Unsafe(RAW_TRAVERSAL_STACK_SIZE);
				const Node* const nodeBase = tree.getNodes();
				stack[0] = nodeBase;
				PxU32 stackIndex = 1;

				while(stackIndex > 0)
				{
					const Node* node = stack[--stackIndex];
					Vec3V center, extents;
					node->getAABBCenterExtentsV(&center, &extents);
					while(test(center, extents))
					{
						if(node->isLeaf())
						{
							if(!doOverlapLeafTest<tHasIndices, Test, Node>(test, node, bounds, tree.getIndices(), visitor))
								return false;
							break;
						}

						const Node* children = node->getPos(nodeBase);

						node = children;
						stack[stackIndex++] = children + 1;
						if(stackIndex == stack.capacity())
							stack.resizeUninitialized(stack.capacity() * 2);
						node->getAABBCenterExtentsV(&center, &extents);
					}
				}
				return true;
			}
		};

		//////////////////////////////////////////////////////////////////////////

		template <const bool tInflate, const bool tHasIndices, typename Node, typename QueryCallback> // use inflate=true for sweeps, inflate=false for raycasts
		static PX_FORCE_INLINE bool doLeafTest(	const Node* node, Gu::RayAABBTest& test, const PxBounds3* bounds, const PxU32* indices, PxReal& maxDist, QueryCallback& pcb)
		{
			PxU32 nbPrims = node->getNbPrimitives();
			const bool doBoxTest = nbPrims > 1;
			const PxU32* prims = tHasIndices ? node->getPrimitives(indices) : NULL;
			while(nbPrims--)
			{
				const PxU32 primIndex = tHasIndices ? *prims++ : node->getPrimitiveIndex();
				if(doBoxTest)
				{
					Vec4V center_, extents_;
					getBoundsTimesTwo(center_, extents_, bounds, primIndex);

					if(!test.check<tInflate>(Vec3V_From_Vec4V(center_), Vec3V_From_Vec4V(extents_)))
						continue;
				}

				// PT:
				// - 'maxDist' is the current best distance. It can be seen as a "maximum allowed distance" (as passed to the
				//   template by users initially) but also as the "current minimum impact distance", so the name is misleading.
				//   Either way this is where we write & communicate the final/best impact distance to users.
				//
				// - the invoke function also takes a distance parameter, and this one is in/out. In input we must pass the
				//   current best distance to the leaf node, so that subsequent leaf-level queries can cull things away as
				//   much as possible. In output users return a shrunk distance value if they found a hit. We need to pass a
				//   copy of 'maxDist' ('md') since it would be too dangerous to rely on the arbitrary user code to always do
				//   the right thing. In particular if we'd pass 'maxDist' to invoke directly, and the called code would NOT
				//   respect the passed max value, it could potentially return a hit further than the best 'maxDist'. At which
				//   point the '(md < oldMaxDist)' test would fail but the damage would have already been done ('maxDist' would
				//   have already been overwritten with a larger value than before). Hence, we need 'md'.
				//
				// - now 'oldMaxDist' however is more subtle. In theory we wouldn't need it and we could just use '(md < maxDist)'
				//   in the test below. But that opens the door to subtle bugs: 'maxDist' is a reference to some value somewhere
				//   in the user's code, and we call the same user in invoke. It turns out that the invoke code can access and
				//   modify 'maxDist' on their side, even if we do not pass it to invoke. It's basically the same problem as
				//   before, but much more difficult to see. It does happen with the current PhysX implementations of the invoke
				//   functions: they modify the 'md' that we send them, but *also* 'maxDist' without the code below knowing
				//   about it. So the subsequent test fails again because md == maxDist. A potential solution would have been to
				//   work on a local copy of 'maxDist' in operator(), only writing out the final distance when returning from the
				//   function. Another solution used below is to introduce that local copy just here in the leaf code: that's
				//   where 'oldMaxDist' comes from.

				PxReal oldMaxDist = maxDist;
				PxReal md = maxDist;
				if(!pcb.invoke(md, primIndex))
					return false;

				if(md < oldMaxDist)
				{
					maxDist = md;
					test.setDistance(md);
				}
			}
			return true;
		}

		//////////////////////////////////////////////////////////////////////////

		template <const bool tInflate, const bool tHasIndices, typename Tree, typename Node, typename QueryCallback> // use inflate=true for sweeps, inflate=false for raycasts
		class AABBTreeRaycast
		{
		public:
			bool operator()(
				const AABBTreeBounds& treeBounds, const Tree& tree,
				const PxVec3& origin, const PxVec3& unitDir, PxReal& maxDist, const PxVec3& inflation,
				QueryCallback& pcb)
			{
				const PxBounds3* bounds = treeBounds.getBounds();

				// PT: we will pass center*2 and extents*2 to the ray-box code, to save some work per-box
				// So we initialize the test with values multiplied by 2 as well, to get correct results
				Gu::RayAABBTest test(origin*2.0f, unitDir*2.0f, maxDist, inflation*2.0f);

				PxInlineArray<const Node*, RAW_TRAVERSAL_STACK_SIZE> stack;
				stack.forceSize_Unsafe(RAW_TRAVERSAL_STACK_SIZE);
				const Node* const nodeBase = tree.getNodes();
				stack[0] = nodeBase;
				PxU32 stackIndex = 1;

				while(stackIndex--)
				{
					const Node* node = stack[stackIndex];
					Vec3V center, extents;
					node->getAABBCenterExtentsV2(&center, &extents);
					if(test.check<tInflate>(center, extents))	// TODO: try timestamp ray shortening to skip this
					{
						while(!node->isLeaf())
						{
							const Node* children = node->getPos(nodeBase);

							Vec3V c0, e0;
							children[0].getAABBCenterExtentsV2(&c0, &e0);
							const PxU32 b0 = test.check<tInflate>(c0, e0);

							Vec3V c1, e1;
							children[1].getAABBCenterExtentsV2(&c1, &e1);
							const PxU32 b1 = test.check<tInflate>(c1, e1);

							if(b0 && b1)	// if both intersect, push the one with the further center on the stack for later
							{
								// & 1 because FAllGrtr behavior differs across platforms
								const PxU32 bit = FAllGrtr(V3Dot(V3Sub(c1, c0), test.mDir), FZero()) & 1;
								stack[stackIndex++] = children + bit;
								node = children + (1 - bit);
								if(stackIndex == stack.capacity())
									stack.resizeUninitialized(stack.capacity() * 2);
							}
							else if(b0)
								node = children;
							else if(b1)
								node = children + 1;
							else
								goto skip_leaf_code;
						}

						if(!doLeafTest<tInflate, tHasIndices, Node>(node, test, bounds, tree.getIndices(), maxDist, pcb))
							return false;
					skip_leaf_code:;
					}
				}
				return true;
			}
		};


		struct TraversalControl
		{
			enum Enum {
				eDontGoDeeper,
				eGoDeeper,
				eGoDeeperNegFirst,
				eAbort
			};
		};

		template<typename T>
		void traverseBVH(const Gu::BVHNode* nodes, T& traversalController, PxI32 rootNodeIndex = 0)
		{
			PxI32 index = rootNodeIndex;

			PxInlineArray<PxI32, RAW_TRAVERSAL_STACK_SIZE> todoStack;			

			while (true)
			{
				const Gu::BVHNode& a = nodes[index];

				TraversalControl::Enum control = traversalController.analyze(a, index);
				if (control == TraversalControl::eAbort)
					return;
				if (!a.isLeaf() && (control == TraversalControl::eGoDeeper || control == TraversalControl::eGoDeeperNegFirst))
				{
					if (control == TraversalControl::eGoDeeperNegFirst)
					{
						todoStack.pushBack(a.getPosIndex());
						index = a.getNegIndex(); //index gets processed next - assign negative index to it				
					}
					else
					{
						todoStack.pushBack(a.getNegIndex());
						index = a.getPosIndex(); //index gets processed next - assign positive index to it	
					}
					continue;
				}
				if (todoStack.empty()) break;
				index = todoStack.popBack();
			}
		}
	}
}

#endif   // SQ_AABBTREEQUERY_H
