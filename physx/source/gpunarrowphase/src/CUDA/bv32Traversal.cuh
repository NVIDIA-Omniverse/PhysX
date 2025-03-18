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

#ifndef __BV32_TRAVERSAL_CUH__
#define __BV32_TRAVERSAL_CUH__

#include "copy.cuh"
#include "reduction.cuh"

//Not used but handy when creating a new type of tree traversal
struct BV32TraversalTemplate
{
	PX_FORCE_INLINE __device__ BV32TraversalTemplate()
	{

	}

	//Called on leaf nodes to process them
	//A full warp will call the function. If primitiveIndex is 0xFFFFFFFF, then the thread has no leaf node to check.
	PX_FORCE_INLINE __device__ void intersectPrimitiveFullWarp(PxU32 primitiveIndex, PxU32 idxInWarp) const
	{
		
	}

	//Called to determine if the traversal should continue or stop on a branch node
	//A full warp will call the function. If hasBox is false, then the thread has no branch node to check. 
	PX_FORCE_INLINE __device__ bool intersectBoxFullWarp(bool hasBox, const PxVec3& min, const PxVec3& max) const
	{
		if (hasBox)
			return true; //Actually test against the axis aligned box defined by min and max
		return false;
	}
};

// TW: On my laptop RTX 3080 the FPS of some FEM cloth VTs actually drops by about 10% when enabling optimized traversal...
#define OPTIMIZE_TRAVERSAL 0

template<typename T, unsigned int WarpsPerBlock>
__device__ static void bv32TreeTraversal(
	const physx::Gu::BV32DataPacked* bv32PackedNodes,
	int* sBv32Nodes,
	T& callback
)
{
#if OPTIMIZE_TRAVERSAL
	__shared__ Gu::BV32DataPacked shNode[WarpsPerBlock];
	const unsigned int warpId = threadIdx.y;
#endif

	//thread index in warp
	const unsigned int idxInWarp = threadIdx.x;

	PxU32 currentNodeIndex = 0; // start at the root
	const Gu::BV32DataPacked* PX_RESTRICT currentNode = &bv32PackedNodes[0];

	PxU32 nbStack = 0; // number of nodes on the stack of to-be-expanded nodes

	while (currentNode)
	{
#if OPTIMIZE_TRAVERSAL
		//This optimization was found in cudaParticleSystem.cu, line 2373
		// VR: This variant works ~6 times *faster* on my GTX 1080
		Gu::BV32DataPacked& node = shNode[warpId];
		warpCopy((uint*)&node, (uint*)currentNode, sizeof(Gu::BV32DataPacked));
		__syncwarp();
#else
		// VR: This variant works ~6 times *slower* on my GTX 1080
		const Gu::BV32DataPacked& node = *currentNode;
#endif

		__syncwarp(); //sBv32Nodes (shared memory) is read and written in the same loop - read and write must be separated with a sync

		//check to see whether this is a leaf or not
		if (currentNodeIndex & 1)
		{
			const PxU32 leafOffset = (currentNodeIndex >> 1) & 31;
			//at max 32 primitives at a leaf node 
			const PxU32 nbPrimitives = node.getNbReferencedPrimitives(leafOffset);
			const PxU32 primStartIndex = node.getPrimitiveStartIndex(leafOffset);

			// now every node checks a primitive (triangle, tetrahedron etc) against the query geometry
			// result variables:
			PxU32 primitiveIdx = 0xffffffff;

			if (idxInWarp < nbPrimitives)
			{
				primitiveIdx = idxInWarp + primStartIndex;
			}
			callback.intersectPrimitiveFullWarp(primitiveIdx, idxInWarp);
		}
		else // not a leave node
		{
			const PxU32 nbChildren = node.mNbNodes;

			// there are max 32 children nodes. Every thread checks whether its child's bounding box
			// would intersect with the query object (which can be a box, a ray or anything)

			PxVec3 min, max;
			bool hasBox = idxInWarp < nbChildren;
			if (hasBox)
			{
				min = PxLoad3(reinterpret_cast<const float4&>(node.mMin[idxInWarp]));
				max = PxLoad3(reinterpret_cast<const float4&>(node.mMax[idxInWarp]));				
			}
			bool intersect = callback.intersectBoxFullWarp(hasBox, min, max);
			//if (!hasBox)
			//	intersect = false;
			/*if (workIndex == 0 )
			{
			printf("newActiveParticles %x idx %i Finished\n", newActiveParticles, idxInWarp);
			}*/

			// ML: the first one bit is to decide whether the node is a leaf or not.The subsequence 5 bits is used to
			//store the row index of mData so that if the node is leaf we can get the the reference triangle. The subsequence 26 bits
			//is used to store either the next node index if the node isn't a leaf or store the parent node index if the node is leaf node.

			//Masking currentNodeIndex with 0xFFFFFFC0 clears the 6 lowest bits, so we can mask in our idxInWarp<<1 | 1 for leaf masks
			PxU32 currentIndex = ((currentNodeIndex) & 0xFFFFFFC0) | (idxInWarp << 1) | 1;

			//create node bitmap 
			bool isValidNode = false;
			if (idxInWarp < nbChildren)
			{
				const PxU32 childOffset = node.getChildOffset(idxInWarp);

				isValidNode = intersect;

				if (isValidNode && !node.isLeaf(idxInWarp))
				{
					currentIndex = (childOffset) << 6;
				}
			}

			PxU32 resultWarp = __ballot_sync(FULL_MASK, isValidNode);
			/*if (idxInWarp == 0)
			{
			printf("ResultWarp = %x\n", resultWarp);
			}*/

			//PxU32 resultWarp = __ballot_sync(FULL_MASK, intersect);
			PxU32 offset = warpScanExclusive(resultWarp, idxInWarp);
			PxU32 validCount = __popc(resultWarp);

			if (isValidNode)
			{
				//printf("Push Bv32Nodes[%i] = %i mask = %x\n", nbStack + offset, currentIndex, newActiveParticles);
				sBv32Nodes[nbStack + offset] = currentIndex;
			}

			nbStack += validCount;
		}

		__syncwarp();

		// select the next node from the stack, same for all warps
		if (nbStack > 0)
		{
			currentNodeIndex = sBv32Nodes[--nbStack];
			currentNode = &bv32PackedNodes[currentNodeIndex >> 6];

			/*if (idxInWarp == 0)
			{
				printf("Pop CurrentIndex = %i, currentNode = %i, currentActiveParticles = %x\n", currentNodeIndex, currentNodeIndex >> 6, currentNodeActiveParticles);
			}*/
		}

		else
			currentNode = NULL;
	}
}

PX_FORCE_INLINE __device__ void pushOntoStackFullWarp(bool threadPushesElement, PxU32* atomicCounter, uint4* PX_RESTRICT stackPtr, PxU32 stackSize, const uint4& stackItemToPush)
{
	PxU32 stackIndex = globalScanExclusiveSingleWarp(threadPushesElement, atomicCounter);

	// Each thread stores its contact. If we run out of stacksize, drop contacts
	if (threadPushesElement && (stackIndex < stackSize))
	{
		stackPtr[stackIndex] = stackItemToPush;
	}
}

#endif