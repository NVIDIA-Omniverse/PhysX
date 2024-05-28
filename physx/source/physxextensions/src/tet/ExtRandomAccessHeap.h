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

#ifndef EXT_RANDOM_ACCESS_HEAP_H
#define EXT_RANDOM_ACCESS_HEAP_H

#include "foundation/PxArray.h"

// MM: heap which allows the modification of the priorities of entries stored anywhere in the tree
// for this, every entry gets an id via which it can be accessed

// used in ExtMeshSimplificator to sort edges w.r.t. their error metric

// ------------------------------------------------------------------------------

namespace physx
{
	namespace Ext
	{

		template <class T> class RandomAccessHeap 
		{
		public:
			RandomAccessHeap() {
				heap.resize(1);		// dummy such that root is at 1 for faster parent child computation
				ids.resize(1);
				nextId = 0;
			}

			void resizeFast(PxArray<PxI32>& arr, PxU32 newSize, PxI32 value = 0)
			{
				if (newSize < arr.size())
					arr.removeRange(newSize, arr.size() - newSize);
				else
				{
					while (arr.size() < newSize)
						arr.pushBack(value);
				}
			}

			PxI32 insert(T elem, PxI32 id = -1)  // id for ability to alter the entry later or to identify duplicates
			{
				if (id < 0) {
					id = nextId;
					nextId++;
				}
				else if (id >= nextId)
					nextId = id + 1;

				if (id >= 0 && id < PxI32(posOfId.size()) && posOfId[id] >= 0)
					return id;		// already in heap

				heap.pushBack(elem);
				ids.pushBack(id);
				
				if (id >= PxI32(posOfId.size()))
					resizeFast(posOfId, id + 1, -1);

				posOfId[id] = heap.size() - 1;

				percolate(PxI32(heap.size()) - 1);

				return id;
			}

			bool remove(PxI32 id)
			{
				PxI32 i = posOfId[id];
				if (i < 0)
					return false;

				posOfId[id] = -1;
				T prev = heap[i];

				heap[i] = heap.back();
				heap.popBack();
				ids[i] = ids.back();
				ids.popBack();

				if (i < PxI32(heap.size())) 
				{
					posOfId[ids[i]] = i;

					if (heap.size() > 1) 
					{
						if (heap[i] < prev)
							percolate(i);
						else
							siftDown(i);
					}
				}

				return true;
			}

			T deleteMin()
			{
				T min(-1, -1, 0.0f);

				if (heap.size() > 1) 
				{
					min = heap[1];
					posOfId[ids[1]] = -1;

					heap[1] = heap.back();
					heap.popBack();
					ids[1] = ids.back();
					ids.popBack();
					posOfId[ids[1]] = 1;

					siftDown(1);
				}

				return min;
			}

			void makeHeap(const PxArray<T> &elems)	// O(n) instead of inserting one after the other O(n log n) 
			{
				heap.resize(elems.size() + 1);
				ids.resize(elems.size() + 1);
				posOfId.resize(elems.size());

				for (PxU32 i = 0; i < elems.size(); i++) 
				{
					heap[i + 1] = elems[i];
					ids[i + 1] = i;
					posOfId[ids[i + 1]] = i + 1;
				}

				PxI32 n = (heap.size() - 1) >> 1;
				for (PxI32 i = n; i >= 1; i--)
					siftDown(i);
			}

			void clear() 
			{
				heap.capacity() == 0 ? heap.resize(1) : heap.forceSize_Unsafe(1);
				ids.capacity() == 0 ? ids.resize(1) : ids.forceSize_Unsafe(1);
				posOfId.forceSize_Unsafe(0);
				nextId = 0;
			}
			PX_FORCE_INLINE PxI32 size() { return heap.size() - 1; }
			PX_FORCE_INLINE bool empty() { return heap.size() <= 1; }

		private:
			void siftDown(PxI32 i)
			{
				PxI32 n = PxI32(heap.size()) - 1;
				PxI32 k = i;
				PxI32 j;
				do 
				{
					j = k;
					if (2 * j < n && heap[2 * j] < heap[k])
						k = 2 * j;
					if (2 * j < n && heap[2 * j + 1] < heap[k])
						k = 2 * j + 1;
					T temp = heap[j]; heap[j] = heap[k]; heap[k] = temp;
					PxI32 id = ids[j]; ids[j] = ids[k]; ids[k] = id;

					posOfId[ids[j]] = j;
					posOfId[ids[k]] = k;

				} 
				while (j != k);
			}

			void percolate(PxI32 i)
			{
				PxI32 k = i;
				PxI32 j;
				do 
				{
					j = k;
					if (j > 1 && !(heap[j >> 1] < heap[k]))
						k = j >> 1;
					T temp = heap[j]; heap[j] = heap[k]; heap[k] = temp;
					PxI32 id = ids[j]; ids[j] = ids[k]; ids[k] = id;

					posOfId[ids[j]] = j;
					posOfId[ids[k]] = k;
				} 
				while (j != k);
			}

			PxArray<T> heap;

			PxArray<PxI32> ids;
			PxArray<PxI32> posOfId;
			PxI32 nextId;
		};
	}
}

#endif
