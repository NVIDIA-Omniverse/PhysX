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


#ifndef EXT_MULTI_LIST_H
#define EXT_MULTI_LIST_H

// MM: Multiple linked lists in a common array with a free list

#include "foundation/PxArray.h"

namespace physx
{
	namespace Ext
	{

		//-----------------------------------------------------------------------------
		template <class T>
		class MultiList {
		public:
			MultiList(PxI32 maxId = 0) {
				firstFree = -1;
				if (maxId > 0)
					first.reserve(maxId + 1);
			}
			void reserve(int maxId) {
				first.reserve(maxId + 1);
			}
			void clear();
			PxI32 add(PxI32 id, const T &item);
			bool addUnique(PxI32 id, const T &item);
			bool exists(PxI32 id, const T &item) const;
			void remove(PxI32 id, const T &item);
			void removeAll(PxI32 id);
			PxI32 size(PxI32 id) const;
			PxI32 getPairNr(PxI32 id, const T &item) const;

			void replace(PxI32 id, const T &before, const T &after);

			void getItems(PxI32 id) const;
			mutable PxArray<T> queryItems;

			void initIteration(PxI32 id, PxI32& iterator);
			bool iterate(T& item, PxI32& iterator);

			void getPointers(PxI32 id);
			mutable PxArray<T*> queryPointers;

		private:
			PxArray<PxI32> first;
			PxArray<T> items;
			PxArray<PxI32> next;
			PxI32 firstFree;
		};

		//-----------------------------------------------------------------------------
		template <class T>
		void MultiList<T>::clear()
		{
			first.clear();
			next.clear();
			items.clear();
			queryItems.clear();
			queryPointers.clear();
			firstFree = -1;
		}

		//-----------------------------------------------------------------------------
		template <class T>
		PxI32 MultiList<T>::add(PxI32 id, const T &item)
		{
			if (id >= PxI32(first.size()))
				first.resize(id + 1, -1);
			PxI32 pos = firstFree;
			if (pos >= 0)
				firstFree = next[firstFree];
			else 
			{
				pos = PxI32(items.size());
				items.resize(items.size() + 1);
				next.resize(items.size() + 1);
			}
			next[pos] = first[id];
			first[id] = pos;
			items[pos] = item;
			return pos;
		}

		//-----------------------------------------------------------------------------
		template <class T>
		bool MultiList<T>::addUnique(PxI32 id, const T &item)
		{
			if (exists(id, item))
				return false;
			add(id, item);
			return true;
		}

		//-----------------------------------------------------------------------------
		template <class T>
		bool MultiList<T>::exists(PxI32 id, const T &item) const
		{
			return getPairNr(id, item) >= 0;
		}

		//-----------------------------------------------------------------------------
		template <class T>
		PxI32 MultiList<T>::size(PxI32 id) const
		{
			if (id >= PxI32(first.size()))
				return 0;

			PxI32 num = 0;
			PxI32 nr = first[id];
			while (nr >= 0)
			{
				num++;
				nr = next[nr];
			}
			return num;
		}

		//-----------------------------------------------------------------------------
		template <class T>
		PxI32 MultiList<T>::getPairNr(PxI32 id, const T &item) const
		{
			if (id < 0 || id >= PxI32(first.size()))
				return -1;
			PxI32 nr = first[id];
			while (nr >= 0) 
			{
				if (items[nr] == item)
					return nr;
				nr = next[nr];
			}
			return -1;
		}

		//-----------------------------------------------------------------------------
		template <class T>
		void MultiList<T>::remove(PxI32 id, const T &itemNr)
		{
			PxI32 nr = first[id];
			PxI32 prev = -1;
			while (nr >= 0 && items[nr] != itemNr) 
			{
				prev = nr;
				nr = next[nr];
			}
			if (nr < 0)
				return;
			if (prev >= 0)
				next[prev] = next[nr];
			else
				first[id] = next[nr];
			next[nr] = firstFree;
			firstFree = nr;
		}

		//-----------------------------------------------------------------------------
		template <class T>
		void MultiList<T>::replace(PxI32 id, const T &before, const T &after)
		{
			PxI32 nr = first[id];
			while (nr >= 0) 
			{
				if (items[nr] == before)
					items[nr] = after;
				nr = next[nr];
			}
		}

		//-----------------------------------------------------------------------------
		template <class T>
		void MultiList<T>::removeAll(PxI32 id)
		{
			if (id >= PxI32(first.size()))
				return;

			PxI32 nr = first[id];
			if (nr < 0)
				return;

			PxI32 prev = -1;
			while (nr >= 0) 
			{
				prev = nr;
				nr = next[nr];
			}
			next[prev] = firstFree;
			firstFree = first[id];
			first[id] = -1;
		}

		//-----------------------------------------------------------------------------
		template <class T>
		void MultiList<T>::getItems(PxI32 id) const
		{
			queryItems.clear();
			if (id >= PxI32(first.size()))
				return;
			PxI32 nr = first[id];
			while (nr >= 0)
			{
				queryItems.push_back(items[nr]);
				nr = next[nr];
			}
		}

		//-----------------------------------------------------------------------------
		template <class T>
		void MultiList<T>::initIteration(PxI32 id, PxI32& iterator)
		{
			if (id >= PxI32(first.size()))
				iterator = -1;
			else
				iterator = first[id];
		}

		//-----------------------------------------------------------------------------
		template <class T>
		bool MultiList<T>::iterate(T& item, PxI32& iterator)
		{
			if (iterator >= 0) 
			{
				item = items[iterator];
				iterator = next[iterator];
				return true;
			}
			return false;
		}

		//-----------------------------------------------------------------------------
		template <class T>
		void MultiList<T>::getPointers(PxI32 id)
		{
			queryPointers.clear();
			if (id >= PxI32(first.size()))
				return;
			PxI32 nr = first[id];
			while (nr >= 0) 
			{
				queryPointers.push_back(&items[nr]);
				nr = next[nr];
			}
		}

	}
}

#endif
