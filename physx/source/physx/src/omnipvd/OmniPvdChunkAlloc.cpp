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

#include "OmniPvdChunkAlloc.h"
#include <string.h> // memcpy
#include "foundation/PxAllocator.h"

OmniPvdChunkElement::OmniPvdChunkElement()
{
}

OmniPvdChunkElement::~OmniPvdChunkElement()
{
}

OmniPvdChunk::OmniPvdChunk() :
	mData				(NULL),
	mNbrBytesAllocated	(0),
	mNbrBytesFree		(0),
	mBytePtr			(NULL),
	mNextChunk			(NULL)
{
}

OmniPvdChunk::~OmniPvdChunk()
{
	PX_FREE(mData);
}

void OmniPvdChunk::resetChunk(int nbrBytes)
{
	if (nbrBytes>mNbrBytesAllocated)
	{    
		PX_FREE(mData);
		mData = static_cast<unsigned char*>(PX_ALLOC(sizeof(unsigned char)*(nbrBytes), "m_data"));
		mNbrBytesAllocated=nbrBytes;
	}
	mNbrBytesFree=mNbrBytesAllocated;
	mBytePtr=mData;
}

int OmniPvdChunk::nbrBytesUSed()
{
	return mNbrBytesAllocated-mNbrBytesFree;
}

OmniPvdChunkList::OmniPvdChunkList()
{
	resetList();
}

OmniPvdChunkList::~OmniPvdChunkList()
{
}

void OmniPvdChunkList::resetList()
{
	mFirstChunk=0;
	mLastChunk=0;
	mNbrChunks=0;
}

OmniPvdChunk* OmniPvdChunkList::removeFirst()
{
	if (mFirstChunk)
	{   
		OmniPvdChunk* chunk=mFirstChunk;
		mFirstChunk=mFirstChunk->mNextChunk;
		if (!mFirstChunk)
		{ // Did we remove the last one?
			mLastChunk=NULL;
		}
		chunk->mNextChunk=NULL;
		mNbrChunks--;
		return chunk;
	}
	else
	{
		return 0;
	}
}

void OmniPvdChunkList::appendList(OmniPvdChunkList* list)
{
	if (!list) return;
	if (list->mNbrChunks==0) return; 
	if (list->mFirstChunk==NULL) return; 
	if (mFirstChunk)
	{
		mLastChunk->mNextChunk=list->mFirstChunk;
		mLastChunk=list->mLastChunk;
	} else
	{
		mFirstChunk=list->mFirstChunk;
		mLastChunk=list->mLastChunk;
	}
	mNbrChunks+=list->mNbrChunks;
	list->resetList();
}

void OmniPvdChunkList::appendChunk(OmniPvdChunk* chunk)
{
	if (!chunk) return;
	if (mFirstChunk)
	{
		mLastChunk->mNextChunk=chunk;
		mLastChunk=chunk;
	} else
	{
		mFirstChunk=chunk;
		mLastChunk=chunk;
	}
	chunk->mNextChunk=NULL;
	mNbrChunks++;
}

OmniPvdChunkPool::OmniPvdChunkPool()
{
	mNbrAllocatedChunks = 0;
	mChunkSize = 65536; // random default value :-)
}

OmniPvdChunkPool::~OmniPvdChunkPool()
{
	OmniPvdChunk* chunk=mFreeChunks.mFirstChunk;
	while (chunk)
	{
		OmniPvdChunk* nextChunk=chunk->mNextChunk;
		PX_DELETE(chunk);
		chunk=nextChunk;
	}
	mNbrAllocatedChunks = 0;
}

void OmniPvdChunkPool::setChunkSize(int chunkSize)
{
	if (mNbrAllocatedChunks > 0) return; // makes it impossible to re-set the size of a chunk once a chunk was already allocated
	mChunkSize = chunkSize;
}

OmniPvdChunk* OmniPvdChunkPool::getChunk()
{
	OmniPvdChunk* chunk=mFreeChunks.removeFirst();
	if (!chunk)
	{
		chunk = PX_NEW(OmniPvdChunk);
		mNbrAllocatedChunks++;
	}
	chunk->resetChunk(mChunkSize);
	return chunk;
}

OmniPvdChunkByteAllocator::OmniPvdChunkByteAllocator()
{
	mPool = NULL;
}

OmniPvdChunkByteAllocator::~OmniPvdChunkByteAllocator()
{
	freeChunks();
}

OmniPvdChunkElement* OmniPvdChunkByteAllocator::allocChunkElement(int totalStructSize)
{
	return reinterpret_cast<OmniPvdChunkElement*>(allocBytes(totalStructSize));
}

void OmniPvdChunkByteAllocator::setPool(OmniPvdChunkPool* pool)
{
	mPool = pool;
}

unsigned char* OmniPvdChunkByteAllocator::allocBytes(int nbrBytes)
{
	if (!mPool) return 0;
	if (mPool->mChunkSize < nbrBytes)
	{
		return 0; // impossible request!
	}
	if (mUsedChunks.mLastChunk)
	{ // If a chunk is free
		if (mUsedChunks.mLastChunk->mNbrBytesFree< nbrBytes)
		{
			// The last chunk has not enough space, so get a fresh one from the pool
			mUsedChunks.appendChunk(mPool->getChunk());
		}
	}
	else
	{ // No last chunk is available, allocate a fresh one
		mUsedChunks.appendChunk(mPool->getChunk());
	}
	// Use the last chunk to allocate the data necessary
	unsigned char* pSubChunk=mUsedChunks.mLastChunk->mBytePtr;
	mUsedChunks.mLastChunk->mNbrBytesFree-= nbrBytes;
	mUsedChunks.mLastChunk->mBytePtr+= nbrBytes;
	return pSubChunk;
}

void OmniPvdChunkByteAllocator::freeChunks()
{
	if (!mPool) return;
	mPool->mFreeChunks.appendList(&mUsedChunks);
}
