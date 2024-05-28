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

#ifndef OMNI_PVD_CHUNK_ALLOC_H
#define OMNI_PVD_CHUNK_ALLOC_H

#include "foundation/PxUserAllocated.h"

////////////////////////////////////////////////////////////////////////////////
// The usage reason for splitting the Chunk allocator into a ChunkPool and a
// ByteAllocator is so that similarly accessed objects get allocated next to each
// other within the same Chunk, which is managed by the ByteAllocator. The
// ChunkPool is basically just a resource pool. Not thread safe anything :-)
// Locking is bad mkkk...
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Usage example of setting up a ChunkPool, a ByteAllocator and allocating some
// bytes, for the lulz.
////////////////////////////////////////////////////////////////////////////////

/*
	OmniPvdChunkPool pool;
	
	// set the individual Chunks to a size of 100k bytes
	pool.setChunkSize(100000);
	
	// all allocations must go through a ByteAllocator, so create one
	OmniPvdChunkByteAllocator allocator;
	
	// connect the ByteAllocator to the pool
	allocator.setPool(&pool); 
	
	// we allocate 100 bytes through the ByteAllocator
	unsigned char *someBytes=allocator.allocBytes(100);
*/

////////////////////////////////////////////////////////////////////////////////
// Once we want to cleanup the allocations, return the Chunks to the ChunkPool,
// which is also done silently in the destructor of the ByteAllocator.
////////////////////////////////////////////////////////////////////////////////

/*
	allocator.freeChunks();
*/

////////////////////////////////////////////////////////////////////////////////
// Memory management warning! The ByteAllocator does not deallocate the Chunks,
// but must return the Chunks to the ChunkPool before going out of scope. The
// ChunkPool can only deallocate the Chunks that is has in its free Chunks list,
// so all ByteAllocators must either all go out of scope before the ChunkPool
// goes out of scope or the ByteAllocators must all call the function freeChunks
// before the ChunkPool goes out of scope.
////////////////////////////////////////////////////////////////////////////////

class OmniPvdChunkElement
{
public:
	OmniPvdChunkElement();
	~OmniPvdChunkElement();
	OmniPvdChunkElement*	mNextElement;
};

class OmniPvdChunk : public physx::PxUserAllocated
{
public:
	OmniPvdChunk();
	~OmniPvdChunk();

	void			resetChunk(int nbrBytes);
	int				nbrBytesUSed();

	unsigned char*	mData;
	int				mNbrBytesAllocated;
	int				mNbrBytesFree;
	unsigned char*	mBytePtr;
	OmniPvdChunk*	mNextChunk;
};

class OmniPvdChunkList
{
public:
	OmniPvdChunkList();
	~OmniPvdChunkList();

	void			resetList();
	void			appendList(OmniPvdChunkList* list);
	void			appendChunk(OmniPvdChunk* chunk);
	OmniPvdChunk*	removeFirst();

	OmniPvdChunk*	mFirstChunk;
	OmniPvdChunk*	mLastChunk;
	int				mNbrChunks;
};

class OmniPvdChunkPool
{
public:
	OmniPvdChunkPool();
	~OmniPvdChunkPool();

	void				setChunkSize(int chunkSize);
	OmniPvdChunk*		getChunk();

	OmniPvdChunkList	mFreeChunks;
	int					mChunkSize;
	int					mNbrAllocatedChunks;
};


class OmniPvdChunkByteAllocator
{
public:
	OmniPvdChunkByteAllocator();
	~OmniPvdChunkByteAllocator();

	OmniPvdChunkElement*	allocChunkElement(int totalStructSize);
	void					setPool(OmniPvdChunkPool* pool);
	unsigned char*			allocBytes(int nbrBytes);
	void					freeChunks();

	OmniPvdChunkList		mUsedChunks;
	OmniPvdChunkPool*		mPool;
};

#endif
