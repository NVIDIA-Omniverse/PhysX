// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

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

#pragma pack(push, 1)

class OmniPvdChunkElement
{
public:
    OmniPvdChunkElement();
    ~OmniPvdChunkElement();
    OmniPvdChunkElement* mNextElement;
};

class OmniPvdChunk
{
public:
    OmniPvdChunk();
    ~OmniPvdChunk();
    void resetChunk(int nbrBytes);
    int nbrBytesUSed();
    unsigned char* m_data;
    int m_nbrBytesAllocated;
    int m_nbrBytesFree;
    unsigned char* m_bytePtr;
    OmniPvdChunk* m_nextChunk;
};

class OmniPvdChunkList
{
public:
    OmniPvdChunkList();
    ~OmniPvdChunkList();
    void resetList();
    void appendList(OmniPvdChunkList* list);
    void appendChunk(OmniPvdChunk* chunk);
    OmniPvdChunk* removeFirst();
    OmniPvdChunk* m_firstChunk;
    OmniPvdChunk* m_lastChunk;
    int m_nbrChunks;
};

class OmniPvdChunkPool
{
public:
    OmniPvdChunkPool();
    ~OmniPvdChunkPool();
    void setChunkSize(int chunkSize);
    OmniPvdChunk* getChunk();
    OmniPvdChunkList m_freeChunks;
    int mChunkSize;
    int mNbrAllocatedChunks;
};


class OmniPvdChunkByteAllocator
{
public:
    OmniPvdChunkByteAllocator();
    ~OmniPvdChunkByteAllocator();
    OmniPvdChunkElement* allocChunkElement(int totalStructSize);
    void setPool(OmniPvdChunkPool* pool);
    unsigned char* allocBytes(int nbrBytes);
    void freeChunks();
    OmniPvdChunkList m_usedChunks;
    OmniPvdChunkPool* mPool;
};

#pragma pack(pop)
