// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "OmniPvdChunkAlloc.h"
#include <string.h> // memcpy

OmniPvdChunkElement::OmniPvdChunkElement() {
}

OmniPvdChunkElement::~OmniPvdChunkElement() {
}

OmniPvdChunk::OmniPvdChunk() {
  m_data=0;
  m_nbrBytesAllocated=0;
  m_nbrBytesFree=0;
  m_bytePtr=0;
  m_nextChunk=0;
}

OmniPvdChunk::~OmniPvdChunk() {
 delete [] m_data;
}

void OmniPvdChunk::resetChunk(int nbrBytes) {
  if (nbrBytes>m_nbrBytesAllocated) {
    delete [] m_data;
    m_data=new unsigned char[nbrBytes];
    m_nbrBytesAllocated=nbrBytes;
  }
  m_nbrBytesFree=m_nbrBytesAllocated;
  m_bytePtr=m_data;
}

int OmniPvdChunk::nbrBytesUSed() {
  return m_nbrBytesAllocated-m_nbrBytesFree;
}

OmniPvdChunkList::OmniPvdChunkList() {
  resetList();
}

OmniPvdChunkList::~OmniPvdChunkList() {
}

void OmniPvdChunkList::resetList() {
  m_firstChunk=0;
  m_lastChunk=0;
  m_nbrChunks=0;
}

OmniPvdChunk* OmniPvdChunkList::removeFirst() {
   if (m_firstChunk) {
	   OmniPvdChunk *chunk=m_firstChunk;
		m_firstChunk=m_firstChunk->m_nextChunk;
		if (!m_firstChunk) { // Did we remove the last one?
		m_lastChunk=0;
		}
		chunk->m_nextChunk=0;
		m_nbrChunks--;
		return chunk;
   } else {
     return 0;
   }
}

void OmniPvdChunkList::appendList(OmniPvdChunkList* list) {
  if (!list) return;
  if (list->m_nbrChunks==0) return;
  if (list->m_firstChunk==0) return;
  if (m_firstChunk) {
    m_lastChunk->m_nextChunk=list->m_firstChunk;
    m_lastChunk=list->m_lastChunk;
  } else {
    m_firstChunk=list->m_firstChunk;
    m_lastChunk=list->m_lastChunk;
  }
  m_nbrChunks+=list->m_nbrChunks;
  list->resetList();
}

void OmniPvdChunkList::appendChunk(OmniPvdChunk* chunk) {
  if (!chunk) return;
  if (m_firstChunk) {
    if (m_lastChunk != nullptr)
        m_lastChunk->m_nextChunk=chunk;
    m_lastChunk=chunk;
  } else {
    m_firstChunk=chunk;
    m_lastChunk=chunk;
  }
  chunk->m_nextChunk=0;
  m_nbrChunks++;
}

OmniPvdChunkPool::OmniPvdChunkPool() {
	mNbrAllocatedChunks = 0;
	mChunkSize = 65536; // random default value :-)
}

OmniPvdChunkPool::~OmniPvdChunkPool() {
	OmniPvdChunk *chunk=m_freeChunks.m_firstChunk;
	while (chunk) {
		OmniPvdChunk *nextChunk=chunk->m_nextChunk;
		delete chunk;
		chunk=nextChunk;
	}
	mNbrAllocatedChunks = 0;
}

void OmniPvdChunkPool::setChunkSize(int chunkSize) {
	if (mNbrAllocatedChunks > 0) return; // makes it impossible to re-set the size of a chunk once a chunk was already allocated
	mChunkSize = chunkSize;
}

OmniPvdChunk* OmniPvdChunkPool::getChunk() {
	OmniPvdChunk* chunk=m_freeChunks.removeFirst();
	if (!chunk) {
		chunk= new OmniPvdChunk();
		mNbrAllocatedChunks++;
	}
	chunk->resetChunk(mChunkSize);
	return chunk;
}

OmniPvdChunkByteAllocator::OmniPvdChunkByteAllocator() {
	mPool = 0;
}

OmniPvdChunkByteAllocator::~OmniPvdChunkByteAllocator() {
	freeChunks();
}

OmniPvdChunkElement* OmniPvdChunkByteAllocator::allocChunkElement(int totalStructSize) {
	return (OmniPvdChunkElement*)allocBytes(totalStructSize);
}

void OmniPvdChunkByteAllocator::setPool(OmniPvdChunkPool *pool) {
	mPool = pool;
}

unsigned char* OmniPvdChunkByteAllocator::allocBytes(int nbrBytes) {
	if (!mPool) return 0;
	if (mPool->mChunkSize < nbrBytes) {
		return 0; // impossible request!
	}
	if (m_usedChunks.m_lastChunk) { // If a chunk is free
		if (m_usedChunks.m_lastChunk->m_nbrBytesFree< nbrBytes) {
			// The last chunk has not enough space, so get a fresh one from the pool
			m_usedChunks.appendChunk(mPool->getChunk());
		}
	} else { // No last chunk is available, allocate a fresh one
		m_usedChunks.appendChunk(mPool->getChunk());
	}
	// Use the last chunk to allocate the data necessary
	unsigned char *pSubChunk=m_usedChunks.m_lastChunk->m_bytePtr;
	m_usedChunks.m_lastChunk->m_nbrBytesFree-= nbrBytes;
	m_usedChunks.m_lastChunk->m_bytePtr+= nbrBytes;
	return pSubChunk;
}

void OmniPvdChunkByteAllocator::freeChunks() {
	if (!mPool) return;
	mPool->m_freeChunks.appendList(&m_usedChunks);
}
