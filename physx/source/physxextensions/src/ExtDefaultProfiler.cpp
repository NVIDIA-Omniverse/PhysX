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

#include "foundation/PxAssert.h"
#include "foundation/PxTime.h"
#include "foundation/PxThread.h"
#include "ExtDefaultProfiler.h"


using namespace physx;

const PxU32 gMinBufferSize = 32767;


// Ensure alignment for all structs is as expected.
#define DEFAULT_PROFILER_CHECK_ALIGNMENT_SIZE(type, alignment)		PX_COMPILE_TIME_ASSERT((sizeof(type) & (alignment - 1)) == 0);

DEFAULT_PROFILER_CHECK_ALIGNMENT_SIZE(PxDefaultProfilerVersionInfo, DEFAULT_PROFILER_REQUIRED_ALIGNMENT)
DEFAULT_PROFILER_CHECK_ALIGNMENT_SIZE(PxDefaultProfilerHeader, DEFAULT_PROFILER_REQUIRED_ALIGNMENT)
DEFAULT_PROFILER_CHECK_ALIGNMENT_SIZE(PxDefaultProfilerThread, DEFAULT_PROFILER_REQUIRED_ALIGNMENT)
DEFAULT_PROFILER_CHECK_ALIGNMENT_SIZE(PxDefaultProfilerName, DEFAULT_PROFILER_REQUIRED_ALIGNMENT)
DEFAULT_PROFILER_CHECK_ALIGNMENT_SIZE(PxDefaultProfilerEvent, DEFAULT_PROFILER_REQUIRED_ALIGNMENT)
DEFAULT_PROFILER_CHECK_ALIGNMENT_SIZE(PxDefaultProfilerValueEvent, DEFAULT_PROFILER_REQUIRED_ALIGNMENT)


PxDefaultProfiler* physx::PxDefaultProfilerCreate(PxOutputStream& outputStream, PxU32 numberOfBuffers, PxU32 bufferSize)
{
	return PX_NEW(Ext::DefaultProfiler)(outputStream, numberOfBuffers, bufferSize);
}

Ext::DefaultProfiler::DefaultProfiler(PxOutputStream& outputStream, PxU32 numberOfBuffers, PxU32 bufferSize) : 
	mOutputStream(outputStream)
{
	mTlsSlotId = PxTlsAlloc();

	// Ensure the buffer size is large enough to hold some minimal set of data.
	if(bufferSize < gMinBufferSize)
	{
		PxGetFoundation().error(physx::PxErrorCode::eDEBUG_WARNING, PX_FL,
								"Ext::DefaultProfiler::DefaultProfiler: buffer was increased to the minimum buffer size of %d bytes.",
								gMinBufferSize);

		mBufferSize = gMinBufferSize;
	}
	else
	{
		mBufferSize = bufferSize;
	}

	// Pre-allocate all of the profiler data blocks.
	for(PxU32 i = 0; i < numberOfBuffers; i++)
	{
		DefaultProfilerDataBlock* dataBlock = PX_NEW(DefaultProfilerDataBlock)(bufferSize, outputStream, mMutex);
		mEmptyList.push(*dataBlock);
	}

	// Save version info.
	PxDefaultProfilerVersionInfo versionInfo;
	versionInfo.major = PX_PROFILER_VERSION_MAJOR;
	versionInfo.minor = PX_PROFILER_VERSION_MINOR;
	outputStream.write(&versionInfo, sizeof(PxDefaultProfilerVersionInfo));
}

Ext::DefaultProfiler::~DefaultProfiler()
{
	flush();

	// Delete all of the used and empty blocks.
	DefaultProfilerDataBlock* dataBlock = reinterpret_cast<DefaultProfilerDataBlock*>(mEmptyList.pop());

	while(dataBlock)
	{
		PX_DELETE(dataBlock);

		dataBlock = reinterpret_cast<DefaultProfilerDataBlock*>(mEmptyList.pop());
	}

	dataBlock = reinterpret_cast<DefaultProfilerDataBlock*>(mUsedList.pop());

	while(dataBlock)
	{
		PX_DELETE(dataBlock);

		dataBlock = reinterpret_cast<DefaultProfilerDataBlock*>(mUsedList.pop());
	}

	PxTlsFree(mTlsSlotId);
}

void Ext::DefaultProfiler::release() 
{
	PX_DELETE_THIS; 
}

void Ext::DefaultProfiler::flush()
{
	// Loop through every used data block and flush them to the stream.
	DefaultProfilerDataBlock* usedDataBlock = reinterpret_cast<DefaultProfilerDataBlock*>(mUsedList.flush());

	while(usedDataBlock)
	{
		usedDataBlock->flush();

		DefaultProfilerDataBlock* previousDataBlock = usedDataBlock;
		usedDataBlock = reinterpret_cast<DefaultProfilerDataBlock*>(usedDataBlock->next());

		mUsedList.push(*previousDataBlock);
	}
}

Ext::DefaultProfilerDataBlock* Ext::DefaultProfiler::getThreadData()
{
	DefaultProfilerDataBlock* dataBlock = reinterpret_cast<DefaultProfilerDataBlock*>(PxTlsGet(mTlsSlotId));

	// This thread never had a profiling callback, grab a data block from the pool of pre-allocated blocks.
	// If there are none left, allocate a new block.
	if(!dataBlock)
	{
		dataBlock = static_cast<DefaultProfilerDataBlock*>(mEmptyList.pop());

		if(!dataBlock)
		{
			dataBlock = PX_NEW(DefaultProfilerDataBlock)(mBufferSize, mOutputStream, mMutex);
		}

		// Store the data block in the TLS.
		PxTlsSet(mTlsSlotId, dataBlock);

		// Write the event block header.
		PxU64 threadId = PxThread::getId();

		PxDefaultProfilerThread* thread;
		dataBlock->write(PxDefaultProfilerDataType::eTHREAD_BLOCK, thread);
		thread->threadId = threadId;

		// Save the new data block in the user list.
		mUsedList.push(*dataBlock);
	}

	return dataBlock;
}

template <typename TEntry>
TEntry* Ext::DefaultProfiler::writeProfilerEvent(const char* name, PxDefaultProfilerDataType::Enum type, PxU64 contextId)
{
	Ext::DefaultProfilerDataBlock* dataBlock = getThreadData();

	TEntry* entry;
	dataBlock->write(type, entry, name);

	PxDefaultProfilerEvent*event = static_cast<PxDefaultProfilerEvent*>(entry);
	event->time = PxTime::getCurrentTimeInTensOfNanoSeconds();
	event->contextId = contextId;
	event->nameKey = PxU64(name);

	return entry;
}

void* Ext::DefaultProfiler::zoneStart(const char* eventName, bool detached, uint64_t contextId)
{
	PxDefaultProfilerDataType::Enum type;

	if(detached)
	{
		type = PxDefaultProfilerDataType::eZONE_START_CROSS_THREAD;
	}
	else
	{
		type = PxDefaultProfilerDataType::eZONE_START;
	}

	writeProfilerEvent<PxDefaultProfilerEvent>(eventName, type, contextId);

	return NULL;
}

void Ext::DefaultProfiler::zoneEnd(void*, const char* eventName, bool detached, uint64_t contextId)
{
	PxDefaultProfilerDataType::Enum type;

	if(detached)
	{
		type = PxDefaultProfilerDataType::eZONE_END_CROSS_THREAD;
	}
	else
	{
		type = PxDefaultProfilerDataType::eZONE_END;
	}

	writeProfilerEvent<PxDefaultProfilerEvent>(eventName, type, contextId);
}

void Ext::DefaultProfiler::recordData(int32_t value, const char* valueName, uint64_t contextId)
{
	PxDefaultProfilerValueEvent* valueEvent = writeProfilerEvent<PxDefaultProfilerValueEvent>(valueName, PxDefaultProfilerDataType::eVALUE_INT, contextId);
	valueEvent->intValue = value;
}

void Ext::DefaultProfiler::recordData(float value, const char* valueName, uint64_t contextId)
{
	PxDefaultProfilerValueEvent* valueEvent = writeProfilerEvent<PxDefaultProfilerValueEvent>(valueName, PxDefaultProfilerDataType::eVALUE_FLOAT, contextId);
	valueEvent->floatValue = value;
}

void Ext::DefaultProfiler::recordFrame(const char* name, uint64_t contextId)
{
	writeProfilerEvent<PxDefaultProfilerEvent>(name, PxDefaultProfilerDataType::eFRAME, contextId);
}
