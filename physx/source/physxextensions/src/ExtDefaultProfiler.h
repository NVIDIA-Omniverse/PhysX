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

#ifndef EXT_DEFAULT_PROFILER_H
#define EXT_DEFAULT_PROFILER_H

#include "extensions/PxDefaultProfiler.h"

#include "foundation/PxProfiler.h"
#include "foundation/PxHashSet.h"
#include "foundation/PxArray.h"
#include "foundation/PxMutex.h"
#include "foundation/PxSList.h"


#define DEFAULT_PROFILER_CHECK_ALIGNMENT(value, alignment)		PX_ASSERT((value & (alignment - 1)) == 0);
#define DEFAULT_PROFILER_REQUIRED_ALIGNMENT						8


namespace physx
{
namespace Ext
{


class DefaultProfilerDataBlock : public PxSListEntry, public PxUserAllocated
{
public:
	char* mBuffer;
	char* mNextEntry;
	char* mEndBuffer;

	PxHashSet<PxU64> mNameKeys;

	PxOutputStream& mOutputStream;
	PxMutex& mMutex;


	DefaultProfilerDataBlock(const PxU32 bufferSize, PxOutputStream& outputStream, PxMutex& mutex) : 
		mOutputStream(outputStream), 
		mMutex(mutex)
	{
		mBuffer = static_cast<char*>(PxAlignedAllocator<DEFAULT_PROFILER_REQUIRED_ALIGNMENT>().allocate(bufferSize, PX_FL));
		mNextEntry = mBuffer;
		mEndBuffer = mBuffer + bufferSize;
	}

	~DefaultProfilerDataBlock() 
	{ 
		PxAlignedAllocator<DEFAULT_PROFILER_REQUIRED_ALIGNMENT>().deallocate(mBuffer);
	}

	PxU32 size() const
	{ 
		return (PxU32)(mNextEntry - mBuffer);
	}

	bool isFull(const PxU32 size) const
	{ 
		return (mNextEntry + size >= mEndBuffer);
	}

	void seek(const PxU32 offset) 
	{
		mNextEntry = mBuffer + offset;
	}

	void flush()
	{
		// The thread ID is stored at the start of every data block and must be preserved.
		static const PxU32 sThreadOffset = sizeof(PxDefaultProfilerHeader) + sizeof(PxDefaultProfilerThread);

		// The thread must be locked to prevent multiple writes to the stream at the same time.
		PxMutex::ScopedLock scopedLock(mMutex);

		// Keep the header that lists the thread Id for this data block.
		mOutputStream.write(mBuffer, size());
		seek(sThreadOffset);
	}


	template <typename TEntry>
	void write(PxDefaultProfilerDataType::Enum type, TEntry*& entry, const char* name = nullptr)
	{
		static const PxU32 sNameSize = sizeof(PxDefaultProfilerHeader) + sizeof(PxDefaultProfilerName);

		PxU64 key = (PxU64)name;
		PxU32 nameSize;
		PxU32 nameStringSize;
		PxU32 paddedNameStringSize;

		if(name && mNameKeys.contains(key) == false)
		{
			nameSize = sNameSize;
			nameStringSize = (PxU32)strlen(name) + 1;

			// Pad the string buffer.
			paddedNameStringSize = (nameStringSize + (DEFAULT_PROFILER_REQUIRED_ALIGNMENT - 1)) & ~(DEFAULT_PROFILER_REQUIRED_ALIGNMENT - 1);
		}
		else
		{
			nameSize = 0;
			nameStringSize = 0;
			paddedNameStringSize = 0;
		}

		// Check if there is enough space in the buffer.
		const PxU32 requiredSize = sizeof(PxDefaultProfilerHeader) + sizeof(TEntry) + nameSize + paddedNameStringSize;

		if(isFull(requiredSize))
		{
			flush();
		}

		PxDefaultProfilerHeader* header = reinterpret_cast<PxDefaultProfilerHeader*>(mNextEntry);
		DEFAULT_PROFILER_CHECK_ALIGNMENT((PxU64)header, DEFAULT_PROFILER_REQUIRED_ALIGNMENT)
		mNextEntry += sizeof(PxDefaultProfilerHeader);
		header->type = type;

		// Point the entry to the correct point in the buffer. Write to the buffer outside of this funciton.
		entry = reinterpret_cast<TEntry*>(mNextEntry);
		DEFAULT_PROFILER_CHECK_ALIGNMENT((PxU64)entry, DEFAULT_PROFILER_REQUIRED_ALIGNMENT)
		mNextEntry += sizeof(TEntry);

		// Write the name if necessary.
		if(nameSize > 0)
		{
			header = reinterpret_cast<PxDefaultProfilerHeader*>(mNextEntry);
			DEFAULT_PROFILER_CHECK_ALIGNMENT((PxU64)header, DEFAULT_PROFILER_REQUIRED_ALIGNMENT)
			mNextEntry += sizeof(PxDefaultProfilerHeader);
			header->type = PxDefaultProfilerDataType::eNAME_REGISTRATION;

			PxDefaultProfilerName* profilerName = reinterpret_cast<PxDefaultProfilerName*>(mNextEntry);
			DEFAULT_PROFILER_CHECK_ALIGNMENT((PxU64)profilerName, DEFAULT_PROFILER_REQUIRED_ALIGNMENT)
			mNextEntry += sizeof(PxDefaultProfilerName);
			profilerName->key = key;
			profilerName->size = paddedNameStringSize;

			DEFAULT_PROFILER_CHECK_ALIGNMENT((PxU64)mNextEntry, DEFAULT_PROFILER_REQUIRED_ALIGNMENT)
			PxMemCopy(mNextEntry, name, nameStringSize);
			mNextEntry += paddedNameStringSize;

			mNameKeys.insert(key);
		}
	} 
};


class DefaultProfiler : public PxDefaultProfiler, public PxUserAllocated
{
PX_NOCOPY(DefaultProfiler)

private:
	~DefaultProfiler();

public:
	DefaultProfiler(PxOutputStream& outputStream, PxU32 numberOfBuffers, PxU32 bufferSize);

	virtual void release() PX_OVERRIDE;
	virtual void flush() PX_OVERRIDE;

	virtual void* zoneStart(const char* eventName, bool detached, uint64_t contextId) PX_OVERRIDE;
	virtual void zoneEnd(void* profilerData, const char* eventName, bool detached, uint64_t contextId) PX_OVERRIDE;
	virtual void recordData(int32_t value, const char* valueName, uint64_t contextId) PX_OVERRIDE;
	virtual void recordData(float value, const char* valueName, uint64_t contextId) PX_OVERRIDE;
	virtual void recordFrame(const char* name, uint64_t contextId) PX_OVERRIDE;

 protected:

	// Return a pointer to the data block.
	DefaultProfilerDataBlock* getThreadData();

	template <typename TEntry>
	TEntry* writeProfilerEvent(const char* name, PxDefaultProfilerDataType::Enum type, PxU64 contextId);


	PxOutputStream& mOutputStream;

	// Container for the pre-allocated event data blocks.
	PxSList mEmptyList;

	// Container for the used event entry data.
	PxSList mUsedList;

	// TLS Implementation.
	PxU32 mTlsSlotId;
	PxMutex mMutex;

	PxU32 mBufferSize;
};

} // namespace Ext
}

#endif
