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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef PX_PVD_IMPL_H
#define PX_PVD_IMPL_H

#include "foundation/PxProfiler.h"

#include "foundation/PxAllocator.h"
#include "PsPvd.h"
#include "foundation/PxArray.h"
#include "foundation/PxMutex.h"
#include "PxPvdCommStreamTypes.h"
#include "PxPvdFoundation.h"
#include "PxPvdObjectModelMetaData.h"
#include "PxPvdObjectRegistrar.h"

namespace physx
{

namespace profile
{
	class PxProfileZoneManager;
}

namespace pvdsdk
{
class PvdMemClient;
class PvdProfileZoneClient;

struct MetaDataProvider : public PvdOMMetaDataProvider, public PxUserAllocated
{
    typedef PxMutex::ScopedLock TScopedLockType;
    typedef PxHashMap<const void*, int32_t> TInstTypeMap;
	PvdObjectModelMetaData& mMetaData;
    PxMutex mMutex;
	uint32_t mRefCount;
	TInstTypeMap mTypeMap;

	MetaDataProvider()
	: mMetaData(PvdObjectModelMetaData::create()), mRefCount(0), mTypeMap("MetaDataProvider::mTypeMap")
	{
		mMetaData.addRef();
	}
	virtual ~MetaDataProvider()
	{
		mMetaData.release();
	}

	virtual void addRef() PX_OVERRIDE
	{
		TScopedLockType locker(mMutex);
		++mRefCount;
	}
	virtual void release() PX_OVERRIDE
	{
		{
			TScopedLockType locker(mMutex);
			if(mRefCount)
				--mRefCount;
		}
		if(!mRefCount)
			PVD_DELETE(this);
	}
	virtual PvdObjectModelMetaData& lock() PX_OVERRIDE
	{
		mMutex.lock();
		return mMetaData;
	}
	virtual void unlock() PX_OVERRIDE
	{
		mMutex.unlock();
	}

	virtual bool createInstance(const NamespacedName& clsName, const void* instance) PX_OVERRIDE
	{
		TScopedLockType locker(mMutex);
		Option<ClassDescription> cls(mMetaData.findClass(clsName));
		if(cls.hasValue() == false)
			return false;
		int32_t instType = cls->mClassId;
		mTypeMap.insert(instance, instType);
		return true;
	}
	virtual bool isInstanceValid(const void* instance) PX_OVERRIDE
	{
		TScopedLockType locker(mMutex);
		ClassDescription classDesc;
		bool retval = mTypeMap.find(instance) != NULL;
#if PX_DEBUG
		if(retval)
			classDesc = mMetaData.getClass(mTypeMap.find(instance)->second);
#endif
		return retval;
	}
	virtual void destroyInstance(const void* instance) PX_OVERRIDE
	{
		{
			TScopedLockType locker(mMutex);
			mTypeMap.erase(instance);
		}
	}
	virtual int32_t getInstanceClassType(const void* instance) PX_OVERRIDE
	{
		TScopedLockType locker(mMutex);
		const TInstTypeMap::Entry* entry = mTypeMap.find(instance);
		if(entry)
			return entry->second;
		return -1;
	}

  private:
	MetaDataProvider& operator=(const MetaDataProvider&);
	MetaDataProvider(const MetaDataProvider&);
};

//////////////////////////////////////////////////////////////////////////
/*!
PvdImpl is the realization of PxPvd.
It implements the interface methods and provides richer functionality for advanced users or internal clients (such as
PhysX or APEX), including handler notification for clients.
*/
//////////////////////////////////////////////////////////////////////////
class PvdImpl : public PsPvd, public PxUserAllocated
{
	PX_NOCOPY(PvdImpl)

    typedef PxMutex::ScopedLock TScopedLockType;
	typedef void (PvdImpl::*TAllocationHandler)(size_t size, const char* typeName, const char* filename, int line,
	                                            void* allocatedMemory);
	typedef void (PvdImpl::*TDeallocationHandler)(void* allocatedMemory);

  public:
	PvdImpl();
	virtual ~PvdImpl();
	virtual	void release()	PX_OVERRIDE;

	virtual	bool connect(PxPvdTransport& transport, PxPvdInstrumentationFlags flags)	PX_OVERRIDE;
	virtual	void disconnect()	PX_OVERRIDE;
	virtual	bool isConnected(bool useCachedStatus = true)	PX_OVERRIDE;
	virtual	void flush()	PX_OVERRIDE;

	virtual	PxPvdTransport* getTransport()	PX_OVERRIDE;
	virtual	PxPvdInstrumentationFlags getInstrumentationFlags()	PX_OVERRIDE;

	virtual	void addClient(PvdClient* client)	PX_OVERRIDE;
	virtual	void removeClient(PvdClient* client)	PX_OVERRIDE;

	virtual	PvdOMMetaDataProvider& getMetaDataProvider()	PX_OVERRIDE;

	virtual	bool registerObject(const void* inItem)	PX_OVERRIDE;
	virtual	bool unRegisterObject(const void* inItem)	PX_OVERRIDE;

	//AllocationListener
	virtual void onAllocation(size_t size, const char* typeName, const char* filename, int line, void* allocatedMemory)	PX_OVERRIDE;
	virtual void onDeallocation(void* addr)	PX_OVERRIDE;

	virtual	uint64_t getNextStreamId()	PX_OVERRIDE;

	static bool initialize();
	static PvdImpl* getInstance();

	// Profiling

	virtual void* zoneStart(const char* eventName, bool detached, uint64_t contextId) PX_OVERRIDE;

	virtual void zoneEnd(void* profilerData, const char *eventName, bool detached, uint64_t contextId) PX_OVERRIDE;

  private:
	void sendTransportInitialization();

	PxPvdTransport*						mPvdTransport;
	physx::PxArray<PvdClient*>	mPvdClients;

	MetaDataProvider*					mSharedMetaProvider; // shared between clients
	ObjectRegistrar						mObjectRegistrar;

	PvdMemClient*						mMemClient;

	PxPvdInstrumentationFlags			mFlags;
	bool								mIsConnected;
	bool                                mGPUProfilingWasConnected;
	bool								mIsNVTXSupportEnabled;
	uint32_t							mNVTXContext;
	uint64_t							mNextStreamId;
	physx::profile::PxProfileZoneManager*mProfileZoneManager;
	PvdProfileZoneClient*				mProfileClient;
	physx::profile::PxProfileZone*		mProfileZone;
	static PvdImpl*						sInstance;
	static uint32_t						sRefCount;
};

} // namespace pvdsdk
}

#endif

