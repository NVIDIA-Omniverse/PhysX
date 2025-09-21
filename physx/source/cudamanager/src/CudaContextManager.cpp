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

#include "foundation/PxAssert.h"
#include "foundation/PxAtomic.h"
#include "foundation/PxErrorCallback.h"
#include "foundation/PxMath.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxMutex.h"
#include "foundation/PxThread.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxString.h"
#include "foundation/PxAlloca.h"
#include "foundation/PxArray.h"

#include "PhysXDeviceSettings.h"

// from the point of view of this source file the GPU library is linked statically
#ifndef PX_PHYSX_GPU_STATIC
	#define PX_PHYSX_GPU_STATIC
#endif
#include "PxPhysXGpu.h"

#if PX_LINUX && PX_CLANG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdisabled-macro-expansion"
#endif
#include <cuda.h>
#if PX_LINUX && PX_CLANG
#pragma clang diagnostic pop
#endif

#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"

#if PX_WIN32 || PX_WIN64

// Enable/disable NVIDIA secure load library code
#define SECURE_LOAD_LIBRARY !PX_PUBLIC_RELEASE

#include "foundation/windows/PxWindowsInclude.h"


class IDirect3DDevice9;
class IDirect3DResource9;
class IDirect3DVertexBuffer9;
#include <cudad3d9.h>

class IDXGIAdapter;
class ID3D10Device;
class ID3D10Resource;
#include <cudad3d10.h>

struct ID3D11Device;
struct ID3D11Resource;
#include <cudad3d11.h>

#endif // PX_WINDOWS_FAMILY

#if PX_LINUX
#include <dlfcn.h>
static void* GetProcAddress(void* handle, const char* name) { return dlsym(handle, name); }
#endif

// Defining these instead of including gl.h eliminates a dependency
typedef unsigned int GLenum;
typedef unsigned int GLuint;

//#include <GL/gl.h>
#include <cudaGL.h>
#include <assert.h>

#include "foundation/PxErrors.h"
#include "foundation/PxErrorCallback.h"
#include "common/PxPhysXCommonConfig.h"

namespace physx
{

#if PX_VC
#pragma warning(disable: 4191)	//'operator/operation' : unsafe conversion from 'type of expression' to 'type required'
#endif

// CUDA toolkit definitions
// Update the definitions when the Cuda toolkit changes
// Refer to https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html
#define MIN_CUDA_VERSION			12000	// Use Cuda toolkit 12.0 and above
#define NV_DRIVER_MAJOR_VERSION		527
#define NV_DRIVER_MINOR_VERSION		41
#define MIN_SM_MAJOR_VERSION		7
#define MIN_SM_MINOR_VERSION		0

#define USE_DEFAULT_CUDA_STREAM		0
#define FORCE_LAUNCH_SYNCHRONOUS	0
//PX_STOMP_ALLOCATED_MEMORY is defined in common/PxPhysXCommonConfig.h


#if PX_DEBUG
#include "PxgMemoryTracker.h"
static MemTracker mMemTracker;
#endif

PxCudaContext* createCudaContext(CUdevice device, PxDeviceAllocatorCallback* callback, bool launchSynchronous);

class CudaCtxMgr : public PxCudaContextManager, public PxUserAllocated
{
public:
	CudaCtxMgr(const PxCudaContextManagerDesc& desc, PxErrorCallback& errorCallback, bool launchSynchronous);
	virtual ~CudaCtxMgr();

	bool                    safeDelayImport(PxErrorCallback& errorCallback);

	virtual void            acquireContext() PX_OVERRIDE;
	virtual void            releaseContext() PX_OVERRIDE;
	virtual bool            tryAcquireContext() PX_OVERRIDE;

	/* All these methods can be called without acquiring the context */
	virtual bool            contextIsValid() const PX_OVERRIDE;
	virtual bool            supportsArchSM10() const PX_OVERRIDE;  // G80
	virtual bool            supportsArchSM11() const PX_OVERRIDE;  // G92
	virtual bool            supportsArchSM12() const PX_OVERRIDE;
	virtual bool            supportsArchSM13() const PX_OVERRIDE;  // GT200
	virtual bool            supportsArchSM20() const PX_OVERRIDE;  // GF100
	virtual bool            supportsArchSM30() const PX_OVERRIDE;  // GK100
	virtual bool            supportsArchSM35() const PX_OVERRIDE;  // GK110
	virtual bool            supportsArchSM50() const PX_OVERRIDE;  // GM100
	virtual bool            supportsArchSM52() const PX_OVERRIDE;  // GM200
	virtual bool            supportsArchSM60() const PX_OVERRIDE;  // GP100
	virtual bool            isIntegrated() const PX_OVERRIDE;      // true if GPU is integrated (MCP) part
	virtual bool            canMapHostMemory() const PX_OVERRIDE;  // true if GPU map host memory to GPU
	virtual int             getDriverVersion() const PX_OVERRIDE;
	virtual size_t          getDeviceTotalMemBytes() const PX_OVERRIDE;
	virtual int				getMultiprocessorCount() const PX_OVERRIDE;
	virtual int             getSharedMemPerBlock() const PX_OVERRIDE;
	virtual int             getSharedMemPerMultiprocessor() const PX_OVERRIDE;
	virtual unsigned int	getMaxThreadsPerBlock() const PX_OVERRIDE;
	virtual unsigned int	getClockRate() const PX_OVERRIDE;

	virtual const char*     getDeviceName() const PX_OVERRIDE;
	virtual CUdevice		getDevice() const PX_OVERRIDE;

	virtual void			setUsingConcurrentStreams(bool) PX_OVERRIDE;
	virtual bool			getUsingConcurrentStreams() const PX_OVERRIDE;

	virtual void            getDeviceMemoryInfo(size_t& free, size_t& total) const PX_OVERRIDE;

	virtual void            release() PX_OVERRIDE;

	virtual CUcontext		getContext() PX_OVERRIDE { return mCtx; }

	virtual PxCudaContext*  getCudaContext() PX_OVERRIDE { return mCudaCtx; }

	CUmodule* getCuModules() PX_OVERRIDE { return mCuModules.begin(); }

	virtual CUdeviceptr     getMappedDevicePtr(void* pinnedHostBuffer) PX_OVERRIDE;

protected:
	virtual void* allocDeviceBufferInternal(PxU64 numBytes, const char* filename, PxI32 line) PX_OVERRIDE;
	virtual void* allocPinnedHostBufferInternal(PxU64 numBytes, const char* filename, PxI32 line) PX_OVERRIDE;

	virtual void freeDeviceBufferInternal(void* deviceBuffer) PX_OVERRIDE;
	virtual void freePinnedHostBufferInternal(void* pinnedHostBuffer) PX_OVERRIDE;

	virtual void clearDeviceBufferAsyncInternal(void* deviceBuffer, PxU32 numBytes, CUstream stream, PxI32 value) PX_OVERRIDE;

	virtual void copyDToHAsyncInternal(void* hostBuffer, const void* deviceBuffer, PxU32 numBytes, CUstream stream) PX_OVERRIDE;
	virtual void copyHToDAsyncInternal(void* deviceBuffer, const void* hostBuffer, PxU32 numBytes, CUstream stream) PX_OVERRIDE;
	virtual void copyDToDAsyncInternal(void* dstDeviceBuffer, const void* srcDeviceBuffer, PxU32 numBytes, CUstream stream) PX_OVERRIDE;

	virtual void copyDToHInternal(void* hostBuffer, const void* deviceBuffer, PxU32 numBytes) PX_OVERRIDE;
	virtual void copyHToDInternal(void* deviceBuffer, const void* hostBuffer, PxU32 numBytes) PX_OVERRIDE;

	virtual void memsetD8AsyncInternal(void* dstDeviceBuffer, const PxU8& value, PxU32 numBytes, CUstream stream) PX_OVERRIDE;
	virtual void memsetD32AsyncInternal(void* dstDeviceBuffer, const PxU32& value, PxU32 numIntegers, CUstream stream) PX_OVERRIDE;

private:

	PxArray<CUmodule>	mCuModules;

	bool            mIsValid;
	bool			mOwnContext;
	CUdevice        mDevHandle;
	CUcontext       mCtx;
	PxCudaContext*	mCudaCtx;

	/* Cached device attributes, so threads can query w/o context */
	int             mComputeCapMajor;
	int             mComputeCapMinor;
	int				mIsIntegrated;
	int				mCanMapHost;
	int				mDriverVersion;
	size_t			mTotalMemBytes;
	int				mMultiprocessorCount;
	int				mMaxThreadsPerBlock;
	char			mDeviceName[128];
	int				mSharedMemPerBlock;
	int				mSharedMemPerMultiprocessor;
	int				mClockRate;
	bool			mUsingConcurrentStreams;
	uint32_t		mContextRefCountTls;
#if PX_DEBUG
	volatile PxI32 mPushPopCount;
#endif
};

CUdeviceptr CudaCtxMgr::getMappedDevicePtr(void* pinnedHostBuffer)
{
	CUdeviceptr dPtr = 0;
	PxCUresult result = getCudaContext()->memHostGetDevicePointer(&dPtr, pinnedHostBuffer, 0);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Getting mapped device pointer failed with error code %i!\n", PxI32(result));
	return dPtr;
}


void* CudaCtxMgr::allocDeviceBufferInternal(PxU64 numBytes, const char* filename, PxI32 lineNumber)
{
	numBytes = PxMax(PxU64(1u), numBytes);
	PxScopedCudaLock lock(*this);
	CUdeviceptr ptr;
	PxCUresult result = getCudaContext()->memAlloc(&ptr, numBytes);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Mem allocation failed with error code %i!\n", PxI32(result));
	void* deviceBuffer = reinterpret_cast<void*>(ptr);
#if PX_DEBUG
	if (deviceBuffer)
		mMemTracker.registerMemory(deviceBuffer, true, numBytes, filename, lineNumber);
#else
	PX_UNUSED(filename);
	PX_UNUSED(lineNumber);
#endif
	return deviceBuffer;
}
void* CudaCtxMgr::allocPinnedHostBufferInternal(PxU64 numBytes, const char* filename, PxI32 lineNumber)
{
	numBytes = PxMax(PxU64(1u), numBytes);
	PxScopedCudaLock lock(*this);
	void* pinnedHostBuffer;
	const unsigned int cuMemhostallocDevicemap = 0x02;
	const unsigned int cuMemhostallocPortable = 0x01;
	PxCUresult result = getCudaContext()->memHostAlloc(&pinnedHostBuffer, numBytes, cuMemhostallocDevicemap | cuMemhostallocPortable);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Mem allocation failed with error code %i!\n", PxI32(result));

#if PX_DEBUG
	mMemTracker.registerMemory(pinnedHostBuffer, false, numBytes, filename, lineNumber);
#else
	PX_UNUSED(filename);
	PX_UNUSED(lineNumber);
#endif
	return pinnedHostBuffer;
}

void CudaCtxMgr::freeDeviceBufferInternal(void* deviceBuffer)
{
	if (!deviceBuffer)
		return;
	PxScopedCudaLock lock(*this);
	PxCUresult result = getCudaContext()->memFree(CUdeviceptr(deviceBuffer));
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Mem free failed with error code %i!\n", PxI32(result));
#if PX_DEBUG
	mMemTracker.unregisterMemory(deviceBuffer, true);
#endif
}
void CudaCtxMgr::freePinnedHostBufferInternal(void* pinnedHostBuffer)
{
	if (!pinnedHostBuffer)
		return;
	PxScopedCudaLock lock(*this);
	PxCUresult result = getCudaContext()->memFreeHost(pinnedHostBuffer);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Mem free failed with error code %i!\n", PxI32(result));
#if PX_DEBUG
	mMemTracker.unregisterMemory(pinnedHostBuffer, false);
#endif
}

void CudaCtxMgr::clearDeviceBufferAsyncInternal(void* deviceBuffer, PxU32 numBytes, CUstream stream, PxI32 value)
{
	if (!deviceBuffer)
		return;
	PxScopedCudaLock lock(*this);
	PX_ASSERT(numBytes % 4 == 0);
	PxCUresult result = getCudaContext()->memsetD32Async(CUdeviceptr(deviceBuffer), value, numBytes >> 2, stream);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Mem set failed with error code %i!\n", PxI32(result));
}

void CudaCtxMgr::copyDToHAsyncInternal(void* hostBuffer, const void* deviceBuffer, PxU32 numBytes, CUstream stream)
{
	if (!deviceBuffer || !hostBuffer)
		return;
	PxScopedCudaLock lock(*this);
	PxCUresult result = getCudaContext()->memcpyDtoHAsync(hostBuffer, CUdeviceptr(deviceBuffer), numBytes, stream);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "copyDtoHAsync set failed with error code %i!\n", PxI32(result));
}
void CudaCtxMgr::copyHToDAsyncInternal(void* deviceBuffer, const void* hostBuffer, PxU32 numBytes, CUstream stream)
{
	if (!deviceBuffer || !hostBuffer)
		return;
	PxScopedCudaLock lock(*this);
	PxCUresult result = getCudaContext()->memcpyHtoDAsync(CUdeviceptr(deviceBuffer), hostBuffer, numBytes, stream);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "copyHtoDAsync set failed with error code %i!\n", PxI32(result));
}
void CudaCtxMgr::copyDToDAsyncInternal(void* dstDeviceBuffer, const void* srcDeviceBuffer, PxU32 numBytes, CUstream stream)
{
	if (!srcDeviceBuffer || !dstDeviceBuffer)
		return;
	PxScopedCudaLock lock(*this);
	PxCUresult result = getCudaContext()->memcpyDtoDAsync(CUdeviceptr(dstDeviceBuffer), CUdeviceptr(srcDeviceBuffer), numBytes, stream);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "copyDtoDAsync set failed with error code %i!\n", PxI32(result));
}

void CudaCtxMgr::copyDToHInternal(void* hostBuffer, const void* deviceBuffer, PxU32 numBytes)
{
	if (!deviceBuffer || !hostBuffer)
		return;
	PxScopedCudaLock lock(*this);
	PxCUresult result = getCudaContext()->memcpyDtoH(hostBuffer, CUdeviceptr(deviceBuffer), numBytes);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "copyDtoH set failed with error code %i!\n", PxI32(result));
}
void CudaCtxMgr::copyHToDInternal(void* deviceBuffer, const void* hostBuffer, PxU32 numBytes)
{
	if (!deviceBuffer || !hostBuffer)
		return;
	PxScopedCudaLock lock(*this);
	PxCUresult result = getCudaContext()->memcpyHtoD(CUdeviceptr(deviceBuffer), hostBuffer, numBytes);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "copyHtoD set failed with error code %i!\n", PxI32(result));
}

void CudaCtxMgr::memsetD8AsyncInternal(void* dstDeviceBuffer, const PxU8& value, PxU32 numBytes, CUstream stream)
{
	if (!dstDeviceBuffer)
		return;
	PxScopedCudaLock lock(*this);
	PxCUresult result = getCudaContext()->memsetD8Async(CUdeviceptr(dstDeviceBuffer), value, numBytes, stream);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Memset failed with error code %i!\n", PxI32(result));

}

void CudaCtxMgr::memsetD32AsyncInternal(void* dstDeviceBuffer, const PxU32& value, PxU32 numIntegers, CUstream stream)
{
	if (!dstDeviceBuffer)
		return;
	PxScopedCudaLock lock(*this);
	PxCUresult result = getCudaContext()->memsetD32Async(CUdeviceptr(dstDeviceBuffer), value, numIntegers, stream);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Memset failed with error code %i!\n", PxI32(result));
}


bool CudaCtxMgr::contextIsValid() const
{
	return mIsValid;
}
bool CudaCtxMgr::supportsArchSM10() const
{
	return mIsValid;
}
bool CudaCtxMgr::supportsArchSM11() const
{
	return mIsValid && (mComputeCapMinor >= 1 || mComputeCapMajor > 1);
}
bool CudaCtxMgr::supportsArchSM12() const
{
	return mIsValid && (mComputeCapMinor >= 2 || mComputeCapMajor > 1);
}
bool CudaCtxMgr::supportsArchSM13() const
{
	return mIsValid && (mComputeCapMinor >= 3 || mComputeCapMajor > 1);
}
bool CudaCtxMgr::supportsArchSM20() const
{
	return mIsValid && mComputeCapMajor >= 2;
}
bool CudaCtxMgr::supportsArchSM30() const
{
	return mIsValid && mComputeCapMajor >= 3;
}
bool CudaCtxMgr::supportsArchSM35() const
{
	return mIsValid && ((mComputeCapMajor > 3) || (mComputeCapMajor == 3 && mComputeCapMinor >= 5));
}
bool CudaCtxMgr::supportsArchSM50() const
{
	return mIsValid && mComputeCapMajor >= 5;
}
bool CudaCtxMgr::supportsArchSM52() const
{
	return mIsValid && ((mComputeCapMajor > 5) || (mComputeCapMajor == 5 && mComputeCapMinor >= 2));
}
bool CudaCtxMgr::supportsArchSM60() const
{
	return mIsValid && mComputeCapMajor >= 6;
}

bool CudaCtxMgr::isIntegrated() const
{
	return mIsValid && mIsIntegrated;
}
bool CudaCtxMgr::canMapHostMemory() const
{
	return mIsValid && mCanMapHost;
}
int  CudaCtxMgr::getDriverVersion() const
{
	return mDriverVersion;
}
size_t  CudaCtxMgr::getDeviceTotalMemBytes() const
{
	return mTotalMemBytes;
}
int	CudaCtxMgr::getMultiprocessorCount() const
{
	return mMultiprocessorCount;
}
int CudaCtxMgr::getSharedMemPerBlock() const
{
	return mSharedMemPerBlock;
}
int CudaCtxMgr::getSharedMemPerMultiprocessor() const
{
	return mSharedMemPerMultiprocessor;
}
unsigned int CudaCtxMgr::getMaxThreadsPerBlock() const
{
	return (unsigned int)mMaxThreadsPerBlock;
}
unsigned int CudaCtxMgr::getClockRate() const
{
	return (unsigned int)mClockRate;
}

const char* CudaCtxMgr::getDeviceName() const
{
	if (mIsValid)
	{
		return mDeviceName;
	}
	else
	{
		return "Invalid";
	}
}

CUdevice CudaCtxMgr::getDevice() const
{
	if (mIsValid)
	{
		return mDevHandle;
	}
	else
	{
		return -1;
	}
}

void CudaCtxMgr::setUsingConcurrentStreams(bool value)
{
	mUsingConcurrentStreams = value;
}

bool CudaCtxMgr::getUsingConcurrentStreams() const
{
	return mUsingConcurrentStreams;
}

void CudaCtxMgr::getDeviceMemoryInfo(size_t& free, size_t& total) const
{
	cuMemGetInfo(&free, &total);
}

#define CUT_SAFE_CALL(call)  { CUresult ret = call;	\
		if( CUDA_SUCCESS != ret ) { PX_ASSERT(0); } }

/* If a context is not provided, an ordinal must be given */
CudaCtxMgr::CudaCtxMgr(const PxCudaContextManagerDesc& desc, PxErrorCallback& errorCallback, bool launchSynchronous)
	: mOwnContext(false)
	, mCudaCtx(NULL)
	, mUsingConcurrentStreams(true)
#if PX_DEBUG
	, mPushPopCount(0)
#endif
{
	CUresult status;
	mIsValid = false;
	mDeviceName[0] = 0;

	if (safeDelayImport(errorCallback) == false)
	{
		char buffer[256];
		physx::Pxsnprintf(buffer, 256, "NVIDIA Release %u.%u graphics driver and above is required for GPU acceleration.", NV_DRIVER_MAJOR_VERSION, NV_DRIVER_MINOR_VERSION);
		errorCallback.reportError(PxErrorCode::eDEBUG_WARNING, buffer, PX_FL);
		return;
	}

	if (desc.ctx == 0)
	{
		int flags = CU_CTX_LMEM_RESIZE_TO_MAX | CU_CTX_SCHED_BLOCKING_SYNC | CU_CTX_MAP_HOST;
		class FoundationErrorReporter : public PxErrorCallback
		{
		public:
			FoundationErrorReporter(PxErrorCallback& ec)
			: errorCallback(&ec)
			{
			}

			virtual void reportError(PxErrorCode::Enum code, const char* message, const char* file, int line) PX_OVERRIDE
			{
				errorCallback->reportError(code, message, file, line);
			}

		private:
			PxErrorCallback* errorCallback;
		} foundationErrorReporter(errorCallback);

		int devOrdinal = desc.deviceOrdinal;
		if (desc.deviceOrdinal < 0)
		{
			devOrdinal = PhysXDeviceSettings::getSuggestedCudaDeviceOrdinal(foundationErrorReporter);
		}

		if (devOrdinal < 0)
		{
			errorCallback.reportError(PxErrorCode::eDEBUG_INFO, "No PhysX capable GPU suggested.", PX_FL);
			return;
		}

		status = cuInit(0);
		if (CUDA_SUCCESS != status)
		{
			char buffer[128];
			physx::Pxsnprintf(buffer, 128, "cuInit failed with error code %i", status);
			errorCallback.reportError(PxErrorCode::eDEBUG_WARNING, buffer, PX_FL);
			return;
		}

		{
			status = cuDeviceGet(&mDevHandle, devOrdinal);
			if (CUDA_SUCCESS != status)
			{
				errorCallback.reportError(PxErrorCode::eDEBUG_WARNING, "cuDeviceGet failed",__FILE__,__LINE__);
				return;
			}
			
			status = cuCtxCreate(&mCtx, (unsigned int)flags, mDevHandle);
			if (CUDA_SUCCESS != status)
			{
				const size_t bufferSize = 128;
				char errorMsg[bufferSize];
				physx::Pxsnprintf(errorMsg, bufferSize, "cuCtxCreate failed with error %i.", status);
				errorCallback.reportError(PxErrorCode::eDEBUG_WARNING, errorMsg, PX_FL);
				return;
			}
			mOwnContext = true;
		}
	}
	else
	{
		mCtx = *desc.ctx;
		status = cuCtxGetDevice(&mDevHandle);
		if (CUDA_SUCCESS != status)
		{
			errorCallback.reportError(PxErrorCode::eDEBUG_WARNING, "cuCtxGetDevice failed",__FILE__,__LINE__);
			return;
		}
	}

	// create cuda context wrapper
	mCudaCtx = createCudaContext(mDevHandle, desc.deviceAllocator, launchSynchronous);
	
	// Verify we can at least allocate a CUDA event from this context
	CUevent testEvent;
	if (CUDA_SUCCESS == mCudaCtx->eventCreate(&testEvent, 0))
	{
		mCudaCtx->eventDestroy(testEvent);
	}
	else
	{
		errorCallback.reportError(PxErrorCode::eDEBUG_WARNING, "CUDA context validation failed",__FILE__,__LINE__);
		return;
	}

	status = cuDeviceGetName(mDeviceName, sizeof(mDeviceName), mDevHandle);
	if (CUDA_SUCCESS != status)
	{
		errorCallback.reportError(PxErrorCode::eDEBUG_WARNING, "cuDeviceGetName failed",__FILE__,__LINE__);
		return;
	}

	cuDeviceGetAttribute(&mSharedMemPerBlock, CU_DEVICE_ATTRIBUTE_MAX_SHARED_MEMORY_PER_BLOCK, mDevHandle);
	cuDeviceGetAttribute(&mSharedMemPerMultiprocessor, CU_DEVICE_ATTRIBUTE_MAX_SHARED_MEMORY_PER_MULTIPROCESSOR, mDevHandle);
	cuDeviceGetAttribute(&mClockRate, CU_DEVICE_ATTRIBUTE_CLOCK_RATE, mDevHandle);
	cuDeviceGetAttribute(&mComputeCapMajor, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR, mDevHandle);
	cuDeviceGetAttribute(&mComputeCapMinor, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR, mDevHandle);
	cuDeviceGetAttribute(&mIsIntegrated, CU_DEVICE_ATTRIBUTE_INTEGRATED, mDevHandle);
	cuDeviceGetAttribute(&mCanMapHost, CU_DEVICE_ATTRIBUTE_CAN_MAP_HOST_MEMORY, mDevHandle);
	cuDeviceGetAttribute(&mMultiprocessorCount, CU_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT, mDevHandle);
	cuDeviceGetAttribute(&mMaxThreadsPerBlock, CU_DEVICE_ATTRIBUTE_MAX_THREADS_PER_BLOCK, mDevHandle);

	status = cuDeviceTotalMem((size_t*)&mTotalMemBytes, mDevHandle);
	if (CUDA_SUCCESS != status)
	{
		errorCallback.reportError(PxErrorCode::eDEBUG_WARNING, "cuDeviceTotalMem failed",__FILE__,__LINE__);
		return;
	}

	// minimum compute capability is MIN_SM_MAJOR_VERSION.MIN_SM_MINOR_VERSION
	if ((mComputeCapMajor < MIN_SM_MAJOR_VERSION)	||
		(mComputeCapMajor == MIN_SM_MAJOR_VERSION && mComputeCapMinor < MIN_SM_MINOR_VERSION))
	{
		char buffer[256];
		physx::Pxsnprintf(buffer, 256, "Minimum GPU compute capability %d.%d is required", MIN_SM_MAJOR_VERSION, MIN_SM_MINOR_VERSION);
		errorCallback.reportError(PxErrorCode::eDEBUG_WARNING,buffer,__FILE__,__LINE__);
		return;
	}

	mContextRefCountTls = PxTlsAlloc();
	mIsValid = true;

	// Formally load the CUDA modules, get CUmodule handles
	{
		PxScopedCudaLock lock(*this);
		const PxU32 moduleTableSize = PxGpuGetCudaModuleTableSize();
		void** moduleTable = PxGpuGetCudaModuleTable();
		mCuModules.resize(moduleTableSize, NULL);
		for (PxU32 i = 0 ; i < moduleTableSize ; ++i)
		{
			CUresult ret = CUDA_ERROR_UNKNOWN;

			// Make sure that moduleTable[i] is not null
			if (moduleTable[i])
			{
				ret = mCudaCtx->moduleLoadDataEx(&mCuModules[i], moduleTable[i], 0, NULL, NULL);
			}

			if (ret != CUDA_SUCCESS && ret != CUDA_ERROR_NO_BINARY_FOR_GPU)
			{
				const size_t bufferSize = 256;
				char errorMsg[bufferSize];
				physx::Pxsnprintf(errorMsg, bufferSize, "Failed to load CUDA module data. Cuda error code %i.\n", ret);

				PxGetErrorCallback()->reportError(PxErrorCode::eINTERNAL_ERROR, errorMsg, PX_FL);
				mCuModules[i] = NULL;
			}
		}
	}
}

/* Some driver version mismatches can cause delay import crashes.  Load NVCUDA.dll
 * manually, verify its version number, then allow delay importing to bind all the
 * APIs.
 */
bool CudaCtxMgr::safeDelayImport(PxErrorCallback& errorCallback)
{
#if PX_WIN32 || PX_WIN64
	HMODULE hCudaDriver = LoadLibrary("nvcuda.dll");
#elif PX_LINUX
	void*	hCudaDriver = dlopen("libcuda.so.1", RTLD_NOW);
#endif
	if (!hCudaDriver)
	{
		errorCallback.reportError(PxErrorCode::eINTERNAL_ERROR, "nvcuda.dll not found or could not be loaded.", PX_FL);
		return false;
	}

	typedef CUresult(CUDAAPI * pfnCuDriverGetVersion_t)(int*);
	pfnCuDriverGetVersion_t pfnCuDriverGetVersion = (pfnCuDriverGetVersion_t) GetProcAddress(hCudaDriver, "cuDriverGetVersion");
	if (!pfnCuDriverGetVersion)
	{
		errorCallback.reportError(PxErrorCode::eINTERNAL_ERROR, "cuDriverGetVersion missing in nvcuda.dll.", PX_FL);
		return false;
	}

	#if PX_A64
		CUresult status = cuDriverGetVersion(&mDriverVersion);
	#else
		CUresult status = pfnCuDriverGetVersion(&mDriverVersion);
	#endif

	if (status != CUDA_SUCCESS)
	{
		errorCallback.reportError(PxErrorCode::eINTERNAL_ERROR, "Retrieving CUDA driver version failed.", PX_FL);
		return false;
	}

	// Check that the Cuda toolkit used meets the minimum version
	// If the Cuda toolkit has changed, refer to https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html
	// Make the necessary changes to Cuda toolkit definitions
	PX_COMPILE_TIME_ASSERT(CUDA_VERSION >= MIN_CUDA_VERSION);

	// Check whether Cuda driver version meets the min requirement
	if (mDriverVersion < MIN_CUDA_VERSION)
	{
		char buffer[256];
		physx::Pxsnprintf(buffer, 256, "CUDA driver version is %u, expected driver version is at least %u.", mDriverVersion, MIN_CUDA_VERSION);
		errorCallback.reportError(PxErrorCode::eINTERNAL_ERROR, buffer, __FILE__,__LINE__);
		return false;
	}

	/* Now trigger delay import and API binding */
	status = cuDriverGetVersion(&mDriverVersion);
	if (status != CUDA_SUCCESS)
	{
		errorCallback.reportError(PxErrorCode::eINTERNAL_ERROR, "Failed to bind CUDA API.", PX_FL);
		return false;
	}

	/* Not strictly necessary, but good practice */
#if PX_WIN32 | PX_WIN64
	FreeLibrary(hCudaDriver);
#elif PX_LINUX
	dlclose(hCudaDriver);
#endif

	return true;
}

void CudaCtxMgr::release()
{
	PX_DELETE_THIS;
}

CudaCtxMgr::~CudaCtxMgr()
{
	if (mCudaCtx)
	{
		// unload CUDA modules
		{
			PxScopedCudaLock lock(*this);
			for(PxU32 i = 0; i < mCuModules.size(); i++)
			{
				CUresult ret = mCudaCtx->moduleUnload(mCuModules[i]);
				if(ret != CUDA_SUCCESS)
				{
					char msg[128];
					physx::Pxsnprintf(msg, 128, "Failed to unload CUDA module data, returned %i.", ret);
					PxGetErrorCallback()->reportError(PxErrorCode::eINTERNAL_ERROR, msg, PX_FL);
				}
			}
		}

		mCudaCtx->release();
		mCudaCtx = NULL;
	}

	if (mOwnContext)
	{
		CUT_SAFE_CALL(cuCtxDestroy(mCtx));
	}

	PxTlsFree(mContextRefCountTls);

#if PX_DEBUG
	PX_ASSERT(mPushPopCount == 0);
#endif
}

void CudaCtxMgr::acquireContext()
{
	bool result = tryAcquireContext();
	PX_ASSERT(result);
	PX_UNUSED(result);
}

bool CudaCtxMgr::tryAcquireContext()
{
	// AD: we directly store the counter in the per-thread value (instead of using a pointer-to-value.)
	// Using size_t because we have a pointer's width to play with, so the type will potentially depend on the platform.
	// All the values are initialized to NULL at PxTlsAlloc() and for any newly created thread it will be NULL as well.
	// So even if a thread hits this code for the first time, we know it's zero, and then we start by placing the correct refcount
	// below in the set call.
	size_t refCount = PxTlsGetValue(mContextRefCountTls);

	CUresult result = CUDA_SUCCESS;

#if PX_DEBUG
	result = cuCtxPushCurrent(mCtx);
	PxAtomicIncrement(&mPushPopCount);
#else
	if (refCount == 0)
	{
		result = cuCtxPushCurrent(mCtx);
	}
#endif
	PxTlsSetValue(mContextRefCountTls, ++refCount);

	return result == CUDA_SUCCESS;
}

void CudaCtxMgr::releaseContext()
{
	size_t refCount = PxTlsGetValue(mContextRefCountTls);

#if PX_DEBUG
	CUcontext ctx = 0;
	CUT_SAFE_CALL(cuCtxPopCurrent(&ctx));
	PxAtomicDecrement(&mPushPopCount);
#else
	if (--refCount == 0)
	{
		CUcontext ctx = 0;
		CUT_SAFE_CALL(cuCtxPopCurrent(&ctx));
	}
#endif
	PxTlsSetValue(mContextRefCountTls, refCount);
}

class CudaCtx : public PxCudaContext, public PxUserAllocated
{
private:
	CUresult mLastResult;
	bool mLaunchSynchronous;
	bool mIsInAbortMode;

public:
	CudaCtx(PxDeviceAllocatorCallback* callback, bool launchSynchronous);
	~CudaCtx();

	// PxCudaContext
	void		release()	PX_OVERRIDE PX_FINAL;
	PxCUresult	memAlloc(CUdeviceptr* dptr, size_t bytesize)	PX_OVERRIDE PX_FINAL;
	PxCUresult	memFree(CUdeviceptr dptr)	PX_OVERRIDE PX_FINAL;
	PxCUresult	memHostAlloc(void** pp, size_t bytesize, unsigned int Flags)	PX_OVERRIDE PX_FINAL;
	PxCUresult	memFreeHost(void* p)	PX_OVERRIDE PX_FINAL;
	PxCUresult	memHostGetDevicePointer(CUdeviceptr* pdptr, void* p, unsigned int Flags)	PX_OVERRIDE PX_FINAL;
	PxCUresult	moduleLoadDataEx(CUmodule* module, const void* image, unsigned int numOptions, PxCUjit_option* options, void** optionValues)	PX_OVERRIDE PX_FINAL;
	PxCUresult	moduleGetFunction(CUfunction* hfunc, CUmodule hmod, const char* name)	PX_OVERRIDE PX_FINAL;
	PxCUresult	moduleUnload(CUmodule hmod)	PX_OVERRIDE PX_FINAL;
	PxCUresult	streamCreate(CUstream* phStream, unsigned int Flags)	PX_OVERRIDE PX_FINAL;
	PxCUresult	streamCreateWithPriority(CUstream* phStream, unsigned int flags, int priority)	PX_OVERRIDE PX_FINAL;
	PxCUresult	streamFlush(CUstream hStream)	PX_OVERRIDE PX_FINAL;
	PxCUresult	streamWaitEvent(CUstream hStream, CUevent hEvent, unsigned int Flags)	PX_OVERRIDE PX_FINAL;
	PxCUresult	streamWaitEvent(CUstream hStream, CUevent hEvent)	PX_OVERRIDE PX_FINAL;
	PxCUresult	streamDestroy(CUstream hStream)	PX_OVERRIDE PX_FINAL;
	PxCUresult	streamSynchronize(CUstream hStream)	PX_OVERRIDE PX_FINAL;
	PxCUresult	eventCreate(CUevent* phEvent, unsigned int Flags)	PX_OVERRIDE PX_FINAL;
	PxCUresult	eventRecord(CUevent hEvent, CUstream hStream)	PX_OVERRIDE PX_FINAL;
	PxCUresult	eventQuery(CUevent hEvent)	PX_OVERRIDE PX_FINAL;
	PxCUresult	eventSynchronize(CUevent hEvent)	PX_OVERRIDE PX_FINAL;
	PxCUresult	eventDestroy(CUevent hEvent)	PX_OVERRIDE PX_FINAL;

	PxCUresult launchKernel(
		CUfunction f,
		PxU32 gridDimX, PxU32 gridDimY, PxU32 gridDimZ,
		PxU32 blockDimX, PxU32 blockDimY, PxU32 blockDimZ,
		PxU32 sharedMemBytes,
		CUstream hStream,
		PxCudaKernelParam* kernelParams,
		size_t kernelParamsSizeInBytes,
		void** extra,
		const char* file,
		int line
	)	PX_OVERRIDE PX_FINAL;

	PxCUresult launchKernel(
		CUfunction f,
		PxU32 gridDimX, PxU32 gridDimY, PxU32 gridDimZ,
		PxU32 blockDimX, PxU32 blockDimY, PxU32 blockDimZ,
		PxU32 sharedMemBytes,
		CUstream hStream,
		void** kernelParams,
		void** extra,
		const char* file,
		int line
	)	PX_OVERRIDE PX_FINAL;
	
	PxCUresult memcpyDtoH(void* dstHost, CUdeviceptr srcDevice, size_t ByteCount)	PX_OVERRIDE PX_FINAL;
	PxCUresult memcpyDtoHAsync(void* dstHost, CUdeviceptr srcDevice, size_t ByteCount, CUstream hStream)	PX_OVERRIDE PX_FINAL;
	PxCUresult memcpyHtoD(CUdeviceptr dstDevice, const void* srcHost, size_t ByteCount)	PX_OVERRIDE PX_FINAL;
	PxCUresult memcpyHtoDAsync(CUdeviceptr dstDevice, const void* srcHost, size_t ByteCount, CUstream hStream)	PX_OVERRIDE PX_FINAL;
	PxCUresult memcpyDtoD(CUdeviceptr dstDevice, CUdeviceptr srcDevice, size_t ByteCount)	PX_OVERRIDE PX_FINAL;
	PxCUresult memcpyDtoDAsync(CUdeviceptr dstDevice, CUdeviceptr srcDevice, size_t ByteCount, CUstream hStream)	PX_OVERRIDE PX_FINAL;
	PxCUresult memcpyPeerAsync(CUdeviceptr dstDevice, CUcontext dstContext, CUdeviceptr srcDevice, CUcontext srcContext, size_t ByteCount, CUstream hStream)	PX_OVERRIDE PX_FINAL;
	PxCUresult memsetD32Async(CUdeviceptr dstDevice, unsigned int ui, size_t N, CUstream hStream)	PX_OVERRIDE PX_FINAL;
	PxCUresult memsetD8Async(CUdeviceptr dstDevice, unsigned char uc, size_t N, CUstream hStream)	PX_OVERRIDE PX_FINAL;
	PxCUresult memsetD32(CUdeviceptr dstDevice, unsigned int ui, size_t N)	PX_OVERRIDE PX_FINAL;
	PxCUresult memsetD16(CUdeviceptr dstDevice, unsigned short uh, size_t N)	PX_OVERRIDE PX_FINAL;
	PxCUresult memsetD8(CUdeviceptr dstDevice, unsigned char uc, size_t N)	PX_OVERRIDE PX_FINAL;
	PxCUresult getLastError()	PX_OVERRIDE PX_FINAL	{ return isInAbortMode() ? CUDA_ERROR_OUT_OF_MEMORY : mLastResult; }

	void setAbortMode(bool abort) PX_OVERRIDE PX_FINAL;
	bool isInAbortMode() PX_OVERRIDE PX_FINAL { return mIsInAbortMode; }

	//~PxCudaContext
};

CudaCtx::CudaCtx(PxDeviceAllocatorCallback* callback, bool launchSynchronous)
{
	mLastResult = CUDA_SUCCESS;
	mAllocatorCallback = callback;
	mIsInAbortMode = false;
#if FORCE_LAUNCH_SYNCHRONOUS
	PX_UNUSED(launchSynchronous);
	mLaunchSynchronous = true;
#else
	mLaunchSynchronous = launchSynchronous;
#endif
}

CudaCtx::~CudaCtx()
{

}

void CudaCtx::release()
{
	PX_DELETE_THIS;
}

PxCUresult CudaCtx::memAlloc(CUdeviceptr *dptr, size_t bytesize)
{
	if (mIsInAbortMode)
	{
		*dptr = NULL;
		return mLastResult;
	}

	mLastResult = cuMemAlloc(dptr, bytesize);
#if PX_STOMP_ALLOCATED_MEMORY
	if(*dptr && bytesize > 0)
	{
		cuCtxSynchronize();
		PxCUresult result = memsetD8(*dptr, PxU8(0xcd), bytesize);
		PX_ASSERT(result == CUDA_SUCCESS);
		PX_UNUSED(result);
		cuCtxSynchronize();
	}
#endif
	return mLastResult;
}

PxCUresult CudaCtx::memFree(CUdeviceptr dptr)
{
	if ((void*)dptr == NULL)
		return mLastResult;

 	return cuMemFree(dptr);
}

PxCUresult CudaCtx::memHostAlloc(void** pp, size_t bytesize, unsigned int Flags)
{
	CUresult result = cuMemHostAlloc(pp, bytesize, Flags);
#if PX_STOMP_ALLOCATED_MEMORY
	if(*pp != NULL && bytesize > 0)
	{
		PxMemSet(*pp, PxI32(0xcd), PxU32(bytesize));
	}
#endif
	return result;
}

PxCUresult CudaCtx::memFreeHost(void* p)
{
	return cuMemFreeHost(p);
}

PxCUresult CudaCtx::memHostGetDevicePointer(CUdeviceptr* pdptr, void* p, unsigned int Flags)
{
	if (!p)
	{
		*pdptr = reinterpret_cast<CUdeviceptr>(p);
		return CUDA_SUCCESS;
	}
	return cuMemHostGetDevicePointer(pdptr, p, Flags);
}

PxCUresult CudaCtx::moduleLoadDataEx(CUmodule* module, const void* image, unsigned int numOptions, PxCUjit_option* options, void** optionValues)
{
	return cuModuleLoadDataEx(module, image, numOptions, (CUjit_option*)options, optionValues);
}

PxCUresult CudaCtx::moduleGetFunction(CUfunction* hfunc, CUmodule hmod, const char* name)
{
	return cuModuleGetFunction(hfunc, hmod, name);
}

PxCUresult CudaCtx::moduleUnload(CUmodule hmod)
{
	return cuModuleUnload(hmod);
}

PxCUresult CudaCtx::streamCreate(CUstream* phStream, unsigned int Flags)
{
	if (mIsInAbortMode)
	{
		*phStream = NULL;
		return mLastResult;
	}

#if !USE_DEFAULT_CUDA_STREAM
	mLastResult = cuStreamCreate(phStream, Flags);
#else
	PX_UNUSED(Flags);
	*phStream = CUstream(CU_STREAM_DEFAULT);
	mLastResult = CUDA_SUCCESS;
#endif

	return mLastResult;
}

PxCUresult CudaCtx::streamCreateWithPriority(CUstream* phStream, unsigned int flags, int priority)
{
	if (mIsInAbortMode)
	{
		*phStream = NULL;
		return mLastResult;
	}

#if !USE_DEFAULT_CUDA_STREAM
	mLastResult = cuStreamCreateWithPriority(phStream, flags, priority);
#else
	PX_UNUSED(flags);
	PX_UNUSED(priority);
	*phStream = CUstream(CU_STREAM_DEFAULT);
	mLastResult = CUDA_SUCCESS;
#endif

	return mLastResult;
}

PxCUresult CudaCtx::streamFlush(CUstream hStream)
{
	if (mIsInAbortMode)
		return mLastResult;

	// AD: don't remember the error, because this can return CUDA_ERROR_NOT_READY which is not really an error.
	// We just misuse streamquery to push the buffer anyway.
	return cuStreamQuery(hStream);
}

PxCUresult CudaCtx::streamWaitEvent(CUstream hStream, CUevent hEvent, unsigned int Flags)
{
	if (mIsInAbortMode)
		return mLastResult;

	mLastResult = cuStreamWaitEvent(hStream, hEvent, Flags);
	return mLastResult;
}

PxCUresult CudaCtx::streamWaitEvent(CUstream hStream, CUevent hEvent)
{
	return streamWaitEvent(hStream, hEvent, 0);
}

PxCUresult CudaCtx::streamDestroy(CUstream hStream)
{
	PX_UNUSED(hStream);
#if !USE_DEFAULT_CUDA_STREAM
	if (hStream == NULL)
		return mLastResult;
	return cuStreamDestroy(hStream);
#else
	return CUDA_SUCCESS;
#endif
}

PxCUresult CudaCtx::streamSynchronize(CUstream hStream)
{
	if (mIsInAbortMode)
		return mLastResult;

	mLastResult = cuStreamSynchronize(hStream);
	return mLastResult;
}

PxCUresult CudaCtx::eventCreate(CUevent* phEvent, unsigned int Flags)
{
	if (mIsInAbortMode)
	{
		*phEvent = NULL;
		return mLastResult;
	}

	mLastResult = cuEventCreate(phEvent, Flags);
	return mLastResult;
}

PxCUresult CudaCtx::eventRecord(CUevent hEvent, CUstream hStream)
{
	if (mIsInAbortMode)
		return mLastResult;

	mLastResult = cuEventRecord(hEvent, hStream);
	return mLastResult;
}

PxCUresult CudaCtx::eventQuery(CUevent hEvent)
{
	if (mIsInAbortMode)
		return mLastResult;

	mLastResult = cuEventQuery(hEvent);
	return mLastResult;
}

PxCUresult CudaCtx::eventSynchronize(CUevent hEvent)
{
	if (mIsInAbortMode)
		return mLastResult;

	mLastResult = cuEventSynchronize(hEvent);
	return mLastResult;
}

PxCUresult CudaCtx::eventDestroy(CUevent hEvent)
{
	if (hEvent == NULL)
		return mLastResult;

	return cuEventDestroy(hEvent);
}

PxCUresult CudaCtx::launchKernel(
	CUfunction f,
	PxU32 gridDimX, PxU32 gridDimY, PxU32 gridDimZ,
	PxU32 blockDimX, PxU32 blockDimY, PxU32 blockDimZ,
	PxU32 sharedMemBytes,
	CUstream hStream,
	PxCudaKernelParam* kernelParams,
	size_t kernelParamsSizeInBytes,
	void** extra,
	const char* file,
	int line
)
{
	if (mIsInAbortMode)
		return mLastResult;

	//We allow CUDA_ERROR_INVALID_VALUE to be non-terminal error as this is sometimes  hit
	//when we launch an empty block
	if (mLastResult == CUDA_SUCCESS || mLastResult == CUDA_ERROR_INVALID_VALUE)
	{
		const uint32_t kernelParamCount = (uint32_t)(kernelParamsSizeInBytes / sizeof(PxCudaKernelParam));
		PX_ALLOCA(kernelParamsLocal, void*, kernelParamCount);
		for (unsigned int paramIdx = 0u; paramIdx < kernelParamCount; paramIdx++)
		{
			kernelParamsLocal[paramIdx] = kernelParams[paramIdx].data;
		}
		mLastResult = cuLaunchKernel(
			f,
			gridDimX, gridDimY, gridDimZ,
			blockDimX, blockDimY, blockDimZ,
			sharedMemBytes,
			hStream,
			kernelParamsLocal,
			extra
		);

		if (mLaunchSynchronous)
		{
			mLastResult = cuStreamSynchronize(hStream);
			if (mLastResult != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, file, line, "Launch failed!! Error: %i\n", mLastResult);
		}

		PX_ASSERT(mLastResult == CUDA_SUCCESS || mLastResult == CUDA_ERROR_INVALID_VALUE);
	}

	return mLastResult;
}

PxCUresult CudaCtx::launchKernel(
	CUfunction f,
	PxU32 gridDimX, PxU32 gridDimY, PxU32 gridDimZ,
	PxU32 blockDimX, PxU32 blockDimY, PxU32 blockDimZ,
	PxU32 sharedMemBytes,
	CUstream hStream,
	void** kernelParams,
	void** extra,
	const char* file,
	int line
)
{
	if (mIsInAbortMode)
		return mLastResult;

	//We allow CUDA_ERROR_INVALID_VALUE to be non-terminal error as this is sometimes  hit
	//when we launch an empty block
	if (mLastResult == CUDA_SUCCESS || mLastResult == CUDA_ERROR_INVALID_VALUE)
	{
		mLastResult = cuLaunchKernel(
			f,
			gridDimX, gridDimY, gridDimZ,
			blockDimX, blockDimY, blockDimZ,
			sharedMemBytes,
			hStream,
			kernelParams,
			extra
		);

		if (mLaunchSynchronous)
		{
			mLastResult = cuStreamSynchronize(hStream);
			if (mLastResult != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, file, line, "Launch failed!! Error: %i\n", mLastResult);
		}

		PX_ASSERT(mLastResult == CUDA_SUCCESS || mLastResult == CUDA_ERROR_INVALID_VALUE);
	}

	return mLastResult;
}

PxCUresult CudaCtx::memcpyDtoH(void* dstHost, CUdeviceptr srcDevice, size_t ByteCount)
{
	if (mIsInAbortMode)
		return mLastResult;

	if (ByteCount > 0)
		mLastResult = cuMemcpyDtoH(dstHost, srcDevice, ByteCount);
	
	if (mLastResult != CUDA_SUCCESS)
	{
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "copyDToH failed with error code %i!\n", PxI32(mLastResult));
	}

	return mLastResult;
	
}

PxCUresult CudaCtx::memcpyDtoHAsync(void* dstHost, CUdeviceptr srcDevice, size_t ByteCount, CUstream hStream)
{
	if (mIsInAbortMode)
		return mLastResult;

	if (ByteCount > 0)
	{
		mLastResult = cuMemcpyDtoHAsync(dstHost, srcDevice, ByteCount, hStream);
		if (mLaunchSynchronous)
		{
			if (mLastResult != CUDA_SUCCESS)
			{
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "memcpyDtoHAsync invalid parameters!! Error: %i\n", mLastResult);
			}
			mLastResult = cuStreamSynchronize(hStream);
			if (mLastResult != CUDA_SUCCESS)
			{
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "memcpyDtoHAsync failed!! Error: %i\n", mLastResult);
			}
		}
	}
	return mLastResult;
}

PxCUresult CudaCtx::memcpyHtoD(CUdeviceptr dstDevice, const void* srcHost, size_t ByteCount)
{
	if (mIsInAbortMode)
		return mLastResult;

	if (ByteCount > 0)
	{
		mLastResult = cuMemcpyHtoD(dstDevice, srcHost, ByteCount);
		if (mLastResult != CUDA_SUCCESS)
		{
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "memcpyHtoD invalid parameters!! %i\n", mLastResult);
		}
	}
	return mLastResult;
}

PxCUresult CudaCtx::memcpyHtoDAsync(CUdeviceptr dstDevice, const void* srcHost, size_t ByteCount, CUstream hStream)
{
	if (mIsInAbortMode)
		return mLastResult;

	if (ByteCount > 0)
	{
		mLastResult = cuMemcpyHtoDAsync(dstDevice, srcHost, ByteCount, hStream);
		if (mLaunchSynchronous)
		{
			if (mLastResult != CUDA_SUCCESS)
			{
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "memcpyHtoDAsync invalid parameters!! Error: %i\n", mLastResult);
			}
			mLastResult = cuStreamSynchronize(hStream);
			if (mLastResult != CUDA_SUCCESS)
			{
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "memcpyHtoDAsync failed!! Error: %i\n", mLastResult);
			}
		}
	}
	return mLastResult;
}

PxCUresult CudaCtx::memcpyDtoDAsync(CUdeviceptr dstDevice, CUdeviceptr srcDevice, size_t ByteCount, CUstream hStream)
{
	if (mIsInAbortMode)
		return mLastResult;

	if (ByteCount > 0)
	{
		mLastResult = cuMemcpyDtoDAsync(dstDevice, srcDevice, ByteCount, hStream);
		if (mLaunchSynchronous)
		{
			if (mLastResult != CUDA_SUCCESS)
			{
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "memcpyDtoDAsync invalid parameters!! Error: %i\n", mLastResult);
			}
			mLastResult = cuStreamSynchronize(hStream);
			if (mLastResult != CUDA_SUCCESS)
			{
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "memcpyDtoDAsync failed!! Error: %i\n", mLastResult);
			}
		}
	}
	return mLastResult;
}

PxCUresult CudaCtx::memcpyDtoD(CUdeviceptr dstDevice, CUdeviceptr srcDevice, size_t ByteCount)
{
	if (mIsInAbortMode)
		return mLastResult;

	if (ByteCount > 0)
	{
		mLastResult = cuMemcpyDtoD(dstDevice, srcDevice, ByteCount);
		// synchronize to avoid race conditions. 
		// https://docs.nvidia.com/cuda/cuda-driver-api/api-sync-behavior.html#api-sync-behavior__memcpy
		mLastResult = cuStreamSynchronize(0);
		if (mLastResult != CUDA_SUCCESS)
		{
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "memcpyDtoD invalid parameters!! Error: %i\n", mLastResult);
		}
	}
	return mLastResult;
}

PxCUresult CudaCtx::memcpyPeerAsync(CUdeviceptr dstDevice, CUcontext dstContext, CUdeviceptr srcDevice, CUcontext srcContext, size_t ByteCount, CUstream hStream)
{
	if (mIsInAbortMode)
		return mLastResult;

	return cuMemcpyPeerAsync(dstDevice, dstContext, srcDevice, srcContext, ByteCount, hStream);
}

PxCUresult CudaCtx::memsetD32Async(CUdeviceptr dstDevice, unsigned int ui, size_t N, CUstream hStream)
{
	if (mIsInAbortMode)
		return mLastResult;

	if (N > 0)
	{
		mLastResult = cuMemsetD32Async(dstDevice, ui, N, hStream);
		if (mLaunchSynchronous)
		{
			PX_ASSERT(mLastResult == CUDA_SUCCESS);
			if (mLastResult != CUDA_SUCCESS)
			{
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "memsetD32Async invalid parameters!! Error: %i\n", mLastResult);
			}
			mLastResult = cuStreamSynchronize(hStream);
			if (mLastResult != CUDA_SUCCESS)
			{
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "memsetD32Async failed!! Error: %i\n", mLastResult);
			}
		}
	}
	return mLastResult;
}

PxCUresult CudaCtx::memsetD8Async(CUdeviceptr dstDevice, unsigned char uc, size_t N, CUstream hStream)
{
	if (mIsInAbortMode)
		return mLastResult;

	if (N > 0)
	{
		mLastResult = cuMemsetD8Async(dstDevice, uc, N, hStream);
		if (mLaunchSynchronous)
		{
			if (mLastResult!= CUDA_SUCCESS)
			{
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "cuMemsetD8Async invalid parameters!! Error: %i\n", mLastResult);
			}
			mLastResult = cuStreamSynchronize(hStream);
			if (mLastResult != CUDA_SUCCESS)
			{
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "cuMemsetD8Async failed!! Error: %i\n", mLastResult);
			}
		}
	}
	return mLastResult;
}

PxCUresult CudaCtx::memsetD32(CUdeviceptr dstDevice, unsigned int ui, size_t N)
{
	if (mIsInAbortMode)
		return mLastResult;

	if (N > 0)
	{
		mLastResult = cuMemsetD32(dstDevice, ui, N);
		if (mLastResult != CUDA_SUCCESS)
		{
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "memsetD32 failed!! Error: %i\n", mLastResult);
			return mLastResult;
		}

		// synchronize to avoid race conditions.
		// https://docs.nvidia.com/cuda/cuda-driver-api/api-sync-behavior.html#api-sync-behavior__memset
		mLastResult = cuStreamSynchronize(0);
		if (mLastResult != CUDA_SUCCESS)
		{
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "memsetD32 failed!! Error: %i\n", mLastResult);
		}
	}
	return mLastResult;
}

PxCUresult CudaCtx::memsetD16(CUdeviceptr dstDevice, unsigned short uh, size_t N)
{
	if (mIsInAbortMode)
		return mLastResult;

	if (N > 0)
	{
		cuMemsetD16(dstDevice, uh, N);
		// synchronize to avoid race conditions. 
		// https://docs.nvidia.com/cuda/cuda-driver-api/api-sync-behavior.html#api-sync-behavior__memset
		mLastResult = cuStreamSynchronize(0);
		if (mLastResult != CUDA_SUCCESS)
		{
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "memsetD16 failed!! Error: %i\n", mLastResult);
		}
	}
	return mLastResult;
}

PxCUresult CudaCtx::memsetD8(CUdeviceptr dstDevice, unsigned char uc, size_t N)
{
	if (mIsInAbortMode)
		return mLastResult;

	if (N > 0)
	{
		cuMemsetD8(dstDevice, uc, N);
		// synchronize to avoid race conditions. 
		// https://docs.nvidia.com/cuda/cuda-driver-api/api-sync-behavior.html#api-sync-behavior__memset
		mLastResult = cuStreamSynchronize(0);
		if (mLastResult != CUDA_SUCCESS)
		{
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "memsetD8 failed!! Error: %i\n", mLastResult);
		}
	}
	return mLastResult;
}

void CudaCtx::setAbortMode(bool abort)
{
	mIsInAbortMode = abort;
	
	if ((abort == false) && (mLastResult == CUDA_ERROR_OUT_OF_MEMORY))
	{	
		mLastResult = CUDA_SUCCESS;
	}
}

PxCudaContext* createCudaContext(CUdevice device, PxDeviceAllocatorCallback* callback, bool launchSynchronous)
{
	PX_UNUSED(device);
	return PX_NEW(CudaCtx)(callback, launchSynchronous);
}

#if PX_SUPPORT_GPU_PHYSX

PxCudaContextManager* createCudaContextManager(const PxCudaContextManagerDesc& desc, PxErrorCallback& errorCallback, bool launchSynchronous)
{
	return PX_NEW(CudaCtxMgr)(desc, errorCallback, launchSynchronous);
}

#endif

} // end physx namespace


