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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "NpCheck.h"
#include "NpScene.h"

using namespace physx;

NpReadCheck::NpReadCheck(const NpScene* scene, const char* functionName) : mScene(scene), mName(functionName), mErrorCount(0)
{
	if(mScene)
	{
		if(!mScene->startRead())
		{
			if(mScene->getScScene().getFlags() & PxSceneFlag::eREQUIRE_RW_LOCK)
			{
				PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, 
					"An API read call (%s) was made from thread %d but PxScene::lockRead() was not called first, note that "
					"when PxSceneFlag::eREQUIRE_RW_LOCK is enabled all API reads and writes must be "
					"wrapped in the appropriate locks.", mName, PxU32(PxThread::getId()));
			}
			else
			{
				PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, 
					"Overlapping API read and write call detected during %s from thread %d! Note that read operations to "
					"the SDK must not be overlapped with write calls, else the resulting behavior is undefined.", mName, PxU32(PxThread::getId()));
			}
		}

		// Record the NpScene read/write error counter which is
		// incremented any time a NpScene::startWrite/startRead fails
		// (see destructor for additional error checking based on this count)
		mErrorCount = mScene->getReadWriteErrorCount();
	}
}

NpReadCheck::~NpReadCheck()
{
	if(mScene)
	{
		// By checking if the NpScene::mConcurrentErrorCount has been incremented
		// we can detect if an erroneous read/write was performed during 
		// this objects lifetime. In this case we also print this function's
		// details so that the user can see which two API calls overlapped
		if(mScene->getReadWriteErrorCount() != mErrorCount && !(mScene->getScScene().getFlags() & PxSceneFlag::eREQUIRE_RW_LOCK))
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, 
				"Leaving %s on thread %d, an API overlapping write on another thread was detected.", mName, PxU32(PxThread::getId()));
		}

		mScene->stopRead();
	}
}

NpWriteCheck::NpWriteCheck(NpScene* scene, const char* functionName, bool allowReentry) : mScene(scene), mName(functionName), mAllowReentry(allowReentry), mErrorCount(0)
{
	if(mScene)
	{
		switch(mScene->startWrite(mAllowReentry))
		{
		case NpScene::StartWriteResult::eOK:
			break;
		case NpScene::StartWriteResult::eNO_LOCK:
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL,
				"An API write call (%s) was made from thread %d but PxScene::lockWrite() was not called first, note that "
				"when PxSceneFlag::eREQUIRE_RW_LOCK is enabled all API reads and writes must be "
				"wrapped in the appropriate locks.", mName, PxU32(PxThread::getId()));
			break;
		case NpScene::StartWriteResult::eRACE_DETECTED:
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL,
				"Concurrent API write call or overlapping API read and write call detected during %s from thread %d! "
				"Note that write operations to the SDK must be sequential, i.e., no overlap with "
				"other write or read calls, else the resulting behavior is undefined. "
				"Also note that API writes during a callback function are not permitted.", mName, PxU32(PxThread::getId()));
			break;

		case NpScene::StartWriteResult::eIN_FETCHRESULTS:
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL,
				"Illegal write call detected in %s from thread %d during split fetchResults! "
				"Note that write operations to the SDK are not permitted between the start of fetchResultsStart() and end of fetchResultsFinish(). "
				"Behavior will be undefined. ", mName, PxU32(PxThread::getId()));
			break;
		}

		// Record the NpScene read/write error counter which is
		// incremented any time a NpScene::startWrite/startRead fails
		// (see destructor for additional error checking based on this count)
		mErrorCount = mScene->getReadWriteErrorCount();
	}
}

NpWriteCheck::~NpWriteCheck()
{
	if(mScene)
	{
		// By checking if the NpScene::mConcurrentErrorCount has been incremented
		// we can detect if an erroneous read/write was performed during 
		// this objects lifetime. In this case we also print this function's
		// details so that the user can see which two API calls overlapped
		if(mScene->getReadWriteErrorCount() != mErrorCount && !(mScene->getScScene().getFlags() & PxSceneFlag::eREQUIRE_RW_LOCK))
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, 
			"Leaving %s on thread %d, an overlapping API read or write by another thread was detected.", mName, PxU32(PxThread::getId()));
		}

		mScene->stopWrite(mAllowReentry);
	}
}
