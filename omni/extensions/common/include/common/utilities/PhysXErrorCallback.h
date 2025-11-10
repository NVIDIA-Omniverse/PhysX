// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/logging/Log.h>

#include <common/utilities/Utilities.h>

#include <PxPhysicsAPI.h>
#include <cudamanager/PxCudaContext.h>
#include <omni/physx/IPhysx.h>

// AD: hardcode this here to avoid pulling in CUDA headers.
#define CUDA_OUT_OF_MEMORY 2

class CarbPhysXErrorCallback : public ::physx::PxErrorCallback
{
public:
    CarbPhysXErrorCallback()
        : mErrorEventType(0),
          mNumErrors(0),
          mMaxNumErrors(0xffffffff)
#if PX_SUPPORT_GPU_PHYSX        
        , mCudaContextManager(nullptr)
#endif        
    {
    }

    ~CarbPhysXErrorCallback() override
    {
        CARB_ASSERT(mEventStream.get() == nullptr);
    }

    void resetErrorCounter()
    {
        mNumErrors = 0;
    }

    void setMaxNumErrors(uint32_t maxNumErrors) { mMaxNumErrors = maxNumErrors; }

    void reportError(::physx::PxErrorCode::Enum code, const char* message, const char* file, int line) override
    {
        if((code == ::physx::PxErrorCode::eNO_ERROR) || (code & ::physx::PxErrorCode::eDEBUG_INFO))
        {
            CARB_LOG_INFO("PhysX info: %s, FILE %s, LINE %d", message, file, line);
        }
        else if(code & (::physx::PxErrorCode::eDEBUG_WARNING | ::physx::PxErrorCode::ePERF_WARNING))
        {
            // A.B. ignore this msg its valid
            if (!strstr(message,"Filtering: Pair with no contact/trigger reports detected"))
                CARB_LOG_WARN("PhysX warning: %s, FILE %s, LINE %d", message, file, line);
        }
        else if(code & ::physx::PxErrorCode::eABORT)
        {
            CARB_LOG_ERROR("PhysX ABORT error: %s, FILE %s, LINE %d", message, file, line);
            sendTooManyErrorsStopSimulationEvent();
        }
        else
        {
            std::string customErrMsg;
            if (!getCustomErrMsg(message, customErrMsg))
            {
                customErrMsg = message;
            }

            CARB_LOG_ERROR("PhysX error: %s, FILE %s, LINE %d", customErrMsg.c_str(), file, line);

            if (mEventStream)
            {
                if(mNumErrors < mMaxNumErrors)
                {
                    sendErrorEvent(mEventStream, mErrorEventType, std::make_pair("errorString", customErrMsg.c_str()));
                }
                else if(mNumErrors == mMaxNumErrors)
                {
                    sendTooManyErrorsStopSimulationEvent();
                }
#if PX_SUPPORT_GPU_PHYSX
                // AD: this is a bit more complex than it needs to be. We ignore out-of-memory errors here because these might
                // happen even before we have a valid scene, at which point we have a fallback to CPU. This means we should
                // not stop the timeline - which happens in the code below.
                //
                // So in case we run out of GPU memory we spam the console a bit more, but we'll catch it at the end of the 
                // step using the abort error which is handled above and the timeline is stopped. In case of a different
                // error (crash, etc) we still have the error handling as-is below.

                if (mCudaContextManager)
                {
                    physx::PxCUresult error = mCudaContextManager->getCudaContext()->getLastError();
                    if ((error > 0) && (error != CUDA_OUT_OF_MEMORY))
                    {
                        CARB_LOG_ERROR(
                            "Cuda context manager error, simulation will be stopped and new cuda context manager will be created.");
                        sendErrorEvent(mEventStream, omni::physx::ePhysxCudaError,
                                       std::make_pair("errorString", customErrMsg.c_str()));
                    }
                }
#endif
            }
            mNumErrors++;
        }
    }

    void setEventStream(carb::events::IEventStreamPtr eventStream)
    {
        mEventStream = eventStream;
    }

#if PX_SUPPORT_GPU_PHYSX            
    void setCudaContextManager(::physx::PxCudaContextManager* man)
    {
        mCudaContextManager = man;
    }
#endif

    void invalidateEventStream()
    {
        mEventStream = nullptr;
    }

    void setErrorEventType(carb::events::EventType errorEventType)
    {
        mErrorEventType = errorEventType;
    }

private:

    bool getCustomErrMsg(const std::string& msg, std::string& customErrMsg);

    void sendTooManyErrorsStopSimulationEvent()
    {
        // AD: we should figure out if we already had a scene here in case we failed.

        char buffer[200];
        snprintf(buffer, sizeof(buffer),
                "Exceeding maximum number of PhysX errors (%d). "
                "omni.physx will ignore logging subsequent errors to user interface until simulation stop",
                mMaxNumErrors);
        CARB_LOG_ERROR("PhysX error: %s", buffer);
        sendErrorEvent(mEventStream, omni::physx::ePhysxTooManyErrors, std::make_pair("errorString", (const char*)buffer));
    }

    carb::events::IEventStreamPtr   mEventStream;
    carb::events::EventType         mErrorEventType;
    uint32_t                        mNumErrors;
    uint32_t                        mMaxNumErrors;
#if PX_SUPPORT_GPU_PHYSX            
    ::physx::PxCudaContextManager*  mCudaContextManager;
#endif
};
