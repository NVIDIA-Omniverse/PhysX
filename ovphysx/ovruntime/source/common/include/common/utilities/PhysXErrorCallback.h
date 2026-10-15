// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-RUNTIME-ERROR-001
 * @covers AC-3 AC-4
 */

#pragma once

#include <carb/logging/Log.h>

#include <common/utilities/CoreUtilities.h> // sendErrorEvent

#include <PxPhysicsAPI.h>
#include <cudamanager/PxCudaContext.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/RuntimeError.h>

// AD: hardcode this here to avoid pulling in CUDA headers.
#define CUDA_OUT_OF_MEMORY 2

class CarbPhysXErrorCallback : public ::physx::PxErrorCallback
{
public:
    CarbPhysXErrorCallback()
        : mErrorEventType(0),
          mNumErrors(0),
          mMaxNumErrors(0xffffffff)
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
            // Capture only the SDK cause; the log also carries its source file and line.
            omni::physx::recordRuntimeError(message);
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

            omni::physx::recordRuntimeError(customErrMsg.c_str());
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

            }
            mNumErrors++;
        }
    }

    void setEventStream(carb::events::IEventStreamPtr eventStream)
    {
        mEventStream = eventStream;
    }



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
        if (mEventStream)
        {
            sendErrorEvent(mEventStream, omni::physx::ePhysxTooManyErrors,
                           std::make_pair("errorString", (const char*)buffer));
        }
    }

    carb::events::IEventStreamPtr   mEventStream;
    carb::events::EventType         mErrorEventType;
    uint32_t                        mNumErrors;
    uint32_t                        mMaxNumErrors;

};
