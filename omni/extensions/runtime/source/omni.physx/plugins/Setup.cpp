// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/Defines.h>
#include "PhysXTools.h"
#include "Setup.h"
#include <common/utilities/PhysXErrorCallback.h>
#include <common/utilities/MemoryMacros.h>

#include "omnipvd/PxOmniPvd.h"
#if !CARB_AARCH64
    #include "OmniPvdWriter.h"
    #include "OmniPvdFileWriteStream.h"
#endif

#include "PhysXScene.h"
#include "PhysXDefines.h"
#include "PhysXUSDProperties.h"
#include "OmniPhysX.h"
#include "ContactReport.h"
#include "SceneMultiGPUMode.h"

#include "internal/Internal.h"
#include "CookingDataAsync.h"
#include "MeshCache.h"
#include "ConeCylinderConvexMesh.h"
#include "attachment/PhysXPoissonSampling.h"
#include "particles/PhysXParticlePost.h"

#include "PhysXCarbCPUDispatcher.h"

#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>

#include <omni/physx/IPhysxSettings.h>
#include <private/omni/physx/IPhysxCookingServicePrivate.h>
#include <omni/physx/IPhysxFoundation.h>


using namespace physx;
using namespace omni::physx::internal;
using namespace omni::physx;
using namespace omni::physx::usdparser;

#if USE_PHYSX_GPU
#include <cuda.h>
#endif

#if !CARB_PLATFORM_WINDOWS
#define sprintf_s snprintf
#endif


static const ::physx::PxF32 tConeOrCylinderWidth = 2.0f;
static const ::physx::PxF32 tConeOrCylinderRadius = 1.0f;

static const ::physx::PxU32 tCylinderNumCirclePoints = 30;  // with 32 it crashes on GPU
static const ::physx::PxU32 tConeNumCirclePoints = 30;


class OmniPhysxProfileCallback : public PxProfilerCallback
{
    virtual void* zoneStart(const char* eventName, bool detached, uint64_t contextId)
    {
        if (!detached)
            CARB_PROFILE_BEGIN(kPhysicsProfilerMask, "%s",eventName);
        return nullptr;
    }

    virtual void zoneEnd(void* profilerData, const char* eventName, bool detached, uint64_t contextId)
    {
        if (!detached)
            CARB_PROFILE_END(kPhysicsProfilerMask);
    }

    virtual void recordData(int32_t value, const char* valueName, uint64_t contextId)
    {
        CARB_PROFILE_VALUE(value, kPhysicsProfilerMask, "%s (%llu)", valueName, contextId);
    }

    virtual void recordData(float value, const char* valueName, uint64_t contextId)
    {
        CARB_PROFILE_VALUE(value, kPhysicsProfilerMask, "%s (%llu)", valueName, contextId);
    }

    virtual void recordFrame(const char* name, uint64_t contextId)
    {
        CARB_PROFILE_FRAME(kPhysicsProfilerMask, "Frame: %s (%llu)", name, contextId);
    }

} gProfilerCallback;


struct OmniPvdRecordingTime
{
    OmniPvdRecordingTime()
    {
        mWasSet = false;
    }

    bool wasSet()
    {
        return mWasSet;
    }

    bool isSame(OmniPvdRecordingTime& ts)
    {
        return ((year == ts.year)
            && (month == ts.month)
            && (day == ts.day)
            && (hour == ts.hour)
            && (minute == ts.minute)
            && (second == ts.second));
    }

    void setWithDiffCounterIncreaseIfSame(OmniPvdRecordingTime& ts)
    {
        if (wasSet()) {
            if (isSame(ts))
            {
                diffCounter++; // increase the diffcounter if the time stamps are the same
            }
            else
            {
                diffCounter = 1;
            }
        }
        else
        {
            diffCounter = 1;
        }

        year = ts.year;
        month = ts.month;
        day = ts.day;
        hour = ts.hour;
        minute = ts.minute;
        second = ts.second;

        mWasSet = true;
    }

    uint32_t year;
    uint32_t month;
    uint32_t day;
    uint32_t hour;
    uint32_t minute;
    uint32_t second;

    uint32_t diffCounter;

    bool mWasSet;
};

#ifdef OMNI_PVD_WIN

void getLocalOmniPvdTime(OmniPvdRecordingTime& timeStamp)
{
    SYSTEMTIME lt;
    GetLocalTime(&lt);
    // Maybe to use UTC at some point?
    //GetSystemTime(&lt);

    timeStamp.year = lt.wYear;
    timeStamp.month = lt.wMonth;
    timeStamp.day = lt.wDay;
    timeStamp.hour = lt.wHour;
    timeStamp.minute = lt.wMinute;
    timeStamp.second = lt.wSecond;
}

#else

void getLocalOmniPvdTime(OmniPvdRecordingTime& timeStamp)
{
    struct timeval tv;
    struct tm *tm;
    gettimeofday(&tv, NULL);
    // Maybe to use UTC at some point?
    tm = localtime(&tv.tv_sec);

    timeStamp.year = tm->tm_year + 1900;
    timeStamp.month = tm->tm_mon + 1;
    timeStamp.day = tm->tm_mday;
    timeStamp.hour = tm->tm_hour;
    timeStamp.minute = tm->tm_min;
    timeStamp.second = tm->tm_sec;
}

#endif

static OmniPvdRecordingTime gLastOmniPvdTime;

//void logFunc(char *logLine)
//{
//    std::cout << logLine << std::endl;
//}

namespace omni
{
    namespace physx
    {
        PhysXSetup::PhysXSetup() : mDefaultScene(nullptr)
        {
            carb::Framework* framework = carb::getFramework();
            // Make sure that foundation is acquired first so that its carbOnPluginStartup loading PhysXGPU_64.dll is called
            mPhysxFoundation = carb::getCachedInterface<omni::physx::IPhysxFoundation>();
            mCookingServicePrivate = carb::getCachedInterface<IPhysxCookingServicePrivate>();
            mCookingService = carb::getCachedInterface<IPhysxCookingService>();
            omni::physx::PhysxCookingAsyncContextParameters contextParams;
            contextParams.contextName = {"omni.physx", strlen("omni.physx")};
            mCookingServiceContext = mCookingService->createAsyncContext(contextParams);

            mErrorCallback = new CarbPhysXErrorCallback();


            // Subscribe to changes to both kSettingPVDEnabled and kOmniPvdOutputEnabled so we can regenerate the SDK
            // connection to the legacy PVD or OmniPVD in case settings change
            {
                mISettings = carb::getCachedInterface<carb::settings::ISettings>();

                carb::dictionary::SubscriptionId* subID;
                auto legacyPvdEnabledChangedLambda = [](const carb::dictionary::Item* changedItem,
                                                        carb::dictionary::ChangeEventType eventType, void* userData) {
                    if (!OmniPhysX::isStarted()) return;
                    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
                    OmniCachedSettings& cachedSettings = omniPhysX.getCachedSettings();
                    PhysXSetup *ptr = static_cast<PhysXSetup*>(userData);
                    bool setting = omniPhysX.getISettings()->getAsBool(kSettingPVDEnabled);
                    if (setting != (bool)ptr->getPvd())
                    {

                        ptr->changePVDSettings(setting, (bool)ptr->getOmniPvd());
                    }
                };
                subID = mISettings->subscribeToNodeChangeEvents(kSettingPVDEnabled, legacyPvdEnabledChangedLambda, this);
                mSubscribedSettings.push_back(subID);

                auto omniPvdEnabledChangedLambda = [](const carb::dictionary::Item* changedItem,
                                                      carb::dictionary::ChangeEventType eventType, void* userData) {
                    if (!OmniPhysX::isStarted()) return;
                    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
                    OmniCachedSettings& cachedSettings = omniPhysX.getCachedSettings();
                    PhysXSetup *ptr = static_cast<PhysXSetup*>(userData);
                    bool setting = omniPhysX.getISettings()->getAsBool(kOmniPvdOutputEnabled);
                    // check if the stage is OmniPVD, or switched from being an OmniPVD stage to a non OmniPVD stage
                    // not sure what is required really
                    bool isOVDStage = omniPhysX.getISettings()->getAsBool(kOmniPvdIsOVDStage);
                    // If isOVDStage -> should force omniPVD to be false
                    // If isOVDStage is false -> should function as it used to
                    if (!isOVDStage)
                    {
                        if (setting != (bool)ptr->getOmniPvd())
                        {
                            ptr->changePVDSettings((bool)ptr->getPvd(), setting);
                        }
                    }
                    else
                    {
                        if ((bool)ptr->getOmniPvd())
                        {
                            ptr->changePVDSettings((bool)ptr->getPvd(), false);
                        }
                    }
                };
                subID = mISettings->subscribeToNodeChangeEvents(kOmniPvdOutputEnabled, omniPvdEnabledChangedLambda, this);
                mSubscribedSettings.push_back(subID);

                subID = mISettings->subscribeToNodeChangeEvents(kOmniPvdIsOVDStage, omniPvdEnabledChangedLambda, this);
                mSubscribedSettings.push_back(subID);

            }

            clearCudaContextManagersAndRefillWithNull(1);
        }

        PhysXSetup::~PhysXSetup()
        {
            mCookingService->destroyAsyncContext(mCookingServiceContext);
            delete mErrorCallback;
            mErrorCallback = nullptr;

            for (auto subId : mSubscribedSettings)
            {
                mISettings->unsubscribeToChangeEvents(subId);
            }
            mSubscribedSettings.clear();

            mISettings = nullptr;
        }

        void PhysXSetup::changePVDSettings(bool enableLegacyPVD, bool enableOmniPVD)
        {
            if (enableLegacyPVD == (bool)mVisualDebugger && enableOmniPVD == (bool)mOmniPvd)
            {
                return; // do nothing, we're already set with the correct pvd options
            }
            for (PhysXScenesMap::reference ref : mPhysXScenes)
            {
                if (!ref.second->isComplete())
                {
                    CARB_LOG_WARN("PVD settings can only be changed when simulation is stopped");
                    return;
                }
            }
            cleanupPhysics();
            getPhysics(); // Regenerate PhysXSDK object together with all necessary PVD connections
        }

        void PhysXSetup::resetPhysXErrorCounter()
        {
            mErrorCallback->resetErrorCounter();
        }

        void PhysXSetup::setMaxNumberOfPhysXErrors(uint32_t maxNumberOfPhysXErrors)
        {
            mErrorCallback->setMaxNumErrors(maxNumberOfPhysXErrors);
        }

        void PhysXSetup::clearCudaContextManagers()
        {
            for (auto& cudaContextManager : mCudaContextManagers)
            {
                if (cudaContextManager)
                {
                    SAFE_RELEASE(cudaContextManager);
                }
            }
            mCudaContextManagers.clear();
        }

        void PhysXSetup::clearCudaContextManagersAndRefillWithNull(int numToFill)
        {
            clearCudaContextManagers();

            for (int ord = 0; ord < numToFill; ++ord)
            {
                mCudaContextManagers.push_back(nullptr);
            }        
        }

        void PhysXSetup::setupGPU()
        {
#if USE_PHYSX_GPU
            // AD: OM-117654 - the code in this function can throw a PhysX error, and the callback might access an outdated cuda context in that case.
            mErrorCallback->setCudaContextManager(nullptr);

            mNextCudaContextManagerId = 0;
            const bool enableSynchronousKernelLaunches = OmniPhysX::getInstance().getCachedSettings().enableSynchronousKernelLaunches;
            const int multiGPUMode = OmniPhysX::getInstance().getISettings()->getAsInt(kSettingSceneMultiGPUMode);

            if ((mCudaLaunchSynchronous != enableSynchronousKernelLaunches) || (mSceneMultiGPUMode != multiGPUMode))
            {
                // clear cuda context managers so they can be recreated with correct params with one default space
                clearCudaContextManagersAndRefillWithNull(1);
                mCudaLaunchSynchronous = enableSynchronousKernelLaunches;
                mSceneMultiGPUMode = multiGPUMode;
            }

            // check MultiGPU mode settings
            int availableDeviceCount = 0;
            cuDeviceGetCount(&availableDeviceCount);
            if ((mSceneMultiGPUMode != (int)SceneMultiGPUMode::eDisabled) && availableDeviceCount < 2)
            {
                CARB_LOG_WARN("Warning: MultiGpuMode set to %d, but less than 2 CUDA devices found. Reverting to default!", mSceneMultiGPUMode);
                OmniPhysX::getInstance().getISettings()->setInt(kSettingSceneMultiGPUMode, (int)SceneMultiGPUMode::eDisabled);
                mSceneMultiGPUMode = 0;
            }

            if (mSceneMultiGPUMode)
            {
                int32_t firstOrdinal = 0;

                // skip the first device
                if (mSceneMultiGPUMode == 2)
                {
                    firstOrdinal = 1;
                    availableDeviceCount--;
                }

                if (mCudaContextManagers.size() != availableDeviceCount)
                {
                    clearCudaContextManagersAndRefillWithNull(availableDeviceCount);
                }

                for (int32_t off = 0, ord = firstOrdinal; off < availableDeviceCount; ++off, ++ord)
                {
                    // spawn a separate context and a context manager for each available cuda device
                    omni::physx::PhysxFoundationDeviceOrdinal ordinal;
                    ordinal.mode = omni::physx::PhysxFoundationDeviceOrdinal::eMODE_DEVICE_ORDINAL;
                    ordinal.deviceOrdinal = ord;
                    if(!mPhysxFoundation->createOrRefreshPxCudaContextManager(ordinal, mFoundation, mCudaContextManagers[off], enableSynchronousKernelLaunches))
                    {
                        SAFE_RELEASE(mCudaContextManagers[off]);
                        CARB_LOG_ERROR("Unable to create PxCudaContextManager for device with ordinal %d!", ord);
                    }
                }

                // set the first device as default
                mPhysxFoundation->setCudaDevice(0);
            }
            else
            {
                omni::physx::PhysxFoundationDeviceOrdinal ordinal;
                mPhysxFoundation->getSingleCudaContextManagerOrdinal(ordinal);
                if (!mPhysxFoundation->createOrRefreshPxCudaContextManager(ordinal, mFoundation, mCudaContextManagers[0], enableSynchronousKernelLaunches))
                {
                    SAFE_RELEASE(mCudaContextManagers[0]);
                    CARB_LOG_ERROR("Unable to create PxCudaContextManager!");
                }
            }

            mErrorCallback->setCudaContextManager(getCudaContextManager());
#endif
        }

        void PhysXSetup::createPhysics(const PxTolerancesScale& tolerances)
        {
            // Release any previous version of the asynchronous cooking data singleton
            SAFE_RELEASE(mCookingDataAsync);

            CARB_ASSERT(mFoundation);
            SAFE_RELEASE(mPhysics);
            carb::settings::ISettings* iSettings = OmniPhysX::getInstance().getISettings();
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();

            if (iSettings->getAsBool(kSettingPVDEnabled) && !mVisualDebugger)
            {
                mVisualDebugger = PxCreatePvd(*mFoundation);
            }
            if (!iSettings->getAsBool(kSettingPVDEnabled) && mVisualDebugger)
            {
                SAFE_RELEASE(mVisualDebugger)
            }

            releaseVehiclePvdRegistrationHandles();

            SAFE_RELEASE(mOmniPvd)

#if !CARB_AARCH64
            const bool omniPVDOutputEnabled = iSettings->getAsBool(kOmniPvdOutputEnabled);
            const bool omniPVDIsOVDStage = iSettings->getAsBool(kOmniPvdIsOVDStage);
            bool isOmniPVDRecording = false;
            std::string ovdRecordingFileName;
            if (omniPVDOutputEnabled && !omniPVDIsOVDStage)
            {
                mOmniPvd = PxCreateOmniPvd(*mFoundation);
                if (mOmniPvd)
                {
                    OmniPvdWriter* omniWriter = mOmniPvd->getWriter();
                    if (omniWriter)
                    {
                        // Uncomment for debugging the OmniPvd write stream
                        //omniWriter->setLogFunction(logFunc);
                        OmniPvdFileWriteStream* omniFileWriteStream = mOmniPvd->getFileWriteStream();
                        if (omniFileWriteStream)
                        {
                            omniWriter->setWriteStream(*omniFileWriteStream);
                            const char* outputDirectory = iSettings->getStringBuffer(kOmniPvdOvdRecordingDirectory);
                            if ((*outputDirectory) != 0)
                            {
                                ////////////////////////////////////////////////////////////////////////////////
                                // diffCounter
                                //   Starts at 1 for each time stamp that is taken that is different from the
                                //   previous. As soon as a timeStap is sampled with the same time as the
                                //   previously output OVD recording the diffCounter goes up by 1. This can
                                //   happen if you press recording twice in the same second as they will be
                                //   regarded as the same.
                                ////////////////////////////////////////////////////////////////////////////////
                                // dateTimeStamp
                                //   year_month_hour_minute_second + _(diffCounter)
                                ////////////////////////////////////////////////////////////////////////////////
                                // stageName
                                //   Taken from?
                                ////////////////////////////////////////////////////////////////////////////////
                                // outputFilename
                                //   kOmniPvdOvdRecordingDirectory + dateTimeStamp + optional:("_" + stageName) + ".ovd"
                                ////////////////////////////////////////////////////////////////////////////////
                                OmniPvdRecordingTime nowTs;
                                getLocalOmniPvdTime(nowTs);
                                gLastOmniPvdTime.setWithDiffCounterIncreaseIfSame(nowTs);

                                char buffer[200];
                                sprintf_s(buffer, 200, "%04d_%02d_%02d_%02d_%02d_%02d_%02d",
                                    gLastOmniPvdTime.year, gLastOmniPvdTime.month, gLastOmniPvdTime.day, gLastOmniPvdTime.hour, gLastOmniPvdTime.minute, gLastOmniPvdTime.second, gLastOmniPvdTime.diffCounter);
                                ovdRecordingFileName = outputDirectory;
                                ////////////////////////////////////////////////////////////////////////////////
                                // TODO : extract the stage name if it exists
                                ////////////////////////////////////////////////////////////////////////////////
                                ovdRecordingFileName += "tmp.ovd";
                                setOmniPVDoutputDirectory(outputDirectory);
                                setOmniPVDTimeStampedFileName(buffer);
                                omniFileWriteStream->setFileName(ovdRecordingFileName.c_str());

                                mVehiclePvdRegistrationHandles = ::physx::vehicle2::PxVehiclePvdAttributesCreate(mAllocator, *omniWriter);
                            }
                            else
                            {
                                CARB_LOG_ERROR("OmniPvd: reading output directory failed!");
                                SAFE_RELEASE(mOmniPvd);
                            }
                        }
                        else
                        {
                            CARB_LOG_ERROR("OmniPvd writeStream not set!");
                            SAFE_RELEASE(mOmniPvd);
                        }
                    }
                    else
                    {
                        CARB_LOG_ERROR("OmniPvd writer not created");
                        SAFE_RELEASE(mOmniPvd);
                    }
                }
                else
                {
                    CARB_LOG_ERROR("OmniPvd shared library not loaded!");
                }
            }
#endif

#if USE_PHYSX_GPU
            // initialize only when delay-loaded CUDA lib is present
            if (omniPhysX.isCudaLibPresent())
            {
                // check for at least one suitable device to prevent crashing
                if (!mPhysxFoundation->cudaDeviceCheck())
                {
                    CARB_LOG_ERROR("CUDA libs are present, but no suitable CUDA GPU was found!");
                }
                else
                {
                    setupGPU();
                }
            }
#endif

            mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, tolerances, true, mVisualDebugger, mOmniPvd);
            CARB_ASSERT(mPhysics);

#if !CARB_AARCH64            
            if (mOmniPvd)
            {
                // note: requires PxPhysics to be created
                if (mOmniPvd->startSampling())
                {
                    isOmniPVDRecording = true;
                }
                else
                {
                    CARB_LOG_ERROR("OmniPvd error writing to file: %s",ovdRecordingFileName.c_str());
                }
            }
            iSettings->setBool(kOmniPvdIsRecording, isOmniPVDRecording);
#endif


#if USE_PHYSX_GPU
            PxSetPhysXGpuProfilerCallback(PxGetProfilerCallback());
#endif

            if (!mExtensionsInitialized)
            {
                PxInitExtensions(*mPhysics, mVisualDebugger);
                mExtensionsInitialized = true;
            }

            mDefaultCookingParams = getCookingParams(tolerances);
            mCookingDataAsync = cookingdataasync::createCookingDataAsync(*mPhysics, *mCookingServicePrivate, *mCookingService, mCookingServiceContext);
            mCookingDataAsync->setLocalMeshCacheEnabled(iSettings->getAsBool(kSettingUseLocalMeshCache));
            mCookingDataAsync->setLocalMeshCacheSize(iSettings->getAsInt(kSettingLocalMeshCacheSizeMB));

            // profiler callback
            if (iSettings->getAsBool(kSettingExposeProfilerData))
            {
                PxSetProfilerCallback(&gProfilerCallback);
            }
            else
            {
                PxSetProfilerCallback(nullptr);
            }

            disconnectPVD();
            connectPVD();
        }

        void PhysXSetup::setOmniPVDoutputDirectory(const char* directory)
        {
            mOmniPVDOutputDirectory = directory;
        }

        void PhysXSetup::setOmniPVDTimeStampedFileName(const char* fileName)
        {
            mOmniPVDTimeStampedFileName = fileName;
        }


        void PhysXSetup::writeOutOmniPVDFile(bool omniPVDWasActive)
        {

            carb::settings::ISettings* iSettings = OmniPhysX::getInstance().getISettings();
            if (!iSettings)
            {
                return;
            }

            ////////////////////////////////////////////////////////////////////////////////
            // Was omniPVD active and was the stage simulated?
            ////////////////////////////////////////////////////////////////////////////////
            if (omniPVDWasActive && (OmniPhysX::getInstance().getSimulationStepCount() > 0) && iSettings->getAsBool(kOmniPvdIsRecording))
            {
                ////////////////////////////////////////////////////////////////////////////////
                // Rename the file : mOmniPVDOutputDirectory + tmp.ovd
                // into : outputDirFinal + mOmniPVDTimeStampedFileName
                ////////////////////////////////////////////////////////////////////////////////
                std::string outputDirFinal = mOmniPVDOutputDirectory;

                ////////////////////////////////////////////////////////////////////////////////
                // The output directory could have been changed by the user to be different
                // from that of where the tmp was created, so adjust if necessary.
                ////////////////////////////////////////////////////////////////////////////////
                const char* outputDirectory = iSettings->getStringBuffer(kOmniPvdOvdRecordingDirectory);
                if ((*outputDirectory) != 0)
                {
                    outputDirFinal = outputDirectory;
                }
                ////////////////////////////////////////////////////////////////////////////////
                // We have the final output directory
                // First write it out in the dir with the name
                ////////////////////////////////////////////////////////////////////////////////
                std::string tmpFilePath = mOmniPVDOutputDirectory + "tmp.ovd";
                std::string finalFilePath = outputDirFinal + mOmniPVDTimeStampedFileName + "_rec.ovd";
                rename(tmpFilePath.c_str(), finalFilePath.c_str());

                iSettings->setString("/persistent/physics/omniPvdImportDirectory", outputDirFinal.c_str());
            }
        }

        void PhysXSetup::releaseVehiclePvdRegistrationHandles()
        {
            if (mVehiclePvdRegistrationHandles)
            {
                ::physx::vehicle2::PxVehiclePvdAttributesRelease(mAllocator, *mVehiclePvdRegistrationHandles);
                mVehiclePvdRegistrationHandles = nullptr;
            }
        }

        void PhysXSetup::cleanupPhysics()
        {
            SAFE_RELEASE(mCookingDataAsync);
            getMeshCache()->release();
            SAFE_RELEASE(mCylinderMeshX);
            SAFE_RELEASE(mCylinderMeshY);
            SAFE_RELEASE(mCylinderMeshZ);
            SAFE_RELEASE(mConeMeshX);
            SAFE_RELEASE(mConeMeshY);
            SAFE_RELEASE(mConeMeshZ);

            if (mExtensionsInitialized)
            {
                PxCloseExtensions();
                mExtensionsInitialized = false;
            }

            SAFE_RELEASE(mSerializationRegistry)
            SAFE_RELEASE(mPhysics)

            releaseVehiclePvdRegistrationHandles();

            bool omniPVDWasActive = (bool)mOmniPvd;
            SAFE_RELEASE(mOmniPvd)
            writeOutOmniPVDFile(omniPVDWasActive);
        }

        void PhysXSetup::createPhysics()
        {
            mErrorCallback->setEventStream(OmniPhysX::getInstance().getErrorEventStream());
            mErrorCallback->setErrorEventType(ePhysxError);

            // A.B. TODO
            // we really should have a common memory manager and pass it to PhysX too
            mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, mAllocator, *mErrorCallback);

            createCpuDispatcher(mThreadCount);

            // Initialize the vehicle SDK.
            if (!mVehicleSDKInitialized)
            {
                mVehicleSDKInitialized = ::physx::vehicle2::PxInitVehicleExtension(*mFoundation);
            }
        }

        // final release, should be called when app is closing
        void PhysXSetup::releasePhysics()
        {
            omni::sampling::notifyPhysXSceneRelease();
            omni::physx::particles::notifyPhysXRelease();
            // Release the asynchronous cooking data singleton
            SAFE_RELEASE(mCookingDataAsync);

            for (PhysXScenesMap::reference ref : mPhysXScenes)
            {
                delete ref.second;
            }
            mPhysXScenes.clear();

            releaseCpuDispatcher();
#if USE_PHYSX_GPU
            mErrorCallback->setCudaContextManager(nullptr); //OM-111223 - crash because we dereference stale pointer if we output an error during shutdown.
            clearCudaContextManagers();
#endif

            if (mVehicleSDKInitialized)
            {
                ::physx::vehicle2::PxCloseVehicleExtension();
                mVehicleSDKInitialized = false;
            }

            if (mExtensionsInitialized)
            {
                PxCloseExtensions();
                mExtensionsInitialized = false;
            }

            SAFE_RELEASE(mSerializationRegistry)
            SAFE_RELEASE(mPhysics)
            SAFE_RELEASE(mVisualDebugger)

            releaseVehiclePvdRegistrationHandles();
            bool omniPVDWasActive = (bool)mOmniPvd;
            SAFE_RELEASE(mOmniPvd)
            SAFE_RELEASE(mFoundation)

            writeOutOmniPVDFile(omniPVDWasActive);            

            mErrorCallback->invalidateEventStream();
        }

        void addLastSlash(std::string& fileDirectory) {
            if ((fileDirectory.back() != '/') && (fileDirectory.back() != '\\'))
            {
                if (fileDirectory.find('\\') != std::string::npos)
                {
                    fileDirectory += '\\';
                }
                else
                {
                    fileDirectory += '/';
                }
            }
        }
        PxPhysics* PhysXSetup::getPhysics()
        {
            if (!mPhysics)
            {
                ::physx::PxTolerancesScale tolerances = getDefaultTolerances(OmniPhysX::getInstance().getStage());
                createPhysics(tolerances);
            }
            return mPhysics;
        }

        ::physx::PxCudaContextManager* PhysXSetup::getCudaContextManager(size_t id) const
        {
            return mCudaContextManagers[id];
        }

        ::physx::PxCudaContextManager* PhysXSetup::getNextCudaContextManager()
        {
            size_t id = mNextCudaContextManagerId;
            mNextCudaContextManagerId = (mNextCudaContextManagerId + 1) % mCudaContextManagers.size();
            return mCudaContextManagers[id];
        }

        void PhysXSetup::connectPVD()
        {
            static bool wasPVDFileOutput = false;
            if (mVisualDebugger)
            {
                carb::settings::ISettings* iSettings = OmniPhysX::getInstance().getISettings();

                if (iSettings->getAsBool(kSettingPVDStreamToFile))
                {
                    std::string outputFilePath = iSettings->getStringBuffer(kSettingPVDOutputDirectory);
                    addLastSlash(outputFilePath);
                    outputFilePath += "tmp.pvd";
                    mPvdTransport = PxDefaultPvdFileTransportCreate(outputFilePath.c_str());
                }
                else
                    mPvdTransport = PxDefaultPvdSocketTransportCreate(iSettings->getStringBuffer(kSettingPVDIPAddress), 5425, 10);

                PxPvdInstrumentationFlags instrumentationFlags(0);
                if (iSettings->getAsBool(kSettingPVDProfile))
                    instrumentationFlags |= PxPvdInstrumentationFlag::ePROFILE;
                if (iSettings->getAsBool(kSettingPVDMemory))
                    instrumentationFlags |= PxPvdInstrumentationFlag::eMEMORY;
                if (iSettings->getAsBool(kSettingPVDDebug))
                    instrumentationFlags |= PxPvdInstrumentationFlag::eDEBUG;

                wasPVDFileOutput = false;

                if (instrumentationFlags && mPvdTransport != nullptr)
                {
                    bool success = mVisualDebugger->connect(*mPvdTransport, instrumentationFlags);
                    if (!success)
                    {
                        CARB_LOG_WARN("PVD was enabled but the connection failed");
                        if (iSettings->getAsBool(kSettingPVDStreamToFile))
                        {
                            CARB_LOG_WARN("Stream to file is enabled for PVD - maybe double check output file location/access privileges?");
                        }
                    }
                    else if (iSettings->getAsBool(kSettingPVDStreamToFile))
                    {
                        wasPVDFileOutput = true;
                    }
                }
            }
            else
            {
                wasPVDFileOutput = false;
            }
        }

        void PhysXSetup::disconnectPVD()
        {
            if (mVisualDebugger)
            {
                // rename the temporary file to the final destination, if PVD was outputting to a file and there was any simulation done
                mVisualDebugger->disconnect();
                if (mPvdTransport)
                {
                    mPvdTransport->release();
                    mPvdTransport = nullptr;
                    if (OmniPhysX::getInstance().getSimulationStepCount() > 0)
                    {
                        carb::settings::ISettings* iSettings = OmniPhysX::getInstance().getISettings();
                        if (iSettings->getAsBool(kSettingPVDStreamToFile))
                        {
                            std::string outputFileDir = iSettings->getStringBuffer(kSettingPVDOutputDirectory);
                            if (!outputFileDir.empty())
                            {
                                OmniPvdRecordingTime nowTs;
                                getLocalOmniPvdTime(nowTs);
                                gLastOmniPvdTime.setWithDiffCounterIncreaseIfSame(nowTs);
                                addLastSlash(outputFileDir);

                                char buffer[200];
                                sprintf_s(buffer, 200, "%04d_%02d_%02d_%02d_%02d_%02d_%02d",
                                    gLastOmniPvdTime.year, gLastOmniPvdTime.month, gLastOmniPvdTime.day, gLastOmniPvdTime.hour, gLastOmniPvdTime.minute, gLastOmniPvdTime.second, gLastOmniPvdTime.diffCounter);
                                std::string outputFileName = outputFileDir + "rec_" + buffer + ".pxd2";

                                std::string tmpFilepath    = outputFileDir + "tmp.pvd";

                                //printf("in : %s\nout: %s\n", tmpFilepath.c_str(), outputFileName.c_str());
                                rename(tmpFilepath.c_str(), outputFileName.c_str());
                            }
                        }
                    }
                }
            }
        }

        ::physx::PxCookingParams PhysXSetup::getCookingParams(const ::physx::PxTolerancesScale& tolerances)
        {
            PxCookingParams params(tolerances);
            params.buildGPUData = false;
            params.buildTriangleAdjacencies = true;
            return params;
        }

        ::physx::PxTolerancesScale    PhysXSetup::getDefaultTolerances(double metersPerUnit)
        {
            return ::physx::PxTolerancesScale(float(1.0 / metersPerUnit), float(10.0 / metersPerUnit));
        }

        ::physx::PxTolerancesScale   PhysXSetup::getDefaultTolerances(pxr::UsdStageRefPtr stage)
        {
            const double metersPerUnit = pxr::UsdGeomGetStageMetersPerUnit(stage);
            return getDefaultTolerances(metersPerUnit);
        }

        void PhysXSetup::createCpuDispatcher(uint32_t numThreads)
        {
            releaseCpuDispatcher();
            if (!mPhysXDispatcher)
                mDispatcher = new PhysXCarbCpuDispatcher(mThreadCount);
            else
            {
                PxDefaultCpuDispatcher* physxDispatcher = PxDefaultCpuDispatcherCreate(mThreadCount);
                physxDispatcher->setRunProfiled(true);
                mDispatcher = physxDispatcher;
            }
        }

        void PhysXSetup::releaseCpuDispatcher()
        {
            if (!mPhysXDispatcher)
            {
                if (mDispatcher)
                {
                    ((PhysXCarbCpuDispatcher*)mDispatcher)->release();
                    delete mDispatcher;
                    mDispatcher = nullptr;
                }
            }
            else
            {
                if (mDispatcher)
                {
                    ((PxDefaultCpuDispatcher*)mDispatcher)->release();
                    mDispatcher = nullptr;
                }
            }
        }

        void PhysXSetup::releasePhysXScenes()
        {
            mFilteredPairs.clear();
            mCollisionsGroupFilteredPairs.clear();

            disconnectPVD();

            omni::sampling::notifyPhysXSceneRelease();
            
            // we first make sure that we wait for completion of all scenes
            for (PhysXScenesMap::reference ref : mPhysXScenes)
            {
                ref.second->waitForCompletion(false);
            }
            // then we move them as in the InternalScene destructor we will
            // be calling waitForCompletion that is looping all scenes, potentially
            // accessing some of them that will be already freed
            PhysXScenesMap physxScenes = std::move(mPhysXScenes);
            for (PhysXScenesMap::reference ref : physxScenes)
            {
                delete ref.second;
            }
            SAFE_RELEASE(mVehicleWheelCylinderMeshX);
            SAFE_RELEASE(mVehicleWheelCylinderMeshY);
            SAFE_RELEASE(mVehicleWheelCylinderMeshZ);

            cleanupPhysics();

            mDefaultScene = nullptr;
        }

        PhysXScene* PhysXSetup::createPhysXScene(const usdparser::AttachedStage& attachedStage, size_t sceneId, double metersPerUnit, double kilogramsPerUnit, const usdparser::PhysxSceneDesc& sceneDesc)
        {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::settings::ISettings* iSettings = omniPhysX.getISettings();
            PxPhysics* physics = getPhysics();

            if (mPhysXScenes.empty())
            {
                const uint32_t numThreads = iSettings->isAccessibleAs(carb::dictionary::ItemType::eInt, (kSettingNumThreads)) ?
                    iSettings->getAsInt(kSettingNumThreads) :
                    mThreadCount;

                const bool newDispatcher = iSettings->isAccessibleAs(carb::dictionary::ItemType::eBool, (kSettingPhysxDispatcher)) ? iSettings->getAsBool(kSettingPhysxDispatcher) : false;
                if ((mThreadCount != numThreads || newDispatcher != mPhysXDispatcher) && numThreads < 256)
                {
                    releaseCpuDispatcher();
                    mPhysXDispatcher = newDispatcher;
                    mThreadCount = numThreads;
                    createCpuDispatcher(numThreads);
                }

                omniPhysX.setGpuPipelineOverride(iSettings->isAccessibleAs(carb::dictionary::ItemType::eInt, (kSettingOverrideGPU)) ?
                    iSettings->getAsInt(kSettingOverrideGPU) :
                    omniPhysX.getGpuPipelineOverride());
            }

            // tolerances check, they could have changed
            PxTolerancesScale physicsTolerances = physics->getTolerancesScale();
            PxTolerancesScale sceneTol = getDefaultTolerances(metersPerUnit);
            if (fabsf(physicsTolerances.length - sceneTol.length) > 0.001f)
            {
                // tolerances changed recreate physics
                if (mPhysXScenes.empty())
                {
                    cleanupPhysics();
                    getPhysics();
                }
                else
                {
                    CARB_LOG_ERROR("Cant change metersPerUnit for existing scenes.");
                    metersPerUnit = 1.0f / physicsTolerances.length;
                }
            }

            PhysXScene* scene = PhysXScene::createPhysXScene(attachedStage, sceneId, *this, metersPerUnit, kilogramsPerUnit, sceneDesc);
            CARB_ASSERT(scene);
            if (scene)
            {
                if (mPhysXScenes.empty())
                {
                    mDefaultScene = scene;
                }
                mPhysXScenes[sceneId] = scene;

                getCylinderConvexMesh(omni::physx::usdparser::eX);
                getCylinderConvexMesh(omni::physx::usdparser::eY);
                getCylinderConvexMesh(omni::physx::usdparser::eZ);

                getConeConvexMesh(omni::physx::usdparser::eX);
                getConeConvexMesh(omni::physx::usdparser::eY);
                getConeConvexMesh(omni::physx::usdparser::eZ);

                getVehicleWheelCylinderConvexMesh(omni::physx::usdparser::eX);
                getVehicleWheelCylinderConvexMesh(omni::physx::usdparser::eY);
                getVehicleWheelCylinderConvexMesh(omni::physx::usdparser::eZ);
            }

            return scene;
        }


        ::physx::PxConvexMesh* PhysXSetup::getConeConvexMesh(enum omni::physx::usdparser::Axis axis) const
        {
            if (axis == eX)
            {
                if (!mConeMeshX)
                {
                    mConeMeshX = createConeConvexMesh(tConeOrCylinderWidth, tConeOrCylinderRadius, tConeNumCirclePoints, eX);
                }
                return mConeMeshX;
            }
            else if (axis == eY)
            {
                if (!mConeMeshY)
                {
                    mConeMeshY = createConeConvexMesh(tConeOrCylinderWidth, tConeOrCylinderRadius, tConeNumCirclePoints, eY);
                }
                return mConeMeshY;
            }
            else
            {
                if (!mConeMeshZ)
                {
                    mConeMeshZ = createConeConvexMesh(tConeOrCylinderWidth, tConeOrCylinderRadius, tConeNumCirclePoints, eZ);
                }
                return mConeMeshZ;
            }
        }

        ::physx::PxConvexMesh* PhysXSetup::getCylinderConvexMesh(enum Axis axis) const
        {
            if (axis == eX)
            {
                if (!mCylinderMeshX)
                {
                    mCylinderMeshX = createCylinderConvexMesh(tConeOrCylinderWidth, tConeOrCylinderRadius, tCylinderNumCirclePoints, eX);
                }
                return mCylinderMeshX;
            }
            else if (axis == eY)
            {
                if (!mCylinderMeshY)
                {
                    mCylinderMeshY = createCylinderConvexMesh(tConeOrCylinderWidth, tConeOrCylinderRadius, tCylinderNumCirclePoints, eY);
                }
                return mCylinderMeshY;
            }
            else
            {
                if (!mCylinderMeshZ)
                {
                    mCylinderMeshZ = createCylinderConvexMesh(tConeOrCylinderWidth, tConeOrCylinderRadius, tCylinderNumCirclePoints, eZ);
                }
                return mCylinderMeshZ;
            }
        }

        ::physx::PxConvexMesh* PhysXSetup::getVehicleWheelCylinderConvexMesh(enum Axis axis) const
        {
            constexpr float cylinderWidth = 2.0f;
            constexpr float cylinderRadius = 1.0f;
            constexpr uint32_t cylinderNumCirclePoints = tCylinderNumCirclePoints;

            if (axis == eX)
            {
                if (!mVehicleWheelCylinderMeshX)
                {
                    mVehicleWheelCylinderMeshX = createCylinderConvexMesh(cylinderWidth, cylinderRadius, cylinderNumCirclePoints, eX);
                }
                return mVehicleWheelCylinderMeshX;
            }
            else if (axis == eY)
            {
                if (!mVehicleWheelCylinderMeshY)
                {
                    mVehicleWheelCylinderMeshY = createCylinderConvexMesh(cylinderWidth, cylinderRadius, cylinderNumCirclePoints, eY);
                }
                return mVehicleWheelCylinderMeshY;
            }
            else
            {
                if (!mVehicleWheelCylinderMeshZ)
                {
                    mVehicleWheelCylinderMeshZ = createCylinderConvexMesh(cylinderWidth, cylinderRadius, cylinderNumCirclePoints, eZ);
                }
                return mVehicleWheelCylinderMeshZ;
            }
        }
    } // namespace physx
} // namespace omni
