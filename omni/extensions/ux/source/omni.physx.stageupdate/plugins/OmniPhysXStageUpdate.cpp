// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#define CARB_EXPORTS


#include <carb/Framework.h>
#include <carb/PluginUtils.h>

#include <omni/kit/IApp.h>
#include <omni/ext/IExt.h>
#include <omni/ext/IExtensions.h>
#include <omni/ext/ExtensionsUtils.h>
#include <omni/kit/IStageUpdate.h>
#include <omni/kit/KitUpdateOrder.h>

#include <private/omni/physx/IPhysxStageUpdate.h>
#include <omni/physx/IPhysxStageUpdateNode.h>

static constexpr char kPlaySimulations[] = "/app/player/playSimulations";

const struct carb::PluginImplDesc kPluginImpl = { "omni.physx.stageupdate.plugin", "Physics", "NVIDIA",
                                                  carb::PluginHotReload::eDisabled, "dev" };


namespace omni
{
namespace physx
{

class OmniPhysXStageUpdateExtensionImpl;
static OmniPhysXStageUpdateExtensionImpl* gStageUpdateImpl = nullptr;

class OmniPhysXStageUpdateExtensionImpl : public omni::ext::IExt
{
public:
    OmniPhysXStageUpdateExtensionImpl()
    {
        gStageUpdateImpl = this;
    }

    ~OmniPhysXStageUpdateExtensionImpl()
    {
        gStageUpdateImpl = nullptr;
    }

    void onStartup(const char* extId) override
    {
        carb::Framework* framework = carb::getFramework();
        mStageUpdate = omni::kit::getStageUpdate();
        mIPhysxStageUpdate = framework->tryAcquireInterface<omni::physx::IPhysxStageUpdate>();

        omni::kit::IApp* app = carb::getCachedInterface<omni::kit::IApp>();

        mExtensionEnableSub = subscribeToExtensionEnable(
            app->getExtensionManager(),
            [this](const char* extName) {
                CARB_LOG_WARN("Only one of omni.physx.stageupdate or omni.physics.stageupdate can be enabled. Detaching omni.physx.stageupdate.");
                destroyStageUpdateNode();
            },
            nullptr,
            "omni.physics.stageupdate",
            "on_physics_stageupdate_enabled"
        );

        attachStageUpdate();
    }

    void onShutdown() override
    {
        if (mExtensionEnableSub.first)
        {
            mExtensionEnableSub.first = nullptr;
        }

        detachStageUpdate();
        mIPhysxStageUpdate = nullptr;
    }

    omni::physx::IPhysxStageUpdate* getIPhysxStageUpdate() const
    {
        return mIPhysxStageUpdate;
    }

    void destroyStageUpdateNode();
    void attachStageUpdate();
    void detachStageUpdate();
    bool isStageUpdateAttached() const
    {
        return mStageUpdateNode ? true : false;
    }

    bool timeLineEventsBlocked() const        
    {
        return mTimeLineEventsBlocked;
    }

    void setTimeLineEventsBlocked(bool val)
    {
        mTimeLineEventsBlocked = val;
    }
    
private:
    bool mTimeLineEventsBlocked{ false };
    omni::physx::IPhysxStageUpdate* mIPhysxStageUpdate{ nullptr };
    omni::kit::StageUpdatePtr mStageUpdate;
    omni::kit::StageUpdateNode* mStageUpdateNode{ nullptr };
    omni::kit::StageUpdateNode* mFabricStageUpdateNode{ nullptr };
    omni::kit::StageUpdateNode* mLastStageUpdateNode{ nullptr };
    std::pair<omni::ext::IHookHolderPtr, omni::ext::IHookHolderPtr> mExtensionEnableSub{ nullptr, nullptr };
};


void onPhysXAttach(long int stageId, double , void* )
{
    if (gStageUpdateImpl && gStageUpdateImpl->getIPhysxStageUpdate())
    {
        gStageUpdateImpl->getIPhysxStageUpdate()->onAttach(stageId);
    }
}

void onPhysXFabricAttach(long int stageId, double, void*)
{
    if (gStageUpdateImpl && gStageUpdateImpl->getIPhysxStageUpdate())
    {
        gStageUpdateImpl->getIPhysxStageUpdate()->onFabricAttach(stageId);
    }
}

void onPhysXDetach(void*)
{
    if (gStageUpdateImpl && gStageUpdateImpl->getIPhysxStageUpdate())
    {
        gStageUpdateImpl->getIPhysxStageUpdate()->onDetach();
    }
}

void physXUpdate(float currentTime, float elapsedSecs, const omni::kit::StageUpdateSettings* settings, void* )
{
    if (gStageUpdateImpl && gStageUpdateImpl->getIPhysxStageUpdate())
    {
        const bool enableUpdate = (!settings || (settings->isPlaying && settings->playSimulations)) && !gStageUpdateImpl->timeLineEventsBlocked();
        gStageUpdateImpl->getIPhysxStageUpdate()->onUpdate(currentTime, elapsedSecs, enableUpdate);
    }
}

void physXResume(float currentTime, void*)
{
    carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
    const bool playSimulation = settings->getAsBool(kPlaySimulations);

    if (gStageUpdateImpl && gStageUpdateImpl->getIPhysxStageUpdate() && !gStageUpdateImpl->timeLineEventsBlocked() &&
        playSimulation)
    {
        gStageUpdateImpl->getIPhysxStageUpdate()->onResume(currentTime);
    }
}

void physXPause(void*)
{
    if (gStageUpdateImpl && gStageUpdateImpl->getIPhysxStageUpdate() && !gStageUpdateImpl->timeLineEventsBlocked())
    {
        gStageUpdateImpl->getIPhysxStageUpdate()->onPause();
    }
}

void physXReset(void*)
{
    if (gStageUpdateImpl && gStageUpdateImpl->getIPhysxStageUpdate() && !gStageUpdateImpl->timeLineEventsBlocked())
    {
        gStageUpdateImpl->getIPhysxStageUpdate()->onReset();
    }
}

void handleRaycast(const float* orig, const float* dir, bool input, void* )
{
    if (gStageUpdateImpl && gStageUpdateImpl->getIPhysxStageUpdate() && !gStageUpdateImpl->timeLineEventsBlocked())
    {
        gStageUpdateImpl->getIPhysxStageUpdate()->handleRaycast(orig, dir, input);
    }
}

void OmniPhysXStageUpdateExtensionImpl::attachStageUpdate()
{
    if (mStageUpdate)
    {
        if (!mStageUpdateNode)
        {
            omni::kit::StageUpdateNodeDesc desc = { 0 };
            desc.displayName = "PhysX";
            desc.order = omni::kit::update::eIUsdStageUpdatePhysics;
            desc.onAttach = onPhysXAttach;
            desc.onDetach = onPhysXDetach;
            desc.onUpdate = physXUpdate;
            desc.onResume = physXResume;
            desc.onPause = physXPause;
            desc.onStop = physXReset;
            desc.onRaycast = handleRaycast;

            mStageUpdateNode = mStageUpdate->createStageUpdateNode(desc);
        }

        if (!mFabricStageUpdateNode)
        {
            omni::kit::StageUpdateNodeDesc desc = { 0 };
            desc.displayName = "PhysXFabric";
            desc.order = omni::kit::update::eIUsdStageUpdatePhysxFC;
            desc.onAttach = onPhysXFabricAttach;
            mFabricStageUpdateNode = mStageUpdate->createStageUpdateNode(desc);
        }
    }
}

void OmniPhysXStageUpdateExtensionImpl::destroyStageUpdateNode()
{
    if (mStageUpdate)
    {
        if (mStageUpdateNode != nullptr)
        {
            mStageUpdate->destroyStageUpdateNode(mStageUpdateNode);
            mStageUpdateNode = nullptr;
        }

        if (mFabricStageUpdateNode != nullptr)
        {
            mStageUpdate->destroyStageUpdateNode(mFabricStageUpdateNode);
            mFabricStageUpdateNode = nullptr;
        }

        if (mLastStageUpdateNode != nullptr)
        {
            mStageUpdate->destroyStageUpdateNode(mLastStageUpdateNode);
            mLastStageUpdateNode = nullptr;
        }
    }
}

void OmniPhysXStageUpdateExtensionImpl::detachStageUpdate()
{
    destroyStageUpdateNode();

    if (mIPhysxStageUpdate)
    {
        mIPhysxStageUpdate->onDetach();
    }
}

}
}

void attachNode()
{
    if (omni::physx::gStageUpdateImpl)
    {
        omni::physx::gStageUpdateImpl->attachStageUpdate();
    }
}

void detachNode()
{
    if (omni::physx::gStageUpdateImpl)
    {
        omni::physx::gStageUpdateImpl->detachStageUpdate();
    }
}

bool isNodeAttached()
{
    if (omni::physx::gStageUpdateImpl)
    {
        return omni::physx::gStageUpdateImpl->isStageUpdateAttached();
    }
    return false;
}

void blockTimeLineEvents(bool val)
{
    if (omni::physx::gStageUpdateImpl)
    {
        omni::physx::gStageUpdateImpl->setTimeLineEventsBlocked(val);
    }
}

bool timeLineEventsBlocked()
{
    if (omni::physx::gStageUpdateImpl)
    {
        return omni::physx::gStageUpdateImpl->timeLineEventsBlocked();
    }
    return false;
}

CARB_PLUGIN_IMPL(kPluginImpl, omni::physx::OmniPhysXStageUpdateExtensionImpl, omni::physx::IPhysxStageUpdateNode)
CARB_PLUGIN_IMPL_DEPS(omni::kit::IStageUpdate, omni::physx::IPhysxStageUpdate)

void fillInterface(omni::physx::OmniPhysXStageUpdateExtensionImpl& iface)
{
}


void fillInterface(omni::physx::IPhysxStageUpdateNode& iface)
{
    iface.attachNode = attachNode;
    iface.detachNode = detachNode;
    iface.isNodeAttached = isNodeAttached;
    iface.blockTimeLineEvents = blockTimeLineEvents;
    iface.timeLineEventsBlocked = timeLineEventsBlocked;
}

CARB_EXPORT void carbOnPluginShutdown()
{
    detachNode();
}
