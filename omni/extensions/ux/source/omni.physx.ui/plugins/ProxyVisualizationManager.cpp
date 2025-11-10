// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "ProxyVisualizationManager.h"
#include <carb/profiler/Profile.h>

#include <omni/kit/EditorUsd.h>
#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/Selection.h>


using namespace omni::physx::ui;
using namespace pxr;

extern bool gBlockNoticeHandle;
extern UsdStageRefPtr gStage;
extern uint8_t gProxySelectionGroup;

ProxyVisualizationManager::ProxyVisualizationManager()
    : mBufferSelectionDirty(false),
      mSessionLayerProxiesRoot{SdfPath()}
{
}

ProxyVisualizationManager::~ProxyVisualizationManager()
{
    release();
    for (auto& pClient : mProxyVisualizationClients)
    {
        delete pClient;
        pClient = nullptr;
    }
    mProxyVisualizationClients.clear();
}

void ProxyVisualizationManager::addClient(ProxyVisualizationClient& client)
{
    mProxyVisualizationClients.push_back(&client);
}

bool ProxyVisualizationManager::isActive()
{
    for (auto& pClient : mProxyVisualizationClients)
    {
        if (pClient && pClient->isActive())
        {
            return true;
        }
    }

    return false;
}

bool ProxyVisualizationManager::isEmpty()
{
    for (auto& pClient : mProxyVisualizationClients)
    {
        if (!pClient || !pClient->isEmpty())
        {
            return false;
        }
    }

    return true;
}

void ProxyVisualizationManager::handlePrimResync(SdfPath path)
{
    if (!gStage)
        return;

    UsdPrimRange range(gStage->GetPrimAtPath(path));
    for (pxr::UsdPrimRange::const_iterator cit = range.begin(); cit != range.end(); ++cit)
    {
        const UsdPrim& prim = *cit;
        if (!prim)
            continue;

        for (auto& pClient : mProxyVisualizationClients)
        {
            if (pClient)
                pClient->handlePrimResync(prim.GetPath());
        }
    }
}

void ProxyVisualizationManager::handlePrimRemove(SdfPath path)
{
    const auto iteratorPair = mActualTable.FindSubtreeRange(path);
    for (auto it = iteratorPair.first; it != iteratorPair.second; it++)
    {
        for (auto& pClient : mProxyVisualizationClients)
        {
            if (pClient)
                pClient->handlePrimRemove(it->first);
        }
    }
}

void ProxyVisualizationManager::handleAttributeChange(SdfPath path, pxr::TfToken attributeName, bool isXform)
{
    // xform or visibility changes must always be handled because the change may have happened in the hierarchy above the actual prim:
    if (isXform || attributeName == UsdGeomTokens.Get()->visibility)
    {
        const auto iteratorPair = mActualTable.FindSubtreeRange(path);
        for (auto it = iteratorPair.first; it != iteratorPair.second; it++)
        {
            if (isXform)
            {
                for (auto& pClient : mProxyVisualizationClients)
                {
                    if (pClient)
                        pClient->handleTransformChange(it->first);
                }             
            }
            else //attributeName == UsdGeomTokens.Get()->visibility
            {
                for (auto& pClient : mProxyVisualizationClients)
                {
                    if (pClient)
                        pClient->handleVisibilityChange(it->first);
                }
            }
        }
        return;
    }

    auto it = mChangeEventMap.find(attributeName);
    if (it != mChangeEventMap.end())
    {
        for (ProxyChangeEvent changeEvent : it->second)
        {
            changeEvent.callback(*changeEvent.client, path, attributeName);
        }
    }
}

void ProxyVisualizationManager::parseStage()
{
    CARB_PROFILE_ZONE(0, "ProxyVisualizationManager::parseStage");

    bool needsStageParse = false;
    for (auto& pClient : mProxyVisualizationClients)
    {
        if (pClient && pClient->needsStageParse(gStage))
        {
            needsStageParse = true;
        }
    }

    if (needsStageParse && gStage)
    {
        handlePrimResync(gStage->GetPseudoRoot().GetPath());
    }        
}

void ProxyVisualizationManager::update()
{
    CARB_PROFILE_ZONE(0, "ProxyVisualizationManager::Update");

    for (auto& pClient : mProxyVisualizationClients)
    {
        if (!pClient || !pClient->isActive())
        {
            continue;
        }

        // block all notices from changes starting here:
        gBlockNoticeHandle = true;

        // all updates are to session layer
        omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());

        pClient->updateTracking();

        //prep filtering
        if (!pClient->isEmpty())
        {
            if (mBufferSelectionDirty)
            {
                mBufferActualToUpdateActive.insert(mActualSelected.begin(), mActualSelected.end());

                mActualSelected.clear();
                const auto usdContext = omni::usd::UsdContext::getContext();
                const std::vector<SdfPath> selectedPaths = usdContext->getSelection()->getSelectedPrimPathsV2();
                for (uint32_t s = 0; s < selectedPaths.size(); ++s)
                {
                    const auto iteratorPair = mActualTable.FindSubtreeRange(selectedPaths[s]);
                    for (auto it = iteratorPair.first; it != iteratorPair.second; it++)
                    {
                        if (mActualToProxyInfoMap.count(it->first))
                        {
                            mActualSelected.insert(it->first);
                        }
                    }
                }

                mBufferActualToUpdateActive.insert(mActualSelected.begin(), mActualSelected.end());
            }

            pClient->updateModeDirty();

            for (SdfPath path : mBufferActualToUpdateActive)
            {
                bool active = updateActive(path);
                if (active)
                {
                    mActualActive.insert(path);
                }
                else
                {
                    mActualActive.erase(path);
                }
            }

            // Do all attribute updates and prim deletions in Sdf changeblock: (TODO removals)
            {
                SdfChangeBlock block;

                // update xforms and verts:
                mXformCache.Clear();

                pClient->updateProxyProperties(mXformCache);

                if (mBufferSelectionDirty)
                {
                    const auto context = omni::usd::UsdContext::getContext();
                    if (context)
                    {
                        for (SdfPath actualPath : mActualActive)
                        {
                            uint8_t group = isSelected(actualPath) ? gProxySelectionGroup : 0;
                            updateProxySelectionOutline(actualPath, *context, group);
                        }
                    }
                }
            }

            // Moved purpose updates out of change block due to hydra update failing in parts
            for (SdfPath actualPath : mBufferActualToUpdatePurpose)
            {
                ProxyInfo* proxyInfo = getProxyInfo(actualPath);
                if (proxyInfo && proxyInfo->client)
                {
                    proxyInfo->client->updateActualPurpose(actualPath, isActive(actualPath));
                }
            }
        }

        gBlockNoticeHandle = false;

        clearBuffers();

        pClient->clearBuffers();
    }
}

void ProxyVisualizationManager::selectionChanged()
{
    const auto usdContext = omni::usd::UsdContext::getContext();
    const std::vector<SdfPath> selectedPaths = usdContext->getSelection()->getSelectedPrimPathsV2();

    bool needToRedirect = false;
    std::vector<SdfPath> primPaths;
    primPaths.reserve(selectedPaths.size());
    for (SdfPath path : selectedPaths)
    {
        const auto cit = mProxyToActualPrimMap.find(path);
        if (cit != mProxyToActualPrimMap.end()) // is a debug vis
        {
            primPaths.push_back(cit->second);
            needToRedirect = true;
        }
        else
        {
            primPaths.push_back(path);
        }
    }

    if (needToRedirect)
    {
        usdContext->getSelection()->setSelectedPrimPathsV2(primPaths);
    }

    mBufferSelectionDirty = true;
}

void ProxyVisualizationManager::release()
{
    for (auto& pClient : mProxyVisualizationClients)
    {
        if (pClient)
            pClient->release();
    }
    
    clear();
}

void ProxyVisualizationManager::registerAttribute(TfToken attributeName, ProxyVisualizationClient& client, void(*callback)(ProxyVisualizationClient&, SdfPath, TfToken))
{
    ProxyChangeEvent changeEvent = {&client, callback};
    auto it = mChangeEventMap.find(attributeName);
    if (it != mChangeEventMap.end())
    {
        it->second.push_back(changeEvent);
    }
    else
    {
        mChangeEventMap.insert({attributeName, {changeEvent}});
    }
}

SdfPath ProxyVisualizationManager::createProxyRootPrim(SdfPath actualPath)
{
    SdfPath rootProxyPath = getProxyPath(actualPath);
    UsdGeomXform xform = UsdGeomXform::Get(gStage, rootProxyPath);
    if (xform)
    {
        return rootProxyPath;
    }
    xform = UsdGeomXform::Define(gStage, rootProxyPath);

    omni::kit::EditorUsd::setNoDelete(xform.GetPrim(), true);
    omni::kit::EditorUsd::setHideInStageWindow(xform.GetPrim(), true);
    omni::kit::EditorUsd::setNoSelectionOutline(xform.GetPrim(), true);

    // workaround for hydra update not working if op value is set in later update.
    UsdGeomXformOp op = xform.AddTransformOp();
    op.Set(GfMatrix4d(1.0));

    return rootProxyPath;
}

bool ProxyVisualizationManager::isActive(SdfPath actualPath)
{
    bool isAnyClientActive = false;
    for (auto& pClient : mProxyVisualizationClients)
    {
        if (pClient && pClient->isActive())
        {
            isAnyClientActive = true;
        }
    }

    return mActualActive.find(actualPath) != mActualActive.end() && isAnyClientActive;
}

bool ProxyVisualizationManager::isSelected(SdfPath actualPath)
{
    return mActualSelected.find(actualPath) != mActualSelected.end();
}

ProxyInfo* ProxyVisualizationManager::getProxyInfo(SdfPath actualPath)
{
    auto itMap = mActualToProxyInfoMap.find(actualPath);
    return (itMap != mActualToProxyInfoMap.end()) ? itMap->second : nullptr;
}

void ProxyVisualizationManager::addProxy(SdfPath actualPath, ProxyInfo& proxyInfo)
{
    mActualToProxyInfoMap.insert({actualPath, &proxyInfo});
    mActualTable.insert({actualPath, Empty()});
}

void ProxyVisualizationManager::removeProxy(SdfPath actualPath, ProxyInfo* proxyInfo)
{
    if (proxyInfo)
    {
        removeAllProxyPrims(actualPath, *proxyInfo);
        delete proxyInfo;
    }

    mActualToProxyInfoMap.erase(actualPath);
    mActualTable.erase(actualPath);
    mActualActive.erase(actualPath);
    mActualSelected.erase(actualPath);
    mBufferActualToUpdateActive.erase(actualPath);
}

void ProxyVisualizationManager::addProxyPrim(SdfPath actualPath, SdfPath proxyPrimPath)
{
    if (!actualPath.IsEmpty() && !proxyPrimPath.IsEmpty())
    {
        mProxyToActualPrimMap.insert({proxyPrimPath, actualPath});
        auto it = mActualToProxyPrimsMap.find(actualPath);
        if (it != mActualToProxyPrimsMap.end())
        {
            it->second.insert(proxyPrimPath);
        }
        else
        {
            mActualToProxyPrimsMap.insert({actualPath, {proxyPrimPath}});
        }
    }
}

void ProxyVisualizationManager::removeProxyPrim(SdfPath actualPath, SdfPath proxyPrimPath)
{
    if (!proxyPrimPath.IsEmpty())
    {
        mProxyToActualPrimMap.erase(proxyPrimPath);
        auto it = mActualToProxyPrimsMap.find(actualPath);
        if (it != mActualToProxyPrimsMap.end())
        {
            it->second.erase(proxyPrimPath);
            if (it->second.empty())
            {
                mActualToProxyPrimsMap.erase(it);
            }
        }

        ProxyInfo* proxyInfo = getProxyInfo(actualPath);
        if (proxyInfo && proxyInfo->client)
            proxyInfo->client->notifyReleaseProxyPrim(proxyPrimPath);

        UsdPrim proxyPrim = gStage->GetPrimAtPath(proxyPrimPath);
        if (proxyPrim)
        {
            gStage->RemovePrim(proxyPrimPath);
        }
    }
}

void ProxyVisualizationManager::bufferUpdateActive(SdfPath actualPath)
{
    if (!actualPath.IsEmpty())
    {
        mBufferActualToUpdateActive.insert(actualPath);
    }
}

void ProxyVisualizationManager::clear()
{
    mActualActive.clear();
    mActualSelected.clear();
    mSessionLayerProxiesRoot = SdfPath();
    clearBuffers();
}

void ProxyVisualizationManager::clearBuffers()
{
    mBufferSelectionDirty = false;
    mBufferActualToUpdateActive.clear();
    mBufferActualToUpdatePurpose.clear();
}

SdfPath ProxyVisualizationManager::getProxyPath(SdfPath actualPath)
{
    // session layer root is only created if necessary
    if (mSessionLayerProxiesRoot.IsEmpty())
    {
        mSessionLayerProxiesRoot = SdfPath("/PhysxProxiesVisualization");
        CARB_ASSERT(!gStage->GetPrimAtPath(mSessionLayerProxiesRoot));
        UsdGeomScope rootScope = UsdGeomScope::Define(gStage, mSessionLayerProxiesRoot);
        CARB_ASSERT(rootScope);

        omni::kit::EditorUsd::setNoDelete(rootScope.GetPrim(), true);
        omni::kit::EditorUsd::setHideInStageWindow(rootScope.GetPrim(), true);
        omni::kit::EditorUsd::setNoSelectionOutline(rootScope.GetPrim(), true);
    }

    // create the path:
    std::string nameString = actualPath.GetString();
    std::replace(nameString.begin(), nameString.end(), '/', '_');
    SdfPath sessionPath = mSessionLayerProxiesRoot.AppendElementString(nameString);
    return sessionPath;
}

bool ProxyVisualizationManager::updateActive(SdfPath actualPath)
{
    ProxyInfo* proxyInfo = getProxyInfo(actualPath);
    if (!proxyInfo || !proxyInfo->client)
    {
        return false;
    }

    bool active = true;

    //1. user visibility
    UsdGeomImageable actualImg = UsdGeomImageable::Get(gStage, actualPath);
    if (actualImg)
    {
        if (actualImg.ComputeVisibility() == UsdGeomTokens.Get()->invisible)
        {
            active = false;
        }
    }

    //2. test mode/selection
    if (active && !proxyInfo->client->checkMode(actualPath))
    {
        active = false;
    }

    //3. check completeness on the level of the actual prim
    if (active && !proxyInfo->client->checkCompleteness(*proxyInfo))
    {
        active = false;
    }

    //4. check which proxies are needed (count) and create if necessary, release if not needed
    if (active)
    {
        uint32_t numActiveProxies = proxyInfo->client->updateActiveProxies(*proxyInfo, actualPath);
        active = numActiveProxies > 0;
    }

    //enable/disable actual rendering
    mBufferActualToUpdatePurpose.insert(actualPath);

    if (!active)
    {
        removeAllProxyPrims(actualPath, *proxyInfo);
    }

    return active;
}

void ProxyVisualizationManager::removeAllProxyPrims(SdfPath actualPath, ProxyInfo& proxyInfo)
{
    auto it = mActualToProxyPrimsMap.find(actualPath);
    if (it != mActualToProxyPrimsMap.end())
    {
        SdfPathSet& proxyPrimPaths = it->second;
        for (SdfPath proxyPrimPath : proxyPrimPaths)
        {
            mProxyToActualPrimMap.erase(proxyPrimPath);
        }
        mActualToProxyPrimsMap.erase(it);
    }

    if (proxyInfo.client)
        proxyInfo.client->notifyReleaseProxyPrim(proxyInfo.proxyRootPath);

    if (gStage)
    {
        gStage->RemovePrim(proxyInfo.proxyRootPath);
    }
    proxyInfo.proxyRootPath = SdfPath();
}

void ProxyVisualizationManager::updateProxySelectionOutline(SdfPath actualPath, omni::usd::UsdContext& context, uint8_t group)
{
    auto it= mActualToProxyPrimsMap.find(actualPath);
    if (it != mActualToProxyPrimsMap.end())
    {
        for (SdfPath proxyPrimPath : it->second)
        {
            context.setSelectionGroup(group, proxyPrimPath.GetString());
        }
    }
}
