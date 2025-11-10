// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "UsdPCH.h"

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/PhysxUsd.h>
#include <private/omni/physx/ui/VisualizerMode.h>
#include <private/omni/physics/schema/IUsdPhysics.h>

namespace omni
{

namespace usd
{
class UsdContext;
}

namespace physx
{

namespace ui
{

struct ProxyInfoType
{
    enum Enum
    {
        eParticleSystem,
        eParticleSet,
        eParticleCloth,
        eVolumeDeformableMesh,
        eVolumeDeformableSkinMesh,
        eSurfaceDeformableMesh,
        eSurfaceDeformableSkinMesh,
        eVtxXformAttachment,
        eVtxTetAttachment,
        eNone
    };
};

struct ProxyInfo
{
    class ProxyVisualizationClient* client;
    ProxyInfoType::Enum type = ProxyInfoType::eNone;
    pxr::SdfPath proxyRootPath;

    virtual ~ProxyInfo()
    {
    }
};

class ProxyVisualizationClient
{
public:
    virtual ~ProxyVisualizationClient()
    {
    }
    virtual void release() = 0;

    virtual bool isActive() = 0;
    virtual bool isEmpty() = 0;
    virtual bool needsStageParse(pxr::UsdStageWeakPtr stage) = 0;

    virtual void handlePrimResync(pxr::SdfPath path) = 0;
    virtual void handlePrimRemove(pxr::SdfPath path) = 0;
    virtual void handleTransformChange(pxr::SdfPath path) = 0;
    virtual void handleVisibilityChange(pxr::SdfPath path) = 0;

    virtual void updateTracking() = 0;
    virtual void updateModeDirty() = 0;
    virtual bool checkMode(pxr::SdfPath actualPath) = 0;
    virtual bool checkCompleteness(ProxyInfo& proxyInfo) = 0;
    virtual uint32_t updateActiveProxies(ProxyInfo& proxyInfo, pxr::SdfPath actualPath) = 0;
    virtual void updateProxyProperties(pxr::UsdGeomXformCache& xformCache) = 0;
    virtual void updateActualPurpose(pxr::SdfPath actualPath, bool active) = 0;
    virtual void notifyReleaseProxyPrim(pxr::SdfPath proxyPrimPath) = 0;
    virtual void clearBuffers() = 0;
};

class ProxyVisualizationManager
{
public:
    struct Empty
    {
    };
    using SdfPathActualTable = pxr::SdfPathTable<Empty>;

    explicit ProxyVisualizationManager();
    ~ProxyVisualizationManager();

    void addClient(ProxyVisualizationClient& client);

    bool isActive();
    bool isEmpty();

    void handlePrimResync(pxr::SdfPath path);
    void handlePrimRemove(pxr::SdfPath path);
    void handleAttributeChange(pxr::SdfPath path, pxr::TfToken attributeName, bool isXform);

    void parseStage();
    void update();
    void selectionChanged();
    void release();

    // methods for clients

    // registers change events
    void registerAttribute(pxr::TfToken attributeName,
                           ProxyVisualizationClient& client,
                           void (*callback)(ProxyVisualizationClient&, pxr::SdfPath, pxr::TfToken));

    // create proxy root prim based on actual path
    pxr::SdfPath createProxyRootPrim(pxr::SdfPath actualPath);

    // returns whether debug viz is active
    bool isActive(pxr::SdfPath actualPath);

    // returns whether actual is selected
    bool isSelected(pxr::SdfPath actualPath);

    // returns proxy info for actual
    ProxyInfo* getProxyInfo(pxr::SdfPath actualPath);

    // adds a new proxy to the manager
    void addProxy(pxr::SdfPath actualPath, ProxyInfo& proxyInfo);

    // removes proxy from manager and and deletes it
    void removeProxy(pxr::SdfPath actualPath, ProxyInfo* proxyInfo);

    // adds a new proxy prim to the manager
    void addProxyPrim(pxr::SdfPath actualPath, pxr::SdfPath proxyPrimPath);

    // removes a proxy prim from the manager and removes it from the stage
    void removeProxyPrim(pxr::SdfPath actualPath, pxr::SdfPath proxyPrimPath);

    // schedule active/inactive update
    void bufferUpdateActive(pxr::SdfPath actualPath);

private:
    // called by release
    void clear();

    // clears all buffered updates
    void clearBuffers();

    // update helpers
    pxr::SdfPath getProxyPath(pxr::SdfPath actualPath);
    bool updateActive(pxr::SdfPath actualPath);
    void removeAllProxyPrims(pxr::SdfPath actualPath, ProxyInfo& proxyInfo);
    void updateProxySelectionOutline(pxr::SdfPath actualPath, omni::usd::UsdContext& context, uint8_t group);

    struct ProxyChangeEvent
    {
        ProxyVisualizationClient* client;
        void (*callback)(ProxyVisualizationClient&, pxr::SdfPath, pxr::TfToken);
    };

    using SdfPathToPathSetMap = pxr::TfHashMap<pxr::SdfPath, pxr::SdfPathSet, pxr::SdfPath::Hash>;
    using SdfPathToPathMap = pxr::TfHashMap<pxr::SdfPath, pxr::SdfPath, pxr::SdfPath::Hash>;
    using SdfPathToProxyInfoMap = pxr::TfHashMap<pxr::SdfPath, ProxyInfo*, pxr::SdfPath::Hash>;
    using ProxyChangeEventMap = pxr::TfHashMap<pxr::TfToken, std::vector<ProxyChangeEvent>, pxr::TfToken::HashFunctor>;

    pxr::VtArray<ProxyVisualizationClient*> mProxyVisualizationClients;

    // ----------------
    // tracking
    // ----------------
    // applies to all actual objects of interest in the stage, and is maintained if visualizer mode is "Selected" or
    // "All".
    SdfPathToProxyInfoMap mActualToProxyInfoMap; // path map from actual to their infos
    SdfPathActualTable mActualTable; // path table including ancestors, mapping not used
    ProxyChangeEventMap mChangeEventMap;

    // ----------------
    // active state
    // ----------------
    // data structures that are maintained only for active objects that are being visualized with session proxies
    // or which is relevant for filter evaluation
    SdfPathToPathMap mProxyToActualPrimMap; // path map from proxy session primitives to actual objects
    SdfPathToPathSetMap mActualToProxyPrimsMap; // path map from actual path to proxy prim paths
    pxr::SdfPathSet mActualSelected; // all actual objects that are also selected
    pxr::SdfPathSet mActualActive; // all actual objects that are active

    // ----------------
    // event buffering
    // ----------------
    // buffers to store notifications until next stage update, or batch notifications in the update itself
    bool mBufferSelectionDirty;
    pxr::SdfPathSet mBufferActualToUpdateActive;
    pxr::SdfPathSet mBufferActualToUpdatePurpose;

    pxr::UsdGeomXformCache mXformCache;
    pxr::SdfPath mSessionLayerProxiesRoot; // path to session layer scope "folder" for session proxy prims
};

} // namespace ui
} // namespace physx
} // namespace omni
