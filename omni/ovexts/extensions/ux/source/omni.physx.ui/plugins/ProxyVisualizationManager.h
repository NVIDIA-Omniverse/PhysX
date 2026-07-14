// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
        eVolumeDeformableMesh,
        eVolumeDeformableSkinMesh,
        eSurfaceDeformableMesh,
        eSurfaceDeformableSkinMesh,
        eVtxXformAttachment,
        eVtxTetAttachment,
        eElementCollisionFilter,
        eNone
    };
};

struct ProxyInfo
{
    class ProxyVisualizationClient* client;
    ProxyInfoType::Enum type = ProxyInfoType::eNone;
    PXR_NS::SdfPath proxyRootPath;

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
    virtual bool needsStageParse(PXR_NS::UsdStageWeakPtr stage) = 0;

    virtual void handlePrimResync(PXR_NS::SdfPath path) = 0;
    virtual void handlePrimRemove(PXR_NS::SdfPath path) = 0;
    virtual void handleTransformChange(PXR_NS::SdfPath path) = 0;
    virtual void handleVisibilityChange(PXR_NS::SdfPath path) = 0;

    virtual void updateTracking() = 0;
    virtual void updateModeDirty() = 0;
    virtual bool checkMode(PXR_NS::SdfPath actualPath) = 0;
    virtual bool checkCompleteness(ProxyInfo& proxyInfo) = 0;
    virtual uint32_t updateActiveProxies(ProxyInfo& proxyInfo, PXR_NS::SdfPath actualPath) = 0;
    virtual void updateProxyProperties(PXR_NS::UsdGeomXformCache& xformCache) = 0;
    virtual void updateActualPurpose(PXR_NS::SdfPath actualPath, bool active) = 0;
    virtual void notifyReleaseProxyPrim(PXR_NS::SdfPath proxyPrimPath) = 0;
    virtual void clearBuffers() = 0;
};

class ProxyVisualizationManager
{
public:
    struct Empty
    {
    };
    using SdfPathActualTable = PXR_NS::SdfPathTable<Empty>;

    explicit ProxyVisualizationManager();
    ~ProxyVisualizationManager();

    void addClient(ProxyVisualizationClient& client);

    bool isActive();
    bool isEmpty();

    void handlePrimResync(PXR_NS::SdfPath path);
    void handlePrimRemove(PXR_NS::SdfPath path);
    void handleAttributeChange(PXR_NS::SdfPath path, PXR_NS::TfToken attributeName, bool isXform);

    void parseStage();
    void update();
    void selectionChanged();
    void release();

    // methods for clients

    // registers change events
    void registerAttribute(PXR_NS::TfToken attributeName,
                           ProxyVisualizationClient& client,
                           void (*callback)(ProxyVisualizationClient&, PXR_NS::SdfPath, PXR_NS::TfToken));

    // create proxy root prim based on actual path
    PXR_NS::SdfPath createProxyRootPrim(PXR_NS::SdfPath actualPath);

    // returns whether debug viz is active
    bool isActive(PXR_NS::SdfPath actualPath);

    // returns whether actual is selected
    bool isSelected(PXR_NS::SdfPath actualPath);

    // returns proxy info for actual
    ProxyInfo* getProxyInfo(PXR_NS::SdfPath actualPath);

    // adds a new proxy to the manager
    void addProxy(PXR_NS::SdfPath actualPath, ProxyInfo& proxyInfo);

    // removes proxy from manager and and deletes it
    void removeProxy(PXR_NS::SdfPath actualPath, ProxyInfo* proxyInfo);

    // adds a new proxy prim to the manager
    void addProxyPrim(PXR_NS::SdfPath actualPath, PXR_NS::SdfPath proxyPrimPath);

    // removes a proxy prim from the manager and removes it from the stage
    void removeProxyPrim(PXR_NS::SdfPath actualPath, PXR_NS::SdfPath proxyPrimPath);

    // schedule active/inactive update
    void bufferUpdateActive(PXR_NS::SdfPath actualPath);

private:
    // called by release
    void clear();

    // clears all buffered updates
    void clearBuffers();

    // update helpers
    PXR_NS::SdfPath getProxyPath(PXR_NS::SdfPath actualPath);
    bool updateActive(PXR_NS::SdfPath actualPath);
    void removeAllProxyPrims(PXR_NS::SdfPath actualPath, ProxyInfo& proxyInfo);
    void updateProxySelectionOutline(PXR_NS::SdfPath actualPath, omni::usd::UsdContext& context, uint8_t group);

    struct ProxyChangeEvent
    {
        ProxyVisualizationClient* client;
        void (*callback)(ProxyVisualizationClient&, PXR_NS::SdfPath, PXR_NS::TfToken);
    };

    using SdfPathToPathSetMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, PXR_NS::SdfPathSet, PXR_NS::SdfPath::Hash>;
    using SdfPathToPathMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, PXR_NS::SdfPath, PXR_NS::SdfPath::Hash>;
    using SdfPathToProxyInfoMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, ProxyInfo*, PXR_NS::SdfPath::Hash>;
    using ProxyChangeEventMap = PXR_NS::TfHashMap<PXR_NS::TfToken, std::vector<ProxyChangeEvent>, PXR_NS::TfToken::HashFunctor>;

    PXR_NS::VtArray<ProxyVisualizationClient*> mProxyVisualizationClients;

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
    PXR_NS::SdfPathSet mActualSelected; // all actual objects that are also selected
    PXR_NS::SdfPathSet mActualActive; // all actual objects that are active

    // ----------------
    // event buffering
    // ----------------
    // buffers to store notifications until next stage update, or batch notifications in the update itself
    bool mBufferSelectionDirty;
    PXR_NS::SdfPathSet mBufferActualToUpdateActive;
    PXR_NS::SdfPathSet mBufferActualToUpdatePurpose;

    PXR_NS::UsdGeomXformCache mXformCache;
    PXR_NS::SdfPath mSessionLayerProxiesRoot; // path to session layer scope "folder" for session proxy prims
};

} // namespace ui
} // namespace physx
} // namespace omni
