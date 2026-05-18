// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include <private/omni/physx/ui/VisualizerMode.h>
#include "ProxyVisualizationManager.h"

namespace omni
{

namespace physx
{

namespace ui
{

enum struct DeformableVisualizerMeshType : char
{
    eSimulationDefault = 0,
    eSimulationBind = 1,
    eSimulationRestShape = 2,
    eCollisionDefault = 3,
    eCollisionBind = 4,
};

class DeformableBodyVisualizationManager : public ProxyVisualizationClient
{
public:
    explicit DeformableBodyVisualizationManager(ProxyVisualizationManager& proxyVisualizationManager,
                                                const VisualizerMode mode,
                                                const DeformableVisualizerMeshType meshType,
                                                bool displayDeformableAttachments);

    void setMode(const VisualizerMode mode);
    void setMeshType(const DeformableVisualizerMeshType meshType);
    void displayDeformableAttachments(bool enable);

    void onAttributeChange_DeformableBodyPoints(pxr::SdfPath path, pxr::TfToken attributeName);
    void onAttributeChange_DeformableBodyTopology(pxr::SdfPath path, pxr::TfToken attributeName);
    void onAttributeChange_DeformableAttachmentTargetPoints(pxr::SdfPath path, pxr::TfToken attributeName);
    void onAttributeChange_DeformableAttachmentTopology(pxr::SdfPath path, pxr::TfToken attributeName);

    void onAttributeChange_DeformableCollisionFilterTopology(pxr::SdfPath path, pxr::TfToken attributeName);

protected:
    virtual ~DeformableBodyVisualizationManager();

    // ProxyVisualizationClient implementation
    virtual void release();

    virtual bool isActive()
    {
        return mMode != VisualizerMode::eNone;
    }
    virtual bool isEmpty()
    {
        return mActualDeformables.empty() && mActualAttachments.empty();
    }
    virtual bool needsStageParse(pxr::UsdStageWeakPtr stage);

    virtual void handlePrimResync(pxr::SdfPath path);
    virtual void handlePrimRemove(pxr::SdfPath path);
    virtual void handleTransformChange(pxr::SdfPath path);
    virtual void handleVisibilityChange(pxr::SdfPath path);

    virtual void updateTracking();
    virtual void updateModeDirty();
    virtual bool checkMode(pxr::SdfPath actualDeformableBodyPath);
    virtual bool checkCompleteness(ProxyInfo& proxyInfo);
    virtual uint32_t updateActiveProxies(ProxyInfo& proxyInfo, pxr::SdfPath actualPath);
    virtual void updateProxyProperties(pxr::UsdGeomXformCache& xformCache);
    virtual void updateActualPurpose(pxr::SdfPath actualDeformableBodyPath, bool active);
    virtual void notifyReleaseProxyPrim(pxr::SdfPath proxyPrimPath);
    virtual void clearBuffers();
    //~ProxyVisualizationClient implementation

private:
    struct DeformableMeshProxyInfo : public ProxyInfo
    {
        pxr::SdfPath actualMeshPath;
        pxr::SdfPath actualDeformableBodyPath;

        virtual ~DeformableMeshProxyInfo()
        {
        }
    };

    struct BaseTargetInfo
    {
        enum Type
        {
            eVolumeDeformableBody,
            eSurfaceDeformableBody,
            eXformable,
            eNone
        };

        Type type = eNone;

        virtual ~BaseTargetInfo() {}
    };

    // attachment targets are handled without the proxy manager.
    struct AttachmentTargetInfo : public BaseTargetInfo
    {
        float pointScale = 0.0f;
        uint32_t numAttachments = 0;

        virtual ~AttachmentTargetInfo()
        {
        }
    };

    // collision filter targets are handled without the proxy manager.
    struct CollisionFilterTargetInfo : public BaseTargetInfo
    {
        uint32_t numCollisionFilters = 0;

        virtual ~CollisionFilterTargetInfo()
        {
        }
    };

    using TargetPathPair = std::array<pxr::SdfPath, 2>;

    struct AttachmentProxyInfo : public ProxyInfo
    {
        TargetPathPair actualTargets = { pxr::SdfPath(), pxr::SdfPath() };
        uint32_t numPointSamples[2] = { 0, 0 };
        AttachmentTargetInfo* targetInfos[2];
        pxr::SdfPath actualAttachmentPath;

        virtual ~AttachmentProxyInfo()
        {
        }
    };

    struct CollisionFilterProxyInfo : public ProxyInfo
    {
        TargetPathPair actualTargets = { pxr::SdfPath(), pxr::SdfPath() };
        CollisionFilterTargetInfo* collisionFilterTargetInfos[2];
        pxr::SdfPath actualCollisionFilterPath;

        virtual ~CollisionFilterProxyInfo()
        {
        }
    };

    // show wrappers
    bool showProxySimulationMesh()
    {
        return isActive() && (mMeshType == DeformableVisualizerMeshType::eSimulationDefault ||
                              mMeshType == DeformableVisualizerMeshType::eSimulationBind);
    }
    bool showProxyRestShape()
    {
        return isActive() && mMeshType == DeformableVisualizerMeshType::eSimulationRestShape;
    }
    bool showProxyCollisionMesh()
    {
        return isActive() && (mMeshType == DeformableVisualizerMeshType::eCollisionDefault ||
                              mMeshType == DeformableVisualizerMeshType::eCollisionBind);
    }
    bool showDefaultPose()
    {
        return isActive() && (mMeshType == DeformableVisualizerMeshType::eSimulationDefault ||
                              mMeshType == DeformableVisualizerMeshType::eCollisionDefault);
    }
    bool showBindPose()
    {
        return isActive() && (mMeshType == DeformableVisualizerMeshType::eSimulationBind ||
                              mMeshType == DeformableVisualizerMeshType::eCollisionBind);
    }

    pxr::UsdPrim createDeformableBodyProxyPrim(pxr::SdfPath deformableBodyProxyPath, ProxyInfoType::Enum type);
    void releaseDeformableMeshVisualizer(pxr::SdfPath proxyPath);

    DeformableMeshProxyInfo* createDeformableMeshProxyInfo(const pxr::SdfPath actualMeshPath, const pxr::SdfPath actualDeformableBodyPath, const ProxyInfoType::Enum& type);
    DeformableMeshProxyInfo* getDeformableMeshProxyInfo(const pxr::SdfPath actualPath);

    // update tracking for deformables
    void addDeformableBody(const pxr::SdfPath actualDeformableBodyPath);
    void removeDeformableBody(const pxr::SdfPath actualDeformableBodyPath, bool updateActual);
    void updateDeformableBody(const pxr::SdfPath actualDeformableBodyPath);

    // update tracking for attachments
    void addAttachment(const pxr::SdfPath attachmentPath);
    void removeAttachment(const pxr::SdfPath attachmentPath);
    void updateAttachment(const pxr::SdfPath attachmentPath);

    // update tracking for collision filters
    void addCollisionFilter(const pxr::SdfPath collisionFilterPath);
    void removeCollisionFilter(const pxr::SdfPath collisionFilterPath);
    void updateCollisionFilter(const pxr::SdfPath collisionFilterPath);

    // functions for deformables
    void updateTransform(const pxr::SdfPath actualDeformableBodyPath, pxr::UsdGeomXformCache& xformCache);
    void updatePoints(const pxr::SdfPath actualDeformableBodyPath);
    void updateTopology(const pxr::SdfPath actualDeformableBodyPath);
    void updateGap(const pxr::SdfPath actualDeformableBodyPath);
    void clearDeformableBodies(bool updateSkin);
    bool checkDeformableCompleteness(ProxyInfo& proxyInfo);
    class DeformableMeshVisualizer* getDeformableMeshVisualizer(const pxr::SdfPath sessionTetPath);

    pxr::SdfPath getSimMeshPathFromMap(const pxr::SdfPath actualDeformableBodyPath);
    pxr::SdfPath getCollMeshPathFromMap(const pxr::SdfPath actualDeformableBodyPath);
    pxr::SdfPathSet getSkinGeomPathsFromMap(const pxr::SdfPath actualDeformableBodyPath);

    void getFilteredElementIndices(TargetPathPair& targetPaths, pxr::VtArray<uint32_t>(&filterTriIds)[2],
        const pxr::SdfPath targetCollisionFilter);

    // functions for attachments
    pxr::SdfPath getProxyAttachmentPath(const pxr::SdfPath attachmentPath);
    void createProxyAttachmentGeometries(const pxr::SdfPath attachmentPath);
    void setupProxyAttachmentGeometries(const pxr::SdfPath attachmentPath, const float visualizationScale);
    void createSessionPointInstancer(const pxr::SdfPath sessionPointInstancerPath);
    void setupSessionPointInstancer(const pxr::SdfPath pointInstancerPath, const float radius, const pxr::GfVec3f& color);
    void resetSessionPointInstancer(const pxr::SdfPath pointInstancerPath);
    void updateSessionPointInstancerRadius(const pxr::SdfPath pointInstancerPath, const float radius);

    pxr::UsdPrim createProxyPrim(pxr::SdfPath proxyAttachmentPath, ProxyInfoType::Enum type);

    AttachmentProxyInfo* createAttachmentProxyInfo(pxr::SdfPath attachmentPath);
    AttachmentProxyInfo* getAttachmentProxyInfo(pxr::SdfPath attachmentPath);

    AttachmentTargetInfo* createAttachmentTargetInfo(const pxr::SdfPath targetPath);
    void releaseTargetInfo(BaseTargetInfo* targetInfo);
    AttachmentTargetInfo* getAttachmentTargetInfo(pxr::SdfPath targetPath);
    float getPointScale(const pxr::SdfPath attachmentPath, const float visualizationScale);
    float getPointScale(const pxr::UsdPrim& targetMeshPrim, const pxr::VtArray<pxr::GfVec3f>& restPoints) const;

    void updateAttachmentGeometry(const pxr::SdfPath attachmentPath, const float visualizationScale);
    void updateAttachmentTargetTransform(const pxr::SdfPath targetPath, pxr::UsdGeomXformCache& xformCache);
    void updateAttachmentTargetGeometry(const pxr::SdfPath targetPath);
    void updateVisualizationScale(const pxr::SdfPath attachmentPath, const float visualizationScale);
    uint32_t updateVolumeDeformableTargetAttachmentSamplePoints(pxr::UsdGeomPointInstancer& samplePointInstancer,
                                                                const pxr::SdfPath attachmentPath,
                                                                const pxr::SdfPath& targetPath,
                                                                const int slot,
                                                                AttachmentProxyInfo* attachmentInfo);
    uint32_t updateSurfaceDeformableTargetAttachmentSamplePoints(pxr::UsdGeomPointInstancer& samplePointInstancer,
                                                                 const pxr::SdfPath attachmentPath,
                                                                 const pxr::SdfPath& targetPath,
                                                                 const int slot,
                                                                 AttachmentProxyInfo* attachmentInfo);

    void clearAttachments();
    bool checkAttachmentsCompleteness(ProxyInfo& proxyInfo);
    uint32_t updateActiveProxiesAttachments(ProxyInfo& proxyInfo,
                                            pxr::SdfPath actualPath,
                                            bool isActive,
                                            const float visualizationScale);
    void clearAttachmentsBuffers();

    // functions for collisionFilters
    pxr::SdfPath getProxyCollisionFilterPath(const pxr::SdfPath collisionFilterPath);

    CollisionFilterProxyInfo* createCollisionFilterProxyInfo(pxr::SdfPath collisionFilterPath);
    CollisionFilterProxyInfo* getCollisionFilterProxyInfo(pxr::SdfPath collisionFilterPath);

    CollisionFilterTargetInfo* createCollisionFilterTargetInfo(const pxr::SdfPath targetPath);
    CollisionFilterTargetInfo* getCollisionFilterTargetInfo(pxr::SdfPath targetPath);

    void updateCollisionFilterTargetTransform(const pxr::SdfPath targetPath, pxr::UsdGeomXformCache& xformCache);

    void clearCollisionFilters();
    bool checkCollisionFiltersCompleteness(ProxyInfo& proxyInfo);
    uint32_t updateActiveProxiesCollisionFilters(ProxyInfo& proxyInfo,
        pxr::SdfPath actualPath,
        bool isActive);

    void clearCollisionFiltersBuffers();

    void updateBufferDeformablePathsToUpdateTopology(const pxr::SdfPath targetPath);

    // class-scope using:
    using SdfPathToSdfPathMap = pxr::TfHashMap<pxr::SdfPath, pxr::SdfPath, pxr::SdfPath::Hash>;
    using SdfPathToDeformableMeshVisualizerMap =
        pxr::TfHashMap<pxr::SdfPath, DeformableMeshVisualizer*, pxr::SdfPath::Hash>;
    using SdfPathToTargetInfoMap = pxr::TfHashMap<pxr::SdfPath, AttachmentTargetInfo*, pxr::SdfPath::Hash>;
    using SdfPathToSdfPathSetMap = pxr::TfHashMap<pxr::SdfPath, pxr::SdfPathSet, pxr::SdfPath::Hash>;

    using SdfPathToCollisionFilterTargetInfoMap = pxr::TfHashMap<pxr::SdfPath, CollisionFilterTargetInfo*, pxr::SdfPath::Hash>;

    // ----------------
    // tracking for deformables
    // ----------------
    // applies to all actual objects of interest in the stage, and is maintained if visualizer mode is "Selected" or
    // "All".
    pxr::SdfPathSet mActualDeformables;
    SdfPathToSdfPathMap mActualObjectToDeformableBodyMap; // path hashmap from actual object to deformable body (The
                                                          // prim with PhysicsDeformableBodyAPI)
    SdfPathToSdfPathMap mDeformableBodyToActualSimMeshMap; // path hashmap from deformable body to simulation mesh
    SdfPathToSdfPathMap mDeformableBodyToActualCollMeshMap; // path hashmap from deformable body to collision mesh
    SdfPathToSdfPathSetMap mDeformableBodyToActualSkinGeomsMap; // path hashmap from deformable body to skin meshes
    SdfPathToDeformableMeshVisualizerMap mProxyToDeformableMeshVisualizerMap; // path hashmap from session proxy
                                                                              // tets/tris to deformable mesh
                                                                              // visualizers

    // ----------------
    // tracking for attachments
    // ----------------
    pxr::SdfPathSet mActualAttachments;
    SdfPathToSdfPathSetMap mTargetToActualAttachments; // path map from targets in mActualAttachments to their attachments
    SdfPathToTargetInfoMap mTargetToAttachmentInfo; // path map from targets of valid attachments to their info

    // ----------------
    // tracking for collision filters
    // ----------------
    pxr::SdfPathSet mActualCollisionFilters;
    SdfPathToSdfPathSetMap mTargetToActualCollisionFilters; // path map from targets in mActualCollisionFilters to their collision filters
    SdfPathToCollisionFilterTargetInfoMap mTargetToCollisionFilterInfo; // path map from targets of valid collision filters to their info

    float mVisualizationGap;
    float mVisualizationScale;
    ProxyVisualizationManager& mProxyManager;

    // ----------------
    // active state
    // ----------------
    // data structures that are maintained only for active objects that are being visualized with session proxies
    // or which is relevant for filter evaluation
    VisualizerMode mMode; // mode for visualization [none, all, selected]
    DeformableVisualizerMeshType mMeshType; // type of mesh to visualize
    bool mDisplayDeformableAttachments;

    // ----------------
    // event buffering
    // ----------------
    // buffers to store updates until next stage update that calls the manager's update function
    pxr::SdfPathSet mBufferDeformablePathsToAdd;
    pxr::SdfPathSet mBufferDeformablePathsToUpdate;
    pxr::SdfPathSet mBufferDeformablePathsToRemove;
    pxr::SdfPathSet mBufferDeformablePathsToUpdateTransform;
    pxr::SdfPathSet mBufferDeformablePathsToUpdatePoints;
    pxr::SdfPathSet mBufferDeformablePathsToUpdateTopology;
    pxr::SdfPathSet mBufferDeformablePathsToUpdateGap;

    pxr::SdfPathSet mBufferAttachmentPathsToAdd;
    pxr::SdfPathSet mBufferAttachmentPathsToUpdate;
    pxr::SdfPathSet mBufferAttachmentPathsToRemove;
    pxr::SdfPathSet mBufferAttachmentPathsToUpdateGeometry;
    pxr::SdfPathSet mBufferAttachmentPathsToUpdateVizAttribute;

    pxr::SdfPathSet mBufferCollisionFilterPathsToAdd;
    pxr::SdfPathSet mBufferCollisionFilterPathsToUpdate;
    pxr::SdfPathSet mBufferCollisionFilterPathsToRemove;

    pxr::SdfPathSet mBufferTargetPathsToUpdateTransform;
    pxr::SdfPathSet mBufferTargetPathsToUpdateGeometry;

    pxr::SdfPathSet mBufferCollisionFilterTargetPathsToUpdateTransform;

    bool mBufferModeDirtyDeformable;
    bool mBufferModeDirtyAttachments;
    bool mBufferModeDirtyCollisionFilters;
};

} // namespace ui
} // namespace physx
} // namespace omni
