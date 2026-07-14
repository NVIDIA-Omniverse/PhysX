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

    void onAttributeChange_DeformableBodyPoints(PXR_NS::SdfPath path, PXR_NS::TfToken attributeName);
    void onAttributeChange_DeformableBodyTopology(PXR_NS::SdfPath path, PXR_NS::TfToken attributeName);
    void onAttributeChange_DeformableAttachmentTargetPoints(PXR_NS::SdfPath path, PXR_NS::TfToken attributeName);
    void onAttributeChange_DeformableAttachmentTopology(PXR_NS::SdfPath path, PXR_NS::TfToken attributeName);

    void onAttributeChange_DeformableCollisionFilterTopology(PXR_NS::SdfPath path, PXR_NS::TfToken attributeName);

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
    virtual bool needsStageParse(PXR_NS::UsdStageWeakPtr stage);

    virtual void handlePrimResync(PXR_NS::SdfPath path);
    virtual void handlePrimRemove(PXR_NS::SdfPath path);
    virtual void handleTransformChange(PXR_NS::SdfPath path);
    virtual void handleVisibilityChange(PXR_NS::SdfPath path);

    virtual void updateTracking();
    virtual void updateModeDirty();
    virtual bool checkMode(PXR_NS::SdfPath actualDeformableBodyPath);
    virtual bool checkCompleteness(ProxyInfo& proxyInfo);
    virtual uint32_t updateActiveProxies(ProxyInfo& proxyInfo, PXR_NS::SdfPath actualPath);
    virtual void updateProxyProperties(PXR_NS::UsdGeomXformCache& xformCache);
    virtual void updateActualPurpose(PXR_NS::SdfPath actualDeformableBodyPath, bool active);
    virtual void notifyReleaseProxyPrim(PXR_NS::SdfPath proxyPrimPath);
    virtual void clearBuffers();
    //~ProxyVisualizationClient implementation

private:
    struct DeformableMeshProxyInfo : public ProxyInfo
    {
        PXR_NS::SdfPath actualMeshPath;
        PXR_NS::SdfPath actualDeformableBodyPath;

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

    using TargetPathPair = std::array<PXR_NS::SdfPath, 2>;

    struct AttachmentProxyInfo : public ProxyInfo
    {
        TargetPathPair actualTargets = { PXR_NS::SdfPath(), PXR_NS::SdfPath() };
        uint32_t numPointSamples[2] = { 0, 0 };
        AttachmentTargetInfo* targetInfos[2];
        PXR_NS::SdfPath actualAttachmentPath;

        virtual ~AttachmentProxyInfo()
        {
        }
    };

    struct CollisionFilterProxyInfo : public ProxyInfo
    {
        TargetPathPair actualTargets = { PXR_NS::SdfPath(), PXR_NS::SdfPath() };
        CollisionFilterTargetInfo* collisionFilterTargetInfos[2];
        PXR_NS::SdfPath actualCollisionFilterPath;

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

    PXR_NS::UsdPrim createDeformableBodyProxyPrim(PXR_NS::SdfPath deformableBodyProxyPath, ProxyInfoType::Enum type);
    void releaseDeformableMeshVisualizer(PXR_NS::SdfPath proxyPath);

    DeformableMeshProxyInfo* createDeformableMeshProxyInfo(const PXR_NS::SdfPath actualMeshPath, const PXR_NS::SdfPath actualDeformableBodyPath, const ProxyInfoType::Enum& type);
    DeformableMeshProxyInfo* getDeformableMeshProxyInfo(const PXR_NS::SdfPath actualPath);

    // update tracking for deformables
    void addDeformableBody(const PXR_NS::SdfPath actualDeformableBodyPath);
    void removeDeformableBody(const PXR_NS::SdfPath actualDeformableBodyPath, bool updateActual);
    void updateDeformableBody(const PXR_NS::SdfPath actualDeformableBodyPath);

    // update tracking for attachments
    void addAttachment(const PXR_NS::SdfPath attachmentPath);
    void removeAttachment(const PXR_NS::SdfPath attachmentPath);
    void updateAttachment(const PXR_NS::SdfPath attachmentPath);

    // update tracking for collision filters
    void addCollisionFilter(const PXR_NS::SdfPath collisionFilterPath);
    void removeCollisionFilter(const PXR_NS::SdfPath collisionFilterPath);
    void updateCollisionFilter(const PXR_NS::SdfPath collisionFilterPath);

    // functions for deformables
    void updateTransform(const PXR_NS::SdfPath actualDeformableBodyPath, PXR_NS::UsdGeomXformCache& xformCache);
    void updatePoints(const PXR_NS::SdfPath actualDeformableBodyPath);
    void updateTopology(const PXR_NS::SdfPath actualDeformableBodyPath);
    void updateGap(const PXR_NS::SdfPath actualDeformableBodyPath);
    void clearDeformableBodies(bool updateSkin);
    bool checkDeformableCompleteness(ProxyInfo& proxyInfo);
    class DeformableMeshVisualizer* getDeformableMeshVisualizer(const PXR_NS::SdfPath sessionTetPath);

    PXR_NS::SdfPath getSimMeshPathFromMap(const PXR_NS::SdfPath actualDeformableBodyPath);
    PXR_NS::SdfPath getCollMeshPathFromMap(const PXR_NS::SdfPath actualDeformableBodyPath);
    PXR_NS::SdfPathSet getSkinGeomPathsFromMap(const PXR_NS::SdfPath actualDeformableBodyPath);

    void getFilteredElementIndices(TargetPathPair& targetPaths, PXR_NS::VtArray<uint32_t>(&filterTriIds)[2],
        const PXR_NS::SdfPath targetCollisionFilter);

    // functions for attachments
    PXR_NS::SdfPath getProxyAttachmentPath(const PXR_NS::SdfPath attachmentPath);
    void createProxyAttachmentGeometries(const PXR_NS::SdfPath attachmentPath);
    void setupProxyAttachmentGeometries(const PXR_NS::SdfPath attachmentPath, const float visualizationScale);
    void createSessionPointInstancer(const PXR_NS::SdfPath sessionPointInstancerPath);
    void setupSessionPointInstancer(const PXR_NS::SdfPath pointInstancerPath, const float radius, const PXR_NS::GfVec3f& color);
    void resetSessionPointInstancer(const PXR_NS::SdfPath pointInstancerPath);
    void updateSessionPointInstancerRadius(const PXR_NS::SdfPath pointInstancerPath, const float radius);

    PXR_NS::UsdPrim createProxyPrim(PXR_NS::SdfPath proxyAttachmentPath, ProxyInfoType::Enum type);

    AttachmentProxyInfo* createAttachmentProxyInfo(PXR_NS::SdfPath attachmentPath);
    AttachmentProxyInfo* getAttachmentProxyInfo(PXR_NS::SdfPath attachmentPath);

    AttachmentTargetInfo* createAttachmentTargetInfo(const PXR_NS::SdfPath targetPath);
    void releaseTargetInfo(BaseTargetInfo* targetInfo);
    AttachmentTargetInfo* getAttachmentTargetInfo(PXR_NS::SdfPath targetPath);
    float getPointScale(const PXR_NS::SdfPath attachmentPath, const float visualizationScale);
    float getPointScale(const PXR_NS::UsdPrim& targetMeshPrim, const PXR_NS::VtArray<PXR_NS::GfVec3f>& restPoints) const;

    void updateAttachmentGeometry(const PXR_NS::SdfPath attachmentPath, const float visualizationScale);
    void updateAttachmentTargetTransform(const PXR_NS::SdfPath targetPath, PXR_NS::UsdGeomXformCache& xformCache);
    void updateAttachmentTargetGeometry(const PXR_NS::SdfPath targetPath);
    void updateVisualizationScale(const PXR_NS::SdfPath attachmentPath, const float visualizationScale);
    uint32_t updateVolumeDeformableTargetAttachmentSamplePoints(PXR_NS::UsdGeomPointInstancer& samplePointInstancer,
                                                                const PXR_NS::SdfPath attachmentPath,
                                                                const PXR_NS::SdfPath& targetPath,
                                                                const int slot,
                                                                AttachmentProxyInfo* attachmentInfo);
    uint32_t updateSurfaceDeformableTargetAttachmentSamplePoints(PXR_NS::UsdGeomPointInstancer& samplePointInstancer,
                                                                 const PXR_NS::SdfPath attachmentPath,
                                                                 const PXR_NS::SdfPath& targetPath,
                                                                 const int slot,
                                                                 AttachmentProxyInfo* attachmentInfo);

    void clearAttachments();
    bool checkAttachmentsCompleteness(ProxyInfo& proxyInfo);
    uint32_t updateActiveProxiesAttachments(ProxyInfo& proxyInfo,
                                            PXR_NS::SdfPath actualPath,
                                            bool isActive,
                                            const float visualizationScale);
    void clearAttachmentsBuffers();

    // functions for collisionFilters
    PXR_NS::SdfPath getProxyCollisionFilterPath(const PXR_NS::SdfPath collisionFilterPath);

    CollisionFilterProxyInfo* createCollisionFilterProxyInfo(PXR_NS::SdfPath collisionFilterPath);
    CollisionFilterProxyInfo* getCollisionFilterProxyInfo(PXR_NS::SdfPath collisionFilterPath);

    CollisionFilterTargetInfo* createCollisionFilterTargetInfo(const PXR_NS::SdfPath targetPath);
    CollisionFilterTargetInfo* getCollisionFilterTargetInfo(PXR_NS::SdfPath targetPath);

    void updateCollisionFilterTargetTransform(const PXR_NS::SdfPath targetPath, PXR_NS::UsdGeomXformCache& xformCache);

    void clearCollisionFilters();
    bool checkCollisionFiltersCompleteness(ProxyInfo& proxyInfo);
    uint32_t updateActiveProxiesCollisionFilters(ProxyInfo& proxyInfo,
        PXR_NS::SdfPath actualPath,
        bool isActive);

    void clearCollisionFiltersBuffers();

    void updateBufferDeformablePathsToUpdateTopology(const PXR_NS::SdfPath targetPath);

    // class-scope using:
    using SdfPathToSdfPathMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, PXR_NS::SdfPath, PXR_NS::SdfPath::Hash>;
    using SdfPathToDeformableMeshVisualizerMap =
        PXR_NS::TfHashMap<PXR_NS::SdfPath, DeformableMeshVisualizer*, PXR_NS::SdfPath::Hash>;
    using SdfPathToTargetInfoMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, AttachmentTargetInfo*, PXR_NS::SdfPath::Hash>;
    using SdfPathToSdfPathSetMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, PXR_NS::SdfPathSet, PXR_NS::SdfPath::Hash>;

    using SdfPathToCollisionFilterTargetInfoMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, CollisionFilterTargetInfo*, PXR_NS::SdfPath::Hash>;

    // ----------------
    // tracking for deformables
    // ----------------
    // applies to all actual objects of interest in the stage, and is maintained if visualizer mode is "Selected" or
    // "All".
    PXR_NS::SdfPathSet mActualDeformables;
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
    PXR_NS::SdfPathSet mActualAttachments;
    SdfPathToSdfPathSetMap mTargetToActualAttachments; // path map from targets in mActualAttachments to their attachments
    SdfPathToTargetInfoMap mTargetToAttachmentInfo; // path map from targets of valid attachments to their info

    // ----------------
    // tracking for collision filters
    // ----------------
    PXR_NS::SdfPathSet mActualCollisionFilters;
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
    PXR_NS::SdfPathSet mBufferDeformablePathsToAdd;
    PXR_NS::SdfPathSet mBufferDeformablePathsToUpdate;
    PXR_NS::SdfPathSet mBufferDeformablePathsToRemove;
    PXR_NS::SdfPathSet mBufferDeformablePathsToUpdateTransform;
    PXR_NS::SdfPathSet mBufferDeformablePathsToUpdatePoints;
    PXR_NS::SdfPathSet mBufferDeformablePathsToUpdateTopology;
    PXR_NS::SdfPathSet mBufferDeformablePathsToUpdateGap;

    PXR_NS::SdfPathSet mBufferAttachmentPathsToAdd;
    PXR_NS::SdfPathSet mBufferAttachmentPathsToUpdate;
    PXR_NS::SdfPathSet mBufferAttachmentPathsToRemove;
    PXR_NS::SdfPathSet mBufferAttachmentPathsToUpdateGeometry;
    PXR_NS::SdfPathSet mBufferAttachmentPathsToUpdateVizAttribute;

    PXR_NS::SdfPathSet mBufferCollisionFilterPathsToAdd;
    PXR_NS::SdfPathSet mBufferCollisionFilterPathsToUpdate;
    PXR_NS::SdfPathSet mBufferCollisionFilterPathsToRemove;

    PXR_NS::SdfPathSet mBufferTargetPathsToUpdateTransform;
    PXR_NS::SdfPathSet mBufferTargetPathsToUpdateGeometry;

    PXR_NS::SdfPathSet mBufferCollisionFilterTargetPathsToUpdateTransform;

    bool mBufferModeDirtyDeformable;
    bool mBufferModeDirtyAttachments;
    bool mBufferModeDirtyCollisionFilters;
};

} // namespace ui
} // namespace physx
} // namespace omni
