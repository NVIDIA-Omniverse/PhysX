// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

    // attachment targets are handled without the proxy manager.
    struct AttachmentTargetInfo
    {
        enum Type
        {
            eVolumeDeformableBody,
            eSurfaceDeformableBody,
            eXformable,
            eNone
        };

        Type type = eNone;
        float pointScale = 0.0f;
        uint32_t numAttachments = 0;

        virtual ~AttachmentTargetInfo()
        {
        }
    };

    using TargetPathPair = std::array<pxr::SdfPath, 2>;

    struct AttachmentProxyInfo : public ProxyInfo
    {
        TargetPathPair actualTargets = { pxr::SdfPath(), pxr::SdfPath() };
        uint32_t numPointSamples[2] = { 0, 0 };
        // uint32_t numPointFilters[2] = { 0, 0 };

        AttachmentTargetInfo* targetInfos[2];

        pxr::SdfPath actualAttachmentPath;

        virtual ~AttachmentProxyInfo()
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

    DeformableMeshProxyInfo* createDeformableMeshProxyInfo(pxr::SdfPath actualMeshPath, pxr::SdfPath actualDeformableBodyPath);
    DeformableMeshProxyInfo* getDeformableMeshProxyInfo(pxr::SdfPath actualPath);

    // update tracking for deformables
    void addDeformableBody(const pxr::SdfPath actualDeformableBodyPath);
    void removeDeformableBody(const pxr::SdfPath actualDeformableBodyPath, bool updateActual);
    void updateDeformableBody(const pxr::SdfPath actualDeformableBodyPath);

    // update tracking for attachments
    void addAttachment(const pxr::SdfPath attachmentPath);
    void removeAttachment(const pxr::SdfPath attachmentPath);
    void updateAttachment(const pxr::SdfPath attachmentPath);

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

    // functions for attachments
    pxr::SdfPath getProxyAttachmentPath(const pxr::SdfPath attachmentPath);
    void createProxyAttachmentGeometries(const pxr::SdfPath attachmentPath);
    void setupProxyAttachmentGeometries(const pxr::SdfPath attachmentPath, const float visualizationScale);
    void createSessionPointInstancer(const pxr::SdfPath sessionPointInstancerPath);
    void setupSessionPointInstancer(const pxr::SdfPath pointInstancerPath, const float radius, const pxr::GfVec3f& color);
    void resetSessionPointInstancer(const pxr::SdfPath pointInstancerPath);
    void updateSessionPointInstancerRadius(const pxr::SdfPath pointInstancerPath, const float radius);

    pxr::UsdPrim createAttachmentProxyPrim(pxr::SdfPath attachmentProxyPath, ProxyInfoType::Enum type);

    AttachmentProxyInfo* createAttachmentProxyInfo(pxr::SdfPath attachmentPath);
    AttachmentProxyInfo* getAttachmentProxyInfo(pxr::SdfPath attachmentPath);

    AttachmentTargetInfo* createTargetInfo(const pxr::SdfPath targetPath);
    void releaseTargetInfo(AttachmentTargetInfo* targetInfo);
    AttachmentTargetInfo* getTargetInfo(pxr::SdfPath targetPath);
    float getPointScale(const pxr::SdfPath attachmentPath, const float visualizationScale);
    float getPointScale(const pxr::UsdPrim& targetMeshPrim, const pxr::VtArray<pxr::GfVec3f>& restPoints) const;

    void updateAttachmentGeometry(const pxr::SdfPath attachmentPath, const float visualizationScale);
    void updateTargetTransform(const pxr::SdfPath targetPath, pxr::UsdGeomXformCache& xformCache);
    void updateTargetGeometry(const pxr::SdfPath targetPath);
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

    // class-scope using:
    using SdfPathToSdfPathMap = pxr::TfHashMap<pxr::SdfPath, pxr::SdfPath, pxr::SdfPath::Hash>;
    using SdfPathToDeformableMeshVisualizerMap =
        pxr::TfHashMap<pxr::SdfPath, DeformableMeshVisualizer*, pxr::SdfPath::Hash>;
    using SdfPathToTargetInfoMap = pxr::TfHashMap<pxr::SdfPath, AttachmentTargetInfo*, pxr::SdfPath::Hash>;
    using SdfPathToSdfPathSetMap = pxr::TfHashMap<pxr::SdfPath, pxr::SdfPathSet, pxr::SdfPath::Hash>;

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
    SdfPathToSdfPathSetMap mTargetToActualAttachments; // path map from targets in mAttachments to their attachments
    SdfPathToTargetInfoMap mTargetToInfo; // path map from targets of valid attachments to their info

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

    pxr::SdfPathSet mBufferTargetPathsToUpdateTransform;
    pxr::SdfPathSet mBufferTargetPathsToUpdateGeometry;

    bool mBufferModeDirtyDeformable;
    bool mBufferModeDirtyAttachments;
};

} // namespace ui
} // namespace physx
} // namespace omni
