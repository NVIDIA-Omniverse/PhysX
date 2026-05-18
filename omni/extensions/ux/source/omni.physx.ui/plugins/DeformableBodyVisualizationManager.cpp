// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "DeformableTetMeshVisualizer.h"
#include "DeformableTriMeshVisualizer.h"

#include <private/omni/physx/IPhysxUsdLoad.h>
#include <omni/kit/EditorUsd.h>
#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <omni/timeline/ITimeline.h>
#include <omni/physx/IPhysxSettings.h>

#include <omni/fabric/IFabric.h>
#include <usdrt/scenegraph/usd/usd/stage.h>
#include <omni/usd/Selection.h>

#include "DeformableBodyVisualizationManager.h"

using namespace omni::physx::ui;
using namespace pxr;
using namespace omni::physx::usdparser;

extern UsdStageRefPtr gStage;
extern omni::timeline::TimelinePtr gTimeline;
extern carb::settings::ISettings* gSettings;
extern uint8_t gProxySelectionGroup;

static constexpr char kViewportGizmoScalePath[] = PERSISTENT_SETTINGS_PREFIX "/app/viewport/gizmo/scale";
static constexpr char kViewportGizmoConstantScaleEnabledPath[] = PERSISTENT_SETTINGS_PREFIX "/app/viewport/gizmo/constantScaleEnabled";
static constexpr char kViewportGizmoConstantScalePath[] = PERSISTENT_SETTINGS_PREFIX "/app/viewport/gizmo/constantScale";

static const TfToken transformMatrixAttrName{ "xformOp:transform" };
static const TfToken visualizationGapAttributeName{ "visualizationGap" };
static const TfToken scaleAttrName{ "xformOp:scale" };

static const float kEdgeToPointScale = 0.1f;

static const GfVec3f kColorSample[2] = { GfVec3f(0.6f, 0.0f, 0.6f), GfVec3f(0.3f, 0.0f, 0.3f) }; // violet/dark violet

static const GfVec3f kColorCollision(0.0f, 0.3f, 0.05f);  // green
static const GfVec3f kColorSimulation(0.0f, 0.05f, 0.3f); // blue
static const GfVec3f kColorRestShape(1.0f, 0.5f, 0.0f);   // bright orange
static const GfVec3f kColorBindPose(0.6f, 0.2f, 0.8f);    // purple
static const GfVec3f kColorFilter = GfVec3f(0.0f, 0.7f, 0.5f); // cyan

static const float kRadiusSample[2] = { 1.2f, 0.8f };
static const float kRadiusFilter[2] = { 0.6f, 0.6f };

static const double kPrototypeRadiusZero = 0.0;

namespace
{
    SdfPath appendPath(SdfPath basePath, const std::string& name)
    {
        return basePath.IsEmpty() ? SdfPath() : basePath.AppendElementString(name);
    }

    SdfPath getProxySimMeshPath(SdfPath proxyRootPath)
    {
        return appendPath(proxyRootPath, std::string("_sim_mesh"));
    }

    SdfPath getProxyCollMeshPath(SdfPath proxyRootPath)
    {
        return appendPath(proxyRootPath, std::string("_coll_mesh"));
    }

    SdfPath getProxyRestShapePath(SdfPath proxyRootPath)
    {
        return appendPath(proxyRootPath, std::string("_rest_shape"));
    }

    void readVisualizationScale(float& visualizationScale)
    {
        visualizationScale = 1.0f;

        if (gSettings)
        {
            if (gSettings->getAsBool(kViewportGizmoConstantScaleEnabledPath))
            {
                //kViewportGizmoConstantScalePath: default = 10.0f
                visualizationScale = 0.1f * gSettings->getAsFloat(kViewportGizmoConstantScalePath);
            }
            else
            {
                //kViewportGizmoScalePath: default = 1.0f
                //meterPerUnit: default = 0.01f
                visualizationScale = 0.01f * gSettings->getAsFloat(kViewportGizmoScalePath) / float(UsdGeomGetStageMetersPerUnit(gStage));
            }
        }
    }

    bool isEnabledCollision(const UsdPrim prim)
    {
        UsdPhysicsCollisionAPI collisionAPI(prim);
        if (collisionAPI)
        {
            bool isEnabled;
            collisionAPI.GetCollisionEnabledAttr().Get(&isEnabled);
            return isEnabled;
        }
        return false;
    }

    bool findDeformableMeshPaths(ProxyInfoType::Enum& type, SdfPath& simMeshPath, SdfPath& collMeshPath, SdfPathSet& skinGeomPaths,
                                 const SdfPath actualDeformableBodyPath)
    {
        simMeshPath = SdfPath();
        collMeshPath = SdfPath();
        skinGeomPaths.clear();
        type = ProxyInfoType::eNone;

        TfType volumeSimType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->VolumeDeformableSimAPI);
        TfType surfaceSimType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->SurfaceDeformableSimAPI);
        if (!gStage)
        {
            return false;
        }

        UsdPrim bodyPrim = gStage->GetPrimAtPath(actualDeformableBodyPath);
        UsdPrimRange prims(bodyPrim, UsdPrimAllPrimsPredicate);
        for (UsdPrimRange::const_iterator it = prims.begin(); it != prims.end(); ++it)
        {
            UsdPrim prim = *it;
            if (!prim.IsA<UsdGeomPointBased>())
            {
                continue;
            }

            if (prim != bodyPrim)
            {
                bool resetsXformStack = false;
                UsdGeomXformable(prim).GetOrderedXformOps(&resetsXformStack);
                if (resetsXformStack)
                {
                    it.PruneChildren();
                    continue;
                }
            }

            bool isVolumeSim = prim.HasAPI(volumeSimType);
            bool isSurfaceSim = prim.HasAPI(surfaceSimType);
            if (isVolumeSim)
                type = ProxyInfoType::eVolumeDeformableMesh;
            else if (isSurfaceSim)
                type = ProxyInfoType::eSurfaceDeformableMesh;

            bool isSim = isVolumeSim || isSurfaceSim;
            bool isColl = isEnabledCollision(prim);

            if (isSim)
            {
                if (!simMeshPath.IsEmpty())
                {
                    return false;
                }
                if (prim != bodyPrim && prim.GetParent() != bodyPrim)
                {
                    return false;
                }
                simMeshPath = prim.GetPath();
            }
            if (isColl)
            {
                if (!collMeshPath.IsEmpty())
                {
                    return false;
                }
                collMeshPath = prim.GetPath();
            }
            if (!isSim && !isColl)
            {
                skinGeomPaths.insert(prim.GetPath());
            }
        }

        return true;
    }

    void updateImageablePurpose(SdfPath path, bool active)
    {
        if (!gStage)
            return;

        UsdGeomImageable img = UsdGeomImageable::Get(gStage, path);
        if (img)
        {
            if (active)
            {
                img.GetPurposeAttr().Set(UsdGeomTokens->proxy);
            }
            else
            {
                img.GetPurposeAttr().Clear();
            }
        }
    }

    UsdAttribute getPosePurposesAttr(UsdPrim posePrim, TfToken instanceName)
    {
        TfToken attrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
            OmniPhysicsDeformableAttrTokens->multipleApplyTemplate_purposes, instanceName);
        return posePrim.GetAttribute(attrName);
    }

    TfToken getPoseNameFromPurpose(const UsdPrim prim, const TfToken posePurposeToken)
    {
        TfTokenVector allAPIs = prim.GetAppliedSchemas();

        TfType poseType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformablePoseAPI);
        TfToken poseTypeName = UsdSchemaRegistry::GetAPISchemaTypeName(poseType);

        for (const auto& api : allAPIs)
        {
            std::pair<TfToken, TfToken> typeNameAndInstance = UsdSchemaRegistry::GetTypeNameAndInstance(api);
            if (typeNameAndInstance.first == poseTypeName)
            {
                VtArray<TfToken> candTokens;
                getPosePurposesAttr(prim, typeNameAndInstance.second).Get(&candTokens);
                for (const TfToken candToken : candTokens)
                {
                    if (candToken == posePurposeToken)
                    {
                        return typeNameAndInstance.second;
                    }
                }
            }
        }

        return TfToken();
    }

    void getGeomPoints(UsdPrim prim, VtArray<GfVec3f>* points)
    {
        UsdGeomPointBased pointBased(prim);
        if (pointBased)
        {
            pointBased.GetPointsAttr().Get(points);
        }
    }

    void getRestShapePoints(UsdPrim prim, VtArray<GfVec3f>* points)
    {
        UsdAttribute attr = prim.GetAttribute(OmniPhysicsDeformableAttrTokens->restShapePoints);
        if (attr)
        {
            attr.Get(points);
        }
    }

    void getPosePoints(UsdPrim prim, VtArray<GfVec3f>* points, TfToken purpose)
    {
        if (prim)
        {
            TfType poseType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformablePoseAPI);
            TfToken bindPoseName = getPoseNameFromPurpose(prim, purpose);
            if (!bindPoseName.IsEmpty() && prim.HasAPI(poseType, bindPoseName))
            {
                TfToken attrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
                    OmniPhysicsDeformableAttrTokens->multipleApplyTemplate_points, bindPoseName);
                prim.GetAttribute(attrName).Get(points);
            }
        }
    }

    void getBindPosePoints(UsdPrim prim, VtArray<GfVec3f>* points)
    {
        getPosePoints(prim, points, OmniPhysicsDeformableAttrTokens->bindPose);
    }

    void getVtxAttachmentPoints(VtArray<GfVec3f>& attachmentPoints,
                                const UsdAttribute& vtxIndicesAttr,
                                const VtArray<GfVec3f>& simMeshPoints)
    {
        VtArray<int32_t> vtxIndices;
        if (vtxIndicesAttr)
            vtxIndicesAttr.Get(&vtxIndices);

        attachmentPoints.clear();
        attachmentPoints.reserve(vtxIndices.size());
        for (size_t i = 0; i < vtxIndices.size(); ++i)
        {
            const int32_t vtxIndex = vtxIndices[i];
            if (vtxIndex < simMeshPoints.size())
            {
                attachmentPoints.push_back(simMeshPoints[vtxIndex]);
            }
        }
    }

    void getTetAttachmentPoints(VtArray<GfVec3f>& attachmentPoints,
        const UsdAttribute& tetIndicesAttr,
        const UsdAttribute& tetCoordsAttr,
        const VtArray<GfVec4i>& simMeshTetVtxIndices,
        const VtArray<GfVec3f>& simMeshPoints)
    {
        VtArray<int32_t> tetIndices;
        if (tetIndicesAttr)
            tetIndicesAttr.Get(&tetIndices);

        VtArray<GfVec3f> tetCoords;
        if (tetCoordsAttr)
            tetCoordsAttr.Get(&tetCoords);

        attachmentPoints.clear();

        if (tetIndices.size() != tetCoords.size())
            return;

        attachmentPoints.reserve(tetIndices.size());
        for (size_t i = 0; i < tetIndices.size(); ++i)
        {
            const int32_t tetIndex = tetIndices[i];
            if (tetIndex < simMeshTetVtxIndices.size())
            {
                const GfVec4i& vtxIndices = simMeshTetVtxIndices[tetIndex];
                if (vtxIndices[0] < simMeshPoints.size() &&
                    vtxIndices[1] < simMeshPoints.size() &&
                    vtxIndices[2] < simMeshPoints.size() &&
                    vtxIndices[3] < simMeshPoints.size())
                {
                    const GfVec3f& p0 = simMeshPoints[vtxIndices[0]];
                    const GfVec3f& p1 = simMeshPoints[vtxIndices[1]];
                    const GfVec3f& p2 = simMeshPoints[vtxIndices[2]];
                    const GfVec3f& p3 = simMeshPoints[vtxIndices[3]];

                    const GfVec3f& tetCoord = tetCoords[i];
                    GfVec3f point = p0 * tetCoord[0] + p1 * tetCoord[1] + p2 * tetCoord[2] +
                                    p3 * (1.0 - tetCoord[0] - tetCoord[1] - tetCoord[2]);
                    attachmentPoints.push_back(point);
                }
            }
        }
    }

    SdfPath getProxyTargetPath(const SdfPath proxyAttachmentPath, int slot)
    {
        std::string nameString("target_");
        return proxyAttachmentPath.AppendElementString(nameString + std::to_string(slot));
    }

    SdfPath getSessionPointInstancerPrototypePath(const SdfPath sessionPointInstancerPath)
    {
        return sessionPointInstancerPath.AppendElementString(std::string("prototype"));
    }

    SdfPath getSessionSamplePointInstancerPath(const SdfPath proxyAttachmentPath, int slot)
    {
        SdfPath targetPath = getProxyTargetPath(proxyAttachmentPath, slot);
        return targetPath.AppendElementString("samplePoints");
    }

    SdfPath getSessionFilterPointInstancerPath(const SdfPath proxyAttachmentPath, int slot)
    {
        SdfPath targetPath = getProxyTargetPath(proxyAttachmentPath, slot);
        return targetPath.AppendElementString("filterPoints");
    }

    using TargetPathPair = std::array<SdfPath, 2>;

    TargetPathPair getTargets(const TfType type, const SdfPath path)
    {
        TargetPathPair dstTargets;
        SdfPathVector srcTargets;
        SdfPath targetPath;

        const UsdPrim& prim = gStage->GetPrimAtPath(path);
        if (prim)
        {
            UsdRelationship rel;
            if (prim.IsA(type))
            {
                rel = prim.GetRelationship(OmniPhysicsDeformableAttrTokens->src0);
                if (rel)
                    rel.GetTargets(&srcTargets);
                targetPath = srcTargets.size() == 1 ? srcTargets[0] : SdfPath();
                dstTargets[0] = targetPath;

                rel = prim.GetRelationship(OmniPhysicsDeformableAttrTokens->src1);
                if (rel)
                    rel.GetTargets(&srcTargets);
                targetPath = srcTargets.size() == 1 ? srcTargets[0] : SdfPath();
                dstTargets[1] = targetPath;
            }
        }

        return dstTargets;
    }

    TargetPathPair getAttachmentTargets(const SdfPath attachmentPath)
    {
        TfType attachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->Attachment);

        return getTargets(attachmentType, attachmentPath);
    }

    TargetPathPair getCollisionFilterTargets(const SdfPath collisionFilterPath)
    {
        TfType collisionFilterType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->ElementCollisionFilter);

        return getTargets(collisionFilterType, collisionFilterPath);
    }

    SdfPath getProxyAttachmentPathFromRoot(SdfPath proxyRootPath)
    {
        return appendPath(proxyRootPath, std::string("_attachment"));
    }

    SdfPath getProxyCollisionFilterPathFromRoot(SdfPath proxyRootPath)
    {
        return appendPath(proxyRootPath, std::string("_collision_filter"));
    }

    void updatePrototypeScale(const SdfPath& pointInstancerPath, const GfMatrix4d& targetToWorld)
    {
        SdfPath pointPrototypePath = pointInstancerPath.AppendElementString(std::string("prototype"));
        UsdGeomSphere pointPrototype = UsdGeomSphere::Get(gStage, pointPrototypePath);

        GfVec3d scale = GfTransform(targetToWorld).GetScale();
        for (int i = 0; i < 3; ++i)
        {
            if (std::abs(scale[i]) <= 1e-6)
                // sanitize in case of near zero:
                scale[i] = std::signbit(scale[i]) ? -1e6 : 1e6;
            else
                scale[i] = 1.0 / scale[i];
        }
        // set new scale
        const UsdAttribute scaleAttr = pointPrototype.GetPrim().GetAttribute(scaleAttrName);
        if (!scaleAttr.IsValid())
            return;
        scaleAttr.Set(GfVec3f(scale));
    }

    void fillinDummyPoint(UsdGeomPointInstancer& pointInstancer)
    {
        VtArray<GfVec3f> points(1, GfVec3f(0.0f));
        VtArray<int> prototypeIndices(1, 0);
        pointInstancer.GetPositionsAttr().Set(points);
        pointInstancer.GetProtoIndicesAttr().Set(prototypeIndices);
    }

#define ON_ATTRIBUTE_CHANGE_WRAPPER(name)\
    void name(ProxyVisualizationClient& client, SdfPath path, TfToken attributeName)\
    {   static_cast<DeformableBodyVisualizationManager&>(client).name(path, attributeName);    }

    ON_ATTRIBUTE_CHANGE_WRAPPER(onAttributeChange_DeformableBodyPoints)
    ON_ATTRIBUTE_CHANGE_WRAPPER(onAttributeChange_DeformableBodyTopology)
    ON_ATTRIBUTE_CHANGE_WRAPPER(onAttributeChange_DeformableAttachmentTargetPoints)
    ON_ATTRIBUTE_CHANGE_WRAPPER(onAttributeChange_DeformableAttachmentTopology)
    ON_ATTRIBUTE_CHANGE_WRAPPER(onAttributeChange_DeformableCollisionFilterTopology)

}

DeformableBodyVisualizationManager::DeformableBodyVisualizationManager(ProxyVisualizationManager& proxyVisualizationManager,
    const VisualizerMode mode, const DeformableVisualizerMeshType meshType, bool displayDeformableAttachments)
    : mProxyManager(proxyVisualizationManager),
      mMode(mode),
      mMeshType(meshType),
      mVisualizationGap(-1.0f),
      mVisualizationScale(-1.0f),
      mBufferModeDirtyDeformable(false),
      mDisplayDeformableAttachments(displayDeformableAttachments)
{
    mProxyManager.registerAttribute(UsdGeomTokens.Get()->points, *this, &::onAttributeChange_DeformableBodyPoints);
    mProxyManager.registerAttribute(UsdGeomTokens->tetVertexIndices, *this, &::onAttributeChange_DeformableBodyTopology);
    mProxyManager.registerAttribute(UsdGeomTokens->faceVertexCounts, *this, &::onAttributeChange_DeformableBodyTopology);
    mProxyManager.registerAttribute(UsdGeomTokens->faceVertexIndices, *this, &::onAttributeChange_DeformableBodyTopology);
    mProxyManager.registerAttribute(OmniPhysicsDeformableAttrTokens->restTetVtxIndices, *this, &::onAttributeChange_DeformableBodyTopology);  
    mProxyManager.registerAttribute(OmniPhysicsDeformableAttrTokens->restTriVtxIndices, *this, &::onAttributeChange_DeformableBodyTopology);  
    mProxyManager.registerAttribute(UsdGeomTokens.Get()->points, *this, &::onAttributeChange_DeformableAttachmentTargetPoints);
    mProxyManager.registerAttribute(OmniPhysicsDeformableAttrTokens->src0, *this, &::onAttributeChange_DeformableAttachmentTopology);
    mProxyManager.registerAttribute(OmniPhysicsDeformableAttrTokens->src1, *this, &::onAttributeChange_DeformableAttachmentTopology);
    mProxyManager.registerAttribute(OmniPhysicsDeformableAttrTokens->vtxIndicesSrc0, *this, &::onAttributeChange_DeformableAttachmentTopology);
    mProxyManager.registerAttribute(OmniPhysicsDeformableAttrTokens->localPositionsSrc1, *this, &::onAttributeChange_DeformableAttachmentTopology);
    mProxyManager.registerAttribute(OmniPhysicsDeformableAttrTokens->tetIndicesSrc1, *this, &::onAttributeChange_DeformableAttachmentTopology);
    mProxyManager.registerAttribute(OmniPhysicsDeformableAttrTokens->tetCoordsSrc1, *this, &::onAttributeChange_DeformableAttachmentTopology);
    mProxyManager.registerAttribute(OmniPhysicsDeformableAttrTokens->src0, *this, &::onAttributeChange_DeformableCollisionFilterTopology);
    mProxyManager.registerAttribute(OmniPhysicsDeformableAttrTokens->src1, *this, &::onAttributeChange_DeformableCollisionFilterTopology);
    mProxyManager.registerAttribute(OmniPhysicsDeformableAttrTokens->groupElemCounts0, *this, &::onAttributeChange_DeformableCollisionFilterTopology);
    mProxyManager.registerAttribute(OmniPhysicsDeformableAttrTokens->groupElemCounts1, *this, &::onAttributeChange_DeformableCollisionFilterTopology);
    mProxyManager.registerAttribute(OmniPhysicsDeformableAttrTokens->groupElemIndices0, *this, &::onAttributeChange_DeformableCollisionFilterTopology);
    mProxyManager.registerAttribute(OmniPhysicsDeformableAttrTokens->groupElemIndices1, *this, &::onAttributeChange_DeformableCollisionFilterTopology);
}

void DeformableBodyVisualizationManager::onAttributeChange_DeformableBodyPoints(SdfPath path, TfToken attributeName)
{
    auto it = mActualObjectToDeformableBodyMap.find(path);
    if (it == mActualObjectToDeformableBodyMap.end())
        return;

    if (mActualDeformables.count(it->second) > 0)
    {
        mBufferDeformablePathsToUpdatePoints.insert(it->second);
    }
}

void DeformableBodyVisualizationManager::onAttributeChange_DeformableBodyTopology(SdfPath path, TfToken attributeName)
{
    auto it = mActualObjectToDeformableBodyMap.find(path);
    if (it == mActualObjectToDeformableBodyMap.end())
        return;

    if (mActualDeformables.count(it->second) > 0)
    {
        mBufferDeformablePathsToUpdateTopology.insert(it->second);
    }
}

void DeformableBodyVisualizationManager::onAttributeChange_DeformableAttachmentTargetPoints(SdfPath targetPath, TfToken attributeName)
{
    if (mTargetToAttachmentInfo.count(targetPath) > 0 && mProxyManager.getProxyInfo(targetPath))
    {
        mBufferTargetPathsToUpdateGeometry.insert(targetPath);
    }
}

void DeformableBodyVisualizationManager::onAttributeChange_DeformableAttachmentTopology(SdfPath attachmentPath, TfToken attributeName)
{
    if (mActualAttachments.count(attachmentPath) > 0)
    {
        mBufferAttachmentPathsToUpdate.insert(attachmentPath);
    }
}

void DeformableBodyVisualizationManager::onAttributeChange_DeformableCollisionFilterTopology(SdfPath collisionFilterPath, TfToken attributeName)
{
    if (mActualCollisionFilters.count(collisionFilterPath) > 0)
    {
        mBufferCollisionFilterPathsToUpdate.insert(collisionFilterPath);
    }
}

DeformableBodyVisualizationManager::~DeformableBodyVisualizationManager()
{
    release();
}

void DeformableBodyVisualizationManager::setMode(VisualizerMode mode)
{
    if (mMode != mode)
    {
        VisualizerMode oldMode = mMode;
        mMode = mode; // mode needs to be set before handlePrimResync is executed!
        if (oldMode == VisualizerMode::eNone)
        {
            if (gStage)
            {
                // reparse the stage if we are activating the visualizer
                mProxyManager.handlePrimResync(gStage->GetPseudoRoot().GetPath());
                mProxyManager.selectionChanged();
            }
        }
        else if (mode == VisualizerMode::eNone)
        {
            clearDeformableBodies(true);
            clearAttachments();
            clearCollisionFilters();
        }
        mBufferModeDirtyDeformable = true;
        mBufferModeDirtyAttachments = true;
        mBufferModeDirtyCollisionFilters = true;
    }
}

void DeformableBodyVisualizationManager::setMeshType(const DeformableVisualizerMeshType meshType)
{
    mMeshType = meshType;
    mBufferModeDirtyDeformable = true;
}

void DeformableBodyVisualizationManager::displayDeformableAttachments(bool enable)
{
    mDisplayDeformableAttachments = enable;
    mBufferModeDirtyAttachments = true;
    mBufferModeDirtyCollisionFilters = true;
}

void DeformableBodyVisualizationManager::release()
{
    clearDeformableBodies(false);
    clearAttachments();
    clearCollisionFilters();
}

bool DeformableBodyVisualizationManager::needsStageParse(UsdStageWeakPtr stage)
{
    if (!isActive())
        return false;

    PXR_NS::UsdStageCache& cache = PXR_NS::UsdUtilsStageCache::Get();
    omni::fabric::UsdStageId stageId = { static_cast<uint64_t>(cache.GetId(stage).ToLongInt()) };
    omni::fabric::IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(stageId);
    usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(stageId, stageInProgress);

    {
        const std::vector<usdrt::SdfPath> paths = usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("OmniPhysicsDeformableBodyAPI"));
        if (!paths.empty())
            return true;
    }

    {
        const std::vector<usdrt::SdfPath> paths = usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("OmniPhysicsVtxXformAttachment"));
        if (!paths.empty())
            return true;
    }

    {
        const std::vector<usdrt::SdfPath> paths = usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("OmniPhysicsVtxTetAttachment"));
        if (!paths.empty())
            return true;
    }

    {
        const std::vector<usdrt::SdfPath> paths = usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("OmniPhysicsElementCollisionFilter"));
        if (!paths.empty())
            return true;
    }

    return false;
}

void DeformableBodyVisualizationManager::handlePrimResync(SdfPath path)
{
    if (!isActive() || !gStage)
        return;

    TfType bodyType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);

    // check for deformable updates
    if (mActualDeformables.count(path))
    {
        mBufferDeformablePathsToRemove.erase(path);
        mBufferDeformablePathsToUpdate.insert(path);
    }
    else
    {
        const UsdPrim& prim = gStage->GetPrimAtPath(path);
        if (prim && prim.HasAPI(bodyType))
        {
            mBufferDeformablePathsToAdd.insert(path);
        }
    }

    // check for attachment updates
    if (mActualAttachments.count(path))
    {
        mBufferAttachmentPathsToRemove.erase(path);
        mBufferAttachmentPathsToUpdate.insert(path);
    }
    else
    {
        TfType vtxXformAttachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxXformAttachment);
        TfType vtxTetAttachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxTetAttachment);
        const UsdPrim& prim = gStage->GetPrimAtPath(path);
        if (prim && (prim.IsA(vtxXformAttachmentType) || prim.IsA(vtxTetAttachmentType)))
        {
            mBufferAttachmentPathsToAdd.insert(path);
        }
        else if (prim && (prim.IsA<UsdGeomXformable>() || prim.HasAPI(bodyType)))
        {
            //stale targets might have become valid again
            SdfPathToSdfPathSetMap::const_iterator tit = mTargetToActualAttachments.find(path);
            if (tit != mTargetToActualAttachments.cend())
            {
                const SdfPathSet& targetAttachments = tit->second;
                for (SdfPath targetAttachment : targetAttachments)
                {
                    mBufferAttachmentPathsToRemove.erase(targetAttachment);
                    mBufferAttachmentPathsToUpdate.insert(targetAttachment);
                }
                // could be a tranform related attribute removal
                mBufferTargetPathsToUpdateTransform.insert(path);
            }
        }
        else if (mTargetToAttachmentInfo.count(path))
        {
            //target api got removed
            const SdfPathSet& targetAttachments = mTargetToActualAttachments[path];
            for (SdfPath targetAttachment : targetAttachments)
            {
                if (getAttachmentProxyInfo(targetAttachment))
                {
                    mBufferAttachmentPathsToRemove.erase(targetAttachment);
                    mBufferAttachmentPathsToUpdate.insert(targetAttachment);
                }
            }
        }
    }

    // check for collision filter updates
    if (mActualCollisionFilters.count(path))
    {
        mBufferCollisionFilterPathsToRemove.erase(path);
        mBufferCollisionFilterPathsToUpdate.insert(path);
    }
    else
    {
        TfType elementCollisionFilterType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->ElementCollisionFilter);
        const UsdPrim prim = gStage->GetPrimAtPath(path);
        if (prim && prim.IsA(elementCollisionFilterType))
        {
            mBufferCollisionFilterPathsToAdd.insert(path);
        }
        else if (prim && (prim.IsA<UsdGeomXformable>() || prim.HasAPI(bodyType)))
        {          
            //stale targets might have become valid again
            SdfPathToSdfPathSetMap::const_iterator tit = mTargetToActualCollisionFilters.find(path);
            if (tit != mTargetToActualCollisionFilters.cend())
            {
                const SdfPathSet& targetCollisionFilters = tit->second;
                for (SdfPath targetCollisionFilter : targetCollisionFilters)
                {
                    mBufferCollisionFilterPathsToRemove.erase(targetCollisionFilter);
                    mBufferCollisionFilterPathsToUpdate.insert(targetCollisionFilter);
                }
                // could be a tranform related attribute removal
                mBufferCollisionFilterTargetPathsToUpdateTransform.insert(path);
            }
        }
        else if (mTargetToCollisionFilterInfo.count(path))
        {
            //target api got removed
            const SdfPathSet& targetCollisionFilters = mTargetToActualCollisionFilters[path];
            for (SdfPath targetCollisionFilter : targetCollisionFilters)
            {
                if (getCollisionFilterProxyInfo(targetCollisionFilter))
                {
                    mBufferCollisionFilterPathsToRemove.erase(targetCollisionFilter);
                    mBufferCollisionFilterPathsToUpdate.insert(targetCollisionFilter);
                }
            }
        }
    }
}

void DeformableBodyVisualizationManager::handlePrimRemove(SdfPath path)
{
    mBufferDeformablePathsToAdd.erase(path);
    if (mActualDeformables.count(path))
    {
        mBufferDeformablePathsToRemove.insert(path);
        mBufferDeformablePathsToUpdate.erase(path);
    }

    // notification for sim/coll meshes that were removed.
    {
        auto it = mActualObjectToDeformableBodyMap.find(path);
        if (it != mActualObjectToDeformableBodyMap.end())
        {
            SdfPath actualDeformableBodyPath = it->second;
            mProxyManager.bufferUpdateActive(path);
        }
    }

    mBufferAttachmentPathsToAdd.erase(path);
    if (mActualAttachments.count(path))
    {
        mBufferAttachmentPathsToRemove.insert(path);
        mBufferAttachmentPathsToUpdate.erase(path);
        mBufferAttachmentPathsToUpdateGeometry.erase(path);
    }

    // notification for attachment targets that were removed.
    {
        auto it = mTargetToActualAttachments.find(path);
        if (it != mTargetToActualAttachments.end())
        {
            SdfPathSet& actualAttachments = it->second;
            for (SdfPath actualAttachment : actualAttachments)
            {
                mProxyManager.bufferUpdateActive(actualAttachment);
            }
        }
    }

    mBufferCollisionFilterPathsToAdd.erase(path);
    if (mActualCollisionFilters.count(path))
    {
        mBufferCollisionFilterPathsToRemove.insert(path);
        mBufferCollisionFilterPathsToUpdate.erase(path);
    }

    // notification for collision filter targets that were removed.
    {
        auto it = mTargetToActualCollisionFilters.find(path);
        if (it != mTargetToActualCollisionFilters.end())
        {
            SdfPathSet& actualCollisionFilters = it->second;
            for (SdfPath actualCollisionFilter : actualCollisionFilters)
            {
                mProxyManager.bufferUpdateActive(actualCollisionFilter);
            }
        }
    }
}

void DeformableBodyVisualizationManager::handleTransformChange(SdfPath path)
{
    // xform must always be handled because the change may have happened in the xform hierarchy above the deformable:
    // use awesome path table to quickly find all candidate deformable paths to update:
    if (mActualDeformables.count(path))
        mBufferDeformablePathsToUpdateTransform.insert(path);

    SdfPathToSdfPathSetMap::const_iterator tit = mTargetToActualAttachments.find(path);
    if (tit != mTargetToActualAttachments.cend())
    {
        mBufferTargetPathsToUpdateTransform.insert(path);
        const SdfPathSet& targetAttachments = tit->second;
        for (SdfPath attachmentPath : targetAttachments)
        {
            mBufferAttachmentPathsToUpdateVizAttribute.insert(attachmentPath);
        }
    }

    tit = mTargetToActualCollisionFilters.find(path);
    if (tit != mTargetToActualCollisionFilters.cend())
    {
        mBufferCollisionFilterTargetPathsToUpdateTransform.insert(path);
    }
}

void DeformableBodyVisualizationManager::handleVisibilityChange(SdfPath path)
{
    auto it = mActualObjectToDeformableBodyMap.find(path);
    if (it != mActualObjectToDeformableBodyMap.end() && mActualDeformables.count(it->second))
    {
        mProxyManager.bufferUpdateActive(path);
    }

    if (mActualAttachments.count(path))
    {
        mProxyManager.bufferUpdateActive(path);
    }
}

void DeformableBodyVisualizationManager::updateTracking()
{
    if (!isActive())
    {
        return;
    }

    // track new deformable paths
    for (SdfPath path : mBufferDeformablePathsToAdd)
    {
        addDeformableBody(path);
    }

    for (SdfPath path : mBufferDeformablePathsToUpdate)
    {
        updateDeformableBody(path);
    }

    // delete removed deformables:
    for (SdfPath path : mBufferDeformablePathsToRemove)
    {
        removeDeformableBody(path, true);
    }

    // track new attachment paths
    for (SdfPath path : mBufferAttachmentPathsToAdd)
    {
        addAttachment(path);
    }

    for (SdfPath path : mBufferAttachmentPathsToUpdate)
    {
        updateAttachment(path);
    }

    // delete removed attachments:
    for (SdfPath path : mBufferAttachmentPathsToRemove)
    {
        removeAttachment(path);
    }

    // track new collision filter paths
    for (SdfPath path : mBufferCollisionFilterPathsToAdd)
    {
        addCollisionFilter(path);
    }

    for (SdfPath path : mBufferCollisionFilterPathsToUpdate)
    {
        updateCollisionFilter(path);
    }

    // delete removed collision filters:
    for (SdfPath path : mBufferCollisionFilterPathsToRemove)
    {
        removeCollisionFilter(path);
    }

    //always update transforms for kinematics -
    //now let's just update for all xformables, since viz code should be agnostic to
    //colliders or rigid bodies
    for (auto targetIt : mTargetToAttachmentInfo)
    {
        AttachmentTargetInfo* targetInfo = targetIt.second;
        if (targetInfo && targetInfo->type == AttachmentTargetInfo::Type::eXformable)
        {
            mBufferTargetPathsToUpdateTransform.insert(targetIt.first);
        }
    }

    for (auto targetIt : mTargetToCollisionFilterInfo)
    {
        CollisionFilterTargetInfo* targetInfo = targetIt.second;
        if (targetInfo && targetInfo->type == CollisionFilterTargetInfo::Type::eXformable)
        {
            mBufferCollisionFilterTargetPathsToUpdateTransform.insert(targetIt.first);
        }
    }
}

void DeformableBodyVisualizationManager::updateModeDirty()
{
    if (mBufferModeDirtyDeformable)
    {
        for (const auto& entry : mActualObjectToDeformableBodyMap)
        {
            const SdfPath actualPath = entry.first;
            mProxyManager.bufferUpdateActive(actualPath);
        }

        for (const SdfPath& actualPath : mActualAttachments)
        {
            mProxyManager.bufferUpdateActive(actualPath);
        }
    }

    if (mBufferModeDirtyAttachments)
    {
        for (const SdfPath& actualPath : mActualAttachments)
        {
            mProxyManager.bufferUpdateActive(actualPath);
        }
    }

    if (mBufferModeDirtyCollisionFilters)
    {
        for (const SdfPath& actualPath : mActualCollisionFilters)
        {
            mProxyManager.bufferUpdateActive(actualPath);
        }
    }
}

bool DeformableBodyVisualizationManager::checkMode(SdfPath actualPath)
{
    if (mMode == VisualizerMode::eSelected)
    {
        return mProxyManager.isSelected(actualPath);
    }

    //checkMode shouldn't be called if VisualizerMode is eNone, but handle this case for robustness
    return mMode != VisualizerMode::eNone;
}

bool IsTriMeshValid(const UsdPrim& prim)
{
    // Check whether all faces of the input tri mesh have 3 vertices
    UsdGeomMesh mesh(prim);
    VtIntArray faceVertexCounts;
    mesh.GetFaceVertexCountsAttr().Get(&faceVertexCounts);

    if (faceVertexCounts.empty())
        return false;

    for (const int count : faceVertexCounts) {
        if (count != 3) {
            return false;
        }
    }
    return true;
}

bool DeformableBodyVisualizationManager::checkCompleteness(ProxyInfo& proxyInfo)
{
    bool isAttachment = (proxyInfo.type == ProxyInfoType::eVtxXformAttachment || proxyInfo.type == ProxyInfoType::eVtxTetAttachment);
    bool isCollisionFilter = (proxyInfo.type == ProxyInfoType::eElementCollisionFilter);
    if (isAttachment)
    {
        return checkAttachmentsCompleteness(proxyInfo);
    }
    else if (isCollisionFilter)
    {
        return checkCollisionFiltersCompleteness(proxyInfo);
    }
    else
    {
        return checkDeformableCompleteness(proxyInfo);
    }
}

uint32_t DeformableBodyVisualizationManager::updateActiveProxies(ProxyInfo& proxyInfo, SdfPath actualPath)
{
    uint32_t numActiveProxies = 0;

    if (proxyInfo.type == ProxyInfoType::eVolumeDeformableMesh || proxyInfo.type == ProxyInfoType::eSurfaceDeformableMesh)
    {
        auto it = mActualObjectToDeformableBodyMap.find(actualPath);
        if (it == mActualObjectToDeformableBodyMap.end())
            return numActiveProxies;

        const SdfPath actualDeformableBodyPath = it->second;
        const SdfPath simMeshPath = getSimMeshPathFromMap(actualDeformableBodyPath);
        if (simMeshPath.IsEmpty())
            return numActiveProxies;

        const SdfPath collMeshPath = getCollMeshPathFromMap(actualDeformableBodyPath);

        if (actualPath == simMeshPath)
        {
            DeformableMeshProxyInfo& simMeshProxyInfo = static_cast<DeformableMeshProxyInfo&>(proxyInfo);
            
            // Simulation mesh
            bool showSimMesh = showProxySimulationMesh();
            if (showSimMesh)
            {
                simMeshProxyInfo.proxyRootPath = mProxyManager.createProxyRootPrim(simMeshPath);
                const SdfPath proxySimMeshPath = getProxySimMeshPath(simMeshProxyInfo.proxyRootPath);
                UsdPrim proxySimMeshPrim = gStage->GetPrimAtPath(proxySimMeshPath);
                if (!proxySimMeshPrim)
                {
                    proxySimMeshPrim = createDeformableBodyProxyPrim(proxySimMeshPath, proxyInfo.type);
                    mProxyManager.addProxyPrim(simMeshPath, proxySimMeshPath);
                }

                mBufferDeformablePathsToUpdateTransform.insert(actualDeformableBodyPath);
                mBufferDeformablePathsToUpdatePoints.insert(actualDeformableBodyPath);
                mBufferDeformablePathsToUpdateTopology.insert(actualDeformableBodyPath);
                numActiveProxies++;
            }
            else
            {
                const SdfPath proxySimMeshPath = getProxySimMeshPath(simMeshProxyInfo.proxyRootPath);
                if (!proxySimMeshPath.IsEmpty())
                {
                    mProxyManager.removeProxyPrim(simMeshPath, proxySimMeshPath);
                }
            }

            // Rest shape
            bool showRestShape = showProxyRestShape();
            if (showRestShape)
            {
                simMeshProxyInfo.proxyRootPath = mProxyManager.createProxyRootPrim(simMeshPath);
                SdfPath proxyRestShapePath = getProxyRestShapePath(simMeshProxyInfo.proxyRootPath);
                UsdPrim proxyRestShapePrim = gStage->GetPrimAtPath(proxyRestShapePath);
                if (!proxyRestShapePrim)
                {
                    proxyRestShapePrim = createDeformableBodyProxyPrim(proxyRestShapePath, proxyInfo.type);
                    mProxyManager.addProxyPrim(simMeshPath, proxyRestShapePath);
                }

                mBufferDeformablePathsToUpdateTransform.insert(actualDeformableBodyPath);
                mBufferDeformablePathsToUpdatePoints.insert(actualDeformableBodyPath);
                mBufferDeformablePathsToUpdateTopology.insert(actualDeformableBodyPath);
                numActiveProxies++;
            }
            else
            {
                SdfPath proxyRestShapePath = getProxyRestShapePath(simMeshProxyInfo.proxyRootPath);
                if (!proxyRestShapePath.IsEmpty())
                {
                    mProxyManager.removeProxyPrim(simMeshPath, proxyRestShapePath);
                }
            }

            // Collision mesh
            if (simMeshPath == collMeshPath)
            {
                // Only need to handle single node case here
                bool showCollMesh = showProxyCollisionMesh();
                if (showCollMesh)
                {
                    simMeshProxyInfo.proxyRootPath = mProxyManager.createProxyRootPrim(simMeshPath);
                    SdfPath proxyCollMeshPath = getProxyCollMeshPath(simMeshProxyInfo.proxyRootPath);
                    UsdPrim proxyCollMeshPrim = gStage->GetPrimAtPath(proxyCollMeshPath);
                    if (!proxyCollMeshPrim)
                    {
                        proxyCollMeshPrim = createDeformableBodyProxyPrim(proxyCollMeshPath, proxyInfo.type);
                        mProxyManager.addProxyPrim(collMeshPath, proxyCollMeshPath);
                    }

                    mBufferDeformablePathsToUpdateTransform.insert(actualDeformableBodyPath);
                    mBufferDeformablePathsToUpdatePoints.insert(actualDeformableBodyPath);
                    mBufferDeformablePathsToUpdateTopology.insert(actualDeformableBodyPath);
                    numActiveProxies++;
                }
                else
                {
                    SdfPath proxyCollMeshPath = getProxyCollMeshPath(simMeshProxyInfo.proxyRootPath);
                    if (!proxyCollMeshPath.IsEmpty())
                    {
                        mProxyManager.removeProxyPrim(collMeshPath, proxyCollMeshPath);
                    }
                }
            }
        }
        else if (!collMeshPath.IsEmpty() && collMeshPath != simMeshPath && actualPath == collMeshPath)
        {
            DeformableMeshProxyInfo& collMeshProxyInfo = static_cast<DeformableMeshProxyInfo&>(proxyInfo);
            // Collision mesh
            bool showCollMesh = showProxyCollisionMesh();
            if (showCollMesh)
            {
                collMeshProxyInfo.proxyRootPath = mProxyManager.createProxyRootPrim(collMeshPath);
                SdfPath proxyCollMeshPath = getProxyCollMeshPath(collMeshProxyInfo.proxyRootPath);
                UsdPrim proxyCollMeshPrim = gStage->GetPrimAtPath(proxyCollMeshPath);
                if (!proxyCollMeshPrim)
                {
                    proxyCollMeshPrim = createDeformableBodyProxyPrim(proxyCollMeshPath, proxyInfo.type);
                    mProxyManager.addProxyPrim(collMeshPath, proxyCollMeshPath);
                }

                mBufferDeformablePathsToUpdateTransform.insert(actualDeformableBodyPath);
                mBufferDeformablePathsToUpdatePoints.insert(actualDeformableBodyPath);
                mBufferDeformablePathsToUpdateTopology.insert(actualDeformableBodyPath);
                numActiveProxies++;
            }
            else
            {
                SdfPath proxyCollMeshPath = getProxyCollMeshPath(collMeshProxyInfo.proxyRootPath);
                if (!proxyCollMeshPath.IsEmpty())
                {
                    mProxyManager.removeProxyPrim(collMeshPath, proxyCollMeshPath);
                }
            }
        }      
    }
    else if (proxyInfo.type == ProxyInfoType::eVtxXformAttachment || proxyInfo.type == ProxyInfoType::eVtxTetAttachment)
    {
            numActiveProxies = updateActiveProxiesAttachments(proxyInfo, actualPath, isActive() && mDisplayDeformableAttachments, mVisualizationScale);
    }
    else if (proxyInfo.type == ProxyInfoType::eElementCollisionFilter)
    {
            numActiveProxies = updateActiveProxiesCollisionFilters(proxyInfo, actualPath, isActive() && mDisplayDeformableAttachments);
    }

    return numActiveProxies;
}

void DeformableBodyVisualizationManager::updateProxyProperties(UsdGeomXformCache& xformCache)
{
    // set new tet gap
    const float visualizationGap = gSettings ? gSettings->getAsFloat(omni::physx::kSettingVisualizationGap) : 0.0f;

    // set new vis scale
    float visualizationScale;
    readVisualizationScale(visualizationScale);

    const bool visualizationDirty = (mVisualizationGap != visualizationGap) || (mVisualizationScale != visualizationScale);
    mVisualizationGap = visualizationGap;
    mVisualizationScale = visualizationScale;

    if (visualizationDirty)
    {
        for (SdfPath actualDeformableBodyPath : mActualDeformables)
        {
            mBufferDeformablePathsToUpdateGap.insert(actualDeformableBodyPath);
        }
    }

    for (SdfPath actualDeformableBodyPath : mBufferDeformablePathsToUpdateTransform)
    {
        updateTransform(actualDeformableBodyPath, xformCache);
    }

    for (SdfPath actualDeformableBodyPath : mBufferDeformablePathsToUpdatePoints)
    {
        updatePoints(actualDeformableBodyPath);
    }

    for (SdfPath actualDeformableBodyPath : mBufferDeformablePathsToUpdateTopology)
    {
        updateTopology(actualDeformableBodyPath);
    }

    for (SdfPath actualDeformableBodyPath : mBufferDeformablePathsToUpdateGap)
    {
        updateGap(actualDeformableBodyPath);
    }

    for (SdfPath targetPath : mBufferTargetPathsToUpdateTransform)
    {
        updateAttachmentTargetTransform(targetPath, xformCache);
    }

    for (SdfPath targetPath : mBufferTargetPathsToUpdateGeometry)
    {
        updateAttachmentTargetGeometry(targetPath);
    }

    for (SdfPath targetPath : mBufferCollisionFilterTargetPathsToUpdateTransform)
    {
        updateCollisionFilterTargetTransform(targetPath, xformCache);
    }

    for (SdfPath attachmentPath : mBufferAttachmentPathsToUpdateGeometry)
    {
        updateAttachmentGeometry(attachmentPath, mVisualizationScale);
    }

    for (SdfPath attachmentPath : mBufferAttachmentPathsToUpdateVizAttribute)
    {
        updateVisualizationScale(attachmentPath, mVisualizationScale);
    }
}

void DeformableBodyVisualizationManager::updateActualPurpose(SdfPath actualPath, bool active)
{
    auto it = mActualObjectToDeformableBodyMap.find(actualPath);
    if (it == mActualObjectToDeformableBodyMap.end())
        return;

    const SdfPath actualDeformableBodyPath = it->second;
    const SdfPath simMeshPath = getSimMeshPathFromMap(actualDeformableBodyPath);
    if (simMeshPath.IsEmpty())
        return;

    DeformableMeshProxyInfo* deformableMeshProxyInfo = getDeformableMeshProxyInfo(simMeshPath);
    if (!deformableMeshProxyInfo)
        return;

    const SdfPath collMeshPath = getCollMeshPathFromMap(actualDeformableBodyPath);

    if (actualPath == simMeshPath)
    {
        // Simulation mesh
        updateImageablePurpose(simMeshPath, active);

        // Collision mesh in single node case
        if (collMeshPath == simMeshPath)
            updateImageablePurpose(collMeshPath, active);
    }
    else if (actualPath == collMeshPath && collMeshPath != simMeshPath)
    {
        // Collision mesh in Xform/Scope case
        updateImageablePurpose(collMeshPath, active);
    }

    // Hide skin meshes
    if (gStage && active)
    {
        const SdfPathSet skinMeshPaths = getSkinGeomPathsFromMap(actualDeformableBodyPath);
        for (const SdfPath path : skinMeshPaths)
        {
            updateImageablePurpose(path, true);
        }
    }

    if (mMode == VisualizerMode::eNone)
    {
        // Restore visibilities of skin meshes
        if (gStage)
        {
            for (SdfPath actualDeformableBodyPath : mActualDeformables)
            {
                const SdfPathSet skinMeshPaths = getSkinGeomPathsFromMap(actualDeformableBodyPath);
                for (const SdfPath path : skinMeshPaths)
                {
                    updateImageablePurpose(path, false);
                }
            }
        }
    }
    else if (mMode == VisualizerMode::eSelected)
    {
        const auto usdContext = omni::usd::UsdContext::getContext();
        const std::vector<SdfPath> selectedPaths = usdContext->getSelection()->getSelectedPrimPathsV2();

        // Map selectedPaths to selected actual deformable body path
        SdfPathSet selectedDeformableBodyPathSet;
        for (const SdfPath path : selectedPaths)
        {
            if (mActualDeformables.count(path) > 0)
            {
                selectedDeformableBodyPathSet.insert(path);
            }
            else if (mActualObjectToDeformableBodyMap.count(path) > 0)
            {
                selectedDeformableBodyPathSet.insert(mActualObjectToDeformableBodyMap[path]);
            }
        }

        for (const SdfPath deformableBodyPath : mActualDeformables)
        {
            const SdfPathSet skinMeshPaths = getSkinGeomPathsFromMap(deformableBodyPath);
            for (const SdfPath path : skinMeshPaths)
            {
                if (selectedDeformableBodyPathSet.find(deformableBodyPath) != selectedDeformableBodyPathSet.end())
                {
                    // if deformableBodyPath is selected, we hide its skin meshes
                    updateImageablePurpose(path, true);
                }
                else
                {
                    // if deformableBodyPath is not selected, we visualize its skin meshes
                    updateImageablePurpose(path, false);
                }
            }
        }
    }
}

void DeformableBodyVisualizationManager::notifyReleaseProxyPrim(SdfPath proxyPrimPath)
{
    releaseDeformableMeshVisualizer(proxyPrimPath);
}

void DeformableBodyVisualizationManager::clearBuffers()
{
    mBufferDeformablePathsToAdd.clear();
    mBufferDeformablePathsToUpdate.clear();
    mBufferDeformablePathsToRemove.clear();
    mBufferDeformablePathsToUpdateTransform.clear();
    mBufferDeformablePathsToUpdatePoints.clear();
    mBufferDeformablePathsToUpdateTopology.clear();
    mBufferDeformablePathsToUpdateGap.clear();

    mBufferModeDirtyDeformable = false;

    clearAttachmentsBuffers();
    clearCollisionFiltersBuffers();
}

UsdPrim DeformableBodyVisualizationManager::createDeformableBodyProxyPrim(SdfPath proxyPath, ProxyInfoType::Enum type)
{
    UsdGeomMesh proxyMesh = UsdGeomMesh::Define(gStage, proxyPath);
    CARB_ASSERT(proxyMesh);
    omni::kit::EditorUsd::setNoDelete(proxyMesh.GetPrim(), true);
    omni::kit::EditorUsd::setHideInStageWindow(proxyMesh.GetPrim(), true);
    omni::kit::EditorUsd::setNoSelectionOutline(proxyMesh.GetPrim(), true);

    DeformableMeshVisualizer* visualizer = nullptr;
    if (type == ProxyInfoType::eVolumeDeformableMesh)
    {
        visualizer = new DeformableTetMeshVisualizer(proxyPath);      
    }
    else if (type == ProxyInfoType::eSurfaceDeformableMesh)
    {
        visualizer = new DeformableTriMeshVisualizer(proxyPath);
    }
    mProxyToDeformableMeshVisualizerMap.insert({ proxyPath, visualizer });

    // same workaround for colors and other attributes
    proxyMesh.GetPrim().CreateAttribute(visualizationGapAttributeName, SdfValueTypeNames->Float, true).Set(mVisualizationGap);

    return proxyMesh.GetPrim();
}

void DeformableBodyVisualizationManager::releaseDeformableMeshVisualizer(SdfPath proxyPath)
{
    auto it = mProxyToDeformableMeshVisualizerMap.find(proxyPath);
    if (it != mProxyToDeformableMeshVisualizerMap.end())
    {
        DeformableMeshVisualizer* visualizer = it->second;
        if (visualizer)
            delete visualizer;
    }

    mProxyToDeformableMeshVisualizerMap.erase(proxyPath);
}

DeformableBodyVisualizationManager::DeformableMeshProxyInfo* DeformableBodyVisualizationManager::createDeformableMeshProxyInfo(
    const SdfPath actualMeshPath, const SdfPath actualDeformableBodyPath, const ProxyInfoType::Enum& type)
{
    DeformableMeshProxyInfo* proxyInfo = new DeformableMeshProxyInfo();
    proxyInfo->client = this;
    proxyInfo->type = type;
    proxyInfo->actualMeshPath = actualMeshPath;
    proxyInfo->actualDeformableBodyPath = actualDeformableBodyPath;
    return proxyInfo;
}

DeformableBodyVisualizationManager::DeformableMeshProxyInfo* DeformableBodyVisualizationManager::getDeformableMeshProxyInfo(const SdfPath actualPath)
{
    ProxyInfo* proxyInfo = mProxyManager.getProxyInfo(actualPath);
    return (proxyInfo && (proxyInfo->type == ProxyInfoType::eVolumeDeformableMesh || proxyInfo->type == ProxyInfoType::eSurfaceDeformableMesh)) ? static_cast<DeformableMeshProxyInfo*>(proxyInfo) : nullptr;
}

void DeformableBodyVisualizationManager::addDeformableBody(const SdfPath actualDeformableBodyPath)
{
    if (mActualDeformables.count(actualDeformableBodyPath) > 0)
        return;

    // update tracking
    mActualDeformables.insert(actualDeformableBodyPath);

    SdfPath simMeshPath;
    SdfPath collMeshPath;
    SdfPathSet skinGeomPaths;
    ProxyInfoType::Enum type;
    findDeformableMeshPaths(type, simMeshPath, collMeshPath, skinGeomPaths, actualDeformableBodyPath);

    mDeformableBodyToActualSimMeshMap[actualDeformableBodyPath] = simMeshPath;
    mDeformableBodyToActualCollMeshMap[actualDeformableBodyPath] = collMeshPath;
    mDeformableBodyToActualSkinGeomsMap[actualDeformableBodyPath] = skinGeomPaths;

    if (simMeshPath.IsEmpty() || collMeshPath.IsEmpty())
        return;

    DeformableMeshProxyInfo* simMeshProxyInfo = createDeformableMeshProxyInfo(simMeshPath, actualDeformableBodyPath, type);
    if (simMeshProxyInfo)
    {
        mProxyManager.addProxy(simMeshPath, *simMeshProxyInfo);

        // update the map
        mActualObjectToDeformableBodyMap.insert({ simMeshPath, actualDeformableBodyPath });

        // update events
        mProxyManager.bufferUpdateActive(simMeshPath);
    }

    if (collMeshPath != simMeshPath)
    {
        // Create another DeformableMeshProxyInfo when collision mesh is different from simulation mesh
        DeformableMeshProxyInfo* collMeshProxyInfo = createDeformableMeshProxyInfo(collMeshPath, actualDeformableBodyPath, type);
        if (collMeshProxyInfo)
        {
            mProxyManager.addProxy(collMeshPath, *collMeshProxyInfo);

            // update the map
            mActualObjectToDeformableBodyMap.insert({ collMeshPath, actualDeformableBodyPath });

            // update events
            mProxyManager.bufferUpdateActive(collMeshPath);
        }
    }
}

void DeformableBodyVisualizationManager::updateDeformableBody(const SdfPath actualDeformableBodyPath)
{
    TfType bodyType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
    const UsdPrim prim = gStage->GetPrimAtPath(actualDeformableBodyPath);
    if (!prim || !prim.HasAPI(bodyType))
    {
        removeDeformableBody(actualDeformableBodyPath, true);
    }
    else
    {
        removeDeformableBody(actualDeformableBodyPath, false);
        addDeformableBody(actualDeformableBodyPath);

        // could be a tranform related attribute removal
        mBufferDeformablePathsToUpdateTransform.insert(actualDeformableBodyPath);
    }
}

void DeformableBodyVisualizationManager::removeDeformableBody(const SdfPath actualDeformableBodyPath, bool updateActual)
{ 
    const SdfPath simMeshPath = getSimMeshPathFromMap(actualDeformableBodyPath);
    DeformableMeshProxyInfo* simMeshProxyInfo = getDeformableMeshProxyInfo(simMeshPath);

    if (!simMeshPath.IsEmpty())
    {
        // update events
        mProxyManager.bufferUpdateActive(simMeshPath);

        if (updateActual)
        {
            updateActualPurpose(simMeshPath, false);
        }

        if (simMeshProxyInfo)
        {
            if (!simMeshProxyInfo->proxyRootPath.IsEmpty())
            {
                SdfPath proxySimMeshPath = getProxySimMeshPath(simMeshProxyInfo->proxyRootPath);
                // see if there are any viz helpers for this deformable:
                if (!proxySimMeshPath.IsEmpty())
                {
                    DeformableMeshVisualizer* visualizer = getDeformableMeshVisualizer(proxySimMeshPath);
                    if (visualizer)
                        delete visualizer;

                    mProxyToDeformableMeshVisualizerMap.erase(proxySimMeshPath);
                }

                SdfPath proxyRestShapePath = getProxyRestShapePath(simMeshProxyInfo->proxyRootPath);
                if (!proxyRestShapePath.IsEmpty())
                {
                    DeformableMeshVisualizer* visualizer = getDeformableMeshVisualizer(proxyRestShapePath);
                    if (visualizer)
                        delete visualizer;

                    mProxyToDeformableMeshVisualizerMap.erase(proxyRestShapePath);
                }
            }

            mProxyManager.removeProxy(simMeshPath, simMeshProxyInfo);
        }
    }

    const SdfPath collMeshPath = getCollMeshPathFromMap(actualDeformableBodyPath);
    if (collMeshPath == simMeshPath && simMeshProxyInfo && simMeshProxyInfo->proxyRootPath.IsEmpty())
    {
        // Single node case
        SdfPath proxyCollMeshPath = getProxyCollMeshPath(simMeshProxyInfo->proxyRootPath);
        // see if there are any viz helpers for this deformable:
        if (!proxyCollMeshPath.IsEmpty())
        {
            DeformableMeshVisualizer* visualizer = getDeformableMeshVisualizer(proxyCollMeshPath);
            if (visualizer)
                delete visualizer;

            mProxyToDeformableMeshVisualizerMap.erase(proxyCollMeshPath);
        }
    }
    else if (!collMeshPath.IsEmpty() && collMeshPath != simMeshPath)
    {
        // update events  
        mProxyManager.bufferUpdateActive(collMeshPath);

        if (updateActual)
        {
            updateActualPurpose(collMeshPath, false);
        }

        // update tracking
        mActualObjectToDeformableBodyMap.erase(collMeshPath);

        DeformableMeshProxyInfo* collMeshProxyInfo = getDeformableMeshProxyInfo(collMeshPath);
        mProxyManager.removeProxy(collMeshPath, collMeshProxyInfo);
    }

    // update tracking
    mActualDeformables.erase(actualDeformableBodyPath);
    mActualObjectToDeformableBodyMap.erase(simMeshPath);
    mDeformableBodyToActualSimMeshMap.erase(actualDeformableBodyPath);
    mDeformableBodyToActualCollMeshMap.erase(actualDeformableBodyPath);
}

void updateProxyTransform(const SdfPath actualPath, const SdfPath proxyRootPath, UsdGeomXformCache& xformCache)
{
    // source
    const UsdPrim actualObjectPrim = gStage->GetPrimAtPath(actualPath);
    CARB_ASSERT(actualObjectPrim);
    if (!actualObjectPrim)
        return;  // no need to update if there is no source

    // target proxy simulation mesh
    UsdPrim rootProxySimMeshPrim = gStage->GetPrimAtPath(proxyRootPath);
    CARB_ASSERT(rootProxySimMeshPrim);
    if (!rootProxySimMeshPrim)
        return;

    // get transformation with cache:
    const GfMatrix4d transform = xformCache.GetLocalToWorldTransform(actualObjectPrim);

    UsdAttribute localToWorldAttr = rootProxySimMeshPrim.GetAttribute(transformMatrixAttrName);
    CARB_ASSERT(localToWorldAttr);
    localToWorldAttr.Set(transform);
}

void DeformableBodyVisualizationManager::updateTransform(const SdfPath actualDeformableBodyPath, UsdGeomXformCache& xformCache)
{
    if (!isActive())
        return;

    const SdfPath simMeshPath = getSimMeshPathFromMap(actualDeformableBodyPath);
    if (simMeshPath.IsEmpty())
        return;

    DeformableMeshProxyInfo* deformableMeshProxyInfo = getDeformableMeshProxyInfo(simMeshPath);
    if (!deformableMeshProxyInfo)
        return;

    if (!deformableMeshProxyInfo->proxyRootPath.IsEmpty())
        updateProxyTransform(simMeshPath, deformableMeshProxyInfo->proxyRootPath, xformCache);

    // target proxy collision mesh
    const SdfPath collMeshPath = getCollMeshPathFromMap(actualDeformableBodyPath);
    DeformableMeshProxyInfo* collMeshProxyInfo = getDeformableMeshProxyInfo(collMeshPath);
    if (!collMeshPath.IsEmpty() && collMeshPath != simMeshPath && collMeshProxyInfo)
    {
        if (!collMeshProxyInfo->proxyRootPath.IsEmpty())
            updateProxyTransform(collMeshPath, collMeshProxyInfo->proxyRootPath, xformCache);
    }
}

void DeformableBodyVisualizationManager::updatePoints(const SdfPath actualDeformableBodyPath)
{
    if (!isActive())
        return;

    const SdfPath simMeshPath = getSimMeshPathFromMap(actualDeformableBodyPath);
    if (simMeshPath.IsEmpty())
        return;

    DeformableMeshProxyInfo* simMeshProxyInfo = getDeformableMeshProxyInfo(simMeshPath);
    if (!simMeshProxyInfo)
        return;

    TfType volumeSimType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->VolumeDeformableSimAPI);
    TfType surfaceSimType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->SurfaceDeformableSimAPI);

    VtArray<GfVec3f> points;
    UsdAttribute pointsAttr;
    // get attributes:
    if (showProxySimulationMesh())
    {
        const UsdGeomPointBased simMesh = UsdGeomPointBased::Get(gStage, simMeshPath);
        UsdPrim simMeshPrim = gStage->GetPrimAtPath(simMeshPath);
        if (simMesh && simMeshPrim && !simMeshProxyInfo->proxyRootPath.IsEmpty())
        {
            if (showDefaultPose())
            {
                getGeomPoints(simMeshPrim, &points);
            }
            else if (showBindPose())
            {
                getBindPosePoints(simMeshPrim, &points);
            }

            SdfPath proxySimMeshPath = getProxySimMeshPath(simMeshProxyInfo->proxyRootPath);
            DeformableMeshVisualizer* simMeshVisualizer = getDeformableMeshVisualizer(proxySimMeshPath);
            if (simMeshVisualizer)
            {
                if ((simMeshProxyInfo->type == ProxyInfoType::eVolumeDeformableMesh && points.size() >= 4) ||
                    (simMeshProxyInfo->type == ProxyInfoType::eSurfaceDeformableMesh && points.size() >= 3))
                {
                    simMeshVisualizer->setPoints(points);
                    simMeshVisualizer->updatePoints();
                }
            }
        }
    }
    else if (showProxyCollisionMesh())
    {
        const SdfPath collMeshPath = getCollMeshPathFromMap(actualDeformableBodyPath);
        const UsdGeomPointBased collMesh = UsdGeomPointBased::Get(gStage, collMeshPath);
        UsdPrim collMeshPrim = gStage->GetPrimAtPath(collMeshPath);
        if (collMesh && collMeshPrim)
        {
            if (showDefaultPose())
            {
                getGeomPoints(collMeshPrim, &points);
            }
            else if (showBindPose())
            {
                getBindPosePoints(collMeshPrim, &points);
            }

            SdfPath rootPath = simMeshProxyInfo->proxyRootPath;
            DeformableMeshProxyInfo* collMeshProxyInfo = getDeformableMeshProxyInfo(collMeshPath);
            if (collMeshProxyInfo && collMeshPath != simMeshPath)
                rootPath = collMeshProxyInfo->proxyRootPath;

            if (!rootPath.IsEmpty())
            {
                SdfPath proxyCollMeshPath = getProxyCollMeshPath(rootPath);
                DeformableMeshVisualizer* collMeshVisualizer = getDeformableMeshVisualizer(proxyCollMeshPath);
                if (collMeshVisualizer)
                {
                    if ((collMeshProxyInfo->type == ProxyInfoType::eVolumeDeformableMesh && points.size() >= 4) ||
                        (collMeshProxyInfo->type == ProxyInfoType::eSurfaceDeformableMesh && points.size() >= 3))
                    {
                        collMeshVisualizer->setPoints(points);
                        collMeshVisualizer->updatePoints();
                    }
                }
            }
        }
    }
    else if (showProxyRestShape())
    {
        UsdPrim simMeshPrim = gStage->GetPrimAtPath(simMeshPath);

        if (simMeshProxyInfo->type == ProxyInfoType::eVolumeDeformableMesh)
        {
            if (simMeshPrim && simMeshPrim.HasAPI(volumeSimType))
            {
                getRestShapePoints(simMeshPrim, &points);

                if (!simMeshProxyInfo->proxyRootPath.IsEmpty())
                {
                    SdfPath proxyRestShapePath = getProxyRestShapePath(simMeshProxyInfo->proxyRootPath);
                    DeformableMeshVisualizer* restShapeVisualizer = getDeformableMeshVisualizer(proxyRestShapePath);
                    if (points.size() >= 4 && restShapeVisualizer)
                    {
                        restShapeVisualizer->setPoints(points);
                        restShapeVisualizer->updatePoints();
                    }
                }
            }
        }
        else if (simMeshProxyInfo->type == ProxyInfoType::eSurfaceDeformableMesh)
        {
            if (simMeshPrim && simMeshPrim.HasAPI(surfaceSimType))
            {
                getRestShapePoints(simMeshPrim, &points);

                if (!simMeshProxyInfo->proxyRootPath.IsEmpty())
                {
                    SdfPath proxyRestShapePath = getProxyRestShapePath(simMeshProxyInfo->proxyRootPath);
                    DeformableMeshVisualizer* restShapeVisualizer = getDeformableMeshVisualizer(proxyRestShapePath);
                    if (points.size() >= 3 && restShapeVisualizer)
                    {
                        restShapeVisualizer->setPoints(points);
                        restShapeVisualizer->updatePoints();
                    }
                }
            }
        }
    }
}

void DeformableBodyVisualizationManager::updateTopology(const SdfPath actualDeformableBodyPath)
{
    if (!isActive())
        return;

    const SdfPath simMeshPath = getSimMeshPathFromMap(actualDeformableBodyPath);
    if (simMeshPath.IsEmpty())
        return;

    DeformableMeshProxyInfo* simMeshProxyInfo = getDeformableMeshProxyInfo(simMeshPath);
    if (!simMeshProxyInfo)
        return;

    const UsdPrim deformablePrim = gStage->GetPrimAtPath(actualDeformableBodyPath);
    if (!deformablePrim)
        return;

    // get attributes:
    UsdAttribute indsAttr;
    VtArray<GfVec3f> points;
    VtArray<GfVec3i> allIndices_3i;
    VtArray<GfVec4i> allIndices_4i;
    VtArray<int> allIndices_int;

    GfVec3f color;
    DeformableMeshVisualizer* visualizer = nullptr;
    VtArray<GfVec3i> filteredSurfaceFaceVertexIndices;
    std::unordered_set<uint32_t> allFilterTriIds;
    VtArray<GfVec3i> surfaceFaceVertexIndices;
    TargetPathPair targetPaths;

    if (simMeshProxyInfo->type == ProxyInfoType::eVolumeDeformableMesh)
    {
        if (showProxySimulationMesh())
        {           
            UsdPrim simMeshPrim = gStage->GetPrimAtPath(simMeshPath);
            if (simMeshPrim && simMeshPrim.IsA<UsdGeomTetMesh>())
            {
                indsAttr = UsdGeomTetMesh(simMeshPrim).GetTetVertexIndicesAttr();
                indsAttr.Get(&allIndices_4i);

                if (showDefaultPose())
                {
                    getGeomPoints(simMeshPrim, &points);
                }
                else if (showBindPose())
                {
                    getBindPosePoints(simMeshPrim, &points);
                }
            }

            if (!simMeshProxyInfo->proxyRootPath.IsEmpty())
            {
                SdfPath proxySimMeshPath = getProxySimMeshPath(simMeshProxyInfo->proxyRootPath);
                visualizer = getDeformableMeshVisualizer(proxySimMeshPath);
                color = kColorSimulation;
            }
        }
        else if (showProxyCollisionMesh())
        {
            const SdfPath collMeshPath = getCollMeshPathFromMap(actualDeformableBodyPath);
            UsdPrim collMeshPrim = gStage->GetPrimAtPath(collMeshPath);
            if (collMeshPrim && collMeshPrim.IsA<UsdGeomTetMesh>())
            {
                indsAttr = UsdGeomTetMesh(collMeshPrim).GetTetVertexIndicesAttr();
                if (indsAttr)
                {
                    indsAttr.Get(&allIndices_4i);
                }

                UsdAttribute surfaceFaceVertexIndicesAttr = UsdGeomTetMesh(collMeshPrim).GetSurfaceFaceVertexIndicesAttr();
                if (surfaceFaceVertexIndicesAttr)
                {
                    surfaceFaceVertexIndicesAttr.Get(&surfaceFaceVertexIndices);
                }

                if (showDefaultPose())
                {
                    getGeomPoints(collMeshPrim, &points);
                }
                else if (showBindPose())
                {
                    getBindPosePoints(collMeshPrim, &points);
                }
            }

            SdfPath rootPath = simMeshProxyInfo->proxyRootPath;
            DeformableMeshProxyInfo* collMeshProxyInfo = getDeformableMeshProxyInfo(collMeshPath);
            if (collMeshProxyInfo && collMeshPath != simMeshPath)
                rootPath = collMeshProxyInfo->proxyRootPath;

            if (!rootPath.IsEmpty())
            {
                SdfPath proxyCollMeshPath = getProxyCollMeshPath(rootPath);
                visualizer = getDeformableMeshVisualizer(proxyCollMeshPath);
                color = kColorCollision;
            }

            VtArray<uint32_t> filterTriIds[2];
            allFilterTriIds.clear();
            const SdfPathSet& targetCollisionFilters = mTargetToActualCollisionFilters[collMeshPath];
            for (SdfPath targetCollisionFilter : targetCollisionFilters)
            {             
                getFilteredElementIndices(targetPaths, filterTriIds, targetCollisionFilter);

                int slot = (targetPaths[0] == collMeshPath) ? 0 : 1;
                allFilterTriIds.insert(filterTriIds[slot].begin(), filterTriIds[slot].end());
            }

            filteredSurfaceFaceVertexIndices.clear();
            for (uint32_t tri : allFilterTriIds)
            {
                if (tri < surfaceFaceVertexIndices.size())
                {
                    filteredSurfaceFaceVertexIndices.push_back(surfaceFaceVertexIndices[tri]);
                }                
            }
        }
        else if (showProxyRestShape())
        {
            TfType volumeSimType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->VolumeDeformableSimAPI);
            UsdPrim simMeshPrim = gStage->GetPrimAtPath(simMeshPath);
            if (simMeshPrim && simMeshPrim.HasAPI(volumeSimType))
            {
                indsAttr = simMeshPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->restTetVtxIndices);
                indsAttr.Get(&allIndices_4i);

                getRestShapePoints(simMeshPrim, &points);

                if (!simMeshProxyInfo->proxyRootPath.IsEmpty())
                {
                    SdfPath proxyRestShapePath = getProxyRestShapePath(simMeshProxyInfo->proxyRootPath);
                    visualizer = getDeformableMeshVisualizer(proxyRestShapePath);
                    color = kColorRestShape;
                }
            }
        }
    }
    else if (simMeshProxyInfo->type == ProxyInfoType::eSurfaceDeformableMesh)
    {
        if (showProxySimulationMesh())
        {
            const UsdGeomMesh simMesh = UsdGeomMesh::Get(gStage, simMeshPath);
            UsdPrim simMeshPrim = gStage->GetPrimAtPath(simMeshPath);
            if (simMesh && simMeshPrim)
            {
                indsAttr = simMesh.GetFaceVertexIndicesAttr();
                indsAttr.Get(&allIndices_int);

                if (showDefaultPose())
                {
                    getGeomPoints(simMeshPrim, &points);
                }
                else if (showBindPose())
                {
                    getBindPosePoints(simMeshPrim, &points);
                }
            }

            if (!simMeshProxyInfo->proxyRootPath.IsEmpty())
            {
                SdfPath proxySimMeshPath = getProxySimMeshPath(simMeshProxyInfo->proxyRootPath);
                visualizer = getDeformableMeshVisualizer(proxySimMeshPath);
                color = kColorSimulation;
            }
        }
        else if (showProxyCollisionMesh())
        {
            const SdfPath collMeshPath = getCollMeshPathFromMap(actualDeformableBodyPath);
            const UsdGeomMesh collMesh = UsdGeomMesh::Get(gStage, collMeshPath);
            UsdPrim collMeshPrim = gStage->GetPrimAtPath(collMeshPath);
            if (collMesh && collMeshPrim)
            {
                indsAttr = collMesh.GetFaceVertexIndicesAttr();
                indsAttr.Get(&allIndices_int);

                surfaceFaceVertexIndices.reserve(allIndices_int.size()/3);
                for (int i = 0; i < allIndices_int.size() / 3; ++i)
                {
                    surfaceFaceVertexIndices.push_back(GfVec3i(allIndices_int[3*i], allIndices_int[3 * i + 1], allIndices_int[3 * i + 2]));
                }

                if (showDefaultPose())
                {
                    getGeomPoints(collMeshPrim, &points);
                }
                else if (showBindPose())
                {
                    getBindPosePoints(collMeshPrim, &points);
                }
            }

            SdfPath rootPath = simMeshProxyInfo->proxyRootPath;
            DeformableMeshProxyInfo* collMeshProxyInfo = getDeformableMeshProxyInfo(collMeshPath);
            if (collMeshProxyInfo && collMeshPath != simMeshPath)
                rootPath = collMeshProxyInfo->proxyRootPath;

            if (!rootPath.IsEmpty())
            {
                SdfPath proxyCollMeshPath = getProxyCollMeshPath(rootPath);
                visualizer = getDeformableMeshVisualizer(proxyCollMeshPath);
                color = kColorCollision;
            }

            VtArray<uint32_t> filterTriIds[2];
            allFilterTriIds.clear();
            const SdfPathSet& targetCollisionFilters = mTargetToActualCollisionFilters[collMeshPath];
            for (SdfPath targetCollisionFilter : targetCollisionFilters)
            {
                getFilteredElementIndices(targetPaths, filterTriIds, targetCollisionFilter);

                int slot = (targetPaths[0] == collMeshPath) ? 0 : 1;
                allFilterTriIds.insert(filterTriIds[slot].begin(), filterTriIds[slot].end());
            }

            filteredSurfaceFaceVertexIndices.clear();
            for (uint32_t tri : allFilterTriIds)
            {
                if (tri < surfaceFaceVertexIndices.size())
                {
                    filteredSurfaceFaceVertexIndices.push_back(surfaceFaceVertexIndices[tri]);
                }
            }
        }
        else if (showProxyRestShape())
        {
            TfType surfaceSimType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->SurfaceDeformableSimAPI);
            UsdPrim simMeshPrim = gStage->GetPrimAtPath(simMeshPath);
            if (simMeshPrim && simMeshPrim.HasAPI(surfaceSimType))
            {
                indsAttr = simMeshPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->restTriVtxIndices);
                indsAttr.Get(&allIndices_3i);

                for (const GfVec3i& vec : allIndices_3i)
                {
                    for (int32_t i = 0; i < 3; ++i)
                    {
                        allIndices_int.push_back(vec[i]);
                    }
                }

                getRestShapePoints(simMeshPrim, &points);

                if (!simMeshProxyInfo->proxyRootPath.IsEmpty())
                {
                    SdfPath proxyRestShapePath = getProxyRestShapePath(simMeshProxyInfo->proxyRootPath);
                    visualizer = getDeformableMeshVisualizer(proxyRestShapePath);
                    color = kColorRestShape;
                }
            }
        }
    }

    if (!visualizer)
        return;

    bool valid = false;
    if (simMeshProxyInfo->type == ProxyInfoType::eVolumeDeformableMesh)
    {
        valid = points.size() >= 4 && allIndices_4i.size() > 0;
        int32_t numPoints = int32_t(points.size());
        for (GfVec4i index : allIndices_4i)
        {
            for (int32_t i = 0; i < 4; ++i)
            {
                if (index[i] >= numPoints)
                {
                    valid = false;
                }
            }
        }

        for (const GfVec4i& vec : allIndices_4i)
        {
            for (int32_t i = 0; i < 4; ++i)
            {
                allIndices_int.push_back(vec[i]);
            }
        }
    }
    else if (simMeshProxyInfo->type == ProxyInfoType::eSurfaceDeformableMesh)
    {
        valid = points.size() >= 3 && allIndices_int.size() >= 3;
        int32_t numPoints = int32_t(points.size());
        for (int index : allIndices_int)
        {
            if (index >= numPoints)
            {
                valid = false;
            }
        }
    }

    if (valid)
    {
        visualizer->setIndices(allIndices_int);
        visualizer->setPoints(points);
        visualizer->updateTopology();

        VtArray<GfVec3f> colors(allIndices_int.size(), color);

        if (mDisplayDeformableAttachments)
        {
            VtArray<uint32_t> filterVertIndices;
            DeformableTetMeshVisualizer* tetMeshVisualizer = dynamic_cast<DeformableTetMeshVisualizer*>(visualizer);
            DeformableTriMeshVisualizer* triMeshVisualizer = dynamic_cast<DeformableTriMeshVisualizer*>(visualizer);
            if (tetMeshVisualizer)
            {
                // Volume deformable
                std::unordered_set<uint32_t> filteredTetIds = tetMeshVisualizer->getFilteredTetIds(filteredSurfaceFaceVertexIndices);

                for (uint32_t tetId : filteredTetIds)
                {
                    for (uint32_t i = 0; i < 4; ++i)
                    {
                        uint32_t index = tetId * 4 + i;
                        if (index < colors.size())
                            colors[index] = kColorFilter;
                    }
                }
            }
            else if (triMeshVisualizer)
            {
                // Surface deformable
                std::unordered_set<uint32_t> filteredTriIds = triMeshVisualizer->getFilteredTriIds(filteredSurfaceFaceVertexIndices);

                for (uint32_t triId : filteredTriIds)
                {
                    for (uint32_t i = 0; i < 3; ++i)
                    {
                        uint32_t index = triId * 3 + i;
                        if (index < colors.size())
                            colors[index] = kColorFilter;
                    }
                }
            }
        }

        visualizer->setColors(colors);
    }
    else
    {
        visualizer->setIndices(VtArray<int32_t>());      
        visualizer->setPoints(VtArray<GfVec3f>());
        visualizer->updateTopology();
        visualizer->setColors(VtArray<GfVec3f>());
    }
}

void DeformableBodyVisualizationManager::updateGap(const SdfPath actualDeformableBodyPath)
{
    if (!isActive())
        return;

    const SdfPath simMeshPath = getSimMeshPathFromMap(actualDeformableBodyPath);
    if (simMeshPath.IsEmpty())
        return;

    DeformableMeshProxyInfo* deformableMeshProxyInfo = getDeformableMeshProxyInfo(simMeshPath);
    if (!deformableMeshProxyInfo)
        return;

    if (showProxySimulationMesh() && !deformableMeshProxyInfo->proxyRootPath.IsEmpty())
    {
        SdfPath proxySimMeshPath = getProxySimMeshPath(deformableMeshProxyInfo->proxyRootPath);
        UsdPrim proxySimMeshPrim = gStage->GetPrimAtPath(proxySimMeshPath);
        if (proxySimMeshPrim)
        {
            UsdAttribute tetGapAttr = proxySimMeshPrim.GetAttribute(visualizationGapAttributeName);
            DeformableMeshVisualizer* simMeshVisualizer = getDeformableMeshVisualizer(proxySimMeshPath);
            if (tetGapAttr && simMeshVisualizer)
            {
                tetGapAttr.Set(mVisualizationGap);
                if (mProxyToDeformableMeshVisualizerMap.find(proxySimMeshPath) == mProxyToDeformableMeshVisualizerMap.end())
                    return;

                simMeshVisualizer->setGapValue(mVisualizationGap);
            }
        }
    }
    else if (showProxyCollisionMesh())
    {
        SdfPath rootPath = deformableMeshProxyInfo->proxyRootPath;
        const SdfPath collMeshPath = getCollMeshPathFromMap(actualDeformableBodyPath);
        DeformableMeshProxyInfo* collMeshProxyInfo = getDeformableMeshProxyInfo(collMeshPath);
        if (collMeshProxyInfo && collMeshPath != simMeshPath)
            rootPath = collMeshProxyInfo->proxyRootPath;

        if (!rootPath.IsEmpty())
        {
            SdfPath proxyCollMeshPath = getProxyCollMeshPath(rootPath);
            UsdPrim proxyCollMeshPrim = gStage->GetPrimAtPath(proxyCollMeshPath);
            if (proxyCollMeshPrim)
            {
                UsdAttribute tetGapAttr = proxyCollMeshPrim.GetAttribute(visualizationGapAttributeName);
                DeformableMeshVisualizer* collMeshVisualizer = getDeformableMeshVisualizer(proxyCollMeshPath);
                if (tetGapAttr && collMeshVisualizer)
                {
                    tetGapAttr.Set(mVisualizationGap);
                    if (mProxyToDeformableMeshVisualizerMap.find(proxyCollMeshPath) == mProxyToDeformableMeshVisualizerMap.end())
                        return;

                    collMeshVisualizer->setGapValue(mVisualizationGap);
                }
            }
        }
    }
    else if (showProxyRestShape() && !deformableMeshProxyInfo->proxyRootPath.IsEmpty())
    {
        SdfPath proxyRestShapePath = getProxyRestShapePath(deformableMeshProxyInfo->proxyRootPath);
        UsdPrim proxyRestShapePrim = gStage->GetPrimAtPath(proxyRestShapePath);
        if (proxyRestShapePrim)
        {
            UsdAttribute tetGapAttr = proxyRestShapePrim.GetAttribute(visualizationGapAttributeName);
            DeformableMeshVisualizer* restShapeVisualizer = getDeformableMeshVisualizer(proxyRestShapePath);
            if (tetGapAttr && restShapeVisualizer)
            {
                tetGapAttr.Set(mVisualizationGap);
                if (mProxyToDeformableMeshVisualizerMap.find(proxyRestShapePath) == mProxyToDeformableMeshVisualizerMap.end())
                    return;

                restShapeVisualizer->setGapValue(mVisualizationGap);
            }
        }
    }
}

void DeformableBodyVisualizationManager::clearDeformableBodies(bool updateSkin)
{
    if (gStage)
    {
        omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());

        while (mActualDeformables.size())
        {
            removeDeformableBody(*mActualDeformables.begin(), updateSkin);
        }
    }

    mActualDeformables.clear();
    mActualObjectToDeformableBodyMap.clear();
    mDeformableBodyToActualSimMeshMap.clear();
    mDeformableBodyToActualCollMeshMap.clear();
    mDeformableBodyToActualSkinGeomsMap.clear();
    mProxyToDeformableMeshVisualizerMap.clear();

    clearBuffers();
}

SdfPath DeformableBodyVisualizationManager::getSimMeshPathFromMap(const SdfPath actualDeformableBodyPath)
{
    auto it = mDeformableBodyToActualSimMeshMap.find(actualDeformableBodyPath);
    if (it != mDeformableBodyToActualSimMeshMap.end())
    {
        return it->second;
    }
    return SdfPath();
}

SdfPath DeformableBodyVisualizationManager::getCollMeshPathFromMap(const SdfPath actualDeformableBodyPath)
{
    auto it = mDeformableBodyToActualCollMeshMap.find(actualDeformableBodyPath);
    if (it != mDeformableBodyToActualCollMeshMap.end())
    {
        return it->second;
    }
    return SdfPath();
}

SdfPathSet DeformableBodyVisualizationManager::getSkinGeomPathsFromMap(const SdfPath actualDeformableBodyPath)
{
    auto it = mDeformableBodyToActualSkinGeomsMap.find(actualDeformableBodyPath);
    if (it != mDeformableBodyToActualSkinGeomsMap.end())
    {
        return it->second;
    }
    return {};
}

void DeformableBodyVisualizationManager::getFilteredElementIndices(TargetPathPair& targetPaths, VtArray<uint32_t>(&filterTriIds)[2],
    const SdfPath targetCollisionFilter)
{
    if (!gStage)
        return;

    UsdPrim collisionFilterPrim = gStage->GetPrimAtPath(targetCollisionFilter);
    if (collisionFilterPrim)
    {
        targetPaths = getCollisionFilterTargets(targetCollisionFilter);

        UsdAttribute groupElemIndicesAttrs[2] = {
            collisionFilterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemIndices0),
            collisionFilterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemIndices1)
        };

        for (int slot = 0; slot < 2; ++slot)
        {
            if (groupElemIndicesAttrs[slot])
            {
                groupElemIndicesAttrs[slot].Get(&filterTriIds[slot]);
            }
        }
    }
}

DeformableMeshVisualizer* DeformableBodyVisualizationManager::getDeformableMeshVisualizer(const SdfPath proxyTetPath)
{
    DeformableMeshVisualizer* visualizer = nullptr;

    const auto iter = mProxyToDeformableMeshVisualizerMap.find(proxyTetPath);
    if (iter != mProxyToDeformableMeshVisualizerMap.end())
    {
        visualizer = iter->second;
    }

    return visualizer;
}

bool DeformableBodyVisualizationManager::checkAttachmentsCompleteness(ProxyInfo& proxyInfo)
{
    if (!gStage)
        return false;

    if (proxyInfo.type != ProxyInfoType::eVtxXformAttachment && proxyInfo.type != ProxyInfoType::eVtxTetAttachment)
        return false;
    
    AttachmentProxyInfo& attachmentProxyInfo = static_cast<AttachmentProxyInfo&>(proxyInfo);

    const UsdPrim attachmentPrim = gStage->GetPrimAtPath(attachmentProxyInfo.actualAttachmentPath);

    if (!attachmentPrim)
        return false;

    const TargetPathPair& actualTargets = attachmentProxyInfo.actualTargets;
    if (actualTargets[0].IsEmpty() || actualTargets[1].IsEmpty())
        return false;

    if (!gStage->GetPrimAtPath(actualTargets[0]) || !gStage->GetPrimAtPath(actualTargets[1]))
        return false;

    TfType vtxXformAttachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxXformAttachment);
    TfType vtxTetAttachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxTetAttachment);
    if (proxyInfo.type == ProxyInfoType::eVtxXformAttachment)
    {
        if (!attachmentPrim.IsA(vtxXformAttachmentType))
            return false;

        AttachmentTargetInfo::Type vtxType = attachmentProxyInfo.targetInfos[0]->type;
        AttachmentTargetInfo::Type xformType = attachmentProxyInfo.targetInfos[1]->type;

        if (vtxType != AttachmentTargetInfo::Type::eVolumeDeformableBody && vtxType != AttachmentTargetInfo::Type::eSurfaceDeformableBody)
            return false;

        if (xformType != AttachmentTargetInfo::Type::eXformable)
            return false;

        UsdAttribute vtxIndicesAttr = attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->vtxIndicesSrc0);
        UsdAttribute locationsAttr = attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->localPositionsSrc1);

        if (!vtxIndicesAttr || !locationsAttr)
            return false;

        VtArray<int32_t> vtxIndices;
        VtArray<GfVec3f> localPositions;
        vtxIndicesAttr.Get(&vtxIndices);
        locationsAttr.Get(&localPositions);
        if (vtxIndices.size() != localPositions.size())
            return false;
    }
    else if (proxyInfo.type == ProxyInfoType::eVtxTetAttachment)
    {
        if (!attachmentPrim.IsA(vtxTetAttachmentType))
            return false;

        AttachmentTargetInfo::Type vtxType = attachmentProxyInfo.targetInfos[0]->type;
        AttachmentTargetInfo::Type tetType = attachmentProxyInfo.targetInfos[1]->type;

        if (vtxType != AttachmentTargetInfo::Type::eVolumeDeformableBody && vtxType != AttachmentTargetInfo::Type::eSurfaceDeformableBody)
            return false;

        if (tetType != AttachmentTargetInfo::Type::eVolumeDeformableBody)
            return false;

        UsdAttribute vtxIndicesAttr = attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->vtxIndicesSrc0);
        UsdAttribute tetIndicesAttr = attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->tetIndicesSrc1);
        UsdAttribute tetCoordsAttr = attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->tetCoordsSrc1);

        if (!vtxIndicesAttr || !tetIndicesAttr || !tetCoordsAttr)
            return false;

        VtArray<int32_t> vtxIndices;
        VtArray<int32_t> tetIndices;
        VtArray<GfVec3f> tetCoords;
        vtxIndicesAttr.Get(&vtxIndices);
        tetIndicesAttr.Get(&tetIndices);
        tetCoordsAttr.Get(&tetCoords);

        if (vtxIndices.size() != tetIndices.size() || vtxIndices.size() != tetCoords.size())
            return false;
    }

    return true;
}

bool DeformableBodyVisualizationManager::checkDeformableCompleteness(ProxyInfo& proxyInfo)
{
    if (!gStage)
        return false;

    if (proxyInfo.type != ProxyInfoType::eVolumeDeformableMesh && proxyInfo.type != ProxyInfoType::eSurfaceDeformableMesh)
        return false;

    DeformableMeshProxyInfo& deformableMeshProxyInfo = static_cast<DeformableMeshProxyInfo&>(proxyInfo);

    SdfPath actualMeshPath = deformableMeshProxyInfo.actualMeshPath;
    SdfPath actualDeformableBodyPath = deformableMeshProxyInfo.actualDeformableBodyPath;

    if (actualMeshPath.IsEmpty() || actualDeformableBodyPath.IsEmpty())
        return false;

    // Check whether PhysicsDeformableBodyAPI is removed
    TfType bodyType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
    UsdPrim prim = gStage->GetPrimAtPath(actualDeformableBodyPath);
    if (!prim.HasAPI(bodyType))
        return false;

    // Check both simulation mesh and collision mesh are present for each deformable body
    const SdfPath simMeshPath = getSimMeshPathFromMap(actualDeformableBodyPath);
    const SdfPath collMeshPath = getCollMeshPathFromMap(actualDeformableBodyPath);
    if (simMeshPath.IsEmpty() || collMeshPath.IsEmpty())
        return false;

    const UsdPrim simPrim = gStage->GetPrimAtPath(simMeshPath);
    if (!simPrim.IsValid())
        return false;

    // Check whether the simulation mesh is actually a tet mesh for volume deformables
    if (proxyInfo.type == ProxyInfoType::eVolumeDeformableMesh && !simPrim.IsA<UsdGeomTetMesh>())
        return false;

    // Check whether the simulation mesh is actually a tri mesh for surface deformables
    if (proxyInfo.type == ProxyInfoType::eSurfaceDeformableMesh &&
        (!simPrim.IsA<UsdGeomMesh>() || !IsTriMeshValid(simPrim)))
        return false;

    const UsdPrim collPrim = gStage->GetPrimAtPath(collMeshPath);
    if (!collPrim.IsValid())
        return false;

    // Checks for the current mesh prim itself
    const UsdPrim meshPrim = gStage->GetPrimAtPath(actualMeshPath);

    if (showBindPose())
    {
        VtArray<GfVec3f> points;
        getGeomPoints(meshPrim, &points);

        VtArray<GfVec3f> bindPosePoints;
        getBindPosePoints(meshPrim, &bindPosePoints);
        if (points.size() != bindPosePoints.size())
            return false;
    }

    return true;
}

uint32_t DeformableBodyVisualizationManager::updateActiveProxiesAttachments(ProxyInfo& proxyInfo, SdfPath actualPath, bool isActive, const float visualizationScale)
{
    uint32_t numActiveProxies = 0;

    if (proxyInfo.type == ProxyInfoType::eVtxXformAttachment || proxyInfo.type == ProxyInfoType::eVtxTetAttachment)
    {
        AttachmentProxyInfo& attachmentProxyInfo = static_cast<AttachmentProxyInfo&>(proxyInfo);
        if (gStage && isActive)
        {
            attachmentProxyInfo.proxyRootPath = mProxyManager.createProxyRootPrim(actualPath);

            const SdfPath proxyAttachmentPath = getProxyAttachmentPathFromRoot(attachmentProxyInfo.proxyRootPath);
            UsdPrim proxyAttachmentPrim = gStage->GetPrimAtPath(proxyAttachmentPath);
            if (!proxyAttachmentPrim)
            {
                proxyAttachmentPrim = createProxyPrim(proxyAttachmentPath, proxyInfo.type);
                mProxyManager.addProxyPrim(actualPath, proxyAttachmentPath);

                for (int slot = 0; slot < 2; ++slot)
                {
                    const SdfPath proxyTargetPath = getProxyTargetPath(proxyAttachmentPath, slot);
                    UsdPrim proxyTargetPrim = gStage->GetPrimAtPath(proxyTargetPath);
                    if (proxyTargetPrim)
                    {
                        const SdfPath targetPath = attachmentProxyInfo.actualTargets[slot];
                        mProxyManager.addProxyPrim(targetPath, proxyTargetPath);
                        mBufferTargetPathsToUpdateTransform.insert(targetPath);
                    }
                }
            }

            mBufferAttachmentPathsToUpdateGeometry.insert(actualPath);
            numActiveProxies++;

            createProxyAttachmentGeometries(actualPath);
            setupProxyAttachmentGeometries(actualPath, visualizationScale);
        }
        else
        {
            const SdfPath proxyAttachmentPath = getProxyAttachmentPathFromRoot(attachmentProxyInfo.proxyRootPath);
            if (!proxyAttachmentPath.IsEmpty())
            {
                mProxyManager.removeProxyPrim(actualPath, proxyAttachmentPath);

                for (int slot = 0; slot < 2; ++slot)
                {
                    const SdfPath proxyTargetPath = getProxyTargetPath(proxyAttachmentPath, slot);
                    if (!proxyTargetPath.IsEmpty())
                    {
                        const SdfPath targetPath = attachmentProxyInfo.actualTargets[slot];
                        mProxyManager.removeProxyPrim(targetPath, proxyTargetPath);
                    }
                }
            }
        }

    }

    return numActiveProxies;
}

void DeformableBodyVisualizationManager::clearAttachmentsBuffers()
{
    mBufferAttachmentPathsToAdd.clear();
    mBufferAttachmentPathsToUpdate.clear();
    mBufferAttachmentPathsToRemove.clear();
    mBufferAttachmentPathsToUpdateGeometry.clear();
    mBufferAttachmentPathsToUpdateVizAttribute.clear();
    mBufferTargetPathsToUpdateTransform.clear();
    mBufferTargetPathsToUpdateGeometry.clear();

    mBufferModeDirtyAttachments = false;
}

void DeformableBodyVisualizationManager::updateCollisionFilterTargetTransform(const SdfPath targetPath, UsdGeomXformCache& xformCache)
{
    // get target prim
    UsdPrim targetPrim = gStage->GetPrimAtPath(targetPath);
    if (!targetPrim)
        return;  // no need to update if there is no target prim

    if (mTargetToCollisionFilterInfo.count(targetPath))
    {
        const SdfPathSet& targetCollisionFilters = mTargetToActualCollisionFilters[targetPath];

        for (SdfPath collisionFilterPath : targetCollisionFilters)
        {
            TfType collisionFilterType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->ElementCollisionFilter);
            const UsdPrim& collisionFilterPrim = gStage->GetPrimAtPath(collisionFilterPath);
            if (!collisionFilterPrim || (!collisionFilterPrim.IsA(collisionFilterType)))
                continue;

            CollisionFilterProxyInfo* collisionFilterInfo = getCollisionFilterProxyInfo(collisionFilterPath);
            CARB_ASSERT(collisionFilterInfo && collisionFilterInfo->actualTargets[0] == targetPath || collisionFilterInfo->actualTargets[1] == targetPath);
            if (collisionFilterInfo)
            {
                int32_t slot = collisionFilterInfo->actualTargets[0] == targetPath ? 0 : 1;

                const SdfPath proxyCollisionFilterPath = getProxyCollisionFilterPath(collisionFilterPath);
                if (!proxyCollisionFilterPath.IsEmpty())
                {
                    SdfPath targetXformPath = getProxyTargetPath(proxyCollisionFilterPath, slot);

                    // get transformation with cache:
                    const GfMatrix4d targetToWorld = xformCache.GetLocalToWorldTransform(targetPrim);

                    UsdGeomXform targetXform = UsdGeomXform::Get(gStage, targetXformPath);
                    UsdAttribute localToWorldAttr = targetXform.GetPrim().GetAttribute(transformMatrixAttrName);
                    CARB_ASSERT(localToWorldAttr);
                    localToWorldAttr.Set(targetToWorld);
                }
            }
        }
    }
}

void DeformableBodyVisualizationManager::clearCollisionFilters()
{
    if (gStage)
    {
        omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());

        while (mActualCollisionFilters.size())
        {
            removeCollisionFilter(*mActualCollisionFilters.begin());
        }
    }

    mActualCollisionFilters.clear();
    mTargetToActualCollisionFilters.clear();

    for (SdfPathToCollisionFilterTargetInfoMap::const_iterator it = mTargetToCollisionFilterInfo.cbegin(); it != mTargetToCollisionFilterInfo.cend(); ++it)
    {
        releaseTargetInfo(it->second);
    }
    mTargetToCollisionFilterInfo.clear();

    clearCollisionFiltersBuffers();
}

bool DeformableBodyVisualizationManager::checkCollisionFiltersCompleteness(ProxyInfo& proxyInfo)
{
    if (!gStage)
        return false;

    if (proxyInfo.type != ProxyInfoType::eElementCollisionFilter)
        return false;

    CollisionFilterProxyInfo& collisionFilterProxyInfo = static_cast<CollisionFilterProxyInfo&>(proxyInfo);

    const UsdPrim collisionFilterPrim = gStage->GetPrimAtPath(collisionFilterProxyInfo.actualCollisionFilterPath);

    if (!collisionFilterPrim)
        return false;

    const TargetPathPair& actualTargets = collisionFilterProxyInfo.actualTargets;
    if (actualTargets[0].IsEmpty() || actualTargets[1].IsEmpty())
        return false;

    if (!gStage->GetPrimAtPath(actualTargets[0]) || !gStage->GetPrimAtPath(actualTargets[1]))
        return false;

    TfType collisionFilterType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->ElementCollisionFilter);
    if (proxyInfo.type == ProxyInfoType::eElementCollisionFilter)
    {
        if (!collisionFilterPrim.IsA(collisionFilterType))
            return false;

        CollisionFilterTargetInfo::Type targetType0 = collisionFilterProxyInfo.collisionFilterTargetInfos[0]->type;
        CollisionFilterTargetInfo::Type targetType1 = collisionFilterProxyInfo.collisionFilterTargetInfos[1]->type;
        if (targetType0 == CollisionFilterTargetInfo::eNone || targetType1 == CollisionFilterTargetInfo::eNone)
            return false;

        UsdAttribute groupElemCounts0Attr = collisionFilterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemCounts0);
        UsdAttribute groupElemCounts1Attr = collisionFilterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemCounts1);
        UsdAttribute groupElemIndices0Attr = collisionFilterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemIndices0);
        UsdAttribute groupElemIndices1Attr = collisionFilterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemIndices1);
        if (!groupElemCounts0Attr || !groupElemCounts1Attr || !groupElemIndices0Attr || !groupElemIndices1Attr)
            return false;

        VtArray<uint32_t> groupElemCounts0;
        VtArray<uint32_t> groupElemCounts1;
        groupElemCounts0Attr.Get(&groupElemCounts0);
        groupElemCounts1Attr.Get(&groupElemCounts1);

        // Check if groupElemCounts0 has the same size as groupElemCounts1 or is empty
        if (!groupElemCounts0.empty() && !groupElemCounts1.empty() && (groupElemCounts0.size() != groupElemCounts1.size()))
            return false;

        VtArray<uint32_t> groupElemIndices0;
        VtArray<uint32_t> groupElemIndices1;
        groupElemIndices0Attr.Get(&groupElemIndices0);
        groupElemIndices1Attr.Get(&groupElemIndices1);

        size_t totalGroupElements0 = 0;
        for (uint32_t count : groupElemCounts0) {
            totalGroupElements0 += count;
        }

        size_t totalGroupElements1 = 0;
        for (uint32_t count : groupElemCounts1) {
            totalGroupElements1 += count;
        }

        // Check if groupElemCounts0 and groupElemIndices0 are consistent
        if (totalGroupElements0 != groupElemIndices0.size())
            return false;

        // Check if groupElemCounts1 and groupElemIndices1 are consistent
        if (totalGroupElements1 != groupElemIndices1.size())
            return false;
    }

    return true;
}

uint32_t DeformableBodyVisualizationManager::updateActiveProxiesCollisionFilters(ProxyInfo& proxyInfo, SdfPath actualPath, bool isActive)
{
    uint32_t numActiveProxies = 0;

    if (proxyInfo.type == ProxyInfoType::eElementCollisionFilter)
    {
        CollisionFilterProxyInfo& collisionFilterProxyInfo = static_cast<CollisionFilterProxyInfo&>(proxyInfo);
        if (gStage && isActive)
        {
            collisionFilterProxyInfo.proxyRootPath = mProxyManager.createProxyRootPrim(actualPath);

            const SdfPath proxyCollisionFilterPath = getProxyCollisionFilterPathFromRoot(collisionFilterProxyInfo.proxyRootPath);
            UsdPrim proxyCollisionFilterPrim = gStage->GetPrimAtPath(proxyCollisionFilterPath);
            if (!proxyCollisionFilterPrim)
            {
                proxyCollisionFilterPrim = createProxyPrim(proxyCollisionFilterPath, proxyInfo.type);
                mProxyManager.addProxyPrim(actualPath, proxyCollisionFilterPath);

                for (int slot = 0; slot < 2; ++slot)
                {
                    const SdfPath proxyTargetPath = getProxyTargetPath(proxyCollisionFilterPath, slot);
                    UsdPrim proxyTargetPrim = gStage->GetPrimAtPath(proxyTargetPath);
                    if (proxyTargetPrim)
                    {
                        const SdfPath targetPath = collisionFilterProxyInfo.actualTargets[slot];
                        mProxyManager.addProxyPrim(targetPath, proxyTargetPath);
                        mBufferCollisionFilterTargetPathsToUpdateTransform.insert(targetPath);
                        updateBufferDeformablePathsToUpdateTopology(targetPath);
                    }
                }
            }

            numActiveProxies++;
        }
        else
        {
            const SdfPath proxyCollisionFilterPath = getProxyCollisionFilterPathFromRoot(collisionFilterProxyInfo.proxyRootPath);
            if (!proxyCollisionFilterPath.IsEmpty())
            {
                mProxyManager.removeProxyPrim(actualPath, proxyCollisionFilterPath);

                for (int slot = 0; slot < 2; ++slot)
                {
                    const SdfPath proxyTargetPath = getProxyTargetPath(proxyCollisionFilterPath, slot);
                    if (!proxyTargetPath.IsEmpty())
                    {
                        const SdfPath targetPath = collisionFilterProxyInfo.actualTargets[slot];
                        mProxyManager.removeProxyPrim(targetPath, proxyTargetPath);
                        updateBufferDeformablePathsToUpdateTopology(targetPath);
                    }
                }
            }
        }
    }

    return numActiveProxies;
}

void DeformableBodyVisualizationManager::clearCollisionFiltersBuffers()
{
    mBufferCollisionFilterPathsToAdd.clear();
    mBufferCollisionFilterPathsToUpdate.clear();
    mBufferCollisionFilterPathsToRemove.clear();

    mBufferCollisionFilterTargetPathsToUpdateTransform.clear();

    mBufferModeDirtyCollisionFilters = false;
}

void DeformableBodyVisualizationManager::updateBufferDeformablePathsToUpdateTopology(const SdfPath targetPath)
{
    if (!gStage)
        return;

    // Update mBufferDeformablePathsToUpdateTopology if the input path is a collision mesh path of a volume/surface deformable
    const SdfPath actualDeformableBodyPath = mActualObjectToDeformableBodyMap[targetPath];
    const UsdPrim collMeshPrim = gStage->GetPrimAtPath(targetPath);
    if (!actualDeformableBodyPath.IsEmpty() && collMeshPrim && collMeshPrim.HasAPI<UsdPhysicsCollisionAPI>())
    {
        mBufferDeformablePathsToUpdateTopology.insert(actualDeformableBodyPath);
    }
}

void DeformableBodyVisualizationManager::addAttachment(const SdfPath attachmentPath)
{
    if (mActualAttachments.count(attachmentPath) == 0)
    {
        // update tracking
        mActualAttachments.insert(attachmentPath);

        AttachmentProxyInfo* attachmentProxyInfo = createAttachmentProxyInfo(attachmentPath);
        if (!attachmentProxyInfo)
            return;

        TargetPathPair targetPaths = getAttachmentTargets(attachmentPath);

        for (uint32_t slot = 0; slot < 2; ++slot)
        {
            SdfPath targetPath = targetPaths[slot];
            if (!targetPath.IsEmpty())
            {
                SdfPathToSdfPathSetMap::iterator tit = mTargetToActualAttachments.find(targetPath);
                if (tit != mTargetToActualAttachments.end())
                {
                    SdfPathSet& targetAttachments = tit->second;
                    targetAttachments.insert(attachmentPath);
                }
                else
                {
                    SdfPathSet targetAttachments;
                    targetAttachments.insert(attachmentPath);
                    mTargetToActualAttachments.insert({ targetPath, targetAttachments });
                }
            }
        }

        mProxyManager.addProxy(attachmentPath, *attachmentProxyInfo);

        // update events
        mProxyManager.bufferUpdateActive(attachmentPath);
    }
}

void DeformableBodyVisualizationManager::removeAttachment(const SdfPath attachmentPath)
{
    if (mActualAttachments.find(attachmentPath) == mActualAttachments.end())
        return;

    AttachmentProxyInfo* attachmentProxyInfo = getAttachmentProxyInfo(attachmentPath);
    if (attachmentProxyInfo)
    {
        for (uint32_t slot = 0; slot < 2; ++slot)
        {
            SdfPath targetPath = attachmentProxyInfo->actualTargets[slot];
            if (targetPath.IsEmpty())
                continue;

            SdfPathToSdfPathSetMap::iterator tit = mTargetToActualAttachments.find(targetPath);
            if (tit != mTargetToActualAttachments.end())
            {
                SdfPathSet& targetAttachments = tit->second;
                if (targetAttachments.size() == 1)
                {
                    mTargetToActualAttachments.erase(targetPath);
                }
                else
                {
                    targetAttachments.erase(attachmentPath);
                }
            }

            AttachmentTargetInfo* targetInfo = getAttachmentTargetInfo(targetPath);
            if (targetInfo)
            {
                if (targetInfo->numAttachments == 1)
                {
                    releaseTargetInfo(targetInfo);
                    mTargetToAttachmentInfo.erase(targetPath);
                }
                else
                {
                    targetInfo->numAttachments--;
                }
            }
        }
    }

    // update events  
    mProxyManager.bufferUpdateActive(attachmentPath);

    mActualAttachments.erase(attachmentPath);

    mProxyManager.removeProxy(attachmentPath, attachmentProxyInfo);

    // remove session layer representation
    const SdfPath proxyAttachmentPath = getProxyAttachmentPath(attachmentPath);
    if (!proxyAttachmentPath.IsEmpty())
    {
        for (uint32_t slot = 0; slot < 2; ++slot)
        {
            SdfPath samplePointInstancerPath = getSessionSamplePointInstancerPath(proxyAttachmentPath, slot);
            mProxyManager.removeProxyPrim(attachmentPath, samplePointInstancerPath);
            mProxyManager.removeProxyPrim(attachmentPath, getSessionPointInstancerPrototypePath(samplePointInstancerPath));
        }
    }
}

void DeformableBodyVisualizationManager::updateAttachment(const SdfPath attachmentPath)
{
    const UsdPrim attachmentPrim = gStage->GetPrimAtPath(attachmentPath);

    if (attachmentPrim)
    {
        removeAttachment(attachmentPath);
        addAttachment(attachmentPath);
    }
}

void DeformableBodyVisualizationManager::addCollisionFilter(const SdfPath collisionFilterPath)
{
    if (mActualCollisionFilters.count(collisionFilterPath) == 0)
    {
        // update tracking
        mActualCollisionFilters.insert(collisionFilterPath);

        CollisionFilterProxyInfo* collisionFilterProxyInfo = createCollisionFilterProxyInfo(collisionFilterPath);
        if (!collisionFilterProxyInfo)
            return;

        TargetPathPair targetPaths = getCollisionFilterTargets(collisionFilterPath);

        for (uint32_t slot = 0; slot < 2; ++slot)
        {
            SdfPath targetPath = targetPaths[slot];
            if (!targetPath.IsEmpty())
            {
                SdfPathToSdfPathSetMap::iterator tit = mTargetToActualCollisionFilters.find(targetPath);
                if (tit != mTargetToActualCollisionFilters.end())
                {
                    SdfPathSet& targetCollisionFilters = tit->second;
                    targetCollisionFilters.insert(collisionFilterPath);
                }
                else
                {
                    SdfPathSet targetCollisionFilters;
                    targetCollisionFilters.insert(collisionFilterPath);
                    mTargetToActualCollisionFilters.insert({ targetPath, targetCollisionFilters });
                }
            }
        }

        mProxyManager.addProxy(collisionFilterPath, *collisionFilterProxyInfo);

        // update events
        mProxyManager.bufferUpdateActive(collisionFilterPath);
    }
}

void DeformableBodyVisualizationManager::removeCollisionFilter(const SdfPath collisionFilterPath)
{
    if (mActualCollisionFilters.find(collisionFilterPath) == mActualCollisionFilters.end())
        return;

    CollisionFilterProxyInfo* collisionFilterProxyInfo = getCollisionFilterProxyInfo(collisionFilterPath);
    if (collisionFilterProxyInfo)
    {
        for (uint32_t slot = 0; slot < 2; ++slot)
        {
            SdfPath targetPath = collisionFilterProxyInfo->actualTargets[slot];
            if (targetPath.IsEmpty())
                continue;

            SdfPathToSdfPathSetMap::iterator tit = mTargetToActualCollisionFilters.find(targetPath);
            if (tit != mTargetToActualCollisionFilters.end())
            {
                SdfPathSet& targetCollisionFilters = tit->second;
                if (targetCollisionFilters.size() == 1)
                {
                    mTargetToActualCollisionFilters.erase(targetPath);
                }
                else
                {
                    targetCollisionFilters.erase(collisionFilterPath);
                }
            }

            CollisionFilterTargetInfo* targetInfo = getCollisionFilterTargetInfo(targetPath);
            if (targetInfo)
            {
                if (targetInfo->numCollisionFilters == 1)
                {
                    releaseTargetInfo(targetInfo);
                    mTargetToCollisionFilterInfo.erase(targetPath);
                }
                else
                {
                    targetInfo->numCollisionFilters--;
                }
            }

            auto it = mActualObjectToDeformableBodyMap.find(targetPath);
            if (it != mActualObjectToDeformableBodyMap.end())
            {
                mBufferDeformablePathsToUpdateTopology.insert(it->second);
            }
        }
    }

    // update events  
    mProxyManager.bufferUpdateActive(collisionFilterPath);

    mActualCollisionFilters.erase(collisionFilterPath);

    mProxyManager.removeProxy(collisionFilterPath, collisionFilterProxyInfo);
}

void DeformableBodyVisualizationManager::updateCollisionFilter(const SdfPath collisionFilterPath)
{
    const UsdPrim collisionFilterPrim = gStage->GetPrimAtPath(collisionFilterPath);

    if (collisionFilterPrim)
    {
        removeCollisionFilter(collisionFilterPath);
        addCollisionFilter(collisionFilterPath);
    }
}

SdfPath DeformableBodyVisualizationManager::getProxyAttachmentPath(const SdfPath attachmentPath)
{
    SdfPath proxyAttachmentPath;

    AttachmentProxyInfo* attachmentProxyInfo = getAttachmentProxyInfo(attachmentPath);
    if (attachmentProxyInfo)
    {
        proxyAttachmentPath = getProxyAttachmentPathFromRoot(attachmentProxyInfo->proxyRootPath);
    }    

    return proxyAttachmentPath;
}

void DeformableBodyVisualizationManager::createProxyAttachmentGeometries(const SdfPath attachmentPath)
{
    const SdfPath proxyAttachmentPath = getProxyAttachmentPath(attachmentPath);

    if (!proxyAttachmentPath.IsEmpty())
    {
        for (uint32_t slot = 0; slot < 2; ++slot)
        {
            SdfPath samplePointInstancerPath = getSessionSamplePointInstancerPath(proxyAttachmentPath, slot);
            createSessionPointInstancer(samplePointInstancerPath);
            mProxyManager.addProxyPrim(attachmentPath, samplePointInstancerPath);
            mProxyManager.addProxyPrim(attachmentPath, getSessionPointInstancerPrototypePath(samplePointInstancerPath));
        }
    }
}

void DeformableBodyVisualizationManager::setupProxyAttachmentGeometries(const SdfPath attachmentPath, const float visualizationScale)
{
    AttachmentProxyInfo* attachmentInfo = getAttachmentProxyInfo(attachmentPath);
    if (!attachmentInfo)
        return;

    const SdfPath proxyAttachmentPath = getProxyAttachmentPathFromRoot(attachmentInfo->proxyRootPath);

    if (!proxyAttachmentPath.IsEmpty())
    {
        float pointScale = getPointScale(attachmentPath, visualizationScale);

        for (uint32_t slot = 0; slot < 2; ++slot)
        {
            SdfPath targetPath = attachmentInfo->actualTargets[slot];

            SdfPath samplePointInstancerPath = getSessionSamplePointInstancerPath(proxyAttachmentPath, slot);

            setupSessionPointInstancer(samplePointInstancerPath, kRadiusSample[slot] * pointScale, kColorSample[slot]);
            attachmentInfo->numPointSamples[slot] = 0;
        }
    }
}

void DeformableBodyVisualizationManager::createSessionPointInstancer(const SdfPath pointInstancerPath)
{
    SdfPath pointPrototypePath = getSessionPointInstancerPrototypePath(pointInstancerPath);

    UsdGeomSphere samplePointPrototype = UsdGeomSphere::Get(gStage, pointPrototypePath);
    if (!samplePointPrototype)
    {
        samplePointPrototype = UsdGeomSphere::Define(gStage, pointPrototypePath);
        omni::kit::EditorUsd::setNoDelete(samplePointPrototype.GetPrim(), true);
        omni::kit::EditorUsd::setHideInStageWindow(samplePointPrototype.GetPrim(), true);
        omni::kit::EditorUsd::setNoSelectionOutline(samplePointPrototype.GetPrim(), true);
        samplePointPrototype.AddScaleOp().Set(GfVec3f(1.0f));
    }

    UsdGeomPointInstancer pointInstancer = UsdGeomPointInstancer::Get(gStage, pointInstancerPath);
    if (!pointInstancer)
    {
        pointInstancer = UsdGeomPointInstancer::Define(gStage, pointInstancerPath);
        omni::kit::EditorUsd::setNoDelete(pointInstancer.GetPrim(), true);
        omni::kit::EditorUsd::setHideInStageWindow(pointInstancer.GetPrim(), true);
        omni::kit::EditorUsd::setNoSelectionOutline(pointInstancer.GetPrim(), true);
    }
}

void DeformableBodyVisualizationManager::setupSessionPointInstancer(const SdfPath pointInstancerPath, const float radius, const GfVec3f& color)
{
    SdfPath pointPrototypePath = getSessionPointInstancerPrototypePath(pointInstancerPath);
    UsdGeomSphere pointPrototype = UsdGeomSphere::Get(gStage, pointPrototypePath);
    pointPrototype.GetRadiusAttr().Set(kPrototypeRadiusZero);
    pointPrototype.GetDisplayColorAttr().Set(VtArray<GfVec3f>(1, color));

    UsdGeomPointInstancer pointInstancer = UsdGeomPointInstancer::Get(gStage, pointInstancerPath);
    pointInstancer.GetPrototypesRel().AddTarget(pointPrototypePath);

    fillinDummyPoint(pointInstancer);
}

void DeformableBodyVisualizationManager::resetSessionPointInstancer(const SdfPath pointInstancerPath)
{
    UsdGeomPointInstancer pointInstancer = UsdGeomPointInstancer::Get(gStage, pointInstancerPath);
    if (!pointInstancer)
        return;

    SdfPath pointPrototypePath = getSessionPointInstancerPrototypePath(pointInstancerPath);
    UsdGeomSphere pointPrototype = UsdGeomSphere::Get(gStage, pointPrototypePath);
    if (!pointPrototype)
        return;

    pointPrototype.GetRadiusAttr().Set(kPrototypeRadiusZero);
    fillinDummyPoint(pointInstancer);
}

void DeformableBodyVisualizationManager::updateSessionPointInstancerRadius(const SdfPath pointInstancerPath, const float radius)
{
    SdfPath pointPrototypePath = getSessionPointInstancerPrototypePath(pointInstancerPath);
    UsdGeomSphere pointPrototype = UsdGeomSphere::Get(gStage, pointPrototypePath);
    if (pointPrototype)
    {
        pointPrototype.GetRadiusAttr().Set(double(radius));
    }

    UsdGeomPointInstancer pointInstancer = UsdGeomPointInstancer::Get(gStage, pointInstancerPath);
    if (pointInstancer && radius == kPrototypeRadiusZero)
    {
        fillinDummyPoint(pointInstancer);
    }
}

SdfPath DeformableBodyVisualizationManager::getProxyCollisionFilterPath(const SdfPath collisionFilterPath)
{
    SdfPath proxyCollisionFilterPath;

    CollisionFilterProxyInfo* collisionFilterProxyInfo = getCollisionFilterProxyInfo(collisionFilterPath);
    if (collisionFilterProxyInfo)
    {
        proxyCollisionFilterPath = getProxyCollisionFilterPathFromRoot(collisionFilterProxyInfo->proxyRootPath);
    }

    return proxyCollisionFilterPath;
}

bool isVtxXformAttachment(SdfPath attachmentPath)
{
    bool result = false;

    TfType vtxXformAttachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxXformAttachment);
    const UsdPrim prim = gStage->GetPrimAtPath(attachmentPath);
    if (prim && prim.IsA(vtxXformAttachmentType))
    {
        result = true;
    }

    return result;
}

bool isVtxTetAttachment(SdfPath attachmentPath)
{
    bool result = false;

    TfType vtxTetAttachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxTetAttachment);
    const UsdPrim prim = gStage->GetPrimAtPath(attachmentPath);
    if (prim && prim.IsA(vtxTetAttachmentType))
    {
        result = true;
    }

    return result;
}

bool isElementCollisionFilter(SdfPath collisionFilterPath)
{
    bool result = false;

    TfType elementCollisionFilterType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->ElementCollisionFilter);
    const UsdPrim prim = gStage->GetPrimAtPath(collisionFilterPath);
    if (prim && prim.IsA(elementCollisionFilterType))
    {
        result = true;
    }

    return result;
}

UsdPrim DeformableBodyVisualizationManager::createProxyPrim(SdfPath proxyPath, ProxyInfoType::Enum type)
{
    UsdGeomScope proxyScope = UsdGeomScope::Define(gStage, proxyPath);
    CARB_ASSERT(proxyScope);
    omni::kit::EditorUsd::setNoDelete(proxyScope.GetPrim(), true);
    omni::kit::EditorUsd::setHideInStageWindow(proxyScope.GetPrim(), true);

    for (int slot = 0; slot < 2; ++slot)
    {
        const SdfPath proxyTargetPath = getProxyTargetPath(proxyPath, slot);
        UsdGeomXform proxyTargetXform = UsdGeomXform::Get(gStage, proxyTargetPath);
        if (!proxyTargetXform)
        {
            proxyTargetXform = UsdGeomXform::Define(gStage, proxyTargetPath);
            omni::kit::EditorUsd::setNoDelete(proxyTargetXform.GetPrim(), true);
            omni::kit::EditorUsd::setHideInStageWindow(proxyTargetXform.GetPrim(), true);

            UsdGeomXformOp opTransform;
            opTransform = proxyTargetXform.AddTransformOp();
            opTransform.Set(GfMatrix4d(1.0)); // workaround for hydra update not working if op value is set in later update.
        }
    }

    return proxyScope.GetPrim();
}

DeformableBodyVisualizationManager::AttachmentProxyInfo* DeformableBodyVisualizationManager::createAttachmentProxyInfo(SdfPath attachmentPath)
{
    if (!gStage)
        return nullptr;

    AttachmentProxyInfo* attachmentProxyInfo = new AttachmentProxyInfo();
    attachmentProxyInfo->client = this;

    if (isVtxXformAttachment(attachmentPath))
        attachmentProxyInfo->type = ProxyInfoType::eVtxXformAttachment;
    else if (isVtxTetAttachment(attachmentPath))
        attachmentProxyInfo->type = ProxyInfoType::eVtxTetAttachment;
    else
        attachmentProxyInfo->type = ProxyInfoType::eNone;

    TargetPathPair targetPaths = getAttachmentTargets(attachmentPath);

    for (uint32_t slot = 0; slot < 2; ++slot)
    {
        SdfPath targetPath = targetPaths[slot];
        if (!targetPath.IsEmpty())
        {
            SdfPathToTargetInfoMap::iterator tit = mTargetToAttachmentInfo.find(targetPath);
            if (tit == mTargetToAttachmentInfo.end())
            {
                //new target
                AttachmentTargetInfo* info = createAttachmentTargetInfo(targetPath);
                if (info)
                {
                    attachmentProxyInfo->targetInfos[slot] = info;
                    mTargetToAttachmentInfo.insert({ targetPath, info });
                    info->numAttachments = 1;
                }
            }
            else
            {
                attachmentProxyInfo->targetInfos[slot] = tit->second;
                tit->second->numAttachments++;
            }
        }

        attachmentProxyInfo->actualTargets[slot] = targetPath;
        attachmentProxyInfo->numPointSamples[slot] = 0;
    }

    attachmentProxyInfo->actualAttachmentPath = attachmentPath;

    return attachmentProxyInfo;
}

DeformableBodyVisualizationManager::AttachmentProxyInfo* DeformableBodyVisualizationManager::getAttachmentProxyInfo(SdfPath attachmentPath)
{
    ProxyInfo* proxyInfo = mProxyManager.getProxyInfo(attachmentPath);
    return (proxyInfo && (proxyInfo->type == ProxyInfoType::eVtxXformAttachment || proxyInfo->type == ProxyInfoType::eVtxTetAttachment)) ? static_cast<AttachmentProxyInfo*>(proxyInfo) : nullptr;
}

DeformableBodyVisualizationManager::CollisionFilterProxyInfo* DeformableBodyVisualizationManager::createCollisionFilterProxyInfo(SdfPath collisionFilterPath)
{
    if (!gStage)
        return nullptr;

    CollisionFilterProxyInfo* collisionFilterProxyInfo = new CollisionFilterProxyInfo();
    collisionFilterProxyInfo->client = this;

    if (isElementCollisionFilter(collisionFilterPath))
        collisionFilterProxyInfo->type = ProxyInfoType::eElementCollisionFilter;
    else
        collisionFilterProxyInfo->type = ProxyInfoType::eNone;

    TargetPathPair targetPaths = getCollisionFilterTargets(collisionFilterPath);

    for (uint32_t slot = 0; slot < 2; ++slot)
    {
        SdfPath targetPath = targetPaths[slot];
        if (!targetPath.IsEmpty())
        {
            SdfPathToCollisionFilterTargetInfoMap::iterator tit = mTargetToCollisionFilterInfo.find(targetPath);
            if (tit == mTargetToCollisionFilterInfo.end())
            {
                //new target
                CollisionFilterTargetInfo* info = createCollisionFilterTargetInfo(targetPath);
                if (info)
                {
                    collisionFilterProxyInfo->collisionFilterTargetInfos[slot] = info;
                    mTargetToCollisionFilterInfo.insert({ targetPath, info });
                    info->numCollisionFilters = 1;
                }
            }
            else
            {
                collisionFilterProxyInfo->collisionFilterTargetInfos[slot] = tit->second;
                tit->second->numCollisionFilters++;
            }
        }

        collisionFilterProxyInfo->actualTargets[slot] = targetPath;
    }

    collisionFilterProxyInfo->actualCollisionFilterPath = collisionFilterPath;

    return collisionFilterProxyInfo;
}

DeformableBodyVisualizationManager::CollisionFilterProxyInfo* DeformableBodyVisualizationManager::getCollisionFilterProxyInfo(SdfPath collisionFilterPath)
{
    ProxyInfo* proxyInfo = mProxyManager.getProxyInfo(collisionFilterPath);
    return (proxyInfo && (proxyInfo->type == ProxyInfoType::eElementCollisionFilter)) ? static_cast<CollisionFilterProxyInfo*>(proxyInfo) : nullptr;
}

VtArray<int> convertInt4ArrayIntArray(const VtArray<GfVec4i>& int4Array)
{
    VtArray<int32_t> result;
    result.reserve(int4Array.size() * 4);

    for (const GfVec4i& vec : int4Array)
    {
        for (int i = 0; i < 4; ++i)
        {
            result.push_back((int32_t)vec[i]);
        }    
    }

    return result;
}

DeformableBodyVisualizationManager::AttachmentTargetInfo* DeformableBodyVisualizationManager::createAttachmentTargetInfo(const SdfPath targetPath)
{
    AttachmentTargetInfo* targetInfo = new AttachmentTargetInfo();
    UsdPrim targetPrim = gStage->GetPrimAtPath(targetPath);
    if (targetPrim)
    {
        TfType volumeSimType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->VolumeDeformableSimAPI);
        TfType surfaceSimType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->SurfaceDeformableSimAPI);
        if (targetPrim.HasAPI(volumeSimType))
        {
            VtArray<GfVec3f> points;
            getGeomPoints(targetPrim, &points);

            VtArray<GfVec4i> tetVtxIndices;
            UsdAttribute indsAttr = targetPrim.GetAttribute(UsdGeomTokens->tetVertexIndices);
            indsAttr.Get(&tetVtxIndices);
            VtArray<int32_t> indices = convertInt4ArrayIntArray(tetVtxIndices);

            //compute volume for scale heuristic
            float scaleVolume = 0.0f;
            bool valid = points.size() > 0 && indices.size() > 0;
            int32_t numPoints = int32_t(indices.size());
            for (size_t t = 0; t < indices.size() / 4; ++t)
            {
                int32_t i0 = indices[t * 4 + 0];
                int32_t i1 = indices[t * 4 + 1];
                int32_t i2 = indices[t * 4 + 2];
                int32_t i3 = indices[t * 4 + 3];
                valid &= (i0 < numPoints&& i1 < numPoints&& i2 < numPoints&& i3 < numPoints);
                if (valid)
                {
                    const GfVec3f& p0 = points[i0];
                    const GfVec3f& p1 = points[i1];
                    const GfVec3f& p2 = points[i2];
                    const GfVec3f& p3 = points[i3];
                    float tetVolume = std::abs((p1 - p0) * GfCross(p2 - p0, p3 - p0)) / 6.0f;
                    scaleVolume += tetVolume;
                }
            }

            if (valid)
            {
                targetInfo->pointScale = std::pow(scaleVolume / points.size(), 1.0f / 3.0f) * kEdgeToPointScale;
                targetInfo->type = AttachmentTargetInfo::eVolumeDeformableBody;
            }
        }
        else if (targetPrim.HasAPI(surfaceSimType))
        {
            VtArray<GfVec3f> points;
            VtArray<int32_t> triVtxIndices;
            getGeomPoints(targetPrim, &points);
            UsdAttribute indsAttr = targetPrim.GetAttribute(UsdGeomTokens->faceVertexIndices);
            indsAttr.Get(&triVtxIndices);
            bool valid = points.size() > 0 && triVtxIndices.size() > 0;
            if (valid)
            {
                targetInfo->pointScale = getPointScale(targetPrim, points);

                targetInfo->type = AttachmentTargetInfo::eSurfaceDeformableBody;
            }
        }
        else if (targetPrim.IsA<UsdGeomXformable>())
        {
            targetInfo->type = AttachmentTargetInfo::eXformable;
        }
    }
    if (targetInfo->type == AttachmentTargetInfo::eNone)
    {
        delete targetInfo;
        return nullptr;
    }
    return targetInfo;
}

void DeformableBodyVisualizationManager::releaseTargetInfo(BaseTargetInfo* targetInfo)
{
    if (targetInfo)
        delete targetInfo;
}

DeformableBodyVisualizationManager::AttachmentTargetInfo* DeformableBodyVisualizationManager::getAttachmentTargetInfo(SdfPath targetPath)
{
    auto it = mTargetToAttachmentInfo.find(targetPath);
    if (it != mTargetToAttachmentInfo.end())
    {
        return it->second;
    }
    return nullptr;
}

float DeformableBodyVisualizationManager::getPointScale(const SdfPath attachmentPath, const float visualizationScale)
{
    float pointScale = FLT_MAX;

    UsdGeomXformCache xformCache;
    UsdTimeCode time = UsdTimeCode(gTimeline->getCurrentTime() * gStage->GetTimeCodesPerSecond());
    xformCache.SetTime(time);

    AttachmentProxyInfo* attachmentInfo = getAttachmentProxyInfo(attachmentPath);
    if (attachmentInfo)
    {
        for (uint32_t slot = 0; slot < 2; ++slot)
        {
            const SdfPath targetPath = attachmentInfo->actualTargets[slot];
            auto it = mTargetToAttachmentInfo.find(attachmentInfo->actualTargets[slot]);
            if (it != mTargetToAttachmentInfo.end())
            {
                const AttachmentTargetInfo& targetInfo = *it->second;
                UsdPrim targetPrim = gStage->GetPrimAtPath(targetPath);
                if (targetPrim && targetInfo.pointScale > 0.0f)
                {
                    const GfVec3d scale = GfTransform(xformCache.GetLocalToWorldTransform(targetPrim)).GetScale();
                    float maxScaleDim = float(scale[0]);
                    maxScaleDim = std::max(maxScaleDim, float(scale[1]));
                    maxScaleDim = std::max(maxScaleDim, float(scale[2]));

                    pointScale = std::min(maxScaleDim * targetInfo.pointScale, pointScale);
                }
            }
        }
    }

    pointScale = (pointScale == FLT_MAX) ? 0.0f : pointScale;
    return pointScale * visualizationScale;
}

float DeformableBodyVisualizationManager::getPointScale(const UsdPrim& targetMeshPrim, const VtArray<GfVec3f>& restPoints) const
{
    //compute area for scale heuristic, use original mesh and rest positions. TODO, change the desc data
    float scale = 0.0f;
    UsdGeomMesh mesh(targetMeshPrim);
    if (mesh && restPoints.size() > 0)
    {
        float scaleArea = 0.0f;
        VtArray<int32_t> faceVertexCounts;
        VtArray<int32_t> faceVertexIndices;
        mesh.GetFaceVertexCountsAttr().Get(&faceVertexCounts);
        mesh.GetFaceVertexIndicesAttr().Get(&faceVertexIndices);
        int32_t offset = 0;
        int32_t numPoints = int32_t(restPoints.size());
        int32_t numIndices = int32_t(faceVertexIndices.size());
        for (size_t f = 0; f < faceVertexCounts.size(); ++f)
        {
            GfVec3f triAreaVec(0.0f);
            const int32_t count = faceVertexCounts[f];
            if (count > 2 && offset + count - 1 < numIndices)
            {
                int32_t i0 = faceVertexIndices[offset];
                if (i0 < numPoints)
                {
                    const GfVec3f& p0 = restPoints[i0];
                    for (int32_t v = 1; v < count - 1; ++v)
                    {
                        int32_t i1 = faceVertexIndices[offset + v];
                        int32_t i2 = faceVertexIndices[offset + v + 1];
                        if (i1 < numPoints && i2 < numPoints)
                        {
                            const GfVec3f& p1 = restPoints[i1];
                            const GfVec3f& p2 = restPoints[i2];
                            triAreaVec += GfCross(p1 - p0, p2 - p0);
                        }
                    }
                }

                offset += count;
                scaleArea += triAreaVec.Normalize() / 2.0f;
            }
        }

        scale = std::sqrt(scaleArea / restPoints.size()) * kEdgeToPointScale;
    }
    return scale;
}

DeformableBodyVisualizationManager::CollisionFilterTargetInfo* DeformableBodyVisualizationManager::createCollisionFilterTargetInfo(const SdfPath targetPath)
{
    CollisionFilterTargetInfo* targetInfo = new CollisionFilterTargetInfo();
    UsdPrim targetPrim = gStage->GetPrimAtPath(targetPath);
    if (targetPrim && targetPrim.HasAPI<UsdPhysicsCollisionAPI>())
    {
        if (targetPrim.IsA<UsdGeomTetMesh>())
        {
            // element indices represents surface triangle indices for UsdGeomTetMesh
            targetInfo->type = CollisionFilterTargetInfo::eVolumeDeformableBody;
        }
        else if (targetPrim.IsA<UsdGeomMesh>())
        {
            // element indices represents triangle indices for UsdGeomMesh
            targetInfo->type = CollisionFilterTargetInfo::eSurfaceDeformableBody;
        }
        else if (targetPrim.IsA<UsdGeomXformable>())
        {
            targetInfo->type = CollisionFilterTargetInfo::eXformable;
        }
    }
    if (targetInfo->type == CollisionFilterTargetInfo::eNone)
    {
        delete targetInfo;
        return nullptr;
    }
    return targetInfo;
}

DeformableBodyVisualizationManager::CollisionFilterTargetInfo* DeformableBodyVisualizationManager::getCollisionFilterTargetInfo(SdfPath targetPath)
{
    auto it = mTargetToCollisionFilterInfo.find(targetPath);
    if (it != mTargetToCollisionFilterInfo.end())
    {
        return it->second;
    }
    return nullptr;
}

uint32_t DeformableBodyVisualizationManager::updateVolumeDeformableTargetAttachmentSamplePoints(UsdGeomPointInstancer& samplePointInstancer, const SdfPath attachmentPath, const SdfPath& simMeshPath,
    const int slot, AttachmentProxyInfo* attachmentInfo)
{
    VtArray<GfVec3f> attachmentPoints;

    if (attachmentInfo)
    {
        UsdPrim simMeshPrim = gStage->GetPrimAtPath(simMeshPath);
        VtArray<GfVec3f> simMeshPoints;
        getGeomPoints(simMeshPrim, &simMeshPoints);

        VtArray<GfVec4i> tetVtxIndices;
        UsdAttribute indsAttr = simMeshPrim.GetAttribute(UsdGeomTokens->tetVertexIndices);
        indsAttr.Get(&tetVtxIndices);

        TfType vtxXformAttachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxXformAttachment);
        TfType vtxTetAttachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxTetAttachment);
        const UsdPrim& attachmentPrim = gStage->GetPrimAtPath(attachmentPath);
        if (attachmentPrim.IsA(vtxXformAttachmentType))
        {
            UsdAttribute vtxIndicesAttr = attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->vtxIndicesSrc0);
            getVtxAttachmentPoints(attachmentPoints, vtxIndicesAttr, simMeshPoints);
        }
        else if (attachmentPrim.IsA(vtxTetAttachmentType))
        {
            AttachmentTargetInfo::Type otherType = attachmentInfo->targetInfos[1 - slot]->type;
            if (slot == 0 && otherType == AttachmentTargetInfo::Type::eVolumeDeformableBody)
            {
                UsdAttribute vtxIndicesAttr = attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->vtxIndicesSrc0);
                getVtxAttachmentPoints(attachmentPoints, vtxIndicesAttr, simMeshPoints);
            }
            else if (slot == 1 || otherType == AttachmentTargetInfo::Type::eSurfaceDeformableBody)
            {
                UsdAttribute tetIndicesAttr = attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->tetIndicesSrc1);
                UsdAttribute tetCoordsAttr = attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->tetCoordsSrc1);
                getTetAttachmentPoints(attachmentPoints, tetIndicesAttr, tetCoordsAttr, tetVtxIndices, simMeshPoints);
            }
        }
    }

    samplePointInstancer.GetPositionsAttr().Set(attachmentPoints);
    
    return uint32_t(attachmentPoints.size());
}

uint32_t DeformableBodyVisualizationManager::updateSurfaceDeformableTargetAttachmentSamplePoints(UsdGeomPointInstancer& samplePointInstancer, const SdfPath attachmentPath, const SdfPath& simMeshPath,
    const int slot, AttachmentProxyInfo* attachmentInfo)
{
    VtArray<GfVec3f> attachmentPoints;

    if (attachmentInfo)
    {
        UsdPrim simMeshPrim = gStage->GetPrimAtPath(simMeshPath);
        VtArray<GfVec3f> simMeshPoints;
        getGeomPoints(simMeshPrim, &simMeshPoints);

        TfType vtxXformAttachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxXformAttachment);
        TfType vtxTetAttachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxTetAttachment);

        const UsdPrim& attachmentPrim = gStage->GetPrimAtPath(attachmentPath);
        if (attachmentPrim.IsA(vtxXformAttachmentType) || attachmentPrim.IsA(vtxTetAttachmentType))
        {
            UsdAttribute vtxIndicesAttr = attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->vtxIndicesSrc0);
            getVtxAttachmentPoints(attachmentPoints, vtxIndicesAttr, simMeshPoints);
        }
    }

    samplePointInstancer.GetPositionsAttr().Set(attachmentPoints);

    return uint32_t(attachmentPoints.size());
}

void DeformableBodyVisualizationManager::updateAttachmentGeometry(const SdfPath attachmentPath, const float visualizationScale)
{
    // get attachment
    TfType vtxXformAttachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxXformAttachment);
    TfType vtxTetAttachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxTetAttachment);
    const UsdPrim attachmentPrim = gStage->GetPrimAtPath(attachmentPath);
    if (!attachmentPrim || (!attachmentPrim.IsA(vtxXformAttachmentType) && !attachmentPrim.IsA(vtxTetAttachmentType)))
        return;

    AttachmentProxyInfo* attachmentProxyInfo = getAttachmentProxyInfo(attachmentPath);
    if (!attachmentProxyInfo)
        return;

    const SdfPath proxyAttachmentPath = getProxyAttachmentPath(attachmentPath);
    if (proxyAttachmentPath.IsEmpty())
        return;

    for (int32_t s = 0; s < 2; ++s)
    {
        SdfPath targetPath = attachmentProxyInfo->actualTargets[s];

        uint32_t numPointSamples = 0;

        SdfPath samplePointInstancerPath = getSessionSamplePointInstancerPath(proxyAttachmentPath, s);

        UsdGeomPointInstancer samplePointInstancer = UsdGeomPointInstancer::Get(gStage, samplePointInstancerPath);
        if (!samplePointInstancer)
            return;

        UsdPrim targetPrim = gStage->GetPrimAtPath(targetPath);
        auto it = mTargetToAttachmentInfo.find(targetPath);
        if (targetPrim && it != mTargetToAttachmentInfo.end())
        {
            const AttachmentTargetInfo& targetInfo = *it->second;
            if (targetInfo.type == AttachmentTargetInfo::eXformable)
            {
                UsdAttribute locationsAttr = attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->localPositionsSrc1);
                VtArray<GfVec3f> locations;
                if (locationsAttr)
                    locationsAttr.Get(&locations);

                samplePointInstancer.GetPositionsAttr().Set(locations);
                numPointSamples = uint32_t(locations.size());
            }
            else
            {
                if (targetInfo.type == AttachmentTargetInfo::eVolumeDeformableBody)
                {
                    numPointSamples = updateVolumeDeformableTargetAttachmentSamplePoints(samplePointInstancer, attachmentPath, targetPath, s, attachmentProxyInfo);
                }
                else if (targetInfo.type == AttachmentTargetInfo::eSurfaceDeformableBody)
                {
                    numPointSamples = updateSurfaceDeformableTargetAttachmentSamplePoints(samplePointInstancer, attachmentPath, targetPath, s, attachmentProxyInfo);
                }
            }
        }
        else
        {
            //removed target
            numPointSamples = 0;
        }

        //Update selection group in case of changing number of points (workaround)
        const auto usdContext = omni::usd::UsdContext::getContext();
        uint8_t group = mProxyManager.isSelected(attachmentPath) ? gProxySelectionGroup : 0;

        if (gStage->GetPrimAtPath(samplePointInstancerPath))
        {
            usdContext->setSelectionGroup(group, samplePointInstancerPath.GetString());

            if (attachmentProxyInfo->numPointSamples[s] != numPointSamples)
            {
                VtArray<int> protoIndices(numPointSamples, 0);
                samplePointInstancer.GetProtoIndicesAttr().Set(protoIndices);

                if (numPointSamples > 0)
                {
                    float pointScale = getPointScale(attachmentPath, visualizationScale);
                    updateSessionPointInstancerRadius(samplePointInstancerPath, kRadiusSample[s] * pointScale);
                }
                else
                {
                    resetSessionPointInstancer(samplePointInstancerPath);
                }
            }
        }

        attachmentProxyInfo->numPointSamples[s] = numPointSamples;
    }
}

void DeformableBodyVisualizationManager::updateAttachmentTargetTransform(const SdfPath targetPath, UsdGeomXformCache& xformCache)
{
    // get target prim
    UsdPrim targetPrim = gStage->GetPrimAtPath(targetPath);
    if (!targetPrim)
        return;  // no need to update if there is no target prim

    if (mTargetToAttachmentInfo.count(targetPath))
    {
        const SdfPathSet& targetAttachments = mTargetToActualAttachments[targetPath];

        for (SdfPath attachmentPath : targetAttachments)
        {
            TfType vtxXformAttachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxXformAttachment);
            TfType vtxTetAttachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxTetAttachment);
            const UsdPrim& attachmentPrim = gStage->GetPrimAtPath(attachmentPath);
            if (!attachmentPrim || (!attachmentPrim.IsA(vtxXformAttachmentType) && !attachmentPrim.IsA(vtxTetAttachmentType)))
                continue;

            AttachmentProxyInfo* attachmentInfo = getAttachmentProxyInfo(attachmentPath);
            CARB_ASSERT(attachmentInfo && attachmentInfo->actualTargets[0] == targetPath || attachmentInfo->actualTargets[1] == targetPath);
            if (attachmentInfo)
            {
                int32_t slot = attachmentInfo->actualTargets[0] == targetPath ? 0 : 1;

                const SdfPath proxyAttachmentPath = getProxyAttachmentPath(attachmentPath);
                if (!proxyAttachmentPath.IsEmpty())
                {
                    SdfPath targetXformPath = getProxyTargetPath(proxyAttachmentPath, slot);

                    // get transformation with cache:
                    const GfMatrix4d targetToWorld = xformCache.GetLocalToWorldTransform(targetPrim);

                    UsdGeomXform targetXform = UsdGeomXform::Get(gStage, targetXformPath);
                    UsdAttribute localToWorldAttr = targetXform.GetPrim().GetAttribute(transformMatrixAttrName);
                    CARB_ASSERT(localToWorldAttr);
                    localToWorldAttr.Set(targetToWorld);

                    SdfPath samplePointInstancerPath = getSessionSamplePointInstancerPath(proxyAttachmentPath, slot);
                    updatePrototypeScale(samplePointInstancerPath, targetToWorld);
                }
            }
        }
    }
}

void DeformableBodyVisualizationManager::updateAttachmentTargetGeometry(const SdfPath targetPath)
{
    // get target prim
    UsdPrim targetPrim = gStage->GetPrimAtPath(targetPath);
    if (!targetPrim)
        return;  // no need to update if there is no target prim

    TfType volumeSimType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->VolumeDeformableSimAPI);
    TfType surfaceSimType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->SurfaceDeformableSimAPI);
    CARB_ASSERT(targetPrim.HasAPI(volumeSimType) || targetPrim.HasAPI(surfaceSimType));

    SdfPathToTargetInfoMap::const_iterator tit = mTargetToAttachmentInfo.find(targetPath);
    if (tit != mTargetToAttachmentInfo.cend())
    {
        const AttachmentTargetInfo& targetInfo = *tit->second;
        const SdfPathSet& targetAttachments = mTargetToActualAttachments[targetPath];
        for (SdfPath attachmentPath : targetAttachments)
        {
            if (mBufferAttachmentPathsToUpdateGeometry.count(attachmentPath) > 0)
            {
                // skip attachments which are going to be updated later on
                continue;
            }

            TfType vtxXformAttachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxXformAttachment);
            TfType vtxTetAttachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxTetAttachment);
            const UsdPrim attachmentPrim = gStage->GetPrimAtPath(attachmentPath);
            if (!attachmentPrim || (!attachmentPrim.IsA(vtxXformAttachmentType) && !attachmentPrim.IsA(vtxTetAttachmentType)))
                continue;

            AttachmentProxyInfo* attachmentProxyInfo = getAttachmentProxyInfo(attachmentPath);
            CARB_ASSERT(attachmentProxyInfo);
            if (attachmentProxyInfo)
            {
                CARB_ASSERT(attachmentProxyInfo->actualTargets[0] == targetPath || attachmentProxyInfo->actualTargets[1] == targetPath);
                int slot = (attachmentProxyInfo->actualTargets[0] == targetPath) ? 0 : 1;

                const SdfPath proxyAttachmentPath = getProxyAttachmentPath(attachmentPath);
                if (!proxyAttachmentPath.IsEmpty())
                {
                    SdfPath samplePointInstancerPath = getSessionSamplePointInstancerPath(proxyAttachmentPath, slot);

                    UsdGeomPointInstancer samplePointInstancer = UsdGeomPointInstancer::Get(gStage, samplePointInstancerPath);

                    if (targetInfo.type == AttachmentTargetInfo::eVolumeDeformableBody)
                    {
                        updateVolumeDeformableTargetAttachmentSamplePoints(samplePointInstancer, attachmentPath, targetPath, slot, attachmentProxyInfo);
                    }
                    else if (targetInfo.type == AttachmentTargetInfo::eSurfaceDeformableBody)
                    {
                        updateSurfaceDeformableTargetAttachmentSamplePoints(samplePointInstancer, attachmentPath, targetPath, slot, attachmentProxyInfo);
                    }
                }
            }
        }
    }
}

void DeformableBodyVisualizationManager::updateVisualizationScale(const SdfPath attachmentPath, const float visualizationScale)
{
    AttachmentProxyInfo* attachmentInfo = getAttachmentProxyInfo(attachmentPath);
    if (!attachmentInfo || attachmentInfo->proxyRootPath.IsEmpty())
        return;

    const SdfPath proxyAttachmentPath = getProxyAttachmentPathFromRoot(attachmentInfo->proxyRootPath);
    float pointScale = getPointScale(attachmentPath, visualizationScale);

    if (!proxyAttachmentPath.IsEmpty())
    {
        for (int32_t s = 0; s < 2; ++s)
        {
            SdfPath samplePointInstancerPath = getSessionSamplePointInstancerPath(proxyAttachmentPath, s);
            updateSessionPointInstancerRadius(samplePointInstancerPath, kRadiusSample[s] * pointScale * (attachmentInfo->numPointSamples[s] > 0 ? 1.0f : float(kPrototypeRadiusZero)));
        }
    }
}

void DeformableBodyVisualizationManager::clearAttachments()
{
    if (gStage)
    {
        omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());

        while (mActualAttachments.size())
        {
            removeAttachment(*mActualAttachments.begin());
        }
    }

    mActualAttachments.clear();
    mTargetToActualAttachments.clear();

    for (SdfPathToTargetInfoMap::const_iterator it = mTargetToAttachmentInfo.cbegin(); it != mTargetToAttachmentInfo.cend(); ++it)
    {
        releaseTargetInfo(it->second);
    }
    mTargetToAttachmentInfo.clear();

    clearAttachmentsBuffers();
}
