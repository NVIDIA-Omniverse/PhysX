// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "DebugVisualization.h"
#include "DebugColors.h"

#include <common/ui/ImguiDrawingUtils.h>
#include "carb/Defines.h"

#include <private/omni/physx/PhysxUsd.h>
#include <private/omni/physx/IPhysxUsdLoad.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxCooking.h>
#include <private/omni/physx/IPhysxCookingPrivate.h>
#include <omni/physx/IPhysxVisualization.h>
#include <omni/physx/IPhysxSettings.h>
#include <omni/physx/PhysxTokens.h>
#include <carb/settings/ISettings.h>

#include <omni/kit/ViewportTypes.h>
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/Selection.h>
#include <omni/usd/UsdContext.h>
#include <omni/timeline/ITimeline.h>

#include <omni/fabric/AttrNameAndType.h>
#include <omni/fabric/SimStageWithHistory.h>

using namespace carb::renderer;
using namespace pxr;
using namespace omni::physx::usdparser;
using namespace omni::renderer;
using namespace omni::physics::ui;


#define ENABLE_CHECKS 1

extern UsdStageRefPtr gStage;
omni::physx::IPhysxVisualization* gPhysXVisualization = nullptr;
extern omni::timeline::TimelinePtr gTimeline;
static omni::renderer::IDebugDraw* gDebugDraw;
extern omni::physx::usdparser::IPhysxUsdLoad* gUsdLoad;
extern bool gBlockNoticeHandle;
extern omni::physx::IPhysx *gPhysX;
extern omni::physx::IPhysxCooking* gPhysXCooking;
extern omni::physx::IPhysxCookingPrivate* gPhysXCookingPrivate;
extern carb::settings::ISettings* gSettings;

static constexpr char kAsyncRenderingPath[] = "app/asyncRendering";
static constexpr float kLineWidth = 1.0f;

// dim simplified AABBs to further indicate that they are not real colliders.
static constexpr float simplifiedAABBColorLuminance = 0.65f;

// Minimal camera movement to take into account.
static constexpr float kMinCameraMovement = 0.001f;

// Do not react to camera movements any sooner.
// In other words, AABB distance simplification will have this delay.
static constexpr float kCameraUpdateDelay = 0.5f; // [in seconds]

// Do not react to USD notifications (like add/remove/resync) immediately.
// Rather wait for this amount of time before reflecting on USD changes.
static constexpr float kLastUSDNotificationDelay = 0.015f; // [in seconds]
// If USD changes are still coming, so kLastUSDNotificationDelay keeps
// blocking updates, after kLastUSDChangeDelay we will update no matter what.
static constexpr float kLastUSDChangeDelay = 3.0f; // [in seconds]

// These 2 constants specify lower and upper time limit on USD xform notification changes.
// NOTE: The following does not apply to simulation/playing mode.
// kLastXformNotificationDelay - wait for this amount of time before reflecting on xform changes.
static constexpr float kLastXformNotificationDelay = 0.015f; // [in seconds]
// If xform changes are still coming, so kLastXformNotificationDelay keeps
// blocking movement, after kLastXformChangeDelay we will update no matter what.
static constexpr float kLastXformChangeDelay = 0.05f; // [in seconds]

namespace omni
{
namespace physx
{
namespace ui
{

bool hasNewCollisionResults(DebugVisualizationCollisionMesh *cmesh)
{
    bool ret = false;

    static uint32_t gLastCookCount=0;
    if (gPhysXCookingPrivate)
    {
        omni::physx::PhysxCookingStatistics statistics = gPhysXCookingPrivate->getCookingStatistics();
        if (statistics.totalFinishedTasks != gLastCookCount)
        {
            gLastCookCount =  statistics.totalFinishedTasks;
            ret = true;
        }
    }

    return ret;
}

usdparser::MeshKey ShapeUniqueKey::EmptyMeshKey;

void ShapeUniqueKey::FromShapeDesc(const usdparser::PhysxShapeDesc* desc, bool includeMeshScale)
{
    if (desc == nullptr)
    {
        return;
    }

    this->mObjType = desc->type;
    MeshKey& meshKey = this->mMeshKey;

    switch (desc->type)
    {
        case eTriangleMeshShape: {
            TriangleMeshPhysxShapeDesc* meshDesc = (TriangleMeshPhysxShapeDesc*)desc;
            meshKey = meshDesc->crc;

            if (includeMeshScale)
            {
                meshKey.setSignScale(&meshDesc->meshScale.x);
            }
        }
        break;

        case eConvexMeshShape: {
            ConvexMeshPhysxShapeDesc* convexDesc = (ConvexMeshPhysxShapeDesc*)desc;
            meshKey = convexDesc->crc;

            if (includeMeshScale)
            {
                meshKey.setSignScale(&convexDesc->meshScale.x);
            }
        }
        break;

        case eConvexMeshDecompositionShape: {
            ConvexMeshDecompositionPhysxShapeDesc* convexDecompositionDesc = (ConvexMeshDecompositionPhysxShapeDesc*)desc;
            meshKey = convexDecompositionDesc->crc;

            if (includeMeshScale)
            {
                meshKey.setSignScale(&convexDecompositionDesc->meshScale.x);
            }
        }
        break;

        case eSpherePointsShape: {
            const SpherePointsPhysxShapeDesc* sphereDesc = (SpherePointsPhysxShapeDesc*)desc;
            meshKey = sphereDesc->crc;

            if (includeMeshScale)
            {
                meshKey.setSignScale(&sphereDesc->meshScale.x);
            }

            meshKey.setMaxSpheres((uint32_t)sphereDesc->spheres.size());
        }
        break;

        case eCustomShape: {
            const CustomPhysxShapeDesc* customShapeDesc = (CustomPhysxShapeDesc*)desc;
            meshKey = EmptyMeshKey;
            meshKey.setMiscData((const uint8_t*)TfToken::HashFunctor()(customShapeDesc->customGeometryToken), sizeof(size_t));
        }
        break;

        case eCapsuleShape: {
            const CapsulePhysxShapeDesc* capsuleDesc = (CapsulePhysxShapeDesc*)desc;
            meshKey = EmptyMeshKey;
            meshKey.setFillMode(capsuleDesc->axis);
            // OM-120862: Capsule cannot be properly scaled, we need to create an unique hash which includes current props
            meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&capsuleDesc->radius), sizeof(capsuleDesc->radius));
            meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&capsuleDesc->halfHeight), sizeof(capsuleDesc->halfHeight));
        }
        break;

        case eCylinderShape: {
            CylinderPhysxShapeDesc* cylinderDesc = (CylinderPhysxShapeDesc*)desc;
            meshKey = EmptyMeshKey;
            meshKey.setFillMode(cylinderDesc->axis);
            meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&cylinderDesc->halfHeight), sizeof(cylinderDesc->halfHeight));
            meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&cylinderDesc->radius), sizeof(cylinderDesc->radius));
            meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&cylinderDesc->margin), sizeof(cylinderDesc->margin));
        }
        break;

        case eConeShape: {
            ConePhysxShapeDesc* coneDesc = (ConePhysxShapeDesc*)desc;
            meshKey = EmptyMeshKey;
            meshKey.setFillMode(coneDesc->axis);
            meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&coneDesc->halfHeight), sizeof(coneDesc->halfHeight));
            meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&coneDesc->radius), sizeof(coneDesc->radius));
            meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&coneDesc->margin), sizeof(coneDesc->margin));
        }
        break;

        default:
            meshKey = EmptyMeshKey;
    };
}

// returns null in 'type' is unsupported
const usdparser::PhysxShapeDesc* ShapeUtils::getUnitShapeDesc(const usdparser::PhysxShapeDesc* shapeDesc, usdparser::Axis axis)
{
    switch (shapeDesc->type)
    {
    case eSphereShape:
        static SpherePhysxShapeDesc* spherePhysxShapeDesc = new SpherePhysxShapeDesc();
        spherePhysxShapeDesc->radius = 1.0f;
        return spherePhysxShapeDesc;

    case eBoxShape:
        static BoxPhysxShapeDesc* boxPhysxShapeDesc = new BoxPhysxShapeDesc();
        boxPhysxShapeDesc->halfExtents = { 0.5f, 0.5f, 0.5f };
        return boxPhysxShapeDesc;

    case eBoundingSphereShape:
        static BoundingSpherePhysxShapeDesc* boundingSpherePhysxShapeDesc = new BoundingSpherePhysxShapeDesc();
        boundingSpherePhysxShapeDesc->radius = 1.0f;
        boundingSpherePhysxShapeDesc->positionOffset = { 0.0f, 0.0f, 0.0f };
        return boundingSpherePhysxShapeDesc;

    case eBoundingBoxShape:
        static BoundingBoxPhysxShapeDesc* boundingBoxPhysxShapeDesc = new BoundingBoxPhysxShapeDesc();
        boundingBoxPhysxShapeDesc->halfExtents = { 0.5f, 0.5f, 0.5f };
        boundingBoxPhysxShapeDesc->positionOffset = { 0.0f, 0.0f, 0.0f };
        boundingBoxPhysxShapeDesc->rotationOffset = { 0.0f, 0.0f, 0.0f, 0.0f };
        return boundingBoxPhysxShapeDesc;

    case eCapsuleShape:
        static CapsulePhysxShapeDesc* capsulePhysxShapeDesc = new CapsulePhysxShapeDesc();
        capsulePhysxShapeDesc->radius = 1.0f;
        capsulePhysxShapeDesc->halfHeight = 1.0f;
        capsulePhysxShapeDesc->axis = ((CapsulePhysxShapeDesc*)shapeDesc)->axis;
        return capsulePhysxShapeDesc;

    case eCylinderShape:
        static CylinderPhysxShapeDesc* cylinderPhysxShapeDesc = new CylinderPhysxShapeDesc();
        cylinderPhysxShapeDesc->radius = 1.0f;
        cylinderPhysxShapeDesc->halfHeight = 1.0f;
        cylinderPhysxShapeDesc->axis = ((CylinderPhysxShapeDesc*)shapeDesc)->axis;
        return cylinderPhysxShapeDesc;

    case eConeShape:
        static ConePhysxShapeDesc* conePhysxShapeDesc = new ConePhysxShapeDesc();
        conePhysxShapeDesc->radius = 1.0f;
        conePhysxShapeDesc->halfHeight = 1.0f;
        conePhysxShapeDesc->axis = ((ConePhysxShapeDesc*)shapeDesc)->axis;
        return conePhysxShapeDesc;
    case ePlaneShape:
        static PlanePhysxShapeDesc* planePhysxShapeDesc = new PlanePhysxShapeDesc();
        planePhysxShapeDesc->axis = ((PlanePhysxShapeDesc*)shapeDesc)->axis;
        return planePhysxShapeDesc;
    default: break;
    }

    return nullptr;
}

void ShapeUtils::getShapeScale(const usdparser::PhysxShapeDesc* desc, pxr::GfVec3f& scale)
{
    if (desc == nullptr)
    {
        return;
    }

    switch (desc->type)
    {
    case eSphereShape: {
        SpherePhysxShapeDesc* sphereDesc = (SpherePhysxShapeDesc*)desc;
        scale = { sphereDesc->radius, sphereDesc->radius, sphereDesc->radius };
    }
    break;

    case eBoxShape: {
        BoxPhysxShapeDesc* boxDesc = (BoxPhysxShapeDesc*)desc;
        scale = { boxDesc->halfExtents.x, boxDesc->halfExtents.y, boxDesc->halfExtents.z };
        scale *= 2.0f;
    }
    break;

    case eBoundingSphereShape: {
        BoundingSpherePhysxShapeDesc* sphereDesc = (BoundingSpherePhysxShapeDesc*)desc;
        scale = { sphereDesc->radius, sphereDesc->radius, sphereDesc->radius };
    }
    break;

    case eBoundingBoxShape: {
        BoundingBoxPhysxShapeDesc* boxDesc = (BoundingBoxPhysxShapeDesc*)desc;
        scale = { boxDesc->halfExtents.x, boxDesc->halfExtents.y, boxDesc->halfExtents.z };
        scale *= 2.0f;
    }
    break;

    case eCapsuleShape: {
        CapsulePhysxShapeDesc* capsuleDesc = (CapsulePhysxShapeDesc*)desc;
        scale = { capsuleDesc->radius, capsuleDesc->radius, capsuleDesc->radius };
        switch (capsuleDesc->axis)
        {
        case eX:
            scale[0] = capsuleDesc->halfHeight;
            break;

        case eY:
            scale[1] = capsuleDesc->halfHeight;
            break;

        case eZ:
            scale[2] = capsuleDesc->halfHeight;
            break;
        }
    }
    break;

    case eCylinderShape: {
        CylinderPhysxShapeDesc* cylinderDesc = (CylinderPhysxShapeDesc*)desc;
        scale = { cylinderDesc->radius, cylinderDesc->radius, cylinderDesc->radius };
        switch (cylinderDesc->axis)
        {
        case eX:
            scale[0] = cylinderDesc->halfHeight;
            break;

        case eY:
            scale[1] = cylinderDesc->halfHeight;
            break;

        case eZ:
            scale[2] = cylinderDesc->halfHeight;
            break;
        }
    }
    break;

    case eConeShape: {
        ConePhysxShapeDesc* coneDesc = (ConePhysxShapeDesc*)desc;
        scale = { coneDesc->radius, coneDesc->radius, coneDesc->radius };
        switch (coneDesc->axis)
        {
        case eX:
            scale[0] = coneDesc->halfHeight;
            break;

        case eY:
            scale[1] = coneDesc->halfHeight;
            break;

        case eZ:
            scale[2] = coneDesc->halfHeight;
            break;
        }

    }
    break;

    case ePlaneShape: {
        PlanePhysxShapeDesc* planeDesc = (PlanePhysxShapeDesc*)desc;
        scale = { planeDesc->localScale.x, planeDesc->localScale.y, planeDesc->localScale.z };
    }
    break;

    default:
        scale = { 1.0f, 1.0f, 1.0f };
    }
}

void ShapeUtils::setMeshScale(usdparser::PhysxShapeDesc* desc, const carb::Float3& scale)
{
    if (desc == nullptr)
    {
        return;
    }

    switch (desc->type)
    {
    case eTriangleMeshShape:
    case eSpherePointsShape:
        ((TriangleMeshPhysxShapeDesc*)desc)->meshScale = scale;
        break;
    case eConvexMeshDecompositionShape:
        ((ConvexMeshDecompositionPhysxShapeDesc*)desc)->meshScale = scale;
        break;
    case eConvexMeshShape:
        ((ConvexMeshPhysxShapeDesc*)desc)->meshScale = scale;
        break;
    default: break;
    }
}

RenderInstanceCache::RenderInstanceDesc* RenderInstanceCache::get(const pxr::SdfPath& path)
{
    auto it = mRenderInstanceDescMap.find(path);
    if (it != mRenderInstanceDescMap.end())
    {
        return &(it->second);
    }

    return nullptr;
}

bool RenderInstanceCache::getAndRemove(const pxr::SdfPath& path,
                                       omni::renderer::RenderInstanceBuffer& renderInstanceBuffer,
                                       size_t& renderInstanceNum,
                                       TriangleMeshBuffer*& triangleMeshBuffer,
                                       pxr::GfMatrix4f& matrix)
{
    auto it = mRenderInstanceDescMap.find(path);
    if (it != mRenderInstanceDescMap.end())
    {
        RenderInstanceDesc& rid = it->second;
        renderInstanceBuffer = rid.renderInstanceBuffer;
        renderInstanceNum = rid.renderInstanceNum;
        triangleMeshBuffer = rid.triangleMeshBuffer;
        matrix = rid.matrix;

        mRenderInstanceDescMap.erase(it);

        // now we must subtract -1 for all renderInstanceNums > renderInstanceNums
        for (auto& i : mRenderInstanceDescMap)
        {
            RenderInstanceDesc& rid = i.second;

#if ENABLE_CHECKS
            if (renderInstanceBuffer == rid.renderInstanceBuffer && rid.renderInstanceNum == renderInstanceNum)
            {
                CARB_LOG_ERROR("RenderInstanceCache::getAndRemove() - There should never be another entry with the same instance num!");
            }
#endif

            if (renderInstanceBuffer == rid.renderInstanceBuffer && rid.renderInstanceNum > renderInstanceNum)
            {
                rid.renderInstanceNum--;
            }
        }

        return true;
    }

    return false;
}

bool RenderInstanceCache::exists(const pxr::SdfPath& path, const GfMatrix4f& matrix)
{
    auto it = mRenderInstanceDescMap.find(path);
    if (it != mRenderInstanceDescMap.end())
    {
        return it->second.matrix == matrix;
    }

    return false;
}

void RenderInstanceCache::addOrUpdate(const pxr::SdfPath& path,
                                      omni::renderer::RenderInstanceBuffer renderInstanceBuffer,
                                      size_t renderInstanceNum,
                                      TriangleMeshBuffer* triangleMeshBuffer,
                                      const GfMatrix4f& matrix)
{
    auto it = mRenderInstanceDescMap.find(path);
    if (it != mRenderInstanceDescMap.end())
    {
        RenderInstanceDesc& rid = it->second;
        rid.renderInstanceBuffer = renderInstanceBuffer;
        rid.renderInstanceNum = renderInstanceNum;
        rid.triangleMeshBuffer = triangleMeshBuffer;
        rid.matrix = matrix;
    }
    else
    {
        mRenderInstanceDescMap[path] = { renderInstanceBuffer, renderInstanceNum, triangleMeshBuffer, matrix };
    }
}

DebugVisualization::DebugVisualization()
    : mColliderVisualization(VisualizerMode::eNone),
      mSceneIsPlaying(false),
      mSceneLoaded(true),
      mSceneDirty(true),
      mXformDirty(false),
      mCameraMovement(false),
      mOptimizeRedraw(true),
      mAABBDebugVisBuffer{ 0 },
      mShapeDebugRenderInstanceBuffer(IDebugDraw::eInvalidBuffer),
      mLastLineBufferSize(2048),
      mShapeDebugIndex(0),
      mShapeDebugLineEmpty(true),
      mVehicleDebugVis(mXformCache),
      mCurrentCameraPos(0.0f), mPrevCameraPos(0.0f),
      mSimplifyDebugVisAtDistanceSq(std::numeric_limits<float>::max())
{
    mCollisionMeshDebugVis = DebugVisualizationCollisionMesh::create(mXformCache);
    gPhysXVisualization = carb::getCachedInterface<omni::physx::IPhysxVisualization>();

    gDebugDraw = carb::getCachedInterface<omni::renderer::IDebugDraw>();

    mLastUSDNoticeTime.start();
    mLastUSDChangeTime.start();
    mLastCameraUpdateTime.start();
    mLastXformNotificationTime.start();
    mLastXformChangeTime.start();
}

DebugVisualization::~DebugVisualization()
{
    releaseShapeDebugLineList();
    mAABBRenderInstanceCache.release();
    mTrimeshInstanceCache.release();
    releaseTriangleMeshLineLists();
    if ( mCollisionMeshDebugVis )
    {
        mCollisionMeshDebugVis->release();
    }
    gPhysXVisualization = nullptr;
    gDebugDraw = nullptr;
    releaseFabric();
}

void DebugVisualization::createShapeDebugLineList()
{
    if (mShapeDebugLineBuffer == IDebugDraw::eInvalidBuffer)
    {
        mShapeDebugLineBuffer = gDebugDraw->allocateLineBuffer(std::max<size_t>(32768, mLastLineBufferSize));
        mShapeDebugRenderInstanceBuffer = gDebugDraw->allocateRenderInstanceBuffer(mShapeDebugLineBuffer, 1);
        float transform[16] = {};
        transform[0] = 1.f;
        transform[1 + 4] = 1.f;
        transform[2 + 8] = 1.f;
        transform[3 + 12] = 1.f;

        gDebugDraw->setRenderInstance(mShapeDebugRenderInstanceBuffer, 0, &transform[0], 0, 0);
        mShapeDebugIndex = 0;
    }
}

void DebugVisualization::releaseShapeDebugLineList()
{
    if (mShapeDebugLineBuffer != IDebugDraw::eInvalidBuffer)
    {
        gDebugDraw->deallocateLineBuffer(mShapeDebugLineBuffer);
        gDebugDraw->deallocateRenderInstanceBuffer(mShapeDebugRenderInstanceBuffer);
        mShapeDebugLineBuffer = IDebugDraw::eInvalidBuffer;
        mShapeDebugRenderInstanceBuffer = IDebugDraw::eInvalidBuffer;
        mShapeDebugIndex = 0;
        mShapeDebugLineEmpty = true;
    }
}

void DebugVisualization::createAABBVisBuffer()
{
    if (mAABBDebugVisBuffer.renderInstanceBuffer == IDebugDraw::eInvalidBuffer)
    {
        uint32_t numLines = 0;
        size_t counter = 0;

        // color does not matter will be overriden later via setRenderInstance()
        const DebugLine* lines = getUnitCornerAABBDebugDraw(0, numLines);

        mAABBDebugVisBuffer.lineBuffer = gDebugDraw->allocateLineBuffer(numLines);
        mAABBDebugVisBuffer.renderInstanceBuffer =
            gDebugDraw->allocateRenderInstanceBuffer(mAABBDebugVisBuffer.lineBuffer, 0x4000);
        mAABBDebugVisBuffer.currentRenderInstance = 0;

        for (uint32_t i = 0; i < numLines; i++)
        {
            const DebugLine& line = lines[i];
            drawLine(line, mAABBDebugVisBuffer.lineBuffer, counter);
        }

        delete[] lines;
    }
}

void DebugVisualization::releaseAABBVisBuffer()
{
    if (mAABBDebugVisBuffer.lineBuffer != 0)
    {
        gDebugDraw->deallocateLineBuffer(mAABBDebugVisBuffer.lineBuffer);
        mAABBDebugVisBuffer.lineBuffer = 0;
    }

    if (mAABBDebugVisBuffer.renderInstanceBuffer != 0)
    {
        gDebugDraw->deallocateRenderInstanceBuffer(mAABBDebugVisBuffer.renderInstanceBuffer);
        mAABBDebugVisBuffer.renderInstanceBuffer = 0;
    }

    mAABBDebugVisBuffer.currentRenderInstance = 0;
}

void DebugVisualization::setSimplifyDebugVisDistance(float distance)
{
    mSimplifyDebugVisAtDistanceSq = distance * distance;
    updateDebugVisualization();
}

void DebugVisualization::drawLine(const DebugLine& line, omni::renderer::SimplexBuffer lineId, size_t& counter)
{
    // NOTE this method completely ignores the color as we are overriding it later on
    gDebugDraw->setLine(lineId, counter++, line.mPos0, 0, kLineWidth, line.mPos1, 0, kLineWidth);
}

void DebugVisualization::drawLineCol(const DebugLine& line, omni::renderer::SimplexBuffer lineId, size_t& counter)
{
    gDebugDraw->setLine(lineId, counter++, line.mPos0, convertColor(line.mColor0), kLineWidth, line.mPos1,
                        convertColor(line.mColor1), kLineWidth);
}

void DebugVisualization::drawPoint(const DebugPoint& point, float metersPerUnit)
{
    gDebugDraw->drawPoint(point.mPos, convertColor(point.mColor), kLineWidth);
}

void DebugVisualization::releaseTriangleMeshLineLists()
{
    for (TriangleMeshBufferMap::const_reference& ref : mTriangleMeshBuffers)
    {
        gDebugDraw->deallocateLineBuffer(ref.second.lineBuffer);
        gDebugDraw->deallocateRenderInstanceBuffer(ref.second.renderInstanceBuffer);
    }

    mTriangleMeshBuffers.clear();
}

bool DebugVisualization::HasPointInstancerParent(const pxr::UsdPrim& prim)
{
    UsdPrim parent = prim;

    while (parent != gStage->GetPseudoRoot())
    {
        if (parent.IsA<UsdGeomPointInstancer>())
        {
            PathSet::const_iterator it = mPointInstancerPathCache.find(parent.GetPrimPath());
            if (it != mPointInstancerPathCache.end())
            {
                return true;
            }
        }

        parent = parent.GetParent();
    }

    return false;
}

void DebugVisualization::enableNormalsVisualization(bool enable)
{
    gPhysXVisualization->enableNormalsVisualization(enable);
}

void DebugVisualization::setColliderVisualization(VisualizerMode mode)
{
    mColliderVisualization = mode;

    if (mode == VisualizerMode::eNone)
    {
        clearCaches();
    }
    else
    {
        mSceneDirty = true;
    }
}

void DebugVisualization::clearCaches()
{
    gPhysXVisualization->clearDebugVisualizationData();

    releaseShapeDebugLineList();
    mAABBRenderInstanceCache.release();
    mTrimeshInstanceCache.release();
    releaseTriangleMeshLineLists();
    mDescCache.release();
    mShapePathCache.clear();
    mPointInstancerPathCache.clear();
    mMergeMeshPathCache.clear();
    mTimeVaryingPrimMap.clear();
    releaseAABBVisBuffer();
}

void DebugVisualization::draw()
{
    if (!mSceneLoaded)
        return;

    pxr::UsdStageWeakPtr stage = omni::usd::UsdContext::getContext()->getStage();
    if (!stage)
    {
        return;
    }
    if (!gStage)
        gStage = stage;

    if (!mSceneIsPlaying && !mXformDirty)
    {
        // If USD changes are still coming, so kLastUSDNotificationDelay keeps
        // blocking updates, after kLastUSDChangeDelay we will update no matter what.
        float elapsedTime = mLastUSDChangeTime.getElapsedTime<float>(carb::extras::Timer::Scale::eSeconds);
        if (mOptimizeRedraw && elapsedTime < kLastUSDChangeDelay)
        {
            // Do not redraw if last received USD notification happened in less than kLastUSDNotificationDelay time.
            // This way we can absorb USD changes rather than refreshing each time when a change notification arrives.
            const float elapsedTime = mLastUSDNoticeTime.getElapsedTime<float>(carb::extras::Timer::Scale::eSeconds);
            if (elapsedTime < kLastUSDNotificationDelay)
            {
                return;
            }
        }
    }

    CARB_PROFILE_ZONE(0, "DebugVisualization::draw");
    const float metersPerUnit = float(pxr::UsdGeomGetStageMetersPerUnit(stage));

    float camElapsedTime = mLastCameraUpdateTime.getElapsedTime<float>(carb::extras::Timer::Scale::eSeconds);
    if (!mOptimizeRedraw || (mOptimizeRedraw && camElapsedTime >= kCameraUpdateDelay))
    {
        mLastCameraUpdateTime.start();

        if ((mPrevCameraPos - mCurrentCameraPos).GetLength() > kMinCameraMovement)
        {
            mCameraMovement = true;
            mPrevCameraPos = mCurrentCameraPos;
        }
    }

    if (isUsdBasedVisualizationEnabled())
    {
        if (mFabricEnabled && mQueryFabricWhileSim && isFabricDirty())
        {
            updateFabricSync();
            mXformDirty = true;
        }

        // Configure PhysX to show normals if we have the setting set to 'true'
        bool showNormals = gSettings->get<bool>(omni::physx::kSettingDisplayColliderNormals);
        enableNormalsVisualization(showNormals);

        updateUsdBasedDebugVisualization(metersPerUnit, stage);

        if (mColliderVisualization != VisualizerMode::eNone)
        {
            if (mShapeDebugLineEmpty)
            {
                releaseShapeDebugLineList();
            }
        }

        if (mVehicleDebugVis.getVisualizationFlags())
            mVehicleDebugVis.releaseDrawBuffersIfEmpty();
    }

    mLastUSDChangeTime.start();
}

void DebugVisualization::setSceneLoaded(bool loaded)
{
    mSceneLoaded = loaded;

    if (!loaded)
    {
        // on stage close, the render buffers get released automatically, thus it is
        // not safe to delete them here any longer.

        mShapeDebugLineBuffer = omni::renderer::IDebugDraw::eInvalidBuffer;
        mShapeDebugRenderInstanceBuffer = omni::renderer::IDebugDraw::eInvalidBuffer;
        mShapeDebugLineEmpty = true;

        clearCaches();
        mVehicleDebugVis.clear(true);
    }

    if (mFabricEnabled)
    {
        loaded ? initFabric() : releaseFabric();
    }
}

void DebugVisualization::onResyncedPrimPath(const pxr::SdfPath& path)
{
    mDescCache.releasePath(path);
    mCollisionMeshDebugVis->releaseDescCache(path);
    mSceneDirty = true;
    mLastUSDNoticeTime.start();
}

void DebugVisualization::onAttributeChange(const pxr::SdfPath& attrPath,
                                           const pxr::SdfPath& primPath,
                                           const pxr::UsdPrim& prim,
                                           bool resynced,
                                           bool isXform,
                                           bool needsDescCacheReset)
{
    if (isXform)
    {
        mLastXformNotificationTime.start();

        // early exit if dirty already
        if (mXformDirty)
            return;

        UsdPrim parent = prim;

        while (parent != gStage->GetPseudoRoot())
        {
            if (parent.IsA<UsdGeomPointInstancer>())
            {
                PathSet::const_iterator it = mPointInstancerPathCache.find(parent.GetPrimPath());
                if (it != mPointInstancerPathCache.end())
                {
                    mXformDirty = true;
                    if (needsDescCacheReset)
                    {
                        mDescCache.releasePath(primPath);
                        mCollisionMeshDebugVis->releaseDescCache(primPath);
                    }
                    return;
                }
            }
            else if (!mSceneIsPlaying && mMergeMeshPathCache.find(parent.GetPrimPath()) != mMergeMeshPathCache.end())
            {
                mSceneDirty = true; // TODO FIXME this forces all debug vis. to redraw! Optimize this to update only changed prims.
                mLastUSDNoticeTime.start();
                return;
            }
            parent = parent.GetParent();
        }

        const UsdPrimRange range(prim);
        for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
        {
            const SdfPath path = (*iter).GetPrimPath();

            {
                PathSet::const_iterator it = mShapePathCache.find(path);
                if (it != mShapePathCache.end())
                {
                    mXformDirty = true;
                    break;
                }
            }

            {
                PathSet::const_iterator it = mPointInstancerPathCache.find(path);
                if (it != mPointInstancerPathCache.end())
                {
                    mXformDirty = true;
                    break;
                }
            }

            {
                if ( mCollisionMeshDebugVis->isXformPathTracked(path) )
                {
                    mXformDirty = true;
                    break;
                }
            }

            {
                if (mVehicleDebugVis.isXformPathTracked(path))
                {
                    mXformDirty = true;
                    break;
                }
            }
        }

        if (mXformDirty && needsDescCacheReset)
        {
            mDescCache.releasePath(primPath);
            mCollisionMeshDebugVis->releaseDescCache(primPath);
        }
    }
    else
    {
        if(!mXformDirty)
        {
            const UsdPrimRange range(prim);
            for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
            {
                const SdfPath path = (*iter).GetPrimPath();
                // We need this to pick changes in approximation attribute for collision
                mCollisionMeshDebugVis->releaseDescCache(path);
            }
        }
        mSceneDirty = true; // TODO FIXME this forces all debug vis. to redraw! Optimize this to update only changed prims.
        mLastUSDNoticeTime.start();
    }
}

void DebugVisualization::updateUsdBasedDebugVisualization(float metersPerUnit,
                                                          pxr::UsdStageWeakPtr stage)
{
    CARB_PROFILE_ZONE(0, "DebugVisualization::updateUsdBasedDebugVisualization");

    mXformCache.Clear();
    mXformCache.SetTime(pxr::UsdTimeCode(gTimeline->getCurrentTime() * stage->GetTimeCodesPerSecond()));

    if (mSceneDirty)
    {
        gBlockNoticeHandle = true;

        if (mColliderVisualization != VisualizerMode::eNone)
        {
            releaseShapeDebugLineList();
            mAABBRenderInstanceCache.release();
            mTrimeshInstanceCache.release();
            releaseTriangleMeshLineLists();
            createShapeDebugLineList();
            mLastLineBufferSize = 0;

            mShapePathCache.clear();
            mPointInstancerPathCache.clear();
            mMergeMeshPathCache.clear();

            mDescCache.release();

            mTimeVaryingPrimMap.clear();
            releaseAABBVisBuffer();
        }

        if (mVehicleDebugVis.getVisualizationFlags())
        {
            mVehicleDebugVis.clear(false);
        }

        mXformDirty = false;
        gPhysXVisualization->clearDebugVisualizationData();

        const std::vector<std::string> selectedPrims =
            omni::usd::UsdContext::getContext()->getSelection()->getSelectedPrimPaths();

        if (mColliderVisualization == VisualizerMode::eAll)
        {
            if (mFabricEnabled && mQueryUsdrtForTraversal)
            {
                updateUsdBasedDebugVisualizationInGroupsFabric(metersPerUnit, stage);
            }
            else
            {
                pxr::UsdPrimRange range = stage->Traverse(UsdTraverseInstanceProxies());
                updateUsdBasedDebugVisualizationInRange(range, metersPerUnit, stage, false);
            }
        }
        else
        {
            if (mColliderVisualization == VisualizerMode::eSelected)
            {
                for (size_t i = 0; i < selectedPrims.size(); i++)
                {
                    const UsdPrim prim = stage->GetPrimAtPath(SdfPath(selectedPrims[i]));
                    const UsdPrimRange range(prim, UsdTraverseInstanceProxies());
                    updateUsdBasedDebugVisualizationInRange(range, metersPerUnit, stage, false);
                }
            }

            if (mVehicleDebugVis.getVisualizationFlags())
            {
                pxr::UsdPrimRange range = stage->Traverse(UsdTraverseInstanceProxies());
                updateUsdBasedDebugVisualizationInRange(range, metersPerUnit, stage, true);
            }
        }

        if (mCollisionMeshDebugVis->isEnabled())
        {
            CARB_PROFILE_ZONE(0, "DebugVisualization::collisionMeshDebugVisUpdate");
            mCollisionMeshDebugVis->updateDebugVisualization();
        }

        gBlockNoticeHandle = false;
        mSceneDirty = false;
    }
    else if (mXformDirty || hasNewCollisionResults(mCollisionMeshDebugVis) || mCollisionMeshDebugVis->wantsForcedUpdate() )
    {
        if (!mSceneIsPlaying)
        {
            // If mLastXformChangeTime elapses, we will bypass mLastXformNotificationTime and go ahead and update.
            // This is here to update debug visualization in case user has been moving an object for
            // kLastXformChangeDelay and longer.
            float elapsedTime = mLastXformChangeTime.getElapsedTime<float>(carb::extras::Timer::Scale::eSeconds);
            if (mOptimizeRedraw && elapsedTime < kLastXformChangeDelay)
            {
                // Do not update if last received xform change happened in less than kLastXformChangeDelay time.
                // This way we can absorb changes rather than updating each time when an xform change notification
                // arrives.
                elapsedTime = mLastXformNotificationTime.getElapsedTime<float>(carb::extras::Timer::Scale::eSeconds);
                if (elapsedTime < kLastXformNotificationDelay)
                {
                    return;
                }
            }
        }

        if (mColliderVisualization != VisualizerMode::eNone)
        {
            releaseShapeDebugLineList();
            createShapeDebugLineList();
            mLastLineBufferSize = 0;
            {
                CARB_PROFILE_ZONE(0, "DebugVisualization::pointInstancerDebugVisUpdate");
                for (const SdfPath& path : mPointInstancerPathCache)
                {
                    const UsdPrim prim = stage->GetPrimAtPath(path);
                    updatePointInstancerPrimDebugVisualization(prim, metersPerUnit, stage);
                }
            }

            {
                CARB_PROFILE_ZONE(0, "DebugVisualization::collisionDebugVisUpdate");
                for (const SdfPath& path : mShapePathCache)
                {
                    const UsdPrim prim = stage->GetPrimAtPath(path);
                    removePrimFromAABBCache(path);
                    updateCollisionPrimDebugVisualization(prim, metersPerUnit);
                }
            }
        }
        if ((mXformDirty || mCollisionMeshDebugVis->wantsForcedUpdate()) && mCollisionMeshDebugVis->isEnabled())
        {
            CARB_PROFILE_ZONE(0, "DebugVisualization::collisionMeshDebugVisUpdate");
            mCollisionMeshDebugVis->updateDebugVisualization();
        }

        if (mXformDirty && mVehicleDebugVis.getVisualizationFlags())  // not interested in the hasNewCollisionResults() case
        {
            mVehicleDebugVis.updateTrackedVehicles(stage);
        }

        mXformDirty = false;
        mLastXformChangeTime.start();
    }
    else if (mCameraMovement)
    {
        createShapeDebugLineList();

        if (mColliderVisualization == VisualizerMode::eAll)
        {
            if (mFabricEnabled && mQueryUsdrtForTraversal)
            {
                updateUsdBasedDebugVisualizationInGroupsFabric(metersPerUnit, stage);
            }
            else
            {
                pxr::UsdPrimRange range = stage->Traverse(UsdTraverseInstanceProxies());
                updateUsdBasedDebugVisualizationInRange(range, metersPerUnit, stage, false);
            }
        }
        else
        {
            if (mColliderVisualization == VisualizerMode::eSelected)
            {
                const std::vector<std::string> selectedPrims =
                    omni::usd::UsdContext::getContext()->getSelection()->getSelectedPrimPaths();

                for (size_t i = 0; i < selectedPrims.size(); i++)
                {
                    const UsdPrim prim = stage->GetPrimAtPath(SdfPath(selectedPrims[i]));
                    const UsdPrimRange range(prim, UsdTraverseInstanceProxies());
                    updateUsdBasedDebugVisualizationInRange(range, metersPerUnit, stage, false);
                }
            }
        }

        mCameraMovement = false;
    }
}

template <typename T>
void getAttributeArray(pxr::VtArray<T>& array, pxr::UsdAttribute& attribute)
{
    pxr::VtValue arrayDataValue;
    attribute.Get(&arrayDataValue);
    const size_t size = arrayDataValue.GetArraySize();
    array.resize(size);
    if (size)
    {
        const pxr::VtArray<T>& arrayData = arrayDataValue.Get<pxr::VtArray<T>>();
        array.assign(arrayData.begin(), arrayData.end());
    }
}

usdparser::PhysxShapeDesc* DebugVisualization::getShapeDesc(const pxr::UsdPrim& prim,
                                                            const pxr::UsdPrim& colPrim,
                                                            const uint64_t stageId)
{
    CARB_PROFILE_ZONE(0, "DebugVisualization::getShapeDesc");

    usdparser::PhysxShapeDesc* shapeDesc = (usdparser::PhysxShapeDesc*)mDescCache.getDesc(prim.GetPrimPath(), eShape);

    if (!shapeDesc)
    {
        CARB_PROFILE_ZONE(0, "DebugVisualization::shapeDescParse");

        static TfToken oldConvexPrim("ConvexMesh");
        if (!(prim.IsA<UsdGeomGprim>() || prim.HasAPI<PhysxSchemaPhysxMeshMergeCollisionAPI>() || prim.GetTypeName() == oldConvexPrim))
        {
            return nullptr;
        }

        shapeDesc = gUsdLoad->parseCollision(stageId, colPrim.GetPrimPath(), prim.GetPrimPath());
        if (!shapeDesc)
        {
            return nullptr;
        }

        mDescCache.addDesc(prim.GetPrimPath(), eShape, shapeDesc);
    }

    return shapeDesc;
}

void DebugVisualization::removePrimFromAABBCache(const pxr::SdfPath& primPath)
{
    CARB_PROFILE_ZONE(0, "DebugVisualization::removePrimFromAABBCache");

    // if an AABB debug vis. simplification exists, remove it
    omni::renderer::RenderInstanceBuffer renderInstanceBuffer;
    size_t renderInstanceNum;
    TriangleMeshBuffer* triangleMeshBuffer;
    GfMatrix4f matrix;
    if (mAABBRenderInstanceCache.getAndRemove(primPath, renderInstanceBuffer, renderInstanceNum, triangleMeshBuffer, matrix))
    {
        gDebugDraw->removeRenderInstance(renderInstanceBuffer, renderInstanceNum);

        if (triangleMeshBuffer)
        {
#if ENABLE_CHECKS
            if (triangleMeshBuffer->currentRenderInstance == 0)
            {
                CARB_LOG_ERROR("triangleMeshBuffer->currentRenderInstance out of sync!");
            }
#endif
            triangleMeshBuffer->currentRenderInstance--;
        }
    }
}

void DebugVisualization::removePrimFromTrimeshCache(const pxr::SdfPath& primPath)
{
    CARB_PROFILE_ZONE(0, "DebugVisualization::removePrimFromTrimeshCache");

    // remove any existing instance of the trimesh debug draw
    omni::renderer::RenderInstanceBuffer renderInstanceBuffer;
    size_t renderInstanceNum;
    TriangleMeshBuffer* triangleMeshBuffer;
    GfMatrix4f matrix;
    if (mTrimeshInstanceCache.getAndRemove(primPath, renderInstanceBuffer, renderInstanceNum, triangleMeshBuffer, matrix))
    {
        gDebugDraw->removeRenderInstance(renderInstanceBuffer, renderInstanceNum);

        if (triangleMeshBuffer)
        {

#if ENABLE_CHECKS
            if (triangleMeshBuffer->currentRenderInstance == 0)
            {
                CARB_LOG_ERROR("triangleMeshBuffer->currentRenderInstance out of sync!");
            }
#endif
            triangleMeshBuffer->currentRenderInstance--;
        }
    }
}

void DebugVisualization::updateCollisionPrimSimplifiedAABB(const pxr::SdfPath& primPath, const pxr::GfRange3d& primAABB)
{
    CARB_PROFILE_ZONE(0, "DebugVisualization::updateCollisionPrimSimplifiedAABB");

    GfTransform transform;
    transform.SetIdentity();
    transform.SetTranslation(primAABB.GetMin());
    transform.SetScale(primAABB.GetSize());
    GfMatrix4f matrix = GfMatrix4f(transform.GetMatrix());

    if (mAABBRenderInstanceCache.exists(primPath, matrix))
    {
        return;
    }

    // remove any existing instance of the trimesh debug draw
    removePrimFromTrimeshCache(primPath);

    createAABBVisBuffer(); // create the buffer if it does not exist

    omni::renderer::RenderInstanceBuffer renderInstanceBuffer;
    size_t renderInstanceNum;

    auto pDesc = mAABBRenderInstanceCache.get(primPath);

    if (pDesc)
    {
        // just update the matrix on the instance
        renderInstanceBuffer = pDesc->renderInstanceBuffer;
        renderInstanceNum = pDesc->renderInstanceNum;
        pDesc->matrix = matrix;
    }
    else
    {
        renderInstanceBuffer = mAABBDebugVisBuffer.renderInstanceBuffer;
        renderInstanceNum = mAABBDebugVisBuffer.currentRenderInstance;

        mAABBRenderInstanceCache.addOrUpdate(
            primPath, renderInstanceBuffer, renderInstanceNum, &mAABBDebugVisBuffer, matrix);

        mAABBDebugVisBuffer.currentRenderInstance++;
    }

    // dim the color
    uint32_t color = gPhysXVisualization->getDebugDrawCollShapeColor(primPath);
    {
        float h, s, v;
        ColorConvertRGBtoHSV(color, h, s, v);
        color = ColorConvertHSVtoRGB(h, s, v * simplifiedAABBColorLuminance, getAlpha(color));
    }

    gDebugDraw->setRenderInstance(renderInstanceBuffer,
                                  renderInstanceNum,
                                  matrix.data(),
                                  color,
                                  0);
}


void DebugVisualization::updateCollisionPrimTriangeMeshShape(const pxr::SdfPath& primPath,
                                                             const pxr::GfMatrix4d& primTransformMtx,
                                                             usdparser::PhysxShapeDesc* shapeDesc)
{
    CARB_PROFILE_ZONE(0, "DebugVisualization::updateCollisionPrimTriangeMeshShape");

    GfMatrix4d newTransformMtx = primTransformMtx;

    // shape we will go with. It can be the original shape or a unit shape
    const PhysxShapeDesc* targetShapeDesc = shapeDesc;

    // get mesh crc
    ShapeUniqueKey crc;
    crc.FromShapeDesc(targetShapeDesc);
    bool isMesh = crc.isMeshKey();

    // NOTE: if the shape is a non-mesh shape (box, sphere, ...), we need to get a unit sized shape!
    if (!isMesh) // Please note OM-120862: Capsule shape is an exception, it cannot be properly scaled => it is identified as a mesh.
    {
        const PhysxShapeDesc* unitShapeDesc = ShapeUtils::getUnitShapeDesc(targetShapeDesc);

        if (unitShapeDesc)
        {
            // get scale from shape desc
            GfVec3f scale;
            ShapeUtils::getShapeScale(targetShapeDesc, scale);

            targetShapeDesc = unitShapeDesc;

            GfTransform tf(newTransformMtx);
            GfMatrix4d scaleMatrix(1.0);
            scaleMatrix.SetScale(GfCompDiv(GfVec3d(scale), tf.GetScale()));
            newTransformMtx = scaleMatrix * newTransformMtx;

            // special case for bounding primitives, which may contain translation and rotation offset
            if (shapeDesc->type == eBoundingBoxShape)
            {
                BoundingBoxPhysxShapeDesc* boxDesc = (BoundingBoxPhysxShapeDesc*)shapeDesc;
                const carb::Float3& p = boxDesc->positionOffset;
                const carb::Float4& r = boxDesc->rotationOffset;

                const GfMatrix4d offsetMatrix(GfRotation(GfQuatd(r.w, r.x, r.y, r.z)), GfVec3d(p.x, p.y, p.z));
                scaleMatrix.SetScale(GfVec3d(scale));

                newTransformMtx = scaleMatrix * offsetMatrix * primTransformMtx.RemoveScaleShear();
            }
            else if (shapeDesc->type == eBoundingSphereShape)
            {
                BoundingSpherePhysxShapeDesc* sphereDesc = (BoundingSpherePhysxShapeDesc*)shapeDesc;
                const carb::Float3& p = sphereDesc->positionOffset;

                GfTransform transform(newTransformMtx);
                GfRotation rot = transform.GetRotation();
                GfVec3d pos = transform.GetTranslation();
                pos += rot.TransformDir(GfVec3d(p.x, p.y, p.z));
                transform.SetTranslation(pos);
                newTransformMtx.SetTranslateOnly(transform.GetTranslation());
            }
        }
    }

    GfMatrix4f matrix = GfMatrix4f(newTransformMtx);

    if (mTrimeshInstanceCache.exists(primPath, matrix))
    {
        return;
    }

    removePrimFromAABBCache(primPath);

    RenderInstanceCache::RenderInstanceDesc* pRenderInstanceDesc = mTrimeshInstanceCache.get(primPath);
    TriangleMeshBufferMap::iterator triangleMeshBufferIt = mTriangleMeshBuffers.find(crc);

    if (pRenderInstanceDesc && triangleMeshBufferIt == mTriangleMeshBuffers.end())
    {
        // This happens only when capsule shape changes, because capsule is treated as a mesh.
        // Mesh CRC changes each time capsule changes props (radius, height)!

        // remove any existing instance of the trimesh debug draw
        removePrimFromTrimeshCache(primPath);
        pRenderInstanceDesc = nullptr;
    }

    if (isMesh && shapeDesc == targetShapeDesc) // it is not a shape
    {
        shapeDesc->localPos = { 0.0f, 0.0f, 0.0f };
        shapeDesc->localRot = { 0.0f, 0.0f, 0.0f, 1.0f };

        // for meshes reset to unit scale as the scale is solely driven by matrix -
        // - we don't want it baked into the trimesh
        ShapeUtils::setMeshScale(shapeDesc, { 1.0f, 1.0f, 1.0f });
    }

    if (shapeDesc->type == eCapsuleShape || shapeDesc->type == eCylinderShape || shapeDesc->type == eConeShape)
    {
        // Reset current matrix scale (scale is already baked into capsule props).
        matrix = matrix.RemoveScaleShear();
    }

    if (pRenderInstanceDesc)
    {
        pRenderInstanceDesc->matrix = matrix;

        // just update the matrix on the instance
        gDebugDraw->setRenderInstance(pRenderInstanceDesc->renderInstanceBuffer, pRenderInstanceDesc->renderInstanceNum,
                                      matrix.data(), gPhysXVisualization->getDebugDrawCollShapeColor(primPath), 0);
    }
    else
    {
        // get existing or create new mesh buffer
        TriangleMeshBuffer* pTriangleMeshBuffer = nullptr;

        if (triangleMeshBufferIt != mTriangleMeshBuffers.end())
        {
            pTriangleMeshBuffer = &(triangleMeshBufferIt->second);
        }
        else
        {
            // create new trimesh buffer
            uint32_t numLines = 0;
            const DebugLine* lines = nullptr;

            lines = gPhysXVisualization->getShapeDebugDraw(primPath, targetShapeDesc, numLines);

            if (lines && numLines)
            {
                size_t counter = 0;

                pTriangleMeshBuffer = &(mTriangleMeshBuffers[crc]);
                pTriangleMeshBuffer->lineBuffer = gDebugDraw->allocateLineBuffer(size_t(numLines));
                pTriangleMeshBuffer->renderInstanceBuffer =
                    gDebugDraw->allocateRenderInstanceBuffer(pTriangleMeshBuffer->lineBuffer, 1);
                pTriangleMeshBuffer->currentRenderInstance = 0;

                for (uint32_t i = 0; i < numLines; i++)
                {
                    const DebugLine& line = lines[i];
                    drawLine(line, pTriangleMeshBuffer->lineBuffer, counter);
                }
            }
        }

        if (pTriangleMeshBuffer != nullptr)
        {
            mTrimeshInstanceCache.addOrUpdate(primPath, pTriangleMeshBuffer->renderInstanceBuffer,
                                              pTriangleMeshBuffer->currentRenderInstance, pTriangleMeshBuffer, matrix);

            gDebugDraw->setRenderInstance(pTriangleMeshBuffer->renderInstanceBuffer,
                                          pTriangleMeshBuffer->currentRenderInstance,
                                          matrix.data(), gPhysXVisualization->getDebugDrawCollShapeColor(primPath), 0);

            pTriangleMeshBuffer->currentRenderInstance++;
        }
    }
}

void DebugVisualization::updateCollisionPrimDebugVisualization(const pxr::UsdPrim& colPrim, float metersPerUnit)
{
    CARB_PROFILE_ZONE(0, "DebugVisualization::updateCollisionPrimDebugVisualization %s", colPrim.GetPath().GetText());

    const uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(colPrim.GetStage()).ToLongInt();

    pxr::UsdPrimRange range(colPrim, UsdTraverseInstanceProxies());
    for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
    {
        UsdPrim prim = *iter;

        if (!prim)
            continue;

        usdparser::PhysxShapeDesc* shapeDesc = getShapeDesc(prim, colPrim, stageId);

        if (!shapeDesc || !shapeDesc->collisionEnabled)
        {
            continue;
        }

        if ((shapeDesc->type >= ObjectType::eConvexMeshShape && shapeDesc->type <= ObjectType::eTriangleMeshShape) ||
            shapeDesc->type == ObjectType::eSpherePointsShape)
        {
            const MergeMeshPhysxShapeDesc* meshMergeShape = reinterpret_cast<const MergeMeshPhysxShapeDesc*>(shapeDesc);
            if (meshMergeShape->mergedMesh)
            {
                const PhysxSchemaPhysxMeshMergeCollisionAPI mergeAPI(prim);
                if (mergeAPI)
                {
                    const UsdCollectionAPI collection = mergeAPI.GetCollisionMeshesCollectionAPI();
                    const UsdCollectionMembershipQuery query = collection.ComputeMembershipQuery();
                    const SdfPathSet collectionPaths =
                        UsdCollectionAPI::ComputeIncludedPaths(query, prim.GetStage(), UsdTraverseInstanceProxies());
                    mMergeMeshPathCache = collectionPaths;
                }
                mMergeMeshPathCache.insert(prim.GetPrimPath());
            }
        }

        // use AABB midpoint for the pos of the prim
        pxr::GfRange3d primAABB = omni::usd::UsdContext::getContext()->computePrimWorldBoundingBox(prim);

        // prim world pos
        GfVec3f pos(0.0f);

        if (primAABB.IsEmpty())
        {
            // we don't have AABB, get world pos of the prim
            const GfTransform transform = GfTransform(GetLocalToWorldTransform(prim));
            pos = GfVec3f(transform.GetTranslation());
        }
        else
        {
            pos = GfVec3f(primAABB.GetMidpoint());
        }

        const GfVec3f deltaVec = mCurrentCameraPos - pos;
        float distFromCamSq = deltaVec.GetLengthSq();

        bool debugVisTypeOverrideToAABB = false;

        // For more complex debug visualizations, fall back to AABB
        // if camera distance from the prim is greater than simplifyDebugVisAtDistanceSq setting
        if (distFromCamSq >= mSimplifyDebugVisAtDistanceSq)
        {
            ObjectType debugVistype = shapeDesc->type;

            // do not simplify simple box shapes
            if (debugVistype != eBoxShape && debugVistype != eBoundingBoxShape)
            {
                debugVisTypeOverrideToAABB = true;
            }
         }

        pxr::SdfPath primPath = prim.GetPrimPath();

        if (debugVisTypeOverrideToAABB)
        {
            updateCollisionPrimSimplifiedAABB(primPath, primAABB);
        }
        else
        {
            GfMatrix4d matrix = GetLocalToWorldTransform(prim);
            updateCollisionPrimTriangeMeshShape(primPath, matrix, shapeDesc);
        }
    }
}

void DebugVisualization::parsePrototype(const UsdStageRefPtr stage,
                                        const UsdPrim& usdPrim,
                                        TargetCollisionList& collisionList)
{
    bool resetXformStack = false;
    const GfMatrix4d protoMatrix = mXformCache.GetLocalTransformation(usdPrim, &resetXformStack);
    const uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();
    const UsdPrimRange range(usdPrim);

    for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
    {
        const pxr::UsdPrim& prim = *iter;
        if (!prim)
            continue;

        if (prim.HasAPI<UsdPhysicsCollisionAPI>()) // check if its a standalone static body
        {
            // handle collision shapes
            usdparser::PhysxShapeDesc* shapeDesc =
                (usdparser::PhysxShapeDesc*)mDescCache.getDesc(prim.GetPrimPath(), eShape);
            if (!shapeDesc)
            {
                shapeDesc = gUsdLoad->parseCollision(stageId, prim.GetPrimPath(), prim.GetPrimPath());
                mDescCache.addDesc(prim.GetPrimPath(), eShape, shapeDesc);
            }

            // if it is still null don't bother adding it to the collision list
            if (!shapeDesc)
            {
                continue;
            }

            GfMatrix4d localColMatrix = mXformCache.ComputeRelativeTransform(prim, usdPrim, &resetXformStack);

            localColMatrix = localColMatrix.RemoveScaleShear() * protoMatrix;

            collisionList.push_back(PrototypeCollDesc(prim.GetPrimPath(), localColMatrix, shapeDesc));
        }
    }
}

void DebugVisualization::updatePointInstancerPrimDebugVisualization(const pxr::UsdPrim& prim,
                                                                    float metersPerUnit,
                                                                    pxr::UsdStageWeakPtr stage)
{
    CARB_PROFILE_ZONE(0, "DebugVisualization::updatePointInstancerPrimDebugVisualization");

    const UsdGeomPointInstancer& pointInstancer = UsdGeomPointInstancer(prim);

    UsdRelationship prototypes = pointInstancer.GetPrototypesRel();

    const GfMatrix4d instancerMatrix = GetLocalToWorldTransform(prim);

    SdfPathVector targets;
    prototypes.GetTargets(&targets);
    UsdAttributeVector attributes = prim.GetAttributes();
    std::vector<TargetCollisionList> targetLists;
    targetLists.resize(targets.size());

    VtArray<int> indices;
    VtArray<GfVec3f> positions;
    VtArray<GfQuath> orientations;
    VtArray<GfVec3f> scales;
    static const TfToken posToken("positions");
    static const TfToken rotToken("orientations");
    static const TfToken scaleToken("scales");
    static const TfToken indicesToken("protoIndices");

    for (UsdAttribute attr : attributes)
    {
        const TfToken usdPropName = attr.GetName();

        if (usdPropName == posToken)
        {
            getAttributeArray(positions, attr);
        }
        else if (usdPropName == rotToken)
        {
            getAttributeArray(orientations, attr);
        }
        else if (usdPropName == scaleToken)
        {
            getAttributeArray(scales, attr);
        }
        else if (usdPropName == indicesToken)
        {
            getAttributeArray(indices, attr);
        }
    }

    for (size_t i = 0; i < targets.size(); i++)
    {
        parsePrototype(stage, stage->GetPrimAtPath(targets[i]), targetLists[i]);
    }

    for (size_t i = 0; i < indices.size(); i++)
    {
        if (indices[i] >= static_cast<int>(targets.size()))
            continue;

        const GfVec3f instancePos = i < positions.size() ? positions[i] : GfVec3f(0.0f);
        const GfQuatd instanceOrient = i < orientations.size() ? GfQuatd(orientations[i]) : GfQuatd(1.0);
        const GfVec3f instanceScale = i < scales.size() ? scales[i] : GfVec3f(1.0f);

        GfMatrix4d scaleMatrix(1.0);
        scaleMatrix.SetScale(instanceScale);

        GfMatrix4d instanceMatrix(1.0);
        instanceMatrix.SetTranslate(GfVec3d(instancePos));
        instanceMatrix.SetRotateOnly(instanceOrient);
        instanceMatrix = scaleMatrix * instanceMatrix;

        const TargetCollisionList& collisionList = targetLists[indices[i]];

        for (size_t j = 0; j < collisionList.size(); j++)
        {
            const PrototypeCollDesc& collDesc = collisionList[j];
            const GfTransform matrix = GfTransform(collDesc.matrix * instanceMatrix * instancerMatrix);
            const GfVec3d pos = matrix.GetTranslation();
            const GfQuatd rot = matrix.GetRotation().GetQuat();
            const GfVec3d scl = matrix.GetScale();

            collDesc.shapeDesc->localPos = { float(pos[0]), float(pos[1]), float(pos[2]) };
            collDesc.shapeDesc->localRot = { float(rot.GetImaginary()[0]), float(rot.GetImaginary()[1]), float(rot.GetImaginary()[2]), float(rot.GetReal()) };
            collDesc.shapeDesc->localScale = { float(scl[0]), float(scl[1]), float(scl[2]) };

            if (collDesc.shapeDesc->type == ObjectType::eConvexMeshShape)
            {
                ConvexMeshPhysxShapeDesc* convexDesc = (ConvexMeshPhysxShapeDesc*)collDesc.shapeDesc;
                convexDesc->meshScale = { float(scl[0]), float(scl[1]), float(scl[2]) };
            }

            uint32_t numLines = 0;
            const DebugLine* lines = gPhysXVisualization->getShapeDebugDraw(collDesc.primPath, collDesc.shapeDesc, numLines);
            for (uint32_t l = 0; l < numLines; l++)
            {
                const DebugLine& line = lines[l];
                mShapeDebugLineEmpty = false;
                drawLineCol(line, mShapeDebugLineBuffer, mShapeDebugIndex);
            }
        }
    }
}

bool DebugVisualization::isTimeVaryingPrim(UsdStageWeakPtr stage, const UsdPrim& prim)
{
    CARB_PROFILE_ZONE(0, "DebugVisualization::isTimeVaryingPrim");

    bool isTimeVarying = false;
    UsdPrim parent = prim;
    while (!isTimeVarying && parent != stage->GetPseudoRoot())
    {
        const pxr::SdfPath parentPrimPath = parent.GetPath();

        auto it = mTimeVaryingPrimMap.find(parentPrimPath);
        if (it != mTimeVaryingPrimMap.end())
        {
            return it->second;
        }

        UsdGeomXformable xform(parent);
        bool resetsXformStack;
        for (auto xformOp : xform.GetOrderedXformOps(&resetsXformStack))
        {
            if (xformOp.GetNumTimeSamples() > 1)
            {
                isTimeVarying = true;
                break;
            }
        }

        mTimeVaryingPrimMap[parentPrimPath] = isTimeVarying;

        parent = parent.GetParent();
    }

    return isTimeVarying;
};

void DebugVisualization::updateUsdBasedDebugVisualizationInGroupsFabric(float metersPerUnit, pxr::UsdStageWeakPtr stage)
{
    CARB_PROFILE_ZONE(0, "DebugVisualization::updateUsdBasedDebugVisualizationInGroupsFabric");

    auto forEach = [&stage](const std::vector<usdrt::SdfPath>& vec, auto fn) {
        for (const usdrt::SdfPath& sdfPath : vec)
        {
            const omni::fabric::PathC pathC(sdfPath);
            const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
            const pxr::UsdPrim& prim = stage->GetPrimAtPath(usdPath);
            if (!prim)
                continue;
            fn(prim);
        }
    };

    static const usdrt::TfToken tokenPhysicsCollisionAPI("PhysicsCollisionAPI");
    static const usdrt::TfToken tokenPhysxVehicleAPI("PhysxVehicleAPI");
    static const usdrt::TfToken tokenPointInstancer("PointInstancer");

    auto vecCollision = mUsdrtStage->GetPrimsWithAppliedAPIName(tokenPhysicsCollisionAPI);
    auto vecVehicle = mUsdrtStage->GetPrimsWithAppliedAPIName(tokenPhysxVehicleAPI);
    auto vecPointInst = mUsdrtStage->GetPrimsWithTypeName(tokenPointInstancer);

    {
        CARB_PROFILE_ZONE(0, "DebugVisualization::updateUsdBasedDebugVisualizationInGroupsFabric::updatePointInstancerPrimDebugVisualization");
        forEach(vecPointInst, [this, stage, metersPerUnit](const pxr::UsdPrim& prim) {
            mPointInstancerPathCache.insert(prim.GetPrimPath());
            updatePointInstancerPrimDebugVisualization(prim, metersPerUnit, stage);
        });
    }
    {
        CARB_PROFILE_ZONE(0, "DebugVisualization::updateUsdBasedDebugVisualizationInGroupsFabric::updateCollisionPrimDebugVisualization");
        forEach(vecCollision, [this, stage, metersPerUnit](const pxr::UsdPrim& prim) {
            if (!HasPointInstancerParent(prim))
            {
                mShapePathCache.insert(prim.GetPrimPath());
                updateCollisionPrimDebugVisualization(prim, metersPerUnit);
            }
        });
    }
    {
        CARB_PROFILE_ZONE(0, "DebugVisualization::updateUsdBasedDebugVisualizationInGroupsFabric::mVehicleDebugVisUpdate");
        forEach(vecVehicle, [this, stage, metersPerUnit](const pxr::UsdPrim& prim) {
            if (mVehicleDebugVis.getVisualizationFlags())
            {
                mVehicleDebugVis.updateVehicle(prim, stage);
            }
        });
    }
}

void DebugVisualization::updateUsdBasedDebugVisualizationInRange(const pxr::UsdPrimRange range,
                                                                 float metersPerUnit,
                                                                 pxr::UsdStageWeakPtr stage,
                                                                 bool ignorePrimsWithSelectionSupport)
{
    CARB_PROFILE_ZONE(0, "DebugVisualization::updateUsdBasedDebugVisualizationInRange");

    for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
    {
        const pxr::UsdPrim& prim = *iter;
        if (!prim)
            continue;

        bool foundMatch;
        if ((mColliderVisualization != VisualizerMode::eNone) && (!ignorePrimsWithSelectionSupport))
        {
            foundMatch = true;

            if (prim.IsA<UsdGeomPointInstancer>())
            {
                mPointInstancerPathCache.insert(prim.GetPrimPath());
                updatePointInstancerPrimDebugVisualization(prim, metersPerUnit, stage);

                iter.PruneChildren();
            }
            else if (prim.HasAPI<UsdPhysicsCollisionAPI>())
            {
                mShapePathCache.insert(prim.GetPrimPath());
                updateCollisionPrimDebugVisualization(prim, metersPerUnit);
                iter.PruneChildren();
            }
            else
            {
                foundMatch = false;
            }
        }
        else
        {
            foundMatch = false;
        }

        if (mVehicleDebugVis.getVisualizationFlags() && (!foundMatch) && prim.HasAPI<PhysxSchemaPhysxVehicleAPI>())
        {
            // note: so far all the matching prim types exclude the vehicle, thus there is no
            //       need to test if a match was found

            mVehicleDebugVis.updateVehicle(prim, stage);
            // note: not pruning children as a vehicle might have collision shapes as children
        }
    }
}

void DebugVisualization::selectionChanged()
{
    mSceneDirty = mSceneDirty || (mColliderVisualization == VisualizerMode::eSelected);
    if ( mCollisionMeshDebugVis->refreshSelectionSet() )
    {
        mSceneDirty = true; // TODO FIXME - no need to release all caches on selection change!
    }
}

void DebugVisualization::initFabric()
{
    CARB_PROFILE_ZONE(0, "DebugVisualization::initFabric");

    static const omni::fabric::Token tokenFabricWorldMatrix(omni::physx::gWorldMatrixTokenString);

    releaseFabric();

    mIFabricStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    mIChangeTrackerConfig = carb::getCachedInterface<omni::fabric::IChangeTrackerConfig>();

    if (!mIFabricStageReaderWriter || !mIChangeTrackerConfig)
    {
        return;
    }

    pxr::UsdStageWeakPtr stage = omni::usd::UsdContext::getContext()->getStage();

    if (stage)
    {
        const uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

        omni::fabric::StageReaderWriter srw = mIFabricStageReaderWriter->get(stageId);

        mUsdrtStage = usdrt::UsdStage::Attach(stageId, srw.getId());

        // listen for pos/rot/scale changes
        mFabricListener = mIChangeTrackerConfig->createListener();
        srw.attributeEnableChangeTracking(tokenFabricWorldMatrix, mFabricListener);

        mFabricSync = omni::core::createType<usdrt::xformcache::IXformCache>();
        if (mFabricSync)
        {
            CARB_PROFILE_ZONE(0, "DebugVisualization::initFabric attachToStage");
            mFabricSync->attachToStage(stageId);
        }
    }
}

void DebugVisualization::releaseFabric()
{
    pxr::UsdStageWeakPtr stage = omni::usd::UsdContext::getContext()->getStage();

    if (stage)
    {
        const uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

        if (mIFabricStageReaderWriter)
        {
            omni::fabric::StageReaderWriter srw = mIFabricStageReaderWriter->get(stageId);
            srw.detachListener(mFabricListener);
        }
    }

    mUsdrtStage = nullptr;
    mFabricSync = nullptr;
    mIFabricStageReaderWriter = nullptr;
    mIChangeTrackerConfig = nullptr;
}

pxr::GfMatrix4d DebugVisualization::GetLocalToWorldTransform(const pxr::UsdPrim& prim)
{
    CARB_PROFILE_ZONE(0, "DebugVisualization::GetLocalToWorldTransform");
    if (mFabricEnabled && mQueryFabricWhileSim && mSceneIsPlaying && mFabricSync)
    {
        const pxr::SdfPath& primPath = prim.GetPrimPath();
        const usdrt::GfMatrix4d matrix = mFabricSync->computeWorldXform(omni::fabric::asInt(primPath));
        pxr::GfMatrix4d convertedMat;
        memcpy(&convertedMat, &matrix, sizeof(pxr::GfMatrix4d));
        return convertedMat;
    }

    return mXformCache.GetLocalToWorldTransform(prim);
}

bool DebugVisualization::isFabricDirty()
{
    CARB_PROFILE_ZONE(0, "DebugVisualization::isFabricDirty");

    if (!mSceneIsPlaying || !mFabricEnabled || !mFabricSync || !mIFabricStageReaderWriter)
    {
        return false;
    }

    omni::fabric::StageReaderWriter srw = mIFabricStageReaderWriter->get(mFabricSync->getStageId());
    omni::fabric::ChangedPrimBucketList changesByType = srw.getChanges(mFabricListener);
    size_t bucketCount = changesByType.bucketCount();
    srw.popChanges(mFabricListener);

    return bucketCount > 0;
}

void DebugVisualization::updateFabricSync()
{
    if (!mFabricEnabled || !mFabricSync)
    {
        return;
    }

    mFabricSync->syncXforms();
}

bool DebugVisualization::testInternalState(int phase) const
{
    // initial camera position.
    auto phase0Test = [this]() {
        int occurences[3] = { 0 };

        for (const auto& entry : mTrimeshInstanceCache.mRenderInstanceDescMap)
        {
            const std::string& key = entry.first.GetAsString();
            const RenderInstanceCache::RenderInstanceDesc& rid = entry.second;
            if (key.size() >= 4)
            {
                if (key.substr(key.size() - 3) == "_01")
                {
                    occurences[1]++;
                }
                else if (key.substr(key.size() - 3) == "_02")
                {
                    occurences[2]++;
                }
                else
                {
                    occurences[0]++;
                }
            }
        }

        bool r = occurences[0] == 12 && occurences[1] == 11 && occurences[2] == 2 &&
                 mAABBRenderInstanceCache.mRenderInstanceDescMap.size() == 0 &&
                 mAABBDebugVisBuffer.currentRenderInstance == 0 &&
                 mTrimeshInstanceCache.mRenderInstanceDescMap.size() == 25 && mTriangleMeshBuffers.size() == 21;
        if (!r)
        {
            CARB_LOG_ERROR(
                "testInternalState(0): occurences: [%d, %d, %d] ([12, 11, 2]), mAABBRenderInstanceCache.mRenderInstanceDescMap.size() == %zd (0), mAABBDebugVisBuffer.currentRenderInstance == %zd (0), mTrimeshInstanceCache.mRenderInstanceDescMap.size() == %zd (25), mTriangleMeshBuffers.size() == %zd (21)",
                occurences[0], occurences[1], occurences[2], mAABBRenderInstanceCache.mRenderInstanceDescMap.size(),
                mAABBDebugVisBuffer.currentRenderInstance, mTrimeshInstanceCache.mRenderInstanceDescMap.size(),
                mTriangleMeshBuffers.size());
        }

        return r;
    };

    // Far camera position with completely boxed debug visualization with either AABB or box/bounding box.
    // (box/bounding box shapes are never replaced with AABB visualizer just because it's more efficient)
    auto phase1Test = [this]() {
        return mAABBRenderInstanceCache.mRenderInstanceDescMap.size() == 22 &&
               // must be equal as all AABB shapes share a single render instance buffer:
               mAABBDebugVisBuffer.currentRenderInstance == mAABBRenderInstanceCache.mRenderInstanceDescMap.size() &&
               // box/bounding box shapes are never replaced with AABB visualizer just because it's more efficient:
               mTrimeshInstanceCache.mRenderInstanceDescMap.size() == 3;
    };

    // Midpoint where some objects are replaced with AABB and some are not.
    auto phase2Test = [this]() {
        return mAABBRenderInstanceCache.mRenderInstanceDescMap.size() == 13 &&
               // must be equal as all AABB shapes share a single render instance buffer:
               mAABBDebugVisBuffer.currentRenderInstance == mAABBRenderInstanceCache.mRenderInstanceDescMap.size() &&
               mTrimeshInstanceCache.mRenderInstanceDescMap.size() == 12;
    };

    switch (phase)
    {
    case 0: return phase0Test();
    case 1: return phase1Test();
    case 2: return phase2Test();
    }

    return false;
}

} // namespace ui
} // namespace physx
} // namespace omni
