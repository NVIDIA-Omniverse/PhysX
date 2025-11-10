// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "DescCache.h"
#include "UsdPCH.h"
#include "DebugVisualizationVehicle.h"
#include "DebugVisualizationCollisionMesh.h"
#include "DebugVisualizationHelpers.h"

#include <carb/extras/Timer.h>
#include <omni/renderer/IDebugDraw.h>
#include <omni/physx/IPhysxVisualization.h>
#include <private/omni/physx/ui/VisualizerMode.h>

// Fabric
#include <omni/core/ITypeFactory.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/fabric/IFabric.h>
#include <usdrt/scenegraph/usd/rt/xformable.h>
#include <usdrt/scenegraph/usd/usd/stage.h>
#include <usdrt/xformcache/ISharedXformCache.h>
#include <usdrt/xformcache/IXformCache.h>

namespace omni
{
namespace physx
{
namespace ui
{

class ShapeUniqueKey
{
public:
    void FromShapeDesc(const usdparser::PhysxShapeDesc* desc, bool includeMeshScale = false);

    inline bool isMeshKey() const
    {
        return mObjType == usdparser::eTriangleMeshShape || mObjType == usdparser::eConvexMeshShape ||
               mObjType == usdparser::eConvexMeshDecompositionShape || mObjType == usdparser::eSpherePointsShape ||
               mObjType == usdparser::eCustomShape || mObjType == usdparser::eCapsuleShape || // OM-120862: Capsule is
                                                                                              // an exception, it cannot
                                                                                              // be properly scaled
               mObjType == usdparser::eCylinderShape || mObjType == usdparser::eConeShape; // Cylinder/cone with margin
                                                                                           // can't be scaled too
    }

    inline bool operator==(const ShapeUniqueKey& rhs) const
    {
        return mObjType == rhs.mObjType && mMeshKey == rhs.mMeshKey;
    }

    struct Hash
    {
        inline size_t operator()(const ShapeUniqueKey& obj) const
        {
            return pxr::TfHash::Combine(obj.mObjType, static_cast<size_t>(obj.mMeshKey.getHashIndex()));
        }
    };

    static usdparser::MeshKey EmptyMeshKey;

private:
    usdparser::ObjectType mObjType;
    usdparser::MeshKey mMeshKey;
};

class ShapeUtils
{
public:
    static const usdparser::PhysxShapeDesc* getUnitShapeDesc(const usdparser::PhysxShapeDesc* shapeDesc,
                                                             usdparser::Axis axis = usdparser::eX);

    static void getShapeScale(const usdparser::PhysxShapeDesc* desc, pxr::GfVec3f& scale);
    static void setMeshScale(usdparser::PhysxShapeDesc* desc, const carb::Float3& scale);
};

struct PrototypeCollDesc
{
    PrototypeCollDesc(const pxr::SdfPath& primPath,
                      const pxr::GfMatrix4d& matrix = pxr::GfMatrix4d(1.0),
                      usdparser::PhysxShapeDesc* shapeDesc = nullptr)
        : primPath(primPath), matrix(matrix), shapeDesc(shapeDesc)
    {
    }

    const pxr::SdfPath primPath;
    const pxr::GfMatrix4d matrix;
    usdparser::PhysxShapeDesc* shapeDesc;
};

typedef std::vector<PrototypeCollDesc> TargetCollisionList;

typedef pxr::TfHashSet<pxr::SdfPath, pxr::SdfPath::Hash> PathSet;

struct TriangleMeshBuffer
{
    omni::renderer::SimplexBuffer lineBuffer;
    omni::renderer::RenderInstanceBuffer renderInstanceBuffer;
    size_t currentRenderInstance;
};

typedef pxr::TfHashMap<ShapeUniqueKey, TriangleMeshBuffer, ShapeUniqueKey::Hash> TriangleMeshBufferMap;

class RenderInstanceCache
{
    friend class DebugVisualization; // just for testInternalState()

public:
    struct RenderInstanceDesc
    {
        omni::renderer::RenderInstanceBuffer renderInstanceBuffer;
        size_t renderInstanceNum;
        TriangleMeshBuffer* triangleMeshBuffer;
        pxr::GfMatrix4f matrix;
    };

    typedef pxr::TfHashMap<pxr::SdfPath, RenderInstanceDesc, pxr::SdfPath::Hash> RenderInstanceDescMap;

public:
    void release()
    {
        mRenderInstanceDescMap.clear();
    }

    bool exists(const pxr::SdfPath& path, const pxr::GfMatrix4f& matrix);

    RenderInstanceDesc* get(const pxr::SdfPath& path);

    void addOrUpdate(const pxr::SdfPath& path,
                     omni::renderer::RenderInstanceBuffer renderInstanceBuffer,
                     size_t renderInstanceNum,
                     TriangleMeshBuffer* triangleMeshBuffer,
                     const pxr::GfMatrix4f& matrix);

    bool getAndRemove(const pxr::SdfPath& path,
                      omni::renderer::RenderInstanceBuffer& renderInstanceBuffer,
                      size_t& renderInstanceNum,
                      TriangleMeshBuffer*& triangleMeshBuffer,
                      pxr::GfMatrix4f& matrix);

private:
    RenderInstanceDescMap mRenderInstanceDescMap;
};

class DebugVisualization
{
public:
    typedef pxr::TfHashMap<pxr::SdfPath, bool, pxr::SdfPath::Hash> TimeVaryingPrimMap;

public:
    DebugVisualization();

    ~DebugVisualization();

    void enableNormalsVisualization(bool enable);

    void setColliderVisualization(VisualizerMode mode);

    bool isUsdBasedVisualizationEnabled() const
    {
        return mColliderVisualization != VisualizerMode::eNone || mVehicleDebugVis.getVisualizationFlags() ||
               mCollisionMeshDebugVis->isEnabled();
    }

    void draw();

    void setSceneLoaded(bool loaded);

    void setIsPlaying(bool isPlaying)
    {
        if (mSceneIsPlaying && !isPlaying)
        {
            updateDebugVisualization();
        }

        mSceneIsPlaying = isPlaying;
    }

    void onResyncedPrimPath(const pxr::SdfPath&); // excluding resynced path notifications of attributes
    void onAttributeChange(const pxr::SdfPath& attrPath,
                           const pxr::SdfPath& primPath,
                           const pxr::UsdPrim& prim,
                           bool resynced,
                           bool isXform,
                           bool needsDescCacheReset);

    void selectionChanged();

    void setSceneDirty()
    {
        mSceneDirty = true;
    }

    void setCollisionMeshType(const char* type)
    {
        mCollisionMeshDebugVis->setCollisionMeshType(type);
        mXformDirty = true;
    }

    void enableCollisionMeshVisualization(bool enable)
    {
        mCollisionMeshDebugVis->enableCollisionMeshVisualization(enable);
        // Also trigger an update for the debug mesh visualization: this in turn will trigger syncTransform()
        // and will prevent any artifact since the collider mesh will be *immediately* synchronized at the same position
        // as the mesh it's referring to.
        mCollisionMeshDebugVis->updateDebugVisualization();
        mSceneDirty = true;
    }

    bool HasPointInstancerParent(const pxr::UsdPrim& prim);

    void explodeViewDistance(float distance)
    {
        mCollisionMeshDebugVis->explodeViewDistance(distance);
        mXformDirty = true;
    }

    void setVehicleVisualization(PhysXVehicleVisualizationParameter::Enum param, bool enable)
    {
        bool old = getVehicleVisualization(param);
        if (old != enable)
        {
            mVehicleDebugVis.setVisualization(param, enable);

            if (mVehicleDebugVis.getVisualizationFlags())
                mSceneDirty = true;
            else
                mVehicleDebugVis.clear(false);
        }
    }

    bool getVehicleVisualization(PhysXVehicleVisualizationParameter::Enum param) const
    {
        return mVehicleDebugVis.getVisualization(param);
    }

    void updateDebugVisualization()
    {
        mSceneDirty = true;
    }

    void setFabricEnabled(bool enabled)
    {
        mFabricEnabled = enabled;
        mFabricEnabled ? initFabric() : releaseFabric();
        updateDebugVisualization();
    }

    void setQueryFabricWhileSimEnabled(bool enabled)
    {
        mQueryFabricWhileSim = enabled;
    }

    void setQueryUsdrtForTraversalEnabled(bool enabled)
    {
        mQueryUsdrtForTraversal = enabled;
    }

    void enableRedrawOptimizations(bool enabled)
    {
        if (mOptimizeRedraw != enabled)
        {
            mOptimizeRedraw = enabled;
            mSceneDirty = true;
        }
    }

    void setSimplifyDebugVisDistance(float distance);

    void setCameraPos(const pxr::GfVec3f& worldPos)
    {
        mCurrentCameraPos = worldPos;
    }

    bool testInternalState(int phase) const;

private:
    void initFabric();
    void releaseFabric();

    pxr::GfMatrix4d GetLocalToWorldTransform(const pxr::UsdPrim& prim);

    usdparser::PhysxShapeDesc* getShapeDesc(const pxr::UsdPrim& prim, const pxr::UsdPrim& colPrim, const uint64_t stageId);

    void updateCollisionPrimSimplifiedAABB(const pxr::SdfPath& primPath, const pxr::GfRange3d& primAABB);

    void updateCollisionPrimTriangeMeshShape(const pxr::SdfPath& primPath,
                                             const pxr::GfMatrix4d& primTransformMtx,
                                             usdparser::PhysxShapeDesc* shapeDesc);

    void removePrimFromAABBCache(const pxr::SdfPath& primPath);
    void removePrimFromTrimeshCache(const pxr::SdfPath& primPath);

    void updateUsdBasedDebugVisualization(float metersPerUnit, pxr::UsdStageWeakPtr stage);

    void updateUsdBasedDebugVisualizationInRange(const pxr::UsdPrimRange range,
                                                 float metersPerUnit,
                                                 pxr::UsdStageWeakPtr stage,
                                                 bool ignorePrimsWithSelectionSupport);

    void updateUsdBasedDebugVisualizationInGroupsFabric(float metersPerUnit, pxr::UsdStageWeakPtr stage);

    void updateCollisionPrimDebugVisualization(const pxr::UsdPrim& prim, float metersPerUnit);

    void updatePointInstancerPrimDebugVisualization(const pxr::UsdPrim& prim,
                                                    float metersPerUnit,
                                                    pxr::UsdStageWeakPtr stage);

    void parsePrototype(const pxr::UsdStageRefPtr stage, const pxr::UsdPrim& usdPrim, TargetCollisionList& collisionList);

    void drawLine(const DebugLine& line, omni::renderer::SimplexBuffer lineId, size_t& counter);
    void drawLineCol(const DebugLine& line, omni::renderer::SimplexBuffer lineId, size_t& counter);

    void drawPoint(const DebugPoint& point, float metersPerUnit);

    void createShapeDebugLineList();
    void releaseShapeDebugLineList();

    void releaseTriangleMeshLineLists();

    void clearCaches();

    bool isFabricDirty();
    void updateFabricSync();

    void createAABBVisBuffer();
    void releaseAABBVisBuffer();

    bool isTimeVaryingPrim(pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& prim);

private:
    VisualizerMode mColliderVisualization;
    bool mSceneIsPlaying;

    bool mSceneLoaded;
    bool mSceneDirty;
    bool mXformDirty;
    bool mCameraMovement;
    bool mOptimizeRedraw;

    carb::extras::Timer mLastUSDNoticeTime; // The amount of time elapsed since the last USD notification was received
    carb::extras::Timer mLastUSDChangeTime; // The amount of time elapsed since the last USD update
    carb::extras::Timer mLastCameraUpdateTime; // The amount of time elapsed since the last debug vis update based on
                                               // camera movement
    carb::extras::Timer mLastXformNotificationTime; // The amount of time elapsed since the last xform notification
                                                    // change
    carb::extras::Timer mLastXformChangeTime; // The amount of time elapsed since the last xform update

    bool mFabricEnabled{ true }; // General on/off switch for Fabric
    bool mQueryFabricWhileSim{ false }; // Query fabric while simulating. mFabricEnabled must be true for this to work.
    bool mQueryUsdrtForTraversal{ true }; // Query usdrt for stage traversal. mFabricEnabled must be true for this to
                                          // work.
    omni::core::ObjectPtr<usdrt::xformcache::IXformCache> mFabricSync{ nullptr };
    omni::fabric::IStageReaderWriter* mIFabricStageReaderWriter{ nullptr };
    omni::fabric::IChangeTrackerConfig* mIChangeTrackerConfig{ nullptr };
    omni::fabric::ListenerId mFabricListener{ 0 }; // for getting fabric values while simulating
    usdrt::UsdStageRefPtr mUsdrtStage; // USDRT stage for fast traversing prims

    TimeVaryingPrimMap mTimeVaryingPrimMap; // a dictionary of time varying prims (true/false). Unused right now.
    TriangleMeshBuffer mAABBDebugVisBuffer; // a single AABB debug visualization to be instanced and transformed for
                                            // simplified debug render
    omni::renderer::SimplexBuffer mShapeDebugLineBuffer; // debug line buffer used for all debug draw lines except for
                                                         // triangle mesh shapes
    omni::renderer::RenderInstanceBuffer mShapeDebugRenderInstanceBuffer; // render instance buffer used for
                                                                          // mShapeDebugLineBuffer
    TriangleMeshBufferMap mTriangleMeshBuffers; // cache of trimesh debug draw line buffers for instancing
    RenderInstanceCache mAABBRenderInstanceCache; // Active AABB instances cache
    RenderInstanceCache mTrimeshInstanceCache; // Active trimesh instances cache
    size_t mLastLineBufferSize;
    size_t mShapeDebugIndex;
    bool mShapeDebugLineEmpty;
    DescCache mDescCache; // cache of shape descriptions
    PathSet mShapePathCache;
    PathSet mPointInstancerPathCache;
    pxr::SdfPathSet mMergeMeshPathCache;
    pxr::UsdGeomXformCache mXformCache;
    DebugVisualizationVehicle mVehicleDebugVis;
    DebugVisualizationCollisionMesh* mCollisionMeshDebugVis{ nullptr };
    pxr::GfVec3f mCurrentCameraPos, mPrevCameraPos;
    float mSimplifyDebugVisAtDistanceSq;
};

} // namespace ui
} // namespace physx
} // namespace omni
