// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once
#include <common/utilities/PrimHierarchyStorage.h>
#include <omni/physics/ui/IUsdPhysicsUI.h>
#include <omni/renderer/IDebugDraw.h>

#include "JointBillboardScaleTask.h"


namespace carb
{
    namespace imgui
    {
        struct ImGui;
    }
} // namespace carb
namespace omni
{
namespace physics
{
namespace schema
{
    struct JointDesc;
    struct FixedJointDesc;
    struct RevoluteJointDesc;
    struct SphericalJointDesc;
    struct DistanceJointDesc;
    struct D6JointDesc;
    struct PrismaticJointDesc;
    struct ObjectType;
    struct Axis;
} // namespace schema
namespace usdparser
{
struct IUsdPhysicsParse;
}
} // namespace physics
} // namespace omni
namespace omni
{

namespace physics
{

namespace ui
{
class JointAuthoring
{
public:
    JointAuthoring(pxr::UsdStageWeakPtr stage, const pxr::SdfPath& jointPath, const pxr::SdfPath& selectedPrimPath);
    ~JointAuthoring();

    // void updateAxisGizmo(omni::physics::schema::JointDesc* desc);  // NEEDED?

    void draw(const pxr::GfMatrix4d& viewMatrix,
              const pxr::GfMatrix4d& projMatrix,
              const carb::Float4& viewPortRect,
              bool clipPositiveZ);

    void release();


private:
    friend class JointAuthoringManager;
    pxr::UsdStageWeakPtr mStage;
    carb::settings::ISettings*  mSettings;

    omni::physics::usdparser::IUsdPhysicsParse* mUsdPhysicsParse;
    omni::physics::schema::JointDesc* mJointDesc;

    pxr::SdfPath mSelectedPrimPath;

    bool mWasUsing = false;
    pxr::GfMatrix4d mOrigTransform;
};

using JointAuthoringMap = pxr::TfHashMap<pxr::SdfPath, JointAuthoring*, pxr::SdfPath::Hash>;
using JointBodyMap = pxr::TfHashMap<pxr::SdfPath, pxr::SdfPathSet, pxr::SdfPath::Hash>;
using JointBodyChange = pxr::TfHashMap<pxr::SdfPath, uint32_t, pxr::SdfPath::Hash>;

class JointAuthoringManager
{
public:
    JointAuthoringManager(bool showJointMeshes);
    ~JointAuthoringManager();

    void init(const std::string& extensionPath);

    IUsdPhysicsUICustomJointAuthoring::RegistrationID registerCustomJointAuthoring(
        IUsdPhysicsUICustomJointAuthoring& customJointAuthoring, const char* name);
    bool unregisterCustomJointAuthoring (IUsdPhysicsUICustomJointAuthoring::RegistrationID registrationID);
    uint32_t registerCustomJointBillboard(IUsdPhysicsUICustomJointAuthoring::RegistrationID registrationID,
                                          const char* pngPath);

    void handleUsdNotice(const pxr::UsdNotice::ObjectsChanged& objectsChanged);

    void addJointPath(const pxr::SdfPath& jointPath, const pxr::SdfPath& selectedPrimPath);
    void removeJointPath(const pxr::SdfPath& jointPath);
    void update();
    void selectionChanged();
    void release();
    bool hasSomeGizmoBeenDeleted(bool resetStatus); // This is only for VP1, can be removed once VP1 is dropped

    void draw(
        const pxr::GfMatrix4d& viewMatrix,
        const pxr::GfMatrix4d& projMatrix,
        const carb::Float4& viewPortRect,
        bool clipPositiveZ);

    void resyncPath(const pxr::SdfPath& path);
    void updateJointTransformData(const pxr::SdfPath& primPath);
    void updateJointBodyPaths(const pxr::SdfPath& jointPath, bool body0);
    void clearBufferedUpdatesForJointBodyPaths(const pxr::SdfPath& jointPath);
    void updateJointGizmoTransformFromPath(const pxr::SdfPath& jointPath);
    void updateJointVisibility(const pxr::SdfPath& jointPath);
    void refreshAllGizmos();

    void parseStage(pxr::UsdStageWeakPtr stage);
    void stageClosed();
    bool removePrim(const pxr::SdfPath& path);
    void resyncPrim(const pxr::SdfPath& path);

    void gizmoSettingsDirty()
    {
        mGizmosDirty = true;
    }

    void setJointMeshesVisibilty(bool showJointMeshes);

    bool empty() const
    {
        return (mJointAuthoringMap.empty() && mJointDataMap.empty());
    }

    bool notificationBlocked() const
    {
        return mNotificationBlocked;
    }

    void setNoficationBlocked(bool val)
    {
        mNotificationBlocked = val;
        if(!mNotificationBlocked)
        {
            for (const pxr::SdfPathSet::const_reference& path : mBlockedResyncPrims)
            {
                resyncPrim(path);
            }
            mBlockedResyncPrims.clear();
        }
    }

    bool hidden() const
    {
        return mHidden;
    }

    void setIsPlaying(bool playing)
    {
        mIsPlaying = playing;
    }

    void setVisibilityFlags(VisibilityFlags::Enum visibilityFlags)
    {
        mVisibilityFlags = visibilityFlags;
    }

    VisibilityFlags::Enum getVisibilityFlags() const 
    {
        return mVisibilityFlags;
    }

private:
    // make sure default constructor isn't publicly available
    JointAuthoringManager();
    void checkRegistrations();

    void updateJointGizmoTransform(omni::physics::schema::JointDesc* desc);
    void mapBodyToJoint(const pxr::SdfPath& bodyPath, const pxr::SdfPath& jointPath);
    void removeJointFromBodies(const pxr::SdfPath& jointPath);

    void updateJointScaleDataInternal();
    void updateJointTransformDataInternal();
    void updateJointBodyPathsInternal();
    void updateJointGizmoTransformInternal();
    void updateGizmosFadeOutScale();

    void createJointHelper(const pxr::SdfPath& jointPath, omni::physics::schema::JointDesc* desc, JointData& jointData);

    void setupSelectionOutline(const pxr::UsdPrim& jointPrim);
    void clearSelectionOutline();

    void removeBillboard(uint32_t assetId, uint32_t& billboardId);
    void addOrUpdateBillboard(uint32_t assetId,
                              uint32_t& billboardId,
                              uint32_t billboardUid,
                              const pxr::GfMatrix4f& localTransform);
    void clearAllBillboards();

    omni::renderer::IDebugDraw* mDebugDraw;
    carb::settings::ISettings*  mSettings;
    pxr::UsdStageWeakPtr        mStage;
    bool                        mHasSomeGizmoBeenDeleted;

    std::vector<uint32_t> mBillboardAssetIds;
    std::vector<std::string> mBillboardImagePaths;

    JointAuthoringMap   mJointAuthoringMap;    
    JointBodyMap        mJointBodyMap;
    JointDataMap        mJointDataMap;
    bool                mHidden;
    bool                mUpdateSelection;
    bool                mIsPlaying;
    bool                mNotificationBlocked;

    uint8_t             mDynamicBodySelectionGroup;
    uint8_t             mStaticBodySelectionGroup;

    pxr::SdfPath        mSelectedJointPrim;
    pxr::SdfPathVector  mSelectionOutlineMeshes;

    // buffered notices
    pxr::SdfPathSet     mAddedPrims;
    pxr::SdfPathSet     mJointTransformData;
    pxr::SdfPathSet     mJointScaleData;
    pxr::SdfPathSet     mJointGizmoTransform;
    JointBodyChange     mJointBodyPaths;

    pxr::SdfPathSet     mBlockedResyncPrims;

    PrimHierarchyStorage    mPrimHierarchyStorage;
    pxr::UsdGeomXformCache  mXfCache;

    pxr::GfVec3f        mCameraPos;
    float               mPrevGizmoScale;
    bool                mGizmosDirty;

    pxr::GfMatrix4d     mViewMatrix;
    pxr::GfMatrix4d     mProjectionMatrix;
    pxr::GfMatrix4d     mViewProjection;
    pxr::GfMatrix4d     mViewInverse;
    carb::Float4        mViewPortRect;

    VisibilityFlags::Enum mVisibilityFlags =  VisibilityFlags::eSHOW_ALL;

    JointBillboardScaleTask mJointBillboardScaleTask;
    omni::physics::usdparser::IUsdPhysicsParse* mUsdPhysicsParse;
    uint64_t lastRegistrationID = 1;

    struct CustomJointAuthoring
    {
        std::string name;
        IUsdPhysicsUICustomJointAuthoring callbacks;
        struct JointBillboard
        {
            std::string path;
            uint32_t billboardId;
            JointBillboard(const char* cpath) : path(cpath), billboardId(0){};
        };
        // address of the string must not change to avoid crashes in async renderer
        std::vector<std::unique_ptr<JointBillboard>> jointBillboards; 
    };

    std::unordered_map<IUsdPhysicsUICustomJointAuthoring::RegistrationID, CustomJointAuthoring> registrations;
    enum class JointEvent
    {
        JointCreated,
        JointDeleted
    };
    void notifyListenersAbout(JointEvent event, uint64_t jointPath);
    bool notifyListenersAboutScale(uint64_t jointPath,
                                   const IUsdPhysicsUICustomJointAuthoring::JointScaleData& jointScaleData);
};


} // namespace ui
} // namespace physics
} // namespace omni
