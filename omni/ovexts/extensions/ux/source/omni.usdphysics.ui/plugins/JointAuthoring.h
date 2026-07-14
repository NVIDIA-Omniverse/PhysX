// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
    JointAuthoring(PXR_NS::UsdStageWeakPtr stage, const PXR_NS::SdfPath& jointPath, const PXR_NS::SdfPath& selectedPrimPath);
    ~JointAuthoring();

    // void updateAxisGizmo(omni::physics::schema::JointDesc* desc);  // NEEDED?

    void draw(const PXR_NS::GfMatrix4d& viewMatrix,
              const PXR_NS::GfMatrix4d& projMatrix,
              const carb::Float4& viewPortRect,
              bool clipPositiveZ);

    void release();


private:
    friend class JointAuthoringManager;
    PXR_NS::UsdStageWeakPtr mStage;
    carb::settings::ISettings*  mSettings;

    omni::physics::usdparser::IUsdPhysicsParse* mUsdPhysicsParse;
    omni::physics::schema::JointDesc* mJointDesc;

    PXR_NS::SdfPath mSelectedPrimPath;

    bool mWasUsing = false;
    PXR_NS::GfMatrix4d mOrigTransform;
};

using JointAuthoringMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, JointAuthoring*, PXR_NS::SdfPath::Hash>;
using JointBodyMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, PXR_NS::SdfPathSet, PXR_NS::SdfPath::Hash>;
using JointBodyChange = PXR_NS::TfHashMap<PXR_NS::SdfPath, uint32_t, PXR_NS::SdfPath::Hash>;

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

    void handleUsdNotice(const PXR_NS::UsdNotice::ObjectsChanged& objectsChanged);

    void addJointPath(const PXR_NS::SdfPath& jointPath, const PXR_NS::SdfPath& selectedPrimPath);
    void removeJointPath(const PXR_NS::SdfPath& jointPath);
    void update();
    void selectionChanged();
    void release();
    bool hasSomeGizmoBeenDeleted(bool resetStatus); // This is only for VP1, can be removed once VP1 is dropped

    void draw(
        const PXR_NS::GfMatrix4d& viewMatrix,
        const PXR_NS::GfMatrix4d& projMatrix,
        const carb::Float4& viewPortRect,
        bool clipPositiveZ);

    void resyncPath(const PXR_NS::SdfPath& path);
    void updateJointTransformData(const PXR_NS::SdfPath& primPath);
    void updateJointBodyPaths(const PXR_NS::SdfPath& jointPath, bool body0);
    void clearBufferedUpdatesForJointBodyPaths(const PXR_NS::SdfPath& jointPath);
    void updateJointGizmoTransformFromPath(const PXR_NS::SdfPath& jointPath);
    void updateJointVisibility(const PXR_NS::SdfPath& jointPath);
    void refreshAllGizmos();

    void parseStage(PXR_NS::UsdStageWeakPtr stage);
    void stageClosed();
    bool removePrim(const PXR_NS::SdfPath& path);
    void resyncPrim(const PXR_NS::SdfPath& path);

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
            for (const PXR_NS::SdfPathSet::const_reference& path : mBlockedResyncPrims)
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
    void mapBodyToJoint(const PXR_NS::SdfPath& bodyPath, const PXR_NS::SdfPath& jointPath);
    void removeJointFromBodies(const PXR_NS::SdfPath& jointPath);

    void updateJointScaleDataInternal();
    void updateJointTransformDataInternal();
    void updateJointBodyPathsInternal();
    void updateJointGizmoTransformInternal();
    void updateGizmosFadeOutScale();

    void createJointHelper(const PXR_NS::SdfPath& jointPath, omni::physics::schema::JointDesc* desc, JointData& jointData);

    void setupSelectionOutline(const PXR_NS::UsdPrim& jointPrim);
    void clearSelectionOutline();

    void removeBillboard(uint32_t assetId, uint32_t& billboardId);
    void addOrUpdateBillboard(uint32_t assetId,
                              uint32_t& billboardId,
                              uint32_t billboardUid,
                              const PXR_NS::GfMatrix4f& localTransform);
    void clearAllBillboards();

    omni::renderer::IDebugDraw* mDebugDraw;
    carb::settings::ISettings*  mSettings;
    PXR_NS::UsdStageWeakPtr        mStage;
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

    PXR_NS::SdfPath        mSelectedJointPrim;
    PXR_NS::SdfPathVector  mSelectionOutlineMeshes;

    // buffered notices
    PXR_NS::SdfPathSet     mAddedPrims;
    PXR_NS::SdfPathSet     mJointTransformData;
    PXR_NS::SdfPathSet     mJointScaleData;
    PXR_NS::SdfPathSet     mJointGizmoTransform;
    JointBodyChange     mJointBodyPaths;

    PXR_NS::SdfPathSet     mBlockedResyncPrims;

    PrimHierarchyStorage    mPrimHierarchyStorage;
    PXR_NS::UsdGeomXformCache  mXfCache;

    PXR_NS::GfVec3f        mCameraPos;
    float               mPrevGizmoScale;
    bool                mGizmosDirty;

    PXR_NS::GfMatrix4d     mViewMatrix;
    PXR_NS::GfMatrix4d     mProjectionMatrix;
    PXR_NS::GfMatrix4d     mViewProjection;
    PXR_NS::GfMatrix4d     mViewInverse;
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
