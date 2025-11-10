// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <common/utilities/Utilities.h>
#include <private/omni/physics/IUsdPhysicsParse.h>
#include <common/utilities/PrimUtilities.h>
#include <omni/physics/IUsdPhysicsSettings.h>
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/Selection.h>
#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>
#include <omni/kit/PythonInterOpHelper.h>

#include "JointAuthoring.h"

#include <common/ui/ImguiDrawingUtils.h>

// usdrt
#include <usdrt/scenegraph/usd/usd/stage.h>
#include <omni/fabric/usd/PathConversion.h>


static constexpr char kViewportGizmoScalePath[] = PERSISTENT_SETTINGS_PREFIX "/app/viewport/gizmo/scale";
static constexpr char kViewportGizmoConstantScaleEnabledPath[] = PERSISTENT_SETTINGS_PREFIX "/app/viewport/gizmo/constantScaleEnabled";
static constexpr char kViewportGizmoConstantScalePath[] = PERSISTENT_SETTINGS_PREFIX "/app/viewport/gizmo/constantScale";
static constexpr char kViewportGizmoMinFadeOutPath[] = PERSISTENT_SETTINGS_PREFIX "/app/viewport/gizmo/minFadeOut";
static constexpr char kViewportGizmoMaxFadeOutPath[] = PERSISTENT_SETTINGS_PREFIX "/app/viewport/gizmo/maxFadeOut";

static constexpr int kMaxUpdateTimeMilliseconds = 8; // Max number of milliseconds spent in a single update, before
                                                     // offloading the remaining joints to parse to next update

static const float gCameraDistanceCheck = 0.01f;

static constexpr float kGizmoMagicScaleFactor = 0.5f;

using namespace carb;
using namespace imgui;
using namespace omni::physics::ui;
using namespace omni::physics::ui;
using namespace omni::renderer;
using namespace pxr;
using namespace omni::physics::schema;


static float angleToRadians(float angleInDegrees)
{
    return static_cast<float>(angleInDegrees / 180.0f * M_PI);
}

static const uint32_t gLimitColor = 0xBB4444FF;
static const uint32_t gLimitFreeColor = 0xBB44FF44;
static const uint32_t gBlueColor = 0xDB801010;
static const uint32_t gGreenColor = 0xDB108010;
static const uint32_t gRedColor = 0xDB101080;
static float gLineThickness = 2.0f;

#define USE_FILLED_TRIANGLES 0


// helper to calculate the world space transform for the given body in a joint
static pxr::GfMatrix4d getWorldSpaceTransform(UsdStageWeakPtr stage, const omni::physics::schema::JointDesc* desc, bool forBody0)
{
    const pxr::SdfPath usePrimPath = (forBody0 ? desc->body0 : desc->body1);
    const pxr::GfVec3f localPos = (forBody0 ? desc->localPose0Position : desc->localPose1Position);
    const pxr::GfQuatf localRot = (forBody0 ? desc->localPose0Orientation : desc->localPose1Orientation);
    return GetGizmoMatrix(stage, localPos, localRot, usePrimPath);
}


// Figure out what the pose of the given joint should be in world space
static pxr::GfMatrix4d getWorldSpacePose(UsdStageWeakPtr stage, const JointDesc* desc, bool forMesh)
{
    // default to using the transform from body0
    pxr::GfMatrix4d worldPose = getWorldSpaceTransform(stage, desc, true);

    // then figure out the rotation component
    pxr::GfVec3f worldAxis = {0, 0, 1.0f};
    switch (desc->type)
    {
        // default is to use the x-axis of the body
        default:
            worldAxis = pxr::GfVec3f(worldPose.GetRow3(Axis::eX));
            break;

        // these joints define a specific axis in body space
        case ObjectType::eJointPrismatic:
            worldAxis = pxr::GfVec3f(worldPose.GetRow3(((PrismaticJointDesc*)desc)->axis));
            break;
        case ObjectType::eJointSpherical:
            worldAxis = pxr::GfVec3f(worldPose.GetRow3(((SphericalJointDesc*)desc)->axis));
            break;
        case ObjectType::eJointRevolute:
            worldAxis = pxr::GfVec3f(worldPose.GetRow3(((RevoluteJointDesc*)desc)->axis));
            break;

        // distance joint defines its axis as the vector between the attach points
        case ObjectType::eJointDistance:
        {
            const pxr::GfVec3d body0Pos = worldPose.ExtractTranslation();
            const pxr::GfVec3d body1Pos = getWorldSpaceTransform(stage, desc, false).ExtractTranslation();

            // only have to calculate the axis vector if this is for the joint gizmo mesh
            if (forMesh)
            {
                worldAxis = pxr::GfVec3f(body1Pos - body0Pos);
                const float len = worldAxis.Normalize();
                if (len < 0.001f)
                {
                    worldAxis = pxr::GfVec3f(worldPose.GetRow3(Axis::eX));
                }
            }

            // override the translation to be the mid-point between the attach points
            const pxr::GfVec3d avgPos = 0.5 * (body1Pos + body0Pos);
            worldPose.SetTranslateOnly(avgPos);
            break;
        }
    }

    // if we are getting the world space pose for the gizmo mesh,
    // then we need to do an extra rotation since the mesh is aligned to its local x-axis
    if (forMesh)
    {
        const pxr::GfVec3f xAxis(1.0f, 0.0f, 0.0f);
        const pxr::GfRotation rotW = pxr::GfRotation(xAxis, worldAxis);
        worldPose.SetRotateOnly(rotW);
    }

    return worldPose;
}


JointAuthoring::JointAuthoring(pxr::UsdStageWeakPtr stage, const pxr::SdfPath& jointPath, const pxr::SdfPath& selectedPrimPath)
{
    mStage = stage;
    mSettings = carb::getCachedInterface<carb::settings::ISettings>();
    const uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();

    mUsdPhysicsParse = carb::getCachedInterface<omni::physics::usdparser::IUsdPhysicsParse>();

    JointDesc* jointDesc = mUsdPhysicsParse->parseJoint(stageId, jointPath);
    CARB_ASSERT(jointDesc, "Failed to parse joint: %s", jointPath.GetText());
    mJointDesc = nullptr;
    mSelectedPrimPath = selectedPrimPath;
}


pxr::SdfPath GetJointPrimPath(const JointDesc* desc)
{
    return desc->usdPrim.GetPrimPath();
}

static void calcLocalPoseMatrix(
    bool editLocalPose,
    const GfMatrix4d& refMatrix,
    pxr::UsdStageWeakPtr stage,
    const pxr::SdfPath& relPath,
    const pxr::UsdAttribute& localPosAttr,
    const pxr::UsdAttribute& localRotAttr
)
{
    GfMatrix4d matR = refMatrix;
    const UsdPrim relPrim = stage->GetPrimAtPath(relPath);
    if (relPrim)
    {
        UsdGeomXformCache xfCache;
        const GfMatrix4d relMat = xfCache.GetLocalToWorldTransform(relPrim);
        if (editLocalPose)
        {
            // local pose needs to be calculated as the relative transform from the body to the given refMatrix
            matR = matR * relMat.GetInverse();
        }
        else
        {
            // set the transform of the rel prim so the local pose is preserved
            GfVec3f curPos;
            GfQuatf curRot;
            localPosAttr.Get(&curPos);
            localRotAttr.Get(&curRot);
            matR.SetRotate(curRot);
            matR.SetTranslateOnly(curPos);

            // have to pull scale out of the current matrix so it is preserved as well
            const GfTransform tr(relMat);
            const GfVec3d scale = tr.GetScale();
            const GfMatrix4d scaleMat(GfVec4d(scale[0], scale[1], scale[2], 1.0));

            // now combine the components to get the new transform for the rel prim
            const pxr::GfMatrix4d newMatrix = scaleMat * matR.GetInverse() * refMatrix;
            omni::usd::UsdUtils::setLocalTransformFromWorldTransformMatrix(relPrim, newMatrix);

            // no need to update the prim attributes, the local pose info doesn't change in this case
            // the position of the rel prim is updated instead
            // local pose and rel prim transform are combined in the joint description,
            // so we only need to update one here
            return;
        }
    }

    // batch up the USD changes so notice handlers don't get spammed by updating the attributes separately
    pxr::SdfChangeBlock changeBlock;

    matR = matR.RemoveScaleShear();
    const GfVec3f pos = GfVec3f(matR.ExtractTranslation());
    const GfQuatf rot = GfQuatf(matR.ExtractRotationQuat().GetNormalized());

    localPosAttr.Set(pos);
    localRotAttr.Set(rot);
}



void JointAuthoring::release()
{
    if (mJointDesc)
    {
        mUsdPhysicsParse->releaseDesc(mJointDesc);
        mJointDesc = nullptr;
    }
}

JointAuthoring::~JointAuthoring()
{
    release();
}

struct JointType
{
    enum Enum 
    {
        eRevoluteJoint = 0,
        eSphericalJoint,
        eDistanceJoint,
        eFixedJoint,
        ePrismaticJoint,
        eD6Joint,
        eJJoint,
        eJointError,
    };
};

JointAuthoringManager::JointAuthoringManager(bool showJointMeshes)
{
    mHasSomeGizmoBeenDeleted = false;
    mSettings = carb::getCachedInterface<carb::settings::ISettings>();
    mUsdPhysicsParse = carb::getCachedInterface<omni::physics::usdparser::IUsdPhysicsParse>();

    setJointMeshesVisibilty(showJointMeshes);
    mUpdateSelection = false;
    mCameraPos = GfVec3f(0.0f);
    mGizmosDirty = false;
    mPrevGizmoScale = 1.0f;
    mIsPlaying = false;
    mNotificationBlocked = false;

    mSelectedJointPrim = SdfPath();

    mDebugDraw = carb::getCachedInterface<omni::renderer::IDebugDraw>();

    const auto usdContext = omni::usd::UsdContext::getContext();
    mDynamicBodySelectionGroup = usdContext->registerSelectionGroup();
    mStaticBodySelectionGroup = usdContext->registerSelectionGroup();
    usdContext->setSelectionGroupOutlineColor(mStaticBodySelectionGroup, { 0.9f, 0.0f, 0.0f, 1.0f });
    usdContext->setSelectionGroupShadeColor(mStaticBodySelectionGroup, { 0.0f, 0.0f, 0.0f, 0.0f });
    usdContext->setSelectionGroupOutlineColor(mDynamicBodySelectionGroup, { 0.0f, 0.9f, 0.0f, 1.0f });
    usdContext->setSelectionGroupShadeColor(mDynamicBodySelectionGroup, { 0.0f, 0.0f, 0.0f, 0.0f });    
}

void JointAuthoringManager::init(const std::string& extensionPath)
{
    // order has to match JointType::Enum
    std::vector<std::string> jointStrings =
    { "RevoluteJoint", "SphericalJoint", "DistanceJoint", "FixedJoint", "PrismaticJoint",
       "D6Joint",  "JJoint", "JointError"
    };

    mBillboardAssetIds.resize(jointStrings.size());
    mBillboardImagePaths.resize(jointStrings.size());
    for (size_t i = 0; i < jointStrings.size(); i++)
    {
        const std::string imagePath = extensionPath + "/icons/physicsJoint/" + jointStrings[i] + ".png";
        mBillboardImagePaths[i] = imagePath;
        // DebugDraw expects that the c_str we're passing is valid until when the async renderer
        // decides to actually load this image...
        mBillboardAssetIds[i] = mDebugDraw->addBillboardAsset(mBillboardImagePaths[i].c_str());
    }
}

void JointAuthoringManager::checkRegistrations()
{
    while(!registrations.empty())
    {        
        CARB_LOG_ERROR("JointAuthoringManager client \"%s\" forgot to call unregisterCustomJointAuthoring", registrations.begin()->second.name.c_str() );
        // unregister all custom joints that forgot to de-register themselves
        unregisterCustomJointAuthoring(registrations.begin()->first);
    }
}

void JointAuthoringManager::clearAllBillboards()
{
    for (size_t i = 0; i < mBillboardAssetIds.size(); i++)
    {
        mDebugDraw->clearBillboardInstances(mBillboardAssetIds[i]);
    }

    for(const auto& registration : registrations)
    {
        for(const auto& jointBillboard : registration.second.jointBillboards)
        {            
            mDebugDraw->clearBillboardInstances(jointBillboard->billboardId);
        }
    }

    for (JointDataMap::reference jointDataRef : mJointDataMap)
    {
        jointDataRef.second.billboardId = kInvalidBillboardId;
    }
}


void JointAuthoringManager::addJointPath(const SdfPath& jointPath, const SdfPath& selectedPrimPath)
{
    JointAuthoringMap::iterator it = mJointAuthoringMap.find(jointPath);
    if (it == mJointAuthoringMap.end())
    {
        mJointAuthoringMap[jointPath] = new JointAuthoring(mStage, jointPath, selectedPrimPath);
        notifyListenersAbout(JointEvent::JointCreated, asInt(jointPath));
    }

    // make sure the gizmo and mesh are in the right place to start
    updateJointGizmoTransformFromPath(jointPath);
}

void JointAuthoringManager::removeJointPath(const SdfPath& jointPath)
{
    JointAuthoringMap::iterator it = mJointAuthoringMap.find(jointPath);
    if (it != mJointAuthoringMap.end())
    {
        delete it->second;
        notifyListenersAbout(JointEvent::JointDeleted, asInt(jointPath));
        mHasSomeGizmoBeenDeleted = true;
        mJointAuthoringMap.erase(it);
    }
}

void JointAuthoringManager::selectionChanged()
{
    const auto usdContext = omni::usd::UsdContext::getContext();
    const auto selectedPaths = usdContext->getSelection()->getSelectedPrimPaths();    

    release();
    clearSelectionOutline();

    if (!mStage && mStage != usdContext->getStage())
        return;
    
    std::vector<std::string> primPaths;
    for (std::string pathStr : selectedPaths)
    {
        const pxr::SdfPath path(pathStr);
        UsdPrim prim = mStage->GetPrimAtPath(path);
        if (!prim)
            continue;

        // first check the joint body map to see if the select prim is connected to a joint
        // if so, grab the joint prim path from the body map
        JointBodyMap::iterator bodyItr = mJointBodyMap.find(path);
        if (bodyItr != mJointBodyMap.end())
        {
            const pxr::SdfPathSet& jointPaths = bodyItr->second;
            for (auto jointPath : jointPaths)
            {
                addJointPath(jointPath, path);
            }
        }
        else 
        {
            // check if the selected prim is a joint prim
            if (prim.IsA<UsdPhysicsJoint>())
            {
                // only show the joint gizmo if there is exactly 1 prim selected
                // we don't support more than 1 gizmo at a time
                mSelectedJointPrim = (selectedPaths.size() == 1 ? path : pxr::SdfPath());
                addJointPath(path, mSelectedJointPrim);

                // setup selection outline                
                setupSelectionOutline(prim);
            }
        }

        primPaths.push_back(pathStr);
    }
}

bool isDynamicBody(const UsdPrim& usdPrim)
{
    UsdPrim parent = usdPrim;
    while (parent && !parent.IsPseudoRoot())
    {
        if (parent.HasAPI<UsdPhysicsRigidBodyAPI>())
        {
            UsdPhysicsRigidBodyAPI rbAPI(parent);
            bool enabled = false;
            rbAPI.GetRigidBodyEnabledAttr().Get(&enabled);

            return enabled;
        }

        parent = parent.GetParent();
    }

    return false;
}

void JointAuthoringManager::setupSelectionOutline(const pxr::UsdPrim& prim)
{
    const auto usdContext = omni::usd::UsdContext::getContext();

    UsdPhysicsJoint physicsJoint(prim);

    SdfPath body[2];
    body[0] = SdfPath();
    body[1] = SdfPath();

    SdfPathVector target;
    physicsJoint.GetBody0Rel().GetTargets(&target);
    if (!target.empty())
    {
        body[0] = target[0];
    }
    target.clear();
    physicsJoint.GetBody1Rel().GetTargets(&target);
    if (!target.empty())
    {
        body[1] = target[0];
    }

    for (int i = 0; i < 2; i++)
    {
        if (body[i] != SdfPath())
        {
            const UsdPrim bodyPrim = mStage->GetPrimAtPath(body[i]);
            if (bodyPrim)
            {                
                uint8_t selectionGroup = isDynamicBody(bodyPrim) ? mDynamicBodySelectionGroup : mStaticBodySelectionGroup;
                UsdPrimRange range(bodyPrim);
                for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
                {
                    const pxr::UsdPrim& meshPrim = *iter;
                    if (!meshPrim)
                        continue;

                    if (meshPrim.IsA<UsdGeomGprim>())
                    {
                        usdContext->setSelectionGroup(selectionGroup, meshPrim.GetPrimPath().GetString());
                        mSelectionOutlineMeshes.push_back(meshPrim.GetPrimPath());
                    }
                }
            }
        }
    }
}

void JointAuthoringManager::stageClosed()
{
    release();
    mJointBodyMap.clear();
    mPrimHierarchyStorage.clear();

    mAddedPrims.clear();
    mJointDataMap.clear();

    mJointBillboardScaleTask.clear();

    mUpdateSelection = false;
    mStage = pxr::UsdStageWeakPtr();
}

void JointAuthoringManager::release()
{
    for (JointAuthoringMap::const_reference& it : mJointAuthoringMap)
    {
        delete it.second;
        notifyListenersAbout(JointEvent::JointDeleted, asInt(it.first));
        mHasSomeGizmoBeenDeleted = true;
    }

    mJointAuthoringMap.clear();

    mJointTransformData.clear();
    mJointGizmoTransform.clear();    
    mJointBodyPaths.clear();
    mJointScaleData.clear();

    mCameraPos = GfVec3f(0.0f);
    mPrevGizmoScale = 1.0f;

    mSelectedJointPrim = SdfPath();
}

void JointAuthoringManager::clearSelectionOutline()
{
    const auto usdContext = omni::usd::UsdContext::getContext();
    for (const SdfPath& path : mSelectionOutlineMeshes)
    {
        if (path != SdfPath())
        {
            usdContext->setSelectionGroup(0, path.GetString());
        }
    }
    mSelectionOutlineMeshes.clear();    
}

void JointAuthoringManager::update()
{
    const bool shouldHideJoints = (mVisibilityFlags & VisibilityFlags::eSHOW_JOINTS) == 0;
    CARB_PROFILE_ZONE(0, "JointAuthoringManager::Update");
    carb::extras::Timer timer;
    timer.start();
    for (auto it = mAddedPrims.cbegin(); it != mAddedPrims.cend(); )
    {
        resyncPath(*it);
        it = mAddedPrims.erase(it);
        // Avoid blocking the main thread for more than kMaxUpdateTimeMilliseconds
        if (timer.getElapsedTime<int64_t>(carb::extras::Timer::Scale::eMilliseconds) >= kMaxUpdateTimeMilliseconds)
        {
            break; // we'll process additional joints in next updates
        }
    }


    updateJointScaleDataInternal();
    updateJointTransformDataInternal();
    updateJointBodyPathsInternal();
    updateJointGizmoTransformInternal();

    // update the selection last because it will clear out all the buffers
    if (mUpdateSelection)
    {
        mUpdateSelection = false;
        selectionChanged();
    }
}

void JointAuthoringManager::updateJointVisibility(const pxr::SdfPath& topPath)
{
    const UsdPrim topPrim = mStage->GetPrimAtPath(topPath);
    // A.B. this has to run on the traversal
    pxr::UsdPrimRange range(topPrim);
    for (pxr::UsdPrimRange::iterator iter = range.begin(); iter != range.end(); ++iter)
    {
        const UsdPrim& prim = *iter;
        if (!prim)
            continue;

        const SdfPath jointPath = prim.GetPrimPath();

        JointDataMap::const_iterator meshItr = mJointDataMap.find(jointPath);
        if (meshItr != mJointDataMap.end())
        {
            if (UsdGeomImageable(prim).ComputeVisibility() == UsdGeomTokens->invisible) // remove prim
            {
                removePrim(jointPath);
            }
            else // resync prim
            {
                resyncPrim(jointPath);
            }
        }
        else
        {
            if (prim.IsA<UsdPhysicsJoint>() && UsdGeomImageable(prim).ComputeVisibility() != UsdGeomTokens->invisible)
            {
                resyncPrim(jointPath);
            }
            else
            {
                mAddedPrims.erase(jointPath);
            }
        }
    }
}

bool JointAuthoringManager::hasSomeGizmoBeenDeleted(bool resetStatus)
{
    const bool oldValue = mHasSomeGizmoBeenDeleted;
    if(resetStatus)
    {
        mHasSomeGizmoBeenDeleted = false;
    }
    return oldValue;
}

void JointAuthoringManager::draw(const pxr::GfMatrix4d& viewMatrix, const pxr::GfMatrix4d& projMatrix, const carb::Float4& viewPortRect, bool clipPositiveZ)
{
    CARB_PROFILE_ZONE(0, "JointAuthoringManager::Draw");

    mViewMatrix = viewMatrix;
    mProjectionMatrix = projMatrix;
    mViewPortRect = viewPortRect;
    mViewProjection = mViewMatrix * mProjectionMatrix;
    mViewInverse = mViewMatrix.GetInverse();


    // Update scale
    if (!mHidden && mStage)
    {
        float gizmoScale = 1.0f;
        const float fixedScreenFactor = mSettings->getAsFloat(kViewportGizmoScalePath);
        const bool constantScaleEnabled = mSettings->getAsBool(kViewportGizmoConstantScaleEnabledPath);

        const float metersPerUnit = float(UsdGeomGetStageMetersPerUnit(mStage));

        if (constantScaleEnabled)
        {
            gizmoScale = mSettings->getAsFloat(kViewportGizmoConstantScalePath) * kGizmoMagicScaleFactor;
        }
        else
        {
            gizmoScale = fixedScreenFactor * kGizmoMagicScaleFactor * 10.0f;
        }

        if (fabsf(gizmoScale - mPrevGizmoScale) >= 1e-6f)
        {
            for (size_t i = 0; i < mBillboardAssetIds.size(); i++)
            {
                mDebugDraw->updateBillboardAsset(mBillboardAssetIds[i], gizmoScale, constantScaleEnabled);
            }

            for (const auto& registration : registrations)
            {
                const auto& callbacks = registration.second.callbacks;
                for(const auto& billboard : registration.second.jointBillboards)
                {
                    mDebugDraw->updateBillboardAsset(billboard->billboardId, gizmoScale, constantScaleEnabled);
                }
            }
            mPrevGizmoScale = gizmoScale;
        }

        GfVec3f currentPos(0.0f);
        {
            currentPos = GfVec3f(mViewInverse.ExtractTranslation());
            const GfVec3f deltaVec = mCameraPos - currentPos;
            const float delta = deltaVec.GetLengthSq();            
            if (delta > gCameraDistanceCheck || mGizmosDirty)
            {
                if (!mJointBillboardScaleTask.initialized())
                {
                    mJointBillboardScaleTask.initialize(currentPos, mJointDataMap, mViewProjection, mViewInverse, mViewPortRect, mSettings, mStage);
                    mJointBillboardScaleTask.execute(!mGizmosDirty);

                    if (mGizmosDirty)
                    {
                        updateGizmosFadeOutScale();
                    }
                }
                else
                {
                    if (mJointBillboardScaleTask.finished())
                    {
                        mJointBillboardScaleTask.setInitialized(false);

                        mCameraPos = mJointBillboardScaleTask.getCurrentCameraPos();
                        updateGizmosFadeOutScale();
                    }
                }

                mGizmosDirty = false;
            }
        }
    }
}

JointAuthoringManager::~JointAuthoringManager()
{
    clearAllBillboards();
    release();
    clearSelectionOutline();
    checkRegistrations();
}

void JointAuthoringManager::parseStage(pxr::UsdStageWeakPtr stage)
{
    CARB_PROFILE_ZONE(0, "JointAuthoringManager::parseStage");
    if (!stage)
        return;

    mStage = stage;
    mJointBodyMap.clear();
    mPrimHierarchyStorage.clear();
    mPrimHierarchyStorage.init(mStage);

    PXR_NS::UsdStageCache& cache = PXR_NS::UsdUtilsStageCache::Get();
    omni::fabric::UsdStageId stageId = { static_cast<uint64_t>(cache.GetId(stage).ToLongInt()) };
    omni::fabric::IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(stageId);
    usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(stageId, stageInProgress);

    for (auto& usdrtPath : usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("UsdPhysicsJoint")))
    {
        const omni::fabric::PathC pathC(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
        mAddedPrims.insert(usdPath);
    }
}

void JointAuthoringManager::setJointMeshesVisibilty(bool showJointMeshes)
{
    if(mHidden == !showJointMeshes)
        return;
    mHidden = !showJointMeshes;

    if (mHidden)
    {
        // Clear everything but without clearing selection outlines
        clearAllBillboards();
        release();
        mAddedPrims.clear();
    }
    else
    {
        parseStage(mStage);
        refreshAllGizmos();
        update();
    }
}


void JointAuthoringManager::removeJointFromBodies(const pxr::SdfPath& jointPath)
{
    // remove the given joint path from all entries in the body map
    std::vector<pxr::SdfPath> bodiesToRemove;
    for (auto& itr : mJointBodyMap)
    {
        // remove the joint path from the list the body is tracking
        itr.second.erase(jointPath);

        // keep track of any bodies that now have empty joint lists
        if (itr.second.size() == 0)
        {
            bodiesToRemove.push_back(itr.first);
        }
    }

    // clean up empty joint lists, can't do it while we iterate because it would modify the map under the iterator
    for (auto& itr : bodiesToRemove)
    {
        mJointBodyMap.erase(itr);
    }
}

bool JointAuthoringManager::removePrim(const pxr::SdfPath& primPath)
{
    bool removedJoint = false;

    PrimHierarchyStorage::Iterator iterator(mPrimHierarchyStorage, primPath);
    for (size_t i = iterator.getDescendentsPaths().size(); i--;)
    {
        const pxr::SdfPath& path = iterator.getDescendentsPaths()[i];

        JointAuthoringMap::iterator jhit = mJointAuthoringMap.find(path);
        if (jhit != mJointAuthoringMap.end())
        {
            delete jhit->second;
            notifyListenersAbout(JointEvent::JointDeleted, asInt(jhit->first));
            mJointAuthoringMap.erase(jhit);
            removedJoint = true;
            mHasSomeGizmoBeenDeleted = true;
        }

        JointDataMap::iterator jdit = mJointDataMap.find(path);
        if (jdit != mJointDataMap.end())
        {
            JointData& jointData = jdit->second;

            removeBillboard(jointData.assetId, jointData.billboardId);

            omni::usd::UsdContext::getContext()->freeGizmoUID(jointData.billboardUid);

            mJointDataMap.erase(jdit);

            // remove the joint path from the body map as well
            removeJointFromBodies(path);

            // and the authoring map
            removeJointPath(path);

            removedJoint = true;
        }
    }
    mPrimHierarchyStorage.removeIteration(iterator);
    return removedJoint;
}

void JointAuthoringManager::removeBillboard(uint32_t assetId, uint32_t& billboardId)
{
    const uint32_t swappedId = mDebugDraw->removeBillboard(assetId, billboardId);

    // A.B. This can easily become a bottleneck, might consider a better solution
    if (swappedId != kInvalidBillboardId)
    {
        bool found = false;
        for (JointDataMap::reference ref : mJointDataMap)
        {
            if (ref.second.assetId == assetId && ref.second.billboardId == swappedId)
            {
                ref.second.billboardId = billboardId;
                found = true;
                break;
            }
        }
        CARB_ASSERT(found);
    }

    billboardId = kInvalidBillboardId;
}

void JointAuthoringManager::addOrUpdateBillboard(uint32_t assetId, uint32_t& billboardId, uint32_t billboardUid, const pxr::GfMatrix4f& localTransform)
{
    if (billboardId == kInvalidBillboardId)
    {
        billboardId = mDebugDraw->addBillboard(assetId, billboardUid, localTransform.data());
    }
    else
    {
        mDebugDraw->updateBillboard(assetId, billboardId, localTransform.data());
    }
}


void JointAuthoringManager::resyncPrim(const pxr::SdfPath& path)
{
    if(mHidden)
        return; // We will parse stage from scratch when joint visualization will be re-enabled in setJointMeshesVisibilty

    if (mNotificationBlocked) // when notifications are blocked we cant mess with the state, bufferup
    {
        mBlockedResyncPrims.insert(path);
        return;
    }

    // if this prim already exists in the system, then we need to pump the selection changed code
    // after resyncing it to update the authoring map in case this prim is currently selected
    // because removing the prim removes it from the authoring map
    mUpdateSelection |= removePrim(path);
    mAddedPrims.insert(path);
}

void JointAuthoringManager::mapBodyToJoint(const pxr::SdfPath& bodyPath, const pxr::SdfPath& jointPath)
{
    if (bodyPath != SdfPath())
    {
        mJointBodyMap[bodyPath].insert(jointPath);
    }
}

void JointAuthoringManager::updateJointBodyPaths(const pxr::SdfPath& jointPath, bool body0)
{
    const uint32_t flag = body0 ? 1 : 2;
    JointBodyChange::iterator it = mJointBodyPaths.find(jointPath);
    if (it != mJointBodyPaths.end())
    {
        it->second |= flag;
    }
    else
    {
        mJointBodyPaths[jointPath] = flag;
    }
}

void JointAuthoringManager::clearBufferedUpdatesForJointBodyPaths(const pxr::SdfPath& jointPath)
{
    mJointBodyPaths[jointPath] = 0;
}

void JointAuthoringManager::updateJointBodyPathsInternal()
{
    UsdGeomXformCache xfCache;

    if (!mJointBodyPaths.empty() && mSelectedJointPrim != SdfPath())
    {
        const UsdPrim selectedPrim = mStage->GetPrimAtPath(mSelectedJointPrim);
        clearSelectionOutline();

        if (selectedPrim.IsValid() && selectedPrim.IsA<UsdPhysicsJoint>())
        {
            
            setupSelectionOutline(selectedPrim);
        }        
    }

    const uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();

    for (JointBodyChange::const_reference& ref : mJointBodyPaths)
    {
        const SdfPath& jointPath = ref.first;
        UsdPrim jointPrim = mStage->GetPrimAtPath(jointPath);

        // make sure this is a joint that we know about
        if (jointPrim && jointPrim.IsA<UsdPhysicsJoint>())
        {
            UsdPhysicsJoint physicsJoint(jointPrim);

            // clear out any existing entries with this joint path
            removeJointFromBodies(jointPath);

            // we have to pull the full description because body0/1 aren't stored, they are calculated on the fly
            // but we need to track them as well so we can update things if they move
            JointDesc* desc = mUsdPhysicsParse->parseJoint(stageId, jointPath);
            if (desc)
            {
                // body0 localPose update
                if (ref.second & 1)
                {
                    const GfMatrix4d source = getWorldSpaceTransform(mStage, desc, false);

                    const pxr::UsdAttribute localPosAttr = physicsJoint.GetLocalPos0Attr();
                    const pxr::UsdAttribute localRotAttr = physicsJoint.GetLocalRot0Attr();
                    SdfPathVector targets;
                    physicsJoint.GetBody0Rel().GetTargets(&targets);
                    if (!targets.empty())
                        calcLocalPoseMatrix(true, source, mStage, targets[0], localPosAttr, localRotAttr);
                    else
                        calcLocalPoseMatrix(true, source, mStage, SdfPath(), localPosAttr, localRotAttr);
                }
                if (ref.second & 2)
                {
                    const GfMatrix4d source = getWorldSpaceTransform(mStage, desc, true);

                    const pxr::UsdAttribute localPosAttr = physicsJoint.GetLocalPos1Attr();
                    const pxr::UsdAttribute localRotAttr = physicsJoint.GetLocalRot1Attr();
                    SdfPathVector targets;
                    physicsJoint.GetBody1Rel().GetTargets(&targets);
                    if (!targets.empty())
                        calcLocalPoseMatrix(true, source, mStage, targets[0], localPosAttr, localRotAttr);
                    else
                        calcLocalPoseMatrix(true, source, mStage, SdfPath(), localPosAttr, localRotAttr);
                }

                // then add in fresh entries with the updated paths
                mapBodyToJoint(desc->body0, jointPath);
                mapBodyToJoint(desc->body1, jointPath);
                mapBodyToJoint(desc->rel0, jointPath);
                mapBodyToJoint(desc->rel1, jointPath);

                mUsdPhysicsParse->releaseDesc(desc);
            }
        }
    }

    mJointBodyPaths.clear();
}

void JointAuthoringManager::resyncPath(const pxr::SdfPath& resyncPath)
{
    UsdGeomXformCache xfCache;

    const uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();
    const UsdPrim resyncPrim = mStage->GetPrimAtPath(resyncPath);
    pxr::UsdPrimRange range(resyncPrim);
    for (pxr::UsdPrimRange::iterator iter = range.begin(); iter != range.end(); ++iter)
    {
        pxr::UsdPrim jointPrim = *iter;
        if (jointPrim && jointPrim.IsA<UsdPhysicsJoint>())
        {            
            const SdfPath& jointPath = jointPrim.GetPrimPath();

            // store joint data, we care about scale for now
            JointData& jointData = mJointDataMap[jointPath];
            {
                const GfMatrix4d worldPose = xfCache.GetLocalToWorldTransform(jointPrim);
                const GfTransform gftr(worldPose);
                jointData.scale = gftr.GetScale();
            }

            JointDesc* desc = mUsdPhysicsParse->parseJoint(stageId, jointPath);
            if (desc)
            {
                mPrimHierarchyStorage.addPrim(jointPath);

                
                createJointHelper(jointPath, desc, jointData);
                desc = nullptr;
                // desc ownership is passed to createJointHelper
            }
            mJointTransformData.insert(jointPath);
        }
    }

    mGizmosDirty = true;
}

void JointAuthoringManager::createJointHelper(const pxr::SdfPath& jointPath, JointDesc* desc, JointData& jointData)
{
    CARB_PROFILE_ZONE(0, "JointAuthoringManager::createJointHelper");
    typedef omni::physics::schema::ObjectType::Enum ObjectTypeEnum;
    const ObjectTypeEnum type = (desc ? desc->type : ObjectType::eUndefined);

    uint32_t assetId = 0;

    switch (type)
    {
    case ObjectType::eJointRevolute:
        assetId = mBillboardAssetIds[JointType::eRevoluteJoint];
        break;
    case ObjectType::eJointSpherical:
        assetId = mBillboardAssetIds[JointType::eSphericalJoint];
        break;
    case ObjectType::eJointDistance:
        assetId = mBillboardAssetIds[JointType::eDistanceJoint];
        break;
    case ObjectType::eJointFixed:
        assetId = mBillboardAssetIds[JointType::eFixedJoint];
        break;
    case ObjectType::eJointPrismatic:
        assetId = mBillboardAssetIds[JointType::ePrismaticJoint];
        break;
    case ObjectType::eJointD6:
        assetId = mBillboardAssetIds[JointType::eD6Joint];
        break;
    default: 
    {
        // Default billboard
        assetId = mBillboardAssetIds[JointType::eJJoint];

        const auto stageID = pxr::UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();
        for (const auto& registration : registrations)
        {
            const auto& callbacks = registration.second.callbacks;
            if (callbacks.onJointGetCustomBillboard)
            {
               if(callbacks.onJointGetCustomBillboard(stageID, asInt(jointPath), assetId, callbacks.userData))
               {
                    // Some registered backend gave us the billboard for this one
                    break;
               }
            }
        }

        break;
    }
    }

    jointData.assetId = assetId;

    if (desc && !mHidden)
    {
        CARB_PROFILE_ZONE(0, "JointAuthoringManager::setTransform");

        mapBodyToJoint(desc->body0, jointPath);
        mapBodyToJoint(desc->body1, jointPath);
        mapBodyToJoint(desc->rel0, jointPath);
        mapBodyToJoint(desc->rel1, jointPath);


        const pxr::GfMatrix4d localTransform = getWorldSpaceTransform(mStage, desc, true);

        updateJointGizmoTransform(desc);
        // desc ownership is transferred to updateJointGizmoTransform 
        desc = nullptr;

        PXR_NS::GfMatrix4f tr({ 1.0f, 1.0f, 1.0f, 1.f });
        tr *= PXR_NS::GfMatrix4f(localTransform);

        jointData.currentGizmoMatrix = tr;
        pxr::GfMatrix4f scaleMatrix;
        scaleMatrix.SetScale(0.0f);

        tr = scaleMatrix * tr;

        if (jointData.billboardUid == kInvalidBillboardId)
        {
            uint32_t id = omni::usd::UsdContext::getContext()->getGizmoUID(jointPath);
            jointData.billboardUid = id + 1; // WTH?
        }

        if (jointData.billboardId == kInvalidBillboardId)
        {
            jointData.billboardId = mDebugDraw->addBillboard(assetId, jointData.billboardUid, tr.data());
        }

        mGizmosDirty = true;
    }    
}


// Public interface to update joint gizmo data
void JointAuthoringManager::updateJointGizmoTransformFromPath(const pxr::SdfPath& jointPath)
{
    JointDataMap::iterator fit = mJointDataMap.find(jointPath);
    if (fit != mJointDataMap.end())
    {
        mJointGizmoTransform.insert(jointPath);
    }
}

void JointAuthoringManager::updateJointGizmoTransformInternal()
{
    const uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();
    for (pxr::SdfPathSet::const_reference& ref : mJointGizmoTransform)
    {
        const SdfPath& primPath = ref;

        JointDesc* desc = mUsdPhysicsParse->parseJoint(stageId, primPath);
        if (desc)
        {
            updateJointGizmoTransform(desc);
            // desc ownership is passed to updateJointGizmoTransform
            desc = nullptr;
        }
    }

    mJointGizmoTransform.clear();
}


// Main entry point to update the joint gizmo associated with the prim at the given path
// Handles updating the pos and orient based on the current state of the attached bodies
void JointAuthoringManager::updateJointGizmoTransform(JointDesc* desc)
{
    pxr::UsdGeomXformCache xfCache;
    pxr::GfMatrix4f localTransform = GfMatrix4f(getWorldSpaceTransform(mStage, desc, true));

    // A.B. scale based on the object sizes
    pxr::GfMatrix4f scaleMatrix;
    scaleMatrix.SetScale(0.0f);

    // then update the joint gizmo mesh instance
    JointDataMap::iterator jdItr = mJointDataMap.find(GetJointPrimPath(desc));
    if (jdItr != mJointDataMap.end())
    {
        JointData& jointData = jdItr->second;        
        jointData.currentGizmoMatrix = localTransform;

        localTransform = scaleMatrix * localTransform;

        if (jointData.billboardUid == kInvalidBillboardId)
        {
            uint32_t id = omni::usd::UsdContext::getContext()->getGizmoUID(GetJointPrimPath(desc));
            jointData.billboardUid = id + 1; // WTH?
        }
        if (!mHidden)
        {
            // base icon
            addOrUpdateBillboard(jointData.assetId, jointData.billboardId, jointData.billboardUid, localTransform);
        }

        mGizmosDirty = true;
    }
}

void JointAuthoringManager::updateGizmosFadeOutScale()
{
    // A.B. this should go to a separate thread computation
    CARB_PROFILE_ZONE(0, "JointAuthoringManager::UpdateGizmoFadeOutScale");

    if (mJointDataMap.size() != mJointBillboardScaleTask.getInstanceMatrices().size())
    {
        mJointBillboardScaleTask.initialize(mCameraPos, mJointDataMap, mViewProjection, mViewInverse, mViewPortRect, mSettings, mStage);
        mJointBillboardScaleTask.execute(false);
    }

    const std::vector<GfMatrix4f>& instanceMatrices = mJointBillboardScaleTask.getInstanceMatrices();
    size_t i = 0;

    CARB_ASSERT(instanceMatrices.size() == mJointDataMap.size());

    for (JointDataMap::reference jointDataRef : mJointDataMap)
    {
        JointData& jointData = jointDataRef.second;
        if (!mHidden)
        {
            const GfMatrix4f instanceMatrix = instanceMatrices[i];

            if (jointData.billboardId != kInvalidBillboardId)
            {
                mDebugDraw->updateBillboard(jointData.assetId, jointData.billboardId, instanceMatrix.data());
            }
        }
        i++;
    }
}

// This is called when prims move
// It checks if they are part of a joint and updates the joint gizmo if they are
void JointAuthoringManager::updateJointTransformData(const pxr::SdfPath& primPathIn)
{    
    pxr::UsdPrimRange range(mStage->GetPrimAtPath(primPathIn));
    for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
    {
        if (!(*iter))
            continue;

        const SdfPath& primPath = (*iter).GetPrimPath();
            
        // check if we are interested in the path and buffer it
        JointBodyMap::const_iterator bodyItr = mJointBodyMap.find(primPath);
        if (bodyItr != mJointBodyMap.end())
        {
            mJointTransformData.insert(primPath);            
        }
       
        JointDataMap::const_iterator jIt = mJointDataMap.find(primPath);
        if (jIt != mJointDataMap.end())
        {
            mJointScaleData.insert(primPath);
        }
    }
}

void scaleLimit(const pxr::UsdAttribute& limitAttr, float scale)
{
    float limit;
    limitAttr.Get(&limit);
    if (isfinite(limit))
    {
        limit *= scale;
        limitAttr.Set(limit);
    }
}

void scalePositiveLimit(const pxr::UsdAttribute& limitAttr, float scale)
{
    float limit;
    limitAttr.Get(&limit);
    if (limit > 0.0f)
    {
        limit *= scale;
        limitAttr.Set(limit);
    }
}

int getAxis(const TfToken axisToken)
{
    return axisToken == UsdPhysicsTokens->x ? 0 : (axisToken == UsdPhysicsTokens->y) ? 1 : 2;
}

void JointAuthoringManager::updateJointScaleDataInternal()
{
    UsdGeomXformCache xfCache;
    const float epsilon = 1e-3f;

    for (pxr::SdfPathSet::const_reference& ref : mJointScaleData)
    {
        const SdfPath& jointPath = ref;

        UsdPrim jointPrim = mStage->GetPrimAtPath(jointPath);
        if (!jointPrim)
            continue;

        const GfMatrix4d worldPose = xfCache.GetLocalToWorldTransform(jointPrim);
        const GfTransform tr(worldPose);
        const GfVec3d scale = tr.GetScale();

        JointDataMap::iterator fit = mJointDataMap.find(jointPath);
        if (fit != mJointDataMap.end())
        {
            if (!GfIsClose(fit->second.scale, scale, epsilon))
            {
                // scale up the joint
                const GfVec3d ratio = GfCompDiv(scale, fit->second.scale);
                const bool scaleAll = abs(ratio[0] - ratio[1]) < epsilon ? (abs(ratio[1] - ratio[2]) < epsilon ? true : false) : false;
                bool jointHandled = false;
                if (jointPrim.IsA<UsdPhysicsPrismaticJoint>())
                {
                    UsdPhysicsPrismaticJoint prismaticJoint(jointPrim);
                    TfToken axisToken;
                    prismaticJoint.GetAxisAttr().Get(&axisToken);
                    const int axis = getAxis(axisToken);
                    scaleLimit(prismaticJoint.GetLowerLimitAttr(), float(ratio[axis]));
                    scaleLimit(prismaticJoint.GetUpperLimitAttr(), float(ratio[axis]));
                    jointHandled = true;
                }
                else if (scaleAll && jointPrim.IsA<UsdPhysicsDistanceJoint>())
                {
                    UsdPhysicsDistanceJoint distanceJoint(jointPrim);
                    scalePositiveLimit(distanceJoint.GetMinDistanceAttr(), float(ratio[0]));
                    scalePositiveLimit(distanceJoint.GetMaxDistanceAttr(), float(ratio[0]));
                    jointHandled = true;
                }

                if (!jointHandled)
                {
                    // Let's give it a try to custom registered joint authoring nodes to handle the scale
                    IUsdPhysicsUICustomJointAuthoring::JointScaleData jointScaleData;
                    jointScaleData.scaleRatio = {(float)ratio[0], (float)ratio[1], (float)ratio[2]};
                    jointHandled = notifyListenersAboutScale(asInt(jointPath), jointScaleData);
                }
                
                if (!jointHandled && jointPrim.IsA<UsdPhysicsJoint>())
                {
                    std::vector<TfToken> axisVector = {
                        UsdPhysicsTokens->transX, UsdPhysicsTokens->transY, UsdPhysicsTokens->transZ, UsdPhysicsTokens->distance
                    };

                    for (size_t i = 0; i < axisVector.size(); i++)
                    {
                        const TfToken& axisToken = axisVector[i];

                        const UsdPhysicsLimitAPI limitAPI = UsdPhysicsLimitAPI::Get(jointPrim, axisToken);
                        if (limitAPI)
                        {
                            if (i < 3)
                            {
                                scaleLimit(limitAPI.GetLowAttr(), float(ratio[i]));
                                scaleLimit(limitAPI.GetHighAttr(), float(ratio[i]));
                            }
                            else if (scaleAll)
                            {
                                scalePositiveLimit(limitAPI.GetLowAttr(), float(ratio[0]));
                                scalePositiveLimit(limitAPI.GetHighAttr(), float(ratio[0]));
                            }
                        }
                    }
                }

                fit->second.scale = scale;
            }
        }
        else
        {
            mJointDataMap[jointPath].scale = scale;
        }
    }

    mJointScaleData.clear();
}

void JointAuthoringManager::updateJointTransformDataInternal()
{
    const uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();
    for (pxr::SdfPathSet::const_reference& ref : mJointTransformData)
    {
        const SdfPath& primPath = ref;
        // see if a this prim is a body connected to a joint
        // have to check all known bodies to make sure we cover all hierarchies
        JointBodyMap::iterator bodyItr = mJointBodyMap.find(primPath);
        if (bodyItr != mJointBodyMap.end())
        {

            // walk over the joints that this body is part of and update the joint gizmos associated with them
            const pxr::SdfPathSet& jointPaths = bodyItr->second;
            for (auto jointPath : jointPaths)
            {
                mJointGizmoTransform.insert(jointPath);
            }
        }
    }

    mJointTransformData.clear();
}

void JointAuthoringManager::refreshAllGizmos()
{
    for (JointDataMap::reference jointDataRef : mJointDataMap)
    {
        const SdfPath& jointPath = jointDataRef.first;
        mJointGizmoTransform.insert(jointPath);
    }
    mGizmosDirty = true;
}

IUsdPhysicsUICustomJointAuthoring::RegistrationID  JointAuthoringManager::registerCustomJointAuthoring(IUsdPhysicsUICustomJointAuthoring& customJointAuthoring, const char* name)
{
    const auto newRegistrationID = static_cast<IUsdPhysicsUICustomJointAuthoring::RegistrationID>(lastRegistrationID);
    CustomJointAuthoring item;
    item.name = name;
    item.callbacks = customJointAuthoring;
    registrations[newRegistrationID] = std::move(item);
    lastRegistrationID++;
    return newRegistrationID;
}

bool JointAuthoringManager::unregisterCustomJointAuthoring (IUsdPhysicsUICustomJointAuthoring::RegistrationID registrationID)
{
    auto foundItem = registrations.find(registrationID);
    if (foundItem != registrations.end())
    {
        // remove all custom joint billboard
        for(const auto& jointBillboard : foundItem->second.jointBillboards)
        {            
            mDebugDraw->clearBillboardInstances(jointBillboard->billboardId);
        }
        registrations.erase(foundItem);
        return true;
    }
    return false;
}

uint32_t JointAuthoringManager::registerCustomJointBillboard (IUsdPhysicsUICustomJointAuthoring::RegistrationID registrationID, const char* pngPath)
{
    auto foundItem = registrations.find(registrationID);
    if (foundItem != registrations.end())
    {
        auto& registration = foundItem->second;
        auto jointBillboard = std::make_unique<CustomJointAuthoring::JointBillboard>(pngPath);
        jointBillboard->billboardId = mDebugDraw->addBillboardAsset(jointBillboard->path.c_str());
        registration.jointBillboards.emplace_back(std::move(jointBillboard));
        return registration.jointBillboards.back()->billboardId;
    }
    return 0;
}

void JointAuthoringManager::notifyListenersAbout(JointEvent event, uint64_t jointPath)
{
    const auto stageID = pxr::UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();
    for(const auto& registration : registrations)
    {
        const auto& callbacks = registration.second.callbacks;
        switch(event)
        {
            case JointEvent::JointCreated:
            {
                if(callbacks.onJointAuthoringCreate)
                {
                    callbacks.onJointAuthoringCreate(stageID, jointPath, callbacks.userData);
                }
            }
            break;
            case JointEvent::JointDeleted:
            {
                if(callbacks.onJointAuthoringDelete)
                {
                    callbacks.onJointAuthoringDelete(stageID, jointPath, callbacks.userData);
                }
            }
            break;
        }
    }
}

bool JointAuthoringManager::notifyListenersAboutScale(uint64_t jointPath, const IUsdPhysicsUICustomJointAuthoring::JointScaleData& jointScaleData)
{
    const auto stageID = pxr::UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();
    bool handled = false;
    for(const auto& registration : registrations)
    {
        const auto& callbacks = registration.second.callbacks;
        if(callbacks.onJointAuthoringScale)
        {
            handled |= callbacks.onJointAuthoringScale(stageID, jointPath, jointScaleData, callbacks.userData);
        }
    }
    return handled;
}


