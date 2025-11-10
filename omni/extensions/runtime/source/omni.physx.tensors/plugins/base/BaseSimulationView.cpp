// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
#include <pxr/base/tf/patternMatcher.h>
// clang-format on

#include "BaseSimulationView.h"
#include "BaseArticulationView.h"
#include "BaseRigidBodyView.h"
#include "BaseSdfShapeView.h"
#include "BaseSoftBodyView.h"
#include "BaseSoftBodyMaterialView.h"
#include "BaseVolumeDeformableBodyView.h"
#include "BaseSurfaceDeformableBodyView.h"
#include "BaseDeformableMaterialView.h"
#include "BaseRigidContactView.h"
#include "BaseParticleSystemView.h"
#include "BaseParticleClothView.h"
#include "BaseParticleMaterialView.h"

#include "../GlobalsAreBad.h"
#include "../SimulationBackend.h"

#include <carb/logging/Log.h>
#include <private/omni/physx/IPhysxPrivate.h>

#include <PxPhysicsAPI.h>
#include <physicsSchemaTools/physicsSchemaTokens.h>

#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>

#include <omni/physx/IPhysxFabric.h>
#include <omni/physx/IPhysxJoint.h>
#include <omni/physx/IPhysxReplicator.h>
#include <omni/physx/PhysxTokens.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/fabric/IFabric.h>
#include <omni/fabric/SimStageWithHistory.h>
#include <omni/timeline/TimelineTypes.h>


#include <omni/fabric/connectivity/Connectivity.h>
#include <omni/fabric/FabricUSD.h>
#include <usdrt/population/FabricTypes.h>
#include <usdrt/scenegraph/usd/usd/stage.h>


using namespace pxr;
using namespace physx;
using namespace omni::fabric;
using namespace carb;
using omni::physics::tensors::ObjectType;
const omni::fabric::IToken* omni::fabric::Token::iToken = nullptr;
const omni::fabric::IPath* omni::fabric::Path::iPath = nullptr;


namespace omni
{
namespace physx
{
namespace tensors
{

namespace
{

void findMatchingChildrenUsdRt(const usdrt::UsdPrim& root, const std::string& pattern, std::vector<usdrt::UsdPrim>& primsRet)
{
    if (!root)
    {
        return;
    }

    TfPatternMatcher matcher(pattern, true, true);
    std::vector<usdrt::UsdPrim> allChilds = root.GetAllChildren();
    for (auto child : allChilds)
    {
        if (matcher.Match(child.GetName().GetString()))
        {
            primsRet.push_back(child);
        }
    }
}

void findMatchingChildren(UsdPrim root, const std::string& pattern, std::vector<UsdPrim>& primsRet)
{
    if (!root)
    {
        return;
    }

    TfPatternMatcher matcher(pattern, true, true);
    UsdPrimSiblingRange range = root.GetAllChildren();
    for (auto child : range)
    {
        if (matcher.Match(child.GetName()))
        {
            primsRet.push_back(child);
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

bool findDeformableMeshPaths(SdfPath& simMeshPath,
                             SdfPath& collMeshPath,
                             SdfPathSet& skinGeomPaths,
                             const SdfPath actualDeformableBodyPath,
                             const UsdStageWeakPtr stage)
{
    simMeshPath = SdfPath();
    collMeshPath = SdfPath();
    skinGeomPaths.clear();

    TfType volumeSimType =
        UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->VolumeDeformableSimAPI);
    TfType surfaceSimType =
        UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->SurfaceDeformableSimAPI);
    if (!stage)
    {
        return false;
    }

    UsdPrim bodyPrim = stage->GetPrimAtPath(actualDeformableBodyPath);
    UsdPrimRange prims(bodyPrim, pxr::UsdPrimAllPrimsPredicate);
    for (pxr::UsdPrimRange::const_iterator it = prims.begin(); it != prims.end(); ++it)
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

        bool isSim = prim.HasAPI(volumeSimType) || prim.HasAPI(surfaceSimType);
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

} // namespace

BaseSimulationView::BaseSimulationView(UsdStageRefPtr stage)
    : mStage(stage), mSimData(std::make_shared<BaseSimulationData>())
{
#if 0
    {
        PxScene* scene = g_simBackend.findPhysicsScene();
        if (scene)
        {
            PxU32 numSbs = scene->getNbSoftBodies();
            std::vector<PxSoftBody*> sbs(numSbs);
            scene->getSoftBodies(sbs.data(), numSbs);
            for (PxU32 i = 0; i < numSbs; i++)
            {
                PxShape* shape = sbs[i]->getShape();
                printf("Shape %u\n", i);
                printf("  Shape type: '%s'\n", shape->getConcreteTypeName());
                printf("  Contact offset: %f\n", shape->getContactOffset());
                printf("  Rest offset:    %f\n", shape->getRestOffset());
                // HAAAACK!!!!
                //shape->setRestOffset(0.0f);
                //shape->setContactOffset(0.02f);
            }
        }
    }
#endif

    omni::physx::IPhysicsObjectChangeCallback callback;
    callback.objectDestructionNotifyFn = onPhysXObjectDeletedCallback;
    callback.allObjectsDestructionNotifyFn = onAllPhysXObjectDeletedCallback;
    callback.userData = this;
    subscriptionObjId = g_physx->subscribeObjectChangeNotifications(callback);
    mTimeline = omni::timeline::getTimeline();
    mTimelineEvtSub = carb::events::createSubscriptionToPop(
        mTimeline->getTimelineEventStream(),
        [this](carb::events::IEvent* e) {
            if (static_cast<omni::timeline::TimelineEventType>(e->type) == omni::timeline::TimelineEventType::eStop)
            {
                this->invalidate();
            }
        },
        0, "BaseSimulationView::BaseSimulationView");


    // set up fabric for updating kinematic transforms
    omni::fabric::IToken* iToken = carb::getCachedInterface<omni::fabric::IToken>();

    mWorldMatrixToken = iToken->getHandle(gWorldMatrixTokenString);
    mDynamicBodyToken = iToken->getHandle(gDynamicBodyTokenString);
    mLocalMatrixToken = iToken->getHandle(gLocalMatrixTokenString);
    mRigidBodyWorldPositionToken = iToken->getHandle(gRigidBodyWorldPositionTokenString);
    mRigidBodyWorldOrientationToken = iToken->getHandle(gRigidBodyWorldOrientationTokenString);
    mRigidBodyWorldScaleToken = iToken->getHandle(gRigidBodyWorldScaleTokenString);

}

void BaseSimulationView::invalidate()
{
    std::lock_guard<std::mutex> guard(mMutex);
    if (isValid)
    {
        if (mTimelineEvtSub)
        {
            mTimelineEvtSub->unsubscribe();
            mTimelineEvtSub = nullptr;
        }
        isValid = false;
        // Can't reset the backend here, as it will cause a deadlock since the SimulationBackend reset calls SimulationView invalidate function
        // GetSimulationBackend().reset();
        // only Reset the stage here if using shared_ptr i.e. TfRefPtr<UsdStage>
        // mStage.Reset();
        // Alternative is to use TfweakPtr<UsdStage> to avoid increasing the reference count
    }
}


void BaseSimulationView::InitializeKinematicBodies()
{
    if (fabricKinematicsInitialized)
    {
        CARB_LOG_WARN("Fabric Kinematics already initialized");
        return;
    }
 
 
    IStageReaderWriter* iSip = carb::getCachedInterface<IStageReaderWriter>();
    long stageID = omni::usd::UsdContext::getContext()->getStageId();
    StageReaderWriter stageRW = iSip->get(stageID);    

    omni::fabric::Type double3Type = omni::fabric::Type(
        omni::fabric::BaseDataType::eDouble, 3, 0, omni::fabric::AttributeRole::eNone);
    omni::fabric::Type float3Type = omni::fabric::Type(
        omni::fabric::BaseDataType::eFloat, 3, 0, omni::fabric::AttributeRole::eNone);
    omni::fabric::Type quatType = omni::fabric::Type(
        omni::fabric::BaseDataType::eFloat, 4, 0, omni::fabric::AttributeRole::eQuaternion);
    omni::fabric::Type mMatrix4dType = omni::fabric::Type(
        BaseDataType::eDouble, 16, 0, omni::fabric::AttributeRole::eMatrix);
        
    // apply transform attributes to fabric for kinematic bodies
    pxr::UsdPrimRange primRange = mStage->Traverse();
    pxr::UsdGeomXformCache xfCache;
    for (pxr::UsdPrimRange::const_iterator iter = primRange.begin(); iter != primRange.end(); ++iter)
    {
        const pxr::UsdPrim& prim = *iter;
        if (!prim)
            continue;

        const pxr::SdfPath path = prim.GetPrimPath();
        if (prim.HasAPI<UsdPhysicsRigidBodyAPI>())
        {
            UsdPhysicsRigidBodyAPI rboAPI(prim);
            if (rboAPI)
            {
                bool kinematic = false;
                rboAPI.GetKinematicEnabledAttr().Get(&kinematic);
                if (kinematic)
                {
                    const omni::fabric::PathC primPath = omni::fabric::asInt(prim.GetPrimPath());

                    std::array<omni::fabric::AttrNameAndType, 5> attrNameTypeVec = {
                        omni::fabric::AttrNameAndType(mMatrix4dType, mWorldMatrixToken),
                        omni::fabric::AttrNameAndType(mMatrix4dType, mLocalMatrixToken),
                        omni::fabric::AttrNameAndType(double3Type, mRigidBodyWorldPositionToken),
                        omni::fabric::AttrNameAndType(quatType, mRigidBodyWorldOrientationToken),
                        omni::fabric::AttrNameAndType(float3Type, mRigidBodyWorldScaleToken),
                    };

                    stageRW.createAttributes(primPath, attrNameTypeVec);

                    const GfMatrix4d worldPose = xfCache.GetLocalToWorldTransform(prim);
                    const GfTransform tr(worldPose);

                    carb::Double3& posData =
                        *(carb::Double3*)(iSip->getAttributeWr(stageRW.getId(), primPath, mRigidBodyWorldPositionToken)).ptr;
                    carb::Float4& orientData =
                        *(carb::Float4*)(iSip->getAttributeWr(stageRW.getId(), primPath, mRigidBodyWorldOrientationToken)).ptr;
                    carb::Float3& scaleData =
                        *(carb::Float3*)(iSip->getAttributeWr(stageRW.getId(), primPath, mRigidBodyWorldScaleToken)).ptr;
                    pxr::GfMatrix4d& wMatData =
                        *(pxr::GfMatrix4d*)(iSip->getAttributeWr(stageRW.getId(), primPath, mWorldMatrixToken)).ptr;
                    pxr::GfMatrix4d& lMatData =
                        *(pxr::GfMatrix4d*)(iSip->getAttributeWr(stageRW.getId(), primPath, mLocalMatrixToken)).ptr;

                    const GfVec3d wPos = tr.GetTranslation();
                    posData.x = wPos[0];
                    posData.y = wPos[1];
                    posData.z = wPos[2];

                    const GfQuatd wRot = tr.GetRotation().GetQuat();
                    orientData.x = float(wRot.GetImaginary()[0]);
                    orientData.y = float(wRot.GetImaginary()[1]);
                    orientData.z = float(wRot.GetImaginary()[2]);
                    orientData.w = float(wRot.GetReal());

                    const GfVec3f wScale = GfVec3f(tr.GetScale());

                    scaleData.x = wScale[0];
                    scaleData.y = wScale[1];
                    scaleData.z = wScale[2];


                    omni::fabric::USDHierarchy usdHierarchy(stageRW.getFabricId());
                    pxr::GfMatrix4d localPose(1.0);
                    omni::fabric::PathC parentPath = usdHierarchy.getParent(primPath);
                    while (parentPath != omni::fabric::kUninitializedPath)
                    {
                        pxr::GfMatrix4d* parentWorldMatrix = (pxr::GfMatrix4d*)(iSip->getAttributeWr(stageRW.getId(), parentPath, mWorldMatrixToken)).ptr;
                        if (parentWorldMatrix)
                        {
                            localPose = worldPose * parentWorldMatrix->GetInverse();
                            break;
                        }
                        else
                        {
                            parentPath = usdHierarchy.getParent(parentPath);
                        }
                    }
                    wMatData = worldPose;
                    lMatData = localPose;
                }
            }
        }
    }
    fabricKinematicsInitialized = true;
}


BaseSimulationView::~BaseSimulationView()
{
    g_physx->unsubscribeObjectChangeNotifications(subscriptionObjId);
    invalidate();
    GetSimulationBackend().removeSimulationView(this);

    for (auto artiView : mArtiViews)
    {
        artiView->_onParentRelease();
    }

    for (auto rbView : mRbViews)
    {
        rbView->_onParentRelease();
    }

    for (auto sbView : mSbViews)
    {
        sbView->_onParentRelease();
    }
    
    for (auto sbMaterialView: mSbMaterialViews)
    {
        sbMaterialView->_onParentRelease();
    }

    for (auto vdbView : mVolumeDeformableBodyViews)
    {
        vdbView->_onParentRelease();
    }

    for (auto sdbView : mSurfaceDeformableBodyViews)
    {
        sdbView->_onParentRelease();
    }
    
    for (auto dMaterialView : mDeformableMaterialViews)
    {
        dMaterialView->_onParentRelease();
    }

    for (auto rcView : mRcViews)
    {
        rcView->_onParentRelease();
    }

    for (auto psView: mParticleSysViews)
    {
        psView->_onParentRelease();
    }

    for (auto pcView: mParticleClothViews)
    {
        pcView->_onParentRelease();
    }

    for (auto pmView: mParticleMatViews)
    {
        pmView->_onParentRelease();
    }

    for (auto sdfView : mSDFViews)
    {
        sdfView->_onParentRelease();
    }
}

void BaseSimulationView::onPhysXObjectDeletedCallback(const pxr::SdfPath& sdfPath,
                                                      usdparser::ObjectId objectId,
                                                      PhysXType type,
                                                      void* userData)
{
    auto sim = (BaseSimulationView*)userData;
    omni::physx::IPhysx* physx = omni::physx::tensors::g_physx;
    if (physx && sim)
    {
        // no need to check further and print warning if the sim is already invalidated
        if (!sim->getValid())
            return;

        if (type == ePTActor)
        {
            // PxActor* actor = static_cast<PxActor*>(g_physx->getPhysXPtr(path, omni::physx::ePTActor));
            PxRigidBody* actor = static_cast<PxRigidBody*>(physx->getPhysXPtrFast(objectId));
            if (sim->hasRigidBody(actor))
            {
                CARB_LOG_WARN(
                    "prim '%s' was deleted while being used by a tensor view class. The physics.tensors simulationView was invalidated.",
                    sdfPath.GetText());
                sim->invalidate();
            }
        }
        if (type == ePTArticulation)
        {
            auto arti = static_cast<PxArticulationReducedCoordinate*>(physx->getPhysXPtrFast(objectId));
            if (sim->hasArticulation(arti))
            {
                CARB_LOG_WARN(
                    "prim '%s' was deleted while being used by a tensor view class. The physics.tensors simulationView was invalidated.",
                    sdfPath.GetText());
                sim->invalidate();
            }
        }
        if (type == ePTLink)
        {
            auto link = static_cast<PxArticulationLink*>(physx->getPhysXPtrFast(objectId));
            if (sim->hasLink(link))
            {
                CARB_LOG_WARN(
                    "prim '%s' was deleted while being used by a link in a tensor view class. The physics.tensors simulationView was invalidated.",
                    sdfPath.GetText());
                sim->invalidate();
            }
        }
        if (type == ePTShape)
        {
            auto shape = static_cast<PxShape*>(physx->getPhysXPtrFast(objectId));
            if (sim->hasShape(shape))
            {
                CARB_LOG_WARN(
                    "prim '%s' was deleted while being used by a shape in a tensor view class. The physics.tensors simulationView was invalidated.",
                    sdfPath.GetText());
                sim->invalidate();
            }
        }
    }
}

void BaseSimulationView::onAllPhysXObjectDeletedCallback(void* userData)
{
    auto sim = (BaseSimulationView*)userData;
    if (sim)
    {
        // no need to check further and print warning if the sim is already invalidated
        if (!sim->getValid())
            return;

        CARB_LOG_WARN(
            "All physics information was deleted while being used by a tensor view class. The physics.tensors simulationView was invalidated.");
        sim->invalidate();
    }
}

bool BaseSimulationView::setSubspaceRoots(const char* pattern)
{
    if (!pattern || !*pattern)
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return false;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern);
        return false;
    }

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    for (unsigned i = 0; i < paths.size(); i++)
    {
        UsdPrim prim = mStage->GetPrimAtPath(paths[i]);
        UsdGeomXformable xf(prim);
        if (xf)
        {
            GfMatrix4d localToWorld = xf.ComputeLocalToWorldTransform(UsdTimeCode::Default());
            GfVec3d tran = GfTransform(localToWorld).GetTranslation();

            Subspace subspace;
            subspace.origin = { float(tran[0]), float(tran[1]), float(tran[2]) };

            // printf("Subspace '%s' origin (%g, %g, %g)\n", paths[i].GetString().c_str(),
            //    subspace->origin.x, subspace->origin.y, subspace->origin.z);

            mSimData->mSubspaces[paths[i]] = subspace;
        }
    }

    return true;
}

Subspace* BaseSimulationView::findSubspaceForPath(const pxr::SdfPath& path) const
{
    for (auto& entry : mSimData->mSubspaces)
    {
        if (path.HasPrefix(entry.first))
        {
            return &entry.second;
        }
    }
    return nullptr;
}

void BaseSimulationView::findMatchingPaths(const std::string& pattern_, std::vector<SdfPath>& pathsRet)
{
    if (!mStage)
    {
        return;
    }

    long stageId = UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();
    bool useUsdrt = false;

    omni::physx::IPhysxReplicator* replicator = carb::getCachedInterface<omni::physx::IPhysxReplicator>();
    
    if (replicator)
    {
        bool replStage = false;
        replicator->isReplicatorStage(stageId, replStage, useUsdrt);
    }

    if (useUsdrt)
    {
        usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(omni::fabric::UsdStageId(stageId));


        // First try if the pattern_ is an actual prim path itself; if so, do not try to pattern match for performance reasons
        // The use case for this is when the user provides a list of actual prim paths rather than patterns. 
        // Doing pattern matching for such cases doesn't make sense and is very slow
        if (pxr::SdfPath::IsValidPathString(pattern_))
        {
            if (usdrtStage->GetPrimAtPath(usdrt::SdfPath(pattern_)))
            {
                pathsRet.push_back(SdfPath(pattern_));
                return;
            }
        }

        std::string pattern = TfStringTrim(pattern_, "/");
        std::vector<std::string> tokens = TfStringSplit(pattern, "/");

        // need to wrap the token patterns in '^' and '$' to prevent matching anywhere in the string
        for (std::string& tok : tokens)
        {
            tok = '^' + tok + '$';
        }

        std::vector<usdrt::UsdPrim> roots;
        std::vector<usdrt::UsdPrim> matches;

        roots.push_back(usdrtStage->GetPseudoRoot());

        int numTokens = int(tokens.size());

        for (int i = 0; i < numTokens; i++)
        {
            for (auto& prim : roots)
            {
                findMatchingChildrenUsdRt(prim, tokens[i], matches);
            }

            if (i < numTokens - 1)
            {
                std::swap(roots, matches);
                matches.clear();
            }
        }
        
        for (auto& prim : matches)
        {
            usdrt::SdfPath path = prim.GetPath();
            omni::fabric::PathC fabricPath = omni::fabric::PathC(path); // convert usdrt::SdfPath to omni::fabric::PathC
            const pxr::SdfPath& sdfPath = omni::fabric::toSdfPath(fabricPath);
            pathsRet.push_back(sdfPath);
            //printf("%s\n", path.GetText());
        }    
    }
    else
    {
        // First try if the pattern_ is an actual prim path itself; if so, do not try to pattern match for performance reasons
        // The use case for this is when the user provides a list of actual prim paths rather than patterns. 
        // Doing pattern matching for such cases doesn't make sense and is very slow
        if (pxr::SdfPath::IsValidPathString(pattern_))
        {
            if (mStage->GetPrimAtPath(SdfPath(pattern_)))
            {
                pathsRet.push_back(SdfPath(pattern_));
                return;
            }
        }

        std::string pattern = TfStringTrim(pattern_, "/");
        std::vector<std::string> tokens = TfStringSplit(pattern, "/");

        // need to wrap the token patterns in '^' and '$' to prevent matching anywhere in the string
        for (std::string& tok : tokens)
        {
            tok = '^' + tok + '$';
        }

        std::vector<UsdPrim> roots;
        std::vector<UsdPrim> matches;

        roots.push_back(mStage->GetPseudoRoot());

        int numTokens = int(tokens.size());

        for (int i = 0; i < numTokens; i++)
        {
            for (auto& prim : roots)
            {
                findMatchingChildren(prim, tokens[i], matches);
            }

            if (i < numTokens - 1)
            {
                std::swap(roots, matches);
                matches.clear();
            }
        }

        for (auto& prim : matches)
        {
            pathsRet.push_back(prim.GetPath());
        }
    }        
}


void BaseSimulationView::processArticulationEntries(const std::vector<std::string>& patterns,
                                                    std::vector<ArticulationEntry>& entries)
{
    for (const auto& pattern : patterns)
    {
        size_t currentSize = entries.size();
        findMatchingArticulations(pattern, entries);
        if (entries.size() == currentSize)
        {
            CARB_LOG_ERROR("Pattern '%s' did not match any rigid bodies\n", pattern.c_str());
        }
    }
}
void BaseSimulationView::findMatchingArticulations(const std::string& pattern, std::vector<ArticulationEntry>& entriesRet)
{
    if (!mStage)
    {
        return;
    }
    if (pattern.empty())
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return;
    }
    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    /*
    printf("Paths matching '%s'\n", pattern.c_str());
    for (auto& p : paths)
    {
        printf("  %s\n", p.GetString().c_str());
    }
    */

    // artisRet.clear();

    for (unsigned i = 0; i < paths.size(); i++)
    {
        ArticulationEntry entry;
        if (getArticulationAtPath(paths[i], entry))
        {
            entriesRet.push_back(entry);
        }
    }

    //printf("Found %d articulations matching '%s'\n", int(entriesRet.size()), pattern.c_str());
    /*
    for (unsigned i = 0; i < entriesRet.size(); i++)
    {
        auto& entry = entriesRet[i];
        printf("  %s: Articulation @ %p\n", entry.path.GetString().c_str(), entry.arti);
    }
    */
}

void BaseSimulationView::processRigidBodyEntries(const std::vector<std::string>& patterns,
                                                 std::vector<RigidBodyEntry>& entries)
{
    ;
    for (const auto& pattern : patterns)
    {
        size_t currentSize = entries.size();
        findMatchingRigidBodies(pattern, entries);
        if (entries.size() == currentSize)
        {
            CARB_LOG_ERROR("Pattern '%s' did not match any rigid bodies\n", pattern.c_str());
        }
    }
}

void BaseSimulationView::findMatchingRigidBodies(const std::string& pattern, std::vector<RigidBodyEntry>& entriesRet)
{
    if (!mStage)
    {
        return;
    }
    if (pattern.empty())
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return;
    }
    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    /*
    printf("Paths matching '%s'\n", pattern.c_str());
    for (auto& p : paths)
    {
        printf("  %s\n", p.GetString().c_str());
    }
    */

    // entriesRet.clear();

    for (unsigned i = 0; i < paths.size(); i++)
    {
        RigidBodyEntry entry;
        if (getRigidBodyAtPath(paths[i], entry))
        {
            entriesRet.push_back(entry);
        }
    }

    //printf("Found %d rigid bodies matching '%s'\n", int(entriesRet.size()), pattern.c_str());
    /*
    for (unsigned i = 0; i < entriesRet.size(); i++)
    {
        auto& entry = entriesRet[i];
        printf("  %s: %s @ %p\n", entry.path.GetString().c_str(), entry.body->getConcreteTypeName(), entry.body);
    }
    */
}

void BaseSimulationView::findMatchingSoftBodies(const std::string& pattern, std::vector<SoftBodyEntry>& entriesRet)
{
    if (!mStage)
    {
        return;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    /*
    printf("Paths matching '%s'\n", pattern.c_str());
    for (auto& p : paths)
    {
        printf("  %s\n", p.GetString().c_str());
    }
    */

    // entriesRet.clear();

    for (unsigned i = 0; i < paths.size(); i++)
    {
        SoftBodyEntry entry;
        if (getSoftBodyAtPath(paths[i], entry))
        {
            entriesRet.push_back(entry);
        }
    }

    //printf("Found %d soft bodies matching '%s'\n", int(entriesRet.size()), pattern.c_str());
    /*
    for (unsigned i = 0; i < entriesRet.size(); i++)
    {
        auto& entry = entriesRet[i];
        printf("  %s: %s @ %p\n", entry.path.GetString().c_str(), entry.body->getConcreteTypeName(), entry.body);
    }
    */
}

void BaseSimulationView::findMatchingSoftBodyMaterials(const std::string& pattern, std::vector<SoftBodyMaterialEntry>& entriesRet)
{
    if (!mStage)
    {
        return;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    _hackFixMaterialConnectivity();

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    // entriesRet.clear();

    for (unsigned i = 0; i < paths.size(); i++)
    {
        SoftBodyMaterialEntry entry;
        if (getSoftBodyMaterialAtPath(paths[i], entry))
        {
            entriesRet.push_back(entry);
        }
    }
}

void BaseSimulationView::findMatchingVolumeDeformableBodies(const std::string& pattern, std::vector<DeformableBodyEntry>& entriesRet)
{
    if (!mStage)
    {
        return;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    for (unsigned i = 0; i < paths.size(); i++)
    {
        DeformableBodyEntry entry;
        if (getVolumeDeformableBodyAtPath(paths[i], entry))
        {
            entriesRet.push_back(entry);
        }
    }
}

void BaseSimulationView::findMatchingSurfaceDeformableBodies(const std::string& pattern,
                                                             std::vector<DeformableBodyEntry>& entriesRet)
{
    if (!mStage)
    {
        return;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    for (unsigned i = 0; i < paths.size(); i++)
    {
        DeformableBodyEntry entry;
        if (getSurfaceDeformableBodyAtPath(paths[i], entry))
        {
            entriesRet.push_back(entry);
        }
    }
}

void BaseSimulationView::findMatchingDeformableMaterials(const std::string& pattern, std::vector<DeformableMaterialEntry>& entriesRet)
{
    if (!mStage)
    {
        return;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    _hackFixMaterialConnectivity();

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    // entriesRet.clear();

    for (unsigned i = 0; i < paths.size(); i++)
    {
        DeformableMaterialEntry entry;
        if (getDeformableMaterialAtPath(paths[i], entry))
        {
            entriesRet.push_back(entry);
        }
    }
}

void BaseSimulationView::processRigidContactViewEntries(const std::vector<std::string>& patterns,
                                                         const std::vector<std::vector<std::string>>& _filterPatterns,
                                                         std::vector<RigidContactSensorEntry>& entries,
                                                         uint32_t& filterPatternSize)
{
    if (patterns.empty())
    {
        CARB_LOG_ERROR("Empty patterns not allowed");
        return;
    }
    std::vector<std::vector<std::string>> filterPatterns;
    if (patterns.size() != _filterPatterns.size())
    {
        // No filter pattern specified, pass an empty filter pattern list for each sub pattern
        if (_filterPatterns.empty())
        {
            filterPatterns.resize(patterns.size());
        }
        else
        {
            CARB_LOG_ERROR("Size of the filter pattern list must match the size of the sensor pattern list");
            return;
        }
    }
    else
    {
        filterPatterns = _filterPatterns;
    }

    filterPatternSize = 0;
    for (size_t i = 0; i < patterns.size(); ++i)
    {
        size_t currentSize = entries.size();
        findMatchingRigidContactSensors(patterns[i], filterPatterns[i], entries);
        if (entries.size() == currentSize)
        {
            CARB_LOG_ERROR("Pattern '%s' did not match any rigid contact for filters\n", patterns[i].c_str());
            for (size_t j = 0; j < filterPatterns[i].size(); ++j)
            {
                CARB_LOG_ERROR("%s", filterPatterns[i][j].c_str());
            }
        }
        filterPatternSize = uint32_t(filterPatterns[i].size());
        if (i > 0 && filterPatterns[i].size() != filterPatterns[i - 1].size())
        {
            CARB_LOG_ERROR(
                "Number of all filter patterns for sensors should be equal. Sensor %s has %d filters while sensor %s has %d filters\n",
                patterns[i].c_str(), int(filterPatterns[i].size()), patterns[i - 1].c_str(),
                int(filterPatterns[i - 1].size()));
        }
    }
}
void BaseSimulationView::findMatchingRigidContactSensors(const std::string& pattern,
                                                         const std::vector<std::string>& filterPatterns,
                                                         std::vector<RigidContactSensorEntry>& entriesRet)
{
    if (!mStage)
    {
        return;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }
    if (pattern.empty())
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return;
    }
    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    uint32_t numFilterPatterns = uint32_t(filterPatterns.size());
    std::vector<std::vector<SdfPath>> filterPaths(numFilterPatterns);
    for (uint32_t i = 0; i < numFilterPatterns; i++)
    {
        const std::string& filterPattern = filterPatterns[i];

        if (filterPattern[0] != '/')
        {
            CARB_LOG_ERROR("Pattern must be an absolute USD path, got filter pattern '%s'\n", filterPattern.c_str());
            filterPaths[i].resize(paths.size());
            continue;
        }

        findMatchingPaths(filterPattern, filterPaths[i]);

        // Special case: if only a single match is found, then assume all sensors should report contacts with a single
        // object, like a common ground plane.
        if (filterPaths[i].size() == 1)
        {
            filterPaths[i].resize(paths.size(), filterPaths[i][0]);
        }
        else if (filterPaths[i].size() != paths.size())
        {
            CARB_LOG_ERROR("Filter pattern '%s' did not match the correct number of entries (expected %u, found %u)",
                filterPattern.c_str(), unsigned(paths.size()), unsigned(filterPaths[i].size()));
            filterPaths[i].clear();
            filterPaths[i].resize(paths.size());
        }
    }

    for (unsigned i = 0; i < paths.size(); i++)
    {
        RigidContactSensorEntry entry;
        if (getRigidContactSensorAtPath(paths[i], entry))
        {
            entriesRet.push_back(entry);

            // add contact filter mappings
            auto& e = entriesRet.back();
            for (uint32_t j = 0; j < numFilterPatterns; j++)
            {
                e.filterPaths.push_back(filterPaths[j][i]);
                if (!filterPaths[j][i].IsEmpty())
                {
                    uint64_t pathId = asInt(filterPaths[j][i]);
                    e.filterIndexMap[pathId] = j;
                }
            }
        }
    }

    //printf("Found %d rigid contact sensors matching '%s'\n", int(entriesRet.size()), pattern.c_str());
    /*
    for (unsigned i = 0; i < entriesRet.size(); i++)
    {
        auto& entry = entriesRet[i];
        printf("  %s: %s @ %p\n", entry.path.GetString().c_str(), entry.body->getConcreteTypeName(), entry.body);
    }
    */
}

void BaseSimulationView::findMatchingParticleSystems(const std::string& pattern, std::vector<ParticleSystemEntry>& entriesRet)
{
    if (!mStage)
    {
        return;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    for (unsigned i = 0; i < paths.size(); i++)
    {
        ParticleSystemEntry entry;
        if (getParticleSystemAtPath(paths[i], entry))
        {
            entriesRet.push_back(entry);
        }
    }
}

void BaseSimulationView::findMatchingParticleCloths(const std::string& pattern, std::vector<ParticleClothEntry>& entriesRet)
{
    if (!mStage)
    {
        return;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    for (unsigned i = 0; i < paths.size(); i++)
    {
        ParticleClothEntry entry;
        if (getParticleClothAtPath(paths[i], entry))
        {
            entriesRet.push_back(entry);
        }
    }
}

void BaseSimulationView::findMatchingParticleMaterials(const std::string& pattern, std::vector<ParticleMaterialEntry>& entriesRet)
{
    if (!mStage)
    {
        return;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    _hackFixMaterialConnectivity();

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    for (unsigned i = 0; i < paths.size(); i++)
    {
        ParticleMaterialEntry entry;
        if (getParticleMaterialAtPath(paths[i], entry))
        {
            entriesRet.push_back(entry);
        }
    }
}
void BaseSimulationView::findMatchingSDFShapes(const std::string& pattern, std::vector<SdfShapeEntry>& entriesRet, uint32_t numSamplePoints)
{
    if (!mStage)
    {
        return;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    for (unsigned i = 0; i < paths.size(); i++)
    {
        SdfShapeEntry entry;
        if (getSDFShapeAtPath(paths[i], entry))
        {
            entry.numSamplePoints = numSamplePoints;
            entriesRet.push_back(entry);
        }
    }
}

ObjectType BaseSimulationView::getObjectType(const char* path)
{
    if (!g_physx)
    {
        return ObjectType::eInvalid;
    }
    if (pxr::SdfPath::IsValidPathString(path))
    {
        SdfPath sdfPath = SdfPath(path);

        UsdPrim prim = mStage->GetPrimAtPath(sdfPath);
        usdrt::UsdPrim usdRtPrim;

        if (!prim)
        {
            long stageId = UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();    
            usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(omni::fabric::UsdStageId(stageId));

            usdRtPrim = usdrtStage->GetPrimAtPath(usdrt::SdfPath(path));
        }

        if (prim || usdRtPrim)
        {
            PxArticulationReducedCoordinate* arti = nullptr;
            // check if it's an articulation link
            PxArticulationLink* link = (PxArticulationLink*)g_physx->getPhysXPtr(sdfPath, omni::physx::ePTLink);
            if (link)
            {
                arti = &static_cast<PxArticulationReducedCoordinate&>(link->getArticulation());
                if (arti)
                {
                    PxU32 numLinks = arti->getNbLinks();
                    if (numLinks > 0)
                    {
                        const PxU32 linkIndex = link->getLinkIndex();
                        if (linkIndex == 0)
                            return ObjectType::eArticulationRootLink;
                        else
                            return ObjectType::eArticulationLink;
            
                    }
                }
            }
            // check if it's an articulation but not a link, i.e.
            arti = (PxArticulationReducedCoordinate*)g_physx->getPhysXPtr(sdfPath, omni::physx::ePTArticulation);
            if (arti)
            {
                return ObjectType::eArticulation;
            }

            // check if it's an articulation joint
            PxArticulationJointReducedCoordinate* joint =
                (PxArticulationJointReducedCoordinate*)g_physx->getPhysXPtr(sdfPath, omni::physx::ePTLinkJoint);
            if (joint)
            {
                return ObjectType::eArticulationJoint;
            }

            PxActor* actor = static_cast<PxActor*>(g_physx->getPhysXPtr(sdfPath, omni::physx::ePTActor));
            if (actor)
            {
                // check if it's a rigid dynamic
                if (actor->getType() == PxActorType::eRIGID_DYNAMIC)
                {
                    return ObjectType::eRigidBody;
                }
            }
        }
    }
    return ObjectType::eInvalid;
}

bool BaseSimulationView::getArticulationAtPath(const SdfPath& path, ArticulationEntry& entryRet)
{
    if (!g_physx)
    {
        return false;
    }
    // check if it's an articulation
    PxArticulationReducedCoordinate* arti =
        (PxArticulationReducedCoordinate*)g_physx->getPhysXPtr(path, omni::physx::ePTArticulation);
    if (arti)
    {
        // printf("Got %s at %p\n", arti->getConcreteTypeName(), (void*)arti);
    }
    else
    {
        // check if it's an articulation link
        PxArticulationLink* link = (PxArticulationLink*)g_physx->getPhysXPtr(path, omni::physx::ePTLink);
        if (link)
        {
            // printf("Got %s at %p\n", link->getConcreteTypeName(), (void*)link);
            arti = &static_cast<PxArticulationReducedCoordinate&>(link->getArticulation());
        }
        else
        {
            // check if it's an articulation joint
            PxArticulationJointReducedCoordinate* joint =
                (PxArticulationJointReducedCoordinate*)g_physx->getPhysXPtr(path, omni::physx::ePTLinkJoint);
            if (joint)
            {
                // printf("Got %s at %p\n", joint->getConcreteTypeName(), (void*)joint);
                arti =
                    &static_cast<PxArticulationReducedCoordinate&>(joint->getChildArticulationLink().getArticulation());
            }
        }
    }

    if (!arti || arti->getConcreteType() != PxConcreteType::eARTICULATION_REDUCED_COORDINATE)
    {
        CARB_LOG_WARN("Failed to find articulation at '%s'", path.GetString().c_str());
        return false;
    }

    // TODO: handle this better
    if (!arti->getScene())
    {
        CARB_LOG_ERROR("Articulation is not in a scene!");
        return false;
    }

    // get links
    PxU32 numLinks = arti->getNbLinks();
    std::vector<PxArticulationLink*> links(numLinks);
    arti->getLinks(links.data(), numLinks);

    std::vector<PxU32> dofStarts(numLinks, 0);
    std::vector<FreeD6RotationAxesFlags> freeRotationAxes(numLinks, FreeD6RotationAxesFlags(0));

    dofStarts[0] = 0; // The root link never has an incoming articulation joint
    // get the ordering of the links in the articulation cache
    std::vector<PxArticulationLink*> orderedLinks(numLinks);
    for (PxU32 j = 0; j < numLinks; j++)
    {
        PxU32 linkIdx = links[j]->getLinkIndex();
        orderedLinks[linkIdx] = links[j];
        if (j > 0)
        {
            PxU32 dofs = links[j]->getInboundJointDof();
            dofStarts[linkIdx] = dofs;
        }
    }

    // this is important so that link and DOF traversals are done in articulation cache order
    links = orderedLinks;


    // count DOFs, and dofStarts scan
    PxU32 numDofs = 0;
    PxU32 count = 0;
    for (PxU32 j = 1; j < numLinks; j++)
    {
        numDofs += dofStarts[j];
        PxU32 dofs = dofStarts[j];
        dofStarts[j] = count;
        count += dofs;
    }
    entryRet.dofStarts = dofStarts;
    // printf("Articulation number of Dofs = %u\n", numDofs);
    // figure out the canonical path that this articulation is mapped to in omni.physx
    size_t objectId = reinterpret_cast<size_t>(arti->userData);
    SdfPath canonicalPath;
    if (g_physx)
    {
        canonicalPath = g_physx->getPhysXObjectUsdPath(objectId);
    }

    //
    // figure out the metatype (kinematic desc)
    //

    ArticulationMetatype metatype;
    std::vector<DofImpl> dofImpls;

    // inverses of the rotations applied to incoming joint local poses during USD parsing.
    // needed to transform the output of linkIncomingJointForce to the right
    // local space because omniphysics rotates joint local poses internally during parsing.
    std::vector<PxQuat> physxToUsdRotations(numLinks);
    std::vector<PxTransform> jointChildxforms(numLinks);
    std::vector<PxTransform> jointParentxforms(numLinks);
    std::vector<PxU32> parentIndices(numLinks);
    std::vector<bool> isUsdBody0Parent(numLinks);
    std::vector<PxU32> linkIdMap(numLinks);
    metatype.setFixedBase(arti->getArticulationFlags().isSet(PxArticulationFlag::eFIX_BASE));
    // printf("Articulation structure:\n");
    for (PxU32 i = 0; i < numLinks; i++)
    {
        // get link info
        PxArticulationLink* link = links[i];
        size_t linkId = reinterpret_cast<size_t>(link->userData);
        SdfPath linkPath = g_physx->getPhysXObjectUsdPath(linkId);
        std::string linkName = linkPath.GetName();
        // printf("  Link %u\n", i);
        // printf("    Link object id: %llu\n", (unsigned long long)linkId);
        // printf("    Link path: %s\n", linkPath.GetString().c_str());
        // printf("    Link name: %s\n", linkName.c_str());

        // To avoid duplicate name
        PxU32 result = metatype.findLinkIndex(linkName.c_str());
        if (result != -1)
        {
            std::string newLinkName;
            for (PxU32 j = 0; j < numLinks; j++)
            {
                newLinkName = linkName + "_" + std::to_string(j);
                result = metatype.findLinkIndex(newLinkName.c_str());
                if (result == -1)
                    break;
            }
            linkName = newLinkName;
        }
        linkIdMap[i] = static_cast<PxU32>(linkId);
        metatype.addLink(linkName);
    }

    for (PxU32 i = 0; i < numLinks; i++)
    {
        // get link info
        PxArticulationLink* link = links[i];
        // set to identity first.
        carb::Float4 physxToUsdRotation = carb::Float4{0, 0, 0, 1.f};
        PxQuat rotationQuat(0.0f, 0.0f, 0.0f, 1.0f);
        PxTransform jointChild(PxIdentity);
        PxTransform jointParent(PxIdentity);
        PxU32 ParentLinkIndex = 0xffffffff;

        bool body0IsParent = true;
        // get joint info
        PxArticulationJointReducedCoordinate* joint =
            static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
        if (joint)
        {
            size_t jointId = reinterpret_cast<size_t>(joint->userData);
            SdfPath jointPath = g_physx->getPhysXObjectUsdPath(jointId);
            std::string jointName = jointPath.GetName();

            // To avoid duplicate name
            PxU32 result = metatype.findJointIndex(jointName.c_str());
            if (result != -1)
            {
                std::string newJointName;
                for (PxU32 j = 0; j < numLinks; j++)
                {
                    newJointName = jointName + "_" + std::to_string(j);
                    result = metatype.findJointIndex(newJointName.c_str());
                    if (result == -1)
                        break;
                }
                jointName = newJointName;
            }

            UsdPrim jointPrim = mStage->GetPrimAtPath(jointPath);
            usdrt::UsdPrim jointPrimUsdRt;

            if (!jointPrim)
            {
                long stageId = UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();    
                usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(omni::fabric::UsdStageId(stageId));

                jointPrimUsdRt = usdrtStage->GetPrimAtPath(usdrt::SdfPath(jointPath.GetString()));
            }

            // printf("    Joint object id: %llu\n", (unsigned long long)jointId);
            // printf("    Joint path: %s\n", jointPath.GetString().c_str());
            // printf("    Joint name: %s\n", jointName.c_str());

            ArticulationMetatype::JointDesc jointDesc;
            jointDesc.name = jointName;
            const usdparser::ObjectId objectId = g_physx->getObjectId(jointPath, ePTLinkJoint);
            omni::physx::JointStateData jointStateData;
            g_physxJoint->getJointStateData(objectId, &jointStateData);
            jointDesc.body0IsParent = jointStateData.body0IsParentLink;
            body0IsParent = jointStateData.body0IsParentLink;
            physxToUsdRotation = g_physxJoint->getJointFramePhysxToUsdQuat(jointId);
            rotationQuat = PxQuat(physxToUsdRotation.x, physxToUsdRotation.y, physxToUsdRotation.z, physxToUsdRotation.w);

            PxArticulationLink& parentLink = joint->getParentArticulationLink();
            PxArticulationLink& childLink = joint->getChildArticulationLink();
            size_t linkId = reinterpret_cast<size_t>(parentLink.userData);
            SdfPath linkPath = g_physx->getPhysXObjectUsdPath(linkId);
            // Search parent link index using linkId
            ParentLinkIndex = -1;
            for (PxU32 j = 0; j < numLinks; j++)
            {
                if (linkIdMap[j] == linkId)
                {
                    ParentLinkIndex = j;
                    break;
                }
            }
            std::string linkName = metatype.getLinkName(ParentLinkIndex);

            size_t childLinkId = reinterpret_cast<size_t>(childLink.userData);
            SdfPath childLinkPath = g_physx->getPhysXObjectUsdPath(childLinkId);
            PxU32 childLinkIndex = i;
            metatype.setLinkParentIndex(childLinkIndex, ParentLinkIndex);
            jointParent = joint->getParentPose();
            jointChild = joint->getChildPose();

            PxArticulationJointType::Enum jointType = joint->getJointType();
            PxU32 drive = 0;

            std::string performanceEnvelopeAPI = PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI.GetString();
            switch (jointType)
            {
            case PxArticulationJointType::eFIX:
                jointDesc.type = JointType::eFixed;
                break;
            case PxArticulationJointType::eREVOLUTE:
            case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
                jointDesc.type = JointType::eRevolute;
                jointDesc.dofs.emplace_back(jointName, DofType::eRotation);

                if (jointPrim)
                {
                    if (jointPrim.HasAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, UsdPhysicsTokens->angular))
                        drive = 1;
                }
                else
                {
                    // TODO : usdrt HasAPI with multiple applied APIs seems not working like usd
                    // if (jointPrim.HasAPI(usdrt::TfToken(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI.GetText()), usdrt::TfToken(UsdPhysicsTokens->angular.GetText())))
                    for (const auto& schema : jointPrimUsdRt.GetAppliedSchemas())
                    {
                        if (schema == usdrt::TfToken(performanceEnvelopeAPI + ":" + UsdPhysicsTokens->angular.GetText()))
                        {
                            drive = 1;
                            break;
                        }
                    }
                }
                dofImpls.push_back({ joint, PxArticulationAxis::eTWIST, drive });
                break;
            case PxArticulationJointType::ePRISMATIC:
                jointDesc.type = JointType::ePrismatic;
                jointDesc.dofs.emplace_back(jointName, DofType::eTranslation);
                if (jointPrim)
                {
                    if (jointPrim.HasAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, UsdPhysicsTokens->linear))
                        drive = 1;
                }
                else
                {
                    // if (jointPrim.HasAPI(usdrt::TfToken(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI.GetText()), usdrt::TfToken(UsdPhysicsTokens->linear.GetText())))
                    for (const auto& schema : jointPrimUsdRt.GetAppliedSchemas())
                    {
                        if (schema == usdrt::TfToken(performanceEnvelopeAPI + ":" + UsdPhysicsTokens->linear.GetString()))
                        {
                            drive = 1;
                            break;
                        }
                    }
                }
                dofImpls.push_back({ joint, PxArticulationAxis::eX, drive });
                break;
            case PxArticulationJointType::eSPHERICAL:
                if (!rotationQuat.isIdentity())
                    CARB_LOG_WARN("Using USD spherical joints with any axis except x is not currently supported.");
                // figure out which axes are unlocked
                jointDesc.type = JointType::eSpherical;
                if (joint->getMotion(PxArticulationAxis::eTWIST) != PxArticulationMotion::eLOCKED)
                {
                    // HACK? resolve custom DOF name from MJCF importer
                    freeRotationAxes[i].raise(FreeD6RotationAxesFlag::Enum::eTWIST);

                    if (jointPrim)
                    {
                        static TfToken dofNameAttribToken("mjcf:rotX:name");
                        UsdAttribute dofNameAttrib = jointPrim.GetAttribute(dofNameAttribToken);

                        TfToken dofNameToken;
                        std::string dofName;
                        if (dofNameAttrib && dofNameAttrib.Get(&dofNameToken))
                        {
                            dofName = dofNameToken.GetString();
                        }
                        else
                        {
                            dofName = jointName + ":0";
                        }
                        jointDesc.dofs.emplace_back(dofName, DofType::eRotation);
                        if (jointPrim.HasAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, UsdPhysicsTokens->rotX))
                            drive = 1;
                    }
                    else
                    {
                        static usdrt::TfToken dofNameAttribToken("mjcf:rotX:name");
                        usdrt::UsdAttribute dofNameAttrib = jointPrimUsdRt.GetAttribute(dofNameAttribToken);

                        TfToken dofNameToken;
                        std::string dofName;
                        if (dofNameAttrib && dofNameAttrib.Get(&dofNameToken))
                        {
                            dofName = dofNameToken.GetString();
                        }
                        else
                        {
                            dofName = jointName + ":0";
                        }
                        jointDesc.dofs.emplace_back(dofName, DofType::eRotation);
                        // if (jointPrim.HasAPI(usdrt::TfToken(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI.GetText()), usdrt::TfToken(UsdPhysicsTokens->rotX.GetText())))
                        for (const auto& schema : jointPrimUsdRt.GetAppliedSchemas())
                        {
                            if (schema == usdrt::TfToken(performanceEnvelopeAPI + ":" + UsdPhysicsTokens->rotX.GetString()))
                            {
                                drive = 1;
                                break;
                            }
                        }                        
                    }
                    dofImpls.push_back({joint, PxArticulationAxis::eTWIST, drive });
                }
                if (joint->getMotion(PxArticulationAxis::eSWING1) != PxArticulationMotion::eLOCKED)
                {
                    // HACK? resolve custom DOF name from MJCF importer
                    freeRotationAxes[i].raise(FreeD6RotationAxesFlag::Enum::eSWING1);
                    if (jointPrim)
                    {
                        static TfToken dofNameAttribToken("mjcf:rotY:name");
                        UsdAttribute dofNameAttrib = jointPrim.GetAttribute(dofNameAttribToken);
                        TfToken dofNameToken;
                        std::string dofName;
                        if (dofNameAttrib && dofNameAttrib.Get(&dofNameToken))
                        {
                            dofName = dofNameToken.GetString();
                        }
                        else
                        {
                            dofName = jointName + ":1";
                        }
                        jointDesc.dofs.emplace_back(dofName, DofType::eRotation);
                        if (jointPrim.HasAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, UsdPhysicsTokens->rotY))
                            drive = 1;
                    }
                    else
                    {
                        static usdrt::TfToken dofNameAttribToken("mjcf:rotY:name");
                        usdrt::UsdAttribute dofNameAttrib = jointPrimUsdRt.GetAttribute(dofNameAttribToken);
                        TfToken dofNameToken;
                        std::string dofName;
                        if (dofNameAttrib && dofNameAttrib.Get(&dofNameToken))
                        {
                            dofName = dofNameToken.GetString();
                        }
                        else
                        {
                            dofName = jointName + ":1";
                        }
                        jointDesc.dofs.emplace_back(dofName, DofType::eRotation);
                        //if (jointPrim.HasAPI(usdrt::TfToken(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI.GetText()), usdrt::TfToken(UsdPhysicsTokens->rotY.GetText())))
                        for (const auto& schema : jointPrimUsdRt.GetAppliedSchemas())
                        {
                            if (schema == usdrt::TfToken(performanceEnvelopeAPI + ":" + UsdPhysicsTokens->rotY.GetString()))
                            {
                                drive = 1;
                                break;
                            }
                        }
                    }
                    dofImpls.push_back({joint, PxArticulationAxis::eSWING1, drive });
                }
                if (joint->getMotion(PxArticulationAxis::eSWING2) != PxArticulationMotion::eLOCKED)
                {
                    // HACK? resolve custom DOF name from MJCF importer
                    freeRotationAxes[i].raise(FreeD6RotationAxesFlag::Enum::eSWING2);
                    if (jointPrim)
                    {
                        static TfToken dofNameAttribToken("mjcf:rotZ:name");
                        UsdAttribute dofNameAttrib = jointPrim.GetAttribute(dofNameAttribToken);
                        TfToken dofNameToken;
                        std::string dofName;
                        if (dofNameAttrib && dofNameAttrib.Get(&dofNameToken))
                        {
                            dofName = dofNameToken.GetString();
                        }
                        else
                        {
                            dofName = jointName + ":2";
                        }
                        jointDesc.dofs.emplace_back(dofName, DofType::eRotation);
                        if (jointPrim.HasAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, UsdPhysicsTokens->rotZ))
                            drive = 1;
                    }
                    else
                    {
                        static usdrt::TfToken dofNameAttribToken("mjcf:rotZ:name");
                        usdrt::UsdAttribute dofNameAttrib = jointPrimUsdRt.GetAttribute(dofNameAttribToken);
                        TfToken dofNameToken;
                        std::string dofName;
                        if (dofNameAttrib && dofNameAttrib.Get(&dofNameToken))
                        {
                            dofName = dofNameToken.GetString();
                        }
                        else
                        {
                            dofName = jointName + ":2";
                        }
                        jointDesc.dofs.emplace_back(dofName, DofType::eRotation);
                        // if (jointPrim.HasAPI(usdrt::TfToken(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI.GetText()), usdrt::TfToken(UsdPhysicsTokens->rotZ.GetText())))
                        for (const auto& schema : jointPrimUsdRt.GetAppliedSchemas())
                        {
                            if (schema == usdrt::TfToken(performanceEnvelopeAPI + ":" + UsdPhysicsTokens->rotZ.GetString()))
                            {
                                drive = 1;
                                break;
                            }
                        }
                    }
                    dofImpls.push_back({joint, PxArticulationAxis::eSWING2, drive });
                }
                //printf("FreeD6RotationAxesFlags = %u\n", PxU32(freeRotationAxes[i]));
                break;
            case PxArticulationJointType::eUNDEFINED:
            default:
                CARB_LOG_ERROR("Unknown joint type for joint '%s'", jointPath.GetString().c_str());
                break;
            }

            if (jointDesc.type != JointType::eInvalid)
            {
                metatype.addJoint(jointDesc);
            }
        }

        CARB_ASSERT(rotationQuat.isUnit());
        physxToUsdRotations[i] = rotationQuat;
        jointChildxforms[i] = jointChild;
        jointParentxforms[i] = jointParent;
        parentIndices[i] = ParentLinkIndex;
        isUsdBody0Parent[i] = body0IsParent;
    }

    //
    // populate entry
    //

    entryRet.metatype = getUniqueArticulationMetatype(metatype);
    entryRet.arti = arti;
    entryRet.numLinks = numLinks;
    entryRet.numDofs = numDofs;
    entryRet.freeD6Axes = freeRotationAxes;


    // printf("~!~! Arti metatype @ %p\n", entryRet.metatype);

    entryRet.links = links;
    entryRet.dofImpls = dofImpls;
    entryRet.incomingJointPhysxToUsdRotations = physxToUsdRotations;
    entryRet.isIncomingJointBody0Parent = isUsdBody0Parent;
    entryRet.jointChild = jointChildxforms;
    entryRet.jointParent = jointParentxforms;
    entryRet.parentIndices = parentIndices;

    // shapes
    PxU32 numShapes = 0;
    for (PxU32 i = 0; i < numLinks; i++)
    {
        PxU32 linkNumShapes = links[i]->getNbShapes();
        if (linkNumShapes > 0)
        {
            std::vector<::physx::PxShape*> shapes(linkNumShapes);
            links[i]->getShapes(shapes.data(), linkNumShapes);
            entryRet.shapes.insert(entryRet.shapes.end(), shapes.begin(), shapes.end());
            numShapes += linkNumShapes;
        }
    }
    entryRet.numShapes = numShapes;

    // fixed tendons
    PxU32 numFixedTendons = arti->getNbFixedTendons();
    if (numFixedTendons > 0)
    {
        entryRet.numFixedTendons = numFixedTendons;
        entryRet.fixedTendons.resize(numFixedTendons);
        arti->getFixedTendons(entryRet.fixedTendons.data(), numFixedTendons);
    }

    // spatial tendons
    PxU32 numSpatialTendons = arti->getNbSpatialTendons();
    if (numSpatialTendons > 0)
    {
        entryRet.numSpatialTendons = numSpatialTendons;
        entryRet.spatialTendons.resize(numSpatialTendons);
        arti->getSpatialTendons(entryRet.spatialTendons.data(), numSpatialTendons);
    }

    if (!canonicalPath.IsEmpty())
    {
        entryRet.path = canonicalPath;
    }
    else
    {
        entryRet.path = path;
    }

    entryRet.subspace = findSubspaceForPath(entryRet.path);

    return true;
}

bool BaseSimulationView::getRigidBodyAtPath(const pxr::SdfPath& path, RigidBodyEntry& entryRet)
{
    if (!g_physx)
    {
        return false;
    }

    PxRigidBody* body = nullptr;
    RigidBodyType type = RigidBodyType::eInvalid;

    // check if it's an articulation link
    PxArticulationLink* link = static_cast<PxArticulationLink*>(g_physx->getPhysXPtr(path, omni::physx::ePTLink));
    if (link)
    {
        body = link;
        type = RigidBodyType::eArticulationLink;
    }
    else
    {
        // check if it's an articulation, in which case we'll use the root link
        // NOTE: This is an important edge case when we instance single-body actors as articulations with a fixed base.
        //       (We can't use kinematic bodies, because OmniPhysX does not update kinematic transforms to USD/hydra.)
        PxArticulationReducedCoordinate* arti =
            (PxArticulationReducedCoordinate*)g_physx->getPhysXPtr(path, omni::physx::ePTArticulation);
        if (arti)
        {
            // printf("Got %s at %p\n", arti->getConcreteTypeName(), (void*)abase);
            PxU32 numLinks = arti->getNbLinks();
            if (numLinks > 0)
            {
                std::vector<PxArticulationLink*> links(numLinks);
                arti->getLinks(links.data(), numLinks);
                body = links[0];
                type = RigidBodyType::eArticulationLink;
            }
        }
        else
        {
            // check if it's an actor
            PxActor* actor = static_cast<PxActor*>(g_physx->getPhysXPtr(path, omni::physx::ePTActor));
            if (actor)
            {
                // check if it's a rigid dynamic
                if (actor->getType() == PxActorType::eRIGID_DYNAMIC)
                {
                    PxRigidDynamic* rd = static_cast<PxRigidDynamic*>(actor);
                    body = rd;
                    type = RigidBodyType::eRigidDynamic;
                }
            }
        }
    }

    if (!body)
    {
        CARB_LOG_WARN("Failed to find rigid body at '%s'", path.GetString().c_str());
        return false;
    }

#if 0
    printf("Got rigid body of type %s at %p (%s)\n", body->getConcreteTypeName(), body, path.GetText());
    PxU32 numShapes = body->getNbShapes();
    std::vector<PxShape*> shapes(numShapes);
    body->getShapes(shapes.data(), numShapes);
    printf("  %u shapes\n", numShapes);
    for (PxU32 i = 0; i < numShapes; i++)
    {
        printf("    Shape %u\n", i);
        printf("      Contact offset: %f\n", shapes[i]->getContactOffset());
        printf("      Rest offset:    %f\n", shapes[i]->getRestOffset());
    }
#endif

    //
    // populate entry
    //

    entryRet.body = body;
    entryRet.type = type;
    entryRet.path = path;
    entryRet.subspace = findSubspaceForPath(entryRet.path);

    // shapes
    PxU32 numShapes = body->getNbShapes();
    if (numShapes > 0)
    {
        entryRet.shapes.resize(numShapes);
        body->getShapes(entryRet.shapes.data(), numShapes);
    }
    entryRet.numShapes = numShapes;

    return true;
}

bool BaseSimulationView::getSoftBodyAtPath(const pxr::SdfPath& path, SoftBodyEntry& entryRet)
{
    if (!g_physx)
    {
        return false;
    }
   
    // check if it's a soft body
    PxSoftBody* sb = static_cast<PxSoftBody*>(g_physx->getPhysXPtr(path, omni::physx::ePTSoftBodyDeprecated));
    if (!sb)
    {
        CARB_LOG_WARN("Failed to find soft body at '%s'", path.GetString().c_str());
        return false;
    }

    UsdPrim usdPrim = mStage->GetPrimAtPath(path);
    if (!usdPrim.IsValid() || !usdPrim.HasAPI<PhysxSchemaPhysxDeformableBodyAPI>())
    {
        CARB_LOG_WARN("soft body at '%s' requires PhysxSchemaPhysxDeformableBodyAPI", path.GetText());
        return false;
    }

    pxr::PhysxSchemaPhysxDeformableBodyAPI deformableAPI(usdPrim);
    if (deformableAPI)
    {
        VtArray<GfVec3f> meshPointsSim;
        VtArray<int> meshIndicesSim;
        pxr::PhysxSchemaPhysxDeformableAPI(deformableAPI).GetSimulationIndicesAttr().Get(&meshIndicesSim);
        deformableAPI.GetSimulationRestPointsAttr().Get(&meshPointsSim);
        std::vector<PxMat33>& simRestPoses = entryRet.simRestPoses;
        std::vector<PxU32>& simIndices = entryRet.simIndices;
        // printf("Points size = %u, Indices.size()=%u, element size= %u\n", (PxU32)meshPointsSim.size(),
        //        (PxU32)meshIndicesSim.size(), (PxU32)meshIndicesSim.size() / 4);
        simRestPoses.resize(meshIndicesSim.size() / 4);
        simIndices.resize(meshIndicesSim.size());

        for (PxU32 i = 0; i < meshIndicesSim.size(); i = i + 4)
        {
            simIndices[i + 0] = meshIndicesSim[i + 0];
            simIndices[i + 1] = meshIndicesSim[i + 1];
            simIndices[i + 2] = meshIndicesSim[i + 2];
            simIndices[i + 3] = meshIndicesSim[i + 3];
            // printf("%u, %u ,%u, %u, ", meshIndicesSim[i + 0], meshIndicesSim[i + 1], meshIndicesSim[i + 2], meshIndicesSim[i + 3]);

            const GfVec3f u1 = meshPointsSim[meshIndicesSim[i + 1]] - meshPointsSim[meshIndicesSim[i]];
            const GfVec3f u2 = meshPointsSim[meshIndicesSim[i + 2]] - meshPointsSim[meshIndicesSim[i]];
            const GfVec3f u3 = meshPointsSim[meshIndicesSim[i + 3]] - meshPointsSim[meshIndicesSim[i]];
            PxMat33 P = PxMat33({ u1[0], u1[1], u1[2] }, { u2[0], u2[1], u2[2] }, { u3[0], u3[1], u3[2] });
            // printf("elem %u P => %f,%f,%f,%f,%f,%f,%f,%f,%f\n", i/4, P[0][0], P[0][1], P[0][2], P[1][0], P[1][1], P[1][2],
            //        P[2][0], P[2][1], P[2][2]);

            const PxReal det = P.getDeterminant();
            const PxReal volume = det / 6.0f;
            if (volume < 1e-9f)
                simRestPoses[i / 4] = PxMat33(PxZero);
            else
                simRestPoses[i / 4] = P.getInverse();
            // PxMat33 Q =simRestPoses[i / 4];
            // printf("elem %u Q => %f,%f,%f,%f,%f,%f,%f,%f,%f\n", i/4, Q[0][0], Q[0][1], Q[0][2], Q[1][0], Q[1][1], Q[1][2],
            //        Q[2][0], Q[2][1], Q[2][2]);
        }
    }

    //
    // populate entry
    //

    entryRet.body = sb;
    entryRet.path = path;
    entryRet.subspace = findSubspaceForPath(entryRet.path);
    // std::cout << "getSoftBodyAtPath: " << path.GetString() << " origin=" << entryRet.subspace->origin.x << " "
    //           << entryRet.subspace->origin.y << " " << entryRet.subspace->origin.z << std::endl;

    return true;
}

bool BaseSimulationView::getSoftBodyMaterialAtPath(const pxr::SdfPath& path, SoftBodyMaterialEntry& entryRet)
{
    if (!g_physx)
    {
        return false;
    }

    PxFEMSoftBodyMaterial* sbMaterial = static_cast<PxFEMSoftBodyMaterial*>(g_physx->getPhysXPtr(path, omni::physx::ePTSoftBodyMaterialDeprecated));
    if (!sbMaterial)
    {
        CARB_LOG_WARN("Failed to find soft body at '%s'", path.GetText());
        return false;
    }

    //
    // populate entry
    //
    entryRet.femSoftBodyMaterial = sbMaterial;
    entryRet.path = path;
    // std::cout << "getSoftBodyAtPath: " << path.GetString() << " origin=" << entryRet.subspace->origin.x << " "
    //           << entryRet.subspace->origin.y << " " << entryRet.subspace->origin.z << std::endl;

    return true;
}

bool BaseSimulationView::getVolumeDeformableBodyAtPath(const pxr::SdfPath& path, DeformableBodyEntry& entryRet)
{
    if (!g_physx)
    {
        return false;
    }

    // check if it's a volume deformable body
    PxDeformableVolume* deformable = static_cast<PxDeformableVolume*>(g_physx->getPhysXPtr(path, omni::physx::ePTDeformableVolume));
    if (!deformable)
    {
        CARB_LOG_WARN("Failed to find volume deformable body at '%s'", path.GetString().c_str());
        return false;
    }

    TfType deformableBodyType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
    UsdPrim usdPrim = mStage->GetPrimAtPath(path);
    if (!usdPrim.IsValid() || !usdPrim.HasAPI(deformableBodyType))
    {
        CARB_LOG_WARN("Volume deformable body at '%s' requires OmniPhysicsDeformableBodyAPI", path.GetText());
        return false;
    }

    SdfPath simMeshPath;
    SdfPath collMeshPath;
    SdfPathSet skinMeshPaths;
    bool success = findDeformableMeshPaths(simMeshPath, collMeshPath, skinMeshPaths, path, mStage);
    if (!success)
    {
        CARB_LOG_WARN("Failed to find simulation or collision mesh for volume deformable body at '%s'", path.GetText());
        return false;
    }

    UsdGeomTetMesh simTetMesh(mStage->GetPrimAtPath(simMeshPath));
    if (!simTetMesh)
    {
        CARB_LOG_WARN("Simulation mesh at '%s' is not a UsdGeomTetMesh", simMeshPath.GetText());
        return false;
    }

    UsdGeomTetMesh collTetMesh(mStage->GetPrimAtPath(collMeshPath));
    if (!collTetMesh)
    {
        CARB_LOG_WARN("Collision mesh at '%s' is not a UsdGeomTetMesh", collMeshPath.GetText());
        return false;
    }

    VtArray<GfVec4i> simMeshIndices;
    simTetMesh.GetTetVertexIndicesAttr().Get(&simMeshIndices);

    // Read rest shape attributes and check on current restrictions
    VtArray<GfVec3f> restPositions;
    {
        VtArray<GfVec4i> restTetVtxIndices;
        simTetMesh.GetPrim().GetAttribute(OmniPhysicsDeformableAttrTokens->restTetVtxIndices).Get(&restTetVtxIndices);

        if (simMeshIndices.size() != restTetVtxIndices.size() ||
            std::memcmp(simMeshIndices.data(), restTetVtxIndices.data(), sizeof(GfVec4i) * simMeshIndices.size()) != 0)
        {
            CARB_LOG_WARN("No support for distinct rest shape topology. The simulation mesh's tetVertexIndices need to "
                          "match up with VolumeDeformableSimAPI restTetVtxIndices at '%s'", simMeshPath.GetText());
            return false;
        }

        VtArray<GfVec3f> simPoints;
        simTetMesh.GetPointsAttr().Get(&simPoints);

        simTetMesh.GetPrim().GetAttribute(OmniPhysicsDeformableAttrTokens->restShapePoints).Get(&restPositions);

        if (simPoints.size() != restPositions.size())
        {
            CARB_LOG_WARN("No support for distinct rest shape topology. The simulation mesh's points need to match up "
                          "with VolumeDeformableSimAPI restShapePoints at '%s'", simMeshPath.GetText());
        }
    }

    VtArray<GfVec4i> collMeshIndices;
    collTetMesh.GetTetVertexIndicesAttr().Get(&collMeshIndices);

    // populate entry
    entryRet.body = deformable;
    entryRet.path = path;
    entryRet.simMeshPath = simMeshPath;
    entryRet.collMeshPath = collMeshPath;
    entryRet.subspace = findSubspaceForPath(entryRet.path);

    entryRet.simIndices.resize(simMeshIndices.size() * 4);
    std::memcpy(entryRet.simIndices.data(), simMeshIndices.data(), sizeof(PxU32) * entryRet.simIndices.size());

    // transform restPositions into world space to account for scaling
    entryRet.restPositions.resize(restPositions.size());
    GfMatrix4d sim_to_world = UsdGeomXformable(simTetMesh).ComputeLocalToWorldTransform(UsdTimeCode::Default());
    for (size_t i = 0; i < entryRet.restPositions.size(); ++i)
    {
        GfVec3f restPoint = GfVec3f(sim_to_world.Transform(restPositions[i]));
        entryRet.restPositions[i] = { restPoint[0], restPoint[1], restPoint[2] };
    }

    entryRet.collIndices.resize(collMeshIndices.size() * 4);
    std::memcpy(entryRet.collIndices.data(), collMeshIndices.data(), sizeof(PxU32) * entryRet.collIndices.size());

    return true;
}

bool BaseSimulationView::getSurfaceDeformableBodyAtPath(const pxr::SdfPath& path, DeformableBodyEntry& entryRet)
{
    if (!g_physx)
    {
        return false;
    }

    // check if it's a surface deformable body
    PxDeformableSurface* deformable = static_cast<PxDeformableSurface*>(g_physx->getPhysXPtr(path, omni::physx::ePTDeformableSurface));
    if (!deformable)
    {
        CARB_LOG_WARN("Failed to find surface deformable body at '%s'", path.GetString().c_str());
        return false;
    }

    TfType deformableBodyType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
    UsdPrim usdPrim = mStage->GetPrimAtPath(path);
    if (!usdPrim.IsValid() || !usdPrim.HasAPI(deformableBodyType))
    {
        CARB_LOG_WARN("Surface deformable body at '%s' requires OmniPhysicsDeformableBodyAPI", path.GetText());
        return false;
    }

    SdfPath simMeshPath;
    SdfPath collMeshPath;
    SdfPathSet skinMeshPaths;
    bool success = findDeformableMeshPaths(simMeshPath, collMeshPath, skinMeshPaths, path, mStage);
    if (!success)
    {
        CARB_LOG_WARN("Failed to find simulation or collision mesh for surface deformable body at '%s'", path.GetText());
        return false;
    }

    if (simMeshPath != collMeshPath)
    {
        CARB_LOG_WARN("Found surface deformable body with separate collision mesh, which is not supported '%s'", path.GetText());
        return false;
    }

    UsdGeomMesh simTriMesh(mStage->GetPrimAtPath(simMeshPath));
    if (!simTriMesh)
    {
        CARB_LOG_WARN("Simulation mesh at '%s' is not a UsdGeomMesh", simMeshPath.GetText());
        return false;
    }

    VtArray<int> simMeshIndices;
    simTriMesh.GetFaceVertexIndicesAttr().Get(&simMeshIndices);
    if (simMeshIndices.size() % 3 != 0)
    {
        CARB_LOG_WARN("Simulation mesh at '%s' has non-triangular faces", simMeshPath.GetText());
        return false;
    }

    // Read rest shape attributes and check on current restrictions
    VtArray<GfVec3f> restPositions;
    {
        VtArray<GfVec3i> restTriVtxIndices;
        simTriMesh.GetPrim().GetAttribute(OmniPhysicsDeformableAttrTokens->restTriVtxIndices).Get(&restTriVtxIndices);

        if (simMeshIndices.size() != restTriVtxIndices.size()*3 ||
            std::memcmp(simMeshIndices.data(), restTriVtxIndices.data(), sizeof(int32_t) * simMeshIndices.size()) != 0)
        {
            CARB_LOG_WARN(
                "No support for distinct rest shape topology. The simulation mesh's faceVertexIndices need to "
                "match up with SurfaceDeformableSimAPI restTriVtxIndices at '%s'",
                simMeshPath.GetText());
            return false;
        }

        VtArray<GfVec3f> simPoints;
        simTriMesh.GetPointsAttr().Get(&simPoints);

        simTriMesh.GetPrim().GetAttribute(OmniPhysicsDeformableAttrTokens->restShapePoints).Get(&restPositions);

        if (simPoints.size() != restPositions.size())
        {
            CARB_LOG_WARN(
                "No support for distinct rest shape topology. The simulation mesh's points need to match up "
                "with SurfaceDeformableSimAPI restShapePoints at '%s'",
                simMeshPath.GetText());
        }
    }

    // populate entry
    entryRet.body = deformable;
    entryRet.path = path;
    entryRet.simMeshPath = simMeshPath;
    entryRet.collMeshPath = simMeshPath;
    entryRet.subspace = findSubspaceForPath(entryRet.path);

    entryRet.simIndices.resize(simMeshIndices.size());
    std::memcpy(entryRet.simIndices.data(), simMeshIndices.data(), sizeof(PxU32) * entryRet.simIndices.size());

    //transform restPositions into world space to account for scaling
    entryRet.restPositions.resize(restPositions.size());
    GfMatrix4d sim_to_world = UsdGeomXformable(simTriMesh).ComputeLocalToWorldTransform(UsdTimeCode::Default());
    for (size_t i = 0; i < entryRet.restPositions.size(); ++i)
    {
        GfVec3f restPoint = GfVec3f(sim_to_world.Transform(restPositions[i]));
        entryRet.restPositions[i] = { restPoint[0], restPoint[1], restPoint[2] };
    }

    return true;
}

bool BaseSimulationView::getDeformableMaterialAtPath(const pxr::SdfPath& path, DeformableMaterialEntry& entryRet)
{
    if (!g_physx)
    {
        return false;
    }

    PxDeformableMaterial* material = static_cast<PxDeformableMaterial*>(g_physx->getPhysXPtr(path, omni::physx::ePTDeformableVolumeMaterial));
    if (!material)
    {
        material = static_cast<PxDeformableMaterial*>(g_physx->getPhysXPtr(path, omni::physx::ePTDeformableSurfaceMaterial));
    }

    if (!material)
    {
        CARB_LOG_WARN("Failed to find deformable material at '%s'", path.GetText());
        return false;
    }

    // populate entry
    entryRet.material = material;
    entryRet.path = path;

    return true;
}

bool BaseSimulationView::getRigidContactSensorAtPath(const pxr::SdfPath& path, RigidContactSensorEntry& entryRet)
{
    // eek, we need to grok the sensor details from USD...
    UsdPrim sensorPrim = mStage->GetPrimAtPath(path);
    usdrt::UsdPrim sensorPrimUsdRt;
    if (!sensorPrim)
    {
        long stageId = UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();    
        usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(omni::fabric::UsdStageId(stageId));

        sensorPrimUsdRt = usdrtStage->GetPrimAtPath(path.GetString());
    }

    if (!sensorPrim && !sensorPrimUsdRt)
    {
        CARB_LOG_WARN("Failed to get prim at '%s'", path.GetText());
        return false;
    }

    if (sensorPrim)
    {
        PhysxSchemaPhysxContactReportAPI crApi = PhysxSchemaPhysxContactReportAPI(sensorPrim);
        if (!crApi)
        {
            CARB_LOG_WARN("Failed to find contact report API at '%s'", path.GetText());
            return false;
        }
    }
    else
    {
        bool hasAPI = sensorPrimUsdRt.HasAPI("PhysxSchemaPhysxContactReportAPI");
        if (!hasAPI)
        {
            CARB_LOG_WARN("Failed to find contact report API at '%s'", path.GetText());
            return false;
        }
    }

    /*
    // needed?
    UsdPhysicsRigidBodyAPI rbApi = UsdPhysicsRigidBodyAPI(sensorPrim);
    UsdPhysicsCollisionAPI collisionApi = UsdPhysicsCollisionAPI(sensorPrim);
    if (!rbApi && !collisionApi)
    {
        CARB_LOG_WARN("Contact sensor not attached to a body or collider ('%s')", path.GetText());
        return false;
    }
    */

    // figure out if it's a rigid dynamic, articulation link, or shape
    PxArticulationLink* link = nullptr;
    PxRigidDynamic* rd = nullptr;
    PxShape* shape = nullptr;
    link = static_cast<PxArticulationLink*>(g_physx->getPhysXPtr(path, omni::physx::ePTLink));
    if (!link)
    {
        PxActor* actor = static_cast<PxActor*>(g_physx->getPhysXPtr(path, omni::physx::ePTActor));
        if (actor && actor->getType() == PxActorType::eRIGID_DYNAMIC)
        {
            rd = static_cast<PxRigidDynamic*>(actor);
        }
        else
        {
            shape = static_cast<PxShape*>(g_physx->getPhysXPtr(path, omni::physx::ePTShape));
            if (!shape)
            {
                return false;
            }
        }
    }
    else
    {
        // for articulation links specify names for later to match with the articulation meta type
        size_t linkId = reinterpret_cast<size_t>(link->userData);
        SdfPath linkPath = g_physx->getPhysXObjectUsdPath(linkId);
        std::string name = linkPath.GetName();
        uint32_t currSize = (uint32_t)mSimData->mUniqueRCNames2Idx.size();
        if (mSimData->mUniqueRCNames2Idx.find(name) == mSimData->mUniqueRCNames2Idx.end())
        {
            mSimData->mUniqueRCNames2Idx.insert(std::make_pair(name, currSize));
            mSimData->mUniqueRCIdx2Names.insert(std::make_pair(currSize, name));
            entryRet.nameID = currSize;
        }
        else
        {
            entryRet.nameID = mSimData->mUniqueRCNames2Idx[name];
        }
    }

    //
    // populate entry
    //

    entryRet.path = path;
    entryRet.referentId = asInt(path);
    entryRet.link = link;
    entryRet.rd = rd;
    entryRet.shape = shape;
    entryRet.subspace = findSubspaceForPath(entryRet.path);

    return true;
}

bool BaseSimulationView::getSDFShapeAtPath(const pxr::SdfPath& path, SdfShapeEntry& entryRet)
{
    UsdPrim Prim = mStage->GetPrimAtPath(path);
    usdrt::UsdPrim PrimUsdRt;

    if (!Prim)
    {
        long stageId = UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();    
        usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(omni::fabric::UsdStageId(stageId));

        PrimUsdRt = usdrtStage->GetPrimAtPath(path.GetString());
    }

    if (!Prim && !PrimUsdRt)
    {
        CARB_LOG_WARN("Failed to get prim at '%s'", path.GetText());
        return false;
    }

    if (Prim)
    {
        UsdPhysicsCollisionAPI collisionApi = UsdPhysicsCollisionAPI(Prim);
        PhysxSchemaPhysxSDFMeshCollisionAPI SDFMeshApi = PhysxSchemaPhysxSDFMeshCollisionAPI(Prim);
        if (!SDFMeshApi || !collisionApi)
        {
            CARB_LOG_WARN("Failed to find CollisionAPI and PhysxSDFMeshCollisionAPI for prim at ('%s')", path.GetText());
            return false;
        }
    }
    else
    {
        bool hasCollisionAPI = PrimUsdRt.HasAPI("UsdPhysicsCollisionAPI");
        bool hasSDFMeshAPI = PrimUsdRt.HasAPI("PhysxSchemaPhysxSDFMeshCollisionAPI");
        if (!hasCollisionAPI || !hasSDFMeshAPI)
        {
            CARB_LOG_WARN("Failed to find CollisionAPI and PhysxSDFMeshCollisionAPI for prim at ('%s')", path.GetText());
            return false;
        }
    }

    PxShape* shape = static_cast<PxShape*>(g_physx->getPhysXPtr(path, omni::physx::ePTShape));
    if (!shape)
    {
        CARB_LOG_WARN("Failed to find a mesh at '%s'", path.GetText());
        return false;
    }

    bool hasSDF = false;
    const PxGeometry& geom = shape->getGeometry();
    if(geom.getType() == PxGeometryType::eTRIANGLEMESH)
    {
        const PxTriangleMeshGeometry& triangleMeshGeom = static_cast<const PxTriangleMeshGeometry&>(geom);
        if(triangleMeshGeom.isValid())
        {
            hasSDF = triangleMeshGeom.triangleMesh->getSDF() != NULL;

            if(hasSDF)
            {
                PxU32 dimX, dimY, dimZ;
                triangleMeshGeom.triangleMesh->getSDFDimensions(dimX, dimY, dimZ);
                hasSDF = dimX > 0 && dimY > 0 && dimZ > 0;
            }
        }
    }

    if (!hasSDF)
    {
        CARB_LOG_WARN("Failed to find a valid SDF mesh for prim at ('%s')", path.GetText());
        return false;
    }

    entryRet.shape = shape;
    entryRet.path = path;
    entryRet.subspace = findSubspaceForPath(entryRet.path);
    return true;
}

bool BaseSimulationView::getParticleSystemAtPath(const pxr::SdfPath& path, ParticleSystemEntry& entryRet)
{
    if (!g_physx)
    {
        return false;
    }

    // check if it's a particle system
    PxPBDParticleSystem* ps = static_cast<PxPBDParticleSystem*>(g_physx->getPhysXPtr(path, omni::physx::ePTParticleSystem));
    if (!ps)
    {
        CARB_LOG_WARN("Failed to find particle system at '%s'", path.GetString().c_str());
        return false;
    }

    //
    // populate entry
    //

    entryRet.particleSystem = ps;
    entryRet.path = path;

    return true;
}

bool BaseSimulationView::getParticleClothAtPath(const pxr::SdfPath& path, ParticleClothEntry& entryRet)
{
    if (!g_physx)
    {
        return false;
    }

    // check if it's a particle cloth
    PxParticleClothBuffer* pc = static_cast<PxParticleClothBuffer*>(g_physx->getPhysXPtr(path, omni::physx::ePTParticleClothDeprecated));
    if (!pc)
    {
        CARB_LOG_WARN("Failed to find particle cloth at '%s'", path.GetString().c_str());
        return false;
    }

    // get simulation owner particle system
    pxr::UsdPrim clothPrim = mStage->GetPrimAtPath(path);
    pxr::SdfPath particleSystemPath;
    pxr::PhysxSchemaPhysxParticleAPI particleApi(clothPrim);
    if (particleApi)
    {
        if (pxr::UsdRelationship rel = particleApi.GetParticleSystemRel())
        {
            pxr::SdfPathVector paths;
            if (rel.GetTargets(&paths) && paths.size() > 0)
            {
                particleSystemPath = paths[0].GetPrimPath();
            }
        }
    }
    PxPBDParticleSystem* particleSystem = static_cast<PxPBDParticleSystem*>(g_physx->getPhysXPtr(particleSystemPath, omni::physx::ePTParticleSystem));
    if (!particleSystem)
    {
        CARB_LOG_WARN("Failed to find particle system at '%s' for cloth '%s'", particleSystemPath.GetString().c_str(), path.GetString().c_str());
    }

    //
    // populate entry
    //

    entryRet.cloth = pc;
    entryRet.particleSystem = particleSystem;
    entryRet.path = path;
    entryRet.subspace = findSubspaceForPath(entryRet.path);

    return true;
}


bool BaseSimulationView::getParticleMaterialAtPath(const pxr::SdfPath& path, ParticleMaterialEntry& entryRet)
{
    if (!g_physx)
    {
        return false;
    }

    // check if it's a PBD material
    PxPBDMaterial* pm = static_cast<PxPBDMaterial*>(g_physx->getPhysXPtr(path, omni::physx::ePTPBDMaterial));
    if (!pm)
    {
        CARB_LOG_WARN("Failed to find particle material at '%s'", path.GetString().c_str());
        return false;
    }

    //
    // populate entry
    //

    entryRet.pbdMaterial = pm;
    entryRet.path = path;

    return true;
}

const ArticulationMetatype* BaseSimulationView::getUniqueArticulationMetatype(const ArticulationMetatype& metatype)
{
    auto it = mSimData->mUniqueTypes.insert(metatype).first;
    return &(*it);
}

bool BaseSimulationView::setGravity(const carb::Float3& gravity)
{
    if (!g_physxPrivate)
    {
        CARB_LOG_ERROR("%s: Failed to aquire PhysX private interface", __FUNCTION__);
        return false;
    }

    PxScene* scene = g_physxPrivate->getPhysXScene();
    if (!scene)
    {
        CARB_LOG_ERROR("%s: Failed to get physics scene. Is the simulation active?", __FUNCTION__);
        return false;
    }

    scene->setGravity((const PxVec3&)gravity);

    return true;
}

bool BaseSimulationView::getGravity(carb::Float3& gravity)
{
    if (!g_physxPrivate)
    {
        CARB_LOG_ERROR("%s: Failed to aquire PhysX private interface", __FUNCTION__);
        return false;
    }

    PxScene* scene = g_physxPrivate->getPhysXScene();
    if (!scene)
    {
        CARB_LOG_ERROR("%s: Failed to get physics scene. Is the simulation active?", __FUNCTION__);
        return false;
    }

    PxVec3 physxGravity = scene->getGravity();
    gravity.x = physxGravity.x;
    gravity.y = physxGravity.y;
    gravity.z = physxGravity.z;

    return true;
}

bool BaseSimulationView::check() const
{
    bool result = true;

    for (auto artiView : mArtiViews)
    {
        if (!artiView->check())
        {
            result = false;
        }
    }

    for (auto rbView : mRbViews)
    {
        if (!rbView->check())
        {
            result = false;
        }
    }

    for (auto sbView : mSbViews)
    {
        if (!sbView->check())
        {
            result = false;
        }
    }

    for (auto sbMaterialView : mSbMaterialViews)
    {
        if (!sbMaterialView->check())
        {
            result = false;
        }
    }

    for (auto vdbView : mVolumeDeformableBodyViews)
    {
        if (!vdbView->check())
        {
            result = false;
        }
    }

    for (auto sdbView : mSurfaceDeformableBodyViews)
    {
        if (!sdbView->check())
        {
            result = false;
        }
    }

    for (auto dMaterialView : mDeformableMaterialViews)
    {
        if (!dMaterialView->check())
        {
            result = false;
        }
    }

    for (auto rcView : mRcViews)
    {
        if (!rcView->check())
        {
            result = false;
        }
    }

    for (auto psView: mParticleSysViews)
    {
        if (!psView->check())
        {
            result = false;
        }
    }

    for (auto pcView: mParticleClothViews)
    {
        if (!pcView->check())
        {
            result = false;
        }
    }

    for (auto pmView: mParticleMatViews)
    {
        if (!pmView->check())
        {
            result = false;
        }
    }

    return result;
}

void BaseSimulationView::release(bool recursive)
{
    if (recursive)
    {
        for (auto artiView : mArtiViews)
        {
            artiView->release();
        }

        for (auto rbView : mRbViews)
        {
            rbView->release();
        }

        for (auto sbView : mSbViews)
        {
            sbView->release();
        }

        for (auto sbMaterialView : mSbMaterialViews)
        {
            sbMaterialView->release();
        }

        for (auto vdbView : mVolumeDeformableBodyViews)
        {
            vdbView->release();
        }

        for (auto sdbView : mSurfaceDeformableBodyViews)
        {
            sdbView->release();
        }

        for (auto dMaterialView : mDeformableMaterialViews)
        {
            dMaterialView->release();
        }

        for (auto rcView : mRcViews)
        {
            rcView->release();
        }

        for (auto psView: mParticleSysViews)
        {
            psView->release();
        }

        for (auto pcView: mParticleClothViews)
        {
            pcView->release();
        }

        for (auto pmView: mParticleMatViews)
        {
            pmView->release();
        }
    }
    delete this;
}

void BaseSimulationView::_onChildRelease(const BaseSdfShapeView* sdfView)
{
    auto it = std::find(mSDFViews.begin(), mSDFViews.end(), sdfView);
    if (it != mSDFViews.end())
    {
        // printf("~!~! Detaching child articulation view %p\n", artiView);
        mSDFViews.erase(it);
    }
}


void BaseSimulationView::_onChildRelease(const BaseArticulationView* artiView)
{
    auto it = std::find(mArtiViews.begin(), mArtiViews.end(), artiView);
    if (it != mArtiViews.end())
    {
        // printf("~!~! Detaching child articulation view %p\n", artiView);
        mArtiViews.erase(it);
    }
}

void BaseSimulationView::_onChildRelease(const BaseRigidBodyView* rbView)
{
    auto it = std::find(mRbViews.begin(), mRbViews.end(), rbView);
    if (it != mRbViews.end())
    {
        // printf("~!~! Detaching child rigid body view %p\n", rbView);
        mRbViews.erase(it);
    }
}

void BaseSimulationView::_onChildRelease(const BaseSoftBodyView* sbView)
{
    auto it = std::find(mSbViews.begin(), mSbViews.end(), sbView);
    if (it != mSbViews.end())
    {
        // printf("~!~! Detaching child soft body view %p\n", rbView);
        mSbViews.erase(it);
    }
}

void BaseSimulationView::_onChildRelease(const BaseSoftBodyMaterialView* sbView)
{
    auto it = std::find(mSbMaterialViews.begin(), mSbMaterialViews.end(), sbView);
    if (it != mSbMaterialViews.end())
    {
        // printf("~!~! Detaching child soft body material view %p\n", rbView);
        mSbMaterialViews.erase(it);
    }
}

void BaseSimulationView::_onChildRelease(const BaseVolumeDeformableBodyView* deformableView)
{
    auto it = std::find(mVolumeDeformableBodyViews.begin(), mVolumeDeformableBodyViews.end(), deformableView);
    if (it != mVolumeDeformableBodyViews.end())
    {
        // printf("~!~! Detaching child volume deformable body view %p\n", rbView);
        mVolumeDeformableBodyViews.erase(it);
    }
}

void BaseSimulationView::_onChildRelease(const BaseSurfaceDeformableBodyView* deformableView)
{
    auto it = std::find(mSurfaceDeformableBodyViews.begin(), mSurfaceDeformableBodyViews.end(), deformableView);
    if (it != mSurfaceDeformableBodyViews.end())
    {
        // printf("~!~! Detaching child surface deformable body view %p\n", rbView);
        mSurfaceDeformableBodyViews.erase(it);
    }
}

void BaseSimulationView::_onChildRelease(const BaseDeformableMaterialView* deformableView)
{
    auto it = std::find(mDeformableMaterialViews.begin(), mDeformableMaterialViews.end(), deformableView);
    if (it != mDeformableMaterialViews.end())
    {
        // printf("~!~! Detaching child deformable material view %p\n", rbView);
        mDeformableMaterialViews.erase(it);
    }
}

void BaseSimulationView::_onChildRelease(const BaseRigidContactView* rcView)
{
    auto it = std::find(mRcViews.begin(), mRcViews.end(), rcView);
    if (it != mRcViews.end())
    {
        // printf("~!~! Detaching child rigid contact view %p\n", rcView);
        mRcViews.erase(it);
    }
}

void BaseSimulationView::_onChildRelease(const BaseParticleSystemView* psView)
{
    auto it = std::find(mParticleSysViews.begin(), mParticleSysViews.end(), psView);
    if (it != mParticleSysViews.end())
    {
        // printf("~!~! Detaching child particle system view %p\n", psView);
        mParticleSysViews.erase(it);
    }
}

void BaseSimulationView::_onChildRelease(const BaseParticleClothView* pcView)
{
    auto it = std::find(mParticleClothViews.begin(), mParticleClothViews.end(), pcView);
    if (it != mParticleClothViews.end())
    {
        // printf("~!~! Detaching child particle cloth view %p\n", pcView);
        mParticleClothViews.erase(it);
    }
}

void BaseSimulationView::_onChildRelease(const BaseParticleMaterialView* pmView)
{
    auto it = std::find(mParticleMatViews.begin(), mParticleMatViews.end(), pmView);
    if (it != mParticleMatViews.end())
    {
        // printf("~!~! Detaching child particle material view %p\n", pmView);
        mParticleMatViews.erase(it);
    }
}

#define COLLECT_STEP_TIMINGS 0

void BaseSimulationView::step(float dt)
{
    //printf("WOO, step\n");

    if (!g_physxPrivate)
    {
        CARB_LOG_ERROR("%s: Failed to aquire PhysX private interface", __FUNCTION__);
        return;
    }

    PxScene* scene = g_physxPrivate->getPhysXScene();
    if (!scene)
    {
        CARB_LOG_ERROR("%s: Failed to get physics scene. Is the simulation active?", __FUNCTION__);
        return;
    }

#if COLLECT_STEP_TIMINGS
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;

    static int stepno = 1;
    static double tsum = 0.0f;

    TimePoint t1 = Clock::now();
#endif

    // step the physx scene directly
    scene->simulate(dt);
    scene->fetchResults(true);
    scene->fetchResultsParticleSystem();

#if COLLECT_STEP_TIMINGS
    TimePoint t2 = Clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() * 0.001;
    tsum += duration;
    ++stepno;
    //printf("~~~ Step %d, time: %.3f ms\n", stepno, duration);

    if (stepno == 600)
    {
        printf("Average step time: %.3f ms\n", tsum / stepno);
    }
#endif

    GetSimulationBackend().incrementStepCount();
}

PxMaterial* BaseSimulationView::createSharedMaterial(float staticFriction, float dynamicFriction, float restitution)
{
    std::string key;
    char keybuffer[100];
    snprintf(keybuffer, 100, "%.6f", staticFriction);
    key += std::string(keybuffer) + "_";
    snprintf(keybuffer, 100, "%.6f", dynamicFriction);
    key += std::string(keybuffer) + "_";
    snprintf(keybuffer, 100, "%.6f", restitution);
    key += std::string(keybuffer);

    std::unordered_map<std::string, PxMaterial*>::const_iterator got = mMaterials.find(key);

    PxMaterial* material = nullptr;

    // Exact material does not exist. Assign parameters to existing or make new one
    if (got == mMaterials.end())
    {
        // Find a material not in use by a shape
        if (mUnusedMaterials.size() > 0)
        {
            material = *mUnusedMaterials.begin();
            material->setStaticFriction(staticFriction);
            material->setDynamicFriction(dynamicFriction);
            material->setRestitution(restitution);

            mUnusedMaterials.erase(mUnusedMaterials.begin());
        }
        else
        {
            if (g_physxPrivate)
            {
                PxPhysics& physics = g_physxPrivate->getPhysXScene()->getPhysics();
                material = physics.createMaterial(staticFriction, dynamicFriction, restitution);
            }
        }

        if (material) {
            // Assign the new values to the material
            material->setFrictionCombineMode(PxCombineMode::eAVERAGE);
            material->setRestitutionCombineMode(PxCombineMode::eAVERAGE);
            material->setDampingCombineMode(PxCombineMode::eAVERAGE);
            mMaterials[key] = material;
        }
    }
    else
    {
        material = got->second;
    }

    if (mMaterialsRefCount.find(material) == mMaterialsRefCount.end())
    {
        mMaterialsRefCount[material] = 1;
    }
    else
    {
        mMaterialsRefCount[material] += 1;
    }

    return material;
}

bool BaseSimulationView::hasRigidBody(PxRigidBody* body) const
{
    if (rigidBodies.find(body) != rigidBodies.end())
        return true;
    else
        return false;
}

bool BaseSimulationView::hasArticulation(PxArticulationReducedCoordinate* arti) const
{
    if (articulations.find(arti) != articulations.end())
        return true;
    else
        return false;
}

bool BaseSimulationView::hasLink(PxArticulationLink* link) const
{
    if (links.find(link) != links.end())
        return true;
    else
        return false;
}

bool BaseSimulationView::hasShape(PxShape* shape) const
{
    if (shapes.find(shape) != shapes.end())
        return true;
    else
        return false;
}

bool BaseSimulationView::hasfixedTendon(PxArticulationFixedTendon* tendon) const
{
    if (fixedTendons.find(tendon) != fixedTendons.end())
        return true;
    else
        return false;
}
bool BaseSimulationView::hasSpatialTendon(PxArticulationSpatialTendon* tendon) const
{
    if (spatialTendons.find(tendon) != spatialTendons.end())
        return true;
    else
        return false;
}

void BaseSimulationView::_hackFixMaterialConnectivity()
{
    // This function worked around two issues
    // - Makes a call to GetPrimsWithTypeName so it implicitly syncs any pending Material population from USD to FSD.
    // - Material and Shader Fabric Prim has no connectivity info on them. Thus any hierarchy query (e.g. GetAllChildren)
    //   do not include them:
    //     - While FSD population code is being fixed (OMPE-55315), this function tries to patch the connectivity.

    long stageId = UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();
    usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(omni::fabric::UsdStageId(stageId));
    auto materialPrimPaths = usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("Material"));
    if (!materialPrimPaths.empty())
    {
        omni::fabric::Connectivity connectivity(usdrtStage->GetFabricId());
        connectivity.pushConnectionType(
            omni::fabric::connectivity::kParentEdgeType, omni::fabric::connectivity::kChildEdgeType);

        for (auto& path : materialPrimPaths)
        {
            connectivity.connectIfNot(
                static_cast<omni::fabric::PathC>(path), static_cast<omni::fabric::PathC>(path.GetParentPath()));
        }

        connectivity.popConnectionType();
    }
}
}
}
}
