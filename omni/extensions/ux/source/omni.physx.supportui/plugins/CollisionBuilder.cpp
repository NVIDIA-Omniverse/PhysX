// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <pxr/usd/usdPhysics/tokens.h>
#include <physxSchema/tokens.h>

#include <common/utilities/Utilities.h>
#include <carb/profiler/Profile.h>

#include <private/omni/physx/IPhysxSupportUi.h>
#include <omni/kit/EditorUsd.h>

#include "AddToStageHelper.h"
#include "PrimUpdateMap.h"
#include "Utils.h"
#include "UsdUtils.h"

OMNI_LOG_DECLARE_CHANNEL(kSupportUiLogChannel)

using namespace pxr;

namespace omni
{
namespace physx
{

CollisionBuilder::CollisionBuilder(size_t numPrimsToBatchProcess)
    : mNumPrimsToBatchProcess(numPrimsToBatchProcess)
{
    mDictionary = carb::getCachedInterface<carb::dictionary::IDictionary>(); // Get the carbonite dictionary interface
    CARB_ASSERT(mDictionary != nullptr, "CollisionBuilder: Failed to obtain dictionary interface!");
}

CollisionBuilder::~CollisionBuilder()
{
    mEventStream = nullptr;
}

void CollisionBuilder::reset()
{
    mNumTotalItems = 0;
    mNumProcessedItems = 0;
    mRigidBodiesForAdd.clear();
    mRigidBodiesForRemoval.clear();
    mPrimsToCreateColliders.clear();
}

void CollisionBuilder::setEventStream(carb::events::IEventStreamPtr eventStream)
{
    mEventStream = eventStream;
}

// returns invalid prim if not found
UsdPrim CollisionBuilder::isRigidBodyOnNodeOrUpInHierarchy(UsdPrim startingPrim,
                                                           const TRigidBodiesForAddContainer& rigidBodiesForAdd,
                                                           const TRigidBodiesForRemovalContainer& rigidBodiesForRemoval,
                                                           std::unordered_set<SdfPath, SdfPath::Hash>& primsWithRbParents)
{
    UsdPrim invalidPrim;
    std::unordered_set<SdfPath, SdfPath::Hash> tmpPrimsWithRbParent;

    UsdPrim parent = startingPrim;
    while (parent && !parent.IsPseudoRoot())
    {
        tmpPrimsWithRbParent.insert(parent.GetPath());

        bool foundInCache = primsWithRbParents.find(parent.GetPath()) != primsWithRbParents.end();
        if (foundInCache || ((rigidBodiesForAdd.find(parent.GetPath()) != rigidBodiesForAdd.end() ||
                              parent.HasAPI<UsdPhysicsRigidBodyAPI>()) &&
                             rigidBodiesForRemoval.find(parent.GetPath()) == rigidBodiesForRemoval.end()))
        {
            // update the primsWithRbParents cache
            primsWithRbParents.insert(tmpPrimsWithRbParent.begin(), tmpPrimsWithRbParent.end());

            return parent;
        }

        parent = parent.GetParent();
    }

    return invalidPrim;
}

bool CollisionBuilder::hasGeomPrimInstancedProxyChild(UsdPrim prim)
{
    if (!prim.IsValid())
    {
        return false;
    }
    
    UsdPrimRange primRange(prim, UsdTraverseInstanceProxies());

    for (UsdPrimRange::const_iterator iter = primRange.begin(); iter != primRange.end(); ++iter)
    {
        const UsdPrim& subPrim = *iter;
        if (!subPrim.IsValid() || subPrim.IsPseudoRoot() || omni::kit::EditorUsd::isHideInStageWindow(subPrim))
        {
            continue;
        }

        if (subPrim.IsA<UsdGeomGprim>())
        {
            return true;
        }
    }

    return false;
}

// No API changes are being made in this method. All are done in createColliders().
// User induced coll. creation has valid PrimDef.mPrim (it's the selected one),
// otherwise is not valid (for example auto-coll. creation upon opening a stage provides only primRange)
void CollisionBuilder::enumerateForColliderCreation(UsdStageWeakPtr stage,
                                                    const std::vector<PrimDef>& primDefs,
                                                    bool avoidChangingExistingColliders)
{
    CARB_PROFILE_ZONE(0, "PhysXSupportUi CollisionBuilder::enumerateForColliderCreation");

    // cache of prims that have a parent which has rigid body component
    std::unordered_set<SdfPath, SdfPath::Hash> primsWithRbParent;
    TRigidBodiesForRemovalContainer rigidBodiesForRemoval;
    TRigidBodiesForAddContainer rigidBodiesForAdd;

    // Show the cancellation message only when there was at least one collider to be created.
    // Otherwise there was nothing to cancel, really.
    bool sendCancellationMessage = false; 
    UsdPrim showStopperPrim; // a prim that already contains some physics. In non-enforced mode it means stop.

    for (const auto& primDef : primDefs)
    {
        OMNI_LOG_INFO(kSupportUiLogChannel, "enumerateForColliderCreation: \"%s\" - inspecting...",
                      primDef.mPrim.GetName().GetText());

        // prims to create collider on
        std::unordered_set<SdfPath, SdfPath::Hash> collPrims;

        UsdPrimRange primRange(primDef.mPrim); // do not iterate over instanced proxies
        for (UsdPrimRange::const_iterator iter = primRange.begin(); iter != primRange.end(); ++iter)
        {  
            const UsdPrim& subPrim = *iter;
            const SdfPath subPrimPath = subPrim.GetPath();

            if (!subPrim.IsValid() || subPrim.IsPseudoRoot())
            {
                OMNI_LOG_INFO(kSupportUiLogChannel, "\"%s\" - invalid / DebugViz - SKIPPING",
                              subPrimPath.GetText());
                continue;
            }

            if (omni::kit::EditorUsd::isHideInStageWindow(subPrim))
            {
                OMNI_LOG_INFO(kSupportUiLogChannel, "\"%s\" - hidden in Stage Window - SKIPPING",
                              subPrimPath.GetText());

                iter.PruneChildren();

                continue;
            }

            if (subPrim.IsInstanceProxy())
            {
                // This prim is scene graph instanced and we cannot modify it in any way.
                // The collider must be set to its instanced parent.
                // FIXME: We ignore instance proxies now, so it will never get here.

                iter.PruneChildren();

                continue;
            }

            if (subPrim.IsA<UsdGeomPointInstancer>())
            {
                iter.PruneChildren();
            }

            if (!primDef.mForceCollCreation)
            {
                if (hasPhysicsPhysXSchemaApplied(subPrim))
                {
                    // in non-enforced mode, a prim with an existing physics or physx schema is a reason for cancelation.
                    OMNI_LOG_WARN(kSupportUiLogChannel,
                        "\"%s\" - has existing physics/physx schema(s) applied",
                        subPrimPath.GetText());

                    showStopperPrim = subPrim;
                }
            }

            if (subPrim.HasAPI<UsdPhysicsRigidBodyAPI>())
            {
                // remove existing rigid body (if any) in case of static collider
                if (primDef.mColliderType == IPhysxSupportUi::ColliderType::eStatic)
                {
                    rigidBodiesForRemoval.insert(subPrimPath);
                }
            }

            // if "component kind" is encountered, rigid body creation is queued
            if (primDef.mColliderType == IPhysxSupportUi::ColliderType::eDynamic
                && isComponentKind(subPrim)
                && !subPrim.HasAPI<UsdPhysicsRigidBodyAPI>())
            {
                // There can be another rigid body up in the hierarchy, we will check for that later.
                // Creation of nested RBs even on the component kind is forbidden!
                rigidBodiesForAdd.insert(subPrimPath);
            }

            if (subPrim.IsA<UsdGeomGprim>()
                || (subPrim.IsInstanceable() && hasGeomPrimInstancedProxyChild(subPrim)))
            {
                collPrims.insert(subPrimPath);
            }

            if (subPrim.HasAPI<UsdPhysicsCollisionAPI>())
            {
                if (primDef.mForceCollCreation)
                {
                    // We will add this prim to the container as the collider simplification type may be different.
                    // it will be resolved later on.
                    collPrims.insert(subPrimPath);
                }
                else
                {
                    OMNI_LOG_INFO(kSupportUiLogChannel,
                        "\"%s\" - existing collider found, leaving it intact!",
                        subPrimPath.GetText());

                    collPrims.erase(subPrimPath);
                }
            }
        }

        if (collPrims.size() > 0 && !sendCancellationMessage)
        {
            // handle rigid body creation / removal
            for (const auto& collPrimPath : collPrims)
            {
                UsdPrim prim = stage->GetPrimAtPath(collPrimPath);

                UsdPrim rbPrim = isRigidBodyOnNodeOrUpInHierarchy(prim,
                                                                  rigidBodiesForAdd,
                                                                  rigidBodiesForRemoval,
                                                                  primsWithRbParent);

                IPhysxSupportUi::ColliderType collType = primDef.mColliderType;

                if (primDef.mColliderType == IPhysxSupportUi::ColliderType::eAutodetect)
                {
                    collType = rbPrim ? IPhysxSupportUi::ColliderType::eDynamic : IPhysxSupportUi::ColliderType::eStatic;
                }

                switch (collType)
                {
                case IPhysxSupportUi::ColliderType::eDynamic: {

                    if (!rbPrim)
                    {
                        CARB_ASSERT(!primDef.mPrim.IsPseudoRoot(),
                            "This should not happen. This means that coll. detection on stage root failed (should have been static coll. type)!");

                        OMNI_LOG_INFO(
                            kSupportUiLogChannel,
                            "\"%s\" - Dynamic collider type requested and no rigid body was found anyhere in hierarchy - will create one on \"%s\".",
                            collPrimPath.GetText(), primDef.mPrim.GetPath().GetText());

                        rigidBodiesForAdd.insert(primDef.mPrim.GetPath());
                    }
                }
                break;

                case IPhysxSupportUi::ColliderType::eStatic: {
                    while(rbPrim)
                    {
                        rigidBodiesForRemoval.insert(rbPrim.GetPath());

                        OMNI_LOG_WARN(
                            kSupportUiLogChannel,
                            "\"%s\" - Static collider type requested but rigid body on \"%s\" was found in parent hierarchy - will be removed!",
                            collPrimPath.GetText(), rbPrim.GetPath().GetText());

                        // check again
                        rbPrim = isRigidBodyOnNodeOrUpInHierarchy(rbPrim.GetParent(),
                                                                  rigidBodiesForAdd,
                                                                  rigidBodiesForRemoval,
                                                                  primsWithRbParent);
                    }
                case IPhysxSupportUi::ColliderType::eAutodetect:
                break;
                }
                break;
                }

                // now let's skip this prim if its current collider simplification type matches the requested type.
                if (isCollApproximationPresent(prim,
                                               primDef.mColliderType,
                                               primDef.mStaticColliderSimplificationType,
                                               primDef.mDynamicColliderSimplificationType))
                {
                    OMNI_LOG_INFO(kSupportUiLogChannel,
                                  "\"%s\" - existing collider matching simplification type found, skipping.",
                                  collPrimPath.GetText());

                    continue;
                }

                if (showStopperPrim.IsValid())
                {
                    sendCancellationMessage = true;
                    break; // remember, we don't want to create any collider
                }

                if (avoidChangingExistingColliders)
                {
                    // Detect current approximation and decide, if we can keep it or not.

                    const TfToken& tApprox = UsdPhysicsTokens.Get()->physicsApproximation;
                    const auto approximationAPI = UsdPhysicsMeshCollisionAPI::Get(prim.GetStage(), prim.GetPrimPath());

                    // if there is no approximation, then we bail out and proceed normally
                    if (approximationAPI)
                    {
                        TfToken currentApproximation;
                        approximationAPI.GetApproximationAttr().Get(&currentApproximation);

                        if (collType == IPhysxSupportUi::ColliderType::eStatic)
                        {
                            // conversion from dynamic to static -> all dynamic collider types are supported.

                            OMNI_LOG_INFO(
                                kSupportUiLogChannel,
                                "\"%s\" - conversion to static - keeping existing collider approximation \"%s\", because AvoidChangingExistingColliders setting is ON.",
                                collPrimPath.GetText(), currentApproximation.GetText());

                            continue;
                        }
                        else if (collType == IPhysxSupportUi::ColliderType::eDynamic)
                        {
                            // conversion from static to dynamic -> triangle, mesh simplification unsupported.
                            if (currentApproximation == UsdPhysicsTokens.Get()->none ||
                                currentApproximation == UsdPhysicsTokens.Get()->meshSimplification)
                            {
                                OMNI_LOG_INFO(
                                    kSupportUiLogChannel,
                                    "\"%s\" - conversion to dynamic - cannot keep existing collider approximation \"%s\", even when AvoidChangingExistingColliders setting is ON, because it's unsupported.",
                                    collPrimPath.GetText(), currentApproximation.GetText());
                            }
                            else
                            {
                                OMNI_LOG_INFO(
                                    kSupportUiLogChannel,
                                    "\"%s\" - conversion to dynamic - keeping existing collider approximation \"%s\", because AvoidChangingExistingColliders setting is ON.",
                                    collPrimPath.GetText(), currentApproximation.GetText());

                                continue;
                            }
                        }
                    }
                }

                PrimCollDef primCollDef;
                primCollDef.mDynamicBody = primDef.mColliderType == IPhysxSupportUi::ColliderType::eDynamic;
                primCollDef.mStaticColliderSimplificationType = primDef.mStaticColliderSimplificationType;
                primCollDef.mDynamicColliderSimplificationType = primDef.mDynamicColliderSimplificationType;
                primCollDef.mPrimPath = collPrimPath;

                mPrimsToCreateColliders.push_back(primCollDef);

                mNumTotalItems++;
            }
        }
    }

    if (sendCancellationMessage)
    {
        sendAutoCollCanceledChangedEvent(mEventStream, mDictionary, showStopperPrim.GetPath());
    }

    if (showStopperPrim.IsValid())
    {
        return; // early exit
    }

    // Now check rigid body to be added hierarchy
    for (const auto& primPath : rigidBodiesForAdd)
    {
        UsdPrim prim = stage->GetPrimAtPath(primPath);

        if (!prim || prim.IsPseudoRoot())
        {
            continue;
        }

        UsdPrim rbPrim = isRigidBodyOnNodeOrUpInHierarchy(prim.GetParent(),
                                                          rigidBodiesForAdd,
                                                          rigidBodiesForRemoval,
                                                          primsWithRbParent);
        if (rbPrim)
        {
            // if there is a rigid body up in the hierarchy, we cannot create another one here
            // Log this even if the supportui channel is off
            OMNI_LOG_WARN(
                kSupportUiLogChannel,
                "\"%s\" - Rigid body creation skipped on this prim, because another rigid body was found in upper hierarchy on \"%s\".",
                primPath.GetText(), rbPrim.GetPath().GetText());
        }
        else
        {
            mRigidBodiesForAdd.insert(primPath);
        }
    }

    for (const auto& primPath : rigidBodiesForRemoval)
    {
        mRigidBodiesForRemoval.insert(primPath);
    }
}

void CollisionBuilder::createColliders(UsdStageWeakPtr stage,
                                       std::function<bool(const SdfPath& primPath, bool created)> rigidBodyChangeCallback,
                                       std::function<bool(const PrimCollDef& primCollDef)> creationCallback)
{
    CARB_PROFILE_ZONE(0, "PhysXSupportUi CollisionBuilder::createColliders");

    if (mRigidBodiesForRemoval.size() > 0)
    {
        OMNI_LOG_INFO(kSupportUiLogChannel, "Removing [%zux] rigid body API", mRigidBodiesForRemoval.size());

        for (const auto& primPath : mRigidBodiesForRemoval)
        {
            UsdPrim prim = stage->GetPrimAtPath(primPath);
            prim.RemoveAPI<UsdPhysicsRigidBodyAPI>();

            if (rigidBodyChangeCallback != nullptr && !rigidBodyChangeCallback(primPath, false))
            {
                break;
            }
        }

        mRigidBodiesForRemoval.clear();
    }

    if (mRigidBodiesForAdd.size() > 0)
    {
        OMNI_LOG_INFO(kSupportUiLogChannel, "Adding [%zux] rigid body API", mRigidBodiesForAdd.size());

        for (const auto& primPath : mRigidBodiesForAdd)
        {
            UsdPrim prim = stage->GetPrimAtPath(primPath);
            UsdPhysicsRigidBodyAPI::Apply(prim);

            if (rigidBodyChangeCallback != nullptr && !rigidBodyChangeCallback(primPath, true))
            {
                break;
            }
        }

        mRigidBodiesForAdd.clear();
    }

    if (mPrimsToCreateColliders.size() == 0)
    {
        return; // nothing to do 
    }

    size_t numItemsToProcess = std::min(mNumPrimsToBatchProcess, mPrimsToCreateColliders.size());
    {
        CARB_PROFILE_ZONE(0, "PhysXSupportUi batched UsdPhysicsCollisionAPI::Apply");

        while (numItemsToProcess-- > 0)
        {
            auto& primCollDef = mPrimsToCreateColliders.front();
            const SdfPath& primPath = primCollDef.mPrimPath;
            UsdPrim prim = stage->GetPrimAtPath(primPath);

            if (creationCallback != nullptr && !creationCallback(primCollDef))
            {
                break;
            }

            // make sure UsdPhysicsCollisionAPI exists or gets created
            if (!prim.HasAPI<UsdPhysicsCollisionAPI>())
            {
                UsdPhysicsCollisionAPI::Apply(prim);
            }

            const TfToken& tApprox = UsdPhysicsTokens.Get()->physicsApproximation;
            TfToken currentApproximation;
            const auto approximationAPI = UsdPhysicsMeshCollisionAPI::Get(prim.GetStage(), prim.GetPrimPath());
            bool tokenAttrSet = true;

            if (approximationAPI)
            {
                if (primCollDef.mStaticColliderSimplificationType == IPhysxSupportUi::StaticColliderSimplificationType::eNone &&
                    !primCollDef.mDynamicBody)
                {
                    tokenAttrSet = setTokenAttributeWithSdf(prim.GetStage(), prim, tApprox, UsdPhysicsTokens.Get()->none);
                }
                else
                {
                    approximationAPI.GetApproximationAttr().Get(&currentApproximation);
                }
            }
            else
            {
                // create default triangle mesh approximation
                UsdPhysicsMeshCollisionAPI::Apply(prim);
            }

            if (primCollDef.mDynamicBody)
            {
                switch (primCollDef.mDynamicColliderSimplificationType)
                {
                case IPhysxSupportUi::DynamicColliderSimplificationType::eConvexHull:
                    if (currentApproximation != UsdPhysicsTokens.Get()->convexHull)
                    {
                        tokenAttrSet = setTokenAttributeWithSdf(prim.GetStage(), prim, tApprox, UsdPhysicsTokens.Get()->convexHull);
                    }
                    break;
                case IPhysxSupportUi::DynamicColliderSimplificationType::eConvexDecomposition:
                    if (currentApproximation != UsdPhysicsTokens.Get()->convexDecomposition)
                    {
                        tokenAttrSet = setTokenAttributeWithSdf(prim.GetStage(), prim, tApprox, UsdPhysicsTokens.Get()->convexDecomposition);
                    }
                    break;
                case IPhysxSupportUi::DynamicColliderSimplificationType::eSDF:
                    if (currentApproximation != PhysxSchemaTokens.Get()->sdf)
                    {
                        auto physxSDFCollisionAPI = PhysxSchemaPhysxSDFMeshCollisionAPI::Get(prim.GetStage(), prim.GetPrimPath());
                        physxSDFCollisionAPI.Apply(prim);
                        tokenAttrSet = setTokenAttributeWithSdf(prim.GetStage(), prim, tApprox, PhysxSchemaTokens.Get()->sdf);
                    }
                    break;
                default:
                    CARB_LOG_ERROR("Unknown dynamic collider simplification type (%d)", static_cast<int>(primCollDef.mDynamicColliderSimplificationType));
                }
            }
            else
            {
                switch (primCollDef.mStaticColliderSimplificationType)
                {
                case IPhysxSupportUi::StaticColliderSimplificationType::eNone:
                    // triangle mesh is created by default, nothing to do
                    break;
                case IPhysxSupportUi::StaticColliderSimplificationType::eMeshSimplification: {
                    if (currentApproximation != UsdPhysicsTokens.Get()->meshSimplification)
                    {
                        tokenAttrSet = setTokenAttributeWithSdf(prim.GetStage(), prim, tApprox, UsdPhysicsTokens.Get()->meshSimplification);
                    }
                }
                break;
                default:
                    CARB_LOG_ERROR("Unknown static collider simplification type (%d)", static_cast<int>(primCollDef.mStaticColliderSimplificationType));
                }
            }

            if (!tokenAttrSet)
            {
                CARB_LOG_ERROR("Failed to set approximation attr for prim \"%s\"!", prim.GetPath().GetText());
            }

            mNumProcessedItems++;
            mPrimsToCreateColliders.pop_front();
        }
    }

    if (mPrimsToCreateColliders.size() == 0) // we emptied the container, reset values
    {
        reset();
    }
}

bool CollisionBuilder::isCollApproximationPresent(UsdPrim prim,
                                                  IPhysxSupportUi::ColliderType collType,
                                                  IPhysxSupportUi::StaticColliderSimplificationType staticCollSimplType,
                                                  IPhysxSupportUi::DynamicColliderSimplificationType dynamicCollSimplType)
{
    const TfToken& tApprox = UsdPhysicsTokens.Get()->physicsApproximation;
    const auto approximationAPI = UsdPhysicsMeshCollisionAPI::Get(prim.GetStage(), prim.GetPrimPath());

    if (approximationAPI)
    {
        TfToken currentApproximation;
        approximationAPI.GetApproximationAttr().Get(&currentApproximation);

        if (collType == IPhysxSupportUi::ColliderType::eStatic)
        {
            switch (staticCollSimplType)
            {
            case IPhysxSupportUi::StaticColliderSimplificationType::eNone:
                return currentApproximation == UsdPhysicsTokens.Get()->none;
            case IPhysxSupportUi::StaticColliderSimplificationType::eMeshSimplification:
                return currentApproximation == UsdPhysicsTokens.Get()->meshSimplification;
            }
        }
        else if (collType == IPhysxSupportUi::ColliderType::eDynamic)
        {
            switch (dynamicCollSimplType)
            {
            case IPhysxSupportUi::DynamicColliderSimplificationType::eConvexHull:
                return currentApproximation == UsdPhysicsTokens.Get()->convexHull;
            case IPhysxSupportUi::DynamicColliderSimplificationType::eConvexDecomposition:
                return currentApproximation == UsdPhysicsTokens.Get()->convexDecomposition;
            case IPhysxSupportUi::DynamicColliderSimplificationType::eSDF:
                return currentApproximation == PhysxSchemaTokens.Get()->sdf;
            }
        }
    }

    return false;
}

} // namespace physx
} // namespace omni
