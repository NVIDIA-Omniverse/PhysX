// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include <carb/profiler/Profile.h>

#include "UsdPhysicsUINoticeHandler.h"
#include "JointAuthoring.h"

using namespace omni::physics::ui;
using namespace pxr;

void UsdPhysicsUINoticeListener::handle(const pxr::UsdNotice::ObjectsChanged& objectsChanged)
{
    if (mBlockNoticeHandler || !mStage || mStage != objectsChanged.GetStage())
    {
        return;
    }
    TRACE_FUNCTION();

    if (mJointAuthoringManager)
    {
        CARB_PROFILE_ZONE(0, "UsdPhysicsUINoticeListener::JointAuthoringManager::handle");
        mJointAuthoringManager->handleUsdNotice(objectsChanged);
    }
}

void JointAuthoringManager::handleUsdNotice(const pxr::UsdNotice::ObjectsChanged& objectsChanged)
{
    const bool shouldHideJoints = (mVisibilityFlags & VisibilityFlags::eSHOW_JOINTS) == 0;
    if (!mStage || mStage != objectsChanged.GetStage())
    {
        return;
    }
    for (const SdfPath& path : objectsChanged.GetResyncedPaths())
    {
        const SdfPath primPath =
            mStage->GetPseudoRoot().GetPath() == path ? mStage->GetPseudoRoot().GetPath() : path.GetPrimPath();

        // If prim is removed, remove it and its descendants from selection.
        UsdPrim prim = mStage->GetPrimAtPath(primPath);
        if (prim.IsValid() == false || !prim.IsActive()) // remove prim
        {
            removePrim(primPath);
        }
        else // resync prim
        {
            const TfToken& attrToken = path.GetNameToken();
            if (attrToken == UsdPhysicsTokens.Get()->physicsBody0)
            {
                updateJointBodyPaths(primPath, true);
            }
            else if (attrToken == UsdPhysicsTokens.Get()->physicsBody1)
            {
                updateJointBodyPaths(primPath, false);
            }
            else if (!shouldHideJoints && attrToken == UsdGeomTokens->visibility)
            {
                updateJointVisibility(primPath);
            }
            else
            {
                if( attrToken == UsdPhysicsTokens.Get()->physicsLocalPos0 || 
                    attrToken == UsdPhysicsTokens.Get()->physicsLocalPos1)
                {
                    // If called from script it may happen that we get the calls to updateJointBodyPaths
                    // buffered but still not executed, and later on localPos0/1 changed.
                    // When this happens we need to disable the code that recomputes localPos0/1 from body positions.
                    // See OM-96674
                    clearBufferedUpdatesForJointBodyPaths(primPath);
                }
                resyncPrim(primPath);
            }
        }
    }

    for (const SdfPath& path : objectsChanged.GetChangedInfoOnlyPaths())
    {
        const SdfPath primPath = path.GetPrimPath();
        const TfToken& attrToken = path.GetNameToken();
        if (!shouldHideJoints && attrToken == UsdGeomTokens->visibility)
        {
            updateJointVisibility(primPath);
        }

        if (!empty() && !shouldHideJoints && !notificationBlocked())
        {
            if (UsdGeomXformable::IsTransformationAffectedByAttrNamed(attrToken))
            {
                updateJointTransformData(primPath);
            }
            else if (attrToken == UsdPhysicsTokens.Get()->physicsBody0)
            {
                updateJointBodyPaths(primPath, true);
            }
            else if (attrToken == UsdPhysicsTokens.Get()->physicsBody1)
            {
                updateJointBodyPaths(primPath, false);
            }
            else
            {
                if( attrToken == UsdPhysicsTokens.Get()->physicsLocalPos0 || 
                    attrToken == UsdPhysicsTokens.Get()->physicsLocalPos1)
                {
                    // See OM-96674
                    clearBufferedUpdatesForJointBodyPaths(primPath);

                    updateJointGizmoTransformFromPath(primPath);
                }
            }
        }
    }
}
