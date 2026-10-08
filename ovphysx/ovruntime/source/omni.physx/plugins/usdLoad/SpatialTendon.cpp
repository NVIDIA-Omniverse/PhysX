// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27
 */

#include <carb/logging/Log.h>
#include <common/foundation/Allocator.h>

#include "LoadTools.h"
#include "LoadUsd.h"
// AttachedStage.h only forward-declares PhysXUsdPhysicsInterface; this TU calls
// into it (createObject), so it needs the real definition.
#include <usdInterface/UsdInterface.h>
// PhysXPropertiesUpdate.h / AttributeHelpers.h: unused by this file's own body (no
// getAttribute<>/REGISTER_CHANGE call sites), but both unconditionally drag in
// PhysXTools.h -> OmniPhysX.h -> internal/InternalScene.h's real InternalActor.h/
// InternalParticle.h chain. Dropped rather than fenced since nothing here actually needs them.

namespace omni
{
namespace physx
{
namespace usdparser
{

    // pxr-free diagnostic mirror of tfTokenFor(...).GetText(): resolve a TokenId
    // to plain text for %s log formatting, without materializing a TfToken.
    // Returns "" when the source/token is invalid.
    static std::string tokenText(const AttachedStage& attachedStage, omni::physics::parse::TokenId token)
    {
        const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
        return source ? std::string(source->tokenToString(token)) : std::string();
    }

    // linkKey/parentToken/instanceToken on PhysxTendonAttachmentDesc are
    // ObjectKey/TokenId (ADR-0019 increment 7); TendonAttachmentMap is now
    // ObjectKey-keyed too (LoadTools.h), so the map lookup below keys off
    // parentDesc->linkKey directly. createObject/findOrCreateEntry have
    // ObjectKey-native overloads (UsdInterface.h/LoadTools.h); diagnostics
    // resolve through AttachedStage::textFor / IPhysicsSource::tokenToString
    // rather than materializing an SdfPath/TfToken, so the whole function is
    // pxr-free.
    void createSpatialTendonAttachmentsRecursive(AttachedStage& attachedStage, const ObjectId parentId,
        const std::shared_ptr<PhysxTendonAttachmentDesc> parentDesc,
        TendonAttachmentMap& attachmentMap)
    {
        bool foundChild = false;
        // loop over attachments that point to the given parent's link/Xform
        for (TendonAttachmentMap::mapped_type::const_reference attachmentRef : attachmentMap[parentDesc->linkKey])
        {
            // ensure that the child does not point to another attachment on parent link/Xform
            if (attachmentRef->parentToken == parentDesc->instanceToken)
            {
                foundChild = true;

                attachmentRef->parentId = parentId;
                const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, attachmentRef->linkKey, *attachmentRef);

                if (id == kInvalidObjectId)
                {
                    // User has already been warned in createObject();
                    continue;
                }

                attachedStage.getObjectDatabase()->findOrCreateEntry(
                    attachmentRef->linkKey, attachedStage.textFor(attachmentRef->linkKey), eTendonAttachment, id);

                // create children if not leaf
                if (attachmentRef->type != eTendonAttachmentLeaf)
                {
                    createSpatialTendonAttachmentsRecursive(attachedStage, id, attachmentRef, attachmentMap);
                }
            }
        }

        if (!foundChild)
        {
            CARB_LOG_ERROR("Could not find any children of non-leaf tendon attachment '%s' at %s. Please check topology.",
                tokenText(attachedStage, parentDesc->instanceToken).c_str(),
                attachedStage.textFor(parentDesc->linkKey));
        }
    }

    void createSpatialTendons(AttachedStage& attachedStage, TendonAttachmentMap& attachmentMap, SpatialTendonVector& spatialTendons)
    {
        ObjectDb* objDb = attachedStage.getObjectDatabase();

        for (SpatialTendonVector::const_reference tendonRootDesc : spatialTendons)
        {
            const ObjectId attachmentId = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, tendonRootDesc->linkKey, *tendonRootDesc);

            if (attachmentId == kInvalidObjectId)
            {
                // warning already there in createObject();
                continue;
            }

            objDb->findOrCreateEntry(
                tendonRootDesc->linkKey, attachedStage.textFor(tendonRootDesc->linkKey), eTendonAttachment, attachmentId);

            createSpatialTendonAttachmentsRecursive(attachedStage, attachmentId, tendonRootDesc, attachmentMap);
        }

        // give out warnings for unparsed tendons
        for (TendonAttachmentMap::reference attachmentVec : attachmentMap)
        {
            for (TendonAttachmentMap::mapped_type::reference attachment : attachmentVec.second)
            {
                // parent id was set if attachment was parsed
                if (attachment->parentId == kInvalidObjectId)
                {
                    CARB_LOG_WARN("The spatial tendon attachment '%s' was not parsed, because its supposed parent was declared a leaf or could not be found at %s.",
                        tokenText(attachedStage, attachment->instanceToken).c_str(),
                        attachedStage.textFor(attachmentVec.first));
                }
            }
        }
    }

} // namespace usdparser
} // namespace physx
} // namespace omni
