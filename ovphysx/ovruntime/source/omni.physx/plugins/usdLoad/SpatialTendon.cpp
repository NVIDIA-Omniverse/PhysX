// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/logging/Log.h>
#include <common/foundation/Allocator.h>

#include <ChangeRegister.h>
#include <propertiesUpdate/PhysXPropertiesUpdate.h>

#include "LoadTools.h"
#include "LoadUsd.h"
#include "AttributeHelpers.h"

using namespace PXR_NS;

namespace omni
{
namespace physx
{
namespace usdparser
{
    void createSpatialTendonAttachmentsRecursive(AttachedStage& attachedStage, const ObjectId parentId,
        const std::shared_ptr<PhysxTendonAttachmentDesc> parentDesc,
        TendonAttachmentMap& attachmentMap)
    {
        bool foundChild = false;
        // loop over attachments that point to the given parent's link/Xform
        for (TendonAttachmentMap::mapped_type::const_reference attachmentRef : attachmentMap[parentDesc->linkPath])
        {
            // ensure that the child does not point to another attachment on parent link/Xform
            if (attachmentRef->parentToken == parentDesc->instanceToken)
            {
                foundChild = true;

                attachmentRef->parentId = parentId;
                const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, attachmentRef->linkPath, *attachmentRef);

                if (id == kInvalidObjectId)
                {
                    // User has already been warned in createObject();
                    continue;
                }

                attachedStage.getObjectDatabase()->findOrCreateEntry(attachmentRef->linkPath, eTendonAttachment, id);

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
                parentDesc->instanceToken.GetText(), parentDesc->linkPath.GetText());
        }
    }

    void createSpatialTendons(AttachedStage& attachedStage, TendonAttachmentMap& attachmentMap, SpatialTendonVector& spatialTendons)
    {        
        ObjectDb* objDb = attachedStage.getObjectDatabase();

        for (SpatialTendonVector::const_reference tendonRootDesc : spatialTendons)
        {
            const SdfPath& rootPath = tendonRootDesc->linkPath;

            const ObjectId attachmentId = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, rootPath, *tendonRootDesc);

            if (attachmentId == kInvalidObjectId)
            {
                // warning already there in createObject();
                continue;
            }

            objDb->findOrCreateEntry(rootPath, eTendonAttachment, attachmentId);

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
                        attachment->instanceToken.GetText(), attachmentVec.first.GetText());
                }
            }
        }
    }

} // namespace usdparser
} // namespace physx
} // namespace omni
