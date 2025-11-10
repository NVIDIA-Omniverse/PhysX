// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

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

using namespace pxr;

namespace omni
{
namespace physx
{
namespace usdparser
{
    static const TfToken g_tendonAttachment("PhysxTendonAttachmentAPI:");
    static const TfToken g_tendonAttachmentRoot("PhysxTendonAttachmentRootAPI:");
    static const TfToken g_tendonAttachmentLeaf("PhysxTendonAttachmentLeafAPI:");

    void parseAttachment(AttachedStage& attachedStage, const PhysxSchemaPhysxTendonAttachmentAPI& attachmentApi,
        PhysxTendonAttachmentDesc* desc, UsdGeomXformCache& xfCache, const UsdPrim& linkPrim)
    {
        // setup change listeners & allow time-sampled gearing values
        registerTendonAttachmentChangeParams(attachedStage, attachmentApi.GetName().GetString());

        if (desc->type != eTendonAttachmentRoot)
        {
            getAttribute(desc->gearing, attachmentApi.GetGearingAttr(), -FLT_MAX, FLT_MAX, updateTendonAttachmentGearing);
        }

        // get position of attachment in link local frame
        GfVec3f localPos(0.f);
        // attachment located on link, so just get property
        attachmentApi.GetLocalPosAttr().Get(&localPos);

        const pxr::GfTransform attachmentTrans = xfCache.GetLocalToWorldTransform(linkPrim);
        const pxr::GfVec3f scale(attachmentTrans.GetScale());
        localPos = GfCompMult(localPos, scale);

        GfVec3ToFloat3(localPos, desc->localPos);

        // allow time-sampled values
        if (attachmentApi.GetLocalPosAttr().ValueMightBeTimeVarying())
        {
            UsdLoad::getUsdLoad()->registerTimeSampledAttribute(attachmentApi.GetLocalPosAttr(), updateTendonAttachmentLocalPos);
        }

        // extract some more topology information
        desc->instanceToken = attachmentApi.GetName();
        desc->parentId = kInvalidObjectId;
        desc->linkPath = linkPrim.GetPath();

        SdfPathVector targets;
        attachmentApi.GetParentLinkRel().GetTargets(&targets);
        if(targets.size() == 0)
        {
            desc->parentPath =  SdfPath();
        }else
        {
            if(targets.size() > 1)
            {
                CARB_LOG_WARN("Attachment at %s has relationship to multiple parents! Will use parent at 0 position of array.",
                              attachmentApi.GetPath().GetText());
            }
            desc->parentPath = targets[0];
        }
        attachmentApi.GetParentAttachmentAttr().Get(&desc->parentToken);
    }

    void parseLeafAttachment(AttachedStage& attachedStage, const PhysxSchemaPhysxTendonAttachmentLeafAPI& leafApi,
        PhysxTendonAttachmentLeafDesc* leafDesc, UsdGeomXformCache& xfCache, const UsdPrim& linkPrim)
    {
        // setup change listeners
        registerTendonAttachmentLeafChangeParams(attachedStage, leafApi.GetName().GetString());

        // use helper to enforce value ranges & enable time sampling
        getAttribute(leafDesc->restLength, leafApi.GetRestLengthAttr(), -FLT_MAX, FLT_MAX, updateTendonAttachmentLeafRestLength); // allow negative sentinel value
        getAttribute(leafDesc->lowLimit, leafApi.GetLowerLimitAttr(), -FLT_MAX, FLT_MAX, updateTendonAttachmentLeafLowLimit);
        getAttribute(leafDesc->highLimit, leafApi.GetUpperLimitAttr(), leafDesc->lowLimit, FLT_MAX, updateTendonAttachmentLeafHighLimit);

        // use inheritance to parse basic attributes
        parseAttachment(attachedStage, PhysxSchemaPhysxTendonAttachmentAPI(leafApi, leafApi.GetName()), leafDesc, xfCache, linkPrim);
    }

    void parseRootAttachment(AttachedStage& attachedStage, const PhysxSchemaPhysxTendonAttachmentRootAPI& rootApi,
        PhysxTendonSpatialDesc* rootDesc, UsdGeomXformCache& xfCache, const UsdPrim& linkPrim)
    {
        // setup change listeners
        registerSpatialTendonChangeParams(attachedStage, rootApi.GetName().GetString());

        // use helper to enforce value ranges & enable time sampling
        getBoolAttribute(rootDesc->isEnabled, rootApi.GetTendonEnabledAttr(), updateSpatialTendonEnabled);
        getAttribute(rootDesc->stiffness, rootApi.GetStiffnessAttr(), 0.0f, FLT_MAX, updateSpatialTendonStiffness);
        getAttribute(rootDesc->limitStiffness, rootApi.GetLimitStiffnessAttr(), 0.0f, FLT_MAX, updateSpatialTendonLimitStiffness);
        getAttribute(rootDesc->damping, rootApi.GetDampingAttr(), 0.0f, FLT_MAX, updateSpatialTendonDamping);
        getAttribute(rootDesc->offset, rootApi.GetOffsetAttr(), -FLT_MAX, FLT_MAX, updateSpatialTendonOffset);

        // use inheritance to parse basic attributes
        parseAttachment(attachedStage, PhysxSchemaPhysxTendonAttachmentAPI(rootApi, rootApi.GetName()), rootDesc, xfCache, linkPrim);

        // ignore gearing attribute for root
        rootDesc->gearing = 1.f;
    }

    bool isNoDuplicate(const PhysxSchemaPhysxTendonAttachmentAPI& api, TfToken::Set& instanceNames)
    {
        if (instanceNames.insert(api.GetName()).second)
        {
            return true;
        }

        CARB_LOG_ERROR("More than one tendon attachment instance with name %s was applied at prim %s.",
            api.GetName().GetText(), api.GetPath().GetText());
        return false;
    }

    void parseTendonAttachments(AttachedStage& attachedStage, const UsdPrim& prim, UsdGeomXformCache& xfCache, TendonAttachmentMap& attachmentMap,
        SpatialTendonVector& spatialTendons)
    {
        // assert preconditions
        CARB_ASSERT(prim.HasAPI<PhysxSchemaPhysxTendonAttachmentAPI>());
        CARB_ASSERT(prim.HasAPI<UsdPhysicsRigidBodyAPI>());
        CARB_ASSERT(prim.IsA<UsdGeomXformable>());

        // loop over applied APIs and parse the descriptions
        TfTokenVector appliedSchemas = prim.GetPrimTypeInfo().GetAppliedAPISchemas();
        const size_t attachmentLen = g_tendonAttachment.GetString().length();
        const size_t leafLen = g_tendonAttachmentLeaf.GetString().length();
        const size_t rootLen = g_tendonAttachmentRoot.GetString().length();
        TfToken::Set attachInstanceNames;
        for (auto token : appliedSchemas)
        {
            if (token.GetString().length() > attachmentLen &&
                token.GetString().substr(0, attachmentLen) == g_tendonAttachment.GetString())
            {
                PhysxSchemaPhysxTendonAttachmentAPI attachmentApi =
                    PhysxSchemaPhysxTendonAttachmentAPI::Get(prim, TfToken(token.GetString().substr(attachmentLen)));
                if (attachmentApi && isNoDuplicate(attachmentApi, attachInstanceNames))
                {
                    std::shared_ptr<PhysxTendonAttachmentDesc> attachmentDesc(
                        ICE_PLACEMENT_NEW(PhysxTendonAttachmentDesc), [](PhysxTendonAttachmentDesc* p){ ICE_FREE(p); });
                    parseAttachment(attachedStage, attachmentApi, attachmentDesc.get(), xfCache, prim);
                    attachmentMap[attachmentDesc->parentPath].push_back(attachmentDesc);
                }
            }
            else if (token.GetString().length() > leafLen &&
                token.GetString().substr(0, leafLen) == g_tendonAttachmentLeaf.GetString())
            {
                PhysxSchemaPhysxTendonAttachmentLeafAPI leafApi =
                    PhysxSchemaPhysxTendonAttachmentLeafAPI::Get(prim, TfToken(token.GetString().substr(leafLen)));
                if (leafApi && isNoDuplicate(PhysxSchemaPhysxTendonAttachmentAPI(leafApi, leafApi.GetName()), attachInstanceNames))
                {
                    std::shared_ptr<PhysxTendonAttachmentLeafDesc> leafDesc(
                        ICE_PLACEMENT_NEW(PhysxTendonAttachmentLeafDesc), [](PhysxTendonAttachmentLeafDesc* p) { ICE_FREE(p); });
                    parseLeafAttachment(attachedStage, leafApi, leafDesc.get(), xfCache, prim);
                    attachmentMap[leafDesc->parentPath].push_back(leafDesc);
                }
            }
            else if (token.GetString().length() > rootLen &&
                token.GetString().substr(0, rootLen) == g_tendonAttachmentRoot.GetString())
            {
                PhysxSchemaPhysxTendonAttachmentRootAPI rootApi =
                    PhysxSchemaPhysxTendonAttachmentRootAPI::Get(prim, TfToken(token.GetString().substr(rootLen)));
                if (rootApi && isNoDuplicate(PhysxSchemaPhysxTendonAttachmentAPI(rootApi, rootApi.GetName()), attachInstanceNames))
                {
                    //prim.GetMetadata(TfToken("kilogramsPerUnit"), &kgPerUnit);
                    std::shared_ptr<PhysxTendonSpatialDesc> rootDesc(
                        ICE_PLACEMENT_NEW(PhysxTendonSpatialDesc), [](PhysxTendonSpatialDesc* p) { ICE_FREE(p); });
                    parseRootAttachment(attachedStage, rootApi, rootDesc.get(), xfCache, prim);
                    spatialTendons.push_back(rootDesc);
                }
            }
        }
    }

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

                // create Attachment
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

        // loop over spatial tendons
        for (SpatialTendonVector::const_reference tendonRootDesc : spatialTendons)
        {
            const SdfPath& rootPath = tendonRootDesc->linkPath;

            // create root attachment
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

    std::vector<PhysxTendonAttachmentHierarchyDesc*> getUiInfoRecursive(const std::shared_ptr<PhysxTendonAttachmentDesc> parentDesc,
        TendonAttachmentMap attachmentMap)
    {
        std::vector<PhysxTendonAttachmentHierarchyDesc*> result;
        // loop over attachments that point to the given parent's link
        for (TendonAttachmentMap::mapped_type::const_reference attachmentRef : attachmentMap[parentDesc->linkPath])
        {
            // ensure that the child does not point to another attachment on parent link/Xform
            if (attachmentRef->parentToken == parentDesc->instanceToken)
            {
                PhysxTendonAttachmentHierarchyDesc* attachmentDesc = ICE_PLACEMENT_NEW(PhysxTendonAttachmentHierarchyDesc);

                attachmentDesc->linkPath = attachmentRef->linkPath;
                attachmentDesc->localPos = attachmentRef->localPos;
                attachmentDesc->type = attachmentRef->type;
                attachmentDesc->children = getUiInfoRecursive(attachmentRef, attachmentMap);
                attachmentRef->type = eTendonAttachmentUI; // mark as parsed

                result.push_back(attachmentDesc);
            }
        }
        return result;
    }

    std::vector<PhysxTendonAttachmentHierarchyDesc*> getAttachmentUiInfo(TendonAttachmentMap& attachmentMap,
        SpatialTendonVector& spatialTendons)
    {
        std::vector<PhysxTendonAttachmentHierarchyDesc*> result;

        // loop over spatial tendons
        for (SpatialTendonVector::const_reference tendonRef : spatialTendons)
        {
            PhysxTendonAttachmentHierarchyDesc* attachmentDesc = ICE_PLACEMENT_NEW(PhysxTendonAttachmentHierarchyDesc);

            attachmentDesc->linkPath = tendonRef->linkPath;
            attachmentDesc->localPos = tendonRef->localPos;
            attachmentDesc->type = tendonRef->type;
            attachmentDesc->children = getUiInfoRecursive(tendonRef, attachmentMap);
            tendonRef->type = eTendonAttachmentUI; // mark as parsed

            result.push_back(attachmentDesc);
        }

        // also send currently unused attachments
        for (TendonAttachmentMap::reference attachmentVec : attachmentMap)
        {
            for (TendonAttachmentMap::mapped_type::reference attachmentRef : attachmentVec.second)
            {
                // type was set if attachment was parsed
                if (attachmentRef->type != eTendonAttachmentUI)
                {
                    PhysxTendonAttachmentHierarchyDesc* attachmentDesc = ICE_PLACEMENT_NEW(PhysxTendonAttachmentHierarchyDesc);

                    attachmentDesc->linkPath = attachmentRef->linkPath;
                    attachmentDesc->localPos = attachmentRef->localPos;
                    attachmentDesc->children = getUiInfoRecursive(attachmentRef, attachmentMap);

                    result.push_back(attachmentDesc);
                }
            }
        }

        return result;
    }

} // namespace usdparser
} // namespace physx
} // namespace omni
