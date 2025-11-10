// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <private/omni/physics/schema/IUsdPhysicsListener.h>

#include <common/utilities/Utilities.h>
#include <common/utilities/UsdMaterialParsing.h>

#include "Attachment.h"
#include "UsdLoad.h"

using namespace pxr;

namespace omni
{
    namespace physics
    {
        namespace schema
        {

            bool getSourcePath(const UsdRelationship& srcRel, SdfPath& srcPath)
            {
                if (srcRel)
                {
                    SdfPathVector sources;
                    srcRel.GetTargets(&sources);
                    if (sources.size() == 1)
                    {
                        srcPath = sources[0];
                        return true;
                    }
                    else
                    {
                        CARB_LOG_ERROR("Invalid number of target sources (%zd). Only 1 target source is allowed for attachments.", sources.size());
                    }
                }

                return false;
            }

            AttachmentDesc* parseAttachment(const UsdPrim prim, const ObjectType::Enum objectType, const TfToken derivedTypeName)
            {
                const TfType derivedType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(derivedTypeName);
                if (!prim.IsA(derivedType))
                {
                    return nullptr;
                }

                AttachmentDesc* desc = new AttachmentDesc(objectType);
                if (!desc)
                {
                    return nullptr;
                }

                prim.GetAttribute(OmniPhysicsDeformableAttrTokens->attachmentEnabled).Get(&desc->enabled);

                UsdRelationship src0Rel = prim.GetRelationship(OmniPhysicsDeformableAttrTokens->src0);
                UsdRelationship src1Rel = prim.GetRelationship(OmniPhysicsDeformableAttrTokens->src1);
                bool result = true;
                result &= getSourcePath(src0Rel, desc->src0);
                result &= getSourcePath(src1Rel, desc->src1);
                if (!result)
                {
                    delete desc;
                    return nullptr;
                }

                prim.GetAttribute(OmniPhysicsDeformableAttrTokens->damping).Get(&desc->damping);
                prim.GetAttribute(OmniPhysicsDeformableAttrTokens->stiffness).Get(&desc->stiffness);

                return desc;
            }

            AttachmentDesc* parseAttachment(const UsdStageWeakPtr stage, const UsdPrim& prim, uint64_t typeFlags)
            {
                TfType aType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->Attachment);
                if (!prim.IsA(aType))
                {
                    return nullptr;
                }

                if (typeFlags & PrimType::eUsdPhysicsVtxVtxAttachment)
                {
                    return parseAttachment(prim, ObjectType::eAttachmentVtxVtx, OmniPhysicsDeformableTypeTokens->VtxVtxAttachment);
                }
                else if (typeFlags & PrimType::eUsdPhysicsVtxTriAttachment)
                {
                    return parseAttachment(prim, ObjectType::eAttachmentVtxTri, OmniPhysicsDeformableTypeTokens->VtxTriAttachment);
                }
                else if (typeFlags & PrimType::eUsdPhysicsVtxTetAttachment)
                {
                    return parseAttachment(prim, ObjectType::eAttachmentVtxTet, OmniPhysicsDeformableTypeTokens->VtxTetAttachment);
                }
                else if (typeFlags & PrimType::eUsdPhysicsVtxCrvAttachment)
                {
                    return parseAttachment(prim, ObjectType::eAttachmentVtxCrv, OmniPhysicsDeformableTypeTokens->VtxCrvAttachment);
                }
                else if (typeFlags & PrimType::eUsdPhysicsVtxXformAttachment)
                {
                    return parseAttachment(prim, ObjectType::eAttachmentVtxXform, OmniPhysicsDeformableTypeTokens->VtxXformAttachment);
                }
                else if (typeFlags & PrimType::eUsdPhysicsTetXformAttachment)
                {
                    return parseAttachment(prim, ObjectType::eAttachmentTetXform, OmniPhysicsDeformableTypeTokens->TetXformAttachment);
                }
                else if (typeFlags & PrimType::eUsdPhysicsTriTriAttachment)
                {
                    return parseAttachment(prim, ObjectType::eAttachmentTriTri, OmniPhysicsDeformableTypeTokens->TriTriAttachment);
                }
                return nullptr;
            }

            ElementCollisionFilterDesc* parseCollisionFilter(const UsdStageWeakPtr stage, const UsdPrim& prim)
            {
                TfType fType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->ElementCollisionFilter);
                if (!prim.IsA(fType))
                {
                    return nullptr;
                }

                ElementCollisionFilterDesc* desc = new ElementCollisionFilterDesc();

                prim.GetAttribute(OmniPhysicsDeformableAttrTokens->filterEnabled).Get(&desc->enabled);

                UsdRelationship src0Rel = prim.GetRelationship(OmniPhysicsDeformableAttrTokens->src0);
                UsdRelationship src1Rel = prim.GetRelationship(OmniPhysicsDeformableAttrTokens->src1);
                bool result = true;
                result &= getSourcePath(src0Rel, desc->src0);
                result &= getSourcePath(src1Rel, desc->src1);
                if (!result)
                {
                    delete desc;
                    return nullptr;
                }

                return desc;
            }

        } // namespace schema
    } // namespace physics
} // namespace omni
