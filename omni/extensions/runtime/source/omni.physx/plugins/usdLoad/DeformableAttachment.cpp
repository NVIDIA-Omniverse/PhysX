// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <common/foundation/Allocator.h>

#include "LoadTools.h"
#include "DeformableAttachment.h"

#include <attachment/PhysXAttachment.h>
#include <attachment/PhysXPointFinder.h>

using namespace pxr;
using namespace carb;

static const TfToken physxAttachmentDistanceAxesToken{ "physxAttachment:distanceAxes" };

namespace
{
    void convert(std::vector<carb::Float3>& out, VtArray<pxr::GfVec3f> const& in)
    {
        out.resize(in.size());
        for (size_t i = 0; i < out.size(); i++)
        {
            out[i].x = in[i][0];
            out[i].y = in[i][1];
            out[i].z = in[i][2];
        }
    }

    template<typename T>
    void convert(std::vector<T>& out, VtArray<T> const& in)
    {
        out.resize(in.size());
        for (size_t i = 0; i < out.size(); i++)
        {
            out[i] = in[i];
        }
    }
}

namespace omni
{
namespace physx
{
namespace usdparser
{
    PhysxDeformableAttachmentDesc* parseDeformableAttachment(const pxr::UsdStageWeakPtr stage, const omni::physics::schema::AttachmentDesc& inDesc)
    {
        ObjectType outType = ObjectType::eUndefined;
        switch (inDesc.type)
        {
        case omni::physics::schema::ObjectType::eAttachmentVtxVtx:
            outType = ObjectType::eAttachmentVtxVtx;
            break;
        case omni::physics::schema::ObjectType::eAttachmentVtxTri:
            outType = ObjectType::eAttachmentVtxTri;
            break;
        case omni::physics::schema::ObjectType::eAttachmentVtxTet:
            outType = ObjectType::eAttachmentVtxTet;
            break;
        case omni::physics::schema::ObjectType::eAttachmentVtxXform:
            outType = ObjectType::eAttachmentVtxXform;
            break;
        case omni::physics::schema::ObjectType::eAttachmentTetXform:
            outType = ObjectType::eAttachmentTetXform;
            break;
        default:
            return nullptr;
        }

        PhysxDeformableAttachmentDesc* outDesc = ICE_PLACEMENT_NEW(PhysxDeformableAttachmentDesc)();
        if (outDesc)
        {
            outDesc->type = outType;
            outDesc->enabled = inDesc.enabled;
            outDesc->src0 = inDesc.src0;
            outDesc->src1 = inDesc.src1;
            outDesc->stiffness = inDesc.stiffness;
            outDesc->damping = inDesc.damping;
        }
        return outDesc;
    }

    PhysxDeformableCollisionFilterDesc* parseDeformableCollisionFilter(const pxr::UsdStageWeakPtr stage, const omni::physics::schema::ElementCollisionFilterDesc& inDesc)
    {
        PhysxDeformableCollisionFilterDesc* outDesc = ICE_PLACEMENT_NEW(PhysxDeformableCollisionFilterDesc)();

        outDesc->enabled = inDesc.enabled;
        outDesc->src0 = inDesc.src0;
        outDesc->src1 = inDesc.src1;

        return outDesc;
    }

} // namespace usdparser
} // namespace physx
} // namespace omni
