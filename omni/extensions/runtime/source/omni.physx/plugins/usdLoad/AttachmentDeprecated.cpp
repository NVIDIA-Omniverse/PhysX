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
#include "AttachmentDeprecated.h"

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
    PhysxAttachmentDesc* parsePhysxAttachmentDeprecated(const pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& usdPrim)
    {
        PhysxAttachmentDesc* attachmentDesc = ICE_PLACEMENT_NEW(PhysxAttachmentDesc)();
        PhysxSchemaPhysxPhysicsAttachment attachment = PhysxSchemaPhysxPhysicsAttachment::Get(stage, usdPrim.GetPath());

        for (int i = 0; i < 2; i++)
        {
            pxr::SdfPathVector targets;
            attachment.GetActorRel(i).GetTargets(&targets);

            if (targets.empty())
                attachmentDesc->actor[i].path = SdfPath();
            else
                attachmentDesc->actor[i].path = targets[0];

            VtArray<GfVec3f> attachmentPoints;
            VtArray<uint32_t> filterIds;

            attachment.GetPointsAttr(i).Get(&attachmentPoints);
            attachment.GetCollisionFilterIndicesAttr(i).Get(&filterIds);

            convert(attachmentDesc->actor[i].points, attachmentPoints);
            convert(attachmentDesc->actor[i].filterIndices, filterIds);

            TfToken attachmentFilterType;
            attachment.GetFilterTypeAttr(i).Get(&attachmentFilterType);
            if (attachmentFilterType == PhysxSchemaTokens.Get()->Vertices)
                attachmentDesc->actor[i].filterType = eVERTICES;
            if (attachmentFilterType == PhysxSchemaTokens.Get()->Geometry)
                attachmentDesc->actor[i].filterType = eGEOMETRY;
        }

        size_t size = std::max(attachmentDesc->actor[0].points.size(), attachmentDesc->actor[1].points.size());

        VtArray<GfVec3f> distanceAxes(size, GfVec3f(0.0f));
        const UsdAttribute distanceAxesAttr = usdPrim.GetAttribute(physxAttachmentDistanceAxesToken);
        if (distanceAxesAttr.HasAuthoredValue())
        {
            distanceAxesAttr.Get(&distanceAxes);
        }
        convert(attachmentDesc->distanceAxes, distanceAxes);

        attachment.GetAttachmentEnabledAttr().Get(&attachmentDesc->attachmentEnabled);

        return attachmentDesc;
    }

} // namespace usdparser
} // namespace physx
} // namespace omni
