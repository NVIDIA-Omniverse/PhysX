// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-JOINT-003
 * @covers AC-1 AC-2 AC-3
 *
 * On-demand single-prim joint parser. Drives the native USD walker
 * via `scanStage` over a single-prim range. Per-instance `UsdSource`
 * is retained so callers can resolve `ObjectKey` fields on returned
 * descriptors back to their `SdfPath`s across multiple parseJoint
 * calls — keys minted by the per-call `scanStage` are translated into
 * this parser's persistent source vocabulary before the descriptor is
 * returned.
 */

#include <omni/physics/usd/StandaloneParser.h>

#include <pxr/base/gf/transform.h>
#include <pxr/usd/usd/collectionMembershipQuery.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xformCache.h>

#include <omni/physics/usd/StageScan.h>

#include "UsdSource.h"

#include <omni/physics/usd/PrimIterator.h>

namespace omni::physics::usd
{
using namespace omni::physics::parse;

// ---------------------------------------------------------------------------
// StandaloneParser
// ---------------------------------------------------------------------------

struct StandaloneParser::Impl
{
    PXR_NS::UsdStageWeakPtr stage;
    UsdSource source;
    parse::IDescriptorAllocator& allocator;

    Impl(PXR_NS::UsdStageWeakPtr s, parse::IDescriptorAllocator& a)
        : stage(s), source(s), allocator(a) {}
};

StandaloneParser::StandaloneParser(PXR_NS::UsdStageWeakPtr stage,
                                   parse::IDescriptorAllocator& allocator)
    : mImpl(std::make_unique<Impl>(stage, allocator))
{
}

StandaloneParser::~StandaloneParser() = default;

parse::DescPtr<parse::PhysxJointDesc> StandaloneParser::parseJoint(const PXR_NS::SdfPath& jointKey)
{
    if (!mImpl->stage)
        return {};

    PXR_NS::UsdPrim prim = mImpl->stage->GetPrimAtPath(jointKey);
    if (!prim)
        return {};

    // Scope `scanStage` to the single joint prim's subtree. Joints
    // don't typically have physics children, so the walker emits
    // exactly one joint (or none, when the prim isn't a joint).
    PXR_NS::UsdPrimRange range(prim);
    omni::physics::schema::PrimIteratorRange iter(range);
    ScannedStage scanned = scanStage(mImpl->stage, iter, mImpl->allocator);
    if (scanned.joints.empty())
        return {};

    // Re-key the descriptor's ObjectKey fields from the scan's
    // internal source vocabulary into this parser's persistent source.
    // Callers resolve `desc->body0` / `body1` / `jointPrimKey` /
    // `rel0` / `rel1` via `StandaloneParser::pathFor`, so each field
    // must land in `mImpl->source`'s intern table.
    parse::DescPtr<parse::PhysxJointDesc> desc = std::move(scanned.joints[0]);
    auto reKey = [&](parse::ObjectKey k) -> parse::ObjectKey {
        if (!k.valid()) return {};
        return mImpl->source.keyFor(scanned.pathFor(k));
    };
    desc->jointPrimKey = reKey(desc->jointPrimKey);
    desc->body0 = reKey(desc->body0);
    desc->body1 = reKey(desc->body1);
    desc->rel0  = reKey(desc->rel0);
    desc->rel1  = reKey(desc->rel1);

    return desc;
}

PXR_NS::SdfPath StandaloneParser::pathFor(parse::ObjectKey key) const
{
    return mImpl->source.pathFor(key);
}

} // namespace omni::physics::usd
