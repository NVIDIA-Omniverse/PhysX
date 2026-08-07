// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * Standalone parse entry points — on-demand, single-prim parsing for callers
 * outside the runtime's stage-attach lifecycle (e.g. authoring UI tools).
 *
 * Built on `scanStage` with a single-prim iterator; runs the native USD
 * walker inside `omni.physics.usd`. The public surface is the
 * parse library's own descriptor types and `ObjectKey`; callers use
 * `pathFor` to resolve keys back to `SdfPath` when they need to look up
 * source prims on the stage themselves.
 *
 * @implements REQ-PARSE-JOINT-003
 * @covers AC-1 AC-2 AC-3
 */

#pragma once

#include <omni/physics/parse/Allocator.h>
#include <omni/physics/parse/Descriptors.h>
#include <omni/physics/parse/Handles.h>

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>

#include <memory>

namespace omni::physics::usd
{
// (Parse-lib types are referenced explicitly as `parse::...`; no namespace-
// scope `using namespace` in this public header.)

// On-demand single-prim parser. Holds an internal `UsdSource` so callers
// can resolve `ObjectKey` fields on returned descriptors back to `SdfPath`
// for stage lookups. One instance per stage; safe to reuse for many parse
// calls. Not thread-safe. The `allocator` is propagated to every
// descriptor `parseJoint` returns and must outlive this parser plus any
// returned or released descriptors — a stack-scoped allocator makes
// cleanup unsafe because returned `DescPtr`s hold references to it.
class StandaloneParser
{
public:
    StandaloneParser(PXR_NS::UsdStageWeakPtr stage, parse::IDescriptorAllocator& allocator);
    ~StandaloneParser();

    StandaloneParser(const StandaloneParser&) = delete;
    StandaloneParser& operator=(const StandaloneParser&) = delete;

    // Parse a single joint prim. Returns a heap-allocated PhysxJointDesc
    // typed subclass (Revolute / Prismatic / Spherical / Distance / Fixed /
    // D6 / Custom) wrapped in `parse::DescPtr` for RAII cleanup. Returns
    // a null `DescPtr` when the prim is missing, the stage is invalid, or
    // the schema parser does not classify the prim as a joint.
    parse::DescPtr<parse::PhysxJointDesc> parseJoint(const PXR_NS::SdfPath& jointKey);

    // Resolve a parse-lib `ObjectKey` produced by this parser back to its
    // source `SdfPath`. Returns an empty `SdfPath` when the key is invalid
    // or unknown to this parser's source.
    PXR_NS::SdfPath pathFor(parse::ObjectKey key) const;

private:
    struct Impl;
    std::unique_ptr<Impl> mImpl;
};

} // namespace omni::physics::usd
