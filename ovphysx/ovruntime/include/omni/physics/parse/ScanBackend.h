// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <omni/physics/parse/Allocator.h>
#include <omni/physics/parse/IPhysicsSource.h> // parse::DescendantScope
#include <omni/physics/parse/IParseBackend.h> // parse::AttachTarget

#include <memory>
#include <string>
#include <vector>

namespace omni::physics::parse
{

class ScannedStage;

struct ScanOptions
{
    // Normal whole-stage scans skip descendants of point-instancer prims so
    // prototype objects are not loaded as regular scene objects. Explicit
    // prototype-root scans disable this to intentionally parse that subtree.
    bool prunePointInstancerDescendants = true;

    // Which objects a scoped scan should visit. The default matches the
    // path-driven stage-load and prototype scans that descend through instance
    // proxies.
    DescendantScope descendantScope = DescendantScope::eActiveInstanced;
};

// One registered whole-stage walker.
class IScanBackend
{
public:
    virtual ~IScanBackend() = default;

    // Produce a ScannedStage for `target` (the same backend-opaque handle the
    // parse backend received). `scanRoots` and `excludePaths` are source-path
    // strings supplied by the USD dispatch layer without exposing USD types to
    // parse core. `allocator` is propagated to every descriptor. Returns an
    // empty scan when the target is not one this backend understands.
    virtual ScannedStage scan(const parse::AttachTarget& target,
                              const std::vector<std::string>& scanRoots,
                              const std::vector<std::string>& excludePaths,
                              const parse::ScanOptions& options,
                              parse::IDescriptorAllocator& allocator) = 0;
};

// Install `backend` as the single active scan backend, replacing any previous
// one. null restores the default (native USD walk).
void setScanBackend(std::unique_ptr<IScanBackend> backend);

// The active scan backend, or null when none is installed (use the native walk).
IScanBackend* scanBackend();

} // namespace omni::physics::parse
