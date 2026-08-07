// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * Shared articulation root-election + graph-aggregation (ADR-0001 §11 / ADR-0002).
 *
 * The articulation pass is identical regardless of the parse source: given the
 * scanned rigid bodies + joints (which carry resolved `body0`/`body1`), plus the
 * `ArticulationRootAPI` prims and their parsed fields, it builds the body/joint
 * graph, elects a root per articulation (weight + centre-of-graph), and emits one
 * `PhysxArticulationDesc` per resolved root. It is USD-free: namespace + ancestry
 * walks go through `IPhysicsSource`, so the USD walker and the ovstage walker run
 * the SAME algorithm instead of duplicating it.
 *
 * @implements REQ-PARSE-ART-002
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 */

#pragma once

#include <omni/physics/parse/Allocator.h>
#include <omni/physics/parse/Descriptors.h>
#include <omni/physics/parse/Handles.h>
#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h> // ArticulationFields

#include <vector>

namespace omni::physics::parse
{

// One ArticulationRootAPI prim and its parsed extension fields. Each walker
// collects these (enumerate the API, `parseArticulation` the fields) and hands
// them to `buildArticulations`.
struct ArticulationRootInput
{
    ObjectKey key;
    ArticulationFields fields;
    std::vector<ObjectKey> sourceFilteredCollisions;
};

// Build the articulation descriptors. `bodies` / `joints` are the scanned
// descriptor lists (joints must already carry resolved `body0`/`body1`); the
// result is appended to `outArticulations`. `source` is used only for namespace
// + ancestry traversal (`forEachDescendant` / `getParent`) — no USD.
void buildArticulations(IPhysicsSource& source,
                        IDescriptorAllocator& allocator,
                        const std::vector<ArticulationRootInput>& roots,
                        const std::vector<DescPtr<PhysxRigidBodyDesc>>& bodies,
                        const std::vector<DescPtr<PhysxJointDesc>>& joints,
                        std::vector<DescPtr<PhysxArticulationDesc>>& outArticulations);

} // namespace omni::physics::parse
