// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Types.h>

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxGraph.h>

#include <PxPhysicsAPI.h>

#include <common/foundation/TypeCast.h>
#include <omni/graph/core/OgnHelpers.h>
#include <omni/fabric/FabricUSD.h>

namespace omni
{
namespace physx
{
namespace graph
{

const omni::graph::core::NameToken asNameToken(const uint64_t PxPath);

uint64_t toPhysX(const NameToken name);

// Extracts all paths from a relationship bundle and appends them as Tokens to the given array
bool appendRelationshipPrimPathsToNameTokenArray(omni::graph::core::ogn::OmniGraphDatabase& db,
                                                 std::vector<NameToken>& pathVector,
                                                 NameToken primInput);

void createPrimName(NameToken prefix, size_t index, gsl::span<NameToken> outputNames);

} // namespace graph
} // namespace physx
} // namespace omni
