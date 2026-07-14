// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once
#include <omni/graph/core/Handle.h>
#include <omni/graph/core/IConstBundle.h>
#include <omni/graph/core/ogn/Database.h>
#include <omni/graph/core/ogn/ArrayAttribute.h>
#include <omni/graph/core/ogn/Bundle.h>

#include "../MeshTypes.h"

namespace physx
{
struct PxGeomIndexPair;
}

namespace omni
{
namespace physx
{
namespace graph
{
struct ImmediateShared;

struct ImmediateNode
{
    // Access the shared globals
    static ImmediateShared& getGlobals();

    // Fills the MeshInputView class with data coming from omnigraph input prims bundle
    // It can be a single prim or a bundle with multiple prim children. From each prim child reads 'sourecePrimPath',
    // 'sourcePrimType', 'points', 'faceVertexIndices', 'faceVertexCounts', 'worldMatrix', 'sourcePrimPath' (and
    // optionally 'holeIndices', 'orientation', 'meshKey' and 'physics:approximation') attributes"
    static bool fillMeshInputViewFromBundle(
        omni::graph::core::ogn::OmniGraphDatabase& db,
        omni::graph::core::IConstBundle2* inputBundle,
        std::vector<MeshInputView>& inputMeshView,
        std::unordered_map<omni::graph::core::NameToken, size_t>* primToIndicesMap = nullptr);

    static bool fillMeshInputViewFromTargets(ogn::OmniGraphDatabase& db,
                                             omni::graph::core::ogn::ArrayInput<const TargetPath, ogn::kCpu>& targets,
                                             std::vector<MeshInputView>& inputMeshView);


    static bool fillBoxOverlapPairsVector(omni::graph::core::ogn::OmniGraphDatabase& db,
                                          omni::graph::core::ogn::ArrayInput<const NameToken, ogn::kCpu>& overlapsPair0,
                                          omni::graph::core::ogn::ArrayInput<const NameToken, ogn::kCpu>& overlapsPair1,
                                          std::vector<::physx::PxGeomIndexPair>& boxesOverlapPairs,
                                          const std::unordered_map<NameToken, size_t>& namesToIndices);

    static bool validateMeshBundle(omni::graph::core::ogn::OmniGraphDatabase& db,
                                   const omni::graph::core::ogn::BundleContents<ogn::kOgnInput, ogn::kCpu>& childBundle);

    static void checkCookingWarnings(omni::graph::core::ogn::OmniGraphDatabase& db,
                                     const std::vector<MeshInputView>& meshes,
                                     const std::vector<MeshCookedData>& meshCookedData);
};
} // namespace graph
} // namespace physx
} // namespace omni
