// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "GraphShared.h"
#include <omni/fabric/FabricTypes.h>

namespace omni
{
namespace physx
{
namespace graph
{

const omni::graph::core::NameToken asNameToken(const uint64_t PxPath)
{
    const omni::fabric::PathC pathC(PxPath);
    const char* name = omni::fabric::toSdfPath(pathC).GetString().c_str();
    const auto tokenInterface = carb::getCachedInterface<omni::fabric::IToken>();
    if (!tokenInterface)
    {
        CARB_LOG_ERROR("Failed to translate PhysX path to NameToken - no token interface");
        return omni::fabric::kUninitializedToken;
    }
    return tokenInterface->getHandle(name);
}

uint64_t toPhysX(const NameToken name)
{
    const auto tokenInterface = carb::getCachedInterface<omni::fabric::IToken>();
    if (!tokenInterface)
    {
        CARB_LOG_ERROR("Failed to translate NameToken to PhysX path - no token interface");
        return 0;
    }
    const char* name_txt = tokenInterface->getText(name);
    return omni::fabric::asInt(pxr::SdfPath(name_txt)).path;
}

bool appendRelationshipPrimPathsToNameTokenArray(omni::graph::core::ogn::OmniGraphDatabase& db,
                                                 std::vector<NameToken>& pathVector,
                                                 NameToken primInput)
{
    const GraphContextObj& context = db.abi_context();
    const NodeObj& nodeObj = db.abi_node();
    const INode& iNode = *nodeObj.iNode;
    const char* thisPrimPathStr = iNode.getPrimPath(nodeObj);
    long stageId = context.iContext->getStageId(context);
    auto stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
    const pxr::UsdPrim thisPrim = stage->GetPrimAtPath(pxr::SdfPath(thisPrimPathStr));
    if (!thisPrim.IsValid())
    {
        db.logError("Could not find node's prim to read relationships from");
        return false;
    }

    const pxr::UsdRelationship relationship = thisPrim.GetRelationship(omni::fabric::toTfToken(primInput));
    pxr::SdfPathVector paths;
    relationship.GetTargets(&paths);
    pathVector.reserve(pathVector.size() + paths.size());
    for (auto inputPath : paths)
    {
        pathVector.push_back(db.stringToToken(inputPath.GetString().c_str()));
    }
    return true;
}

void createPrimName(NameToken prefix, size_t index, gsl::span<NameToken> outputNames)
{
    auto const& prefixToken = omni::fabric::toTfToken(prefix);

    // allocate temporary buffer for name and index
    auto maxDigits = std::numeric_limits<size_t>::digits10 + 1;
    size_t const size = prefixToken.size() + maxDigits + 1;
    char* buffer = CARB_STACK_ALLOC(char, size);
    memset(buffer, 0, size);

    // create primitive name
    auto iToken = carb::getCachedInterface<IToken>();
    for (size_t i = 0; i < outputNames.size(); ++i)
    {
        std::snprintf(buffer, size, "%s%zu", prefixToken.GetText(), index + i);
        outputNames[i] = iToken->getHandle(buffer);
    }
}
} // namespace graph
} // namespace physx
} // namespace omni
