// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-CORE-001
 * @covers AC-1
 */

// clang-format off
// clang-format on

#include "tensors/base/SupersetRigidEntries.h"

#include <PxPhysicsAPI.h>

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

void collectSupersetRigidEntries(PxScene* const scene, std::vector<RigidBodyEntry>& entries)
{
    const PxU32 numRds = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
    std::vector<PxActor*> actors(numRds);
    if (numRds)
        scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, actors.data(), numRds);

    auto addBody = [&entries](PxRigidBody* body, RigidBodyType type)
    {
        RigidBodyEntry e;
        e.body = body;
        e.type = type;
        e.subspace = nullptr;
        const PxU32 ns = body->getNbShapes();
        if (ns)
        {
            e.shapes.resize(ns);
            body->getShapes(e.shapes.data(), ns);
        }
        e.numShapes = ns;
        entries.push_back(std::move(e));
    };

    for (PxActor* a : actors)
        if (PxRigidDynamic* rd = a ? a->is<PxRigidDynamic>() : nullptr)
            addBody(rd, RigidBodyType::eRigidDynamic);

    const PxU32 numArtis = scene->getNbArticulations();
    if (numArtis)
    {
        std::vector<PxArticulationReducedCoordinate*> artis(numArtis);
        scene->getArticulations(artis.data(), numArtis);
        for (PxArticulationReducedCoordinate* arti : artis)
        {
            if (!arti)
                continue;
            const PxU32 numLinks = arti->getNbLinks();
            std::vector<PxArticulationLink*> links(numLinks);
            if (numLinks)
                arti->getLinks(links.data(), numLinks);
            for (PxArticulationLink* link : links)
                if (link)
                    addBody(link, RigidBodyType::eArticulationLink);
        }
    }
}

} // namespace tensors
} // namespace physx
} // namespace omni
