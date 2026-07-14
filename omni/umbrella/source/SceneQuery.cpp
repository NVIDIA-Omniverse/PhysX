// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#include <omni/physics/simulation/IPhysicsSceneQuery.h>
#include "OmniPhysics.h"


using namespace omni;
using namespace physics;

// Raycast closest
bool raycastClosest(const carb::Float3& origin, const carb::Float3& unitDir, float distance, RaycastHit& hit, bool bothSides)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    bool foundHit = false;
    float closestDistance = distance;
    
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.sceneQueryFns.raycastClosest)
        {
            RaycastHit currentHit;
            if (simulation.second.simulation.sceneQueryFns.raycastClosest(origin, unitDir, closestDistance, currentHit, bothSides))
            {
                if (!foundHit || currentHit.distance < closestDistance)
                {
                    hit = currentHit;
                    closestDistance = currentHit.distance;
                    foundHit = true;
                }
            }
        }
    }
    return foundHit;
}

// Raycast any
bool raycastAny(const carb::Float3& origin, const carb::Float3& unitDir, float distance, bool bothSides)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.sceneQueryFns.raycastAny)
        {
            if (simulation.second.simulation.sceneQueryFns.raycastAny(origin, unitDir, distance, bothSides))
            {
                return true;
            }
        }
    }
    return false;
}

// Raycast all
void raycastAll(
    const carb::Float3& origin, const carb::Float3& unitDir, float distance, RaycastHitReportFn reportFn, bool bothSides)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.sceneQueryFns.raycastAll)
        {
            simulation.second.simulation.sceneQueryFns.raycastAll(origin, unitDir, distance, reportFn, bothSides);
        }
    }    
}

// Sweep sphere closest
bool sweepSphereClosest(
    float radius, const carb::Float3& origin, const carb::Float3& unitDir, float distance, SweepHit& hit, bool bothSides)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    bool foundHit = false;
    float closestDistance = distance;

    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.sceneQueryFns.sweepSphereClosest)
        {
            SweepHit currentHit;
            if (simulation.second.simulation.sceneQueryFns.sweepSphereClosest(
                    radius, origin, unitDir, closestDistance, currentHit, bothSides))
            {
                if (!foundHit || currentHit.distance < closestDistance)
                {
                    hit = currentHit;
                    closestDistance = currentHit.distance;
                    foundHit = true;
                }
            }
        }
    }
    return foundHit;
}

// Sweep sphere any
bool sweepSphereAny(float radius, const carb::Float3& origin, const carb::Float3& unitDir, float distance, bool bothSides)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.sceneQueryFns.sweepSphereAny)
        {
            if (simulation.second.simulation.sceneQueryFns.sweepSphereAny(
                    radius, origin, unitDir, distance, bothSides))
            {
                return true;
            }
        }
    }
    return false;
}

// Sweep sphere all
void sweepSphereAll(float radius,
                    const carb::Float3& origin,
                    const carb::Float3& unitDir,
                    float distance,
                    SweepHitReportFn reportFn,
                    bool bothSides)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.sceneQueryFns.sweepSphereAll)
        {
            simulation.second.simulation.sceneQueryFns.sweepSphereAll(
                radius, origin, unitDir, distance, reportFn, bothSides);
        }
    }    
}

// Overlap test of a sphere against objects in the physics scene
uint32_t overlapSphere(float radius, const carb::Float3& pos, OverlapHitReportFn reportFn)
{
    uint32_t totalOverlaps = 0;
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.sceneQueryFns.overlapSphere)
        {
            totalOverlaps += simulation.second.simulation.sceneQueryFns.overlapSphere(radius, pos, reportFn);
        }
    }
    return totalOverlaps;
}

// Overlap test of a sphere against objects in the physics scene, reports only boolean
bool overlapSphereAny(float radius, const carb::Float3& pos)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.sceneQueryFns.overlapSphereAny)
        {
            if (simulation.second.simulation.sceneQueryFns.overlapSphereAny(radius, pos))
            {
                return true;
            }
        }
    }
    return false;
}

// Overlap test of a box against objects in the physics scene
uint32_t overlapBox(const carb::Float3& halfExtent,
                    const carb::Float3& pos,
                    const carb::Float4& rot,
                    OverlapHitReportFn reportFn)
{
    uint32_t totalOverlaps = 0;
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.sceneQueryFns.overlapBox)
        {
            totalOverlaps += simulation.second.simulation.sceneQueryFns.overlapBox(halfExtent, pos, rot, reportFn);
        }
    }
    return totalOverlaps;
}

// Overlap test of a box against objects in the physics scene, reports only boolean
bool overlapBoxAny(const carb::Float3& halfExtent, const carb::Float3& pos, const carb::Float4& rot)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.sceneQueryFns.overlapBoxAny)
        {
            if (simulation.second.simulation.sceneQueryFns.overlapBoxAny(halfExtent, pos, rot))
            {
                return true;
            }
        }
    }
    return false;
}

// Overlap test of a UsdGeomGPrim against objects in the physics scene
uint32_t overlapShape(uint64_t gPrimPath, OverlapHitReportFn reportFn)
{
    uint32_t totalOverlaps = 0;
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.sceneQueryFns.overlapShape)
        {
            totalOverlaps += simulation.second.simulation.sceneQueryFns.overlapShape(gPrimPath, reportFn);
        }
    }
    return totalOverlaps;
}

// Overlap test of a mesh against objects in the physics scene, reports only boolean
bool overlapShapeAny(uint64_t gPrimPath)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.sceneQueryFns.overlapShapeAny)
        {
            if (simulation.second.simulation.sceneQueryFns.overlapShapeAny(gPrimPath))
            {
                return true;
            }
        }
    }
    return false;
}

// Sweep test of a box against all objects in the physics scene, returning the closest hit found.
bool sweepBoxClosest(const carb::Float3& halfExtent,
                     const carb::Float3& pos,
                     const carb::Float4& rot,
                     const carb::Float3& unitDir,
                     float distance,
                     SweepHit& hit,
                     bool bothSides)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    bool foundHit = false;
    float closestDistance = distance;

    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.sceneQueryFns.sweepBoxClosest)
        {
            SweepHit currentHit;
            if (simulation.second.simulation.sceneQueryFns.sweepBoxClosest(
                    halfExtent, pos, rot, unitDir, closestDistance, currentHit, bothSides))
            {
                if (!foundHit || currentHit.distance < closestDistance)
                {
                    hit = currentHit;
                    closestDistance = currentHit.distance;
                    foundHit = true;
                }
            }
        }
    }
    return foundHit;
}

// Sweep test of a UsdGeomGPrim against all objects in the physics scene, returning the closest hit found.
bool sweepShapeClosest(uint64_t gPrimPath, const carb::Float3& unitDir, float distance, SweepHit& hit, bool bothSides)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    bool foundHit = false;
    float closestDistance = distance;

    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.sceneQueryFns.sweepShapeClosest)
        {
            SweepHit currentHit;
            if (simulation.second.simulation.sceneQueryFns.sweepShapeClosest(
                    gPrimPath, unitDir, closestDistance, currentHit, bothSides))
            {
                if (!foundHit || currentHit.distance < closestDistance)
                {
                    hit = currentHit;
                    closestDistance = currentHit.distance;
                    foundHit = true;
                }
            }
        }
    }
    return foundHit;
}

// Sweep test of a box against all objects in the physics scene, returning whether any hit was found.
bool sweepBoxAny(const carb::Float3& halfExtent,
                 const carb::Float3& pos,
                 const carb::Float4& rot,
                 const carb::Float3& unitDir,
                 float distance,
                 bool bothSides)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.sceneQueryFns.sweepBoxAny)
        {
            if (simulation.second.simulation.sceneQueryFns.sweepBoxAny(
                    halfExtent, pos, rot, unitDir, distance, bothSides))
            {
                return true;
            }
        }
    }
    return false;
}

// Sweep test of a UsdGeomGPrim against all objects in the physics scene, returning whether any hit was found.
bool sweepShapeAny(uint64_t gPrimPath, const carb::Float3& unitDir, float distance, bool bothSides)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.sceneQueryFns.sweepShapeAny)
        {
            if (simulation.second.simulation.sceneQueryFns.sweepShapeAny(gPrimPath, unitDir, distance, bothSides))
            {
                return true;
            }
        }
    }
    return false;
}

// Sweep test of a box against all objects in the physics scene, returning all the hits found.
void sweepBoxAll(const carb::Float3& halfExtent,
                 const carb::Float3& pos,
                 const carb::Float4& rot,
                 const carb::Float3& unitDir,
                 float distance,
                 SweepHitReportFn reportFn,
                 bool bothSides)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.sceneQueryFns.sweepBoxAll)
        {
            simulation.second.simulation.sceneQueryFns.sweepBoxAll(
                halfExtent, pos, rot, unitDir, distance, reportFn, bothSides);
        }
    }    
}

// Sweep test of a UsdGeomGPrim against all objects in the physics scene, returning all the hits found.
void sweepShapeAll(uint64_t gPrimPath, const carb::Float3& unitDir, float distance, SweepHitReportFn reportFn, bool bothSides)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.sceneQueryFns.sweepShapeAll)
        {
            simulation.second.simulation.sceneQueryFns.sweepShapeAll(
                gPrimPath, unitDir, distance, reportFn, bothSides);
        }
    }    
}

void fillInterface(IPhysicsSceneQuery& iface)
{
    iface.raycastClosest = raycastClosest;
    iface.raycastAny = raycastAny;
    iface.raycastAll = raycastAll;
    iface.sweepSphereClosest = sweepSphereClosest;
    iface.sweepSphereAny = sweepSphereAny;
    iface.sweepSphereAll = sweepSphereAll;
    iface.overlapSphere = overlapSphere;
    iface.overlapSphereAny = overlapSphereAny;
    iface.overlapBox = overlapBox;
    iface.overlapBoxAny = overlapBoxAny;
    iface.overlapShape = overlapShape;
    iface.overlapShapeAny = overlapShapeAny;
    iface.sweepBoxClosest = sweepBoxClosest;
    iface.sweepShapeClosest = sweepShapeClosest;
    iface.sweepBoxAny = sweepBoxAny;
    iface.sweepShapeAny = sweepShapeAny;
    iface.sweepBoxAll = sweepBoxAll;
    iface.sweepShapeAll = sweepShapeAll;
}
