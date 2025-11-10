// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <omni/physx/IPhysxStatistics.h>

#include "OmniPhysX.h"
#include "internal/InternalPhysXDatabase.h"

#include "usdLoad/LoadUsd.h"
#include "ObjectDataQuery.h"

using namespace ::physx;

namespace omni
{

namespace physx
{
void enableCarbStatsUpload(bool enable)
{
    OmniPhysX::getInstance().enableUploadPhysXStatsToCarb(enable);
}

bool getPhysXSceneStatistics(uint64_t stageId, uint64_t path, PhysicsSceneStats& sceneStats)
{
    const usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (!attachedStage)
    {
        CARB_LOG_ERROR("getPhysXSceneStatistics: stageId %llu not attached.", stageId);
        return false;
    }
    
    PxScene* scene = (PxScene*) getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(intToPath(path), ePTScene, OmniPhysX::getInstance().getInternalPhysXDatabase(), *attachedStage);
    if (!scene)
    {
        // This is valid we can have a simulation without PhysX scene, when PhysX is not the default simulator
        return false;
    }

    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    PxSimulationStatistics simStats;
    scene->getSimulationStatistics(simStats);

    // bodies
    sceneStats.nbDynamicRigids = simStats.nbDynamicBodies;
    sceneStats.nbActiveDynamicRigids = simStats.nbActiveDynamicBodies;
    sceneStats.nbStaticRigids = simStats.nbStaticBodies;
    sceneStats.nbKinematicBodies = simStats.nbKinematicBodies;
    sceneStats.nbActiveKinematicBodies = simStats.nbActiveKinematicBodies;

    // articulations
    sceneStats.nbArticulations = simStats.nbArticulations;
    sceneStats.nbAggregates = simStats.nbAggregates;

    // shapes
    sceneStats.nbBoxShapes = simStats.nbShapes[PxGeometryType::eBOX];
    sceneStats.nbSphereShapes = simStats.nbShapes[PxGeometryType::eSPHERE];
    sceneStats.nbCapsuleShapes = simStats.nbShapes[PxGeometryType::eCAPSULE];
    sceneStats.nbConvexShapes = simStats.nbShapes[PxGeometryType::eCONVEXMESH];
    sceneStats.nbTriMeshShapes = simStats.nbShapes[PxGeometryType::eTRIANGLEMESH];
    sceneStats.nbPlaneShapes = simStats.nbShapes[PxGeometryType::ePLANE];

    sceneStats.nbActiveConstraints = simStats.nbActiveConstraints;

    // We don't currently return this value but use it to limit how many actors and shapes that must be iterated.
    uint32_t convexCoreShapesToCheck = simStats.nbShapes[PxGeometryType::eCONVEXCORE];

    const uint32_t nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC);

    for (uint32_t i = 0; i < nbActors && convexCoreShapesToCheck > 0; i++)
    {
        PxActor* actor = nullptr;
        scene->getActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC, &actor, 1, i);
        {
            PxRigidActor* rbo = actor->is<PxRigidActor>();
            if (rbo)
            {
                PxShape* shape = nullptr;
                const PxU32 numShapes = rbo->getNbShapes();
                for (PxU32 i = 0; i < numShapes; i++)
                {
                    rbo->getShapes(&shape, 1, i);

                    const PxGeometry& geom = shape->getGeometry();
                    if (geom.getType() == PxGeometryType::eCONVEXCORE)
                    {
                        convexCoreShapesToCheck--;
                        const PxConvexCoreGeometry& convex_core_geometry = static_cast<const PxConvexCoreGeometry&>(geom);
                        if (convex_core_geometry.getCoreType() == PxConvexCore::eCYLINDER)
                        {
                            sceneStats.nbCylinderShapes++;
                        }
                        else if (convex_core_geometry.getCoreType() == PxConvexCore::eCONE)
                        {
                            sceneStats.nbConeShapes++;
                        }
                    }
                }
            }
        }
    }

    const usdparser::Axis axis[] = { usdparser::eX , usdparser::eY ,usdparser::eZ };
    for (int i = 0; i < sizeof(axis)/sizeof(axis[0]); i++)
    {
        {
            PxConvexMesh* mesh = physxSetup.getCylinderConvexMesh(axis[i]);
            if (mesh && mesh->getReferenceCount() > 1)
            {
                sceneStats.nbCylinderShapes = mesh->getReferenceCount() - 1;
                sceneStats.nbConvexShapes -= mesh->getReferenceCount() - 1;
            }
        }
        {
            PxConvexMesh* mesh = physxSetup.getConeConvexMesh(axis[i]);
            if (mesh && mesh->getReferenceCount() > 1)
            {
                sceneStats.nbConeShapes = mesh->getReferenceCount() - 1;
                sceneStats.nbConvexShapes -= mesh->getReferenceCount() - 1;
            }
        }
    }

	sceneStats.nbAxisSolverConstraints = simStats.nbAxisSolverConstraints;
	sceneStats.compressedContactSize = simStats.compressedContactSize;
	sceneStats.requiredContactConstraintMemory = simStats.requiredContactConstraintMemory;
	sceneStats.peakConstraintMemory = simStats.peakConstraintMemory;
	sceneStats.nbDiscreteContactPairsTotal = simStats.nbDiscreteContactPairsTotal;
	sceneStats.nbDiscreteContactPairsWithCacheHits = simStats.nbDiscreteContactPairsWithCacheHits;
	sceneStats.nbDiscreteContactPairsWithContacts = simStats.nbDiscreteContactPairsWithContacts;
	sceneStats.nbNewPairs = simStats.nbNewPairs;
	sceneStats.nbLostPairs = simStats.nbLostPairs;
	sceneStats.nbNewTouches = simStats.nbNewTouches;
	sceneStats.nbLostTouches = simStats.nbLostTouches;
	sceneStats.nbPartitions = simStats.nbPartitions;

    // data that the user can setup in the PhysxSceneAPI
    sceneStats.gpuMemTempBufferCapacity = simStats.gpuDynamicsMemoryConfigStatistics.tempBufferCapacity;
    sceneStats.gpuMemRigidContactCount = simStats.gpuDynamicsMemoryConfigStatistics.rigidContactCount;
    sceneStats.gpuMemRigidPatchCount = simStats.gpuDynamicsMemoryConfigStatistics.rigidPatchCount;
    sceneStats.gpuMemFoundLostPairs = simStats.gpuDynamicsMemoryConfigStatistics.foundLostPairs;
    sceneStats.gpuMemFoundLostAggregatePairs = simStats.gpuDynamicsMemoryConfigStatistics.foundLostAggregatePairs;
    sceneStats.gpuMemTotalAggregatePairs = simStats.gpuDynamicsMemoryConfigStatistics.totalAggregatePairs;
    sceneStats.gpuMemParticleContacts = simStats.gpuDynamicsMemoryConfigStatistics.particleContacts;
    sceneStats.gpuMemDeformableVolumeContacts = simStats.gpuDynamicsMemoryConfigStatistics.deformableVolumeContacts;
    sceneStats.gpuMemDeformableSurfaceContacts = simStats.gpuDynamicsMemoryConfigStatistics.deformableSurfaceContacts;
    sceneStats.gpuMemCollisionStackSize = simStats.gpuDynamicsMemoryConfigStatistics.collisionStackSize;

    // other gpu memory statistics
	sceneStats.gpuMemParticles = simStats.gpuMemParticles;
    sceneStats.gpuMemDeformableVolumes = simStats.gpuMemDeformableVolumes;
    sceneStats.gpuMemDeformableSurfaces = simStats.gpuMemDeformableSurfaces;
    sceneStats.gpuMemHeap = simStats.gpuMemHeap;
	sceneStats.gpuMemHeapBroadPhase = simStats.gpuMemHeapBroadPhase;
	sceneStats.gpuMemHeapNarrowPhase = simStats.gpuMemHeapNarrowPhase;
	sceneStats.gpuMemHeapSolver = simStats.gpuMemHeapSolver;
	sceneStats.gpuMemHeapArticulation = simStats.gpuMemHeapArticulation;
	sceneStats.gpuMemHeapSimulation = simStats.gpuMemHeapSimulation;
	sceneStats.gpuMemHeapSimulationArticulation = simStats.gpuMemHeapSimulationArticulation;
	sceneStats.gpuMemHeapSimulationParticles = simStats.gpuMemHeapSimulationParticles;
    sceneStats.gpuMemHeapSimulationDeformableVolume = simStats.gpuMemHeapSimulationDeformableVolume;
    sceneStats.gpuMemHeapSimulationDeformableSurface = simStats.gpuMemHeapSimulationDeformableSurface;
    sceneStats.gpuMemHeapParticles = simStats.gpuMemHeapParticles;
    sceneStats.gpuMemHeapDeformableVolumes = simStats.gpuMemHeapDeformableVolumes;
    sceneStats.gpuMemHeapDeformableSurfaces = simStats.gpuMemHeapDeformableSurfaces;
    sceneStats.gpuMemHeapOther = simStats.gpuMemHeapOther;

    return true;
}
}
}

void fillInterface(omni::physx::IPhysxStatistics& iface)
{
    iface.enableCarbStatsUpload = omni::physx::enableCarbStatsUpload;
    iface.getPhysXSceneStatistics = omni::physx::getPhysXSceneStatistics;
}
