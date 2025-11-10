// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>


namespace omni
{

namespace physx
{
enum class PhysXStat
{
    eNbDynamicRigids, //!< The number of dynamic rigid bodies.
    eNbActiveDynamicRigids, //!< The number of active dynamic rigid bodies.
    eNbStaticRigids, //!< The number of static rigid bodies.
    eNbKinematicBodies, //!< The number of kinematic rigid bodies.
    eNbActiveKinematicBodies, //!< The number of active kinematic rigid bodies.

    eNbArticulations, //!< The number of articulations.
    eNbAggregates, //!< The number of aggregates.

    // shapes
    eNbSphereShapes, //!< The number of sphere shapes.
    eNbBoxShapes, //!< The number of box shapes.
    eNbCapsuleShapes, //!< The number of capsule shapes.
    eNbCylinderShapes, //!< The number of cylinder shapes.
    eNbConvexShapes, //!< The number of convex shapes.
    eNbConeShapes, //!< The number of cone shapes.
    eNbTriMeshShapes, //!< The number of triangles mesh shapes.
    eNbPlaneShapes, //!< The number of plane shapes.

    // constraints
    eNbActiveConstraints, //!< The number of active constraints.

    // solver
    eNbAxisSolverConstraints, //!< The number of 1D axis constraints(joints+contact) present in the current
                              //!< simulation step.
    eCompressedContactSize, //!< The size (in bytes) of the compressed contact stream in the current simulation
                            //!< step
    eRequiredContactConstraintMemory, //!< The total required size (in bytes) of the contact constraints in the
                                      //!< current simulation step
    ePeakConstraintMemory, //!< The peak amount of memory (in bytes) that was allocated for constraints (this
                           //!< includes joints) in the current simulation step
    eNbDiscreteContactPairsTotal, //!< Total number of (non CCD) pairs reaching narrow phase
    eNbDiscreteContactPairsWithCacheHits, //!< Total number of (non CCD) pairs for which contacts are
                                          //!< successfully cached (<=nbDiscreteContactPairsTotal) note This
                                          //!< includes pairs for which no contacts are generated, it still
                                          //!< counts as a cache hit.
    eNbDiscreteContactPairsWithContacts, //!< Total number of (non CCD) pairs for which at least 1 contact was
                                         //!< generated (<=nbDiscreteContactPairsTotal)

    // broadphase
    eNbNewPairs, //!< Number of new pairs found by BP this frame
    eNbLostPairs, //!< Number of lost pairs from BP this frame
    eNbNewTouches, //!< Number of new touches found by NP this frame
    eNbLostTouches, //!< Number of lost touches from NP this frame
    eNbPartitions, //!< Number of partitions used by the solver this frame

    // GPU stats
    eGpuMemParticles, //!< GPU device memory in bytes allocated for particle state accessible through API
    eGpuMemDeformableVolumes, //!< GPU device memory in bytes allocated for deformable volume state accessible through
                              //!< API
    eGpuMemDeformableSurfaces, //!< GPU device memory in bytes allocated for deformable surface state accessible through
                               //!< API
    eGpuMemHeap, //!< GPU device memory in bytes allocated for internal heap allocation
    eGpuMemHeapBroadPhase, //!< GPU device heap memory used for broad phase in bytes
    eGpuMemHeapNarrowPhase, //!< GPU device heap memory used for narrow phase in bytes
    eGpuMemHeapSolver, //!< GPU device heap memory used for solver in bytes
    eGpuMemHeapArticulation, //!< GPU device heap memory used for articulations in bytes
    eGpuMemHeapSimulation, //!< GPU device heap memory used for simulation pipeline in bytes
    eGpuMemHeapSimulationArticulation, //!< GPU device heap memory used for articulations in the simulation
                                       //!< pipeline in bytes
    eGpuMemHeapSimulationParticles, //!< GPU device heap memory used for particles in the simulation pipeline in
                                    //!< bytes
    eGpuMemHeapSimulationDeformableVolume, //!< GPU device heap memory used for deformable volumes in the
                                           //!< simulation pipeline in bytes
    eGpuMemHeapSimulationDeformableSurface, //!< GPU device heap memory used for deformable surfaces in the
                                            //!< simulation pipeline in bytes
    eGpuMemHeapParticles, //!< GPU device heap memory used for shared buffers in the particles pipeline in bytes
    eGpuMemHeapDeformableVolumes, //!< GPU device heap memory used for shared buffers in the deformable volume
                                  //!< pipeline in bytes
    eGpuMemHeapDeformableSurfaces, //!< GPU device heap memory used for shared buffers in the deformable surface
                                   //!< pipeline in bytes
    eGpuMemHeapOther, //!< GPU device heap memory not covered by other stats in bytes

    // GPU memory config statistics / actual use
    eGpuMemTempBufferCapacity, //!< actual size needed (bytes) for PxGpuDynamicsMemoryConfig::tempBufferCapacity.
    eGpuMemRigidContactCount, //!< actual number of rigid contacts needed - see
                              //!< PxGpuDynamicsMemoryConfig::maxRigidContactCount.
    eGpuMemRigidPatchCount, //!< actual number of rigid contact patches needed - see
                            //!< PxGpuDynamicsMemoryConfig::maxRigidPatchCount.
    eGpuMemFoundLostPairs, //!< actual number of lost/found pairs needed - see
                           //!< PxGpuDynamicsMemoryConfig::foundLostPairsCapacity.
    eGpuMemFoundLostAggregatePairs, //!< actual number of lost/found aggregate pairs needed - see
                                    //!< PxGpuDynamicsMemoryConfig::foundLostAggregatePairsCapacity.
    eGpuMemTotalAggregatePairs, //!< actual number of aggregate pairs needed - see
                                //!< PxGpuDynamicsMemoryConfig::totalAggregatePairsCapacity.
    eGpuMemDeformableVolumeContacts, //!< actual number of deformable volume contacts needed - see
                                     //!< PxGpuDynamicsMemoryConfig::maxDeformableVolumeContacts.
    eGpuMemDeformableSurfaceContacts, //!< actual number of Deformable Surface contacts needed - see
                                      //!< PxGpuDynamicsMemoryConfig::maxDeformableSurfaceContacts.
    eGpuMemParticleContacts, //!< actual number of particle contacts needed - see
                             //!< PxGpuDynamicsMemoryConfig::maxParticleContacts.
    eGpuMemCollisionStackSize, //!< actual size (bytes) needed for the collision stack - see
                               //!< PxGpuDynamicsMemoryConfig::collisionStackSize.
    eCount
};

struct PhysicsSceneStats
{
    PhysicsSceneStats()
        : nbDynamicRigids(0),
          nbActiveDynamicRigids(0),
          nbStaticRigids(0),
          nbKinematicBodies(0),
          nbActiveKinematicBodies(0),
          nbArticulations(0),
          nbAggregates(0),
          nbSphereShapes(0),
          nbBoxShapes(0),
          nbCapsuleShapes(0),
          nbCylinderShapes(0),
          nbConvexShapes(0),
          nbConeShapes(0),
          nbTriMeshShapes(0),
          nbPlaneShapes(0),
          nbActiveConstraints(0),

          nbAxisSolverConstraints(0),
          compressedContactSize(0),
          requiredContactConstraintMemory(0),
          peakConstraintMemory(0),
          nbDiscreteContactPairsTotal(0),
          nbDiscreteContactPairsWithCacheHits(0),
          nbDiscreteContactPairsWithContacts(0),
          nbNewPairs(0),
          nbLostPairs(0),
          nbNewTouches(0),
          nbLostTouches(0),
          nbPartitions(0),
          gpuMemParticles(0),
          gpuMemDeformableVolumes(0),
          gpuMemDeformableSurfaces(0),
          gpuMemHeap(0),
          gpuMemHeapBroadPhase(0),
          gpuMemHeapNarrowPhase(0),
          gpuMemHeapSolver(0),
          gpuMemHeapArticulation(0),
          gpuMemHeapSimulation(0),
          gpuMemHeapSimulationArticulation(0),
          gpuMemHeapSimulationParticles(0),
          gpuMemHeapSimulationDeformableVolume(0),
          gpuMemHeapSimulationDeformableSurface(0),
          gpuMemHeapParticles(0),
          gpuMemHeapDeformableVolumes(0),
          gpuMemHeapDeformableSurfaces(0),
          gpuMemHeapOther(0),
          gpuMemTempBufferCapacity(0),
          gpuMemRigidContactCount(0),
          gpuMemRigidPatchCount(0),
          gpuMemFoundLostPairs(0),
          gpuMemFoundLostAggregatePairs(0),
          gpuMemTotalAggregatePairs(0),
          gpuMemDeformableVolumeContacts(0),
          gpuMemDeformableSurfaceContacts(0),
          gpuMemParticleContacts(0),
          gpuMemCollisionStackSize(0)

    {
    }

    // rigid bodies
    uint32_t nbDynamicRigids; //!< The number of dynamic rigid bodies.
    uint32_t nbActiveDynamicRigids; //!< The number of active dynamic rigid bodies.
    uint32_t nbStaticRigids; //!< The number of static rigid bodies.
    uint32_t nbKinematicBodies; //!< The number of kinematic rigid bodies.
    uint32_t nbActiveKinematicBodies; //!< The number of active kinematic rigid bodies.

    uint32_t nbArticulations; //!< The number of articulations.
    uint32_t nbAggregates; //!< The number of aggregates.

    // shapes
    uint32_t nbSphereShapes; //!< The number of sphere shapes.
    uint32_t nbBoxShapes; //!< The number of box shapes.
    uint32_t nbCapsuleShapes; //!< The number of capsule shapes.
    uint32_t nbCylinderShapes; //!< The number of cylinder shapes.
    uint32_t nbConvexShapes; //!< The number of convex shapes.
    uint32_t nbConeShapes; //!< The number of cone shapes.
    uint32_t nbTriMeshShapes; //!< The number of triangles mesh shapes.
    uint32_t nbPlaneShapes; //!< The number of plane shapes.

    // constraints
    uint32_t nbActiveConstraints; //!< The number of active constraints.

    // solver
    uint32_t nbAxisSolverConstraints; //!< The number of 1D axis constraints(joints+contact) present in the current
                                      //!< simulation step.
    uint32_t compressedContactSize; //!< The size (in bytes) of the compressed contact stream in the current simulation
                                    //!< step
    uint32_t requiredContactConstraintMemory; //!< The total required size (in bytes) of the contact constraints in the
                                              //!< current simulation step
    uint32_t peakConstraintMemory; //!< The peak amount of memory (in bytes) that was allocated for constraints (this
                                   //!< includes joints) in the current simulation step
    uint32_t nbDiscreteContactPairsTotal; //!< Total number of (non CCD) pairs reaching narrow phase
    uint32_t nbDiscreteContactPairsWithCacheHits; //!< Total number of (non CCD) pairs for which contacts are
                                                  //!< successfully cached (<=nbDiscreteContactPairsTotal) note This
                                                  //!< includes pairs for which no contacts are generated, it still
                                                  //!< counts as a cache hit.
    uint32_t nbDiscreteContactPairsWithContacts; //!< Total number of (non CCD) pairs for which at least 1 contact was
                                                 //!< generated (<=nbDiscreteContactPairsTotal)

    // broadphase
    uint32_t nbNewPairs; //!< Number of new pairs found by BP this frame
    uint32_t nbLostPairs; //!< Number of lost pairs from BP this frame
    uint32_t nbNewTouches; //!< Number of new touches found by NP this frame
    uint32_t nbLostTouches; //!< Number of lost touches from NP this frame
    uint32_t nbPartitions; //!< Number of partitions used by the solver this frame

    // GPU stats
    uint64_t gpuMemParticles; //!< GPU device memory in bytes allocated for particle state accessible through API
    uint64_t gpuMemDeformableVolumes; //!< GPU device memory in bytes allocated for deformable volume state
                                      //!< accessible through API
    uint64_t gpuMemDeformableSurfaces; //!< GPU device memory in bytes allocated for deformable surface state
                                       //!< accessible through API
    uint64_t gpuMemHeap; //!< GPU device memory in bytes allocated for internal heap allocation
    uint64_t gpuMemHeapBroadPhase; //!< GPU device heap memory used for broad phase in bytes
    uint64_t gpuMemHeapNarrowPhase; //!< GPU device heap memory used for narrow phase in bytes
    uint64_t gpuMemHeapSolver; //!< GPU device heap memory used for solver in bytes
    uint64_t gpuMemHeapArticulation; //!< GPU device heap memory used for articulations in bytes
    uint64_t gpuMemHeapSimulation; //!< GPU device heap memory used for simulation pipeline in bytes
    uint64_t gpuMemHeapSimulationArticulation; //!< GPU device heap memory used for articulations in the simulation
                                               //!< pipeline in bytes
    uint64_t gpuMemHeapSimulationParticles; //!< GPU device heap memory used for particles in the simulation pipeline in
                                            //!< bytes
    uint64_t gpuMemHeapSimulationDeformableVolume; //!< GPU device heap memory used for deformable volumes
                                                   //!< in the simulation pipeline in bytes
    uint64_t gpuMemHeapSimulationDeformableSurface; //!< GPU device heap memory used for deformable surfaces
                                                    //!< in the simulation pipeline in bytes
    uint64_t gpuMemHeapParticles; //!< GPU device heap memory used for shared buffers in the particles pipeline in bytes
    uint64_t gpuMemHeapDeformableVolumes; //!< GPU device heap memory used for shared buffers in the deformable volume
                                          //!< pipeline in bytes
    uint64_t gpuMemHeapDeformableSurfaces; //!< GPU device heap memory used for shared buffers in the deformable volume
                                           //!< pipeline in bytes
    uint64_t gpuMemHeapOther; //!< GPU device heap memory not covered by other stats in bytes

    // GPU memory config statistics / actual use
    uint64_t gpuMemTempBufferCapacity; //!< actual size needed (bytes) for
                                       //!< PxGpuDynamicsMemoryConfig::tempBufferCapacity.
    uint32_t gpuMemRigidContactCount; //!< actual number of rigid contacts needed - see
                                      //!< PxGpuDynamicsMemoryConfig::maxRigidContactCount.
    uint32_t gpuMemRigidPatchCount; //!< actual number of rigid contact patches needed - see
                                    //!< PxGpuDynamicsMemoryConfig::maxRigidPatchCount.
    uint32_t gpuMemFoundLostPairs; //!< actual number of lost/found pairs needed - see
                                   //!< PxGpuDynamicsMemoryConfig::foundLostPairsCapacity.
    uint32_t gpuMemFoundLostAggregatePairs; //!< actual number of lost/found aggregate pairs needed - see
                                            //!< PxGpuDynamicsMemoryConfig::foundLostAggregatePairsCapacity.
    uint32_t gpuMemTotalAggregatePairs; //!< actual number of aggregate pairs needed - see
                                        //!< PxGpuDynamicsMemoryConfig::totalAggregatePairsCapacity.
    uint32_t gpuMemDeformableVolumeContacts; //!< actual number of deformable volume contact needed - see
                                             //!< PxGpuDynamicsMemoryConfig::maxDeformableVolumeContacts.
    uint32_t gpuMemDeformableSurfaceContacts; //!< actual number of deformable surface contacts needed - see
                                              //!< PxGpuDynamicsMemoryConfig::maxDeformableSurfaceContacts.
    uint32_t gpuMemParticleContacts; //!< actual number of particle contacts needed - see
                                     //!< PxGpuDynamicsMemoryConfig::maxParticleContacts.
    uint32_t gpuMemCollisionStackSize; //!< actual size (bytes) needed for the collision stack - see
                                       //!< PxGpuDynamicsMemoryConfig::collisionStackSize.
};

/// Interface for PhysX SDK statistics
struct IPhysxStatistics
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxStatistics", 3, 0)

    /// Returns simulation statistics for a particular PhysicsScene.
    ///
    /// \param[in] id USD stageId (can be retrieved from a stagePtr -
    ///     pxr::UsdUtilsStageCache::Get().GetId(stagePtr).ToLongInt())
    /// \param[in] path The Physics scene path.
    /// \param[out] sceneStats Physics scene statistics.
    /// \return True if the scene was found and statistics were filled.
    bool(CARB_ABI* getPhysXSceneStatistics)(uint64_t stageId, uint64_t path, PhysicsSceneStats& sceneStats);

    /// Enables/disables upload of Physx statistics for all PhysicsScenes into carb::stats.
    ///
    /// \param[in] enable bool true=enable, false=disable.
    void(CARB_ABI* enableCarbStatsUpload)(bool enable);
};

} // namespace physx
} // namespace omni
