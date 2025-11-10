// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include <omni/physx/IPhysxStatistics.h>

#include <carb/BindingsPythonUtils.h>


void PhysXStatisticsBindings(pybind11::module& m)
{
    using namespace pybind11;
    using namespace carb;
    using namespace omni::physx;

    const char* docString;

    py::class_<PhysicsSceneStats>(m, "PhysicsSceneStats", R"(
            Physics scene statistics.
        )")
        .def(py::init<>())
        .def_readonly("nb_dynamic_rigids", &PhysicsSceneStats::nbDynamicRigids, R"(The number of dynamic rigid bodies.)")
        .def_readonly("nb_active_dynamic_rigids", &PhysicsSceneStats::nbActiveDynamicRigids, R"(The number of active dynamic rigid bodies.)")
        .def_readonly("nb_static_rigids", &PhysicsSceneStats::nbStaticRigids, R"(The number of static rigid bodies.)")
        .def_readonly("nb_kinematic_rigids", &PhysicsSceneStats::nbKinematicBodies, R"(The number of kinematic rigid bodies.)")
        .def_readonly("nb_active_kinematic_rigids", &PhysicsSceneStats::nbActiveKinematicBodies, R"(The number of active kinematic rigid bodies.)")

        .def_readonly("nb_articulations", &PhysicsSceneStats::nbArticulations, R"(The number of articulations.)")
        .def_readonly("nb_aggregates", &PhysicsSceneStats::nbAggregates, R"(The number of aggregates.)")

        .def_readonly("nb_sphere_shapes", &PhysicsSceneStats::nbSphereShapes, R"(The number of sphere shapes.)")
        .def_readonly("nb_box_shapes", &PhysicsSceneStats::nbBoxShapes, R"(The number of box shapes.)")
        .def_readonly("nb_capsule_shapes", &PhysicsSceneStats::nbCapsuleShapes, R"(The number of capsule shapes.)")
        .def_readonly("nb_cylinder_shapes", &PhysicsSceneStats::nbCylinderShapes, R"(The number of cylinder shapes.)")
        .def_readonly("nb_convex_shapes", &PhysicsSceneStats::nbConvexShapes, R"(The number of convex shapes.)")
        .def_readonly("nb_cone_shapes", &PhysicsSceneStats::nbConeShapes, R"(The number of cone shapes.)")
        .def_readonly("nb_trimesh_shapes", &PhysicsSceneStats::nbTriMeshShapes, R"(The number of triangle mesh shapes.)")
        .def_readonly("nb_plane_shapes", &PhysicsSceneStats::nbPlaneShapes, R"(The number of plane shapes.)")

        .def_readonly("nb_active_constraints", &PhysicsSceneStats::nbActiveConstraints, R"(The number of active constraints.)")

        .def_readonly("nb_axis_solver_constaints", &PhysicsSceneStats::nbAxisSolverConstraints, R"(The number of 1D axis constraints(joints+contact) present in the current simulation step.)")
        .def_readonly("compressed_contact_size", &PhysicsSceneStats::compressedContactSize, R"(The size (in bytes) of the compressed contact stream in the current simulation step.)")
        .def_readonly("required_contact_constraint_memory", &PhysicsSceneStats::requiredContactConstraintMemory, R"(The total required size (in bytes) of the contact constraints in the current simulation step.)")
        .def_readonly("peak_constraint_memory", &PhysicsSceneStats::peakConstraintMemory, R"(The peak amount of memory (in bytes) that was allocated for constraints (this includes joints) in the current simulation step.)")
        .def_readonly("nb_discrete_contact_pairs_total", &PhysicsSceneStats::nbDiscreteContactPairsTotal, R"(Total number of (non CCD) pairs reaching narrow phase.)")
        .def_readonly("nb_discrete_contact_pairs_with_cache_hits", &PhysicsSceneStats::nbDiscreteContactPairsWithCacheHits, R"(Total number of (non CCD) pairs for which contacts are successfully cached (<=nbDiscreteContactPairsTotal) note This includes pairs for which no contacts are generated, it still counts as a cache hit.)")
        .def_readonly("nb_discrete_contact_pairs_with_contacts", &PhysicsSceneStats::nbDiscreteContactPairsWithContacts, R"(Total number of (non CCD) pairs for which at least 1 contact was generated (<=nbDiscreteContactPairsTotal).)")

        .def_readonly("nb_new_pairs", &PhysicsSceneStats::nbNewPairs, R"(Number of new pairs found by BP this frame.)")
        .def_readonly("nb_lost_pairs", &PhysicsSceneStats::nbLostPairs, R"(Number of lost pairs from BP this frame.)")
        .def_readonly("nb_new_touches", &PhysicsSceneStats::nbNewTouches, R"(Number of new touches found by NP this frame.)")
        .def_readonly("nb_lost_touches", &PhysicsSceneStats::nbLostTouches, R"(Number of lost touches from NP this frame.)")
        .def_readonly("nb_partitions", &PhysicsSceneStats::nbPartitions, R"(Number of partitions used by the solver this frame.)")

        // stats related to the gpu memory config that the user can set in the scene API
        .def_readonly("gpu_mem_temp_buffer_capacity",  &PhysicsSceneStats::gpuMemTempBufferCapacity, R"(Actual GPU device memory (bytes) used for Temp Buffer based on initial Gpu Temp Buffer Capacity set with physxScene:gpuTempBufferCapacity)")
        .def_readonly("gpu_mem_rigid_contact_count", &PhysicsSceneStats::gpuMemRigidContactCount, R"(Actual number of rigid contacts needed of Gpu Max Rigid Contact Count set with physxScene:gpuMaxRigidContactCount)")
        .def_readonly("gpu_mem_rigid_patch_count", &PhysicsSceneStats::gpuMemRigidPatchCount, R"(Actual number of rigid contact patches needed of Gpu Max Rigid Patch Count set with physxScene:gpuMaxRigidPatchCount)")
        .def_readonly("gpu_mem_found_lost_pairs", &PhysicsSceneStats::gpuMemFoundLostPairs, R"(Actual number of found/lost pairs needed of Gpu Found Lost Pairs Capacity set with physxScene:gpuFoundLostPairsCapacity)")
        .def_readonly("gpu_mem_found_lost_aggregate_pairs", &PhysicsSceneStats::gpuMemFoundLostAggregatePairs, R"(Actual number of found/lost aggregate pairs needed of Gpu Found Lost Aggregate Pairs Capacity set with physxScene:gpuFoundLostAggregatePairsCapacity)")
        .def_readonly("gpu_mem_total_aggregate_pairs", &PhysicsSceneStats::gpuMemTotalAggregatePairs, R"(Actual number of aggregate pairs needed of Gpu Total Aggregate Pairs Capacity set with physxScene:gpuTotalAggregatePairsCapacity)")
        .def_readonly("gpu_mem_particle_contacts", &PhysicsSceneStats::gpuMemParticleContacts, R"(Actual number of particle contacts needed of Gpu Max Particle Contact Count set with physxScene:gpuMaxParticleContacts)")
        .def_readonly("gpu_mem_deformable_volume_contacts", &PhysicsSceneStats::gpuMemDeformableVolumeContacts, R"(Actual number of deformable volume contacts needed of Gpu Max Softbody Contacts set with physxScene:gpuMaxSoftBodyContacts)")
        .def_readonly("gpu_mem_deformable_surface_contacts", &PhysicsSceneStats::gpuMemDeformableSurfaceContacts, R"(Actual number of deformable surface contacts needed of Gpu Max Deformable Surface Contacts set with physxScene:gpuMaxDeformableSurfaceContacts)")
        .def_readonly("gpu_mem_collision_stack_size", &PhysicsSceneStats::gpuMemCollisionStackSize, R"(Actual GPU device memory (bytes) needed for the collision stack of Gpu Collision Stack Size set with physxScene:gpuCollisionStackSize)")

        // other gpu stats for completeness
        .def_readonly("gpu_mem_particles", &PhysicsSceneStats::gpuMemParticles, R"(GPU device memory in bytes allocated for particle state accessible through API.)")
        .def_readonly("gpu_mem_deformable_volumes", &PhysicsSceneStats::gpuMemDeformableVolumes, R"(GPU device memory in bytes allocated for deformable volume state accessible through API.)")
        .def_readonly("gpu_mem_deformable_surfaces", &PhysicsSceneStats::gpuMemDeformableSurfaces, R"(GPU device memory in bytes allocated for deformable surface state accessible through API.)")
        .def_readonly("gpu_mem_heap", &PhysicsSceneStats::gpuMemHeap, R"(GPU device memory in bytes allocated for internal heap allocation based on initial Gpu Heap Capacity set with physxScene:gpuHeapCapacity.)")
        .def_readonly("gpu_mem_heap_broadphase", &PhysicsSceneStats::gpuMemHeapBroadPhase, R"(GPU device heap memory used for broad phase in bytes.)")
        .def_readonly("gpu_mem_heap_narrowphase", &PhysicsSceneStats::gpuMemHeapNarrowPhase, R"(GPU device heap memory used for narrow phase in bytes.)")
        .def_readonly("gpu_mem_heap_solver", &PhysicsSceneStats::gpuMemHeapSolver, R"(GPU device heap memory used for solver in bytes.)")

        .def_readonly("gpu_mem_heap_articulation", &PhysicsSceneStats::gpuMemHeapArticulation, R"(GPU device heap memory used for articulations in bytes.)")
        .def_readonly("gpu_mem_heap_simulation", &PhysicsSceneStats::gpuMemHeapSimulation, R"(GPU device heap memory used for simulation pipeline in bytes.)")
        .def_readonly("gpu_mem_heap_simulation_articulation", &PhysicsSceneStats::gpuMemHeapSimulationArticulation, R"(GPU device heap memory used for articulations in the simulation pipeline in bytes.)")
        .def_readonly("gpu_mem_heap_simulation_particles", &PhysicsSceneStats::gpuMemHeapSimulationParticles, R"(GPU device heap memory used for particles in the simulation pipeline in bytes.)")

        .def_readonly("gpu_mem_heap_simulation_deformable_volume", &PhysicsSceneStats::gpuMemHeapSimulationDeformableVolume, R"(GPU device heap memory used for deformable volumes in the simulation pipeline in bytes.)")
        .def_readonly("gpu_mem_heap_simulation_deformable_surface", &PhysicsSceneStats::gpuMemHeapSimulationDeformableSurface, R"(GPU device heap memory used for deformable surfaces in the simulation pipeline in bytes.)")
        .def_readonly("gpu_mem_heap_particles", &PhysicsSceneStats::gpuMemHeapParticles, R"(GPU device heap memory used for shared buffers in the particles pipeline in bytes.)")

        .def_readonly("gpu_mem_heap_deformable_volumes", &PhysicsSceneStats::gpuMemHeapDeformableVolumes, R"(GPU device heap memory used for shared buffers in the deformable volume pipeline in bytes.)")
        .def_readonly("gpu_mem_heap_deformable_surfaces", &PhysicsSceneStats::gpuMemHeapDeformableSurfaces, R"(GPU device heap memory used for shared buffers in the deformable surface pipeline in bytes.)")
        .def_readonly("gpu_mem_heap_other", &PhysicsSceneStats::gpuMemHeapOther, R"(GPU device heap memory not covered by other stats in bytes.)")
        .def("__repr__",
        [](const PhysicsSceneStats &a) {
            return fmt::format(
                "PhysicsSceneStatistics:\nnb_dynamic_rigids: {},\nnb_active_dynamic_rigids: {},\nnb_static_rigids: {},\n"
                "nb_kinematic_rigids: {},\nnb_active_kinematic_rigids: {},\nnb_articulations: {},\nnb_aggregates: {},\n"
                "nb_sphere_shapes: {},\nnb_box_shapes: {},\nnb_capsule_shapes: {},\nnb_cylinder_shapes: {},\n"
                "nb_convex_shapes: {},\nnb_cone_shapes: {},\nnb_trimesh_shapes: {},\nnb_plane_shapes: {},\n"
                "nb_active_constraints: {},\nnb_axis_solver_constaints: {},\ncompressed_contact_size: {},\nrequired_contact_constraint_memory: {},\n"
                "peak_constraint_memory: {},\nnb_discrete_contact_pairs_total: {},\nnb_discrete_contact_pairs_with_cache_hits: {},\nnb_discrete_contact_pairs_with_contacts: {},\n"
                "nb_new_pairs: {},\nnb_lost_pairs: {},\nnb_new_touches: {},\nnb_lost_touches: {},\nnb_partitions: {},\n"
                "gpu_mem_particles: {},\ngpu_mem_deformable_volumes: {},\ngpu_mem_deformable_surfaces: {},\n"
                "gpu_mem_heap: {},\ngpu_mem_heap_broadphase: {},\ngpu_mem_heap_narrowphase: {},\ngpu_mem_heap_solver: {},\n"
                "gpu_mem_heap_articulation: {},\ngpu_mem_heap_simulation: {},\ngpu_mem_heap_simulation_articulation: {},\ngpu_mem_heap_simulation_particles: {},\n"
                "gpu_mem_heap_simulation_deformable_volume: {},\ngpu_mem_heap_simulation_deformable_surface: {},\ngpu_mem_heap_particles: {},\n"
                "gpu_mem_heap_deformable_volumes: {},\ngpu_mem_heap_deformable_surfaces: {},\ngpu_mem_heap_other: {},\n",
                a.nbDynamicRigids, a.nbActiveDynamicRigids, a.nbStaticRigids, a.nbKinematicBodies, a.nbActiveKinematicBodies,
                a.nbArticulations, a.nbAggregates,
                a.nbSphereShapes, a.nbBoxShapes, a.nbCapsuleShapes, a.nbCylinderShapes,
                a.nbConvexShapes, a.nbConeShapes, a.nbTriMeshShapes, a.nbPlaneShapes,
                a.nbActiveConstraints, a.nbAxisSolverConstraints, a.compressedContactSize, a.requiredContactConstraintMemory,
                a.peakConstraintMemory, a.nbDiscreteContactPairsTotal, a.nbDiscreteContactPairsWithCacheHits, a.nbDiscreteContactPairsWithContacts,
                a.nbNewPairs, a.nbLostPairs, a.nbNewTouches, a.nbLostTouches, a.nbPartitions,
                a.gpuMemParticles, a.gpuMemDeformableVolumes, a.gpuMemDeformableSurfaces,
                a.gpuMemHeap, a.gpuMemHeapBroadPhase, a.gpuMemHeapNarrowPhase, a.gpuMemHeapSolver,
                a.gpuMemHeapArticulation, a.gpuMemHeapSimulation, a.gpuMemHeapSimulationArticulation, a.gpuMemHeapSimulationParticles,
                a.gpuMemHeapSimulationDeformableVolume, a.gpuMemHeapSimulationDeformableSurface, a.gpuMemHeapParticles,
                a.gpuMemHeapDeformableVolumes, a.gpuMemHeapDeformableSurfaces, a.gpuMemHeapOther);
        });

    auto physxStatistics = defineInterfaceClass<IPhysxStatistics>(
        m, "IPhysxStatistics", "acquire_physx_statistics_interface", "release_physx_statistics_interface");

    m.def("release_physx_statistics_interface_scripting", [](IPhysxStatistics* iface) { carb::getFramework()->releaseInterfaceWithClient("carb.scripting-python.plugin", iface); });  // OM-60917

    docString = R"(
        Get physics scene PhysX simulation statistics.

        Args:
            stage_id: current stageId
            path: physics scene path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.
            stats: PhysicsSceneStats object to store the statistics.
    )";
    physxStatistics.def("get_physx_scene_statistics", wrapInterfaceFunction(&IPhysxStatistics::getPhysXSceneStatistics), docString,
        py::arg("stage_id"), py::arg("path"), py::arg("stats"));

    docString = R"(
        Enables/disables upload of Physx statistics for all PhysicsScenes into carb::stats.

        Args:
            enable: bool true=enable, false=disable.  
    )";
    physxStatistics.def("enable_carb_stats_upload", wrapInterfaceFunction(&IPhysxStatistics::enableCarbStatsUpload), docString,
        py::arg("enable"));
}
