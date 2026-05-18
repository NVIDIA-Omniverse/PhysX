// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/ujitso/UjitsoUtils.inl>

/**
 * Public declarations of functions to create ujitso processors used in the ujitso physx mesh cooking service.
 */

namespace omni
{
namespace physx
{

// Forward declaration
struct ICookingComputeService;

/***********************************************************************************************************************
 * PhysxMeshCookingProcessor
 *
 * This processor can cook convex mesh, convex decomposition, triangle mesh (including sdf), and sphere fill
 * approximations.
 **********************************************************************************************************************/

/**
 * Create a PhysxMeshCookingProcessor
 *
 * \param[in]   cookingComputeService   Interface to a ICookingComputeService implementation.  This is used
 *                                      in case GPU cooking is required; in this case the service is expected
 *                                      to produce a CUDA context.
 *
 * \return the mesh cooking processor if successful, nullptr otherwise.
 */
carb::ujitso::Processor* createPhysxMeshCookingProcessor(ICookingComputeService& cookingComputeService);

/**
 * Retrieve the default process filter for the PhysxMeshCookingProcessor
 *
 * \return the default process filter for the PhysxMeshCookingProcessor.
 */
const carb::ujitso::RequestFilter getPhysxMeshCookingProcessorFilter();

/**
 * Release the PhysxMeshCookingProcessor created with createPhysxMeshCookingProcessor
 *
 * \param[in]   processor   The PhysxMeshCookingProcessor created with createPhysxMeshCookingProcessor.
 */
void releasePhysxMeshCookingProcessor(carb::ujitso::Processor* processor);


/***********************************************************************************************************************
 * PhysxMeshTriangulationProcessor
 *
 * This processor simply triangulates an input mesh.  It is a dependency of PhysxMeshCookingProcessor, but
 * may be used externally when the triangulation is required.
 **********************************************************************************************************************/

/**
 * Create a PhysxMeshTriangulationProcessor
 *
 * \return the mesh triangulation processor if successful, nullptr otherwise.
 */
carb::ujitso::Processor* createPhysxMeshTriangulationProcessor();

/**
 * Retrieve the default process filter for the PhysxMeshTriangulationProcessor
 *
 * \return the default process filter for the PhysxMeshTriangulationProcessor.
 */
const carb::ujitso::RequestFilter getPhysxMeshTriangulationProcessorFilter();

/**
 * Release the PhysxMeshTriangulationProcessor created with createPhysxMeshTriangulationProcessor
 *
 * \param[in]   processor   The PhysxMeshTriangulationProcessor created with createPhysxMeshTriangulationProcessor.
 */
void releasePhysxMeshTriangulationProcessor(carb::ujitso::Processor* processor);

} // namespace physx
} // namespace omni
