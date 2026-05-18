// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

namespace omni
{
namespace physx
{

// Forward declaration
struct ICookingComputeService;

/**
 * Public declaration of the functions to create a ujitso cooking compute service.  This is an implementation
 * of the ICookingComputeService interface.  It does not fully implement that interface, relying on an internal
 * CookingComputeService implementation as a fallback service.
 *
 * The ujitso features may be enabled/disabled using the bool setting:
 *      "/physics/cooking/ujitsoCollisionCooking"
 * Setting that value to true enables ujitso.  Setting the value to false causes the fallback service to be used
 * exclusively.
 *
 * When ujitso is enabled, the setting "/physics/cooking/ujitsoRemoteCache" can be used to enable
 * remote caching of builds.  The current remote cache path is:
 *      omniverse://content.ov.nvidia.com/Projects/Physics/Cache/CookedMeshCache
 * If remote caching is enabled and ujitso cannot find a build in its local cache, it will look at that remote path
 * for the build.  If found there, the build will be copied into the local cache.
 * Also, with remote caching enabled, ujitso builds will be stored not only to the local cache but also to the remote
 * cache.
 *
 * \return a new ujitso cooking compute service if successful, nullptr otherwise.
 */
ICookingComputeService* createUjitsoCookingComputingService();

} // namespace physx
} // namespace omni
