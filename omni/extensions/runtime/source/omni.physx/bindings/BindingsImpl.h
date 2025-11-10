// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <array>

bool descendantHasAPI(const pxr::TfType& name, const pxr::UsdPrim& prim);
bool ancestorHasAPI(const pxr::TfType& name, const pxr::UsdPrim& prim);
bool hasconflictingapis_RigidBodyAPI(const pxr::UsdPrim& prim, bool check_itself = false);
bool hasconflictingapis_CollisionAPI(const pxr::UsdPrim& prim, bool check_itself = false);
bool hasconflictingapis_ArticulationRoot(const pxr::UsdPrim& prim, bool check_itself = false);
bool hasconflictingapis_RigidBodyAPI_WRet(const pxr::UsdPrim& prim, pxr::UsdPrim& ret, bool check_itself = false, bool check_prim = true);
bool hasconflictingapis_CollisionAPI_WRet(const pxr::UsdPrim& prim, pxr::UsdPrim& ret, bool check_itself = false, bool check_prim = true);
bool hasconflictingapis_ArticulationRoot_WRet(const pxr::UsdPrim& prim, pxr::UsdPrim& ret, bool check_itself = false, bool check_prim = true);
bool hasconflictingapis_PhysxDeformableBodyAPI_deprecated(const pxr::UsdPrim& prim, bool check_itself = false);
bool hasconflictingapis_PhysxDeformableSurfaceAPI_deprecated(const pxr::UsdPrim& prim, bool check_itself = false);
bool hasconflictingapis_DeformableBodyAPI(const pxr::UsdPrim& prim, bool check_itself = false);
bool hasconflictingapis_PhysxParticleSamplingAPI(const pxr::UsdPrim& prim, bool check_itself = false);
bool hasconflictingapis_PhysxParticleClothAPI_deprecated(const pxr::UsdPrim& prim, bool check_itself = false);
bool isOverConflictingApisSubtreeLimit(const pxr::UsdPrim& prim, unsigned int limit);
std::array<bool, 3> hasconflictingapis_Precompute(const pxr::UsdPrim& prim);
