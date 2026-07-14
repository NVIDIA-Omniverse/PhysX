// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <array>

bool descendantHasAPI(const PXR_NS::TfType& name, const PXR_NS::UsdPrim& prim);
bool ancestorHasAPI(const PXR_NS::TfType& name, const PXR_NS::UsdPrim& prim);
bool hasconflictingapis_RigidBodyAPI(const PXR_NS::UsdPrim& prim, bool check_itself = false);
bool hasconflictingapis_CollisionAPI(const PXR_NS::UsdPrim& prim, bool check_itself = false);
bool hasconflictingapis_ArticulationRoot(const PXR_NS::UsdPrim& prim, bool check_itself = false);
bool hasconflictingapis_RigidBodyAPI_WRet(const PXR_NS::UsdPrim& prim, PXR_NS::UsdPrim& ret, bool check_itself = false, bool check_prim = true);
bool hasconflictingapis_CollisionAPI_WRet(const PXR_NS::UsdPrim& prim, PXR_NS::UsdPrim& ret, bool check_itself = false, bool check_prim = true);
bool hasconflictingapis_ArticulationRoot_WRet(const PXR_NS::UsdPrim& prim, PXR_NS::UsdPrim& ret, bool check_itself = false, bool check_prim = true);
bool hasconflictingapis_DeformableBodyAPI(const PXR_NS::UsdPrim& prim, bool check_itself = false);
bool hasconflictingapis_PhysxParticleSamplingAPI(const PXR_NS::UsdPrim& prim, bool check_itself = false);
bool isOverConflictingApisSubtreeLimit(const PXR_NS::UsdPrim& prim, unsigned int limit);
std::array<bool, 3> hasconflictingapis_Precompute(const PXR_NS::UsdPrim& prim);
