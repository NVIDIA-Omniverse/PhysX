// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

namespace omni
{
namespace physx
{

bool setTokenAttributeWithSdf(pxr::UsdStageRefPtr stage,
                              pxr::UsdPrim& prim,
                              pxr::TfToken attributeToken,
                              pxr::TfToken value);

bool isComponentKind(const pxr::UsdPrim& bodyPrim);

bool physicsSceneEnableCcd(pxr::UsdPrim& prim, bool state);
bool physicsSceneIsCcdEnabled(const pxr::UsdPrim& prim, bool& value);

const pxr::UsdEditTarget& SetSessionLayer(pxr::UsdStageRefPtr stage);

// returns true if any token from physics or physx schema is applied on the prim
bool hasPhysicsPhysXSchemaApplied(const pxr::UsdPrim& prim);

// returns true if passed token exists in changed fields vector on specified prim
bool hasChangedToken(const pxr::UsdNotice::ObjectsChanged& objectsChanged,
                     const pxr::SdfPath& primPath,
                     const pxr::TfToken token);

} // namespace physx
} // namespace omni
