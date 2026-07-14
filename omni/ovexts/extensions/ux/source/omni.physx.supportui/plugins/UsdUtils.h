// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

namespace omni
{
namespace physx
{

bool setTokenAttributeWithSdf(PXR_NS::UsdStageRefPtr stage,
                              PXR_NS::UsdPrim& prim,
                              PXR_NS::TfToken attributeToken,
                              PXR_NS::TfToken value);

bool isComponentKind(const PXR_NS::UsdPrim& bodyPrim);

bool physicsSceneEnableCcd(PXR_NS::UsdPrim& prim, bool state);
bool physicsSceneIsCcdEnabled(const PXR_NS::UsdPrim& prim, bool& value);

const PXR_NS::UsdEditTarget& SetSessionLayer(PXR_NS::UsdStageRefPtr stage);

// returns true if any token from physics or physx schema is applied on the prim
bool hasPhysicsPhysXSchemaApplied(const PXR_NS::UsdPrim& prim);

// returns true if passed token exists in changed fields vector on specified prim
bool hasChangedToken(const PXR_NS::UsdNotice::ObjectsChanged& objectsChanged,
                     const PXR_NS::SdfPath& primPath,
                     const PXR_NS::TfToken token);

} // namespace physx
} // namespace omni
