// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

namespace omni
{
namespace physx
{

struct IPhysxSupportUiRigidBodyManipulator
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxSupportUiRigidBodyManipulator", 0, 1)

    /// Moves an object using physics force.
    ///
    /// \param   'primPath': Usd path to the manipulated prim.
    /// \param   'deltaTranslation': translation delta.
    /// \param   'lockRotation': allow/disallow rotation while translating.
    /// \param   'lockTranslation': allow/disallow translating on other axes while translating.
    bool(CARB_ABI* move)(uint64_t primPath, const carb::Float3& deltaTranslation, bool lockRotation, bool lockTranslation);
    /// Rotates an object using physics force.
    ///
    /// \param   'primPath': Usd path to the manipulated prim.
    /// \param   'pivotWorldPos': world pivot position to rotate around.
    /// \param   'deltaRotation': target rotation quaternion.
    /// \param   'lockRotation': allow/disallow rotation on other axes while rotating.
    /// \param   'lockTranslation': allow/disallow translating while rotating.
    bool(CARB_ABI* rotate)(uint64_t primPath,
                           const carb::Float3& pivotWorldPos,
                           const carb::Float4& deltaRotation,
                           bool lockRotation,
                           bool lockTranslation);
    /// Must be called when manipulation begins, before calling move() or rotate().
    ///
    /// \param   'primPath': Usd path to the manipulated prim.
    void(CARB_ABI* manipulationBegan)(uint64_t primPath);

    /// Must be called when manipulation ends, always in pairs with manipulation_began().
    ///
    /// \param   'primPath': Usd path to the manipulated prim.
    void(CARB_ABI* manipulationEnded)(uint64_t primPath);
};

} // namespace physx
} // namespace omni
