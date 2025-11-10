// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <omni/physics/ui/IUsdPhysicsUI.h>

namespace omni
{
namespace physx
{
struct IPhysx;
namespace ui
{

class PhysXJointAuthoring
{
    typedef omni::physics::ui::IUsdPhysicsUICustomJointAuthoring::RegistrationID JointRegistrationID;

    omni::physx::IPhysx* mIPhysx;
    omni::physics::ui::IUsdPhysicsUI* mUsdPhysicsUI = nullptr;
    std::vector<uint32_t> mBillboardAssetIds;
    JointRegistrationID customJointRegistrationID;
    float mPrevGizmoScale;
    bool OnJointScale(uint64_t stageId,
                      uint64_t jointUSDPath,
                      const omni::physics::ui::IUsdPhysicsUICustomJointAuthoring::JointScaleData& jointScaleData);
    bool OnJointGetCustomBillboard(uint64_t stageID, uint64_t jointUSDPath, uint32_t& assetId);

public:
    PhysXJointAuthoring();
    ~PhysXJointAuthoring();
    void init(const std::string& extensionPath);
    void shutdown();
    void update();
};

} // namespace ui
} // namespace physx
} // namespace omni
