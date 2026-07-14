// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

namespace omni
{
namespace physics
{
namespace ui
{
class JointAuthoringManager;
struct UsdPhysicsUINoticeListener : public PXR_NS::TfWeakBase
{
    JointAuthoringManager* mJointAuthoringManager = nullptr;
    PXR_NS::UsdStageRefPtr mStage = nullptr;

    void handle(const PXR_NS::UsdNotice::ObjectsChanged& objectsChanged);

    void blockUsdNoticeHandler(bool block)
    {
        mBlockNoticeHandler = block;
    }
    bool isUsdNoticeHandlerEnabled() const
    {
        return mBlockNoticeHandler;
    }

private:
    bool mBlockNoticeHandler = false;
};
} // namespace ui
} // namespace physics
} // namespace omni
