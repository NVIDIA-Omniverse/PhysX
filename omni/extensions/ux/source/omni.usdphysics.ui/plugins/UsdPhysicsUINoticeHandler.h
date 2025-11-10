// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
struct UsdPhysicsUINoticeListener : public pxr::TfWeakBase
{
    JointAuthoringManager* mJointAuthoringManager = nullptr;
    pxr::UsdStageRefPtr mStage = nullptr;

    void handle(const pxr::UsdNotice::ObjectsChanged& objectsChanged);

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
