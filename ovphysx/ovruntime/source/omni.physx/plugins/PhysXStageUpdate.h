// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-PUBLICAPI-001
// @covers AC-51

#pragma once

#include <omni/physics/AttachHandle.h>
#include <omni/physx/RuntimeResult.h>

namespace omni
{
namespace physx
{

// @implements REQ-ATTACH-IO-001
// @covers AC-2
// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-1 AC-2
// Native USD stage-update entries. Every operation selects an explicit attachment;
// physXStageUpdate completes synchronously under runtime admission.
AttachHandle onPhysXAttach(long stageId);
RuntimeResult onPhysXDetach(AttachHandle attach);
RuntimeResult physXPause(AttachHandle attach);
RuntimeResult physXReset(AttachHandle attach);
RuntimeResult physXResume(AttachHandle attach, float currentTime);
RuntimeResult physXStageUpdate(AttachHandle attach, float currentTime, float elapsedSecs, bool enableUpdate);

// Process-wide pump for queued USD property queries. Requires idle runtime.
RuntimeResult updatePropertyQueries();

class PhysXStageUpdate
{
public:
    void attachPropertyQueryStageUpdate()
    {
        mPropertyQueryAttached = true;
    }

    void detachPropertyQueryStageUpdate()
    {
        mPropertyQueryAttached = false;
    }

    bool isPropertyQueryAttached() const
    {
        return mPropertyQueryAttached;
    }


private:
    bool mPropertyQueryAttached{ false };
};


} // namespace physx
} // namespace omni
