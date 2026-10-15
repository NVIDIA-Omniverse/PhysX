// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-LIFETIME-001
// @covers AC-3 AC-4

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-1 AC-3 AC-4

#include <omni/physx/IPhysxCooking.h>

namespace omni
{
namespace physx
{
PhysxCollisionRepresentationTask requestConvexCollisionRepresentation(
    const PhysxCollisionRepresentationRequest& request, PhysxCollisionRepresentationConvexResult::CallbackType onResult);
void retireCollisionRepresentationTasks(AttachHandle attach);
// Internal diagnostic: opaque token membership, without borrowing native task storage.
bool isCollisionRepresentationTaskLive(PhysxCollisionRepresentationTask task);
RuntimeResult cancelCollisionRepresentationTask(PhysxCollisionRepresentationTask task, bool invokeCallbackAnyway);
} // namespace physx
} // namespace omni
