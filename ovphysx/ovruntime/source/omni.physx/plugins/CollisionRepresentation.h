// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <omni/physx/IPhysxCooking.h>

namespace omni
{
namespace physx
{
PhysxCollisionRepresentationTask requestConvexCollisionRepresentation(
    const PhysxCollisionRepresentationRequest& request, PhysxCollisionRepresentationConvexResult::CallbackType onResult);
void cancelCollisionRepresentationTask(PhysxCollisionRepresentationTask task, bool invokeCallbackAnyway);
} // namespace physx
} // namespace omni
