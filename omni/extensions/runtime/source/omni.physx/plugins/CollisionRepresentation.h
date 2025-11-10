// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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
