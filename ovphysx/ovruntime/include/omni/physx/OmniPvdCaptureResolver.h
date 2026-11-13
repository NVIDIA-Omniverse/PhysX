// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-OMNIPVD-CAPTURE-001
 * @covers AC-5
 */

#pragma once

#include <omni/physics/AttachHandle.h>
#include <omni/physx/RuntimeResult.h>
#include <string>
#include <vector>

namespace physx
{
class PxBase;
}

namespace omni::physx
{

// The caller validates exact absolute paths, drains shared work and excludes
// attachment transitions. Resolution takes one owner execution scope and expands
// all matching supported native objects.
RuntimeResult omniPvdResolveCaptureObjects(omni::physics::AttachHandle attach, const std::vector<std::string>& paths,
                                          std::vector<const ::physx::PxBase*>& objects);

} // namespace omni::physx
