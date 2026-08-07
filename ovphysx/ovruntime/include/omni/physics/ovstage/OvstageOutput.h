// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <omni/physics/parse/Handles.h> // parse::ObjectKey

#include <ovstage/ovx_path_dictionary.h> // ovx_primpath_list_t / ovx_path_dictionary_t

#include <cstddef>

namespace omni::physics::parse
{
class IPhysicsSource;
}

namespace omni::physics::ovstage
{

// Build an immutable ovx_primpath_list_t over `keys` using the active source's
// shared path dictionary, interning each key's prim-path handle canonically (no
// string round-trip). `source` must be the ovstage-backed `OvstageSource`;
// returns OVX_INVALID_PRIMPATH_LIST when it is any other backend (the output read
// is ovstage-only). The list is owned by the caller and must be released with
// `ovx_path_dictionary_destroy_path_list` on `outDict` (also returned, so the
// caller can destroy the list and reuse the dictionary for the write-back).
ovx_primpath_list_t buildPathList(const omni::physics::parse::IPhysicsSource& source,
                                  const omni::physics::parse::ObjectKey* keys,
                                  size_t count,
                                  ovx_path_dictionary_t** outDict);

} // namespace omni::physics::ovstage
