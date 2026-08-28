// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <omni/physics/parse/Allocator.h>
#include <omni/physics/parse/ScanBackend.h>
#include <omni/physics/parse/ScannedStage.h>

#include <ovstage/ovstage.h>
#include <ovstage/ovx_path_dictionary.h>

#include <memory>

namespace omni::physics::ovstage
{


struct OvstageScanFilter;

// Produce a `ScannedStage` from an ovstage instance (ADR-0002 Milestone 2b).
// Runs the OvstageWalker, then wraps the emitted descriptors + a live
// OvstageSource (the scan's path/key/token resolver) into a ScannedStage via
// the parse core's `makeScannedStageFromSource` builder. Scope matches the
// walker: Scene + RigidBody + Box collision.
//
// Returns the USD-free `parse::ScannedStage` — ovstage depends only on the
// parse core, never on the USD layer. A USD consumer wraps the result in
// `omni::physics::usd::ScannedStage` for SdfPath/TfToken resolution.
//
// The direct entry a parity test / consumer can call. Production routing goes
// through the scan backend below.
omni::physics::parse::ScannedStage scanStageOvstage(ovstage_instance_t* instance,
                                                    ovx_path_dictionary_t* dict,
                                                    parse::IDescriptorAllocator& allocator,
                                                    ovstage_ordinal_t readOrdinal = 1,
                                                    const OvstageScanFilter* filter = nullptr,
                                                    uint64_t usdStageId = 0);

// The ovstage scan backend (ADR-0002 M2c). Register it via
// `omni::physics::parse::setScanBackend(makeOvstageScanBackend())` so the runtime
// walker dispatch produces a ScannedStage from ovstage. Its `scan()` interprets
// `AttachTarget::nativeStage` as a `const OvstageAttach*` (same payload the
// ovstage parse backend consumes) and delegates to `scanStageOvstage`.
std::unique_ptr<omni::physics::parse::IScanBackend> makeOvstageScanBackend();

} // namespace omni::physics::ovstage
