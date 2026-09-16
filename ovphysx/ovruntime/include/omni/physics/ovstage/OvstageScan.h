// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-11 AC-12
 */
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
class OvstageSource;

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
// through the scan backend below. With `attached` (the attach's live source over
// this instance) the scan reads through it and the returned ScannedStage borrows
// it (`sourcePtr() == attached`, which must outlive the scan); otherwise the
// scan owns a fresh source.
omni::physics::parse::ScannedStage scanStageOvstage(ovstage_instance_t* instance,
                                                    ovx_path_dictionary_t* dict,
                                                    parse::IDescriptorAllocator& allocator,
                                                    ovstage_ordinal_t readOrdinal = 1,
                                                    const OvstageScanFilter* filter = nullptr,
                                                    uint64_t usdStageId = 0,
                                                    OvstageSource* attached = nullptr);

// The ovstage scan backend (ADR-0002 M2c). Register it via
// `omni::physics::parse::setScanBackend(makeOvstageScanBackend(payload))` so the
// runtime walker dispatch produces a ScannedStage from ovstage. Its `scan()`
// interprets `AttachTarget::nativeStage` as a `const OvstageAttach*` (same payload
// the ovstage parse backend consumes) and delegates to `scanStageOvstage`.
//
// `attachPayload` is the `const OvstageAttach*` this backend is being registered
// for -- the same pointer the attach hands out as `AttachTarget::nativeStage`. It
// is required, not a convenience: `nativeStage` is a bare `const void*` with no
// tag naming the backend that minted it, so without a known-good pointer to
// compare against, a target belonging to another backend (a USD attach's
// `UsdStageWeakPtr*`) cannot be told apart from a real payload and would be
// reinterpreted -- undefined behaviour. `scan()` therefore serves this payload and
// reports any other target instead of dereferencing it. Not owned; must outlive
// the registration (the producer already keeps it alive for the attach).
std::unique_ptr<omni::physics::parse::IScanBackend> makeOvstageScanBackend(const void* attachPayload);

} // namespace omni::physics::ovstage
