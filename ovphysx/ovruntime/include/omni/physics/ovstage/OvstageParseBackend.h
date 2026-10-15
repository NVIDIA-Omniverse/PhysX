// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-2 AC-6
 */
#pragma once

// Ovstage parse backend. Each runtime attachment owns its provider and scanner.
// Developer backend tests can construct providers directly.
//
// Factory only — the concrete backend type stays private to the
// omni.physics.ovstage library.

#include <ovstage/ovstage.h>
#include <ovstage/ovx_path_dictionary.h>

#include <cstddef> // offsetof
#include <memory>

namespace omni { namespace physics { namespace parse { class IParseBackend; } } }

namespace omni
{
namespace physics
{
namespace ovstage
{

// What `AttachTarget::nativeStage` points to for the ovstage backend.
//
// ADR-0005 describes nativeStage as "the ovstage instance" for this backend;
// in practice the source also needs the application-owned path dictionary and
// an optional explicit developer backing id. So nativeStage points to this small
// backend-specific struct rather than the bare instance. The read ordinal and
// normalized resident backing id are separate AttachTarget fields. The producer
// retains the referenced instance and dictionary through detach. The runtime
// copies this descriptor at attach; a false attach result retains no references.
struct OvstageAttach
{
    ovstage_instance_t* instance = nullptr;
    ovx_path_dictionary_t* dict = nullptr;
    // NOTE: the sealed read ordinal is no longer carried here. It is passed
    // explicitly through IPhysxSimulation::attachOvstage(payload, readOrdinal)
    // and reaches the backends via AttachTarget::readOrdinal.
    // Optional developer compatibility stage (UsdUtilsStageCache id), supplied
    // explicitly by a USD-aware caller. The runtime classifies local residency
    // before carrying it in AttachTarget::residentBackingStageId.
    // 0 = ovstage-only attach; never query the instance for a backing USD stage.
    // Production ovphysx always supplies 0.
    uint64_t usdStageId = 0;
};

// ABI guard: ovphysx builds a layout-compatible mirror of this struct
// (OvstageAttachPayload in ovphysxSDK.hpp) and hands it across the module
// boundary as a const void* that this backend reinterprets as OvstageAttach.
// ovphysx cannot include this header (it would re-couple libovphysx to the
// ovstage headers), so the shared ABI layout is pinned independently on both
// sides. Keep these numbers identical to the OvstageAttachPayload asserts.
static_assert(sizeof(OvstageAttach) == 24, "OvstageAttach ABI size drifted from ovphysx OvstageAttachPayload");
static_assert(offsetof(OvstageAttach, instance) == 0, "OvstageAttach::instance offset drifted");
static_assert(offsetof(OvstageAttach, dict) == 8, "OvstageAttach::dict offset drifted");
static_assert(offsetof(OvstageAttach, usdStageId) == 16, "OvstageAttach::usdStageId offset drifted");

// Create the ovstage parse backend. The returned backend interprets
// `AttachTarget::nativeStage` as a `const OvstageAttach*`. The write sink is
// null by design; configured sources create a pull-based change feed.
std::unique_ptr<omni::physics::parse::IParseBackend> makeOvstageParseBackend();

} // namespace ovstage
} // namespace physics
} // namespace omni
