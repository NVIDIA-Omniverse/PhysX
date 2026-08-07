// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-2
 */
#pragma once

// ovstage parse backend (ADR-0002, selected via ADR-0005). It builds an
// OvstageSource for a populated ovstage instance instead of a USD stage.
// IPhysxSimulation::attachOvstage installs it for the attachment and restores
// the USD backend on detach. Direct backend tests may install it manually while
// no stage is attached.
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
// the attach-time backing candidate. So nativeStage is a pointer to this small
// backend-specific struct rather than the bare instance. The read ordinal and
// normalized resident backing id are separate AttachTarget fields. The producer
// keeps this payload alive until detach after a successful attachment. A false
// attach result retains no reference to the payload.
struct OvstageAttach
{
    ovstage_instance_t* instance = nullptr;
    ovx_path_dictionary_t* dict = nullptr;
    // NOTE: the sealed read ordinal is no longer carried here. It is passed
    // explicitly through IPhysxSimulation::attachOvstage(payload, readOrdinal)
    // and reaches the backends via AttachTarget::readOrdinal.
    // Candidate backing USD stage id (omni::fabric / UsdUtilsStageCache id).
    // The attach-time runtime validates this candidate, or queries a Fabric-backed
    // instance when it is zero, and carries only the normalized locally resident id
    // in AttachTarget::residentBackingStageId. Parse and scan never consume this
    // raw candidate directly. 0 = query the instance.
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

// Resolve the backing USD stage id for an ovstage attach payload (a
// `const OvstageAttach*`) while preserving the ovstage status. A null instance
// is invalid. For a valid instance, an explicit non-zero
// OvstageAttach::usdStageId returns OVSTAGE_OK without querying the instance;
// otherwise the result from ovstage_get_usd_stage_id is returned.
ovstage_api_status_t queryBackingUsdStageId(const void* attachPayload, uint64_t& outStageId);

} // namespace ovstage
} // namespace physics
} // namespace omni
