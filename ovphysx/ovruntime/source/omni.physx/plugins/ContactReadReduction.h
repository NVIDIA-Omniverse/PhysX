// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-CONTACT-001
 * @covers AC-3, AC-12
 *
 * @implements REQ-READ-CONTACT-003
 * @covers AC-1
 *
 * @implements REQ-READ-CONTACT-004
 * @covers AC-1
 *
 * @implements REQ-READ-CONTACT-005
 * @covers AC-1
 *
 * @implements REQ-READ-CONTACT-006
 * @covers AC-1
 */

// Host reductions of one scene's contact report into the contact-sensor read's columns (ADR-0044). Pure
// functions over the report's buffers and the rows' keys: no read session, scene or view, so each can be
// checked against hand-built headers.

#pragma once

#include <omni/physx/ContactEvent.h>
#include <omni/physics/parse/Handles.h>

#include "tensors/ForceComponent.h"

#include <cstdint>
#include <vector>

namespace omni
{
namespace physx
{
namespace contactread
{

// One scene's contact report: headers, contact points and friction anchors, indexed by the headers.
struct ContactReportView
{
    const ContactEventHeader* headers = nullptr;
    uint32_t numHeaders = 0;
    const ContactData* points = nullptr;
    const FrictionAnchor* anchors = nullptr;
};

// Net force per sensor row, one component: SoA [S, 3]. The normal component sums each pair's
// contact-point impulses, the friction component its friction-anchor impulses, as
// CpuRigidContactView::getNetNormalContactForces / getNetFrictionContactForces do. A header side feeds both
// its actor's row and its collider's row when those distinct keys are present; a shared actor/collider key
// is counted once. Side 0 adds the pair's impulse and side 1 negates it (a header with both sides sensors
// feeds both), a lost pair is skipped, and the sum is scaled to a force by invDt (= 1/dt).
std::vector<float> reduceNetForces(const std::vector<omni::physics::parse::ObjectKey>& sensors,
                                   const ContactReportView& report,
                                   float invDt,
                                   omni::physx::tensors::ForceComponent component);

// Force matrix per sensor row, one component: [S, F, 3] row-major, `filters` holding each row's F filter
// keys. A sensor side's pair lands in the column of the OTHER side's body -- its actor id, or failing that
// its collider id, in the row's filters -- as CpuRigidContactView::getNormalContactForceMatrix /
// getFrictionContactForceMatrix place it; a pair with a body outside the row's filters lands nowhere. Same
// sensor matching, sign, lost-pair and invDt rules as reduceNetForces.
std::vector<float> reduceForceMatrix(const std::vector<omni::physics::parse::ObjectKey>& sensors,
                                     const std::vector<omni::physics::parse::ObjectKey>& filters,
                                     uint32_t filterCount,
                                     const ContactReportView& report,
                                     float invDt,
                                     omni::physx::tensors::ForceComponent component);

// Per-element contact data laid out slot by slot: each sensor row has a fixed number of slots (a filtered
// read: one per filter column; a raw read: one), and a slot's elements follow the previous slots' in report
// order -- the binding's getNormalContactData / getFrictionContactData / getRawContactData layout, sized to the
// step's elements rather than to a capacity.
struct ContactElements
{
    std::vector<int32_t> counts;   // elements per slot, [S, slots per row]
    std::vector<float> force;      // contact point: 1 per element, +/-|impulse|/dt; friction anchor: 3, +/-impulse/dt
    std::vector<float> point;      // 3 per element: the contact point or anchor position
    std::vector<float> normal;     // contact points only: 3 per element, the normal as reported (not flipped)
    std::vector<float> separation; // contact points only: 1 per element
    std::vector<uint64_t> other;   // raw only: 1 per element, the other side's actor id
};

// Filtered detail per sensor row, one component: an element per contact point (normal) or friction anchor
// (friction) of a pair between the sensor and one of its filters' bodies (matched as in reduceForceMatrix), one
// slot per filter column. A side-1 sensor's forces are negated; lost pairs are skipped.
ContactElements reduceFilteredDetail(const std::vector<omni::physics::parse::ObjectKey>& sensors,
                                     const std::vector<omni::physics::parse::ObjectKey>& filters,
                                     uint32_t filterCount,
                                     const ContactReportView& report,
                                     float invDt,
                                     omni::physx::tensors::ForceComponent component);

// Only the filtered detail's counts -- reduceFilteredDetail's ContactElements::counts -- for a read of the
// count companions alone, which needs no element.
std::vector<int32_t> countFilteredDetail(const std::vector<omni::physics::parse::ObjectKey>& sensors,
                                         const std::vector<omni::physics::parse::ObjectKey>& filters,
                                         uint32_t filterCount,
                                         const ContactReportView& report,
                                         omni::physx::tensors::ForceComponent component);

// Raw contacts per sensor row: an element per contact point of every pair the sensor is in, whatever the
// other body, one slot per row, with the other side's actor id -- CpuRigidContactView::getRawContactData's
// elements. Same sensor matching, sign, lost-pair and invDt rules as reduceNetForces.
ContactElements reduceRawContacts(const std::vector<omni::physics::parse::ObjectKey>& sensors,
                                  const ContactReportView& report,
                                  float invDt);

// Contact events per sensor row: every header the row is a side of -- found, persisting and lost alike --
// as seen from that side, in report order, with its contact points and friction anchors concatenated in
// event order. A header whose two sides are both rows appears under each. Each side feeds both its actor's
// row and its collider's row when those distinct keys are present; a shared actor/collider key is counted
// once. For a lost pair whose sensor-side shape was removed
// the report zeroes that side's actor id, so the pair is claimed only while its collider id still names the
// sensor (a body that is its own collider), and by no row when both ids are 0. Identities are the report's
// ids (object keys), reordered own-then-other; values are as reported (no sign change, no dt).
struct ContactEvents
{
    std::vector<int32_t> eventCounts;  // events per row
    std::vector<int32_t> pointCounts;  // contact points per row, across its events
    std::vector<int32_t> anchorCounts; // friction anchors per row, across its events
    // Per event.
    std::vector<int32_t> type;       // ContactEventType::Enum
    std::vector<int32_t> sensorSide; // 0 or 1: the header side the row is
    std::vector<uint64_t> collider, otherBody, otherCollider;
    std::vector<uint32_t> protoIndex, otherProtoIndex;
    std::vector<int32_t> pointCount, anchorCount;
    // Per contact point.
    std::vector<float> pointPosition, pointNormal, pointImpulse; // 3 each
    std::vector<float> pointSeparation;
    std::vector<uint32_t> pointFaceIndex, pointOtherFaceIndex;
    std::vector<uint64_t> pointMaterial, pointOtherMaterial;
    // Per friction anchor.
    std::vector<float> anchorPosition, anchorImpulse; // 3 each
};
ContactEvents reduceContactEvents(const std::vector<omni::physics::parse::ObjectKey>& sensors,
                                  const ContactReportView& report);

} // namespace contactread
} // namespace physx
} // namespace omni
