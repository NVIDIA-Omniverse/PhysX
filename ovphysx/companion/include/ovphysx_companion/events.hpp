// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-EVENTS-001
// @covers AC-1 AC-4
#pragma once
#include "events.h"
#include <array>
#include <algorithm>
#include <optional>
namespace ovphysx::companion
{
struct ContactPoint
{
    std::array<float,3> position,normal,impulse;
    float separation;
};
inline std::optional<ovphysx_companion_contact_state_t> contactState(
    const ovphysx_companion_event_t& event) noexcept
{
    if (event.kind!=OVPHYSX_COMPANION_EVENT_CONTACT || event.type>OVPHYSX_COMPANION_CONTACT_PERSISTS)
        return std::nullopt;
    return static_cast<ovphysx_companion_contact_state_t>(event.type);
}
inline std::optional<ovphysx_companion_trigger_state_t> triggerState(
    const ovphysx_companion_event_t& event) noexcept
{
    if (event.kind!=OVPHYSX_COMPANION_EVENT_TRIGGER || event.type>OVPHYSX_COMPANION_TRIGGER_LEAVE)
        return std::nullopt;
    return static_cast<ovphysx_companion_trigger_state_t>(event.type);
}
// Lost/empty contacts and other event kinds have no point payload.
inline std::optional<ContactPoint> contactPoint(const ovphysx_companion_event_t& event) noexcept
{
    const std::optional<ovphysx_companion_contact_state_t> state=contactState(event);
    if (!state || *state==OVPHYSX_COMPANION_CONTACT_LOST || !event.point_count || event.point_index>=event.point_count)
        return std::nullopt;
    ContactPoint point{};
    std::copy_n(event.position,3,point.position.begin());
    std::copy_n(event.normal,3,point.normal.begin());
    std::copy_n(event.impulse,3,point.impulse.begin());
    point.separation=event.separation;
    return point;
}
}
