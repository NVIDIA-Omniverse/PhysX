// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cmath>

#include <carb/events/EventsUtils.h>

// pxr-free half of Utilities.h, split out so USD-free consumers (e.g. omni.physx.cooking, ADR-0018)
// can pull in sendErrorEvent / the scale helpers without dragging in Utilities.h's pxr includes.
// Utilities.h includes this header too, so existing direct includers of Utilities.h are unaffected.

template <typename... ValuesT>
void sendErrorEvent(carb::events::IEventStreamPtr eventStream, carb::events::EventType type, ValuesT... values)
{
    using namespace carb::events;
    carb::events::IEventPtr event = carb::stealObject(
        eventStream->createEventPtr(type, kGlobalSenderId, values...));

    eventStream->push(event.get());
}

// eps is relative tolerance on difference between max and min scale elements
template<typename T>
inline bool scaleIsUniform(T scaleX, T scaleY, T scaleZ, T eps = T(1.0e-5))
{
    T lo, hi;

    if (scaleX < scaleY)
    {
        lo = scaleX;
        hi = scaleY;
    }
    else
    {
        lo = scaleY;
        hi = scaleX;
    }

    if (scaleZ < lo)
    {
        lo = scaleZ;
    }
    else if (scaleZ > hi)
    {
        hi = scaleZ;
    }

    if (lo*hi < 0.0)
    {
        return false;   // opposite signs
    }

    return hi > 0.0 ? hi - lo <= eps*lo : lo - hi >= eps*hi;
}

inline float getAbsValue(float value)
{
    return ::fabsf(value);
}

inline double getAbsValue(double value)
{
    return ::fabs(value);
}

template<typename T>
inline bool scaleIsIdentity(T scaleX, T scaleY, T scaleZ, T eps = T(1.0e-4))
{
    return ((getAbsValue(T(1.0) - scaleX) < eps) && (getAbsValue(T(1.0) - scaleY) < eps) && (getAbsValue(T(1.0) - scaleZ) < eps));
}
