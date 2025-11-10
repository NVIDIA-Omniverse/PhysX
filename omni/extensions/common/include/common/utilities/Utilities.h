// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/logging/Log.h>
#include <carb/events/EventsUtils.h>

// asInt() is the same as SdfPath::_AsInt()
// Fabric relies on asInt(a)==asInt(b) <=> a is same path as b,
// which is how SdfPath::operator== is currently defined.
// If USD changes sizeof(pxr::SdfPath), we will need to change

inline uint64_t asInt(const pxr::SdfPath& path)
{
    static_assert(sizeof(pxr::SdfPath) == sizeof(uint64_t), "Change to make the same size as pxr::SdfPath");
    uint64_t ret;
    std::memcpy(&ret, &path, sizeof(pxr::SdfPath));

    return ret;
}

inline const pxr::SdfPath& intToPath(const uint64_t& path)
{
    static_assert(sizeof(pxr::SdfPath) == sizeof(uint64_t), "Change to make the same size as pxr::SdfPath");

    return reinterpret_cast<const pxr::SdfPath&>(path);
}

inline void createEventFromSdfPath(carb::events::IEventPtr& eventPtr,
                            const char* itemName,
                            carb::dictionary::IDictionary* dict,
                            const pxr::SdfPath& path)
{
    const uint64_t ui64Path = asInt(path);

    carb::dictionary::Item* item = dict->createItem(eventPtr->payload, itemName, carb::dictionary::ItemType::eDictionary);
    dict->setArray<int32_t>(item, (const int32_t*)&ui64Path, 2);
}

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
    // Find min and max scale values
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

class ScopedLayerEdit
{
public:
    ScopedLayerEdit(pxr::UsdStageWeakPtr stage, const pxr::SdfLayerHandle& layer)
        : m_usdEditCtx(stage, layer), m_layer(layer)
    {
        m_wasLayerEditable = m_layer->PermissionToEdit();
        m_layer->SetPermissionToEdit(true);
        CARB_ASSERT(m_layer->PermissionToEdit());
    }

    ~ScopedLayerEdit()
    {
        m_layer->SetPermissionToEdit(m_wasLayerEditable);
    }

private:
    pxr::UsdEditContext m_usdEditCtx;
    pxr::SdfLayerHandle m_layer;
    bool m_wasLayerEditable;
};
