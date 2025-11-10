// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include <omni/Function.h>

#include <vector>

namespace omni
{
namespace physics
{

/**
\brief Contact event type
*/
struct ContactEventType
{
    enum Enum
    {
        eCONTACT_FOUND, //!< Contact issued for newly found contact pairs.
        eCONTACT_LOST, //!< Contact issued for contact pair lost.
        eCONTACT_PERSIST //!< Contact issued for persistent contact pairs.
    };
};

/**
\brief Contact event header, issued for contact pair.
Contains information about the pair and the number of contact data.
*/
struct ContactEventHeader
{
    ContactEventType::Enum type; //!< Contact event header type.
    long stageId; //!< Stage id of the simulated USD stage.
    uint64_t actor0; //!< Actor0 of the contact pair, can be retyped to SdfPath.
    uint64_t actor1; //!< Actor1 of the contact pair, can be retyped to SdfPath.
    uint64_t collider0; //!< Collider0 of the contact pair, can be retyped to SdfPath.
    uint64_t collider1; //!< Collider1 of the contact pair, can be retyped to SdfPath.
    uint32_t contactDataOffset; //!< Contact data offset index to the contact data array.
    uint32_t numContactData; //!< Number of contact data in the contact data array for given pair.
    uint32_t frictionAnchorsDataOffset; //!< Friction anchros offset index to the friction anchors data array.
    uint32_t numfrictionAnchorsData; //!< Number of friction anchors data in the friction anchors data array for given
                                     //!< pair.
    uint32_t protoIndex0; //!< Point instancer prototypeIndex for collider0, used only for
                          // point instancers otherwise 0xFFFFFFFF
    uint32_t protoIndex1; //!< Point instancer prototypeIndex for collider1, used only for
                          // point instancers otherwise 0xFFFFFFFF
};

/**
\brief Contact data for contact pair, each pair can have multiple contact data
*/
struct ContactData
{
    carb::Float3 position; //!< Contact position.
    carb::Float3 normal; //!< Contact normal.
    carb::Float3 impulse; //!< Contact impulse.
    float separation; //!< Contact separation.
    uint32_t faceIndex0; //!< Contact face index for collider0 used only for triangle meshes.
    uint32_t faceIndex1; //!< Contact face index for collider1 used only for triangle meshes.
    uint64_t material0; //!< Contact collider0 assigned material.
    uint64_t material1; //!< Contact collider1 assigned material.
};

/**
\brief FrictionAnchor data for contact pair, each pair can have multiple FrictionAnchor data
*/
struct FrictionAnchor
{
    carb::Float3 position; //!< The position of the friction anchor in world space.
    carb::Float3 impulse; //!<  The impulse applied at the friction anchor, in world space. Divide by the simulation
                          //!<  time step to get a force value.
};

using ContactEventHeaderVector = std::vector<omni::physics::ContactEventHeader>;
using ContactDataVector = std::vector<omni::physics::ContactData>;
using FrictionAnchorsDataVector = std::vector<omni::physics::FrictionAnchor>;


/**
\brief Contact report event function including friction anchors

\param eventHeaders Event headers buffer
\param contactData Contact data buffer
\param frictionAnchors FrictionAnchors data buffer
*/
using OnContactReportEventFn = std::function<void(const ContactEventHeaderVector& eventHeaders,
                                                  const ContactDataVector& contactData,
                                                  const FrictionAnchorsDataVector& frictionAnchors)>;

} // namespace physics
} // namespace omni
