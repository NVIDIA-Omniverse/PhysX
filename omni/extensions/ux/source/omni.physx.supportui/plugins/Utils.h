// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/events/IEvents.h>
#include <omni/usd/UsdTypes.h>

namespace omni
{
namespace physx
{

/// Resets provided setting on 'path' to default value found on [default/'path'] path
/// @param path: full setting path
void resetSettingAtPath(const char* path);

/// @param primPath: Usd path to the prim
/// @param collType: collider type. false = static, true = dynamic
/// @param simplificationType: if collType is static then StaticColliderSimplificationType, otherwise
/// DynamicColliderSimplificationType
/// @param numRemainingCollTasks: number of remaining collision tasks
/// @param numTotalCollTasks: total number of collision tasks
void sendColliderCreatedEvent(carb::events::IEventStreamPtr eventStream,
                              carb::dictionary::IDictionary* dict,
                              const pxr::SdfPath& primPath,
                              bool colliderType,
                              int simplificationType,
                              size_t numRemainingCollTasks,
                              size_t numTotalCollTasks);

/// @param primPath: Usd path to the prim
/// @param created: true if created false if removed
void sendRigidBodyApiChangedEvent(carb::events::IEventStreamPtr eventStream,
                                  carb::dictionary::IDictionary* dict,
                                  const pxr::SdfPath& primPath,
                                  bool created);

/// @param primPath: Usd path to the prim
void sendKinematicsToggleChangedEvent(carb::events::IEventStreamPtr eventStream,
                                      carb::dictionary::IDictionary* dict,
                                      const pxr::SdfPath& primPath);

/// @param primPath: Usd path to the prim with an existing collider (reason for cancel)
void sendAutoCollCanceledChangedEvent(carb::events::IEventStreamPtr eventStream,
                                      carb::dictionary::IDictionary* dict,
                                      const pxr::SdfPath& primPath);

const char* toString(omni::usd::StageState stageState);
const char* toString(omni::usd::StageEventType stageEventType);

} // namespace physx
} // namespace omni
