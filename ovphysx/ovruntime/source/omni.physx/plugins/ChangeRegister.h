// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <omni/physics/parse/Handles.h> // ObjectKey, for registerDriveTimeSampledChanges/registerSceneTimeSampledChanges/registerJointTimeSampledChanges

#include <string>
#include <vector>

namespace omni
{
namespace physx
{
namespace usdparser
{
struct ChangeParams;
class AttachedStage;
} // namespace usdparser

void registerChangeParams(std::vector<usdparser::ChangeParams>& changeParams);
// Live-notify time-sampled attribute registration (ChangeRegister.cpp). Reads
// through IPhysicsSource::isAttributeTimeSampled/internToken and stores into
// AttachedStage's ObjectKey/TokenId-keyed mTimeSampledAttributes -- no pxr
// dependency; on a backend that never reports an attribute as time-sampled
// (e.g. OvstageSource) they naturally register nothing.
void registerDriveTimeSampledChanges(usdparser::AttachedStage&, omni::physics::parse::ObjectKey jointPrimKey, std::string driveAxis);
void registerSceneTimeSampledChanges(usdparser::AttachedStage&, omni::physics::parse::ObjectKey scenePrimKey);
void registerJointTimeSampledChanges(usdparser::AttachedStage&, omni::physics::parse::ObjectKey jointPrimKey);

void registerSpatialTendonChangeParams(usdparser::AttachedStage&, const std::string& instanceName);
void registerTendonAttachmentChangeParams(usdparser::AttachedStage&, const std::string& instanceName);
void registerTendonAttachmentLeafChangeParams(usdparser::AttachedStage&, const std::string& instanceName);

void registerFixedTendonChangeParams(usdparser::AttachedStage&, const std::string& instanceName);
void registerTendonAxisChangeParam(usdparser::AttachedStage&, const std::string& instanceName);

void registerDeformablePoseChangeParams(usdparser::AttachedStage&, const std::string& instanceName);

} // namespace physx
} // namespace omni
