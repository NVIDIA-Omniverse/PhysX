// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-ATTACH-EXECUTION-001
 * @covers AC-1 AC-2 AC-3 AC-4
 */

#pragma once

#include <cstdint>
#include <omni/physics/AttachHandle.h>
#include <omni/physx/IPhysxSimulation.h>

namespace omni
{
namespace kit
{
struct StageUpdateSettings;
}
} // namespace omni

namespace omni
{
namespace physx
{
class PhysXScene;
namespace usdparser { class AttachedStage; }

void physXUpdateNonRender(usdparser::AttachedStage& attachedStage, float elapsedSecs, float currentTime);
void physXUpdateSceneNonRender(usdparser::AttachedStage& attachedStage, uint64_t scenePath, float elapsedSecs, float currentTime);
void physxSimulate(usdparser::AttachedStage& attachedStage, float elapsedSecs, float currentTime);
void physxSimulateScene(usdparser::AttachedStage& attachedStage, uint64_t scenePath, float elapsedSecs, float currentTime);
void physxFetchResults(usdparser::AttachedStage& attachedStage);
void physxFetchResultsScene(usdparser::AttachedStage& attachedStage, uint64_t scenePath);
bool physxCheckResults(usdparser::AttachedStage& attachedStage);
bool physxCheckResultsScene(usdparser::AttachedStage& attachedStage, uint64_t scenePath);
void physXUpdateUsd(usdparser::AttachedStage& attachedStage);
void physXUpdate(usdparser::AttachedStage& attachedStage, float currentTime, float elapsedSecs, bool enableUpdate);
void waitForSimulationCompletion(const usdparser::AttachedStage& attachedStage, bool doPostWork);

// Public dispatchers resolve an exact attachment and enforce execution admission.
RuntimeResult physXUpdateNonRender(AttachHandle attach, float elapsedSecs, float currentTime);
RuntimeResult physXUpdateSceneNonRender(AttachHandle attach, uint64_t scenePath, float elapsedSecs, float currentTime);
void physXUpdateNonRenderDispatch(
    PhysXScene* sc, float elapsedSecs, float currentTime, bool forceAsync = false, bool noStepping = false);
RuntimeResult physxSimulate(AttachHandle attach, float elapsedSecs, float currentTime);
RuntimeResult physxFetchResults(AttachHandle attach);
bool physxCheckResults(AttachHandle attach);
RuntimeResult physXUpdateUsd(AttachHandle attach);
RuntimeResult physxSimulateScene(AttachHandle attach, uint64_t scenePath, float elapsedSecs, float currentTime);
RuntimeResult physxFetchResultsScene(AttachHandle attach, uint64_t scenePath);
bool physxCheckResultsScene(AttachHandle attach, uint64_t scenePath);
void updateCooking();

} // namespace physx
} // namespace omni
