// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

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

void physXUpdateNonRender(float elapsedSecs, float currentTime);
void physXUpdateSceneNonRender(uint64_t scenePath, float elapsedSecs, float currentTime);
void physXUpdateNonRenderDispatch(
    PhysXScene* sc, float elapsedSecs, float currentTime, bool forceAsync = false, bool noStepping = false);
void physXUpdate(float currentTime, float elapsedSecs, bool enableUpdate);
void waitForSimulationCompletion(bool doPostWork);
void physxSimulate(float elapsedSecs, float currentTime);
void physxFetchResults();
bool physxCheckResults();
void physXUpdateUsd();
void physxSimulateScene(uint64_t scenePath, float elapsedSecs, float currentTime);
void physxFetchResultsScene(uint64_t scenePath);
bool physxCheckResultsScene(uint64_t scenePath);
void updateCooking();

} // namespace physx
} // namespace omni
