// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <omni/kit/IStageUpdate.h>

void applyGizmos();
void applyGizmosDrawCall(float, float, const omni::kit::StageUpdateSettings*, void*);
void selectGizmo(uint32_t selectedGizmo, uint32_t selectionState);
void setGizmoScale(uint32_t selectedGizmo, float scale);
void setGizmoScale(float scale);
void cleanupDebugViz();
