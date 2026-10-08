// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <carb/logging/Log.h>
#include <carb/events/EventsUtils.h>
#include <carb/cpp/StringView.h>

#include <common/utilities/CoreUtilities.h>

// pxr-free (ADR-0027). The SdfPath<->uint64 encoders and ScopedLayerEdit that used to live
// here are in omni.physics.usd (UsdPathEncoding.h, ScopedLayerEdit.h).
