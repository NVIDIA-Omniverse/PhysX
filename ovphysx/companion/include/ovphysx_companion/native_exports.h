// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

// Export an entry point from a separately compiled companion plugin.
#if defined(_WIN32)
#define OVPHYSX_COMPANION_PLUGIN_EXPORT extern "C" __declspec(dllexport)
#else
#define OVPHYSX_COMPANION_PLUGIN_EXPORT extern "C" __attribute__((visibility("default")))
#endif
