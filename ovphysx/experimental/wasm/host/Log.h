// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once
namespace omni { namespace log { class ILog; } }
omni::log::ILog* ovphysxWasmLog();
void ovphysxWasmInitializeLog();
