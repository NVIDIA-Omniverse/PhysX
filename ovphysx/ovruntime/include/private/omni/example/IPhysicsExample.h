// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

namespace omni
{

namespace example
{


/// Example interfae
struct IPhysicsExample
{
    /// Example interface to print a string into output
    ///
    /// \param text Text to print
    void(CARB_ABI* printWord)(const char* text);
};


} // namespace example
} // namespace omni
