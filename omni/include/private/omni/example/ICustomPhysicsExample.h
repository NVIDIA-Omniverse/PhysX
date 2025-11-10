// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

namespace omni
{

namespace example
{


/// Example interfae
struct ICustomPhysicsExample
{
    CARB_PLUGIN_INTERFACE("omni::example::ICustomPhysicsExample", 1, 0)

    /// Example interface to print a string into output
    ///
    /// \param text Text to print
    void(CARB_ABI* printWord)(const char* text);
};


} // namespace example
} // namespace omni
