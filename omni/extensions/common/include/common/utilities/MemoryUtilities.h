// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

// Allows wrapping Px objects that need to be deleted with a call to ->release() in unique_ptr<>
struct CallReleaseOnPointer
{
    template <typename Object>
    void operator()(Object* obj)
    {
        obj->release();
    }
};
