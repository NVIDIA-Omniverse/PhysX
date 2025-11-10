// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once
#include <carb/BindingsPythonUtils.h>

template <typename T = int>
struct ResultBuffer
{
    ~ResultBuffer()
    {
        if (ptr)
        {
            free(ptr);
            ptr = nullptr;
        }
        size = 0;
    }

    static void* allocate(size_t numBytes)
    {
        return (void*)malloc(numBytes);
    }

    explicit operator py::list() const
    {
        py::list list;
        for (uint32_t i = 0; i < size; ++i)
        {
            list.append(ptr[i]);
        }
        return list;
    }

    T* ptr = nullptr;
    uint32_t size = 0;
};
