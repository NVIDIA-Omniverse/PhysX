// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

// PT: standard macros (e.g. see D3D_SAFE_RELEASE) that we really should have somewhere already and use everywhere

#define SAFE_RELEASE(x) \
    if (x)              \
    {                   \
        x->release();   \
        x = nullptr;    \
    }

#define SAFE_DELETE_ALLOCABLE_SINGLE(x)   \
    if (x && x->mOwnsMemory)              \
    {                           \
        delete x;               \
        x = nullptr;            \
    }

#define SAFE_DELETE_SINGLE(x)   \
    if (x)                      \
    {                           \
        delete x;               \
        x = nullptr;            \
    }

#define SAFE_DELETE_ARRAY(x)    \
    if (x)                      \
    {                           \
        delete[] x;             \
        x = nullptr;            \
    }

///////////////////////////////////////////////////////////////////////////////
