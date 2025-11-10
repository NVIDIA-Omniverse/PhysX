// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

template <typename Lambda>
struct DeferLambda
{
    Lambda deferLambda;
    DeferLambda(Lambda deferLambda) : deferLambda(deferLambda)
    {
    }
    ~DeferLambda()
    {
        deferLambda();
    }
};

template <typename Lambda>
DeferLambda<Lambda> CreateDeferLambda(Lambda deferLambda)
{
    return DeferLambda<Lambda>(deferLambda);
};
