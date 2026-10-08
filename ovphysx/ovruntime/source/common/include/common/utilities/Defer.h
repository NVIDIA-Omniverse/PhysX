// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
