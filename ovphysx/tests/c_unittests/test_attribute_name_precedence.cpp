// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "../../ovruntime/source/omni.physx/plugins/OvxAttributeName.h"

#include <gtest/gtest.h>

namespace
{
#if OVSTAGE_VERSION_MAJOR == 0 && OVSTAGE_VERSION_MINOR == 4
constexpr bool kStringFirst = true;
#else
constexpr bool kStringFirst = false;
#endif
} // namespace

TEST(AttributeNamePrecedence, NonemptyStringWithZeroToken)
{
    const ovx_string_or_token_t name{ 0, { "position", 8 } };
    EXPECT_FALSE(omni::physx::attributeUsesToken(name));
}

TEST(AttributeNamePrecedence, ConflictingNonzeroTokenFollowsSelectedStage)
{
    const ovx_string_or_token_t name{ 17, { "position", 8 } };
    EXPECT_EQ(omni::physx::attributeUsesToken(name), !kStringFirst);
}

TEST(AttributeNamePrecedence, ExplicitEmptyStringDoesNotFallBackInStage04)
{
    const ovx_string_or_token_t name{ 17, { "", 0 } };
    EXPECT_EQ(omni::physx::attributeUsesToken(name), !kStringFirst);
}

TEST(AttributeNamePrecedence, InvalidTokenDoesNotOverrideString)
{
    const ovx_string_or_token_t name{ OVX_INVALID_TOKEN, { "position", 8 } };
    EXPECT_FALSE(omni::physx::attributeUsesToken(name));
}

TEST(AttributeNamePrecedence, TokenOnlyIgnoresUnusedStringLength)
{
    const ovx_string_or_token_t name{ 17, { nullptr, 99 } };
    EXPECT_TRUE(omni::physx::attributeUsesToken(name));
}

TEST(AttributeNamePrecedence, TokenOnlyZeroAndInvalidRemainDistinct)
{
    const ovx_string_or_token_t zero{ 0, { nullptr, 0 } };
    const ovx_string_or_token_t invalid{ OVX_INVALID_TOKEN, { nullptr, 0 } };
    EXPECT_EQ(omni::physx::attributeUsesToken(zero), kStringFirst);
    EXPECT_EQ(omni::physx::attributeUsesToken(invalid), kStringFirst);
}
