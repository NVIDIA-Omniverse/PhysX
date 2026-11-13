// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CAPI-ATTRIBUTE-NAMES-001
// @covers AC-1 AC-2 AC-3

#include <gtest/gtest.h>
#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_attributes.h>
#include <ovphysx/ovphysx_attribute_tokens.h>
#include <cstring>

namespace
{
struct DictionaryState
{
    ovx_token_t token;
    bool fail;
    unsigned calls;
    unsigned releases;
};

ovx_api_result_t intern(path_dictionary_context_t* context, const ovx_string_t* strings,
                        size_t count, ovx_token_t* tokens)
{
    DictionaryState* state = reinterpret_cast<DictionaryState*>(context);
    ++state->calls;
    EXPECT_EQ(count, 1u);
    EXPECT_EQ(std::string(strings[0].ptr, strings[0].length), OVPHYSX_ATTR_POSITION);
    tokens[0] = state->token; // A failing backend may still touch its output.
    if (state->fail)
        return { OVX_API_ERROR, { "failed", 6 } };
    return { OVX_API_SUCCESS, { nullptr, 0 } };
}

void releaseError(path_dictionary_context_t* context, ovx_string_t error)
{
    DictionaryState* state = reinterpret_cast<DictionaryState*>(context);
    EXPECT_EQ(std::string(error.ptr, error.length), "failed");
    ++state->releases;
}
} // namespace

TEST(AttributeNames, PortableNamesAndDictionaryOwnership)
{
    EXPECT_EQ(OVPHYSX_NAME_POSITION.token, OVX_INVALID_TOKEN);
    EXPECT_EQ(std::string(OVPHYSX_NAME_POSITION.string.ptr, OVPHYSX_NAME_POSITION.string.length),
              OVPHYSX_ATTR_POSITION);
    EXPECT_EQ(OVPHYSX_NAME_DEBUG_LINE_POSITION.token, OVX_INVALID_TOKEN);
    DictionaryState first{ 17, false, 0, 0 };
    DictionaryState second{ 29, false, 0, 0 };
    path_dictionary_vtable_t vtable{};
    vtable.create_tokens_from_strings = intern;
    vtable.release_error = releaseError;
    path_dictionary_instance_t a{ &vtable, reinterpret_cast<path_dictionary_context_t*>(&first) };
    path_dictionary_instance_t b{ &vtable, reinterpret_cast<path_dictionary_context_t*>(&second) };
    ovx_token_t token{0};
    EXPECT_EQ(ovphysx_intern_attribute_name(&a, OVPHYSX_NAME_POSITION.string, &token).status, OVX_API_SUCCESS);
    EXPECT_EQ(token.raw_id, 17u);
    EXPECT_EQ(ovphysx_intern_attribute_name(&b, OVPHYSX_NAME_POSITION.string, &token).status, OVX_API_SUCCESS);
    EXPECT_EQ(token.raw_id, 29u);
    EXPECT_EQ(ovphysx_intern_attribute_name(&a, OVPHYSX_NAME_POSITION.string, &token).status, OVX_API_SUCCESS);
    EXPECT_EQ(token.raw_id, 17u);
    EXPECT_EQ(first.calls, 2u);
    EXPECT_EQ(second.calls, 1u);

    first.fail = true;
    const ovx_api_result_t failure = ovphysx_intern_attribute_name(&a, OVPHYSX_NAME_POSITION.string, &token);
    EXPECT_EQ(failure.status, OVX_API_ERROR);
    EXPECT_EQ(token, OVX_INVALID_TOKEN);
    EXPECT_EQ(first.releases, 0u);
    path_dictionary_release_error(&a, failure.error);
    EXPECT_EQ(first.releases, 1u);
}

TEST(AttributeNames, InvalidInputsClearOutput)
{
    ovx_token_t token{99};
    EXPECT_EQ(ovphysx_intern_attribute_name(nullptr, OVPHYSX_NAME_POSITION.string, &token).status, OVX_API_ERROR);
    EXPECT_EQ(token, OVX_INVALID_TOKEN);
    path_dictionary_instance_t empty{};
    EXPECT_EQ(ovphysx_intern_attribute_name(&empty, OVPHYSX_NAME_POSITION.string, nullptr).status, OVX_API_ERROR);
}
