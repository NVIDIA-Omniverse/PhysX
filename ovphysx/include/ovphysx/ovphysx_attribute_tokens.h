// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @cond OVPHYSX_PLC_TRACE */
/**
 * @implements REQ-CAPI-ATTRIBUTE-NAMES-001
 * @covers AC-1 AC-2 AC-3
 */
/** @endcond */
#ifndef OVPHYSX_ATTRIBUTE_TOKENS_H
#define OVPHYSX_ATTRIBUTE_TOKENS_H

#include <ovx/path_dictionary/path_dictionary.h>

/** Intern a portable attribute name in the caller's dictionary.
 *
 * Pass the .string member of an OVPHYSX_NAME_* object. The result belongs to
 * this dictionary and becomes invalid when its owner destroys it. No token is
 * cached. On failure *out_token is invalid; release a nonempty result.error
 * with path_dictionary_release_error(dictionary, result.error).
 * Invalid arguments return OVX_API_ERROR with an empty error string.
 */
static inline ovx_api_result_t ovphysx_intern_attribute_name(
    path_dictionary_instance_t* dictionary, ovx_string_t name, ovx_token_t* out_token)
{
    ovx_api_result_t result = { OVX_API_ERROR, { 0, 0 } };
    ovx_token_t token = OVX_INVALID_TOKEN;
    if (out_token)
        *out_token = OVX_INVALID_TOKEN;
    if (!dictionary || !dictionary->vtable || !dictionary->vtable->create_tokens_from_strings ||
        !name.ptr || !name.length || !out_token)
        return result;
    result = path_dictionary_create_tokens_from_strings(dictionary, &name, 1, &token);
    if (result.status == OVX_API_SUCCESS)
        *out_token = token;
    return result;
}

#endif /* OVPHYSX_ATTRIBUTE_TOKENS_H */
