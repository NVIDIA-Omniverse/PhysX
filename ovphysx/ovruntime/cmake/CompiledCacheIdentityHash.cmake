# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# Build step of CompiledCacheIdentity.cmake: hash the listed inputs into the header.
file(READ "${IDENTITY}" _cache_identity)
string(APPEND _cache_identity "config=${CONFIG};")
# UTF-8: a checkout path may contain non-ASCII characters.
file(STRINGS "${FILES}" _cache_identity_files ENCODING UTF-8)
foreach(_cache_file IN LISTS _cache_identity_files)
    if(NOT IS_DIRECTORY "${_cache_file}")
        file(SHA256 "${_cache_file}" _cache_file_hash)
        string(APPEND _cache_identity "${_cache_file_hash};")
    endif()
endforeach()
string(SHA256 _cache_build_hash "${_cache_identity}")
file(WRITE "${OUTPUT}" "#pragma once\n#define OVRUNTIME_COMPILED_CACHE_BUILD \"${_cache_build_hash}\"\n")
