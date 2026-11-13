# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-DOCS-001
# @covers AC-2

include_guard(GLOBAL)

# The source headers installed by the SDK also provide the link targets for
# source-only documentation validation. Generated/dependency headers still
# belong to the native installation; this does not invent placeholder files.
function(ovphysx_public_header_sources PROJECT_ROOT OUT_DIRS OUT_PATTERNS)
    set(${OUT_DIRS} "${PROJECT_ROOT}/include/" "${PROJECT_ROOT}/companion/include/" PARENT_SCOPE)
    set(${OUT_PATTERNS} PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.inc" PARENT_SCOPE)
endfunction()
