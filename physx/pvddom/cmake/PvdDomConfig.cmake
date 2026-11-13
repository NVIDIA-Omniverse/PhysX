## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

@PACKAGE_INIT@

include("${CMAKE_CURRENT_LIST_DIR}/PvdDomRuntimeTargets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/PvdDomTargets.cmake")
check_required_components(PvdDom)
