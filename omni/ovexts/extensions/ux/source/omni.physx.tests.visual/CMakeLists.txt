# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

set(EXT_NAME "omni.physx.tests.visual")
set(EXT_DIR "${OVEXTS_EXTS_PHYSICS_DIR}/${EXT_NAME}")
set(EXT_PYTHON_NS "omni/physxtestsvisual")

# Prebuild
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/config"  "${EXT_DIR}/config")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/docs"    "${EXT_DIR}/docs")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/python"  "${EXT_DIR}/${EXT_PYTHON_NS}")
ovexts_prebuild_link("${OVEXTS_ROOT_DIR}/data/usd/tests/Physics/Visual"        "${EXT_DIR}/data")
ovexts_prebuild_link("${OVEXTS_ROOT_DIR}/data/icons/omni.physx.demos"          "${EXT_DIR}/icons")

# Test runner scripts
ovexts_define_physics_test_experience(${EXT_NAME} SUITE "visual")
ovexts_define_etm_test_experience(${EXT_NAME})
