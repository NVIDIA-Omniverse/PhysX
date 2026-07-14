# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

set(EXT_NAME "omni.physx.demos")
set(EXT_DIR "${OVEXTS_EXTS_PHYSICS_DIR}/${EXT_NAME}")
set(EXT_PYTHON_NS "omni/physxdemos")

# Prebuild
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/config"  "${EXT_DIR}/config")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/docs"    "${EXT_DIR}/docs")
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/python"  "${EXT_DIR}/${EXT_PYTHON_NS}")
ovexts_prebuild_link("${OVEXTS_ROOT_DIR}/data/icons/omni.physx.demos"          "${EXT_DIR}/icons")
ovexts_prebuild_link("${OVEXTS_ROOT_DIR}/data/usd/tests/Physics/Demos"         "${EXT_DIR}/data")
ovexts_prebuild_link("${OVEXTS_ROOT_DIR}/data/audio/physics/demo"              "${EXT_DIR}/audio")

# Test runner scripts
ovexts_define_physics_test_experience(${EXT_NAME}
    DIR "extsPhysicsRepo"
    SUITE "demos"
    TEST_ARGS "--" "--/exts/omni.kit.test/excludeTests/0='*PhysxXAssetDemosTest*'"
)
ovexts_define_physics_test_experience(${EXT_NAME}
    DIR "extsPhysicsRepo"
    SUITE "fsddemos"
    SUFFIX ".fsdon-fabric"
    TEST_ARGS "--"
        "--/exts/omni.kit.test/runTestsFilter='*test_physics_visual_demos*'"
        "--/exts/omni.kit.test/excludeTests/0='*PhysxXAssetDemosTest*'"
        "--/app/useFabricSceneDelegate=true"
        "--/physics/fabricEnabled=true"
        "--enable omni.physx.fabric"
)
# WAR: fsdoff-fabric tests are currently broken — not a physics bug. With FSD off,
# fabric should still drive the render through its own update path; investigation is
# pending on the rendering side. Re-enable once that's resolved.
# ovexts_define_physics_test_experience(${EXT_NAME}
#     DIR "extsPhysicsRepo"
#     SUITE "fsddemos"
#     SUFFIX ".fsdoff-fabric"
#     TEST_ARGS "--"
#         "--/exts/omni.kit.test/runTestsFilter='*test_physics_visual_demos*'"
#         "--/exts/omni.kit.test/excludeTests/0='*PhysxXAssetDemosTest*'"
#         "--/app/useFabricSceneDelegate=false"
#         "--/physics/fabricEnabled=true"
#         "--enable omni.physx.fabric"
# )
ovexts_define_physics_test_experience(${EXT_NAME}
    DIR "extsPhysicsRepo"
    SUITE "assetdemos"
    SUFFIX ".s3"
    TEST_ARGS "--"
        "--/physics/demoDevelopmentMode=0"
        "--/exts/omni.kit.test/runTestsFilter='*PhysxXAssetDemosTest*'"
)
ovexts_define_etm_test_experience(${EXT_NAME})
