# Determine platform-specific directory names for Carbonite and PhysX SDKs.
#
# CARB_PLATFORM_DIR  — Carbonite SDK lib path suffix (e.g. "linux-x86_64")
#   Usage: ${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}/<config>
#
# PHYSX_PLATFORM_BIN — PhysX SDK bin path suffix (e.g. "linux.x86_64")
#   Usage: ${PHYSX_SDK_DIR}/bin/${PHYSX_PLATFORM_BIN}/<config>

if(WIN32)
    set(CARB_PLATFORM_DIR "windows-x86_64")
    set(PHYSX_PLATFORM_BIN "win.x86_64.vc142.md")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(CARB_PLATFORM_DIR "linux-aarch64")
    set(PHYSX_PLATFORM_BIN "linux.aarch64")
else()
    set(CARB_PLATFORM_DIR "linux-x86_64")
    set(PHYSX_PLATFORM_BIN "linux.x86_64")
endif()
