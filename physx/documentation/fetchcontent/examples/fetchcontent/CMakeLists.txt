# Required minimum CMake version
#
# PhysX requires CMake 3.21+. FetchContent_MakeAvailable is available since 3.14.
cmake_minimum_required(VERSION 3.21)

# Define project name and languages
project(hello-physx-cmake LANGUAGES CXX)

#########################################################
# DEPENDENCIES SETUP
#########################################################

# Include required CMake modules
include(FetchContent)       # Module for downloading and configuring external dependencies
include(GNUInstallDirs)     # Sets standard installation directories (bin, lib, etc.)

# <DEBUG>
# To use a local PhysX repository instead of downloading, set the path to the repo root
# (the directory that contains the "physx" subdirectory):
# if(WIN32)
#   set(FETCHCONTENT_SOURCE_DIR_PHYSX_LIB "C:/path/to/physics-repo")
# else()
#   set(FETCHCONTENT_SOURCE_DIR_PHYSX_LIB "/path/to/physics-repo")
# endif()
# </DEBUG>

# Display download progress in the terminal (useful for larger dependencies)
set(FETCHCONTENT_QUIET OFF)

#########################################################
# PHYSX CONFIGURATION - Choose your build preset
#########################################################

# OPTION 1: Use a PhysX preset for common configurations (RECOMMENDED)
# Presets provide well-tested combinations of build options:
#   - vc17win64: Windows VS2022, GPU enabled, shared libraries  
#   - vc17win64-cpu-only: Windows VS2022, CPU only, shared libraries
#   - linux-gcc: Linux GCC, GPU enabled, static libraries
#   - linux-gcc-cpu-only: Linux GCC, CPU only, static libraries  
#   - linux-clang: Linux Clang, GPU enabled, static libraries
#   - linux-aarch64-gcc: ARM64 Linux GCC, GPU enabled, static libraries

# Only set default presets if not already specified (allows CI override)
if(NOT DEFINED PHYSX_PRESET OR PHYSX_PRESET STREQUAL "")
    if(WIN32)
        # Windows: Use VS2022 with GPU support (most common)
        set(PHYSX_PRESET "vc17win64" CACHE STRING "PhysX build preset")
    elseif(UNIX AND NOT APPLE)
        # Linux: Use GCC with GPU support (most common)
        set(PHYSX_PRESET "linux-gcc" CACHE STRING "PhysX build preset") 
    endif()
    # Note: macOS is not currently supported via public presets
endif()

# OPTION 2: Manual configuration (advanced users only)
# You can override specific options instead of using a preset:
# set(PX_GENERATE_GPU_PROJECTS OFF CACHE BOOL "" FORCE)      # Disable GPU
# set(PX_GENERATE_STATIC_LIBRARIES ON CACHE BOOL "" FORCE)   # Use static libs
# set(PX_BUILDSNIPPETS ON CACHE BOOL "" FORCE)               # Build 50+ examples (NOT recommended: +8min build time + OpenGL deps)

#########################################################
# FETCHCONTENT DECLARATION  
#########################################################

# Declare the PhysX dependency using FetchContent
# - physx_lib: Name used to refer to this dependency in our project
# - GIT_REPOSITORY: Source repository URL
# - GIT_TAG: Specific branch, tag, or commit to use
# - SOURCE_SUBDIR: Subdirectory in the repository containing the main CMakeLists.txt
FetchContent_Declare(
    physx_lib
    GIT_REPOSITORY https://github.com/NVIDIA-Omniverse/PhysX.git
    GIT_TAG        origin/main  # Or pin to a specific release tag/commit
    SOURCE_SUBDIR  physx
)

# Note: We intentionally avoid requiring very new CMake features (e.g.
# CMAKE_POLICY_VERSION_MINIMUM) so the example configures with CMake 3.21+.
# You can also set additional cmake options here for PhysX before the MakeAvailable call
# where PhysX is configured.
# set(PX_GENERATE_STATIC_LIBRARIES ON)
# set(PX_GENERATE_GPU_STATIC_LIBRARIES ON)

# Make PhysX available to our project
# This downloads, configures, and makes PhysX targets available
FetchContent_MakeAvailable(physx_lib)

# Log PhysX directories for debugging purposes
message(STATUS "PhysX source directory: ${physx_lib_SOURCE_DIR}")
message(STATUS "PhysX binary directory: ${physx_lib_BINARY_DIR}")

#########################################################
# APPLICATION SETUP
#########################################################

# Create our application executable
add_executable(hello-physx-cmake main.cpp)

# Link our executable with PhysX - the modern CMake way
# This automatically includes all necessary dependencies:
# - All PhysX libraries
# - Required include directories  
# - Compiler definitions
# - Runtime library settings (/MD on Windows by default)
# - Other properties from the PhysX targets
target_link_libraries(hello-physx-cmake PRIVATE physx_lib)

#########################################################
# INSTALLATION OF EXECUTABLE AND DEPENDENCIES
#########################################################

# Configure runtime path for the executable
# This ensures the executable can find its shared libraries at runtime
set_target_properties(hello-physx-cmake PROPERTIES
  # $ORIGIN refers to the executable's location (Linux specific)
  # This sets the runtime search path to look in ../lib relative to the executable
  INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"

  # Don't use install RPATH during the build phase
  BUILD_WITH_INSTALL_RPATH OFF

  # Remove build directory paths from the binary, activate this if you just want
  # to install your app and not test it from the build folder
  #  SKIP_BUILD_RPATH        ON
)

if (WIN32) # Optional
  # After building hello-physx-cmake, copy all its runtime DLLs next to the EXE in the build folder.
  # This is useful for windows (since there's no rpath) to find the DLLs at runtime
  add_custom_command(TARGET hello-physx-cmake
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_RUNTIME_DLLS:hello-physx-cmake>
      $<TARGET_FILE_DIR:hello-physx-cmake>
    # ensure the generator expression list is expanded
    COMMAND_EXPAND_LISTS
  )
endif()

# Optional: Basic installation of the executable 
# Note: For advanced dependency installation, use cmake --install with --prefix
install(TARGETS hello-physx-cmake
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"   # For executables
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"   # For shared libraries  
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"   # For static libraries
)
