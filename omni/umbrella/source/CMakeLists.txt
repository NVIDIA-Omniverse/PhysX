# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

set(UMBRELLA_LIBTYPE SHARED)

# Collect header files
set(UMBRELLA_HEADERS
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/IPhysics.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/IPhysicsBenchmark.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/IPhysicsInteraction.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/IPhysicsSceneQuery.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/IPhysicsSimulation.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/IPhysicsStageUpdate.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/simulator/Simulator.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/simulator/Simulation.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/simulator/Benchmark.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/simulator/Interaction.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/simulator/InteractionTools.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/simulator/SceneQuery.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/simulator/StageUpdate.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/simulator/ContactEvent.h
)
source_group(include FILES ${UMBRELLA_HEADERS})

# Collect source files
set(UMBRELLA_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/OmniPhysics.h
    ${CMAKE_CURRENT_SOURCE_DIR}/OmniPhysics.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Physics.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Simulation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Benchmark.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Interaction.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SceneQuery.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/StageUpdate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SimulationRegistry.h
    ${CMAKE_CURRENT_SOURCE_DIR}/SimulationRegistry.cpp
)
source_group(source FILES ${UMBRELLA_SOURCE})

# Create the library
add_library(PhysicsUmbrella ${UMBRELLA_LIBTYPE}
    ${UMBRELLA_HEADERS}
    ${UMBRELLA_SOURCE}
)

set_property(TARGET PhysicsUmbrella PROPERTY FOLDER "Umbrella")

# Include directories
target_include_directories(PhysicsUmbrella
    PUBLIC ${UMBRELLA_ROOT_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Add Carbonite include if available
if(DEFINED CARB_SDK_DIR)
    target_include_directories(PhysicsUmbrella
        SYSTEM PRIVATE ${CARB_SDK_DIR}/include
    )
endif()

# Compile definitions
target_compile_definitions(PhysicsUmbrella
    PRIVATE PHYSICS_UMBRELLA_EXPORTS
)

# Link directories and libraries
if(DEFINED CARB_SDK_DIR)
    # Carbonite SDK layout: _build/<platform>/<config>/carb.lib
    target_link_directories(PhysicsUmbrella
        PRIVATE ${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}/$<LOWER_CASE:$<CONFIG>>
    )

    target_link_libraries(PhysicsUmbrella
        PRIVATE carb
    )
endif()

# Set output name (default lib prefix on Linux matches Kit/Carbonite plugin loading)
set_target_properties(PhysicsUmbrella PROPERTIES
    OUTPUT_NAME "PhysicsUmbrella"
)

# Install rules - simulation headers (preserving directory structure)
install(FILES
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/IPhysics.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/IPhysicsBenchmark.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/IPhysicsInteraction.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/IPhysicsSceneQuery.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/IPhysicsSimulation.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/IPhysicsStageUpdate.h
    DESTINATION include/omni/physics/simulation
)

install(FILES
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/simulator/Simulator.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/simulator/Simulation.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/simulator/Benchmark.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/simulator/Interaction.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/simulator/InteractionTools.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/simulator/SceneQuery.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/simulator/StageUpdate.h
    ${UMBRELLA_ROOT_DIR}/include/omni/physics/simulation/simulator/ContactEvent.h
    DESTINATION include/omni/physics/simulation/simulator
)

install(
    TARGETS PhysicsUmbrella
    EXPORT PhysicsUmbrella
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION bin
    ARCHIVE DESTINATION lib
)
