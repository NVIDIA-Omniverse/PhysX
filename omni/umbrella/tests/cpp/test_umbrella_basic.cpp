// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//
// Basic C++ test for umbrella library

#include <omni/physics/simulation/IPhysics.h>
#include "SimulationRegistry.h"
#include <iostream>
#include <cassert>

using namespace omni::physics;

void test_register_simulation()
{
    std::cout << "\nTest: Register simulation..." << std::endl;

    SimulationRegistry registry;

    // Create a simple simulation
    Simulation simulation;
    const char* name = "TestSimulation";

    // Register it
    SimulationId simId = registry.registerSimulation(simulation, name);
    assert(simId != kInvalidSimulationId && "Should get valid simulation ID");

    std::cout << "  Registered simulation with ID: " << simId.id << std::endl;

    // Verify we can get it back
    const char* retrievedName = registry.getSimulationName(simId);
    assert(retrievedName != nullptr && "Should get simulation name");
    assert(std::string(retrievedName) == name && "Name should match");

    std::cout << "  SUCCESS: Simulation name matches: " << retrievedName << std::endl;

    // Clean up
    registry.unregisterSimulation(simId);
    std::cout << "  SUCCESS: Unregistered simulation" << std::endl;
}

void test_simulation_count()
{
    std::cout << "\nTest: Simulation count..." << std::endl;

    SimulationRegistry registry;

    size_t initialCount = registry.getNumSimulations();
    std::cout << "  Initial simulation count: " << initialCount << std::endl;

    // Register a simulation
    Simulation simulation;
    SimulationId simId = registry.registerSimulation(simulation, "CountTest");

    size_t newCount = registry.getNumSimulations();
    assert(newCount == initialCount + 1 && "Count should increase by 1");
    std::cout << "  After register: " << newCount << std::endl;

    // Unregister
    registry.unregisterSimulation(simId);

    size_t finalCount = registry.getNumSimulations();
    assert(finalCount == initialCount && "Count should return to initial");
    std::cout << "  SUCCESS: After unregister: " << finalCount << std::endl;
}

void test_simulation_activation()
{
    std::cout << "\nTest: Simulation activation..." << std::endl;

    SimulationRegistry registry;

    // Register a simulation (starts active)
    Simulation simulation;
    SimulationId simId = registry.registerSimulation(simulation, "ActivationTest");

    bool isActive = registry.isSimulationActive(simId);
    assert(isActive && "Simulation should start active");
    std::cout << "  Simulation starts active: " << isActive << std::endl;

    // Deactivate
    registry.deactivateSimulation(simId);
    isActive = registry.isSimulationActive(simId);
    assert(!isActive && "Simulation should be inactive after deactivate");
    std::cout << "  After deactivate: " << isActive << std::endl;

    // Reactivate
    registry.activateSimulation(simId);
    isActive = registry.isSimulationActive(simId);
    assert(isActive && "Simulation should be active after activate");
    std::cout << "  SUCCESS: After activate: " << isActive << std::endl;

    // Clean up
    registry.unregisterSimulation(simId);
}

void test_types()
{
    std::cout << "\nTest: Basic types..." << std::endl;

    // Test carb::Float3
    carb::Float3 vec3{1.0f, 2.0f, 3.0f};
    assert(vec3.x == 1.0f && vec3.y == 2.0f && vec3.z == 3.0f);
    (void)vec3;
    std::cout << "  carb::Float3 works" << std::endl;

    // Test carb::Float4
    carb::Float4 vec4{1.0f, 2.0f, 3.0f, 4.0f};
    assert(vec4.x == 1.0f && vec4.y == 2.0f && vec4.z == 3.0f && vec4.w == 4.0f);
    (void)vec4;
    std::cout << "  carb::Float4 works" << std::endl;

    // Test SimulationId
    SimulationId id1(42);
    SimulationId id2(42);
    SimulationId id3(99);
    assert(id1 == id2 && "SimulationIds with same value should be equal");
    assert(id1 != id3 && "SimulationIds with different values should not be equal");
    std::cout << "  SUCCESS: SimulationId comparison works" << std::endl;
}

int main()
{
    std::cout << "============================================================" << std::endl;
    std::cout << "Physics Umbrella Library - C++ Basic Tests" << std::endl;
    std::cout << "============================================================" << std::endl;

    try
    {
        test_types();
        test_register_simulation();
        test_simulation_count();
        test_simulation_activation();

        std::cout << "\n============================================================" << std::endl;
        std::cout << "All tests PASSED" << std::endl;
        std::cout << "============================================================" << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "\nTest FAILED with exception: " << e.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cerr << "\nTest FAILED with unknown exception" << std::endl;
        return 1;
    }
}
