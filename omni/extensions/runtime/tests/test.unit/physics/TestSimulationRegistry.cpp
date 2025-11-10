// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include "PhysicsTools.h"

#include <omni/physics/simulation/IPhysics.h>
#include <string>

using namespace omni::physics;
using std::string;

//-----------------------------------------------------------------------------
// Simulation Registry Tests
TEST_CASE("Simulation Registry Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    ScopedOmniPhysicsActivation scopedOmniPhysicsActivation;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysics* physics = physicsTests.getApp()->getFramework()->acquireInterface<IPhysics>();
    REQUIRE(physics);

    SUBCASE("Create a new simulation")
    {
        Simulation simulation;
        const char* simulationName = "TestSimulation";

        SimulationId simulationId = physics->registerSimulation(simulation, simulationName);
        REQUIRE(simulationId != kInvalidSimulationId);
        
        // Test getting simulation name
        const char* retrievedName = physics->getSimulationName(simulationId);
        REQUIRE(strcmp(retrievedName, simulationName) == 0);
        
        physics->unregisterSimulation(simulationId);
    }

    SUBCASE("Register and unregister simulation")
    {
        Simulation simulation;
        const char* simulationName = "TestSimulation";
        
        // Test registration
        SimulationId simulationId = physics->registerSimulation(simulation, simulationName);
        REQUIRE(simulationId != kInvalidSimulationId);
        
        // Test getting simulation by ID
        const Simulation* retrievedSim = physics->getSimulation(simulationId);
        REQUIRE(retrievedSim);
        
        // Test getting simulation name
        const char* retrievedName = physics->getSimulationName(simulationId);
        REQUIRE(strcmp(retrievedName, simulationName) == 0);
        
        // Test unregistration
        physics->unregisterSimulation(simulationId);
        retrievedSim = physics->getSimulation(simulationId);
        REQUIRE(retrievedSim == nullptr);
    }

    SUBCASE("Multiple simulations management")
    {
        Simulation sim1, sim2, sim3;
        const char* sim1Name = "Simulation1";
        const char* sim2Name = "Simulation2";
        const char* sim3Name = "Simulation3";
        
        // Register multiple simulations
        SimulationId id1 = physics->registerSimulation(sim1, sim1Name);
        SimulationId id2 = physics->registerSimulation(sim2, sim2Name);
        SimulationId id3 = physics->registerSimulation(sim3, sim3Name);
        
        REQUIRE(id1 != kInvalidSimulationId);
        REQUIRE(id2 != kInvalidSimulationId);
        REQUIRE(id3 != kInvalidSimulationId);
        
        // Test getNumSimulations
        size_t numSims = physics->getNumSimulations();
        REQUIRE(numSims == 3);
        
        // Test getSimulationIds
        SimulationId* simIds = new SimulationId[numSims];
        size_t retrievedCount = physics->getSimulationIds(simIds, numSims);
        REQUIRE(retrievedCount == numSims);
        
        // Verify all IDs are present and names are correct
        bool foundId1 = false, foundId2 = false, foundId3 = false;
        for(size_t i = 0; i < numSims; i++) {
            if(simIds[i] == id1) {
                foundId1 = true;
                REQUIRE(strcmp(physics->getSimulationName(simIds[i]), sim1Name) == 0);
            }
            if(simIds[i] == id2) {
                foundId2 = true;
                REQUIRE(strcmp(physics->getSimulationName(simIds[i]), sim2Name) == 0);
            }
            if(simIds[i] == id3) {
                foundId3 = true;
                REQUIRE(strcmp(physics->getSimulationName(simIds[i]), sim3Name) == 0);
            }
        }
        REQUIRE(foundId1);
        REQUIRE(foundId2);
        REQUIRE(foundId3);
                
        // Cleanup
        physics->unregisterSimulation(id1);
        physics->unregisterSimulation(id2);
        physics->unregisterSimulation(id3);

        delete[] simIds;
    }

    SUBCASE("Simulation activation")
    {
        Simulation simulation;
        const char* simulationName = "TestSimulation";
        
        // Register simulation
        SimulationId simulationId = physics->registerSimulation(simulation, simulationName);
        REQUIRE(simulationId != kInvalidSimulationId);
        
        // Test initial state (should be active by default)
        REQUIRE(physics->isSimulationActive(simulationId));
                
        // Test deactivation
        physics->deactivateSimulation(simulationId);
        REQUIRE_FALSE(physics->isSimulationActive(simulationId));
        
        // Test activation
        physics->activateSimulation(simulationId);
        REQUIRE(physics->isSimulationActive(simulationId));
        
        // Cleanup
        physics->unregisterSimulation(simulationId);
    }

    SUBCASE("Invalid simulation operations")
    {        
        // Test getting simulation with invalid ID
        const Simulation* invalidSim = physics->getSimulation(kInvalidSimulationId);
        REQUIRE(invalidSim == nullptr);
        
        // Test getting name with invalid ID
        const char* invalidName = physics->getSimulationName(kInvalidSimulationId);
        REQUIRE(invalidName == nullptr);
        
        // Test activation state with invalid ID
        REQUIRE_FALSE(physics->isSimulationActive(kInvalidSimulationId));
        
        // Test unregistering invalid ID (should not crash)
        physics->unregisterSimulation(kInvalidSimulationId);
        
        // Test activating invalid ID (should not crash)
        physics->activateSimulation(kInvalidSimulationId);
        
        // Test deactivating invalid ID (should not crash)
        physics->deactivateSimulation(kInvalidSimulationId);
    }

    SUBCASE("Simulation buffer handling")
    {
        const size_t numSimsToCreate = 5;
        std::vector<Simulation> sims(numSimsToCreate);
        std::vector<SimulationId> ids(numSimsToCreate);
        std::vector<string> names(numSimsToCreate);
        
        // Register multiple simulations
        for(size_t i = 0; i < numSimsToCreate; i++) {
            names[i] = "Simulation" + std::to_string(i);
            ids[i] = physics->registerSimulation(sims[i], names[i].c_str());
            REQUIRE(ids[i] != kInvalidSimulationId);
        }
        
        // Test getting simulation IDs with different buffer sizes
        {
            // Test with smaller buffer
            SimulationId smallBuffer[2];
            size_t retrieved = physics->getSimulationIds(&smallBuffer[0], 2);
            REQUIRE(retrieved == 2);
            
            // Test with exact buffer size
            SimulationId* exactBuffer = new SimulationId[numSimsToCreate];
            retrieved = physics->getSimulationIds(exactBuffer, numSimsToCreate);
            REQUIRE(retrieved == numSimsToCreate);
            delete[] exactBuffer;
            
            // Test with larger buffer
            SimulationId* largeBuffer = new SimulationId[numSimsToCreate + 2];
            retrieved = physics->getSimulationIds(largeBuffer, numSimsToCreate + 2);
            REQUIRE(retrieved == numSimsToCreate);
            delete[] largeBuffer;
        }
        
        // Cleanup
        for(auto id : ids) {
            physics->unregisterSimulation(id);
        }
    }    
}
