// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Tests for custom error callback functionality

#include <omni/physics/umbrella/IPhysics.h>
#include <doctest/doctest.h>
#include <vector>
#include <string>

using namespace omni::physics;

// Custom error callback for testing
class TestErrorCallback : public ErrorCallback
{
public:
    struct ErrorRecord
    {
        ErrorCode::Enum code;
        std::string message;
        std::string file;
        int line;
    };

    std::vector<ErrorRecord> errors;

    void reportError(ErrorCode::Enum code, const char* message, const char* file, int line) override
    {
        ErrorRecord record;
        record.code = code;
        record.message = message ? message : "";
        record.file = file ? file : "";
        record.line = line;
        errors.push_back(record);
    }

    void clear()
    {
        errors.clear();
    }

    size_t getErrorCount() const { return errors.size(); }
    size_t getWarningCount() const
    {
        size_t count = 0;
        for (const auto& err : errors)
        {
            if (err.code == ErrorCode::eWARNING)
                count++;
        }
        return count;
    }
    size_t getInfoCount() const
    {
        size_t count = 0;
        for (const auto& err : errors)
        {
            if (err.code == ErrorCode::eINFO)
                count++;
        }
        return count;
    }
};

//=============================================================================
// TEST: Error Callback
//=============================================================================
TEST_CASE("Error Callback")
{
    TestErrorCallback callback;
    IPhysics* physics = getPhysicsInterface();
    REQUIRE(physics);

    // Set the custom callback for this test
    physics->setErrorCallback(&callback);
    REQUIRE(physics->getErrorCallback() == &callback);

    SUBCASE("Capture errors from invalid operations")
    {
        callback.clear();

        // Try to get an invalid simulation
        const Simulation* sim = physics->getSimulation(kInvalidSimulationId);
        REQUIRE(sim == nullptr);

        // Check that an error was logged
        REQUIRE(callback.getErrorCount() >= 1);
        REQUIRE(callback.errors.back().code == ErrorCode::eERROR);
        REQUIRE(callback.errors.back().message.find("simulation not found") != std::string::npos);

        callback.clear();

        // Try to check if invalid simulation is active
        bool isActive = physics->isSimulationActive(kInvalidSimulationId);
        REQUIRE(isActive == false);

        // Check that an error was logged
        REQUIRE(callback.getErrorCount() >= 1);
        REQUIRE(callback.errors.back().code == ErrorCode::eERROR);
        REQUIRE(callback.errors.back().message.find("simulation not found") != std::string::npos);

        // Reset callback before ending test
        physics->setErrorCallback(nullptr);
    }

    SUBCASE("No errors for valid operations")
    {
        callback.clear();

        // Register a valid simulation
        Simulation simulation;
        SimulationId simId = physics->registerSimulation(simulation, "TestSimulation");
        REQUIRE(simId != kInvalidSimulationId);

        // No errors should have been logged
        REQUIRE(callback.getErrorCount() == 0);

        // Get the simulation
        const Simulation* sim = physics->getSimulation(simId);
        REQUIRE(sim != nullptr);

        // Still no errors
        REQUIRE(callback.getErrorCount() == 0);

        // Clean up
        physics->unregisterSimulation(simId);

        // Reset callback before ending test
        physics->setErrorCallback(nullptr);
    }

    SUBCASE("Multiple error collection")
    {
        callback.clear();

        // Cause multiple errors
        physics->getSimulation(SimulationId(9999));
        physics->getSimulation(SimulationId(8888));
        physics->isSimulationActive(SimulationId(7777));

        // Check that all errors were collected
        REQUIRE(callback.getErrorCount() >= 3);

        // Verify error messages contain expected text
        for (const auto& err : callback.errors)
        {
            REQUIRE(err.code == ErrorCode::eERROR);
            REQUIRE(err.message.find("simulation not found") != std::string::npos);
        }

        // Reset callback before ending test
        physics->setErrorCallback(nullptr);
    }
}
