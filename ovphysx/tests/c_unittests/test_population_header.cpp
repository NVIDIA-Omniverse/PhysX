// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-POPULATION-001
 * @covers AC-1 AC-2
 */

// Smoke gate for the shipped population header: it compiles as C++17 in an SDK consumer's
// translation unit (this executable), lives in namespace ovphysx::population, and enforces the
// contract rules that need no PhysX: applicability, required co-applied APIs, resolved defaults,
// ancestor derivation. What the builder writes is verified against PhysX by the runtime's
// generated doctest suite, not here.

#include <gtest/gtest.h>

#include "ovphysx/population/Population.hpp"

#include <ovstage/ovstage.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace
{

class PopulationHeader : public ::testing::Test
{
protected:
    void SetUp() override
    {
        ovstage_instance_desc_t desc{};
        desc.name = "population-header-smoke";
        ASSERT_EQ(ovstage_create_instance(&desc, &mInstance), OVSTAGE_OK);
        ASSERT_NE(mInstance, nullptr);
    }

    void TearDown() override
    {
        if (mInstance)
            ovstage_destroy_instance(mInstance);
    }

    ovstage_instance_t* mInstance = nullptr;
};

} // namespace

TEST_F(PopulationHeader, ResolvedDefaultsFollowStageUnits)
{
    using namespace ovphysx::population;
    Units cm{};
    cm.metersPerUnit = 0.01f;
    const PhysicsSceneDefaults z = PhysicsSceneDefaults::resolved(cm);
    EXPECT_FLOAT_EQ(z.gravityMagnitude, 981.0f); // 9.81 / metersPerUnit
    EXPECT_FLOAT_EQ(z.gravityDirection[2], -1.0f);
    Units y{};
    y.upAxis = 'Y';
    EXPECT_FLOAT_EQ(PhysicsSceneDefaults::resolved(y).gravityDirection[1], -1.0f);
}

TEST_F(PopulationHeader, AncestorPathsAreDerivedOnce)
{
    const std::vector<std::string> parents =
        ovphysx::population::ancestorPaths({ "/World/Env/Box_0", "/World/Env/Box_1", "/Other" });
    EXPECT_EQ(parents, (std::vector<std::string>{ "/World", "/World/Env" }));
}

TEST_F(PopulationHeader, RejectsApiOnTypeItDoesNotApplyTo)
{
    ovphysx::population::PrimBatch batch(mInstance, { "/World/Scene" });
    batch.definePhysicsScene();
    EXPECT_THROW(batch.applyPhysicsRigidBodyAPI(), std::invalid_argument);
}

TEST_F(PopulationHeader, RejectsWriteMissingRequiredApi)
{
    ovphysx::population::PrimBatch batch(mInstance, { "/World/Box" });
    batch.defineCube();
    batch.applyPhysxCollisionAPI(); // requires PhysicsCollisionAPI on the same prim
    EXPECT_THROW(batch.write(1), std::invalid_argument);
}

TEST_F(PopulationHeader, RequiresTheSameDriveInstance)
{
    ovphysx::population::PrimBatch batch(mInstance, { "/World/Joint" });
    batch.definePhysicsJoint();
    batch.applyPhysicsDriveAPI("rotX");
    batch.applyPhysxDrivePerformanceEnvelopeAPI("rotY"); // envelope extends the same-instance drive only
    EXPECT_THROW(batch.write(1), std::invalid_argument);
}

TEST_F(PopulationHeader, WritesAndSealsARigidCube)
{
    using namespace ovphysx::population;
    writeStageUnits(mInstance, 1, Units{});
    PrimBatch batch(mInstance, { "/World/Box_0", "/World/Box_1" });
    CubeArgs cube;
    cube.size = std::vector<double>{ 2.0, 2.0 };
    batch.defineCube(cube);
    batch.applyPhysicsRigidBodyAPI();
    batch.applyPhysicsCollisionAPI();
    EXPECT_NO_THROW(batch.write(1));
}
