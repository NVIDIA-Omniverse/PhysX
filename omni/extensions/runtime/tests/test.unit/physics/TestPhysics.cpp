// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxTests.h>
#include <omni/physx/IPhysxSimulation.h>

#include "PhysicsTools.h"


//-----------------------------------------------------------------------------
// Init test
TEST_CASE("Init test",
          "[omniphysics]"
          "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    SECTION("Module init test")
    {
        PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

        omni::physx::IPhysx* physx = physicsTests.acquirePhysxInterface();
        CHECK(physx);

        omni::physx::IPhysxUnitTests* physxUt = physicsTests.acquirePhysxUnitTestInterface();
        CHECK(physxUt);

        omni::physx::IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
        CHECK(physxSim);
    }
}
