// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include <common/utilities/PhysXErrorCallback.h>
#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxTests.h>
#include <omni/physx/IPhysxSimulation.h>

#include "PhysicsTools.h"


TEST_CASE("PhysX abort without an error event stream",
          "[omniphysics]"
          "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    CarbPhysXErrorCallback errorCallback;
    CHECK_NOTHROW(errorCallback.reportError(
        physx::PxErrorCode::eABORT, "Test abort without an event stream", __FILE__, __LINE__));
}


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
