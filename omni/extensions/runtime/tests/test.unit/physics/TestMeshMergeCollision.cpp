// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <chrono>
#include <thread>

#include "../common/TestHelpers.h"

#include <omni/physx/IPhysxCooking.h>
#include <private/omni/physx/IPhysxTests.h>

#include "PhysicsTools.h"

using namespace pxr;
using namespace omni::physx;
using namespace ::physx;

//-----------------------------------------------------------------------------
// Init test
TEST_CASE("Mesh Merge Collision",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath xformPath = SdfPath("/World/Xform");
    UsdGeomXform xform = UsdGeomXform::Define(stage, xformPath);
    UsdPhysicsCollisionAPI::Apply(xform.GetPrim());
    PhysxSchemaPhysxMeshMergeCollisionAPI meshMergeAPI = PhysxSchemaPhysxMeshMergeCollisionAPI::Apply(xform.GetPrim());
    meshMergeAPI.GetCollisionMeshesCollectionAPI().GetIncludesRel().AddTarget(xform.GetPrim().GetPrimPath());
    
    const SdfPath meshPath0 = SdfPath("/World/Xform/Mesh0");
    createMeshBox(stage, meshPath0);
    const SdfPath meshPath1 = SdfPath("/World/Xform/Mesh1");
    createMeshBox(stage, meshPath1);

    physxSim->attachStage(stageId);

    SECTION("Parsing")
    {
        PxRigidStatic* actor = getPhysxBaseDerivedFromPathChecked<PxRigidStatic>(xformPath, ePTActor);
        REQUIRE(actor != nullptr);
        CHECK(actor->getNbShapes() == 1);

        PxShape* shape = getPhysxBaseDerivedFromPathChecked<PxShape>(xformPath, ePTShape);
        REQUIRE(shape != nullptr);
        CHECK(shape->getGeometry().getType() == PxGeometryType::eTRIANGLEMESH);
    }

    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

