// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#include "UsdPCH.h"


#include "../../common/TestHelpers.h"
#include "../../framework/BmBenchmark.h"
#include "../../framework/BmGlobals.h"
#include "../../../common/PhysicsChangeTemplate.h"

#include <iostream>
#include <string>

#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>

#include <PxPhysicsAPI.h>

using namespace omni::physx;
using namespace PXR_NS;

void initPhysicsKinematicBodies()
{
} // see implementation of bmInitialize


class PhysicsKinematicBodiesBenchmark : public BmBenchmark
{
public:
    PhysicsKinematicBodiesBenchmark()
        : BmBenchmark(), mPhysXSimulation(nullptr), mStage(nullptr)
    {
        mPhysXSimulation = BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysxSimulation>();
    }

    virtual ~PhysicsKinematicBodiesBenchmark()
    {
    }

    virtual uint32_t getNbRuns() const
    {
        return 10;
    }


    virtual uint32_t getNbSteps() const
    {
        return 200;
    }

    virtual void startRun()
    {
        // setup basic mStage
        mStage = UsdStage::CreateInMemory();
        PXR_NS::UsdGeomSetStageUpAxis(mStage, TfToken("Z"));
        const float metersPerStageUnit = 0.01f; // work in default centimeters
        const double metersPerUnit = PXR_NS::UsdGeomSetStageMetersPerUnit(mStage, static_cast<double>(metersPerStageUnit));
        const SdfPath defaultPrimPath = SdfPath("/World");
        UsdPrim defaultPrim = mStage->DefinePrim(defaultPrimPath);
        mStage->SetDefaultPrim(defaultPrim);

        PXR_NS::UsdUtilsStageCache::Get().Insert(mStage);
        long stageId = PXR_NS::UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();

        const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
        UsdPhysicsScene scene = UsdPhysicsScene::Define(mStage, physicsScenePath);

        const uint32_t numPrims = 1000;

        for (uint32_t i = 0; i < numPrims; i++)
        {
            // create rigid body
            std::string primPath = "/World/box" + std::to_string(i);
            UsdGeomCube cube = UsdGeomCube::Define(mStage, SdfPath(primPath));
            UsdGeomXformOp translateOp = cube.AddTranslateOp(UsdGeomXformOp::Precision::PrecisionFloat);
            translateOp.Set(GfVec3f(0.0f, 0.0f, 10.0f * i), UsdTimeCode(0.0));
            translateOp.Set(GfVec3f(50.0f, 0.0f, 10.0f * i), UsdTimeCode(200.0));

            UsdGeomXformOp rotateOp = cube.AddRotateXOp();
            rotateOp.Set(0.0f, UsdTimeCode(0.0));
            rotateOp.Set(90.0f, UsdTimeCode(100.0));
            rotateOp.Set(180.0f, UsdTimeCode(200.0));

            mPrims.push_back(cube.GetPrim());

            UsdPhysicsRigidBodyAPI rbAPI = UsdPhysicsRigidBodyAPI::Apply(cube.GetPrim());
            rbAPI.CreateKinematicEnabledAttr().Set(true);
            UsdPhysicsCollisionAPI::Apply(cube.GetPrim());
        }

        // attach sim to mStage which parses and creates the pointers that we can check directly
        mPhysXSimulation->attachStage(stageId);

        // make initial step
        mPhysXSimulation->simulate(1.0f / 60.0f, 0.0f);
        mPhysXSimulation->fetchResults();

        mStep = 0;
    }

    virtual void preStep()
    {
    }

    virtual void endRun()
    {
        mPhysXSimulation->detachStage();
        mPrims.clear();

        PXR_NS::UsdUtilsStageCache::Get().Erase(mStage);
        mStage = nullptr;
    }

protected:
    omni::physx::IPhysxSimulation* mPhysXSimulation;
    PXR_NS::UsdStageRefPtr             mStage;
    std::vector<PXR_NS::UsdPrim>       mPrims;
    uint32_t                        mStep;
};

class PhysicsKinematicBodiesBenchmarkSimulate : public PhysicsKinematicBodiesBenchmark
{
public:
    PhysicsKinematicBodiesBenchmarkSimulate()
        : PhysicsKinematicBodiesBenchmark()
    {        
    }

    virtual ~PhysicsKinematicBodiesBenchmarkSimulate()
    {
    }

    virtual void step()
    {
        mPhysXSimulation->simulate(1.0f / 60.0f, 1.0f / 60.0f * mStep++);
        mPhysXSimulation->fetchResults();

    }
};

class PhysicsKinematicBodiesBenchmarkUsdUpdate : public PhysicsKinematicBodiesBenchmark
{
public:
    PhysicsKinematicBodiesBenchmarkUsdUpdate()
        : PhysicsKinematicBodiesBenchmark()
    {
    }

    virtual ~PhysicsKinematicBodiesBenchmarkUsdUpdate()
    {
    }

    virtual void preStep()
    {
        mPositions.resize(mPrims.size());
        mScales.resize(mPrims.size());
        mOrientations.resize(mPrims.size());
    }

    virtual void step()
    {
        PXR_NS::UsdTimeCode timeCode(1.0f/60.0f * mStep++ * mStage->GetTimeCodesPerSecond());
        UsdGeomXformCache xfCache;
        xfCache.SetTime(timeCode);

        for (size_t i = mPrims.size(); i--;)
        {
            PXR_NS::GfMatrix4d localToWorld = xfCache.GetLocalToWorldTransform(mPrims[i]);

            const GfTransform tr(localToWorld);
            mPositions[i] = tr.GetTranslation();
            mOrientations[i] = tr.GetRotation().GetQuat();
            mScales[i] = tr.GetScale();
        }
    }

private:
    std::vector<PXR_NS::GfVec3d>       mPositions;
    std::vector<PXR_NS::GfVec3d>       mScales;
    std::vector<PXR_NS::GfQuatd>       mOrientations;
};

Register<PhysicsKinematicBodiesBenchmarkSimulate> prbKin("PhysicsKinematicBodies.PhysicsUpdate");
Register<PhysicsKinematicBodiesBenchmarkUsdUpdate> urbKin("PhysicsKinematicBodies.UsdUpdate");
