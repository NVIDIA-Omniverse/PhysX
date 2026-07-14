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

#include <omni/fabric/SimStageWithHistory.h>

#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/PhysxTokens.h>

#include <PxPhysicsAPI.h>

using namespace omni::physx;
using namespace PXR_NS;
using namespace carb;

void initPhysicsAttributeChange()
{
} // see implementation of bmInitialize


inline ::physx::PxQuat toPhysX(const PXR_NS::GfQuatf& v)
{
    return ::physx::PxQuat(v.GetImaginary()[0], v.GetImaginary()[1], v.GetImaginary()[2], v.GetReal());
}

class PhysicsVelocityChangesBenchmark : public BmBenchmark
{
public:
    PhysicsVelocityChangesBenchmark()
        : BmBenchmark(), mPhysXSimulation(nullptr), mRandom(50), mStage(nullptr)
    {
        mPhysXSimulation = BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysxSimulation>();
    }

    virtual ~PhysicsVelocityChangesBenchmark()
    {
    }

    virtual uint32_t getNbRuns() const
    {
        return 10;
    }


    virtual uint32_t getNbSteps() const
    {
        return 20;
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
        mStageId = PXR_NS::UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();

        const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
        UsdPhysicsScene scene = UsdPhysicsScene::Define(mStage, physicsScenePath);

        const uint32_t numPrims = 1000;

        for (uint32_t i = 0; i < numPrims; i++)
        {
            // create rigid body
            std::string primPath = "/World/box" + std::to_string(i);
            addRigidBox(mStage, primPath, GfVec3f(1.f), GfVec3f(0.0f, 0.0f, 5.0f * i), GfQuatf(1.0f),
                GfVec3f(0.7f), 0.001f);

            mPrims.push_back(SdfPath(primPath));
            mVelocities.push_back(mRandom.unitRandomPt());
        }
    }

    void preStep()
    {
        for (size_t i = 0; i < mVelocities.size(); i++)
        {
            mVelocities[i] = mRandom.unitRandomPt();
        }
    }


    virtual void endRun()
    {
        mPhysXSimulation->detachStage();
        mPrims.clear();
        mVelocities.clear();

        PXR_NS::UsdUtilsStageCache::Get().Erase(mStage);
        mStage = nullptr;
    }

protected:
    omni::physx::IPhysxSimulation*  mPhysXSimulation;
    BasicRandom                     mRandom;
    PXR_NS::UsdStageRefPtr             mStage;
    long                            mStageId;
    std::vector<PXR_NS::GfVec3f>       mVelocities;
    std::vector<PXR_NS::SdfPath>       mPrims;
};

// changes done directory to PhysX
class PhysicsVelocityChangesSolBenchmark : public PhysicsVelocityChangesBenchmark
{
public:
    PhysicsVelocityChangesSolBenchmark()
        : PhysicsVelocityChangesBenchmark()
    {
    }

    virtual ~PhysicsVelocityChangesSolBenchmark()
    {
    }

    virtual void startRun()
    {
        PhysicsVelocityChangesBenchmark::startRun();

        // attach sim to mStage which parses and creates the pointers that we can check directly
        mPhysXSimulation->attachStage(mStageId);

        // make initial step
        mPhysXSimulation->simulate(1.0f / 60.0f, 0.0f);
        mPhysXSimulation->fetchResults();

        omni::physx::IPhysx* iPhysX = BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysx>();

        for (size_t i = 0; i < mPrims.size(); i++)
        {
            ::physx::PxBase* basePtr = reinterpret_cast<::physx::PxBase *>(iPhysX->getPhysXPtr(mPrims[i], ePTActor));
            CARB_ASSERT(basePtr && basePtr->is<::physx::PxRigidDynamic>());

            mRigidBodies.push_back(basePtr->is<::physx::PxRigidDynamic>());
        }
    }

    void step()
    {
        for (size_t i = mRigidBodies.size(); i--;)
        {
            mRigidBodies[i]->setLinearVelocity((const ::physx::PxVec3&)mVelocities[i]);
        }
    }

    virtual void endRun()
    {
        mRigidBodies.clear();

        PhysicsVelocityChangesBenchmark::endRun();
    }

private:
    std::vector<::physx::PxRigidDynamic*>       mRigidBodies;
};

class PhysicsVelocityChangesUSDBenchmarkBase : public PhysicsVelocityChangesBenchmark
{
public:
    PhysicsVelocityChangesUSDBenchmarkBase()
        : PhysicsVelocityChangesBenchmark()
    {
    }

    virtual ~PhysicsVelocityChangesUSDBenchmarkBase()
    {
    }

    virtual void startRun()
    {
        PhysicsVelocityChangesBenchmark::startRun();

        // attach sim to mStage which parses and creates the pointers that we can check directly
        mPhysXSimulation->attachStage(mStageId);

        // make initial step
        mPhysXSimulation->simulate(1.0f / 60.0f, 0.0f);
        mPhysXSimulation->fetchResults();

        for (size_t i = 0; i < mPrims.size(); i++)
        {
            mRigidBodyAPIs.push_back(PXR_NS::UsdPhysicsRigidBodyAPI::Get(mStage, mPrims[i]));
        }
    }

    virtual void endRun()
    {
        mRigidBodyAPIs.clear();

        PhysicsVelocityChangesBenchmark::endRun();
    }

protected:
    std::vector<PXR_NS::UsdPhysicsRigidBodyAPI>       mRigidBodyAPIs;
};

class PhysicsVelocityChangesUSDBenchmarkNonBatched : public PhysicsVelocityChangesUSDBenchmarkBase
{
public:
    PhysicsVelocityChangesUSDBenchmarkNonBatched()
        : PhysicsVelocityChangesUSDBenchmarkBase()
    {
    }

    virtual ~PhysicsVelocityChangesUSDBenchmarkNonBatched()
    {
    }

    void step()
    {
        for (size_t i = mRigidBodyAPIs.size(); i--;)
        {
            mRigidBodyAPIs[i].GetVelocityAttr().Set(mVelocities[i]);
        }
    }
};

class PhysicsVelocityChangesUSDBenchmarkBatched : public PhysicsVelocityChangesUSDBenchmarkBase
{
public:
    PhysicsVelocityChangesUSDBenchmarkBatched()
        : PhysicsVelocityChangesUSDBenchmarkBase()
    {
    }

    virtual ~PhysicsVelocityChangesUSDBenchmarkBatched()
    {
    }

    void step()
    {
        SdfChangeBlock block;
        for (size_t i = mRigidBodyAPIs.size(); i--;)
        {
            mRigidBodyAPIs[i].GetVelocityAttr().Set(mVelocities[i]);
        }
    }
};

class PhysicsVelocityChangesUSDBenchmarkNoticeOnly : public PhysicsVelocityChangesUSDBenchmarkBase
{
public:
    PhysicsVelocityChangesUSDBenchmarkNoticeOnly()
        : PhysicsVelocityChangesUSDBenchmarkBase(), mChangeBlock(nullptr)
    {
    }

    virtual ~PhysicsVelocityChangesUSDBenchmarkNoticeOnly()
    {
    }

    virtual void preStep()
    {
        PhysicsVelocityChangesUSDBenchmarkBase::preStep();

        mChangeBlock = new SdfChangeBlock();

        for (size_t i = mRigidBodyAPIs.size(); i--;)
        {
            mRigidBodyAPIs[i].GetVelocityAttr().Set(mVelocities[i]);
        }
    }

    void step()
    {
        delete mChangeBlock;
    }

protected:
    PXR_NS::SdfChangeBlock* mChangeBlock;
};

// changes done through fabric
class PhysicsVelocityChangesFabricBenchmarkBase : public PhysicsVelocityChangesBenchmark
{
public:
    PhysicsVelocityChangesFabricBenchmarkBase()
        : PhysicsVelocityChangesBenchmark(), mFrame(1)
    {
    }

    virtual ~PhysicsVelocityChangesFabricBenchmarkBase()
    {
    }

    virtual void startRun()
    {
        PhysicsVelocityChangesBenchmark::startRun();

        mStageId = PXR_NS::UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();
        mFabricChange.init(mStageId, BmGlobals::getInstance().getFramework());

        mVelocityToken = omni::fabric::Token(UsdPhysicsTokens->physicsVelocity.GetText());

        // attach sim to mStage which parses and creates the pointers that we can check directly
        mPhysXSimulation->attachStage(mStageId);

        // make initial step
        mPhysXSimulation->simulate(1.0f / 60.0f, 0.0f);
        mPhysXSimulation->fetchResults();
    }

    void preStep()
    {
        PhysicsVelocityChangesBenchmark::preStep();

        mFabricChange.mSrwId = mFabricChange.iStageReaderWriter->create({ uint64_t(mStageId) }, mFrame++);
    }

    virtual void endRun()
    {
        mFabricChange.destroy();

        PhysicsVelocityChangesBenchmark::endRun();
    }

protected:
    FabricChange                             mFabricChange;
    uint32_t                                    mFrame;
    omni::fabric::Token                      mVelocityToken;
};

class PhysicsVelocityChangesFabricBenchmarkFull : public PhysicsVelocityChangesFabricBenchmarkBase
{
public:
    PhysicsVelocityChangesFabricBenchmarkFull()
        : PhysicsVelocityChangesFabricBenchmarkBase()
    {
    }

    virtual ~PhysicsVelocityChangesFabricBenchmarkFull()
    {
    }

    virtual uint32_t getNbRuns() const
    {
        return 50;
    }


    virtual uint32_t getNbSteps() const
    {
        return 1;
    }

    void step()
    {
        for (size_t i = mPrims.size(); i--;)
        {
            mFabricChange.setAttributeValue(mPrims[i], UsdPhysicsTokens->physicsVelocity, mVelocities[i]);
        }
        mPhysXSimulation->flushChanges();
    }

private:
};

class PhysicsVelocityChangesFabricBenchmarkChangeTracking: public PhysicsVelocityChangesFabricBenchmarkBase
{
public:
    PhysicsVelocityChangesFabricBenchmarkChangeTracking()
        : PhysicsVelocityChangesFabricBenchmarkBase()
    {
    }

    virtual ~PhysicsVelocityChangesFabricBenchmarkChangeTracking()
    {
    }

    void preStep()
    {
        PhysicsVelocityChangesFabricBenchmarkBase::preStep();

        for (size_t i = mPrims.size(); i--;)
        {
            const omni::fabric::Path fabricPath = omni::fabric::convertToPathType<omni::fabric::Path>(
                mFabricChange.iStageReaderWriter->getFabricId(mFabricChange.mSrwId), mPrims[i]);

            omni::fabric::Type float3Type(omni::fabric::BaseDataType::eFloat, 3, 0);
            mFabricChange.iStageReaderWriter->createAttribute(
                mFabricChange.mSrwId, fabricPath, mVelocityToken, omni::fabric::TypeC(float3Type));
        }
    }

    void step()
    {
        const omni::fabric::Type typeAppliedSchema(omni::fabric::BaseDataType::eTag, 1, 0, omni::fabric::AttributeRole::eAppliedSchema);
        const omni::fabric::Token tokenRigidBody("PhysicsRigidBodyAPI");
        omni::fabric::Type float3Type(omni::fabric::BaseDataType::eFloat, 3, 0);

        omni::fabric::StageReaderWriter stageIP = mFabricChange.iStageReaderWriter->get(mFabricChange.mStageId);
        const omni::fabric::set<omni::fabric::AttrNameAndType> requiredAll = { omni::fabric::AttrNameAndType(typeAppliedSchema, tokenRigidBody) };
        omni::fabric::PrimBucketList primBuckets = stageIP.findPrims(requiredAll);
        size_t bucketCount = primBuckets.bucketCount();
        for (size_t i = 0; i != bucketCount; i++)
        {
            gsl::span<const omni::fabric::Path> paths = stageIP.getPathArray(primBuckets, i);

            gsl::span<PXR_NS::GfVec3f> linVelocities =
                stageIP.getAttributeArray<PXR_NS::GfVec3f>(primBuckets, i, mVelocityToken);

            size_t j = 0;
            for (const omni::fabric::Path& path : paths)
            {
                linVelocities[j] = mVelocities[j];
                j++;
            }
        }

        mPhysXSimulation->flushChanges();
    }

private:
};

class PhysicsVelocityChangesPointInstancerBenchmark : public BmBenchmark
{
public:
    PhysicsVelocityChangesPointInstancerBenchmark()
        : BmBenchmark(), mPhysXSimulation(nullptr), mRandom(50), mStage(nullptr)
    {
        mPhysXSimulation = BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysxSimulation>();
    }

    virtual ~PhysicsVelocityChangesPointInstancerBenchmark()
    {
    }

    virtual uint32_t getNbRuns() const
    {
        return 10;
    }


    virtual uint32_t getNbSteps() const
    {
        return 20;
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

        const SdfPath geomPointInstancerPath = defaultPrimPath.AppendChild(TfToken("pointInstancer"));
        const SdfPath boxActorPath = geomPointInstancerPath.AppendChild(TfToken("boxActor"));

        UsdGeomCube cubeGeom = UsdGeomCube::Define(mStage, boxActorPath);

        UsdPhysicsCollisionAPI::Apply(cubeGeom.GetPrim());
        UsdPhysicsRigidBodyAPI::Apply(cubeGeom.GetPrim());

        VtArray<int> meshIndices;
        VtArray<GfVec3f> positions;
        VtArray<GfQuath> orientations;
        VtArray<GfVec3f> angularVelocities;

        mPointInstancer = UsdGeomPointInstancer::Define(mStage, geomPointInstancerPath);
        mPointInstancer.GetPrototypesRel().AddTarget(boxActorPath);

        const uint32_t numPrims = 1000;

        for (uint32_t i = 0; i < numPrims; i++)
        {
            meshIndices.push_back(0);
            positions.push_back(GfVec3f(0.0f, 0.0f, 5.0f * i));
            mLinearVelocities.push_back(GfVec3f(0.0f));
            angularVelocities.push_back(GfVec3f(0.0f));
            orientations.push_back(GfQuath(1.0f));
        }

        mPointInstancer.GetProtoIndicesAttr().Set(meshIndices);
        mPointInstancer.GetPositionsAttr().Set(positions);
        mPointInstancer.GetOrientationsAttr().Set(orientations);
        mPointInstancer.GetVelocitiesAttr().Set(mLinearVelocities);
        mPointInstancer.GetAngularVelocitiesAttr().Set(angularVelocities);

        // attach sim to mStage which parses and creates the pointers that we can check directly
        mPhysXSimulation->attachStage(stageId);

        // make initial step
        mPhysXSimulation->simulate(1.0f / 60.0f, 0.0f);
        mPhysXSimulation->fetchResults();
    }

    void preStep()
    {
        for (size_t i = 0; i < mLinearVelocities.size(); i++)
        {
            mLinearVelocities[i] = mRandom.unitRandomPt();
        }
    }


    virtual void endRun()
    {
        mPhysXSimulation->detachStage();
        mLinearVelocities.clear();

        PXR_NS::UsdUtilsStageCache::Get().Erase(mStage);
        mStage = nullptr;
    }

protected:
    omni::physx::IPhysxSimulation*  mPhysXSimulation;
    BasicRandom                     mRandom;
    PXR_NS::UsdStageRefPtr             mStage;
    VtArray<GfVec3f>                mLinearVelocities;
    UsdGeomPointInstancer           mPointInstancer;
};

class PhysicsVelocityChangesUSDPointInstancerBenchmark: public PhysicsVelocityChangesPointInstancerBenchmark
{
public:
    PhysicsVelocityChangesUSDPointInstancerBenchmark()
        : PhysicsVelocityChangesPointInstancerBenchmark()
    {
    }

    ~PhysicsVelocityChangesUSDPointInstancerBenchmark()
    {
    }

    void step()
    {
        mPointInstancer.GetVelocitiesAttr().Set(mLinearVelocities);
    }

};


Register<PhysicsVelocityChangesSolBenchmark> pvcSol("PhysicsChanges.VelocitySOL");

Register<PhysicsVelocityChangesUSDBenchmarkNonBatched> pvcUSDnonb("PhysicsChanges.VelocityUSDNonBatched");
Register<PhysicsVelocityChangesUSDBenchmarkBatched> pvcUSDb("PhysicsChanges.VelocityUSDBatched");
Register<PhysicsVelocityChangesUSDBenchmarkNoticeOnly> pvcUSDNo("PhysicsChanges.VelocityNoticeOnly");

Register<PhysicsVelocityChangesFabricBenchmarkFull> pvcFabric("PhysicsChanges.VelocityFabricFull");
Register<PhysicsVelocityChangesFabricBenchmarkChangeTracking> pvcFabricChangeTrack("PhysicsChanges.VelocityFabricChangeTracking");

Register<PhysicsVelocityChangesUSDPointInstancerBenchmark> pvcUSDPINo("PhysicsChanges.VelocityUSDPointInstancer");


class PhysicsTransformationChangesBenchmark : public BmBenchmark
{
public:
    PhysicsTransformationChangesBenchmark()
        : BmBenchmark(), mPhysXSimulation(nullptr), mRandom(50), mStage(nullptr)
    {
        mPhysXSimulation = BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysxSimulation>();
    }

    virtual ~PhysicsTransformationChangesBenchmark()
    {
    }

    virtual uint32_t getNbRuns() const
    {
        return 10;
    }


    virtual uint32_t getNbSteps() const
    {
        return 20;
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
        mStageId = PXR_NS::UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();

        const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
        UsdPhysicsScene scene = UsdPhysicsScene::Define(mStage, physicsScenePath);

        const uint32_t numPrims = 1000;

        for (uint32_t i = 0; i < numPrims; i++)
        {
            // create rigid body
            std::string primPath = "/World/box" + std::to_string(i);
            addRigidBox(mStage, primPath, GfVec3f(1.f), GfVec3f(0.0f, 0.0f, 5.0f * i), GfQuatf(1.0f),
                GfVec3f(0.7f), 0.001f);

            mPrims.push_back(SdfPath(primPath));
            mPositions.push_back(mRandom.unitRandomPt() * 1000.0f);
            mRotations.push_back(mRandom.unitRandomQuat());
        }
    }

    void preStep()
    {
        for (size_t i = 0; i < mPositions.size(); i++)
        {
            mPositions[i] = mRandom.unitRandomPt() * 1000.0f;
            mRotations[i] = mRandom.unitRandomQuat();
        }
    }


    virtual void endRun()
    {
        mPhysXSimulation->detachStage();
        mPrims.clear();
        mPositions.clear();
        mRotations.clear();

        PXR_NS::UsdUtilsStageCache::Get().Erase(mStage);
        mStage = nullptr;
    }

protected:
    omni::physx::IPhysxSimulation* mPhysXSimulation;
    BasicRandom                     mRandom;
    PXR_NS::UsdStageRefPtr             mStage;
    long                            mStageId;
    std::vector<PXR_NS::GfVec3f>       mPositions;
    std::vector<PXR_NS::GfQuatf>       mRotations;
    std::vector<PXR_NS::SdfPath>       mPrims;
};

// changes done directory to PhysX
class PhysicsTransformationChangesSolBenchmark : public PhysicsTransformationChangesBenchmark
{
public:
    PhysicsTransformationChangesSolBenchmark()
        : PhysicsTransformationChangesBenchmark()
    {
    }

    virtual ~PhysicsTransformationChangesSolBenchmark()
    {
    }

    virtual void startRun()
    {
        PhysicsTransformationChangesBenchmark::startRun();

        // attach sim to mStage which parses and creates the pointers that we can check directly
        mPhysXSimulation->attachStage(mStageId);

        // make initial step
        mPhysXSimulation->simulate(1.0f / 60.0f, 0.0f);
        mPhysXSimulation->fetchResults();

        omni::physx::IPhysx* iPhysX = BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysx>();

        for (size_t i = 0; i < mPrims.size(); i++)
        {
            ::physx::PxBase* basePtr = reinterpret_cast<::physx::PxBase*>(iPhysX->getPhysXPtr(mPrims[i], ePTActor));
            CARB_ASSERT(basePtr && basePtr->is<::physx::PxRigidDynamic>());

            mRigidBodies.push_back(basePtr->is<::physx::PxRigidDynamic>());
        }
    }

    void step()
    {
        for (size_t i = mRigidBodies.size(); i--;)
        {
            mRigidBodies[i]->setGlobalPose(::physx::PxTransform((const ::physx::PxVec3&) mPositions[i], toPhysX(mRotations[i])));
        }
    }

    virtual void endRun()
    {
        mRigidBodies.clear();

        PhysicsTransformationChangesBenchmark::endRun();
    }

private:
    std::vector<::physx::PxRigidDynamic*>       mRigidBodies;
};

class PhysicsTransformationUSDChangesBenchmark : public PhysicsTransformationChangesBenchmark
{
public:
    PhysicsTransformationUSDChangesBenchmark()
        : PhysicsTransformationChangesBenchmark()
    {
    }

    virtual ~PhysicsTransformationUSDChangesBenchmark()
    {
    }

    virtual void startRun()
    {
        PhysicsTransformationChangesBenchmark::startRun();

        // attach sim to mStage which parses and creates the pointers that we can check directly
        mPhysXSimulation->attachStage(mStageId);

        // make initial step
        mPhysXSimulation->simulate(1.0f / 60.0f, 0.0f);
        mPhysXSimulation->fetchResults();

        for (size_t i = 0; i < mPrims.size(); i++)
        {
            static const TfToken xformOpPosition = TfToken("xformOp:translate");
            static const TfToken xformOpRotation = TfToken("xformOp:orient");
            UsdAttribute posProp = mStage->GetPrimAtPath(mPrims[i]).GetAttribute(xformOpPosition);
            UsdAttribute rotProp = mStage->GetPrimAtPath(mPrims[i]).GetAttribute(xformOpRotation);
            mPositionAttr.push_back(posProp);
            mRotationAttr.push_back(rotProp);
        }
    }

    virtual void endRun()
    {
        mPositionAttr.clear();
        mRotationAttr.clear();

        PhysicsTransformationChangesBenchmark::endRun();
    }

protected:
    std::vector<PXR_NS::UsdAttribute>       mPositionAttr;
    std::vector<PXR_NS::UsdAttribute>       mRotationAttr;
};

class PhysicsTransformationChangesUSDBenchmarkNonBatched : public PhysicsTransformationUSDChangesBenchmark
{
public:
    PhysicsTransformationChangesUSDBenchmarkNonBatched()
        : PhysicsTransformationUSDChangesBenchmark()
    {
    }

    virtual ~PhysicsTransformationChangesUSDBenchmarkNonBatched()
    {
    }

    void step()
    {
        for (size_t i = mPositionAttr.size(); i--;)
        {
            mPositionAttr[i].Set(mPositions[i]);
            mRotationAttr[i].Set(mRotations[i]);
        }
    }
};

class PhysicsTransformationChangesUSDBenchmarkBatched : public PhysicsTransformationUSDChangesBenchmark
{
public:
    PhysicsTransformationChangesUSDBenchmarkBatched()
        : PhysicsTransformationUSDChangesBenchmark()
    {
    }

    virtual ~PhysicsTransformationChangesUSDBenchmarkBatched()
    {
    }

    void step()
    {
        SdfChangeBlock block;
        for (size_t i = mPositionAttr.size(); i--;)
        {
            mPositionAttr[i].Set(mPositions[i]);
            mRotationAttr[i].Set(mRotations[i]);
        }
    }
};

class PhysicsTransformationChangesUSDBenchmarkNoticeOnly : public PhysicsTransformationUSDChangesBenchmark
{
public:
    PhysicsTransformationChangesUSDBenchmarkNoticeOnly()
        : PhysicsTransformationUSDChangesBenchmark(), mChangeBlock(nullptr)
    {
    }

    virtual ~PhysicsTransformationChangesUSDBenchmarkNoticeOnly()
    {
    }

    virtual void preStep()
    {
        PhysicsTransformationUSDChangesBenchmark::preStep();

        mChangeBlock = new SdfChangeBlock();

        for (size_t i = mPositionAttr.size(); i--;)
        {
            mPositionAttr[i].Set(mPositions[i]);
            mRotationAttr[i].Set(mRotations[i]);
        }
    }

    void step()
    {
        delete mChangeBlock;
    }

protected:
    PXR_NS::SdfChangeBlock* mChangeBlock;
};

// changes done through fabric
class PhysicsTransformationChangesFabricBenchmarkBase : public PhysicsTransformationChangesBenchmark
{
public:
    PhysicsTransformationChangesFabricBenchmarkBase()
        : PhysicsTransformationChangesBenchmark(), mFrame(1)
    {
    }

    virtual ~PhysicsTransformationChangesFabricBenchmarkBase()
    {
    }

    virtual void startRun()
    {
        PhysicsTransformationChangesBenchmark::startRun();

        mStageId = PXR_NS::UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();

        mFabricChange.init(mStageId, BmGlobals::getInstance().getFramework());

        mWorldMatrixToken = omni::fabric::Token(gWorldMatrixTokenString);
        mLocalMatrixToken = omni::fabric::Token(gLocalMatrixTokenString);

        // attach sim to mStage which parses and creates the pointers that we can check directly
        mPhysXSimulation->attachStage(mStageId);

        // make initial step
        mPhysXSimulation->simulate(1.0f / 60.0f, 0.0f);
        mPhysXSimulation->fetchResults();
    }

    void preStep()
    {
        PhysicsTransformationChangesBenchmark::preStep();

        mFabricChange.mSrwId = mFabricChange.iStageReaderWriter->create({ uint64_t(mStageId) }, mFrame++);
    }

    virtual void endRun()
    {
        mFabricChange.destroy();

        PhysicsTransformationChangesBenchmark::endRun();
    }

protected:
    FabricChange                             mFabricChange;
    long                                        mStageId;
    uint32_t                                    mFrame;
    omni::fabric::Token                      mWorldMatrixToken;
    omni::fabric::Token                      mLocalMatrixToken;
};

class PhysicsTransformationChangesFabricChangeTracking: public PhysicsTransformationChangesFabricBenchmarkBase
{
public:
    PhysicsTransformationChangesFabricChangeTracking()
        : PhysicsTransformationChangesFabricBenchmarkBase()
    {
    }

    virtual ~PhysicsTransformationChangesFabricChangeTracking()
    {
    }

    void preStep()
    {
        PhysicsTransformationChangesFabricBenchmarkBase::preStep();

        omni::fabric::Type matrix4dType(omni::fabric::BaseDataType::eDouble, 16, 0, omni::fabric::AttributeRole::eMatrix);
        for (size_t i = mPrims.size(); i--;)
        {
            const omni::fabric::Path fabricPath = omni::fabric::convertToPathType<omni::fabric::Path>(
                mFabricChange.iStageReaderWriter->getFabricId(mFabricChange.mSrwId), mPrims[i]);

            mFabricChange.iStageReaderWriter->createAttribute(mFabricChange.mSrwId, fabricPath , mWorldMatrixToken, omni::fabric::TypeC(matrix4dType));
            mFabricChange.iStageReaderWriter->createAttribute(mFabricChange.mSrwId, fabricPath , mLocalMatrixToken, omni::fabric::TypeC(matrix4dType));
        }

    }

    void step()
    {
        const omni::fabric::Type typeAppliedSchema(omni::fabric::BaseDataType::eTag, 1, 0, omni::fabric::AttributeRole::eAppliedSchema);
        const omni::fabric::Token tokenRigidBody("PhysicsRigidBodyAPI");

        omni::fabric::StageReaderWriter stageIP = mFabricChange.iStageReaderWriter->get(mFabricChange.mStageId);
        const omni::fabric::set<omni::fabric::AttrNameAndType> requiredAll = { omni::fabric::AttrNameAndType(typeAppliedSchema, tokenRigidBody) };
        omni::fabric::PrimBucketList primBuckets = stageIP.findPrims(requiredAll);
        size_t bucketCount = primBuckets.bucketCount();
        for (size_t i = 0; i != bucketCount; i++)
        {
            gsl::span<const omni::fabric::Path> paths = stageIP.getPathArray(primBuckets, i);

            gsl::span<PXR_NS::GfMatrix4d> matrices = stageIP.getAttributeArray<PXR_NS::GfMatrix4d>(primBuckets, i, mLocalMatrixToken);

            size_t j = 0;
            for (const omni::fabric::Path& path : paths)
            {
                matrices[j].SetTranslate(mPositions[j]); // this will reset the scale
                matrices[j].SetRotateOnly(mRotations[j]);
                j++;
            }
        }

        mPhysXSimulation->flushChanges();
    }

private:
};


class PhysicsTransformationChangesPointInstancerBenchmark : public BmBenchmark
{
public:
    PhysicsTransformationChangesPointInstancerBenchmark()
        : BmBenchmark(), mPhysXSimulation(nullptr), mRandom(50), mStage(nullptr)
    {
        mPhysXSimulation = BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysxSimulation>();
    }

    virtual ~PhysicsTransformationChangesPointInstancerBenchmark()
    {
    }

    virtual uint32_t getNbRuns() const
    {
        return 10;
    }


    virtual uint32_t getNbSteps() const
    {
        return 20;
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

        const SdfPath geomPointInstancerPath = defaultPrimPath.AppendChild(TfToken("pointInstancer"));
        const SdfPath boxActorPath = geomPointInstancerPath.AppendChild(TfToken("boxActor"));

        UsdGeomCube cubeGeom = UsdGeomCube::Define(mStage, boxActorPath);

        UsdPhysicsCollisionAPI::Apply(cubeGeom.GetPrim());
        UsdPhysicsRigidBodyAPI::Apply(cubeGeom.GetPrim());

        VtArray<int> meshIndices;
        VtArray<GfVec3f> linearVelocities;
        VtArray<GfVec3f> angularVelocities;

        mPointInstancer = UsdGeomPointInstancer::Define(mStage, geomPointInstancerPath);
        mPointInstancer.GetPrototypesRel().AddTarget(boxActorPath);

        const uint32_t numPrims = 1000;

        for (uint32_t i = 0; i < numPrims; i++)
        {
            meshIndices.push_back(0);
            mPositionsArray.push_back(GfVec3f(0.0f, 0.0f, 5.0f * i));
            linearVelocities.push_back(GfVec3f(0.0f));
            angularVelocities.push_back(GfVec3f(0.0f));
            mOrientationsArray.push_back(GfQuath(1.0f));
        }

        mPointInstancer.GetProtoIndicesAttr().Set(meshIndices);
        mPointInstancer.GetPositionsAttr().Set(mPositionsArray);
        mPointInstancer.GetOrientationsAttr().Set(mOrientationsArray);
        mPointInstancer.GetVelocitiesAttr().Set(linearVelocities);
        mPointInstancer.GetAngularVelocitiesAttr().Set(angularVelocities);

        // attach sim to mStage which parses and creates the pointers that we can check directly
        mPhysXSimulation->attachStage(stageId);

        // make initial step
        mPhysXSimulation->simulate(1.0f / 60.0f, 0.0f);
        mPhysXSimulation->fetchResults();
    }

    void preStep()
    {
        for (size_t i = 0; i < mPositionsArray.size(); i++)
        {
            mPositionsArray[i] = mRandom.unitRandomPt() * 1000.0f;
            mOrientationsArray[i] = mRandom.unitRandomQuath();
        }
    }


    virtual void endRun()
    {
        mPhysXSimulation->detachStage();
        mPositionsArray.clear();
        mOrientationsArray.clear();

        PXR_NS::UsdUtilsStageCache::Get().Erase(mStage);
        mStage = nullptr;
    }

protected:
    omni::physx::IPhysxSimulation* mPhysXSimulation;
    BasicRandom                     mRandom;
    PXR_NS::UsdStageRefPtr             mStage;
    UsdGeomPointInstancer           mPointInstancer;
    VtArray<GfVec3f>                mPositionsArray;
    VtArray<GfQuath>                mOrientationsArray;
};


class PhysicsTransformationChangesUSDPointInstancerBenchmark : public PhysicsTransformationChangesPointInstancerBenchmark
{
public:
    PhysicsTransformationChangesUSDPointInstancerBenchmark()
        : PhysicsTransformationChangesPointInstancerBenchmark()
    {
    }

    ~PhysicsTransformationChangesUSDPointInstancerBenchmark()
    {
    }

    void step()
    {
        mPointInstancer.GetPositionsAttr().Set(mPositionsArray);
        mPointInstancer.GetOrientationsAttr().Set(mOrientationsArray);
    }

};


Register<PhysicsTransformationChangesSolBenchmark> ptrSol("PhysicsChanges.TransformationSOL");

Register<PhysicsTransformationChangesUSDBenchmarkNonBatched> ptrUSDnonb("PhysicsChanges.TransformationUSDNonBatched");
Register<PhysicsTransformationChangesUSDBenchmarkBatched> ptrUSDb("PhysicsChanges.TransformationUSDBatched");
Register<PhysicsTransformationChangesUSDBenchmarkNoticeOnly> ptrUSDNo("PhysicsChanges.TransformationNoticeOnly");

Register<PhysicsTransformationChangesUSDPointInstancerBenchmark> ptrUSDPI("PhysicsChanges.TransformationUSDPointInstancer");

Register<PhysicsTransformationChangesFabricChangeTracking> ptrFabricChangeTracking("PhysicsChanges.TransformationFabricChangeTracking");
