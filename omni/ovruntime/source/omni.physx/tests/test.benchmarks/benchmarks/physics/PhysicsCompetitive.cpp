// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#include "UsdPCH.h"


#include "../../common/TestHelpers.h"
#include "../../framework/BmBenchmark.h"
#include "../../framework/BmGlobals.h"
#include "omni/physx/IPhysxReplicator.h"
#include "omni/physx/IPhysxSimulation.h"

#include <string>
#include <cmath>
#include <random>
#include <cuda.h>
#include <memory>

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxTests.h>
#include <omni/physx/IPhysxSettings.h>

#include <PxPhysicsAPI.h>
#include "MLIRPolicy.h"
#include "Utils.h"


void initPhysicsCompetitive() { } // see implementation of bmInitialize

using namespace carb;
using namespace PXR_NS;
using namespace omni::physx;
using namespace physx;

// Competitive benchmarks
// 3 Benchmarks are implemented
// - Anymal, random jitting joint targets
// - Humanoid, walking policy
// - Humanoid kitchen scene
//
//  ** Notes **
// - Joint drive targets are written via the Direct GPU API when running on GPU
// - During construction:
//     - Scene is loaded
//     - Replication or cloning
//     - Buffers for direct GPU API are allocated
//
// - At the beginning of each run:
//     - Scene is loaded
//     - Replication or cloning
//     - One step is taken
//     - Direct GPU API is initialized

enum struct ComputeDevice { CPU = 0, GPU = 1};
enum struct PolicyType { NONE = 0, CANNED = 1, MLIR = 2};
enum struct Separation { ENVIDS = 0, AREA = 2 };

const static std::string ExportStartUSD = "/tmp/export_scene_multi_start.usd";
const std::string ExportEndUSD = "/tmp/export_scene_multi_end.usd";

const static bool gDumpSceneStartEnd = false;  // Dump scene on first run after setup and when finished
const static bool gRecordCanned = false; // Record canned policies

// To record a policy, set the policy type to MLIR, set gRecordCanned to true, and run the benchmark.
// The benchmark can then be run with policytype PolicyType Canned and gRecordCanned false
// If the model is run with type MLIRpolicy and the policy file cannot be found, it tries to find a canned policy.

// Config for competitive benchmarks
struct CBMCfg
{
    bool mCreateGroundPlane = true;

    // benchmark options
    int mNbEnvs = 4;
    int mNbSteps = 200;
    float mEnvSpacing = 1.0;
    float mDt = 1.0f/200;

    bool mUseEnvIds = false;
    bool mUseReplicator = true;

    // USD config
    std::string mModelPath;
    SdfPath mModelPrimPath;
    int mNbDofsPerEnv = 0;
    int mNbObservations = 0;

    // Policy config
    PolicyType mPolicyType = PolicyType::NONE;
    std::string mPolicyName;

    bool validate() const
    {
        CARB_ASSERT(mUseReplicator || !mUseEnvIds, "Env IDs are not available without replicator");
        return true;
    }
};


template <ComputeDevice tDevice, uint32_t tThreadCount>
class PhysicsCompetitiveBenchmark : public BmBenchmark
{
public:
    PhysicsCompetitiveBenchmark(CBMCfg config,
                                uint32_t nbEnvs,
                                Separation sep,
                                const char* const* argv = nullptr,
                                size_t argc = 0) :
        BmBenchmark(), mCfg(config), mArgv(argv), mArgc(argc), mPhysXScene(nullptr)
    {
        if (sep == Separation::AREA)
        {
            mCfg.mUseEnvIds = false;
        }
        else if (sep == Separation::ENVIDS)
        {
            mCfg.mUseEnvIds = true;
            mCfg.mUseReplicator = true;
            mCfg.mEnvSpacing = 0;
        }
        else
        {
            CARB_LOG_ERROR("Separation set to unknown value");
        }
        mCfg.validate();

        mCfg.mNbEnvs = nbEnvs;
        mNbRobotDofs = mCfg.mNbDofsPerEnv;

        // load interfaces
        carb::Framework* framework = BmGlobals::getInstance().getFramework();
        mPhysXBenchmarks = framework->acquireInterface<omni::physx::IPhysxBenchmarks>();
        mPhysXSimulation = framework->acquireInterface<omni::physx::IPhysxSimulation>();
        mPhysX = framework->acquireInterface<omni::physx::IPhysx>();

        // restore settings at destruction
        mSettingsGuard = std::make_unique<SettingsGuard>(framework->acquireInterface<carb::settings::ISettings>());
        framework->acquireInterface<carb::settings::ISettings>()->setInt(omni::physx::kSettingOverrideGPU, 1);

        mSettingsGuard->addInt(omni::physx::kSettingOverrideGPU, tDevice == ComputeDevice::GPU);
        mSettingsGuard->addInt(omni::physx::kSettingSuppressReadback, tDevice == ComputeDevice::GPU);
        mSettingsGuard->addInt(omni::physx::kSettingNumThreads);
        mSettingsGuard->addBool(kSettingUpdateToUsd, false);

        loadPolicy(); // load policy, if any
    }

    virtual bool isValid() const override { return mPolicyMLIR || mPolicyCanned; };

    // compute the observations for the NN policy, if one is used
    virtual void computeObservations(int nbArticulations) { }

    // define joint drive targets
    // dofs(output): joint drive target array
    // nbDofs(input): number of dofs to sample
    // nbArticulations(input): number of articulations to sample dofs for
    // artIdx(input): index of the first articulation (rest assumed to be contiguous)
    virtual void sampleJointDriveTargets(float* dofs, int nbDofs, int nbArticulations, int artIdx)
    {
        getPolicyDriveTargets(dofs, nbDofs, nbArticulations, artIdx);
    };

    // apply joint drive targets
    // policyOutput(input): policy output (values for artIdx starting at position 0)
    // dofs(output): joint drive target array
    // nbArticulations(input): number of articulations to sample dofs for
    // artIdx(input): index of the first articulation (rest assumed to be contiguous)
    virtual void applyPolicyActions(float* dofs, const float* policyOutput, int nbArticulations, int artIdx)
    {
    };

    // load the USD stage, clone and replicate the env, create a ground plane
    void prepareStage()
    {
        loadUsdStage();

        if (mCfg.mCreateGroundPlane)
        {
            // setup ground plane at zero pointing up:
            UsdGeomPlane groundPlane = UsdGeomPlane::Define(mStage, SdfPath("/World").AppendElementString("GroundPlane"));
            groundPlane.CreateAxisAttr().Set(UsdGeomGetStageUpAxis(mStage));
            groundPlane.AddTranslateOp().Set(GfVec3f(0.0f));
            groundPlane.AddOrientOp().Set(GfQuatf(0.0f));
            groundPlane.AddScaleOp().Set(GfVec3f(1.0f));
            UsdPhysicsCollisionAPI::Apply(groundPlane.GetPrim());
        }

        clone(mCfg.mNbEnvs, mCfg.mEnvSpacing, false);  // TODO(CA): prefer shallow "cloning" with replicator

        if (mCfg.mUseReplicator)
            replicate();
        else
        {
            if (!mPhysXSimulation->attachStage(mStageId))
                CARB_LOG_ERROR("Failed to attach stage %ld.", mStageId);
            mPhysX->forceLoadPhysicsFromUSD();
        }

        if (gDumpSceneStartEnd && mRunIdx == 0)
        {
            exportUsdStage(ExportStartUSD);
            CARB_LOG_INFO("Initial stage written to %s", ExportStartUSD.c_str());
        }
    }

    void replicate()
    {
        IPhysxReplicator* replicator =
            BmGlobals::getInstance().getFramework()->acquireInterface<omni::physx::IPhysxReplicator>();

        IReplicatorCallback repCb = { nullptr, nullptr, nullptr };

        repCb.replicationAttachFn = [](uint64_t stageId, uint32_t& numExludePaths, uint64_t*& excludePaths, void* userData) {
            numExludePaths = 1;
            const SdfPath boxPath = SdfPath("/envs");
            static uint64_t excludePath = sdfPathToInt(boxPath);
            excludePaths = &excludePath;
        };

        repCb.hierarchyRenameFn = [](uint64_t replicatePath, uint32_t index, void* userData) {
            std::string stringPath = "/envs/env_" + std::to_string(index + 1);
            const SdfPath outPath(stringPath);
            return sdfPathToInt(outPath);
        };

        replicator->registerReplicator(mStageId, repCb);
        if (!mPhysXSimulation->attachStage(mStageId))
            CARB_LOG_ERROR("Failed to attach stage %ld.", mStageId);

        replicator->replicate(
                mStageId, sdfPathToInt(SdfPath("/envs/env_0")), mCfg.mNbEnvs - 1, mCfg.mUseEnvIds, false);
    }

    void freeBuffers()
    {
        if (tDevice == ComputeDevice::CPU)
            return;
        mDriveTargetsGPU.free();
        mArtGPUIdx.free();
        mRootVec3.free();
        mRootTransform.free();
        mJointReal.free();
    }

    void checkGPUAPI()
    {
        // check if setup is correct
        PxSceneFlags sceneFlags = mPhysXScene->getFlags();
        if (!sceneFlags.isSet(PxSceneFlag::eENABLE_GPU_DYNAMICS))
            CARB_LOG_ERROR("Failed to prepare GPU data: simulation is running on CPU");
        else if (!sceneFlags.isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API))
            CARB_LOG_ERROR("Failed to prepare GPU data: direct GPU API is not enabled");
    }

    void preStep() override // pre-step setup  (happens before cache clearing and allocator reset)
    {
        if (mStep >= 2)
            setJointDriveTargets();
    }

    void simStep()
    {

        mPhysXSimulation->simulate(mCfg.mDt, mT);
        mPhysXSimulation->fetchResults();
        mT += mCfg.mDt;
        ++mStep;
    }

    ~PhysicsCompetitiveBenchmark() { mPhysXBenchmarks->loadTargetStage(nullptr); }

    // * load the usd stage containing the model into the stage cache
    // * populate `mStage` & `mStageId`
    // * wrap the model in env prims
    void loadUsdStage()
    {
        // load usd
        const std::string usdFilename = std::string("Physics/") + mCfg.mModelPath;
        const std::string usdPath = getAssetDirectory(TestAssetDirectoryType::eDataRoot) +
            "/" + getAssetUriInDataSource(TestAssetType::eUsd, usdFilename.c_str());

        if (!mStage)
            mStage = UsdStage::Open(usdPath.c_str());
        // in subsequent runs, stage is reloaded in endRun

        mStageId = UsdUtilsStageCache::Get().Insert(mStage).ToLongInt();

        const SdfPath envsPath("/envs");
        const SdfLayerHandle layer = mStage->GetRootLayer();

        mStage->DefinePrim(envsPath);
        SdfPrimSpecHandle anymal = layer->GetPrimAtPath(mCfg.mModelPrimPath);

        USDOps::createXform(layer, envsPath);

        const SdfPath env0Path = envsPath.AppendChild(TfToken("env_0"));
        USDOps::createXform(layer, env0Path);

        USDOps::reparentJoints(anymal, SdfPath("/World"), SdfPath(env0Path));

        SdfBatchNamespaceEdit edit;
        edit.Add(SdfNamespaceEdit::Reparent(mCfg.mModelPrimPath, env0Path, 0));
        layer->Apply(edit);
    }

    void exportUsdStage(const std::string& path) { mStage->Export(path); }

    // clone the environment in USD, (not yet: replicate in PhysX)
    // exact number reached for perfect squares and powers of two, otherwise the number of envs will be higher
    void clone(uint32_t nbEnvs, float envSpacing, bool shallow)
    {
        uint32_t nx, ny;
        if ((nbEnvs & (nbEnvs - 1)) == 0) // power of two
        {
            nx = 1;
            for (uint32_t k = nbEnvs; k > 1; k >>= 2)
                nx <<= 1;
        }
        else
            nx = static_cast<uint32_t>(std::roundf(std::sqrt(nbEnvs)));
        ny = (nbEnvs + nx - 1) / nx;

        cloneUSD(mStage->GetRootLayer(), nx, ny, envSpacing, shallow);
    }

    // post:
    // - mRobots contains all robot articulations in the scene
    // - if setupCaches, then mArticulationCaches contains one cache for each robot articulation
    void setupArticulationVec(bool setupCaches)
    {
        mArticulationCaches.clear();
        mRobots.clear();

        // get articulations
        uint32_t nbArticulations = mPhysXScene->getNbArticulations();

        std::vector<PxArticulationReducedCoordinate *> sceneArticulations(nbArticulations);
        mPhysXScene->getArticulations(sceneArticulations.data(), static_cast<uint32_t>(sceneArticulations.size()));

        // filter articulations based on nb of dofs (this could be improved)
        for (PxArticulationReducedCoordinate* art: sceneArticulations)
            if (art->getDofs() == mNbRobotDofs)
                mRobots.push_back(art);

        if (setupCaches)
            for (auto art: mRobots)
                mArticulationCaches.push_back(art->createCache());
        return;
    }

    // must be called after first step
    // pre:
    //  - mRobots is filled with the scene's robot articulations (1 per env)
    //  - mPhysXScene is populated
    //  post:
    //  - mDirectGPUAPI is populated
    //  - mArtGPUIdx buffers are populated
    //  - mDriveTargetBufH is resized appropriately
    void initDirectGPUAPI()
    {
        assert(mRobots.size() == mCfg.mNbEnvs && "Unequal number of articulations vs. envs.");
        checkGPUAPI();
        mDirectGPUAPI = &mPhysXScene->getDirectGPUAPI();

        // 1. indices
        const size_t indexSize = sizeof(PxArticulationGPUIndex) * mRobots.size();

        mArtGPUIdx.allocate(mRobots.size());

        for (int i = 0; i < mRobots.size(); ++i)
            mArtGPUIdx.host[i] = mRobots[i]->getGPUIndex();

        mArtGPUIdx.copyHtoD();

        // 2. articulation data
        const int maxDofs = mDirectGPUAPI->getArticulationGPUAPIMaxCounts().maxDofs;

        mDriveTargetsGPU.allocate(mRobots.size() * maxDofs);
        mRootVec3.allocate(mRobots.size());
        mRootTransform.allocate(mRobots.size());
        mJointReal.allocate(mRobots.size()*maxDofs);
    }

    // - initialize the PhysX scene from the USD file
    // - initialize Direct GPU API
    // - initialize robot data for benchmarks
    // - reset joint positions
    void startRun() override // per-run initialization
    {

        const BmGlobals& bmGlobals = BmGlobals::getInstance();
        if (bmGlobals.numThreads() != -1)
            mPhysXBenchmarks->setThreadCount(bmGlobals.numThreads());
        else
            mPhysXBenchmarks->setThreadCount(tThreadCount);

        mPhysXBenchmarks->overwriteGPUSetting(bmGlobals.forceGpu() ? true : tDevice == ComputeDevice::GPU);

        if (bmGlobals.enableProfile())
            mPhysXBenchmarks->enablePVDProfile(true);

        prepareStage();

        const PXR_NS::SdfPath scenePath("/physicsScene");
        mPhysXScene = static_cast<::physx::PxScene*>(mPhysX->getPhysXPtr(scenePath, ePTScene));
        if (mPhysXScene == nullptr)
            CARB_LOG_ERROR("Physics Scene not found");

        mT = 0.0;
        mStep = 0;

        setupArticulationVec(tDevice != ComputeDevice::GPU);

        // initial step
        simStep();

        if (tDevice == ComputeDevice::GPU)
        {
            initDirectGPUAPI();
            mRobotData = std::make_unique<GPURobotData>(
                    mCfg.mNbEnvs, mNbRobotDofs, mDirectGPUAPI, &mArtGPUIdx, this->mPhysXScene);
        } else {
            mRobotData = std::make_unique<CPURobotData>(
                    mCfg.mNbEnvs, mNbRobotDofs, mRobots, this->mPhysXScene);
        }

        getDefaultJointDriveTargets();

        resetJointPositions(); // TODO(CA): reset velocities (and positions)
        if (gRecordCanned && mRunIdx == 0 && mCfg.mPolicyType != PolicyType::NONE) // record canned policy
        {
            std::cout << "Starting to record to " << getPolicyPath(true) << "\n";
            mPolicyRecording = std::make_unique<CannedPolicy>(getPolicyPath(true), mCfg.mNbSteps, mCfg.mNbEnvs * mNbRobotDofs);
        }
    }

    void resetJointPositions() { tDevice == ComputeDevice::GPU ? resetJointPositionsGPU() : resetJointPositionsCPU(); }
    void resetJointPositionsCPU()
    {
        for (int i = 0; i < mArticulationCaches.size(); ++i)
        {
            auto art = mRobots[i];
            auto cache = mArticulationCaches[i];
            art->copyInternalStateToCache(*cache, PxArticulationCacheFlag::ePOSITION);
            const size_t nbDofs = art->getDofs();
            for (int dof = 0; dof < nbDofs; ++dof)
                cache->jointPosition[dof] = mDofDefaults[dof];
            art->applyCache(*cache, PxArticulationCacheFlag::ePOSITION);
        }
    }
    void resetJointPositionsGPU()
    {
        const int maxDofs = mDirectGPUAPI->getArticulationGPUAPIMaxCounts().maxDofs;
        // use drive targets buffer since equal size
        for (int i = 0; i < mRobots.size(); ++i)
            for (int dof = 0; dof < mNbRobotDofs; ++dof)
                mDriveTargetsGPU.host[i*maxDofs+dof] = mDofDefaults[dof];
        mDriveTargetsGPU.copyHtoD();
        mDirectGPUAPI->setArticulationData(
            (void*) mDriveTargetsGPU.device,
            (PxArticulationGPUIndex*) mArtGPUIdx.device,
            PxArticulationGPUAPIWriteType::eJOINT_POSITION,
            static_cast<uint32_t>(mRobots.size())
        );
    }

    void endRun() override // per run tear-down
    {
        if (mPolicyRecording)
        {
            std::cout << "Dumping recording." << "\n";

            mPolicyRecording->dump();
            mPolicyRecording.reset();
        }
        ++mRunIdx;
        freeBuffers();
        if (gDumpSceneStartEnd && mRunIdx == 0)
        {
            mPhysX->updateTransformations(false, true, true, false);
            exportUsdStage(ExportEndUSD);
            CARB_LOG_INFO("Final stage written to %s", ExportEndUSD.c_str());
        }

        mPhysXSimulation->detachStage();
        mStage->Reload(); // reload stage to clear changes
    }

    uint32_t getNbRuns() const { return 3; }
    uint32_t getNbSteps() const { return mCfg.mNbSteps; }

    void getDefaultJointDriveTargets() { tDevice == ComputeDevice::GPU ? getDefaultJointDriveTargetsGPU() : getDefaultJointDriveTargetsCPU(); }

    // get default joint targets
    void getDefaultJointDriveTargetsCPU()
    {
        assert (mRobots.size() > 0);
        const PxArticulationReducedCoordinate* art = mRobots[0];
        PxArticulationCache* artCache = art->createCache();
        art->copyInternalStateToCache(*artCache, PxArticulationCacheFlag::eJOINT_TARGET_POSITIONS);
        const size_t nbDofs = art->getDofs();

        mDofDefaults.resize(nbDofs);
        for (int i = 0; i < nbDofs; ++i)
            mDofDefaults[i] = artCache->jointTargetPositions[i];
        // artCache->release();  // TODO: this gives a segfault
    }

    // get default joint targets
    void getDefaultJointDriveTargetsGPU()
    {
        mDirectGPUAPI->getArticulationData(
            (void*) mDriveTargetsGPU.device,
            (PxArticulationGPUIndex*) mArtGPUIdx.device,
            PxArticulationGPUAPIReadType::eJOINT_TARGET_POSITION,
            1
        );
        mDriveTargetsGPU.copyDtoH();
        mDofDefaults.resize(mNbRobotDofs);
        std::memcpy(mDofDefaults.data(), mDriveTargetsGPU.host.data(), mNbRobotDofs*sizeof(float));
    }

    void setJointDriveTargets() {
        const int nbObsTotal = mCfg.mNbEnvs*mNbRobotDofs;
        if (mActions.size() != nbObsTotal)
        {
            mActions.clear();
            mActions.resize(nbObsTotal);
        }
        computeObservations(mCfg.mNbEnvs);

        tDevice == ComputeDevice::GPU ? setJointDriveTargetsGPU() : setJointDriveTargetsCPU();
    }

    void setJointDriveTargetsCPU()
    {
        for (int i = 0; i < mArticulationCaches.size(); ++i)
        {
            auto art = mRobots[i];
            auto cache = mArticulationCaches[i];
            art->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eJOINT_TARGET_POSITIONS);
            const uint32_t nbDofs = art->getDofs();
            sampleJointDriveTargets(cache->jointTargetPositions, nbDofs, 1, i);
            art->applyCache(*cache, PxArticulationCacheFlag::eJOINT_TARGET_POSITIONS);
        }
    }
    void setJointDriveTargetsGPU()
    {
        // all articulations should have the same size
        const int maxDofs = mDirectGPUAPI->getArticulationGPUAPIMaxCounts().maxDofs;
        sampleJointDriveTargets(mDriveTargetsGPU.host.data(), mNbRobotDofs, static_cast<uint32_t>(mRobots.size()), 0);
        mDriveTargetsGPU.copyHtoD();
        assert(mDriveTargetsGPU.host.size() == mNbRobotDofs * mCfg.mNbEnvs);
        assert(mArtGPUIdx.host.size() == mCfg.mNbEnvs);
        assert(mRobots.size() == mCfg.mNbEnvs);
        mDirectGPUAPI->setArticulationData(
            (void*) mDriveTargetsGPU.device,
            (PxArticulationGPUIndex*) mArtGPUIdx.device,
            PxArticulationGPUAPIWriteType::eJOINT_TARGET_POSITION,
            static_cast<uint32_t>(mRobots.size())
        );
    }

    // get the path for the policy
    // extended name is used for canned policies, as they are specific to a certain benchmark setup
    std::string getPolicyPath(bool useExtendedName) {
        std::string name = mCfg.mPolicyName;
        if (useExtendedName)
            name += "-envs" + std::to_string(mCfg.mNbEnvs) + "-st" + std::to_string(mCfg.mNbSteps) + "-sp" + std::to_string(mCfg.mEnvSpacing);
        const std::string path = getAssetDirectory(TestAssetDirectoryType::eDataRoot) + "/models/tests/Physics/cbm-policy/" + name;
        return path;
    }

    // attempt to load the policy at mPolicyName
    void loadPolicy()
    {
        if (mCfg.mPolicyType == PolicyType::NONE)
            return;

        assert(!mCfg.mPolicyName.empty() && "No policy name given.");
        bool mlirFailed = false;
        if (mCfg.mPolicyType == PolicyType::MLIR)
        {
            const std::string path = getPolicyPath(false);
            mPolicyMLIR = std::make_unique<MLIRPolicy>(path.c_str(), Shape{1, mCfg.mNbObservations});
            if (*mPolicyMLIR) // success
                return;
            // attempt to load canned policy
            mPolicyMLIR.reset();
            mlirFailed = true;
            std::cout << "Failed to load MLIR policy at " << path.c_str() << ". Falling back to canned policy." << std::endl;
            if (mPolicyRecording)
            {
                std::cerr << "Cannot record canned policy." << std::endl;
                mPolicyRecording.reset();
            }
        }
        if (mCfg.mPolicyType == PolicyType::CANNED || mlirFailed)
        {
            std::string path = getPolicyPath(true);
            mPolicyCanned = std::make_unique<CannedPolicy>(path);
            if (*mPolicyCanned) // success
                return;
            std::cout << "Failed to load canned policy at " << path.c_str() << "." << std::endl;
            mPolicyCanned.reset();
        }
    }

    // get drive targets from policy
    // dofs(output): joint drive target array
    // nbDofs(input): number of dofs to sample
    // nbArticulations(input): number of articulations to sample dofs for
    // artIdx(input): index of the first articulation (rest assumed to be contiguous)
    void getPolicyDriveTargets(float* dofs, int nbDofs, int nbArticulations, int artIdx)
    {
        if (mPolicyCanned)
            return getCannedDriveTargets(dofs, nbDofs, nbArticulations, artIdx);
        else if (!mPolicyMLIR)
            return;

        float* policyInput = mPolicyMLIR->inputData();
        const int nbObservations = mPolicyMLIR->inputShape();

        for (int i = 0; i < nbArticulations; ++i)  // TODO(CA): batching
        {
            int artGlobalIdx = artIdx + i;
            // fill policy input vector
            for(int obs = 0; obs < nbObservations; ++obs)
                policyInput[obs] = mObservations[nbObservations*artGlobalIdx+obs];

            // inference
            mPolicyMLIR->forward();

            for(int obs = 0; obs < nbObservations; ++obs)
            {
                const float k = mObservations[nbObservations*artGlobalIdx+obs];
                if (std::isnan(k) or std::abs(k) > 1e6)
                    std::cout << "Invalid observation! articulation = " << artGlobalIdx << ", obs = " << obs << "\n";
            }

            for(int dof = 0; dof < nbDofs; ++dof)
            {
                const float k = *(dofs+nbDofs*i + dof);
                if (std::isnan(k) or std::abs(k) > 1e6)
                    std::cout << "Invalid action! articulation = " << artGlobalIdx << ", dof = " << dof << "\n";
            }

            std::memcpy(&mActions[artGlobalIdx*nbDofs], mPolicyMLIR->outputData(), nbDofs*sizeof(float));

            applyPolicyActions(dofs+nbDofs*i, this->mPolicyMLIR->outputData(), 1, artGlobalIdx);
            if (mPolicyRecording)
                mPolicyRecording->record(dofs+nbDofs*i, nbDofs);
        }
    }

    void getCannedDriveTargets(float* dofs, int nbDofs, int nbArticulations, int artIdx)
    {
        if (mStep < 2)
            CARB_LOG_ERROR("Warning: mStep < 2: %d", mStep);

        const int step = this->mStep - 2;
        const int cannedEnvDofs = mPolicyCanned->mNbValuesPerStep;  // dofs per step in canned env

        if (cannedEnvDofs/nbDofs < nbArticulations)
            CARB_LOG_WARN("Canned drive targets with fewer dofs/step than benchmark.");

        for (int i = 0; i < nbArticulations; ++i)
        {
            int artGlobalIdx = artIdx + i;
            std::memcpy(
                    &dofs[i*nbDofs],
                    &mPolicyCanned->vector()[step*cannedEnvDofs+artGlobalIdx*nbDofs],
                    nbDofs*sizeof(float));
        }
    }

    static void cloneUSD(SdfLayerHandle layer, uint32_t dim0, uint32_t dim1, float offset, bool shallow)
    {
        const uint32_t nbEnvs = dim0*dim1;

        SdfChangeBlock changeBlock;
        const SdfPath envPath("/envs");
        const SdfPath sourceEnvPath("/envs/env_0");

        for (uint32_t i = 1; i < nbEnvs; i++)
        {
            // copy env
            const uint32_t row = i / dim1;
            const uint32_t col = i % dim1;
            const double x = row * offset;
            const double y = col * offset;
            const GfVec3d offsetVec(x, y, 0.0);

            const std::string curEnvName = "env_" + std::to_string(i);
            const SdfPath curEnvPath = envPath.AppendChild(TfToken(curEnvName));

            SdfPrimSpecHandle primSpec = SdfCreatePrimInLayer(layer, curEnvPath);

            if (shallow)
                USDOps::createXform(layer, curEnvPath);
            else
                SdfCopySpec(layer, sourceEnvPath, layer, curEnvPath);

            static TfToken gTranslate("xformOp:translate");
            const PXR_NS::SdfPath attributePath = curEnvPath.AppendProperty(gTranslate);
            SdfAttributeSpecHandle posAttr = primSpec->GetAttributeAtPath(attributePath);
            const GfVec3d currentPos = posAttr->GetDefaultValue().UncheckedGet<GfVec3d>();
            posAttr->SetDefaultValue(VtValue(currentPos + offsetVec));
        }
    }

protected:
    void step() override { simStep(); }

    std::vector<PxArticulationReducedCoordinate*> mRobots;

    std::vector<PxArticulationCache*> mArticulationCaches;

    std::vector<PxArticulationLimit> mDofLimits; // limits for each dof, unused
    std::vector<float> mDofDefaults; // default value for each dof

    PxDirectGPUAPI* mDirectGPUAPI = nullptr;
    int mStep = 0;

    CBMCfg mCfg;
    int mNbRobotDofs = 0;

    omni::physx::IPhysx* mPhysX = nullptr;
    omni::physx::IPhysxBenchmarks*  mPhysXBenchmarks = nullptr;
    physx::PxScene* mPhysXScene = nullptr;
    omni::physx::IPhysxSimulation* mPhysXSimulation = nullptr;

    const char* const* mArgv = nullptr;
    size_t mArgc = 0;


    // Internal state
    float mT = 0;
    int mRunIdx = 0;

    PXR_NS::UsdStageRefPtr mStage;
    long mStageId = 0;

    // joint drive target buffer for Direct GPU API
    DirectGPUBuffer<float> mDriveTargetsGPU;

    // Root observations
    DirectGPUBuffer<PxVec3> mRootVec3;
    DirectGPUBuffer<PxTransform> mRootTransform;

    // scalar per-joint property
    DirectGPUBuffer<float> mJointReal;

    // link pose buffer
    DirectGPUBuffer<PxTransform> mLinkPosesGPU;

    // articulation GPU index buffer for Direct GPU API
    DirectGPUBuffer<PxArticulationGPUIndex> mArtGPUIdx;

    std::vector<float> mObservations;
    std::vector<float> mActions;

    // policies (canned or MLIR)
    std::unique_ptr<CannedPolicy> mPolicyCanned;
    std::unique_ptr<MLIRPolicy> mPolicyMLIR;

    // only defined when recording is taking place
    std::unique_ptr<CannedPolicy> mPolicyRecording;

    // robot data
    std::unique_ptr<RobotData> mRobotData;

    std::unique_ptr<SettingsGuard> mSettingsGuard;
}; // class PhysicsCompetitiveBenchmark


CBMCfg AnymalRandomCfg()
{
    CBMCfg cfg;
    cfg.mNbEnvs = 16;
    cfg.mNbDofsPerEnv = 12;
    cfg.mModelPath = "AnymalMenagerieCollidersOnly.usda";
    cfg.mModelPrimPath = SdfPath("/World/anymal_c_mjx");
    return cfg;
}

template <ComputeDevice tDevice, uint32_t tThreads, uint32_t tEnvs, Separation tSpacing>
class AnymalRandBM : public PhysicsCompetitiveBenchmark<tDevice, tThreads>
{
public:
    AnymalRandBM() :
        PhysicsCompetitiveBenchmark<tDevice, tThreads>(AnymalRandomCfg(), tEnvs, tSpacing),
        mRng(mSeed), action_dist(-0.5f, 0.5f) { }
private:
    void sampleJointDriveTargets(float* dofs, int nbDofs, int nbArticulations, int artIdx) override
    {
        assert (nbDofs == 12);
        for (int i = 0; i < nbArticulations; ++i)
            for (int j = 0; j < nbDofs; ++j)
                dofs[i * nbDofs + j] = action_dist(mRng) + this->mDofDefaults[j];
    }

    uint32_t mSeed = 123;
    std::mt19937_64 mRng;
    std::uniform_real_distribution<float> action_dist;
};

CBMCfg HumanoidWalkCfg()
{
    CBMCfg cfg;
    cfg.mEnvSpacing = 8;
    cfg.mNbEnvs = 16;
    cfg.mNbSteps = 200;

    cfg.mModelPath = "H1/h1.usd";
    cfg.mModelPrimPath = SdfPath("/World/h1");
    cfg.mNbDofsPerEnv = 19;
    cfg.mNbObservations = 69;
    cfg.mPolicyType = PolicyType::MLIR;
    cfg.mPolicyName = "h1_walk";
    return cfg;
}


template <ComputeDevice tDevice, uint32_t tThreads, uint32_t tEnvs, Separation tSpacing>
class H1WalkBM : public PhysicsCompetitiveBenchmark<tDevice, tThreads>
{
public:
    H1WalkBM() : PhysicsCompetitiveBenchmark<tDevice, tThreads>(HumanoidWalkCfg(), tEnvs, tSpacing) { }

private:
    void computeObservations(int nbArticulations) override
    {
        this->mRobotData->readState();

        const int nbDofs = this->mNbRobotDofs;
        const int nbObservations = this->mCfg.mNbObservations;
        assert(nbObservations == 69);

        this->mObservations.resize(nbObservations*nbArticulations);

        // root velocities, projected gravity & command
        const std::vector<PxVec3>& linVel = this->mRobotData->rootLinearVelocity();
        const std::vector<PxVec3>& angVel = this->mRobotData->rootAngularVelocity();
        const std::vector<PxVec3>& projGrav = this->mRobotData->rootProjectedGravity();

        float* obs = this->mObservations.data();

        const float command[] = {0, 0, 0};

        int offset = 0;
        for (int art = 0; art < nbArticulations; ++art)
        {
            std::memcpy(obs+art*nbObservations+offset,   &linVel[art].x,   3*sizeof(float));
            std::memcpy(obs+art*nbObservations+offset+3, &angVel[art].x,   3*sizeof(float));
            std::memcpy(obs+art*nbObservations+offset+6, &projGrav[art].x, 3*sizeof(float));
            std::memcpy(obs+art*nbObservations+offset+9, command,          3*sizeof(float));
        }
        offset += 12;

        // joint positions & joint velocities
        const std::vector<float>& jointPos = this->mRobotData->jointPosition();
        const std::vector<float>& jointVel = this->mRobotData->jointVelocity();
        for (int art = 0; art < nbArticulations; ++art)
        {
            for (int dof = 0; dof < nbDofs; ++dof)
                obs[art*nbObservations+offset+dof] = jointPos[art*nbDofs+dof] - this->mDofDefaults[dof];
            std::memcpy(obs+art*nbObservations+offset+nbDofs, &jointVel[art*nbDofs], nbDofs*sizeof(float));
        }
        offset += 2*nbDofs;

        for (int art = 0; art < nbArticulations; ++art)
            for (int dof = 0; dof < nbDofs; ++dof)
                obs[art*nbObservations+offset+dof] = this->mActions[art*nbDofs+dof];
    }

    void applyPolicyActions(float* dofs, const float* policyOutput, int nbArticulations, int artIdx) override
    {
        // write dofs
        const int nbDofs = this->mNbRobotDofs;
        for (int i = 0 ; i < nbArticulations; ++i)
            for (int dof = 0; dof < nbDofs; ++dof)
                dofs[i*nbDofs+dof] = 0.4 * policyOutput[i*nbDofs+dof]+this->mDofDefaults[dof];
    }
};

CBMCfg RoboKitchenCfg()
{
    CBMCfg cfg;
    cfg.mCreateGroundPlane = false;
    cfg.mNbEnvs = 4;
    cfg.mEnvSpacing = 8;
    cfg.mNbDofsPerEnv = 19;
    cfg.mNbObservations = 69;

    cfg.mNbSteps = 300;
    cfg.mModelPath = "RobocasaKitchen/KitchenWithRobot.usd";
    cfg.mModelPrimPath = SdfPath("/World/env");
    cfg.mPolicyType = PolicyType::MLIR;
    cfg.mPolicyName = "h1_walk";

    return cfg;
}

template <ComputeDevice tDevice, uint32_t tThreads, uint32_t tEnvs, Separation tSpacing>
class H1KitchenBM : public PhysicsCompetitiveBenchmark<tDevice, tThreads>
{
public:
    H1KitchenBM() : PhysicsCompetitiveBenchmark<tDevice, tThreads>(RoboKitchenCfg(), tEnvs, tSpacing) { }

private:
    void computeObservations(int nbArticulations) override
    {
        this->mRobotData->readState();

        const int nbDofs = this->mNbRobotDofs;
        const int nbObservations = this->mCfg.mNbObservations;

        this->mObservations.resize(nbObservations*nbArticulations);

        // root velocities, projected gravity & command
        const std::vector<PxVec3>& linVel = this->mRobotData->rootLinearVelocity();
        const std::vector<PxVec3>& angVel = this->mRobotData->rootAngularVelocity();
        const std::vector<PxVec3>& projGrav = this->mRobotData->rootProjectedGravity();

        float* obs = this->mObservations.data();

        const float command[] = {1.0, 0, 0};

        int offset = 0;
        for (int art = 0; art < nbArticulations; ++art)
        {
            std::memcpy(obs+art*nbObservations+offset,   &linVel[art].x,   3*sizeof(float));
            std::memcpy(obs+art*nbObservations+offset+3, &angVel[art].x,   3*sizeof(float));
            std::memcpy(obs+art*nbObservations+offset+6, &projGrav[art].x, 3*sizeof(float));
            std::memcpy(obs+art*nbObservations+offset+9, command,          3*sizeof(float));
        }
        offset += 12;

        // joint positions & joint velocities
        const std::vector<float>& jointPos = this->mRobotData->jointPosition();
        const std::vector<float>& jointVel = this->mRobotData->jointVelocity();
        for (int art = 0; art < nbArticulations; ++art)
        {
            for (int dof = 0; dof < nbDofs; ++dof)
                obs[art*nbObservations+offset+dof] = jointPos[art*nbDofs+dof] - this->mDofDefaults[dof];
            std::memcpy(obs+art*nbObservations+offset+nbDofs, &jointVel[art*nbDofs], nbDofs*sizeof(float));
        }
        offset += 2*nbDofs;

        for (int art = 0; art < nbArticulations; ++art)
            for (int dof = 0; dof < nbDofs; ++dof)
                obs[art*nbObservations+offset+dof] = this->mActions[art*nbDofs+dof];
    }

    void applyPolicyActions(float* dofs, const float* policyOutput, int nbArticulations, int artIdx) override
    {
        // write dofs
        const int nbDofs = this->mNbRobotDofs;
        for (int i = 0 ; i < nbArticulations; ++i)
        {
            for (int dof = 0; dof < nbDofs; ++dof)
                dofs[i*nbDofs+dof] = 0.4 * policyOutput[i*nbDofs+dof]+this->mDofDefaults[dof];
            dofs[i*nbDofs+9] = 1.30;
            dofs[i*nbDofs+10] = -1.30;
        }
    }
};

using Dev = ComputeDevice;
using Sep = Separation;

#ifndef _WIN32  // The RL-focused competitive benchmarks are disabled on Windows for now
// Anymal random motion benchmark
Register<AnymalRandBM<Dev::CPU, 8, 64, Sep::AREA  >> AnymalRand_64E_SPAC_CPU("CBM.Anymal_Random.64Envs.SPAC.CPU");
Register<AnymalRandBM<Dev::GPU, 8, 64, Sep::AREA  >> AnymalRand_64E_SPAC_GPU("CBM.Anymal_Random.64Envs.SPAC.GPU");
Register<AnymalRandBM<Dev::GPU, 8, 64, Sep::ENVIDS>> AnymalRand_64E_COLO_GPU("CBM.Anymal_Random.64Envs.COLO.GPU");

// Too slow
// Register<AnymalRandBM<Dev::CPU, 8, 16384, Sep::AREA  >> AnymalRand_16kE_SPAC_CPU("CBM.Anymal_Random.16384Envs.SPAC.CPU");
Register<AnymalRandBM<Dev::GPU, 8, 16384, Sep::AREA  >> AnymalRand_16kE_SPAC_GPU("CBM.Anymal_Random.16384Envs.SPAC.GPU");
Register<AnymalRandBM<Dev::GPU, 8, 16384, Sep::ENVIDS>> AnymalRand_16kE_COLO_GPU("CBM.Anymal_Random.16384Envs.COLO.GPU");

// Humanoid plain walk
Register<H1WalkBM<Dev::CPU, 8, 64, Sep::AREA  >> H1Walk_64E_SPAC_CPU("CBM.H1_Walk.64Envs.SPAC.CPU");
Register<H1WalkBM<Dev::GPU, 8, 64, Sep::AREA  >> H1Walk_64E_SPAC_GPU("CBM.H1_Walk.64Envs.SPAC.GPU");
Register<H1WalkBM<Dev::GPU, 8, 64, Sep::ENVIDS>> H1Walk_64E_COLO_GPU("CBM.H1_Walk.64Envs.COLO.GPU");

// Too slow
// Register<H1WalkBM<Dev::CPU, 8, 16384, Sep::AREA  >> H1Walk_16kE_SPAC_CPU("CBM.H1_Walk.16384Envs.SPAC.CPU");
Register<H1WalkBM<Dev::GPU, 8, 16384, Sep::AREA  >> H1Walk_16kE_SPAC_GPU("CBM.H1_Walk.16384Envs.SPAC.GPU");
Register<H1WalkBM<Dev::GPU, 8, 16384, Sep::ENVIDS>> H1Walk_16kE_COLO_GPU("CBM.H1_Walk.16384Envs.COLO.GPU");

// Humanoid Kitchen walk
Register<H1KitchenBM<Dev::CPU, 8, 8, Sep::AREA  >> H1Kitchen_8E_SPAC_CPU("CBM.H1_Kitchen.8Envs.SPAC.CPU");
Register<H1KitchenBM<Dev::GPU, 8, 8, Sep::AREA  >> H1Kitchen_8E_SPAC_GPU("CBM.H1_Kitchen.8Envs.SPAC.GPU");
Register<H1KitchenBM<Dev::GPU, 8, 8, Sep::ENVIDS>> H1Kitchen_8E_COLO_GPU("CBM.H1_Kitchen.8Envs.COLO.GPU");

Register<H1KitchenBM<Dev::CPU, 8, 64, Sep::AREA  >> H1Kitchen_64E_SPAC_CPU("CBM.H1_Kitchen.64Envs.SPAC.CPU");
Register<H1KitchenBM<Dev::GPU, 8, 64, Sep::AREA  >> H1Kitchen_64E_SPAC_GPU("CBM.H1_Kitchen.64Envs.SPAC.GPU");
Register<H1KitchenBM<Dev::GPU, 8, 64, Sep::ENVIDS>> H1Kitchen_64E_COLO_GPU("CBM.H1_Kitchen.64Envs.COLO.GPU");
#endif
