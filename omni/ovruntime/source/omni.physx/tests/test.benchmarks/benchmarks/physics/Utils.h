// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#ifndef PHYSICS_CBM_UTILS_H
#define PHYSICS_CBM_UTILS_H

#include <iomanip>
#include <string>
#include <cmath>
#include <cuda.h>

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxTests.h>
#include <omni/physx/IPhysxSettings.h>

#include <PxPhysicsAPI.h>

using namespace carb;
using namespace PXR_NS;
using namespace physx;

// class for handling a file with "canned", i.e. precomputed joint targets
// supports reading from disk (no writing)
class CannedPolicy
{
    // format:
    // First line: CannedPolicy v<version number: uint32> <number of steps: uint32> <values per step: uint32>
    // The rest of the file are whitespace-separated floating-point values
    // Values are simply produced in order. Whitespace can be used for better human readability
public:
    // initialize playback
    CannedPolicy(const std::string& path)
        : mRecording(false), mPath(path)
    {
        std::ifstream file(path);
        if(!file.is_open())
        {
            loaded = false;
            return;
        }
        loaded = true;
        // read header
        std::string magicNumber;
        file >> magicNumber;
        CARB_ASSERT (magicNumber.compare("CannedPolicy") == 0);
        char vee;
        uint32_t version;
        file >> vee;
        file >> version;
        CARB_ASSERT (vee == 'v' && version == 0);
        file >> mNbSteps;
        file >> mNbValuesPerStep;

        for (uint32_t i = 0; i < mNbSteps*mNbValuesPerStep; ++i)
        {
            float val;
            file >> val;
            mData.push_back(val);
        }
    }

    // initialize recording
    CannedPolicy(const std::string& path, uint32_t numSteps, uint32_t valuesPerStep)
        : mPath(path), mRecording(true), mNbSteps(numSteps), mNbValuesPerStep(valuesPerStep), loaded(true)
    { }

    operator bool(){ return loaded; }

    void record(const float* data, int n)
    {
        // write header
        for (int i = 0; i < n; ++i)
            mData.push_back(data[i]);
    }

    void dump()
    {
        std::cout << "Dumping to file " << mPath<< "\n";
        CARB_ASSERT(mData.size() == mNbSteps*mNbValuesPerStep);
        std::ofstream file(mPath, std::ios_base::out|std::ios_base::trunc);
        file << std::setprecision(12);
        // write header
        file << "CannedPolicy v0 " << mNbSteps << " " << mNbValuesPerStep << "\n";

        for (uint32_t step = 0; step < mNbSteps; ++step)
            for (uint32_t dof = 0; dof < mNbValuesPerStep; ++dof)
                file << mData[step*mNbValuesPerStep+dof] << ((dof+1)%mNbValuesPerStep == 0 ? "\n" : " ");
    }

    std::vector<float>& vector() { return mData; }
    const std::vector<float>& vector() const { return mData; }

    uint32_t mNbSteps = 0, mNbValuesPerStep = 0;
private:
    bool loaded;
    const bool mRecording = false;
    const std::string mPath;
    std::vector<float> mData;
};

// Guard for settings. Allows overwriting carb settings with a new value, and resets them to the overwritten
// value when it is destructed.
class SettingsGuard
{
public:
    SettingsGuard() = default;
    SettingsGuard(carb::settings::ISettings* settingsInterface) : mSettingsInterface(settingsInterface) { }
    ~SettingsGuard() { restore(); }

    // restore all overwritten settings to their previous states
    void restore() {
        restore(mBoolSettings);
        restore(mIntSettings);
        restoreStrings(mStringSettings);
    }

    template <typename T>
    void restore(const std::vector<std::pair<std::string, T>>& values)
    {
        for (auto pair_it = values.rbegin(); pair_it < values.rend(); ++pair_it)
            mSettingsInterface->set<T>(pair_it->first.c_str(), pair_it->second);
    }

    void restoreStrings(const std::vector<std::pair<std::string, std::string>>& values)
    {
        for (auto pair_it = values.rbegin(); pair_it < values.rend(); ++pair_it)
            mSettingsInterface->setString(pair_it->first.c_str(), pair_it->second.c_str());
    }

    // register bool setting to restore, and set setting to value
    void addBool(const char* setting)
    {
        mBoolSettings.emplace_back(setting, mSettingsInterface->getAsBool(setting));
    }
    // register bool setting to restore, and set setting to value
    void addBool(const char* setting, bool value)
    {
        addBool(setting);
        mSettingsInterface->setBool(setting, value);
    }

    // register int setting to restore, and set setting to value
    void addInt(const char* setting)
    {
        mIntSettings.emplace_back(setting, mSettingsInterface->getAsInt(setting));
    }
    // register int setting to restore, and set setting to value
    void addInt(const char* setting, int value)
    {
        addInt(setting);
        mSettingsInterface->setInt(setting, value);
    }

    // register string setting to restore, and set setting to value
    void addString(const char* setting)
    {
        // does not handle NUL-containing strings
        mStringSettings.emplace_back(setting, mSettingsInterface->getStringBuffer(setting));
    }
    // register string setting to restore, and set setting to value
    void addString(const char* setting, const char* value)
    {
        addString(setting);
        mSettingsInterface->setString(setting, value);
    }

protected:
    carb::settings::ISettings* mSettingsInterface;
    std::vector<std::pair<std::string, bool>> mBoolSettings;
    std::vector<std::pair<std::string, int>> mIntSettings;
    std::vector<std::pair<std::string, std::string>> mStringSettings;
};

namespace USDOps
{
    void repathRelationship(SdfRelationshipSpec relationship, const SdfPath& from, const SdfPath& to)
    {
        if (!relationship.HasTargetPathList())
            return;

        SdfTargetsProxy pathProxy = relationship.GetTargetPathList();
        // assume only one path
        auto targets = pathProxy.GetAddedOrExplicitItems();
        if (targets.begin() == targets.end())
            return;

        const SdfPath oldPath = *targets.begin();
        const SdfPath newPath = oldPath.ReplacePrefix(from, to);
        relationship.ReplaceTargetPath(oldPath, newPath);
    }

    // traverse a prim's children, repathing all joints
    void reparentJoints(SdfPrimSpecHandle& prim, const SdfPath& from, const SdfPath& to)
    {
        // consider a block update here
        std::vector<SdfPrimSpecHandle> pathsToVisit = {prim};
        while (pathsToVisit.size())
        {
            SdfPrimSpecHandle p = pathsToVisit.back();
            pathsToVisit.pop_back();
            // append children
            for (SdfPrimSpecHandle child: p->GetNameChildren())
                pathsToVisit.push_back(child);
            // repath joints
            if (p->GetTypeName() == TfToken("PhysicsRevoluteJoint")
             || p->GetTypeName() == TfToken("PhysicsFixedJoint")
             || p->GetTypeName() == TfToken("PhysicsJoint")
             || p->GetTypeName() == TfToken("PhysicsPrismaticJoint")
             )
            {
                for (auto r: p->GetRelationships())
                    repathRelationship(r.GetSpec(), from, to);
            }
        }
    }

    void createXform(SdfLayerHandle layer, const SdfPath& path)
    {
        SdfPrimSpecHandle xformPrimSpec = SdfCreatePrimInLayer(layer, path);
        xformPrimSpec->SetTypeName("Xform");
        xformPrimSpec->SetSpecifier(SdfSpecifierDef);

        SdfAttributeSpecHandle translate = SdfAttributeSpec::New(
                xformPrimSpec, "xformOp:translate", SdfValueTypeNames->Double3, SdfVariabilityVarying);
        translate->SetDefaultValue(VtValue(GfVec3d(0, 0, 0)));

        SdfAttributeSpecHandle xformOpOrder = SdfAttributeSpec::New(
                xformPrimSpec, "xformOpOrder", SdfValueTypeNames->TokenArray, SdfVariabilityUniform);
        VtArray<TfToken> xformOpOrderArray;
        xformOpOrderArray.push_back(TfToken("xformOp:translate"));
        xformOpOrder->SetDefaultValue(VtValue(xformOpOrderArray));
    }
};


template <typename T>
class DirectGPUBuffer
{
// The following conditions are equivalent:
// * the host vector is empty
// * the device buffer is not allocated
// * device is 0
// * nbElem is 0
// * nbBytes is 0

public:
    DirectGPUBuffer() = default;
    ~DirectGPUBuffer() { if (isAllocated()) free(); }
    DirectGPUBuffer(DirectGPUBuffer& other) = delete;
    DirectGPUBuffer& operator=(DirectGPUBuffer& other) = delete;

    void allocate(size_t nbElements)
    {
        assert(!isAllocated());
        nbElem = nbElements;
        nbBytes = sizeof(T) * nbElem;

        cuMemAlloc(&device, nbBytes);
        host.resize(nbElem);
    }

    void free()
    {
        assert(isAllocated());
        host.clear();
        cuMemFree(device);
        device = 0;
        nbElem = 0;
        nbBytes = 0;
    }

    bool isAllocated() const
    {
        CARB_ASSERT(bool(device) == bool(nbElem));
        CARB_ASSERT(nbBytes == sizeof(T) * nbElem);
        CARB_ASSERT(nbElem == host.size());
        return nbElem != 0;
    }

    // copy the entire buffer to the device
    void copyHtoD() { CARB_ASSERT(isAllocated()); cuMemcpyHtoD(device, static_cast<void*>(host.data()), nbBytes); }

    // copy the entire buffer to the host
    void copyDtoH() { CARB_ASSERT(isAllocated()); cuMemcpyDtoH(static_cast<void*>(host.data()), device, nbBytes); }

    std::vector<T> host;
    CUdeviceptr device = 0;

private:
    size_t nbElem = 0;
    size_t nbBytes = 0;
};

// robots = articulations
// all robots must have the same number of dofs (identical envs)
class RobotData
{
public:
    RobotData(int nbRobots, int nbDofs, const PxScene* scene): mNbDofs(nbDofs), mNbRobots(nbRobots)
    {
        mGravityDirection = scene->getGravity().getNormalized();
    }

    virtual void readState() { }

    virtual const std::vector<PxVec3>& rootLinearVelocity() const = 0;
    virtual const std::vector<PxVec3>& rootAngularVelocity() const = 0;
    virtual const std::vector<PxVec3>& rootProjectedGravity() const = 0;

    virtual const std::vector<float>& jointPosition() const = 0;
    virtual const std::vector<float>& jointVelocity() const = 0;

protected:

    int mNbDofs = 0;
    int mNbRobots = 0;
    PxVec3 mGravityDirection;
};

class CPURobotData: public RobotData
{
public:
    CPURobotData(int nbRobots, int nbDofs, std::vector<PxArticulationReducedCoordinate*> articulations, const PxScene* scene) :
        RobotData(nbRobots, nbDofs, scene), mArticulations(articulations)
    {
        mRootTransform.resize(mNbRobots);
        mRootLinearVelocity.resize(mNbRobots);
        mRootAngularVelocity.resize(mNbRobots);
        mRootProjectedGravity.resize(mNbRobots);

        mJointPosition.resize(mNbRobots*mNbDofs);
        mJointVelocity.resize(mNbRobots*mNbDofs);
    }
    const std::vector<PxVec3>& rootLinearVelocity() const override { return mRootLinearVelocity; }
    const std::vector<PxVec3>& rootAngularVelocity() const override { return mRootAngularVelocity; }
    const std::vector<PxVec3>& rootProjectedGravity() const override { return mRootProjectedGravity; }

    const std::vector<float>& jointPosition() const override { return mJointPosition; }
    const std::vector<float>& jointVelocity() const override { return mJointVelocity; }

protected:
    void readState() override
    {
        PxArticulationCache* artCache = mArticulations[0]->createCache(); // one cache since all articulations assumed to be identical
        for (int art = 0; art < mNbRobots; ++art)
        {
            const PxArticulationReducedCoordinate* articulation = mArticulations[art];
            articulation->copyInternalStateToCache(*artCache, PxArticulationCacheFlag::eROOT_TRANSFORM);
            CARB_ASSERT(articulation->getDofs() == mNbDofs && "All articulations must have the same number of dofs.");

            const PxTransform baseFrame = artCache->rootLinkData->transform;

            articulation->copyInternalStateToCache(*artCache, PxArticulationCacheFlag::eROOT_VELOCITIES);
            mRootLinearVelocity[art] = baseFrame.rotateInv(artCache->rootLinkData->worldLinVel);
            mRootAngularVelocity[art] = baseFrame.rotateInv(artCache->rootLinkData->worldAngVel);
            mRootProjectedGravity[art] = baseFrame.rotateInv(this->mGravityDirection);

            // joint positions
            articulation->copyInternalStateToCache(*artCache, PxArticulationCacheFlag::ePOSITION);
            std::memcpy(mJointPosition.data() + mNbDofs * art, artCache->jointPosition, mNbDofs*sizeof(float));

            // joint velocities
            articulation->copyInternalStateToCache(*artCache, PxArticulationCacheFlag::eVELOCITY);
            std::memcpy(mJointVelocity.data() + mNbDofs * art, artCache->jointVelocity, mNbDofs*sizeof(float));
        }
    }

    std::vector<PxTransform> mRootTransform;
    std::vector<PxVec3> mRootLinearVelocity;
    std::vector<PxVec3> mRootAngularVelocity;
    std::vector<PxVec3> mRootProjectedGravity;

    std::vector<float> mJointPosition;
    std::vector<float> mJointVelocity;

    std::vector<PxArticulationReducedCoordinate*> mArticulations;
};

class GPURobotData: public RobotData
{
public:
    GPURobotData(int nbRobots, int nbDofs,
            PxDirectGPUAPI* gpuApi, const DirectGPUBuffer<PxArticulationGPUIndex>* articulationIdx, const PxScene* scene) :
        RobotData(nbRobots, nbDofs, scene), mGpuApi(gpuApi), mArticulationIdx(articulationIdx)
    {
        mRootTransform.allocate(mNbRobots);
        mRootLinearVelocity.allocate(mNbRobots);
        mRootAngularVelocity.allocate(mNbRobots);
        mRootProjectedGravity.resize(mNbRobots);

        mJointPosition.allocate(mNbRobots*mNbDofs);
        mJointVelocity.allocate(mNbRobots*mNbDofs);
    }
    const std::vector<PxVec3>& rootLinearVelocity() const override { return mRootLinearVelocity.host; }
    const std::vector<PxVec3>& rootAngularVelocity() const override { return mRootAngularVelocity.host; }
    const std::vector<PxVec3>& rootProjectedGravity() const override { return mRootProjectedGravity; }
    const std::vector<float>& jointPosition() const override { return mJointPosition.host; }
    const std::vector<float>& jointVelocity() const override { return mJointVelocity.host; }

    template<typename T>
    void getData(DirectGPUBuffer<T>& dstBuf, PxArticulationGPUAPIReadType::Enum readType)
    {
        mGpuApi->getArticulationData(
            (void*) dstBuf.device, (PxArticulationGPUIndex*) mArticulationIdx->device, readType, this->mNbRobots);
    }

    void readState() override
    {
        getData(mRootTransform, PxArticulationGPUAPIReadType::eROOT_GLOBAL_POSE);
        mRootTransform.copyDtoH();

        // linear velocity w.r.t. base frame
        getData(mRootLinearVelocity, PxArticulationGPUAPIReadType::eROOT_LINEAR_VELOCITY);
        mRootLinearVelocity.copyDtoH();

        // angular velocity w.r.t. base frame
        getData(mRootAngularVelocity, PxArticulationGPUAPIReadType::eROOT_ANGULAR_VELOCITY);
        this->mRootAngularVelocity.copyDtoH();

        // apply transforms in-place
        for (int art = 0; art < mNbRobots; ++art)
        {
            const PxTransform baseFrame = mRootTransform.host[art];
            mRootLinearVelocity.host[art] = baseFrame.rotateInv(mRootLinearVelocity.host[art]);
            mRootAngularVelocity.host[art] = baseFrame.rotateInv(mRootAngularVelocity.host[art]);
            mRootProjectedGravity[art] = baseFrame.rotateInv(this->mGravityDirection);
        }

        // joint positions
        getData(mJointPosition, PxArticulationGPUAPIReadType::eJOINT_POSITION);
        this->mJointPosition.copyDtoH();

        // joint velocities
        getData(mJointVelocity, PxArticulationGPUAPIReadType::eJOINT_VELOCITY);
        this->mJointVelocity.copyDtoH();
    }

protected:

    PxDirectGPUAPI* mGpuApi;
    const DirectGPUBuffer<PxArticulationGPUIndex>* mArticulationIdx;

    DirectGPUBuffer<PxTransform> mRootTransform;
    DirectGPUBuffer<PxVec3> mRootLinearVelocity;
    DirectGPUBuffer<PxVec3> mRootAngularVelocity;
    std::vector<PxVec3> mRootProjectedGravity;

    DirectGPUBuffer<float> mJointPosition;
    DirectGPUBuffer<float> mJointVelocity;
};

#endif  // PHYSICS_CBM_UTILS_H
