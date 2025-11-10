// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "ObjectTypes.h"
namespace omni
{
namespace physics
{
namespace tensors
{
class IArticulationView;
class IRigidBodyView;
class ISoftBodyView;
class ISoftBodyMaterialView;
class IDeformableBodyView;
class IDeformableMaterialView;
class IRigidContactView;
class IParticleSystemView;
class IParticleClothView;
class IParticleMaterialView;
class ISdfShapeView;

class ISimulationView
{
public:
    // return -1 for CPU or 0+ for GPU
    virtual int getDeviceOrdinal() const = 0;

    // return nullptr for CPU or CUcontext for GPU
    virtual void* getCudaContext() const = 0;

    // return false if the simulation view is invalidated due to prim deletion
    virtual bool getValid() const = 0;

    // invalidate the simulationView
    virtual void invalidate() = 0;

    virtual bool setSubspaceRoots(const char* pattern) = 0;

    virtual IArticulationView* createArticulationView(const char* pattern) = 0;
    virtual IArticulationView* createArticulationView(const std::vector<std::string>& patterns) = 0;

    virtual IRigidBodyView* createRigidBodyView(const std::vector<std::string>& patterns) = 0;
    virtual IRigidBodyView* createRigidBodyView(const char* pattern) = 0;

    // DEPRECATED
    virtual IRigidContactView* createRigidContactView(const char* pattern,
                                                      const char** filterPatterns,
                                                      uint32_t numFilterPatterns,
                                                      uint32_t maxContactDataCount) = 0;
    virtual IRigidContactView* createRigidContactView(const std::string pattern,
                                                      const std::vector<std::string>& filterPatterns,
                                                      uint32_t maxContactDataCount) = 0;
    virtual IRigidContactView* createRigidContactView(const std::vector<std::string>& patterns,
                                                      const std::vector<std::vector<std::string>>& filterPatterns,
                                                      uint32_t maxContactDataCount) = 0;

    virtual ISdfShapeView* createSdfShapeView(const char* pattern, uint32_t numSamplePoints) = 0;

    virtual ISoftBodyView* createSoftBodyView(const char* pattern) = 0;
    virtual ISoftBodyMaterialView* createSoftBodyMaterialView(const char* pattern) = 0;

    virtual IDeformableBodyView* createVolumeDeformableBodyView(const char* pattern) = 0;
    virtual IDeformableBodyView* createSurfaceDeformableBodyView(const char* pattern) = 0;
    virtual IDeformableMaterialView* createDeformableMaterialView(const char* pattern) = 0;

    virtual IParticleSystemView* createParticleSystemView(const char* pattern) = 0;

    virtual IParticleClothView* createParticleClothView(const char* pattern) = 0;

    virtual IParticleMaterialView* createParticleMaterialView(const char* pattern) = 0;

    virtual ObjectType getObjectType(const char* path) = 0;

    virtual void clearForces() = 0;

    // needed to submit all commands to PhysX when using the direct GPU pipeline
    virtual bool flush() = 0;

    virtual void updateArticulationsKinematic() = 0;
    virtual void InitializeKinematicBodies() = 0;

    virtual bool check() const = 0;

    virtual void enableGpuUsageWarnings(bool enable) = 0;

    virtual void release(bool recursive) = 0;

    // HACK? manually stepping simulation
    virtual void step(float dt) = 0;

    // physics scene properties
    virtual bool setGravity(const carb::Float3&) = 0;
    virtual bool getGravity(carb::Float3&) = 0;

protected:
    virtual ~ISimulationView() = default;
};

} // namespace tensors
} // namespace physics
} // namespace omni
