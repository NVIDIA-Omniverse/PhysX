// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include <carb/BindingsPythonUtils.h>

#include <omni/physics/tensors/IArticulationView.h>
#include <omni/physics/tensors/IRigidBodyView.h>
#include <omni/physics/tensors/ISoftBodyView.h>
#include <omni/physics/tensors/ISoftBodyMaterialView.h>
#include <omni/physics/tensors/IDeformableBodyView.h>
#include <omni/physics/tensors/IDeformableMaterialView.h>
#include <omni/physics/tensors/IRigidContactView.h>
#include <omni/physics/tensors/ISdfShapeView.h>
#include <omni/physics/tensors/IParticleSystemView.h>
#include <omni/physics/tensors/IParticleClothView.h>
#include <omni/physics/tensors/IParticleMaterialView.h>
#include <omni/physics/tensors/ISimulationBackend.h>
#include <omni/physics/tensors/ISimulationView.h>
#include <omni/physics/tensors/TensorApi.h>
#include <omni/physics/tensors/BodyTypes.h>
#include <omni/physics/tensors/ObjectTypes.h>
#include <pybind11/numpy.h>

CARB_BINDINGS("omni.physics.tensors.python")

namespace omni
{
namespace physics
{
namespace tensors
{

#if 0
class PySimulationBackend : public ISimulationBackend
{
public:
    ISimulationView* createSimulationView(long stageId) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(ISimulationView*, ISimulationBackend, "create_simulation_view", createSimulationView, stageId);
    }
};
#endif

class PySimulationView : public ISimulationView
{
public:
    struct Deleter
    {
        void operator()(ISimulationView* sim) const
        {
            // printf("~!~!~! Simulation view deleter called\n");
            if (sim)
            {
                // use non-recursive release from Python
                sim->release(false);
            }
        }
    };

    int getDeviceOrdinal() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISimulationView, "get_device_ordinal", getDeviceOrdinal);
    }

    void* getCudaContext() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void*, ISimulationView, "get_cuda_context", getCudaContext);
    }
    
    bool getValid() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISimulationView, "get_valid", getValid);
    }

    void invalidate() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, ISimulationView, "invalidate", invalidate);
    }

    bool setSubspaceRoots(const char* pattern) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISimulationView, "set_subspace_roots", setSubspaceRoots, pattern);
    }

    ObjectType getObjectType(const char* path) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(ObjectType, ISimulationView, "get_object_type", getObjectType, path);
    }

    IArticulationView* createArticulationView(const char* pattern) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            IArticulationView*, ISimulationView, "create_articulation_view", createArticulationView, pattern);
    }
    
    IArticulationView* createArticulationView(const std::vector<std::string>& patternLis) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            IArticulationView*, ISimulationView, "create_articulation_view", createArticulationView, patternLis);
    }

    IRigidBodyView* createRigidBodyView(const char* pattern) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            IRigidBodyView*, ISimulationView, "create_rigid_body_view", createRigidBodyView, pattern);
    }

    IRigidBodyView* createRigidBodyView(const std::vector<std::string>& patternList) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            IRigidBodyView*, ISimulationView, "create_rigid_body_view", createRigidBodyView, patternList);
    }

    ISoftBodyView* createSoftBodyView(const char* pattern) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            ISoftBodyView*, ISimulationView, "create_soft_body_view", createSoftBodyView, pattern);
    }
    
    ISoftBodyMaterialView* createSoftBodyMaterialView(const char* pattern) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            ISoftBodyMaterialView*, ISimulationView, "create_soft_body_material_view", createSoftBodyMaterialView, pattern);
    }

    IDeformableBodyView* createVolumeDeformableBodyView(const char* pattern) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(IDeformableBodyView*, ISimulationView, "create_volume_deformable_body_view", createVolumeDeformableBodyView, pattern);
    }

    IDeformableBodyView* createSurfaceDeformableBodyView(const char* pattern) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(IDeformableBodyView*, ISimulationView, "create_surface_deformable_body_view", createSurfaceDeformableBodyView, pattern);
    }

    IDeformableMaterialView* createDeformableMaterialView(const char* pattern) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(IDeformableMaterialView*, ISimulationView, "create_deformable_material_view", createDeformableMaterialView, pattern);
    }

    IParticleSystemView* createParticleSystemView(const char* pattern) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            IParticleSystemView*, ISimulationView, "create_particle_system_view", createParticleSystemView, pattern);
    }

    IParticleClothView* createParticleClothView(const char* pattern) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            IParticleClothView*, ISimulationView, "create_particle_cloth_view", createParticleClothView, pattern);
    }

    IParticleMaterialView* createParticleMaterialView(const char* pattern) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            IParticleMaterialView*, ISimulationView, "create_particle_material_view", createParticleMaterialView, pattern);
    }

    bool flush() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISimulationView, "flush", flush);
    }

    //DEPRECATED
    IRigidContactView* createRigidContactView(const char* pattern,
                                              const char** filterPatterns,
                                              uint32_t numFilterPatterns,
                                              uint32_t maxContactDataCount) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(IRigidContactView*, ISimulationView, "create_rigid_contact_view",
                                    createRigidContactView,
                                    // pattern, filterPatterns, numFilterPatterns);
                                    pattern, nullptr, 0, maxContactDataCount); // FIXME?
    }
    IRigidContactView* createRigidContactView(const std::string pattern,
                                              const std::vector<std::string>& filterPatterns,
                                              uint32_t maxContactDataCount) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(IRigidContactView*, ISimulationView, "create_rigid_contact_view",
                                    createRigidContactView, pattern, filterPatterns, maxContactDataCount);
    }
    IRigidContactView* createRigidContactView(const std::vector<std::string>& patternList,
                                              const std::vector<std::vector<std::string>>& filterPatterns,
                                              uint32_t maxContactDataCount) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(IRigidContactView*, ISimulationView, "create_rigid_contact_view",
                                    createRigidContactView, patternList, filterPatterns, maxContactDataCount);
    }


    ISdfShapeView* createSdfShapeView(const char* pattern, uint32_t numSamplePoints) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            ISdfShapeView*, ISimulationView, "create_sdf_shape_view", createSdfShapeView, pattern, numSamplePoints);
    }

    void clearForces() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, ISimulationView, "clear_forces", clearForces);
    }

    void enableGpuUsageWarnings(bool enable) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, ISimulationView, "enable_warnings", enableGpuUsageWarnings, enable);
    }

    void updateArticulationsKinematic() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, ISimulationView, "update_articulations_kinematic", updateArticulationsKinematic);
    }
    void InitializeKinematicBodies() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, ISimulationView, "initialize_kinematic_bodies", InitializeKinematicBodies);
    }

    bool check() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISimulationView, "check", check);
    }

    void step(float dt)
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, ISimulationView, "step", step, dt);
    }

    bool setGravity(const carb::Float3& gravity) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISimulationView, "set_gravity", gravity);
    }

    bool getGravity(carb::Float3& gravity) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISimulationView, "get_gravity", getGravity, gravity);
    }

private:
    // NOTE: this is not exposed to Python, it's only used in the Deleter
    void release(bool recursive)
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, ISimulationView, "_release", release, recursive);
    }
};

class PyArticulationMetatype : public IArticulationMetatype
{
public:
    uint32_t getLinkCount() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IArticulationMetatype, "get_link_count", getLinkCount);
    }

    uint32_t getJointCount() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IArticulationMetatype, "get_joint_count", getJointCount);
    }

    uint32_t getDofCount() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IArticulationMetatype, "get_dof_count", getDofCount);
    }

    const char* getLinkName(uint32_t linkIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(const char*, IArticulationMetatype, "get_link_name", getLinkName, linkIdx);
    }

    const char* getLinkParentName(uint32_t linkIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(const char*, IArticulationMetatype, "get_link_parent_name", getLinkParentName, linkIdx);
    }

    const char* getJointName(uint32_t jointIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(const char*, IArticulationMetatype, "get_joint_name", getJointName, jointIdx);
    }

    const char* getDofName(uint32_t dofIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(const char*, IArticulationMetatype, "get_dof_name", getDofName, dofIdx);
    }

    int32_t findLinkIndex(const char* linkName) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(int32_t, IArticulationMetatype, "find_link_index", findLinkIndex, linkName);
    }

    int32_t findLinkParentIndex(const char* linkName) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(int32_t, IArticulationMetatype, "find_link_parent_index", findLinkParentIndex, linkName);
    }

    int32_t findJointIndex(const char* jointName) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(int32_t, IArticulationMetatype, "find_joint_index", findJointIndex, jointName);
    }

    int32_t findDofIndex(const char* dofName) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(int32_t, IArticulationMetatype, "find_dof_index", findDofIndex, dofName);
    }

    JointType getJointType(uint32_t jointIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(JointType, IArticulationMetatype, "get_joint_type", getJointType, jointIdx);
    }

    uint32_t getJointDofOffset(uint32_t jointIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IArticulationMetatype, "get_joint_dof_offset", getJointDofOffset, jointIdx);
    }

    uint32_t getJointDofCount(uint32_t jointIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IArticulationMetatype, "get_joint_dof_count", getJointDofCount, jointIdx);
    }

    DofType getDofType(uint32_t dofIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(DofType, IArticulationMetatype, "get_dof_type", getDofType, dofIdx);
    }

    bool getFixedBase() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationMetatype, "get_fixed_base", getFixedBase);
    }
};

class PyArticulationView : public IArticulationView
{
public:
    struct Deleter
    {
        void operator()(IArticulationView* artiView) const
        {
            // printf("~!~!~! Articulation view deleter called\n");
            if (artiView)
            {
                artiView->release();
            }
        }
    };

    uint32_t getCount() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IArticulationView, "get_count", getCount);
    }

    uint32_t getMaxLinks() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IArticulationView, "get_max_links", getMaxLinks);
    }

    uint32_t getMaxDofs() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IArticulationView, "get_max_dofs", getMaxDofs);
    }

    uint32_t getMaxShapes() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IArticulationView, "get_max_shapes", getMaxShapes);
    }

    uint32_t getMaxFixedTendons() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IArticulationView, "get_max_fixed_tendons", getMaxFixedTendons);
    }

    uint32_t getMaxSpatialTendons() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IArticulationView, "get_max_spatial_tendons", getMaxSpatialTendons);
    }

    const char* getUsdPrimPath(uint32_t artiIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(const char*, IArticulationView, "get_prim_paths", getUsdPrimPath);
    }

    const char* getUsdDofPath(uint32_t artiIdx, uint32_t dofIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(const char*, IArticulationView, "get_dof_paths", getUsdDofPath);
    }

    const char* getUsdLinkPath(uint32_t artiIdx, uint32_t linkIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(const char*, IArticulationView, "get_link_paths", getUsdLinkPath);
    }

    bool isHomogeneous() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "is_homogeneous", isHomogeneous);
    }

    const IArticulationMetatype* getSharedMetatype() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            const IArticulationMetatype*, IArticulationView, "get_shared_metatype", getSharedMetatype);
    }

    const IArticulationMetatype* getMetatype(uint32_t artiIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            const IArticulationMetatype*, IArticulationView, "get_metatype", getMetatype, artiIdx);
    }

    bool getDofTypes(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_dof_types", getDofTypes, dstTensor);
    }

    bool getDofMotions(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_dof_motions", getDofMotions, dstTensor);
    }

    bool getDofLimits(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_dof_limits", getDofLimits, dstTensor);
    }

    bool getDriveTypes(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_drive_types", getDriveTypes, dstTensor);
    }

    bool getDofStiffnesses(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_dof_stiffnesses", getDofStiffnesses, dstTensor);
    }

    bool getDofDampings(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_dof_dampings", getDofDampings, dstTensor);
    }

    bool getDofMaxForces(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_dof_max_forces", getDofMaxForces, dstTensor);
    }

    bool getDofDriveModelProperties(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_dof_drive_model_properties", getDofDriveModelProperties, dstTensor);
    }

    //DEPRECATED
    bool getDofFrictionCoefficients(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_dof_friction_coefficients", getDofFrictionCoefficients, dstTensor);
    }

    bool getDofFrictionProperties(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_dof_friction_properties", getDofFrictionProperties, dstTensor);
    }

    bool getDofMaxVelocities(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_dof_max_velocities", getDofMaxVelocities, dstTensor);
    }

    bool getDofArmatures(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_dof_armatures", getDofArmatures, dstTensor);
    }

    bool setDofLimits(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "set_dof_limits", setDofLimits, srcTensor, indexTensor);
    }

    bool setDofStiffnesses(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "set_dof_stiffnesses", setDofStiffnesses, srcTensor, indexTensor);
    }

    bool setDofDampings(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "set_dof_dampings", setDofDampings, srcTensor, indexTensor);
    }

    bool setDofMaxForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "set_dof_max_forces", setDofMaxForces, srcTensor, indexTensor);
    }

    bool setDofDriveModelProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "set_dof_drive_model_properties", setDofDriveModelProperties, srcTensor, indexTensor);
    }

    //DEPRECATED
    bool setDofFrictionCoefficients(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "set_dof_friction_coefficients", setDofFrictionCoefficients, srcTensor, indexTensor);
    }

    bool setDofFrictionProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "set_dof_friction_properties", setDofFrictionProperties, srcTensor, indexTensor);
    }

    bool setDofMaxVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "set_dof_max_velocities", setDofMaxVelocities, srcTensor, indexTensor);
    }

    bool setDofArmatures(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "set_dof_armatures", setDofArmatures, srcTensor, indexTensor);
    }

    bool getLinkTransforms(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_link_transforms", getLinkTransforms, dstTensor);
    }

    bool getLinkVelocities(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_link_velocities", getLinkVelocities, dstTensor);
    }

    bool getLinkAccelerations(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_link_accelerations", getLinkAccelerations, dstTensor);
    }

    bool getDofPositions(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_dof_positions", getDofPositions, dstTensor);
    }

    bool getDofVelocities(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_dof_velocities", getDofVelocities, dstTensor);
    }

    bool getRootTransforms(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_root_transforms", getRootTransforms, dstTensor);
    }

    bool getRootVelocities(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_root_velocities", getRootVelocities, dstTensor);
    }

    bool setRootTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            bool, IArticulationView, "set_root_transforms", setRootTransforms, srcTensor, indexTensor);
    }

    bool setRootVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            bool, IArticulationView, "set_root_velocities", setRootVelocities, srcTensor, indexTensor);
    }

    bool setDofPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            bool, IArticulationView, "set_dof_positions", setDofPositions, srcTensor, indexTensor);
    }

    bool setDofVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            bool, IArticulationView, "set_dof_velocities", setDofVelocities, srcTensor, indexTensor);
    }

    bool setDofActuationForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            bool, IArticulationView, "set_dof_actuation_forces", setDofActuationForces, srcTensor, indexTensor);
    }

    bool setDofPositionTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            bool, IArticulationView, "set_dof_position_targets", setDofPositionTargets, srcTensor, indexTensor);
    }

    bool setDofVelocityTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            bool, IArticulationView, "set_dof_velocity_targets", setDofVelocityTargets, srcTensor, indexTensor);
    }

    bool getDofPositionTargets(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            bool, IArticulationView, "get_dof_position_targets", getDofPositionTargets, dstTensor);
    }

    bool getDofVelocityTargets(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            bool, IArticulationView, "get_dof_velocity_targets", getDofVelocityTargets, dstTensor);
    }

    // gets the actuation forces that is set by setDofActuationForces 
    bool getDofActuationForces(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            bool, IArticulationView, "get_dof_actuation_forces", getDofActuationForces, dstTensor);
    }

    // gets the joint forces projected onto the joint motion space
    bool getDofProjectedJointForces(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            bool, IArticulationView, "get_dof_projected_joint_forces", getDofProjectedJointForces, dstTensor);
    }

    bool getJacobianShape(uint32_t* numRows, uint32_t* numCols) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_jacobian_shape", getJacobianShape, numRows, numCols);
    }

    bool getJacobians(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_jacobians", getJacobians, dstTensor);
    }

    //DEPRECATED
    bool getMassMatrixShape(uint32_t* numRows, uint32_t* numCols) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_mass_matrix_shape", getMassMatrixShape, numRows, numCols);
    }

    bool getGeneralizedMassMatrixShape(uint32_t* numRows, uint32_t* numCols) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_generalized_mass_matrix_shape", getGeneralizedMassMatrixShape, numRows, numCols);
    }

    //DEPRECATED
    bool getMassMatrices(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_mass_matrices", getMassMatrices, dstTensor);
    }

    bool getGeneralizedMassMatrices(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_generalized_mass_matrices", getGeneralizedMassMatrices, dstTensor);
    }

    //DEPRECATED
    bool getCoriolisAndCentrifugalForces(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_coriolis_and_centrifugal_forces", getCoriolisAndCentrifugalForces, dstTensor);
    }

    bool getCoriolisAndCentrifugalCompensationForces(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_coriolis_and_centrifugal_compensation_forces", getCoriolisAndCentrifugalCompensationForces, dstTensor);
    }

    //DEPRECATED
    bool getGeneralizedGravityForces(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_generalized_gravity_forces", getGeneralizedGravityForces, dstTensor);
    }

    bool getGravityCompensationForces(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_gravity_compensation_forces", getGravityCompensationForces, dstTensor);
    }

    bool getArticulationCentroidalMomentum(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_articulation_centroidal_momentum", getArticulationCentroidalMomentum, dstTensor);
    }
    bool getArticulationMassCenter(const TensorDesc* dstTensor, bool localFrame) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_articulation_mass_center", getArticulationMassCenter, dstTensor, localFrame);
    }
    bool getLinkIncomingJointForce(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_link_incoming_joint_force", getLinkIncomingJointForce, dstTensor);
    }

    bool applyForcesAndTorquesAtPosition(const TensorDesc* srcForceTensor,
                                         const TensorDesc* srcTorqueTensor,
                                         const TensorDesc* srcPositionTensor,
                                         const TensorDesc* indexTensor,
                                         const bool isGlobal) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "apply_forces_and_torques_at_position", applyForces,
                                    srcForceTensor, srcTorqueTensor, srcPositionTensor, indexTensor, isGlobal);
    }

    bool getMasses(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_masses", getMasses, dstTensor);
    }
    bool getInvMasses(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_inv_masses", getInvMasses, dstTensor);
    }
    bool getCOMs(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_coms", getCOMs, dstTensor);
    }
    bool getInertias(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_intertias", getInertias, dstTensor);
    }
    bool getInvInertias(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_inv_intertias", getInvInertias, dstTensor);
    }
    bool getDisableGravities(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_disable_gravities", getDisableGravities, dstTensor);
    }

    bool setMasses(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "set_masses", setMasses, srcTensor, indexTensor);
    }
    bool setCOMs(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "set_coms", setCOMs, srcTensor, indexTensor);
    }
    bool setInertias(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "set_inertias", setInertias, srcTensor, indexTensor);
    }
    bool setDisableGravities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "set_disable_gravities", setDisableGravities, srcTensor, indexTensor);
    }
    bool getMaterialProperties(const TensorDesc* dstTensor) const override
    {
       PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_material_properties", getMaterialProperties, dstTensor);
    }
    bool getRestOffsets(const TensorDesc* dstTensor) const override
    {
       PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_rest_offsets", getRestOffsets, dstTensor);
    }
    bool getContactOffsets(const TensorDesc* dstTensor) const override
    {
       PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_contact_offsets", getContactOffsets, dstTensor);
    }

    bool setMaterialProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override
    {
       PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "set_material_properties", setMaterialProperties, srcTensor, indexTensor);
    }
    bool setRestOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override
    {
       PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "set_rest_offsets", setRestOffsets, srcTensor, indexTensor);
    }
    bool setContactOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override
    {
       PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "set_contact_offsets", setContactOffsets, srcTensor, indexTensor);
    }

    bool getFixedTendonStiffnesses(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_fixed_tendon_stiffnesses", getFixedTendonStiffnesses, dstTensor);
    }
    bool getFixedTendonDampings(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_fixed_tendon_dampings", getFixedTendonDampings, dstTensor);
    }
    bool getFixedTendonLimitStiffnesses(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_fixed_tendon_limit_stiffnesses", getFixedTendonLimitStiffnesses, dstTensor);
    }
    bool getFixedTendonLimits(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_fixed_tendon_limits", getFixedTendonLimits, dstTensor);
    }
    bool getFixedTendonfixedSpringRestLengths(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_fixed_tendon_rest_lengths", getFixedTendonfixedSpringRestLengths, dstTensor);
    }
    bool getFixedTendonOffsets(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_fixed_tendon_offsets", getFixedTendonOffsets, dstTensor);
    }

    bool getSpatialTendonStiffnesses(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_spatial_tendon_stiffnesses", getSpatialTendonStiffnesses, dstTensor);
    }
    bool getSpatialTendonDampings(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_spatial_tendon_dampings", getSpatialTendonDampings, dstTensor);
    }
    bool getSpatialTendonLimitStiffnesses(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_spatial_tendon_limit_stiffnesses", getSpatialTendonLimitStiffnesses, dstTensor);
    }
    bool getSpatialTendonOffsets(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "get_spatial_tendon_offsets", getSpatialTendonOffsets, dstTensor);
    }

    bool setFixedTendonProperties(const TensorDesc* stiffnesses, const TensorDesc* dampings, const TensorDesc* limitStiffnesses, const TensorDesc* limits, const TensorDesc* restLengths, const TensorDesc* offsets, const TensorDesc* indexTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "set_fixed_tendon_properties", setFixedTendonProperties, stiffnesses, dampings, limitStiffnesses, limits, restLengths, offsets, indexTensor);
    }

    bool setSpatialTendonProperties(const TensorDesc* stiffnesses, const TensorDesc* dampings, const TensorDesc* limitStiffnesses, const TensorDesc* offsets, const TensorDesc* indexTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "set_spatial_tendon_properties", setSpatialTendonProperties, stiffnesses, dampings, limitStiffnesses, offsets, indexTensor);
    }

    bool check() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IArticulationView, "check", check);
    }

private:
    // NOTE: this is not exposed to Python, it's only used in the Deleter
    void release() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, IArticulationView, "_release", release);
    }
};

class PyRigidBodyView : public IRigidBodyView
{
public:
    struct Deleter
    {
        void operator()(IRigidBodyView* rbView) const
        {
            // printf("~!~!~! RB view deleter called\n");
            if (rbView)
            {
                rbView->release();
            }
        }
    };

    uint32_t getCount() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IRigidBodyView, "get_count", getCount);
    }

    uint32_t getMaxShapes() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IRigidBodyView, "get_max_shapes", getMaxShapes);
    }

    const char* getUsdPrimPath(uint32_t rbIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(const char*, IRigidBodyView, "get_prim_paths", getUsdPrimPath);
    }

    bool getTransforms(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "get_transforms", getTransforms, dstTensor);
    }

    bool getVelocities(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "get_velocities", getVelocities, dstTensor);
    }
    
    bool getAccelerations(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "get_accelerations",  getAccelerations, dstTensor);
    }

    bool setKinematicTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "set_kinematic_targets", setKinematicTargets, srcTensor, indexTensor);
    }

    bool setTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "set_transforms", setTransforms, srcTensor, indexTensor);
    }

    bool setVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "set_velocities", setVelocities, srcTensor, indexTensor);
    }

    bool applyForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "apply_force", applyForces, srcTensor, indexTensor);
    }
        
    bool applyForcesAndTorquesAtPosition(const TensorDesc* srcForceTensor,
                                         const TensorDesc* srcTorqueTensor,
                                         const TensorDesc* srcPositionTensor,
                                         const TensorDesc* indexTensor,
                                         const bool isGlobal) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "apply_forces_and_torques_at_position", applyForces, srcForceTensor, srcTorqueTensor, srcPositionTensor, indexTensor, isGlobal);
    }

    bool getMasses(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "get_masses", getMasses, dstTensor);
    }
    bool getInvMasses(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "get_inv_masses", getInvMasses, dstTensor);
    }
    bool getCOMs(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "get_coms", getCOMs, dstTensor);
    }
    bool getInertias(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "get_intertias", getInertias, dstTensor);
    }
    bool getInvInertias(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "get_inv_intertias", getInvInertias, dstTensor);
    }
    bool getDisableGravities(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "get_disable_gravities", getDisableGravities, dstTensor);
    }
    bool getDisableSimulations(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "get_disable_simulations", getDisableSimulations, dstTensor);
    }

    bool setMasses(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "set_masses", setMasses, srcTensor, indexTensor);
    }
    bool setCOMs(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "set_coms", setCOMs, srcTensor, indexTensor);
    }
    bool setInertias(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "set_inertias", setInertias, srcTensor, indexTensor);
    }
    bool setDisableGravities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "set_disable_gravities", setDisableGravities, srcTensor, indexTensor);
    }
    bool setDisableSimulations(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "set_disable_simulations", setDisableSimulations, srcTensor, indexTensor);
    }

    bool getMaterialProperties(const TensorDesc* dstTensor) const override
    {
       PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "get_material_properties", getMaterialProperties, dstTensor);
    }
    bool getRestOffsets(const TensorDesc* dstTensor) const override
    {
       PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "get_rest_offsets", getRestOffsets, dstTensor);
    }
    bool getContactOffsets(const TensorDesc* dstTensor) const override
    {
       PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "get_contact_offsets", getContactOffsets, dstTensor);
    }

    bool setMaterialProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override
    {
       PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "set_material_properties", setMaterialProperties, srcTensor, indexTensor);
    }
    bool setRestOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override
    {
       PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "set_rest_offsets", setRestOffsets, srcTensor, indexTensor);
    }
    bool setContactOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override
    {
       PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "set_contact_offsets", setContactOffsets, srcTensor, indexTensor);
    }

    bool check() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidBodyView, "check", check);
    }

private:
    // NOTE: this is not exposed to Python, it's only used in the Deleter
    void release() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, IRigidBodyView, "_release", release);
    }
};

class PySoftBodyView : public ISoftBodyView
{
public:
    struct Deleter
    {
        void operator()(ISoftBodyView* sbView) const
        {
            // printf("~!~!~! SB view deleter called\n");
            if (sbView)
            {
                sbView->release();
            }
        }
    };

    uint32_t getCount() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, ISoftBodyView, "get_count", getCount);
    }

    bool getTransforms(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyView, "get_transforms", getTransforms, dstTensor);
    }
    
    uint32_t getMaxElementsPerBody() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, ISoftBodyView, "get_max_elements_per_body", getMaxElementsPerBody);
    }

    uint32_t getMaxSimElementsPerBody() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, ISoftBodyView, "get_max_sim_elements_per_body", getMaxSimElementsPerBody);
    }

    uint32_t getMaxVerticesPerBody() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, ISoftBodyView, "get_max_vertices_per_body", getMaxVerticesPerBody);
    }

    uint32_t getMaxSimVerticesPerBody() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, ISoftBodyView, "get_max_sim_vertices_per_body", getMaxSimVerticesPerBody);
    }

    bool getElementDeformationGradients(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyView, "get_element_deformation_gradients", getElementCauchyStresses, dstTensor);
    }

    bool getElementStresses(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyView, "get_element_stresses", getElementStresses, dstTensor);
    }

    bool getElementRestPoses(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyView, "get_element_rest_poses", getElementRestPoses, dstTensor);
    }

    bool getElementRotations(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyView, "get_element_rotations", getElementRotations, dstTensor);
    }

    bool getNodalPositions(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyView, "get_nodal_positions", getNodalPositions, dstTensor);
    }

    bool getElementIndices(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyView, "get_element_indices", getElementIndices, dstTensor);
    }

    bool getSimElementIndices(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyView, "get_sim_element_indices", getSimElementIndices, dstTensor);
    }

    bool getSimElementDeformationGradients(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyView, "get_sim_element_deformation_gradients", getSimElementCauchyStresses, dstTensor);
    }

    bool getSimElementStresses(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyView, "get_sim_element_stresses", getSimElementStresses, dstTensor);
    }

    bool getSimElementRestPoses(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyView, "get_sim_element_rest_poses", getSimElementRestPoses, dstTensor);
    }

    bool getSimElementRotations(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyView, "get_sim_element_rotations", getSimElementRotations, dstTensor);
    }

    bool getSimNodalPositions(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyView, "get_sim_nodal_positions", getSimNodalPositions, dstTensor);
    }

    bool setSimNodalPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            bool, ISoftBodyView, "set_sim_nodal_positions", setSimNodalPositions, srcTensor, indexTensor);
    }

    bool getSimNodalVelocities(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyView, "get_sim_nodal_velocities", getSimNodalVelocities, dstTensor);
    }

    bool setSimNodalVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            bool, ISoftBodyView, "set_sim_nodal_velocities", setSimNodalVelocities, srcTensor, indexTensor);
    }

    bool getSimKinematicTargets(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyView, "get_sim_kinematic_targets", getSimKinematicTargets, dstTensor);
    }

    bool setSimKinematicTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            bool, ISoftBodyView, "set_sim_kinematic_targets", setSimKinematicTargets, srcTensor, indexTensor);
    }

    bool check() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyView, "check", check);
    }

private:
    // NOTE: this is not exposed to Python, it's only used in the Deleter
    void release() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, ISoftBodyView, "_release", release);
    }
};


class PySoftBodyMaterialView : public ISoftBodyMaterialView
{
public:
    struct Deleter
    {
        void operator()(ISoftBodyMaterialView* sbMaterialView) const
        {
            // printf("~!~!~! SB material view deleter called\n");
            if (sbMaterialView)
            {
                sbMaterialView->release();
            }
        }
    };

    uint32_t getCount() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, ISoftBodyMaterialView, "get_count", getCount);
    }

    bool getYoungsModulus(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyMaterialView, "get_youngs_modulus", getYoungsModulus, dstTensor);
    }

    bool setYoungsModulus(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            bool, ISoftBodyMaterialView, "set_youngs_modulus", setYoungsModulus, srcTensor, indexTensor);
    }

    bool getPoissonsRatio(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyMaterialView, "get_poissons_ratio", getPoissonsRatio, dstTensor);
    }

    bool setPoissonsRatio(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyMaterialView, "set_poissons_ratio", setPoissonsRatio, srcTensor, indexTensor);
    }

    bool getDynamicFriction(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyMaterialView, "get_dynamic_friction", getDynamicFriction, dstTensor);
    }

    bool setDynamicFriction(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            bool, ISoftBodyMaterialView, "set_dynamic_friction", setDynamicFriction, srcTensor, indexTensor);
    }

    bool getDamping(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyMaterialView, "get_damping", getDamping, dstTensor);
    }

    bool setDamping(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyMaterialView, "set_damping", setDamping, srcTensor, indexTensor);
    }

    bool getDampingScale(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyMaterialView, "get_damping_scale", getDampingScale, dstTensor);
    }

    bool setDampingScale(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            bool, ISoftBodyMaterialView, "set_damping_scale", setDampingScale, srcTensor, indexTensor);
    }

    bool check() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISoftBodyMaterialView, "check", check);
    }

private:
    // NOTE: this is not exposed to Python, it's only used in the Deleter
    void release() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, ISoftBodyMaterialView, "_release", release);
    }
};


class PyDeformableBodyView : public IDeformableBodyView
{
public:
    struct Deleter
    {
        void operator()(IDeformableBodyView* view) const
        {
            // printf("~!~!~! Deformable body view deleter called\n");
            if (view)
            {
                view->release();
            }
        }
    };

    uint32_t getCount() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IDeformableBodyView, "get_count", getCount);
    }

    const char* getUsdPrimPath(uint32_t dbIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(const char*, IDeformableBodyView, "get_prim_paths", getUsdPrimPath);
    }

    const char* getUsdSimulationMeshPrimPath(uint32_t dbIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(const char*, IDeformableBodyView, "get_simulation_mesh_prim_paths", getUsdSimulationMeshPrimPath);
    }

    const char* getUsdCollisionMeshPrimPath(uint32_t dbIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(const char*, IDeformableBodyView, "get_collision_mesh_prim_paths", getUsdCollisionMeshPrimPath);
    }

    bool getTransforms(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableBodyView, "get_transforms", getTransforms, dstTensor);
    }

    uint32_t getNumNodesPerElement() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IDeformableBodyView, "get_num_nodes_per_element", getNumNodesPerElement);
    }

    uint32_t getMaxSimulationElementsPerBody() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IDeformableBodyView, "get_max_simulation_elements_per_body", getMaxSimulationElementsPerBody);
    }

    uint32_t getMaxSimulationNodesPerBody() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IDeformableBodyView, "get_max_simulation_nodes_per_body", getMaxSimulationNodesPerBody);
    }

    bool getSimulationElementIndices(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableBodyView, "get_simulation_element_indices", getSimulationElementIndices, dstTensor);
    }

    bool getSimulationNodalPositions(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableBodyView, "get_simulation_nodal_positions", getSimulationNodalPositions, dstTensor);
    }

    bool setSimulationNodalPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableBodyView, "set_simulation_nodal_positions", setSimulationNodalPositions, srcTensor, indexTensor);
    }

    bool getSimulationNodalVelocities(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableBodyView, "get_simulation_nodal_velocities", getSimulationNodalVelocities, dstTensor);
    }

    bool setSimulationNodalVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableBodyView, "set_simulation_nodal_velocities", setSimulationNodalVelocities, srcTensor, indexTensor);
    }

    bool getSimulationNodalKinematicTargets(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableBodyView, "get_simulation_nodal_kinematic_targets", getSimulationNodalKinematicTargets, dstTensor);
    }

    bool setSimulationNodalKinematicTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableBodyView, "set_simulation_nodal_kinematic_targets", setSimulationNodalKinematicTargets, srcTensor, indexTensor);
    }

    uint32_t getMaxRestNodesPerBody() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IDeformableBodyView, "get_max_rest_nodes_per_body", getMaxRestNodesPerBody);
    }

    bool getRestElementIndices(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableBodyView, "get_rest_element_indices", getRestElementIndices, dstTensor);
    }

    bool getRestNodalPositions(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableBodyView, "get_rest_nodal_positions", getRestNodalPositions, dstTensor);
    }

    uint32_t getMaxCollisionElementsPerBody() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IDeformableBodyView, "get_max_collision_elements_per_body", getMaxCollisionElementsPerBody);
    }

    uint32_t getMaxCollisionNodesPerBody() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IDeformableBodyView, "get_max_collision_nodes_per_body", getMaxCollisionNodesPerBody);
    }

    bool getCollisionNodalPositions(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableBodyView, "get_collision_nodal_positions", getCollisionNodalPositions, dstTensor);
    }

    bool getCollisionElementIndices(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableBodyView, "get_collision_element_indices", getCollisionElementIndices, dstTensor);
    }

    bool check() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableBodyView, "check", check);
    }

private:
    // NOTE: this is not exposed to Python, it's only used in the Deleter
    void release() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, IDeformableBodyView, "_release", release);
    }
};


class PyDeformableMaterialView : public IDeformableMaterialView
{
public:
    struct Deleter
    {
        void operator()(IDeformableMaterialView* materialView) const
        {
            // printf("~!~!~! Deformable body material view deleter called\n");
            if (materialView)
            {
                materialView->release();
            }
        }
    };

    uint32_t getCount() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IDeformableMaterialView, "get_count", getCount);
    }

    bool getDynamicFriction(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableMaterialView, "get_dynamic_friction", getDynamicFriction, dstTensor);
    }

    bool setDynamicFriction(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableMaterialView, "set_dynamic_friction", setDynamicFriction, srcTensor, indexTensor);
    }

    bool getYoungsModulus(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableMaterialView, "get_youngs_modulus", getYoungsModulus, dstTensor);
    }

    bool setYoungsModulus(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableMaterialView, "set_youngs_modulus", setYoungsModulus, srcTensor, indexTensor);
    }

    bool getPoissonsRatio(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableMaterialView, "get_poissons_ratio", getPoissonsRatio, dstTensor);
    }

    bool setPoissonsRatio(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableMaterialView, "set_poissons_ratio", setPoissonsRatio, srcTensor, indexTensor);
    }

    bool check() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IDeformableMaterialView, "check", check);
    }

private:
    // NOTE: this is not exposed to Python, it's only used in the Deleter
    void release() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, IDeformableMaterialView, "_release", release);
    }
};


class PyRigidContactView : public IRigidContactView
{
public:
    struct Deleter
    {
        void operator()(IRigidContactView* rcView) const
        {
            // printf("~!~!~! RC view deleter called\n");
            if (rcView)
            {
                rcView->release();
            }
        }
    };

    uint32_t getSensorCount() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IRigidContactView, "get_sensor_count", getSensorCount);
    }

    uint32_t getFilterCount() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IRigidContactView, "get_filter_count", getFilterCount);
    }

    uint32_t getMaxContactDataCount() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IRigidContactView, "get_max_contact_data_count", getMaxContactDataCount);
    }

    bool getNetContactForces(const TensorDesc* dstTensor, float dt) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidContactView, "get_net_contact_forces", getNetContactForces, dstTensor, dt);
    }

    bool getContactForceMatrix(const TensorDesc* dstTensor, float dt) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidContactView, "get_contact_force_matrix", getContactForceMatrix, dstTensor, dt);
    }

    bool getContactData(const TensorDesc* contactForceTensor,
                        const TensorDesc* contactPointTensor,
                        const TensorDesc* contactNormalTensor,
                        const TensorDesc* contactSeparationTensor,
                        const TensorDesc* contactCountTensor,
                        const TensorDesc* contactStartIndicesTensor,
                        float dt) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidContactView, "get_contact_data", getContactData, contactForceTensor,
                                    contactPointTensor, contactNormalTensor, contactSeparationTensor, contactCountTensor,
                                    contactStartIndicesTensor, dt);
    }

    bool getFrictionData(const TensorDesc* FrictionForceTensor,
                        const TensorDesc* contactPointTensor,
                        const TensorDesc* contactCountTensor,
                        const TensorDesc* contactStartIndicesTensor,
                        float dt) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidContactView, "get_friction_data", getFrictionData, FrictionForceTensor,
                                    contactPointTensor, contactCountTensor, contactStartIndicesTensor, dt);
    }

    bool check() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IRigidContactView, "check", check);
    }

    const char* getUsdPrimName(uint32_t sensorIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(const char*, IRigidContactView, "get_prim_names", getUsdPrimName, sensorIdx);
    }

    const char* getUsdPrimPath(uint32_t sensorIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(const char*, IRigidContactView, "get_prim_paths", getUsdPrimPath, sensorIdx);
    }
    
    const char* getFilterUsdPrimName(uint32_t sensorIdx, uint32_t filterIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(const char*, IRigidContactView, "get_filter_prim_name", getFilterUsdPrimName, sensorIdx, filterIdx);
    }

    const char* getFilterUsdPrimPath(uint32_t sensorIdx, uint32_t filterIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(const char*, IRigidContactView, "get_filter_prim_path", getFilterUsdPrimPath, sensorIdx, filterIdx);
    }

private:
    // NOTE: this is not exposed to Python, it's only used in the Deleter
    void release() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, IRigidContactView, "_release", release);
    }
};


class PySdfShapeView : public ISdfShapeView
{
public:
    struct Deleter
    {
        void operator()(ISdfShapeView* sdfView) const
        {
            if (sdfView)
            {
                sdfView->release();
            }
        }
    };

    uint32_t getCount() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, ISdfShapeView, "get_count", getCount);
    }

    uint32_t getMaxNumPoints() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, ISdfShapeView, "get_max_num_points", getMaxNumPoints);
    }
    
    bool getSdfAndGradients(const TensorDesc* dstTensor, const TensorDesc* srcPointTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISdfShapeView, "get_sdf_and_gradients", getSdfAndGradients, dstTensor, srcPointTensor);
    }

    bool check() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ISdfShapeView, "check", check);
    }

    const char* getUsdPrimPath(uint32_t sensorIdx) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(const char*, ISdfShapeView, "get_prim_paths", getUsdPrimPath);
    }

private:
    // NOTE: this is not exposed to Python, it's only used in the Deleter
    void release() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, ISdfShapeView, "_release", release);
    }
};


class PyParticleSystemView : public IParticleSystemView
{
public:
    struct Deleter
    {
        void operator()(IParticleSystemView* psView) const
        {
            if (psView)
            {
                psView->release();
            }
        }
    };
    
    uint32_t getCount() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IParticleSystemView, "get_count", getCount);
    }

    bool getSolidRestOffset(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleSystemView, "get_solid_rest_offsets", getSolidRestOffset, dstTensor);
    }

    bool setSolidRestOffset(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleSystemView, "set_solid_rest_offsets", setSolidRestOffset, srcTensor, indexTensor);
    }

    bool getFluidRestOffset(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleSystemView, "get_fluid_rest_offsets", getFluidRestOffset, dstTensor);
    }

    bool setFluidRestOffset(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleSystemView, "set_fluid_rest_offsets", setFluidRestOffset, srcTensor, indexTensor);
    }

    bool getParticleContactOffset(const TensorDesc* dstTensor) const  override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleSystemView, "get_particle_contact_offsets", getParticleContactOffset, dstTensor);
    }

    bool setParticleContactOffset(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleSystemView, "set_particle_contact_offsets", setParticleContactOffset, srcTensor, indexTensor);
    }

    bool getWind(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleSystemView, "get_wind", getWind, dstTensor);
    }

    bool setWind(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleSystemView, "set_wind", setWind, srcTensor, indexTensor);
    }
    
    bool check() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleSystemView, "check", check);
    }

private:
    // NOTE: this is not exposed to Python, it's only used in the Deleter
    void release() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, IParticleSystemView, "_release", release);
    }
};


class PyParticleClothView : public IParticleClothView
{
public:
    struct Deleter
    {
        void operator()(IParticleClothView* pcView) const
        {
            if (pcView)
            {
                pcView->release();
            }
        }
    };

    uint32_t getCount() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IParticleClothView, "get_count", getCount);
    }

    uint32_t getMaxParticlesPerCloth() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IParticleClothView, "get_max_particles_per_cloth", getMaxParticlesPerCloth);
    }

    uint32_t getMaxSpringsPerCloth() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IParticleClothView, "get_max_springs_per_cloth", getMaxSpringsPerCloth);
    }

    bool getPositions(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleClothView, "get_positions",  getPositions, dstTensor);
    }

    bool setPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleClothView, "set_positions", setPositions, srcTensor, indexTensor);
    }

    bool getVelocities(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleClothView, "get_velocities",  getVelocities, dstTensor);
    }

    bool setVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleClothView, "set_velocities", setVelocities, srcTensor, indexTensor);
    }

    bool getMasses(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleClothView, "get_masses",  getMasses, dstTensor);
    }

    bool setMasses(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleClothView, "set_masses", setMasses, srcTensor, indexTensor);
    }

    bool getSpringDamping(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleClothView, "get_spring_damping",  getSpringDamping, dstTensor);
    }

    bool setSpringDamping(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleClothView, "set_spring_damping", setSpringDamping, srcTensor, indexTensor);
    }

    bool getSpringStiffness(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleClothView, "get_spring_stiffness",  getSpringStiffness, dstTensor);
    }

    bool setSpringStiffness(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleClothView, "set_spring_stiffness", setSpringStiffness, srcTensor, indexTensor);
    }

    bool check() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleClothView, "check", check);
    }

private:
    // NOTE: this is not exposed to Python, it's only used in the Deleter
    void release() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, IParticleClothView, "_release", release);
    }
};


class PyParticleMaterialView : public IParticleMaterialView
{
public:
    struct Deleter
    {
        void operator()(IParticleMaterialView* pmView) const
        {
            if (pmView)
            {
                pmView->release();
            }
        }
    };

    uint32_t getCount() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(uint32_t, IParticleMaterialView, "get_count", getCount);
    }

    bool getFriction(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleMaterialView, "get_friction",  getFriction, dstTensor);
    }

    bool setFriction(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleMaterialView, "set_friction", setFriction, srcTensor, indexTensor);
    }

    bool getDamping(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleMaterialView, "get_damping",  getDamping, dstTensor);
    }

    bool setDamping(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleMaterialView, "set_damping", setDamping, srcTensor, indexTensor);
    }

    bool getGravityScale(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleMaterialView, "get_gravity_scale",  getGravityScale, dstTensor);
    }

    bool setGravityScale(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleMaterialView, "set_gravity_scale", setGravityScale, srcTensor, indexTensor);
    }

    bool getLift(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleMaterialView, "get_lift",  getLift, dstTensor);
    }

    bool setLift(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleMaterialView, "set_lift", setLift, srcTensor, indexTensor);
    }

    bool getDrag(const TensorDesc* dstTensor) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleMaterialView, "get_drag",  getDrag, dstTensor);
    }

    bool setDrag(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleMaterialView, "set_drag", setDrag, srcTensor, indexTensor);
    }

    bool check() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, IParticleMaterialView, "check", check);
    }                       

private:
    // NOTE: this is not exposed to Python, it's only used in the Deleter
    void release() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, IParticleMaterialView, "_release", release);
    }
};

}
}
}

namespace
{
PYBIND11_MODULE(_physicsTensors, m)
{
    using namespace carb;
    using namespace omni::physics::tensors;

    m.doc() = "Physics tensor API";

    // ObjectType
    py::enum_<ObjectType>(m, "ObjectType", py::arithmetic(), "Types of Objects")
        .value("Invalid", ObjectType::eInvalid, "Invalid/unknown/uninitialized object type")
        .value("RigidBody", ObjectType::eRigidBody,
               "The object is a rigid body but not a link on an articulation")
        .value("Articulation", ObjectType::eArticulation, "The object is an articulation")
        .value("ArticulationLink", ObjectType::eArticulationLink, "The object is an articulation link but not the root link")
        .value("ArticulationRootLink", ObjectType::eArticulationRootLink, "The object is an articulation root link")
        .value("ArticulationJoint", ObjectType::eArticulationJoint, "The object is a joint")
        .export_values();

    // JointType
    py::enum_<JointType>(m, "JointType", py::arithmetic(), "Joint type.")
        .value("Invalid", JointType::eInvalid)
        .value("Fixed", JointType::eFixed)
        .value("Revolute", JointType::eRevolute)
        .value("Prismatic", JointType::ePrismatic)
        .value("Spherical", JointType::eSpherical)
        .export_values();

    // DofType
    py::enum_<DofType>(m, "DofType", py::arithmetic(), "DOF type.")
        .value("Invalid", DofType::eInvalid)
        .value("Rotation", DofType::eRotation)
        .value("Translation", DofType::eTranslation)
        .export_values();

    // DofMotion
    py::enum_<DofMotion>(m, "DofMotion", py::arithmetic(), "DOF motion type.")
        .value("Invalid", DofMotion::eInvalid)
        .value("Free", DofMotion::eFree)
        .value("Limited", DofMotion::eLimited)
        .value("Locked", DofMotion::eLocked)
        .export_values();

    // DofDrive
    py::enum_<DofDriveType>(m, "DofDriveType", py::arithmetic(), "DOF drive type.")
        .value("None", DofDriveType::eNone)
        .value("Force", DofDriveType::eForce)
        .value("Acceleration", DofDriveType::eAcceleration)
        .export_values();

    // TensorDataType
    py::enum_<TensorDataType>(m, "TensorDataType", py::arithmetic(), "Defines the data type of tensors.")
        .value("float32", TensorDataType::eFloat32)
        .value("float64", TensorDataType::eFloat64)
        .value("int8", TensorDataType::eInt8)
        .value("int16", TensorDataType::eInt16)
        .value("int32", TensorDataType::eInt32)
        .value("int64", TensorDataType::eInt64)
        .value("uint8", TensorDataType::eUint8)
        .value("uint16", TensorDataType::eUint16)
        .value("uint32", TensorDataType::eUint32)
        .value("uint64", TensorDataType::eUint64)
        .export_values();

    // TensorDesc
    py::class_<TensorDesc>(m, "TensorDesc", "Tensor descriptor.")
        .def(py::init<>())
        .def_readwrite("dtype", &TensorDesc::dtype, "data type")
        .def_readwrite("device", &TensorDesc::device, "device")
        .def_readonly("ndim", &TensorDesc::numDims, "number of dimensions")
        .def_property("shape",
                      [](const TensorDesc& self) {
                          py::tuple shape = py::tuple(self.numDims);
                          for (int i = 0; i < self.numDims; i++)
                          {
                              shape[i] = self.dims[i];
                          }
                          return shape;
                      },
                      [](TensorDesc& self, const std::vector<int>& shapev) {
                          self.numDims = int(shapev.size());
                          for (int i = 0; i < self.numDims; i++)
                          {
                              self.dims[i] = shapev[i];
                          }
                      },
                      "tensor shape")
        .def_readonly("data_ptr", &TensorDesc::data, "pointer to buffer")
        .def_property("data_address", [](const TensorDesc& self) { return reinterpret_cast<size_t>(self.data); },
                      [](TensorDesc& self, size_t address) { self.data = reinterpret_cast<void*>(address); },
                      "address of data")
        .def_readwrite("own_data", &TensorDesc::ownData, "flag for ownership");

    py::class_<DofProperties>(m, "DofProperties", "Properties of a degree-of-freedom (DOF)")
        .def(py::init<>())
        .def_readwrite("type", &DofProperties::type,
                       R"pbdoc(
                           Type of joint (read only) (:obj:`omni.physics.tensor.DofDriveType`))pbdoc")
        .def_readonly(
            "has_limits", &DofProperties::hasLimits, "Flags whether the DOF has limits (read only) (:obj:`bool`)")
        .def_readonly(
            "lower", &DofProperties::lower, 
            "lower limit of DOF. In stage-units for linear and radians for angular limits, respectively. (read only) (:obj:`float`)")
        .def_readonly(
            "upper", &DofProperties::upper, 
            "upper limit of DOF. In stage-units for linear and radians for angular limits, respectively. (read only) (:obj:`float`)")
        .def_readwrite("drive_mode", &DofProperties::driveMode,
                       "Drive mode for the DOF (:obj:`omni.physics.tensors.DriveMode`)")
        .def_readwrite("max_velocity", &DofProperties::maxVelocity,
                       "Maximum velocity of DOF. Units are stage_units / second for linear DOFs, and radians / second for angular DOFs (:obj:`float`)")
        .def_readwrite("max_effort", &DofProperties::maxEffort, 
            "Maximum effort of DOF.  Units are mass * stage_units / (second * second), i.e. a force for linear DOFs,  and mass * stage_units * stage_units / (second * second), i.e. a torque for angular DOFs (:obj:`float`)")
        .def_readwrite("stiffness", &DofProperties::stiffness, 
            "Stiffness of DOF. stiffness units are mass / (second * second) for linear DOFS, and mass * stage_units * stage_units / (second * second * radians) for angular DOFS (:obj:`float`)")
        .def_readwrite("damping", &DofProperties::damping, 
            "Damping of DOF. damping units are mass / second for linear DOFs, and  mass * stage_units * stage_units / (second * radians) for angular DOFS (:obj:`float`)")
        .def(py::pickle(
            [](const DofProperties& props)
            {
                return py::make_tuple(props.type, props.hasLimits, props.lower, props.upper, props.driveMode,
                                      props.maxVelocity, props.maxEffort, props.stiffness, props.damping);
            },
            [](py::tuple t)
            {
                DofProperties props;
                props.type = t[0].cast<DofType>();
                props.hasLimits = t[1].cast<bool>();
                props.lower = t[2].cast<float>();
                props.upper = t[3].cast<float>();
                props.driveMode = t[4].cast<DofDriveType>();
                props.maxVelocity = t[5].cast<float>();
                props.maxEffort = t[6].cast<float>();
                props.stiffness = t[7].cast<float>();
                props.damping = t[8].cast<float>();
                return props;
            }));
            
        // numpy dtypes
        PYBIND11_NUMPY_DTYPE(DofProperties, type, hasLimits, lower, upper, driveMode, maxVelocity, maxEffort, stiffness, damping);


    // ArticulationMetatype
    py::class_<IArticulationMetatype, PyArticulationMetatype, std::unique_ptr<IArticulationMetatype, py::nodelete>>(
        m, "ArticulationMetatype")
        .def(py::init<>())
        .def_property_readonly("link_count", &IArticulationMetatype::getLinkCount)
        .def_property_readonly("joint_count", &IArticulationMetatype::getJointCount)
        .def_property_readonly("dof_count", &IArticulationMetatype::getDofCount)
        .def_property_readonly("link_names",
                               [](const IArticulationMetatype& self) {
                                   uint32_t n = self.getLinkCount();
                                   std::vector<std::string> names(n);
                                   for (uint32_t i = 0; i < n; i++)
                                   {
                                       const char* name = self.getLinkName(i);
                                       if (name)
                                       {
                                           names[i] = name;
                                       }
                                   }
                                   return names;
                               })
        .def_property_readonly("link_parents",
                               [](const IArticulationMetatype& self) {
                                   uint32_t n = self.getLinkCount();
                                   std::vector<std::string> names(n);
                                   for (uint32_t i = 0; i < n; i++)
                                   {
                                       const char* name = self.getLinkParentName(i);
                                       if (name)
                                       {
                                           names[i] = name;
                                       }
                                   }
                                return names;
                               })
        .def_property_readonly("joint_names",
                               [](const IArticulationMetatype& self) {
                                   uint32_t n = self.getJointCount();
                                   std::vector<std::string> names(n);
                                   for (uint32_t i = 0; i < n; i++)
                                   {
                                       const char* name = self.getJointName(i);
                                       if (name)
                                       {
                                           names[i] = name;
                                       }
                                   }
                                   return names;
                               })
        .def_property_readonly("dof_names",
                               [](const IArticulationMetatype& self) {
                                   uint32_t n = self.getDofCount();
                                   std::vector<std::string> names(n);
                                   for (uint32_t i = 0; i < n; i++)
                                   {
                                       const char* name = self.getDofName(i);
                                       if (name)
                                       {
                                           names[i] = name;
                                       }
                                   }
                                   return names;
                               })
        .def_property_readonly("link_indices",
                               [](const IArticulationMetatype& self) {
                                   uint32_t n = self.getLinkCount();
                                   std::map<std::string, uint32_t> dict;
                                   for (uint32_t i = 0; i < n; i++)
                                   {
                                       const char* name = self.getLinkName(i);
                                       if (name)
                                       {
                                           dict[name] = i;
                                       }
                                   }
                                   return dict;
                               })
        .def_property_readonly("link_parent_indices",
                               [](const IArticulationMetatype& self) {
                                   uint32_t n = self.getLinkCount();
                                   std::map<std::string, uint32_t> dict;
                                   for (uint32_t i = 0; i < n; i++)
                                   {
                                       const char* name = self.getLinkName(i);
                                       if (name)
                                       {
                                           uint32_t index = self.findLinkParentIndex(name);
                                           if (index != -1)
                                               dict[name] = index;
                                       }
                                   }
                                   return dict;
                               })
        .def_property_readonly("joint_indices",
                               [](const IArticulationMetatype& self) {
                                   uint32_t n = self.getJointCount();
                                   std::map<std::string, uint32_t> dict;
                                   for (uint32_t i = 0; i < n; i++)
                                   {
                                       const char* name = self.getJointName(i);
                                       if (name)
                                       {
                                           dict[name] = i;
                                       }
                                   }
                                   return dict;
                               })
        .def_property_readonly("dof_indices",
                               [](const IArticulationMetatype& self) {
                                   uint32_t n = self.getDofCount();
                                   std::map<std::string, uint32_t> dict;
                                   for (uint32_t i = 0; i < n; i++)
                                   {
                                       const char* name = self.getDofName(i);
                                       if (name)
                                       {
                                           dict[name] = i;
                                       }
                                   }
                                   return dict;
                               })
        .def_property_readonly("joint_types",
                               [](const IArticulationMetatype& self) {
                                   uint32_t n = self.getJointCount();
                                   std::vector<JointType> types(n);
                                   for (uint32_t i = 0; i < n; i++)
                                   {
                                       types[i] = self.getJointType(i);
                                   }
                                   return types;
                               })
        .def_property_readonly("joint_dof_offsets",
                               [](const IArticulationMetatype& self) {
                                   uint32_t n = self.getJointCount();
                                   std::vector<uint32_t> offsets(n);
                                   for (uint32_t i = 0; i < n; i++)
                                   {
                                       offsets[i] = self.getJointDofOffset(i);
                                   }
                                   return offsets;
                               })
        .def_property_readonly("joint_dof_counts",
                               [](const IArticulationMetatype& self) {
                                   uint32_t n = self.getJointCount();
                                   std::vector<uint32_t> counts(n);
                                   for (uint32_t i = 0; i < n; i++)
                                   {
                                       counts[i] = self.getJointDofCount(i);
                                   }
                                   return counts;
                               })
        .def_property_readonly("dof_types",
                               [](const IArticulationMetatype& self) {
                                   uint32_t n = self.getDofCount();
                                   std::vector<DofType> types(n);
                                   for (uint32_t i = 0; i < n; i++)
                                   {
                                       types[i] = self.getDofType(i);
                                   }
                                   return types;
                               })
        .def_property_readonly("fixed_base", &IArticulationMetatype::getFixedBase);


    // ArticulationView
    py::class_<IArticulationView, PyArticulationView, std::unique_ptr<IArticulationView, PyArticulationView::Deleter>>(
        m, "ArticulationView")
        .def(py::init<>())
        .def_property_readonly("count", &IArticulationView::getCount)
        .def_property_readonly("max_links", &IArticulationView::getMaxLinks)
        .def_property_readonly("max_dofs", &IArticulationView::getMaxDofs)
    
        .def_property_readonly("max_shapes", &IArticulationView::getMaxShapes)
        .def_property_readonly("max_fixed_tendons", &IArticulationView::getMaxFixedTendons)
        .def_property_readonly("max_spatial_tendons", &IArticulationView::getMaxSpatialTendons)
        .def_property_readonly("is_homogeneous", &IArticulationView::isHomogeneous)
        .def_property_readonly("prim_paths",
                               [](const IArticulationView* aview) {
                                   uint32_t count = aview->getCount();
                                   std::vector<std::string> primPaths(count);
                                   for (uint32_t i = 0; i < count; ++i)
                                   {
                                       primPaths[i] = aview->getUsdPrimPath(i);
                                   }
                                   return primPaths;
                               })
        .def_property_readonly("dof_paths",
                               [](const IArticulationView* aview) {
                                   uint32_t count = aview->getCount();
                                   uint32_t dofCount = aview->getMaxDofs();
                                   std::vector<std::vector<std::string>> dofPaths(count, std::vector<std::string>(dofCount));
                                   for (uint32_t i = 0; i < count; ++i)
                                   {
                                       for (uint32_t j = 0; j < dofCount; ++j)
                                       {
                                           dofPaths[i][j] = aview->getUsdDofPath(i, j);
                                       }
                                   }
                                   return dofPaths;
                               })
        .def_property_readonly("link_paths",
                               [](const IArticulationView* aview) {
                                   uint32_t count = aview->getCount();
                                   uint32_t linkCount = aview->getMaxLinks();
                                   std::vector<std::vector<std::string>> linkPaths(count, std::vector<std::string>(linkCount));
                                   for (uint32_t i = 0; i < count; ++i)
                                   {
                                       for (uint32_t j = 0; j < linkCount; ++j)
                                       {
                                           linkPaths[i][j] = aview->getUsdLinkPath(i, j);
                                       }
                                   }
                                   return linkPaths;
                               })
        .def_property_readonly("shared_metatype", &IArticulationView::getSharedMetatype)
        .def_property_readonly("jacobian_shape",
                               [](const IArticulationView* aview) -> py::object {
                                   uint32_t numRows = 0, numCols = 0;
                                   if (aview && aview->getJacobianShape(&numRows, &numCols))
                                   {
                                       return py::make_tuple(numRows, numCols);
                                   }
                                   return py::none();
                               })
        .def_property_readonly("mass_matrix_shape", // deprecated
                               [](const IArticulationView* aview) -> py::object {
                                   uint32_t numRows = 0, numCols = 0;
                                   if (aview && aview->getMassMatrixShape(&numRows, &numCols))
                                   {
                                       return py::make_tuple(numRows, numCols);
                                   }
                                   return py::none();
                               })
        .def_property_readonly("generalized_mass_matrix_shape",
                               [](const IArticulationView* aview) -> py::object {
                                   uint32_t numRows = 0, numCols = 0;
                                   if (aview && aview->getGeneralizedMassMatrixShape(&numRows, &numCols))
                                   {
                                       return py::make_tuple(numRows, numCols);
                                   }
                                   return py::none();
                               })
        .def("get_metatype", &IArticulationView::getMetatype, py::return_value_policy::reference)
        .def("get_dof_types", &IArticulationView::getDofTypes)
        .def("get_dof_motions", &IArticulationView::getDofMotions)
        .def("get_dof_limits", &IArticulationView::getDofLimits)
        .def("get_drive_types", &IArticulationView::getDriveTypes)
        .def("get_dof_stiffnesses", &IArticulationView::getDofStiffnesses)
        .def("get_dof_dampings", &IArticulationView::getDofDampings)
        .def("get_dof_max_forces", &IArticulationView::getDofMaxForces)
        .def("get_dof_drive_model_properties", &IArticulationView::getDofDriveModelProperties)
        .def("get_dof_friction_coefficients", &IArticulationView::getDofFrictionCoefficients) // deprecated
        .def("get_dof_friction_properties", &IArticulationView::getDofFrictionProperties)
        .def("get_dof_max_velocities", &IArticulationView::getDofMaxVelocities)
        .def("get_dof_armatures", &IArticulationView::getDofArmatures)
        .def("set_dof_limits", &IArticulationView::setDofLimits)
        .def("set_dof_stiffnesses", &IArticulationView::setDofStiffnesses)
        .def("set_dof_dampings", &IArticulationView::setDofDampings)
        .def("set_dof_max_forces", &IArticulationView::setDofMaxForces)
        .def("set_dof_drive_model_properties", &IArticulationView::setDofDriveModelProperties)
        .def("set_dof_friction_coefficients", &IArticulationView::setDofFrictionCoefficients) // deprecated
        .def("set_dof_friction_properties", &IArticulationView::setDofFrictionProperties)
        .def("set_dof_max_velocities", &IArticulationView::setDofMaxVelocities)
        .def("set_dof_armatures", &IArticulationView::setDofArmatures)
        .def("get_dof_actuation_forces", &IArticulationView::getDofActuationForces)
        .def("get_dof_projected_joint_forces", &IArticulationView::getDofProjectedJointForces)
        .def("get_link_transforms", &IArticulationView::getLinkTransforms)
        .def("get_link_velocities", &IArticulationView::getLinkVelocities)
        .def("get_link_accelerations", &IArticulationView::getLinkAccelerations)
        .def("get_root_transforms", &IArticulationView::getRootTransforms)
        .def("get_root_velocities", &IArticulationView::getRootVelocities)
        .def("set_root_transforms", &IArticulationView::setRootTransforms)
        .def("set_root_velocities", &IArticulationView::setRootVelocities)
        .def("get_dof_positions", &IArticulationView::getDofPositions)
        .def("get_dof_velocities", &IArticulationView::getDofVelocities)
        .def("set_dof_positions", &IArticulationView::setDofPositions)
        .def("set_dof_velocities", &IArticulationView::setDofVelocities)
        .def("set_dof_actuation_forces", &IArticulationView::setDofActuationForces)
        .def("set_dof_position_targets", &IArticulationView::setDofPositionTargets)
        .def("set_dof_velocity_targets", &IArticulationView::setDofVelocityTargets)
        .def("get_dof_position_targets", &IArticulationView::getDofPositionTargets)
        .def("get_dof_velocity_targets", &IArticulationView::getDofVelocityTargets)
        .def("get_jacobians", &IArticulationView::getJacobians)
        .def("get_mass_matrices", &IArticulationView::getMassMatrices) // deprecated
        .def("get_generalized_mass_matrices", &IArticulationView::getGeneralizedMassMatrices)
        .def("get_coriolis_and_centrifugal_forces", &IArticulationView::getCoriolisAndCentrifugalForces) // deprecated
        .def("get_coriolis_and_centrifugal_compensation_forces", &IArticulationView::getCoriolisAndCentrifugalCompensationForces)
        .def("get_generalized_gravity_forces", &IArticulationView::getGeneralizedGravityForces) // deprecated
        .def("get_gravity_compensation_forces", &IArticulationView::getGravityCompensationForces)
        .def("get_articulation_centroidal_momentum", &IArticulationView::getArticulationCentroidalMomentum)
        .def("get_articulation_mass_center", &IArticulationView::getArticulationMassCenter)
        .def("get_link_incoming_joint_force", &IArticulationView::getLinkIncomingJointForce)
        .def("apply_forces_and_torques_at_position", &IArticulationView::applyForcesAndTorquesAtPosition,
             py::arg("force_data") = nullptr, py::arg("torque_data") = nullptr, py::arg("position_data") = nullptr,
             py::arg("indices") = nullptr, py::arg("is_global") = true)
        .def("get_masses", &IArticulationView::getMasses)
        .def("get_inv_masses", &IArticulationView::getInvMasses)
        .def("get_coms", &IArticulationView::getCOMs)
        .def("get_inertias", &IArticulationView::getInertias)
        .def("get_inv_inertias", &IArticulationView::getInvInertias)
        .def("get_disable_gravities", &IArticulationView::getDisableGravities)
        .def("set_masses", &IArticulationView::setMasses)
        .def("set_coms", &IArticulationView::setCOMs)
        .def("set_inertias", &IArticulationView::setInertias)
        .def("set_disable_gravities", &IArticulationView::setDisableGravities)
        .def("get_material_properties", &IArticulationView::getMaterialProperties)
        .def("get_contact_offsets", &IArticulationView::getContactOffsets)
        .def("get_rest_offsets", &IArticulationView::getRestOffsets)
        .def("set_material_properties", &IArticulationView::setMaterialProperties)
        .def("set_contact_offsets", &IArticulationView::setContactOffsets)
        .def("set_rest_offsets", &IArticulationView::setRestOffsets)
        .def("get_fixed_tendon_stiffnesses", &IArticulationView::getFixedTendonStiffnesses)
        .def("get_fixed_tendon_dampings", &IArticulationView::getFixedTendonDampings)
        .def("get_fixed_tendon_limit_stiffnesses", &IArticulationView::getFixedTendonLimitStiffnesses)
        .def("get_fixed_tendon_limits", &IArticulationView::getFixedTendonLimits)
        .def("get_fixed_tendon_rest_lengths", &IArticulationView::getFixedTendonfixedSpringRestLengths)
        .def("get_fixed_tendon_offsets", &IArticulationView::getFixedTendonOffsets)
        .def("get_spatial_tendon_stiffnesses", &IArticulationView::getSpatialTendonStiffnesses)
        .def("get_spatial_tendon_dampings", &IArticulationView::getSpatialTendonDampings)
        .def("get_spatial_tendon_limit_stiffnesses", &IArticulationView::getSpatialTendonLimitStiffnesses)
        .def("get_spatial_tendon_offsets", &IArticulationView::getSpatialTendonOffsets)
        .def("set_fixed_tendon_properties", &IArticulationView::setFixedTendonProperties)
        .def("set_spatial_tendon_properties", &IArticulationView::setSpatialTendonProperties)
        .def("check", &IArticulationView::check);


    py::class_<Transform>(m, "Transform", "Represents a 3D transform in the system")
        .def_readwrite("p", &Transform::p, "Position as a tuple of (x,y,z) (:obj:`carb._carb.Float3`)")
        .def_readwrite(
            "r", &Transform::r,
            R"pbdoc(Rotation Quaternion, represented in the format :math:`x\hat{i} + y\hat{j} + z\hat{k} + w` (:obj:`carb._carb.Float4`))pbdoc")
        .def(py::init(
                 [](const carb::Float3& p, const carb::Float4& r)
                 {
                     Transform transform;
                     transform.p = p;
                     transform.r = r;
                     return transform;
                 }),
             py::arg("p") = nullptr, py::arg("r") = nullptr, "Initialize from a position and a rotation quaternion")
        .def(py::init<>(), "Initialize from another Transform object")
        .def_property_readonly_static(
            "dtype", [](const py::object&) { return py::dtype::of<Transform>(); }, "return the numpy structured dtype")
        .def_static(
            "from_buffer",
            [](py::buffer buf) -> py::object
            {
                py::buffer_info info = buf.request();
                if (info.ptr != nullptr)
                {
                    if ((info.itemsize == 7 * sizeof(float)) ||
                        (info.itemsize == sizeof(float) && info.ndim > 0 && info.shape[info.ndim - 1] >= 7))
                    {
                        float* data = static_cast<float*>(info.ptr);
                        Transform tx;
                        tx.p = carb::Float3{ data[0], data[1], data[2] };
                        tx.r = carb::Float4{ data[3], data[4], data[5], data[6] };
                        return py::cast(tx);
                    }
                }
                return py::none();
            },
            "assign a transform from an array of 7 values [p.x, p.y, p.z, r.x, r.y, r.z, r.w]")

        .def(py::pickle([](const Transform& tx)
                        { return py::make_tuple(tx.p.x, tx.p.y, tx.p.z, tx.r.x, tx.r.y, tx.r.z, tx.r.w); },
                        [](py::tuple t)
                        {
                            Transform tx;
                            tx.p = { t[0].cast<float>(), t[1].cast<float>(), t[2].cast<float>() };
                            tx.r = { t[3].cast<float>(), t[4].cast<float>(), t[5].cast<float>(), t[6].cast<float>() };
                            return tx;
                        }))
        .def("__repr__",
             [](const Transform& tx) {
                 return fmt::format(
                     "({}, {}, {}), ({}, {}, {}, {})", tx.p.x, tx.p.y, tx.p.z, tx.r.x, tx.r.y, tx.r.z, tx.r.w);
             })
        .def("__eq__",
             [](const Transform& a, const Transform& b)
             {
                 return a.p.x == b.p.x && a.p.y == b.p.y && a.p.z == b.p.z && a.r.x == b.r.x && a.r.y == b.r.y &&
                        a.r.z == b.r.z && a.r.w == b.r.w;
             });

    py::class_<Velocity>(m, "Velocity", "Linear and angular velocity")
        .def_readwrite("linear", &Velocity::linear, "Linear 3D velocity as a tuple (x,y,z) , (:obj:`carb._carb.Float3`)")
        .def_readwrite(
            "angular", &Velocity::angular, "Angular 3D velocity as a tuple (x,y,z), (:obj:`carb._carb.Float3`)")
        .def(py::init(
                 [](const carb::Float3& linear, const carb::Float3& angular)
                 {
                     Velocity vel;
                     vel.linear = linear;
                     vel.angular = angular;
                     return vel;
                 }),
             py::arg("linear") = nullptr, py::arg("angular") = nullptr,
             "initialize from a linear velocity and angular velocity")
        .def(py::init<>(), "initialize from another Velocity Object")
        .def_property_readonly_static(
            "dtype",
            [](const py::object&)
            {
                // return the numpy structured dtype
                return py::dtype::of<Velocity>();
            },
            "return the numpy structured dtype")
        .def(py::pickle(
            [](const Velocity& v)
            { return py::make_tuple(v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z); },
            [](py::tuple t)
            {
                Velocity v;
                v.linear = { t[0].cast<float>(), t[1].cast<float>(), t[2].cast<float>() };
                v.angular = { t[3].cast<float>(), t[4].cast<float>(), t[5].cast<float>() };
                return v;
            }))
        .def("__repr__",
             [](const Velocity& v)
             {
                 return fmt::format("({}, {}, {}), ({}, {}, {})", v.linear.x, v.linear.y, v.linear.z, v.angular.x,
                                    v.angular.y, v.angular.z);
             });

    // RigidBodyView
    py::class_<IRigidBodyView, PyRigidBodyView, std::unique_ptr<IRigidBodyView, PyRigidBodyView::Deleter>>(
        m, "RigidBodyView")
        .def(py::init<>())
        .def_property_readonly("count", &IRigidBodyView::getCount)
        .def_property_readonly("max_shapes", &IRigidBodyView::getMaxShapes)
        .def_property_readonly("prim_paths",
                               [](const IRigidBodyView* rbview) {
                                   uint32_t count = rbview->getCount();
                                   std::vector<std::string> primPaths(count);
                                   for (uint32_t i = 0; i < count; ++i)
                                   {
                                       primPaths[i] = rbview->getUsdPrimPath(i);
                                   }
                                   return primPaths;
                               })
        .def("get_transforms", &IRigidBodyView::getTransforms)
        .def("get_velocities", &IRigidBodyView::getVelocities)
        .def("get_accelerations", &IRigidBodyView::getAccelerations)
        .def("set_kinematic_targets", &IRigidBodyView::setKinematicTargets)
        .def("set_transforms", &IRigidBodyView::setTransforms)
        .def("set_velocities", &IRigidBodyView::setVelocities)
        .def("apply_forces", &IRigidBodyView::applyForces)
        .def("apply_forces_and_torques_at_position", &IRigidBodyView::applyForcesAndTorquesAtPosition,
             py::arg("force_data") = nullptr, py::arg("torque_data") = nullptr, py::arg("position_data") = nullptr,
             py::arg("indices") = nullptr, py::arg("is_global") = true)
        .def("get_masses", &IRigidBodyView::getMasses)
        .def("get_inv_masses", &IRigidBodyView::getInvMasses)
        .def("get_coms", &IRigidBodyView::getCOMs)
        .def("get_inertias", &IRigidBodyView::getInertias)
        .def("get_inv_inertias", &IRigidBodyView::getInvInertias)
        .def("get_disable_gravities", &IRigidBodyView::getDisableGravities)
        .def("get_disable_simulations", &IRigidBodyView::getDisableSimulations)
        .def("set_masses", &IRigidBodyView::setMasses)
        .def("set_coms", &IRigidBodyView::setCOMs)
        .def("set_inertias", &IRigidBodyView::setInertias)
        .def("set_disable_gravities", &IRigidBodyView::setDisableGravities)
        .def("set_disable_simulations", &IRigidBodyView::setDisableSimulations)
        .def("get_material_properties", &IRigidBodyView::getMaterialProperties)
        .def("get_contact_offsets", &IRigidBodyView::getContactOffsets)
        .def("get_rest_offsets", &IRigidBodyView::getRestOffsets)
        .def("set_material_properties", &IRigidBodyView::setMaterialProperties)
        .def("set_contact_offsets", &IRigidBodyView::setContactOffsets)
        .def("set_rest_offsets", &IRigidBodyView::setRestOffsets)
        .def("check", &IRigidBodyView::check);

    // SoftBodyView
    py::class_<ISoftBodyView, PySoftBodyView, std::unique_ptr<ISoftBodyView, PySoftBodyView::Deleter>>(
        m, "SoftBodyView")
        .def(py::init<>())
        .def_property_readonly("count", &ISoftBodyView::getCount)
        .def_property_readonly("max_elements_per_body", &ISoftBodyView::getMaxElementsPerBody)
        .def_property_readonly("max_vertices_per_body", &ISoftBodyView::getMaxVerticesPerBody)
        .def_property_readonly("max_sim_elements_per_body", &ISoftBodyView::getMaxSimElementsPerBody)
        .def_property_readonly("max_sim_vertices_per_body", &ISoftBodyView::getMaxSimVerticesPerBody)
        .def("get_element_rest_poses", &ISoftBodyView::getElementRestPoses)
        .def("get_element_rotations", &ISoftBodyView::getElementRotations)
        .def("get_element_indices", &ISoftBodyView::getElementIndices)
        .def("get_element_stresses", &ISoftBodyView::getElementStresses)
        .def("get_element_deformation_gradients", &ISoftBodyView::getElementDeformationGradients)
        .def("get_nodal_positions", &ISoftBodyView::getNodalPositions)
        .def("get_sim_element_indices", &ISoftBodyView::getSimElementIndices)  
        .def("get_sim_element_rest_poses", &ISoftBodyView::getSimElementRestPoses)
        .def("get_sim_element_rotations", &ISoftBodyView::getSimElementRotations)
        .def("get_sim_element_stresses", &ISoftBodyView::getSimElementStresses)
        .def("get_sim_element_deformation_gradients", &ISoftBodyView::getSimElementDeformationGradients)      
        .def("get_sim_nodal_positions", &ISoftBodyView::getSimNodalPositions)
        .def("set_sim_nodal_positions", &ISoftBodyView::setSimNodalPositions)
        .def("get_sim_nodal_velocities", &ISoftBodyView::getSimNodalVelocities)
        .def("set_sim_nodal_velocities", &ISoftBodyView::setSimNodalVelocities)
        .def("get_sim_kinematic_targets", &ISoftBodyView::getSimKinematicTargets)
        .def("set_sim_kinematic_targets", &ISoftBodyView::setSimKinematicTargets)
        .def("get_transforms", &ISoftBodyView::getTransforms)
        .def("check", &ISoftBodyView::check);

    // SoftBodyMaterialView
    py::class_<ISoftBodyMaterialView, PySoftBodyMaterialView, std::unique_ptr<ISoftBodyMaterialView, PySoftBodyMaterialView::Deleter>>(
        m, "SoftBodyMaterialView")
        .def(py::init<>())
        .def_property_readonly("count", &ISoftBodyMaterialView::getCount)
        .def("get_youngs_modulus", &ISoftBodyMaterialView::getYoungsModulus)
        .def("set_youngs_modulus", &ISoftBodyMaterialView::setYoungsModulus)
        .def("get_poissons_ratio", &ISoftBodyMaterialView::getPoissonsRatio)
        .def("set_poissons_ratio", &ISoftBodyMaterialView::setPoissonsRatio)
        .def("get_dynamic_friction", &ISoftBodyMaterialView::getDynamicFriction)
        .def("set_dynamic_friction", &ISoftBodyMaterialView::setDynamicFriction)
        .def("get_damping", &ISoftBodyMaterialView::getDamping)
        .def("set_damping", &ISoftBodyMaterialView::setDamping)
        .def("get_damping_scale", &ISoftBodyMaterialView::getDampingScale)
        .def("set_damping_scale", &ISoftBodyMaterialView::setDampingScale)
        .def("check", &ISoftBodyMaterialView::check);

    // DeformableBodyView
    py::class_<IDeformableBodyView, PyDeformableBodyView, std::unique_ptr<IDeformableBodyView, PyDeformableBodyView::Deleter>>(m, "DeformableBodyView")
        .def(py::init<>())
        .def_property_readonly("count", &IDeformableBodyView::getCount)
        .def_property_readonly("prim_paths",
            [](const IDeformableBodyView* dbview) {
                uint32_t count = dbview->getCount();
                std::vector<std::string> primPaths(count);
                for (uint32_t i = 0; i < count; ++i)
                {
                    primPaths[i] = dbview->getUsdPrimPath(i);
                }
                return primPaths;
            })
        .def_property_readonly("simulation_mesh_prim_paths",
            [](const IDeformableBodyView* dbview) {
                uint32_t count = dbview->getCount();
                std::vector<std::string> simPaths(count);
                for (uint32_t i = 0; i < count; ++i)
                {
                    simPaths[i] = dbview->getUsdSimulationMeshPrimPath(i);
                }
                return simPaths;
            })
        .def_property_readonly("collision_mesh_prim_paths",
            [](const IDeformableBodyView* dbview) {
                uint32_t count = dbview->getCount();
                std::vector<std::string> collPaths(count);
                for (uint32_t i = 0; i < count; ++i)
                {
                    collPaths[i] = dbview->getUsdCollisionMeshPrimPath(i);
                }
                return collPaths;
            })
        .def_property_readonly("max_simulation_elements_per_body", &IDeformableBodyView::getMaxSimulationElementsPerBody)
        .def_property_readonly("max_simulation_nodes_per_body", &IDeformableBodyView::getMaxSimulationNodesPerBody)
        .def_property_readonly("max_rest_nodes_per_body", &IDeformableBodyView::getMaxRestNodesPerBody)
        .def_property_readonly("max_collision_elements_per_body", &IDeformableBodyView::getMaxCollisionElementsPerBody)
        .def_property_readonly("max_collision_nodes_per_body", &IDeformableBodyView::getMaxCollisionNodesPerBody)
        .def_property_readonly("num_nodes_per_element", &IDeformableBodyView::getNumNodesPerElement)
        .def("get_simulation_element_indices", &IDeformableBodyView::getSimulationElementIndices)
        .def("get_simulation_nodal_positions", &IDeformableBodyView::getSimulationNodalPositions)
        .def("set_simulation_nodal_positions", &IDeformableBodyView::setSimulationNodalPositions)
        .def("get_simulation_nodal_velocities", &IDeformableBodyView::getSimulationNodalVelocities)
        .def("set_simulation_nodal_velocities", &IDeformableBodyView::setSimulationNodalVelocities)
        .def("get_simulation_nodal_kinematic_targets", &IDeformableBodyView::getSimulationNodalKinematicTargets)
        .def("set_simulation_nodal_kinematic_targets", &IDeformableBodyView::setSimulationNodalKinematicTargets)
        .def("get_rest_element_indices", &IDeformableBodyView::getRestElementIndices)
        .def("get_rest_nodal_positions", &IDeformableBodyView::getRestNodalPositions)
        .def("get_collision_element_indices", &IDeformableBodyView::getCollisionElementIndices)
        .def("get_collision_nodal_positions", &IDeformableBodyView::getCollisionNodalPositions)
        .def("get_transforms", &IDeformableBodyView::getTransforms)
        .def("check", &IDeformableBodyView::check);

    // DeformableMaterialView
    py::class_<IDeformableMaterialView, PyDeformableMaterialView, std::unique_ptr<IDeformableMaterialView, PyDeformableMaterialView::Deleter>>(m, "DeformableMaterialView")
        .def(py::init<>())
        .def_property_readonly("count", &IDeformableMaterialView::getCount)
        .def("get_dynamic_friction", &IDeformableMaterialView::getDynamicFriction)
        .def("set_dynamic_friction", &IDeformableMaterialView::setDynamicFriction)
        .def("get_youngs_modulus", &IDeformableMaterialView::getYoungsModulus)
        .def("set_youngs_modulus", &IDeformableMaterialView::setYoungsModulus)
        .def("get_poissons_ratio", &IDeformableMaterialView::getPoissonsRatio)
        .def("set_poissons_ratio", &IDeformableMaterialView::setPoissonsRatio)
        .def("check", &IDeformableMaterialView::check);

    // RigidContactView
    py::class_<IRigidContactView, PyRigidContactView, std::unique_ptr<IRigidContactView, PyRigidContactView::Deleter>>(
        m, "RigidContactView")
        .def(py::init<>())
        .def_property_readonly("sensor_count", &IRigidContactView::getSensorCount)
        .def_property_readonly("filter_count", &IRigidContactView::getFilterCount)
        .def_property_readonly("max_contact_data_count", &IRigidContactView::getMaxContactDataCount)
        .def_property_readonly("sensor_paths",
                               [](const IRigidContactView* rcview) {
                                   const uint32_t count = rcview->getSensorCount();
                                   std::vector<std::string> primPaths(count);
                                   for (uint32_t i = 0; i < count; ++i)
                                   {
                                       const char* path = rcview->getUsdPrimPath(i);
                                       std::string str = path ? path : "";
                                       primPaths[i] = str;
                                   }
                                   return primPaths;
                               })
        .def_property_readonly("filter_paths",
                               [](const IRigidContactView* rcview) {
                                   const uint32_t numSensors = rcview->getSensorCount();
                                   const uint32_t numFilters = rcview->getFilterCount();
                                   std::vector<std::vector<std::string>> primPaths(numSensors);
                                   for (uint32_t i = 0; i < numSensors; ++i)
                                   {
                                       primPaths[i].resize(numFilters);
                                       for (uint32_t j = 0; j < numFilters; ++j)
                                       {
                                           const char* path = rcview->getFilterUsdPrimPath(i, j);
                                           std::string str = path ? path : "";
                                           primPaths[i][j] = str;
                                       }
                                   }
                                   return primPaths;
                               })
        .def_property_readonly("sensor_names",
                               [](const IRigidContactView* rcview) {
                                   const uint32_t count = rcview->getSensorCount();
                                   std::vector<std::string> primPaths(count);
                                   for (uint32_t i = 0; i < count; ++i)
                                   {
                                       const char* path = rcview->getUsdPrimName(i);
                                       std::string str = path ? path : "";
                                       primPaths[i] = str;
                                   }
                                   return primPaths;
                               })
        .def_property_readonly("filter_names",
                               [](const IRigidContactView* rcview) {
                                   const uint32_t numSensors = rcview->getSensorCount();
                                   const uint32_t numFilters = rcview->getFilterCount();
                                   std::vector<std::vector<std::string>> primPaths(numSensors);
                                   for (uint32_t i = 0; i < numSensors; ++i)
                                   {
                                       primPaths[i].resize(numFilters);
                                       for (uint32_t j = 0; j < numFilters; ++j)
                                       {
                                           const char* path = rcview->getFilterUsdPrimName(i, j);
                                           std::string str = path ? path : "";
                                           primPaths[i][j] = str;
                                       }
                                   }
                                   return primPaths;
                               })
        .def("get_net_contact_forces", &IRigidContactView::getNetContactForces)
        .def("get_contact_force_matrix", &IRigidContactView::getContactForceMatrix)
        .def("get_contact_data", &IRigidContactView::getContactData)
        .def("get_friction_data", &IRigidContactView::getFrictionData)
        .def("check", &IRigidContactView::check);

    // SdfShapeView
    py::class_<ISdfShapeView, PySdfShapeView, std::unique_ptr<ISdfShapeView, PySdfShapeView::Deleter>>(
        m, "SdfShapeView")
        .def(py::init<>())
        .def_property_readonly("count", &ISdfShapeView::getCount)
        .def_property_readonly("max_num_points", &ISdfShapeView::getMaxNumPoints)
        .def_property_readonly("object_paths",
                               [](const ISdfShapeView* sdfview) {
                                   uint32_t count = sdfview->getCount();
                                   std::vector<std::string> primPaths(count);
                                   for (uint32_t i = 0; i < count; ++i)
                                   {
                                       primPaths[i] = sdfview->getUsdPrimPath(i);
                                   }
                                   return primPaths;
                               })
        .def("get_sdf_and_gradients", &ISdfShapeView::getSdfAndGradients)
        .def("check", &ISdfShapeView::check);

    // ParticleSystemView
    py::class_<IParticleSystemView, PyParticleSystemView, std::unique_ptr<IParticleSystemView, PyParticleSystemView::Deleter>>(
        m, "ParticleSystemView")
        .def(py::init<>())
        .def_property_readonly("count", &IParticleSystemView::getCount)
        .def("get_solid_rest_offsets", &IParticleSystemView::getSolidRestOffset)
        .def("set_solid_rest_offsets", &IParticleSystemView::setSolidRestOffset)
        .def("get_fluid_rest_offsets", &IParticleSystemView::getFluidRestOffset)
        .def("set_fluid_rest_offsets", &IParticleSystemView::setFluidRestOffset)
        .def("get_particle_contact_offsets", &IParticleSystemView::getParticleContactOffset)
        .def("set_particle_contact_offsets", &IParticleSystemView::setParticleContactOffset)
        .def("get_wind", &IParticleSystemView::getWind)
        .def("set_wind", &IParticleSystemView::setWind)
        .def("check", &IParticleSystemView::check);

    // ParticleClothView
    py::class_<IParticleClothView, PyParticleClothView, std::unique_ptr<IParticleClothView, PyParticleClothView::Deleter>>(
        m, "ParticleClothView")
        .def(py::init<>())
        .def_property_readonly("count", &IParticleClothView::getCount)
        .def_property_readonly("max_particles_per_cloth", &IParticleClothView::getMaxParticlesPerCloth)
        .def_property_readonly("max_springs_per_cloth", &IParticleClothView::getMaxSpringsPerCloth)
        .def("get_positions", &IParticleClothView::getPositions)
        .def("set_positions", &IParticleClothView::setPositions)
        .def("get_velocities", &IParticleClothView::getVelocities)
        .def("set_velocities", &IParticleClothView::setVelocities)
        .def("get_masses", &IParticleClothView::getMasses)
        .def("set_masses", &IParticleClothView::setMasses)
        .def("get_spring_damping", &IParticleClothView::getSpringDamping)
        .def("set_spring_damping", &IParticleClothView::setSpringDamping)
        .def("get_spring_stiffness", &IParticleClothView::getSpringStiffness)
        .def("set_spring_stiffness", &IParticleClothView::setSpringStiffness)
        .def("check", &IParticleClothView::check);

    // ParticleMaterialView
    py::class_<IParticleMaterialView, PyParticleMaterialView, std::unique_ptr<IParticleMaterialView, PyParticleMaterialView::Deleter>>(
        m, "ParticleMaterialView")
        .def(py::init<>())
        .def_property_readonly("count", &IParticleMaterialView::getCount)
        .def("get_friction", &IParticleMaterialView::getFriction)
        .def("set_friction", &IParticleMaterialView::setFriction)
        .def("get_damping", &IParticleMaterialView::getDamping)
        .def("set_damping", &IParticleMaterialView::setDamping)
        .def("get_gravity_scale", &IParticleMaterialView::getGravityScale)
        .def("set_gravity_scale", &IParticleMaterialView::setGravityScale)
        .def("get_lift", &IParticleMaterialView::getLift)
        .def("set_lift", &IParticleMaterialView::setLift)
        .def("get_drag", &IParticleMaterialView::getDrag)
        .def("set_drag", &IParticleMaterialView::setDrag)
        .def("check", &IParticleMaterialView::check);

    // SimulationView
    py::class_<ISimulationView, PySimulationView, std::unique_ptr<ISimulationView, PySimulationView::Deleter>>(
        m, "SimulationView")
        .def(py::init<>())
        .def_property_readonly("device_ordinal", &ISimulationView::getDeviceOrdinal)
        //.def_property_readonly("cuda_context", &ISimulationView::getCudaContext)
        .def_property_readonly("cuda_context", [](const ISimulationView* sim) { return size_t(sim->getCudaContext()); })
        .def_property_readonly("is_valid", [](const ISimulationView* sim) { return sim->getValid(); })
        .def("invalidate", &ISimulationView::invalidate)
        .def("set_subspace_roots", &ISimulationView::setSubspaceRoots)
        .def("get_object_type", &ISimulationView::getObjectType)
        .def("create_articulation_view", py::overload_cast<const char*>(&ISimulationView::createArticulationView), py::return_value_policy::take_ownership)
        .def("create_articulation_view", py::overload_cast<const std::vector<std::string>&>(&ISimulationView::createArticulationView), py::return_value_policy::take_ownership)
        .def("create_rigid_body_view", py::overload_cast<const char*>(&ISimulationView::createRigidBodyView), py::return_value_policy::take_ownership)
        .def("create_rigid_body_view", py::overload_cast<const std::vector<std::string>&>(&ISimulationView::createRigidBodyView), py::return_value_policy::take_ownership)
        .def("create_soft_body_view", &ISimulationView::createSoftBodyView, py::return_value_policy::take_ownership)
        .def("create_soft_body_material_view", &ISimulationView::createSoftBodyMaterialView, py::return_value_policy::take_ownership)
        .def("create_volume_deformable_body_view", &ISimulationView::createVolumeDeformableBodyView, py::return_value_policy::take_ownership)
        .def("create_surface_deformable_body_view", &ISimulationView::createSurfaceDeformableBodyView, py::return_value_policy::take_ownership)
        .def("create_deformable_material_view", &ISimulationView::createDeformableMaterialView, py::return_value_policy::take_ownership)
        .def("create_rigid_contact_view",
             py::overload_cast<const std::string, const std::vector<std::string>&, uint32_t>(
                 &ISimulationView::createRigidContactView),
             py::arg("patterns"), py::arg("filter_patterns") = std::vector<std::string>(),
             py::arg("max_contact_data_count") = 0, py::return_value_policy::take_ownership)
        .def("create_rigid_contact_view",
             py::overload_cast<const std::vector<std::string>&, const std::vector<std::vector<std::string>>&, uint32_t>(
                 &ISimulationView::createRigidContactView),
             py::arg("pattern"), py::arg("filter_patterns") = std::vector<std::string>(),
             py::arg("max_contact_data_count") = 0, py::return_value_policy::take_ownership)
        .def("create_particle_system_view", &ISimulationView::createParticleSystemView,
             py::return_value_policy::take_ownership)
        .def("create_particle_cloth_view", &ISimulationView::createParticleClothView,
             py::return_value_policy::take_ownership)
        .def("create_particle_material_view", &ISimulationView::createParticleMaterialView,
             py::return_value_policy::take_ownership)
        .def(
            "create_sdf_shape_view",
            [](ISimulationView* sim, const std::string& pattern, uint32_t samplePointsCount) {
                return sim->createSdfShapeView(pattern.c_str(), samplePointsCount);
            },
            py::arg("pattern"), py::arg("sample_point_counts") = std::vector<uint32_t>(),
            py::return_value_policy::take_ownership)

        .def("clear_forces", &ISimulationView::clearForces)
        .def("set_gravity", &ISimulationView::setGravity)
        .def("get_gravity", &ISimulationView::getGravity)
        .def("flush", &ISimulationView::flush)
        .def("enable_warnings", &ISimulationView::enableGpuUsageWarnings)
        .def("update_articulations_kinematic", &ISimulationView::updateArticulationsKinematic)
        .def("initialize_kinematic_bodies", &ISimulationView::InitializeKinematicBodies)
        .def("check", &ISimulationView::check)
        .def("step", &ISimulationView::step);

    // TensorApi
    defineInterfaceClass<TensorApi>(m, "TensorApi", "acquire_tensor_api", "release_tensor_api")
        .def("create_simulation_view", wrapInterfaceFunction(&TensorApi::createSimulationView),
             py::arg("stage_id") = -1L, py::return_value_policy::take_ownership)
        .def("reset", wrapInterfaceFunction(&TensorApi::reset));

    // BackendRegistry
    defineInterfaceClass<BackendRegistry>(m, "BackendRegistry", "acquire_backend_registry", "release_backend_registry")
        .def("register_backend", wrapInterfaceFunction(&BackendRegistry::registerBackend))
        .def("unregister_backend", wrapInterfaceFunction(&BackendRegistry::unregisterBackend));
}
}
