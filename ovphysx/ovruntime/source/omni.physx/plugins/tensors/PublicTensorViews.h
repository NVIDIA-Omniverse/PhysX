// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-1 AC-3 AC-4
// @implements REQ-ATTACH-VIEWS-001
// @covers AC-1 AC-2 AC-3 AC-4
#pragma once
#include <stdexcept>
#include "OmniPhysX.h"
#include <type_traits>

#include "RuntimeExecution.h"
#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <omni/physics/tensors/IRigidBodyView.h>
#include <omni/physics/tensors/IArticulationView.h>
#include <omni/physics/tensors/IDeformableBodyView.h>
#include <omni/physics/tensors/IDeformableMaterialView.h>
#include <omni/physics/tensors/IRigidContactView.h>
#include <omni/physics/tensors/ISdfShapeView.h>
#include <omni/physics/tensors/ISimulationView.h>

namespace omni::physx::tensors
{
namespace publicViews
{
using namespace omni::physics::tensors;

// Only TensorApi exports these adapters. Backend caches and concrete views stay
// private and execute under their caller's admission, including masked setters
// and derived-to-base dispatch. No recursive gate or ambient owner is needed.
class Child;
struct State
{
    AttachHandle attach;
    ISimulationView* simulation;
    std::vector<Child*> children;
    std::shared_ptr<SimulationBackend> backend = OmniPhysX::getInstance().tensorBackend;
};

class Child
{
public:
    explicit Child(const std::shared_ptr<State>& state) : mState(state)
    {
        mState->children.push_back(this);
    }
    virtual ~Child()
    {
        std::vector<Child*>& children = mState->children;
        children.erase(std::remove(children.begin(), children.end(), this), children.end());
    }
protected:
    std::shared_ptr<State> mState;
};

template <typename Interface>
class BoundChild : public Interface, public Child
{
public:
    BoundChild(Interface* view, const std::shared_ptr<State>& state) : Child(state), mView(view) {}
    bool release() override
    {
        RuntimeExecutionScope execution(mState->attach);
        if (!execution)
            return false;
        mView->release();
        delete this;
        return true;
    }
protected:
    template <typename Result, typename Function>
    Result invoke(Function function) const
    {
        RuntimeExecutionScope execution(mState->attach);
        if (!execution || !mState->simulation || !mState->simulation->getValid())
        {
            if constexpr (std::is_void_v<Result> || std::is_same_v<Result, ObjectType>)
                throw std::runtime_error("Tensor operation requires an idle, valid attachment");
            else
                return Result();
        }
        return function();
    }
    Interface* mView;
};

// The interface compiler checks that every method has an admission wrapper.
// Keeping signatures here explicit makes newly added virtual methods fail the
// build until their public execution contract is classified.
#define PHYSX_TENSOR_FORWARD(Result, Name, Parameters, Arguments, Qualifiers) \
    Result Name Parameters Qualifiers override \
    { \
        return invoke<Result>([&]() -> Result { return mView->Name Arguments; }); \
    }

// These getters inspect storage retained by the view. Pending simulation never
// changes their meaning; invalidation/release obey the view lifetime contract.
#define PHYSX_TENSOR_METADATA(Result, Name, Parameters, Arguments, Qualifiers) \
    Result Name Parameters Qualifiers override { return mView->Name Arguments; }

class GuardedRigidBodyView final : public BoundChild<IRigidBodyView>
{
public:
    using BoundChild<IRigidBodyView>::BoundChild;
    PHYSX_TENSOR_METADATA(uint32_t, getCount, (), (), const)
    PHYSX_TENSOR_METADATA(uint32_t, getMaxShapes, (), (), const)
    PHYSX_TENSOR_METADATA(const char*, getUsdPrimPath, (uint32_t rbIdx), (rbIdx), const)
    PHYSX_TENSOR_FORWARD(bool, getTransforms, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getVelocities, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getAccelerations, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getKinematicTargets, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setKinematicTargets, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setTransforms, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setVelocities, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, applyForces, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, applyForcesAndTorquesAtPosition, (const TensorDesc* srcForceTensor, const TensorDesc* srcTorqueTensor, const TensorDesc* srcPositionTensor, const TensorDesc* indexTensor, const bool isGlobal), (srcForceTensor, srcTorqueTensor, srcPositionTensor, indexTensor, isGlobal), )
    PHYSX_TENSOR_FORWARD(bool, setKinematicTargetsMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setTransformsMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setVelocitiesMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, applyForcesMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, applyForcesAndTorquesAtPositionMasked, (const TensorDesc* srcForceTensor, const TensorDesc* srcTorqueTensor, const TensorDesc* srcPositionTensor, const TensorDesc* maskTensor, const bool isGlobal), (srcForceTensor, srcTorqueTensor, srcPositionTensor, maskTensor, isGlobal), )
    PHYSX_TENSOR_FORWARD(bool, getMasses, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getInvMasses, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getCOMs, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getInertias, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getInvInertias, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getDisableGravities, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getDisableSimulations, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setMasses, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setCOMs, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setInertias, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDisableGravities, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDisableSimulations, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setMassesMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setCOMsMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setInertiasMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDisableGravitiesMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDisableSimulationsMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, wakeUp, (const TensorDesc* indexTensor), (indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, putToSleep, (const TensorDesc* indexTensor), (indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, getMaterialProperties, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getCompliantMaterialProperties, (const TensorDesc* dstTensor, const TensorDesc* dstCombineModeTensor), (dstTensor, dstCombineModeTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getRestOffsets, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getContactOffsets, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setMaterialProperties, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setCompliantMaterialProperties, (const TensorDesc* srcTensor, const TensorDesc* srcCombineTensor, const TensorDesc* indexTensor), (srcTensor, srcCombineTensor, indexTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setRestOffsets, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setContactOffsets, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setMaterialPropertiesMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setCompliantMaterialPropertiesMasked, (const TensorDesc* srcTensor, const TensorDesc* srcCombineTensor, const TensorDesc* maskTensor), (srcTensor, srcCombineTensor, maskTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setRestOffsetsMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setContactOffsetsMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), const)
    PHYSX_TENSOR_FORWARD(bool, check, (), (), const)
};

class GuardedArticulationView final : public BoundChild<IArticulationView>
{
public:
    using BoundChild<IArticulationView>::BoundChild;
    PHYSX_TENSOR_METADATA(uint32_t, getCount, (), (), const)
    PHYSX_TENSOR_METADATA(uint32_t, getMaxLinks, (), (), const)
    PHYSX_TENSOR_METADATA(uint32_t, getMaxDofs, (), (), const)
    PHYSX_TENSOR_METADATA(uint32_t, getMaxShapes, (), (), const)
    PHYSX_TENSOR_METADATA(uint32_t, getMaxFixedTendons, (), (), const)
    PHYSX_TENSOR_METADATA(uint32_t, getMaxSpatialTendons, (), (), const)
    PHYSX_TENSOR_METADATA(bool, isHomogeneous, (), (), const)
    PHYSX_TENSOR_METADATA(const IArticulationMetatype*, getSharedMetatype, (), (), const)
    PHYSX_TENSOR_METADATA(const IArticulationMetatype*, getMetatype, (uint32_t artiIdx), (artiIdx), const)
    PHYSX_TENSOR_METADATA(const char*, getUsdPrimPath, (uint32_t artiIdx), (artiIdx), const)
    PHYSX_TENSOR_METADATA(const char*, getUsdDofPath, (uint32_t artiIdx, uint32_t dofIdx), (artiIdx, dofIdx), const)
    PHYSX_TENSOR_METADATA(const char*, getUsdLinkPath, (uint32_t artiIdx, uint32_t linkIdx), (artiIdx, linkIdx), const)
    PHYSX_TENSOR_FORWARD(bool, getDofTypes, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getDofMotions, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getDofLimits, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getDriveTypes, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getDofStiffnesses, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getDofDampings, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getDofMaxForces, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getDofDriveModelProperties, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getDofFrictionProperties, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getDofMaxVelocities, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getDofArmatures, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setDofLimits, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofStiffnesses, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofDampings, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofMaxForces, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofDriveModelProperties, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofFrictionProperties, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofMaxVelocities, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofArmatures, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofLimitsMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofStiffnessesMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofDampingsMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofMaxForcesMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofDriveModelPropertiesMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofFrictionPropertiesMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofMaxVelocitiesMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofArmaturesMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, getDofActuationForces, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getDofProjectedJointForces, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getLinkTransforms, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getLinkVelocities, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getLinkAccelerations, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getRootTransforms, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getRootVelocities, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setRootTransforms, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setRootVelocities, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setRootTransformsMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setRootVelocitiesMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, getDofPositions, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getDofVelocities, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setDofPositions, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofVelocities, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofActuationForces, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofPositionTargets, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofVelocityTargets, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofPositionsMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofVelocitiesMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofActuationForcesMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofPositionTargetsMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDofVelocityTargetsMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, getDofPositionTargets, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getDofVelocityTargets, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getJacobianShape, (uint32_t* numRows, uint32_t* numCols), (numRows, numCols), const)
    PHYSX_TENSOR_FORWARD(bool, getJacobians, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getGeneralizedMassMatrixShape, (uint32_t* numRows, uint32_t* numCols), (numRows, numCols), const)
    PHYSX_TENSOR_FORWARD(bool, getGeneralizedMassMatrices, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getCoriolisAndCentrifugalCompensationForces, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getGravityCompensationForces, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getLinkIncomingJointForce, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, applyForcesAndTorquesAtPosition, (const TensorDesc* srcForceTensor, const TensorDesc* srcTorqueTensor, const TensorDesc* srcPositionTensor, const TensorDesc* indexTensor, const bool isGlobal), (srcForceTensor, srcTorqueTensor, srcPositionTensor, indexTensor, isGlobal), )
    PHYSX_TENSOR_FORWARD(bool, applyForcesAndTorquesAtPositionMasked, (const TensorDesc* srcForceTensor, const TensorDesc* srcTorqueTensor, const TensorDesc* srcPositionTensor, const TensorDesc* maskTensor, const bool isGlobal), (srcForceTensor, srcTorqueTensor, srcPositionTensor, maskTensor, isGlobal), )
    PHYSX_TENSOR_FORWARD(bool, getMasses, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getInvMasses, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getCOMs, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getInertias, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getInvInertias, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getDisableGravities, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getArticulationMassCenter, (const TensorDesc* dstTensor, bool localFrame), (dstTensor, localFrame), const)
    PHYSX_TENSOR_FORWARD(bool, getArticulationCentroidalMomentum, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setMasses, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setCOMs, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setInertias, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDisableGravities, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setMassesMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setCOMsMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setInertiasMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDisableGravitiesMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, getMaterialProperties, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getCompliantMaterialProperties, (const TensorDesc* dstTensor, const TensorDesc* dstCombineModeTensor), (dstTensor, dstCombineModeTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getRestOffsets, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getContactOffsets, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setMaterialProperties, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setCompliantMaterialProperties, (const TensorDesc* srcTensor, const TensorDesc* srcCombineTensor, const TensorDesc* indexTensor), (srcTensor, srcCombineTensor, indexTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setRestOffsets, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setContactOffsets, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setMaterialPropertiesMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setCompliantMaterialPropertiesMasked, (const TensorDesc* srcTensor, const TensorDesc* srcCombineTensor, const TensorDesc* maskTensor), (srcTensor, srcCombineTensor, maskTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setRestOffsetsMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setContactOffsetsMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getFixedTendonStiffnesses, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getFixedTendonDampings, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getFixedTendonLimitStiffnesses, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getFixedTendonLimits, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getFixedTendonfixedSpringRestLengths, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getFixedTendonOffsets, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getSpatialTendonStiffnesses, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getSpatialTendonDampings, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getSpatialTendonLimitStiffnesses, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, getSpatialTendonOffsets, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setFixedTendonProperties, (const TensorDesc* stiffnesses, const TensorDesc* dampings, const TensorDesc* limitStiffnesses, const TensorDesc* limits, const TensorDesc* restLengths, const TensorDesc* offsets, const TensorDesc* indexTensor), (stiffnesses, dampings, limitStiffnesses, limits, restLengths, offsets, indexTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setSpatialTendonProperties, (const TensorDesc* stiffnesses, const TensorDesc* dampings, const TensorDesc* limitStiffnesses, const TensorDesc* offsets, const TensorDesc* indexTensor), (stiffnesses, dampings, limitStiffnesses, offsets, indexTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setFixedTendonPropertiesMasked, (const TensorDesc* stiffnesses, const TensorDesc* dampings, const TensorDesc* limitStiffnesses, const TensorDesc* limits, const TensorDesc* restLengths, const TensorDesc* offsets, const TensorDesc* maskTensor), (stiffnesses, dampings, limitStiffnesses, limits, restLengths, offsets, maskTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setSpatialTendonPropertiesMasked, (const TensorDesc* stiffnesses, const TensorDesc* dampings, const TensorDesc* limitStiffnesses, const TensorDesc* offsets, const TensorDesc* maskTensor), (stiffnesses, dampings, limitStiffnesses, offsets, maskTensor), const)
    PHYSX_TENSOR_FORWARD(bool, check, (), (), const)
};

class GuardedDeformableBodyView final : public BoundChild<IDeformableBodyView>
{
public:
    using BoundChild<IDeformableBodyView>::BoundChild;
    PHYSX_TENSOR_METADATA(uint32_t, getCount, (), (), const)
    PHYSX_TENSOR_METADATA(const char*, getUsdPrimPath, (uint32_t dbIdx), (dbIdx), const)
    PHYSX_TENSOR_METADATA(const char*, getUsdSimulationMeshPrimPath, (uint32_t dbIdx), (dbIdx), const)
    PHYSX_TENSOR_METADATA(const char*, getUsdCollisionMeshPrimPath, (uint32_t dbIdx), (dbIdx), const)
    PHYSX_TENSOR_FORWARD(bool, getTransforms, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_METADATA(uint32_t, getNumNodesPerElement, (), (), const)
    PHYSX_TENSOR_METADATA(uint32_t, getMaxSimulationElementsPerBody, (), (), const)
    PHYSX_TENSOR_FORWARD(bool, getSimulationElementIndices, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_METADATA(uint32_t, getMaxSimulationNodesPerBody, (), (), const)
    PHYSX_TENSOR_FORWARD(bool, getSimulationNodalPositions, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setSimulationNodalPositions, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setSimulationNodalPositionsMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, getSimulationNodalVelocities, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setSimulationNodalVelocities, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setSimulationNodalVelocitiesMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, getSimulationNodalKinematicTargets, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setSimulationNodalKinematicTargets, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setSimulationNodalKinematicTargetsMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, getRestElementIndices, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_METADATA(uint32_t, getMaxRestNodesPerBody, (), (), const)
    PHYSX_TENSOR_FORWARD(bool, getRestNodalPositions, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_METADATA(uint32_t, getMaxCollisionElementsPerBody, (), (), const)
    PHYSX_TENSOR_FORWARD(bool, getCollisionElementIndices, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_METADATA(uint32_t, getMaxCollisionNodesPerBody, (), (), const)
    PHYSX_TENSOR_FORWARD(bool, getCollisionNodalPositions, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, check, (), (), const)
};

class GuardedDeformableMaterialView final : public BoundChild<IDeformableMaterialView>
{
public:
    using BoundChild<IDeformableMaterialView>::BoundChild;
    PHYSX_TENSOR_METADATA(uint32_t, getCount, (), (), const)
    PHYSX_TENSOR_FORWARD(bool, getDynamicFriction, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setDynamicFriction, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setDynamicFrictionMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, getYoungsModulus, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setYoungsModulus, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setYoungsModulusMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, getPoissonsRatio, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setPoissonsRatio, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setPoissonsRatioMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, getElasticityDamping, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setElasticityDamping, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setElasticityDampingMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, getBendingStiffness, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setBendingStiffness, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setBendingStiffnessMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, getThickness, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setThickness, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setThicknessMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, getBendingDamping, (const TensorDesc* dstTensor), (dstTensor), const)
    PHYSX_TENSOR_FORWARD(bool, setBendingDamping, (const TensorDesc* srcTensor, const TensorDesc* indexTensor), (srcTensor, indexTensor), )
    PHYSX_TENSOR_FORWARD(bool, setBendingDampingMasked, (const TensorDesc* srcTensor, const TensorDesc* maskTensor), (srcTensor, maskTensor), )
    PHYSX_TENSOR_FORWARD(bool, check, (), (), const)
};

class GuardedRigidContactView final : public BoundChild<IRigidContactView>
{
public:
    using BoundChild<IRigidContactView>::BoundChild;
    PHYSX_TENSOR_METADATA(uint32_t, getSensorCount, (), (), const)
    PHYSX_TENSOR_METADATA(uint32_t, getFilterCount, (), (), const)
    PHYSX_TENSOR_METADATA(uint32_t, getMaxContactDataCount, (), (), const)
    PHYSX_TENSOR_FORWARD(bool, getNetContactForces, (const TensorDesc* dstTensor, float dt), (dstTensor, dt), const)
    PHYSX_TENSOR_FORWARD(bool, getNetNormalContactForces, (const TensorDesc* dstTensor, float dt), (dstTensor, dt), const)
    PHYSX_TENSOR_FORWARD(bool, getNetFrictionContactForces, (const TensorDesc* dstTensor, float dt), (dstTensor, dt), const)
    PHYSX_TENSOR_FORWARD(bool, getContactForceMatrix, (const TensorDesc* dstTensor, float dt), (dstTensor, dt), const)
    PHYSX_TENSOR_FORWARD(bool, getNormalContactForceMatrix, (const TensorDesc* dstTensor, float dt), (dstTensor, dt), const)
    PHYSX_TENSOR_FORWARD(bool, getFrictionContactForceMatrix, (const TensorDesc* dstTensor, float dt), (dstTensor, dt), const)
    PHYSX_TENSOR_FORWARD(bool, getContactData, (const TensorDesc* contactForceTensor, const TensorDesc* contactPointTensor, const TensorDesc* contactNormalTensor, const TensorDesc* contactSeparationTensor, const TensorDesc* contactCountTensor, const TensorDesc* contactStartIndicesTensor, uint32_t* outRequiredContactCount, float dt), (contactForceTensor, contactPointTensor, contactNormalTensor, contactSeparationTensor, contactCountTensor, contactStartIndicesTensor, outRequiredContactCount, dt), const)
    PHYSX_TENSOR_FORWARD(bool, getNormalContactData, (const TensorDesc* contactForceTensor, const TensorDesc* contactPointTensor, const TensorDesc* contactNormalTensor, const TensorDesc* contactSeparationTensor, const TensorDesc* contactCountTensor, const TensorDesc* contactStartIndicesTensor, uint32_t* outRequiredContactCount, float dt), (contactForceTensor, contactPointTensor, contactNormalTensor, contactSeparationTensor, contactCountTensor, contactStartIndicesTensor, outRequiredContactCount, dt), const)
    PHYSX_TENSOR_FORWARD(bool, getFrictionData, (const TensorDesc* FrictionForceTensor, const TensorDesc* contactPointTensor, const TensorDesc* contactCountTensor, const TensorDesc* contactStartIndicesTensor, uint32_t* outRequiredFrictionCount, float dt), (FrictionForceTensor, contactPointTensor, contactCountTensor, contactStartIndicesTensor, outRequiredFrictionCount, dt), const)
    PHYSX_TENSOR_FORWARD(bool, getFrictionContactData, (const TensorDesc* FrictionForceTensor, const TensorDesc* contactPointTensor, const TensorDesc* contactCountTensor, const TensorDesc* contactStartIndicesTensor, uint32_t* outRequiredFrictionCount, float dt), (FrictionForceTensor, contactPointTensor, contactCountTensor, contactStartIndicesTensor, outRequiredFrictionCount, dt), const)
    PHYSX_TENSOR_FORWARD(bool, getRawContactData, (const TensorDesc* contactForceTensor, const TensorDesc* contactPointTensor, const TensorDesc* contactNormalTensor, const TensorDesc* contactSeparationTensor, const TensorDesc* sensorLayoutTensor, const TensorDesc* actorIdsTensor, uint32_t* outRequiredContactCount, float dt), (contactForceTensor, contactPointTensor, contactNormalTensor, contactSeparationTensor, sensorLayoutTensor, actorIdsTensor, outRequiredContactCount, dt), const)
    PHYSX_TENSOR_FORWARD(void, getOtherActorPathsFromIds, (const TensorDesc* otherActorIdsTensor, std::vector<std::string>& outPaths), (otherActorIdsTensor, outPaths), const)
    PHYSX_TENSOR_FORWARD(bool, check, (), (), const)
    PHYSX_TENSOR_METADATA(const char*, getUsdPrimPath, (uint32_t sensorIdx), (sensorIdx), const)
    PHYSX_TENSOR_METADATA(const char*, getUsdPrimName, (uint32_t sensorIdx), (sensorIdx), const)
    PHYSX_TENSOR_METADATA(const char*, getFilterUsdPrimPath, (uint32_t sensorIdx, uint32_t filterIdx), (sensorIdx, filterIdx), const)
    PHYSX_TENSOR_METADATA(const char*, getFilterUsdPrimName, (uint32_t sensorIdx, uint32_t filterIdx), (sensorIdx, filterIdx), const)
};

class GuardedSdfShapeView final : public BoundChild<ISdfShapeView>
{
public:
    using BoundChild<ISdfShapeView>::BoundChild;
    PHYSX_TENSOR_METADATA(uint32_t, getCount, (), (), const)
    PHYSX_TENSOR_METADATA(uint32_t, getMaxNumPoints, (), (), const)
    PHYSX_TENSOR_FORWARD(bool, getSdfAndGradients, (const TensorDesc* dstTensor, const TensorDesc* srcPointTensor), (dstTensor, srcPointTensor), const)
    PHYSX_TENSOR_FORWARD(bool, check, (), (), const)
    PHYSX_TENSOR_METADATA(const char*, getUsdPrimPath, (uint32_t sensorIdx), (sensorIdx), const)
};

class GuardedSimulationView final : public ISimulationView
{
public:
    GuardedSimulationView(ISimulationView* view, AttachHandle attach)
        : mView(view), mState(std::make_shared<State>(State{attach, view, {}})) {}

    // Device/context accessors describe retained storage, including after owner
    // invalidation; callers need them to free their own scratch allocations.
    // This is an invalidation snapshot, safe to inspect during a pending step.
    // A transient busy interval must not look like permanent invalidation.
    bool getValid() const override { return mView->getValid(); }

    bool release(bool recursive) override
    {
        RuntimeExecutionScope execution(mState->attach);
        if (!execution)
            return false;
        mView->release(recursive);
        mState->simulation = nullptr;
        if (recursive)
        {
            // The concrete parent has released its children. Retire adapters
            // without releasing those underlying objects a second time.
            while (!mState->children.empty())
                delete mState->children.back();
        }
        delete this;
        return true;
    }
    int getDeviceOrdinal() const override { return mView->getDeviceOrdinal(); }
    int getParamDeviceOrdinal() const override { return mView->getParamDeviceOrdinal(); }
    void* getCudaContext() const override { return mView->getCudaContext(); }
    PHYSX_TENSOR_FORWARD(bool, isSelectionCurrent, (), (), const)
    PHYSX_TENSOR_FORWARD(void, invalidate, (), (), )
    PHYSX_TENSOR_FORWARD(bool, setSubspaceRoots, (const char* pattern), (pattern), )
    IArticulationView* createArticulationView(const char* pattern)  override
    {
        return invoke<IArticulationView*>([&]() -> IArticulationView* {
            IArticulationView* child = mView->createArticulationView(pattern);
            return wrapChild<GuardedArticulationView>(child);
        });
    }
    IArticulationView* createArticulationView(const std::vector<std::string>& patterns)  override
    {
        return invoke<IArticulationView*>([&]() -> IArticulationView* {
            IArticulationView* child = mView->createArticulationView(patterns);
            return wrapChild<GuardedArticulationView>(child);
        });
    }
    IRigidBodyView* createRigidBodyView(const std::vector<std::string>& patterns)  override
    {
        return invoke<IRigidBodyView*>([&]() -> IRigidBodyView* {
            IRigidBodyView* child = mView->createRigidBodyView(patterns);
            return wrapChild<GuardedRigidBodyView>(child);
        });
    }
    IRigidBodyView* createRigidBodyView(const char* pattern)  override
    {
        return invoke<IRigidBodyView*>([&]() -> IRigidBodyView* {
            IRigidBodyView* child = mView->createRigidBodyView(pattern);
            return wrapChild<GuardedRigidBodyView>(child);
        });
    }
    IRigidContactView* createRigidContactView(const char* pattern, const char** filterPatterns, uint32_t numFilterPatterns, uint32_t maxContactDataCount)  override
    {
        return invoke<IRigidContactView*>([&]() -> IRigidContactView* {
            IRigidContactView* child = mView->createRigidContactView(pattern, filterPatterns, numFilterPatterns, maxContactDataCount);
            return wrapChild<GuardedRigidContactView>(child);
        });
    }
    IRigidContactView* createRigidContactView(const std::string pattern, const std::vector<std::string>& filterPatterns, uint32_t maxContactDataCount)  override
    {
        return invoke<IRigidContactView*>([&]() -> IRigidContactView* {
            IRigidContactView* child = mView->createRigidContactView(pattern, filterPatterns, maxContactDataCount);
            return wrapChild<GuardedRigidContactView>(child);
        });
    }
    IRigidContactView* createRigidContactView(const std::vector<std::string>& patterns, const std::vector<std::vector<std::string>>& filterPatterns, uint32_t maxContactDataCount)  override
    {
        return invoke<IRigidContactView*>([&]() -> IRigidContactView* {
            IRigidContactView* child = mView->createRigidContactView(patterns, filterPatterns, maxContactDataCount);
            return wrapChild<GuardedRigidContactView>(child);
        });
    }
    ISdfShapeView* createSdfShapeView(const char* pattern, uint32_t numSamplePoints)  override
    {
        return invoke<ISdfShapeView*>([&]() -> ISdfShapeView* {
            ISdfShapeView* child = mView->createSdfShapeView(pattern, numSamplePoints);
            return wrapChild<GuardedSdfShapeView>(child);
        });
    }
    IDeformableBodyView* createVolumeDeformableBodyView(const char* pattern)  override
    {
        return invoke<IDeformableBodyView*>([&]() -> IDeformableBodyView* {
            IDeformableBodyView* child = mView->createVolumeDeformableBodyView(pattern);
            return wrapChild<GuardedDeformableBodyView>(child);
        });
    }
    IDeformableBodyView* createVolumeDeformableBodyView(const std::vector<std::string>& patterns)  override
    {
        return invoke<IDeformableBodyView*>([&]() -> IDeformableBodyView* {
            IDeformableBodyView* child = mView->createVolumeDeformableBodyView(patterns);
            return wrapChild<GuardedDeformableBodyView>(child);
        });
    }
    IDeformableBodyView* createSurfaceDeformableBodyView(const char* pattern)  override
    {
        return invoke<IDeformableBodyView*>([&]() -> IDeformableBodyView* {
            IDeformableBodyView* child = mView->createSurfaceDeformableBodyView(pattern);
            return wrapChild<GuardedDeformableBodyView>(child);
        });
    }
    IDeformableBodyView* createSurfaceDeformableBodyView(const std::vector<std::string>& patterns)  override
    {
        return invoke<IDeformableBodyView*>([&]() -> IDeformableBodyView* {
            IDeformableBodyView* child = mView->createSurfaceDeformableBodyView(patterns);
            return wrapChild<GuardedDeformableBodyView>(child);
        });
    }
    IDeformableMaterialView* createDeformableMaterialView(const char* pattern)  override
    {
        return invoke<IDeformableMaterialView*>([&]() -> IDeformableMaterialView* {
            IDeformableMaterialView* child = mView->createDeformableMaterialView(pattern);
            return wrapChild<GuardedDeformableMaterialView>(child);
        });
    }
    IDeformableMaterialView* createDeformableMaterialView(const std::vector<std::string>& patterns)  override
    {
        return invoke<IDeformableMaterialView*>([&]() -> IDeformableMaterialView* {
            IDeformableMaterialView* child = mView->createDeformableMaterialView(patterns);
            return wrapChild<GuardedDeformableMaterialView>(child);
        });
    }
    PHYSX_TENSOR_FORWARD(ObjectType, getObjectType, (const char* path), (path), )
    void clearForces() override
    {
        RuntimeExecutionScope execution(mState->attach);
        if (!execution)
            throw std::runtime_error("clearForces requires an idle attachment");
        // Both concrete implementations reject invalidated views before
            // walking cached actors, with the documented invalid-view diagnostic.
            mView->clearForces();
    }
    PHYSX_TENSOR_FORWARD(bool, flush, (), (), )
    PHYSX_TENSOR_FORWARD(bool, updateArticulationsKinematic, (), (), )
    PHYSX_TENSOR_FORWARD(void, InitializeKinematicBodies, (), (), )
    PHYSX_TENSOR_FORWARD(bool, check, (), (), const)
    PHYSX_TENSOR_FORWARD(void, enableGpuUsageWarnings, (bool enable), (enable), )
    PHYSX_TENSOR_FORWARD(void, step, (float dt), (dt), )
    PHYSX_TENSOR_FORWARD(bool, setGravity, (const carb::Float3& value), (value), )
    PHYSX_TENSOR_FORWARD(bool, getGravity, (carb::Float3& value), (value), )
    PHYSX_TENSOR_FORWARD(void, setNoMatchLoggingQuiet, (bool value), (value), )
    PHYSX_TENSOR_METADATA(bool, isNoMatchLoggingQuiet, (), (), const)
private:
    template<class Wrapper, class Interface>
    Interface* wrapChild(Interface* child)
    {
        if (!child)
            return nullptr;
        try { return new Wrapper(child, mState); }
        catch (...) { child->release(); throw; }
    }

    template <typename Result, typename Function>
    Result invoke(Function function) const
    {
        RuntimeExecutionScope execution(mState->attach);
        if (!execution || !mView->getValid())
        {
            if constexpr (std::is_void_v<Result> || std::is_same_v<Result, ObjectType>)
                throw std::runtime_error("Tensor operation requires an idle, valid attachment");
            else
                return Result();
        }
        return function();
    }
    ISimulationView* mView;
    std::shared_ptr<State> mState;
};
#undef PHYSX_TENSOR_FORWARD
#undef PHYSX_TENSOR_METADATA

inline ISimulationView* wrap(ISimulationView* implementation, AttachHandle attach)
{
    if (!implementation)
        return nullptr;
    try { return new GuardedSimulationView(implementation, attach); }
    catch (...) { implementation->release(true); throw; }
}
}
}
