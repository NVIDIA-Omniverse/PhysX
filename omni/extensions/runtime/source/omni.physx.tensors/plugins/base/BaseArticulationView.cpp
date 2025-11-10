// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "BaseArticulationView.h"
#include "BaseSimulationView.h"

#include "../GlobalsAreBad.h"
#include "../CommonTypes.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>

#include <omni/physics/tensors/TensorUtils.h>
#include <omni/physics/tensors/JointTypes.h>

using omni::physics::tensors::checkTensorDevice;
using omni::physics::tensors::checkTensorFloat32;
using omni::physics::tensors::checkTensorInt32;
using omni::physics::tensors::checkTensorSizeExact;
using omni::physics::tensors::checkTensorSizeMinimum;
using omni::physics::tensors::getTensorTotalSize;

using namespace physx;
using omni::physics::tensors::DofDriveType;

namespace omni
{
namespace physx
{
namespace tensors
{

BaseArticulationView::BaseArticulationView(BaseSimulationView* sim, const std::vector<ArticulationEntry>& entries)
    : mSim(sim), mEntries(entries)
{
    if (mSim)
    {
        // acquire a shared data pointer so the stuff we need doesn't get deleted
        mSimData = mSim->getBaseSimulationData();
    }

    PxU32 numArtis = PxU32(mEntries.size());
    // initialize default indices
    mAllIndices.resize(numArtis);

    for (PxU32 i = 0; i < numArtis; i++)
    {
        mSim->articulations.insert(mEntries[i].arti);
        for (auto ele : mEntries[i].links)
            mSim->links.insert(ele);
        for (auto ele : mEntries[i].shapes)
            mSim->shapes.insert(ele);
        for (auto ele : mEntries[i].fixedTendons)
            mSim->fixedTendons.insert(ele);
        for (auto ele : mEntries[i].spatialTendons)
            mSim->spatialTendons.insert(ele);

        // update max dims
        if (mEntries[i].numLinks > mMaxLinks)
        {
            mMaxLinks = mEntries[i].numLinks;
        }
        if (mEntries[i].numDofs > mMaxDofs)
        {
            mMaxDofs = mEntries[i].numDofs;
        }
        if (mEntries[i].numShapes > mMaxShapes)
        {
            mMaxShapes = mEntries[i].numShapes;
        }
        if (mEntries[i].numFixedTendons > mMaxFixedTendons)
        {
            mMaxFixedTendons = mEntries[i].numFixedTendons;
        }
        if (mEntries[i].numSpatialTendons > mMaxSpatialTendons)
        {
            mMaxSpatialTendons = mEntries[i].numSpatialTendons;
        }

        // track whether this is a homogeneous collection
        if (i == 0)
        {
            mIsHomogeneous = true;
            // printf("~!~! homo (%p)\n", mEntries[0].metatype);
            // mEntries[0].metatype->print();
        }
        else if (mEntries[i].metatype != mEntries[0].metatype)
        {
            mIsHomogeneous = false;
            // printf("~!~! not homo (%p)\n", mEntries[i].metatype);
            // mEntries[i].metatype->print();
        }

        mAllIndices[i] = i;
    }

    // printf("~!~! View is homogeneous: %d\n", int(mIsHomogeneous));
}

BaseArticulationView::~BaseArticulationView()
{
    if (mSim)
    {
        mSim->_onChildRelease(this);
    }
}

uint32_t BaseArticulationView::getCount() const
{
    return uint32_t(mEntries.size());
}

uint32_t BaseArticulationView::getMaxLinks() const
{
    return mMaxLinks;
}

uint32_t BaseArticulationView::getMaxDofs() const
{
    return mMaxDofs;
}

uint32_t BaseArticulationView::getMaxShapes() const
{
    return mMaxShapes;
}

uint32_t BaseArticulationView::getMaxFixedTendons() const
{
    return mMaxFixedTendons;
}

uint32_t BaseArticulationView::getMaxSpatialTendons() const
{
    return mMaxSpatialTendons;
}

bool BaseArticulationView::isHomogeneous() const
{
    return mIsHomogeneous && getCount() > 0;
}

const char* BaseArticulationView::getUsdPrimPath(uint32_t artiIdx) const
{
    if (artiIdx < mEntries.size())
    {
        return mEntries[artiIdx].path.GetString().c_str();
    }
    return nullptr;
}

const char* BaseArticulationView::getUsdDofPath(uint32_t artiIdx, uint32_t dofIdx) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, nullptr);
    if (artiIdx < mEntries.size())
    {
        if (dofIdx < mEntries[artiIdx].dofImpls.size())
        {
            ::physx::PxArticulationJointReducedCoordinate* joint = mEntries[artiIdx].dofImpls[dofIdx].joint;
            size_t jointId = reinterpret_cast<size_t>(joint->userData);
            pxr::SdfPath jointPath = g_physx->getPhysXObjectUsdPath(jointId);
            return jointPath.GetString().c_str();
        }
        return nullptr;
    }
    return nullptr;
}

const char* BaseArticulationView::getUsdLinkPath(uint32_t artiIdx, uint32_t linkIdx) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, nullptr);
    if (artiIdx < mEntries.size())
    {
        if (linkIdx < mEntries[artiIdx].links.size())
        {
            ::physx::PxArticulationLink* link = mEntries[artiIdx].links[linkIdx];
            size_t linkId = reinterpret_cast<size_t>(link->userData);
            pxr::SdfPath linkPath = g_physx->getPhysXObjectUsdPath(linkId);
            return linkPath.GetString().c_str();
        }
        return nullptr;
    }
    return nullptr;
}

const ArticulationMetatype* BaseArticulationView::getSharedMetatype() const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, nullptr);
    if (mIsHomogeneous && getCount() > 0)
    {
        return mEntries[0].metatype;
    }
    else
    {
        return nullptr;
    }
}

const ArticulationMetatype* BaseArticulationView::getMetatype(uint32_t artiIdx) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, nullptr);
    if (artiIdx < mEntries.size())
    {
        return mEntries[artiIdx].metatype;
    }
    return nullptr;
}

bool BaseArticulationView::getJacobianShape(uint32_t* numRows, uint32_t* numCols) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    if (!mIsHomogeneous)
    {
        CARB_LOG_ERROR("Attempted to get Jacobian size for non-homogeneous articulation view");
        return false;
    }

    bool isBaseFixed = mEntries[0].metatype->getFixedBase();
    *numCols = (isBaseFixed ? 0 : 6) + mEntries[0].numDofs;
    *numRows = (isBaseFixed ? 0 : 6) + (mEntries[0].numLinks - 1) * 6;

    return true;
}

// DEPRECATED
bool BaseArticulationView::getMassMatrixShape(uint32_t* numRows, uint32_t* numCols) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    if (!mIsHomogeneous)
    {
        CARB_LOG_ERROR("Attempted to get Mass Matrix size for non-homogeneous articulation view");
        return false;
    }

    *numCols = mEntries[0].numDofs;
    *numRows = mEntries[0].numDofs;

    return true;
}

bool BaseArticulationView::getGeneralizedMassMatrixShape(uint32_t* numRows, uint32_t* numCols) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    if (!mIsHomogeneous)
    {
        CARB_LOG_ERROR("Attempted to get Mass Matrix size for non-homogeneous articulation view");
        return false;
    }

    const bool isFixedBase = mEntries[0].metatype->getFixedBase();
    if (isFixedBase)
    {
        *numCols = mEntries[0].numDofs;
        *numRows = mEntries[0].numDofs;
    }
    else
    {
        *numCols = mEntries[0].numDofs + 6;
        *numRows = mEntries[0].numDofs + 6;
    }

    return true;
}

bool BaseArticulationView::getDofTypes(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "DOF type", __FUNCTION__) ||
        !checkTensorInt8(*dstTensor, "DOF type", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "DOF type", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        const ArticulationMetatype* metatype = mEntries[i].metatype;
        if (metatype)
        {
            uint8_t* dst = static_cast<uint8_t*>(dstTensor->data) + i * mMaxDofs;
            for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
            {
                *dst++ = static_cast<uint8_t>(metatype->getDofType(j));
            }
        }
    }

    return true;
}

bool BaseArticulationView::getDofMotions(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "DOF motion", __FUNCTION__) ||
        !checkTensorInt8(*dstTensor, "DOF motion", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "DOF motion", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        uint8_t* dst = static_cast<uint8_t*>(dstTensor->data) + i * mMaxDofs;
        for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
        {
            const DofImpl& dofImpl = mEntries[i].dofImpls[j];
            if (dofImpl.joint)
            {
                DofMotion m = fromPhysx(dofImpl.joint->getMotion(dofImpl.axis));
                *dst++ = static_cast<uint8_t>(m);
            }
            else
            {
                *dst++ = static_cast<uint8_t>(DofMotion::eInvalid);
            }
        }
    }

    return true;
}

bool BaseArticulationView::getDofLimits(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "DOF limit", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "DOF limit", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs * 2, "DOF limit", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs * 2;
        for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
        {
            PxArticulationLimit limit(0.f, 0.f);
            const DofImpl& dofImpl = mEntries[i].dofImpls[j];
            if (dofImpl.joint)
            {
                PxArticulationMotion::Enum m = dofImpl.joint->getMotion(dofImpl.axis);
                if (m == PxArticulationMotion::eLIMITED)
                {
                    limit = dofImpl.joint->getLimitParams(dofImpl.axis);
                }
                else if (m == PxArticulationMotion::eFREE)
                {
                    limit.low = -std::numeric_limits<float>::max();
                    limit.high = std::numeric_limits<float>::max();
                }
            }

            if (mEntries[i].metatype->isDofBody0Parent(j))
            {
                *dst++ = limit.low;
                *dst++ = limit.high;
            }
            else
            {
                *dst++ = -limit.high;
                *dst++ = -limit.low;
            }
        }
    }

    return true;
}


bool BaseArticulationView::getDriveTypes(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Drive type", __FUNCTION__) ||
        !checkTensorInt8(*dstTensor, "Drive type", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "Drive type", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        uint8_t* dst = static_cast<uint8_t*>(dstTensor->data) + i * mMaxDofs;
        for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
        {
            const DofImpl& dofImpl = mEntries[i].dofImpls[j];
            if (dofImpl.joint)
            {
                PxArticulationDrive driveProps = dofImpl.joint->getDriveParams(dofImpl.axis);
                // Note: for compatiblity with dynamic control extension we aren't including other types here 
                DofDriveType type;
                switch (driveProps.driveType)
                {
                case PxArticulationDriveType::eFORCE:
                    type = DofDriveType::eForce;
                    break;
                case PxArticulationDriveType::eACCELERATION:
                    type = DofDriveType::eAcceleration;
                    break;
                default:
                    type = DofDriveType::eNone;
                    break;
                }
                *dst++ = static_cast<uint8_t>(type);
            }
        }
    }

    return true;
}

bool BaseArticulationView::getDofStiffnesses(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "DOF stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "DOF stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "DOF stiffness", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs;
        for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
        {
            const DofImpl& dofImpl = mEntries[i].dofImpls[j];
            if (dofImpl.joint)
            {
                PxArticulationDrive driveProps = dofImpl.joint->getDriveParams(dofImpl.axis);
                *dst++ = driveProps.stiffness;
            }
            else
            {
                *dst++ = 0.0f;
            }
        }
    }

    return true;
}

bool BaseArticulationView::getDofDampings(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "DOF damping", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "DOF damping", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "DOF damping", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs;
        for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
        {
            const DofImpl& dofImpl = mEntries[i].dofImpls[j];
            if (dofImpl.joint)
            {
                PxArticulationDrive driveProps = dofImpl.joint->getDriveParams(dofImpl.axis);
                *dst++ = driveProps.damping;
            }
            else
            {
                *dst++ = 0.0f;
            }
        }
    }

    return true;
}

bool BaseArticulationView::getDofMaxForces(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "DOF max force", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "DOF max force", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "DOF max force", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs;
        for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
        {
            const DofImpl& dofImpl = mEntries[i].dofImpls[j];
            if (dofImpl.joint)
            {
                PxArticulationDrive driveProps = dofImpl.joint->getDriveParams(dofImpl.axis);
                if (driveProps.envelope.maxEffort != 0.0)
                {
                    *dst++ = driveProps.envelope.maxEffort;
                }
                else
                {
                    *dst++ = driveProps.maxForce;
                }
            }
            else
            {
                *dst++ = FLT_MAX;
            }
        }
    }

    return true;
}

bool BaseArticulationView::getDofDriveModelProperties(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "DOF drive model properties", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "DOF drive model properties", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs * 3u, "DOF drive model properties", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs * 3;
        for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
        {
            const DofImpl& dofImpl = mEntries[i].dofImpls[j];
            if (dofImpl.joint)
            {
                PxArticulationDrive driveProps = dofImpl.joint->getDriveParams(dofImpl.axis);
                *dst++ = driveProps.envelope.speedEffortGradient;
                *dst++ = driveProps.envelope.maxActuatorVelocity;
                *dst++ = driveProps.envelope.velocityDependentResistance;
            }
            else
            {
                *dst++ = FLT_MAX;
                *dst++ = FLT_MAX;
                *dst++ = FLT_MAX;
            }
        }
    }

    return true;
}

// DEPRECATED
bool BaseArticulationView::getDofFrictionCoefficients(const TensorDesc* dstTensor) const
{
    CARB_LOG_WARN("DEPRECATED: Please use getDofFrictionProperties instead.");
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "DOF friction coefficient", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "DOF friction coefficient", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "DOF friction coefficient", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs;
        for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
        {
            const DofImpl& dofImpl = mEntries[i].dofImpls[j];
            if (dofImpl.joint)
            {
                PxReal friction = dofImpl.joint->getFrictionCoefficient();
                *dst++ = friction;
            }
            else
            {
                *dst++ = 0.0f;
            }
        }
    }

    return true;
}

bool BaseArticulationView::getDofFrictionProperties(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "friction properties", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "friction properties", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs * 3u, "friction properties", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs * 3;
        for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
        {
            const DofImpl& dofImpl = mEntries[i].dofImpls[j];
            if (dofImpl.joint)
            {
                PxJointFrictionParams frictionParams = dofImpl.joint->getFrictionParams(dofImpl.axis);
                *dst++ = frictionParams.staticFrictionEffort;
                *dst++ = frictionParams.dynamicFrictionEffort;
                *dst++ = frictionParams.viscousFrictionCoefficient;
            }
            else
            {
                *dst++ = 0.0f;
                *dst++ = 0.0f;
                *dst++ = 0.0f;
            }
        }
    }

    return true;
}

bool BaseArticulationView::getDofMaxVelocities(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "DOF max velocity", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "DOF max velocity", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "DOF max velocity", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs;
        for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
        {
            const DofImpl& dofImpl = mEntries[i].dofImpls[j];
            if (dofImpl.joint)
            {
                PxReal velocity = dofImpl.joint->getMaxJointVelocity(dofImpl.axis);
                *dst++ = velocity;
            }
            else
            {
                *dst++ = FLT_MAX;
            }
        }
    }

    return true;
}

bool BaseArticulationView::getDofArmatures(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "DOF armature", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "DOF armature", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "DOF armature", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs;
        for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
        {
            const DofImpl& dofImpl = mEntries[i].dofImpls[j];
            if (dofImpl.joint)
            {
                PxReal armature = dofImpl.joint->getArmature(dofImpl.axis);
                *dst++ = armature;
            }
            else
            {
                *dst++ = 0.0f;
            }
        }
    }

    return true;
}

bool BaseArticulationView::setDofLimits(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "DOF limit", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "DOF limit", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs * 2, "DOF limit", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxDofs * 2;
            for (PxU32 j = 0; j < mEntries[idx].numDofs; j++)
            {
                const DofImpl& dofImpl = mEntries[idx].dofImpls[j];
                if (dofImpl.joint)
                {
                    PxArticulationLimit limit = dofImpl.joint->getLimitParams(dofImpl.axis);
                    if (mEntries[idx].metatype->isDofBody0Parent(j))
                    {
                        limit.low = src[j * 2];
                        limit.high = src[j * 2 + 1];
                    }
                    else
                    {
                        limit.low = -src[j * 2 + 1];
                        limit.high = -src[j * 2];
                    }
                    dofImpl.joint->setLimitParams(dofImpl.axis, limit);
                }
            }
        }
    }

    return true;
}

bool BaseArticulationView::setDofStiffnesses(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "DOF stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "DOF stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs, "DOF stiffness", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxDofs;
            for (PxU32 j = 0; j < mEntries[idx].numDofs; j++)
            {
                const DofImpl& dofImpl = mEntries[idx].dofImpls[j];
                if (dofImpl.joint)
                {
                    PxArticulationDrive params = dofImpl.joint->getDriveParams(dofImpl.axis);
                    params.stiffness = src[j];
                    dofImpl.joint->setDriveParams(dofImpl.axis, params);
                }
            }
        }
    }

    return true;
}

bool BaseArticulationView::setDofDampings(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "DOF damping", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "DOF damping", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs, "DOF damping", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxDofs;
            for (PxU32 j = 0; j < mEntries[idx].numDofs; j++)
            {
                const DofImpl& dofImpl = mEntries[idx].dofImpls[j];
                if (dofImpl.joint)
                {
                    PxArticulationDrive params = dofImpl.joint->getDriveParams(dofImpl.axis);
                    params.damping = src[j];
                    dofImpl.joint->setDriveParams(dofImpl.axis, params);
                }
            }
        }
    }

    return true;
}

bool BaseArticulationView::setDofMaxForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "DOF max force", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "DOF max force", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs, "DOF max force", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxDofs;
            for (PxU32 j = 0; j < mEntries[idx].numDofs; j++)
            {
                const DofImpl& dofImpl = mEntries[idx].dofImpls[j];
                if (dofImpl.joint)
                {
                    PxArticulationDrive params = dofImpl.joint->getDriveParams(dofImpl.axis);
                    if (dofImpl.driveType)
                        params.envelope.maxEffort = src[j];
                    else
                        params.maxForce = src[j];
                    dofImpl.joint->setDriveParams(dofImpl.axis, params);
                }
            }
        }
    }

    return true;
}

bool BaseArticulationView::setDofDriveModelProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor){
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "DOF drive model properties", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "DOF drive model properties", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs * 3u, "DOF drive model properties", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxDofs * 3;
            for (PxU32 j = 0; j < mEntries[idx].numDofs; j++)
            {
                const DofImpl& dofImpl = mEntries[idx].dofImpls[j];
                if (dofImpl.joint)
                {
                    PxArticulationDrive params = dofImpl.joint->getDriveParams(dofImpl.axis);
                    params.envelope.speedEffortGradient = src[j * 3 + 0];
                    params.envelope.maxActuatorVelocity = src[j * 3 + 1];
                    params.envelope.velocityDependentResistance = src[j * 3 + 2];
                    dofImpl.joint->setDriveParams(dofImpl.axis, params);
                }
            }
        }
    }

    return true;
}

// DEPRECATED
bool BaseArticulationView::setDofFrictionCoefficients(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CARB_LOG_WARN("DEPRECATED: Please use setDofFrictionProperties instead.");
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "DOF friction coefficient", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "DOF friction coefficient", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs, "DOF friction coefficient", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxDofs;
            for (PxU32 j = 0; j < mEntries[idx].numDofs; j++)
            {
                const DofImpl& dofImpl = mEntries[idx].dofImpls[j];
                if (dofImpl.joint)
                {
                    dofImpl.joint->setFrictionCoefficient(src[j]);
                }
            }
        }
    }

    return true;
}

bool BaseArticulationView::setDofFrictionProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "friction properties", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "friction properties", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs * 3u, "friction properties", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxDofs * 3;
            for (PxU32 j = 0; j < mEntries[idx].numDofs; j++)
            {
                const DofImpl& dofImpl = mEntries[idx].dofImpls[j];

                if (dofImpl.joint)
                {
                    PxJointFrictionParams frictionParams(src[3 * j], src[3 * j + 1], src[3 * j + 2]);
                    dofImpl.joint->setFrictionParams(dofImpl.axis, frictionParams);
                }
            }
        }
    }

    return true;
}

bool BaseArticulationView::setDofMaxVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "DOF max velocity", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "DOF max velocity", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs, "DOF max velocity", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxDofs;
            for (PxU32 j = 0; j < mEntries[idx].numDofs; j++)
            {
                const DofImpl& dofImpl = mEntries[idx].dofImpls[j];
                if (dofImpl.joint)
                {
                    dofImpl.joint->setMaxJointVelocity(dofImpl.axis, src[j]);
                }
            }
        }
    }

    return true;
}

bool BaseArticulationView::setDofArmatures(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "DOF armature", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "DOF armature", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs, "DOF armature", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxDofs;
            for (PxU32 j = 0; j < mEntries[idx].numDofs; j++)
            {
                const DofImpl& dofImpl = mEntries[idx].dofImpls[j];
                if (dofImpl.joint)
                {
                    dofImpl.joint->setArmature(dofImpl.axis, src[j]);
                }
            }
        }
    }

    return true;
}

bool BaseArticulationView::getMasses(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "masses", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "masses", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxLinks, "masses", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxLinks;
        for (PxU32 j = 0; j < mEntries[i].numLinks; j++)
        {
            PxReal mass = mEntries[i].links[j]->getMass();
            *dst++ = mass;
        }
    }

    return true;
}

bool BaseArticulationView::getInvMasses(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "inv masses", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "inv masses", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxLinks, "inv masses", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxLinks;
        for (PxU32 j = 0; j < mEntries[i].numLinks; j++)
        {
            PxReal invMass = mEntries[i].links[j]->getInvMass();
            *dst++ = invMass;
        }
    }

    return true;
}

bool BaseArticulationView::getCOMs(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "com", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "com", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxLinks * 7u, "com", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxLinks * 7;
        for (PxU32 j = 0; j < mEntries[i].numLinks; j++)
        {
            PxTransform comPose = mEntries[i].links[j]->getCMassLocalPose();
            *dst++ = comPose.p.x;
            *dst++ = comPose.p.y;
            *dst++ = comPose.p.z;
            *dst++ = comPose.q.x;
            *dst++ = comPose.q.y;
            *dst++ = comPose.q.z;
            *dst++ = comPose.q.w;
        }
    }

    return true;
}

bool BaseArticulationView::getInertias(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "inertia", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "inertia", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxLinks * 9u, "inertia", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxLinks * 9;
        for (PxU32 j = 0; j < mEntries[i].numLinks; j++)
        {
            PxTransform comPose = mEntries[i].links[j]->getCMassLocalPose();
            PxMat33 R{ comPose.q }; // the matrix that diagonalizes the inertia i.e. I = R * D * R'
            PxMat33 Rt = R.getTranspose();

            PxMat33 massSpaceDiagInertia = PxMat33::createDiagonal(mEntries[i].links[j]->getMassSpaceInertiaTensor());
            PxMat33 inertia = R * massSpaceDiagInertia * Rt;

            *dst++ = inertia.column0.x;
            *dst++ = inertia.column0.y;
            *dst++ = inertia.column0.z;
            *dst++ = inertia.column1.x;
            *dst++ = inertia.column1.y;
            *dst++ = inertia.column1.z;
            *dst++ = inertia.column2.x;
            *dst++ = inertia.column2.y;
            *dst++ = inertia.column2.z;
        }
    }

    return true;
}

bool BaseArticulationView::getInvInertias(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "inv inertia", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "inv inertia", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxLinks * 9u, "inv inertia", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxLinks * 9;
        for (PxU32 j = 0; j < mEntries[i].numLinks; j++)
        {
            PxTransform comPose = mEntries[i].links[j]->getCMassLocalPose();
            PxMat33 R{ comPose.q }; // the matrix that diagonalizes the inertia i.e. I = R * D * R'
            PxMat33 Rt = R.getTranspose();

            PxMat33 massSpaceDiagInvInertia = PxMat33::createDiagonal(mEntries[i].links[j]->getMassSpaceInvInertiaTensor());
            PxMat33 invInertia = R * massSpaceDiagInvInertia * Rt;

            *dst++ = invInertia.column0.x;
            *dst++ = invInertia.column0.y;
            *dst++ = invInertia.column0.z;
            *dst++ = invInertia.column1.x;
            *dst++ = invInertia.column1.y;
            *dst++ = invInertia.column1.z;
            *dst++ = invInertia.column2.x;
            *dst++ = invInertia.column2.y;
            *dst++ = invInertia.column2.z;
        }
    }

    return true;
}

bool BaseArticulationView::getDisableGravities(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "disable gravity", __FUNCTION__) ||
        !checkTensorInt8(*dstTensor, "disable gravity", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxLinks, "disable gravity", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        uint8_t* dst = static_cast<uint8_t*>(dstTensor->data) + i * mMaxLinks;
        for (PxU32 j = 0; j < mEntries[i].numLinks; j++)
        {
            dst[j] = (mEntries[i].links[j]->getActorFlags() & PxActorFlag::eDISABLE_GRAVITY) ? 1 : 0;
        }
    }

    return true;
}

bool BaseArticulationView::setMasses(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "mass", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "mass", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxLinks, "mass", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxLinks;
            for (PxU32 j = 0; j < mEntries[idx].numLinks; j++)
            {
                mEntries[idx].links[j]->setMass(src[j]);
            }
        }
    }

    return true;
}

bool BaseArticulationView::setCOMs(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "com", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "com", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxLinks * 7, "com", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxLinks * 7;
            for (PxU32 j = 0; j < mEntries[idx].numLinks; j++)
            {
                /*PxVec3 comPos{ src[j*7], src[j*7+1], src[j*7+2] };
                PxQuat comRot{ src[j*7+3], src[j*7+4], src[j*7+5], src[j*7+6] };*/
                PxVec3 comPos{ src[0], src[1], src[2] };
                PxQuat comRot{ src[3], src[4], src[5], src[6] };
                mEntries[idx].links[j]->setCMassLocalPose(PxTransform(comPos, comRot));
                src += 7;
            }
        }
    }

    return true;
}


bool BaseArticulationView::setInertias(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "inertia", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "inertia", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxLinks * 9u, "inertia", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxLinks * 9;
            for (PxU32 j = 0; j < mEntries[idx].numLinks; j++)
            {
                PxMat33 inertia;
                inertia.column0 = { src[j*9+0], src[j*9+1], src[j*9+2] };
                inertia.column1 = { src[j*9+3], src[j*9+4], src[j*9+5] };
                inertia.column2 = { src[j*9+6], src[j*9+7], src[j*9+8] };
                // diagnoalize the inertia tensor and update diagonal inertia and the inertial frame axes
                PxQuat axes;
                PxVec3 diagInertia = PxDiagonalize(inertia, axes);
                mEntries[idx].links[j]->setMassSpaceInertiaTensor(diagInertia);
                PxTransform comPose = mEntries[idx].links[j]->getCMassLocalPose();
                comPose.q = axes;
                mEntries[idx].links[j]->setCMassLocalPose(comPose);
            }
        }
    }

    return true;
}

bool BaseArticulationView::setDisableGravities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);   
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "disable gravity", __FUNCTION__) ||
        !checkTensorInt8(*srcTensor, "disable gravity", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxLinks, "disable gravity", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const uint8_t* src = static_cast<const uint8_t*>(srcTensor->data) + idx * mMaxLinks;
            for (PxU32 j = 0; j < mEntries[idx].numLinks; j++)
            {
                mEntries[idx].links[j]->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, src[j]);
            }
        }
    }

    return true;
}

bool BaseArticulationView::getMaterialProperties(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "material properties", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "material properties", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxShapes * 3u, "material properties", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxShapes * 3;
        for (PxU32 j = 0; j < mEntries[i].numShapes; j++)
        {
            const PxShape* shape = mEntries[i].shapes[j];
            PxMaterial* material;
            shape->getMaterials(&material, 1);
            *dst++ = material->getStaticFriction();
            *dst++ = material->getDynamicFriction();
            *dst++ = material->getRestitution();
        }
    }

    return true;
}

bool BaseArticulationView::getRestOffsets(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "rest offset", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "rest offset", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxShapes, "rest offset", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxShapes;
        for (PxU32 j = 0; j < mEntries[i].numShapes; j++)
        {
            *dst++ = mEntries[i].shapes[j]->getRestOffset();
        }
    }

    return true;
}

bool BaseArticulationView::getContactOffsets(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "contact offset", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "contact offset", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxShapes, "contact offset", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxShapes;
        for (PxU32 j = 0; j < mEntries[i].numShapes; j++)
        {
            *dst++ = mEntries[i].shapes[j]->getContactOffset();
        }
    }

    return true;
}

bool BaseArticulationView::setMaterialProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "material properties", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "material properties", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxShapes * 3u, "material properties", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxShapes * 3;
            for (PxU32 j = 0; j < mEntries[idx].numShapes; j++)
            {
                PxMaterial* material = mSim->createSharedMaterial(src[j*3], src[j*3+1], src[j*3+2]);
                
                int nMaterials = mEntries[idx].shapes[j]->getNbMaterials();
                std::vector<PxMaterial*> extraMats;
                extraMats.resize(nMaterials);

                mEntries[idx].shapes[j]->getMaterials(extraMats.data(), (PxU32)extraMats.size(), 0);

                for (auto mat : extraMats)
                {
                    mSim->mMaterialsRefCount[mat] -= 1;
                    if (mSim->mMaterialsRefCount[mat] == 0)
                    {
                        std::string key;
                        char keybuffer[100];
                        snprintf(keybuffer, 100, "%.6f", mat->getStaticFriction());
                        key += std::string(keybuffer) + "_";
                        snprintf(keybuffer, 100, "%.6f", mat->getDynamicFriction());
                        key += std::string(keybuffer) + "_";
                        snprintf(keybuffer, 100, "%.6f", mat->getRestitution());
                        key += std::string(keybuffer);
                        mSim->mMaterials.erase(key);
                        mSim->mUnusedMaterials.insert(mat);
                    }
                }

                mEntries[idx].shapes[j]->setMaterials(&material, 1);
            }
        }
    }

    return true;
}

bool BaseArticulationView::setRestOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "rest offset", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "rest offset", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxShapes, "rest offset", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxShapes;
            for (PxU32 j = 0; j < mEntries[idx].numShapes; j++)
            {
                mEntries[idx].shapes[j]->setRestOffset(src[j]);
            }
        }
    }

    return true;
}

bool BaseArticulationView::setContactOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "contact offset", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "contact offset", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxShapes, "contact offset", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxShapes;
            for (PxU32 j = 0; j < mEntries[idx].numShapes; j++)
            {
                mEntries[idx].shapes[j]->setContactOffset(src[j]);
            }
        }
    }

    return true;
}

bool BaseArticulationView::check() const
{
    bool result = true;

    // printf("~!~!~ Checking articulation view\n");

    if (!g_physx)
    {
        return false;
    }

    for (auto& entry : mEntries)
    {
        void* ptr = g_physx->getPhysXPtr(entry.path, omni::physx::PhysXType::ePTArticulation);
        if (ptr != entry.arti)
        {
            result = false;
        }
    }

    return result;
}

void BaseArticulationView::release()
{
    delete this;
}

void BaseArticulationView::_onParentRelease()
{
    // printf("~!~!~! Detaching arti view from parent %p\n", mSim);
    mSim = nullptr;
}

}
}
}
