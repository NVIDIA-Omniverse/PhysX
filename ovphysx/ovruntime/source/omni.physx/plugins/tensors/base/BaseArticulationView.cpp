// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-TENSOR-PATH-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-TENSOR-INDEX-001
 * @covers AC-3
 *
 * @implements REQ-TENSOR-ATTACH-001
 * @covers AC-1
 *
 * @implements REQ-TENSOR-CPU-ONLY-001
 * @covers AC-1 AC-2 AC-5
 *
 * @implements REQ-READ-ARTICULATION-001
 * @covers AC-5
 *
 * @implements REQ-READ-ATTRS-001
 * @covers AC-15, AC-16
 *
 * @implements REQ-INPUT-COVERAGE-001
 * @covers AC-10, AC-11
 */

// clang-format off
// clang-format on

#include "tensors/base/BaseArticulationView.h"
#include "tensors/base/BaseSimulationView.h"
#include "usdLoad/AttachedStage.h"
#include "tensors/base/OvStageShapeProperty.h"

#include "tensors/GlobalsAreBad.h"
#include "tensors/CommonTypes.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>
#include <omni/physics/tensors/TensorUtils.h>
#include <omni/physics/tensors/JointTypes.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>

using omni::physics::tensors::checkRecordIndices;
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
        }
        else if (mEntries[i].metatype != mEntries[0].metatype)
        {
            mIsHomogeneous = false;
        }

        // Tracked apart from mIsHomogeneous: differing DOF counts are fine, differing base
        // types are not.
        if (i > 0 && mEntries[i].metatype->getFixedBase() != mEntries[0].metatype->getFixedBase())
        {
            mIsBaseTypeUniform = false;
        }

        mAllIndices[i] = i;
    }
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

bool BaseArticulationView::requireUniformBaseType(const char* funcName) const
{
    if (!mIsBaseTypeUniform)
    {
        CARB_LOG_ERROR(
            "%s: the articulations in this view do not share a base type. Row layout is derived from it (fixed base uses the view's maximum DOF count, floating base that plus six), so a single layout cannot describe them. Build separate views for the fixed- and floating-base articulations.",
            funcName);
        return false;
    }
    return true;
}

const char* BaseArticulationView::getUsdPrimPath(uint32_t artiIdx) const
{
    if (artiIdx < mEntries.size())
    {
        return mEntries[artiIdx].path.c_str();
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
            return mEntries[artiIdx].dofImpls[dofIdx].path.c_str();
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
        if (linkIdx < mEntries[artiIdx].linkPaths.size())
        {
            return mEntries[artiIdx].linkPaths[linkIdx].c_str();
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

    // A homogeneous view's rows agree by definition, so row 0 answers for all of them.
    return jacobianShapeForRow(0, *numRows, *numCols);
}

PxU32 BaseArticulationView::generalizedCoordinateCountForRow(PxU32 row) const
{
    if (row >= mEntries.size() || !mEntries[row].metatype)
        return 0;
    return mEntries[row].numDofs + (mEntries[row].metatype->getFixedBase() ? 0u : 6u);
}

bool BaseArticulationView::jacobianShapeForRow(PxU32 row, PxU32& numRows, PxU32& numCols) const
{
    if (row >= mEntries.size() || !mEntries[row].metatype)
        return false;
    const bool isBaseFixed = mEntries[row].metatype->getFixedBase();
    numCols = (isBaseFixed ? 0u : 6u) + mEntries[row].numDofs;
    numRows = (isBaseFixed ? 0u : 6u) + (mEntries[row].numLinks - 1u) * 6u;
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
        // Zero the full [0, mMaxDofs) row so padded columns beyond numDofs read
        // as 0 (contract documented on the ovphysx drive-type tensor).
        for (PxU32 j = 0; j < mMaxDofs; j++)
            dst[j] = 0;
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

const DofImpl* BaseArticulationView::resolveDofImpl(const ArticulationDofOvStageRecord& record) const
{
    if (record.viewArtiIdx >= mEntries.size())
        return nullptr;
    const ArticulationEntry& entry = mEntries[record.viewArtiIdx];
    if (record.physxDofIdx >= entry.numDofs || record.physxDofIdx >= entry.dofImpls.size())
        return nullptr;
    return &entry.dofImpls[record.physxDofIdx];
}

namespace
{
// An "unlimited" bound is a sentinel, not a measurement: PhysX carries FLT_MAX for a free axis and an
// unset max velocity or force, and the parse library leaves it alone (ParseJoint's `< FLT_MAX` guard).
// Scaling it would turn the sentinel into +inf and report a limit nobody authored.
inline float scaleDofValue(float raw, float scale)
{
    if (!std::isfinite(raw) || raw >= FLT_MAX || raw <= -FLT_MAX)
        return raw;
    return scale * raw;
}
} // namespace

// Every requested property in one pass, under the same per-property rules as the single-column gather
// below: which fold applies, the FLT_MAX sentinel, the limit's swap-and-negate, and the
// unresolved-record fallback.
bool BaseArticulationView::getDofPropertiesOvStage(const DofPropertyColumn* columns,
                                                   PxU32 numColumns,
                                                   const ArticulationDofOvStageRecord* records,
                                                   PxU32 numOutputs) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    if (numColumns == 0 || numOutputs == 0)
        return true; // nothing to emit -- not a failure
    if (!columns || !records)
    {
        CARB_LOG_ERROR("%s: null columns or records", __FUNCTION__);
        return false;
    }

    // Validated per column with the column's own name, so a refusal says which property it was.
    for (PxU32 c = 0; c < numColumns; ++c)
    {
        const TensorDesc* dst = columns[c].dst;
        const char* label = dofPropertyLabel(columns[c].prop);
        if (!dst || !dst->data)
        {
            CARB_LOG_ERROR("%s: %s null destination", __FUNCTION__, label);
            return false;
        }
        const PxU32 comp = dofPropertyComponents(columns[c].prop);
        if (!checkTensorDevice(*dst, -1, label, __FUNCTION__) ||
            !checkTensorSizeExact(*dst, numOutputs * comp, label, __FUNCTION__))
            return false;
        if (dofPropertyIsByte(columns[c].prop))
        {
            if (!checkTensorInt8(*dst, label, __FUNCTION__))
                return false;
        }
        else if (!checkTensorFloat32(*dst, label, __FUNCTION__))
        {
            return false;
        }
    }

    // Which PhysX structs this SET of columns needs, decided once rather than per record: seven
    // columns come out of getDriveParams and three out of getFrictionParams, so all thirteen cost at
    // most four fetches per DOF. dofPropertySource must name every struct the value switch below
    // reads -- a missing source silently computes from a struct nobody fetched, and the friction
    // triple's default ctor leaves its fields uninitialised.
    uint32_t sources = 0;
    for (PxU32 c = 0; c < numColumns; ++c)
        sources |= dofPropertySource(columns[c].prop);

    for (PxU32 i = 0; i < numOutputs; i++)
    {
        const ArticulationDofOvStageRecord& r = records[i];
        const DofImpl* dofImpl = resolveDofImpl(r);
        if (!dofImpl || !dofImpl->joint)
        {
            // Same per-property fallback as the single-column gather: a magnitude bound reads as
            // "unlimited", everything else as zero, so an unresolved record is an absent property.
            for (PxU32 c = 0; c < numColumns; ++c)
            {
                const DofProperty prop = columns[c].prop;
                const PxU32 comp = dofPropertyComponents(prop);
                if (dofPropertyIsByte(prop))
                {
                    static_cast<uint8_t*>(columns[c].dst->data)[i] = static_cast<uint8_t>(DofDriveType::eNone);
                    continue;
                }
                const bool unlimited = (prop == DofProperty::eMaxVelocity || prop == DofProperty::eMaxForce ||
                                        prop == DofProperty::eSpeedEffortGradient ||
                                        prop == DofProperty::eMaxActuatorVelocity ||
                                        prop == DofProperty::eVelocityDependentResistance);
                float* dst = static_cast<float*>(columns[c].dst->data);
                for (PxU32 k = 0; k < comp; ++k)
                    dst[i * comp + k] = unlimited ? FLT_MAX : 0.0f;
            }
            continue;
        }

        PxArticulationJointReducedCoordinate* joint = dofImpl->joint;
        const PxArticulationAxis::Enum axis = dofImpl->axis;

        // At most four fetches, once for this DOF and shared by every column that wants them.
        PxArticulationDrive drive;
        PxJointFrictionParams friction;
        PxArticulationLimit limit(0.0f, 0.0f);
        float maxVel = 0.0f, armature = 0.0f;
        if (sources & eDofSrcDrive)
            drive = joint->getDriveParams(axis);
        if (sources & eDofSrcFriction)
            friction = joint->getFrictionParams(axis);
        if (sources & eDofSrcLimit)
        {
            const PxArticulationMotion::Enum motion = joint->getMotion(axis);
            if (motion == PxArticulationMotion::eLIMITED)
                limit = joint->getLimitParams(axis);
            else if (motion == PxArticulationMotion::eFREE)
            {
                limit.low = -std::numeric_limits<float>::max();
                limit.high = std::numeric_limits<float>::max();
            }
        }
        if (sources & eDofSrcMaxVelocity)
            maxVel = joint->getMaxJointVelocity(axis);
        if (sources & eDofSrcArmature)
            armature = joint->getArmature(axis);

        const float fwd = dofScaleFor(r, DofScalePolicy::eAngularForward);
        const float inv = dofScaleFor(r, DofScalePolicy::eAngularInverse);

        for (PxU32 c = 0; c < numColumns; ++c)
        {
            const DofProperty prop = columns[c].prop;

            // Byte columns first, so the float pointer below is always valid rather than a null one
            // live across twelve arms. dofPropertyIsByte is what the dtype validation above checked.
            if (dofPropertyIsByte(prop))
            {
                // An enum, so no fold. Same mapping as the dense getDriveTypes: the two the
                // dynamic-control extension knows, else none.
                DofDriveType type = DofDriveType::eNone;
                switch (drive.driveType)
                {
                case PxArticulationDriveType::eFORCE:        type = DofDriveType::eForce; break;
                case PxArticulationDriveType::eACCELERATION: type = DofDriveType::eAcceleration; break;
                default:                                     type = DofDriveType::eNone; break;
                }
                static_cast<uint8_t*>(columns[c].dst->data)[i] = static_cast<uint8_t>(type);
                continue;
            }

            float* dstF = static_cast<float*>(columns[c].dst->data);
            switch (prop)
            {
            case DofProperty::eStiffness: dstF[i] = scaleDofValue(drive.stiffness, inv); break;
            case DofProperty::eDamping:   dstF[i] = scaleDofValue(drive.damping, inv); break;
            case DofProperty::eLimit:
            {
                // A bound expressed against the other body is the negated OPPOSITE bound, not the
                // same number with a sign -- the swap-and-negate getDofLimits does.
                const bool body0Parent = r.sign > 0.0f;
                dstF[i * 2 + 0] = scaleDofValue(body0Parent ? limit.low : -limit.high, fwd);
                dstF[i * 2 + 1] = scaleDofValue(body0Parent ? limit.high : -limit.low, fwd);
                break;
            }
            case DofProperty::eMaxVelocity: dstF[i] = scaleDofValue(maxVel, fwd); break;
            case DofProperty::eMaxForce:
                // Prefer the envelope's max effort when set, matching the dense getDofMaxForces.
                dstF[i] = (drive.envelope.maxEffort != 0.0f) ? drive.envelope.maxEffort : drive.maxForce;
                break;
            case DofProperty::eArmature:        dstF[i] = armature; break;
            case DofProperty::eStaticFriction:  dstF[i] = friction.staticFrictionEffort; break;
            case DofProperty::eDynamicFriction: dstF[i] = friction.dynamicFrictionEffort; break;
            case DofProperty::eViscousFriction:
                dstF[i] = scaleDofValue(friction.viscousFrictionCoefficient, inv);
                break;
            case DofProperty::eSpeedEffortGradient:
                dstF[i] = scaleDofValue(drive.envelope.speedEffortGradient, fwd);
                break;
            case DofProperty::eMaxActuatorVelocity:
                dstF[i] = scaleDofValue(drive.envelope.maxActuatorVelocity, fwd);
                break;
            case DofProperty::eVelocityDependentResistance:
                dstF[i] = scaleDofValue(drive.envelope.velocityDependentResistance, inv);
                break;
            case DofProperty::eDriveType: break; // handled above, before dstF exists
            }
        }
    }
    return true;
}

bool BaseArticulationView::setDofPropertyOvStage(const DofProperty prop,
                                                 const ArticulationDofOvStageRecord* const records,
                                                 const PxU32 numOutputs,
                                                 const TensorDesc* const srcTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    if (!srcTensor || !srcTensor->data || !records)
        return false;
    if (numOutputs == 0)
        return true;
    const PxU32 comp = dofPropertyComponents(prop);
    if (!omni::physics::tensors::checkTensorDevice(*srcTensor, -1, "dof property (ovstage write)", __FUNCTION__) ||
        !omni::physics::tensors::checkTensorSizeExact(*srcTensor, numOutputs * comp, "dof property (ovstage write)",
                                                      __FUNCTION__))
    {
        return false;
    }

    // eDriveType is the one uint8 column; every other property is float32. Only the float case is
    // checked here -- the tensor helpers have no uint8 predicate, and the byte path below indexes
    // with the element size the caller was handed.
    if (!dofPropertyIsByte(prop) &&
        !omni::physics::tensors::checkTensorFloat32(*srcTensor, "dof property (ovstage write)", __FUNCTION__))
    {
        return false;
    }

    const float* const srcF = dofPropertyIsByte(prop) ? nullptr : static_cast<const float*>(srcTensor->data);
    const uint8_t* const srcB = dofPropertyIsByte(prop) ? static_cast<const uint8_t*>(srcTensor->data) : nullptr;

    // An axis only HAS a limit interval when its motion is eLIMITED, and PhysX refuses setMotion()
    // on an articulation that is in a scene -- so a finite limit aimed at a free axis cannot be
    // honoured at any price. Refuse it BEFORE writing anything: the alternative is a call that
    // reports success while some of its DOFs kept their old interval, which is the wrong result a
    // caller cannot tell from the right one. Writing the getter's +-FLT_MAX sentinel back to a free
    // axis is NOT a finite limit and stays a legal no-op, which is what keeps read-modify-write over
    // a whole articulation working when some of its axes are unlimited.
    if (prop == DofProperty::eLimit)
    {
        for (PxU32 i = 0; i < numOutputs; i++)
        {
            const DofImpl* dofImpl = resolveDofImpl(records[i]);
            if (!dofImpl || !dofImpl->joint)
                continue;
            if (dofImpl->joint->getMotion(dofImpl->axis) == PxArticulationMotion::eLIMITED)
                continue;
            const float low = srcF[i * 2 + 0];
            const float high = srcF[i * 2 + 1];
            if (PxIsFinite(low) && PxIsFinite(high) && low > -FLT_MAX && high < FLT_MAX)
            {
                CARB_LOG_ERROR("jointLimit: DOF %u belongs to a joint axis whose motion is not limited, and PhysX "
                               "does not allow an axis to become limited while its articulation is in a scene. "
                               "Author the limit on the joint prim instead. No limit was written.",
                               i);
                return false;
            }
        }
    }

    for (PxU32 i = 0; i < numOutputs; i++)
    {
        const ArticulationDofOvStageRecord& r = records[i];
        const DofImpl* dofImpl = resolveDofImpl(r);
        if (!dofImpl || !dofImpl->joint)
            continue; // a record that does not resolve writes nothing, as the getter reads nothing
        PxArticulationJointReducedCoordinate* joint = dofImpl->joint;
        const PxArticulationAxis::Enum axis = dofImpl->axis;

        // The inverse of exactly the factor the getter applied. dofInverseScaleFor, NOT a division:
        // the record carries angScale and invAngScale separately because their float values are not
        // reciprocals, so dividing would not return an authored value to itself.
        switch (prop)
        {
        case DofProperty::eStiffness:
        {
            PxArticulationDrive d = joint->getDriveParams(axis); // RMW: shares a struct
            d.stiffness = srcF[i] * dofInverseScaleFor(r, DofScalePolicy::eAngularInverse);
            joint->setDriveParams(axis, d);
            break;
        }
        case DofProperty::eDamping:
        {
            PxArticulationDrive d = joint->getDriveParams(axis);
            d.damping = srcF[i] * dofInverseScaleFor(r, DofScalePolicy::eAngularInverse);
            joint->setDriveParams(axis, d);
            break;
        }
        case DofProperty::eLimit:
        {
            // The pair is ONE attribute, so both lanes arrive together and no read-back is needed --
            // unlike its struct-sharing neighbours.
            //
            // The inverse of the getter is NOT a signed scale. The getter unscales with
            // eAngularForward (MAGNITUDE only) and then applies a SWAP-AND-NEGATE when body0 is not
            // the parent, because a bound expressed against the other body is the negated OPPOSITE
            // bound, not the same number with a sign. Undo those two steps in the opposite order.
            const float k = dofInverseScaleFor(r, DofScalePolicy::eAngularForward);
            const float lowU = srcF[i * 2 + 0] * k;
            const float highU = srcF[i * 2 + 1] * k;
            const bool body0Parent = r.sign > 0.0f;
            PxArticulationLimit limit(0.0f, 0.0f);
            limit.low = body0Parent ? lowU : -highU;
            limit.high = body0Parent ? highU : -lowU;

            // A free axis reaching here carries the sentinel and nothing to store: the pre-pass
            // above already refused every finite limit aimed at one.
            if (joint->getMotion(axis) != PxArticulationMotion::eLIMITED)
                break;
            joint->setLimitParams(axis, limit);
            break;
        }
        case DofProperty::eMaxVelocity:
            joint->setMaxJointVelocity(axis, scaleDofValue(srcF[i], dofInverseScaleFor(r, DofScalePolicy::eAngularForward)));
            break;
        case DofProperty::eMaxForce:
        {
            PxArticulationDrive d = joint->getDriveParams(axis);
            d.maxForce = scaleDofValue(srcF[i], dofInverseScaleFor(r, DofScalePolicy::eNone));
            joint->setDriveParams(axis, d);
            break;
        }
        case DofProperty::eArmature:
            joint->setArmature(axis, srcF[i]);
            break;
        case DofProperty::eStaticFriction:
        {
            PxJointFrictionParams fp = joint->getFrictionParams(axis); // RMW: shares a struct
            fp.staticFrictionEffort = srcF[i];
            joint->setFrictionParams(axis, fp);
            break;
        }
        case DofProperty::eDynamicFriction:
        {
            PxJointFrictionParams fp = joint->getFrictionParams(axis);
            fp.dynamicFrictionEffort = srcF[i];
            joint->setFrictionParams(axis, fp);
            break;
        }
        case DofProperty::eViscousFriction:
        {
            PxJointFrictionParams fp = joint->getFrictionParams(axis);
            fp.viscousFrictionCoefficient = srcF[i] * dofInverseScaleFor(r, DofScalePolicy::eAngularInverse);
            joint->setFrictionParams(axis, fp);
            break;
        }
        case DofProperty::eSpeedEffortGradient:
        {
            PxArticulationDrive d = joint->getDriveParams(axis);
            d.envelope.speedEffortGradient =
                scaleDofValue(srcF[i], dofInverseScaleFor(r, DofScalePolicy::eAngularForward));
            joint->setDriveParams(axis, d);
            break;
        }
        case DofProperty::eMaxActuatorVelocity:
        {
            PxArticulationDrive d = joint->getDriveParams(axis);
            d.envelope.maxActuatorVelocity =
                scaleDofValue(srcF[i], dofInverseScaleFor(r, DofScalePolicy::eAngularForward));
            joint->setDriveParams(axis, d);
            break;
        }
        case DofProperty::eVelocityDependentResistance:
        {
            PxArticulationDrive d = joint->getDriveParams(axis);
            d.envelope.velocityDependentResistance =
                scaleDofValue(srcF[i], dofInverseScaleFor(r, DofScalePolicy::eAngularInverse));
            joint->setDriveParams(axis, d);
            break;
        }
        case DofProperty::eDriveType:
        {
            PxArticulationDrive d = joint->getDriveParams(axis);
            // The enum the read publishes: 0 none, 1 force, 2 acceleration. Anything else is left
            // alone rather than cast into whatever it happens to land on.
            switch (srcB[i])
            {
            case 0:
                d.driveType = PxArticulationDriveType::eNONE;
                break;
            case 1:
                d.driveType = PxArticulationDriveType::eFORCE;
                break;
            case 2:
                d.driveType = PxArticulationDriveType::eACCELERATION;
                break;
            default:
                continue;
            }
            joint->setDriveParams(axis, d);
            break;
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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
                    if(dofImpl.joint->getMotion(dofImpl.axis) == PxArticulationMotion::eLIMITED)
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
                    else
                    {
                        OMNI_LOG_WARN(
                            "setDofLimits - Cannot update articulation joint limits because the joint was not initially configured with limits.");
                    }
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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
                    if (dofImpl.isEnvelopeUsed)
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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
                    // isEnvelopeUsed is set at USD load time based on DrivePerformanceEnvelopeAPI.
                    // It guards which drive model parameters can be applied (envelope vs legacy).
                    if (dofImpl.isEnvelopeUsed)
                    {
                        PxArticulationDrive params = dofImpl.joint->getDriveParams(dofImpl.axis);
                        params.envelope.speedEffortGradient = src[j * 3 + 0];
                        params.envelope.maxActuatorVelocity = src[j * 3 + 1];
                        params.envelope.velocityDependentResistance = src[j * 3 + 2];
                        dofImpl.joint->setDriveParams(dofImpl.axis, params);
                    }
                    else if (src[j * 3 + 0] != 0.0f || src[j * 3 + 1] != 0.0f || src[j * 3 + 2] != 0.0f)
                    {
                        CARB_LOG_WARN_ONCE("setDofDriveModelProperties - Some DOFs do not have PhysxDrivePerformanceEnvelopeAPI applied; "
                            "non-zero envelope parameters were ignored. The API must be applied in USD before simulation starts.");
                    }
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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
        // Zero the full [0, mMaxLinks) row so padded columns beyond numLinks
        // read as 0 (contract documented on the ovphysx articulation tensor).
        for (PxU32 j = 0; j < mMaxLinks; j++)
            dst[j] = 0;
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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
                PxVec3 comPos{ src[0], src[1], src[2] };
                PxQuat comRot{ src[3], src[4], src[5], src[6] };
                mEntries[idx].links[j]->setCMassLocalPose(PxTransform(comPos, comRot));
                src += 7;
            }
        }
    }

    // OMPE-103213: mirror BaseRigidBodyView::setCOMs -- GpuArticulationView::
    // applyForcesAndTorquesAtPosition skips updateCMassData while validComsCache
    // is true, so a setCOMs that leaves the flag set computes lever arms against
    // the pre-setCOMs COMs.
    setComsCacheStateValid(false);
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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

bool BaseArticulationView::getCompliantMaterialProperties(const TensorDesc* dstTensor,
                                                         const TensorDesc* dstCombineModeTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "material properties", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "material properties", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxShapes * 2u, "material properties", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*dstCombineModeTensor, -1, "combination properties", __FUNCTION__) ||
        !checkTensorInt8(*dstCombineModeTensor, "combination properties", __FUNCTION__) ||
        !checkTensorSizeExact(*dstCombineModeTensor, getCount() * mMaxShapes * 2u, "combination properties", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxShapes * 2;
        uint8_t* dstCombinedMode = static_cast<uint8_t*>(dstCombineModeTensor->data) + i * mMaxShapes * 2;
        for (PxU32 j = 0; j < mEntries[i].numShapes; j++)
        {
            const PxShape* shape = mEntries[i].shapes[j];
            PxMaterial* material;
            shape->getMaterials(&material, 1);
            float restitution = material->getRestitution();
            *dst++ = restitution < 0.0f ? -restitution : std::numeric_limits<float>::infinity();
            *dst++ = material->getDamping();
            *dstCombinedMode++ = static_cast<uint8_t>(material->getRestitutionCombineMode());
            *dstCombinedMode++ = static_cast<uint8_t>(material->getDampingCombineMode());
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

bool BaseArticulationView::getShapePropertyOvStage(OvStageShapeProperty property,
                                                   const TensorDesc* dstTensor,
                                                   const PxU32* rows,
                                                   PxU32 count) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    const char* label = "articulation shape property";
    switch (property)
    {
    case OvStageShapeProperty::eStaticFriction:
        label = "articulation static friction";
        break;
    case OvStageShapeProperty::eDynamicFriction:
        label = "articulation dynamic friction";
        break;
    case OvStageShapeProperty::eRestitution:
        label = "articulation restitution";
        break;
    case OvStageShapeProperty::eContactOffset:
        label = "articulation contact offset";
        break;
    case OvStageShapeProperty::eRestOffset:
        label = "articulation rest offset";
        break;
    }

    const PxU32 width = dstTensor->numDims >= 2 ? static_cast<PxU32>(dstTensor->dims[1]) : 0;
    if (width == 0 || !checkTensorDevice(*dstTensor, -1, label, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, label, __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, count * width, label, __FUNCTION__) ||
        !checkRecordIndices(rows, count, mEntries.size(), label, __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < count; i++)
    {
        const PxU32 row = rows ? rows[i] : i;
        const ArticulationEntry& entry = mEntries[row];
        gatherOvStageShapeRow(property, dst + size_t(i) * width, width, entry.numShapes, entry.shapes);
    }
    return true;
}

bool BaseArticulationView::getStaticFrictionsOvStage(const TensorDesc* dstTensor, const PxU32* rows, PxU32 count) const
{
    return getShapePropertyOvStage(OvStageShapeProperty::eStaticFriction, dstTensor, rows, count);
}

bool BaseArticulationView::getDynamicFrictionsOvStage(const TensorDesc* dstTensor, const PxU32* rows, PxU32 count) const
{
    return getShapePropertyOvStage(OvStageShapeProperty::eDynamicFriction, dstTensor, rows, count);
}

bool BaseArticulationView::getRestitutionsOvStage(const TensorDesc* dstTensor, const PxU32* rows, PxU32 count) const
{
    return getShapePropertyOvStage(OvStageShapeProperty::eRestitution, dstTensor, rows, count);
}

bool BaseArticulationView::getContactOffsetsOvStage(const TensorDesc* dstTensor, const PxU32* rows, PxU32 count) const
{
    return getShapePropertyOvStage(OvStageShapeProperty::eContactOffset, dstTensor, rows, count);
}

bool BaseArticulationView::getRestOffsetsOvStage(const TensorDesc* dstTensor, const PxU32* rows, PxU32 count) const
{
    return getShapePropertyOvStage(OvStageShapeProperty::eRestOffset, dstTensor, rows, count);
}

PxU32 BaseArticulationView::inverseDynamicsColumnWidthForRow(PxU32 row, InverseDynamicsColumn column) const
{
    if (row >= mEntries.size() || !mEntries[row].metatype)
        return 0;

    switch (column)
    {
    case InverseDynamicsColumn::eNone:
        return 0;
    case InverseDynamicsColumn::eJacobian:
    {
        PxU32 jacobianRows = 0, jacobianCols = 0;
        return jacobianShapeForRow(row, jacobianRows, jacobianCols) ? jacobianRows * jacobianCols : 0u;
    }
    case InverseDynamicsColumn::eMassMatrix:
    {
        const PxU32 m = generalizedCoordinateCountForRow(row);
        return m * m;
    }
    case InverseDynamicsColumn::eGeneralizedForce:
        return generalizedCoordinateCountForRow(row);
    case InverseDynamicsColumn::eCentroidalMomentum:
        // Floating base only. Zero is the honest answer for a fixed-base row: the reader omits the
        // group rather than emitting an empty one (REQ-READ-INVDYN-001 AC-5).
        return mEntries[row].metatype->getFixedBase() ? 0u : 6u * (mEntries[row].numDofs + 7u);
    case InverseDynamicsColumn::eJacobianShape:
        // Constant two int32 lanes, the jacobian's rows and cols; this entry exists to put the
        // column on the cohort partitioning, not to compute a width.
        return 2u;
    }
    return 0;
}

bool BaseArticulationView::checkInverseDynamicsColumn(const TensorDesc& dstTensor,
                                                      const PxU32* rows,
                                                      PxU32 count,
                                                      InverseDynamicsColumn column,
                                                      PxU32& widthOut,
                                                      const char* label,
                                                      const char* funcName) const
{
    widthOut = dstTensor.numDims >= 2 ? static_cast<PxU32>(dstTensor.dims[1]) : 0;
    if (widthOut == 0)
    {
        CARB_LOG_ERROR("%s: %s needs a column width in dims[1]", funcName, label);
        return false;
    }
    const ArticulationMetatype* cohortMetatype = nullptr;
    for (PxU32 i = 0; i < count; ++i)
    {
        const PxU32 row = rows ? rows[i] : i;
        // Bounds before width: inverseDynamicsColumnWidthForRow answers 0 for a row it cannot resolve, which
        // the comparison below would report as a cohort mismatch. Checking here makes a bad index read
        // the same on the GPU path (which reaches this before checkRecordIndices) and on the CPU path.
        if (row >= mEntries.size())
        {
            CARB_LOG_ERROR("%s: %s row index %u is out of range; the view has %zu row(s)", funcName,
                           label, row, mEntries.size());
            return false;
        }

        // Metatype identity, not merely equal width: getGeneralizedForceColumnOvStage resolves its
        // dof-record block from the cohort's FIRST row and applies that row's per-dof parentage to
        // every row, which only holds under identity. Two topologies can agree on generalized
        // coordinate count and disagree on parentage, and a width-only check would let the device
        // gather flip signs on every row after the first while the CPU sibling stays right.
        if (i == 0)
        {
            cohortMetatype = mEntries[row].metatype;
        }
        else if (mEntries[row].metatype != cohortMetatype)
        {
            CARB_LOG_ERROR("%s: %s row %u has a different articulation topology from row %u -- the "
                           "selection is not one cohort",
                           funcName, label, row, rows ? rows[0] : 0u);
            return false;
        }

        const PxU32 want = inverseDynamicsColumnWidthForRow(row, column);
        if (want != widthOut)
        {
            CARB_LOG_ERROR("%s: %s row %u needs width %u but the column is %u wide -- the selection "
                           "is not one cohort",
                           funcName, label, row, want, widthOut);
            return false;
        }
    }
    return true;
}

bool BaseArticulationView::getJacobianShapesOvStage(const TensorDesc* dstTensor,
                                                    const PxU32* rows,
                                                    PxU32 count) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    const char* label = "articulation jacobian shape";
    if (!checkTensorDevice(*dstTensor, -1, label, __FUNCTION__) ||
        !checkTensorInt32(*dstTensor, label, __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, count * 2u, label, __FUNCTION__) ||
        !checkRecordIndices(rows, count, mEntries.size(), label, __FUNCTION__))
    {
        return false;
    }

    int32_t* dst = static_cast<int32_t*>(dstTensor->data);
    for (PxU32 i = 0; i < count; i++)
    {
        const PxU32 row = rows ? rows[i] : i;
        uint32_t jacobianRows = 0, jacobianCols = 0;
        if (!jacobianShapeForRow(row, jacobianRows, jacobianCols))
        {
            CARB_LOG_ERROR("%s: row %u has no resolvable articulation metatype", __FUNCTION__, row);
            return false;
        }
        dst[size_t(i) * 2 + 0] = static_cast<int32_t>(jacobianRows);
        dst[size_t(i) * 2 + 1] = static_cast<int32_t>(jacobianCols);
    }
    return true;
}

bool BaseArticulationView::getShapeCountsOvStage(const TensorDesc* dstTensor, const PxU32* rows, PxU32 count) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    const char* label = "articulation shape count";
    if (!checkTensorDevice(*dstTensor, -1, label, __FUNCTION__) ||
        !checkTensorInt32(*dstTensor, label, __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, count, label, __FUNCTION__) ||
        !checkRecordIndices(rows, count, mEntries.size(), label, __FUNCTION__))
    {
        return false;
    }

    int32_t* dst = static_cast<int32_t*>(dstTensor->data);
    for (PxU32 i = 0; i < count; i++)
    {
        const PxU32 row = rows ? rows[i] : i;
        const ArticulationEntry& entry = mEntries[row];
        // Clamped, not declared: this tells a consumer where the real values stop, so it must match
        // how many gatherOvStageShapeRow wrote. Same rule as BaseRigidBodyView.
        dst[i] = static_cast<int32_t>(effectiveOvStageShapeCount(entry.numShapes, entry.shapes));
    }
    return true;
}

uint32_t BaseArticulationView::maxShapesForRows(const PxU32* rows, PxU32 count) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, 0);
    PxU32 width = 0;
    for (PxU32 i = 0; i < count; i++)
    {
        const PxU32 row = rows ? rows[i] : i;
        if (row < mEntries.size())
        {
            const ArticulationEntry& entry = mEntries[row];
            width = std::max(width, effectiveOvStageShapeCount(entry.numShapes, entry.shapes));
        }
    }
    return width;
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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
                PxMaterial* material = mSim->createSharedMaterial(
                    src[j * 3], src[j * 3 + 1], src[j * 3 + 2], 0.0, PxCombineMode::Enum::eAVERAGE,
                    PxCombineMode::Enum::eAVERAGE, PxCombineMode::Enum::eAVERAGE);

                int nMaterials = mEntries[idx].shapes[j]->getNbMaterials();
                std::vector<PxMaterial*> extraMats;
                extraMats.resize(nMaterials);

                mEntries[idx].shapes[j]->getMaterials(extraMats.data(), (PxU32)extraMats.size(), 0);

                for (auto mat : extraMats)
                {
                    mSim->releaseSharedMaterial(mat);
                }

                mEntries[idx].shapes[j]->setMaterials(&material, 1);
            }
        }
    }

    return true;
}
bool BaseArticulationView::setCompliantMaterialProperties(const TensorDesc* srcTensor,
                                                          const TensorDesc* srcCombineModeTensor,
                                                          const TensorDesc* indexTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "material properties", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "material properties", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxShapes * 4u, "material properties", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*srcCombineModeTensor, -1, "combination modes", __FUNCTION__) ||
        !checkTensorInt8(*srcCombineModeTensor, "combination modes", __FUNCTION__) ||
        !checkTensorSizeExact(*srcCombineModeTensor, getCount() * mMaxShapes * 3u, "combination modes", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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
            float* src = static_cast<float*>(srcTensor->data) + idx * mMaxShapes * 4;
            uint8_t* srcCombineMode = static_cast<uint8_t*>(srcCombineModeTensor->data) + idx * mMaxShapes * 3;

            for (PxU32 j = 0; j < mEntries[idx].numShapes; j++)
            {
                PxMaterial* material =
                    mSim->createSharedMaterial(src[j * 4], src[j * 4 + 1], -src[j * 4 + 2], src[j * 4 + 3],
                                               static_cast<PxCombineMode::Enum>(srcCombineMode[j * 3]),
                                               static_cast<PxCombineMode::Enum>(srcCombineMode[j * 3 + 1]),
                                               static_cast<PxCombineMode::Enum>(srcCombineMode[j * 3 + 2]));

                int nMaterials = mEntries[idx].shapes[j]->getNbMaterials();
                std::vector<PxMaterial*> extraMats;
                extraMats.resize(nMaterials);

                mEntries[idx].shapes[j]->getMaterials(extraMats.data(), (PxU32)extraMats.size(), 0);

                for (auto mat : extraMats)
                {
                    mSim->releaseSharedMaterial(mat);
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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

    if (!g_physx)
    {
        return false;
    }

    usdparser::AttachedStage* attachedStage = mSim ? mSim->getAttachedStage() : nullptr;
    for (auto& entry : mEntries)
    {
        const omni::physics::parse::ObjectKey key =
            attachedStage ? attachedStage->keyFor(entry.path) : omni::physics::parse::ObjectKey{};
        void* ptr = BaseSimulationView::resolvePhysXPtr(attachedStage, key, omni::physx::PhysXType::ePTArticulation);
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
    mSim = nullptr;
}

// ---------------------------------------------------------------------------
// CPU-only property masked wrappers (OMPE-103213). Host mask only.
// ---------------------------------------------------------------------------

using omni::physics::tensors::MaskResult;
using omni::physics::tensors::resolveMaskToIndices;
using omni::physics::tensors::makeIndexTensorDesc;

bool BaseArticulationView::setDisableGravitiesMasked(const TensorDesc* src, const TensorDesc* mask)
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(mask, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setDisableGravities(src, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setDisableGravities(src, &idx);
}

bool BaseArticulationView::setMaterialPropertiesMasked(const TensorDesc* src, const TensorDesc* mask) const
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(mask, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setMaterialProperties(src, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setMaterialProperties(src, &idx);
}

bool BaseArticulationView::setCompliantMaterialPropertiesMasked(const TensorDesc* src,
                                                                const TensorDesc* srcCombine,
                                                                const TensorDesc* mask) const
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(mask, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setCompliantMaterialProperties(src, srcCombine, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setCompliantMaterialProperties(src, srcCombine, &idx);
}

bool BaseArticulationView::setRestOffsetsMasked(const TensorDesc* src, const TensorDesc* mask) const
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(mask, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setRestOffsets(src, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setRestOffsets(src, &idx);
}

bool BaseArticulationView::setContactOffsetsMasked(const TensorDesc* src, const TensorDesc* mask) const
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(mask, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setContactOffsets(src, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setContactOffsets(src, &idx);
}

}
}
}
