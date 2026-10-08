// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-TENSOR-INDEX-001
 * @covers AC-3
 *
 * @implements REQ-TENSOR-ATTACH-001
 * @covers AC-1
 *
 * @implements REQ-TENSOR-CPU-ONLY-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-READ-ATTRS-001
 * @covers AC-1, AC-7, AC-8, AC-10, AC-11, AC-12, AC-14
 *
 * The ovstage body-property columns at the end of this file: one host implementation serving both
 * backends, because these are authored inputs with no device source for a backend to differ over.
 */

// clang-format off
// clang-format on

#include "tensors/base/BaseRigidBodyView.h"
#include "tensors/base/BaseSimulationView.h"
#include "usdLoad/AttachedStage.h"
#include "tensors/base/OvStageShapeProperty.h"

#include "tensors/GlobalsAreBad.h"
#include "tensors/CommonTypes.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>

#include <omni/physics/tensors/TensorUtils.h>

#include <algorithm>

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

BaseRigidBodyView::BaseRigidBodyView(BaseSimulationView* sim, const std::vector<RigidBodyEntry>& entries)
    : mSim(sim), mEntries(entries)
{
    if (mSim)
    {
        // acquire a shared data pointer so the stuff we need doesn't get deleted
        mSimData = mSim->getBaseSimulationData();

        // initialize default indices
        uint32_t numBodies = uint32_t(mEntries.size());
        mAllIndices.resize(numBodies);
        for (PxU32 i = 0; i < numBodies; i++)
        {
            mAllIndices[i] = i;
            mSim->rigidBodies.insert(mEntries[i].body);
            for (auto ele : mEntries[i].shapes)
                mSim->shapes.insert(ele);
            if (mEntries[i].numShapes > mMaxShapes)
            {
                mMaxShapes = mEntries[i].numShapes;
            }
        }
    }
}

BaseRigidBodyView::~BaseRigidBodyView()
{
    if (mSim)
    {
        mSim->_onChildRelease(this);
    }
}

uint32_t BaseRigidBodyView::getCount() const
{
    return uint32_t(mEntries.size());
}

uint32_t BaseRigidBodyView::getMaxShapes() const
{
    return mMaxShapes;
}

const char* BaseRigidBodyView::getUsdPrimPath(uint32_t rbIdx) const 
{
    if (rbIdx < mEntries.size())
    {
        return mEntries[rbIdx].path.c_str();
    }
    return nullptr;
}

bool BaseRigidBodyView::getMasses(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "masses", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "masses", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount(), "masses", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxReal mass = mEntries[i].body->getMass();
        *dst++ = mass;
    }

    return true;
}

bool BaseRigidBodyView::getInvMasses(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "inv masses", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "inv masses", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount(), "inv masses", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxReal invMass = mEntries[i].body->getInvMass();
        *dst++ = invMass;
    }

    return true;
}

bool BaseRigidBodyView::getCOMs(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "com", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "com", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 7u, "com", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxTransform comPose = mEntries[i].body->getCMassLocalPose();
        *dst++ = comPose.p.x;
        *dst++ = comPose.p.y;
        *dst++ = comPose.p.z;
        *dst++ = comPose.q.x;
        *dst++ = comPose.q.y;
        *dst++ = comPose.q.z;
        *dst++ = comPose.q.w;
    }

    return true;
}

bool BaseRigidBodyView::getInertias(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "inertia", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "inertia", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 9u, "inertia", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxTransform comPose = mEntries[i].body->getCMassLocalPose();
        PxMat33 R{ comPose.q }; // the matrix that diagonalizes the inertia i.e. I = R * D * R'
        PxMat33 Rt = R.getTranspose();

        PxMat33 massSpaceDiagInertia = PxMat33::createDiagonal(mEntries[i].body->getMassSpaceInertiaTensor());
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

    return true;
}

bool BaseRigidBodyView::getInvInertias(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "inv inertia", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "inv inertia", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 9u, "inv inertia", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxTransform comPose = mEntries[i].body->getCMassLocalPose();
        PxMat33 R{ comPose.q }; // the matrix that diagonalizes the inertia i.e. I = R * D * R'
        PxMat33 Rt = R.getTranspose();

        PxMat33 massSpaceDiagInvInertia = PxMat33::createDiagonal(mEntries[i].body->getMassSpaceInvInertiaTensor());
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

    return true;
}

bool BaseRigidBodyView::getDisableGravities(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "disable gravity", __FUNCTION__) ||
        !checkTensorInt8(*dstTensor, "disable gravity", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount(), "disable gravity", __FUNCTION__))
    {
        return false;
    }

    uint8_t* dst = static_cast<uint8_t*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        *dst++ = (mEntries[i].body->getActorFlags() & PxActorFlag::eDISABLE_GRAVITY) ? 1 : 0;
    }

    return true;
}

bool BaseRigidBodyView::getDisableSimulations(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "disable simulation", __FUNCTION__) ||
        !checkTensorInt8(*dstTensor, "disable simulation", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount(), "disable simulation", __FUNCTION__))
    {
        return false;
    }

    uint8_t* dst = static_cast<uint8_t*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        *dst++ = (mEntries[i].body->getActorFlags() & PxActorFlag::eDISABLE_SIMULATION) ? 1 : 0;
    }

    return true;
}

bool BaseRigidBodyView::setMasses(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "mass", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "mass", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount(), "mass", __FUNCTION__))
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
            const float* src = static_cast<const float*>(srcTensor->data) + idx;
            mEntries[idx].body->setMass(*src);
        }
    }

    return true;
}

bool BaseRigidBodyView::setCOMs(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "com", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "com", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * 7, "com", __FUNCTION__))
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
            const float* src = static_cast<const float*>(srcTensor->data) + idx * 7;
            PxVec3 comPos{ src[0], src[1], src[2] };
            PxQuat comRot{ src[3], src[4], src[5], src[6] };
            mEntries[idx].body->setCMassLocalPose(PxTransform(comPos, comRot));
        }
    }

    setComsCacheStateValid(false);
    return true;
}

bool BaseRigidBodyView::setInertias(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "inertia", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "inertia", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * 9u, "inertia", __FUNCTION__))
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
            const float* src = static_cast<const float*>(srcTensor->data) + idx * 9;
            PxMat33 inertia;
            inertia.column0 = { src[0], src[1], src[2] };
            inertia.column1 = { src[3], src[4], src[5] };
            inertia.column2 = { src[6], src[7], src[8] };
            // diagnoalize the inertia tensor and update diagonal inertia and the inertial frame axes
            PxQuat axes;
            PxVec3 diagInertia = PxDiagonalize(inertia, axes);
            mEntries[idx].body->setMassSpaceInertiaTensor(diagInertia);
            PxTransform comPose = mEntries[idx].body->getCMassLocalPose();
            comPose.q = axes;
            mEntries[idx].body->setCMassLocalPose(comPose);
        }
    }

    return true;
}

bool BaseRigidBodyView::setDisableGravities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "disable gravity", __FUNCTION__) ||
        !checkTensorInt8(*srcTensor, "disable gravity", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount(), "disable gravity", __FUNCTION__))
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
            const uint8_t* src = static_cast<const uint8_t*>(srcTensor->data) + idx;
            mEntries[idx].body->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, *src);
        }
    }

    return true;
}

bool BaseRigidBodyView::setDisableSimulations(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "disable simulation", __FUNCTION__) ||
        !checkTensorInt8(*srcTensor, "disable simulation", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount(), "disable simulation", __FUNCTION__))
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
            const uint8_t* src = static_cast<const uint8_t*>(srcTensor->data) + idx;
            mEntries[idx].body->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, *src);
        }
    }

    return true;
}

bool BaseRigidBodyView::wakeUp(const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);

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
            if (mEntries[idx].type == RigidBodyType::eRigidDynamic)
            {
                PxRigidDynamic* dynamicBody = static_cast<PxRigidDynamic*>(mEntries[idx].body);
                // skip bodies that have the disable simulation flag set
                if(!dynamicBody->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION))
                {
                    dynamicBody->wakeUp();
                }
            }
        }
    }

    return true;
}

bool BaseRigidBodyView::putToSleep(const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);

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
            if (mEntries[idx].type == RigidBodyType::eRigidDynamic)
            {
                PxRigidDynamic* dynamicBody = static_cast<PxRigidDynamic*>(mEntries[idx].body);
                if (!dynamicBody->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION))
                    dynamicBody->putToSleep();
            }
        }
    }

    return true;
}

bool BaseRigidBodyView::getMaterialProperties(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
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

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numShapes; j++)
        {
            PxMaterial* material;
            mEntries[i].shapes[j]->getMaterials(&material, 1);
            dst[3 * i * mMaxShapes + 3 * j + 0] = material->getStaticFriction();
            dst[3 * i * mMaxShapes + 3 * j + 1] = material->getDynamicFriction();
            dst[3 * i * mMaxShapes + 3 * j + 2] = material->getRestitution();
        }

    }

    return true;
}

bool BaseRigidBodyView::getCompliantMaterialProperties(const TensorDesc* dstTensor,
                                                       const TensorDesc* dstCombineModeTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
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

    float* dst = static_cast<float*>(dstTensor->data);
    uint8_t* dstModes = static_cast<uint8_t*>(dstCombineModeTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numShapes; j++)
        {
            PxMaterial* material;
            mEntries[i].shapes[j]->getMaterials(&material, 1);
            float restitution = material->getRestitution();

            dst[2 * i * mMaxShapes + 2 * j + 0] =
                restitution < 0.0f ? -restitution : std::numeric_limits<float>::infinity();
            dst[2 * i * mMaxShapes + 2 * j + 1] = material->getDamping();
            dstModes[2 * i * mMaxShapes + 2 * j + 0] = static_cast<uint8_t>(material->getRestitutionCombineMode());
            dstModes[2 * i * mMaxShapes + 2 * j + 1] = static_cast<uint8_t>(material->getDampingCombineMode());
        }
    }

    return true;
}

bool BaseRigidBodyView::getRestOffsets(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
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

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numShapes; j++)
        {
            dst[i * mMaxShapes + j] = mEntries[i].shapes[j]->getRestOffset();
        }
    }

    return true;
}

bool BaseRigidBodyView::getContactOffsets(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
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

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numShapes; j++)
        {
             dst[i * mMaxShapes + j] = mEntries[i].shapes[j]->getContactOffset();
        }
    }

    return true;
}

bool BaseRigidBodyView::setMaterialProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
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
                PxMaterial* material = mSim->createSharedMaterial(src[j * 3], src[j * 3 + 1], src[j * 3 + 2], 0.0f,
                                                                  PxCombineMode::eAVERAGE, PxCombineMode::eAVERAGE,
                                                                  PxCombineMode::eAVERAGE);

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

bool BaseRigidBodyView::setCompliantMaterialProperties(const TensorDesc* srcTensor,
                                                       const TensorDesc* srcCombineModeTensor,
                                                       const TensorDesc* indexTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
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
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxShapes * 4;
            const uint8_t* srcCombineMode = static_cast<const uint8_t*>(srcCombineModeTensor->data) + idx * mMaxShapes * 3;
            for (PxU32 j = 0; j < mEntries[idx].numShapes; j++)
            {
                PxMaterial* material =
                    mSim->createSharedMaterial(src[j * 4], src[j * 4 + 1], -src[j * 4 + 2], src[j * 4 + 3],
                                               static_cast<PxCombineMode::Enum>(srcCombineMode[j * 3 + 0]),
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

bool BaseRigidBodyView::setRestOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
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

bool BaseRigidBodyView::setContactOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
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

bool BaseRigidBodyView::check() const
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
        if (entry.type == RigidBodyType::eRigidDynamic)
        {
            void* ptr = BaseSimulationView::resolvePhysXPtr(attachedStage, key, omni::physx::PhysXType::ePTActor);
            if (ptr != entry.body)
            {
                result = false;
            }
        }
        else if (entry.type == RigidBodyType::eArticulationLink)
        {
            void* ptr = BaseSimulationView::resolvePhysXPtr(attachedStage, key, omni::physx::PhysXType::ePTLink);
            if (ptr != entry.body)
            {
                result = false;
            }
        }
        else
        {
            result = false;
        }
    }

    return result;
}

void BaseRigidBodyView::release()
{
    delete this;
}

void BaseRigidBodyView::_onParentRelease()
{
    mSim = nullptr;
}

// ---------------------------------------------------------------------------
// CPU-only property masked wrappers (OMPE-103213). Host mask only; forwards to
// the virtual indexed setter so Gpu overrides of setDisable* still run.
// ---------------------------------------------------------------------------

using omni::physics::tensors::MaskResult;
using omni::physics::tensors::resolveMaskToIndices;
using omni::physics::tensors::makeIndexTensorDesc;

bool BaseRigidBodyView::setDisableGravitiesMasked(const TensorDesc* src, const TensorDesc* mask)
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(mask, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setDisableGravities(src, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setDisableGravities(src, &idx);
}

bool BaseRigidBodyView::setDisableSimulationsMasked(const TensorDesc* src, const TensorDesc* mask)
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(mask, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setDisableSimulations(src, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setDisableSimulations(src, &idx);
}

bool BaseRigidBodyView::setMaterialPropertiesMasked(const TensorDesc* src, const TensorDesc* mask) const
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(mask, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setMaterialProperties(src, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setMaterialProperties(src, &idx);
}

bool BaseRigidBodyView::setCompliantMaterialPropertiesMasked(const TensorDesc* src,
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

bool BaseRigidBodyView::setRestOffsetsMasked(const TensorDesc* src, const TensorDesc* mask) const
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(mask, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setRestOffsets(src, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setRestOffsets(src, &idx);
}

bool BaseRigidBodyView::setContactOffsetsMasked(const TensorDesc* src, const TensorDesc* mask) const
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(mask, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setContactOffsets(src, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setContactOffsets(src, &idx);
}

// ----------------------------------------------------------------------------------------------
// ovstage property columns (REQ-READ-ATTRS-001 AC-1)
// ----------------------------------------------------------------------------------------------

BaseRigidBodyView::BodyPropertyLayout BaseRigidBodyView::layoutOf(const BodyProperty prop)
{
    // One row per enum value, in declaration order. Labels match what the whole-view getters above log
    // for the same quantity, so a rejection reads the same whichever entry point produced it; the com
    // columns are the exception, since getCOMs logs one "com" for the whole 7-tuple.
    static constexpr BodyPropertyLayout kLayouts[] = {
        { 1, false, false, "masses" },            // eMass
        { 1, false, false, "inv masses" },        // eInvMass
        { 9, false, false, "inertia" },           // eInertia
        { 9, false, false, "inv inertia" },       // eInvInertia
        { 3, false, false, "com position" },      // eComPosition
        { 4, false, false, "com orientation" },   // eComOrientation
        { 1, true, false, "disable gravity" },    // eDisableGravity
        { 1, true, false, "disable simulation" }, // eDisableSimulation
        // Per-shape: width 0 because it belongs to the read (its widest body), not to the attribute;
        // the gather takes it from the destination tensor instead.
        { 0, false, true, "static friction" },  // eStaticFriction
        { 0, false, true, "dynamic friction" }, // eDynamicFriction
        { 0, false, true, "restitution" },      // eRestitution
        { 0, false, true, "contact offset" },   // eContactOffset
        { 0, false, true, "rest offset" },      // eRestOffset
        // One int32 per body, not per shape: how many of a per-shape row's entries are real.
        { 1, false, false, "shape count" }, // eShapeCount
    };
    static_assert(sizeof(kLayouts) / sizeof(kLayouts[0]) == static_cast<size_t>(BodyProperty::eCount),
                  "kLayouts must have exactly one row per BodyProperty, in declaration order");

    CARB_ASSERT(prop < BodyProperty::eCount);
    return kLayouts[static_cast<size_t>(prop)];
}

namespace
{
// PhysX stores the inertia (or its inverse) as a mass-space diagonal D plus the rotation R that
// diagonalizes it; this reassembles I = R * D * R'.
//
// R is getCMassLocalPose().q, so the result is in body-local coordinates taken about the centre of
// mass -- not the world frame, and not the COM's principal frame (which is D itself). "COM frame" is
// the house term for exactly this. getInertias/getInvInertias reconstruct it the same way inline.
PxMat33 comFrameInertia(const PxRigidBody& body, bool inverse)
{
    const PxTransform comPose = body.getCMassLocalPose();
    const PxMat33 R{ comPose.q };
    const PxMat33 diag = PxMat33::createDiagonal(inverse ? body.getMassSpaceInvInertiaTensor() :
                                                           body.getMassSpaceInertiaTensor());
    return R * diag * R.getTranspose();
}
} // namespace

bool BaseRigidBodyView::ovStageShapePropertyOf(const BodyProperty prop, OvStageShapeProperty& out)
{
    // Every per-shape enumerator gets its own case: a new one added to the table without a case here
    // answers false and is refused by name, rather than publishing another property's values.
    switch (prop)
    {
    case BodyProperty::eStaticFriction:
        out = OvStageShapeProperty::eStaticFriction;
        return true;
    case BodyProperty::eDynamicFriction:
        out = OvStageShapeProperty::eDynamicFriction;
        return true;
    case BodyProperty::eContactOffset:
        out = OvStageShapeProperty::eContactOffset;
        return true;
    case BodyProperty::eRestOffset:
        out = OvStageShapeProperty::eRestOffset;
        return true;
    case BodyProperty::eRestitution:
        out = OvStageShapeProperty::eRestitution;
        return true;
    default:
        return false;
    }
}

bool BaseRigidBodyView::getBodyPropertyOvStage(const BodyProperty prop,
                                              const TensorDesc* const dstTensor,
                                              const PxU32* const outRecordIdx,
                                              const PxU32 numOutputs) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    if (!dstTensor || !dstTensor->data)
        return false;

    const BodyPropertyLayout layout = layoutOf(prop);
    // A per-shape column's width comes from the destination the emit sized; everything else has a
    // fixed width from the table. dims[1] is the column width both branches agree on.
    const uint32_t comp = layout.perShape ?
                              (dstTensor->numDims >= 2 ? static_cast<uint32_t>(dstTensor->dims[1]) : 0u) :
                              layout.comp;
    if (comp == 0)
        return false;

    // Host tensor required by contract: none of these properties has a device source (see the header),
    // so a device pointer is refused rather than written to from the host.
    const omni::physics::tensors::TensorDataType expectedType =
        (prop == BodyProperty::eShapeCount) ? omni::physics::tensors::TensorDataType::eInt32 :
        layout.isByte                       ? omni::physics::tensors::TensorDataType::eUint8 :
                                              omni::physics::tensors::TensorDataType::eFloat32;
    if (!checkTensorDevice(*dstTensor, -1, layout.label, __FUNCTION__) ||
        !omni::physics::tensors::checkTensorDataType(*dstTensor, expectedType, layout.label, __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, numOutputs * comp, layout.label, __FUNCTION__) ||
        !omni::physics::tensors::checkRecordIndices(outRecordIdx, numOutputs, mEntries.size(), layout.label,
                                                   __FUNCTION__))
    {
        return false;
    }

    float* fdst = layout.isByte ? nullptr : static_cast<float*>(dstTensor->data);
    uint8_t* bdst = layout.isByte ? static_cast<uint8_t*>(dstTensor->data) : nullptr;

    // Per-shape columns write by index, not by an advancing cursor: a body narrower than the column
    // leaves a zero-filled tail, and shapeCount tells a consumer where the real values stop.
    if (layout.perShape)
    {
        OvStageShapeProperty shapeProperty = OvStageShapeProperty::eRestitution;
        if (!ovStageShapePropertyOf(prop, shapeProperty))
        {
            // Unreachable while layout.perShape and ovStageShapePropertyOf agree; refuse rather than
            // guess a property.
            CARB_LOG_ERROR("%s: per-shape property %d has no OvStageShapeProperty mapping",
                           __FUNCTION__, static_cast<int>(prop));
            return false;
        }
        for (PxU32 i = 0; i < numOutputs; i++)
        {
            const PxU32 recIdx = outRecordIdx ? outRecordIdx[i] : i;
            const RigidBodyEntry& entry = mEntries[recIdx];
            gatherOvStageShapeRow(shapeProperty, fdst + size_t(i) * comp, comp, entry.numShapes, entry.shapes);
        }
        return true;
    }

    if (prop == BodyProperty::eShapeCount)
    {
        int32_t* dst = static_cast<int32_t*>(dstTensor->data);
        for (PxU32 i = 0; i < numOutputs; i++)
        {
            const PxU32 recIdx = outRecordIdx ? outRecordIdx[i] : i;
            const RigidBodyEntry& entry = mEntries[recIdx];
            // The clamped count, not the declared one: it must match how many values
            // gatherOvStageShapeRow actually wrote, or padded zeros read back as data.
            *dst++ = static_cast<int32_t>(effectiveOvStageShapeCount(entry.numShapes, entry.shapes));
        }
        return true;
    }

    for (PxU32 i = 0; i < numOutputs; i++)
    {
        const PxU32 recIdx = outRecordIdx ? outRecordIdx[i] : i;
        const PxRigidBody& body = *mEntries[recIdx].body;
        switch (prop)
        {
        case BodyProperty::eMass:
            *fdst++ = body.getMass();
            break;
        case BodyProperty::eInvMass:
            *fdst++ = body.getInvMass();
            break;
        case BodyProperty::eInertia:
        case BodyProperty::eInvInertia:
        {
            const PxMat33 m = comFrameInertia(body, prop == BodyProperty::eInvInertia);
            *fdst++ = m.column0.x;
            *fdst++ = m.column0.y;
            *fdst++ = m.column0.z;
            *fdst++ = m.column1.x;
            *fdst++ = m.column1.y;
            *fdst++ = m.column1.z;
            *fdst++ = m.column2.x;
            *fdst++ = m.column2.y;
            *fdst++ = m.column2.z;
            break;
        }
        case BodyProperty::eComPosition:
        {
            const PxVec3 p = body.getCMassLocalPose().p;
            *fdst++ = p.x;
            *fdst++ = p.y;
            *fdst++ = p.z;
            break;
        }
        case BodyProperty::eComOrientation:
        {
            const PxQuat q = body.getCMassLocalPose().q;
            *fdst++ = q.x;
            *fdst++ = q.y;
            *fdst++ = q.z;
            *fdst++ = q.w;
            break;
        }
        case BodyProperty::eDisableGravity:
            *bdst++ = (body.getActorFlags() & PxActorFlag::eDISABLE_GRAVITY) ? 1 : 0;
            break;
        case BodyProperty::eDisableSimulation:
            *bdst++ = (body.getActorFlags() & PxActorFlag::eDISABLE_SIMULATION) ? 1 : 0;
            break;
        case BodyProperty::eCount:
            break; // not a property; the per-shape and shapeCount cases returned above
        }
    }
    return true;
}

// The multi-column form of the per-shape branch above.
bool BaseRigidBodyView::getShapePropertyColumnsOvStage(const ShapePropertyColumn* const columns,
                                                       const PxU32 numColumns,
                                                       const PxU32* const outRecordIdx,
                                                       const PxU32 numOutputs) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    if (numColumns == 0 || numOutputs == 0)
        return true; // nothing to emit -- not a failure
    if (!columns)
    {
        return false;
    }

    // Validated per column with its own label, so a rejection names the column that caused it.
    std::vector<OvStageShapeColumn> shapeColumns(numColumns);
    uint32_t width = 0;
    for (PxU32 c = 0; c < numColumns; ++c)
    {
        const TensorDesc* const dst = columns[c].dst;
        const OvStageShapeProperty shapeProperty = columns[c].property;
        if (shapeProperty == OvStageShapeProperty::eNone)
        {
            CARB_LOG_ERROR("%s: eNone is not a per-shape column", __FUNCTION__);
            return false;
        }
        const char* const label = ovStageShapePropertyLabel(shapeProperty);
        if (!dst || !dst->data)
        {
            return false;
        }

        // The emit sizes every per-shape column of one read to the same width, and
        // gatherOvStageShapeRows writes them all at the same shape index, so a disagreement is a
        // caller bug.
        const uint32_t comp = dst->numDims >= 2 ? static_cast<uint32_t>(dst->dims[1]) : 0u;
        if (comp == 0 || (c > 0 && comp != width))
        {
            CARB_LOG_ERROR("%s: per-shape columns disagree about width (%u after %u)", __FUNCTION__, comp, width);
            return false;
        }
        width = comp;

        if (!checkTensorDevice(*dst, -1, label, __FUNCTION__) ||
            !omni::physics::tensors::checkTensorDataType(
                *dst, omni::physics::tensors::TensorDataType::eFloat32, label, __FUNCTION__) ||
            !checkTensorSizeExact(*dst, numOutputs * comp, label, __FUNCTION__))
        {
            return false;
        }
        shapeColumns[c].property = shapeProperty;
        shapeColumns[c].dst = nullptr; // per row, below
    }

    if (!omni::physics::tensors::checkRecordIndices(outRecordIdx, numOutputs, mEntries.size(),
                                                   ovStageShapePropertySetLabel(), __FUNCTION__))
    {
        return false;
    }

    // One walk of each body's shapes for the whole column set, so a shape's material resolves once.
    for (PxU32 i = 0; i < numOutputs; ++i)
    {
        const PxU32 recIdx = outRecordIdx ? outRecordIdx[i] : i;
        const RigidBodyEntry& entry = mEntries[recIdx];
        for (PxU32 c = 0; c < numColumns; ++c)
        {
            shapeColumns[c].dst = static_cast<float*>(columns[c].dst->data) + size_t(i) * width;
        }
        gatherOvStageShapeRows(shapeColumns.data(), numColumns, width, entry.numShapes, entry.shapes);
    }
    return true;
}

// The write counterpart of getBodyPropertyOvStage. Same layout table, same record indirection, same
// host-only contract -- see the header for why one implementation serves both backends here.
bool BaseRigidBodyView::setBodyPropertyOvStage(const BodyProperty prop,
                                               const TensorDesc* const srcTensor,
                                               const PxU32* const outRecordIdx,
                                               const PxU32 numOutputs)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    if (!srcTensor || !srcTensor->data)
        return false;

    // Derived and structural properties have no setter. Refused here rather than reaching a switch
    // with no case for them, so a caller learns the name is not writable instead of getting silence.
    if (prop == BodyProperty::eInvMass || prop == BodyProperty::eInvInertia ||
        prop == BodyProperty::eShapeCount || prop == BodyProperty::eCount)
    {
        CARB_LOG_ERROR("%s is not writable: it is derived from other properties or structural",
                       layoutOf(prop).label);
        return false;
    }

    const BodyPropertyLayout layout = layoutOf(prop);
    const uint32_t comp = layout.perShape ?
                              (srcTensor->numDims >= 2 ? static_cast<uint32_t>(srcTensor->dims[1]) : 0u) :
                              layout.comp;
    if (comp == 0)
        return false;

    const omni::physics::tensors::TensorDataType expectedType =
        layout.isByte ? omni::physics::tensors::TensorDataType::eUint8 :
                        omni::physics::tensors::TensorDataType::eFloat32;
    if (!checkTensorDevice(*srcTensor, -1, layout.label, __FUNCTION__) ||
        !omni::physics::tensors::checkTensorDataType(*srcTensor, expectedType, layout.label, __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, numOutputs * comp, layout.label, __FUNCTION__) ||
        !omni::physics::tensors::checkRecordIndices(outRecordIdx, numOutputs, mEntries.size(), layout.label,
                                                    __FUNCTION__))
    {
        return false;
    }

    const float* fsrc = layout.isByte ? nullptr : static_cast<const float*>(srcTensor->data);
    const uint8_t* bsrc = layout.isByte ? static_cast<const uint8_t*>(srcTensor->data) : nullptr;

    // Per-shape columns read by index, mirroring the gather: a body narrower than the column ignores
    // the padded tail rather than writing shapes it does not have.
    if (layout.perShape)
    {
        for (PxU32 i = 0; i < numOutputs; i++)
        {
            const PxU32 recIdx = outRecordIdx ? outRecordIdx[i] : i;
            const RigidBodyEntry& entry = mEntries[recIdx];
            const PxU32 nShapes =
                std::min<PxU32>(std::min<PxU32>(entry.numShapes, static_cast<PxU32>(entry.shapes.size())), comp);
            for (PxU32 j = 0; j < nShapes; j++)
            {
                PxShape* const shape = entry.shapes[j];
                if (!shape)
                    continue;
                const float v = fsrc[size_t(i) * comp + j];
                if (prop == BodyProperty::eContactOffset)
                {
                    shape->setContactOffset(v);
                }
                else if (prop == BodyProperty::eRestOffset)
                {
                    shape->setRestOffset(v);
                }
                else
                {
                    // Material 0 only, matching the gather and the tensor API. A material is SHARED:
                    // writing a friction coefficient through one shape changes it for every shape
                    // bound to the same material, which is PhysX's model rather than something this
                    // layer can localise.
                    PxMaterial* material = nullptr;
                    if (shape->getMaterials(&material, 1) == 0 || !material)
                        continue;
                    if (prop == BodyProperty::eStaticFriction)
                        material->setStaticFriction(v);
                    else if (prop == BodyProperty::eDynamicFriction)
                        material->setDynamicFriction(v);
                    else
                        material->setRestitution(v);
                }
            }
        }
        return true;
    }

    for (PxU32 i = 0; i < numOutputs; i++)
    {
        const PxU32 recIdx = outRecordIdx ? outRecordIdx[i] : i;
        PxRigidBody& body = *mEntries[recIdx].body;
        switch (prop)
        {
        case BodyProperty::eMass:
            body.setMass(*fsrc++);
            break;
        case BodyProperty::eInertia:
        {
            PxMat33 inertia;
            inertia.column0 = { fsrc[0], fsrc[1], fsrc[2] };
            inertia.column1 = { fsrc[3], fsrc[4], fsrc[5] };
            inertia.column2 = { fsrc[6], fsrc[7], fsrc[8] };
            fsrc += 9;
            // PhysX stores a DIAGONAL mass-space tensor plus the principal axes, so a full COM-frame
            // matrix has to be diagonalised -- the same thing setInertiasMasked does, and the reason
            // the gather rebuilds R * diag * R^T on the way out.
            //
            // CONSEQUENCE, and it is not hidden: this OVERWRITES centerOfMassOrientation with the
            // principal axes. The two attributes are separable on the read and coupled on the write,
            // because that is how the engine stores them. A caller writing both should write inertia
            // first.
            PxQuat axes;
            const PxVec3 diag = PxDiagonalize(inertia, axes);
            body.setMassSpaceInertiaTensor(diag);
            PxTransform comPose = body.getCMassLocalPose();
            comPose.q = axes;
            body.setCMassLocalPose(comPose);
            break;
        }
        case BodyProperty::eComPosition:
        {
            // Read-modify-write: the COM pose is one PxTransform and a session writes one attribute,
            // so the orientation half has to come from the body.
            PxTransform comPose = body.getCMassLocalPose();
            comPose.p = PxVec3(fsrc[0], fsrc[1], fsrc[2]);
            fsrc += 3;
            body.setCMassLocalPose(comPose);
            break;
        }
        case BodyProperty::eComOrientation:
        {
            PxTransform comPose = body.getCMassLocalPose();
            comPose.q = PxQuat(fsrc[0], fsrc[1], fsrc[2], fsrc[3]);
            fsrc += 4;
            body.setCMassLocalPose(comPose);
            break;
        }
        case BodyProperty::eDisableGravity:
            body.setActorFlag(PxActorFlag::eDISABLE_GRAVITY, *bsrc++ != 0);
            break;
        case BodyProperty::eDisableSimulation:
            // Toggling this frees or reallocates the body's DirectGPU island index, which every
            // packed write and every rigid gather indexes through. This host setter cannot reach
            // GpuRigidBodyView. The ovstage scatter calls markRdDisableDirty() after a successful
            // disableSimulation write so this view and siblings refresh before the next device
            // operation. Other callers still rely on the OMPE-94459 epoch / the next
            // refreshRdGpuIndices.
            body.setActorFlag(PxActorFlag::eDISABLE_SIMULATION, *bsrc++ != 0);
            break;
        default:
            break; // the read-only and per-shape cases returned above
        }
    }
    return true;
}

bool BaseRigidBodyView::setMassesOvStage(const TensorDesc* srcTensor, const PxU32* outRecordIdx, const PxU32 numOutputs, uint64_t)
{
    return setBodyPropertyOvStage(BodyProperty::eMass, srcTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::setInertiasOvStage(const TensorDesc* srcTensor, const PxU32* outRecordIdx, const PxU32 numOutputs, uint64_t)
{
    return setBodyPropertyOvStage(BodyProperty::eInertia, srcTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::setComPositionsOvStage(const TensorDesc* srcTensor, const PxU32* outRecordIdx, const PxU32 numOutputs, uint64_t)
{
    return setBodyPropertyOvStage(BodyProperty::eComPosition, srcTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::setComOrientationsOvStage(const TensorDesc* srcTensor, const PxU32* outRecordIdx, const PxU32 numOutputs, uint64_t)
{
    return setBodyPropertyOvStage(BodyProperty::eComOrientation, srcTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::setDisableGravitiesOvStage(const TensorDesc* srcTensor, const PxU32* outRecordIdx, const PxU32 numOutputs, uint64_t)
{
    return setBodyPropertyOvStage(BodyProperty::eDisableGravity, srcTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::setDisableSimulationsOvStage(const TensorDesc* srcTensor, const PxU32* outRecordIdx, const PxU32 numOutputs, uint64_t)
{
    return setBodyPropertyOvStage(BodyProperty::eDisableSimulation, srcTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::setStaticFrictionsOvStage(const TensorDesc* srcTensor, const PxU32* outRecordIdx, const PxU32 numOutputs, uint64_t)
{
    return setBodyPropertyOvStage(BodyProperty::eStaticFriction, srcTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::setDynamicFrictionsOvStage(const TensorDesc* srcTensor, const PxU32* outRecordIdx, const PxU32 numOutputs, uint64_t)
{
    return setBodyPropertyOvStage(BodyProperty::eDynamicFriction, srcTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::setRestitutionsOvStage(const TensorDesc* srcTensor, const PxU32* outRecordIdx, const PxU32 numOutputs, uint64_t)
{
    return setBodyPropertyOvStage(BodyProperty::eRestitution, srcTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::setContactOffsetsOvStage(const TensorDesc* srcTensor, const PxU32* outRecordIdx, const PxU32 numOutputs, uint64_t)
{
    return setBodyPropertyOvStage(BodyProperty::eContactOffset, srcTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::setRestOffsetsOvStage(const TensorDesc* srcTensor, const PxU32* outRecordIdx, const PxU32 numOutputs, uint64_t)
{
    return setBodyPropertyOvStage(BodyProperty::eRestOffset, srcTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::getMassesOvStage(const TensorDesc* dstTensor, const PxU32* outRecordIdx,
                                         const PxU32 numOutputs, uint64_t) const
{
    return getBodyPropertyOvStage(BodyProperty::eMass, dstTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::getInvMassesOvStage(const TensorDesc* dstTensor, const PxU32* outRecordIdx,
                                            const PxU32 numOutputs, uint64_t) const
{
    return getBodyPropertyOvStage(BodyProperty::eInvMass, dstTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::getInertiasOvStage(const TensorDesc* dstTensor, const PxU32* outRecordIdx,
                                           const PxU32 numOutputs, uint64_t) const
{
    return getBodyPropertyOvStage(BodyProperty::eInertia, dstTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::getInvInertiasOvStage(const TensorDesc* dstTensor, const PxU32* outRecordIdx,
                                              const PxU32 numOutputs, uint64_t) const
{
    return getBodyPropertyOvStage(BodyProperty::eInvInertia, dstTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::getComPositionsOvStage(const TensorDesc* dstTensor, const PxU32* outRecordIdx,
                                               const PxU32 numOutputs, uint64_t) const
{
    return getBodyPropertyOvStage(BodyProperty::eComPosition, dstTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::getComOrientationsOvStage(const TensorDesc* dstTensor, const PxU32* outRecordIdx,
                                                  const PxU32 numOutputs, uint64_t) const
{
    return getBodyPropertyOvStage(BodyProperty::eComOrientation, dstTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::getDisableGravitiesOvStage(const TensorDesc* dstTensor, const PxU32* outRecordIdx,
                                                   const PxU32 numOutputs, uint64_t) const
{
    return getBodyPropertyOvStage(BodyProperty::eDisableGravity, dstTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::getDisableSimulationsOvStage(const TensorDesc* dstTensor, const PxU32* outRecordIdx,
                                                     const PxU32 numOutputs, uint64_t) const
{
    return getBodyPropertyOvStage(BodyProperty::eDisableSimulation, dstTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::getStaticFrictionsOvStage(const TensorDesc* dstTensor, const PxU32* outRecordIdx,
                                                 const PxU32 numOutputs, uint64_t) const
{
    return getBodyPropertyOvStage(BodyProperty::eStaticFriction, dstTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::getDynamicFrictionsOvStage(const TensorDesc* dstTensor, const PxU32* outRecordIdx,
                                                  const PxU32 numOutputs, uint64_t) const
{
    return getBodyPropertyOvStage(BodyProperty::eDynamicFriction, dstTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::getRestitutionsOvStage(const TensorDesc* dstTensor, const PxU32* outRecordIdx,
                                               const PxU32 numOutputs, uint64_t) const
{
    return getBodyPropertyOvStage(BodyProperty::eRestitution, dstTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::getContactOffsetsOvStage(const TensorDesc* dstTensor, const PxU32* outRecordIdx,
                                                 const PxU32 numOutputs, uint64_t) const
{
    return getBodyPropertyOvStage(BodyProperty::eContactOffset, dstTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::getRestOffsetsOvStage(const TensorDesc* dstTensor, const PxU32* outRecordIdx,
                                              const PxU32 numOutputs, uint64_t) const
{
    return getBodyPropertyOvStage(BodyProperty::eRestOffset, dstTensor, outRecordIdx, numOutputs);
}

bool BaseRigidBodyView::getShapeCountsOvStage(const TensorDesc* dstTensor, const PxU32* outRecordIdx,
                                              const PxU32 numOutputs, uint64_t) const
{
    return getBodyPropertyOvStage(BodyProperty::eShapeCount, dstTensor, outRecordIdx, numOutputs);
}

uint32_t BaseRigidBodyView::maxShapesForRows(const PxU32* const outRecordIdx, const PxU32 numOutputs) const
{
    uint32_t widest = 0;
    for (PxU32 i = 0; i < numOutputs; i++)
    {
        const PxU32 recIdx = outRecordIdx ? outRecordIdx[i] : i;
        if (recIdx < mEntries.size())
            widest = std::max(widest, mEntries[recIdx].numShapes);
    }
    return widest;
}

}
}
}
