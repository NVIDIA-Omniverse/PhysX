// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "BaseRigidBodyView.h"
#include "BaseSimulationView.h"

#include "../GlobalsAreBad.h"
#include "../CommonTypes.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>

#include <omni/physics/tensors/TensorUtils.h>

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
        return mEntries[rbIdx].path.GetString().c_str();
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
            const uint8_t* src = static_cast<const uint8_t*>(srcTensor->data) + idx;
            mEntries[idx].body->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, *src);
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

bool BaseRigidBodyView::check() const
{
    bool result = true;

    // printf("~!~!~ Checking rigid body view\n");

    if (!g_physx)
    {
        return false;
    }

    for (auto& entry : mEntries)
    {
        if (entry.type == RigidBodyType::eRigidDynamic)
        {
            void* ptr = g_physx->getPhysXPtr(entry.path, omni::physx::PhysXType::ePTActor);
            if (ptr != entry.body)
            {
                result = false;
            }
        }
        else if (entry.type == RigidBodyType::eArticulationLink)
        {
            void* ptr = g_physx->getPhysXPtr(entry.path, omni::physx::PhysXType::ePTLink);
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
    // printf("~!~!~! Detaching RB view from parent %p\n", mSim);
    mSim = nullptr;
}

}
}
}
