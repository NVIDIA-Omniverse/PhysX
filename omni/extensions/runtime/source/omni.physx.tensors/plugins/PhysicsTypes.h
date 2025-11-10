// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Defines.h>

#include <cudamanager/PxCudaContextManager.h>
#include <foundation/PxSimpleTypes.h>
#include <foundation/PxTransform.h>
#include <solver/PxSolverDefs.h> // PxArticulationAxis, PxArticulationMotion

#include <PxArticulationFlag.h>
#include <PxArticulationTendon.h>
#include <PxArticulationTendonData.h>
#include <PxParticleSystemFlag.h>
#include <PxParticleSystem.h>
#include <PxParticleBuffer.h>
#include <PxShape.h>
#include <PxSoftBody.h>
#include <PxSoftBodyFlag.h>
#include <PxFEMSoftBodyMaterial.h>
#include <foundation/PxFlags.h>
#include <PxContact.h>

namespace physx
{
class PxArticulationReducedCoordinate;
class PxArticulationCache;
class PxArticulationLink;
class PxArticulationJointReducedCoordinate;
class PxArticulationFixedTendon;
class PxGpuFixedTendonData;
class PxArticulationSpatialTendon;
class PxGpuSpatialTendonData;
class PxRigidBody;
class PxRigidDynamic;
class PxShape;
class PxScene;
struct PxGpuContactPair;
} // namespace physx

namespace omni
{
namespace physx
{
namespace tensors
{

// how transforms are stored in public tensors - NOTE: it swaps p and q wrt PxTransform
struct TensorTransform
{
    ::physx::PxVec3 p;
    ::physx::PxQuat q;
};

// how velocities are stored in public tensors - NOTE: it swaps linear and angular wrt PhysX direct GPU API
struct TensorVelAcc
{
    ::physx::PxVec3 linear;
    ::physx::PxVec3 angular;
};

// how velocities are reported by PhysX direct GPU API (Cm::UnAlignedSpatialVector)
// TODO: can choose more general names for PhysxVelAcc and TensorTransform such as SO3LieGroupTuple and
// SO3LieAlgebraTuple to reuse the same structure for accelerations as well.
struct PhysxVelAcc
{
    ::physx::PxVec3 angular;
    ::physx::PxVec3 linear;
};

// how force sensor data are reported by PhysX direct GPU API (Cm::UnAlignedSpatialVector)
struct PhysxGpuSpatialForces
{
    ::physx::PxVec3 force;
    ::physx::PxVec3 torque;
};


class PhysxCudaContextGuard
{
public:
    explicit PhysxCudaContextGuard(::physx::PxCudaContextManager* cudaContextManager)
        : mCudaContextManager(cudaContextManager)
    {
        if (mCudaContextManager)
        {
            mCudaContextManager->acquireContext();
        }
    }

    ~PhysxCudaContextGuard()
    {
        if (mCudaContextManager)
        {
            mCudaContextManager->releaseContext();
        }
    }

private:
    ::physx::PxCudaContextManager* mCudaContextManager;
};

struct FreeD6RotationAxesFlag
{
    enum Enum
    {
        eTWIST = 1 << 0, //!< free rotation around X axis
        eSWING1 = 1 << 1, //!< free rotation around Y axis
        eSWING2 = 1 << 2 //!< free rotation around Z axis
    };
};

using FreeD6RotationAxesFlags = ::physx::PxFlags<FreeD6RotationAxesFlag::Enum, ::physx::PxU8>;

} // namespace tensors
} // namespace physx
} // namespace omni
