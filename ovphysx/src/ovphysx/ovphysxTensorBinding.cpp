// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "ovphysx/ovphysx.h"
#include "internal/sdk/ovphysxSDK.hpp"
#include "internal/sdk/DLPackConvert.h"
#include "internal/sidecar/ovphysxInternalInterop.h"
#include "AsyncEventManager/AsyncEventManager.hpp"
#include <omni/physx/PhysXRuntime.h>

#include <carb/Framework.h>
#include <carb/logging/Log.h>
#include <omni/physx/IOptionalCuda.h>

#include <omni/physx/IPhysxSimulation.h>

#include <omni/physics/tensors/ISimulationView.h>
#include <omni/physics/tensors/IRigidBodyView.h>
#include <omni/physics/tensors/IArticulationView.h>
#include <omni/physics/tensors/IArticulationMetatype.h>
#include <omni/physics/tensors/IDeformableBodyView.h>
#include <omni/physics/tensors/IDeformableMaterialView.h>

using ovphysx::internal::DLConvertError;
using ovphysx::internal::dlToTensorDesc;
using ovphysx::internal::dlConvertErrorMessage;
using ovphysx::internal::getTensorApi;

#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <vector>
#include <string>

#include "WrenchConversion.h"

namespace
{

static constexpr int kCudaErrorNotInitialized = 3;

static omni::physx::IOptionalCuda* getOptionalCuda()
{
    if (isProcessGpuDisabled())
        return nullptr;
    return omni::physx::runtime::tryGetOptionalCudaInterface();
}

static ovphysx_result_t validateProcessTensorDevice(const DLTensor* tensor, const char* op)
{
    if (isProcessGpuDisabled() && tensor &&
        (tensor->device.device_type == kDLCUDA || tensor->device.device_type == kDLCUDAManaged))
    {
        return set_error(OVPHYSX_API_DEVICE_MISMATCH,
                         std::string(op) + ": CUDA tensor is not allowed in process-wide CPU-only mode");
    }
    return success();
}

class ScopedCudaContextPush
{
public:
    ScopedCudaContextPush(omni::physx::IOptionalCuda* cuda, uintptr_t ctx)
        : mCuda(cuda)
    {
        if (!mCuda || !ctx)
        {
            mOk = false;
            mStatus = kCudaErrorNotInitialized;
            return;
        }
        // Only push if the desired context is not already current (avoids
        // unnecessary CUDA context stack depth changes).
        uintptr_t current = 0;
        if (!mCuda->ctxGetCurrent(&current, nullptr))
        {
            mOk = false;
            mStatus = kCudaErrorNotInitialized;
            return;
        }
        if (current == ctx)
        {
            // Already in the right context -- nothing to push/pop.
            mOk = true;
            mStatus = 0;
            mPushed = false;
            return;
        }
        int st = 0;
        mOk = mCuda->ctxPushCurrent(ctx, &st);
        mStatus = mOk ? 0 : st;
        mPushed = mOk;
    }

    ~ScopedCudaContextPush()
    {
        if (mCuda && mPushed)
        {
            (void)mCuda->ctxPopCurrent(nullptr, nullptr);
        }
    }

    // Non-copyable/movable: the destructor pops the CUDA context stack, so a
    // copy or move would cause a double-pop.
    ScopedCudaContextPush(const ScopedCudaContextPush&) = delete;
    ScopedCudaContextPush& operator=(const ScopedCudaContextPush&) = delete;
    ScopedCudaContextPush(ScopedCudaContextPush&&) = delete;
    ScopedCudaContextPush& operator=(ScopedCudaContextPush&&) = delete;

    bool ok() const { return mOk; }
    int status() const { return mStatus; }

private:
    omni::physx::IOptionalCuda* mCuda = nullptr;
    bool mOk = false;
    bool mPushed = false;
    int mStatus = 0;
};

class ScopedTensorNoMatchLogQuiet
{
public:
    ScopedTensorNoMatchLogQuiet(omni::physics::tensors::ISimulationView* view, bool enabled)
        : mView(view)
    {
        if (!enabled || !mView)
        {
            return;
        }

        mPreviousValue = mView->isNoMatchLoggingQuiet();
        mView->setNoMatchLoggingQuiet(true);
        mActive = true;
    }

    ~ScopedTensorNoMatchLogQuiet()
    {
        if (mActive && mView)
        {
            mView->setNoMatchLoggingQuiet(mPreviousValue);
        }
    }

    ScopedTensorNoMatchLogQuiet(const ScopedTensorNoMatchLogQuiet&) = delete;
    ScopedTensorNoMatchLogQuiet& operator=(const ScopedTensorNoMatchLogQuiet&) = delete;

private:
    omni::physics::tensors::ISimulationView* mView = nullptr;
    bool mPreviousValue = false;
    bool mActive = false;
};

// Three TensorDescs for force/torque/position after AoS->SoA wrench conversion.
struct WrenchSoaDescs
{
    omni::physics::tensors::TensorDesc force;
    omni::physics::tensors::TensorDesc torque;
    omni::physics::tensors::TensorDesc position;
};

// GPU AoS-to-SoA wrench conversion helper.
// Sets up CUDA context and performs the strided 2D copies into a caller-provided SoA buffer.
// Returns an error result on failure; on success returns status==OVPHYSX_API_SUCCESS.
static ovphysx_result_t convertWrenchAoSToSoaGpu(
    omni::physx::IOptionalCuda* cuda,
    uintptr_t dstSoaDev,
    uintptr_t srcAosDev,
    int64_t totalRows,
    uintptr_t cudaCtx,
    const char* errorContext)
{
    if (!cuda || !cuda->cudaAvailable())
        return set_error(OVPHYSX_API_GPU_NOT_AVAILABLE, "CUDA driver not available");

    if (!cudaCtx)
    {
        std::ostringstream oss;
        oss << "missing PhysX CUDA context for " << errorContext;
        return set_error(OVPHYSX_API_ERROR, oss.str());
    }

    ScopedCudaContextPush ctx(cuda, cudaCtx);
    if (!ctx.ok())
    {
        CARB_LOG_ERROR("[TensorBindings] Failed to activate PhysX CUDA context for %s (cuCtxPushCurrent error %d)",
                       errorContext ? errorContext : "(unknown)", ctx.status());
        std::ostringstream oss;
        oss << "failed to activate PhysX CUDA context for " << errorContext
            << " (cuCtxPushCurrent error " << ctx.status() << ")";
        return set_error(OVPHYSX_API_ERROR, oss.str());
    }

    int cuStatus = 0;

    // Convert AoS [totalRows, 9] -> SoA: three contiguous [totalRows, 3] arrays.
    // Each row has 9 floats = 3 components x 3 floats. We extract component `comp`
    // (offset comp*3 floats into each row) into a contiguous [totalRows, 3] block.
    for (int comp = 0; comp < 3; ++comp)
    {
        const uintptr_t srcComp = srcAosDev + static_cast<size_t>(comp) * 3 * sizeof(float);
        const uintptr_t dstComp = dstSoaDev + static_cast<size_t>(comp) * static_cast<size_t>(totalRows) * 3 * sizeof(float);
        if (!cuda->memcpy2DDeviceToDevice(
                dstComp,
                /*dstPitch=*/3 * sizeof(float),
                srcComp,
                /*srcPitch=*/9 * sizeof(float),
                /*widthInBytes=*/3 * sizeof(float),
                /*height=*/static_cast<size_t>(totalRows),
                &cuStatus))
        {
            std::ostringstream oss;
            oss << "failed to convert " << errorContext << " to SoA layout (CUDA status " << cuStatus << ")";
            return set_error(OVPHYSX_API_ERROR, oss.str());
        }
    }

    return success();
}

static ovphysx_result_t ensureWrenchScratchGpu(
    TensorBindingState& binding,
    omni::physx::IOptionalCuda* cuda,
    uintptr_t cudaCtx,
    size_t neededBytes,
    const char* errorContext)
{
    if (!cuda || !cuda->cudaAvailable())
        return set_error(OVPHYSX_API_GPU_NOT_AVAILABLE, "CUDA driver not available");
    if (!cudaCtx)
        return set_error(OVPHYSX_API_ERROR, "missing PhysX CUDA context");

    if (binding.wrenchSoaScratchDev != 0 && binding.wrenchSoaScratchBytes >= neededBytes)
        return success();

    ScopedCudaContextPush ctx(cuda, cudaCtx);
    if (!ctx.ok())
    {
        std::ostringstream oss;
        oss << "failed to activate PhysX CUDA context for " << errorContext
            << " (cuCtxPushCurrent error " << ctx.status() << ")";
        return set_error(OVPHYSX_API_ERROR, oss.str());
    }

    if (binding.wrenchSoaScratchDev)
    {
        (void)cuda->memFree(binding.wrenchSoaScratchDev, nullptr);
        binding.wrenchSoaScratchDev = 0;
        binding.wrenchSoaScratchBytes = 0;
    }

    int st = 0;
    uintptr_t ptr = 0;
    if (!cuda->memAlloc(&ptr, neededBytes, &st))
    {
        std::ostringstream oss;
        oss << "failed to allocate GPU scratch buffer for " << errorContext << " (CUDA status " << st << ")";
        return set_error(OVPHYSX_API_ERROR, oss.str());
    }

    binding.wrenchSoaScratchDev = ptr;
    binding.wrenchSoaScratchBytes = neededBytes;
    return success();
}

// Build force/torque/position TensorDescs from a contiguous SoA buffer (forces|torques|positions).
// 2D (rigid body):        L < 0, totalRows = N          -> three [N, 3] descriptors
// 3D (articulation links): L >= 0, totalRows = N * L     -> three [N, L, 3] descriptors
inline WrenchSoaDescs buildWrenchSoaDescs(
    const omni::physics::tensors::TensorDesc& base,
    void* soaBase, int64_t totalRows, int64_t N, int64_t L = -1)
{
    const size_t compBytes = static_cast<size_t>(totalRows) * 3 * sizeof(float);
    auto* baseBytes = reinterpret_cast<uint8_t*>(soaBase);
    auto makeDesc = [&](int comp) {
        omni::physics::tensors::TensorDesc d = base;
        if (L < 0) { d.numDims = 2; d.dims[0] = N; d.dims[1] = 3; }
        else        { d.numDims = 3; d.dims[0] = N; d.dims[1] = L; d.dims[2] = 3; }
        d.data = baseBytes + static_cast<size_t>(comp) * compBytes;
        return d;
    };
    return { makeDesc(0), makeDesc(1), makeDesc(2) };
}

// dlToTensorDesc, DLConvertError, dlConvertErrorMessage, getTensorApi
// are defined in internal/sdk/DLPackConvert.h and imported via using-declarations above.

void destroyBindingResources(TensorBindingState& b)
{
    // Free scratch GPU buffer (wrench AoS->SoA conversion) before releasing the simulation view.
    if (b.wrenchSoaScratchDev && b.simView)
    {
        auto* cuda = getOptionalCuda();
        const uintptr_t cudaCtx = reinterpret_cast<uintptr_t>(b.simView->getCudaContext());
        if (cuda && cudaCtx)
        {
            ScopedCudaContextPush ctx(cuda, cudaCtx);
            if (ctx.ok())
            {
                (void)cuda->memFree(b.wrenchSoaScratchDev, nullptr);
            }
        }
    }
    // Unconditional reset: even if simView was null or context push failed, avoid dangling pointers.
    b.wrenchSoaScratchDev = 0;
    b.wrenchSoaScratchBytes = 0;

    if (b.rbView) { b.rbView->release(); b.rbView = nullptr; }
    if (b.artiView) { b.artiView->release(); b.artiView = nullptr; }
    if (b.defBodyView) { b.defBodyView->release(); b.defBodyView = nullptr; }
    if (b.defMatView) { b.defMatView->release(); b.defMatView = nullptr; }
    if (b.simView) { b.simView->release(false); b.simView = nullptr; }
}

bool requiresRigidBodyView(ovphysx_tensor_type_t type)
{
    switch (type)
    {
        case OVPHYSX_TENSOR_RIGID_BODY_POSE_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_ACCELERATION_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_FORCE_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_WRENCH_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_MASS_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_INERTIA_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_COM_POSE_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_INV_MASS_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_INV_INERTIA_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL:
        case OVPHYSX_TENSOR_RIGID_BODY_DISABLE_GRAVITY_BOOL:
        case OVPHYSX_TENSOR_RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_CONTACT_OFFSET_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_REST_OFFSET_F32:
            return true;
        default:
            return false;
    }
}

bool requiresArticulationView(ovphysx_tensor_type_t type)
{
    switch (type)
    {
        case OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32:
        case OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_WORLD_F32:
        case OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_LOCAL_F32:
        case OVPHYSX_TENSOR_ARTICULATION_CENTROIDAL_MOMENTUM_F32:
        case OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_LINK_VELOCITY_F32:
        case OVPHYSX_TENSOR_ARTICULATION_LINK_ACCELERATION_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_LINK_WRENCH_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_STIFFNESS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_DAMPING_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_LIMIT_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_VELOCITY_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_FORCE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_ARMATURE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_MODEL_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_TYPE_U8:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_FRICTION_PROPERTIES_F32:
        case OVPHYSX_TENSOR_ARTICULATION_BODY_MASS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL:
        case OVPHYSX_TENSOR_ARTICULATION_BODY_COM_POSE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_BODY_INERTIA_F32:
        case OVPHYSX_TENSOR_ARTICULATION_JACOBIAN_F32:
        case OVPHYSX_TENSOR_ARTICULATION_MASS_MATRIX_F32:
        case OVPHYSX_TENSOR_ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_GRAVITY_FORCE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_LINK_INCOMING_JOINT_FORCE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_PROJECTED_JOINT_FORCE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_BODY_INV_MASS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_BODY_INV_INERTIA_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_STIFFNESS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_DAMPING_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_STIFFNESS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_REST_LENGTH_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_OFFSET_F32:
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_STIFFNESS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_DAMPING_F32:
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_LIMIT_STIFFNESS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_OFFSET_F32:
        case OVPHYSX_TENSOR_ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION_F32:
        case OVPHYSX_TENSOR_ARTICULATION_CONTACT_OFFSET_F32:
        case OVPHYSX_TENSOR_ARTICULATION_REST_OFFSET_F32:
            return true;
        default:
            return false;
    }
}

bool requiresDeformableBodyView(ovphysx_tensor_type_t type)
{
    switch (type)
    {
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_POSITION_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_VELOCITY_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_KINEMATIC_TARGET_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_REST_NODAL_POSITION_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_ELEMENT_INDICES_S32:
        case OVPHYSX_TENSOR_DEFORMABLE_COLLISION_ELEMENT_INDICES_S32:
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_POSITION_F32:
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_VELOCITY_F32:
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_REST_POSITION_F32:
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_ELEMENT_INDICES_S32:
            return true;
        default:
            return false;
    }
}

static bool isSurfaceDeformableBodyType(ovphysx_tensor_type_t type)
{
    switch (type)
    {
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_POSITION_F32:
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_VELOCITY_F32:
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_REST_POSITION_F32:
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_ELEMENT_INDICES_S32:
            return true;
        default:
            return false;
    }
}

bool requiresDeformableMaterialView(ovphysx_tensor_type_t type)
{
    switch (type)
    {
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_DYNAMIC_FRICTION_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_YOUNGS_MODULUS_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_POISSONS_RATIO_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_ELASTICITY_DAMPING_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_STIFFNESS_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_THICKNESS_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_DAMPING_F32:
            return true;
        default:
            return false;
    }
}

static bool isInt32TensorType(ovphysx_tensor_type_t type)
{
    switch (type)
    {
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_ELEMENT_INDICES_S32:
        case OVPHYSX_TENSOR_DEFORMABLE_COLLISION_ELEMENT_INDICES_S32:
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_ELEMENT_INDICES_S32:
            return true;
        default:
            return false;
    }
}

static bool isUint8TensorType(ovphysx_tensor_type_t type)
{
    switch (type)
    {
        case OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL:
        case OVPHYSX_TENSOR_RIGID_BODY_DISABLE_GRAVITY_BOOL:
        case OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_TYPE_U8:
            return true;
        default:
            return false;
    }
}

static DLDataType getBindingDtype(ovphysx_tensor_type_t type)
{
    DLDataType dtype{};
    dtype.lanes = 1;
    if (isInt32TensorType(type))
    {
        dtype.code = kDLInt;
        dtype.bits = 32;
    }
    else if (isUint8TensorType(type))
    {
        dtype.code = kDLUInt;
        dtype.bits = 8;
    }
    else
    {
        dtype.code = kDLFloat;
        dtype.bits = 32;
    }
    return dtype;
}

static const char* getBindingDtypeName(ovphysx_tensor_type_t type)
{
    if (isInt32TensorType(type)) return "int32";
    if (isUint8TensorType(type)) return "uint8";
    return "float32";
}

static bool dtypeMatches(const DLDataType& actual, const DLDataType& expected)
{
    return actual.code == expected.code && actual.bits == expected.bits && actual.lanes == expected.lanes;
}

bool isWriteOnlyTensor(ovphysx_tensor_type_t type)
{
    switch (type)
    {
        case OVPHYSX_TENSOR_RIGID_BODY_FORCE_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_WRENCH_F32:
        case OVPHYSX_TENSOR_ARTICULATION_LINK_WRENCH_F32:
            return true;
        default:
            return false;
    }
}

// Write a single fixed tendon property via read-back + batch setter.
// The underlying IArticulationView only exposes a batch setter (setFixedTendonProperties)
// that requires all 6 property tensors at once. This helper reads back the other 5
// properties, overlays the property being written, and calls the batch setter.
static bool writeFixedTendonProperty(
    const TensorBindingState& binding,
    const omni::physics::tensors::TensorDesc& src,
    const omni::physics::tensors::TensorDesc* idxPtr,
    const omni::physics::tensors::TensorDesc* maskPtr)
{
    using namespace omni::physics::tensors;
    auto* artiView = binding.artiView;

    const int N = static_cast<int>(artiView->getCount());
    const int T = static_cast<int>(artiView->getMaxFixedTendons());
    if (T == 0) return true;

    const bool isGpu = (src.device >= 0);
    const size_t szNT = static_cast<size_t>(N) * T;
    const size_t szNT2 = szNT * 2;

    auto makeTD = [&](void* data, int nd, int d0, int d1, int d2 = 0) -> TensorDesc {
        TensorDesc td{};
        td.device = src.device;
        td.dtype = TensorDataType::eFloat32;
        td.numDims = nd;
        td.dims[0] = d0;
        td.dims[1] = d1;
        if (nd >= 3) td.dims[2] = d2;
        td.data = data;
        return td;
    };

    auto selectTarget = [&](float* stiff, float* damp, float* limStiff, float* lim, float* rest, float* off,
                            float*& target, size_t& copyFloats) {
        switch (binding.tensorType)
        {
            case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_STIFFNESS_F32:       target = stiff;    copyFloats = szNT;  break;
            case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_DAMPING_F32:         target = damp;     copyFloats = szNT;  break;
            case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_STIFFNESS_F32: target = limStiff; copyFloats = szNT;  break;
            case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_F32:           target = lim;      copyFloats = szNT2; break;
            case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_REST_LENGTH_F32:     target = rest;     copyFloats = szNT;  break;
            case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_OFFSET_F32:          target = off;      copyFloats = szNT;  break;
            default: target = nullptr; copyFloats = 0; break;
        }
    };

    auto readBackAll = [&](TensorDesc& stiffTD, TensorDesc& dampTD, TensorDesc& limStiffTD,
                           TensorDesc& limTD, TensorDesc& restTD, TensorDesc& offTD) {
        artiView->getFixedTendonStiffnesses(&stiffTD);
        artiView->getFixedTendonDampings(&dampTD);
        artiView->getFixedTendonLimitStiffnesses(&limStiffTD);
        artiView->getFixedTendonLimits(&limTD);
        artiView->getFixedTendonfixedSpringRestLengths(&restTD);
        artiView->getFixedTendonOffsets(&offTD);
    };

    auto callSetter = [&](TensorDesc& stiffTD, TensorDesc& dampTD, TensorDesc& limStiffTD,
                          TensorDesc& limTD, TensorDesc& restTD, TensorDesc& offTD) -> bool {
        if (maskPtr)
            return artiView->setFixedTendonPropertiesMasked(&stiffTD, &dampTD, &limStiffTD, &limTD, &restTD, &offTD, maskPtr);
        else
            return artiView->setFixedTendonProperties(&stiffTD, &dampTD, &limStiffTD, &limTD, &restTD, &offTD, idxPtr);
    };

    if (!isGpu)
    {
        std::vector<float> stiffBuf(szNT), dampBuf(szNT), limStiffBuf(szNT);
        std::vector<float> limBuf(szNT2), restBuf(szNT), offBuf(szNT);

        TensorDesc stiffTD = makeTD(stiffBuf.data(), 2, N, T);
        TensorDesc dampTD  = makeTD(dampBuf.data(),  2, N, T);
        TensorDesc limStiffTD = makeTD(limStiffBuf.data(), 2, N, T);
        TensorDesc limTD   = makeTD(limBuf.data(),   3, N, T, 2);
        TensorDesc restTD  = makeTD(restBuf.data(),  2, N, T);
        TensorDesc offTD   = makeTD(offBuf.data(),   2, N, T);

        readBackAll(stiffTD, dampTD, limStiffTD, limTD, restTD, offTD);

        float* target = nullptr;
        size_t copyFloats = 0;
        selectTarget(stiffBuf.data(), dampBuf.data(), limStiffBuf.data(),
                     limBuf.data(), restBuf.data(), offBuf.data(), target, copyFloats);
        if (!target) return false;

        std::memcpy(target, src.data, copyFloats * sizeof(float));
        return callSetter(stiffTD, dampTD, limStiffTD, limTD, restTD, offTD);
    }
    else
    {
        auto* cuda = getOptionalCuda();
        if (!cuda) return false;

        uintptr_t cudaCtx = binding.simView ? reinterpret_cast<uintptr_t>(binding.simView->getCudaContext()) : 0;
        ScopedCudaContextPush ctxPush(cuda, cudaCtx);
        if (!ctxPush.ok()) return false;

        const size_t totalBytes = (szNT * 5 + szNT2) * sizeof(float);
        uintptr_t gpuBase = 0;
        int cuStatus = 0;
        if (!cuda->memAlloc(&gpuBase, totalBytes, &cuStatus))
            return false;

        float* base = reinterpret_cast<float*>(gpuBase);
        float* stiffPtr   = base;
        float* dampPtr    = stiffPtr + szNT;
        float* limStiffPtr = dampPtr + szNT;
        float* limPtr      = limStiffPtr + szNT;
        float* restPtr     = limPtr + szNT2;
        float* offPtr      = restPtr + szNT;

        TensorDesc stiffTD = makeTD(stiffPtr, 2, N, T);
        TensorDesc dampTD  = makeTD(dampPtr,  2, N, T);
        TensorDesc limStiffTD = makeTD(limStiffPtr, 2, N, T);
        TensorDesc limTD   = makeTD(limPtr,   3, N, T, 2);
        TensorDesc restTD  = makeTD(restPtr,  2, N, T);
        TensorDesc offTD   = makeTD(offPtr,   2, N, T);

        readBackAll(stiffTD, dampTD, limStiffTD, limTD, restTD, offTD);

        float* target = nullptr;
        size_t copyFloats = 0;
        selectTarget(stiffPtr, dampPtr, limStiffPtr, limPtr, restPtr, offPtr, target, copyFloats);
        if (!target) { cuda->memFree(gpuBase, &cuStatus); return false; }

        const size_t copyBytes = copyFloats * sizeof(float);
        cuda->memcpy2DDeviceToDevice(
            reinterpret_cast<uintptr_t>(target), copyBytes,
            reinterpret_cast<uintptr_t>(src.data), copyBytes,
            copyBytes, 1, &cuStatus);

        bool result = callSetter(stiffTD, dampTD, limStiffTD, limTD, restTD, offTD);
        cuda->memFree(gpuBase, &cuStatus);
        return result;
    }
}

// Write a single spatial tendon property via read-back + batch setter.
// Spatial tendons have 4 properties (vs 6 for fixed): stiffness, damping,
// limit_stiffness, offset. The batch setter setSpatialTendonProperties()
// requires all 4 at once, so we read back the other 3 and overlay.
static bool writeSpatialTendonProperty(
    const TensorBindingState& binding,
    const omni::physics::tensors::TensorDesc& src,
    const omni::physics::tensors::TensorDesc* idxPtr,
    const omni::physics::tensors::TensorDesc* maskPtr)
{
    using namespace omni::physics::tensors;
    auto* artiView = binding.artiView;

    const int N = static_cast<int>(artiView->getCount());
    const int T = static_cast<int>(artiView->getMaxSpatialTendons());
    if (T == 0) return true;

    const bool isGpu = (src.device >= 0);
    const size_t szNT = static_cast<size_t>(N) * T;

    auto makeTD = [&](void* data, int d0, int d1) -> TensorDesc {
        TensorDesc td{};
        td.device = src.device;
        td.dtype = TensorDataType::eFloat32;
        td.numDims = 2;
        td.dims[0] = d0;
        td.dims[1] = d1;
        td.data = data;
        return td;
    };

    auto selectTarget = [&](float* stiff, float* damp, float* limStiff, float* off,
                            float*& target, size_t& copyFloats) {
        switch (binding.tensorType)
        {
            case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_STIFFNESS_F32:       target = stiff;    copyFloats = szNT; break;
            case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_DAMPING_F32:         target = damp;     copyFloats = szNT; break;
            case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_LIMIT_STIFFNESS_F32: target = limStiff; copyFloats = szNT; break;
            case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_OFFSET_F32:          target = off;      copyFloats = szNT; break;
            default: target = nullptr; copyFloats = 0; break;
        }
    };

    auto readBackAll = [&](TensorDesc& stiffTD, TensorDesc& dampTD,
                           TensorDesc& limStiffTD, TensorDesc& offTD) {
        artiView->getSpatialTendonStiffnesses(&stiffTD);
        artiView->getSpatialTendonDampings(&dampTD);
        artiView->getSpatialTendonLimitStiffnesses(&limStiffTD);
        artiView->getSpatialTendonOffsets(&offTD);
    };

    auto callSetter = [&](TensorDesc& stiffTD, TensorDesc& dampTD,
                          TensorDesc& limStiffTD, TensorDesc& offTD) -> bool {
        if (maskPtr)
            return artiView->setSpatialTendonPropertiesMasked(&stiffTD, &dampTD, &limStiffTD, &offTD, maskPtr);
        else
            return artiView->setSpatialTendonProperties(&stiffTD, &dampTD, &limStiffTD, &offTD, idxPtr);
    };

    if (!isGpu)
    {
        std::vector<float> stiffBuf(szNT), dampBuf(szNT), limStiffBuf(szNT), offBuf(szNT);

        TensorDesc stiffTD    = makeTD(stiffBuf.data(), N, T);
        TensorDesc dampTD     = makeTD(dampBuf.data(), N, T);
        TensorDesc limStiffTD = makeTD(limStiffBuf.data(), N, T);
        TensorDesc offTD      = makeTD(offBuf.data(), N, T);

        readBackAll(stiffTD, dampTD, limStiffTD, offTD);

        float* target = nullptr;
        size_t copyFloats = 0;
        selectTarget(stiffBuf.data(), dampBuf.data(), limStiffBuf.data(),
                     offBuf.data(), target, copyFloats);
        if (!target) return false;

        std::memcpy(target, src.data, copyFloats * sizeof(float));
        return callSetter(stiffTD, dampTD, limStiffTD, offTD);
    }
    else
    {
        auto* cuda = getOptionalCuda();
        if (!cuda) return false;

        uintptr_t cudaCtx = binding.simView ? reinterpret_cast<uintptr_t>(binding.simView->getCudaContext()) : 0;
        ScopedCudaContextPush ctxPush(cuda, cudaCtx);
        if (!ctxPush.ok()) return false;

        const size_t totalBytes = szNT * 4 * sizeof(float);
        uintptr_t gpuBase = 0;
        int cuStatus = 0;
        if (!cuda->memAlloc(&gpuBase, totalBytes, &cuStatus))
            return false;

        float* base = reinterpret_cast<float*>(gpuBase);
        float* stiffPtr    = base;
        float* dampPtr     = stiffPtr + szNT;
        float* limStiffPtr = dampPtr + szNT;
        float* offPtr      = limStiffPtr + szNT;

        TensorDesc stiffTD    = makeTD(stiffPtr, N, T);
        TensorDesc dampTD     = makeTD(dampPtr, N, T);
        TensorDesc limStiffTD = makeTD(limStiffPtr, N, T);
        TensorDesc offTD      = makeTD(offPtr, N, T);

        readBackAll(stiffTD, dampTD, limStiffTD, offTD);

        float* target = nullptr;
        size_t copyFloats = 0;
        selectTarget(stiffPtr, dampPtr, limStiffPtr, offPtr, target, copyFloats);
        if (!target) { cuda->memFree(gpuBase, &cuStatus); return false; }

        const size_t copyBytes = copyFloats * sizeof(float);
        cuda->memcpy2DDeviceToDevice(
            reinterpret_cast<uintptr_t>(target), copyBytes,
            reinterpret_cast<uintptr_t>(src.data), copyBytes,
            copyBytes, 1, &cuStatus);

        bool result = callSetter(stiffTD, dampTD, limStiffTD, offTD);
        cuda->memFree(gpuBase, &cuStatus);
        return result;
    }
}

bool getBindingSpec(const TensorBindingState& binding, int32_t& ndim, int64_t shape[4])
{
    shape[0] = shape[1] = shape[2] = shape[3] = 0;

    switch (binding.tensorType)
    {
        // Rigid body tensors [N, C]
        case OVPHYSX_TENSOR_RIGID_BODY_POSE_F32:
            ndim = 2;
            shape[0] = binding.rbView ? binding.rbView->getCount() : 0;
            shape[1] = 7;
            return true;

        case OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32:
            ndim = 2;
            shape[0] = binding.rbView ? binding.rbView->getCount() : 0;
            shape[1] = 6;
            return true;

        case OVPHYSX_TENSOR_RIGID_BODY_ACCELERATION_F32:
            ndim = 2;
            shape[0] = binding.rbView ? binding.rbView->getCount() : 0;
            shape[1] = 6;
            return true;

        // Articulation root tensors [N, C]
        case OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32:
            ndim = 2;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = 7;
            return true;

        case OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32:
            ndim = 2;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = 6;
            return true;

        case OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_WORLD_F32:
        case OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_LOCAL_F32:
            ndim = 2;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = 3;
            return true;

        case OVPHYSX_TENSOR_ARTICULATION_CENTROIDAL_MOMENTUM_F32:
            // [N, 6, D+7]: 6 spatial-momentum rows of (D+6) matrix cols + 1 bias col.
            // Matches omni.physx.tensors CpuArticulationView::getArticulationCentroidalMomentum
            // layout so callers can reshape uniformly across backends.
            ndim = 3;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = 6;
            shape[2] = binding.artiView ? (binding.artiView->getMaxDofs() + 7) : 0;
            return true;

        // Articulation link tensors [N, L, C] - 3D!
        case OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32:
            ndim = 3;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxLinks() : 0;
            shape[2] = 7;
            return true;

        case OVPHYSX_TENSOR_ARTICULATION_LINK_VELOCITY_F32:
            ndim = 3;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxLinks() : 0;
            shape[2] = 6;
            return true;

        case OVPHYSX_TENSOR_ARTICULATION_LINK_ACCELERATION_F32:
            ndim = 3;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxLinks() : 0;
            shape[2] = 6;
            return true;

        // Articulation DOF tensors [N, D]
        case OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32:
            ndim = 2;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxDofs() : 0;
            return true;

        // External forces - rigid body [N, 3]
        case OVPHYSX_TENSOR_RIGID_BODY_FORCE_F32:
            ndim = 2;
            shape[0] = binding.rbView ? binding.rbView->getCount() : 0;
            shape[1] = 3;
            return true;

        // External wrenches - rigid body [N, 9] = force(3) + torque(3) + position(3)
        case OVPHYSX_TENSOR_RIGID_BODY_WRENCH_F32:
            ndim = 2;
            shape[0] = binding.rbView ? binding.rbView->getCount() : 0;
            shape[1] = 9;
            return true;

        // External wrenches - articulation links [N, L, 9]
        case OVPHYSX_TENSOR_ARTICULATION_LINK_WRENCH_F32:
            ndim = 3;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxLinks() : 0;
            shape[2] = 9;
            return true;

        // DOF properties [N, D]
        case OVPHYSX_TENSOR_ARTICULATION_DOF_STIFFNESS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_DAMPING_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_VELOCITY_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_FORCE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_ARMATURE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_TYPE_U8:
            ndim = 2;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxDofs() : 0;
            return true;

        // DOF limits [N, D, 2]
        case OVPHYSX_TENSOR_ARTICULATION_DOF_LIMIT_F32:
            ndim = 3;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxDofs() : 0;
            shape[2] = 2;
            return true;

        // DOF property triples [N, D, 3]:
        //   FRICTION_PROPERTIES -- (static, dynamic, viscous)
        //   DRIVE_MODEL         -- (speedEffortGradient, maxActuatorVelocity, velocityDependentResistance)
        case OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_MODEL_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_FRICTION_PROPERTIES_F32:
            ndim = 3;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxDofs() : 0;
            shape[2] = 3;
            return true;

        // Body mass [N, L]
        case OVPHYSX_TENSOR_ARTICULATION_BODY_MASS_F32:
            ndim = 2;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxLinks() : 0;
            return true;

        // Body disable-gravity [N, L]
        case OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL:
            ndim = 2;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxLinks() : 0;
            return true;

        // Body COM [N, L, 7] (position + quaternion)
        case OVPHYSX_TENSOR_ARTICULATION_BODY_COM_POSE_F32:
            ndim = 3;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxLinks() : 0;
            shape[2] = 7;
            return true;

        // Body inertia [N, L, 9] (full 3x3 inertia tensor, row-major)
        case OVPHYSX_TENSOR_ARTICULATION_BODY_INERTIA_F32:
            ndim = 3;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxLinks() : 0;
            shape[2] = 9;
            return true;

        // Jacobian [N, R, C] where (R,C) from getJacobianShape()
        case OVPHYSX_TENSOR_ARTICULATION_JACOBIAN_F32:
        {
            ndim = 3;
            if (!binding.artiView)
            {
                shape[0] = shape[1] = shape[2] = 0;
                return true;
            }
            uint32_t rows = 0, cols = 0;
            if (!binding.artiView->getJacobianShape(&rows, &cols)) return false;
            shape[0] = binding.artiView->getCount();
            shape[1] = rows;
            shape[2] = cols;
            return true;
        }

        // Mass matrix [N, M, M] where M from getGeneralizedMassMatrixShape()
        case OVPHYSX_TENSOR_ARTICULATION_MASS_MATRIX_F32:
        {
            ndim = 3;
            if (!binding.artiView)
            {
                shape[0] = shape[1] = shape[2] = 0;
                return true;
            }
            uint32_t rows = 0, cols = 0;
            if (!binding.artiView->getGeneralizedMassMatrixShape(&rows, &cols)) return false;
            shape[0] = binding.artiView->getCount();
            shape[1] = rows;
            shape[2] = cols;
            return true;
        }

        // Coriolis/centrifugal [N, M]
        case OVPHYSX_TENSOR_ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE_F32:
        {
            ndim = 2;
            if (!binding.artiView)
            {
                shape[0] = shape[1] = 0;
                return true;
            }
            uint32_t rows = 0, cols = 0;
            if (!binding.artiView->getGeneralizedMassMatrixShape(&rows, &cols)) return false;
            shape[0] = binding.artiView->getCount();
            shape[1] = rows;
            return true;
        }

        // Gravity compensation [N, M]
        case OVPHYSX_TENSOR_ARTICULATION_GRAVITY_FORCE_F32:
        {
            ndim = 2;
            if (!binding.artiView)
            {
                shape[0] = shape[1] = 0;
                return true;
            }
            uint32_t rows = 0, cols = 0;
            if (!binding.artiView->getGeneralizedMassMatrixShape(&rows, &cols)) return false;
            shape[0] = binding.artiView->getCount();
            shape[1] = rows;
            return true;
        }

        // Link incoming joint force [N, L, 6]
        case OVPHYSX_TENSOR_ARTICULATION_LINK_INCOMING_JOINT_FORCE_F32:
            ndim = 3;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxLinks() : 0;
            shape[2] = 6;
            return true;

        // DOF projected joint forces [N, D] (read-only)
        case OVPHYSX_TENSOR_ARTICULATION_DOF_PROJECTED_JOINT_FORCE_F32:
            ndim = 2;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxDofs() : 0;
            return true;

        // Standalone rigid body mass [N]
        case OVPHYSX_TENSOR_RIGID_BODY_MASS_F32:
            ndim = 1;
            shape[0] = binding.rbView ? binding.rbView->getCount() : 0;
            return true;

        // Standalone rigid body inertia [N, 9]
        case OVPHYSX_TENSOR_RIGID_BODY_INERTIA_F32:
            ndim = 2;
            shape[0] = binding.rbView ? binding.rbView->getCount() : 0;
            shape[1] = 9;
            return true;

        // Standalone rigid body COM [N, 7]
        case OVPHYSX_TENSOR_RIGID_BODY_COM_POSE_F32:
            ndim = 2;
            shape[0] = binding.rbView ? binding.rbView->getCount() : 0;
            shape[1] = 7;
            return true;

        // Standalone rigid body inverse mass [N] (read-only)
        case OVPHYSX_TENSOR_RIGID_BODY_INV_MASS_F32:
            ndim = 1;
            shape[0] = binding.rbView ? binding.rbView->getCount() : 0;
            return true;

        // Standalone rigid body inverse inertia [N, 9] (read-only)
        case OVPHYSX_TENSOR_RIGID_BODY_INV_INERTIA_F32:
            ndim = 2;
            shape[0] = binding.rbView ? binding.rbView->getCount() : 0;
            shape[1] = 9;
            return true;

        // Standalone rigid body disable-simulation / disable-gravity flags [N] (runtime read/write)
        case OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL:
        case OVPHYSX_TENSOR_RIGID_BODY_DISABLE_GRAVITY_BOOL:
            ndim = 1;
            shape[0] = binding.rbView ? binding.rbView->getCount() : 0;
            return true;

        // Articulation body inverse mass [N, L] (read-only)
        case OVPHYSX_TENSOR_ARTICULATION_BODY_INV_MASS_F32:
            ndim = 2;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxLinks() : 0;
            return true;

        // Articulation body inverse inertia [N, L, 9] (read-only)
        case OVPHYSX_TENSOR_ARTICULATION_BODY_INV_INERTIA_F32:
            ndim = 3;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxLinks() : 0;
            shape[2] = 9;
            return true;

        // Fixed tendon properties [N, T]
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_STIFFNESS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_DAMPING_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_STIFFNESS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_REST_LENGTH_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_OFFSET_F32:
            ndim = 2;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxFixedTendons() : 0;
            return true;

        // Fixed tendon limits [N, T, 2]
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_F32:
            ndim = 3;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxFixedTendons() : 0;
            shape[2] = 2;
            return true;

        // Spatial tendon properties [N, T]
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_STIFFNESS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_DAMPING_F32:
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_LIMIT_STIFFNESS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_OFFSET_F32:
            ndim = 2;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxSpatialTendons() : 0;
            return true;

        // Rigid body shape-level: material properties [N, S, 3]
        case OVPHYSX_TENSOR_RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION_F32:
            ndim = 3;
            shape[0] = binding.rbView ? binding.rbView->getCount() : 0;
            shape[1] = binding.rbView ? binding.rbView->getMaxShapes() : 0;
            shape[2] = 3;
            return true;

        // Rigid body shape-level: contact/rest offsets [N, S]
        case OVPHYSX_TENSOR_RIGID_BODY_CONTACT_OFFSET_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_REST_OFFSET_F32:
            ndim = 2;
            shape[0] = binding.rbView ? binding.rbView->getCount() : 0;
            shape[1] = binding.rbView ? binding.rbView->getMaxShapes() : 0;
            return true;

        // Articulation shape-level: material properties [N, S, 3]
        case OVPHYSX_TENSOR_ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION_F32:
            ndim = 3;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxShapes() : 0;
            shape[2] = 3;
            return true;

        // Articulation shape-level: contact/rest offsets [N, S]
        case OVPHYSX_TENSOR_ARTICULATION_CONTACT_OFFSET_F32:
        case OVPHYSX_TENSOR_ARTICULATION_REST_OFFSET_F32:
            ndim = 2;
            shape[0] = binding.artiView ? binding.artiView->getCount() : 0;
            shape[1] = binding.artiView ? binding.artiView->getMaxShapes() : 0;
            return true;

        // Volume deformable simulation mesh nodal state [N, V, C]
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_POSITION_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_VELOCITY_F32:
            ndim = 3;
            shape[0] = binding.defBodyView ? binding.defBodyView->getCount() : 0;
            shape[1] = binding.defBodyView ? binding.defBodyView->getMaxSimulationNodesPerBody() : 0;
            shape[2] = 3;
            return true;

        // Volume deformable simulation mesh kinematic targets [N, V, 4]
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_KINEMATIC_TARGET_F32:
            ndim = 3;
            shape[0] = binding.defBodyView ? binding.defBodyView->getCount() : 0;
            shape[1] = binding.defBodyView ? binding.defBodyView->getMaxSimulationNodesPerBody() : 0;
            shape[2] = 4;
            return true;

        // Volume deformable rest mesh nodal positions [N, R, 3]
        case OVPHYSX_TENSOR_DEFORMABLE_REST_NODAL_POSITION_F32:
            ndim = 3;
            shape[0] = binding.defBodyView ? binding.defBodyView->getCount() : 0;
            shape[1] = binding.defBodyView ? binding.defBodyView->getMaxRestNodesPerBody() : 0;
            shape[2] = 3;
            return true;

        // Volume deformable simulation mesh element indices [N, E, K] K=4 tetmesh
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_ELEMENT_INDICES_S32:
            ndim = 3;
            shape[0] = binding.defBodyView ? binding.defBodyView->getCount() : 0;
            shape[1] = binding.defBodyView ? binding.defBodyView->getMaxSimulationElementsPerBody() : 0;
            shape[2] = binding.defBodyView ? binding.defBodyView->getNumNodesPerElement() : 0;
            return true;

        // Volume deformable collision element indices [N, F, 4]
        // K is hardcoded to 4 to match GpuVolumeDeformableBodyView::getCollisionElementIndices
        // which calls fetchData(..., mMaxCollElementsPerBody, 4u, ...).
        // getNumNodesPerElement() describes the simulation mesh and must not be used here.
        case OVPHYSX_TENSOR_DEFORMABLE_COLLISION_ELEMENT_INDICES_S32:
            ndim = 3;
            shape[0] = binding.defBodyView ? binding.defBodyView->getCount() : 0;
            shape[1] = binding.defBodyView ? binding.defBodyView->getMaxCollisionElementsPerBody() : 0;
            shape[2] = 4;
            return true;

        // Surface deformable simulation mesh nodal state [N, V, C]
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_POSITION_F32:
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_VELOCITY_F32:
            ndim = 3;
            shape[0] = binding.defBodyView ? binding.defBodyView->getCount() : 0;
            shape[1] = binding.defBodyView ? binding.defBodyView->getMaxSimulationNodesPerBody() : 0;
            shape[2] = 3;
            return true;

        // Surface deformable rest mesh nodal positions [N, R, 3]
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_REST_POSITION_F32:
            ndim = 3;
            shape[0] = binding.defBodyView ? binding.defBodyView->getCount() : 0;
            shape[1] = binding.defBodyView ? binding.defBodyView->getMaxRestNodesPerBody() : 0;
            shape[2] = 3;
            return true;

        // Surface deformable simulation mesh element indices [N, E, 3] K=3 trimesh
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_ELEMENT_INDICES_S32:
            ndim = 3;
            shape[0] = binding.defBodyView ? binding.defBodyView->getCount() : 0;
            shape[1] = binding.defBodyView ? binding.defBodyView->getMaxSimulationElementsPerBody() : 0;
            shape[2] = binding.defBodyView ? binding.defBodyView->getNumNodesPerElement() : 0;
            return true;

        // Deformable material properties [M]
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_DYNAMIC_FRICTION_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_YOUNGS_MODULUS_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_POISSONS_RATIO_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_ELASTICITY_DAMPING_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_STIFFNESS_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_THICKNESS_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_DAMPING_F32:
            ndim = 1;
            shape[0] = binding.defMatView ? binding.defMatView->getCount() : 0;
            return true;

        default:
            return false;
    }
}

// True when any binding shape dim is zero, i.e. the binding describes a tensor
// with no elements to read or write. Examples that hit this path:
//   - bindingShape = (0, ...)           : zero matched prims
//   - bindingShape = (N, 0)             : N articulations, but max_dofs == 0
//                                          (D6-with-LimitAPI articulation config etc.)
//   - bindingShape = (N, M, 0)          : N x M with zero inner length
// All three should be treated as no-op success rather than rejected downstream.
static bool bindingHasZeroElements(int32_t ndim, const int64_t shape[4])
{
    for (int32_t i = 0; i < ndim; ++i)
    {
        if (shape[i] == 0)
            return true;
    }
    return false;
}

// Returns the expected DLDataType for a tensor binding type. Most bindings
// are float32; uint8/bool disable flags use kDLUInt/8 (see isUint8TensorType).
static void getExpectedDtype(ovphysx_tensor_type_t type, uint8_t& code, uint8_t& bits)
{
    if (isUint8TensorType(type))
    {
        code = kDLUInt;
        bits = 8;
    }
    else
    {
        code = kDLFloat;
        bits = 32;
    }
}

ovphysx_result_t validateTensorShape(const DLTensor* tensor, const TensorBindingState& binding, const char* op)
{
    if (!tensor || !tensor->shape)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "tensor has null shape");

    int32_t expectedNdim;
    int64_t expectedShape[4];
    if (!getBindingSpec(binding, expectedNdim, expectedShape))
        return set_error(OVPHYSX_API_ERROR, "unsupported tensor type");

    // Validate dtype
    const DLDataType expectedDtype = getBindingDtype(binding.tensorType);
    if (!dtypeMatches(tensor->dtype, expectedDtype))
    {
        std::ostringstream oss;
        oss << op << ": expected " << getBindingDtypeName(binding.tensorType) << " tensor";
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, oss.str());
    }

    // Validate ndim
    if (tensor->ndim != expectedNdim)
    {
        std::ostringstream oss;
        oss << op << ": expected " << expectedNdim << "D tensor, got " << tensor->ndim << "D";
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, oss.str());
    }

    // Validate shape
    for (int i = 0; i < expectedNdim; i++)
    {
        if (tensor->shape[i] != expectedShape[i])
        {
            std::ostringstream oss;
            oss << op << ": shape mismatch at dim " << i << ", expected " << expectedShape[i] << ", got " << tensor->shape[i];
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, oss.str());
        }
    }

    return success();
}

// PhysX TensorAPI stores DOF/body/shape *property* tensors on the host (CPU) even
// when the simulation runs on GPU. The underlying BaseArticulationView and
// BaseRigidBodyView getters/setters hardcode checkTensorDevice(tensor, -1, ...)
// for these types. State tensors, dynamics queries, wrenches, and tendons live
// on the simulation device.
static bool isCpuOnlyTensorType(ovphysx_tensor_type_t t)
{
    switch (t)
    {
        // Standalone rigid body properties (Base class, always CPU)
        case OVPHYSX_TENSOR_RIGID_BODY_MASS_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_INERTIA_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_COM_POSE_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_INV_MASS_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_INV_INERTIA_F32:
        // Articulation DOF properties (Base class, always CPU)
        case OVPHYSX_TENSOR_ARTICULATION_DOF_STIFFNESS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_DAMPING_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_LIMIT_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_VELOCITY_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_FORCE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_ARMATURE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_MODEL_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_FRICTION_PROPERTIES_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_TYPE_U8:
        // Articulation body properties (Base class, always CPU)
        case OVPHYSX_TENSOR_ARTICULATION_BODY_MASS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_BODY_COM_POSE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_BODY_INERTIA_F32:
        case OVPHYSX_TENSOR_ARTICULATION_BODY_INV_MASS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_BODY_INV_INERTIA_F32:
        // Shape properties (Base class, always CPU)
        case OVPHYSX_TENSOR_RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_CONTACT_OFFSET_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_REST_OFFSET_F32:
        case OVPHYSX_TENSOR_ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION_F32:
        case OVPHYSX_TENSOR_ARTICULATION_CONTACT_OFFSET_F32:
        case OVPHYSX_TENSOR_ARTICULATION_REST_OFFSET_F32:
        // Disable-gravity flags (PxActorFlag toggled via CPU-only PhysX API; the
        // DirectGPU body-sim refresh is a separate wakeUp in the GPU view, not a
        // buffer transfer). Callers pass CPU buffers; no hidden host<->device copy.
        case OVPHYSX_TENSOR_RIGID_BODY_DISABLE_GRAVITY_BOOL:
        case OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL:
        // Deformable material properties (Base class, always CPU)
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_DYNAMIC_FRICTION_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_YOUNGS_MODULUS_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_POISSONS_RATIO_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_ELASTICITY_DAMPING_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_STIFFNESS_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_THICKNESS_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_DAMPING_F32:
            return true;
        default:
            return false;
    }
}

// Validate that tensor device matches binding's expected device.
// For CPU-only property types the expected device is always CPU (-1),
// regardless of simulation device. For all other types the expected
// device comes from the simulation view.
ovphysx_result_t validateTensorDevice(const DLTensor* tensor, const TensorBindingState& binding, const char* op)
{
    if (!tensor)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "tensor is NULL");

    int expectedDevice = -1;  // Default to CPU
    if (isCpuOnlyTensorType(binding.tensorType))
    {
        expectedDevice = -1;
    }
    else if (binding.simView)
    {
        expectedDevice = binding.simView->getDeviceOrdinal();
    }
    
    // Map DLPack device type to TensorAPI convention (-1 = CPU, >=0 = GPU ordinal)
    int tensorDevice;
    switch (tensor->device.device_type)
    {
        case kDLCPU:
        case kDLCUDAHost:  // Pinned host memory maps to CPU
            tensorDevice = -1;
            break;
        case kDLCUDA:
        case kDLCUDAManaged:  // CUDA and unified memory map to GPU
            tensorDevice = tensor->device.device_id;
            break;
        default:
            {
                std::ostringstream oss;
                oss << op << ": unsupported tensor device type " << static_cast<int>(tensor->device.device_type);
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, oss.str());
            }
    }
    
    // Check device match
    if (tensorDevice != expectedDevice)
    {
        std::ostringstream oss;
        oss << op << ": device mismatch: "
            << "binding expects " << (expectedDevice < 0 ? "CPU" : "GPU")
            << " (device=" << expectedDevice << "), "
            << "tensor is " << (tensorDevice < 0 ? "CPU" : "GPU")
            << " (device=" << tensorDevice << ")";
        return set_error(OVPHYSX_API_DEVICE_MISMATCH, oss.str());
    }

    return success();
}

// Cross-device staging buffer used by both ovphysx_read_tensor_binding and
// ovphysx_write_tensor_binding so callers can pass GPU tensors against CPU
// bindings (or vice versa) without doing the host<->device copy themselves.
//
// On same-device input, no staging is allocated; the caller's buffer is
// consumed directly.
//
// On (binding GPU[X], tensor CPU): GPU staging on the binding's device.
// On (binding CPU, tensor GPU[X]): CPU staging on host.
// On (binding GPU[X], tensor GPU[Y], X != Y): rejected as DEVICE_MISMATCH;
// cross-GPU P2P is out of scope.
//
// TensorStagingInfo owns any allocated staging memory; cleanup happens in
// its destructor (RAII), so callers don't need to free explicitly on
// early-return paths or in switch cases that return directly.
struct TensorStagingInfo
{
    uintptr_t devicePtr = 0;          // GPU staging (0 if none)
    std::vector<uint8_t> hostBuf;     // CPU staging (empty if none)
    omni::physx::IOptionalCuda* cuda = nullptr;

    TensorStagingInfo() = default;
    TensorStagingInfo(const TensorStagingInfo&) = delete;
    TensorStagingInfo& operator=(const TensorStagingInfo&) = delete;
    TensorStagingInfo(TensorStagingInfo&& o) noexcept
        : devicePtr(o.devicePtr), hostBuf(std::move(o.hostBuf)), cuda(o.cuda)
    {
        o.devicePtr = 0;
        o.cuda = nullptr;
    }
    TensorStagingInfo& operator=(TensorStagingInfo&& o) noexcept
    {
        if (this != &o)
        {
            release();
            devicePtr = o.devicePtr;
            hostBuf = std::move(o.hostBuf);
            cuda = o.cuda;
            o.devicePtr = 0;
            o.cuda = nullptr;
        }
        return *this;
    }
    ~TensorStagingInfo() { release(); }

    void release()
    {
        if (devicePtr && cuda)
            cuda->memFree(devicePtr, nullptr);
        devicePtr = 0;
    }
};

static ovphysx_result_t stageTensorForWrite(
    const DLTensor* tensor,
    int bindingDeviceOrdinal,
    omni::physics::tensors::TensorDesc& desc,
    TensorStagingInfo& outStaging,
    const char* op)
{
    int tensorLogicalDevice = -1;
    switch (tensor->device.device_type)
    {
        case kDLCPU:
        case kDLCUDAHost:
            tensorLogicalDevice = -1;
            break;
        case kDLCUDA:
        case kDLCUDAManaged:
            tensorLogicalDevice = tensor->device.device_id;
            break;
        default:
        {
            std::ostringstream oss;
            oss << op << ": unsupported tensor device type "
                << static_cast<int>(tensor->device.device_type);
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, oss.str());
        }
    }

    // Same-device: no staging needed.
    if (bindingDeviceOrdinal == tensorLogicalDevice)
        return success();

    // Cross-GPU (different ordinals): rejected to match read-side policy.
    if (bindingDeviceOrdinal >= 0 && tensorLogicalDevice >= 0)
    {
        std::ostringstream oss;
        oss << op << ": cross-GPU staging (binding=" << bindingDeviceOrdinal
            << ", tensor=" << tensorLogicalDevice << ") is not supported";
        return set_error(OVPHYSX_API_DEVICE_MISMATCH, oss.str());
    }

    size_t numElements = 1;
    for (int i = 0; i < tensor->ndim; ++i)
        numElements *= static_cast<size_t>(tensor->shape[i]);
    const size_t elementBytes = static_cast<size_t>(tensor->dtype.bits / 8) *
                                static_cast<size_t>(tensor->dtype.lanes);
    const size_t byteCount = numElements * elementBytes;

    auto* cuda = getOptionalCuda();
    if (!cuda || !cuda->cudaAvailable())
    {
        return set_error(OVPHYSX_API_ERROR,
                         std::string(op) +
                         ": cross-device write requires IOptionalCuda; interface not available");
    }
    outStaging.cuda = cuda;

    // Source pointer for the staging copy: read from the byte-offset-adjusted
    // caller pointer (desc.data has already been computed by dlToTensorDesc as
    // tensor->data + tensor->byte_offset). Using tensor->data here would silently
    // copy from the start of the caller's allocation and skip any offset on a
    // sliced view, corrupting the staging buffer.
    void* const callerSrc = desc.data;
    if (bindingDeviceOrdinal >= 0)
    {
        // Binding GPU, tensor CPU → allocate GPU staging, memcpyHtoD caller's data.
        int cudaStatus = 0;
        if (!cuda->memAlloc(&outStaging.devicePtr, byteCount, &cudaStatus))
        {
            std::ostringstream oss;
            oss << op << ": GPU staging allocation failed (cuda_status="
                << cudaStatus << ", bytes=" << byteCount << ")";
            return set_error(OVPHYSX_API_ERROR, oss.str());
        }
        if (!cuda->memcpyHtoD(outStaging.devicePtr, callerSrc, byteCount, &cudaStatus))
        {
            std::ostringstream oss;
            oss << op << ": memcpyHtoD failed (cuda_status=" << cudaStatus
                << ", bytes=" << byteCount << ")";
            return set_error(OVPHYSX_API_ERROR, oss.str());
        }
        desc.data = reinterpret_cast<void*>(outStaging.devicePtr);
        desc.device = bindingDeviceOrdinal;
    }
    else
    {
        // Binding CPU, tensor GPU → allocate CPU staging, memcpyDtoH caller's data.
        outStaging.hostBuf.resize(byteCount);
        int cudaStatus = 0;
        if (!cuda->memcpyDtoH(outStaging.hostBuf.data(),
                              reinterpret_cast<uintptr_t>(callerSrc),
                              byteCount, &cudaStatus))
        {
            std::ostringstream oss;
            oss << op << ": memcpyDtoH failed (cuda_status=" << cudaStatus
                << ", bytes=" << byteCount << ")";
            return set_error(OVPHYSX_API_ERROR, oss.str());
        }
        desc.data = outStaging.hostBuf.data();
        desc.device = -1;
    }

    return success();
}

} // namespace

// Called from ovphysx_destroy_instance to clean up all tensor bindings
void ovphysx_tensor_binding_cleanup_instance(InstanceData* instance)
{
    if (!instance) return;
    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    for (auto& kv : instance->tensor_bindings)
        destroyBindingResources(kv.second);
    instance->tensor_bindings.clear();
}

extern "C" {

OVPHYSX_API ovphysx_result_t ovphysx_create_tensor_binding(
    ovphysx_handle_t handle,
    const ovphysx_tensor_binding_desc_t* desc,
    ovphysx_tensor_binding_handle_t* out_binding_handle)
{
    if (!desc || !out_binding_handle)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "invalid parameters");

    // Build list of patterns/paths - prim_paths takes precedence over pattern
    std::vector<std::string> patterns;
    bool usingExplicitPaths = (desc->prim_paths != nullptr && desc->prim_paths_count > 0);
    
    if (usingExplicitPaths)
    {
        for (uint32_t i = 0; i < desc->prim_paths_count; i++)
        {
            if (!desc->prim_paths[i].ptr || desc->prim_paths[i].length == 0)
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "prim_paths contains empty/null entry");
            if (hasEmbeddedNul(desc->prim_paths[i]))
                return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                                 "prim_paths contains an embedded NUL byte");
            patterns.push_back(toStdString(desc->prim_paths[i]));
        }
    }
    else if (desc->pattern.ptr && desc->pattern.length > 0)
    {
        if (hasEmbeddedNul(desc->pattern))
            return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                             "pattern contains an embedded NUL byte");
        patterns.push_back(toStdString(desc->pattern));
    }
    else
    {
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "must provide pattern or prim_paths");
    }

    omni_sdk_physx_wait_all_pending_internal(handle);

    // Ensure physics is attached and the initial scene parse has completed.
    // Without this, TensorAPI can't discover any physics prims.
    {
        ovphysx_api_status_t attach_status = ovphysx_ensure_physics_attached(handle);
        if (attach_status != OVPHYSX_API_SUCCESS)
            return set_error(attach_status, "failed to attach physics stage");
    }

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance || instance->attachedStageId == 0)
        return set_error(OVPHYSX_API_ERROR, "no USD stage loaded");

    auto* tensorApi = getTensorApi();
    if (!tensorApi)
        return set_error(OVPHYSX_API_ERROR, "TensorApi unavailable (plugins not loaded?)");

    TensorBindingState binding;
    struct BindingGuard {
        TensorBindingState* state{nullptr};
        bool active{true};
        ~BindingGuard() {
            if (active && state) {
                destroyBindingResources(*state);
            }
        }
        void disarm() { active = false; }
    } guard{&binding};

    binding.stageId = instance->attachedStageId;
    binding.tensorType = desc->tensor_type;
    // Store descriptive pattern string for debugging
    binding.pattern = usingExplicitPaths
        ? ("explicit_paths[" + std::to_string(desc->prim_paths_count) + "]")
        : patterns[0];

    binding.simView = tensorApi->createSimulationView(instance->attachedStageId);
    if (!binding.simView || !binding.simView->getValid())
    {
        return set_error(OVPHYSX_API_ERROR, "failed to create simulation view");
    }

    // Create appropriate view based on tensor type using vector overload.
    // TensorAPI may return nullptr if no prims match the pattern; ovphysx
    // treats that as a valid empty binding for optional pattern queries.
    {
        ScopedTensorNoMatchLogQuiet quietNoMatchLogs(binding.simView, !usingExplicitPaths);
        if (requiresRigidBodyView(desc->tensor_type))
        {
            binding.rbView = binding.simView->createRigidBodyView(patterns);
            // Null view is OK - means 0 prims matched. getCount() will return 0.
            if (binding.rbView)
            {
                CARB_LOG_INFO("Created rigid body binding with %u prims for pattern '%s'",
                              binding.rbView->getCount(), binding.pattern.c_str());
            }
            else
            {
                CARB_LOG_INFO("Created valid empty rigid body binding with 0 prims for pattern '%s'",
                              binding.pattern.c_str());
            }
        }
        else if (requiresArticulationView(desc->tensor_type))
        {
            binding.artiView = binding.simView->createArticulationView(patterns);
            // Null view is OK - means 0 prims matched. getCount() will return 0.
            if (binding.artiView)
            {
                // Centroidal momentum is only defined for floating-base articulations
                // (PhysX errors out on fixed-base). Reject up front so callers detect the
                // unsupported configuration at binding creation instead of at read time.
                // Check every matched articulation rather than the shared metatype: a pattern
                // may resolve to a heterogeneous mix of fixed- and floating-base articulations
                // (distinct metatypes, so getSharedMetatype() is null), and a single fixed-base
                // entry already makes the read undefined. Empty views (getCount()==0) stay valid.
                // The BindingGuard tears down the view on the early return.
                if (desc->tensor_type == OVPHYSX_TENSOR_ARTICULATION_CENTROIDAL_MOMENTUM_F32)
                {
                    const uint32_t artiCount = binding.artiView->getCount();
                    for (uint32_t i = 0; i < artiCount; ++i)
                    {
                        const auto* metatype = binding.artiView->getMetatype(i);
                        if (metatype && metatype->getFixedBase())
                            return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                                             "centroidal momentum is only defined for floating-base "
                                             "articulations; matched a fixed-base articulation");
                    }
                }
                CARB_LOG_INFO("Created articulation binding with %u prims for pattern '%s'",
                              binding.artiView->getCount(), binding.pattern.c_str());
            }
            else
            {
                CARB_LOG_INFO("Created valid empty articulation binding with 0 prims for pattern '%s'",
                              binding.pattern.c_str());
            }
        }
        else if (requiresDeformableBodyView(desc->tensor_type))
        {
            if (isSurfaceDeformableBodyType(desc->tensor_type))
                binding.defBodyView = binding.simView->createSurfaceDeformableBodyView(patterns);
            else
                binding.defBodyView = binding.simView->createVolumeDeformableBodyView(patterns);
            const char* bodyKind = isSurfaceDeformableBodyType(desc->tensor_type) ? "surface" : "volume";
            if (binding.defBodyView)
            {
                CARB_LOG_INFO("Created %s deformable body binding with %u prims for pattern '%s'",
                              bodyKind, binding.defBodyView->getCount(), binding.pattern.c_str());
            }
            else
            {
                CARB_LOG_WARN("%s deformable body binding created with 0 prims for pattern '%s'",
                              bodyKind, binding.pattern.c_str());
            }
        }
        else if (requiresDeformableMaterialView(desc->tensor_type))
        {
            binding.defMatView = binding.simView->createDeformableMaterialView(patterns);
            if (binding.defMatView)
            {
                CARB_LOG_INFO("Created deformable material binding with %u prims for pattern '%s'",
                              binding.defMatView->getCount(), binding.pattern.c_str());
            }
            else
            {
                CARB_LOG_WARN("Deformable material binding created with 0 prims for pattern '%s'",
                              binding.pattern.c_str());
            }
        }
        else
        {
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "unsupported tensor type");
        }
    }

    const ovphysx_tensor_binding_handle_t bindingHandle = ovphysx::internal::allocateOpaqueObjectHandle();
    if (bindingHandle == OVPHYSX_INVALID_HANDLE)
        return set_error(OVPHYSX_API_ERROR, "opaque object handle space exhausted");

    {
        std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
        instance->tensor_bindings[bindingHandle] = std::move(binding);
    }
    guard.disarm();

    *out_binding_handle = bindingHandle;
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_destroy_tensor_binding(
    ovphysx_handle_t handle,
    ovphysx_tensor_binding_handle_t binding_handle)
{
    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    {
        std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
        auto it = instance->tensor_bindings.find(binding_handle);
        if (it == instance->tensor_bindings.end())
            return set_error(OVPHYSX_API_NOT_FOUND, "binding not found");
        destroyBindingResources(it->second);
        instance->tensor_bindings.erase(it);
    }

    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_get_tensor_binding_spec(
    ovphysx_handle_t handle,
    ovphysx_tensor_binding_handle_t binding_handle,
    ovphysx_tensor_spec_t* out_spec)
{
    if (!out_spec)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_spec is NULL");

    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    int32_t ndim;
    int64_t shape[4];
    DLDataType dtype{};
    {
        // Hold the lock while computing the spec. TensorBindingState contains raw pointers
        // (TensorAPI views) that are released by ovphysx_destroy_tensor_binding().
        std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
        auto it = instance->tensor_bindings.find(binding_handle);
        if (it == instance->tensor_bindings.end())
            return set_error(OVPHYSX_API_NOT_FOUND, "binding not found");
        const TensorBindingState& binding = it->second;
        if (!getBindingSpec(binding, ndim, shape))
            return set_error(OVPHYSX_API_ERROR, "unsupported tensor type");
        dtype = getBindingDtype(binding.tensorType);
    }

    out_spec->dtype = dtype;
    out_spec->ndim = ndim;
    for (int i = 0; i < 4; i++)
        out_spec->shape[i] = shape[i];

    return success();
}

} // extern "C" (temporarily close for internal C++ helper)

ovphysx_result_t ovphysx_gpu_warmup_if_needed(ovphysx_handle_t handle, bool is_explicit_call)
{
    std::shared_lock<std::shared_mutex> check_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");
    
    // Check if warmup is needed
    const int64_t stageId = instance->attachedStageId;
    if (instance->gpu_warmup_done.load(std::memory_order_acquire) &&
        instance->gpu_warmup_stage_id.load(std::memory_order_acquire) == stageId)
        return success();  // Already done
    
    // GPU-disabled process: skip warmup (no CUDA resources to initialize). Covers both
    // OVPHYSX_DISABLE_GPU and ovphysx_set_cpu_mode(true); the latter is the primary
    // CPU-only path used by the CPU test suite. Skipping here avoids running a needless
    // warmup step in CPU-only mode.
    if (isProcessGpuDisabled())
        return success();
    
    // No stage attached yet
    if (stageId == 0)
        return success();
    
    // Release shared lock before warmup (simulate may need locks)
    check_lock.unlock();
    
    if (is_explicit_call) {
        CARB_LOG_INFO("[ovphysx] Explicit GPU warmup: performing initial simulation step to populate GPU buffers.");
    } else {
        CARB_LOG_WARN("[ovphysx] Auto-warmup: performing initial simulation step to populate GPU buffers. "
                      "Call ovphysx_warmup_gpu() explicitly to control when this happens.");
    }
    
    // Perform warmup step - we use a minimal elapsed time to minimize state change.
    // Note: PhysX requires elapsedSecs > 0 to actually dispatch simulation,
    // so we use an extremely small timestep. The physics state change is negligible.
    constexpr float kWarmupDt = 1.0e-9f;  // 1 nanosecond - effectively zero
    // IMPORTANT: DirectGPU requires a complete simulate()+fetchResults() warm
    // start before tensor bindings read or write GPU simulation buffers.
    //
    // Keep this synchronous and independent of op_index / async plumbing.
    const ovphysx_api_status_t simulate_status = omni_sdk_physx_simulate_instance(handle, kWarmupDt, 0.0f);
    if (simulate_status != OVPHYSX_API_SUCCESS)
    {
        return set_error(OVPHYSX_API_ERROR, "GPU warmup simulate() failed");
    }
    // Complete the warmup step by calling fetchResults().
    // NOTE: This logic mirrors the former omni_sdk_physx_sync() implementation in ovphysx.cpp.
    // Keep in sync if that internal helper is refactored.
    // This mirrors the simulation wait path in ovphysx_wait_op() for simulation events:
    // - fetchResults() must only be called when a stage is attached (it can hang otherwise)
    // - complete + cleanup the internal event and clear pending state
    try
    {
        std::shared_ptr<InstanceData> instanceShared = get_instance(handle);
        if (!instanceShared)
            return set_error(OVPHYSX_API_ERROR, "GPU warmup failed: invalid instance handle");

        std::unique_lock<std::mutex> instance_lock(instanceShared->simulationMutex);

        auto physxSim = instanceShared->carbonite ? instanceShared->carbonite->getPhysxSimulation() : nullptr;

        if (physxSim && instanceShared->attachedStageId != 0)
        {
            physxSim->fetchResults();
        }

        // Complete and cleanup the internal event created by omni_sdk_physx_simulate_instance().
        if (instanceShared->pendingSimulationEvent != 0)
        {
            const async_event_handle_t event_to_cleanup = instanceShared->pendingSimulationEvent;
            ovphysx::async::AsyncEventManager::complete_event(event_to_cleanup, true);
            ovphysx::async::AsyncEventManager::cleanup_event(event_to_cleanup);
            instanceShared->pendingSimulationEvent = 0;
        }
    }
    catch (const std::exception& e)
    {
        std::shared_ptr<InstanceData> instanceShared = get_instance(handle);
        if (instanceShared)
        {
            std::unique_lock<std::mutex> instance_lock(instanceShared->simulationMutex);
            if (instanceShared->pendingSimulationEvent != 0)
            {
                const async_event_handle_t event_to_cleanup = instanceShared->pendingSimulationEvent;
                ovphysx::async::AsyncEventManager::complete_event(event_to_cleanup, false, e.what());
                ovphysx::async::AsyncEventManager::cleanup_event(event_to_cleanup);
                instanceShared->pendingSimulationEvent = 0;
            }
        }
        return set_error(OVPHYSX_API_ERROR, "GPU warmup fetchResults() failed");
    }
    catch (...)
    {
        std::shared_ptr<InstanceData> instanceShared = get_instance(handle);
        if (instanceShared)
        {
            std::unique_lock<std::mutex> instance_lock(instanceShared->simulationMutex);
            if (instanceShared->pendingSimulationEvent != 0)
            {
                const async_event_handle_t event_to_cleanup = instanceShared->pendingSimulationEvent;
                ovphysx::async::AsyncEventManager::complete_event(event_to_cleanup, false, "Unknown exception");
                ovphysx::async::AsyncEventManager::cleanup_event(event_to_cleanup);
                instanceShared->pendingSimulationEvent = 0;
            }
        }
        return set_error(OVPHYSX_API_ERROR, "GPU warmup fetchResults() failed");
    }
    
    // Re-acquire lock and set flag
    std::shared_lock<std::shared_mutex> set_lock(g_instances_mutex);
    instance = get_instance_ptr(handle);
    if (instance)
    {
        instance->gpu_warmup_done.store(true, std::memory_order_release);
        instance->gpu_warmup_stage_id.store(instance->attachedStageId, std::memory_order_release);
        CARB_LOG_INFO("[ovphysx] GPU warmup complete (TensorBindingsAPI mode).");
    }
    
    return success();
}

extern "C" {

OVPHYSX_API ovphysx_result_t ovphysx_warmup_gpu(ovphysx_handle_t handle)
{
    omni_sdk_physx_wait_all_pending_internal(handle);
    return ovphysx_gpu_warmup_if_needed(handle, /*is_explicit_call=*/true);
}

OVPHYSX_API ovphysx_result_t ovphysx_update_articulations_kinematic(ovphysx_handle_t handle)
{
    ovphysx_api_status_t wait_status = omni_sdk_physx_wait_all_pending_internal(handle);
    if (wait_status != OVPHYSX_API_SUCCESS)
        return set_error(wait_status, "failed to complete pending operations before kinematic articulation update");

    ovphysx_api_status_t attach_status = ovphysx_ensure_physics_attached(handle);
    if (attach_status != OVPHYSX_API_SUCCESS)
        return set_error(attach_status, "failed to attach physics stage before kinematic articulation update");

    auto instance = get_instance(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");
    if (instance->attachedStageId == 0)
        return set_error(OVPHYSX_API_ERROR, "no USD stage loaded");

    auto* tensorApi = getTensorApi();
    if (!tensorApi)
        return set_error(OVPHYSX_API_ERROR, "TensorApi unavailable (plugins not loaded?)");

    // DirectGPU articulation data must be initialized before TensorAPI FK refresh.
    {
        ovphysx_result_t warmup_result = ovphysx_gpu_warmup_if_needed(handle, /*is_explicit_call=*/false);
        if (warmup_result.status != OVPHYSX_API_SUCCESS)
            return warmup_result;
    }

    omni::physics::tensors::ISimulationView* simView = tensorApi->createSimulationView(instance->attachedStageId);
    if (!simView)
        return set_error(OVPHYSX_API_ERROR, "failed to create simulation view");

    struct SimulationViewGuard
    {
        omni::physics::tensors::ISimulationView* view{nullptr};
        ~SimulationViewGuard()
        {
            if (view)
                view->release(false);
        }
    } guard{simView};

    if (!simView->getValid())
        return set_error(OVPHYSX_API_ERROR, "failed to create valid simulation view");

    simView->updateArticulationsKinematic();
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_read_tensor_binding(
    ovphysx_handle_t handle,
    ovphysx_tensor_binding_handle_t binding_handle,
    DLTensor* dst_tensor)
{
    if (!dst_tensor)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "dst_tensor is NULL");
    if (!dst_tensor->shape)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "dst_tensor has null shape");

    ovphysx_result_t processValidation = validateProcessTensorDevice(dst_tensor, "read_tensor_binding");
    if (processValidation.status != OVPHYSX_API_SUCCESS)
        return processValidation;

    omni_sdk_physx_wait_all_pending_internal(handle);

    // Auto-warmup if needed (GPU mode requires one simulation step to populate buffers)
    {
        ovphysx_result_t warmup_result = ovphysx_gpu_warmup_if_needed(handle, /*is_explicit_call=*/false);
        if (warmup_result.status != OVPHYSX_API_SUCCESS)
            return warmup_result;
    }

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    // CRITICAL: Hold lock during entire operation to prevent use-after-free if binding
    // is destroyed by another thread. TensorAPI views are NOT ref-counted.
    // TODO: If profiling shows contention, consider per-binding locks or ref-counting.
    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    
    auto it = instance->tensor_bindings.find(binding_handle);
    if (it == instance->tensor_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "binding not found");
    
    TensorBindingState& binding = it->second;

    if (instance->attachedStageId != binding.stageId)
    {
        return set_error(OVPHYSX_API_NOT_FOUND,
                "binding invalidated (stage changed): binding.stageId=" + std::to_string(binding.stageId) +
                           " current.attachedStageId=" + std::to_string(instance->attachedStageId) +
                           "; recreate binding");
    }

    // Check for write-only tensor types (force/wrench are control inputs, not readable)
    if (isWriteOnlyTensor(binding.tensorType))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "cannot read write-only tensor type (force/wrench)");

    ovphysx_result_t validation = validateTensorShape(dst_tensor, binding, "read_tensor_binding");
    if (validation.status != OVPHYSX_API_SUCCESS)
        return validation;

    // If the binding describes zero elements (zero matched prims OR any inner
    // dim is zero), there's nothing to read; succeed as a no-op. Skips
    // downstream TensorAPI calls that may not handle zero-size buffers.
    {
        int32_t bindingNdim = 0;
        int64_t bindingShape[4];
        if (!getBindingSpec(binding, bindingNdim, bindingShape))
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "unsupported tensor type");
        if (bindingHasZeroElements(bindingNdim, bindingShape))
            return success();
    }

    // Determine binding/dst devices to decide whether we need cross-device staging.
    // The binding's simView writes into a TensorDesc whose device must match its own
    // (PhysX's tensor API rejects cross-device targets). When the caller's dst lives
    // on a different memory space than the binding, we allocate a staging buffer
    // matching the binding's device, run the read into staging, then copy into the
    // caller's buffer.
    //
    //   (binding CPU,    dst CPU)  → existing path, no staging
    //   (binding GPU[X], dst GPU[X]) → existing path, no staging
    //   (binding GPU[X], dst CPU)  → GPU staging + memcpyDtoH after read
    //   (binding CPU,    dst GPU)  → CPU staging + memcpyHtoD after read
    //   (binding GPU[X], dst GPU[Y], X!=Y) → rejected by validateTensorDevice
    //                                        (cross-GPU P2P out of scope)
    int bindingDeviceOrdinal = -1;
    if (!isCpuOnlyTensorType(binding.tensorType) && binding.simView)
        bindingDeviceOrdinal = binding.simView->getDeviceOrdinal();

    int dstLogicalDevice = -1;
    switch (dst_tensor->device.device_type)
    {
        case kDLCPU:
        case kDLCUDAHost:
            dstLogicalDevice = -1;
            break;
        case kDLCUDA:
        case kDLCUDAManaged:
            dstLogicalDevice = dst_tensor->device.device_id;
            break;
        default:
            // Leave as -1; validateTensorDevice will surface the unsupported-type error.
            break;
    }

    const bool needGpuStaging = (bindingDeviceOrdinal >= 0 && dstLogicalDevice == -1);
    const bool needCpuStaging = (bindingDeviceOrdinal == -1 && dstLogicalDevice >= 0);

    if (!needGpuStaging && !needCpuStaging)
    {
        // Same-device path: existing strict device check.
        ovphysx_result_t deviceValidation = validateTensorDevice(dst_tensor, binding, "read_tensor_binding");
        if (deviceValidation.status != OVPHYSX_API_SUCCESS)
            return deviceValidation;
    }

    // If the binding matches zero elements, treat reads as a successful no-op.
    // (Bindings are allowed to be created with 0 matches.)
    {
        int32_t specNdim = 0;
        int64_t specShape[4];
        if (!getBindingSpec(binding, specNdim, specShape))
            return set_error(OVPHYSX_API_ERROR, "unsupported tensor type");
        if (specShape[0] == 0)
            return success();
    }


    omni::physics::tensors::TensorDesc dst{};
    DLConvertError convertErr = dlToTensorDesc(dst_tensor, dst);
    if (convertErr != DLConvertError::Success)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, dlConvertErrorMessage(convertErr));

    // Hold the byte-offset-adjusted user destination pointer so we can write
    // the staged result back into the correct slice. dlToTensorDesc applied
    // dst_tensor->byte_offset to dst.data, but the cross-device staging path
    // overwrites dst.data with the staging buffer; without a saved copy we'd
    // memcpy back to dst_tensor->data (i.e. base allocation, ignoring
    // byte_offset) and corrupt unrelated rows of a sliced view.
    void* const userDstData = dst.data;

    // Allocate staging when we need to bridge a cross-device read.
    // GPU staging when (binding GPU, dst CPU); CPU staging when (binding CPU, dst GPU).
    // Lifetime is tied to readStaging's destructor (RAII); any switch case that
    // returns directly will release the GPU buffer automatically.
    TensorStagingInfo readStaging;
    size_t stagingByteCount = 0;
    if (needGpuStaging || needCpuStaging)
    {
        // Compute byte count from the validated shape + dtype.
        size_t numElements = 1;
        for (int i = 0; i < dst_tensor->ndim; ++i)
            numElements *= static_cast<size_t>(dst_tensor->shape[i]);
        const size_t elementBytes = static_cast<size_t>(dst_tensor->dtype.bits / 8) *
                                    static_cast<size_t>(dst_tensor->dtype.lanes);
        stagingByteCount = numElements * elementBytes;

        // Both directions need IOptionalCuda for the host<->device copy step.
        omni::physx::IOptionalCuda* cuda = getOptionalCuda();
        if (!cuda || !cuda->cudaAvailable())
        {
            return set_error(OVPHYSX_API_ERROR,
                             "read_tensor_binding: cross-device read requires IOptionalCuda; "
                             "interface not available");
        }
        readStaging.cuda = cuda;

        if (needGpuStaging)
        {
            int cudaStatus = 0;
            if (!cuda->memAlloc(&readStaging.devicePtr, stagingByteCount, &cudaStatus))
            {
                std::ostringstream oss;
                oss << "read_tensor_binding: GPU staging allocation failed (cuda_status="
                    << cudaStatus << ", bytes=" << stagingByteCount << ")";
                return set_error(OVPHYSX_API_ERROR, oss.str());
            }
            // Redirect the simView read into the GPU staging buffer; memcpyDtoH happens
            // after the switch.
            dst.data   = reinterpret_cast<void*>(readStaging.devicePtr);
            dst.device = bindingDeviceOrdinal;
        }
        else  // needCpuStaging
        {
            readStaging.hostBuf.resize(stagingByteCount);
            // Redirect the simView read into the host staging buffer; memcpyHtoD happens
            // after the switch to copy into the user's GPU dst.
            dst.data   = readStaging.hostBuf.data();
            dst.device = -1;  // CPU
        }
    }

    bool ok = false;
    switch (binding.tensorType)
    {
        // Rigid body
        case OVPHYSX_TENSOR_RIGID_BODY_POSE_F32:
            ok = binding.rbView && binding.rbView->getTransforms(&dst);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32:
            ok = binding.rbView && binding.rbView->getVelocities(&dst);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_ACCELERATION_F32:
            ok = binding.rbView && binding.rbView->getAccelerations(&dst);
            break;

        // Articulation root
        case OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32:
            ok = binding.artiView && binding.artiView->getRootTransforms(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32:
            ok = binding.artiView && binding.artiView->getRootVelocities(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_WORLD_F32:
            ok = binding.artiView && binding.artiView->getArticulationMassCenter(&dst, /*localFrame=*/false);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_LOCAL_F32:
            ok = binding.artiView && binding.artiView->getArticulationMassCenter(&dst, /*localFrame=*/true);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_CENTROIDAL_MOMENTUM_F32:
            // Floating-base requirement is enforced by the underlying
            // CpuArticulationView / GpuArticulationView impl, which returns
            // false and logs an error on fixed-base. Surface that as a
            // generic read failure (last_error preserves the engine message).
            ok = binding.artiView && binding.artiView->getArticulationCentroidalMomentum(&dst);
            break;

        // Articulation links
        case OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32:
            ok = binding.artiView && binding.artiView->getLinkTransforms(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_LINK_VELOCITY_F32:
            ok = binding.artiView && binding.artiView->getLinkVelocities(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_LINK_ACCELERATION_F32:
            ok = binding.artiView && binding.artiView->getLinkAccelerations(&dst);
            break;

        // Articulation DOF
        case OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32:
            ok = binding.artiView && binding.artiView->getDofPositions(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32:
            ok = binding.artiView && binding.artiView->getDofVelocities(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32:
            ok = binding.artiView && binding.artiView->getDofPositionTargets(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32:
            ok = binding.artiView && binding.artiView->getDofVelocityTargets(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32:
            ok = binding.artiView && binding.artiView->getDofActuationForces(&dst);
            break;

        // DOF properties
        case OVPHYSX_TENSOR_ARTICULATION_DOF_STIFFNESS_F32:
            ok = binding.artiView && binding.artiView->getDofStiffnesses(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_DAMPING_F32:
            ok = binding.artiView && binding.artiView->getDofDampings(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_LIMIT_F32:
            ok = binding.artiView && binding.artiView->getDofLimits(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_VELOCITY_F32:
            ok = binding.artiView && binding.artiView->getDofMaxVelocities(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_FORCE_F32:
            ok = binding.artiView && binding.artiView->getDofMaxForces(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_ARMATURE_F32:
            ok = binding.artiView && binding.artiView->getDofArmatures(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_MODEL_F32:
            ok = binding.artiView && binding.artiView->getDofDriveModelProperties(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_FRICTION_PROPERTIES_F32:
            ok = binding.artiView && binding.artiView->getDofFrictionProperties(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_TYPE_U8:
            ok = binding.artiView && binding.artiView->getDriveTypes(&dst);
            break;

        // Body properties
        case OVPHYSX_TENSOR_ARTICULATION_BODY_MASS_F32:
            ok = binding.artiView && binding.artiView->getMasses(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_BODY_COM_POSE_F32:
            ok = binding.artiView && binding.artiView->getCOMs(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_BODY_INERTIA_F32:
            ok = binding.artiView && binding.artiView->getInertias(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL:
            ok = binding.artiView && binding.artiView->getDisableGravities(&dst);
            break;

        // Dynamics queries (read-only)
        case OVPHYSX_TENSOR_ARTICULATION_JACOBIAN_F32:
            ok = binding.artiView && binding.artiView->getJacobians(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_MASS_MATRIX_F32:
            ok = binding.artiView && binding.artiView->getGeneralizedMassMatrices(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE_F32:
            ok = binding.artiView && binding.artiView->getCoriolisAndCentrifugalCompensationForces(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_GRAVITY_FORCE_F32:
            ok = binding.artiView && binding.artiView->getGravityCompensationForces(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_LINK_INCOMING_JOINT_FORCE_F32:
            ok = binding.artiView && binding.artiView->getLinkIncomingJointForce(&dst);
            break;

        case OVPHYSX_TENSOR_ARTICULATION_DOF_PROJECTED_JOINT_FORCE_F32:
            ok = binding.artiView && binding.artiView->getDofProjectedJointForces(&dst);
            break;

        // Standalone rigid body properties
        case OVPHYSX_TENSOR_RIGID_BODY_MASS_F32:
            ok = binding.rbView && binding.rbView->getMasses(&dst);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_INERTIA_F32:
            ok = binding.rbView && binding.rbView->getInertias(&dst);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_COM_POSE_F32:
            ok = binding.rbView && binding.rbView->getCOMs(&dst);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_INV_MASS_F32:
            ok = binding.rbView && binding.rbView->getInvMasses(&dst);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_INV_INERTIA_F32:
            ok = binding.rbView && binding.rbView->getInvInertias(&dst);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL:
            ok = binding.rbView && binding.rbView->getDisableSimulations(&dst);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_DISABLE_GRAVITY_BOOL:
            ok = binding.rbView && binding.rbView->getDisableGravities(&dst);
            break;

        // Articulation body inverse properties (read-only)
        case OVPHYSX_TENSOR_ARTICULATION_BODY_INV_MASS_F32:
            ok = binding.artiView && binding.artiView->getInvMasses(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_BODY_INV_INERTIA_F32:
            ok = binding.artiView && binding.artiView->getInvInertias(&dst);
            break;

        // Fixed tendon properties
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_STIFFNESS_F32:
            ok = binding.artiView && binding.artiView->getFixedTendonStiffnesses(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_DAMPING_F32:
            ok = binding.artiView && binding.artiView->getFixedTendonDampings(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_STIFFNESS_F32:
            ok = binding.artiView && binding.artiView->getFixedTendonLimitStiffnesses(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_F32:
            ok = binding.artiView && binding.artiView->getFixedTendonLimits(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_REST_LENGTH_F32:
            ok = binding.artiView && binding.artiView->getFixedTendonfixedSpringRestLengths(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_OFFSET_F32:
            ok = binding.artiView && binding.artiView->getFixedTendonOffsets(&dst);
            break;

        // Spatial tendon properties
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_STIFFNESS_F32:
            ok = binding.artiView && binding.artiView->getSpatialTendonStiffnesses(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_DAMPING_F32:
            ok = binding.artiView && binding.artiView->getSpatialTendonDampings(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_LIMIT_STIFFNESS_F32:
            ok = binding.artiView && binding.artiView->getSpatialTendonLimitStiffnesses(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_OFFSET_F32:
            ok = binding.artiView && binding.artiView->getSpatialTendonOffsets(&dst);
            break;

        // Rigid body shape-level properties
        case OVPHYSX_TENSOR_RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION_F32:
            ok = binding.rbView && binding.rbView->getMaterialProperties(&dst);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_CONTACT_OFFSET_F32:
            ok = binding.rbView && binding.rbView->getContactOffsets(&dst);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_REST_OFFSET_F32:
            ok = binding.rbView && binding.rbView->getRestOffsets(&dst);
            break;

        // Articulation shape-level properties
        case OVPHYSX_TENSOR_ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION_F32:
            ok = binding.artiView && binding.artiView->getMaterialProperties(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_CONTACT_OFFSET_F32:
            ok = binding.artiView && binding.artiView->getContactOffsets(&dst);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_REST_OFFSET_F32:
            ok = binding.artiView && binding.artiView->getRestOffsets(&dst);
            break;

        // Volume deformable body state
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_POSITION_F32:
            ok = binding.defBodyView && binding.defBodyView->getSimulationNodalPositions(&dst);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_VELOCITY_F32:
            ok = binding.defBodyView && binding.defBodyView->getSimulationNodalVelocities(&dst);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_KINEMATIC_TARGET_F32:
            ok = binding.defBodyView && binding.defBodyView->getSimulationNodalKinematicTargets(&dst);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_REST_NODAL_POSITION_F32:
            ok = binding.defBodyView && binding.defBodyView->getRestNodalPositions(&dst);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_ELEMENT_INDICES_S32:
            ok = binding.defBodyView && binding.defBodyView->getSimulationElementIndices(&dst);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_COLLISION_ELEMENT_INDICES_S32:
            ok = binding.defBodyView && binding.defBodyView->getCollisionElementIndices(&dst);
            break;

        // Surface deformable body state
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_POSITION_F32:
            ok = binding.defBodyView && binding.defBodyView->getSimulationNodalPositions(&dst);
            break;
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_VELOCITY_F32:
            ok = binding.defBodyView && binding.defBodyView->getSimulationNodalVelocities(&dst);
            break;
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_REST_POSITION_F32:
            ok = binding.defBodyView && binding.defBodyView->getRestNodalPositions(&dst);
            break;
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_ELEMENT_INDICES_S32:
            ok = binding.defBodyView && binding.defBodyView->getSimulationElementIndices(&dst);
            break;

        // Deformable material properties
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_DYNAMIC_FRICTION_F32:
            ok = binding.defMatView && binding.defMatView->getDynamicFriction(&dst);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_YOUNGS_MODULUS_F32:
            ok = binding.defMatView && binding.defMatView->getYoungsModulus(&dst);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_POISSONS_RATIO_F32:
            ok = binding.defMatView && binding.defMatView->getPoissonsRatio(&dst);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_ELASTICITY_DAMPING_F32:
            ok = binding.defMatView && binding.defMatView->getElasticityDamping(&dst);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_STIFFNESS_F32:
            ok = binding.defMatView && binding.defMatView->getBendingStiffness(&dst);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_THICKNESS_F32:
            ok = binding.defMatView && binding.defMatView->getThickness(&dst);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_DAMPING_F32:
            ok = binding.defMatView && binding.defMatView->getBendingDamping(&dst);
            break;

        default:
            // readStaging's destructor releases any GPU staging buffer when we return.
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "unsupported tensor type");
    }

    if (!ok)
    {
        // readStaging's destructor releases any GPU staging buffer when we return.
        return set_error(OVPHYSX_API_ERROR, "TensorAPI read failed");
    }

    // Cross-device staging copy: shuttle the staged read into the caller's buffer.
    // readStaging owns the buffers; we let its destructor release them at end of
    // function rather than freeing eagerly, which keeps the success/failure paths
    // symmetric.
    if (needGpuStaging)
    {
        int cudaStatus = 0;
        const bool copyOk = readStaging.cuda->memcpyDtoH(
            userDstData, readStaging.devicePtr, stagingByteCount, &cudaStatus);
        if (!copyOk)
        {
            std::ostringstream oss;
            oss << "read_tensor_binding: memcpyDtoH failed (cuda_status=" << cudaStatus
                << ", bytes=" << stagingByteCount << ")";
            return set_error(OVPHYSX_API_ERROR, oss.str());
        }
    }
    else if (needCpuStaging)
    {
        int cudaStatus = 0;
        const bool copyOk = readStaging.cuda->memcpyHtoD(
            reinterpret_cast<uintptr_t>(userDstData),
            readStaging.hostBuf.data(), stagingByteCount, &cudaStatus);
        if (!copyOk)
        {
            std::ostringstream oss;
            oss << "read_tensor_binding: memcpyHtoD failed (cuda_status=" << cudaStatus
                << ", bytes=" << stagingByteCount << ")";
            return set_error(OVPHYSX_API_ERROR, oss.str());
        }
    }

    return success();
}

// Internal helper: caller must already hold instance->tensor_binding_mutex.
// Performs the actual indexed/non-indexed write dispatch without re-acquiring the mutex.
static ovphysx_result_t write_tensor_binding_locked(
    InstanceData* instance,
    ovphysx_tensor_binding_handle_t binding_handle,
    const DLTensor* src_tensor,
    const DLTensor* index_tensor)
{
    auto it = instance->tensor_bindings.find(binding_handle);
    if (it == instance->tensor_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "binding not found");
    
    TensorBindingState& binding = it->second;

    if (instance->attachedStageId != binding.stageId)
    {
        return set_error(OVPHYSX_API_NOT_FOUND,
                "binding invalidated (stage changed): binding.stageId=" + std::to_string(binding.stageId) +
                           " current.attachedStageId=" + std::to_string(instance->attachedStageId) +
                           "; recreate binding");
    }

    int32_t bindingNdim = 0;
    int64_t bindingShape[4];
    if (!getBindingSpec(binding, bindingNdim, bindingShape))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "unsupported tensor type");

    // Validate src_tensor shape: always require full [N,...] regardless of indexing
    {
        ovphysx_result_t validation = validateTensorShape(src_tensor, binding, "write_tensor_binding");
        if (validation.status != OVPHYSX_API_SUCCESS)
            return validation;
    }

    if (index_tensor)
    {
        if (!index_tensor->shape)
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "index_tensor has null shape");

        if (index_tensor->ndim != 1)
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "index_tensor must be 1D");
        const bool isInt32 = (index_tensor->dtype.code == kDLInt && index_tensor->dtype.bits == 32 && index_tensor->dtype.lanes == 1);
        const bool isInt64 = (index_tensor->dtype.code == kDLInt && index_tensor->dtype.bits == 64 && index_tensor->dtype.lanes == 1);
        if (!isInt32 && !isInt64)
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "index_tensor must be int32 or int64");

        if (index_tensor->shape[0] > bindingShape[0])
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "index_tensor length exceeds binding element count");
    }

    // If the binding describes zero elements (zero matched prims OR any inner
    // dim is zero, e.g. (count, max_dofs=0)), treat writes as a no-op success.
    if (bindingHasZeroElements(bindingNdim, bindingShape))
        return success();

    // Stage int64 index tensors down to int32. The underlying TensorAPI only
    // accepts int32 indices; callers such as the IsaacSim umbrella adapter may
    // supply int64. CPU-only: reading a GPU pointer from the host would crash,
    // so GPU int64 indices are rejected with a clear error.
    std::vector<int32_t> int64StagingBuf;
    DLTensor int64StagedTensor{};
    int64_t int64StagedShape = 0;
    const DLTensor* effective_index_tensor = index_tensor;
    if (index_tensor && index_tensor->dtype.code == kDLInt && index_tensor->dtype.bits == 64)
    {
        if (index_tensor->device.device_type != kDLCPU && index_tensor->device.device_type != kDLCUDAHost)
            return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                             "int64 index_tensor must be CPU-resident; GPU int64 not supported -- "
                             "convert to int32 before calling ovphysx_write_tensor_binding");

        const int64_t K = index_tensor->shape[0];
        int64StagingBuf.resize(static_cast<size_t>(K));
        const int64_t* src64 = reinterpret_cast<const int64_t*>(
            reinterpret_cast<const char*>(index_tensor->data) + index_tensor->byte_offset);
        for (int64_t i = 0; i < K; ++i)
            int64StagingBuf[static_cast<size_t>(i)] = static_cast<int32_t>(src64[i]);
        int64StagedShape = K;
        int64StagedTensor.data        = int64StagingBuf.data();
        int64StagedTensor.device      = { kDLCPU, 0 };
        int64StagedTensor.ndim        = 1;
        int64StagedTensor.dtype       = { kDLInt, 32, 1 };
        int64StagedTensor.shape       = &int64StagedShape;
        int64StagedTensor.strides     = nullptr;
        int64StagedTensor.byte_offset = 0;
        effective_index_tensor = &int64StagedTensor;
    }

    omni::physics::tensors::TensorDesc src{};
    DLConvertError srcErr = dlToTensorDesc(src_tensor, src);
    if (srcErr != DLConvertError::Success)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                         std::string("source tensor: ") + dlConvertErrorMessage(srcErr));

    omni::physics::tensors::TensorDesc idx{};
    const omni::physics::tensors::TensorDesc* idxPtr = nullptr;
    if (effective_index_tensor)
    {
        DLConvertError idxErr = dlToTensorDesc(effective_index_tensor, idx);
        if (idxErr != DLConvertError::Success)
            return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                             std::string("index tensor: ") + dlConvertErrorMessage(idxErr));
        idxPtr = &idx;

        // Validate index values on CPU (GPU indices are validated by TensorAPI/driver).
        // Runs against the (possibly staged) int32 buffer.
        if (idx.device < 0 && idx.data)
        {
            const int64_t K = effective_index_tensor->shape[0];
            const int32_t* idxData = static_cast<const int32_t*>(idx.data);
            for (int64_t i = 0; i < K; ++i)
            {
                const int32_t v = idxData[i];
                if (v < 0 || v >= bindingShape[0])
                    return set_error(OVPHYSX_API_INVALID_ARGUMENT, "index_tensor contains out-of-range index");
            }
        }
    }

    // Cross-device staging: allow callers to pass a tensor on a different
    // device than the binding (e.g. GPU tensor against a CPU-only property
    // binding). On same-device input no staging is allocated; on cross-
    // device input, allocates a buffer on the binding's device, copies the
    // caller's data into it, and redirects the TensorDesc to the staging
    // buffer. Cleanup is automatic via TensorStagingInfo's destructor.
    int bindingDeviceOrdinal = -1;
    if (!isCpuOnlyTensorType(binding.tensorType) && binding.simView)
        bindingDeviceOrdinal = binding.simView->getDeviceOrdinal();

    TensorStagingInfo srcStaging;
    {
        ovphysx_result_t r = stageTensorForWrite(
            src_tensor, bindingDeviceOrdinal, src, srcStaging, "write_tensor_binding");
        if (r.status != OVPHYSX_API_SUCCESS)
            return r;
    }

    TensorStagingInfo idxStaging;
    if (effective_index_tensor)
    {
        ovphysx_result_t r = stageTensorForWrite(
            effective_index_tensor, bindingDeviceOrdinal, idx, idxStaging, "write_tensor_binding (index)");
        if (r.status != OVPHYSX_API_SUCCESS)
            return r;
    }

    bool ok = false;
    switch (binding.tensorType)
    {
        // Rigid body
        case OVPHYSX_TENSOR_RIGID_BODY_POSE_F32:
            ok = binding.rbView && binding.rbView->setTransforms(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32:
            ok = binding.rbView && binding.rbView->setVelocities(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_ACCELERATION_F32:
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "rigid body acceleration is read-only");

        // Articulation root
        case OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32:
            ok = binding.artiView && binding.artiView->setRootTransforms(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32:
            ok = binding.artiView && binding.artiView->setRootVelocities(&src, idxPtr);
            break;

        // Articulation links are read-only (no TensorAPI setter)
        case OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_LINK_VELOCITY_F32:
        case OVPHYSX_TENSOR_ARTICULATION_LINK_ACCELERATION_F32:
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "link poses/velocities/accelerations are read-only");

        case OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_WORLD_F32:
        case OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_LOCAL_F32:
        case OVPHYSX_TENSOR_ARTICULATION_CENTROIDAL_MOMENTUM_F32:
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "articulation mass center / centroidal momentum are read-only");

        // Articulation DOF
        case OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32:
            ok = binding.artiView && binding.artiView->setDofPositions(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32:
            ok = binding.artiView && binding.artiView->setDofVelocities(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32:
            ok = binding.artiView && binding.artiView->setDofPositionTargets(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32:
            ok = binding.artiView && binding.artiView->setDofVelocityTargets(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32:
            ok = binding.artiView && binding.artiView->setDofActuationForces(&src, idxPtr);
            break;

        // DOF properties (indexed write)
        case OVPHYSX_TENSOR_ARTICULATION_DOF_STIFFNESS_F32:
            ok = binding.artiView && binding.artiView->setDofStiffnesses(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_DAMPING_F32:
            ok = binding.artiView && binding.artiView->setDofDampings(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_LIMIT_F32:
            ok = binding.artiView && binding.artiView->setDofLimits(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_VELOCITY_F32:
            ok = binding.artiView && binding.artiView->setDofMaxVelocities(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_FORCE_F32:
            ok = binding.artiView && binding.artiView->setDofMaxForces(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_ARMATURE_F32:
            ok = binding.artiView && binding.artiView->setDofArmatures(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_MODEL_F32:
            ok = binding.artiView && binding.artiView->setDofDriveModelProperties(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_FRICTION_PROPERTIES_F32:
            ok = binding.artiView && binding.artiView->setDofFrictionProperties(&src, idxPtr);
            break;

        // Body properties (indexed write)
        case OVPHYSX_TENSOR_ARTICULATION_BODY_MASS_F32:
            ok = binding.artiView && binding.artiView->setMasses(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_BODY_COM_POSE_F32:
            ok = binding.artiView && binding.artiView->setCOMs(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_BODY_INERTIA_F32:
            ok = binding.artiView && binding.artiView->setInertias(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL:
            ok = binding.artiView && binding.artiView->setDisableGravities(&src, idxPtr);
            break;

        // Standalone rigid body properties (indexed write)
        case OVPHYSX_TENSOR_RIGID_BODY_MASS_F32:
            ok = binding.rbView && binding.rbView->setMasses(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_INERTIA_F32:
            ok = binding.rbView && binding.rbView->setInertias(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_COM_POSE_F32:
            ok = binding.rbView && binding.rbView->setCOMs(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_INV_MASS_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_INV_INERTIA_F32:
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "rigid body inverse mass/inertia are read-only");
        case OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL:
            ok = binding.rbView && binding.rbView->setDisableSimulations(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_DISABLE_GRAVITY_BOOL:
            ok = binding.rbView && binding.rbView->setDisableGravities(&src, idxPtr);
            break;

        // Fixed tendon properties (indexed write via batch setter -- reads back other properties)
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_STIFFNESS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_DAMPING_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_STIFFNESS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_REST_LENGTH_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_OFFSET_F32:
        {
            if (!binding.artiView)
                return set_error(OVPHYSX_API_ERROR, "binding has no articulation view");

            ok = writeFixedTendonProperty(binding, src, idxPtr, nullptr);
            break;
        }

        // Spatial tendon properties (indexed write via batch setter -- reads back other properties)
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_STIFFNESS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_DAMPING_F32:
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_LIMIT_STIFFNESS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_OFFSET_F32:
        {
            if (!binding.artiView)
                return set_error(OVPHYSX_API_ERROR, "binding has no articulation view");

            ok = writeSpatialTendonProperty(binding, src, idxPtr, nullptr);
            break;
        }

        // Rigid body shape-level properties (indexed write)
        case OVPHYSX_TENSOR_RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION_F32:
            ok = binding.rbView && binding.rbView->setMaterialProperties(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_CONTACT_OFFSET_F32:
            ok = binding.rbView && binding.rbView->setContactOffsets(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_REST_OFFSET_F32:
            ok = binding.rbView && binding.rbView->setRestOffsets(&src, idxPtr);
            break;

        // Articulation shape-level properties (indexed write)
        case OVPHYSX_TENSOR_ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION_F32:
            ok = binding.artiView && binding.artiView->setMaterialProperties(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_CONTACT_OFFSET_F32:
            ok = binding.artiView && binding.artiView->setContactOffsets(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_REST_OFFSET_F32:
            ok = binding.artiView && binding.artiView->setRestOffsets(&src, idxPtr);
            break;

        // Volume deformable body state (indexed write)
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_POSITION_F32:
            ok = binding.defBodyView && binding.defBodyView->setSimulationNodalPositions(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_VELOCITY_F32:
            ok = binding.defBodyView && binding.defBodyView->setSimulationNodalVelocities(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_KINEMATIC_TARGET_F32:
            ok = binding.defBodyView && binding.defBodyView->setSimulationNodalKinematicTargets(&src, idxPtr);
            break;

        // Surface deformable body state (indexed write)
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_POSITION_F32:
            ok = binding.defBodyView && binding.defBodyView->setSimulationNodalPositions(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_VELOCITY_F32:
            ok = binding.defBodyView && binding.defBodyView->setSimulationNodalVelocities(&src, idxPtr);
            break;

        // Deformable material properties (indexed write)
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_DYNAMIC_FRICTION_F32:
            ok = binding.defMatView && binding.defMatView->setDynamicFriction(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_YOUNGS_MODULUS_F32:
            ok = binding.defMatView && binding.defMatView->setYoungsModulus(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_POISSONS_RATIO_F32:
            ok = binding.defMatView && binding.defMatView->setPoissonsRatio(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_ELASTICITY_DAMPING_F32:
            ok = binding.defMatView && binding.defMatView->setElasticityDamping(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_STIFFNESS_F32:
            ok = binding.defMatView && binding.defMatView->setBendingStiffness(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_THICKNESS_F32:
            ok = binding.defMatView && binding.defMatView->setThickness(&src, idxPtr);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_DAMPING_F32:
            ok = binding.defMatView && binding.defMatView->setBendingDamping(&src, idxPtr);
            break;

        // Read-only tensors
        case OVPHYSX_TENSOR_ARTICULATION_JACOBIAN_F32:
        case OVPHYSX_TENSOR_ARTICULATION_MASS_MATRIX_F32:
        case OVPHYSX_TENSOR_ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_GRAVITY_FORCE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_LINK_INCOMING_JOINT_FORCE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_PROJECTED_JOINT_FORCE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_BODY_INV_MASS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_BODY_INV_INERTIA_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_REST_NODAL_POSITION_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_ELEMENT_INDICES_S32:
        case OVPHYSX_TENSOR_DEFORMABLE_COLLISION_ELEMENT_INDICES_S32:
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_REST_POSITION_F32:
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_ELEMENT_INDICES_S32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_TYPE_U8:
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "tensor type is read-only");

        // External forces - rigid body [N, 3] at center of mass
        case OVPHYSX_TENSOR_RIGID_BODY_FORCE_F32:
            ok = binding.rbView && binding.rbView->applyForces(&src, idxPtr);
            break;

        // External wrenches - rigid body [N, 9]  (AoS -> SoA conversion, see masked variant for rationale)
        case OVPHYSX_TENSOR_RIGID_BODY_WRENCH_F32:
        {
            if (!binding.rbView)
                return set_error(OVPHYSX_API_ERROR, "binding has no rigid body view");

            if (src_tensor->ndim != 2)
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "wrench tensor must have shape [N, 9]");

            const int64_t N = src_tensor->shape[0];
            if (src_tensor->shape[1] != 9)
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "wrench tensor must have shape [N, 9]");

            const float* srcPtr = reinterpret_cast<const float*>(src.data);
            const bool isGpu = (src.device >= 0);
            
            // Allocate temporary SoA buffer (forces|torques|positions)
            if (N < 0)
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "invalid wrench tensor shape");

            if (N > static_cast<int64_t>(std::numeric_limits<size_t>::max() / (9 * sizeof(float))))
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "wrench tensor too large");

            // Use (and grow as needed) a per-binding cached SoA scratch buffer to avoid
            // per-call GPU allocations in tight loops.
            const size_t soaSize = static_cast<size_t>(N) * 9 * sizeof(float);
            
            if (isGpu)
            {
                auto* cuda = getOptionalCuda();
                const uintptr_t srcDev = reinterpret_cast<uintptr_t>(src.data);
                const uintptr_t cudaCtx = binding.simView ? reinterpret_cast<uintptr_t>(binding.simView->getCudaContext()) : 0;
                ovphysx_result_t scratchResult = ensureWrenchScratchGpu(binding, cuda, cudaCtx, soaSize, "wrench conversion");
                if (scratchResult.status != OVPHYSX_API_SUCCESS)
                    return scratchResult;

                ovphysx_result_t convResult = convertWrenchAoSToSoaGpu(
                    cuda, binding.wrenchSoaScratchDev, srcDev, N, cudaCtx, "wrench conversion");
                if (convResult.status != OVPHYSX_API_SUCCESS)
                    return convResult;

                auto soa = buildWrenchSoaDescs(src, reinterpret_cast<void*>(binding.wrenchSoaScratchDev), N, N);
                ok = binding.rbView->applyForcesAndTorquesAtPosition(
                    &soa.force, &soa.torque, &soa.position, idxPtr, /*isGlobal=*/true);
                break;
            }
            else
            {
                // CPU path
                std::unique_ptr<float[]> soaBuffer(new float[static_cast<size_t>(N) * 9]);
                ovphysx::internal::convertWrenchAosToSoaCpu(srcPtr, N, soaBuffer.get());
                
                auto soa = buildWrenchSoaDescs(src, soaBuffer.get(), N, N);
                ok = binding.rbView->applyForcesAndTorquesAtPosition(
                    &soa.force, &soa.torque, &soa.position, idxPtr, /*isGlobal=*/true);
            }
            
            break;
        }

        // External wrenches - articulation links [N, L, 9]  (AoS -> SoA, same rationale as rigid body wrench above)
        case OVPHYSX_TENSOR_ARTICULATION_LINK_WRENCH_F32:
        {
            if (!binding.artiView)
                return set_error(OVPHYSX_API_ERROR, "binding has no articulation view");

            if (src_tensor->ndim != 3)
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "link wrench tensor must have shape [N, L, 9]");

            const int64_t N = src_tensor->shape[0];
            const int64_t L = src_tensor->shape[1];
            if (src_tensor->shape[2] != 9)
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "link wrench tensor must have shape [N, L, 9]");

            const float* srcPtr = reinterpret_cast<const float*>(src.data);
            const bool isGpu = (src.device >= 0);

            if (N < 0 || L < 0)
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "invalid link wrench tensor shape");

            if (L != 0 && N > (std::numeric_limits<int64_t>::max() / L))
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "link wrench tensor too large");

            const int64_t totalElements = N * L;
            if (totalElements > static_cast<int64_t>(std::numeric_limits<size_t>::max() / (9 * sizeof(float))))
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "link wrench tensor too large");

            // Allocate temporary SoA buffer (forces|torques|positions)
            const size_t soaSize = static_cast<size_t>(totalElements) * 9 * sizeof(float);
            
            if (isGpu)
            {
                auto* cuda = getOptionalCuda();
                const uintptr_t srcDev = reinterpret_cast<uintptr_t>(src.data);
                const uintptr_t cudaCtx = binding.simView ? reinterpret_cast<uintptr_t>(binding.simView->getCudaContext()) : 0;
                ovphysx_result_t scratchResult = ensureWrenchScratchGpu(binding, cuda, cudaCtx, soaSize, "link wrench conversion");
                if (scratchResult.status != OVPHYSX_API_SUCCESS)
                    return scratchResult;

                ovphysx_result_t convResult = convertWrenchAoSToSoaGpu(
                    cuda, binding.wrenchSoaScratchDev, srcDev, totalElements, cudaCtx, "link wrench conversion");
                if (convResult.status != OVPHYSX_API_SUCCESS)
                    return convResult;

                auto soa = buildWrenchSoaDescs(src, reinterpret_cast<void*>(binding.wrenchSoaScratchDev), totalElements, N, L);
                ok = binding.artiView->applyForcesAndTorquesAtPosition(
                    &soa.force, &soa.torque, &soa.position, idxPtr, /*isGlobal=*/true);
                break;
            }
            else
            {
                // CPU path
                std::unique_ptr<float[]> soaBuffer(new float[static_cast<size_t>(totalElements) * 9]);
                ovphysx::internal::convertWrenchAosToSoaCpu(srcPtr, totalElements, soaBuffer.get());
                
                auto soa = buildWrenchSoaDescs(src, soaBuffer.get(), totalElements, N, L);
                ok = binding.artiView->applyForcesAndTorquesAtPosition(
                    &soa.force, &soa.torque, &soa.position, idxPtr, /*isGlobal=*/true);
            }
            
            break;
        }

        default:
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "unsupported tensor type");
    }

    if (!ok)
        return set_error(OVPHYSX_API_ERROR, "TensorAPI write failed");

    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_write_tensor_binding(
    ovphysx_handle_t handle,
    ovphysx_tensor_binding_handle_t binding_handle,
    const DLTensor* src_tensor,
    const DLTensor* index_tensor)
{
    if (!src_tensor)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "src_tensor is NULL");
    if (!src_tensor->shape)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "src_tensor has null shape");

    ovphysx_result_t processValidation = validateProcessTensorDevice(src_tensor, "write_tensor_binding");
    if (processValidation.status != OVPHYSX_API_SUCCESS)
        return processValidation;
    processValidation = validateProcessTensorDevice(index_tensor, "write_tensor_binding (index)");
    if (processValidation.status != OVPHYSX_API_SUCCESS)
        return processValidation;

    omni_sdk_physx_wait_all_pending_internal(handle);

    // Auto-warmup if needed (GPU mode requires one simulation step to populate buffers)
    {
        ovphysx_result_t warmup_result = ovphysx_gpu_warmup_if_needed(handle, /*is_explicit_call=*/false);
        if (warmup_result.status != OVPHYSX_API_SUCCESS)
            return warmup_result;
    }

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    return write_tensor_binding_locked(instance, binding_handle, src_tensor, index_tensor);
}

OVPHYSX_API ovphysx_result_t ovphysx_write_tensor_binding_masked(
    ovphysx_handle_t handle,
    ovphysx_tensor_binding_handle_t binding_handle,
    const DLTensor* src_tensor,
    const DLTensor* mask_tensor)
{
    if (!src_tensor)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "src_tensor is NULL");
    if (!src_tensor->shape)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "src_tensor has null shape");
    if (!mask_tensor)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "mask_tensor is NULL");
    if (!mask_tensor->shape)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "mask_tensor has null shape");

    ovphysx_result_t processValidation = validateProcessTensorDevice(src_tensor, "write_tensor_binding_masked");
    if (processValidation.status != OVPHYSX_API_SUCCESS)
        return processValidation;
    processValidation = validateProcessTensorDevice(mask_tensor, "write_tensor_binding_masked (mask)");
    if (processValidation.status != OVPHYSX_API_SUCCESS)
        return processValidation;

    // Validate mask dtype: must be bool (kDLBool, bits=8) or uint8 (kDLUInt, bits=8)
    const bool isBoolMask = (mask_tensor->dtype.code == kDLBool && mask_tensor->dtype.bits == 8 && mask_tensor->dtype.lanes == 1);
    const bool isUint8Mask = (mask_tensor->dtype.code == kDLUInt && mask_tensor->dtype.bits == 8 && mask_tensor->dtype.lanes == 1);
    if (!isBoolMask && !isUint8Mask)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "mask_tensor must be bool (kDLBool, bits=8) or uint8 (kDLUInt, bits=8)");

    // Validate mask is 1D
    if (mask_tensor->ndim != 1)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "mask_tensor must be 1D");

    omni_sdk_physx_wait_all_pending_internal(handle);

    // Auto-warmup if needed (GPU mode requires one simulation step to populate buffers)
    {
        ovphysx_result_t warmup_result = ovphysx_gpu_warmup_if_needed(handle, /*is_explicit_call=*/false);
        if (warmup_result.status != OVPHYSX_API_SUCCESS)
            return warmup_result;
    }

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    // CRITICAL: Hold lock during entire operation to prevent use-after-free if binding
    // is destroyed by another thread. TensorAPI views are NOT ref-counted.
    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);

    auto it = instance->tensor_bindings.find(binding_handle);
    if (it == instance->tensor_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "binding not found");

    TensorBindingState& binding = it->second;

    if (instance->attachedStageId != binding.stageId)
    {
        return set_error(OVPHYSX_API_NOT_FOUND,
                "binding invalidated (stage changed): binding.stageId=" + std::to_string(binding.stageId) +
                           " current.attachedStageId=" + std::to_string(instance->attachedStageId) +
                           "; recreate binding");
    }

    int32_t bindingNdim = 0;
    int64_t bindingShape[4];
    if (!getBindingSpec(binding, bindingNdim, bindingShape))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "unsupported tensor type");

    // Validate src_tensor: full shape [N,...] matching spec exactly
    ovphysx_result_t validation = validateTensorShape(src_tensor, binding, "write_tensor_binding_masked");
    if (validation.status != OVPHYSX_API_SUCCESS)
        return validation;

    // Validate mask shape[0] == binding's first dimension N
    if (mask_tensor->shape[0] != bindingShape[0])
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "mask_tensor.shape[0] must match binding element count N");

    // If the binding describes zero elements (zero matched prims OR any inner
    // dim is zero), treat writes as a no-op success. See bindingHasZeroElements.
    if (bindingHasZeroElements(bindingNdim, bindingShape))
        return success();


    omni::physics::tensors::TensorDesc src{};
    DLConvertError srcErr = dlToTensorDesc(src_tensor, src);
    if (srcErr != DLConvertError::Success)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, dlConvertErrorMessage(srcErr));

    omni::physics::tensors::TensorDesc mask{};
    DLConvertError maskErr = dlToTensorDesc(mask_tensor, mask);
    if (maskErr != DLConvertError::Success)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, dlConvertErrorMessage(maskErr));

    // Cross-device staging for both src and mask (mirrors write_tensor_binding).
    // After staging, src.data / mask.data point at the binding's device — for
    // CPU-only property types that means CPU, so the mask scan below operates
    // on a CPU-resident buffer regardless of the caller's original mask device.
    int bindingDeviceOrdinal = -1;
    if (!isCpuOnlyTensorType(binding.tensorType) && binding.simView)
        bindingDeviceOrdinal = binding.simView->getDeviceOrdinal();

    TensorStagingInfo srcStaging;
    {
        ovphysx_result_t r = stageTensorForWrite(
            src_tensor, bindingDeviceOrdinal, src, srcStaging, "write_tensor_binding_masked");
        if (r.status != OVPHYSX_API_SUCCESS)
            return r;
    }

    TensorStagingInfo maskStaging;
    {
        ovphysx_result_t r = stageTensorForWrite(
            mask_tensor, bindingDeviceOrdinal, mask, maskStaging, "write_tensor_binding_masked (mask)");
        if (r.status != OVPHYSX_API_SUCCESS)
            return r;
    }

    // CPU-only property types (DOF stiffness/damping/limits, body mass/COM/inertia):
    // the GPU view's masked setters expect GPU masks (they run a device-side compaction
    // kernel). Stage above made mask CPU-resident regardless of caller's original
    // device, so we can convert mask → CPU index array and forward to the indexed
    // write path. The forwarded call passes the original src_tensor; if it was on
    // the wrong device, write_tensor_binding_locked will re-stage it (the wasted
    // copy is acceptable for this corner case — CPU-only property + GPU mask is
    // unusual).
    if (isCpuOnlyTensorType(binding.tensorType))
    {
        const uint8_t* maskData = static_cast<const uint8_t*>(mask.data);
        const int64_t N = mask_tensor->shape[0];

        int64_t K = 0;
        for (int64_t i = 0; i < N; ++i)
            if (maskData[i]) ++K;

        if (K == 0)
            return success();

        if (K == N)
            return write_tensor_binding_locked(instance, binding_handle, src_tensor, nullptr);

        std::vector<int32_t> cpuIndices(K);
        int64_t j = 0;
        for (int64_t i = 0; i < N; ++i)
            if (maskData[i]) cpuIndices[j++] = static_cast<int32_t>(i);

        int64_t idxShape[1] = {K};
        DLTensor idx_dl{};
        idx_dl.data = cpuIndices.data();
        idx_dl.device.device_type = kDLCPU;
        idx_dl.device.device_id = 0;
        idx_dl.ndim = 1;
        idx_dl.dtype.code = kDLInt;
        idx_dl.dtype.bits = 32;
        idx_dl.dtype.lanes = 1;
        idx_dl.shape = idxShape;
        idx_dl.strides = nullptr;
        idx_dl.byte_offset = 0;

        return write_tensor_binding_locked(instance, binding_handle, src_tensor, &idx_dl);
    }

    bool ok = false;
    switch (binding.tensorType)
    {
        // Rigid body
        case OVPHYSX_TENSOR_RIGID_BODY_POSE_F32:
            ok = binding.rbView && binding.rbView->setTransformsMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32:
            ok = binding.rbView && binding.rbView->setVelocitiesMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_ACCELERATION_F32:
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "rigid body acceleration is read-only");

        // Articulation root
        case OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32:
            ok = binding.artiView && binding.artiView->setRootTransformsMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32:
            ok = binding.artiView && binding.artiView->setRootVelocitiesMasked(&src, &mask);
            break;

        // Articulation links are read-only (no TensorAPI setter)
        case OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_LINK_VELOCITY_F32:
        case OVPHYSX_TENSOR_ARTICULATION_LINK_ACCELERATION_F32:
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "link poses/velocities/accelerations are read-only");

        case OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_WORLD_F32:
        case OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_LOCAL_F32:
        case OVPHYSX_TENSOR_ARTICULATION_CENTROIDAL_MOMENTUM_F32:
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "articulation mass center / centroidal momentum are read-only");

        // Articulation DOF
        case OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32:
            ok = binding.artiView && binding.artiView->setDofPositionsMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32:
            ok = binding.artiView && binding.artiView->setDofVelocitiesMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32:
            ok = binding.artiView && binding.artiView->setDofPositionTargetsMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32:
            ok = binding.artiView && binding.artiView->setDofVelocityTargetsMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32:
            ok = binding.artiView && binding.artiView->setDofActuationForcesMasked(&src, &mask);
            break;

        // DOF properties (masked write)
        case OVPHYSX_TENSOR_ARTICULATION_DOF_STIFFNESS_F32:
            ok = binding.artiView && binding.artiView->setDofStiffnessesMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_DAMPING_F32:
            ok = binding.artiView && binding.artiView->setDofDampingsMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_LIMIT_F32:
            ok = binding.artiView && binding.artiView->setDofLimitsMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_VELOCITY_F32:
            ok = binding.artiView && binding.artiView->setDofMaxVelocitiesMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_FORCE_F32:
            ok = binding.artiView && binding.artiView->setDofMaxForcesMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_ARMATURE_F32:
            ok = binding.artiView && binding.artiView->setDofArmaturesMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_MODEL_F32:
            ok = binding.artiView && binding.artiView->setDofDriveModelPropertiesMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_DOF_FRICTION_PROPERTIES_F32:
            ok = binding.artiView && binding.artiView->setDofFrictionPropertiesMasked(&src, &mask);
            break;

        // Body properties (masked write)
        case OVPHYSX_TENSOR_ARTICULATION_BODY_MASS_F32:
            ok = binding.artiView && binding.artiView->setMassesMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_BODY_COM_POSE_F32:
            ok = binding.artiView && binding.artiView->setCOMsMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_BODY_INERTIA_F32:
            ok = binding.artiView && binding.artiView->setInertiasMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL:
            ok = binding.artiView && binding.artiView->setDisableGravitiesMasked(&src, &mask);
            break;

        // Standalone rigid body properties (masked write)
        case OVPHYSX_TENSOR_RIGID_BODY_MASS_F32:
            ok = binding.rbView && binding.rbView->setMassesMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_INERTIA_F32:
            ok = binding.rbView && binding.rbView->setInertiasMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_COM_POSE_F32:
            ok = binding.rbView && binding.rbView->setCOMsMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_INV_MASS_F32:
        case OVPHYSX_TENSOR_RIGID_BODY_INV_INERTIA_F32:
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "rigid body inverse mass/inertia are read-only");
        case OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL:
            ok = binding.rbView && binding.rbView->setDisableSimulationsMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_DISABLE_GRAVITY_BOOL:
            ok = binding.rbView && binding.rbView->setDisableGravitiesMasked(&src, &mask);
            break;

        // Fixed tendon properties (masked write via batch setter)
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_STIFFNESS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_DAMPING_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_STIFFNESS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_REST_LENGTH_F32:
        case OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_OFFSET_F32:
        {
            if (!binding.artiView)
                return set_error(OVPHYSX_API_ERROR, "binding has no articulation view");

            ok = writeFixedTendonProperty(binding, src, nullptr, &mask);
            break;
        }

        // Spatial tendon properties (masked write via batch setter)
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_STIFFNESS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_DAMPING_F32:
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_LIMIT_STIFFNESS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_OFFSET_F32:
        {
            if (!binding.artiView)
                return set_error(OVPHYSX_API_ERROR, "binding has no articulation view");

            ok = writeSpatialTendonProperty(binding, src, nullptr, &mask);
            break;
        }

        // Rigid body shape-level properties (masked write)
        case OVPHYSX_TENSOR_RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION_F32:
            ok = binding.rbView && binding.rbView->setMaterialPropertiesMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_CONTACT_OFFSET_F32:
            ok = binding.rbView && binding.rbView->setContactOffsetsMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_RIGID_BODY_REST_OFFSET_F32:
            ok = binding.rbView && binding.rbView->setRestOffsetsMasked(&src, &mask);
            break;

        // Articulation shape-level properties (masked write)
        case OVPHYSX_TENSOR_ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION_F32:
            ok = binding.artiView && binding.artiView->setMaterialPropertiesMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_CONTACT_OFFSET_F32:
            ok = binding.artiView && binding.artiView->setContactOffsetsMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_ARTICULATION_REST_OFFSET_F32:
            ok = binding.artiView && binding.artiView->setRestOffsetsMasked(&src, &mask);
            break;

        // Deformable body state
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_POSITION_F32:
            ok = binding.defBodyView && binding.defBodyView->setSimulationNodalPositionsMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_VELOCITY_F32:
            ok = binding.defBodyView && binding.defBodyView->setSimulationNodalVelocitiesMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_KINEMATIC_TARGET_F32:
            ok = binding.defBodyView && binding.defBodyView->setSimulationNodalKinematicTargetsMasked(&src, &mask);
            break;

        // Surface deformable body state (masked write)
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_POSITION_F32:
            ok = binding.defBodyView && binding.defBodyView->setSimulationNodalPositionsMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_VELOCITY_F32:
            ok = binding.defBodyView && binding.defBodyView->setSimulationNodalVelocitiesMasked(&src, &mask);
            break;

        // Deformable material properties (masked write)
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_DYNAMIC_FRICTION_F32:
            ok = binding.defMatView && binding.defMatView->setDynamicFrictionMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_YOUNGS_MODULUS_F32:
            ok = binding.defMatView && binding.defMatView->setYoungsModulusMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_POISSONS_RATIO_F32:
            ok = binding.defMatView && binding.defMatView->setPoissonsRatioMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_ELASTICITY_DAMPING_F32:
            ok = binding.defMatView && binding.defMatView->setElasticityDampingMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_STIFFNESS_F32:
            ok = binding.defMatView && binding.defMatView->setBendingStiffnessMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_THICKNESS_F32:
            ok = binding.defMatView && binding.defMatView->setThicknessMasked(&src, &mask);
            break;
        case OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_DAMPING_F32:
            ok = binding.defMatView && binding.defMatView->setBendingDampingMasked(&src, &mask);
            break;

        // Read-only tensors
        case OVPHYSX_TENSOR_ARTICULATION_JACOBIAN_F32:
        case OVPHYSX_TENSOR_ARTICULATION_MASS_MATRIX_F32:
        case OVPHYSX_TENSOR_ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_GRAVITY_FORCE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_LINK_INCOMING_JOINT_FORCE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_PROJECTED_JOINT_FORCE_F32:
        case OVPHYSX_TENSOR_ARTICULATION_BODY_INV_MASS_F32:
        case OVPHYSX_TENSOR_ARTICULATION_BODY_INV_INERTIA_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_REST_NODAL_POSITION_F32:
        case OVPHYSX_TENSOR_DEFORMABLE_SIM_ELEMENT_INDICES_S32:
        case OVPHYSX_TENSOR_DEFORMABLE_COLLISION_ELEMENT_INDICES_S32:
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_REST_POSITION_F32:
        case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_ELEMENT_INDICES_S32:
        case OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_TYPE_U8:
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "tensor type is read-only");

        // External forces - rigid body [N, 3] at center of mass
        case OVPHYSX_TENSOR_RIGID_BODY_FORCE_F32:
            ok = binding.rbView && binding.rbView->applyForcesMasked(&src, &mask);
            break;

        // External wrenches - rigid body [N, 9]
        // User provides a combined wrench per body: each row is [fx,fy,fz,tx,ty,tz,px,py,pz].
        // This matches the convention used by other physics APIs (e.g. Newton uses a combined
        // spatial_vector [fx,fy,fz,tx,ty,tz] per body). However, PhysX's DirectGPU API
        // (applyForcesAndTorquesAtPosition) takes three separate [N,3] tensors (force, torque,
        // position), so we need to convert from AoS to SoA here. Newton avoids this because
        // Warp natively consumes spatial vectors; we pay the conversion cost for PhysX.
        case OVPHYSX_TENSOR_RIGID_BODY_WRENCH_F32:
        {
            if (!binding.rbView)
                return set_error(OVPHYSX_API_ERROR, "binding has no rigid body view");

            if (src_tensor->ndim != 2)
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "wrench tensor must have shape [N, 9]");

            const int64_t N = src_tensor->shape[0];
            if (src_tensor->shape[1] != 9)
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "wrench tensor must have shape [N, 9]");

            const float* srcPtr = reinterpret_cast<const float*>(src.data);
            const bool isGpu = (src.device >= 0);

            if (N < 0)
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "invalid wrench tensor shape");

            if (N > static_cast<int64_t>(std::numeric_limits<size_t>::max() / (9 * sizeof(float))))
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "wrench tensor too large");

            const size_t soaSize = static_cast<size_t>(N) * 9 * sizeof(float);

            if (isGpu)
            {
                auto* cuda = getOptionalCuda();
                const uintptr_t srcDev = reinterpret_cast<uintptr_t>(src.data);
                const uintptr_t cudaCtx = binding.simView ? reinterpret_cast<uintptr_t>(binding.simView->getCudaContext()) : 0;
                ovphysx_result_t scratchResult = ensureWrenchScratchGpu(binding, cuda, cudaCtx, soaSize, "masked wrench conversion");
                if (scratchResult.status != OVPHYSX_API_SUCCESS)
                    return scratchResult;

                ovphysx_result_t convResult = convertWrenchAoSToSoaGpu(
                    cuda, binding.wrenchSoaScratchDev, srcDev, N, cudaCtx, "masked wrench conversion");
                if (convResult.status != OVPHYSX_API_SUCCESS)
                    return convResult;

                auto soa = buildWrenchSoaDescs(src, reinterpret_cast<void*>(binding.wrenchSoaScratchDev), N, N);
                ok = binding.rbView->applyForcesAndTorquesAtPositionMasked(
                    &soa.force, &soa.torque, &soa.position, &mask, /*isGlobal=*/true);
                break;
            }
            else
            {
                // CPU path
                std::unique_ptr<float[]> soaBuffer(new float[static_cast<size_t>(N) * 9]);
                ovphysx::internal::convertWrenchAosToSoaCpu(srcPtr, N, soaBuffer.get());

                auto soa = buildWrenchSoaDescs(src, soaBuffer.get(), N, N);
                ok = binding.rbView->applyForcesAndTorquesAtPositionMasked(
                    &soa.force, &soa.torque, &soa.position, &mask, /*isGlobal=*/true);
            }

            break;
        }

        // External wrenches - articulation links [N, L, 9]
        // User provides standard row-major layout: each element is [fx,fy,fz,tx,ty,tz,px,py,pz]
        // We convert internally to three separate [N,L,3] tensors for TensorAPI
        case OVPHYSX_TENSOR_ARTICULATION_LINK_WRENCH_F32:
        {
            if (!binding.artiView)
                return set_error(OVPHYSX_API_ERROR, "binding has no articulation view");

            if (src_tensor->ndim != 3)
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "link wrench tensor must have shape [N, L, 9]");

            const int64_t N = src_tensor->shape[0];
            const int64_t L = src_tensor->shape[1];
            if (src_tensor->shape[2] != 9)
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "link wrench tensor must have shape [N, L, 9]");

            const float* srcPtr = reinterpret_cast<const float*>(src.data);
            const bool isGpu = (src.device >= 0);

            if (N < 0 || L < 0)
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "invalid link wrench tensor shape");

            if (L != 0 && N > (std::numeric_limits<int64_t>::max() / L))
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "link wrench tensor too large");

            const int64_t totalElements = N * L;
            if (totalElements > static_cast<int64_t>(std::numeric_limits<size_t>::max() / (9 * sizeof(float))))
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "link wrench tensor too large");

            const size_t soaSize = static_cast<size_t>(totalElements) * 9 * sizeof(float);

            if (isGpu)
            {
                auto* cuda = getOptionalCuda();
                const uintptr_t srcDev = reinterpret_cast<uintptr_t>(src.data);
                const uintptr_t cudaCtx = binding.simView ? reinterpret_cast<uintptr_t>(binding.simView->getCudaContext()) : 0;
                ovphysx_result_t scratchResult = ensureWrenchScratchGpu(binding, cuda, cudaCtx, soaSize, "masked link wrench conversion");
                if (scratchResult.status != OVPHYSX_API_SUCCESS)
                    return scratchResult;

                ovphysx_result_t convResult = convertWrenchAoSToSoaGpu(
                    cuda, binding.wrenchSoaScratchDev, srcDev, totalElements, cudaCtx, "masked link wrench conversion");
                if (convResult.status != OVPHYSX_API_SUCCESS)
                    return convResult;

                auto soa = buildWrenchSoaDescs(src, reinterpret_cast<void*>(binding.wrenchSoaScratchDev), totalElements, N, L);
                ok = binding.artiView->applyForcesAndTorquesAtPositionMasked(
                    &soa.force, &soa.torque, &soa.position, &mask, /*isGlobal=*/true);
                break;
            }
            else
            {
                // CPU path
                std::unique_ptr<float[]> soaBuffer(new float[static_cast<size_t>(totalElements) * 9]);
                ovphysx::internal::convertWrenchAosToSoaCpu(srcPtr, totalElements, soaBuffer.get());

                auto soa = buildWrenchSoaDescs(src, soaBuffer.get(), totalElements, N, L);
                ok = binding.artiView->applyForcesAndTorquesAtPositionMasked(
                    &soa.force, &soa.torque, &soa.position, &mask, /*isGlobal=*/true);
            }

            break;
        }

        default:
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "unsupported tensor type for masked write");
    }

    if (!ok)
        return set_error(OVPHYSX_API_ERROR, "TensorAPI masked write failed");

    return success();
}

// ---------------------------------------------------------------------------
// Articulation metadata queries
// ---------------------------------------------------------------------------

// OMPE-94459 (#13): classify a prim by high-level TensorAPI object type.
// Creates a transient simulation view (no binding needed) and queries
// ISimulationView::getObjectType. Unresolved paths yield INVALID with a
// SUCCESS status -- "this isn't a known sim object" is not an error.
// OMPE-94459 (_KINEMATIC_UPDATE_NOOP fix): explicitly propagate root + DOF
// state into the link buffer for every articulation in the binding by calling
// PhysX SDK's PxArticulationReducedCoordinate::updateKinematic. The umbrella's
// OvPhysxSimulationView.update_articulations_kinematic now routes through
// here instead of being a no-op.
OVPHYSX_API ovphysx_result_t ovphysx_articulation_update_kinematic(
    ovphysx_handle_t handle,
    ovphysx_tensor_binding_handle_t binding_handle,
    uint32_t flags)
{
    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->tensor_bindings.find(binding_handle);
    if (it == instance->tensor_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "binding not found");

    TensorBindingState& binding = it->second;
    // Stale-binding check: same pattern as the other binding ops in this
    // file. A binding created against a previous stage attach must not run
    // sidecar calls against the new stage's articulations.
    if (instance->attachedStageId != binding.stageId)
        return set_error(OVPHYSX_API_NOT_FOUND,
                         "binding invalidated (stage changed); recreate binding");
    if (!binding.artiView)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                         "update_kinematic requires an articulation tensor binding");

    auto sidecarFn = g_sidecarUpdateKinematic.load(std::memory_order_acquire);
    if (!sidecarFn)
        return set_error(OVPHYSX_API_ERROR, "internal sidecar update_kinematic not loaded");

    // Build the sidecar-side flag mask. Bit assignments match the values in
    // ovphysx_articulation_kinematic_flag_t (POSITION=0x1, VELOCITY=0x2);
    // the sidecar re-maps them to PxArticulationKinematicFlag.
    const uint32_t sidecarFlags = flags & (OVPHYSX_ARTICULATION_KINEMATIC_POSITION |
                                           OVPHYSX_ARTICULATION_KINEMATIC_VELOCITY);
    if (!sidecarFlags)
        return success();

    // Surface per-articulation sidecar failures: the sidecar returns false
    // for unresolvable / disposed articulations and silently swallowing that
    // would hide stale-state bugs that the stale-binding check above cannot
    // catch (e.g. an articulation removed from the live stage but the
    // binding is otherwise current).
    const uint32_t count = binding.artiView->getCount();
    bool allOk = true;
    for (uint32_t i = 0; i < count; ++i)
    {
        const char* path = binding.artiView->getUsdPrimPath(i);
        // A null/empty path is an unresolvable articulation -- exactly the
        // stale/disposed case this is meant to surface, so flag it instead of
        // silently skipping.
        if (!path || !*path)
        {
            allOk = false;
            continue;
        }
        if (!sidecarFn(path, sidecarFlags))
            allOk = false;
    }

    return allOk ? success()
                 : set_error(OVPHYSX_API_ERROR,
                             "update_kinematic failed for one or more articulations in the binding");
}

// Wake rigid bodies in a binding (optionally a subset via indices).
// Mirrors PhysX SDK PxRigidDynamic::wakeUp. Bodies with eDISABLE_SIMULATION
// set are silently skipped (the engine refuses to wake disabled actors).
// Pair with a RIGID_BODY_DISABLE_SIMULATION clear write: clearing the flag
// re-adds the actor to the simulation in a sleep state, and this call brings
// it back active before the next simulate.
OVPHYSX_API ovphysx_result_t ovphysx_rigid_body_view_wake_up(
    ovphysx_handle_t handle,
    ovphysx_tensor_binding_handle_t binding_handle,
    const DLTensor* indices)
{
    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->tensor_bindings.find(binding_handle);
    if (it == instance->tensor_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "binding not found");

    TensorBindingState& binding = it->second;
    // Stale-binding check: same pattern as other binding ops in this file.
    if (instance->attachedStageId != binding.stageId)
        return set_error(OVPHYSX_API_NOT_FOUND,
                         "binding invalidated (stage changed); recreate binding");
    if (!binding.rbView)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                         "wake_up requires a rigid-body tensor binding");

    // The engine accepts a null index tensor to wake every body in the view.
    omni::physics::tensors::TensorDesc idxDesc{};
    const omni::physics::tensors::TensorDesc* idxPtr = nullptr;
    std::vector<int32_t> idxHostBuf; // backs staged GPU indices; must outlive idxPtr
    if (indices && indices->data)
    {
        // BaseRigidBodyView::wakeUp dereferences the index tensor as a host
        // pointer. GpuRigidBodyView stages GPU index tensors to host
        // internally, but a CPU-simulation binding forwards straight to
        // BaseRigidBodyView and would read a GPU pointer as host memory.
        // Validate the layout, then stage GPU indices DtoH here so every
        // backend is safe (a redundant stage for GPU-sim + GPU indices is a
        // no-op there). int32 matches the engine's PxU32 index reads.
        if (!indices->shape || indices->ndim != 1 || indices->dtype.code != kDLInt ||
            indices->dtype.bits != 32 || indices->dtype.lanes != 1)
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "wake_up: indices must be a 1D int32 tensor");
        if (indices->shape[0] > static_cast<int64_t>(binding.rbView->getCount()))
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "wake_up: indices length exceeds rigid-body count");

        const DLConvertError err = dlToTensorDesc(indices, idxDesc);
        if (err != DLConvertError::Success)
            return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                             "wake_up: failed to convert indices DLTensor");

        if (idxDesc.device >= 0)
        {
            omni::physx::IOptionalCuda* cuda = getOptionalCuda();
            if (!cuda || !cuda->cudaAvailable())
                return set_error(OVPHYSX_API_GPU_NOT_AVAILABLE,
                                 "wake_up: GPU index tensor requires CUDA for host staging");
            const size_t byteCount = static_cast<size_t>(indices->shape[0]) * sizeof(int32_t);
            idxHostBuf.resize(static_cast<size_t>(indices->shape[0]));
            int cudaStatus = 0;
            if (!cuda->memcpyDtoH(idxHostBuf.data(), reinterpret_cast<uintptr_t>(idxDesc.data), byteCount,
                                  &cudaStatus))
            {
                std::ostringstream oss;
                oss << "wake_up: memcpyDtoH failed staging GPU indices (cuda_status=" << cudaStatus << ")";
                return set_error(OVPHYSX_API_ERROR, oss.str());
            }
            idxDesc.data = idxHostBuf.data();
            idxDesc.device = -1;
        }
        idxPtr = &idxDesc;
    }
    const bool ok = binding.rbView->wakeUp(idxPtr);
    return ok ? success()
              : set_error(OVPHYSX_API_ERROR, "wake_up failed for one or more bodies");
}

// Mirrors PhysX SDK PxRigidDynamic::putToSleep. Symmetric counterpart to
// ovphysx_rigid_body_view_wake_up.
OVPHYSX_API ovphysx_result_t ovphysx_rigid_body_view_sleep(
    ovphysx_handle_t handle,
    ovphysx_tensor_binding_handle_t binding_handle,
    const DLTensor* indices)
{
    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->tensor_bindings.find(binding_handle);
    if (it == instance->tensor_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "binding not found");

    TensorBindingState& binding = it->second;
    if (instance->attachedStageId != binding.stageId)
        return set_error(OVPHYSX_API_NOT_FOUND,
                         "binding invalidated (stage changed); recreate binding");
    if (!binding.rbView)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                         "sleep requires a rigid-body tensor binding");

    omni::physics::tensors::TensorDesc idxDesc{};
    const omni::physics::tensors::TensorDesc* idxPtr = nullptr;
    std::vector<int32_t> idxHostBuf;
    if (indices && indices->data)
    {
        if (!indices->shape || indices->ndim != 1 || indices->dtype.code != kDLInt ||
            indices->dtype.bits != 32 || indices->dtype.lanes != 1)
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "sleep: indices must be a 1D int32 tensor");
        if (indices->shape[0] > static_cast<int64_t>(binding.rbView->getCount()))
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "sleep: indices length exceeds rigid-body count");

        const DLConvertError err = dlToTensorDesc(indices, idxDesc);
        if (err != DLConvertError::Success)
            return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                             "sleep: failed to convert indices DLTensor");

        if (idxDesc.device >= 0)
        {
            omni::physx::IOptionalCuda* cuda = getOptionalCuda();
            if (!cuda || !cuda->cudaAvailable())
                return set_error(OVPHYSX_API_GPU_NOT_AVAILABLE,
                                 "sleep: GPU index tensor requires CUDA for host staging");
            const size_t byteCount = static_cast<size_t>(indices->shape[0]) * sizeof(int32_t);
            idxHostBuf.resize(static_cast<size_t>(indices->shape[0]));
            int cudaStatus = 0;
            if (!cuda->memcpyDtoH(idxHostBuf.data(), reinterpret_cast<uintptr_t>(idxDesc.data), byteCount,
                                  &cudaStatus))
            {
                std::ostringstream oss;
                oss << "sleep: memcpyDtoH failed staging GPU indices (cuda_status=" << cudaStatus << ")";
                return set_error(OVPHYSX_API_ERROR, oss.str());
            }
            idxDesc.data = idxHostBuf.data();
            idxDesc.device = -1;
        }
        idxPtr = &idxDesc;
    }
    const bool ok = binding.rbView->putToSleep(idxPtr);
    return ok ? success()
              : set_error(OVPHYSX_API_ERROR, "sleep failed for one or more bodies");
}

OVPHYSX_API ovphysx_result_t ovphysx_get_object_type(
    ovphysx_handle_t handle,
    ovphysx_string_t prim_path,
    ovphysx_object_type_t* out_type)
{
    if (!out_type)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_type is NULL");
    *out_type = OVPHYSX_OBJECT_TYPE_INVALID;
    if (!isValid(prim_path))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "prim_path is NULL or empty");
    if (hasEmbeddedNul(prim_path))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "prim_path contains an embedded NUL byte");

    const std::string pathStr = toStdString(prim_path);

    omni_sdk_physx_wait_all_pending_internal(handle);

    // TensorAPI's createSimulationView needs the initial scene parse to
    // have happened (physxSim->simulate(0,0) + fetchResults), otherwise it
    // returns an invalid view and we report "failed to create simulation
    // view". Callers reaching this from on_start (before any explicit
    // step) hit that path. The other tensor-binding entry points
    // (create_tensor_binding, update_articulations_kinematic) call this
    // helper for the same reason; mirror the pattern here so callers
    // don't have to step first to classify a prim.
    {
        ovphysx_api_status_t attach_status = ovphysx_ensure_physics_attached(handle);
        if (attach_status != OVPHYSX_API_SUCCESS)
            return set_error(attach_status, "failed to attach physics stage");
    }

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance || instance->attachedStageId == 0)
        return set_error(OVPHYSX_API_ERROR, "no USD stage attached");

    auto* tensorApi = getTensorApi();
    if (!tensorApi)
        return set_error(OVPHYSX_API_ERROR, "TensorAPI unavailable");

    omni::physics::tensors::ISimulationView* simView =
        tensorApi->createSimulationView(instance->attachedStageId);
    if (!simView || !simView->getValid())
    {
        if (simView) simView->release(false);
        return set_error(OVPHYSX_API_ERROR, "failed to create simulation view");
    }

    const omni::physics::tensors::ObjectType ot = simView->getObjectType(pathStr.c_str());
    simView->release(false);

    switch (ot)
    {
        case omni::physics::tensors::ObjectType::eRigidBody:
            *out_type = OVPHYSX_OBJECT_TYPE_RIGID_BODY; break;
        case omni::physics::tensors::ObjectType::eArticulation:
            *out_type = OVPHYSX_OBJECT_TYPE_ARTICULATION; break;
        case omni::physics::tensors::ObjectType::eArticulationLink:
            *out_type = OVPHYSX_OBJECT_TYPE_ARTICULATION_LINK; break;
        case omni::physics::tensors::ObjectType::eArticulationRootLink:
            *out_type = OVPHYSX_OBJECT_TYPE_ARTICULATION_ROOT_LINK; break;
        case omni::physics::tensors::ObjectType::eArticulationJoint:
            *out_type = OVPHYSX_OBJECT_TYPE_ARTICULATION_JOINT; break;
        default:
            *out_type = OVPHYSX_OBJECT_TYPE_INVALID; break;
    }
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_get_articulation_metadata(
    ovphysx_handle_t handle,
    ovphysx_tensor_binding_handle_t binding_handle,
    ovphysx_articulation_metadata_t* out_metadata)
{
    if (!out_metadata)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_metadata is NULL");

    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->tensor_bindings.find(binding_handle);
    if (it == instance->tensor_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "binding not found");

    const TensorBindingState& binding = it->second;
    if (!binding.artiView)
        return set_error(OVPHYSX_API_ERROR, "binding is not an articulation binding");

    const auto* metatype = binding.artiView->getSharedMetatype();
    if (!metatype)
        return set_error(OVPHYSX_API_ERROR, "heterogeneous or empty view; metadata unavailable");

    out_metadata->dof_count            = static_cast<int32_t>(metatype->getDofCount());
    out_metadata->body_count           = static_cast<int32_t>(metatype->getLinkCount());
    out_metadata->joint_count          = static_cast<int32_t>(metatype->getJointCount());
    out_metadata->is_fixed_base        = metatype->getFixedBase();
    out_metadata->fixed_tendon_count   = static_cast<int32_t>(binding.artiView->getMaxFixedTendons());
    out_metadata->spatial_tendon_count = static_cast<int32_t>(binding.artiView->getMaxSpatialTendons());

    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_articulation_get_dof_names(
    ovphysx_handle_t handle,
    ovphysx_tensor_binding_handle_t binding_handle,
    ovphysx_string_t* out_names,
    uint32_t max_names,
    uint32_t* out_count)
{
    if (!out_names || !out_count)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_names or out_count is NULL");

    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->tensor_bindings.find(binding_handle);
    if (it == instance->tensor_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "binding not found");

    const TensorBindingState& binding = it->second;
    if (!binding.artiView)
        return set_error(OVPHYSX_API_ERROR, "binding is not an articulation binding");

    const auto* metatype = binding.artiView->getSharedMetatype();
    if (!metatype)
        return set_error(OVPHYSX_API_ERROR, "heterogeneous or empty view; metadata unavailable");

    const uint32_t count = metatype->getDofCount();
    const uint32_t toWrite = (count < max_names) ? count : max_names;
    for (uint32_t i = 0; i < toWrite; ++i)
    {
        const char* name = metatype->getDofName(i);
        out_names[i] = ovphysx_cstr(name);
    }
    *out_count = toWrite;
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_articulation_get_body_names(
    ovphysx_handle_t handle,
    ovphysx_tensor_binding_handle_t binding_handle,
    ovphysx_string_t* out_names,
    uint32_t max_names,
    uint32_t* out_count)
{
    if (!out_names || !out_count)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_names or out_count is NULL");

    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->tensor_bindings.find(binding_handle);
    if (it == instance->tensor_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "binding not found");

    const TensorBindingState& binding = it->second;
    if (!binding.artiView)
        return set_error(OVPHYSX_API_ERROR, "binding is not an articulation binding");

    const auto* metatype = binding.artiView->getSharedMetatype();
    if (!metatype)
        return set_error(OVPHYSX_API_ERROR, "heterogeneous or empty view; metadata unavailable");

    const uint32_t count = metatype->getLinkCount();
    const uint32_t toWrite = (count < max_names) ? count : max_names;
    for (uint32_t i = 0; i < toWrite; ++i)
    {
        const char* name = metatype->getLinkName(i);
        out_names[i] = ovphysx_cstr(name);
    }
    *out_count = toWrite;
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_articulation_get_joint_names(
    ovphysx_handle_t handle,
    ovphysx_tensor_binding_handle_t binding_handle,
    ovphysx_string_t* out_names,
    uint32_t max_names,
    uint32_t* out_count)
{
    if (!out_names || !out_count)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_names or out_count is NULL");

    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->tensor_bindings.find(binding_handle);
    if (it == instance->tensor_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "binding not found");

    const TensorBindingState& binding = it->second;
    if (!binding.artiView)
        return set_error(OVPHYSX_API_ERROR, "binding is not an articulation binding");

    const auto* metatype = binding.artiView->getSharedMetatype();
    if (!metatype)
        return set_error(OVPHYSX_API_ERROR, "heterogeneous or empty view; metadata unavailable");

    const uint32_t count = metatype->getJointCount();
    const uint32_t toWrite = (count < max_names) ? count : max_names;
    for (uint32_t i = 0; i < toWrite; ++i)
    {
        const char* name = metatype->getJointName(i);
        out_names[i] = ovphysx_cstr(name);
    }
    *out_count = toWrite;
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_tensor_binding_get_prim_paths(
    ovphysx_handle_t handle,
    ovphysx_tensor_binding_handle_t binding_handle,
    ovphysx_string_t* out_paths,
    uint32_t max_paths,
    uint32_t* out_count)
{
    if (!out_paths || !out_count)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_paths or out_count is NULL");

    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "invalid handle");

    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    auto it = instance->tensor_bindings.find(binding_handle);
    if (it == instance->tensor_bindings.end())
        return set_error(OVPHYSX_API_NOT_FOUND, "binding not found");

    TensorBindingState& binding = it->second;
    if (!binding.rbView && !binding.artiView && !binding.defBodyView)
    {
        // These are the same view-family predicates used at binding creation.
        // Missing views here mean a valid zero-count binding for that family.
        if (requiresRigidBodyView(binding.tensorType) || requiresArticulationView(binding.tensorType) ||
            requiresDeformableBodyView(binding.tensorType))
        {
            *out_count = 0;
            return success();
        }
        return set_error(OVPHYSX_API_ERROR, "binding does not expose prim path metadata");
    }

    if (instance->attachedStageId != binding.stageId)
        return set_error(OVPHYSX_API_NOT_FOUND, "binding invalidated (stage changed); recreate binding");

    std::vector<std::string>* cache = nullptr;
    uint32_t count = 0;
    if (binding.rbView)
    {
        count = binding.rbView->getCount();
        cache = &binding.rigidBodyPrimPathCache;
        if (cache->size() != count)
        {
            cache->clear();
            cache->reserve(count);
            for (uint32_t i = 0; i < count; ++i)
            {
                const char* path = binding.rbView->getUsdPrimPath(i);
                cache->emplace_back(path ? path : "");
            }
        }
    }
    else if (binding.artiView)
    {
        count = binding.artiView->getCount();
        cache = &binding.articulationPrimPathCache;
        if (cache->size() != count)
        {
            cache->clear();
            cache->reserve(count);
            for (uint32_t i = 0; i < count; ++i)
            {
                const char* path = binding.artiView->getUsdPrimPath(i);
                cache->emplace_back(path ? path : "");
            }
        }
    }
    else if (binding.defBodyView)
    {
        count = binding.defBodyView->getCount();
        cache = &binding.deformableBodyPrimPathCache;
        if (cache->size() != count)
        {
            cache->clear();
            cache->reserve(count);
            for (uint32_t i = 0; i < count; ++i)
            {
                const char* path = binding.defBodyView->getUsdPrimPath(i);
                cache->emplace_back(path ? path : "");
            }
        }
    }
    else
    {
        // Deformable material bindings intentionally fall here: IDeformableMaterialView
        // has no getUsdPrimPath() accessor, so prim path metadata is unavailable by design.
        return set_error(OVPHYSX_API_ERROR, "binding does not expose prim path metadata");
    }

    const uint32_t toWrite = (count < max_paths) ? count : max_paths;
    for (uint32_t i = 0; i < toWrite; ++i)
    {
        out_paths[i] = ovphysx_cstr((*cache)[i].c_str());
    }
    *out_count = toWrite;
    return success();
}

} // extern "C"
