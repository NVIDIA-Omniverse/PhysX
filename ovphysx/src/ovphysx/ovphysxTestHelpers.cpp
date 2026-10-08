// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ovphysxTestHelpers.h"

#include <omni/physx/PhysXRuntime.h>

#include <carb/Framework.h>
#include <carb/settings/ISettings.h>
#include <omni/physx/IOptionalCuda.h>
#include <omni/physics/tensors/ISimulationView.h>
#include <PxPhysicsAPI.h>

#include <array>
#include <new>

#include "internal/sdk/ovphysxSDK.hpp"
#include "internal/sidecar/ovphysxInternalInterop.h"

namespace {

struct PhysXExtensionsFixture
{
    physx::PxCustomGeometryExt::CylinderCallbacks cylinder{ 1.0f, 0.25f, 1, 0.01f };
    physx::PxCustomGeometryExt::ConeCallbacks cone{ 1.0f, 0.25f, 1, 0.01f };
    physx::PxMaterial* material{ nullptr };
    std::array<physx::PxRigidDynamic*, 2> actors{};
    std::array<physx::PxShape*, 2> shapes{};
    std::array<physx::PxJoint*, 8> joints{};

    bool create(physx::PxPhysics& physics)
    {
        using namespace physx;
        material = physics.createMaterial(0.5f, 0.5f, 0.0f);
        actors = {
            physics.createRigidDynamic(PxTransform(PxVec3(-0.5f, 0.0f, 0.0f))),
            physics.createRigidDynamic(PxTransform(PxVec3(0.5f, 0.0f, 0.0f))),
        };
        if (!material || !actors[0] || !actors[1])
            return false;

        shapes = {
            physics.createShape(PxCustomGeometry(cylinder), *material),
            physics.createShape(PxCustomGeometry(cone), *material),
        };
        if (!shapes[0] || !shapes[1])
            return false;

        const PxTransform frame(PxIdentity);
        joints = {
            PxFixedJointCreate(physics, actors[0], frame, actors[1], frame),
            PxPrismaticJointCreate(physics, actors[0], frame, actors[1], frame),
            PxRevoluteJointCreate(physics, actors[0], frame, actors[1], frame),
            PxSphericalJointCreate(physics, actors[0], frame, actors[1], frame),
            PxDistanceJointCreate(physics, actors[0], frame, actors[1], frame),
            PxD6JointCreate(physics, actors[0], frame, actors[1], frame),
            PxGearJointCreate(physics, actors[0], frame, actors[1], frame),
            PxRackAndPinionJointCreate(physics, actors[0], frame, actors[1], frame),
        };
        for (PxJoint* joint : joints)
        {
            if (!joint)
                return false;
        }

        auto* prismatic = static_cast<PxPrismaticJoint*>(joints[1]);
        auto* revolute = static_cast<PxRevoluteJoint*>(joints[2]);
        auto* d6 = static_cast<PxD6Joint*>(joints[5]);
        auto* gear = static_cast<PxGearJoint*>(joints[6]);
        auto* rackAndPinion = static_cast<PxRackAndPinionJoint*>(joints[7]);
        return gear->setHinges(revolute, d6) && rackAndPinion->setJoints(revolute, prismatic);
    }

    ~PhysXExtensionsFixture()
    {
        for (auto it = joints.rbegin(); it != joints.rend(); ++it)
        {
            if (*it)
                (*it)->release();
        }
        for (physx::PxShape* shape : shapes)
        {
            if (shape)
                shape->release();
        }
        for (physx::PxRigidDynamic* actor : actors)
        {
            if (actor)
                actor->release();
        }
        if (material)
            material->release();
    }
};

} // namespace

extern "C" {

OVPHYSX_API bool ovphysx_get_tensor_binding_cuda_context_internal(
    ovphysx_handle_t handle,
    ovphysx_tensor_binding_handle_t binding,
    uintptr_t* out_cuda_ctx)
{
    if (!out_cuda_ctx) return false;
    *out_cuda_ctx = 0;

    auto instance = get_instance(handle);
    if (!instance) return false;

    // NOTE: tensor_bindings is guarded by InstanceData::tensor_binding_mutex.
    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    const auto it = instance->tensor_bindings.find(binding);
    if (it == instance->tensor_bindings.end()) return false;

    const auto& b = it->second;
    if (!b.simView) return true;  // binding exists but has no simView (ctx remains 0)

    *out_cuda_ctx = reinterpret_cast<uintptr_t>(b.simView->getCudaContext());
    return true;
}

OVPHYSX_API void* ovphysx_get_optional_cuda_internal(void)
{
    return static_cast<void*>(omni::physx::runtime::tryGetOptionalCudaInterface());
}

OVPHYSX_API void* ovphysx_get_tensor_api_internal(void)
{
    return static_cast<void*>(omni::physx::runtime::tryGetTensorApiInterface());
}

OVPHYSX_API bool ovphysx_get_attach_cuda_selector_for_test_internal(int32_t* out_value)
{
    if (!out_value)
        return false;

    carb::Framework* framework = carb::getFramework();
    carb::settings::ISettings* settings =
        framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
    constexpr const char* cudaDevicePath = "/physics/cudaDevice";
    if (!settings || settings->getItemType(cudaDevicePath) != carb::dictionary::ItemType::eInt)
        return false;

    *out_value = settings->getAsInt(cudaDevicePath);
    return true;
}

OVPHYSX_API bool ovphysx_set_raw_string_setting_for_test_internal(
    const char* path,
    const char* value,
    size_t length)
{
    if (!path || (!value && length != 0))
        return false;
    carb::Framework* framework = carb::getFramework();
    carb::settings::ISettings* settings =
        framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
    if (!settings)
        return false;
    settings->setString(path, value, length);
    return true;
}

OVPHYSX_API ovphysx_test_set_viz_scope_tokens_fn
ovphysx_exchange_set_viz_scope_tokens_internal(ovphysx_test_set_viz_scope_tokens_fn replacement)
{
    return g_sidecarSetVizScopeTokens.exchange(replacement, std::memory_order_acq_rel);
}

OVPHYSX_API bool ovphysx_set_ovstage_attachment_state_internal(
    ovphysx_handle_t handle,
    bool attached,
    int64_t stage_id)
{
    std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance)
        return false;
    instance->ovstage_attached = attached;
    instance->attachedStageId = stage_id;
    // Attach identity is separate from the stage id (ADR-0013) and is what the
    // binding staleness checks compare, so a faked attach has to set it too.
    // Any nonzero value works here: the runtime never resolves this one.
    instance->attachHandle =
        attached ? omni::physics::tensors::AttachHandle(1) : omni::physics::tensors::kNoAttach;
    return true;
}

OVPHYSX_API bool ovphysx_create_extensions_fixture_for_test_internal(
    ovphysx_handle_t handle,
    void** out_fixture)
{
    if (!out_fixture)
        return false;
    *out_fixture = nullptr;

    void* physicsPtr = nullptr;
    if (ovphysx_get_physx_ptr(
            handle, { nullptr, 0 }, OVPHYSX_PHYSX_TYPE_PHYSICS, &physicsPtr).status != OVPHYSX_API_SUCCESS)
    {
        return false;
    }

    PhysXExtensionsFixture* fixture = new (std::nothrow) PhysXExtensionsFixture;
    if (!fixture || !fixture->create(*static_cast<physx::PxPhysics*>(physicsPtr)))
    {
        delete fixture;
        return false;
    }
    *out_fixture = fixture;
    return true;
}

OVPHYSX_API void ovphysx_destroy_extensions_fixture_for_test_internal(void* fixture)
{
    delete static_cast<PhysXExtensionsFixture*>(fixture);
}

} // extern "C"
