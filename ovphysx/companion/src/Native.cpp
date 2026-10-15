// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-NATIVE-ABI-001
// @covers AC-1 AC-2 AC-3 AC-4
// @implements REQ-COMP-NATIVE-SCHEMA-001
// @covers AC-3
// @implements REQ-COMP-JOINT-001
// @covers AC-1 AC-2 AC-3 AC-4
// @implements REQ-COMP-GEOMETRY-001
// @covers AC-1 AC-3 AC-4
// @implements REQ-CAPI-COMPANION-001
// @covers AC-4
// @implements REQ-COMP-OVSTAGE-001
// @covers AC-2
#include "ovphysx_companion/native.h"
#include "ovphysx_companion/native_sdk.h"
#include "internal/sdk/ovphysxSDK.hpp"
#include "internal/sdk/CompanionRuntime.hpp"
#include <omni/physx/IPhysxCustomJoint.h>
#include <omni/physx/NativeJointControl.h>
#include <omni/physx/IPhysxCustomGeometry.h>
#include <omni/physx/PhysXRuntime.h>
#include <carb/extras/Library.h>
#include <filesystem>
#include <map>
#include <cmath>
#include <cstring>
#include <limits>

namespace ovphysx::companion::native_modules
{
namespace sdk = ovphysx::companion::native;
namespace rt = omni::physx;
using Status = ovphysx_companion_status_t;
using Key = omni::physics::parse::ObjectKey;
using Id = std::pair<uint64_t, uint64_t>;
const sdk::HostApi host{sizeof(sdk::HostApi), 1, &physx::PxGeometryQuery::raycast,
    &physx::PxGeometryQuery::overlap, &physx::PxGeometryQuery::sweep};

struct Module
{
    CompanionServiceLease lease{companionRuntime()};
    std::shared_ptr<ovphysx::CarboniteLoader> services;
    carb::extras::LibraryHandle library = nullptr;
    const sdk::Plugin* plugin = nullptr;
    void* context = nullptr;
    size_t registration = 0;
    std::mutex objectsMutex;
    std::map<Id, void*> joints;
    explicit Module(std::shared_ptr<ovphysx::CarboniteLoader> value) : services(std::move(value))
    {}
    ~Module()
    {
        // Destruction is reachable only before registration or after retirement.
        if (context) plugin->stop(context);
        if (library) carb::extras::unloadLibrary(library);
        services.reset();

    }
    void* object(uint64_t owner, uint64_t key)
    {
        std::lock_guard<std::mutex> lock(objectsMutex);
        const std::map<std::pair<uint64_t, uint64_t>, void *>::iterator found = joints.find({owner, key});
        return found == joints.end() ? nullptr : found->second;
    }
    bool parameters(uint64_t owner, uint64_t key, std::vector<sdk::ParameterValue>& values)
    {
        values.resize(plugin->parameterCount);
        uint32_t invalid;
        return rt::native::copyParameters(owner, key, plugin->parameters,
            plugin->parameterCount, values.data(), invalid);
    }
};

bool validParameter(const sdk::ParameterSpec& spec)
{
    if (!spec.name || !spec.name[0] || spec.required > 1 || spec.kind != spec.fallback.kind || spec.fallback.reserved)
        return false;
    // StageFlow publishes output under companion:output:; an input there would
    // consume published values on a later reparse.
    static const char kOutputNamespace[] = "companion:output:";
    if (!std::strncmp(spec.name, kOutputNamespace, sizeof(kOutputNamespace) - 1))
        return false;
    const omni::physx::native::ParameterValue & v = spec.fallback;
    switch (spec.kind) {
    case sdk::ParameterKind::Float: return std::isfinite(v.scalar);
    case sdk::ParameterKind::Double: return std::isfinite(v.real);
    case sdk::ParameterKind::Int: return true;
    case sdk::ParameterKind::Bool: return v.boolean <= 1;
    case sdk::ParameterKind::Float3:
        return std::isfinite(v.vector[0]) && std::isfinite(v.vector[1]) && std::isfinite(v.vector[2]);
    }
    return false;
}
Status validate(const sdk::Plugin* p)
{
    if (!p || p->size != sizeof(*p) || p->version != 1 || !sdk::compatibleBuild(p->compatibility))
        return OVPHYSX_COMPANION_ABI_MISMATCH;
    if (!p->schema || !p->schema[0] || !p->start || !p->stop || p->reserved || p->capabilities ||
        p->parameterCount > 1024 || (p->parameterCount && !p->parameters)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    for (const char* s = p->schema; *s; ++s)
        if (!((*s >= 'a' && *s <= 'z') || (*s >= 'A' && *s <= 'Z') ||
              (*s >= '0' && *s <= '9') || *s == '_')) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    for (uint32_t i=0; i<p->parameterCount; ++i) {
        if (!validParameter(p->parameters[i])) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        for (uint32_t j=0; j<i; ++j)
            if (!std::strcmp(p->parameters[i].name, p->parameters[j].name)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    }
    if (p->kind == sdk::Kind::Joint) {
        const ovphysx::companion::native::JointCallbacks & j = p->joint;
        if (!j.create || !j.release || !j.prepare || !j.constantBlock || !j.comShift || !j.originShift ||
            !j.solverPrep || !j.dataSize || j.dataSize > 16368 || !j.dataAlignment || j.dataAlignment > 16 ||
            (j.dataAlignment & (j.dataAlignment-1))) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    } else if (p->kind == sdk::Kind::Geometry) {
        const ovphysx::companion::native::GeometryCallbacks & g = p->geometry;
        if (!g.create || !g.release || !g.bounds || !g.contacts || !g.mass || !g.manifold)
            return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    } else return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    return OVPHYSX_COMPANION_OK;
}

bool createJoint(Key key, rt::AttachHandle owner, physx::PxRigidActor* a, const physx::PxTransform& fa,
    physx::PxRigidActor* b, const physx::PxTransform& fb, rt::CustomJointFlag::Enum& flags, void* user)
{
    // v1 is an ordinary CPU rigid-constraint provider. Reject unsupported link
    // integration before parameters or any consumer callback is evaluated.
    if ((a && a->getType()==physx::PxActorType::eARTICULATION_LINK) ||
        (b && b->getType()==physx::PxActorType::eARTICULATION_LINK)) return false;
    Module & m = *static_cast<Module*>(user);
    void* object = nullptr;
    try {
        std::vector<sdk::ParameterValue> values;
        if (!m.parameters(owner, key.handle, values)) return false;
        uint32_t f = 0;
        const sdk::ObjectContext input{owner, key.handle, values.data(), uint32_t(values.size())};
        object = m.plugin->joint.create(input, a, fa, b, fb, f, m.context);
        if (!object) return false;
        const void * data = m.plugin->joint.constantBlock(object);
        if ((f & ~1u) || !data || reinterpret_cast<uintptr_t>(data) % m.plugin->joint.dataAlignment) {
            m.plugin->joint.release(object, m.context);
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(m.objectsMutex);
            if (!m.joints.emplace(Id{owner, key.handle}, object).second) {
                // A duplicate cannot occur under owner admission. Release outside the lock.
            } else {
                flags = rt::CustomJointFlag::Enum(f);
                return true;
            }
        }
        m.plugin->joint.release(object, m.context);
        return false;
    } catch (...) {
        if (object) m.plugin->joint.release(object, m.context);
        return false;
    }
}
void releaseJoint(Key key, rt::AttachHandle owner, void* user)
{
    Module & m = *static_cast<Module*>(user);
    void* object = nullptr;
    {
        std::lock_guard<std::mutex> lock(m.objectsMutex);
        const std::map<std::pair<uint64_t, uint64_t>, void *>::iterator found = m.joints.find({owner, key.handle});
        if (found == m.joints.end()) return;
        object = found->second;
        m.joints.erase(found);
    }
    m.plugin->joint.release(object, m.context);
}
void* prepareJoint(Key key, rt::AttachHandle owner, void* user)
{
    Module & m = *static_cast<Module*>(user);
    return m.plugin->joint.prepare(m.object(owner, key.handle));
}
const void* constantJoint(Key key, rt::AttachHandle owner, void* user)
{
    Module & m = *static_cast<Module*>(user);
    return m.plugin->joint.constantBlock(m.object(owner, key.handle));
}
void comJoint(Key key, rt::AttachHandle owner, uint32_t actor, void* user)
{
    Module & m = *static_cast<Module*>(user);
    m.plugin->joint.comShift(m.object(owner, key.handle), actor);
}
void originJoint(Key key, rt::AttachHandle owner, const physx::PxVec3& shift, void* user)
{
    Module & m = *static_cast<Module*>(user);
    m.plugin->joint.originShift(m.object(owner, key.handle), shift);
}
void* createGeometry(Key key, rt::AttachHandle owner, const physx::PxCustomGeometry::Type& type, void* user)
{
    Module & m = *static_cast<Module*>(user);
    try {
        std::vector<sdk::ParameterValue> values;
        if (!m.parameters(owner, key.handle, values)) return nullptr;
        const sdk::ObjectContext input{owner, key.handle, values.data(), uint32_t(values.size())};
        return m.plugin->geometry.create(input, type, m.context);
    } catch (...) { return nullptr; }
}
void releaseGeometry(Key, rt::AttachHandle, void* object, void* user)
{
    Module & m = *static_cast<Module*>(user);
    m.plugin->geometry.release(object, m.context);
}

Status load(uint64_t source, const char* path, uint64_t* output) noexcept
{
    if (!path || !output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    try {
        if (!std::filesystem::path(path).is_absolute()) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        CompanionOperationGuard admission(companionRuntime());
        if (!admission || rt::runtime::isRuntimeBusy(0)) return OVPHYSX_COMPANION_BUSY;
        // v1 is qualified only with hard CPU admission. Reject other execution
        // modes before loading code or invoking plugin startup/create callbacks.
        bool cpuOnly = false;
        if (ovphysx_get_cpu_mode(&cpuOnly).status != OVPHYSX_API_SUCCESS) return OVPHYSX_COMPANION_ERROR;
        if (!cpuOnly) return OVPHYSX_COMPANION_UNAVAILABLE;
        std::unique_lock<std::mutex> lock(companionRuntime().modules().mutex, std::try_to_lock);
        if (!lock.owns_lock()) return OVPHYSX_COMPANION_BUSY;
        std::shared_ptr<Module> m;
        {
            std::unique_lock<std::shared_mutex> instances(g_instances_mutex);
            const std::unordered_map<uint64_t, std::shared_ptr<InstanceData>>::iterator found = g_instances.find(source);
            if (found == g_instances.end()) return OVPHYSX_COMPANION_NOT_FOUND;
            if (!companionRuntime().admissionOpen() || found->second->companionDestroying) return OVPHYSX_COMPANION_BUSY;
            m = std::make_shared<Module>(found->second->carbonite);
        }
        if (!m->services) return OVPHYSX_COMPANION_UNAVAILABLE;
        m->library = carb::extras::loadLibrary(path, carb::extras::fLibFlagNow);
        if (!m->library) return OVPHYSX_COMPANION_NOT_FOUND;
        m->plugin = carb::extras::getLibrarySymbol<const sdk::Plugin*>(m->library, "ovphysx_companion_plugin_v1");
        const Status valid = validate(m->plugin);
        if (valid != OVPHYSX_COMPANION_OK) return valid;
        // Preallocate handle entry so native registration is the final throwing step.
        if (companionRuntime().modules().nextHandle == UINT64_MAX) return OVPHYSX_COMPANION_ERROR;
        const uint64_t handle = companionRuntime().modules().nextHandle++;
        companionRuntime().modules().entries.emplace(handle, m);
        struct Rollback {
            uint64_t handle;
            bool committed = false;
            ~Rollback() { if (!committed) companionRuntime().modules().entries.erase(handle); }
        } rollback{handle};
        m->context = m->plugin->start(host);
        if (!m->context) return OVPHYSX_COMPANION_ERROR;
        if (m->plugin->kind == sdk::Kind::Joint) {
            rt::ICustomJointCallback cb;
            cb.createJointFn = createJoint; cb.releaseJointFn = releaseJoint;
            cb.prepareJointDataFn = prepareJoint; cb.getConstantBlockFn = constantJoint;
            cb.onComShiftFn = comJoint; cb.onOriginShift = originJoint; cb.userData = m.get();
            m->registration = rt::runtime::getPhysxCustomJointInterface().registerCustomJoint(
                m->plugin->schema, cb, m->plugin->joint.solverPrep, m->plugin->joint.dataSize);
        } else {
            rt::ICustomGeometryCallback cb;
            const ovphysx::companion::native::GeometryCallbacks & g = m->plugin->geometry;
            cb.createCustomGeometryFn = createGeometry; cb.releaseCustomGeometryObjectFn = releaseGeometry;
            cb.computeCustomGeometryLocalBoundsFn = g.bounds; cb.generateCustomGeometryContactsFn = g.contacts;
            cb.computeCustomGeometryMassPropertiesFn = g.mass; cb.useCustomGeometryPersistentContactManifoldFn = g.manifold;
            cb.raycastCustomGeometryFn = g.raycast; cb.overlapCustomGeometryFn = g.overlap;
            cb.sweepCustomGeometryFn = g.sweep; cb.visualizeCustomGeometryFn = g.visualize; cb.userData = m.get();
            m->registration = rt::runtime::getPhysxCustomGeometryInterface().registerCustomGeometry(m->plugin->schema, cb);
        }
        if (!m->registration) return OVPHYSX_COMPANION_ERROR;
        rollback.committed = true;
        *output = handle;
        return OVPHYSX_COMPANION_OK;
    } catch (...) { return OVPHYSX_COMPANION_ERROR; }
}
bool refreshJoint(uint64_t owner, uint64_t key, const sdk::ParameterValue* values, uint32_t count, void* user) noexcept
{
    Module & m = *static_cast<Module*>(user);
    const sdk::ObjectContext input{owner, key, values, count};
    return m.plugin->joint.refresh(input, m.object(owner, key), m.context);
}
Status jointControl(uint64_t handle, uint64_t owner, const char* path, uint32_t action, uint32_t* flags) noexcept
{
    if (!path || path[0] != '/' || action > 3 || ((action == 1 || action == 2) && !flags))
        return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    try {
        CompanionOperationGuard admission(companionRuntime());
        if (!admission) return OVPHYSX_COMPANION_BUSY;
        std::unique_lock<std::mutex> lock(companionRuntime().modules().mutex, std::try_to_lock);
        if (!lock.owns_lock()) return OVPHYSX_COMPANION_BUSY;
        const std::map<uint64_t, std::shared_ptr<Module>>::iterator found = companionRuntime().modules().entries.find(handle);
        if (found == companionRuntime().modules().entries.end()) return OVPHYSX_COMPANION_NOT_FOUND;
        Module & m = *found->second;
        if (m.plugin->kind != sdk::Kind::Joint || (action == 3 && !m.plugin->joint.refresh))
            return OVPHYSX_COMPANION_UNAVAILABLE;
        uint32_t value = action == 1 ? *flags : 0;
        using Result = rt::native::JointResult;
        const Result result = rt::native::controlJoint(owner, path, &m, rt::native::JointAction(action), value,
            m.plugin->parameters, m.plugin->parameterCount, refreshJoint, &m);
        switch (result) {
        case Result::Success: if (action == 2) *flags = value; return OVPHYSX_COMPANION_OK;
        case Result::Busy: return OVPHYSX_COMPANION_BUSY;
        case Result::Stale: return OVPHYSX_COMPANION_STALE;
        case Result::NotFound: return OVPHYSX_COMPANION_NOT_FOUND;
        case Result::Invalid: return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        case Result::Rejected: return OVPHYSX_COMPANION_ERROR;
        }
    } catch (...) {}
    return OVPHYSX_COMPANION_ERROR;
}

Status capabilities(uint64_t handle, uint32_t* output) noexcept
{
    if (!output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    try {
        std::unique_lock<std::mutex> lock(companionRuntime().modules().mutex,std::try_to_lock);
        if (!lock.owns_lock()) return OVPHYSX_COMPANION_BUSY;
        const std::map<uint64_t, std::shared_ptr<Module>>::iterator found=companionRuntime().modules().entries.find(handle);
        if (found==companionRuntime().modules().entries.end()) return OVPHYSX_COMPANION_NOT_FOUND;
        const sdk::Plugin & p=*found->second->plugin;
        uint32_t value=0;
        if (p.kind==sdk::Kind::Joint) {
            value=OVPHYSX_COMPANION_NATIVE_JOINT_CPU;
            if (p.joint.refresh) value|=OVPHYSX_COMPANION_NATIVE_JOINT_REFRESH;
        } else {
            value=OVPHYSX_COMPANION_NATIVE_GEOMETRY_CPU;
            if (p.geometry.raycast) value|=OVPHYSX_COMPANION_NATIVE_RAYCAST;
            if (p.geometry.overlap) value|=OVPHYSX_COMPANION_NATIVE_OVERLAP;
            if (p.geometry.sweep) value|=OVPHYSX_COMPANION_NATIVE_SWEEP;
            if (p.geometry.visualize) value|=OVPHYSX_COMPANION_NATIVE_DEBUG;
        }
        *output=value;
        return OVPHYSX_COMPANION_OK;
    } catch (...) { return OVPHYSX_COMPANION_ERROR; }
}

Status close(uint64_t handle) noexcept
{
    try {
        CompanionOperationGuard admission(companionRuntime());
        if (!admission || rt::runtime::isRuntimeBusy(0)) return OVPHYSX_COMPANION_BUSY;
        std::unique_lock<std::mutex> lock(companionRuntime().modules().mutex, std::try_to_lock);
        if (!lock.owns_lock()) return OVPHYSX_COMPANION_BUSY;
        const std::map<uint64_t, std::shared_ptr<Module>>::iterator found = companionRuntime().modules().entries.find(handle);
        if (found == companionRuntime().modules().entries.end()) return OVPHYSX_COMPANION_NOT_FOUND;
        std::shared_ptr<Module> m = found->second;
        const bool retired = m->plugin->kind == sdk::Kind::Joint ?
            rt::runtime::getPhysxCustomJointInterface().tryUnregisterCustomJoint(m->registration) :
            rt::runtime::getPhysxCustomGeometryInterface().tryUnregisterCustomGeometry(m->registration);
        if (!retired) return OVPHYSX_COMPANION_BUSY;
        m->registration = 0;
        companionRuntime().modules().entries.erase(found);
        return OVPHYSX_COMPANION_OK;
    } catch (...) { return OVPHYSX_COMPANION_ERROR; }
}
}
using namespace ovphysx::companion::native_modules;
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_native_acquire(
    uint32_t version, uint32_t size, ovphysx_companion_native_api_t* output)
{
    if (!output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if (version != 1 || size != sizeof(*output)) return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output = {sizeof(*output), 1, load, close, jointControl};
    return OVPHYSX_COMPANION_OK;
}

extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_native_capabilities_acquire(
    uint32_t version, uint32_t size, ovphysx_companion_native_capabilities_api_t* output)
{
    if (!output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if (version!=1 || size!=sizeof(*output)) return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output={sizeof(*output),1,capabilities};
    return OVPHYSX_COMPANION_OK;
}
