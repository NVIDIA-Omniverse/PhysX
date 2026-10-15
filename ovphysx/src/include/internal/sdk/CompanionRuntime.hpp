// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CAPI-COMPANION-001
// @covers AC-4 AC-5
#pragma once
#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <thread>

namespace ovphysx { class CarboniteLoader; }
namespace omni::physx::native { struct DebugFrame; }
namespace omni::physx::diagnostics { struct OvdDocument; }
namespace ovphysx::companion {
namespace sessions { struct Session; }
namespace previews { struct Preview; }
namespace cooking { struct Service; struct Result; }
namespace native_modules { struct Module; }
namespace custom_interfaces { struct Lease; }
namespace events { struct Queue; }
namespace interaction { struct Session; }
}

// Registries belong to the companion owner, including their locks and handle
// sequences. Entries retain a service until its last in-flight call completes.
template<class T> struct CompanionRegistry
{
    std::mutex mutex;
    std::map<uint64_t, std::shared_ptr<T>> entries;
    uint64_t nextHandle = 1;
};

// Owned by the SDK lifecycle, alongside its initialization state. This is the
// coordinator for the one loaded runtime, not a current-attachment selector.
// Independent attachments share operation admission and retain their own locks.
class CompanionRuntime
{
public:
    CompanionRuntime() = default;
    CompanionRuntime(const CompanionRuntime&) = delete;
    CompanionRuntime& operator=(const CompanionRuntime&) = delete;

    bool enterOperation() noexcept
    {
        try
        {
            std::lock_guard<std::mutex> lock(m_admissionMutex);
            if (m_transitionDepth != 0)
                return false;
            return m_operations.emplace(std::this_thread::get_id(), false).second;
        }
        catch (...) { return false; }
    }
    void leaveOperation()
    {
        std::lock_guard<std::mutex> lock(m_admissionMutex);
        m_operations.erase(std::this_thread::get_id());
    }
    bool enterTransition()
    {
        std::lock_guard<std::mutex> lock(m_admissionMutex);
        if (!m_operations.empty())
            return false;
        const std::thread::id thread = std::this_thread::get_id();
        if (m_transitionDepth != 0 && m_transitionThread != thread)
            return false;
        m_transitionThread = thread;
        ++m_transitionDepth; // Core destroy nests core detach on the same thread.
        return true;
    }
    void leaveTransition()
    {
        std::lock_guard<std::mutex> lock(m_admissionMutex);
        --m_transitionDepth;
    }
    void setNativeAccess(bool active)
    {
        std::lock_guard<std::mutex> lock(m_admissionMutex);
        const std::map<std::thread::id, bool>::iterator found = m_operations.find(std::this_thread::get_id());
        if (found != m_operations.end())
            found->second = active;
    }
    bool nativeAccessOnThisThread() const
    {
        std::lock_guard<std::mutex> lock(m_admissionMutex);
        const std::map<std::thread::id, bool>::const_iterator found = m_operations.find(std::this_thread::get_id());
        return found != m_operations.end() && found->second;
    }
    void retainService() { ++m_serviceLeases; }
    void releaseService() { --m_serviceLeases; }
    bool hasServiceLeases() const { return m_serviceLeases.load() != 0; }

    // Lifecycle/open calls already hold the core instance-registry lock.
    void setAdmissionOpen(bool open) { m_admissionOpen = open; }
    bool admissionOpen() const { return m_admissionOpen; }

    // Called under exclusive transition admission. Pending state lives only on
    // the previews; no process counter can drift after a failed fetch or close.
    bool hasPendingPreviews();

    CompanionRegistry<ovphysx::companion::sessions::Session>& sessions() { return m_sessions; }
    CompanionRegistry<ovphysx::companion::previews::Preview>& previews() { return m_previews; }
    CompanionRegistry<ovphysx::companion::cooking::Service>& cookingServices() { return m_cookingServices; }
    CompanionRegistry<const ovphysx::companion::cooking::Result>& cookingResults() { return m_cookingResults; }
    CompanionRegistry<ovphysx::companion::custom_interfaces::Lease>& customInterfaces() { return m_customInterfaces; }
    CompanionRegistry<ovphysx::companion::native_modules::Module>& modules() { return m_modules; }
    CompanionRegistry<ovphysx::companion::events::Queue>& eventQueues() { return m_eventQueues; }
    CompanionRegistry<ovphysx::companion::interaction::Session>& interactions() { return m_interactions; }
    CompanionRegistry<const omni::physx::native::DebugFrame>& debugFrames() { return m_debugFrames; }
    CompanionRegistry<const omni::physx::diagnostics::OvdDocument>& ovdDocuments() { return m_ovdDocuments; }

private:
    mutable std::mutex m_admissionMutex;
    std::map<std::thread::id, bool> m_operations;
    std::thread::id m_transitionThread;
    uint32_t m_transitionDepth = 0;
    std::atomic<uint32_t> m_serviceLeases{0};
    bool m_admissionOpen = false;
    CompanionRegistry<ovphysx::companion::sessions::Session> m_sessions;
    CompanionRegistry<ovphysx::companion::previews::Preview> m_previews;
    CompanionRegistry<ovphysx::companion::cooking::Service> m_cookingServices;
    CompanionRegistry<const ovphysx::companion::cooking::Result> m_cookingResults;
    CompanionRegistry<ovphysx::companion::native_modules::Module> m_modules;
    CompanionRegistry<ovphysx::companion::custom_interfaces::Lease> m_customInterfaces;
    CompanionRegistry<ovphysx::companion::events::Queue> m_eventQueues;
    CompanionRegistry<ovphysx::companion::interaction::Session> m_interactions;
    CompanionRegistry<const omni::physx::native::DebugFrame> m_debugFrames;
    CompanionRegistry<const omni::physx::diagnostics::OvdDocument> m_ovdDocuments;
};

// The C ABI resolves its SDK lifecycle owner here. Guards take that owner
// explicitly; none of their state is static or thread-local.
CompanionRuntime& companionRuntime();

class CompanionTransitionGuard
{
public:
    explicit CompanionTransitionGuard(CompanionRuntime& runtime) : m_runtime(runtime), m_admitted(runtime.enterTransition()) {}
    ~CompanionTransitionGuard() { if (m_admitted) m_runtime.leaveTransition(); }
    explicit operator bool() const { return m_admitted; }
    CompanionTransitionGuard(const CompanionTransitionGuard&) = delete;
    CompanionTransitionGuard& operator=(const CompanionTransitionGuard&) = delete;
private:
    CompanionRuntime& m_runtime;
    bool m_admitted;
};

class CompanionOperationGuard
{
public:
    explicit CompanionOperationGuard(CompanionRuntime& runtime) : m_runtime(runtime), m_admitted(runtime.enterOperation()) {}
    ~CompanionOperationGuard() { if (m_admitted) m_runtime.leaveOperation(); }
    explicit operator bool() const { return m_admitted; }
    CompanionOperationGuard(const CompanionOperationGuard&) = delete;
    CompanionOperationGuard& operator=(const CompanionOperationGuard&) = delete;
private:
    CompanionRuntime& m_runtime;
    bool m_admitted;
};

class CompanionServiceLease
{
public:
    explicit CompanionServiceLease(CompanionRuntime& runtime) : m_runtime(runtime) { m_runtime.retainService(); }
    ~CompanionServiceLease() { m_runtime.releaseService(); }
    CompanionServiceLease(const CompanionServiceLease&) = delete;
    CompanionServiceLease& operator=(const CompanionServiceLease&) = delete;
private:
    CompanionRuntime& m_runtime;
};

class CompanionNativeAccessScope
{
public:
    explicit CompanionNativeAccessScope(CompanionRuntime& runtime) : m_runtime(runtime) { m_runtime.setNativeAccess(true); }
    ~CompanionNativeAccessScope() { m_runtime.setNativeAccess(false); }
    CompanionNativeAccessScope(const CompanionNativeAccessScope&) = delete;
    CompanionNativeAccessScope& operator=(const CompanionNativeAccessScope&) = delete;
private:
    CompanionRuntime& m_runtime;
};

// Called under operation admission. Retain the exact facade/preview owner.
std::shared_ptr<ovphysx::CarboniteLoader> companionRuntimeForAttachment(uint64_t attachment);
