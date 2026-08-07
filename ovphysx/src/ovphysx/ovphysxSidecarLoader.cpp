// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// SDK-side dynamic loader for the internal sidecar.
//
// Resolves the sidecar shared object (libovphysx_internal.so / ovphysx_internal.dll)
// at first use, verifies the interface version against OVPHYSX_INTERNAL_INTERFACE_VERSION,
// and stores resolved function pointers into the g_sidecar* atomics whose
// externs are declared in the per-subsystem sidecar headers and whose storage
// lives in each subsystem's cpp file. Wrappers load those atomics with acquire ordering.

#include "internal/sdk/ovphysxSDKSidecarLoader.hpp"
#include "internal/sdk/PlatformIncludes.hpp"
#include "internal/sidecar/ovphysxInternal.h"            // OVPHYSX_INTERNAL_INTERFACE_VERSION + ovphysx_plugin_version typedef
#include "internal/sidecar/ovphysxInternalInterop.h"     // interop g_sidecar* declarations
#include "internal/sidecar/ovphysxInternalObjectChange.h" // object-change g_sidecar* declarations
#include "internal/sidecar/ovphysxInternalStage.h"       // stage g_sidecar* declarations
#include <omni/physx/PhysXRuntime.h>
#include <omni/physx/IOvxPhysicsRead.h>  // omni::physx::ovx* read entry points (injected into the sidecar)

#include <carb/Framework.h>
#include <carb/logging/Log.h>
#include <omni/core/BuiltIn.h>
#include <omni/core/ITypeFactory.h>
#include <omni/core/Omni.h>
#include <omni/log/ILog.h>
#include <omni/structuredlog/IStructuredLog.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>

// The g_sidecar* atomics themselves are defined in each subsystem's cpp file
// (e.g. the stage-close entry point in ovphysxInternalStage.cpp), next to the
// code that reads them. The loader writes through the externs declared in the
// per-subsystem sidecar headers (#include'd above).

namespace {

#ifdef _WIN32
using SidecarHandle = HMODULE;
#else
using SidecarHandle = void*;
#endif

// s_sidecarHandle is read by resolveSidecarSymbol() from outside the loader
// mutex, so it's atomic too. Loaded path is only accessed under
// s_sidecarLoadMutex and doubles as the loader's idempotency gate:
// `!s_sidecarLoadedPath.empty()` means the sidecar finished loading
// successfully. unload() clears the path on any failure.
static std::atomic<SidecarHandle> s_sidecarHandle{nullptr};
static std::string s_sidecarLoadedPath;
static std::mutex s_sidecarLoadMutex;

// Single dlsym/GetProcAddress entry point. The loader uses this with the
// just-opened handle (while still inside loadInternalSidecar()); the public
// resolveSidecarSymbol() uses it with the published s_sidecarHandle.
void* doSidecarSym(SidecarHandle handle, const char* name)
{
    if (!handle || !name) {
        return nullptr;
    }
#ifdef _WIN32
    return reinterpret_cast<void*>(GetProcAddress(handle, name));
#else
    return dlsym(handle, name);
#endif
}

void logSidecarSource(const char* reason)
{
    if (!s_sidecarLoadedPath.empty())
    {
        // Loading (or reusing) the internal sidecar is expected during normal operation.
        // Errors are logged explicitly at the call sites where dlopen/LoadLibrary fails.
        CARB_LOG_INFO("Internal sidecar: %s %s", reason ? reason : "state", s_sidecarLoadedPath.c_str());
    }
}

} // namespace

bool loadInternalSidecar() {
    std::lock_guard<std::mutex> lock(s_sidecarLoadMutex);

    // Already loaded?
    if (!s_sidecarLoadedPath.empty()) {
        logSidecarSource("already loaded");
        return true;
    }

#ifdef _WIN32
    constexpr const char* kDefaultName = OVPHYSX_INTERNAL_LIB_NAME;

    // Add the directory where ovphysx.dll lives to PATH (once only).
    // This ensures Windows can find ovphysx_internal.dll which is in the same directory.
    // CarboniteLoader adds dependency directories (USD libs, etc.) to PATH as well.
    static bool s_pathUpdated = false;
    if (!s_pathUpdated)
    {
        HMODULE hOvphysx = NULL;
        if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                               (LPCSTR)&loadInternalSidecar, &hOvphysx))
        {
            char ovphysxPath[MAX_PATH] = {0};
            DWORD pathLen = GetModuleFileNameA(hOvphysx, ovphysxPath, MAX_PATH);
            if (pathLen > 0 && pathLen < MAX_PATH)
            {
                char* lastSlash = strrchr(ovphysxPath, '\\');
                if (lastSlash) *lastSlash = '\0';

                // Prepend to PATH
                const char* currentPath = std::getenv("PATH");
                std::string newPath = std::string(ovphysxPath);
                if (currentPath && currentPath[0] != '\0')
                {
                    newPath += ";";
                    newPath += currentPath;
                }
                const errno_t envRc = _putenv_s("PATH", newPath.c_str());
                if (envRc == 0)
                {
                    s_pathUpdated = true;
                }
                else
                {
                    CARB_LOG_WARN("Internal sidecar: failed to update PATH (errno=%d)", int(envRc));
                }
            }
            else
            {
                CARB_LOG_WARN("Internal sidecar: GetModuleFileNameA failed (GetLastError=%lu)", GetLastError());
            }
        }
    }

    SidecarHandle handle = LoadLibraryA(kDefaultName);
    if (!handle)
    {
        CARB_LOG_ERROR("Internal sidecar: failed to load %s (GetLastError=%lu)", kDefaultName, GetLastError());
        return false;
    }
#else
    constexpr const char* kDefaultName = OVPHYSX_INTERNAL_LIB_NAME;

    SidecarHandle handle = dlopen(kDefaultName, RTLD_NOW | RTLD_LOCAL);
    if (!handle)
    {
        const char* err = dlerror();
        CARB_LOG_ERROR("Internal sidecar: dlopen failed for %s -> %s", kDefaultName, err ? err : "unknown");
        return false;
    }
#endif

    // Defer publishing s_sidecarHandle / s_sidecarLoadedPath until after the
    // full handshake + symbol resolution succeeds. A concurrent caller of
    // resolveSidecarSymbol() must never observe the handle while we're still
    // in the middle of resolving (or about to unload() on failure).
    auto unload = [handle]() {
#ifdef _WIN32
        FreeLibrary(handle);
#else
        dlclose(handle);
#endif
        s_sidecarHandle.store(nullptr, std::memory_order_release);
        // Clear every resolved pointer so a partial-load failure can't leave
        // stale function pointers visible to wrappers on the next call.
        g_sidecarCloseUsdStage.store(nullptr, std::memory_order_release);
        g_sidecarGetPhysXPtr.store(nullptr, std::memory_order_release);
        g_sidecarUpdateKinematic.store(nullptr, std::memory_order_release);
        g_sidecarEncodeSdfPath.store(nullptr, std::memory_order_release);
        g_sidecarOutputQuery.store(nullptr, std::memory_order_release);
        g_sidecarReadOutputs.store(nullptr, std::memory_order_release);
        g_sidecarFetchReadNext.store(nullptr, std::memory_order_release);
        g_sidecarReleaseRead.store(nullptr, std::memory_order_release);
        g_sidecarReleaseQuery.store(nullptr, std::memory_order_release);
        g_sidecarSubscribeObjectChanges.store(nullptr, std::memory_order_release);
        g_sidecarUnsubscribeObjectChanges.store(nullptr, std::memory_order_release);
        g_sidecarEnableVisualization.store(nullptr, std::memory_order_release);
        g_sidecarSetVizParameter.store(nullptr, std::memory_order_release);
        g_sidecarSetVizParameterValue.store(nullptr, std::memory_order_release);
        g_sidecarSetVizScopeTokens.store(nullptr, std::memory_order_release);
        g_sidecarSetVizScale.store(nullptr, std::memory_order_release);
        g_sidecarSetVizCullingBox.store(nullptr, std::memory_order_release);
        g_sidecarGetDebugPoints.store(nullptr, std::memory_order_release);
        g_sidecarGetDebugLines.store(nullptr, std::memory_order_release);
        g_sidecarGetDebugTriangles.store(nullptr, std::memory_order_release);
        s_sidecarLoadedPath.clear();  // clearing this is what un-gates the "already loaded" check at the top of loadInternalSidecar()
    };

    // Version probe is only used here for the handshake; not stored as a global.
    using VersionFn = uint32_t (*)();
    VersionFn versionFn = (VersionFn)doSidecarSym(handle, "ovphysx_plugin_version");
    if (!versionFn) {
        CARB_LOG_ERROR("Internal sidecar loaded but version function 'ovphysx_plugin_version' not found");
        unload();
        return false;
    }

    uint32_t version = versionFn();
    if (version != OVPHYSX_INTERNAL_INTERFACE_VERSION) {
        CARB_LOG_ERROR("Internal sidecar interface version mismatch: expected %u, got %u. "
                       "Rebuild the sidecar to match the main library version.",
                       OVPHYSX_INTERNAL_INTERFACE_VERSION, version);
        unload();
        return false;
    }

    // Inject the main library's framework + OmniCore built-ins into the sidecar
    // before resolving sidecar entry points. The sidecar is a peer DSO, not a
    // Carbonite plugin; CARB_GLOBALS and OMNI_MODULE_DEFINE_OMNI_FUNCTIONS keep
    // framework/log/type-factory state in module-local storage, so dlopen alone
    // does not seed it. This setter handshake follows the ovrtx/rtx.hydra
    // peer-DSO pattern and runs before any CARB_LOG_* or ObjectPtr<T> cleanup
    // can observe null globals.
    using SetFrameworkFn   = void (*)(carb::Framework*);
    using SetOmniBuiltInsFn = void (*)(omni::core::ITypeFactory*, omni::log::ILog*, omni::structuredlog::IStructuredLog*);
    using SetPhysxRuntimeAccessorsFn = void (*)(
        OvphysxInternalGetPhysxInterfaceFn,
        OvphysxInternalGetPhysxVisualizationInterfaceFn);
    SetFrameworkFn setFramework = (SetFrameworkFn)doSidecarSym(handle, "ovphysx_internal_set_framework");
    SetOmniBuiltInsFn setOmniBuiltIns = (SetOmniBuiltInsFn)doSidecarSym(handle, "ovphysx_internal_set_omni_builtins");
    SetPhysxRuntimeAccessorsFn setPhysxRuntimeAccessors =
        (SetPhysxRuntimeAccessorsFn)doSidecarSym(handle, "ovphysx_internal_set_physx_runtime_accessors");
    if (!setFramework || !setOmniBuiltIns || !setPhysxRuntimeAccessors) {
        CARB_LOG_ERROR("Internal sidecar loaded but runtime-context setters "
                       "(ovphysx_internal_set_framework / ovphysx_internal_set_omni_builtins / "
                       "ovphysx_internal_set_physx_runtime_accessors) not found");
        unload();
        return false;
    }

    // Owner-side preflight: fail fast if the main library hasn't completed its
    // own Carbonite bootstrap, instead of injecting nulls into the sidecar and
    // then deferring the failure to every sidecar entry point. Both must be
    // non-null because the sidecar relies on the framework for settings and runtime
    // interfaces and on the typeFactory for type creation through createTypeFromMainOmniCore.
    carb::Framework* ownerFramework = carb::getFramework();
    omni::core::ITypeFactory* ownerTypeFactory = static_cast<omni::core::ITypeFactory*>(
        omniGetBuiltInWithoutAcquire(OmniBuiltIn::eITypeFactory));
    if (!ownerFramework || !ownerTypeFactory) {
        CARB_LOG_ERROR("Internal sidecar: cannot inject runtime context — main library not bootstrapped "
                       "(framework=%p, typeFactory=%p). Ensure ovphysx_create_instance has acquired the "
                       "framework before loadInternalSidecar() is called.",
                       (void*)ownerFramework, (void*)ownerTypeFactory);
        unload();
        return false;
    }
    setFramework(ownerFramework);
    setOmniBuiltIns(ownerTypeFactory,
                    static_cast<omni::log::ILog*>(omniGetBuiltInWithoutAcquire(OmniBuiltIn::eILog)),
                    static_cast<omni::structuredlog::IStructuredLog*>(omniGetBuiltInWithoutAcquire(OmniBuiltIn::eIStructuredLog)));

    omni::physx::IPhysx* ownerPhysx = omni::physx::runtime::tryGetPhysxInterface();
    if (!ownerPhysx)
    {
        CARB_LOG_ERROR("Internal sidecar: cannot inject PhysX runtime accessors "
                       "(IPhysx=%p). Ensure omni.physx runtime startup "
                       "has completed before loadInternalSidecar() is called.",
                       (void*)ownerPhysx);
        unload();
        return false;
    }
    setPhysxRuntimeAccessors(&omni::physx::runtime::tryGetPhysxInterface,
                             &omni::physx::runtime::tryGetPhysxVisualizationInterface);

    // Inject the ovstage-native output-read entry points (ADR-0007). They live in
    // the owner's statically-linked OvruntimePhysX; taking their address here pulls
    // OvxPhysicsRead.o into libovphysx and resolves it internally. The sidecar has no
    // OvruntimePhysX of its own and the owner's symbols are hidden, so without this
    // injection ovphysx_query() would silently fail to open every output read.
    using SetOvxReadAccessorsFn = void (*)(void*, void*, void*, void*, void*, void*, void*, void*);
    SetOvxReadAccessorsFn setOvxReadAccessors =
        (SetOvxReadAccessorsFn)doSidecarSym(handle, "ovphysx_internal_set_ovx_read_accessors");
    if (setOvxReadAccessors)
    {
        setOvxReadAccessors(reinterpret_cast<void*>(&omni::physx::ovxQuery),
                            reinterpret_cast<void*>(&omni::physx::ovxFetchQueryResult),
                            reinterpret_cast<void*>(&omni::physx::ovxQueryDictionary),
                            reinterpret_cast<void*>(&omni::physx::ovxReadAttributes),
                            reinterpret_cast<void*>(&omni::physx::ovxFetchReadNext),
                            reinterpret_cast<void*>(&omni::physx::ovxReleaseGroup),
                            reinterpret_cast<void*>(&omni::physx::ovxReleaseRead),
                            reinterpret_cast<void*>(&omni::physx::ovxReleaseQuery));
    }
    else
    {
        CARB_LOG_WARN("Internal sidecar: ovphysx_internal_set_ovx_read_accessors not found; "
                      "ovstage-native output read (ovphysx_query) will be unavailable.");
    }

    auto closeUsdStage = (OvphysxSidecarCloseUsdStageFn)doSidecarSym(handle, "ovphysx_close_usd_stage");
    if (!closeUsdStage) {
        // Required: without close, every opened stage stays pinned in the
        // sidecar's UsdUtilsStageCache for the process lifetime. Fail fast.
        CARB_LOG_ERROR("Internal sidecar loaded but 'ovphysx_close_usd_stage' not found - "
                       "stage cache cleanup would be unavailable. Sidecar may be outdated.");
        unload();
        return false;
    }
    g_sidecarCloseUsdStage.store(closeUsdStage, std::memory_order_release);

    auto getPhysXPtr = (OvphysxSidecarGetPhysXPtrFn)doSidecarSym(handle, "ovphysx_internal_get_physx_ptr");
    if (!getPhysXPtr) {
        CARB_LOG_WARN("Internal sidecar loaded but 'ovphysx_internal_get_physx_ptr' not found - "
                      "PhysX object pointer lookup will be unavailable");
    }
    g_sidecarGetPhysXPtr.store(getPhysXPtr, std::memory_order_release);

    auto encodeSdfPath = (OvphysxSidecarEncodeSdfPathFn)doSidecarSym(handle, "ovphysx_encode_sdf_path");
    if (!encodeSdfPath) {
        CARB_LOG_WARN("Internal sidecar loaded but 'ovphysx_encode_sdf_path' not found - "
                      "scene query path encoding will be unavailable");
    }
    g_sidecarEncodeSdfPath.store(encodeSdfPath, std::memory_order_release);

    auto updateKinematic = (OvphysxSidecarUpdateKinematicFn)doSidecarSym(handle, "ovphysx_internal_update_kinematic");
    if (!updateKinematic) {
        // Required: ovphysx_articulation_update_kinematic is a public API
        // backed by this symbol. A warn-and-continue policy would mark the
        // sidecar as loaded but defer the actual failure to the first user
        // call. Treat as a fatal load error so callers see the sidecar as
        // not-loaded and can react / report cleanly.
        CARB_LOG_ERROR("Internal sidecar loaded but 'ovphysx_internal_update_kinematic' not found - "
                       "ovphysx_articulation_update_kinematic cannot work. Sidecar may be outdated.");
        unload();
        return false;
    }
    g_sidecarUpdateKinematic.store(updateKinematic, std::memory_order_release);

    // Physics output read (ADR-0007). Optional: warn-and-continue so an older
    // sidecar still loads; the public read API reports "not loaded" if missing.
    {
        auto outputQuery = (OvphysxSidecarOutputQueryFn)doSidecarSym(handle, "ovphysx_internal_output_query");
        auto fetchQueryResult = (OvphysxSidecarFetchQueryResultFn)doSidecarSym(handle, "ovphysx_internal_fetch_query_result");
        auto queryDictionary = (OvphysxSidecarQueryDictionaryFn)doSidecarSym(handle, "ovphysx_internal_query_dictionary");
        auto readOutputs = (OvphysxSidecarReadOutputsFn)doSidecarSym(handle, "ovphysx_internal_read_outputs");
        auto fetchReadNext = (OvphysxSidecarFetchReadNextFn)doSidecarSym(handle, "ovphysx_internal_fetch_read_next");
        auto releaseGroup = (OvphysxSidecarReleaseGroupFn)doSidecarSym(handle, "ovphysx_internal_release_group");
        auto releaseRead = (OvphysxSidecarReleaseReadFn)doSidecarSym(handle, "ovphysx_internal_release_read");
        auto releaseQuery = (OvphysxSidecarReleaseQueryFn)doSidecarSym(handle, "ovphysx_internal_release_query");
        if (!outputQuery || !fetchQueryResult || !queryDictionary || !readOutputs || !fetchReadNext || !releaseGroup ||
            !releaseRead || !releaseQuery)
        {
            CARB_LOG_WARN("Internal sidecar loaded but the physics output read entry points "
                          "(ovphysx_internal_output_query/fetch_query_result/query_dictionary/read_outputs/"
                          "fetch_read_next/release_*) were not all found - ovphysx_query/ovphysx_read will be "
                          "unavailable. Sidecar may be outdated.");
        }
        g_sidecarOutputQuery.store(outputQuery, std::memory_order_release);
        g_sidecarFetchQueryResult.store(fetchQueryResult, std::memory_order_release);
        g_sidecarQueryDictionary.store(queryDictionary, std::memory_order_release);
        g_sidecarReadOutputs.store(readOutputs, std::memory_order_release);
        g_sidecarFetchReadNext.store(fetchReadNext, std::memory_order_release);
        g_sidecarReleaseGroup.store(releaseGroup, std::memory_order_release);
        g_sidecarReleaseRead.store(releaseRead, std::memory_order_release);
        g_sidecarReleaseQuery.store(releaseQuery, std::memory_order_release);
    }

    auto subscribeObjectChanges = (OvphysxSidecarSubscribeObjectChangesFn)doSidecarSym(handle, "ovphysx_internal_subscribe_object_changes");
    if (!subscribeObjectChanges) {
        CARB_LOG_WARN("Internal sidecar loaded but 'ovphysx_internal_subscribe_object_changes' not found - "
                      "object change notifications will be unavailable");
    }
    g_sidecarSubscribeObjectChanges.store(subscribeObjectChanges, std::memory_order_release);

    auto unsubscribeObjectChanges = (OvphysxSidecarUnsubscribeObjectChangesFn)doSidecarSym(handle, "ovphysx_internal_unsubscribe_object_changes");
    if (!unsubscribeObjectChanges) {
        CARB_LOG_WARN("Internal sidecar loaded but 'ovphysx_internal_unsubscribe_object_changes' not found - "
                      "object change unsubscribe will be unavailable");
    }
    g_sidecarUnsubscribeObjectChanges.store(unsubscribeObjectChanges, std::memory_order_release);

    // Debug-visualization bridge symbols. Warn-and-continue (non-fatal): an older
    // sidecar without them simply leaves debug viz unavailable (the main-lib calls
    // no-op to SUCCESS), matching the get_physx_ptr / object-change policy.
    auto enableViz = (OvphysxSidecarEnableVisualizationFn)doSidecarSym(handle, "ovphysx_internal_enable_visualization");
    if (!enableViz) {
        CARB_LOG_WARN("Internal sidecar loaded but 'ovphysx_internal_enable_visualization' not found - "
                      "PhysX debug visualization will be unavailable");
    }
    g_sidecarEnableVisualization.store(enableViz, std::memory_order_release);
    g_sidecarSetVizParameter.store(
        (OvphysxSidecarSetVizParameterFn)doSidecarSym(handle, "ovphysx_internal_set_visualization_parameter"), std::memory_order_release);
    g_sidecarSetVizParameterValue.store(
        (OvphysxSidecarSetVizParameterValueFn)doSidecarSym(handle, "ovphysx_internal_set_visualization_parameter_value"), std::memory_order_release);
    g_sidecarSetVizScopeTokens.store(
        (OvphysxSidecarSetVizScopeTokensFn)doSidecarSym(handle, "ovphysx_internal_set_visualization_scope_tokens"), std::memory_order_release);
    g_sidecarSetVizScale.store(
        (OvphysxSidecarSetVizScaleFn)doSidecarSym(handle, "ovphysx_internal_set_visualization_scale"), std::memory_order_release);
    g_sidecarSetVizCullingBox.store(
        (OvphysxSidecarSetVizCullingBoxFn)doSidecarSym(handle, "ovphysx_internal_set_visualization_culling_box"), std::memory_order_release);
    g_sidecarGetDebugPoints.store(
        (OvphysxSidecarGetDebugBufferFn)doSidecarSym(handle, "ovphysx_internal_get_debug_points"), std::memory_order_release);
    g_sidecarGetDebugLines.store(
        (OvphysxSidecarGetDebugBufferFn)doSidecarSym(handle, "ovphysx_internal_get_debug_lines"), std::memory_order_release);
    g_sidecarGetDebugTriangles.store(
        (OvphysxSidecarGetDebugBufferFn)doSidecarSym(handle, "ovphysx_internal_get_debug_triangles"), std::memory_order_release);

    // Publish the handle + loaded-path gate only now that the full handshake
    // succeeded. Until this point, resolveSidecarSymbol() returns nullptr
    // because s_sidecarHandle is still null.
    s_sidecarHandle.store(handle, std::memory_order_release);
    s_sidecarLoadedPath = kDefaultName;
    logSidecarSource("loaded");
    return true;
}

void* resolveSidecarSymbol(const char* name) {
    return doSidecarSym(s_sidecarHandle.load(std::memory_order_acquire), name);
}
