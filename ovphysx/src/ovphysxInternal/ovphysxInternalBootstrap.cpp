// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// Sidecar module bootstrap: holds the peer-DSO OmniCore/Carbonite globals and the
// two setter exports the SDK loader calls right after dlopen to seed them from the
// main library. Shared sidecar infrastructure that every sidecar entry point relies on.

#include "internal/sidecar/ovphysxInternal.h"  // OVPHYSX_INTERNAL_API

#include <carb/ClientUtils.h>
#include <carb/detail/SetRuntimeGlobals.h>
#include <carb/logging/Log.h>
#include <omni/core/BuiltIn.h>
#include <omni/core/ITypeFactory.h>
#include <omni/core/Omni.h>
#include <omni/core/OmniInit.h>

// The sidecar is a peer DSO of the main ovphysx library: it uses plain CARB_GLOBALS
// (not CARB_STATIC_BINARY_GLOBALS), with g_carbFramework and OmniCore built-ins
// injected at load by the SDK loader via the setters below (mirrors the ovrtx
// peer-DSO pattern). The macro generates omniGetBuiltInWithoutAcquire() over the
// s_omni* built-in slots the setters seed.
OMNI_MODULE_DEFINE_OMNI_FUNCTIONS()
CARB_GLOBALS("ovphysx_internal")

extern "C" {

// Seeds the sidecar's module-local carb::Framework, the omni::core / omni::log /
// IStructuredLog runtime globals, and the logging source from the main library's
// framework. Called once by loadInternalSidecar() right after the dlopen handshake,
// single-threaded, before any other entry point resolves.
//
// Unlike the rtx.hydra setter (which leaves logging silent in peer DSOs), this
// registers the logging source here because the sidecar has many CARB_LOG_* calls
// in error/info paths and silent logs would hide diagnostics. Safe to do at-load
// since it runs exactly once on the loader thread, with no per-call race.
OVPHYSX_INTERNAL_API void ovphysx_internal_set_framework(carb::Framework* framework)
{
    g_carbFramework = framework;
    if (framework)
    {
        carb::detail::setRuntimeGlobalsFromFramework(framework);
        carb::logging::registerLoggingSourceForClient();
    }
}

// Seed the sidecar's OmniCore built-ins (typeFactory / log / structured log)
// from the main library's instances. Paired with ovphysx_internal_set_framework;
// must be called from loadInternalSidecar() after the framework setter.
OVPHYSX_INTERNAL_API void ovphysx_internal_set_omni_builtins(
    omni::core::ITypeFactory* typeFactory,
    omni::log::ILog* log,
    omni::structuredlog::IStructuredLog* structuredLog)
{
    s_omniTypeFactory = typeFactory;
    s_omniLog = log;
    s_omniStructuredLog = structuredLog;
}

} // extern "C"
