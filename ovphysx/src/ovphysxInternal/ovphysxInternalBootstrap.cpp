// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Sidecar module bootstrap. Holds the peer-DSO OmniCore/Carbonite globals and the
// two setter exports the SDK loader calls right after dlopen to seed them from the
// main library.

#include "internal/sidecar/ovphysxInternal.h"  // OVPHYSX_INTERNAL_API

#include <carb/ClientUtils.h>
#include <carb/detail/SetRuntimeGlobals.h>
#include <carb/logging/Log.h>
#include <omni/core/BuiltIn.h>
#include <omni/core/ITypeFactory.h>
#include <omni/core/Omni.h>
#include <omni/core/OmniInit.h>

// The sidecar is a peer DSO of the main ovphysx library. It uses plain CARB_GLOBALS
// rather than CARB_STATIC_BINARY_GLOBALS, and the SDK loader injects g_carbFramework
// and the OmniCore built-ins at load through the setters below (the ovrtx peer-DSO
// pattern). The macro generates omniGetBuiltInWithoutAcquire() over the s_omni*
// built-in slots the setters seed.
OMNI_MODULE_DEFINE_OMNI_FUNCTIONS()
CARB_GLOBALS("ovphysx_internal")

extern "C" {

// Seeds the sidecar's module-local carb::Framework, the omni::core / omni::log /
// IStructuredLog runtime globals, and the logging source from the main library's
// framework. Called once by loadInternalSidecar() on the loader thread, right after
// the dlopen handshake and before any other entry point resolves.
//
// Unlike the rtx.hydra setter, this registers the logging source, because the
// sidecar reports diagnostics through CARB_LOG_* in its error and info paths.
OVPHYSX_INTERNAL_API void ovphysx_internal_set_framework(carb::Framework* framework)
{
    g_carbFramework = framework;
    if (framework)
    {
        carb::detail::setRuntimeGlobalsFromFramework(framework);
        carb::logging::registerLoggingSourceForClient();
    }
}

// Seeds the sidecar's OmniCore built-ins (typeFactory / log / structured log)
// from the main library's instances. loadInternalSidecar() calls it after
// ovphysx_internal_set_framework.
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
