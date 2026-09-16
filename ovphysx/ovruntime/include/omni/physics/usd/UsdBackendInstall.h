// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 */

#pragma once

// Export macro for the test-only USD library (ADR-0027). The library is built STATIC
// (OMNI_PHYSICS_USD_STATIC is a PUBLIC define on the target, so this expands to nothing);
// the C install entry below is its deliberate load boundary either way.
#if defined(_MSC_VER)
#    if defined(OMNI_PHYSICS_USD_STATIC)
#        define OMNI_PHYSICS_USD_API
#    elif defined(OMNI_PHYSICS_USD_EXPORTS)
#        define OMNI_PHYSICS_USD_API __declspec(dllexport)
#    else
#        define OMNI_PHYSICS_USD_API __declspec(dllimport)
#    endif
#else
#    define OMNI_PHYSICS_USD_API __attribute__((visibility("default")))
#endif

extern "C"
{
// Installs ONLY the two stateless opaque-handle op tables (ADR-0027 seam #3:
// parse::setAttachedStageUsdHandleOps / setSimulationLayerHandleOps). Idempotent: the
// tables are process-constant, so repeated calls store the same pointers. Must run before
// the first handle owner exists -- i.e. before omni::physx::runtime::startup(), whose
// OmniPhysX instance owns a SimulationLayerHandle -- and the tables are never removed for
// the rest of the process: a table change while an owner lives would alter the owner's
// representation under it. Only test executables link this library (production never
// installs a table, so every handle there is a zeroed POD for its whole life).
OMNI_PHYSICS_USD_API void omniPhysicsUsdInstallHandleOps(void);

// Constructs the USD parse + scan backends and the functional ADR-0027 seams (stage
// lifecycle, attachment/backing authors, reparse) and installs them into the process-global
// ADR-0005 registries. Also installs the handle op tables (idempotent, see above).
// Idempotent-replacing; must be called detached (ADR-0005). Only test executables link this
// library and call this (ADR-0027) — production never does.
OMNI_PHYSICS_USD_API void omniPhysicsUsdInstallBackends(void);

// Functional teardown: clears the six functional slots (parse, scan, stageLifecycle,
// attachmentAuthor, backingAuthor, reparse), restoring "no functional USD backend
// installed". Leaves the two handle op tables installed -- handle owners (OmniPhysX's
// simulation layer, AttachedStage's stage) outlive any functional-backend toggle.
OMNI_PHYSICS_USD_API void omniPhysicsUsdRemoveBackends(void);
}
