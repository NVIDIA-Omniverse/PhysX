// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>

// SDK-side dynamic loader for the internal sidecar (libovphysx_internal.so /
// ovphysx_internal.dll). Resolves the sidecar's exported entry points at load
// time and publishes them via the g_sidecar* atomics declared in each
// subsystem's sidecar header (e.g. ovphysxInternalInterop.h).
// Subsystem consumers include their own sidecar header and read the atomic
// directly. They only need this header to call the loader itself or
// resolveSidecarSymbol().

// Loads the sidecar and resolves symbols. Successful initialization is cached,
// so subsequent calls return true immediately. Failed attempts are retryable:
// every g_sidecar* is cleared and the next call runs a fresh dlopen/dlsym pass.
// On failure every g_sidecar* remains nullptr.
bool loadInternalSidecar();

// Resolves an arbitrary symbol from the already-loaded internal sidecar.
// Returns nullptr if the sidecar is not loaded yet or the symbol is missing.
// Use for one-off symbols not tracked by the per-subsystem g_sidecar* globals.
// Standard sidecar exports are resolved by loadInternalSidecar() and available
// via those globals directly.
//
// Does not load the sidecar. Call loadInternalSidecar() first.
void* resolveSidecarSymbol(const char* name);
