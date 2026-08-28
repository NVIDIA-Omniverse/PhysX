// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <cstdint>

// SDK-side dynamic loader for the internal sidecar (libovphysx_internal.so /
// ovphysx_internal.dll). Resolves the sidecar's exported entry points at load
// time and publishes them via the g_sidecar* atomics declared in each
// subsystem's sidecar header (e.g. ovphysxInternalClone.h, ovphysxInternalStage.h).
// Subsystem consumers include their own sidecar header and read the atomic
// directly; they don't need this header unless they call the loader itself,
// resolveSidecarSymbol(), or one of the convenience wrappers below.

// Load the sidecar and resolve symbols. Successful initialization is cached
// (subsequent calls return true immediately). Failed attempts are retryable:
// every g_sidecar* is cleared and the next call will try a fresh dlopen/
// dlsym pass. Returns true on success; on failure, every g_sidecar* remains
// nullptr.
bool loadInternalSidecar();

// Resolve an arbitrary symbol from the already-loaded internal sidecar.
// Returns nullptr if the sidecar isn't loaded yet or if the symbol is
// missing. Use for one-off symbols not tracked by the per-subsystem
// g_sidecar* globals; standard sidecar exports are resolved by
// loadInternalSidecar() and available via those globals directly.
//
// Does NOT load the sidecar -- call loadInternalSidecar() first.
void* resolveSidecarSymbol(const char* name);

// Convenience wrappers around the resolved sidecar function pointers. Each
// is a no-op if the corresponding pointer is nullptr (sidecar not loaded or
// symbol missing). Wrapper names mirror the sidecar exports they delegate to.
// Definitions live with the subsystems they serve (stage pair in
// ovphysxStage.cpp); this file only holds the loader itself.
void ovphysx_close_usd_stage_wrapper(int64_t stage_id);
