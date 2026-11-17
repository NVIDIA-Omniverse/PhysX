// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Pinned Emscripten pool state belongs to the runtime owner, not a solver pthread.
// Reserve four additional workers for the bounded TBB/population host during attach.
export function requireSolverCapacity(runtime) {
    const headroom = 4;
    if (typeof runtime._ovp_wasm_solver_threads !== 'function')
        throw new Error('The solver experiment requires the configured-thread getter');
    const requested = runtime._ovp_wasm_solver_threads();
    if (![0, 2, 4].includes(requested)) throw new Error('Unsupported experimental solver thread count');
    if (!requested) return { requested, available: null, required: 0 };
    const unused = runtime.PThread?.unusedWorkers;
    if (!Array.isArray(unused)) throw new Error('The solver experiment requires owner-side Emscripten pool capacity');
    const required = requested + headroom;
    if (unused.length < required)
        throw new Error(`Solver attachment requires ${required} idle pthread workers (${requested} solver + ${headroom} headroom), only ${unused.length} available`);
    return { requested, available: unused.length, required };
}
