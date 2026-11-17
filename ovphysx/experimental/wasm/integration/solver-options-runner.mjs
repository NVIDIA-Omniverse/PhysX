// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
import { requireSolverCapacity } from './solver-capacity.mjs';

export async function runSolverOptions(runtime, source) {
    const require = (condition, message) => { if (!condition) throw Error(message); };
    const near = (a, b, message) => require(Number.isFinite(a) && Math.abs(a - b) < 3e-4, message + ': ' + a + ' != ' + b);
    const check = (status, physics = false) => {
        if (status) throw Error(runtime.UTF8ToString(Number(physics ? runtime._ovp_wasm_error() : runtime._ovs_wasm_error())));
    };
    const counts = () => ({ running: runtime.PThread.runningWorkers.length, idle: runtime.PThread.unusedWorkers.length });
    const text = (value, callback) => {
        const bytes = new TextEncoder().encode(value), pointer = BigInt(runtime._malloc(bytes.length + 1));
        try { new Uint8Array(runtime.wasmMemory.buffer, Number(pointer), bytes.length).set(bytes); return callback(pointer, bytes.length); }
        finally { runtime._free(pointer); }
    };
    let stage = 0, ordinal = 1n;
    const records = [];
    const read = (path, name, size) => text(path, (p, pn) => text(name, (a, an) => {
        check(runtime._ovs_wasm_read_attribute(stage, p, pn, a, an, ordinal));
        require(runtime._ovs_wasm_result_count() === size, 'Unexpected native attribute shape');
        const Type = runtime._ovs_wasm_result_bits() === 64 ? Float64Array : Float32Array;
        return [...new Type(runtime.wasmMemory.buffer, Number(runtime._ovs_wasm_result_data()), size)];
    }));
    try {
        require(typeof runtime._ovp_wasm_configure_solver_threads === 'function', 'Runtime lacks solver configuration setter');
        for (const invalid of [1, 3, 5, 0xffffffff]) {
            const previous = runtime._ovp_wasm_solver_threads();
            require(runtime._ovp_wasm_configure_solver_threads(invalid) !== 0, 'Invalid worker selection must reject');
            require(runtime._ovp_wasm_solver_threads() === previous, 'Rejected selection changed configuration');
        }
        stage = runtime._ovs_wasm_create(); require(stage, 'Stage creation failed');
        text(source, (p, n) => check(runtime._ovs_wasm_open_usd_string(stage, p, n, ordinal, 0)));
        check(runtime._ovs_wasm_seal(stage, ordinal));
        for (const workers of [0, 4, 0, 2]) {
            check(runtime._ovp_wasm_configure_solver_threads(workers), true);
            require(runtime._ovp_wasm_solver_threads() === workers, 'Getter disagrees with selected count');
            const capacity = requireSolverCapacity(runtime), before = counts();
            check(runtime._ovp_wasm_create(stage, ordinal), true);
            const attached = counts();
            require(attached.running >= before.running + workers, 'SDK workers did not start');
            require(runtime._ovp_wasm_configure_solver_threads(workers === 4 ? 0 : 4) !== 0, 'Live owner accepted reconfiguration');
            require(runtime._ovp_wasm_solver_threads() === workers, 'Rejected live reconfiguration changed count');
            for (let tick = 0; tick < 8; ++tick) {
                check(runtime._ovp_wasm_step(1 / 60), true);
                check(runtime._ovp_wasm_publish(stage, ++ordinal), true); check(runtime._ovs_wasm_seal(stage, ordinal));
                check(runtime._ovp_wasm_read_vectors(0), true);
                require(runtime._ovp_wasm_value_count() === 6, 'Expected two live rigid bodies');
                const positions = [...new Float32Array(runtime.wasmMemory.buffer, Number(runtime._ovp_wasm_values()), 6)];
                require(positions.every(Number.isFinite), 'Nonfinite physics pose');
                for (const [path, scale] of [['/World/Mirrored', [2, 3, 4]], ['/World/Parent/Body', [1, 6, 1]]]) {
                    const matrix = read(path, 'omni:fabric:worldMatrix', 16), extent = read(path, '_worldExtent', 6);
                    const offset = Math.abs(positions[0] - matrix[12]) < 0.01 ? 0 : 3;
                    for (let axis = 0; axis < 3; ++axis) {
                        near(matrix[12 + axis], positions[offset + axis], 'Published pose mismatch');
                        near(Math.hypot(...matrix.slice(axis * 4, axis * 4 + 3)), scale[axis], 'Authored scale changed');
                        const radius = Math.abs(matrix[axis]) + Math.abs(matrix[4 + axis]) + Math.abs(matrix[8 + axis]);
                        near(extent[axis], matrix[12 + axis] - radius, 'World extent minimum mismatch');
                        near(extent[axis + 3], matrix[12 + axis] + radius, 'World extent maximum mismatch');
                    }
                }
            }
            check(runtime._ovp_wasm_destroy(), true); const detached = counts();
            require(detached.running <= attached.running - workers, 'Solver detach did not join its workers');
            require(detached.idle >= attached.idle + workers, 'Joined solver workers were not returned to pool');
            records.push({ workers, nativeGetter: { name: '_ovp_wasm_solver_threads', value: runtime._ovp_wasm_solver_threads() }, capacity, before, attached, detached, publications: 8 });
        }
        check(runtime._ovp_wasm_configure_solver_threads(0), true);
        require(runtime._ovp_wasm_create(stage, 0n) !== 0, 'Invalid attachment must fail');
        check(runtime._ovp_wasm_destroy(), true);
        check(runtime._ovp_wasm_configure_solver_threads(2), true);
        return { passed: true, records, finalConfiguredWorkers: runtime._ovp_wasm_solver_threads(),
            limitations: ['One USD stage owner; this does not qualify raw module/provider reuse.'],
            checks: ['0 to 4 to 0 to 2 on one side module', 'real SDK thread startup/join', 'invalid and live configuration rejected',
                '32 fresh pose/scale/world-bound publications', 'failed attachment cleanup permits a new selection'] };
    } finally {
        try {
            try { check(runtime._ovp_wasm_destroy(), true); }
            finally { if (stage) check(runtime._ovs_wasm_destroy(stage)); }
        } finally { runtime.PThread.terminateAllThreads(); }
    }
}
