// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** Exercise publication buffer reuse without weakening native scale/hierarchy/lifetime semantics. */
export async function runOutputPublication(runtime, sourceText, { recreateStage = true } = {}) {
    const require = (condition, message) => { if (!condition) throw new Error(message); };
    const near = (actual, expected, message, tolerance = 2e-4) =>
        require(Number.isFinite(actual) && Math.abs(actual - expected) <= tolerance, `${message}: ${actual} != ${expected}`);
    const offset = (pointer, bytes = 0) => {
        const value = Number(pointer);
        if (!Number.isSafeInteger(value) || value < 0 || value + bytes > runtime.wasmMemory.buffer.byteLength)
            throw new RangeError('Read outside shared WASM memory');
        return value;
    };
    const message = physics => runtime.UTF8ToString(offset(physics ? runtime._ovp_wasm_error() : runtime._ovs_wasm_error()));
    const check = (status, physics = false) => {
        if (status) throw new Error(`${physics ? 'OvPhysX' : 'OVStage'} status ${status}: ${message(physics)}`);
    };
    const rejected = (status, reason) => { require(status !== 0, reason); require(message(true).length > 0, 'Rejected publication must explain its failure'); };
    const withStrings = (strings, action) => {
        const bytes = strings.map(text => new TextEncoder().encode(text));
        const pointers = bytes.map(bytes => BigInt(runtime._malloc(bytes.length + 1)));
        try {
            bytes.forEach((bytes, i) => new Uint8Array(runtime.wasmMemory.buffer, offset(pointers[i], bytes.length), bytes.length).set(bytes));
            return action(pointers, bytes.map(bytes => bytes.length));
        } finally { pointers.forEach(pointer => runtime._free(pointer)); }
    };
    const createStage = source => {
        const stage = runtime._ovs_wasm_create();
        require(stage, 'Create stage failed: ' + message(false));
        try {
            withStrings([source], ([pointer], [length]) => check(runtime._ovs_wasm_open_usd_string(stage, pointer, length, 1n, 0)));
            check(runtime._ovs_wasm_seal(stage, 1n));
            return stage;
        } catch (error) { runtime._ovs_wasm_destroy(stage); throw error; }
    };
    const matrix = (stage, path, ordinal, attribute = 'omni:fabric:worldMatrix', components = 16) => withStrings([path, attribute], (pointers, lengths) => {
        check(runtime._ovs_wasm_read_attribute(stage, pointers[0], lengths[0], pointers[1], lengths[1], ordinal));
        require(runtime._ovs_wasm_result_code() === 2 && runtime._ovs_wasm_result_count() === components, 'Expected a floating point transform or bound');
        const ArrayType = runtime._ovs_wasm_result_bits() === 64 ? Float64Array : Float32Array;
        return [...new ArrayType(runtime.wasmMemory.buffer, offset(runtime._ovs_wasm_result_data(), components * ArrayType.BYTES_PER_ELEMENT), components)];
    });
    const positions = () => {
        check(runtime._ovp_wasm_read_vectors(0), true);
        const count = runtime._ovp_wasm_value_count();
        const values = [...new Float32Array(runtime.wasmMemory.buffer, offset(runtime._ovp_wasm_values(), count * 4), count)];
        require(values.every(Number.isFinite), 'Physics output must stay finite');
        return Array.from({ length: count / 3 }, (_, i) => values.slice(i * 3, i * 3 + 3));
    };
    const scale = m => [0, 4, 8].map(row => Math.hypot(m[row], m[row + 1], m[row + 2]));
    const determinant = m => m[0] * (m[5] * m[10] - m[6] * m[9]) - m[1] * (m[4] * m[10] - m[6] * m[8]) + m[2] * (m[4] * m[9] - m[5] * m[8]);
    const report = { passed: false, runtime: 'official-ovphysx-cpu-wasm64', checks: [], publications: 0 };
    let stage = 0, ordinal = 1n;
    const publish = () => {
        check(runtime._ovp_wasm_publish(stage, ++ordinal), true);
        check(runtime._ovs_wasm_seal(stage, ordinal));
        ++report.publications;
    };
    const validate = () => {
        const rows = positions();
        require(rows.length === 2, 'Output fixture must have two rigid bodies');
        const body = matrix(stage, '/World/Parent/Body', ordinal);
        const mirrored = matrix(stage, '/World/Mirrored', ordinal);
        for (const [name, m, expectedScale, sign] of [['nested', body, [1, 6, 1], 1], ['mirrored', mirrored, [2, 3, 4], -1]]) {
            const position = rows.find(row => Math.abs(row[0] - m[12]) < 0.01);
            require(position, `${name} published matrix must identify a real actor`);
            for (let lane = 0; lane < 3; ++lane) {
                near(m[12 + lane], position[lane], `${name} publication matches current physics position`);
                near(scale(m)[lane], expectedScale[lane], `${name} publication preserves authored world scale`);
            }
            require(Math.sign(determinant(m)) === sign, `${name} publication preserves scale handedness`);
            const path = name === 'nested' ? '/World/Parent/Body' : '/World/Mirrored';
            const extent = matrix(stage, path, ordinal, '_worldExtent', 6);
            for (let axis = 0; axis < 3; ++axis) {
                const radius = Math.abs(m[axis]) + Math.abs(m[4 + axis]) + Math.abs(m[8 + axis]);
                near(extent[axis], m[12 + axis] - radius, `${name} world bound minimum follows its physics pose`);
                near(extent[axis + 3], m[12 + axis] + radius, `${name} world bound maximum follows its physics pose`);
            }
        }
        const marker = matrix(stage, '/World/Parent/Body/Marker', ordinal);
        for (let lane = 0; lane < 3; ++lane)
            near(marker[12 + lane], body[12 + lane] + body[4 + lane], 'Descendant hierarchy must follow the published body');
        const local = matrix(stage, '/World/Parent/Body', ordinal, 'omni:fabric:localMatrix');
        near(local[12], (body[12] - 10) / 2, 'Publication folds body world X into its parent-relative local transform');
        near(local[13], body[13] / 3, 'Publication folds body world Y into its parent-relative local transform');
        near(local[14], body[14] / 4, 'Publication folds body world Z into its parent-relative local transform');
        return body;
    };
    try {
        stage = createStage(sourceText);
        rejected(runtime._ovp_wasm_create(stage, 0n), 'Invalid initial attachment must fail');
        check(runtime._ovp_wasm_destroy(), true);
        check(runtime._ovp_wasm_create(stage, ordinal), true);
        for (let tick = 0; tick < 12; ++tick) {
            check(runtime._ovp_wasm_step(1 / 60), true);
            publish(); validate();
        }
        report.checks.push('Repeated publication matches current physics poses, signed and nested scale, descendant world matrices, current world bounds, and folded local transforms');
        const before = positions();
        const matrixBefore = matrix(stage, '/World/Parent/Body', ordinal);
        publish(); validate();
        require(JSON.stringify(positions()) === JSON.stringify(before), 'Publication must not advance simulation');
        const matrixAfter = matrix(stage, '/World/Parent/Body', ordinal);
        matrixAfter.forEach((value, i) => near(value, matrixBefore[i], 'Repeated publication without stepping keeps the matrix'));
        report.checks.push('Repeated publication does not step physics');
        rejected(runtime._ovp_wasm_publish(stage, ordinal), 'Publishing at a sealed write floor must reject');
        require(runtime._ovs_wasm_create() === 0, 'The WASM wrapper must reject a second simultaneous stage owner');
        rejected(runtime._ovp_wasm_publish(stage + 1, ordinal + 1n), 'Publishing through another stage identity must reject');
        rejected(runtime._ovp_wasm_publish(0, ordinal + 1n), 'Publishing to an invalid stage handle must reject');
        publish(); validate();
        report.checks.push('Rejected ordinals, second-owner creation, and invalid stage identities do not poison the valid attachment or cache');
        check(runtime._ovp_wasm_destroy(), true);
        rejected(runtime._ovp_wasm_publish(stage, ordinal + 1n), 'Publication after physics destruction must reject');
        check(runtime._ovp_wasm_create(stage, ordinal), true);
        check(runtime._ovp_wasm_step(1 / 60), true);
        publish(); validate();
        report.checks.push('Destroy/recreate on the same stage starts a fresh attachment-scoped output cache');
        check(runtime._ovp_wasm_destroy(), true);
        if (recreateStage) {
            check(runtime._ovs_wasm_destroy(stage)); stage = 0;
            const replacement = sourceText.replaceAll('"Mirrored"', '"Replacement"').replace('(-10, 12, 0)', '(-20, 20, 0)');
            stage = createStage(replacement); ordinal = 1n;
            check(runtime._ovp_wasm_create(stage, ordinal), true);
            check(runtime._ovp_wasm_step(1 / 60), true);
            publish();
            near(matrix(stage, '/World/Replacement', ordinal)[12], -20, 'New stage uses new prim membership');
            report.checks.push('A different stage in the same runtime publishes its own prim membership after owner destruction');
        } else {
            report.limitations = ['Second-stage-owner reuse was explicitly excluded; known TBB/provider lifecycle limitation.'];
        }
        report.passed = true;
        return report;
    } finally {
        try {
            check(runtime._ovp_wasm_destroy(), true);
            if (stage) check(runtime._ovs_wasm_destroy(stage));
        } finally { runtime.PThread.terminateAllThreads(); }
    }
}
