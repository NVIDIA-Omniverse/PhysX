// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
const assert = {
    equal(actual, expected, message = 'Unexpected value') {
        if (!Object.is(actual, expected)) throw new Error(`${message}: ${actual} !== ${expected}`);
    },
    ok(value, message = 'Assertion failed') { if (!value) throw new Error(message); },
};

/** Exercise genuine population, CPU simulation, output publication and destruction. */
export async function runRuntimeSmoke(runtime, sourceText) {
const offset = pointer => {
    const value = Number(pointer);
    if (!Number.isSafeInteger(value) || value < 0 || value > runtime.wasmMemory.buffer.byteLength)
        throw new RangeError('Pointer outside WASM memory');
    return value;
};
const text = pointer => runtime.UTF8ToString(offset(pointer));
const check = (status, physics = false) => {
    if (status) throw new Error(`${physics ? 'OvPhysX' : 'OVStage'} status ${status}: ${text(physics ? runtime._ovp_wasm_error() : runtime._ovs_wasm_error())}`);
};
let stage = 0, physics = false;
try {
    assert.equal(runtime._ovs_wasm_pointer_bytes(), 8);
    console.log('Runtime smoke: creating stage');
    stage = runtime._ovs_wasm_create();
    if (!stage) throw new Error(text(runtime._ovs_wasm_error()));
    const source = new TextEncoder().encode(sourceText);
    const pointer = BigInt(runtime._malloc(source.length + 1));
    try {
        new Uint8Array(runtime.wasmMemory.buffer, offset(pointer), source.length).set(source);
        check(runtime._ovs_wasm_open_usd_string(stage, pointer, source.length, 1n, 0));
    } finally { runtime._free(pointer); }
    check(runtime._ovs_wasm_seal(stage, 1n));
    console.log('Native stage populated and sealed; starting official OvPhysX');
    check(runtime._ovp_wasm_create(stage, 1n), true);
    physics = true;
    const vectors = velocity => {
        check(runtime._ovp_wasm_read_vectors(velocity ? 1 : 0), true);
        const count = runtime._ovp_wasm_value_count();
        return [...new Float32Array(runtime.wasmMemory.buffer, offset(runtime._ovp_wasm_values()), count)];
    };
    console.log('Runtime smoke: attached, reading initial vectors');
    const before = { positions: vectors(false), velocities: vectors(true) };
    assert.equal(before.positions.length, 3, 'The fixture must produce one actual rigid body');
    assert.ok(Math.abs(before.positions[1] - 2) < 0.01);
    console.log('Runtime smoke: stepping');
    for (let step = 0; step < 30; ++step) check(runtime._ovp_wasm_step(1 / 60), true);
    const after = { positions: vectors(false), velocities: vectors(true) };
    assert.equal(after.positions.length, 3);
    assert.ok(after.positions[1] < before.positions[1] - 0.5, 'The real rigid body must fall under gravity');
    assert.ok(after.velocities[1] < -4, 'The real rigid body must gain downward velocity');
    console.log('Runtime smoke: publishing output');
    check(runtime._ovp_wasm_publish(stage, 2n), true);
    check(runtime._ovs_wasm_seal(stage, 2n));
    const strings = ['/World/Box', 'omni:fabric:worldMatrix'].map(value => new TextEncoder().encode(value));
    const pointers = strings.map(bytes => BigInt(runtime._malloc(bytes.length + 1)));
    let publishedMatrix;
    try {
        strings.forEach((bytes, index) => new Uint8Array(runtime.wasmMemory.buffer, offset(pointers[index]), bytes.length).set(bytes));
        check(runtime._ovs_wasm_read_attribute(stage, pointers[0], strings[0].length, pointers[1], strings[1].length, 2n));
        assert.equal(runtime._ovs_wasm_result_code(), 2);
        assert.equal(runtime._ovs_wasm_result_count(), 16);
        const ArrayType = runtime._ovs_wasm_result_bits() === 64 ? Float64Array : Float32Array;
        publishedMatrix = [...new ArrayType(runtime.wasmMemory.buffer, offset(runtime._ovs_wasm_result_data()), 16)];
        assert.ok(Math.abs(publishedMatrix[13] - after.positions[1]) < 1e-5, 'OVStage world matrix must match actual physics output');
    } finally { pointers.forEach(pointer => runtime._free(pointer)); }
    console.log('Runtime smoke: destroying physics');
    check(runtime._ovp_wasm_destroy(), true); physics = false;
    console.log('Runtime smoke: destroying stage');
    check(runtime._ovs_wasm_destroy(stage)); stage = 0;
    const result = { passed: true, runtime: 'official-ovphysx-cpu-wasm64', before, after, publishedMatrix, steps: 30, dt: 1 / 60,
        checks: ['real native population', 'official initialize/create/attach', 'nonempty rigid-body output',
            'official CPU stepping', 'pose and velocity change', 'official CPU output published to OVStage', 'read/query/group lifetime', 'detach/destroy/shutdown'] };
    return result;
} finally {
    console.log('Runtime smoke: final cleanup');
    try {
        check(runtime._ovp_wasm_destroy(), true);
        if (stage) check(runtime._ovs_wasm_destroy(stage));
    } finally { runtime.PThread.terminateAllThreads(); }
}
}
