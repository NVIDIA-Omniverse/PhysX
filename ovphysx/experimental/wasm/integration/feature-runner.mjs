// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

const requireCondition = (condition, message) => {
    if (!condition) throw new Error(message);
};

/** Execute one scene in its own stage/runtime owner using the original public C API. */
export async function runRuntimeFeature(runtime, fixtures, { scenario = 'contact' } = {}) {
    requireCondition(['contact', 'articulation'].includes(scenario), 'Unknown physics feature scenario');
    const offset = (pointer, bytes = 0) => {
        const value = Number(pointer);
        if (!Number.isSafeInteger(value) || value < 0 || value + bytes > runtime.wasmMemory.buffer.byteLength)
            throw new RangeError('Read outside shared WASM memory');
        return value;
    };
    const text = pointer => runtime.UTF8ToString(offset(pointer));
    const check = (status, physics = true) => {
        if (status) throw new Error(`${physics ? 'OvPhysX' : 'OVStage'} status ${status}: ${text(
            physics ? runtime._ovp_wasm_error() : runtime._ovs_wasm_error())}`);
    };
    const read = (kind, velocity = false) => {
        const readers = {
            rigid: runtime._ovp_wasm_read_vectors,
            link: runtime._ovp_wasm_read_links,
            joint: runtime._ovp_wasm_read_joints,
        };
        check(readers[kind](velocity ? 1 : 0));
        const count = runtime._ovp_wasm_value_count();
        const values = [...new Float32Array(runtime.wasmMemory.buffer,
            offset(runtime._ovp_wasm_values(), count * 4), count)];
        requireCondition(values.every(Number.isFinite), `${kind} values must be finite`);
        return values;
    };
    const step = (count, dt) => {
        for (let i = 0; i < count; ++i) check(runtime._ovp_wasm_step(dt));
    };
    const withStage = async (sourceText, test, rejectOrdinal = false) => {
        const stage = runtime._ovs_wasm_create();
        if (!stage) throw new Error(text(runtime._ovs_wasm_error()));
        try {
            const source = new TextEncoder().encode(sourceText);
            const pointer = BigInt(runtime._malloc(source.length + 1));
            try {
                new Uint8Array(runtime.wasmMemory.buffer, offset(pointer, source.length), source.length).set(source);
                check(runtime._ovs_wasm_open_usd_string(stage, pointer, source.length, 1n, 0), false);
            } finally { runtime._free(pointer); }
            check(runtime._ovs_wasm_seal(stage, 1n), false);
            let rejection;
            if (rejectOrdinal) {
                const status = runtime._ovp_wasm_create(stage, 0n);
                requireCondition(status !== 0, 'The official API must reject an unsealed zero attach ordinal');
                rejection = { status, message: text(runtime._ovp_wasm_error()) };
                check(runtime._ovp_wasm_destroy());
            }
            check(runtime._ovp_wasm_create(stage, 1n));
            return { ...test(), ...(rejection ? { rejectedZeroOrdinal: rejection } : {}) };
        } finally {
            // A failed detach must retain the stage for the caller's retry.
            check(runtime._ovp_wasm_destroy());
            check(runtime._ovs_wasm_destroy(stage), false);
        }
    };
    const contact = () => {
        const before = read('rigid');
        requireCondition(before.length === 3 && Math.abs(before[1] - 2) < 0.01,
            'Contact fixture must produce exactly one dynamic body at its authored height');
        step(240, 1 / 120);
        const position = read('rigid');
        const velocity = read('rigid', true);
        requireCondition(position.length === 3 && velocity.length === 3, 'Contact body must remain queryable');
        requireCondition(position[1] >= 0.47 && position[1] <= 0.54,
            `The body must rest on the actual floor at Y=0.5, got ${position[1]}`);
        requireCondition(Math.abs(velocity[1]) < 0.15,
            `The resting body must stop falling, got vertical velocity ${velocity[1]}`);
        requireCondition(Math.abs(position[0]) < 0.03 && Math.abs(position[2]) < 0.03,
            'The symmetric falling body must remain above the floor origin');
        return { before, position, velocity, steps: 240, dt: 1 / 120 };
    };
    const articulation = () => {
        const beforeLinks = read('link');
        const beforeJoints = read('joint');
        requireCondition(beforeLinks.length === 6, 'The articulation must have two real link rows');
        requireCondition(beforeJoints.length === 1 && Math.abs(beforeJoints[0]) < 0.05,
            'The articulation must have exactly one unlocked joint axis at its authored zero angle');
        step(10, 1 / 60);
        const movingJointVelocity = read('joint', true);
        requireCondition(movingJointVelocity.length === 1 && Math.abs(movingJointVelocity[0]) > 1,
            'The real hinge must gain angular velocity under gravity');
        step(20, 1 / 60);
        const links = read('link');
        const joints = read('joint');
        requireCondition(links.length === 6 && joints.length === 1, 'Articulation membership must remain stable');
        requireCondition(Math.abs(joints[0]) > 5 && Math.abs(joints[0]) < 82,
            `Joint position must move while respecting authored limits in degrees, got ${joints[0]}`);
        const rows = [links.slice(0, 3), links.slice(3, 6)];
        const distance = row => Math.hypot(row[0], row[1] - 2, row[2]);
        rows.sort((left, right) => distance(left) - distance(right));
        const [base, arm] = rows;
        requireCondition(distance(base) < 0.002, 'The world-fixed articulation base must remain at its anchor');
        requireCondition(arm[1] < 1.9 && Math.abs(distance(arm) - 0.5) < 0.025,
            'The arm must swing below its anchor while retaining the real hinge constraint');
        return { beforeLinks, beforeJoints, links, joints, movingJointVelocity,
            angularUnits: 'degrees', steps: 30, dt: 1 / 60 };
    };
    try {
        requireCondition(runtime._ovs_wasm_pointer_bytes() === 8, 'Expected genuine WASM64 stage/runtime image');
        const result = { passed: true, runtime: 'official-ovphysx-cpu-wasm64', scenario, checks: [] };
        if (scenario !== 'articulation') {
            result.contact = await withStage(fixtures.contact, contact, true);
            console.log(`Contact passed: resting Y=${result.contact.position[1]}, vy=${result.contact.velocity[1]}`);
            result.checks.push('actual rigid body floor collision and resting velocity',
                'rejected zero ordinal cleans up and permits valid attach');
        }
        if (scenario !== 'contact') {
            result.articulation = await withStage(fixtures.articulation, articulation);
            console.log(`Articulation passed: joint position=${result.articulation.joints[0]} degrees`);
            result.checks.push('actual two-link articulation and one joint DOF',
                'gravity-driven joint motion in documented degree units',
                'fixed base and hinge constraint remain enforced');
        }
        return result;
    } finally {
        runtime.PThread.terminateAllThreads();
    }
}

/** Reset through fresh owner modules; native stage host reinitialization is unsupported. */
export async function runRuntimeFeatures(createRuntime, fixtures) {
    requireCondition(typeof createRuntime === 'function', 'A fresh runtime factory is required');
    const memories = new WeakSet();
    const run = async scenario => {
        const runtime = await createRuntime();
        requireCondition(!memories.has(runtime.wasmMemory), 'Each scene must own independent WASM memory');
        memories.add(runtime.wasmMemory);
        return runRuntimeFeature(runtime, fixtures, { scenario });
    };
    const contact = await run('contact');
    const articulation = await run('articulation');
    const repeatedContact = await run('contact');
    for (const field of ['position', 'velocity']) {
        requireCondition(contact.contact[field].every((value, index) =>
            Math.abs(value - repeatedContact.contact[field][index]) < 0.0001),
        `Reset contact ${field} must reproduce the original outcome`);
    }
    return {
        passed: true,
        runtime: 'official-ovphysx-cpu-wasm64',
        scenario: 'all',
        checks: [...contact.checks, ...articulation.checks,
            'three complete lifecycles with a fresh stage/runtime owner module per scene',
            'reset to a fresh owner restores authored contact state and repeats the same collision'],
        contact: contact.contact,
        articulation: articulation.articulation,
        repeatedContact: repeatedContact.contact,
    };
}
