// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
import assert from 'node:assert/strict';
import { readFile, writeFile } from 'node:fs/promises';
import { pathToFileURL } from 'node:url';
const [modulePath = '/tmp/ovphysx-stage-integration/ovstage-physics-population.mjs',
    facadePath = '/home/rilei/dev/kit_2/rendering/ovstage/experimental/wasm/dist/native/index.mjs', reportPath] = process.argv.slice(2);
const { createNativePopulationStage } = await import(pathToFileURL(facadePath));
const stage = await createNativePopulationStage({ moduleUrl: pathToFileURL(modulePath) });
try {
    await stage.openUsdFromString(await readFile(new URL('./physics.usda', import.meta.url), 'utf8'), { ordinal: 1n });
    await stage.seal(1n);
    const read = async (path, name) => Array.from((await stage.readAttribute(path, name, 1n)).data);
    const attributes = {};
    for (const name of ['physics:mass', 'physics:rigidBodyEnabled', 'physics:collisionEnabled', 'physics:velocity',
        'physxRigidBody:linearDamping', 'physxRigidBody:angularDamping', 'usd-schemas']) {
        attributes[name] = await read('/World/Box', name);
    }
    assert.deepEqual(attributes['physics:mass'], [2]);
    assert.deepEqual(attributes['physics:rigidBodyEnabled'], [1]);
    assert.deepEqual(attributes['physics:collisionEnabled'], [1]);
    assert.deepEqual(attributes['physics:velocity'], [0, 0, 0]);
    assert.deepEqual(attributes['physxRigidBody:linearDamping'], [0]);
    assert.ok(Math.abs(attributes['physxRigidBody:angularDamping'][0] - 0.05) < 1e-7);
    for (const schema of ['PhysicsRigidBodyAPI', 'PhysicsCollisionAPI', 'PhysicsMassAPI', 'PhysxRigidBodyAPI']) {
        assert.ok(attributes['usd-schemas'].includes(schema), `Applied schema ${schema} must survive population`);
    }
    const gravity = await read('/World/Physics', 'physics:gravityMagnitude');
    assert.ok(Math.abs(gravity[0] - 9.81) < 1e-5);
    assert.deepEqual(await read('/', 'usd-metadata:metersPerUnit'), [1]);
    assert.deepEqual(await read('/', 'usd-metadata:upAxis'), ['Y']);
    const result = { passed: true, simulationRuntimeLinked: false, paths: await stage.queryPaths(), attributes, gravity,
        checks: ['official population description', 'official codeless schema defaults', 'authored mass and scene gravity',
            'applied schema identity', 'root units and axis', 'genuine native OVStage readback'] };
    if (reportPath) await writeFile(reportPath, JSON.stringify(result, null, 2) + '\n');
    console.log(JSON.stringify(result, null, 2));
} finally { await stage.dispose(); }
