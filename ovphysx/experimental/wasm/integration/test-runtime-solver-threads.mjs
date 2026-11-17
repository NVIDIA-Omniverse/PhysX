// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFile, writeFile } from 'node:fs/promises';
import { pathToFileURL } from 'node:url';
import { runOutputPublication } from './output-runner.mjs';
import { runRuntimeFeature } from './feature-runner.mjs';
import { requireSolverCapacity } from './solver-capacity.mjs';

const [modulePath, sidePath, expectedText, reportPath] = process.argv.slice(2);
if (!modulePath || !sidePath || !expectedText)
    throw new Error('Usage: node test-runtime-solver-threads.mjs /path/to/ovstage.mjs /path/to/ovphysx.wasm 0|2|4 [report.json]');
const expected = Number(expectedText);
assert.ok([0, 2, 4].includes(expected));
assert.throws(() => requireSolverCapacity({ _ovp_wasm_solver_threads: () => 4, PThread: { unusedWorkers: [] } }), /idle pthread workers/);
assert.throws(() => requireSolverCapacity({ _ovp_wasm_solver_threads: () => 2 }), /owner-side/);
assert.deepEqual(requireSolverCapacity({ _ovp_wasm_solver_threads: () => 0 }), { requested: 0, available: null, required: 0 });
const moduleUrl = pathToFileURL(modulePath);
const { default: createModule } = await import(moduleUrl);
const workerCounts = runtime => ({ running: runtime.PThread.runningWorkers.length, unused: runtime.PThread.unusedWorkers.length });
const records = [];
const runtimePools = [];
const ownedRuntimes = new Set();
const mainWasmUrl = new URL(moduleUrl);
mainWasmUrl.pathname = mainWasmUrl.pathname.replace(/\.m?js$/, '.wasm');
const artifactSha256 = Object.fromEntries(await Promise.all([
    ['ovstage.mjs', moduleUrl], ['ovstage.wasm', mainWasmUrl], ['ovphysx.wasm', pathToFileURL(sidePath)],
].map(async ([name, path]) => [name, createHash('sha256').update(await readFile(path)).digest('hex')])));
const createRuntime = async () => {
    const options = { dynamicLibraries: [pathToFileURL(sidePath).href], locateFile: name => new URL(name, moduleUrl).href };
    options.onAbort = () => options.PThread?.terminateAllThreads();
    const runtime = await createModule(options);
    ownedRuntimes.add(runtime);
    assert.equal(runtime._ovp_wasm_solver_threads(), expected);
    runtimePools.push(workerCounts(runtime));
    const rawCreate = runtime._ovp_wasm_create;
    const rawDestroy = runtime._ovp_wasm_destroy;
    let attached = false;
    runtime._ovp_wasm_create = (...args) => {
        const capacity = requireSolverCapacity(runtime);
        const before = workerCounts(runtime);
        const status = rawCreate(...args);
        const after = workerCounts(runtime);
        if (!status) {
            attached = true;
            assert.ok(after.running >= before.running + expected,
                `Attachment must start ${expected} real SDK pthread workers; before ${before.running}, after ${after.running}`);
        }
        records.push({ operation: 'create', status, capacity, before, after });
        return status;
    };
    runtime._ovp_wasm_destroy = () => {
        const before = workerCounts(runtime), status = rawDestroy(), after = workerCounts(runtime);
        if (!status && attached) {
            assert.ok(after.running <= before.running - expected,
                `Detach must join ${expected} solver workers; before ${before.running}, after ${after.running}`);
            assert.ok(after.unused >= before.unused + expected, 'Joined solver workers return to the preallocated pool');
            attached = false;
        }
        records.push({ operation: 'destroy', status, before, after });
        return status;
    };
    return runtime;
};
const fixtures = Object.fromEntries(await Promise.all(['contact', 'articulation', 'sdf'].map(async name =>
    [name, await readFile(new URL(`./${name}.usda`, import.meta.url), 'utf8')])));
try {
const publication = await runOutputPublication(await createRuntime(), await readFile(new URL('./output.usda', import.meta.url), 'utf8'));
const features = [];
for (const scenario of ['contact', 'articulation', 'sdf'])
    features.push(await runRuntimeFeature(await createRuntime(), fixtures, { scenario }));
const successfulCreates = records.filter(record => record.operation === 'create' && record.status === 0);
assert.ok(successfulCreates.length >= 5, 'Exercise several fresh and recreated attachments');
const output = { passed: true, configuredSolverWorkers: expected, nativeGetter: { name: '_ovp_wasm_solver_threads', value: expected }, artifactSha256, runtimePools, dispatcher: expected ? 'SDK PxDefaultCpuDispatcher' : 'inline Carbonite dispatcher',
    publication, features, records,
    limitations: ['Physics tests do not establish complete same-module schema/material/render provider lifecycle.', 'Node qualification; browser-worker validation is still required.'],
    scope: 'Capacity rejection, actual pthread startup, fresh poses/bounds/hierarchy, contacts, articulation, serial SDF cooking and owner teardown; numerical trajectories are not asserted identical across thread counts' };
if (reportPath) await writeFile(reportPath, JSON.stringify(output, null, 2) + '\n');
console.log(JSON.stringify({ passed: true, configuredSolverWorkers: expected, successfulAttachments: successfulCreates.length }));

} catch (error) {
    if (reportPath) await writeFile(reportPath, JSON.stringify({ passed:false, configuredSolverWorkers:expected,
        nativeGetter:{name:'_ovp_wasm_solver_threads',value:expected}, artifactSha256, runtimePools, records,
        error:String(error), stack:error?.stack }, null, 2) + '\n');
    throw error;
} finally {
    for (const runtime of ownedRuntimes) runtime.PThread.terminateAllThreads();
}
