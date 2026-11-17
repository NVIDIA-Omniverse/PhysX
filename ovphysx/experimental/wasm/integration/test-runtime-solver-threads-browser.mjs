// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import http from 'node:http';
import { readFile, writeFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const [runtimeDirectory, expectedText, playwrightPath, reportPath, ownership = 'full'] = process.argv.slice(2);
if (!runtimeDirectory || !expectedText || !playwrightPath)
    throw Error('Usage: node test-runtime-solver-threads-browser.mjs runtime-dir 0|2|4 playwright-core/index.mjs [report.json] [full|single-owner]');
const expected = Number(expectedText);
assert.ok([0, 2, 4].includes(expected));
assert.ok(['full', 'single-owner', 'runtime-options'].includes(ownership));
const runtimeRoot = path.resolve(runtimeDirectory), testsRoot = path.dirname(fileURLToPath(import.meta.url));
const engine = process.env.OVPHYSX_BROWSER || 'chrome';
assert.ok(['chrome', 'firefox'].includes(engine));
const worker = `
import createModule from '/runtime/ovstage.mjs';
import { runOutputPublication } from '/tests/output-runner.mjs';
import { runRuntimeFeature } from '/tests/feature-runner.mjs';
import { runSolverOptions } from '/tests/solver-options-runner.mjs';
import { requireSolverCapacity } from '/tests/solver-capacity.mjs';
const expected = ${expected}, records = [], owned = new Set();
const check = (condition, message) => { if (!condition) throw Error(message); };
const counts = runtime => ({ running: runtime.PThread.runningWorkers.length, unused: runtime.PThread.unusedWorkers.length });
const create = async () => {
    const options = { dynamicLibraries: [new URL('/runtime/ovphysx.wasm', location.href).href], locateFile: name => new URL(name, new URL('/runtime/', location.href)).href };
    options.onAbort = () => options.PThread?.terminateAllThreads();
    const runtime = await createModule(options); owned.add(runtime);
    check(runtime._ovp_wasm_solver_threads() === expected, 'Configured solver count mismatch');
    const createNative = runtime._ovp_wasm_create, destroyNative = runtime._ovp_wasm_destroy;
    let attached = false;
    runtime._ovp_wasm_create = (...args) => {
        const capacity = requireSolverCapacity(runtime), before = counts(runtime);
        const status = createNative(...args), after = counts(runtime);
        if (!status) { attached = true; check(after.running >= before.running + expected, 'Real SDK pthread startup missing'); }
        records.push({ operation: 'create', status, capacity, before, after }); return status;
    };
    runtime._ovp_wasm_destroy = () => {
        const before = counts(runtime), status = destroyNative(), after = counts(runtime);
        if (!status && attached) {
            check(after.running <= before.running - expected, 'SDK worker join missing');
            check(after.unused >= before.unused + expected, 'Joined SDK workers did not return to pool'); attached = false;
        }
        records.push({ operation: 'destroy', status, before, after }); return status;
    };
    return runtime;
};
try {
    const fixtures = Object.fromEntries(await Promise.all(['output', 'contact', 'articulation', 'sdf'].map(async name => {
        const response = await fetch('/tests/' + name + '.usda'); check(response.ok, 'Fixture HTTP failure'); return [name, await response.text()];
    })));
    const scenario = new URL(location.href).searchParams.get('scenario');
    let publication = null, selection = null; const features = [];
    if (scenario === 'solver-options') selection = await runSolverOptions(await create(), fixtures.output);
    else {
        if (!scenario || scenario === 'publication') publication = await runOutputPublication(await create(), fixtures.output, { recreateStage: ${ownership === 'full'} });
        for (const feature of scenario && scenario !== 'publication' ? [scenario] : scenario ? [] : ['contact', 'articulation', 'sdf'])
            features.push(await runRuntimeFeature(await create(), fixtures, { scenario: feature }));
    }
    if (!scenario) check(records.filter(record => record.operation === 'create' && !record.status).length >= 5, 'Missing repeated attachment coverage');
    postMessage({ passed: true, isolated: crossOriginIsolated, nativeGetter: { name: '_ovp_wasm_solver_threads', value: expected }, scenario, records, publication, features, selection,
        limitations: ['Physics checks do not establish complete same-module schema/material/render lifecycle.'] });
} catch (error) { postMessage({ passed: false, error: String(error), stack: error?.stack, records }); }
finally { for (const runtime of owned) runtime.PThread.terminateAllThreads(); }
`;
const server = http.createServer(async (request, response) => {
    response.setHeader('Cross-Origin-Opener-Policy', 'same-origin');
    response.setHeader('Cross-Origin-Embedder-Policy', 'require-corp');
    try {
        const pathname = new URL(request.url, 'http://localhost').pathname;
        if (pathname === '/') { response.setHeader('Content-Type', 'text/html'); response.end('<!doctype html><title>SDK worker qualification</title>'); return; }
        if (pathname === '/favicon.ico') { response.writeHead(204).end(); return; }
        if (pathname === '/worker.mjs') { response.setHeader('Content-Type', 'text/javascript'); response.end(worker); return; }
        const prefix = pathname.startsWith('/runtime/') ? '/runtime/' : pathname.startsWith('/tests/') ? '/tests/' : null;
        if (!prefix) { response.writeHead(404).end(); return; }
        const root = prefix === '/runtime/' ? runtimeRoot : testsRoot, file = path.resolve(root, pathname.slice(prefix.length));
        if (!file.startsWith(root + '/')) { response.writeHead(403).end(); return; }
        response.setHeader('Content-Type', file.endsWith('.wasm') ? 'application/wasm' : file.endsWith('.mjs') ? 'text/javascript' : 'text/plain');
        response.end(await readFile(file));
    } catch { response.writeHead(404).end(); }
});
await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
let browser;
try {
    if (engine === 'firefox') {
        const { launchFirefox } = await import('./firefox-webdriver.mjs');
        browser = await launchFirefox({ logPath: reportPath ? reportPath + '.webdriver.log' : undefined });
    } else {
        const { chromium } = await import(pathToFileURL(playwrightPath));
        browser = await chromium.launch({ executablePath: process.env.CHROME_BIN || '/usr/bin/google-chrome', headless: true, args: ['--no-sandbox'] });
    }
    const page = await browser.newPage(), errors = [];
    page.on('pageerror', error => errors.push(String(error)));
    await page.goto('http://127.0.0.1:' + server.address().port);
    const result = await page.evaluate(async ownership => {
        const run = scenario => new Promise((resolve, reject) => {
            const worker = new Worker('/worker.mjs' + (scenario ? '?scenario=' + scenario : ''), { type: 'module' });
            let ticks = 0; const ticker = setInterval(() => ++ticks, 1);
            const finish = action => { clearTimeout(timeout); clearInterval(ticker); worker.terminate(); action(); };
            const timeout = setTimeout(() => finish(() => reject(Error('Solver worker exceeded 60 seconds'))), 60000);
            worker.onmessage = ({ data }) => finish(() => resolve({ ...data, uiTicks: ticks }));
            worker.onerror = event => finish(() => reject(Error((scenario || 'all') + ': ' + event.message)));
        });
        if (ownership === 'full') return run(null);
        if (ownership === 'runtime-options') return run('solver-options');
        const scenarios = [];
        for (const scenario of ['publication', 'contact', 'articulation', 'sdf']) {
            const result = await run(scenario); scenarios.push(result);
            if (!result.passed) return { ...result, scenarios };
        }
        return { passed: true, isolated: scenarios.every(result => result.isolated), scenarios,
            nativeGetter: scenarios[0].nativeGetter, uiTicks: scenarios.reduce((sum, result) => sum + result.uiTicks, 0),
            records: scenarios.flatMap(result => result.records), publication: scenarios[0].publication,
            features: scenarios.flatMap(result => result.features),
            limitations: ['One native module and one USD stage per dedicated worker; raw module/worker reuse is excluded.'] };
    }, ownership);
    const artifactSha256 = Object.fromEntries(await Promise.all(['ovstage.mjs', 'ovstage.wasm', 'ovphysx.wasm'].map(async name => [name, createHash('sha256').update(await readFile(path.join(runtimeRoot, name))).digest('hex')])));
    try {
        artifactSha256['compiled-artifacts.mjs'] = createHash('sha256').update(await readFile(path.join(runtimeRoot, 'compiled-artifacts.mjs'))).digest('hex');
    } catch (error) { if (error.code !== 'ENOENT') throw error; }
    const report = { ...result, ownership, engine, artifactSha256, browser: await browser.version(), errors };
    if (reportPath) await writeFile(reportPath, JSON.stringify(report, null, 2) + '\n');
    assert.equal(result.passed, true, JSON.stringify(report));
    assert.equal(result.isolated, true); assert.ok(result.uiTicks > 0, 'Browser UI must remain responsive');
    assert.deepEqual(errors, []);
    console.log(JSON.stringify({ passed: true, configuredSolverWorkers: expected, browser: report.browser, uiTicks: result.uiTicks }));
} finally { await browser?.close(); server.closeAllConnections(); await new Promise(resolve => server.close(resolve)); }
