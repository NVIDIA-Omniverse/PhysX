// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { pathToFileURL } from 'node:url';
const [demoUrl, playwrightPath, reportPath, workerName = 'runtime-smoke-worker.mjs', moduleUrl, sideUrl] = process.argv.slice(2);
if (!demoUrl || !playwrightPath) throw new Error('Usage: test-runtime-browser.mjs <demo URL> <playwright-core index.mjs> [report.json] [worker URL] [module URL] [physics WASM URL]');
const { chromium } = await import(pathToFileURL(playwrightPath));
const browser = await chromium.launch({ executablePath: process.env.CHROME_BIN || '/usr/bin/google-chrome', headless: true,
    args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
try {
    const page = await browser.newPage();
    const requests = [];
    page.on('request', request => requests.push(request.url()));
    page.on('console', message => console.log(`[browser ${message.type()}] ${message.text()}`));
    await page.goto(demoUrl);
    const result = await page.evaluate(async ({ workerName, moduleUrl, sideUrl }) => {
        const workerUrl = new URL(workerName, new URL('/artifacts/physics-runtime/', location.href));
        if (moduleUrl) workerUrl.searchParams.set('module', new URL(moduleUrl, workerUrl).href);
        const physicsUrl = sideUrl ? new URL(sideUrl, workerUrl).href : null;
        if (physicsUrl) workerUrl.searchParams.set('physics', physicsUrl);
        const worker = new Worker(workerUrl, { type: 'module' });
        let ticks = 0;
        const ticker = setInterval(() => ++ticks, 1);
        try {
            const result = await new Promise((resolve, reject) => {
                const timeout = setTimeout(() => reject(new Error('Physics worker exceeded 90 seconds')), 90000);
                worker.onmessage = ({ data }) => { clearTimeout(timeout); data.error ? reject(new Error(data.error)) : resolve(data.result); };
                worker.onerror = event => { clearTimeout(timeout); reject(new Error(event.message)); };
            });
            return { ...result, isolated: crossOriginIsolated, ticks, physicsUrl };
        } finally { clearInterval(ticker); worker.terminate(); }
    }, { workerName, moduleUrl, sideUrl });
    assert.equal(result.passed, true);
    assert.equal(result.isolated, true);
    assert.ok(result.ticks > 0, 'The UI must remain responsive during actual CPU simulation');
    if (sideUrl) assert.ok(requests.includes(result.physicsUrl), 'The browser must fetch the selected physics side module');
    const report = { ...result, browser: browser.version(), wasmRequests: requests.filter(url => new URL(url).pathname.endsWith('.wasm')) };
    if (reportPath) await writeFile(reportPath, JSON.stringify(report, null, 2) + '\n');
    console.log(JSON.stringify(report, null, 2));
} finally { await browser.close(); }
