// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { pathToFileURL } from 'node:url';
const [demoUrl, playwrightPath, reportPath] = process.argv.slice(2);
if (!demoUrl || !playwrightPath) throw new Error('Usage: test-browser.mjs <demo URL> <playwright-core index.mjs> [report.json]');
const { chromium } = await import(pathToFileURL(playwrightPath));
const browser = await chromium.launch({ executablePath: process.env.CHROME_BIN || '/usr/bin/google-chrome', headless: true,
    args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
try {
    const page = await browser.newPage();
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    await page.goto(demoUrl);
    const result = await page.evaluate(async () => {
        const { createNativePopulationStage } = await import('/packages/ovstage/dist/native/index.mjs');
        const moduleUrl = new URL('/artifacts/physics-population/ovstage-physics-population.mjs', location.href);
        const response = await fetch(new URL('physics.usda', moduleUrl));
        if (!response.ok) throw new Error(`Fixture HTTP ${response.status}`);
        const source = await response.text();
        let ticks = 0;
        const timer = setInterval(() => ticks++, 1);
        const stage = await createNativePopulationStage({ moduleUrl });
        try {
            await stage.openUsdFromString(source, { ordinal: 1n });
            await stage.seal(1n);
            const values = {};
            for (const name of ['physics:mass', 'physics:rigidBodyEnabled', 'physxRigidBody:angularDamping', 'usd-schemas']) {
                values[name] = Array.from((await stage.readAttribute('/World/Box', name, 1n)).data);
            }
            return { isolated: crossOriginIsolated, ticks, values };
        } finally { await stage.dispose(); clearInterval(timer); }
    });
    assert.equal(result.isolated, true);
    assert.ok(result.ticks > 0);
    assert.deepEqual(result.values['physics:mass'], [2]);
    assert.deepEqual(result.values['physics:rigidBodyEnabled'], [1]);
    assert.ok(Math.abs(result.values['physxRigidBody:angularDamping'][0] - 0.05) < 1e-7);
    assert.ok(result.values['usd-schemas'].includes('PhysxRigidBodyAPI'));
    assert.deepEqual(errors, []);
    const report = { passed: true, simulationRuntimeLinked: false, browser: browser.version(), ...result };
    if (reportPath) await writeFile(reportPath, JSON.stringify(report, null, 2) + '\n');
    console.log(JSON.stringify(report, null, 2));
} finally { await browser.close(); }
