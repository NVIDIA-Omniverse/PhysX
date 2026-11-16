// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
import { readFile, writeFile } from 'node:fs/promises';
import { pathToFileURL } from 'node:url';
import { runRuntimeFeature, runRuntimeFeatures } from './feature-runner.mjs';

const [modulePath = '/tmp/ovphysx-runtime-integration/ovstage-ovphysx.mjs', reportPath, scenario = 'all', sidePath] = process.argv.slice(2);
const moduleUrl = pathToFileURL(modulePath);
const sideUrl = sidePath ? pathToFileURL(sidePath).href : null;
const { default: createModule } = await import(moduleUrl);
const createRuntime = () => createModule(sideUrl ? {
    dynamicLibraries: [sideUrl],
    locateFile: file => new URL(file, moduleUrl).href,
} : undefined);
const fixtures = Object.fromEntries(await Promise.all(['contact', 'articulation'].map(async name =>
    [name, await readFile(new URL(`./${name}.usda`, import.meta.url), 'utf8')])));
const result = scenario === 'all'
    ? await runRuntimeFeatures(createRuntime, fixtures)
    : await runRuntimeFeature(await createRuntime(), fixtures, { scenario });
if (reportPath) await writeFile(reportPath, JSON.stringify(result, null, 2) + '\n');
console.log(JSON.stringify(result, null, 2));
