// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
import { readFile, writeFile } from 'node:fs/promises';
import { pathToFileURL } from 'node:url';
import { runRuntimeSmoke } from './smoke-runner.mjs';
const [modulePath = '/tmp/ovphysx-runtime-integration/ovstage-ovphysx.mjs', reportPath, sidePath] = process.argv.slice(2);
const moduleUrl = pathToFileURL(modulePath);
const sideUrl = sidePath ? pathToFileURL(sidePath).href : null;
const { default: createModule } = await import(moduleUrl);
const options = sideUrl ? {
    dynamicLibraries: [sideUrl],
    locateFile: file => new URL(file, moduleUrl).href,
} : undefined;
const result = await runRuntimeSmoke(await createModule(options), await readFile(new URL('./physics.usda', import.meta.url), 'utf8'));
if (reportPath) await writeFile(reportPath, JSON.stringify(result, null, 2) + '\n');
console.log(JSON.stringify(result, null, 2));
