// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
import { readFile, writeFile } from 'node:fs/promises';
import { pathToFileURL } from 'node:url';
import { runOutputPublication } from './output-runner.mjs';

const [modulePath, reportPath, sidePath] = process.argv.slice(2);
if (!modulePath) throw new Error('Usage: node test-runtime-output.mjs /path/to/ovstage.mjs [report.json] [/path/to/ovphysx.wasm]');
const moduleUrl = pathToFileURL(modulePath);
const { default: createModule } = await import(moduleUrl);
const runtime = await createModule(sidePath ? {
    dynamicLibraries: [pathToFileURL(sidePath).href],
    locateFile: file => new URL(file, moduleUrl).href,
} : undefined);
const result = await runOutputPublication(runtime, await readFile(new URL('./output.usda', import.meta.url), 'utf8'));
if (reportPath) await writeFile(reportPath, JSON.stringify(result, null, 2) + '\n');
console.log(JSON.stringify(result, null, 2));
