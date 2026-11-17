// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
import { createHash } from 'node:crypto';
import { readFile, writeFile } from 'node:fs/promises';
import { pathToFileURL } from 'node:url';
import { runSolverOptions } from './solver-options-runner.mjs';
const [modulePath, sidePath, reportPath] = process.argv.slice(2);
if (!modulePath || !sidePath) throw Error('Usage: test-runtime-solver-options.mjs main.mjs side.wasm [receipt.json]');
const moduleUrl = pathToFileURL(modulePath), sideUrl = pathToFileURL(sidePath);
const { default: createModule } = await import(moduleUrl);
const runtime = await createModule({ dynamicLibraries: [sideUrl.href], locateFile: file => new URL(file, moduleUrl).href });
const result = await runSolverOptions(runtime, await readFile(new URL('./output.usda', import.meta.url), 'utf8'));
const mainWasm = new URL(moduleUrl); mainWasm.pathname = mainWasm.pathname.replace(/\.m?js$/, '.wasm');
result.artifactSha256 = Object.fromEntries(await Promise.all([['ovstage.mjs',moduleUrl],['ovstage.wasm',mainWasm],['ovphysx.wasm',sideUrl]].map(async ([name,file]) => [name,createHash('sha256').update(await readFile(file)).digest('hex')])));
try {
    result.artifactSha256['compiled-artifacts.mjs'] = createHash('sha256').update(await readFile(new URL('compiled-artifacts.mjs', moduleUrl))).digest('hex');
} catch (error) { if (error.code !== 'ENOENT') throw error; }
if (reportPath) await writeFile(reportPath, JSON.stringify(result, null, 2) + '\n');
console.log(JSON.stringify({passed:true,workers:result.records.map(record => record.workers)}));
