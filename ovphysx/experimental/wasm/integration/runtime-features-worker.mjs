// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
import { runRuntimeFeature, runRuntimeFeatures } from './feature-runner.mjs';
try {
    const parameters = new URL(import.meta.url).searchParams;
    const moduleUrl = new URL(parameters.get('module') || './ovstage-ovphysx.mjs', import.meta.url);
    const sideUrl = parameters.has('physics') ? new URL(parameters.get('physics'), moduleUrl).href : null;
    const { default: createModule } = await import(moduleUrl.href);
    const createRuntime = () => createModule(sideUrl ? {
        dynamicLibraries: [sideUrl],
        locateFile: file => new URL(file, moduleUrl).href,
    } : undefined);
    const fixtures = Object.fromEntries(await Promise.all(['contact', 'articulation', 'sdf'].map(async name => {
        const response = await fetch(new URL(`./${name}.usda`, import.meta.url));
        if (!response.ok) throw new Error(`Fixture HTTP ${response.status}`);
        return [name, await response.text()];
    })));
    const options = {
        ...(parameters.has('workers') ? { solverWorkers: Number(parameters.get('workers')) } : {}),
        ...(parameters.has('populationThreads') ? { populationThreads: Number(parameters.get('populationThreads')) } : {}),
    };
    const scenario = parameters.get('scenario') ?? 'all';
    const result = scenario === 'all' ? await runRuntimeFeatures(createRuntime, fixtures, options)
        : await runRuntimeFeature(await createRuntime(), fixtures, { ...options, scenario });
    postMessage({ result });
} catch (error) { postMessage({ error: error?.stack || String(error) }); }
