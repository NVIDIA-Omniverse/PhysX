// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
import { runRuntimeSmoke } from './smoke-runner.mjs';
try {
    const parameters = new URL(import.meta.url).searchParams;
    const moduleUrl = new URL(parameters.get('module') || './ovstage-ovphysx.mjs', import.meta.url);
    const sideUrl = parameters.has('physics') ? new URL(parameters.get('physics'), moduleUrl).href : null;
    const { default: createModule } = await import(moduleUrl.href);
    const options = sideUrl ? {
        dynamicLibraries: [sideUrl],
        locateFile: file => new URL(file, moduleUrl).href,
    } : undefined;
    const response = await fetch(new URL('./physics.usda', import.meta.url));
    if (!response.ok) throw new Error(`Fixture HTTP ${response.status}`);
    const result = await runRuntimeSmoke(await createModule(options), await response.text());
    postMessage({ result });
} catch (error) { postMessage({ error: error?.stack || String(error) }); }
