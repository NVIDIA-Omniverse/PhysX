#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Prepare the synchronous SDK-worker stepper for the bounded WASM host."""
import argparse
import hashlib
import json
from pathlib import Path

ANCHOR = """    const uint32_t wc = taskManager->getCpuDispatcher()->getWorkerCount();
    if (wc > 0)
    {
        omniPhysX.getITasking()->addTask(carb::tasking::Priority::eHigh, {}, [this] { run(); });
    }
    else
    {
        run();
    }
"""
REPLACEMENT = """    const uint32_t wc = taskManager->getCpuDispatcher()->getWorkerCount();
#if defined(__EMSCRIPTEN__)
    carb::tasking::ITasking* const tasking = omniPhysX.getITasking();
    bool synchronousSdkWorkers = false;
    if (wc > 0 && scene->getUpdateType() == eSynchronous &&
        scene->getAttachedStage().getSharedSettings().usePhysxDispatcher &&
        tasking && tasking->getDesc().threadCount == 0)
    {
        ovphysx_config_value_t execution{};
        const ovphysx_result_t status = ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION, &execution);
        synchronousSdkWorkers = status.status == OVPHYSX_API_SUCCESS &&
            execution.type == OVPHYSX_CONFIG_VALUE_BOOL && !execution.value.bool_value;
    }
    if (synchronousSdkWorkers)
    {
        // The host has no scheduler; simulate/fetch still dispatches solver work to the SDK workers.
        run();
    }
    else
#endif
    if (wc > 0)
    {
        omniPhysX.getITasking()->addTask(carb::tasking::Priority::eHigh, {}, [this] { run(); });
    }
    else
    {
        run();
    }
"""


def prepare_stepper(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    original = source.read_bytes()
    text = original.decode("utf-8")
    if text.count(ANCHOR) != 1:
        raise ValueError("PhysXStepper launch anchor changed; review the synchronous WASM adaptation")
    output.mkdir(parents=True, exist_ok=True)
    baseline = output / "PhysXScene.baseline.cpp"
    adapted = output / "PhysXScene.cpp"
    if source in (baseline, adapted):
        raise ValueError("The source and generated stepper paths must be distinct")
    baseline.write_bytes(original)
    include = '#include "PhysXScene.h"\n'
    if text.count(include) != 1:
        raise ValueError("PhysXScene header anchor changed")
    text = text.replace(include, include + '#if defined(__EMSCRIPTEN__)\n#include <ovphysx/ovphysx.h>\n#endif\n')
    adapted.write_text(text.replace(ANCHOR, REPLACEMENT))
    receipt = {
        "adaptation": "synchronous SDK solver workers with no Carbonite scheduler",
        "source": str(source),
        "sourceSha256": hashlib.sha256(original).hexdigest(),
        "baseline": str(baseline),
        "compiledSource": str(adapted),
        "compiledSourceSha256": hashlib.sha256(adapted.read_bytes()).hexdigest(),
        "recipeSha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "conditions": ["__EMSCRIPTEN__", "SDK worker count > 0", "ASYNC_EXECUTION == false",
                       "scene update type == synchronous", "native PhysX dispatcher selected",
                       "Carbonite host worker count == 0"],
        "unchanged": "Native builds, asynchronous requests, Carbonite dispatch and zero solver workers retain the original branch",
    }
    (output / "stepper-overlay.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return adapted


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(prepare_stepper(args.source, args.output))


if __name__ == "__main__":
    main()
