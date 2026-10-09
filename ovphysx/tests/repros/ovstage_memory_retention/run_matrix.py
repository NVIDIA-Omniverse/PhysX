#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
# @implements REQ-CAPI-IMPORT-MEMORY-REPRO-001
"""Run SDK-only memory controls in fresh processes; exit 1 on API/run failure."""
import argparse
import json
import os
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--binary', type=Path, default=Path('output/build/control_drain_repro'))
parser.add_argument('--output', type=Path, default=Path('output/results'))
args = parser.parse_args()
root = Path(__file__).resolve().parent
args.output.mkdir(parents=True, exist_ok=True)
cases = [('zero', {'ITERATIONS': 0}), ('pose-400', {'ITERATIONS': 400}),
         ('pose-2000', {'ITERATIONS': 2000}), ('pose-10000', {}),
         ('pose-10000-repeat', {}), ('pose-50000', {'ITERATIONS': 50000}),
         ('no-drain-50000', {'ITERATIONS': 50000, 'DRAIN': 0}),
         ('no-write-10000', {'WRITE': 0}), ('constant-10000', {'ALTERNATE': 0}),
         ('one-body-10000', {'BODIES': 1}), ('mass-10000', {'MASS': 1}),
         ('no-warmup-10000', {'WARMUP': 0})]
results = []
for name, overrides in cases:
    params = {'ITERATIONS': 10000, 'BODIES': 4, 'WRITE': 1, 'DRAIN': 1,
              'ALTERNATE': 1, 'MASS': 0, 'WARMUP': 1, **overrides}
    env = {k: v for k, v in os.environ.items() if not k.startswith('PROBE_')}
    env.update({'PROBE_' + k: str(v) for k, v in params.items()})
    command = [str(args.binary.resolve()), str(root/'scene.usda')]
    log = args.output/(name + '.log')
    with log.open('w') as stream:
        run = subprocess.run(command, env=env, stdout=stream, stderr=subprocess.STDOUT, timeout=300)
    samples = [json.loads(line[7:]) for line in log.read_text().splitlines() if line.startswith('MEMORY ')]
    entry = {'name': name, 'parameters': params, 'command': command, 'exit_code': run.returncode, 'samples': samples}
    if run.returncode == 0:
        ready = next(s for s in samples if s['phase'] == 'ready')
        last = next((s for s in reversed(samples) if s['phase'] == 'running'), ready)
        entry['growth_mib'] = {k: (last[k]-ready[k])/2**20 for k in ('allocated_bytes','mmap_bytes','rss_bytes')}
    results.append(entry)
    (args.output/'matrix.json').write_text(json.dumps(results, indent=2)+'\n')
    print(name, run.returncode, entry.get('growth_mib'), flush=True)
    if run.returncode:
        raise SystemExit(f'Run failed: {log}')
