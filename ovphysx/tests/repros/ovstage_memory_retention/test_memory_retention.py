#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
# @implements REQ-CAPI-IMPORT-MEMORY-REPRO-001
"""Fail when physics imports retain >4 MiB beyond the write-only control."""
import argparse
import json
import os
from pathlib import Path
import subprocess


def run_case(binary, output, drain):
    name = 'physics-update' if drain else 'write-only'
    params = dict(ITERATIONS=50000, BODIES=4, WRITE=1, DRAIN=int(drain),
                  ALTERNATE=1, MASS=0, WARMUP=1)
    env = {k: v for k, v in os.environ.items() if not k.startswith('PROBE_')}
    env.update({'PROBE_' + key: str(value) for key, value in params.items()})
    command = [str(binary.resolve()), str(Path(__file__).with_name('scene.usda'))]
    log = output / (name + '.log')
    with log.open('w') as stream:
        result = subprocess.run(command, env=env, stdout=stream,
                                stderr=subprocess.STDOUT, timeout=300)
    if result.returncode:
        raise RuntimeError(f'{name}: API/process failure ({result.returncode}); see {log}')
    samples = [json.loads(line[7:]) for line in log.read_text().splitlines()
               if line.startswith('MEMORY ')]
    start = next(s for s in samples if s['phase'] == 'running' and s['iteration'] == 10000)
    end = next(s for s in samples if s['phase'] == 'running' and s['iteration'] == 50000)
    allocated = lambda s: s['allocated_bytes'] + s['mmap_bytes']
    return dict(command=command, parameters=params, samples=samples,
                allocated_growth_bytes=allocated(end) - allocated(start),
                rss_growth_bytes=end['rss_bytes'] - start['rss_bytes'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path('output/memory-test'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    try:
        control = run_case(args.binary, args.output, False)
        imports = run_case(args.binary, args.output, True)
        excess = imports['allocated_growth_bytes'] - control['allocated_growth_bytes']
        limit = 4 * 2**20
        report = dict(control=control, physics_update=imports,
                      excess_allocated_bytes=excess, limit_bytes=limit,
                      passed=excess <= limit)
        (args.output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
        print(f"Write-only growth (10,000 to 50,000): {control['allocated_growth_bytes']/2**20:.3f} MiB")
        print(f"Physics-update growth: {imports['allocated_growth_bytes']/2**20:.3f} MiB")
        print(f'Excess retained allocation: {excess/2**20:.3f} MiB; limit: 4.000 MiB')
        print('PASS' if report['passed'] else 'FAIL: memory retention reproduced')
        return 0 if report['passed'] else 1
    except (OSError, RuntimeError, subprocess.TimeoutExpired, ValueError, StopIteration) as error:
        message = f'INVALID RUN: {error}'
        (args.output / 'error.txt').write_text(message + '\n')
        print(message)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
