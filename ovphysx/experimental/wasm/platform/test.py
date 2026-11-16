#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Execute failure semantics of the built unavailable-CUDA adapter."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=Path('/tmp/ovphysx-wasm-canonical'))
    parser.add_argument('--emsdk', type=Path, default=Path('/tmp/ovstage-wasm-emsdk'))
    parser.add_argument('--stage-build', type=Path, default=Path('/tmp/ovstage-population-build'))
    parser.add_argument('--node', type=Path, default=Path('/tmp/ovstage-wasm-emsdk/node/24.19.0_64bit/bin/node'))
    args = parser.parse_args()
    destination = args.out / 'platform-test'
    destination.mkdir(exist_ok=True)
    inputs = args.out / 'runtime-objects'
    recorded = json.loads((inputs / 'platform_CudaUnavailable.json').read_text())['command']
    flags = [x for x in recorded[:-3] if x != '-c']
    stage_cuda = list((args.stage_build / 'native/CMakeFiles').rglob('CudaUnavailable.cpp.o'))
    if len(stage_cuda) != 1:
        raise ValueError('Expected exactly one real OVStage unavailable-CUDA provider object')
    output = destination / 'test.cjs'
    command = flags + [str(Path(__file__).with_name('test.cpp')),
                      str(inputs / 'platform_CudaUnavailable.o'), str(inputs / 'platform_CudaKernelsUnavailable.o'), str(stage_cuda[0]),
                      '-sENVIRONMENT=node', '-sASSERTIONS=2', '-sEXIT_RUNTIME=1', '-o', str(output)]
    env = dict(os.environ, EM_CONFIG=str(args.emsdk / '.emscripten'))
    with (destination / 'build.log').open('w') as log:
        built = subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT)
    if built.returncode:
        raise RuntimeError(f'CUDA failure test build failed; see {destination / "build.log"}')
    result = subprocess.run([str(args.node), str(output)], env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    (destination / 'test.log').write_text(result.stdout)
    (destination / 'receipt.json').write_text(json.dumps({'build': command, 'run': [str(args.node), str(output)],
                                                       'inputSha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in [stage_cuda[0], inputs / 'platform_CudaUnavailable.o', inputs / 'platform_CudaKernelsUnavailable.o', Path(__file__).with_name('test.cpp')]},
                                                       'exitCode': result.returncode, 'output': result.stdout}, indent=2) + '\n')
    print(result.stdout, end='')
    raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
