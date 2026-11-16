#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Generate explicit unavailable GPU entries using the official kernel signatures."""
import hashlib
import json
from pathlib import Path
import re


def generate(source, output):
    header = Path(source) / 'ovphysx/ovruntime/source/omni.physx/plugins/tensors/gpu/CudaKernels.h'
    original = header.read_text()
    text = re.sub(r'/\*.*?\*/|//[^\n]*', '', original, flags=re.S)
    declarations = re.findall(r'((?:bool|void|int|::physx::PxU32)\s+[A-Za-z_][A-Za-z_0-9]*\s*\([^;{}]*\))\s*;', text, re.S)
    if not declarations:
        raise ValueError('No official CUDA kernel declarations found')
    definitions = []
    for declaration in declarations:
        signature = re.sub(r'\s*=\s*[^,)]+', '', declaration)
        name = signature.split('(')[0].split()[-1]
        definitions.append(signature + '\n{\n    throw std::runtime_error("OvPhysX WASM CPU runtime has no CUDA kernel: ' + name + '");\n}\n')
    destination = Path(output) / 'CudaKernelsUnavailable.cpp'
    destination.write_text('// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.\n'
                           '// SPDX-License-Identifier: Apache-2.0\n'
                           '// Generated from the pinned official CUDA kernel declarations.\n'
                           '#include "tensors/gpu/CudaKernels.h"\n#include <stdexcept>\n'
                           '#if !defined(__EMSCRIPTEN__)\n#error "Browser-only unavailable CUDA adapter"\n#endif\n'
                           'namespace omni::physx::tensors\n{\n' + '\n'.join(definitions) + '}\n')
    (Path(output) / 'cuda-unavailable-receipt.json').write_text(json.dumps({
        'header': str(header), 'headerSha256': hashlib.sha256(original.encode()).hexdigest(),
        'generated': str(destination), 'generatedSha256': hashlib.sha256(destination.read_bytes()).hexdigest(),
        'kernels': [x.split('(')[0].split()[-1] for x in declarations],
        'behavior': 'Every entry throws before reading or writing any argument. No GPU compute is implemented.'}, indent=2) + '\n')
    return destination
