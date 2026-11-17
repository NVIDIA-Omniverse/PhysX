#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Isolate a metadata typedef collision introduced by Emscripten's SIMD headers."""
import hashlib
import json
from pathlib import Path

RELATIVE = 'physx/source/physxmetadata/core/include/PxMetaDataObjects.h'
PIN = '5ca03e7f44a39bfac447bcbd84cd11dd8abf5f2895a53e87e79aa542f014d357'


def prepare(source, destination):
    original = Path(source).resolve() / RELATIVE
    data = original.read_bytes()
    if hashlib.sha256(data).hexdigest() != PIN:
        raise RuntimeError('Pinned PhysX metadata header changed')
    anchor = 'typedef bool _Bool;'
    text = data.decode()
    if text.count(anchor) != 1:
        raise RuntimeError('Unexpected PhysX metadata bool typedef')
    prepared_text = text.replace(anchor, '#ifndef _Bool\n' + anchor + '\n#endif')
    variant = hashlib.sha256(prepared_text.encode()).hexdigest()
    destination = Path(destination).resolve() / variant
    destination.mkdir(parents=True, exist_ok=True)
    prepared = destination / original.name
    prepared.write_text(prepared_text)
    overlay = destination / 'metadata-vfs.json'
    overlay.write_text(json.dumps({'version': 0, 'roots': [
        {'type': 'file', 'name': str(original), 'external-contents': str(prepared),
         'use-external-name': False}]}, indent=2) + '\n')
    receipt = {'original': str(original), 'originalSha256': PIN, 'prepared': str(prepared),
               'preparedSha256': hashlib.sha256(prepared.read_bytes()).hexdigest(),
               'overlay': str(overlay), 'overlaySha256': hashlib.sha256(overlay.read_bytes()).hexdigest(),
               'scope': 'Skip redundant bool typedef only when _Bool is already a preprocessor alias'}
    (destination / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return receipt
