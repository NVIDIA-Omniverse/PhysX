# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-ATTACH-COMPUTE-001
# @covers AC-1 AC-2 AC-3 AC-4

"""One native stopped point-matching request over stdin and a dedicated JSON reply file.

The customer-USD sample uses this isolated process to keep its USD installation
independent of the native environment. Production hosts can keep a worker alive
and add their own transport, scheduling and cancellation policy.
"""
import json
from ovphysx_companion._worker import write_reply
import sys


def run(request):
    from ovphysx import _bindings
    from ovphysx_companion.native import match_points
    return match_points(_bindings._lib, request['source'], request['target'], request['distance'])


if __name__ == '__main__':
    write_reply(run(json.load(sys.stdin)))
