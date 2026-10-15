# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-MASS-001
# @covers AC-1 AC-2 AC-3
# @implements REQ-COMP-JOBS-001
# @covers AC-1 AC-2 AC-4

"""Stopped property-panel flow: copied shapes -> native job -> display data.

No stage is attached or stepped. The host captures shapes in SI/body-local
coordinates and retains its source revision beside the job. Before applying
results to customer USD it must check that revision through EditSession.validate.
This sample prints JSON and can run separately from a customer-USD editor process.
"""

from dataclasses import asdict
import json

from ovphysx_companion.jobs import AuthoringService
from ovphysx_companion.native import MassShape, PrimitiveKind


def run():
    # Bindings load the installed core library; no runtime is initialized.
    from ovphysx import _bindings

    shapes = [MassShape(PrimitiveKind.BOX, (1, 1, 1), 0.125, (x, 0, 0)) for x in (-2, 2)]
    with AuthoringService(_bindings._lib) as authoring:
        job = authoring.compute_mass_properties(shapes)
        result = job.result(timeout=10)
        assert result.mass == 2
        assert result.center_of_mass == (0, 0, 0)
        return result


if __name__ == "__main__":
    print(json.dumps(asdict(run()), indent=2))
