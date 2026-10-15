# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-PREVIEW-001
# @covers AC-1 AC-2 AC-3
"""Explicit CPU Inspector worker. Input USD belongs to ovstage's private population process."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--paths', nargs='+', required=True)
    parser.add_argument('--dt', type=float, default=1/60)
    parser.add_argument('--steps', type=int, default=1)
    args = parser.parse_args()
    import ovstage
    import ovphysx
    from .preview import PreviewSession
    from ovphysx_companion.schemas import physx_schema_paths
    schemas = [str(p) for p in physx_schema_paths()]
    if ovphysx.newton_schema_root():
        schemas.append(str(ovphysx.newton_schema_root()))
    ovstage.population.register_usd_schemas(schemas)
    ovphysx.PhysX.set_cpu_mode(True)
    physics = ovphysx.PhysX()
    stage = ovstage.Stage('inspector-worker')
    try:
        ovstage.population.open_usd(stage, args.scene, ordinal=1, domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        with PreviewSession(physics, stage, read_ordinal=1, gpu_pipeline=0) as preview:
            for _ in range(args.steps):
                preview.step(args.dt)
            result = {path: asdict(preview.rigid_pose(path)) for path in args.paths}
        Path(args.output).write_text(json.dumps(result), encoding='utf-8')
    finally:
        physics.destroy()
        stage.destroy()


if __name__ == '__main__':
    main()
