# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-NATIVE-ABI-001
# @covers AC-1 AC-3 AC-4
# @implements REQ-COMP-JOINT-001
# @covers AC-1 AC-2 AC-3
# @implements REQ-COMP-GEOMETRY-001
# @covers AC-2 AC-3
"""Headless equivalent of custom joint/geometry extension registration.

Build samples/native first, then run this in the native population process:
    python native_extensions.py /absolute/path/to/plugin-build
Customer USD authoring can use external_schema_authoring.py in a separate process.
"""
from pathlib import Path
import json
import sys
from external_schema_authoring import sample_bundle
from ovphysx_companion.schemas import register_population_schemas, population_description
from ovphysx_companion.plugins import NativePlugin
from ovphysx_companion.preview import PreviewSession


def main(directory):
    import ovphysx
    import ovstage
    directory = Path(directory).resolve()
    suffix = '.dll' if sys.platform == 'win32' else '.so'
    def plugin(name):
        matches = list(directory.rglob('*'+name+suffix))
        if len(matches) != 1:
            raise ValueError(f'Expected one compiled {name} module in {directory}')
        return matches[0]
    bundle = sample_bundle()
    register_population_schemas([bundle])
    ovphysx.PhysX.set_cpu_mode(True)
    physics = ovphysx.PhysX()
    try:
        with NativePlugin(physics, plugin('companion_spring')) as spring, NativePlugin(physics, plugin('companion_ball')):
            with ovstage.Stage('custom-extension-sample') as stage:
                ovstage.population.open_usd_with_desc(stage, str(Path(__file__).parent/'scenes/native_extensions.usda'),
                    ordinal=1, time=0, descs=[population_description([bundle])])
                stage.advance_write_floor(ordinal=1).wait()
                with PreviewSession(physics, stage, read_ordinal=1, gpu_pipeline=0) as owner:
                    owner.step(1/120)
                    spring.set_joint_always_update(owner.identity, '/World/Spring', True)
                    spring.refresh_joint(owner.identity, '/World/Spring')
                    for _ in range(180):
                        owner.step(1/120)
                    result = {'spring_body': owner.rigid_pose('/World/Body').position,
                              'ball': owner.rigid_pose('/World/Ball').position}
                    assert result['spring_body'][0] < 2.5
                    assert .70 < result['ball'][1] < .85
                # Owner and shapes are now gone: exiting module contexts can unload.
    finally:
        physics.destroy()
    return result


if __name__ == '__main__':
    print(json.dumps(main(sys.argv[1])))
