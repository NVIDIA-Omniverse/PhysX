# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-CCT-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Headless CCT extension loop: host input, native movement, copied collisions.

The host owns the update clock and converts keyboard/gamepad input into world
movement. This sample uses a deterministic input sequence and an independent
preview attachment. No USD Python module, input service or UI is loaded.
"""
from pathlib import Path


def run():
    import ovphysx
    import ovstage
    from ovphysx_companion.preview import PreviewSession
    from ovphysx_companion.cct import CharacterController, Collision
    from ovphysx_companion.schemas import physx_schema_paths
    ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
    ovphysx.PhysX.set_cpu_mode(True)
    physics = ovphysx.PhysX()
    stage = ovstage.Stage('character-controller-sample')
    collisions = 0
    try:
        ovstage.population.open_usd(stage, str(Path(__file__).with_name('scenes')/'character.usda'),
                                    ordinal=1, domains=ovstage.PopulationDomain.PHYSICS |
                                    ovstage.PopulationDomain.RENDERING)
        stage.advance_write_floor(ordinal=1).wait()
        with PreviewSession(physics, stage, read_ordinal=1) as preview:
            with CharacterController(preview, '/World/Character') as controller:
                controller.set_gravity('scene')
                for frame in range(180):
                    # An extension supplies this velocity from its input policy.
                    velocity = (0, 0, 0) if frame < 90 else (3, 0, 0)
                    state = controller.move(tuple(v/60 for v in velocity), dt=1/60)
                    collisions |= state.collisions
                    preview.step(1/60)
                assert collisions & Collision.BELOW and collisions & Collision.SIDES
                assert state.position[0] < 3
                # Host camera code can consume this copied world-stage pose.
                position = state.position
            # Closing the controller removes its actor, but not its source prim.
            preview.step(1/60)
        return {'position': position, 'collisions': int(collisions)}
    finally:
        physics.destroy()
        stage.destroy()


if __name__ == '__main__':
    print(run())
