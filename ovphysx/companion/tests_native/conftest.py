# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Native bridge tests run separately from companion customer-USD tests."""

from pathlib import Path
import sys

import pytest

COMPANION = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(COMPANION / "python"))


@pytest.fixture(scope="session")
def physics():
    import ovphysx
    import ovstage
    from ovphysx_companion.schemas import physx_schema_paths
    schemas = [str(p) for p in physx_schema_paths()]
    schemas.append(str(COMPANION / "samples/native/schema/resources/plugInfo.json"))
    newton = ovphysx.newton_schema_root()
    if newton:
        schemas.append(str(newton))
    ovstage.population.register_usd_schemas(schemas)
    ovphysx.PhysX.set_cpu_mode(True)
    owner = ovphysx.PhysX()
    yield owner
    owner.destroy()


@pytest.fixture
def attached(physics):
    import ovstage
    stage = ovstage.Stage("companion-bridge-test")
    ovstage.population.open_usd(stage, str(COMPANION / "samples/scenes/two_boxes.usda"), ordinal=1,
                                domains=ovstage.PopulationDomain.ALL)
    stage.advance_write_floor(ordinal=1).wait()
    physics.wait_task(physics.attach_ovstage(stage, read_ordinal=1))
    physics.step(1 / 60)
    physics.wait_all()
    try:
        yield physics
    finally:
        physics.wait_task(physics.detach_ovstage())
        stage.destroy()
