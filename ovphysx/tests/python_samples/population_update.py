# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Apply later population changes to an attached PhysX scene.

Use a fresh ordinal for each change. The added USD file must have a
``defaultPrim`` and no ``PhysicsScene``. Choose a new absolute target path to
avoid replacing existing simulated objects and resetting their state.
"""

import ovstage


def drain_population_change(stage, physx, ordinal):
    ovstage.population.apply_usd_changes(stage, ordinal=ordinal)
    stage.advance_write_floor(ordinal=ordinal).wait()
    physx.wait_task(physx.update_from_ovstage(ordinal, ordinal))


def add_usd_file_and_drain(stage, physx, usd_path, target_path, ordinal):
    ovstage.population.add_usd_reference(stage, str(usd_path), target_path)
    drain_population_change(stage, physx, ordinal)
