# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-MASS-001
# @covers AC-3
# @implements REQ-COMP-PREVIEW-001
# @covers AC-2
"""Isolated native oracle for customer-USD mass and snapshot regressions."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))

if __name__=='__main__':
    import json
    import ovphysx
    import ovstage
    from ovphysx_companion.inspection import Inspector
    from ovphysx_companion.preview import PreviewSession
    from ovphysx_companion._worker import write_reply
    request=json.load(sys.stdin)
    from ovphysx_companion.schemas import physx_schema_paths
    ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
    ovphysx.PhysX.set_cpu_mode(True)
    physics=ovphysx.PhysX();stage=ovstage.Stage('native-mass-oracle')
    try:
        with open('scene.usda','w',encoding='utf-8') as scene:scene.write(request['scene'])
        ovstage.population.open_usd(stage,str(Path('scene.usda').resolve()),ordinal=1,domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        with PreviewSession(physics,stage,read_ordinal=1) as owner,Inspector(owner) as inspector:
            owner.step(.001)
            if 'probe' in request:
                from ovphysx_companion.queries import SceneQueries
                with SceneQueries(owner) as queries:
                    write_reply(dict(distances=[hit.distance for hit in queries.raycast(request['probe'],(0,0,-1),10).hits]))
            elif 'ray' in request:
                from ovphysx_companion.queries import SceneQueries
                with SceneQueries(owner) as queries:
                    hits=queries.raycast(request['ray'],(0,0,-1),10).hits
                    assert len(hits)==1 and hits[0].material
                    write_reply(dict(friction=inspector.read(hits[0].material).static_friction))
            else:
                state=inspector.read(inspector.resolve(request['path']))
                write_reply(dict(mass=state.mass,center=state.center_of_mass[:3],axes=state.center_of_mass[3:],diagonal=state.inertia))
    finally:physics.destroy();stage.destroy()
