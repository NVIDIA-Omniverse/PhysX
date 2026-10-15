# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-OVSTAGE-001
# @covers AC-1 AC-2 AC-3
"""Stageupdate adapter: sealed authored edits -> core step -> explicit output.

The host selects this world's output column and keeps it distinct from authored
USD transforms. To also publish customer USD, use a separate editor process with
EditSession(output_guard=OutputNotices(...).suppress); ordinals do not filter USD
notices. No customer USD module is imported by this worker.
"""
from pathlib import Path


def main():
    import ovphysx,ovstage,numpy as np
    from ovphysx_companion.native import ToolsSession
    from ovphysx_companion.inspection import Inspector
    from ovphysx_companion.exchange import StageFlow,OutputColumn
    from ovphysx_companion.host import FrameCoordinator
    from ovphysx_companion.schemas import physx_schema_paths
    schemas=[str(p) for p in physx_schema_paths()]
    if ovphysx.newton_schema_root():schemas.append(str(ovphysx.newton_schema_root()))
    ovstage.population.register_usd_schemas(schemas);ovphysx.PhysX.set_cpu_mode(True)
    physics=ovphysx.PhysX();stage=ovstage.Stage('explicit-flow')
    try:
        ovstage.population.open_usd(stage,str(Path(__file__).parent/'scenes/exchange.usda'),ordinal=1,domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait();physics.wait_task(physics.attach_ovstage(stage,read_ordinal=1))
        with ToolsSession(physics) as owner,StageFlow(owner,stage,consumed_ordinal=1) as flow,FrameCoordinator(owner) as host,Inspector(owner) as inspector:
            with ovstage.PathDictionary(stage) as dictionary:
                with dictionary.create_path_list_from_strings(['/World/BoxA']) as selected:
                    with stage.query_from_path_list(selected) as query:
                        stage.write_attribute(query,'physics:mass',2,np.array([6],dtype=np.float32),is_array=False).wait()
            stage.advance_write_floor(ordinal=2).wait()
            flow.consume(2)
            host.tick(1,.01)
            body=inspector.read(inspector.resolve('/World/BoxA'));assert body.mass==6
            flow.publish(3,['/World/BoxA'],[OutputColumn('companion:output:worldPosition',np.array([body.pose[:3]],dtype=np.float32),'world','stage',semantic=5)])
            assert not flow.consume(3) and flow.input_updates==1
            print('One authored input update; completed position copied at output ordinal 3; no feedback.')
        physics.wait_task(physics.detach_ovstage())
    finally:physics.destroy();stage.destroy()


if __name__=='__main__':main()
