# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-EVENTS-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Contact/trigger graph adapter: native collection, host-thread post-results dispatch."""
from pathlib import Path


def run():
    import ovphysx,ovstage
    from ovphysx_companion.preview import PreviewSession
    from ovphysx_companion.host import FrameCoordinator
    from ovphysx_companion.events import EventQueue,Mask,Kind,TriggerType,EventOverflow
    from ovphysx_companion.schemas import physx_schema_paths
    ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
    ovphysx.PhysX.set_cpu_mode(True)
    physics=ovphysx.PhysX();stage=ovstage.Stage('event-monitor')
    try:
        ovstage.population.open_usd(stage,str(Path(__file__).with_name('scenes')/'events.usda'),
            ordinal=1,domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        with PreviewSession(physics,stage,read_ordinal=1) as preview,FrameCoordinator(preview) as loop:
            # Subscribe before stepping. The source explicitly applies trigger and
            # contact-report APIs; monitoring does not mutate USD or solver flags.
            with EventQueue(preview,mask=Mask.CONTACT|Mask.TRIGGER|Mask.OBJECT|Mask.ERROR,filter_path='/World/BoxA') as events:
                graph_events=[]
                for frame in range(120):
                    try:
                        loop.tick(frame,1/60,after_step=lambda completed:events.dispatch(graph_events.append))
                    except EventOverflow:
                        # Pause presentation. Pair history is invalid: a real host
                        # offers explicit scene reset/rebuild and re-subscription.
                        loop.pause();raise
                assert any(e.trigger_type==TriggerType.ENTER for e in graph_events)
                assert any(e.kind==Kind.TRIGGER_STAY for e in graph_events)
                assert any(e.trigger_type==TriggerType.LEAVE for e in graph_events)
                assert any(e.kind==Kind.CONTACT for e in graph_events)
                return graph_events
    finally:
        physics.destroy();stage.destroy()


if __name__=='__main__':print(f'{len(run())} copied events delivered to the host')
