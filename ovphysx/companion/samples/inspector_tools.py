# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-QUERY-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Inspector/viewport workflow: pick, display mass/joint/DOF state, step only B.

An extension supplies its selected native owner and UI rendering. This sample
uses a standalone CPU preview. Customer USD editing can run separately and apply
copied results through ovphysx_companion.usd; no USD is imported in this worker.
"""
from pathlib import Path


def inspect_selection(owner):
    from ovphysx_companion.queries import SceneQueries,Geometry,Mode
    from ovphysx_companion.inspection import ObjectKind
    from ovphysx_companion.host import cpu_tool_availability
    assert cpu_tool_availability(owner,'scene-queries').available
    with SceneQueries(owner) as tools:
        selection=tools.raycast((4,0,10),(0,0,-1),20)
        body=tools.read(selection.hits[0].body)
        name=tools.path(body.identity)
        neighbors=tools.overlap(Geometry.sphere(1),body.pose[:3],exclude=[body.identity])
        joint=tools.read(tools.resolve('/World/Hinge',ObjectKind.JOINT))
        articulation=tools.read(tools.resolve('/World/Robot',ObjectKind.ARTICULATION))
        # UI reads only these immutable copies after native admission has ended.
        panel={'selection':name,'mass':body.mass,'inertia':body.inertia,'nearby':neighbors.total,
               'joint_world_frames':(joint.world0,joint.world1),
               'dofs':tuple((d.axis,d.position,d.velocity) for d in articulation.dofs)}
        assert panel['selection']=='/World/Body' and panel['mass']==6 and len(panel['dofs'])==1
        return panel


def main():
    import ovphysx,ovstage
    from ovphysx_companion.preview import PreviewSession
    from ovphysx_companion.schemas import physx_schema_paths
    ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
    ovphysx.PhysX.set_cpu_mode(True)
    physics=ovphysx.PhysX();stage=ovstage.Stage('inspector-tools')
    try:
        ovstage.population.open_usd(stage,str(Path(__file__).parent/'scenes/inspection.usda'),ordinal=1,
                                    domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        with PreviewSession(physics,stage,read_ordinal=1) as preview:
            for _ in range(30):preview.step(1/120)
            print(inspect_selection(preview))
    finally:stage.destroy();physics.destroy()


if __name__=='__main__':main()
