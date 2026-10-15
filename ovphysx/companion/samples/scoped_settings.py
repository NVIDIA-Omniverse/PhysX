# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-CONFIG-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Settings-panel model: categorize controls, construct Inspector, inspect values.

Run normally for a native preview; --author demonstrates a customer-USD scene
settings command in a separate process. A real host supplies its preference store,
selected layer, and explicit input publication/recreation after scene edits.
"""
from pathlib import Path


def run():
    import ovphysx,ovstage
    from ovphysx_companion.configuration import (
        CATALOG,HostPreferences,AttachmentOptions,AttachmentConfiguration,Pipeline,Solver)
    from ovphysx_companion.schemas import physx_schema_paths
    ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
    ovphysx.PhysX.set_cpu_mode(True)
    physics=ovphysx.PhysX();stage=ovstage.Stage('settings-preview')
    try:
        ovstage.population.open_usd(stage,str(Path(__file__).with_name('scenes')/'two_boxes.usda'),
            ordinal=1,domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        store={};preferences=HostPreferences(store);preferences.set('show_debug_geometry',True)
        options=AttachmentOptions(threads=2,gpu_pipeline=Pipeline.CPU,solver=Solver.TGS)
        with options.open_preview(physics,stage,read_ordinal=1) as preview:
            config=AttachmentConfiguration(preview)
            config.set_contact_processing_disabled(True)
            snapshot=config.snapshot
            assert snapshot.threads==2 and snapshot.solver==Solver.TGS
            # UI can render scope/phase/persistence beside every exposed option.
            return {'threads':snapshot.threads,'solver':snapshot.solver.name,
                    'settings':len(CATALOG),'host_preferences':store}
    finally:
        physics.destroy();stage.destroy()


def author():
    from ovphysx.utils import codeless
    from ovphysx_companion.usd import EditSession
    from ovphysx_companion.usd.configuration import SceneOptions,apply_scene_options
    from ovphysx_companion.configuration import Solver
    codeless.register_schemas(verify=True)
    from pxr import Usd,UsdGeom,UsdPhysics
    stage=Usd.Stage.CreateInMemory();UsdGeom.SetStageMetersPerUnit(stage,.01)
    scene=UsdPhysics.Scene.Define(stage,'/Scene')
    with EditSession(stage) as edits:
        patch=apply_scene_options(edits,'/Scene',SceneOptions((0,0,-9.81),Solver.TGS,False),
                                  source=edits.source,layer=stage.GetRootLayer())
        patch.undo();patch.redo()
    assert abs(scene.GetGravityMagnitudeAttr().Get()-981)<.001
    return stage


if __name__=='__main__':
    import sys
    print(author().GetRootLayer().ExportToString() if '--author' in sys.argv else run())
