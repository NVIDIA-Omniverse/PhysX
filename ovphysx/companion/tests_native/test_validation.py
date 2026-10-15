# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-VALIDATE-001
# @covers AC-1 AC-2 AC-3
from pathlib import Path
from ovphysx_companion.cooking import CookingService
from ovphysx_companion.descriptions import Attribute, Description, DescriptionResult
from ovphysx_companion.validation import validate_descriptions, ValidationPreview


def test_invalid_mesh_cooking_returns_identified_diagnostic(physics):
    mesh=Description('input-1','/Mesh','shape','Mesh',(),(
        Attribute('points','point3f[]',((0,0,0),(1,0,0),(0,1,0)),True),
        Attribute('faceVertexIndices','int[]',(0,1,999),True),
        Attribute('faceVertexCounts','int[]',(3,),True)),(),
        ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1)),1,1)
    snapshot=DescriptionResult('input-1',(mesh,))
    with CookingService(physics) as cooking:
        report=validate_descriptions(snapshot,checks=['collision-cook'],cooking=cooking)
    assert report.complete and not report.valid
    assert report.issues[0].code=='COLLIDER_COOK_FAILED'
    assert report.issues[0].source=='input-1' and report.issues[0].path=='/Mesh'
    assert report.issues[0].check.requires=='cooking'


def test_validation_preview_leaves_playback_receipt_and_owner_intact(attached):
    import ovstage
    from ovphysx_companion.native import ToolsSession
    stage=ovstage.Stage('validation-B')
    try:
        ovstage.population.open_usd(stage,str(Path(__file__).resolve().parents[1]/'samples/scenes/two_boxes.usda'),
                                    ordinal=1,domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        with ValidationPreview(attached,stage,source_revision='validation-1',read_ordinal=1) as validation:
            with ToolsSession(attached) as tools:
                assert tools.identity.attachment!=validation.preview.identity.attachment
            receipt=attached.step(.01)
            report=validation.check_motion(['/World/BoxA'],steps=3,dt=.1,max_displacement=.001)
            assert not report.valid and report.complete
            assert report.issues[0].code=='PREVIEW_EXCESSIVE_MOTION'
            assert report.issues[0].check.requires=='simulation'
        attached.wait_task(receipt)
        with ToolsSession(attached) as tools:
            assert tools.scene_statistics('/World/PhysicsScene').dynamic_bodies==2
    finally:
        stage.destroy()
