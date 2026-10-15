# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-CAMERA-001
# @covers AC-1 AC-2 AC-3 AC-4
from dataclasses import replace
import numpy as np
import pytest
from pxr import Sdf, Usd, UsdGeom
from ovphysx_companion.camera import FollowCamera, CameraPose
from ovphysx_companion.output import BodyTarget, BodyOutput
from ovphysx_companion.native import Identity
from ovphysx_companion.usd import EditSession, StaleResultError
from ovphysx_companion.usd.camera import CameraOutput, FollowCameraOutput


def test_native_follow_camera_sample():
    from camera_follow import run
    frames=run()
    assert len(frames)==6 and frames[-1][2]<frames[0][2]


def camera_scene():
    stage=Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage,.01)
    parent=UsdGeom.Xform.Define(stage,'/World')
    parent.AddTranslateOp().Set((100,0,0))
    camera=UsdGeom.Camera.Define(stage,'/World/Camera')
    camera.AddTranslateOp().Set((0,20,30))
    stage.DefinePrim('/World/Body','Xform')
    # Session layer is stronger than the original root camera transform order.
    layer=stage.GetSessionLayer()
    return stage,layer


def test_camera_world_conversion_selected_layer_restore_and_session_close():
    stage,layer=camera_scene()
    root_before=stage.GetRootLayer().ExportToString()
    before=layer.ExportToString()
    session=EditSession(stage)
    output=CameraOutput(session,'/World/Camera',layer=layer)
    source=session.source
    output.publish(CameraPose((2,1,3),(0,0,0,1)),source=source)
    world=UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath('/World/Camera'))
    np.testing.assert_allclose(world.ExtractTranslation(),(200,100,300))
    assert root_before==stage.GetRootLayer().ExportToString()
    assert session.source==source
    session.close()
    assert layer.ExportToString()==before
    with pytest.raises(RuntimeError,match='closed'):output.publish(CameraPose((1,2,3),(0,0,0,1)),source=source)


@pytest.mark.parametrize('trigger',['missing','recreated','disable','close'])
def test_follow_restore_on_missing_recreated_disable_and_stage_close(trigger):
    stage,layer=camera_scene()
    session=EditSession(stage)
    before=layer.ExportToString()
    target=BodyTarget(Identity(1,2),'/World/Body',session.source)
    state=BodyOutput(target,(100,0,0),(0,0,0,1),(0,0,0),1,.1,.01)
    controller=FollowCamera(target,offset=(0,0,5))
    adapter=FollowCameraOutput(session,'/World/Camera',controller,layer=layer)
    try:
        assert adapter.update(state)
        after=layer.ExportToString()
        assert after!=before and not adapter.update(state)
        if trigger=='missing':
            assert not adapter.update(None)
        elif trigger=='recreated':
            stage.RemovePrim('/World/Body');stage.DefinePrim('/World/Body','Xform')
            assert not adapter.update(replace(state,target=replace(target,source_revision=session.source),step=2))
        elif trigger=='disable':
            adapter.close()
        else:
            session.close()
        assert not controller.active
        assert layer.ExportToString()==before
        assert not adapter.update(replace(state,step=3))
    finally:
        adapter.close();session.close()


def test_camera_conflicting_layer_edit_is_not_overwritten():
    stage,layer=camera_scene()
    session=EditSession(stage)
    output=CameraOutput(session,'/World/Camera',layer=layer)
    output.publish(CameraPose((1,2,3),(0,0,0,1)),source=session.source)
    authored=layer.ExportToString()
    with Usd.EditContext(stage,layer):stage.DefinePrim('/UserEdit','Xform')
    changed=layer.ExportToString()
    with pytest.raises(StaleResultError):output.close()
    assert layer.ExportToString()==changed
    # Host resolves the conflict explicitly before retrying cleanup.
    layer.ImportFromString(authored)
    output.close();session.close()
