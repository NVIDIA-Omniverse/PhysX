# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-OVSTAGE-001
# @covers AC-3 AC-4
import numpy as np
import pytest
from ovphysx_companion.exchange import OutputNotices
from ovphysx_companion.usd import EditSession,capture_mesh,MeshFrame,write_simulation_output


def test_independent_usd_output_notice_suppression_and_failure(scene):
    from pxr import Tf,Usd
    stage,_=scene;inputs=[];gate=OutputNotices(lambda *args:inputs.append('authored'))
    listener=Tf.Notice.Register(Usd.Notice.ObjectsChanged,gate.notice,stage)
    try:
        stage.DefinePrim('/World/Authored','Xform')
        assert inputs;inputs.clear()
        with EditSession(stage,output_guard=gate.suppress) as session:
            snapshot=capture_mesh(session,'/World/A')
            frame=MeshFrame(snapshot,np.array(snapshot.points,dtype=np.float32)+1,1,.01)
            patch=write_simulation_output(session,[frame],layer=stage.GetSessionLayer(),time_code=None)
            patch.undo();patch.redo();assert not inputs
            before=stage.GetSessionLayer().ExportToString()
            def fail():
                stage.DefinePrim('/World/Partial','Xform')
                raise RuntimeError('injected')
            with pytest.raises(RuntimeError,match='injected'):session._apply(stage.GetSessionLayer(),fail,output=True)
            assert stage.GetSessionLayer().ExportToString()==before and not inputs
        stage.DefinePrim('/World/MoreInput','Xform');assert inputs
    finally:listener.Revoke()


@pytest.mark.parametrize('attribute',['physics:mass','points','xformOp:translate','drive:angular:physics:targetPosition',
                                       # Custom-physics inputs share the companion: prefix.
                                       'companion:stiffness','companion:radius','companion:output','companion:output:'])
def test_output_rejects_solver_inputs_before_publication(attribute):
    from ovphysx_companion.exchange import OutputColumn
    with pytest.raises(ValueError,match='reserved companion:output: namespace'):
        OutputColumn(attribute,np.array([100],dtype=np.float32),'body','stage')
    OutputColumn('companion:output:stiffness',np.array([100],dtype=np.float32),'joint','N/m')


def test_custom_physics_inputs_cannot_declare_output_namespace():
    from ovphysx_companion.schemas import ExternalSchema
    ExternalSchema('CompanionSpringJoint','joint',('companion:stiffness',))
    with pytest.raises(ValueError,match='reserved for published output'):
        ExternalSchema('CompanionSpringJoint','joint',('companion:stiffness','companion:output:stiffness'))
