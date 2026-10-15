# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-VALIDATE-001
# @covers AC-1 AC-2 AC-4
"""Asset-validator UI flow: inspect, select a repair, apply and undo/redo."""
def run():
    from ovphysx.utils import codeless
    codeless.register_schemas(verify=True)
    from pxr import Gf, Usd, UsdPhysics
    from ovphysx_companion.usd import EditSession, StaleResultError
    from ovphysx_companion.usd.descriptions import capture_descriptions
    from ovphysx_companion.validation import validate_descriptions
    from ovphysx_companion.usd.validation import apply_repairs
    stage=Usd.Stage.CreateInMemory()
    joint=UsdPhysics.FixedJoint.Define(stage,'/Joint')
    joint.CreateLocalRot1Attr(Gf.Quatf(2))
    with EditSession(stage) as session:
        report=validate_descriptions(capture_descriptions(session))
        assert len(report.issues)==1 and report.issues[0].check.requires=='usd-data'
        patch=apply_repairs(session,report,[0],layer=stage.GetRootLayer())
        assert validate_descriptions(capture_descriptions(session)).valid
        patch.undo()
        assert joint.GetLocalRot1Attr().Get().GetReal()==2
        patch.redo()
        try:
            apply_repairs(session,report,[0],layer=stage.GetRootLayer())
        except StaleResultError:
            pass
        else:
            raise AssertionError('A stale repair was applied')
    return stage


if __name__=='__main__':
    print(run().GetRootLayer().ExportToString())
