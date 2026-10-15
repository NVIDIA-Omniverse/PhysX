# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Customer USD only: selection, progress, cancellation and one undo command."""
from ovphysx_companion.usd import EditSession
from ovphysx_companion.usd.colliders import ColliderBatch


def main():
    from pxr import Usd, UsdGeom, UsdPhysics
    from ovphysx.utils import codeless
    codeless.register_schemas(verify=True)
    stage=Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage,.01)
    selected=[f'/Box{i}' for i in range(4)]
    for path in selected:UsdGeom.Cube.Define(stage,path)
    with EditSession(stage) as session:
        cancelled=ColliderBatch(session,selected,layer=stage.GetSessionLayer())
        cancelled.advance(1);assert cancelled.cancel()
        command=ColliderBatch(session,selected,layer=stage.GetSessionLayer(),contact_offset_meters=.02)
        while command.progress.fraction<1:
            progress=command.advance(2) # called by the host's approved-thread tick
            print(f'Colliders: {progress.prepared}/{progress.total}')
        patch=command.commit() # push this patch onto the host command stack
        assert all(stage.GetPrimAtPath(p).HasAPI(UsdPhysics.CollisionAPI) for p in selected)
        patch.undo()
        assert not any(stage.GetPrimAtPath(p).HasAPI(UsdPhysics.CollisionAPI) for p in selected)
        patch.redo()
    return stage


if __name__=='__main__':main()
