# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-CONFIG-001
# @covers AC-1 AC-2 AC-3
"""Explicit scene-setting edits in customer USD; no native or process mutation."""
from dataclasses import dataclass
import math
from .session import _usd,DependencyError
from ..configuration import Solver
from ..native import _host_scalar,_host_sequence


@dataclass(frozen=True)
class SceneOptions:
    """None preserves the existing opinion. Gravity uses world meters/sec squared.

    Edits author default-time inputs only. The host explicitly republishes inputs
    and recreates the affected attachment for construction-time solver/GPU changes.
    Native attachment overrides and process hard-CPU policy still take precedence.
    """
    gravity_world_mps2:tuple|None=None
    solver:Solver|None=None
    gpu_enabled:bool|None=None

    def __post_init__(self):
        if self.gravity_world_mps2 is not None:
            gravity=tuple(_host_scalar(v) for v in _host_sequence(self.gravity_world_mps2))
            if len(gravity)!=3 or not all(math.isfinite(v) for v in gravity):raise ValueError('Gravity requires finite world XYZ acceleration')
            object.__setattr__(self,'gravity_world_mps2',gravity)
        if self.solver is not None and (isinstance(self.solver,bool) or not isinstance(self.solver,int) or self.solver not in (Solver.PGS,Solver.TGS)):
            raise ValueError('Select PGS, TGS or None to preserve the opinion')
        if self.gpu_enabled is not None and not isinstance(self.gpu_enabled,bool):raise TypeError('gpu_enabled must be bool or None')


def apply_scene_options(session,path,options,*,source,layer):
    Sdf,_,Usd,UsdGeom,_=_usd()
    from pxr import Gf,UsdPhysics
    session.validate(source);session._validate_layer(layer)
    if not isinstance(options,SceneOptions):raise TypeError('Pass SceneOptions')
    path=Sdf.Path(path);prim=session.stage.GetPrimAtPath(path)
    if (not path.IsAbsolutePath() or not path.IsPrimPath() or not prim or not prim.IsActive() or
        prim.IsInstanceProxy() or prim.IsInPrototype() or not UsdPhysics.Scene(prim)):
        raise ValueError('Select an existing editable PhysicsScene')
    if options.solver is not None or options.gpu_enabled is not None:
        if not Usd.SchemaRegistry().FindAppliedAPIPrimDefinition('PhysxSceneAPI'):raise DependencyError('Register compatible PhysxSceneAPI codeless schema')
    edits=[]
    if options.gravity_world_mps2 is not None:
        meters=UsdGeom.GetStageMetersPerUnit(session.stage)
        if not math.isfinite(meters) or meters<=0:raise ValueError('Stage requires positive meters-per-unit')
        magnitude=math.hypot(*options.gravity_world_mps2);native_magnitude=magnitude/meters
        if not math.isfinite(native_magnitude) or native_magnitude>3.402823466e38:raise ValueError('Gravity exceeds stage float range')
        direction=tuple(v/magnitude for v in options.gravity_world_mps2) if magnitude else (0,-1,0)
        edits += [('physics:gravityDirection',Sdf.ValueTypeNames.Vector3f,Gf.Vec3f(*direction)),
                  ('physics:gravityMagnitude',Sdf.ValueTypeNames.Float,native_magnitude)]
    if options.solver is not None:edits.append(('physxScene:solverType',Sdf.ValueTypeNames.Token,'PGS' if options.solver==Solver.PGS else 'TGS'))
    if options.gpu_enabled is not None:edits.append(('physxScene:enableGPUDynamics',Sdf.ValueTypeNames.Bool,options.gpu_enabled))
    if not edits:raise ValueError('Provide at least one explicit authored setting')
    session.validate(source)
    def edit():
        if options.solver is not None or options.gpu_enabled is not None:
            if not prim.HasAPI('PhysxSceneAPI') and not prim.ApplyAPI('PhysxSceneAPI'):raise RuntimeError('USD rejected scene API')
        for name,kind,value in edits:
            if not prim.CreateAttribute(name,kind,custom=False).Set(value):raise RuntimeError('USD rejected scene setting')
    return session._apply(layer,edit)
