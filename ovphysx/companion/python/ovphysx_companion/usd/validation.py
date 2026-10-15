# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-VALIDATE-001
# @covers AC-2 AC-4
"""Apply only selected, fresh validation repairs through customer USD."""
import math
from .session import _usd


def apply_repairs(session, report, selected, *, layer):
    """Atomically apply selected issue indices; the returned patch supplies undo/redo."""
    Sdf, _, Usd, _, _ = _usd()
    from pxr import Gf
    session.validate(report.source)
    session._validate_layer(layer)
    selected = tuple(selected)
    if (not selected or len(set(selected))!=len(selected) or
            any(isinstance(i,bool) or not isinstance(i,int) or i<0 or i>=len(report.issues) for i in selected)):
        raise ValueError('Select unique valid diagnostic indices')
    prepared, seen = [], set()
    for index in selected:
        issue = report.issues[index]
        repair = issue.repair
        if issue.source != report.source or repair is None:
            raise ValueError('Selected diagnostic has no applicable repair')
        path = Sdf.Path(repair.path)
        if not path.IsAbsolutePath() or not path.IsPrimPath():
            raise ValueError('Repair requires an absolute prim path')
        prim = session.stage.GetPrimAtPath(path)
        if not prim or not prim.IsActive() or prim.IsInstanceProxy() or prim.IsInPrototype():
            raise ValueError('Repair destination is not editable')
        attribute = prim.GetAttribute(repair.attribute)
        key = (repair.path,repair.attribute)
        if key in seen or not attribute or str(attribute.GetTypeName())!=repair.type_name:
            raise ValueError('Repair has a conflicting or incompatible attribute')
        seen.add(key)
        value = repair.value
        if repair.type_name in ('float','double'):
            if not isinstance(value,(int,float)) or not math.isfinite(value):
                raise ValueError('Repair requires a finite scalar')
            value = float(value)
        elif repair.type_name in ('quatf','quatd'):
            if len(value)!=4 or not all(math.isfinite(v) for v in value) or abs(math.hypot(*value)-1)>1e-5:
                raise ValueError('Repair requires a unit quaternion')
            value = Gf.Quatf(value[3],Gf.Vec3f(*value[:3])) if repair.type_name=='quatf' else Gf.Quatd(value[3],Gf.Vec3d(*value[:3]))
        else:
            raise ValueError('This repair value type is unsupported')
        if repair.time_code is not None and (isinstance(repair.time_code,bool) or not math.isfinite(repair.time_code)):
            raise ValueError('Repair requires a finite USD time')
        time = Usd.TimeCode.Default() if repair.time_code is None else Usd.TimeCode(repair.time_code)
        prepared.append((attribute,value,time))
    def edit():
        for attribute,value,time in prepared:
            if not attribute.Set(value,time):
                raise RuntimeError(f'USD rejected repair of {attribute.GetPath()}')
    return session._apply(layer,edit)
