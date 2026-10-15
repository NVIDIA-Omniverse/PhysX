# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-DESCRIPTION-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Customer-USD extraction into copied stopped descriptions, without attachment."""
import math
from .session import _usd
from ..descriptions import Attribute, Description, DescriptionResult, Diagnostic


def _copied(value):
    from pxr import Sdf
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    if isinstance(value, Sdf.Path):
        return str(value)
    if isinstance(value, Sdf.AssetPath):
        return value.path
    if hasattr(value, 'GetImaginary') and hasattr(value, 'GetReal'):
        return tuple(float(v) for v in value.GetImaginary()) + (float(value.GetReal()),)
    if isinstance(value, dict):
        return tuple((str(k), _copied(v)) for k,v in sorted(value.items()))
    try:
        return tuple(_copied(v) for v in value)
    except TypeError as error:
        raise TypeError(f'Unsupported description value type {type(value).__name__}') from error


def capture_descriptions(session, paths=None, *, time_code=None):
    """Capture composed schema data and transforms on the approved USD thread.

    Families cover collision shapes, joints, articulation roots, applied tendon
    APIs, vehicle/wheel APIs and particle systems/samplers. Public schema names,
    typed attributes (including defaults) and relationships form the description;
    this is input inspection, not a claim about realized solver topology. Inherited
    transforms and instance-proxy paths are copied. Missing custom value converters
    produce per-property diagnostics rather than private USD objects in results.
    """
    Sdf, _, Usd, UsdGeom, _ = _usd()
    from pxr import UsdPhysics
    session._check()
    if time_code is not None and (isinstance(time_code, bool) or not math.isfinite(time_code)):
        raise ValueError('time_code must be None or a finite USD time')
    time = Usd.TimeCode.Default() if time_code is None else Usd.TimeCode(time_code)
    source = session.source
    meters, kilograms = UsdGeom.GetStageMetersPerUnit(session.stage), UsdPhysics.GetStageKilogramsPerUnit(session.stage)
    if not all(math.isfinite(v) and v > 0 for v in (meters, kilograms)):
        raise ValueError('Stage units must be finite and positive')
    diagnostics, descriptions = [], []
    if paths is None:
        prims = list(Usd.PrimRange(session.stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()))
    else:
        paths = tuple(dict.fromkeys(str(p) for p in paths))
        if any(not Sdf.Path(p).IsAbsolutePath() or not Sdf.Path(p).IsPrimPath() for p in paths):
            raise ValueError('Select absolute prim paths')
        prims = []
        for path in paths:
            prim = session.stage.GetPrimAtPath(path)
            if prim:
                prims.append(prim)
            else:
                diagnostics.append(Diagnostic(source,path,'DESCRIPTION_MISSING_PRIM','Source prim is absent'))
    cache = UsdGeom.XformCache(time)
    for prim in prims:
        schemas = tuple(str(s) for s in prim.GetAppliedSchemas())
        names = tuple(name.split(':',1)[0] for name in schemas)
        families = []
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):families.append('body')
        if prim.HasAPI(UsdPhysics.CollisionAPI):families.append('shape')
        if prim.IsA(UsdPhysics.Joint):families.append('joint')
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):families.append('articulation')
        if any(name.startswith('PhysxTendon') for name in names):families.append('tendon')
        if any(name.startswith('PhysxVehicle') for name in names):families.append('vehicle')
        if str(prim.GetTypeName()) == 'PhysxParticleSystem' or any(name.startswith('PhysxParticle') for name in names):
            families.append('particle')
        if not families:
            continue
        attributes = []
        for attribute in sorted(prim.GetAttributes(),key=lambda a:a.GetName()):
            try:
                attributes.append(Attribute(attribute.GetName(),str(attribute.GetTypeName()),
                                             _copied(attribute.Get(time)),attribute.HasAuthoredValueOpinion()))
            except TypeError as error:
                diagnostics.append(Diagnostic(source,str(attribute.GetPath()),'DESCRIPTION_UNSUPPORTED_VALUE',str(error)))
        relationships = tuple((r.GetName(),tuple(str(p) for p in r.GetForwardedTargets()))
                              for r in sorted(prim.GetRelationships(),key=lambda r:r.GetName()))
        matrix = tuple(tuple(float(v) for v in row) for row in cache.GetLocalToWorldTransform(prim))
        if not all(math.isfinite(v) for row in matrix for v in row):
            diagnostics.append(Diagnostic(source,str(prim.GetPath()),'DESCRIPTION_INVALID_TRANSFORM',
                                          'Transform contains nonfinite values','error'))
        instance = prim
        while instance and not instance.IsInstance():
            instance = instance.GetParent()
        instance_path = str(instance.GetPath()) if instance and prim.IsInstanceProxy() else ''
        for family in families:
            descriptions.append(Description(source,str(prim.GetPath()),family,str(prim.GetTypeName()),schemas,
                tuple(attributes),relationships,matrix,meters,kilograms,instance_path))
    session.validate(source)
    return DescriptionResult(source,tuple(descriptions),tuple(diagnostics),time_code=time_code)
