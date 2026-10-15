# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-VALIDATE-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Classified validation over copied data; repair application belongs to Python USD."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Check:
    name: str
    requires: str  # usd-data, cooking, simulation


CHECKS = (Check('joint-frames','usd-data'), Check('mass','usd-data'),
          Check('collider-transform','usd-data'), Check('collision-cook','cooking'),
          Check('preview-motion','simulation'))


@dataclass(frozen=True)
class Repair:
    path: str
    attribute: str
    type_name: str
    value: object
    time_code: object = None


@dataclass(frozen=True)
class Issue:
    source: object
    path: str
    severity: str
    code: str
    check: Check
    message: str
    repair: object = None


@dataclass(frozen=True)
class Report:
    source: object
    checks: tuple
    issues: tuple

    @property
    def complete(self):
        return not any(issue.severity == 'unavailable' for issue in self.issues)

    @property
    def valid(self):
        return self.complete and not any(issue.severity == 'error' for issue in self.issues)


def validate_descriptions(snapshot, *, checks=('joint-frames','mass','collider-transform'), cooking=None):
    """Pure checks never initialize/attach. Cooking checks require an explicit service.

    Results are proposals, with stable codes and captured source revision. A Busy
    or missing backend is an unavailable check, not an assertion that the asset is
    invalid. Simulation checks use ValidationPreview separately. No checks edit input.
    """
    catalog = {check.name: check for check in CHECKS}
    selected = tuple(catalog[name] for name in dict.fromkeys(checks))
    if any(check.requires == 'simulation' for check in selected):
        raise ValueError('Simulation checks require an explicit ValidationPreview')
    issues = []
    seen = set()
    for check in selected:
        for item in snapshot.descriptions:
            key = (check.name, item.path)
            if key in seen:
                continue
            if check.name == 'joint-frames' and item.family != 'joint':continue
            if check.name == 'collider-transform' and item.family != 'shape':continue
            if check.name == 'collision-cook' and (item.family != 'shape' or item.type_name != 'Mesh'):continue
            seen.add(key)
            def issue(code, message, repair=None, severity='error'):
                issues.append(Issue(snapshot.source,item.path,severity,code,check,message,repair))
            if check.name == 'joint-frames':
                for name in ('physics:localRot0','physics:localRot1'):
                    attribute = next((a for a in item.attributes if a.name == name), None)
                    if attribute is None or attribute.value is None:continue
                    value = attribute.value
                    valid = isinstance(value,tuple) and len(value)==4 and all(math.isfinite(v) for v in value)
                    norm = math.hypot(*value) if valid else 0
                    if not valid or abs(norm-1) > 1e-5:
                        repair = Repair(item.path,name,attribute.type_name,tuple(v/norm for v in value),snapshot.time_code) if valid and math.isfinite(norm) and norm>0 else None
                        issue('JOINT_FRAME_NON_UNIT',f'{name} must be a finite unit quaternion',repair)
                for name in ('physics:body0','physics:body1'):
                    if len(item.targets(name)) > 1:
                        issue('JOINT_MULTIPLE_BODIES',f'{name} has more than one target')
            elif check.name == 'mass':
                for name in ('physics:mass','physics:density'):
                    attribute = next((a for a in item.attributes if a.name == name), None)
                    if attribute and attribute.value is not None and (not math.isfinite(attribute.value) or attribute.value<0):
                        issue('MASS_INVALID',f'{name} must be finite and nonnegative',
                              Repair(item.path,name,attribute.type_name,0.0,snapshot.time_code))
            elif check.name == 'collider-transform':
                import numpy as np
                matrix = np.asarray(item.local_to_world)
                if not np.isfinite(matrix).all() or abs(np.linalg.det(matrix[:3,:3])) < 1e-12:
                    issue('COLLIDER_SINGULAR_TRANSFORM','Collider transform is singular or nonfinite')
            elif check.name == 'collision-cook':
                if cooking is None:
                    issue('VALIDATION_UNAVAILABLE','A stopped cooking service is required',severity='unavailable')
                    continue
                from .cooking import Kind
                from .native import CompanionError, Status
                approximation = item.attribute('physics:approximation','none')
                modes = {'none':Kind.TRIANGLE,'meshSimplification':Kind.TRIANGLE,
                         'convexHull':Kind.CONVEX,'convexDecomposition':Kind.DECOMPOSITION,'sdf':Kind.SDF}
                if approximation not in modes or item.attribute('holeIndices',()):
                    issue('VALIDATION_UNAVAILABLE','This approximation or polygon-hole recipe is unsupported',severity='unavailable')
                    continue
                try:
                    parameters = {'triangle_mode':1} if approximation=='meshSimplification' else {}
                    cooking.cook(modes[approximation],item.attribute('points',()),item.attribute('faceVertexIndices',()),
                                 faces=item.attribute('faceVertexCounts',()),meters_per_unit=item.meters_per_unit,
                                 frame='prim-local',units='stage',right_handed=item.attribute('orientation','rightHanded')=='rightHanded',
                                 **parameters).result()
                except CompanionError as error:
                    if error.status in (Status.BUSY,Status.UNAVAILABLE):
                        issue('VALIDATION_UNAVAILABLE',str(error),severity='unavailable')
                    else:
                        issue('COLLIDER_COOK_FAILED',str(error))
                except (TypeError,ValueError) as error:
                    issue('COLLIDER_INPUT_INVALID',str(error))
    return Report(snapshot.source,selected,tuple(issues))


class ValidationPreview:
    """Dedicated native attachment for simulation checks beside live playback.

    The host supplies a distinct sealed ovstage and its source revision. This
    service owns only that preview, never steps or collects the bootstrap core's
    work, and closes through PreviewSession's native drain/retirement contract.
    """
    def __init__(self, physics, stage, *, source_revision, read_ordinal, **options):
        from .preview import PreviewSession
        if source_revision is None:
            raise ValueError('Declare the source revision associated with the sealed input')
        self.source = source_revision
        self.preview = PreviewSession(physics,stage,read_ordinal=read_ordinal,**options)

    def check_motion(self, paths, *, steps=1, dt=1/60, max_displacement=1):
        """Flag excessive world displacement, in the supplied stage's length units."""
        if (isinstance(steps,bool) or not isinstance(steps,int) or not 1<=steps<=100000 or
                not math.isfinite(dt) or not 0<dt<=1 or not math.isfinite(max_displacement) or max_displacement<0):
            raise ValueError('Declare a bounded step count, positive dt and nonnegative motion tolerance')
        paths = tuple(dict.fromkeys(paths))
        if not paths or any(not isinstance(path,str) or not path.startswith('/') for path in paths):
            raise ValueError('Select absolute body paths')
        start = {path:self.preview.rigid_pose(path).position for path in paths}
        for _ in range(steps):self.preview.step(dt)
        check = next(check for check in CHECKS if check.name=='preview-motion')
        issues = []
        for path in paths:
            end = self.preview.rigid_pose(path).position
            distance = math.dist(start[path],end)
            if not math.isfinite(distance) or distance>max_displacement:
                issues.append(Issue(self.source,path,'error','PREVIEW_EXCESSIVE_MOTION',check,
                                    f'World displacement {distance} exceeds stage-unit tolerance {max_displacement}'))
        return Report(self.source,(check,),tuple(issues))

    def close(self):self.preview.close()
    def __enter__(self):return self
    def __exit__(self,*args):self.close()
