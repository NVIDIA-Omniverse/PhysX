# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-VEHICLE-001
# @covers AC-1 AC-4
"""Apply copied stopped vehicle calculations through customer-installed USD."""
import math
from .session import _usd,DependencyError


def apply_authoring(session,vehicle_path,*,suspension_frames,sprung_masses,reference_frame,source,layer):
    """One reversible default-time batch; no native runtime or schema binding.

    suspension_frames maps wheel-attachment paths to XYZ meters + quaternion XYZW
    at maximum compression, relative to the declared actor-local or com-local
    reference. sprung_masses maps suspension-component paths to kilograms. Shared
    suspension components require equal per-wheel results; the caller must split
    components before assigning different masses. No implicit component creation.
    Vehicle transforms must be rigid (unit scale). Stage length/mass units are
    converted explicitly; existing reference-frame metadata must match the data.
    """
    Sdf,_,Usd,UsdGeom,_=_usd()
    from pxr import Gf,UsdPhysics
    session.validate(source);session._validate_layer(layer)
    if reference_frame not in ('actor-local','com-local'):raise ValueError('Declare actor-local or com-local reference frame')
    def prim(path,schema):
        path=Sdf.Path(path)
        value=session.stage.GetPrimAtPath(path)
        if not value or not path.IsAbsolutePath() or not path.IsPrimPath() or not value.IsActive() or value.IsInstanceProxy() or value.IsInPrototype():
            raise ValueError('Select existing editable vehicle components')
        if not Usd.SchemaRegistry().FindAppliedAPIPrimDefinition(schema):raise DependencyError(f'Register compatible {schema} codeless schema')
        if not value.HasAPI(schema):raise ValueError(f'{path} lacks {schema}')
        return value
    vehicle=prim(vehicle_path,'PhysxVehicleAPI')
    com=vehicle.GetCustomDataByKey('physxVehicle:referenceFrameIsCenterOfMass')
    if com is None:com=True
    if not isinstance(com,bool) or com!=(reference_frame=='com-local'):raise ValueError('Computed suspension reference frame differs from vehicle metadata')
    transform=UsdGeom.XformCache().GetLocalToWorldTransform(vehicle)
    axes=[transform.TransformDir(Gf.Vec3d(*(1 if i==j else 0 for j in range(3)))) for i in range(3)]
    if (any(abs(a.GetLength()-1)>1e-5 for a in axes) or any(abs(Gf.Dot(axes[i],axes[j]))>1e-5 for i in range(3) for j in range(i))
        or transform.GetDeterminant()<=0):raise ValueError('Vehicle authoring requires a rigid transform without scale or shear')
    meters=UsdGeom.GetStageMetersPerUnit(session.stage);kg=UsdPhysics.GetStageKilogramsPerUnit(session.stage)
    if not all(math.isfinite(x) and x>0 for x in (meters,kg)):raise ValueError('Stage requires positive length and mass units')
    frames=[];masses=[]
    for path,pose in suspension_frames.items():
        wheel=prim(path,'PhysxVehicleWheelAttachmentAPI')
        if not wheel.GetPath().HasPrefix(vehicle.GetPath()) or wheel==vehicle:raise ValueError('Wheel must belong to selected vehicle subtree')
        pose=tuple(float(v) for v in pose)
        if len(pose)!=7 or not all(math.isfinite(v) for v in pose) or abs(sum(v*v for v in pose[3:])-1)>1e-5:
            raise ValueError('Expected finite local-meter pose with normalized XYZW quaternion')
        position=tuple(v/meters for v in pose[:3])
        if not all(math.isfinite(v) and abs(v)<=3.402823466e38 for v in position):raise ValueError('Suspension position exceeds stage float range')
        frames.append((wheel,Gf.Vec3f(*position),Gf.Quatf(pose[6],Gf.Vec3f(*pose[3:6]))))
    for path,mass in sprung_masses.items():
        suspension=prim(path,'PhysxVehicleSuspensionAPI');mass=float(mass)
        if not math.isfinite(mass) or mass<0:raise ValueError('Sprung masses must be finite nonnegative kilograms')
        value=mass/kg
        if not math.isfinite(value) or value>3.402823466e38:raise ValueError('Sprung mass exceeds stage float range')
        masses.append((suspension,value))
    if not frames and not masses:raise ValueError('Provide computed suspension frames or sprung masses')
    session.validate(source)
    def edit():
        for wheel,p,q in frames:
            for name,kind,value in (('suspensionFramePosition',Sdf.ValueTypeNames.Point3f,p),('suspensionFrameOrientation',Sdf.ValueTypeNames.Quatf,q)):
                if not wheel.CreateAttribute('physxVehicleWheelAttachment:'+name,kind,custom=False).Set(value):raise RuntimeError('USD rejected suspension frame')
        for suspension,mass in masses:
            if not suspension.CreateAttribute('physxVehicleSuspension:sprungMass',Sdf.ValueTypeNames.Float,custom=False).Set(mass):raise RuntimeError('USD rejected sprung mass')
    return session._apply(layer,edit)
