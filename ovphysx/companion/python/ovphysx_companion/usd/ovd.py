# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-OVD-001
# @covers AC-2 AC-3 AC-4
"""Customer-USD OVD data conversion with explicit lifetime and timeline policy."""
from dataclasses import dataclass
import json
import math
from .session import _usd
from ..ovd import Document, Record, attribute_values


@dataclass(frozen=True)
class ObjectLifetime:
    context: int
    handle: int
    generation: int
    class_handle: int
    path: str
    born: float
    died: object


@dataclass(frozen=True)
class FrameMarker:
    command: int
    context: int
    timestamp: int
    time_code: float


@dataclass(frozen=True)
class Conversion:
    objects: tuple
    frames: tuple
    patch: object
    timeline: str
    diagnostics: tuple=()

    def object_at(self,context,handle,time_code):
        """Resolve one recorded handle at a chosen playback time, without USD access."""
        if type(time_code) not in (int,float) or not math.isfinite(time_code):
            raise ValueError('Playback time must be finite')
        for obj in self.objects:
            if (obj.context==context and obj.handle==handle and obj.born<=time_code and
                    (obj.died is None or time_code<obj.died)):
                return obj
        return None


def convert(session,document,*,root,layer,meters_per_unit,source=None,time_for_record=None,up_axis=None):
    """Author a generic OVD data stage and recognized rigid global poses atomically.

    Each (context, handle, creation generation) gets a distinct Xform and visibility
    lifetime. Primitive attributes/list updates retain their declared typed values;
    class-valued opaque payloads are hex strings. Raw object references remain
    uint64 arrays, never ambiguous recycled-handle relationships. Full schema and
    frame markers remain in root metadata. This converter does not manufacture
    collider meshes or provide viewer controls.

    Default time codes are command indices, preserving total stream ordering.
    Native scene timestamps are *not* seconds and scene-local frame clocks can
    differ. A host may supply time_for_record(index, record), evaluated before any
    USD edit; values must be finite/nonnegative/nondecreasing. FrameMarker maps raw
    scene ticks to the chosen time codes for timeline controls.

    meters_per_unit declares recording length units. A reset root transform scales
    to the customer's existing stage units. Coordinates retain their recorded axes;
    optional up_axis must match the target stage, otherwise convert explicitly first.
    No stage metadata, selection or output destination is chosen implicitly.
    """
    Sdf,Tf,Usd,UsdGeom,Vt=_usd()
    from pxr import Gf
    session._validate_layer(layer)
    expected_source=session.source if source is None else source
    session.validate(expected_source)
    if not isinstance(document,Document) or not all(isinstance(r,Record) for r in document.records):
        raise TypeError('Pass a decoded immutable OVD Document')
    if type(meters_per_unit) not in (int,float) or not math.isfinite(meters_per_unit) or meters_per_unit<=0:
        raise ValueError('Recording meters_per_unit must be positive and finite')
    stage_meters=UsdGeom.GetStageMetersPerUnit(session.stage)
    if not math.isfinite(stage_meters) or stage_meters<=0:raise ValueError('Target stage has invalid length units')
    if not math.isfinite(meters_per_unit/stage_meters) or meters_per_unit/stage_meters>3.402823466e38:
        raise ValueError('Recording-to-stage length scale exceeds supported float32 range')
    if up_axis is not None and (up_axis not in ('Y','Z') or str(UsdGeom.GetStageUpAxis(session.stage))!=up_axis):
        raise ValueError('Recording and target up axes must match; convert axes explicitly first')
    path=Sdf.Path(root)
    if not path.IsAbsolutePath() or not path.IsPrimPath() or path==Sdf.Path.absoluteRootPath or session.stage.GetPrimAtPath(path):
        raise ValueError('Choose a new absolute capture root prim')
    parent=path.GetParentPath()
    if parent!=Sdf.Path.absoluteRootPath:
        prim=session.stage.GetPrimAtPath(parent)
        if not prim or not prim.IsActive() or prim.IsInstanceProxy() or prim.IsInPrototype():
            raise ValueError('Capture parent must be editable and exist')
    if time_for_record is not None and not callable(time_for_record):raise TypeError('time_for_record must be callable')
    classes={};attributes={};active={};generations={};lifetimes=[];markers=[];messages=[];operations=[];lists={};diagnostics=[]
    previous=-1.
    for index,record in enumerate(document.records):
        time=float(index if time_for_record is None else time_for_record(index,record))
        if not math.isfinite(time) or time<0 or time<previous:raise ValueError('OVD time codes must be finite nonnegative and ordered')
        previous=time
        if record.command==1:
            if record.class_handle in classes:raise ValueError('Duplicate OVD class handle')
            classes[record.class_handle]=record
        elif record.command in (3,4,5):
            if record.attribute in attributes:raise ValueError('Duplicate OVD attribute handle')
            attributes[record.attribute]=record
        elif record.command==9:
            key=(record.context,record.object)
            if key in active or record.class_handle not in classes:raise ValueError('Invalid OVD object creation')
            generation=generations.get(key,0)+1;generations[key]=generation
            destination=f'{root}/c{key[0]}_o{key[1]}_g{generation}'
            identity=(key[0],key[1],generation)
            active[key]=len(lifetimes)
            lifetimes.append(ObjectLifetime(*identity,record.class_handle,destination,time,None))
            operations.append(('create',destination,time,record))
        elif record.command==10:
            key=(record.context,record.object)
            if key not in active:
                # Late SDK captures can contain destruction for an already retired
                # shared object. Preserve the diagnostic rather than invent a prim.
                diagnostics.append(('OVD_DESTROY_WITHOUT_LIVE_OBJECT',index,record.context,record.object))
                continue
            i=active.pop(key);old=lifetimes[i]
            lifetimes[i]=ObjectLifetime(old.context,old.handle,old.generation,old.class_handle,old.path,old.born,time)
            operations.append(('destroy',old.path,time,None))
        elif record.command in (6,7,8):
            key=(record.context,record.object)
            if key not in active or not record.attribute_path:raise ValueError('OVD attribute has no live object or path')
            if any(h not in attributes for h in record.attribute_path):raise ValueError('OVD attribute schema is missing')
            schema=attributes[record.attribute_path[-1]]
            obj=lifetimes[active[key]]
            prop='ovd:'+':'.join(f'a{h}_{Tf.MakeValidIdentifier(attributes[h].name)}' for h in record.attribute_path)
            if schema.command==4:
                value=record.payload.hex();value_type=Sdf.ValueTypeNames.String
            else:
                value=attribute_values(schema.data_type,record.payload)
                value_types={0:'IntArray',1:'IntArray',2:'IntArray',3:'Int64Array',4:'UIntArray',5:'UIntArray',
                             6:'UIntArray',7:'UInt64Array',8:'FloatArray',9:'DoubleArray',10:'String',
                             11:'UInt64Array',12:'UIntArray',13:'UInt64Array'}
                value_type=getattr(Sdf.ValueTypeNames,value_types[schema.data_type])
                if schema.command==3 and schema.elements and schema.data_type!=10 and len(value)!=schema.elements:
                    raise ValueError('OVD fixed-size attribute payload does not match schema')
            if record.command in (7,8):
                if schema.command!=5 or not isinstance(value,tuple):raise ValueError('Unsupported OVD list operation')
                identity=(obj.path,prop)
                current=lists.setdefault(identity,[])
                for item in value:
                    if record.command==7 and item not in current:current.append(item)
                    elif record.command==8 and item in current:current.remove(item)
                value=tuple(current)
            elif schema.command==5:
                lists[(obj.path,prop)]=list(value)
            contents=getattr(Vt,value_types[schema.data_type])(value) if isinstance(value,tuple) else value
            operations.append(('attribute',obj.path,time,(prop,value_type,contents)))
            defining_class=classes.get(schema.class_handle)
            if (record.command==6 and schema.name=='globalPose' and defining_class is not None and
                    defining_class.name=='PxRigidActor' and schema.data_type==8):
                if len(value)!=7 or not all(math.isfinite(v) for v in value):raise ValueError('OVD global pose is invalid')
                length=sum(v*v for v in value[:4])
                if not math.isclose(length,1,rel_tol=1e-4,abs_tol=1e-4):raise ValueError('OVD global pose quaternion is not normalized')
                operations.append(('pose',obj.path,time,value))
        elif record.command in (11,12):
            markers.append(FrameMarker(record.command,record.context,record.timestamp,time))
        elif record.command==13:
            messages.append((index,record.context,record.message_type,record.class_handle,record.file,record.line,record.name))
        else:
            raise ValueError('Unsupported OVD command')
    # Check type/class references before entering the atomic layer batch.
    for record in attributes.values():
        if record.class_handle not in classes:raise ValueError('OVD attribute owner class is missing')
    for record in classes.values():
        if record.base_class and record.base_class not in classes:raise ValueError('OVD base class is missing')
    schema_json=json.dumps({'classes':[(r.class_handle,r.base_class,r.name) for r in classes.values()],
                            'attributes':[(r.attribute,r.class_handle,r.command,r.data_type,r.elements,
                                           r.attribute_class,r.enum_value,r.name) for r in attributes.values()]})
    frame_json=json.dumps([(r.command,r.context,r.timestamp,r.time_code) for r in markers])
    def apply():
        capture=UsdGeom.Xform.Define(session.stage,path)
        capture.SetResetXformStack(True)
        capture.AddScaleOp().Set(Gf.Vec3f(meters_per_unit/stage_meters))
        capture.GetPrim().SetCustomDataByKey('ovd:schema',schema_json)
        capture.GetPrim().SetCustomDataByKey('ovd:frames',frame_json)
        capture.GetPrim().SetCustomDataByKey('ovd:messages',json.dumps(messages))
        capture.GetPrim().SetCustomDataByKey('ovd:diagnostics',json.dumps(diagnostics))
        capture.GetPrim().SetCustomDataByKey('ovd:timeline','record-index' if time_for_record is None else 'host')
        capture.GetPrim().SetCustomDataByKey('ovd:metersPerUnit',float(meters_per_unit))
        poses={}
        for kind,destination,time,value in operations:
            if kind=='create':
                prim=UsdGeom.Xform.Define(session.stage,destination).GetPrim()
                image=UsdGeom.Imageable(prim)
                image.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
                if time>0:image.GetVisibilityAttr().Set(UsdGeom.Tokens.invisible,Usd.TimeCode(0))
                image.GetVisibilityAttr().Set(UsdGeom.Tokens.inherited,Usd.TimeCode(time))
                for name,number in (('context',value.context),('handle',value.object),('class',value.class_handle)):
                    prim.CreateAttribute('ovd:'+name,Sdf.ValueTypeNames.UInt64,custom=True).Set(number)
                prim.SetCustomDataByKey('ovd:className',classes[value.class_handle].name)
                prim.SetCustomDataByKey('ovd:name',value.name)
            else:
                prim=session.stage.GetPrimAtPath(destination)
                if kind=='destroy':UsdGeom.Imageable(prim).GetVisibilityAttr().Set(UsdGeom.Tokens.invisible,Usd.TimeCode(time))
                elif kind=='attribute':
                    name,type_name,contents=value
                    if not prim.CreateAttribute(name,type_name,custom=True).Set(contents,Usd.TimeCode(time)):
                        raise RuntimeError('USD rejected OVD attribute data')
                elif kind=='pose':
                    if destination not in poses:
                        xform=UsdGeom.Xformable(prim)
                        poses[destination]=(xform.AddTranslateOp(),xform.AddOrientOp())
                    translation,rotation=poses[destination]
                    translation.Set(Gf.Vec3d(*value[4:]),Usd.TimeCode(time))
                    rotation.Set(Gf.Quatf(value[3],Gf.Vec3f(*value[:3])),Usd.TimeCode(time))
    session.validate(expected_source)
    patch=session._apply(layer,apply)
    return Conversion(tuple(lifetimes),tuple(markers),patch,'record-index' if time_for_record is None else 'host',tuple(diagnostics))
