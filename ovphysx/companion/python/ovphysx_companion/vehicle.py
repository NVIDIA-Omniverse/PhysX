# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-VEHICLE-001
# @covers AC-1 AC-2 AC-3 AC-4
"""CPU vehicle authoring math, explicit host inputs and copied native telemetry."""
import atexit
import ctypes as C
from dataclasses import dataclass
import math
import threading
import warnings
import weakref
from .native import CompanionError, Status, _check, _host_sequence, _host_scalar


class _Identity(C.Structure):
    _fields_=[('key',C.c_uint64),('generation',C.c_uint64)]


class _Wheel(C.Structure):
    _fields_=[(name,_Identity) for name in ('identity','groundActor','groundShape','groundMaterial')]+[
        (name,C.c_uint32) for name in ('index','onGround','comFrame','reserved')]+[
        ('position',C.c_float*3),('orientation',C.c_float*4),('rotationSpeed',C.c_float),('rotationAngle',C.c_float),
        ('steerAngle',C.c_float),('groundPlane',C.c_float*4),('groundPosition',C.c_float*3),('suspensionJounce',C.c_float),
        ('suspensionForce',C.c_float*3),('tireFriction',C.c_float),('longitudinalDirection',C.c_float*3),
        ('lateralDirection',C.c_float*3),('longitudinalSlip',C.c_float),('lateralSlip',C.c_float),('tireForce',C.c_float*3)]


class _Command(C.Structure):
    _fields_=[('generation',C.c_uint64),('action',C.c_uint32),('direction',C.c_int32)]+[
        (name,C.c_float) for name in ('throttle','brake0','brake1','steer')]


class _State(C.Structure):
    _fields_=[('identity',_Identity),('step',C.c_uint64),('engineFields',C.c_uint32),('wheelCount',C.c_uint32)]+[
        (name,C.c_float) for name in ('throttle','brake0','brake1','steer','forwardVelocity','clutch','gearSwitchTime','autoboxTime','engineSpeed')]+[
        ('currentGear',C.c_int32),('targetGear',C.c_int32),('automaticTransmission',C.c_uint32),('reserved',C.c_uint32),('wheels',_Wheel*20)]


_FloatPtr=C.POINTER(C.c_float)
class _Api(C.Structure):
    _fields_=[('size',C.c_uint32),('version',C.c_uint32),('capabilities',C.c_uint64),
        ('execute',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.c_char_p,C.POINTER(_Command),C.c_uint32,C.POINTER(_State),C.c_uint32)),
        ('sprung_masses',C.CFUNCTYPE(C.c_uint32,C.c_uint32,_FloatPtr,C.c_float,C.c_uint32,_FloatPtr,C.c_uint32)),
        ('suspension_frame',C.CFUNCTYPE(C.c_uint32,_FloatPtr,_FloatPtr,_FloatPtr)),
        ('ackermann',C.CFUNCTYPE(C.c_uint32,C.c_float,C.c_float,C.c_float,_FloatPtr))]


def _api(library):
    try:acquire=library.ovphysx_companion_vehicle_acquire
    except AttributeError as error:raise CompanionError(Status.UNAVAILABLE,'vehicle table') from error
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
    api=_Api();_check(acquire(1,C.sizeof(api),C.byref(api)),'acquire vehicle')
    if api.size!=C.sizeof(api) or api.version!=1 or api.capabilities&3!=3 or not all((api.execute,api.sprung_masses,api.suspension_frame,api.ackermann)):
        raise CompanionError(Status.ABI_MISMATCH,'vehicle table')
    return api


def _floats(value,count):
    values=tuple(_host_scalar(v) for v in _host_sequence(value))
    if len(values)!=count or not all(math.isfinite(v) and math.isfinite(C.c_float(v).value) for v in values):
        raise ValueError(f'Expected {count} finite ready CPU float values')
    return (C.c_float*count)(*values)


class VehicleMath:
    """Stopped native calculations with a retained CDLL, without creating PhysX.

    Positions are meters, masses kilograms, quaternions normalized XYZW. Body
    pose selects the desired suspension reference frame (body or COM), explicitly.
    Returned tuples own all values and can cross into a customer-USD process.
    """
    def __init__(self,library):self._library,self._api=library,_api(library)

    def sprung_masses(self,positions,mass,*,gravity_axis=1):
        points=tuple(_host_sequence(positions))
        if not 1<=len(points)<=20 or isinstance(gravity_axis,bool) or gravity_axis not in (0,1,2):
            raise ValueError('Expected 1..20 wheel coordinates and gravity axis 0, 1 or 2')
        points=[_floats(point,3) for point in points]
        xyz=(C.c_float*(3*len(points)))(*(v for p in points for v in p));out=(C.c_float*len(points))()
        _check(self._api.sprung_masses(len(points),xyz,_host_scalar(mass),gravity_axis,out,len(out)),'sprung masses')
        return tuple(out)

    def suspension_frame(self,body_pose,wheel_pose):
        out=(C.c_float*7)()
        _check(self._api.suspension_frame(_floats(body_pose,7),_floats(wheel_pose,7),out),'suspension frame')
        return tuple(out)

    def ackermann(self,wheel_base,track_width,inner_angle):
        out=C.c_float()
        _check(self._api.ackermann(_host_scalar(wheel_base),_host_scalar(track_width),_host_scalar(inner_angle),C.byref(out)),'Ackermann steering')
        return out.value


@dataclass(frozen=True)
class ObjectIdentity:
    attachment:int
    key:int
    generation:int


@dataclass(frozen=True)
class VehicleInput:
    throttle:float=0
    brake:float=0
    handbrake:float=0
    steer:float=0

    def __post_init__(self):
        for name in ('throttle','brake','handbrake','steer'):
            v=_host_scalar(getattr(self,name));low=-1 if name=='steer' else 0
            if not math.isfinite(v) or not low<=v<=1:raise ValueError(f'{name} outside [{low},1]')
            object.__setattr__(self,name,v)


@dataclass(frozen=True)
class EngineState:
    clutch:float
    current_gear:int
    target_gear:int
    gear_switch_time:float
    autobox_time:float
    rotation_speed:float
    automatic_transmission:bool


@dataclass(frozen=True)
class GroundContact:
    plane:tuple
    position:tuple
    actor:ObjectIdentity|None
    shape:ObjectIdentity|None
    material:ObjectIdentity|None


@dataclass(frozen=True)
class WheelState:
    identity:ObjectIdentity|None
    index:int
    position:tuple
    orientation:tuple
    frame:str
    rotation_speed:float
    rotation_angle:float
    steer_angle:float
    ground:GroundContact|None
    suspension_jounce:float
    suspension_force:tuple
    tire_friction:float
    longitudinal_direction:tuple
    lateral_direction:tuple
    longitudinal_slip:float
    lateral_slip:float
    tire_force:tuple


@dataclass(frozen=True)
class VehicleState:
    identity:ObjectIdentity
    step:int
    inputs:VehicleInput
    forward_velocity:float
    engine:EngineState|None
    wheels:tuple
    units:str='stage-length/mass, seconds, radians'


def _state(attach,s):
    def identity(value):return ObjectIdentity(attach,value.key,value.generation) if value.key else None
    wheels=[]
    if s.wheelCount>20:raise CompanionError(Status.ABI_MISMATCH,'vehicle wheel count')
    for w in s.wheels[:s.wheelCount]:
        ground=(GroundContact(tuple(w.groundPlane),tuple(w.groundPosition),identity(w.groundActor),identity(w.groundShape),identity(w.groundMaterial))
                if w.onGround else None)
        wheels.append(WheelState(identity(w.identity),w.index,tuple(w.position),tuple(w.orientation),'com-local' if w.comFrame else 'actor-local',
            w.rotationSpeed,w.rotationAngle,w.steerAngle,ground,w.suspensionJounce,tuple(w.suspensionForce),w.tireFriction,
            tuple(w.longitudinalDirection),tuple(w.lateralDirection),w.longitudinalSlip,w.lateralSlip,tuple(w.tireForce)))
    engine=(EngineState(s.clutch,s.currentGear,s.targetGear,s.gearSwitchTime,s.autoboxTime,s.engineSpeed,bool(s.automaticTransmission)) if s.engineFields else None)
    return VehicleState(identity(s.identity),s.step,VehicleInput(s.throttle,s.brake0,s.brake1,s.steer),s.forwardVelocity,engine,tuple(wheels))


_live=weakref.WeakValueDictionary()
_slots=set()
_controllers_lock=threading.Lock()
_retained=[]
_registered=False

def _cleanup():
    for controller in list(_live.values())+list(_retained):
        try:controller.close()
        except Exception as error:warnings.warn(f'Vehicle control retained after failed clear: {error}',RuntimeWarning)


class VehicleController:
    """Exclusive host-thread input adapter for one CPU managed vehicle controller.

    Call pre_step in FrameCoordinator.before_step (or your own sole stepping loop).
    Inputs latch natively until replaced. Optional response_rate limits normalized
    input change per second; auto_reverse treats brake as reverse throttle only
    below reverse_threshold (stage units/sec). Forward throttle brakes a moving
    reverse vehicle before shifting forward. Handbrake remains brake1. Direction
    starts forward; auto reverse explicitly selects reverse/first gear, replacing
    any automatic transmission request. With auto_reverse=False, transmission is
    preserved unless direction is explicitly supplied to pre_step.

    pause/disable/close clear throttle, both brakes and steer immediately under
    idle admission; they preserve transmission, body pose and velocity. Busy close
    is retryable after the host completes pending work. Removal/recreation retires
    this controller; construct a new one. No native actor pointer is cached.
    """
    def __init__(self,owner,path,*,response_rate=None,auto_reverse=False,reverse_threshold=.2):
        global _registered
        from .native import ToolsSession
        from .preview import PreviewSession
        self._closed=True
        if not isinstance(owner,(ToolsSession,PreviewSession)):raise TypeError('Pass a ToolsSession or PreviewSession')
        if not isinstance(path,str) or not path.startswith('/') or '\0' in path:raise ValueError('Expected absolute vehicle path')
        if response_rate is not None:
            response_rate=_host_scalar(response_rate)
            if not math.isfinite(response_rate) or response_rate<=0:raise ValueError('response_rate must be positive')
        reverse_threshold=_host_scalar(reverse_threshold)
        if not isinstance(auto_reverse,bool) or not math.isfinite(reverse_threshold) or reverse_threshold<0:raise ValueError('Invalid auto reverse policy')
        self._owner,self._library,self.path=owner,owner._library,path
        self._attachment=owner.identity.attachment;self._slot=(self._attachment,path)
        self._thread=threading.get_ident();self._api=_api(self._library);self._generation=0
        with _controllers_lock:
            if self._slot in _slots:raise RuntimeError('Vehicle already has a host input controller')
            state=self._execute(0);self._generation=state.identity.generation
            self.response_rate,self.auto_reverse,self.reverse_threshold=response_rate,auto_reverse,reverse_threshold
            self._filtered=VehicleInput();self._direction=1;self._enabled=True;self._closed=False
            _slots.add(self._slot);_live[self._slot]=self
            if not _registered:atexit.register(_cleanup);_registered=True

    def _check(self):
        if threading.get_ident()!=self._thread:raise RuntimeError('Vehicle control must run on its host thread')
        if self._closed:raise RuntimeError('Vehicle controller is closed')
        if self._owner.identity.attachment!=self._attachment:raise CompanionError(Status.STALE,'vehicle owner')

    def _execute(self,action,inputs=VehicleInput(),direction=-2):
        command=_Command(self._generation,action,direction,inputs.throttle,inputs.brake,inputs.handbrake,inputs.steer)
        result=_State()
        status=self._api.execute(self._attachment,self.path.encode(),C.byref(command),C.sizeof(command),C.byref(result),C.sizeof(result))
        if status in (Status.STALE,Status.NOT_FOUND):
            self._filtered=VehicleInput();self._enabled=False
        _check(status,('describe','apply','clear')[action]+' vehicle')
        return _state(self._attachment,result)

    @property
    def state(self):
        self._check();return self._execute(0)

    def pre_step(self,inputs,*,dt,direction=None):
        self._check()
        if not self._enabled:raise RuntimeError('Vehicle inputs are paused or disabled')
        if not isinstance(inputs,VehicleInput):raise TypeError('Pass VehicleInput')
        dt=_host_scalar(dt)
        if not math.isfinite(dt) or not 0<dt<=1:raise ValueError('Expected dt in (0,1] seconds')
        if direction is not None and (isinstance(direction,bool) or direction not in (-1,0,1)):raise ValueError('direction must be -1, 0, 1 or None')
        if self.auto_reverse and direction is not None:raise ValueError('Explicit direction conflicts with auto reverse')
        desired=inputs;gear=-2 if direction is None else int(direction)
        if self.auto_reverse:
            speed=self._execute(0).forward_velocity;gear=self._direction
            if inputs.throttle>0:
                if speed < -self.reverse_threshold:desired=VehicleInput(0,max(inputs.brake,inputs.throttle),inputs.handbrake,inputs.steer)
                else:gear=1
            elif inputs.brake>0 and (gear==-1 or abs(speed)<=self.reverse_threshold):
                gear=-1;desired=VehicleInput(inputs.brake,0,inputs.handbrake,inputs.steer)
        if self.response_rate is not None:
            amount=self.response_rate*dt
            desired=VehicleInput(*(old+max(-amount,min(amount,new-old)) for old,new in zip(
                (self._filtered.throttle,self._filtered.brake,self._filtered.handbrake,self._filtered.steer),
                (desired.throttle,desired.brake,desired.handbrake,desired.steer))))
        # No filter/direction progress on rejected native admission.
        state=self._execute(1,desired,gear)
        self._filtered=desired
        if gear!=-2:self._direction=gear
        return state

    def pause(self):
        self._check();state=self._execute(2);self._filtered=VehicleInput();self._enabled=False;return state

    def resume(self):
        self._check();self._execute(0);self._enabled=True

    def set_enabled(self,enabled):
        if not isinstance(enabled,bool):raise TypeError('enabled must be bool')
        return self.resume() if enabled else self.pause()

    def close(self):
        if self._closed:return
        if threading.get_ident()!=self._thread:raise RuntimeError('Close vehicle control on its host thread')
        try:self._execute(2)
        except CompanionError as error:
            if error.status not in (Status.STALE,Status.NOT_FOUND):raise
        self._closed=True;self._enabled=False;self._filtered=VehicleInput();self._owner=None
        with _controllers_lock:
            _slots.discard(self._slot)
            if _live.get(self._slot) is self:_live.pop(self._slot,None)

    def __enter__(self):self._check();return self
    def __exit__(self,*args):self.close()
    def __del__(self):
        if not getattr(self,'_closed',True):
            try:self.close()
            except Exception:_retained.append(self)


# @implements REQ-COMP-OVSTAGE-001
# @covers AC-1
def population_desc():
    """Resolved population plus vehicle suspension reference-frame metadata.

    Pass this descriptor to ovstage.population.open_usd_with_desc. No customer USD
    is imported. Omitting the metadata preserves the runtime's legacy COM default.
    ALL population resolves nested world transforms in OVStage 0.3.
    """
    import ovstage
    from ovstage.population import Desc, Selector, PrimPredicate
    return Desc(domains=ovstage.PopulationDomain.ALL,selectors=[Selector(
        PrimPredicate.has_applied_schema('PhysxVehicleAPI'),
        prim_metadata_paths=['customData:physxVehicle:referenceFrameIsCenterOfMass'])])
