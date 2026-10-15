# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-CONFIG-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Scoped settings metadata and copied owner configuration, without USD imports."""
import ctypes as C
import threading
from dataclasses import dataclass
from enum import Enum,IntEnum
from types import MappingProxyType
from .native import CompanionError,Status,_check


class Scope(str,Enum):
    PROCESS='process'
    INSTANCE='instance'
    ATTACHMENT='attachment'
    SCENE='authored-scene'
    HOST='host-preference'


@dataclass(frozen=True)
class Setting:
    name:str
    scope:Scope
    default:object
    phase:str
    persistence:str
    description:str


# Core owns concrete fallback values. None in this catalog means the declared
# policy in description, never an invented effective value of zero/False.
_GLOBAL={
    'cpu_only':('CPU_ONLY','before-first-instance','Hard CPU policy; sticky once enabled.'),
    'num_threads':('NUM_THREADS','facade-detached','Default for future attachments; does not rewrite existing previews.'),
    'disable_contact_processing':('DISABLE_CONTACT_PROCESSING','idle','Core global policy and current facade attachment. Owner-local changes use AttachmentConfiguration.'),
    'collision_cone_custom_geometry':('COLLISION_CONE_CUSTOM_GEOMETRY','facade-detached','Collision representation for future attachments.'),
    'collision_cylinder_custom_geometry':('COLLISION_CYLINDER_CUSTOM_GEOMETRY','facade-detached','Collision representation for future attachments.'),
    'scene_multi_gpu_mode':('SCENE_MULTI_GPU_MODE','facade-detached','Future scene distribution default; explicit instance GPU selection takes precedence.'),
    'ovstage_read_pool_max_mb':('OVSTAGE_READ_POOL_MAX_MB','idle','Device read-buffer pool retention budget in MiB.'),
    'ovd_recording_capable':('OVD_RECORDING_CAPABLE','before-first-instance','Bool, default false. Process recording capability stays dormant until explicit start. Writes freeze after the first successful instance creation until full successful shutdown with no live instances.'),
    'profiler_backend':('PROFILER_BACKEND','before-first-instance','Process profiler backend; the core validates supported values.'),
    'profiler_output_path':('PROFILER_OUTPUT_PATH','before-first-instance','Trace destination retained by the core.'),
    'cooked_collider_cache_directory':('COOKED_COLLIDER_CACHE_DIRECTORY','before-first-instance','Process cooked-cache destination.'),
    'log_level':('LOG_LEVEL','live','Process logging threshold.'),
    'default_log_output':('DEFAULT_LOG_OUTPUT','live','Core default log sink enabled state.'),
}
_entries=[Setting('process.'+name,Scope.PROCESS,False if name=='ovd_recording_capable' else None,phase,'application startup/configuration',
    description+' get_global returns the retained/effective value, falling back to the core default. Only unset string keys return None.')
    for name,(_,phase,description) in _GLOBAL.items()]
_entries += [Setting('instance.active_cuda_gpus',Scope.INSTANCE,None,'before-attach','application instance configuration',
                     'Default delegates to core automatic selection; core retains the validated ordinal list.'),
             Setting('attachment.disable_contact_processing',Scope.ATTACHMENT,None,'idle','attachment lifetime',
                     'Initially copied from the source/defaults. Owner-local changes preserve all peers.')]
_entries += [Setting('attachment.'+name,Scope.ATTACHMENT,None,'construction-only','attachment lifetime',
                     'None copies the bootstrap attachment setting, or process default when unattached. Read snapshot for retained value.')
             for name in ('threads','cuda_device','gpu_pipeline','solver')]
_entries += [Setting('attachment.'+name,Scope.ATTACHMENT,None,'read-only captured request','attachment lifetime',
                     'Copied native setting; read snapshot for retained value. No mutation or resource release is exposed here.')
             for name in ('multi_gpu_mode','gizmo_scale','native_pose_output','native_velocity_output',
                          'native_particle_output','borrow_active_cuda_context','synchronous_cuda',
                          'physx_dispatcher','suppress_readback','local_velocity_output')]
_entries += [Setting('scene.'+name,Scope.SCENE,None,'authoring; effective after explicit population/recreation','selected USD layer',
                     'None preserves authored opinion/schema fallback. Native attachment overrides and hard CPU policy still apply.')
             for name in ('gravity_world_mps2','solver','gpu_enabled')]
_entries += [Setting('host.show_debug_geometry',Scope.HOST,False,'host-thread','caller-provided preference store',
                     'Presentation preference only; host decides whether to configure DebugDraw.'),
             Setting('host.vehicle_auto_reverse',Scope.HOST,False,'host-thread','caller-provided preference store',
                     'Host input policy only; host constructs/configures VehicleController explicitly.')]
CATALOG=MappingProxyType({entry.name:entry for entry in _entries})
del _entries


def _global_key(name):
    from ovphysx import GlobalConfigKey
    try:return GlobalConfigKey[_GLOBAL[name][0]]
    except KeyError as error:raise ValueError('Unknown exposed process configuration setting') from error


def set_global(name,value):
    """Delegate to the typed core, including lifecycle/type/conflict validation.

    This explicit process action can change defaults for future attachments. It
    is never used to temporarily configure Inspector. No persistence is implicit.
    """
    from ovphysx import set_global_config
    set_global_config(_global_key(name),value)


def get_global(name):
    from ovphysx import get_global_config
    return get_global_config(_global_key(name))


def set_instance_devices(physics,value):
    from ovphysx import ConfigKey
    physics.set_config(ConfigKey.ACTIVE_CUDA_GPUS,value)


def get_instance_devices(physics):
    from ovphysx import ConfigKey
    return physics.get_config(ConfigKey.ACTIVE_CUDA_GPUS)


class Pipeline(IntEnum):
    AUTHORED=-1
    CPU=0
    GPU=1


class Solver(IntEnum):
    AUTHORED=-1
    PGS=0
    TGS=1


@dataclass(frozen=True)
class AttachmentOptions:
    """Copied settings for a new PreviewSession; never writes process defaults."""
    threads:int|None=None
    cuda_device:int|None=None
    gpu_pipeline:Pipeline|None=None
    solver:Solver|None=None

    def __post_init__(self):
        for name,lo,hi in (('threads',1,255),('cuda_device',-1,2**31-1),('gpu_pipeline',-1,1),('solver',-1,1)):
            value=getattr(self,name)
            if value is not None and (isinstance(value,bool) or not isinstance(value,int) or not lo<=value<=hi):
                raise ValueError(f'{name} requires an integer in [{lo},{hi}] or None')

    def open_preview(self,physics,stage,*,read_ordinal,excluded_paths=()):
        from .preview import PreviewSession
        return PreviewSession(physics,stage,read_ordinal=read_ordinal,excluded_paths=excluded_paths,
            threads=self.threads,cuda_device=self.cuda_device,gpu_pipeline=self.gpu_pipeline,solver=self.solver)


class _Snapshot(C.Structure):
    _fields_=[('size',C.c_uint32),('version',C.c_uint32),('attachment',C.c_uint64),('step',C.c_uint64),
        ('threads',C.c_uint32),('flags',C.c_uint32),('cuda_device',C.c_int32),('multi_gpu_mode',C.c_int32),
        ('gpu_pipeline',C.c_int32),('solver',C.c_int32),('gizmo_scale',C.c_float),('reserved',C.c_uint32)]


class _Api(C.Structure):
    _fields_=[('size',C.c_uint32),('version',C.c_uint32),('capabilities',C.c_uint64),
        ('get_configuration',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.c_uint32,C.POINTER(_Snapshot))),
        ('set_contact_processing_disabled',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.c_uint32,C.c_uint32,C.POINTER(_Snapshot)))]


@dataclass(frozen=True)
class AttachmentSnapshot:
    attachment:int
    step:int
    threads:int
    cuda_device:int
    multi_gpu_mode:int
    gpu_pipeline:Pipeline
    solver:Solver
    gizmo_scale:float
    disable_contact_processing:bool
    native_pose_output:bool
    native_velocity_output:bool
    native_particle_output:bool
    borrow_active_cuda_context:bool
    synchronous_cuda:bool
    physx_dispatcher:bool
    suppress_readback:bool
    local_velocity_output:bool


class AttachmentConfiguration:
    """Retain a ToolsSession/PreviewSession and inspect its copied settings.

    Snapshots contain requested resources, not a GPU availability assertion. The
    runtime owns dispatchers/contexts; no resource can be manually released here.
    An active context borrowed by lower-level configuration must remain alive
    through runtime shutdown. Resources/solver/output are immutable here. Rebuild
    a preview with copied AttachmentOptions to change construction-time settings.
    Only contact processing has an owner-local live mutation, admitted while idle.
    """
    def __init__(self,owner):
        from .native import ToolsSession
        from .preview import PreviewSession
        if not isinstance(owner,(ToolsSession,PreviewSession)):raise TypeError('Pass a ToolsSession or PreviewSession')
        self._owner,self._library=owner,owner._library
        self._attachment=owner.identity.attachment;self._api=_Api()
        try:acquire=self._library.ovphysx_companion_configuration_acquire
        except AttributeError as error:raise CompanionError(Status.UNAVAILABLE,'configuration table') from error
        acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
        _check(acquire(1,C.sizeof(self._api),C.byref(self._api)),'acquire configuration')
        if self._api.size!=C.sizeof(self._api) or self._api.version!=1 or self._api.capabilities&3!=3 or not self._api.get_configuration or not self._api.set_contact_processing_disabled:
            raise CompanionError(Status.ABI_MISMATCH,'configuration table')

    def _execute(self,action=0,disabled=False):
        if self._owner.identity.attachment!=self._attachment:raise CompanionError(Status.STALE,'configuration owner')
        result=_Snapshot()
        status=(self._api.get_configuration(self._attachment,C.sizeof(result),C.byref(result)) if action==0 else
                self._api.set_contact_processing_disabled(self._attachment,int(disabled),C.sizeof(result),C.byref(result)))
        _check(status,'attachment configuration')
        return AttachmentSnapshot(result.attachment,result.step,result.threads,result.cuda_device,result.multi_gpu_mode,
            Pipeline(result.gpu_pipeline),Solver(result.solver),result.gizmo_scale,
            *(bool(result.flags&(1<<bit)) for bit in range(9)))

    @property
    def snapshot(self):return self._execute()

    def set_contact_processing_disabled(self,disabled):
        if not isinstance(disabled,bool):raise TypeError('disabled must be bool')
        return self._execute(1,disabled)


class HostPreferences:
    """Store host-only booleans in a caller-owned mapping; no native side effects.

    The host owns thread scheduling and persistence of its mapping. These values
    never pretend to be scene or SDK settings; adapters consume them explicitly.
    """
    def __init__(self,store):
        self._store=store
        self._thread=threading.get_ident()

    def _check_thread(self):
        if threading.get_ident()!=self._thread:raise RuntimeError("Host preferences require their creating thread")

    def get(self,name):
        self._check_thread()
        item=CATALOG.get('host.'+name)
        if item is None:raise ValueError('Unknown host preference')
        value=self._store.get(name,item.default)
        if not isinstance(value,bool):raise ValueError('Stored host preference must be bool')
        return value

    def set(self,name,value):
        self._check_thread()
        if 'host.'+name not in CATALOG or not isinstance(value,bool):raise ValueError('Unknown host preference or non-bool value')
        self._store[name]=value
