# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-OVD-001
# @covers AC-1 AC-2 AC-4
"""Native bounded OVD decoding and process recording coordination; no USD imports."""
import ctypes as C
from dataclasses import asdict, dataclass
import json
import struct
import threading
import weakref
from .native import CompanionError, Status, _check


class DecodeError(ValueError):
    def __init__(self,status,offset,message):
        super().__init__(f'OVD decode at byte {offset}: {message}')
        self.status,self.offset=status,offset


class _Limits(C.Structure):
    _fields_=[('size',C.c_uint32),('version',C.c_uint32),('input_bytes',C.c_uint64),
              ('records',C.c_uint32),('payload_bytes',C.c_uint32),('string_bytes',C.c_uint32),('reserved',C.c_uint32)]


class _Diagnostic(C.Structure):
    _fields_=[('size',C.c_uint32),('version',C.c_uint32),('offset',C.c_uint64),('message',C.c_char*160)]


class _Info(C.Structure):
    _fields_=[(name,C.c_uint32) for name in ('size','version','major','minor','patch','reserved')]+[('records',C.c_uint64)]


class _Record(C.Structure):
    _fields_=([(name,C.c_uint32) for name in ('size','version','command','reserved')]+
              [(name,C.c_uint64) for name in ('offset','context','object','timestamp')]+
              [(name,C.c_uint32) for name in ('class_handle','base_class','attribute','data_type','elements',
                'attribute_class','enum_value','line','message_type','name_bytes','file_bytes','path_count','payload_bytes')])


class _Api(C.Structure):
    _fields_=[('size',C.c_uint32),('version',C.c_uint32),('capabilities',C.c_uint64),
              ('decode',C.CFUNCTYPE(C.c_uint32,C.c_void_p,C.c_uint64,C.POINTER(_Limits),C.POINTER(C.c_uint64),C.POINTER(_Diagnostic))),
              ('describe',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.c_uint32,C.POINTER(_Info))),
              ('record',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.c_uint64,C.c_uint32,C.POINTER(_Record))),
              ('copy',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.c_uint64,C.c_uint32,C.c_void_p,C.c_uint64,C.POINTER(C.c_uint64))),
              ('release',C.CFUNCTYPE(C.c_uint32,C.c_uint64))]


@dataclass(frozen=True)
class Record:
    offset: int
    command: int
    context: int=0
    object: int=0
    timestamp: int=0
    class_handle: int=0
    base_class: int=0
    attribute: int=0
    data_type: int=0
    elements: int=0
    attribute_class: int=0
    enum_value: int=0
    line: int=0
    message_type: int=0
    name: str=''
    file: str=''
    attribute_path: tuple=()
    payload: bytes=b''


@dataclass(frozen=True)
class Document:
    version: tuple
    records: tuple

    def to_json(self):
        """Portable worker-to-customer-USD transport; no native pointers or pickle."""
        records=[]
        for record in self.records:
            fields=asdict(record);fields['payload']=record.payload.hex();records.append(fields)
        return json.dumps({'version':self.version,'records':records})

    @classmethod
    def from_json(cls,text,*,max_bytes=128*1024*1024):
        if not isinstance(text,str) or len(text)>max_bytes:raise ValueError('OVD JSON exceeds limit or is not text')
        value=json.loads(text)
        if set(value)!= {'version','records'}:raise ValueError('Invalid OVD document fields')
        version=tuple(value['version'])
        if len(version)!=3 or any(type(v) is not int or v<0 for v in version):raise ValueError('Invalid OVD version')
        records=[]
        for fields in value['records']:
            fields=dict(fields)
            fields['payload']=bytes.fromhex(fields['payload'])
            fields['attribute_path']=tuple(fields['attribute_path'])
            records.append(Record(**fields))
        return cls(version,tuple(records))


def _acquire(library):
    table=_Api()
    try:acquire=library.ovphysx_companion_ovd_acquire
    except AttributeError as error:raise CompanionError(Status.UNAVAILABLE,'OVD decoder table') from error
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
    _check(acquire(1,C.sizeof(table),C.byref(table)),'acquire OVD decoder')
    if (table.size!=C.sizeof(table) or table.version!=1 or not table.capabilities&1 or
            not all((table.decode,table.describe,table.record,table.copy,table.release))):
        raise CompanionError(Status.ABI_MISMATCH,'OVD decoder table')
    return table


def decode(library,data,*,max_input_bytes=64*1024*1024,max_records=1000000,
           max_payload_bytes=16*1024*1024,max_string_bytes=2048):
    """Copy a bounded stream through the native decoder, returning immutable records.

    library is the host's existing core CDLL. Decoding initializes no runtime and
    requires no attachment. Native command parsing preserves all attribute-path
    components and raw frame stamps; consumers must supply time/unit policy.
    """
    limits=(max_input_bytes,max_records,max_payload_bytes,max_string_bytes)
    if any(type(v) is not int or v<=0 for v in limits):raise ValueError('Decoder limits must be positive integers')
    if (max_input_bytes>512*1024*1024 or max_records>1000000 or max_payload_bytes>max_input_bytes or
            max_string_bytes>65535):raise ValueError('Decoder limits exceed supported bounds')
    if not isinstance(data,(bytes,bytearray,memoryview)):raise TypeError('OVD input must be ready CPU bytes')
    if memoryview(data).nbytes>max_input_bytes:raise DecodeError(8,0,'Input exceeds configured limit')
    data=bytes(data)
    table=_acquire(library)
    config=_Limits(C.sizeof(_Limits),1,*limits)
    handle=C.c_uint64();diagnostic=_Diagnostic()
    status=table.decode(C.c_char_p(data),len(data),C.byref(config),C.byref(handle),C.byref(diagnostic))
    if status!=Status.OK:
        raise DecodeError(status,diagnostic.offset,bytes(diagnostic.message).decode('utf-8',errors='replace'))
    try:
        info=_Info()
        _check(table.describe(handle.value,C.sizeof(info),C.byref(info)),'describe OVD')
        if info.records>max_records:raise CompanionError(Status.ABI_MISMATCH,'OVD record count')
        result=[]
        for index in range(info.records):
            record=_Record()
            _check(table.record(handle.value,index,C.sizeof(record),C.byref(record)),'read OVD record')
            sizes=(record.name_bytes,record.file_bytes,record.path_count*4,record.payload_bytes)
            if (sizes[0]>max_string_bytes or sizes[1]>max_string_bytes or record.path_count>255 or sizes[3]>max_payload_bytes):
                raise CompanionError(Status.ABI_MISMATCH,'OVD record sizes')
            buffers=[]
            for kind,size in enumerate(sizes):
                buffer=(C.c_ubyte*size)();required=C.c_uint64()
                _check(table.copy(handle.value,index,kind,buffer,size,C.byref(required)),'copy OVD record')
                if required.value!=size:raise CompanionError(Status.ABI_MISMATCH,'OVD record copy')
                buffers.append(bytes(buffer))
            fields={name:getattr(record,name) for name,_ in _Record._fields_ if name not in
                    ('size','version','reserved','name_bytes','file_bytes','path_count','payload_bytes')}
            try:
                fields['name']=buffers[0].decode('utf-8');fields['file']=buffers[1].decode('utf-8')
            except UnicodeDecodeError as error:raise DecodeError(9,record.offset,'Name is not UTF-8') from error
            fields['attribute_path']=struct.unpack('<'+'I'*record.path_count,buffers[2])
            fields['payload']=buffers[3]
            result.append(Record(**fields))
        return Document((info.major,info.minor,info.patch),tuple(result))
    finally:_check(table.release(handle.value),'release OVD document')


def attribute_values(data_type,payload):
    """Decode primitive little-endian payloads. Class-valued layouts need schema policy."""
    if data_type==10:return payload.rstrip(b'\0').decode('utf-8')
    # PxFlags stores its actual underlying word width (uint8/uint16/uint32),
    # although the schema's FLAGS_WORD type is associated with a uint32 enum class.
    if data_type==13:
        if len(payload) not in (1,2,4,8):raise ValueError('Unsupported OVD flag-word width')
        return (int.from_bytes(payload,'little'),)
    formats={0:'b',1:'h',2:'i',3:'q',4:'B',5:'H',6:'I',7:'Q',8:'f',9:'d',11:'Q',12:'I',13:'I'}
    if data_type not in formats:raise ValueError('Unsupported OVD attribute data type')
    code=formats[data_type];size=struct.calcsize('<'+code)
    if len(payload)%size:raise ValueError('OVD payload length does not match its data type')
    return struct.unpack('<'+code*(len(payload)//size),payload)


_recording_lock=threading.Lock()
_recording=None


class RecordingCoordinator:
    """One host coordinator for the shared SDK capture, delegated to core recording.

    Before the first PhysX instance, call
    set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True). This bool
    defaults to false and stays dormant until explicit start. Writes freeze
    after the first successful instance creation until full successful shutdown with
    no live instances. Create, attach and reattach never start recording;
    an explicit start may occur before the first simulation step.
    Transitions use the core's all-owner admission: pending previews return Busy
    without consuming work. This object never owns preview close/detach. Core
    detach by the recording instance finalizes the file per core lifecycle policy.
    The host must route recording controls through this coordinator; direct core
    callers still obey native recording ownership/admission.
    """
    def __init__(self,physics):
        global _recording
        self._owner=physics;self._thread=threading.get_ident();self._closed=True
        with _recording_lock:
            if _recording is not None and _recording() is not None:
                raise RuntimeError('A process recording coordinator already exists')
            _recording=weakref.ref(self)
        self._closed=False

    def _check(self):
        if threading.get_ident()!=self._thread:raise RuntimeError('Recording controls belong to the host approved thread')
        if self._closed:raise RuntimeError('Recording coordinator is closed')

    @property
    def active(self):self._check();return self._owner.is_recording()

    def start(self,destination):
        self._check();self._owner.start_recording(destination)

    def stop(self):
        self._check()
        if self._owner.is_recording():self._owner.stop_recording()

    def close(self):
        global _recording
        if self._closed:return
        self.stop() # Busy retains ownership; caller drains its own owners then retries.
        with _recording_lock:
            if _recording is not None and _recording() is self:_recording=None
        self._closed=True;self._owner=None

    def __enter__(self):self._check();return self
    def __exit__(self,*args):self.close()
