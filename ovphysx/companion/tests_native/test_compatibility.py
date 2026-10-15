# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-API-001
# @covers AC-1 AC-2 AC-3 AC-4
import ctypes as C
import importlib
import pytest
from ovphysx_companion.compatibility import SERVICE_ABIS
from ovphysx_companion.native import Status,CompanionError,_acquire


@pytest.mark.parametrize('name',SERVICE_ABIS)
def test_every_declared_table_negotiates_without_touching_rejected_output(physics,name):
    module,layout,abi=SERVICE_ABIS[name]
    table_type=getattr(importlib.import_module('ovphysx_companion.'+module),layout)
    class Guarded(C.Structure):
        _fields_=[('before',C.c_uint64),('table',table_type),('after',C.c_uint64)]
    value=Guarded();C.memset(C.byref(value),0xa5,C.sizeof(value));original=bytes(value)
    acquire=getattr(physics._lib,'ovphysx_companion_'+(name+'_' if name else '')+'acquire')
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(table_type)];acquire.restype=C.c_uint32
    for version,size in ((abi+1,C.sizeof(table_type)),(abi,C.sizeof(table_type)-1),(abi,C.sizeof(table_type)+1)):
        assert acquire(version,size,C.byref(value.table))==Status.ABI_MISMATCH
        assert bytes(value)==original
    assert acquire(abi,C.sizeof(table_type),C.byref(value.table))==Status.OK
    assert value.table.size==C.sizeof(table_type) and value.table.version==abi
    assert value.before==value.after==0xa5a5a5a5a5a5a5a5


def test_absent_service_and_abi_mismatch_are_distinct_before_session_open():
    class Absent:pass
    with pytest.raises(CompanionError) as error:_acquire(Absent())
    assert error.value.status==Status.UNAVAILABLE
    calls=[]
    class Reject:
        argtypes=None;restype=None
        def __call__(self,*args):calls.append('acquire');return Status.ABI_MISMATCH
    class Incompatible:ovphysx_companion_acquire=Reject()
    with pytest.raises(CompanionError) as error:_acquire(Incompatible())
    assert error.value.status==Status.ABI_MISMATCH and calls==['acquire']
