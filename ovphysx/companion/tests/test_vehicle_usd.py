# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-VEHICLE-001
# @covers AC-1 AC-4
import pytest
from pxr import UsdGeom,UsdPhysics
from ovphysx.utils import codeless
from ovphysx_companion.usd.vehicle import apply_authoring
from ovphysx_companion.usd.session import StaleResultError


def prepare(stage):
    vehicle=stage.DefinePrim('/World/Car','Xform');codeless.apply_api(vehicle,'PhysxVehicleAPI')
    vehicle.SetCustomDataByKey('physxVehicle:referenceFrameIsCenterOfMass',False)
    wheel=stage.DefinePrim('/World/Car/Wheel','Xform');codeless.apply_api(wheel,'PhysxVehicleWheelAttachmentAPI')
    suspension=stage.DefinePrim('/World/Suspension','Xform');codeless.apply_api(suspension,'PhysxVehicleSuspensionAPI')
    return vehicle,wheel,suspension


def test_vehicle_authoring_unit_conversion_atomic_undo(scene):
    stage,session=scene
    vehicle,wheel,suspension=prepare(stage)
    UsdGeom.SetStageMetersPerUnit(stage,.01);UsdPhysics.SetStageKilogramsPerUnit(stage,.1)
    before=stage.GetRootLayer().ExportToString()
    patch=apply_authoring(session,'/World/Car',suspension_frames={'/World/Car/Wheel':(1,2,3,0,0,0,1)},
        sprung_masses={'/World/Suspension':300},reference_frame='actor-local',source=session.source,layer=stage.GetRootLayer())
    assert tuple(wheel.GetAttribute('physxVehicleWheelAttachment:suspensionFramePosition').Get())==pytest.approx((100,200,300))
    assert suspension.GetAttribute('physxVehicleSuspension:sprungMass').Get()==pytest.approx(3000)
    after=stage.GetRootLayer().ExportToString();patch.undo();assert stage.GetRootLayer().ExportToString()==before
    patch.redo();assert stage.GetRootLayer().ExportToString()==after


def test_vehicle_authoring_stale_frame_and_invalid_batch(scene):
    stage,session=scene
    vehicle,wheel,suspension=prepare(stage)
    args=dict(suspension_frames={'/World/Car/Wheel':(1,2,3,0,0,0,1)},sprung_masses={},reference_frame='actor-local',source=session.source,layer=stage.GetRootLayer())
    with pytest.raises(ValueError,match='reference frame'):apply_authoring(session,'/World/Car',**(args|{'reference_frame':'com-local'}))
    before=stage.GetRootLayer().ExportToString()
    with pytest.raises(ValueError):apply_authoring(session,'/World/Car',**(args|{'sprung_masses':{'/World/Missing':100}}))
    assert stage.GetRootLayer().ExportToString()==before
    vehicle.SetCustomDataByKey('host:changed',True)
    with pytest.raises(StaleResultError):apply_authoring(session,'/World/Car',**args)
    UsdGeom.Xformable(vehicle).AddScaleOp().Set((2,1,1))
    with pytest.raises(ValueError,match='rigid transform'):apply_authoring(session,'/World/Car',**(args|{'source':session.source}))


def test_vehicle_native_worker_to_customer_authoring():
    from vehicle_authoring import run
    stage=run()
    assert stage.GetPrimAtPath('/World/Suspension').GetAttribute('physxVehicleSuspension:sprungMass').Get()==300
