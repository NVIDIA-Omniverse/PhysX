# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.bindings._physx import PhysxPropertyQueryRigidBodyResponse, PhysxPropertyQueryColliderResponse, PhysxPropertyQueryResult, PhysxPropertyQueryMode
from pxr import UsdGeom, UsdUtils, Gf, PhysicsSchemaTools
from omni.physx import get_physx_property_query_interface
import weakref
import carb
from enum import auto, IntEnum

class Result():
    def __init__(self, prim=None):
        self._prim = prim

        self.mass = -1.0
        self.diagonal_inertia = Gf.Vec3f(-1.0)
        self.center_of_mass = Gf.Vec3f(0.0)
        self.principal_axes = Gf.Quatf().GetIdentity()

        self.colliders_local_aabbs = []
        self.colliders_volumes = []
        self.colliders_path_ids = []
        self.colliders_stage_ids = []

    def get_bbox(self) -> Gf.BBox3d:
        bbox = Gf.BBox3d()
        xform_cache = UsdGeom.XformCache()
        rigid_body_xform = xform_cache.GetLocalToWorldTransform(self._prim)
        bbox.SetMatrix(rigid_body_xform)
        if len(self.colliders_local_aabbs) == 0:
             return bbox

        range3d = Gf.Range3d()
        stage = self._prim.GetStage()

        for n in range(len(self.colliders_local_aabbs)):
            collider_prim = stage.GetPrimAtPath(PhysicsSchemaTools.intToSdfPath(self.colliders_path_ids[n]))
            xform = xform_cache.ComputeRelativeTransform(collider_prim, self._prim)[0]
            range3d.UnionWith(Gf.Range3d(xform.Transform(self.colliders_local_aabbs[n].GetMin()), xform.Transform(self.colliders_local_aabbs[n].GetMax())))

        bbox.SetRange(range3d)
        return bbox

class Query():
    class Status(IntEnum):
        UNSET = 0
        IN_PROGRESS = auto()
        COMPLETE = auto()
        FAIL = auto()

    def __init__(self, prim):
        self.status = Query.Status.UNSET
        self.result = Result(prim)
        self._prim = prim

class QueryManager():
    def __init__(self):
        self._query_object = None
        self.timeout = 60000

    def get_query_status(self) -> Query.Status:
        if self._query_object is not None:
            return self._query_object.status
        return Query.Status.UNSET

    def get_query_result(self) -> Result:
        if self._query_object is not None:
            return self._query_object.result
        return Result()

    # To be overridden by derived classes.
    def on_query_finished(self):
        pass

    def submit_query(self, prim):
        self._query_object = Query(prim)

        # Weak reference that we pass to the query functions to allow them to track if they are still relevant when finished.
        # If a new query is made or the parent is deactivated or removed in the meantime, the reference will point to None.
        physx_property_query_object_ref = weakref.ref(self._query_object)

        def _on_rigid_body_info_received(rigid_body_info : PhysxPropertyQueryRigidBodyResponse):
            nonlocal physx_property_query_object_ref
            physx_property_query_object = physx_property_query_object_ref()
            if (physx_property_query_object is None):
                return

            if rigid_body_info.result != PhysxPropertyQueryResult.VALID:
                physx_property_query_object.status = Query.Status.FAIL
                carb.log_error(f"PhysX rigid body property query for {physx_property_query_object._prim.GetPrimPath()} returned with error: {rigid_body_info.result.name}")
                return

            physx_property_query_object.result.mass = rigid_body_info.mass
            physx_property_query_object.result.diagonal_inertia = Gf.Vec3f(*rigid_body_info.inertia)
            physx_property_query_object.result.center_of_mass = Gf.Vec3f(*rigid_body_info.center_of_mass)
            physx_property_query_object.result.principal_axes = Gf.Quatf(rigid_body_info.principal_axes[3], rigid_body_info.principal_axes[0], rigid_body_info.principal_axes[1], rigid_body_info.principal_axes[2])

        def _on_collider_info_received(collider_info : PhysxPropertyQueryColliderResponse):
            nonlocal physx_property_query_object_ref
            physx_property_query_object = physx_property_query_object_ref()
            if (physx_property_query_object is None):
                return

            if collider_info.result != PhysxPropertyQueryResult.VALID:
                physx_property_query_object.status = Query.Status.FAIL
                carb.log_error(f"PhysX rigid body property query for {physx_property_query_object._prim.GetPrimPath()} returned with error: {collider_info.result.name}")
                return

            physx_property_query_object.result.colliders_local_aabbs.append(Gf.Range3d(Gf.Vec3d(*collider_info.aabb_local_min), Gf.Vec3d(*collider_info.aabb_local_max)))
            physx_property_query_object.result.colliders_volumes.append(collider_info.volume)
            physx_property_query_object.result.colliders_path_ids.append(collider_info.path_id)
            physx_property_query_object.result.colliders_stage_ids.append(collider_info.stage_id)

        def _on_property_query_finished():
            nonlocal physx_property_query_object_ref
            physx_property_query_object = physx_property_query_object_ref()

            if physx_property_query_object is None:
                return

            assert physx_property_query_object == self._query_object, "Physx property query error - a new query was made without the current being deleted."

            if physx_property_query_object.status == Query.Status.FAIL:
                return

            physx_property_query_object.status = Query.Status.COMPLETE

            self.on_query_finished()

        body_path = PhysicsSchemaTools.sdfPathToInt(prim.GetPrimPath())
        stage_id = UsdUtils.StageCache.Get().Insert(prim.GetStage()).ToLongInt()
        self._query_object.status = Query.Status.IN_PROGRESS

        get_physx_property_query_interface().query_prim(stage_id = stage_id, 
                                                        prim_id = body_path,
                                                        query_mode = PhysxPropertyQueryMode.QUERY_RIGID_BODY_WITH_COLLIDERS,
                                                        finished_fn = _on_property_query_finished,
                                                        rigid_body_fn = _on_rigid_body_info_received,
                                                        collider_fn = _on_collider_info_received,
                                                        timeout_ms = self.timeout) # Timeout after 1 minute.
