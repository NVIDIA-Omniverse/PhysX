# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

# This line isn't strictly necessary. It's only useful for more stringent type information of the compute parameter.
# Note how the extra submodule "ogn" is appended to the extension's module to find the database file.
from omni.physxgraph.ogn.OgnPhysXPropertyQueryRigidBodyDatabase import OgnPhysXPropertyQueryRigidBodyDatabase
from omni.physx import get_physx_property_query_interface
from omni.physx.bindings._physx import PhysxPropertyQueryRigidBodyResponse, PhysxPropertyQueryColliderResponse, PhysxPropertyQueryResult, PhysxPropertyQueryMode
from pxr import UsdUtils, UsdPhysics, PhysicsSchemaTools, Sdf, Gf, UsdGeom
import weakref
import time
import omni.usd
import carb


class PhysxPropertyQueryObject():
    class Status:
        IN_PROGRESS = 0
        COMPLETE = 1
        FAIL = 2

    def __init__(self, stage, prim, use_local_frame):
        self.status = PhysxPropertyQueryObject.Status.IN_PROGRESS
        self.stage = stage
        self.prim = prim
        self.use_local_frame = use_local_frame
        self.body_mass = -1.0
        self.body_diagonal_inertia = None
        self.body_center_of_mass = None
        self.body_principal_axes = None
        self.body_bounds = Gf.Range3d()
        self.xform_cache = UsdGeom.XformCache()

class OgnPhysXPropertyQueryRigidBody:
    @staticmethod
    def compute(db: OgnPhysXPropertyQueryRigidBodyDatabase) -> bool:
        """Retrieves physics property information of the input rigid body."""
        num_prims = len(db.inputs.prim)
        if num_prims == 0:
            return True

        stage = omni.usd.get_context().get_stage()

        # Targets are received as USDRT while PhysicsSchemaTools is based on USD, which is why we have to do this conversion.
        body_path = Sdf.Path(db.inputs.prim[0].GetToken())
        body_path_physics = PhysicsSchemaTools.sdfPathToInt(body_path)
        body_prim = stage.GetPrimAtPath(body_path)
        if not body_prim or not body_prim.IsValid():
            db.log_error(f"Failed to retrieve input prim at path: {body_path}")
            return False

        if not body_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            db.log_error(f"Input prim does not have the rigid body API applied: {body_path}")
            return False

        # Fetch mass properties from PhysX.
        physx_property_query_object = PhysxPropertyQueryObject(stage, body_prim, db.inputs.use_local_frame)

        # Weak reference that we pass to the query functions to allow them to track if they are still relevant when finished.
        # If a new query is made or the node is deactivated or removed in the meantime, the reference will point to None.
        physx_property_query_object_ref = weakref.ref(physx_property_query_object)

        def on_rigid_body_info_received(rigid_info : PhysxPropertyQueryRigidBodyResponse):
            nonlocal physx_property_query_object_ref
            physx_property_query_object = physx_property_query_object_ref()

            if (physx_property_query_object is None):
                return

            if rigid_info.result == PhysxPropertyQueryResult.VALID: 
                physx_property_query_object.body_mass = rigid_info.mass
                physx_property_query_object.body_diagonal_inertia = rigid_info.inertia
                center_of_mass = Gf.Vec3d(*rigid_info.center_of_mass)
                if not physx_property_query_object.use_local_frame:
                    xform = physx_property_query_object.xform_cache.GetLocalToWorldTransform(physx_property_query_object.prim)
                    center_of_mass = xform.Transform(center_of_mass)
                physx_property_query_object.body_center_of_mass = [center_of_mass[0], center_of_mass[1], center_of_mass[2]]
                physx_property_query_object.body_principal_axes = rigid_info.principal_axes
            else:
                physx_property_query_object.status = PhysxPropertyQueryObject.Status.FAIL
                carb.log_error(f"PhysX rigid body property query for failed for prim {physx_property_query_object.prim}: {rigid_info.result.name}")

        def on_collider_info_received(collider_info : PhysxPropertyQueryColliderResponse):
            nonlocal physx_property_query_object_ref
            physx_property_query_object = physx_property_query_object_ref()

            if (physx_property_query_object is None):
                return

            if collider_info.result == PhysxPropertyQueryResult.VALID:
                collider_prim = physx_property_query_object.stage.GetPrimAtPath(PhysicsSchemaTools.intToSdfPath(collider_info.path_id))
                if collider_prim is None or not collider_prim.IsValid():
                    physx_property_query_object.status = PhysxPropertyQueryObject.Status.FAIL
                    carb.log_error(f"PhysX rigid body property query for failed for collider {PhysicsSchemaTools.intToSdfPath(collider_info.path_id)} attached to prim {physx_property_query_object.prim}: {collider_info.result.name}")
                elif physx_property_query_object.use_local_frame:
                    xform = physx_property_query_object.xform_cache.ComputeRelativeTransform(collider_prim, physx_property_query_object.prim)[0]
                else:
                    xform = physx_property_query_object.xform_cache.GetLocalToWorldTransform(collider_prim)

                collider_bbox = Gf.BBox3d(Gf.Range3d(Gf.Vec3d(*collider_info.aabb_local_min), Gf.Vec3d(*collider_info.aabb_local_max)), xform).ComputeAlignedRange()
                physx_property_query_object.body_bounds.UnionWith(collider_bbox)
            else:
                physx_property_query_object.status = PhysxPropertyQueryObject.Status.FAIL
                carb.log_error(f"PhysX rigid body property query for failed for collider {PhysicsSchemaTools.intToSdfPath(collider_info.path_id)} attached to prim {physx_property_query_object.prim}: {collider_info.result.name}")

        def on_property_query_finished():
            nonlocal physx_property_query_object_ref
            physx_property_query_object = physx_property_query_object_ref()

            if physx_property_query_object is None:
                return

            if physx_property_query_object.status == PhysxPropertyQueryObject.Status.FAIL:
                return

            physx_property_query_object.status = PhysxPropertyQueryObject.Status.COMPLETE

        stage_id = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()

        get_physx_property_query_interface().query_prim(stage_id = stage_id, 
                                                        prim_id = body_path_physics,
                                                        query_mode = PhysxPropertyQueryMode.QUERY_RIGID_BODY_WITH_COLLIDERS,
                                                        finished_fn = on_property_query_finished,
                                                        rigid_body_fn = on_rigid_body_info_received,
                                                        collider_fn = on_collider_info_received,
                                                        timeout_ms = db.inputs.timeout)
        
        timer = 0.0
        timer_interval = 0.01
        while physx_property_query_object.status == PhysxPropertyQueryObject.Status.IN_PROGRESS:
            time.sleep(timer_interval)
            timer += timer_interval
            if timer > db.inputs.timeout:
                physx_property_query_object.status == PhysxPropertyQueryObject.Status.FAIL
                db.log_error("PhysX rigid body property query timeout.")
                break

        if PhysxPropertyQueryObject.Status.COMPLETE:
            db.outputs.centerOfMass = physx_property_query_object.body_center_of_mass
            db.outputs.mass = physx_property_query_object.body_mass
            db.outputs.diagonalInertia = physx_property_query_object.body_diagonal_inertia
            db.outputs.principalAxes = physx_property_query_object.body_principal_axes

            AABBMin = physx_property_query_object.body_bounds.GetMin()
            db.outputs.AABBMin = [*AABBMin]
            AABBMax = physx_property_query_object.body_bounds.GetMax()
            db.outputs.AABBMax = [*AABBMax]
            return True

        db.log_error("Physx rigid body property query failed to complete within the set timeout period.")
        return False
