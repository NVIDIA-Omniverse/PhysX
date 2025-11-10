# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

__all__ = ["CookingApproximationChecker"]

from omni.asset_validator.core import BaseRuleChecker, registerRule, Suggestion
from omni.asset_validator.core.complianceChecker import is_omni_path
from .. import get_physx_asset_validator_interface
from pxr import Usd, UsdPhysics, UsdUtils, PhysicsSchemaTools, UsdGeom
import carb
from omni.physx import get_physx_interface, get_physx_simulation_interface, get_physx_cooking_private_interface
import omni.physx.bindings._physx as physx_bindings
import asyncio

MARSHAL_TO_MAIN_THREAD = True  # Must run on main thread because it uses omni.physx runtime that is not thread safe


@registerRule("Omni:Physx")
class CookingApproximationChecker(BaseRuleChecker):
    def change_approximation_to_OOBB(self, stage: Usd.Stage, location: Usd.Prim) -> None:
        # We just need to change the approximation to OBB
        mesh_collision_api = UsdPhysics.MeshCollisionAPI.Get(stage, location.GetPath())
        mesh_collision_api.GetApproximationAttr().Set(UsdPhysics.Tokens.boundingCube)

    def CheckStage(self, stage: Usd.Stage):
        try:
            asyncio.get_event_loop()
            is_on_main_thread = True
        except Exception as e:
            is_on_main_thread = False
        if not is_on_main_thread and MARSHAL_TO_MAIN_THREAD:
            from omni.kit.async_engine import run_coroutine

            result = run_coroutine(self._async_check_stage(stage))
            import time

            try:
                while not result.done():
                    time.sleep(0.5)
            except Exception as e:
                print(e)
        else:
            self._internal_check_stage(stage)

    async def _async_check_stage(self, stage: Usd.Stage):
        self._internal_check_stage(stage)

    def _internal_check_stage(self, stage: Usd.Stage):
        if MARSHAL_TO_MAIN_THREAD:
            try:
                import asyncio

                asyncio.get_event_loop()
            except Exception:
                carb.log_error("CheckStage can only be run on main asyncio thread")
                return
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        if stage_id == -1:
            stage_id = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()

        settings = carb.settings.acquire_settings_interface()
        # NOTE: CheckStage may be invoked on an asyncio thread, so it's important avoiding writing to USD
        saved_update_to_usd = settings.get(physx_bindings.SETTING_UPDATE_TO_USD)
        saved_ujitso_collision_cooking = settings.get(physx_bindings.SETTING_UJITSO_COLLISION_COOKING)
        saved_use_local_mesh_cache = settings.get(physx_bindings.SETTING_USE_LOCAL_MESH_CACHE)
        settings.set(physx_bindings.SETTING_UPDATE_TO_USD, False)
        settings.set(physx_bindings.SETTING_UJITSO_COLLISION_COOKING, False)
        settings.set(physx_bindings.SETTING_USE_LOCAL_MESH_CACHE, False)

        get_physx_simulation_interface().attach_stage(stage_id)

        if stage.GetDefaultPrim():
            # Traverse the prims under the default prim
            traversal = iter(
                Usd.PrimRange(
                    stage.GetDefaultPrim(),
                    Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate),
                )
            )
        else:
            # No default prim, so traverse all prims
            traversal = iter(Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()))

        # TODO: Maybe on a large stage it could be useful trying usdrt
        for prim in traversal:
            if is_omni_path(prim.GetPath()):
                traversal.PruneChildren()
                continue
            if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                mesh_collision_api = UsdPhysics.MeshCollisionAPI.Get(stage, prim.GetPath())
                approximation = mesh_collision_api.GetApproximationAttr().Get()
                if (
                    approximation == UsdPhysics.Tokens.convexHull
                    or approximation == UsdPhysics.Tokens.convexDecomposition
                ):
                    prim_path = str(prim.GetPath())
                    prim_id = PhysicsSchemaTools.sdfPathToInt(prim.GetPath())
                    get_physx_cooking_private_interface().release_runtime_mesh_cache()
                    if not get_physx_asset_validator_interface().convex_gpu_compatibility_is_valid(stage_id, prim_id):
                        # carb.log_error(f'Convex approximation for "{prim_path}" is not GPU compatible (likely the object is too thin)')
                        self._AddFailedCheck(
                            message=f'Convex approximation for "{prim_path}" is not GPU compatible (likely the object is too thin)',
                            at=prim,
                            suggestion=Suggestion(
                                message="Change Approximation to Bounding Box",
                                callable=self.change_approximation_to_OOBB,
                                at=[stage.GetRootLayer()],
                            ),
                        )
                traversal.PruneChildren()  # Do not traverse childrens of articulation root
                continue

        get_physx_interface().reset_simulation()
        get_physx_simulation_interface().detach_stage()
        settings.set(physx_bindings.SETTING_UPDATE_TO_USD, saved_update_to_usd)
        settings.set(physx_bindings.SETTING_UJITSO_COLLISION_COOKING, saved_ujitso_collision_cooking)
        settings.set(physx_bindings.SETTING_USE_LOCAL_MESH_CACHE, saved_use_local_mesh_cache)


@registerRule("Omni:Physx")
class CookingApproximationFallbackChecker(BaseRuleChecker):
    def change_approximation_to_convex(self, stage: Usd.Stage, location: Usd.Prim) -> None:
        # We just need to change the approximation to OBB        
        mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(location)
        mesh_collision_api.GetApproximationAttr().Set(UsdPhysics.Tokens.convexHull)

    def is_part_of_dynamic_body(self, prim: Usd.Prim) -> bool:
        parent = prim
        while parent and not parent.IsPseudoRoot():
            if parent.HasAPI(UsdPhysics.RigidBodyAPI):
                rb_api = UsdPhysics.RigidBodyAPI(parent)
                enabled = rb_api.GetRigidBodyEnabledAttr().Get()
                
                if enabled:
                    kinematic = rb_api.GetKinematicEnabledAttr().Get()
                    if not kinematic:
                        return True  # Found a dynamic rigid body that is not kinematic
                
                break  # Found a rigid body but it's not enabled
            
            parent = parent.GetParent()
        
        return False  # No dynamic rigid body found in hierarchy

    def CheckPrim(self, prim: Usd.Prim):
        if not prim.IsA(UsdGeom.Mesh):
            return
        
        collision_api = UsdPhysics.CollisionAPI(prim)
        if collision_api:
            collision_mesh_api = UsdPhysics.MeshCollisionAPI(prim)
            
            in_dynamic_body = False
            if not collision_mesh_api:
                in_dynamic_body = self.is_part_of_dynamic_body(prim)
            else:
                approximation = collision_mesh_api.GetApproximationAttr().Get()
                if approximation == UsdPhysics.Tokens.none:
                    in_dynamic_body = self.is_part_of_dynamic_body(prim)

            if in_dynamic_body:
                self._AddFailedCheck(
                    message=f'Triangle mesh approximation for "{prim.GetPath()}" cant be used for dynamic bodies.',
                    at=prim,
                    suggestion=Suggestion(
                        message="Change Approximation to Convex",
                        callable=self.change_approximation_to_convex,
                        at=[prim.GetStage().GetRootLayer()],
                    ),
                )
