# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from .utils import *
from .components_data import *
from .properties import *
from .schemas import *
from .aliases import *
from .property_order import *
from .property_builders import *
from .custom_properties import *
from collections import namedtuple
from pxr import UsdPhysics, PhysxSchema, UsdGeom, UsdShade
from omni.kit.commands import execute
import omni.physx.bindings._physx as pxb
from dataclasses import dataclass
from omni.physx.scripts import particleUtils

# Extra items to add to the Add menu
ExtraAddItem = namedtuple("ExtraItem", "title, can_add, on_click, can_show")
ExtraAddItem.__new__.__defaults__ = (
    None,   # can_show
)

extra_add_items = [
    ExtraAddItem(
        "Rigid Body with Colliders Preset",
        lambda p: not p._refresh_cache.has_rigidbody_api,
        lambda p: execute("SetRigidBody", path=p.GetPath(), approximationShape=UsdPhysics.Tokens.convexHull, kinematic=False),
        lambda p: p.IsA(UsdGeom.Xformable) and not p.IsA(PhysxSchema.PhysxParticleSystem) and not p._refresh_cache.hasconflictingapis_RigidBodyAPI,
    ),
    ExtraAddItem(
        "Colliders Preset",
        lambda p: not p._refresh_cache.has_collision_api,
        lambda p: execute("SetStaticCollider", path=p.GetPath(), approximationShape=UsdPhysics.Tokens.none),
        lambda p: p.IsA(UsdGeom.Xformable) and not p.IsA(PhysxSchema.PhysxParticleSystem) and not p._refresh_cache.hasconflictingapis_CollisionAPI,
    ),
    ExtraAddItem(
        "Volume Deformable Body",
        lambda p: not p.HasAPI("OmniPhysicsDeformableBodyAPI"),
        lambda p: execute("SetVolumeDeformableBody", prim_path=p.GetPath()),
        lambda p: not deformable_deprecated_enabled and p.IsA(UsdGeom.TetMesh) and not pxb.hasconflictingapis_DeformableBodyAPI(p, True),
    ),
    ExtraAddItem(
        "Surface Deformable Body",
        lambda p: not p.HasAPI("OmniPhysicsDeformableBodyAPI"),
        lambda p: check_surface_deformable_application(p) and execute("SetSurfaceDeformableBody", prim_path=p.GetPath()),
        lambda p: not deformable_deprecated_enabled and p.IsA(UsdGeom.Mesh) and not pxb.hasconflictingapis_DeformableBodyAPI(p, True),
    ),
    ExtraAddItem(
        "Water Preset (PBD Material)",
        lambda p: p.IsA(UsdShade.Material),
        lambda p: particleUtils.AddPBDMaterialWater(p),
    ),
    ExtraAddItem(
        "Viscous Fluid Preset (PBD Material)",
        lambda p: p.IsA(UsdShade.Material),
        lambda p: particleUtils.AddPBDMaterialViscous(p),
    )
]

# Stage information
@dataclass()
class StageInfo():
    mpu: float
    kgpu: float
    up: str

    def __init__(self, stage):
        self.load(stage)

    def load(self, stage):
        self.kgpu = UsdPhysics.GetStageKilogramsPerUnit(stage)
        self.mpu = UsdGeom.GetStageMetersPerUnit(stage)
        self.up = UsdGeom.GetStageUpAxis(stage)
