# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-UTILS-001
# @covers AC-1 AC-2 AC-5 AC-15

"""Physics materials and their binding to prims.

A physics material is a ``UsdShade.Material`` carrying
``UsdPhysics.MaterialAPI``, bound to a collider through the ``physics`` purpose
of ``UsdShade.MaterialBindingAPI`` so that it composes independently of any
render material on the same prim. Only the friction, restitution and density
values the caller supplies are authored; the rest keep their schema fallback.
"""

import logging
import typing

from pxr import Sdf, Usd, UsdPhysics, UsdShade

logger = logging.getLogger(__name__)

__all__ = [
    "add_physics_material_to_prim",
    "ensure_material_on_path",
    "add_rigid_body_material",
]


def add_physics_material_to_prim(
    stage: Usd.Stage,
    prim: Usd.Prim,
    material_path: typing.Union["str", Sdf.Path],
):
    """
    Bind physics material to a given prim.

    Args:
        stage:      The Usd.Stage to add path.
        prim:       The Usd.Prim where material should have the binding to.
        material_path:  The path of the material.
    """
    bindingAPI = UsdShade.MaterialBindingAPI.Apply(prim)
    materialPrim = UsdShade.Material(stage.GetPrimAtPath(material_path))
    bindingAPI.Bind(materialPrim, UsdShade.Tokens.weakerThanDescendants, "physics")


def ensure_material_on_path(stage: Usd.Stage, path: typing.Union[str, Sdf.Path]) -> bool:
    """
    Define a UsdShade.Material at path if nothing incompatible is already there.

    Args:
        stage:      The Usd.Stage to add the material.
        path:       The desired material path.
    """
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        if not prim.IsA(UsdShade.Material):
            logger.warning(f"AddMaterial: Prim at path {path} is already defined and not a Material")
            return False
        return True

    UsdShade.Material.Define(stage, path)
    return True


def add_rigid_body_material(
    stage: Usd.Stage,
    path: typing.Union[str, Sdf.Path],
    density=None,
    static_friction=None,
    dynamic_friction=None,
    restitution=None,
) -> bool:
    """
    Define a physics material, authoring only the attributes that were supplied.

    Args:
        stage:      The Usd.Stage to add the material.
        path:       The desired material path.
        density:    The material density.
        static_friction: The static friction coefficient.
        dynamic_friction: The dynamic friction coefficient.
        restitution:     The restitution coefficient.
    """
    if not ensure_material_on_path(stage, path):
        return False

    UsdShade.Material.Define(stage, path)
    material = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(path))

    if static_friction is not None:
        material.CreateStaticFrictionAttr().Set(static_friction)
    if dynamic_friction is not None:
        material.CreateDynamicFrictionAttr().Set(dynamic_friction)
    if restitution is not None:
        material.CreateRestitutionAttr().Set(restitution)
    if density is not None:
        material.CreateDensityAttr().Set(density)

    return True
